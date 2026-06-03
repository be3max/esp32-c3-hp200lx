/*
 * ESP32-C3 BLE Keyboard -> UART Bridge for HP 200LX
 *
 * BLE HID host with OLED UI, 2-button navigation, and Programs menu.
 * GPIO9=BACK (up/cancel)  GPIO10=OK (select)  Both held 300ms=system menu
 * OLED: SSD1306 128x32 I2C on GPIO4/GPIO5
 * UART: Serial1 GPIO20/GPIO21 -> MAX3232 -> HP 200LX at 9600 8N1
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <string.h>
#include "driver/periph_ctrl.h"
#include <WiFi.h>
#include <WebServer.h>
#include "qrcode.h"

// ---------------------------------------------------------------------------
// Hardware configuration
// ---------------------------------------------------------------------------
#define UART_TX_PIN     21
#define UART_RX_PIN     20
#define UART_BAUD       9600

#define LED_PIN          8
#define LED_ACTIVE_LOW   1
#define BLINK_SCAN_MS  500
#define BLINK_CONN_MS  100

#define BTN_OK          10   // GPIO10 — select / confirm
#define BTN_BACK         9   // GPIO9  — back / up
#define BTN_DEBOUNCE_MS 50
#define BTN_BOTH_MS    300   // hold both this long for system menu

#define SCAN_DURATION_S 15
#define MAX_DEVICES     10

#define OLED_SDA_PIN     4
#define OLED_SCL_PIN     5
#define OLED_ADDR       0x3C
#define KEY_DISPLAY_MS  600

// Display layout: 128x32, text size 1 = 6x8 px, 4 rows of 21 chars
#define DISP_W  128
#define DISP_H   32
#define ROW0      0
#define ROW1      8
#define ROW2     16
#define ROW3     24
#define COLS     21

// XMODEM protocol
#define XMODEM_SOH   0x01
#define XMODEM_EOT   0x04
#define XMODEM_ACK   0x06
#define XMODEM_NAK   0x15
#define XMODEM_CAN   0x18
#define XMODEM_BLOCK 128

// HID-over-GATT UUIDs
static const uint16_t UUID_HID_SVC        = 0x1812;
static const uint16_t UUID_PROTOCOL_MODE  = 0x2A4E;
static const uint16_t UUID_BOOT_KBD_INPUT = 0x2A22;
static const uint16_t UUID_REPORT         = 0x2A4D;
static const uint16_t UUID_HID_CTRL_POINT = 0x2A4C;

static const uint8_t kModifierKeycode[8] = {
    0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7,
};

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
enum AppState {
    STATE_BOOT,
    STATE_SCANNING,
    STATE_DEVICE_LIST,
    STATE_CONNECTING,
    STATE_CONNECTED,
    STATE_ERROR,
    STATE_POPUP_MENU,
    STATE_PROGRAMS,
    STATE_DINO_GAME,
    STATE_DINO_GAMEOVER,
    STATE_CAR_GAME,
    STATE_CAR_GAMEOVER,
    STATE_RACING_LOGO,
    STATE_RACING_GAME,
    STATE_RACING_GAMEOVER,
    STATE_RACING_CRASH,
    STATE_KBDDRV_MENU,
    STATE_KBDDRV_CONFIRM,
    STATE_KBDDRV_SENDING,
    STATE_KBDDRV_INFO,
    STATE_SETTINGS,
};
static AppState g_state     = STATE_BOOT;
static AppState g_prevState = STATE_SCANNING;

static const char* kPopupItems[] = { "Scan for devices", "Restart", "Programs", "Settings", "Reset settings" };
static const int   kPopupCount   = 5;
static int         g_popupCursor = 0;

// ---------------------------------------------------------------------------
// BLE / application globals
// ---------------------------------------------------------------------------
static NimBLEClient*   g_client    = nullptr;
static NimBLEAddress   g_targetAddr;
static volatile bool   g_connected = false;
static uint8_t         g_prevReport[8] = {0};
static Preferences     g_prefs;
static String          g_connectedName;

// Button state
static bool     g_btnOkPrev    = true,  g_btnBackPrev  = true;
static uint32_t g_btnOkLast    = 0,     g_btnBackLast  = 0;
static uint32_t g_bothSince    = 0;
static bool     g_bothFired    = false;
static bool     g_connectArmed = false;
static int      g_savedFailCount = 0;
static int      g_scanRetryCount = 0;

// BLE scan results
static String        g_foundNames[MAX_DEVICES];
static NimBLEAddress g_foundAddrs[MAX_DEVICES];
static int           g_foundCount   = 0;
static volatile bool g_scanDone     = false;
static volatile bool g_savedDevFound = false;

// List navigation
static int g_listCursor = 0, g_listScroll = 0;
static int g_progCursor = 0, g_progScroll = 0;

// OLED
static Adafruit_SSD1306 g_display(DISP_W, DISP_H, &Wire, -1);
static bool             g_oledReady = false;

// Key event display
static uint32_t g_keyUntil     = 0;
static uint8_t  g_lastKeyEvent = 0;
static uint8_t  g_lastKeyCode  = 0;

// ---------------------------------------------------------------------------
// KBD driver transfer globals
// ---------------------------------------------------------------------------
static const char* KBDDRV_FILES[]  = { "/KBDSER2.COM", "/LOAD2.BAT", "/RESTORE.BAT" };
static const char* KBDDRV_LABELS[] = { "KBDSER2.COM",  "LOAD2.BAT",  "RESTORE.BAT"  };
static const char* KBDDRV_MENU_ITEMS[] = {
    "< Back",
    "Send KBDSER2.COM",
    "Send LOAD2.BAT",
    "Send RESTORE.BAT",
    "Information",
};
static const int KBDDRV_MENU_COUNT = 5;

static int    g_kdrvCursor = 0;
static int    g_kdrvScroll = 0;
static int    g_kdrvFile   = -1;
static size_t g_kdrvTotal  = 0;
static int    g_infoScroll = 0;
static bool   g_kdrvSending = false;  // true while in KBDDRV_SENDING state

enum XmodemState {
    XMDM_WAIT_NAK,
    XMDM_SEND_BLOCK,
    XMDM_WAIT_ACK,
    XMDM_SEND_EOT,
    XMDM_WAIT_EOT_ACK,
    XMDM_DONE,
    XMDM_ERROR,
};
static XmodemState g_xState      = XMDM_WAIT_NAK;
static uint8_t     g_xBlockNum   = 1;
static uint8_t     g_xRetry      = 0;
static uint32_t    g_xTimeout    = 0;
static File        g_xFile;
static uint8_t     g_xBuf[XMODEM_BLOCK];
static int         g_xTotalBlocks = 0;
static int         g_xSentBlocks  = 0;

// ---------------------------------------------------------------------------
// WiFi Settings globals
// ---------------------------------------------------------------------------
static WebServer* g_webServer          = nullptr;
static bool       g_wifiIsAP           = false;
static String     g_wifiIP;
static uint32_t   g_settingsLastRender = 0;
static bool       g_settingsSaved      = false;
static QRCode     g_settingsQR;
static uint8_t    g_settingsQRBuf[80];   // qrcode_getBufferSize(2) = 80 bytes
static bool       g_settingsQRReady    = false;
static String     g_settingsQRUrl;
static uint32_t   g_wifiConnectStart  = 0;
static bool       g_wifiConnectFailed = false;
static String     g_apPass;

static const char* const INFO_LINES[] = {
    "=== KBDSER2.COM ===",
    "TSR: receives ASCII",
    "from COM1 9600 8N1,",
    "injects to BIOS buf.",
    "~1KB resident. Flags:",
    "/G=game /D=dbg /U=off",
    "",
    "=== LOAD2.BAT ===",
    "Unloads+reloads drv.",
    "Run from A: drive.",
    "/G game mode",
    "/G /D with counter",
    "",
    "=== RESTORE.BAT ===",
    "Rewrites CONFIG.SYS",
    "& AUTOEXEC.BAT.",
    "Auto-loads on boot.",
    "Reboots HP 200LX.",
    "",
    "Install to A: drive",
    "(flash, survives",
    "battery removal).",
    "COM1 9600 8N1.",
    "",
    "=== TRANSFER ===",
    "Uses XMODEM protocol",
    "HP 200LX: DataComm",
    "Receive via XMODEM.",
    "9600 8N1, COM1.",
};
static const int INFO_LINE_COUNT = 29;

// ---------------------------------------------------------------------------
// Dino game
// ---------------------------------------------------------------------------
#define DINO_X        8
#define DINO_W        8
#define DINO_H       10
#define GROUND_Y     27
#define DINO_REST_Y  (GROUND_Y - DINO_H)
#define DINO_GRAVITY  0.5f
#define DINO_JUMP_VY (-4.0f)
#define DINO_TICK_MS  40

struct DinoObs { float x; uint8_t w, h; bool active; };

static float    g_dinoY, g_dinoVY;
static bool     g_dinoOnGround;
static uint8_t  g_dinoLeg;
static uint32_t g_dinoLegTimer;
static bool     g_dinoJumpReq;
static int      g_dinoScore;
static float    g_dinoSpeed;
static uint32_t g_dinoTickLast;
static bool     g_dinoOver;
static DinoObs  g_obs[3];

// ---------------------------------------------------------------------------
// 3D Racing game
// ---------------------------------------------------------------------------
#define RACING_TICK_MS      40
#define RACING_HORIZON_Y    8
#define RACING_BOTTOM_Y     31
#define RACING_VP_X         64
#define RACING_ROAD_BOT     56   // road half-width at bottom (px)
#define RACING_MAX_OPP      4
#define RACING_Z_FAR        1000
#define RACING_PLAYER_Y     25

struct RacingOpp {
    int  z;        // z-units ahead of player (1000=horizon, 0=player level)
    int  speedQ4;  // forward speed in z-units/tick * 4
    int  laneQ8;   // lane position: -256=left, 0=center, +256=right
    bool active;
    bool scored;
};

struct RacingSide {
    int     z;
    int8_t  side;   // -1=left, +1=right
    uint8_t type;   // 0=grass tuft, 1=bush, 2=tree
    bool    active;
};

static RacingOpp g_racingOpp[RACING_MAX_OPP];
static int       g_racingPlayerLane;   // -1, 0, +1 (committed lane)
static int       g_racingTargetLane;   // -1, 0, +1 (input target)
static int       g_racingSlide;        // Q8: -256..+256 (visual interp)
static int       g_racingSpeedQ4;      // player speed * 4 (z-units/tick)
static int       g_racingAccTick;      // accel counter
static uint16_t  g_racingScore;
static uint16_t  g_racingHiScore;
static uint32_t  g_racingLastMs;
static uint8_t   g_racingInputQ;       // bit0=left, bit1=right (queued edge)
static uint8_t   g_racingFlash;        // frames of flash remaining
static bool      g_racingOver;
static uint8_t   g_racingCrashFrame;
static int8_t    g_racingCrashDir;
static int       g_racingCrashCX;
static int16_t   g_expPX[12], g_expPY[12];  // debris positions Q4
static int8_t    g_expVX[12], g_expVY[12];  // debris velocities Q4
static uint8_t   g_racingSpinTicks;
static float     g_racingSpinAngle;
static int8_t    g_racingSpinDir;
static uint16_t  g_racingLogoFrame;
static int       g_racingScroll;       // centerline phase (0..Z_FAR-1)
static int       g_racingCurveOff;    // current horizon X offset (curve)
static int       g_racingCurveTarget; // target curve offset
static int       g_racingCurveTicks;  // countdown to next curve change
static int       g_racingSideTimer;   // countdown to next roadside spawn
static RacingSide g_racingSide[8];

// ---------------------------------------------------------------------------
// Car physics game — constants and globals
// ---------------------------------------------------------------------------
#define CAR_TICK_MS        40
#define CAR_TBUF_SIZE      512    // ring buffer (512 bytes RAM)
#define CAR_SEG_W          2      // px per terrain segment on screen
#define CAR_SCREEN_X       22     // car fixed x on screen
#define CAR_GRAVITY        0.45f
#define CAR_THROTTLE_TORQ  0.009f // pitch torque per unit wheelSpin (tuned: ~2s to wheelie)
#define CAR_WHEEL_ACCEL    0.09f
#define CAR_WHEEL_MAX      3.5f
#define CAR_WHEEL_MAX_REV  1.8f
#define CAR_PITCH_RESTORE  0.022f // ground restoring (eq pitch ~40deg at full throttle)
#define CAR_PITCH_DAMP_GND 0.60f
#define CAR_PITCH_DAMP_AIR 0.997f
#define CAR_FLIP_ANGLE     1.65f  // crash pitch ~95 deg
#define CAR_BODY_H         7      // car center to wheel-bottom px
#define CAR_HALFWB         6      // half-wheelbase px
#define CAR_WHEEL_R        3
#define CAR_EXPL_FRAMES    55
#define CAR_LOOKAHEAD      80
#define CAR_GENBLOCK       64

static uint8_t  g_carTBuf[CAR_TBUF_SIZE];
static int32_t  g_terrainHead;  // global seg index of next segment to generate
static float    g_terrainH;
static float    g_terrainSlope;

static float    g_carSeg;
static float    g_carY;
static float    g_carVx;
static float    g_carVy;
static float    g_carPitch;    // body pitch: 0=upright, +ve=nose-up
static float    g_pitchRate;
static float    g_wheelSpin;
static bool     g_carOnGround;
static uint32_t g_carDist;
static bool     g_carOver;
static uint32_t g_carTickLast;

struct CarPart { float x, y, vx, vy; };
static CarPart  g_explParts[8];
static uint8_t  g_explFrame;   // 0=idle, 1..CAR_EXPL_FRAMES=animating

// ---------------------------------------------------------------------------
// Button helpers
// ---------------------------------------------------------------------------
static bool btnPressed(int pin, bool& prev, uint32_t& last, bool inhibit) {
    bool cur  = (digitalRead(pin) == LOW);
    bool edge = !inhibit && cur && !prev && (millis() - last > BTN_DEBOUNCE_MS);
    prev = cur;
    if (edge) last = millis();
    return edge;
}

static bool bothPressed() {
    bool both = (digitalRead(BTN_OK) == LOW) && (digitalRead(BTN_BACK) == LOW);
    if (both) {
        if (!g_bothSince) g_bothSince = millis();
        if (!g_bothFired && millis() - g_bothSince >= BTN_BOTH_MS) {
            g_bothFired = true;
            return true;
        }
    } else {
        g_bothSince = 0;
        g_bothFired = false;
    }
    return false;
}

// ---------------------------------------------------------------------------
// I2C peripheral reset — NimBLE init wedges I2C0 FSM (arduino-esp32 #8454)
// ---------------------------------------------------------------------------
static void resetI2CPeripheral() {
    Wire.end();
    periph_module_disable(PERIPH_I2C0_MODULE);
    delay(10);
    periph_module_enable(PERIPH_I2C0_MODULE);
    delay(10);
    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    Wire.setClock(100000);
}

// ---------------------------------------------------------------------------
// Display primitives
// ---------------------------------------------------------------------------
static void dispClear() {
    g_display.clearDisplay();
    g_display.setTextColor(SSD1306_WHITE);
    g_display.setTextSize(1);
}

static String trunc(const String& s, int n = COLS) {
    return s.length() <= (unsigned)n ? s : s.substring(0, n);
}

// Full-width inverted header bar on ROW0 with centered text
static void drawHeader(const char* text) {
    g_display.fillRect(0, ROW0, DISP_W, 8, SSD1306_WHITE);
    int x = max(0, (DISP_W - (int)strlen(text) * 6) / 2);
    g_display.setTextColor(SSD1306_BLACK);
    g_display.setCursor(x, ROW0);
    g_display.print(text);
    g_display.setTextColor(SSD1306_WHITE);
}

// List row: inverted (white bg, black text) when selected
static void drawRow(int y, const String& label, bool selected) {
    if (selected) {
        g_display.fillRect(0, y, DISP_W, 8, SSD1306_WHITE);
        g_display.setTextColor(SSD1306_BLACK);
        g_display.setCursor(1, y);
        g_display.print(trunc(label, COLS - 1));
        g_display.setTextColor(SSD1306_WHITE);
    } else {
        g_display.setCursor(1, y);
        g_display.print(trunc(label, COLS - 1));
    }
}

// Scroll indicator arrows at right edge of content area
static void drawScrollIndicators(int scroll, int total, int visible = 3) {
    if (scroll > 0)                     { g_display.setCursor(122, ROW1); g_display.print("^"); }
    if (scroll + visible < total)       { g_display.setCursor(122, ROW3); g_display.print("v"); }
}

// ---------------------------------------------------------------------------
// Render — Scanning
// ---------------------------------------------------------------------------
static void renderScanning() {
    if (!g_oledReady) return;
    static uint32_t lastUpdate = 0;
    static uint8_t  spinIdx    = 0;
    static const char kSpin[]  = "|/-\\";
    if (millis() - lastUpdate < 150) return;
    lastUpdate = millis();
    spinIdx = (spinIdx + 1) & 3;

    dispClear();
    drawHeader("BLE SCAN");
    if (g_targetAddr != NimBLEAddress()) {
        g_display.setCursor(0, ROW1); g_display.printf("Seeking... %c", kSpin[spinIdx]);
        g_display.setCursor(0, ROW2); g_display.print(trunc(g_connectedName));
    } else {
        g_display.setCursor(0, ROW1); g_display.printf("Searching... %c", kSpin[spinIdx]);
        if (g_foundCount > 0) {
            g_display.setCursor(0, ROW2);
            g_display.printf("Found: %d device%s", g_foundCount, g_foundCount == 1 ? "" : "s");
        }
    }
    g_display.display();
}

// ---------------------------------------------------------------------------
// Render — Device list
// ---------------------------------------------------------------------------
static void clampListScroll() {
    int total = g_foundCount + 1;
    if (g_listCursor < g_listScroll)      g_listScroll = g_listCursor;
    if (g_listCursor >= g_listScroll + 3) g_listScroll = g_listCursor - 2;
    if (g_listScroll < 0)                  g_listScroll = 0;
    if (g_listScroll + 3 > total)         g_listScroll = max(0, total - 3);
}

static void renderDeviceList() {
    if (!g_oledReady) return;
    static uint32_t lastUpdate = 0;
    if (millis() - lastUpdate < 80) return;
    lastUpdate = millis();

    int total = g_foundCount + 1;
    dispClear();

    char hdr[22];
    snprintf(hdr, sizeof(hdr), g_foundCount ? "SELECT (%d)" : "NO DEVICES", g_foundCount);
    drawHeader(hdr);

    static const int rows[] = { ROW1, ROW2, ROW3 };
    for (int row = 0; row < 3; row++) {
        int idx = g_listScroll + row;
        if (idx >= total) break;
        String label = (idx == g_foundCount) ? "<Rescan>" : trunc(g_foundNames[idx]);
        drawRow(rows[row], label, idx == g_listCursor);
    }
    drawScrollIndicators(g_listScroll, total);
    g_display.display();
}

// ---------------------------------------------------------------------------
// Render — Connecting
// ---------------------------------------------------------------------------
static void renderConnecting() {
    if (!g_oledReady) return;
    static uint32_t lastUpdate = 0, lastAnim = 0;
    static uint8_t  dotPhase   = 0;
    if (millis() - lastUpdate < 80) return;
    lastUpdate = millis();
    if (millis() - lastAnim >= 350) { lastAnim = millis(); dotPhase = (dotPhase + 1) % 4; }
    static const char* kDots[] = { "", ".", "..", "..." };

    dispClear();
    drawHeader("CONNECTING");
    g_display.setCursor(0, ROW1); g_display.print(trunc(g_connectedName));
    g_display.setCursor(0, ROW2); g_display.printf("Please wait%s", kDots[dotPhase]);
    g_display.display();
}

// ---------------------------------------------------------------------------
// Render — Connected
// ---------------------------------------------------------------------------
static void renderConnected() {
    if (!g_oledReady) return;
    static uint32_t lastUpdate = 0;
    if (millis() - lastUpdate < 80) return;
    lastUpdate = millis();

    dispClear();
    if (millis() < g_keyUntil) {
        g_display.setTextSize(2);
        g_display.setCursor(0, 0);
        g_display.print(g_lastKeyEvent == 0x01 ? "KEY DOWN" : "KEY UP  ");
        g_display.setTextSize(1);
        g_display.setCursor(0, ROW3);
        g_display.printf("HID 0x%02X", g_lastKeyCode);
    } else {
        drawHeader("CONNECTED");
        g_display.setCursor(0, ROW1); g_display.print(trunc(g_connectedName));
        g_display.setCursor(0, ROW3); g_display.print("BACK=scan  BOTH=menu");
    }
    g_display.display();
}

// ---------------------------------------------------------------------------
// Render — Error
// ---------------------------------------------------------------------------
static void renderError() {
    if (!g_oledReady) return;
    dispClear();
    drawHeader("CONNECT FAILED");
    g_display.setCursor(0, ROW1); g_display.print(trunc(g_connectedName));
    g_display.setCursor(0, ROW2); g_display.print("Try pairing mode");
    g_display.setCursor(0, ROW3); g_display.print("OK=scan   BOTH=menu");
    g_display.display();
}

// ---------------------------------------------------------------------------
// Render — System popup menu
// ---------------------------------------------------------------------------
static void renderPopupMenu() {
    if (!g_oledReady) return;
    static uint32_t lastUpdate = 0;
    if (millis() - lastUpdate < 80) return;
    lastUpdate = millis();

    int scroll = max(0, min(g_popupCursor - 2, kPopupCount - 3));
    dispClear();
    drawHeader("== MENU ==");
    static const int rows[] = { ROW1, ROW2, ROW3 };
    for (int i = 0; i < 3 && (scroll + i) < kPopupCount; i++)
        drawRow(rows[i], kPopupItems[scroll + i], (scroll + i) == g_popupCursor);
    drawScrollIndicators(scroll, kPopupCount);
    g_display.display();
}

// ---------------------------------------------------------------------------
// Dino game — drawing
// ---------------------------------------------------------------------------
static void drawDino(int x, int y, int frame) {
    g_display.fillRect(x+3, y,   5, 3, SSD1306_WHITE);
    g_display.drawPixel(x+5, y+1, SSD1306_BLACK);
    g_display.fillRect(x,   y+2, 8, 5, SSD1306_WHITE);
    if (frame == 0) {
        g_display.fillRect(x+1, y+7, 2, 3, SSD1306_WHITE);
        g_display.fillRect(x+5, y+7, 2, 2, SSD1306_WHITE);
    } else {
        g_display.fillRect(x+5, y+7, 2, 3, SSD1306_WHITE);
        g_display.fillRect(x+1, y+7, 2, 2, SSD1306_WHITE);
    }
}

static void drawCactus(int x, int h) {
    g_display.fillRect(x+1, GROUND_Y-h,      3, h, SSD1306_WHITE);
    g_display.fillRect(x,   GROUND_Y-h*2/3,  5, 2, SSD1306_WHITE);
}

static void renderDinoGame() {
    if (!g_oledReady) return;
    g_display.clearDisplay();
    g_display.setTextColor(SSD1306_WHITE);
    g_display.setTextSize(1);
    g_display.setCursor(DISP_W - 6*5, ROW0);
    g_display.printf("%05d", g_dinoScore);
    g_display.drawFastHLine(0, GROUND_Y, DISP_W, SSD1306_WHITE);
    drawDino(DINO_X, (int)g_dinoY, g_dinoOnGround ? g_dinoLeg : 0);
    for (auto& o : g_obs) {
        if (!o.active) continue;
        int ox = (int)o.x;
        if (ox >= -5 && ox < DISP_W) drawCactus(ox, o.h);
    }
    g_display.display();
}

static void renderDinoGameOver() {
    if (!g_oledReady) return;
    dispClear();
    drawHeader("GAME OVER");
    g_display.setCursor(0, ROW1); g_display.printf("Score: %d", g_dinoScore);
    g_display.setCursor(0, ROW3); g_display.print("OK=retry  BACK=exit");
    g_display.display();
}

// ---------------------------------------------------------------------------
// KBD Driver — render functions
// ---------------------------------------------------------------------------
static void renderKbdDrvMenu() {
    if (!g_oledReady) return;
    static uint32_t lastUp = 0;
    if (millis() - lastUp < 80) return;
    lastUp = millis();

    dispClear();
    drawHeader("KBD DRIVER");
    static const int rows[] = { ROW1, ROW2, ROW3 };
    for (int row = 0; row < 3; row++) {
        int idx = g_kdrvScroll + row;
        if (idx >= KBDDRV_MENU_COUNT) break;
        drawRow(rows[row], KBDDRV_MENU_ITEMS[idx], idx == g_kdrvCursor);
    }
    drawScrollIndicators(g_kdrvScroll, KBDDRV_MENU_COUNT);
    g_display.display();
}

static void renderKbdDrvConfirm() {
    if (!g_oledReady) return;
    static uint32_t lastUp = 0;
    if (millis() - lastUp < 80) return;
    lastUp = millis();

    dispClear();
    drawHeader(KBDDRV_LABELS[g_kdrvFile]);
    char szBuf[22];
    snprintf(szBuf, sizeof(szBuf), "%u bytes", (unsigned)g_kdrvTotal);
    g_display.setCursor(0, ROW1); g_display.print(szBuf);
    g_display.setCursor(0, ROW2); g_display.print("OK=send  BACK=cancel");
    g_display.setCursor(0, ROW3); g_display.print("Start DataComm rcv!");
    g_display.display();
}

static void renderKbdDrvSending() {
    if (!g_oledReady) return;
    static uint32_t lastUp = 0;
    if (millis() - lastUp < 100) return;
    lastUp = millis();

    dispClear();
    drawHeader(KBDDRV_LABELS[g_kdrvFile]);

    // Progress bar with border, fill proportional to blocks sent
    g_display.drawRect(0, ROW1, DISP_W, 8, SSD1306_WHITE);
    if (g_xTotalBlocks > 0) {
        int filled = (int)((int64_t)g_xSentBlocks * 126 / g_xTotalBlocks);
        if (filled > 0) g_display.fillRect(1, ROW1 + 1, filled, 6, SSD1306_WHITE);
    }

    char buf[22];
    snprintf(buf, sizeof(buf), "blk %d/%d", g_xSentBlocks, g_xTotalBlocks);
    g_display.setCursor(0, ROW2); g_display.print(buf);
    g_display.setCursor(0, ROW3); g_display.print("BACK=stop");
    g_display.display();
}

static void renderKbdDrvInfo() {
    if (!g_oledReady) return;
    static uint32_t lastUp = 0;
    if (millis() - lastUp < 80) return;
    lastUp = millis();

    dispClear();
    drawHeader("INFORMATION");
    static const int rows[] = { ROW1, ROW2, ROW3 };
    for (int r = 0; r < 3; r++) {
        int idx = g_infoScroll + r;
        if (idx >= INFO_LINE_COUNT) break;
        g_display.setCursor(0, rows[r]);
        g_display.print(trunc(String(INFO_LINES[idx])));
    }
    if (g_infoScroll > 0)                         { g_display.setCursor(122, ROW1); g_display.print("^"); }
    if (g_infoScroll + 3 < INFO_LINE_COUNT)        { g_display.setCursor(122, ROW3); g_display.print("v"); }
    g_display.display();
}

// ---------------------------------------------------------------------------
// XMODEM — non-blocking state machine, called each loop while sending
// ---------------------------------------------------------------------------
static void handleXmodem() {
    if (g_xState == XMDM_DONE || g_xState == XMDM_ERROR) return;

    if (millis() > g_xTimeout) {
        Serial.println("[xmodem] timeout");
        g_xFile.close();
        g_xState = XMDM_ERROR;
        return;
    }

    switch (g_xState) {

    case XMDM_WAIT_NAK:
        if (Serial1.available()) {
            uint8_t b = Serial1.read();
            Serial.printf("[xmodem] start byte 0x%02X\n", b);
            if (b == XMODEM_NAK || b == 'C') {
                g_xState   = XMDM_SEND_BLOCK;
                g_xTimeout = millis() + 10000;
            }
        }
        break;

    case XMDM_SEND_BLOCK: {
        memset(g_xBuf, 0x1A, XMODEM_BLOCK);
        g_xFile.read(g_xBuf, XMODEM_BLOCK);

        uint8_t cksum = 0;
        for (int i = 0; i < XMODEM_BLOCK; i++) cksum += g_xBuf[i];

        uint8_t hdr[3] = { XMODEM_SOH, g_xBlockNum, (uint8_t)(255u - g_xBlockNum) };
        Serial1.write(hdr, 3);
        Serial1.write(g_xBuf, XMODEM_BLOCK);
        Serial1.write(&cksum, 1);
        Serial.printf("[xmodem] sent block %d\n", g_xBlockNum);

        g_xState   = XMDM_WAIT_ACK;
        g_xTimeout = millis() + 10000;
        break;
    }

    case XMDM_WAIT_ACK:
        if (Serial1.available()) {
            uint8_t b = Serial1.read();
            Serial.printf("[xmodem] ack byte 0x%02X\n", b);
            if (b == XMODEM_ACK) {
                g_xBlockNum++;
                g_xSentBlocks++;
                g_xRetry   = 0;
                g_xTimeout = millis() + 10000;
                if ((size_t)g_xSentBlocks * XMODEM_BLOCK >= g_kdrvTotal) {
                    g_xState = XMDM_SEND_EOT;
                } else {
                    g_xState = XMDM_SEND_BLOCK;
                }
            } else if (b == XMODEM_NAK) {
                g_xRetry++;
                Serial.printf("[xmodem] NAK retry %d\n", g_xRetry);
                if (g_xRetry >= 10) {
                    g_xFile.close();
                    g_xState = XMDM_ERROR;
                } else {
                    // Seek back to retry current block
                    g_xFile.seek((size_t)g_xSentBlocks * XMODEM_BLOCK);
                    g_xState   = XMDM_SEND_BLOCK;
                    g_xTimeout = millis() + 10000;
                }
            } else if (b == XMODEM_CAN) {
                Serial.println("[xmodem] CAN received");
                g_xFile.close();
                g_xState = XMDM_ERROR;
            }
        }
        break;

    case XMDM_SEND_EOT: {
        uint8_t eot = XMODEM_EOT;
        Serial1.write(&eot, 1);
        Serial.println("[xmodem] sent EOT");
        g_xState   = XMDM_WAIT_EOT_ACK;
        g_xTimeout = millis() + 10000;
        break;
    }

    case XMDM_WAIT_EOT_ACK:
        if (Serial1.available()) {
            uint8_t b = Serial1.read();
            if (b == XMODEM_ACK) {
                g_xFile.close();
                g_xState = XMDM_DONE;
                Serial.println("[xmodem] done");
            } else if (b == XMODEM_NAK) {
                // Some receivers NAK the EOT once; resend
                uint8_t eot = XMODEM_EOT;
                Serial1.write(&eot, 1);
                g_xTimeout = millis() + 10000;
            }
        }
        break;

    default: break;
    }
}

// ---------------------------------------------------------------------------
// Dino game — logic
// ---------------------------------------------------------------------------
static void dinoSpawn() {
    for (auto& o : g_obs) {
        if (!o.active) {
            o.x = 132.0f + (float)random(10, 50);
            o.w = 5;
            o.h = 8 + random(0, 7);
            o.active = true;
            return;
        }
    }
}

static void initDinoGame() {
    g_dinoY        = DINO_REST_Y;
    g_dinoVY       = 0;
    g_dinoOnGround = true;
    g_dinoLeg      = 0;
    g_dinoLegTimer = 0;
    g_dinoJumpReq  = false;
    g_dinoScore    = 0;
    g_dinoSpeed    = 2.0f;
    g_dinoTickLast = millis();
    g_dinoOver     = false;
    for (auto& o : g_obs) o.active = false;
    g_obs[0] = { 132.0f, 5, 10, true };
}

static void updateDino() {
    if (g_dinoJumpReq && g_dinoOnGround) {
        g_dinoVY = DINO_JUMP_VY;
        g_dinoOnGround = false;
    }
    g_dinoJumpReq = false;

    g_dinoVY += DINO_GRAVITY;
    g_dinoY  += g_dinoVY;
    if (g_dinoY >= DINO_REST_Y) {
        g_dinoY = DINO_REST_Y;
        g_dinoVY = 0;
        g_dinoOnGround = true;
    }

    if (g_dinoOnGround && millis() - g_dinoLegTimer > 120) {
        g_dinoLeg     ^= 1;
        g_dinoLegTimer = millis();
    }

    int rightmostX = -999;
    for (auto& o : g_obs) {
        if (!o.active) continue;
        o.x -= g_dinoSpeed;
        if ((int)o.x > rightmostX) rightmostX = (int)o.x;
        if (o.x + o.w < 0) { o.active = false; g_dinoScore++; }
    }
    if (rightmostX < 70) dinoSpawn();

    g_dinoSpeed = min(5.0f, 2.0f + g_dinoScore * 0.12f);

    int dy = (int)g_dinoY;
    for (auto& o : g_obs) {
        if (!o.active) continue;
        int ox = (int)o.x;
        int oy = GROUND_Y - o.h;
        if (DINO_X + DINO_W - 1 > ox + 1 &&
            DINO_X + 1         < ox + o.w - 1 &&
            dy + DINO_H - 1    > oy + 2 &&
            dy + 2             < GROUND_Y) {
            g_dinoOver = true;
        }
    }
}

// ---------------------------------------------------------------------------
// Car physics game — functions
// ---------------------------------------------------------------------------

// Ring-buffer terrain lookup with linear interpolation
static int getCarTerrainY(float seg) {
    int32_t i = (int32_t)seg;
    if (g_terrainHead < 2) return 22;
    if (i < 0) i = 0;
    if (i >= g_terrainHead - 1) i = g_terrainHead - 2;
    float f = seg - (float)i;
    return (int)((float)g_carTBuf[i % CAR_TBUF_SIZE] * (1.0f - f) +
                 (float)g_carTBuf[(i + 1) % CAR_TBUF_SIZE] * f + 0.5f);
}

// Centered-difference slope at seg position (px/px)
static float getTerrainSlope(float seg) {
    int32_t i = (int32_t)seg;
    if (i < 1 || i >= g_terrainHead - 1) return 0.0f;
    return ((float)g_carTBuf[(i + 1) % CAR_TBUF_SIZE] -
            (float)g_carTBuf[(i - 1) % CAR_TBUF_SIZE]) / (2.0f * CAR_SEG_W);
}

// Generate terrain segments [from, to). Difficulty ramps with distance.
static void carGenTerrain(int32_t from, int32_t to) {
    float diff       = min(1.0f, (float)from / 500.0f); // 0→1 over 500 segs
    float slopeMax   = 1.3f + diff * 2.8f;              // 1.3 → 4.1 px/seg
    float wander     = 0.30f + diff * 0.28f;            // slope randomness
    float jumpChance = diff * 0.004f;                   // ramp probability/seg

    for (int32_t i = from; i < to; i++) {
        // Random slope walk with momentum
        g_terrainSlope += ((float)random(-100, 101) / 100.0f) * wander;
        g_terrainSlope  = constrain(g_terrainSlope, -slopeMax, slopeMax);
        g_terrainH     += g_terrainSlope;

        // Bounce off vertical limits → forces sharp direction reversal
        if (g_terrainH < 10.0f) { g_terrainH = 10.0f; g_terrainSlope =  fabsf(g_terrainSlope); }
        if (g_terrainH > 26.0f) { g_terrainH = 26.0f; g_terrainSlope = -fabsf(g_terrainSlope); }

        // Inject sharp ramp features at higher difficulties
        if ((float)random(1000) / 1000.0f < jumpChance)
            g_terrainSlope = slopeMax * (random(2) ? 1.0f : -1.0f);

        g_carTBuf[i % CAR_TBUF_SIZE] = (uint8_t)constrain(g_terrainH, 10.0f, 26.0f);
    }
    g_terrainHead = to;
}

static void initCarGame() {
    // Flat start
    g_terrainH     = 22.0f;
    g_terrainSlope = 0.0f;
    g_terrainHead  = 0;
    for (int i = 0; i < 28; i++) g_carTBuf[i] = 22;
    g_terrainHead = 28;
    carGenTerrain(28, 28 + CAR_TBUF_SIZE - 1); // fill ring buffer

    g_carSeg      = 10.0f;
    g_carY        = (float)getCarTerrainY(10.0f) - (float)CAR_BODY_H;
    g_carVx       = 0.0f;
    g_carVy       = 0.0f;
    g_carPitch    = 0.0f;
    g_pitchRate   = 0.0f;
    g_wheelSpin   = 0.0f;
    g_carOnGround = true;
    g_carDist     = 0;
    g_carOver     = false;
    g_explFrame   = 0;
    g_carTickLast = millis();
}

static void updateCarGame() {
    bool fwd = (digitalRead(BTN_OK)   == LOW) && !g_bothSince;
    bool rev = (digitalRead(BTN_BACK) == LOW) && !g_bothSince;

    // ---- Wheel spin ----
    if (fwd)      g_wheelSpin += CAR_WHEEL_ACCEL;
    else if (rev) g_wheelSpin -= CAR_WHEEL_ACCEL * 0.65f;
    else          g_wheelSpin *= (g_carOnGround ? 0.84f : 0.92f);
    g_wheelSpin = constrain(g_wheelSpin, -CAR_WHEEL_MAX_REV, CAR_WHEEL_MAX);

    // ---- Horizontal movement FIRST (so slope is computed at new position) ----
    if (g_carOnGround) {
        float grip = cosf(g_carPitch);
        if (grip < 0.05f) grip = 0.05f;
        g_carVx = g_wheelSpin * grip;
    } else {
        g_carVx *= 0.996f;
    }
    g_carSeg += g_carVx;
    if (g_carSeg < 2.0f) {
        g_carSeg    = 2.0f;
        g_carVx     = 0.0f;
        g_wheelSpin *= -0.25f;
        g_pitchRate *= -0.3f;
    }

    // ---- Slope at NEW position (centered diff = smoother) ----
    float terrSlope  = getTerrainSlope(g_carSeg);
    float targetPitch = atanf(terrSlope);

    // ---- Angular dynamics ----
    if (g_carOnGround) {
        float throttleTorq = g_wheelSpin * CAR_THROTTLE_TORQ;
        float restoreTorq  = (targetPitch - g_carPitch) * CAR_PITCH_RESTORE;
        g_pitchRate += throttleTorq + restoreTorq;
        g_pitchRate *= CAR_PITCH_DAMP_GND;
    } else {
        g_pitchRate -= g_carPitch * 0.004f;
        g_pitchRate *= CAR_PITCH_DAMP_AIR;
    }
    g_carPitch += g_pitchRate;

    // ---- Vertical physics ----
    // KEY: when grounded, g_carVy = slope velocity set at END of previous tick.
    // We carry it over (no reset here) so the car launches naturally off peaks.
    // Gravity only applies when airborne.
    if (!g_carOnGround) g_carVy += CAR_GRAVITY;
    g_carY += g_carVy;

    // ---- Ground detection ----
    float groundY = (float)getCarTerrainY(g_carSeg);
    float carBot  = g_carY + (float)CAR_BODY_H;

    if (carBot >= groundY) {
        bool wasAir = !g_carOnGround;
        g_carOnGround = true;

        if (wasAir && fabsf(g_carPitch) > 1.4f && g_carVy > 0.8f)
            g_carOver = true; // landed inverted

        g_carY = groundY - (float)CAR_BODY_H;

        if (wasAir && g_carVy > 1.5f) {
            g_carVy    *= -0.15f; // small bounce on hard landing
            g_pitchRate *= 0.4f;
        } else {
            // Set slope-following velocity for NEXT tick — this is what launches
            // the car off a ramp: next tick if terrain drops, g_carVy is upward.
            g_carVy = g_carVx * (float)CAR_SEG_W * terrSlope;
        }
    } else {
        g_carOnGround = false;
        // g_carVy carries over: upward ramp velocity → natural airborne launch
    }

    // ---- Distance + crashes ----
    if (g_carVx > 0.1f) g_carDist++;
    if (fabsf(g_carPitch) > CAR_FLIP_ANGLE) g_carOver = true;
    if (g_carY > (float)(DISP_H + 4))       g_carOver = true;

    // ---- Terrain lookahead ----
    if ((int32_t)g_carSeg + CAR_LOOKAHEAD > g_terrainHead)
        carGenTerrain(g_terrainHead, g_terrainHead + CAR_GENBLOCK);
}

// Shared: draw terrain background for current car position
static void drawCarTerrain() {
    float startSeg = g_carSeg - (float)CAR_SCREEN_X / (float)CAR_SEG_W;
    for (int x = 0; x < DISP_W; x++) {
        float seg = startSeg + (float)x / (float)CAR_SEG_W;
        int h;
        if (seg < 0.0f) {
            h = 22;
        } else {
            int32_t si = (int32_t)seg;
            if (si >= g_terrainHead - 1) {
                h = (int)g_carTBuf[(g_terrainHead - 1) % CAR_TBUF_SIZE];
            } else {
                float f = seg - (float)si;
                h = (int)((float)g_carTBuf[si % CAR_TBUF_SIZE] * (1.0f - f) +
                           (float)g_carTBuf[(si + 1) % CAR_TBUF_SIZE] * f + 0.5f);
            }
        }
        if (h < DISP_H) g_display.drawFastVLine(x, h, DISP_H - h, SSD1306_WHITE);
    }
}

// Shared: draw car sprite at screen position with pitch angle
static void drawCarSprite(int cx, int cy, float pitch) {
    float cosP = cosf(pitch);
    float sinP = sinf(pitch);
    int   w1x  = cx - (int)((float)CAR_HALFWB * cosP);
    int   w1y  = cy + (int)((float)CAR_HALFWB * sinP) + 4;  // rear: goes DOWN when nose-up
    int   w2x  = cx + (int)((float)CAR_HALFWB * cosP);
    int   w2y  = cy - (int)((float)CAR_HALFWB * sinP) + 4;  // front: goes UP when nose-up
    g_display.drawCircle(w1x, w1y, CAR_WHEEL_R, SSD1306_WHITE);
    g_display.drawCircle(w2x, w2y, CAR_WHEEL_R, SSD1306_WHITE);
    g_display.drawLine(w1x, w1y - CAR_WHEEL_R, w2x, w2y - CAR_WHEEL_R, SSD1306_WHITE);
    int cabX = (w1x + w2x) / 2 - (int)(3.0f * sinP);
    int cabY = (w1y + w2y) / 2 - CAR_WHEEL_R - (int)(3.0f * cosP);
    g_display.drawRect(cabX - 3, cabY - 4, 6, 4, SSD1306_WHITE);
}

static void renderCarGame() {
    if (!g_oledReady) return;
    g_display.clearDisplay();
    g_display.setTextColor(SSD1306_WHITE);
    g_display.setTextSize(1);

    drawCarTerrain();
    drawCarSprite(CAR_SCREEN_X, (int)g_carY, g_carPitch);

    // Distance score top-right
    char buf[8];
    snprintf(buf, sizeof(buf), "%u", (unsigned)g_carDist);
    g_display.setCursor(DISP_W - (int)strlen(buf) * 6, ROW0);
    g_display.print(buf);

    // Wheelie warning: pitch indicator bar top-left
    // Bar fills left→right as pitch magnitude grows toward flip angle
    int pitchPct = (int)(fabsf(g_carPitch) / CAR_FLIP_ANGLE * 18.0f);
    if (pitchPct > 0) {
        pitchPct = min(pitchPct, 18);
        g_display.drawRect(0, 0, 20, 4, SSD1306_WHITE);
        g_display.fillRect(1, 1, pitchPct, 2, SSD1306_WHITE);
    }

    g_display.display();
}

// Explosion init: scatter 8 debris particles from car position
static void startCarExplosion() {
    float cx = (float)CAR_SCREEN_X;
    float cy = g_carY;
    for (int i = 0; i < 8; i++) {
        float angle = (float)i * (6.2832f / 8.0f) + g_carPitch;
        float spd   = 1.3f + (float)random(150) / 100.0f;
        g_explParts[i] = { cx, cy, cosf(angle) * spd, sinf(angle) * spd - 0.7f };
    }
    g_explFrame = 1;
}

// Explosion tick: advance particle physics
static void updateCarExplosion() {
    if (g_explFrame > 8) {
        for (int i = 0; i < 8; i++) {
            g_explParts[i].x  += g_explParts[i].vx;
            g_explParts[i].y  += g_explParts[i].vy;
            g_explParts[i].vy += 0.22f; // debris gravity
            g_explParts[i].vx *= 0.95f; // drag
        }
    }
    g_explFrame++;
}

// Explosion render: 3 phases — blast wave / debris+fire / game-over text
static void renderCarExplosion() {
    if (!g_oledReady) return;
    g_display.clearDisplay();
    g_display.setTextColor(SSD1306_WHITE);
    g_display.setTextSize(1);

    drawCarTerrain();

    int cx = CAR_SCREEN_X;
    int cy = (int)g_carY;

    if (g_explFrame <= 8) {
        // Phase 1: expanding blast ring + BOOM text
        int r = g_explFrame * 3;
        if (r < 32) g_display.drawCircle(cx, cy, r, SSD1306_WHITE);
        if (r > 4)  g_display.drawCircle(cx, cy, r - 3, SSD1306_WHITE);
        int tx = cx - 12; if (tx < 0) tx = 0;
        int ty = cy - 10; if (ty < 0) ty = 0;
        g_display.setCursor(tx, ty);
        g_display.print("BOOM!");
    } else if (g_explFrame <= 40) {
        // Phase 2: flying debris rectangles + fire flicker
        for (int i = 0; i < 8; i++) {
            int px = (int)g_explParts[i].x;
            int py = (int)g_explParts[i].y;
            if (px >= 0 && px < DISP_W && py >= 0 && py < DISP_H)
                g_display.fillRect(px, py, 2, 2, SSD1306_WHITE);
        }
        if (g_explFrame % 2 == 0) {
            // Fire block flash
            g_display.fillRect(cx - 4, cy - 5, 9, 6, SSD1306_WHITE);
        } else {
            // Scattered fire pixels
            for (int f = 0; f < 5; f++) {
                int fx = cx + (int)random(-5, 6);
                int fy = cy + (int)random(-4, 5);
                if (fx >= 0 && fx < DISP_W && fy >= 0 && fy < DISP_H)
                    g_display.drawPixel(fx, fy, SSD1306_WHITE);
            }
        }
    } else {
        // Phase 3: debris settling + CRASHED text
        for (int i = 0; i < 8; i++) {
            int px = (int)g_explParts[i].x;
            int py = (int)g_explParts[i].y;
            if (px >= 0 && px < DISP_W && py >= 0 && py < DISP_H)
                g_display.drawPixel(px, py, SSD1306_WHITE);
        }
        if (g_explFrame >= 47) {
            g_display.setCursor(17, ROW1);
            g_display.print("** CRASHED **");
            char buf[14];
            snprintf(buf, sizeof(buf), "   Dist: %u", (unsigned)g_carDist);
            g_display.setCursor(0, ROW2);
            g_display.print(buf);
        }
    }

    g_display.display();
}

static void renderCarGameOver() {
    if (!g_oledReady) return;
    static uint32_t lastUp = 0;
    if (millis() - lastUp < 80) return;
    lastUp = millis();

    dispClear();
    drawHeader("CAR GAME");
    drawRow(ROW1, "CRASHED!", false);
    char buf[22];
    snprintf(buf, sizeof(buf), "Dist: %u", (unsigned)g_carDist);
    drawRow(ROW2, buf, false);
    drawRow(ROW3, "OK=retry BACK=exit", false);
    g_display.display();
}

// ---------------------------------------------------------------------------
// 3D Racing — shared wireframe car geometry (logo + player car)
// +x=right +y=up +z=rear; viewed from behind at angle=0 (ca=1 sa=0)
// ---------------------------------------------------------------------------
static const int8_t kCarVerts[20][3] = {
    // Left side profile (x=-5)
    {-5,-2,-9}, // 0  front-bumper-bottom
    {-5,-2, 9}, // 1  rear-bumper-bottom
    {-5, 2,-9}, // 2  front-bumper-top
    {-5, 3,-3}, // 3  windshield-base
    {-5, 5, 0}, // 4  roof-front
    {-5, 5, 4}, // 5  roof-rear
    {-5, 3, 7}, // 6  rear-window-base
    {-5, 2, 9}, // 7  trunk-top
    // Right side profile (x=+5)
    { 5,-2,-9}, // 8
    { 5,-2, 9}, // 9
    { 5, 2,-9}, // 10
    { 5, 3,-3}, // 11
    { 5, 5, 0}, // 12
    { 5, 5, 4}, // 13
    { 5, 3, 7}, // 14
    { 5, 2, 9}, // 15
    // Wheel centers (stick out laterally, y=-4)
    {-7,-4,-6}, // 16 FL
    { 7,-4,-6}, // 17 FR
    {-7,-4, 6}, // 18 RL
    { 7,-4, 6}, // 19 RR
};
static const uint8_t kCarEdges[24][2] = {
    {0,2},{2,3},{3,4},{4,5},{5,6},{6,7},{7,1},{0,1},   // left outline
    {8,10},{10,11},{11,12},{12,13},{13,14},{14,15},{15,9},{8,9}, // right outline
    {2,10},{3,11},{4,12},{5,13},{6,14},{7,15},{0,8},{1,9},       // cross-bars
};

// ---------------------------------------------------------------------------
// 3D Racing — pure math helpers
// ---------------------------------------------------------------------------
static int racingProjY(int z) {
    // y = HORIZON + (BOTTOM-HORIZON) * (Z_FAR-z) / Z_FAR
    return RACING_HORIZON_Y +
           (int)((long)(RACING_BOTTOM_Y - RACING_HORIZON_Y) * (RACING_Z_FAR - z) / RACING_Z_FAR);
}
static int racingRoadHalf(int z) {
    return (int)((long)RACING_ROAD_BOT * (RACING_Z_FAR - z) / RACING_Z_FAR);
}
static int racingCenterX(int z) {
    // Curve: full at horizon (z=Z_FAR), zero at player level (z=0)
    int curveX = (int)((long)g_racingCurveOff * z / RACING_Z_FAR);
    // Camera: follows player lane — pans opposite to slide direction
    // Full effect at bottom (z=0), zero at horizon (z=Z_FAR)
    int camOff = -g_racingSlide * 18 / 256;
    int camX   = (int)((long)camOff * (RACING_Z_FAR - z) / RACING_Z_FAR);
    return RACING_VP_X + curveX + camX;
}
static int racingProjX(int z, int slideQ8) {
    int rh = racingRoadHalf(z);
    return racingCenterX(z) + (int)((long)rh * 55 / 100 * slideQ8 / 256);
}
static int racingSprite(int z) {
    int s = (int)((long)9 * (RACING_Z_FAR - z) / RACING_Z_FAR) + 1;
    return s > 10 ? 10 : (s < 1 ? 1 : s);
}

// ---------------------------------------------------------------------------
// 3D Racing — NVS
// ---------------------------------------------------------------------------
static void racingLoadHi() {
    g_racingHiScore = (uint16_t)g_prefs.getUShort("hi_racing", 0);
}
static void racingSaveHi() {
    if (g_racingScore > g_racingHiScore) {
        g_racingHiScore = g_racingScore;
        g_prefs.putUShort("hi_racing", g_racingHiScore);
    }
}

// ---------------------------------------------------------------------------
// 3D Racing — spawn director (anti-cheese)
// ---------------------------------------------------------------------------
static void racingSpawn() {
    int slot = -1;
    for (int i = 0; i < RACING_MAX_OPP; i++) {
        if (!g_racingOpp[i].active) { slot = i; break; }
    }
    if (slot < 0) return;

    // Shuffle lane candidates
    int lanes[3] = { -1, 0, 1 };
    for (int i = 2; i > 0; i--) {
        int j = random(i + 1);
        int t = lanes[i]; lanes[i] = lanes[j]; lanes[j] = t;
    }

    // Pick lane: prefer not player's target lane and not already occupied far ahead
    int best = lanes[0];
    for (int li = 0; li < 3; li++) {
        int ln = lanes[li];
        bool playerHere = (ln == g_racingTargetLane);
        bool blocked = false;
        for (auto& o : g_racingOpp) {
            if (!o.active) continue;
            if (o.laneQ8 / 256 == ln && o.z > 400) { blocked = true; break; }
        }
        if (!playerHere && !blocked) { best = ln; break; }
        if (!blocked) best = ln; // blocked-by-player still better than all-blocked
    }

    int playerUnits = g_racingSpeedQ4 >> 2;
    int oppUnits    = playerUnits * (30 + random(31)) / 100; // 30..60% of player
    if (oppUnits < 2) oppUnits = 2;

    g_racingOpp[slot] = { RACING_Z_FAR - 20, oppUnits * 4, best * 256, true, false };
}

// ---------------------------------------------------------------------------
// 3D Racing — init
// ---------------------------------------------------------------------------
static void racingInitLogo() {
    g_racingLogoFrame = 0;
}

static void racingInitGame() {
    racingSaveHi();
    g_racingScore      = 0;
    g_racingSpeedQ4    = 6 * 4;    // 6 z-units/tick
    g_racingAccTick    = 0;
    g_racingPlayerLane = 0;
    g_racingTargetLane = 0;
    g_racingSlide      = 0;
    g_racingScroll     = 0;
    g_racingFlash      = 0;
    g_racingOver       = false;
    g_racingInputQ     = 0;
    g_racingCrashFrame = 0;
    g_racingSpinTicks  = 0;
    g_racingSpinAngle  = 0.0f;
    g_racingLastMs     = millis();
    for (auto& o : g_racingOpp) o.active = false;
    g_racingOpp[0]     = { 800, 3 * 4, 0, true, false }; // first car in center
    g_racingCurveOff    = 0;
    g_racingCurveTarget = 0;
    g_racingCurveTicks  = 80;
    g_racingSideTimer   = 0;
    for (auto& s : g_racingSide) s.active = false;
}

// ---------------------------------------------------------------------------
// 3D Racing — sim step (fixed timestep)
// ---------------------------------------------------------------------------
static void racingStep() {
    // Consume input queue
    if (g_racingSpinTicks > 0) {
        // Off-road: spinning on grass — no steering, slow down, spin angle grows
        g_racingSpinTicks--;
        g_racingSpinAngle += 0.5f;
        if (g_racingSpeedQ4 > 4) g_racingSpeedQ4 -= 2;
        if (g_racingSpinTicks == 0) {           // snap back to center lane
            g_racingTargetLane = 0;
            g_racingSlide      = 0;
        }
    } else {
        if (g_racingInputQ & 1) {
            if (g_racingTargetLane <= -1) {     // already at left edge → off road
                g_racingSpinTicks = 20;
                g_racingSpinAngle = 0.0f;
                g_racingSpinDir   = -1;
            } else { g_racingTargetLane--; }
        }
        if (g_racingInputQ & 2) {
            if (g_racingTargetLane >= 1) {      // already at right edge → off road
                g_racingSpinTicks = 20;
                g_racingSpinAngle = 0.0f;
                g_racingSpinDir   = 1;
            } else { g_racingTargetLane++; }
        }
    }
    g_racingInputQ = 0;

    // Slide easing (exponential, Q8)
    int targetQ8 = g_racingTargetLane * 256;
    int diff      = targetQ8 - g_racingSlide;
    g_racingSlide += diff >> 2;
    if (diff > -3 && diff < 3) {
        g_racingSlide      = targetQ8;
        g_racingPlayerLane = g_racingTargetLane;
    }

    // Acceleration: +1 z-unit/tick every 40 ticks (~1.6 s)
    if (++g_racingAccTick >= 40) {
        g_racingAccTick = 0;
        if (g_racingSpeedQ4 < 15 * 4) g_racingSpeedQ4 += 4;
    }

    int playerZ  = g_racingSpeedQ4 >> 2;
    g_racingScroll = (g_racingScroll + playerZ) % RACING_Z_FAR;

    // Curve: lerp toward target, change target periodically
    {
        int diff = g_racingCurveTarget - g_racingCurveOff;
        g_racingCurveOff += diff >> 3;
        if (--g_racingCurveTicks <= 0) {
            g_racingCurveTicks  = 80 + random(80); // 3.2-6.4 s between turns
            int mag = 15 + random(14);              // 15..28 px at horizon
            g_racingCurveTarget = random(2) ? mag : -mag;
        }
    }

    // Roadside objects: move forward toward player
    for (auto& s : g_racingSide) {
        if (!s.active) continue;
        s.z -= playerZ;
        if (s.z < 0) s.active = false;
    }
    if (++g_racingSideTimer >= 4) {
        g_racingSideTimer = 0;
        for (int i = 0; i < 8; i++) {
            if (!g_racingSide[i].active) {
                g_racingSide[i] = {
                    RACING_Z_FAR - 30,
                    (int8_t)(random(2) ? 1 : -1),
                    (uint8_t)random(4),
                    true
                };
                break;
            }
        }
    }

    // Update opponents
    bool needSpawn = true;
    int  farthestZ = 0;

    for (int i = 0; i < RACING_MAX_OPP; i++) {
        auto& o = g_racingOpp[i];
        if (!o.active) continue;
        needSpawn = false;

        int closure = playerZ - (o.speedQ4 >> 2);
        o.z -= closure;

        if (o.z > farthestZ) farthestZ = o.z;

        // Score when opponent passes player
        if (!o.scored && o.z < 0) {
            o.scored = true;
            g_racingScore++;
        }

        // Collision: in zone and overlapping lane
        if (o.z > -30 && o.z < 90) {
            if (abs(o.laneQ8 - g_racingSlide) < 210) {
                g_racingOver = true;
            }
        }

        // Despawn behind player
        if (o.z < -80) o.active = false;
    }

    if (needSpawn || farthestZ < RACING_Z_FAR * 6 / 10) racingSpawn();
}

// ---------------------------------------------------------------------------
// 3D Racing — render helpers
// ---------------------------------------------------------------------------
static void racingDrawRoad() {
    int depth   = RACING_BOTTOM_Y - RACING_HORIZON_Y;

    // --- Road edges: per-scanline so they follow the curve ---
    for (int y = RACING_HORIZON_Y + 1; y <= RACING_BOTTOM_Y; y++) {
        int zAtY = (int)((long)RACING_Z_FAR * (depth - (y - RACING_HORIZON_Y)) / depth);
        int cx   = racingCenterX(zAtY);
        int rh   = racingRoadHalf(zAtY);
        // Banking tilt: full at horizon, zero at bottom
        int tilt = g_racingSlide * 2 / 256 * (RACING_BOTTOM_Y - y) / depth;
        int lx = cx - rh, rx = cx + rh;
        if (lx >= 0 && lx < DISP_W) g_display.drawPixel(lx, y + tilt, SSD1306_WHITE);
        if (rx >= 0 && rx < DISP_W) g_display.drawPixel(rx, y - tilt, SSD1306_WHITE);
    }

    // --- Grass dots outside road per scanline ---
    int scrollP = (g_racingScroll >> 2) & 0xF;
    for (int y = RACING_HORIZON_Y + 2; y <= RACING_BOTTOM_Y; y++) {
        int zAtY  = (int)((long)RACING_Z_FAR * (depth - (y - RACING_HORIZON_Y)) / depth);
        int cx    = racingCenterX(zAtY);
        int rh    = racingRoadHalf(zAtY);
        int lEdge = cx - rh;
        int rEdge = cx + rh;
        int ph    = (y + scrollP) & 7;
        int lx1 = lEdge - 3 - (y & 2);
        int lx2 = lEdge - 8 - (y & 1);
        int rx1 = rEdge + 3 + (y & 2);
        int rx2 = rEdge + 8 + (y & 1);
        if (ph < 3 && lx1 >= 0)     g_display.drawPixel(lx1, y, SSD1306_WHITE);
        if (ph < 2 && lx2 >= 0)     g_display.drawPixel(lx2, y, SSD1306_WHITE);
        if (ph < 3 && rx1 < DISP_W) g_display.drawPixel(rx1, y, SSD1306_WHITE);
        if (ph < 2 && rx2 < DISP_W) g_display.drawPixel(rx2, y, SSD1306_WHITE);
    }

    // --- Dashed center line (scrolls with speed) ---
    {
        int dashPeriod = 5;
        for (int y = RACING_HORIZON_Y + 1; y <= RACING_BOTTOM_Y; y++) {
            int zAtY2 = (int)((long)RACING_Z_FAR * (depth - (y - RACING_HORIZON_Y)) / depth);
            int phase = ((y * 2) + (int)(g_racingScroll >> 2)) % dashPeriod;
            if (phase < 3)
                g_display.drawPixel(racingCenterX(zAtY2), y, SSD1306_WHITE);
        }
    }
}

static void racingDrawPlayer() {
    int cx = racingProjX(0, g_racingSlide);
    if (g_racingSpinTicks > 0)
        cx += g_racingSpinDir * (20 - g_racingSpinTicks); // drift off road
    cx = constrain(cx, 2, DISP_W - 2);
    int cy = RACING_PLAYER_Y + 1;

    int sinY, cosY;
    if (g_racingSpinTicks > 0) {
        sinY = (int)(sinf(g_racingSpinAngle) * 256);
        cosY = (int)(cosf(g_racingSpinAngle) * 256);
    } else {
        sinY = g_racingSlide * 3 / 8;
        cosY = 256;
    }

    int px[20], py[20];
    for (int i = 0; i < 20; i++) {
        int vx = kCarVerts[i][0], vy = kCarVerts[i][1], vz = kCarVerts[i][2];
        int wx = (vx * cosY + vz * sinY) / 256;
        int wz = (-vx * sinY + vz * cosY) / 256;
        px[i] = constrain(cx + wx * 8 / 10, 0, DISP_W - 1);
        py[i] = constrain(cy - vy + wz / 4, RACING_HORIZON_Y, RACING_BOTTOM_Y);
    }
    g_display.fillRect(cx - 12, cy - 8, 24, 16, SSD1306_BLACK);
    for (int e = 0; e < 24; e++)
        g_display.drawLine(px[kCarEdges[e][0]], py[kCarEdges[e][0]],
                           px[kCarEdges[e][1]], py[kCarEdges[e][1]], SSD1306_WHITE);
    for (int w = 16; w < 20; w++)
        g_display.fillRect(px[w] - 1, py[w], 3, 2, SSD1306_WHITE);
}

static void racingDrawOpponent(const RacingOpp& o) {
    if (o.z <= 0 || o.z >= RACING_Z_FAR) return;
    int sy = racingProjY(o.z);
    if (sy <= RACING_HORIZON_Y || sy > RACING_BOTTOM_Y - 1) return;
    int sx = racingProjX(o.z, o.laneQ8);
    int sc = racingSprite(o.z);
    int w  = sc;
    int h  = sc * 3 / 4;
    if (h < 1) h = 1;
    if (sc < 4) {
        g_display.fillRect(sx - w / 2 - 1, sy - h - 1, w + 2, h + 2, SSD1306_BLACK);
        g_display.fillRect(sx - w / 2, sy - h, w, h, SSD1306_WHITE);
    } else {
        // 3D perspective box: front face (near player) + rear face + top connecting edges
        static const int OPP_ZDEPTH = 80;
        int zr = o.z + OPP_ZDEPTH;
        if (zr >= RACING_Z_FAR) zr = RACING_Z_FAR - 1;
        int sy_r = racingProjY(zr);
        int sx_r = racingProjX(zr, o.laneQ8);
        int sc_r = racingSprite(zr);
        int wr = sc_r;          if (wr < 1) wr = 1;
        int hr = sc_r * 3 / 4; if (hr < 1) hr = 1;
        int fBL = sx - w / 2, fBR = sx + w / 2;
        int rBL = sx_r - wr / 2, rBR = sx_r + wr / 2;
        // Clear behind entire car volume
        g_display.fillRect(fBL - 1, sy_r - hr - 1, w + 2, (sy - sy_r) + h + 2, SSD1306_BLACK);
        // Front face
        g_display.drawFastHLine(fBL, sy,     w, SSD1306_WHITE);   // bumper
        g_display.drawFastHLine(fBL, sy - h, w, SSD1306_WHITE);   // roofline
        g_display.drawFastVLine(fBL, sy - h, h, SSD1306_WHITE);   // left pillar
        g_display.drawFastVLine(fBR, sy - h, h, SSD1306_WHITE);   // right pillar
        if (h >= 3)
            g_display.drawFastHLine(fBL + 1, sy - h * 2 / 3, w - 2, SSD1306_WHITE); // windshield
        // Rear face (smaller, near horizon)
        g_display.drawFastHLine(rBL, sy_r,      wr, SSD1306_WHITE);
        g_display.drawFastHLine(rBL, sy_r - hr, wr, SSD1306_WHITE);
        // Top face: front roofline to rear roofline
        g_display.drawLine(fBL, sy - h, rBL, sy_r - hr, SSD1306_WHITE);
        g_display.drawLine(fBR, sy - h, rBR, sy_r - hr, SSD1306_WHITE);
    }
}

static void racingDrawSide(const RacingSide& s) {
    if (s.z <= 0 || s.z >= RACING_Z_FAR) return;
    int sy = racingProjY(s.z);
    if (sy < RACING_HORIZON_Y || sy > RACING_BOTTOM_Y) return;
    int cx = racingCenterX(s.z);
    int rh = racingRoadHalf(s.z);
    int sc = racingSprite(s.z);
    int gap = (s.type == 3) ? 4 : 0;
    int bx = (s.side < 0) ? (cx - rh - 8 - sc - gap) : (cx + rh + 8 + gap);
    if (bx < 0 || bx + sc + 3 >= DISP_W) return;

    switch (s.type) {
    case 0: // grass tuft
        g_display.drawPixel(bx, sy, SSD1306_WHITE);
        if (sc >= 2) g_display.drawPixel(bx + 1, sy - 1, SSD1306_WHITE);
        if (sc >= 3) g_display.drawPixel(bx + 2, sy,     SSD1306_WHITE);
        break;
    case 1: // bush
        {
            int bw = sc + 2, bh = sc / 2 + 1;
            g_display.fillRect(bx, sy - bh, bw, bh + 1, SSD1306_WHITE);
            if (sc >= 6 && bw >= 4 && bh >= 2) // hollow when large
                g_display.fillRect(bx + 1, sy - bh + 1, bw - 2, bh - 1, SSD1306_BLACK);
        }
        break;
    case 2: // tree: trunk + triangular canopy
        {
            int trunkH = sc / 2 + 1;
            int topW   = sc + 2;
            int tcx    = bx + topW / 2;
            g_display.drawFastVLine(tcx, sy - trunkH, trunkH + 1, SSD1306_WHITE);
            for (int row = 0; row <= trunkH; row++) {
                int tw2 = topW * (trunkH - row + 1) / (trunkH + 1);
                if (tw2 < 1) tw2 = 1;
                g_display.drawFastHLine(tcx - tw2 / 2, sy - trunkH - row, tw2, SSD1306_WHITE);
            }
        }
        break;
    case 3: // building: tall rectangle with floor lines
        {
            int bw     = sc + 4;
            int bh     = sc * 2 + 4;
            int floors = bh / 3;
            g_display.drawRect(bx, sy - bh, bw, bh + 1, SSD1306_WHITE);
            for (int f = 1; f < floors; f++)
                g_display.drawFastHLine(bx + 1, sy - f * 3, bw - 2, SSD1306_WHITE);
        }
        break;
    }
}

// ---------------------------------------------------------------------------
// 3D Racing — render screens
// ---------------------------------------------------------------------------
static void renderRacingLogo() {
    if (!g_oledReady) return;
    static uint32_t lastUp = 0;
    if (millis() - lastUp < 80) return;
    lastUp = millis();

    dispClear();
    drawHeader("3D RACING");

    // Left zone: BACK button
    g_display.drawFastVLine(37, ROW1, 24, SSD1306_WHITE);
    g_display.setCursor(4, ROW1 + 3); g_display.print("<");
    g_display.setCursor(1, ROW2 + 2); g_display.print("BACK");

    // Right zone: PLAY button
    g_display.drawFastVLine(89, ROW1, 24, SSD1306_WHITE);
    g_display.setCursor(102, ROW1 + 3); g_display.print(">");
    g_display.setCursor(91,  ROW2 + 2); g_display.print("PLAY");

    // Center zone: spinning car (x=38..88, center=63)
    float angle = g_racingLogoFrame * 0.15f;
    float ca = cosf(angle), sa = sinf(angle);
    int carCX = 63, carCY = 20;

    int px[20], py[20];
    for (int i = 0; i < 20; i++) {
        float wx = kCarVerts[i][0] * ca - kCarVerts[i][2] * sa;
        float wy = kCarVerts[i][1];
        float wz = kCarVerts[i][0] * sa + kCarVerts[i][2] * ca;
        px[i] = constrain(carCX + (int)(wx * 1.7f),         38, 88);
        py[i] = constrain(carCY - (int)(wy * 2.0f) + (int)(wz * 0.3f), ROW1+1, RACING_BOTTOM_Y);
    }
    for (int e = 0; e < 24; e++)
        g_display.drawLine(px[kCarEdges[e][0]], py[kCarEdges[e][0]],
                           px[kCarEdges[e][1]], py[kCarEdges[e][1]], SSD1306_WHITE);
    for (int w = 16; w < 20; w++)
        g_display.fillRect(px[w]-2, py[w]-1, 4, 2, SSD1306_WHITE);

    g_display.display();
    g_racingLogoFrame++;
}

static void renderRacingGame() {
    if (!g_oledReady) return;
    static uint32_t lastUp = 0;
    if (millis() - lastUp < RACING_TICK_MS) return;
    lastUp = millis();

    g_display.clearDisplay();

    racingDrawRoad();
    for (auto& s : g_racingSide) if (s.active) racingDrawSide(s);

    // Painter's algorithm: sort by z desc (far first), insertion sort N=4
    RacingOpp sorted[RACING_MAX_OPP];
    memcpy(sorted, g_racingOpp, sizeof(sorted));
    for (int i = 1; i < RACING_MAX_OPP; i++) {
        RacingOpp key = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j].z < key.z) { sorted[j + 1] = sorted[j]; j--; }
        sorted[j + 1] = key;
    }
    for (auto& o : sorted) if (o.active) racingDrawOpponent(o);
    racingDrawPlayer();

    // Score: bottom-left, no leading zeros, black background patch
    {
        char sc[8];
        snprintf(sc, sizeof(sc), "%u", g_racingScore);
        int sw = strlen(sc) * 6;
        g_display.fillRect(0, ROW3, sw + 2, 8, SSD1306_BLACK);
        g_display.setTextColor(SSD1306_WHITE);
        g_display.setCursor(1, ROW3);
        g_display.print(sc);
    }

    g_display.display();
}

static void renderRacingGameOver() {
    if (!g_oledReady) return;
    static uint32_t lastUp = 0;
    if (millis() - lastUp < 80) return;
    lastUp = millis();

    dispClear();
    drawHeader("CRASHED!");
    char ln1[22];
    snprintf(ln1, sizeof(ln1), "Score: %u", g_racingScore);
    g_display.setCursor(0, ROW1);
    g_display.print(ln1);
    if (g_racingScore > 0 && g_racingScore >= g_racingHiScore) {
        g_display.setCursor(0, ROW2);
        g_display.print("** NEW HI-SCORE! **");
    } else {
        char ln2[22];
        snprintf(ln2, sizeof(ln2), "Best: %u", g_racingHiScore);
        g_display.setCursor(0, ROW2);
        g_display.print(ln2);
    }
    g_display.setCursor(0, ROW3);
    g_display.print("OK=retry BACK=exit");
    g_display.display();
}

// ---------------------------------------------------------------------------
// 3D Racing — crash animation
// ---------------------------------------------------------------------------
static void racingInitCrash() {
    g_racingCrashFrame = 0;
    g_racingCrashCX    = racingProjX(0, g_racingSlide);
    g_racingCrashDir   = (g_racingSlide >= 0) ? 1 : -1;
    // 12 debris particles: {vx*4, vy*4} — upward = negative y
    static const int8_t kDebV[12][2] = {
        { 8,-20},{-8,-20},{16,-16},{-16,-16},
        {24, -8},{-24, -8},{ 4,-28},{  -4,-28},
        {20,-12},{-20,-12},{12,-24},{ -12,-24}
    };
    for (int i = 0; i < 12; i++) {
        g_expPX[i] = (int16_t)(g_racingCrashCX << 2);
        g_expPY[i] = (int16_t)(RACING_PLAYER_Y  << 2);
        g_expVX[i] = kDebV[i][0];
        g_expVY[i] = kDebV[i][1];
    }
}

static void renderRacingCrash() {
    if (!g_oledReady) return;
    static uint32_t lastUp = 0;
    if (millis() - lastUp < RACING_TICK_MS) return;
    lastUp = millis();

    uint8_t f   = g_racingCrashFrame++;
    int     ccx = g_racingCrashCX;
    int     ccy = RACING_PLAYER_Y;

    if (f < 20) {
        // Phase 1: impact flash → spinning car + flying debris pixels
        g_display.clearDisplay();
        racingDrawRoad();

        // Update debris and draw as pixels
        for (int i = 0; i < 12; i++) {
            g_expVY[i] += 3;                          // gravity Q4
            g_expPX[i] += g_expVX[i];
            g_expPY[i] += g_expVY[i];
            int sx = g_expPX[i] >> 2, sy = g_expPY[i] >> 2;
            if (sx >= 0 && sx < DISP_W && sy >= 0 && sy < DISP_H)
                g_display.drawPixel(sx, sy, SSD1306_WHITE);
        }

        if (f == 0) {
            // Impact flash: bright white blob at hit point
            g_display.fillRect(ccx - 10, ccy - 5, 20, 10, SSD1306_WHITE);
        } else {
            // Spinning car: angle accelerates, drifts sideways, deterministic jitter
            float ca = cosf(f * 0.7f), sa = sinf(f * 0.7f);
            int cx = constrain(ccx + g_racingCrashDir * f * 3, 2, DISP_W - 2);
            int cy = ccy + (int)((f * 13 % 5) - 2);
            int px[20], py[20];
            for (int i = 0; i < 20; i++) {
                int vx = kCarVerts[i][0], vy = kCarVerts[i][1], vz = kCarVerts[i][2];
                int wx = (int)(vx * ca - vz * sa);
                int wz = (int)(vx * sa + vz * ca);
                px[i] = constrain(cx + wx * 8 / 10, 0, DISP_W - 1);
                py[i] = constrain(cy - vy + wz / 4, RACING_HORIZON_Y, RACING_BOTTOM_Y);
            }
            g_display.fillRect(cx - 12, cy - 8, 24, 16, SSD1306_BLACK);
            for (int e = 0; e < 24; e++)
                g_display.drawLine(px[kCarEdges[e][0]], py[kCarEdges[e][0]],
                                   px[kCarEdges[e][1]], py[kCarEdges[e][1]], SSD1306_WHITE);
        }

    } else if (f < 42) {
        // Phase 2: explosion — debris trails + two expanding rings + fireball
        g_display.clearDisplay();
        uint8_t ef = f - 20;   // 0..21

        // Debris: draw trail line from previous to new position
        for (int i = 0; i < 12; i++) {
            int px0 = g_expPX[i] >> 2, py0 = g_expPY[i] >> 2;
            g_expVY[i] += 3;
            g_expPX[i] += g_expVX[i];
            g_expPY[i] += g_expVY[i];
            int sx = g_expPX[i] >> 2, sy = g_expPY[i] >> 2;
            g_display.drawLine(constrain(px0, 0, DISP_W-1), constrain(py0, 0, DISP_H-1),
                               constrain(sx,  0, DISP_W-1), constrain(sy,  0, DISP_H-1),
                               SSD1306_WHITE);
        }

        // Two concentric expanding rings (ring 2 lags by 4 frames)
        static const int8_t kRing[8][2] = {
            { 2, 0},{ 1, 1},{ 0, 2},{-1, 1},
            {-2, 0},{-1,-1},{ 0,-2},{ 1,-1}
        };
        for (int ring = 0; ring < 2; ring++) {
            int lag = ring * 4;
            if (ef < (uint8_t)lag) continue;
            int r = (ef - lag) * 3 + 2;
            for (auto& d : kRing) {
                int sx = ccx + d[0] * r / 2, sy = ccy + d[1] * r / 4;
                if (sx >= 0 && sx < DISP_W && sy >= 0 && sy < DISP_H)
                    g_display.drawPixel(sx, sy, SSD1306_WHITE);
            }
        }

        // Central fireball: expands then contracts
        int blob = (ef < 5) ? ef + 2 : (ef < 10) ? (10 - ef) : 0;
        if (blob > 0)
            g_display.fillRect(ccx - blob, ccy - blob / 2, blob * 2, blob, SSD1306_WHITE);

    } else {
        // Phase 3: blowup — 3 intense full-screen white flashes
        uint8_t ff = f - 42;
        bool lit = (ff < 3) || (ff >= 5 && ff < 8) || (ff >= 10 && ff < 12);
        if (lit)
            g_display.fillRect(0, 0, DISP_W, DISP_H, SSD1306_WHITE);
        else
            g_display.clearDisplay();
    }

    g_display.display();

    if (f >= 54) {
        racingSaveHi();
        g_state = STATE_RACING_GAMEOVER;
    }
}

// ---------------------------------------------------------------------------
// Scan complete callback
// ---------------------------------------------------------------------------
static void onScanComplete(NimBLEScanResults r) {
    Serial.printf("[scan] done, %d devices\n", r.getCount());
    g_scanDone = true;
}

// ---------------------------------------------------------------------------
// UART packet helpers
// ---------------------------------------------------------------------------
static bool keyInReport(uint8_t k, const uint8_t* report) {
    for (int i = 2; i < 8; ++i) if (report[i] == k) return true;
    return false;
}

static void sendPacket(uint8_t event, uint8_t keycode) {
    uint8_t pkt[2] = { event, keycode };
    Serial1.write(pkt, 2);
    Serial.printf("[key] %s HID=0x%02X\n", event == 0x01 ? "DOWN" : "UP  ", keycode);
    g_lastKeyEvent = event;
    g_lastKeyCode  = keycode;
    g_keyUntil     = millis() + KEY_DISPLAY_MS;
}

// ---------------------------------------------------------------------------
// BLE HID notification
// ---------------------------------------------------------------------------
static void notifyCB(NimBLERemoteCharacteristic* /*chr*/,
                     uint8_t* data, size_t len, bool /*isNotify*/) {
    if (len < 8) return;
    uint8_t curMods  = data[0];
    uint8_t prevMods = g_prevReport[0];
    for (int i = 0; i < 8; ++i) {
        uint8_t bit = (1u << i);
        if ((curMods & bit) && !(prevMods & bit)) sendPacket(0x01, kModifierKeycode[i]);
        if (!(curMods & bit) && (prevMods & bit)) sendPacket(0x02, kModifierKeycode[i]);
    }
    for (int i = 2; i < 8; ++i) {
        uint8_t k = data[i];
        if (k == 0 || k == 0x01) continue;
        if (!keyInReport(k, g_prevReport)) sendPacket(0x01, k);
    }
    for (int i = 2; i < 8; ++i) {
        uint8_t k = g_prevReport[i];
        if (k == 0 || k == 0x01) continue;
        if (!keyInReport(k, data)) sendPacket(0x02, k);
    }
    memcpy(g_prevReport, data, 8);
}

// ---------------------------------------------------------------------------
// BLE callbacks
// ---------------------------------------------------------------------------
class ScanCB : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* dev) override {
        bool hasName = dev->haveName();
        String devName = hasName ? dev->getName().c_str() : "";
        Serial.printf("[scan] found %s %s%s\n", dev->getAddress().toString().c_str(),
                      hasName ? devName.c_str() : "(no name)",
                      dev->isAdvertisingService(NimBLEUUID(UUID_HID_SVC)) ? " [HID]" : "");

        bool addrMatch = (g_targetAddr != NimBLEAddress() && dev->getAddress() == g_targetAddr);
        bool nameMatch = (g_connectedName.length() > 0 && hasName && devName == g_connectedName);
        bool hidMatch  = (g_targetAddr != NimBLEAddress() &&
                          dev->isAdvertisingService(NimBLEUUID(UUID_HID_SVC)));

        if (addrMatch || nameMatch || hidMatch) {
            g_targetAddr = dev->getAddress();
            if (hasName) g_connectedName = devName;
            g_savedDevFound = true;
            return;
        }
        if (!hasName || g_foundCount >= MAX_DEVICES) return;
        for (int i = 0; i < g_foundCount; i++)
            if (g_foundAddrs[i] == dev->getAddress()) return;
        g_foundNames[g_foundCount] = dev->getName().c_str();
        g_foundAddrs[g_foundCount] = dev->getAddress();
        g_foundCount++;
    }
};

class ClientCB : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* /*c*/) override { Serial.println("[ble] link up"); }
    void onDisconnect(NimBLEClient* /*c*/) override {
        Serial.println("[ble] disconnected");
        g_connected      = false;
        g_foundCount     = 0;
        g_scanDone       = false;
        g_savedDevFound  = false;
        g_scanRetryCount = 0;
        memset(g_prevReport, 0, sizeof(g_prevReport));
        if (g_targetAddr != NimBLEAddress()) {
            g_connectArmed = false;
            g_state = STATE_CONNECTING;
        } else {
            g_state = STATE_SCANNING;
        }
    }
    uint32_t onPassKeyRequest() override         { return 0;    }
    bool onConfirmPIN(uint32_t /*pin*/) override { return true; }
};

static ScanCB   g_scanCB;
static ClientCB g_clientCB;

// ---------------------------------------------------------------------------
// Connect to keyboard (blocking)
// ---------------------------------------------------------------------------
static bool connectToKeyboard(const String& name) {
    if (name.length() > 0 && name != "conn timeout" && name != "pair failed" &&
        name != "reconnect failed") {
        g_connectedName = name;
    }

    if (!g_client) {
        g_client = NimBLEDevice::createClient();
        g_client->setClientCallbacks(&g_clientCB, false);
        g_client->setConnectionParams(24, 48, 0, 400);
        g_client->setConnectTimeout(20);
    } else if (g_client->isConnected()) {
        g_client->disconnect();
    }

    Serial.printf("[ble] connecting to %s type=%d name=%s\n",
        g_targetAddr.toString().c_str(), (int)g_targetAddr.getType(), name.c_str());
    if (!g_client->connect(g_targetAddr)) {
        Serial.println("[ble] connect() failed");
        g_connectedName = "conn timeout";
        return false;
    }

    if (!g_client->secureConnection()) {
        Serial.println("[ble] pairing failed");
        g_client->disconnect();
        g_connectedName = "pair failed";
        return false;
    }
    Serial.println("[ble] link secured");

    NimBLERemoteService* hid = g_client->getService(NimBLEUUID(UUID_HID_SVC));
    if (!hid) { Serial.println("[hid] no HID service"); g_client->disconnect(); return false; }
    delay(300);

    if (auto* pm = hid->getCharacteristic(NimBLEUUID(UUID_PROTOCOL_MODE))) {
        uint8_t boot = 0; pm->writeValue(&boot, 1, false);
    }
    if (auto* cp = hid->getCharacteristic(NimBLEUUID(UUID_HID_CTRL_POINT))) {
        uint8_t ex = 1; cp->writeValue(&ex, 1, false);
    }
    hid->getCharacteristic(NimBLEUUID(UUID_BOOT_KBD_INPUT));
    hid->getCharacteristic(NimBLEUUID(UUID_REPORT));

    int subCount = 0;
    if (auto* chars = hid->getCharacteristics())
        for (auto* c : *chars)
            if (c->canNotify() && c->subscribe(true, notifyCB)) ++subCount;

    if (subCount == 0) { Serial.println("[hid] no notify chars"); g_client->disconnect(); return false; }

    NimBLEAddress peerAddr = g_client->getPeerAddress();
    Serial.printf("[hid] scan addr=%s peer addr=%s type=%d\n",
        g_targetAddr.toString().c_str(), peerAddr.toString().c_str(), (int)peerAddr.getType());
    if (peerAddr.getType() == 0 || peerAddr.getType() == 1) {
        g_targetAddr = peerAddr;
    }
    g_prefs.putString("kbd_addr", String(g_targetAddr.toString().c_str()));
    g_prefs.putString("kbd_name", name);
    Serial.printf("[hid] subscribed to %d chars\n", subCount);
    return true;
}

// ---------------------------------------------------------------------------
// LED helper
// ---------------------------------------------------------------------------
static inline void ledWrite(bool on) {
    digitalWrite(LED_PIN, (on ^ LED_ACTIVE_LOW) ? HIGH : LOW);
}

// ---------------------------------------------------------------------------
// WiFi Settings — HTML page (stored in flash, ~700 bytes)
// ---------------------------------------------------------------------------
static const char kSettingsHtml[] PROGMEM = R"html(<!DOCTYPE html>
<html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>HP200LX Bridge</title>
<style>
body{font-family:sans-serif;max-width:420px;margin:24px auto;padding:0 16px}
h2{margin-bottom:16px}
label{display:block;margin-top:12px;font-size:.9em;color:#555}
input{width:100%;box-sizing:border-box;padding:8px;margin-top:4px;border:1px solid #ccc;border-radius:4px}
input[readonly]{background:#f4f4f4;color:#888}
button{margin-top:20px;width:100%;padding:12px;background:#0078d4;color:#fff;border:none;border-radius:4px;font-size:1em;cursor:pointer}
button:hover{background:#005fa3}
.scan-btn{margin-top:6px;background:#555;padding:8px 12px;font-size:.85em;width:auto}
.scan-btn:hover{background:#333}
.pw-wrap{display:flex;gap:6px;margin-top:4px}
.pw-wrap input{margin-top:0;flex:1}
.pw-wrap button{margin-top:0;width:auto;padding:8px 12px;font-size:.85em;background:#555;flex-shrink:0}
.pw-wrap button:hover{background:#333}
select{width:100%;box-sizing:border-box;padding:8px;margin-top:6px;border:1px solid #ccc;border-radius:4px;font-size:.9em}
.msg{margin-top:12px;padding:8px;background:#e6f4ea;border-left:4px solid #34a853;font-size:.9em}
</style></head>
<body>
<h2>HP200LX Bridge Settings</h2>
<form method="POST" action="/save">
<label>WiFi SSID</label>
<input id="ssid" name="ssid" value="%SSID%" maxlength="32" placeholder="Network name">
<button type="button" class="scan-btn" id="scanBtn" onclick="doScan()">Scan Networks</button>
<select id="nets" style="display:none" onchange="pickNet(this)">
<option value="">-- select network --</option>
</select>
<label>WiFi Password</label>
<div class="pw-wrap">
<input id="pass" name="pass" type="password" value="%PASS%" maxlength="64" placeholder="Password">
<button type="button" onclick="togglePass()">Show</button>
</div>
<label>BLE Keyboard (saved)</label>
<input value="%KBDNAME%" readonly>
<button type="submit">Save &amp; Reconnect</button>
</form>
%MSG%
<script>
async function doScan(){
  var btn=document.getElementById('scanBtn');
  btn.textContent='Scanning...';btn.disabled=true;
  try{
    var r=await fetch('/scan');
    var nets=await r.json();
    var sel=document.getElementById('nets');
    sel.innerHTML='<option value="">-- select network --</option>'+
      nets.map(function(n){return '<option value="'+escHtml(n.ssid)+'">'+escHtml(n.ssid)+' ('+n.rssi+'dBm)</option>';}).join('');
    sel.style.display='';
  }catch(e){alert('Scan failed: '+e);}
  btn.textContent='Scan Networks';btn.disabled=false;
}
function pickNet(sel){if(sel.value)document.getElementById('ssid').value=sel.value;}
function escHtml(s){return s.replace(/&/g,'&amp;').replace(/"/g,'&quot;').replace(/</g,'&lt;');}
function togglePass(){var f=document.getElementById('pass'),b=event.target;var show=f.type==='password';f.type=show?'text':'password';b.textContent=show?'Hide':'Show';}
</script>
</body></html>
)html";

static const char kSettingsSavedHtml[] PROGMEM = R"html(<!DOCTYPE html>
<html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<meta http-equiv="refresh" content="3;url=/">
<title>Saved</title></head>
<body style="font-family:sans-serif;max-width:420px;margin:24px auto;padding:0 16px">
<h2>Saved!</h2><p>Reconnecting to new network. Page will reload in 3 seconds.</p>
</body></html>
)html";

// ---------------------------------------------------------------------------
// WiFi Settings — helper functions
// ---------------------------------------------------------------------------
static void drawQR(int px, int py) {
    for (int y = 0; y < g_settingsQR.size; y++)
        for (int x = 0; x < g_settingsQR.size; x++)
            if (qrcode_getModule(&g_settingsQR, x, y))
                g_display.drawPixel(px + x, py + y, SSD1306_WHITE);
}

static void loadOrGenApPass() {
    g_apPass = g_prefs.getString("ap_pass", "");
    if (g_apPass.length() == 8) return;
    // Generate 8 random lowercase letters (easy to type, unambiguous)
    static const char kChars[] = "abcdefghjkmnpqrstuvwxyz23456789"; // no i,l,o,0,1
    const int kLen = sizeof(kChars) - 1;
    g_apPass = "";
    for (int i = 0; i < 8; i++)
        g_apPass += kChars[esp_random() % kLen];
    g_prefs.putString("ap_pass", g_apPass);
    Serial.printf("[wifi] generated AP pass: %s\n", g_apPass.c_str());
}

static void enterWifiSettings() {
    // If BLE happens to be initialized, fully stop it to free the radio.
    // Normally this runs in the WiFi-only boot path where BLE was never
    // inited (getInitialized()==false) so WiFi gets a clean PHY.
    if (NimBLEDevice::getInitialized()) {
        NimBLEScan* scan = NimBLEDevice::getScan();
        if (scan->isScanning()) scan->stop();
        if (g_client && g_client->isConnected()) g_client->disconnect();
        delay(50);
        NimBLEDevice::deinit(false);
        g_client    = nullptr;
        g_connected = false;
        delay(100);
    }

    loadOrGenApPass();
    String ssid = g_prefs.getString("wifi_ssid", "");
    String pass = g_prefs.getString("wifi_pass", "");

    if (ssid.length() == 0) {
        g_wifiIsAP = true;
        WiFi.mode(WIFI_AP);
        delay(100);
        // ESP32-C3 SuperMini: weak regulator browns out at full TX power and
        // the beacon never radiates. Lower TX power to keep current draw sane.
        WiFi.setTxPower(WIFI_POWER_8_5dBm);
        // explicit config: channel 1, not hidden, max 4 clients
        bool ok = WiFi.softAP("HP200LX-Setup", g_apPass.c_str(), 1, 0, 4);
        delay(500);   // give AP time to broadcast before render
        g_wifiIP = "192.168.4.1";
        Serial.printf("[wifi] AP start=%d mode=%d ip=%s ch=%d tx=%d\n",
            ok, WiFi.getMode(), WiFi.softAPIP().toString().c_str(),
            WiFi.channel(), WiFi.getTxPower());
    } else {
        g_wifiIsAP = false;
        WiFi.mode(WIFI_STA);
        delay(100);
        WiFi.setTxPower(WIFI_POWER_8_5dBm);  // prevent brownout on weak C3 regulator
        WiFi.begin(ssid.c_str(), pass.c_str());
        g_wifiConnectStart = millis();
        g_wifiIP = "";
        Serial.printf("[wifi] STA mode, SSID=%s\n", ssid.c_str());
    }

    g_webServer = new WebServer(80);

    g_webServer->on("/", HTTP_GET, []() {
        String pg = FPSTR(kSettingsHtml);
        pg.replace("%SSID%",    g_prefs.getString("wifi_ssid", ""));
        pg.replace("%PASS%",    g_prefs.getString("wifi_pass", ""));
        pg.replace("%KBDNAME%", g_prefs.getString("kbd_name", "(none)"));
        pg.replace("%MSG%",     "");
        g_webServer->send(200, "text/html", pg);
    });

    g_webServer->on("/save", HTTP_POST, []() {
        String newSsid = g_webServer->arg("ssid");
        String newPass = g_webServer->arg("pass");
        g_prefs.putString("wifi_ssid", newSsid);
        g_prefs.putString("wifi_pass", newPass);
        g_webServer->send(200, "text/html", FPSTR(kSettingsSavedHtml));
        g_settingsSaved = true;
        Serial.printf("[wifi] saved SSID=%s\n", newSsid.c_str());
    });

    g_webServer->on("/scan", HTTP_GET, []() {
        int n = WiFi.scanNetworks();
        String json = "[";
        for (int i = 0; i < n; i++) {
            if (i > 0) json += ",";
            String ssid = WiFi.SSID(i);
            ssid.replace("\\", "\\\\");
            ssid.replace("\"", "\\\"");
            json += "{\"ssid\":\"" + ssid + "\",\"rssi\":" + String(WiFi.RSSI(i)) + "}";
        }
        json += "]";
        WiFi.scanDelete();
        g_webServer->send(200, "application/json", json);
    });

    g_webServer->begin();
    g_settingsSaved      = false;
    g_settingsQRReady    = false;
    g_settingsQRUrl      = "";
    g_settingsLastRender = 0;

    // Guard against I2C wedge (same issue as after NimBLE init)
    resetI2CPeripheral();
    if (g_oledReady) g_display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
}

static void exitWifiSettings() {
    // Reboot for a guaranteed-clean radio state. Normal setup() re-inits
    // NimBLE and auto-reconnects the saved keyboard (bond preserved).
    Serial.println("[wifi] exit -> reboot");
    delay(50);
    ESP.restart();
}

static void fallbackToAP() {
    // Reboot into clean WiFi-only AP mode — in-place STA->AP mode switch
    // leaves the PHY in a dirty state (same issue as BLE->WiFi switch).
    Serial.println("[wifi] STA failed -> reboot into AP mode");
    g_prefs.remove("wifi_ssid");
    g_prefs.remove("wifi_pass");
    g_prefs.putBool("to_settings", true);
    delay(50);
    ESP.restart();
}

static void renderSettings() {
    if (!g_oledReady) return;
    if (millis() - g_settingsLastRender < 200) return;
    g_settingsLastRender = millis();

    // Resolve URL
    String url;
    if (g_wifiIsAP) {
        url = "http://192.168.4.1";
    } else {
        if (WiFi.status() == WL_CONNECTED && g_wifiIP.length() == 0)
            g_wifiIP = WiFi.localIP().toString();
        url = g_wifiIP.length() > 0 ? ("http://" + g_wifiIP) : "http://0.0.0.0";
    }

    // Regenerate QR only when URL changes
    if (!g_settingsQRReady || url != g_settingsQRUrl) {
        qrcode_initText(&g_settingsQR, g_settingsQRBuf, 2, ECC_LOW, url.c_str());
        g_settingsQRUrl   = url;
        g_settingsQRReady = true;
    }

    dispClear();

    // QR: 25x25 modules at 1px/module, vertically centered: y=(32-25)/2=3
    drawQR(0, 3);

    // Right panel from x=27
    if (g_wifiIsAP) {
        g_display.setCursor(27, ROW0); g_display.print("HP200LX-Setup");
        g_display.setCursor(27, ROW1); g_display.print("pw:" + g_apPass);
        g_display.setCursor(27, ROW2); g_display.print("192.168.4.1");
        g_display.setCursor(27, ROW3); g_display.print("BACK=exit");
    } else if (g_wifiConnectFailed) {
        g_display.setCursor(27, ROW0); g_display.print("Connect failed");
        g_display.setCursor(27, ROW1); g_display.print("Bad SSID/pass?");
        g_display.setCursor(27, ROW2); g_display.print("WiFi cleared");
        g_display.setCursor(27, ROW3); g_display.print("Press any key");
    } else {
        String ssid = g_prefs.getString("wifi_ssid", "");
        g_display.setCursor(27, ROW0); g_display.print(trunc(ssid, 16));
        if (WiFi.status() == WL_CONNECTED) {
            g_display.setCursor(27, ROW1); g_display.print(g_wifiIP);
            g_display.setCursor(27, ROW2); g_display.print("Settings ready");
        } else {
            int secsLeft = 30 - (int)((millis() - g_wifiConnectStart) / 1000UL);
            if (secsLeft < 0) secsLeft = 0;
            g_display.setCursor(27, ROW1);
            g_display.print("Connecting " + String(secsLeft) + "s");
        }
        g_display.setCursor(27, ROW3); g_display.print("BACK=exit");
    }

    g_display.display();
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
    pinMode(LED_PIN,  OUTPUT); ledWrite(false);
    pinMode(BTN_OK,   INPUT_PULLUP);
    pinMode(BTN_BACK, INPUT_PULLUP);

    Serial.begin(115200);
    delay(2000);
    Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.println("=== ESP32-C3 BLE keyboard -> HP 200LX ===");

    g_prefs.begin("ble-kbd", false);
    racingLoadHi();
    LittleFS.begin(true);

    // WiFi Settings boot path: when requested, boot WITHOUT initializing BLE so
    // WiFi gets a clean radio/PHY. The in-place BLE->WiFi switch leaves the C3
    // PHY unable to transmit beacons (AP stack reports OK but radio is silent).
    if (g_prefs.getBool("to_settings", false)) {
        g_prefs.putBool("to_settings", false);
        Serial.println("[boot] WiFi settings mode (BLE skipped)");
        resetI2CPeripheral();
        g_oledReady = g_display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
        enterWifiSettings();       // BLE never inited; guarded inside
        g_state = STATE_SETTINGS;
        return;
    }

    NimBLEDevice::init("ESP32C3-HP200LX");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    NimBLEDevice::setSecurityAuth(true, false, false);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
    Serial.printf("[ble] own addr=%s\n", NimBLEDevice::getAddress().toString().c_str());

    resetI2CPeripheral();
    g_oledReady = g_display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    if (!g_oledReady) Serial.println("[oled] init failed");

    if (g_oledReady) {
        dispClear();
        drawHeader("ESP32-C3 BRIDGE");
        g_display.setCursor(0, ROW1); g_display.printf("RAM: %dK free", ESP.getFreeHeap()/1024);
        g_display.setCursor(0, ROW2); g_display.print("NVS: ok");
        g_display.setCursor(0, ROW3); g_display.print("Starting...");
        g_display.display();
        delay(1500);

        String savedName = g_prefs.getString("kbd_name", "");
        dispClear();
        if (savedName.length() > 0) {
            drawHeader("AUTO-CONNECT");
            g_display.setCursor(0, ROW1); g_display.print(trunc(savedName));
            g_display.setCursor(0, ROW3); g_display.print("BOTH=cancel to menu");
        } else {
            drawHeader("NO SAVED DEVICE");
            g_display.setCursor(0, ROW2); g_display.print("Starting BLE scan");
            g_display.setCursor(0, ROW3); g_display.print("BOTH=menu anytime");
        }
        g_display.display();
        delay(1500);
    }

    NimBLEScan* scan = NimBLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(&g_scanCB, false);
    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(99);

    String savedAddr = g_prefs.getString("kbd_addr", "");
    if (savedAddr.length() > 0) {
        g_targetAddr    = NimBLEAddress(savedAddr.c_str());
        g_connectedName = g_prefs.getString("kbd_name", "");
    }
    g_state = STATE_SCANNING;
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------
void loop() {
    bool both        = bothPressed();
    bool okPressed   = btnPressed(BTN_OK,   g_btnOkPrev,   g_btnOkLast,   both || g_bothSince > 0);
    bool backPressed = btnPressed(BTN_BACK, g_btnBackPrev, g_btnBackLast, both || g_bothSince > 0);

    // If both-button fires while a transfer is running, abort it cleanly
    if (both && g_kdrvSending) {
        g_kdrvSending = false;
        uint8_t can = XMODEM_CAN;
        Serial1.write(&can, 1);
        Serial1.write(&can, 1);
        g_xFile.close();
    }

    // Both buttons held: toggle system popup from any state
    if (both) {
        if (g_state == STATE_POPUP_MENU || g_state == STATE_PROGRAMS) {
            g_state = g_prevState;
        } else if (g_state == STATE_SETTINGS) {
            exitWifiSettings();
            g_state = g_prevState;
        } else {
            g_prevState   = g_state;
            g_popupCursor = 0;
            g_state       = STATE_POPUP_MENU;
        }
    }

    switch (g_state) {

    // ---- Scanning --------------------------------------------------------
    case STATE_SCANNING: {
        NimBLEScan* scan = NimBLEDevice::getScan();
        if (!scan->isScanning() && !g_scanDone && !g_savedDevFound) {
            Serial.println("[scan] starting");
            scan->start(SCAN_DURATION_S, onScanComplete, false);
        }
        renderScanning();
        if (g_savedDevFound) {
            if (scan->isScanning()) scan->stop();
            g_connectArmed   = false;
            g_scanRetryCount = 0;
            g_state = STATE_CONNECTING;
        } else if (g_scanDone) {
            if (!g_savedDevFound && g_targetAddr != NimBLEAddress() && g_scanRetryCount < 5) {
                Serial.printf("[scan] saved target not found, retry %d/5\n", g_scanRetryCount + 1);
                g_scanRetryCount++;
                g_scanDone   = false;
                g_foundCount = 0;
            } else if (g_foundCount == 0 && g_targetAddr == NimBLEAddress()) {
                g_scanDone = false;
            } else {
                g_scanRetryCount = 0;
                g_listCursor = 0; g_listScroll = 0; g_state = STATE_DEVICE_LIST;
            }
        }
        break;
    }

    // ---- Device list -----------------------------------------------------
    case STATE_DEVICE_LIST: {
        int total = g_foundCount + 1;
        renderDeviceList();
        if (backPressed) { g_listCursor = (g_listCursor + 1) % total; clampListScroll(); }
        if (okPressed) {
            if (g_listCursor == g_foundCount) {
                g_foundCount = 0; g_scanDone = false; g_savedDevFound = false; g_state = STATE_SCANNING;
            } else {
                g_targetAddr    = g_foundAddrs[g_listCursor];
                g_connectedName = g_foundNames[g_listCursor];
                g_connectArmed  = false;
                g_state         = STATE_CONNECTING;
            }
        }
        break;
    }

    // ---- Connecting ------------------------------------------------------
    case STATE_CONNECTING: {
        renderConnecting();
        static uint32_t s_cancelUntil = 0;
        if (!g_connectArmed) {
            g_connectArmed = true;
            uint32_t window = (g_savedFailCount == 0 && g_prefs.getString("kbd_addr","").length() > 0)
                              ? 500 : 1500;
            s_cancelUntil  = millis() + window;
        }
        if (millis() < s_cancelUntil) break;
        g_connectArmed = false;
        if (connectToKeyboard(g_connectedName)) {
            g_connected = true;
            g_savedFailCount = 0;
            g_state = STATE_CONNECTED;
        } else {
            String savedAddr = g_prefs.getString("kbd_addr", "");
            if (savedAddr.length() > 0) {
                g_savedFailCount++;
                if (g_savedFailCount >= 2) {
                    Serial.println("[ble] saved kbd 2x fail — rescanning");
                    g_connectedName = g_prefs.getString("kbd_name", g_connectedName);
                    g_savedFailCount = 0;
                    g_foundCount     = 0;
                    g_scanDone       = false;
                    g_savedDevFound  = false;
                    g_state = STATE_SCANNING;
                } else {
                    g_connectedName = g_prefs.getString("kbd_name", g_connectedName);
                    g_savedDevFound = false;
                    g_scanDone      = false;
                    g_foundCount    = 0;
                    g_state = STATE_SCANNING;
                }
            } else {
                g_state = STATE_ERROR;
            }
        }
        break;
    }

    // ---- Connected -------------------------------------------------------
    case STATE_CONNECTED:
        renderConnected();
        if (backPressed && g_client && g_client->isConnected()) g_client->disconnect();
        break;

    // ---- Error -----------------------------------------------------------
    case STATE_ERROR:
        renderError();
        if (okPressed) { g_foundCount = 0; g_scanDone = false; g_state = STATE_SCANNING; }
        break;

    // ---- System popup menu -----------------------------------------------
    case STATE_POPUP_MENU:
        renderPopupMenu();
        if (backPressed) g_popupCursor = (g_popupCursor + 1) % kPopupCount;
        if (okPressed) {
            switch (g_popupCursor) {
            case 0:
                g_prefs.remove("kbd_addr");
                g_prefs.remove("kbd_name");
                if (g_client && g_client->isConnected()) g_client->disconnect();
                NimBLEDevice::deleteAllBonds();
                g_prefs.remove("kbd_addr");
                g_prefs.remove("kbd_name");
                g_connected     = false;
                g_foundCount    = 0;
                g_scanDone      = false;
                g_savedDevFound = false;
                g_savedFailCount = 0;
                g_targetAddr    = NimBLEAddress();
                g_connectedName = "";
                g_state         = STATE_SCANNING;
                break;
            case 1:
                ESP.restart();
                break;
            case 2:
                g_prevState   = STATE_POPUP_MENU;
                g_progCursor  = 0;
                g_progScroll  = 0;
                g_state       = STATE_PROGRAMS;
                break;
            case 3:
                // Reboot into WiFi-only mode (clean PHY) — see setup()
                g_prefs.putBool("to_settings", true);
                ESP.restart();
                break;
            case 4:
                // Reset all settings to defaults
                g_prefs.remove("wifi_ssid");
                g_prefs.remove("wifi_pass");
                g_prefs.remove("kbd_addr");
                g_prefs.remove("kbd_name");
                if (g_client && g_client->isConnected()) g_client->disconnect();
                NimBLEDevice::deleteAllBonds();
                g_connected      = false;
                g_foundCount     = 0;
                g_scanDone       = false;
                g_savedDevFound  = false;
                g_savedFailCount = 0;
                g_targetAddr     = NimBLEAddress();
                g_connectedName  = "";
                g_state          = STATE_SCANNING;
                break;
            }
        }
        break;

    // ---- Programs --------------------------------------------------------
    case STATE_PROGRAMS: {
        static const char* kBuiltins[] = { "< Back", "Dino Game", "Car Game", "3D Racing", "KBD Driver" };
        static const int   kBCount     = 5;
        static String  fsNames[16];
        static int     fsCount     = 0;
        static bool    needRefresh = true;
        if (needRefresh) {
            fsCount     = 0;
            needRefresh = false;
            File root = LittleFS.open("/");
            File f    = root.openNextFile();
            while (f && fsCount < 16) {
                if (!f.isDirectory()) {
                    String nm = String(f.name());
                    // Hide files owned by KBD Driver program
                    bool hide = false;
                    for (auto* kf : KBDDRV_FILES) {
                        String k = String(kf);
                        if (k.startsWith("/")) k = k.substring(1);
                        if (nm.equalsIgnoreCase(k)) { hide = true; break; }
                    }
                    if (!hide) fsNames[fsCount++] = nm;
                }
                f = root.openNextFile();
            }
        }
        int total = kBCount + fsCount;

        if (g_progCursor < g_progScroll)      g_progScroll = g_progCursor;
        if (g_progCursor >= g_progScroll + 3) g_progScroll = g_progCursor - 2;
        if (g_progScroll < 0)                  g_progScroll = 0;
        if (g_progScroll + 3 > total && total > 0) g_progScroll = max(0, total - 3);

        if (g_oledReady) {
            static uint32_t lastUp = 0;
            if (millis() - lastUp >= 80) {
                lastUp = millis();
                dispClear();
                char hdr[22]; snprintf(hdr, sizeof(hdr), "PROGRAMS (%d)", total - 1);
                drawHeader(hdr);
                static const int rows[] = { ROW1, ROW2, ROW3 };
                for (int row = 0; row < 3; row++) {
                    int idx = g_progScroll + row;
                    if (idx >= total) break;
                    String label = (idx < kBCount) ? kBuiltins[idx] : trunc(fsNames[idx - kBCount]);
                    drawRow(rows[row], label, idx == g_progCursor);
                }
                drawScrollIndicators(g_progScroll, total);
                g_display.display();
            }
        }

        if (backPressed) g_progCursor = (g_progCursor + 1) % total;
        if (okPressed) {
            if (g_progCursor == 0) {
                needRefresh   = true;
                g_state       = STATE_POPUP_MENU;
                g_popupCursor = 2;
            } else if (g_progCursor == 1) {
                initDinoGame();
                g_state = STATE_DINO_GAME;
            } else if (g_progCursor == 2) {
                initCarGame();
                g_state = STATE_CAR_GAME;
            } else if (g_progCursor == 3) {
                g_state = STATE_RACING_LOGO;
            } else if (g_progCursor == 4) {
                g_kdrvCursor = 0;
                g_kdrvScroll = 0;
                g_state      = STATE_KBDDRV_MENU;
            }
        }
        break;
    }

    // ---- Dino game -------------------------------------------------------
    case STATE_DINO_GAME:
        if (okPressed)   g_dinoJumpReq = true;
        if (backPressed) { g_state = STATE_PROGRAMS; break; }
        if (millis() - g_dinoTickLast >= DINO_TICK_MS) {
            g_dinoTickLast = millis();
            updateDino();
            if (g_dinoOver) g_state = STATE_DINO_GAMEOVER;
        }
        renderDinoGame();
        break;

    // ---- Dino game over --------------------------------------------------
    case STATE_DINO_GAMEOVER:
        renderDinoGameOver();
        if (okPressed)   { initDinoGame(); g_state = STATE_DINO_GAME; }
        if (backPressed) { g_state = STATE_PROGRAMS; }
        break;

    // ---- Car physics game ------------------------------------------------
    case STATE_CAR_GAME:
        if (millis() - g_carTickLast >= CAR_TICK_MS) {
            g_carTickLast = millis();
            updateCarGame();
            if (g_carOver) { startCarExplosion(); g_state = STATE_CAR_GAMEOVER; }
        }
        renderCarGame();
        break;

    // ---- Car game over: explosion animation then static screen -----------
    case STATE_CAR_GAMEOVER:
        if (g_explFrame > 0 && g_explFrame <= CAR_EXPL_FRAMES) {
            if (millis() - g_carTickLast >= CAR_TICK_MS) {
                g_carTickLast = millis();
                updateCarExplosion();
            }
            renderCarExplosion();
        } else {
            renderCarGameOver();
            if (okPressed)   { initCarGame(); g_state = STATE_CAR_GAME; }
            if (backPressed) { g_state = STATE_PROGRAMS; }
        }
        break;

    // ---- 3D Racing logo --------------------------------------------------
    case STATE_RACING_LOGO:
        renderRacingLogo();
        if (okPressed)   { racingInitGame(); g_state = STATE_RACING_GAME; }
        if (backPressed) g_state = STATE_PROGRAMS;
        break;

    // ---- 3D Racing game --------------------------------------------------
    case STATE_RACING_GAME:
        // BACK = steer left, OK = steer right (no exit during play)
        if (backPressed) g_racingInputQ |= 1;
        if (okPressed)   g_racingInputQ |= 2;
        {
            // Anti-spiral: cap accumulated debt to 4 ticks after long pause
            if (millis() - g_racingLastMs > (uint32_t)RACING_TICK_MS * 4)
                g_racingLastMs = millis() - RACING_TICK_MS;
            while (millis() - g_racingLastMs >= RACING_TICK_MS) {
                g_racingLastMs += RACING_TICK_MS;
                racingStep();
            }
            if (g_racingOver) {
                racingInitCrash();
                g_state = STATE_RACING_CRASH;
            }
        }
        renderRacingGame();
        break;

    // ---- 3D Racing game over ---------------------------------------------
    case STATE_RACING_GAMEOVER:
        renderRacingGameOver();
        if (okPressed)   { racingInitGame(); g_state = STATE_RACING_GAME; }
        if (backPressed) g_state = STATE_PROGRAMS;
        break;

    case STATE_RACING_CRASH:
        renderRacingCrash();
        break;

    // ---- KBD Driver menu -------------------------------------------------
    case STATE_KBDDRV_MENU:
        if (g_kdrvCursor < g_kdrvScroll)      g_kdrvScroll = g_kdrvCursor;
        if (g_kdrvCursor >= g_kdrvScroll + 3) g_kdrvScroll = g_kdrvCursor - 2;
        if (g_kdrvScroll < 0)                  g_kdrvScroll = 0;
        if (g_kdrvScroll + 3 > KBDDRV_MENU_COUNT)
            g_kdrvScroll = max(0, KBDDRV_MENU_COUNT - 3);

        renderKbdDrvMenu();

        if (backPressed) g_kdrvCursor = (g_kdrvCursor + 1) % KBDDRV_MENU_COUNT;
        if (okPressed) {
            if (g_kdrvCursor == 0) {
                g_state = STATE_PROGRAMS;
            } else if (g_kdrvCursor >= 1 && g_kdrvCursor <= 3) {
                g_kdrvFile = g_kdrvCursor - 1;
                File f = LittleFS.open(KBDDRV_FILES[g_kdrvFile]);
                g_kdrvTotal = f ? f.size() : 0;
                if (f) f.close();
                g_state = STATE_KBDDRV_CONFIRM;
            } else if (g_kdrvCursor == 4) {
                g_infoScroll = 0;
                g_state = STATE_KBDDRV_INFO;
            }
        }
        break;

    // ---- KBD Driver confirm ----------------------------------------------
    case STATE_KBDDRV_CONFIRM:
        renderKbdDrvConfirm();
        if (backPressed) g_state = STATE_KBDDRV_MENU;
        if (okPressed) {
            g_xFile = LittleFS.open(KBDDRV_FILES[g_kdrvFile]);
            if (!g_xFile) {
                // File not found — go back
                g_state = STATE_KBDDRV_MENU;
                break;
            }
            g_xTotalBlocks = (g_kdrvTotal + XMODEM_BLOCK - 1) / XMODEM_BLOCK;
            g_xSentBlocks  = 0;
            g_xBlockNum    = 1;
            g_xRetry       = 0;
            g_xState       = XMDM_WAIT_NAK;
            g_xTimeout     = millis() + 30000;
            g_kdrvSending  = true;
            g_state        = STATE_KBDDRV_SENDING;
        }
        break;

    // ---- KBD Driver sending (XMODEM) -------------------------------------
    case STATE_KBDDRV_SENDING: {
        handleXmodem();
        renderKbdDrvSending();

        if (backPressed) {
            uint8_t can = XMODEM_CAN;
            Serial1.write(&can, 1);
            Serial1.write(&can, 1);
            g_xFile.close();
            g_kdrvSending = false;
            g_state       = STATE_KBDDRV_MENU;
            break;
        }

        if (g_xState == XMDM_DONE || g_xState == XMDM_ERROR) {
            static uint32_t s_msgUntil = 0;
            static bool     s_msgShown = false;
            if (!s_msgShown) {
                s_msgShown = true;
                s_msgUntil = millis() + (g_xState == XMDM_DONE ? 1500 : 2000);
                if (g_oledReady) {
                    dispClear();
                    drawHeader(KBDDRV_LABELS[g_kdrvFile]);
                    g_display.setCursor(20, ROW2);
                    g_display.print(g_xState == XMDM_DONE ? "Done!" : "XMODEM error");
                    g_display.display();
                }
            }
            if (millis() >= s_msgUntil) {
                s_msgShown    = false;
                g_kdrvSending = false;
                g_state       = STATE_KBDDRV_MENU;
            }
        }
        break;
    }

    // ---- KBD Driver info (scrollable) ------------------------------------
    case STATE_KBDDRV_INFO:
        renderKbdDrvInfo();
        if (okPressed)   g_infoScroll = min(g_infoScroll + 1, INFO_LINE_COUNT - 3);
        if (backPressed) g_state = STATE_KBDDRV_MENU;
        break;

    // ---- WiFi Settings ---------------------------------------------------
    case STATE_SETTINGS: {
        renderSettings();
        if (g_webServer) g_webServer->handleClient();
        if (g_wifiConnectFailed) {
            if (okPressed || backPressed) fallbackToAP();
        } else if (!g_wifiIsAP && g_wifiConnectStart) {
            wl_status_t st = WiFi.status();
            uint32_t elapsed = millis() - g_wifiConnectStart;
            Serial.printf("[wifi] connecting st=%d elapsed=%lus\n", st, elapsed/1000);
            bool timedOut = elapsed > 30000UL;
            bool failed   = false;
            if (failed || timedOut) {
                Serial.printf("[wifi] connect fail st=%d timeout=%d\n", st, timedOut);
                g_wifiConnectFailed = true;
                g_wifiConnectStart  = 0;
                g_prefs.remove("wifi_ssid");
                g_prefs.remove("wifi_pass");
            }
        }
        if (g_settingsSaved) {
            // creds already written to NVS; let HTTP response flush, then reboot
            static uint32_t s_until = 0;
            if (!s_until) s_until = millis() + 1500;
            if (millis() >= s_until) ESP.restart();
        }
        if (!g_wifiConnectFailed && backPressed) exitWifiSettings();   // reboots
        break;
    }

    default: break;
    }

    // LED: fast blink when connected, slow when scanning
    static uint32_t lastToggle = 0;
    static bool     ledState   = false;
    uint32_t period = g_connected ? BLINK_CONN_MS : BLINK_SCAN_MS;
    if (millis() - lastToggle >= period) {
        lastToggle = millis();
        ledState = !ledState;
        ledWrite(ledState);
    }

    delay(10);
}
