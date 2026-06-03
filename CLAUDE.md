# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Flash Commands

```powershell
pio run                          # compile only
pio run -t upload                # compile + flash (closes serial monitor first)
pio run -t uploadfs              # flash LittleFS filesystem (data/ directory)
pio device monitor --port COM6 --baud 115200   # serial debug output
```

The port is COM6 on this machine. Serial monitor must be closed before flashing (USB CDC conflict).

## Hardware

- **MCU**: ESP32-C3 SuperMini
- **OLED**: SSD1306 128x32, I2C on GPIO4(SDA)/GPIO5(SCL), address 0x3C
- **Buttons**: GPIO9=BACK, GPIO10=OK (active LOW, INPUT_PULLUP); both held 300ms = system menu
- **UART**: Serial1 GPIO20(RX)/GPIO21(TX) → MAX3232 → HP 200LX at 9600 8N1
- **Debug**: Serial (USB-CDC) at 115200
- **LED**: GPIO8, active LOW

## Architecture

Everything lives in `src/main.cpp` — single-file firmware with these layers:

### State Machine (`AppState` enum)
`BOOT → SCANNING → CONNECTING → CONNECTED`  
Alternative branches:
- `DEVICE_LIST` — manual BLE device selection after scan
- `ERROR` — connect failure screen
- `POPUP_MENU` / `PROGRAMS` — overlays accessible from any state via both-buttons
- `DINO_GAME` / `DINO_GAMEOVER` — side-scrolling jump game
- `CAR_GAME` / `CAR_GAMEOVER` — physics-based car driving (OK=throttle, BACK=reverse)
- `RACING_LOGO` / `RACING_GAME` / `RACING_CRASH` / `RACING_GAMEOVER` — pseudo-3D racer (BACK=left, OK=right)
- `KBDDRV_MENU` / `KBDDRV_CONFIRM` / `KBDDRV_SENDING` / `KBDDRV_INFO` — XMODEM file transfer to HP 200LX
- `STATE_SETTINGS` — WiFi settings mode (WiFi-only boot path)

`g_state` drives `loop()` via switch. `g_prevState` used by popup overlay to restore state on dismiss.

### BLE Connection Flow
1. **Scan**: `NimBLEScan` runs for `SCAN_DURATION_S=15s`. `ScanCB::onResult()` matches saved keyboard by address OR name OR HID service UUID (0x1812) — needed because bonded keyboards advertise without name (RPA privacy mode).
2. **Connect**: `connectToKeyboard()` is blocking. Called after 500ms (reconnect) or 1500ms (first connect) cancel window in `STATE_CONNECTING`. Uses `NimBLEClient` created once, reused across reconnects.
3. **Secure**: `secureConnection()` handles BLE bonding. After success, saves peer identity address (`g_client->getPeerAddress()`) to NVS — critical because scanned RPA differs from stable identity address.
4. **HID**: subscribes to all notifiable characteristics on service 0x1812, forwards 8-byte HID boot keyboard reports to HP 200LX via Serial1 as 2-byte packets `[event, keycode]` (0x01=down, 0x02=up).
5. **Disconnect**: `ClientCB::onDisconnect()` skips scan and goes directly to `STATE_CONNECTING` if `g_targetAddr` is known — avoids 15s scan delay during which keyboard stops advertising.

### Critical BLE Quirks (hard-won)
- **`periph_module_reset(PERIPH_I2C0_MODULE)`** must be called after `NimBLEDevice::init()` — NimBLE wedges the I2C0 FSM on ESP32-C3 (arduino-esp32 issue #8454). See `resetI2CPeripheral()`.
- **Never call `NimBLEDevice::deleteClient()`** immediately before `createClient()` — corrupts NimBLE internal state. Client is created once (`g_client`) and reused.
- **Never call `deleteAllBonds()`** during reconnect — destroys the IRK needed to resolve the keyboard's RPA. Only call on user-initiated "Scan for devices" (popup menu item 0).
- **Peer identity address vs scan address**: after `secureConnection()`, save `g_client->getPeerAddress()` not `g_targetAddr` — the scan address is an unresolved RPA that changes; identity address is stable.

### Critical WiFi Quirks (hard-won)
- **BLE and WiFi share one 2.4GHz radio on ESP32-C3.** Calling `WiFi.mode(WIFI_OFF)` while NimBLE holds the radio causes `wifi:timeout when WiFi un-init` errors and leaves the WiFi driver corrupted — `softAP()` returns `true` and `ip=192.168.4.1` but never transmits a beacon.
- **Solution: WiFi-only boot path.** Settings popup sets NVS `to_settings=true` and calls `ESP.restart()`. `setup()` detects the flag, skips `NimBLEDevice::init()` entirely, and calls `enterWifiSettings()` directly — WiFi gets a clean PHY with no BLE interference.
- **ESP32-C3 SuperMini TX power**: at default 19.5 dBm the weak onboard regulator browns out and the beacon never radiates (AP stack reports OK). Fix: `WiFi.setTxPower(WIFI_POWER_8_5dBm)` before `softAP()`.
- **Never call `WiFi.mode(WIFI_OFF)`** unless WiFi was actually started (`WiFi.getMode() != WIFI_MODE_NULL`). Calling it when WiFi was never initialized also triggers the un-init timeout.
- **Exit Settings = reboot** (`ESP.restart()`). This is intentional — cleanly returns the radio to BLE without any driver teardown ordering issues. Bond data is preserved (`deinit(false)`).

### NVS Storage (`Preferences`, namespace `"ble-kbd"`)
- `kbd_addr` — keyboard identity address (post-secureConnection peer addr)
- `kbd_name` — keyboard display name
- `wifi_ssid` — saved WiFi SSID for Settings STA mode
- `wifi_pass` — saved WiFi password for Settings STA mode
- `to_settings` — bool flag: set before reboot to enter WiFi-only settings boot path
- `hi_racing` — 3D Racing high score

### Display Layout (128×32, text size 1 = 6×8px, 21 chars wide)
- ROW0=0, ROW1=8, ROW2=16, ROW3=24
- `drawHeader()` — inverted (white bg, black text) full-width bar on ROW0
- `drawRow()` — inverted when selected, normal otherwise
- All render functions are rate-limited (~80ms) and idempotent

### Programs Menu
Built-in entries: `< Back`, `Dino Game`, `Car Game`, `3D Racing`, `KBD Driver`. LittleFS files in `data/` appear as additional entries (KBD Driver's own files are hidden from this list). Flash filesystem with `pio run -t uploadfs`.

### KBD Driver / XMODEM Transfer
`STATE_KBDDRV_MENU` lists 3 sendable files (`KBDSER2.COM`, `LOAD2.BAT`, `RESTORE.BAT`) stored in LittleFS.  
`STATE_KBDDRV_CONFIRM` shows file size and waits for user to start DataComm XMODEM receive on HP 200LX.  
`STATE_KBDDRV_SENDING` runs a non-blocking XMODEM state machine (`handleXmodem()`) via `g_xState`. States: `XMDM_WAIT_NAK → XMDM_SEND_BLOCK → XMDM_WAIT_ACK → XMDM_SEND_EOT → XMDM_WAIT_EOT_ACK → XMDM_DONE`. Progress bar shows `g_xSentBlocks / g_xTotalBlocks`. Both-buttons or BACK sends CAN bytes and aborts.  
XMODEM uses Serial1 (same UART as keyboard forwarding) — do not type during transfer.

### Games
- **Dino**: fixed-position runner; obstacles scroll right→left; speed ramps with score.
- **Car**: ring-buffer terrain (`g_carTBuf[512]`), physics with pitch/wheelSpin/gravity; crash on flip (`>1.65 rad`) or landing inverted; procedural difficulty ramp. Explosion animation on crash.
- **3D Racing**: perspective projection via `racingProjY()` / `racingRoadHalf()` / `racingCenterX()`; 4 opponents, 3 lanes, curve system, roadside scenery (grass/bush/tree/building); off-road = spin penalty. Player car is wireframe 3D model (`kCarVerts[20]`, `kCarEdges[24]`). Hi-score in NVS key `hi_racing`.

## Key Globals

| Global | Purpose |
|--------|---------|
| `g_client` | Single NimBLE client, created once, never deleted |
| `g_targetAddr` | Current keyboard address (updated to identity addr after pair) |
| `g_connectedName` | Display name; must not be overwritten with error strings on retry |
| `g_savedDevFound` | Set by ScanCB when saved keyboard seen; triggers immediate connect |
| `g_connectArmed` | Guards the non-blocking cancel window in STATE_CONNECTING |
| `g_savedFailCount` | Consecutive connect failures; resets on success |
| `g_scanRetryCount` | Auto-retry scan count when saved target not found (max 5) |
