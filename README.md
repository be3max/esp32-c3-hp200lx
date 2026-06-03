# ESP32-C3 BLE Keyboard Bridge for HP 200LX

Connects a modern Bluetooth HID keyboard to an HP 200LX palmtop via UART/MAX3232. The ESP32-C3 acts as a BLE central (HID host), receives keystrokes, and forwards them as 2-byte packets over serial.

## Hardware

| Component | Detail |
|-----------|--------|
| MCU | ESP32-C3 SuperMini |
| Display | SSD1306 OLED 128×32, I2C |
| Serial bridge | MAX3232 TTL↔RS-232 level shifter |
| Target | HP 200LX palmtop, 9600 8N1 |

### Wiring

```
ESP32-C3          OLED SSD1306
GPIO4  ────────── SDA
GPIO5  ────────── SCL
3V3    ────────── VCC
GND    ────────── GND

ESP32-C3          MAX3232         HP 200LX
GPIO21 (TX) ───── T1IN → T1OUT ── RX (DB9 pin 2)
GPIO20 (RX) ───── R1OUT ← R1IN ── TX (DB9 pin 3)
GND    ─────────────────────────── GND (DB9 pin 5)

ESP32-C3
GPIO9  ── BACK button (to GND)
GPIO10 ── OK button (to GND)
GPIO8  ── LED (active LOW)
```

## UI Controls

- **OK** (GPIO10) — select / confirm / scroll down
- **BACK** (GPIO9) — back / cancel / scroll up
- **Both held 300ms** — open system menu from any screen

## Serial Packet Format

Each key event sent to HP 200LX is 2 bytes:

```
[0x01, keycode]  — key down
[0x02, keycode]  — key up
```

Keycodes are USB HID boot keyboard codes. Modifiers (Ctrl, Shift, Alt, etc.) are sent as individual keycodes `0xE0`–`0xE7`.

## BLE Pairing

1. Power on ESP32 — it scans automatically
2. Put keyboard in pairing mode
3. Select keyboard from list (or auto-connects if previously paired)
4. After first pair, reconnects automatically on subsequent boots

To forget a keyboard and re-pair: hold both buttons → **Scan for devices**.

## Build & Flash

Requires [PlatformIO](https://platformio.org/).

```powershell
pio run              # compile
pio run -t upload    # flash
pio device monitor --port COM6 --baud 115200   # debug serial
```

## WiFi Settings Page

A built-in web interface lets you configure WiFi credentials and view device info from any browser on the local network.

### How to use

1. Hold both buttons → **Settings** → OK
2. Device reboots into WiFi-only mode and creates an access point:
   - **SSID**: `HP200LX-Setup`
   - **Password**: `12345678`
3. OLED shows a QR code and the IP address (`192.168.4.1`)
4. Connect phone/laptop to `HP200LX-Setup`, scan the QR code or open `http://192.168.4.1`
5. Enter your home WiFi SSID and password → **Save & Reconnect**
6. Device reboots; re-open Settings to see STA IP and QR code for direct access

### After WiFi credentials are saved

- Settings mode connects to your network (STA mode) instead of creating an AP
- OLED shows the network name, assigned IP, and a QR code
- Access the settings page from any device on the same network

### Exiting Settings

Press **BACK** — device reboots back to normal BLE keyboard bridge mode. The paired keyboard reconnects automatically (bond is preserved).

## Dependencies

- `h2zero/NimBLE-Arduino` ^1.4.2
- `adafruit/Adafruit SSD1306` ^2.5.7
- `adafruit/Adafruit GFX Library` ^1.11.9
- `ricmoo/QRCode` ^0.0.1
- LittleFS (built-in, for Programs menu storage)
- WiFi, WebServer (built-in to ESP32 Arduino SDK)
