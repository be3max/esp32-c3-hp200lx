# ESP32-C3 BLE Keyboard Bridge for HP 200LX

Connects a modern Bluetooth HID keyboard to an HP 200LX palmtop via UART/MAX3232. The ESP32-C3 acts as a BLE central (HID host), receives keystrokes, and forwards them as 2-byte packets over serial. Includes OLED UI, mini-games, and an XMODEM file transfer tool for installing the keyboard driver on the HP 200LX.

---

## Features

- **BLE HID host**: pairs and reconnects to any Bluetooth keyboard (BLE HID/HoG profile)
- **HP 200LX serial bridge**: forwards keystrokes as 2-byte `[event, keycode]` packets at 9600 8N1
- **OLED UI**: 128×32 display with scrollable menus, status, key event feedback
- **Auto-reconnect**: remembers last keyboard by identity address; skips 15 s scan on disconnect
- **WiFi settings**: built-in web interface for configuring WiFi and viewing device info, accessed via QR code
- **KBD Driver transfer**: sends `KBDSER2.COM` / `LOAD2.BAT` / `RESTORE.BAT` to HP 200LX via XMODEM
- **Programs menu**: built-in games plus LittleFS-stored executables
- **Games**: Dino jump, Car physics, 3D pseudo-3D racing with hi-score saved to NVS

---

## Bill of Materials

| Qty | Component | Notes |
|-----|-----------|-------|
| 1 | **ESP32-C3 SuperMini** | Main MCU, USB-C, onboard LED on GPIO8 |
| 1 | **SSD1306 OLED 128×32** | I2C, 3.3 V, 4-pin (GND/VCC/SCL/SDA) |
| 1 | **MAX3232 module** | TTL ↔ RS-232 level shifter; 3.3 V compatible; 4× 100 nF caps usually on-board |
| 1 | **DE-9 female connector** | Connects to HP 200LX serial port (COM1) |
| 2 | **Tactile pushbutton** | 6×6 mm or similar, to GND |
| 1 | **USB-C cable / 5 V supply** | Powers the ESP32-C3 |
| — | **Hookup wire, PCB / perfboard** | For assembly |

> **MAX3232 vs MAX232**: Use MAX3232 (3.3 V logic). MAX232 is 5 V only and will damage the ESP32.

---

## Wiring

```
ESP32-C3 SuperMini        SSD1306 OLED 128×32
GPIO4  (SDA) ──────────── SDA
GPIO5  (SCL) ──────────── SCL
3V3          ──────────── VCC
GND          ──────────── GND

ESP32-C3 SuperMini        MAX3232 module           HP 200LX DE-9
GPIO21 (TX1) ──────────── T1IN  →  T1OUT ────────── pin 2  (RX)
GPIO20 (RX1) ──────────── R1OUT ← R1IN  ────────── pin 3  (TX)
3V3          ──────────── VCC
GND          ──────────────────────────────────────  pin 5  (GND)

ESP32-C3 SuperMini        Buttons
GPIO9  ── BACK button ── GND   (INPUT_PULLUP, active LOW)
GPIO10 ── OK   button ── GND   (INPUT_PULLUP, active LOW)

GPIO8  ── onboard LED (active LOW, built into SuperMini)
```

### HP 200LX DE-9 pinout (DTE, male on palmtop)

| Pin | Signal | Connect to |
|-----|--------|-----------|
| 2 | RX (into HP) | MAX3232 T1OUT |
| 3 | TX (from HP) | MAX3232 R1IN |
| 5 | GND | common GND |

The HP 200LX serial port is DB-9 DTE male. Use a DE-9 female connector on your cable.  
Only RX, TX, and GND are needed — no hardware flow control required at 9600 8N1.

### Assembly notes

- Both buttons connect between their GPIO pin and GND. Internal pull-ups are enabled in firmware — no external resistors needed.
- The MAX3232 module typically has the four 100 nF charge-pump capacitors on-board. If using a bare MAX3232 IC, add 4× 100 nF between C1+/C1−, C2+/C2−, V+/GND, and V−/GND.
- Power the MAX3232 from the ESP32-C3's 3V3 pin (not 5 V).
- Keep the UART wires short. Runs fine up to ~30 cm without shielding.

---

## UI Controls

| Input | Action |
|-------|--------|
| **OK** (GPIO10) | Select / confirm / scroll down in lists |
| **BACK** (GPIO9) | Back / cancel / scroll up in lists |
| **Both held 300 ms** | Open system menu from any screen |

---

## Serial Packet Format

Each key event sent to the HP 200LX is exactly 2 bytes:

```
[0x01, keycode]  — key down
[0x02, keycode]  — key up
```

Keycodes are USB HID boot keyboard scan codes. Modifier keys (Ctrl, Shift, Alt, Win, etc.) are sent as individual keycodes `0xE0`–`0xE7`.

---

## BLE Pairing

1. Power on the ESP32 — it scans automatically for 15 s
2. Put your Bluetooth keyboard in pairing mode
3. Select the keyboard from the discovered device list (OK to select)
4. Pairing happens automatically; bond is saved to NVS
5. On subsequent boots the bridge auto-reconnects without showing the list

**To forget the keyboard and re-pair:** hold both buttons → select **Scan for devices**.  
**To re-pair without forgetting:** hold both buttons → **Restart** — the bridge will reconnect on the next scan cycle.

---

## Installing the Keyboard Driver on HP 200LX

The firmware includes a built-in XMODEM file sender that transfers `KBDSER2.COM`, `LOAD2.BAT`, and `RESTORE.BAT` to the HP 200LX over the same serial connection used for keystrokes.

### What the files do

| File | Purpose |
|------|---------|
| `KBDSER2.COM` | TSR: reads 2-byte `[event, keycode]` packets from COM1 at 9600 8N1 and injects them into the BIOS keyboard buffer. ~1 KB resident. Flags: `/G` game mode, `/D` debug, `/U` unload |
| `LOAD2.BAT` | Unloads then reloads the driver. Run from A: drive. `/G` for game mode, `/G /D` with counter |
| `RESTORE.BAT` | Rewrites `CONFIG.SYS` and `AUTOEXEC.BAT` to auto-load the driver on boot, then reboots the HP 200LX |

Install all three files to the HP 200LX **A: drive** (internal flash — survives battery removal).

### Transfer procedure (XMODEM)

#### On HP 200LX

1. Open **DataComm** (built-in terminal emulator: `Ctrl+Alt+D` from System Manager, or via the Applications menu)
2. Set port to COM1, 9600 baud, 8N1, no flow control (these are DataComm defaults)
3. Start a receive: **File → Receive → XMODEM** and enter the destination filename (e.g. `A:\KBDSER2.COM`)
4. DataComm begins waiting (sends a NAK byte to signal readiness)

#### On the bridge

1. Hold both buttons → **Programs** → **KBD Driver**
2. Select the file to send (e.g. **Send KBDSER2.COM**)
3. Confirm the file size shown, then press **OK** to begin
4. The OLED shows a progress bar with block count
5. Transfer completes in a few seconds; OLED shows "Done!"
6. Repeat for `LOAD2.BAT` and `RESTORE.BAT`

#### First-time setup on HP 200LX

After transferring all three files, run `RESTORE.BAT` from the HP 200LX command prompt:

```dos
A:\RESTORE.BAT
```

This rewrites `CONFIG.SYS` and `AUTOEXEC.BAT` to auto-load `KBDSER2.COM` on every boot. The palmtop reboots automatically. After reboot, keystrokes from the BLE keyboard appear on the HP 200LX.

> **Note:** The bridge's UART TX is shared between keyboard forwarding and XMODEM. Do not attempt a transfer while keystrokes are being typed.

### Sending your own `.EXE` / `.COM` files

The same XMODEM path works for any file you put in the LittleFS filesystem. To add files:

1. Place files in the `data/` directory of this project
2. Flash the filesystem: `pio run -t uploadfs`
3. Files appear in the **Programs** menu (files owned by KBD Driver are hidden there but visible in LittleFS)

To receive on the HP 200LX: use DataComm's **File → Receive → XMODEM** as described above.  
Alternative: use any XMODEM-capable terminal (HyperTerminal, minicom, TERATERM) connected to the same COM port on a PC — the protocol is standard.

---

## WiFi Settings

A built-in web interface lets you configure WiFi credentials and view the paired keyboard name.

### Access

1. Hold both buttons → **Settings** → OK
2. Device reboots into WiFi-only mode (BLE is not started — required on ESP32-C3 because BLE and WiFi share one radio)
3. Creates access point **`HP200LX-Setup`** with an 8-character auto-generated password shown on the OLED
4. OLED shows a QR code and `192.168.4.1`
5. Connect a phone or laptop to `HP200LX-Setup`, scan the QR code or open `http://192.168.4.1`
6. Enter WiFi SSID and password → **Save & Reconnect**

### After saving credentials

- Next time Settings mode starts, the device connects to your network (STA mode) instead of creating an AP
- OLED shows the network name, assigned IP, and a QR code for direct browser access

### Exit Settings

Press **BACK** — device reboots back to BLE bridge mode. The paired keyboard reconnects automatically.

---

## Programs Menu & Games

Access via: hold both buttons → **Programs**

| Entry | Description |
|-------|-------------|
| **Dino Game** | Side-scrolling jump game. OK = jump. Obstacle speed increases with score. |
| **Car Game** | Physics-based car driving. OK = throttle forward, BACK = reverse. Terrain is procedurally generated with increasing difficulty. Wheelie/flip = crash. Distance is the score. |
| **3D Racing** | Pseudo-3D perspective racer. BACK = steer left, OK = steer right. 3 lanes, curved road, roadside scenery. Score = opponents overtaken. Hi-score saved to NVS. |
| **KBD Driver** | XMODEM file transfer tool — see above. |
| **File Sync** | WiFi file manager for the HP 200LX filesystem — see below. |
| *(LittleFS files)* | Any `.COM`/`.EXE`/other files in `data/` appear here (not yet launched — future feature). |

---

## File Sync (browse HP 200LX files over WiFi)

Manage the HP 200LX filesystem from a phone or PC browser — list, download,
upload, delete, rename, and create/remove folders. The ESP32 bridges your WiFi
to the HP 200LX serial link by speaking the **Kermit** protocol to MS-Kermit
running in server mode; no custom DOS program is required.

> Because the ESP32-C3 shares one radio between BLE and WiFi, File Sync reboots
> into a WiFi-only mode (like Settings). The BLE keyboard bridge is **not**
> active while syncing. Press **BACK** (or hold both buttons) to reboot back to
> normal keyboard mode.

**On the HP 200LX (one-time per session):**

1. Connect the serial cable (MAX3232 → COM1) as for keyboard use.
2. At the DOS prompt run Kermit and put it in server mode:
   ```
   kermit
   SET PORT COM1
   SET SPEED 9600
   SET FLOW NONE
   SET FILE TYPE BINARY
   SERVER
   ```
   (You can drop `kermit.exe` plus a `KERMSRV.BAT` shortcut onto the HP using
   the **KBD Driver** XMODEM transfer.)

**On the ESP32:**

1. Make sure WiFi credentials are saved (**Settings** → Save). With no creds,
   File Sync bounces you into the Settings AP first.
2. Hold both buttons → **Programs → File Sync**. The device reboots, joins your
   WiFi, and the OLED shows the URL + a QR code.
3. Open `http://<ip>` in a browser. Browse folders, **get**/**ren**/**del**
   files, **Upload**, and create/remove folders.

Notes: the link is 9600 baud (~960 B/s) so transfers are slow; one operation
runs at a time. Uploads are spooled through LittleFS, so upload size is capped
by free flash. Use DOS 8.3 names.

---

## Build & Flash

Requires [PlatformIO](https://platformio.org/).

```powershell
pio run                                          # compile only
pio run -t upload                                # compile + flash
pio run -t uploadfs                              # flash LittleFS filesystem (data/ dir)
pio device monitor --port COM6 --baud 115200     # serial debug output
```

> Close the serial monitor before flashing — the USB CDC port is shared.  
> Port is COM6 on the development machine; adjust for your system.

---

## Dependencies

| Library | Version |
|---------|---------|
| `h2zero/NimBLE-Arduino` | ^1.4.2 |
| `adafruit/Adafruit SSD1306` | ^2.5.7 |
| `adafruit/Adafruit GFX Library` | ^1.11.9 |
| `ricmoo/QRCode` | ^0.0.1 |
| LittleFS | built-in to ESP32 Arduino SDK |
| WiFi, WebServer | built-in to ESP32 Arduino SDK |
