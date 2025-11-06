# M5 OUI‐Spy

M5Stack ATOM (Lite / ATOM GPS) Unified (beta) detector firmware for multi‐target BLE and Wi‐Fi with GPS+SD logging and Web configuration.

![image](https://github.com/user-attachments/assets/14f7ec00-2bfe-4ad6-944a-362dbbdd67d4)

Related fork: [Fox Hunter variant](https://github.com/lukeswitz/ouispy-foxhunter/tree/main/M5_Atom_Foxhunt)

---

## Hardware

- **Boards:**
  - ATOM Lite (ESP32, 1x NeoPixel) - Unmaintained FW
  - ATOM GPS (ESP32 with integrated GPS; equivalent to ATOM Lite + Atomic GPS) - Recommended

---

## Software / Build

- **Board:** M5Stack‐ATOM (ESP32)
- **Libraries:**
  - M5Atom
  - TinyGPSPlus
  - FastLED
  - SD, SPI
  - NimBLE‐Arduino 2.3.4
  - ESPAsyncWebServer v3.8.0, AsyncTCP
- **Build:** Arduino IDE or PlatformIO

---

## What It Does

- Scans BLE advertisements and Wi‐Fi frames
- Matches devices by OUI (first 3 bytes) or full MAC (BLE or Wi‐Fi)
- Logs matched events with UTC and GPS to CSV on SD
- Web portal via SoftAP to add/remove filters

**CSV format:**
```csv
WhenUTC,MatchType,MAC,RSSI,Lat,Lon,AltM,HDOP,Filter
```

---

## Installation

### Prerequisites

1. **Install Arduino IDE**
   - Download from [arduino.cc](https://www.arduino.cc/en/software)
   - Version 1.8.19 or 2.x recommended

2. **Add M5Stack Board Support**
   - Open Arduino IDE
   - Navigate to **File → Preferences**
   - Add this URL to **Additional Boards Manager URLs:**
```
     https://static-cdn.m5stack.com/resource/arduino/package_m5stack_index.json
```
   - Alternative URL (if primary fails):
```
     https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/arduino/package_m5stack_index.json
```
   - Click **OK**
   - Go to **Tools → Board → Boards Manager**
   - Search for "M5Stack"
   - Install **M5Stack by M5Stack official**

3. **Install Required Libraries**
   - Go to **Sketch → Include Library → Manage Libraries** (or **Tools → Manage Libraries** in IDE 2.x)
   - Install each of the following:
     - `M5Atom` by M5Stack
     - `TinyGPSPlus` by Mikal Hart
     - `FastLED` by Daniel Garcia
     - `NimBLE-Arduino` by h2zero (version 2.3.4)
     - `ESPAsyncWebServer` by dvarrel (version 3.8.0)
     - `AsyncTCP` by dvarrel
   - [fact] SD and SPI libraries are included with ESP32 core

---

## Flashing

### Method 1: Arduino IDE

1. **Choose Firmware File:**
   - For **ATOM GPS:** Open `AtomGPS/AtomGPS.ino`
   - For **ATOM Lite:** Open `Atom/Atom.ino`

2. **Configure Board:**
   - **Tools → Board → M5Stack Arduino → M5Stack-ATOM**
   - **Tools → Upload Speed:** 921600 (or 115200 if upload fails)
   - **Tools → Port:** Select your device port
     - Windows: COMx
     - macOS: /dev/cu.usbserial-*
     - Linux: /dev/ttyUSB*

3. **Upload:**
   - Click **Verify** (✓) to compile
   - Click **Upload** (→) to flash
   - Open **Tools → Serial Monitor** at 115200 baud to verify operation

### Method 2: PlatformIO

1. **Install PlatformIO:**
   - VSCode: Install "PlatformIO IDE" extension
   - CLI: `pip install platformio`

2. **Create platformio.ini in project root:**
```ini
   [env:m5stack-atom]
   platform = espressif32
   board = m5stack-atom
   framework = arduino
   lib_deps = 
       m5stack/M5Atom
       mikalhart/TinyGPSPlus
       fastled/FastLED
       h2zero/NimBLE-Arduino@2.3.4
       dvarrel/ESPAsyncWebServer@3.8.0
       dvarrel/AsyncTCP
   monitor_speed = 115200
```

3. **Build and Upload:**
```bash
   pio run --target upload
   pio device monitor
```

---

## Quick Start

1. **Prepare Hardware:**
   - Format microSD card as FAT32
   - Insert card into device
   - Power on

2. **Wait for GPS Lock:**
   - Blue LED pulses (faster = more satellites)
   - Purple blink indicates GPS fix acquired

3. **Connect to Configuration Portal:**
   - **SSID:** `snoopuntothem`
   - **Password:** `astheysnoopuntous`
   - Navigate to `192.168.4.1` (or IP shown in Serial Monitor)

4. **Add Target Filters:**
   - **OUI (3 bytes):** `58:2D:34`
   - **Full MAC (6 bytes):** `58:2D:34:12:AB:CD`
   - Click **Save**

5. **Operation:**
   - Device scans BLE/Wi‐Fi for targets
   - Logs matches to `/OUISPY-YYYY-MM-DD-N.csv`

---

## LED Indicators

| LED Behavior         | Meaning                                           |
|----------------------|---------------------------------------------------|
| Blue Pulse           | Waiting for GPS fix (faster = more satellites)    |
| Orange Pulse         | Web configuration mode active                     |
| Purple Blink (~15s)  | Scanning heartbeat                                |
| Green Blink ×3       | New target detected or ≥30s re‐detect            |
| Red Blink ×2         | Target re‐detected after ≥5s                      |

---

## Notes and Tips

- Experiment with BLE scanning modes (passive/active) for performance balance
- Adjust Wi‐Fi channel dwell time vs BLE scan intervals for your use case
- Use 32GB or smaller SD cards; larger cards require longer format times

---

## Troubleshooting

| Issue                  | Solution                                                      |
|------------------------|---------------------------------------------------------------|
| SD init failed         | Format card as FAT32 (not exFAT); verify card inserted        |
| No GPS fix             | Move outdoors; clear sky view; cold start takes 30s–2min      |
| Cannot connect to AP   | Verify SSID/password; check Serial Monitor for actual IP      |
| Compilation errors     | Verify library versions; update ESP32 core to latest          |
| Upload failed          | Try lower upload speed (115200); check USB cable/port         |

---
