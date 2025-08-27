# M5 OUI‑Spy 
### (BLE + Wi‑Fi) — M5Stack ATOM (Lite / ATOM GPS)

Unified (beta) detector firmware for multi‑target BLE and Wi‑Fi with GPS+SD logging and Web configuration.

Related fork [Fox Hunter variant](https://github.com/lukeswitz/ouispy-foxhunter/tree/main/M5_Atom_Foxhunt)

---

## Hardware

- Boards:
  - ATOM Lite (ESP32, 1x NeoPixel) - Unmaintained FW
  - ATOM GPS (ESP32 with integrated GPS; equivalent to ATOM Lite + Atomic GPS) - Recommended

---

## Software / Build

- Board: M5Stack‑ATOM (ESP32)
- Libraries:
  - M5Atom
  - TinyGPSPlus
  - FastLED
  - SD, SPI
  - NimBLE‑Arduino 2.3.4
  - ESPAsyncWebServer v3.8.0, AsyncTCP
- Build in Arduino IDE or PlatformIO

---

## What It Does

- Scans BLE advertisements and Wi‑Fi frames
- Matches devices by OUI (first 3 bytes) or full MAC (BLE or Wi‑Fi)
- Logs matched events with UTC and GPS to CSV on SD
- Web portal via SoftAP to add/remove filters

CSV format:
```csv
WhenUTC,MatchType,MAC,RSSI,Lat,Lon,AltM,HDOP,Filter
```

---

## Quick Start

1) Insert FAT32 microSD, power on  
2) Wait for GPS (purple LED blink; speeds up with satellites)  
3) Connect to AP:
   - SSID: `snoopuntothem`
   - Password: `astheysnoopuntous`
   - Open the IP printed to Serial to access Web UI
4) Add filters and Save:
   - OUI: `58:2D:34`
   - Full MAC: `58:2D:34:12:AB:CD`
5) Device scans; new/rediscovered matches blink and log to:
   - `/OUISPY-YYYY-MM-DD-N.csv`

---


## LED Indicators

| LED Behavior | Meaning |
|--------------|---------|
| Purple Blink | Scanning heartbeat |
| Blue Blink x2 | ≥ 5s re‑detect|
| Orange Pulse | Web config mode |
| Red Blink x3 | Target acquired for the first time or ≥ 30s re‑detect |
| Blue Pulse | Waiting for GPS fix (pulses faster with more satellites) |


## LED Indicators

- Orange Pulse: Web configuration mode
- Blue Pulse: Waiting for GPS
- Purple Blink (≈15 s): Scanning heartbeat
- Green Rapid ×3: New match detected 
- Blue Blink x2: ≥ 5s re‑detect
---

## Notes and Tips

- Experiment with BLE scanning passive/active; set modest Wi‑Fi dwell for balance
---

## Troubleshooting

- SD init failed:
   - Format the card as FAT32
  
---
