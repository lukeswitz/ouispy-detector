#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <NimBLEScan.h>
#include <NimBLEDevice.h>
#include <NimBLEAdvertisedDevice.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>
#include <nvs_flash.h>
#include <vector>
#include <algorithm>
#include <M5Atom.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPS++.h>

// FreeRTOS task handles
TaskHandle_t LEDTaskHandle = NULL;
TaskHandle_t ScanTaskHandle = NULL;
const size_t MAX_DEVICES = 100;

// Hardware Configuration - M5 Atom Lite
#define LED_PIN 27
#define LED_BRIGHTNESS 80
#define ATOM_LEDS 1

// GPS
TinyGPSPlus gps;
static const int SD_CS = 15;
static const uint32_t GPS_BAUD = 9600;
bool sdReady = false;
char fileName[64];
bool REQUIRE_GPS_FIX = true;  // set false to skip blocking wait or press button

// WiFi AP Configuration
String AP_SSID = "snoopuntothem";
String AP_PASSWORD = "astheysnoopuntous";
#define CONFIG_TIMEOUT 20000  // 20 seconds timeout for config mode

// Operating Modes
enum OperatingMode {
  CONFIG_MODE,
  SCANNING_MODE
};

// LED Color Reference (GRB format for M5Atom)
// ==========================================
// Bright Colors:
// 0xFFFFFF - White
// 0xFF0000 - Blue
// 0x00FF00 - Green
// 0x0000FF - Red
// 0xFFFF00 - Cyan
// 0x00FFFF - Yellow
// 0xFF00FF - Magenta
// 0xA5FF00 - Orange
//
// Dim Colors (good for status blinks):
// 0x404040 - Warm White
// 0x400040 - Dim Blue
// 0x004000 - Dim Green
// 0x000040 - Dim Red
// 0x401040 - Dim Purple
// 0x404020 - Dim Teal
// 0x402040 - Dim Magenta
// 0x204020 - Dim Yellow-Green
// 0x602010 - Dim Orange
// 0x301030 - Very Dim Purple

// LED Pattern States
enum LEDPattern {
  LED_OFF,
  LED_STATUS_BLINK,
  LED_SINGLE_BLINK,  // Purple x1
  LED_DOUBLE_BLINK,  // Blue x2
  LED_TRIPLE_BLINK,  // Red x3 for new device alert
  LED_READY_SIGNAL,  // Purple/Blue fade
  LED_GPS_WAIT       // Blue variable brightness
};

// Global Variables
OperatingMode currentMode = CONFIG_MODE;
AsyncWebServer server(80);
Preferences preferences;
NimBLEScan* pBLEScan;
unsigned long configStartTime = 0;
unsigned long lastConfigActivity = 0;
unsigned long modeSwitchScheduled = 0;
unsigned long deviceResetScheduled = 0;
unsigned long normalRestartScheduled = 0;
unsigned long lastWiFiScanTime = 0;
const int WIFI_SCAN_INTERVAL = 10000;
bool wifiScanInProgress = false;

// LED control variables
volatile LEDPattern currentLEDPattern = LED_OFF;
volatile int ledPatternParam = 0;

// Persistent settings
bool buzzerEnabled = false;
bool ledEnabled = true;

// Device tracking
struct DeviceInfo {
  String macAddress;
  int rssi;
  unsigned long firstSeen;
  unsigned long lastSeen;
  bool inCooldown;
  unsigned long cooldownUntil;
  String matchedFilter;
  String filterDescription;
};

struct TargetFilter {
  String identifier;
  bool isFullMAC;
  String description;
};

struct DeviceAlias {
  String macAddress;
  String alias;
};

std::vector<DeviceInfo> devices;
std::vector<TargetFilter> targetFilters;
std::vector<DeviceAlias> deviceAliases;

// Forward declarations
void startScanningMode();
class MyAdvertisedDeviceCallbacks;

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

const char CONFIG_HTML_TEMPLATE[] PROGMEM = R"html(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>M5GPS OUI-SPY Configuration</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin: 0; padding: 20px; background: #0a0a0a; color: #ffffff;
            position: relative; overflow-x: hidden;
        }
        .container {
            max-width: 700px; margin: 0 auto;
            background: rgba(255, 255, 255, 0.03);
            padding: 40px; border-radius: 16px;
            box-shadow: 0 8px 32px rgba(0, 0, 0, 0.3);
            backdrop-filter: blur(8px);
            border: 1px solid rgba(255, 107, 53, 0.2);
            position: relative; z-index: 1;
        }
        h1 {
            text-align: center; margin-bottom: 20px; font-size: 42px;
            font-weight: 700; letter-spacing: 2px;
            background: linear-gradient(45deg, #ff6b35, #f7931e);
            -webkit-background-clip: text; -webkit-text-fill-color: transparent;
        }
        .section {
            margin-bottom: 30px; padding: 25px;
            border: 1px solid rgba(255, 107, 53, 0.3);
            border-radius: 12px;
            background: rgba(255, 107, 53, 0.05);
        }
        .section h3 {
            margin-top: 0; color: #ff6b35; font-size: 18px;
            font-weight: 600; margin-bottom: 15px;
        }
        textarea {
            width: 100%; min-height: 120px; padding: 15px;
            border: 1px solid rgba(255, 107, 53, 0.4);
            border-radius: 8px;
            background: rgba(255, 255, 255, 0.05);
            color: #ffffff; font-family: monospace;
            font-size: 14px; resize: vertical;
        }
        textarea:focus {
            outline: none; border-color: #ff6b35;
            box-shadow: 0 0 0 3px rgba(255, 107, 53, 0.3);
        }
        .help-text {
            font-size: 13px; color: #cccccc; margin-top: 8px;
            line-height: 1.4;
        }
        input[type="text"] {
            width: 100%; padding: 12px;
            border: 1px solid rgba(255, 107, 53, 0.4);
            border-radius: 8px;
            background: rgba(255, 255, 255, 0.05);
            color: #ffffff; font-size: 14px;
        }
        input[type="text"]:focus {
            outline: none; border-color: #ff6b35;
            box-shadow: 0 0 0 3px rgba(255, 107, 53, 0.3);
        }
        button {
            background: linear-gradient(135deg, #ff6b35 0%, #f7931e 100%);
            color: #ffffff; padding: 14px 28px; border: none;
            border-radius: 8px; cursor: pointer; font-size: 16px;
            font-weight: 500; margin: 10px 5px; transition: all 0.3s;
        }
        button:hover {
            transform: translateY(-2px);
            box-shadow: 0 8px 25px rgba(255, 107, 53, 0.4);
        }
        .button-container {
            text-align: center; margin-top: 40px;
            padding-top: 30px; border-top: 1px solid #444444;
        }
        .status {
            padding: 15px; border-radius: 8px; margin-bottom: 30px;
            border-left: 4px solid #ff6b35;
            background: rgba(255, 107, 53, 0.1);
            color: #ffffff; border: 1px solid rgba(255, 107, 53, 0.3);
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>M5GPS OUI-SPY</h1>
        
        <div class="status">
            Configure MAC addresses and OUI prefixes to detect. LED patterns:
            <br><strong>Green x3:</strong> New device or re-detected after 30s
            <br><strong>Blue x2:</strong> Re-detected after 5s
            <br><strong>Dim Purple:</strong> Steady scanning status blink
            <br><strong>Orange:</strong> Configuration mode
            <br><strong>Purple fade:</strong> Ready signal when switching modes
        </div>

        <form method="POST" action="/save">
            <div class="section">
                <h3>OUI Prefixes (Manufacturer Detection)</h3>
                <textarea name="ouis" placeholder="%PLACEHOLDER_OUI_EXAMPLES%">%OUI_VALUES%</textarea>
                <div class="help-text">
                    OUI prefixes (first 3 bytes) detect all devices from a manufacturer.<br>
                    Format: XX:XX:XX (8 characters with colons)<br>
                    Example: 58:2D:34 for Espressif devices
                </div>
            </div>
            
            <div class="section">
                <h3>Specific MAC Addresses</h3>
                <textarea name="macs" placeholder="%PLACEHOLDER_MAC_EXAMPLES%">%MAC_VALUES%</textarea>
                <div class="help-text">
                    Full MAC addresses detect specific devices only.<br>
                    Format: XX:XX:XX:XX:XX:XX (17 characters with colons)<br>
                    Example: 58:2D:34:12:AB:CD for a specific ESP32
                </div>
            </div>
            
            <div class="section">
                <h3>WiFi Access Point Settings</h3>
                <div class="help-text" style="margin-bottom: 15px;">
                    Customize the WiFi network name and password for the configuration portal.<br>
                    <strong>Changes take effect on next device boot.</strong>
                </div>
                <div style="margin-bottom: 15px;">
                    <label style="display: block; margin-bottom: 8px; font-weight: 500;">Network Name (SSID)</label>
                    <input type="text" name="ap_ssid" value="%AP_SSID%" maxlength="32">
                    <div class="help-text" style="margin-top: 5px;">1-32 characters</div>
                </div>
                <div>
                    <label style="display: block; margin-bottom: 8px; font-weight: 500;">Password</label>
                    <input type="text" name="ap_password" value="%AP_PASSWORD%" minlength="8" maxlength="63">
                    <div class="help-text" style="margin-top: 5px;">8-63 characters</div>
                </div>
            </div>

            <div class="button-container">
                <button type="submit">Save Configuration & Start Scanning</button>
                <button type="button" onclick="clearConfig()" style="background: linear-gradient(135deg, #dc3545 0%, #c82333 100%);">Clear All Filters</button>
                <button type="button" onclick="deviceReset()" style="background: linear-gradient(135deg, #6c757d 0%, #495057 100%); font-size: 12px;">Device Reset</button>
            </div>
        </form>
        
        <script>
        function clearConfig() {
            if (confirm('Clear all filters?')) {
                fetch('/clear', { method: 'POST' })
                    .then(() => location.reload());
            }
        }
        
        function deviceReset() {
            if (confirm('DEVICE RESET: Complete wipe and restart. Continue?')) {
                fetch('/device-reset', { method: 'POST' })
                    .then(() => {
                        alert('Device resetting...');
                        setTimeout(() => window.location.href = '/', 5000);
                    });
            }
        }
        </script>
    </div>
</body>
</html>
)html";


// LED Task
void LEDTask(void* pvParameters) {
  static unsigned long previousMillis = 0;
  static unsigned long patternStartTime = 0;
  static bool patternActive = false;
  static bool statusBlinkState = false;

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  while (1) {
    unsigned long currentMillis = millis();

    UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    if (stackHighWaterMark < 100) {
      Serial.println("WARNING: LED Task stack low: " + String(stackHighWaterMark));
    }

    switch (currentLEDPattern) {
      case LED_OFF:
        M5.dis.drawpix(0, 0x000000);
        patternActive = false;
        break;

      case LED_STATUS_BLINK:
        {
          unsigned long interval = 2500;
          if (currentMillis - previousMillis >= interval) {
            statusBlinkState = !statusBlinkState;
            uint32_t color = (currentMode == CONFIG_MODE) ? 0xA5FF00 : 0x401040;
            M5.dis.drawpix(0, statusBlinkState ? color : 0x000000);
            previousMillis = currentMillis;
          }
        }
        break;

      case LED_SINGLE_BLINK:
        if (!patternActive) {
          patternStartTime = currentMillis;
          patternActive = true;
        }
        {
          unsigned long elapsed = currentMillis - patternStartTime;
          if (elapsed <= 200) {
            M5.dis.drawpix(0, 0x401040);
          } else if (elapsed <= 400) {
            M5.dis.drawpix(0, 0x000000);
          } else {
            M5.dis.drawpix(0, 0x000000);
            currentLEDPattern = LED_STATUS_BLINK;
            patternActive = false;
          }
        }
        break;

      case LED_DOUBLE_BLINK:
        {
          if (!patternActive) {
            patternStartTime = currentMillis;
            patternActive = true;
          }

          unsigned long elapsed = currentMillis - patternStartTime;

          if (elapsed < 150) {
            M5.dis.drawpix(0, 0xFF0000);
          } else if (elapsed < 300) {
            M5.dis.drawpix(0, 0x000000);
          } else if (elapsed < 450) {
            M5.dis.drawpix(0, 0xFF0000);
          } else if (elapsed < 600) {
            M5.dis.drawpix(0, 0x000000);
          } else {
            M5.dis.drawpix(0, 0x000000);
            currentLEDPattern = LED_STATUS_BLINK;
            patternActive = false;
          }
        }
        break;

      case LED_TRIPLE_BLINK:
        {
          if (!patternActive) {
            patternStartTime = currentMillis;
            patternActive = true;
          }

          unsigned long elapsed = currentMillis - patternStartTime;

          if (elapsed < 150) {
            M5.dis.drawpix(0, 0x00FF00);
          } else if (elapsed < 300) {
            M5.dis.drawpix(0, 0x000000);
          } else if (elapsed < 450) {
            M5.dis.drawpix(0, 0x00FF00);
          } else if (elapsed < 600) {
            M5.dis.drawpix(0, 0x000000);
          } else if (elapsed < 750) {
            M5.dis.drawpix(0, 0x00FF00);
          } else if (elapsed < 900) {
            M5.dis.drawpix(0, 0x000000);
          } else {
            M5.dis.drawpix(0, 0x000000);
            currentLEDPattern = LED_STATUS_BLINK;
            patternActive = false;
          }
        }
        break;

      case LED_READY_SIGNAL:
        {
          if (!patternActive) {
            patternStartTime = currentMillis;
            patternActive = true;
          }

          unsigned long elapsed = currentMillis - patternStartTime;
          if (elapsed > 2000) {
            M5.dis.drawpix(0, 0x000000);
            currentLEDPattern = LED_STATUS_BLINK;
            patternActive = false;
          } else {
            int brightness = (elapsed < 1000) ? (elapsed * 255 / 1000) : (255 - ((elapsed - 1000) * 255 / 1000));
            uint32_t purple = (brightness << 8) | brightness;
            M5.dis.drawpix(0, purple);
          }
        }
        break;

      case LED_GPS_WAIT:
        {
          static int brightness = 0;
          static bool ascending = true;
          int numSat = ledPatternParam;
          unsigned long interval = (numSat <= 1) ? 100UL : max(50UL, 300UL / (unsigned long)numSat);

          if (currentMillis - previousMillis >= interval) {
            if (ascending) {
              brightness += 10;
              if (brightness >= 255) ascending = false;
            } else {
              brightness -= 10;
              if (brightness <= 0) ascending = true;
            }

            uint32_t purple = (brightness << 8) | brightness;
            M5.dis.drawpix(0, purple);
            previousMillis = currentMillis;
          }
        }
        break;
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// Scanning Task
void ScanTask(void* pvParameters) {
  static unsigned long lastScanTime = 0;
  static unsigned long lastBLECleanupTime = 0;
  static unsigned long lastDeviceCleanupTime = 0;
  static unsigned long lastStatusTime = 0;
  static unsigned long lastDeviceSaveTime = 0;

  while (1) {
    if (currentMode == SCANNING_MODE) {
      unsigned long currentMillis = millis();

      if (currentMillis - lastBLECleanupTime >= 30000) {
        if (pBLEScan) {
          pBLEScan->clearResults();
        }
        lastBLECleanupTime = currentMillis;
      }

      if (currentMillis - lastDeviceCleanupTime >= 10000) {
        int devicesBefore = devices.size();
        for (auto it = devices.begin(); it != devices.end();) {
          if (currentMillis - it->lastSeen >= 60000) {
            Serial.println("Removed expired device: " + it->macAddress);
            it = devices.erase(it);
          } else {
            ++it;
          }
        }
        if (devices.size() != devicesBefore) {
          Serial.println("Cleanup: Removed " + String(devicesBefore - devices.size()) + " expired devices");
        }
        lastDeviceCleanupTime = currentMillis;
      }

      // Save detected devices to SD every 60 seconds
      if (currentMillis - lastDeviceSaveTime >= 60000) {
        saveDetectedDevices();
        lastDeviceSaveTime = currentMillis;
      }

      if (currentMillis - lastScanTime >= 1000) {
        if (pBLEScan) {
          pBLEScan->stop();
          vTaskDelay(10 / portTICK_PERIOD_MS);
          pBLEScan->start(0.8, false, false);
        }
        lastScanTime = currentMillis;
      }

      if (currentMillis - lastWiFiScanTime >= WIFI_SCAN_INTERVAL && !wifiScanInProgress) {
        performWiFiScan();
        lastWiFiScanTime = currentMillis;
      }

      if (currentMillis - lastStatusTime >= 30000) {
        Serial.println("Status: Scanning - " + String(devices.size()) + " active devices tracked");
        lastStatusTime = currentMillis;
      }
    }

    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

// LED Pattern Functions
void triggerSingleBlink() {
  currentLEDPattern = LED_SINGLE_BLINK;
}

void triggerDoubleBlink() {
  currentLEDPattern = LED_DOUBLE_BLINK;
}

void triggerTripleBlink() {
  currentLEDPattern = LED_TRIPLE_BLINK;
}

void triggerReadySignal() {
  currentLEDPattern = LED_READY_SIGNAL;
}

void startGPSWaitPattern(int numSatellites) {
  ledPatternParam = numSatellites;
  currentLEDPattern = LED_GPS_WAIT;
}

void stopGPSWaitPattern() {
  currentLEDPattern = LED_OFF;
}

void startStatusBlinking() {
  currentLEDPattern = LED_STATUS_BLINK;
}

void waitForGPSFix() {
  Serial.println("Waiting for GPS fix... (Press button to bypass)");

  while (REQUIRE_GPS_FIX && !gps.location.isValid()) {
    M5.update();

    if (M5.Btn.wasPressed()) {
      Serial.println("Button pressed - bypassing GPS fix requirement");
      stopGPSWaitPattern();
      M5.dis.drawpix(0, 0x00FF00);
      delay(1500);
      M5.dis.clear();
      return;
    }

    while (Serial1.available() > 0) gps.encode(Serial1.read());
    startGPSWaitPattern(gps.satellites.value());
    delay(100);
  }

  stopGPSWaitPattern();
  Serial.println("GPS fix obtained or bypassed.");
}

void startupTest() {
  M5.dis.drawpix(0, 0xFFFFFF);
  delay(300);
  M5.dis.drawpix(0, 0x000000);
}

// SD & File Creation
bool initSD() {
  SPI.begin(23, 33, 19, -1);
  delay(300);
  if (SD.begin(SD_CS, SPI, 20000000)) {
    return true;
  }
  Serial.println("SD 20MHz failed, trying 10MHz");
  SD.end();
  delay(200);
  return SD.begin(SD_CS, SPI, 10000000);
}

void initializeFile() {
  if (!sdReady) {
    Serial.println("SD not ready in initializeFile!");
    return;
  }

  int y = gps.date.isValid() ? gps.date.year() : 1970;
  int m = gps.date.isValid() ? gps.date.month() : 1;
  int d = gps.date.isValid() ? gps.date.day() : 1;

  char dateStamp[16];
  sprintf(dateStamp, "%04d-%02d-%02d-", y, m, d);

  int fileNumber = 0;
  do {
    snprintf(fileName, sizeof(fileName), "/OUISPY-%s%d.csv", dateStamp, fileNumber);
    fileNumber++;
  } while (SD.exists(fileName) && fileNumber < 999);

  File f = SD.open(fileName, FILE_WRITE);
  if (!f) {
    Serial.println("Failed to create log file!");
    sdReady = false;
    return;
  }

  f.println("WhenUTC,MatchType,MAC,RSSI,Lat,Lon,AltM,HDOP,Filter,Alias");
  f.close();

  if (!SD.exists(fileName)) {
    Serial.println("File creation failed!");
    sdReady = false;
    return;
  }

  Serial.println("Log file created: " + String(fileName));
}

void logMatchRow(const String& type, const String& mac, int rssi, const String& filt) {
  if (!sdReady || !SD.exists(fileName)) {
    Serial.println("SD or file not ready!");
    return;
  }

  char utc[21];
  if (gps.date.isValid() && gps.time.isValid()) {
    sprintf(utc, "%04d-%02d-%02d %02d:%02d:%02d",
            gps.date.year(), gps.date.month(), gps.date.day(),
            gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    strcpy(utc, "1970-01-01 00:00:00");
  }

  String alias = getDeviceAlias(mac);

  char line[512];
  snprintf(line, sizeof(line),
           "%s,%s,%s,%d,%.6f,%.6f,%.2f,%.2f,%s,%s\n",
           utc, type.c_str(), mac.c_str(), rssi,
           gps.location.isValid() ? gps.location.lat() : 0.0,
           gps.location.isValid() ? gps.location.lng() : 0.0,
           gps.altitude.isValid() ? gps.altitude.meters() : 0.0,
           gps.hdop.isValid() ? gps.hdop.hdop() : -1.0,
           filt.c_str(),
           alias.c_str());

  File f = SD.open(fileName, FILE_APPEND);
  if (!f) {
    return;
  }

  f.print(line);
  f.flush();
  f.close();
}

// WiFi Scanning
void performWiFiScan() {
  if (wifiScanInProgress) return;

  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  delay(100);

  esp_wifi_stop();
  esp_wifi_deinit();
  delay(100);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_start();
  delay(100);

  WiFi.mode(WIFI_STA);
  delay(100);

  Serial.println("Starting WiFi scan...");
  wifiScanInProgress = true;

  int networksFound = WiFi.scanNetworks(false, true, false, 200, 0);

  if (networksFound == WIFI_SCAN_FAILED || networksFound < 0) {
    Serial.println("WiFi scan failed with code: " + String(networksFound));
    wifiScanInProgress = false;
    return;
  }

  if (networksFound == 0) {
    Serial.println("No networks found");
  } else {
    Serial.println(String(networksFound) + " networks found");
    processWiFiResults(networksFound);
  }

  WiFi.scanDelete();
  wifiScanInProgress = false;
}

void processWiFiResults(int networksFound) {
  unsigned long currentMillis = millis();
  
  for (int i = 0; i < networksFound; i++) {
    String bssid = WiFi.BSSIDstr(i);
    String ssid = WiFi.SSID(i);
    int rssi = WiFi.RSSI(i);

    String matchedDescription;
    bool matchFound = matchesTargetFilter(bssid, matchedDescription);

    if (!matchFound) {
      continue;
    }

    Serial.println("MATCHED FILTER: " + matchedDescription);

    bool known = false;
    for (auto& dev : devices) {
      if (dev.macAddress == bssid) {
        known = true;

        if (dev.inCooldown && currentMillis < dev.cooldownUntil) break;
        if (dev.inCooldown && currentMillis >= dev.cooldownUntil) dev.inCooldown = false;

        unsigned long dt = currentMillis - dev.lastSeen;

        if (dt >= 30000) {
          triggerTripleBlink();
          Serial.println("WIFI RE-DETECTED after 30+ sec: " + matchedDescription);
          Serial.println("MAC: " + bssid + " | SSID: " + ssid + " | RSSI: " + String(rssi));
          logMatchRow("WIFI-RE30s", bssid, rssi, matchedDescription);
          dev.inCooldown = true;
          dev.cooldownUntil = currentMillis + 10000;
        } else if (dt >= 5000) {
          triggerDoubleBlink();
          Serial.println("WIFI RE-DETECTED after 5+ sec: " + matchedDescription);
          Serial.println("MAC: " + bssid + " | SSID: " + ssid + " | RSSI: " + String(rssi));
          logMatchRow("WIFI-RE5s", bssid, rssi, matchedDescription);
          dev.inCooldown = true;
          dev.cooldownUntil = currentMillis + 5000;
        }

        dev.lastSeen = currentMillis;
        dev.rssi = rssi;
        break;
      }
    }

    if (!known) {
      DeviceInfo newDev{ bssid, rssi, currentMillis, currentMillis, false, 0, matchedDescription, matchedDescription };
      devices.push_back(newDev);

      triggerTripleBlink();
      Serial.println("NEW WIFI DEVICE DETECTED: " + matchedDescription);
      Serial.println("MAC: " + bssid + " | SSID: " + ssid + " | RSSI: " + String(rssi));
      logMatchRow("WIFI-NEW", bssid, rssi, matchedDescription);

      auto& dev = devices.back();
      dev.inCooldown = true;
      dev.cooldownUntil = currentMillis + 5000;
    }
  }

  WiFi.scanDelete();
  wifiScanInProgress = false;
}

// MAC Address Utility Functions
void normalizeMACAddress(String& mac) {
  mac.toLowerCase();
  mac.replace("-", ":");
  mac.replace(" ", "");
}

bool isValidMAC(const String& mac) {
  String normalized = mac;
  normalizeMACAddress(normalized);

  if (normalized.length() != 8 && normalized.length() != 17) {
    return false;
  }

  for (int i = 0; i < normalized.length(); i++) {
    char c = normalized.charAt(i);
    if (i % 3 == 2) {
      if (c != ':') return false;
    } else {
      if (!isxdigit(c)) return false;
    }
  }

  return true;
}

bool matchesTargetFilter(const String& deviceMAC, String& matchedDescription) {
  if (targetFilters.empty()) {
    return false;
  }

  String normalizedDeviceMAC = deviceMAC;
  normalizeMACAddress(normalizedDeviceMAC);

  for (const TargetFilter& filter : targetFilters) {
    String filterID = filter.identifier;
    normalizeMACAddress(filterID);

    if (filter.isFullMAC) {
      if (normalizedDeviceMAC.equals(filterID)) {
        matchedDescription = filter.description;
        return true;
      }
    } else {
      if (normalizedDeviceMAC.startsWith(filterID)) {
        matchedDescription = filter.description;
        return true;
      }
    }
  }
  return false;
}

// Configuration Storage Functions
void saveConfiguration() {
  Serial.println("=== SAVING CONFIGURATION ===");
  Serial.println("Filters to save: " + String(targetFilters.size()));
  
  if (!preferences.begin("ouispy", false)) {
    Serial.println("ERROR: Failed to open NVS for write");
    return;
  }
  
  size_t written = preferences.putInt("filterCount", targetFilters.size());
  if (written == 0) {
    Serial.println("ERROR: Failed to write filterCount");
    preferences.end();
    return;
  }
  
  Serial.println("Successfully wrote filterCount: " + String(targetFilters.size()));
  
  for (int i = 0; i < targetFilters.size(); i++) {
    String keyId = "id_" + String(i);
    String keyMAC = "mac_" + String(i);  
    String keyDesc = "desc_" + String(i);

    size_t id_written = preferences.putString(keyId.c_str(), targetFilters[i].identifier);
    size_t mac_written = preferences.putBool(keyMAC.c_str(), targetFilters[i].isFullMAC) ? 1 : 0;
    size_t desc_written = preferences.putString(keyDesc.c_str(), targetFilters[i].description);
    
    Serial.println("Filter " + String(i) + " write status - ID:" + String(id_written) + 
                   " MAC:" + String(mac_written) + " DESC:" + String(desc_written));
  }

  preferences.end();
  Serial.println("NVS write completed");
  Serial.println("===============================");
}

void loadConfiguration() {
  preferences.begin("ouispy", true);
  int filterCount = preferences.getInt("filterCount", 0);
  buzzerEnabled = preferences.getBool("buzzerEnabled", true);
  ledEnabled = preferences.getBool("ledEnabled", true);

  targetFilters.clear();

  if (filterCount > 0) {
    for (int i = 0; i < filterCount; i++) {
      String keyId = "id_" + String(i);
      String keyMAC = "mac_" + String(i);
      String keyDesc = "desc_" + String(i);

      TargetFilter filter;
      filter.identifier = preferences.getString(keyId.c_str(), "");
      filter.isFullMAC = preferences.getBool(keyMAC.c_str(), false);
      filter.description = preferences.getString(keyDesc.c_str(), "");

      if (filter.identifier.length() > 0) {
        targetFilters.push_back(filter);
      }
    }
  }

  preferences.end();
  Serial.println("Configuration loaded - " + String(targetFilters.size()) + " filters");
}

void loadWiFiCredentials() {
  preferences.begin("ouispy", true);
  AP_SSID = preferences.getString("ap_ssid", "snoopuntothem");
  AP_PASSWORD = preferences.getString("ap_password", "astheysnoopuntous");
  preferences.end();
}

void saveWiFiCredentials() {
  preferences.begin("ouispy", false);
  preferences.putString("ap_ssid", AP_SSID);
  preferences.putString("ap_password", AP_PASSWORD);
  preferences.end();
}

// Device Alias Functions - SD Card Based
void saveDeviceAliases() {
  if (!sdReady) return;

  File f = SD.open("/aliases.json", FILE_WRITE);
  if (!f) {
    Serial.println("Failed to open aliases file for writing");
    return;
  }

  f.print("[");
  for (size_t i = 0; i < deviceAliases.size(); i++) {
    if (i > 0) f.print(",");
    f.print("{\"mac\":\"");
    f.print(deviceAliases[i].macAddress);
    f.print("\",\"alias\":\"");
    f.print(deviceAliases[i].alias);
    f.print("\"}");
  }
  f.print("]");
  f.flush();
  f.close();

  Serial.println("Device aliases saved to SD (" + String(deviceAliases.size()) + " aliases)");
}

void loadDeviceAliases() {
  if (!sdReady) return;

  deviceAliases.clear();

  if (!SD.exists("/aliases.json")) {
    Serial.println("No aliases file found");
    return;
  }

  File f = SD.open("/aliases.json", FILE_READ);
  if (!f) {
    Serial.println("Failed to open aliases file");
    return;
  }

  String jsonContent = "";
  while (f.available()) {
    jsonContent += (char)f.read();
  }
  f.close();

  // Simple JSON parsing for {"mac":"...","alias":"..."}
  int startIdx = 0;
  while (true) {
    int macStart = jsonContent.indexOf("\"mac\":\"", startIdx);
    if (macStart == -1) break;
    macStart += 7;
    int macEnd = jsonContent.indexOf("\"", macStart);
    
    int aliasStart = jsonContent.indexOf("\"alias\":\"", macEnd);
    if (aliasStart == -1) break;
    aliasStart += 9;
    int aliasEnd = jsonContent.indexOf("\"", aliasStart);

    String mac = jsonContent.substring(macStart, macEnd);
    String alias = jsonContent.substring(aliasStart, aliasEnd);

    if (mac.length() > 0 && alias.length() > 0) {
      DeviceAlias da;
      da.macAddress = mac;
      da.alias = alias;
      deviceAliases.push_back(da);
    }

    startIdx = aliasEnd + 1;
  }

  Serial.println("Device aliases loaded from SD (" + String(deviceAliases.size()) + " aliases)");
}

String getDeviceAlias(const String& macAddress) {
  String normalizedMAC = macAddress;
  normalizeMACAddress(normalizedMAC);

  for (const DeviceAlias& alias : deviceAliases) {
    String normalizedAliasMAC = alias.macAddress;
    normalizeMACAddress(normalizedAliasMAC);

    if (normalizedAliasMAC.equals(normalizedMAC)) {
      return alias.alias;
    }
  }

  return "";
}

void setDeviceAlias(const String& macAddress, const String& alias) {
  String normalizedMAC = macAddress;
  normalizeMACAddress(normalizedMAC);

  for (auto& deviceAlias : deviceAliases) {
    String normalizedAliasMAC = deviceAlias.macAddress;
    normalizeMACAddress(normalizedAliasMAC);

    if (normalizedAliasMAC.equals(normalizedMAC)) {
      if (alias.length() > 0) {
        deviceAlias.alias = alias;
      } else {
        for (size_t i = 0; i < deviceAliases.size(); i++) {
          String mac = deviceAliases[i].macAddress;
          normalizeMACAddress(mac);
          if (mac.equals(normalizedMAC)) {
            deviceAliases.erase(deviceAliases.begin() + i);
            break;
          }
        }
      }
      saveDeviceAliases();
      return;
    }
  }

  if (alias.length() > 0) {
    DeviceAlias newAlias;
    newAlias.macAddress = normalizedMAC;
    newAlias.alias = alias;
    deviceAliases.push_back(newAlias);
    saveDeviceAliases();
  }
}

// Persistent Device Storage Functions - SD Card Based
void saveDetectedDevices() {
  if (!sdReady) return;

  File f = SD.open("/devices.json", FILE_WRITE);
  if (!f) {
    Serial.println("Failed to open devices file for writing");
    return;
  }

  f.print("[");
  for (size_t i = 0; i < devices.size(); i++) {
    if (i > 0) f.print(",");
    f.print("{\"mac\":\"");
    f.print(devices[i].macAddress);
    f.print("\",\"rssi\":");
    f.print(devices[i].rssi);
    f.print(",\"first\":");
    f.print(devices[i].firstSeen);
    f.print(",\"last\":");
    f.print(devices[i].lastSeen);
    f.print(",\"filter\":\"");
    f.print(devices[i].filterDescription);
    f.print("\"}");
  }
  f.print("]");
  f.flush();
  f.close();
}

void loadDetectedDevices() {
  if (!sdReady) return;

  devices.clear();

  if (!SD.exists("/devices.json")) {
    Serial.println("No devices file found");
    return;
  }

  File f = SD.open("/devices.json", FILE_READ);
  if (!f) {
    Serial.println("Failed to open devices file");
    return;
  }

  String jsonContent = "";
  while (f.available()) {
    jsonContent += (char)f.read();
  }
  f.close();

  // Simple JSON parsing
  int startIdx = 0;
  while (true) {
    int macStart = jsonContent.indexOf("\"mac\":\"", startIdx);
    if (macStart == -1) break;
    macStart += 7;
    int macEnd = jsonContent.indexOf("\"", macStart);
    
    int rssiStart = jsonContent.indexOf("\"rssi\":", macEnd);
    if (rssiStart == -1) break;
    rssiStart += 7;
    int rssiEnd = jsonContent.indexOf(",", rssiStart);
    
    int firstStart = jsonContent.indexOf("\"first\":", rssiEnd);
    if (firstStart == -1) break;
    firstStart += 8;
    int firstEnd = jsonContent.indexOf(",", firstStart);
    
    int lastStart = jsonContent.indexOf("\"last\":", firstEnd);
    if (lastStart == -1) break;
    lastStart += 7;
    int lastEnd = jsonContent.indexOf(",", lastStart);
    
    int filterStart = jsonContent.indexOf("\"filter\":\"", lastEnd);
    if (filterStart == -1) break;
    filterStart += 10;
    int filterEnd = jsonContent.indexOf("\"", filterStart);

    DeviceInfo device;
    device.macAddress = jsonContent.substring(macStart, macEnd);
    device.rssi = jsonContent.substring(rssiStart, rssiEnd).toInt();
    device.firstSeen = jsonContent.substring(firstStart, firstEnd).toInt();
    device.lastSeen = jsonContent.substring(lastStart, lastEnd).toInt();
    device.filterDescription = jsonContent.substring(filterStart, filterEnd);
    device.inCooldown = false;
    device.cooldownUntil = 0;
    device.matchedFilter = "";

    if (device.macAddress.length() > 0) {
      devices.push_back(device);
    }

    startIdx = filterEnd + 1;
  }

  Serial.println("Detected devices loaded from SD (" + String(devices.size()) + " devices)");
}

void clearDetectedDevices() {
  devices.clear();
  
  if (sdReady && SD.exists("/devices.json")) {
    SD.remove("/devices.json");
  }
  
  Serial.println("All detected devices cleared from memory and SD");
}

// Web Server HTML Generation
const char* getASCIIArt() {
  return R"(
   ____  __  ______                
  / __ \/ / / /  _/________  __  __
 / / / / / / // // ___/ __ \/ / / /
/ /_/ / /_/ // /(__  ) /_/ / /_/ / 
\____/\____/___/____/ .___/\__, /  
                   /_/    /____/   
    )";
}

String generateRandomOUI() {
  String oui = "";
  for (int i = 0; i < 3; i++) {
    if (i > 0) oui += ":";
    int val = random(0, 256);
    if (val < 16) oui += "0";
    oui += String(val, HEX);
  }
  oui.toLowerCase();
  return oui;
}

String generateRandomMAC() {
  String mac = "";
  for (int i = 0; i < 6; i++) {
    if (i > 0) mac += ":";
    int val = random(0, 256);
    if (val < 16) mac += "0";
    mac += String(val, HEX);
  }
  mac.toLowerCase();
  return mac;
}

String generateConfigHTML() {
  String ouiValues = "";
  String macValues = "";

  for (const TargetFilter& filter : targetFilters) {
    if (filter.isFullMAC) {
      if (macValues.length() > 0) macValues += "\n";
      macValues += filter.identifier;
    } else {
      if (ouiValues.length() > 0) ouiValues += "\n";
      ouiValues += filter.identifier;
    }
  }

  String randomOUIExamples = generateRandomOUI() + "\n" + generateRandomOUI() + "\n" + generateRandomOUI();
  String randomMACExamples = generateRandomMAC() + "\n" + generateRandomMAC() + "\n" + generateRandomMAC();

  // Read HTML template from PROGMEM
  String html = FPSTR(CONFIG_HTML_TEMPLATE);
  
  // Replace placeholders with actual values
  html.replace("%OUI_VALUES%", ouiValues);
  html.replace("%MAC_VALUES%", macValues);
  html.replace("%PLACEHOLDER_OUI_EXAMPLES%", randomOUIExamples);
  html.replace("%PLACEHOLDER_MAC_EXAMPLES%", randomMACExamples);
  html.replace("%AP_SSID%", AP_SSID);
  html.replace("%AP_PASSWORD%", AP_PASSWORD);

  return html;
}

// WiFi and Web Server Functions
void startConfigMode() {
  currentMode = CONFIG_MODE;
  
  Serial.println("\n=== STARTING CONFIG MODE ===");
  Serial.println("SSID: " + AP_SSID);
  Serial.println("Password: " + AP_PASSWORD);
  
  // Critical fix for AP visibility
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  delay(1000);
  
  WiFi.mode(WIFI_AP);
  delay(500);
  
  // Fix: specify channel 6 explicitly for compatibility
  bool apStarted = WiFi.softAP(AP_SSID.c_str(), AP_PASSWORD.c_str(), 6, 0, 4);
  
  if (!apStarted) {
    Serial.println("Failed to create Access Point!");
    return;
  }
  
  delay(2000);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.println("AP IP address: " + IP.toString());
  Serial.println("Config portal: http://" + IP.toString());
  Serial.println("==============================\n");
  
  configStartTime = millis();
  lastConfigActivity = millis();
  
  startStatusBlinking();
  
  // Setup web server routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    lastConfigActivity = millis();
    
    Serial.println("=== HTTP REQUEST DEBUG ===");
    Serial.println("Free heap before: " + String(ESP.getFreeHeap()));
    Serial.println("Largest free block: " + String(ESP.getMaxAllocHeap()));
    
    String html = generateConfigHTML();
    
    Serial.println("HTML length: " + String(html.length()));
    Serial.println("Free heap after HTML gen: " + String(ESP.getFreeHeap()));
    
    if (html.length() == 0) {
      Serial.println("ERROR: HTML generation returned empty string!");
      request->send(500, "text/plain", "HTML generation failed");
      return;
    }
    
    // Check if we can send the response
    AsyncWebServerResponse *response = request->beginResponse(200, "text/html", html);
    if (response == nullptr) {
      Serial.println("ERROR: Failed to create response object!");
      request->send(500, "text/plain", "Response creation failed");
      return;
    }
    
    request->send(response);
    Serial.println("Response sent successfully");
    Serial.println("Free heap after send: " + String(ESP.getFreeHeap()));
    Serial.println("========================");
  });
  
  server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) {
    lastConfigActivity = millis();
    
    targetFilters.clear();
    
    if (request->hasParam("ouis", true)) {
      String ouiData = request->getParam("ouis", true)->value();
      ouiData.trim();
      if (ouiData.length() > 0) {
        int start = 0;
        int end = ouiData.indexOf('\n');
        while (start < ouiData.length()) {
          String oui;
          if (end == -1) {
            oui = ouiData.substring(start);
            start = ouiData.length();
          } else {
            oui = ouiData.substring(start, end);
            start = end + 1;
            end = ouiData.indexOf('\n', start);
          }
          oui.trim();
          oui.replace("\r", "");
          if (oui.length() > 0 && isValidMAC(oui)) {
            TargetFilter filter;
            filter.identifier = oui;
            filter.description = "OUI: " + oui;
            filter.isFullMAC = false;
            targetFilters.push_back(filter);
          }
        }
      }
    }
    
    if (request->hasParam("macs", true)) {
      String macData = request->getParam("macs", true)->value();
      macData.trim();
      if (macData.length() > 0) {
        int start = 0;
        int end = macData.indexOf('\n');
        while (start < macData.length()) {
          String mac;
          if (end == -1) {
            mac = macData.substring(start);
            start = macData.length();
          } else {
            mac = macData.substring(start, end);
            start = end + 1;
            end = macData.indexOf('\n', start);
          }
          mac.trim();
          mac.replace("\r", "");
          if (mac.length() > 0 && isValidMAC(mac)) {
            TargetFilter filter;
            filter.identifier = mac;
            filter.description = "MAC: " + mac;
            filter.isFullMAC = true;
            targetFilters.push_back(filter);
          }
        }
      }
    }
    
    if (request->hasParam("ap_ssid", true)) {
      String newSSID = request->getParam("ap_ssid", true)->value();
      newSSID.trim();
      if (newSSID.length() > 0 && newSSID.length() <= 32) {
        AP_SSID = newSSID;
      }
    }
    
    if (request->hasParam("ap_password", true)) {
      String newPassword = request->getParam("ap_password", true)->value();
      newPassword.trim();
      if (newPassword.length() == 0 || (newPassword.length() >= 8 && newPassword.length() <= 63)) {
        AP_PASSWORD = newPassword;
      }
    }
    
    saveWiFiCredentials();
    
    if (targetFilters.size() > 0) {
      saveConfiguration();
      
      String responseHTML = R"html(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Configuration Saved</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {
            font-family: 'Segoe UI', sans-serif; margin: 0; padding: 20px;
            background: #0a0a0a; color: #ffffff; text-align: center;
        }
        .container {
            max-width: 600px; margin: 0 auto;
            background: rgba(255,255,255,0.03); padding: 40px;
            border-radius: 12px; backdrop-filter: blur(8px);
            border: 1px solid rgba(255, 107, 53, 0.3);
        }
        h1 { color: #ff6b35; margin-bottom: 30px; }
        .success {
            background: rgba(255, 107, 53, 0.2); color: #ff6b35;
            border: 1px solid #ff6b35; padding: 20px; border-radius: 8px;
            margin: 30px 0;
        }
    </style>
    <script>
        setTimeout(() => {
            document.getElementById('countdown').innerHTML = 'Switching to scanning mode now...';
        }, 5000);
    </script>
</head>
<body>
    <div class="container">
        <h1>Configuration Saved</h1>
        <div class="success">
            <p><strong>Saved )html" + String(targetFilters.size()) + R"html( filters successfully!</strong></p>
            <p id="countdown">Switching to scanning mode in 5 seconds...</p>
        </div>
    </div>
</body>
</html>
)html";
      
      request->send(200, "text/html", responseHTML);
      modeSwitchScheduled = millis() + 5000;
    } else {
      request->send(400, "text/html", "<h1>Error: No valid filters provided</h1>");
    }
  });
  
  server.on("/clear", HTTP_POST, [](AsyncWebServerRequest *request) {
    lastConfigActivity = millis();
    targetFilters.clear();
    saveConfiguration();
    request->send(200, "text/plain", "Filters cleared successfully");
  });
  
  server.on("/device-reset", HTTP_POST, [](AsyncWebServerRequest *request) {
    lastConfigActivity = millis();
    request->send(200, "text/html", 
      "<html><body style='background:#0a0a0a;color:#ff6b35;'>Device resetting...</body></html>");
    deviceResetScheduled = millis() + 3000;
  });
  
  server.on("/api/devices", HTTP_GET, [](AsyncWebServerRequest *request) {
    lastConfigActivity = millis();
    
    String json = "{\"devices\":[";
    unsigned long currentTime = millis();
    
    for (size_t i = 0; i < devices.size(); i++) {
      if (i > 0) json += ",";
      String alias = getDeviceAlias(devices[i].macAddress);
      unsigned long timeSince = (currentTime >= devices[i].lastSeen) ? 
                               (currentTime - devices[i].lastSeen) : 0;
      
      json += "{";
      json += "\"mac\":\"" + devices[i].macAddress + "\",";
      json += "\"rssi\":" + String(devices[i].rssi) + ",";
      json += "\"filter\":\"" + devices[i].filterDescription + "\",";
      json += "\"alias\":\"" + alias + "\",";
      json += "\"lastSeen\":" + String(devices[i].lastSeen) + ",";
      json += "\"timeSince\":" + String(timeSince);
      json += "}";
    }
    
    json += "],";
    json += "\"currentTime\":" + String(currentTime);
    json += "}";
    
    request->send(200, "application/json", json);
  });
  
  server.on("/api/alias", HTTP_POST, [](AsyncWebServerRequest *request) {
    lastConfigActivity = millis();
    
    if (request->hasParam("mac", true) && request->hasParam("alias", true)) {
      String mac = request->getParam("mac", true)->value();
      String alias = request->getParam("alias", true)->value();
      
      setDeviceAlias(mac, alias);
      
      String action = (alias.length() > 0) ? "saved" : "removed";
      Serial.println("Alias " + action + ": " + mac);
      
      request->send(200, "application/json", "{\"success\":true}");
    } else {
      request->send(400, "application/json", "{\"success\":false}");
    }
  });
  
  server.on("/api/clear-devices", HTTP_POST, [](AsyncWebServerRequest *request) {
    lastConfigActivity = millis();
    clearDetectedDevices();
    request->send(200, "application/json", "{\"success\":true}");
  });
  
  server.on("/api/lock-config", HTTP_POST, [](AsyncWebServerRequest *request) {
    lastConfigActivity = millis();
    
    Serial.println("Configuration lock requested");
    
    preferences.begin("ouispy", false);
    preferences.putBool("configLocked", true);
    preferences.end();
    
    Serial.println("Configuration locked - reflash to unlock");
    
    String responseHTML = R"html(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Configuration Locked</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {
            font-family: 'Segoe UI', sans-serif; margin: 0; padding: 20px;
            background: #0a0a0a; color: #ffffff; text-align: center;
        }
        .container {
            max-width: 750px; margin: 0 auto;
            background: rgba(255,255,255,0.03); padding: 50px;
            border-radius: 16px; border: 2px solid #8b0000;
        }
        h1 { color: #ff6b6b; }
        .warning {
            background: rgba(139, 0, 0, 0.2); color: #ffcccc;
            border: 2px solid #8b0000; padding: 25px;
            border-radius: 10px; margin: 25px 0;
        }
    </style>
    <script>
        setTimeout(() => window.location.href = 'about:blank', 3000);
    </script>
</head>
<body>
    <div class="container">
        <h1>Configuration Locked</h1>
        <div class="warning">
            <p><strong>CONFIGURATION PERMANENTLY LOCKED</strong></p>
            <p>WiFi AP disabled on next boot</p>
            <p>Reflash firmware to unlock</p>
        </div>
        <p>Device restarting in 3 seconds...</p>
    </div>
</body>
</html>
)html";
    
    request->send(200, "text/html", responseHTML);
    normalRestartScheduled = millis() + 3000;
  });
  
  server.begin();
  Serial.println("Web server started");
}

// BLE Advertised Device Callback Class
class MyAdvertisedDeviceCallbacks: public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    if (currentMode != SCANNING_MODE) return;
    
    String mac = String(advertisedDevice->getAddress().toString().c_str());
    int rssi = advertisedDevice->getRSSI();
    unsigned long currentMillis = millis();
    
    String matchedDescription;
    bool matchFound = matchesTargetFilter(mac, matchedDescription);
    
    if (!matchFound) return;
    
    bool known = false;
    for (auto& dev : devices) {
      if (dev.macAddress == mac) {
        known = true;
        
        if (dev.inCooldown && currentMillis < dev.cooldownUntil) return;
        if (dev.inCooldown && currentMillis >= dev.cooldownUntil) dev.inCooldown = false;
        
        unsigned long timeSinceLastSeen = currentMillis - dev.lastSeen;
        
        if (timeSinceLastSeen >= 30000) {
          triggerTripleBlink();
          Serial.println("BLE RE-DETECTED after 30+ sec: " + matchedDescription);
          Serial.println("MAC: " + mac + " | RSSI: " + String(rssi));
          logMatchRow("BLE-RE30s", mac, rssi, matchedDescription);
          dev.inCooldown = true;
          dev.cooldownUntil = currentMillis + 10000;
        } else if (timeSinceLastSeen >= 5000) {
          triggerDoubleBlink();
          Serial.println("BLE RE-DETECTED after 5+ sec: " + matchedDescription);
          Serial.println("MAC: " + mac + " | RSSI: " + String(rssi));
          logMatchRow("BLE-RE5s", mac, rssi, matchedDescription);
          dev.inCooldown = true;
          dev.cooldownUntil = currentMillis + 5000;
        }
        
        dev.lastSeen = currentMillis;
        dev.rssi = rssi;
        break;
      }
    }
    
    if (!known) {
      DeviceInfo newDev;
      newDev.macAddress = mac;
      newDev.rssi = rssi;
      newDev.firstSeen = currentMillis;
      newDev.lastSeen = currentMillis;
      newDev.inCooldown = false;
      newDev.cooldownUntil = 0;
      newDev.matchedFilter = matchedDescription;
      newDev.filterDescription = matchedDescription;
      devices.push_back(newDev);
      
      triggerTripleBlink();
      Serial.println("NEW BLE DEVICE DETECTED: " + matchedDescription);
      Serial.println("MAC: " + mac + " | RSSI: " + String(rssi));
      logMatchRow("BLE-NEW", mac, rssi, matchedDescription);
      
      auto& dev = devices.back();
      dev.inCooldown = true;
      dev.cooldownUntil = currentMillis + 5000;
    }
  }
};

void startScanningMode() {
  currentMode = SCANNING_MODE;
  
  server.end();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(500);
  
  esp_wifi_stop();
  esp_wifi_deinit();
  delay(500);
  
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_start();
  delay(500);
  
  WiFi.mode(WIFI_STA);
  delay(500);
  
  Serial.println("\n=== STARTING SCANNING MODE ===");
  Serial.println("Configured Filters:");
  for (const TargetFilter& filter : targetFilters) {
    String type = filter.isFullMAC ? "Full MAC" : "OUI";
    Serial.println("- " + filter.identifier + " (" + type + "): " + filter.description);
  }
  Serial.println("==============================\n");
  
  NimBLEDevice::init("");
  delay(1000);

  pBLEScan = NimBLEDevice::getScan();
  if (pBLEScan != nullptr) {
    pBLEScan->setScanCallbacks(new MyAdvertisedDeviceCallbacks(), false);
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(300);
    pBLEScan->setWindow(200);
  }

  delay(500);
  triggerReadySignal();
  delay(2000);

  if (pBLEScan != nullptr) {
    pBLEScan->start(3, false);
    Serial.println("BLE scanning started!");
  }
  
  Serial.println("WiFi scanning ready!");
  startStatusBlinking();
}
void setup() {
  delay(2000);
  Serial.begin(115200);
  delay(1000);
  
  M5.begin(true, false, true);
  delay(50);
  
  startupTest();
  
  // Create LED task on Core 1
  xTaskCreatePinnedToCore(
    LEDTask,
    "LEDTask",
    4096,
    NULL,
    3,
    &LEDTaskHandle,
    1
  );
  
  Serial.println("Free heap at startup: " + String(ESP.getFreeHeap()));
  
  WiFi.mode(WIFI_OFF);
  
  // SD init
  sdReady = initSD();
  if (!sdReady) {
    Serial.println("SD init failed. Continuing without logging.");
  } else {
    Serial.println("SD ready.");
  }
  
  // GPS init
  Serial1.begin(GPS_BAUD, SERIAL_8N1, 22, -1);
  delay(200);
  Serial.println("GPS serial ready.");
  
  Serial.println("\n\n=== M5GPS OUI-SPY ===");
  Serial.println("Mode: Multi-target BLE/WiFi device detection");
  Serial.println("Storage: SD card for device history and aliases");
  Serial.println("Initializing...\n");
  
  // Randomize MAC address
  uint8_t newMAC[6];
  WiFi.macAddress(newMAC);
  
  Serial.print("Original MAC: ");
  for (int i = 0; i < 6; i++) {
    if (newMAC[i] < 16) Serial.print("0");
    Serial.print(newMAC[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  
  randomSeed(analogRead(0) + micros());
  for (int i = 0; i < 6; i++) {
    newMAC[i] = random(0, 256);
  }
  newMAC[0] |= 0x02;
  newMAC[0] &= 0xFE;
  
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, newMAC);
  
  Serial.print("Randomized MAC: ");
  for (int i = 0; i < 6; i++) {
    if (newMAC[i] < 16) Serial.print("0");
    Serial.print(newMAC[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  
  esp_log_level_set("*", ESP_LOG_NONE);
  
  preferences.begin("ouispy", true);
  bool factoryReset = preferences.getBool("factoryReset", false);
  preferences.end();
  
  if (factoryReset) {
    Serial.println("FACTORY RESET FLAG DETECTED - Clearing all data...");
    
    nvs_flash_erase();
    nvs_flash_init();
    
    preferences.begin("ouispy", false);
    preferences.clear();
    preferences.putBool("factoryReset", false);
    preferences.end();
    
    targetFilters.clear();
    deviceAliases.clear();
    devices.clear();
    
    if (sdReady) {
      SD.remove("/devices.json");
      SD.remove("/aliases.json");
    }
    
    Serial.println("Factory reset complete");
  }

  Serial.println("DEBUG: About to load configuration...");
  loadConfiguration();
  Serial.println("DEBUG: Configuration loaded, filter count: " + String(targetFilters.size()));
  
  loadWiFiCredentials();
  
  waitForGPSFix();
  
  if (sdReady) {
    initializeFile();
    loadDeviceAliases();
    loadDetectedDevices();
    
    // Create empty SD files if they don't exist
    if (!SD.exists("/aliases.json")) {
      Serial.println("Creating empty aliases.json");
      File f = SD.open("/aliases.json", FILE_WRITE);
      if (f) {
        f.println("[]");
        f.close();
      }
    }
    
    if (!SD.exists("/devices.json")) {
      Serial.println("Creating empty devices.json");  
      File f = SD.open("/devices.json", FILE_WRITE);
      if (f) {
        f.println("[]");
        f.close();
      }
    }
  } else {
    Serial.println("SD not ready, device tracking disabled.");
  }
  
  preferences.begin("ouispy", true);
  bool configLocked = preferences.getBool("configLocked", false);
  preferences.end();
  
  Serial.println("DEBUG: Final filter count before mode selection: " + String(targetFilters.size()));
  
  if (configLocked) {
    Serial.println("======================================");
    Serial.println("CONFIGURATION LOCKED");
    Serial.println("Skipping config mode - going to scanning");
    Serial.println("======================================");
    startScanningMode();
  } else {
    Serial.println("Starting configuration mode...");
    startConfigMode();
  }
  
  // Create scanning task on Core 0
  xTaskCreatePinnedToCore(
    ScanTask,
    "ScanTask",
    8192,
    NULL,
    1,
    &ScanTaskHandle,
    0
  );
}

void loop() {
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }
  
  unsigned long currentMillis = millis();
  
  if (currentMode == CONFIG_MODE) {
    if (normalRestartScheduled > 0 && currentMillis >= normalRestartScheduled) {
      Serial.println("Scheduled restart with locked configuration...");
      delay(500);
      ESP.restart();
    }
    
    if (deviceResetScheduled > 0 && currentMillis >= deviceResetScheduled) {
      Serial.println("Scheduled device reset - setting factory reset flag...");
      preferences.begin("ouispy", false);
      preferences.putBool("factoryReset", true);
      preferences.end();
      delay(500);
      ESP.restart();
    }
    
    if (modeSwitchScheduled > 0 && currentMillis >= modeSwitchScheduled) {
      Serial.println("Scheduled mode switch - switching to scanning mode");
      modeSwitchScheduled = 0;
      startScanningMode();
      return;
    }
    
    if (targetFilters.size() == 0) {
      if (currentMillis - configStartTime > CONFIG_TIMEOUT && lastConfigActivity == configStartTime) {
        Serial.println("No filters - staying in config mode");
        Serial.println("Connect to '" + AP_SSID + "' to configure!");
      }
    } else if (targetFilters.size() > 0) {
      if (currentMillis - configStartTime > CONFIG_TIMEOUT && lastConfigActivity == configStartTime) {
        Serial.println("No connection - using saved filters");
        startScanningMode();
      } else if (lastConfigActivity > configStartTime) {
        static unsigned long lastConnectedMsg = 0;
        if (currentMillis - configStartTime > CONFIG_TIMEOUT && currentMillis - lastConnectedMsg > 30000) {
          Serial.println("Web interface connected - waiting for configuration...");
          lastConnectedMsg = currentMillis;
        }
      }
    }
    
    delay(100);
    return;
  }
  
  delay(100);
}