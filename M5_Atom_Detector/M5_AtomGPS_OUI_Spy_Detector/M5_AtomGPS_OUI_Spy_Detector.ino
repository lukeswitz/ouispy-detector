#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <NimBLEScan.h>
#include <NimBLEDevice.h>
#include <NimBLEAdvertisedDevice.h>
#include <esp_log.h>
#include <esp_wifi.h>
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
bool REQUIRE_GPS_FIX = true;  // set false to skip blocking wait

// WiFi AP Configuration
#define AP_SSID "snoopuntothem"
#define AP_PASSWORD "astheysnoopuntous"
#define CONFIG_TIMEOUT 20000  // 20 seconds timeout for config mode

// Operating Modes
enum OperatingMode {
  CONFIG_MODE,
  SCANNING_MODE
};

// LED Pattern States
enum LEDPattern {
  LED_OFF,
  LED_STATUS_BLINK,
  LED_SINGLE_BLINK,  // Red x1
  LED_DOUBLE_BLINK,  // Blue x2
  LED_TRIPLE_BLINK,  // Green x3
  LED_READY_SIGNAL,  // Purple ascending
  LED_GPS_WAIT       // Purple variable brightness
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

// LED control variables - accessed by both cores
volatile LEDPattern currentLEDPattern = LED_OFF;
volatile int ledPatternParam = 0;  // For GPS satellites count, etc.

// Device tracking
struct DeviceInfo {
  String macAddress;
  int rssi;
  unsigned long firstSeen;
  unsigned long lastSeen;
  bool inCooldown;
  unsigned long cooldownUntil;
  String matchedFilter;
};

struct TargetFilter {
  String identifier;
  bool isFullMAC;
  String description;
};

std::vector<DeviceInfo> devices;
std::vector<TargetFilter> targetFilters;

// Forward declarations
void startScanningMode();
class MyAdvertisedDeviceCallbacks;

// FreeRTOS task definitions
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

// LED Task - runs on Core 0
void LEDTask(void *pvParameters) {
  static unsigned long previousMillis = 0;
  static unsigned long patternStartTime = 0;
  static bool patternActive = false;
  
  while (1) {
    unsigned long currentMillis = millis();
    
    switch (currentLEDPattern) {
      case LED_OFF:
        M5.dis.drawpix(0, 0x000000);  // Black
        patternActive = false;
        break;
        
      case LED_STATUS_BLINK:
        {
          unsigned long interval = (currentMode == CONFIG_MODE) ? 10000 : 15000;
          if (currentMillis - previousMillis >= interval) {
            M5.dis.drawpix(0, 0xFFFFFF);  // White
            vTaskDelay(50 / portTICK_PERIOD_MS);
            M5.dis.drawpix(0, 0x000000);  // Black
            previousMillis = currentMillis;
          }
        }
        break;
        
      case LED_SINGLE_BLINK: // Red x1
        if (!patternActive) {
          patternStartTime = currentMillis;
          patternActive = true;
        }
        {
          unsigned long elapsed = currentMillis - patternStartTime;
          if (elapsed <= 200) {
            M5.dis.drawpix(0, 0x0000FF);  // RED (note: GRB format, so 0x0000FF = red)
          } else if (elapsed <= 400) {
            M5.dis.drawpix(0, 0x000000);  // Black
          } else {
            M5.dis.drawpix(0, 0x000000);
            currentLEDPattern = LED_OFF;
            patternActive = false;
          }
        }
        break;
        
      case LED_DOUBLE_BLINK: // Blue x2
        {
          if (!patternActive) {
            patternStartTime = currentMillis;
            patternActive = true;
          }
          
          unsigned long elapsed = currentMillis - patternStartTime;
          
          if (elapsed < 150) {
            M5.dis.drawpix(0, 0xFF0000);  // BLUE (note: GRB format)
          } else if (elapsed < 300) {
            M5.dis.drawpix(0, 0x000000);
          } else if (elapsed < 450) {
            M5.dis.drawpix(0, 0xFF0000);  // BLUE
          } else if (elapsed < 600) {
            M5.dis.drawpix(0, 0x000000);
          } else {
            M5.dis.drawpix(0, 0x000000);
            currentLEDPattern = LED_OFF;
            patternActive = false;
          }
        }
        break;
        
      case LED_TRIPLE_BLINK: // Green x3
        {
          if (!patternActive) {
            patternStartTime = currentMillis;
            patternActive = true;
          }
          
          unsigned long elapsed = currentMillis - patternStartTime;
          
          if (elapsed < 150) {
            M5.dis.drawpix(0, 0x00FF00);  // GREEN (GRB format)
          } else if (elapsed < 300) {
            M5.dis.drawpix(0, 0x000000);
          } else if (elapsed < 450) {
            M5.dis.drawpix(0, 0x00FF00);  // GREEN
          } else if (elapsed < 600) {
            M5.dis.drawpix(0, 0x000000);
          } else if (elapsed < 750) {
            M5.dis.drawpix(0, 0x00FF00);  // GREEN
          } else if (elapsed < 900) {
            M5.dis.drawpix(0, 0x000000);
          } else {
            M5.dis.drawpix(0, 0x000000);
            currentLEDPattern = LED_OFF;
            patternActive = false;
          }
        }
        break;
        
      case LED_READY_SIGNAL: // Purple fade
        {
          if (!patternActive) {
            patternStartTime = currentMillis;
            patternActive = true;
          }
          
          unsigned long elapsed = currentMillis - patternStartTime;
          if (elapsed > 2000) { // 2 second fade
            M5.dis.drawpix(0, 0x000000);
            currentLEDPattern = LED_OFF;
            patternActive = false;
          } else {
            // Simple purple fade
            int brightness = (elapsed < 1000) ? (elapsed * 255 / 1000) : (255 - ((elapsed - 1000) * 255 / 1000));
            uint32_t purple = (brightness << 8) | brightness;  // Purple in GRB format
            M5.dis.drawpix(0, purple);
          }
        }
        break;
        
      case LED_GPS_WAIT: // Purple variable brightness
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
            
            uint32_t purple = (brightness << 8) | brightness;  // Purple in GRB format  
            M5.dis.drawpix(0, purple);
            previousMillis = currentMillis;
          }
        }
        break;
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Scanning Task - runs on Core 1
void ScanTask(void* pvParameters) {
  static unsigned long lastScanTime = 0;
  static unsigned long lastCleanupTime = 0;
  static unsigned long lastStatusTime = 0;

  while (1) {
    if (currentMode == SCANNING_MODE) {
      unsigned long currentMillis = millis();

      // Restart BLE scan every 3 seconds
      if (currentMillis - lastScanTime >= 3000) {
        if (pBLEScan) {
          pBLEScan->stop();
          vTaskDelay(10 / portTICK_PERIOD_MS);
          pBLEScan->start(2, false, false);
        }
        lastScanTime = currentMillis;
      }

      // Clean up expired devices every 10 seconds
      if (currentMillis - lastCleanupTime >= 10000) {
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
        lastCleanupTime = currentMillis;
      }

      // Status report every 30 seconds
      if (currentMillis - lastStatusTime >= 30000) {
        Serial.println("Status: Scanning - " + String(devices.size()) + " active devices tracked");
        lastStatusTime = currentMillis;
      }
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);  // 100ms delay
  }
}

// LED Pattern Functions (now just set the pattern, don't block)
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
  Serial.println("Waiting for GPS fix...");
  while (REQUIRE_GPS_FIX && !gps.location.isValid()) {
    while (Serial1.available() > 0) gps.encode(Serial1.read());
    startGPSWaitPattern(gps.satellites.value());
    delay(100);
  }
  stopGPSWaitPattern();
  Serial.println("GPS fix obtained or bypassed.");
}

void startupTest() {
  
  M5.dis.drawpix(0, 0xFFFFFF);  // White
  delay(300);
  M5.dis.drawpix(0, 0x000000);  // Black
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
  int y = gps.date.isValid() ? gps.date.year() : 1970;
  int m = gps.date.isValid() ? gps.date.month() : 1;
  int d = gps.date.isValid() ? gps.date.day() : 1;

  char dateStamp[16];
  sprintf(dateStamp, "%04d-%02d-%02d-", y, m, d);

  int fileNumber = 0;
  bool isNewFile = false;
  do {
    snprintf(fileName, sizeof(fileName), "/FoxHunt-%s%d.csv", dateStamp, fileNumber);
    isNewFile = !SD.exists(fileName);
    fileNumber++;
  } while (!isNewFile);

  File f = SD.open(fileName, FILE_WRITE);
  if (f) {
    f.println("WhenUTC,MatchType,MAC,RSSI,Lat,Lon,AltM,HDOP,Filter");
    f.close();
    Serial.println(String("Log file: ") + fileName);
  } else {
    Serial.println("Failed to create log file.");
  }
}

void logMatchRow(const String& type, const String& mac, int rssi, const String& filt) {
  if (!sdReady) return;

  char utc[21];
  if (gps.date.isValid() && gps.time.isValid()) {
    sprintf(utc, "%04d-%02d-%02d %02d:%02d:%02d",
            gps.date.year(), gps.date.month(), gps.date.day(),
            gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    sprintf(utc, "1970-01-01 00:00:00");
  }

  double lat = gps.location.isValid() ? gps.location.lat() : 0.0;
  double lon = gps.location.isValid() ? gps.location.lng() : 0.0;
  double alt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  double hdop = gps.hdop.isValid() ? gps.hdop.hdop() : -1.0;

  char line[512];
  snprintf(line, sizeof(line),
           "%s,%s,%s,%d,%.6f,%.6f,%.2f,%.2f,%s",
           utc, type.c_str(), mac.c_str(), rssi, lat, lon, alt, hdop, filt.c_str());

  File f = SD.open(fileName, FILE_APPEND);
  if (f) {
    f.println(line);
    f.close();
  } else {
    Serial.println(String("Error opening ") + fileName);
  }
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

  // Check for valid OUI (8 chars) or full MAC (17 chars)
  if (normalized.length() != 8 && normalized.length() != 17) {
    return false;
  }

  // Basic format validation
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
  preferences.begin("ouispy", false);
  preferences.putInt("filterCount", targetFilters.size());

  for (int i = 0; i < targetFilters.size(); i++) {
    String keyId = "id_" + String(i);
    String keyMAC = "mac_" + String(i);
    String keyDesc = "desc_" + String(i);

    preferences.putString(keyId.c_str(), targetFilters[i].identifier);
    preferences.putBool(keyMAC.c_str(), targetFilters[i].isFullMAC);
    preferences.putString(keyDesc.c_str(), targetFilters[i].description);
  }

  preferences.end();
  Serial.println("Configuration saved to NVS");
}

void loadConfiguration() {
  preferences.begin("ouispy", true);
  int filterCount = preferences.getInt("filterCount", 0);

  targetFilters.clear();

  // Load saved filters or use defaults
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
  } else {
    // Default configuration with some common OUIs
    targetFilters.push_back({ "AA:BB:CC", false, "Example OUI 1" });
    targetFilters.push_back({ "DD:EE:FF", false, "Example OUI 2" });
    targetFilters.push_back({ "AA:BB:CC:12:34:56", true, "Specific Device" });
  }

  preferences.end();
  Serial.println("Configuration loaded - " + String(targetFilters.size()) + " filters");
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

  String randomOUIExamples =
    generateRandomOUI() + "\n" + generateRandomOUI() + "\n" + generateRandomOUI();
  String randomMACExamples =
    generateRandomMAC() + "\n" + generateRandomMAC() + "\n" + generateRandomMAC();

  String html = R"html(
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
        .ascii-background {
            position: fixed; top: 0; left: 0; width: 100%; height: 100%;
            z-index: -1; opacity: 0.3; color: #ff6b35;
            font-family: monospace; font-size: 6px;
            line-height: 6px; white-space: pre; pointer-events: none;
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
        .ascii {
            font-family: monospace; font-size: 8px;
            color: #ff6b35; text-align: center; margin-bottom: 30px;
            white-space: pre; opacity: 0.8;
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
        @media (max-width: 768px) {
            h1 { font-size: 28px; }
            .container { padding: 20px; margin: 10px; }
        }
    </style>
</head>
<body>
    <div class="ascii-background">)html"
                + String(getASCIIArt()).substring(0, 2000) + R"html(</div>
    <div class="container">
        <div class="ascii">)html"
                + String(getASCIIArt()) + R"html(</div>
        <h1>M5GPS OUI-SPY</h1>
        
        <div class="status">
            Configure MAC addresses and OUI prefixes to detect. LED patterns indicate detection type:
            <br><strong>Green x 3:</strong> New device or re-detected after 30s
            <br><strong>Blue x 2:</strong> Re-detected after 5s
            <br><strong>Red x 1:</strong> Status blink
        </div>

        <form method="POST" action="/save">
            <div class="section">
                <h3>OUI Prefixes (Manufacturer Detection)</h3>
                <textarea name="ouis" placeholder="Enter OUI prefixes, one per line:
)html" + randomOUIExamples
                + R"html(">)html" + ouiValues + R"html(</textarea>
                <div class="help-text">
                    OUI prefixes (first 3 bytes) detect all devices from a manufacturer.<br>
                    Format: XX:XX:XX (8 characters with colons)<br>
                    Example: 58:2D:34 for Espressif devices
                </div>
            </div>
            
            <div class="section">
                <h3>Specific MAC Addresses</h3>
                <textarea name="macs" placeholder="Enter full MAC addresses, one per line:
)html" + randomMACExamples
                + R"html(">)html" + macValues + R"html(</textarea>
                <div class="help-text">
                    Full MAC addresses detect specific devices only.<br>
                    Format: XX:XX:XX:XX:XX:XX (17 characters with colons)<br>
                    Example: 58:2D:34:12:AB:CD for a specific ESP32
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
            if (confirm('Clear all filters? This action cannot be undone.')) {
                document.querySelector('textarea[name="ouis"]').value = '';
                document.querySelector('textarea[name="macs"]').value = '';
                fetch('/clear', { method: 'POST' })
                    .then(() => { alert('All filters cleared!'); location.reload(); })
                    .catch(error => alert('Error: ' + error));
            }
        }
        
        function deviceReset() {
            if (confirm('DEVICE RESET: Complete wipe and restart. Are you sure?')) {
                if (confirm('This cannot be undone. Continue?')) {
                    fetch('/device-reset', { method: 'POST' })
                        .then(() => {
                            alert('Device resetting...');
                            setTimeout(() => window.location.href = '/', 5000);
                        });
                }
            }
        }
        </script>
    </div>
</body>
</html>
)html";

  return html;
}

void startConfigMode() {
  currentMode = CONFIG_MODE;

  Serial.println("\n=== STARTING M5 ATOM DETECTOR CONFIG MODE ===");
  Serial.println("SSID: " + String(AP_SSID));
  Serial.println("Password: " + String(AP_PASSWORD));

  WiFi.mode(WIFI_OFF);
  delay(1000);
  WiFi.mode(WIFI_AP);
  delay(500);

  bool apStarted = WiFi.softAP(AP_SSID, AP_PASSWORD);
  if (!apStarted) {
    Serial.println("Failed to create Access Point!");
    return;
  }

  delay(2000);
  IPAddress IP = WiFi.softAPIP();
  Serial.println("AP IP address: " + IP.toString());
  Serial.println("Config portal: http://" + IP.toString());

  configStartTime = millis();
  lastConfigActivity = millis();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    lastConfigActivity = millis();
    request->send(200, "text/html; charset=utf-8", generateConfigHTML());
  });

  server.on("/save", HTTP_POST, [](AsyncWebServerRequest* request) {
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
            const el = document.getElementById('countdown');
            if (el) el.innerHTML = 'Switching to scanning mode now...';
        }, 5000);
    </script>
</head>
<body>
    <div class="container">
        <h1>Configuration Saved</h1>
        <div class="success">
            <p><strong>Saved )html"
                            + String(targetFilters.size()) + R"html( filters successfully!</strong></p>
            <p id="countdown">Switching to scanning mode in 5 seconds...</p>
        </div>
        <p>M5 Atom will now start scanning for your configured devices.</p>
        <p>Watch for LED patterns when matches are found!</p>
    </div>
</body>
</html>
)html";

      request->send(200, "text/html; charset=utf-8", responseHTML);
      modeSwitchScheduled = millis() + 5000;
    } else {
      request->send(400, "text/html; charset=utf-8", "<h1>Error: No valid filters provided</h1>");
    }
  });

  server.on("/clear", HTTP_POST, [](AsyncWebServerRequest* request) {
    lastConfigActivity = millis();
    targetFilters.clear();
    saveConfiguration();
    request->send(200, "text/plain; charset=utf-8", "Filters cleared successfully");
  });

  server.on("/device-reset", HTTP_POST, [](AsyncWebServerRequest* request) {
    lastConfigActivity = millis();
    request->send(
      200, "text/html; charset=utf-8",
      "<html><head><meta charset='UTF-8'></head>"
      "<body style='background:#0a0a0a;color:#ff6b35;font-family:Arial;text-align:center;padding:50px;'>"
      "<h1>Device Reset Complete</h1>"
      "<p>Device restarting in 3 seconds...</p>"
      "<script>setTimeout(function(){window.location.href='/'},5000);</script>"
      "</body></html>");
    deviceResetScheduled = millis() + 3000;
  });
  Serial.println("==Web server started===");
  server.begin();
}


/// BLE Advertised Device Callback Class
class MyAdvertisedDeviceCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    if (currentMode != SCANNING_MODE) return;

    String mac = advertisedDevice->getAddress().toString().c_str();
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

        unsigned long dt = currentMillis - dev.lastSeen;

        if (dt >= 30000) {
          // Trigger LED pattern without blocking
          triggerTripleBlink();
          Serial.println("RE-DETECTED after 30+ sec: " + matchedDescription);
          Serial.println("MAC: " + mac + " | RSSI: " + String(rssi));
          logMatchRow("RE-30s", mac, rssi, matchedDescription);
          dev.inCooldown = true;
          dev.cooldownUntil = currentMillis + 10000;
        } else if (dt >= 5000) {
          // Trigger LED pattern without blocking
          triggerDoubleBlink();
          Serial.println("RE-DETECTED after 5+ sec: " + matchedDescription);
          Serial.println("MAC: " + mac + " | RSSI: " + String(rssi));
          logMatchRow("RE-5s", mac, rssi, matchedDescription);
          dev.inCooldown = true;
          dev.cooldownUntil = currentMillis + 5000;
        }

        dev.lastSeen = currentMillis;
        break;
      }
    }

    if (!known) {
      DeviceInfo newDev{ mac, rssi, currentMillis, currentMillis, false, 0, matchedDescription };
      devices.push_back(newDev);

      // Trigger LED pattern without blocking
      triggerTripleBlink();
      Serial.println("NEW DEVICE DETECTED: " + matchedDescription);
      Serial.println("MAC: " + mac + " | RSSI: " + String(rssi));
      logMatchRow("NEW", mac, rssi, matchedDescription);

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

  Serial.println("\n=== STARTING M5 ATOM SCANNING MODE ===");
  for (const TargetFilter& filter : targetFilters) {
    String type = filter.isFullMAC ? "Full MAC" : "OUI";
    Serial.println("- " + filter.identifier + " (" + type + "): " + filter.description);
  }
  Serial.println("==============================\n");

  NimBLEDevice::init("");
  delay(1000);

  pBLEScan = NimBLEDevice::getScan();
  if (pBLEScan != nullptr) {
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(300);
    pBLEScan->setWindow(200);
    pBLEScan->setScanCallbacks(new MyAdvertisedDeviceCallbacks());
  }

  delay(500);
  triggerReadySignal();
  delay(2000);

  if (pBLEScan != nullptr) {
    pBLEScan->start(3, false, false);
    Serial.println("BLE scanning started!");
    startStatusBlinking();  // Start status blinking pattern
  }
}

void setup() {
  delay(2000);
  Serial.begin(115200);
  delay(1000);

  M5.begin(true, false, true); 
  delay(50);  
  
  startupTest();

  // Create LED task on Core 0 (protocol core)
  xTaskCreatePinnedToCore(
    LEDTask,         // Function to implement the task
    "LEDTask",       // Name of the task
    2048,            // Stack size in bytes
    NULL,            // Task input parameter
    1,               // Priority of the task
    &LEDTaskHandle,  // Task handle
    0                // Core where the task should run (0)
  );


  // Power down radio while doing SD init
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
  Serial.println("LED: GPIO27 (Single RGB LED)");
  Serial.println("Mode: Multi-target BLE device detection");
  Serial.println("Patterns: Green×3 (new), Blue×2 (5s), Red×1 (status)");
  Serial.println("Initializing...\n");

  // Randomize MAC address for stealth
  uint8_t newMAC[6];
  WiFi.macAddress(newMAC);

  Serial.print("Original MAC: ");
  for (int i = 0; i < 6; i++) {
    if (newMAC[i] < 16) Serial.print("0");
    Serial.print(newMAC[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  // Randomize ALL 6 bytes for maximum anonymity
  randomSeed(analogRead(0) + micros());
  for (int i = 0; i < 6; i++) {
    newMAC[i] = random(0, 256);
  }
  // Ensure valid locally administered address
  newMAC[0] |= 0x02;  // Set locally administered bit
  newMAC[0] &= 0xFE;  // Clear multicast bit

  // Set randomized MAC for both modes
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, newMAC);

  Serial.print("Randomized MAC: ");
  for (int i = 0; i < 6; i++) {
    if (newMAC[i] < 16) Serial.print("0");
    Serial.print(newMAC[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  // Silence ESP-IDF logs
  esp_log_level_set("*", ESP_LOG_NONE);

  // Check for factory reset flag
  preferences.begin("ouispy", true);
  bool factoryReset = preferences.getBool("factoryReset", false);
  preferences.end();

  if (factoryReset) {
    Serial.println("FACTORY RESET FLAG DETECTED - Clearing all data...");
    preferences.begin("ouispy", false);
    preferences.clear();
    preferences.end();
    targetFilters.clear();
    Serial.println("Factory reset complete - starting with clean state");
  } else {
    Serial.println("Loading configuration...");
    loadConfiguration();
  }

  // Wait for GPS fix so the log filename gets the real date
  waitForGPSFix();  // blocks until fix if REQUIRE_GPS_FIX = true

  // Create the daily CSV file if SD is ready
  if (sdReady) {
    initializeFile();
  } else {
    Serial.println("SD not ready, logging disabled.");
  }

  // Configuration AP and web UI
  Serial.println("Starting configuration mode...");
  startConfigMode();

  // Create scanning task on Core 1 (application core)
  xTaskCreatePinnedToCore(
    ScanTask,         // Function to implement the task
    "ScanTask",       // Name of the task
    4096,             // Stack size in bytes
    NULL,             // Task input parameter
    2,                // Priority of the task (higher than LED)
    &ScanTaskHandle,  // Task handle
    1                 // Core where the task should run (1)
  );
}

void loop() {
  // Handle GPS reading on main loop
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  unsigned long currentMillis = millis();

  if (currentMode == CONFIG_MODE) {
    // Handle scheduled device reset
    if (deviceResetScheduled > 0 && currentMillis >= deviceResetScheduled) {
      Serial.println("Scheduled device reset - setting factory reset flag and restarting...");
      preferences.begin("ouispy", false);
      preferences.putBool("factoryReset", true);
      preferences.end();
      delay(500);
      ESP.restart();
    }

    // Handle scheduled mode switch
    if (modeSwitchScheduled > 0 && currentMillis >= modeSwitchScheduled) {
      Serial.println("Scheduled mode switch - switching to scanning mode");
      modeSwitchScheduled = 0;
      startScanningMode();
      return;
    }

    // Config timeout logic
    if (targetFilters.size() == 0) {
      if (currentMillis - configStartTime > CONFIG_TIMEOUT && lastConfigActivity == configStartTime) {
        Serial.println("No saved filters - staying in config mode indefinitely");
        Serial.println("Connect to 'snoopuntothem' AP to configure filters!");
      }
    } else if (targetFilters.size() > 0) {
      if (currentMillis - configStartTime > CONFIG_TIMEOUT && lastConfigActivity == configStartTime) {
        Serial.println("No connection within 20s - using saved filters, switching to scanning");
        startScanningMode();
      } else if (lastConfigActivity > configStartTime) {
        static unsigned long lastConnectedMsg = 0;
        if (currentMillis - configStartTime > CONFIG_TIMEOUT && currentMillis - lastConnectedMsg > 30000) {
          Serial.println("Web interface connected - waiting for configuration...");
          lastConnectedMsg = currentMillis;
        }
      }
    }

    // Status blink in config mode every 10 seconds
    static unsigned long lastConfigBlink = 0;
    if (currentMillis - lastConfigBlink >= 10000) {
      triggerSingleBlink();
      lastConfigBlink = currentMillis;
    }
  }

  delay(100);
}