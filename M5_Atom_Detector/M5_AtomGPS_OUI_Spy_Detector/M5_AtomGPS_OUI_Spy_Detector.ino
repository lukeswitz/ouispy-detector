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
#include <FastLED.h>
#include <M5Atom.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPS++.h>


// Hardware Configuration - M5 Atom Lite
#define LED_PIN 27
#undef NUM_LEDS
#define LED_COUNT 1
#define LED_BRIGHTNESS 80

// LED setup
CRGB leds[LED_COUNT];

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


// Global Variables
OperatingMode currentMode = CONFIG_MODE;
AsyncWebServer server(80);
Preferences preferences;
NimBLEScan* pBLEScan;
unsigned long configStartTime = 0;
unsigned long lastConfigActivity = 0;
unsigned long modeSwitchScheduled = 0;
unsigned long deviceResetScheduled = 0;

// LED blink synchronization - avoid concurrent operations
volatile bool newMatchFound = false;
String detectedMAC = "";
int detectedRSSI = 0;
String matchedFilter = "";
String matchType = "";  // "NEW", "RE-5s", "RE-30s"

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


// LED Pattern Functions

void initializeLED() {
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, LED_COUNT);
  FastLED.setBrightness(LED_BRIGHTNESS);
  FastLED.clear();
  FastLED.show();
}

void singleBlink() {
  leds[0] = CRGB::Red;
  FastLED.show();
  delay(200);
  leds[0] = CRGB::Black;
  FastLED.show();
}

void doubleBlink() {
  // Two fast blue blinks for re-detection after 5+ seconds
  for (int i = 0; i < 2; i++) {
    leds[0] = CRGB::Blue;
    FastLED.show();
    delay(150);
    leds[0] = CRGB::Black;
    FastLED.show();
    if (i < 1) delay(150);
  }
}

void tripleBlink() {
  // Three fast green blinks for new device or re-detection after 30+ seconds
  for (int i = 0; i < 3; i++) {
    leds[0] = CRGB::Green;
    FastLED.show();
    delay(150);
    leds[0] = CRGB::Black;
    FastLED.show();
    if (i < 2) delay(150);
  }
}

void readySignal() {
  // Purple ascending brightness pattern to indicate "ready to scan"
  for (int brightness = 0; brightness <= 150; brightness += 10) {
    leds[0] = CRGB(brightness / 2, 0, brightness);  // Purple gradient
    FastLED.show();
    delay(50);
  }
  for (int brightness = 150; brightness >= 0; brightness -= 10) {
    leds[0] = CRGB(brightness / 2, 0, brightness);
    FastLED.show();
    delay(50);
  }
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(500);
}

void blinkLEDFaster(int numSat) {
  // Similar pattern to readySignal but simplified for the waiting phase
  static unsigned long prev = 0;
  static bool on = false;
  static int brightness = 0;
  static bool ascending = true;
  
  // Adjust timing based on satellite count, but keep it relatively smooth
  unsigned long interval = (numSat <= 1) ? 100UL : max(50UL, 300UL / (unsigned long)numSat);
  
  unsigned long now = millis();
  if (now - prev >= interval) {
    prev = now;
    
    if (ascending) {
      // Gradually increase brightness
      brightness += 5;
      if (brightness >= 100) {
        ascending = false;
      }
    } else {
      // Gradually decrease brightness
      brightness -= 5;
      if (brightness <= 0) {
        ascending = true;
        // Complete one cycle
        on = !on;  // Toggle for satellite count effect
      }
    }
    
    // Purple with variable brightness
    leds[0] = on ? CRGB(brightness/2, 0, brightness) : CRGB::Black;
    FastLED.show();
  }
}

void waitForGPSFix() {
  Serial.println("Waiting for GPS fix...");
  while (REQUIRE_GPS_FIX && !gps.location.isValid()) {
    while (Serial1.available() > 0) gps.encode(Serial1.read());
    blinkLEDFaster(gps.satellites.value());
    delay(100);
  }
  leds[0] = CRGB::Black;
  FastLED.show();
  Serial.println("GPS fix obtained or bypassed.");
}

void startupTest() {
  // Quick LED test on startup
  leds[0] = CRGB::White;
  FastLED.show();
  delay(300);
  leds[0] = CRGB::Black;
  FastLED.show();
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


// BLE Advertised Device Callback Class

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
          detectedMAC = mac;
          detectedRSSI = rssi;
          matchedFilter = matchedDescription;
          matchType = "RE-30s";
          newMatchFound = true;
          tripleBlink();
          dev.inCooldown = true;
          dev.cooldownUntil = currentMillis + 10000;
        } else if (dt >= 5000) {
          detectedMAC = mac;
          detectedRSSI = rssi;
          matchedFilter = matchedDescription;
          matchType = "RE-5s";
          newMatchFound = true;
          doubleBlink();
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

      detectedMAC = mac;
      detectedRSSI = rssi;
      matchedFilter = matchedDescription;
      matchType = "NEW";
      newMatchFound = true;
      tripleBlink();

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
  readySignal();
  delay(2000);

  if (pBLEScan != nullptr) {
    // bool start(uint32_t duration, bool isContinue=false, bool stayActive=true)
    pBLEScan->start(3, false, false);
    Serial.println("BLE scanning started!");
  }
}

void setup() {
  delay(2000);

  // Initialize Serial
  Serial.begin(115200);
  delay(1000);

  M5.begin(true, false, true);
  initializeLED();
  startupTest();

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
}


void loop() {
  // Wait on GPS
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  static unsigned long lastScanTime = 0;
  static unsigned long lastCleanupTime = 0;
  static unsigned long lastStatusTime = 0;
  static unsigned long lastStatusBlink = 0;
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
      // No saved filters - stay in config mode
      if (currentMillis - configStartTime > CONFIG_TIMEOUT && lastConfigActivity == configStartTime) {
        Serial.println("No saved filters - staying in config mode indefinitely");
        Serial.println("Connect to 'snoopuntothem' AP to configure filters!");
      }
    } else if (targetFilters.size() > 0) {
      // Have saved filters - timeout if no one connected
      if (currentMillis - configStartTime > CONFIG_TIMEOUT && lastConfigActivity == configStartTime) {
        Serial.println("No connection within 20s - using saved filters, switching to scanning");
        startScanningMode();
      } else if (lastConfigActivity > configStartTime) {
        // Someone connected - wait for submission
        static unsigned long lastConnectedMsg = 0;
        if (currentMillis - configStartTime > CONFIG_TIMEOUT && currentMillis - lastConnectedMsg > 30000) {
          Serial.println("Web interface connected - waiting for configuration...");
          lastConnectedMsg = currentMillis;
        }
      }
    }

    // Status blink in config mode every 10 seconds
    if (currentMillis - lastStatusBlink >= 10000) {
      singleBlink();
      lastStatusBlink = currentMillis;
    }

    delay(100);
    return;
  }

  // Scanning mode loop
  if (currentMode == SCANNING_MODE) {
    if (newMatchFound) {
      Serial.println(">> Match found! <<");
      Serial.print("Device: " + detectedMAC);
      Serial.print(" | RSSI: " + String(detectedRSSI));
      Serial.println(" | Filter: " + matchedFilter);

      if (matchType == "NEW") {
        Serial.println("NEW DEVICE DETECTED: " + matchedFilter);
        Serial.println("MAC: " + detectedMAC);
      } else if (matchType == "RE-30s") {
        Serial.println("RE-DETECTED after 30+ sec: " + matchedFilter);
      } else if (matchType == "RE-5s") {
        Serial.println("RE-DETECTED after 5+ sec: " + matchedFilter);
      }

      // Log to SD with GPS
      logMatchRow(matchType, detectedMAC, detectedRSSI, matchedFilter);

      Serial.println("==============================");
      newMatchFound = false;
    }

    // Restart BLE scan every 3 seconds
    if (currentMillis - lastScanTime >= 3000) {
      if (pBLEScan) {
        pBLEScan->stop();
        delay(10);
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

    // Subtle status blink every 15 seconds in scanning mode
    if (currentMillis - lastStatusBlink >= 15000) {
      leds[0] = CRGB(10, 10, 10);  // Very dim white
      FastLED.show();
      delay(50);
      leds[0] = CRGB::Black;
      FastLED.show();
      lastStatusBlink = currentMillis;
    }
  }

  delay(100);
}