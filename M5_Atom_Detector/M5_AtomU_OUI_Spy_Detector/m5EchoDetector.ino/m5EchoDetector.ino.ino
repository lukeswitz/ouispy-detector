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
#include <driver/i2s_std.h>

// Hardware Configuration - Atom Echo
#define I2S_BCLK 19
#define I2S_LRCK 33
#define I2S_DOUT 22
#define I2S_NUM I2S_NUM_0

// FreeRTOS task handles
TaskHandle_t AudioLEDTaskHandle = NULL;
TaskHandle_t ScanTaskHandle = NULL;
const size_t MAX_DEVICES = 100;

// WiFi AP Configuration
String AP_SSID = "snoopuntothem";
String AP_PASSWORD = "astheysnoopuntous";
#define CONFIG_TIMEOUT 20000

// Operating Modes
enum OperatingMode {
  CONFIG_MODE,
  SCANNING_MODE
};

// RGB LED Effects
enum RGBEffect {
  RGB_OFF,
  RGB_CONFIG_PULSE,
  RGB_SCANNING_RAINBOW,
  RGB_GPS_WAIT,
  RGB_DETECTION_BURST,
  RGB_REDETECT_FLASH,
  RGB_READY_FADE,
  RGB_ERROR_BLINK
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

// Audio & LED control variables
volatile RGBEffect currentEffect = RGB_OFF;
volatile int effectParam = 0;
volatile unsigned long effectStartTime = 0;
bool audioEnabled = true;

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
void performWiFiScan();
void processWiFiResults(int networksFound);
bool matchesTargetFilter(const String& deviceMAC, String& matchedDescription);
void triggerDetectionBurst();
void triggerRedetectFlash();
void triggerReadyFade();
void triggerErrorBlink();
void startConfigPulse();
void startScanningRainbow();

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif
i2s_chan_handle_t tx_handle = NULL;


// I2S Speaker Functions
void initI2SSpeaker() {
  i2s_chan_config_t chan_cfg = {
    .id = I2S_NUM_0,
    .role = I2S_ROLE_MASTER,
    .dma_desc_num = 8,
    .dma_frame_num = 64,
    .auto_clear = true,
  };
  
  i2s_std_config_t std_cfg = {
    .clk_cfg = {
      .sample_rate_hz = 16000,
      .clk_src = I2S_CLK_SRC_DEFAULT,
      .mclk_multiple = I2S_MCLK_MULTIPLE_256,
    },
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_BCLK,
      .ws = (gpio_num_t)I2S_LRCK,
      .dout = (gpio_num_t)I2S_DOUT,
      .din = I2S_GPIO_UNUSED,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false,
      },
    },
  };
  
  i2s_new_channel(&chan_cfg, &tx_handle, NULL);
  i2s_channel_init_std_mode(tx_handle, &std_cfg);
  i2s_channel_enable(tx_handle);
}

void playTone(int frequency, int duration) {
  if (!audioEnabled || tx_handle == NULL) return;
  
  const int sampleRate = 16000;
  const int samples = (sampleRate * duration) / 1000;
  int16_t sample;
  size_t bytes_written;
  
  for (int i = 0; i < samples; i++) {
    sample = (int16_t)(sin(2.0 * PI * frequency * i / sampleRate) * 8000);
    i2s_channel_write(tx_handle, &sample, sizeof(sample), &bytes_written, portMAX_DELAY);
  }
  
  sample = 0;
  for (int i = 0; i < 500; i++) {
    i2s_channel_write(tx_handle, &sample, sizeof(sample), &bytes_written, portMAX_DELAY);
  }
}

void playAscendingTones() {
  playTone(1800, 80);
  delay(30);
  playTone(2200, 80);
  delay(30);
  playTone(2600, 120);
}

void playDetectionAlert() {
  playTone(2400, 100);
  delay(40);
  playTone(2800, 100);
  delay(40);
  playTone(3200, 120);
}

void playRedetectAlert() {
  playTone(2000, 100);
  delay(50);
  playTone(2400, 100);
}

void playReadyChime() {
  playTone(1600, 80);
  delay(30);
  playTone(2000, 80);
  delay(30);
  playTone(2400, 100);
}

void playErrorTone() {
  playTone(800, 200);
  delay(100);
  playTone(800, 200);
}

// RGB Color Utilities
uint32_t hsvToRgb(float h, float s, float v) {
  float r, g, b;
  
  int i = int(h * 6);
  float f = h * 6 - i;
  float p = v * (1 - s);
  float q = v * (1 - f * s);
  float t = v * (1 - (1 - f) * s);
  
  switch (i % 6) {
    case 0: r = v; g = t; b = p; break;
    case 1: r = q; g = v; b = p; break;
    case 2: r = p; g = v; b = t; break;
    case 3: r = p; g = q; b = v; break;
    case 4: r = t; g = p; b = v; break;
    case 5: r = v; g = p; b = q; break;
  }
  
  uint8_t red = r * 255;
  uint8_t green = g * 255;
  uint8_t blue = b * 255;
  return (green << 16) | (red << 8) | blue;
}

uint32_t colorWheel(uint8_t pos) {
  uint8_t r, g, b;
  if (pos < 85) {
    r = pos * 3;
    g = 255 - pos * 3;
    b = 0;
  } else if (pos < 170) {
    pos -= 85;
    r = 255 - pos * 3;
    g = 0;
    b = pos * 3;
  } else {
    pos -= 170;
    r = 0;
    g = pos * 3;
    b = 255 - pos * 3;
  }
  return (g << 16) | (r << 8) | b;
}

// Audio & LED Task
void AudioLEDTask(void* pvParameters) {
  static unsigned long lastUpdate = 0;
  static float animationPhase = 0.0;
  static uint8_t wheelPosition = 0;
  
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  
  while (1) {
    unsigned long currentMillis = millis();
    
    UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    if (stackHighWaterMark < 100) {
      Serial.println("WARNING: AudioLED Task stack low: " + String(stackHighWaterMark));
    }
    
    if (currentMillis - lastUpdate >= 20) {
      lastUpdate = currentMillis;
      
      switch (currentEffect) {
        case RGB_OFF:
          M5.dis.clear();
          break;
          
        case RGB_CONFIG_PULSE:
          {
            float breath = (sin(currentMillis * 0.003) + 1.0) / 2.0;
            uint8_t brightness = 50 + (breath * 205);
            uint8_t r = brightness;
            uint8_t g = brightness * 0.5;
            uint8_t b = 0;
            M5.dis.drawpix(0, (g << 16) | (r << 8) | b);
          }
          break;
          
        case RGB_SCANNING_RAINBOW:
          {
            wheelPosition += 2;
            M5.dis.drawpix(0, colorWheel(wheelPosition));
          }
          break;
          
        case RGB_GPS_WAIT:
          {
            float pulse = (sin(currentMillis * 0.005) + 1.0) / 2.0;
            uint8_t brightness = 30 + (pulse * 225);
            uint8_t r = 0;
            uint8_t g = brightness * 0.3;
            uint8_t b = brightness;
            M5.dis.drawpix(0, (g << 16) | (r << 8) | b);
          }
          break;
          
        case RGB_DETECTION_BURST:
          {
            unsigned long elapsed = currentMillis - effectStartTime;
            
            if (elapsed < 300) {
              float intensity = 1.0 - (elapsed / 300.0);
              uint8_t brightness = 255 * intensity;
              uint8_t r = 0;
              uint8_t g = brightness * 0.8;
              uint8_t b = brightness;
              M5.dis.drawpix(0, (g << 16) | (r << 8) | b);
            } else if (elapsed < 600) {
              float intensity = 1.0 - ((elapsed - 300) / 300.0);
              uint8_t brightness = 255 * intensity;
              uint8_t r = brightness * 0.8;
              uint8_t g = 0;
              uint8_t b = brightness;
              M5.dis.drawpix(0, (g << 16) | (r << 8) | b);
            } else if (elapsed < 900) {
              float intensity = 1.0 - ((elapsed - 600) / 300.0);
              uint8_t brightness = 255 * intensity;
              uint8_t r = brightness;
              uint8_t g = brightness * 0.3;
              uint8_t b = 0;
              M5.dis.drawpix(0, (g << 16) | (r << 8) | b);
            } else {
              currentEffect = RGB_SCANNING_RAINBOW;
            }
          }
          break;
          
        case RGB_REDETECT_FLASH:
          {
            unsigned long elapsed = currentMillis - effectStartTime;
            
            if (elapsed < 100) {
              M5.dis.drawpix(0, 0xFFFF00);
            } else if (elapsed < 200) {
              M5.dis.clear();
            } else if (elapsed < 300) {
              M5.dis.drawpix(0, 0xFFFF00);
            } else if (elapsed < 400) {
              M5.dis.clear();
            } else {
              currentEffect = RGB_SCANNING_RAINBOW;
            }
          }
          break;
          
        case RGB_READY_FADE:
          {
            unsigned long elapsed = currentMillis - effectStartTime;
            
            if (elapsed < 2000) {
              float progress = elapsed / 2000.0;
              float hue = 0.75 + (progress * 0.25);
              if (hue > 1.0) hue -= 1.0;
              M5.dis.drawpix(0, hsvToRgb(hue, 1.0, 1.0));
            } else {
              currentEffect = RGB_SCANNING_RAINBOW;
            }
          }
          break;
          
        case RGB_ERROR_BLINK:
          {
            unsigned long elapsed = currentMillis - effectStartTime;
            
            if (elapsed < 3000) {
              bool on = (elapsed / 250) % 2 == 0;
              if (on) {
                M5.dis.drawpix(0, 0x00FF00);
              } else {
                M5.dis.clear();
              }
            } else {
              currentEffect = RGB_OFF;
            }
          }
          break;
      }
    }
    
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// Scanning Task - runs on Core 0
void ScanTask(void* pvParameters) {
  static unsigned long lastScanTime = 0;
  static unsigned long lastBLECleanupTime = 0;
  static unsigned long lastDeviceCleanupTime = 0;
  static unsigned long lastStatusTime = 0;

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

      if (currentMillis - lastScanTime >= 1000) {
        if (pBLEScan) {
          pBLEScan->stop();
          vTaskDelay(10 / portTICK_PERIOD_MS);
          
          NimBLEScanResults results = pBLEScan->getResults();
          for (int i = 0; i < results.getCount(); i++) {
            const NimBLEAdvertisedDevice* advertisedDevice = results.getDevice(i);
            
            String mac = String(advertisedDevice->getAddress().toString().c_str());
            int rssi = advertisedDevice->getRSSI();
            
            String matchedDescription;
            bool matchFound = matchesTargetFilter(mac, matchedDescription);
            
            if (!matchFound) continue;
            
            bool known = false;
            for (auto& dev : devices) {
              if (dev.macAddress == mac) {
                known = true;
                
                if (dev.inCooldown && currentMillis < dev.cooldownUntil) break;
                if (dev.inCooldown && currentMillis >= dev.cooldownUntil) dev.inCooldown = false;
                
                unsigned long timeSinceLastSeen = currentMillis - dev.lastSeen;
                
                if (timeSinceLastSeen >= 30000) {
                  triggerDetectionBurst();
                  Serial.println("BLE RE-DETECTED after 30+ sec: " + matchedDescription);
                  dev.inCooldown = true;
                  dev.cooldownUntil = currentMillis + 10000;
                } else if (timeSinceLastSeen >= 5000) {
                  triggerRedetectFlash();
                  Serial.println("BLE RE-DETECTED after 5+ sec: " + matchedDescription);
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
              
              triggerDetectionBurst();
              Serial.println("NEW BLE DEVICE DETECTED: " + matchedDescription);
              Serial.println("MAC: " + mac + " | RSSI: " + String(rssi));
              
              auto& dev = devices.back();
              dev.inCooldown = true;
              dev.cooldownUntil = currentMillis + 5000;
            }
          }
          
          pBLEScan->start(0.8, false);
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

// Effect trigger functions
void triggerDetectionBurst() {
  effectStartTime = millis();
  currentEffect = RGB_DETECTION_BURST;
  playDetectionAlert();
}

void triggerRedetectFlash() {
  effectStartTime = millis();
  currentEffect = RGB_REDETECT_FLASH;
  playRedetectAlert();
}

void triggerReadyFade() {
  effectStartTime = millis();
  currentEffect = RGB_READY_FADE;
  playReadyChime();
}

void triggerErrorBlink() {
  effectStartTime = millis();
  currentEffect = RGB_ERROR_BLINK;
  playErrorTone();
}

void startConfigPulse() {
  currentEffect = RGB_CONFIG_PULSE;
}

void startScanningRainbow() {
  currentEffect = RGB_SCANNING_RAINBOW;
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

  int networksFound = WiFi.scanNetworks(false, true, false, 500, 0);

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

    if (!matchFound) continue;

    Serial.println("MATCHED FILTER: " + matchedDescription);

    bool known = false;
    for (auto& dev : devices) {
      if (dev.macAddress == bssid) {
        known = true;

        if (dev.inCooldown && currentMillis < dev.cooldownUntil) break;
        if (dev.inCooldown && currentMillis >= dev.cooldownUntil) dev.inCooldown = false;

        unsigned long dt = currentMillis - dev.lastSeen;

        if (dt >= 30000) {
          triggerDetectionBurst();
          Serial.println("WIFI RE-DETECTED after 30+ sec: " + matchedDescription);
          Serial.println("MAC: " + bssid + " | SSID: " + ssid + " | RSSI: " + String(rssi));
          dev.inCooldown = true;
          dev.cooldownUntil = currentMillis + 10000;
        } else if (dt >= 5000) {
          triggerRedetectFlash();
          Serial.println("WIFI RE-DETECTED after 5+ sec: " + matchedDescription);
          Serial.println("MAC: " + bssid + " | SSID: " + ssid + " | RSSI: " + String(rssi));
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

      triggerDetectionBurst();
      Serial.println("NEW WIFI DEVICE DETECTED: " + matchedDescription);
      Serial.println("MAC: " + bssid + " | SSID: " + ssid + " | RSSI: " + String(rssi));

      auto& dev = devices.back();
      dev.inCooldown = true;
      dev.cooldownUntil = currentMillis + 5000;
    }
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
  preferences.begin("ouispy", false);
  preferences.putInt("filterCount", targetFilters.size());
  preferences.putBool("audioEnabled", audioEnabled);

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
  audioEnabled = preferences.getBool("audioEnabled", true);

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

String generateConfigHTML() {
  String html = R"html(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Atom Echo OUI-SPY Configuration</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', sans-serif;
            margin: 0; padding: 20px; background: #0a0a0a; color: #ffffff;
        }
        .container {
            max-width: 700px; margin: 0 auto;
            background: rgba(255, 255, 255, 0.03);
            padding: 40px; border-radius: 16px;
            border: 1px solid rgba(138, 43, 226, 0.3);
        }
        h1 {
            text-align: center; font-size: 42px;
            background: linear-gradient(45deg, #8a2be2, #4169e1);
            -webkit-background-clip: text; -webkit-text-fill-color: transparent;
        }
        .section {
            margin-bottom: 30px; padding: 25px;
            border: 1px solid rgba(138, 43, 226, 0.3);
            border-radius: 12px;
            background: rgba(138, 43, 226, 0.05);
        }
        textarea {
            width: 100%; min-height: 120px; padding: 15px;
            border: 1px solid rgba(138, 43, 226, 0.4);
            border-radius: 8px;
            background: rgba(255, 255, 255, 0.05);
            color: #ffffff; font-family: monospace; font-size: 14px;
        }
        button {
            background: linear-gradient(135deg, #8a2be2 0%, #4169e1 100%);
            color: #ffffff; padding: 14px 28px; border: none;
            border-radius: 8px; cursor: pointer; font-size: 16px;
            margin: 10px 5px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Atom Echo OUI-SPY</h1>
        <form method="POST" action="/save">
            <div class="section">
                <h3>OUI Prefixes</h3>
                <textarea name="ouis"></textarea>
            </div>
            <div class="section">
                <h3>MAC Addresses</h3>
                <textarea name="macs"></textarea>
            </div>
            <button type="submit">Save & Start Scanning</button>
        </form>
    </div>
</body>
</html>
)html";
  return html;
}

void startConfigMode() {
  currentMode = CONFIG_MODE;
  
  Serial.println("\n=== STARTING CONFIG MODE ===");
  Serial.println("SSID: " + AP_SSID);
  
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  delay(1000);
  
  WiFi.mode(WIFI_AP);
  delay(500);
  
  bool apStarted = WiFi.softAP(AP_SSID.c_str(), AP_PASSWORD.c_str(), 6, 0, 4);
  
  if (!apStarted) {
    Serial.println("Failed to create Access Point!");
    triggerErrorBlink();
    return;
  }
  
  delay(2000);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.println("AP IP address: " + IP.toString());
  
  configStartTime = millis();
  lastConfigActivity = millis();
  
  startConfigPulse();
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    lastConfigActivity = millis();
    request->send(200, "text/html", generateConfigHTML());
  });
  
  server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) {
    lastConfigActivity = millis();
    
    targetFilters.clear();
    
    Serial.println("=== Form Data Received ===");
    
    // Process OUI prefixes
    if (request->hasParam("ouis", true)) {
      String ouiData = request->getParam("ouis", true)->value();
      ouiData.trim();
      
      Serial.println("OUI data: " + ouiData);
      
      if (ouiData.length() > 0) {
        int lineStart = 0;
        
        while (lineStart < ouiData.length()) {
          int lineEnd = ouiData.indexOf('\n', lineStart);
          if (lineEnd == -1) lineEnd = ouiData.length();
          
          String line = ouiData.substring(lineStart, lineEnd);
          line.replace("\r", "");
          line.trim();
          
          if (line.length() >= 8 && !line.startsWith("#")) {
            normalizeMACAddress(line);
            
            if (line.length() == 8 && line.charAt(2) == ':' && line.charAt(5) == ':') {
              TargetFilter filter;
              filter.identifier = line;
              filter.isFullMAC = false;
              filter.description = "OUI: " + line;
              targetFilters.push_back(filter);
              Serial.println("Added OUI: " + line);
            }
          }
          
          lineStart = lineEnd + 1;
        }
      }
    }
    
    // Process full MAC addresses
    if (request->hasParam("macs", true)) {
      String macData = request->getParam("macs", true)->value();
      macData.trim();
      
      Serial.println("MAC data: " + macData);
      
      if (macData.length() > 0) {
        int lineStart = 0;
        
        while (lineStart < macData.length()) {
          int lineEnd = macData.indexOf('\n', lineStart);
          if (lineEnd == -1) lineEnd = macData.length();
          
          String line = macData.substring(lineStart, lineEnd);
          line.replace("\r", "");
          line.trim();
          
          if (line.length() >= 17 && !line.startsWith("#")) {
            normalizeMACAddress(line);
            
            if (line.length() == 17) {
              bool validFormat = true;
              for (int i = 0; i < 17; i++) {
                if (i % 3 == 2) {
                  if (line.charAt(i) != ':') {
                    validFormat = false;
                    break;
                  }
                } else {
                  if (!isxdigit(line.charAt(i))) {
                    validFormat = false;
                    break;
                  }
                }
              }
              
              if (validFormat) {
                TargetFilter filter;
                filter.identifier = line;
                filter.isFullMAC = true;
                filter.description = "MAC: " + line;
                targetFilters.push_back(filter);
                Serial.println("Added MAC: " + line);
              }
            }
          }
          
          lineStart = lineEnd + 1;
        }
      }
    }
    
    Serial.println("Total filters: " + String(targetFilters.size()));
    
    if (targetFilters.size() > 0) {
      saveConfiguration();
      String response = "<html><body><h1>Success!</h1>";
      response += "<p>Saved " + String(targetFilters.size()) + " filter(s)</p>";
      response += "<p>Switching to scanning mode in 3 seconds...</p></body></html>";
      request->send(200, "text/html", response);
      modeSwitchScheduled = millis() + 3000;
    } else {
      String response = "<html><body><h1>Error: No valid filters</h1>";
      response += "<p>Enter MAC addresses in format:</p>";
      response += "<pre>OUI: aa:bb:cc\nFull MAC: aa:bb:cc:dd:ee:ff</pre>";
      response += "<p><a href='/'>Go Back</a></p></body></html>";
      request->send(200, "text/html", response);
    }
  });
  
  server.begin();
}

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
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
  }
  
  delay(500);
  triggerReadyFade();
  delay(2500);
  
  if (pBLEScan != nullptr) {
    pBLEScan->start(0, false);
    Serial.println("BLE scanning started!");
  }
  
  Serial.println("WiFi scanning ready!");
  startScanningRainbow();
}

void setup() {
  delay(2000);
  Serial.begin(115200);
  delay(1000);
  
  M5.begin(true, false, true);
  delay(50);
  
  initI2SSpeaker();
  
  M5.dis.drawpix(0, 0x008000FF);
  playTone(2000, 200);
  delay(300);
  M5.dis.clear();
  
  Serial.println("\n\n=== ATOM ECHO OUI-SPY ===");
  Serial.println("Mode: BLE/WiFi device detection with RGB + Audio");
  Serial.println("Initializing...\n");
  
  xTaskCreatePinnedToCore(
    AudioLEDTask,
    "AudioLEDTask",
    4096,
    NULL,
    3,
    &AudioLEDTaskHandle,
    1
  );
  
  Serial.println("Free heap at startup: " + String(ESP.getFreeHeap()));
  
  WiFi.mode(WIFI_OFF);
  
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
    devices.clear();
    
    Serial.println("Factory reset complete");
  } else {
    loadConfiguration();
    loadWiFiCredentials();
  }
  
  preferences.begin("ouispy", true);
  bool configLocked = preferences.getBool("configLocked", false);
  preferences.end();
  
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
      }
    }
    
    delay(100);
    return;
  }
  
  delay(100);
}