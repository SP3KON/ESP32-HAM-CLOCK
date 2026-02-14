/*
 * ESP32 HAM CLOCK - Main Application
 * 
 * Clock and DX Cluster terminal for ESP32-2432S028 (CYD) with 2.4" TFT
 * Features: DX Cluster, POTA, APRS-IS, Weather, Propagation, Web UI
 * 
 * Hardware: ESP32-2432S028 (ESP32 WROOM + ILI9341 TFT + XPT2046 Touch)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

#ifdef ENABLE_TFT_DISPLAY
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#endif

// Configuration
#define AP_SSID "ESP32-HAM-CLOCK"
#define AP_PASSWORD "1234567890"
#define AP_IP IPAddress(192, 168, 4, 1)

// Pin definitions for ESP32-2432S028
#define TFT_BL_PIN 21
#define TOUCH_IRQ_PIN 36
#define TOUCH_CS_PIN 33

// Global objects
Preferences preferences;
AsyncWebServer server(80);

#ifdef ENABLE_TFT_DISPLAY
TFT_eSPI tft = TFT_eSPI();
XPT2046_Touchscreen touch(TOUCH_CS_PIN, TOUCH_IRQ_PIN);
#endif

// Configuration variables
String wifi_ssid = "";
String wifi_password = "";
String wifi_ssid2 = "";
String wifi_password2 = "";
String dx_cluster_host = "";
int dx_cluster_port = 7300;
String callsign = "";
String locator = "";
String openweather_key = "";
String qrz_key = "";
int tft_brightness = 128;
String language = "EN";
int tft_rotation = 1;

// Screen management
enum Screen {
    SCREEN_CLOCK,
    SCREEN_DX,
    SCREEN_APRS,
    SCREEN_BANDS,
    SCREEN_PROPAGATION,
    SCREEN_WEATHER,
    SCREEN_POTA,
    SCREEN_MATRIX
};

Screen currentScreen = SCREEN_CLOCK;

// Network clients
WiFiClient dxClusterClient;
WiFiClient aprsClient;
WiFiClientSecure secureClient;

// Timing variables
unsigned long lastWeatherUpdate = 0;
unsigned long lastPropagationUpdate = 0;
unsigned long lastPotaUpdate = 0;
unsigned long lastDxClusterKeepalive = 0;
unsigned long lastQrzLookup = 0;

// Constants
const unsigned long WEATHER_UPDATE_INTERVAL = 10 * 60 * 1000;  // 10 minutes
const unsigned long PROPAGATION_UPDATE_INTERVAL = 60 * 60 * 1000;  // 60 minutes
const unsigned long POTA_UPDATE_INTERVAL = 180 * 1000;  // 180 seconds
const unsigned long DX_KEEPALIVE_INTERVAL = 30 * 1000;  // 30 seconds
const unsigned long QRZ_LOOKUP_INTERVAL = 3 * 1000;  // 3 seconds (when DX/POTA active)

// Function declarations
void setupWiFi();
void setupAP();
void loadPreferences();
void savePreferences();
void setupWebServer();
void setupTFT();
void setupTouch();
void setupLittleFS();
void connectDXCluster();
void connectAPRS();
void updateWeather();
void updatePropagation();
void updatePota();
void handleTouch();
void drawScreen();
void setBacklight(int brightness);

void setup() {
    Serial.begin(115200);
    Serial.println("\n\nESP32 HAM CLOCK Starting...");
    
    // Initialize preferences (NVS)
    preferences.begin("hamclock", false);
    loadPreferences();
    
    // Initialize LittleFS
    setupLittleFS();
    
    // Initialize TFT display
    #ifdef ENABLE_TFT_DISPLAY
    setupTFT();
    setupTouch();
    setBacklight(tft_brightness);
    #endif
    
    // Setup WiFi
    setupWiFi();
    
    // Setup web server
    setupWebServer();
    
    Serial.println("Setup complete!");
}

void loop() {
    static unsigned long lastUpdate = 0;
    unsigned long currentMillis = millis();
    
    // Handle touch input
    #ifdef ENABLE_TFT_DISPLAY
    handleTouch();
    #endif
    
    // Update weather data
    if (currentMillis - lastWeatherUpdate >= WEATHER_UPDATE_INTERVAL) {
        updateWeather();
        lastWeatherUpdate = currentMillis;
    }
    
    // Update propagation data
    if (currentMillis - lastPropagationUpdate >= PROPAGATION_UPDATE_INTERVAL) {
        updatePropagation();
        lastPropagationUpdate = currentMillis;
    }
    
    // Update POTA data
    if (currentMillis - lastPotaUpdate >= POTA_UPDATE_INTERVAL) {
        updatePota();
        lastPotaUpdate = currentMillis;
    }
    
    // DX Cluster keepalive
    if (dxClusterClient.connected()) {
        if (currentMillis - lastDxClusterKeepalive >= DX_KEEPALIVE_INTERVAL) {
            dxClusterClient.print("\r\n");
            lastDxClusterKeepalive = currentMillis;
        }
        // Process incoming DX spots
        while (dxClusterClient.available()) {
            String line = dxClusterClient.readStringUntil('\n');
            Serial.println("DX: " + line);
            // TODO: Parse and display DX spots
        }
    } else {
        // Reconnect to DX Cluster if needed
        if (dx_cluster_host.length() > 0 && currentMillis - lastDxClusterKeepalive >= 20000) {
            connectDXCluster();
            lastDxClusterKeepalive = currentMillis;
        }
    }
    
    // APRS-IS connection
    if (aprsClient.connected()) {
        while (aprsClient.available()) {
            String line = aprsClient.readStringUntil('\n');
            Serial.println("APRS: " + line);
            // TODO: Parse and display APRS frames
        }
    }
    
    // Update display periodically
    if (currentMillis - lastUpdate >= 1000) {
        #ifdef ENABLE_TFT_DISPLAY
        drawScreen();
        #endif
        lastUpdate = currentMillis;
    }
    
    // Small delay to prevent watchdog issues
    delay(10);
}

void setupLittleFS() {
    Serial.println("Mounting LittleFS...");
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS mount failed!");
        return;
    }
    Serial.println("LittleFS mounted successfully");
}

void loadPreferences() {
    wifi_ssid = preferences.getString("wifi_ssid", "");
    wifi_password = preferences.getString("wifi_pass", "");
    wifi_ssid2 = preferences.getString("wifi_ssid2", "");
    wifi_password2 = preferences.getString("wifi_pass2", "");
    dx_cluster_host = preferences.getString("dx_host", "");
    dx_cluster_port = preferences.getInt("dx_port", 7300);
    callsign = preferences.getString("callsign", "");
    locator = preferences.getString("locator", "");
    openweather_key = preferences.getString("ow_key", "");
    qrz_key = preferences.getString("qrz_key", "");
    tft_brightness = preferences.getInt("brightness", 128);
    language = preferences.getString("language", "EN");
    tft_rotation = preferences.getInt("rotation", 1);
    
    Serial.println("Preferences loaded");
}

void savePreferences() {
    preferences.putString("wifi_ssid", wifi_ssid);
    preferences.putString("wifi_pass", wifi_password);
    preferences.putString("wifi_ssid2", wifi_ssid2);
    preferences.putString("wifi_pass2", wifi_password2);
    preferences.putString("dx_host", dx_cluster_host);
    preferences.putInt("dx_port", dx_cluster_port);
    preferences.putString("callsign", callsign);
    preferences.putString("locator", locator);
    preferences.putString("ow_key", openweather_key);
    preferences.putString("qrz_key", qrz_key);
    preferences.putInt("brightness", tft_brightness);
    preferences.putString("language", language);
    preferences.putInt("rotation", tft_rotation);
    
    Serial.println("Preferences saved");
}

void setupWiFi() {
    // Try to connect to WiFi
    if (wifi_ssid.length() > 0) {
        Serial.println("Connecting to WiFi: " + wifi_ssid);
        WiFi.mode(WIFI_STA);
        WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());
        
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            Serial.print(".");
            attempts++;
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nConnected to WiFi!");
            Serial.print("IP address: ");
            Serial.println(WiFi.localIP());
            return;
        }
        
        // Try second WiFi if first fails
        if (wifi_ssid2.length() > 0) {
            Serial.println("\nTrying second WiFi: " + wifi_ssid2);
            WiFi.begin(wifi_ssid2.c_str(), wifi_password2.c_str());
            
            attempts = 0;
            while (WiFi.status() != WL_CONNECTED && attempts < 20) {
                delay(500);
                Serial.print(".");
                attempts++;
            }
            
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("\nConnected to second WiFi!");
                Serial.print("IP address: ");
                Serial.println(WiFi.localIP());
                return;
            }
        }
    }
    
    // If no WiFi configured or connection failed, start AP
    Serial.println("\nStarting Access Point...");
    setupAP();
}

void setupAP() {
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    
    Serial.print("AP started. IP: ");
    Serial.println(WiFi.softAPIP());
}

void setupWebServer() {
    // Serve static files from LittleFS
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
    
    // Config endpoint - GET current config
    server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        doc["wifi_ssid"] = wifi_ssid;
        doc["wifi_ssid2"] = wifi_ssid2;
        doc["dx_host"] = dx_cluster_host;
        doc["dx_port"] = dx_cluster_port;
        doc["callsign"] = callsign;
        doc["locator"] = locator;
        doc["brightness"] = tft_brightness;
        doc["language"] = language;
        doc["rotation"] = tft_rotation;
        
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });
    
    // Config endpoint - POST to update config
    server.on("/api/config", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, data, len);
            
            if (error) {
                request->send(400, "text/plain", "Invalid JSON");
                return;
            }
            
            // Update configuration
            if (doc.containsKey("wifi_ssid")) wifi_ssid = doc["wifi_ssid"].as<String>();
            if (doc.containsKey("wifi_password")) wifi_password = doc["wifi_password"].as<String>();
            if (doc.containsKey("wifi_ssid2")) wifi_ssid2 = doc["wifi_ssid2"].as<String>();
            if (doc.containsKey("wifi_password2")) wifi_password2 = doc["wifi_password2"].as<String>();
            if (doc.containsKey("dx_host")) dx_cluster_host = doc["dx_host"].as<String>();
            if (doc.containsKey("dx_port")) dx_cluster_port = doc["dx_port"].as<int>();
            if (doc.containsKey("callsign")) callsign = doc["callsign"].as<String>();
            if (doc.containsKey("locator")) locator = doc["locator"].as<String>();
            if (doc.containsKey("ow_key")) openweather_key = doc["ow_key"].as<String>();
            if (doc.containsKey("qrz_key")) qrz_key = doc["qrz_key"].as<String>();
            if (doc.containsKey("brightness")) {
                tft_brightness = doc["brightness"].as<int>();
                setBacklight(tft_brightness);
            }
            if (doc.containsKey("language")) language = doc["language"].as<String>();
            if (doc.containsKey("rotation")) tft_rotation = doc["rotation"].as<int>();
            
            savePreferences();
            request->send(200, "text/plain", "Configuration saved. Restarting...");
            
            delay(1000);
            ESP.restart();
        }
    );
    
    // Status endpoint
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        doc["uptime"] = millis() / 1000;
        doc["wifi_connected"] = WiFi.status() == WL_CONNECTED;
        doc["wifi_rssi"] = WiFi.RSSI();
        doc["ip"] = WiFi.localIP().toString();
        doc["dx_connected"] = dxClusterClient.connected();
        doc["aprs_connected"] = aprsClient.connected();
        doc["screen"] = currentScreen;
        
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });
    
    server.begin();
    Serial.println("Web server started");
}

#ifdef ENABLE_TFT_DISPLAY
void setupTFT() {
    tft.init();
    tft.setRotation(tft_rotation);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.println("ESP32 HAM CLOCK");
    tft.println("Initializing...");
    Serial.println("TFT initialized");
}

void setupTouch() {
    touch.begin();
    touch.setRotation(tft_rotation);
    Serial.println("Touch initialized");
}

void setBacklight(int brightness) {
    // Setup PWM for backlight
    ledcSetup(0, 5000, 8);  // Channel 0, 5kHz, 8-bit resolution
    ledcAttachPin(TFT_BL_PIN, 0);
    ledcWrite(0, brightness);
}

void handleTouch() {
    if (touch.tirqTouched() && touch.touched()) {
        TS_Point p = touch.getPoint();
        
        // Map touch coordinates to screen coordinates
        int x = map(p.x, 200, 3700, 0, tft.width());
        int y = map(p.y, 240, 3800, 0, tft.height());
        
        Serial.printf("Touch: x=%d, y=%d\n", x, y);
        
        // Bottom corners for screen navigation
        if (y > tft.height() - 40) {
            if (x < 40) {
                // Previous screen
                currentScreen = (Screen)((currentScreen - 1 + 8) % 8);
            } else if (x > tft.width() - 40) {
                // Next screen
                currentScreen = (Screen)((currentScreen + 1) % 8);
            }
        }
        
        // Wait for touch release
        while (touch.touched()) {
            delay(10);
        }
        delay(100);
    }
}

void drawScreen() {
    switch (currentScreen) {
        case SCREEN_CLOCK:
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_WHITE);
            tft.setCursor(10, 10);
            tft.setTextSize(3);
            tft.println("CLOCK");
            tft.setTextSize(2);
            tft.printf("UTC: %02d:%02d:%02d\n", 12, 0, 0);  // TODO: Get real time
            tft.printf("IP: %s\n", WiFi.localIP().toString().c_str());
            break;
            
        case SCREEN_DX:
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_GREEN);
            tft.setCursor(10, 10);
            tft.setTextSize(2);
            tft.println("DX CLUSTER");
            tft.println("Connecting...");
            break;
            
        case SCREEN_APRS:
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_CYAN);
            tft.setCursor(10, 10);
            tft.setTextSize(2);
            tft.println("APRS-IS");
            break;
            
        case SCREEN_BANDS:
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_YELLOW);
            tft.setCursor(10, 10);
            tft.setTextSize(2);
            tft.println("HF BANDS");
            break;
            
        case SCREEN_PROPAGATION:
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_ORANGE);
            tft.setCursor(10, 10);
            tft.setTextSize(2);
            tft.println("PROPAGATION");
            break;
            
        case SCREEN_WEATHER:
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_BLUE);
            tft.setCursor(10, 10);
            tft.setTextSize(2);
            tft.println("WEATHER");
            break;
            
        case SCREEN_POTA:
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_MAGENTA);
            tft.setCursor(10, 10);
            tft.setTextSize(2);
            tft.println("POTA");
            break;
            
        case SCREEN_MATRIX:
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_GREEN);
            tft.setCursor(10, 10);
            tft.setTextSize(2);
            tft.println("MATRIX CLOCK");
            break;
    }
}
#else
void setupTFT() {}
void setupTouch() {}
void setBacklight(int brightness) {}
void handleTouch() {}
void drawScreen() {}
#endif

void connectDXCluster() {
    if (dx_cluster_host.length() == 0 || callsign.length() == 0) {
        return;
    }
    
    Serial.println("Connecting to DX Cluster: " + dx_cluster_host);
    
    if (dxClusterClient.connect(dx_cluster_host.c_str(), dx_cluster_port)) {
        Serial.println("Connected to DX Cluster");
        delay(1000);
        dxClusterClient.println(callsign);
        delay(500);
    } else {
        Serial.println("DX Cluster connection failed");
    }
}

void connectAPRS() {
    // TODO: Implement APRS-IS connection
    Serial.println("APRS-IS connection not yet implemented");
}

void updateWeather() {
    if (openweather_key.length() == 0 || locator.length() == 0) {
        return;
    }
    
    Serial.println("Updating weather data...");
    // TODO: Implement OpenWeather API calls
    // - Current weather
    // - Forecast
    // - Air quality (PM2.5/PM10)
}

void updatePropagation() {
    Serial.println("Updating propagation data...");
    // TODO: Implement hamqsl solarxml API call
}

void updatePota() {
    Serial.println("Updating POTA spots...");
    // TODO: Implement POTA API call
}
