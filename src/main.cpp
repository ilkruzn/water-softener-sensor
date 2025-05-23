#define MQTT_MAX_PACKET_SIZE 1024  // Add this at the top before includes
#include <M5Unified.h>
#include <WiFi.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <esp_sleep.h>
#include <time.h>

// ===== Logger =====
class Logger {
private:
    static Logger* instance;
    bool loggingEnabled = true;
    unsigned long startTime;
    
    Logger() {
        startTime = millis();
    }
    
public:
    enum LogLevel {
        DEBUG,
        INFO,
        WARNING,
        ERROR
    };
    
    static Logger* getInstance() {
        if (instance == nullptr) {
            instance = new Logger();
        }
        return instance;
    }
    
    void init() {
        Serial.begin(115200);
        delay(100); // Allow serial to initialize
        log(INFO, "Logger initialized");
    }
    
    void enableLogging(bool enabled) {
        loggingEnabled = enabled;
    }
    
    void log(LogLevel level, const String& message) {
        if (!loggingEnabled) return;
        
        String levelStr;
        switch (level) {
            case DEBUG:   levelStr = "DEBUG"; break;
            case INFO:    levelStr = "INFO"; break;
            case WARNING: levelStr = "WARN"; break;
            case ERROR:   levelStr = "ERROR"; break;
        }
        
        unsigned long uptime = millis() - startTime;
        String timestamp = String(uptime / 1000) + "." + String(uptime % 1000);
        
        Serial.printf("[%s][%s] %s\n", timestamp.c_str(), levelStr.c_str(), message.c_str());
    }
    
    void debug(const String& message) {
        log(DEBUG, message);
    }
    
    void info(const String& message) {
        log(INFO, message);
    }
    
    void warning(const String& message) {
        log(WARNING, message);
    }
    
    void error(const String& message) {
        log(ERROR, message);
    }
};

Logger* Logger::instance = nullptr;

// ===== Configuration =====
// These values are in seconds for readability but will be converted to microseconds
// for the actual deep sleep function
class Config {
private:
    static Config* instance;
    
    Config() {}
    
public:
    // Settings - easily changeable
    const char* WIFI_SSID = "";
    const char* WIFI_PASSWORD = "";
    const char* MQTT_SERVER = "192.168.1.251";
    const int MQTT_PORT = 1883;
    const char* MQTT_USER = ""; // if needed
    const char* MQTT_PASSWORD = ""; // if needed
    
    // Single measurement interval for both distance and battery
    // const uint32_t MEASUREMENT_INTERVAL = 10; // 10 seconds for testing
    const uint32_t MEASUREMENT_INTERVAL = 6 * 60 * 60; // 6 hours in seconds
    const uint32_t DISPLAY_TIMEOUT = 5; // 5 seconds
    
    // Device information
    const char* DEVICE_NAME = "m5stickc_sensor_block_salt";
    const char* DEVICE_MODEL = "M5StickC Plus2";
    
    static Config* getInstance() {
        if (instance == nullptr) {
            instance = new Config();
        }
        return instance;
    }
};

Config* Config::instance = nullptr;

// ===== Sensor Manager =====
class SensorManager {
private:
    static SensorManager* instance;
    VL53L0X distanceSensor;
    bool sensorInitialized = false;
    Logger* logger;
    
    SensorManager() {
        logger = Logger::getInstance();
    }
    
public:
    static SensorManager* getInstance() {
        if (instance == nullptr) {
            instance = new SensorManager();
        }
        return instance;
    }
    
    bool init() {
        logger->info("Initializing VL53L0X sensor on pins SDA=0, SCL=26");
        Wire.begin(0, 26); // SDA, SCL pins on M5StickC Plus2
        sensorInitialized = distanceSensor.init();
        if (sensorInitialized) {
            logger->info("VL53L0X sensor initialized successfully");
            // Configure for better accuracy at the cost of speed
            distanceSensor.setMeasurementTimingBudget(200000);
        } else {
            logger->error("Failed to initialize VL53L0X sensor!");
        }
        return sensorInitialized;
    }
    
    uint16_t readDistance() {
        logger->debug("Reading distance...");
        if (!sensorInitialized) {
            logger->debug("Sensor not initialized, initializing now");
            if (!init()) {
                logger->error("Could not initialize sensor for distance reading");
                return 0;
            }
        }
        
        uint16_t distance = distanceSensor.readRangeSingleMillimeters();
        if (distanceSensor.timeoutOccurred()) {
            logger->warning("VL53L0X timeout occurred during reading");
            return 0;
        }
        logger->info("Distance reading: " + String(distance) + " mm");
        return distance;
    }
    
    float readBatteryLevel() {
        logger->debug("Reading battery level...");
        float level = M5.Power.getBatteryLevel();
        logger->info("Battery level: " + String(level) + "%");
        return level;
    }
    
    void powerDownSensor() {
        if (sensorInitialized) {
            distanceSensor.setTimeout(0); // Disable timeouts
            digitalWrite(21, LOW); // XSHUT pin to power down - adjust GPIO if needed
            sensorInitialized = false;
            logger->debug("VL53L0X sensor powered down");
        }
    }
};

SensorManager* SensorManager::instance = nullptr;

// ===== Communication Manager =====
class CommunicationManager {
private:
    static CommunicationManager* instance;
    WiFiClient wifiClient;
    PubSubClient mqttClient;
    bool wifiConnected = false;
    bool mqttConnected = false;
    Logger* logger;
    
    CommunicationManager() : mqttClient(wifiClient) {
        logger = Logger::getInstance();
    }
    
public:
    static CommunicationManager* getInstance() {
        if (instance == nullptr) {
            instance = new CommunicationManager();
        }
        return instance;
    }
    
    bool connectWiFi() {
        if (wifiConnected) {
            logger->debug("WiFi already connected");
            return true;
        }
        
        Config* config = Config::getInstance();
        
        logger->info("Connecting to WiFi: " + String(config->WIFI_SSID));
        WiFi.begin(config->WIFI_SSID, config->WIFI_PASSWORD);
        WiFi.setTxPower(WIFI_POWER_19_5dBm); // Set max TX power
        
        // Try to connect with timeout
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            attempts++;
            if (attempts % 5 == 0) {
                logger->debug("Still trying to connect to WiFi... (" + String(attempts) + "/20)");
            }
        }
        
        wifiConnected = (WiFi.status() == WL_CONNECTED);
        
        if (wifiConnected) {
            logger->info("WiFi connected. IP: " + WiFi.localIP().toString());
        } else {
            logger->error("Failed to connect to WiFi");
        }
        
        return wifiConnected;
    }
    
    void disconnectWiFi() {
        if (wifiConnected) {
            logger->info("Disconnecting from WiFi");
            WiFi.disconnect();
            wifiConnected = false;
        }
    }
    
    bool connectMQTT() {
        if (!wifiConnected && !connectWiFi()) {
            logger->error("Cannot connect to MQTT: WiFi not connected");
            return false;
        }
        
        if (mqttConnected) {
            if (mqttClient.connected()) {
                logger->debug("MQTT already connected. Servicing connection.");
                mqttClient.loop(); // Service the existing connection
                return true;
            } else {
                logger->warning("MQTT was marked as connected, but actual connection is lost. Attempting to reconnect.");
                mqttConnected = false; // Force reconnection
            }
        }
        
        Config* config = Config::getInstance();
        logger->info("Connecting to MQTT server: " + String(config->MQTT_SERVER));
        // Ensure PubSubClient's buffer is large enough. If MQTT_MAX_PACKET_SIZE define isn't picked up,
        // you might need to call mqttClient.setBufferSize(MQTT_MAX_PACKET_SIZE); here or in constructor.
        // However, defining MQTT_MAX_PACKET_SIZE before including PubSubClient.h usually works.
        mqttClient.setServer(config->MQTT_SERVER, config->MQTT_PORT);
        
        String clientId = "M5StickC-";
        clientId += String(random(0xffff), HEX);
        logger->debug("MQTT client ID: " + clientId);
        
        if (mqttClient.connect(clientId.c_str(), config->MQTT_USER, config->MQTT_PASSWORD)) {
            mqttConnected = true;
            logger->info("Connected to MQTT server");
            mqttClient.loop(); // Call loop immediately after successful connect
            delay(500); // The delay might be less critical now or can be reduced
            logger->debug("Sending MQTT discovery messages");
            sendDiscoveryMessages();
        } else {
            logger->error("Failed to connect to MQTT server, error: " + String(mqttClient.state()));
            mqttConnected = false; // Explicitly set to false on failure
        }
        
        return mqttConnected;
    }
    
    void disconnectMQTT() {
        if (mqttConnected) {
            logger->info("Disconnecting from MQTT server");
            mqttClient.disconnect();
            mqttConnected = false;
        }
    }
    
    void sendDiscoveryMessages() {
        Config* config = Config::getInstance();
        
        String distanceId = String(config->DEVICE_NAME) + "_distance";
        String batteryId = String(config->DEVICE_NAME) + "_battery";
        
        logger->debug("Creating discovery message for distance sensor");
        // Distance sensor discovery
        DynamicJsonDocument distanceDoc(1024);
        distanceDoc["name"] = "Distance Sensor";
        distanceDoc["device_class"] = "distance";
        distanceDoc["state_topic"] = "homeassistant/sensor/" + distanceId + "/state";
        distanceDoc["unit_of_measurement"] = "mm";
        distanceDoc["value_template"] = "{{ value_json.distance }}";
        distanceDoc["unique_id"] = distanceId;
        distanceDoc["force_update"] = true; // Ensures HA updates last_changed
        
        JsonObject device = distanceDoc.createNestedObject("device");
        device["identifiers"] = config->DEVICE_NAME;
        device["name"] = config->DEVICE_NAME;
        device["model"] = config->DEVICE_MODEL;
        device["manufacturer"] = "M5Stack";
        
        String distanceOutput;
        serializeJson(distanceDoc, distanceOutput);
        String distanceTopic = "homeassistant/sensor/" + distanceId + "/config";
        
        const int MAX_PUBLISH_ATTEMPTS = 3;
        const int PUBLISH_RETRY_DELAY_MS = 1000;
        int attempts = 0;
        bool publishedDistance = false;

        logger->debug("Attempting to publish distance sensor discovery to: " + distanceTopic);
        while (attempts < MAX_PUBLISH_ATTEMPTS && !publishedDistance) {
            if (!mqttClient.connected()) {
                logger->warning("MQTT disconnected before publishing discovery. Attempting to reconnect...");
                if (!connectMQTT()) { // connectMQTT will call loop() if successful
                    logger->error("Failed to reconnect MQTT for discovery. Aborting.");
                    break; 
                }
            }
            if (mqttClient.connected()) {
                publishedDistance = mqttClient.publish(distanceTopic.c_str(), distanceOutput.c_str(), true);
                if (publishedDistance) {
                    logger->debug("Distance sensor discovery message published successfully.");
                } else {
                    attempts++;
                    logger->warning("Failed to publish distance sensor discovery. Attempt " + String(attempts) + "/" + String(MAX_PUBLISH_ATTEMPTS) + ". MQTT State: " + String(mqttClient.state()));
                    if (attempts < MAX_PUBLISH_ATTEMPTS) {
                        delay(PUBLISH_RETRY_DELAY_MS);
                    }
                }
            } else { // Should not happen if connectMQTT above worked, but as a safeguard
                attempts++;
                 logger->error("Still not connected after reconnect attempt for discovery. Attempt " + String(attempts) + "/" + String(MAX_PUBLISH_ATTEMPTS));
                if (attempts < MAX_PUBLISH_ATTEMPTS) {
                    delay(PUBLISH_RETRY_DELAY_MS);
                } else {
                    break; 
                }
            }
            if (mqttClient.connected()) { // Call loop if still connected
                mqttClient.loop();
            }
        }
        if (!publishedDistance) {
            logger->error("Failed to publish distance sensor discovery after " + String(MAX_PUBLISH_ATTEMPTS) + " attempts.");
        }
                         
        logger->debug("Creating discovery message for battery sensor");
        // Battery sensor discovery
        DynamicJsonDocument batteryDoc(1024);
        batteryDoc["name"] = "Battery Level";
        batteryDoc["device_class"] = "battery";
        batteryDoc["state_topic"] = "homeassistant/sensor/" + batteryId + "/state";
        batteryDoc["unit_of_measurement"] = "%";
        batteryDoc["value_template"] = "{{ value_json.battery }}";
        batteryDoc["unique_id"] = batteryId;
        batteryDoc["force_update"] = true; // Ensures HA updates last_changed
        batteryDoc["device"] = device; // Reuse device info
        
        String batteryOutput;
        serializeJson(batteryDoc, batteryOutput);
        String batteryTopic = "homeassistant/sensor/" + batteryId + "/config";
        
        attempts = 0;
        bool publishedBattery = false;
        logger->debug("Attempting to publish battery sensor discovery to: " + batteryTopic);
        while (attempts < MAX_PUBLISH_ATTEMPTS && !publishedBattery) {
             if (!mqttClient.connected()) {
                logger->warning("MQTT disconnected before publishing discovery. Attempting to reconnect...");
                if (!connectMQTT()) {
                    logger->error("Failed to reconnect MQTT for discovery. Aborting.");
                    break;
                }
            }
            if (mqttClient.connected()) {
                publishedBattery = mqttClient.publish(batteryTopic.c_str(), batteryOutput.c_str(), true);
                if (publishedBattery) {
                    logger->debug("Battery sensor discovery message published successfully.");
                } else {
                    attempts++;
                    logger->warning("Failed to publish battery sensor discovery. Attempt " + String(attempts) + "/" + String(MAX_PUBLISH_ATTEMPTS) + ". MQTT State: " + String(mqttClient.state()));
                    if (attempts < MAX_PUBLISH_ATTEMPTS) {
                        delay(PUBLISH_RETRY_DELAY_MS);
                    }
                }
            } else {
                 attempts++;
                 logger->error("Still not connected after reconnect attempt for discovery. Attempt " + String(attempts) + "/" + String(MAX_PUBLISH_ATTEMPTS));
                if (attempts < MAX_PUBLISH_ATTEMPTS) {
                    delay(PUBLISH_RETRY_DELAY_MS);
                } else {
                    break;
                }
            }
            if (mqttClient.connected()) { // Call loop if still connected
                mqttClient.loop();
            }
        }
        if (!publishedBattery) {
            logger->error("Failed to publish battery sensor discovery after " + String(MAX_PUBLISH_ATTEMPTS) + " attempts.");
        }
    }
    
    void publishSensorData(uint16_t distance, float battery) {
        if (!mqttConnected && !connectMQTT()) { // connectMQTT now handles loop and better state checking
            logger->error("Cannot publish data: MQTT not connected after connection attempts.");
            return;
        }
        
        Config* config = Config::getInstance();
        const int MAX_PUBLISH_ATTEMPTS = 3;
        const int PUBLISH_RETRY_DELAY_MS = 1000;
        
        // Create unique IDs
        String distanceId = String(config->DEVICE_NAME) + "_distance";
        String batteryId = String(config->DEVICE_NAME) + "_battery";
        
        // Send distance data
        logger->info("Publishing distance data: " + String(distance) + " mm");
        DynamicJsonDocument distanceDoc(256);
        distanceDoc["distance"] = distance;
        String distanceOutput;
        serializeJson(distanceDoc, distanceOutput);
        String distanceStateTopic = "homeassistant/sensor/" + distanceId + "/state";
        
        int attempts = 0;
        bool publishedDistance = false;
        logger->debug("Attempting to publish distance data to: " + distanceStateTopic);
        while (attempts < MAX_PUBLISH_ATTEMPTS && !publishedDistance) {
            if (!mqttClient.connected()) {
                logger->warning("MQTT disconnected before publishing data. Attempting to reconnect...");
                if (!connectMQTT()) {
                    logger->error("Failed to reconnect MQTT for data publish. Aborting.");
                    break;
                }
            }
            if (mqttClient.connected()) {
                publishedDistance = mqttClient.publish(distanceStateTopic.c_str(), distanceOutput.c_str());
                if (publishedDistance) {
                    logger->debug("Distance data published successfully.");
                } else {
                    attempts++;
                    logger->warning("Failed to publish distance data. Attempt " + String(attempts) + "/" + String(MAX_PUBLISH_ATTEMPTS) + ". MQTT State: " + String(mqttClient.state()));
                    if (attempts < MAX_PUBLISH_ATTEMPTS) {
                        delay(PUBLISH_RETRY_DELAY_MS);
                    }
                }
            } else {
                attempts++;
                logger->error("Still not connected after reconnect attempt for data publish. Attempt " + String(attempts) + "/" + String(MAX_PUBLISH_ATTEMPTS));
                if (attempts < MAX_PUBLISH_ATTEMPTS) {
                    delay(PUBLISH_RETRY_DELAY_MS);
                } else {
                    break;
                }
            }
            if (mqttClient.connected()) { // Call loop if still connected
                mqttClient.loop();
            }
        }
        if (!publishedDistance) {
            logger->error("Failed to publish distance data after " + String(MAX_PUBLISH_ATTEMPTS) + " attempts.");
        }
        
        // Send battery data
        logger->info("Publishing battery data: " + String(battery) + "%");
        DynamicJsonDocument batteryDoc(256);
        batteryDoc["battery"] = battery;
        String batteryOutput;
        serializeJson(batteryDoc, batteryOutput);
        String batteryStateTopic = "homeassistant/sensor/" + batteryId + "/state";
        
        attempts = 0;
        bool publishedBattery = false;
        logger->debug("Attempting to publish battery data to: " + batteryStateTopic);
        while (attempts < MAX_PUBLISH_ATTEMPTS && !publishedBattery) {
            if (!mqttClient.connected()) {
                logger->warning("MQTT disconnected before publishing data. Attempting to reconnect...");
                if (!connectMQTT()) {
                    logger->error("Failed to reconnect MQTT for data publish. Aborting.");
                    break;
                }
            }
            if (mqttClient.connected()) {
                publishedBattery = mqttClient.publish(batteryStateTopic.c_str(), batteryOutput.c_str());
                if (publishedBattery) {
                    logger->debug("Battery data published successfully.");
                } else {
                    attempts++;
                    logger->warning("Failed to publish battery data. Attempt " + String(attempts) + "/" + String(MAX_PUBLISH_ATTEMPTS) + ". MQTT State: " + String(mqttClient.state()));
                    if (attempts < MAX_PUBLISH_ATTEMPTS) {
                        delay(PUBLISH_RETRY_DELAY_MS);
                    }
                }
            } else {
                attempts++;
                logger->error("Still not connected after reconnect attempt for data publish. Attempt " + String(attempts) + "/" + String(MAX_PUBLISH_ATTEMPTS));
                if (attempts < MAX_PUBLISH_ATTEMPTS) {
                    delay(PUBLISH_RETRY_DELAY_MS);
                } else {
                    break;
                }
            }
            if (mqttClient.connected()) { // Call loop if still connected
                mqttClient.loop();
            }
        }
        if (!publishedBattery) {
            logger->error("Failed to publish battery data after " + String(MAX_PUBLISH_ATTEMPTS) + " attempts.");
        }
    }
};

CommunicationManager* CommunicationManager::instance = nullptr;


// ===== Display Manager =====
class DisplayManager {
private:
    static DisplayManager* instance;
    Logger* logger;
    
    DisplayManager() {
        logger = Logger::getInstance();
    }
    
public:
    static DisplayManager* getInstance() {
        if (instance == nullptr) {
            instance = new DisplayManager();
        }
        return instance;
    }
    
    void init() {
        logger->info("Initializing display");
        // Configure screen for power efficiency
        M5.Display.setBrightness(100);
        M5.Display.setRotation(3); // Landscape mode
    }
    
    void showBatteryLevel(float batteryLevel) {
        logger->info("Showing battery level on display: " + String(batteryLevel) + "%");
        
        // Switch to portrait mode
        M5.Display.setRotation(0);
        M5.Display.clear(BLACK);
        
        // Define battery dimensions and position
        const int batteryWidth = 70;
        const int batteryHeight = 120;
        const int batteryX = (M5.Display.width() - batteryWidth) / 2;
        const int batteryY = 30;
        const int batteryTipHeight = 10;
        const int batteryTipWidth = 30;
        const int cornerRadius = 8;
        const int borderThickness = 3;
        
        // Calculate fill height based on percentage
        int fillHeight = (batteryHeight - 2 * borderThickness) * batteryLevel / 100.0;
        
        // Determine battery color based on level
        uint16_t batteryColor;
        if (batteryLevel <= 20) {
            batteryColor = RED;
        } else if (batteryLevel <= 50) {
            batteryColor = YELLOW;
        } else {
            batteryColor = GREEN;
        }
        
        // Draw battery outline
        M5.Display.fillRoundRect(batteryX, batteryY, batteryWidth, batteryHeight, cornerRadius, WHITE);
        M5.Display.fillRoundRect(
            batteryX + borderThickness, 
            batteryY + borderThickness, 
            batteryWidth - 2 * borderThickness, 
            batteryHeight - 2 * borderThickness, 
            cornerRadius - 1, 
            BLACK
        );
        
        // Draw battery top tip
        M5.Display.fillRect(
            batteryX + (batteryWidth - batteryTipWidth) / 2, 
            batteryY - batteryTipHeight, 
            batteryTipWidth, 
            batteryTipHeight + borderThickness, 
            WHITE
        );
        
        // Draw the fill level with gradient effect
        for (int i = 0; i < fillHeight; i++) {
            // Create gradient effect from bottom to top (darker at bottom)
            float intensity = 0.7 + (0.3 * i / fillHeight);
            
            // Adjust color intensity
            uint8_t r = ((batteryColor >> 11) & 0x1F) * intensity;
            uint8_t g = ((batteryColor >> 5) & 0x3F) * intensity;
            uint8_t b = (batteryColor & 0x1F) * intensity;
            
            uint16_t adjustedColor = (r << 11) | (g << 5) | b;
            
            M5.Display.drawFastHLine(
                batteryX + borderThickness, 
                batteryY + batteryHeight - borderThickness - i, 
                batteryWidth - 2 * borderThickness, 
                adjustedColor
            );
        }
        
        // Add highlight reflection effect (diagonal line at top-right)
        for (int i = 0; i < 10; i++) {
            M5.Display.drawLine(
                batteryX + batteryWidth/2 + i*3, 
                batteryY + borderThickness + 5,
                batteryX + batteryWidth - borderThickness - 5, 
                batteryY + borderThickness + 5 + i*3,
                WHITE);
        }
        
        // Display battery percentage text
        M5.Display.setTextSize(3);
        M5.Display.setTextColor(batteryColor);
        
        // Center the text below the battery icon
        char percentText[10];
        sprintf(percentText, "%d%%", (int)batteryLevel);
        int textWidth = M5.Display.textWidth(percentText);
        M5.Display.setCursor((M5.Display.width() - textWidth) / 2, batteryY + batteryHeight + 20);
        M5.Display.print(percentText);
        
        logger->debug("Battery display rendered at " + String(batteryLevel) + "%");
    }
    
    void turnOff() {
        logger->info("Turning off display to save power");
        M5.Display.sleep();
        M5.Display.setBrightness(0);
    }
};

DisplayManager* DisplayManager::instance = nullptr;

// ===== Power Manager =====
class PowerManager {
private:
    static PowerManager* instance;
    
    // Single timestamp for last measurement
    Logger* logger;
    
    PowerManager() {
        logger = Logger::getInstance();
    }
    
public:
    static PowerManager* getInstance() {
        if (instance == nullptr) {
            instance = new PowerManager();
        }
        return instance;
    }
    
    void init() {
        // First boot initialization
        esp_sleep_wakeup_cause_t wakeupCause = esp_sleep_get_wakeup_cause();
        
        if (wakeupCause == ESP_SLEEP_WAKEUP_UNDEFINED) {
            logger->info("First boot detected, initializing RTC data");
        } else if (wakeupCause == ESP_SLEEP_WAKEUP_TIMER) {
            logger->info("Woke up from timer");
        } else {
            logger->info("Woke up from other cause: " + String(wakeupCause));
        }
    }
    
    void enterSleep() {
        // Configure RTC peripherals for deep sleep
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_ON);
        
        Config* config = Config::getInstance();
        
        uint32_t sleepTime = config->MEASUREMENT_INTERVAL;
        
        logger->info("Preparing to enter deep sleep");
        logger->debug("Next measurement in: " + String(sleepTime) + "s");
        
        // Disable WiFi and Bluetooth to save power
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        btStop();
        
        // Configure timer wake-up
        // esp_sleep_enable_timer_wakeup(sleepTime * 1000000ULL); // Convert seconds to microseconds
        // logger->debug("Timer wake-up enabled for " + String(sleepTime) + "s");
        
        // Configure button wake-up (EXT0)
        gpio_hold_en(GPIO_NUM_37);  // Enable GPIO hold for Button A
        gpio_pullup_en(GPIO_NUM_37);  // Enable pull-up for proper detection
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_37, 0); // Button A, wake on LOW
        logger->debug("Button A wake-up configured (GPIO 37)");
        
        // Force GPIO power during deep sleep
        gpio_deep_sleep_hold_en();
        logger->debug("GPIO power hold enabled during deep sleep");

        // Flush serial and add delay
        Serial.flush();
        delay(100);
        
        // Enter deep sleep
        logger->info("Entering deep sleep now...");
        delay(50); // Short delay to ensure log message is sent
        // esp_deep_sleep_start();
        M5.Power.timerSleep(sleepTime);
    }
};

PowerManager* PowerManager::instance = nullptr;

// ===== Application =====
class Application {
private:
    Config* config;
    SensorManager* sensorManager;
    CommunicationManager* commManager;
    PowerManager* powerManager;
    DisplayManager* displayManager;  // Add this
    Logger* logger;
    
public:
    Application() {
        logger = Logger::getInstance();
        config = Config::getInstance();
        sensorManager = SensorManager::getInstance();
        commManager = CommunicationManager::getInstance();
        powerManager = PowerManager::getInstance();
        displayManager = DisplayManager::getInstance();  // Add this
    }
    
    void setup() {
        // Initialize logger first
        logger->init();
        // logger->enableLogging(false); // Disable logging for saving power
        logger->info("===== M5StickC Plus2 Distance & Battery Monitor =====");
        
        // Log wake-up cause
        esp_sleep_wakeup_cause_t wakeupCause = esp_sleep_get_wakeup_cause();
        String wakeupCauseStr;
        switch (wakeupCause) {
            case ESP_SLEEP_WAKEUP_UNDEFINED: wakeupCauseStr = "Undefined/First boot"; break;
            case ESP_SLEEP_WAKEUP_EXT0: wakeupCauseStr = "External (EXT0/Button A)"; break;
            case ESP_SLEEP_WAKEUP_TIMER: wakeupCauseStr = "Timer"; break;
            default: wakeupCauseStr = "Unknown (" + String(wakeupCause) + ")"; break;
        }
        logger->info("Wake-up cause: " + wakeupCauseStr);
        
        // Initialize M5 hardware
        logger->debug("Initializing M5 hardware");
        auto cfg = M5.config();
        cfg.clear_display = true;
        M5.begin(cfg);
        logger->debug("M5 hardware initialized");
        
        // Lower CPU frequency to save power
        setCpuFrequencyMhz(80);
        logger->debug("CPU frequency set to 80MHz");
        
        // Initialize power manager
        powerManager->init();
        
        // Show battery level on wake up
        logger->info("Button pressed, showing battery level");
        displayManager->init();
        float batteryLevel = sensorManager->readBatteryLevel();
        displayManager->showBatteryLevel(batteryLevel);
        delay(5000); // Show battery level for 5 seconds
        displayManager->turnOff();
        
        // Configure NTP for accurate time
        logger->info("Configuring NTP...");
        configTime(0, 0, "pool.ntp.org");
        logger->info("Setup complete");
    }
    
    void loop() {
        logger->debug("Main loop started");
        
        // Read distance
        uint16_t distance = sensorManager->readDistance();
        
        // Read battery
        float battery = sensorManager->readBatteryLevel();
        
        // Update both measurements at once
        logger->info("New measurements available, connecting to send data");
        commManager->connectWiFi();
        commManager->connectMQTT();
        commManager->publishSensorData(distance, battery);
        commManager->disconnectMQTT();
        commManager->disconnectWiFi();
        
        // Power down the sensor to save power
        sensorManager->powerDownSensor();
        
        // Go to deep sleep to save power
        logger->debug("Preparing to enter deep sleep");
        powerManager->enterSleep();
    }
};

Application app;

void setup() {
    app.setup();
}

void loop() {
    app.loop();
}
