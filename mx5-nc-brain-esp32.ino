#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>

// --- Debug Configuration ---
#define DEBUG 1  // Set to 1 to enable debug output, 0 to disable

#if DEBUG
  #define DEBUG_BEGIN(baud) Serial.begin(baud)
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_BEGIN(baud)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(...)
#endif

// --- Configuration ---
namespace Config {
    // Hardware Pins for ESP32-C6
    constexpr uint8_t I2C_SDA = 21;
    constexpr uint8_t I2C_SCL = 22;

    // --- VOLTAGE DIVIDER SETTINGS ---
    /** R1 is the fixed pull-up resistor (Updated to 2k). */
    constexpr float R_PULLUP = 2000.0; 
    
    /** Measured Supply Voltage: Updated to your measured 5.02V for precision. */
    constexpr float V_SUPPLY = 5.02;

    // --- TEMPERATURE SENSOR CONSTANTS (NTC) ---
    /** BETA: Optimized for 70°C to 150°C range. */
    constexpr float NTC_BETA     = 3654.55; 
    /** R_NOMINAL: Resistance of the sensor at 20°C. */
    constexpr float NTC_R_NOM    = 2480.0;
    /** NTC_T_NOM: 20°C in Kelvin. */
    constexpr float NTC_T_NOM    = 293.15; 

    // --- PRESSURE SENSOR CONSTANTS ---
    constexpr float PRESS_C0     = 0.1;
    constexpr float PRESS_C1     = 0.0008;

    // --- PRESSURE CALIBRATION ---
    /** Initial pressure offset for auto-zero calibration. */
    constexpr float PRESSURE_OFFSET = 0.0;
}

/* --- Telemetry Structure --- */
typedef struct __attribute__((packed)) {
  float oilTemp;
  float waterTemp;
  uint32_t engineRPM;
  float oilPressure;
  float brakePressure;
  int brakePercent;
  float throttlePos;
  float speed;
  float accelPos;
} TelemetryData;

// Create global instances
Adafruit_ADS1115 ads;
TelemetryData myData;
float pressureOffset = Config::PRESSURE_OFFSET; // Auto-zero value initialized from config

/* --- ESP-NOW Configuration --- */
#define ESPNOW_WIFI_CHANNEL 6

class ESP_NOW_Broadcast_Peer : public ESP_NOW_Peer {
public:
  ESP_NOW_Broadcast_Peer(uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) 
    : ESP_NOW_Peer(ESP_NOW.BROADCAST_ADDR, channel, iface, lmk) {}

  ~ESP_NOW_Broadcast_Peer() { remove(); }

  bool begin() {
    if (!ESP_NOW.begin() || !add()) return false;
    return true;
  }

  bool send_message(const uint8_t *data, size_t len) {
    return send(data, len);
  }
};

ESP_NOW_Broadcast_Peer broadcast_peer(ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, nullptr);

/**
 * Converts ADC voltage to Temperature in Celsius
 */
double calculateTemperature(double v_measured) {
    if (v_measured >= Config::V_SUPPLY || v_measured <= 0.01) return -99.9; 
    double r_ntc = (v_measured * Config::R_PULLUP) / (Config::V_SUPPLY - v_measured);
    double steinhart = log(r_ntc / Config::NTC_R_NOM);
    steinhart /= Config::NTC_BETA;
    steinhart += 1.0 / Config::NTC_T_NOM;
    return (1.0 / steinhart) - 273.15; 
}

/**
 * Converts ADC voltage to Pressure in Bars (Raw)
 */
float calculatePressureBarRaw(float v_measured) {
    float v_clamped = constrain(v_measured, 0.5f, 4.5f);
    float pressureKPa = ((v_clamped / Config::V_SUPPLY) - Config::PRESS_C0) / Config::PRESS_C1;
    return pressureKPa / 100.0f;
}

void setup() {
    DEBUG_BEGIN(115200);
    delay(1000);
    DEBUG_PRINTLN("Initializing Combined Telemetry System...");

    Wire.begin(Config::I2C_SDA, Config::I2C_SCL);
    ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
    // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
    // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
    // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
    // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
    // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

    if (!ads.begin()) {
        DEBUG_PRINTLN("Critical Error: ADS1115 not found!");
        while (1) delay(10);
    }

    // --- Auto-Zero Calibration ---
    DEBUG_PRINTLN("Calibrating Pressure Offset (Keep Engine OFF)...");
    float sum = 0;
    for(int i = 0; i < 20; i++) {
        sum += calculatePressureBarRaw(ads.computeVolts(ads.readADC_SingleEnded(1)));
        delay(50);
    }
    pressureOffset = sum / 20.0;
    DEBUG_PRINTF("Calibration Done. Offset: %.2f Bar\n", pressureOffset);

    // --- Wi-Fi / ESP-NOW Setup ---
    WiFi.mode(WIFI_STA);
    WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
    while (!WiFi.STA.started()) { delay(100); }

    if (!broadcast_peer.begin()) {
        DEBUG_PRINTLN("ESP-NOW Init Failed!");
        delay(5000);
        ESP.restart();
    }
    DEBUG_PRINTLN("Telemetry Master Online.");
}

void loop() {
    int16_t adc[4];
    float volts[4];

    // 1. Read ADS1115 channels
    for (int i = 0; i < 4; i++) {
        adc[i] = ads.readADC_SingleEnded(i);
        volts[i] = ads.computeVolts(adc[i]);
    }

    // 2. Process data
    float rawOilPress = calculatePressureBarRaw(volts[1]);
    
    myData.oilTemp     = (float)calculateTemperature(volts[0]); 
    myData.oilPressure = rawOilPress - pressureOffset;
    if (myData.oilPressure < 0) myData.oilPressure = 0; // Floor to zero

    //TMP with button
    myData.waterTemp     = ((float)(volts[2]) / 5.0) * 140;   
    // Example/Static Data
    // myData.waterTemp     = 92.5;   
    myData.engineRPM     = 3500;   
    myData.brakePressure = 0.0;
    myData.brakePercent  = 0;
    myData.throttlePos   = 25.5;
    myData.speed         = 110.2;
    myData.accelPos      = 20.0;

    // 3. Print Local Debug Info (Raw vs Corrected)
    DEBUG_PRINTLN("-----------------------------------------------------------");
    for (int i = 0; i < 4; i++) {
        DEBUG_PRINTF("AIN%d: %-6d | %5.3f V\n", i, adc[i], volts[i]);
    }
    DEBUG_PRINTF("TEMP:  Raw: %5.2f °C | Corrected (5.02V): %5.2f °C\n", calculateTemperature(volts[0]), myData.oilTemp);
    DEBUG_PRINTF("PRESS: Raw: %5.2f Bar | Corrected (Zeroed): %5.2f Bar\n", rawOilPress, myData.oilPressure);
    DEBUG_PRINTF("FAKE Water Temp:  %5.2f °C", myData.waterTemp);
    DEBUG_PRINTF("---");

    // 4. Broadcast via ESP-NOW
    bool success = broadcast_peer.send_message((uint8_t *)&myData, sizeof(myData));
    DEBUG_PRINTF("ESP-NOW Broadcast: %s\n", success ? "Success" : "Failed");

    
    
    

    // SLEEP CYCLE
    delay(30); // Slightly slower loop for readability

    // Not working
    // // 5. Enter Light Sleep
    // // This stops the CPU and lowers radio power until the timer hits
    // if(esp_sleep_enable_timer_wakeup(200 * 1000) == ESP_OK) {
    //     DEBUG_PRINTLN("Entering Light Sleep for 200ms...");
    //     esp_light_sleep_start();
    // } else {
    //     DEBUG_PRINTLN("Failed to set wakeup timer!");
    // }
}