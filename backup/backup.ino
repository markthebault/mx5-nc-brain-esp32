#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>

// --- Configuration (from First Code) ---
namespace Config {
    // Hardware Pins for ESP32-C6
    constexpr uint8_t I2C_SDA = 21;
    constexpr uint8_t I2C_SCL = 22;

    // --- VOLTAGE DIVIDER SETTINGS ---
    /** R1 is the fixed pull-up resistor in your voltage divider (Ohms). */
    constexpr float R_PULLUP = 2000.0; 
    
    /** The reference voltage (Us) provided to Pin 3 (Pressure) and the Divider (Temp). 
     * This is taken directly from the "Characteristic Data" box in the image. */
    constexpr float V_SUPPLY = 5.0;

    // --- TEMPERATURE SENSOR CONSTANTS (NTC) ---
    /** BETA: Material constant optimized for the 70°C to 150°C range via linear regression. 
     * Calculated using Python from the provided datasheet table. */
    constexpr float NTC_BETA     = 3654.55; 
    // constexpr float NTC_BETA     = 3472.16; // Updated Beta based on full-table regression
    /** R_NOMINAL: Resistance of the sensor at 20°C (from the Bosch table). */
    constexpr float NTC_R_NOM    = 2480.0;
    /** NTC_T_NOM: Reference temperature of 20°C converted to Kelvin (20 + 273.15). */
    constexpr float NTC_T_NOM    = 293.15; 

    // --- PRESSURE SENSOR CONSTANTS ---
    /** C0: The transfer function offset (0.1) from the Bosch datasheet formula. */
    constexpr float PRESS_C0     = 0.1;
    /** C1: The transfer function sensitivity (0.0008 kPa^-1) from the Bosch datasheet. */
    constexpr float PRESS_C1     = 0.0008;
}

/* --- Telemetry Structure (from Second Code) --- */
// This MUST match the receiver's structure exactly
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

/* --- ESP-NOW Configuration --- */
#define ESPNOW_WIFI_CHANNEL 6

/* --- ESP-NOW Class --- */
class ESP_NOW_Broadcast_Peer : public ESP_NOW_Peer {
public:
  ESP_NOW_Broadcast_Peer(uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) 
    : ESP_NOW_Peer(ESP_NOW.BROADCAST_ADDR, channel, iface, lmk) {}

  ~ESP_NOW_Broadcast_Peer() { remove(); }

  bool begin() {
    if (!ESP_NOW.begin() || !add()) {
      log_e("Failed to initialize ESP-NOW");
      return false;
    }
    return true;
  }

  bool send_message(const uint8_t *data, size_t len) {
    if (!send(data, len)) {
      log_e("Failed to broadcast message");
      return false;
    }
    return true;
  }
};

ESP_NOW_Broadcast_Peer broadcast_peer(ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, nullptr);

/**
 * Converts ADC voltage to Temperature in Celsius
 * @param v_measured The voltage read by the microcontroller (0-5V) from the NTC divider
 * @return Temperature in Celsius
 */
double calculateTemperature(double v_measured) {
    // Safety check to prevent division by zero or log(0) if wire is disconnected
    if (v_measured >= Config::V_SUPPLY || v_measured <= 0.01) return -99.9; 

    // Calculate NTC resistance: R_ntc = (V_out * R1) / (V_supply - V_out)
    double r_ntc = (v_measured * Config::R_PULLUP) / (Config::V_SUPPLY - v_measured);

    // Beta Equation: 1/T = 1/T0 + 1/B * ln(R/R0)
    double steinhart = log(r_ntc / Config::NTC_R_NOM);
    steinhart /= Config::NTC_BETA;
    steinhart += 1.0 / Config::NTC_T_NOM;
    
    return (1.0 / steinhart) - 273.15; // Result in Celsius
}

/**
 * Converts ADC voltage to Pressure in Bars
 * @param v_measured The voltage read by the microcontroller (0-5V) from Pin 2
 * @return Pressure in Bars
 */
float calculatePressureBar(float v_measured) {
    // Clamp to Bosch valid signal range (0.5V - 4.5V) per the characteristic graph
    float v_clamped = constrain(v_measured, 0.5f, 4.5f);

    // Bosch Formula: p = ((Uout / Us) - c0) / c1 (Result in kPa)
    float pressureKPa = ((v_clamped / Config::V_SUPPLY) - Config::PRESS_C0) / Config::PRESS_C1;
    
    return pressureKPa / 100.0f; // Convert kPa to Bar (100 kPa = 1 Bar)
}

void setup() {
    Serial.begin(115200);
    
    // Initializing after delay as requested
    delay(1000);
    Serial.println("Initializing Combined Telemetry System...");

    // Initialize I2C for ESP32-C6
    Wire.begin(Config::I2C_SDA, Config::I2C_SCL);

    // --- ADS1115 Setup ---
    ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 0.1875mV (default)

    if (!ads.begin()) {
        Serial.println("Critical Error: ADS1115 not found!");
        while (1) delay(10);
    }
    Serial.println("ADS1115 Online.");

    // --- Wi-Fi / ESP-NOW Setup ---
    WiFi.mode(WIFI_STA);
    WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
    while (!WiFi.STA.started()) { delay(100); }

    if (!broadcast_peer.begin()) {
        Serial.println("Failed to initialize broadcast peer. Restarting...");
        delay(5000);
        ESP.restart();
    }

    Serial.println("Telemetry Master Online and Ready.");
}

void loop() {
    int16_t adc[4];
    float volts[4];

    // 1. Read ADS1115 channels
    for (int i = 0; i < 4; i++) {
        adc[i] = ads.readADC_SingleEnded(i);
        volts[i] = ads.computeVolts(adc[i]);
    }

    // 2. Process sensor data into the Telemetry struct
    // AIN0 = Oil Temp, AIN1 = Oil Pressure
    myData.oilTemp       = (float)calculateTemperature(volts[0]); 
    myData.oilPressure   = calculatePressureBar(volts[1]);  
    
    // Static / Example data for remaining fields
    myData.waterTemp     = 92.5;   
    myData.engineRPM     = 3500;   
    myData.brakePressure = 0.0;
    myData.brakePercent  = 0;
    myData.throttlePos   = 25.5;
    myData.speed         = 110.2;
    myData.accelPos      = 20.0;

    // 3. Print Local Debug Info
    Serial.println("-----------------------------------------------------------");
    for (int i = 0; i < 4; i++) {
        Serial.printf("AIN%d: %-6d | %5.3f V\n", i, adc[i], volts[i]);
    }
    Serial.printf("Calculated Oil Temp: %.2f °C | Oil Pressure: %.2f Bar\n", myData.oilTemp, myData.oilPressure);

    // 4. Broadcast the entire struct via ESP-NOW
    Serial.print("Broadcasting Telemetry... ");
    bool success = broadcast_peer.send_message((uint8_t *)&myData, sizeof(myData));

    if (success) {
        Serial.println("Success");
    } else {
        Serial.println("Failed");
    }

    delay(100);
}