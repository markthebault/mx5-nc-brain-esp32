/**
 * @file mx5-nc-brain-esp32.ino
 * @brief Main telemetry system for Mazda MX5 NC
 *
 * This system reads sensor data (oil temp, oil pressure) via ADS1115,
 * receives CAN bus data (RPM, speed, throttle, etc.) from the vehicle ECU,
 * and broadcasts all telemetry via ESP-NOW to remote displays.
 *
 * Hardware: ESP32-C6
 * Author: Mark Thebault
 * Date: 2025
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// Project headers
#include "Config.h"
#include "Debug.h"
#include "TelemetryData.h"
#include "SensorUtils.h"
#include "CANBus.h"
#include "ESPNowBroadcast.h"

// --- Global Objects ---
Adafruit_ADS1115 ads;
TelemetryData myData;
ESP_NOW_Broadcast_Peer broadcast_peer(GeneralConfig::ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, nullptr);

// --- Calibration Data ---
float pressureOffset = ADS1115Config::PRESSURE_OFFSET;

void setup() {
    DEBUG_BEGIN(GeneralConfig::SERIAL_BAUD_RATE);
    delay(1000);
    DEBUG_PRINTLN("===========================================");
    DEBUG_PRINTLN("  MX5 NC Telemetry System - Initializing");
    DEBUG_PRINTLN("===========================================");

    // --- I2C and ADS1115 Setup ---
    DEBUG_PRINTLN("Initializing I2C and ADS1115...");
    Wire.begin(GeneralConfig::I2C_SDA, GeneralConfig::I2C_SCL);

    // Configure ADC gain
    // NOTE: Keep all gain options below for reference - DO NOT REMOVE
    ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
    // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
    // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
    // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
    // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
    // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

    if (!ads.begin()) {
        DEBUG_PRINTLN("CRITICAL ERROR: ADS1115 not found!");
        while (1) delay(10);
    }
    DEBUG_PRINTLN("ADS1115 initialized successfully.");

    // --- Auto-Zero Calibration for Pressure Sensor ---
    pressureOffset = SensorUtils::calibratePressureOffset(ads, 1);

    // --- Wi-Fi / ESP-NOW Setup ---
    if (!ESPNowBroadcast::init(broadcast_peer)) {
        DEBUG_PRINTLN("ESP-NOW initialization failed! Restarting...");
        delay(5000);
        ESP.restart();
    }

    // --- CAN Bus Setup ---
    CANBus::init();
    CANBus::startReceiveTask(&myData);

    // --- Initialize test data for debugging ---
    myData.engineRPM     = 3666;
    myData.brakePressure = 5.0;
    myData.brakePercent  = 0;
    myData.throttlePos   = 25.5;
    myData.speed         = 110.2;
    myData.accelPos      = 20.0;

    DEBUG_PRINTLN("===========================================");
    DEBUG_PRINTLN("  Telemetry Master Online");
    DEBUG_PRINTLN("===========================================");
}

void loop() {
    // --- 1. Read ADS1115 Channels ---
    int16_t adc[4];
    float volts[4];

    for (int i = 0; i < 4; i++) {
        adc[i] = ads.readADC_SingleEnded(i);
        volts[i] = ads.computeVolts(adc[i]);
    }

    // --- 2. Process Sensor Data ---
    // Oil Temperature (Channel 0)
    myData.oilTemp = (float)SensorUtils::calculateTemperature(volts[0]);

    // Oil Pressure (Channel 1)
    float rawOilPress = SensorUtils::calculatePressureBarRaw(volts[1]);
    myData.oilPressure = rawOilPress - pressureOffset;
    if (myData.oilPressure < 0) {
        myData.oilPressure = 0;  // Floor to zero
    }

    // Water Temperature - Temporary simulation with button (Channel 2)
    myData.waterTemp = ((float)(volts[2]) / 5.0) * 140;
    // NOTE: Will be replaced by CAN bus data (ID 0x240) when available

    // --- 3. Debug Output ---
    DEBUG_PRINTLN("-----------------------------------------------------------");
    for (int i = 0; i < 4; i++) {
        DEBUG_PRINTF("AIN%d: %-6d | %5.3f V\n", i, adc[i], volts[i]);
    }
    DEBUG_PRINTF("Oil Temp:   %.2f °C\n", myData.oilTemp);
    DEBUG_PRINTF("Oil Press:  %.2f Bar (Raw: %.2f)\n", myData.oilPressure, rawOilPress);
    DEBUG_PRINTF("Water Temp: %.2f °C\n", myData.waterTemp);
    DEBUG_PRINTF("Engine RPM: %d\n", myData.engineRPM);
    DEBUG_PRINTF("Speed:      %.2f km/h\n", myData.speed);
    DEBUG_PRINTF("Throttle:   %.1f%%\n", myData.throttlePos);
    DEBUG_PRINTF("Accel Pos:  %.1f%%\n", myData.accelPos);
    DEBUG_PRINTF("Brake:      %.2f kPa (%d%%)\n", myData.brakePressure, myData.brakePercent);

    // --- 4. Broadcast Telemetry via ESP-NOW ---
    ESPNowBroadcast::broadcastTelemetry(broadcast_peer, myData);

    // --- 5. Test CAN Bus Transmission (Uncomment for testing) ---
    // NOTE: Requires TWAI_MODE_NO_ACK in CANBus::init()
    // CANBus::sendTestMessages();

    // --- 6. Loop Delay ---
    delay(GeneralConfig::MAIN_LOOP_DELAY_MS);
}
