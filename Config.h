#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "driver/twai.h"

/**
 * @file Config.h
 * @brief Central configuration file for MX5 NC Brain ESP32 telemetry system
 *
 * Hardware: ESP32-C6 DevKit
 * Peripherals: ADS1115 ADC, TWAI (CAN Bus), ESP-NOW
 */

// =============================================================================
// GENERAL CONFIGURATION
// =============================================================================
namespace GeneralConfig {
    // --- Hardware Pins - ESP32-C6 ---
    // I2C SDA (Serial Data) pin - connects to ADS1115 SDA
    constexpr uint8_t I2C_SDA = 22;

    // I2C SCL (Serial Clock) pin - connects to ADS1115 SCL
    constexpr uint8_t I2C_SCL = 23;

    // --- Serial Debug ---
    // Serial baud rate for debug output (115200 is standard for ESP32)
    constexpr uint32_t SERIAL_BAUD_RATE = 115200;

    // --- ESP-NOW Wireless ---
    // WiFi channel for ESP-NOW communication (1-13, must match on all devices)
    constexpr uint8_t ESPNOW_WIFI_CHANNEL = 6;

    // --- Timing ---
    // Main loop delay in milliseconds (controls telemetry update rate)
    // 500ms = 2 updates per second, 100ms = 10 updates per second, etc.
    constexpr int MAIN_LOOP_DELAY_MS = 50;

    // --- Button Configuration ---
    // GPIO pin for push button input
    constexpr uint8_t BUTTON_PIN = 1;

    // Button debounce time in milliseconds
    constexpr uint8_t BUTTON_DEBOUNCE_MS = 50;

    // --- Potentiometer Configuration ---
    // GPIO pin for potentiometer (luminosity control)
    constexpr uint8_t POT_PIN = 2;

    // Potentiometer voltage range (connected to 3.3V)
    constexpr float POT_V_MAX = 3.3;

    // Luminosity range mapping (0V → 10%, 3.3V → 100%)
    constexpr uint8_t LUMINOSITY_MIN = 10;
    constexpr uint8_t LUMINOSITY_MAX = 100;
}

// =============================================================================
// WEB SERVER CONFIGURATION (WiFi AP + Dashboard)
// =============================================================================
namespace WebServerConfig {
    // WiFi Access Point credentials
    constexpr const char* AP_SSID = "MX5_Dashboard";
    constexpr const char* AP_PASSWORD = "laepaithio565";  // Min 8 characters

    // HTTP server port
    constexpr uint16_t PORT = 80;

    // Dashboard data refresh interval in milliseconds
    constexpr uint16_t REFRESH_INTERVAL_MS = 500;

    // FreeRTOS task stack size in bytes
    constexpr uint16_t TASK_STACK_SIZE = 4096;
}

// =============================================================================
// CAN BUS CONFIGURATION (TWAI - Two-Wire Automotive Interface)
// =============================================================================
namespace CANConfig {
    // GPIO pin for CAN TX (Transmit) - connects to CAN transceiver TX pin
    constexpr gpio_num_t TX_GPIO = (gpio_num_t)11;

    // GPIO pin for CAN RX (Receive) - connects to CAN transceiver RX pin
    constexpr gpio_num_t RX_GPIO = (gpio_num_t)10;

    // CAN bus speed in bits per second (500kbps is standard for automotive)
    constexpr uint32_t SPEED = 500000;
}

// =============================================================================
// ADS1115 ADC CONFIGURATION (Analog-to-Digital Converter)
// =============================================================================
namespace ADS1115Config {
    // --- Voltage Divider Settings (for NTC thermistor) ---
    // Fixed pull-up resistor value in Ohms (connects between V_SUPPLY and NTC)
    // Used in voltage divider: V_SUPPLY --- R_PULLUP --- [measurement point] --- NTC --- GND
    constexpr float R_PULLUP = 4700.0;

    // Measured supply voltage in Volts (calibrated for accuracy)
    // Should be measured with multimeter for best temperature readings
    constexpr float V_SUPPLY = 5.02;

    // --- NTC Thermistor Configuration (Oil Temperature Sensor) ---
    // BETA coefficient for NTC thermistor (material constant)
    // Optimized for 70°C to 150°C range typical of engine oil temperature
    constexpr float NTC_BETA = 3654.55;

    // Nominal resistance of NTC at reference temperature (20°C) in Ohms
    // Specific to the NTC thermistor model being used
    constexpr float NTC_R_NOM = 2480.0;

    // Nominal temperature in Kelvin (20°C = 293.15K)
    // Reference temperature for NTC_R_NOM resistance value
    constexpr float NTC_T_NOM = 293.15;

    // --- Pressure Sensor Configuration (Oil Pressure Sensor) ---
    // Pressure sensor linear equation: Pressure(kPa) = ((V / V_SUPPLY) - C0) / C1
    // Y-intercept constant for voltage-to-pressure conversion
    constexpr float PRESS_C0 = 0.1;

    // Slope constant for voltage-to-pressure conversion (kPa per volt ratio)
    constexpr float PRESS_C1 = 0.0008;

    // --- Pressure Calibration Settings ---
    // Initial pressure offset in Bar (0.0 = will be calibrated at startup)
    // Auto-zero calibration determines actual offset with engine OFF
    constexpr float PRESSURE_OFFSET = 0.0;

    // Number of samples to average during pressure calibration
    // More samples = more accurate but slower calibration
    constexpr int CALIBRATION_SAMPLES = 20;

    // Delay in milliseconds between each calibration sample
    // Allows sensor reading to settle
    constexpr int CALIBRATION_DELAY_MS = 50;
}

#endif // CONFIG_H
