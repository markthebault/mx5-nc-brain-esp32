#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include "driver/twai.h"

/**
 * @file Config.h
 * @brief Central configuration file for MX5 NC Brain ESP32 telemetry system
 */

// --- CAN Bus Configuration ---
#define CAN_TX_GPIO     (gpio_num_t)11
#define CAN_RX_GPIO     (gpio_num_t)10
#define CANBUS_SPEED    500000   // 500kbps

// --- ESP-NOW Configuration ---
#define ESPNOW_WIFI_CHANNEL 6

namespace Config {
    // --- Hardware Pins for ESP32-C6 ---
    constexpr uint8_t I2C_SDA = 5;
    constexpr uint8_t I2C_SCL = 6;

    // --- ADS1115 ADC Configuration ---
    constexpr uint32_t SERIAL_BAUD_RATE = 115200;

    // --- Voltage Divider Settings ---
    /** R1 is the fixed pull-up resistor (Ohms) */
    constexpr float R_PULLUP = 4700.0;

    /** Measured Supply Voltage (V) - Calibrated for precision */
    constexpr float V_SUPPLY = 5.02;

    // --- Temperature Sensor Constants (NTC) ---
    /** BETA coefficient: Optimized for 70°C to 150°C range */
    constexpr float NTC_BETA     = 3654.55;

    /** Nominal resistance at 20°C (Ohms) */
    constexpr float NTC_R_NOM    = 2480.0;

    /** Nominal temperature in Kelvin (20°C) */
    constexpr float NTC_T_NOM    = 293.15;

    // --- Pressure Sensor Constants ---
    constexpr float PRESS_C0     = 0.1;
    constexpr float PRESS_C1     = 0.0008;

    // --- Pressure Calibration ---
    /** Initial pressure offset for auto-zero calibration */
    constexpr float PRESSURE_OFFSET = 0.0;

    /** Number of samples for calibration averaging */
    constexpr int CALIBRATION_SAMPLES = 20;

    /** Delay between calibration samples (ms) */
    constexpr int CALIBRATION_DELAY_MS = 50;

    // --- Timing Configuration ---
    /** Main loop delay (ms) */
    constexpr int MAIN_LOOP_DELAY_MS = 500;
}

#endif // CONFIG_H
