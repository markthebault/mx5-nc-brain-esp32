#ifndef TELEMETRY_DATA_H
#define TELEMETRY_DATA_H

#include <Arduino.h>

/**
 * @file TelemetryData.h
 * @brief Data structure definitions for MX5 NC telemetry system
 */

/**
 * @enum GaugeType
 * @brief Display mode for telemetry gauges
 */
enum GaugeType : uint8_t {
    GAUGE_NORMAL = 0,   // Normal/street gauge display
    GAUGE_RACING = 1    // Racing gauge display
};

/**
 * @struct TelemetryData
 * @brief Packed structure containing all vehicle telemetry data
 *
 * This structure is transmitted via ESP-NOW to receiving devices.
 * The __attribute__((packed)) ensures no padding is added between fields.
 */
typedef struct __attribute__((packed)) {
    float oilTemp;         // Oil temperature in Celsius
    float waterTemp;       // Coolant temperature in Celsius
    uint32_t engineRPM;    // Engine RPM
    float oilPressure;     // Oil pressure in Bar
    float brakePressure;   // Brake pressure in kPa
    int brakePercent;      // Brake percentage (0-100)
    float throttlePos;     // Throttle position percentage
    float speed;           // Vehicle speed in km/h
    float accelPos;        // Accelerator pedal position percentage
    GaugeType gaugeType;   // Display mode: normal or racing
} TelemetryData;

#endif // TELEMETRY_DATA_H
