#ifndef SENSOR_UTILS_H
#define SENSOR_UTILS_H

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>

/**
 * @file SensorUtils.h
 * @brief Utility functions for sensor data processing
 *
 * Provides temperature and pressure calculation functions
 * for NTC thermistors and pressure sensors.
 */

namespace SensorUtils {
    /**
     * @brief Convert ADC voltage to temperature using NTC thermistor
     *
     * Uses the Steinhart-Hart equation to convert resistance to temperature.
     * Assumes a voltage divider with NTC as the lower resistor.
     *
     * @param v_measured Measured voltage from ADC (V)
     * @return Temperature in Celsius, or -99.9 if out of range
     */
    double calculateTemperature(double v_measured);

    /**
     * @brief Convert ADC voltage to pressure (raw, uncalibrated)
     *
     * Calculates pressure from sensor voltage using linear relationship.
     *
     * @param v_measured Measured voltage from ADC (V)
     * @return Pressure in Bar (uncalibrated)
     */
    float calculatePressureBarRaw(float v_measured);

    /**
     * @brief Perform auto-zero calibration for pressure sensor
     *
     * Reads multiple samples and averages them to determine the
     * zero-pressure offset. Engine must be OFF during calibration.
     *
     * @param ads Reference to ADS1115 ADC object
     * @param channel ADC channel to read (0-3)
     * @return Calculated pressure offset in Bar
     */
    float calibratePressureOffset(Adafruit_ADS1115 &ads, uint8_t channel);
}

#endif // SENSOR_UTILS_H
