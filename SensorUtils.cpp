#include "SensorUtils.h"
#include "Config.h"
#include "Debug.h"
#include <Adafruit_ADS1X15.h>
#include <math.h>

namespace SensorUtils {

    double calculateTemperature(double v_measured) {
        // Validate input voltage range
        if (v_measured >= ADS1115Config::V_SUPPLY || v_measured <= 0.01) {
            return -99.9;  // Error value for out-of-range
        }

        // Calculate NTC resistance from voltage divider
        double r_ntc = (v_measured * ADS1115Config::R_PULLUP) / (ADS1115Config::V_SUPPLY - v_measured);

        // Apply Steinhart-Hart equation (simplified Beta model)
        double steinhart = log(r_ntc / ADS1115Config::NTC_R_NOM);
        steinhart /= ADS1115Config::NTC_BETA;
        steinhart += 1.0 / ADS1115Config::NTC_T_NOM;

        // Convert to Celsius
        return (1.0 / steinhart) - 273.15;
    }

    float calculatePressureBarRaw(float v_measured) {
        // Clamp voltage to sensor's valid range
        float v_clamped = constrain(v_measured, 0.5f, 4.5f);

        // Convert voltage to pressure using linear calibration
        float pressureKPa = ((v_clamped / ADS1115Config::V_SUPPLY) - ADS1115Config::PRESS_C0) / ADS1115Config::PRESS_C1;

        // Convert kPa to Bar
        return pressureKPa / 100.0f;
    }

    float calibratePressureOffset(Adafruit_ADS1115 &ads, uint8_t channel) {
        DEBUG_PRINTLN("Calibrating Pressure Offset (Keep Engine OFF)...");

        float sum = 0.0;
        for (int i = 0; i < ADS1115Config::CALIBRATION_SAMPLES; i++) {
            int16_t adc = ads.readADC_SingleEnded(channel);
            float volts = ads.computeVolts(adc);
            sum += calculatePressureBarRaw(volts);
            delay(ADS1115Config::CALIBRATION_DELAY_MS);
        }

        float offset = sum / ADS1115Config::CALIBRATION_SAMPLES;
        DEBUG_PRINTF("Calibration Done. Offset: %.2f Bar\n", offset);

        return offset;
    }

}
