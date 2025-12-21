#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>

// --- Configuration ---
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

Adafruit_ADS1115 ads;

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
    Serial.println("Initializing...");

    // Initialize I2C for ESP32-C6
    Wire.begin(Config::I2C_SDA, Config::I2C_SCL);

    // --- ADC Gain Configuration ---
    // Note: Be careful never to exceed VDD +0.3V max!
    //                                                                ADS1015  ADS1115
    //                                                                -------  -------
    ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
    // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
    // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
    // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
    // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
    // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

    if (!ads.begin()) {
        Serial.println("Critical Error: ADS1115 not found!");
        while (1) delay(10);
    }

    Serial.println("ADS1115 Online and Ready.");
}

void loop() {
    int16_t adc[4];
    float volts[4];

    // Read and store all 4 channels
    for (int i = 0; i < 4; i++) {
        adc[i] = ads.readADC_SingleEnded(i);
        volts[i] = ads.computeVolts(adc[i]);
    }

    // Process converted values (AIN0 = Temp, AIN1 = Pressure)
    double tempC    = calculateTemperature(volts[0]); 
    float  pressBar = calculatePressureBar(volts[1]);  

    // Print raw AIN0..3 data
    Serial.println("-----------------------------------------------------------");
    for (int i = 0; i < 4; i++) {
        Serial.printf("AIN%d: %-6d | %5.3f V\n", i, adc[i], volts[i]);
    }
    
    // Print interpreted sensor data
    Serial.println("--- High-End Optimized Readings (70-150C) ---");
    Serial.printf("Temperature: %6.2f °C\n", tempC);
    Serial.printf("Pressure:    %6.2f Bar\n", pressBar);
    
    delay(1000);
}