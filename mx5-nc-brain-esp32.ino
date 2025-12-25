#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>
#include "driver/twai.h"

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
#define CAN_TX_GPIO     (gpio_num_t)11
#define CAN_RX_GPIO     (gpio_num_t)10

#define CANBUS_SPEED    500000   // 500kbps

namespace Config {
    // Hardware Pins for ESP32-C6
    constexpr uint8_t I2C_SDA = 5;
    constexpr uint8_t I2C_SCL = 6;

    // --- VOLTAGE DIVIDER SETTINGS ---
    /** R1 is the fixed pull-up resistor */
    constexpr float R_PULLUP = 4700.0; 
    
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


void canbus_init(void);
void test_can_send();
void can_receive_task(void *arg);


double calculateTemperature(double v_measured);
float calculatePressureBarRaw(float v_measured);

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


    // --- CAN Bus Setup ---
    canbus_init();
    xTaskCreate(can_receive_task, "can_receive_task", 4096, NULL, 5, NULL);


    /// Debug purpose:
    myData.engineRPM     = 3666;   
    myData.brakePressure = 5.0;
    myData.brakePercent  = 0;
    myData.throttlePos   = 25.5;
    myData.speed         = 110.2;
    myData.accelPos      = 20.0;

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
    // // Example/Static Data
    // // myData.waterTemp     = 92.5;   
    // myData.engineRPM     = 3500;   
    // myData.brakePressure = 0.0;
    // myData.brakePercent  = 0;
    // myData.throttlePos   = 25.5;
    // myData.speed         = 110.2;
    // myData.accelPos      = 20.0;
    

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

    
    // Test can bus requires TWAI_MODE_NO_ACK instead of TWAI_MODE_NORMAL
    //test_can_send();
    

    // SLEEP CYCLE
    // delay(30);
    delay(500); // Slightly slower loop for readability

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




void canbus_init(void) {

  // Configure TWAI (CAN)
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    // twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NO_ACK);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();  // Accept all IDs
 
    // Install and start TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("TWAI driver installed.");
    } else {
        Serial.println("Failed to install TWAI driver.");
        while (1);
    }

    if (twai_start() == ESP_OK) {
        Serial.println("TWAI driver started. Listening for messages...");
    } else {
        Serial.println("Failed to start TWAI driver.");
        while (1);
    }
}

void can_receive_task(void *arg) {
    twai_message_t rx_msg;
    for (;;) {
        // Wait for a message (100ms timeout)
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(100)) == ESP_OK) {
            
            switch (rx_msg.identifier) {

                case 0x085: { // Braking System (DSC cars)
                    // Brake Pressure (kPa): (3.4518689053 * bytesToInt(raw, 0, 2) - 327.27) / 1000.0
                    uint16_t rawBrake = (rx_msg.data[0] << 8) | rx_msg.data[1];
                    myData.brakePressure = (3.4518689053 * rawBrake - 327.27) / 1000.0;
                    
                    // Brake Percentage: min(0.2 * (bytesToInt(raw, 0, 2) - 102), 100)
                    myData.brakePercent = (int)max(0.0, min(0.2 * (rawBrake - 102.0), 100.0));
                    
                    DEBUG_PRINTF("CAN 0x085: Brake Press: %.2f kPa | Brake %%: %d\n", myData.brakePressure, myData.brakePercent);
                    break;
                }

                case 0x201: { // Engine Speed, Vehicle Speed, Accel Position
                    // Engine RPM: Bytes 0 & 1 / 4.0
                    uint16_t rpmRaw = (rx_msg.data[0] << 8) | rx_msg.data[1];
                    myData.engineRPM = rpmRaw / 4; 

                    // Speed: ((Raw / 100.0) - 100.0). Note: Result is km/h.
                    uint16_t speedRaw = (rx_msg.data[4] << 8) | rx_msg.data[5];
                    myData.speed = ((float)speedRaw / 100.0) - 100.0;

                    // Accelerator Position: Byte 6 * 2.0 (percentage)
                    myData.accelPos = rx_msg.data[6] * 2.0;

                    DEBUG_PRINTF("CAN 0x201: RPM: %d | Speed: %.2f km/h | Accel: %.1f%%\n", myData.engineRPM, myData.speed, myData.accelPos);
                    break;
                }

                case 0x215: { // Throttle Valve Position
                    // Throttle: Byte 6 * 100 / 255.0
                    myData.throttlePos = (rx_msg.data[6] * 100.0) / 255.0;
                    DEBUG_PRINTF("CAN 0x215: Throttle: %.1f%%\n", myData.throttlePos);
                    break;
                }

                case 0x240: { // Engine Temps
                    // Coolant Temperature: Byte 1 - 40
                    myData.waterTemp = (float)(rx_msg.data[1] - 40);
                    DEBUG_PRINTF("CAN 0x240: Coolant: %.1f C\n", myData.waterTemp);
                    break;
                }
            }
        }
    }
}

void test_can_send() {
    twai_message_t tx_msg;
    tx_msg.extd = 0;
    tx_msg.data_length_code = 8;
    tx_msg.self = 1; 

    // --- TEST 1: ID 0x201 (RPM, Speed, Accel) ---
    tx_msg.identifier = 0x201;
    // RPM: Want 3500. Equation: Raw / 4 = 3500 -> Raw = 14000 (0x36B0)
    tx_msg.data[0] = 0x36; 
    tx_msg.data[1] = 0xB0; 
    // Speed: Want 120 km/h. Equation: (Raw/100) - 100 = 120 -> Raw = 22000 (0x55F0)
    tx_msg.data[4] = 0x55;
    tx_msg.data[5] = 0xF0;
    // Accel: Want 25%. Equation: Raw * 2 = 25 -> Raw = 12.5 (0x0D approx)
    tx_msg.data[6] = 0x0D; 
    
    twai_transmit(&tx_msg, pdMS_TO_TICKS(10));

    // --- TEST 2: ID 0x085 (Brake Pressure) ---
    tx_msg.identifier = 0x085;
    // Brake: Let's simulate moderate braking. Raw = 500 (0x01F4)
    // Pressure: (3.45 * 500 - 327) / 1000 = ~1.39 kPa
    // Percent: 0.2 * (500 - 102) = ~79%
    tx_msg.data[0] = 0x01;
    tx_msg.data[1] = 0xF4;

    twai_transmit(&tx_msg, pdMS_TO_TICKS(10));
}


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