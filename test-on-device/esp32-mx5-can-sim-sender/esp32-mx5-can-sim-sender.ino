#include <Arduino.h>
#include "driver/twai.h"

#define CAN_TX_GPIO (gpio_num_t)5
#define CAN_RX_GPIO (gpio_num_t)4
#define POTENTIOMETER_PIN 2  // Potentiometer for water temperature control

// Simulated sensor values
float engineRPM = 800.0;      // Start at idle
float speed = 0.0;            // km/h
float accelPos = 0.0;         // 0-100%
float throttlePos = 0.0;      // 0-100%
float waterTemp = 85.0;       // Coolant temp in C (will be read from potentiometer)
float brakePressure = 0.0;    // kPa
int brakePercent = 0;         // 0-100%

// Simulation state
unsigned long lastUpdate = 0;
bool accelerating = true;
uint32_t rngState = 12345;

// Simple pseudo-random number generator
uint32_t simpleRandom(uint32_t max) {
  rngState = (rngState * 1103515245 + 12345) & 0x7fffffff;
  return rngState % max;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("MX5 NC CAN Bus Simulator");

  // Configure potentiometer pin
  pinMode(POTENTIOMETER_PIN, INPUT);
  Serial.println("Potentiometer configured on pin 19");

  // Configure TWAI (CAN)
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install and start TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("TWAI driver installed.");
  } else {
    Serial.println("Failed to install TWAI driver.");
    while (1);
  }

  if (twai_start() == ESP_OK) {
    Serial.println("TWAI driver started.");
  } else {
    Serial.println("Failed to start TWAI driver.");
    while (1);
  }
}

void updateSimulatedValues() {
  // Simulate driving scenario with acceleration/deceleration cycles
  if (accelerating) {
    accelPos += 2.0f;
    if (accelPos > 100.0f) accelPos = 100.0f;

    throttlePos += 1.5f;
    if (throttlePos > 100.0f) throttlePos = 100.0f;

    engineRPM += 100.0f;
    if (engineRPM > 7000.0f) engineRPM = 7000.0f;

    speed += 2.0f;
    if (speed > 180.0f) speed = 180.0f;

    brakePressure = 0.0f;
    brakePercent = 0;

    if (speed >= 150.0f || engineRPM >= 6500.0f) {
      accelerating = false;  // Start braking
    }
  } else {
    accelPos -= 3.0f;
    if (accelPos < 0.0f) accelPos = 0.0f;

    throttlePos -= 2.5f;
    if (throttlePos < 0.0f) throttlePos = 0.0f;

    engineRPM -= 150.0f;
    if (engineRPM < 800.0f) engineRPM = 800.0f;

    speed -= 4.0f;
    if (speed < 0.0f) speed = 0.0f;

    // Simulate braking
    if (speed > 20.0f) {
      brakePressure = 1.5f + simpleRandom(50) / 100.0f;  // 1.5-2.0 kPa
      brakePercent = 60 + simpleRandom(30);               // 60-90%
    } else {
      brakePressure = 0.0f;
      brakePercent = 0;
    }

    if (speed <= 10.0f) {
      accelerating = true;  // Start accelerating again
    }
  }

  // Read water temperature from potentiometer on pin 19
  // ADC range: 0-4095, map to temperature range: 40-140°C
  int potValue = analogRead(POTENTIOMETER_PIN);
  waterTemp = 40.0f + (potValue / 4095.0f) * 100.0f;  // Maps 0-4095 to 40-140°C
}

void sendCAN_0x201() {
  // Engine RPM, Vehicle Speed, Accel Position
  twai_message_t msg;
  msg.identifier = 0x201;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = 8;

  // RPM: Raw = RPM * 4
  uint16_t rpmRaw = (uint16_t)(engineRPM * 4.0);
  msg.data[0] = (rpmRaw >> 8) & 0xFF;
  msg.data[1] = rpmRaw & 0xFF;

  msg.data[2] = 0x00;  // Unused
  msg.data[3] = 0x00;  // Unused

  // Speed: Raw = (Speed + 100) * 100
  uint16_t speedRaw = (uint16_t)((speed + 100.0) * 100.0);
  msg.data[4] = (speedRaw >> 8) & 0xFF;
  msg.data[5] = speedRaw & 0xFF;

  // Accel Position: Raw = AccelPos / 2
  uint8_t accelRaw = (uint8_t)(accelPos / 2.0);
  msg.data[6] = accelRaw;
  msg.data[7] = 0x00;  // Unused

  if (twai_transmit(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
    Serial.printf("  RPM: %d (0x201[0-1]=0x%04X) | Speed: %.1f km/h (0x201[4-5]=0x%04X) | Accel: %.1f%% (0x201[6]=0x%02X)\n",
                  (int)engineRPM, rpmRaw, speed, speedRaw, accelPos, accelRaw);
  } else {
    Serial.println("  [ERROR] Failed to send 0x201");
  }
}

void sendCAN_0x215() {
  // Throttle Valve Position
  twai_message_t msg;
  msg.identifier = 0x215;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = 8;

  msg.data[0] = 0x00;
  msg.data[1] = 0x00;
  msg.data[2] = 0x00;
  msg.data[3] = 0x00;
  msg.data[4] = 0x00;
  msg.data[5] = 0x00;

  // Throttle: Raw = (ThrottlePos * 255) / 100
  uint8_t throttleRaw = (uint8_t)((throttlePos * 255.0) / 100.0);
  msg.data[6] = throttleRaw;
  msg.data[7] = 0x00;

  if (twai_transmit(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
    Serial.printf("  Throttle: %.1f%% (0x215[6]=0x%02X)\n", throttlePos, throttleRaw);
  } else {
    Serial.println("  [ERROR] Failed to send 0x215");
  }
}

void sendCAN_0x240() {
  // Engine Coolant Temperature
  twai_message_t msg;
  msg.identifier = 0x240;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = 8;

  msg.data[0] = 0x00;

  // Coolant Temp: Raw = Temp + 40
  uint8_t coolantRaw = (uint8_t)(waterTemp + 40.0);
  msg.data[1] = coolantRaw;

  msg.data[2] = 0x00;
  msg.data[3] = 0x00;
  msg.data[4] = 0x00;
  msg.data[5] = 0x00;
  msg.data[6] = 0x00;
  msg.data[7] = 0x00;

  if (twai_transmit(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
    Serial.printf("  Coolant: %.1fC (0x240[1]=0x%02X)\n", waterTemp, coolantRaw);
  } else {
    Serial.println("  [ERROR] Failed to send 0x240");
  }
}

void sendCAN_0x085() {
  // Brake Pressure
  twai_message_t msg;
  msg.identifier = 0x085;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = 8;

  // Brake Pressure: Raw = (Pressure * 1000 + 327.27) / 3.4518689053
  uint16_t rawBrake = (uint16_t)((brakePressure * 1000.0 + 327.27) / 3.4518689053);
  msg.data[0] = (rawBrake >> 8) & 0xFF;
  msg.data[1] = rawBrake & 0xFF;

  msg.data[2] = 0x00;
  msg.data[3] = 0x00;
  msg.data[4] = 0x00;
  msg.data[5] = 0x00;
  msg.data[6] = 0x00;
  msg.data[7] = 0x00;

  if (twai_transmit(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
    Serial.printf("  Brake: %.2f kPa / %d%% (0x085[0-1]=0x%04X)\n", brakePressure, brakePercent, rawBrake);
  } else {
    Serial.println("  [ERROR] Failed to send 0x085");
  }
}

void loop() {
  unsigned long now = millis();
  
  // Update simulated values every 50ms
  if (now - lastUpdate >= 50) {
    lastUpdate = now;
    updateSimulatedValues();
    
    // Send all CAN messages
    sendCAN_0x201();  // RPM, Speed, Accel
    vTaskDelay(pdMS_TO_TICKS(5));
    
    sendCAN_0x215();  // Throttle
    vTaskDelay(pdMS_TO_TICKS(5));
    
    sendCAN_0x240();  // Coolant Temp
    vTaskDelay(pdMS_TO_TICKS(5));
    
    sendCAN_0x085();  // Brake Pressure
    
    Serial.println("---");
  }
  
  vTaskDelay(pdMS_TO_TICKS(100));
}
