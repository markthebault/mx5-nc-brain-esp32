/*
    ESP-NOW Broadcast Telemetry Master
    Updated for Multi-Sensor Struct Transmission
*/

#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>

/* --- Telemetry Structure --- */
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

// Create a global instance of the data
TelemetryData myData;

/* --- Configuration --- */
#define ESPNOW_WIFI_CHANNEL 6
const int analogInPin = A0; 

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

void setup() {
  Serial.begin(115200);
  
  // ADC Configuration
  analogReadResolution(12);

  // Initialize Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) { delay(100); }

  // Register the broadcast peer
  if (!broadcast_peer.begin()) {
    Serial.println("Failed to initialize broadcast peer. Restarting...");
    delay(5000);
    ESP.restart();
  }

  Serial.println("Telemetry Master Online");
}

void loop() {
  // 1. Read your sensors and fill the struct
  // (Example: reading oil temperature/pressure from analog pins)
  int sensorValue = analogRead(analogInPin);
  
  myData.oilTemp       = map(sensorValue, 0, 3330, 0, 170); // Example mapping
  myData.waterTemp     = 92.5;   // Static example
  myData.engineRPM     = 3500;   // Static example
  myData.oilPressure   = 4.2;    // Static example
  myData.brakePressure = 0.0;
  myData.brakePercent  = 0;
  myData.throttlePos   = 25.5;
  myData.speed         = 110.2;
  myData.accelPos      = 20.0;

  // 2. Broadcast the entire struct as raw bytes
  Serial.print("Broadcasting Telemetry... ");
  
  bool success = broadcast_peer.send_message((uint8_t *)&myData, sizeof(myData));

  if (success) {
    Serial.println("Success");
  } else {
    Serial.println("Failed");
  }

  // Debug local values
  Serial.printf("Oil Temp: %.2f | RPM: %u\n", myData.oilTemp, myData.engineRPM);

  delay(100); // 10Hz update rate
}