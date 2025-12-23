#include <U8g2lib.h>
#include <Wire.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>
#include <vector>

/* --- Telemetry Structure (MUST MATCH MASTER) --- */
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

/* Definitions */
#define ESPNOW_WIFI_CHANNEL 6

// OLED Setup for C3 0.43" (Adjust pins if necessary, usually 6 & 5 for SDA/SCL)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 6, 5);
int width = 72;
int height = 40;
int xOffset = 28; // = (128-w)/2
int yOffset = 24; // = (64-h) //not div/2 I don't know why

/* Classes */

int data_received = 0;

class ESP_NOW_Peer_Class : public ESP_NOW_Peer {
public:
  ESP_NOW_Peer_Class(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) 
    : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}

  bool add_peer() {
    if (!add()) {
      log_e("Failed to register the broadcast peer");
      return false;
    }
    return true;
  }

  // --- UPDATED RECEIVE FUNCTION ---
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    // 1. Verify data size matches our struct
    if (len != sizeof(TelemetryData)) {
      Serial.printf("Received packet of unknown size: %d\n", len);
      return;
    }

    // 2. Map the raw bytes to our TelemetryData struct
    TelemetryData* incoming = (TelemetryData*)data;

    // 3. Print to Serial for debugging
    Serial.printf("Oil P: %.1f bar | Oil T: %.1f C| Water T: %.1f C\n", 
                  incoming->oilPressure, incoming->oilTemp, incoming->waterTemp);

    // 4. Update OLED Display
    u8g2.clearBuffer();

    u8g2.setFont(u8g2_font_4x6_tr);
    
    u8g2.setCursor(xOffset, yOffset+7);
    u8g2.printf("Oil P: %.1f bar", incoming->oilPressure);
    
    u8g2.setCursor(xOffset, yOffset+14);
    u8g2.printf("Oil T: %.1f C", incoming->oilTemp);
    
    u8g2.setCursor(xOffset, yOffset+21);
    u8g2.printf("Water T: %.1f C", incoming->waterTemp);
    
    u8g2.setCursor(xOffset, yOffset+28);
    u8g2.printf("RPM: %d", (int)incoming->engineRPM);

    u8g2.sendBuffer();

    data_received = 1;
  }
};

/* Global Variables */
std::vector<ESP_NOW_Peer_Class *> masters;

/* Callbacks */
void register_new_master(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) {
    ESP_NOW_Peer_Class *new_master = new ESP_NOW_Peer_Class(info->src_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, nullptr);
    if (!new_master->add_peer()) {
      delete new_master;
      return;
    }
    masters.push_back(new_master);
  }
}

void setup() {
  Serial.begin(115200);

  // Setup Oled Screen
  u8g2.begin();
  u8g2.setContrast(255); 
  u8g2.setBusClock(400000); 

  // Initialize Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) { delay(100); }

  // Initialize ESP-NOW
  if (!ESP_NOW.begin()) {
    ESP.restart();
  }

  // Register the new peer callback
  ESP_NOW.onNewPeer(register_new_master, nullptr);

  Serial.println("Receiver Ready. Waiting for Telemetry...");
}

int count = 0;

void loop() {

  // Loop remains empty as processing happens in onReceive callback
  if(!data_received){
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_4x6_tr);
    u8g2.setCursor(xOffset, yOffset+7);
    u8g2.printf("Waiting for Data");
    u8g2.sendBuffer();
  }

  if (count >= 4){
    Serial.println("Data not received since 5s reset screen");
    data_received = 0;
    count = 0;
  }

  count++;

  delay(1000);
}