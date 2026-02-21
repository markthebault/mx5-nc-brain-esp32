#include "ESPNowBroadcast.h"
#include "Config.h"
#include "Debug.h"
#include <esp_mac.h>

// --- ESP_NOW_Broadcast_Peer Implementation ---

ESP_NOW_Broadcast_Peer::ESP_NOW_Broadcast_Peer(uint8_t channel, wifi_interface_t iface, const uint8_t *lmk)
    : ESP_NOW_Peer(ESP_NOW.BROADCAST_ADDR, channel, iface, lmk) {
}

ESP_NOW_Broadcast_Peer::~ESP_NOW_Broadcast_Peer() {
    remove();
}

bool ESP_NOW_Broadcast_Peer::begin() {
    if (!ESP_NOW.begin() || !add()) {
        return false;
    }
    return true;
}

bool ESP_NOW_Broadcast_Peer::send_message(const uint8_t *data, size_t len) {
    return send(data, len);
}

// --- ESPNowBroadcast Namespace Implementation ---

namespace ESPNowBroadcast {

    bool init(ESP_NOW_Broadcast_Peer &broadcast_peer) {
        DEBUG_PRINTLN("Initializing WiFi (AP+STA) and ESP-NOW...");

        // AP+STA mode: ESP-NOW uses STA interface, web dashboard uses AP interface
        WiFi.mode(WIFI_AP_STA);

        // Start Access Point on the same channel as ESP-NOW
        if (!WiFi.softAP(WebServerConfig::AP_SSID, WebServerConfig::AP_PASSWORD,
                         GeneralConfig::ESPNOW_WIFI_CHANNEL)) {
            DEBUG_PRINTLN("Soft AP creation failed!");
            return false;
        }
        DEBUG_PRINTF("AP started - SSID: %s  IP: %s\n",
                     WebServerConfig::AP_SSID,
                     WiFi.softAPIP().toString().c_str());

        // Explicitly start STA interface (required for ESP-NOW in AP+STA mode)
        WiFi.STA.begin();

        // Wait for STA to be ready with timeout to avoid watchdog reset
        unsigned long staTimeout = millis() + 5000;
        while (!WiFi.STA.started()) {
            if (millis() > staTimeout) {
                DEBUG_PRINTLN("WARNING: WiFi STA start timeout - proceeding anyway");
                break;
            }
            delay(100);
        }

        // Initialize broadcast peer
        if (!broadcast_peer.begin()) {
            DEBUG_PRINTLN("ESP-NOW Init Failed!");
            return false;
        }

        DEBUG_PRINTLN("ESP-NOW initialized successfully.");
        return true;
    }

    bool broadcastTelemetry(ESP_NOW_Broadcast_Peer &broadcast_peer, const TelemetryData &data) {
        bool success = broadcast_peer.send_message((uint8_t *)&data, sizeof(data));
        DEBUG_PRINTF("ESP-NOW Broadcast: %s\n", success ? "Success" : "Failed");
        return success;
    }

}
