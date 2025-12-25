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
        DEBUG_PRINTLN("Initializing ESP-NOW...");

        // Configure WiFi in Station mode
        WiFi.mode(WIFI_STA);
        WiFi.setChannel(ESPNOW_WIFI_CHANNEL);

        // Wait for WiFi to start
        while (!WiFi.STA.started()) {
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
