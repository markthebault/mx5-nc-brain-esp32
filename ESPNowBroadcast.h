#ifndef ESPNOW_BROADCAST_H
#define ESPNOW_BROADCAST_H

#include <Arduino.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include "TelemetryData.h"

/**
 * @file ESPNowBroadcast.h
 * @brief ESP-NOW broadcast communication module
 *
 * Handles initialization and broadcasting of telemetry data
 * to all listening ESP-NOW peers on the network.
 */

/**
 * @class ESP_NOW_Broadcast_Peer
 * @brief Custom ESP-NOW peer class for broadcast communication
 *
 * Extends the ESP_NOW_Peer class to simplify broadcast operations.
 */
class ESP_NOW_Broadcast_Peer : public ESP_NOW_Peer {
public:
    /**
     * @brief Constructor
     * @param channel WiFi channel to use
     * @param iface WiFi interface (WIFI_IF_STA or WIFI_IF_AP)
     * @param lmk Local master key (nullptr for no encryption)
     */
    ESP_NOW_Broadcast_Peer(uint8_t channel, wifi_interface_t iface, const uint8_t *lmk);

    /**
     * @brief Destructor - removes peer on destruction
     */
    ~ESP_NOW_Broadcast_Peer();

    /**
     * @brief Initialize ESP-NOW and add broadcast peer
     * @return true if successful, false otherwise
     */
    bool begin();

    /**
     * @brief Send data via ESP-NOW broadcast
     * @param data Pointer to data buffer
     * @param len Length of data in bytes
     * @return true if transmission successful, false otherwise
     */
    bool send_message(const uint8_t *data, size_t len);
};

namespace ESPNowBroadcast {
    /**
     * @brief Initialize WiFi and ESP-NOW for broadcasting
     *
     * Configures WiFi in station mode, sets the channel,
     * and initializes the broadcast peer.
     *
     * @param broadcast_peer Reference to broadcast peer object
     * @return true if successful, false on failure
     */
    bool init(ESP_NOW_Broadcast_Peer &broadcast_peer);

    /**
     * @brief Broadcast telemetry data to all peers
     *
     * @param broadcast_peer Reference to broadcast peer object
     * @param data Telemetry data to broadcast
     * @return true if transmission successful, false otherwise
     */
    bool broadcastTelemetry(ESP_NOW_Broadcast_Peer &broadcast_peer, const TelemetryData &data);
}

#endif // ESPNOW_BROADCAST_H
