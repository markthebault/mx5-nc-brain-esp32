#ifndef DASHBOARD_SERVER_H
#define DASHBOARD_SERVER_H

#include <Arduino.h>
#include "TelemetryData.h"

/**
 * @file DashboardServer.h
 * @brief HTTP dashboard server for live telemetry viewing
 *
 * Runs a web server on the ESP32 soft AP. Connect your device to the
 * WiFi AP and open http://192.168.4.1 to view live vehicle telemetry.
 */

namespace DashboardServer {
    /**
     * @brief Initialize the HTTP server
     *
     * Starts the NetworkServer on the configured port.
     * Must be called after ESPNowBroadcast::init() (which starts the AP).
     */
    void init();

    /**
     * @brief Start the server FreeRTOS task
     *
     * Creates a background task that handles incoming HTTP clients
     * without blocking the main loop.
     *
     * @param telemetryData Pointer to shared telemetry data structure
     */
    void startTask(TelemetryData* telemetryData);
}

#endif // DASHBOARD_SERVER_H
