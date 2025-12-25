#ifndef CANBUS_H
#define CANBUS_H

#include <Arduino.h>
#include "driver/twai.h"
#include "TelemetryData.h"

/**
 * @file CANBus.h
 * @brief CAN Bus interface for Mazda MX5 NC
 *
 * Handles initialization and reception of CAN messages from the vehicle.
 * Decodes standard Mazda CAN IDs for engine RPM, speed, temperatures, etc.
 */

namespace CANBus {
    /**
     * @brief Initialize CAN bus controller
     *
     * Configures TWAI (CAN) peripheral with 500kbps speed and
     * starts the driver. Blocks on failure.
     */
    void init();

    /**
     * @brief Start CAN receive task
     *
     * Creates a FreeRTOS task to continuously monitor and process
     * incoming CAN messages. Updates the provided telemetry data structure.
     *
     * @param telemetryData Pointer to shared telemetry data structure
     */
    void startReceiveTask(TelemetryData* telemetryData);

    /**
     * @brief Send test CAN messages
     *
     * Transmits simulated CAN data for testing without a real vehicle.
     * Requires TWAI_MODE_NO_ACK to be set in init().
     */
    void sendTestMessages();
}

#endif // CANBUS_H
