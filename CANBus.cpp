#include "CANBus.h"
#include "Config.h"
#include "Debug.h"

namespace CANBus {
    // Internal task function
    static void receiveTask(void *pvParameters);

    void init() {
        // Configure TWAI (CAN) controller
        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
            CANConfig::TX_GPIO,
            CANConfig::RX_GPIO,
            TWAI_MODE_NORMAL
        );
        // For testing without acknowledgment:
        // twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CANConfig::TX_GPIO, CANConfig::RX_GPIO, TWAI_MODE_NO_ACK);

        twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
        twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        // Install TWAI driver
        if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
            DEBUG_PRINTLN("TWAI driver installed.");
        } else {
            DEBUG_PRINTLN("Failed to install TWAI driver.");
            while (1) delay(10);  // Halt on failure
        }

        // Start TWAI driver
        if (twai_start() == ESP_OK) {
            DEBUG_PRINTLN("TWAI driver started. Listening for messages...");
        } else {
            DEBUG_PRINTLN("Failed to start TWAI driver.");
            while (1) delay(10);  // Halt on failure
        }
    }

    void startReceiveTask(TelemetryData* telemetryData) {
        xTaskCreate(
            receiveTask,
            "can_receive_task",
            4096,                    // Stack size
            (void*)telemetryData,    // Pass telemetry data pointer
            5,                       // Priority
            NULL
        );
    }

    static void receiveTask(void *pvParameters) {
        TelemetryData* data = (TelemetryData*)pvParameters;
        twai_message_t rx_msg;

        for (;;) {
            // Wait for incoming message (100ms timeout)
            if (twai_receive(&rx_msg, pdMS_TO_TICKS(100)) == ESP_OK) {

                switch (rx_msg.identifier) {

                    case 0x085: { // Braking System (DSC cars)
                        // Brake Pressure (kPa): (3.4518689053 * bytesToInt(raw, 0, 2) - 327.27) / 1000.0
                        uint16_t rawBrake = (rx_msg.data[0] << 8) | rx_msg.data[1];
                        data->brakePressure = (3.4518689053 * rawBrake - 327.27) / 1000.0;

                        // Brake Percentage: min(0.2 * (bytesToInt(raw, 0, 2) - 102), 100)
                        data->brakePercent = (int)max(0.0, min(0.2 * (rawBrake - 102.0), 100.0));

                        DEBUG_PRINTF("CAN 0x085: Brake Press: %.2f kPa | Brake %%: %d\n",
                                     data->brakePressure, data->brakePercent);
                        break;
                    }

                    case 0x201: { // Engine Speed, Vehicle Speed, Accel Position
                        // Engine RPM: Bytes 0 & 1 / 4.0
                        uint16_t rpmRaw = (rx_msg.data[0] << 8) | rx_msg.data[1];
                        data->engineRPM = rpmRaw / 4;

                        // Speed: ((Raw / 100.0) - 100.0). Result is km/h.
                        uint16_t speedRaw = (rx_msg.data[4] << 8) | rx_msg.data[5];
                        data->speed = ((float)speedRaw / 100.0) - 100.0;

                        // Accelerator Position: Byte 6 * 2.0 (percentage)
                        data->accelPos = rx_msg.data[6] * 2.0;

                        DEBUG_PRINTF("CAN 0x201: RPM: %d | Speed: %.2f km/h | Accel: %.1f%%\n",
                                     data->engineRPM, data->speed, data->accelPos);
                        break;
                    }

                    case 0x215: { // Throttle Valve Position
                        // Throttle: Byte 6 * 100 / 255.0
                        data->throttlePos = (rx_msg.data[6] * 100.0) / 255.0;
                        DEBUG_PRINTF("CAN 0x215: Throttle: %.1f%%\n", data->throttlePos);
                        break;
                    }

                    case 0x240: { // Engine Temps
                        // Coolant Temperature: Byte 1 - 40
                        data->waterTemp = (float)(rx_msg.data[1] - 40);
                        DEBUG_PRINTF("CAN 0x240: Coolant: %.1f C\n", data->waterTemp);
                        break;
                    }

                    default:
                        // Ignore unknown CAN IDs
                        break;
                }
            }
        }
    }

    void sendTestMessages() {
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

        DEBUG_PRINTLN("Test CAN messages sent.");
    }

}
