#include "Buttons.h"
#include "Config.h"
#include "Debug.h"

namespace Buttons {
    // Internal task function
    static void buttonTask(void *pvParameters);

    void init() {
        // Configure button pin with internal pull-up resistor
        // Button connects to GND when pressed (active LOW)
        pinMode(GeneralConfig::BUTTON_PIN, INPUT_PULLUP);
        DEBUG_PRINTLN("Button GPIO initialized.");
    }

    void startTask(TelemetryData* telemetryData) {
        xTaskCreate(
            buttonTask,
            "button_task",
            2048,                           // Stack size
            (void*)telemetryData,           // Pass telemetry data pointer
            1,                              // Priority (lower than CAN task)
            NULL
        );
        DEBUG_PRINTF("Button task started on PIN %d.\n", GeneralConfig::BUTTON_PIN);
    }

    static void buttonTask(void *pvParameters) {
        TelemetryData* data = (TelemetryData*)pvParameters;
        unsigned long lastButtonChange = 0;
        bool lastButtonState = HIGH;
        bool buttonPressed = false;

        // Give system time to stabilize
        delay(100);

        for (;;) {
            bool currentButtonState = digitalRead(GeneralConfig::BUTTON_PIN);

            // Detect state change
            if (currentButtonState != lastButtonState) {
                lastButtonChange = millis();
                lastButtonState = currentButtonState;
                DEBUG_PRINTF("Button state changed to: %s\n",
                           currentButtonState == LOW ? "LOW (pressed)" : "HIGH (released)");
            }

            // Check if debounce time has passed
            if ((millis() - lastButtonChange) > GeneralConfig::BUTTON_DEBOUNCE_MS) {
                if (currentButtonState == LOW && !buttonPressed) {
                    // Button is pressed (active LOW due to INPUT_PULLUP)
                    // Toggle gauge type
                    if (data->gaugeType == GAUGE_NORMAL) {
                        data->gaugeType = GAUGE_RACING;
                        DEBUG_PRINTLN("===== GAUGE MODE: RACING =====");
                    } else {
                        data->gaugeType = GAUGE_NORMAL;
                        DEBUG_PRINTLN("===== GAUGE MODE: NORMAL =====");
                    }
                    buttonPressed = true;
                } else if (currentButtonState == HIGH && buttonPressed) {
                    // Button released
                    buttonPressed = false;
                }
            }

            // Check button state frequently (10ms polling rate)
            delay(10);
        }
    }
}
