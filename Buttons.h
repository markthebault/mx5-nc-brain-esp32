#ifndef BUTTONS_H
#define BUTTONS_H

#include <Arduino.h>
#include "TelemetryData.h"

/**
 * @file Buttons.h
 * @brief Button input handling for MX5 NC telemetry system
 *
 * This module manages physical button inputs using FreeRTOS tasks
 * for responsive, non-blocking button detection with debouncing.
 */

namespace Buttons {
    /**
     * @brief Initialize button hardware (GPIO pins)
     *
     * Configures all button pins with appropriate pull-up/pull-down settings.
     */
    void init();

    /**
     * @brief Start the button monitoring task
     * @param telemetryData Pointer to the telemetry data structure to modify
     *
     * Creates a FreeRTOS task that continuously monitors button states
     * and updates telemetry data accordingly (e.g., gauge type toggle).
     */
    void startTask(TelemetryData* telemetryData);
}

#endif // BUTTONS_H
