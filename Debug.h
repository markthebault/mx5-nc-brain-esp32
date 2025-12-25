#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

/**
 * @file Debug.h
 * @brief Debug output macros for conditional compilation
 *
 * Set DEBUG to 1 to enable debug output, 0 to disable.
 * When disabled, all debug macros compile to nothing.
 */

// --- Debug Configuration ---
#define DEBUG 1  // Set to 1 to enable debug output, 0 to disable

#if DEBUG
    #define DEBUG_BEGIN(baud) Serial.begin(baud)
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
    #define DEBUG_BEGIN(baud)
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTF(...)
#endif

#endif // DEBUG_H
