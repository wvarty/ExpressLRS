#pragma once

// Macros for logging debug messages to the serial port
// Assumes that the Serial object has been initialised and setup

#ifdef DEBUG
    #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
    #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
    #define DEBUG_PRINTF(format, ...) Serial.printf(format, ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(...)
    #define DEBUG_PRINTLN(...)
    #define DEBUG_PRINTF(format, ...)
#endif
