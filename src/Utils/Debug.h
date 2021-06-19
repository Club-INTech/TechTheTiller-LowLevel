#ifndef TECH_THE_TILLER_DEBUG
#define TECH_THE_TILLER_DEBUG

#include <Arduino.h>

#define SERIAL_DEBUG(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)

#endif