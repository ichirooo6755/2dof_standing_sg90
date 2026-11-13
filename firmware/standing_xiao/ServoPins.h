#pragma once
#include <Arduino.h>

// The LM20A robot's upper signal was moved from D1 to D2 on 2026-11-13.
// Keep the earlier L15 robot's verified D0/D1 wiring.
#define SG90_LOWER_SERVO_PIN PIN_D0
#if defined(ARDUINO_NRF54LM20A)
#define SG90_UPPER_SERVO_PIN PIN_D2
#define SG90_SERVO_PINS_CSV "0,2"
#define SG90_UPPER_SERVO_NAME "D2"
#else
#define SG90_UPPER_SERVO_PIN PIN_D1
#define SG90_SERVO_PINS_CSV "0,1"
#define SG90_UPPER_SERVO_NAME "D1"
#endif
