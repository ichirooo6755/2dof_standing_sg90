#pragma once
#include <Arduino.h>
// Use the installed nRF54L15 core clock; never touch nRF52840 TIMER4 registers.
class MicrosecondClock {
 public:
  void begin(){}
  uint32_t now()const{return uint32_t(micros());}
};
