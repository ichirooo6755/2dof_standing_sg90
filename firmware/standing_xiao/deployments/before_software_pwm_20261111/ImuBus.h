#pragma once
#include <Arduino.h>
#include <Wire.h>

// nRF54L15 core owns the correct TWIM instance and its bounded waits.
// Do not use the upstream nRF52840 TWIM1 register driver on this MCU.
class ImuBus {
 public:
  uint32_t lastError=0, hardwareError=0;
  void begin(){Wire1.begin();Wire1.setClock(400000);}
  bool write(uint8_t address,uint8_t reg,uint8_t value){
    Wire1.beginTransmission(address);Wire1.write(reg);Wire1.write(value);
    hardwareError=Wire1.endTransmission();lastError=hardwareError?2:0;
    return lastError==0;
  }
  bool read(uint8_t address,uint8_t reg,uint8_t* out,size_t count){
    if(!count||count>255){lastError=3;return false;}
    Wire1.beginTransmission(address);Wire1.write(reg);
    hardwareError=Wire1.endTransmission(false);
    if(hardwareError){lastError=2;return false;}
    if(Wire1.requestFrom(address,uint8_t(count))!=count){lastError=3;return false;}
    for(size_t i=0;i<count;++i)out[i]=Wire1.read();
    lastError=0;return true;
  }
};
