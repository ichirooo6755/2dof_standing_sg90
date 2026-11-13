#pragma once
#include <Arduino.h>
#include <Wire.h>
#ifndef SG90_IMU_I2C_HZ
#define SG90_IMU_I2C_HZ 400000U
#endif

// The selected nRF54 core owns the correct TWIM instance and bounded waits.
// Do not use the upstream nRF52840 TWIM1 register driver on this MCU.
class ImuBus {
 public:
  uint32_t lastError=0, hardwareError=0;
  uint32_t txFailures=0,rxFailures=0,lastHardwareFailure=0;
  void begin(){Wire1.begin();Wire1.setClock(SG90_IMU_I2C_HZ);}
  bool write(uint8_t address,uint8_t reg,uint8_t value){
    Wire1.beginTransmission(address);Wire1.write(reg);Wire1.write(value);
    hardwareError=Wire1.endTransmission();lastError=hardwareError?2:0;
    if(hardwareError){++txFailures;lastHardwareFailure=hardwareError;}
    return lastError==0;
  }
  bool read(uint8_t address,uint8_t reg,uint8_t* out,size_t count){
    if(!count||count>255){lastError=3;return false;}
    Wire1.beginTransmission(address);Wire1.write(reg);
    hardwareError=Wire1.endTransmission(false);
    if(hardwareError){lastError=2;++txFailures;lastHardwareFailure=hardwareError;return false;}
    if(Wire1.requestFrom(address,uint8_t(count))!=count){lastError=3;++rxFailures;return false;}
    for(size_t i=0;i<count;++i)out[i]=Wire1.read();
    lastError=0;return true;
  }
};
