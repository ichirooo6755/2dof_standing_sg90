#pragma once
#include <stdint.h>

// Power-on only: wait safely for a fresh attitude and retry an IMU that did
// not initialize. Stops after arming remain latched by the controller.
struct StartupWait {
  static constexpr uint32_t startDelayMs=2000, retryDelayMs=2000, reportAfterMs=15000;
  uint32_t startedAt=0, lastImuAttemptAt=0;

  void begin(uint32_t now,bool skipDelay=false){
    startedAt=skipDelay?now-startDelayMs:now;
    lastImuAttemptAt=now;
  }
  bool retryImu(uint32_t now,bool imuReady){
    if(imuReady||uint32_t(now-lastImuAttemptAt)<retryDelayMs)return false;
    lastImuAttemptAt=now;return true;
  }
  bool mayArm(uint32_t now,bool sensorFresh)const{
    return sensorFresh&&uint32_t(now-startedAt)>=startDelayMs;
  }
  bool delayed(uint32_t now)const{return uint32_t(now-startedAt)>=reportAfterMs;}
};
