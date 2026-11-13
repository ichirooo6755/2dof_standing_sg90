#pragma once
#include <math.h>
#include <stdint.h>

// Entry gate for the existing recovery actor. No blocking delay, PWM detach,
// sensor reinitialization, or automatic clearing of hardware faults.
struct FallRecoveryGate {
  enum Phase : uint8_t { WATCH=0, WAITING=1, RECOVERING=2 };
  static constexpr uint32_t stillDelayUs=300000;
  static constexpr float fallenUp=.96592583f; // Enter at >=15 degrees.
  static constexpr float cancelUp=.99026807f; // Cancel pending fall below 8 degrees.
  static constexpr float levelUp=.99026807f; // Finish within 8 degrees.
  static constexpr float quietGyro2=.04f; // |gyro| <0.2 rad/s.
  Phase phase=WATCH;
  bool haveAccel=false,stillTracking=false,levelTracking=false;
  uint32_t lastSample=0,stillAt=0,levelAt=0,startedAt=0,starts=0;
  float accelMean[3]={};

  void reset(){*this=FallRecoveryGate();}
  bool recovering()const{return phase==RECOVERING;}
  // Do not move while waiting for the requested continuous still interval.
  bool runActor()const{return phase==RECOVERING;}
  uint32_t stillMs(uint32_t now)const{
    if(phase!=WAITING||!stillTracking)return 0;
    const uint32_t ms=uint32_t(now-stillAt)/1000;
    const uint32_t limit=stillDelayUs/1000;
    return ms>limit?limit:ms;
  }
  void observe(uint32_t now,const float* down,const float* accel,const float* gyro,const float* target){
    for(int i=0;i<3;++i){
      if(!isfinite(down[i])||!isfinite(accel[i])||!isfinite(gyro[i])){
        stillTracking=levelTracking=false;haveAccel=false;return;
      }
    }
    const uint32_t gap=uint32_t(now-lastSample);
    if(haveAccel&&gap>50000){haveAccel=false;stillTracking=levelTracking=false;}
    const float blend=haveAccel?1.f-expf(-float(gap)*1e-6f/.08f):1.f;
    float residual2=0.f,norm2=0.f,speed2=0.f;
    for(int i=0;i<3;++i){
      if(!haveAccel)accelMean[i]=accel[i];
      const float delta=accel[i]-accelMean[i];residual2+=delta*delta;
      accelMean[i]+=blend*delta;norm2+=accel[i]*accel[i];speed2+=gyro[i]*gyro[i];
    }
    haveAccel=true;lastSample=now;
    const float up=-down[2];
    // Reject spin, acceleration far from gravity, and changing acceleration.
    // A renewed bump restarts the full stationary interval.
    const bool quiet=speed2<quietGyro2&&norm2>.81f&&norm2<1.21f&&residual2<.0016f;
    if(phase==WATCH){
      if(up<=fallenUp){phase=WAITING;stillTracking=false;}
      else return;
    }
    if(phase==WAITING){
      if(up>cancelUp){phase=WATCH;stillTracking=false;return;}
      if(!quiet){stillTracking=false;return;}
      if(!stillTracking){stillAt=now;stillTracking=true;}
      if(uint32_t(now-stillAt)>=stillDelayUs){
        phase=RECOVERING;startedAt=now;++starts;stillTracking=levelTracking=false;
      }
      return;
    }
    // Let the existing neutral supervisor finish centering before re-arming
    // the entry gate. These are commanded targets, not encoder readbacks.
    const bool centered=isfinite(target[0])&&isfinite(target[1])&&fabsf(target[0])<.05f&&fabsf(target[1])<.05f;
    if(up>levelUp&&quiet&&centered){
      if(!levelTracking){levelAt=now;levelTracking=true;}
      if(uint32_t(now-levelAt)>=stillDelayUs){phase=WATCH;levelTracking=false;}
    }else levelTracking=false;
  }
};
