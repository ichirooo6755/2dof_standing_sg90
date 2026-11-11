#pragma once
#include <Arduino.h>
#include <stdint.h>
constexpr uint8_t INVALID_SERVO=255;

// Core TIMER/GPIOTE/DPPI output on D0/D1. Per-pin 50 Hz avoids the shared PWM
// sequence/frequency reconfiguration waits. Edges do not depend on loop timing.
class FineServo {
 public:
  bool attached()const{return active_;}
  uint8_t attach(uint8_t pin,int minimum,int maximum){
    if(pin!=PIN_D0&&pin!=PIN_D1)return INVALID_SERVO;
    if(active_)detach();
    pin_=pin;minimum_=minimum;maximum_=maximum;
    digitalWrite(pin_,LOW);pinMode(pin_,OUTPUT);
    static bool configured=false;
    if(!configured){analogWriteResolution(16);analogWriteFrequency(50);configured=true;}
    analogWritePinFrequency(pin_,50);
    active_=true;return 0;
  }
  void writeMicroseconds(int us){
    if(!active_)return;
    if(us<minimum_)us=minimum_;if(us>maximum_)us=maximum_;
    const int duty=int((uint32_t(us)*65535u+10000u)/20000u);
    analogWrite(pin_,duty);
  }
  void detach(){
    if(!active_)return;
    // Core digitalWrite explicitly disconnects analogWrite PWM from this pin.
    digitalWrite(pin_,LOW);pinMode(pin_,OUTPUT);active_=false;
  }
 private:
  uint8_t pin_=0;bool active_=false;int minimum_=500,maximum_=2500;
};
