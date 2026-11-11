#pragma once
#include <Arduino.h>
#include <stdint.h>
constexpr uint8_t INVALID_SERVO=255;

// Share the foreground pulse generation used by the commissioning sketch.
// Every pulse finishes before IMU reads or actor inference begin.
class FineServo {
 public:
  bool attached()const{return active_;}
  uint8_t attach(uint8_t pin,int minimum,int maximum){
    if(pin!=PIN_D0&&pin!=PIN_D1)return INVALID_SERVO;
    const uint8_t axis=pin==PIN_D0?0:1;
    if(channels_[axis]&&channels_[axis]!=this)return INVALID_SERVO;
    if(active_)detach();
    pin_=pin;minimum_=minimum;maximum_=maximum;
    digitalWrite(pin_,LOW);pinMode(pin_,OUTPUT);
    channels_[axis]=this;active_=true;return 0;
  }
  void writeMicroseconds(int us){
    if(!active_)return;
    if(us<minimum_)us=minimum_;
    if(us>maximum_)us=maximum_;
    pulseUs_=us;
  }
  void detach(){
    if(!active_)return;
    channels_[pin_==PIN_D0?0:1]=nullptr;
    digitalWrite(pin_,LOW);pinMode(pin_,OUTPUT);active_=false;
    if(!channels_[0]&&!channels_[1])haveFrame_=false;
  }
  uint32_t measuredHighUs()const{return highUs_;}
  static uint32_t periodUs(){return periodUs_;}
  static uint32_t maxPeriodUs(){return maxPeriodUs_;}
  static void service(){
    if(!channels_[0]&&!channels_[1])return;
    const uint32_t now=micros();
    const uint32_t elapsed=uint32_t(now-frameAtUs_);
    if(haveFrame_&&elapsed<20000)return;
    periodUs_=haveFrame_?elapsed:0;
    if(periodUs_>maxPeriodUs_)maxPeriodUs_=periodUs_;
    frameAtUs_=now;haveFrame_=true;
    for(uint8_t axis=0;axis<2;++axis){
      FineServo* servo=channels_[axis];
      if(!servo)continue;
      const uint32_t rise=micros();
      digitalWrite(servo->pin_,HIGH);
      delayMicroseconds(servo->pulseUs_);
      digitalWrite(servo->pin_,LOW);
      servo->highUs_=uint32_t(micros()-rise);
    }
  }
 private:
  inline static FineServo* channels_[2]={nullptr,nullptr};
  inline static uint32_t frameAtUs_=0,periodUs_=0,maxPeriodUs_=0;
  inline static bool haveFrame_=false;
  uint8_t pin_=0;
  bool active_=false;
  int minimum_=500,maximum_=2500,pulseUs_=1500;
  uint32_t highUs_=0;
};
