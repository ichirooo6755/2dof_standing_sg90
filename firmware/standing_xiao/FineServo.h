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
    // L15 D0/D1 are P1.04/P1.05; LM20A D0/D1 are P1.00/P1.31.
    // Resolve the selected board's actual pad before enabling input readback.
    uint8_t port=0,bit=0;
    if(!pinToPortPin(pin,&port,&bit)||port!=1||bit>=32)return INVALID_SERVO;
    if(active_)detach();
    pin_=pin;minimum_=minimum;maximum_=maximum;
    digitalWrite(pin_,LOW);pinMode(pin_,OUTPUT);
    NRF_P1->PIN_CNF[bit]&=~GPIO_PIN_CNF_INPUT_Msk;
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
  uint16_t emittedPulseUs()const{return emittedUs_;}
  uint32_t padFaults()const{return padFaults_;}
  uint32_t maxHighErrorUs()const{return maxHighErrorUs_;}
  static uint32_t periodUs(){return periodUs_;}
  static uint32_t maxPeriodUs(){return maxPeriodUs_;}
  // Keep the HOLD pulse phase when handing off to scheduled policy frames.
  static uint32_t nextFrameAtUs(){return (haveFrame_?frameAtUs_:uint32_t(micros()))+20000;}
  static void service(bool scheduled=false){
    if(!channels_[0]&&!channels_[1])return;
    const uint32_t now=micros();
    const uint32_t elapsed=uint32_t(now-frameAtUs_);
    if(haveFrame_&&!scheduled&&elapsed<20000)return;
    periodUs_=haveFrame_?elapsed:0;
    if(periodUs_>maxPeriodUs_)maxPeriodUs_=periodUs_;
    frameAtUs_=now;haveFrame_=true;
    for(uint8_t axis=0;axis<2;++axis){
      FineServo* servo=channels_[axis];
      if(!servo)continue;
      servo->emittedUs_=uint16_t(servo->pulseUs_);
      const uint32_t rise=micros();
      digitalWrite(servo->pin_,HIGH);
      const uint32_t highAt=micros();
      bool badPad=false;
      while(uint32_t(micros()-highAt)<servo->emittedUs_){
        if(digitalRead(servo->pin_)!=HIGH)badPad=true;
      }
      digitalWrite(servo->pin_,LOW);
      if(digitalRead(servo->pin_)!=LOW)badPad=true;
      if(badPad)++servo->padFaults_;
      servo->highUs_=uint32_t(micros()-rise);
      const uint32_t error=servo->highUs_>servo->emittedUs_?servo->highUs_-servo->emittedUs_:servo->emittedUs_-servo->highUs_;
      if(error>servo->maxHighErrorUs_)servo->maxHighErrorUs_=error;
    }
  }
 private:
  inline static FineServo* channels_[2]={nullptr,nullptr};
  inline static uint32_t frameAtUs_=0,periodUs_=0,maxPeriodUs_=0;
  inline static bool haveFrame_=false;
  uint8_t pin_=0;
  bool active_=false;
  int minimum_=500,maximum_=2500,pulseUs_=1500;
  uint16_t emittedUs_=1500;
  uint32_t highUs_=0,padFaults_=0,maxHighErrorUs_=0;
};
