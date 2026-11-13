"""Exercise the board-specific servo pad and IMU-to-base mappings on a host."""

from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

ARDUINO = r'''
#pragma once
#include <cstdint>
constexpr uint8_t PIN_D0=0, PIN_D1=1, PIN_D2=2;
constexpr int LOW=0, HIGH=1, OUTPUT=1;
constexpr uint32_t GPIO_PIN_CNF_INPUT_Msk=2;
struct GPIO { uint32_t PIN_CNF[32]; };
inline GPIO gpio{};
inline GPIO* NRF_P1=&gpio;
inline bool levels[3]={};
inline uint32_t tick=0;
inline uint32_t micros(){ return ++tick; }
inline void digitalWrite(uint8_t pin,int value){ levels[pin]=(value==HIGH); }
inline int digitalRead(uint8_t pin){ return levels[pin]?HIGH:LOW; }
inline void pinMode(uint8_t,int){}
inline bool pinToPortPin(uint8_t pin,uint8_t* port,uint8_t* bit){
  *port=1;
#if defined(ARDUINO_NRF54LM20A)
  *bit=pin==PIN_D0?0:(pin==PIN_D1?31:30);
#else
  *bit=pin==PIN_D0?4:5;
#endif
  return pin==PIN_D0||pin==PIN_D1||pin==PIN_D2;
}
'''

TEST = r'''
#include "FineServo.h"
#include "MountCalibration.h"
#include "FallRecoveryGate.h"
#include "StartupWait.h"
#include "ServoProfile.h"
#include <cassert>
#include <cmath>
#include <initializer_list>

int main(){
  // Startup may wait longer than 15 s for a still, fresh sensor, but never
  // arm early; a failed sensor initialization is retried at bounded intervals.
  StartupWait startup;startup.begin(1000);
  assert(!startup.mayArm(2999,true));
  assert(startup.mayArm(3000,true));
  assert(!startup.mayArm(20000,false) && startup.delayed(20000));
  assert(startup.mayArm(20001,true));
  assert(!startup.retryImu(2999,false));
  assert(startup.retryImu(3000,false));
  assert(!startup.retryImu(4999,false));
  assert(!startup.retryImu(5000,true));
  assert(startup.retryImu(5000,false));
  startup.begin(100,true);
  assert(startup.mayArm(100,true));
  assert(!startup.retryImu(2099,false));
  assert(startup.retryImu(2100,false));

  for(auto& p:gpio.PIN_CNF)p=0xff;
  FineServo lower,upper,unused;
  assert(lower.attach(SG90_LOWER_SERVO_PIN,500,2500)!=INVALID_SERVO);
  assert(upper.attach(SG90_UPPER_SERVO_PIN,500,2500)!=INVALID_SERVO);
#if defined(ARDUINO_NRF54LM20A)
  constexpr int p0=0,p1=30;
  assert(SG90_UPPER_SERVO_PIN==PIN_D2);
  assert(unused.attach(PIN_D1,500,2500)==INVALID_SERVO);
  assert(gpio.PIN_CNF[4]==0xff && gpio.PIN_CNF[5]==0xff && gpio.PIN_CNF[31]==0xff);
#else
  constexpr int p0=4,p1=5;
  assert(SG90_UPPER_SERVO_PIN==PIN_D1);
  assert(unused.attach(PIN_D2,500,2500)==INVALID_SERVO);
  assert(gpio.PIN_CNF[0]==0xff && gpio.PIN_CNF[31]==0xff);
#endif
  assert(gpio.PIN_CNF[p0]==(0xff & ~GPIO_PIN_CNF_INPUT_Msk));
  assert(gpio.PIN_CNF[p1]==(0xff & ~GPIO_PIN_CNF_INPUT_Msk));
  lower.writeMicroseconds(1500);upper.writeMicroseconds(1464);
  FineServo::service();
  assert(lower.emittedPulseUs()==1500 && upper.emittedPulseUs()==1464);
  assert(lower.padFaults()==0 && upper.padFaults()==0);
  lower.detach();upper.detach();

  const float sensor[3]={0.25f,-0.5f,1.f};float base[3],gyro[3];
  imuToBase(sensor,base);gyroToBase(sensor,gyro);
#if defined(ARDUINO_NRF54LM20A)
  assert(base[0]==-0.25f && base[1]==0.5f && base[2]==1.f);
#else
  assert(base[0]==0.25f && base[1]==-0.5f && base[2]==1.f);
#endif
  for(int i=0;i<3;++i)assert(gyro[i]==base[i]);

  // A stationary 20-degree tilt in each cardinal direction waits 300 ms.
  const float s=std::sin(20.f*3.14159265358979323846f/180.f);
  const float c=std::cos(20.f*3.14159265358979323846f/180.f);
  const float gyro0[3]={0,0,0},target[2]={0,0};
  for(int axis=0;axis<2;++axis)for(int sign:{-1,1}){
    FallRecoveryGate gate;float down[3]={0,0,-c};down[axis]=sign*s;
    const float accel[3]={-down[0],-down[1],-down[2]};
    gate.observe(1000,down,accel,gyro0,target);
    assert(gate.phase==FallRecoveryGate::WAITING);
    assert(!gate.runActor()); // no recovery commands during the still wait
    gate.observe(300000,down,accel,gyro0,target);
    assert(gate.phase==FallRecoveryGate::WAITING);
    gate.observe(301000,down,accel,gyro0,target);
    assert(gate.phase==FallRecoveryGate::WAITING); // stale sample resets timing
    for(uint32_t us=306000;us<=606000;us+=5000)gate.observe(us,down,accel,gyro0,target);
    assert(gate.recovering());
    assert(gate.runActor());
  }
  FallRecoveryGate level;
  assert(!level.runActor());
  const float upright[3]={0,0,-0.997f},flatAccel[3]={0,0,1.03f};
  level.observe(1000,upright,flatAccel,gyro0,target);
  assert(level.phase==FallRecoveryGate::WATCH && !level.runActor());

  // A bump during the wait restarts the entire 300 ms quiet interval.
  FallRecoveryGate bumped;
  const float fallen[3]={0.f,s,-c},fallenAccel[3]={0.f,-s,c};
  const float moving[3]={0.5f,0.f,0.f};
  for(uint32_t us=1000;us<=201000;us+=5000){
    bumped.observe(us,fallen,fallenAccel,gyro0,target);
    assert(!bumped.runActor());
  }
  bumped.observe(206000,fallen,fallenAccel,moving,target);
  assert(!bumped.runActor());
  for(uint32_t us=211000;us<511000;us+=5000){
    bumped.observe(us,fallen,fallenAccel,gyro0,target);
    assert(!bumped.runActor());
  }
  bumped.observe(511000,fallen,fallenAccel,gyro0,target);
  assert(bumped.runActor());

  // ±90° is ±1000 µs from the lower center. Upper +90 saturates at 500.
  uint16_t pulse=0;
  assert(controlPulse(0.f,pulse,0) && pulse==1500);
  assert(controlPulse(CONTROL_LIMIT_RAD,pulse,0) && pulse==500);
  assert(controlPulse(-CONTROL_LIMIT_RAD,pulse,0) && pulse==2500);
  assert(controlPulse(0.f,pulse,1) && pulse==1464);
  assert(controlPulse(-CONTROL_LIMIT_RAD,pulse,1) && pulse==2464);
  assert(controlPulse(CONTROL_LIMIT_RAD,pulse,1) && pulse==500);
}
'''

with tempfile.TemporaryDirectory() as directory:
    tmp = Path(directory)
    (tmp / "Arduino.h").write_text(ARDUINO)
    (tmp / "test.cpp").write_text(TEST)
    for label, define in (("L15", []), ("LM20A", ["-DARDUINO_NRF54LM20A"])):
        binary = tmp / label
        subprocess.run(
            ["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror", *define,
             "-I", str(tmp), "-I", str(ROOT), str(tmp / "test.cpp"), "-o", str(binary)],
            check=True,
        )
        subprocess.run([str(binary)], check=True)
        print(f"PASS {label}: board pad mapping, PWM hold, IMU axes, startup wait, and 0.3s fall gate")
