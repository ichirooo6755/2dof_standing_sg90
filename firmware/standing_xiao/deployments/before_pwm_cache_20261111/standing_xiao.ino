// Port of ggoonnzzaallo/2dof_standing controller 0.9, commit 3a9cb54.
// XIAO nRF54L15 Sense + SG90s, D0 lower / D1 upper. See README.md.
#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include "Lsm6ds3.h"
#include "GravityFilter.h"
#include "MicrosecondClock.h"
#include "MountCalibration.h"
#include "FineServo.h"
#include "ServoProfile.h"
#include "InactivityGuard.h"
#include "RecoveryStrategy.h"
#include "FallRecoveryGate.h"
#include "PolicySelection.h"
static_assert(!POLICY_RETRIES || POLICY_ACTION_INCREMENT_RAD==RecoveryStrategy::increment,"Retry integration rate mismatch");
static_assert(CONTROL_LIMIT_RAD==RecoveryStrategy::limit,"Control and recovery range differ");
#ifndef ARDUINO_ARCH_NRF54L15CLEAN
#error "Use XIAO nRF54L15 / Sense, lolren core 1.0.23."
#endif

enum Mode { OFF=0, HOLD=1, RUN=2, SHADOW=3, STANDALONE=4, CHARGE=5, INACTIVE=6 };
Mode mode=OFF;
InactivityGuard inactivity;
RecoveryStrategy recovery;
FallRecoveryGate fallGate;
bool actorWasActive=false;
bool supervisorOverride=false;
const char* INACTIVE_REASON="IMU inactive for 60 s; reboot required. PWM off; charging available.";
const char* lastStopReason="Boot disarmed.";
Lsm6ds3 imu;
GravityFilter attitude;
MicrosecondClock clockUs;
FineServo motors[2];
float accelBase[3]={},gyroBase[3]={},observation[10]={},action[2]={},target[2]={},previous[2]={};
uint16_t pulses[2]={},batteryMv=0;
uint32_t sampleAt=0,lastPoll=0,batteryAt=0,heartbeatAt=0,armedAt=0,runAt=0,nextPolicy=0;
uint32_t frames=0,inferenceUs=0,lateUs=0,dropped=0,filterRejects=0;
uint8_t lowCount=0;
bool hadSerial=false,discard=false;
bool bootPending=false,bootNeutral=false;
uint32_t bootAt=0;
char command[24];size_t used=0;
const char* HEADER="frame,t_us,mode,obs0_u6,obs1_u6,obs2_u6,obs3_u6,obs4_u6,obs5_u6,obs6_u6,obs7_u6,obs8_u6,obs9_u6,action0_u6,action1_u6,target0_urad,target1_urad,pwm0_us,pwm1_us,battery_mv,inference_us,late_us,sample_age_us,sample_seq,i2c_errors,dropped,filter_rejects,override,fall_gate,still_ms";

// nRF52840 ADC/charger setup is not valid on this board. No calibrated
// cell reading exists for this wiring: 0 means unavailable, never 0 volts.
uint16_t readBatteryMv(){return 0;}
float clip(float v){return fminf(1.f,fmaxf(-1.f,v));}
void resetPolicy(){
  for(int i=0;i<2;++i){target[i]=previous[i]=action[i]=0;pulses[i]=0;}
  frames=inferenceUs=lateUs=dropped=0;supervisorOverride=false;
  recovery.reset(clockUs.now()^imu.sequence^batteryMv);
  fallGate.reset();actorWasActive=false;
}
void stop(){
  bootPending=bootNeutral=false; // A stop/fault cancels startup; never auto-retry.
  for(int i=0;i<2;++i)if(motors[i].attached())motors[i].detach();
  digitalWrite(PIN_D0,LOW);pinMode(PIN_D0,OUTPUT);digitalWrite(PIN_D1,LOW);pinMode(PIN_D1,OUTPUT);
  mode=inactivity.locked?INACTIVE:OFF;resetPolicy();
  if(inactivity.locked)lastStopReason=INACTIVE_REASON;
}
void halt(const char* reason){stop();if(!inactivity.locked)lastStopReason=reason;if(Serial){Serial.print("OFF: ");Serial.println(lastStopReason);}}
void beginCharge(){
  stop(); // Detach both hardware PWM outputs before acknowledging this mode.
  batteryMv=readBatteryMv();mode=CHARGE;
  lastStopReason=inactivity.locked?INACTIVE_REASON:"Charge mode: PWM off; servo power remains connected.";
  Serial.println("CHARGE: PWM off; charger configuration unchanged. Charging is not measured.");
}
bool fresh(){return imu.ready&&attitude.ready&&sampleAt&&uint32_t(clockUs.now()-sampleAt)<20000;}
bool validStartBattery(){return true;} // Voltage guard awaits nRF54 ADC calibration.
void status(){
  Serial.println("ROBOT POLICY CONTROL 0.9 / SG90-nRF54L15 port 5");
  Serial.print("POLICY_GEOMETRY: ");Serial.println(POLICY_GEOMETRY);
  Serial.print("POLICY_ACTOR_SHA256: ");Serial.println(POLICY_ACTOR_SHA256);
  Serial.print("POLICY_RUNTIME_SHA256: ");Serial.println(POLICY_RUNTIME_SHA256);
  Serial.print("MODE: ");Serial.println(int(mode));
  Serial.print("IMU_READY: ");Serial.println(imu.ready?1:0);
  Serial.print("FILTER_READY: ");Serial.println(attitude.ready?1:0);
  Serial.println("SERVO_PINS: lower=D0, upper=D1; IMU axes=1,2,3.");
  Serial.println("RANGE: +/-90 deg targets, SG90 PWM 500..2500 us, centers=1500,1464; no shaft feedback.");
  Serial.print("PROFILE: 50 Hz, action increment rad: ");Serial.println(POLICY_ACTION_INCREMENT_RAD);
  Serial.print("VARIED_RETRIES: ");Serial.println(POLICY_RETRIES?1:0);
  Serial.println("UPRIGHT: stable level base eases both targets to zero, then holds; tilt/spin returns to recovery.");
  Serial.println("FALL_GATE: >=45 deg, then stationary for 300 ms; bump restarts wait. PWM targets held while waiting.");
  Serial.print("RETRY_COUNT: ");Serial.println(recovery.retries);
  Serial.println("GUARDS: supervised trials need heartbeat; automatic standalone retains sensor/timing/numeric stops.");
  Serial.println("INACTIVITY: stop disabled; continuous operation requested.");
  Serial.print("INACTIVITY_LOCKED: ");Serial.println(inactivity.locked?1:0);
  Serial.println("BOOT: USB and battery auto-start after IMU ready + neutral hold.");
  Serial.println("BATTERY_GUARD: unavailable/uncalibrated; battery_mv=0 means NA.");
  Serial.print("LAST_STOP: ");Serial.println(lastStopReason);
  Serial.println("POWER: off/charge stop PWM only; no nRF52840 charger GPIOs used.");
  Serial.println("Commands: arm, run, standalone, charge (PWM off), shadow (no PWM), keep, off, !, state, sensors, gate, config, status, help");
  Serial.println("READY");
}
void arm(){
  stop();
  if(inactivity.locked){Serial.println("REFUSED: inactivity lock; reboot required before arming.");return;}
  if(!fresh()){Serial.println("REFUSED: IMU/filter is not fresh and ready.");return;}
  if(!validStartBattery()){Serial.println("REFUSED: battery start guard, 3700..4400 mV.");return;}
  for(int i=0;i<2;++i){
    if(motors[i].attach(i==0?PIN_D0:PIN_D1,500,2500)==INVALID_SERVO){halt("PWM attach failed.");return;}
    if(!controlPulse(0.f,pulses[i],uint8_t(i))){halt("invalid neutral calibration.");return;}
    motors[i].writeMicroseconds(pulses[i]);
  }
  mode=HOLD;armedAt=clockUs.now();heartbeatAt=millis();lowCount=0;
  lastStopReason="NONE";
  Serial.println("ARMED: calibrated centers 1500,1464 us. Wait one second, then run.");
}
void beginControl(bool shadow,bool standalone=false){
  if(inactivity.locked){stop();Serial.println("REFUSED: inactivity lock; reboot required before control.");return;}
  if(!shadow && (mode!=HOLD||uint32_t(clockUs.now()-armedAt)<1000000)){
    halt("run requires arm and one-second neutral hold.");return;
  }
  if(!fresh()){halt("IMU/filter stale or invalid.");return;}
  if(!shadow&&!validStartBattery()){halt("battery start guard.");return;}
  if(shadow)stop();
  // Print before starting policy deadlines: no blocking print during live control.
  Serial.println(HEADER);
  if(shadow){mode=SHADOW;}else{mode=standalone?STANDALONE:RUN;}
  if(standalone)Serial.println("STANDALONE: running; USB may be unplugged. Rev C: switch OFF after unplugging USB for full OFF.");
  runAt=heartbeatAt=millis();nextPolicy=clockUs.now()+20000;lowCount=0;
  fallGate.reset();actorWasActive=false;
  inactivity.begin(millis(),accelBase);
}
void automaticStartup(){
  if(!bootPending&&!bootNeutral)return;
  if(bootPending){
    if(uint32_t(millis()-bootAt)>=15000){halt("automatic startup: IMU initialization timeout; reboot or explicitly arm.");return;}
    if(uint32_t(millis()-bootAt)<2000||!fresh())return;
    // arm() enforces sensor/attach guards and cancels pending startup.
    arm();
    if(mode==HOLD)bootNeutral=true;
  }else if(bootNeutral&&uint32_t(clockUs.now()-armedAt)>=1000000){
    bootNeutral=false;beginControl(false,true);
  }
}
bool appendInteger(char* row,size_t& n,long value){
  const int count=snprintf(row+n,256-n,",%ld",value);
  if(count<0||size_t(count)>=256-n)return false;
  n+=size_t(count);return true;
}
bool appendUnsigned(char* row,size_t& n,uint32_t value){
  const int count=snprintf(row+n,256-n,",%lu",static_cast<unsigned long>(value));
  if(count<0||size_t(count)>=256-n)return false;
  n+=size_t(count);return true;
}
void report(){
  if(!Serial)return; // Intentional USB absence is not lost telemetry.
  char row[256];row[0]='D';size_t n=1;bool ok=true;
  ok &= appendUnsigned(row,n,uint32_t(frames));ok &= appendUnsigned(row,n,uint32_t(sampleAt));
  ok &= appendUnsigned(row,n,uint32_t(mode));
  for(int i=0;i<10;++i)ok &= appendInteger(row,n,lroundf(observation[i]*1e6f));
  for(int i=0;i<2;++i)ok &= appendInteger(row,n,lroundf(action[i]*1e6f));
  for(int i=0;i<2;++i)ok &= appendInteger(row,n,lroundf(target[i]*1e6f));
  for(int i=0;i<2;++i)ok &= appendInteger(row,n,pulses[i]);
  ok &= appendInteger(row,n,batteryMv);ok &= appendUnsigned(row,n,uint32_t(inferenceUs));
  ok &= appendUnsigned(row,n,uint32_t(lateUs));ok &= appendUnsigned(row,n,uint32_t(clockUs.now()-sampleAt));
  ok &= appendUnsigned(row,n,uint32_t(imu.sequence));ok &= appendUnsigned(row,n,uint32_t(imu.errors));
  ok &= appendUnsigned(row,n,uint32_t(dropped));ok &= appendUnsigned(row,n,uint32_t(filterRejects));
  ok &= appendInteger(row,n,supervisorOverride?1:0);
  ok &= appendInteger(row,n,int(fallGate.phase));
  ok &= appendUnsigned(row,n,fallGate.stillMs(clockUs.now()));
  if(!ok||n+1>=sizeof(row)||Serial.availableForWrite()<int(n+1)){++dropped;return;}
  row[n++]='\n';
  if(Serial.write(reinterpret_cast<const uint8_t*>(row),n)!=n)++dropped;
}
bool infer(){
  for(int i=0;i<3;++i)if(!isfinite(attitude.down[i])||!isfinite(gyroBase[i])){halt("non-finite sensor input.");return false;}
  for(int i=0;i<2;++i)if(!isfinite(target[i])||!isfinite(previous[i])){halt("non-finite policy state.");return false;}
  for(int i=0;i<3;++i){observation[i]=clip(attitude.down[i]);observation[i+3]=clip(gyroBase[i]/10.f);}
  for(int i=0;i<2;++i){observation[i+6]=clip(target[i]/CONTROL_LIMIT_RAD);observation[i+8]=previous[i];}
  for(float value:observation)if(!isfinite(value)){halt("non-finite observation.");return false;}
  const bool actorActive=fallGate.recovering();
  if(actorActive){
    // Start retries from the actual recovery onset, excluding the still wait.
    if(!actorWasActive){
      recovery.reset(clockUs.now()^imu.sequence^batteryMv);
      recovery.observe(accelBase,gyroBase,1.f/208.f);
    }
    uint32_t at=clockUs.now();robot_policy(observation,action);inferenceUs=clockUs.now()-at;
    for(float value:action)if(!isfinite(value)){halt("non-finite actor action.");return false;}
    supervisorOverride=POLICY_RETRIES&&recovery.apply(attitude.down,target,action,gyroBase);
  }else{
    inferenceUs=0;supervisorOverride=true;
    for(int i=0;i<2;++i){
      action[i]=0.f; // Preserve the last target throughout the fallen wait.
      if(fallGate.phase==FallRecoveryGate::WATCH&&-attitude.down[2]>.97814760f)
        action[i]=fmaxf(-.4f,fminf(.4f,-target[i]/POLICY_ACTION_INCREMENT_RAD));
    }
  }
  actorWasActive=actorActive;
  uint16_t proposed[2];
  for(int i=0;i<2;++i){
    if(!isfinite(action[i])){halt("non-finite action.");return false;}
    action[i]=clip(action[i]);
    target[i]=fminf(CONTROL_LIMIT_RAD,fmaxf(-CONTROL_LIMIT_RAD,target[i]+POLICY_ACTION_INCREMENT_RAD*action[i]));
    if(!controlPulse(target[i],proposed[i],uint8_t(i))){halt("invalid angle/pulse mapping.");return false;}
  }
  for(int i=0;i<2;++i){
    previous[i]=action[i];
    if(mode==RUN||mode==STANDALONE){pulses[i]=proposed[i];motors[i].writeMicroseconds(pulses[i]);}
  }
  ++frames;return true;
}
void reportState(){
  if(!Serial)return;
  char row[256];
  int n=snprintf(row,sizeof(row),"STATE,%d,%u,%lu,%d,%lu,%s,source=2dof_standing,port=sg90-nrf54-5\n",int(mode),unsigned(batteryMv),
    static_cast<unsigned long>(frames),fresh()?1:0,static_cast<unsigned long>(imu.errors),lastStopReason);
  if(n>0&&size_t(n)<sizeof(row)&&Serial.availableForWrite()>=n)
    Serial.write(reinterpret_cast<const uint8_t*>(row),size_t(n));
}
void reportSensors(){
  if(!Serial)return;
  char row[256];
  int n=snprintf(row,sizeof(row),"SENSORS,%d,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",fresh()?1:0,
    lroundf(attitude.down[0]*1e6f),lroundf(attitude.down[1]*1e6f),lroundf(attitude.down[2]*1e6f),
    lroundf(gyroBase[0]*1e6f),lroundf(gyroBase[1]*1e6f),lroundf(gyroBase[2]*1e6f),
    lroundf(accelBase[0]*1e6f),lroundf(accelBase[1]*1e6f),lroundf(accelBase[2]*1e6f));
  if(n>0&&size_t(n)<sizeof(row)&&Serial.availableForWrite()>=n)
    Serial.write(reinterpret_cast<const uint8_t*>(row),size_t(n));
}
void reportConfig(){
  const char* row="CONFIG port=sg90-nrf54-5 policy=sg90_1000mah actor=c06e4797 pins=0,1 axes=1,2,3 dir=-1,-1 center_us=1500,1464 range_us=500,2500 fall_deg=45 still_ms=300 gyro_max_mrad_s=200 auto_usb=1 inactivity_stop=0 battery_guard=NA\n";
  size_t n=strlen(row);if(Serial.availableForWrite()>=int(n))Serial.write(reinterpret_cast<const uint8_t*>(row),n);
}
void reportGate(){
  if(!Serial)return;
  char row[160];
  const int n=snprintf(row,sizeof(row),"GATE,phase=%u,still_ms=%lu,starts=%lu,delay_ms=300,fall_deg=45\n",unsigned(fallGate.phase),
    static_cast<unsigned long>(fallGate.stillMs(clockUs.now())),static_cast<unsigned long>(fallGate.starts));
  if(n>0&&size_t(n)<sizeof(row)&&Serial.availableForWrite()>=n)Serial.write(reinterpret_cast<const uint8_t*>(row),size_t(n));
}
bool buttonPressed(){
  static bool stable=false,candidate=false;static uint32_t changed=0;
  bool value=digitalRead(PIN_BUTTON)==LOW;uint32_t now=millis();
  if(value!=candidate){candidate=value;changed=now;}
  if(candidate!=stable&&uint32_t(now-changed)>=40){stable=candidate;return stable;}
  return false;
}
void execute(){
  command[used]=0;
  if(!strcmp(command,"keep")){if(mode!=OFF)heartbeatAt=millis();}
  else if(!strcmp(command,"charge"))beginCharge();
  else if(mode==CHARGE&&(!strcmp(command,"run")||!strcmp(command,"standalone")||!strcmp(command,"shadow"))){
    Serial.println(inactivity.locked?"REFUSED: inactivity lock; reboot required before control.":"REFUSED: charge mode is latched; explicitly arm to leave charge mode.");
  }
  else if(!strcmp(command,"arm"))arm();
  else if(!strcmp(command,"run"))beginControl(false);
  else if(!strcmp(command,"standalone"))beginControl(false,true);
  else if(!strcmp(command,"state"))reportState();
  else if(!strcmp(command,"sensors"))reportSensors();
  else if(!strcmp(command,"status"))reportState();
  else if(!strcmp(command,"config"))reportConfig();
  else if(!strcmp(command,"gate"))reportGate();
  else if(!strcmp(command,"help")){if(mode==OFF||mode==CHARGE)status();else reportState();}
  else if(!strcmp(command,"shadow"))beginControl(true);
  else{stop();if(!inactivity.locked)lastStopReason="Operator stop or command.";Serial.println("OFF. PWM stopped; servo supply may remain powered.");
    if(strcmp(command,"off"))status();}
}
void setup(){
  stop();pinMode(PIN_BUTTON,INPUT_PULLUP);pinMode(LED_BUILTIN,OUTPUT);digitalWrite(LED_BUILTIN,HIGH);
  Serial.begin(115200);clockUs.begin();imu.begin();batteryMv=readBatteryMv();
  bootAt=millis();bootPending=true;
}
void loop(){
  uint32_t now=clockUs.now();
  if(buttonPressed()){
    if(mode!=OFF||bootPending||bootNeutral)halt("USR stop.");
    else{bootAt=millis()-2000;bootPending=true;}
  }
  if(uint32_t(now-lastPoll)>=750){
    lastPoll=now;
    if(imu.poll()){
      uint32_t stamp=clockUs.now();float dt=sampleAt?float(uint32_t(stamp-sampleAt))*1e-6f:0;
      sampleAt=stamp;imuToBase(imu.sample.accelG,accelBase);gyroToBase(imu.sample.gyroRadS,gyroBase);
      bool gyroClip=false,accelClip=false;
      for(int i=0;i<3;++i){gyroClip|=fabsf(imu.sample.gyroRadS[i])>=39.95f;accelClip|=fabsf(imu.sample.accelG[i])>=7.9788f;}
      if(!attitude.update(accelBase,gyroBase,dt,gyroClip,accelClip))++filterRejects;
      recovery.observe(accelBase,gyroBase,dt);
      if(mode==RUN||mode==STANDALONE||mode==SHADOW)
        fallGate.observe(stamp,attitude.down,accelBase,gyroBase,target);
      if(mode==RUN||mode==STANDALONE||mode==SHADOW)inactivity.observe(millis(),accelBase,gyroBase);
    }
  }
  automaticStartup();
  const bool connected=bool(Serial);
  if(!connected){
    if(mode!=STANDALONE&&mode!=CHARGE&&mode!=INACTIVE&&!bootPending&&!bootNeutral)stop();
    hadSerial=false;used=0;discard=false;
    if(mode!=STANDALONE&&mode!=CHARGE&&mode!=INACTIVE&&!bootPending&&!bootNeutral){delay(1);return;}
  }
  if(connected&&!hadSerial){
    // A USB/DTR reconnect must not reset targets or block an active standalone loop.
    if(mode==OFF||mode==CHARGE)status();
    hadSerial=true;
  }
  for(int n=0;connected&&n<64&&Serial.available();++n){
    char c=Serial.read();
    if(c=='!'){stop();if(!inactivity.locked)lastStopReason="Emergency PWM stop.";used=0;discard=true;Serial.println("OFF. Emergency PWM stop.");}
    else if(c=='\n'||c=='\r'){if(!discard&&used)execute();used=0;discard=false;}
    else if(!discard&&used<sizeof(command)-1)command[used++]=c;
    else if(!discard){stop();used=0;discard=true;Serial.println("OFF: command too long.");}
  }
  if(mode!=OFF&&mode!=CHARGE&&mode!=INACTIVE){
    if(!fresh())halt("stale/invalid IMU/filter.");
    // No inactivity expiration: user requested continuous operation.
    else if(mode!=STANDALONE&&!bootNeutral&&uint32_t(millis()-heartbeatAt)>=3000)halt("host heartbeat timeout.");
    else if((mode==RUN||mode==SHADOW)&&uint32_t(millis()-runAt)>=60000)halt("trial duration limit.");
    else if(mode==HOLD&&uint32_t(clockUs.now()-armedAt)>=10000000)halt("armed hold timeout.");
    else if(mode==RUN||mode==SHADOW||mode==STANDALONE){
      now=clockUs.now();
      if(int32_t(now-nextPolicy)>=0){
        lateUs=now-nextPolicy;
        if(lateUs>=10000)halt("policy deadline over 10 ms late.");
        else{nextPolicy+=20000;if(infer())report();}
      }
    }
  }
  delay(1);
}
