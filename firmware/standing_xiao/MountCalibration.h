#pragma once
// L15 mapping was verified on this robot, not copied from the upstream unit.
// With the USB connectors in the same direction, Seeed's LM20A V1.0 design
// rotates the same IMU footprint by 180 degrees relative to L15 Sense V1.2.
// LM20A mapping therefore negates X/Y while retaining Z. Verify the actual
// installed board orientation with a motor-free sensor check before use.
static inline void imuToBase(const float sensor[3],float base[3]){
#if defined(ARDUINO_NRF54LM20A)
  base[0]=-sensor[0];base[1]=-sensor[1];base[2]=sensor[2];
#else
  for(int i=0;i<3;++i)base[i]=sensor[i];
#endif
}
static inline void gyroToBase(const float sensor[3],float base[3]){imuToBase(sensor,base);}
