#pragma once
#include <math.h>
#include <stdint.h>
// SG90 mapping used on this robot: 1500 us center, positive angle lowers pulse.
// Commanded angles only; three-wire SG90 has no shaft feedback.
constexpr float CONTROL_LIMIT_RAD=1.5707963267948966f;
constexpr float PULSE_US_PER_RAD=-636.6197723675814f;
inline bool controlPulse(float angle,uint16_t& pulse){
  if(!isfinite(angle)||fabsf(angle)>CONTROL_LIMIT_RAD+1e-6f)return false;
  const long value=lroundf(1500.f+PULSE_US_PER_RAD*angle);
  if(value<500||value>2500)return false;
  pulse=uint16_t(value);return true;
}
