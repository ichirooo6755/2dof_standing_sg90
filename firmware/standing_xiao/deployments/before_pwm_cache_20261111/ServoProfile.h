#pragma once
#include <math.h>
#include <stdint.h>
// Positive angle lowers the pulse. Centers were aligned on the installed horns.
// Commanded angles only; three-wire SG90 has no shaft feedback.
constexpr float CONTROL_LIMIT_RAD=1.5707963267948966f;
constexpr float PULSE_US_PER_RAD=-636.6197723675814f;
constexpr uint16_t SERVO_CENTER_US[2]={1500,1464};
inline bool controlPulse(float angle,uint16_t& pulse,uint8_t axis=0){
  if(axis>=2||!isfinite(angle)||fabsf(angle)>CONTROL_LIMIT_RAD+1e-6f)return false;
  long value=lroundf(float(SERVO_CENTER_US[axis])+PULSE_US_PER_RAD*angle);
  // The upper trim reaches the electrical endpoint before +90 degrees.
  // Saturating here keeps valid policy targets inside the SG90 pulse range.
  if(value<500)value=500;
  if(value>2500)value=2500;
  pulse=uint16_t(value);return true;
}
