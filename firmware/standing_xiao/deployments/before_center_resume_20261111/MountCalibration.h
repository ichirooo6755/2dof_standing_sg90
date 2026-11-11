#pragma once
// This robot's verified mounting, not the upstream unit's calibration/bias.
// Lower +X joint, upper +Y joint. Chip XYZ = base XYZ, checked 2026-11-10.
// Recheck this if the board is rotated when installing the new CAD enclosure.
static inline void imuToBase(const float sensor[3],float base[3]){
  for(int i=0;i<3;++i)base[i]=sensor[i];
}
static inline void gyroToBase(const float sensor[3],float base[3]){imuToBase(sensor,base);}
