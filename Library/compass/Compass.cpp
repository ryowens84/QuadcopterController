#include "Compass.h"
#include "math.h"
#include "DCM.h"
#include "main.h"
#define GLOBALOBJECT
//Used to find the compass heading based on the magnetometer readings.
float MAG_Heading;

void Compass_Heading()
{
  float MAG_X;
  float MAG_Y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  // Tilt compensated Magnetic filed X:
  MAG_X = magnetom_x*cos_pitch+magnetom_y*sin_roll*sin_pitch+magnetom_z*cos_roll*sin_pitch;
  // Tilt compensated Magnetic filed Y:
  MAG_Y = magnetom_y*cos_roll-magnetom_z*sin_roll;
  // Magnetic Heading
  MAG_Heading = atan2(-MAG_Y,MAG_X);
}
