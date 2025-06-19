#include "rad_pid.h"
#include <Arduino.h>

#define PI 3.141592653589

RadPid::RadPid() {
  for (int i = 0; i < 3; i++)gain[i] = 0;
  in = 0;
  pre = 0;
  dif = 0;
  last_measure = 0;
  displacement = 0;
}

void RadPid::init(float p_gain, float i_gain, float d_gain, int value_width ,int Radius_wheel,int Gear) {
  gain[0] = p_gain;
  gain[1] = i_gain;
  gain[2] = d_gain;
  v_width = value_width;
  R = Radius_wheel;
  G = Gear; 
}

void RadPid::now_value(int measure) {
  if (measure - last_measure > v_width / 2) displacement += measure - last_measure - v_width;
  else if (measure - last_measure < -1 *v_width / 2) displacement += measure - last_measure + v_width;
  else displacement += measure - last_measure;
  last_measure = measure;
}

int RadPid::pid_out(int target) {
  pre = in;
  in = 2*PI*R*displacement/(v_width*G);
  dif += target - in;
  if ((target - in) > 100) dif = 0;
  return (int)(gain[0] * (target - in) + gain[1] * dif  + gain[2] * (pre - in));
}

int RadPid::debug() {
  return 2*PI*R*displacement/(v_width*G);
}

void RadPid::reset(){
  displacement = 0;
}
