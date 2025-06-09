#ifndef rad_pid_h
#define rad_pid_h
#include <Arduino.h>

class RadPid{
  public:
    RadPid();
    void init(float p_gain,float i_gain,float d_gain,int value_width,int Radius_wheel,int Gear);
    void now_value(int measure);
    void reset();
    int pid_out(int target);
    int debug();
  private:
    float gain[3];
    int in, pre, dif, v_width, R, G;
    int last_measure;
    int displacement;
};

#endif
