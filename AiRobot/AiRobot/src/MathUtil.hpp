#include <math.h>  // hypotf, atan2f

#define RAD2DEG 57.2957795f  // 180.0 / π

struct Polar {
  float r;      // ベクトル長
  float theta;  // 角度 [rad]  [-π, π]
};

inline Polar xy2polar(int x, int y) {
  Polar p;
  p.r = hypotf(x, y);                // √(x² + y²)
  p.theta = atan2f(y, x) * RAD2DEG;  // [-180, 180]
  return p;
}