#include <FlexCAN_T4.h>
#include "simplePID.h"
#include "DjiMotor.hpp"
#include <IntervalTimer.h>

///////////////////////////////////////////////////  LCD  ////////////////////////////////////////////////////

#include <LiquidCrystal.h>

const int rs = 16, en = 17, d4 = 15, d5 = 19, d6 = 20, d7 = 21;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

///////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <math.h>  // hypotf, atan2f

#define RAD2DEG 57.2957795f  // 180.0 / π

struct Polar {
  float r;      // ベクトル長
  float theta;  // 角度 [rad]  [-π, π]
};

Polar pol;

inline Polar xy2polar(int x, int y) {
  Polar p;
  p.r = hypotf(x, y);                // √(x² + y²)
  p.theta = atan2f(y, x) * RAD2DEG;  // [-180, 180]
  return p;
}


///////////////////////////////////////// serial /////////////////////////////////////////


struct ControllerInput {
  int L_x, L_y, R_x, R_y;
};
ControllerInput controller_input = { 0, 0, 0, 0 };

void checkSerial1Input() {

  if (Serial.available() > 0) {
    // 改行文字まで文字列として一括で読み込む
    String input_string = Serial.readStringUntil('\n');

    input_string.trim();

    // sscanfを使って、文字列から4つのint値を安全に解析(パース)する
    int parsed_count = sscanf(input_string.c_str(), "%d,%d,%d,%d",
                              &controller_input.L_x,
                              &controller_input.L_y,
                              &controller_input.R_x,
                              &controller_input.R_y);

    // NOTE: 4つの値が正しく読み取れた場合のみ、値を採用する。
    // これにより、不完全なデータによる誤動作を防ぐ。
    if (parsed_count != 4) {
      // 読み取りに失敗した場合、値をリセットする
      controller_input.L_x = 0.0f;
      controller_input.L_y = 0.0f;
      controller_input.R_x = 0.0f;
      controller_input.R_y = 0.0f;
    }
  }
}

//////////////////////////////////////////////////////   CAN   /////////////////////////////////////////////////////////////////////////


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

DjiMotorCan1 dji1(can1);
DjiMotorCan2 dji2(can2);

constexpr float CNT2DEG_OUT = 360.0f / (8192.0f * 72.0f);  // 出力軸: 1 cnt → 0.00061 deg


void printCANstatus() {
  for (int idx = 0; idx <= 6; idx++) {
    const auto fb = dji1.feedback(idx+1);
    Serial.printf("{1-%1u,%4u,%4d}",
                  idx + 1,
                  fb.angleRaw,
                  fb.speedRaw);
  }
  for (int idx = 0; idx <= 3; idx++) {
    const auto fb = dji2.feedback(idx+1);
    Serial.printf("{2-%1u,%4u,%4d}",
                  idx + 1,
                  fb.angleRaw,
                  fb.speedRaw);
  }
  Serial.println();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline float  wrap180(float deg) { while (deg > 180) deg -= 360; while (deg < -180) deg += 360; return deg; }

// PID
//SimplePID(kp, ki, kd, outMin, outMax, sampleMs)

#define Kp_a 500
#define Ki_a 0
#define Kd_a 0
#define outMin_a -13000
#define outMax_a 13000
#define sampleMs_a 10

#define Kp_s 25
#define Ki_s 0
#define Kd_s 0
#define outMin_s -10000
#define outMax_s 10000
#define sampleMs_s 1

SimplePID pidAngle11(Kp_a,Ki_a,Kd_a,outMin_a,outMax_a,sampleMs_a);
SimplePID pidSpeed11(Kp_s,Ki_s,Kd_s,outMin_s,outMax_s,sampleMs_s);


IntervalTimer m2006Timer;

int target = 0;

void m2006ISR(){
  const auto &fb = dji1.feedback(1);
  int cmd1 = pidAngle11.compute(wrap180(fb.positionCnt*CNT2DEG_OUT), pol.theta);
  int cmd = pidSpeed11.compute(fb.speedRaw, cmd1);
  dji1.sendCurrent(1, cmd);
  dji1.flush();
}

void setup() {
  // led
  pinMode(13, OUTPUT);

  // lcd
  lcd.begin(16, 2);
  lcd.print("hello, world!");

  // usb serial
  Serial.begin(115200);
  Serial.setTimeout(10);

  m2006Timer.begin(m2006ISR, 1000);          // 1000 µs = 1 kHz
  m2006Timer.priority(128);                  // (0=最高, 255=最低) デフォ 128

}

void loop() {
  // printCANstatus();

  // serial input
  checkSerial1Input();

  pol = xy2polar(controller_input.L_x, controller_input.L_y);

  const auto &fb = dji1.feedback(1);
  static char buf[17];
  snprintf(buf, sizeof(buf), "%8d%8d",
  //          controller_input.L_x,
  //          controller_input.L_y,
              fb.positionCnt,
              int(wrap180(fb.positionCnt*CNT2DEG_OUT)));
  lcd.setCursor(0, 0); lcd.print(buf);

  // LCD monitor
  // static char buf[17];
  // snprintf(buf, sizeof(buf), "%4d%4d%4d%4d",
  //          controller_input.L_x,
  //          controller_input.L_y,
  //          int(pol.r),
  //          int(pol.theta));

  // lcd.setCursor(0, 1);  lcd.print(buf);


  // lcd.setCursor(0, 0); lcd.print(buf);


  delay(1);
}
