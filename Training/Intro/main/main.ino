#include <Arduino.h>
#include "myBLEController.h"

hw_timer_t *timer = NULL;    // タイマーのポインタ

void onTimer();

// グローバルにインスタンスを用意
MyBLEController myBLE;

#define MR_PWM 4
#define MR_DIR 5
#define ML_PWM 6
#define ML_DIR 7

void setup() {
  Serial.begin(115200);
  // BLE初期化
  myBLE.begin("My BLE Controller");  // 広告するデバイス名

  timer = timerBegin(1000);  // タイマーを1kHzで初期化
  timerAttachInterrupt(timer, &onTimer);  // 割り込み関数を設定
  timerAlarm(timer, 50, true, 0);  // 50msごとに割り込みを発生させる

  pinMode(MR_PWM,OUTPUT);
  pinMode(MR_DIR,OUTPUT);
  pinMode(ML_PWM,OUTPUT);
  pinMode(ML_DIR,OUTPUT);
}

static std::array<std::array<int, 2>, 2> current;

int x = 0;
int y = 0;

void IRAM_ATTR onTimer() {
  // 何かキーが受信されていれば確認
  // static String lastVal;
  current = myBLE.getLastReceived();
  Serial.print("d0:");
  Serial.print(current[0][0]);
  Serial.print(current[0][1]);
  Serial.print(",d1:");
  Serial.print(current[1][0]);
  Serial.println(current[1][1]);

  x = (current[0][0] - 512);
  y = (current[0][1] - 512);
}

float ml_power = 0;
float mr_power = 0;

void loop() {
  analogWrite(ML_PWM, 0);
  analogWrite(MR_PWM, 0);

  ml_power = 255*(-x+y)/1024;
  mr_power = 255*(-x-y)/1024;

  analogWrite(ML_PWM, abs(ml_power));
  analogWrite(MR_PWM, abs(mr_power));

  if(ml_power>0) digitalWrite(ML_DIR,1);
  else digitalWrite(ML_DIR,0);
  if(mr_power>0) digitalWrite(MR_DIR,1);
  else digitalWrite(MR_DIR,0);
  
  Serial.print(ml_power);
  Serial.println(mr_power);

  delay(50);
}
