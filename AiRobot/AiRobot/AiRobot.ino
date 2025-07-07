#include <FlexCAN_T4.h>

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

constexpr float CNT2DEG_OUT = 360.0f / (8192.0f * 72.0f);  // 出力軸: 1 cnt → 0.00061 deg

struct RoboMasFeedback {
  uint16_t angle_raw;  // 0–8191   （エンコーダ生値）
  int16_t speed_raw;   // [rpm]
  int16_t current_mA;  // [mA]
  uint16_t last_raw;   // 前回の生値 (0-8191)
  int32_t pos_cnt;     // 積算カウント [motor側]  ±2 billion 以上余裕
};

volatile RoboMasFeedback m2006[8];  // M2006
volatile RoboMasFeedback fb2[4];    // M3508

void canReceive1(const CAN_message_t &msg) {
  if (msg.id < 0x201 || msg.id > 0x208) return;
  uint8_t idx = msg.id - 0x201;  // 0–7

  uint16_t angle_raw = (msg.buf[0] << 8) | msg.buf[1];
  int16_t speed_raw = (msg.buf[2] << 8) | msg.buf[3];
  uint16_t curr = (msg.buf[4] << 8) | msg.buf[5];

  // --- 多回転位置積算 -----------------------------
  int16_t diff = int16_t(angle_raw) - int16_t(m2006[idx].last_raw);  // −8191…+8191
  if (diff > 4096) diff -= 8192;                                     // wrap-around 補正
  if (diff < -4096) diff += 8192;

  m2006[idx].pos_cnt += diff;       // 積算
  m2006[idx].last_raw = angle_raw;  // 更新
  // -----------------------------------------------

  m2006[idx].angle_raw = angle_raw;
  m2006[idx].speed_raw = speed_raw;
  m2006[idx].current_mA = curr;
}

void canReceive2(const CAN_message_t &msg) {
  if (msg.id < 0x201 || msg.id > 0x208) return;
  uint8_t idx = msg.id - 0x201;  // 0–7

  uint16_t angle = (msg.buf[0] << 8) | msg.buf[1];
  int16_t speed = (msg.buf[2] << 8) | msg.buf[3];
  uint16_t curr = (msg.buf[4] << 8) | msg.buf[5];

  fb2[idx].angle_raw = angle;
  fb2[idx].speed_raw = speed;
  fb2[idx].current_mA = curr;
}

void printCANstatus() {
  for (int idx = 0; idx <= 6; idx++) {
    Serial.printf("{1-%1u,%4u,%4d}",
                  idx + 1,
                  m2006[idx].angle_raw,
                  m2006[idx].speed_raw);
  }
  for (int idx = 0; idx <= 3; idx++) {
    Serial.printf("{2-%1u,%4u,%4d}",
                  idx + 1,
                  fb2[idx].angle_raw,
                  fb2[idx].speed_raw);
  }
  Serial.println();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline float  wrap180(float deg)            { while (deg > 180) deg -= 360; while (deg < -180) deg += 360; return deg; }

void setup() {
  can1.begin();
  can1.setBaudRate(1'000'000);
  can1.enableMBInterrupt(MB1);
  can1.onReceive(MB1, canReceive1);

  can2.begin();
  can2.setBaudRate(1'000'000);
  can2.enableMBInterrupt(MB2);
  can2.onReceive(MB2, canReceive2);

  // led
  pinMode(13, OUTPUT);

  // lcd
  lcd.begin(16, 2);
  lcd.print("hello, world!");

  // usb serial
  Serial.begin(115200);
  Serial.setTimeout(10);
}

void loop() {
  // printCANstatus();

  // serial input
  checkSerial1Input();

  Polar pol = xy2polar(controller_input.L_x, controller_input.L_y);

  // LCD monitor
  // static char buf[17];
  // snprintf(buf, sizeof(buf), "%4d%4d%4d%4d",
  //          controller_input.L_x,
  //          controller_input.L_y,
  //          int(pol.r),
  //          int(pol.theta));

  // lcd.setCursor(0, 1);  lcd.print(buf);


  /* ---- ② 現在の出力角度を取得 ---- */
  float cur_deg;
  noInterrupts();  // 読み取り中に割込みで書き替えられないようガード
  cur_deg = m2006[0].pos_cnt*CNT2DEG_OUT;
  interrupts();


  constexpr float KP = 20.0f;
  constexpr float KI = 1.0f;
  constexpr float KD = 0.05f;
  constexpr int16_t CUR_LIM = 10000;  // 0.01 A/LSB → ≒±80 A (ESC仕様で制限)

  /* ---- ③ 誤差 (±180°) ---- */
  float err_deg = pol.theta - cur_deg;
  if (err_deg > 180) err_deg -= 360;
  if (err_deg < -180) err_deg += 360;


  static float i_err = 0.0f;

  if(abs(err_deg) < 1) i_err = 0;

  if(abs(err_deg) < 100) i_err += err_deg;
  i_err = constrain(i_err, -CUR_LIM / KI, CUR_LIM / KI);

  static float prev_err = 0.0f;

  float d_err = err_deg - prev_err;
  prev_err = err_deg;

  int16_t i_cmd = int16_t(KP * err_deg + KI * i_err);
  i_cmd = constrain(i_cmd, -CUR_LIM, CUR_LIM);

  /* ---- ⑤ CAN フレーム送信 ---- */
  CAN_message_t tx{};
  tx.id = 0x200;
  tx.len = 8;
  tx.buf[0] = i_cmd >> 8;
  tx.buf[1] = i_cmd & 0xFF;   // Motor1
  tx.buf[2] = tx.buf[3] = 0;  // Motor2
  tx.buf[4] = tx.buf[5] = 0;  // Motor3
  tx.buf[6] = tx.buf[7] = 0;  // Motor4
  can1.write(tx);

  static char buf[17];
  snprintf(buf, sizeof(buf), "%8f%8f%",
           cur_deg,
           pol.theta);

  lcd.setCursor(0, 0);  lcd.print(buf);

  lcd.setCursor(0, 1); lcd.print(i_cmd);

  delay(1);
}
