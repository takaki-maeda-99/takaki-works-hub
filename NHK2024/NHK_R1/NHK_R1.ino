#include <MsTimer2.h>
#include <FlexCAN_T4.h>
#include "rad_pid.h"
#include "PID.h"

// shooter PID Parameter
#define Shoot_Kp 1
#define Shoot_Ki 0
#define Shoot_Kd 0
#define Max_shoot_power 200

// Rad_PID
#define Rad_Kp 30
#define Rad_Ki 0
#define Rad_Kd 1000

// Rad PID Parameter
#define Width 8191
#define Radius_wheel 50  // mm
#define Gear 19

// Rpm_PID
#define Rpm_Kp 10
#define Rpm_Ki 0
#define Rpm_Kd 0

#define Max_speed 4000
#define Max_power 16000
#define Diagonal_distance 660

#define sinphi 0.707106781
#define cosphi 0.707106781

#define PWM_M1 0   // 射出ローラー
#define DIR_M1 1   // 射出ローラー
#define PWM_M2 2   // コンベア
#define DIR_M2 3   // コンベア
#define PWM_M3 4   // アーム
#define DIR_M3 5   // アーム
#define PWM_M4 7   // 左プランター
#define DIR_M4 8   // 左プランター
#define PWM_M5 9   // 右プランター
#define DIR_M5 10  // 右プランター
#define SV_open 20
#define SV_close 21
#define IR_1 16
#define IR_2 17
#define IR_3 18
#define LS 19
#define ENCODER 20

// CAN
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;  // can1 port
static CAN_message_t msg;
static CAN_message_t rxmsg;

typedef struct
{
  int16_t angle;
  int16_t rotation;
  int16_t torque;
  short temp;
} DjiEscDataSt;

DjiEscDataSt EscData0;  // 受信用配列
DjiEscDataSt EscData1;  // 受信用配列
DjiEscDataSt EscData2;  // 受信用配列
DjiEscDataSt EscData3;  // 受信用配列

// PID
RadPid rad_pid0;
RadPid rad_pid1;
RadPid rad_pid2;
RadPid rad_pid3;
Pid pid0;
Pid pid1;
Pid pid2;
Pid pid3;
Pid Pid_shoot;

// global_variables
int u[4] = { 0 };
int v[4] = { 0 };
int w[4] = { 0 };

float controller_R_x = 0;
float controller_R_y = 0;
float controller_L_x = 0;
float controller_L_y = 0;

int target_shoot_rpm = 0;

int toggle_L = 3;
int toggle_R = 3;

bool manual_trig = true;

int done = 0;

float phi = 0;

void setup() {
  Serial.begin(115200);
  Serial3.begin(100000, SERIAL_8E1);  // UART

  can1.begin();
  can1.setBaudRate(1000000);

  rad_pid0.init(Rad_Kp, Rad_Ki, Rad_Kd, Width, Radius_wheel, Gear);  // p,i,d
  rad_pid1.init(Rad_Kp, Rad_Ki, Rad_Kd, Width, Radius_wheel, Gear);  // p,i,d
  rad_pid2.init(Rad_Kp, Rad_Ki, Rad_Kd, Width, Radius_wheel, Gear);  // p,i,d
  rad_pid3.init(Rad_Kp, Rad_Ki, Rad_Kd, Width, Radius_wheel, Gear);  // p,i,d

  pid0.init(Rpm_Kp, Rpm_Ki, Rpm_Kd);  // p,i,d
  pid1.init(Rpm_Kp, Rpm_Ki, Rpm_Kd);  // p,i,d
  pid2.init(Rpm_Kp, Rpm_Ki, Rpm_Kd);  // p,i,d
  pid3.init(Rpm_Kp, Rpm_Ki, Rpm_Kd);  // p,i,d

  Pid_shoot.init(Shoot_Kp, Shoot_Ki, Shoot_Kd);  // p,i,d

  msg.id = 0x200;
  msg.len = 8;
  for (int idx = 0; idx < msg.len; ++idx) {
    msg.buf[idx] = 0;
  }

  pinMode(13, OUTPUT);
  pinMode(PWM_M1, OUTPUT);
  pinMode(DIR_M1, OUTPUT);
  pinMode(PWM_M2, OUTPUT);
  pinMode(DIR_M2, OUTPUT);
  pinMode(PWM_M3, OUTPUT);
  pinMode(DIR_M3, OUTPUT);
  pinMode(PWM_M4, OUTPUT);
  pinMode(DIR_M4, OUTPUT);
  pinMode(PWM_M5, OUTPUT);
  pinMode(DIR_M5, OUTPUT);
  pinMode(SV_open, OUTPUT);
  pinMode(SV_close, OUTPUT);
  pinMode(IR_1, INPUT);
  pinMode(IR_2, INPUT);
  pinMode(IR_3, INPUT);
  pinMode(LS, INPUT_PULLUP);
  pinMode(ENCODER, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER), pulse_counter, CHANGE);

  MsTimer2::set(1, timerInt);  // CAN read 用 タイマ
  MsTimer2::start();

  Serial.print("START");
}



void loop() {
  set_zero();

  if (toggle_R == 1 || toggle_R == 2)
    manual_trig = false;

  if (toggle_R == 1) {
      digitalWrite(DIR_M1, LOW);
      analogWrite(PWM_M1, 20);
    if (toggle_L == 1) {
      if (done == 0) {
        digitalWrite(DIR_M2, HIGH);
        analogWrite(PWM_M2, 250);
        digitalWrite(DIR_M3, HIGH);
        analogWrite(PWM_M3, 50);
        delay(500);
        digitalWrite(DIR_M3, LOW);
        analogWrite(PWM_M3, 100);
        delay(1000);
        done = 1;
      }
    } else
      manual_trig = true;
    if (toggle_L == 2) {
  
      manual_trig = false;
      if (digitalRead(LS) == HIGH) {
        digitalWrite(DIR_M2, HIGH);
        analogWrite(PWM_M2, 250);
        digitalWrite(DIR_M3, HIGH);
        analogWrite(PWM_M3, 15);
        if (digitalRead(IR_2) == LOW)
          order_v_movement(500, 0, 0);
        if (digitalRead(IR_3) == LOW)
          order_v_movement(-500, 0, 0);
      } else
        order_v_movement(0, 0, 0);
    }

    // if (controller_R_y >= 0.5)
    //   rolling(0, 2000, -90, 2000);
    // if (controller_R_y <= -0.5)
    //   rolling(0, 2000, 90, 2000);
  }
  if (toggle_R == 2) {  // collect
    manual_trig = false;
    int roll_trig = abs(int(phi / PI) % 2);
    if (controller_R_x >= 0.5 && done == 0) rolling(0, 2000, 180, 2000);
    if (controller_R_x <= -0.5 && done == 0) rolling(0, -2000, -180, 2000);
    if (((controller_L_y >= 0.5 && roll_trig == 0) || (controller_R_y >= 0.5 && roll_trig == 1)) && done == 0) collect(0);
    if (((controller_L_y <= -0.5 && roll_trig == 0) || (controller_R_y <= -0.5 && roll_trig == 1)) && done == 0) plant(0);
    if (((controller_L_y >= 0.5 && roll_trig == 1) || (controller_R_y >= 0.5 && roll_trig == 0)) && done == 0) collect(1);
    if (((controller_L_y <= -0.5 && roll_trig == 1) || (controller_R_y <= -0.5 && roll_trig == 0)) && done == 0) plant(1);
  }
  if (toggle_R == 3) {
    // manual
    if (toggle_L == 1)
      digitalWrite(SV_open, HIGH);
    if (toggle_L == 2) {
      digitalWrite(SV_close, HIGH);
      // digitalWrite(DIR_M3, LOW);
      // analogWrite(PWM_M3, 50);
    }
  }

  delay(30);
}

void timerInt() {
  if (manual_trig) {
    order_v_movement(
      controller_L_x * Max_speed,
      controller_L_y * Max_speed,
      controller_R_x * Max_speed);
    rad_pid0.reset();
    rad_pid1.reset();
    rad_pid2.reset();
    rad_pid3.reset();
    done = 0;
  }

  // 演算処理
  if (CAN_Read()) {
    // ジョイスティック操作
    pid_run();
  }

  if (UART_Read() != 1) {
    Serial.print("UARTerror");
    u[0] = 0;
    u[1] = 0;
    u[2] = 0;
    u[3] = 0;
  }

  CAN_Write();

  // Serial.print(digitalRead(IR_1));
  // Serial.print(digitalRead(IR_2));
  // Serial.print(digitalRead(IR_3));

  // if (toggle_R == 1) {
  //   pid_shoot();
  // }

  Serial.println("");
}

#define collect_height 110
#define collect_count 4

#define collect_speed 550
#define move_speed 800

#define rack_space 230
#define plant_space 510

int dir, pwm;
int hi, lo;

void collect(int LR) {

  if (LR == 0) {
    dir = DIR_M4;
    pwm = PWM_M4;
    hi = 1;
    lo = 0;
  }
  if (LR == 1) {
    dir = DIR_M5;
    pwm = PWM_M5;
    hi = 0;
    lo = 1;
  }
  for (int n = 0; n < 6; n++) {
    digitalWrite(dir, hi);
    analogWrite(pwm, 250);
    move_v(0, collect_height, collect_speed);
    analogWrite(pwm, 0);
    if (n == collect_count - 1) {
      move_v(0, -100, move_speed);
      break;
    }
    move_v(0, rack_space - collect_height, move_speed);
  }
  done = 1;
}

void plant(int LR) {
  if (LR == 0) {
    dir = DIR_M4;
    pwm = PWM_M4;
    hi = 1;
    lo = 0;
  }
  if (LR == 1) {
    dir = DIR_M5;
    pwm = PWM_M5;
    hi = 0;
    lo = 1;
  }
  digitalWrite(dir, lo);
  analogWrite(pwm, 250);
  for (int n = 0; n < 6; n++) {
    analogWrite(pwm, 250);
    move_v(0, -collect_height, collect_speed);
    analogWrite(pwm, 0);
    if (n == collect_count - 1) {
      move_v(0, -100, move_speed);
      break;
    }
    move_v(0, -(plant_space - collect_height), move_speed);
  }
  done = 1;
}



/////////////////////////////////////////////////////////common////////////////////////////////////////////////////////////////////////////

void move_v(int x, int y, int rpm) {
  manual_trig = false;
  X_Write(0, 0, 0);
  while (toggle_R != 3) {
    int dx = (-rad_pid0.debug() * sinphi - rad_pid1.debug() * sinphi + rad_pid2.debug() * sinphi + rad_pid3.debug() * sinphi) / 2;
    int dy = (+rad_pid0.debug() * sinphi - rad_pid1.debug() * sinphi - rad_pid2.debug() * sinphi + rad_pid3.debug() * sinphi) / 2;
    if (abs(x - dx) > 3 && abs(y - dy) > 3)
      order_v_movement(get_sign(x - dx) * rpm, get_sign(y - dy) * rpm, 0);
    if (abs(x - dx) > 3)
      order_v_movement(get_sign(x - dx) * rpm, 0, 0);
    else if (abs(y - dy) > 3)
      order_v_movement(0, get_sign(y - dy) * rpm, 0);
    else
      break;
    delay(10);
  }
  X_Write(0, 0, 0);
}


bool X_Write(int x, int y, int t) {
  manual_trig = false;
  float t_elem = PI * Diagonal_distance * t / 360;
  v[0] = -sinphi * x + cosphi * y + t_elem;  // 右前
  v[1] = -cosphi * x - sinphi * y + t_elem;  // 左前
  v[2] = sinphi * x - cosphi * y + t_elem;   // 左後
  v[3] = cosphi * x + sinphi * y + t_elem;   // 右後

  w[0] = 0;
  w[1] = 0;
  w[2] = 0;
  w[3] = 0;

  if (abs(v[0]) + abs(v[1]) + abs(v[2]) + abs(v[3]) == 0) {
    rad_pid0.reset();
    rad_pid1.reset();
    rad_pid2.reset();
    rad_pid3.reset();
  }

  int d = abs(v[0] - rad_pid0.debug()) + abs(v[1] - rad_pid1.debug()) + abs(v[2] - rad_pid2.debug()) + abs(v[3] - rad_pid3.debug());
  return d >= 10;
}


void rolling(int target_vx, int target_vy, float target_theta, int target_rpm) {
  target_theta = PI * target_theta / 180;
  while (toggle_R != 3) {
    int dt = (rad_pid0.debug() + rad_pid1.debug() + rad_pid2.debug() + rad_pid3.debug()) / 4;
    float theta = (dt / (PI * Diagonal_distance)) * 2 * PI;
    float vx_temp = target_vx * cos(theta) - target_vy * sin(theta);
    float vy_temp = target_vx * sin(theta) + target_vy * cos(theta);
    float vx = vx_temp * cos(phi) - vy_temp * sin(phi);
    float vy = vx_temp * sin(phi) + vy_temp * cos(phi);
    order_v_movement(vx, vy, target_rpm);
    if (abs(target_theta * (Diagonal_distance / 2) - dt) < 3) break;
  }
  X_Write(0,0,0);
  phi += target_theta;
  done = 1;
}


void set_zero() {
  analogWrite(PWM_M1, 0);
  analogWrite(PWM_M2, 0);
  analogWrite(PWM_M3, 0);
  analogWrite(PWM_M4, 0);
  analogWrite(PWM_M5, 0);
  digitalWrite(SV_open, LOW);
  digitalWrite(SV_close, LOW);
  target_shoot_rpm = 0;
  manual_trig = true;
}




int pulse_count = 0;
void pulse_counter() {
  pulse_count++;
}

void pid_shoot() {
  int shoot_rpm = (pulse_count * 60000) / 2024;
  int shoot_power = limit(Pid_shoot.pid_out(target_shoot_rpm), Max_shoot_power);
  get_sign(shoot_power) == -1 ? digitalWrite(DIR_M1, HIGH) : digitalWrite(DIR_M1, LOW);
  analogWrite(PWM_M1, abs(shoot_power));
  Pid_shoot.now_value(shoot_rpm);
  pulse_count = 0;
}

void move_x(int target_x, int target_y, int target_theta, int target_rpm) {
  while (1) {
    int dx = (-rad_pid0.debug() * sinphi - rad_pid1.debug() * sinphi + rad_pid2.debug() * sinphi + rad_pid3.debug() * sinphi) / 2;
    int dy = (+rad_pid0.debug() * sinphi - rad_pid1.debug() * sinphi - rad_pid2.debug() * sinphi + rad_pid3.debug() * sinphi) / 2;
    int dt = (rad_pid0.debug() + rad_pid1.debug() + rad_pid2.debug() + rad_pid3.debug()) / 4;
    order_x_movement(target_x, target_y, target_theta, target_rpm);
    if (abs(target_x - dx) + abs(target_y - dy) + abs(PI * Diagonal_distance * target_theta / 360 - dt) < 3) break;
  }
}

bool pid_x_active = false;
int pid_x_rpm = 0;

void pid_run() {
  conv_omni();
  if (pid_x_active) pid_x();
  pid_v();
}

void order_x_movement(int target_x, int target_y, int target_theta, int target_rpm) {
  target_x = conv_trapezoid(target_x, w[0]);
  target_y = conv_trapezoid(target_y, w[1]);
  w[0] = target_x;
  w[1] = target_y;
  w[2] = PI * Diagonal_distance * target_theta / 360;
  pid_x_active = true;
  pid_x_rpm = target_rpm;
}

void order_v_movement(int target_vx, int target_vy, int target_omega) {
  target_vx = conv_trapezoid(target_vx, w[0]);
  target_vy = conv_trapezoid(target_vy, w[1]);
  w[0] = target_vx;
  w[1] = target_vy;
  w[2] = target_omega;
  pid_x_active = false;
}

void conv_omni() {
  v[0] = -sinphi * w[0] + cosphi * w[1] + w[2];  // 右前
  v[1] = -cosphi * w[0] - sinphi * w[1] + w[2];  // 左前
  v[2] = sinphi * w[0] - cosphi * w[1] + w[2];   // 左後
  v[3] = cosphi * w[0] + sinphi * w[1] + w[2];   // 右後
}

void pid_x() {
  v[0] = limit(rad_pid0.pid_out(v[0]), pid_x_rpm);
  v[1] = limit(rad_pid1.pid_out(v[1]), pid_x_rpm);
  v[2] = limit(rad_pid2.pid_out(v[2]), pid_x_rpm);
  v[3] = limit(rad_pid3.pid_out(v[3]), pid_x_rpm);
}

void pid_v() {
  u[0] = limit(pid0.pid_out(v[0]), Max_power);
  u[1] = limit(pid1.pid_out(v[1]), Max_power);
  u[2] = limit(pid2.pid_out(v[2]), Max_power);
  u[3] = limit(pid3.pid_out(v[3]), Max_power);
}

inline int get_sign(double num) {
  return num >= 0 ? 1 : -1;
}

inline int limit(int input, int max_min) {
  return max(-max_min, min(input, max_min));
}

#define a_max 200
#define conv_th 2000
inline int conv_trapezoid(int target, int now_value) {
  return abs(now_value) < conv_th ? target : now_value + limit(target - now_value, a_max);
}

int count = 0;
int trig = 0;
static unsigned long testch[6];  /// 実際にデータを入れる配列
int UART_Read() {
  static int data[18];  // 入力の生データ入れる配列
  if (count > 15) {
    count = 0;
    static int dataNumber = 0;                                           // 入力データの数(Serial3.available()の返値),受信バッファの数を見る変数
    static unsigned long lastConnectTime = 0;                            // 直前の通信の時間?
    if (Serial3.available() > 0) {                                       // 受信バッファが0以上=何か受信している
      for (int dataNum = Serial3.available(); dataNum > 0; dataNum--) {  // 受信したバイト数を見る
        if (dataNumber < 0) {
          Serial3.read();
          dataNumber++;
          continue;
        }
        data[dataNumber % 18] = Serial3.read();
        dataNumber++;
        if (dataNumber > 18) {
          dataNumber = 0;
        } else if (dataNumber == 18) {                                               // データが揃ったとき
          testch[0] = (((data[1] & 0x07) << 8) | data[0]);                           // ch0(364～1024～1684)
          testch[1] = (((data[2] & 0x3F) << 5) | (data[1] >> 3));                    // ch1(364～1024～1684)
          testch[2] = (((data[4] & 0x01) << 10) | (data[3] << 2) | (data[2] >> 6));  // ch2(364～1024～1684)
          testch[3] = (((data[5] & 0x0F) << 7) | (data[4] >> 1));                    // ch3(364～1024～1684)
          if (!(364 <= testch[0] && testch[0] <= 1684 && 364 <= testch[1] && testch[1] <= 1684 && 364 <= testch[2] && testch[2] <= 1684 && 364 <= testch[3] && testch[3] <= 1684)) {
            for (int i = 1; i < 18; i++) {
              testch[0] = (((data[(1 + i) % 18] & 0x07) << 8) | data[(0 + i) % 18]);                           // ch0(364～1024～1684)
              testch[1] = (((data[(2 + i) % 18] & 0x3F) << 5) | (data[(1 + i) % 18] >> 3));                    // ch1(364～1024～1684)
              testch[2] = (((data[(4 + i) % 18] & 0x01) << 10) | (data[(3 + i) % 18] << 2) | (data[2] >> 6));  // ch2(364～1024～1684)
              testch[3] = (((data[(5 + i) % 18] & 0x0F) << 7) | (data[(4 + i) % 18] >> 1));                    // ch3(364～1024～1684)
              if (364 <= testch[0] && testch[0] <= 1684 && 364 <= testch[1] && testch[1] <= 1684 && 364 <= testch[2] && testch[2] <= 1684 && 364 <= testch[3] && testch[3] <= 1684) {
                dataNumber = -i;
                break;
              }
            }
            if (dataNumber > 18) {
              dataNumber = -1;
            }
          } else {
            dataNumber = 0;
          }
        }
      }

      toggle_L = (data[5] & 0xC0) >> 6;
      toggle_R = (data[5] & 0x30) >> 4;

      controller_R_x = (int(testch[0]) - 1024) / 660.0;
      controller_R_y = (int(testch[1]) - 1024) / 660.0;
      controller_L_x = (int(testch[2]) - 1024) / 660.0;
      controller_L_y = (int(testch[3]) - 1024) / 660.0;

      float x_temp = controller_L_x;
      float y_temp = controller_L_y;

      controller_L_x = x_temp * cos(phi) - y_temp * sin(phi);
      controller_L_y = x_temp * sin(phi) + y_temp * cos(phi);

      if (toggle_R == 2) {
        x_temp = controller_R_x;
        y_temp = controller_R_y;

        // controller_R_x = x_temp * cos(phi) - y_temp * sin(phi);
        controller_R_y = x_temp * sin(phi) + y_temp * cos(phi);
      }

      digitalWrite(13, HIGH);
      trig = 1;
    } else {
      digitalWrite(13, LOW);
      trig = 0;
    }
  } else
    count++;
  return trig;
}

void CAN_Write() {
  for (int i = 0; i < 4; i++) {
    msg.buf[i * 2] = u[i] >> 8;
    msg.buf[i * 2 + 1] = u[i] & 0xFF;
  }
  can1.write(msg);  // write to can1
}

int CAN_Read() {
  int check[4] = { 0 };
  while (can1.read(rxmsg)) {
    switch (rxmsg.id) {
      case 0x201:
        EscData0.angle = rxmsg.buf[0] * 256 + rxmsg.buf[1];
        EscData0.rotation = rxmsg.buf[2] * 256 + rxmsg.buf[3];
        EscData0.torque = rxmsg.buf[4] * 256 + rxmsg.buf[5];
        EscData0.temp = rxmsg.buf[6];
        rad_pid0.now_value(EscData0.angle);
        pid0.now_value(EscData0.rotation);
        check[0] = 1;
        break;
      case 0x202:
        EscData1.angle = rxmsg.buf[0] * 256 + rxmsg.buf[1];
        EscData1.rotation = rxmsg.buf[2] * 256 + rxmsg.buf[3];
        EscData1.torque = rxmsg.buf[4] * 256 + rxmsg.buf[5];
        EscData1.temp = rxmsg.buf[6];
        rad_pid1.now_value(EscData1.angle);
        pid1.now_value(EscData1.rotation);
        check[1] = 1;
        break;
      case 0x203:
        EscData2.angle = rxmsg.buf[0] * 256 + rxmsg.buf[1];
        EscData2.rotation = rxmsg.buf[2] * 256 + rxmsg.buf[3];
        EscData2.torque = rxmsg.buf[4] * 256 + rxmsg.buf[5];
        EscData2.temp = rxmsg.buf[6];
        rad_pid2.now_value(EscData2.angle);
        pid2.now_value(EscData2.rotation);
        check[2] = 1;
        break;
      case 0x204:
        EscData3.angle = rxmsg.buf[0] * 256 + rxmsg.buf[1];
        EscData3.rotation = rxmsg.buf[2] * 256 + rxmsg.buf[3];
        EscData3.torque = rxmsg.buf[4] * 256 + rxmsg.buf[5];
        EscData3.temp = rxmsg.buf[6];
        rad_pid3.now_value(EscData3.angle);
        pid3.now_value(EscData3.rotation);
        check[3] = 1;
        break;
    }
  }
  if (check[0] * check[1] * check[2] * check[3])
    return 1;
  else {
    Serial.print("CANerror:");
    Serial.print(check[0]);
    Serial.print(check[1]);
    Serial.print(check[2]);
    Serial.print(check[3]);
    Serial.print(",");
  }
  return 0;
}
