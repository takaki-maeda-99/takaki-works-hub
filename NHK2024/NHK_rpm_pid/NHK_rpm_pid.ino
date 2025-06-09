#include <MsTimer2.h>
#include <FlexCAN_T4.h>
#include "PID.h"

// #define PWM_M1 3         // モーター１
// #define DIR_M1 4         // モーター１

#define Kp 15
#define Ki 0.0001
#define Kd 30
#define max_rpm 5000
#define cycletime 30 //ms
#define max_d 500 //ms



#define sinphi 0.707106781
#define cosphi 0.707106781

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;  // can1 port

static CAN_message_t msg;
static CAN_message_t rxmsg;

Pid pid0;
Pid pid1;
Pid pid2;
Pid pid3;

int u[4] = { 0 };
float v[4] = { 0 };
float vx = 0;
float vy = 0;
float vt = 0;
float vx_c = 0;
float vy_c = 0;
float vt_c = 0;
float last_vy = 0;
float last_vx = 0;
float last_vt = 0;


void setup() {
  Serial.begin(115200);
  can1.begin();
  can1.setBaudRate(1000000);

  // pinMode(PWM_M1, OUTPUT);
  // pinMode(DIR_M1, OUTPUT);

  pinMode(13, OUTPUT);
  Serial1.begin(100000, SERIAL_8E1);
  
  digitalWrite(13, HIGH);
  delay(2000);
  digitalWrite(13, LOW);

  pid0.init(Kp, Ki, Kd);  //p,i,d
  pid1.init(Kp, Ki, Kd);  //p,i,d
  pid2.init(Kp, Ki, Kd);  //p,i,d
  pid3.init(Kp, Ki, Kd);  //p,i,d

  msg.id = 0x200;
  msg.len = 8;
  for ( int idx = 0; idx < msg.len; ++idx ) {
    msg.buf[idx] = 0;
  }

  MsTimer2::set(1, timerInt);  // CAN read 用 タイマ
  MsTimer2::start();
}


static unsigned long testch[6];  ///実際にデータを入れる配列

void loop() {
  static int data[18];                                                 //入力の生データ入れる配列
  static int dataNumber = 0;                                           //入力データの数(Serial1.available()の返値),受信バッファの数を見る変数
  static unsigned long lastConnectTime = 0;                            //直前の通信の時間?
  if (Serial1.available() > 0) {                                       //受信バッファが0以上=何か受信している
    for (int dataNum = Serial1.available(); dataNum > 0; dataNum--) {  //受信したバイト数を見る
      if (dataNumber < 0) {
        Serial1.read();
        dataNumber++;
        continue;
      }
      data[dataNumber % 18] = Serial1.read();
      dataNumber++;
      if (dataNumber > 18) {
        dataNumber = 0;
      } else if (dataNumber == 18) {                                               //データが揃ったとき
        testch[0] = (((data[1] & 0x07) << 8) | data[0]);                           //ch0(364～1024～1684)
        testch[1] = (((data[2] & 0x3F) << 5) | (data[1] >> 3));                    //ch1(364～1024～1684)
        testch[2] = (((data[4] & 0x01) << 10) | (data[3] << 2) | (data[2] >> 6));  //ch2(364～1024～1684)
        testch[3] = (((data[5] & 0x0F) << 7) | (data[4] >> 1));                    //ch3(364～1024～1684)
        if (!(364 <= testch[0] && testch[0] <= 1684 && 364 <= testch[1] && testch[1] <= 1684 && 364 <= testch[2] && testch[2] <= 1684 && 364 <= testch[3] && testch[3] <= 1684)) {
          for (int i = 1; i < 18; i++) {
            testch[0] = (((data[(1 + i) % 18] & 0x07) << 8) | data[(0 + i) % 18]);                           //ch0(364～1024～1684)
            testch[1] = (((data[(2 + i) % 18] & 0x3F) << 5) | (data[(1 + i) % 18] >> 3));                    //ch1(364～1024～1684)
            testch[2] = (((data[(4 + i) % 18] & 0x01) << 10) | (data[(3 + i) % 18] << 2) | (data[2] >> 6));  //ch2(364～1024～1684)
            testch[3] = (((data[(5 + i) % 18] & 0x0F) << 7) | (data[(4 + i) % 18] >> 1));                    //ch3(364～1024～1684)
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
    int toggle_L = (data[5]&0xC0)>>6;
    int toggle_R = (data[5]&0x30)>>4;

    Serial.print(toggle_L,",");
    Serial.print(toggle_R,",");  

    digitalWrite(13,HIGH);
  }
  else Serial.println("UART error");


  vx_c = map(testch[2], 364, 1684, -max_rpm, max_rpm);
  vy_c = map(testch[3], 364, 1684, -max_rpm, max_rpm);  
  vt_c = map(testch[0], 364, 1684, -max_rpm, max_rpm);

  //減速
  if(abs(vx_c-last_vx) > max_d) vx_c = last_vx + map(vx_c-last_vx, -max_rpm, max_rpm, -max_d, max_d);
  if(abs(vy_c-last_vy) > max_d) vy_c = last_vy + map(vy_c-last_vy, -max_rpm, max_rpm, -max_d, max_d);
  if(abs(vt_c-last_vt) > max_d) vt_c = last_vt + map(vt_c-last_vt, -max_rpm, max_rpm, -max_d, max_d);
  last_vx = vx_c;
  last_vy = vy_c;
  last_vt = vt_c;

  //出力指令
  vx = vx_c;
  vy = vy_c;
  vt = vt_c;

  Serial.print(vx);
  Serial.print(",");
  Serial.print(vy);
  Serial.print(",");
  Serial.print(vt);
  Serial.println(" ");

  delay(cycletime);
}

int count = 0;

void timerInt() {

  count++;

  int check[4] = {0};

  while ( can1.read(rxmsg) ) {
    if (rxmsg.id == 0x201) {
      pid0.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
      check[0] = 1;
    }
    if (rxmsg.id == 0x202) {
      pid1.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
      check[1] = 1;
    }
    if (rxmsg.id == 0x203) {
      pid2.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
      check[2] = 1;
    }
    if (rxmsg.id == 0x204) {
      pid3.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
      check[3] = 1;
    }
  }
  // if(check[0]*check[1]*check[2]*check[3]){

    if(count > cycletime) count = 0;

    float L = 0.5;

    v[0] = -sinphi * vx + cosphi * vy + L * vt;  //右前
    v[1] = -cosphi * vx - sinphi * vy + L * vt;  //左前
    v[2] = sinphi * vx - cosphi * vy + L * vt;   //左後
    v[3] = cosphi * vx + sinphi * vy + L * vt;   //右後


    u[0] = pid0.pid_out(v[0]);
    u[1] = pid1.pid_out(v[1]);
    u[2] = pid2.pid_out(v[2]);
    u[3] = pid3.pid_out(v[3]);

    // Serial.print(pid0.debug());//現在速度
    // Serial.print(",");
    // Serial.print(pid1.debug());
    // Serial.print(",");
    // Serial.print(pid2.debug());
    // Serial.print(",");
    // Serial.print(pid3.debug());
    // Serial.println(" ");
  // }
  // else{
    Serial.print("CAN error");
    Serial.print(check[0]);
    Serial.print(check[1]);
    Serial.print(check[2]);
    Serial.println(check[3]);
  // }

  for (int i = 0; i < 4; i++) {
  msg.buf[i * 2] = u[i] >> 8;
  msg.buf[i * 2 + 1] = u[i] & 0xFF;
  }
  can1.write(msg); // write to can1
}

