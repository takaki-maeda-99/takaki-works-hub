#include <MsTimer2.h>
#include <FlexCAN_T4.h>
#include "rad_pid.h"
#include "PID.h"

// float32 center_x;
// float32 center_y;
// float32 radius;

//Rad_PID
#define Rad_Kp 300
#define Rad_Ki 0
#define Rad_Kd 100000

//Rpm_PID
#define Rpm_Kp 30
#define Rpm_Ki 0
#define Rpm_Kd 100

#define max_rpm 4000
#define max_power 3000

#define max_d 1500

//Rad PID Parameter
#define width 8191
#define Radius_wheel 50 //mm
#define Gear 19

#define sinphi 0.707106781
#define cosphi 0.707106781

//CAN
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

DjiEscDataSt EscData0;//受信用配列
DjiEscDataSt EscData1;//受信用配列
DjiEscDataSt EscData2;//受信用配列
DjiEscDataSt EscData3;//受信用配列

//PID
RadPid rad_pid0;
RadPid rad_pid1;
RadPid rad_pid2;
RadPid rad_pid3;
Pid pid0;
Pid pid1;
Pid pid2;
Pid pid3;

//global_variables
int u[4] = { 0 };
int v[4] = { 0 };

float vx = 0;
float vy = 0;
float vt = 0;

float vx_c = 0;
float vy_c = 0;
float vt_c = 0;

float L = 0.5;
int UART_error_trg = 0;
int toggle_L = 3;
int toggle_R = 3;

void setup() {
  Serial.begin(115200);

  Serial1.begin(100000, SERIAL_8E1); //UART

  can1.begin();
  can1.setBaudRate(1000000);

  rad_pid0.init(Rad_Kp, Rad_Ki, Rad_Kd, width, Radius_wheel, Gear);  //p,i,dの順に指定できる
  rad_pid1.init(Rad_Kp, Rad_Ki, Rad_Kd, width, Radius_wheel, Gear);  //p,i,dの順に指定できる
  rad_pid2.init(Rad_Kp, Rad_Ki, Rad_Kd, width, Radius_wheel, Gear);  //p,i,dの順に指定できる
  rad_pid3.init(Rad_Kp, Rad_Ki, Rad_Kd, width, Radius_wheel, Gear);  //p,i,dの順に指定できる

  pid0.init(Rpm_Kp, Rpm_Ki, Rpm_Kd);  //p,i,d
  pid1.init(Rpm_Kp, Rpm_Ki, Rpm_Kd);  //p,i,d
  pid2.init(Rpm_Kp, Rpm_Ki, Rpm_Kd);  //p,i,d
  pid3.init(Rpm_Kp, Rpm_Ki, Rpm_Kd);  //p,i,d

  msg.id = 0x200;
  msg.len = 8;
  for ( int idx = 0; idx < msg.len; ++idx ) {
    msg.buf[idx] = 0;
  }

  MsTimer2::set(1, timerInt);  // CAN read 用 タイマ
  MsTimer2::start();

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
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
    toggle_L = (data[5]&0xC0)>>6;
    toggle_R = (data[5]&0x30)>>4;
    digitalWrite(13,HIGH);
    UART_error_trg = 0;
  }
  else{
    Serial.println("UART error");
    digitalWrite(13,LOW);
    UART_error_trg = 1;
  }

  // Serial.print(rad_pid0.debug());
  // Serial.print(",");
  // Serial.print(rad_pid1.debug());
  // Serial.print(",");
  // Serial.print(rad_pid2.debug());
  // Serial.print(",");
  // Serial.println(rad_pid3.debug());  
  
  delay(30);
}

void timerInt() {
  
  int check[4] = {0}; //完全性チェック用配列

  while (can1.read(rxmsg)) {
    switch (rxmsg.id) {
      case 0x201:
        EscData0.angle    = rxmsg.buf[0] * 256 + rxmsg.buf[1];
        EscData0.rotation = rxmsg.buf[2] * 256 + rxmsg.buf[3];
        EscData0.torque   = rxmsg.buf[4] * 256 + rxmsg.buf[5];
        EscData0.temp     = rxmsg.buf[6] ;
        rad_pid0.now_value(EscData0.angle);
        pid0.now_value(EscData0.rotation);
        check[0] = 1;
        break;
      case 0x202:
        EscData1.angle    = rxmsg.buf[0] * 256 + rxmsg.buf[1];
        EscData1.rotation = rxmsg.buf[2] * 256 + rxmsg.buf[3];
        EscData1.torque   = rxmsg.buf[4] * 256 + rxmsg.buf[5];
        EscData1.temp     = rxmsg.buf[6] ;
        rad_pid1.now_value(EscData1.angle);
        pid1.now_value(EscData1.rotation);
        check[1] = 1;
        break;
      case 0x203:
        EscData2.angle    = rxmsg.buf[0] * 256 + rxmsg.buf[1];
        EscData2.rotation = rxmsg.buf[2] * 256 + rxmsg.buf[3];
        EscData2.torque   = rxmsg.buf[4] * 256 + rxmsg.buf[5];
        EscData2.temp     = rxmsg.buf[6] ;
        rad_pid2.now_value(EscData2.angle);
        pid2.now_value(EscData2.rotation);
        check[2] = 1;
        break;
      case 0x204:
        EscData3.angle    = rxmsg.buf[0] * 256 + rxmsg.buf[1];
        EscData3.rotation = rxmsg.buf[2] * 256 + rxmsg.buf[3];
        EscData3.torque   = rxmsg.buf[4] * 256 + rxmsg.buf[5];
        EscData3.temp     = rxmsg.buf[6] ;
        rad_pid3.now_value(EscData3.angle);
        pid3.now_value(EscData3.rotation);
        check[3] = 1;
        break;
    }
  }
  

  //データがそろってるとき
  if(check[0]*check[1]*check[2]*check[3]){

    if(toggle_L == 3){ 
      rad_pid0.reset();
      rad_pid1.reset();
      rad_pid2.reset();
      rad_pid3.reset();

      vx_c = map(testch[2], 364, 1684, -max_rpm, max_rpm);
      vy_c = map(testch[3], 364, 1684, -max_rpm, max_rpm);  
      vt_c = map(testch[0], 364, 1684, -max_rpm, max_rpm);

      v[0] = -sinphi * vx_c + cosphi * vy_c + L * vt_c;  //右前
      v[1] = -cosphi * vx_c - sinphi * vy_c + L * vt_c;  //左前
      v[2] = sinphi * vx_c - cosphi * vy_c + L * vt_c;   //左後
      v[3] = cosphi * vx_c + sinphi * vy_c + L * vt_c;   //右後

      u[0] = max(-max_power,min(pid0.pid_out(v[0]),max_power));
      u[1] = max(-max_power,min(pid1.pid_out(v[1]),max_power));
      u[2] = max(-max_power,min(pid2.pid_out(v[2]),max_power));
      u[3] = max(-max_power,min(pid3.pid_out(v[3]),max_power));
    }
    else if(toggle_L == 2){
      vx = 500;//center_x;
      vy = 500;//center_y;
      vt = 0;
      
      v[0] = -sinphi * vx + cosphi * vy + L * vt;  //右前
      v[1] = -cosphi * vx - sinphi * vy + L * vt;  //左前
      v[2] = sinphi * vx - cosphi * vy + L * vt;   //左後
      v[3] = cosphi * vx + sinphi * vy + L * vt;   //右後

      u[0] = max(-max_power,min(rad_pid0.pid_out(v[0]),max_power));
      u[1] = max(-max_power,min(rad_pid1.pid_out(v[1]),max_power));
      u[2] = max(-max_power,min(rad_pid2.pid_out(v[2]),max_power));
      u[3] = max(-max_power,min(rad_pid3.pid_out(v[3]),max_power));
    }

  }
  else{
    // Serial.print("CAN error");
    // Serial.print(check[0]);
    // Serial.print(check[1]);
    // Serial.print(check[2]);
    // Serial.println(check[3]);
  }

  if(UART_error_trg == 1){
    u[0] = 0;
    u[1] = 0;
    u[2] = 0;
    u[3] = 0;
  }
  else{
    Serial.print(v[0]);
    Serial.print(",");
    Serial.print(v[1]);
    Serial.print(",");
    Serial.print(v[2]);
    Serial.print(",");
    Serial.println(v[3]);
  }

  for (int i = 0; i < 4; i++) {
    msg.buf[i * 2] = u[i] >> 8;
    msg.buf[i * 2 + 1] = u[i] & 0xFF;
  }

  can1.write(msg); // write to can1
}
