#define elevator_top 10    // エレベーター上限用マイクロスイッチピン
#define arm_right 9     // アーム水平用マイクロスイッチピン
#define arm_left 11    // アーム垂直用マイクロスイッチピン
#define close 21
#define open 20

#define TrigPin 17         // トリガーピンをD8に
#define EchoPin 16         // エコーピンをD9に

#define PWM_M1 2         // モーター１
#define DIR_M1 3         // モーター１
#define PWM_M2 7         // モーター2
#define DIR_M2 8         // モーター2

#define height 14        // 接地時の超音波センサ距離しきい値[cm]
#define c_num 100        // 飛び地対策用カウントしきい値

#include <MsTimer2.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;  // can1 port

static CAN_message_t rxmsg;//can受信用buf


float Duration = 0;  // 計測した時間
float Distance = 0;  // 距離

int distance = 0;

int right = 0;
int left = 0;


void setup(){
    pinMode(elevator_top, INPUT);
    pinMode(arm_right, INPUT);
    pinMode(arm_left, INPUT);

    pinMode(EchoPin,INPUT);               // エコーピンを入力に
    pinMode(TrigPin,OUTPUT);              // トリガーピンを出力に

    pinMode(PWM_M1, OUTPUT);
    pinMode(DIR_M1, OUTPUT);
    pinMode(PWM_M2, OUTPUT);
    pinMode(DIR_M2, OUTPUT);

    Serial.begin(110000);
    Serial.println("START");      
    can1.begin();
    can1.setBaudRate(1000000);     // 500kbps data rate
    Serial1.begin(100000, SERIAL_8E1);
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    MsTimer2::set(2, timerInt); // CAN read 用 タイマ
    MsTimer2::start();
}

int i = 0;
int trig = 1;
int count1 = 0;
int count2 = 0;


void loop(){


  if(left == 0 || left == 3);
  else if(left == 1) collect();
  else if(left == 2) construct();
  if(right == 0 || right == 3)motor(2,0);
  else if(right == 1) set_collect_potision();
  else if(right == 2) motor(2,-250);



  // analogWrite(PWM_M1,0);
  // analogWrite(PWM_M2,0);

  // if(distance<14){
  //   i+=1;
  //   if(i > 5){
  //     trig = 0;
  //   }
  // }
  // else{
  //   i = 0;
  //   trig = 1;
  // }

  // if(right == 0 || right == 3) count1 = 0;
  // else if(right == 1){
  //   if(digitalRead(elevator_top)){
  //     digitalWrite(DIR_M1,LOW);
  //     analogWrite(PWM_M1,50);
  //     count1+=1;
  //     if(count1>c_num){
  //       analogWrite(PWM_M1,230);
  //     }
  //   }
  // }
  // else if(right == 2){
  //   if((digitalRead(arm_right)||digitalRead(arm_left))&&trig){
  //     digitalWrite(DIR_M1,HIGH);
  //     analogWrite(PWM_M1,50);      
  //     count1+=1;
  //     if(count1>c_num){
  //       analogWrite(PWM_M1,230);
  //     }
  //   }
  // }

  // if(left == 0 || left == 3) count2 = 0;
  // else if(left == 1){
  //   if(digitalRead(close)||(digitalRead(elevator_top) == 0)){
  //     digitalWrite(DIR_M2,LOW);
  //     analogWrite(PWM_M2,50);
  //     count2+=1;
  //     if(count2>c_num){
  //       analogWrite(PWM_M2,230);
  //     }
  //   }
  // }
  // else if(left == 2){
  //   if(digitalRead(open)||(digitalRead(elevator_top) == 0)){
  //     digitalWrite(DIR_M2,HIGH);
  //     analogWrite(PWM_M2,50);      
  //     count2+=1;
  //     if(count2>c_num){
  //       analogWrite(PWM_M2,230);
  //     }
  //   } 
  // }




  delay(10);
}





void timerInt(){
  distance = distance_echo();
  while ( can1.read(rxmsg) ) {
    if (rxmsg.id == 0x1FF) {
      right = rxmsg.buf[0];
      left = rxmsg.buf[1];

      Serial.print(right);
      Serial.println(left);
    }
  }
}

int distance_echo(){                      //超音波センサの距離取得

  digitalWrite(TrigPin,LOW);              // 計測前に一旦トリガーピンをLowに
  delayMicroseconds(2);

  digitalWrite(TrigPin,HIGH);             // トリガーピンを10μsの時間だけHighに
  delayMicroseconds(10);
  digitalWrite(TrigPin,LOW);

  Duration = pulseIn(EchoPin,HIGH);      // エコーピンからの入力
  
  Duration = Duration / 2;               // 距離を1/2に
  Distance = Duration*340*100/1000000;   // 音速を掛けて単位をcmに

  return Distance;
}


void motor(int m,int pwm){
  if(m==1){
    if(pwm>0)digitalWrite(DIR_M1,LOW);
    else digitalWrite(DIR_M1,HIGH);
    analogWrite(PWM_M1,pwm);
  }
  if(m==2){
    if(pwm>0)digitalWrite(DIR_M2,LOW);
    else digitalWrite(DIR_M2,HIGH);
    analogWrite(PWM_M2,pwm);
  }
}


void collect(){                           //レンガ回収
  while(digitalRead(close)){              //上限まで上昇
      motor(2,250);
      delay(100);
  }
  motor(2,0);
  while(digitalRead(elevator_top)){       //上限まで上昇
      motor(1,250);
      delay(100);
  }
  motor(1,0);
  motor(2,50);
  delay(3000);
  motor(2,0);
}


void construct(){                         //レンガ設置
    int i = 0;
    while(1){                             //接地するまでアームを下げる ⇒このままだと多分早すぎる
      if((distance>15)||(distance==0)){
        i+=1;
        if(i > 10){
          break;
        }
      }
      else i = 0;
      delay(100);
      motor(1,-100);
    }
    motor(1,0);
    motor(2,-50);                      //レンガをはなす　delayは要調整
    delay(1000);
    while(digitalRead(open)){                             //上限まで上昇
        motor(2,-250);
        delay(100);
    }
    motor(2,0);
    while(digitalRead(elevator_top)){                      //上限まで上昇
        motor(1,250);
        delay(100);
    }
    motor(1,0);
}



void set_collect_potision(){               //アームをレンガ回収位置に
    motor(2,-50);
    delay(5000);
    motor(2,0);
    while(digitalRead(arm_right)||digitalRead(arm_left)){                              //下限まで下降
      motor(1,-250);
      while(digitalRead(open)){                              //最大まで開き
        motor(2,-50);
        delay(100);
      }
      delay(100);
      motor(2,0);
    }
    motor(1,0);
}


