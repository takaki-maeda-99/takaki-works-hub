#include <LiquidCrystal.h>
#include <FlexCAN_T4.h>
#include <MsTimer2.h>


// LiquidCrystalライブラリを初期化します。
// 接続したTeensyのピン番号を順番に指定します。
// LiquidCrystal(RS, E, D4, D5, D6, D7)
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

int counter = 0;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;


void setup() {
  // LCDの列数と行数を設定します (16文字x2行)
  lcd.begin(16, 2);
  Serial1.begin(115200);

  // 起動メッセージを表示
  lcd.print("Teensy 4.0");
  lcd.setCursor(0, 1); // 1列目、2行目にカーソルを移動
  lcd.print("Parallel LCD OK!");

  can1.begin();
  can1.setBaudRate(1000000);

  MsTimer2::set(1, timerCallback);
  MsTimer2::start();

  delay(2000);
  lcd.clear(); // 画面をクリア

  updateLcd("CAN error",1);
}

CAN_message_t rxmsg;

String text1 = "";
String text2 = "";

void loop() {
  // CANメッセージを読み込み、ライブラリにデータを渡す
  updateLcd(text1,0);
  updateLcd(text2,1);
  delay(10);
}

void timerCallback() {
  // Serial1ポートにデータが届いているかチェック
  if (Serial1.available() > 0) {
    // 改行コード('\n')までを1つの文字列として読み込む
    String uartString = Serial1.readStringUntil('\n');

    // 文字列の前後の不要な空白や改行コードを削除
    uartString.trim();

    if (uartString.length() > 0) {
      text1 = uartString;
    }
  }

  while (can1.read(rxmsg)) {
    if (rxmsg.id == 0x201) {
      int16_t angle = (rxmsg.buf[0] << 8) | rxmsg.buf[1];
      int16_t rotation = (rxmsg.buf[2] << 8) | rxmsg.buf[3];
      text2 = String(angle);
    }
  }
}

void updateLcd(String str, int16_t i ) {
  lcd.setCursor(0, i); // 1行目の先頭にカーソルを移動
  lcd.print("                "); // 16文字の空白
  lcd.setCursor(0, i); // 1行目の先頭にカーソルを移動
  lcd.print(str);
}
