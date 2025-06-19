/**
 * @file BasicMovement.ino
 * @brief OmniRobotControllerライブラリの使用例を示すサンプルスケッチ
 * @details
 * - Teensy 4.x と FlexCAN_T4 を使用することを想定。
 * - 物理パラメータやPIDゲインをスケッチ側で設定し、ライブラリに渡す。
 * - 起動3秒後に、(500, 500) へ90度向きで移動を開始する。
 * * @note
 * このスケッチをコンパイルするには、以下のライブラリが必要です。
 * - OmniRobotController (今回作成)
 * - FlexCAN_T4
 * - MsTimer2
 * - PID (自作)
 * - rad_pid (自作)
 */

#include "OmniRobotController.h"
#include <FlexCAN_T4.h>
#include <MsTimer2.h>
#include <LiquidCrystal.h> // ADDED: LCDライブラリを追加

// --- 1. ハードウェアとライブラリのインスタンス化 ---
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// ADDED: LCDのピン設定 (実際の配線に合わせてピン番号を変更してください)
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// ロボットの物理パラメータを設定
RobotParams robot_physical_params = {
  .wheel_radius = 50.0f,
  .diagonal_distance = 660.0f,
  .gear_ratio = 19,
  .encoder_width = 8191
};

// パラメータを渡してロボット制御クラスのインスタンスを作成
OmniRobotController robot(robot_physical_params);

// 手動操作用のコントローラー入力（デバッグ用）
// 実際の運用では、PS4コントローラーなどからの入力でこの値を更新する
struct ControllerInput { int R_x, R_y, L_x, L_y; };
ControllerInput controller_input = {0, 0, 0, 0};

const float CONTROLLER_MAX_VALUE = 128.0f;
const int MAX_SPEED = 4000;

unsigned long lastLcdUpdateTime = 0;
const int LCD_UPDATE_INTERVAL = 200; // 200msごとにLCDを更新


// --- 2. 初期化 ---
void setup() {
  Serial1.begin(115200);

  lcd.begin(16, 2); // 16文字 x 2行
  lcd.print("Booting...");

  // --- PIDゲインのチューニング ---
  // スケッチ側で自由にゲインを設定できる
  robot.setSpeedPidGains({ .kp = 12.5f, .ki = 0.0f, .kd = 0.0f });
  robot.setPositionPidGains({ .kp = 35.0f, .ki = 0.0f, .kd = 950.0f });

  // ハードウェアとライブラリの初期化
  can1.begin();
  can1.setBaudRate(1000000);
  robot.begin();

  // タイマー割り込みの設定
  MsTimer2::set(1, timerCallback);
  MsTimer2::start(); 
}

/**
 * @brief UART(シリアル)からの入力をチェックし、controller_inputを更新する
 * @note フォーマット: "lx,ly,rx,ry\n" (例: "0.5,0.0,-0.2,0.0\n")
 */
void checkSerial1Input() {

  if (Serial1.available() > 0) {
    // 改行文字まで文字列として一括で読み込む
    String input_string = Serial1.readStringUntil('\n');

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

// --- 3. メインループ ---
void loop() {
  float vx = (float)controller_input.L_x / CONTROLLER_MAX_VALUE * MAX_SPEED;
  float vy = (float)controller_input.L_y / CONTROLLER_MAX_VALUE * MAX_SPEED;
  float omega = (float)controller_input.R_x / CONTROLLER_MAX_VALUE * MAX_SPEED;


  //       L_y -> X軸方向の並進速度 (vx)
  //       L_x -> Y軸方向の並進速度 (vy)
  //       R_x -> 旋回速度 (omega)
  robot.setManualVelocity(vx, vy, omega);

  // ライブラリに状態機械の更新を依頼
  robot.updateStateMachine();

  updateLcdDisplay();

  delay(5);
}

void updateLcdDisplay() {
  // 一定時間ごとにのみ更新処理を実行
  if (millis() - lastLcdUpdateTime > LCD_UPDATE_INTERVAL) {
    lastLcdUpdateTime = millis();
    
    // --- 1行目: 現在の状態を表示 ---
    lcd.setCursor(0, 0);
    lcd.print("State: ");
    switch (robot.getCurrentState()) {
      case OmniRobotController::State::IDLE:
        lcd.print("IDLE        "); // ← 末尾のスペースで前の表示を消す
        break;
      case OmniRobotController::State::MOVING:
        lcd.print("MOVING      ");
        break;
      case OmniRobotController::State::MANUAL_CONTROL:
        lcd.print("MANUAL      ");
        break;
    }

    // --- 2行目: 状態に応じた指令値を表示 ---
    lcd.setCursor(0, 1);
    switch (robot.getCurrentState()) {
      case OmniRobotController::State::IDLE:
        lcd.print("Ready...        ");
        break;
      case OmniRobotController::State::MOVING:
        // 自律移動中の目標値を表示
        // (motion_targetはスケッチ側のグローバル変数にする必要がある)
        // ※この機能のため、motion_targetをライブラリからスケッチ側に移すか、
        //   ライブラリに目標値を取得するメソッドを追加する必要があります。
        //   今回は手動操作の表示を実装します。
        lcd.print("Target Mode     ");
        break;
      case OmniRobotController::State::MANUAL_CONTROL:
        // 手動操作の指令値を表示
        char buffer[17];
        sprintf(buffer, "%4d,%4d,%4d",
                controller_input.L_x, 
                controller_input.L_y, 
                controller_input.R_x);
        lcd.print(buffer);
        break;
    }
  }
}


// --- 4. タイマー割り込みコールバック ---
void timerCallback() {
  checkSerial1Input();

  CAN_message_t rxmsg;

  // CANメッセージを読み込み、ライブラリにデータを渡す
  while (can1.read(rxmsg)) {
    if (rxmsg.id >= 0x201 && rxmsg.id <= 0x204) {
      int index = rxmsg.id - 0x201;
      int16_t angle = (rxmsg.buf[0] << 8) | rxmsg.buf[1];
      int16_t rotation = (rxmsg.buf[2] << 8) | rxmsg.buf[3];
      robot.updateMotorState(index, angle, rotation);
    }
  }

  // ライブラリに高精度な制御計算を実行させる
  robot.updateControl();

  // 計算結果を取得
  int power_commands[4];
  robot.getMotorPowerCommands(power_commands);

  // CANでモーターに送信
  CAN_message_t txmsg;
  txmsg.id = 0x200;
  txmsg.len = 8;
  for (int i = 0; i < 4; i++) {
    txmsg.buf[i * 2]     = power_commands[i] >> 8;
    txmsg.buf[i * 2 + 1] = power_commands[i] & 0xFF;
  }
  can1.write(txmsg);
}