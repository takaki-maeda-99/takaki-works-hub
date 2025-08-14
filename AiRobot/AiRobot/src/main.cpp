#include <FlexCAN_T4.h>
#include "simplePID.h"
#include "DjiMotor.hpp"
#include <IntervalTimer.h>
#include "indicator.hpp"
#include "controller.hpp"
#include "MathUtil.hpp"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

DjiMotorCan1 dji2(can1);
DjiMotorCan2 dji1(can2);

DjiFeedback fb1[8];
DjiFeedback fb2[8];

#define LSPIN11 2
#define LSPIN12 3
#define LSPIN21 4
#define LSPIN22 5
#define LSPIN31 6
#define LSPIN32 7
#define LSPIN41 8
#define LSPIN42 9

struct PIDParam {
    float kp, ki, kd;
    int16_t outMin, outMax;
    uint32_t sampleMs;
};

constexpr float CNT2DEG_OUT = 360.0f / (8192.0f * 72.0f);  // 出力軸: 1 cnt → 0.00061 deg

void printCANstatus() {
  for (int idx = 0; idx <= 6; idx++) {
    const auto &fb = dji1.feedback(idx+1);
    Serial.printf("{1-%1u,%4u,%4d}",
                  idx + 1,
                  fb.angleRaw,
                  fb.speedRaw);
  }
  for (int idx = 0; idx <= 3; idx++) {
    const auto &fb = dji2.feedback(idx+1);
    Serial.printf("{2-%1u,%4u,%4d}",
                  idx + 1,
                  fb.angleRaw,
                  fb.speedRaw);
  }
  Serial.println();
}

struct ControllerInput {
  int L_x, L_y, R_x, R_y;
};
ControllerInput controller_input = { 0, 0, 0, 0 };
char rxBuf[32];
uint8_t rxPos = 0;

void checkSerial1Input() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || rxPos >= sizeof(rxBuf)-1) {
            rxBuf[rxPos] = '\0';               // 終端
            Serial.printf("受信: %s\n", rxBuf);  // デバッグ出力
            int v[4];
            if (sscanf(rxBuf, "%d,%d,%d,%d", &v[0], &v[1], &v[2], &v[3]) == 4) {
                controller_input = { v[0], v[1], v[2], v[3] };
                Serial.printf("パース成功: L_x=%d, L_y=%d, R_x=%d, R_y=%d\n", v[0], v[1], v[2], v[3]);
            } else {
                Serial.printf("パースエラー: %s\n", rxBuf);
            }
            rxPos = 0;                         // バッファをリセット
        } else {
            rxBuf[rxPos++] = c;
        }
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline float  wrap180(float deg) { while (deg > 180) deg -= 360; while (deg < -180) deg += 360; return deg; }

// PID
//SimplePID(kp, ki, kd, outMin, outMax, sampleMs)

// ── 角度用と速度用の定数をまとめる ──
constexpr PIDParam steerAP{200, 0, 0, -10000, 10000, 10};
constexpr PIDParam steerSP{10,  0, 0, -10000, 10000, 1};
constexpr PIDParam wheelSP{30,  0, 0, -9000, 9000, 1};

// 使い回すときに feed できるユーティリティ
inline SimplePID makePID(const PIDParam &p) {
    return SimplePID(p.kp, p.ki, p.kd, p.outMin, p.outMax, p.sampleMs);
}

// PID オブジェクトを配列で保持
SimplePID SteerPidAngle[4] = { makePID(steerAP), makePID(steerAP), makePID(steerAP), makePID(steerAP)};
SimplePID SteerPidSpeed[4] = { makePID(steerSP), makePID(steerSP), makePID(steerSP), makePID(steerSP)};

SimplePID WheelPidSpeed[4] = { makePID(wheelSP), makePID(wheelSP), makePID(wheelSP), makePID(wheelSP)};

IntervalTimer m2006Timer;

// ホーミング状態管理
bool homing_complete[4] = {false, false, false, false};
// 各モーターの初期角度 [deg]
float initial_angles[4] = {90.0f, 0.0f, -90.0f, -180.0f}; // モーター1,2,3,4の初期角度

// リミットスイッチ読み取り関数
bool readLimitSwitch(int motor_id, int switch_num) {
    int pin = 0;
    if (motor_id == 0) pin = (switch_num == 1) ? LSPIN11 : LSPIN12;
    else if (motor_id == 1) pin = (switch_num == 1) ? LSPIN21 : LSPIN22;
    else if (motor_id == 2) pin = (switch_num == 1) ? LSPIN31 : LSPIN32;
    else if (motor_id == 3) pin = (switch_num == 1) ? LSPIN41 : LSPIN42;
    
    return digitalRead(pin); // アクティブLowと仮定
}

// ホーミングルーチン - 速度PID制御を使用
void performHoming(int motor_id) {
    if (homing_complete[motor_id]) return;
    
    // 現在のフィードバックを取得
    const auto& current_fb = dji1.feedback(motor_id + 1);
    
    // 2つのLSが両方ONの状態をチェック
    bool ls1_active = readLimitSwitch(motor_id, 1);
    bool ls2_active = readLimitSwitch(motor_id, 2);
    
    // 常にリミットスイッチ状態をシリアルに出力
    static uint32_t debug_counter[4] = {0};
    if (++debug_counter[motor_id] % 100 == 0) {
        Serial.printf("Motor%d homing: LS1=%d, LS2=%d, feedback: angle=%u, speed=%d\n", 
                     motor_id + 1, ls1_active, ls2_active, current_fb.angleRaw, current_fb.speedRaw);
    }
    
    if (ls1_active && ls2_active) {
        // 両方のリミットスイッチがON → 初期位置に設定
        float initial_deg = initial_angles[motor_id];
        
        // DjiMotorクラスの角度リセット機能を使用
        dji1.resetAngle(motor_id + 1, initial_deg);
        homing_complete[motor_id] = true;
        
        // 速度PIDで停止
        int16_t stopCmd = SteerPidSpeed[motor_id].compute(current_fb.speedRaw, 0);
        dji1.sendCurrent(motor_id + 1, stopCmd);
        
        Serial.printf("*** Motor %d HOMED at %.1f degrees ***\n", motor_id + 1, initial_deg);
    } else {
        // どちらかのLSがOFFの場合、速度PIDで低速回転
        const float homing_rpm = 1000.0f; // ホーミング速度 [rpm]
        int16_t homingCmd = SteerPidSpeed[motor_id].compute(current_fb.speedRaw, homing_rpm);
        dji1.sendCurrent(motor_id + 1, homingCmd);
    }
}

inline void updateSteerMotor(size_t idx, float targetDeg) {
    const auto& fb = fb1[idx];

    int16_t angleCmd  = SteerPidAngle[idx].compute(fb.positionCnt*CNT2DEG_OUT, targetDeg);
    int16_t speedCmd  = SteerPidSpeed[idx].compute(fb.speedRaw,  angleCmd);

    dji1.sendCurrent(idx + 1, speedCmd);  // モーター番号は 1 始まり
}

inline void updateWheelMotor(size_t idx, float targetSpeed) {
    const auto& fb = fb2[idx];

    int16_t speedCmd  = WheelPidSpeed[idx].compute(fb.speedRaw,  targetSpeed);

    dji2.sendCurrent(idx + 1, speedCmd);  // モーター番号は 1 始まり
}

Polar pol;

void m2006ISR(){
    // ❶ 全フィードバック先読み
  for (size_t i = 0; i < 8; ++i) fb1[i] = dji1.feedback(i + 1);  
  for (size_t i = 0; i < 4; ++i) fb2[i] = dji2.feedback(i + 1);

  // ホーミング中は通常制御をスキップ
  bool all_homed = true;
  for (int i = 0; i < 4; i++) {
    if (!homing_complete[i]) {
      all_homed = false;
      break;
    }
  }
  
  if (!all_homed) {
    return; // ホーミング中は通常制御をスキップ
  }

  // 例: ロボットの半分の長さと幅（中心から各ホイールまでの距離）
  const float half_length = 0.25f;   // 単位[m]（例）
  const float half_width  = 0.25f;   // 単位[m]（例）

  // コントローラ入力（必要に応じスケール変換）
  float vx    = controller_input.L_x;    // 前後方向速度指令
  float vy    = controller_input.L_y;    // 左右方向速度指令
  float omega = -controller_input.R_x;    // 旋回速度指令（±値）

  // 各ホイール位置オフセット（前右, 前左, 後左, 後右）
  // ※ロボット中心を原点、+x前方、+y右方向の座標系と仮定
  float wheelX[4] = { +half_length, -half_length, -half_length, +half_length };
  float wheelY[4] = { +half_width,  +half_width,  -half_width,  -half_width  };

  // 各ホイールの目標角度・速度を計算
  for (int i = 0; i < 4; ++i) {
      float vx_i = vx - omega * wheelY[i];
      float vy_i = vy + omega * wheelX[i];
      float targetAngle = atan2(vy_i, vx_i) * RAD2DEG;   // 修正：引数順序を正しく
      float targetSpeed = hypotf(vx_i, vy_i);     // 大きさ（速度指令）
      
      // 全モーターで角度制御を実行（全フィードバック有効）
      // 現在角度との偏差を算出（現在角度も同じ基準でdegに換算）
      float currentAngle = fb1[i].positionCnt * CNT2DEG_OUT;  // fb1から現在角度[deg]取得
      
      // 無限回転対応：最短経路で目標角度へ向かう
      // 角度差を-180〜+180に正規化して最短経路を選択
      float delta = wrap180(targetAngle - currentAngle);
      float finalTargetAngle = currentAngle + delta;  // 最短経路の目標角度
      
      // 計算した目標値を各PID制御にセット
      int16_t SteerAngleCmd  = SteerPidAngle[i].compute(currentAngle, finalTargetAngle);
      int16_t SteerSpeedCmd  = SteerPidSpeed[i].compute(fb1[i].speedRaw,  SteerAngleCmd);
      dji1.sendCurrent(i + 1, SteerSpeedCmd);  // モーター番号は 1 始まり

      int16_t WheelSpeedCmd  = WheelPidSpeed[i].compute(fb2[i].speedRaw,  targetSpeed*70);
      dji2.sendCurrent(i + 1, WheelSpeedCmd);  // モーター番号は 1 始まり

  }
  dji1.flush();
  dji2.flush();

}

void setup() {
  // led
  pinMode(13, OUTPUT);

  // リミットスイッチピン設定
  pinMode(LSPIN11, INPUT_PULLUP);
  pinMode(LSPIN12, INPUT_PULLUP);
  pinMode(LSPIN21, INPUT_PULLUP);
  pinMode(LSPIN22, INPUT_PULLUP);
  pinMode(LSPIN31, INPUT_PULLUP);
  pinMode(LSPIN32, INPUT_PULLUP);
  pinMode(LSPIN41, INPUT_PULLUP);
  pinMode(LSPIN42, INPUT_PULLUP);

  lcd.begin(16,2);
  lcdPrintLine(0,"Hello world.");

  // usb serial
  Serial.begin(115200);
  Serial.setTimeout(10);

  m2006Timer.begin(m2006ISR, 1000);          // 1000 µs = 1 kHz
  m2006Timer.priority(128);                  // (0=最高, 255=最低) デフォ 128

  Serial.println("Starting homing sequence...");
}


void loop() {
  printCANstatus();

  // ホーミングシーケンス実行
  for (int i = 0; i < 4; i++) {
    if (!homing_complete[i]) {
      performHoming(i);
    } else {
      // ホーミング完了したモーターは速度0で停止維持
      const auto& current_fb = dji1.feedback(i + 1);
      int16_t stopCmd = SteerPidSpeed[i].compute(current_fb.speedRaw, 0);
      dji1.sendCurrent(i + 1, stopCmd);
    }
  }
  
  // 全モーターがホーミング完了したかチェック
  bool all_homed = true;
  for (int i = 0; i < 4; i++) {
    if (!homing_complete[i]) {
      all_homed = false;
      break;
    }
  }
  
  // 全モーターホーミング完了後のみ通常制御
  if (all_homed) {
    // serial input
    checkSerial1Input();
    
    lcdPrintLine(1,"%4d%4d%8d",
                controller_input.L_x,
                controller_input.L_y,
                controller_input.R_x);
  } else {
    // ホーミング中表示
    static int homing_counter = 0;
    if (++homing_counter % 1000 == 0) {
      Serial.printf("Homing: %d/%d complete\n", 
                   homing_complete[0] + homing_complete[1] + 
                   homing_complete[2] + homing_complete[3], 4);
    }
    dji1.flush(); // ホーミングコマンド送信
  }

  delay(1);
}
