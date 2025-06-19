/**
 * @file main.ino
 * @brief 4輪オムニホイールロボットのステートマシンベース制御スケッチ
 * @author Takaki Maeda
 * @date 2025-06-19
 *
 * @details
 * - MsTimer2による1ms周期のタイマー割り込みで高精度なPID制御とCAN通信を実行。
 * - loop()内のステートマシンで、自律移動と手動操作のモードを管理。
 * - ブロッキング処理を排除し、応答性の高いシステムを実現。
 */

// =========================================================================
// 1. ヘッダーインクルード
// =========================================================================
#include <MsTimer2.h>
#include <FlexCAN_T4.h>
#include "rad_pid.h"
#include "PID.h"

// =========================================================================
// 2. 設定・定数
// =========================================================================
namespace constants {
  // Rad_PID (位置制御用)
  constexpr float Rad_Kp = 30.0f;
  constexpr float Rad_Ki = 0.0f;
  constexpr float Rad_Kd = 1000.0f;

  // Rad PID Parameter
  constexpr int   Width = 8191;         // エンコーダの分解能
  constexpr float Radius_wheel = 50.0f; // ホイール半径 (mm)
  constexpr int   Gear = 19;            // ギア比

  // Rpm_PID (速度制御用)
  constexpr float Rpm_Kp = 10.0f;
  constexpr float Rpm_Ki = 0.0f;
  constexpr float Rpm_Kd = 0.0f;

  // 制御パラメータ
  constexpr int Max_speed = 4000;
  constexpr int Max_power = 16000;
  constexpr float Diagonal_distance = 660.0f; // 機体の対角距離 (mm)

  // 45度のsin/cos (オムニホイールの配置角度)
  constexpr float sin45 = 0.707106781f;
  constexpr float cos45 = 0.707106781f;

  // システム構成
  constexpr int NUM_MOTORS = 4;
  constexpr int BASE_CAN_ID_RX = 0x201; // 受信用ベースID
  constexpr int CAN_ID_TX = 0x200;      // 送信用ID
  
  // 移動制御パラメータ
  constexpr float COMPLETION_THRESHOLD = 5.0f; // 移動完了と判断する誤差の閾値
} // namespace constants

// namespace内の定数を `constants::` なしで使えるようにする
using namespace constants;

// NOTE: 各モーターの運動学計算における係数 (順運動学用)
// wheel_rpm = K[0]*vx + K[1]*vy + K_omega*omega の形
const float wheel_kinematics_coeffs[NUM_MOTORS][2] = {
    {-sin45,  cos45}, // Motor 0 (右前) {vx係数, vy係数}
    {-cos45, -sin45}, // Motor 1 (左前)
    { sin45, -cos45}, // Motor 2 (左後)
    { cos45,  sin45}  // Motor 3 (右後)
};

// =========================================================================
// 3. データ構造・状態定義
// =========================================================================

// DJIモーターからの受信データ
struct DjiEscDataSt {
  int16_t angle;
  int16_t rotation;
  int16_t torque;
  int8_t  temp;
};

// モーター1台に関連するオブジェクトをまとめた構造体
struct MotorController {
  DjiEscDataSt data;
  Pid pid;
  RadPid rad_pid;
};

// ロボット全体の制御目標値を保持する構造体
struct RobotControlState {
  float target_velocity_x = 0; // 目標速度 (X方向)
  float target_velocity_y = 0; // 目標速度 (Y方向)
  float target_omega = 0;      // 目標角速度

  int wheel_target_rpm[NUM_MOTORS] = { 0 };       // 各ホイールの目標RPM
  int motor_power_command[NUM_MOTORS] = { 0 }; // 各モーターへの最終的な電力指令
};

// 手動操作コントローラーからの入力を保持する構造体
struct ControllerInput {
  float R_x = 0;
  float R_y = 0;
  float L_x = 0;
  float L_y = 0;
};

// 自律移動時の最終目標地点を保持する構造体
struct MotionTarget {
  float x = 0;
  float y = 0;
  float theta = 0;
  int rpm = 0;
};

// ロボットの動作状態を定義
enum class RobotState {
  IDLE,           // 待機中
  MOVING,         // 指定位置への移動中
  MANUAL_CONTROL  // 手動速度制御中
};

// =========================================================================
// 4. グローバル変数
// =========================================================================
MotorController   motors[NUM_MOTORS];
RobotControlState robot_state;
ControllerInput   controller_input;
MotionTarget      motion_target;
RobotState        current_state = RobotState::IDLE;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
static CAN_message_t txmsg;
static CAN_message_t rxmsg;

// =========================================================================
// 5. メインのライフサイクル関数 
// =========================================================================

/**
 * @brief 初期化処理
 */
void setup() {
  Serial.begin(115200);

  // CAN通信の初期化
  can1.begin();
  can1.setBaudRate(1000000);

  // モーターコントローラー(PID)の初期化
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].pid.init(Rpm_Kp, Rpm_Ki, Rpm_Kd);
    motors[i].rad_pid.init(Rad_Kp, Rad_Ki, Rad_Kd, Width, Radius_wheel, Gear);
  }

  // 送信用CANメッセージの初期化
  txmsg.id = CAN_ID_TX;
  txmsg.len = 8;
  for (int i = 0; i < txmsg.len; ++i) {
    txmsg.buf[i] = 0;
  }

  pinMode(LED_BUILTIN, OUTPUT); // 標準のLEDピン定義を使用

  // 1ms周期のタイマー割り込みを設定
  MsTimer2::set(1, timerInt);
  MsTimer2::start();

  Serial.println("Robot Initialized. State: IDLE");
}

/**
 * @brief メインループ (ロボットの頭脳・意思決定)
 */
void loop() {
  // 1. 手動コントローラーの入力を最優先で処理
  handleControllerInput();

  // 2. 現在のロボットの状態に応じて、次の行動を決定・実行
  switch (current_state) {
    case RobotState::IDLE:
      // 待機中は特に何もしない
      break;
    // case RobotState::MOVING:
    //   handleMoving();
    //   break;
    case RobotState::MANUAL_CONTROL:
      // 実際の速度指令は handleControllerInput() で行われる
      break;
  }
  
  delay(5);
}

// =========================================================================
// 6. ステートマシン処理関数
// =========================================================================

/**
 * @brief 手動コントローラーの入力を処理し、必要に応じてモードを切り替える
 */
void handleControllerInput() {
  // コントローラーの入力値を取得
  float vx = controller_input.L_x * Max_speed;
  float vy = controller_input.L_y * Max_speed;
  float omega = controller_input.R_x * Max_speed;

  // 何か入力があれば、手動制御モードに移行
  if (vx != 0 || vy != 0 || omega != 0) {
    // 他の自律移動タスクが実行中であれば、それを中断して手動モードを優先
    if (current_state != RobotState::MANUAL_CONTROL) {
      Serial.println("Switched to MANUAL_CONTROL.");
      current_state = RobotState::MANUAL_CONTROL;
    }
    setRobotVelocity(vx, vy, omega);
  }
  // 逆に入力がなく、現在が手動モードであれば、待機モードに戻る
  else if (current_state == RobotState::MANUAL_CONTROL) {
    Serial.println("Switching back to IDLE from MANUAL_CONTROL.");
    stopMovement(); // 停止処理をしてIDLE状態に
  }
}

// /**
//  * @brief 指定位置への自律移動タスクを実行する
//  */
// void handleMoving() {
//   // 1. 全モーターのエンコーダ値からロボットの現在位置(dx, dy, dt)を推定する（逆運動学）
//   float dx = (-motors[0].rad_pid.debug() * sin45 - motors[1].rad_pid.debug() * sin45 + motors[2].rad_pid.debug() * sin45 + motors[3].rad_pid.debug() * sin45) / 2.0f;
//   float dy = (+motors[0].rad_pid.debug() * sin45 - motors[1].rad_pid.debug() * sin45 - motors[2].rad_pid.debug() * sin45 + motors[3].rad_pid.debug() * sin45) / 2.0f;
//   float dt = (motors[0].rad_pid.debug() + motors[1].rad_pid.debug() + motors[2].rad_pid.debug() + motors[3].rad_pid.debug()) / 4.0f;

//   // 2. 目標地点との誤差を計算し、完了したか判定
//   float error_dist = abs(motion_target.x - dx) + abs(motion_target.y - dy);
//   float error_theta = abs(PI * Diagonal_distance * motion_target.theta / 360.0f - dt);

//   if (error_dist + error_theta < COMPLETION_THRESHOLD) {
//     Serial.println("Movement Complete.");
//     stopMovement();
//     return; // タスク完了
//   }

//   // 3. 移動継続：位置PIDで目標RPMを計算
//   for (int i = 0; i < NUM_MOTORS; ++i) {
//     // 目標位置に到達するために、各ホイールが目指すべきエンコーダ位置を計算
//     float wheel_pos_command = wheel_kinematics_coeffs[i][0] * motion_target.x + wheel_kinematics_coeffs[i][1] * motion_target.y + (0.5 * PI * Diagonal_distance * motion_target.theta / 360.0f);
    
//     // 位置PID(rad_pid)の出力を、その瞬間の目標RPMとする
//     int target_rpm = motors[i].rad_pid.pid_out(wheel_pos_command);
//     robot_state.wheel_target_rpm[i] = limit(target_rpm, motion_target.rpm);
//   }
// }

// =========================================================================
// 7. 外部API関数
// =========================================================================

// /**
//  * @brief 指定した絶対座標への移動を開始する。
//  * @param x 目標X座標
//  * @param y 目標Y座標
//  * @param theta 目標角度
//  * @param rpm 移動時の最大RPM
//  */
// void moveTo(float x, float y, float theta, int rpm) {
//   if (current_state == RobotState::IDLE) {
//     Serial.print("Start moving to (");
//     Serial.print(x); Serial.print(", ");
//     Serial.print(y); Serial.print(", ");
//     Serial.print(theta); Serial.println(")");
//     motion_target = {x, y, theta, rpm};
//     current_state = RobotState::MOVING;
//   } else {
//     Serial.println("Cannot start new movement, robot is busy.");
//   }
// }

/**
 * @brief すべての動きを停止し、待機状態にする。
 */
void stopMovement() {
  setRobotVelocity(0, 0, 0);
  current_state = RobotState::IDLE;
  Serial.println("Movement Stopped. State: IDLE");
}

// =========================================================================
// 8. 内部制御・運動学関数
// =========================================================================

/**
 * @brief ロボットの目標速度を設定し、各ホイールの目標RPMを計算する。
 * @param target_vx 目標速度 (X方向)
 * @param target_vy 目標速度 (Y方向)
 * @param target_omega 目標角速度
 */
void setRobotVelocity(int target_vx, int target_vy, int target_omega) {
  // 台形加速を適用して急な速度変化を防ぐ
  robot_state.target_velocity_x = conv_trapezoid(target_vx, robot_state.target_velocity_x);
  robot_state.target_velocity_y = conv_trapezoid(target_vy, robot_state.target_velocity_y);
  robot_state.target_omega = target_omega;

  // 順運動学：ロボットの目標速度から各ホイールに必要な回転速度を計算
  for (int i = 0; i < NUM_MOTORS; ++i) {
    robot_state.wheel_target_rpm[i] = 
      wheel_kinematics_coeffs[i][0] * robot_state.target_velocity_x +
      wheel_kinematics_coeffs[i][1] * robot_state.target_velocity_y + 
      0.5 * robot_state.target_omega; // NOTE: 角速度の係数は機体のジオメトリに依存
  }
}

/**
 * @brief PID制御を実行し、最終的なモーターへの電力指令を計算する。
 * この関数は `timerInt` から高精度で呼び出される。
 */
void pid_run() {
  // `robot_state` に設定された目標RPMを達成するためのモーター出力を計算する
  for(int i = 0; i < NUM_MOTORS; ++i) {
    int rpm_command = robot_state.wheel_target_rpm[i];
    int power_command = motors[i].pid.pid_out(rpm_command); // 速度PID
    robot_state.motor_power_command[i] = limit(power_command, Max_power);
  }
}

// =========================================================================
// 9. ハードウェア・通信関数
// =========================================================================

/**
 * @brief タイマー割り込み関数 (ロボットの心臓)
 */
void timerInt() {
  // 1. CANメッセージを読み込み、センサー値を更新
  if (CAN_Read()) {
    // 2. 成功したら、PID計算を実行
    pid_run();
  }
  // 3. 計算されたモーター出力をCANで送信
  CAN_Write();
}

/**
 * @brief 全モーターのデータが揃うまでCANメッセージを読み込む
 * @return int 全データが受信できたら1、それ以外は0
 */
int CAN_Read() {
  bool check[NUM_MOTORS] = { false };

  while (can1.read(rxmsg)) {
    int index = rxmsg.id - BASE_CAN_ID_RX;

    if (index >= 0 && index < NUM_MOTORS) {
      motors[index].data.angle    = static_cast<int16_t>(rxmsg.buf[0] << 8 | rxmsg.buf[1]);
      motors[index].data.rotation = static_cast<int16_t>(rxmsg.buf[2] << 8 | rxmsg.buf[3]);
      motors[index].data.torque   = static_cast<int16_t>(rxmsg.buf[4] << 8 | rxmsg.buf[5]);
      motors[index].data.temp     = rxmsg.buf[6];

      motors[index].rad_pid.now_value(motors[index].data.angle);
      motors[index].pid.now_value(motors[index].data.rotation);
      
      check[index] = true;
    }
  }

  // 全データが受信できたかチェック
  for (int i = 0; i < NUM_MOTORS; ++i) {
    if (!check[i]) {
      // NOTE: 頻繁なシリアル出力はパフォーマンスに影響するため、デバッグ時のみ有効化を推奨
      // Serial.println("CAN read error.");
      return 0; // 失敗
    }
  }
  return 1; // 成功
}

/**
 * @brief モーターへの電力指令値をCANで送信する
 */
void CAN_Write() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    int power = robot_state.motor_power_command[i];
    txmsg.buf[i * 2]     = power >> 8;
    txmsg.buf[i * 2 + 1] = power & 0xFF;
  }
  can1.write(txmsg);
}

// =========================================================================
// 10. ユーティリティ・ヘルパー関数
// =========================================================================

/**
 * @brief 数値の符号を取得する (1 or -1)
 */
inline int get_sign(double num) {
  return num >= 0 ? 1 : -1;
}

/**
 * @brief 入力値を指定した最大/最小値の範囲内に収める
 */
inline int limit(int input, int max_min) {
  return max(-max_min, min(input, max_min));
}

/**
 * @brief 台形加速。目標値と現在値から、急加速しないように次のステップの値を計算する。
 */
inline int conv_trapezoid(int target, int now_value) {
  constexpr int a_max = 200;   // 最大加速度
  constexpr int conv_th = 2000; // この値以下なら目標値に即時変更
  return abs(now_value) < conv_th ? target : now_value + limit(target - now_value, a_max);
}