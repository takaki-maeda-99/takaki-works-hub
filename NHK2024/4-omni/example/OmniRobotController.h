#include "WString.h"
/**
 * @file OmniRobotController.h
 * @brief 4輪オムニホイールロボットの制御ロジックを提供するライブラリのヘッダーファイル
 * @author (あなたの名前)
 * @date 2025-06-19
 */
#ifndef OMNI_ROBOT_CONTROLLER_H
#define OMNI_ROBOT_CONTROLLER_H

#include <Arduino.h>
#include "PID.h"      // 依存ライブラリ
#include "rad_pid.h"  // 依存ライブラリ

// ロボットの物理パラメータを定義する構造体
struct RobotParams {
  float wheel_radius;      // ホイール半径 (mm)
  float diagonal_distance; // 機体の対角距離 (mm)
  int gear_ratio;          // ギア比
  int encoder_width;       // エンコーダ分解能
};

// PIDゲインを定義する構造体
struct PidGains {
  float kp, ki, kd;
};

class OmniRobotController {
public:
  // コンストラクタ：ロボットの物理パラメータを受け取る
  OmniRobotController(const RobotParams& params);

  // 内部状態を定義
  enum class State { IDLE, MOVING, MANUAL_CONTROL };

  // 初期化
  void begin();

  // PIDゲインを設定するメソッド
  void setSpeedPidGains(const PidGains& gains);     // 速度PID用 (RPM)
  void setPositionPidGains(const PidGains& gains);  // 位置PID用 (Rad)

  // メインループから呼ばれ、状態遷移を管理する
  void updateStateMachine();

  // タイマー割り込みから呼ばれ、高精度な制御計算を行う
  void updateControl();

  // --- APIメソッド (ユーザーが呼び出す) ---
  void moveTo(float x, float y, float theta, int rpm);
  void setManualVelocity(float vx, float vy, float omega);
  void stop();

  // --- データ交換メソッド ---
  void updateMotorState(int motor_index, int16_t angle, int16_t rotation);
  void getMotorPowerCommands(int power_commands[4]);

  // 現在の状態を取得するためのメソッド
  State getCurrentState() const;

private:
  
  // 内部で使われるデータ構造
  struct MotionTarget { float x, y, theta; int rpm; };
  struct DjiEscData { int16_t angle; int16_t rotation; };
  struct MotorController { DjiEscData data; Pid pid; RadPid rad_pid; };
  
  // メンバー変数
  RobotParams m_params;
  PidGains m_speed_pid_gains;
  PidGains m_position_pid_gains;
  State m_current_state;
  MotionTarget m_motion_target;
  MotorController m_motors[4];
  float m_target_velocity_x, m_target_velocity_y, m_target_omega;
  int m_wheel_target_rpm[4];
  int m_power_command[4];

  // 内部処理用のプライベートメソッド
  void handleMoving();
  void calculateWheelRPMsFromVelocity();
  int limit(int input, int max_min);
  int conv_trapezoid(int target, int now_value);
};

#endif // OMNI_ROBOT_CONTROLLER_H