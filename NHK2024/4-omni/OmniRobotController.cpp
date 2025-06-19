/**
 * @file OmniRobotController.cpp
 * @brief OmniRobotControllerクラスの実装ファイル
 * @author (あなたの名前)
 * @date 2025-06-19
 */

#include "OmniRobotController.h"

namespace { // 無名名前空間: このファイル内でのみ有効な定数
  constexpr float PI_VAL = 3.1415926535f;
  constexpr float sin45 = 0.707106781f;
  constexpr float cos45 = 0.707106781f;
  constexpr int NUM_MOTORS = 4;
  constexpr int MAX_POWER = 16000;
  
  const float wheel_kinematics_coeffs[NUM_MOTORS][2] = {
    {-sin45, cos45}, {-cos45, -sin45}, {sin45, -cos45}, {cos45, sin45}
  };
}

OmniRobotController::OmniRobotController(const RobotParams& params) 
  : m_params(params), m_current_state(State::IDLE) {
  // デフォルトゲインを設定
  m_speed_pid_gains = {10.0f, 0.0f, 0.0f};
  m_position_pid_gains = {30.0f, 0.0f, 1000.0f};
  
  m_target_velocity_x = 0;
  m_target_velocity_y = 0;
  m_target_omega = 0;
  for (int i = 0; i < NUM_MOTORS; ++i) {
    m_wheel_target_rpm[i] = 0;
    m_power_command[i] = 0;
  }
}

void OmniRobotController::setSpeedPidGains(const PidGains& gains) {
  m_speed_pid_gains = gains;
}

void OmniRobotController::setPositionPidGains(const PidGains& gains) {
  m_position_pid_gains = gains;
}

void OmniRobotController::begin() {
  m_current_state = State::IDLE;
  for (int i = 0; i < NUM_MOTORS; i++) {
    m_motors[i].pid.init(m_speed_pid_gains.kp, m_speed_pid_gains.ki, m_speed_pid_gains.kd);
    m_motors[i].rad_pid.init(m_position_pid_gains.kp, m_position_pid_gains.ki, m_position_pid_gains.kd, 
                             m_params.encoder_width, m_params.wheel_radius, m_params.gear_ratio);
  }
}

void OmniRobotController::updateStateMachine() {
  switch (m_current_state) {
    case State::IDLE: break;
    case State::MOVING: handleMoving(); break;
    case State::MANUAL_CONTROL: break;
  }
}

void OmniRobotController::updateControl() {
  for(int i = 0; i < NUM_MOTORS; ++i) {
    int rpm_command = m_wheel_target_rpm[i];
    int power_command = m_motors[i].pid.pid_out(rpm_command);
    m_power_command[i] = limit(power_command, MAX_POWER);
  }
}

void OmniRobotController::moveTo(float x, float y, float theta, int rpm) {
  if (m_current_state == State::IDLE) {
    m_motion_target = {x, y, theta, rpm};
    m_current_state = State::MOVING;
  }
}

void OmniRobotController::setManualVelocity(float vx, float vy, float omega) {
  if (vx != 0 || vy != 0 || omega != 0) {
    if (m_current_state != State::MANUAL_CONTROL) {
      m_current_state = State::MANUAL_CONTROL;
    }
    m_target_velocity_x = conv_trapezoid(vx, m_target_velocity_x);
    m_target_velocity_y = conv_trapezoid(vy, m_target_velocity_y);
    m_target_omega = omega;
    calculateWheelRPMsFromVelocity();
  } else if (m_current_state == State::MANUAL_CONTROL) {
    stop();
  }
}

void OmniRobotController::stop() {
  m_target_velocity_x = 0; m_target_velocity_y = 0; m_target_omega = 0;
  calculateWheelRPMsFromVelocity();
  m_current_state = State::IDLE;
}

void OmniRobotController::updateMotorState(int motor_index, int16_t angle, int16_t rotation) {
  if (motor_index < 0 || motor_index >= NUM_MOTORS) return;
  m_motors[motor_index].data.angle = angle;
  m_motors[motor_index].data.rotation = rotation;
  m_motors[motor_index].rad_pid.now_value(angle);
  m_motors[motor_index].pid.now_value(rotation);
}

void OmniRobotController::getMotorPowerCommands(int power_commands[4]) {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    power_commands[i] = m_power_command[i];
  }
}

void OmniRobotController::handleMoving() {
  float dx = (-m_motors[0].rad_pid.debug() * sin45 - m_motors[1].rad_pid.debug() * sin45 + m_motors[2].rad_pid.debug() * sin45 + m_motors[3].rad_pid.debug() * sin45) / 2.0f;
  float dy = (+m_motors[0].rad_pid.debug() * sin45 - m_motors[1].rad_pid.debug() * sin45 - m_motors[2].rad_pid.debug() * sin45 + m_motors[3].rad_pid.debug() * sin45) / 2.0f;
  float dt = (m_motors[0].rad_pid.debug() + m_motors[1].rad_pid.debug() + m_motors[2].rad_pid.debug() + m_motors[3].rad_pid.debug()) / 4.0f;

  float error_dist = abs(m_motion_target.x - dx) + abs(m_motion_target.y - dy);
  float error_theta = abs(PI_VAL * m_params.diagonal_distance * m_motion_target.theta / 360.0f - dt);

  if (error_dist + error_theta < 5.0f) { // COMPLETION_THRESHOLD
    stop();
    return;
  }

  for (int i = 0; i < NUM_MOTORS; ++i) {
    float wheel_pos_command = wheel_kinematics_coeffs[i][0] * m_motion_target.x + wheel_kinematics_coeffs[i][1] * m_motion_target.y + (0.5 * PI_VAL * m_params.diagonal_distance * m_motion_target.theta / 360.0f);
    int target_rpm = m_motors[i].rad_pid.pid_out(wheel_pos_command);
    m_wheel_target_rpm[i] = limit(target_rpm, m_motion_target.rpm);
  }
}

void OmniRobotController::calculateWheelRPMsFromVelocity() {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    m_wheel_target_rpm[i] = wheel_kinematics_coeffs[i][0] * m_target_velocity_x + wheel_kinematics_coeffs[i][1] * m_target_velocity_y + 0.5 * m_target_omega;
  }
}

int OmniRobotController::limit(int input, int max_min) {
  return max(-max_min, min(input, max_min));
}

int OmniRobotController::conv_trapezoid(int target, int now_value) {
  constexpr int a_max = 200;
  constexpr int conv_th = 2000;
  return abs(now_value) < conv_th ? target : now_value + limit(target - now_value, a_max);
}