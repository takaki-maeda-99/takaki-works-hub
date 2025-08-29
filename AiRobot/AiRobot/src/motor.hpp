#pragma once
#include <cstdint>
#include <functional>
#include <Arduino.h>

enum class CtrlMode : uint8_t {
    Disabled, Homing, Ready, Position, Velocity, Torque
};

struct Feedback {
    float position;        // 位置 [deg, mm, rad等の物理単位]
    float velocity;        // 速度 [rpm, mm/s, rad/s等]
    float current;         // 電流 [A] (トルク推定用)
    uint32_t time;
    bool connected;
};

struct Config {
    float limit_current, limit_vel;
    float min_pos, max_pos;        // 物理単位での制限値
    float gear, cpr, radius;
    bool bounded = false;
    uint32_t timeout = 50;
};

struct Pid {
    float Kp = 1.0f, Ki = 0.0f, Kd = 0.0f;
    float integralMax = 1000.0f, outputMax = 10000.0f;
    float integral = 0.0f, lastError = 0.0f;
    uint32_t lastTime = 0;
    
    float compute(float error, uint32_t currentTime) {
        float dt = (currentTime - lastTime) / 1000.0f;
        if (dt <= 0 || lastTime == 0) dt = 0.001f;
        
        integral += error * dt;
        integral = constrain(integral, -integralMax, integralMax);
        
        float derivative = (error - lastError) / dt;
        float output = Kp * error + Ki * integral + Kd * derivative;
        
        lastError = error;
        lastTime = currentTime;
        
        return constrain(output, -outputMax, outputMax);
    }
    
    void reset() {
        integral = lastError = 0.0f;
        lastTime = 0;
    }
};

class IMotorDriver {
public:
    virtual ~IMotorDriver() = default;
    virtual Feedback getFeedback() = 0;
    virtual void setCurrent(float normalizedCurrent) = 0;
    virtual bool flush() = 0;
    virtual bool isConnected() = 0;
    virtual void resetPosition(float position) = 0;
    virtual void clearErrors() = 0;
};


inline float wrapPosition(float position, float min_pos, float max_pos) {
    float range = max_pos - min_pos;
    if (range <= 0) return position;
    while (position >= max_pos) position -= range;
    while (position < min_pos) position += range;
    return position;
}

inline float shortestPathError(float current, float target, float min_pos, float max_pos) {
    float range = max_pos - min_pos;
    if (range <= 0) return target - current;
    
    float direct_error = target - current;
    float wrap_error = (target > current) ? direct_error - range : direct_error + range;
    
    return (fabs(direct_error) <= fabs(wrap_error)) ? direct_error : wrap_error;
}

class Motor {
private:
    IMotorDriver* driver_;
    Config config_;
    Pid pid_pos_, pid_vel_, pid_current_;
    
    CtrlMode mode_ = CtrlMode::Disabled;
    float target_ = 0.0f;
    Feedback feedback_;
    
    bool homingComplete_ = false;
    std::function<bool()> homingCondition_;
    float homingSpeed_ = 1000.0f;
    float homePosition_ = 0.0f;
    
    // 速度監視ホーミング用
    bool velocityHomingMode_ = false;
    uint32_t stallStartTime_ = 0;
    uint32_t stallThresholdMs_ = 500;  // 500ms停止で原点認識
    float stallVelocityThreshold_ = 10.0f;  // 10rpm以下で停止とみなす
    
    // デュアルLS ホーミング用
    bool dualLsMode_ = false;
    std::function<bool()> dualLsCondition1_;
    std::function<bool()> dualLsCondition2_;

public:
    Motor(IMotorDriver* driver, const Config& config = Config{})
        : driver_(driver), config_(config) {}
    
    void setPosition(float targetDeg) {
        mode_ = CtrlMode::Position;
        if (config_.bounded) {
            target_ = constrain(targetDeg, config_.min_pos, config_.max_pos);
        } else {
            target_ = wrapPosition(targetDeg, config_.min_pos, config_.max_pos);
        }
    }
    
    void setVelocity(float targetRpm) {
        mode_ = CtrlMode::Velocity;
        target_ = constrain(targetRpm, -config_.limit_vel, config_.limit_vel);
    }
    
    void setCurrent(float targetCurrent) {
        mode_ = CtrlMode::Torque;
        target_ = constrain(targetCurrent, -config_.limit_current, config_.limit_current);
    }
    
    void disable() {
        mode_ = CtrlMode::Disabled;
        target_ = 0.0f;
        pid_pos_.reset();
        pid_vel_.reset();
        pid_current_.reset();
    }
    
    void startHoming(std::function<bool()> condition, float homePos = 0.0f, float speed = 1000.0f) {
        homingCondition_ = condition;
        homePosition_ = homePos;
        homingSpeed_ = speed;
        homingComplete_ = false;
        velocityHomingMode_ = false;
        dualLsMode_ = false;  // ← 重要！
        mode_ = CtrlMode::Homing;
    }
    
    // ステア用2LS同時ON検出ホーミング
    void startDualLimitSwitchHoming(std::function<bool()> ls1, std::function<bool()> ls2, 
                                   float homePos = 0.0f, float speed = 3000.0f) {
        dualLsCondition1_ = ls1;
        dualLsCondition2_ = ls2;
        homePosition_ = homePos;
        homingSpeed_ = speed;
        homingComplete_ = false;
        velocityHomingMode_ = false;
        dualLsMode_ = true;  // ← この行が重要！
        mode_ = CtrlMode::Homing;
    }
    
    void startVelocityHoming(float homePos = 0.0f, float speed = 500.0f, float stallThreshold = 10.0f, uint32_t stallTime = 500) {
        homePosition_ = homePos;
        homingSpeed_ = speed;
        stallVelocityThreshold_ = stallThreshold;
        stallThresholdMs_ = stallTime;
        homingComplete_ = false;
        velocityHomingMode_ = true;
        dualLsMode_ = false;  // ← 重要！
        stallStartTime_ = 0;
        mode_ = CtrlMode::Homing;
    }
    
    bool isHomingComplete() const { return homingComplete_; }
    
    void update() {
        feedback_ = driver_->getFeedback();
        float output = 0.0f;
        uint32_t currentTime = millis();
        
        switch (mode_) {
            case CtrlMode::Disabled:
                output = 0.0f;
                break;
            case CtrlMode::Position: {
                float posError = config_.bounded ? 
                    target_ - feedback_.position :
                    shortestPathError(feedback_.position, target_, config_.min_pos, config_.max_pos);
                
                float velTarget = pid_pos_.compute(posError, currentTime);
                output = pid_vel_.compute(velTarget - feedback_.velocity, currentTime);
                break;
            }
            case CtrlMode::Velocity:
                output = pid_vel_.compute(target_ - feedback_.velocity, currentTime);
                break;
            case CtrlMode::Torque:
                output = target_;
                break;
            case CtrlMode::Homing:
                if (velocityHomingMode_) {
                    // 速度監視ホーミング（unbounded用）
                    if (fabs(feedback_.velocity) < stallVelocityThreshold_) {
                        if (stallStartTime_ == 0) {
                            stallStartTime_ = currentTime;
                        } else if (currentTime - stallStartTime_ > stallThresholdMs_) {
                            driver_->resetPosition(homePosition_);
                            homingComplete_ = true;
                            mode_ = CtrlMode::Ready;
                            output = 0.0f;
                            break;
                        }
                    } else {
                        stallStartTime_ = 0;
                    }
                    output = pid_vel_.compute(homingSpeed_ - feedback_.velocity, currentTime);
                } else if (dualLsMode_) {
                    // デュアルLS ホーミング（ステア用）
                    if (dualLsCondition1_ && dualLsCondition1_() && 
                        dualLsCondition2_ && dualLsCondition2_()) {
                        // LS検出時：即座に停止してホーム位置設定
                        driver_->resetPosition(homePosition_);
                        homingComplete_ = true;
                        mode_ = CtrlMode::Ready;
                        pid_vel_.reset();  // PIDリセット
                        output = 0.0f;
                    } else {
                        // LS未検出：設定速度で回転
                        output = pid_vel_.compute(homingSpeed_ - feedback_.velocity, currentTime);
                    }
                } else {
                    // 従来のシングルLS監視ホーミング
                    if (homingCondition_ && homingCondition_()) {
                        driver_->resetPosition(homePosition_);
                        homingComplete_ = true;
                        mode_ = CtrlMode::Ready;
                        output = 0.0f;
                    } else {
                        output = pid_vel_.compute(homingSpeed_ - feedback_.velocity, currentTime);
                    }
                }
                break;
            case CtrlMode::Ready: {
                // Ready状態：ホーム位置での位置制御
                float posError = config_.bounded ? 
                    homePosition_ - feedback_.position :
                    shortestPathError(feedback_.position, homePosition_, config_.min_pos, config_.max_pos);
                
                float velTarget = pid_pos_.compute(posError, currentTime);
                output = pid_vel_.compute(velTarget - feedback_.velocity, currentTime);
                break;
            }
            default:
                output = 0.0f;
                break;
        }
        
        driver_->setCurrent(constrain(output / config_.limit_current, -1.0f, 1.0f));
    }
    
    void setPidPosition(float Kp, float Ki, float Kd, float outMax = 10000.0f) {
        pid_pos_.Kp = Kp; pid_pos_.Ki = Ki; pid_pos_.Kd = Kd; pid_pos_.outputMax = outMax;
    }
    
    void setPidVelocity(float Kp, float Ki, float Kd, float outMax = 10000.0f) {
        pid_vel_.Kp = Kp; pid_vel_.Ki = Ki; pid_vel_.Kd = Kd; pid_vel_.outputMax = outMax;
    }
    
    void setPidCurrent(float Kp, float Ki, float Kd, float outMax = 10000.0f) {
        pid_current_.Kp = Kp; pid_current_.Ki = Ki; pid_current_.Kd = Kd; pid_current_.outputMax = outMax;
    }
    
    const Feedback& getFeedback() const { return feedback_; }
    CtrlMode getMode() const { return mode_; }
    float getTarget() const { return target_; }
    const Config& getConfig() const { return config_; }
    void setConfig(const Config& config) { config_ = config; }
};
