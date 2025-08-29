#pragma once
#include "motor.hpp"
#include <Arduino.h>

class IndependentDjiMotorDriver : public IMotorDriver {
public:
    struct DjiRawFeedback {
        uint16_t angleRaw = 0;
        int16_t speedRaw = 0;
        int16_t current = 0;
        uint32_t timestamp = 0;
    };

private:
    uint8_t motorId_;
    int16_t currentCommand_ = 0;
    bool hasNewCommand_ = false;
    
    DjiRawFeedback rawFeedback_;
    Feedback processedFeedback_;
    
    uint16_t lastAngleRaw_ = 0;
    int32_t positionCount_ = 0;
    bool firstFeedback_ = true;
    
    static constexpr float CNT2DEG = 360.0f / (8192.0f * 72.0f);
    static constexpr float CURRENT_TO_RAW = 16384.0f;
    static constexpr uint32_t CONNECTION_TIMEOUT_MS = 100;
    
public:
    explicit IndependentDjiMotorDriver(uint8_t motorId) : motorId_(motorId) {}
    
    Feedback getFeedback() override { return processedFeedback_; }
    
    void setCurrent(float normalizedCurrent) override {
        currentCommand_ = static_cast<int16_t>(constrain(normalizedCurrent, -1.0f, 1.0f) * CURRENT_TO_RAW);
        hasNewCommand_ = true;
    }
    
    bool flush() override { return hasNewCommand_; }
    
    bool isConnected() override {
        return (millis() - rawFeedback_.timestamp) < CONNECTION_TIMEOUT_MS;
    }
    
    void resetPosition(float position) override {
        positionCount_ = static_cast<int32_t>(position / CNT2DEG);
        lastAngleRaw_ = rawFeedback_.angleRaw;
        updateProcessedFeedback();
    }
    
    void clearErrors() override {}
    
    void receiveFeedback(uint16_t angleRaw, int16_t speedRaw, int16_t current) {
        rawFeedback_.angleRaw = angleRaw;
        rawFeedback_.speedRaw = speedRaw;
        rawFeedback_.current = current;
        rawFeedback_.timestamp = millis();
        
        if (!firstFeedback_) {
            int16_t delta = angleRaw - lastAngleRaw_;
            if (delta > 4096) delta -= 8192;
            else if (delta < -4096) delta += 8192;
            positionCount_ += delta;
        } else {
            firstFeedback_ = false;
        }
        
        lastAngleRaw_ = angleRaw;
        updateProcessedFeedback();
    }
    
    int16_t getCurrentCommand() const { return currentCommand_; }
    uint8_t getMotorId() const { return motorId_; }
    bool hasNewCommand() const { return hasNewCommand_; }
    void clearCommandFlag() { hasNewCommand_ = false; }
    const DjiRawFeedback& getRawFeedback() const { return rawFeedback_; }

private:
    void updateProcessedFeedback() {
        processedFeedback_.position = positionCount_ * CNT2DEG;
        processedFeedback_.velocity = static_cast<float>(rawFeedback_.speedRaw);
        processedFeedback_.current = static_cast<float>(rawFeedback_.current) / 1000.0f;  // mA → A
        processedFeedback_.time = rawFeedback_.timestamp;
        processedFeedback_.connected = isConnected();
    }
};
