#pragma once
#include <FlexCAN_T4.h>

template <typename CAN_T>
class SimpleCAN {
public:
    using ReceiveCallback = void(*)(uint32_t id, const uint8_t* data, uint8_t len);

    SimpleCAN(CAN_T& can) : can_(can), callback_(nullptr) {
        instance_ = this; // 単一インスタンスとして登録
    }

    void begin(uint32_t baud, ReceiveCallback callback) {
        callback_ = callback;
        can_.begin();
        can_.setBaudRate(baud);
        can_.enableFIFO();
        can_.enableFIFOInterrupt();
        can_.onReceive(onReceiveStatic);
    }

    bool send(uint32_t id, const uint8_t* data, uint8_t len) {
        CAN_message_t msg;
        msg.id = id;
        msg.len = len;
        memcpy(msg.buf, data, len);
        return can_.write(msg);
    }

private:
    static void onReceiveStatic(const CAN_message_t &msg) {
        if (instance_ && instance_->callback_) {
            instance_->callback_(msg.id, msg.buf, msg.len);
        }
    }

    CAN_T& can_;
    ReceiveCallback callback_;
    inline static SimpleCAN* instance_ = nullptr;
};
