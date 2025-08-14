#include "usb_serial.h"
#pragma once
/**
 * @file DjiMotorCan.hpp
 * @brief DJI インテリジェント・ブラシレスモータ（M2006／M3508 など）を
 *        Teensy 4.x + FlexCAN_T4 で制御するための薄いラッパークラス。
 *
 * ## 特徴
 * - **CAN1 / CAN2 を型で分離**して安全に運用（誤バス送信防止）
 * - 最大 8 台までのモータに **電流指令**を一括発行（2 フレーム）
 * - 受信角度を用いた **多回転位置積算**を内部で自動計算
 * - 余計な抽象化を排し、ISR を使わず *polling* API のみ提供
 *
 * ## 最小構成例
 * ```cpp
 * #include <DjiMotorCan.hpp>
 * 
 * FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
 * DjiMotorCan1 motors(can1);
 * 
 * void setup() {
 *     can1.begin();
 *     can1.setBaudRate(1000000);   // DJI 推奨 1 Mbps
 * }
 * 
 * void loop() {
 *     // --- 送信サイド --------------------------------------------------
 *     static const int16_t kIq = 2000;         // 2 A 相当
 *     motors.sendCurrent(1, kIq);              // モータ ID 1 に 2 A
 *     motors.sendCurrent(2, -kIq);             // モータ ID 2 に -2 A
 *     motors.flush();                          // 2 フレーム送信 (0x200 / 0x1FF)
 * 
 *     // --- 受信サイド --------------------------------------------------
 *     uint8_t id;
 *     while ((id = motors.poll())) {
 *         const auto &fb = motors.feedback(id);
 *         Serial.printf("ID%u: pos=%ld raw=%u rpm=%d iq=%d\n",
 *                       id, fb.positionCnt, fb.angleRaw,
 *                       fb.speedRaw, fb.current);
 *     }
 * }
 * ```
 * 本ヘッダ 1 枚のみで動作します。CAN2 を使う場合は `DjiMotorCan2` を同様に生成します。
 */

#include <FlexCAN_T4.h>
#include <cstdint>
#include <cstring>

/**
 * @struct DjiFeedback
 * @brief モータ 1 台ぶんの最新フィードバック値
 */
struct DjiFeedback {
    uint16_t angleRaw     = 0;   //!< 0–8191：単回転エンコーダ値
    int16_t  speedRaw     = 0;   //!< [rpm]：実軸回転速度
    int16_t  current      = 0;   //!< [mA]：トルク電流（推定電流）
    uint16_t lastAngleRaw = 0;   //!< 前回 angleRaw（内部用）
    int32_t  positionCnt  = 0;   //!< 多回転積算（angleRaw 単位）
};

// ============================================================================
//                            DjiMotorCan1 (CAN1)
// ----------------------------------------------------------------------------
/**
 * @class DjiMotorCan1
 * @brief FlexCAN_T4<CAN1> 専用モータドライバ
 */
class DjiMotorCan1 {
public:
    /**
     * @param bus 既に初期化済みの FlexCAN_T4<CAN1> 参照
     */
    explicit DjiMotorCan1(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &bus) : bus_(bus) {
        for (auto &f : txFrame_) f = CAN_message_t{};
        _self = this;
        bus_.begin();
        bus_.setBaudRate(1'000'000);
        bus_.enableFIFO();
        bus_.enableFIFOInterrupt();
        bus_.onReceive(isr);
    }

    /**
     * @brief 電流指令を内部バッファに設定（未送信）
     * @param motorId DJI プロトコル上の ID (1–8)
     * @param currentCmd mA 単位の電流指令（‑16384～+16384 程度）
     *
     * 4 台ごと（ID1‑4＝0x200、ID5‑8＝0x1FF）に 1 フレームへパックし、
     * `flush()` 呼び出し時にまとめて送信する。
     */
    void sendCurrent(uint8_t motorId, int16_t currentCmd) {
        if (motorId < 1 || motorId > 8) return;              // 範囲外
        const uint8_t group = (motorId - 1) / 4;             // 0 or 1
        const uint8_t slot  = (motorId - 1) % 4;             // 0–3

        CAN_message_t &frm = txFrame_[group];
        frm.id  = 0x200 - group;                        // 0x200 / 0x1FF
        frm.len = 8;
        // Big‑endian で 2 バイト格納
        frm.buf[slot * 2]     = currentCmd >> 8;
        frm.buf[slot * 2 + 1] = currentCmd & 0xFF;
    }

    /**
     * @brief 内部バッファに溜まったフレームを送信
     * @return すべて送信成功したら true
     */
    bool flush() {
        bool ok = true;
        for (auto &f : txFrame_) if (f.len) ok &= bus_.write(f);
        return ok;
    }

    static void isr(const CAN_message_t& msg) {
        if (_self) _self -> poll(msg);
    }

    void poll(const CAN_message_t &msg) {
        if (msg.id < 0x201 || msg.id > 0x208) return; // 非対象

        const uint8_t idx = msg.id - 0x201;                  // 0–7
        const uint16_t ang  = (msg.buf[0] << 8) | msg.buf[1];
        const int16_t  rpm  = (msg.buf[2] << 8) | msg.buf[3];
        const int16_t  curr = (msg.buf[4] << 8) | msg.buf[5];

        // 多回転位置積算（符号拡張した差分）
        int16_t delta = ang - fb_[idx].lastAngleRaw;
        if (delta > 4096)  delta -= 8192;                    // 正方向オーバフロー補正
        if (delta < -4096) delta += 8192;                    // 負方向オーバフロー補正
        fb_[idx].positionCnt += delta;
        fb_[idx].lastAngleRaw = ang;

        fb_[idx].angleRaw = ang;
        fb_[idx].speedRaw = rpm;
        fb_[idx].current  = curr;
    }

    /**
     * @brief 最新フィードバック参照
     */
    const DjiFeedback &feedback(uint8_t motorId) const { return fb_[motorId - 1]; }

private:
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &bus_;
    CAN_message_t txFrame_[2];   //!< 0x200 / 0x1FF 用
    DjiFeedback   fb_[8];        //!< ID1–8
    static DjiMotorCan1* _self;                  // ISR から参照
};

DjiMotorCan1* DjiMotorCan1::_self = nullptr;


// ============================================================================
//                            DjiMotorCan2 (CAN2)
// ----------------------------------------------------------------------------
/**
 * @class DjiMotorCan2
 * @brief FlexCAN_T4<CAN2> 専用モータドライバ（実装は CAN1 版と同一）
 */
class DjiMotorCan2 {
public:
    explicit DjiMotorCan2(FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &bus) : bus_(bus) {
        for (auto &f : txFrame_) f = CAN_message_t{};
        _self = this;
        bus_.begin();
        bus_.setBaudRate(1'000'000);
        bus_.enableFIFO();
        bus_.enableFIFOInterrupt();
        bus_.onReceive(isr);
    }

    void sendCurrent(uint8_t motorId, int16_t currentCmd) {
        if (motorId < 1 || motorId > 8) return;              // 範囲外
        const uint8_t group = (motorId - 1) / 4;             // 0 or 1
        const uint8_t slot  = (motorId - 1) % 4;             // 0–3

        CAN_message_t &frm = txFrame_[group];
        frm.id  = 0x200 - group;                        // 0x200 / 0x1FF
        frm.len = 8;
        // Big‑endian で 2 バイト格納
        frm.buf[slot * 2]     = currentCmd >> 8;
        frm.buf[slot * 2 + 1] = currentCmd & 0xFF;
    }

    bool flush() {
        bool ok = true;
        for (auto &f : txFrame_) if (f.len) ok &= bus_.write(f);
        return ok;
    }

    static void isr(const CAN_message_t& msg) {
        if (_self) _self -> poll(msg);
    }

    void poll(const CAN_message_t &msg) {
        if (msg.id < 0x201 || msg.id > 0x208) return; // 非対象
        const uint8_t idx = msg.id - 0x201;
        const uint16_t ang  = (msg.buf[0] << 8) | msg.buf[1];
        const int16_t  rpm  = (msg.buf[2] << 8) | msg.buf[3];
        const int16_t  curr = (msg.buf[4] << 8) | msg.buf[5];
        int16_t delta = ang - fb_[idx].lastAngleRaw;
        if (delta > 4096)  delta -= 8192;
        if (delta < -4096) delta += 8192;
        fb_[idx].positionCnt += delta;
        fb_[idx].lastAngleRaw = ang;
        fb_[idx].angleRaw     = ang;
        fb_[idx].speedRaw     = rpm;
        fb_[idx].current      = curr;
    }

    const DjiFeedback &feedback(uint8_t motorId) const { return fb_[motorId - 1]; }

private:
    FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &bus_;
    CAN_message_t txFrame_[2];
    DjiFeedback   fb_[8];
    static DjiMotorCan2* _self;                  // ISR から参照
};

DjiMotorCan2* DjiMotorCan2::_self = nullptr;


/* ========================================================================
 * Example (Arduino / Teensy 4.x)
 * ========================================================================
 *
 * #include <Arduino.h>
 * #include "DjiMotorCan.hpp"
 *
 * FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
 * FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
 *
 * DjiMotorCan1 motorCan1(can1);
 * DjiMotorCan2 motorCan2(can2);
 *
 * void setup() {
 *     Serial.begin(115200);
 *
 *     can1.begin();
 *     can1.setBaudRate(1000000);
 *     can2.begin();
 *     can2.setBaudRate(1000000);
 * }
 *
 * void loop() {
 *     // 1. 電流指令発行 --------------------------------------------------
 *     motorCan1.sendCurrent(1, 2000);  // CAN1, ID1 を +2A
 *     motorCan1.sendCurrent(2, -2000); // CAN1, ID2 を -2A
 *     motorCan2.sendCurrent(5, 1500);  // CAN2, ID5 を +1.5A
 *     motorCan1.flush();
 *     motorCan2.flush();
 *
 *     // 2. フィードバック受信 ------------------------------------------
 *     uint8_t id;
 *     while ((id = motorCan1.poll())) {
 *         const auto &fb = motorCan1.feedback(id);
 *         Serial.printf("[CAN1] ID%u pos=%ld rpm=%d mA=%d\n",
 *                       id, fb.positionCnt, fb.speedRaw, fb.current);
 *     }
 *     while ((id = motorCan2.poll())) {
 *         const auto &fb = motorCan2.feedback(id);
 *         Serial.printf("[CAN2] ID%u pos=%ld rpm=%d mA=%d\n",
 *                       id, fb.positionCnt, fb.speedRaw, fb.current);
 *     }
 * }
 */