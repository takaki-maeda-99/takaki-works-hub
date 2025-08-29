#pragma once
#include <FlexCAN_T4.h>
#include <cstdint>
#include <cstring>

/**
 * @struct DjiFeedback
 * @brief モータ1台分の最新フィードバック値を保持する構造体
 */
struct DjiFeedback {
    uint16_t angleRaw     = 0;   //!< 0–8191: 単回転エンコーダ値 (生の角度値)
    int16_t  speedRaw     = 0;   //!< [rpm] 軸の回転速度
    int16_t  current      = 0;   //!< [mA] トルク電流（推定値）
    uint16_t lastAngleRaw = 0;   //!< 前回取得したangleRaw（内部用）
    int32_t  positionCnt  = 0;   //!< 多回転積算角度（angleRaw単位）
};

/**
 * @class DjiMotorCan
 * @brief FlexCAN_T4 (CAN2.0) を用いた DJI インテリジェントモータ用ドライバのテンプレートクラス
 * @tparam CAN_BUS 使用するCANバスを表すテンプレートパラメータ（例: CAN1, CAN2 など）
 *
 * 本クラスは DjiMotorCan1/CAN2 の共通実装であり、テンプレート引数により CAN1, CAN2, CAN3 等に対応します。
 * - **型安全なCANバス区別:** テンプレート引数により異なるCANバス用インスタンスは異なる型となり、誤って別バスに送信することを防ぎます。
 * - **最大8台のモータへの電流指令一括送信:** 4台ごとにフレーム（ID 0x200 または 0x1FF）を組み立て、`flush()`で2フレーム以内にまとめて送信します。
 * - **多回転位置の自動積算:** フィードバック受信時に角度値(0–8191)の符号付差分を計算し、オーバーフローを補正しながら内部で多回転カウントを更新します。
 * - **ポーリングAPIのみ提供:** 割り込みISRによるコールバック内で内部状態を更新しますが、ユーザが扱うAPIはポーリング形式（`poll()`で受信処理）です。
 *
 * **使用例:**  
 * ```cpp
 * FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
 * FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
 * DjiMotorCan<CAN1> motorCan1(can1);
 * DjiMotorCan<CAN2> motorCan2(can2);
 * 
 * void setup() {
 *     Serial.begin(115200);
 *     // （※DjiMotorCan コンストラクタ内で canX.begin() や onReceive 登録を行うため、
 *     //    setup内で別途 can.begin() 等を呼ぶ必要はありません）
 * }
 * 
 * void loop() {
 *     // --- 1. 複数モータへの電流指令発行 -----------------------------
 *     motorCan1.sendCurrent(1,  2000);  // CAN1のID1に +2A 相当の指令
 *     motorCan1.sendCurrent(2, -2000);  // CAN1のID2に -2A
 *     motorCan2.sendCurrent(5,  1500);  // CAN2のID5に +1.5A
 *     motorCan1.flush();               // CAN1側の指令フレーム送信 (0x200)
 *     motorCan2.flush();               // CAN2側の指令フレーム送信 (0x1FF)
 * 
 *     // --- 2. フィードバック受信（ポーリング） ------------------------
 *     uint8_t id;
 *     while ((id = motorCan1.poll())) {
 *         const auto &fb = motorCan1.feedback(id);
 *         Serial.printf("[CAN1] ID%u: pos=%ld rpm=%d mA=%d\n",
 *                       id, fb.positionCnt, fb.speedRaw, fb.current);
 *     }
 *     while ((id = motorCan2.poll())) {
 *         const auto &fb = motorCan2.feedback(id);
 *         Serial.printf("[CAN2] ID%u: pos=%ld rpm=%d mA=%d\n",
 *                       id, fb.positionCnt, fb.speedRaw, fb.current);
 *     }
 * }
 * ```
 */
template<uint32_t CAN_BUS>
class DjiMotorCan {
public:
    /**
     * @brief コンストラクタ（指定したCANバスで初期化）
     * @param bus 既に作成済みの FlexCAN_T4<CAN_BUS, RX_SIZE_256, TX_SIZE_16> オブジェクトへの参照
     *
     * コンストラクタ内でCANバスの初期設定（begin/baud設定/FIFO設定/割り込み登録）を行います。  
     * ※注意: 本クラスのインスタンスは各CANバスにつき**一つ**にしてください。複数作成するとコールバック参照が上書きされます。
     */
    explicit DjiMotorCan(FlexCAN_T4<CAN_BUS, RX_SIZE_256, TX_SIZE_16> &bus) : bus_(bus) {
        // 送信フレームバッファ初期化
        for (auto &f : txFrame_) {
            f = CAN_message_t{};
        }
        // コールバック用の静的インスタンスポインタ設定
        _self = this;
        // CAN初期化とコールバック登録
        bus_.begin();
        bus_.setBaudRate(1'000'000);      // DJI推奨 1 Mbps
        bus_.enableFIFO();
        bus_.enableFIFOInterrupt();
        bus_.onReceive(isr);              // 受信割り込み時にisr関数を呼ぶ
    }

    /**
     * @brief モータへの電流指令を内部バッファに設定（即時送信はしない）
     * @param motorId モータID (DJIプロトコル上の ID: 1～8)
     * @param currentCmd 指令する電流値 [mA]（およそ -16384～+16384 の範囲）
     *
     * 同時に最大8台までのモータに電流指令を出せます。ID1-4はID0x200フレーム、ID5-8はID0x1FFフレームに4台分まとめてパックされます。  
     * 本関数を複数回呼んで各IDの指令値をセットした後、`flush()`を呼ぶことで実際にフレームが送信されます。
     */
    void sendCurrent(uint8_t motorId, int16_t currentCmd) {
        if (motorId < 1 || motorId > 8) return;  // ID範囲外は無視
        const uint8_t group = (motorId - 1) / 4;  // グループ（0: ID1-4→0x200, 1: ID5-8→0x1FF）
        const uint8_t slot  = (motorId - 1) % 4;  // グループ内のスロット(0～3番目)
        CAN_message_t &frm = txFrame_[group];
        frm.id  = 0x200 - group;                 // group=0 -> 0x200, group=1 -> 0x1FF
        frm.len = 8;
        // 2バイトの電流指令値をビッグエンディアンでフレームに格納
        frm.buf[slot * 2]     = static_cast<uint8_t>(currentCmd >> 8);
        frm.buf[slot * 2 + 1] = static_cast<uint8_t>(currentCmd & 0xFF);
    }

    /**
     * @brief 内部バッファに蓄えた電流指令フレームを送信
     * @return すべて送信成功した場合は true（送信バッファに空きがなく送れなかった場合 false）
     *
     * `sendCurrent()`で設定されたフレーム（最大2フレーム）を送信します。送信後もバッファ内には前回値が残るため、必要なら次回送信前に上書き可能です。
     */
    bool flush() {
        bool ok = true;
        for (auto &f : txFrame_) {
            if (f.len != 0) {              // バッファに有効なデータがある場合のみ送信
                ok &= bus_.write(f);
                // ※必要であればここで f.len=0 などリセット（今回は次回上書きするので省略）
            }
        }
        return ok;
    }

    /**
     * @brief 受信フィードバックのポーリング処理
     * @return 受信処理を行ったモータID（キューが空の場合0）
     *
     * 内部でCANバスから次のメッセージを1つ取得し（FIFO経由）、`poll(const CAN_message_t&)`を用いて処理します。  
     * 処理されたメッセージが該当するモータID(1～8)を返すので、それを用いて `feedback(id)` で各値を取得できます。受信すべきメッセージが無ければ0を返します。
     * 
     * この関数をループ内で呼び出し、0を返すまで処理することで、受信バッファ内のすべてのフィードバックを処理できます。
     */
    uint8_t poll() {
        CAN_message_t msg;
        if (bus_.read(msg)) {              // FIFOからメッセージを1つ取得（メッセージがなければfalse）
            poll(msg);                    // メッセージ内容を処理して内部フィードバック更新
            if (msg.id >= 0x201 && msg.id <= 0x208) {
                return static_cast<uint8_t>(msg.id - 0x200);  // モータIDを算出 (0x201→ID1, ... 0x208→ID8)
            }
        }
        return 0;
    }

    /**
     * @brief 最新のフィードバックデータを取得
     * @param motorId モータID (1～8)
     * @return 指定したモータのフィードバック構造体（参照）
     *
     * フィードバック構造体にはangleRawやpositionCntなどの最新値が格納されています。  
     * ※この関数は`poll()`で新しいデータを処理した後に使用してください（それまでは古い値のままです）。
     */
    const DjiFeedback& feedback(uint8_t motorId) const {
        return fb_[motorId - 1];
    }

    /**
     * @brief モータ角度のリセット（原点復帰用）
     * @param motorId モータID (1～8)
     * @param resetAngleDeg リセット後の角度 [deg]
     *
     * 指定したモータの現在位置を任意の角度としてリセットします。この関数呼び出し後、該当モータのpositionCntが指定角度相当のカウント値に変更されます。  
     * （内部的には現在のangleRawを基準に、多回転カウント値に所定のオフセットを加えて実現します）
     */
    void resetAngle(uint8_t motorId, float resetAngleDeg) {
        if (motorId < 1 || motorId > 8) return;
        const uint8_t idx = motorId - 1;
        // 角度[deg]をエンコーダのカウント値に換算 (1回転=8192カウント, 外部ギア比72:1想定)
        constexpr float CNT2DEG = 360.0f / (8192.0f * 72.0f);
        const int32_t resetCnt = static_cast<int32_t>(resetAngleDeg / CNT2DEG);
        // 現在のエンコーダ生値を基準に積算位置カウントを調整
        fb_[idx].positionCnt = resetCnt;
        fb_[idx].lastAngleRaw = fb_[idx].angleRaw;
    }

private:
    // 受信割り込み（FIFO受信）時に呼ばれるコールバック関数（静的）
    static void isr(const CAN_message_t &msg) {
        if (_self) {
            _self->poll(msg);  // 受信メッセージを処理
        }
    }

    // 受信メッセージを処理してフィードバック更新（内部用）
    void poll(const CAN_message_t &msg) {
        if (msg.id < 0x201 || msg.id > 0x208) return;  // 対象外IDは無視
        const uint8_t idx = msg.id - 0x201;            // インデックス 0～7
        // メッセージから各値を取り出し
        const uint16_t ang  = (msg.buf[0] << 8) | msg.buf[1];
        const int16_t  rpm  = (static_cast<int16_t>(msg.buf[2] << 8)) | msg.buf[3];
        const int16_t  curr = (static_cast<int16_t>(msg.buf[4] << 8)) | msg.buf[5];
        // 多回転位置積算: 前回との差分を計算（±4096を超える差分はオーバーフローとみなして補正）
        int16_t delta = ang - fb_[idx].lastAngleRaw;
        if (delta >  4096) delta -= 8192;  // 正方向に一周超えた（差分をマイナス補正）
        if (delta < -4096) delta += 8192;  // 負方向に一周超えた（差分をプラス補正）
        fb_[idx].positionCnt  += delta;
        fb_[idx].lastAngleRaw  = ang;
        // 構造体に格納
        fb_[idx].angleRaw  = ang;
        fb_[idx].speedRaw  = rpm;
        fb_[idx].current   = curr;
    }

    FlexCAN_T4<CAN_BUS, RX_SIZE_256, TX_SIZE_16> &bus_;  //!< 使用するFlexCANバス（テンプレート引数で決定）
    CAN_message_t txFrame_[2];     //!< 電流指令フレームバッファ (0:ID0x200用, 1:ID0x1FF用)
    DjiFeedback   fb_[8];          //!< モータID1～8 のフィードバックデータ配列
    static DjiMotorCan<CAN_BUS>* _self;  //!< 自身のインスタンス（静的）※割り込みハンドラから参照
};

// 静的メンバの実体をテンプレートごとに定義
template<uint32_t CAN_BUS>
DjiMotorCan<CAN_BUS>* DjiMotorCan<CAN_BUS>::_self = nullptr;
