#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <map>
#include <vector>
#include "esp_gap_ble_api.h"

class MyBLEController {
public:
  // コンストラクタ/デストラクタ（必要に応じて）
  MyBLEController();
  ~MyBLEController();

  // BLE初期化＆開始
  void begin(const char* deviceName);

  // もしloopで毎フレームやりたい処理があれば
  void update();

  // 受信した文字列などを取り出すためのアクセサ（例）
  std::array<std::array<int, 2>, 2> getLastReceived() const;  // 修正

  bool isDeviceConnected() const;

private:
  // 内部で使うコールバッククラスを前方宣言
  class ServerCallbacks;
  class CharacteristicCallbacks;

  // BLE関連
  BLEServer* pServer = nullptr;
  BLEService* pService = nullptr;
  BLECharacteristic* pCharacteristic = nullptr;

  // 接続フラグ＆接続先アドレス
  bool deviceConnected = false;
  esp_bd_addr_t currentRemoteBdAddr;

  // 初期値
  std::array<std::array<int, 2>, 2> lastReceived = {{{512, 512}, {512, 512}}};  // メモリ削減

  // 接続パラメータ更新要求
  void requestConnectionParamUpdate();

  // コールバックからアクセスしやすいよう、友達宣言
  friend class ServerCallbacks;
  friend class CharacteristicCallbacks;
};
