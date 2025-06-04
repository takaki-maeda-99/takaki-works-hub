#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include "myBLEController.h"

// -- コールバック実装用クラスの定義 --
class MyBLEController::ServerCallbacks : public BLEServerCallbacks {
public:
  ServerCallbacks(MyBLEController& ctrl) : controller(ctrl) {}

  void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) override {
    controller.deviceConnected = true;
    Serial.println("[ServerCallbacks] Device connected!");

    memcpy(controller.currentRemoteBdAddr, param->connect.remote_bda, sizeof(esp_bd_addr_t));
    controller.requestConnectionParamUpdate();
  }

  void onDisconnect(BLEServer* pServer) override {
    controller.deviceConnected = false;
    Serial.println("[ServerCallbacks] Device disconnected!");
  }

private:
  MyBLEController& controller;
};

class MyBLEController::CharacteristicCallbacks : public BLECharacteristicCallbacks {
public:
  CharacteristicCallbacks(MyBLEController& ctrl) : controller(ctrl) {}

  void onWrite(BLECharacteristic* characteristic) override {
    String val = characteristic->getValue();
    if (val.length() > 0) {
      val.replace(String((char)0x01), "");
      val.replace(String((char)0x02), "");
      val.replace(String((char)0x03), "");

      String cmd = val.substring(0, 2); // 例: "d1" または "d0"
      String coords = val.substring(2); // 例: "512,512"

      int key = cmd.substring(1).toInt(); // "d1" → 1, "d0" → 0

      int commaIndex = coords.indexOf(",");
      if (commaIndex != -1) {
        controller.lastReceived[key][0] = coords.substring(0, commaIndex).toInt();
        controller.lastReceived[key][1] = coords.substring(commaIndex + 1).toInt();
      }
    }
  }

private:
  MyBLEController& controller;
};

MyBLEController::MyBLEController() {
  deviceConnected = false;
}

MyBLEController::~MyBLEController() {}

void MyBLEController::begin(const char* deviceName) {
  Serial.println("[MyBLEController] begin()");

  BLEDevice::init(deviceName);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks(*this));

  pService = pServer->createService("12345678-0000-0000-0000-123456789abc");

  pCharacteristic = pService->createCharacteristic(
    "12345678-0000-0000-0000-abcdef123456",
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->setCallbacks(new CharacteristicCallbacks(*this));

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID("12345678-0000-0000-0000-123456789abc");
  pAdvertising->start();

  Serial.println("[MyBLEController] BLE server ready");
}

void MyBLEController::update() {
}

std::array<std::array<int, 2>, 2> MyBLEController::getLastReceived() const {
  return lastReceived;
}

bool MyBLEController::isDeviceConnected() const {
  return deviceConnected;
}

void MyBLEController::requestConnectionParamUpdate() {
  esp_ble_conn_update_params_t connParams = {0};
  memcpy(connParams.bda, currentRemoteBdAddr, sizeof(esp_bd_addr_t));

  connParams.min_int = 0x08;
  connParams.max_int = 0x10;
  connParams.latency = 0;
  connParams.timeout = 400;

  esp_err_t err = esp_ble_gap_update_conn_params(&connParams);
  if (err == ESP_OK) {
    Serial.println("[MyBLEController] Requested conn param update");
  } else {
    Serial.printf("[MyBLEController] Conn param update failed, err=0x%x\n", err);
  }
}
