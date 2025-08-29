#include "../src/can.hpp"

void onCANMessage(const CANMessage& msg, uint8_t canPort) {
    Serial.print("CAN");
    Serial.print(canPort);
    Serial.print(" Received ID: 0x");
    Serial.print(msg.id, HEX);
    Serial.print(" Data: ");
    
    for (int i = 0; i < msg.len; i++) {
        if (msg.data[i] < 0x10) Serial.print("0");
        Serial.print(msg.data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("CAN Library Example");
    
    if (canBus.begin(CANBus::CAN_PORT_1, CANBus::BAUD_500K)) {
        Serial.println("CAN1 initialized successfully");
    } else {
        Serial.println("CAN1 initialization failed");
        return;
    }
    
    canBus.setCallback(onCANMessage);
    
    Serial.println("CAN example ready");
}

void loop() {
    static unsigned long lastSend = 0;
    static uint32_t counter = 0;
    
    canBus.update();
    
    if (millis() - lastSend > 1000) {
        uint8_t data[8];
        data[0] = (counter >> 24) & 0xFF;
        data[1] = (counter >> 16) & 0xFF;
        data[2] = (counter >> 8) & 0xFF;
        data[3] = counter & 0xFF;
        data[4] = 0xAA;
        data[5] = 0xBB;
        data[6] = 0xCC;
        data[7] = 0xDD;
        
        if (canBus.send(CANBus::CAN_PORT_1, 0x123, data, 8)) {
            Serial.print("Sent message ");
            Serial.println(counter);
        } else {
            Serial.println("Failed to send message");
        }
        
        counter++;
        lastSend = millis();
    }
    
    delay(10);
}

void demoFunctions() {
    canBus.begin(CANBus::CAN_PORT_2, CANBus::BAUD_250K);
    
    canBus.setBaudRate(CANBus::CAN_PORT_1, CANBus::BAUD_1M);
    
    canBus.setFilter(CANBus::CAN_PORT_1, 0x100, 0x700);
    
    canBus.enableLoopback(CANBus::CAN_PORT_1, true);
    
    canBus.enableSilentMode(CANBus::CAN_PORT_1, true);
    
    uint32_t errors = canBus.getErrorCount(CANBus::CAN_PORT_1);
    bool isActive = canBus.isErrorActive(CANBus::CAN_PORT_1);
    
    canBus.clearErrors(CANBus::CAN_PORT_1);
    
    CANMessage msg(0x456, nullptr, 0, true);
    msg.data[0] = 0x12;
    msg.data[1] = 0x34;
    msg.len = 2;
    canBus.send(CANBus::CAN_PORT_1, msg);
    
    CANMessage receivedMsg;
    if (canBus.read(CANBus::CAN_PORT_1, receivedMsg)) {
        Serial.println("Message received via polling");
    }
}