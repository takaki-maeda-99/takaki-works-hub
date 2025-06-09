#include <Arduino.h>
#include <MsgPack.h>
#include <MsgPacketizer.h>

#define BAUDRATE 115200

struct Circle {
   float center_x, center_y, radius;
   MSGPACK_DEFINE(center_x, center_y, radius);
};

struct Wall {
   float direction, distance;
   MSGPACK_DEFINE(direction, distance);
};

struct BallCamera {
   int center_x, center_y, color;
   float confidence;
   MSGPACK_DEFINE(center_x, center_y, color, confidence);
};

struct SerialData {
   MsgPack::fix_arr_t<Wall, 4> walls;
   Circle nearest_circle;
   BallCamera ball_camera;
   bool lidar_condition, camera_condition;
   MSGPACK_DEFINE(walls, nearest_circle, ball_camera, lidar_condition, camera_condition);
};

SerialData rx_msg;
SerialData tx_msg;

static constexpr float PUBLISH_RATE{20.f}; // 20 Hz

void setup() {
   Serial.begin(BAUDRATE);
   delay(2000);

   MsgPacketizer::subscribe(Serial, 0x01, rx_msg);
   MsgPacketizer::publish(Serial, 0x02, tx_msg)->setFrameRate(PUBLISH_RATE);
}

void loop() {
   tx_msg.nearest_circle.center_x = rx_msg.nearest_circle.center_x;
   tx_msg.nearest_circle.center_y = rx_msg.nearest_circle.center_y;
   tx_msg.nearest_circle.radius = rx_msg.nearest_circle.radius;
   for (int i = 0; i < 4; i++) {
      tx_msg.walls[i].direction = rx_msg.walls[i].direction;
      tx_msg.walls[i].distance = rx_msg.walls[i].distance;
   }
   tx_msg.ball_camera.center_x = rx_msg.ball_camera.center_x;
   tx_msg.ball_camera.center_y = rx_msg.ball_camera.center_y;
   tx_msg.ball_camera.color = rx_msg.ball_camera.color;
   tx_msg.ball_camera.confidence = rx_msg.ball_camera.confidence;

   MsgPacketizer::update();

   delay(5);
}
