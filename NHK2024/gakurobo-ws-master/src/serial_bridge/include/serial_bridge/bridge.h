#pragma once

#include <array>

#include <ball_camera_msgs/msg/balls.hpp>
#include <obstacle_msgs/msg/obstacles.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/bool.hpp>

#include "MsgPacketizer/MsgPacketizer.h"

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

class Bridge : public rclcpp::Node {
   public:
   Bridge();

   private:
   void obstaclesCallback(const obstacle_msgs::msg::Obstacles::SharedPtr msg);
   void ballCameraCallback(const ball_camera_msgs::msg::Balls::SharedPtr msg);
   void timerCallback();

   rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lidar_condition_publisher_;
   rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr camera_condition_publisher_;
   rclcpp::Subscription<obstacle_msgs::msg::Obstacles>::SharedPtr obstacles_subscription_;
   rclcpp::Subscription<ball_camera_msgs::msg::Balls>::SharedPtr ball_camera_subscription_;
   rclcpp::TimerBase::SharedPtr timer_;

   serial::Serial serial_;
   SerialData tx_msg_;
   SerialData rx_msg_;

   std::string port_;
   int baudrate_;
   bool debug_;
   int interval_ms_;
};
