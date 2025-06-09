#pragma once

#include <obstacle_msgs/msg/obstacles.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class Visualizer : public rclcpp::Node {
   public:
   Visualizer();

   private:
   void obstaclesCallback(const obstacle_msgs::msg::Obstacles::SharedPtr msg);

   rclcpp::Subscription<obstacle_msgs::msg::Obstacles>::SharedPtr subscription_;
   rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;

   size_t max_line_segments_size_ = 0;
   size_t max_circles_size_ = 0;
};
