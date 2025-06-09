#pragma once

#include <obstacle_msgs/msg/obstacles.hpp>
#include <obstacle_msgs/msg/wall.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class Filter : public rclcpp::Node {
   public:
   Filter();

   private:
   void obstaclesCallback(const obstacle_msgs::msg::Obstacles::SharedPtr msg);
   void lidarConditionCallback(const std_msgs::msg::Bool::SharedPtr msg);
   double calculate_angle_between_lines(double slope1, double slope2);
   double calculate_direction_from_origin(double slope, double intercept);
   double calculate_distance_with_two_line_segments(const obstacle_msgs::msg::LineSegment &line_segment1, const obstacle_msgs::msg::LineSegment &line_segment2);

   rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_condition_subscription_;
   rclcpp::Subscription<obstacle_msgs::msg::Obstacles>::SharedPtr subscription_;
   rclcpp::Publisher<obstacle_msgs::msg::Obstacles>::SharedPtr publisher_;

   int counted_frames_ = 0;
   std::vector<obstacle_msgs::msg::Circle> circles_;
   std::vector<int> circle_counts_;

   int count_duration_;
   int count_threshold_;
   float parallel_angle_threshold_;
   float perpendicular_angle_threshold_;
   float distance_threshold_;

   bool lidar_condition_ = false;
};
