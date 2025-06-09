#pragma once

#include <Eigen/Geometry>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

class Merger : public rclcpp::Node {
   public:
   Merger();

   private:
   void scan1Callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
   void scan2Callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
   void scan3Callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
   void lidarConditionCallback(const std_msgs::msg::Bool::SharedPtr msg);
   void timerCallback();

   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription2_;
   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription3_;
   rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_condition_subscription_;
   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
   rclcpp::TimerBase::SharedPtr timer_;

   laser_geometry::LaserProjection projector_;
   pcl::PointCloud<pcl::PointXYZ>::Ptr scan1_cloud_;
   pcl::PointCloud<pcl::PointXYZ>::Ptr scan2_cloud_;
   pcl::PointCloud<pcl::PointXYZ>::Ptr scan3_cloud_;

   geometry_msgs::msg::TransformStamped static_transform_;
   bool first_callback1_ = true;
   bool first_callback2_ = true;
   bool first_callback3_ = true;
   std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> frame_id_to_cloud_map_;

   int interval_ms_;
   std::string base_frame_id_;
   std::string second_frame_id_;
   std::string third_frame_id_;
   bool first_flip_x_;
   bool first_flip_y_;
   double second_translation_x_;
   double second_translation_y_;
   double second_rotation_;
   bool second_flip_x_;
   bool second_flip_y_;
   double third_translation_x_;
   double third_translation_y_;
   double third_rotation_;
   bool third_flip_x_;
   bool third_flip_y_;

   bool lidar_condition_ = false;
};
