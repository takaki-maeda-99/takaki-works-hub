#pragma once

#include <optional>

#include <Eigen/Dense>

#include <obstacle_msgs/msg/circle.hpp>
#include <obstacle_msgs/msg/line_segment.hpp>
#include <obstacle_msgs/msg/obstacles.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "clusterer.h"

struct CircleParams {
   double center_x;
   double center_y;
   double radius;
};

struct LineParams {
   double slope;
   double intercept;
};

class Detector : public rclcpp::Node {
   public:
   Detector();

   private:
   void pcCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
   void lidarConditionCallback(const std_msgs::msg::Bool::SharedPtr msg);

   double calcRSSFromCircle(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud, const CircleParams &circle_params);
   double calcRSSFromLine(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud, const LineParams &line_params);
   std::optional<CircleParams> fitCircle(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);
   std::optional<LineParams> fitLine(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);

   pcl::Normal estimateNormal(const pcl::search::KdTree<pcl::PointXYZ>::Ptr &kdtree, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointXYZ &point, double radius);
   pcl::PointCloud<pcl::PointNormal>::Ptr estimateNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double radius);

   rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_condition_subscription_;
   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
   rclcpp::Publisher<obstacle_msgs::msg::Obstacles>::SharedPtr publisher_;
   Clusterer clusterer_;

   double cluster_tolerance_;
   int min_cluster_size_;
   double normal_estimation_radius_;
   double min_radius_;
   double max_radius_;
   double dot_threshold_for_line_;
   double dot_threshold_for_circle_;
   double normal_angle_threshold_;

   bool lidar_condition_ = false;
};
