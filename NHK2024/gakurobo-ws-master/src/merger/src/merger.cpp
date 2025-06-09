#include "merger/merger.h"

Merger::Merger() : Node("merger") {
   subscription1_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan_1st", 1, std::bind(&Merger::scan1Callback, this, std::placeholders::_1));
   subscription2_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan_2nd", 1, std::bind(&Merger::scan2Callback, this, std::placeholders::_1));
   subscription3_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan_3rd", 1, std::bind(&Merger::scan3Callback, this, std::placeholders::_1));
   lidar_condition_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/lidar_condition", 1, std::bind(&Merger::lidarConditionCallback, this, std::placeholders::_1));
   publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_pc2", 1);

   interval_ms_ = declare_parameter("interval_ms", 100);
   base_frame_id_ = declare_parameter("base_frame_id", "laser");
   second_frame_id_ = declare_parameter("second_frame_id", "laser_2nd");
   third_frame_id_ = declare_parameter("third_frame_id", "laser_3rd");
   first_flip_x_ = declare_parameter("first_flip_x", false);
   first_flip_y_ = declare_parameter("first_flip_y", false);
   second_translation_x_ = declare_parameter("second_translation_x", 0.0);
   second_translation_y_ = declare_parameter("second_translation_y", 0.0);
   second_rotation_ = declare_parameter("second_rotation", 0.0);
   second_flip_x_ = declare_parameter("second_flip_x", false);
   second_flip_y_ = declare_parameter("second_flip_y", false);
   third_translation_x_ = declare_parameter("third_translation_x", 0.0);
   third_translation_y_ = declare_parameter("third_translation_y", 0.0);
   third_rotation_ = declare_parameter("third_rotation", 0.0);
   third_flip_x_ = declare_parameter("third_flip_x", false);
   third_flip_y_ = declare_parameter("third_flip_y", false);

   timer_ = this->create_wall_timer(std::chrono::milliseconds(interval_ms_), std::bind(&Merger::timerCallback, this));

   scan1_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
   scan2_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
   scan3_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
}

void Merger::scan1Callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
   if (!lidar_condition_) {
      return;
   }

   sensor_msgs::msg::PointCloud2 cloud_msg;
   projector_.projectLaser(*msg, cloud_msg);
   pcl::fromROSMsg(cloud_msg, *scan1_cloud_);

   if (first_flip_x_ || first_flip_y_) {
      Eigen::Affine3f transform = Eigen::Affine3f::Identity();

      if (first_flip_x_) transform(0, 0) = -1.0f;
      if (first_flip_y_) transform(1, 1) = -1.0f;

      pcl::transformPointCloud(*scan1_cloud_, *scan1_cloud_, transform);
   }

   if (first_callback1_) {
      frame_id_to_cloud_map_[msg->header.frame_id] = scan1_cloud_;
      first_callback1_ = false;
   }
}

void Merger::scan2Callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
   if (!lidar_condition_) {
      return;
   }

   sensor_msgs::msg::PointCloud2 cloud_msg;
   projector_.projectLaser(*msg, cloud_msg);
   pcl::fromROSMsg(cloud_msg, *scan2_cloud_);

   if (second_flip_x_ || second_flip_y_) {
      Eigen::Affine3f transform = Eigen::Affine3f::Identity();

      if (second_flip_x_) transform(0, 0) = -1.0f;
      if (second_flip_y_) transform(1, 1) = -1.0f;

      pcl::transformPointCloud(*scan2_cloud_, *scan2_cloud_, transform);
   }

   if (first_callback2_) {
      frame_id_to_cloud_map_[msg->header.frame_id] = scan2_cloud_;
      first_callback2_ = false;
   }
}

void Merger::scan3Callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
   if (!lidar_condition_) {
      return;
   }

   sensor_msgs::msg::PointCloud2 cloud_msg;
   projector_.projectLaser(*msg, cloud_msg);
   pcl::fromROSMsg(cloud_msg, *scan3_cloud_);

   if (third_flip_x_ || third_flip_y_) {
      Eigen::Affine3f transform = Eigen::Affine3f::Identity();

      if (third_flip_x_) transform(0, 0) = -1.0f;
      if (third_flip_y_) transform(1, 1) = -1.0f;

      pcl::transformPointCloud(*scan3_cloud_, *scan3_cloud_, transform);
   }

   if (first_callback3_) {
      frame_id_to_cloud_map_[msg->header.frame_id] = scan3_cloud_;
      first_callback3_ = false;
   }
}

void Merger::lidarConditionCallback(const std_msgs::msg::Bool::SharedPtr msg) {
   lidar_condition_ = msg->data;
}

void Merger::timerCallback() {
   if (!lidar_condition_) {
      return;
   }

   pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud2 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
   pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud3 = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

   if (first_callback1_ || first_callback2_) {
      return;
   }

   auto base_cloud = frame_id_to_cloud_map_[base_frame_id_];
   auto second_cloud = frame_id_to_cloud_map_[second_frame_id_];
   auto third_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

   Eigen::Affine3f transformation2 = pcl::getTransformation(second_translation_x_, second_translation_y_, 0.0, 0.0, 0.0, second_rotation_);
   pcl::transformPointCloud(*second_cloud, *transformed_cloud2, transformation2);

   if (!first_callback3_) {
      third_cloud = frame_id_to_cloud_map_[third_frame_id_];
      Eigen::Affine3f transformation3 = pcl::getTransformation(third_translation_x_, third_translation_y_, 0.0, 0.0, 0.0, third_rotation_);
      pcl::transformPointCloud(*third_cloud, *transformed_cloud3, transformation3);
   }

   *transformed_cloud2 += *base_cloud;
   *transformed_cloud2 += *transformed_cloud3;

   sensor_msgs::msg::PointCloud2 merged_cloud_msg;
   pcl::toROSMsg(*transformed_cloud2, merged_cloud_msg);
   merged_cloud_msg.header.frame_id = base_frame_id_;
   publisher_->publish(merged_cloud_msg);
}

int main(int argc, char *argv[]) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<Merger>());
   rclcpp::shutdown();
   return 0;
}
