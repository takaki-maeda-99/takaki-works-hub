#include "detector/visualizer.h"

Visualizer::Visualizer() : Node("visualizer") {
   subscription_ = this->create_subscription<obstacle_msgs::msg::Obstacles>("/obstacles", 1, std::bind(&Visualizer::obstaclesCallback, this, std::placeholders::_1));
   publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/obstacle_visualizations", 1);
}

void Visualizer::obstaclesCallback(const obstacle_msgs::msg::Obstacles::SharedPtr msg) {
   auto markers = std::make_shared<visualization_msgs::msg::MarkerArray>();

   max_line_segments_size_ = std::max(max_line_segments_size_, msg->line_segments.size());
   max_circles_size_ = std::max(max_circles_size_, msg->circles.size());

   size_t i = 0;
   for (const auto &line_segment : msg->line_segments) {
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = msg->header.frame_id;
      marker.header.stamp = msg->header.stamp;
      marker.ns = "line_segment";
      marker.id = i++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.01;
      marker.color.g = 1.0;
      marker.color.a = 1.0;

      marker.points.push_back(line_segment.start);
      marker.points.push_back(line_segment.end);

      markers->markers.push_back(marker);
   }
   for (; i < max_line_segments_size_; ++i) {
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = msg->header.frame_id;
      marker.header.stamp = msg->header.stamp;
      marker.ns = "line_segment";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::DELETE;
      markers->markers.push_back(marker);
   }

   i = 0;
   for (const auto &circle : msg->circles) {
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = msg->header.frame_id;
      marker.header.stamp = msg->header.stamp;
      marker.ns = "circle";
      marker.id = i++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.01;
      marker.color.b = 1.0;
      marker.color.a = 1.0;

      marker.pose.position = circle.center;
      marker.scale.x = marker.scale.y = marker.scale.z = 2 * circle.radius;

      markers->markers.push_back(marker);
   }
   for (; i < max_circles_size_; ++i) {
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = msg->header.frame_id;
      marker.header.stamp = msg->header.stamp;
      marker.ns = "circle";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::DELETE;
      markers->markers.push_back(marker);
   }

   publisher_->publish(*markers);
}
