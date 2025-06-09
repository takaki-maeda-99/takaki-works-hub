#include "detector/filter.h"

Filter::Filter() : Node("filter") {
   lidar_condition_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "/lidar_condition", 1, std::bind(&Filter::lidarConditionCallback, this, std::placeholders::_1));
   subscription_ = this->create_subscription<obstacle_msgs::msg::Obstacles>(
      "obstacles", 10, std::bind(&Filter::obstaclesCallback, this, std::placeholders::_1));
   publisher_ = this->create_publisher<obstacle_msgs::msg::Obstacles>("/filtered_obstacles", 10);

   count_duration_ = declare_parameter("count_duration", 10);
   count_threshold_ = declare_parameter("count_threshold", 5);
   parallel_angle_threshold_ = declare_parameter("parallel_angle_threshold", 0.1);
   perpendicular_angle_threshold_ = declare_parameter("perpendicular_angle_threshold", 1.0);
   distance_threshold_ = declare_parameter("distance_threshold", 0.1);
}

// TODO: Refactor !!!!!!!!

void Filter::obstaclesCallback(const obstacle_msgs::msg::Obstacles::SharedPtr msg) {
   if (!lidar_condition_) {
      return;
   }

   counted_frames_++;

   // TODO: とりあえず円のみフィルタリング.

   for (const auto &circle : msg->circles) {
      if (circle.center.x > 0) {
         continue;
      }
      bool found = false;
      for (size_t i = 0; i < circles_.size(); i++) {
         if (circle.center.x - circles_[i].center.x < 0.1 && circle.center.y - circles_[i].center.y < 0.1) {
            circle_counts_[i]++;
            found = true;
            break;
         }
      }
      if (!found) {
         circles_.push_back(circle);
         circle_counts_.push_back(1);
      }
   }

   auto filtered_circles = std::vector<obstacle_msgs::msg::Circle>();
   if (counted_frames_ >= count_duration_) {
      for (size_t i = 0; i < circles_.size(); i++) {
         if (circle_counts_[i] >= count_threshold_) {
            filtered_circles.push_back(circles_[i]);
         }
      }
   }

   auto obstacles = obstacle_msgs::msg::Obstacles();

   if (!filtered_circles.empty()) {
      auto nearest_circle = filtered_circles[0];
      auto nearest_squared_distance = std::pow(nearest_circle.center.x, 2) + std::pow(nearest_circle.center.y, 2);
      for (const auto &circle : filtered_circles) {
         auto squared_distance = std::pow(circle.center.x, 2) + std::pow(circle.center.y, 2);
         if (squared_distance < nearest_squared_distance) {
            nearest_circle = circle;
            nearest_squared_distance = squared_distance;
         }
      }
      obstacles.circles.push_back(nearest_circle);
   }

   if (!msg->line_segments.empty()) {

      std::vector<obstacle_msgs::msg::LineSegment> line_segments;
      std::vector<float> line_segment_lengths, slopes, intercepts;
      for (size_t i = 0; i < msg->line_segments.size(); i++) {
         auto line_segment = msg->line_segments[i];
         auto length = std::pow(line_segment.end.x - line_segment.start.x, 2) + std::pow(line_segment.end.y - line_segment.start.y, 2);
         if (line_segment.end.x - line_segment.start.x == 0) {
            continue;
         }
         slopes.push_back((line_segment.end.y - line_segment.start.y) / (line_segment.end.x - line_segment.start.x));
         intercepts.push_back(line_segment.start.y - slopes.back() * line_segment.start.x);
         line_segment_lengths.push_back(length);
         line_segments.push_back(line_segment);
      }

      std::vector<size_t> sorted_indices(line_segment_lengths.size());
      std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
      std::sort(sorted_indices.begin(), sorted_indices.end(), [&line_segment_lengths](size_t i1, size_t i2) {
         return line_segment_lengths[i1] > line_segment_lengths[i2];
      });

      float longest_length = line_segment_lengths[sorted_indices[0]];
      size_t longest_index = sorted_indices[0];

      int parallel_with_longest_index = -1;
      for (size_t i = 0; i < sorted_indices.size(); i++) {
         if (i == 0) {
            continue;
         }
         double angle = calculate_angle_between_lines(slopes[longest_index], slopes[sorted_indices[i]]);
         if (angle < parallel_angle_threshold_ && calculate_distance_with_two_line_segments(line_segments[longest_index], line_segments[sorted_indices[i]]) > distance_threshold_) {
            parallel_with_longest_index = i;
            break;
         }
      }

      int perpendicular_with_longest_index = -1;
      for (size_t i = 0; i < sorted_indices.size(); i++) {
         if (i == 0 || i == parallel_with_longest_index) {
            continue;
         }
         double angle = calculate_angle_between_lines(slopes[longest_index], slopes[sorted_indices[i]]);
         if (angle > perpendicular_angle_threshold_) {
            perpendicular_with_longest_index = i;
            break;
         }
      }

      int parallel_with_perpendicular_index = -1;
      if (perpendicular_with_longest_index != -1) {
         for (size_t i = 0; i < sorted_indices.size(); i++) {
            if (i == 0 || i == parallel_with_longest_index || i == perpendicular_with_longest_index) {
               continue;
            }
            double angle = calculate_angle_between_lines(slopes[sorted_indices[perpendicular_with_longest_index]], slopes[sorted_indices[i]]);
            if (angle < parallel_angle_threshold_ && calculate_distance_with_two_line_segments(line_segments[sorted_indices[perpendicular_with_longest_index]], line_segments[sorted_indices[i]]) > distance_threshold_) {
               parallel_with_perpendicular_index = i;
               break;
            }
         }
      }

      // line_segments to wall
      obstacle_msgs::msg::Wall longest_wall;
      longest_wall.distance = std::abs(intercepts[longest_index]) / std::sqrt(1 + std::pow(slopes[longest_index], 2));
      longest_wall.angle = calculate_direction_from_origin(slopes[longest_index], intercepts[longest_index]);
      obstacles.walls.push_back(longest_wall);

      if (parallel_with_longest_index != -1) {
         obstacle_msgs::msg::Wall parallel_wall;
         parallel_wall.distance = std::abs(intercepts[sorted_indices[parallel_with_longest_index]]) / std::sqrt(1 + std::pow(slopes[sorted_indices[parallel_with_longest_index]], 2));
         parallel_wall.angle = calculate_direction_from_origin(slopes[sorted_indices[parallel_with_longest_index]], intercepts[sorted_indices[parallel_with_longest_index]]);
         obstacles.walls.push_back(parallel_wall);
      }

      if (perpendicular_with_longest_index != -1) {
         obstacle_msgs::msg::Wall perpendicular_wall;
         perpendicular_wall.distance = std::abs(intercepts[sorted_indices[perpendicular_with_longest_index]]) / std::sqrt(1 + std::pow(slopes[sorted_indices[perpendicular_with_longest_index]], 2));
         perpendicular_wall.angle = calculate_direction_from_origin(slopes[sorted_indices[perpendicular_with_longest_index]], intercepts[sorted_indices[perpendicular_with_longest_index]]);
         obstacles.walls.push_back(perpendicular_wall);
      }

      if (parallel_with_perpendicular_index != -1) {
         obstacle_msgs::msg::Wall parallel_wall;
         parallel_wall.distance = std::abs(intercepts[sorted_indices[parallel_with_perpendicular_index]]) / std::sqrt(1 + std::pow(slopes[sorted_indices[parallel_with_perpendicular_index]], 2));
         parallel_wall.angle = calculate_direction_from_origin(slopes[sorted_indices[parallel_with_perpendicular_index]], intercepts[sorted_indices[parallel_with_perpendicular_index]]);
         obstacles.walls.push_back(parallel_wall);
      }

      obstacles.line_segments.push_back(line_segments[longest_index]);
      if (parallel_with_longest_index != -1)
         obstacles.line_segments.push_back(line_segments[sorted_indices[parallel_with_longest_index]]);
      if (perpendicular_with_longest_index != -1)
         obstacles.line_segments.push_back(line_segments[sorted_indices[perpendicular_with_longest_index]]);
      if (parallel_with_perpendicular_index != -1)
         obstacles.line_segments.push_back(line_segments[sorted_indices[parallel_with_perpendicular_index]]);
   }

   obstacles.header = msg->header;
   publisher_->publish(obstacles);

   if (counted_frames_ >= count_duration_) {
      counted_frames_ = 0;
      circles_.clear();
      circle_counts_.clear();
   }
}

void Filter::lidarConditionCallback(const std_msgs::msg::Bool::SharedPtr msg) {
   lidar_condition_ = msg->data;
}

double Filter::calculate_angle_between_lines(double slope1, double slope2) {
   if (slope1 == slope2) {
      return 0;
   }
   double angle_rad = std::atan(std::abs((slope2 - slope1) / (1 + slope1 * slope2)));
   double angle_deg = std::fabs(angle_rad * 180 / M_PI);
   return angle_deg;
}

double Filter::calculate_direction_from_origin(double slope, double intercept) {
   // 原点からみてどの方向にあるか. すなわち, 直線に下した垂線とx軸のなす角.
   if (slope == 0) {
      return intercept > 0 ? 90 : 270;
   }
   double perpendicular_slope = -1 / slope;
   double cross_x = -intercept / (slope - perpendicular_slope);
   double cross_y = slope * cross_x + intercept;
   double angle_rad = std::atan(perpendicular_slope);
   if (angle_rad < 0) {
      angle_rad += 2 * M_PI;
   }
   double angle_deg = angle_rad * 180 / M_PI;
   if ((perpendicular_slope > 0 && cross_y > 0) || (perpendicular_slope < 0 && cross_y < 0)) {
      angle_deg += 180;
   }
   if (angle_deg >= 360) {
      angle_deg -= 360;
   }
   if (angle_deg < 0) {
      angle_deg += 360;
   }
   return angle_deg;
}

double Filter::calculate_distance_with_two_line_segments(const obstacle_msgs::msg::LineSegment &line_segment1, const obstacle_msgs::msg::LineSegment &line_segment2) {
   double min_x1, min_y1, max_x1, max_y1, min_x2, min_y2, max_x2, max_y2;
   double slope1 = (line_segment1.end.y - line_segment1.start.y) / (line_segment1.end.x - line_segment1.start.x);
   if (line_segment1.start.x < line_segment1.end.x) {
      min_x1 = line_segment1.start.x;
      max_x1 = line_segment1.end.x;
      min_y1 = line_segment1.start.y;
      max_y1 = line_segment1.end.y;
   } else {
      min_x1 = line_segment1.end.x;
      max_x1 = line_segment1.start.x;
      min_y1 = line_segment1.end.y;
      max_y1 = line_segment1.start.y;
   }
   if (line_segment2.start.x < line_segment2.end.x) {
      min_x2 = line_segment2.start.x;
      max_x2 = line_segment2.end.x;
      min_y2 = line_segment2.start.y;
      max_y2 = line_segment2.end.y;
   } else {
      min_x2 = line_segment2.end.x;
      max_x2 = line_segment2.start.x;
      min_y2 = line_segment2.end.y;
      max_y2 = line_segment2.start.y;
   }
   if (std::abs(std::atan(slope1)) > 0.785) {
      return std::max(std::abs(max_x1 - max_x2), std::abs(min_x1 - min_x2));
   } else {
      return std::max(std::abs(max_y1 - max_y2), std::abs(min_y1 - min_y2));
   }
}
