#include "detector/detector.h"

Detector::Detector() : Node("detector") {
   lidar_condition_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/lidar_condition", 1, std::bind(&Detector::lidarConditionCallback, this, std::placeholders::_1));
   subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/pc2", 1, std::bind(&Detector::pcCallback, this, std::placeholders::_1));
   publisher_ = this->create_publisher<obstacle_msgs::msg::Obstacles>("/obstacles", 1);

   cluster_tolerance_ = declare_parameter("cluster_tolerance", 0.1);
   min_cluster_size_ = declare_parameter("min_cluster_size", 100);
   normal_estimation_radius_ = declare_parameter("normal_estimation_radius", 0.1);
   min_radius_ = declare_parameter("min_radius", 0.1);
   max_radius_ = declare_parameter("max_radius", 0.5);
   dot_threshold_for_line_ = declare_parameter("dot_threshold_for_line", 0.9);
   dot_threshold_for_circle_ = declare_parameter("dot_threshold_for_circle", 0.9);
   normal_angle_threshold_ = declare_parameter("normal_angle_threshold", 30.0);

   clusterer_.setClusterTolerance(cluster_tolerance_);
   clusterer_.setMinClusterSize(min_cluster_size_);
   clusterer_.setNormalAngleThreshold(normal_angle_threshold_);
}

void Detector::pcCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
   if (!lidar_condition_) {
      return;
   }

   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::fromROSMsg(*msg, *cloud);

   auto cloud_normals = estimateNormals(cloud, normal_estimation_radius_);

   auto clusters = clusterer_.cluster(cloud_normals);

   std::vector<std::optional<LineParams>> line_params;
   std::vector<bool> is_line_available;
   for (size_t i = 0; i < clusters.size(); ++i) {
      auto params = fitLine(clusters[i]);
      line_params.push_back(params);
      if (params.has_value()) {
         is_line_available.push_back(true);
      } else {
         is_line_available.push_back(false);
      }
   }

   std::vector<std::optional<CircleParams>> circle_params;
   std::vector<bool> is_circle_available;
   for (size_t i = 0; i < clusters.size(); ++i) {
      auto params = fitCircle(clusters[i]);
      circle_params.push_back(params);
      if (params.has_value() && params->radius >= min_radius_ && params->radius <= max_radius_) {
         is_circle_available.push_back(true);
      } else {
         is_circle_available.push_back(false);
      }
   }

   for (size_t i = 0; i < clusters.size(); ++i) {
      if (!(is_line_available[i] && is_circle_available[i])) {
         continue;
      }
      auto [slope, intercept] = line_params[i].value();
      Eigen::Vector2d line_direction(1, slope);
      line_direction.normalize();

      std::vector<double> dots;
      for (const auto &p : clusters[i]->points) {
         Eigen::Vector2d normal_vector(p.normal_x, p.normal_y);
         normal_vector.normalize();
         double dot = line_direction.dot(normal_vector);
         dots.push_back(std::abs(dot));
      }
      auto max_dot = *std::max_element(dots.begin(), dots.end());
      if (max_dot < dot_threshold_for_line_) {
         is_circle_available[i] = false;
      }
      if (max_dot > dot_threshold_for_circle_) {
         is_line_available[i] = false;
      }
   }

   for (size_t i = 0; i < clusters.size(); ++i) {
      if (!is_line_available[i] || !is_circle_available[i]) {
         continue;
      }

      double line_residual = calcRSSFromLine(clusters[i], line_params[i].value());
      double circle_residual = calcRSSFromCircle(clusters[i], circle_params[i].value());

      if (line_residual < circle_residual) {
         is_circle_available[i] = false;
      } else {
         is_line_available[i] = false;
      }
   }

   auto obstacles = obstacle_msgs::msg::Obstacles();
   obstacles.header = msg->header;
   for (size_t i = 0; i < clusters.size(); ++i) {
      if (is_line_available[i]) {
         auto [slope, intercept] = line_params[i].value();
         auto line_segment = obstacle_msgs::msg::LineSegment();
         line_segment.start.x = clusters[i]->points.front().x;
         line_segment.start.y = slope * line_segment.start.x + intercept;
         line_segment.end.x = clusters[i]->points.back().x;
         line_segment.end.y = slope * line_segment.end.x + intercept;
         obstacles.line_segments.push_back(line_segment);
      } else if (is_circle_available[i]) {
         auto [center_x, center_y, radius] = circle_params[i].value();
         auto circle = obstacle_msgs::msg::Circle();
         circle.center.x = center_x;
         circle.center.y = center_y;
         circle.radius = radius;
         obstacles.circles.push_back(circle);
      }
   }
   publisher_->publish(obstacles);
}

void Detector::lidarConditionCallback(const std_msgs::msg::Bool::SharedPtr msg) {
   lidar_condition_ = msg->data;
}

double Detector::calcRSSFromCircle(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud, const CircleParams &circle_params) {
   double sum = 0.0;
   auto [center_x, center_y, radius] = circle_params;
   for (const auto &p : cloud->points) {
      double residual = std::hypot(p.x - center_x, p.y - center_y) - radius;
      sum += residual * residual;
   }
   return sum;
};

double Detector::calcRSSFromLine(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud, const LineParams &line_params) {
   double sum = 0.0;
   auto [slope, intercept] = line_params;
   for (const auto &p : cloud->points) {
      double expected_y = slope * p.x + intercept;
      double residual = p.y - expected_y;
      sum += residual * residual;
   }
   return sum;
}

std::optional<CircleParams> Detector::fitCircle(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud) {
   int n = cloud->size();
   if (n < 3) {
      return std::nullopt;
   }

   Eigen::MatrixXd A(n, 3);
   Eigen::VectorXd b(n);

   for (int i = 0; i < n; ++i) {
      double x = cloud->points[i].x;
      double y = cloud->points[i].y;
      A(i, 0) = 2 * x;
      A(i, 1) = 2 * y;
      A(i, 2) = -1;
      b(i) = x * x + y * y;
   }

   Eigen::Vector3d solution = A.colPivHouseholderQr().solve(b);

   double center_x = solution(0);
   double center_y = solution(1);
   double radius = sqrt(center_x * center_x + center_y * center_y - solution(2));

   return CircleParams{center_x, center_y, radius};
}

std::optional<LineParams> Detector::fitLine(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud) {
   int n = cloud->size();
   if (n < 2) {
      return std::nullopt;
   }

   Eigen::MatrixXd A(n, 2);
   Eigen::VectorXd b(n);

   for (int i = 0; i < n; ++i) {
      A(i, 0) = cloud->points[i].x;
      A(i, 1) = 1.0;
      b(i) = cloud->points[i].y;
   }

   Eigen::Vector2d solution = A.colPivHouseholderQr().solve(b);

   return LineParams{solution(0), solution(1)};
}

pcl::Normal Detector::estimateNormal(const pcl::search::KdTree<pcl::PointXYZ>::Ptr &kdtree, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointXYZ &point, double radius) {
   pcl::PointIndices::Ptr nearestIndices(new pcl::PointIndices);
   std::vector<float> squaredDistances;
   kdtree->setInputCloud(cloud);
   kdtree->radiusSearch(point, radius, nearestIndices->indices, squaredDistances);

   pcl::PointCloud<pcl::PointXYZ> nearestPoints;
   pcl::ExtractIndices<pcl::PointXYZ> extract;
   extract.setInputCloud(cloud);
   extract.setIndices(nearestIndices);
   extract.filter(nearestPoints);

   Eigen::MatrixXd data(2, nearestPoints.size());
   for (size_t i = 0; i < nearestPoints.size(); ++i) {
      data(0, i) = nearestPoints.points[i].x;
      data(1, i) = nearestPoints.points[i].y;
   }

   Eigen::MatrixXd centered = data.colwise() - data.rowwise().mean();
   Eigen::MatrixXd cov = (centered * centered.transpose()) / double(nearestPoints.size() - 1);

   Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(cov);

   return pcl::Normal(eigensolver.eigenvectors()(0, 0), eigensolver.eigenvectors()(1, 0), 0.0);
}

pcl::PointCloud<pcl::PointNormal>::Ptr Detector::estimateNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double radius) {
   pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
   pcl::PointCloud<pcl::Normal>::Ptr pc_normals(new pcl::PointCloud<pcl::Normal>);
   pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);

   for (const auto &p : cloud->points) {
      pcl::Normal normal = estimateNormal(kdtree, cloud, p, radius);
      pc_normals->push_back(normal);
   }

   pcl::concatenateFields(*cloud, *pc_normals, *cloud_normals);

   return cloud_normals;
}
