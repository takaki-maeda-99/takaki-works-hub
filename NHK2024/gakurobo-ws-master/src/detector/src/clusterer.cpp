#include "detector/clusterer.h"

double Clusterer::normal_angle_threshold_ = 10.0;

Clusterer::Clusterer() {
   tree_ = std::make_shared<pcl::search::KdTree<pcl::PointNormal>>();

   ec_ = pcl::ConditionalEuclideanClustering<pcl::PointNormal>(true);
   ec_.setSearchMethod(tree_);
   ec_.setConditionFunction(&isAngleSimilar);
}

std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> Clusterer::cluster(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud) {
   tree_->setInputCloud(cloud);
   ec_.setInputCloud(cloud);

   std::vector<pcl::PointIndices> cluster_indices;
   ec_.segment(cluster_indices);

   std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> clusters;
   pcl::ExtractIndices<pcl::PointNormal> extract;
   extract.setNegative(false);
   extract.setInputCloud(cloud);
   for (const auto &indices : cluster_indices) {
      pcl::PointCloud<pcl::PointNormal>::Ptr cluster(new pcl::PointCloud<pcl::PointNormal>);
      extract.setIndices(std::make_shared<const pcl::PointIndices>(indices));
      extract.filter(*cluster);
      clusters.push_back(cluster);
   }
   return clusters;
}

bool Clusterer::isAngleSimilar(const pcl::PointNormal &seed_point, const pcl::PointNormal &candidate_point, float squared_distance) {
   Eigen::Vector2f seed_normal(seed_point.normal_x, seed_point.normal_y);
   Eigen::Vector2f candidate_normal(candidate_point.normal_x, candidate_point.normal_y);
   double angle = acos(std::abs(seed_normal.dot(candidate_normal)) / seed_normal.norm() / candidate_normal.norm()); // rad
   if (angle / M_PI * 180.0 < normal_angle_threshold_)
      return true;
   else
      return false;
}
