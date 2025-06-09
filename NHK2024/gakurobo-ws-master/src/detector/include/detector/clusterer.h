#pragma once

#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>

class Clusterer {
   public:
   Clusterer();

   void setClusterTolerance(float cluster_tolerance);
   void setMinClusterSize(int min_cluster_size);
   void setNormalAngleThreshold(double normal_angle_threshold);

   std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> cluster(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);

   private:
   static bool isAngleSimilar(const pcl::PointNormal &seed_point, const pcl::PointNormal &candidate_point, float squared_distance);

   static double normal_angle_threshold_; // deg

   pcl::search::KdTree<pcl::PointNormal>::Ptr tree_;
   pcl::ConditionalEuclideanClustering<pcl::PointNormal> ec_;
};

inline void Clusterer::setClusterTolerance(float cluster_tolerance) {
   ec_.setClusterTolerance(cluster_tolerance);
}

inline void Clusterer::setMinClusterSize(int min_cluster_size) {
   ec_.setMinClusterSize(min_cluster_size);
}

inline void Clusterer::setNormalAngleThreshold(double normal_angle_threshold) {
   normal_angle_threshold_ = normal_angle_threshold;
}
