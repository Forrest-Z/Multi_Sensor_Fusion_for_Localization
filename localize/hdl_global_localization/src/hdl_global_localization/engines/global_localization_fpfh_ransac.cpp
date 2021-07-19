#include <hdl_global_localization/engines/global_localization_fpfh_ransac.hpp>

#include <ros/ros.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/impl/kdtree.hpp>

#include <hdl_global_localization/ransac/ransac_pose_estimation.hpp>

namespace hdl_global_localization {

GlobalLocalizationEngineFPFH_RANSAC::GlobalLocalizationEngineFPFH_RANSAC(ros::NodeHandle& private_nh) : private_nh(private_nh) {}

GlobalLocalizationEngineFPFH_RANSAC::~GlobalLocalizationEngineFPFH_RANSAC() {}

/**
 * @brief 提取点云的FPFH特征, 参考了PCL官网的Robust pose estimation of rigid objects教程
 * @param cloud 输入点云
 * @return pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr
 */
pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr GlobalLocalizationEngineFPFH_RANSAC::extract_fpfh(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  double normal_estimation_radius = private_nh.param<double>("fpfh/normal_estimation_radius", 2.0);
  double search_radius = private_nh.param<double>("fpfh/search_radius", 8.0);

  // 估计场景的法线
  ROS_INFO_STREAM("Normal Estimation: Radius(" << normal_estimation_radius << ")");
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nest;
  nest.setRadiusSearch(normal_estimation_radius);
  nest.setInputCloud(cloud);
  nest.compute(*normals);

  // 估计点云的特征
  ROS_INFO_STREAM("FPFH Extraction: Search Radius(" << search_radius << ")");
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fest;
  fest.setRadiusSearch(search_radius);
  fest.setInputCloud(cloud);
  fest.setInputNormals(normals);
  ROS_INFO_STREAM("debug1");
  fest.compute(*features);  // 这一步卡死
  ROS_INFO_STREAM("debug2");
  return features;
}

void GlobalLocalizationEngineFPFH_RANSAC::set_global_map(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  global_map = cloud;
  // 全局地图提取FPFH特征
  global_map_features = extract_fpfh(cloud);

  ransac.reset(new RansacPoseEstimation<pcl::FPFHSignature33>(private_nh));
  ransac->set_target(global_map, global_map_features);
}

/**
 * @brief 查询全局位姿
 * @param cloud 输入的点云
 * @param max_num_candidates 返回几个result
 * @return GlobalLocalizationResults
 */
GlobalLocalizationResults GlobalLocalizationEngineFPFH_RANSAC::query(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int max_num_candidates) {
  // 特征估计
  pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr cloud_features = extract_fpfh(cloud);
  // 设置待配准点云
  ransac->set_source(cloud, cloud_features);
  // 实施配准
  auto results = ransac->estimate();
  // 对结果按照内点比例排序并只保留max_num_candidates个结果
  return results.sort(max_num_candidates);
}

}  // namespace hdl_global_localization