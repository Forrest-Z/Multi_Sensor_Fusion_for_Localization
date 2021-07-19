/*
 * This RansacPoseEstimation implementation was written based on pcl::SampleConsensusPrerejective
 */

#include <atomic>
#include <vector>
#include <random>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_rejection_poly.h>

#include <ros/ros.h>

#include <hdl_global_localization/ransac/ransac_pose_estimation.hpp>
#include <hdl_global_localization/ransac/matching_cost_evaluater_flann.hpp>
#include <hdl_global_localization/ransac/matching_cost_evaluater_voxels.hpp>

// 全部使用的是模板类的方式，保证点云特征可替换
namespace hdl_global_localization {

template <typename FeatureT>
RansacPoseEstimation<FeatureT>::RansacPoseEstimation(ros::NodeHandle& private_nh) : private_nh(private_nh) {}

template <typename FeatureT>
void RansacPoseEstimation<FeatureT>::set_target(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target, typename pcl::PointCloud<FeatureT>::ConstPtr target_features) {
  this->target = target;
  this->target_features = target_features;
  feature_tree.reset(new pcl::KdTreeFLANN<FeatureT>);
  feature_tree->setInputCloud(target_features);  // 对点云特征建立kdtree
  // TODO:下面的配置作用是什么
  if (private_nh.param<bool>("ransac/voxel_based", true)) {
    evaluater.reset(new MatchingCostEvaluaterVoxels());
  } else {
    evaluater.reset(new MatchingCostEvaluaterFlann());
  }
  // 判断是否是内点的阈值
  evaluater->set_target(target, private_nh.param<double>("ransac/max_correspondence_distance", 1.0));
}

template <typename FeatureT>
void RansacPoseEstimation<FeatureT>::set_source(pcl::PointCloud<pcl::PointXYZ>::ConstPtr source, typename pcl::PointCloud<FeatureT>::ConstPtr source_features) {
  this->source = source;
  this->source_features = source_features;
}

/**
 * @brief 重点看这个函数,实现了位姿的估计
 * @tparam FeatureT
 * @return GlobalLocalizationResults
 */
template <typename FeatureT>
GlobalLocalizationResults RansacPoseEstimation<FeatureT>::estimate() {
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformation_estimation;

  // CorrespondenceRejectorPoly类用于提前去除错误的位姿假设
  pcl::registration::CorrespondenceRejectorPoly<pcl::PointXYZ, pcl::PointXYZ> correspondence_rejection;
  correspondence_rejection.setInputTarget(target);  // 全局地图
  correspondence_rejection.setInputSource(source);  // 点云
  correspondence_rejection.setCardinality(3);       // 多边形基数
  correspondence_rejection.setSimilarityThreshold(  // 多边形边长的相似度阈值，越接近1越快，但是可能排除正确的位置
    private_nh.param<double>("ransac/similarity_threshold", 0.5));

  // 预先计算最近的点,用了OMP多线程加速
  ROS_INFO_STREAM("RANSAC : Precompute Nearest Features");
  std::vector<std::vector<int>> similar_features(source->size());  // 保存匹配的点对
#pragma omp parallel for
  for (int i = 0; i < source->size(); i++) {
    std::vector<float> sq_dists;

    feature_tree->nearestKSearch(
      source_features->at(i),
      private_nh.param<int>("ransac/correspondence_randomness", 2),  // 在N个最佳匹配点之间随机选择，越大越慢
      similar_features[i],
      sq_dists);
  }
  // 产生随机数
  std::vector<std::mt19937> mts(omp_get_max_threads());
  for (int i = 0; i < mts.size(); i++) {
    mts[i] = std::mt19937(i * 8191 + i + target->size() + source->size());
  }

  // 执行ransac
  ROS_INFO_STREAM("RANSAC : Main Loop");
  std::atomic_int matching_count(0);
  std::atomic_int iterations(0);
  std::vector<GlobalLocalizationResult::Ptr> results(private_nh.param<int>("ransac/max_iterations", 100000));
  int matching_budget = private_nh.param<int>("ransac/matching_budget", 10000);
  double min_inlier_fraction = private_nh.param<double>("ransac/inlier_fraction", 0.25);

#pragma omp parallel for
  for (int i = 0; i < results.size(); i++) {
    if (matching_count > matching_budget) {
      continue;
    }
    iterations++;

    auto& mt = mts[omp_get_thread_num()];
    std::vector<int> samples;
    std::vector<int> correspondences;
    select_samples(mt, similar_features, samples, correspondences);

    if (!correspondence_rejection.thresholdPolygon(samples, correspondences)) {
      continue;
    }

    // 得到变换矩阵
    Eigen::Matrix4f transformation;
    transformation_estimation.estimateRigidTransformation(*source, samples, *target, correspondences, transformation);

    // if (!params.is_valid(Eigen::Isometry3f(transformation))) {
    //   continue;
    // }

    matching_count++;
    double inlier_fraction = 0.0;
    // 计算匹配误差和内点比例
    double matching_error = evaluater->calc_matching_error(*source, transformation, &inlier_fraction);
    // TODO: 每次迭代下面这里输出误差太占CPU了
    // ROS_INFO_STREAM("RANSAC : iteration:" << iterations << " matching_count:" << matching_count << " error:" << matching_error << " inlier:" << inlier_fraction);

    // 保存内点比例足够大的匹配
    if (inlier_fraction > min_inlier_fraction) {
      results[i].reset(new GlobalLocalizationResult(matching_error, inlier_fraction, Eigen::Isometry3f(transformation)));
    }
  }

  return GlobalLocalizationResults(results);
}

/**
 * @brief TODO:没看懂啥操作
 * @tparam FeatureT 
 * @param mt 
 * @param similar_features 
 * @param samples 
 * @param correspondences 
 */
template <typename FeatureT>
void RansacPoseEstimation<FeatureT>::select_samples(
  std::mt19937& mt,
  const std::vector<std::vector<int>>& similar_features,
  std::vector<int>& samples,
  std::vector<int>& correspondences) const {
  samples.resize(3);
  for (int i = 0; i < samples.size(); i++) {
    samples[i] = std::uniform_int_distribution<>(0, similar_features.size() - 1)(mt);

    for (int j = 0; j < i; j++) {
      if (samples[j] == samples[i]) {
        i--;
      }
    }
  }

  correspondences.resize(3);
  for (int i = 0; i < samples.size(); i++) {
    if (similar_features[samples[i]].size() == 1) {
      correspondences[i] = similar_features[samples[i]][0];
    } else {
      int idx = std::uniform_int_distribution<>(0, similar_features[samples[i]].size() - 1)(mt);
      correspondences[i] = similar_features[samples[i]][idx];
    }
  }
}

// explicit instantiation
template class RansacPoseEstimation<pcl::FPFHSignature33>;

}  // namespace hdl_global_localization
