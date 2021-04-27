#ifndef GET_CORNERS_H
#define GET_CORNERS_H

#include <iostream>
#include <fstream>
#include <map>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <ros/package.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/common/intersections.h>

bool getCorners(cv::Mat img, pcl::PointCloud<pcl::PointXYZ> scan, cv::Mat P, int num_of_markers, int MAX_ITERS);

cv::Point project(const pcl::PointXYZ &pt, const cv::Mat &projection_matrix);

cv::Mat project(cv::Mat projection_matrix, cv::Rect frame, pcl::PointCloud<pcl::PointXYZ> point_cloud, pcl::PointCloud<pcl::PointXYZ> *visible_points);

void onMouse(int event, int x, int y, int f, void *g);

#endif