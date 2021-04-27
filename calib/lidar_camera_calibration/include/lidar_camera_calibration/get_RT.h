#ifndef GET_RT_H
#define GET_RT_H
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <utility>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <ros/ros.h>
#include <ros/package.h>

Eigen::Quaterniond addQ(Eigen::Quaterniond a, Eigen::Quaterniond b);

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> readArray();

// calculates rotation and translation that transforms points in the lidar frame to the camera frame
// 对传进来的lidar中的corner点和camera中的corner点，计算 lidar->camera 的旋转和平移矩阵
void calc_RT(Eigen::MatrixXd lidar, Eigen::MatrixXd camera, int MAX_ITERS, Eigen::Matrix3d lidarToCamera);

void readArucoPose(std::vector<float> marker_info, int num_of_marker_in_config);

void find_transformation(std::vector<float> marker_info, int num_of_marker_in_config, int MAX_ITERS, Eigen::Matrix3d lidarToCamera);

#endif