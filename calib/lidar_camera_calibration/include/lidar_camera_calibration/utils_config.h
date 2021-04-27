#ifndef UTILS_CONFIG_H
#define UTILS_CONFIG_H

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <ros/package.h>

/* 参数相关 */
struct config_data
{
    cv::Size s;                                //图像尺寸
    std::vector<std::pair<float, float>> xyz_; //点云感兴趣区域过滤
    float intensity_thresh;// 激光强度阈值
    int num_of_markers;// marker数量
    cv::Mat P;// 相机内参
    int MAX_ITERS;// 迭代次数
    std::vector<float> initialRot; // 初始旋转(XYZ欧拉角)
    std::vector<float> initialTra; // 初始平移
};

void print_config(const config_data &config);

void read_config(config_data &config);

#endif