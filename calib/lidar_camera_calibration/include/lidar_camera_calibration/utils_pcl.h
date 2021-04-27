#ifndef UTILS_PCL_H
#define UTILS_PCL_H

#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/common/intersections.h>

#include <lidar_camera_calibration/utils_config.h>

//自定义点云格式
struct myPointXYZRID
{
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    float range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW //结构体对齐吧
};

// 注册点类型宏
POINT_CLOUD_REGISTER_POINT_STRUCT(
    myPointXYZRID,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring))

// 自定义格式 -> XYZ格式
pcl::PointCloud<pcl::PointXYZ> *toPointsXYZ(pcl::PointCloud<myPointXYZRID> point_cloud);

// 点云变换，输入 平移+xyz欧拉角
pcl::PointCloud<myPointXYZRID> transform(pcl::PointCloud<myPointXYZRID> pc, float x, float y, float z, float rot_x, float rot_y, float rot_z);

// 正则化点云强度
pcl::PointCloud<myPointXYZRID> normalizeIntensity(pcl::PointCloud<myPointXYZRID> point_cloud, float min, float max);

// 过滤函数，这个最重要
pcl::PointCloud<myPointXYZRID> intensityByRangeDiff(pcl::PointCloud<myPointXYZRID> point_cloud, config_data config);

#endif