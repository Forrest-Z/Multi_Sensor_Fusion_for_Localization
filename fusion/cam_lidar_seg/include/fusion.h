#ifndef FUSION_H
#define FUSION_H
/* ROS */
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
/* opencv */
#include <opencv/cv.hpp>
/* pcl */
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>
/* eigen */
#include <Eigen/Dense>
/* c++ */
#include <iostream>
#include <cmath>
#include <algorithm>

// 四个相机和雷达的投影
class fusion
{
private:
    // 输入的图像和点云
    cv_bridge::CvImageConstPtr cv_ptr1;
    cv_bridge::CvImageConstPtr cv_ptr2;
    cv_bridge::CvImageConstPtr cv_ptr3;
    cv_bridge::CvImageConstPtr cv_ptr4;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // 待删除物体的颜色(BGR)
    cv::Vec3d road = cv::Vec3d(140, 140, 140);
    cv::Vec3d car = cv::Vec3d(200, 102, 0);
    cv::Vec3d people = cv::Vec3d(61, 5, 150);

    cv::Mat points_in_lidar_homo = cv::Mat(4, 1, cv::DataType<double>::type); // 原始点齐次形式
    cv::Mat pixel_homo = cv::Mat(3, 1, cv::DataType<double>::type);           // 像素点齐次形式
    cv::Point pixel;                                                          // 像素点
    cv::Vec3b color_bgr;                                                      // 像素点颜色
    pcl::PointXYZRGB seg_point;                                               // 添加颜色的点
    pcl::PointCloud<pcl::PointXYZRGB> seg_cloud;                              // 分割后的点云
    sensor_msgs::PointCloud2 seg_cloud_ros;                                   // 分割后的点云ROS

    ros::Publisher seg_cloud_pub; // 点云发布

    // 存储参数
    struct initial_parameters
    {
        std::string cam1_topic;
        std::string cam2_topic;
        std::string cam3_topic;
        std::string cam4_topic;
        std::string lidar_topic;
        cv::Mat cam1_in;
        cv::Mat cam2_in;
        cv::Mat cam3_in;
        cv::Mat cam4_in;
        cv::Mat lidar_cam1;
        cv::Mat lidar_cam2;
        cv::Mat lidar_cam3;
        cv::Mat lidar_cam4;
    } params;

private:
    void fusion_callback(const sensor_msgs::Image::ConstPtr &img1,
                         const sensor_msgs::Image::ConstPtr &img2,
                         const sensor_msgs::Image::ConstPtr &img3,
                         const sensor_msgs::Image::ConstPtr &img4,
                         const sensor_msgs::PointCloud2::ConstPtr &pc);

    void seg(const pcl::PointCloud<pcl::PointXYZ>::const_iterator &it,
             const cv_bridge::CvImageConstPtr &cv_ptr,
             const cv::Mat &camIn, const cv::Mat &RT, const cv::Rect2d &frame);

    void initParams(ros::NodeHandle nh);

    void printParams();

public:
    fusion();
};

#endif