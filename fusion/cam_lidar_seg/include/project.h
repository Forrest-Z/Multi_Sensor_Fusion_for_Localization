#ifndef PROJECT_H
#define PROJECT_H
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

// kitti数据集中相机和雷达的投影
class projector
{
private:
    image_transport::Publisher image_publisher;
    ros::Publisher seg_cloud_pub;

    struct initial_parameters
    {
        std::string camera_topic;
        std::string lidar_topic;
        cv::Mat camtocam_mat;
        cv::Mat cameraIn;
        cv::Mat RT;
    } i_params;

    void projection_callback(const sensor_msgs::Image::ConstPtr &img,
                             const sensor_msgs::PointCloud2::ConstPtr &pc);
    void initParams();

    float calPointAngle(const pcl::PointCloud<pcl::PointXYZ>::const_iterator &point);

public:
    projector();
};

// 四个相机和雷达的投影
class fusion
{
private:
    // 图像和点云的发布
    image_transport::Publisher image_publisher;
    ros::Publisher seg_cloud_pub;

    // 输入的四个图像
    cv_bridge::CvImagePtr cv_ptr1;
    cv_bridge::CvImagePtr cv_ptr2;
    cv_bridge::CvImagePtr cv_ptr3;
    cv_bridge::CvImagePtr cv_ptr4;

    // 输入的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    // 输出分割后的点云
    pcl::PointCloud<pcl::PointXYZRGB> seg_cloud;

    // 存储参数
    struct initial_parameters
    {
        std::string camera_topic_1;
        std::string camera_topic_2;
        std::string camera_topic_3;
        std::string camera_topic_4;
        std::string lidar_topic;
        cv::Mat camera1_in;
        cv::Mat camera2_in;
        cv::Mat camera3_in;
        cv::Mat camera4_in;
        cv::Mat cam1tolidar;
        cv::Mat cam2tolidar;
        cv::Mat cam3tolidar;
        cv::Mat cam4tolidar;
    } i_params;

    void fusion_callback(const sensor_msgs::Image::ConstPtr &img1,
                         const sensor_msgs::Image::ConstPtr &img2,
                         const sensor_msgs::Image::ConstPtr &img3,
                         const sensor_msgs::Image::ConstPtr &img4,
                         const sensor_msgs::PointCloud2::ConstPtr &pc);
    void seg_pxpy(pcl::PointCloud<pcl::PointXYZ>::const_iterator &it);
    void seg_nxpy(pcl::PointCloud<pcl::PointXYZ>::const_iterator &it);
    void seg_nxny(pcl::PointCloud<pcl::PointXYZ>::const_iterator &it);
    void seg_pxny(pcl::PointCloud<pcl::PointXYZ>::const_iterator &it);

    void initParams();

public:
    fusion();
};

#endif