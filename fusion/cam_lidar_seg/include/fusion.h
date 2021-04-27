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
    cv_bridge::CvImageConstPtr cv_ptr1;
    cv_bridge::CvImageConstPtr cv_ptr2;
    cv_bridge::CvImageConstPtr cv_ptr3;
    cv_bridge::CvImageConstPtr cv_ptr4;

    // 分割后的点云
    pcl::PointCloud<pcl::PointXYZRGB> seg_cloud;

    // 输出图像和点云
    // image_transport::Publisher image_publisher;
    ros::Publisher seg_cloud_pub;

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
        cv::Mat lidar_cam1;
        cv::Mat lidar_cam2;
        cv::Mat lidar_cam3;
        cv::Mat lidar_cam4;
    } params;

    void fusion_callback(const sensor_msgs::Image::ConstPtr &img1,
                         const sensor_msgs::Image::ConstPtr &img2,
                         const sensor_msgs::Image::ConstPtr &img3,
                         const sensor_msgs::Image::ConstPtr &img4,
                         const sensor_msgs::PointCloud2::ConstPtr &pc);

    void seg(pcl::PointCloud<pcl::PointXYZ>::const_iterator it,
             const cv::Mat &camIn, const cv::Mat &RT, cv_bridge::CvImageConstPtr img);

    void initParams(ros::NodeHandle nh);

public:
    fusion();
};

#endif