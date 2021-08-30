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

// 点云投到像素平面
class projector
{
private:
    // 待保存物体的颜色(BGR)
    cv::Vec3d wall = cv::Vec3d(120, 120, 120);
    cv::Vec3d building = cv::Vec3d(180, 120, 120);
    cv::Vec3d floor = cv::Vec3d(80, 50, 50);
    cv::Vec3d ceiling = cv::Vec3d(120, 120, 80);
    cv::Vec3d road = cv::Vec3d(140, 140, 140);
    cv::Vec3d light = cv::Vec3d(255, 173, 0);
    cv::Vec3d door = cv::Vec3d(8, 255, 51);
    cv::Vec3d window = cv::Vec3d(230, 230, 230);
    cv::Vec3d ground = cv::Vec3d(120, 120, 70);

    cv::Vec3d box = cv::Vec3d(0, 255, 20);
    cv::Vec3d person = cv::Vec3d(150, 5, 61);

    image_transport::Publisher image_publisher;
    ros::Publisher seg_cloud_pub;

    struct initial_parameters
    {
        std::string camera_topic;
        std::string lidar_topic;
        cv::Mat cameraIn;
        cv::Mat RT;
    } params;

    void projection_callback(const sensor_msgs::Image::ConstPtr &img,
                             const sensor_msgs::PointCloud2::ConstPtr &pc);
    void initParams(ros::NodeHandle nh);

    void printParams();

public:
    projector();
};

#endif