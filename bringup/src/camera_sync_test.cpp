/* ROS */
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
/* opencv */
#include <opencv/cv.hpp>
/* eigen */
// #include <Eigen/Dense>
/* c++ */
#include <iostream>
#include <cmath>
#include <algorithm>

#define DEBUG

/* 四个相机同步 */
class camera_sync
{
private:
    // 拼接图像的发布
    image_transport::Publisher image_publisher;

    // 输入的四个图像
    cv_bridge::CvImagePtr cv_ptr1;
    cv_bridge::CvImagePtr cv_ptr2;
    cv_bridge::CvImagePtr cv_ptr3;
    cv_bridge::CvImagePtr cv_ptr4;

    void sync_callback(const sensor_msgs::Image::ConstPtr &img1,
                       const sensor_msgs::Image::ConstPtr &img2,
                       const sensor_msgs::Image::ConstPtr &img3,
                       const sensor_msgs::Image::ConstPtr &img4);

public:
    camera_sync();
};

void camera_sync::sync_callback(const sensor_msgs::Image::ConstPtr &img1,
                                const sensor_msgs::Image::ConstPtr &img2,
                                const sensor_msgs::Image::ConstPtr &img3,
                                const sensor_msgs::Image::ConstPtr &img4)
{
    // 获取四个图像
    cv_ptr1 = cv_bridge::toCvCopy(img1, "bgr8");
    cv_ptr2 = cv_bridge::toCvCopy(img2, "bgr8");
    cv_ptr3 = cv_bridge::toCvCopy(img3, "bgr8");
    cv_ptr4 = cv_bridge::toCvCopy(img4, "bgr8");

#ifdef DEBUG
    std::cout << "图像获取成功:" << '\n'
              << "img1: " << cv_ptr1->image.cols << " x " << cv_ptr1->image.rows << '\n'
              << "img2: " << cv_ptr2->image.cols << " x " << cv_ptr2->image.rows << '\n'
              << "img3: " << cv_ptr3->image.cols << " x " << cv_ptr3->image.rows << '\n'
              << "img4: " << cv_ptr4->image.cols << " x " << cv_ptr4->image.rows << std::endl;
#endif

    // 拼接图像
    // TODO
    cv::Mat result;
    cv::vconcat(cv_ptr1->image, cv_ptr2->image);
}

camera_sync::camera_sync()
{
    ros::NodeHandle nh("~");

    // 四个相机和雷达的topic
    message_filters::Subscriber<sensor_msgs::Image> image_sub_1(nh, "", 5);
    message_filters::Subscriber<sensor_msgs::Image> image_sub_2(nh, "", 5);
    message_filters::Subscriber<sensor_msgs::Image> image_sub_3(nh, "", 5);
    message_filters::Subscriber<sensor_msgs::Image> image_sub_4(nh, "", 5);
    // 同步器
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), image_sub_1, image_sub_2, image_sub_3, image_sub_4);
    sync.registerCallback(boost::bind(&camera_sync::sync_callback, this, _1, _2, _3, _4));

    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_sync");
    camera_sync cs;
    return 0;
}