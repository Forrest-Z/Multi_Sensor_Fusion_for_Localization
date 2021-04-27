#include "../include/fusion.h"
#define TIMER
// #define DEBUG

// 待删除物体的颜色(BGR)
cv::Vec3d road(140, 140, 140);
cv::Vec3d car(200, 102, 0);
cv::Vec3d people(61, 5, 150);

void fusion::initParams(ros::NodeHandle nh)
{
    std::string cfg_path;
    // 局部空间读取全局空间的参数，参数名称前要加/
    nh.getParam("/cfg", cfg_path);
    std::cout << "从 " << cfg_path << " 加载参数" << std::endl;
    std::ifstream infile(cfg_path);
    // 读取话题
    infile >> params.camera_topic_1;
    infile >> params.camera_topic_2;
    infile >> params.camera_topic_3;
    infile >> params.camera_topic_4;
    infile >> params.lidar_topic;
#ifdef DEBUG
    std::cout << "相机话题：" << '\n'
              << params.camera_topic_1 << '\n'
              << params.camera_topic_2 << '\n'
              << params.camera_topic_3 << '\n'
              << params.camera_topic_4 << std::endl;
    std::cout << "雷达话题：" << '\n'
              << params.lidar_topic << std::endl;
#endif

    // 读取四个相机内参
    double_t cam_in[12];
    for (int i = 0; i < 12; i++)
        infile >> cam_in[i];
    cv::Mat(3, 4, 6, &cam_in).copyTo(params.camera1_in);
    for (int i = 0; i < 12; i++)
        infile >> cam_in[i];
    cv::Mat(3, 4, 6, &cam_in).copyTo(params.camera2_in);
    for (int i = 0; i < 12; i++)
        infile >> cam_in[i];
    cv::Mat(3, 4, 6, &cam_in).copyTo(params.camera3_in);
    for (int i = 0; i < 12; i++)
        infile >> cam_in[i];
    cv::Mat(3, 4, 6, &cam_in).copyTo(params.camera4_in);
#ifdef DEBUG
    std::cout << "相机内参" << '\n'
              << params.camera1_in << '\n'
              << params.camera2_in << '\n'
              << params.camera3_in << '\n'
              << params.camera4_in << std::endl;
#endif
    // 读取四个相机-雷达外参
    double_t cam2lidar[16];
    for (int i = 0; i < 16; i++)
        infile >> cam2lidar[i];
    cv::Mat(4, 4, 6, &cam2lidar).copyTo(params.lidar_cam1);
    for (int i = 0; i < 16; i++)
        infile >> cam2lidar[i];
    cv::Mat(4, 4, 6, &cam2lidar).copyTo(params.lidar_cam2);
    for (int i = 0; i < 16; i++)
        infile >> cam2lidar[i];
    cv::Mat(4, 4, 6, &cam2lidar).copyTo(params.lidar_cam3);
    for (int i = 0; i < 16; i++)
        infile >> cam2lidar[i];
    cv::Mat(4, 4, 6, &cam2lidar).copyTo(params.lidar_cam4);
#ifdef DEBUG
    std::cout << "相机-雷达外参" << '\n'
              << params.lidar_cam1 << '\n'
              << params.lidar_cam2 << '\n'
              << params.lidar_cam3 << '\n'
              << params.lidar_cam4 << std::endl;
#endif
}

void fusion::fusion_callback(const sensor_msgs::Image::ConstPtr &img1,
                             const sensor_msgs::Image::ConstPtr &img2,
                             const sensor_msgs::Image::ConstPtr &img3,
                             const sensor_msgs::Image::ConstPtr &img4,
                             const sensor_msgs::PointCloud2::ConstPtr &pc)
{
#ifdef TIMER
    // 计时
    pcl::console::TicToc run_time;
    run_time.tic();
#endif
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
    // 获取点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 底层是一个智能指针，记得初始化
    pcl::fromROSMsg(*pc, *cloud);
#ifdef DEBUG
    std::cout << "点云获取成功:" << cloud->size() << std::endl;
#endif
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud->points.begin(); it != cloud->points.end(); it++)
    {
        /* 按照xy分为四个区域，每个区域对应一个相机
           x         
        1/ | \4
        y--z---
        2\ | /3
        */
        if (it->x > 0.0)
        {
            if (it->y > 0.0)
                fusion::seg(it, params.camera1_in, params.lidar_cam1, cv_ptr1);
            else
                fusion::seg(it, params.camera4_in, params.lidar_cam4, cv_ptr4);
        }
        else
        {
            if (it->y > 0.0)
                fusion::seg(it, params.camera2_in, params.lidar_cam2, cv_ptr2);
            else
                fusion::seg(it, params.camera3_in, params.lidar_cam3, cv_ptr3);
        }
    }
    // 发布分割后的点云
    sensor_msgs::PointCloud2 seg_cloud_ros;
    pcl::toROSMsg(seg_cloud, seg_cloud_ros);
    seg_cloud.clear();// 成员变量，不清除的话会越来越大
    seg_cloud_ros.header.frame_id = pc->header.frame_id;
    seg_cloud_ros.header.stamp = pc->header.stamp;
    seg_cloud_pub.publish(seg_cloud_ros);
#ifdef DEBUG
    std::cout << "发布点云" << std::endl;
#endif
#ifdef TIMER
    std::cout << "本次分割耗时:" << run_time.toc() << " ms" << std::endl;
#endif
}

void fusion::seg(pcl::PointCloud<pcl::PointXYZ>::const_iterator it,
                 const cv::Mat &camIn, const cv::Mat &RT, cv_bridge::CvImageConstPtr img)
{
    cv::Mat points_in_lidar_homo(4, 1, cv::DataType<double>::type);
    cv::Mat pixel_homo(3, 1, cv::DataType<double>::type);
    cv::Point pixel;
    cv::Vec3b color_bgr;
    // 添加颜色的点
    pcl::PointXYZRGB seg_point;
    // 把原始点存到齐次坐标点中
    points_in_lidar_homo.at<double>(0, 0) = it->x;
    points_in_lidar_homo.at<double>(1, 0) = it->y;
    points_in_lidar_homo.at<double>(2, 0) = it->z;
    points_in_lidar_homo.at<double>(3, 0) = 1;
    // 像素坐标 = 内参矩阵 * 相机和雷达的外参 * 雷达坐标系下的齐次点
    pixel_homo = camIn * RT * points_in_lidar_homo;
    // 归一化,注意处理负数
    pixel.x = pixel_homo.at<double>(0, 0) / abs(pixel_homo.at<double>(2, 0));
    pixel.y = pixel_homo.at<double>(1, 0) / abs(pixel_homo.at<double>(2, 0));
#ifdef DEBUG
    std::cout << "像素投影成功" << std::endl;
#endif
    // 图像像素范围
    cv::Rect2d frame(0, 0, img->image.cols, img->image.rows);
    // 视野内的点
    if (pixel.inside(frame))
    {
        // 拿到像素点的颜色，注意opencv的横纵坐标顺序，直接用cv::Point最保险
        color_bgr = cv_ptr1->image.at<cv::Vec3b>(pixel);
        // 把人的颜色删除
        // if (color_bgr(0) == people(0) && color_bgr(1) == people(1) && color_bgr(2) == people(2))
        // {
        //     return;
        // }
        seg_point.x = it->x;
        seg_point.y = it->y;
        seg_point.z = it->z;
        seg_point.b = color_bgr(0);
        seg_point.g = color_bgr(1);
        seg_point.r = color_bgr(2);
        seg_cloud.push_back(seg_point);
    }
    // 视野外的点
    else
    {
        seg_point.x = it->x;
        seg_point.y = it->y;
        seg_point.z = it->z;
        seg_point.r = 255;
        seg_point.g = 0;
        seg_point.b = 0;
        seg_cloud.push_back(seg_point);
    }
}

fusion::fusion()
{
    ros::NodeHandle nh("~");
    // 读取参数
    initParams(nh);
    // 四个相机和雷达的topic
    message_filters::Subscriber<sensor_msgs::Image> image_sub_1(nh, params.camera_topic_1, 5);
    message_filters::Subscriber<sensor_msgs::Image> image_sub_2(nh, params.camera_topic_2, 5);
    message_filters::Subscriber<sensor_msgs::Image> image_sub_3(nh, params.camera_topic_3, 5);
    message_filters::Subscriber<sensor_msgs::Image> image_sub_4(nh, params.camera_topic_4, 5);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, params.lidar_topic, 5);
    // 同步
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), image_sub_1, image_sub_2, image_sub_3, image_sub_4, pc_sub);
    sync.registerCallback(boost::bind(&fusion::fusion_callback, this, _1, _2, _3, _4, _5));
    // 输出投影图片
    // image_transport::ImageTransport imageTransport(nh);
    // image_publisher = imageTransport.advertise("image_plus_cloud", 20);
    // 输出分割点云
    seg_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("seg_cloud", 10);
    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_lidar_fusion");
    fusion fs;
    return 0;
}