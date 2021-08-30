#include "../include/fusion.h"
// #define TIMER
// #define DEBUG

/* 打印参数 */
void fusion::printParams()
{
    std::cout << "*********** 当前运行参数 ***********" << std::endl;
    std::cout << "相机话题：" << '\n'
              << params.cam1_topic << '\n'
              << params.cam2_topic << '\n'
              << params.cam3_topic << '\n'
              << params.cam4_topic << std::endl;
    std::cout << "雷达话题：" << '\n'
              << params.lidar_topic << std::endl;
    std::cout << "相机内参" << '\n'
              << params.cam1_in << '\n'
              << params.cam2_in << '\n'
              << params.cam3_in << '\n'
              << params.cam4_in << std::endl;
    std::cout << "相机-雷达外参" << '\n'
              << params.lidar_cam1 << '\n'
              << params.lidar_cam2 << '\n'
              << params.lidar_cam3 << '\n'
              << params.lidar_cam4 << std::endl;
    std::cout << "********************************" << std::endl;
}

/* 读取参数 */
void fusion::initParams(ros::NodeHandle nh)
{
    std::string cfg_path;
    // 局部空间读取全局空间的参数，参数名称前要加/
    nh.getParam("/cfg", cfg_path);
    std::cout << "从 " << cfg_path << " 文件加载参数" << std::endl;
    std::ifstream infile(cfg_path);
    // 读取话题
    infile >> params.cam1_topic;
    infile >> params.cam2_topic;
    infile >> params.cam3_topic;
    infile >> params.cam4_topic;
    infile >> params.lidar_topic;
    // 读取四个相机内参
    double cam_in[12];
    for (int i = 0; i < 12; i++)
        infile >> cam_in[i];
    cv::Mat(3, 4, 6, &cam_in).copyTo(params.cam1_in);
    for (int i = 0; i < 12; i++)
        infile >> cam_in[i];
    cv::Mat(3, 4, 6, &cam_in).copyTo(params.cam2_in);
    for (int i = 0; i < 12; i++)
        infile >> cam_in[i];
    cv::Mat(3, 4, 6, &cam_in).copyTo(params.cam3_in);
    for (int i = 0; i < 12; i++)
        infile >> cam_in[i];
    cv::Mat(3, 4, 6, &cam_in).copyTo(params.cam4_in);
    // 读取四个相机-雷达外参
    double RT[16];
    for (int i = 0; i < 16; i++)
        infile >> RT[i];
    cv::Mat(4, 4, 6, &RT).copyTo(params.lidar_cam1);
    for (int i = 0; i < 16; i++)
        infile >> RT[i];
    cv::Mat(4, 4, 6, &RT).copyTo(params.lidar_cam2);
    for (int i = 0; i < 16; i++)
        infile >> RT[i];
    cv::Mat(4, 4, 6, &RT).copyTo(params.lidar_cam3);
    for (int i = 0; i < 16; i++)
        infile >> RT[i];
    cv::Mat(4, 4, 6, &RT).copyTo(params.lidar_cam4);
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
    // 图像ros->opencv
    try
    {
        cv_ptr1 = cv_bridge::toCvCopy(img1, "bgr8");
        cv_ptr2 = cv_bridge::toCvCopy(img2, "bgr8");
        cv_ptr3 = cv_bridge::toCvCopy(img3, "bgr8");
        cv_ptr4 = cv_bridge::toCvCopy(img4, "bgr8");
    }
    catch (cv_bridge::Exception &err)
    {
        ROS_ERROR("cv_bridge exception: %s", err.what());
        return;
    }
    // 图像像素范围
    cv::Rect2d frame1(0, 0, cv_ptr1->image.cols, cv_ptr2->image.rows);
    cv::Rect2d frame2(0, 0, cv_ptr2->image.cols, cv_ptr2->image.rows);
    cv::Rect2d frame3(0, 0, cv_ptr3->image.cols, cv_ptr3->image.rows);
    cv::Rect2d frame4(0, 0, cv_ptr4->image.cols, cv_ptr4->image.rows);
    // 点云ros->pcl
    pcl::fromROSMsg(*pc, cloud);
#ifdef DEBUG
    std::cout << "点云获取成功:" << cloud->size() << std::endl;
#endif
    /* 遍历点云，两层判断
    1. 根据xy判断是哪个相机所在区域
           X         
        1/ | \4
        Y--Z---
        2\ | /3 
    2. 根据颜色判断是否需要保留 */
    for (auto it = cloud.points.begin(); it != cloud.points.end(); it++)
    {
        if (it->x > 0.0 && it->y > 0.0)
        {
            fusion::seg(it, cv_ptr1, params.cam1_in, params.lidar_cam1, frame1);
        }
        else if (it->x < 0.0 && it->y > 0.0)
        {
            fusion::seg(it, cv_ptr2, params.cam2_in, params.lidar_cam2, frame2);
        }
        else if (it->x < 0.0 && it->y < 0.0)
        {
            fusion::seg(it, cv_ptr3, params.cam3_in, params.lidar_cam3, frame3);
        }
        else
        {
            fusion::seg(it, cv_ptr4, params.cam4_in, params.lidar_cam4, frame4);
        }
    }
    // 发布分割后的点云
    pcl::toROSMsg(seg_cloud, seg_cloud_ros);
    seg_cloud.clear();
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

void fusion::seg(const pcl::PointCloud<pcl::PointXYZ>::const_iterator &it,
                 const cv_bridge::CvImageConstPtr &cv_ptr,
                 const cv::Mat &camIn, const cv::Mat &RT, const cv::Rect2d &frame)
{
    // 把原始点存到齐次坐标点中
    points_in_lidar_homo.at<double>(0, 0) = it->x;
    points_in_lidar_homo.at<double>(1, 0) = it->y;
    points_in_lidar_homo.at<double>(2, 0) = it->z;
    points_in_lidar_homo.at<double>(3, 0) = 1;
    // 像素坐标 = 内参矩阵 * 相机和雷达的外参 * 雷达坐标系下的齐次点
    pixel_homo = camIn * RT * points_in_lidar_homo;
    // 归一化 处理负数
    pixel.x = pixel_homo.at<double>(0, 0) / abs(pixel_homo.at<double>(2, 0));
    pixel.y = pixel_homo.at<double>(1, 0) / abs(pixel_homo.at<double>(2, 0));
#ifdef DEBUG
    std::cout << "像素投影成功" << std::endl;
#endif
    // 视野内的点
    if (pixel.inside(frame))
    {
        // 拿到像素点的颜色，注意opencv的横纵坐标顺序，直接用cv::Point最保险
        color_bgr = cv_ptr->image.at<cv::Vec3b>(pixel);

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

        // if ((color_bgr(0) == wall(0) && color_bgr(1) == wall(1) && color_bgr(2) == wall(2)) ||
        //     (color_bgr(0) == building(0) && color_bgr(1) == building(1) && color_bgr(2) == building(2)) ||
        //     (color_bgr(0) == floor(0) && color_bgr(1) == floor(1) && color_bgr(2) == floor(2)) ||
        //     (color_bgr(0) == ceiling(0) && color_bgr(1) == ceiling(1) && color_bgr(2) == ceiling(2)) ||
        //     (color_bgr(0) == road(0) && color_bgr(1) == road(1) && color_bgr(2) == road(2)) ||
        //     (color_bgr(0) == light(0) && color_bgr(1) == light(1) && color_bgr(2) == light(2)) ||
        //     (color_bgr(0) == door(0) && color_bgr(1) == door(1) && color_bgr(2) == door(2)) ||
        //     (color_bgr(0) == window(0) && color_bgr(1) == window(1) && color_bgr(2) == window(2))||
        //     (color_bgr(0) == ground(0) && color_bgr(1) == ground(1) && color_bgr(2) == ground(2)))
        // {
        //     seg_point.x = it->x;
        //     seg_point.y = it->y;
        //     seg_point.z = it->z;
        //     seg_point.b = color_bgr(0);
        //     seg_point.g = color_bgr(1);
        //     seg_point.r = color_bgr(2);
        //     // seg_point.b = 0;
        //     // seg_point.g = 0;
        //     // seg_point.r = 0;
        //     seg_cloud.push_back(seg_point);
        // }
        // else
        // {
        //     seg_point.x = it->x;
        //     seg_point.y = it->y;
        //     seg_point.z = it->z;
        //     seg_point.b = 0;
        //     seg_point.g = 0;
        //     seg_point.r = 255;
        //     seg_cloud.push_back(seg_point);
        // }
    }
    // 视野外的点
    else
    {
        seg_point.x = it->x;
        seg_point.y = it->y;
        seg_point.z = it->z;
        seg_point.r = 0;
        seg_point.g = 0;
        seg_point.b = 0;
        seg_cloud.push_back(seg_point);
    }
}

fusion::fusion()
{
    ros::NodeHandle nh("~"); // 局部空间
    initParams(nh);
    printParams();
    // 四个相机和雷达的topic
    message_filters::Subscriber<sensor_msgs::Image> img1_sub(nh, params.cam1_topic, 5);
    message_filters::Subscriber<sensor_msgs::Image> img2_sub(nh, params.cam2_topic, 5);
    message_filters::Subscriber<sensor_msgs::Image> img3_sub(nh, params.cam3_topic, 5);
    message_filters::Subscriber<sensor_msgs::Image> img4_sub(nh, params.cam4_topic, 5);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, params.lidar_topic, 5);
    // 同步
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), img1_sub, img2_sub, img3_sub, img4_sub, pc_sub);
    sync.registerCallback(boost::bind(&fusion::fusion_callback, this, _1, _2, _3, _4, _5));
    // 输出分割点云
    seg_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("seg_cloud", 10);
    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_lidar_fusion");
    fusion fusion_node;
}