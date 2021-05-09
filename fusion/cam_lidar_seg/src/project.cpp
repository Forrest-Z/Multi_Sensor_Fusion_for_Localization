#include "../include/project.h"
// #define TIMER
// #define DEBUG

/* 打印参数 */
void projector::printParams()
{
    std::cout << "*********** 当前运行参数 ***********" << std::endl;
    std::cout << "从" << params.camera_topic << "话题读取相机数据" << '\n'
              << "从" << params.lidar_topic << "话题读取雷达数据" << '\n'
              << "相机内参" << '\n'
              << params.cameraIn << '\n'
              << "相机-雷达外参" << '\n'
              << params.RT << std::endl;
    std::cout << "********************************" << std::endl;
}

/* 读取参数 */
void projector::initParams(ros::NodeHandle nh)
{
    std::string cfg_path;
    nh.getParam("/cfg", cfg_path);
    std::cout << "从 " << cfg_path << " 文件加载参数" << std::endl;
    std::ifstream infile(cfg_path);
    infile >> params.camera_topic;
    infile >> params.lidar_topic;

    double_t cameraIn[12];
    double_t RT[16];
    for (int i = 0; i < 12; i++)
        infile >> cameraIn[i];
    cv::Mat(3, 4, 6, &cameraIn).copyTo(params.cameraIn);

    for (int i = 0; i < 16; i++)
        infile >> RT[i];
    cv::Mat(4, 4, 6, &RT).copyTo(params.RT);
}

/* 同步后的数据回调函数 */
void projector::projection_callback(const sensor_msgs::Image::ConstPtr &img,
                                    const sensor_msgs::PointCloud2::ConstPtr &pc)
{
#ifdef TIMER
    // 计时
    pcl::console::TicToc run_time;
    run_time.tic();
#endif
    // 获取图像
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat raw_img = cv_ptr->image;
    cv::Mat gray_img;
    cvtColor(raw_img, gray_img, cv::COLOR_BGR2GRAY);
    cv::Mat fusion_img;
    cv::cvtColor(gray_img, fusion_img, cv::COLOR_GRAY2BGR);
#ifdef DEBUG
    std::cout << "图像获取成功:" << raw_img.cols << " * " << raw_img.rows << " *" << raw_img.channels() << std::endl;
#endif
    // 获取点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pc, *cloud);
#ifdef DEBUG
    std::cout << "点云获取成功:" << cloud->size() << std::endl;
#endif
    // 各坐标下的点
    cv::Mat points_in_lidar_homo(4, 1, cv::DataType<double>::type);
    cv::Mat points_in_cam_homo(4, 1, cv::DataType<double>::type);
    cv::Mat pixel_homo(3, 1, cv::DataType<double>::type);
    cv::Point pixel;
    cv::Vec3b color_bgr;
    // 上色后的点云
    pcl::PointCloud<pcl::PointXYZRGB> seg_cloud;
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud->points.begin(); it != cloud->points.end(); it++)
    {
        // 使用坐标过滤点云
        // if (it->x < 0.0)
        // {
        //     continue;
        // }
        // 把点云数据存到齐次坐标点X中
        points_in_lidar_homo.at<double>(0, 0) = it->x;
        points_in_lidar_homo.at<double>(1, 0) = it->y;
        points_in_lidar_homo.at<double>(2, 0) = it->z;
        points_in_lidar_homo.at<double>(3, 0) = 1;
        // 相机坐标下的点云 = 相机之间的外参 * 相机和雷达的外参 * 雷达坐标系下的齐次点云
        points_in_cam_homo = params.RT * points_in_lidar_homo;
        // 像素坐标 = 内参矩阵 * 相机坐标下的点
        pixel_homo = params.cameraIn * points_in_cam_homo;
        // 归一化像素坐标
        // 这里可能会负负得正！
        pixel.x = pixel_homo.at<double>(0, 0) / abs(pixel_homo.at<double>(2, 0));
        pixel.y = pixel_homo.at<double>(1, 0) / abs(pixel_homo.at<double>(2, 0));
#ifdef DEBUG
        std::cout << "像素投影成功" << std::endl;
#endif
        // 点云上色和图像上色
        pcl::PointXYZRGB seg_point;
        // 相机视野框
        cv::Rect2d frame(0, 0, raw_img.cols, raw_img.rows);
        // 视野内点云按照相机上色
        if (pixel.inside(frame))
        {
            color_bgr = raw_img.at<cv::Vec3b>(pixel);
            /*  
            // 根据颜色删除点云
            // 道路
            if (color_bgr(0) == 140 && color_bgr(1) == 140 && color_bgr(2) == 140)
            {
                continue;
            }
            // 车辆
            if (color_bgr(0) == 200 && color_bgr(1) == 102 && color_bgr(2) == 0)
            {
                continue;
            }
            // 行人
            if (color_bgr(0) == 61 && color_bgr(1) == 5 && color_bgr(2) == 150)
            {
                continue;
            } 
            */
            seg_point.x = points_in_cam_homo.at<double>(0, 0);
            seg_point.y = points_in_cam_homo.at<double>(1, 0);
            seg_point.z = points_in_cam_homo.at<double>(2, 0);
            seg_point.b = color_bgr(0);
            seg_point.g = color_bgr(1);
            seg_point.r = color_bgr(2);
            seg_cloud.push_back(seg_point);
            cv::circle(fusion_img, pixel, 5, color_bgr, -1);
        }
        // 视野外点云为红色
        else
        {
            seg_point.x = points_in_cam_homo.at<double>(0, 0);
            seg_point.y = points_in_cam_homo.at<double>(1, 0);
            seg_point.z = points_in_cam_homo.at<double>(2, 0);
            seg_point.r = 255;
            seg_point.g = 0;
            seg_point.b = 0;
            seg_cloud.push_back(seg_point);
        }
    }
    // 发布点云投影后的图像
    cv_ptr->image = fusion_img;
    image_publisher.publish(cv_ptr->toImageMsg());
#ifdef DEBUG
    std::cout << "发布点云投影后的图像" << std::endl;
#endif
    // 发布分割后的点云
    sensor_msgs::PointCloud2 seg_cloud_ros;
    pcl::toROSMsg(seg_cloud, seg_cloud_ros);
    seg_cloud.clear();
    seg_cloud_ros.header.frame_id = pc->header.frame_id;
    seg_cloud_ros.header.stamp = pc->header.stamp;
    seg_cloud_pub.publish(seg_cloud_ros);
#ifdef DEBUG
    std::cout << "发布分割后的点云" << std::endl;
#endif
#ifdef TIMER
    std::cout << "程序运行时间:" << run_time.toc() << " ms" << std::endl;
#endif
}

/* projector */
projector::projector()
{
    ros::NodeHandle nh("~");
    // 读取参数
    initParams(nh);
    printParams();
    // 读取相机和雷达的topic并同步
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, params.camera_topic, 5);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, params.lidar_topic, 5);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    //从两个sub里面同步
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), image_sub, pcl_sub);
    sync.registerCallback(boost::bind(&projector::projection_callback, this, _1, _2));

    // 图像发布器
    image_transport::ImageTransport imageTransport(nh);
    image_publisher = imageTransport.advertise("image_plus_cloud", 20);

    // 点云发布器
    seg_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("seg_cloud", 10);

    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "project_pc_to_image");
    projector projector_node;
}