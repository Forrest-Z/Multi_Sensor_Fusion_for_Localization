#include "../include/fusion.h"
// #define TEST
// #define DEBUG

void fusion::initParams()
{
    // 拿到package的路径
    std::string pkg_loc = ros::package::getPath("cam_lidar_seg");
    std::ifstream infile(pkg_loc + "/cfg/fusion.txt");

    // 读取话题
    infile >> i_params.camera_topic_1;
    infile >> i_params.camera_topic_2;
    infile >> i_params.camera_topic_3;
    infile >> i_params.camera_topic_4;
    infile >> i_params.lidar_topic;
#ifdef DEBUG
    std::cout << "相机话题：" << '\n'
              << i_params.camera_topic_1 << '\n'
              << i_params.camera_topic_2 << '\n'
              << i_params.camera_topic_3 << '\n'
              << i_params.camera_topic_4 << std::endl;
    std::cout << "雷达话题：" << '\n'
              << i_params.lidar_topic << std::endl;
#endif

    // 读取四个相机内参
    double_t cam_in[12];
    for (int i = 0; i < 12; i++)
        infile >> cam_in[i];
    cv::Mat(3, 4, 6, &cam_in).copyTo(i_params.camera1_in);
    for (int i = 0; i < 12; i++)
        infile >> cam_in[i];
    cv::Mat(3, 4, 6, &cam_in).copyTo(i_params.camera2_in);
    for (int i = 0; i < 12; i++)
        infile >> cam_in[i];
    cv::Mat(3, 4, 6, &cam_in).copyTo(i_params.camera3_in);
    for (int i = 0; i < 12; i++)
        infile >> cam_in[i];
    cv::Mat(3, 4, 6, &cam_in).copyTo(i_params.camera4_in);
#ifdef DEBUG
    std::cout << "相机内参" << '\n'
              << i_params.camera1_in << '\n'
              << i_params.camera2_in << '\n'
              << i_params.camera3_in << '\n'
              << i_params.camera4_in << std::endl;
#endif

    // 读取四个相机-雷达外参
    double_t cam2lidar[16];
    for (int i = 0; i < 16; i++)
        infile >> cam2lidar[i];
    cv::Mat(4, 4, 6, &cam2lidar).copyTo(i_params.cam1tolidar);
    for (int i = 0; i < 16; i++)
        infile >> cam2lidar[i];
    cv::Mat(4, 4, 6, &cam2lidar).copyTo(i_params.cam2tolidar);
    for (int i = 0; i < 16; i++)
        infile >> cam2lidar[i];
    cv::Mat(4, 4, 6, &cam2lidar).copyTo(i_params.cam3tolidar);
    for (int i = 0; i < 16; i++)
        infile >> cam2lidar[i];
    cv::Mat(4, 4, 6, &cam2lidar).copyTo(i_params.cam4tolidar);
#ifdef DEBUG
    std::cout << "相机-雷达外参" << '\n'
              << i_params.cam1tolidar << '\n'
              << i_params.cam2tolidar << '\n'
              << i_params.cam3tolidar << '\n'
              << i_params.cam3tolidar << std::endl;
#endif
}

void fusion::fusion_callback(const sensor_msgs::Image::ConstPtr &img1,
                             const sensor_msgs::Image::ConstPtr &img2,
                             const sensor_msgs::Image::ConstPtr &img3,
                             const sensor_msgs::Image::ConstPtr &img4,
                             const sensor_msgs::PointCloud2::ConstPtr &pc)
{
#ifdef TEST
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
    pcl::fromROSMsg(*pc, *cloud);
#ifdef DEBUG
    std::cout << "点云获取成功:" << cloud->size() << std::endl;
#endif
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud->points.begin(); it != cloud->points.end(); it++)
    {
        /* 每个点分区处理
        雷达坐标系和相机编号
           x         
        1/ | \4
        y------
        2\ | /3
        */
        if (it->x > 0.0 && it->y > 0.0)
            fusion::seg_pxpy(it);
        else if (it->x < 0.0 && it->y < 0.0)
            fusion::seg_nxpy(it);
        else if (it->x < 0.0 && it->y < 0.0)
            fusion::seg_nxny(it);
        else
            fusion::seg_pxny(it);
    }
    // 发布分割后的点云
    sensor_msgs::PointCloud2 seg_cloud_ros;
    pcl::toROSMsg(seg_cloud, seg_cloud_ros);
    seg_cloud_ros.header.frame_id = pc->header.frame_id;
    seg_cloud_ros.header.stamp = pc->header.stamp;
    seg_cloud_pub.publish(seg_cloud_ros);
#ifdef DEBUG
    std::cout << "发布分割后的点云" << std::endl;
#endif
#ifdef TEST
    std::cout << "程序运行时间:" << run_time.toc() << " ms" << std::endl;
#endif
}

/* 点云分割函数1(positve x,positve y) */
void fusion::seg_pxpy(pcl::PointCloud<pcl::PointXYZ>::const_iterator &it)
{
    // 坐标变换后的点
    cv::Mat points_in_lidar_homo(4, 1, cv::DataType<double>::type);
    cv::Mat points_in_cam_homo(4, 1, cv::DataType<double>::type);
    cv::Mat pixel_homo(3, 1, cv::DataType<double>::type);
    cv::Point pixel;
    cv::Vec3b color_bgr;
    // 添加颜色后的点
    pcl::PointXYZRGB seg_point;
    // 把点云数据存到齐次坐标点X中
    points_in_lidar_homo.at<double>(0, 0) = it->x;
    points_in_lidar_homo.at<double>(1, 0) = it->y;
    points_in_lidar_homo.at<double>(2, 0) = it->z;
    points_in_lidar_homo.at<double>(3, 0) = 1;
    // 相机坐标下的点云 = 相机和雷达的外参 * 雷达坐标系下的齐次点云
    points_in_cam_homo = i_params.cam1tolidar * points_in_lidar_homo;
    // 像素坐标 = 内参矩阵 * 相机坐标下的点
    pixel_homo = i_params.camera1_in * points_in_cam_homo;
    // 归一化像素坐标
    pixel.x = pixel_homo.at<double>(0, 0) / pixel_homo.at<double>(2, 0);
    pixel.y = pixel_homo.at<double>(1, 0) / pixel_homo.at<double>(2, 0);
#ifdef DEBUG
    std::cout << "像素投影成功" << std::endl;
#endif
    // 视野外的点
    if (pixel.x < 0 || pixel.y < 0 || pixel.x > cv_ptr1->image.cols || pixel.y > cv_ptr1->image.rows)
    {
        seg_point.x = points_in_cam_homo.at<double>(0, 0);
        seg_point.y = points_in_cam_homo.at<double>(1, 0);
        seg_point.z = points_in_cam_homo.at<double>(2, 0);
        seg_point.r = 255;
        seg_point.g = 0;
        seg_point.b = 0;
        seg_cloud.push_back(seg_point);
    }
    // 视野内的点根据颜色进行分割
    else
    {
        // 拿到像素点的颜色，注意opencv的横纵坐标顺序，直接用cv::Point最保险
        color_bgr = cv_ptr1->image.at<cv::Vec3b>(pixel);
        // 定义一些待删除的颜色
        cv::Vec3d color_road(140, 140, 140);
        cv::Vec3d color_car(200, 102, 0);
        cv::Vec3d color_people(61, 5, 150);
        if (color_bgr(0) == color_road(0) && color_bgr(1) == color_road(1) && color_bgr(2) == color_road(2))
        {
            return;
        }
        seg_point.x = points_in_cam_homo.at<double>(0, 0);
        seg_point.y = points_in_cam_homo.at<double>(1, 0);
        seg_point.z = points_in_cam_homo.at<double>(2, 0);
        seg_point.b = color_bgr(0);
        seg_point.g = color_bgr(1);
        seg_point.r = color_bgr(2);
        seg_cloud.push_back(seg_point);
    }
}

/* 点云分割函数2(negative x,positve y) */
void fusion::seg_nxpy(pcl::PointCloud<pcl::PointXYZ>::const_iterator &it)
{
    // 坐标变换后的点
    cv::Mat points_in_lidar_homo(4, 1, cv::DataType<double>::type);
    cv::Mat points_in_cam_homo(4, 1, cv::DataType<double>::type);
    cv::Mat pixel_homo(3, 1, cv::DataType<double>::type);
    cv::Point pixel;
    cv::Vec3b color_bgr;
    // 添加颜色后的点
    pcl::PointXYZRGB seg_point;
    // 把点云数据存到齐次坐标点X中
    points_in_lidar_homo.at<double>(0, 0) = it->x;
    points_in_lidar_homo.at<double>(1, 0) = it->y;
    points_in_lidar_homo.at<double>(2, 0) = it->z;
    points_in_lidar_homo.at<double>(3, 0) = 1;
    // 相机坐标下的点云 = 相机和雷达的外参 * 雷达坐标系下的齐次点云
    points_in_cam_homo = i_params.cam2tolidar * points_in_lidar_homo;
    // 像素坐标 = 内参矩阵 * 相机坐标下的点
    pixel_homo = i_params.camera2_in * points_in_cam_homo;
    // 归一化像素坐标
    pixel.x = pixel_homo.at<double>(0, 0) / pixel_homo.at<double>(2, 0);
    pixel.y = pixel_homo.at<double>(1, 0) / pixel_homo.at<double>(2, 0);
#ifdef DEBUG
    std::cout << "像素投影成功" << std::endl;
#endif
    // 视野外的点
    if (pixel.x < 0 || pixel.y < 0 || pixel.x > cv_ptr2->image.cols || pixel.y > cv_ptr2->image.rows)
    {
        seg_point.x = points_in_cam_homo.at<double>(0, 0);
        seg_point.y = points_in_cam_homo.at<double>(1, 0);
        seg_point.z = points_in_cam_homo.at<double>(2, 0);
        seg_point.r = 255;
        seg_point.g = 0;
        seg_point.b = 0;
        seg_cloud.push_back(seg_point);
    }
    // 视野内的点根据颜色进行分割
    else
    {
        // 拿到像素点的颜色，注意opencv的横纵坐标顺序，直接用cv::Point
        color_bgr = cv_ptr2->image.at<cv::Vec3b>(pixel);
        // 定义一些待删除的颜色
        cv::Vec3d color_road(140, 140, 140);
        cv::Vec3d color_car(200, 102, 0);
        cv::Vec3d color_people(61, 5, 150);
        if (color_bgr(0) == color_road(0) && color_bgr(1) == color_road(1) && color_bgr(2) == color_road(2))
        {
            return;
        }
        seg_point.x = points_in_cam_homo.at<double>(0, 0);
        seg_point.y = points_in_cam_homo.at<double>(1, 0);
        seg_point.z = points_in_cam_homo.at<double>(2, 0);
        seg_point.b = color_bgr(0);
        seg_point.g = color_bgr(1);
        seg_point.r = color_bgr(2);
        seg_cloud.push_back(seg_point);
    }
}

/* 点云分割函数3(negative x,negative y) */
void fusion::seg_nxny(pcl::PointCloud<pcl::PointXYZ>::const_iterator &it)
{
    // 坐标变换后的点
    cv::Mat points_in_lidar_homo(4, 1, cv::DataType<double>::type);
    cv::Mat points_in_cam_homo(4, 1, cv::DataType<double>::type);
    cv::Mat pixel_homo(3, 1, cv::DataType<double>::type);
    cv::Point pixel;
    cv::Vec3b color_bgr;
    // 添加颜色后的点
    pcl::PointXYZRGB seg_point;
    // 把点云数据存到齐次坐标点X中
    points_in_lidar_homo.at<double>(0, 0) = it->x;
    points_in_lidar_homo.at<double>(1, 0) = it->y;
    points_in_lidar_homo.at<double>(2, 0) = it->z;
    points_in_lidar_homo.at<double>(3, 0) = 1;
    // 相机坐标下的点云 = 相机和雷达的外参 * 雷达坐标系下的齐次点云
    points_in_cam_homo = i_params.cam3tolidar * points_in_lidar_homo;
    // 像素坐标 = 内参矩阵 * 相机坐标下的点
    pixel_homo = i_params.camera3_in * points_in_cam_homo;
    // 归一化像素坐标
    pixel.x = pixel_homo.at<double>(0, 0) / pixel_homo.at<double>(2, 0);
    pixel.y = pixel_homo.at<double>(1, 0) / pixel_homo.at<double>(2, 0);
#ifdef DEBUG
    std::cout << "像素投影成功" << std::endl;
#endif
    // 视野外的点
    if (pixel.x < 0 || pixel.y < 0 || pixel.x > cv_ptr3->image.cols || pixel.y > cv_ptr3->image.rows)
    {
        seg_point.x = points_in_cam_homo.at<double>(0, 0);
        seg_point.y = points_in_cam_homo.at<double>(1, 0);
        seg_point.z = points_in_cam_homo.at<double>(2, 0);
        seg_point.r = 255;
        seg_point.g = 0;
        seg_point.b = 0;
        seg_cloud.push_back(seg_point);
    }
    // 视野内的点根据颜色进行分割
    else
    {
        // 拿到像素点的颜色，注意opencv的横纵坐标顺序，直接用cv::Point最保险
        color_bgr = cv_ptr3->image.at<cv::Vec3b>(pixel);
        // 定义一些待删除的颜色
        cv::Vec3d color_road(140, 140, 140);
        cv::Vec3d color_car(200, 102, 0);
        cv::Vec3d color_people(61, 5, 150);
        if (color_bgr(0) == color_road(0) && color_bgr(1) == color_road(1) && color_bgr(2) == color_road(2))
        {
            return;
        }
        seg_point.x = points_in_cam_homo.at<double>(0, 0);
        seg_point.y = points_in_cam_homo.at<double>(1, 0);
        seg_point.z = points_in_cam_homo.at<double>(2, 0);
        seg_point.b = color_bgr(0);
        seg_point.g = color_bgr(1);
        seg_point.r = color_bgr(2);
        seg_cloud.push_back(seg_point);
    }
}

/* 点云分割函数4(positve x,negative y) */
void fusion::seg_pxny(pcl::PointCloud<pcl::PointXYZ>::const_iterator &it)
{
    // 坐标变换后的点
    cv::Mat points_in_lidar_homo(4, 1, cv::DataType<double>::type);
    cv::Mat points_in_cam_homo(4, 1, cv::DataType<double>::type);
    cv::Mat pixel_homo(3, 1, cv::DataType<double>::type);
    cv::Point pixel;
    cv::Vec3b color_bgr;
    // 添加颜色后的点
    pcl::PointXYZRGB seg_point;
    // 把点云数据存到齐次坐标点X中
    points_in_lidar_homo.at<double>(0, 0) = it->x;
    points_in_lidar_homo.at<double>(1, 0) = it->y;
    points_in_lidar_homo.at<double>(2, 0) = it->z;
    points_in_lidar_homo.at<double>(3, 0) = 1;
    // 相机坐标下的点云 = 相机和雷达的外参 * 雷达坐标系下的齐次点云
    points_in_cam_homo = i_params.cam1tolidar * points_in_lidar_homo;
    // 像素坐标 = 内参矩阵 * 相机坐标下的点
    pixel_homo = i_params.camera1_in * points_in_cam_homo;
    // 归一化像素坐标
    pixel.x = pixel_homo.at<double>(0, 0) / pixel_homo.at<double>(2, 0);
    pixel.y = pixel_homo.at<double>(1, 0) / pixel_homo.at<double>(2, 0);
#ifdef DEBUG
    std::cout << "像素投影成功" << std::endl;
#endif
    // 视野外的点
    if (pixel.x < 0 || pixel.y < 0 || pixel.x > cv_ptr4->image.cols || pixel.y > cv_ptr4->image.rows)
    {
        seg_point.x = points_in_cam_homo.at<double>(0, 0);
        seg_point.y = points_in_cam_homo.at<double>(1, 0);
        seg_point.z = points_in_cam_homo.at<double>(2, 0);
        seg_point.r = 255;
        seg_point.g = 0;
        seg_point.b = 0;
        seg_cloud.push_back(seg_point);
    }
    // 视野内的点根据颜色进行分割
    else
    {
        // 拿到像素点的颜色，注意opencv的横纵坐标顺序，直接用cv::Point最保险
        color_bgr = cv_ptr4->image.at<cv::Vec3b>(pixel);
        // 定义一些待删除的颜色
        cv::Vec3d color_road(140, 140, 140);
        cv::Vec3d color_car(200, 102, 0);
        cv::Vec3d color_people(61, 5, 150);
        if (color_bgr(0) == color_road(0) && color_bgr(1) == color_road(1) && color_bgr(2) == color_road(2))
        {
            return;
        }
        seg_point.x = points_in_cam_homo.at<double>(0, 0);
        seg_point.y = points_in_cam_homo.at<double>(1, 0);
        seg_point.z = points_in_cam_homo.at<double>(2, 0);
        seg_point.b = color_bgr(0);
        seg_point.g = color_bgr(1);
        seg_point.r = color_bgr(2);
        seg_cloud.push_back(seg_point);
    }
}

fusion::fusion()
{
    ros::NodeHandle nh("~");
    // 读取参数
    initParams();

    // 四个相机和雷达的topic
    message_filters::Subscriber<sensor_msgs::Image> image_sub_1(nh, i_params.camera_topic_1, 5);
    message_filters::Subscriber<sensor_msgs::Image> image_sub_2(nh, i_params.camera_topic_2, 5);
    message_filters::Subscriber<sensor_msgs::Image> image_sub_3(nh, i_params.camera_topic_3, 5);
    message_filters::Subscriber<sensor_msgs::Image> image_sub_4(nh, i_params.camera_topic_4, 5);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, i_params.lidar_topic, 5);
    // 同步器
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), image_sub_1, image_sub_2, image_sub_3, image_sub_4, pc_sub);
    sync.registerCallback(boost::bind(&fusion::fusion_callback, this, _1, _2, _3, _4, _5));

    // 输出投影效果
    image_transport::ImageTransport imageTransport(nh);
    image_publisher = imageTransport.advertise("image_plus_cloud", 20);

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