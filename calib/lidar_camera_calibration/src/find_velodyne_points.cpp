// ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// 自定义的头文件
#include "lidar_camera_calibration/get_corners.h"
#include "lidar_camera_calibration/get_RT.h"
#include "lidar_camera_calibration/utils_config.h"
#include "lidar_camera_calibration/utils_pcl.h"
// msg生成的头文件
#include "lidar_camera_calibration/marker_6dof.h"

// 全局参数
config_data config;
extern int iteration_counter = 0;

void callback(const sensor_msgs::PointCloud2ConstPtr &msg_pc,
              const lidar_camera_calibration::marker_6dof::ConstPtr &msg_rt)
{
    pcl::PointCloud<myPointXYZRID> point_cloud;
    fromROSMsg(*msg_pc, point_cloud);

    // 点云按照 初始旋转平移 变换到相机坐标系下面
    point_cloud = transform(point_cloud, config.initialTra[0], config.initialTra[1], config.initialTra[2], config.initialRot[0], config.initialRot[1], config.initialRot[2]);
    // XYZ欧拉角->四元数
    Eigen::Quaterniond qlidarToCamera = Eigen::AngleAxisd(config.initialRot[2], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(config.initialRot[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(config.initialRot[0], Eigen::Vector3d::UnitX());
    // 四元数->旋转矩阵
    Eigen::Matrix3d lidarToCamera = qlidarToCamera.matrix();
    std::cout << "初始旋转矩阵(3x3):\n"
              << lidarToCamera << std::endl;

    // 点云已经按照估计的初始参数变换到了相机坐标系下面
    // 按点云强度和xyz值过滤
    // xyz是在相机坐标系下，即像素平面x向右，y向下，z向前
    point_cloud = intensityByRangeDiff(point_cloud, config);
    pcl::PointCloud<pcl::PointXYZ> retval = *(toPointsXYZ(point_cloud));

    std::vector<float> marker_info;
    std::cout << "marker的位姿:\n";
    for (auto it = msg_rt->dof.data.begin(); it != msg_rt->dof.data.end(); ++it)
    {
        marker_info.push_back(*it);
        std::cout << *it << " ";
    }
    std::cout << std::endl;

    cv::Mat temp_mat(config.s, CV_8UC3);
    // 在点云和图像中都找到角点
    if (getCorners(temp_mat, retval, config.P, config.num_of_markers, config.MAX_ITERS))
    {
        // 找外参
        find_transformation(marker_info, config.num_of_markers, config.MAX_ITERS, lidarToCamera);
    }
}

int main(int argc, char **argv)
{
    // 读取参数
    read_config(config);
    print_config(config);
    // ROS初始化
    ros::init(argc, argv, "find_transform");
    ros::NodeHandle n;
    std::string VELODYNE_TOPIC;
    n.getParam("/lidar_camera_calibration/velodyne_topic", VELODYNE_TOPIC);

    // 订阅点云和aruco_mapping得到的marker位姿
    // 同步处理
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, VELODYNE_TOPIC, 1);
    message_filters::Subscriber<lidar_camera_calibration::marker_6dof> rt_sub(n, "lidar_camera_calibration_rt", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, lidar_camera_calibration::marker_6dof> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, rt_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return EXIT_SUCCESS;
}
