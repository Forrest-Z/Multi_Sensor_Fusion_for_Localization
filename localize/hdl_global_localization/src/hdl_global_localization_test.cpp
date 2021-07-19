#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// 定义的三个srv
#include <hdl_global_localization/SetGlobalLocalizationEngine.h>
#include <hdl_global_localization/SetGlobalMap.h>
#include <hdl_global_localization/QueryGlobalLocalization.h>

class GlobalLocalizationTestNode {
public:
  GlobalLocalizationTestNode() : nh() {
    // nh.param<std::string>("points_topic", points_topic, "/velodyne_points");
    // nh.param<std::string>("engine", engine, "FPFH_RANSAC");
    // nh.param<std::string>("global_map_path", global_map_path, "/home/sean/ROS/seg_ws/src/localize/hdl_localization/data/6_new_mesh.pcd");

    // 注册三个服务的client
    set_engine_service = nh.serviceClient<hdl_global_localization::SetGlobalLocalizationEngine>("/hdl_global_localization/set_engine");
    set_global_map_service = nh.serviceClient<hdl_global_localization::SetGlobalMap>("/hdl_global_localization/set_global_map");
    query_service = nh.serviceClient<hdl_global_localization::QueryGlobalLocalization>("/hdl_global_localization/query");

    globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 1, true);  // 第三个参数开启latch,相当于静态话题
    points_pub = nh.advertise<sensor_msgs::PointCloud2>("/aligned_points", 1);
    points_sub = nh.subscribe("/velodyne_points", 1, &GlobalLocalizationTestNode::points_callback, this);
    // 发布匹配结果
    error_pub = nh.advertise<std_msgs::Float64>("/error", 1);
    inlier_fraction_pub = nh.advertise<std_msgs::Float64>("/inlier_fraction", 1);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::io::loadPCDFile(global_map_path, *global_map);
    // set_engine(engine);
    // set_global_map(global_map);
  }

  // 设置全局定位方式
  void set_engine(const std::string& engine_name) {
    hdl_global_localization::SetGlobalLocalizationEngine srv;
    srv.request.engine_name.data = engine_name;

    if (!set_engine_service.call(srv)) {
      ROS_INFO_STREAM("Failed to set global localization engine");
    }
  }

  // 设置全局地图
  void set_global_map(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    hdl_global_localization::SetGlobalMap srv;
    pcl::toROSMsg(*cloud, srv.request.global_map);

    if (!set_global_map_service.call(srv)) {
      ROS_INFO_STREAM("Failed to set global map");
    } else {
      ROS_INFO_STREAM("Succeed to set global map");
    }
    // 这里输出的点云没有降采样，但是在service的服务函数中会执行降采样
    cloud->header.frame_id = "map";
    globalmap_pub.publish(cloud);
  }

  void points_callback(sensor_msgs::PointCloud2ConstPtr cloud_msg) {
    ROS_INFO_STREAM("Callback:" << cloud_msg->header.seq);
    hdl_global_localization::QueryGlobalLocalization srv;
    srv.request.cloud = *cloud_msg;
    srv.request.max_num_candidates = 1;  // 可以得到几个全局定位位姿,srv的response是pose数组

    if (!query_service.call(srv) || srv.response.poses.empty()) {
      ROS_INFO_STREAM("Failed to find a global localization solution");
      return;
    }

    // 输出匹配的位姿
    const auto& estimated = srv.response.poses[0];
    Eigen::Quaternionf quat(estimated.orientation.w, estimated.orientation.x, estimated.orientation.y, estimated.orientation.z);
    Eigen::Vector3f trans(estimated.position.x, estimated.position.y, estimated.position.z);

    Eigen::Isometry3f transformation = Eigen::Isometry3f::Identity();
    transformation.linear() = quat.toRotationMatrix();
    transformation.translation() = trans;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud, *transformed, transformation);
    // 点云上色
    std::uint8_t r, g, b;
    r = 255, g = 0, b = 0;  // Red color
    std::uint32_t red_color = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
    r = 0, g = 255, b = 0;  // Green color
    std::uint32_t green_color = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
    if (srv.response.inlier_fractions[0] > nh.param<double>("output_inlier_fraction_threshold", 0.90)) {
      for (auto& point : transformed->points) {
        point.rgb = *reinterpret_cast<float*>(&green_color);
      }
    } else {
      for (auto& point : transformed->points) {
        point.rgb = *reinterpret_cast<float*>(&red_color);
      }
    }
    transformed->header.frame_id = "map";

    // 发布全局定位后的点云
    points_pub.publish(transformed);
    // 输出匹配误差和内点比例
    std_msgs::Float64 error_msg, inlier_fractions_msg;
    error_msg.data = srv.response.errors[0];
    inlier_fractions_msg.data = (double)srv.response.inlier_fractions[0];
    error_pub.publish(error_msg);
    inlier_fraction_pub.publish(inlier_fractions_msg);
    ROS_INFO_STREAM("error: " << srv.response.errors[0] << " inlier_fraction: " << srv.response.inlier_fractions[0]);
  }

private:
  ros::NodeHandle nh;
  ros::ServiceClient set_engine_service;
  ros::ServiceClient set_global_map_service;
  ros::ServiceClient query_service;

  ros::Publisher globalmap_pub;
  ros::Publisher points_pub;
  ros::Subscriber points_sub;

  // 发布定位结果
  ros::Publisher inlier_fraction_pub;
  ros::Publisher error_pub;

  // std::string points_topic;
  // std::string global_map_path;
  // std::string engine;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "hdl_global_localization_test");

  // 导入本地全局地图
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZ>);
  if (argc == 2) {
    pcl::io::loadPCDFile(argv[1], *global_map);
  } else {
    pcl::io::loadPCDFile("/home/sean/ROS/seg_ws/src/localize/hdl_localization/data/6_new_mesh.pcd", *global_map);
  }

  GlobalLocalizationTestNode node;
  // 配置参数搜索方式和全局地图
  node.set_engine("FPFH_RANSAC");  // BBS, FPFH_RANSAC, FPFH_TEASER
  node.set_global_map(global_map);

  ros::spin();
}