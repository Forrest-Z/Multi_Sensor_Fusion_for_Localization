#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>

// 定义的三个srv
#include <hdl_global_localization/SetGlobalLocalizationEngine.h>
#include <hdl_global_localization/SetGlobalMap.h>
#include <hdl_global_localization/QueryGlobalLocalization.h>

class GlobalLocalizationTestNode {
public:
  GlobalLocalizationTestNode() : nh() {
    // 注册三个服务的client
    set_engine_service = nh.serviceClient<hdl_global_localization::SetGlobalLocalizationEngine>("/hdl_global_localization/set_engine");
    set_global_map_service = nh.serviceClient<hdl_global_localization::SetGlobalMap>("/hdl_global_localization/set_global_map");
    query_service = nh.serviceClient<hdl_global_localization::QueryGlobalLocalization>("/hdl_global_localization/query");

    globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 1, true);  // 第三个参数开启latch,相当于静态话题
    points_pub = nh.advertise<sensor_msgs::PointCloud2>("/aligned_points", 1);
    points_sub = nh.subscribe(nh.param<std::string>("points_topic", "/velodyne_points"), 1, &GlobalLocalizationTestNode::points_callback, this);
    // 发布匹配结果
    error_pub = nh.advertise<std_msgs::Float64>("/error", 1);
    inlier_fraction_pub = nh.advertise<std_msgs::Float64>("/inlier_fraction", 1);

    set_engine(nh.param<std::string>("engine_name", "FPFH_RANSAC"));
    // 拿到全局地图
    global_map.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::io::loadPCDFile(nh.param<std::string>("global_map_path", "/home/sean/ROS/seg_ws/src/localize/hdl_localization/data/6_new_mesh.pcd"), *global_map);
    global_map->header.frame_id = "map";
    
    // 降采样
    double global_map_downsample_resolution = nh.param<double>("global_map_downsample_resolution", 0.1);
    boost::shared_ptr<pcl::VoxelGrid<pcl::PointXYZ>> voxelgrid(new pcl::VoxelGrid<pcl::PointXYZ>());
    voxelgrid->setLeafSize(global_map_downsample_resolution, global_map_downsample_resolution, global_map_downsample_resolution);
    voxelgrid->setInputCloud(global_map);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    voxelgrid->filter(*filtered);
    global_map = filtered;

    set_global_map(global_map);
  }

  // 设置全局定位方式
  void set_engine(const std::string& engine_name) {
    hdl_global_localization::SetGlobalLocalizationEngine srv;
    srv.request.engine_name.data = engine_name;

    if (!set_engine_service.call(srv)) {
      ROS_INFO_STREAM("Failed to set global localization engine");
    } else {
      ROS_INFO_STREAM("Succeed to set global localization engine:" << engine_name);
    }
  }

  // 设置全局地图
  void set_global_map(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // 通过服务把全局地图发给engine
    hdl_global_localization::SetGlobalMap srv;
    pcl::toROSMsg(*cloud, srv.request.global_map);

    if (!set_global_map_service.call(srv)) {
      ROS_INFO_STREAM("Failed to set global map");
    } else {
      ROS_INFO_STREAM("Succeed to set global map");
    }
    // 发布latched点云
    globalmap_pub_timer = nh.createWallTimer(ros::WallDuration(1.0), &GlobalLocalizationTestNode::pub_once_cb, this, true, true);
  }

  void pub_once_cb(const ros::WallTimerEvent& event) { globalmap_pub.publish(global_map); }

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
    if (srv.response.inlier_fractions[0] > nh.param<double>("output_inlier_fraction_threshold", 0.95)) {
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

  ros::Publisher points_pub;
  ros::Subscriber points_sub;

  // 发布定位结果
  ros::Publisher inlier_fraction_pub;
  ros::Publisher error_pub;

  // 发布全局地图
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map;
  ros::Publisher globalmap_pub;
  ros::WallTimer globalmap_pub_timer;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "hdl_global_localization_test");
  GlobalLocalizationTestNode node;
  ros::spin();
}