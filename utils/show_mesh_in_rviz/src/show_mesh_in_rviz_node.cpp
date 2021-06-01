#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "load_mesh");
    ros::NodeHandle nh;
    ros::Rate rate(1);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("building_mesh", 1);

    while (ros::ok())
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "mesh";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        // 位置和姿态
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        // 缩放
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        // 颜色和透明度
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.6;

        marker.lifetime = ros::Duration();

        marker.mesh_resource = "package://show_mesh_in_rviz/stl/6_v2.stl";
        marker_pub.publish(marker);

        rate.sleep();
    }
}
