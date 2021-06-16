
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
// #include "../include/imu_transformer/tf2_sensor_msgs.h" // 必须用这个头文件
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <message_filters/subscriber.h>

class imu_trans
{
public:
    imu_trans() : tf2_(buffer_), target_frame_("velodyne"),
                  imu_filter_(imu_sub_, buffer_, target_frame_, 10, 0)
    {
        n_.param<std::string>("imu_in", imu_in_topic_, "/camera4/imu");
        imu_sub_.subscribe(n_, imu_in_topic_, 10);
        imu_pub_ = n_.advertise<sensor_msgs::Imu>("out", 10);
        imu_filter_.registerCallback(boost::bind(&imu_trans::imu_cb, this, _1));
        std::cout << "初始化" << std::endl;
    }

    void imu_cb(const sensor_msgs::ImuConstPtr &imu_in)
    {
        std::cout << imu_in->header.frame_id << std::endl;
        sensor_msgs::Imu imu_out;
        try
        {
            buffer_.transform(*imu_in, imu_out, target_frame_);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Failure %s\n", ex.what());
        }
    }

private:
    ros::NodeHandle n_;

    tf2_ros::MessageFilter<sensor_msgs::Imu> imu_filter_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf2_;

    ros::Publisher imu_pub_;

    std::string imu_in_topic_;
    std::string target_frame_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_trans");
    std::cout << "初始化1" << std::endl;
    imu_trans imu_trans_node;
    std::cout << "初始化2" << std::endl;
    ros::spin();
    return 0;
};