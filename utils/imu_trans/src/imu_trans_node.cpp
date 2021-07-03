#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <iostream>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

ros::Publisher pub_;
sensor_msgs::ImuPtr imu_;
std::string target_frame_;
std::string imu_frame_;
Eigen::Quaterniond orientation;
Eigen::Quaterniond r;
Eigen::Transform<double, 3, Eigen::Affine> t;
Eigen::Vector3d accel_;
Eigen::Vector3d vel_;

void transformCovariance(const boost::array<double, 9> &in, boost::array<double, 9> &out)
{
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> cov_in(in.data());
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> cov_out(out.data());
    cov_out = r * cov_in * r.inverse();
}

void imu_callback(const sensor_msgs::ImuConstPtr &msg)
{
    // header
    imu_->header.seq = msg->header.seq;
    imu_->header.stamp = msg->header.stamp;
    imu_->header.frame_id = target_frame_;

    // orientation
    orientation = r * Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z) * r.inverse();
    imu_->orientation.w = orientation.w();
    imu_->orientation.x = orientation.x();
    imu_->orientation.y = orientation.y();
    imu_->orientation.z = orientation.z();
    transformCovariance(msg->orientation_covariance, imu_->orientation_covariance);

    // accel
    accel_ = t * Eigen::Vector3d(msg->linear_acceleration.x,
                                 msg->linear_acceleration.y,
                                 msg->linear_acceleration.z);
    imu_->linear_acceleration.x = accel_.x();
    imu_->linear_acceleration.y = accel_.y();
    imu_->linear_acceleration.z = accel_.z();
    transformCovariance(msg->linear_acceleration_covariance, imu_->linear_acceleration_covariance);

    // vel
    vel_ = t * Eigen::Vector3d(msg->angular_velocity.x,
                               msg->angular_velocity.y,
                               msg->angular_velocity.z);
    imu_->angular_velocity.x = vel_.x();
    imu_->angular_velocity.y = vel_.y();
    imu_->angular_velocity.z = vel_.z();
    transformCovariance(msg->angular_velocity_covariance, imu_->angular_velocity_covariance);

    // Publish transformed message
    pub_.publish(imu_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_trans_node");
    ros::NodeHandle n;

    // Get parameters
    n.param<std::string>("target_frame", target_frame_, "velodyne");
    n.param<std::string>("imu_frame", imu_frame_, "camera4_imu_optical_frame");

    imu_.reset(new sensor_msgs::Imu());

    // 得到旋转矩阵
    // base->imu
    // in Quaternion [-0.284, 0.661, -0.639, 0.270]
    // in RPY (radian) [-1.623, -0.805, -3.097]
    // x: 0.284117
    // y: -0.66144
    // z: 0.639444
    // w: -0.269975

    // imu->base
    // in Quaternion [-0.284, 0.661, -0.639, -0.270]
    // x: -0.284117
    // y: 0.66144
    // z: -0.639444
    // w: -0.269975
    geometry_msgs::TransformStamped t_in;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tfListener(tf_buffer);

    // 这里就是为了防止tf读取的是0
    while (true)
    {
        if (tf_buffer.canTransform(target_frame_, imu_frame_, ros::Time(0)))
        {
            t_in = tf_buffer.lookupTransform(target_frame_, imu_frame_, ros::Time(0));
        }
        if (t_in.transform.rotation.x != 0)
        {
            break;
        }
    }

    // std::cout << t_in.transform.rotation << std::endl;

    Eigen::Quaterniond r(t_in.transform.rotation.w,
                         t_in.transform.rotation.x,
                         t_in.transform.rotation.y,
                         t_in.transform.rotation.z);
    t = Eigen::Transform<double, 3, Eigen::Affine>(r);

    // Create publisher
    pub_ = n.advertise<sensor_msgs::Imu>("imu_transformed", 10);

    // Subscriber
    ros::Subscriber sub = n.subscribe<sensor_msgs::Imu>("/camera4/imu", 100, imu_callback);

    ros::spin();

    return 0;
}