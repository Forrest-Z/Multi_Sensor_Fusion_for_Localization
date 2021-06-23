#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

ros::Publisher pub_;
sensor_msgs::ImuPtr imu_;
Eigen::Matrix3d rotation_matrix;
std::string target_frame_;

void transformCovariance(const boost::array<double, 9> &in, boost::array<double, 9> &out, Eigen::Quaternion<double> r)
{

    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> cov_in(in.data());
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> cov_out(out.data());
    cov_out = r * cov_in * r.inverse();
}

/**
* Transforms sensor_msgs::Imu data from one frame to another
*/
void doTransform(const sensor_msgs::Imu &imu_in, sensor_msgs::Imu &imu_out, const geometry_msgs::TransformStamped &t_in)
{

    imu_out.header = t_in.header;

    // Discard translation, only use orientation for IMU transform
    Eigen::Quaternion<double> r(
        t_in.transform.rotation.w, t_in.transform.rotation.x, t_in.transform.rotation.y, t_in.transform.rotation.z);
    Eigen::Transform<double, 3, Eigen::Affine> t(r);

    Eigen::Vector3d vel = t * Eigen::Vector3d(
                                  imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);

    imu_out.angular_velocity.x = vel.x();
    imu_out.angular_velocity.y = vel.y();
    imu_out.angular_velocity.z = vel.z();

    transformCovariance(imu_in.angular_velocity_covariance, imu_out.angular_velocity_covariance, r);

    Eigen::Vector3d accel = t * Eigen::Vector3d(
                                    imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);

    imu_out.linear_acceleration.x = accel.x();
    imu_out.linear_acceleration.y = accel.y();
    imu_out.linear_acceleration.z = accel.z();

    transformCovariance(imu_in.linear_acceleration_covariance, imu_out.linear_acceleration_covariance, r);

    Eigen::Quaternion<double> orientation = r * Eigen::Quaternion<double>(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z) * r.inverse();

    imu_out.orientation.w = orientation.w();
    imu_out.orientation.x = orientation.x();
    imu_out.orientation.y = orientation.y();
    imu_out.orientation.z = orientation.z();

    transformCovariance(imu_in.orientation_covariance, imu_out.orientation_covariance, r);
}

void imu_callback(const sensor_msgs::ImuConstPtr &msg)
{
    // 不变的
    imu_->header.seq = msg->header.seq;
    imu_->header.stamp = msg->header.stamp;
    imu_->orientation = msg->orientation;
    imu_->orientation_covariance = msg->orientation_covariance;
    imu_->angular_velocity_covariance = msg->angular_velocity_covariance;
    imu_->linear_acceleration_covariance = msg->linear_acceleration_covariance;

    // 变的
    imu_->header.frame_id = target_frame_;
    Eigen::Vector3d linear;
    linear << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    linear = rotation_matrix * linear;
    imu_->linear_acceleration.x = linear[0];
    imu_->linear_acceleration.y = linear[1];
    imu_->linear_acceleration.z = linear[2];
    // imu_->linear_acceleration = msg->linear_acceleration;

    Eigen::Vector3d angular;
    angular << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    angular = rotation_matrix * angular;
    imu_->angular_velocity.x = angular[0];
    imu_->angular_velocity.y = angular[1];
    imu_->angular_velocity.z = angular[2];

    // Publish transformed message
    pub_.publish(imu_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_trans_node");
    ros::NodeHandle n;

    // Get parameters
    n.param<std::string>("target_frame", target_frame_, "velodyne");
    imu_.reset(new sensor_msgs::Imu());

    rotation_matrix = Eigen::AngleAxisd(-1.607, Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(-0.006, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(-2.336, Eigen::Vector3d::UnitX());

    // Create publisher
    pub_ = n.advertise<sensor_msgs::Imu>("imu_transformed", 10);

    // Subscriber
    ros::Subscriber sub = n.subscribe<sensor_msgs::Imu>("/camera4/imu", 100, imu_callback);

    ros::spin();

    return 0;
}