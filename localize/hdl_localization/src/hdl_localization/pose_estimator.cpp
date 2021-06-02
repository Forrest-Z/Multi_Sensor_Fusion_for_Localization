#include <hdl_localization/pose_estimator.hpp>

#include <pcl/filters/voxel_grid.h>
#include <hdl_localization/pose_system.hpp>
#include <hdl_localization/odom_system.hpp>
#include <kkl/alg/unscented_kalman_filter.hpp>

namespace hdl_localization {

/**
 * @brief constructor
 * @param registration        registration method
 * @param stamp               timestamp
 * @param pos                 initial position
 * @param quat                initial orientation
 * @param cool_time_duration  during "cool time", prediction is not performed
 */
PoseEstimator::PoseEstimator(
  pcl::Registration<PointT, PointT>::Ptr& registration,
  const ros::Time& stamp,
  const Eigen::Vector3f& pos,
  const Eigen::Quaternionf& quat,
  double cool_time_duration)
: init_stamp(stamp),
  registration(registration),
  cool_time_duration(cool_time_duration) {
  last_observation = Eigen::Matrix4f::Identity();
  last_observation.block<3, 3>(0, 0) = quat.toRotationMatrix();
  last_observation.block<3, 1>(0, 3) = pos;
  // 过程噪声矩阵初始化，后面还要乘以间隔时间再传入ukf(Q矩阵)
  // TODO:需要调参
  process_noise = Eigen::MatrixXf::Identity(16, 16);
  process_noise.middleRows(0, 3) *= 1.0;
  process_noise.middleRows(3, 3) *= 1.0;
  process_noise.middleRows(6, 4) *= 0.5;
  process_noise.middleRows(10, 3) *= 1e-6;
  process_noise.middleRows(13, 3) *= 1e-6;
  // 观测噪声初始化(R矩阵)
  // TODO:需要调参
  Eigen::MatrixXf measurement_noise = Eigen::MatrixXf::Identity(7, 7);
  measurement_noise.middleRows(0, 3) *= 0.01;
  measurement_noise.middleRows(3, 4) *= 0.001;
  // 状态量均值
  Eigen::VectorXf mean(16);
  mean.middleRows(0, 3) = pos;
  mean.middleRows(3, 3).setZero();
  mean.middleRows(6, 4) = Eigen::Vector4f(quat.w(), quat.x(), quat.y(), quat.z());
  mean.middleRows(10, 3).setZero();
  mean.middleRows(13, 3).setZero();
  // 初始化协方差矩阵
  Eigen::MatrixXf cov = Eigen::MatrixXf::Identity(16, 16) * 0.01;
  PoseSystem system;
  // 最重要的:初始化ukf
  // ukf是类模板，这里指定了模板参数system为PoseSystem
  ukf.reset(new kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>(system, 16, 6, 7, process_noise, measurement_noise, mean, cov));
}

PoseEstimator::~PoseEstimator() {}

/**
 * @brief predict
 * 没有IMU输入的时候的位姿估计方法
 * @param stamp    timestamp
 * @param acc      acceleration
 * @param gyro     angular velocity
 */
void PoseEstimator::predict(const ros::Time& stamp) {
  if ((stamp - init_stamp).toSec() < cool_time_duration || prev_stamp.is_zero() || prev_stamp == stamp) {
    prev_stamp = stamp;
    return;
  }

  double dt = (stamp - prev_stamp).toSec();
  prev_stamp = stamp;

  ukf->setProcessNoiseCov(process_noise * dt);
  ukf->system.dt = dt;

  ukf->predict();
}

/**
 * @brief predict
 * 有IMU输入的时候的位姿估计方法
 * @param stamp    timestamp        时间戳
 * @param acc      acceleration     加速度
 * @param gyro     angular velocity 角速度
 */
void PoseEstimator::predict(const ros::Time& stamp, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro) {
  // 1.当前与初始化的时间间隔小于设置的时间
  // 2.上次更新时间为0（未更新）
  // 3.prev_stamp等于当前时间
  // 更新prev_stamp并跳出
  if ((stamp - init_stamp).toSec() < cool_time_duration || prev_stamp.is_zero() || prev_stamp == stamp) {
    prev_stamp = stamp;
    return;
  }
  // 先计算dt，再更新prev_stamp
  double dt = (stamp - prev_stamp).toSec();
  prev_stamp = stamp;
  // 设置ukf的Q矩阵
  ukf->setProcessNoiseCov(process_noise * dt);
  // 设置采样时间
  ukf->system.dt = dt;
  // 用imu数据构造控制量
  Eigen::VectorXf control(6);
  control.head<3>() = acc;
  control.tail<3>() = gyro;
  // 有控制量的ukf预测更新
  ukf->predict(control);
}

/**
 * @brief update the state of the odomety-based pose estimation
 * @param odom_delta
 * @todo 这个借口可以扩展出其他的融合形式
 * @attention odom的系统状态量和imu系统的不一样
 */
void PoseEstimator::predict_odom(const Eigen::Matrix4f& odom_delta) {
  // 初始化一个使用odom系统状态量的卡尔曼滤波智能指针
  if (!odom_ukf) {
    // 两个误差矩阵
    Eigen::MatrixXf odom_process_noise = Eigen::MatrixXf::Identity(7, 7);
    Eigen::MatrixXf odom_measurement_noise = Eigen::MatrixXf::Identity(7, 7) * 1e-3;

    // 均值和协方差矩阵初值
    Eigen::VectorXf odom_mean(7);
    odom_mean.block<3, 1>(0, 0) = Eigen::Vector3f(ukf->mean[0], ukf->mean[1], ukf->mean[2]);
    odom_mean.block<4, 1>(3, 0) = Eigen::Vector4f(ukf->mean[6], ukf->mean[7], ukf->mean[8], ukf->mean[9]);
    Eigen::MatrixXf odom_cov = Eigen::MatrixXf::Identity(7, 7) * 1e-2;

    // odom和pose的系统状态量不同，传给ukf类模板system参数不同
    OdomSystem odom_system;
    odom_ukf.reset(new kkl::alg::UnscentedKalmanFilterX<float, OdomSystem>(odom_system, 7, 7, 7, odom_process_noise, odom_measurement_noise, odom_mean, odom_cov));
  }

  // invert quaternion if the rotation axis is flipped
  Eigen::Quaternionf quat(odom_delta.block<3, 3>(0, 0));
  if (odom_quat().coeffs().dot(quat.coeffs()) < 0.0) {
    quat.coeffs() *= -1.0f;
  }

  // 控制量
  Eigen::VectorXf control(7);
  control.middleRows(0, 3) = odom_delta.block<3, 1>(0, 3);
  control.middleRows(3, 4) = Eigen::Vector4f(quat.w(), quat.x(), quat.y(), quat.z());

  Eigen::MatrixXf process_noise = Eigen::MatrixXf::Identity(7, 7);
  process_noise.topLeftCorner(3, 3) = Eigen::Matrix3f::Identity() * odom_delta.block<3, 1>(0, 3).norm() + Eigen::Matrix3f::Identity() * 1e-3;
  process_noise.bottomRightCorner(4, 4) = Eigen::Matrix4f::Identity() * (1 - std::abs(quat.w())) + Eigen::Matrix4f::Identity() * 1e-3;

  odom_ukf->setProcessNoiseCov(process_noise);
  odom_ukf->predict(control);
}

/**
 * @brief correct
 * @param stamp
 * @param cloud input cloud
 * @return cloud aligned to the globalmap
 */
pcl::PointCloud<PoseEstimator::PointT>::Ptr PoseEstimator::correct(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud) {
  last_correction_stamp = stamp;
  Eigen::Matrix4f no_guess = last_observation;
  Eigen::Matrix4f imu_guess;
  Eigen::Matrix4f odom_guess;
  // 单位阵初始化
  Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();

  if (!odom_ukf) {
    // 没有odom就用predict得到的结果作为ndt匹配初值
    init_guess = imu_guess = matrix();
  }
  // 使用里程计的话
  // ndt匹配的初值是odom和imu的融合
  else {
    imu_guess = matrix();
    odom_guess = odom_matrix();

    Eigen::VectorXf imu_mean(7);
    Eigen::MatrixXf imu_cov = Eigen::MatrixXf::Identity(7, 7);
    imu_mean.block<3, 1>(0, 0) = ukf->mean.block<3, 1>(0, 0);
    imu_mean.block<4, 1>(3, 0) = ukf->mean.block<4, 1>(6, 0);

    imu_cov.block<3, 3>(0, 0) = ukf->cov.block<3, 3>(0, 0);
    imu_cov.block<3, 4>(0, 3) = ukf->cov.block<3, 4>(0, 6);
    imu_cov.block<4, 3>(3, 0) = ukf->cov.block<4, 3>(6, 0);
    imu_cov.block<4, 4>(3, 3) = ukf->cov.block<4, 4>(6, 6);

    Eigen::VectorXf odom_mean = odom_ukf->mean;
    Eigen::MatrixXf odom_cov = odom_ukf->cov;

    if (imu_mean.tail<4>().dot(odom_mean.tail<4>()) < 0.0) {
      odom_mean.tail<4>() *= -1.0;
    }

    Eigen::MatrixXf inv_imu_cov = imu_cov.inverse();
    Eigen::MatrixXf inv_odom_cov = odom_cov.inverse();

    Eigen::MatrixXf fused_cov = (inv_imu_cov + inv_odom_cov).inverse();
    Eigen::VectorXf fused_mean = fused_cov * inv_imu_cov * imu_mean + fused_cov * inv_odom_cov * odom_mean;

    init_guess.block<3, 1>(0, 3) = Eigen::Vector3f(fused_mean[0], fused_mean[1], fused_mean[2]);
    init_guess.block<3, 3>(0, 0) = Eigen::Quaternionf(fused_mean[3], fused_mean[4], fused_mean[5], fused_mean[6]).normalized().toRotationMatrix();
  }

  // 点云的配准,registration指针已经指定了匹配方法和目标全局地图
  // 注意这里得到的结果是相对于点云地图坐标系的，也就是观测结果是一个全局坐标
  pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());  // 匹配后的点云
  registration->setInputSource(cloud);
  registration->align(*aligned, init_guess);

  Eigen::Matrix4f trans = registration->getFinalTransformation();  // 匹配得到的结果
  Eigen::Vector3f p = trans.block<3, 1>(0, 3);
  Eigen::Quaternionf q(trans.block<3, 3>(0, 0));
  // TODO:这个判断预测和观测的姿态四元数点乘小于0干嘛的?
  if (quat().coeffs().dot(q.coeffs()) < 0.0f) {
    q.coeffs() *= -1.0f;
  }
  // 观测值
  Eigen::VectorXf observation(7);
  observation.middleRows(0, 3) = p;
  observation.middleRows(3, 4) = Eigen::Vector4f(q.w(), q.x(), q.y(), q.z());
  last_observation = trans;  // 保存上一次观测结果

  // TODO:这两个erro干啥的
  wo_pred_error = no_guess.inverse() * registration->getFinalTransformation();
  // ukf更新
  ukf->correct(observation);
  imu_pred_error = imu_guess.inverse() * registration->getFinalTransformation();

  // 先不看
  if (odom_ukf) {
    if (observation.tail<4>().dot(odom_ukf->mean.tail<4>()) < 0.0) {
      odom_ukf->mean.tail<4>() *= -1.0;
    }
    odom_ukf->correct(observation);
    odom_pred_error = odom_guess.inverse() * registration->getFinalTransformation();
  }

  return aligned;
}

/* getters */
ros::Time PoseEstimator::last_correction_time() const {
  return last_correction_stamp;
}

Eigen::Vector3f PoseEstimator::pos() const {
  return Eigen::Vector3f(ukf->mean[0], ukf->mean[1], ukf->mean[2]);
}

Eigen::Vector3f PoseEstimator::vel() const {
  return Eigen::Vector3f(ukf->mean[3], ukf->mean[4], ukf->mean[5]);
}

Eigen::Quaternionf PoseEstimator::quat() const {
  return Eigen::Quaternionf(ukf->mean[6], ukf->mean[7], ukf->mean[8], ukf->mean[9]).normalized();
}

/**
 * @brief 返回当前ukf预测得到的位姿矩阵(4x4)
 */
Eigen::Matrix4f PoseEstimator::matrix() const {
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m.block<3, 3>(0, 0) = quat().toRotationMatrix();
  m.block<3, 1>(0, 3) = pos();
  return m;
}

Eigen::Vector3f PoseEstimator::odom_pos() const {
  return Eigen::Vector3f(odom_ukf->mean[0], odom_ukf->mean[1], odom_ukf->mean[2]);
}

Eigen::Quaternionf PoseEstimator::odom_quat() const {
  return Eigen::Quaternionf(odom_ukf->mean[3], odom_ukf->mean[4], odom_ukf->mean[5], odom_ukf->mean[6]).normalized();
}

Eigen::Matrix4f PoseEstimator::odom_matrix() const {
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m.block<3, 3>(0, 0) = odom_quat().toRotationMatrix();
  m.block<3, 1>(0, 3) = odom_pos();
  return m;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::wo_prediction_error() const {
  return wo_pred_error;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::imu_prediction_error() const {
  return imu_pred_error;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::odom_prediction_error() const {
  return odom_pred_error;
}
}  // namespace hdl_localization