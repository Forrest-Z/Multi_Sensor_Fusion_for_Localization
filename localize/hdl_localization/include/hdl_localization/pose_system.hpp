#ifndef POSE_SYSTEM_HPP
#define POSE_SYSTEM_HPP

#include <kkl/alg/unscented_kalman_filter.hpp>

namespace hdl_localization {

/**
 * @brief Definition of system to be estimated by ukf
 * @note state = [px, py, pz, vx, vy, vz, qw, qx, qy, qz, acc_bias_x, acc_bias_y, acc_bias_z, gyro_bias_x, gyro_bias_y, gyro_bias_z]
 * 16维系统状态量(待估计的量)   位置1-3 速度4-6 姿态四元数7-10 加速度偏置11-13 角速度偏置14-16
 *  7维测量状态量(ndt匹配结果)  位置1-3 姿态四元数4-7
 *  6维输入控制量(IMU输入)     加速度1-3 角速度4-6
 */
class PoseSystem {
public:
  typedef float T;
  typedef Eigen::Matrix<T, 3, 1> Vector3t;
  typedef Eigen::Matrix<T, 4, 4> Matrix4t;
  typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXt;
  typedef Eigen::Quaternion<T> Quaterniont;
  double dt;

public:
  PoseSystem() { dt = 0.01; }

  /**
   * @brief system equation (without input)
   * 系统的状态转移方程(无控制量输入)
   * @param state
   * @return VectorXt
   */
  VectorXt f(const VectorXt& state) const {
    VectorXt next_state(16);

    Vector3t pt = state.middleRows(0, 3);
    Vector3t vt = state.middleRows(3, 3);
    Quaterniont qt(state[6], state[7], state[8], state[9]);
    qt.normalize();

    Vector3t acc_bias = state.middleRows(10, 3);
    Vector3t gyro_bias = state.middleRows(13, 3);

    // position
    next_state.middleRows(0, 3) = pt + vt * dt;  //

    // velocity
    next_state.middleRows(3, 3) = vt;

    // orientation
    Quaterniont qt_ = qt;

    next_state.middleRows(6, 4) << qt_.w(), qt_.x(), qt_.y(), qt_.z();
    next_state.middleRows(10, 3) = state.middleRows(10, 3);  // constant bias on acceleration
    next_state.middleRows(13, 3) = state.middleRows(13, 3);  // constant bias on angular velocity

    return next_state;
  }

  /**
   * @brief system equation
   * 有输入的系统状态转移方程
   * @param state    上一时刻的系统状态量
   * @param control  控制量,也就是IMU的数据
   * @return VectorXt
   */
  VectorXt f(const VectorXt& state, const VectorXt& control) const {
    VectorXt next_state(16);
    // 当前时刻状态和控制量
    Vector3t pt = state.middleRows(0, 3);                    // 位置
    Vector3t vt = state.middleRows(3, 3);                    // 速度
    Quaterniont qt(state[6], state[7], state[8], state[9]);  // 四元数
    qt.normalize();                                          // 四元数归一化
    Vector3t acc_bias = state.middleRows(10, 3);             // 加速度偏置
    Vector3t gyro_bias = state.middleRows(13, 3);            // 陀螺仪偏置
    Vector3t raw_acc = control.middleRows(0, 3);             // 加速度(控制量)
    Vector3t raw_gyro = control.middleRows(3, 3);            // 角速度(控制量)

    // 根据上一时刻的状态量，以及当前的控制输入预测当前的状态量
    // 1.更新位置(匀速模型)
    next_state.middleRows(0, 3) = pt + vt * dt;  // 恒定运动速度模型更新位置

    // 2.更新速度
    Vector3t g(0.0f, 0.0f, 9.80665f);
    Vector3t acc_ = raw_acc - acc_bias;                 // 去除偏置
    Vector3t acc = qt * acc_;                           // 加速度转移到世界坐标系
    next_state.middleRows(3, 3) = vt + (acc - g) * dt;  // 速度更新(非匀速)
    // next_state.middleRows(3, 3) = vt;                   // 速度更新(匀速)

    // 3.更新姿态
    Vector3t gyro = raw_gyro - gyro_bias;                                     // 减去偏置
    Quaterniont dq(1, gyro[0] * dt / 2, gyro[1] * dt / 2, gyro[2] * dt / 2);  // 旋转转换到四元数，小量
    dq.normalize();                                                           // 归一化
    Quaterniont qt_ = (qt * dq).normalized();                                 // 四元数更新
    next_state.middleRows(6, 4) << qt_.w(), qt_.x(), qt_.y(), qt_.z();

    // 4.更新偏置(不变)
    next_state.middleRows(10, 3) = state.middleRows(10, 3);
    next_state.middleRows(13, 3) = state.middleRows(13, 3);

    return next_state;
  }

  /**
   * @brief observation equation
   * 观测方程
   * @param state     更新阶段(correct)生成的带误差方差的扩展状态空间下的ext_sigma_points
   * @return VectorXt
   */
  VectorXt h(const VectorXt& state) const {
    VectorXt observation(7);
    observation.middleRows(0, 3) = state.middleRows(0, 3);
    observation.middleRows(3, 4) = state.middleRows(6, 4).normalized();

    return observation;
  }
};

}  // namespace hdl_localization

#endif  // POSE_SYSTEM_HPP
