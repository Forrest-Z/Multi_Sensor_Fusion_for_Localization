#ifndef POSE_SYSTEM_HPP
#define POSE_SYSTEM_HPP

#include <kkl/alg/unscented_kalman_filter.hpp>

namespace hdl_localization {

/**
 * @brief Definition of system to be estimated by ukf
 * @note state = [px, py, pz, vx, vy, vz, qw, qx, qy, qz, acc_bias_x, acc_bias_y, acc_bias_z, gyro_bias_x, gyro_bias_y, gyro_bias_z]
 * 系统状态量16个
 * 位置1-3 速度4-6 四元数7-10 加速度偏置11-13 角速度偏置14-16
 * 观测状态量7个
 * 位置1-3 四元数4-7
 * 控制量6个
 * 加速度1-3 角速度4-6
 */
class PoseSystem {
public:
  typedef float T;
  typedef Eigen::Matrix<T, 3, 1> Vector3t;
  typedef Eigen::Matrix<T, 4, 4> Matrix4t;
  typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXt;
  typedef Eigen::Quaternion<T> Quaterniont;

public:
  PoseSystem() { dt = 0.01; }

  // system equation (without input)
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

  // system equation
  /**
   * @brief 状态方程f
   *
   * @param state
   * @param control
   * @return VectorXt
   */
  VectorXt f(const VectorXt& state, const VectorXt& control) const {
    VectorXt next_state(16);
    // 读取状态
    Vector3t pt = state.middleRows(0, 3);                    // 位置
    Vector3t vt = state.middleRows(3, 3);                    // 速度
    Quaterniont qt(state[6], state[7], state[8], state[9]);  // 四元数
    qt.normalize();                                          // 四元数归一化
    Vector3t acc_bias = state.middleRows(10, 3);             // 加速度偏置
    Vector3t gyro_bias = state.middleRows(13, 3);            // 陀螺仪偏置
    // 控制量
    Vector3t raw_acc = control.middleRows(0, 3);   // 加速度计
    Vector3t raw_gyro = control.middleRows(3, 3);  // 角速度

    /* 根据上一时刻的状态量，以及当前的控制输入预测当前的状态量 */
    // position
    next_state.middleRows(0, 3) = pt + vt * dt;  // 恒定运动速度模型更新位置
    // velocity
    Vector3t g(0.0f, 0.0f, 9.80665f);
    Vector3t acc_ = raw_acc - acc_bias;                 // 去除偏置
    Vector3t acc = qt * acc_;                           // 加速度转移到世界坐标系
    next_state.middleRows(3, 3) = vt + (acc - g) * dt;  // 速度更新(不是恒定速度)
    // next_state.middleRows(3, 3) = vt; // + (acc - g) * dt;		// acceleration didn't contribute to accuracy due to large noise
    // orientation
    Vector3t gyro = raw_gyro - gyro_bias;                                     // 减去偏置
    Quaterniont dq(1, gyro[0] * dt / 2, gyro[1] * dt / 2, gyro[2] * dt / 2);  // 旋转转换到四元数，小量
    dq.normalize();
    Quaterniont qt_ = (qt * dq).normalized();  // 四元数更新
    next_state.middleRows(6, 4) << qt_.w(), qt_.x(), qt_.y(), qt_.z();

    next_state.middleRows(10, 3) = state.middleRows(10, 3);  // constant bias on acceleration
    next_state.middleRows(13, 3) = state.middleRows(13, 3);  // constant bias on angular velocity

    return next_state;
  }

  // observation equation
  /**
   * @brief 观测方程h
   * 输入是在更新阶段(correct)生成的带误差方差的(error variances)的扩展状态空间下的(extended state space)状态量
   * 也就是ext_sigma_points
   * @param state
   * @return VectorXt
   */
  VectorXt h(const VectorXt& state) const {
    VectorXt observation(7);
    observation.middleRows(0, 3) = state.middleRows(0, 3);
    observation.middleRows(3, 4) = state.middleRows(6, 4).normalized();

    return observation;
  }

  double dt;
};

}  // namespace hdl_localization

#endif  // POSE_SYSTEM_HPP
