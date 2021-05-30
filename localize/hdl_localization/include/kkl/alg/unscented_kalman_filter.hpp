/**
 * UnscentedKalmanFilterX.hpp
 * @author koide
 * 16/02/01
 **/
/* UKF实现 */
#ifndef KKL_UNSCENTED_KALMAN_FILTER_X_HPP
#define KKL_UNSCENTED_KALMAN_FILTER_X_HPP

#include <random>
#include <Eigen/Dense>

namespace kkl {
namespace alg {

/**
 * @brief Unscented Kalman Filter class
 * @param T        scaler type
 * @param System   system class to be estimated
 */
template <typename T, class System>
class UnscentedKalmanFilterX {
  typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXt;
  typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> MatrixXt;

public:
  /**
   * @brief constructor
   * @param system               system to be estimated
   * @param state_dim            state vector dimension
   * @param input_dim            input vector dimension
   * @param measurement_dim      measurement vector dimension
   * @param process_noise        process noise covariance (state_dim x state_dim)
   * @param measurement_noise    measurement noise covariance (measurement_dim x measuremend_dim)
   * @param mean                 initial mean
   * @param cov                  initial covariance
   */
  UnscentedKalmanFilterX(
    const System& system,
    int state_dim,
    int input_dim,
    int measurement_dim,
    const MatrixXt& process_noise,
    const MatrixXt& measurement_noise,
    const VectorXt& mean,
    const MatrixXt& cov)
  : state_dim(state_dim),
    input_dim(input_dim),
    measurement_dim(measurement_dim),
    N(state_dim),
    M(input_dim),
    K(measurement_dim),
    S(2 * state_dim + 1),
    mean(mean),
    cov(cov),
    system(system),
    process_noise(process_noise),
    measurement_noise(measurement_noise),
    lambda(1),
    normal_dist(0.0, 1.0) {
    weights.resize(S, 1);                              // 33*1
    sigma_points.resize(S, N);                         // 33*16
    ext_weights.resize(2 * (N + K) + 1, 1);            // 47*1
    ext_sigma_points.resize(2 * (N + K) + 1, N + K);   // 47*23
    expected_measurements.resize(2 * (N + K) + 1, K);  // 47*7

    // initialize weights for unscented filter
    weights[0] = lambda / (N + lambda);
    for (int i = 1; i < 2 * N + 1; i++) {
      weights[i] = 1 / (2 * (N + lambda));
    }

    // weights for extended state space which includes error variances
    ext_weights[0] = lambda / (N + K + lambda);
    for (int i = 1; i < 2 * (N + K) + 1; i++) {
      ext_weights[i] = 1 / (2 * (N + K + lambda));
    }
  }

  /**
   * @brief predict 没有IMU的时候预测更新系统的状态量
   * 注释内容同predict(const VectorXt& control)
   */
  void predict() {
    // calculate sigma points
    ensurePositiveFinite(cov);
    computeSigmaPoints(mean, cov, sigma_points);
    for (int i = 0; i < S; i++) {
      // 系统的状态转移方程
      sigma_points.row(i) = system.f(sigma_points.row(i));
    }

    const auto& R = process_noise;

    // unscented transform
    VectorXt mean_pred(mean.size());
    MatrixXt cov_pred(cov.rows(), cov.cols());

    mean_pred.setZero();
    cov_pred.setZero();
    for (int i = 0; i < S; i++) {
      mean_pred += weights[i] * sigma_points.row(i);
    }
    for (int i = 0; i < S; i++) {
      VectorXt diff = sigma_points.row(i).transpose() - mean_pred;
      cov_pred += weights[i] * diff * diff.transpose();
    }
    cov_pred += R;

    mean = mean_pred;
    cov = cov_pred;
  }

  /**
   * @brief predict 有IMU的时候预测更新系统的状态
   * @param control  传入的一帧IMU数据
   * @todo 为什么predict没有把状态量和运动噪声合成一个增广状态量?
   */
  void predict(const VectorXt& control) {
    // calculate sigma points
    ensurePositiveFinite(cov);                    // 确保cov正定确保可以取逆
    computeSigmaPoints(mean, cov, sigma_points);  // 得到sigma_points
    // sigma点 和 控制量 代入状态传递函数f计算新时刻的分布
    for (int i = 0; i < S; i++) {
      sigma_points.row(i) = system.f(sigma_points.row(i), control);  // 采样分布和控制量输入传递函数
    }

    // 过程噪声
    const auto& R = process_noise;

    // unscented transform
    // 根据状态迁移后的sigma_points计算均值和协方差
    VectorXt mean_pred(mean.size());
    MatrixXt cov_pred(cov.rows(), cov.cols());
    mean_pred.setZero();
    cov_pred.setZero();
    // 均值
    for (int i = 0; i < S; i++) {
      mean_pred += weights[i] * sigma_points.row(i);
    }
    // 协方差
    for (int i = 0; i < S; i++) {
      VectorXt diff = sigma_points.row(i).transpose() - mean_pred;
      cov_pred += weights[i] * diff * diff.transpose();
    }
    // 协方差+过程噪声
    cov_pred += R;
    // 更新mean和cov
    mean = mean_pred;
    cov = cov_pred;
  }

  /**
   * @brief correct 根据ndt的测量更新系统状态
   * @param measurement  measurement vector
   */
  void correct(const VectorXt& measurement) {
    // create extended state space which includes error variances
    // 根据预测的状态建立增广矩阵 N-状态方程维度 K-观测维度
    VectorXt ext_mean_pred = VectorXt::Zero(N + K, 1);
    MatrixXt ext_cov_pred = MatrixXt::Zero(N + K, N + K);
    ext_mean_pred.topLeftCorner(N, 1) = VectorXt(mean);        // 扩展状态量前N项为mean
    ext_cov_pred.topLeftCorner(N, N) = MatrixXt(cov);          // 扩展协方差左上角为N*N的cov
    ext_cov_pred.bottomRightCorner(K, K) = measurement_noise;  // 扩展协方差右下角为K*K的观测噪声

    // 计算ext_sigma_points
    ensurePositiveFinite(ext_cov_pred);
    computeSigmaPoints(ext_mean_pred, ext_cov_pred, ext_sigma_points);

    // unscented transform
    // 对每个sigma点的状态做观测转换，并叠加观测噪声
    expected_measurements.setZero();
    for (int i = 0; i < ext_sigma_points.rows(); i++) {
      expected_measurements.row(i) = system.h(ext_sigma_points.row(i).transpose().topLeftCorner(N, 1));
      expected_measurements.row(i) += VectorXt(ext_sigma_points.row(i).transpose().bottomRightCorner(K, 1));
    }

    // 均值
    VectorXt expected_measurement_mean = VectorXt::Zero(K);
    for (int i = 0; i < ext_sigma_points.rows(); i++) {
      expected_measurement_mean += ext_weights[i] * expected_measurements.row(i);
    }
    // 协方差
    MatrixXt expected_measurement_cov = MatrixXt::Zero(K, K);
    for (int i = 0; i < ext_sigma_points.rows(); i++) {
      VectorXt diff = expected_measurements.row(i).transpose() - expected_measurement_mean;
      expected_measurement_cov += ext_weights[i] * diff * diff.transpose();
    }
    // calculated transformed covariance
    MatrixXt sigma = MatrixXt::Zero(N + K, K);
    for (int i = 0; i < ext_sigma_points.rows(); i++) {
      auto diffA = (ext_sigma_points.row(i).transpose() - ext_mean_pred);
      auto diffB = (expected_measurements.row(i).transpose() - expected_measurement_mean);
      sigma += ext_weights[i] * (diffA * diffB.transpose());
    }
    // 卡尔曼增益
    kalman_gain = sigma * expected_measurement_cov.inverse();
    const auto& K = kalman_gain;

    // 更新
    VectorXt ext_mean = ext_mean_pred + K * (measurement - expected_measurement_mean);
    MatrixXt ext_cov = ext_cov_pred - K * expected_measurement_cov * K.transpose();

    mean = ext_mean.topLeftCorner(N, 1);
    cov = ext_cov.topLeftCorner(N, N);
  }

  /*			getter			*/
  const VectorXt& getMean() const { return mean; }
  const MatrixXt& getCov() const { return cov; }
  const MatrixXt& getSigmaPoints() const { return sigma_points; }
  System& getSystem() { return system; }
  const System& getSystem() const { return system; }
  const MatrixXt& getProcessNoiseCov() const { return process_noise; }
  const MatrixXt& getMeasurementNoiseCov() const { return measurement_noise; }
  const MatrixXt& getKalmanGain() const { return kalman_gain; }

  /*			setter			*/
  UnscentedKalmanFilterX& setMean(const VectorXt& m) {
    mean = m;
    return *this;
  }
  UnscentedKalmanFilterX& setCov(const MatrixXt& s) {
    cov = s;
    return *this;
  }
  UnscentedKalmanFilterX& setProcessNoiseCov(const MatrixXt& p) {
    process_noise = p;
    return *this;
  }
  UnscentedKalmanFilterX& setMeasurementNoiseCov(const MatrixXt& m) {
    measurement_noise = m;
    return *this;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  const int state_dim;
  const int input_dim;
  const int measurement_dim;

  const int N;
  const int M;
  const int K;
  const int S;

public:
  VectorXt mean;
  MatrixXt cov;

  System system;
  MatrixXt process_noise;
  MatrixXt measurement_noise;

  T lambda;
  VectorXt weights;

  MatrixXt sigma_points;

  VectorXt ext_weights;
  MatrixXt ext_sigma_points;
  MatrixXt expected_measurements;

private:
  /**
   * @brief compute sigma points
   * 通过均值和协方差矩阵计算sigma点
   * 思路是将cov做Cholesky分解，用下三角矩阵L对mean做处理，得到一系列sigma_points
   * @param mean          mean 均值
   * @param cov           covariance 协方差矩阵
   * @param sigma_points  返回值calculated sigma points
   */
  void computeSigmaPoints(const VectorXt& mean, const MatrixXt& cov, MatrixXt& sigma_points) {
    const int n = mean.size();
    assert(cov.rows() == n && cov.cols() == n);

    // LLT分解求协方差的逆矩阵
    Eigen::LLT<MatrixXt> llt;
    llt.compute((n + lambda) * cov);
    MatrixXt l = llt.matrixL();

    sigma_points.row(0) = mean;
    for (int i = 0; i < n; i++) {
      sigma_points.row(1 + i * 2) = mean + l.col(i);      // 奇数
      sigma_points.row(1 + i * 2 + 1) = mean - l.col(i);  // 偶数
    }
  }

  /**
   * @brief make covariance matrix positive finite
   * @param cov  covariance matrix
   */
  void ensurePositiveFinite(MatrixXt& cov) {
    return;
    const double eps = 1e-9;

    Eigen::EigenSolver<MatrixXt> solver(cov);
    MatrixXt D = solver.pseudoEigenvalueMatrix();
    MatrixXt V = solver.pseudoEigenvectors();
    for (int i = 0; i < D.rows(); i++) {
      if (D(i, i) < eps) {
        D(i, i) = eps;
      }
    }

    cov = V * D * V.inverse();
  }

public:
  MatrixXt kalman_gain;

  std::mt19937 mt;
  std::normal_distribution<T> normal_dist;
};

}  // namespace alg
}  // namespace kkl

#endif
