#ifndef HDL_LOCALIZATION_DELTA_ESTIMATER_HPP
#define HDL_LOCALIZATION_DELTA_ESTIMATER_HPP

#include <mutex>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>

namespace hdl_localization {

class DeltaEstimater {
public:
  using PointT = pcl::PointXYZI;

  /**
   * @brief Construct a new Delta Estimater object
   * @param reg
   */
  DeltaEstimater(pcl::Registration<PointT, PointT>::Ptr reg) : delta(Eigen::Isometry3f::Identity()), reg(reg) {}
  ~DeltaEstimater() {}

  void reset() {
    std::lock_guard<std::mutex> lock(mutex);
    delta.setIdentity();
    last_frame.reset();
  }

  /**
   * @brief 点云的帧间匹配，并维护一个里程计
   * @param frame
   */
  void add_frame(pcl::PointCloud<PointT>::ConstPtr frame) {
    std::unique_lock<std::mutex> lock(mutex);
    if (last_frame == nullptr) {
      last_frame = frame;
      return;
    }

    reg->setInputTarget(last_frame);
    reg->setInputSource(frame);
    lock.unlock();

    pcl::PointCloud<PointT> aligned;
    reg->align(aligned);  // 匹配后的点云

    lock.lock();
    last_frame = frame;
    delta = delta * Eigen::Isometry3f(reg->getFinalTransformation());
  }

  /**
   * @brief 返回delta
   * @return Eigen::Isometry3f
   */
  Eigen::Isometry3f estimated_delta() const {
    std::lock_guard<std::mutex> lock(mutex);
    return delta;
  }

private:
  mutable std::mutex mutex;
  Eigen::Isometry3f delta;
  pcl::Registration<PointT, PointT>::Ptr reg;

  pcl::PointCloud<PointT>::ConstPtr last_frame;
};

}  // namespace hdl_localization

#endif