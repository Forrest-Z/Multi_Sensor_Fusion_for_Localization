#ifndef HDL_GLOBAL_LOCALIZATION_ENGINE_HPP
#define HDL_GLOBAL_LOCALIZATION_ENGINE_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <hdl_global_localization/global_localization_results.hpp>

namespace hdl_global_localization {

/**
 * @brief 基类
 */
class GlobalLocalizationEngine {
public:
  GlobalLocalizationEngine() {}
  virtual ~GlobalLocalizationEngine() {}

  // 两个纯虚函数接口set_global_map query
  virtual void set_global_map(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) = 0;
  virtual GlobalLocalizationResults query(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int max_num_candidates) = 0;
};

}  // namespace hdl_global_localization

#endif