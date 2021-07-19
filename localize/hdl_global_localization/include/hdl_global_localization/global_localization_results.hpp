#ifndef HDL_GLOBAL_LOCALIZATION_RESULTS_HPP
#define HDL_GLOBAL_LOCALIZATION_RESULTS_HPP

#include <vector>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace hdl_global_localization {

struct GlobalLocalizationResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<GlobalLocalizationResult>;

  GlobalLocalizationResult(double error, double inlier_fraction, const Eigen::Isometry3f& pose) : error(error), inlier_fraction(inlier_fraction), pose(pose) {}

  double error;
  double inlier_fraction;
  Eigen::Isometry3f pose;
};

struct GlobalLocalizationResults {
  GlobalLocalizationResults(const std::vector<GlobalLocalizationResult::Ptr>& results) : results(results) {}

  GlobalLocalizationResults& sort(int max_num_candidates) {
    // remove_if和erase成对使用
    // 删除空结果
    auto remove_loc = std::remove_if(results.begin(), results.end(), [](const auto& result) { return result == nullptr; });
    results.erase(remove_loc, results.end());

    std::cout << "valid solutions:" << results.size() << std::endl;

    // lambda:按照内点比值排序从大到小排序
    // std::sort(results.begin(), results.end(), [](const auto& lhs, const auto& rhs) { return lhs->error < rhs->error; });
    std::sort(results.begin(), results.end(), [](const auto& lhs, const auto& rhs) { return lhs->inlier_fraction > rhs->inlier_fraction; });
    // 删除max_num_candidates之后的数据
    if (results.size() > max_num_candidates) {
      results.erase(results.begin() + max_num_candidates, results.end());
    }

    return *this;
  }

  std::vector<GlobalLocalizationResult::Ptr> results;
};
}  // namespace hdl_global_localization

#endif