#pragma once
#include "slam/datasets/fr079_offline.hpp"
#include <cstddef>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace slam::datasets {
// Controlled recovery relative to this run's accepted SLAM estimate.
// That estimate is NOT independent truth or a surveyed reference pose.
class Fr079CaptureBenchmark {
 public:
  Fr079CaptureBenchmark(const std::string& csv_path, const std::string& source_sha256,
                        bool use_ceres, std::size_t every, std::size_t max_frames);
  void probe(std::size_t frame, const Pose2& estimated,
             std::shared_ptr<const mapping::MapSnapshot> prior_map,
             const std::vector<Eigen::Vector2d>& laser_points);
  [[nodiscard]] std::size_t sampled_frames() const noexcept {return sampled_frames_;}
  [[nodiscard]] std::size_t tested_trials() const noexcept {return tested_trials_;}
 private:
  std::ofstream output_;
  std::string source_sha256_;
  bool use_ceres_;
  std::size_t every_,max_frames_,sampled_frames_{},tested_trials_{};
};
}  // namespace slam::datasets
