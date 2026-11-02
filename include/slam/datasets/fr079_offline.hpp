#pragma once
#include "slam/datasets/carmen.hpp"
#include "slam/mapping/map_bundle.hpp"

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace slam::datasets {

// FR079 contains scan-level timestamps and embedded odometry estimates, NOT
// beam timestamps, calibrated laser extrinsics, IMU data or independent truth.
// This is sequential LiDAR + odometry mapping, not synthetic gyro fusion.
struct Fr079SlamOptions {
  double map_resolution_m{0.10};
  double max_hit_range_m{18.0};
  double map_margin_m{21.0};
  std::size_t map_beam_stride{3};
  std::size_t match_beam_stride{4};
  std::size_t match_every{5};
  std::size_t rebuild_field_every{15};
  double distance_truncation_m{0.70};
  double max_correction_m{0.40};
  double max_correction_yaw_rad{0.17};
  bool use_ceres{true};
  // Optional diagnostic observer: receives only the last accepted estimated
  // pose, query scan, and a *previously built* immutable map snapshot.
  // Never influences SLAM state, and never supplies ground-truth poses.
  std::function<void(std::size_t,const Pose2&,
                     std::shared_ptr<const mapping::MapSnapshot>,
                     const std::vector<Eigen::Vector2d>&)> capture_probe;
};
struct Fr079SlamFrame {
  Pose2 odom;           // odometry in first-scan gauge; never ground truth
  Pose2 predicted;      // propagated prior before optional scan matching
  Pose2 estimated;      // accepted correction or predicted
  double scan_time_s{};
  std::size_t beam_count{}, valid_hits{}, mapped_hits{};
  bool matched{}, accepted{}, map_inserted{};
  std::string match_status{"not_attempted"};
  std::size_t supported_hits{};
  std::size_t ceres_invalid_residual_evaluations{};
  std::size_t ceres_invalid_unknown_cells{},ceres_invalid_truncated{},ceres_invalid_outside_grid{},ceres_invalid_other{};
  std::size_t ceres_unsuccessful_steps{}, ceres_successful_steps{};
  std::string ceres_termination{"not_run"};
  double overlap{}, robust_cost{};
  double field_ms{}, match_ms{}, insertion_ms{}, frame_ms{};
};
struct Fr079SlamResult {
  mapping::MapBundle map;
  std::vector<Fr079SlamFrame> frames;
  std::size_t attempts{}, accepted{}, rejected{}, inserted_scans{};
  double total_field_ms{}, total_match_ms{}, total_insertion_ms{};
  std::size_t total_invalid_residual_evaluations{};
  std::size_t total_invalid_unknown_cells{},total_invalid_truncated{},total_invalid_outside_grid{},total_invalid_other{};
  std::size_t total_unsuccessful_steps{};
  bool used_ceres{};
  // No graph constraints from correlated scan-to-map results or logged poses.
  bool graph_optimized{false};
  bool independent_ground_truth{false};
};

[[nodiscard]] Fr079SlamResult run_fr079_offline_slam(
    const std::vector<CarmenLaser>& scans,
    const Fr079SlamOptions& options = {});

}  // namespace slam::datasets
