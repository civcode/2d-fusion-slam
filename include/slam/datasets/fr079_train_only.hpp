#pragma once
#include "slam/datasets/carmen.hpp"
#include "slam/mapping/distance_field.hpp"
#include "slam/matching/geometry.hpp"
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace slam::datasets {
struct TrainOnlyOptions {
  std::size_t holdout_modulus{5};     // indices 0,5,10,... cannot contribute to map
  std::size_t training_stride{6};    // candidate index % stride == 1
  std::size_t heldout_query_stride{12}; // one in N held-out scans is evaluated
  std::size_t beam_stride{4};
  double min_hit_range_m{0.10}, max_hit_range_m{18.0};
  double grid_resolution_m{0.10}, grid_margin_m{10.0}, edt_truncation_m{0.70};
  double seed_dx_m{0.07}, seed_dy_m{-0.05}, seed_dyaw_rad{0.02};
  bool use_ceres{true};
};
struct TrainingMap {
  std::shared_ptr<const mapping::MapSnapshot> snapshot;
  std::shared_ptr<const mapping::DistanceField> field;
  std::size_t input_scans{}, training_eligible{}, training_inserted{}, training_rays{};
};
struct HeldoutReport {
  std::size_t input_scans{}, training_eligible{}, training_inserted{}, heldout_scans{};
  std::size_t evaluated_queries{}, accepted{}, no_improvement{}, rejected_support{}, rejected_degenerate{}, invalid{};
  std::size_t attempted_hits{};
  std::uint64_t training_map_fingerprint{};
  double mean_initial_overlap{}, mean_final_overlap{};
  double mean_initial_robust_cost{}, mean_final_robust_cost{};
  double mean_displacement_from_logged_seed_m{}; // NOT absolute error
  bool independent_ground_truth{false};
  bool per_beam_time_verified{false};
  bool reference_map_from_heldout{false};
  bool used_ceres{false};
};
[[nodiscard]] TrainingMap build_fr079_training_map(
    const std::vector<CarmenLaser>& scans,const TrainOnlyOptions& options = {});
[[nodiscard]] HeldoutReport evaluate_fr079_holdout(
    const std::vector<CarmenLaser>& scans,const TrainingMap& training,
    const TrainOnlyOptions& options = {});
} // namespace slam::datasets
