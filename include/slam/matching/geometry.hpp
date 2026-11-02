#pragma once

#include "slam/mapping/distance_field.hpp"

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace slam::matching {

struct MatchSettings {
  double sigma_metres{0.05};
  double huber_delta{1.0}; // units of normalized (distance/sigma) residual
  std::size_t min_supported_hits{12};
  double minimum_overlap{0.65};
  double minimum_relative_eigenvalue{0.0001};
  unsigned maximum_iterations{30};
  double initial_damping{1e-3};
  double maximum_step_translation_m{0.1};
  double maximum_step_yaw_rad{0.1};
};

struct MatchDiagnostics {
  std::size_t input_hits{};
  std::size_t supported_hits{};
  std::size_t unsupported_hits{};
  double overlap{};
  double robust_cost{};
  double mean_distance_metres{};
  Eigen::Matrix3d lidar_information{Eigen::Matrix3d::Zero()};
  Eigen::Vector3d eigenvalues{Eigen::Vector3d::Zero()};
  bool full_rank{};
  std::uint64_t source_revision{};
  std::uint64_t source_fingerprint{};
};

enum class MatchStatus { accepted, insufficient_support, degenerate, no_improvement, invalid_input };
struct MatchResult {
  MatchStatus status{MatchStatus::invalid_input};
  Pose2 map_from_laser{};
  MatchDiagnostics diagnostics{};
  unsigned iterations{};
  std::size_t ceres_invalid_residual_evaluations{}; // failed EDT samples across solver trials
  std::size_t ceres_invalid_unknown_cells{}, ceres_invalid_truncated{}, ceres_invalid_outside_grid{}, ceres_invalid_other{};
  std::size_t ceres_unsuccessful_steps{}, ceres_successful_steps{};
  std::string ceres_termination{"not_run"};
  std::size_t support_aware_selected_hits{}, support_aware_skipped_hits{};
  std::size_t support_aware_stages{}, support_aware_accepted_stages{};
  bool support_aware_no_safe_stage{}; // insufficient certified geometry, not solver success
  // No posterior covariance is asserted. This is a conditioned tracking pose,
  // NOT an independent LiDAR graph factor, even with a geometry-only objective.
};

// Evaluates geometry-only, single scalar robust hit residuals. Unknown,
// unsupported and truncated EDT samples contribute NO false LiDAR evidence.
[[nodiscard]] MatchDiagnostics evaluate_geometry(
    const mapping::DistanceField& field,
    const std::vector<Eigen::Vector2d>& laser_points,
    Pose2 map_from_laser, const MatchSettings& settings);

// Lightweight local reference optimizer for offline synthetic tests, not a
// replacement for the optional Ceres implementation. No odometry prior.
[[nodiscard]] MatchResult match_reference(
    std::shared_ptr<const mapping::DistanceField> field,
    const std::vector<Eigen::Vector2d>& laser_points,
    Pose2 initial_map_from_laser, const MatchSettings& settings = {});

#ifdef SLAM_HAS_CERES
// Implemented in src/match_ceres.cpp and enabled only when a Ceres toolchain
// is available. Uses the same independent geometry diagnostics and gating.
[[nodiscard]] MatchResult match_ceres(
    std::shared_ptr<const mapping::DistanceField> field,
    const std::vector<Eigen::Vector2d>& laser_points,
    Pose2 initial_map_from_laser, const MatchSettings& settings = {});
// Opt-in bounded multi-stage Ceres matcher. Every residual is selected only
// when its complete pose-step region has real, observed EDT support. No
// fabricated residual is supplied for unknown cells. The legacy path above
// remains the default for A/B comparisons.
[[nodiscard]] MatchResult match_ceres_support_aware(
    std::shared_ptr<const mapping::DistanceField> field,
    const std::vector<Eigen::Vector2d>& laser_points,
    Pose2 initial_map_from_laser, const MatchSettings& settings = {});
#endif

}  // namespace slam::matching
