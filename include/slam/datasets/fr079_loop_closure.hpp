#pragma once

// FR079 has scan-level LiDAR and logged odometry only: no calibrated
// per-beam timing, extrinsics or independent pose ground truth. These offline
// graph results are hypotheses, not a certified accuracy improvement.
#include "slam/datasets/fr079_offline.hpp"
#include "slam/graph/pose_graph.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace slam::datasets {
// Explicitly distinguish the legacy per-scan matcher from a full SE(2)
// pose-prior matcher. A future submap matcher can coexist as another mode.
enum class Fr079LoopRegistrationMode {
  legacy_scan_to_scan,
  pose_prior_scan_to_scan,
  correlative_scan_to_submap
};
struct Fr079LoopEvent {
  std::size_t from{}, to{};
  std::string decision;
  std::size_t forward_inliers{}, reverse_inliers{};
  double overlap{}, rms_m{};
  double innovation_m{}, innovation_yaw_rad{};
  double predicted_relative_yaw_rad{};  // absolute wrapped Ceres prior
  // Only populated for correlative scan-to-submap registrations.
  std::size_t submap_contributing_scans{};
  double submap_score{}, submap_support_fraction{};
  double submap_translation_curvature_min{}, submap_translation_curvature_ratio{};
  double submap_minimum_shift_score_drop{};
};
// Fixed FR079 diagnostic pairs from the previous heading-aware run.
// These probes NEVER become graph factors; they are not labeled truth.
struct Fr079SubmapProbe {
  std::size_t from{}, to{};
  bool accepted{}, observable{};
  double predicted_relative_yaw_rad{}, score{}, support_fraction{};
  double translation_curvature_min{}, translation_curvature_ratio{};
  double minimum_shift_score_drop{};
};
struct Fr079LoopResult {
  mapping::MapBundle odometry_graph_map;
  mapping::MapBundle loop_graph_map;
  std::vector<Pose2> odometry_graph_poses, loop_graph_poses;
  // The first vector is the sequential-only graph control in both modes;
  // the historical field name is kept for backward-compatible consumers.
  std::string sequential_factor_source{"logged_odometry_scan_time_v1"};
  std::vector<Fr079LoopEvent> decisions;
  std::vector<Fr079SubmapProbe> submap_probes;
  std::size_t nearby_pairs{}, registration_attempts{}, geometry_verified{};
  std::size_t redundant_suppressed{}, proposed_loops{}, accepted_loops{};
  std::size_t large_heading_nearby_pairs{}, large_heading_registration_attempts{};
  std::size_t large_heading_geometry_verified{}, large_heading_accepted_loops{};
  std::string registration_mode{"legacy_scan_to_scan_v1"};
  std::size_t submaps_built{}, submap_registration_attempts{};
  std::size_t submap_geometry_verified{}, submap_ambiguous_rejected{};
  double control_initial_cost{}, control_final_cost{};
  double loop_initial_cost{}, loop_final_cost{};
  double mean_graph_shift_m{}, maximum_graph_shift_m{};
  double maximum_graph_shift_yaw_rad{};
  bool first_pose_fixed{}, independent_ground_truth{false};
  // Post-optimization warning only; rejected candidates remain exported
  // for experiment review and are never automatically promoted.
  bool graph_shift_within_safety_limit{};
};

[[nodiscard]] Fr079LoopResult run_fr079_loop_experiment(
    const std::vector<CarmenLaser>& scans,
    const Fr079SlamResult& frontend,
    const Fr079SlamOptions& options,
    bool frontend_sequential_graph = false,
    Fr079LoopRegistrationMode registration_mode =
        Fr079LoopRegistrationMode::legacy_scan_to_scan);
} // namespace slam::datasets
