#pragma once

#include "slam/core/se2.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace slam::graph {

// No odometry or loop edge is accepted without evidence of its source.
// 'synthetic_exact' is for deterministic test fixtures, never real hardware.
enum class EvidenceClass { unverified, synthetic_exact, independently_measured };
struct FactorProvenance {
  EvidenceClass source{EvidenceClass::unverified};
  std::string source_id;        // identifiable log/sensor/fixture
  bool clock_and_frame_verified{false};
  bool independent_of_initial_trajectory{false};
};
struct PoseEdge {
  std::size_t from{}, to{};  // odometry must join consecutive nodes
  Pose2 from_to{};          // measured T_from^-1 * T_to (body-relative)
  double sigma_x_m{0.1}, sigma_y_m{0.1}, sigma_yaw_rad{0.05};
  FactorProvenance provenance;
};
struct LoopCandidate {
  PoseEdge edge;
  std::size_t supported_hits{};
  double overlap{};
  bool full_rank{};
  // These are quality tests, NOT an independent covariance measurement.
};
struct BatchGraphInput {
  std::vector<Pose2> initial_map_from_base;
  std::vector<PoseEdge> odometry;
  std::vector<LoopCandidate> loops;
};
struct BatchGraphOptions {
  std::size_t max_nodes{10000};
  std::size_t max_loop_candidates{20000};
  std::size_t minimum_loop_separation{3};
  std::size_t minimum_supported_hits{25};
  double minimum_overlap{0.65};
  double maximum_loop_residual_translation_m{3.0};
  double maximum_loop_residual_yaw_rad{0.8};
  double loop_huber_k{1.345};
  unsigned maximum_iterations{100};
};
enum class LoopDecision {
  accepted,
  insufficient_geometry,
  unverified_or_correlated_evidence,
  inconsistent_with_initial_path,
  duplicate_or_nearby_nodes
};
struct BatchGraphResult {
  std::vector<Pose2> optimized_map_from_base;
  std::vector<LoopDecision> loop_decisions;
  std::size_t accepted_loops{};
  double initial_cost{}, final_cost{}; // graph objective, NOT independent ATE/RPE
  bool fixed_first_pose{};
  bool independent_ground_truth{}; // always false; this API has no truth input
};

// Offline-only GTSAM batch reference. No GTSAM dependency in public header
// or on known-map Pi localization. Rejects unsupported graph provenance.
[[nodiscard]] BatchGraphResult optimize_pose_graph(
    const BatchGraphInput& input, const BatchGraphOptions& options = {});

}  // namespace slam::graph
