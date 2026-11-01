#pragma once

#include "slam/graph/pose_graph.hpp"
#include "slam/mapping/generation_store.hpp"
#include "slam/mapping/map_bundle.hpp"
#include "slam/matching/geometry.hpp"

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace slam::offline {

// A scan is expressed in one keyframe's BASE frame. The producer must have
// independently verified acquisition time, calibration and any needed deskew.
// No interpolation/extrapolation is performed here.
struct LocalRay {
  Eigen::Vector2d origin_base{Eigen::Vector2d::Zero()};
  Eigen::Vector2d endpoint_base{Eigen::Vector2d::Zero()};
  sensors::ReturnKind kind{sensors::ReturnKind::invalid};
};
struct VerifiedKeyframe {
  std::size_t node_index{};
  std::vector<LocalRay> rays;
  graph::FactorProvenance source;
  bool acquisition_and_deskew_verified{false};
};

// Completely separate reference observations, already expressed in the
// graph/map frame. Independent synthetic truth works; logged FR079 poses do NOT.
struct HeldoutReference {
  std::vector<Eigen::Vector2d> hit_endpoints_map;
  graph::FactorProvenance source;
};

struct PromotionOptions {
  mapping::GridConfig grid;
  graph::BatchGraphOptions graph;
  std::size_t minimum_heldout_hits{25};
  unsigned occupied_neighborhood_cells{1};
  double occupied_log_odds_threshold{0.0};
  double minimum_hit_fraction_gain{0.05};
  double maximum_known_fraction_regression{0.02};
};

struct HeldoutScore {
  std::size_t attempted{}, known{}, near_occupied{};
  [[nodiscard]] double known_fraction() const noexcept;
  [[nodiscard]] double hit_fraction() const noexcept;
};

enum class PromotionDecision {
  promoted,
  no_accepted_loop,
  insufficient_reference,
  insufficient_improvement,
  known_coverage_regression
};

struct MapGenerationResult {
  graph::BatchGraphResult graph_result;
  mapping::MapBundle incumbent;
  mapping::MapBundle candidate;
  std::shared_ptr<const mapping::MapSnapshot> authoritative;
  HeldoutScore incumbent_score, candidate_score;
  PromotionDecision decision{PromotionDecision::insufficient_reference};
  std::size_t mapped_keyframes{}, inserted_rays{};
  [[nodiscard]] bool promoted() const noexcept {
    return decision == PromotionDecision::promoted;
  }
};

// Deliberately only a *gate* for a separately sourced relative measurement:
// Ceres map-from-laser tracking output is never converted into an independent
// graph constraint or a covariance. This cannot infer a measurement from pose.
[[nodiscard]] graph::LoopCandidate qualified_frontend_loop(
    graph::PoseEdge separately_measured_relative,
    const matching::MatchResult& scan_match,
    std::uint64_t expected_map_fingerprint);

// Graph optimize -> build BOTH immutable generations with identical training
// rays -> compare against reserved independent reference -> promote or retain
// original. No mutation to incumbent or externally published map on rejection.
[[nodiscard]] MapGenerationResult evaluate_map_generation(
    const graph::BatchGraphInput& graph_input,
    const std::vector<VerifiedKeyframe>& mapping_frames,
    const HeldoutReference& reserved_reference,
    const PromotionOptions& options);

// Durable promotion is a separate explicit action. A selected candidate can
// be committed only if the store's active generation matches the exact
// incumbent snapshot used by the evaluation; otherwise it fails closed.
// A rejected candidate never writes or switches the authoritative map.
[[nodiscard]] std::optional<mapping::GenerationRef> publish_approved_generation(
    mapping::GenerationStore& store, const MapGenerationResult& evaluated);

} // namespace slam::offline
