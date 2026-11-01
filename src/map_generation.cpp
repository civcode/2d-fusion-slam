#include "slam/offline/map_generation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace slam::offline {
namespace {
bool verified(const graph::FactorProvenance& p) {
  return p.source != graph::EvidenceClass::unverified && !p.source_id.empty() &&
         p.source_id.size() <= 256 && p.clock_and_frame_verified &&
         p.independent_of_initial_trajectory;
}
bool finite_pose(const Pose2& p) {
  return p.translation.allFinite() && std::isfinite(p.yaw_rad);
}
void validate_options(const PromotionOptions& o) {
  if (o.minimum_heldout_hits == 0 || o.occupied_neighborhood_cells > 8 ||
      !std::isfinite(o.occupied_log_odds_threshold) ||
      !std::isfinite(o.minimum_hit_fraction_gain) ||
      o.minimum_hit_fraction_gain <= 0 || o.minimum_hit_fraction_gain > 1 ||
      !std::isfinite(o.maximum_known_fraction_regression) ||
      o.maximum_known_fraction_regression < 0 || o.maximum_known_fraction_regression > 1)
    throw std::invalid_argument("invalid map-generation promotion settings");
}

mapping::MapBundle replay(const std::vector<Pose2>& poses,
                          const std::vector<VerifiedKeyframe>& frames,
                          const PromotionOptions& o) {
  mapping::OccupancyGrid grid(o.grid);
  for (const auto& frame : frames) {
    std::vector<mapping::WorldRay> rays;
    rays.reserve(frame.rays.size());
    const auto& pose = poses[frame.node_index];
    for (const auto& ray : frame.rays) {
      // Invalid/missing observations do not create free space.
      if (ray.kind == sensors::ReturnKind::invalid ||
          ray.kind == sensors::ReturnKind::missing) continue;
      rays.push_back({transform(pose, ray.origin_base),
                      transform(pose, ray.endpoint_base), ray.kind});
    }
    (void)grid.integrate_rays(rays);
  }
  const bool exact = std::all_of(frames.begin(), frames.end(), [](const auto& f) {
    return f.source.source == graph::EvidenceClass::synthetic_exact;
  });
  return {mapping::MapSnapshot::capture(grid),
          {"map", "verified_keyframe_replay", exact ? "synthetic_exact" : "measured_unverified_truth", exact}};
}

HeldoutScore score(const mapping::MapSnapshot& map,
                   const HeldoutReference& ref, const PromotionOptions& o) {
  HeldoutScore s;
  s.attempted=ref.hit_endpoints_map.size();
  const auto& cfg=map.config();
  const auto local_from_map=inverse(cfg.map_from_grid);
  const auto r=static_cast<std::int64_t>(o.occupied_neighborhood_cells);
  for (const auto& endpoint: ref.hit_endpoints_map) {
    const auto local=transform(local_from_map,endpoint);
    const auto gx=std::floor(local.x()/cfg.resolution_m);
    const auto gy=std::floor(local.y()/cfg.resolution_m);
    if (!std::isfinite(gx) || !std::isfinite(gy) || gx<0 || gy<0 ||
        gx>=static_cast<double>(cfg.width) || gy>=static_cast<double>(cfg.height)) continue;
    const auto x=static_cast<std::int64_t>(gx), y=static_cast<std::int64_t>(gy);
    const auto center=map.cell(static_cast<std::size_t>(x),static_cast<std::size_t>(y));
    if (center && center->observed) ++s.known;
    bool occupied=false;
    for (std::int64_t dx=-r; dx<=r && !occupied; ++dx)
      for (std::int64_t dy=-r; dy<=r && !occupied; ++dy) {
        if (dx*dx + dy*dy > r*r) continue;
        if (x+dx<0 || y+dy<0 ||
            x+dx>=static_cast<std::int64_t>(cfg.width) ||
            y+dy>=static_cast<std::int64_t>(cfg.height)) continue;
        const auto cell=map.cell(static_cast<std::size_t>(x+dx),
                                 static_cast<std::size_t>(y+dy));
        if (cell && cell->observed && cell->log_odds > o.occupied_log_odds_threshold)
          occupied=true;
      }
    if (occupied) ++s.near_occupied;
  }
  return s;
}
} // namespace

double HeldoutScore::known_fraction() const noexcept {
  return attempted ? static_cast<double>(known)/attempted : 0.0;
}
double HeldoutScore::hit_fraction() const noexcept {
  return attempted ? static_cast<double>(near_occupied)/attempted : 0.0;
}

graph::LoopCandidate qualified_frontend_loop(
    graph::PoseEdge measured, const matching::MatchResult& match,
    std::uint64_t expected_map_fingerprint) {
  const auto& d=match.diagnostics;
  if (match.status != matching::MatchStatus::accepted ||
      d.source_fingerprint == 0 || d.source_fingerprint != expected_map_fingerprint ||
      d.supported_hits < 3 || d.supported_hits > d.input_hits ||
      !std::isfinite(d.overlap) || d.overlap <= 0 || d.overlap > 1 ||
      !d.full_rank || !verified(measured.provenance) ||
      !finite_pose(measured.from_to) ||
      !std::isfinite(measured.sigma_x_m) || measured.sigma_x_m < 1e-6 ||
      !std::isfinite(measured.sigma_y_m) || measured.sigma_y_m < 1e-6 ||
      !std::isfinite(measured.sigma_yaw_rad) || measured.sigma_yaw_rad < 1e-6)
    throw std::invalid_argument("frontend match is not an independently justified loop factor");
  // No pose/covariance is copied from tracking. The only relative pose here
  // comes from the separately verified external measurement passed by caller.
  return {std::move(measured), d.supported_hits, d.overlap, d.full_rank};
}

MapGenerationResult evaluate_map_generation(
    const graph::BatchGraphInput& input,
    const std::vector<VerifiedKeyframe>& frames,
    const HeldoutReference& reference,
    const PromotionOptions& options) {
  validate_options(options);
  const auto n=input.initial_map_from_base.size();
  if (n < 2 || frames.empty() || !verified(reference.source) ||
      reference.hit_endpoints_map.size() > 2'000'000)
    throw std::invalid_argument("missing independent held-out evaluation data");
  for (const auto& p : reference.hit_endpoints_map)
    if (!p.allFinite()) throw std::invalid_argument("nonfinite held-out reference hit");
  std::set<std::size_t> nodes;
  std::set<std::string> scan_sources;
  std::size_t inserted=0;
  for (const auto& frame : frames) {
    if (frame.node_index >= n || !nodes.insert(frame.node_index).second ||
        !verified(frame.source) || !frame.acquisition_and_deskew_verified ||
        frame.source.source_id == reference.source.source_id)
      throw std::invalid_argument("invalid or correlated mapping frame provenance");
    scan_sources.insert(frame.source.source_id);
    if (frame.rays.size() > 8192) throw std::invalid_argument("oversized keyframe scan");
    for (const auto& ray:frame.rays) {
      if (!ray.origin_base.allFinite() || !ray.endpoint_base.allFinite())
        throw std::invalid_argument("nonfinite map insertion ray");
      if (ray.kind == sensors::ReturnKind::hit) ++inserted;
    }
  }
  if (inserted == 0) throw std::invalid_argument("no verified mapping hits");
  std::set<std::string> odometry_sources;
  for (const auto& edge:input.odometry) {
    if (edge.provenance.source_id == reference.source.source_id ||
        scan_sources.contains(edge.provenance.source_id))
      throw std::invalid_argument("odometry shares map-building or held-out evidence");
    odometry_sources.insert(edge.provenance.source_id);
  }
  for (const auto& loop:input.loops)
    if (scan_sources.contains(loop.edge.provenance.source_id) ||
        odometry_sources.contains(loop.edge.provenance.source_id) ||
        loop.edge.provenance.source_id == reference.source.source_id)
      throw std::invalid_argument("loop factor shares map-building, odometry or evaluation evidence");

  MapGenerationResult result;
  result.graph_result=graph::optimize_pose_graph(input,options.graph);
  result.incumbent=replay(input.initial_map_from_base, frames,options);
  result.candidate=replay(result.graph_result.optimized_map_from_base, frames,options);
  result.mapped_keyframes=frames.size();
  result.inserted_rays=inserted;
  result.incumbent_score=score(*result.incumbent.snapshot,reference,options);
  result.candidate_score=score(*result.candidate.snapshot,reference,options);
  result.authoritative=result.incumbent.snapshot;
  if (result.graph_result.accepted_loops == 0)
    result.decision=PromotionDecision::no_accepted_loop;
  else if (result.candidate_score.attempted < options.minimum_heldout_hits)
    result.decision=PromotionDecision::insufficient_reference;
  else if (result.candidate_score.known_fraction() + options.maximum_known_fraction_regression <
           result.incumbent_score.known_fraction())
    result.decision=PromotionDecision::known_coverage_regression;
  else if (result.candidate_score.hit_fraction() <
           result.incumbent_score.hit_fraction() + options.minimum_hit_fraction_gain)
    result.decision=PromotionDecision::insufficient_improvement;
  else {
    result.decision=PromotionDecision::promoted;
    result.authoritative=result.candidate.snapshot;
  }
  return result;
}

std::optional<mapping::GenerationRef> publish_approved_generation(
    mapping::GenerationStore& store, const MapGenerationResult& evaluated) {
  if(!evaluated.promoted())return std::nullopt;
  if(!evaluated.incumbent.snapshot || !evaluated.candidate.snapshot ||
     evaluated.authoritative != evaluated.candidate.snapshot ||
     evaluated.incumbent.snapshot->fingerprint()==evaluated.candidate.snapshot->fingerprint())
    throw std::invalid_argument("invalid promoted map-generation evidence");
  const auto current=store.recover();
  if(!current.active ||
     current.active->fingerprint!=evaluated.incumbent.snapshot->fingerprint())
    throw std::runtime_error("durable incumbent differs from evaluated snapshot");
  return store.publish(evaluated.candidate,evaluated.incumbent.snapshot->fingerprint());
}

} // namespace slam::offline
