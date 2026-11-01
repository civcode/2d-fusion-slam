#include "slam/graph/pose_graph.hpp"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <utility>

namespace slam::graph {
namespace {
bool finite_pose(const Pose2& p) {
  return p.translation.allFinite() && std::isfinite(p.yaw_rad);
}
bool valid_sigma(const PoseEdge& e) {
  return std::isfinite(e.sigma_x_m) && e.sigma_x_m >= 1e-6 && e.sigma_x_m < 100 &&
         std::isfinite(e.sigma_y_m) && e.sigma_y_m >= 1e-6 && e.sigma_y_m < 100 &&
         std::isfinite(e.sigma_yaw_rad) && e.sigma_yaw_rad >= 1e-6 && e.sigma_yaw_rad < 3.15;
}
bool verified(const FactorProvenance& p) {
  return p.source != EvidenceClass::unverified && !p.source_id.empty() &&
         p.source_id.size() <= 256 && p.clock_and_frame_verified &&
         p.independent_of_initial_trajectory;
}
gtsam::Pose2 gpose(const Pose2& p) {
  return {p.translation.x(),p.translation.y(),p.yaw_rad};
}
Pose2 spose(const gtsam::Pose2& p) {
  return {{p.x(),p.y()}, slam::wrap_yaw(p.theta())};
}
auto diagonal(const PoseEdge& e) {
  return gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector3() << e.sigma_x_m,e.sigma_y_m,e.sigma_yaw_rad).finished());
}
void valid_edge_shape(const PoseEdge& e, std::size_t count) {
  if (e.from >= count || e.to >= count || e.from == e.to ||
      !finite_pose(e.from_to) || !valid_sigma(e))
    throw std::invalid_argument("invalid graph edge geometry/index/uncertainty");
}
} // namespace

BatchGraphResult optimize_pose_graph(const BatchGraphInput& input,
                                    const BatchGraphOptions& options) {
  const auto n=input.initial_map_from_base.size();
  if (n < 2 || n > options.max_nodes || options.max_nodes > 100000 ||
      input.odometry.size() != n-1 || input.loops.size() > options.max_loop_candidates ||
      options.max_loop_candidates > 100000 || options.minimum_loop_separation < 2 ||
      options.minimum_supported_hits < 3 || !std::isfinite(options.minimum_overlap) ||
      options.minimum_overlap <= 0 || options.minimum_overlap > 1 ||
      !std::isfinite(options.maximum_loop_residual_translation_m) ||
      options.maximum_loop_residual_translation_m <= 0 ||
      !std::isfinite(options.maximum_loop_residual_yaw_rad) ||
      options.maximum_loop_residual_yaw_rad <= 0 ||
      !std::isfinite(options.loop_huber_k) || options.loop_huber_k <= 0 ||
      options.maximum_iterations == 0 || options.maximum_iterations > 1000)
    throw std::invalid_argument("invalid pose graph size/configuration");
  for (const auto& p: input.initial_map_from_base)
    if(!finite_pose(p)) throw std::invalid_argument("non-finite initial pose");
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  // Fixed gauge: first pose is not free to drift with loop changes.
  graph.addPrior(0,gpose(input.initial_map_from_base.front()),
                 gtsam::noiseModel::Constrained::All(3));
  for(std::size_t i=0;i<n;++i) values.insert(i,gpose(input.initial_map_from_base[i]));
  std::set<std::string> odometry_sources;
  for(std::size_t i=0;i<input.odometry.size();++i){
    const auto& e=input.odometry[i];
    valid_edge_shape(e,n);
    if(e.from!=i || e.to!=i+1 || !verified(e.provenance))
      throw std::invalid_argument("odometry edges must be consecutive and independently verified");
    odometry_sources.insert(e.provenance.source_id);
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
        e.from,e.to,gpose(e.from_to),diagonal(e));
  }
  BatchGraphResult out;
  std::set<std::pair<std::size_t,std::size_t>> used;
  out.loop_decisions.reserve(input.loops.size());
  for (const auto& candidate:input.loops) {
    const auto& e=candidate.edge;
    valid_edge_shape(e,n);
    if(e.from >= e.to || e.to-e.from < options.minimum_loop_separation ||
       used.contains({e.from,e.to})){
      out.loop_decisions.push_back(LoopDecision::duplicate_or_nearby_nodes);continue;
    }
    if (!verified(e.provenance) ||
        (e.provenance.source == EvidenceClass::independently_measured &&
         odometry_sources.contains(e.provenance.source_id))) {
      out.loop_decisions.push_back(LoopDecision::unverified_or_correlated_evidence);continue;
    }
    if(candidate.supported_hits < options.minimum_supported_hits ||
       !std::isfinite(candidate.overlap) || candidate.overlap < options.minimum_overlap ||
       candidate.overlap > 1 || !candidate.full_rank) {
      out.loop_decisions.push_back(LoopDecision::insufficient_geometry);continue;
    }
    const auto predicted=slam::compose(slam::inverse(input.initial_map_from_base[e.from]),
                                        input.initial_map_from_base[e.to]);
    const auto discrepancy=slam::compose(slam::inverse(e.from_to),predicted);
    if (discrepancy.translation.norm() > options.maximum_loop_residual_translation_m ||
        std::abs(discrepancy.yaw_rad) > options.maximum_loop_residual_yaw_rad) {
      out.loop_decisions.push_back(LoopDecision::inconsistent_with_initial_path);continue;
    }
    auto robust=gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(options.loop_huber_k), diagonal(e));
    used.insert({e.from,e.to}); // only a verified accepted candidate consumes the pair
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
        e.from,e.to,gpose(e.from_to),robust);
    out.loop_decisions.push_back(LoopDecision::accepted);++out.accepted_loops;
  }
  out.initial_cost=graph.error(values);
  if (!std::isfinite(out.initial_cost))
    throw std::invalid_argument("non-finite initial graph objective");
  try {
    gtsam::LevenbergMarquardtParams params;
    params.maxIterations=static_cast<int>(options.maximum_iterations);
    params.verbosityLM=gtsam::LevenbergMarquardtParams::SILENT;
    auto result=gtsam::LevenbergMarquardtOptimizer(graph,values,params).optimize();
    out.final_cost=graph.error(result);
    if (!std::isfinite(out.final_cost) || out.final_cost > out.initial_cost+1e-7)
      throw std::runtime_error("pose graph optimization did not reduce objective");
    out.optimized_map_from_base.reserve(n);
    for(std::size_t i=0;i<n;++i){
      const auto pose=spose(result.at<gtsam::Pose2>(i));
      if(!finite_pose(pose))throw std::runtime_error("pose graph returned nonfinite pose");
      out.optimized_map_from_base.push_back(pose);
    }
  } catch(const gtsam::ValuesKeyDoesNotExist&) {
    throw std::runtime_error("GTSAM optimized graph missing node");
  }
  out.fixed_first_pose=(out.optimized_map_from_base[0].translation-
                         input.initial_map_from_base[0].translation).norm()<1e-9 &&
                         std::abs(slam::wrap_yaw(out.optimized_map_from_base[0].yaw_rad-
                             input.initial_map_from_base[0].yaw_rad))<1e-9;
  if(!out.fixed_first_pose)throw std::runtime_error("GTSAM gauge anchor drifted");
  return out;
}
} // namespace slam::graph
