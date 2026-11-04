#include "slam/datasets/fr079_loop_closure.hpp"

#include "slam/core/se2.hpp"
#include "slam/mapping/occupancy.hpp"
#include "slam/matching/scan_to_scan.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace slam::datasets {
namespace {
using Cloud=std::vector<Eigen::Vector2d>;
constexpr std::size_t kKeyframeStride=8;
constexpr std::size_t kTemporalExclusion=120;
constexpr std::size_t kMaxHypotheses=6;
constexpr std::size_t kMaxLoops=24;
constexpr std::size_t kDistinctAnchorGap=24;
constexpr double kNearbyM=2.8;
constexpr double kNearbyYaw=.52;

void need(bool ok,const char* message) {
  if(!ok)throw std::invalid_argument(message);
}
Cloud scan_hits(const CarmenLaser& scan,double max_range) {
  Cloud out;
  out.reserve(scan.ranges.size());
  for(const auto& beam:scan.ranges) {
    if(beam.status!=RangeStatus::hit||!std::isfinite(beam.metres)||
       !std::isfinite(beam.angle_rad)||beam.metres<.1||beam.metres>max_range)
      continue;
    out.emplace_back(beam.metres*std::cos(beam.angle_rad),
                     beam.metres*std::sin(beam.angle_rad));
  }
  return out;
}
std::size_t gap(std::size_t a,std::size_t b){return a>b?a-b:b-a;}
const char* decision(slam::graph::LoopDecision d) {
  using D=slam::graph::LoopDecision;
  switch(d) {
    case D::accepted:return "accepted";
    case D::insufficient_geometry:return "graph_insufficient_geometry";
    case D::unverified_or_correlated_evidence:return "graph_unverified_evidence";
    case D::inconsistent_with_initial_path:return "graph_inconsistent_with_prior";
    case D::duplicate_or_nearby_nodes:return "graph_duplicate_or_nearby";
  }
  return "unknown";
}
mapping::MapBundle build_map(const std::vector<CarmenLaser>& scans,
                            const std::vector<Pose2>& poses,
                            const mapping::GridConfig& config,
                            const Fr079SlamOptions& options,
                            const std::string& provenance) {
  need(scans.size()==poses.size(),"graph map and scan count mismatch");
  mapping::OccupancyGrid grid(config);
  for(std::size_t i=0;i<scans.size();++i) {
    std::vector<mapping::WorldRay> rays;
    const auto& beams=scans[i].ranges;
    rays.reserve((beams.size()+options.map_beam_stride-1)/
                 options.map_beam_stride);
    for(std::size_t b=0;b<beams.size();b+=options.map_beam_stride) {
      const auto& beam=beams[b];
      if(beam.status!=RangeStatus::hit||!std::isfinite(beam.metres)||
         !std::isfinite(beam.angle_rad)||beam.metres<.10||
         beam.metres>options.max_hit_range_m)continue;
      const Eigen::Vector2d xy{beam.metres*std::cos(beam.angle_rad),
                               beam.metres*std::sin(beam.angle_rad)};
      rays.push_back({poses[i].translation,slam::transform(poses[i],xy),
                      sensors::ReturnKind::hit});
    }
    if(!rays.empty())(void)grid.integrate_rays(rays);
  }
  return {mapping::MapSnapshot::capture(grid),
          {"fr079_odom_scan_frame_UNVERIFIED",provenance,
           "estimated_trajectory_no_independent_ground_truth",false}};
}
struct Candidate {
  slam::graph::LoopCandidate loop;
  Fr079LoopEvent event;
};
} // namespace

Fr079LoopResult run_fr079_loop_experiment(
    const std::vector<CarmenLaser>& scans,const Fr079SlamResult& frontend,
    const Fr079SlamOptions& options,bool frontend_sequential_graph) {
  need(scans.size()==frontend.frames.size()&&scans.size()>kTemporalExclusion,
       "FR079 loop experiment requires >120 aligned scans and frontend poses");
  slam::graph::BatchGraphInput input;
  input.initial_map_from_base.reserve(scans.size());
  input.odometry.reserve(scans.size()-1);
  for(const auto& frame:frontend.frames)
    input.initial_map_from_base.push_back(frame.estimated);

  // CARMEN embeds odometry evaluated at each scan time. These increments
  // do NOT derive from accepted Ceres map corrections, are NOT ground truth,
  // and have no independent laser-extrinsic calibration. Here verified
  // refers to the parsed same-log timestamp/frame convention only.
  const slam::graph::FactorProvenance odom{
      slam::graph::EvidenceClass::independently_measured,
      "fr079_recorded_odom_scan_time_v1",true,true};
  const slam::graph::FactorProvenance frontend_relative{
      slam::graph::EvidenceClass::frontend_scanmatched_correlated,
      "fr079_frontend_relative_correlated_v1",true,false};
  for(std::size_t i=1;i<scans.size();++i) {
    const auto& poses=frontend.frames;
    const Pose2 increment=frontend_sequential_graph?
        slam::compose(slam::inverse(poses[i-1].estimated),poses[i].estimated):
        slam::compose(slam::inverse(poses[i-1].odom),poses[i].odom);
    input.odometry.push_back({i-1,i,increment,.10,.10,.08,
                              frontend_sequential_graph?frontend_relative:odom});
  }

  // First optimize the *identical* odometry graph without loops. This
  // control is essential: raw odometry factors may already move a
  // map relative to the incremental Ceres frontend.
  slam::graph::BatchGraphOptions graph_settings;
  graph_settings.maximum_iterations=75;
  graph_settings.maximum_loop_residual_translation_m=.8;
  graph_settings.maximum_loop_residual_yaw_rad=.30;
  const auto control=slam::graph::optimize_pose_graph(input,graph_settings);
  need(control.fixed_first_pose&&control.optimized_map_from_base.size()==scans.size(),
       "odometry-only graph lost its fixed first pose");

  Fr079LoopResult out;
  out.odometry_graph_poses=control.optimized_map_from_base;
  out.sequential_factor_source=frontend_sequential_graph?
      "frontend_scanmatched_relative_correlated_v1":"logged_odometry_scan_time_v1";
  if(frontend_sequential_graph) {
    // With no loop factors, consecutive frontend constraints must recover
    // the input frontend trajectory, not silently create a new odometry map.
    for(std::size_t i=0;i<scans.size();++i) {
      const auto& a=out.odometry_graph_poses[i];
      const auto& b=frontend.frames[i].estimated;
      need((a.translation-b.translation).norm()<1e-4 &&
           std::abs(slam::wrap_yaw(a.yaw_rad-b.yaw_rad))<1e-4,
           "frontend graph control failed to preserve frontend trajectory");
    }
  }
  out.control_initial_cost=control.initial_cost;
  out.control_final_cost=control.final_cost;

  // Scan-to-scan hypotheses use ONLY the accepted frontend estimate to
  // select neighbors; the loop measurement comes from reciprocal raw
  // LiDAR registrations, not a copied pose difference or logged laser pose.
  std::vector<Cloud> clouds(scans.size());
  for(std::size_t i=0;i<scans.size();i+=kKeyframeStride)
    clouds[i]=scan_hits(scans[i],options.max_hit_range_m);

  std::vector<Candidate> selected;
  for(std::size_t i=kTemporalExclusion+kKeyframeStride;
      i<scans.size()&&selected.size()<kMaxLoops;i+=kKeyframeStride) {
    const auto& current=clouds[i];
    if(current.size()<80)continue;
    struct Neighbor {std::size_t index;double distance;};
    std::vector<Neighbor> nearby;
    for(std::size_t j=0;j+kTemporalExclusion<i;j+=kKeyframeStride) {
      const auto relative=slam::compose(
          slam::inverse(input.initial_map_from_base[j]),
          input.initial_map_from_base[i]);
      if(relative.translation.norm()<=kNearbyM &&
         std::abs(slam::wrap_yaw(relative.yaw_rad))<=kNearbyYaw)
        nearby.push_back({j,relative.translation.norm()});
    }
    out.nearby_pairs+=nearby.size();
    std::sort(nearby.begin(),nearby.end(),[](const auto& a,const auto& b){
      return a.distance==b.distance?a.index<b.index:a.distance<b.distance;
    });

    std::vector<Candidate> valid;
    for(std::size_t k=0;k<std::min(kMaxHypotheses,nearby.size());++k) {
      const auto j=nearby[k].index;
      const auto& previous=clouds[j];
      if(previous.size()<80)continue;
      const Pose2 prior=slam::compose(
          slam::inverse(input.initial_map_from_base[j]),
          input.initial_map_from_base[i]);
      auto registered=slam::matching::register_consecutive_scans(
          previous,current,prior,false,{3.0,.7});
      ++out.registration_attempts;
      const double overlap=std::min(registered.forward_overlap,
                                    registered.reverse_overlap);
      const auto change=slam::compose(
          slam::inverse(prior),registered.previous_from_current);
      Fr079LoopEvent event{j,i,"registration_rejected",
          registered.forward_inliers,registered.reverse_inliers,overlap,
          registered.mean_error_m,change.translation.norm(),
          std::abs(slam::wrap_yaw(change.yaw_rad))};
      if(!registered.accepted||!registered.correspondence_rank||
         registered.forward_inliers<80||registered.reverse_inliers<80||
         overlap<.82||registered.mean_error_m>.11||
         event.innovation_m>.55||event.innovation_yaw_rad>.17) {
        out.decisions.push_back(event);
        continue;
      }
      const auto reverse=slam::matching::register_consecutive_scans(
          current,previous,slam::inverse(registered.previous_from_current),
          false,{3.0,.7});
      const auto cycle=slam::compose(registered.previous_from_current,
                                     reverse.previous_from_current);
      if(!reverse.accepted||!reverse.correspondence_rank||
         reverse.forward_inliers<80||reverse.reverse_inliers<80||
         std::min(reverse.forward_overlap,reverse.reverse_overlap)<.82||
         cycle.translation.norm()>.14||
         std::abs(slam::wrap_yaw(cycle.yaw_rad))>.07) {
        event.decision="reciprocal_consistency_rejected";
        out.decisions.push_back(event);
        continue;
      }
      ++out.geometry_verified;
      const slam::graph::FactorProvenance lidar{
          slam::graph::EvidenceClass::lidar_scan_registration,
          "fr079_raw_scan_reciprocal_registration_v1",true,false};
      event.decision="geometry_verified";
      valid.push_back({{{j,i,registered.previous_from_current,
                          .30,.30,.20,lidar},
                         std::min(registered.forward_inliers,
                                  registered.reverse_inliers),
                         overlap,true,registered.mean_error_m},event});
    }
    std::sort(valid.begin(),valid.end(),[](const auto& a,const auto& b){
      if(a.loop.overlap!=b.loop.overlap)
        return a.loop.overlap>b.loop.overlap;
      if(a.loop.registration_error_m!=b.loop.registration_error_m)
        return a.loop.registration_error_m<b.loop.registration_error_m;
      return a.loop.edge.from<b.loop.edge.from;
    });
    bool chosen=false;
    for(auto& item:valid) {
      const bool redundant=chosen||std::any_of(
          selected.begin(),selected.end(),[&](const auto& existing) {
            return gap(existing.loop.edge.from,item.loop.edge.from)<
                      kDistinctAnchorGap ||
                   gap(existing.loop.edge.to,item.loop.edge.to)<
                      kDistinctAnchorGap;
          });
      if(redundant) {
        item.event.decision="redundant_revisit_suppressed";
        ++out.redundant_suppressed;
        out.decisions.push_back(item.event);
      }else {
        selected.push_back(std::move(item));
        chosen=true;
      }
    }
  }
  for(const auto& item:selected)input.loops.push_back(item.loop);
  out.proposed_loops=input.loops.size();

  const auto result=slam::graph::optimize_pose_graph(input,graph_settings);
  need(result.fixed_first_pose&&
       result.optimized_map_from_base.size()==scans.size()&&
       result.loop_decisions.size()==input.loops.size(),
       "invalid FR079 graph optimization result");
  out.accepted_loops=result.accepted_loops;
  out.loop_graph_poses=result.optimized_map_from_base;
  out.loop_initial_cost=result.initial_cost;
  out.loop_final_cost=result.final_cost;
  out.first_pose_fixed=result.fixed_first_pose;
  for(std::size_t i=0;i<selected.size();++i) {
    auto event=selected[i].event;
    event.decision=decision(result.loop_decisions[i]);
    out.decisions.push_back(std::move(event));
  }
  double total_shift=0;
  out.graph_shift_within_safety_limit=true;
  for(std::size_t i=0;i<scans.size();++i) {
    const auto& a=out.odometry_graph_poses[i];
    const auto& b=out.loop_graph_poses[i];
    const double shift=(b.translation-a.translation).norm();
    const double angle=std::abs(slam::wrap_yaw(b.yaw_rad-a.yaw_rad));
    total_shift+=shift;
    out.maximum_graph_shift_m=std::max(out.maximum_graph_shift_m,shift);
    out.maximum_graph_shift_yaw_rad=
        std::max(out.maximum_graph_shift_yaw_rad,angle);
    if(shift>3.0||angle>.9)out.graph_shift_within_safety_limit=false;
  }
  out.mean_graph_shift_m=total_shift/scans.size();
  // Same map cell dimensions, beam subset and source observations for both
  // graph arms. No future truth or archived FR079 map used in estimation.
  const auto& config=frontend.map.snapshot->config();
  out.odometry_graph_map=build_map(
      scans,out.odometry_graph_poses,config,options,
      frontend_sequential_graph?
          "fr079-correlated-frontend-graph-control-v1":
          "fr079-recorded-odometry-graph-control-v1");
  out.loop_graph_map=build_map(
      scans,out.loop_graph_poses,config,options,
      frontend_sequential_graph?
          "fr079-correlated-frontend-graph-with-lidar-loops-v1":
          "fr079-recorded-odometry-graph-with-lidar-loops-v1");
  return out;
}
} // namespace slam::datasets
