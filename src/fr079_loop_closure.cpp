#include "slam/datasets/fr079_loop_closure.hpp"

#include "slam/core/se2.hpp"
#include "slam/mapping/occupancy.hpp"
#include "slam/matching/scan_to_scan.hpp"
#include "slam/matching/scan_to_submap.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
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
constexpr double kLargeHeadingRad=1.57079632679489661923; // >90 degrees
constexpr std::size_t kSubmapHalfWindow=32;  // scans on either side of anchor
constexpr std::size_t kSubmapSampleStride=3;
constexpr std::size_t kLateScanStart=1769; // old 24-loop early-stop boundary
constexpr std::size_t kGlobalClusterLimit=2;
constexpr double kMinDescriptorSimilarity=.82;

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
    const Fr079SlamOptions& options,bool frontend_sequential_graph,
    Fr079LoopRegistrationMode registration_mode) {
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

  // Optimize the selected sequential graph without loops. Frontend-derived
  // factors must preserve Ceres exactly; the old raw-odometry mode remains
  // available separately for diagnostic compatibility.
  slam::graph::BatchGraphOptions graph_settings;
  graph_settings.maximum_iterations=75;
  graph_settings.maximum_loop_residual_translation_m=.8;
  graph_settings.maximum_loop_residual_yaw_rad=.30;
  const bool distributed_mode=
      registration_mode==Fr079LoopRegistrationMode::distributed_scan_to_submap ||
      registration_mode==Fr079LoopRegistrationMode::globally_retrieved_scan_to_submap;
  const bool appearance_mode=
      registration_mode==Fr079LoopRegistrationMode::globally_retrieved_scan_to_submap;
  const bool submap_mode=
      registration_mode==Fr079LoopRegistrationMode::correlative_scan_to_submap ||
      distributed_mode;
  // A submap's current-scan support fraction is not symmetric two-scan
  // overlap. The submap factor has its own stronger geometry/quality gates.
  if(submap_mode)graph_settings.minimum_overlap=.58;
  const auto control=slam::graph::optimize_pose_graph(input,graph_settings);
  need(control.fixed_first_pose&&control.optimized_map_from_base.size()==scans.size(),
       "odometry-only graph lost its fixed first pose");

  Fr079LoopResult out;
  const bool pose_prior_scan =
      registration_mode==Fr079LoopRegistrationMode::pose_prior_scan_to_scan;
  out.registration_mode=appearance_mode?"global_scan_to_submap_v2":
      (distributed_mode?"distributed_scan_to_submap_v2":
        (submap_mode?"correlative_scan_to_submap_v1":
          (pose_prior_scan?"pose_prior_scan_to_scan_v1":"legacy_scan_to_scan_v1")));
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

  // All hypothesis selection uses the accepted frontend poses only.
  // Actual loop measurements come from raw LiDAR scan-to-scan or frozen
  // historical scan-to-submap registration, never from copied Ceres relative
  // poses, future reference poses or the mutable global Ceres map.
  std::vector<Cloud> clouds(scans.size());
  for(std::size_t i=0;i<scans.size();i+=kKeyframeStride)
    clouds[i]=scan_hits(scans[i],options.max_hit_range_m);

  // Immutable local historical submaps are built once per anchor, only in
  // the opt-in submap arm. Each window ends at least 120 scans before its
  // candidate current scan, preventing temporal-neighbor self-validation.
  std::vector<std::optional<slam::matching::FrozenLidarSubmap>> submap_cache(
      submap_mode?scans.size():0);
  std::vector<std::optional<slam::matching::SubmapDescriptor>> descriptor_cache(
      appearance_mode?scans.size():0);
  auto historical_submap=[&](std::size_t j)
      -> const slam::matching::FrozenLidarSubmap& {
    auto& cached=submap_cache.at(j);
    if(!cached) {
      std::vector<slam::matching::SubmapSweep> sweeps;
      const auto first=j>kSubmapHalfWindow?j-kSubmapHalfWindow:0;
      const auto last=std::min(scans.size()-1,j+kSubmapHalfWindow);
      for(std::size_t k=first;k<=last;k+=kSubmapSampleStride) {
        const auto relative=slam::compose(
            slam::inverse(input.initial_map_from_base[j]),
            input.initial_map_from_base[k]);
        sweeps.push_back({relative,scan_hits(scans[k],options.max_hit_range_m)});
      }
      // Always include the keyframe anchor, even if a window's sampling
      // phase did not land on it.
      if((j-first)%kSubmapSampleStride!=0)
        sweeps.push_back({Pose2{},scan_hits(scans[j],options.max_hit_range_m)});
      cached=slam::matching::build_frozen_lidar_submap(sweeps);
      ++out.submaps_built;
    }
    return *cached;
  };

  std::vector<Candidate> selected;
  // Distributed modes do not stop matching when a provisional factor
  // budget is reached: they collect candidates over the *entire* log.
  std::vector<Candidate> pool;
  for(std::size_t i=kTemporalExclusion+kKeyframeStride;
      i<scans.size() && (distributed_mode||selected.size()<kMaxLoops);
      i+=kKeyframeStride) {
    out.last_keyframe_searched=i;
    const auto& current=clouds[i];
    if(current.size()<80)continue;
    struct Neighbor {std::size_t index;double distance,heading_difference;};
    std::vector<Neighbor> nearby;
    for(std::size_t j=0;j+kTemporalExclusion<i;j+=kKeyframeStride) {
      const auto relative=slam::compose(
          slam::inverse(input.initial_map_from_base[j]),
          input.initial_map_from_base[i]);
      const double heading_difference=
          std::abs(slam::wrap_yaw(relative.yaw_rad));
      if(relative.translation.norm()<=kNearbyM &&
         (pose_prior_scan||submap_mode||heading_difference<=kNearbyYaw) &&
         (!submap_mode ||
          j+kSubmapHalfWindow+kTemporalExclusion<=i)) {
        nearby.push_back({j,relative.translation.norm(),heading_difference});
        if(heading_difference>=kLargeHeadingRad)
          ++out.large_heading_nearby_pairs;
      }
    }
    out.nearby_pairs+=nearby.size();
    std::sort(nearby.begin(),nearby.end(),[](const auto& a,const auto& b){
      return a.distance==b.distance?a.index<b.index:a.distance<b.distance;
    });

    // In pose-prior mode reserve up to half the registration attempts for
    // large heading differences. Without this, the closest six candidates
    // could all be same-direction and starve reverse corridor revisits.
    // Remaining capacity is filled by nearest candidates of either class.
    std::vector<std::size_t> registrations;
    if(pose_prior_scan||submap_mode) {
      for(int large=0;large<2;++large) {
        std::size_t allocated=0;
        for(std::size_t k=0;k<nearby.size()&&allocated<kMaxHypotheses/2;++k) {
          if((nearby[k].heading_difference>=kLargeHeadingRad)==
              (large==1)) {
            registrations.push_back(k);
            ++allocated;
          }
        }
      }
    }
    for(std::size_t k=0;k<nearby.size()&&
        registrations.size()<kMaxHypotheses;++k)
      if(std::find(registrations.begin(),registrations.end(),k)==
         registrations.end())
        registrations.push_back(k);
    std::sort(registrations.begin(),registrations.end());
    // Appearance-retrieved submap clusters come from shape (pair-distance)
    // descriptors, not from the predicted Ceres XY distance. The descriptor
    // is only a candidate generator; matching and graph verification still
    // decide whether there is evidence of a revisit.
    struct AppearanceProposal {std::size_t index;double similarity;};
    std::vector<AppearanceProposal> global_candidates;
    if(appearance_mode && i%16==0) {
      std::vector<slam::matching::SubmapSweep> query_sweeps;
      const auto first=i>kSubmapHalfWindow?i-kSubmapHalfWindow:0;
      for(std::size_t k=first;k<=i;k+=kSubmapSampleStride) {
        const Pose2 relative=slam::compose(
            slam::inverse(input.initial_map_from_base[i]),
            input.initial_map_from_base[k]);
        query_sweeps.push_back({relative,scan_hits(scans[k],options.max_hit_range_m)});
      }
      const auto query_map=slam::matching::build_frozen_lidar_submap(query_sweeps);
      const auto descriptor=slam::matching::describe_frozen_lidar_submap(query_map);
      std::vector<AppearanceProposal> ranked;
      if(descriptor.valid) {
        for(std::size_t j=0;j+kTemporalExclusion+kSubmapHalfWindow<=i;
            j+=kKeyframeStride) {
          const auto relative=slam::compose(
              slam::inverse(input.initial_map_from_base[j]),
              input.initial_map_from_base[i]);
          if(relative.translation.norm()<=kNearbyM)continue;
          if(!descriptor_cache[j])
            descriptor_cache[j]=slam::matching::describe_frozen_lidar_submap(
                historical_submap(j));
          const double sim=slam::matching::submap_descriptor_similarity(
              descriptor,*descriptor_cache[j]);
          if(sim>=kMinDescriptorSimilarity)ranked.push_back({j,sim});
        }
      }
      std::sort(ranked.begin(),ranked.end(),[](const auto& a,const auto& b) {
        return a.similarity==b.similarity?a.index<b.index:
                                           a.similarity>b.similarity;
      });
      std::vector<std::size_t> cluster_centres;
      for(const auto& r:ranked) {
        if(cluster_centres.size()>=kGlobalClusterLimit)break;
        if(std::any_of(cluster_centres.begin(),cluster_centres.end(),
             [&](std::size_t j){return gap(j,r.index)<64;}))continue;
        cluster_centres.push_back(r.index);
        global_candidates.push_back(r);
        // A second nearby anchor provides cross-anchor agreement evidence.
        // Both observations are correlated; this is not independent truth.
        for(const int delta:{-8,8}) {
          const auto neighbor=static_cast<long long>(r.index)+delta;
          if(neighbor<0)continue;
          const auto it=std::find_if(ranked.begin(),ranked.end(),
              [&](const auto& x){return x.index==static_cast<std::size_t>(neighbor);});
          if(it!=ranked.end() && it->similarity>=kMinDescriptorSimilarity)
            {global_candidates.push_back(*it);break;}
        }
      }
      out.appearance_retrieved_candidates+=global_candidates.size();
    }
    std::vector<Candidate> valid;
    for(std::size_t q=0;q<registrations.size()+global_candidates.size();++q) {
      const bool global=q>=registrations.size();
      const std::size_t k=global?0:registrations[q];
      const std::size_t j=global?
          global_candidates[q-registrations.size()].index:nearby[k].index;
      const double heading_difference=global?
          std::abs(slam::wrap_yaw(slam::compose(
              slam::inverse(input.initial_map_from_base[j]),
              input.initial_map_from_base[i]).yaw_rad)):
          nearby[k].heading_difference;
      const bool large_heading=heading_difference>=kLargeHeadingRad;
      const auto& previous=clouds[j];
      if(previous.size()<80)continue;
      const Pose2 prior=slam::compose(
          slam::inverse(input.initial_map_from_base[j]),
          input.initial_map_from_base[i]);
      if(submap_mode) {
        const auto& map=historical_submap(j);
        const auto match=slam::matching::register_scan_to_frozen_submap(
            map,current,prior,global?
              slam::matching::SubmapSearchLimits{4.5,.55,.70,.60,.08,.12,80,true}:
              slam::matching::SubmapSearchLimits{});
        ++out.registration_attempts;
        ++out.submap_registration_attempts;
        if(global)++out.appearance_registration_attempts;
        if(large_heading)++out.large_heading_registration_attempts;
        Fr079LoopEvent event{j,i,"submap_registration_rejected",
            match.supported_hits,0,match.support_fraction,match.rms_m,
            match.innovation_m,match.innovation_yaw_rad};
        event.predicted_relative_yaw_rad=heading_difference;
        event.appearance_retrieved=global;
        event.retrieval_similarity=global?
            global_candidates[q-registrations.size()].similarity:0.;
        event.submap_contributing_scans=map.contributing_scans;
        event.submap_score=match.correlation_score;
        event.submap_support_fraction=match.support_fraction;
        event.submap_translation_curvature_min=match.translation_curvature_min;
        event.submap_translation_curvature_ratio=match.translation_curvature_ratio;
        event.submap_minimum_shift_score_drop=match.minimum_shift_score_drop;
        if(!match.accepted) {
          if(!match.observable) {
            event.decision="submap_ambiguous_geometry_rejected";
            ++out.submap_ambiguous_rejected;
          }
          out.decisions.push_back(std::move(event));
          continue;
        }
        ++out.geometry_verified;
        ++out.submap_geometry_verified;
        if(global)++out.appearance_geometry_verified;
        if(large_heading)++out.large_heading_geometry_verified;
        const slam::graph::FactorProvenance lidar{
            global?
              slam::graph::EvidenceClass::lidar_global_scan_to_submap_registration:
              slam::graph::EvidenceClass::lidar_scan_to_submap_registration,
            global?"fr079_raw_scans_global_submap_shape_correlative_v2":
                   "fr079_raw_scans_frozen_submap_correlative_v1",
            true,false};
        event.decision="geometry_verified";
        valid.push_back({{{j,i,match.anchor_from_current,
                            global?.60:.40,global?.60:.40,global?.30:.24,lidar},
                           match.supported_hits,
                           match.support_fraction,match.observable,
                           match.rms_m},event});
        continue;
      }
      auto registered=pose_prior_scan?
          slam::matching::register_scans_from_pose_prior(
              previous,current,prior,false,{3.0,.55,.17}):
          slam::matching::register_consecutive_scans(
              previous,current,prior,false,{3.0,.7});
      ++out.registration_attempts;
      if(large_heading)++out.large_heading_registration_attempts;
      const double overlap=std::min(registered.forward_overlap,
                                    registered.reverse_overlap);
      const auto change=slam::compose(
          slam::inverse(prior),registered.previous_from_current);
      Fr079LoopEvent event{j,i,"registration_rejected",
          registered.forward_inliers,registered.reverse_inliers,overlap,
          registered.mean_error_m,change.translation.norm(),
          std::abs(slam::wrap_yaw(change.yaw_rad))};
      event.predicted_relative_yaw_rad=nearby[k].heading_difference;
      if(!registered.accepted||!registered.correspondence_rank||
         registered.forward_inliers<80||registered.reverse_inliers<80||
         overlap<.82||registered.mean_error_m>.11||
         event.innovation_m>.55||event.innovation_yaw_rad>.17) {
        out.decisions.push_back(event);
        continue;
      }
      const auto reverse=pose_prior_scan?
          slam::matching::register_scans_from_pose_prior(
              current,previous,slam::inverse(registered.previous_from_current),
              false,{3.0,.55,.17}):
          slam::matching::register_consecutive_scans(
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
      if(large_heading)++out.large_heading_geometry_verified;
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
    if(appearance_mode) {
      for(auto& item:valid) {
        if(!item.event.appearance_retrieved)continue;
        const auto& pose_j=input.initial_map_from_base[item.loop.edge.from];
        const auto world_pose=slam::compose(pose_j,item.loop.edge.from_to);
        const bool agreed=std::any_of(valid.begin(),valid.end(),[&](const auto& other) {
          if(!other.event.appearance_retrieved ||
             other.loop.edge.from==item.loop.edge.from ||
             gap(other.loop.edge.from,item.loop.edge.from)>32)return false;
          const auto other_world=slam::compose(
              input.initial_map_from_base[other.loop.edge.from],
              other.loop.edge.from_to);
          return (other_world.translation-world_pose.translation).norm()<=.55 &&
                 std::abs(slam::wrap_yaw(other_world.yaw_rad-world_pose.yaw_rad))<=.12;
        });
        item.event.multi_anchor_consistent=agreed;
        if(!agreed) {
          item.event.decision="global_cross_anchor_rejected";
          ++out.appearance_corrob_rejected;
          out.decisions.push_back(item.event);
        }
      }
      valid.erase(std::remove_if(valid.begin(),valid.end(),[](const auto& v){
        return v.event.appearance_retrieved && !v.event.multi_anchor_consistent;
      }),valid.end());
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
      const bool redundant=chosen||(!distributed_mode && std::any_of(
          selected.begin(),selected.end(),[&](const auto& existing) {
            return gap(existing.loop.edge.from,item.loop.edge.from)<
                      kDistinctAnchorGap ||
                   gap(existing.loop.edge.to,item.loop.edge.to)<
                      kDistinctAnchorGap;
          })); 
      if(redundant) {
        item.event.decision="redundant_revisit_suppressed";
        ++out.redundant_suppressed;
        out.decisions.push_back(item.event);
      }else {
        if(distributed_mode) {
          if(item.event.to>=kLateScanStart)++out.late_verified_candidates;
          pool.push_back(std::move(item));
        } else selected.push_back(std::move(item));
        chosen=true;
      }
    }
  }
  if(distributed_mode) {
    out.verified_candidate_pool=pool.size();
    // Allocate across the whole recording, then use leftover budget for
    // highest-quality remaining matches. This prevents the first revisit
    // from exhausting all 24 slots and preserves spatial anchor diversity.
    constexpr std::size_t bins=8;
    std::array<std::vector<std::size_t>,bins> per_bin;
    for(std::size_t k=0;k<pool.size();++k) {
      const auto b=std::min<std::size_t>(
          bins-1,pool[k].loop.edge.to*bins/scans.size());
      per_bin[b].push_back(k);
    }
    const auto quality=[&](std::size_t a,std::size_t b) {
      const auto& x=pool[a].loop;
      const auto& y=pool[b].loop;
      if(x.overlap!=y.overlap)return x.overlap>y.overlap;
      if(x.registration_error_m!=y.registration_error_m)
        return x.registration_error_m<y.registration_error_m;
      return x.edge.to<y.edge.to;
    };
    for(auto& list:per_bin)std::sort(list.begin(),list.end(),quality);
    std::vector<bool> used(pool.size(),false);
    auto choose=[&](std::size_t k) {
      if(used[k])return false;
      const auto& item=pool[k];
      if(std::any_of(selected.begin(),selected.end(),
           [&](const auto& existing) {
             // Multiple later traversals may legitimately revisit the
             // same historical anchor. Suppress only *pair* duplicates;
             // the old single-endpoint OR policy starved later loops.
             return gap(existing.loop.edge.from,item.loop.edge.from)<
                       kDistinctAnchorGap &&
                    gap(existing.loop.edge.to,item.loop.edge.to)<
                       kDistinctAnchorGap;
           }))return false;
      selected.push_back(item);
      used[k]=true;
      return true;
    };
    // Each temporal bin gets three narrower slots, so the earliest strong
    // observations inside a bin cannot crowd out its final traversal.
    for(std::size_t round=0;round<kMaxLoops/bins;++round)
      for(std::size_t b=0;b<bins;++b) {
        if(selected.size()>=kMaxLoops)break;
        for(const auto k:per_bin[b]) {
          const auto slot=std::min<std::size_t>(
              kMaxLoops-1,pool[k].loop.edge.to*kMaxLoops/scans.size());
          if(slot!=b*(kMaxLoops/bins)+round)continue;
          if(choose(k))break;
        }
      }
    std::vector<std::size_t> ranked(pool.size());
    for(std::size_t k=0;k<pool.size();++k)ranked[k]=k;
    std::sort(ranked.begin(),ranked.end(),quality);
    for(const auto k:ranked) {
      if(selected.size()>=kMaxLoops)break;
      (void)choose(k);
    }
    out.redundant_suppressed+=pool.size()-selected.size();
    for(std::size_t k=0;k<pool.size();++k)
      if(!used[k]) {
        auto event=pool[k].event;
        event.decision="global_budget_or_redundancy_suppressed";
        out.decisions.push_back(std::move(event));
      }
    std::sort(selected.begin(),selected.end(),[](const auto& a,const auto& b) {
      return a.loop.edge.to==b.loop.edge.to?
          a.loop.edge.from<b.loop.edge.from:a.loop.edge.to<b.loop.edge.to;
    });
  }
  for(const auto& item:selected)input.loops.push_back(item.loop);
  out.proposed_loops=input.loops.size();

  // Existing rejected opposite-heading FR079 pairs provide deterministic,
  // diagnosis-only regression probes for the new matcher. They are NOT
  // assumed correct, are not counted as loop candidates, and can never
  // influence graph optimization. Only the known 3,118-scan experiment is
  // probed; ordinary synthetic tests use the standalone submap test.
  if(submap_mode && scans.size()==3118) {
    constexpr std::array<std::pair<std::size_t,std::size_t>,3> probes{{
        {1040,1288},{2704,2936},{536,872}}};
    for(const auto [j,i]:probes) {
      const Pose2 prior=slam::compose(
          slam::inverse(input.initial_map_from_base[j]),
          input.initial_map_from_base[i]);
      const auto& map=historical_submap(j);
      const auto match=slam::matching::register_scan_to_frozen_submap(
          map,clouds[i],prior);
      out.submap_probes.push_back({j,i,match.accepted,match.observable,
          std::abs(slam::wrap_yaw(prior.yaw_rad)),
          match.correlation_score,match.support_fraction,
          match.translation_curvature_min,match.translation_curvature_ratio,
          match.minimum_shift_score_drop});
    }
  }

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
    if(result.loop_decisions[i]==slam::graph::LoopDecision::accepted) {
      if(event.predicted_relative_yaw_rad>=kLargeHeadingRad)
        ++out.large_heading_accepted_loops;
      if(event.to>=kLateScanStart)++out.late_selected_loops;
      if(event.appearance_retrieved)++out.appearance_accepted_loops;
    }
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
