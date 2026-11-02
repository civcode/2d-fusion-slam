#include <cxxopts.hpp>
#include "slam/evaluation/map_accuracy.hpp"
#include "slam/fixtures/synthetic.hpp"
#include "slam/fusion/gyro_wheel.hpp"
#include "slam/fusion/pipeline_profiler.hpp"
#include "slam/graph/pose_graph.hpp"
#include "slam/mapping/map_bundle.hpp"
#include "slam/matching/geometry.hpp"
#include "slam/matching/scan_to_scan.hpp"
#include "slam/motion/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace {
using slam::Pose2;
using slam::Timestamp;
constexpr slam::TimeNs kTickNs=25'000'000LL;
constexpr slam::TimeNs kScanNs=300'000'000LL;
constexpr slam::TimeNs kInitialNs=1'000'000'000LL;
constexpr double kPi=std::numbers::pi;
struct FusionMatchOptions {
  std::size_t match_every{1};     // every eligible held-out / LiDAR-only query
  std::size_t field_every{1};     // incremental LiDAR-only EDT refresh
  std::size_t map_stride{1};     // occupancy insertion in every map
  std::size_t match_stride{1};   // every Nth valid deskewed/raw hit
  std::size_t match_offset{0};   // start offset modulo stride (0 preserves baseline)
  double distance_truncation_m{0.7};
  bool continuous_cost{};
  // Read-only bounded diagnostic around the stalled LiDAR-only seed.
  bool lidar_only_support_probe{};
  bool lidar_only_bootstrap{};
  // Alternate A/B path: LiDAR relative motion predicts pose before EDT.
  bool lidar_only_predict_first{};
};
std::size_t posint(std::string_view value,std::size_t max) {
  if(value.empty()||value.front()=='0'||
     value.find_first_not_of("0123456789")!=std::string_view::npos)
    throw std::invalid_argument("option must be a positive decimal integer");
  const auto number=std::stoull(std::string(value));
  if(number<1||number>max)throw std::invalid_argument("option exceeds allowed limit");
  return static_cast<std::size_t>(number);
}
const Pose2 kBaseFromLaser{{.12,.01},.02};

// Truth is used in the synthetic sensor GENERATOR and in independent scoring,
// NEVER in the sensor-fusion, scan-matching or map-building functions.
struct Observations {
  std::vector<slam::fusion::SensorTick> ticks;
  std::vector<slam::sensors::AssembledScan> scans;
};
struct IndependentSyntheticLoop {
  std::size_t from{}, to{};
  Pose2 measured_from_to{};
};
struct Simulation {
  slam::fixtures::Scene scene;
  slam::mapping::GridConfig grid;
  Observations sensors;
  std::vector<Pose2> reference; // withheld post-hoc scoring only
  std::vector<IndependentSyntheticLoop> independent_loop_measurements;
  Pose2 initial_anchor;
  bool warehouse{};
};
struct Motion {
  Pose2 pose;
  double forward{}, yaw_rate{};
};
struct RouteConfig {
  double cx{},cy{},rx{},ry{},period{};
  std::size_t scans{};
  bool large{};
};
// The original two routes remain bit-for-bit unchanged. New routes provide
// different speeds, curvature, observed spaces, and trajectory lengths; the
// corresponding exact poses never enter any matcher, only the simulator and
// final independent scoring.
RouteConfig route_config(const std::string& name) {
  if(name=="apartment")return {2.5,3.0,1.35,1.35,20.0,130,false};
  if(name=="warehouse")return {15.0,10.0,14.0,9.0,56.0,195,true};
  if(name=="office")return {3.1,3.0,1.6,1.0,24.0,150,false};
  if(name=="loft")return {7.5,6.0,3.5,2.5,34.0,168,false};
  if(name=="atrium")return {15.0,10.0,10.6,6.4,60.0,200,true};
  if(name=="industrial")return {15.0,10.0,13.2,8.2,57.0,195,true};
  if(name=="warehouse_asymmetric")return {15.0,10.0,14.0,9.0,56.0,195,true};
  throw std::invalid_argument("unknown fusion synthetic scene: "+name);
}
Motion at_time(const RouteConfig& route,double seconds_after_start) {
  const double active=std::max(0.0,seconds_after_start-.30);
  const double w=2*kPi/route.period,phase=w*active;
  const double sn=std::sin(phase),cs=std::cos(phase);
  const double length=std::hypot(route.rx*sn,route.ry*cs);
  Motion m;
  m.pose={{route.cx+route.rx*cs,route.cy+route.ry*sn},
          std::atan2(route.ry*cs,-route.rx*sn)};
  m.forward=seconds_after_start>=.30?w*length:0.0;
  m.yaw_rate=seconds_after_start>=.30?
    w*route.rx*route.ry/(route.rx*route.rx*sn*sn+
                         route.ry*route.ry*cs*cs):0.0;
  return m;
}
Simulation generate(const std::string& label) {
  const auto route=route_config(label);
  Simulation d;
  d.warehouse=route.large;
  d.scene=slam::fixtures::named_scene(label);
  d.grid.width=route.large?621:321;
  d.grid.height=route.large?421:261;
  d.grid.resolution_m=.05;
  d.grid.map_from_grid={{-.5,-.5},0};
  d.initial_anchor=at_time(route,0).pose;
  const std::size_t scans=route.scans;
  const std::size_t ticks=12*(scans+4)+32;
  d.sensors.ticks.reserve(ticks);
  for(std::size_t i=0;i<ticks;++i) {
    const double t=double(i)*.025;
    const Motion exact=at_time(route,t);
    slam::fusion::SensorTick sample;
    sample.time={kInitialNs+static_cast<slam::TimeNs>(i)*kTickNs,101,1};
    sample.gyro_valid=true;sample.wheel_valid=true;
    sample.stationary_calibration=i<12;
    // Synthetic sensor model: a constant gyro bias, deterministically
    // varying gyro noise, wheel scale error and independently biased
    // wheel yaw encoder. None is an oracle pose measurement.
    sample.gyro_z_rad_s=exact.yaw_rate+.025+.001*std::sin(double(i)*.13);
    sample.wheel_forward_m_s=exact.forward*1.025+
      (i<12?0.0:.006*std::sin(double(i)*.21));
    sample.wheel_yaw_rad_s=exact.yaw_rate+
      (i<12?0.0:.013+.003*std::cos(double(i)*.07));
    d.sensors.ticks.push_back(sample);
  }
  for(std::size_t i=0;i<scans;++i) {
    const auto ns=kInitialNs+600'000'000LL+
      static_cast<slam::TimeNs>(i)*kScanNs;
    const double seconds_after_start=double(ns-kInitialNs)*1e-9;
    const Motion exact=at_time(route,seconds_after_start);
    const Timestamp stamp{ns,101,1};
    const auto simulated=slam::fixtures::simulate_scan(
      d.scene,exact.pose,{exact.forward,0,exact.yaw_rate},
      kBaseFromLaser,stamp,240,100'000'000LL,d.warehouse?42.0:23.0);
    slam::sensors::AssembledScan scan;
    scan.source_id=16;scan.calibration_revision=1;
    scan.time_quality=slam::sensors::BeamTimeQuality::synthetic_exact;
    scan.midpoint=stamp;
    scan.beams.reserve(simulated.beams.size());
    for(const auto& beam:simulated.beams) {
      slam::sensors::BeamObservation out;
      out.acquired_at=beam.acquired_at;
      out.angle_rad=beam.angle_rad;
      out.range_m=beam.range_m;
      out.kind=beam.hit?slam::sensors::ReturnKind::hit:
                        slam::sensors::ReturnKind::no_return;
      out.source_id=16;
      out.calibration_revision=1;
      out.time_quality=slam::sensors::BeamTimeQuality::synthetic_exact;
      scan.beams.push_back(out);
      if(beam.hit)++scan.hit_count;else ++scan.no_return_count;
    }
    scan.first_acquired=scan.beams.front().acquired_at;
    scan.last_acquired=scan.beams.back().acquired_at;
    scan.acquisition_coverage=1.0;
    scan.partial=false;
    d.sensors.scans.push_back(std::move(scan));
    d.reference.push_back(exact.pose);
  }
  // Simulate a distinct, externally produced *relative pose sensor*. This
  // generator alone has oracle simulator access. The graph optimizer receives
  // only measured relative transforms; it never reads a reference trajectory.
  for(std::size_t i=19;i<d.reference.size();i+=2){
    if(i%4!=1)continue;
    for(std::size_t j=0;j+15<i;++j){
      if((d.reference[i].translation-d.reference[j].translation).norm()>=.65)
        continue;
      auto measurement=slam::compose(slam::inverse(d.reference[j]),d.reference[i]);
      measurement.translation.x()+=.012*std::sin(double(i));
      measurement.translation.y()+=.012*std::cos(double(i));
      measurement.yaw_rad=slam::wrap_yaw(measurement.yaw_rad+
                                        .004*std::sin(double(i)*.3));
      d.independent_loop_measurements.push_back({j,i,measurement});
      break;
    }
  }
  return d;
}
using Points=std::vector<std::vector<Eigen::Vector2d>>;
using Mask=std::vector<bool>;
struct Prepared {
  std::vector<Pose2> gyro_wheel,wheel_only;
  Points gyro_deskewed,wheel_deskewed,raw_lidar;
  slam::fusion::FusionResult integrated;
};
Prepared prepare(const Observations& obs,Pose2 initial,
                 slam::fusion::PipelineProfiler& profiler) {
  slam::fusion::FusionOptions options;
  options.minimum_stationary_ticks=12;
  options.maximum_interval_ns=50'000'000LL;
  Prepared p;
  // The synthetic fixture's shared clock domain, epoch and base/laser frame
  // are explicitly known. This is NOT a real-hardware calibration claim.
  {
    slam::fusion::PipelineProfiler::Scope timing(profiler,"sensor_integration");
    p.integrated=slam::fusion::integrate_sensors(obs.ticks,initial,true,options);
  }
  if(p.integrated.capability!=slam::motion::TrajectoryCapability::full_se2)
    throw std::runtime_error("full wheel SE2 required for moving-scan deskew");
  slam::motion::SampledTrajectory fused_track(
      p.integrated.fused,p.integrated.capability);
  slam::motion::SampledTrajectory wheel_track(
      p.integrated.wheel_only,p.integrated.capability);
  for(std::size_t scan_index=0;scan_index<obs.scans.size();++scan_index) {
    slam::fusion::PipelineProfiler::Scope timing(profiler,"scan_deskew",scan_index,true);
    const auto& scan=obs.scans[scan_index];
    if(scan.time_quality!=slam::sensors::BeamTimeQuality::synthetic_exact ||
       scan.source_id!=16||scan.calibration_revision!=1)
      throw std::invalid_argument("unverified synthetic scan metadata");
    for(const auto& beam:scan.beams)
      if(beam.source_id!=scan.source_id ||
         beam.calibration_revision!=scan.calibration_revision ||
         beam.time_quality!=scan.time_quality)
        throw std::invalid_argument("mixed beam source, calibration or time trust");
    const auto fused=fused_track.at(scan.midpoint);
    const auto wheel=wheel_track.at(scan.midpoint);
    const auto deskew_f=slam::motion::deskew_hits(scan,fused_track,kBaseFromLaser);
    const auto deskew_w=slam::motion::deskew_hits(scan,wheel_track,kBaseFromLaser);
    if(!fused||!wheel||!deskew_f||!deskew_w||
       !deskew_f->suitable_for_moving_map_insertion()||
       !deskew_w->suitable_for_moving_map_insertion())
      throw std::invalid_argument("scan outside verified timed full-SE2 trajectory");
    p.gyro_wheel.push_back(*fused);
    p.wheel_only.push_back(*wheel);
    auto& fh=p.gyro_deskewed.emplace_back();
    auto& wh=p.wheel_deskewed.emplace_back();
    auto& raw=p.raw_lidar.emplace_back();
    for(const auto& b:deskew_f->hits)fh.push_back(b.point_laser_reference);
    for(const auto& b:deskew_w->hits)wh.push_back(b.point_laser_reference);
    for(const auto& b:scan.beams)
      if(b.kind==slam::sensors::ReturnKind::hit)
        raw.emplace_back(b.range_m*std::cos(b.angle_rad),
                         b.range_m*std::sin(b.angle_rad));
    if(fh.size()!=wh.size()||fh.size()!=raw.size())
      throw std::runtime_error("deskew changed valid hit cardinality");
  }
  return p;
}
void insert(slam::mapping::OccupancyGrid& grid,Pose2 base,
            const std::vector<Eigen::Vector2d>& hits,std::size_t map_stride) {
  const auto laser=slam::compose(base,kBaseFromLaser);
  std::vector<slam::mapping::WorldRay> rays;
  rays.reserve(hits.size());
  for(std::size_t i=0;i<hits.size();i+=map_stride)
    rays.push_back({laser.translation,slam::transform(laser,hits[i]),
                    slam::sensors::ReturnKind::hit});
  (void)grid.integrate_rays(rays);
}
slam::mapping::MapBundle make_map(const Simulation& sim,
                                  const std::vector<Pose2>& poses,
                                  const Points& scans,
                                  const Mask& use,
                                  const FusionMatchOptions& options,
                                  slam::fusion::PipelineProfiler* profiler=nullptr) {
  if(poses.size()!=scans.size()||poses.size()!=use.size())
    throw std::invalid_argument("map pose/ray size mismatch");
  slam::mapping::OccupancyGrid grid(sim.grid);
  for(std::size_t i=0;i<poses.size();++i)if(use[i]) {
    if(profiler) {
      slam::fusion::PipelineProfiler::Scope sample(*profiler,"fused_map_insert",i,true);
      insert(grid,poses[i],scans[i],options.map_stride);
    } else insert(grid,poses[i],scans[i],options.map_stride);
  }
  return {slam::mapping::MapSnapshot::capture(grid),
    {"map",sim.scene.id,"synthetic_estimated",true}};
}
Mask all(std::size_t n) {return Mask(n,true);}
struct Matched {
  std::vector<Pose2> poses;
  std::vector<slam::matching::MatchResult> outcomes;
  Mask accepted_mask; // passed *all* source/rank/innovation gates
  std::size_t attempted{},accepted{},rejected{},skipped{};
  std::size_t matching_input_hits_total{};
  std::string backend;
  std::map<std::string,std::size_t> rejections;
};
std::string rejection_reason(slam::matching::MatchStatus status) {
  using slam::matching::MatchStatus;
  switch(status) {
    case MatchStatus::insufficient_support:return "insufficient_support";
    case MatchStatus::degenerate:return "degenerate";
    case MatchStatus::no_improvement:return "no_improvement";
    case MatchStatus::invalid_input:return "invalid_input";
    case MatchStatus::accepted:return "qualification_gate";
  }
  throw std::logic_error("unknown matcher status");
}
void write_rejections(std::ostream& out,const char* name,
                      const std::map<std::string,std::size_t>& counts) {
  out<<"\n\""<<name<<"\":{";
  bool comma=false;
  for(const auto& [reason,count]:counts) {
    if(comma)out<<',';
    out<<'\"'<<reason<<"\":"<<count;comma=true;
  }
  out<<"},";
}
std::vector<Eigen::Vector2d> sample_matching_hits(
    const std::vector<Eigen::Vector2d>& points,std::size_t stride,
    std::size_t offset) {
  std::vector<Eigen::Vector2d> sampled;
  sampled.reserve((points.size()+stride-1)/stride);
  // Offset is measured among valid hits, not original raw angular bins.
  for(std::size_t i=offset%stride;i<points.size();i+=stride)
    sampled.push_back(points[i]);
  return sampled;
}
slam::matching::MatchResult match(
    std::shared_ptr<const slam::mapping::DistanceField> field,
    const std::vector<Eigen::Vector2d>& points,Pose2 seed,
    const FusionMatchOptions& options) {
  slam::matching::MatchSettings settings;
  settings.min_supported_hits=18;
  settings.minimum_overlap=.45;
  settings.maximum_iterations=30;
  settings.maximum_step_translation_m=.12;
  settings.maximum_step_yaw_rad=.075;
#ifdef SLAM_FUSION_WITH_CERES
  if(options.continuous_cost)
    return slam::matching::match_ceres_continuous(field,points,seed,settings);
  return slam::matching::match_ceres(field,points,seed,settings);
#else
  if(options.continuous_cost)
    throw std::invalid_argument("--continuous-cost requires a Ceres-enabled fusion build");
  return slam::matching::match_reference(field,points,seed,settings);
#endif
}
bool qualified(const slam::matching::MatchResult& m,
               const slam::mapping::MapSnapshot& snapshot,
               Pose2 seed, double max_delta=.32) {
  return m.status==slam::matching::MatchStatus::accepted &&
    m.diagnostics.source_fingerprint==snapshot.fingerprint() &&
    m.diagnostics.source_revision==snapshot.revision() &&
    m.diagnostics.full_rank&&m.diagnostics.supported_hits>=18&&
    m.diagnostics.overlap>=.45&&
    (m.map_from_laser.translation-seed.translation).norm()<=max_delta&&
    std::abs(slam::wrap_yaw(m.map_from_laser.yaw_rad-seed.yaw_rad))<=.16;
}
Matched match_heldout(const Simulation& sim,const Prepared& p,
                      slam::fusion::PipelineProfiler& profiler,
                      const FusionMatchOptions& options) {
  Mask even(p.gyro_wheel.size(),false);
  for(std::size_t i=0;i<even.size();i+=2)even[i]=true;
  const auto prepared=[&]{
    slam::fusion::PipelineProfiler::Scope timing(profiler,"training_map_build");
    const auto frozen=make_map(sim,p.gyro_wheel,p.gyro_deskewed,even,options);
    return std::pair{frozen,slam::mapping::DistanceField::build(
        frozen.snapshot,options.distance_truncation_m)};
  }();
  const auto& frozen=prepared.first;
  const auto& field=prepared.second;
  Matched result;
#ifdef SLAM_FUSION_WITH_CERES
  result.backend=options.continuous_cost?"ceres_continuous_cost":"ceres";
#else
  result.backend="geometry_reference_fallback";
#endif
  result.poses=p.gyro_wheel;
  result.outcomes.resize(p.gyro_wheel.size());
  result.accepted_mask.assign(p.gyro_wheel.size(),false);
  for(std::size_t i=1;i<result.poses.size();i+=2) {
    if(((i-1)/2)%options.match_every!=0){++result.skipped;continue;}
    slam::fusion::PipelineProfiler::Scope timing(profiler,"heldout_scan_match",i,true);
    const Pose2 seed=slam::compose(p.gyro_wheel[i],kBaseFromLaser);
    const auto matching_hits=sample_matching_hits(
        p.gyro_deskewed[i],options.match_stride,options.match_offset);
    result.matching_input_hits_total+=matching_hits.size();
    auto candidate=match(field,matching_hits,seed,options);
    ++result.attempted;
    if(qualified(candidate,*frozen.snapshot,seed)){
      // Convert matched laser pose back into map-from-base exactly.
      result.poses[i]=slam::compose(candidate.map_from_laser,
                                   slam::inverse(kBaseFromLaser));
      result.accepted_mask[i]=true;
      ++result.accepted;
    } else {
      ++result.rejected;
      ++result.rejections[rejection_reason(candidate.status)];
    }
    result.outcomes[i]=std::move(candidate);
  }
  return result;
}
struct LidarMatchAudit {
  std::size_t frame{}, input_beams{}, seed_supported_hits{};
  std::size_t candidate_supported_hits{}, residual_blocks{}, map_revision{};
  std::size_t solver_invalid_residuals{};
  double seed_overlap{}, candidate_translation_m{};
  bool seed_full_rank{}, candidate_evaluated{}, accepted{}, has_obstacles{};
  std::string reason,registration_method{"none"};
  bool used_scan_prediction{},edt_prediction_disagreed{};
  double predicted_edt_translation_delta_m{},predicted_edt_yaw_delta_rad{};
  std::size_t registration_forward_inliers{},registration_reverse_inliers{};
  double registration_forward_overlap{},registration_reverse_overlap{};
  double registration_error_m{},registration_translation_m{},registration_yaw_rad{};
  // All probe measurements are read-only against the historical map.
  std::size_t probe_poses{}, probe_supported_poses{}, probe_ranked_poses{};
  std::size_t probe_best_supported_hits{};
  double probe_best_overlap{};
  double probe_best_dx_m{},probe_best_dy_m{},probe_best_dyaw_rad{};
};
struct LidarOnly {
  std::vector<LidarMatchAudit> audit;
  std::vector<Pose2> poses;
  Mask mapped;
  std::size_t accepted{},rejected{},skipped{},field_rebuilds{};
  std::size_t matching_input_hits_total{};
  std::size_t bootstrap_attempted{},bootstrap_accepted{},bootstrap_rejected{};
  std::size_t edt_accepted{},edt_refined_accepted{};
  std::size_t predicted_edt_accepted{},edt_prediction_disagreements{};
  std::map<std::string,std::size_t> rejections;
};
LidarOnly lidar_only(const Simulation& sim,const Prepared& p,Pose2 first_anchor,
                     slam::fusion::PipelineProfiler& profiler,
                     const FusionMatchOptions& options) {
  LidarOnly out;
  out.poses.reserve(p.raw_lidar.size());
  out.audit.reserve(p.raw_lidar.size()-1);
  out.mapped.assign(p.raw_lidar.size(),false);
  slam::mapping::OccupancyGrid rolling(sim.grid);
  out.poses.push_back(first_anchor);out.mapped[0]=true;
  {
    slam::fusion::PipelineProfiler::Scope timing(profiler,"lidar_only_baseline",0,true);
    insert(rolling,first_anchor,p.raw_lidar[0],options.map_stride);
  }
  std::shared_ptr<const slam::mapping::MapSnapshot> cached_snapshot;
  std::shared_ptr<const slam::mapping::DistanceField> cached_field;
  std::size_t last_field_frame=0;
  std::size_t last_accepted_scan=0;
  slam::Pose2 last_relative_scan{};
  bool has_relative_scan=false;
  for(std::size_t i=1;i<p.raw_lidar.size();++i) {
    if((i-1)%options.match_every!=0) {
      // A skipped scan is neither a rejection nor a trusted insertion.
      out.poses.push_back(out.poses.back());
      ++out.skipped;
      continue;
    }
    slam::fusion::PipelineProfiler::Scope timing(profiler,"lidar_only_baseline",i,true);
    if(!cached_field||i-last_field_frame>=options.field_every) {
      cached_snapshot=slam::mapping::MapSnapshot::capture(rolling);
      cached_field=slam::mapping::DistanceField::build(
          cached_snapshot,options.distance_truncation_m);
      last_field_frame=i;
      ++out.field_rebuilds;
    }
    // All prediction data comes from accepted raw LiDAR scans alone.
    const Pose2 previous_laser=slam::compose(out.poses.back(),kBaseFromLaser);
    const auto matching_hits=sample_matching_hits(
        p.raw_lidar[i],options.match_stride,options.match_offset);
    out.matching_input_hits_total+=matching_hits.size();
    std::optional<slam::matching::ScanToScanResult> scan_prediction;
    if(options.lidar_only_predict_first) {
      const auto previous_hits=sample_matching_hits(
          p.raw_lidar[last_accepted_scan],options.match_stride,options.match_offset);
      auto predicted=slam::matching::register_consecutive_scans(
          previous_hits,matching_hits,
          has_relative_scan?last_relative_scan:Pose2{},
          !has_relative_scan||i-last_accepted_scan>1);
      if(!predicted.accepted&&has_relative_scan) {
        const auto recovery=slam::matching::register_consecutive_scans(
            previous_hits,matching_hits,Pose2{},true);
        if(recovery.accepted)predicted=recovery;
      }
      scan_prediction=predicted;
    }
    const Pose2 seed=scan_prediction&&scan_prediction->accepted?
        slam::compose(previous_laser,scan_prediction->previous_from_current):
        previous_laser;
    // With a failed LiDAR-only prediction, do not optimize around a stale
    // previous pose: its Ceres correction box is much smaller than the scan
    // displacement. A failed prediction is a rejected/unmapped scan.
    const auto candidate=(!options.lidar_only_predict_first||
                          (scan_prediction&&scan_prediction->accepted))?
        match(cached_field,matching_hits,seed,options):
        slam::matching::MatchResult{};
    LidarMatchAudit evidence;
    evidence.frame=i;
    evidence.input_beams=matching_hits.size();
    evidence.seed_supported_hits=candidate.seed_supported_hits;
    evidence.candidate_supported_hits=candidate.candidate_supported_hits;
    evidence.residual_blocks=candidate.solver_residual_blocks;
    evidence.map_revision=cached_snapshot->revision();
    evidence.solver_invalid_residuals=candidate.ceres_invalid_residual_evaluations;
    // Diagnostic fields describe the original seed, even after an
    // accepted match replaces the result's final geometry diagnostics.
    evidence.seed_overlap=candidate.seed_overlap;
    evidence.seed_full_rank=candidate.seed_full_rank;
    evidence.candidate_evaluated=candidate.candidate_evaluated;
    evidence.candidate_translation_m=candidate.candidate_translation_m;
    evidence.has_obstacles=cached_field->has_obstacles();
    // The same LiDAR registration must be auditable whether Ceres accepts
    // its predicted pose or the scan-pair fallback is applied.
    if(scan_prediction) {
      evidence.registration_forward_inliers=scan_prediction->forward_inliers;
      evidence.registration_reverse_inliers=scan_prediction->reverse_inliers;
      evidence.registration_forward_overlap=scan_prediction->forward_overlap;
      evidence.registration_reverse_overlap=scan_prediction->reverse_overlap;
      evidence.registration_error_m=scan_prediction->mean_error_m;
      evidence.registration_translation_m=scan_prediction->translation_m;
      evidence.registration_yaw_rad=scan_prediction->rotation_rad;
    }
    // Probe only scan frames 1-3 when selected by match cadence; never
    // apply a candidate, insert a ray, or use it to seed a later frame.
    // The probe never consults simulated truth, wheel or gyro estimates.
    if(options.lidar_only_support_probe&&i<=3) {
      slam::matching::MatchSettings probe_settings;
      probe_settings.min_supported_hits=18;
      probe_settings.minimum_overlap=.45;
      const auto centre=seed;
      for(int ix=-6;ix<=6;++ix)
        for(int iy=-6;iy<=6;++iy)
          for(int ia=-4;ia<=4;++ia) {
            const double dx=double(ix)*.10,dy=double(iy)*.10;
            const double da=double(ia)*.06;
            const Pose2 trial{{centre.translation.x()+dx,
                               centre.translation.y()+dy},
                              slam::wrap_yaw(centre.yaw_rad+da)};
            const auto d=slam::matching::evaluate_geometry(
                *cached_field,matching_hits,trial,probe_settings);
            ++evidence.probe_poses;
            if(d.supported_hits>=probe_settings.min_supported_hits&&
               d.overlap>=probe_settings.minimum_overlap) {
              ++evidence.probe_supported_poses;
              if(d.full_rank)++evidence.probe_ranked_poses;
            }
            // Select the most-supported probe strictly as a diagnostic,
            // with no claim this candidate is a true motion estimate.
            if(d.supported_hits>evidence.probe_best_supported_hits) {
              evidence.probe_best_supported_hits=d.supported_hits;
              evidence.probe_best_overlap=d.overlap;
              evidence.probe_best_dx_m=dx;
              evidence.probe_best_dy_m=dy;
              evidence.probe_best_dyaw_rad=da;
            }
          }
    }
    evidence.accepted=qualified(candidate,*cached_snapshot,seed,.65);
    evidence.reason=evidence.accepted?"accepted":rejection_reason(candidate.status);
    if(scan_prediction) {
      evidence.used_scan_prediction=true;
      if(!scan_prediction->accepted) {
        evidence.accepted=false;
        evidence.reason="scan_to_scan_rejected";
      } else if(evidence.accepted) {
        // EDT must agree with LiDAR-derived motion in the previous laser
        // frame; a geometrically valid *local* map correction is not
        // automatically an admissible new relative movement.
        const auto refined_relative=slam::compose(
            slam::inverse(previous_laser),candidate.map_from_laser);
        evidence.predicted_edt_translation_delta_m=(
            refined_relative.translation-
            scan_prediction->previous_from_current.translation).norm();
        evidence.predicted_edt_yaw_delta_rad=std::abs(slam::wrap_yaw(
            refined_relative.yaw_rad-
            scan_prediction->previous_from_current.yaw_rad));
        if(evidence.predicted_edt_translation_delta_m>.12||
           evidence.predicted_edt_yaw_delta_rad>.075) {
          evidence.accepted=false;
          evidence.edt_prediction_disagreed=true;
          evidence.reason="edt_scan_disagreement";
          ++out.edt_prediction_disagreements;
        }
      }
    }
    Pose2 chosen_laser=candidate.map_from_laser;
    if(evidence.accepted) {
      evidence.registration_method=scan_prediction?"edt_predicted":"edt";
      ++out.edt_accepted;
      if(scan_prediction)++out.predicted_edt_accepted;
    } else if(options.lidar_only_bootstrap) {
      ++out.bootstrap_attempted;
      // For predict-first, reuse the same independently gated registration
      // that seeded Ceres. Otherwise preserve the previous Ceres-first
      // experimental baseline exactly for A/B comparison.
      auto registered=scan_prediction.value_or(
          slam::matching::ScanToScanResult{});
      if(!scan_prediction) {
        const auto previous_scan=sample_matching_hits(
            p.raw_lidar[last_accepted_scan],options.match_stride,options.match_offset);
        registered=slam::matching::register_consecutive_scans(
            previous_scan,matching_hits,
            has_relative_scan?last_relative_scan:Pose2{},
            !has_relative_scan||i-last_accepted_scan>1);
        if(!registered.accepted&&has_relative_scan) {
          const auto recovery=slam::matching::register_consecutive_scans(
              previous_scan,matching_hits,Pose2{},true);
          if(recovery.accepted)registered=recovery;
        }
      }
      evidence.registration_forward_inliers=registered.forward_inliers;
      evidence.registration_reverse_inliers=registered.reverse_inliers;
      evidence.registration_forward_overlap=registered.forward_overlap;
      evidence.registration_reverse_overlap=registered.reverse_overlap;
      evidence.registration_error_m=registered.mean_error_m;
      evidence.registration_translation_m=registered.translation_m;
      evidence.registration_yaw_rad=registered.rotation_rad;
      if(registered.accepted) {
        const Pose2 previous_laser=slam::compose(
            out.poses.back(),kBaseFromLaser);
        const Pose2 proposed_laser=slam::compose(
            previous_laser,registered.previous_from_current);
        // Prefer strict Ceres refinement when the new laser pose has
        // independent real EDT support. A finite unknown cost alone never
        // authorizes a pose, and the registered pose remains the fallback.
        slam::matching::MatchSettings gate;
        gate.min_supported_hits=18;
        gate.minimum_overlap=.45;
        const auto support=slam::matching::evaluate_geometry(
            *cached_field,matching_hits,proposed_laser,gate);
        // Predict-first already evaluated EDT at this registered pose.
        // Do not re-run it here: that would bypass the explicit disagreement
        // gate and silently apply a correction we just rejected.
        if(!options.lidar_only_predict_first&&
           support.supported_hits>=gate.min_supported_hits&&
           support.overlap>=gate.minimum_overlap&&support.full_rank) {
          const auto refined=match(cached_field,matching_hits,proposed_laser,options);
          if(qualified(refined,*cached_snapshot,proposed_laser,.32)&&
             (refined.map_from_laser.translation-
              proposed_laser.translation).norm()<=.25&&
             std::abs(slam::wrap_yaw(refined.map_from_laser.yaw_rad-
                                     proposed_laser.yaw_rad))<=.12) {
            chosen_laser=refined.map_from_laser;
            evidence.registration_method="edt_refined";
            ++out.edt_refined_accepted;
          } else {
            chosen_laser=proposed_laser;
            evidence.registration_method="scan_to_scan";
          }
        } else {
          chosen_laser=proposed_laser;
          evidence.registration_method="scan_to_scan";
        }
        evidence.accepted=true;
        evidence.reason="accepted";
        ++out.bootstrap_accepted;
      } else {
        evidence.reason=scan_prediction?
            "scan_to_scan_rejected":evidence.reason;
        ++out.bootstrap_rejected;
      }
    }
    out.audit.push_back(evidence);
    if(evidence.accepted) {
      const Pose2 previous_laser=slam::compose(
          out.poses.back(),kBaseFromLaser);
      const Pose2 relative=slam::compose(
          slam::inverse(previous_laser),chosen_laser);
      const auto base=slam::compose(chosen_laser,
                                    slam::inverse(kBaseFromLaser));
      out.poses.push_back(base);
      out.mapped[i]=true;
      insert(rolling,base,p.raw_lidar[i],options.map_stride);
      last_relative_scan=relative;
      has_relative_scan=true;
      last_accepted_scan=i;
      ++out.accepted;
    } else {
      out.poses.push_back(out.poses.back());
      ++out.rejected;
      ++out.rejections[evidence.reason];
      // An unlocalized moving scan is never inserted into the occupancy map.
    }
  }
  return out;
}
struct GraphLoopEvent {
  std::size_t from{},to{};
  std::string decision;
};
struct GraphOutput {
  std::vector<Pose2> poses;
  std::vector<GraphLoopEvent> events;
  std::size_t accepted_loops{},loop_candidates{};
  std::string status;
};
GraphOutput optimize_graph(const Simulation& sim,const Prepared& p,
                           const Matched& matcher,bool synthetic_loops) {
  GraphOutput output;
#ifndef SLAM_FUSION_WITH_GTSAM
  (void)sim;(void)p;(void)matcher;(void)synthetic_loops;
  output.status="gtsam_not_built";return output;
#else
  slam::graph::BatchGraphInput input;
  input.initial_map_from_base=matcher.poses;
  const slam::graph::FactorProvenance sensor{
    slam::graph::EvidenceClass::synthetic_exact,
    "synthetic_raw_wheel_gyro_v1",true,true};
  for(std::size_t i=1;i<p.gyro_wheel.size();++i) {
    // Relative sensor increments were computed *before* scan-to-map
    // corrections. Never create graph factors from matcher-corrected poses.
    const auto increment=slam::compose(slam::inverse(p.gyro_wheel[i-1]),
                                       p.gyro_wheel[i]);
    input.odometry.push_back({i-1,i,increment,.06,.06,.035,sensor});
  }
  if(synthetic_loops) {
    // This deliberately *separate simulated relative-pose observation* is
    // generated only inside the synthetic fixture. It is NOT inferred from
    // the LiDAR match or assumed to exist on a real LD19/MPU6050 robot.
    const slam::graph::FactorProvenance loop_source{
      slam::graph::EvidenceClass::synthetic_exact,
      "independent_simulated_loop_sensor_v1",true,true};
    for(const auto& measurement:sim.independent_loop_measurements){
      const auto i=measurement.to,j=measurement.from;
      if(i>=matcher.outcomes.size()||i<=j+15||
         !matcher.accepted_mask[i])
        continue;
      // Existing geometry diagnostics and source fingerprint of the fixed
      // training map must corroborate independently sourced loop proposals.
      const auto& result=matcher.outcomes[i];
      if(result.status!=slam::matching::MatchStatus::accepted||
         result.diagnostics.supported_hits<25||
         result.diagnostics.overlap<.65||!result.diagnostics.full_rank)
        continue;
      // Do not construct a loop constraint from a map-match output; use
      // the independent sensor's raw, bounded relative observation.
      input.loops.push_back({{j,i,measurement.measured_from_to,
        .04,.04,.025,loop_source},
        result.diagnostics.supported_hits,result.diagnostics.overlap,true});
    }
  }
  output.loop_candidates=input.loops.size();
  auto graph=slam::graph::optimize_pose_graph(input);
  if(graph.loop_decisions.size()!=input.loops.size())
    throw std::runtime_error("GTSAM loop decision cardinality mismatch");
  for(std::size_t k=0;k<input.loops.size();++k) {
    const auto decision=graph.loop_decisions[k];
    std::string label;
    switch(decision){
      case slam::graph::LoopDecision::accepted: label="accepted";break;
      case slam::graph::LoopDecision::insufficient_geometry:label="insufficient_geometry";break;
      case slam::graph::LoopDecision::unverified_or_correlated_evidence:label="unverified_evidence";break;
      case slam::graph::LoopDecision::inconsistent_with_initial_path:label="inconsistent";break;
      case slam::graph::LoopDecision::duplicate_or_nearby_nodes:label="duplicate_or_nearby";break;
    }
    output.events.push_back({input.loops[k].edge.from,input.loops[k].edge.to,label});
  }
  if(!graph.fixed_first_pose)throw std::runtime_error("graph lost gauge anchor");
  output.poses=std::move(graph.optimized_map_from_base);
  output.accepted_loops=graph.accepted_loops;
  output.status=synthetic_loops?"gtsam_with_simulated_independent_loops":
                                "gtsam_odometry_only_no_verified_loops";
  return output;
#endif
}
struct TrajectoryError {
  double translation_rmse_m{},translation_p95_m{},yaw_rmse_rad{};
};
TrajectoryError error(const std::vector<Pose2>& estimate,
                      const std::vector<Pose2>& reference) {
  if(estimate.size()!=reference.size()||estimate.empty())
    throw std::invalid_argument("invalid trajectory truth comparison");
  double sum=0,yaw=0;std::vector<double> distances;
  distances.reserve(estimate.size());
  for(std::size_t i=0;i<estimate.size();++i) {
    const double dx=(estimate[i].translation-reference[i].translation).norm();
    const double dy=slam::wrap_yaw(estimate[i].yaw_rad-reference[i].yaw_rad);
    distances.push_back(dx);sum+=dx*dx;yaw+=dy*dy;
  }
  const std::size_t rank=static_cast<std::size_t>(std::ceil(.95*distances.size()))-1;
  std::nth_element(distances.begin(),distances.begin()+rank,distances.end());
  return {std::sqrt(sum/estimate.size()),distances[rank],
          std::sqrt(yaw/estimate.size())};
}
void export_bundle(const std::filesystem::path& p,
                   const slam::mapping::MapBundle& b) {
  std::ofstream out(p,std::ios::binary|std::ios::trunc);
  if(!out)throw std::runtime_error("failed opening FUSMAP2D output");
  slam::mapping::write_map_bundle(out,b);
  if(!out)throw std::runtime_error("failed writing FUSMAP2D output");
}
void item(std::ostream& out,const char* name,const Simulation& sim,
          const slam::mapping::MapBundle& b,const TrajectoryError& err,
          bool comma=true) {
  out<<"\""<<name<<"\":{"
     <<"\"translation_rmse_m\":"<<err.translation_rmse_m
     <<",\"translation_p95_m\":"<<err.translation_p95_m
     <<",\"yaw_rmse_rad\":"<<err.yaw_rmse_rad
     <<",\"map\":";
  auto score=slam::evaluation::evaluate_map_accuracy(b,sim.scene);
  slam::evaluation::write_map_accuracy_json(out,score,sim.scene);
  out<<"}"<<(comma?",\n":"\n");
}
void write_path(std::ostream& out,const char* name,
                const std::vector<Pose2>& poses,bool comma=true) {
  out<<"\""<<name<<"\":[";
  for(std::size_t i=0;i<poses.size();++i){
    if(i)out<<",";
    const auto& p=poses[i];
    out<<"["<<p.translation.x()<<","<<p.translation.y()<<","<<p.yaw_rad<<"]";
  }
  out<<"]"<<(comma?",\n":"\n");
}
void export_trajectories(const std::filesystem::path& output,
                         const Simulation& sim,const Prepared& p,
                         const LidarOnly& lidar,const Matched& matched,
                         const GraphOutput& graph) {
  std::ofstream out(output);
  if(!out)throw std::runtime_error("failed opening trajectory export");
  out<<std::setprecision(17);
  out<<"{\n\"format\":\"FUSPOSES1\","
     <<"\n\"frame_id\":\"map\","
     <<"\n\"clock_domain\":101,"
     <<"\n\"clock_epoch\":\"1\","
     <<"\n\"angle_units\":\"radians\","
     <<"\n\"length_units\":\"metres\","
     <<"\n\"reference_class\":\"synthetic_exact\","
     <<"\n\"timestamps_ns\":[";
  for(std::size_t i=0;i<sim.sensors.scans.size();++i){
    if(i)out<<",";
    // JSON string prevents nanosecond quantization in JavaScript viewers.
    out<<"\""<<sim.sensors.scans[i].midpoint.ns<<"\"";
  }
  out<<"],\n\"paths\":{\n";
  write_path(out,"wheel_only",p.wheel_only);
  write_path(out,"lidar_only",lidar.poses);
  write_path(out,"gyro_wheel",p.gyro_wheel);
  write_path(out,"fused_scan_matched",matched.poses,!graph.poses.empty());
  if(!graph.poses.empty())write_path(out,"graph_optimized",graph.poses,false);
  out<<"},\n\"synthetic_truth_for_scoring\":";
  out<<"[";
  for(std::size_t i=0;i<sim.reference.size();++i){
    if(i)out<<",";
    const auto& truth=sim.reference[i];
    out<<"["<<truth.translation.x()<<","<<truth.translation.y()
       <<","<<truth.yaw_rad<<"]";
  }
  out<<"]\n}\n";
  if(!out)throw std::runtime_error("failed writing trajectory JSON");
}
// A stand-alone diagnostic for the incremental LiDAR-only baseline.
// Crucially, it includes rejected frames. Neither reference trajectory nor
// simulation truth is fed into this diagnostic or its optional support probe.
void export_lidar_match_audit(const std::filesystem::path& output,
                              const LidarOnly& lidar,
                              const FusionMatchOptions& options) {
  std::ofstream out(output);
  if(!out)throw std::runtime_error("failed opening LiDAR-only matcher audit");
  out<<std::setprecision(15)
     <<"{\"format\":\"FUSLIDARAUDIT1\","
     <<"\"ground_truth_used\":false,"
     <<"\"support_probe_read_only\":"
     <<(options.lidar_only_support_probe?"true":"false")
     <<",\"lidar_only_bootstrap\":"
     <<(options.lidar_only_bootstrap?"true":"false")
     <<",\"lidar_only_predict_first\":"
     <<(options.lidar_only_predict_first?"true":"false")
     <<",\"match_stride\":"<<options.match_stride
     <<",\"match_offset\":"<<options.match_offset
     <<",\"distance_truncation_m\":"<<options.distance_truncation_m
     <<",\"attempts\":"<<(lidar.accepted+lidar.rejected)
     <<",\"frames\":[";
  for(std::size_t j=0;j<lidar.audit.size();++j) {
    if(j)out<<',';
    const auto& a=lidar.audit[j];
    out<<"{\"index\":"<<a.frame
       <<",\"status\":\""<<a.reason<<"\""
       <<",\"accepted\":"<<(a.accepted?"true":"false")
       <<",\"input_beams\":"<<a.input_beams
       <<",\"seed_supported_hits\":"<<a.seed_supported_hits
       <<",\"seed_overlap\":"<<a.seed_overlap
       <<",\"seed_full_rank\":"<<(a.seed_full_rank?"true":"false")
       <<",\"map_has_obstacles\":"<<(a.has_obstacles?"true":"false")
       <<",\"map_revision\":"<<a.map_revision
       <<",\"solver_residual_blocks\":"<<a.residual_blocks
       <<",\"candidate_evaluated\":"<<(a.candidate_evaluated?"true":"false")
       <<",\"candidate_supported_hits\":"<<a.candidate_supported_hits
       <<",\"candidate_translation_m\":"<<a.candidate_translation_m
       <<",\"solver_invalid_residual_evaluations\":"<<a.solver_invalid_residuals
       <<",\"registration_method\":\""<<a.registration_method<<"\""
       <<",\"used_scan_prediction\":"<<(a.used_scan_prediction?"true":"false")
       <<",\"edt_prediction_disagreed\":"<<(a.edt_prediction_disagreed?"true":"false")
       <<",\"predicted_edt_translation_delta_m\":"<<a.predicted_edt_translation_delta_m
       <<",\"predicted_edt_yaw_delta_rad\":"<<a.predicted_edt_yaw_delta_rad
       <<",\"registration_forward_inliers\":"<<a.registration_forward_inliers
       <<",\"registration_reverse_inliers\":"<<a.registration_reverse_inliers
       <<",\"registration_forward_overlap\":"<<a.registration_forward_overlap
       <<",\"registration_reverse_overlap\":"<<a.registration_reverse_overlap
       <<",\"registration_error_m\":"<<a.registration_error_m
       <<",\"registration_translation_m\":"<<a.registration_translation_m
       <<",\"registration_yaw_rad\":"<<a.registration_yaw_rad
       <<",\"probe_poses\":"<<a.probe_poses
       <<",\"probe_supported_poses\":"<<a.probe_supported_poses
       <<",\"probe_ranked_poses\":"<<a.probe_ranked_poses
       <<",\"probe_best_supported_hits\":"<<a.probe_best_supported_hits
       <<",\"probe_best_overlap\":"<<a.probe_best_overlap
       <<",\"probe_best_offset\":["
       <<a.probe_best_dx_m<<','<<a.probe_best_dy_m<<','
       <<a.probe_best_dyaw_rad<<"]}";
  }
  out<<"]}\n";
  if(!out)throw std::runtime_error("failed writing LiDAR-only matcher audit");
}
// FUSFUSREPLAY1 contains *actual* events from this run, not browser-side
// synthesized matcher/graph outcomes. No raw oracle trajectory is included.
void export_replay(const std::filesystem::path& output,const Simulation& sim,
                   const Prepared& p,const LidarOnly& lidar,
                   const Matched& matched,const GraphOutput& graph,
                   const FusionMatchOptions& options) {
  std::ofstream out(output);
  if(!out)throw std::runtime_error("failed opening fusion replay");
  out<<std::setprecision(17);
  out<<"{\"format\":\"FUSFUSREPLAY1\",\"scene\":\""<<sim.scene.id
     <<"\",\"reference_class\":\"synthetic_exact\","
     <<"\"source\":\"moving_synthetic_lidar_gyro_wheel\","
     <<"\"matcher\":\""<<matched.backend<<"\","
     <<"\"match_every\":"<<options.match_every<<","
     <<"\"field_every\":"<<options.field_every<<","
     <<"\"map_stride\":"<<options.map_stride<<","
     <<"\"match_stride\":"<<options.match_stride<<","
     <<"\"match_offset\":"<<options.match_offset<<","
     <<"\"distance_truncation_m\":"<<options.distance_truncation_m<<","
     <<"\"scan_frame\":\"laser_midpoint_deskewed_full_se2\","
     <<"\"laser_from_base_translation_m\":["
     <<kBaseFromLaser.translation.x()<<","<<kBaseFromLaser.translation.y()<<"],"
     <<"\"base_from_laser_yaw_rad\":"<<kBaseFromLaser.yaw_rad<<","
     <<"\"frame_count\":"<<sim.sensors.scans.size()<<",\"frames\":[\n";
  std::size_t accepted=0,rejected=0,accepted_loops=0;
  for(std::size_t i=0;i<sim.sensors.scans.size();++i) {
    if(i)out<<",\n";
    const auto& scan=sim.sensors.scans[i];
    const bool query=i%2==1;
    const bool attempted=query&&(((i-1)/2)%options.match_every==0);
    const auto& result=matched.outcomes[i];
    const bool approved=attempted&&matched.accepted_mask[i];
    if(attempted){if(approved)++accepted;else ++rejected;}
    const char* status="training";
    std::string reason="not_evaluated_training_scan";
    if(query&&!attempted) {
      status="skipped";
      reason="match_cadence_skip";
    }else if(attempted) {
      status=approved?"accepted":"rejected";
      reason=approved?"accepted":rejection_reason(result.status);
    }
    out<<"{\"index\":"<<i<<",\"time_ns\":\""<<scan.midpoint.ns<<"\","
       <<"\"hits\":"<<scan.hit_count<<",\"beam_count\":"<<scan.beams.size()
       <<",\"points_laser\":[";
    // Bounded rendering sample; actual full-resolution mapping and matching
    // happened before this export and are not affected by its decimation.
    const auto& hits=p.gyro_deskewed[i];
    for(std::size_t k=0;k<hits.size();k+=2){
      if(k)out<<",";
      out<<"["<<hits[k].x()<<","<<hits[k].y()<<"]";
    }
    out<<"],\"matching\":{\"attempted\":"<<(attempted?"true":"false")
       <<",\"accepted\":"<<(approved?"true":"false")
       <<",\"status\":\""<<status<<"\",\"reason\":\""<<reason<<"\""
       <<",\"cumulative_accepted\":"<<accepted
       <<",\"cumulative_rejected\":"<<rejected;
    if(attempted){
      const auto& d=result.diagnostics;
      out<<",\"supported_hits\":"<<d.supported_hits
         <<",\"input_hits\":"<<d.input_hits
         <<",\"solver_residual_blocks\":"<<result.solver_residual_blocks
         <<",\"overlap\":"<<(std::isfinite(d.overlap)?d.overlap:0)
         <<",\"full_rank\":"<<(d.full_rank?"true":"false")
         <<",\"training_map_fingerprint\":\""<<d.source_fingerprint<<"\"";
      if(std::isfinite(d.mean_distance_metres))
        out<<",\"mean_distance_m\":"<<d.mean_distance_metres;
      if(std::isfinite(d.robust_cost))
        out<<",\"robust_cost\":"<<d.robust_cost;
    }
    out<<"},\"mapping\":{\"lidar_only_inserted\":"
       <<(lidar.mapped[i]?"true":"false")
       <<",\"gyro_wheel_inserted\":true,\"fused_scan_matched_inserted\":true}"
       <<",\"loop_events\":[";
    bool first=true;
    for(const auto& event:graph.events)if(event.to==i){
      if(!first)out<<',';
      first=false;
      if(event.decision=="accepted")++accepted_loops;
      out<<"{\"from\":"<<event.from<<",\"to\":"<<event.to
         <<",\"decision\":\""<<event.decision<<"\"}";
    }
    out<<"],\"cumulative_accepted_loops\":"<<accepted_loops<<"}";
  }
  out<<"]}\n";
  if(!out)throw std::runtime_error("failed writing fusion replay");
}
void execute(const Simulation& sim,const std::filesystem::path& directory,
             bool synthetic_loops,slam::fusion::PipelineProfiler& profiler,
             const FusionMatchOptions& options) {
  if(std::filesystem::exists(directory))
    throw std::invalid_argument("output already exists; refuse to overwrite");
  if(!std::filesystem::create_directories(directory))
    throw std::runtime_error("cannot create exclusive output directory");
  const auto p=prepare(sim.sensors,sim.initial_anchor,profiler);
  const auto matched=match_heldout(sim,p,profiler,options);
  const auto lidar=lidar_only(sim,p,p.gyro_wheel.front(),profiler,options);
  const auto graph=[&]{
    slam::fusion::PipelineProfiler::Scope timing(profiler,"graph_optimize");
    return optimize_graph(sim,p,matched,synthetic_loops);
  }();
  const auto wheel_map=[&]{
    slam::fusion::PipelineProfiler::Scope timing(profiler,"wheel_map_build");
    return make_map(sim,p.wheel_only,p.wheel_deskewed,all(p.wheel_only.size()),options);
  }();
  const auto lidar_map=[&]{
    slam::fusion::PipelineProfiler::Scope timing(profiler,"lidar_map_build");
    return make_map(sim,lidar.poses,p.raw_lidar,lidar.mapped,options);
  }();
  const auto fused_map=[&]{
    slam::fusion::PipelineProfiler::Scope timing(profiler,"gyro_map_build");
    return make_map(sim,p.gyro_wheel,p.gyro_deskewed,all(p.gyro_wheel.size()),options);
  }();
  const auto matched_map=[&]{
    slam::fusion::PipelineProfiler::Scope timing(profiler,"fused_map_build");
    return make_map(sim,matched.poses,p.gyro_deskewed,all(matched.poses.size()),options,&profiler);
  }();
  export_bundle(directory/"wheel_only.slammap",wheel_map);
  export_bundle(directory/"lidar_only.slammap",lidar_map);
  export_bundle(directory/"gyro_wheel.slammap",fused_map);
  export_bundle(directory/"fused_scan_matched.slammap",matched_map);
  export_trajectories(directory/"trajectories.json",sim,p,lidar,matched,graph);
  export_replay(directory/"fusion_replay.json",sim,p,lidar,matched,graph,options);
  export_lidar_match_audit(directory/"lidar_only_match_audit.json",lidar,options);
  std::optional<slam::mapping::MapBundle> graph_map;
  if(!graph.poses.empty()) {
    {
      slam::fusion::PipelineProfiler::Scope timing(profiler,"graph_map_build");
      graph_map=make_map(sim,graph.poses,p.gyro_deskewed,all(graph.poses.size()),options);
    }
    export_bundle(directory/"graph_optimized.slammap",*graph_map);
  }
  std::ofstream out(directory/"fusion_report.json");
  if(!out)throw std::runtime_error("failed opening fusion report");
  out<<std::setprecision(14);
  out<<"{\n\"format\":\"FUSFUSION1\","
     <<"\n\"scene\":\""<<sim.scene.id<<"\","
     <<"\n\"truth_class\":\"synthetic_exact\","
     <<"\n\"source\":\"moving_synthetic_lidar_gyro_wheel\","
     <<"\n\"map_pose_truth_used\":false,"
     <<"\n\"physical_gyro_calibration_verified\":false,"
     <<"\n\"sensor_clock\":\"synthetic_exact_single_epoch\","
     <<"\n\"scan_beams\":240,"
     <<"\n\"scan_period_ns\":"<<kScanNs<<","
     <<"\n\"scan_duration_ns\":100000000,"
     <<"\n\"sensor_tick_ns\":"<<kTickNs<<","
     <<"\n\"frames\":"<<sim.reference.size()<<","
     <<"\n\"gyro_bias_estimate_rad_s\":"<<p.integrated.estimated_gyro_bias_rad_s<<","
     <<"\n\"stationary_calibration_ticks\":"<<p.integrated.stationary_ticks<<","
     <<"\n\"full_se2_deskew_verified\":true,"
     <<"\n\"matcher\":\""<<matched.backend<<"\","
     <<"\n\"match_every\":"<<options.match_every<<","
     <<"\n\"field_every\":"<<options.field_every<<","
     <<"\n\"map_stride\":"<<options.map_stride<<","
     <<"\n\"match_stride\":"<<options.match_stride<<","
     <<"\n\"match_offset\":"<<options.match_offset<<","
     <<"\n\"distance_truncation_m\":"<<options.distance_truncation_m<<","
     <<"\n\"heldout_skipped\":"<<matched.skipped<<","
     <<"\n\"heldout_matching_input_hits_total\":"<<matched.matching_input_hits_total<<","
     <<"\n\"lidar_only_matching_input_hits_total\":"<<lidar.matching_input_hits_total<<","
     <<"\n\"heldout_attempted\":"<<matched.attempted<<","
     <<"\n\"heldout_accepted\":"<<matched.accepted<<","
     <<"\n\"heldout_rejected\":"<<matched.rejected<<","
     <<"\n\"lidar_only_accepted\":"<<lidar.accepted<<","
     <<"\n\"lidar_only_rejected\":"<<lidar.rejected<<","
     <<"\n\"lidar_only_skipped\":"<<lidar.skipped<<","
     <<"\n\"lidar_only_field_rebuilds\":"<<lidar.field_rebuilds<<","
     <<"\n\"lidar_only_support_probe\":"<<(options.lidar_only_support_probe?"true":"false")<<","
     <<"\n\"lidar_only_bootstrap\":"<<(options.lidar_only_bootstrap?"true":"false")<<","
     <<"\n\"lidar_only_predict_first\":"<<(options.lidar_only_predict_first?"true":"false")<<","
     <<"\n\"lidar_only_predicted_edt_accepted\":"<<lidar.predicted_edt_accepted<<","
     <<"\n\"lidar_only_edt_prediction_disagreements\":"<<lidar.edt_prediction_disagreements<<","
     <<"\n\"lidar_only_bootstrap_attempted\":"<<lidar.bootstrap_attempted<<","
     <<"\n\"lidar_only_bootstrap_accepted\":"<<lidar.bootstrap_accepted<<","
     <<"\n\"lidar_only_bootstrap_rejected\":"<<lidar.bootstrap_rejected<<","
     <<"\n\"lidar_only_edt_accepted\":"<<lidar.edt_accepted<<","
     <<"\n\"lidar_only_edt_refined_accepted\":"<<lidar.edt_refined_accepted<<","
     <<"\n\"lidar_only_before_solver_insufficient_support\":"
     <<std::count_if(lidar.audit.begin(),lidar.audit.end(),
         [](const LidarMatchAudit& a){
           return a.reason=="insufficient_support"&&!a.candidate_evaluated&&
                  a.residual_blocks==0;
         })<<","
     <<"\n\"lidar_only_after_solver_insufficient_support\":"
     <<std::count_if(lidar.audit.begin(),lidar.audit.end(),
         [](const LidarMatchAudit& a){
           return a.reason=="insufficient_support"&&a.candidate_evaluated;
         })<<",";
  write_rejections(out,"heldout_rejection_reasons",matched.rejections);
  write_rejections(out,"lidar_only_rejection_reasons",lidar.rejections);
  out<<"\n\"graph_status\":\""<<graph.status<<"\","
     <<"\n\"loop_measurements\":\""<<(synthetic_loops?
          "independently_simulated_relative_pose":"none")<<"\","
     <<"\n\"graph_loop_candidates\":"<<graph.loop_candidates<<","
     <<"\n\"graph_accepted_loops\":"<<graph.accepted_loops<<","
     <<"\n\"results\":{\n";
  item(out,"wheel_only",sim,wheel_map,error(p.wheel_only,sim.reference));
  item(out,"lidar_only",sim,lidar_map,error(lidar.poses,sim.reference));
  item(out,"gyro_wheel",sim,fused_map,error(p.gyro_wheel,sim.reference));
  item(out,"fused_scan_matched",sim,matched_map,error(matched.poses,sim.reference),
       graph_map.has_value());
  if(graph_map)
    item(out,"graph_optimized",sim,*graph_map,error(graph.poses,sim.reference),false);
  out<<"}\n}\n";
  if(!out)throw std::runtime_error("failed writing fusion report");
  out.close();
  {
    std::ofstream profile(directory/"fusion_profile.json");
    if(!profile)throw std::runtime_error("failed opening fusion profiler output");
    profiler.write_json(profile,sim.reference.size(),kScanNs*1e-6,
                        matched.backend,sim.scene.id);
    if(!profile)throw std::runtime_error("failed writing fusion profiler output");
  }
  std::cout<<"scene="<<sim.scene.id
           <<" frames="<<sim.reference.size()
           <<" matcher="<<matched.backend
           <<" accepted="<<matched.accepted<<"/"<<matched.attempted
           <<" graph="<<graph.status
           <<" loops="<<graph.accepted_loops
           <<"\nreport="<<(directory/"fusion_report.json").string()<<'\n';
}
} // namespace

int main(int argc,char** argv) {
  cxxopts::Options cli("slam_offline_fusion",
      "Offline synthetic LiDAR/odometry fusion and optional independent loops");
  cli.add_options()
      ("scene","Synthetic scene",cxxopts::value<std::string>())
      ("output","New output directory",cxxopts::value<std::string>())
      ("synthetic-independent-loops","Enable synthetic independent loop constraints")
      ("match-every","Match every N eligible scans (default 1)",cxxopts::value<std::string>())
      ("field-every","Refresh incremental LiDAR-only EDT every N frames (default 1)",cxxopts::value<std::string>())
      ("map-stride","Insert every Nth valid LiDAR hit into all maps (default 1)",cxxopts::value<std::string>())
      ("match-stride","Match every Nth valid LiDAR hit (default 1)",cxxopts::value<std::string>())
      ("match-offset","Valid-hit subsampling offset 0..63 (default 0)",cxxopts::value<std::string>())
      ("distance-truncation","EDT cutoff in metres (default 0.7)",cxxopts::value<std::string>())
      ("continuous-cost","Use experimental finite-cost Ceres for both matchers")
      ("lidar-only-support-probe","Read-only local support search on first 3 LiDAR-only attempts")
      ("lidar-only-bootstrap","Opt-in LiDAR scan-pair registration and strict EDT handover")
      ("lidar-only-predict-first","LiDAR-only scan motion seeds Ceres before map registration (requires bootstrap)")
      ("h,help","Show help");
  cli.parse_positional({"scene","output"});
  cli.positional_help("<apartment|warehouse|office|loft|atrium|industrial|warehouse_asymmetric> <new-output-directory>");
  cxxopts::ParseResult args;
  try { args=cli.parse(argc,argv); }
  catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
  if(args.count("help")&&args["help"].as<bool>()){std::cout<<cli.help();return 0;}
  if(!args.count("scene")||!args.count("output")){std::cerr<<cli.help();return 2;}

  try{
    const bool loops=args.count("synthetic-independent-loops")&&
        args["synthetic-independent-loops"].as<bool>();
    FusionMatchOptions options;
    const auto positive=[&](const char* name,std::size_t fallback,std::size_t max) {
      return args.count(name)?posint(args[name].as<std::string>(),max):fallback;
    };
    options.match_every=positive("match-every",options.match_every,200);
    options.field_every=positive("field-every",options.field_every,200);
    options.map_stride=positive("map-stride",options.map_stride,64);
    options.match_stride=positive("match-stride",options.match_stride,64);
    if(args.count("match-offset")) {
      const auto offset=args["match-offset"].as<std::string>();
      if(offset=="0")options.match_offset=0;
      else options.match_offset=posint(offset,63);
    }
    options.continuous_cost=args.count("continuous-cost")&&
        args["continuous-cost"].as<bool>();
    options.lidar_only_support_probe=args.count("lidar-only-support-probe")&&
        args["lidar-only-support-probe"].as<bool>();
    options.lidar_only_bootstrap=args.count("lidar-only-bootstrap")&&
        args["lidar-only-bootstrap"].as<bool>();
    options.lidar_only_predict_first=args.count("lidar-only-predict-first")&&
        args["lidar-only-predict-first"].as<bool>();
    if(options.lidar_only_predict_first&&!options.lidar_only_bootstrap)
      throw std::invalid_argument(
          "--lidar-only-predict-first requires --lidar-only-bootstrap");
    if(args.count("distance-truncation")) {
      const auto value=args["distance-truncation"].as<std::string>();
      std::size_t parsed=0;
      const double cutoff=std::stod(value,&parsed);
      if(parsed!=value.size()||!std::isfinite(cutoff)||cutoff<0.2||cutoff>3.0)
        throw std::invalid_argument("distance-truncation must be finite within [0.2,3.0]");
      options.distance_truncation_m=cutoff;
    }
#ifndef SLAM_FUSION_WITH_CERES
    if(options.continuous_cost)throw std::invalid_argument(
        "--continuous-cost requires a Ceres-enabled fusion build");
#endif
#ifndef SLAM_FUSION_WITH_GTSAM
    if(loops)throw std::invalid_argument(
      "independent synthetic loop demonstration requires a GTSAM-enabled build");
#endif
    slam::fusion::PipelineProfiler profiler;
    const auto simulation=[&]{
      slam::fusion::PipelineProfiler::Scope timing(profiler,"synthetic_generation");
      return generate(args["scene"].as<std::string>());
    }();
    execute(simulation,args["output"].as<std::string>(),loops,profiler,options);
    return 0;
  }catch(const std::exception& e){
    std::cerr<<"offline fusion failed: "<<e.what()<<'\n';
    return 1;
  }
}
