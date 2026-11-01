#include "slam/evaluation/map_accuracy.hpp"
#include "slam/fixtures/synthetic.hpp"
#include "slam/fusion/gyro_wheel.hpp"
#include "slam/graph/pose_graph.hpp"
#include "slam/mapping/map_bundle.hpp"
#include "slam/matching/geometry.hpp"
#include "slam/motion/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
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
const Pose2 kBaseFromLaser{{.12,.01},.02};

// Truth is used in the synthetic sensor GENERATOR and in independent scoring,
// NEVER in the sensor-fusion, scan-matching or map-building functions.
struct Observations {
  std::vector<slam::fusion::SensorTick> ticks;
  std::vector<slam::sensors::AssembledScan> scans;
};
struct Simulation {
  slam::fixtures::Scene scene;
  slam::mapping::GridConfig grid;
  Observations sensors;
  std::vector<Pose2> reference; // post-hoc accuracy and explicit synthetic loop sensor only
  Pose2 initial_anchor;
  bool warehouse{};
};
struct Motion {
  Pose2 pose;
  double forward{}, yaw_rate{};
};
Motion at_time(bool warehouse,double seconds_after_start) {
  const double active=std::max(0.0,seconds_after_start-.30);
  const double period=warehouse?56.0:20.0;
  const double rx=warehouse?13.0:1.35,ry=warehouse?8.0:1.35;
  const double cx=warehouse?15.0:2.5,cy=warehouse?10.0:3.0;
  const double w=2*kPi/period,phase=w*active;
  const double s=std::sin(phase),c=std::cos(phase);
  const double length=std::hypot(rx*s,ry*c);
  Motion m;
  m.pose={{cx+rx*c,cy+ry*s},std::atan2(ry*c,-rx*s)};
  m.forward=seconds_after_start>=.30?w*length:0.0;
  m.yaw_rate=seconds_after_start>=.30?
    w*rx*ry/(rx*rx*s*s+ry*ry*c*c):0.0;
  return m;
}
Simulation generate(const std::string& label) {
  if(label!="apartment"&&label!="warehouse")
    throw std::invalid_argument("expected apartment or warehouse");
  Simulation d;
  d.warehouse=label=="warehouse";
  d.scene=d.warehouse?slam::fixtures::warehouse():slam::fixtures::apartment();
  d.grid.width=d.warehouse?621:321;
  d.grid.height=d.warehouse?421:261;
  d.grid.resolution_m=.05;
  d.grid.map_from_grid={{-.5,-.5},0};
  d.initial_anchor=at_time(d.warehouse,0).pose;
  const std::size_t scans=d.warehouse?195:130;
  const std::size_t ticks=12*(scans+4)+32;
  d.sensors.ticks.reserve(ticks);
  for(std::size_t i=0;i<ticks;++i) {
    const double t=double(i)*.025;
    const Motion exact=at_time(d.warehouse,t);
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
    const Motion exact=at_time(d.warehouse,seconds_after_start);
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
  return d;
}
using Points=std::vector<std::vector<Eigen::Vector2d>>;
using Mask=std::vector<bool>;
struct Prepared {
  std::vector<Pose2> gyro_wheel,wheel_only;
  Points gyro_deskewed,wheel_deskewed,raw_lidar;
  slam::fusion::FusionResult integrated;
};
Prepared prepare(const Observations& obs,Pose2 initial) {
  slam::fusion::FusionOptions options;
  options.minimum_stationary_ticks=12;
  options.maximum_interval_ns=50'000'000LL;
  Prepared p;
  // The synthetic fixture's shared clock domain, epoch and base/laser frame
  // are explicitly known. This is NOT a real-hardware calibration claim.
  p.integrated=slam::fusion::integrate_sensors(obs.ticks,initial,true,options);
  if(p.integrated.capability!=slam::motion::TrajectoryCapability::full_se2)
    throw std::runtime_error("full wheel SE2 required for moving-scan deskew");
  slam::motion::SampledTrajectory fused_track(
      p.integrated.fused,p.integrated.capability);
  slam::motion::SampledTrajectory wheel_track(
      p.integrated.wheel_only,p.integrated.capability);
  for(const auto& scan:obs.scans) {
    if(scan.time_quality!=slam::sensors::BeamTimeQuality::synthetic_exact ||
       scan.source_id!=16||scan.calibration_revision!=1)
      throw std::invalid_argument("unverified synthetic scan metadata");
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
            const std::vector<Eigen::Vector2d>& hits) {
  const auto laser=slam::compose(base,kBaseFromLaser);
  std::vector<slam::mapping::WorldRay> rays;
  rays.reserve(hits.size());
  for(const auto& pt:hits)
    rays.push_back({laser.translation,slam::transform(laser,pt),
                    slam::sensors::ReturnKind::hit});
  (void)grid.integrate_rays(rays);
}
slam::mapping::MapBundle make_map(const Simulation& sim,
                                  const std::vector<Pose2>& poses,
                                  const Points& scans,
                                  const Mask& use) {
  if(poses.size()!=scans.size()||poses.size()!=use.size())
    throw std::invalid_argument("map pose/ray size mismatch");
  slam::mapping::OccupancyGrid grid(sim.grid);
  for(std::size_t i=0;i<poses.size();++i)if(use[i])
    insert(grid,poses[i],scans[i]);
  return {slam::mapping::MapSnapshot::capture(grid),
    {"map",sim.scene.id,"synthetic_estimated",true}};
}
Mask all(std::size_t n) {return Mask(n,true);}
struct Matched {
  std::vector<Pose2> poses;
  std::vector<slam::matching::MatchResult> outcomes;
  std::size_t attempted{},accepted{},rejected{};
  std::string backend;
};
slam::matching::MatchResult match(
    std::shared_ptr<const slam::mapping::DistanceField> field,
    const std::vector<Eigen::Vector2d>& points,Pose2 seed) {
  slam::matching::MatchSettings settings;
  settings.min_supported_hits=18;
  settings.minimum_overlap=.45;
  settings.maximum_iterations=30;
  settings.maximum_step_translation_m=.12;
  settings.maximum_step_yaw_rad=.075;
#ifdef SLAM_FUSION_WITH_CERES
  return slam::matching::match_ceres(field,points,seed,settings);
#else
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
Matched match_heldout(const Simulation& sim,const Prepared& p) {
  Mask even(p.gyro_wheel.size(),false);
  for(std::size_t i=0;i<even.size();i+=2)even[i]=true;
  const auto frozen=make_map(sim,p.gyro_wheel,p.gyro_deskewed,even);
  const auto field=slam::mapping::DistanceField::build(frozen.snapshot,.7);
  Matched result;
#ifdef SLAM_FUSION_WITH_CERES
  result.backend="ceres";
#else
  result.backend="geometry_reference_fallback";
#endif
  result.poses=p.gyro_wheel;
  result.outcomes.resize(p.gyro_wheel.size());
  for(std::size_t i=1;i<result.poses.size();i+=2) {
    const Pose2 seed=slam::compose(p.gyro_wheel[i],kBaseFromLaser);
    auto candidate=match(field,p.gyro_deskewed[i],seed);
    ++result.attempted;
    if(qualified(candidate,*frozen.snapshot,seed)){
      // Convert matched laser pose back into map-from-base exactly.
      result.poses[i]=slam::compose(candidate.map_from_laser,
                                   slam::inverse(kBaseFromLaser));
      ++result.accepted;
    } else ++result.rejected;
    result.outcomes[i]=std::move(candidate);
  }
  return result;
}
struct LidarOnly {
  std::vector<Pose2> poses;
  Mask mapped;
  std::size_t accepted{},rejected{};
};
LidarOnly lidar_only(const Simulation& sim,const Prepared& p,Pose2 first_anchor) {
  LidarOnly out;
  out.poses.reserve(p.raw_lidar.size());
  out.mapped.assign(p.raw_lidar.size(),false);
  slam::mapping::OccupancyGrid rolling(sim.grid);
  out.poses.push_back(first_anchor);out.mapped[0]=true;
  insert(rolling,first_anchor,p.raw_lidar[0]);
  for(std::size_t i=1;i<p.raw_lidar.size();++i) {
    auto snapshot=slam::mapping::MapSnapshot::capture(rolling);
    const auto field=slam::mapping::DistanceField::build(snapshot,.7);
    // No wheel, gyro or ground-truth seed; only previous accepted laser pose.
    const auto seed=slam::compose(out.poses.back(),kBaseFromLaser);
    const auto candidate=match(field,p.raw_lidar[i],seed);
    if(qualified(candidate,*snapshot,seed,.65)) {
      const auto base=slam::compose(candidate.map_from_laser,
                                   slam::inverse(kBaseFromLaser));
      out.poses.push_back(base);
      out.mapped[i]=true;
      insert(rolling,base,p.raw_lidar[i]);
      ++out.accepted;
    } else {
      out.poses.push_back(out.poses.back());
      ++out.rejected;
      // An unlocalized moving scan is never inserted into the occupancy map.
    }
  }
  return out;
}
struct GraphOutput {
  std::vector<Pose2> poses;
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
    for(std::size_t i=19;i<matcher.poses.size();i+=2) {
      const auto& result=matcher.outcomes[i];
      if(result.status!=slam::matching::MatchStatus::accepted||
         result.diagnostics.supported_hits<25||
         result.diagnostics.overlap<.65||!result.diagnostics.full_rank)
        continue;
      std::optional<std::size_t> earlier;
      for(std::size_t j=0;j+15<i;++j) {
        if((matcher.poses[i].translation-matcher.poses[j].translation).norm()<.60){
          earlier=j;break;
        }
      }
      if(!earlier)continue;
      Pose2 measured=slam::compose(slam::inverse(sim.reference[*earlier]),
                                   sim.reference[i]);
      measured.translation.x()+=.012*std::sin(double(i));
      measured.translation.y()+=.012*std::cos(double(i));
      measured.yaw_rad=slam::wrap_yaw(measured.yaw_rad+.004*std::sin(double(i)*.3));
      input.loops.push_back({{*earlier,i,measured,.04,.04,.025,loop_source},
            result.diagnostics.supported_hits,result.diagnostics.overlap,true});
    }
  }
  output.loop_candidates=input.loops.size();
  auto graph=slam::graph::optimize_pose_graph(input);
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
void execute(const Simulation& sim,const std::filesystem::path& directory,
             bool synthetic_loops) {
  if(std::filesystem::exists(directory))
    throw std::invalid_argument("output already exists; refuse to overwrite");
  if(!std::filesystem::create_directories(directory))
    throw std::runtime_error("cannot create exclusive output directory");
  const auto p=prepare(sim.sensors,sim.initial_anchor);
  const auto matched=match_heldout(sim,p);
  const auto lidar=lidar_only(sim,p,p.gyro_wheel.front());
  const auto graph=optimize_graph(sim,p,matched,synthetic_loops);
  const auto wheel_map=make_map(sim,p.wheel_only,p.wheel_deskewed,
                                all(p.wheel_only.size()));
  const auto lidar_map=make_map(sim,lidar.poses,p.raw_lidar,lidar.mapped);
  const auto fused_map=make_map(sim,p.gyro_wheel,p.gyro_deskewed,
                                all(p.gyro_wheel.size()));
  const auto matched_map=make_map(sim,matched.poses,p.gyro_deskewed,
                                  all(matched.poses.size()));
  export_bundle(directory/"wheel_only.slammap",wheel_map);
  export_bundle(directory/"lidar_only.slammap",lidar_map);
  export_bundle(directory/"gyro_wheel.slammap",fused_map);
  export_bundle(directory/"fused_scan_matched.slammap",matched_map);
  std::optional<slam::mapping::MapBundle> graph_map;
  if(!graph.poses.empty()) {
    graph_map=make_map(sim,graph.poses,p.gyro_deskewed,all(graph.poses.size()));
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
     <<"\n\"heldout_attempted\":"<<matched.attempted<<","
     <<"\n\"heldout_accepted\":"<<matched.accepted<<","
     <<"\n\"heldout_rejected\":"<<matched.rejected<<","
     <<"\n\"lidar_only_accepted\":"<<lidar.accepted<<","
     <<"\n\"lidar_only_rejected\":"<<lidar.rejected<<","
     <<"\n\"graph_status\":\""<<graph.status<<"\","
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
  if(argc!=3&&argc!=4){
    std::cerr<<"Usage: slam_offline_fusion <apartment|warehouse> "
                "<new-output-directory> [--synthetic-independent-loops]\n";
    return 2;
  }
  try{
    const bool loops=argc==4;
    if(loops&&std::string(argv[3])!="--synthetic-independent-loops")
      throw std::invalid_argument("unrecognized option");
#ifndef SLAM_FUSION_WITH_GTSAM
    if(loops)throw std::invalid_argument(
      "independent synthetic loop demonstration requires a GTSAM-enabled build");
#endif
    execute(generate(argv[1]),argv[2],loops);
    return 0;
  }catch(const std::exception& e){
    std::cerr<<"offline fusion failed: "<<e.what()<<'\n';
    return 1;
  }
}
