#include "slam/datasets/fr079_offline.hpp"

#include "slam/core/se2.hpp"
#include "slam/mapping/distance_field.hpp"
#include "slam/mapping/occupancy.hpp"
#include "slam/matching/geometry.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace slam::datasets {
namespace {
using Clock=std::chrono::steady_clock;
double milliseconds(Clock::time_point a,Clock::time_point b){
  return std::chrono::duration<double,std::milli>(b-a).count();
}
void require(bool condition,const char* why) {
  if(!condition)throw std::invalid_argument(why);
}
bool finite_pose(const Pose2Record& pose) {
  return std::isfinite(pose.x)&&std::isfinite(pose.y)&&std::isfinite(pose.yaw)&&
         std::abs(pose.x)<1e5&&std::abs(pose.y)<1e5&&std::abs(pose.yaw)<1e5;
}
Pose2 as_pose(const Pose2Record& p) {
  return {{p.x,p.y},slam::wrap_yaw(p.yaw)};
}
std::vector<Eigen::Vector2d> points(const CarmenLaser& scan,
                                    std::size_t stride,double max_range) {
  std::vector<Eigen::Vector2d> out;
  out.reserve((scan.ranges.size()+stride-1)/stride);
  for(std::size_t i=0;i<scan.ranges.size();i+=stride) {
    const auto& b=scan.ranges[i];
    if(b.status!=RangeStatus::hit||!std::isfinite(b.metres)||
       !std::isfinite(b.angle_rad)||b.metres<0.10||b.metres>max_range)
      continue;
    out.emplace_back(b.metres*std::cos(b.angle_rad),
                     b.metres*std::sin(b.angle_rad));
  }
  return out;
}
mapping::GridConfig bounds_from_odom(const std::vector<Pose2>& odom,
                                    const Fr079SlamOptions& opt) {
  // Allocating canvas from the entire *odometry-only* path does not insert
  // future scan geometry into the map. A future online port should implement
  // growing map chunks instead of offline canvas preallocation.
  double minx=std::numeric_limits<double>::infinity(),miny=minx;
  double maxx=-minx,maxy=-minx;
  for(const auto& pose:odom) {
    minx=std::min(minx,pose.translation.x());
    miny=std::min(miny,pose.translation.y());
    maxx=std::max(maxx,pose.translation.x());
    maxy=std::max(maxy,pose.translation.y());
  }
  const auto width=std::ceil((maxx-minx+2*opt.map_margin_m)/opt.map_resolution_m);
  const auto height=std::ceil((maxy-miny+2*opt.map_margin_m)/opt.map_resolution_m);
  require(std::isfinite(width)&&std::isfinite(height)&&width>=4&&height>=4&&
          width<=2500&&height<=2500&&width*height<=4'000'000,
          "odometry envelope exceeds safe FR079 map bounds");
  mapping::GridConfig grid;
  grid.width=static_cast<std::size_t>(width);
  grid.height=static_cast<std::size_t>(height);
  grid.resolution_m=opt.map_resolution_m;
  grid.map_from_grid={{minx-opt.map_margin_m,miny-opt.map_margin_m},0};
  // There are NO trusted no-return rays in this CARMEN format.
  grid.clear_trusted_no_returns=false;
  return grid;
}
const char* match_status(matching::MatchStatus s) {
  switch(s){
    case matching::MatchStatus::accepted:return "accepted";
    case matching::MatchStatus::insufficient_support:return "insufficient_support";
    case matching::MatchStatus::degenerate:return "degenerate";
    case matching::MatchStatus::no_improvement:return "no_improvement";
    case matching::MatchStatus::invalid_input:return "invalid_input";
  }
  return "invalid_input";
}
}

Fr079SlamResult run_fr079_offline_slam(
    const std::vector<CarmenLaser>& scans,const Fr079SlamOptions& opt) {
  require(!scans.empty()&&scans.size()<=6000,"FR079 scan count outside supported range");
  require(std::isfinite(opt.map_resolution_m)&&opt.map_resolution_m>=0.05&&
          opt.map_resolution_m<=0.5&&std::isfinite(opt.max_hit_range_m)&&
          opt.max_hit_range_m>=1&&opt.max_hit_range_m<=40&&
          std::isfinite(opt.map_margin_m)&&opt.map_margin_m>=opt.max_hit_range_m+1&&
          opt.map_margin_m<=80&&opt.map_beam_stride>=1&&opt.map_beam_stride<=64&&
          opt.match_beam_stride>=1&&opt.match_beam_stride<=64&&
          opt.match_every>=1&&opt.match_every<=200&&
          opt.rebuild_field_every>=1&&opt.rebuild_field_every<=200&&
          std::isfinite(opt.max_correction_m)&&opt.max_correction_m>0&&
          opt.max_correction_m<=2&&
          std::isfinite(opt.max_correction_yaw_rad)&&
          opt.max_correction_yaw_rad>0&&opt.max_correction_yaw_rad<=0.5,
          "invalid FR079 SLAM options");

#ifndef SLAM_HAS_CERES
  require(!opt.use_ceres,"Ceres requested but not linked: select --reference explicitly");
#endif

  const Pose2 first_raw=as_pose(scans.front().logged_odometry_pose);
  const Pose2 first_inverse=slam::inverse(first_raw);
  std::vector<Pose2> odometry;
  odometry.reserve(scans.size());
  double previous_time=-std::numeric_limits<double>::infinity();
  for(const auto& scan:scans){
    require(finite_pose(scan.logged_odometry_pose)&&
            std::isfinite(scan.ipc_seconds)&&scan.ipc_seconds>=previous_time&&
            scan.ranges.size()>0&&scan.ranges.size()<=4096,
            "nonmonotonic timestamps, invalid odometry or unsupported scan");
    require(!scan.per_beam_time_known,"FR079 unexpectedly claims calibrated beam timing");
    previous_time=scan.ipc_seconds;
    odometry.push_back(slam::compose(first_inverse,as_pose(scan.logged_odometry_pose)));
  }

  mapping::OccupancyGrid grid(bounds_from_odom(odometry,opt));
  Fr079SlamResult result;
  result.used_ceres=opt.use_ceres;
  result.frames.reserve(scans.size());
  Pose2 last_estimate=odometry.front(),last_odom=odometry.front();
  std::shared_ptr<const mapping::MapSnapshot> cached_snapshot;
  std::shared_ptr<const mapping::DistanceField> cached_field;
  std::size_t last_field_frame=0;

  for(std::size_t i=0;i<scans.size();++i){
    const auto begin=Clock::now();
    const auto& scan=scans[i];
    const auto& odom=odometry[i];
    Fr079SlamFrame frame;
    frame.odom=odom;
    frame.scan_time_s=scan.ipc_seconds;
    frame.beam_count=scan.ranges.size();
    frame.valid_hits=static_cast<std::size_t>(std::count_if(
      scan.ranges.begin(),scan.ranges.end(),[&](const RangeRecord& beam){
        return beam.status==RangeStatus::hit&&std::isfinite(beam.metres)&&
               beam.metres>=.10&&beam.metres<=opt.max_hit_range_m;
      }));

    // Odometry increments, not logged laser poses or simulator truth.
    // Accepted scan corrections persist into subsequent predictions.
    const Pose2 delta=slam::compose(slam::inverse(last_odom),odom);
    frame.predicted=slam::compose(last_estimate,delta);
    frame.estimated=frame.predicted;
    last_odom=odom;

    const auto scan_points=points(scan,opt.map_beam_stride,opt.max_hit_range_m);
    frame.mapped_hits=scan_points.size();
    if(i>0&&i%opt.match_every==0&&scan_points.size()>=12&&
       result.inserted_scans>=3){
      const auto start_field=Clock::now();
      if(!cached_field||i-last_field_frame>=opt.rebuild_field_every){
        cached_snapshot=mapping::MapSnapshot::capture(grid);
        cached_field=mapping::DistanceField::build(cached_snapshot,.70);
        last_field_frame=i;
      }
      frame.field_ms=milliseconds(start_field,Clock::now());
      result.total_field_ms+=frame.field_ms;
      if(cached_field&&cached_field->has_obstacles()){
        ++result.attempts;frame.matched=true;
        matching::MatchSettings settings;
        settings.min_supported_hits=12;
        settings.minimum_overlap=.50;
        settings.maximum_iterations=30;
        settings.maximum_step_translation_m=.12;
        settings.maximum_step_yaw_rad=.08;
        const auto scan_match_points=points(scan,opt.match_beam_stride,opt.max_hit_range_m);
        const auto start_match=Clock::now();
        // The last cached field contains strictly earlier scans, with an
        // intentionally bounded refresh cadence; current scan never trains
        // against itself. Neither logged laser pose nor full-log map is used.
        matching::MatchResult candidate;
#ifdef SLAM_HAS_CERES
        if(opt.use_ceres)
          candidate=matching::match_ceres(cached_field,scan_match_points,
                                          frame.predicted,settings);
        else
#endif
          candidate=matching::match_reference(cached_field,scan_match_points,
                                               frame.predicted,settings);
        frame.match_ms=milliseconds(start_match,Clock::now());
        result.total_match_ms+=frame.match_ms;
        const auto& d=candidate.diagnostics;
        frame.supported_hits=d.supported_hits;frame.overlap=d.overlap;
        frame.robust_cost=d.robust_cost;
        frame.match_status=match_status(candidate.status);
        const bool quality=candidate.status==matching::MatchStatus::accepted&&
          d.source_revision==cached_snapshot->revision()&&
          d.source_fingerprint==cached_snapshot->fingerprint()&&
          d.full_rank&&d.supported_hits>=12&&d.overlap>=.50&&
          (candidate.map_from_laser.translation-frame.predicted.translation).norm()
            <=opt.max_correction_m&&
          std::abs(slam::wrap_yaw(candidate.map_from_laser.yaw_rad-
                                  frame.predicted.yaw_rad))<=opt.max_correction_yaw_rad;
        if(quality){
          frame.accepted=true;frame.estimated=candidate.map_from_laser;
          ++result.accepted;
        }else {
          if(candidate.status==matching::MatchStatus::accepted)
            frame.match_status="rejected_innovation_or_provenance";
          ++result.rejected;
        }
      }else frame.match_status="insufficient_prior_map_geometry";
    }

    // Laser mounting is *assumed identity* in this first FR079 experiment;
    // this is not an independently calibrated base-to-laser transform.
    // Only valid finite in-range hits create rays, never invalid returns.
    const auto start_insert=Clock::now();
    if(!scan_points.empty()) {
      std::vector<mapping::WorldRay> rays;
      rays.reserve(scan_points.size());
      for(const auto& point:scan_points){
        rays.push_back({frame.estimated.translation,
                        slam::transform(frame.estimated,point),
                        sensors::ReturnKind::hit});
      }
      (void)grid.integrate_rays(rays);
      frame.map_inserted=true;++result.inserted_scans;
    }
    frame.insertion_ms=milliseconds(start_insert,Clock::now());
    result.total_insertion_ms+=frame.insertion_ms;
    last_estimate=frame.estimated;
    frame.frame_ms=milliseconds(begin,Clock::now());
    result.frames.push_back(std::move(frame));
  }
  result.map={mapping::MapSnapshot::capture(grid),
    {"fr079_odom_scan_frame_UNVERIFIED",
     "fr079-carmen-odom-lidar-incremental-v1",
     "estimated_trajectory_no_independent_ground_truth",false}};
  return result;
}
} // namespace slam::datasets
