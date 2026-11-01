#include "slam/datasets/fr079_train_only.hpp"
#include "slam/core/se2.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace slam::datasets {
namespace {
void validate(const TrainOnlyOptions& o){
  if (o.holdout_modulus<2||o.training_stride<2||o.heldout_query_stride==0||o.beam_stride==0||
      !std::isfinite(o.min_hit_range_m)||o.min_hit_range_m<=0||
      !std::isfinite(o.max_hit_range_m)||o.max_hit_range_m<=o.min_hit_range_m||o.max_hit_range_m>100||
      !std::isfinite(o.grid_resolution_m)||o.grid_resolution_m<=0||o.grid_resolution_m>1||
      !std::isfinite(o.grid_margin_m)||o.grid_margin_m<=0||o.grid_margin_m>100||
      !std::isfinite(o.edt_truncation_m)||o.edt_truncation_m<=0||o.edt_truncation_m>10||
      !std::isfinite(o.seed_dx_m)||!std::isfinite(o.seed_dy_m)||!std::isfinite(o.seed_dyaw_rad))
    throw std::invalid_argument("invalid train-only FR079 options");
}
slam::Pose2 logged_pose(const CarmenLaser& scan){
  const auto& pose=scan.logged_laser_pose;
  if(!std::isfinite(pose.x)||!std::isfinite(pose.y)||!std::isfinite(pose.yaw)||
     !std::isfinite(scan.ipc_seconds))throw std::invalid_argument("invalid logged pose/time");
  return {{pose.x,pose.y},pose.yaw};
}
std::vector<Eigen::Vector2d> points(const CarmenLaser& scan,const TrainOnlyOptions& o){
  std::vector<Eigen::Vector2d> hits;
  hits.reserve((scan.ranges.size()+o.beam_stride-1)/o.beam_stride);
  for(std::size_t i=0;i<scan.ranges.size();i+=o.beam_stride){
    const auto& b=scan.ranges[i];
    if(b.status!=RangeStatus::hit||b.metres<o.min_hit_range_m||b.metres>o.max_hit_range_m)continue;
    if(!std::isfinite(b.metres)||!std::isfinite(b.angle_rad))throw std::invalid_argument("invalid hit");
    hits.emplace_back(b.metres*std::cos(b.angle_rad),b.metres*std::sin(b.angle_rad));
  }
  return hits;
}
}
TrainingMap build_fr079_training_map(const std::vector<CarmenLaser>& scans,const TrainOnlyOptions& o){
  validate(o);
  if(scans.empty())throw std::invalid_argument("empty FR079 log");
  TrainingMap out;
  out.input_scans=scans.size();
  double minx=std::numeric_limits<double>::infinity(),miny=minx;
  double maxx=-minx,maxy=-minx;
  for(std::size_t i=0;i<scans.size();++i){
    if(i%o.holdout_modulus==0)continue;
    ++out.training_eligible;
    if(i%o.training_stride!=1)continue;
    const auto pose=logged_pose(scans[i]);
    minx=std::min(minx,pose.translation.x());miny=std::min(miny,pose.translation.y());
    maxx=std::max(maxx,pose.translation.x());maxy=std::max(maxy,pose.translation.y());
  }
  if(!std::isfinite(minx)||!std::isfinite(maxx))throw std::invalid_argument("no selected training poses");
  const auto cells=[&](double extent){
    const double count=std::ceil(extent/o.grid_resolution_m);
    if(!std::isfinite(count)||count>4000)throw std::invalid_argument("oversized FR079 grid");
    return static_cast<std::size_t>(count);
  };
  slam::mapping::GridConfig cfg;
  cfg.resolution_m=o.grid_resolution_m;
  cfg.width=cells(maxx-minx+2*o.grid_margin_m);
  cfg.height=cells(maxy-miny+2*o.grid_margin_m);
  cfg.map_from_grid={{minx-o.grid_margin_m,miny-o.grid_margin_m},0.0};
  cfg.clear_trusted_no_returns=false; // never synthesize free-space rays
  slam::mapping::OccupancyGrid map(cfg);
  for(std::size_t i=0;i<scans.size();++i){
    if(i%o.holdout_modulus==0||i%o.training_stride!=1)continue;
    const auto pose=logged_pose(scans[i]);
    std::vector<slam::mapping::WorldRay> rays;
    for(const auto& hit:points(scans[i],o))
      rays.push_back({pose.translation,slam::transform(pose,hit),slam::sensors::ReturnKind::hit});
    if(rays.empty())continue;
    (void)map.integrate_rays(rays);
    ++out.training_inserted;
    out.training_rays+=rays.size();
  }
  if(!out.training_inserted)throw std::invalid_argument("no training LiDAR hits");
  out.snapshot=slam::mapping::MapSnapshot::capture(map);
  out.field=slam::mapping::DistanceField::build(out.snapshot,o.edt_truncation_m);
  if(!out.field->has_obstacles())throw std::invalid_argument("no occupied training-map cells");
  return out;
}
HeldoutReport evaluate_fr079_holdout(const std::vector<CarmenLaser>& scans,
                                     const TrainingMap& train,const TrainOnlyOptions& o){
  validate(o);
  if(scans.size()!=train.input_scans||!train.snapshot||!train.field||
     train.field->source().get()!=train.snapshot.get())
    throw std::invalid_argument("FR079 scan set or immutable map generation mismatch");
#ifndef SLAM_HAS_CERES
  if(o.use_ceres)throw std::invalid_argument("Ceres requested but unavailable: build Ceres matcher or explicitly request reference mode");
#endif
  HeldoutReport r;
#ifdef SLAM_HAS_CERES
  r.used_ceres=o.use_ceres;
#endif
  r.input_scans=scans.size();r.training_eligible=train.training_eligible;
  r.training_inserted=train.training_inserted;r.training_map_fingerprint=train.snapshot->fingerprint();
  slam::matching::MatchSettings settings;
  settings.minimum_overlap=0.30;
  settings.min_supported_hits=10;
  settings.sigma_metres=o.grid_resolution_m;
  settings.maximum_step_translation_m=0.20;
  settings.maximum_step_yaw_rad=0.07;
  settings.maximum_iterations=25;
  std::size_t heldout_seen=0;
  for(std::size_t i=0;i<scans.size();++i){
    if(i%o.holdout_modulus!=0)continue;
    ++r.heldout_scans;
    if((heldout_seen++)%o.heldout_query_stride!=0)continue;
    const auto seed_logged=logged_pose(scans[i]);
    const slam::Pose2 seed{{seed_logged.translation.x()+o.seed_dx_m,
                            seed_logged.translation.y()+o.seed_dy_m},
                           slam::wrap_yaw(seed_logged.yaw_rad+o.seed_dyaw_rad)};
    const auto hits=points(scans[i],o);
    if(hits.empty()){++r.invalid;continue;}
    ++r.evaluated_queries;
    r.attempted_hits+=hits.size();
    const auto before=slam::matching::evaluate_geometry(*train.field,hits,seed,settings);
    slam::matching::MatchResult result;
#ifdef SLAM_HAS_CERES
    if(o.use_ceres)result=slam::matching::match_ceres(train.field,hits,seed,settings);
    else
#endif
      result=slam::matching::match_reference(train.field,hits,seed,settings);
    if(result.diagnostics.source_fingerprint!=train.snapshot->fingerprint())
      throw std::runtime_error("scan matcher used wrong source map");
    if(result.status==slam::matching::MatchStatus::accepted)++r.accepted;
    else if(result.status==slam::matching::MatchStatus::no_improvement)++r.no_improvement;
    else if(result.status==slam::matching::MatchStatus::insufficient_support)++r.rejected_support;
    else if(result.status==slam::matching::MatchStatus::degenerate)++r.rejected_degenerate;
    else ++r.invalid;
    r.mean_initial_overlap+=before.overlap;
    r.mean_final_overlap+=result.diagnostics.overlap;
    // Undefined empty-map residual cost is reported with overlap instead.
    r.mean_initial_robust_cost+=before.robust_cost;
    r.mean_final_robust_cost+=result.diagnostics.robust_cost;
    r.mean_displacement_from_logged_seed_m+=(result.map_from_laser.translation-seed_logged.translation).norm();
  }
  if(r.evaluated_queries){
    const double denom=static_cast<double>(r.evaluated_queries);
    r.mean_initial_overlap/=denom;r.mean_final_overlap/=denom;
    r.mean_initial_robust_cost/=denom;r.mean_final_robust_cost/=denom;
    r.mean_displacement_from_logged_seed_m/=denom;
  }
  return r;
}
} // namespace slam::datasets
