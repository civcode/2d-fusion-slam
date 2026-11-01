#include "slam/datasets/fr079_alignment.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace slam::datasets {
namespace {
struct Point { double x{},y{}; };
struct SampleSets { std::vector<Point> calibration, held_out; std::size_t training_scans{}, heldout_scans{}; };
SampleSets select(const std::vector<CarmenLaser>& scans, const AlignmentOptions& options) {
  SampleSets out;
  for (std::size_t idx=0; idx<scans.size(); ++idx) {
    const bool holdout = idx % options.holdout_modulus == 0;
    if (holdout) ++out.heldout_scans;
    else ++out.training_scans;
    // Only a subset of training scans is used for origin calibration.
    if (!holdout && idx % options.calibration_scan_stride != 1) continue;
    auto& dst=holdout ? out.held_out : out.calibration;
    const auto& scan=scans[idx];
    const auto& pose=scan.logged_laser_pose;
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw))
      throw std::invalid_argument("non-finite logged laser pose");
    for (std::size_t b=0; b<scan.ranges.size(); b+=options.beam_stride) {
      const auto& beam=scan.ranges[b];
      if (beam.status != RangeStatus::hit) continue;
      if (!std::isfinite(beam.metres) || beam.metres <= 0 || !std::isfinite(beam.angle_rad))
        throw std::invalid_argument("non-finite hit geometry");
      const double yaw=pose.yaw+beam.angle_rad;
      dst.push_back({pose.x+beam.metres*std::cos(yaw), pose.y+beam.metres*std::sin(yaw)});
    }
  }
  return out;
}
Point project(Point p, AlignmentHypothesis t) {
  if(t.swap_xy) std::swap(p.x,p.y);
  return {t.sign_x*p.x+t.translation_x,t.sign_y*p.y+t.translation_y};
}
std::vector<std::uint8_t> proximity(const CarmenMap& map, const AlignmentOptions& options) {
  const std::size_t n=static_cast<std::size_t>(map.width)*map.height;
  std::vector<std::uint8_t> near(n);
  const int radius=static_cast<int>(options.wall_radius_cells);
  for(std::size_t x=0;x<map.width;++x)for(std::size_t y=0;y<map.height;++y) {
    if(map.at_xy(x,y).value() <= options.occupied_threshold)continue;
    for(int dx=-radius;dx<=radius;++dx)for(int dy=-radius;dy<=radius;++dy) {
      if(dx*dx+dy*dy>radius*radius)continue;
      const auto xx=static_cast<std::int64_t>(x)+dx, yy=static_cast<std::int64_t>(y)+dy;
      if(xx>=0 && yy>=0 && xx<map.width && yy<map.height)
        near[static_cast<std::size_t>(xx)*map.height+static_cast<std::size_t>(yy)]=1;
    }
  }
  return near;
}
AlignmentScore score(const CarmenMap& map, const std::vector<std::uint8_t>& near,
                     const std::vector<Point>& points, AlignmentHypothesis transform) {
  AlignmentScore result;
  result.endpoints=points.size();
  for(const auto p:points) {
    const auto pt=project(p,transform);
    const double gx=pt.x/map.resolution_m, gy=pt.y/map.resolution_m;
    if(!std::isfinite(gx)||!std::isfinite(gy)||gx<0||gy<0||gx>=map.width||gy>=map.height)continue;
    const auto x=static_cast<std::size_t>(gx),y=static_cast<std::size_t>(gy);
    ++result.within_grid;
    if(map.at_xy(x,y).value()<0)continue;
    ++result.known;
    if(near[x*map.height+y])++result.near_occupied;
  }
  return result;
}
double objective(AlignmentScore score) {
  if(score.endpoints==0)return -std::numeric_limits<double>::infinity();
  const auto n=static_cast<double>(score.endpoints);
  // Penalizes unsupported geometry. An unknown/out-of-map ray is NEVER a match.
  return (score.near_occupied+0.10*score.known-0.25*(score.endpoints-score.known))/n;
}
}
double AlignmentScore::known_fraction()const noexcept {
  return endpoints?static_cast<double>(known)/endpoints:0.0;
}
double AlignmentScore::wall_fraction()const noexcept {
  return endpoints?static_cast<double>(near_occupied)/endpoints:0.0;
}
AlignmentResult diagnose_fr079_alignment(const CarmenMap& map,
                                         const std::vector<CarmenLaser>& scans,
                                         const AlignmentOptions& options) {
  if(scans.empty() || map.width==0 || map.height==0 ||
     map.values_in_file_order.size()!=static_cast<std::size_t>(map.width)*map.height ||
     !std::isfinite(map.resolution_m)||map.resolution_m<=0 ||
     options.holdout_modulus<2||options.beam_stride==0||options.calibration_scan_stride<2||
     options.wall_radius_cells>10||!std::isfinite(options.occupied_threshold)||
     options.occupied_threshold<0||options.occupied_threshold>1||
     !std::isfinite(options.coarse_step_m)||options.coarse_step_m<=0||
     !std::isfinite(options.offset_x_min)||!std::isfinite(options.offset_x_max)||
     !std::isfinite(options.offset_y_min)||!std::isfinite(options.offset_y_max)||
     options.offset_x_max<options.offset_x_min||options.offset_y_max<options.offset_y_min||
     (options.offset_x_max-options.offset_x_min)/options.coarse_step_m>200 ||
     (options.offset_y_max-options.offset_y_min)/options.coarse_step_m>200)
    throw std::invalid_argument("invalid FR079 reference map/registration options");
  const auto samples=select(scans,options);
  if(samples.calibration.empty()||samples.held_out.empty())
    throw std::invalid_argument("insufficient calibration or held-out hit observations");
  const auto near=proximity(map,options);
  AlignmentHypothesis best;
  AlignmentScore bestscore;
  double bestvalue=-std::numeric_limits<double>::infinity();
  // Eight discrete orthogonal frame hypotheses (rotations AND reflections).
  // A reflection cannot become a valid SE(2) transform without extra evidence.
  for(int swapped=0;swapped<2;++swapped)for(int sx:{-1,1})for(int sy:{-1,1}) {
    for(double x=options.offset_x_min; x<=options.offset_x_max+1e-9; x+=options.coarse_step_m)
      for(double y=options.offset_y_min; y<=options.offset_y_max+1e-9; y+=options.coarse_step_m) {
        const AlignmentHypothesis candidate{swapped!=0,sx,sy,x,y};
        const auto measurement=score(map,near,samples.calibration,candidate);
        const auto value=objective(measurement);
        if(value>bestvalue) {bestvalue=value;best=candidate;bestscore=measurement;}
      }
  }
  // Submetre refinement uses training data only; never held-out queries.
  const auto coarse=best;
  for(int dx=-4;dx<=4;++dx)for(int dy=-4;dy<=4;++dy) {
    auto t=coarse;
    t.translation_x+=dx*options.coarse_step_m/4;
    t.translation_y+=dy*options.coarse_step_m/4;
    const auto measurement=score(map,near,samples.calibration,t);
    if(const double value=objective(measurement);value>bestvalue){bestvalue=value;best=t;bestscore=measurement;}
  }
  return {best,bestscore,score(map,near,samples.held_out,best),samples.training_scans,
          samples.heldout_scans,false};
}
} // namespace slam::datasets
