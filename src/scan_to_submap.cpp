#include "slam/matching/scan_to_submap.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace slam::matching {
namespace {
constexpr std::size_t kMaxRasterCells=900000;
struct Score {
  double value{}, rms{};
  std::size_t supported{};
};

[[nodiscard]] bool finite(const Pose2& p) {
  return p.translation.allFinite() && std::isfinite(p.yaw_rad);
}
[[nodiscard]] std::size_t offset(const FrozenLidarSubmap& map,
                                 std::size_t x,std::size_t y) {
  return y*map.width+x;
}
[[nodiscard]] Score score_pose(const FrozenLidarSubmap& map,
                               const std::vector<Eigen::Vector2d>& hits,
                               const Pose2& pose,
                               std::size_t step = 1) {
  Score result;
  if(hits.empty() || !finite(pose) || !map.valid())return result;
  const double c=std::cos(pose.yaw_rad),s=std::sin(pose.yaw_rad);
  const double inv=1.0/map.resolution_m;
  double sum=0.,sq_error=0.;
  std::size_t examined=0;
  for(std::size_t k=0;k<hits.size();k+=step) {
    const auto& p=hits[k];
    const double x=pose.translation.x()+c*p.x()-s*p.y();
    const double y=pose.translation.y()+s*p.x()+c*p.y();
    const int ix=static_cast<int>(std::floor((x-map.origin_x)*inv));
    const int iy=static_cast<int>(std::floor((y-map.origin_y)*inv));
    ++examined;
    if(ix<0 || iy<0 || ix>=static_cast<int>(map.width) ||
       iy>=static_cast<int>(map.height))continue;
    const std::uint8_t weight=map.likelihood[offset(map,ix,iy)];
    sum+=double(weight)/255.;
    if(weight>=110) {
      ++result.supported;
      const double d=weight>=250?0.03:(weight>=210?0.10:0.19);
      sq_error+=d*d;
    }
  }
  result.value=sum/double(examined);
  result.rms=result.supported?std::sqrt(sq_error/double(result.supported)):0.;
  return result;
}
[[nodiscard]] Pose2 displaced(const Pose2& p,double dx,double dy,double yaw) {
  return {{p.translation.x()+dx,p.translation.y()+dy},
          slam::wrap_yaw(p.yaw_rad+yaw)};
}
[[nodiscard]] bool within_window(const Pose2& prior, const Pose2& p,
                                 const SubmapSearchLimits& limits) {
  const Pose2 correction=slam::compose(slam::inverse(prior),p);
  return correction.translation.norm()<=limits.translation_window_m+1e-10 &&
         std::abs(slam::wrap_yaw(correction.yaw_rad))<=
             limits.rotation_window_rad+1e-10;
}
} // namespace

SubmapDescriptor describe_frozen_lidar_submap(const FrozenLidarSubmap& map) {
  SubmapDescriptor descriptor;
  if(!map.valid() || map.occupied_cells<40)return descriptor;
  // Use geometric farthest-point sampling rather than row-major raster
  // subsampling. The seed is closest to the point centroid and subsequent
  // samples maximize distance from the already selected set. This keeps
  // the pair-distance distribution comparatively stable under SE(2) change
  // of local coordinates (up to the 10cm raster quantization).
  constexpr std::size_t max_samples=96;
  std::vector<Eigen::Vector2d> all;
  all.reserve(map.occupied_cells);
  Eigen::Vector2d centroid=Eigen::Vector2d::Zero();
  for(std::size_t y=0;y<map.height;++y)
    for(std::size_t x=0;x<map.width;++x)
      if(map.likelihood[y*map.width+x]==255) {
        all.emplace_back((double(x)+.5)*map.resolution_m,
                         (double(y)+.5)*map.resolution_m);
        centroid+=all.back();
      }
  if(all.size()<40)return descriptor;
  centroid/=double(all.size());
  std::size_t seed=0;
  double best_distance=std::numeric_limits<double>::infinity();
  for(std::size_t i=0;i<all.size();++i) {
    const double d=(all[i]-centroid).squaredNorm();
    if(d<best_distance){best_distance=d;seed=i;}
  }
  std::vector<double> nearest(all.size(),
                              std::numeric_limits<double>::infinity());
  std::vector<Eigen::Vector2d> samples;
  samples.reserve(max_samples);
  for(std::size_t k=0;k<std::min(max_samples,all.size());++k) {
    samples.push_back(all[seed]);
    double farthest=-1.;
    std::size_t next=seed;
    for(std::size_t i=0;i<all.size();++i) {
      nearest[i]=std::min(nearest[i],
                         (all[i]-all[seed]).squaredNorm());
      if(nearest[i]>farthest) {farthest=nearest[i];next=i;}
    }
    seed=next;
  }
  if(samples.size()<30)return descriptor;
  for(std::size_t i=0;i<samples.size();++i)
    for(std::size_t j=i+1;j<samples.size();++j) {
      const double dist=(samples[i]-samples[j]).norm();
      const std::size_t bin=std::min<std::size_t>(
          descriptor.pair_distance_histogram.size()-1,
          static_cast<std::size_t>(dist/1.0));
      ++descriptor.pair_distance_histogram[bin];
    }
  double sq=0.;
  for(double v:descriptor.pair_distance_histogram)sq+=v*v;
  if(sq<=0)return descriptor;
  const double inv=1./std::sqrt(sq);
  for(double& v:descriptor.pair_distance_histogram)v*=inv;
  descriptor.valid=true;
  return descriptor;
}
double submap_descriptor_similarity(const SubmapDescriptor& a,
                                    const SubmapDescriptor& b) noexcept {
  if(!a.valid||!b.valid)return 0.;
  double score=0.;
  for(std::size_t i=0;i<a.pair_distance_histogram.size();++i)
    score+=a.pair_distance_histogram[i]*b.pair_distance_histogram[i];
  return std::clamp(score,0.,1.);
}

bool FrozenLidarSubmap::valid() const noexcept {
  return width>0 && height>0 && width*height==likelihood.size() &&
         std::isfinite(origin_x) && std::isfinite(origin_y) &&
         std::isfinite(resolution_m) && resolution_m>0 && occupied_cells>0;
}

FrozenLidarSubmap build_frozen_lidar_submap(
    const std::vector<SubmapSweep>& sweeps,double resolution_m) {
  if(!std::isfinite(resolution_m) || resolution_m<.05 || resolution_m>.4 ||
     sweeps.empty() || sweeps.size()>80)
    throw std::invalid_argument("invalid frozen LiDAR submap arguments");
  std::vector<Eigen::Vector2d> points;
  double xmin=std::numeric_limits<double>::infinity();
  double ymin=xmin,xmax=-xmin,ymax=-xmin;
  std::size_t contributors=0;
  for(const auto& sweep:sweeps) {
    if(!finite(sweep.anchor_from_scan) || sweep.hits_scan.size()>4096)
      throw std::invalid_argument("invalid frozen submap scan pose/size");
    if(sweep.hits_scan.empty())continue;
    ++contributors;
    for(const auto& hit:sweep.hits_scan) {
      if(!hit.allFinite() || hit.norm()>100.)
        throw std::invalid_argument("invalid frozen submap hit");
      const auto p=slam::transform(sweep.anchor_from_scan,hit);
      xmin=std::min(xmin,p.x());xmax=std::max(xmax,p.x());
      ymin=std::min(ymin,p.y());ymax=std::max(ymax,p.y());
      points.push_back(p);
    }
  }
  FrozenLidarSubmap map;
  if(points.size()<80 || contributors<3)return map;
  const double pad=.5;
  map.origin_x=std::floor((xmin-pad)/resolution_m)*resolution_m;
  map.origin_y=std::floor((ymin-pad)/resolution_m)*resolution_m;
  map.width=static_cast<std::size_t>(
      std::ceil((xmax+pad-map.origin_x)/resolution_m))+1;
  map.height=static_cast<std::size_t>(
      std::ceil((ymax+pad-map.origin_y)/resolution_m))+1;
  if(!map.width || !map.height || map.width>kMaxRasterCells/map.height)
    throw std::invalid_argument("frozen submap exceeds raster budget");
  map.resolution_m=resolution_m;
  map.contributing_scans=contributors;
  const auto count=map.width*map.height;
  std::vector<std::uint8_t> occupancy(count,0);
  for(const auto& p:points) {
    const auto x=static_cast<std::size_t>(std::floor(
        (p.x()-map.origin_x)/resolution_m));
    const auto y=static_cast<std::size_t>(std::floor(
        (p.y()-map.origin_y)/resolution_m));
    if(x>=map.width || y>=map.height)continue;
    auto& n=occupancy[offset(map,x,y)];
    if(n<255)++n;
  }
  map.likelihood.assign(count,0);
  // Endpoint counts suppress isolated noise; a compact distance-likelihood
  // kernel makes coarse correlative search tolerant of discretization.
  for(std::size_t y=0;y<map.height;++y)
    for(std::size_t x=0;x<map.width;++x) {
      if(occupancy[offset(map,x,y)]<2)continue;
      ++map.occupied_cells;
      for(int dy=-3;dy<=3;++dy)
        for(int dx=-3;dx<=3;++dx) {
          const int d2=dx*dx+dy*dy;
          if(d2>9)continue;
          const auto xx=static_cast<int>(x)+dx;
          const auto yy=static_cast<int>(y)+dy;
          if(xx<0 || yy<0 || xx>=static_cast<int>(map.width) ||
             yy>=static_cast<int>(map.height))continue;
          const std::uint8_t value=d2==0?255:(d2<=2?220:(d2<=5?155:65));
          auto& cell=map.likelihood[offset(map,xx,yy)];
          cell=std::max(cell,value);
        }
    }
  return map;
}

SubmapMatchResult register_scan_to_frozen_submap(
    const FrozenLidarSubmap& map,
    const std::vector<Eigen::Vector2d>& hits,
    Pose2 prior,SubmapSearchLimits limits) {
  SubmapMatchResult result;
  result.anchor_from_current=prior;
  if(!map.valid() || hits.size()<limits.minimum_supported_hits ||
     hits.size()>4096 || !finite(prior) ||
     !std::isfinite(limits.translation_window_m) ||
     !std::isfinite(limits.rotation_window_rad) ||
     limits.translation_window_m<=0 ||
     limits.translation_window_m>(limits.global_search?5.0:.8) ||
     limits.rotation_window_rad<=0 ||
     limits.rotation_window_rad>(limits.global_search?.65:.35) ||
     !std::isfinite(limits.minimum_support_fraction) ||
     limits.minimum_support_fraction<=0 || limits.minimum_support_fraction>1 ||
     !std::isfinite(limits.minimum_score) || limits.minimum_score<=0 ||
     limits.minimum_score>1 || !std::isfinite(limits.minimum_translation_curvature) ||
     limits.minimum_translation_curvature<=0 ||
     !std::isfinite(limits.minimum_curvature_ratio) ||
     limits.minimum_curvature_ratio<=0 || limits.minimum_curvature_ratio>1)
    return result;
  for(const auto& p:hits)
    if(!p.allFinite() || p.norm()>100.)return result;
  // Coarse-to-fine deterministic correlative search (no ICP dependence).
  // Evaluate sparse beams during search; recompute all beams for acceptance.
  const std::size_t step=std::max<std::size_t>(1,hits.size()/120);
  double best=-1.;
  Pose2 best_pose=prior;
  auto search=[&](const Pose2& center,double xy_step,int steps,
                  double yaw_step,int yaw_steps) {
    for(int yi=-yaw_steps;yi<=yaw_steps;++yi)
      for(int xi=-steps;xi<=steps;++xi)
        for(int xj=-steps;xj<=steps;++xj) {
          const Pose2 p=displaced(center,xi*xy_step,xj*xy_step,
                                   yi*yaw_step);
          if(!within_window(prior,p,limits))continue;
          const double score=score_pose(map,hits,p,step).value;
          if(score>best+1e-9) {
            best=score;
            best_pose=p;
          }
        }
  };
  best=score_pose(map,hits,prior,step).value;
  if(limits.global_search) {
    // Wide, orientation-aware coarse grid: search displacement even when
    // Ceres poses differ by more than the normal spatial candidate radius.
    // Score is never sufficient alone: later guards check the curvature
    // and local distinctiveness of the correlation peak.
    const int n=static_cast<int>(std::ceil(
        limits.translation_window_m/.50));
    const int yaw_n=static_cast<int>(std::ceil(
        limits.rotation_window_rad/.10));
    search(prior,.50,n,.10,yaw_n);
    const Pose2 wide_best=best_pose;
    search(wide_best,.12,4,.025,4);
    const Pose2 middle_best=best_pose;
    search(middle_best,.04,3,.01,3);
  } else {
    search(prior,.16,3,.06,3);
    const Pose2 coarse_best=best_pose;
    search(coarse_best,.04,3,.015,3);
  }
  const Score final=score_pose(map,hits,best_pose);
  result.anchor_from_current=best_pose;
  result.correlation_score=final.value;
  result.supported_hits=final.supported;
  result.support_fraction=double(final.supported)/double(hits.size());
  result.rms_m=final.rms;
  const Pose2 correction=slam::compose(slam::inverse(prior),best_pose);
  result.innovation_m=correction.translation.norm();
  result.innovation_yaw_rad=std::abs(slam::wrap_yaw(correction.yaw_rad));

  // Verify that the match has a 2D peak, not a flat ridge along a corridor.
  // The negative Hessian of the local correlation score is approximated by
  // symmetric finite differences. Both translation eigen-directions must
  // have meaningful curvature. It is a heuristic, NOT calibrated covariance.
  constexpr double d=.40;
  const double s0=final.value;
  const double xp=score_pose(map,hits,displaced(best_pose,d,0,0)).value;
  const double xm=score_pose(map,hits,displaced(best_pose,-d,0,0)).value;
  const double yp=score_pose(map,hits,displaced(best_pose,0,d,0)).value;
  const double ym=score_pose(map,hits,displaced(best_pose,0,-d,0)).value;
  const double pp=score_pose(map,hits,displaced(best_pose,d,d,0)).value;
  const double pm=score_pose(map,hits,displaced(best_pose,d,-d,0)).value;
  const double mp=score_pose(map,hits,displaced(best_pose,-d,d,0)).value;
  const double mm=score_pose(map,hits,displaced(best_pose,-d,-d,0)).value;
  const double xx=(2*s0-xp-xm)/(d*d);
  const double yy=(2*s0-yp-ym)/(d*d);
  const double xy=-(pp-pm-mp+mm)/(4*d*d);
  const double trace=xx+yy;
  const double disc=std::hypot(xx-yy,2*xy);
  const double smallest=.5*(trace-disc),largest=.5*(trace+disc);
  result.translation_curvature_min=smallest;
  result.translation_curvature_ratio=largest>0?smallest/largest:0.;
  result.minimum_shift_score_drop=std::min({s0-xp,s0-xm,s0-yp,s0-ym});
  result.observable=smallest>=limits.minimum_translation_curvature &&
      result.translation_curvature_ratio>=limits.minimum_curvature_ratio &&
      result.minimum_shift_score_drop>=.01;
  result.accepted=result.observable &&
      result.supported_hits>=limits.minimum_supported_hits &&
      result.support_fraction>=limits.minimum_support_fraction &&
      result.correlation_score>=limits.minimum_score &&
      result.rms_m<=.22 &&
      within_window(prior,best_pose,limits);
  return result;
}
} // namespace slam::matching
