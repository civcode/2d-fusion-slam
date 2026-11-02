#include "slam/matching/scan_to_scan.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace slam::matching {
namespace {
using Points=std::vector<Eigen::Vector2d>;
constexpr double kMaxPairM=0.55;
constexpr double kCheckM=0.30;
constexpr std::size_t kMinMatches=24;

struct Pair {
  Eigen::Vector2d source,target;
  double d2{};
};
double nearest_squared(const Eigen::Vector2d& query,const Points& cloud,
                       Eigen::Vector2d* nearest=nullptr) {
  double best=std::numeric_limits<double>::infinity();
  for(const auto& target:cloud) {
    const double d2=(query-target).squaredNorm();
    if(d2<best) {
      best=d2;
      if(nearest)*nearest=target;
    }
  }
  return best;
}
bool valid_cloud(const Points& cloud) {
  if(cloud.size()<kMinMatches||cloud.size()>4096)return false;
  for(const auto& point:cloud)
    if(!point.allFinite()||point.norm()>100.)return false;
  Eigen::Vector2d mean=Eigen::Vector2d::Zero();
  for(const auto& point:cloud)mean+=point;
  mean/=double(cloud.size());
  Eigen::Matrix2d cov=Eigen::Matrix2d::Zero();
  for(const auto& point:cloud) {
    const Eigen::Vector2d d=point-mean;
    cov.noalias()+=d*d.transpose();
  }
  cov/=double(cloud.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
  return eig.info()==Eigen::Success&&eig.eigenvalues()[0]>.05&&
      eig.eigenvalues()[0]>.003*eig.eigenvalues()[1];
}
std::vector<Pair> pairs(const Points& previous,const Points& current,
                        const Pose2& estimate) {
  std::vector<Pair> out;
  out.reserve(current.size());
  constexpr double limit=kMaxPairM*kMaxPairM;
  for(const auto& point:current) {
    const Eigen::Vector2d transformed=transform(estimate,point);
    Eigen::Vector2d closest;
    const double d2=nearest_squared(transformed,previous,&closest);
    if(d2<limit)out.push_back({point,closest,d2});
  }
  // Trim the largest fifth of distances: avoid a few occlusions or gaps
  // dominating the rigid least-squares update.
  std::sort(out.begin(),out.end(),[](const Pair& a,const Pair& b){
    return a.d2<b.d2;
  });
  if(out.size()>kMinMatches)
    out.resize(std::max(kMinMatches,out.size()*4/5));
  return out;
}
bool selected_correspondence_rank(const std::vector<Pair>& matched) {
  if(matched.size()<40)return false;
  // A full scan can be well distributed even if only a short wall segment
  // was matched. Check geometry among actual selected correspondences.
  Eigen::Vector2d mean=Eigen::Vector2d::Zero();
  for(const auto& p:matched)mean+=p.target;
  mean/=double(matched.size());
  Eigen::Matrix2d cov=Eigen::Matrix2d::Zero();
  for(const auto& p:matched) {
    const auto d=p.target-mean;
    cov.noalias()+=d*d.transpose();
  }
  cov/=double(matched.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen(cov);
  return eigen.info()==Eigen::Success &&
         eigen.eigenvalues()[0]>.05 &&
         eigen.eigenvalues()[0]>.01*eigen.eigenvalues()[1];
}
Pose2 solve_pairs(const std::vector<Pair>& matched) {
  Eigen::Vector2d a=Eigen::Vector2d::Zero(),b=Eigen::Vector2d::Zero();
  for(const auto& p:matched) {a+=p.source;b+=p.target;}
  a/=double(matched.size());b/=double(matched.size());
  double c=0,s=0;
  for(const auto& p:matched) {
    const Eigen::Vector2d x=p.source-a,y=p.target-b;
    c+=x.dot(y);
    s+=x.x()*y.y()-x.y()*y.x();
  }
  const double yaw=std::atan2(s,c);
  const double cosine=std::cos(yaw),sine=std::sin(yaw);
  const Eigen::Vector2d rotated{cosine*a.x()-sine*a.y(),
                                 sine*a.x()+cosine*a.y()};
  return {b-rotated,yaw};
}
ScanToScanResult score(const Points& previous,const Points& current,
                       Pose2 estimated,std::size_t iterations,
                       const ScanRegistrationLimits& limits) {
  ScanToScanResult r;
  r.previous_from_current=estimated;
  r.iterations=iterations;
  r.correspondence_rank=selected_correspondence_rank(
      pairs(previous,current,estimated));
  r.translation_m=estimated.translation.norm();
  r.rotation_rad=std::abs(wrap_yaw(estimated.yaw_rad));
  if(r.translation_m>limits.max_translation_m||
     r.rotation_rad>limits.max_rotation_rad)
    return r;
  constexpr double max_d2=kCheckM*kCheckM;
  double loss=0;
  for(const auto& p:current) {
    const double d2=nearest_squared(transform(estimated,p),previous);
    if(d2<max_d2) {
      ++r.forward_inliers;
      loss+=d2;
    }
  }
  const auto inv=inverse(estimated);
  for(const auto& p:previous)
    if(nearest_squared(transform(inv,p),current)<max_d2)
      ++r.reverse_inliers;
  r.forward_overlap=double(r.forward_inliers)/double(current.size());
  r.reverse_overlap=double(r.reverse_inliers)/double(previous.size());
  // Keep rejected-run diagnostics valid JSON even with zero inliers.
  r.mean_error_m=r.forward_inliers?std::sqrt(loss/double(r.forward_inliers)):0.;
  // Symmetric check: a scan cannot "match" only a tiny convenient corner.
  r.accepted=r.forward_inliers>=kMinMatches&&r.reverse_inliers>=kMinMatches&&
      r.forward_overlap>=.65&&r.reverse_overlap>=.65&&
      r.mean_error_m<=.20;
  return r;
}
} // namespace

ScanToScanResult register_consecutive_scans(
    const Points& previous,const Points& current,
    Pose2 initial_previous_from_current,bool wide_initial_search,
    ScanRegistrationLimits limits) {
  ScanToScanResult best;
  if(!valid_cloud(previous)||!valid_cloud(current)||
     !initial_previous_from_current.translation.allFinite()||
     !std::isfinite(initial_previous_from_current.yaw_rad) ||
     !std::isfinite(limits.max_translation_m) ||
     !std::isfinite(limits.max_rotation_rad) ||
     limits.max_translation_m<=0 || limits.max_translation_m>3.0 ||
     limits.max_rotation_rad<=0 || limits.max_rotation_rad>.8)
    return best;
  // Candidate seeds differ only in yaw, not oracle motion. A smooth
  // constant-relative-pose estimate may be supplied from prior accepted
  // LiDAR-only registrations; identity is the initial bootstrap.
  const std::array<double,3> shifts{0.,-.12,.12};
  // A broad translation search is only needed without an accepted prior
  // relative LiDAR displacement (initial bootstrap or recovery after gap).
  // After that, three small yaw variants around last LiDAR-derived motion
  // avoid repeated high-cost multi-start optimization.
  const std::array<Eigen::Vector2d,5> offsets{
    Eigen::Vector2d::Zero(),Eigen::Vector2d{.2,0.},
    Eigen::Vector2d{-.2,0.},Eigen::Vector2d{0.,.2},
    Eigen::Vector2d{0.,-.2}};
  bool chosen=false;
  const std::size_t seeds=wide_initial_search?offsets.size():1;
  for(std::size_t k=0;k<seeds;++k)
    for(const double shift:shifts) {
    Pose2 pose=initial_previous_from_current;
    pose.translation+=offsets[k];
    pose.yaw_rad=wrap_yaw(pose.yaw_rad+shift);
    std::size_t iterations=0;
    for(;iterations<20;++iterations) {
      const auto matched=pairs(previous,current,pose);
      if(matched.size()<kMinMatches)break;
      const auto next=solve_pairs(matched);
      const double moved=(next.translation-pose.translation).norm();
      const double rotated=std::abs(wrap_yaw(next.yaw_rad-pose.yaw_rad));
      if(next.translation.norm()>limits.max_translation_m||
         std::abs(wrap_yaw(next.yaw_rad))>limits.max_rotation_rad)break;
      pose=next;
      if(moved<1e-5&&rotated<1e-5)break;
    }
    const auto r=score(previous,current,pose,iterations,limits);
    // Prefer the most reciprocal inliers, then minimum inlier error.
    if(!chosen||(r.accepted&&!best.accepted)||
       (r.accepted==best.accepted&&
        (r.forward_overlap+r.reverse_overlap>
           best.forward_overlap+best.reverse_overlap+1e-8||
         (std::abs(r.forward_overlap+r.reverse_overlap-
                   best.forward_overlap-best.reverse_overlap)<=1e-8&&
          r.mean_error_m<best.mean_error_m)))) {
      chosen=true;
      best=r;
    }
  }
  return best;
}
} // namespace slam::matching
