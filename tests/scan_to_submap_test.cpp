#include "slam/matching/scan_to_submap.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace {
void check(bool yes) {
  if(!yes)throw std::runtime_error("scan-to-submap registration regression");
}
std::vector<Eigen::Vector2d> corridor(bool distinctive) {
  std::vector<Eigen::Vector2d> world;
  for(int k=0;k<=180;++k) {
    const double x=-9.+18.*k/180.;
    world.emplace_back(x,-2.);
    world.emplace_back(x,2.);
  }
  if(distinctive) {
    // Distinctive transverse structure supplies the longitudinal
    // observability that a uniform straight corridor lacks.
    for(int k=0;k<=100;++k) {
      const double y=-2.+4.*k/100.;
      world.emplace_back(-1.,y);
      world.emplace_back(3.4,y*.68);
    }
    for(int k=0;k<=40;++k) {
      const double u=double(k)/40.;
      world.emplace_back(-3.4+.6*u,3.0+.7*u);
    }
  }
  return world;
}
std::vector<Eigen::Vector2d> half_scan(
    const std::vector<Eigen::Vector2d>& world,slam::Pose2 pose) {
  const auto inv=slam::inverse(pose);
  std::vector<Eigen::Vector2d> hits;
  for(const auto& p:world) {
    const auto local=slam::transform(inv,p);
    if(local.x()>.01 && local.norm()<=13.)hits.push_back(local);
  }
  return hits;
}
} // namespace

int main() {
  using slam::matching::SubmapSweep;
  using slam::matching::build_frozen_lidar_submap;
  using slam::matching::register_scan_to_frozen_submap;
  const slam::Pose2 reverse_truth{{.25,0.},3.14159265358979323846};
  const slam::Pose2 reverse_prior{{.34,-.04},
                                   3.14159265358979323846-.04};
  for(bool distinctive:{true,false}) {
    const auto world=corridor(distinctive);
    std::vector<SubmapSweep> sweeps;
    // Historical forward-facing scans, each with only 180-degree coverage.
    for(int k=-10;k<=10;++k) {
      const slam::Pose2 pose{{double(k)*.45,0.},0.};
      sweeps.push_back({pose,half_scan(world,pose)});
    }
    const auto submap=build_frozen_lidar_submap(sweeps);
    check(submap.valid() && submap.contributing_scans==21);
    const auto fingerprint=slam::matching::describe_frozen_lidar_submap(submap);
    check(fingerprint.valid);
    check(std::abs(slam::matching::submap_descriptor_similarity(
        fingerprint,fingerprint)-1.)<1e-10);
    check(slam::matching::submap_descriptor_similarity(
        fingerprint,{})==0.);
    // Descriptor generation and ranking must be deterministic.
    const auto again=slam::matching::describe_frozen_lidar_submap(submap);
    check(fingerprint.pair_distance_histogram==again.pair_distance_histogram);
    const auto current=half_scan(world,reverse_truth);
    const auto first=register_scan_to_frozen_submap(
        submap,current,reverse_prior);
    const auto repeat=register_scan_to_frozen_submap(
        submap,current,reverse_prior);
    check(first.accepted==repeat.accepted);
    check(first.correlation_score==repeat.correlation_score);
    if(distinctive) {
      check(first.accepted && first.observable);
      check(first.supported_hits>=80);
      check(first.support_fraction>=.58);
      check((first.anchor_from_current.translation-
             reverse_truth.translation).norm()<.4);
      check(std::abs(slam::wrap_yaw(first.anchor_from_current.yaw_rad-
                                    reverse_truth.yaw_rad))<.08);
      check(!register_scan_to_frozen_submap(
          submap,current,{{10.,10.},1.}).accepted);
      check(!register_scan_to_frozen_submap(
          submap,current,reverse_prior,{.54,.18,.58,.48,.08,.12,5000}).accepted);
      // A large initial XY error needs the separate global-search budget;
      // the original local matcher must stay bounded.
      const slam::Pose2 drifted_prior{{3.2,-.05},
                                     reverse_truth.yaw_rad-.18};
      check(!register_scan_to_frozen_submap(
          submap,current,drifted_prior).accepted);
      const auto global=register_scan_to_frozen_submap(
          submap,current,drifted_prior,
          {4.5,.55,.58,.48,.08,.12,80,true});
      check(global.correlation_score>.40);
      check(global.innovation_m>.54);
      check(global.supported_hits>=80);
    } else {
      check(!first.accepted);
      check(!first.observable);
      check(first.translation_curvature_min<.08 ||
            first.translation_curvature_ratio<.12 ||
            first.minimum_shift_score_drop<.01);
    }
  }
  return 0;
}
