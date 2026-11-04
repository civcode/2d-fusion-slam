#include "slam/matching/scan_to_scan.hpp"

#include <stdexcept>
#include <cmath>
#include <vector>

void check(bool condition) {
  if(!condition)throw std::runtime_error("scan-to-scan registration regression");
}
int main() {
  using slam::Pose2;
  using slam::matching::register_consecutive_scans;
  std::vector<Eigen::Vector2d> previous;
  // Asymmetric 2D walls/corners, not a degenerate single line.
  for(int i=0;i<70;++i) {
    const double u=double(i)/69.;
    previous.emplace_back(1.+6.*u,1.);
    previous.emplace_back(7.,1.+4.*u);
    previous.emplace_back(1.+2.5*u,5.);
    previous.emplace_back(2.,2.+3.*u);
  }
  const Pose2 expected{{.24,-.14},.075};
  const auto current_from_previous=slam::inverse(expected);
  std::vector<Eigen::Vector2d> current;
  for(std::size_t i=0;i<previous.size();++i) {
    if(i%13==0)continue; // deterministic view occlusion
    const auto p=slam::transform(current_from_previous,previous[i]);
    current.emplace_back(p.x()+.002*std::sin(double(i)),
                         p.y()+.002*std::cos(double(i)));
  }
  const auto first=register_consecutive_scans(previous,current);
  check(first.accepted);
  check(first.correspondence_rank);
  check(first.forward_overlap>=.65&&first.reverse_overlap>=.65);
  check((first.previous_from_current.translation-expected.translation).norm()<.09);
  check(std::abs(slam::wrap_yaw(
      first.previous_from_current.yaw_rad-expected.yaw_rad))<.035);
  const auto again=register_consecutive_scans(previous,current);
  check(again.accepted);
  check((again.previous_from_current.translation-first.previous_from_current.translation).norm()<1e-12);
  check(again.previous_from_current.yaw_rad==first.previous_from_current.yaw_rad);
  // Loop candidates sometimes see the same geometry from farther apart
  // than consecutive frames. Only explicit loop calls may widen the
  // displacement envelope; ordinary odometry ICP must remain bounded.
  const Pose2 farther{{1.55,.42},.16};
  std::vector<Eigen::Vector2d> revisited;
  for(std::size_t i=0;i<previous.size();++i) {
    if(i%17==0)continue;
    revisited.push_back(slam::transform(slam::inverse(farther),previous[i]));
  }
  const auto narrow=register_consecutive_scans(
      previous,revisited,farther,false);
  check(!narrow.accepted);
  const auto wide=register_consecutive_scans(
      previous,revisited,farther,false,
      {.max_translation_m=3.0,.max_rotation_rad=.6});
  check(wide.accepted&&wide.correspondence_rank);
  check(wide.forward_overlap>.82&&wide.reverse_overlap>.82);
  check((wide.previous_from_current.translation-farther.translation).norm()<.07);
  check(std::abs(slam::wrap_yaw(
      wide.previous_from_current.yaw_rad-farther.yaw_rad))<.04);
  // An opposite-direction revisit can have arbitrary absolute heading,
  // provided its *correction from the supplied SE(2) prior* is small.
  const Pose2 reverse_pose{{.28,.10},slam::wrap_yaw(3.14159265358979323846-.06)};
  std::vector<Eigen::Vector2d> opposite;
  for(std::size_t i=0;i<previous.size();++i) {
    if(i%19==0)continue;
    const auto p=slam::transform(slam::inverse(reverse_pose),previous[i]);
    opposite.emplace_back(p.x()+.001*std::sin(double(i)),
                          p.y()+.001*std::cos(double(i)));
  }
  const Pose2 reverse_prior{{.25,.08},
       slam::wrap_yaw(reverse_pose.yaw_rad-.035)};
  const auto legacy_reverse=register_consecutive_scans(
       previous,opposite,reverse_prior,false,{3.0,.7});
  check(!legacy_reverse.accepted);
  const auto pose_prior_reverse=slam::matching::register_scans_from_pose_prior(
       previous,opposite,reverse_prior,false);
  check(pose_prior_reverse.accepted && pose_prior_reverse.correspondence_rank);
  check(pose_prior_reverse.forward_overlap>.82 &&
        pose_prior_reverse.reverse_overlap>.82);
  // Discrete wall points can permit a small longitudinal ICP ambiguity;
  // this tests large absolute heading plus bounded correction, not an
  // unrealistically perfect corridor localization requirement.
  check((pose_prior_reverse.previous_from_current.translation-
         reverse_pose.translation).norm()<.20);
  check(std::abs(slam::wrap_yaw(pose_prior_reverse.previous_from_current.yaw_rad-
                                reverse_pose.yaw_rad))<.05);
  const auto repeated=slam::matching::register_scans_from_pose_prior(
       previous,opposite,reverse_prior,false);
  check(repeated.accepted);
  check((repeated.previous_from_current.translation-
         pose_prior_reverse.previous_from_current.translation).norm()<1e-12);
  const Pose2 bad_prior{{2.8,2.8},-1.5};
  check(!slam::matching::register_scans_from_pose_prior(
       previous,opposite,bad_prior,false).accepted);
  const auto invalid_correction=slam::matching::register_scans_from_pose_prior(
       previous,opposite,reverse_prior,false,{3.0,2.0,.17});
  check(!invalid_correction.accepted);
  const auto invalid_limits=register_consecutive_scans(
      previous,revisited,farther,false,
      {.max_translation_m=10.,.max_rotation_rad=.6});
  check(!invalid_limits.accepted);
  // A single wall is geometrically insufficient for reliable 2D tracking.
  std::vector<Eigen::Vector2d> line;
  for(int i=0;i<100;++i)line.emplace_back(double(i)*.04,0.);
  check(!register_consecutive_scans(line,line).accepted);
  // Unrelated scans must not be inserted merely by minimizing partial costs.
  for(auto& p:current)p+=Eigen::Vector2d(5.,4.);
  check(!register_consecutive_scans(previous,current).accepted);
  return 0;
}
