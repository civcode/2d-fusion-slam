#include "slam/matching/scan_to_scan.hpp"

#include <cassert>
#include <cmath>
#include <vector>

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
  assert(first.accepted);
  assert(first.forward_overlap>=.65&&first.reverse_overlap>=.65);
  assert((first.previous_from_current.translation-expected.translation).norm()<.09);
  assert(std::abs(slam::wrap_yaw(
      first.previous_from_current.yaw_rad-expected.yaw_rad))<.035);
  const auto again=register_consecutive_scans(previous,current);
  assert(again.accepted);
  assert(again.previous_from_current.translation==first.previous_from_current.translation);
  assert(again.previous_from_current.yaw_rad==first.previous_from_current.yaw_rad);
  // A single wall is geometrically insufficient for reliable 2D tracking.
  std::vector<Eigen::Vector2d> line;
  for(int i=0;i<100;++i)line.emplace_back(double(i)*.04,0.);
  assert(!register_consecutive_scans(line,line).accepted);
  // Unrelated scans must not be inserted merely by minimizing partial costs.
  for(auto& p:current)p+=Eigen::Vector2d(5.,4.);
  assert(!register_consecutive_scans(previous,current).accepted);
  return 0;
}
