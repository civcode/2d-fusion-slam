#include "slam/matching/geometry.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

using namespace slam;
using namespace slam::mapping;
using namespace slam::matching;
using slam::sensors::ReturnKind;

namespace {
void require(bool ok,const char* why) {
  if (!ok) { std::cerr << "FAILED: " << why << '\n'; std::exit(1); }
}
}

int main() {
  GridConfig c;
  c.width=120; c.height=120; c.resolution_m=0.05;
  c.clear_trusted_no_returns=true;
  OccupancyGrid grid(c);
  std::vector<WorldRay> clear;
  for(int y=0; y<120; ++y) {
    const double yy=(y+0.5)*0.05;
    clear.push_back({{0.025,yy},{5.975,yy},ReturnKind::no_return});
  }
  (void)grid.integrate_rays(clear);
  // Non-symmetric 2D landmarks: sufficiently constrained XY and yaw.
  std::vector<Eigen::Vector2d> points;
  for (int i=0; i<35; ++i) {
    const int x=18+(i*37)%84, y=17+(i*53)%82;
    const Eigen::Vector2d p{(x+0.5)*0.05,(y+0.5)*0.05};
    points.push_back(p);
    (void)grid.integrate_rays({{{0.025,0.025},p,ReturnKind::hit}});
  }
  auto snap=MapSnapshot::capture(grid);
  auto field=DistanceField::build(snap,0.6);
  const Pose2 correct{{0,0},0};
  const Pose2 seed{{0.035,-0.03},0.014};
  const auto seed_quality=evaluate_geometry(*field,points,seed,{});
  require(seed_quality.supported_hits > 25 && seed_quality.full_rank,
          "synthetic map geometry is observable");
  const auto found=match_reference(field,points,seed);
  require(found.status==MatchStatus::accepted,"reference scan matching improved");
  require(found.diagnostics.source_revision==snap->revision() &&
          found.diagnostics.source_fingerprint==snap->fingerprint(),"source pinned");
  require(found.diagnostics.overlap>=0.65 && found.diagnostics.full_rank,"accepted rank and support");
  require(found.diagnostics.robust_cost < seed_quality.robust_cost,"geometry cost improved");
  require((found.map_from_laser.translation-correct.translation).norm()<0.025 &&
          std::abs(found.map_from_laser.yaw_rad)<0.008,"pose converges near ground truth");

#ifdef SLAM_HAS_CERES
  const auto ceres_found=match_ceres(field,points,seed);
  require(ceres_found.status==MatchStatus::accepted,"Ceres scan matching improved");
  require(ceres_found.diagnostics.source_fingerprint==snap->fingerprint(),"Ceres pinned source");
  require(ceres_found.diagnostics.overlap>=0.65 && ceres_found.diagnostics.full_rank,
          "Ceres geometry support and observability");
  require(ceres_found.diagnostics.robust_cost < seed_quality.robust_cost,
          "Ceres improves robust geometric cost");
  require((ceres_found.map_from_laser.translation-correct.translation).norm()<0.025 &&
          std::abs(ceres_found.map_from_laser.yaw_rad)<0.008,
          "Ceres converges near synthetic truth");
#endif

  std::vector<Eigen::Vector2d> impossible(20,{50.0,60.0});
  const auto reject=match_reference(field,impossible,{});
  require(reject.status==MatchStatus::insufficient_support &&
          reject.diagnostics.supported_hits==0 && reject.diagnostics.unsupported_hits==20,
          "unknown/outside never contributes zero residuals");
  const auto empty=evaluate_geometry(*DistanceField::build(MapSnapshot::capture(OccupancyGrid(c))),points,seed,{});
  require(empty.supported_hits==0 && !empty.full_rank,"all unknown map cannot match");
  std::cout<<"geometry-only match and quality checks passed\n";
}
