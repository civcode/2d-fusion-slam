#include "slam/mapping/distance_field.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <numbers>
#include <vector>

using namespace slam;
using namespace slam::mapping;
using slam::sensors::ReturnKind;

namespace {
void require(bool value, const char* message) {
  if (!value) { std::cerr << "FAILED: " << message << '\n'; std::exit(1); }
}
bool close(double a, double b, double tolerance=1e-9) {
  return std::abs(a - b) < tolerance;
}
}

int main() {
  GridConfig config;
  config.width = 40; config.height = 40; config.resolution_m = 0.1;
  config.clear_trusted_no_returns = true; // synthetic fixture declares trusted no-return
  OccupancyGrid grid(config);
  auto empty = MapSnapshot::capture(grid);
  auto empty_field = DistanceField::build(empty);
  require(!empty_field->has_obstacles(), "no occupied cells means no EDT support");
  require(!empty_field->sample_map({1.5, 1.5}), "empty map no false residual");
  require(empty->revision() == 0, "empty revision");
  std::vector<WorldRay> clear;
  for (int y = 0; y < 40; ++y) {
    const double yy = (y + 0.5) * 0.1;
    clear.push_back({{0.05, yy}, {3.95, yy}, ReturnKind::no_return});
  }
  (void)grid.integrate_rays(clear);
  (void)grid.integrate_rays({{{0.05, 2.05}, {2.05, 2.05}, ReturnKind::hit}});
  auto snapshot = MapSnapshot::capture(grid);
  require(snapshot->revision() == 2, "snapshot authoritative revision");
  require(snapshot->cell(20,20)->observed && snapshot->cell(20,20)->log_odds > 0,
          "obstacle in known cell");
  auto field = DistanceField::build(snapshot, 5.0);
  require(field->source().get() == snapshot.get(), "field pins exact immutable source");
  require(field->has_obstacles(), "known obstacle seeds EDT");
  for (int y = 0; y < 40; ++y) {
    for (int x = 0; x < 40; ++x) {
      const double expected = std::hypot(x - 20.0, y - 20.0) * 0.1;
      require(close(*field->cell_distance(x,y), expected, 1e-8), "exact Euclidean distances on grid");
    }
  }
  const auto sample = field->sample_map({2.10,2.05});
  require(sample && close(sample->metres, 0.05, 1e-8), "bilinear centre interpolation");
  require(close(sample->gradient_map.x(), 1.0, 1e-8), "bilinear analytical gradient");
  require(!field->sample_map({-1,0}) && !field->sample_map({4.0,2}), "outside not zero residual");
  const auto original = snapshot->fingerprint();
  (void)grid.integrate_rays({{{0.05,3.05},{1.05,3.05},ReturnKind::hit}});
  require(snapshot->revision() == 2 && snapshot->fingerprint() == original,
          "immutable snapshot stays pinned after source mutation");
  const auto changed = MapSnapshot::capture(grid);
  require(changed->fingerprint() != original && changed->revision() == 3,
          "new publication has different content and revision");
  require(close(*field->cell_distance(10,30), std::hypot(10.,10.) * 0.1),
          "old EDT not silently mixed with new occupancy");

  OccupancyGrid unknown(config);
  (void)unknown.integrate_rays({{{0.05,2.05},{2.05,2.05},ReturnKind::hit}});
  auto partial = DistanceField::build(MapSnapshot::capture(unknown),5.0);
  require(partial->has_obstacles(), "partial map has occupied seed");
  require(!partial->sample_map({3.15,3.15}), "unknown stencil cannot match");
  require(!partial->sample_map({2.05,2.05}), "even hit needs fully known bilinear support");

  // Test rotated map gradient in the world frame and capped support.
  config.map_from_grid = {{10,-5}, std::numbers::pi/2};
  OccupancyGrid rotated(config);
  std::vector<WorldRay> rotated_clear;
  for (int y=0; y<40; ++y) {
    const double yy = (y+0.5)*0.1;
    rotated_clear.push_back({transform(config.map_from_grid,{0.05,yy}),
                             transform(config.map_from_grid,{3.95,yy}),ReturnKind::no_return});
  }
  (void)rotated.integrate_rays(rotated_clear);
  (void)rotated.integrate_rays({{transform(config.map_from_grid,{0.05,2.05}),
                                transform(config.map_from_grid,{2.05,2.05}),ReturnKind::hit}});
  auto rotfield = DistanceField::build(MapSnapshot::capture(rotated),5.0);
  const auto rot_sample = rotfield->sample_map(transform(config.map_from_grid,{2.10,2.05}));
  require(rot_sample && close(rot_sample->metres,0.05,1e-7), "rotated distances");
  require(close(rot_sample->gradient_map.x(),-sample->gradient_map.y(),1e-7) &&
          close(rot_sample->gradient_map.y(),sample->gradient_map.x(),1e-7), "rotated gradient in world frame");
  auto truncated=DistanceField::build(snapshot,0.20);
  require(close(*truncated->cell_distance(0,0),0.20), "truncation cap");
  require(!truncated->sample_map({0.5,0.5}), "truncated plateau excluded");
  std::cout << "immutable snapshot and exact Euclidean EDT checks passed\n";
}
