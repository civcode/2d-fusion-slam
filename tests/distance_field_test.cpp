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
  const auto empty_cost=empty_field->sample_matching_cost({1.5,1.5});
  require(close(empty_cost.metres,empty_field->truncation_metres()) &&
          close(empty_cost.gradient_map.norm(),0),
          "empty map is capped and has no invented wall gradient");
  require(empty_field->support_map({1.5, 1.5})==DistanceSupport::no_obstacles,
          "empty EDT classified without fabricated data");
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
  const auto continuous_known=field->sample_matching_cost({2.10,2.05});
  require(close(continuous_known.metres,sample->metres,1e-12) &&
          (continuous_known.gradient_map-sample->gradient_map).norm()<1e-12,
          "finite and strict fields match exactly where strict support exists");
  require(!field->sample_map({-1,0}) && !field->sample_map({4.0,2}), "outside not zero residual");
  require(field->support_map({-1,0})==DistanceSupport::outside_grid,
          "out of bounds classified");
  require(field->support_map({2.10,2.05})==DistanceSupport::supported,
          "valid interpolation classified");
  require(field->supports_disk_map({2.10,2.05},0.02),
          "small disk around observed geometry is certified");
  require(!field->supports_disk_map({2.10,2.05},3.0),
          "support region extending beyond map is not certified");
  require(!field->supports_disk_map({2.10,2.05},-1.0),
          "negative support radius is rejected");
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
  require(partial->support_map({3.15,3.15})==DistanceSupport::unknown_cells,
          "unknown stencil classified");
  require(!partial->supports_disk_map({3.15,3.15},0.01),
          "unknown wedges cannot satisfy support certificate");
  require(!partial->sample_map({2.05,2.05}), "even hit needs fully known bilinear support");
  const auto unknown_cost=partial->sample_matching_cost({3.15,3.15});
  require(close(unknown_cost.metres,partial->truncation_metres()) &&
          close(unknown_cost.gradient_map.norm(),0),
          "unknown space is maximum cost with no false wall attraction");
  const auto mixed=partial->sample_matching_cost({2.05,2.05});
  require(mixed.metres>=0&&mixed.metres<=partial->truncation_metres() &&
          mixed.gradient_map.allFinite(),
          "unknown-adjacent wall is finitely interpolated");

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
  const auto rotated_cost=rotfield->sample_matching_cost(
      transform(config.map_from_grid,{2.10,2.05}));
  require(close(rotated_cost.metres,rot_sample->metres,1e-8) &&
          (rotated_cost.gradient_map-rot_sample->gradient_map).norm()<1e-8,
          "finite matcher gradient rotates correctly into world coordinates");
  auto truncated=DistanceField::build(snapshot,0.20);
  require(close(*truncated->cell_distance(0,0),0.20), "truncation cap");
  require(!truncated->sample_map({0.5,0.5}), "truncated plateau excluded");
  require(close(truncated->sample_matching_cost({0.5,0.5}).metres,0.20),
          "truncated observed cells have finite maximum cost");
  for(const Eigen::Vector2d query:std::vector<Eigen::Vector2d>{
      {-100.,-100.},{-0.04,2.05},{0.01,2.05},{3.99,2.05},{200.,200.}}) {
    const auto cost=truncated->sample_matching_cost(query);
    require(std::isfinite(cost.metres)&&cost.metres>=0&&cost.metres<=0.20 &&
            cost.gradient_map.allFinite(),
            "off-grid costs stay finite, capped, and differentiable");
  }
  const double h=1e-5;
  const auto near_edge=truncated->sample_matching_cost({0.04,2.05});
  const double numerical_x=
      (truncated->sample_matching_cost({0.04+h,2.05}).metres-
       truncated->sample_matching_cost({0.04-h,2.05}).metres)/(2*h);
  require(close(near_edge.gradient_map.x(),numerical_x,1e-6),
          "interpolated border gradient agrees with finite difference");
  require(truncated->support_map({0.5,0.5})==DistanceSupport::truncated,
          "truncation boundary classified");
  require(!truncated->supports_disk_map({0.5,0.5},0.0),
          "truncated area cannot satisfy support certificate");
  require(truncated->support_map({2.10,2.05})==DistanceSupport::supported,
          "near-wall interpolation remains supported");
  std::cout << "immutable snapshot and exact Euclidean EDT checks passed\n";
}
