#include "slam/fixtures/synthetic.hpp"
#include "slam/mapping/occupancy.hpp"
#include "slam/sensors/scan.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <variant>
#include <vector>

using namespace slam;
using namespace slam::sensors;
using namespace slam::mapping;

namespace {
void check(bool good, const char* text) {
  if (!good) { std::cerr << "FAIL: " << text << '\n'; std::exit(1); }
}
BeamObservation measure(const fixtures::Beam& b) {
  return {b.acquired_at, b.angle_rad, b.range_m,
          b.hit ? ReturnKind::hit : ReturnKind::no_return};
}
bool approx(double a, double b, double tol = 1e-8) { return std::abs(a - b) <= tol; }
}

int main() {
  const Timestamp reference{1'000'000'000, 4, 7};
  const Pose2 stationary{{2.0, 2.0}, 0.0};
  const auto scene = fixtures::apartment();
  const auto fixture = fixtures::simulate_scan(scene, stationary, {}, {}, reference);

  ScanAssembler assembler;
  std::optional<AssembledScan> completed;
  for (const auto& b : fixture.beams) {
    if (auto scan = assembler.push(measure(b))) completed = std::move(scan);
  }
  check(!completed, "first revolution has no closure yet");
  const auto next = fixtures::simulate_scan(scene, stationary, {}, {},
                                             Timestamp{reference.ns + 100'000'000, 4, 7});
  completed = assembler.push(measure(next.beams.front()));
  check(completed.has_value(), "seam crossing finishes first revolution");
  check(!completed->partial && completed->ended_at_seam, "full revolution coverage");
  check(completed->beams.size() == 360 && completed->acquisition_coverage > 0.98,
        "every observed angular bin counted");
  check(completed->first_acquired.ns == reference.ns - 50'000'000, "first time");
  check(completed->last_acquired.ns == reference.ns + 50'000'000, "last time");
  check(completed->midpoint.ns == reference.ns, "scan midpoint from true beam times");
  check(assembler.flush()->partial, "trailing incomplete scan flushes partial");
  check(!assembler.flush(), "empty flush");

  // Missing sectors and explicit placeholders cannot become phantom acquisitions.
  ScanAssembler gap_assembler;
  for (std::size_t i = 0; i < fixture.beams.size(); ++i) {
    if (i == 90) {
      const auto ignored = gap_assembler.push({{}, 0, 0, ReturnKind::missing});
      check(!ignored, "missing placeholder never closes scan");
    }
    if (i >= 90 && i < 180) continue;
    if (i == 210) {
      auto invalid = measure(fixture.beams[i]);
      invalid.kind = ReturnKind::invalid;
      invalid.range_m = std::numeric_limits<double>::quiet_NaN();
      check(!gap_assembler.push(invalid), "invalid return keeps angle and time");
    } else check(!gap_assembler.push(measure(fixture.beams[i])), "gap input continues");
  }
  const auto gap = gap_assembler.push(measure(next.beams.front()));
  check(gap && gap->partial && gap->acquisition_coverage < 0.80, "missing quadrant partial");
  check(gap->largest_missing_arc_rad > 1.4, "largest unobserved angular arc");
  check(gap->missing_placeholders == 1 && gap->invalid_count == 1,
        "placeholder/invalid bookkeeping");
  check(gap->first_acquired.ns == fixture.beams.front().acquired_at.ns &&
        gap->last_acquired.ns == fixture.beams.back().acquired_at.ns,
        "missing does not extend timestamp bounds");

  // Clock domain/epoch/time discontinuity is explicitly rejected, no silent merge.
  ScanAssembler discontinuity;
  check(!discontinuity.push({reference, -std::numbers::pi, 1, ReturnKind::hit}), "first beam");
  bool rejected = false;
  try { (void)discontinuity.push({{reference.ns + 1, 4, 8}, -3, 1, ReturnKind::hit}); }
  catch (const std::invalid_argument&) { rejected = true; }
  check(rejected, "epoch discontinuity rejected");
  check(discontinuity.flush()->beams.size() == 1, "rejected beam not appended");
  ScanAssembler reverse;
  check(!reverse.push({reference, -2.0, 1, ReturnKind::hit}), "reverse initial");
  rejected = false;
  try { (void)reverse.push({{reference.ns + 1, 4, 7}, -2.7, 1, ReturnKind::hit}); }
  catch (const std::invalid_argument&) { rejected = true; }
  check(rejected, "reverse rotation without explicit direction rejected");
  ScanAssembler clockwise({0, RotationDirection::clockwise});
  check(!clockwise.push({reference, 0.3, 1, ReturnKind::hit}), "clockwise first");
  check(!clockwise.push({{reference.ns + 1, 4, 7}, 0.1, 1, ReturnKind::hit}),
        "clockwise progresses as expected");

  // Occupancy: unknown is distinct from known cells at log-odds zero.
  GridConfig grid_config;
  grid_config.width = 100; grid_config.height = 100; grid_config.resolution_m = 0.1;
  OccupancyGrid grid(grid_config);
  check(!grid.cell(90, 90)->observed && approx(grid.cell(90, 90)->log_odds, 0), "unknown baseline");
  check(!grid.cell(100, 100), "bounds checked");
  const std::vector<WorldRay> rays{
    {{1.05, 1.05}, {3.05, 1.05}, ReturnKind::hit},
    {{1.05, 1.05}, {5.05, 1.05}, ReturnKind::invalid},
    {{1.05, 1.05}, {5.05, 1.05}, ReturnKind::missing},
    {{1.05, 1.05}, {5.05, 1.05}, ReturnKind::no_return}};
  const auto stats = grid.integrate_rays(rays);
  check(stats.hit_cells == 1 && stats.free_cells >= 19 && stats.ignored_rays == 3,
        "only genuine hit ray changes map by default");
  check(grid.revision() == 1, "revision after actual insert");
  check(grid.at_world({3.05, 1.05})->log_odds > 0, "observed obstacle endpoint");
  check(grid.at_world({2.05, 1.05})->log_odds < 0, "observed free traversed cell");
  check(!grid.at_world({4.05, 1.05})->observed, "no clearing behind hit");
  check(!grid.at_world({8.05, 8.05})->observed, "untouched remains unknown");
  const auto ignored = grid.integrate_rays({{{0, 0}, {2, 2}, ReturnKind::missing}});
  check(ignored.ignored_rays == 1 && grid.revision() == 1, "missing does not change revision");

  // Same-scan hit precedence, independent of the ordering of the beams.
  OccupancyGrid precedence(grid_config);
  const WorldRay hit{{1.05, 2.05}, {3.05, 2.05}, ReturnKind::hit};
  const WorldRay passing{{1.05, 2.05}, {5.05, 2.05}, ReturnKind::hit};
  (void)precedence.integrate_rays({passing, hit});
  check(approx(precedence.at_world({3.05, 2.05})->log_odds, grid_config.hit_log_odds),
        "hit overrides free in one scan");
  OccupancyGrid reversed(grid_config);
  (void)reversed.integrate_rays({hit, passing});
  check(approx(reversed.at_world({3.05, 2.05})->log_odds,
               precedence.at_world({3.05, 2.05})->log_odds), "order-independent hit priority");

  GridConfig no_return_config = grid_config;
  no_return_config.clear_trusted_no_returns = true;
  OccupancyGrid trusted(no_return_config);
  (void)trusted.integrate_rays({{{1.05, 4.05}, {3.05, 4.05}, ReturnKind::no_return}});
  check(trusted.at_world({3.05, 4.05})->log_odds < 0, "explicit opt-in no-return clearing");

  // Clip rays to grid and reject occupied endpoints outside it.
  OccupancyGrid clipping(grid_config);
  const auto clipped = clipping.integrate_rays({{{-2, 5.05}, {15, 5.05}, ReturnKind::hit}});
  check(clipped.hit_cells == 0 && clipped.free_cells == 100, "out-of-map ray only clears intersected grid");
  check(!clipping.at_world({10.0, 5.05}), "map upper bound exclusive");

  // Rotated grid origin transforms world-space inputs correctly.
  grid_config.map_from_grid = {{10, -5}, std::numbers::pi / 2};
  OccupancyGrid rotated(grid_config);
  (void)rotated.integrate_rays({{{9.95, -4.95}, {9.95, -3.95}, ReturnKind::hit}});
  check(rotated.cell(10, 0)->log_odds > 0, "rotated occupied endpoint cell");
  check(rotated.cell(1, 0)->log_odds < 0, "rotated free ray origin cell");

  // Use the existing synthetic scene for a real 360-ray static mapping pass.
  std::vector<WorldRay> world_rays;
  for (const auto& beam : completed->beams) {
    const auto direction = Eigen::Vector2d{std::cos(beam.angle_rad), std::sin(beam.angle_rad)};
    world_rays.push_back({stationary.translation,
      stationary.translation + beam.range_m * direction,
      beam.kind});
  }
  // Map this fixture in an unrotated frame (motion is exactly zero).
  GridConfig fixture_grid;
  fixture_grid.width = 300; fixture_grid.height = 240; fixture_grid.resolution_m = 0.05;
  OccupancyGrid mapped(fixture_grid);
  const auto inserted = mapped.integrate_rays(world_rays);
  check(inserted.hit_cells > 0 && inserted.free_cells > inserted.hit_cells,
        "synthetic fixture produces known free space and occupied boundaries");
  check(mapped.at_world({0.0, 2.0})->observed, "apartment exterior wall mapped");
  check(!mapped.at_world({14.95, 11.95})->observed, "unseen apartment far corner remains unknown");

  std::cout << "scan assembler / occupancy mapping tests passed\n";
}
