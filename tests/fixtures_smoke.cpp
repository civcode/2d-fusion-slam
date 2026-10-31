#include "slam/fixtures/synthetic.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <variant>

void require(bool condition, const char* why) {
  if (!condition) { std::cerr << "FAIL: " << why << '\n'; std::exit(1); }
}
int main() {
  using namespace slam;
  using namespace slam::fixtures;
  const auto ap = apartment();
  const auto wh = warehouse();
  require(ap.id == "synthetic-apartment-v1" && wh.id == "synthetic-warehouse-v1", "scene names");
  require(wh.walls.size() > ap.walls.size(), "repeat shelving exists");
  const Timestamp ref{2'000'000'000, 1, 1};
  const Pose2 robot{{2.0, 2.0}, 0.0};
  const auto a = simulate_scan(ap, robot, {}, {}, ref, 360);
  const auto b = simulate_scan(ap, robot, {}, {}, ref, 360);
  require(a.beams.size() == 360, "beam count");
  require(a.beams[0].acquired_at.ns == ref.ns - 50'000'000, "first beam time");
  require(a.beams.back().acquired_at.ns == ref.ns + 50'000'000, "last beam time");
  require(a.beams[0].hit && std::abs(a.beams[0].range_m - 2.0) < 1e-10, "analytic left wall");
  for (std::size_t i = 0; i < a.beams.size(); ++i) {
    require(a.beams[i].acquired_at.ns == b.beams[i].acquired_at.ns &&
      a.beams[i].range_m == b.beams[i].range_m, "deterministic exact beams");
    require(a.beams[i].range_m > 0 && a.beams[i].range_m <= 30, "valid ranges");
  }
  const auto moving = simulate_scan(ap, robot, {0.5, 0, 1.0}, {}, ref, 360);
  require(std::abs(moving.beams[0].range_m - a.beams[0].range_m) > 1e-4, "moving ray acquisition");
  const auto short_range = simulate_scan(ap, robot, {}, {}, ref, 360, 100'000'000, 0.3);
  require(!short_range.beams[0].hit && short_range.beams[0].range_m == 0.3, "no-return distinguished");
  try { (void)simulate_scan(ap, robot, {}, {}, ref, 1); require(false, "reject invalid beam count"); }
  catch (const std::invalid_argument&) {}
  std::cout << "synthetic apartment and warehouse fixture checks passed\n";
}
