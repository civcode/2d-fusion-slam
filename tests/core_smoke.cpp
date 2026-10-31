#include "slam/core/se2.hpp"
#include "slam/core/time.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numbers>
#include <variant>

namespace {
void expect(bool ok, const char* msg) {
  if (!ok) { std::cerr << "FAIL: " << msg << '\n'; std::exit(1); }
}
bool close(double a, double b, double tol = 1e-10) { return std::abs(a - b) < tol; }
}
int main() {
  using namespace slam;
  constexpr auto max = std::numeric_limits<TimeNs>::max();
  constexpr auto min = std::numeric_limits<TimeNs>::min();
  const Timestamp t{100, 3, 7};
  expect(std::get<DurationNs>(between(t, Timestamp{40, 3, 7})).value == 60, "time diff");
  expect(std::get<TimeError>(between(t, Timestamp{40, 4, 7})) == TimeError::different_domain, "clock domain");
  expect(std::get<TimeError>(between(t, Timestamp{40, 3, 8})) == TimeError::different_epoch, "clock epoch");
  expect(std::get<TimeError>(between(Timestamp{max, 3, 7}, Timestamp{min, 3, 7})) == TimeError::overflow, "time subtraction overflow");
  expect(std::get<TimeError>(advance(Timestamp{max, 3, 7}, DurationNs{1})) == TimeError::overflow, "time addition overflow");
  expect(std::get<Timestamp>(advance(t, DurationNs{-30})).ns == 70, "time addition");
  expect(close(seconds(DurationNs{1'000'000'000}), 1.0), "nanoseconds to seconds");
  const Pose2 a{{1.0, -2.0}, 0.7}, b{{3.0, 0.3}, -0.4};
  const Pose2 unity = compose(a, inverse(a));
  expect(unity.translation.norm() < 1e-11 && close(unity.yaw_rad, 0), "inverse identity");
  const Pose2 c = compose(a, b);
  expect((transform(a, transform(b, {2.0, -3.0})) - transform(c, {2.0, -3.0})).norm() < 1e-10, "SE2 composition");
  for (const Tangent2 xi : {Tangent2{1, -0.3, 0}, Tangent2{1, 2, 1e-8}, Tangent2{2, -0.4, 1.2}}) {
    const auto v = log_se2(exp_se2(xi));
    expect(close(xi.dx, v.dx) && close(xi.dy, v.dy) && close(xi.dyaw, v.dyaw), "exp/log roundtrip");
  }
  const Pose2 rot = integrate_body_twist({}, {1, 0, std::numbers::pi}, 1.0);
  expect(close(rot.translation.x(), 0, 1e-9) && close(rot.translation.y(), 2 / std::numbers::pi, 1e-9), "arc integration");
  std::cout << "slam_core: time and SE(2) checks passed\n";
}
