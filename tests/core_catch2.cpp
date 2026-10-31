#include "slam/core/se2.hpp"
#include "slam/core/time.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <limits>
#include <numbers>
#include <variant>

using Catch::Approx;
TEST_CASE("different clock epochs and domains are not comparable") {
  using namespace slam;
  const Timestamp t{1, 7, 9};
  REQUIRE(std::get<TimeError>(between(t, {0, 8, 9})) == TimeError::different_domain);
  REQUIRE(std::get<TimeError>(between(t, {0, 7, 8})) == TimeError::different_epoch);
  REQUIRE(std::get<TimeError>(between({std::numeric_limits<TimeNs>::max(), 7, 9},
                                     {std::numeric_limits<TimeNs>::min(), 7, 9})) == TimeError::overflow);
}
TEST_CASE("SE2 exponential preserves body-frame arc and inverse") {
  using namespace slam;
  const auto p = exp_se2({1, 0, std::numbers::pi});
  REQUIRE(p.translation.x() == Approx(0).margin(1e-12));
  REQUIRE(p.translation.y() == Approx(2 / std::numbers::pi));
  const auto restored = log_se2(p);
  REQUIRE(restored.dx == Approx(1));
  REQUIRE(restored.dy == Approx(0).margin(1e-12));
  const auto identity = compose(p, inverse(p));
  REQUIRE(identity.translation.norm() == Approx(0).margin(1e-12));
}
