#include "slam/core/se2.hpp"
#include "slam/core/time.hpp"

#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

namespace slam {
namespace {
[[nodiscard]] bool checked_add(TimeNs a, TimeNs b, TimeNs& result) noexcept {
  constexpr auto min = std::numeric_limits<TimeNs>::min();
  constexpr auto max = std::numeric_limits<TimeNs>::max();
  if ((b > 0 && a > max - b) || (b < 0 && a < min - b)) return false;
  result = a + b;
  return true;
}

[[nodiscard]] bool checked_sub(TimeNs a, TimeNs b, TimeNs& result) noexcept {
  constexpr auto min = std::numeric_limits<TimeNs>::min();
  constexpr auto max = std::numeric_limits<TimeNs>::max();
  if ((b < 0 && a > max + b) || (b > 0 && a < min + b)) return false;
  result = a - b;
  return true;
}

[[nodiscard]] std::pair<double, double> se2_ab(double angle) noexcept {
  if (std::abs(angle) < 1e-5) {
    const double t2 = angle * angle;
    return {1.0 - t2 / 6.0 + t2 * t2 / 120.0,
            angle * (0.5 - t2 / 24.0 + t2 * t2 / 720.0)};
  }
  return {std::sin(angle) / angle, (1.0 - std::cos(angle)) / angle};
}
}  // namespace

DurationResult between(Timestamp later, Timestamp earlier) noexcept {
  if (later.clock_domain != earlier.clock_domain) return TimeError::different_domain;
  if (later.epoch != earlier.epoch) return TimeError::different_epoch;
  TimeNs ns{};
  if (!checked_sub(later.ns, earlier.ns, ns)) return TimeError::overflow;
  return DurationNs{ns};
}

TimestampResult advance(Timestamp origin, DurationNs delta) noexcept {
  TimeNs ns{};
  if (!checked_add(origin.ns, delta.value, ns)) return TimeError::overflow;
  origin.ns = ns;
  return origin;
}

double seconds(DurationNs duration) noexcept { return static_cast<double>(duration.value) * 1e-9; }

double wrap_yaw(double radians) noexcept {
  const double v = std::remainder(radians, 2.0 * std::numbers::pi);
  return v == -std::numbers::pi ? std::numbers::pi : v;
}

Eigen::Vector2d transform(const Pose2& pose, const Eigen::Vector2d& point) noexcept {
  const double c = std::cos(pose.yaw_rad), s = std::sin(pose.yaw_rad);
  return {pose.translation.x() + c * point.x() - s * point.y(),
          pose.translation.y() + s * point.x() + c * point.y()};
}

Pose2 compose(const Pose2& a, const Pose2& b) noexcept {
  return {transform(a, b.translation), wrap_yaw(a.yaw_rad + b.yaw_rad)};
}

Pose2 inverse(const Pose2& pose) noexcept {
  const double c = std::cos(pose.yaw_rad), s = std::sin(pose.yaw_rad);
  const auto& t = pose.translation;
  return {{-c * t.x() - s * t.y(), s * t.x() - c * t.y()}, wrap_yaw(-pose.yaw_rad)};
}

Pose2 exp_se2(Tangent2 displacement) noexcept {
  const auto [a, b] = se2_ab(displacement.dyaw);
  return {{a * displacement.dx - b * displacement.dy,
           b * displacement.dx + a * displacement.dy}, wrap_yaw(displacement.dyaw)};
}

Tangent2 log_se2(const Pose2& pose) noexcept {
  const double angle = wrap_yaw(pose.yaw_rad);
  const auto [a, b] = se2_ab(angle);
  const double denom = a * a + b * b;
  const auto& t = pose.translation;
  return {(a * t.x() + b * t.y()) / denom,
          (-b * t.x() + a * t.y()) / denom, angle};
}

Pose2 integrate_body_twist(const Pose2& initial, Tangent2 twist, double dt) noexcept {
  return compose(initial, exp_se2({twist.dx * dt, twist.dy * dt, twist.dyaw * dt}));
}
}  // namespace slam
