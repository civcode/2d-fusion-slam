#pragma once

#include <Eigen/Core>

namespace slam {
struct Pose2 {
  Eigen::Vector2d translation{Eigen::Vector2d::Zero()};
  double yaw_rad{};
};

// Integrated body-frame displacement over an interval (metres, metres, radians).
struct Tangent2 {
  double dx{};
  double dy{};
  double dyaw{};
};

[[nodiscard]] double wrap_yaw(double radians) noexcept;
[[nodiscard]] Pose2 compose(const Pose2& a, const Pose2& b) noexcept;
[[nodiscard]] Pose2 inverse(const Pose2& pose) noexcept;
[[nodiscard]] Eigen::Vector2d transform(const Pose2& pose, const Eigen::Vector2d& point) noexcept;
[[nodiscard]] Pose2 exp_se2(Tangent2 displacement) noexcept;
[[nodiscard]] Tangent2 log_se2(const Pose2& pose) noexcept;
[[nodiscard]] Pose2 integrate_body_twist(const Pose2& initial, Tangent2 twist_per_second,
                                         double duration_seconds) noexcept;
}  // namespace slam
