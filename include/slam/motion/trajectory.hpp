#pragma once

#include "slam/core/se2.hpp"
#include "slam/core/time.hpp"
#include "slam/sensors/scan.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace slam::motion {

enum class TrajectoryCapability { gyro_only, full_se2 };
struct TimedPose {
  Timestamp time;
  Pose2 odom_from_base;
};

// A simple, bounded interpolation-only trajectory. Samples have a common
// clock domain and epoch and must be strictly increasing; no extrapolation.
// Individual intervals interpolate via the SE(2) exponential, not by
// independently interpolating XY and yaw.
class SampledTrajectory {
 public:
  SampledTrajectory(std::vector<TimedPose> samples, TrajectoryCapability capability);
  [[nodiscard]] std::optional<Pose2> at(Timestamp t) const noexcept;
  [[nodiscard]] TrajectoryCapability capability() const noexcept { return capability_; }
  [[nodiscard]] Timestamp start() const noexcept { return samples_.front().time; }
  [[nodiscard]] Timestamp end() const noexcept { return samples_.back().time; }

 private:
  std::vector<TimedPose> samples_;
  TrajectoryCapability capability_;
};

struct DeskewedHit {
  Eigen::Vector2d point_laser_reference{};
  Timestamp acquired_at{};
};
struct DeskewedScan {
  Timestamp reference_time{};
  std::vector<DeskewedHit> hits; // returns: only physically valid endpoints
  std::size_t skipped_non_hits{};
  TrajectoryCapability capability{TrajectoryCapability::gyro_only};
  bool physical_beam_time_verified{};
  // No-wheel gyro-only interpolation does NOT justify full XY mapping deskew.
  [[nodiscard]] bool suitable_for_moving_map_insertion() const noexcept {
    return capability == TrajectoryCapability::full_se2 && physical_beam_time_verified;
  }
};

// Uses exact scan midpoint and lidar mounting translation + yaw lever arm.
// Returns nullopt for any unsupported timestamp/clock interval or if scan
// timestamps are provisional. For diagnostic synthetic comparisons callers
// may explicitly allow provisional time but the result remains untrusted.
[[nodiscard]] std::optional<DeskewedScan> deskew_hits(
    const sensors::AssembledScan& scan, const SampledTrajectory& odom_from_base,
    const Pose2& base_from_laser, bool allow_provisional_for_diagnostics = false);

} // namespace slam::motion
