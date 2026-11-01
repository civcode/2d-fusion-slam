#pragma once
#include "slam/motion/trajectory.hpp"

#include <cstddef>
#include <vector>

namespace slam::fusion {

// Clock mappings and sensor-to-base calibration must be verified by the
// caller. Clock identity is *not* inferred by matching numeric timestamps.
// Wheel velocities are body-frame measurements; gyro is yaw rate in base frame.
struct SensorTick {
  Timestamp time{};
  double gyro_z_rad_s{};
  double wheel_forward_m_s{};
  double wheel_yaw_rad_s{}; // independent wheel-only baseline
  bool gyro_valid{false};
  bool wheel_valid{false};
  bool stationary_calibration{false}; // externally declared stationary interval
};

struct FusionOptions {
  std::size_t minimum_stationary_ticks{8};
  TimeNs maximum_interval_ns{100'000'000};
  double maximum_gyro_abs_rate_rad_s{12.0};
  double maximum_wheel_speed_m_s{4.0};
  double maximum_bias_abs_rad_s{0.25};
  // Default refuses missing wheel translation for moving-map insertion.
  bool permit_gyro_only_diagnostics{false};
};

struct FusionResult {
  std::vector<motion::TimedPose> fused;
  std::vector<motion::TimedPose> wheel_only;
  motion::TrajectoryCapability capability{motion::TrajectoryCapability::gyro_only};
  double estimated_gyro_bias_rad_s{};
  std::size_t stationary_ticks{};
  bool frame_and_clock_attested{false};
};

// Explicit clock/frame trust and a fixed first-pose map gauge are required.
// No interpolation of missing gyro/wheel measurements or extrapolation;
// an entire batch is rejected on bad provenance or a time discontinuity.
// The initial stationary window is the ONLY source for gyro bias; its
// existence is a caller attestation, not automatically verified by this API.
[[nodiscard]] FusionResult integrate_sensors(
    const std::vector<SensorTick>& ticks, Pose2 initial_map_from_base,
    bool clock_and_frame_verified, const FusionOptions& options = {});

} // namespace slam::fusion
