#pragma once

#include "slam/core/se2.hpp"
#include "slam/core/time.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace slam::fixtures {
struct Segment {
  Eigen::Vector2d start, end;
};

struct Scene {
  std::string id;
  std::vector<Segment> walls;
};

struct Beam {
  Timestamp acquired_at;
  double angle_rad{};
  double range_m{};
  bool hit{}; // false means a simulated valid no-return, never a measured obstacle
};

struct Scan {
  Pose2 reference_pose; // exact simulated ground truth, NOT provided to SLAM estimators
  Timestamp reference_time;
  std::vector<Beam> beams;
};

[[nodiscard]] Scene apartment();
[[nodiscard]] Scene warehouse();

// Sample a moving synthetic robot with exact per-beam acquisition time.
// Throws std::invalid_argument on bad config, std::overflow_error on time overflow.
[[nodiscard]] Scan simulate_scan(const Scene& scene, Pose2 reference_pose,
                                 Tangent2 body_twist_per_second,
                                 Pose2 base_to_laser, Timestamp reference_time,
                                 std::size_t beam_count = 360,
                                 TimeNs scan_duration_ns = 100'000'000,
                                 double max_range_m = 30.0);
}  // namespace slam::fixtures
