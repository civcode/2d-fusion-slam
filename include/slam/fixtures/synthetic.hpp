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
// New suite alias: unchanged Apartment floorplan with a distinct route and identity.
[[nodiscard]] Scene apartment_explore();
// Native contour representation of the existing raster-based demo workshop.
[[nodiscard]] Scene workshop();
[[nodiscard]] Scene warehouse();
[[nodiscard]] Scene office();
[[nodiscard]] Scene loft();
[[nodiscard]] Scene atrium();
[[nodiscard]] Scene industrial();
[[nodiscard]] Scene warehouse_asymmetric();

// All fixture reference geometries. Synthetic truth is used only to
// simulate observations and independently score generated maps.
[[nodiscard]] Scene named_scene(const std::string& name);


// Sample a moving synthetic robot with exact per-beam acquisition time.
// Throws std::invalid_argument on bad config, std::overflow_error on time overflow.
[[nodiscard]] Scan simulate_scan(const Scene& scene, Pose2 reference_pose,
                                 Tangent2 body_twist_per_second,
                                 Pose2 base_to_laser, Timestamp reference_time,
                                 std::size_t beam_count = 360,
                                 TimeNs scan_duration_ns = 100'000'000,
                                 double max_range_m = 30.0);
}  // namespace slam::fixtures
