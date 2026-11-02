#pragma once

#include "slam/mapping/distance_field.hpp"

#include <Eigen/Core>
#include <cstddef>
#include <vector>

namespace slam::matching {

// Pose of the current LiDAR scan expressed in the previous scan's laser
// coordinates. No odometry, simulator reference, map, or range raster needed.
struct ScanToScanResult {
  bool accepted{};
  Pose2 previous_from_current{};
  std::size_t forward_inliers{}, reverse_inliers{};
  std::size_t iterations{};
  double forward_overlap{}, reverse_overlap{};
  double mean_error_m{};
  double translation_m{}, rotation_rad{};
};

// Conservative deterministic trimmed nearest-neighbor registration. Intended
// only as an explicitly enabled bootstrap for the synthetic LiDAR-only path.
// Rejects insufficient or weakly constrained geometry instead of inventing
// occupancy in unknown regions.
[[nodiscard]] ScanToScanResult register_consecutive_scans(
    const std::vector<Eigen::Vector2d>& previous,
    const std::vector<Eigen::Vector2d>& current,
    Pose2 initial_previous_from_current = {},
    bool wide_initial_search = true);

} // namespace slam::matching
