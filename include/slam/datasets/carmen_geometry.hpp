#pragma once
#include "slam/datasets/carmen.hpp"
#include <Eigen/Core>
#include <cstddef>
#include <vector>

namespace slam::datasets {
// Native input to geometry-only scan matching. Angles and ranges are scan
// geometry, not a claim about physical per-beam acquisition timestamps.
struct CarmenPointCloud {
  std::vector<Eigen::Vector2d> hit_points_laser;
  std::size_t rejected_ranges{};
  double scan_ipc_seconds{};
  bool per_beam_time_known{false};
};
[[nodiscard]] CarmenPointCloud carmen_laser_points(const CarmenLaser& scan);
} // namespace slam::datasets
