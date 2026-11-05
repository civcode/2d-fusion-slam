#pragma once

#include "slam/core/se2.hpp"

#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace slam::matching {

// A short group of historical laser scans expressed in a fixed anchor scan
// frame. No future scan, online occupancy map, or reference trajectory enters
// the submap. The caller chooses the temporal window and the anchor.
struct SubmapSweep {
  Pose2 anchor_from_scan;
  std::vector<Eigen::Vector2d> hits_scan;
};

// Frozen, private-to-the-experiment occupied-endpoint likelihood raster.
// This is intentionally NOT the global occupancy map used by the frontend.
// No mutable data or optimization is shared between the two representations.
struct FrozenLidarSubmap {
  double resolution_m{0.10};
  double origin_x{}, origin_y{};
  std::size_t width{}, height{}, contributing_scans{}, occupied_cells{};
  std::vector<std::uint8_t> likelihood;
  [[nodiscard]] bool valid() const noexcept;
};

[[nodiscard]] FrozenLidarSubmap build_frozen_lidar_submap(
    const std::vector<SubmapSweep>& sweeps, double resolution_m = 0.10);

// Translation/heading-invariant *retrieval* fingerprint from occupied
// endpoint separations. This proposes places, it never verifies a loop.
struct SubmapDescriptor {
  std::array<double,16> pair_distance_histogram{};
  bool valid{};
};
[[nodiscard]] SubmapDescriptor describe_frozen_lidar_submap(
    const FrozenLidarSubmap& submap);
[[nodiscard]] double submap_descriptor_similarity(
    const SubmapDescriptor& a,const SubmapDescriptor& b) noexcept;

struct SubmapSearchLimits {
  double translation_window_m{0.54};
  double rotation_window_rad{0.18};
  double minimum_support_fraction{0.58};
  double minimum_score{0.48};
  double minimum_translation_curvature{0.08}; // normalized score per m^2
  double minimum_curvature_ratio{0.12};
  std::size_t minimum_supported_hits{80};
  // Only for appearance-retrieved hypotheses. Allows larger Ceres drift,
  // with a wide coarse search and separate graph provenance.
  bool global_search{false};
};

struct SubmapMatchResult {
  Pose2 anchor_from_current{};
  bool accepted{};
  bool observable{};
  std::size_t supported_hits{};
  double support_fraction{}, correlation_score{}, rms_m{};
  double innovation_m{}, innovation_yaw_rad{};
  double translation_curvature_min{}, translation_curvature_ratio{};
  // The smallest four-way score drop for a 0.4m translation. Near zero
  // indicates a potentially ambiguous straight-corridor displacement.
  double minimum_shift_score_drop{};
};

// Local, coarse-to-fine correlative matching. The initial full SE(2) transform
// may have any heading; only the correction is windowed. A scan-to-submap
// factor must also pass observability, inlier and global graph checks.
[[nodiscard]] SubmapMatchResult register_scan_to_frozen_submap(
    const FrozenLidarSubmap& submap,
    const std::vector<Eigen::Vector2d>& hits_current,
    Pose2 initial_anchor_from_current,
    SubmapSearchLimits limits = {});

} // namespace slam::matching
