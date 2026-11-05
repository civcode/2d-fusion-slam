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
  // Independent of the full-cloud check: covariance of the *selected*
  // point correspondences at the final pose (not a calibrated covariance).
  bool correspondence_rank{};
  Pose2 previous_from_current{};
  std::size_t forward_inliers{}, reverse_inliers{};
  std::size_t iterations{};
  double forward_overlap{}, reverse_overlap{};
  double mean_error_m{};
  double translation_m{}, rotation_rad{};
};

// The default is deliberately narrow for consecutive scans. Nonconsecutive
// place-revisit hypotheses may opt into a wider displacement envelope, but
// must still pass separate reciprocal geometry and graph-innovation checks.
struct ScanRegistrationLimits {
  double max_translation_m{0.90};
  double max_rotation_rad{0.32};
};

// For revisits rather than consecutive scans. The input may contain any
// relative yaw in [-pi, pi]; only the *correction* from that SE(2) prior
// is restricted. Reusable later for scan-to-submap verification.
// Not a descriptor, global place-recognition method or uncertainty model.
struct PosePriorRegistrationLimits {
  double max_relative_translation_m{3.0};
  double max_correction_translation_m{0.55};
  double max_correction_yaw_rad{0.17};
};

// Conservative deterministic trimmed nearest-neighbor registration. Intended
// only as an explicitly enabled bootstrap for the synthetic LiDAR-only path.
// Rejects insufficient or weakly constrained geometry instead of inventing
// occupancy in unknown regions.
[[nodiscard]] ScanToScanResult register_consecutive_scans(
    const std::vector<Eigen::Vector2d>& previous,
    const std::vector<Eigen::Vector2d>& current,
    Pose2 initial_previous_from_current = {},
    bool wide_initial_search = true,
    ScanRegistrationLimits limits = {});

// Initial previous_from_current may have arbitrary rotation, including 180
// degrees. Match near this full SE(2) prior, not near zero heading. Both
// translation and yaw *innovation* are bounded around the prior, while the
// existing consecutive-scans API above keeps its historical constraints.
[[nodiscard]] ScanToScanResult register_scans_from_pose_prior(
    const std::vector<Eigen::Vector2d>& previous,
    const std::vector<Eigen::Vector2d>& current,
    Pose2 initial_previous_from_current,
    bool wide_initial_search = false,
    PosePriorRegistrationLimits limits = {});

} // namespace slam::matching
