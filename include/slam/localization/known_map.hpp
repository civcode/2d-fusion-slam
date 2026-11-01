#pragma once

#include "slam/matching/geometry.hpp"
#include "slam/core/time.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace slam::localization {

enum class State { lost, uncertain, tracking, relocalizing };
enum class Reason {
  no_anchor, matched, match_rejected, invalid_evidence, excessive_innovation,
  source_mismatch, clock_discontinuity, calibration_changed, stale, input_over_budget,
  relocalization_requested
};

struct Settings {
  matching::MatchSettings matching{};
  std::size_t maximum_hits{720};
  unsigned maximum_consecutive_rejections{2};
  TimeNs maximum_scan_gap_ns{1'000'000'000};
  TimeNs freshness_timeout_ns{500'000'000}; // host-steady time, NOT sensor time
  double maximum_translation_innovation_m{0.35};
  double maximum_yaw_innovation_rad{0.35};
};

// Provenance flags are caller attestations, NOT verified by this component.
// An unverified map/frame/anchor must never be marked authoritative.
struct Anchor {
  Timestamp stamp{};
  Pose2 map_from_base{};
  Pose2 odom_from_base{};
  std::uint64_t calibration_revision{};
  bool map_provenance_verified{};
  bool extrinsics_verified{};
  bool position_independently_anchored{};
};

struct Observation {
  Timestamp stamp{};
  TimeNs received_steady_ns{};
  Pose2 odom_from_base{};
  std::vector<Eigen::Vector2d> laser_hits;
  std::uint64_t calibration_revision{};
  bool acquisition_time_verified{};
  bool extrinsics_verified{};
  bool odom_valid{};
};

struct Estimate {
  State state{State::lost};
  Reason reason{Reason::no_anchor};
  Timestamp stamp{};
  std::optional<Pose2> map_from_base;
  std::optional<Pose2> map_from_odom;
  matching::MatchStatus match_status{matching::MatchStatus::invalid_input};
  std::uint64_t map_revision{};
  std::uint64_t map_fingerprint{};
  [[nodiscard]] bool globally_valid() const noexcept {
    return state == State::tracking && map_from_base.has_value() && map_from_odom.has_value();
  }
};

using Matcher = std::function<matching::MatchResult(
    std::shared_ptr<const mapping::DistanceField>,
    const std::vector<Eigen::Vector2d>&, Pose2, const matching::MatchSettings&)>;

// Read-only map tracking with explicit host-provided absolute anchor. No
// autonomous global search, IMU/odometry fabrication, map mutation or graph factors.
class KnownMapTracker final {
 public:
  KnownMapTracker(std::shared_ptr<const mapping::DistanceField> field,
                  Pose2 base_from_laser, Settings settings = {}, Matcher matcher = {});
  [[nodiscard]] bool install_anchor(const Anchor& anchor);
  [[nodiscard]] Estimate observe(const Observation& frame);
  [[nodiscard]] Estimate watchdog(TimeNs now_steady_ns);
  void request_relocalization() noexcept;
  [[nodiscard]] State state() const noexcept { return state_; }

 private:
  [[nodiscard]] Estimate report(Reason reason, Timestamp stamp,
                                matching::MatchStatus match_status = matching::MatchStatus::invalid_input) const;
  [[nodiscard]] Estimate reject(Reason reason, Timestamp stamp,
                                matching::MatchStatus match_status = matching::MatchStatus::invalid_input);
  void lose() noexcept;

  std::shared_ptr<const mapping::DistanceField> field_;
  Pose2 base_from_laser_;
  Settings settings_;
  Matcher matcher_;
  State state_{State::lost};
  std::optional<Pose2> map_from_odom_;
  std::optional<Timestamp> anchor_stamp_;
  std::optional<Timestamp> last_scan_stamp_;
  std::optional<TimeNs> last_received_steady_ns_;
  std::optional<TimeNs> last_accepted_steady_ns_;
  std::uint64_t calibration_revision_{};
  unsigned rejections_{};
};

} // namespace slam::localization
