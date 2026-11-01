#include "slam/localization/known_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace slam::localization {
namespace {
bool finite_pose(const Pose2& pose) {
  return pose.translation.allFinite() && std::isfinite(pose.yaw_rad);
}
bool valid_settings(const Settings& s) {
  return s.maximum_hits >= s.matching.min_supported_hits && s.maximum_hits <= 100'000 &&
         s.maximum_consecutive_rejections > 0 && s.maximum_consecutive_rejections <= 100 &&
         s.maximum_scan_gap_ns > 0 && s.freshness_timeout_ns > 0 &&
         std::isfinite(s.maximum_translation_innovation_m) &&
         s.maximum_translation_innovation_m > 0 &&
         std::isfinite(s.maximum_yaw_innovation_rad) &&
         s.maximum_yaw_innovation_rad > 0 && s.maximum_yaw_innovation_rad <= 3.141592653589793;
}
std::uint64_t ordered_difference(TimeNs later, TimeNs earlier) {
  return static_cast<std::uint64_t>(later) - static_cast<std::uint64_t>(earlier);
}
} // namespace

KnownMapTracker::KnownMapTracker(std::shared_ptr<const mapping::DistanceField> field,
                                 Pose2 base_from_laser, Settings settings, Matcher matcher)
  : field_(std::move(field)), base_from_laser_(base_from_laser),
    settings_(settings), matcher_(std::move(matcher)) {
  if (!field_ || !field_->source() || !field_->has_obstacles() ||
      !finite_pose(base_from_laser_) || !valid_settings(settings_))
    throw std::invalid_argument("known-map tracker requires an informative immutable map and valid bounds");
  if (!matcher_) matcher_ = matching::match_reference;
}

void KnownMapTracker::lose() noexcept {
  state_ = State::lost;
  map_from_odom_.reset();
  anchor_stamp_.reset();
  last_scan_stamp_.reset();
  last_received_steady_ns_.reset();
  last_accepted_steady_ns_.reset();
  calibration_revision_ = 0;
  rejections_ = 0;
}

bool KnownMapTracker::install_anchor(const Anchor& a) {
  lose();
  if (!a.map_provenance_verified || !a.extrinsics_verified ||
      !a.position_independently_anchored || a.calibration_revision == 0 ||
      !finite_pose(a.map_from_base) || !finite_pose(a.odom_from_base))
    return false;
  map_from_odom_ = compose(a.map_from_base, inverse(a.odom_from_base));
  anchor_stamp_ = a.stamp;
  calibration_revision_ = a.calibration_revision;
  // An anchor alone never validates a current scan or publishes a pose.
  return true;
}

Estimate KnownMapTracker::report(Reason reason, Timestamp stamp,
                                 matching::MatchStatus match_status) const {
  Estimate e;
  e.state = state_;
  e.reason = reason;
  e.stamp = stamp;
  e.match_status = match_status;
  e.map_revision = field_->source()->revision();
  e.map_fingerprint = field_->source()->fingerprint();
  return e;
}

Estimate KnownMapTracker::reject(Reason reason, Timestamp stamp,
                                 matching::MatchStatus match_status) {
  if (!anchor_stamp_ || !map_from_odom_) return report(Reason::no_anchor, stamp, match_status);
  ++rejections_;
  if (rejections_ >= settings_.maximum_consecutive_rejections) lose();
  else state_ = State::uncertain;
  return report(reason, stamp, match_status); // Never publish the last good pose.
}

Estimate KnownMapTracker::observe(const Observation& frame) {
  if (!anchor_stamp_ || !map_from_odom_)
    return report(state_ == State::relocalizing ? Reason::relocalization_requested :
                  Reason::no_anchor, frame.stamp);
  if (frame.calibration_revision != calibration_revision_) {
    lose();
    return report(Reason::calibration_changed, frame.stamp);
  }
  if (frame.stamp.clock_domain != anchor_stamp_->clock_domain ||
      frame.stamp.epoch != anchor_stamp_->epoch || frame.stamp.ns < anchor_stamp_->ns ||
      (last_scan_stamp_ && frame.stamp.ns <= last_scan_stamp_->ns) ||
      (ordered_difference(frame.stamp.ns, last_scan_stamp_ ? last_scan_stamp_->ns : anchor_stamp_->ns) >
         static_cast<std::uint64_t>(settings_.maximum_scan_gap_ns)) ||
      (last_received_steady_ns_ && frame.received_steady_ns < *last_received_steady_ns_)) {
    lose();
    return report(Reason::clock_discontinuity, frame.stamp);
  }
  if (last_accepted_steady_ns_ &&
      ordered_difference(frame.received_steady_ns, *last_accepted_steady_ns_) >
          static_cast<std::uint64_t>(settings_.freshness_timeout_ns)) {
    lose();
    return report(Reason::stale, frame.stamp);
  }
  last_scan_stamp_ = frame.stamp;
  last_received_steady_ns_ = frame.received_steady_ns;
  if (!frame.acquisition_time_verified || !frame.extrinsics_verified || !frame.odom_valid ||
      !finite_pose(frame.odom_from_base))
    return reject(Reason::invalid_evidence, frame.stamp);
  if (frame.laser_hits.size() > settings_.maximum_hits)
    return reject(Reason::input_over_budget, frame.stamp);

  const Pose2 predicted_base = compose(*map_from_odom_, frame.odom_from_base);
  const Pose2 predicted_laser = compose(predicted_base, base_from_laser_);
  matching::MatchResult matched;
  try {
    matched = matcher_(field_, frame.laser_hits, predicted_laser, settings_.matching);
  } catch (const std::exception&) {
    return reject(Reason::match_rejected, frame.stamp);
  }
  const auto& d = matched.diagnostics;
  if (d.source_revision != field_->source()->revision() ||
      d.source_fingerprint != field_->source()->fingerprint())
    return reject(Reason::source_mismatch, frame.stamp, matched.status);
  if (matched.status != matching::MatchStatus::accepted ||
      !finite_pose(matched.map_from_laser) || !d.full_rank ||
      d.input_hits != frame.laser_hits.size() ||
      d.supported_hits < settings_.matching.min_supported_hits ||
      d.supported_hits > d.input_hits || d.unsupported_hits != d.input_hits - d.supported_hits ||
      !std::isfinite(d.overlap) || d.overlap < settings_.matching.minimum_overlap ||
      d.overlap > 1.0 || !std::isfinite(d.robust_cost))
    return reject(Reason::match_rejected, frame.stamp, matched.status);
  const Pose2 candidate_base = compose(matched.map_from_laser, inverse(base_from_laser_));
  if (!finite_pose(candidate_base) ||
      (candidate_base.translation - predicted_base.translation).norm() >
          settings_.maximum_translation_innovation_m ||
      std::abs(wrap_yaw(candidate_base.yaw_rad - predicted_base.yaw_rad)) >
          settings_.maximum_yaw_innovation_rad)
    return reject(Reason::excessive_innovation, frame.stamp, matched.status);
  const Pose2 candidate_map_odom = compose(candidate_base, inverse(frame.odom_from_base));
  if (!finite_pose(candidate_map_odom))
    return reject(Reason::invalid_evidence, frame.stamp, matched.status);

  map_from_odom_ = candidate_map_odom;
  state_ = State::tracking;
  rejections_ = 0;
  last_accepted_steady_ns_ = frame.received_steady_ns;
  Estimate e = report(Reason::matched, frame.stamp, matched.status);
  e.map_from_base = candidate_base;
  e.map_from_odom = candidate_map_odom;
  return e;
}

Estimate KnownMapTracker::watchdog(TimeNs now_steady_ns) {
  if (!last_accepted_steady_ns_ || !last_scan_stamp_)
    return report(Reason::no_anchor, anchor_stamp_.value_or(Timestamp{}));
  if (now_steady_ns < *last_accepted_steady_ns_ ||
      ordered_difference(now_steady_ns, *last_accepted_steady_ns_) >
          static_cast<std::uint64_t>(settings_.freshness_timeout_ns)) {
    const auto stamp = *last_scan_stamp_;
    lose();
    return report(Reason::stale, stamp);
  }
  // Watchdog never republishes a previous pose as a new current estimate.
  return report(state_ == State::tracking ? Reason::matched : Reason::match_rejected,
                *last_scan_stamp_);
}

void KnownMapTracker::request_relocalization() noexcept {
  lose();
  state_ = State::relocalizing;
}

} // namespace slam::localization
