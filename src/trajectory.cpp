#include "slam/motion/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <variant>

namespace slam::motion {

SampledTrajectory::SampledTrajectory(std::vector<TimedPose> samples, TrajectoryCapability capability)
    : samples_(std::move(samples)), capability_(capability) {
  if (samples_.size() < 2) throw std::invalid_argument("trajectory needs two or more samples");
  for (std::size_t i = 0; i < samples_.size(); ++i) {
    const auto& pose = samples_[i].odom_from_base;
    if (!pose.translation.allFinite() || !std::isfinite(pose.yaw_rad))
      throw std::invalid_argument("invalid trajectory pose");
    if (i == 0) continue;
    const auto delta = between(samples_[i].time, samples_[i - 1].time);
    if (!std::holds_alternative<DurationNs>(delta) || std::get<DurationNs>(delta).value <= 0)
      throw std::invalid_argument("trajectory timestamp discontinuity");
    if (capability_ == TrajectoryCapability::gyro_only &&
        (samples_[i].odom_from_base.translation - samples_[i - 1].odom_from_base.translation).norm() > 1e-9)
      throw std::invalid_argument("gyro-only trajectory cannot contain XY measurements");
  }
}

std::optional<Pose2> SampledTrajectory::at(Timestamp t) const noexcept {
  const auto whole = between(t, samples_.front().time);
  const auto remaining = between(samples_.back().time, t);
  if (!std::holds_alternative<DurationNs>(whole) || !std::holds_alternative<DurationNs>(remaining) ||
      std::get<DurationNs>(whole).value < 0 || std::get<DurationNs>(remaining).value < 0)
    return std::nullopt;
  const auto it = std::lower_bound(samples_.begin(), samples_.end(), t.ns,
       [](const TimedPose& sample, TimeNs ns) { return sample.time.ns < ns; });
  if (it == samples_.end()) return samples_.back().odom_from_base;
  if (it->time.ns == t.ns) return it->odom_from_base;
  const auto& first = *(it - 1);
  const auto& second = *it;
  const double fraction = static_cast<double>(t.ns - first.time.ns) /
                          static_cast<double>(second.time.ns - first.time.ns);
  const auto delta = log_se2(compose(inverse(first.odom_from_base), second.odom_from_base));
  return compose(first.odom_from_base,
    exp_se2({fraction * delta.dx, fraction * delta.dy, fraction * delta.dyaw}));
}

std::optional<DeskewedScan> deskew_hits(
    const sensors::AssembledScan& scan, const SampledTrajectory& trajectory,
    const Pose2& base_from_laser, bool allow_provisional_for_diagnostics) {
  if (scan.beams.empty() || !base_from_laser.translation.allFinite() ||
      !std::isfinite(base_from_laser.yaw_rad)) return std::nullopt;
  const bool verified_time = scan.time_quality == sensors::BeamTimeQuality::calibrated ||
                             scan.time_quality == sensors::BeamTimeQuality::synthetic_exact;
  if (!verified_time && !allow_provisional_for_diagnostics) return std::nullopt;
  const auto midpoint_pose = trajectory.at(scan.midpoint);
  if (!midpoint_pose) return std::nullopt;
  const Pose2 odom_from_laser_reference = compose(*midpoint_pose, base_from_laser);
  const auto laser_reference_from_odom = inverse(odom_from_laser_reference);
  DeskewedScan result;
  result.reference_time = scan.midpoint;
  result.capability = trajectory.capability();
  result.physical_beam_time_verified = verified_time;
  for (const auto& beam : scan.beams) {
    if (beam.kind != sensors::ReturnKind::hit) { ++result.skipped_non_hits; continue; }
    if (!std::isfinite(beam.angle_rad) || !std::isfinite(beam.range_m) || beam.range_m <= 0)
      return std::nullopt;
    const auto pose = trajectory.at(beam.acquired_at);
    if (!pose) return std::nullopt;
    const auto laser_pose = compose(*pose, base_from_laser);
    const Eigen::Vector2d local{beam.range_m * std::cos(beam.angle_rad),
                                 beam.range_m * std::sin(beam.angle_rad)};
    const auto point_odom = transform(laser_pose, local);
    result.hits.push_back({transform(laser_reference_from_odom, point_odom), beam.acquired_at});
  }
  return result;
}
} // namespace slam::motion
