#include "slam/sensors/scan.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <variant>

namespace slam::sensors {
namespace {
constexpr double kTurn = 2.0 * std::numbers::pi;
}

ScanAssembler::ScanAssembler(ScanAssemblerConfig config) : config_(config) {
  if (!std::isfinite(config.seam_rad) || !std::isfinite(config.minimum_coverage) ||
      config.minimum_coverage <= 0 || config.minimum_coverage > 1 ||
      !std::isfinite(config.maximum_reverse_jitter_rad) ||
      config.maximum_reverse_jitter_rad < 0 || config.maximum_reverse_jitter_rad >= 1.0 ||
      config.angular_bins < 4 || config.angular_bins > 65536 ||
      config.maximum_beams == 0 || config.maximum_beams > 1'000'000) {
    throw std::invalid_argument("invalid scan assembler configuration");
  }
}

double ScanAssembler::phase(double angle_rad) const noexcept {
  const double relative = config_.direction == RotationDirection::counterclockwise
                              ? angle_rad - config_.seam_rad
                              : config_.seam_rad - angle_rad;
  const double rem = std::fmod(relative, kTurn);
  return rem < 0 ? rem + kTurn : rem;
}

std::optional<AssembledScan> ScanAssembler::push(BeamObservation beam) {
  if (beam.kind == ReturnKind::missing) {
    ++missing_;
    return std::nullopt;
  }
  if (!std::isfinite(beam.angle_rad) ||
      ((beam.kind == ReturnKind::hit || beam.kind == ReturnKind::no_return) &&
       (!std::isfinite(beam.range_m) || beam.range_m <= 0))) {
    throw std::invalid_argument("invalid beam angle/range");
  }
  const double now = phase(beam.angle_rad);
  bool crossed = false;
  if (!pending_.empty()) {
    if (beam.source_id != pending_.back().source_id ||
        beam.calibration_revision != pending_.back().calibration_revision) {
      throw std::invalid_argument("mixed sensor source or calibration revision");
    }
    const auto diff = between(beam.acquired_at, pending_.back().acquired_at);
    if (!std::holds_alternative<DurationNs>(diff) ||
        std::get<DurationNs>(diff).value < 0) {
      throw std::invalid_argument("beam times must increase in the same domain and epoch");
    }
    const double delta = now - *previous_phase_;
    if (delta < -std::numbers::pi) crossed = true;
    else if (delta < -config_.maximum_reverse_jitter_rad) {
      throw std::invalid_argument("unannounced scan angle reversal");
    }
  }
  if (pending_.size() >= config_.maximum_beams && !crossed) {
    throw std::invalid_argument("scan exceeds configured maximum beam count");
  }
  std::optional<AssembledScan> result;
  if (crossed) {
    result = finish(true);
    reset();
  }
  pending_.push_back(beam);
  previous_phase_ = now;
  return result;
}

AssembledScan ScanAssembler::finish(bool crossed) const {
  AssembledScan scan;
  scan.beams = pending_;
  scan.source_id = pending_.front().source_id;
  scan.calibration_revision = pending_.front().calibration_revision;
  scan.first_acquired = pending_.front().acquired_at;
  scan.last_acquired = pending_.back().acquired_at;
  const auto diff = between(scan.last_acquired, scan.first_acquired);
  if (!std::holds_alternative<DurationNs>(diff)) {
    throw std::overflow_error("scan timestamp span overflow");
  }
  const auto mid = advance(scan.first_acquired, DurationNs{std::get<DurationNs>(diff).value / 2});
  if (!std::holds_alternative<Timestamp>(mid)) {
    throw std::overflow_error("scan midpoint overflow");
  }
  scan.midpoint = std::get<Timestamp>(mid);
  scan.missing_placeholders = missing_;
  scan.ended_at_seam = crossed;
  std::vector<bool> angular_seen(config_.angular_bins, false);
  for (const auto& beam : pending_) {
    if (beam.kind == ReturnKind::hit) ++scan.hit_count;
    else if (beam.kind == ReturnKind::no_return) ++scan.no_return_count;
    else ++scan.invalid_count;
    const auto index = std::min(config_.angular_bins - 1,
      static_cast<std::size_t>((phase(beam.angle_rad) / kTurn) * config_.angular_bins + 1e-10));
    angular_seen[index] = true;
  }
  const auto count = static_cast<std::size_t>(std::count(angular_seen.begin(), angular_seen.end(), true));
  scan.acquisition_coverage = static_cast<double>(count) / config_.angular_bins;
  std::size_t run = 0, longest = 0;
  for (std::size_t i = 0; i < config_.angular_bins * 2; ++i) {
    if (!angular_seen[i % config_.angular_bins]) {
      run = std::min(run + 1, config_.angular_bins);
      longest = std::max(longest, run);
    } else run = 0;
  }
  scan.largest_missing_arc_rad = kTurn * static_cast<double>(longest) / config_.angular_bins;
  scan.partial = !crossed || scan.acquisition_coverage < config_.minimum_coverage;
  return scan;
}

std::optional<AssembledScan> ScanAssembler::flush() {
  if (pending_.empty()) {
    missing_ = 0;
    return std::nullopt;
  }
  auto scan = finish(false);
  reset();
  return scan;
}

void ScanAssembler::reset() noexcept {
  pending_.clear();
  missing_ = 0;
  previous_phase_.reset();
}

}  // namespace slam::sensors
