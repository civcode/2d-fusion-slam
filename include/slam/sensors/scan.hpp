#pragma once

#include "slam/core/time.hpp"

#include <cstddef>
#include <numbers>
#include <optional>
#include <vector>

namespace slam::sensors {

// Missing describes an explicit placeholder/gap, not a physical laser sample.
// It never contributes acquisition time, angular support or map evidence.
enum class ReturnKind { hit, no_return, invalid, missing };
enum class RotationDirection { counterclockwise, clockwise };
// Time provenance: unknown/provisional physical beam timing is not mapping-ready.
enum class BeamTimeQuality { unspecified, provisional, calibrated, synthetic_exact };

struct BeamObservation {
  Timestamp acquired_at{};
  double angle_rad{};  // sensor-local angle in radians (unwrapped or wrapped)
  double range_m{};
  ReturnKind kind{ReturnKind::invalid};
  std::uint64_t source_id{}; // 0 means unspecified synthetic/import source
  std::uint64_t calibration_revision{};
  BeamTimeQuality time_quality{BeamTimeQuality::unspecified};
};

struct AssembledScan {
  std::vector<BeamObservation> beams; // excludes missing placeholders
  std::uint64_t source_id{};
  std::uint64_t calibration_revision{};
  BeamTimeQuality time_quality{BeamTimeQuality::unspecified};
  Timestamp first_acquired{};
  Timestamp last_acquired{};
  Timestamp midpoint{};
  std::size_t hit_count{};
  std::size_t no_return_count{};
  std::size_t invalid_count{};
  std::size_t missing_placeholders{};
  double acquisition_coverage{}; // fraction of bins with actual observed angles
  double largest_missing_arc_rad{};
  bool ended_at_seam{};
  bool partial{};
};

struct ScanAssemblerConfig {
  double seam_rad{-std::numbers::pi};
  RotationDirection direction{RotationDirection::counterclockwise};
  std::size_t angular_bins{360};
  double minimum_coverage{0.95};
  double maximum_reverse_jitter_rad{0.10};
  std::size_t maximum_beams{20000};
};

// Single sensor stream in acquisition order. Caller must explicitly reset after
// a clock discontinuity; a mismatched domain/epoch never joins the old scan.
// Throws invalid_argument for invalid configuration, beam or timestamp ordering;
// throws overflow_error for overflowing time-span arithmetic.
class ScanAssembler {
 public:
  explicit ScanAssembler(ScanAssemblerConfig config = {});
  [[nodiscard]] std::optional<AssembledScan> push(BeamObservation beam);
  [[nodiscard]] std::optional<AssembledScan> flush();
  void reset() noexcept;

 private:
  [[nodiscard]] double phase(double angle_rad) const noexcept;
  [[nodiscard]] AssembledScan finish(bool crossed_seam) const;
  ScanAssemblerConfig config_;
  std::vector<BeamObservation> pending_;
  std::size_t missing_{};
  std::optional<double> previous_phase_;
};

}  // namespace slam::sensors
