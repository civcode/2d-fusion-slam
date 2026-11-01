#pragma once
#include "slam/datasets/carmen.hpp"
#include "slam/datasets/carmen_map.hpp"

#include <cstddef>
#include <vector>

namespace slam::datasets {
// Diagnostic for a reference grid generated from the same recording. NOT an
// independent SLAM benchmark, ground-truth alignment or valid localization.
struct AlignmentOptions {
  std::size_t holdout_modulus{5};       // index % 5 == 0 is held out
  std::size_t beam_stride{12};          // deterministic thinning, not deskew
  std::size_t calibration_scan_stride{17};
  double occupied_threshold{0.65};
  unsigned wall_radius_cells{3};
  double offset_x_min{-5.0}, offset_x_max{30.0};
  double offset_y_min{-5.0}, offset_y_max{20.0};
  double coarse_step_m{1.0};
};
struct AlignmentScore {
  std::size_t endpoints{}, within_grid{}, known{}, near_occupied{};
  [[nodiscard]] double known_fraction() const noexcept;
  [[nodiscard]] double wall_fraction() const noexcept; // among all attempted rays
};
struct AlignmentHypothesis {
  bool swap_xy{};
  int sign_x{1}, sign_y{1}; // may represent reflection: diagnostic only
  double translation_x{}, translation_y{};
};
struct AlignmentResult {
  AlignmentHypothesis selected;
  AlignmentScore calibration, held_out;
  std::size_t calibration_scans{}, held_out_scans{};
  bool independent_ground_truth{false}; // published reference map uses FR079 observations
};
[[nodiscard]] AlignmentResult diagnose_fr079_alignment(
    const CarmenMap& map, const std::vector<CarmenLaser>& scans,
    const AlignmentOptions& options = {});
} // namespace slam::datasets
