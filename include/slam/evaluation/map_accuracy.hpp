#pragma once

#include "slam/fixtures/synthetic.hpp"
#include "slam/mapping/map_bundle.hpp"

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <optional>
#include <vector>

namespace slam::evaluation {

// Compare an authoritative FUSMAP2D occupancy snapshot to exact synthetic
// wall geometry. No registration or truth fitting is performed.
struct MapAccuracyOptions {
  double occupied_threshold_log_odds{0.0};  // occupied if strictly greater
  double free_threshold_log_odds{0.0};      // free if strictly less
  std::size_t maximum_cells{4'000'000};     // bound evaluation memory/work
};

struct MapAccuracyReport {
  std::uint64_t map_fingerprint{};
  std::uint64_t map_revision{};
  std::size_t width{}, height{};
  double resolution_m{};
  double occupied_threshold_log_odds{};
  double free_threshold_log_odds{};
  std::size_t reference_wall_cells{};
  std::size_t predicted_wall_cells{};
  std::size_t true_positive{}, false_positive{}, false_negative{}, true_negative{};
  std::size_t unknown_wall_cells{}, unknown_free_cells{};
  std::size_t known_cells{}, unknown_cells{};
  std::optional<double> occupied_iou;
  std::optional<double> occupied_precision;
  std::optional<double> occupied_recall;
  std::optional<double> occupied_f1;
  double known_coverage{};
  double reference_wall_coverage{};
  std::optional<double> symmetric_mean_wall_distance_m;
  std::optional<double> symmetric_p95_wall_distance_m;
};

// The reference raster is deterministic, in the supplied grid frame, and
// contains 1 for cells whose centres lie within half a cell diagonal of an
// exact scene segment. Refuse partial/cropped scenes rather than score them.
[[nodiscard]] std::vector<std::uint8_t> rasterize_reference_walls(
    const fixtures::Scene& scene, const mapping::GridConfig& grid,
    std::size_t maximum_cells = 4'000'000);

// Requires a clearly named exact synthetic source and an explicit verified
// metric frame claim; unknown/ambiguous cells never count as free evidence.
[[nodiscard]] MapAccuracyReport evaluate_map_accuracy(
    const mapping::MapBundle& generated, const fixtures::Scene& scene,
    const MapAccuracyOptions& options = {});

// Stable JSON stdout/report format FUSMAPACC1; missing metrics serialize as
// null, not zero/infinity. The scene ID is emitted as exact source provenance.
void write_map_accuracy_json(std::ostream& out, const MapAccuracyReport& report,
                             const fixtures::Scene& scene);

} // namespace slam::evaluation
