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

// Optional distance-tolerant surface agreement complements, but never
// replaces, exact-cell occupied IoU. Both directed fractions are measured
// against fixed synthetic wall geometry after the map has been produced.
struct WallToleranceScore {
  double tolerance_m{};
  double predicted_near_reference{}; // precision-like
  double reference_near_predicted{}; // recall-like
  double symmetric_f1{};
};
[[nodiscard]] std::vector<WallToleranceScore> evaluate_wall_tolerances(
    const mapping::MapBundle& generated, const fixtures::Scene& scene,
    const std::vector<double>& tolerances_m = {0.10,0.25,0.50});

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

// Spatial comparison footprint derived only from an earlier reconstructed
// baseline map's ray-observed cells, padded by a fixed metric margin. No
// ground-truth geometry is used to select cells. The same footprint must
// be reused for every mapping variant within a frame window.
[[nodiscard]] std::vector<std::uint8_t> observed_footprint_mask(
    const mapping::MapBundle& baseline, double padding_m = 0.25);

// Compute exactly the usual occupancy metrics and wall-distance statistics,
// but restrict BOTH prediction and reference to the same externally supplied
// evaluation region. An unknown reference wall inside the footprint is still
// a missed wall. Coverage is relative to region cells, not the whole grid.
// Existing whole-scene scoring is intentionally unchanged.
[[nodiscard]] MapAccuracyReport evaluate_map_accuracy_region(
    const mapping::MapBundle& generated, const fixtures::Scene& scene,
    const std::vector<std::uint8_t>& region,
    const MapAccuracyOptions& options = {});

[[nodiscard]] std::vector<WallToleranceScore> evaluate_wall_tolerances_region(
    const mapping::MapBundle& generated, const fixtures::Scene& scene,
    const std::vector<std::uint8_t>& region,
    const std::vector<double>& tolerances_m = {0.10, 0.25, 0.50});

// Stable JSON stdout/report format FUSMAPACC1; missing metrics serialize as
// null, not zero/infinity. The scene ID is emitted as exact source provenance.
void write_map_accuracy_json(std::ostream& out, const MapAccuracyReport& report,
                             const fixtures::Scene& scene);

} // namespace slam::evaluation
