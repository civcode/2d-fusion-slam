#include "slam/evaluation/map_accuracy.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace slam;
using namespace slam::evaluation;
using namespace slam::mapping;

namespace {
void require(bool yes, const char* why) {
  if (!yes) { std::cerr << "FAIL map_accuracy: " << why << '\n'; std::exit(1); }
}

template<class F> void must_reject(F&& fn, const char* why) {
  bool rejected = false;
  try { fn(); } catch (const std::invalid_argument&) { rejected = true; }
  require(rejected, why);
}

MapBundle perfect_bundle(const fixtures::Scene& scene, GridConfig cfg) {
  const auto wall = rasterize_reference_walls(scene, cfg);
  std::vector<Cell> cells;
  cells.reserve(wall.size());
  for (const auto flag : wall) cells.push_back({flag ? 2.0 : -2.0, true});
  return {MapSnapshot::from_cells(cfg, 17, std::move(cells)),
          {"map", scene.id, "synthetic_exact", true}};
}

GridConfig grid() {
  GridConfig cfg;
  cfg.width = 310;
  cfg.height = 250;
  cfg.resolution_m = 0.05;
  cfg.map_from_grid = {{-0.25, -0.25}, 0.0};
  return cfg;
}
} // namespace

int main() {
  const auto ap = fixtures::apartment();
  const auto wh = fixtures::warehouse();
  auto map = perfect_bundle(ap, grid());
  const auto ideal = evaluate_map_accuracy(map, ap);
  require(ideal.reference_wall_cells > 0 &&
          ideal.predicted_wall_cells == ideal.reference_wall_cells,
          "exact reference-wall raster populated");
  require(ideal.occupied_iou && *ideal.occupied_iou == 1.0 &&
          ideal.occupied_precision && *ideal.occupied_precision == 1.0 &&
          ideal.occupied_recall && *ideal.occupied_recall == 1.0 &&
          ideal.occupied_f1 && *ideal.occupied_f1 == 1.0,
          "perfect reference map scores exactly 1");
  require(ideal.known_coverage == 1.0 && ideal.reference_wall_coverage == 1.0 &&
          ideal.symmetric_mean_wall_distance_m &&
          *ideal.symmetric_mean_wall_distance_m == 0.0 &&
          ideal.symmetric_p95_wall_distance_m &&
          *ideal.symmetric_p95_wall_distance_m == 0.0,
          "perfect reference wall distance is zero");

  std::ostringstream text1, text2;
  write_map_accuracy_json(text1, ideal, ap);
  write_map_accuracy_json(text2, evaluate_map_accuracy(map, ap), ap);
  require(text1.str() == text2.str() &&
          text1.str().find("\"format\": \"FUSMAPACC1\"") != std::string::npos &&
          text1.str().find("\"occupied_iou\": 1") != std::string::npos,
          "stable machine-readable report");

  // FUSMAP2D import uses exactly the map fields consumed by this evaluator.
  std::stringstream bytes(std::ios::in | std::ios::out | std::ios::binary);
  write_map_bundle(bytes, map);
  bytes.seekg(0);
  const auto imported = read_map_bundle(bytes);
  require(evaluate_map_accuracy(imported, ap).occupied_iou == ideal.occupied_iou &&
          imported.snapshot->fingerprint() == map.snapshot->fingerprint(),
          "binary bundle roundtrip retains accuracy and identity");

  auto cells = map.snapshot->cells();
  const auto truth = rasterize_reference_walls(ap, map.snapshot->config());
  std::size_t wall_index = 0, free_index = 0;
  for (std::size_t i = 0; i < truth.size(); ++i) {
    if (truth[i]) wall_index = i;
    else free_index = i;
  }
  cells[wall_index] = {0.0, false};
  cells[free_index] = {2.0, true};
  auto degraded = map;
  degraded.snapshot = MapSnapshot::from_cells(grid(), 18, cells);
  const auto worse = evaluate_map_accuracy(degraded, ap);
  require(worse.unknown_wall_cells == 1 && worse.false_positive == 1 &&
          worse.occupied_iou && *worse.occupied_iou < 1.0 &&
          worse.occupied_recall && *worse.occupied_recall < 1.0 &&
          worse.known_coverage < 1.0,
          "unknown truth walls and invented obstacles penalize IoU");
  require(worse.symmetric_mean_wall_distance_m &&
          *worse.symmetric_mean_wall_distance_m > 0 &&
          worse.symmetric_p95_wall_distance_m,
          "symmetric wall error detects geometric distortion");

  // Observed near-neutral is still ambiguous, not fake observed free.
  cells = map.snapshot->cells();
  cells[wall_index] = {0.0, true};
  degraded.snapshot = MapSnapshot::from_cells(grid(), 19, cells);
  const auto ambiguous = evaluate_map_accuracy(degraded, ap);
  require(ambiguous.unknown_wall_cells == 1 && ambiguous.known_cells + 1 == cells.size(),
          "neutral known-mask cell is truthfully unclassified");

  // Completely unknown map cannot achieve an artificially high score.
  std::vector<Cell> unknown(truth.size());
  degraded.snapshot = MapSnapshot::from_cells(grid(), 20, std::move(unknown));
  const auto empty = evaluate_map_accuracy(degraded, ap);
  require(empty.occupied_iou && *empty.occupied_iou == 0.0 &&
          empty.known_coverage == 0.0 &&
          !empty.occupied_precision && !empty.symmetric_mean_wall_distance_m,
          "unknown map has zero IoU and null unmeasurable wall distance");
  std::ostringstream null_report;
  write_map_accuracy_json(null_report, empty, ap);
  require(null_report.str().find("\"precision\": null") != std::string::npos &&
          null_report.str().find("\"symmetric_mean_wall_distance_m\": null") != std::string::npos,
          "undefined metrics are JSON null rather than zero/nan");

  // Rotated map origin is respected when rasterizing walls.
  auto rotated = grid();
  rotated.width = 650;
  rotated.height = 550;
  rotated.map_from_grid = {{-7.0, -7.0}, -0.16};
  const auto rotated_map = perfect_bundle(ap, rotated);
  require(evaluate_map_accuracy(rotated_map, ap).occupied_iou == 1.0,
          "rotated map grid retains exact reference alignment");

  auto wh_grid = grid();
  wh_grid.width = 620;
  wh_grid.height = 420;
  wh_grid.map_from_grid = {{-0.5,-0.5},0};
  const auto warehouse = perfect_bundle(wh, wh_grid);
  require(evaluate_map_accuracy(warehouse, wh).occupied_iou == 1.0,
          "warehouse scene exact reference is independent and deterministic");

  auto unverified = map;
  unverified.provenance.metric_origin_verified = false;
  must_reject([&]{ (void)evaluate_map_accuracy(unverified, ap); },
              "unverified map origin refused");
  unverified = map;
  unverified.provenance.truth_class = "derived_reference";
  must_reject([&]{ (void)evaluate_map_accuracy(unverified, ap); },
              "derived/FR079 reference cannot masquerade as truth");
  unverified = map;
  unverified.provenance.source_id = wh.id;
  must_reject([&]{ (void)evaluate_map_accuracy(unverified, ap); },
              "scene source mismatch refused");
  unverified = map;
  unverified.provenance.frame_id = "fr079_logged_laser_frame_UNVERIFIED";
  must_reject([&]{ (void)evaluate_map_accuracy(unverified, ap); },
              "unregistered frame refused");
  must_reject([&]{ (void)evaluate_map_accuracy(map, wh); },
              "wrong scene refused");
  auto clipped = grid();
  clipped.width = 100;
  must_reject([&]{ (void)rasterize_reference_walls(ap, clipped); },
              "clipped reference scene refused");
  auto small = MapAccuracyOptions{};
  small.maximum_cells = 100;
  must_reject([&]{ (void)evaluate_map_accuracy(map, ap, small); },
              "oversized evaluation refuses unexpected memory work");
  auto bad = MapAccuracyOptions{};
  bad.free_threshold_log_odds = 2.0;
  bad.occupied_threshold_log_odds = 1.0;
  must_reject([&]{ (void)evaluate_map_accuracy(map, ap, bad); },
              "overlapping classification thresholds refused");

  std::cout << "synthetic ground-truth occupancy, wall distance and safety gates passed\n";
}
