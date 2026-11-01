#include "slam/evaluation/map_accuracy.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <ostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace slam::evaluation {
namespace {
constexpr double infinity = std::numeric_limits<double>::infinity();

void validate(const fixtures::Scene& scene, const mapping::GridConfig& g,
              std::size_t max_cells) {
  if (scene.id.empty() || scene.walls.empty() || max_cells == 0 ||
      g.width == 0 || g.height == 0 || g.width > max_cells ||
      g.height > max_cells / g.width || !std::isfinite(g.resolution_m) ||
      g.resolution_m <= 0 || !g.map_from_grid.translation.allFinite() ||
      !std::isfinite(g.map_from_grid.yaw_rad))
    throw std::invalid_argument("invalid/cropped synthetic reference grid");
  const Pose2 local_from_map = inverse(g.map_from_grid);
  const double xmax = static_cast<double>(g.width) * g.resolution_m;
  const double ymax = static_cast<double>(g.height) * g.resolution_m;
  if (!std::isfinite(xmax) || !std::isfinite(ymax))
    throw std::invalid_argument("grid extents overflow");
  for (const auto& wall : scene.walls) {
    if (!wall.start.allFinite() || !wall.end.allFinite() ||
        (wall.end - wall.start).squaredNorm() <= 0)
      throw std::invalid_argument("invalid reference segment");
    for (const auto& p : {wall.start, wall.end}) {
      const auto local = transform(local_from_map, p);
      // A wall must be completely inside the evaluated map; no crop-based
      // metric inflation. Boundary endpoints (x == xmax) are permitted.
      const double eps = 1e-9 * std::max({1.0, xmax, ymax});
      if (!local.allFinite() || local.x() < -eps || local.y() < -eps ||
          local.x() > xmax + eps || local.y() > ymax + eps)
        throw std::invalid_argument("generated grid does not contain entire reference scene");
    }
  }
}

double distance_squared(const Eigen::Vector2d& p, const fixtures::Segment& segment) {
  const Eigen::Vector2d direction = segment.end - segment.start;
  const double fraction = std::clamp((p - segment.start).dot(direction) /
                                    direction.squaredNorm(), 0.0, 1.0);
  return (p - (segment.start + fraction * direction)).squaredNorm();
}

// Exact one-dimensional squared Euclidean distance transform, lower-envelope
// of parabolas. Infinite input sites are omitted to avoid inf - inf NaNs.
void edt_1d(const std::vector<double>& f, std::vector<double>& out) {
  const std::size_t n = f.size();
  out.assign(n, infinity);
  std::vector<std::size_t> sites;
  sites.reserve(n);
  for (std::size_t i = 0; i < n; ++i)
    if (std::isfinite(f[i])) sites.push_back(i);
  if (sites.empty()) return;
  std::vector<std::size_t> v(sites.size());
  std::vector<double> z(sites.size() + 1);
  std::size_t k = 0;
  v[0] = sites[0];
  z[0] = -infinity; z[1] = infinity;
  for (std::size_t i = 1; i < sites.size(); ++i) {
    const auto q = sites[i];
    double s{};
    for (;;) {
      const double qq = static_cast<double>(q), vv = static_cast<double>(v[k]);
      s = ((f[q] + qq * qq) - (f[v[k]] + vv * vv)) / (2.0 * (qq - vv));
      if (k == 0 || s > z[k]) break;
      --k;
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = infinity;
  }
  k = 0;
  for (std::size_t q = 0; q < n; ++q) {
    while (z[k + 1] < static_cast<double>(q)) ++k;
    const double delta = static_cast<double>(q) - static_cast<double>(v[k]);
    out[q] = delta * delta + f[v[k]];
  }
}

std::vector<double> squared_edt(const std::vector<std::uint8_t>& occupied,
                                std::size_t width, std::size_t height) {
  std::vector<double> temporary(occupied.size(), infinity);
  std::vector<double> distances(occupied.size(), infinity);
  std::vector<double> in(std::max(width, height));
  std::vector<double> transformed;
  for (std::size_t x = 0; x < width; ++x) {
    in.resize(height);
    for (std::size_t y = 0; y < height; ++y)
      in[y] = occupied[y * width + x] ? 0.0 : infinity;
    edt_1d(in, transformed);
    for (std::size_t y = 0; y < height; ++y)
      temporary[y * width + x] = transformed[y];
  }
  for (std::size_t y = 0; y < height; ++y) {
    in.resize(width);
    for (std::size_t x = 0; x < width; ++x)
      in[x] = temporary[y * width + x];
    edt_1d(in, transformed);
    for (std::size_t x = 0; x < width; ++x)
      distances[y * width + x] = transformed[x];
  }
  return distances;
}

std::pair<double, double> distance_summary(std::vector<double>& samples) {
  double sum = 0.0;
  for (double x : samples) sum += x;
  const double mean = sum / static_cast<double>(samples.size());
  const std::size_t rank = static_cast<std::size_t>(
      std::ceil(0.95 * static_cast<double>(samples.size()))) - 1;
  std::nth_element(samples.begin(), samples.begin() + static_cast<std::ptrdiff_t>(rank),
                   samples.end());
  return {mean, samples[rank]};
}

std::optional<double> fraction(std::size_t numerator, std::size_t denominator) {
  if (!denominator) return std::nullopt;
  return static_cast<double>(numerator) / static_cast<double>(denominator);
}

void json_number(std::ostream& out, const std::optional<double>& n) {
  if (n) out << *n;
  else out << "null";
}

void json_string(std::ostream& out, const std::string& value) {
  out << '"';
  constexpr char hex[] = "0123456789abcdef";
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') out << '\\' << static_cast<char>(c);
    else if (c < 0x20) out << "\\u00" << hex[c >> 4] << hex[c & 15];
    else out << static_cast<char>(c);
  }
  out << '"';
}
} // namespace

std::vector<std::uint8_t> rasterize_reference_walls(
    const fixtures::Scene& scene, const mapping::GridConfig& grid,
    std::size_t maximum_cells) {
  validate(scene, grid, maximum_cells);
  std::vector<std::uint8_t> wall_cells(grid.width * grid.height, 0);
  const double half_diagonal_squared = grid.resolution_m * grid.resolution_m * 0.5;
  for (std::size_t y = 0; y < grid.height; ++y) {
    for (std::size_t x = 0; x < grid.width; ++x) {
      const Eigen::Vector2d local{
          (static_cast<double>(x) + 0.5) * grid.resolution_m,
          (static_cast<double>(y) + 0.5) * grid.resolution_m};
      const auto p = transform(grid.map_from_grid, local);
      for (const auto& wall : scene.walls) {
        if (distance_squared(p, wall) <= half_diagonal_squared + 1e-12) {
          wall_cells[y * grid.width + x] = 1;
          break;
        }
      }
    }
  }
  return wall_cells;
}

MapAccuracyReport evaluate_map_accuracy(const mapping::MapBundle& generated,
                                       const fixtures::Scene& scene,
                                       const MapAccuracyOptions& options) {
  if (!generated.snapshot ||
      (generated.provenance.truth_class != "synthetic_exact" &&
       generated.provenance.truth_class != "synthetic_estimated") ||
      !generated.provenance.metric_origin_verified ||
      generated.provenance.frame_id != "map" ||
      generated.provenance.source_id != scene.id)
    throw std::invalid_argument("map is not explicitly aligned to the selected exact synthetic scene");
  if (!std::isfinite(options.occupied_threshold_log_odds) ||
      !std::isfinite(options.free_threshold_log_odds) ||
      options.free_threshold_log_odds > options.occupied_threshold_log_odds)
    throw std::invalid_argument("invalid occupancy classification thresholds");
  const auto& g = generated.snapshot->config();
  const auto truth = rasterize_reference_walls(scene, g, options.maximum_cells);
  const auto& cells = generated.snapshot->cells();
  if (cells.size() != truth.size())
    throw std::invalid_argument("snapshot dimensions/cells mismatch");
  std::vector<std::uint8_t> predicted(cells.size(), 0);
  MapAccuracyReport r;
  r.map_fingerprint = generated.snapshot->fingerprint();
  r.map_revision = generated.snapshot->revision();
  r.width = g.width; r.height = g.height; r.resolution_m = g.resolution_m;
  r.occupied_threshold_log_odds = options.occupied_threshold_log_odds;
  r.free_threshold_log_odds = options.free_threshold_log_odds;
  for (std::size_t i = 0; i < cells.size(); ++i) {
    if (truth[i]) ++r.reference_wall_cells;
    // Cells at a classification boundary, even if observed, are unknown
    // for this evaluation. Unknown is never silently counted as free.
    const bool occupied = cells[i].observed &&
        cells[i].log_odds > options.occupied_threshold_log_odds;
    const bool free = cells[i].observed &&
        cells[i].log_odds < options.free_threshold_log_odds;
    if (occupied) {
      predicted[i] = 1;
      ++r.predicted_wall_cells;
      ++r.known_cells;
      if (truth[i]) ++r.true_positive;
      else ++r.false_positive;
    } else if (free) {
      ++r.known_cells;
      if (truth[i]) ++r.false_negative;
      else ++r.true_negative;
    } else {
      ++r.unknown_cells;
      if (truth[i]) ++r.unknown_wall_cells;
      else ++r.unknown_free_cells;
    }
  }
  if (!r.reference_wall_cells)
    throw std::invalid_argument("no rasterized reference walls in grid");
  const std::size_t union_count = r.true_positive + r.false_positive +
                                  r.false_negative + r.unknown_wall_cells;
  r.occupied_iou = fraction(r.true_positive, union_count);
  r.occupied_precision = fraction(r.true_positive, r.predicted_wall_cells);
  r.occupied_recall = fraction(r.true_positive, r.reference_wall_cells);
  if (r.occupied_precision && r.occupied_recall &&
      *r.occupied_precision + *r.occupied_recall > 0)
    r.occupied_f1 = 2.0 * (*r.occupied_precision) * (*r.occupied_recall) /
                    (*r.occupied_precision + *r.occupied_recall);
  else if (r.occupied_precision && r.occupied_recall)
    r.occupied_f1 = 0.0;
  r.known_coverage = static_cast<double>(r.known_cells) /
                     static_cast<double>(cells.size());
  r.reference_wall_coverage = r.reference_wall_cells ?
      static_cast<double>(r.true_positive + r.false_negative) /
      static_cast<double>(r.reference_wall_cells) : 0.0;
  // Wall-distance metrics require evidence on both sides. With an empty
  // predicted wall set, returning 0 or infinity would mislead.
  if (r.predicted_wall_cells && r.reference_wall_cells) {
    const auto distance_to_truth = squared_edt(truth, g.width, g.height);
    const auto distance_to_prediction = squared_edt(predicted, g.width, g.height);
    std::vector<double> predicted_to_truth, truth_to_predicted;
    predicted_to_truth.reserve(r.predicted_wall_cells);
    truth_to_predicted.reserve(r.reference_wall_cells);
    for (std::size_t i = 0; i < cells.size(); ++i) {
      if (predicted[i])
        predicted_to_truth.push_back(std::sqrt(distance_to_truth[i]) * g.resolution_m);
      if (truth[i])
        truth_to_predicted.push_back(std::sqrt(distance_to_prediction[i]) * g.resolution_m);
    }
    const auto [mean_a, p95_a] = distance_summary(predicted_to_truth);
    const auto [mean_b, p95_b] = distance_summary(truth_to_predicted);
    r.symmetric_mean_wall_distance_m = 0.5 * (mean_a + mean_b);
    r.symmetric_p95_wall_distance_m = 0.5 * (p95_a + p95_b);
  }
  return r;
}

void write_map_accuracy_json(std::ostream& out, const MapAccuracyReport& r,
                             const fixtures::Scene& scene) {
  out.imbue(std::locale::classic());
  out << std::setprecision(12);
  out << "{\n  \"format\": \"FUSMAPACC1\",\n  \"reference_class\": \"synthetic_exact\",\n"
      << "  \"reference_scene\": ";
  json_string(out, scene.id);
  out << ",\n  \"alignment\": \"declared_same_metric_map_frame_no_fitting\",\n"
      << "  \"map_fingerprint\": " << r.map_fingerprint
      << ",\n  \"map_revision\": " << r.map_revision
      << ",\n  \"width\": " << r.width << ",\n  \"height\": " << r.height
      << ",\n  \"resolution_m\": " << r.resolution_m
      << ",\n  \"occupied_threshold_log_odds\": " << r.occupied_threshold_log_odds
      << ",\n  \"free_threshold_log_odds\": " << r.free_threshold_log_odds
      << ",\n  \"wall_raster_rule\": \"cell_center_within_half_diagonal\",\n"
      << "  \"confusion\": {\"tp\": " << r.true_positive
      << ", \"fp\": " << r.false_positive
      << ", \"fn_observed_free\": " << r.false_negative
      << ", \"tn\": " << r.true_negative
      << ", \"unknown_wall\": " << r.unknown_wall_cells
      << ", \"unknown_free\": " << r.unknown_free_cells << "},\n"
      << "  \"counts\": {\"truth_wall\": " << r.reference_wall_cells
      << ", \"predicted_wall\": " << r.predicted_wall_cells
      << ", \"known\": " << r.known_cells
      << ", \"unknown\": " << r.unknown_cells << "},\n"
      << "  \"metrics\": {\n    \"occupied_iou\": ";
  json_number(out, r.occupied_iou);
  out << ",\n    \"precision\": "; json_number(out, r.occupied_precision);
  out << ",\n    \"recall\": "; json_number(out, r.occupied_recall);
  out << ",\n    \"f1\": "; json_number(out, r.occupied_f1);
  out << ",\n    \"known_coverage\": " << r.known_coverage
      << ",\n    \"reference_wall_coverage\": " << r.reference_wall_coverage
      << ",\n    \"symmetric_mean_wall_distance_m\": ";
  json_number(out, r.symmetric_mean_wall_distance_m);
  out << ",\n    \"symmetric_p95_wall_distance_m\": ";
  json_number(out, r.symmetric_p95_wall_distance_m);
  out << "\n  }\n}\n";
  if (!out) throw std::runtime_error("failed writing map accuracy JSON");
}

} // namespace slam::evaluation
