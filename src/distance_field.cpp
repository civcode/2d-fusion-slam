#include "slam/mapping/distance_field.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace slam::mapping {
namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr std::uint64_t kFnvStart = 14695981039346656037ULL;

void hash_word(std::uint64_t& hash, std::uint64_t word) noexcept {
  for (unsigned i = 0; i < 8; ++i) {
    hash ^= (word >> (i * 8)) & 0xffU;
    hash *= 1099511628211ULL;
  }
}

// Exact squared-distance transform of a one-dimensional array in O(n).
// Assumes nonnegative finite values or +inf, with finite n and squared
// distances representable as double. The all-inf case remains all-inf.
std::vector<double> edt_1d(const std::vector<double>& input) {
  const auto n = input.size();
  std::vector<double> result(n, kInf);
  std::vector<std::size_t> sites;
  sites.reserve(n);
  for (std::size_t i = 0; i < n; ++i) if (std::isfinite(input[i])) sites.push_back(i);
  if (sites.empty()) return result;
  std::vector<std::size_t> envelope(n);
  std::vector<double> bounds(n + 1);
  std::size_t count = 1;
  envelope[0] = sites.front();
  bounds[0] = -kInf;
  bounds[1] = kInf;
  const auto intersection = [&](std::size_t p, std::size_t q) {
    const double a = static_cast<double>(p), b = static_cast<double>(q);
    return ((input[q] + b * b) - (input[p] + a * a)) / (2.0 * (b - a));
  };
  for (std::size_t i = 1; i < sites.size(); ++i) {
    const auto q = sites[i];
    double boundary = intersection(envelope[count - 1], q);
    while (count > 1 && boundary <= bounds[count - 1]) {
      --count;
      boundary = intersection(envelope[count - 1], q);
    }
    if (boundary <= bounds[count - 1]) {
      envelope[0] = q;
      bounds[0] = -kInf;
      count = 1;
    } else {
      envelope[count++] = q;
    }
    bounds[count] = kInf;
    if (count > 1) bounds[count - 1] = boundary;
  }
  std::size_t active = 0;
  for (std::size_t i = 0; i < n; ++i) {
    while (active + 1 < count && bounds[active + 1] < static_cast<double>(i)) ++active;
    const double d = static_cast<double>(i) - static_cast<double>(envelope[active]);
    result[i] = d * d + input[envelope[active]];
  }
  return result;
}
}  // namespace

MapSnapshot::MapSnapshot(GridConfig config, std::uint64_t revision, std::vector<Cell> cells)
    : config_(config), revision_(revision), cells_(std::move(cells)) {
  std::uint64_t hash = kFnvStart;
  hash_word(hash, config_.width); hash_word(hash, config_.height);
  hash_word(hash, std::bit_cast<std::uint64_t>(config_.resolution_m));
  hash_word(hash, std::bit_cast<std::uint64_t>(config_.map_from_grid.translation.x()));
  hash_word(hash, std::bit_cast<std::uint64_t>(config_.map_from_grid.translation.y()));
  hash_word(hash, std::bit_cast<std::uint64_t>(config_.map_from_grid.yaw_rad));
  hash_word(hash, revision_);
  for (const auto& c : cells_) {
    hash_word(hash, std::bit_cast<std::uint64_t>(c.log_odds));
    hash_word(hash, c.observed ? 1 : 0);
  }
  fingerprint_ = hash;
}

std::shared_ptr<const MapSnapshot> MapSnapshot::capture(const OccupancyGrid& grid) {
  return std::shared_ptr<const MapSnapshot>(new MapSnapshot(
      grid.config(), grid.revision(), grid.snapshot_cells()));
}

std::optional<Cell> MapSnapshot::cell(std::size_t x, std::size_t y) const noexcept {
  if (x >= config_.width || y >= config_.height) return std::nullopt;
  return cells_[y * config_.width + x];
}

DistanceField::DistanceField(std::shared_ptr<const MapSnapshot> source,
                             double truncation, double threshold)
    : source_(std::move(source)), grid_from_map_(inverse(source_->config().map_from_grid)),
      truncation_metres_(truncation) {
  const auto& c = source_->config();
  const auto w = c.width, h = c.height;
  std::vector<double> interim(w * h, kInf);
  for (std::size_t y = 0; y < h; ++y) {
    for (std::size_t x = 0; x < w; ++x) {
      const auto& cell = source_->cells()[y * w + x];
      if (cell.observed && cell.log_odds > threshold) {
        interim[y * w + x] = 0.0;
        has_obstacles_ = true;
      }
    }
  }
  values_.assign(w * h, truncation);
  if (!has_obstacles_) return;
  for (std::size_t y = 0; y < h; ++y) {
    std::vector<double> line(interim.begin() + static_cast<std::ptrdiff_t>(y * w),
                             interim.begin() + static_cast<std::ptrdiff_t>((y + 1) * w));
    const auto result = edt_1d(line);
    std::copy(result.begin(), result.end(), interim.begin() + static_cast<std::ptrdiff_t>(y * w));
  }
  for (std::size_t x = 0; x < w; ++x) {
    std::vector<double> line(h);
    for (std::size_t y = 0; y < h; ++y) line[y] = interim[y * w + x];
    const auto result = edt_1d(line);
    for (std::size_t y = 0; y < h; ++y) {
      values_[y * w + x] = std::min(truncation, std::sqrt(result[y]) * c.resolution_m);
    }
  }
}

std::shared_ptr<const DistanceField> DistanceField::build(
    std::shared_ptr<const MapSnapshot> source, double truncation, double threshold) {
  if (!source || !std::isfinite(truncation) || truncation <= 0 ||
      !std::isfinite(threshold)) throw std::invalid_argument("invalid EDT configuration");
  return std::shared_ptr<const DistanceField>(new DistanceField(
      std::move(source), truncation, threshold));
}

std::optional<double> DistanceField::cell_distance(std::size_t x, std::size_t y) const noexcept {
  const auto& c = source_->config();
  if (!has_obstacles_ || x >= c.width || y >= c.height) return std::nullopt;
  return values_[y * c.width + x];
}

std::optional<DistanceSample> DistanceField::sample_map(const Eigen::Vector2d& p) const noexcept {
  if (!has_obstacles_ || !p.allFinite()) return std::nullopt;
  const auto& c = source_->config();
  const auto local = transform(grid_from_map_, p);
  const double gx = local.x() / c.resolution_m - 0.5;
  const double gy = local.y() / c.resolution_m - 0.5;
  if (!std::isfinite(gx) || !std::isfinite(gy) || gx < 0 || gy < 0 ||
      gx >= static_cast<double>(c.width - 1) ||
      gy >= static_cast<double>(c.height - 1)) return std::nullopt;
  const auto x = static_cast<std::size_t>(gx), y = static_cast<std::size_t>(gy);
  const double u = gx - static_cast<double>(x), v = gy - static_cast<double>(y);
  const std::size_t index = y * c.width + x;
  const double a = values_[index], b = values_[index + 1];
  const double d = values_[index + c.width], e = values_[index + c.width + 1];
  if (!source_->cells()[index].observed || !source_->cells()[index + 1].observed ||
      !source_->cells()[index + c.width].observed ||
      !source_->cells()[index + c.width + 1].observed ||
      a >= truncation_metres_ || b >= truncation_metres_ ||
      d >= truncation_metres_ || e >= truncation_metres_) return std::nullopt;
  const double distance = (1.0 - u) * ((1.0 - v) * a + v * d) +
                          u * ((1.0 - v) * b + v * e);
  const double grad_x = ((1.0 - v) * (b - a) + v * (e - d)) / c.resolution_m;
  const double grad_y = ((1.0 - u) * (d - a) + u * (e - b)) / c.resolution_m;
  const double cosine = std::cos(c.map_from_grid.yaw_rad);
  const double sine = std::sin(c.map_from_grid.yaw_rad);
  return DistanceSample{distance, {cosine * grad_x - sine * grad_y,
                                   sine * grad_x + cosine * grad_y}};
}
}  // namespace slam::mapping
