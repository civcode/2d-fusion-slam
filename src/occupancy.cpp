#include "slam/mapping/occupancy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace slam::mapping {
namespace {
constexpr std::size_t kMaximumCells = 16'000'000;

// Liang-Barsky line clipping in grid metric coordinates; upper bounds are
// half-open, so clipping uses the preceding representable floating value.
bool clip_to_grid(Eigen::Vector2d& a, Eigen::Vector2d& b,
                  double width_m, double height_m) {
  const Eigen::Vector2d delta = b - a;
  const double min_x = 0, min_y = 0;
  const double max_x = std::nextafter(width_m, 0.0);
  const double max_y = std::nextafter(height_m, 0.0);
  double t0 = 0, t1 = 1;
  const auto bound = [&](double p, double q) -> bool {
    if (p == 0.0) return q >= 0;
    const double t = q / p;
    if (p < 0) t0 = std::max(t0, t);
    else t1 = std::min(t1, t);
    return t0 <= t1;
  };
  if (!bound(-delta.x(), a.x() - min_x) ||
      !bound(delta.x(), max_x - a.x()) ||
      !bound(-delta.y(), a.y() - min_y) ||
      !bound(delta.y(), max_y - a.y())) return false;
  const auto original = a;
  a = original + t0 * delta;
  b = original + t1 * delta;
  a.x() = std::clamp(a.x(), min_x, max_x);
  a.y() = std::clamp(a.y(), min_y, max_y);
  b.x() = std::clamp(b.x(), min_x, max_x);
  b.y() = std::clamp(b.y(), min_y, max_y);
  return true;
}
}  // namespace

OccupancyGrid::OccupancyGrid(GridConfig config)
    : config_(config), grid_from_map_(inverse(config.map_from_grid)) {
  if (config.width == 0 || config.height == 0 || config.width > kMaximumCells ||
      config.height > kMaximumCells / config.width ||
      !std::isfinite(config.resolution_m) || config.resolution_m <= 0 ||
      !std::isfinite(config.map_from_grid.yaw_rad) ||
      !config.map_from_grid.translation.allFinite() ||
      !std::isfinite(config.hit_log_odds) || config.hit_log_odds <= 0 ||
      !std::isfinite(config.free_log_odds) || config.free_log_odds >= 0 ||
      !std::isfinite(config.min_log_odds) || !std::isfinite(config.max_log_odds) ||
      config.min_log_odds >= 0 || config.max_log_odds <= 0 ||
      config.min_log_odds >= config.max_log_odds ||
      !std::isfinite(static_cast<double>(config.width) * config.resolution_m) ||
      !std::isfinite(static_cast<double>(config.height) * config.resolution_m)) {
    throw std::invalid_argument("invalid occupancy grid configuration");
  }
  cells_.resize(config.width * config.height);
}

std::optional<std::size_t> OccupancyGrid::index_local(const Eigen::Vector2d& p) const noexcept {
  const double gx = p.x() / config_.resolution_m;
  const double gy = p.y() / config_.resolution_m;
  if (!std::isfinite(gx) || !std::isfinite(gy) || gx < 0 || gy < 0 ||
      gx >= config_.width || gy >= config_.height) return std::nullopt;
  return static_cast<std::size_t>(gy) * config_.width + static_cast<std::size_t>(gx);
}

std::optional<Cell> OccupancyGrid::cell(std::size_t x, std::size_t y) const noexcept {
  if (x >= config_.width || y >= config_.height) return std::nullopt;
  return cells_[y * config_.width + x];
}

std::optional<Cell> OccupancyGrid::at_world(const Eigen::Vector2d& p) const noexcept {
  const auto index = index_local(transform(grid_from_map_, p));
  return index ? std::optional<Cell>(cells_[*index]) : std::nullopt;
}

InsertStats OccupancyGrid::integrate_rays(const std::vector<WorldRay>& rays) {
  std::unordered_set<std::size_t> hits, frees;
  InsertStats stats;
  const double width = static_cast<double>(config_.width) * config_.resolution_m;
  const double height = static_cast<double>(config_.height) * config_.resolution_m;
  for (const auto& ray : rays) {
    if (ray.kind == sensors::ReturnKind::invalid || ray.kind == sensors::ReturnKind::missing ||
        (ray.kind == sensors::ReturnKind::no_return && !config_.clear_trusted_no_returns)) {
      ++stats.ignored_rays;
      continue;
    }
    if (!ray.origin_map.allFinite() || !ray.endpoint_map.allFinite()) {
      throw std::invalid_argument("non-finite occupancy ray");
    }
    Eigen::Vector2d a = transform(grid_from_map_, ray.origin_map);
    Eigen::Vector2d b = transform(grid_from_map_, ray.endpoint_map);
    const auto endpoint_cell = index_local(b);
    if ((b - a).norm() <= 1e-9 || !clip_to_grid(a, b, width, height)) {
      ++stats.ignored_rays;
      continue;
    }
    const auto first = index_local(a), last = index_local(b);
    if (!first || !last) { ++stats.ignored_rays; continue; }
    int x = static_cast<int>(*first % config_.width);
    int y = static_cast<int>(*first / config_.width);
    const int target_x = static_cast<int>(*last % config_.width);
    const int target_y = static_cast<int>(*last / config_.width);
    const int dx = std::abs(target_x - x), dy = std::abs(target_y - y);
    const int step_x = x < target_x ? 1 : -1;
    const int step_y = y < target_y ? 1 : -1;
    int err = dx - dy;
    while (true) {
      const auto index = static_cast<std::size_t>(y) * config_.width + static_cast<std::size_t>(x);
      const bool terminal_hit = ray.kind == sensors::ReturnKind::hit && endpoint_cell &&
                                index == *endpoint_cell;
      if (terminal_hit) hits.insert(index);
      else frees.insert(index);
      if (x == target_x && y == target_y) break;
      const int doubled = 2 * err;
      if (doubled > -dy) { err -= dy; x += step_x; }
      if (doubled < dx) { err += dx; y += step_y; }
    }
  }
  for (const auto index : hits) frees.erase(index); // hit precedence independent of ray order
  for (const auto index : frees) {
    auto& c = cells_[index];
    c.log_odds = std::clamp(c.log_odds + config_.free_log_odds,
                             config_.min_log_odds, config_.max_log_odds);
    c.observed = true;
  }
  for (const auto index : hits) {
    auto& c = cells_[index];
    c.log_odds = std::clamp(c.log_odds + config_.hit_log_odds,
                             config_.min_log_odds, config_.max_log_odds);
    c.observed = true;
  }
  stats.hit_cells = hits.size();
  stats.free_cells = frees.size();
  if (!hits.empty() || !frees.empty()) ++revision_;
  return stats;
}
}  // namespace slam::mapping
