#pragma once

#include "slam/core/se2.hpp"
#include "slam/sensors/scan.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace slam::mapping {

struct GridConfig {
  std::size_t width{300};
  std::size_t height{240};
  double resolution_m{0.05};
  Pose2 map_from_grid{};   // local grid (0,0) in map frame; cells are half-open
  double hit_log_odds{0.85};
  double free_log_odds{-0.4};
  double min_log_odds{-5.0};
  double max_log_odds{5.0};
  bool clear_trusted_no_returns{false}; // explicit sensor-specific opt-in
};

struct Cell {
  double log_odds{};
  bool observed{};          // independent of whether probability rounds to 0.5
};

struct WorldRay {
  Eigen::Vector2d origin_map;
  Eigen::Vector2d endpoint_map;
  sensors::ReturnKind kind{sensors::ReturnKind::invalid};
};

struct InsertStats {
  std::size_t hit_cells{};
  std::size_t free_cells{};
  std::size_t ignored_rays{};
};

// Authoritative occupancy: never assumes raw moving beams are deskewed.
// Callers must supply correctly transformed, time-valid world rays. No frame
// or pose extrapolation is performed here.
class OccupancyGrid {
 public:
  explicit OccupancyGrid(GridConfig config);
  [[nodiscard]] const GridConfig& config() const noexcept { return config_; }
  [[nodiscard]] std::uint64_t revision() const noexcept { return revision_; }
  [[nodiscard]] std::optional<Cell> cell(std::size_t x, std::size_t y) const noexcept;
  [[nodiscard]] std::optional<Cell> at_world(const Eigen::Vector2d& point_map) const noexcept;
  // Deep-copy for publication. Caller must serialize this with integrate_rays().
  [[nodiscard]] std::vector<Cell> snapshot_cells() const { return cells_; }
  [[nodiscard]] InsertStats integrate_rays(const std::vector<WorldRay>& rays);

 private:
  [[nodiscard]] std::optional<std::size_t> index_local(const Eigen::Vector2d& point) const noexcept;
  GridConfig config_;
  Pose2 grid_from_map_;
  std::vector<Cell> cells_;
  std::uint64_t revision_{};
};

}  // namespace slam::mapping
