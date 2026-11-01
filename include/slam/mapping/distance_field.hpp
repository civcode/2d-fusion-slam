#pragma once

#include "slam/mapping/occupancy.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace slam::mapping {

// A coherent deep copy of authoritative occupancy. Once published, no client
// can modify a cell or change the revision. Construction and insertion in the
// mutable grid must be serialized by its owner (not concurrent).
class MapSnapshot final {
 public:
  [[nodiscard]] static std::shared_ptr<const MapSnapshot> capture(const OccupancyGrid& grid);
  // Reconstruct authoritative cells from a validated, versioned bundle.
  [[nodiscard]] static std::shared_ptr<const MapSnapshot> from_cells(
      GridConfig config, std::uint64_t revision, std::vector<Cell> cells);
  [[nodiscard]] const GridConfig& config() const noexcept { return config_; }
  [[nodiscard]] std::uint64_t revision() const noexcept { return revision_; }
  [[nodiscard]] std::uint64_t fingerprint() const noexcept { return fingerprint_; }
  [[nodiscard]] std::optional<Cell> cell(std::size_t x, std::size_t y) const noexcept;
  [[nodiscard]] const std::vector<Cell>& cells() const noexcept { return cells_; }

 private:
  MapSnapshot(GridConfig config, std::uint64_t revision, std::vector<Cell> cells);
  GridConfig config_;
  std::uint64_t revision_{};
  std::uint64_t fingerprint_{}; // deterministic non-cryptographic content identifier
  std::vector<Cell> cells_;
};

struct DistanceSample {
  double metres{};
  Eigen::Vector2d gradient_map{Eigen::Vector2d::Zero()}; // metres per map metre
};

// Exact centre-to-centre Euclidean distance transform on all occupied known
// cells. Field values are derived caches, not occupancy authority.
class DistanceField final {
 public:
  [[nodiscard]] static std::shared_ptr<const DistanceField> build(
      std::shared_ptr<const MapSnapshot> source, double truncation_metres = 0.5,
      double occupied_log_odds_threshold = 0.0);
  [[nodiscard]] const std::shared_ptr<const MapSnapshot>& source() const noexcept { return source_; }
  [[nodiscard]] double truncation_metres() const noexcept { return truncation_metres_; }
  [[nodiscard]] bool has_obstacles() const noexcept { return has_obstacles_; }
  [[nodiscard]] std::optional<double> cell_distance(std::size_t x, std::size_t y) const noexcept;
  // Bilinear interpolation only with all four observed cells and an
  // informative (nontruncated) distance. Unknown or outside never becomes 0.
  [[nodiscard]] std::optional<DistanceSample> sample_map(const Eigen::Vector2d& point) const noexcept;

 private:
  DistanceField(std::shared_ptr<const MapSnapshot> source, double truncation,
                double occupied_threshold);
  std::shared_ptr<const MapSnapshot> source_;
  Pose2 grid_from_map_;
  double truncation_metres_{};
  bool has_obstacles_{};
  std::vector<double> values_; // metres, truncated for cache storage
};

}  // namespace slam::mapping
