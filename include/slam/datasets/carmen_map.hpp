#pragma once
#include <cstddef>
#include <cstdint>
#include <span>
#include <optional>
#include <vector>

namespace slam::datasets {
// Native CARMENMAPFILEv020 GRIDMAP values. -1 is unknown, [0,1] is
// a measured grid value. Axis registration and occupancy semantics still
// require external validation before using a reference map for scoring.
struct CarmenMap {
  std::uint32_t width{}, height{};
  double resolution_m{};
  std::vector<float> values_in_file_order;
  std::size_t known_cells{}, unknown_cells{};
  // Verified against supplied map05cm.gif: CARMEN stores X-major columns,
  // increasing Y upward. An image's row 0 is at grid Y=height-1.
  [[nodiscard]] std::optional<float> at_xy(std::size_t x, std::size_t y) const noexcept {
    if (x >= width || y >= height) return std::nullopt;
    return values_in_file_order[x * height + y];
  }
  [[nodiscard]] std::optional<std::size_t> image_row_from_y(std::size_t y) const noexcept {
    if (y >= height) return std::nullopt;
    return height - 1 - y;
  }
  // Binary header has no validated map->laser frame translation.
  [[nodiscard]] bool has_metric_origin() const noexcept { return false; }
};
[[nodiscard]] CarmenMap parse_carmen_map(std::span<const std::uint8_t> uncompressed_bytes);
} // namespace slam::datasets
