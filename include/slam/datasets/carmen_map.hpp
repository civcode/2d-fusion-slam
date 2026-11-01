#pragma once
#include <cstddef>
#include <cstdint>
#include <span>
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
};
[[nodiscard]] CarmenMap parse_carmen_map(std::span<const std::uint8_t> uncompressed_bytes);
} // namespace slam::datasets
