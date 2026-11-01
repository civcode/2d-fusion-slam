#include "slam/datasets/carmen_map.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string_view>

namespace slam::datasets {
namespace {
std::uint32_t u32(std::span<const std::uint8_t> bytes, std::size_t index) {
  if (index > bytes.size() || bytes.size() - index < 4)
    throw std::invalid_argument("truncated CARMEN map field");
  return static_cast<std::uint32_t>(bytes[index]) |
         (static_cast<std::uint32_t>(bytes[index + 1]) << 8) |
         (static_cast<std::uint32_t>(bytes[index + 2]) << 16) |
         (static_cast<std::uint32_t>(bytes[index + 3]) << 24);
}
std::size_t locate(std::span<const std::uint8_t> bytes, std::string_view needle) {
  const auto it = std::search(bytes.begin(),bytes.end(),needle.begin(),needle.end());
  return it == bytes.end() ? bytes.size() : static_cast<std::size_t>(it - bytes.begin());
}
}
CarmenMap parse_carmen_map(std::span<const std::uint8_t> bytes) {
  constexpr std::string_view magic="CARMENMAPFILEv020 ";
  constexpr std::string_view grid="GRIDMAP   ";
  const auto magic_position=locate(bytes,magic);
  const auto grid_position=locate(bytes,grid);
  if (magic_position == bytes.size() || grid_position == bytes.size() ||
      grid_position <= magic_position || grid_position > magic_position + 8192)
    throw std::invalid_argument("not a supported CARMENMAPFILEv020 grid");
  const auto start=grid_position + grid.size();
  if (start > bytes.size() || bytes.size() - start < 12)
    throw std::invalid_argument("truncated CARMEN GRIDMAP header");
  CarmenMap map;
  map.width=u32(bytes,start);
  map.height=u32(bytes,start+4);
  map.resolution_m=std::bit_cast<float>(u32(bytes,start+8));
  if(map.width==0 || map.height==0 ||
     map.width>10000 || map.height>10000 ||
     !std::isfinite(map.resolution_m) || map.resolution_m<=0 || map.resolution_m>10)
    throw std::invalid_argument("invalid CARMEN map dimensions/resolution");
  const auto count=static_cast<std::size_t>(map.width)*map.height;
  if(count>16000000 || count>(bytes.size()-start-12)/4)
    throw std::invalid_argument("truncated or oversized CARMEN map payload");
  map.values_in_file_order.reserve(count);
  for(std::size_t i=0;i<count;++i) {
    const float v=std::bit_cast<float>(u32(bytes,start+12+i*4));
    if(v==-1.0F) ++map.unknown_cells;
    else if(std::isfinite(v) && v>=0 && v<=1) ++map.known_cells;
    else throw std::invalid_argument("invalid CARMEN occupancy value");
    map.values_in_file_order.push_back(v);
  }
  return map;
}
} // namespace slam::datasets
