#include "slam/datasets/carmen_map.hpp"
#include <catch2/catch_test_macros.hpp>
#include <bit>
#include <cstdint>
#include <string>
#include <vector>
using namespace slam::datasets;
TEST_CASE("CARMEN binary v020 map preserves unknown versus known values") {
  std::string header="### reference map\nCARMENMAPFILEv020 0123456789GRIDMAP   ";
  std::vector<std::uint8_t> bytes(header.begin(),header.end());
  auto add32=[&](std::uint32_t x){for(unsigned i=0;i<4;++i)bytes.push_back(static_cast<std::uint8_t>(x>>(8*i)));};
  add32(2);add32(2);add32(std::bit_cast<std::uint32_t>(0.05f));
  add32(std::bit_cast<std::uint32_t>(-1.f));
  add32(std::bit_cast<std::uint32_t>(0.25f));
  add32(std::bit_cast<std::uint32_t>(0.95f));
  add32(std::bit_cast<std::uint32_t>(0.f));
  const auto m=parse_carmen_map(bytes);
  REQUIRE(m.width==2);REQUIRE(m.height==2);
  REQUIRE(m.unknown_cells==1);REQUIRE(m.known_cells==3);
  REQUIRE(m.values_in_file_order[0]==-1.f);
  REQUIRE(m.at_xy(0,0).value()==-1.f);
  REQUIRE(m.at_xy(0,1).value()==0.25f);
  REQUIRE(m.at_xy(1,0).value()==0.95f);
  REQUIRE(m.at_xy(1,1).value()==0.f);
  REQUIRE_FALSE(m.at_xy(2,0));
  REQUIRE(m.image_row_from_y(0).value()==1);
  REQUIRE(m.image_row_from_y(1).value()==0);
  REQUIRE_FALSE(m.has_metric_origin());
  bytes.pop_back();
  REQUIRE_THROWS(parse_carmen_map(bytes));
}
