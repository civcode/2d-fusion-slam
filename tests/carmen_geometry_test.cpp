#include "slam/datasets/carmen_geometry.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
using Catch::Approx;
using namespace slam::datasets;
TEST_CASE("CARMEN scan converts to native geometry without claiming beam timing") {
 const auto scan=CarmenReader{}.parse_laser("FLASER 3 2 0 2 1 2 0.3 4 5 0.6 251.25 host 0.05");
 REQUIRE(scan);
 const auto cloud=carmen_laser_points(*scan);
 REQUIRE(cloud.hit_points_laser.size()==2);
 REQUIRE(cloud.rejected_ranges==1);
 REQUIRE_FALSE(cloud.per_beam_time_known);
 REQUIRE(cloud.hit_points_laser[0].x()==Approx(0).margin(1e-10));
 REQUIRE(cloud.hit_points_laser[0].y()==Approx(-2));
 REQUIRE(cloud.hit_points_laser[1].y()==Approx(1));
}
