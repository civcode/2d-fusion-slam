#include "slam/datasets/carmen.hpp"

#include <catch2/catch_test_macros.hpp>
#include <sstream>
#include <stdexcept>

using namespace slam::datasets;
TEST_CASE("CARMEN parses real record layout without inventing per-beam clock") {
  CarmenReader reader;
  const auto scan=reader.parse_laser("FLASER 3 1.2 0 81.91 1 2 0.3 4 5 0.6 251.25 host 0.05");
  REQUIRE(scan);
  REQUIRE(scan->ranges.size()==3);
  REQUIRE(scan->ranges[0].status==RangeStatus::hit);
  REQUIRE(scan->ranges[1].status==RangeStatus::invalid);
  REQUIRE(scan->ranges[2].status==RangeStatus::invalid);
  REQUIRE_FALSE(scan->per_beam_time_known);
  REQUIRE(scan->ipc_seconds==251.25);
  REQUIRE(scan->logged_laser_pose.x==1);
  REQUIRE(scan->logged_odometry_pose.x==4);
}
TEST_CASE("CARMEN strict malformed scan and valid odometry") {
  CarmenReader reader;
  REQUIRE_THROWS_AS(reader.parse_laser("FLASER 360 1.0"),std::invalid_argument);
  REQUIRE_THROWS_AS(reader.parse_laser("FLASER 2 nan 3 0 0 0 0 0 0 1 h 2"),std::invalid_argument);
  const auto o=reader.parse_odom("ODOM 1 2 0.3 0.4 0.5 0.6 251.3 magnum 0.07");
  REQUIRE(o);
  REQUIRE(o->pose.y==2);
  std::istringstream input("# comment\nPARAM laser_front_laser_type LMS h 0\n"
                           "FLASER 2 1 2 0 0 0 0 0 0 251 magnum 0.1\n"
                           "ODOM 1 2 0.3 0.4 0.5 0.6 251.3 magnum 0.07\n");
  const auto counts=reader.inspect(input);
  REQUIRE(counts.scans==1);
  REQUIRE(counts.odometry==1);
  REQUIRE(counts.hits==2);
  REQUIRE(counts.ignored==2);
}
