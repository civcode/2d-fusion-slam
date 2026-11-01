#include "slam/datasets/fr079_alignment.hpp"
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <utility>
#include <stdexcept>
#include <vector>

using namespace slam::datasets;
TEST_CASE("FR079 map registration separates calibration and held-out queries") {
  CarmenMap m;
  m.width=100; m.height=100; m.resolution_m=0.1;
  m.values_in_file_order.assign(10000,0.0f);m.known_cells=10000;
  std::vector<CarmenLaser> scans;
  for(int i=0;i<100;++i){
    CarmenLaser scan;
    scan.logged_laser_pose.x=static_cast<double>(i%10)*0.03;
    scan.logged_laser_pose.y=static_cast<double>(i/10)*0.03;
    scan.ranges={{1.0,0.0,RangeStatus::hit},{0.0,1.57,RangeStatus::invalid},
                 {1.5,0.4,RangeStatus::hit}};
    for(const auto& beam:scan.ranges){
      if(beam.status!=RangeStatus::hit)continue;
      const double x=scan.logged_laser_pose.x+beam.metres*std::cos(beam.angle_rad)+1.0;
      const double y=scan.logged_laser_pose.y+beam.metres*std::sin(beam.angle_rad)+2.0;
      m.values_in_file_order[static_cast<std::size_t>(x/0.1)*m.height+
                             static_cast<std::size_t>(y/0.1)]=0.95f;
    }
    scans.push_back(std::move(scan));
  }
  AlignmentOptions options;
  options.beam_stride=1;
  options.calibration_scan_stride=3;
  options.offset_x_min=0;options.offset_x_max=2;
  options.offset_y_min=0;options.offset_y_max=3;
  options.coarse_step_m=0.5;
  const auto r=diagnose_fr079_alignment(m,scans,options);
  REQUIRE(r.calibration_scans==80);
  REQUIRE(r.held_out_scans==20);
  REQUIRE(r.calibration.endpoints>0);
  REQUIRE(r.held_out.endpoints==40);
  REQUIRE(r.held_out.known_fraction()>0.9);
  REQUIRE(r.held_out.wall_fraction()>0);
  REQUIRE_FALSE(r.independent_ground_truth);
  options.holdout_modulus=1;
  REQUIRE_THROWS_AS(diagnose_fr079_alignment(m,scans,options),std::invalid_argument);
}
