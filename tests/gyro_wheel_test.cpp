#include "slam/fusion/gyro_wheel.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
using slam::Timestamp;
using slam::fusion::SensorTick;
using slam::fusion::integrate_sensors;

void require(bool ok,const char* msg) {
  if(!ok)throw std::runtime_error(msg);
}
template<typename F> void must_refuse(F fn) {
  bool refused=false;try{fn();}catch(const std::invalid_argument&){refused=true;}
  require(refused,"invalid fusion evidence was accepted");
}
std::vector<SensorTick> make_ticks() {
  std::vector<SensorTick> data;
  for(int i=0;i<40;++i) {
    const bool stationary=i<8;
    SensorTick s;
    s.time={1'000'000'000LL+25'000'000LL*i,3,7};
    s.gyro_valid=true;s.wheel_valid=true;
    s.stationary_calibration=stationary;
    s.gyro_z_rad_s=stationary?.035:.135; // true yaw 0.1 + bias .035
    s.wheel_yaw_rad_s=stationary?0:.12; // noisier wheel yaw
    s.wheel_forward_m_s=stationary?0:.8;
    data.push_back(s);
  }
  return data;
}
}
int main() {
  const auto data=make_ticks();
  const slam::Pose2 anchor{{2,3},.2};
  const auto fused=integrate_sensors(data,anchor,true);
  require(fused.fused.size()==data.size(),"fused tick count");
  require(fused.wheel_only.size()==data.size(),"wheel tick count");
  require(std::abs(fused.estimated_gyro_bias_rad_s-.035)<1e-12,"bias estimation");
  require(fused.capability==slam::motion::TrajectoryCapability::full_se2,"missing full se2");
  require(fused.stationary_ticks==8,"calibration tick count");
  require((fused.fused.front().odom_from_base.translation-anchor.translation).norm()<1e-12,
          "initial pose moved");
  require(fused.fused.back().odom_from_base.translation.x()>2.3,"gyro-wheel translation");
  require(fused.fused.back().odom_from_base.yaw_rad <
          fused.wheel_only.back().odom_from_base.yaw_rad,"gyro should reduce yaw drift");
  slam::motion::SampledTrajectory trajectory(fused.fused,fused.capability);
  require(trajectory.at({1'250'000'000LL,3,7}).has_value(),"valid sample interpolation");
  require(!trajectory.at({1'250'000'000LL,4,7}).has_value(),"domain contamination");
  require(!trajectory.at({500'000'000LL,3,7}).has_value(),"unauthorized extrapolation");

  must_refuse([&]{(void)integrate_sensors(data,anchor,false);});
  auto bad=data;
  bad[11].time.clock_domain=4;
  must_refuse([&]{(void)integrate_sensors(bad,anchor,true);});
  bad=data;bad[11].time.epoch=8;
  must_refuse([&]{(void)integrate_sensors(bad,anchor,true);});
  bad=data;bad[11].time.ns=bad[10].time.ns;
  must_refuse([&]{(void)integrate_sensors(bad,anchor,true);});
  bad=data;bad[11].time.ns+=200'000'000LL;
  must_refuse([&]{(void)integrate_sensors(bad,anchor,true);});
  bad=data;bad[11].gyro_valid=false;
  must_refuse([&]{(void)integrate_sensors(bad,anchor,true);});
  bad=data;bad[0].stationary_calibration=false;
  must_refuse([&]{(void)integrate_sensors(bad,anchor,true);});
  bad=data;bad[0].gyro_z_rad_s=3.5; // mean bias exceeds configured .25 rad/s
  must_refuse([&]{(void)integrate_sensors(bad,anchor,true);});
  bad=data;bad[15].wheel_valid=false;
  must_refuse([&]{(void)integrate_sensors(bad,anchor,true);});
  slam::fusion::FusionOptions only_gyro;only_gyro.permit_gyro_only_diagnostics=true;
  const auto gyro_only=integrate_sensors(bad,anchor,true,only_gyro);
  require(gyro_only.capability==slam::motion::TrajectoryCapability::gyro_only,
          "missing wheel data allowed fake full SE2");
  slam::motion::SampledTrajectory gyro_path(gyro_only.fused,gyro_only.capability);
  require(gyro_path.at(data[24].time).has_value(),"gyro-only diagnostic unavailable");
  require(std::abs(gyro_only.fused.back().odom_from_base.translation.x()-anchor.translation.x())<1e-12,
          "gyro only invented translation");
  std::cout<<"gyro/wheel fusion evidence, bias and clock regressions passed\n";
}
