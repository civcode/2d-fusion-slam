#include "slam/fusion/gyro_wheel.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <variant>

namespace slam::fusion {
namespace {
bool finite_pose(const Pose2& p) {
  return p.translation.allFinite() && std::isfinite(p.yaw_rad);
}
bool valid(const SensorTick& t, const FusionOptions& o) {
  return t.gyro_valid && std::isfinite(t.gyro_z_rad_s) &&
         std::abs(t.gyro_z_rad_s) <= o.maximum_gyro_abs_rate_rad_s &&
         (!t.wheel_valid ||
          (std::isfinite(t.wheel_forward_m_s) &&
           std::abs(t.wheel_forward_m_s) <= o.maximum_wheel_speed_m_s &&
           std::isfinite(t.wheel_yaw_rad_s) &&
           std::abs(t.wheel_yaw_rad_s) <= o.maximum_gyro_abs_rate_rad_s));
}
} // namespace

FusionResult integrate_sensors(const std::vector<SensorTick>& ticks,
                               Pose2 initial, bool clock_and_frame_verified,
                               const FusionOptions& options) {
  if (!clock_and_frame_verified || !finite_pose(initial) || ticks.size()<2 ||
      ticks.size()>1'000'000 || options.minimum_stationary_ticks<4 ||
      options.minimum_stationary_ticks>=ticks.size() ||
      options.maximum_interval_ns<=0 || options.maximum_interval_ns>500'000'000 ||
      !std::isfinite(options.maximum_gyro_abs_rate_rad_s) ||
      options.maximum_gyro_abs_rate_rad_s<=0 ||
      !std::isfinite(options.maximum_wheel_speed_m_s) ||
      options.maximum_wheel_speed_m_s<=0 ||
      !std::isfinite(options.maximum_bias_abs_rad_s) ||
      options.maximum_bias_abs_rad_s<0)
    throw std::invalid_argument("invalid gyro/wheel clock, frame or calibration contract");

  std::size_t stationary=0;
  double bias_sum=0;
  bool wheel_supported=true;
  for (std::size_t i=0;i<ticks.size();++i) {
    const auto& t=ticks[i];
    if (!valid(t,options))
      throw std::invalid_argument("missing/invalid gyro or wheel data");
    if (!t.wheel_valid) wheel_supported=false;
    if (i>0) {
      const auto delta=between(t.time,ticks[i-1].time);
      if (!std::holds_alternative<DurationNs>(delta) ||
          std::get<DurationNs>(delta).value<=0 ||
          std::get<DurationNs>(delta).value>options.maximum_interval_ns)
        throw std::invalid_argument("sensor clocks differ, reverse or exceed max gap");
    }
    if (i<options.minimum_stationary_ticks) {
      if (!t.stationary_calibration || !t.wheel_valid ||
          std::abs(t.wheel_forward_m_s)>0.03 ||
          std::abs(t.wheel_yaw_rad_s)>0.03)
        throw std::invalid_argument("gyro bias requires verified stationary wheel ticks");
      bias_sum+=t.gyro_z_rad_s;
      ++stationary;
    }
  }
  if (!wheel_supported && !options.permit_gyro_only_diagnostics)
    throw std::invalid_argument("missing wheel translation: moving deskew disallowed");
  const double bias=bias_sum/static_cast<double>(stationary);
  if (!std::isfinite(bias) || std::abs(bias)>options.maximum_bias_abs_rad_s)
    throw std::invalid_argument("gyro stationary bias outside calibrated bound");

  FusionResult out;
  out.capability=wheel_supported?
    motion::TrajectoryCapability::full_se2:
    motion::TrajectoryCapability::gyro_only;
  out.estimated_gyro_bias_rad_s=bias;
  out.stationary_ticks=stationary;
  out.frame_and_clock_attested=true;
  out.fused.reserve(ticks.size());
  out.wheel_only.reserve(ticks.size());
  Pose2 fused=initial,wheel=initial;
  out.fused.push_back({ticks.front().time,fused});
  out.wheel_only.push_back({ticks.front().time,wheel});
  for(std::size_t i=1;i<ticks.size();++i){
    const auto dt=std::get<DurationNs>(between(ticks[i].time,ticks[i-1].time));
    const double seconds_dt=seconds(dt);
    const double rate=0.5*(ticks[i-1].gyro_z_rad_s+ticks[i].gyro_z_rad_s)-bias;
    const double v=wheel_supported?
      .5*(ticks[i-1].wheel_forward_m_s+ticks[i].wheel_forward_m_s):0.0;
    // Whole interval midpoint twist integration: one time domain and one
    // measurement provenance, no mixing instantaneous yaw with un-timestamped XY.
    fused=integrate_body_twist(fused,{v,0,rate},seconds_dt);
    if(wheel_supported) {
      const double omega=.5*(ticks[i-1].wheel_yaw_rad_s+ticks[i].wheel_yaw_rad_s);
      wheel=integrate_body_twist(wheel,{v,0,omega},seconds_dt);
    } else wheel=fused; // wheel-only unavailable when capability is gyro_only
    if(!finite_pose(fused) || !finite_pose(wheel))
      throw std::runtime_error("gyro/wheel integration diverged");
    out.fused.push_back({ticks[i].time,fused});
    out.wheel_only.push_back({ticks[i].time,wheel});
  }
  return out;
}
} // namespace slam::fusion
