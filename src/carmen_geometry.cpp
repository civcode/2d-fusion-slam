#include "slam/datasets/carmen_geometry.hpp"
#include <cmath>
#include <stdexcept>

namespace slam::datasets {
CarmenPointCloud carmen_laser_points(const CarmenLaser& scan) {
  if (!std::isfinite(scan.ipc_seconds)) throw std::invalid_argument("invalid scan IPC time");
  CarmenPointCloud output;
  output.scan_ipc_seconds = scan.ipc_seconds;
  output.per_beam_time_known = false; // preserve truth of CARMEN source
  output.hit_points_laser.reserve(scan.ranges.size());
  for(const auto& r : scan.ranges) {
    if (r.status != RangeStatus::hit) { ++output.rejected_ranges; continue; }
    if (!std::isfinite(r.metres) || r.metres <= 0 || !std::isfinite(r.angle_rad))
      throw std::invalid_argument("invalid hit range/angle");
    output.hit_points_laser.emplace_back(r.metres*std::cos(r.angle_rad),
                                         r.metres*std::sin(r.angle_rad));
  }
  return output;
}
} // namespace slam::datasets
