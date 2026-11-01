#include "slam/datasets/carmen.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace slam::datasets {
namespace {
void ensure(bool ok, const char* message) {
  if (!ok) throw std::invalid_argument(message);
}
double number(std::istream& in, const char* field) {
  double v{};
  if (!(in >> v) || !std::isfinite(v)) throw std::invalid_argument(std::string("invalid ") + field);
  return v;
}
Pose2Record pose(std::istream& in) {
  return {number(in,"x"),number(in,"y"),number(in,"yaw")};
}
void end_of_record(std::istream& in) {
  std::string extra;
  ensure(!(in >> extra), "unexpected record suffix");
}
}
CarmenReader::CarmenReader(CarmenOptions options):options_(options) {
  ensure(std::isfinite(options.field_of_view_rad) && options.field_of_view_rad > 0 &&
         options.field_of_view_rad <= 2 * 3.14159265358979323846 &&
         std::isfinite(options.max_usable_range_m) && options.max_usable_range_m > 0 &&
         options.maximum_ranges > 0 && options.maximum_ranges <= 100000 &&
         options.maximum_line_bytes > 0, "invalid CARMEN reader options");
}
std::optional<CarmenLaser> CarmenReader::parse_laser(const std::string& line) const {
  std::istringstream in(line);
  std::string kind;
  if (!(in >> kind) || (kind != "FLASER" && kind != "RLASER")) return std::nullopt;
  long long count{};
  ensure(static_cast<bool>(in >> count) && count > 0 &&
         static_cast<unsigned long long>(count) <= options_.maximum_ranges, "invalid laser beam count");
  CarmenLaser scan;
  scan.ranges.reserve(static_cast<std::size_t>(count));
  for (long long i = 0; i < count; ++i) {
    const double distance = number(in,"range");
    const double angle = -options_.field_of_view_rad / 2 +
        options_.field_of_view_rad * static_cast<double>(i) / static_cast<double>(count);
    scan.ranges.push_back({distance,angle,
      distance > 0 && distance <= options_.max_usable_range_m ? RangeStatus::hit : RangeStatus::invalid});
  }
  scan.logged_laser_pose = pose(in);
  scan.logged_odometry_pose = pose(in);
  scan.ipc_seconds = number(in,"IPC timestamp");
  ensure(static_cast<bool>(in >> scan.ipc_host) && !scan.ipc_host.empty(),"missing IPC host");
  scan.logger_seconds = number(in,"logger timestamp");
  end_of_record(in);
  return scan;
}
std::optional<CarmenOdom> CarmenReader::parse_odom(const std::string& line) const {
  std::istringstream in(line);
  std::string kind;
  if (!(in >> kind) || kind != "ODOM") return std::nullopt;
  CarmenOdom odom;
  odom.pose = pose(in);
  odom.translation_velocity = number(in,"translation velocity");
  odom.rotation_velocity = number(in,"rotation velocity");
  odom.acceleration = number(in,"acceleration");
  odom.ipc_seconds = number(in,"IPC timestamp");
  std::string host;
  ensure(static_cast<bool>(in >> host) && !host.empty(),"missing IPC host");
  odom.logger_seconds = number(in,"logger timestamp");
  end_of_record(in);
  return odom;
}
CarmenStatistics CarmenReader::inspect(std::istream& input) const {
  CarmenStatistics stats;
  std::string line;
  while (std::getline(input,line)) {
    ++stats.lines;
    if (line.size() > options_.maximum_line_bytes)
      throw std::invalid_argument("CARMEN line too long at line " + std::to_string(stats.lines));
    try {
      if (auto scan = parse_laser(line)) {
        if (stats.scans == 0) {
          stats.first_scan_seconds = scan->ipc_seconds;
          stats.minimum_beams = scan->ranges.size();
        }
        ++stats.scans;
        stats.last_scan_seconds = scan->ipc_seconds;
        stats.maximum_beams = std::max(stats.maximum_beams,scan->ranges.size());
        stats.minimum_beams = std::min(stats.minimum_beams,scan->ranges.size());
        stats.ranges += scan->ranges.size();
        for (const auto& v : scan->ranges) {
          if (v.status == RangeStatus::hit) ++stats.hits;
          else ++stats.invalid_ranges;
        }
      } else if (parse_odom(line)) ++stats.odometry;
      else ++stats.ignored;
    } catch(const std::invalid_argument& e) {
      throw std::invalid_argument("CARMEN line " + std::to_string(stats.lines) + ": " + e.what());
    }
  }
  if (input.bad()) throw std::runtime_error("failed to read CARMEN dataset");
  return stats;
}
} // namespace slam::datasets
