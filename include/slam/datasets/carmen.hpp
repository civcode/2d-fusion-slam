#pragma once

#include <cstddef>
#include <cstdint>
#include <istream>
#include <optional>
#include <string>
#include <vector>

namespace slam::datasets {

// Public CARMEN dataset poses are logged estimates, NOT verified ground truth.
struct Pose2Record { double x{}, y{}, yaw{}; };
enum class RangeStatus { hit, invalid };
struct RangeRecord { double metres{}, angle_rad{}; RangeStatus status{RangeStatus::invalid}; };
struct CarmenLaser {
  std::vector<RangeRecord> ranges;
  Pose2Record logged_laser_pose;
  Pose2Record logged_odometry_pose;
  double ipc_seconds{};
  double logger_seconds{};
  std::string ipc_host;
  // Dataset has only a scan-level IPC timestamp. No fabricated per-beam timing.
  bool per_beam_time_known{false};
};
struct CarmenOdom {
  Pose2Record pose;
  double translation_velocity{}, rotation_velocity{}, acceleration{};
  double ipc_seconds{}, logger_seconds{};
};
struct CarmenStatistics {
  std::uint64_t lines{}, scans{}, odometry{}, ignored{}, ranges{}, hits{}, invalid_ranges{};
  std::size_t minimum_beams{}, maximum_beams{};
  double first_scan_seconds{}, last_scan_seconds{};
};

// Defaults do not assume max-range returns are valid traversable free space.
struct CarmenOptions {
  double field_of_view_rad{3.1415926535897932384626433832795};
  double max_usable_range_m{80.99};
  std::size_t maximum_ranges{4096};
  std::size_t maximum_line_bytes{1024 * 1024};
};

// Nonblank malformed FLASER/ODOM records throw with line number. PARAM and
// unknown CARMEN records are ignored; no ROS or LD19-specific assumptions.
class CarmenReader {
 public:
  explicit CarmenReader(CarmenOptions options = {});
  [[nodiscard]] CarmenStatistics inspect(std::istream& input) const;
  [[nodiscard]] std::optional<CarmenLaser> parse_laser(const std::string& line) const;
  [[nodiscard]] std::optional<CarmenOdom> parse_odom(const std::string& line) const;
 private:
  CarmenOptions options_;
};
} // namespace slam::datasets
