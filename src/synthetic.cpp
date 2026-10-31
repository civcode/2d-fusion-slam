#include "slam/fixtures/synthetic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace slam::fixtures {
namespace {
void wall(Scene& scene, double x0, double y0, double x1, double y1) {
  scene.walls.push_back({{x0, y0}, {x1, y1}});
}
void rectangle(Scene& scene, double left, double bottom, double right, double top) {
  wall(scene, left, bottom, right, bottom);
  wall(scene, right, bottom, right, top);
  wall(scene, right, top, left, top);
  wall(scene, left, top, left, bottom);
}
double cross(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
  return a.x() * b.y() - a.y() * b.x();
}
double ray_hit(const Eigen::Vector2d& p, const Eigen::Vector2d& direction,
               const Segment& wall) {
  const Eigen::Vector2d edge = wall.end - wall.start;
  const double denom = cross(direction, edge);
  if (std::abs(denom) < 1e-12) return std::numeric_limits<double>::infinity();
  const Eigen::Vector2d delta = wall.start - p;
  const double distance = cross(delta, edge) / denom;
  const double fraction = cross(delta, direction) / denom;
  if (distance >= 0 && fraction >= 0 && fraction <= 1) return distance;
  return std::numeric_limits<double>::infinity();
}
}  // namespace

Scene apartment() {
  Scene scene{"synthetic-apartment-v1", {}};
  rectangle(scene, 0, 0, 15, 12);
  // Interior wall segments leave real door gaps; no wall is inserted there.
  wall(scene, 5, 0, 5, 4);
  wall(scene, 5, 5.5, 5, 12);
  wall(scene, 10, 0, 10, 7);
  wall(scene, 10, 8.5, 10, 12);
  wall(scene, 0, 7, 2.5, 7);
  wall(scene, 4, 7, 5, 7);
  return scene;
}

Scene warehouse() {
  Scene scene{"synthetic-warehouse-v1", {}};
  rectangle(scene, 0, 0, 30, 20);
  for (double x : {6.0, 12.0, 18.0, 24.0}) {
    rectangle(scene, x, 4, x + 1, 16);
  }
  return scene;
}

Scan simulate_scan(const Scene& scene, Pose2 reference_pose, Tangent2 twist,
                   Pose2 base_to_laser, Timestamp reference_time,
                   std::size_t beam_count, TimeNs scan_duration_ns, double max_range_m) {
  if (beam_count < 2 || scan_duration_ns <= 0 || scan_duration_ns > 2'000'000'000LL ||
      !std::isfinite(max_range_m) || max_range_m <= 0) {
    throw std::invalid_argument("invalid synthetic scan configuration");
  }
  Scan scan{reference_pose, reference_time, {}};
  scan.beams.reserve(beam_count);
  // Avoid multiply overflow: duration <= 2e9 and beam index <= 1e6.
  if (beam_count > 1'000'000) throw std::invalid_argument("too many synthetic beams");
  const auto start_offset = -scan_duration_ns / 2;
  for (std::size_t i = 0; i < beam_count; ++i) {
    const auto offset = start_offset +
      static_cast<TimeNs>((scan_duration_ns * static_cast<TimeNs>(i)) /
                          static_cast<TimeNs>(beam_count - 1));
    const auto time = advance(reference_time, DurationNs{offset});
    if (!std::holds_alternative<Timestamp>(time)) throw std::overflow_error("synthetic timestamp overflow");
    const double relative_seconds = seconds(DurationNs{offset});
    const auto body = integrate_body_twist(reference_pose, twist, relative_seconds);
    const auto laser = compose(body, base_to_laser);
    const double angle = -std::numbers::pi + 2.0 * std::numbers::pi *
      static_cast<double>(i) / static_cast<double>(beam_count);
    const double heading = laser.yaw_rad + angle;
    const Eigen::Vector2d direction{std::cos(heading), std::sin(heading)};
    double distance = std::numeric_limits<double>::infinity();
    for (const auto& segment : scene.walls) {
      distance = std::min(distance, ray_hit(laser.translation, direction, segment));
    }
    const bool hit = std::isfinite(distance) && distance <= max_range_m;
    scan.beams.push_back({std::get<Timestamp>(time), angle,
                          hit ? distance : max_range_m, hit});
  }
  return scan;
}
}  // namespace slam::fixtures
