#include "slam/fixtures/synthetic.hpp"

#include <algorithm>
#include <array>
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

// A small, asymmetric office with off-axis partitions and short corridors.
// The test trajectory occupies the open southwest entry zone.
Scene office() {
  Scene scene{"synthetic-office-v1", {}};
  rectangle(scene,0,0,15,12);
  wall(scene,5.5,0,5.5,4.5);
  wall(scene,5.5,6.0,5.5,12);
  wall(scene,5.5,6.0,10.0,6.0);
  wall(scene,11.3,6.0,15.0,6.0);
  wall(scene,9.2,0,9.2,3.2);
  wall(scene,9.2,4.5,9.2,6.0);
  wall(scene,10.0,6.0,10.0,9.1);
  wall(scene,10.0,10.5,10.0,12);
  wall(scene,0,8.0,3.0,8.0);
  wall(scene,4.1,8.0,5.5,8.0);
  wall(scene,12.2,1.5,14.0,3.5); // non-axis-aligned detail
  return scene;
}

// Large open room around an irregular central furniture island. The robot
// travels around, not through, the furniture.
Scene loft() {
  Scene scene{"synthetic-loft-v1", {}};
  rectangle(scene,0,0,15,12);
  wall(scene,6.3,5.0,8.7,4.7);
  wall(scene,8.7,4.7,9.1,7.1);
  wall(scene,9.1,7.1,6.6,7.5);
  wall(scene,6.6,7.5,6.3,5.0);
  rectangle(scene,1.0,1.0,2.8,2.0);
  rectangle(scene,11.8,9.9,14.0,11.0);
  wall(scene,1.1,9.0,3.0,10.7);
  wall(scene,12.2,1.1,13.6,2.6);
  return scene;
}

// Irregular atrium with a central core, diagonal structural ribs and sparse
// outer detail. Unlike the warehouse, it has no repeated parallel shelves.
Scene atrium() {
  Scene scene{"synthetic-atrium-v1", {}};
  rectangle(scene,0,0,30,20);
  wall(scene,10.0,7.5,17.0,5.9);
  wall(scene,17.0,5.9,21.2,9.5);
  wall(scene,21.2,9.5,17.9,13.8);
  wall(scene,17.9,13.8,11.0,13.2);
  wall(scene,11.0,13.2,10.0,7.5);
  wall(scene,4.0,4.2,6.0,5.0);
  wall(scene,23.9,4.0,26.0,5.7);
  wall(scene,24.0,15.4,26.0,14.1);
  wall(scene,4.4,14.5,5.8,15.6);
  rectangle(scene,13.2,8.6,14.6,10.1);
  return scene;
}

// Unequal cabinets, angled machinery, uneven spacing; same large bounding
// area but no shelving lattice to supply repeated scan correspondences.
Scene industrial() {
  Scene scene{"synthetic-industrial-v1", {}};
  rectangle(scene,0,0,30,20);
  rectangle(scene,7.0,5.0,10.0,8.0);
  rectangle(scene,8.5,11.0,10.0,15.8);
  rectangle(scene,13.0,5.2,18.0,7.0);
  rectangle(scene,19.5,10.3,24.5,13.2);
  wall(scene,6.1,10.3,8.0,13.2);
  wall(scene,11.2,8.5,13.0,10.6);
  wall(scene,18.0,14.2,21.3,15.8);
  wall(scene,22.5,4.4,25.0,7.7);
  rectangle(scene,14.1,12.3,16.2,14.4);
  return scene;
}

// Matched-motion control for the original warehouse: break translational
// shelf symmetry while retaining the same scanner, route and outer bounds.
Scene warehouse_asymmetric() {
  Scene scene{"synthetic-warehouse_asymmetric-v1", {}};
  rectangle(scene,0,0,30,20);
  rectangle(scene,5.0,4.3,6.0,13.8);
  rectangle(scene,10.0,6.2,11.2,16.0);
  rectangle(scene,16.1,4.5,18.4,10.9);
  rectangle(scene,23.4,7.7,24.1,15.6);
  wall(scene,6.8,15.2,9.1,13.8);
  wall(scene,12.5,4.8,14.7,6.8);
  wall(scene,18.5,13.0,21.6,15.3);
  rectangle(scene,20.0,4.7,22.0,6.1);
  return scene;
}


Scene apartment_explore() {
  Scene scene=apartment(); // identical reference walls; route differs in fusion.
  scene.id="synthetic-apartment_explore-v1";
  return scene;
}

// The C++ occupancy-scoring reference is the accessible surface of the
// exact 10 cm synthetic workshop raster from apps/demo/web/example.py.
// A free/occupied boundary is represented by one metric segment; unknown
// cells are not assumed free or counted as newly observed obstacles.
// Contiguous cell faces are merged to avoid thousands of tiny segments.
Scene workshop() {
  Scene scene{"synthetic-workshop-v1", {}};
  constexpr int width=120,height=90;
  constexpr double resolution=.1;
  using Raster=std::array<std::array<bool,width>,height>;
  Raster known_free{}, occupied{};
  auto inside=[](double x,double y,double a,double b,double c,double d) {
    return x>=a && x<=b && y>=c && y<=d;
  };
  for(int y=0;y<height;++y)for(int x=0;x<width;++x) {
    const double px=(x+.5)*resolution, py=(y+.5)*resolution;
    const bool known=inside(px,py,.5,11.5,.5,8.5) &&
        !(px>10.1 && py>6.0) && !(px<1.8 && py>6.1);
    if(!known)continue;
    const bool outer=px<.7 || px>11.3 || py<.7 || py>8.3;
    const bool furniture=
        inside(px,py,3.05,4.25,1.85,4.4) ||
        inside(px,py,6.65,8.15,4.0,5.35) ||
        inside(px,py,8.9,10.15,1.85,3.7) ||
        inside(px,py,5.5,6.0,6.6,8.1) ||
        inside(px,py,1.25,2.15,6.55,7.8);
    occupied[y][x]=outer || furniture;
    known_free[y][x]=!occupied[y][x];
  }
  // Horizontal free/occupied transitions.
  for(int y=1;y<height;++y) {
    int first=-1;
    for(int x=0;x<=width;++x) {
      const bool edge=x<width &&
          ((occupied[y-1][x] && known_free[y][x]) ||
           (occupied[y][x] && known_free[y-1][x]));
      if(edge && first<0)first=x;
      if(!edge && first>=0) {
        wall(scene,first*resolution,y*resolution,
             x*resolution,y*resolution);
        first=-1;
      }
    }
  }
  // Vertical free/occupied transitions.
  for(int x=1;x<width;++x) {
    int first=-1;
    for(int y=0;y<=height;++y) {
      const bool edge=y<height &&
          ((occupied[y][x-1] && known_free[y][x]) ||
           (occupied[y][x] && known_free[y][x-1]));
      if(edge && first<0)first=y;
      if(!edge && first>=0) {
        wall(scene,x*resolution,first*resolution,
             x*resolution,y*resolution);
        first=-1;
      }
    }
  }
  return scene;
}

Scene named_scene(const std::string& name) {
  if(name=="apartment")return apartment();
  if(name=="apartment_explore")return apartment_explore();
  if(name=="workshop")return workshop();
  if(name=="warehouse")return warehouse();
  if(name=="office")return office();
  if(name=="loft")return loft();
  if(name=="atrium")return atrium();
  if(name=="industrial")return industrial();
  if(name=="warehouse_asymmetric")return warehouse_asymmetric();
  throw std::invalid_argument("unknown synthetic scene: "+name);
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
