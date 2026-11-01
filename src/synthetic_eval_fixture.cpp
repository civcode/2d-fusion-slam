#include "slam/fixtures/synthetic.hpp"
#include "slam/mapping/map_bundle.hpp"
#include "slam/mapping/occupancy.hpp"

#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
std::vector<slam::Pose2> reference_poses(const std::string& name) {
  std::vector<slam::Pose2> poses;
  if (name == "apartment") {
    for (const double x : {2.0, 7.5, 12.5})
      for (const double y : {2.0, 6.0, 10.0})
        poses.push_back({{x, y}, 0.0});
  } else {
    for (const double x : {3.0, 9.0, 15.0, 21.0, 27.0})
      for (const double y : {2.0, 10.0, 18.0})
        poses.push_back({{x, y}, 0.0});
  }
  return poses;
}
} // namespace

// A deterministic map-building smoke fixture using *exact synthetic poses*.
// It is not a SLAM trajectory benchmark or a real sensor calibration.
int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: slam_synthetic_eval_fixture <apartment|warehouse> <output.slammap>\n";
    return 2;
  }
  try {
    const std::string name(argv[1]);
    slam::fixtures::Scene scene;
    if (name == "apartment") scene = slam::fixtures::apartment();
    else if (name == "warehouse") scene = slam::fixtures::warehouse();
    else throw std::invalid_argument("unknown synthetic scene");
    slam::mapping::GridConfig cfg;
    cfg.resolution_m = 0.05;
    cfg.map_from_grid = {{-0.5,-0.5},0.0};
    cfg.width = name == "apartment" ? 321 : 621;
    cfg.height = name == "apartment" ? 261 : 421;
    slam::mapping::OccupancyGrid grid(cfg);
    const auto poses = reference_poses(name);
    for (std::size_t i = 0; i < poses.size(); ++i) {
      const slam::Timestamp stamp{static_cast<slam::TimeNs>(i) * 200'000'000, 1, 1};
      const auto scan = slam::fixtures::simulate_scan(
          scene, poses[i], {}, {}, stamp, 360, 100'000'000, 30.0);
      std::vector<slam::mapping::WorldRay> rays;
      for (const auto& beam : scan.beams) {
        if (!beam.hit) continue; // no-return was not independently calibrated
        const Eigen::Vector2d end_laser{
            beam.range_m * std::cos(beam.angle_rad),
            beam.range_m * std::sin(beam.angle_rad)};
        rays.push_back({poses[i].translation, slam::transform(poses[i], end_laser),
                        slam::sensors::ReturnKind::hit});
      }
      (void)grid.integrate_rays(rays);
    }
    slam::mapping::MapBundle bundle{
        slam::mapping::MapSnapshot::capture(grid),
        {"map", scene.id, "synthetic_exact", true}};
    std::ofstream out(argv[2], std::ios::binary | std::ios::trunc);
    if (!out) throw std::runtime_error("cannot open fixture map output");
    slam::mapping::write_map_bundle(out, bundle);
    std::cerr << "Exported oracle-pose synthetic fixture " << scene.id
              << " (" << poses.size() << " stationary scans); not a SLAM pose benchmark\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Synthetic accuracy fixture failed: " << e.what() << '\n';
    return 1;
  }
}
