#include "slam/evaluation/map_accuracy.hpp"
#include "slam/fixtures/synthetic.hpp"
#include "slam/mapping/distance_field.hpp"
#include "slam/mapping/map_bundle.hpp"
#include "slam/matching/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
using slam::Pose2;
using slam::mapping::WorldRay;

struct Observation {
  std::vector<Eigen::Vector2d> laser_hits;
  Pose2 measured_previous_from_current{};
};

struct Dataset {
  slam::fixtures::Scene scene;
  slam::mapping::GridConfig grid;
  std::vector<Observation> scans;
  // Truth is held outside the mapping/matching functions. It is used ONLY
  // by the synthetic generator and final independent pose-error evaluator.
  std::vector<Pose2> reference_poses;
};

struct FrontendStats {
  std::size_t attempted{}, accepted{}, rejected{}, training_scans{};
};

std::vector<Pose2> route(const std::string& name) {
  std::vector<Eigen::Vector2d> waypoints;
  if (name == "apartment") {
    waypoints = {{2,2}, {4.2,2}, {4.2,4.75}, {6,4.75}, {8.5,4.75},
                 {8.5,7.75}, {11,7.75}, {12.5,9.8}, {11,10.3},
                 {7,10.3}, {3.5,10.3}, {3.5,8.2}, {3.3,6.4},
                 {3.3,3.0}, {2,2}};
  } else if (name == "warehouse") {
    waypoints = {{2,2}, {28,2}, {28,18}, {2,18}, {2,2}};
  } else {
    throw std::invalid_argument("unknown exact synthetic scene (apartment|warehouse)");
  }
  std::vector<Pose2> truth;
  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    const auto a = waypoints[i-1], b = waypoints[i];
    const auto difference = b-a;
    const auto length = difference.norm();
    if (length <= 0.0) throw std::invalid_argument("invalid route segment");
    const std::size_t steps = static_cast<std::size_t>(std::ceil(length / 1.0));
    const double yaw = std::atan2(difference.y(), difference.x());
    for (std::size_t j=0; j<steps; ++j) {
      const double alpha = static_cast<double>(j)/static_cast<double>(steps);
      truth.push_back({a + alpha*difference, yaw});
    }
  }
  const auto end = waypoints.back();
  truth.push_back({end, truth.back().yaw_rad});
  return truth;
}

Dataset generate(const std::string& name) {
  Dataset d;
  d.scene = name == "apartment" ? slam::fixtures::apartment() :
            name == "warehouse" ? slam::fixtures::warehouse() :
            throw std::invalid_argument("unknown synthetic benchmark scene");
  d.grid.resolution_m = 0.05;
  d.grid.map_from_grid = {{-0.5,-0.5}, 0.0};
  d.grid.width = name == "apartment" ? 321 : 621;
  d.grid.height = name == "apartment" ? 261 : 421;
  d.reference_poses = route(name);
  d.scans.reserve(d.reference_poses.size());
  for (std::size_t i=0; i<d.reference_poses.size(); ++i) {
    const Pose2 truth = d.reference_poses[i];
    const slam::Timestamp stamp{static_cast<slam::TimeNs>(i) * 200'000'000LL +
                                2'000'000'000LL, 1, 1};
    const auto scan = slam::fixtures::simulate_scan(
        d.scene, truth, {}, {}, stamp, 360, 100'000'000,
        name == "apartment" ? 22.0 : 40.0);
    Observation record;
    record.laser_hits.reserve(scan.beams.size());
    for (const auto& b : scan.beams) {
      if (!b.hit) continue;
      record.laser_hits.emplace_back(b.range_m*std::cos(b.angle_rad),
                                     b.range_m*std::sin(b.angle_rad));
    }
    if (i > 0) {
      // Simulated wheel-like relative odometry with repeatable scale, yaw,
      // and lateral biases. The matcher never receives the exact pose.
      const Pose2 exact_relative = slam::compose(
          slam::inverse(d.reference_poses[i-1]), truth);
      record.measured_previous_from_current = exact_relative;
      auto& m = record.measured_previous_from_current;
      m.translation.x() = m.translation.x()*1.018 +
                           0.009 * std::sin(1.3*static_cast<double>(i));
      m.translation.y() = m.translation.y()*0.995 +
                           0.007 * std::cos(0.7*static_cast<double>(i));
      m.yaw_rad = slam::wrap_yaw(m.yaw_rad + 0.004 +
                          0.004 * std::sin(0.31*static_cast<double>(i)));
    }
    d.scans.push_back(std::move(record));
  }
  if (d.scans.size() < 10)
    throw std::runtime_error("too few synthetic observations");
  return d;
}

// No simulator truth enters this function except the deliberately fixed
// initial absolute map-frame anchor (pose zero) shared by both baselines.
std::vector<Pose2> dead_reckoning(const std::vector<Observation>& scans,
                                 const Pose2 first_anchor) {
  std::vector<Pose2> path;
  path.reserve(scans.size());
  path.push_back(first_anchor);
  for (std::size_t i=1; i<scans.size(); ++i)
    path.push_back(slam::compose(path.back(), scans[i].measured_previous_from_current));
  return path;
}

slam::mapping::MapBundle build_map(const Dataset& d,
                                   const std::vector<Pose2>& estimates,
                                   bool only_even_training) {
  if (estimates.size() != d.scans.size())
    throw std::invalid_argument("scan/pose cardinality mismatch");
  slam::mapping::OccupancyGrid occupancy(d.grid);
  for (std::size_t i=0; i<d.scans.size(); ++i) {
    if (only_even_training && i%2 != 0) continue;
    std::vector<WorldRay> rays;
    rays.reserve(d.scans[i].laser_hits.size());
    for (const auto& p : d.scans[i].laser_hits) {
      rays.push_back({estimates[i].translation,
                      slam::transform(estimates[i], p),
                      slam::sensors::ReturnKind::hit});
    }
    (void)occupancy.integrate_rays(rays);
  }
  return {slam::mapping::MapSnapshot::capture(occupancy),
          {"map", d.scene.id, "synthetic_estimated", true}};
}

std::pair<std::vector<Pose2>, FrontendStats> refine(
    const Dataset& d, const std::vector<Pose2>& odom,
    const slam::mapping::MapBundle& training) {
  auto field = slam::mapping::DistanceField::build(training.snapshot, 0.65);
  slam::matching::MatchSettings settings;
  settings.min_supported_hits = 18;
  settings.minimum_overlap = 0.45;
  settings.maximum_iterations = 30;
  settings.maximum_step_translation_m = 0.12;
  settings.maximum_step_yaw_rad = 0.075;
  std::vector<Pose2> path;
  path.reserve(odom.size());
  path.push_back(odom.front());
  FrontendStats stats;
  for (std::size_t i=1; i<d.scans.size(); ++i) {
    // Propagate relative noisy odometry, without accessing the truth path.
    const Pose2 prediction = slam::compose(
        path.back(), d.scans[i].measured_previous_from_current);
    if (i%2 == 0) {
      ++stats.training_scans;
      path.push_back(prediction);
      continue;
    }
    ++stats.attempted;
    // Scan i was excluded from the frozen even-only training occupancy.
    const auto result = slam::matching::match_reference(
        field, d.scans[i].laser_hits, prediction, settings);
    const auto innovation = result.map_from_laser.translation - prediction.translation;
    const double yaw_innovation = std::abs(slam::wrap_yaw(
        result.map_from_laser.yaw_rad - prediction.yaw_rad));
    if (result.status == slam::matching::MatchStatus::accepted &&
        result.diagnostics.source_fingerprint == training.snapshot->fingerprint() &&
        result.diagnostics.full_rank &&
        result.diagnostics.supported_hits >= settings.min_supported_hits &&
        result.diagnostics.overlap >= settings.minimum_overlap &&
        innovation.norm() <= 0.28 && yaw_innovation <= 0.16) {
      path.push_back(result.map_from_laser);
      ++stats.accepted;
    } else {
      path.push_back(prediction);
      ++stats.rejected;
    }
  }
  return {std::move(path), stats};
}

struct PoseError {
  double translation_rmse_m{}, translation_p95_m{}, yaw_rmse_rad{};
};
PoseError pose_error(const std::vector<Pose2>& estimated,
                     const std::vector<Pose2>& reference) {
  if (estimated.size() != reference.size() || reference.empty())
    throw std::invalid_argument("invalid pose evaluation sequence");
  double sum_xy=0, sum_yaw=0;
  std::vector<double> distances;
  distances.reserve(reference.size());
  for (std::size_t i=0; i<reference.size(); ++i) {
    const double error = (estimated[i].translation-reference[i].translation).norm();
    const double dyaw = slam::wrap_yaw(estimated[i].yaw_rad-reference[i].yaw_rad);
    distances.push_back(error);
    sum_xy += error*error;
    sum_yaw += dyaw*dyaw;
  }
  const std::size_t rank = static_cast<std::size_t>(
      std::ceil(0.95*static_cast<double>(distances.size()))) - 1;
  std::nth_element(distances.begin(),
                   distances.begin()+static_cast<std::ptrdiff_t>(rank), distances.end());
  return {std::sqrt(sum_xy/reference.size()), distances[rank],
          std::sqrt(sum_yaw/reference.size())};
}

void export_map(const std::filesystem::path& path,
                const slam::mapping::MapBundle& map) {
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  if (!stream) throw std::runtime_error("cannot create map bundle output");
  slam::mapping::write_map_bundle(stream,map);
}

void write_pose(std::ostream& out, const PoseError& e) {
  out << "{\"translation_rmse_m\":" << e.translation_rmse_m
      << ",\"translation_p95_m\":" << e.translation_p95_m
      << ",\"yaw_rmse_rad\":" << e.yaw_rmse_rad << "}";
}

void write_report(std::ostream& out, const Dataset& d, const FrontendStats& stats,
                  const PoseError& odom, const PoseError& matched,
                  const slam::mapping::MapAccuracyReport& baseline_map,
                  const slam::mapping::MapAccuracyReport& rebuilt_map) {
  out << std::setprecision(12);
  out << "{\n  \"format\":\"FUSSYNTHBENCH1\","
      << "\n  \"scene\":\"" << d.scene.id << "\","
      << "\n  \"truth_class\":\"synthetic_exact\","
      << "\n  \"method\":\"offline_even_training_odd_heldout_scan_to_map\","
      << "\n  \"pose_anchor\":\"first_pose_only\","
      << "\n  \"odometry\":\"deterministically_perturbed_relative_pose\","
      << "\n  \"mapping_truth_poses_used\":false,"
      << "\n  \"reference_map_fingerprint\":" << baseline_map.map_fingerprint
      << ",\n  \"candidate_map_fingerprint\":" << rebuilt_map.map_fingerprint
      << ",\n  \"scan_count\":" << d.scans.size()
      << ",\n  \"query_attempted\":" << stats.attempted
      << ",\n  \"query_accepted\":" << stats.accepted
      << ",\n  \"query_rejected\":" << stats.rejected
      << ",\n  \"even_training_updates\":" << stats.training_scans+1
      << ",\n  \"baseline_pose_error\":";
  write_pose(out, odom);
  out << ",\n  \"candidate_pose_error\":";
  write_pose(out, matched);
  out << ",\n  \"baseline_map\":";
  slam::evaluation::write_map_accuracy_json(out, baseline_map, d.scene);
  out << ",\n  \"candidate_map\":";
  slam::evaluation::write_map_accuracy_json(out, rebuilt_map, d.scene);
  out << "}\n";
  if (!out) throw std::runtime_error("failed writing benchmark report");
}
} // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: slam_synthetic_mapping_benchmark "
                 "<apartment|warehouse> <empty-output-directory>\n";
    return 2;
  }
  try {
    const Dataset d = generate(argv[1]);
    const std::filesystem::path directory(argv[2]);
    if (std::filesystem::exists(directory))
      throw std::invalid_argument("output directory exists; refuse to overwrite benchmark");
    if (!std::filesystem::create_directories(directory))
      throw std::runtime_error("failed to create benchmark directory");

    // The first exact pose is an explicit gauge anchor; subsequent estimates
    // are obtained only from noisy relative measurements and scan geometry.
    const auto odom = dead_reckoning(d.scans, d.reference_poses.front());
    const auto baseline = build_map(d, odom, false);
    const auto training = build_map(d, odom, true);
    const auto [estimated, stats] = refine(d, odom, training);
    const auto candidate = build_map(d, estimated, false);

    const auto baseline_score = slam::evaluation::evaluate_map_accuracy(baseline,d.scene);
    const auto candidate_score = slam::evaluation::evaluate_map_accuracy(candidate,d.scene);
    const auto odom_error = pose_error(odom,d.reference_poses);
    const auto candidate_error = pose_error(estimated,d.reference_poses);

    export_map(directory/"baseline.slammap",baseline);
    export_map(directory/"estimated.slammap",candidate);
    std::ofstream report(directory/"benchmark.json");
    if (!report) throw std::runtime_error("cannot create benchmark JSON");
    write_report(report,d,stats,odom_error,candidate_error,
                 baseline_score,candidate_score);
    std::cout << "synthetic scene=" << d.scene.id << " scans=" << d.scans.size()
              << " heldout_accepted=" << stats.accepted << "/" << stats.attempted
              << " map_iou=" << baseline_score.occupied_iou.value_or(0.0)
              << " -> " << candidate_score.occupied_iou.value_or(0.0)
              << " pose_rmse_m=" << odom_error.translation_rmse_m
              << " -> " << candidate_error.translation_rmse_m
              << "\noutput=" << directory.string() << "/benchmark.json\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Synthetic mapping benchmark failed: " << e.what() << '\n';
    return 1;
  }
}
