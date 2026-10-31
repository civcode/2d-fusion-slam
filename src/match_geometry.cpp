#include "slam/matching/geometry.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace slam::matching {
namespace {
struct Evaluation {
  MatchDiagnostics diagnostics;
  Eigen::Vector3d direction{Eigen::Vector3d::Zero()};
};
void validate(const MatchSettings& settings) {
  if (!std::isfinite(settings.sigma_metres) || settings.sigma_metres <= 0 ||
      !std::isfinite(settings.huber_delta) || settings.huber_delta <= 0 ||
      !std::isfinite(settings.minimum_overlap) || settings.minimum_overlap <= 0 ||
      settings.minimum_overlap > 1 || settings.min_supported_hits < 3 ||
      !std::isfinite(settings.minimum_relative_eigenvalue) ||
      settings.minimum_relative_eigenvalue <= 0 || settings.minimum_relative_eigenvalue >= 1 ||
      settings.maximum_iterations == 0 || settings.maximum_iterations > 1000 ||
      !std::isfinite(settings.initial_damping) || settings.initial_damping <= 0 ||
      !std::isfinite(settings.maximum_step_translation_m) || settings.maximum_step_translation_m <= 0 ||
      !std::isfinite(settings.maximum_step_yaw_rad) || settings.maximum_step_yaw_rad <= 0)
    throw std::invalid_argument("invalid scan matching settings");
}
Evaluation compute(const mapping::DistanceField& field,
                   const std::vector<Eigen::Vector2d>& points, Pose2 pose,
                   const MatchSettings& settings) {
  Evaluation eval;
  auto& out = eval.diagnostics;
  out.input_hits = points.size();
  out.source_revision = field.source()->revision();
  out.source_fingerprint = field.source()->fingerprint();
  double distance_sum = 0.0;
  const double c = std::cos(pose.yaw_rad), s = std::sin(pose.yaw_rad);
  for (const auto& p : points) {
    if (!p.allFinite()) { ++out.unsupported_hits; continue; }
    const Eigen::Vector2d world{pose.translation.x() + c * p.x() - s * p.y(),
                                pose.translation.y() + s * p.x() + c * p.y()};
    const auto sampled = field.sample_map(world);
    if (!sampled) { ++out.unsupported_hits; continue; }
    const double residual = sampled->metres / settings.sigma_metres;
    const double abs_r = std::abs(residual);
    const double weight = abs_r <= settings.huber_delta ? 1.0 : settings.huber_delta / abs_r;
    out.robust_cost += abs_r <= settings.huber_delta ? 0.5 * residual * residual :
                       settings.huber_delta * (abs_r - 0.5 * settings.huber_delta);
    const Eigen::Vector3d jacobian(
      sampled->gradient_map.x() / settings.sigma_metres,
      sampled->gradient_map.y() / settings.sigma_metres,
      (sampled->gradient_map.x() * (-s * p.x() - c * p.y()) +
       sampled->gradient_map.y() * ( c * p.x() - s * p.y())) / settings.sigma_metres);
    out.lidar_information.noalias() += weight * jacobian * jacobian.transpose();
    eval.direction.noalias() += weight * residual * jacobian;
    distance_sum += sampled->metres;
    ++out.supported_hits;
  }
  out.overlap = points.empty() ? 0.0 :
                static_cast<double>(out.supported_hits) / static_cast<double>(points.size());
  out.mean_distance_metres = out.supported_hits ? distance_sum / out.supported_hits :
                             std::numeric_limits<double>::infinity();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(out.lidar_information);
  if (solver.info() == Eigen::Success) {
    out.eigenvalues = solver.eigenvalues();
    out.full_rank = out.eigenvalues[2] > 1e-9 &&
        out.eigenvalues[0] > settings.minimum_relative_eigenvalue * out.eigenvalues[2];
  }
  return eval;
}
}  // namespace

MatchDiagnostics evaluate_geometry(const mapping::DistanceField& field,
                                    const std::vector<Eigen::Vector2d>& points,
                                    Pose2 pose, const MatchSettings& settings) {
  validate(settings);
  return compute(field, points, pose, settings).diagnostics;
}

MatchResult match_reference(std::shared_ptr<const mapping::DistanceField> field,
                             const std::vector<Eigen::Vector2d>& points, Pose2 initial,
                             const MatchSettings& settings) {
  validate(settings);
  MatchResult result;
  result.map_from_laser = initial;
  if (!field || points.empty() || !initial.translation.allFinite() || !std::isfinite(initial.yaw_rad))
    return result;
  Pose2 current = initial;
  auto now = compute(*field, points, current, settings);
  result.diagnostics = now.diagnostics;
  if (now.diagnostics.supported_hits < settings.min_supported_hits ||
      now.diagnostics.overlap < settings.minimum_overlap) {
    result.status = MatchStatus::insufficient_support;
    return result;
  }
  if (!now.diagnostics.full_rank) { result.status = MatchStatus::degenerate; return result; }
  double damping = settings.initial_damping;
  bool improved = false;
  for (unsigned iter = 0; iter < settings.maximum_iterations; ++iter) {
    ++result.iterations;
    Eigen::Matrix3d h = now.diagnostics.lidar_information;
    h.diagonal().array() += damping;
    const Eigen::Vector3d step = h.ldlt().solve(-now.direction);
    if (!step.allFinite()) { result.status = MatchStatus::degenerate; return result; }
    Eigen::Vector3d bounded = step;
    const double length = bounded.head<2>().norm();
    if (length > settings.maximum_step_translation_m)
      bounded.head<2>() *= settings.maximum_step_translation_m / length;
    bounded.z() = std::clamp(bounded.z(), -settings.maximum_step_yaw_rad,
                             settings.maximum_step_yaw_rad);
    if (bounded.norm() < 1e-7) break;
    Pose2 candidate{{current.translation.x() + bounded.x(),
                     current.translation.y() + bounded.y()},
                     wrap_yaw(current.yaw_rad + bounded.z())};
    auto trial = compute(*field, points, candidate, settings);
    // Reject candidates that make observations disappear into unknown regions.
    if (trial.diagnostics.supported_hits < now.diagnostics.supported_hits ||
        trial.diagnostics.overlap < settings.minimum_overlap ||
        !trial.diagnostics.full_rank ||
        trial.diagnostics.robust_cost >= now.diagnostics.robust_cost - 1e-10) {
      damping *= 10.0;
      if (damping > 1e10) break;
      continue;
    }
    damping = std::max(1e-9, damping * 0.5);
    current = candidate;
    now = std::move(trial);
    improved = true;
  }
  result.map_from_laser = current;
  result.diagnostics = now.diagnostics;
  result.status = improved ? MatchStatus::accepted : MatchStatus::no_improvement;
  return result;
}
}  // namespace slam::matching
