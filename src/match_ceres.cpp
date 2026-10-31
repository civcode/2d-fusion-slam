#include "slam/matching/geometry.hpp"

#include <ceres/ceres.h>

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace slam::matching {
namespace {
// The input hit set is selected for valid EDT support at the seed pose.
// If a trial step leaves known, informative EDT support, Ceres' evaluation
// fails (not a fabricated zero-cost LiDAR sample) and the match is rejected.
class EdfHitCost final : public ceres::SizedCostFunction<1,3> {
 public:
  EdfHitCost(const mapping::DistanceField& field, Eigen::Vector2d point, double sigma)
      : field_(field), point_(std::move(point)), sigma_(sigma) {}
  bool Evaluate(double const* const* params, double* residuals, double** jacobians) const override {
    const double* a = params[0];
    const double cosine = std::cos(a[2]);
    const double sine = std::sin(a[2]);
    const Eigen::Vector2d world{a[0] + cosine*point_.x() - sine*point_.y(),
                                a[1] + sine*point_.x() + cosine*point_.y()};
    const auto value = field_.sample_map(world);
    if (!value) return false;
    residuals[0] = value->metres / sigma_;
    if (jacobians && jacobians[0]) {
      double* jac = jacobians[0];
      jac[0] = value->gradient_map.x() / sigma_;
      jac[1] = value->gradient_map.y() / sigma_;
      jac[2] = (value->gradient_map.x()*(-sine*point_.x() - cosine*point_.y()) +
                value->gradient_map.y()*( cosine*point_.x() - sine*point_.y())) / sigma_;
    }
    return true;
  }
 private:
  const mapping::DistanceField& field_;
  Eigen::Vector2d point_;
  double sigma_;
};
} // namespace

MatchResult match_ceres(std::shared_ptr<const mapping::DistanceField> field,
                        const std::vector<Eigen::Vector2d>& points,
                        Pose2 seed, const MatchSettings& settings) {
  MatchResult result;
  result.map_from_laser = seed;
  if (!field || points.empty() || !seed.translation.allFinite() || !std::isfinite(seed.yaw_rad))
    return result;
  const auto initial = evaluate_geometry(*field, points, seed, settings);
  result.diagnostics = initial;
  if (initial.supported_hits < settings.min_supported_hits ||
      initial.overlap < settings.minimum_overlap) {
    result.status = MatchStatus::insufficient_support;
    return result;
  }
  if (!initial.full_rank) { result.status = MatchStatus::degenerate; return result; }
  double pose[3]{seed.translation.x(), seed.translation.y(), seed.yaw_rad};
  ceres::Problem problem;
  const double c = std::cos(seed.yaw_rad), s = std::sin(seed.yaw_rad);
  for (const auto& p : points) {
    if (!p.allFinite()) continue;
    const Eigen::Vector2d at_seed{pose[0]+c*p.x()-s*p.y(),pose[1]+s*p.x()+c*p.y()};
    if (!field->sample_map(at_seed)) continue; // omitted, not zero weighted
    problem.AddResidualBlock(new EdfHitCost(*field,p,settings.sigma_metres),
                             new ceres::HuberLoss(settings.huber_delta),pose);
  }
  for(int i=0;i<2;++i) {
    problem.SetParameterLowerBound(pose,i,pose[i]-settings.maximum_step_translation_m);
    problem.SetParameterUpperBound(pose,i,pose[i]+settings.maximum_step_translation_m);
  }
  problem.SetParameterLowerBound(pose,2,pose[2]-settings.maximum_step_yaw_rad);
  problem.SetParameterUpperBound(pose,2,pose[2]+settings.maximum_step_yaw_rad);
  ceres::Solver::Options options;
  options.max_num_iterations = static_cast<int>(settings.maximum_iterations);
  options.linear_solver_type = ceres::DENSE_QR;
  options.num_threads = 1;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options,&problem,&summary);
  result.iterations = static_cast<unsigned>(summary.iterations.size());
  if (summary.termination_type == ceres::FAILURE ||
      summary.termination_type == ceres::USER_FAILURE) {
    result.status = MatchStatus::no_improvement;
    return result;
  }
  Pose2 candidate{{pose[0],pose[1]},wrap_yaw(pose[2])};
  const auto diagnostics = evaluate_geometry(*field,points,candidate,settings);
  if (diagnostics.supported_hits < initial.supported_hits ||
      diagnostics.overlap < settings.minimum_overlap) {
    result.status = MatchStatus::insufficient_support;
    return result;
  }
  if (!diagnostics.full_rank) {result.status=MatchStatus::degenerate;return result;}
  if (diagnostics.robust_cost >= initial.robust_cost - 1e-10) {
    result.status = MatchStatus::no_improvement;
    return result;
  }
  result.map_from_laser = candidate;
  result.diagnostics = diagnostics;
  result.status = MatchStatus::accepted;
  return result;
}
} // namespace slam::matching
