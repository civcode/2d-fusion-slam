#include "slam/matching/geometry.hpp"

#include <ceres/ceres.h>

#include <algorithm>
#include <array>
#include <atomic>
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
  EdfHitCost(const mapping::DistanceField& field, Eigen::Vector2d point, double sigma,
             std::atomic<std::size_t>& invalid_evaluations,
             std::array<std::atomic<std::size_t>,4>& reasons)
      : field_(field), point_(std::move(point)), sigma_(sigma),
        invalid_evaluations_(invalid_evaluations), reasons_(reasons) {}
  bool Evaluate(double const* const* params, double* residuals, double** jacobians) const override {
    const double* a = params[0];
    const double cosine = std::cos(a[2]);
    const double sine = std::sin(a[2]);
    const Eigen::Vector2d world{a[0] + cosine*point_.x() - sine*point_.y(),
                                a[1] + sine*point_.x() + cosine*point_.y()};
    const auto value = field_.sample_map(world);
    if (!value) {
      invalid_evaluations_.fetch_add(1,std::memory_order_relaxed);
      const auto reason=field_.support_map(world);
      const std::size_t index=
          reason==mapping::DistanceSupport::unknown_cells?0:
          reason==mapping::DistanceSupport::truncated?1:
          reason==mapping::DistanceSupport::outside_grid?2:3;
      reasons_[index].fetch_add(1,std::memory_order_relaxed);
      return false;
    }
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
  std::atomic<std::size_t>& invalid_evaluations_;
  std::array<std::atomic<std::size_t>,4>& reasons_;
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
  std::atomic<std::size_t> invalid_evaluations{0};
  std::array<std::atomic<std::size_t>,4> unsupported_reasons{};
  ceres::Problem problem;
  const double c = std::cos(seed.yaw_rad), s = std::sin(seed.yaw_rad);
  for (const auto& p : points) {
    if (!p.allFinite()) continue;
    const Eigen::Vector2d at_seed{pose[0]+c*p.x()-s*p.y(),pose[1]+s*p.x()+c*p.y()};
    if (!field->sample_map(at_seed)) continue; // omitted, not zero weighted
    problem.AddResidualBlock(new EdfHitCost(*field,p,settings.sigma_metres,invalid_evaluations,unsupported_reasons),
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
  result.ceres_invalid_residual_evaluations=invalid_evaluations.load(std::memory_order_relaxed);
  result.ceres_invalid_unknown_cells=unsupported_reasons[0].load(std::memory_order_relaxed);
  result.ceres_invalid_truncated=unsupported_reasons[1].load(std::memory_order_relaxed);
  result.ceres_invalid_outside_grid=unsupported_reasons[2].load(std::memory_order_relaxed);
  result.ceres_invalid_other=unsupported_reasons[3].load(std::memory_order_relaxed);
  result.ceres_unsuccessful_steps=static_cast<std::size_t>(summary.num_unsuccessful_steps);
  result.ceres_successful_steps=static_cast<std::size_t>(summary.num_successful_steps);
  switch(summary.termination_type) {
    case ceres::CONVERGENCE:result.ceres_termination="convergence";break;
    case ceres::NO_CONVERGENCE:result.ceres_termination="no_convergence";break;
    case ceres::FAILURE:result.ceres_termination="failure";break;
    case ceres::USER_SUCCESS:result.ceres_termination="user_success";break;
    case ceres::USER_FAILURE:result.ceres_termination="user_failure";break;
    default:result.ceres_termination="unknown";break;
  }
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

// Opt-in support-aware local refinement. The old match_ceres remains the
// comparison baseline. For each stage, freeze a subset of observed LiDAR
// endpoints whose EDT bilinear stencils are guaranteed valid throughout
// *every* pose in the stage's Ceres parameter box. No residual disappears,
// and unknown cells never supply an artificial zero-distance measurement.
// Re-select after each validated stage to avoid permanently discarding beams.
MatchResult match_ceres_support_aware(
    std::shared_ptr<const mapping::DistanceField> field,
    const std::vector<Eigen::Vector2d>& points,
    Pose2 seed, const MatchSettings& settings) {
  MatchResult result;
  result.map_from_laser=seed;
  if(!field||points.empty()||!seed.translation.allFinite()||
     !std::isfinite(seed.yaw_rad))return result;
  const auto initial=evaluate_geometry(*field,points,seed,settings);
  result.diagnostics=initial;
  if(initial.supported_hits<settings.min_supported_hits||
     initial.overlap<settings.minimum_overlap) {
    result.status=MatchStatus::insufficient_support;
    return result;
  }
  if(!initial.full_rank) {
    result.status=MatchStatus::degenerate;
    return result;
  }

  Pose2 current=seed;
  auto current_all=initial;
  bool improved=false;
  constexpr unsigned kMaxStages=12;
  for(unsigned stage=0;stage<kMaxStages;++stage) {
    const double remaining_x=settings.maximum_step_translation_m-
        std::abs(current.translation.x()-seed.translation.x());
    const double remaining_y=settings.maximum_step_translation_m-
        std::abs(current.translation.y()-seed.translation.y());
    const double remaining_yaw=settings.maximum_step_yaw_rad-
        std::abs(wrap_yaw(current.yaw_rad-seed.yaw_rad));
    if(remaining_x<0.002||remaining_y<0.002||remaining_yaw<0.0004)
      break;

    // Prefer the largest useful stage; adaptively shrink if it lacks a
    // sufficiently informative set of *guaranteed supported* endpoints.
    std::vector<Eigen::Vector2d> selected;
    double xy=0.,yaw=0.;
    constexpr double kNominalXY=0.03;
    constexpr double kNominalYaw=0.006;
    const std::size_t minimum_safe=std::max(
        settings.min_supported_hits,
        static_cast<std::size_t>(std::ceil(points.size()*0.20)));
    for(double fraction: {1.0,0.5,0.25,0.125}) {
      const double bx=std::min(kNominalXY*fraction,
                               std::min(remaining_x,remaining_y));
      const double byaw=std::min(kNominalYaw*fraction,remaining_yaw);
      if(bx<0.001||byaw<0.0001)continue;
      std::vector<Eigen::Vector2d> candidate;
      candidate.reserve(points.size());
      const double c=std::cos(current.yaw_rad);
      const double sn=std::sin(current.yaw_rad);
      for(const auto& p:points) {
        if(!p.allFinite())continue;
        const Eigen::Vector2d endpoint{
          current.translation.x()+c*p.x()-sn*p.y(),
          current.translation.y()+sn*p.x()+c*p.y()};
        // Every pose in the bounded Ceres box moves this endpoint at
        // most sqrt(2)*bx + 2*range*sin(byaw/2). Conservative, metric,
        // and valid for any beam direction.
        const double max_motion=std::sqrt(2.0)*bx+
            2.0*p.norm()*std::sin(byaw*0.5)+1e-9;
        if(field->supports_disk_map(endpoint,max_motion))
          candidate.push_back(p);
      }
      if(candidate.size()<minimum_safe)continue;
      const auto geometry=evaluate_geometry(*field,candidate,current,settings);
      if(!geometry.full_rank||geometry.supported_hits!=candidate.size())
        continue;
      selected=std::move(candidate);
      xy=bx;yaw=byaw;
      break;
    }
    if(selected.empty()) {
      result.support_aware_no_safe_stage=true;
      break;
    }

    result.support_aware_selected_hits+=selected.size();
    result.support_aware_skipped_hits+=points.size()-selected.size();
    ++result.support_aware_stages;
    double pose[3]{current.translation.x(),current.translation.y(),
                   current.yaw_rad};
    std::atomic<std::size_t> invalid{0};
    std::array<std::atomic<std::size_t>,4> reasons{};
    ceres::Problem problem;
    for(const auto& point:selected)
      problem.AddResidualBlock(
          new EdfHitCost(*field,point,settings.sigma_metres,invalid,reasons),
          new ceres::HuberLoss(settings.huber_delta),pose);
    for(int axis=0;axis<2;++axis) {
      problem.SetParameterLowerBound(pose,axis,pose[axis]-xy);
      problem.SetParameterUpperBound(pose,axis,pose[axis]+xy);
    }
    problem.SetParameterLowerBound(pose,2,pose[2]-yaw);
    problem.SetParameterUpperBound(pose,2,pose[2]+yaw);
    ceres::Solver::Options options;
    options.max_num_iterations=std::min(10u,settings.maximum_iterations);
    options.linear_solver_type=ceres::DENSE_QR;
    options.num_threads=1;
    options.minimizer_progress_to_stdout=false;
    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);

    result.iterations+=static_cast<unsigned>(summary.iterations.size());
    result.ceres_invalid_residual_evaluations+=invalid.load(std::memory_order_relaxed);
    result.ceres_invalid_unknown_cells+=reasons[0].load(std::memory_order_relaxed);
    result.ceres_invalid_truncated+=reasons[1].load(std::memory_order_relaxed);
    result.ceres_invalid_outside_grid+=reasons[2].load(std::memory_order_relaxed);
    result.ceres_invalid_other+=reasons[3].load(std::memory_order_relaxed);
    result.ceres_unsuccessful_steps+=static_cast<std::size_t>(summary.num_unsuccessful_steps);
    result.ceres_successful_steps+=static_cast<std::size_t>(summary.num_successful_steps);
    switch(summary.termination_type) {
      case ceres::CONVERGENCE:result.ceres_termination="convergence";break;
      case ceres::NO_CONVERGENCE:result.ceres_termination="no_convergence";break;
      case ceres::FAILURE:result.ceres_termination="failure";break;
      case ceres::USER_SUCCESS:result.ceres_termination="user_success";break;
      case ceres::USER_FAILURE:result.ceres_termination="user_failure";break;
      default:result.ceres_termination="unknown";break;
    }
    if(summary.termination_type==ceres::FAILURE||
       summary.termination_type==ceres::USER_FAILURE)break;
    const Pose2 trial{{pose[0],pose[1]},wrap_yaw(pose[2])};
    const auto candidate_all=evaluate_geometry(*field,points,trial,settings);
    // Never allow a formerly supported observation to disappear merely
    // because a different beam became supported. Supported-hit counts alone
    // can conceal this sort of evidence swapping.
    bool lost_previous_support=false;
    for(const auto& p:points) {
      if(!p.allFinite())continue;
      if(field->sample_map(slam::transform(current,p)) &&
         !field->sample_map(slam::transform(trial,p))) {
        lost_previous_support=true;
        break;
      }
    }
    // The subset Ceres objective is not enough to accept a pose. Re-check
    // ALL original beams against exactly the pinned prior map. A loss of
    // support, rank, overlap or global robust-cost improvement is rejected.
    if(lost_previous_support||
       candidate_all.supported_hits<current_all.supported_hits||
       candidate_all.overlap<settings.minimum_overlap||
       !candidate_all.full_rank||
       candidate_all.robust_cost>=current_all.robust_cost-1e-10)
      break;
    const double transl=(trial.translation-current.translation).norm();
    const double rot=std::abs(wrap_yaw(trial.yaw_rad-current.yaw_rad));
    current=trial;
    current_all=candidate_all;
    improved=true;
    ++result.support_aware_accepted_stages;
    if(transl<0.0005&&rot<0.0001)break;
  }
  if(improved){
    result.map_from_laser=current;
    result.diagnostics=current_all;
    result.status=MatchStatus::accepted;
  }else result.status=result.support_aware_no_safe_stage?
      MatchStatus::insufficient_support:MatchStatus::no_improvement;
  return result;
}


// This cost is defined at every finite endpoint. Unsupported/unknown map
// geometry is a maximum penalty with smooth bilinear transitions, not a
// newly observed wall. Each Ceres evaluation reports when the strict
// interpolation would have been unavailable for A/B diagnosis.
namespace {
class ContinuousHitCost final : public ceres::SizedCostFunction<1,3> {
 public:
  ContinuousHitCost(const mapping::DistanceField& field,Eigen::Vector2d point,
                    double sigma,std::array<std::atomic<std::size_t>,4>& counts)
      : field_(field),point_(std::move(point)),sigma_(sigma),counts_(counts) {}
  bool Evaluate(double const* const* params,double* residuals,
                double** jacobians) const override {
    const double* p=params[0];
    const double c=std::cos(p[2]),s=std::sin(p[2]);
    const Eigen::Vector2d world{
      p[0]+c*point_.x()-s*point_.y(),
      p[1]+s*point_.x()+c*point_.y()};
    // Ceres parameter boxes are finite; fail closed for unexpected NaNs.
    if(!world.allFinite())return false;
    const auto value=field_.sample_matching_cost(world);
    switch(field_.support_map(world)){
      case mapping::DistanceSupport::supported:break;
      case mapping::DistanceSupport::unknown_cells:
        counts_[0].fetch_add(1,std::memory_order_relaxed);break;
      case mapping::DistanceSupport::truncated:
        counts_[1].fetch_add(1,std::memory_order_relaxed);break;
      case mapping::DistanceSupport::outside_grid:
        counts_[2].fetch_add(1,std::memory_order_relaxed);break;
      default:
        counts_[3].fetch_add(1,std::memory_order_relaxed);break;
    }
    residuals[0]=value.metres/sigma_;
    if(jacobians&&jacobians[0]){
      double* jac=jacobians[0];
      jac[0]=value.gradient_map.x()/sigma_;
      jac[1]=value.gradient_map.y()/sigma_;
      jac[2]=(value.gradient_map.x()*(-s*point_.x()-c*point_.y())+
              value.gradient_map.y()*( c*point_.x()-s*point_.y()))/sigma_;
    }
    return true;
  }
 private:
  const mapping::DistanceField& field_;
  Eigen::Vector2d point_;
  double sigma_;
  std::array<std::atomic<std::size_t>,4>& counts_;
};

// Match the same input beam cohort and the same Huber loss at both poses.
// Strictly-supported geometry is still evaluated independently for rank,
// overlap, and observability; a reduction by dropping beams is impossible.
double fixed_continuous_cost(const mapping::DistanceField& field,
                             const std::vector<Eigen::Vector2d>& points,
                             Pose2 pose,const MatchSettings& settings) {
  double cost=0.;
  for(const auto& point:points){
    if(!point.allFinite())continue;
    const double a=field.sample_matching_cost(transform(pose,point)).metres/
                   settings.sigma_metres;
    const double magnitude=std::abs(a);
    const double delta=settings.huber_delta;
    cost+=magnitude<=delta?0.5*a*a:delta*(magnitude-0.5*delta);
  }
  return cost;
}
} // namespace

MatchResult match_ceres_continuous(
    std::shared_ptr<const mapping::DistanceField> field,
    const std::vector<Eigen::Vector2d>& points,
    Pose2 seed,const MatchSettings& settings) {
  MatchResult result;
  result.map_from_laser=seed;
  if(!field||points.empty()||!seed.translation.allFinite()||
     !std::isfinite(seed.yaw_rad))return result;
  // Do not bootstrap SLAM from synthetic unknown-space costs. Require
  // genuine observed geometry before optimizing.
  const auto initial=evaluate_geometry(*field,points,seed,settings);
  result.diagnostics=initial;
  if(initial.supported_hits<settings.min_supported_hits||
     initial.overlap<settings.minimum_overlap) {
    result.status=MatchStatus::insufficient_support;
    return result;
  }
  if(!initial.full_rank) {
    result.status=MatchStatus::degenerate;
    return result;
  }

  double pose[3]{seed.translation.x(),seed.translation.y(),seed.yaw_rad};
  std::array<std::atomic<std::size_t>,4> finite_unsupported{};
  ceres::Problem problem;
  std::size_t inserted=0;
  for(const auto& point:points){
    if(!point.allFinite())continue;
    problem.AddResidualBlock(
        new ContinuousHitCost(*field,point,settings.sigma_metres,
                              finite_unsupported),
        new ceres::HuberLoss(settings.huber_delta),pose);
    ++inserted;
  }
  if(inserted<settings.min_supported_hits) {
    result.status=MatchStatus::insufficient_support;
    return result;
  }
  for(int i=0;i<2;++i) {
    problem.SetParameterLowerBound(pose,i,pose[i]-settings.maximum_step_translation_m);
    problem.SetParameterUpperBound(pose,i,pose[i]+settings.maximum_step_translation_m);
  }
  problem.SetParameterLowerBound(pose,2,pose[2]-settings.maximum_step_yaw_rad);
  problem.SetParameterUpperBound(pose,2,pose[2]+settings.maximum_step_yaw_rad);
  ceres::Solver::Options options;
  options.max_num_iterations=static_cast<int>(settings.maximum_iterations);
  options.linear_solver_type=ceres::DENSE_QR;
  options.num_threads=1;
  options.minimizer_progress_to_stdout=false;
  ceres::Solver::Summary summary;
  ceres::Solve(options,&problem,&summary);
  result.iterations=static_cast<unsigned>(summary.iterations.size());
  result.ceres_continuous_unknown_evaluations=finite_unsupported[0].load(std::memory_order_relaxed);
  result.ceres_continuous_truncated_evaluations=finite_unsupported[1].load(std::memory_order_relaxed);
  result.ceres_continuous_outside_evaluations=finite_unsupported[2].load(std::memory_order_relaxed);
  result.ceres_continuous_other_evaluations=finite_unsupported[3].load(std::memory_order_relaxed);
  result.ceres_unsuccessful_steps=static_cast<std::size_t>(summary.num_unsuccessful_steps);
  result.ceres_successful_steps=static_cast<std::size_t>(summary.num_successful_steps);
  switch(summary.termination_type) {
    case ceres::CONVERGENCE:result.ceres_termination="convergence";break;
    case ceres::NO_CONVERGENCE:result.ceres_termination="no_convergence";break;
    case ceres::FAILURE:result.ceres_termination="failure";break;
    case ceres::USER_SUCCESS:result.ceres_termination="user_success";break;
    case ceres::USER_FAILURE:result.ceres_termination="user_failure";break;
    default:result.ceres_termination="unknown";break;
  }
  if(summary.termination_type==ceres::FAILURE||
     summary.termination_type==ceres::USER_FAILURE) {
    result.status=MatchStatus::no_improvement;
    return result;
  }
  const Pose2 candidate{{pose[0],pose[1]},wrap_yaw(pose[2])};
  // Independently reject poses that lack real geometric support, even
  // though the solver cost is evaluable throughout unknown space.
  const auto diagnostics=evaluate_geometry(*field,points,candidate,settings);
  if(diagnostics.supported_hits<initial.supported_hits||
     diagnostics.overlap<settings.minimum_overlap) {
    result.status=MatchStatus::insufficient_support;
    return result;
  }
  if(!diagnostics.full_rank) {
    result.status=MatchStatus::degenerate;
    return result;
  }
  // Fixed cohort avoids an apparent improvement from dropping known hits,
  // and avoids an apparent regression when new real hits enter support.
  if(fixed_continuous_cost(*field,points,candidate,settings)>=
     fixed_continuous_cost(*field,points,seed,settings)-1e-10) {
    result.status=MatchStatus::no_improvement;
    return result;
  }
  result.map_from_laser=candidate;
  result.diagnostics=diagnostics;
  result.status=MatchStatus::accepted;
  return result;
}

} // namespace slam::matching
