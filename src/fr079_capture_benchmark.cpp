#include "slam/datasets/fr079_capture_benchmark.hpp"

#include "slam/core/se2.hpp"
#include "slam/mapping/distance_field.hpp"
#include "slam/matching/geometry.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <stdexcept>

namespace slam::datasets {
namespace {
constexpr double kPi=3.14159265358979323846;
const char* status_text(matching::MatchStatus s) {
  switch(s){
    case matching::MatchStatus::accepted:return "accepted";
    case matching::MatchStatus::insufficient_support:return "insufficient_support";
    case matching::MatchStatus::degenerate:return "degenerate";
    case matching::MatchStatus::no_improvement:return "no_improvement";
    case matching::MatchStatus::invalid_input:return "invalid_input";
  }
  return "invalid_input";
}
int bin(double range){
  return range<3?0:range<7?1:range<12?2:3;
}
}

Fr079CaptureBenchmark::Fr079CaptureBenchmark(
    const std::string& csv_path,const std::string& sha,bool ceres,bool support_aware,
    std::size_t every,std::size_t max_frames)
    : output_(csv_path),source_sha256_(sha),use_ceres_(ceres),
      support_aware_(support_aware),every_(every),max_frames_(max_frames) {
  if(!output_)throw std::runtime_error("cannot open FR079 capture benchmark CSV");
  if(sha.size()!=64||every<1||max_frames<1||max_frames>100)
    throw std::invalid_argument("invalid FR079 capture benchmark setup");
  output_<<"format,source_sha256,anchor_kind,frame,source_revision,source_fingerprint,"
         <<"matcher,truncation_m,search_bound_xy_m,search_bound_yaw_deg,"
         <<"seed_dx_m,seed_dy_m,seed_dyaw_deg,"
         <<"initial_xy_offset_m,initial_yaw_offset_deg,"
         <<"initial_supported_hits,initial_overlap,initial_unknown_hits,"
         <<"initial_truncated_hits,initial_outside_hits,"
         <<"support_fraction_0_3m,support_fraction_3_7m,"
         <<"support_fraction_7_12m,support_fraction_12_18m,"
         <<"match_status,ceres_termination,ceres_invalid_residual_evaluations,"
         <<"ceres_invalid_unknown_cells,ceres_invalid_truncated,"
         <<"ceres_invalid_outside_grid,ceres_invalid_other,"
         <<"ceres_unsuccessful_steps,ceres_successful_steps,"
         <<"final_xy_error_to_proxy_m,final_yaw_error_to_proxy_deg,"
         <<"recovered_proxy,accepted_but_far_from_proxy,trial_ms\n";
}
void Fr079CaptureBenchmark::probe(
    std::size_t frame,const Pose2& anchor,
    std::shared_ptr<const mapping::MapSnapshot> snapshot,
    const std::vector<Eigen::Vector2d>& points) {
  if(frame%every_!=0||sampled_frames_>=max_frames_||points.size()<12||!snapshot)
    return;
  ++sampled_frames_;
  const std::array<double,6> truncations{.35,.5,.7,1.,1.5,2.};
  const std::array<double,6> displacements{0.,.1,.2,.4,.7,1.};
  const std::array<double,6> degrees{0.,1.,2.,3.,5.,8.};
  const std::array<double,2> xy_bounds{.12,.50};
  const std::array<double,2> yaw_bounds{.08,.25};
  output_<<std::setprecision(12);
  for(double truncation:truncations) {
    const auto field=mapping::DistanceField::build(snapshot,truncation);
    for(std::size_t bounds=0;bounds<xy_bounds.size();++bounds) {
      matching::MatchSettings settings;
      settings.min_supported_hits=12;
      settings.minimum_overlap=.50;
      settings.maximum_iterations=30;
      settings.maximum_step_translation_m=xy_bounds[bounds];
      settings.maximum_step_yaw_rad=yaw_bounds[bounds];
      for(double delta:displacements)for(double angle:degrees)for(int direction=0;direction<2;++direction){
        const double dx=direction==0?delta:0.;
        const double dy=direction==0?0.:-delta;
        const double da=direction==0?angle:-angle;
        Pose2 seed{{anchor.translation.x()+dx,anchor.translation.y()+dy},
                   slam::wrap_yaw(anchor.yaw_rad+da*kPi/180.)};
        std::array<std::size_t,4> total{},supported{};
        std::size_t unknown=0,truncated=0,outside=0;
        for(const auto& point:points){
          const auto k=static_cast<std::size_t>(bin(point.norm()));
          ++total[k];
          switch(field->support_map(slam::transform(seed,point))) {
            case mapping::DistanceSupport::supported:++supported[k];break;
            case mapping::DistanceSupport::unknown_cells:++unknown;break;
            case mapping::DistanceSupport::truncated:++truncated;break;
            default:++outside;break;
          }
        }
        const auto supported_total=supported[0]+supported[1]+supported[2]+supported[3];
        const double overlap=static_cast<double>(supported_total)/points.size();
        const auto start=std::chrono::steady_clock::now();
        matching::MatchResult fit;
#ifdef SLAM_HAS_CERES
        if(use_ceres_)
          fit=support_aware_?
            matching::match_ceres_support_aware(field,points,seed,settings):
            matching::match_ceres(field,points,seed,settings);
        else
#endif
          fit=matching::match_reference(field,points,seed,settings);
        const double ms=std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-start).count();
        const double xy_error=(fit.map_from_laser.translation-anchor.translation).norm();
        const double yaw_error=std::abs(slam::wrap_yaw(
            fit.map_from_laser.yaw_rad-anchor.yaw_rad))*180./kPi;
        const bool recovered=xy_error<=.10&&yaw_error<=1.0;
        const bool bad_accept=fit.status==matching::MatchStatus::accepted&&!recovered;
        output_<<"FUSFR079CAPTURE1,"<<source_sha256_
               <<",same_run_accepted_pose_proxy_not_ground_truth,"
               <<frame<<","<<snapshot->revision()<<","<<snapshot->fingerprint()
               <<","<<(support_aware_?"ceres_support_aware":(use_ceres_?"ceres":"reference"))
               <<","<<truncation<<","<<xy_bounds[bounds]
               <<","<<yaw_bounds[bounds]*180./kPi
               <<","<<dx<<","<<dy<<","<<da
               <<","<<std::hypot(dx,dy)<<","<<std::abs(da)
               <<","<<supported_total<<","<<overlap
               <<","<<unknown<<","<<truncated<<","<<outside;
        for(std::size_t k=0;k<4;++k)
          output_<<","<<(total[k]?static_cast<double>(supported[k])/total[k]:0.);
        output_<<","<<status_text(fit.status)
               <<","<<fit.ceres_termination
               <<","<<fit.ceres_invalid_residual_evaluations
               <<","<<fit.ceres_invalid_unknown_cells
               <<","<<fit.ceres_invalid_truncated
               <<","<<fit.ceres_invalid_outside_grid
               <<","<<fit.ceres_invalid_other
               <<","<<fit.ceres_unsuccessful_steps
               <<","<<fit.ceres_successful_steps
               <<","<<xy_error<<","<<yaw_error
               <<","<<(recovered?1:0)
               <<","<<(bad_accept?1:0)
               <<","<<ms<<"\n";
        ++tested_trials_;
      }
    }
  }
  if(!output_)throw std::runtime_error("FR079 capture benchmark CSV write failed");
}
} // namespace slam::datasets
