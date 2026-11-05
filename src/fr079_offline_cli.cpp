#include <cxxopts.hpp>
#include "slam/cli/review_artifacts.hpp"
#include "slam/datasets/fr079_offline.hpp"
#include "slam/datasets/fr079_loop_closure.hpp"
#include "slam/datasets/fr079_capture_benchmark.hpp"
#include "slam/datasets/carmen.hpp"
#include "slam/mapping/map_bundle.hpp"

#include <openssl/sha.h>
#include <zlib.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Clock=std::chrono::steady_clock;
using slam::datasets::CarmenLaser;
using slam::datasets::Fr079SlamFrame;
using slam::datasets::Fr079SlamResult;

double elapsed_ms(Clock::time_point a,Clock::time_point b) {
  return std::chrono::duration<double,std::milli>(b-a).count();
}
void ensure(bool ok,const char* msg){
  if(!ok)throw std::invalid_argument(msg);
}
std::string read_log(const std::string& file){
  gzFile in=gzopen(file.c_str(),"rb"); // zlib also accepts uncompressed logs
  if(!in)throw std::runtime_error("cannot open CARMEN input");
  std::string body;
  std::array<char,8192> chunk{};
  int count=0;
  while((count=gzread(in,chunk.data(),static_cast<unsigned>(chunk.size())))>0){
    if(body.size()+static_cast<std::size_t>(count)>128*1024*1024){
      (void)gzclose(in);
      throw std::invalid_argument("CARMEN input exceeds 128 MiB limit");
    }
    body.append(chunk.data(),static_cast<std::size_t>(count));
  }
  const int rc=gzclose(in);
  if(count<0||rc!=Z_OK)throw std::runtime_error("truncated or invalid CARMEN input");
  return body;
}
std::string sha256(const std::string& value) {
  unsigned char digest[SHA256_DIGEST_LENGTH]{};
  if(!SHA256(reinterpret_cast<const unsigned char*>(value.data()),
             value.size(),digest))
    throw std::runtime_error("SHA256 input failed");
  constexpr char hex[]="0123456789abcdef";
  std::string result;
  result.reserve(64);
  for(unsigned char c:digest){
    result.push_back(hex[c>>4]);result.push_back(hex[c&15]);
  }
  return result;
}
std::vector<CarmenLaser> parse(const std::string& content){
  std::istringstream input(content);
  slam::datasets::CarmenReader reader;
  std::vector<CarmenLaser> scans;
  std::string line;
  std::size_t number=0;
  while(std::getline(input,line)){
    ++number;
    if(line.size()>1024*1024)throw std::invalid_argument("oversized CARMEN line");
    try{
      if(auto scan=reader.parse_laser(line)){
        ensure(scans.size()<6000,"CARMEN log exceeds 6000 scans");
        scans.push_back(std::move(*scan));
      }else (void)reader.parse_odom(line); // validate ODOM records; scan odom is time-aligned
    }catch(const std::exception& e){
      throw std::invalid_argument("CARMEN line "+std::to_string(number)+": "+e.what());
    }
  }
  ensure(!scans.empty(),"CARMEN file contains no laser scans");
  return scans;
}
std::string pose(const slam::Pose2& p){
  std::ostringstream out;
  out<<std::setprecision(15)<<"["<<p.translation.x()<<","
     <<p.translation.y()<<","<<p.yaw_rad<<"]";
  return out.str();
}
std::string duration_summary(const std::vector<double>& d) {
  if(d.empty())return "{}";
  auto sorted=d;
  std::sort(sorted.begin(),sorted.end());
  const auto rank=[&](double p){
    return sorted[static_cast<std::size_t>(std::ceil(p*sorted.size()))-1];
  };
  const double total=std::accumulate(sorted.begin(),sorted.end(),0.0);
  std::ostringstream out;
  out<<std::setprecision(12)<<"{\"count\":"<<sorted.size()
     <<",\"mean_ms\":"<<total/sorted.size()
     <<",\"p50_ms\":"<<rank(.5)
     <<",\"p95_ms\":"<<rank(.95)
     <<",\"p99_ms\":"<<rank(.99)
     <<",\"max_ms\":"<<sorted.back()
     <<",\"total_ms\":"<<total<<"}";
  return out.str();
}
void save_trajectory(const std::filesystem::path& file,
                     const Fr079SlamResult& result,const std::string& source){
  std::ofstream out(file);
  if(!out)throw std::runtime_error("failed to write FR079 trajectory");
  out<<std::setprecision(15)<<"{\"format\":\"FUSFR079POSE1\","
     <<"\"source_sha256\":\""<<source<<"\","
     <<"\"pose_kind\":\"estimated_from_odometry_and_prior_lidar_scans_not_truth\","
     <<"\"frame_id\":\"fr079_odom_scan_frame_UNVERIFIED\","
     <<"\"poses\":[";
  for(std::size_t i=0;i<result.frames.size();++i){
    const auto& f=result.frames[i];
    if(i)out<<",";
    out<<"{\"index\":"<<i<<",\"t\":"<<f.scan_time_s
       <<",\"odom\":"<<pose(f.odom)
       <<",\"prior\":"<<pose(f.predicted)
       <<",\"estimated\":"<<pose(f.estimated)<<"}";
  }
  out<<"]}\n";
  if(!out)throw std::runtime_error("trajectory write failed");
}
void save_matches(const std::filesystem::path& file,
                  const Fr079SlamResult& result,const std::string& source){
  std::ofstream out(file);
  if(!out)throw std::runtime_error("failed to write FR079 match decisions");
  out<<std::setprecision(15)<<"{\"format\":\"FUSFR079MATCH1\","
     <<"\"source_sha256\":\""<<source<<"\","
     <<"\"training_independence\":\"causal_prior_scans_only\","
     <<"\"independent_ground_truth\":false,\"frames\":[";
  for(std::size_t i=0;i<result.frames.size();++i){
    const auto& f=result.frames[i];
    if(i)out<<",";
    out<<"{\"index\":"<<i<<",\"status\":\""<<f.match_status
       <<"\",\"attempted\":"<<(f.matched?"true":"false")
       <<",\"accepted\":"<<(f.accepted?"true":"false")
       <<",\"match_input_beams\":"<<f.match_input_beams
       <<",\"solver_residual_blocks\":"<<f.solver_residual_blocks
       <<",\"candidate_evaluated\":"<<(f.candidate_evaluated?"true":"false")
       <<",\"seed_supported_hits\":"<<f.seed_supported_hits
       <<",\"candidate_supported_hits\":"<<f.candidate_supported_hits
       <<",\"seed_strict_cost\":"<<f.seed_strict_cost
       <<",\"candidate_strict_cost\":"<<f.candidate_strict_cost
       <<",\"seed_finite_cost\":"<<f.seed_finite_cost
       <<",\"candidate_finite_cost\":"<<f.candidate_finite_cost
       <<",\"candidate_translation_m\":"<<f.candidate_translation_m
       <<",\"candidate_rotation_rad\":"<<f.candidate_rotation_rad
       <<",\"supported_hits\":"<<f.supported_hits
       <<",\"overlap\":"<<f.overlap
       <<",\"robust_cost\":"<<f.robust_cost
       <<",\"ceres_invalid_residual_evaluations\":"<<f.ceres_invalid_residual_evaluations
       <<",\"ceres_continuous_unknown_evaluations\":"<<f.ceres_continuous_unknown_evaluations
       <<",\"ceres_continuous_truncated_evaluations\":"<<f.ceres_continuous_truncated_evaluations
       <<",\"ceres_continuous_outside_evaluations\":"<<f.ceres_continuous_outside_evaluations
       <<",\"ceres_continuous_other_evaluations\":"<<f.ceres_continuous_other_evaluations
       <<",\"ceres_invalid_unknown_cells\":"<<f.ceres_invalid_unknown_cells
       <<",\"ceres_invalid_truncated\":"<<f.ceres_invalid_truncated
       <<",\"ceres_invalid_outside_grid\":"<<f.ceres_invalid_outside_grid
       <<",\"ceres_invalid_other\":"<<f.ceres_invalid_other
       <<",\"ceres_unsuccessful_steps\":"<<f.ceres_unsuccessful_steps
       <<",\"ceres_successful_steps\":"<<f.ceres_successful_steps
       <<",\"ceres_termination\":\""<<f.ceres_termination<<"\""
       <<",\"support_aware_selected_hits\":"<<f.support_aware_selected_hits
       <<",\"support_aware_skipped_hits\":"<<f.support_aware_skipped_hits
       <<",\"support_aware_stages\":"<<f.support_aware_stages
       <<",\"support_aware_accepted_stages\":"<<f.support_aware_accepted_stages
       <<",\"support_aware_no_safe_stage\":"<<(f.support_aware_no_safe_stage?"true":"false")
       <<",\"map_inserted\":"<<(f.map_inserted?"true":"false")
       <<",\"field_ms\":"<<f.field_ms
       <<",\"match_ms\":"<<f.match_ms
       <<",\"insertion_ms\":"<<f.insertion_ms
       <<",\"frame_ms\":"<<f.frame_ms<<"}";
  }
  out<<"]}\n";
  if(!out)throw std::runtime_error("match report write failed");
}
void save_replay(const std::filesystem::path& file,
                 const std::vector<CarmenLaser>& scans,
                 const Fr079SlamResult& result,const std::string& source){
  std::ofstream out(file);
  if(!out)throw std::runtime_error("failed to write FR079 SLAM replay");
  out<<std::setprecision(14)<<"{\"format\":\"FUSREPLAY1\","
     <<"\"source\":{\"dataset\":\"FR079 CARMEN LiDAR odometry offline SLAM\","
     <<"\"frame_id\":\"fr079_odom_scan_frame_UNVERIFIED\","
     <<"\"pose_kind\":\"estimated_fr079_odometry_scan_matched_not_ground_truth\","
     <<"\"timestamp_kind\":\"CARMEN_IPC_scan_level\","
     <<"\"per_beam_time_known\":false,\"map_alignment_verified\":false,"
     <<"\"source_sha256\":\""<<source<<"\"},\"frames\":[";
  for(std::size_t i=0;i<scans.size();++i){
    if(i)out<<",";
    const auto& scan=scans[i];
    const auto& f=result.frames[i];
    out<<"{\"index\":"<<i<<",\"t\":"<<f.scan_time_s
       <<",\"pose\":"<<pose(f.estimated)
       <<",\"odom\":"<<pose(f.odom)
       <<",\"beam_count\":"<<f.beam_count
       <<",\"hit_count\":"<<f.valid_hits
       <<",\"points\":[";
    bool first=true;
    for(std::size_t b=0;b<scan.ranges.size();b+=6){
      const auto& hit=scan.ranges[b];
      if(hit.status!=slam::datasets::RangeStatus::hit||
         !std::isfinite(hit.metres)||hit.metres<.10||
         hit.metres>18||!std::isfinite(hit.angle_rad))continue;
      if(!first)out<<",";
      first=false;
      out<<"["<<hit.metres*std::cos(hit.angle_rad)
         <<","<<hit.metres*std::sin(hit.angle_rad)<<"]";
    }
    out<<"],\"matching\":null}";
  }
  out<<"]}\n";
  if(!out)throw std::runtime_error("replay write failed");
}

void save_loop_diagnostics(const std::filesystem::path& path,
                           const slam::datasets::Fr079LoopResult& loop,
                           const std::string& sha) {
  std::ofstream out(path);
  if(!out)throw std::runtime_error("cannot create FR079 loop decision artifact");
  out<<std::setprecision(15)
     <<"{\"format\":\"FUSFR079LOOP1\",\"source_sha256\":\""<<sha
     <<"\",\"independent_ground_truth\":false"
     <<",\"loop_evidence\":\""
     <<(loop.registration_mode=="global_scan_to_submap_v2"?
           "correlated_raw_lidar_appearance_global_submap_correlative_v2":
           ((loop.registration_mode=="correlative_scan_to_submap_v1" ||
              loop.registration_mode=="distributed_scan_to_submap_v2")?
           "correlated_raw_lidar_frozen_submap_correlative_registration":
           "correlated_raw_lidar_reciprocal_scan_registration"))<<"\""
     <<",\"odometry_source\":\"embedded_scan_time_odom_not_ground_truth\""
     <<",\"sequential_factor_source\":\""<<loop.sequential_factor_source<<"\""
     <<",\"registration_mode\":\""<<loop.registration_mode<<"\""
     <<",\"submaps_built\":"<<loop.submaps_built
     <<",\"submap_registration_attempts\":"<<loop.submap_registration_attempts
     <<",\"submap_geometry_verified\":"<<loop.submap_geometry_verified
     <<",\"submap_ambiguous_rejected\":"<<loop.submap_ambiguous_rejected
     <<",\"last_keyframe_searched\":"<<loop.last_keyframe_searched
     <<",\"verified_candidate_pool\":"<<loop.verified_candidate_pool
     <<",\"late_verified_candidates\":"<<loop.late_verified_candidates
     <<",\"late_selected_loops\":"<<loop.late_selected_loops
     <<",\"appearance_retrieved_candidates\":"<<loop.appearance_retrieved_candidates
     <<",\"appearance_registration_attempts\":"<<loop.appearance_registration_attempts
     <<",\"appearance_geometry_verified\":"<<loop.appearance_geometry_verified
     <<",\"appearance_corrob_rejected\":"<<loop.appearance_corrob_rejected
     <<",\"appearance_accepted_loops\":"<<loop.appearance_accepted_loops
     <<",\"large_heading_nearby_pairs\":"<<loop.large_heading_nearby_pairs
     <<",\"large_heading_registration_attempts\":"<<loop.large_heading_registration_attempts
     <<",\"large_heading_geometry_verified\":"<<loop.large_heading_geometry_verified
     <<",\"large_heading_accepted_loops\":"<<loop.large_heading_accepted_loops
     <<",\"proposed_loops\":"<<loop.proposed_loops
     <<",\"accepted_loops\":"<<loop.accepted_loops
     <<",\"nearby_pairs\":"<<loop.nearby_pairs
     <<",\"registration_attempts\":"<<loop.registration_attempts
     <<",\"geometry_verified\":"<<loop.geometry_verified
     <<",\"redundant_suppressed\":"<<loop.redundant_suppressed
     <<",\"graph_control_initial_cost\":"<<loop.control_initial_cost
     <<",\"graph_control_final_cost\":"<<loop.control_final_cost
     <<",\"graph_loop_initial_cost\":"<<loop.loop_initial_cost
     <<",\"graph_loop_final_cost\":"<<loop.loop_final_cost
     <<",\"first_pose_fixed\":"<<(loop.first_pose_fixed?"true":"false")
     <<",\"maximum_graph_shift_m\":"<<loop.maximum_graph_shift_m
     <<",\"mean_graph_shift_m\":"<<loop.mean_graph_shift_m
     <<",\"maximum_graph_shift_yaw_rad\":"<<loop.maximum_graph_shift_yaw_rad
     <<",\"graph_shift_within_safety_limit\":"
     <<(loop.graph_shift_within_safety_limit?"true":"false")
     <<",\"loop_decisions\":[";
  for(std::size_t k=0;k<loop.decisions.size();++k) {
    const auto& d=loop.decisions[k];
    if(k)out<<",";
    out<<"{\"from\":"<<d.from<<",\"to\":"<<d.to
       <<",\"decision\":\""<<d.decision<<"\""
       <<",\"forward_inliers\":"<<d.forward_inliers
       <<",\"reverse_inliers\":"<<d.reverse_inliers
       <<",\"overlap\":"<<d.overlap
       <<",\"rms_m\":"<<d.rms_m
       <<",\"innovation_m\":"<<d.innovation_m
       <<",\"innovation_yaw_rad\":"<<d.innovation_yaw_rad
       <<",\"predicted_relative_yaw_rad\":"<<d.predicted_relative_yaw_rad
       <<",\"submap_contributing_scans\":"<<d.submap_contributing_scans
       <<",\"submap_score\":"<<d.submap_score
       <<",\"submap_support_fraction\":"<<d.submap_support_fraction
       <<",\"submap_translation_curvature_min\":"<<d.submap_translation_curvature_min
       <<",\"submap_translation_curvature_ratio\":"<<d.submap_translation_curvature_ratio
       <<",\"submap_minimum_shift_score_drop\":"<<d.submap_minimum_shift_score_drop
       <<",\"appearance_retrieved\":"<<(d.appearance_retrieved?"true":"false")
       <<",\"retrieval_similarity\":"<<d.retrieval_similarity
       <<",\"multi_anchor_consistent\":"<<(d.multi_anchor_consistent?"true":"false")
       <<"}";
  }
  out<<"],\""<<(loop.sequential_factor_source==
      "frontend_scanmatched_relative_correlated_v1"?
        "graph_control_poses":"odometry_graph_poses")<<"\":[";
  for(std::size_t k=0;k<loop.odometry_graph_poses.size();++k) {
    if(k)out<<",";
    out<<pose(loop.odometry_graph_poses[k]);
  }
  out<<"],\"loop_graph_poses\":[";
  for(std::size_t k=0;k<loop.loop_graph_poses.size();++k) {
    if(k)out<<",";
    out<<pose(loop.loop_graph_poses[k]);
  }
  out<<"],\"submap_regression_probes\":[";
  for(std::size_t k=0;k<loop.submap_probes.size();++k) {
    const auto& p=loop.submap_probes[k];
    if(k)out<<",";
    out<<"{\"from\":"<<p.from<<",\"to\":"<<p.to
       <<",\"accepted\":"<<(p.accepted?"true":"false")
       <<",\"observable\":"<<(p.observable?"true":"false")
       <<",\"predicted_relative_yaw_rad\":"<<p.predicted_relative_yaw_rad
       <<",\"score\":"<<p.score
       <<",\"support_fraction\":"<<p.support_fraction
       <<",\"translation_curvature_min\":"<<p.translation_curvature_min
       <<",\"translation_curvature_ratio\":"<<p.translation_curvature_ratio
       <<",\"minimum_shift_score_drop\":"<<p.minimum_shift_score_drop
       <<"}";
  }
  out<<"]}\n";
  if(!out)throw std::runtime_error("failed writing FR079 graph artifact");
}
void save_graph_map(const std::filesystem::path& file,
                    const slam::mapping::MapBundle& bundle) {
  {
    std::ofstream out(file,std::ios::binary);
    if(!out)throw std::runtime_error("failed creating FR079 graph map");
    slam::mapping::write_map_bundle(out,bundle);
  }
  std::ifstream in(file,std::ios::binary);
  if(!in||slam::mapping::read_map_bundle(in).snapshot->fingerprint()!=
          bundle.snapshot->fingerprint())
    throw std::runtime_error("FR079 graph map readback fingerprint mismatch");
}

void save_report(const std::filesystem::path& file,
                 const Fr079SlamResult& result,
                 const std::string& source,
                 const slam::datasets::Fr079SlamOptions& options,
                 double loading_ms,double slam_ms,double export_ms,
                 const slam::datasets::Fr079LoopResult* loop=nullptr,
                 double graph_compute_ms=0) {
  std::ofstream out(file);
  if(!out)throw std::runtime_error("failed to write FR079 report");
  std::vector<double> per_frame,fields,matches,inserts;
  per_frame.reserve(result.frames.size());
  for(const auto& f:result.frames){
    per_frame.push_back(f.frame_ms);
    if(f.field_ms>0)fields.push_back(f.field_ms);
    if(f.matched)matches.push_back(f.match_ms);
    if(f.map_inserted)inserts.push_back(f.insertion_ms);
  }
  const auto frames=result.frames.size();
  const double span=result.frames.back().scan_time_s-result.frames.front().scan_time_s;
  const double scan_rate=(frames>1&&span>0)?(frames-1)/span:0;
  const double compute_ms=std::max(0.0,slam_ms-result.capture_benchmark_ms);
  out<<std::setprecision(12)
     <<"{\"format\":\"FUSFR079SLAM1\","
     <<"\"dataset\":\"FR079 CARMEN LiDAR and logged odometry\","
     <<"\"source_sha256\":\""<<source<<"\","
     <<"\"frames\":"<<frames
     <<",\"scan_input_rate_hz\":"<<scan_rate
     <<",\"initial_map_empty\":true"
     <<",\"uses_logged_laser_pose_as_input\":false"
     <<",\"odometry_source\":\"embedded_FLASER_logged_odometry_at_scan_time\""
     <<",\"odometry_not_independent_truth\":true"
     <<",\"laser_extrinsic\":\"assumed_identity_NOT_CALIBRATED\""
     <<",\"per_beam_time_verified\":false"
     <<",\"map_frame_verified\":false"
     <<",\"independent_ground_truth\":false"
     <<",\"external_map_loaded\":false"
     <<",\"loop_closures\":"<<(loop?loop->accepted_loops:0)
     <<",\"pose_graph_optimized\":"<<(loop?"true":"false")
     <<",\"realtime_deadlines_verified\":false"
     <<",\"loop_mode\":\""<<(loop?
         (loop->sequential_factor_source=="frontend_scanmatched_relative_correlated_v1"?
             "fr079_frontend_relative_gtsam_loops_v2":
             "fr079_lidar_scan_registration_gtsam_v1"):"none")<<"\""
     <<",\"sequential_factor_source\":\""<<(loop?loop->sequential_factor_source:"none")<<"\""
     <<",\"loop_registration_mode\":\""<<(loop?loop->registration_mode:"none")<<"\""
     <<",\"loop_large_heading_accepted\":"<<(loop?loop->large_heading_accepted_loops:0)
     <<",\"loop_submap_geometry_verified\":"<<(loop?loop->submap_geometry_verified:0)
     <<",\"loop_last_keyframe_searched\":"<<(loop?loop->last_keyframe_searched:0)
     <<",\"loop_late_selected\":"<<(loop?loop->late_selected_loops:0)
     <<",\"loop_appearance_accepted\":"<<(loop?loop->appearance_accepted_loops:0)
     <<",\"loop_candidates\":"<<(loop?loop->proposed_loops:0)
     <<",\"loop_registration_attempts\":"<<(loop?loop->registration_attempts:0)
     <<",\"loop_geometry_verified\":"<<(loop?loop->geometry_verified:0)
     <<",\"graph_shift_within_safety_limit\":"
     <<(loop&&loop->graph_shift_within_safety_limit?"true":"false")
     <<",\"matcher\":\""<<(result.used_continuous_cost_ceres?"ceres_continuous_cost":(result.used_support_aware_ceres?"ceres_support_aware":(result.used_ceres?"ceres":"geometry_reference_fallback")))<<"\""
     <<",\"distance_truncation_m\":"<<options.distance_truncation_m
     <<",\"ablation_mode\":\""<<options.ablation_mode<<"\""
     <<",\"ceres_invalid_residual_evaluations\":"<<result.total_invalid_residual_evaluations
     <<",\"ceres_continuous_unknown_evaluations\":"<<result.total_continuous_unknown_evaluations
     <<",\"ceres_continuous_truncated_evaluations\":"<<result.total_continuous_truncated_evaluations
     <<",\"ceres_continuous_outside_evaluations\":"<<result.total_continuous_outside_evaluations
     <<",\"ceres_continuous_other_evaluations\":"<<result.total_continuous_other_evaluations
     <<",\"ceres_invalid_unknown_cells\":"<<result.total_invalid_unknown_cells
     <<",\"ceres_invalid_truncated\":"<<result.total_invalid_truncated
     <<",\"ceres_invalid_outside_grid\":"<<result.total_invalid_outside_grid
     <<",\"ceres_invalid_other\":"<<result.total_invalid_other
     <<",\"ceres_unsuccessful_steps\":"<<result.total_unsuccessful_steps
     <<",\"support_aware_selected_hits\":"<<result.total_support_aware_selected_hits
     <<",\"support_aware_skipped_hits\":"<<result.total_support_aware_skipped_hits
     <<",\"support_aware_stages\":"<<result.total_support_aware_stages
     <<",\"support_aware_accepted_stages\":"<<result.total_support_aware_accepted_stages
     <<",\"support_aware_no_safe_stages\":"<<result.total_support_aware_no_safe_stages
     <<",\"match_every\":"<<options.match_every
     <<",\"field_rebuild_every\":"<<options.rebuild_field_every
     <<",\"map_beam_stride\":"<<options.map_beam_stride
     <<",\"match_beam_stride\":"<<options.match_beam_stride
     <<",\"match_beam_phase\":"<<options.match_beam_phase
     <<",\"attempts\":"<<result.attempts
     <<",\"accepted\":"<<result.accepted
     <<",\"rejected\":"<<result.rejected
     <<",\"inserted_scans\":"<<result.inserted_scans
     <<",\"map\":{\"width\":"<<result.map.snapshot->config().width
     <<",\"height\":"<<result.map.snapshot->config().height
     <<",\"resolution_m\":"<<result.map.snapshot->config().resolution_m
     <<",\"revision\":"<<result.map.snapshot->revision()
     <<",\"fingerprint\":\""<<result.map.snapshot->fingerprint()<<"\"}"
     <<",\"graph_control_map\":"
     <<(loop?("{\"fingerprint\":\""+
         std::to_string(loop->odometry_graph_map.snapshot->fingerprint())+
         "\",\"revision\":"+
         std::to_string(loop->odometry_graph_map.snapshot->revision())+"}"):
         "null")
     <<",\"graph_optimized_map\":"
     <<(loop?("{\"fingerprint\":\""+
         std::to_string(loop->loop_graph_map.snapshot->fingerprint())+
         "\",\"revision\":"+
         std::to_string(loop->loop_graph_map.snapshot->revision())+"}"):
         "null")
      <<",\"map_preview\":{\"source\":\""
      <<(loop?"graph_optimized":"frontend")
      <<"\",\"fingerprint\":\""
      <<(loop?loop->loop_graph_map.snapshot->fingerprint():
               result.map.snapshot->fingerprint())
      <<"\"}"
     <<",\"profiling\":{\"clock\":\"std::chrono::steady_clock\""
     <<",\"input_load_ms\":"<<loading_ms
     <<",\"slam_compute_ms\":"<<compute_ms
     <<",\"graph_compute_ms\":"<<graph_compute_ms
     <<",\"slam_wall_including_capture_ms\":"<<slam_ms
     <<",\"capture_benchmark_ms\":"<<result.capture_benchmark_ms
     <<",\"estimated_offline_fps\":"<<(compute_ms>0?frames*1000./compute_ms:0)
     <<",\"artifact_export_before_report_ms\":"<<export_ms
     <<",\"field_total_ms\":"<<result.total_field_ms
     <<",\"matching_total_ms\":"<<result.total_match_ms
     <<",\"insertion_total_ms\":"<<result.total_insertion_ms
     <<",\"frame_latency\":"<<duration_summary(per_frame)
     <<",\"field_build\":"<<duration_summary(fields)
     <<",\"scan_matching\":"<<duration_summary(matches)
     <<",\"map_insertion\":"<<duration_summary(inserts)
     <<"},\"note\":\"Offline timestamp-order replay; not a live streaming deadline benchmark\"}\n";
  if(!out)throw std::runtime_error("FR079 report write failed");
}
double positive_distance(const std::string& s) {
  std::size_t used=0;
  const double v=std::stod(s,&used);
  ensure(used==s.size() && std::isfinite(v) && v>=0.20 && v<=3.0,
         "distance truncation must be finite and within [0.20,3.0] m");
  return v;
}
std::size_t positive_integer(const std::string& s){
  if(s.empty()||s.size()>4)throw std::invalid_argument("invalid positive integer option");
  for(char c:s)if(c<'0'||c>'9')throw std::invalid_argument("invalid integer option");
  const auto n=static_cast<std::size_t>(std::stoul(s));
  if(n<1||n>200)throw std::invalid_argument("option must be between 1 and 200");
  return n;
}
int run(int argc,char** argv){
  cxxopts::Options cli("slam_fr079_offline",
      "Offline FR079 CARMEN LiDAR and odometry SLAM");
  cli.add_options()
      ("log","FR079 recording",cxxopts::value<std::string>())
      ("prefix","New output prefix or new run directory",cxxopts::value<std::string>())
      ("run-directory","Create an exclusive directory and save run-prefixed files inside")
      ("reference","Use geometry-reference matcher without Ceres")
      ("support-aware","Enable opt-in certified-support Ceres matcher")
      ("continuous-cost","Enable opt-in finite cost for unknown EDT cells")
      ("lidar-loop-closure","Detect reciprocal LiDAR revisits and optimize an offline GTSAM graph")
      ("frontend-sequential-graph","Use correlated frontend-relative sequential factors instead of logged odometry (requires --lidar-loop-closure)")
      ("loop-registration","Loop matching method: legacy-scan (default), pose-prior-scan, scan-to-submap, scan-to-submap-distributed, scan-to-submap-global",cxxopts::value<std::string>())
      ("ablation","Experimental finite Ceres variant: finite-seed-legacy, finite-all-legacy, finite-seed-continuous",cxxopts::value<std::string>())
      ("capture","Write frozen-map pose perturbation benchmark")
      ("capture-every","Capture interval",cxxopts::value<std::string>())
      ("capture-max-frames","Maximum captured frame count",cxxopts::value<std::string>())
      ("match-every","Match once per N scans",cxxopts::value<std::string>())
      ("field-every","Rebuild distance field every N scans",cxxopts::value<std::string>())
      ("map-stride","Use every Nth valid beam for occupancy updates",cxxopts::value<std::string>())
      ("match-stride","Use every Nth original beam for matching",cxxopts::value<std::string>())
      ("match-phase","Starting raw beam index within match stride (0-based)",cxxopts::value<std::string>())
      ("distance-truncation","EDT truncation in metres",cxxopts::value<std::string>())
      ("h,help","Print help");
  cli.parse_positional({"log","prefix"});
  cli.positional_help("<fr079.log[.gz]> <new-output-prefix>");
  cxxopts::ParseResult args;
  try{args=cli.parse(argc,argv);}
  catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
  if(args.count("help")&&args["help"].as<bool>()){std::cout<<cli.help();return 0;}
  if(!args.count("log")||!args.count("prefix")){
    std::cerr<<cli.help();return 2;
  }
  slam::datasets::Fr079SlamOptions options;
  options.use_ceres=!(args.count("reference")&&args["reference"].as<bool>());
  options.support_aware_ceres=args.count("support-aware")&&args["support-aware"].as<bool>();
  options.continuous_cost_ceres=args.count("continuous-cost")&&args["continuous-cost"].as<bool>();
  if(args.count("ablation")){
    options.ablation_mode=args["ablation"].as<std::string>();
    ensure(options.continuous_cost_ceres&&options.use_ceres&&
           !options.support_aware_ceres,
           "--ablation requires --continuous-cost, not --reference/--support-aware");
    ensure(options.ablation_mode=="finite-seed-legacy"||
           options.ablation_mode=="finite-all-legacy"||
           options.ablation_mode=="finite-seed-continuous",
           "invalid --ablation mode");
  }
  const bool lidar_loop_closure=args.count("lidar-loop-closure")&&
      args["lidar-loop-closure"].as<bool>();
  const bool frontend_sequential_graph=args.count("frontend-sequential-graph")&&
      args["frontend-sequential-graph"].as<bool>();
  ensure(!frontend_sequential_graph||lidar_loop_closure,
         "--frontend-sequential-graph requires --lidar-loop-closure");
  const std::string loop_registration=args.count("loop-registration")?
      args["loop-registration"].as<std::string>():"legacy-scan";
  ensure(loop_registration=="legacy-scan" ||
         loop_registration=="pose-prior-scan" ||
         loop_registration=="scan-to-submap" ||
         loop_registration=="scan-to-submap-distributed" ||
         loop_registration=="scan-to-submap-global",
         "invalid --loop-registration mode");
  ensure(!args.count("loop-registration")||lidar_loop_closure,
         "--loop-registration requires --lidar-loop-closure");
  const auto registration_mode=loop_registration=="scan-to-submap-global"?
      slam::datasets::Fr079LoopRegistrationMode::globally_retrieved_scan_to_submap:
      (loop_registration=="scan-to-submap-distributed"?
        slam::datasets::Fr079LoopRegistrationMode::distributed_scan_to_submap:
        (loop_registration=="scan-to-submap"?
          slam::datasets::Fr079LoopRegistrationMode::correlative_scan_to_submap:
          (loop_registration=="pose-prior-scan"?
            slam::datasets::Fr079LoopRegistrationMode::pose_prior_scan_to_scan:
            slam::datasets::Fr079LoopRegistrationMode::legacy_scan_to_scan)));
#ifndef SLAM_FR079_WITH_GTSAM
  ensure(!lidar_loop_closure,
         "--lidar-loop-closure requires a native build with GTSAM");
#endif
  const bool capture=args.count("capture")&&args["capture"].as<bool>();
  ensure(!capture||options.ablation_mode=="none",
         "--capture cannot be combined with --ablation");
  const auto positive=[&](const char* key,std::size_t fallback) {
    return args.count(key)?positive_integer(args[key].as<std::string>()):fallback;
  };
  const std::size_t capture_every=positive("capture-every",120);
  const std::size_t capture_max_frames=positive("capture-max-frames",12);
  options.match_every=positive("match-every",options.match_every);
  options.rebuild_field_every=positive("field-every",options.rebuild_field_every);
  options.map_beam_stride=positive("map-stride",options.map_beam_stride);
  options.match_beam_stride=positive("match-stride",options.match_beam_stride);
  if(args.count("match-phase")){
    const auto raw=args["match-phase"].as<std::string>();
    ensure(!raw.empty()&&raw.size()<=2&&
           std::all_of(raw.begin(),raw.end(),[](char c){return c>='0'&&c<='9';}),
           "match-phase must be a nonnegative integer");
    options.match_beam_phase=static_cast<std::size_t>(std::stoul(raw));
  }
  ensure(options.match_beam_phase<options.match_beam_stride,
         "match-phase must be less than match-stride");
  if(args.count("distance-truncation"))
    options.distance_truncation_m=positive_distance(
        args["distance-truncation"].as<std::string>());
  ensure(capture_max_frames<=100,"capture-max-frames must be at most 100");
  const std::filesystem::path requested(args["prefix"].as<std::string>());
  ensure(!requested.filename().empty()&&
         std::filesystem::is_directory(requested.parent_path()),
         "output parent must already be a directory");
  const bool run_directory=args.count("run-directory")&&
      args["run-directory"].as<bool>();
  std::filesystem::path prefix=requested;
  if(run_directory) {
    const auto id=requested.filename().string();
    ensure(!id.empty()&&id.size()<=80&&
           ((id.front()>='a'&&id.front()<='z')||
            (id.front()>='A'&&id.front()<='Z')||
            (id.front()>='0'&&id.front()<='9')) &&
           std::all_of(id.begin(),id.end(),[](unsigned char ch){
             return (ch>='a'&&ch<='z')||(ch>='A'&&ch<='Z')||
                    (ch>='0'&&ch<='9')||ch=='_'||ch=='-';
           }),"run-directory name must be 1-80 ASCII letters/digits/_/-");
    ensure(!std::filesystem::exists(std::filesystem::symlink_status(requested)),
           "run directory already exists; refusing overwrite");
    prefix=requested/id;
  }
  const auto binary=prefix.string()+".slammap";
  const auto replay=prefix.string()+".slamreplay.json";
  const auto trajectory=prefix.string()+".trajectory.json";
  const auto match=prefix.string()+".matches.json";
  const auto pgm=prefix.string()+".pgm";
  const auto yaml=prefix.string()+".yaml";
  const auto report=prefix.string()+".report.json";
  const auto capture_csv=prefix.string()+".capture.csv";
  const auto review=prefix.string()+".review.json";
  const auto loops_json=prefix.string()+".loops.json";
  const auto graph_control_map=prefix.string()+".graph_control.slammap";
  const auto graph_optimized_map=prefix.string()+".graph_optimized.slammap";
  for(const auto& f:{binary,replay,trajectory,match,pgm,yaml,report,review})
    ensure(!std::filesystem::exists(std::filesystem::symlink_status(f)),
           "output already exists; refusing overwrite");
  if(lidar_loop_closure)
    for(const auto& f:{loops_json,graph_control_map,graph_optimized_map})
      ensure(!std::filesystem::exists(std::filesystem::symlink_status(f)),
             "loop artifact already exists; refusing overwrite");
  if(capture)
    ensure(!std::filesystem::exists(std::filesystem::symlink_status(capture_csv)),
           "capture output already exists; refusing overwrite");
  const auto begin=Clock::now();
  const auto content=read_log(args["log"].as<std::string>());
  const auto digest=sha256(content); // hash uncompressed CARMEN bytes, not filename
  const auto scans=parse(content);
  // Do not create an empty run directory for unreadable/malformed input.
  // The exclusive folder is created before any capture or map artifacts.
  if(run_directory)
    ensure(std::filesystem::create_directory(requested),
           "failed creating exclusive run directory");
  std::unique_ptr<slam::datasets::Fr079CaptureBenchmark> capture_benchmark;
  if(capture) {
    capture_benchmark=std::make_unique<slam::datasets::Fr079CaptureBenchmark>(
        capture_csv,digest,options.use_ceres,options.support_aware_ceres,
        options.continuous_cost_ceres,
        capture_every,capture_max_frames);
    options.capture_probe=[&](std::size_t index,const slam::Pose2& anchor,
        std::shared_ptr<const slam::mapping::MapSnapshot> prior,
        const std::vector<Eigen::Vector2d>& points) {
      capture_benchmark->probe(index,anchor,std::move(prior),points);
    };
  }
  const auto loaded=Clock::now();
  const auto result=slam::datasets::run_fr079_offline_slam(scans,options);
  const std::size_t capture_trials=capture_benchmark?capture_benchmark->tested_trials():0;
  capture_benchmark.reset();
  std::optional<slam::datasets::Fr079LoopResult> loop;
  const auto graph_start=Clock::now();
#ifdef SLAM_FR079_WITH_GTSAM
  if(lidar_loop_closure)
    loop=slam::datasets::run_fr079_loop_experiment(
        scans,result,options,frontend_sequential_graph,registration_mode);
#endif
  const auto computed=Clock::now();
  const double graph_compute_ms=loop?elapsed_ms(graph_start,computed):0.;
  {
    std::ofstream out(binary,std::ios::binary);
    if(!out)throw std::runtime_error("cannot open FUSMAP2D output");
    slam::mapping::write_map_bundle(out,result.map);
  }
  {
    std::ifstream in(binary,std::ios::binary);
    const auto verified=slam::mapping::read_map_bundle(in);
    ensure(verified.snapshot->fingerprint()==result.map.snapshot->fingerprint(),
           "FUSMAP2D output integrity mismatch");
  }
  // For loop runs the .pgm/.yaml depict the optimized graph's occupancy,
  // while .slammap and causal replay still preserve the Ceres frontend.
  const auto& preview_map=loop?loop->loop_graph_map:result.map;
  {
    std::ofstream out(pgm,std::ios::binary);
    if(!out)throw std::runtime_error("cannot open PGM output");
    slam::mapping::write_map_pgm(out,*preview_map.snapshot);
  }
  {
    std::ofstream out(yaml);
    if(!out)throw std::runtime_error("cannot open YAML output");
    slam::mapping::write_map_yaml(out,*preview_map.snapshot,
                                 std::filesystem::path(pgm).filename().string());
  }
  save_trajectory(trajectory,result,digest);
  save_matches(match,result,digest);
  save_replay(replay,scans,result,digest);
  if(loop) {
    save_loop_diagnostics(loops_json,*loop,digest);
    save_graph_map(graph_control_map,loop->odometry_graph_map);
    save_graph_map(graph_optimized_map,loop->loop_graph_map);
  }
  const auto exported=Clock::now();
  save_report(report,result,digest,options,elapsed_ms(begin,loaded),
              elapsed_ms(loaded,computed),elapsed_ms(computed,exported),
              loop?&*loop:nullptr,graph_compute_ms);
  std::cout<<"FR079 offline LiDAR+odometry SLAM: scans="<<scans.size()
           <<" matched="<<result.accepted<<"/"<<result.attempts
           <<" inserted="<<result.inserted_scans
           <<" map="<<binary
           <<" replay="<<replay
           <<" report="<<report
           <<(loop?" accepted_loops="+std::to_string(loop->accepted_loops)+
                    " loop_decisions="+loops_json:"")
           <<(capture?" capture_trials="+std::to_string(capture_trials):"")<<"\n";
  std::vector<slam::cli::ReviewFile> files={
      {"report",report,true},
      {"trajectory",trajectory,true},
      {"matching_decisions",match,true},
      {"replay",replay,true},
      {"occupancy_map",binary,false},
      {"map_preview",pgm,false},
      {"map_metadata",yaml,false},
  };
  if(capture)files.push_back({"capture_diagnostics",capture_csv,true});
  if(loop) {
    files.push_back({"loop_decisions_and_graph_poses",loops_json,true});
    files.push_back({frontend_sequential_graph?
        "frontend_sequential_graph_control_map":"odometry_only_graph_map",
        graph_control_map,false});
    files.push_back({"loop_optimized_map",graph_optimized_map,false});
  }
  slam::cli::write_review_manifest(review,"fr079_offline",files);
  return 0;
}
} // namespace
int main(int argc,char** argv){
  try{return run(argc,argv);}
  catch(const std::exception& e){
    std::cerr<<"FR079 offline SLAM refused: "<<e.what()<<'\n';
    return 1;
  }
}
