#include "slam/datasets/fr079_offline.hpp"
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
       <<",\"supported_hits\":"<<f.supported_hits
       <<",\"overlap\":"<<f.overlap
       <<",\"robust_cost\":"<<f.robust_cost
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
void save_report(const std::filesystem::path& file,
                 const Fr079SlamResult& result,
                 const std::string& source,
                 const slam::datasets::Fr079SlamOptions& options,
                 double loading_ms,double slam_ms,double export_ms) {
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
  const double compute_ms=slam_ms;
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
     <<",\"loop_closures\":0"
     <<",\"pose_graph_optimized\":false"
     <<",\"realtime_deadlines_verified\":false"
     <<",\"matcher\":\""<<(result.used_ceres?"ceres":"geometry_reference_fallback")<<"\""
     <<",\"match_every\":"<<options.match_every
     <<",\"field_rebuild_every\":"<<options.rebuild_field_every
     <<",\"map_beam_stride\":"<<options.map_beam_stride
     <<",\"match_beam_stride\":"<<options.match_beam_stride
     <<",\"attempts\":"<<result.attempts
     <<",\"accepted\":"<<result.accepted
     <<",\"rejected\":"<<result.rejected
     <<",\"inserted_scans\":"<<result.inserted_scans
     <<",\"map\":{\"width\":"<<result.map.snapshot->config().width
     <<",\"height\":"<<result.map.snapshot->config().height
     <<",\"resolution_m\":"<<result.map.snapshot->config().resolution_m
     <<",\"revision\":"<<result.map.snapshot->revision()
     <<",\"fingerprint\":\""<<result.map.snapshot->fingerprint()<<"\"}"
     <<",\"profiling\":{\"clock\":\"std::chrono::steady_clock\""
     <<",\"input_load_ms\":"<<loading_ms
     <<",\"slam_compute_ms\":"<<compute_ms
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
std::size_t positive_integer(const std::string& s){
  if(s.empty()||s.size()>4)throw std::invalid_argument("invalid positive integer option");
  for(char c:s)if(c<'0'||c>'9')throw std::invalid_argument("invalid integer option");
  const auto n=static_cast<std::size_t>(std::stoul(s));
  if(n<1||n>200)throw std::invalid_argument("option must be between 1 and 200");
  return n;
}
int run(int argc,char** argv){
  if(argc<3){
    std::cerr<<"Usage: slam_fr079_offline <fr079.log[.gz]> <new-output-prefix>"
                " [--reference] [--match-every N] [--field-every N]"
                " [--map-stride N] [--match-stride N]\n";
    return 2;
  }
  slam::datasets::Fr079SlamOptions options;
  for(int i=3;i<argc;++i){
    const std::string arg=argv[i];
    if(arg=="--reference")options.use_ceres=false;
    else if(arg=="--match-every"&&i+1<argc)options.match_every=positive_integer(argv[++i]);
    else if(arg=="--field-every"&&i+1<argc)options.rebuild_field_every=positive_integer(argv[++i]);
    else if(arg=="--map-stride"&&i+1<argc)options.map_beam_stride=positive_integer(argv[++i]);
    else if(arg=="--match-stride"&&i+1<argc)options.match_beam_stride=positive_integer(argv[++i]);
    else throw std::invalid_argument("unrecognized or incomplete FR079 option");
  }
  const std::filesystem::path prefix(argv[2]);
  ensure(!prefix.filename().empty()&&std::filesystem::is_directory(prefix.parent_path()),
         "output parent must already be a directory");
  const auto binary=prefix.string()+".slammap";
  const auto replay=prefix.string()+".slamreplay.json";
  const auto trajectory=prefix.string()+".trajectory.json";
  const auto match=prefix.string()+".matches.json";
  const auto pgm=prefix.string()+".pgm";
  const auto yaml=prefix.string()+".yaml";
  const auto report=prefix.string()+".report.json";
  for(const auto& f:{binary,replay,trajectory,match,pgm,yaml,report})
    ensure(!std::filesystem::exists(std::filesystem::symlink_status(f)),
           "output already exists; refusing overwrite");
  const auto begin=Clock::now();
  const auto content=read_log(argv[1]);
  const auto digest=sha256(content); // hash uncompressed CARMEN bytes, not filename
  const auto scans=parse(content);
  const auto loaded=Clock::now();
  const auto result=slam::datasets::run_fr079_offline_slam(scans,options);
  const auto computed=Clock::now();
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
  {
    std::ofstream out(pgm,std::ios::binary);
    if(!out)throw std::runtime_error("cannot open PGM output");
    slam::mapping::write_map_pgm(out,*result.map.snapshot);
  }
  {
    std::ofstream out(yaml);
    if(!out)throw std::runtime_error("cannot open YAML output");
    slam::mapping::write_map_yaml(out,*result.map.snapshot,
                                 std::filesystem::path(pgm).filename().string());
  }
  save_trajectory(trajectory,result,digest);
  save_matches(match,result,digest);
  save_replay(replay,scans,result,digest);
  const auto exported=Clock::now();
  save_report(report,result,digest,options,elapsed_ms(begin,loaded),
              elapsed_ms(loaded,computed),elapsed_ms(computed,exported));
  std::cout<<"FR079 offline LiDAR+odometry SLAM: scans="<<scans.size()
           <<" matched="<<result.accepted<<"/"<<result.attempts
           <<" inserted="<<result.inserted_scans
           <<" map="<<binary
           <<" replay="<<replay
           <<" report="<<report<<"\n";
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
