// Offline Ceres scan-to-frozen-map diagnostics for the read-only replay.
// No graph factor, map update, ground truth, or frame calibration is inferred.
#include "slam/datasets/carmen.hpp"
#include "slam/datasets/fr079_train_only.hpp"
#include "slam/mapping/map_bundle.hpp"
#include "slam/matching/geometry.hpp"

#include <zlib.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
constexpr std::size_t kMaxSourceBytes=128*1024*1024;
std::string source_text(const std::string& path) {
  std::string result;
  if (path.ends_with(".gz")) {
    gzFile file=gzopen(path.c_str(),"rb");
    if (!file) throw std::runtime_error("cannot open compressed CARMEN log");
    char buf[8192];
    int n=0;
    while ((n=gzread(file,buf,sizeof(buf)))>0) {
      if (static_cast<std::size_t>(n)>kMaxSourceBytes-result.size()) {
        gzclose(file); throw std::runtime_error("CARMEN log expands beyond 128 MiB");
      }
      result.append(buf,static_cast<std::size_t>(n));
    }
    const int state=gzclose(file);
    if(n<0 || state!=Z_OK)throw std::runtime_error("invalid compressed CARMEN log");
  } else {
    std::ifstream in(path,std::ios::binary);
    if(!in) throw std::runtime_error("cannot open CARMEN log");
    char buf[8192];
    while(in.read(buf,sizeof(buf)) || in.gcount()) {
      const auto count=static_cast<std::size_t>(in.gcount());
      if(count>kMaxSourceBytes-result.size())throw std::runtime_error("CARMEN log exceeds 128 MiB");
      result.append(buf,count);
    }
    if(in.bad())throw std::runtime_error("CARMEN log read failed");
  }
  return result;
}
std::size_t posint(const char* arg,std::size_t max) {
  std::string_view v(arg);
  if(v.empty()||v.front()=='0' || v.find_first_not_of("0123456789")!=v.npos)
    throw std::invalid_argument("strides must be positive decimal integers");
  const auto n=std::stoull(std::string(v));
  if(n<1||n>max)throw std::invalid_argument("stride exceeds limit");
  return static_cast<std::size_t>(n);
}
const char* status(slam::matching::MatchStatus s) {
  using slam::matching::MatchStatus;
  switch(s){
    case MatchStatus::accepted:return "accepted";
    case MatchStatus::insufficient_support:return "insufficient_support";
    case MatchStatus::degenerate:return "degenerate";
    case MatchStatus::no_improvement:return "no_improvement";
    case MatchStatus::invalid_input:return "invalid_input";
  }
  return "invalid_input";
}
void number_or_null(std::ostream& o,double x){if(std::isfinite(x))o<<x;else o<<"null";}
void pose(std::ostream& o,const slam::Pose2& p) {
  o<<'['<<p.translation.x()<<','<<p.translation.y()<<','<<p.yaw_rad<<']';
}
void emit(const std::vector<slam::datasets::CarmenLaser>& scans,
          const slam::mapping::MapBundle& map,
          const std::shared_ptr<const slam::mapping::DistanceField>& field,
          std::size_t query_stride,std::size_t beam_stride,bool heldout_only) {
  std::cout<<std::setprecision(17);
  std::cout<<"{\"format\":\"FUSMATCH_NATIVE1\",\"matcher\":\"ceres_scan_to_edt\","
           <<"\"map_fingerprint\":\""<<map.snapshot->fingerprint()<<"\","
           <<"\"map_frame_id\":\""<<map.provenance.frame_id<<"\","
           <<"\"map_revision\":\""<<map.snapshot->revision()<<"\","
           <<"\"map_origin_verified\":"<<(map.provenance.metric_origin_verified?"true":"false")<<','
           <<"\"frame_count\":"<<scans.size()
           <<",\"selection\":\""<<(heldout_only?"fr079_heldout_mod5_verified":"sampled_unrestricted")
           <<"\",\"query_stride\":"<<query_stride
           <<",\"beam_stride\":"<<beam_stride<<",\"matches\":[";
  bool first=true;
  std::size_t eligible_seen=0;
  for(std::size_t i=0;i<scans.size();++i){
    if(heldout_only && i%5!=0)continue;
    if((eligible_seen++)%query_stride!=0)continue;
    const auto& scan=scans[i];
    std::vector<Eigen::Vector2d> hits;
    hits.reserve(scan.ranges.size()/beam_stride+1);
    for(std::size_t b=0;b<scan.ranges.size();b+=beam_stride){
      const auto& beam=scan.ranges[b];
      if(beam.status==slam::datasets::RangeStatus::hit && beam.metres>=0.10 && beam.metres<=18.)
        hits.emplace_back(beam.metres*std::cos(beam.angle_rad),beam.metres*std::sin(beam.angle_rad));
    }
    const slam::Pose2 seed{{scan.logged_laser_pose.x,scan.logged_laser_pose.y},scan.logged_laser_pose.yaw};
    const auto initial=slam::matching::evaluate_geometry(*field,hits,seed,{});
    const auto matched=slam::matching::match_ceres(field,hits,seed);
    if(!first)std::cout<<',';
    first=false;
    std::cout<<"{\"index\":"<<i<<",\"t\":"<<scan.ipc_seconds
             <<",\"seed\":";pose(std::cout,seed);
    std::cout<<",\"estimate\":";pose(std::cout,matched.map_from_laser);
    std::cout<<",\"status\":\""<<status(matched.status)<<"\",\"iterations\":"<<matched.iterations
             <<",\"input_hits\":"<<matched.diagnostics.input_hits
             <<",\"supported_hits\":"<<matched.diagnostics.supported_hits
             <<",\"overlap\":";number_or_null(std::cout,matched.diagnostics.overlap);
    std::cout<<",\"initial_robust_cost\":";number_or_null(std::cout,initial.robust_cost);
    std::cout<<",\"robust_cost\":";number_or_null(std::cout,matched.diagnostics.robust_cost);
    std::cout<<",\"mean_distance_m\":";number_or_null(std::cout,matched.diagnostics.mean_distance_metres);
    std::cout<<",\"full_rank\":"<<(matched.diagnostics.full_rank?"true":"false")
             <<",\"map_fingerprint\":\""<<matched.diagnostics.source_fingerprint<<"\"}";
  }
  std::cout<<"]}\n";
}
} // namespace
int main(int argc,char** argv) {
  if(argc<4 || argc>9) {
    std::cerr<<"Usage: slam_ceres_replay_export <map.slammap> <log(.gz)> <scan-stride> [--beam-stride N] [--max-scans N] [--verified-fr079-holdout]\n";
    return 2;
  }
  try {
    const std::size_t stride=posint(argv[3],128);
    std::size_t beam_stride=4, max_scans=6000;
    bool heldout_only=false;
    for(int arg=4;arg<argc;){
      if(std::string_view(argv[arg])=="--verified-fr079-holdout") {
        if(heldout_only)throw std::invalid_argument("duplicate heldout option");
        heldout_only=true; ++arg; continue;
      }
      if(arg+1>=argc)throw std::invalid_argument("missing option value");
      if(std::string_view(argv[arg])=="--beam-stride")beam_stride=posint(argv[arg+1],64);
      else if(std::string_view(argv[arg])=="--max-scans")max_scans=posint(argv[arg+1],6000);
      else throw std::invalid_argument("unknown CLI option");
      arg+=2;
    }
    std::ifstream binary(argv[1],std::ios::binary);
    if(!binary)throw std::runtime_error("cannot open frozen map bundle");
    const auto map=slam::mapping::read_map_bundle(binary);
    const auto field=slam::mapping::DistanceField::build(map.snapshot,0.70);
    const auto raw=source_text(argv[2]);
    std::istringstream stream(raw);
    std::vector<slam::datasets::CarmenLaser> scans;
    slam::datasets::CarmenReader reader;
    std::string line;
    std::size_t lines=0;
    double last_time=-std::numeric_limits<double>::infinity();
    while(std::getline(stream,line)) {
      if(++lines>300000||line.size()>1024*1024)throw std::runtime_error("oversized CARMEN recording");
      if(!line.starts_with("FLASER "))continue;
      auto scan=reader.parse_laser(line);
      if(!scan)throw std::runtime_error("malformed CARMEN FLASER line");
      if(scan->ipc_seconds<last_time)throw std::runtime_error("nonmonotonic scan timestamps");
      last_time=scan->ipc_seconds;
      if(scans.size()>=max_scans)throw std::runtime_error("recording exceeds max scan count");
      scans.push_back(std::move(*scan));
    }
    if(scans.empty())throw std::runtime_error("no FLASER scans found");
    if(heldout_only) {
      // Verify the *actual* map contents were built from the exact CARMEN
      // recording with the same default train/holdout split. A filename or
      // source label by itself would NOT establish nonleakage.
      if(map.provenance.source_id!="fr079-training-index-mod5-mod6-v1" ||
         map.provenance.frame_id!="fr079_logged_laser_frame_UNVERIFIED" ||
         map.provenance.metric_origin_verified)
        throw std::invalid_argument("map is not the FR079 train-only frozen export");
      const auto rebuilt=slam::datasets::build_fr079_training_map(scans);
      if(rebuilt.snapshot->fingerprint()!=map.snapshot->fingerprint())
        throw std::invalid_argument("frozen map differs from training-only rebuild of this recording");
    }
    emit(scans,map,field,stride,beam_stride,heldout_only);
  } catch(const std::exception& error) {
    std::cerr<<"Ceres replay export failed: "<<error.what()<<'\n';return 1;
  }
}
