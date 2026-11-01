#include "apps/demo/native/fusion.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#ifdef __unix__
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace slam::viewer {
namespace {
std::string read_file(const std::filesystem::path& p,std::size_t max_bytes) {
#ifdef __unix__
  const int fd=::open(p.c_str(),O_RDONLY|O_CLOEXEC|O_NOFOLLOW);
  if(fd<0)throw std::invalid_argument("fusion artifact must be a regular non-symlink file");
  struct Closer {int fd;~Closer(){::close(fd);}} closer{fd};
  struct stat st{};
  if(::fstat(fd,&st)!=0||!S_ISREG(st.st_mode)||st.st_size<0||
     static_cast<std::uint64_t>(st.st_size)>max_bytes)
    throw std::invalid_argument("fusion artifact is not a bounded regular file");
  std::string result;result.reserve(static_cast<std::size_t>(st.st_size));
  std::array<char,8192> buf{};
  while(true) {
    const auto n=::read(fd,buf.data(),buf.size());
    if(n<0)throw std::runtime_error("failed reading fusion artifact");
    if(n==0)break;
    if(static_cast<std::size_t>(n)>max_bytes-result.size())
      throw std::invalid_argument("fusion artifact grew beyond maximum size");
    result.append(buf.data(),static_cast<std::size_t>(n));
  }
  return result;
#else
  if(!std::filesystem::is_regular_file(std::filesystem::symlink_status(p))||
     std::filesystem::file_size(p)>max_bytes)
    throw std::invalid_argument("fusion artifact must be a bounded regular file");
  std::ifstream in(p,std::ios::binary);if(!in)throw std::invalid_argument("missing fusion artifact");
  std::ostringstream buffer;buffer<<in.rdbuf();
  auto value=buffer.str();
  if(value.size()>max_bytes)throw std::invalid_argument("oversized fusion artifact");
  return value;
#endif
}
bool run_name(std::string_view s) {
  if(s.empty()||s.size()>80)return false;
  for(unsigned char c:s)if(!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||
                             (c>='0'&&c<='9')||c=='_'||c=='-'))return false;
  return true;
}
bool is_regular(const std::filesystem::path& path) {
  std::error_code ec;
  return std::filesystem::is_regular_file(std::filesystem::symlink_status(path,ec));
}
bool is_directory(const std::filesystem::path& path) {
  std::error_code ec;
  return std::filesystem::is_directory(std::filesystem::symlink_status(path,ec));
}
bool valid_scene(const std::string& scene) {
  return scene=="synthetic-apartment-v1"||scene=="synthetic-warehouse-v1";
}
std::size_t size_array(const Json& j,std::size_t bound,const char* what) {
  need(j.kind==Json::Kind::array && j.array.size()<=bound,what);
  return j.array.size();
}
double finite(const Json& value,double maximum=1'000'000.0) {
  const double v=value.real();
  need(std::abs(v)<=maximum,"fusion number outside physical bound");
  return v;
}
std::int64_t nanosecond(const Json& val) {
  const auto str=val.str();
  need(!str.empty()&&str.size()<=20&&
       (str.size()==1||str[0]!='0'),"invalid nanosecond timestamp");
  std::int64_t ns{};
  const auto [end,err]=std::from_chars(str.data(),str.data()+str.size(),ns);
  need(err==std::errc{}&&end==str.data()+str.size()&&ns>=0,
       "nanoseconds must be canonical nonnegative int64 strings");
  return ns;
}
std::array<double,3> checked_pose(const Json& j) {
  need(size_array(j,3,"invalid fusion pose") ==3,"fusion pose must be SE2");
  return {finite(j.at(std::size_t{0})),finite(j.at(std::size_t{1})),
          finite(j.at(std::size_t{2}),100)};
}
const std::array<std::string,5> methods={
  "wheel_only","lidar_only","gyro_wheel","fused_scan_matched","graph_optimized"};
} // namespace

FusionRuns::FusionRuns(std::filesystem::path root):root_(std::filesystem::canonical(root)) {
  need(is_directory(root_),"fusion run directory must exist");
}
bool FusionRuns::method_name(const std::string& method) {
  return std::find(methods.begin(),methods.end(),method)!=methods.end();
}
std::filesystem::path FusionRuns::directory(const std::string& run) const {
  need(run_name(run),"invalid fusion run identifier");
  const auto path=root_/run;
  need(is_directory(path),"missing or symlinked fusion run directory");
  return path;
}
FusionRuns::Session FusionRuns::load(const std::string& run) const {
  const auto path=directory(run);
  Session s{parse_json(read_file(path/"fusion_report.json",512*1024)),
            parse_json(read_file(path/"trajectories.json",2*1024*1024)),
            parse_json(read_file(path/"fusion_replay.json",16*1024*1024))};
  const auto& r=s.report;
  const auto& p=s.poses;
  const auto& t=s.trace;
  need(r.at("format").str()=="FUSFUSION1" &&
       r.at("source").str()=="moving_synthetic_lidar_gyro_wheel" &&
       !r.at("map_pose_truth_used").flag() &&
       !r.at("physical_gyro_calibration_verified").flag(),
       "unsupported fusion report provenance");
  const auto scene=r.at("scene").str();
  need(valid_scene(scene) && r.at("truth_class").str()=="synthetic_exact",
       "unsupported fusion scene/truth provenance");
  const std::size_t n=static_cast<std::size_t>(r.at("frames").uint());
  need(n>=2&&n<=10000,"invalid fusion run frame count");
  need(p.at("format").str()=="FUSPOSES1"&&p.at("frame_id").str()=="map"&&
       p.at("reference_class").str()=="synthetic_exact" &&
       p.at("angle_units").str()=="radians"&&p.at("length_units").str()=="metres" &&
       p.at("clock_domain").uint()==101 && p.at("clock_epoch").str()=="1",
       "invalid fusion pose clock/frame provenance");
  need(t.at("format").str()=="FUSFUSREPLAY1"&&
       t.at("source").str()==r.at("source").str()&&
       t.at("scene").str()==scene&&
       t.at("reference_class").str()=="synthetic_exact" &&
       t.at("matcher").str()==r.at("matcher").str()&&
       t.at("scan_frame").str()=="laser_midpoint_deskewed_full_se2"&&
       t.at("frame_count").uint()==n,
       "fusion replay does not match source report");
  const auto& result=r.at("results");
  const auto& paths=p.at("paths");
  need(result.kind==Json::Kind::object&&paths.kind==Json::Kind::object&&
       result.object.size()==paths.object.size()&&
       result.object.size()>=4&&result.object.size()<=5,
       "missing fusion baseline results");
  for(const auto& [method,entry]:paths.object) {
    need(method_name(method)&&result.has(method)&&
         size_array(entry,n,"fusion path exceeds frame bound")==n,
         "trajectory/method mismatch");
    for(const auto& pose:entry.array)(void)checked_pose(pose);
    const auto& map_report=result.at(method).at("map");
    need(map_report.at("format").str()=="FUSMAPACC1" &&
         map_report.at("reference_scene").str()==scene &&
         map_report.at("reference_class").str()=="synthetic_exact",
         "fusion method map truth mismatch");
    (void)map_report.at("map_fingerprint").uint();
  }
  for(const auto method:{"wheel_only","lidar_only","gyro_wheel","fused_scan_matched"})
    need(paths.has(method),"missing standard fusion baseline");
  const auto& stamps=p.at("timestamps_ns");
  const auto& frames=t.at("frames");
  need(size_array(stamps,n,"timestamp series too long")==n&&
       size_array(frames,n,"replay series too long")==n,
       "fusion timestamp and replay frame count mismatch");
  const auto& truth=p.at("synthetic_truth_for_scoring");
  need(size_array(truth,n,"invalid scoring-only synthetic truth")==n,
       "reference/trajectory count mismatch");
  for(const auto& pose:truth.array)(void)checked_pose(pose);
  const auto& lever=t.at("laser_from_base_translation_m");
  need(size_array(lever,2,"invalid laser mounting translation")==2,
       "missing laser extrinsics");
  for(const auto& value:lever.array)(void)finite(value,5.0);
  (void)finite(t.at("base_from_laser_yaw_rad"),10.0);
  std::int64_t prev=-1;
  std::size_t accepted=0,rejected=0,accepted_loops=0;
  for(std::size_t i=0;i<n;++i) {
    const std::int64_t now=nanosecond(stamps.at(i));
    need(now>prev,"nonmonotonic fusion timeline");prev=now;
    const auto& frame=frames.at(i);
    need(frame.at("index").uint()==i &&
         nanosecond(frame.at("time_ns"))==now,
         "fusion replay/timestamps index mismatch");
    const auto hits=frame.at("hits").uint();
    const auto beam_count=frame.at("beam_count").uint();
    need(beam_count>0&&beam_count<=4096&&hits<=beam_count,
         "bad replay LiDAR beam counts");
    const auto& points=frame.at("points_laser");
    need(size_array(points,720,"excessive replay scan points")<=hits,
         "rendered hit count exceeds physical hits");
    for(const auto& pt:points.array){
      need(size_array(pt,2,"invalid replay hit point")==2,"hit must be XY");
      for(const auto& coord:pt.array)(void)finite(coord,200.0);
    }
    const auto& m=frame.at("matching");
    const bool attempted=m.at("attempted").flag(),approved=m.at("accepted").flag();
    need(attempted==(i%2==1)&&(!approved||attempted),
         "frozen-map source-excluded query partition mismatched");
    const auto state=m.at("status").str();
    const auto reason=m.at("reason").str();
    need((!attempted&&state=="training"&&
          reason=="not_evaluated_training_scan")||
         (attempted&&approved&&state=="accepted"&&reason=="accepted")||
         (attempted&&!approved&&state=="rejected"&&
          (reason=="insufficient_support"||reason=="degenerate"||
           reason=="no_improvement"||reason=="invalid_input"||
           reason=="qualification_gate")),
         "matcher gate reason/status/acceptance mismatch");
    if(attempted){if(approved)++accepted;else ++rejected;}
    need(m.at("cumulative_accepted").uint()==accepted &&
         m.at("cumulative_rejected").uint()==rejected,
         "inconsistent cumulative match totals");
    if(attempted){
      const auto overlap=finite(m.at("overlap"),1.);
      need(overlap>=0&&m.at("supported_hits").uint()<=m.at("input_hits").uint()&&
           m.at("input_hits").uint()<=4096,"matcher geometry outside limits");
      const auto fingerprint=m.at("training_map_fingerprint").str();
      need(!fingerprint.empty()&&fingerprint.size()<=20&&
           (fingerprint.size()==1||fingerprint.front()!='0'),
           "malformed training-map fingerprint");
      std::uint64_t identity{};
      const auto [end,ec]=std::from_chars(fingerprint.data(),
                                         fingerprint.data()+fingerprint.size(),identity);
      need(ec==std::errc{}&&end==fingerprint.data()+fingerprint.size(),
           "noncanonical training-map fingerprint");
    }
    const auto& insertion=frame.at("mapping");
    need(insertion.at("gyro_wheel_inserted").flag()&&
         insertion.at("fused_scan_matched_inserted").flag(),
         "untrusted synthetic insertion summary");
    (void)insertion.at("lidar_only_inserted").flag();
    const auto& loops=frame.at("loop_events");
    need(size_array(loops,100,"too many loop events per frame")<=100,
         "loop event limit");
    for(const auto& event:loops.array){
      need(event.at("to").uint()==i&&event.at("from").uint()<i,
           "bad graph loop index");
      const auto decision=event.at("decision").str();
      need(decision=="accepted"||decision=="insufficient_geometry"||
           decision=="unverified_evidence"||decision=="inconsistent"||
           decision=="duplicate_or_nearby","invalid loop status");
      if(decision=="accepted")++accepted_loops;
    }
    need(frame.at("cumulative_accepted_loops").uint()==accepted_loops,
         "invalid cumulative accepted loop count");
  }
  need(accepted==r.at("heldout_accepted").uint() &&
       rejected==r.at("heldout_rejected").uint() &&
       accepted+rejected==r.at("heldout_attempted").uint() &&
       accepted_loops==r.at("graph_accepted_loops").uint() &&
       accepted_loops<=r.at("graph_loop_candidates").uint(),
       "replay events disagree with fusion report");
  return s;
}
Json FusionRuns::index() const {
  Json out=Json::dict(),runs=Json::list();
  std::size_t dirs=0;
  for(const auto& item:std::filesystem::directory_iterator(root_)) {
    if(!is_directory(item.path()))continue;
    if(++dirs>128)throw std::invalid_argument("too many fusion run directories");
    const std::string name=item.path().filename().string();
    if(!run_name(name)||
       !is_regular(item.path()/"fusion_report.json")||
       !is_regular(item.path()/"trajectories.json")||
       !is_regular(item.path()/"fusion_replay.json"))continue;
    Json entry=Json::dict();
    entry.object={{"id",Json::text(name)},{"name",Json::text(name)}};
    runs.array.push_back(std::move(entry));
  }
  std::sort(runs.array.begin(),runs.array.end(),[](const Json& a,const Json& b){
    return a.at("id").str()<b.at("id").str();
  });
  out.object={{"runs",runs},{"mode",Json::text("read-only-offline")},
              {"live_tracking",Json::boolean(false)}};
  return out;
}
Json FusionRuns::report(const std::string& run) const {return load(run).report;}
Json FusionRuns::trajectories(const std::string& run) const {return load(run).poses;}
Json FusionRuns::replay(const std::string& run) const {return load(run).trace;}
Json FusionRuns::frame(const std::string& run,std::size_t i) const {
  const auto s=load(run);
  need(i<s.trace.at("frames").array.size(),"fusion frame index out of bounds");
  return s.trace.at("frames").at(i);
}
Json FusionRuns::map_metadata(const std::string& run,const std::string& method) const {
  need(method_name(method),"unrecognized fusion method");
  const auto s=load(run);
  need(s.report.at("results").has(method),"method not present in this run");
  ViewerData viewer(directory(run),directory(run),false);
  const auto metadata=viewer.map_metadata(method+".slammap");
  const auto& r=s.report.at("results").at(method).at("map");
  need(metadata.at("fingerprint").str()==
       std::to_string(r.at("map_fingerprint").uint()) &&
       metadata.at("truth_class").str()=="synthetic_estimated" &&
       metadata.at("source_id").str()==s.report.at("scene").str() &&
       metadata.at("frame_id").str()=="map",
       "FUSMAP2D bundle does not match fusion report identity");
  return metadata;
}
std::string FusionRuns::map_png(const std::string& run,const std::string& method) const {
  (void)map_metadata(run,method); // verifies authoritative fingerprint before image
  const auto path=directory(run);
  return ViewerData(path,path,false).map_png(method+".slammap");
}
} // namespace slam::viewer
