#include "apps/demo/native/data.hpp"

#include <openssl/sha.h>
#include <zlib.h>

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>
#ifdef __unix__
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace slam::viewer {
namespace {
using namespace std::string_literals;
constexpr std::size_t kMaxEntries=512;
constexpr std::string_view kExample="sample-synthetic-workshop.slammap";
constexpr std::string_view kReplayExample="sample-synthetic-workshop.slamreplay.json";

std::string sha256(std::string_view bytes) {
  unsigned char hash[SHA256_DIGEST_LENGTH];
  if(!SHA256(reinterpret_cast<const unsigned char*>(bytes.data()),bytes.size(),hash))
    throw std::runtime_error("SHA-256 calculation failed");
  constexpr char digits[]="0123456789abcdef";
  std::string out;out.reserve(SHA256_DIGEST_LENGTH*2);
  for(unsigned char b:hash){out+=digits[b>>4];out+=digits[b&15];}
  return out;
}
Json string(std::string v) {return Json::text(std::move(v));}
Json num(std::uint64_t n) {return jint(n);}
bool clean(std::string_view filename,std::string_view suffix,std::size_t max_base) {
  if(filename.size()<=suffix.size()||!filename.ends_with(suffix)||
     filename.size()>max_base+suffix.size())return false;
  const auto base=filename.substr(0,filename.size()-suffix.size());
  if(base.empty()||base[0]<'0'||(base[0]>'9'&&base[0]<'A')||
     (base[0]>'Z'&&base[0]<'a')||base[0]>'z')return false;
  for(unsigned char c:base)
    if(!((c>='0'&&c<='9')||(c>='a'&&c<='z')||(c>='A'&&c<='Z')||
         c=='-'||c=='_'||c=='.'))return false;
  return true;
}
bool map_name(std::string_view n) {return clean(n,".slammap",181);}
bool replay_name(std::string_view n) {return clean(n,".slamreplay.json",141);}

std::string read_safe(const std::filesystem::path& path,std::size_t limit) {
#ifdef __unix__
  const int fd=::open(path.c_str(),O_RDONLY|O_CLOEXEC|O_NOFOLLOW);
  if(fd<0)throw std::invalid_argument("cannot open regular viewer file without symlink");
  struct Guard {int fd;~Guard(){::close(fd);}} guard{fd};
  struct stat st{};
  if(::fstat(fd,&st)!=0||!S_ISREG(st.st_mode)||st.st_size<0||
     static_cast<std::uint64_t>(st.st_size)>limit)
    throw std::invalid_argument("viewer file is not a bounded regular file");
  std::string out;out.reserve(static_cast<std::size_t>(st.st_size));
  std::array<char,8192> buffer;
  for(;;){
    const auto n=::read(fd,buffer.data(),buffer.size());
    if(n<0)throw std::runtime_error("viewer read failed");
    if(n==0)break;
    if(static_cast<std::size_t>(n)>limit-out.size())
      throw std::invalid_argument("viewer file grew beyond bound");
    out.append(buffer.data(),static_cast<std::size_t>(n));
  }
  return out;
#else
  if(!std::filesystem::is_regular_file(std::filesystem::symlink_status(path))||
     std::filesystem::file_size(path)>limit)
    throw std::invalid_argument("not a bounded regular file");
  std::ifstream in(path,std::ios::binary);
  if(!in)throw std::runtime_error("file open failed");
  std::ostringstream stream;stream<<in.rdbuf();
  const auto result=stream.str();
  if(result.size()>limit)throw std::invalid_argument("file grew beyond bound");
  return result;
#endif
}
bool safe_exists(const std::filesystem::path& p) {
  std::error_code ec;
  return std::filesystem::is_regular_file(std::filesystem::symlink_status(p,ec));
}
std::vector<std::string> names(const std::filesystem::path& root,
                               bool(*valid)(std::string_view)) {
  std::vector<std::string> list;
  for(const auto& f:std::filesystem::directory_iterator(root)){
    const auto name=f.path().filename().string();
    if(valid(name)&&safe_exists(f.path()))list.push_back(name);
    if(list.size()>kMaxEntries)throw std::invalid_argument("too many files in viewer directory");
  }
  std::sort(list.begin(),list.end());
  return list;
}
bool text_equal(const Json& j,std::string_view x) {return j.kind==Json::Kind::string&&j.scalar==x;}
double real(const Json& j) {return j.real();}
std::uint64_t number(const Json& j) {return j.uint();}
std::array<double,3> pose(const Json& v) {
  need(v.kind==Json::Kind::array&&v.array.size()==3,"invalid scan pose");
  std::array<double,3> p{};
  for(int i=0;i<3;++i){p[static_cast<std::size_t>(i)]=real(v.at(static_cast<std::size_t>(i)));
    need(std::abs(p[static_cast<std::size_t>(i)])<=1'000'000.0,"pose outside limit");}
  return p;
}
void validate_replay(const Json& j) {
  need(keys(j,{"format","source","frames"})&&text_equal(j.at("format"),"FUSREPLAY1"),
       "invalid replay format");
  const auto& source=j.at("source");
  need(source.kind==Json::Kind::object,"invalid replay source");
  for(const auto field:{"dataset","frame_id","pose_kind","timestamp_kind","source_sha256"}) {
    const auto s=source.at(field).str();
    need(!s.empty()&&s.size()<=256,"missing replay provenance");
  }
  const auto kind=source.at("pose_kind").str();
  need(kind=="logged_laser_estimate_not_ground_truth"||kind=="synthetic_exact",
       "unrecognized replay pose provenance");
  const auto clock=source.at("timestamp_kind").str();
  need(clock=="CARMEN_IPC_scan_level"||clock=="synthetic_scan_level",
       "unrecognized replay clock provenance");
  need(!source.at("per_beam_time_known").flag() && !source.at("map_alignment_verified").flag(),
       "unverified replay cannot claim timing or alignment");
  const auto source_sha=source.at("source_sha256").str();
  need(source_sha.size()==64&&
       std::all_of(source_sha.begin(),source_sha.end(),[](char c){
         return (c>='0'&&c<='9')||(c>='a'&&c<='f');}),"invalid replay SHA");
  const auto& frames=j.at("frames");
  need(frames.kind==Json::Kind::array&&
       !frames.array.empty()&&frames.array.size()<=6000,"invalid replay frame count");
  double last=-std::numeric_limits<double>::infinity();
  for(std::size_t i=0;i<frames.array.size();++i){
    const auto& f=frames.at(i);
    need(keys(f,{"index","t","pose","odom","beam_count","hit_count","points","matching"}),
         "invalid replay frame schema");
    need(number(f.at("index"))==i,"nonsequential replay index");
    const double t=real(f.at("t"));
    need(t>=last,"nonmonotonic replay timestamp");last=t;
    (void)pose(f.at("pose"));(void)pose(f.at("odom"));
    const auto n=number(f.at("beam_count")),hit=number(f.at("hit_count"));
    need(n>=1&&n<=4096&&hit<=n,"invalid replay beam counts");
    const auto& points=f.at("points");
    need(points.kind==Json::Kind::array&&points.array.size()<=720&&
         points.array.size()<=hit,"invalid replay point count");
    for(const auto& p:points.array){
      need(p.kind==Json::Kind::array&&p.array.size()==2,"invalid replay point");
      (void)real(p.at(std::size_t{0}));(void)real(p.at(std::size_t{1}));
    }
    need(f.at("matching").kind==Json::Kind::null,"source replay may not contain synthesized matches");
  }
}
std::uint64_t fnv(std::string_view bytes) {
  std::uint64_t h=14695981039346656037ULL;
  for(unsigned char c:bytes){h^=c;h*=1099511628211ULL;}
  return h;
}
std::uint64_t parse_u64(std::string_view s) {
  need(!s.empty()&&(s.size()==1||s.front()!='0'),"noncanonical registry number");
  std::uint64_t n{};
  const auto [end,err]=std::from_chars(s.data(),s.data()+s.size(),n);
  need(err==std::errc{}&&end==s.data()+s.size(),"invalid registry number");
  return n;
}
void put_be32(std::string& out,std::uint32_t n) {
  for(int i=3;i>=0;--i)out+=static_cast<char>(n>>(8*i));
}
void png_chunk(std::string& out,const char* kind,std::string_view data){
  put_be32(out,static_cast<std::uint32_t>(data.size()));
  out.append(kind,4);out.append(data);
  auto crc=::crc32(0,reinterpret_cast<const Bytef*>(out.data()+out.size()-data.size()-4),
                  static_cast<uInt>(data.size()+4));
  put_be32(out,static_cast<std::uint32_t>(crc));
}
std::string png(const mapping::MapSnapshot& snapshot) {
  const auto& g=snapshot.config();
  const auto step=std::max<std::size_t>(1,(std::max(g.width,g.height)+1799)/1800);
  const auto w=(g.width+step-1)/step,h=(g.height+step-1)/step;
  std::string rgba;rgba.reserve(h*(w*4+1));
  for(std::size_t y=0;y<h;++y){
    rgba+='\0';
    const auto iy=g.height-1-std::min(y*step,g.height-1);
    for(std::size_t x=0;x<w;++x){
      const auto ix=std::min(x*step,g.width-1);
      const auto& c=snapshot.cells()[iy*g.width+ix];
      const double p=1.0/(1.0+std::exp(-c.log_odds));
      const std::array<unsigned char,4> rgba_pixel=
          !c.observed?std::array<unsigned char,4>{21,34,54,255}:
          p>=.65?std::array<unsigned char,4>{70,211,184,255}:
          p<=.35?std::array<unsigned char,4>{225,237,241,255}:
                 std::array<unsigned char,4>{131,155,175,255};
      for(auto channel:rgba_pixel)rgba+=static_cast<char>(channel);
    }
  }
  uLongf compressed_bound=compressBound(static_cast<uLong>(rgba.size()));
  std::string compressed(compressed_bound,'\0');
  if(::compress2(reinterpret_cast<Bytef*>(compressed.data()),&compressed_bound,
      reinterpret_cast<const Bytef*>(rgba.data()),static_cast<uLong>(rgba.size()),6)!=Z_OK)
    throw std::runtime_error("PNG zlib compression failed");
  compressed.resize(static_cast<std::size_t>(compressed_bound));
  std::string output="\x89PNG\r\n\x1a\n";
  std::string header;
  put_be32(header,static_cast<std::uint32_t>(w));
  put_be32(header,static_cast<std::uint32_t>(h));
  header.append("\x08\x06\x00\x00\x00",5);
  png_chunk(output,"IHDR",header);png_chunk(output,"IDAT",compressed);
  png_chunk(output,"IEND",{});
  return output;
}
Json map_info(const mapping::MapBundle& b) {
  const auto& s=*b.snapshot;const auto& g=s.config();
  std::size_t known=0,occupied=0,free=0;
  for(const auto& c:s.cells()){
    if(c.observed){
      ++known;const double p=1.0/(1.0+std::exp(-c.log_odds));
      if(p>=.65)++occupied;
      else if(p<=.35)++free;
    }
  }
  Json j=Json::dict(),origin=Json::dict();
  origin.object={{"x",jreal(g.map_from_grid.translation.x())},
                 {"y",jreal(g.map_from_grid.translation.y())},
                 {"yaw_rad",jreal(g.map_from_grid.yaw_rad)}};
  j.object={{"format",string("FUSMAP2D v1")},
       {"width",num(g.width)},{"height",num(g.height)},
       {"resolution_m",jreal(g.resolution_m)},{"origin",origin},
       {"frame_id",string(b.provenance.frame_id)},
       {"source_id",string(b.provenance.source_id)},
       {"truth_class",string(b.provenance.truth_class)},
       {"metric_origin_verified",Json::boolean(b.provenance.metric_origin_verified)},
       {"revision",string(std::to_string(s.revision()))},
       {"fingerprint",string(std::to_string(s.fingerprint()))},
       {"known",num(known)},{"unknown",num(s.cells().size()-known)},
       {"occupied",num(occupied)},{"free",num(free)},
       {"neutral_known",num(known-occupied-free)},
       {"clear_trusted_no_returns",Json::boolean(g.clear_trusted_no_returns)},
       {"integrity",string("verified-corruption-only")}};
  return j;
}
mapping::MapBundle sample_map() {
  mapping::GridConfig cfg;
  cfg.width=120;cfg.height=90;cfg.resolution_m=.1;
  std::vector<mapping::Cell> cells;cells.reserve(cfg.width*cfg.height);
  for(std::size_t y=0;y<cfg.height;++y)for(std::size_t x=0;x<cfg.width;++x){
    const double px=(x+.5)*.1,py=(y+.5)*.1;
    if(px<.5||px>11.5||py<.5||py>8.5||(px>10.1&&py>6)||(px<1.8&&py>6.1)){
      cells.push_back({0,false});continue;
    }
    const auto inside=[](double a,double b,double c,double d,
                         double x0,double y0){return a<=x0&&x0<=b&&c<=y0&&y0<=d;};
    const bool wall=px<.7||px>11.3||py<.7||py>8.3;
    const bool box=inside(3.05,4.25,1.85,4.4,px,py)||
      inside(6.65,8.15,4,5.35,px,py)||
      inside(8.9,10.15,1.85,3.7,px,py)||
      inside(5.5,6,6.6,8.1,px,py)||
      inside(1.25,2.15,6.55,7.8,px,py);
    cells.push_back({wall||box?2.5:-2.0,true});
  }
  return {mapping::MapSnapshot::from_cells(cfg,1,std::move(cells)),
    {"synthetic_demo_frame","schematic_virtual_workshop","synthetic_exact",false}};
}
Json sample_replay() {
  Json root=Json::dict(),source=Json::dict(),frames=Json::list();
  source.object={{"dataset",string("SYNTHETIC virtual workshop (not recorded LiDAR)")},
    {"frame_id",string("synthetic_demo_frame")},
    {"pose_kind",string("synthetic_exact")},
    {"timestamp_kind",string("synthetic_scan_level")},
    {"per_beam_time_known",Json::boolean(false)},
    {"map_alignment_verified",Json::boolean(false)},
    {"source_sha256",string(sha256("synthetic virtual workshop replay v1"))}};
  for(std::size_t i=0;i<56;++i){
    const double angle=2.0*std::numbers::pi*static_cast<double>(i)/56;
    const double x=6+2.25*std::cos(angle),y=4.5+1.55*std::sin(angle);
    const double yaw=std::atan2(1.55*std::cos(angle),-2.25*std::sin(angle));
    Json p=Json::list(),points=Json::list(),f=Json::dict();
    p.array={jreal(x),jreal(y),jreal(yaw)};
    for(std::size_t k=0;k<64;++k){
      const double ray=-std::numbers::pi/2+std::numbers::pi*static_cast<double>(k)/64;
      const double a=yaw+ray,dx=std::cos(a),dy=std::sin(a);
      double distance=5;
      for(const auto wall: {0.7,11.3}){
        if(std::abs(dx)>1e-10){
          const auto t=(wall-x)/dx;
          if(t>0&&y+t*dy>=.7&&y+t*dy<=8.3)distance=std::min(distance,t);
        }
      }
      for(const auto wall: {0.7,8.3}){
        if(std::abs(dy)>1e-10){
          const auto t=(wall-y)/dy;
          if(t>0&&x+t*dx>=.7&&x+t*dx<=11.3)distance=std::min(distance,t);
        }
      }
      Json hit=Json::list();hit.array={jreal(distance*std::cos(ray)),jreal(distance*std::sin(ray))};
      points.array.push_back(std::move(hit));
    }
    f.object={{"index",num(i)},{"t",jreal(i*.12)},{"pose",p},{"odom",p},
      {"beam_count",num(64)},{"hit_count",num(64)},{"points",points},{"matching",Json{}}};
    frames.array.push_back(std::move(f));
  }
  root.object={{"format",string("FUSREPLAY1")},{"source",source},{"frames",frames}};
  validate_replay(root);
  return root;
}
Json error_ceres(std::string message) {
  Json r=Json::dict();r.object={{"status",string("invalid")},{"processed",num(0)},
    {"error",string("Ceres results refused: "+message)}};return r;
}
Json no_ceres() {
  Json r=Json::dict();r.object={{"status",string("absent")},{"processed",num(0)}};return r;
}
Json summary(const Json& side) {
  Json out=Json::dict();
  if(!side.has("selection")||side.at("selection").str()!="fr079_heldout_mod5_verified"){
    out.object={{"mode",string("unrestricted")},{"training_split_checked",Json::boolean(false)}};
    return out;
  }
  const auto& rows=side.at("frames").array;
  Json counts=Json::dict();
  for(const auto kind:{"accepted","degenerate","insufficient_support","invalid_input","no_improvement"})
    counts.object.emplace(kind,num(0));
  double initial=0,final=0,overlap=0,distance=0;
  std::size_t valid_distance=0,accepted=0;
  for(const auto& row:rows){
    const auto status=row.at("status").str();
    counts.object[status]=num(counts.at(status).uint()+1);
    if(status=="accepted")++accepted;
    initial+=row.at("initial_robust_cost").real();
    final+=row.at("robust_cost").real();overlap+=row.at("overlap").real();
    if(row.at("mean_distance_m").kind!=Json::Kind::null){
      distance+=row.at("mean_distance_m").real();++valid_distance;
    }
  }
  out.object={{"mode",string("fr079_heldout_mod5")},
    {"training_split_checked",Json::boolean(true)},
    {"excluded_from_training",Json::boolean(true)},
    {"ground_truth_available",Json::boolean(false)},
    {"per_beam_time_verified",Json::boolean(false)},
    {"map_alignment_verified",Json::boolean(false)},
    {"query_stride",side.at("query_stride")},
    {"beam_stride",side.at("beam_stride")},
    {"evaluated_scans",num(rows.size())},{"accepted",num(accepted)},
    {"acceptance_fraction",rows.empty()?Json{}:jreal(double(accepted)/double(rows.size()))},
    {"status_counts",counts},
    {"mean_initial_robust_cost",rows.empty()?Json{}:jreal(initial/rows.size())},
    {"mean_final_robust_cost",rows.empty()?Json{}:jreal(final/rows.size())},
    {"mean_overlap",rows.empty()?Json{}:jreal(overlap/rows.size())},
    {"mean_edt_distance_m",valid_distance?jreal(distance/valid_distance):Json{}},
    {"quality_metric",string("scan-to-frozen-map EDT geometry; no position error")}};
  return out;
}
} // namespace

ViewerData::ViewerData(std::filesystem::path maps,std::filesystem::path replays,
                       bool example_if_empty)
:maps_(std::filesystem::canonical(std::move(maps))),
 replays_(std::filesystem::canonical(std::move(replays))) {
  need(std::filesystem::is_directory(maps_)&&std::filesystem::is_directory(replays_),
       "viewer map/replay directory must exist");
  if(example_if_empty && maps_==replays_){
    bool any=false;
    for(const auto& p:std::filesystem::directory_iterator(maps_)){
      const auto n=p.path().filename().string();
      if(n=="CURRENT"||n.ends_with(".slammap")||n.ends_with(".slamreplay.json")){
        any=true;break;
      }
    }
    if(!any){synthetic_map_=sample_map();synthetic_replay_=sample_replay();}
  }
}
mapping::MapBundle ViewerData::map(const std::string& name) const {
  need(map_name(name),"invalid map filename");
  if(synthetic_map_ && name==kExample)return *synthetic_map_;
  const auto bytes=read_safe(maps_/name,150*1024*1024);
  std::istringstream stream(bytes,std::ios::binary);
  return mapping::read_map_bundle(stream);
}
Json ViewerData::registry() const {
  const auto file=maps_/"CURRENT";
  if(!std::filesystem::exists(std::filesystem::symlink_status(file))){
    Json j=Json::dict();j.object={{"status",string("absent")},{"active",Json{}},{"previous",Json{}}};
    return j;
  }
  try {
    const auto body=read_safe(file,256);
    std::vector<std::string> lines;std::size_t start=0;
    for(std::size_t i=0;i<body.size();++i)if(body[i]=='\n'){
      lines.push_back(body.substr(start,i-start));start=i+1;
    }
    need(start==body.size()&&lines.size()==4&&lines[0]=="FUSGEN1",
         "malformed FUSGEN1 registry");
    const std::string prefix=lines[0]+"\n"+lines[1]+"\n"+lines[2]+"\n";
    need(fnv(prefix)==parse_u64(lines[3]),"FUSGEN1 checksum mismatch");
    auto parse_ptr=[](std::string_view line) -> Json {
      const auto p=line.find(' ');
      need(p!=std::string_view::npos&&line.find(' ',p+1)==std::string_view::npos,
           "invalid generation pointer");
      const auto id=parse_u64(line.substr(0,p)),fp=parse_u64(line.substr(p+1));
      if(id==0){need(fp==0,"zero generation with fingerprint");return {};}
      Json r=Json::dict();r.object={{"id",string(std::to_string(id))},
        {"fingerprint",string(std::to_string(fp))},
        {"file",string("generation-"+std::to_string(id)+".slammap")}};
      return r;
    };
    Json active=parse_ptr(lines[1]),previous=parse_ptr(lines[2]);
    need(!(active.kind==Json::Kind::null&&previous.kind!=Json::Kind::null),
         "orphan previous pointer");
    if(active.kind!=Json::Kind::null&&previous.kind!=Json::Kind::null)
      need(active.at("id").str()!=previous.at("id").str(),
           "duplicate generation pointer");
    Json result=Json::dict();
    result.object={{"status",string("present")},{"active",active},{"previous",previous}};
    for(const auto role:{"active","previous"})if(result.at(role).kind!=Json::Kind::null){
      try{
        auto& ptr=result.object.at(role);
        const auto b=map(ptr.at("file").str());
        need(std::to_string(b.snapshot->fingerprint())==ptr.at("fingerprint").str(),
             "generation fingerprint differs from disk");
        ptr.object["integrity"]=string("valid");
      }catch(const std::exception& e){
        result.object["status"]=string("degraded");
        result.object["error"]=string(std::string(role)+" generation invalid: "+e.what()+
            ". Recover only through the native GenerationStore API.");
        result.object.at(role).object["integrity"]=string("invalid");
      }
    }
    return result;
  }catch(const std::exception& e){
    Json r=Json::dict();r.object={{"status",string("invalid")},{"active",Json{}},
      {"previous",Json{}},{"error",string(e.what())}};
    return r;
  }
}
Json ViewerData::list_maps() const {
  const auto registry_value=registry();
  auto files=names(maps_,map_name);
  if(synthetic_map_)files.push_back(std::string(kExample));
  Json maps=Json::list();
  for(const auto& name:files){
    std::string role="file";
    for(const auto pointer:{"active","previous"}){
      const auto& j=registry_value.at(pointer);
      if(j.kind==Json::Kind::object&&j.at("file").str()==name)role=pointer;
    }
    Json item=Json::dict();
    item.object={{"id",string(name)},{"name",string(name)},{"role",string(role)}};
    maps.array.push_back(std::move(item));
  }
  Json out=Json::dict();out.object={{"maps",maps},{"registry",registry_value},
    {"mode",string("read-only-offline")},{"live_tracking",Json::boolean(false)},
    {"publication",Json::boolean(false)}};
  return out;
}
Json ViewerData::map_metadata(const std::string& name) const {return map_info(map(name));}
Json ViewerData::map_cell(const std::string& name,std::size_t x,std::size_t y) const {
  const auto m=map(name);
  const auto& g=m.snapshot->config();
  need(x<g.width&&y<g.height,"cell outside map");
  const auto c=m.snapshot->cell(x,y).value();
  const auto local=Eigen::Vector2d{(x+.5)*g.resolution_m,(y+.5)*g.resolution_m};
  const auto world=slam::transform(g.map_from_grid,local);
  Json j=Json::dict();
  j.object={{"x",num(x)},{"y",num(y)},
    {"world_x",jreal(world.x())},{"world_y",jreal(world.y())},
    {"observed",Json::boolean(c.observed)},
    {"log_odds",c.observed?jreal(c.log_odds):Json{}},
    {"occupied_probability",c.observed?jreal(1.0/(1.0+std::exp(-c.log_odds))):Json{}}};
  return j;
}
std::string ViewerData::map_png(const std::string& name) const {
  const auto m=map(name);return png(*m.snapshot);
}
Json ViewerData::list_replays() const {
  auto files=names(replays_,replay_name);
  if(synthetic_replay_)files.push_back(std::string(kReplayExample));
  Json sessions=Json::list();
  for(const auto& f:files)sessions.array.push_back(string(f));
  Json out=Json::dict();out.object={{"sessions",sessions},
    {"read_only",Json::boolean(true)},{"live_tracking",Json::boolean(false)}};
  return out;
}
ViewerData::ReplaySession ViewerData::replay(const std::string& name) const {
  need(replay_name(name),"invalid replay filename");
  const bool synthetic=synthetic_replay_&&name==kReplayExample;
  const std::string bytes=synthetic?"":read_safe(replays_/name,32*1024*1024);
  Json session=synthetic?*synthetic_replay_:parse_json(bytes);
  validate_replay(session);
  Json info=no_ceres();
  std::map<std::size_t,Json> accepted_rows;
  if(!synthetic && std::filesystem::exists(std::filesystem::symlink_status(
        replays_/(name+".ceres.json")))) {
    try{
      const auto side_text=read_safe(replays_/(name+".ceres.json"),4*1024*1024);
      const Json side=parse_json(side_text);
      // The existing Ceres file is visualization ONLY. Never assume its
      // producer has verified registration or independent reference truth.
      const bool legacy=keys(side,{"format","matcher","source_sha256","replay_sha256",
        "map_name","map_fingerprint","map_frame_id","map_revision",
        "map_origin_verified","independent_ground_truth",
        "training_independence_verified","per_beam_time_verified","frames"});
      const bool selected=keys(side,{"format","matcher","source_sha256","replay_sha256",
        "map_name","map_fingerprint","map_frame_id","map_revision",
        "map_origin_verified","independent_ground_truth",
        "training_independence_verified","per_beam_time_verified","frames",
        "selection","query_stride","beam_stride"});
      need(legacy||selected,"invalid Ceres sidecar schema");
      need(text_equal(side.at("format"),"FUSMATCH1")&&
           text_equal(side.at("matcher"),"ceres_scan_to_edt"),
           "invalid Ceres matching format");
      need(side.at("source_sha256").str()==session.at("source").at("source_sha256").str()&&
           side.at("replay_sha256").str()==sha256(bytes),"Ceres source/replay digest mismatch");
      for(const auto f:{"independent_ground_truth","training_independence_verified",
                        "per_beam_time_verified"})
        need(!side.at(f).flag(),"unjustified Ceres source claim");
      const std::string mapname=side.at("map_name").str();
      const auto m=map(mapname),metadata=map_info(m);
      need(side.at("map_fingerprint").uint()==m.snapshot->fingerprint()&&
           side.at("map_revision").uint()==m.snapshot->revision()&&
           side.at("map_origin_verified").flag()==m.provenance.metric_origin_verified&&
           side.at("map_frame_id").str()==m.provenance.frame_id&&
           side.at("map_frame_id").str()==session.at("source").at("frame_id").str(),
           "stale/different Ceres map fingerprint or frame");
      const std::string selection=selected?side.at("selection").str():"sampled_unrestricted";
      need(selection=="sampled_unrestricted"||
           selection=="fr079_heldout_mod5_verified","invalid selection");
      if(selection=="fr079_heldout_mod5_verified")
        need(m.provenance.source_id=="fr079-training-index-mod5-mod6-v1"&&
             m.provenance.frame_id=="fr079_logged_laser_frame_UNVERIFIED"&&
             !m.provenance.metric_origin_verified,"not the verified FR079 training-map source");
      const std::size_t stride=selected?side.at("query_stride").uint():0;
      if(selected)need(stride>=1&&stride<=128&&side.at("beam_stride").uint()>=1&&
                       side.at("beam_stride").uint()<=64,"invalid Ceres strides");
      const auto& rows=side.at("frames");
      const auto& frames=session.at("frames").array;
      need(rows.kind==Json::Kind::array&&rows.array.size()<=frames.size(),"invalid Ceres rows");
      if(selected){
        const auto real_stride=selection=="fr079_heldout_mod5_verified"?5*stride:stride;
        std::size_t ix=0;
        for(const auto& row:rows.array) {
          need(row.at("index").uint()==ix,"sidecar scan selection/coverage mismatch");
          ix+=real_stride;
        }
        need(ix>=frames.size()&&(!rows.array.empty()||frames.empty()),
             "incomplete Ceres scan selection");
      }
      std::size_t prev=0;bool first=true,accepted=0; 
      for(const auto& row:rows.array){
        need(keys(row,{"index","t","seed","estimate","status","iterations",
           "input_hits","supported_hits","overlap","initial_robust_cost",
           "robust_cost","mean_distance_m","full_rank","map_fingerprint"}),
           "invalid Ceres frame schema");
        const auto i=row.at("index").uint();
        need(i<frames.size()&&(first||i>prev),"Ceres index order invalid");
        first=false;prev=static_cast<std::size_t>(i);
        const auto& frame=frames.at(static_cast<std::size_t>(i));
        need(std::abs(row.at("t").real()-frame.at("t").real())<=1e-6,
             "Ceres row timestamp mismatch");
        const auto seed=pose(row.at("seed")),estimate=pose(row.at("estimate"));
        const auto logged=pose(frame.at("pose"));
        for(std::size_t k=0;k<3;++k)need(std::abs(seed[k]-logged[k])<=1e-6,
                                        "Ceres seed differs from source pose");
        const auto status=row.at("status").str();
        need(status=="accepted"||status=="no_improvement"||
             status=="insufficient_support"||status=="degenerate"||
             status=="invalid_input","unrecognized Ceres match status");
        need(row.at("map_fingerprint").uint()==m.snapshot->fingerprint(),
             "Ceres row map fingerprint mismatch");
        need(row.at("iterations").uint()<=1000,"Ceres iteration bound");
        const auto hits=row.at("input_hits").uint(),support=row.at("supported_hits").uint();
        need(hits<=4096&&support<=hits,"Ceres hit count mismatch");
        const auto overlap=row.at("overlap").real();
        need(overlap>=0&&overlap<=1&&
             std::abs(overlap-(hits?double(support)/double(hits):0.))<=1e-9,
             "Ceres overlap inconsistent with count");
        const double initial=row.at("initial_robust_cost").real(),
                     final=row.at("robust_cost").real();
        need(initial>=0&&final>=0,"negative Ceres cost");
        if(row.at("mean_distance_m").kind!=Json::Kind::null)
          need(row.at("mean_distance_m").real()>=0,"invalid Ceres mean EDT");
        const bool rank=row.at("full_rank").flag();
        if(status=="accepted"){
          need(rank&&support>=12&&overlap>=.65&&final<initial,
               "accepted Ceres row fails geometry contract");
          ++accepted;
        } else for(std::size_t k=0;k<3;++k)
          need(std::abs(estimate[k]-seed[k])<=1e-6,"rejected Ceres row changed seed");
        accepted_rows.emplace(static_cast<std::size_t>(i),row);
      }
      info=Json::dict();info.object={{"status",string("valid")},
        {"processed",num(rows.array.size())},{"accepted",num(accepted)},
        {"map_name",string(mapname)},
        {"map_fingerprint",num(m.snapshot->fingerprint())},
        {"map_frame_id",string(m.provenance.frame_id)},
        {"independent_ground_truth",Json::boolean(false)},
        {"training_independence_verified",Json::boolean(false)},
        {"evaluation",summary(side)}};
    }catch(const std::exception& e){info=error_ceres(e.what());accepted_rows.clear();}
  }
  return {std::move(session),std::move(info),std::move(accepted_rows)};
}
Json ViewerData::replay_info(const std::string& name) const {
  const auto r=replay(name);const auto& frames=r.data.at("frames").array;
  Json trajectory=Json::list();
  for(const auto& f:frames){
    Json entry=Json::list();entry.array.push_back(f.at("t"));
    for(const auto& v:f.at("pose").array)entry.array.push_back(v);
    trajectory.array.push_back(std::move(entry));
  }
  Json info=Json::dict();info.object={{"name",string(name)},
    {"format",r.data.at("format")},{"source",r.data.at("source")},
    {"ceres",r.match_info},{"frame_count",num(frames.size())},
    {"first_t",frames.front().at("t")},{"last_t",frames.back().at("t")},
    {"trajectory",trajectory}};
  return info;
}
Json ViewerData::replay_frame(const std::string& name,std::size_t index) const {
  const auto r=replay(name);
  need(index<r.data.at("frames").array.size(),"replay frame out of range");
  Json frame=r.data.at("frames").at(index);
  auto found=r.matches.find(index);
  frame.object["matching"]=found==r.matches.end()?Json{}:found->second;
  frame.object["matching_state"]=r.match_info.at("status");
  return frame;
}
} // namespace slam::viewer
