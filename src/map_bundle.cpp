#include "slam/mapping/map_bundle.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <istream>
#include <ostream>
#include <utility>
#include <iomanip>
#include <locale>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace slam::mapping {
namespace {
constexpr std::array<std::uint8_t,8> kMagic{'F','U','S','M','A','P','2','D'};
constexpr std::size_t kMaxCells=16'000'000;
constexpr std::size_t kMaxBytes=150*1024*1024;
constexpr std::size_t kMaxLabel=256;
constexpr std::uint32_t kVersion=1;
constexpr std::uint64_t kOffset=14695981039346656037ULL;
constexpr std::uint64_t kPrime=1099511628211ULL;

void fail(const char* why) { throw std::invalid_argument(why); }
void label_valid(const std::string& value) {
  if(value.empty()||value.size()>kMaxLabel)fail("map provenance label length invalid");
  for (unsigned char c: value)
    if (!((c>='A'&&c<='Z')||(c>='a'&&c<='z')||(c>='0'&&c<='9')||
          c=='_'||c=='-'||c=='.'||c==':'||c=='/'))
      fail("map provenance labels require ASCII alphanumeric / . _ : -");
}
void validate_snapshot(const GridConfig& cfg, const std::vector<Cell>& cells) {
  if(!cfg.width||!cfg.height||cfg.width>kMaxCells||cfg.height>kMaxCells/cfg.width||
     cfg.width*cfg.height!=cells.size()||!std::isfinite(cfg.resolution_m)||cfg.resolution_m<=0||
     !std::isfinite(static_cast<double>(cfg.width)*cfg.resolution_m)||
     !std::isfinite(static_cast<double>(cfg.height)*cfg.resolution_m)||
     !std::isfinite(cfg.map_from_grid.yaw_rad)||!cfg.map_from_grid.translation.allFinite()||
     !std::isfinite(cfg.hit_log_odds)||cfg.hit_log_odds<=0||
     !std::isfinite(cfg.free_log_odds)||cfg.free_log_odds>=0||
     !std::isfinite(cfg.min_log_odds)||!std::isfinite(cfg.max_log_odds)||
     cfg.min_log_odds>=0||cfg.max_log_odds<=0||cfg.min_log_odds>=cfg.max_log_odds)
    fail("invalid authoritative map configuration");
  for(const auto& c: cells) {
    if(!std::isfinite(c.log_odds) || c.log_odds<cfg.min_log_odds ||
       c.log_odds>cfg.max_log_odds || (!c.observed && c.log_odds!=0.0))
      fail("invalid known/unknown log odds cell");
  }
}

void add_u(std::vector<std::uint8_t>& buf, std::uint64_t value, int n) {
  for(int i=0;i<n;++i) buf.push_back(static_cast<std::uint8_t>(value>>(8*i)));
}
void add_f64(std::vector<std::uint8_t>& buf, double x) {
  add_u(buf,std::bit_cast<std::uint64_t>(x),8);
}
std::uint64_t fnv(const std::uint8_t* bytes,std::size_t length) {
  std::uint64_t h=kOffset;
  for(std::size_t i=0;i<length;++i) {h^=bytes[i];h*=kPrime;}
  return h;
}
class Reader {
public:
  explicit Reader(const std::vector<std::uint8_t>& data): data_(data) {}
  std::uint64_t u(int n) {
    if(n<1||n>8||remaining()<static_cast<std::size_t>(n))fail("truncated map bundle");
    std::uint64_t result=0;
    for(int i=0;i<n;++i) result|=static_cast<std::uint64_t>(data_[position_++])<<(i*8);
    return result;
  }
  double f64() {return std::bit_cast<double>(u(8));}
  std::string label() {
    const auto n=u(4);
    if(n==0||n>kMaxLabel||remaining()<n) fail("malformed map provenance label");
    std::string out;
    out.reserve(static_cast<std::size_t>(n));
    for(std::size_t i=0;i<n;++i)out.push_back(static_cast<char>(u(1)));
    label_valid(out);
    return out;
  }
  std::size_t remaining()const{return data_.size()-position_;}
  std::size_t position()const{return position_;}
private:
  const std::vector<std::uint8_t>& data_;
  std::size_t position_{};
};
void add_label(std::vector<std::uint8_t>& buf,const std::string& name) {
  label_valid(name);
  add_u(buf,name.size(),4);
  buf.insert(buf.end(),name.begin(),name.end());
}
std::string clean_basename(const std::string& name) {
  if(name.empty()||name.size()>128||name=="."||name=="..")fail("invalid PGM basename");
  for(unsigned char c: name)
    if(!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='-'||c=='_'||c=='.'))
      fail("PGM basename must contain only alphanumerics, dashes, underscores or dots");
  return name;
}
}

void write_map_bundle(std::ostream& out,const MapBundle& bundle) {
  if(!bundle.snapshot)fail("missing map snapshot");
  const auto& cfg=bundle.snapshot->config();
  const auto& cells=bundle.snapshot->cells();
  validate_snapshot(cfg,cells);
  const auto& meta=bundle.provenance;
  label_valid(meta.frame_id);label_valid(meta.source_id);label_valid(meta.truth_class);
  std::vector<std::uint8_t> data;
  data.reserve(200+meta.frame_id.size()+meta.source_id.size()+meta.truth_class.size()+cells.size()*9);
  data.insert(data.end(),kMagic.begin(),kMagic.end());
  add_u(data,kVersion,4);
  add_u(data,meta.metric_origin_verified?1:0,4);
  add_u(data,cfg.width,4);add_u(data,cfg.height,4);
  add_f64(data,cfg.resolution_m);
  add_f64(data,cfg.map_from_grid.translation.x());
  add_f64(data,cfg.map_from_grid.translation.y());
  add_f64(data,cfg.map_from_grid.yaw_rad);
  add_f64(data,cfg.hit_log_odds);add_f64(data,cfg.free_log_odds);
  add_f64(data,cfg.min_log_odds);add_f64(data,cfg.max_log_odds);
  add_u(data,cfg.clear_trusted_no_returns?1:0,4);
  add_u(data,bundle.snapshot->revision(),8);
  add_u(data,bundle.snapshot->fingerprint(),8);
  add_label(data,meta.frame_id);add_label(data,meta.source_id);add_label(data,meta.truth_class);
  add_u(data,cells.size(),8);
  for(const auto& c:cells) {add_u(data,c.observed?1:0,1);add_f64(data,c.log_odds);}
  const auto check=fnv(data.data(),data.size());
  add_u(data,check,8);
  out.write(reinterpret_cast<const char*>(data.data()),static_cast<std::streamsize>(data.size()));
  if(!out)throw std::runtime_error("failed writing map bundle");
}

MapBundle read_map_bundle(std::istream& in) {
  std::vector<std::uint8_t> data;
  data.reserve(1024);
  char chunk[8192];
  while(in.read(chunk,sizeof(chunk))||in.gcount()) {
    const auto n=static_cast<std::size_t>(in.gcount());
    if(n>kMaxBytes-data.size())fail("map bundle exceeds maximum size");
    for(std::size_t i=0;i<n;++i)data.push_back(static_cast<std::uint8_t>(chunk[i]));
  }
  if(in.bad())throw std::runtime_error("map bundle stream read failed");
  if(data.size()<8+4+4+8)fail("truncated map bundle header");
  if(!std::equal(kMagic.begin(),kMagic.end(),data.begin()))fail("unknown map bundle magic");
  Reader r(data);
  for(std::size_t i=0;i<kMagic.size();++i)(void)r.u(1);
  if(r.u(4)!=kVersion)fail("unsupported map bundle version");
  const auto flags=r.u(4);
  if(flags>1)fail("unknown map bundle flags");
  GridConfig cfg;
  cfg.width=r.u(4);cfg.height=r.u(4);
  cfg.resolution_m=r.f64();
  cfg.map_from_grid.translation.x()=r.f64();
  cfg.map_from_grid.translation.y()=r.f64();
  cfg.map_from_grid.yaw_rad=r.f64();
  cfg.hit_log_odds=r.f64();cfg.free_log_odds=r.f64();
  cfg.min_log_odds=r.f64();cfg.max_log_odds=r.f64();
  const auto clear=r.u(4);if(clear>1)fail("invalid no-return flag");
  cfg.clear_trusted_no_returns=clear!=0;
  const auto revision=r.u(8);
  const auto fingerprint=r.u(8);
  MapProvenance meta;
  meta.metric_origin_verified=flags==1;
  meta.frame_id=r.label();meta.source_id=r.label();meta.truth_class=r.label();
  const auto count=r.u(8);
  if(count>kMaxCells||cfg.width==0||cfg.height==0||cfg.width>kMaxCells||
     cfg.height>kMaxCells/cfg.width||count!=cfg.width*cfg.height||
     count>(r.remaining()-std::min<std::size_t>(r.remaining(),8))/9)
    fail("invalid map cell count");
  if(r.remaining()!=count*9+8)fail("unexpected map payload length/trailing bytes");
  const auto payload_end=data.size()-8;
  const auto expected_check=fnv(data.data(),payload_end);
  std::vector<Cell> cells;
  cells.reserve(static_cast<std::size_t>(count));
  for(std::uint64_t i=0;i<count;++i) {
    const auto observed=r.u(1);
    if(observed>1)fail("invalid observed mask");
    cells.push_back({r.f64(),observed==1});
  }
  if(r.u(8)!=expected_check)fail("map bundle checksum mismatch");
  auto snapshot=MapSnapshot::from_cells(cfg,revision,std::move(cells));
  if(snapshot->fingerprint()!=fingerprint)fail("map bundle fingerprint mismatch");
  return {std::move(snapshot),std::move(meta)};
}

void write_map_pgm(std::ostream& out,const MapSnapshot& snapshot) {
  const auto& cfg=snapshot.config();
  const auto& cells=snapshot.cells();
  validate_snapshot(cfg,cells);
  out<<"P5\n"<<cfg.width<<' '<<cfg.height<<"\n255\n";
  for(std::size_t y=cfg.height;y>0;--y)
    for(std::size_t x=0;x<cfg.width;++x) {
      const auto& cell=cells[(y-1)*cfg.width+x];
      std::uint8_t pixel=205; // unknown, but grayscale PGM is lossy!
      if(cell.observed) {
        const double probability=1.0/(1.0+std::exp(-cell.log_odds));
        pixel=static_cast<std::uint8_t>(std::lround(255.0*(1.0-probability)));
      }
      out.put(static_cast<char>(pixel));
    }
  if(!out)throw std::runtime_error("failed writing map PGM");
}

void write_map_yaml(std::ostream& out,const MapSnapshot& snapshot,const std::string& pgm_basename) {
  const auto name=clean_basename(pgm_basename);
  const auto& cfg=snapshot.config();
  out.imbue(std::locale::classic());
  out<<std::setprecision(std::numeric_limits<double>::max_digits10)
     <<"# Visualization only: authoritative known mask and log-odds are in .slammap\n"
     <<"image: "<<name<<"\nresolution: "<<cfg.resolution_m
     <<"\norigin: ["<<cfg.map_from_grid.translation.x()<<", "
     <<cfg.map_from_grid.translation.y()<<", "<<cfg.map_from_grid.yaw_rad
     <<"]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.19\n";
  if(!out)throw std::runtime_error("failed writing map YAML");
}
} // namespace slam::mapping
