#include "slam/mapping/generation_store.hpp"

#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <vector>
#include <algorithm>
#include <charconv>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace slam::mapping {
namespace {
namespace fs=std::filesystem;
constexpr std::uint64_t kFnv=14695981039346656037ULL;
constexpr std::uint64_t kPrime=1099511628211ULL;

std::uint64_t checksum(std::string_view x) {
  std::uint64_t h=kFnv;
  for(unsigned char c: x) {h^=c;h*=kPrime;}
  return h;
}
std::uint64_t number(std::string_view s) {
  if(s.empty() || (s.size()>1 && s[0]=='0'))
    throw std::runtime_error("noncanonical generation number");
  std::uint64_t v{};
  const auto [p,e]=std::from_chars(s.data(),s.data()+s.size(),v);
  if(e!=std::errc{} || p!=s.data()+s.size())
    throw std::runtime_error("malformed generation number");
  return v;
}
std::string record(const std::optional<GenerationRef>& ref) {
  if(!ref)return "0 0\n";
  if(ref->id==0)throw std::runtime_error("zero generation id");
  return std::to_string(ref->id)+" "+std::to_string(ref->fingerprint)+"\n";
}
std::string manifest_text(const GenerationState& state) {
  std::string head="FUSGEN1\n"+record(state.active)+record(state.previous);
  return head+std::to_string(checksum(head))+"\n";
}
std::optional<GenerationRef> parse_record(const std::string& row) {
  const auto pos=row.find(' ');
  if(pos==std::string::npos || row.find(' ',pos+1)!=std::string::npos)
    throw std::runtime_error("bad generation pointer");
  const auto id=number(std::string_view(row).substr(0,pos));
  const auto hash=number(std::string_view(row).substr(pos+1));
  if(id==0) {
    if(hash!=0)throw std::runtime_error("null generation fingerprint is not zero");
    return std::nullopt;
  }
  return GenerationRef{id,hash};
}
GenerationState read_manifest(const fs::path& root) {
  std::ifstream f(root/"CURRENT",std::ios::binary);
  if(!f) {
    if(fs::exists(root/"CURRENT"))throw std::runtime_error("cannot read map registry");
    return {};
  }
  std::string lines[4];
  for(auto& line:lines)if(!std::getline(f,line))throw std::runtime_error("truncated generation registry");
  std::string extra;
  if(std::getline(f,extra) || !f.eof() || lines[0]!="FUSGEN1")
    throw std::runtime_error("unexpected generation registry content/version");
  const std::string payload=lines[0]+"\n"+lines[1]+"\n"+lines[2]+"\n";
  if(checksum(payload)!=number(lines[3]))throw std::runtime_error("corrupt generation registry checksum");
  GenerationState s;
  s.active=parse_record(lines[1]);
  s.previous=parse_record(lines[2]);
  if(s.previous && !s.active)throw std::runtime_error("orphaned previous pointer");
  if(s.active && s.previous && s.active->id==s.previous->id)
    throw std::runtime_error("duplicate active/previous pointer");
  return s;
}

class ScopedLock {
 public:
  explicit ScopedLock(const fs::path& root) {
    auto name=(root/".lock").string();
    fd_=::open(name.c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    if(fd_<0 || ::flock(fd_,LOCK_EX)<0) {
      if(fd_>=0)::close(fd_);
      throw std::runtime_error("cannot lock map-generation directory");
    }
  }
  ~ScopedLock(){::flock(fd_,LOCK_UN);::close(fd_);}
  ScopedLock(const ScopedLock&)=delete;
  ScopedLock& operator=(const ScopedLock&)=delete;
 private:
  int fd_{-1};
};
void sync_dir(const fs::path& root) {
  int fd=::open(root.c_str(),O_RDONLY|O_DIRECTORY|O_CLOEXEC|O_NOFOLLOW);
  if(fd<0)throw std::runtime_error("cannot open generation directory");
  int status=::fsync(fd);
  ::close(fd);
  if(status!=0)throw std::runtime_error("generation directory fsync failed");
}
std::string map_filename(std::uint64_t id){return "generation-"+std::to_string(id)+".slammap";}

void atomic_write(const fs::path& root,const std::string& filename,std::string_view data){
  const fs::path dest=root/filename;
  const std::string pattern=(root/(".pending-"+filename+"-XXXXXX")).string();
  std::vector<char> temp_name(pattern.begin(),pattern.end());
  temp_name.push_back('\0');
  int fd=::mkstemp(temp_name.data());
  if(fd<0)throw std::runtime_error("cannot create generation temporary file");
  const std::string t(temp_name.data());
  try {
    std::size_t off=0;
    while(off<data.size()){
      const auto size=std::min<std::size_t>(data.size()-off,1024*1024);
      const auto n=::write(fd,data.data()+off,size);
      if(n<0 && errno==EINTR)continue;
      if(n<=0)throw std::runtime_error("generation file write failed");
      off+=static_cast<std::size_t>(n);
    }
    if(::fsync(fd)!=0)throw std::runtime_error("generation file fsync failed");
    if(::close(fd)!=0){fd=-1;throw std::runtime_error("generation file close failed");}
    fd=-1;
    if(::rename(t.c_str(),dest.c_str())!=0)throw std::runtime_error("generation file rename failed");
    sync_dir(root);
  }catch(...){if(fd>=0)::close(fd);std::error_code ec;fs::remove(t,ec);throw;}
}
MapBundle checked_read(const fs::path& root,GenerationRef ref) {
  const auto path=root/map_filename(ref.id);
  if(fs::is_symlink(path))throw std::runtime_error("map generation symlink refused");
  std::ifstream f(path,std::ios::binary);
  if(!f)throw std::runtime_error("map generation file unavailable");
  MapBundle b=read_map_bundle(f);
  if(b.snapshot->fingerprint()!=ref.fingerprint)
    throw std::runtime_error("map generation fingerprint mismatch");
  return b;
}
std::uint64_t next_id(const fs::path& root,const GenerationState& state) {
  std::uint64_t top=0;
  if(state.active)top=state.active->id;
  if(state.previous)top=std::max(top,state.previous->id);
  for(const auto& e:fs::directory_iterator(root)){
    const auto name=e.path().filename().string();
    constexpr std::string_view prefix="generation-", suffix=".slammap";
    if(name.size()<=prefix.size()+suffix.size() || name.compare(0,prefix.size(),prefix)!=0 ||
       name.compare(name.size()-suffix.size(),suffix.size(),suffix)!=0)continue;
    try {top=std::max(top,number(std::string_view(name).substr(prefix.size(),name.size()-prefix.size()-suffix.size())));}
    catch(const std::runtime_error&) {throw std::runtime_error("invalid generation filename");}
  }
  if(top==std::numeric_limits<std::uint64_t>::max())throw std::runtime_error("generation ids exhausted");
  return top+1;
}
} // namespace

GenerationStore::GenerationStore(fs::path root):root_(std::move(root)) {
  if(root_.empty() || fs::is_symlink(root_))throw std::invalid_argument("invalid generation directory");
  fs::create_directories(root_);
  if(!fs::is_directory(root_))throw std::invalid_argument("generation root not a directory");
}
GenerationState GenerationStore::state(){
  ScopedLock lock(root_);
  return read_manifest(root_);
}
MapBundle GenerationStore::load_active(){
  ScopedLock lock(root_);
  const auto s=read_manifest(root_);
  if(!s.active)throw std::runtime_error("no active map generation");
  return checked_read(root_,*s.active);
}
GenerationRef GenerationStore::publish(const MapBundle& candidate,
                                      std::optional<std::uint64_t> expected_active_fingerprint){
  ScopedLock lock(root_);
  const auto s=read_manifest(root_);
  if(s.active.has_value()!=expected_active_fingerprint.has_value() ||
     (s.active && s.active->fingerprint!=*expected_active_fingerprint))
    throw std::runtime_error("stale map-generation publication request");
  if(s.active)(void)checked_read(root_,*s.active); // cannot overwrite broken incumbent
  if(!candidate.snapshot)throw std::invalid_argument("missing candidate map");
  const auto fp=candidate.snapshot->fingerprint();
  if(s.active && s.active->fingerprint==fp)
    throw std::invalid_argument("new generation must change map content");
  std::ostringstream os(std::ios::binary);
  write_map_bundle(os,candidate); // completely validates candidate before writing
  GenerationRef next{next_id(root_,s),fp};
  atomic_write(root_,map_filename(next.id),os.str());
  const GenerationState news{next,s.active,false};
  atomic_write(root_,"CURRENT",manifest_text(news)); // atomic publication boundary
  return next;
}
GenerationState GenerationStore::recover(){
  ScopedLock lock(root_);
  auto s=read_manifest(root_);
  if(!s.active)return s;
  try{(void)checked_read(root_,*s.active);}catch(const std::exception&){
    if(!s.previous)throw std::runtime_error("no recoverable previous map generation");
    (void)checked_read(root_,*s.previous);
    s.active=s.previous;s.previous.reset();s.recovered_from_previous=true;
    atomic_write(root_,"CURRENT",manifest_text(s));
    return s;
  }
  if(s.previous){
    try{(void)checked_read(root_,*s.previous);}catch(const std::exception&){
      s.previous.reset();
      atomic_write(root_,"CURRENT",manifest_text(s));
    }
  }
  return s;
}
GenerationState GenerationStore::rollback(std::uint64_t expected_active_fingerprint){
  ScopedLock lock(root_);
  auto s=read_manifest(root_);
  if(!s.active || !s.previous || s.active->fingerprint!=expected_active_fingerprint)
    throw std::runtime_error("rollback generation mismatch or no previous generation");
  (void)checked_read(root_,*s.previous);
  s.active=s.previous;s.previous.reset();
  atomic_write(root_,"CURRENT",manifest_text(s));
  return s;
}
} // namespace slam::mapping
