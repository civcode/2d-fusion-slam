#include "apps/demo/native/data.hpp"
#include "apps/demo/native/fusion.hpp"

#include <charconv>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <string>
#include <string_view>
#ifdef __unix__
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#endif

#ifndef FUSION_VIEWER_STATIC_ROOT
#define FUSION_VIEWER_STATIC_ROOT "apps/demo/web/static"
#endif

namespace {
using slam::viewer::Json;
struct Response {
  int code{200};
  std::string mime{"application/json; charset=utf-8"};
  std::string body;
};
Response json(Json j,int code=200) {return {code,"application/json; charset=utf-8",j.dump()};}
Response error(int code,std::string message) {
  Json j=Json::dict();j.object.emplace("error",Json::text(std::move(message)));
  return json(std::move(j),code);
}
bool decimal(std::string_view s,std::size_t& n) {
  if(s.empty()||s.size()>8||(s.size()>1&&s.front()=='0'))return false;
  for(char c:s)if(c<'0'||c>'9')return false;
  auto [end,ec]=std::from_chars(s.data(),s.data()+s.size(),n);
  return ec==std::errc{}&&end==s.data()+s.size();
}
std::string asset(std::string_view name,const std::filesystem::path& root){
  // Never use a URL-derived filesystem path: only fixed allowlisted assets.
  const auto file=root/std::string(name);
  if(!std::filesystem::is_regular_file(std::filesystem::symlink_status(file))||
     std::filesystem::file_size(file)>2*1024*1024)
    throw std::invalid_argument("web asset unavailable");
  std::ifstream in(file,std::ios::binary);
  if(!in)throw std::runtime_error("cannot open web asset");
  std::ostringstream data;data<<in.rdbuf();return data.str();
}
Response route(slam::viewer::ViewerData& data,slam::viewer::FusionRuns& fusion,
               const std::filesystem::path& static_root,
               std::string_view target) {
  if(target.size()>2048) return error(400,"request URL too long");
  const auto q=target.find('?');
  const auto path=target.substr(0,q);
  const auto query=q==std::string_view::npos?std::string_view{}:target.substr(q+1);
  if(path=="/api/health") {
    Json r=Json::dict();r.object={{"status",Json::text("ok")},
      {"mode",Json::text("read-only-offline")}};
    return json(std::move(r));
  }
  if(path=="/api/fusion/runs")return json(fusion.index());
  constexpr std::string_view fusion_prefix="/api/fusion/runs/";
  if(path.starts_with(fusion_prefix)){
    if(!query.empty())return error(422,"unsupported fusion query");
    const auto rest=path.substr(fusion_prefix.size());
    const auto slash=rest.find('/');
    const auto id=std::string(rest.substr(0,slash));
    if(id.empty())return error(404,"invalid fusion run");
    if(slash==std::string_view::npos)return json(fusion.report(id));
    const auto sub=rest.substr(slash+1);
    if(sub=="trajectories")return json(fusion.trajectories(id));
    if(sub=="profile")return json(fusion.profile(id));
    if(sub=="replay")return json(fusion.replay(id));
    constexpr std::string_view frame_marker="frame/";
    if(sub.starts_with(frame_marker)){
      std::size_t frame=0;
      if(!decimal(sub.substr(frame_marker.size()),frame)||frame>=10000)
        return error(404,"invalid fusion frame index");
      return json(fusion.frame(id,frame));
    }
    constexpr std::string_view map_marker="maps/";
    if(sub.starts_with(map_marker)){
      auto method=sub.substr(map_marker.size());
      bool image=false;
      if(method.ends_with("/image.png")){
        method.remove_suffix(std::string_view("/image.png").size());image=true;
      }
      if(method.empty()||method.find('/')!=std::string_view::npos)
        return error(404,"invalid fusion map method");
      if(image)return {200,"image/png",fusion.map_png(id,std::string(method))};
      return json(fusion.map_metadata(id,std::string(method)));
    }
    return error(404,"unknown fusion endpoint");
  }
  if(path=="/api/maps") {
    Json listing=data.list_maps();
    // Fusion run maps are exposed through the *same* map-inspection
    // contract, without copying artifacts or pretending they are published.
    for(const auto& run:fusion.index().at("runs").array) {
      const auto id=run.at("id").str();
      try {
        const auto report=fusion.report(id);
        for(const auto& [method,_]:report.at("results").object) {
          Json item=Json::dict();
          const std::string key="fusion@"+id+"@"+method;
          item.object={{"id",Json::text(key)},
                       {"name",Json::text(id+" / "+method)},
                       {"role",Json::text("fusion")}};
          listing.object.at("maps").array.push_back(std::move(item));
        }
      }catch(const std::exception&){
        // A broken or partially written run must not hide valid maps.
        // Selecting any invalid map remains a hard 422; it is never listed.
      }
    }
    return json(std::move(listing));
  }
  if(path=="/api/replays") return json(data.list_replays());
  if(path=="/"||path=="/index.html"||path=="/app.js"||
     path=="/styles.css"||path=="/replay"||path=="/replay.html"||
     path=="/replay.js"||path=="/replay.css"||
     path=="/fusion"||path=="/fusion.html"||
     path=="/fusion.js"||path=="/fusion.css") {
    const auto file=path=="/"?"index.html":path=="/replay"?"replay.html":
                    path=="/fusion"?"fusion.html":
                    std::string(path.substr(1));
    const auto mime=file.ends_with(".js")?"text/javascript; charset=utf-8":
                    file.ends_with(".css")?"text/css; charset=utf-8":
                    "text/html; charset=utf-8";
    return {200,mime,asset(file,static_root)};
  }
  constexpr std::string_view maps="/api/maps/";
  constexpr std::string_view replays="/api/replays/";
  if(path.starts_with(maps)){
    auto name=path.substr(maps.size());
    std::string mode;
    constexpr std::string_view image="/image.png",cell="/cell";
    if(name.ends_with(image)){name.remove_suffix(image.size());mode="image";}
    else if(name.ends_with(cell)){name.remove_suffix(cell.size());mode="cell";}
    if(name.empty()||name.size()>200||name.find('/')!=std::string_view::npos)
      return error(404,"invalid map route");
    std::string run,method;
    if(name.starts_with("fusion@")){
      const auto id=name.substr(7);
      const auto split=id.find('@');
      if(split==std::string_view::npos||id.find('@',split+1)!=std::string_view::npos)
        return error(404,"invalid fusion map id");
      run=std::string(id.substr(0,split));
      method=std::string(id.substr(split+1));
    }
    if(!mode.empty()&&mode=="cell"){
      const auto amp=query.find('&');
      if(amp==std::string_view::npos||query.find('&',amp+1)!=std::string_view::npos)
        return error(422,"cell query requires x and y");
      auto a=query.substr(0,amp),b=query.substr(amp+1);
      if(a.starts_with("y=")&&b.starts_with("x="))std::swap(a,b);
      if(!a.starts_with("x=")||!b.starts_with("y="))
        return error(422,"cell query requires x and y");
      std::size_t x=0,y=0;
      if(!decimal(a.substr(2),x)||!decimal(b.substr(2),y))
        return error(422,"cell coordinates require bounded nonnegative integers");
      if(!run.empty())return json(fusion.map_cell(run,method,x,y));
      return json(data.map_cell(std::string(name),x,y));
    }
    if(!query.empty()) return error(422,"unsupported map query");
    if(!run.empty()){
      if(mode=="image")return {200,"image/png",fusion.map_png(run,method)};
      return json(fusion.map_metadata(run,method));
    }
    if(mode=="image")return {200,"image/png",data.map_png(std::string(name))};
    return json(data.map_metadata(std::string(name)));
  }
  if(path.starts_with(replays)){
    auto name=path.substr(replays.size());
    constexpr std::string_view marker="/frame/";
    const auto frame=name.find(marker);
    if(frame!=std::string_view::npos){
      const auto index=name.substr(frame+marker.size());name=name.substr(0,frame);
      std::size_t i=0;
      if(!decimal(index,i)||i>6000)return error(404,"invalid replay frame index");
      if(!query.empty())return error(422,"unsupported replay query");
      return json(data.replay_frame(std::string(name),i));
    }
    if(!query.empty())return error(422,"unsupported replay query");
    if(name.empty()||name.find('/')!=std::string_view::npos)return error(404,"invalid replay path");
    return json(data.replay_info(std::string(name)));
  }
  return error(404,"not found");
}
#ifdef __unix__
void reply(int fd,const Response& r){
  const char* reason=r.code==200?"OK":r.code==400?"Bad Request":
                     r.code==404?"Not Found":r.code==405?"Method Not Allowed":
                     r.code==422?"Unprocessable Content":"Internal Server Error";
  std::ostringstream head;
  head<<"HTTP/1.1 "<<r.code<<' '<<reason<<"\r\n"
      <<"Content-Type: "<<r.mime<<"\r\nContent-Length: "<<r.body.size()
      <<"\r\nConnection: close\r\nCache-Control: no-store\r\n"
      <<"X-Content-Type-Options: nosniff\r\nX-Frame-Options: DENY\r\n"
      <<"Referrer-Policy: no-referrer\r\nCross-Origin-Resource-Policy: same-origin\r\n"
      <<"Content-Security-Policy: default-src 'self'; img-src 'self' data: blob:; "
        "style-src 'self'; script-src 'self'; object-src 'none'; "
        "base-uri 'none'; frame-ancestors 'none'\r\n\r\n";
  const auto bytes=head.str()+r.body;
  for(std::size_t i=0;i<bytes.size();){
    const auto size=::send(fd,bytes.data()+i,bytes.size()-i,MSG_NOSIGNAL);
    if(size<=0)break;
    i+=static_cast<std::size_t>(size);
  }
}
void handle(int client,slam::viewer::ViewerData& viewer,
            slam::viewer::FusionRuns& fusion,
            const std::filesystem::path& static_root) {
  timeval timeout{3,0};
  ::setsockopt(client,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
  ::setsockopt(client,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
  try{
    std::string request;request.reserve(4096);
    char chunk[2048];
    while(request.find("\r\n\r\n")==std::string::npos){
      if(request.size()>8192){reply(client,error(400,"headers too long"));return;}
      const auto n=::recv(client,chunk,sizeof(chunk),0);
      if(n<=0){reply(client,error(400,"incomplete HTTP request"));return;}
      request.append(chunk,static_cast<std::size_t>(n));
      const auto end_of_headers=request.find("\r\n\r\n");
      if((end_of_headers==std::string::npos && request.size()>8192) ||
         (end_of_headers!=std::string::npos && end_of_headers>8192)) {
        reply(client,error(400,"headers too long"));return;
      }
    }
    const auto end=request.find("\r\n");
    if(end==std::string::npos||end>4096){reply(client,error(400,"invalid request line"));return;}
    const std::string_view first(request.data(),end);
    const auto sp=first.find(' '),other=sp==std::string_view::npos?
        std::string_view::npos:first.find(' ',sp+1);
    if(sp==std::string_view::npos||other==std::string_view::npos||
       first.substr(other)!=" HTTP/1.1"&&first.substr(other)!=" HTTP/1.0"){
      reply(client,error(400,"invalid HTTP method/version"));return;
    }
    const auto method=first.substr(0,sp),target=first.substr(sp+1,other-sp-1);
    if(method!="GET"){reply(client,error(405,"read-only dashboard: only GET allowed"));return;}
    reply(client,route(viewer,fusion,static_root,target));
  }catch(const std::invalid_argument& e){reply(client,error(422,e.what()));}
  catch(const std::filesystem::filesystem_error&){reply(client,error(422,"invalid viewer file"));}
  catch(const std::exception&){reply(client,error(500,"internal viewer error"));}
}
void serve(slam::viewer::ViewerData& viewer,slam::viewer::FusionRuns& fusion,
           const std::filesystem::path& static_root,
           unsigned port,bool broad){
  const int listener=::socket(AF_INET,SOCK_STREAM,0);
  if(listener<0)throw std::runtime_error("cannot open server socket");
  struct Guard{int fd;~Guard(){::close(fd);}} guard{listener};
  int reuse=1;::setsockopt(listener,SOL_SOCKET,SO_REUSEADDR,&reuse,sizeof(reuse));
  sockaddr_in addr{};addr.sin_family=AF_INET;addr.sin_port=htons(static_cast<std::uint16_t>(port));
  addr.sin_addr.s_addr=htonl(broad?INADDR_ANY:INADDR_LOOPBACK);
  if(::bind(listener,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))!=0||
     ::listen(listener,16)!=0)throw std::runtime_error("failed to bind viewer socket");
  sockaddr_in bound{};socklen_t n=sizeof(bound);
  if(::getsockname(listener,reinterpret_cast<sockaddr*>(&bound),&n)!=0)
    throw std::runtime_error("cannot read bound port");
  std::cout<<"Native read-only SLAM dashboard: http://127.0.0.1:"
           <<ntohs(bound.sin_port)<<'\n'<<std::flush;
  if(broad)std::cerr<<"WARNING: no HTTP authentication; publish host loopback only.\n";
  // Deliberately single-threaded, short-timeout localhost viewer. Keeps HTTP
  // work outside the robot SLAM/control process. No mutation endpoints.
  for(;;){
    const int fd=::accept(listener,nullptr,nullptr);
    if(fd<0){if(errno==EINTR)continue;throw std::runtime_error("viewer accept failed");}
    handle(fd,viewer,fusion,static_root);
    ::close(fd);
  }
}
#endif
}
int main(int argc,char** argv) {
  try{
    std::filesystem::path maps,replays,fusion_directory;
    const std::filesystem::path static_root=FUSION_VIEWER_STATIC_ROOT;
    unsigned port=8765;bool allow_network=false,broad=false,example=false;
    bool has_replays=false,has_fusion=false;
    for(int i=1;i<argc;++i){
      const std::string arg=argv[i];
      auto value=[&]()->std::string{
        if(++i>=argc)throw std::invalid_argument("missing value for "+arg);
        return argv[i];
      };
      if(arg=="--data-dir")maps=value();
      else if(arg=="--replay-dir"){replays=value();has_replays=true;}
      else if(arg=="--fusion-dir"){fusion_directory=value();has_fusion=true;}
      else if(arg=="--port"){
        const auto val=value();
        std::size_t v=0;
        if(!decimal(val,v)||v>65535)
          throw std::invalid_argument("viewer port must be 0..65535 (0 chooses a free local port)");
        port=static_cast<unsigned>(v);
      }
      else if(arg=="--host"){
        const auto val=value();
        if(val=="127.0.0.1")broad=false;
        else if(val=="0.0.0.0")broad=true;
        else throw std::invalid_argument("host must be loopback or 0.0.0.0");
      }
      else if(arg=="--allow-network")allow_network=true;
      else if(arg=="--example-if-empty")example=true;
      else if(arg=="--help"){
        std::cout<<"Usage: fusion_slam_demo --data-dir MAP_DIR [--replay-dir REPLAY_DIR] "
          "[--fusion-dir RUNS_DIR] [--port PORT] [--host 127.0.0.1|0.0.0.0] "
          "[--allow-network] [--example-if-empty]\n";
        return 0;
      }
      else throw std::invalid_argument("unsupported viewer argument "+arg);
    }
    if(maps.empty())throw std::invalid_argument("--data-dir is required");
    if(broad&&!allow_network)throw std::invalid_argument(
       "binding to non-loopback requires explicit --allow-network");
    slam::viewer::ViewerData viewer(maps,has_replays?replays:maps,example&&!has_replays);
    slam::viewer::FusionRuns fusion(has_fusion?fusion_directory:maps);
#ifdef __unix__
    std::signal(SIGPIPE,SIG_IGN);
    serve(viewer,fusion,static_root,port,broad);
#else
    throw std::runtime_error("native demo server requires POSIX sockets");
#endif
  }catch(const std::exception& e){
    std::cerr<<"Native viewer failed: "<<e.what()<<'\n';
    return 2;
  }
}
