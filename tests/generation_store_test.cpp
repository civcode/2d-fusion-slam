#include "slam/mapping/generation_store.hpp"
#include "slam/mapping/occupancy.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace slam;
using namespace slam::mapping;
namespace fs=std::filesystem;
namespace {
void check(bool yes, const char* message){if(!yes){std::cerr<<"FAIL "<<message<<'\n';std::exit(1);}}
template<class F> void throws(F&& f,const char* message){
  bool threw=false;
  try{f();}catch(const std::exception&){threw=true;}
  check(threw,message);
}
MapBundle make_map(unsigned variant){
  GridConfig c;
  c.width=48;c.height=48;c.resolution_m=0.1;
  OccupancyGrid grid(c);
  for(unsigned i=0;i<variant;++i){
    const double x=1.0+i*0.1;
    (void)grid.integrate_rays({{{0.3,0.5},{x,3.1},slam::sensors::ReturnKind::hit}});
  }
  return {MapSnapshot::capture(grid),{"synthetic-map","test-sensor","synthetic_exact",true}};
}
}
int main(){
  char tmp[]="/tmp/slam-generation-test-XXXXXX";
  const char* d=::mkdtemp(tmp);
  if(!d)throw std::runtime_error("mkdtemp failed");
  const fs::path root(d);
  try {
    GenerationStore store(root);
    check(!store.state().active,"empty registry");
    throws([&]{(void)store.load_active();},"empty load denied");
    const auto first=make_map(2);
    const auto one=store.publish(first,std::nullopt);
    check(one.id==1&&one.fingerprint==first.snapshot->fingerprint(),"bootstrap id");
    check(store.load_active().snapshot->fingerprint()==one.fingerprint,"bootstrap roundtrip");
    throws([&]{(void)store.publish(make_map(3),std::nullopt);},"stale empty expectation denied");
    throws([&]{(void)store.publish(make_map(3),one.fingerprint+1);},"stale fingerprint denied");
    throws([&]{(void)store.publish(first,one.fingerprint);},"duplicate map denied");

    const auto two=store.publish(make_map(3),one.fingerprint);
    auto state=store.state();
    check(two.id==2 && state.active && state.active->id==2 && state.previous &&
          state.previous->id==1,"new active and rollback pointer");
    check(store.load_active().snapshot->fingerprint()==two.fingerprint,"second map loaded");

    // Existing, ignored temp files cannot influence the authoritative pointer.
    {std::ofstream temp(root/".pending-stale-CURRENT");temp<<"interrupted publication";}
    check(store.recover().active->id==2,"orphan temporary ignored");

    // Deliberately damage the active map and check automatic, persistent
    // recovery to the original verified immutable generation.
    {std::fstream corrupt(root/"generation-2.slammap",std::ios::in|std::ios::out|std::ios::binary);
      corrupt.seekp(0);corrupt.write("BAD!",4);}
    throws([&]{(void)store.load_active();},"corrupt active never loaded");
    const auto restored=store.recover();
    check(restored.recovered_from_previous && restored.active && restored.active->id==1 &&
          !restored.previous,"fallback to previous");
    check(GenerationStore(root).load_active().snapshot->fingerprint()==one.fingerprint,
          "recovery survives new process/store object");

    // A crash after writing generation but before updating CURRENT leaves
    // an unreferenced map; future IDs must not overwrite this orphan.
    fs::copy_file(root/"generation-1.slammap",root/"generation-15.slammap");
    const auto third=store.publish(make_map(4),one.fingerprint);
    check(third.id==16,"orphan generation id never reused");
    const auto rolled=store.rollback(third.fingerprint);
    check(rolled.active && rolled.active->id==1 && !rolled.previous,"explicit rollback");
    throws([&]{(void)store.rollback(third.fingerprint);},"second rollback denied");

    // Manifest corruption is fail-closed: never guess which orphan map is
    // authoritative or choose the highest numbered filename.
    {std::ofstream bad(root/"CURRENT",std::ios::trunc);bad<<"damaged pointer";}
    throws([&]{(void)store.recover();},"corrupt registry fails closed");
    throws([&]{(void)store.publish(make_map(5),std::nullopt);},"cannot publish over damaged registry");

    // Concurrent writers are serialized: both expect an empty/current map;
    // only one can publish. Use a separate fresh directory for this scenario.
    const auto concurrent=root/"concurrent";
    GenerationStore pairs(concurrent);
    const auto seed=pairs.publish(make_map(2),std::nullopt);
    int successful=0;
    pid_t children[2]{};
    for(int i=0;i<2;++i){
      children[i]=fork();
      if(children[i]<0)throw std::runtime_error("fork failed");
      if(children[i]==0){
        try {GenerationStore child(concurrent);(void)child.publish(make_map(4+static_cast<unsigned>(i)),seed.fingerprint);_exit(0);}
        catch(const std::exception&){_exit(1);}
      }
    }
    for(const pid_t id:children){int status=0;if(waitpid(id,&status,0)<0)throw std::runtime_error("waitpid failed");
      if(WIFEXITED(status) && WEXITSTATUS(status)==0)++successful;}
    check(successful==1,"concurrent compare-and-swap publish admits exactly one");
    check(pairs.recover().active->id==2,"concurrent winner valid");

    std::cout<<"map generation atomic publish / recovery / rollback / race checks passed\n";
  }catch(...){fs::remove_all(root);throw;}
  fs::remove_all(root);
}
