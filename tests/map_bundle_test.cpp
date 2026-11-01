#include "slam/mapping/map_bundle.hpp"
#include "slam/mapping/occupancy.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace slam;
using namespace slam::mapping;
using slam::sensors::ReturnKind;
namespace {
void ensure(bool ok,const char* msg){if(!ok){std::cerr<<"FAILED: "<<msg<<'\n';std::abort();}}
template<class F>void must_throw(F call,const char* message){bool failed=false;try{call();}catch(const std::exception&){failed=true;}ensure(failed,message);}
}
int main(){
 GridConfig config;
 config.width=10;config.height=8;config.resolution_m=0.25;
 config.map_from_grid={{1.25,-3.5},0.3};
 OccupancyGrid map(config);
 const auto original=MapSnapshot::capture(map);
 (void)map.integrate_rays({{{1.25,-3.5},{2.0,-2.8},ReturnKind::hit}});
 const auto current=MapSnapshot::capture(map);
 // Keep known/unknown distinction, including exact zero odds for observed cells.
 auto data=current->cells();
 data[1]={0,true};
 const auto versioned=MapSnapshot::from_cells(config,current->revision(),std::move(data));
 const MapBundle bundle{versioned,{"synthetic_grid","synthetic-fixture-v1","synthetic_exact",true}};
 std::ostringstream encoded(std::ios::binary|std::ios::out);
 write_map_bundle(encoded,bundle);
 const std::string bytes=encoded.str();
 ensure(bytes.size()>200,"binary generated");
 std::ostringstream second(std::ios::binary|std::ios::out);write_map_bundle(second,bundle);
 ensure(second.str()==bytes,"byte-identical repeated serialization");
 std::istringstream input(bytes,std::ios::binary|std::ios::in);
 const auto restored=read_map_bundle(input);
 ensure(restored.snapshot->fingerprint()==versioned->fingerprint(),"exact fingerprint");
 ensure(restored.snapshot->revision()==versioned->revision(),"exact map revision");
 ensure(restored.provenance.frame_id=="synthetic_grid"&&restored.provenance.metric_origin_verified,"provenance roundtrip");
 ensure(restored.snapshot->config().map_from_grid.translation.x()==config.map_from_grid.translation.x(),"metric origin X");
 ensure(restored.snapshot->config().map_from_grid.yaw_rad==config.map_from_grid.yaw_rad,"metric yaw");
 ensure(restored.snapshot->cell(1,0)->observed && restored.snapshot->cell(1,0)->log_odds==0.0,"known neutral remains distinct");
 ensure(!restored.snapshot->cell(9,7)->observed,"unknown stays unknown");
 ensure(!original->cell(1,0)->observed,"original map immutable");
 // EDT generated after roundtrip must match original map generation.
 const auto field1=DistanceField::build(versioned,2.0);
 const auto field2=DistanceField::build(restored.snapshot,2.0);
 for(std::size_t y=0;y<config.height;++y)for(std::size_t x=0;x<config.width;++x)
  ensure(field1->cell_distance(x,y)==field2->cell_distance(x,y),"EDT roundtrip");
 std::ostringstream pgm(std::ios::binary|std::ios::out);
 write_map_pgm(pgm,*restored.snapshot);
 const auto pgm_bytes=pgm.str();
 const auto image_begin=pgm_bytes.find("\n255\n");
 ensure(image_begin!=std::string::npos,"PGM header");
 ensure(pgm_bytes.size()-image_begin-5==config.width*config.height,"PGM exact payload dimensions");
 ensure(static_cast<unsigned char>(pgm_bytes.back())!=0,"PGM nonempty data");
 std::ostringstream yaml;
 write_map_yaml(yaml,*restored.snapshot,"example-map.pgm");
 ensure(yaml.str().find("image: example-map.pgm")!=std::string::npos,"PGM YAML path");
 ensure(yaml.str().find("resolution:")!=std::string::npos,"PGM YAML resolution");
 must_throw([&]{std::ostringstream ignored;write_map_yaml(ignored,*restored.snapshot,"../escape.pgm");},"YAML basename traversal rejected");
 // Unknown version, truncated length, payload tampering, and appended junk fail closed.
 must_throw([&]{std::ostringstream invalid;write_map_bundle(invalid,{versioned,{"bad\"json","source","truth",false}});},"unsafe metadata label rejected");
 std::string corrupt=bytes;
 corrupt[8]=2;
 must_throw([&]{std::istringstream in(corrupt,std::ios::binary);(void)read_map_bundle(in);},"unknown schema rejected");
 corrupt=bytes;corrupt[corrupt.size()-10]^=0x10;
 must_throw([&]{std::istringstream in(corrupt,std::ios::binary);(void)read_map_bundle(in);},"corrupt data detected");
 corrupt=bytes.substr(0,bytes.size()-1);
 must_throw([&]{std::istringstream in(corrupt,std::ios::binary);(void)read_map_bundle(in);},"truncated checksum rejected");
 corrupt=bytes+"J";
 must_throw([&]{std::istringstream in(corrupt,std::ios::binary);(void)read_map_bundle(in);},"trailing bytes rejected");
 corrupt=bytes;corrupt[12]=static_cast<char>(0xff);
 must_throw([&]{std::istringstream in(corrupt,std::ios::binary);(void)read_map_bundle(in);},"unknown flags rejected");
 // Unknown log odds cannot contain fabricated evidence.
 data=versioned->cells();data[79]={1,false};
 must_throw([&]{(void)MapSnapshot::from_cells(config,3,data);},"unknown cannot carry occupied log odds");
 std::cout<<"binary map bundle, integrity, known mask, EDT and PGM tests passed\n";
}
