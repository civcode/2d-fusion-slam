#include "slam/datasets/carmen_map.hpp"
#include <zlib.h>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
int main(int argc,char** argv) {
 if(argc!=2 && argc!=4){std::cerr<<"Usage: slam_carmen_map_inspect <.carmen.map> [width height]\n";return 2;}
 gzFile gz=gzopen(argv[1],"rb");
 if(!gz){std::cerr<<"cannot open CARMEN map\n";return 1;}
 try {
   std::vector<std::uint8_t> bytes;
   unsigned char chunk[8192];int n=0;
   while((n=gzread(gz,chunk,sizeof(chunk)))>0) {
      if(bytes.size()+static_cast<std::size_t>(n)>128*1024*1024)throw std::runtime_error("map exceeds limit");
      bytes.insert(bytes.end(),chunk,chunk+n);
   }
   const int close_status=gzclose(gz);
   gz=nullptr;
   if(n<0 || close_status!=Z_OK)throw std::runtime_error("gzip map failure");
   const auto map=slam::datasets::parse_carmen_map(bytes);
   if(argc==4 && (map.width!=std::stoul(argv[2]) || map.height!=std::stoul(argv[3])))
     throw std::runtime_error("unexpected map dimensions");
   std::cout<<"{\"width\":"<<map.width<<",\"height\":"<<map.height
            <<",\"resolution_m\":"<<map.resolution_m
            <<",\"known\":"<<map.known_cells<<",\"unknown\":"<<map.unknown_cells<<"}\n";
 }catch(const std::exception& e) {
   if(gz)gzclose(gz);
   std::cerr<<"map import failed: "<<e.what()<<'\n';return 1;
 }
}
