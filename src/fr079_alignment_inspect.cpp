#include "slam/datasets/fr079_alignment.hpp"

#include <zlib.h>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
std::string gzip_read(const char* path, std::size_t limit) {
  gzFile f=gzopen(path,"rb"); if(!f)throw std::runtime_error("cannot open source");
  std::string content;char block[16384];int n=0;
  while((n=gzread(f,block,sizeof(block)))>0) {
    if(content.size()+static_cast<std::size_t>(n)>limit){gzclose(f);throw std::runtime_error("decompressed file exceeds limit");}
    content.append(block,static_cast<std::size_t>(n));
  }
  const auto status=gzclose(f);
  if(n<0||status!=Z_OK)throw std::runtime_error("bad gzip input");
  return content;
}
}
int main(int argc,char** argv) {
  if(argc!=3) {
    std::cerr<<"Usage: slam_fr079_alignment_inspect <fr079-sm.log.gz> <fr079-05cm.carmen.map>\n";
    return 2;
  }
  try {
    const auto log=gzip_read(argv[1],128*1024*1024);
    const auto mapfile=gzip_read(argv[2],128*1024*1024);
    const std::vector<std::uint8_t> binary(mapfile.begin(),mapfile.end());
    const auto map=slam::datasets::parse_carmen_map(binary);
    slam::datasets::CarmenReader reader;
    std::istringstream lines(log);std::string line;
    std::vector<slam::datasets::CarmenLaser> scans;
    while(std::getline(lines,line))if(auto scan=reader.parse_laser(line))scans.push_back(std::move(*scan));
    const auto result=slam::datasets::diagnose_fr079_alignment(map,scans);
    const auto& t=result.selected;
    std::cout<<"{\"type\":\"reference_map_registration_diagnostic\",\"scans\":"<<scans.size()
      <<",\"train_scans\":"<<result.calibration_scans
      <<",\"heldout_scans\":"<<result.held_out_scans
      <<",\"swap_xy\":"<<(t.swap_xy?"true":"false")
      <<",\"sign_x\":"<<t.sign_x<<",\"sign_y\":"<<t.sign_y
      <<",\"shift_x_m\":"<<t.translation_x<<",\"shift_y_m\":"<<t.translation_y
      <<",\"train_known_fraction\":"<<result.calibration.known_fraction()
      <<",\"train_near_wall_fraction\":"<<result.calibration.wall_fraction()
      <<",\"heldout_known_fraction\":"<<result.held_out.known_fraction()
      <<",\"heldout_near_wall_fraction\":"<<result.held_out.wall_fraction()
      <<",\"independent_truth\":false,\"independent_map\":false}\n";
  }catch(const std::exception& e){std::cerr<<"alignment inspection failed: "<<e.what()<<'\n';return 1;}
}
