#include "slam/datasets/fr079_train_only.hpp"
#include "slam/mapping/map_bundle.hpp"

#include <zlib.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>

namespace {
std::string read_gzip(const std::string& path) {
  gzFile f=gzopen(path.c_str(),"rb");
  if(!f)throw std::runtime_error("cannot open CARMEN dataset");
  std::string result;char bytes[8192];int n=0;
  while((n=gzread(f,bytes,sizeof(bytes)))>0){
    if(result.size()+static_cast<std::size_t>(n)>128*1024*1024) {
      gzclose(f);throw std::runtime_error("dataset exceeds size limit");
    }
    result.append(bytes,static_cast<std::size_t>(n));
  }
  const auto rc=gzclose(f);
  if(n<0||rc!=Z_OK)throw std::runtime_error("invalid gzip content");
  return result;
}
void verify_and_export(const std::string& filename,const std::filesystem::path& prefix){
  const auto data=read_gzip(filename);
  slam::datasets::CarmenReader reader;
  std::istringstream input(data);
  std::string line;std::vector<slam::datasets::CarmenLaser> scans;
  while(std::getline(input,line)) {
    if(auto scan=reader.parse_laser(line))scans.push_back(std::move(*scan));
  }
  const auto training=slam::datasets::build_fr079_training_map(scans);
  const slam::mapping::MapBundle bundle{training.snapshot,
    {"fr079_logged_laser_frame_UNVERIFIED", "fr079-training-index-mod5-mod6-v1",
     "logged_pose_estimate_not_ground_truth",false}};
  const auto binary=prefix.string()+".slammap";
  const auto image=prefix.string()+".pgm";
  const auto yaml=prefix.string()+".yaml";
  const auto image_basename=std::filesystem::path(image).filename().string();
  // Validate YAML name before creating any files, to avoid partial output
  // caused by an unsafe output basename.
  std::ostringstream yaml_content;
  slam::mapping::write_map_yaml(yaml_content,*training.snapshot,image_basename);
  {
    std::ofstream output(binary,std::ios::binary);
    if(!output)throw std::runtime_error("cannot create binary map bundle");
    slam::mapping::write_map_bundle(output,bundle);
  }
  {
    std::ifstream input_file(binary,std::ios::binary);
    if(!input_file)throw std::runtime_error("cannot reopen map for integrity check");
    const auto verified=slam::mapping::read_map_bundle(input_file);
    if(verified.snapshot->fingerprint()!=training.snapshot->fingerprint())
      throw std::runtime_error("exported map fingerprint mismatch");
  }
  {
    std::ofstream output(image,std::ios::binary);
    if(!output)throw std::runtime_error("cannot create PGM");
    slam::mapping::write_map_pgm(output,*training.snapshot);
  }
  {
    std::ofstream output(yaml);
    if(!output)throw std::runtime_error("cannot create YAML");
    output<<yaml_content.str();
    if(!output)throw std::runtime_error("failed to write YAML");
  }
  std::cout<<"{\"format\":\"FUSMAP2D-v1\",\"training_scans\":"<<training.training_inserted
           <<",\"map_fingerprint\":"<<training.snapshot->fingerprint()
           <<",\"metric_origin_verified\":false,\"independent_pose_truth\":false}\n";
}
}
int main(int argc,char** argv){
 if(argc!=3){std::cerr<<"Usage: slam_fr079_map_export <fr079.log.gz> <output-prefix>\n";return 2;}
 try{verify_and_export(argv[1],argv[2]);}
 catch(const std::exception& e){std::cerr<<"Map export failed: "<<e.what()<<'\n';return 1;}
}
