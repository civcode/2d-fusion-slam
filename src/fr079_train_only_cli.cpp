#include "slam/datasets/fr079_train_only.hpp"
#include <zlib.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
namespace {
std::string load(const std::string& path){
  if(path.ends_with(".gz")){
    gzFile f=gzopen(path.c_str(),"rb");if(!f)throw std::runtime_error("cannot open dataset");
    std::string content;char buffer[8192];int count=0;
    while((count=gzread(f,buffer,sizeof(buffer)))>0){if(content.size()+static_cast<std::size_t>(count)>128*1024*1024){gzclose(f);throw std::runtime_error("dataset too large");}content.append(buffer,static_cast<std::size_t>(count));}
    int result=gzclose(f);if(count<0||result!=Z_OK)throw std::runtime_error("gzip error");return content;
  }
  std::ifstream f(path);if(!f)throw std::runtime_error("cannot open dataset");
  std::ostringstream ss;ss<<f.rdbuf();auto data=ss.str();if(data.size()>128*1024*1024)throw std::runtime_error("dataset too large");return data;
}
}
int main(int argc,char** argv){
 if((argc!=2&&argc!=3)||(argc==3&&std::string(argv[2])!="--reference")){std::cerr<<"Usage: slam_fr079_train_only_benchmark <fr079.log.gz> [--reference]\n";return 2;}
 try {
  const auto data=load(argv[1]);
  std::istringstream in(data);std::vector<slam::datasets::CarmenLaser> scans;
  slam::datasets::CarmenReader reader;std::string line;
  while(std::getline(in,line))if(auto scan=reader.parse_laser(line))scans.push_back(std::move(*scan));
  slam::datasets::TrainOnlyOptions options;
  options.use_ceres=(argc==2);
  const auto map=slam::datasets::build_fr079_training_map(scans,options);
  const auto r=slam::datasets::evaluate_fr079_holdout(scans,map,options);
  std::cout<<"{\"type\":\"fr079_train_only_proxy\",\"dataset_scans\":"<<r.input_scans
    <<",\"training_eligible\":"<<r.training_eligible<<",\"training_inserted\":"<<r.training_inserted
    <<",\"heldout\":"<<r.heldout_scans<<",\"evaluated\":"<<r.evaluated_queries
    <<",\"accepted\":"<<r.accepted<<",\"no_improvement\":"<<r.no_improvement
    <<",\"rejected_support\":"<<r.rejected_support<<",\"rejected_degenerate\":"<<r.rejected_degenerate
    <<",\"initial_overlap\":"<<r.mean_initial_overlap<<",\"final_overlap\":"<<r.mean_final_overlap
    <<",\"initial_robust_cost\":"<<r.mean_initial_robust_cost<<",\"final_robust_cost\":"<<r.mean_final_robust_cost
    <<",\"mean_displacement_from_logged_pose_m\":"<<r.mean_displacement_from_logged_seed_m
    <<",\"solver\":\""<<(r.used_ceres?"ceres":"reference")<<"\""
    <<",\"map_fingerprint\":"<<r.training_map_fingerprint
    <<",\"independent_truth\":false,\"map_uses_heldout\":false,\"per_beam_time_verified\":false}\n";
 }catch(const std::exception& e){std::cerr<<"FR079 training-only benchmark error: "<<e.what()<<'\n';return 1;}
}
