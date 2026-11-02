#include "slam/cli/parse.hpp"
#include "slam/mapping/map_bundle.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>

int main(int argc,char** argv){
  slam::cli::Positionals parsed;
  try {
    parsed=slam::cli::parse_positionals(argc,argv,"slam_map_inspect",
        "<map.slammap>",1,1);
  }catch(const std::exception& e) {
    std::cerr<<e.what()<<'\n';return 2;
  }
  if(parsed.help){std::cout<<parsed.usage;return 0;}
  const auto& cli_args=parsed.values;
  try{
  std::ifstream in(cli_args[0],std::ios::binary);
  if(!in)throw std::runtime_error("cannot open binary map bundle");
  const auto map=slam::mapping::read_map_bundle(in);
  const auto& cells=map.snapshot->cells();
  std::size_t known=0;
  for(const auto& cell:cells)if(cell.observed)++known;
  std::cout<<"{\"format\":\"FUSMAP2D-v1\",\"frame_id\":\""
           <<map.provenance.frame_id<<"\",\"source_id\":\""
           <<map.provenance.source_id<<"\",\"truth_class\":\""
           <<map.provenance.truth_class<<"\",\"width\":"
           <<map.snapshot->config().width<<",\"height\":"<<map.snapshot->config().height
           <<",\"known\":"<<known<<",\"unknown\":"<<cells.size()-known
           <<",\"revision\":"<<map.snapshot->revision()
           <<",\"fingerprint\":"<<map.snapshot->fingerprint()
           <<",\"metric_origin_verified\":"
           <<(map.provenance.metric_origin_verified?"true":"false")<<"}\n";
 }catch(const std::exception& e){std::cerr<<"Map import failed: "<<e.what()<<'\n';return 1;}
}
