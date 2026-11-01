#include "slam/datasets/fr079_train_only.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace slam::datasets;
namespace {void require(bool ok,const char* m){if(!ok){std::cerr<<"FAIL: "<<m<<'\n';std::exit(1);}}}
int main(){
 std::vector<CarmenLaser> data;
 for(int i=0;i<48;++i){
  CarmenLaser scan;
  scan.logged_laser_pose={0.03*i,0.2*std::sin(0.2*i),0.1};
  scan.ipc_seconds=100.0+0.1*i;
  for(int k=0;k<90;++k){double a=-1.57+(k+0.5)*3.14/90;
   scan.ranges.push_back({4.0+0.1*std::sin(k*0.4),a,RangeStatus::hit});}
  data.push_back(std::move(scan));
 }
 TrainOnlyOptions options;options.holdout_modulus=5;options.training_stride=6;
 options.heldout_query_stride=2;options.beam_stride=3;options.use_ceres=false;
 const auto first=build_fr079_training_map(data,options);
 require(first.training_inserted>0&&first.training_eligible==38,"training split");
 require(first.snapshot&&first.field&&first.field->source().get()==first.snapshot.get(),"immutability");
 const auto hash=first.snapshot->fingerprint();
 auto mutated=data;
 for(std::size_t i=0;i<mutated.size();i+=options.holdout_modulus){
   mutated[i].logged_laser_pose.x+=700.0;mutated[i].ranges[0].metres+=10.0;
 }
 const auto second=build_fr079_training_map(mutated,options);
 require(second.snapshot->fingerprint()==hash,"heldout changes cannot enter training map");
 require(second.training_rays==first.training_rays,"holdout independence beam counts");
 const auto r=evaluate_fr079_holdout(data,first,options);
 require(r.heldout_scans==10&&r.evaluated_queries==5,"deterministic heldout sampling");
 require(r.accepted+r.no_improvement+r.rejected_support+r.rejected_degenerate+r.invalid==r.evaluated_queries,"classified outcomes");
 require(r.training_map_fingerprint==hash,"matcher pins training map");
 require(!r.independent_ground_truth&&!r.per_beam_time_verified&&!r.reference_map_from_heldout,"honest truth flags");
#ifdef SLAM_HAS_CERES
 options.use_ceres=true;
 const auto solved=evaluate_fr079_holdout(data,first,options);
 require(solved.used_ceres&&solved.evaluated_queries==5,"Ceres path exercised");
#endif
 options.holdout_modulus=1;
 bool bad=false;try{(void)build_fr079_training_map(data,options);}catch(const std::invalid_argument&){bad=true;}
 require(bad,"invalid split fail closed");
 std::cout<<"FR079 training-only split and provenance tests passed\n";
}
