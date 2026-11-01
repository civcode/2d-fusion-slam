#include "slam/graph/pose_graph.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace slam;
using namespace slam::graph;
namespace {
void require(bool b, const char* m) {if(!b){std::cerr<<"FAIL "<<m<<'\n';std::exit(1);}}
FactorProvenance synthetic() {return {EvidenceClass::synthetic_exact,"synthetic-v1",true,true};}
PoseEdge edge(std::size_t i,std::size_t j,Pose2 delta,double s=0.1) {
  return {i,j,delta,s,s,0.07,synthetic()};
}
}
int main(){
  const std::vector<Pose2> truth{{{0,0},0},{{2,0},0},{{2,2},0},{{0,2},0},{{0,0},0}};
  BatchGraphInput in;
  in.initial_map_from_base.push_back(truth[0]);
  for(std::size_t i=1;i<truth.size();++i){
    const auto actual=compose(inverse(truth[i-1]),truth[i]);
    const Pose2 noisy{{actual.translation.x()+0.12,actual.translation.y()-0.06},0.02};
    in.odometry.push_back(edge(i-1,i,noisy,0.12));
    in.initial_map_from_base.push_back(compose(in.initial_map_from_base.back(),noisy));
  }
  auto correct=edge(0,4,{{0,0},0},0.04);
  in.loops.push_back({correct,120,0.94,true});
  auto false_loop=edge(0,3,{{70,40},1.5},0.01);
  in.loops.push_back({false_loop,120,0.95,true});
  auto untrusted=edge(1,4,{{-2,0},0});
  untrusted.provenance.independent_of_initial_trajectory=false;
  in.loops.push_back({untrusted,120,0.95,true});
  auto weak=edge(0,4,{{0,0},0});
  in.loops.push_back({weak,2,0.1,false}); // duplicate is rejected first
  const double drift=in.initial_map_from_base.back().translation.norm();
  const auto result=optimize_pose_graph(in);
  require(result.optimized_map_from_base.size()==5 && result.fixed_first_pose,"gauge and nodes");
  require(result.loop_decisions.size()==4 && result.accepted_loops==1,"only sound loop accepted");
  require(result.loop_decisions[0]==LoopDecision::accepted,"real loop accepted");
  require(result.loop_decisions[1]==LoopDecision::inconsistent_with_initial_path,"false loop denied");
  require(result.loop_decisions[2]==LoopDecision::unverified_or_correlated_evidence,"correlated loop denied");
  require(result.loop_decisions[3]==LoopDecision::duplicate_or_nearby_nodes,"duplicate denied");
  require(result.optimized_map_from_base.back().translation.norm()<drift*0.3,"loop reduces drift");
  require(result.final_cost < result.initial_cost && !result.independent_ground_truth,"cost/no truth");
  const auto second=optimize_pose_graph(in);
  require(std::abs(second.final_cost-result.final_cost)<1e-8,"deterministic optimization");
  auto measured=in;
  measured.loops.clear();
  auto shared=edge(0,4,{{0,0},0});
  shared.provenance.source=EvidenceClass::independently_measured;
  measured.loops.push_back({shared,120,0.95,true});
  const auto no_double=optimize_pose_graph(measured);
  require(no_double.accepted_loops==0 &&
          no_double.loop_decisions[0]==LoopDecision::unverified_or_correlated_evidence,
          "loop sharing odometry source rejected");
  measured.loops.clear();
  measured.loops.push_back({weak,2,0.1,false});
  measured.loops.push_back({correct,120,0.95,true});
  const auto weak_result=optimize_pose_graph(measured);
  require(weak_result.loop_decisions[0]==LoopDecision::insufficient_geometry &&
          weak_result.loop_decisions[1]==LoopDecision::accepted,
          "unsupported proposal must not poison subsequent valid loop");
  in.odometry[0].provenance.source=EvidenceClass::unverified;
  bool denied=false;try{(void)optimize_pose_graph(in);}catch(const std::invalid_argument&){denied=true;}
  require(denied,"unverified odometry cannot become a graph factor");
  std::cout<<"Pose2 batch graph and false-loop guards passed\n";
}
