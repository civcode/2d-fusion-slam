#include "slam/graph/pose_graph.hpp"
#include "slam/core/se2.hpp"

#include <cmath>
#include <iostream>
#include <vector>

int main() {
  using namespace slam;
  using namespace slam::graph;
  const std::vector<Pose2> truth{{{0,0},0},{{2,0},0},{{2,2},0},{{0,2},0},{{0,0},0}};
  const FactorProvenance synthetic{EvidenceClass::synthetic_exact,"synthetic-square-v1",true,true};
  BatchGraphInput graph;
  graph.initial_map_from_base.push_back(truth[0]);
  for(std::size_t i=1;i<truth.size();++i) {
    const auto exact=compose(inverse(truth[i-1]),truth[i]);
    const Pose2 noisy{{exact.translation.x()+0.12,exact.translation.y()-0.06},0.02};
    graph.odometry.push_back({i-1,i,noisy,0.12,0.12,0.07,synthetic});
    graph.initial_map_from_base.push_back(compose(graph.initial_map_from_base.back(),noisy));
  }
  graph.loops.push_back({{0,4,{{0,0},0},0.04,0.04,0.04,synthetic},120,0.94,true});
  const auto result=optimize_pose_graph(graph);
  std::cout<<"{\"experiment\":\"synthetic_square_pose2\",\"nodes\":"
           <<result.optimized_map_from_base.size()<<",\"loops_accepted\":"
           <<result.accepted_loops<<",\"initial_cost\":"<<result.initial_cost
           <<",\"final_cost\":"<<result.final_cost
           <<",\"fixed_first_pose\":"<<(result.fixed_first_pose?"true":"false")
           <<",\"independent_ground_truth\":false}\n";
  return result.accepted_loops==1 && result.fixed_first_pose &&
         result.final_cost < result.initial_cost ? 0 : 1;
}
