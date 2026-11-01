#include "slam/offline/map_generation.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

using namespace slam;
using namespace slam::graph;
using namespace slam::offline;

namespace {
void check(bool ok,const char* name) {
  if (!ok) {std::cerr<<"FAIL "<<name<<'\n';std::exit(1);}
}
FactorProvenance evidence(const char* id) {
  return {EvidenceClass::synthetic_exact,id,true,true};
}
template<class Function> void rejects(Function&& f,const char* message) {
  bool threw=false;
  try {f();} catch(const std::invalid_argument&) {threw=true;}
  check(threw,message);
}
struct Fixture {
  BatchGraphInput input;
  std::vector<VerifiedKeyframe> frames;
  HeldoutReference heldout;
  PromotionOptions options;
};
Fixture fixture() {
  Fixture f;
  const std::vector<Pose2> truth{{{0,0},0},{{2,0},0},{{2,2},0},{{0,2},0},{{0,0},0}};
  f.input.initial_map_from_base.push_back(truth.front());
  for(std::size_t i=1;i<truth.size();++i){
    const auto actual=compose(inverse(truth[i-1]),truth[i]);
    const Pose2 noisy{{actual.translation.x()+0.12,actual.translation.y()-0.06},0.02};
    f.input.odometry.push_back({i-1,i,noisy,0.12,0.12,0.07,evidence("synthetic-odometry")});
    f.input.initial_map_from_base.push_back(compose(f.input.initial_map_from_base.back(),noisy));
  }
  f.input.loops.push_back({{0,4,{{0,0},0},0.04,0.04,0.04,evidence("synthetic-loop-measurement")},120,0.95,true});
  f.heldout.source=evidence("independent-synthetic-reference");
  // Known, non-collinear static landmarks. Held-out geometry has its own
  // (synthetic exact) reference source and never enters map construction.
  std::vector<Eigen::Vector2d> landmarks;
  for(int x=0;x<16;++x)for(int y=0;y<7;++y){
    landmarks.emplace_back(0.25+x*0.43+0.017*(y%3),0.5+y*0.73+0.014*(x%4));
  }
  for (std::size_t node=1;node<truth.size();++node) {
    VerifiedKeyframe frame;
    frame.node_index=node;
    frame.source=evidence("independent-mapping-lidar");
    frame.acquisition_and_deskew_verified=true;
    const auto base_from_map=inverse(truth[node]);
    for(const auto& world:landmarks){
      const auto base_point=transform(base_from_map,world);
      frame.rays.push_back({{0,0},base_point,sensors::ReturnKind::hit});
    }
    f.frames.push_back(std::move(frame));
  }
  // Independent held-out endpoint set: small fixed offset from synthetic
  // landmarks, not a duplicate of any stored scan measurement.
  for(const auto& world:landmarks)
    f.heldout.hit_endpoints_map.push_back(world+Eigen::Vector2d{0.015,-0.011});
  f.options.grid.width=120;
  f.options.grid.height=120;
  f.options.grid.resolution_m=0.1;
  f.options.grid.map_from_grid={{-2,-2},0};
  f.options.occupied_neighborhood_cells=0;
  f.options.minimum_heldout_hits=30;
  f.options.minimum_hit_fraction_gain=0.04;
  return f;
}
} // namespace

int main() {
  auto data=fixture();
  const auto result=evaluate_map_generation(data.input,data.frames,data.heldout,data.options);
  std::cout<<"synthetic comparison: incumbent="<<result.incumbent_score.hit_fraction()
           <<" candidate="<<result.candidate_score.hit_fraction()
           <<" baseline_known="<<result.incumbent_score.known_fraction()
           <<" candidate_known="<<result.candidate_score.known_fraction()
           <<" loops="<<result.graph_result.accepted_loops
           <<" decision="<<static_cast<int>(result.decision)<<'\n';
  check(result.graph_result.accepted_loops==1,"verified loop accepted");
  check(result.promoted(),"improved candidate promoted");
  check(result.authoritative==result.candidate.snapshot,"authoritative is promoted candidate");
  check(result.incumbent.snapshot!=result.candidate.snapshot,"distinct immutable generations");
  check(result.incumbent.snapshot->fingerprint()!=result.candidate.snapshot->fingerprint(),"different hashes");
  check(result.incumbent_score.attempted==112,"independent heldout sample count");
  check(result.mapped_keyframes==4 && result.inserted_rays==448,"training frame count");
  const auto again=evaluate_map_generation(data.input,data.frames,data.heldout,data.options);
  check(again.candidate.snapshot->fingerprint()==result.candidate.snapshot->fingerprint(),"deterministic candidate map");
  check(again.incumbent.snapshot->fingerprint()==result.incumbent.snapshot->fingerprint(),"deterministic incumbent map");

  // Binary map import retains exact authoritative map fingerprint and mask.
  std::stringstream archive(std::ios::in|std::ios::out|std::ios::binary);
  mapping::write_map_bundle(archive,result.candidate);
  archive.seekg(0);
  auto reloaded=mapping::read_map_bundle(archive);
  check(reloaded.snapshot->fingerprint()==result.candidate.snapshot->fingerprint(),"candidate binary roundtrip");

  auto no_loop=data;
  no_loop.input.loops.clear();
  const auto unchanged=evaluate_map_generation(no_loop.input,no_loop.frames,no_loop.heldout,no_loop.options);
  check(!unchanged.promoted()&&unchanged.decision==PromotionDecision::no_accepted_loop,"no-loop retains baseline");
  check(unchanged.authoritative==unchanged.incumbent.snapshot,"no-loop rollback identity");

  auto false_loop=data;
  false_loop.input.loops.front().edge.from_to={{50,50},2};
  const auto rejected=evaluate_map_generation(false_loop.input,false_loop.frames,false_loop.heldout,false_loop.options);
  check(rejected.graph_result.accepted_loops==0&&!rejected.promoted(),"bad loop is not mapped");
  check(rejected.authoritative==rejected.incumbent.snapshot,"rejected loop keeps incumbent");

  // A plausible but biased odometry graph can pull originally correct
  // poses away from truth; a lower factor cost must not force map promotion.
  auto degraded=data;
  degraded.input.initial_map_from_base={{{0,0},0},{{2,0},0},{{2,2},0},{{0,2},0},{{0,0},0}};
  degraded.input.loops[0].edge.from_to={{1.8,0.5},0};
  degraded.input.loops[0].edge.sigma_x_m=0.01;
  degraded.input.loops[0].edge.sigma_y_m=0.01;
  const auto degraded_result=evaluate_map_generation(degraded.input,degraded.frames,
                                                       degraded.heldout,degraded.options);
  std::cout<<"degraded test: before="<<degraded_result.incumbent_score.hit_fraction()
           <<" after="<<degraded_result.candidate_score.hit_fraction()
           <<" decision="<<static_cast<int>(degraded_result.decision)<<'\n';
  check(!degraded_result.promoted(),"degraded candidate must not replace correct baseline");
  check(degraded_result.authoritative==degraded_result.incumbent.snapshot,
        "degraded candidate rollback identity");

  auto strict=data;
  strict.options.minimum_hit_fraction_gain=0.99;
  const auto strict_result=evaluate_map_generation(strict.input,strict.frames,strict.heldout,strict.options);
  check(strict_result.decision==PromotionDecision::insufficient_improvement,"failed quality gate rolls back");
  check(strict_result.authoritative==strict_result.incumbent.snapshot,"strict rollback identity");

  // Modifying held-out measurements cannot change either map's content.
  auto mutated=data;
  for(auto& p:mutated.heldout.hit_endpoints_map)p.x()+=3;
  const auto holdout_changed=evaluate_map_generation(mutated.input,mutated.frames,mutated.heldout,mutated.options);
  check(holdout_changed.incumbent.snapshot->fingerprint()==result.incumbent.snapshot->fingerprint(),"reference never inserted into incumbent");
  check(holdout_changed.candidate.snapshot->fingerprint()==result.candidate.snapshot->fingerprint(),"reference never inserted into candidate");

  auto bad_clock=data;
  bad_clock.frames[0].acquisition_and_deskew_verified=false;
  rejects([&]{(void)evaluate_map_generation(bad_clock.input,bad_clock.frames,bad_clock.heldout,bad_clock.options);},"unverified timing refused");
  auto double_count=data;
  double_count.input.loops[0].edge.provenance.source_id=double_count.frames[0].source.source_id;
  rejects([&]{(void)evaluate_map_generation(double_count.input,double_count.frames,double_count.heldout,double_count.options);},"correlated mapping/loop evidence refused");
  auto overlapping_odometry=data;
  overlapping_odometry.input.odometry[0].provenance.source_id=data.frames[0].source.source_id;
  rejects([&]{(void)evaluate_map_generation(overlapping_odometry.input,overlapping_odometry.frames,
                                             overlapping_odometry.heldout,overlapping_odometry.options);},
          "odometry cannot reuse mapping observation source");
  auto overlapping_loop=data;
  overlapping_loop.input.loops[0].edge.provenance.source_id=data.input.odometry[0].provenance.source_id;
  rejects([&]{(void)evaluate_map_generation(overlapping_loop.input,overlapping_loop.frames,
                                             overlapping_loop.heldout,overlapping_loop.options);},
          "loops cannot reuse odometry source");
  auto no_truth=data;
  no_truth.heldout.source.source=EvidenceClass::unverified;
  rejects([&]{(void)evaluate_map_generation(no_truth.input,no_truth.frames,no_truth.heldout,no_truth.options);},"unverified heldout truth refused");

  matching::MatchResult match;
  match.status=matching::MatchStatus::accepted;
  match.diagnostics.source_fingerprint=12345;
  match.diagnostics.input_hits=100;
  match.diagnostics.supported_hits=90;
  match.diagnostics.overlap=0.90;
  match.diagnostics.full_rank=true;
  auto loop=qualified_frontend_loop(data.input.loops[0].edge,match,12345);
  check(loop.supported_hits==90 && loop.full_rank,"separate measurement and match diagnostics joined");
  rejects([&]{(void)qualified_frontend_loop(data.input.loops[0].edge,match,987);},"stale map match refused");
  auto untrusted=data.input.loops[0].edge;
  untrusted.provenance.independent_of_initial_trajectory=false;
  rejects([&]{(void)qualified_frontend_loop(untrusted,match,12345);},"correlated tracking pose not a loop factor");
  std::cout<<"graph-to-map promotion/rollback and source provenance passed\n";
}
