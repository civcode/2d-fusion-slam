#include "slam/localization/known_map.hpp"
#include "slam/mapping/occupancy.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

using namespace slam;
using namespace slam::mapping;
using namespace slam::matching;
using namespace slam::localization;
using slam::sensors::ReturnKind;

namespace {
void require(bool value, const char* message) {
  if (!value) { std::cerr << "FAILED: " << message << '\n'; std::exit(1); }
}

std::pair<std::shared_ptr<const DistanceField>, std::vector<Eigen::Vector2d>> fixture() {
  GridConfig c;
  c.width=120; c.height=120; c.resolution_m=0.05;
  c.clear_trusted_no_returns=true;
  OccupancyGrid grid(c);
  std::vector<WorldRay> clearing;
  for (int y=0; y<120; ++y) {
    double yy=(y+0.5)*0.05;
    clearing.push_back({{0.025,yy},{5.975,yy},ReturnKind::no_return});
  }
  (void)grid.integrate_rays(clearing);
  std::vector<Eigen::Vector2d> points;
  for (int i=0; i<35; ++i) {
    int x=18+(i*37)%84, y=17+(i*53)%82;
    Eigen::Vector2d p{(x+0.5)*0.05,(y+0.5)*0.05};
    points.push_back(p);
    (void)grid.integrate_rays({{{0.025,0.025},p,ReturnKind::hit}});
  }
  return {DistanceField::build(MapSnapshot::capture(grid),0.6),points};
}

Anchor anchor(Pose2 base={}) {
  Anchor a;
  a.stamp={100,7,5};
  a.map_from_base=base;
  a.odom_from_base={{0.2,-0.1},0.1};
  a.calibration_revision=11;
  a.map_provenance_verified=true;
  a.extrinsics_verified=true;
  a.position_independently_anchored=true;
  return a;
}
Observation scan(const std::vector<Eigen::Vector2d>& points, TimeNs ns=100,
                 TimeNs steady=1'000'000'000) {
  Observation o;
  o.stamp={ns,7,5};
  o.received_steady_ns=steady;
  o.odom_from_base={{0.2,-0.1},0.1};
  o.laser_hits=points;
  o.calibration_revision=11;
  o.acquisition_time_verified=true;
  o.extrinsics_verified=true;
  o.odom_valid=true;
  return o;
}
} // namespace

int main() {
  const auto [field, points]=fixture();
  require(field->has_obstacles(),"test map has occupied cells");
  const Pose2 extrinsic{{0.10,0.04},0.05};
  int calls=0;
  bool mismatch=false, excessive=false, rejected=false;
  Matcher mock=[&](std::shared_ptr<const DistanceField> source,
                   const std::vector<Eigen::Vector2d>& hits, Pose2 seed,
                   const MatchSettings&) {
    ++calls;
    MatchResult m;
    m.status=rejected ? MatchStatus::insufficient_support : MatchStatus::accepted;
    m.map_from_laser=seed;
    m.map_from_laser.translation.x() += excessive ? 10.0 : 0.01;
    auto& d=m.diagnostics;
    d.input_hits=hits.size();
    d.supported_hits=hits.size();
    d.overlap=1.0;
    d.full_rank=true;
    d.robust_cost=1.0;
    d.source_revision=source->source()->revision();
    d.source_fingerprint=source->source()->fingerprint()+(mismatch?1:0);
    return m;
  };
  KnownMapTracker tracker(field,extrinsic,{},mock);
  auto frame=scan(points);
  require(!tracker.observe(frame).globally_valid(),"unanchored input has no global pose");
  auto weak=anchor(); weak.position_independently_anchored=false;
  require(!tracker.install_anchor(weak),"unverified anchor refused");
  require(tracker.state()==State::lost,"weak anchor leaves LOST");
  const Pose2 origin{{1.0,2.0},0.2};
  require(tracker.install_anchor(anchor(origin)),"explicit verified anchor accepted");
  require(tracker.state()==State::lost,"anchor alone is not tracking");
  auto e=tracker.observe(frame);
  require(e.globally_valid() && e.reason==Reason::matched && calls==1,"observed scan enters tracking");
  require(e.map_revision==field->source()->revision() &&
          e.map_fingerprint==field->source()->fingerprint(),"map identity pinned");
  require((e.map_from_base->translation-origin.translation).norm()<0.02,
          "base pose correctly composes LiDAR offset");
  require((compose(*e.map_from_odom,frame.odom_from_base).translation-
           e.map_from_base->translation).norm()<1e-9,"map/odom same-frame transform");
  require(!tracker.watchdog(1'100'000'000).globally_valid(),"watchdog does not republish stale pose");
  require(tracker.state()==State::tracking,"within freshness window stays tracking");
  e=tracker.watchdog(1'600'000'000);
  require(e.state==State::lost && e.reason==Reason::stale && !e.globally_valid(),
          "host-steady timeout invalidates pose");
  require(!tracker.observe(scan(points,101,1'700'000'000)).globally_valid(),
          "stale tracker requires a new anchor");

  require(tracker.install_anchor(anchor(origin)),"can reacquire with external anchor");
  mismatch=true;
  e=tracker.observe(scan(points));
  require(e.reason==Reason::source_mismatch && e.state==State::uncertain &&
          !e.globally_valid(),"wrong matcher fingerprint refused");
  mismatch=false; excessive=true;
  e=tracker.observe(scan(points,101,1'100'000'000));
  require(e.state==State::lost && e.reason==Reason::excessive_innovation,
          "repeated/oversized innovations force LOST");
  excessive=false;

  require(tracker.install_anchor(anchor(origin)),"re-arm after failure");
  rejected=true;
  e=tracker.observe(scan(points));
  require(e.state==State::uncertain && !e.globally_valid(),"first matcher rejection uncertain");
  e=tracker.observe(scan(points,101,1'100'000'000));
  require(e.state==State::lost && !e.globally_valid(),"rejection streak loses anchor");
  rejected=false;

  require(tracker.install_anchor(anchor(origin)),"re-arm for clock tests");
  e=tracker.observe(scan(points));
  require(e.globally_valid(),"valid update restored");
  e=tracker.observe(scan(points,100,1'100'000'000));
  require(e.reason==Reason::clock_discontinuity && e.state==State::lost,
          "out-of-order frame revokes anchor");
  require(tracker.install_anchor(anchor(origin)),"re-arm for calibration test");
  auto wrong_calibration=scan(points); wrong_calibration.calibration_revision=12;
  e=tracker.observe(wrong_calibration);
  require(e.state==State::lost && e.reason==Reason::calibration_changed,
          "calibration changes force re-anchoring");

  require(tracker.install_anchor(anchor(origin)),"re-arm for input checks");
  auto unknown_time=scan(points); unknown_time.acquisition_time_verified=false;
  e=tracker.observe(unknown_time);
  require(e.reason==Reason::invalid_evidence && e.state==State::uncertain,
          "unverified timing cannot make global pose valid");
  tracker.request_relocalization();
  require(tracker.state()==State::relocalizing,"explicit relocalization state");
  require(!tracker.observe(frame).globally_valid(),"relocalization cannot self-certify");

  Settings small; small.maximum_hits=20;
  KnownMapTracker budget(field,{},small,mock);
  require(budget.install_anchor(anchor()),"budget tracker armed");
  e=budget.observe(frame);
  require(e.reason==Reason::input_over_budget && e.state==State::uncertain,
          "unbounded scan denied before matcher");

  // Real matcher/immutable EDT integration on an exactly synthetic map.
  KnownMapTracker real(field,{});
  Anchor reference=anchor({{0.035,-0.03},0.014});
  reference.odom_from_base={};
  require(real.install_anchor(reference),"synthetic anchor installed");
  auto actual=scan(points); actual.odom_from_base={};
  e=real.observe(actual);
  require(e.globally_valid() && e.match_status==MatchStatus::accepted,
          "actual reference matcher provides accepted synthetic pose");
  require(e.map_from_base->translation.norm()<0.025 &&
          std::abs(e.map_from_base->yaw_rad)<0.008,
          "synthetic reference pose converges");

  std::cout << "read-only known-map trust and reference-matcher tests passed\n";
}
