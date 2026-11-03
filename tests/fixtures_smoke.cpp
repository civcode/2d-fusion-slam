#include "slam/fixtures/synthetic.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <variant>

void require(bool condition, const char* why) {
  if (!condition) { std::cerr << "FAIL: " << why << '\n'; std::exit(1); }
}
int main() {
  using namespace slam;
  using namespace slam::fixtures;
  const auto ap = apartment();
  const auto wh = warehouse();
  require(ap.id == "synthetic-apartment-v1" && wh.id == "synthetic-warehouse-v1", "scene names");
  require(wh.walls.size() > ap.walls.size(), "repeat shelving exists");
  for(const char* name:{"office","loft","atrium",
                        "industrial","warehouse_asymmetric"}) {
    const auto sample=named_scene(name);
    require(sample.id == std::string("synthetic-")+name+"-v1",
            "exact stable named-scene identifier");
    require(sample.walls.size() >= 10, "scene has nontrivial wall geometry");
    bool angled=false;
    for(const auto& wall:sample.walls) {
      const auto dx=wall.end.x()-wall.start.x();
      const auto dy=wall.end.y()-wall.start.y();
      if(std::abs(dx)>1e-6&&std::abs(dy)>1e-6)angled=true;
    }
    require(angled, "new scenes contain non-axis-aligned structure");
    const auto first=simulate_scan(sample,{{3.0,3.0},0.}, {}, {},
                                   {2'000'000'000,1,1}, 240);
    const auto again=simulate_scan(sample,{{3.0,3.0},0.}, {}, {},
                                   {2'000'000'000,1,1}, 240);
    for(std::size_t i=0;i<first.beams.size();++i)
      require(first.beams[i].range_m==again.beams[i].range_m,
              "new synthetic beam geometry is deterministic");
  }
  // The new apartment name changes the route, not the reference geometry.
  const auto ae=apartment_explore();
  require(ae.id=="synthetic-apartment_explore-v1", "versioned apartment identity");
  require(ae.walls.size()==ap.walls.size(), "apartment reference preserved");
  for(std::size_t i=0;i<ap.walls.size();++i)
    require(ae.walls[i].start==ap.walls[i].start &&
            ae.walls[i].end==ap.walls[i].end,
            "apartment wall coordinates preserved");
  const auto wk=workshop();
  require(wk.id=="synthetic-workshop-v1", "workshop identity");
  require(wk.walls.size()==23,
          "workshop exposes merged free/occupied raster contours");
  for(const auto& segment:wk.walls) {
    const auto d=segment.end-segment.start;
    require(d.norm()>.05 &&
            (std::abs(d.x())<1e-9 || std::abs(d.y())<1e-9),
            "workshop raster boundaries are nonempty and axis aligned");
    for(const auto& p:{segment.start,segment.end})
      require(std::abs(10*p.x()-std::round(10*p.x()))<1e-9 &&
              std::abs(10*p.y()-std::round(10*p.y()))<1e-9,
              "workshop walls use exact 10cm raster cell faces");
  }
  const Timestamp workshop_t{2'000'000'000,1,1};
  const Pose2 workshop_pose{{2.5,1.5},0.};
  const auto ws=simulate_scan(wk,workshop_pose,{}, {},workshop_t,240);
  const auto ws_again=simulate_scan(wk,workshop_pose,{}, {},workshop_t,240);
  require(ws.beams.size()==240, "workshop uses native 240-beam simulation");
  int hits=0;
  for(std::size_t i=0;i<ws.beams.size();++i) {
    require(ws.beams[i].range_m==ws_again.beams[i].range_m,
            "workshop laser simulation deterministic");
    hits+=ws.beams[i].hit;
  }
  require(hits>60, "workshop furniture and walls are observable");

  const Timestamp ref{2'000'000'000, 1, 1};
  const Pose2 robot{{2.0, 2.0}, 0.0};
  const auto a = simulate_scan(ap, robot, {}, {}, ref, 360);
  const auto b = simulate_scan(ap, robot, {}, {}, ref, 360);
  require(a.beams.size() == 360, "beam count");
  require(a.beams[0].acquired_at.ns == ref.ns - 50'000'000, "first beam time");
  require(a.beams.back().acquired_at.ns == ref.ns + 50'000'000, "last beam time");
  require(a.beams[0].hit && std::abs(a.beams[0].range_m - 2.0) < 1e-10, "analytic left wall");
  for (std::size_t i = 0; i < a.beams.size(); ++i) {
    require(a.beams[i].acquired_at.ns == b.beams[i].acquired_at.ns &&
      a.beams[i].range_m == b.beams[i].range_m, "deterministic exact beams");
    require(a.beams[i].range_m > 0 && a.beams[i].range_m <= 30, "valid ranges");
  }
  const auto moving = simulate_scan(ap, robot, {0.5, 0, 1.0}, {}, ref, 360);
  require(std::abs(moving.beams[0].range_m - a.beams[0].range_m) > 1e-4, "moving ray acquisition");
  const auto short_range = simulate_scan(ap, robot, {}, {}, ref, 360, 100'000'000, 0.3);
  require(!short_range.beams[0].hit && short_range.beams[0].range_m == 0.3, "no-return distinguished");
  try { (void)simulate_scan(ap, robot, {}, {}, ref, 1); require(false, "reject invalid beam count"); }
  catch (const std::invalid_argument&) {}
  std::cout << "synthetic apartment and warehouse fixture checks passed\n";
}
