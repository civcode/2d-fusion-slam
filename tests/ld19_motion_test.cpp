#include "slam/sensors/ld19.hpp"
#include "slam/motion/trajectory.hpp"
#include "slam/fixtures/synthetic.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <variant>
#include <vector>

using namespace slam;
using namespace slam::sensors;
using namespace slam::motion;

namespace {
void check(bool truth, const char* detail) {
  if (!truth) { std::cerr << "FAIL: " << detail << '\n'; std::exit(1); }
}
bool close(double a, double b, double e=1e-7) { return std::abs(a-b)<e; }
void le16(Ld19Bytes& bytes, std::size_t i, std::uint16_t v) {
  bytes[i]=static_cast<std::uint8_t>(v&255);
  bytes[i+1]=static_cast<std::uint8_t>(v>>8);
}
Ld19Bytes make_frame(int start=35000, int end=1000, int ms=29998) {
  Ld19Bytes p{};
  p[0]=0x54; p[1]=0x2c;
  le16(p,2,3600);
  le16(p,4,static_cast<std::uint16_t>(start));
  for(std::size_t i=0;i<kLd19PointsPerFrame;++i) {
    le16(p,6+3*i,static_cast<std::uint16_t>(1500+10*i));
    p[8+3*i]=static_cast<std::uint8_t>(i+10);
  }
  le16(p,42,static_cast<std::uint16_t>(end));
  le16(p,44,static_cast<std::uint16_t>(ms));
  p[46]=ld19_crc8(std::span<const std::uint8_t>(p.data(),46));
  return p;
}
}
int main() {
  // Published CRC-8 polynomial 0x4D check vector, independently usable
  // regardless of the synthesized packet generation helper.
  constexpr std::array<std::uint8_t,9> ascii{'1','2','3','4','5','6','7','8','9'};
  check(ld19_crc8(ascii)==0xc3, "CRC8 independent check vector");
  const auto raw=make_frame();
  Ld19Packet decoded;
  check(decode_ld19(raw,decoded)==Ld19FrameStatus::valid,"frame decode");
  check(decoded.raw==raw && decoded.speed_degrees_per_second==3600 &&
        decoded.start_angle_centidegrees==35000 && decoded.end_angle_centidegrees==1000 &&
        decoded.device_timestamp_ms==29998,"decoded header, angles and raw provenance");
  check(decoded.points[0].distance_mm==1500 && decoded.points[11].intensity==21,
        "little-endian point ranges and intensities");
  auto broken=raw;
  broken[7]^=0x20;
  check(decode_ld19(broken,decoded)==Ld19FrameStatus::bad_crc, "CRC corruption rejected");
  broken=raw; broken[1]=0x0c;
  check(decode_ld19(broken,decoded)==Ld19FrameStatus::bad_format,"wrong verlen rejected");
  broken=raw; le16(broken,44,40000);
  check(decode_ld19(broken,decoded)==Ld19FrameStatus::bad_format,"counter outside protocol rejected");

  Ld19StreamParser stream;
  std::vector<std::uint8_t> first{0x11,0x54,0x00,0x12,0x54,0x2c};
  // Make actual first packet split after its 2-byte header. The prefix
  // contains a nonmatching header and must be discarded.
  first.insert(first.end(),raw.begin()+2,raw.begin()+24);
  const Timestamp arrival_a{100,3,5}, arrival_b{200,3,5};
  check(stream.push(first,arrival_a).empty(),"fragment held until complete");
  auto out=stream.push(std::span<const std::uint8_t>(raw.data()+24,raw.size()-24),arrival_b);
  check(out.size()==1 && out[0].status==Ld19FrameStatus::valid && out[0].packet &&
        out[0].last_byte_received_at.ns==arrival_b.ns && out[0].raw==raw,
        "resynchronized fragmented frame with last-byte receipt time");
  check(stream.discarded_bytes()==4,"garbage correctly discarded");
  stream.reset();
  std::vector<std::uint8_t> corruption(broken.begin(),broken.end()); // malformed counter
  corruption.insert(corruption.end(),raw.begin(),raw.end());
  const auto recovered=stream.push(corruption,arrival_b);
  check(recovered.size()>=2 && recovered.front().status==Ld19FrameStatus::bad_format &&
        recovered.back().status==Ld19FrameStatus::valid,"bad frame reported and stream recovers");
  auto badcrc=raw; badcrc[10]^=0x33;
  stream.reset();
  const auto events=stream.push(badcrc,arrival_b);
  check(events.size()>=1 && events[0].status==Ld19FrameStatus::bad_crc &&
        events[0].raw==badcrc,"CRC failed raw frame preserved");
  stream.reset();
  check(stream.push(std::span<const std::uint8_t>(raw.data(),10),arrival_a).empty(),"partial epoch");
  bool caught=false;
  try { (void)stream.push(std::span<const std::uint8_t>(raw.data()+10,37),Timestamp{201,3,6}); }
  catch(const std::invalid_argument&){ caught=true; }
  check(caught,"rejected cross-epoch reassembly");
  check(stream.push(raw,Timestamp{202,3,6}).size()==1,"restart after explicit epoch reset");

  Ld19CounterUnwrapper unwrap(100);
  check(unwrap.observe(29980).status==Ld19CounterStatus::first,"counter initial");
  const auto wrap=unwrap.observe(12);
  check(wrap.status==Ld19CounterStatus::wrapped && wrap.unwrapped_ms==32,
        "nominal 30-second rollover");
  check(unwrap.observe(30).unwrapped_ms==50,"counter post-rollover continuous");
  check(unwrap.observe(29).status==Ld19CounterStatus::discontinuity,"backward jump rejected");
  check(unwrap.observe(30).status==Ld19CounterStatus::discontinuity,"counter fails closed");
  unwrap.reset();
  check(unwrap.observe(30000).status==Ld19CounterStatus::first,"upper limit accepted");
  check(unwrap.observe(0).status==Ld19CounterStatus::wrapped,"exact upper-limit wrap accepted");
  unwrap.reset();
  check(unwrap.observe(0).status==Ld19CounterStatus::first,"new device epoch after reset");
  check(unwrap.observe(500).status==Ld19CounterStatus::discontinuity,"long gap ambiguous");

  Ld19Packet packet;
  check(decode_ld19(raw,packet)==Ld19FrameStatus::valid,"decode again");
  const auto relative=ld19_relative_beams(packet);
  check(relative && relative->reference_unverified,"provisional beam model marked");
  check(close(relative->beams.front().angle_rad,350*std::numbers::pi/180) &&
        close(relative->beams.back().angle_rad,10*std::numbers::pi/180),"angles interpolate across seam");
  check(relative->beams.front().offset_from_last_beam_ns==-5'555'556 &&
        relative->beams.back().offset_from_last_beam_ns==0,"beam offsets from motor speed");
  packet.speed_degrees_per_second=0;
  check(!ld19_relative_beams(packet),"stopped rotor no time extrapolation");
  packet.speed_degrees_per_second=3600;
  const auto anchor=anchor_ld19_beams(*relative,{{10'000'000'000LL,7,11},Ld19AnchorQuality::provisional,42,3});
  check(anchor && (*anchor)[0].time_quality==BeamTimeQuality::provisional &&
        (*anchor)[11].acquired_at.ns==10'000'000'000LL && (*anchor)[0].source_id==42,
        "external anchor explicit provisional quality");
  check(!anchor_ld19_beams(*relative,{{10'000'000'000LL,7,11},
        Ld19AnchorQuality::calibrated,42,3}),
        "physical reference cannot be upgraded by quality label alone");
  const auto calibrated=anchor_ld19_beams(*relative,{{10'000'000'000LL,7,11},
        Ld19AnchorQuality::calibrated,42,3,true});
  check(calibrated && (*calibrated)[0].time_quality==BeamTimeQuality::calibrated,
        "explicit validated physical time anchor");
  const auto overflow=anchor_ld19_beams(*relative,{{std::numeric_limits<TimeNs>::min(),7,11},Ld19AnchorQuality::calibrated,42,3,true});
  check(!overflow,"checked nanosecond anchor overflow");

  // Sampled body-frame trajectory and off-center LiDAR lever-arm deskew.
  const Timestamp t0{0,3,2}, tmid{500'000'000,3,2}, t1{1'000'000'000,3,2};
  const Pose2 start{{1,2},0}, finish=integrate_body_twist(start,{1,0,1},1.0);
  const SampledTrajectory traj({{t0,start},{t1,finish}},TrajectoryCapability::full_se2);
  const auto half=traj.at(tmid);
  const auto expected=integrate_body_twist(start,{1,0,1},0.5);
  check(half && (half->translation-expected.translation).norm()<1e-9 &&
        close(half->yaw_rad,expected.yaw_rad),"continuous SE2 twist interpolation");
  check(!traj.at({-1,3,2}) && !traj.at({0,3,3}) && !traj.at({1'000'000'001,3,2}),
        "no extrapolation or epoch crossing");
  caught=false;
  try { (void)SampledTrajectory({{t0,start},{t1,finish}},TrajectoryCapability::gyro_only); }
  catch(const std::invalid_argument&){caught=true;}
  check(caught,"gyro-only cannot fabricate XY motion");
  const Pose2 laser{{0.23,0.07},0.15};
  const Eigen::Vector2d static_point_odom{6,4};
  AssembledScan moving;
  moving.time_quality=BeamTimeQuality::synthetic_exact;
  moving.first_acquired=t0; moving.last_acquired=t1; moving.midpoint=tmid;
  for(const auto t : {t0,tmid,t1}) {
    const auto base=traj.at(t).value();
    const auto sensor=compose(base,laser);
    const auto relative_point=transform(inverse(sensor),static_point_odom);
    moving.beams.push_back({t,std::atan2(relative_point.y(),relative_point.x()),
                            relative_point.norm(),ReturnKind::hit,42,3,BeamTimeQuality::synthetic_exact});
  }
  moving.beams.push_back({tmid,0,0,ReturnKind::invalid,42,3,BeamTimeQuality::synthetic_exact});
  const auto deskewed=deskew_hits(moving,traj,laser);
  check(deskewed && deskewed->suitable_for_moving_map_insertion() &&
        deskewed->hits.size()==3 && deskewed->skipped_non_hits==1,
        "full SE2 scan deskew quality and invalid filtering");
  const auto point_at_mid=transform(inverse(compose(expected,laser)),static_point_odom);
  for(const auto& hit:deskewed->hits)
    check((hit.point_laser_reference-point_at_mid).norm()<1e-8,"moving rays align to stationary scene");
  moving.time_quality=BeamTimeQuality::provisional;
  check(!deskew_hits(moving,traj,laser),"unverified LD19 anchor not mapping-ready");
  const auto diagnostics=deskew_hits(moving,traj,laser,true);
  check(diagnostics && !diagnostics->suitable_for_moving_map_insertion(),
        "provisional timing diagnostic cannot silently enable mapping");
  const SampledTrajectory gyro({{t0,{{0,0},0}},{t1,{{0,0},1.0}}},TrajectoryCapability::gyro_only);
  moving.time_quality=BeamTimeQuality::calibrated;
  const auto yawonly=deskew_hits(moving,gyro,laser);
  check(yawonly && !yawonly->suitable_for_moving_map_insertion(),
        "gyro-only trajectory cannot assert translational deskew support");
  moving.beams[0].acquired_at={0,4,2};
  check(!deskew_hits(moving,traj,laser),"mixed-domain beams rejected by time coverage");
  std::cout << "LD19 packet/clock and supported motion/deskew checks passed\n";
}
