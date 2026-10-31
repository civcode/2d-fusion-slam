#include "slam/sensors/ld19.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <variant>

namespace slam::sensors {
namespace {
constexpr std::uint16_t kDevicePeriodMs = 30000;
constexpr double kRadiansPerCentidegree = std::numbers::pi / 18000.0;
std::uint16_t le16(const Ld19Bytes& raw, std::size_t offset) noexcept {
  return static_cast<std::uint16_t>(raw[offset]) |
         static_cast<std::uint16_t>(raw[offset + 1]) << 8;
}
}

std::uint8_t ld19_crc8(std::span<const std::uint8_t> input) noexcept {
  std::uint8_t crc = 0;
  for (const auto b : input) {
    crc ^= b;
    for (int i = 0; i < 8; ++i) {
      crc = (crc & 0x80) != 0 ? static_cast<std::uint8_t>((crc << 1) ^ 0x4d)
                              : static_cast<std::uint8_t>(crc << 1);
    }
  }
  return crc;
}

Ld19FrameStatus decode_ld19(const Ld19Bytes& raw, Ld19Packet& output) noexcept {
  if (raw[0] != 0x54 || raw[1] != 0x2c || le16(raw, 4) >= 36000 ||
      le16(raw, 42) >= 36000 || le16(raw, 44) > kDevicePeriodMs) {
    return Ld19FrameStatus::bad_format;
  }
  if (ld19_crc8(std::span<const std::uint8_t>(raw.data(), raw.size() - 1)) != raw.back()) {
    return Ld19FrameStatus::bad_crc;
  }
  Ld19Packet decoded;
  decoded.raw = raw;
  decoded.speed_degrees_per_second = le16(raw, 2);
  decoded.start_angle_centidegrees = le16(raw, 4);
  decoded.end_angle_centidegrees = le16(raw, 42);
  decoded.device_timestamp_ms = le16(raw, 44);
  for (std::size_t i = 0; i < kLd19PointsPerFrame; ++i) {
    decoded.points[i] = {le16(raw, 6 + 3 * i), raw[8 + 3 * i]};
  }
  output = decoded;
  return Ld19FrameStatus::valid;
}

std::vector<Ld19ReceivedFrame> Ld19StreamParser::push(
    std::span<const std::uint8_t> bytes, Timestamp receipt) {
  if (active_receive_clock_ &&
      (active_receive_clock_->clock_domain != receipt.clock_domain ||
       active_receive_clock_->epoch != receipt.epoch ||
       receipt.ns < active_receive_clock_->ns)) {
    // Never label partial data from two different clock epochs as one frame.
    reset();
    throw std::invalid_argument("LD19 receive clock discontinuity: parser reset");
  }
  active_receive_clock_ = receipt;
  std::vector<Ld19ReceivedFrame> output;
  for (auto value : bytes) {
    buffer_.push_back(value);
    while (!buffer_.empty()) {
      if (buffer_[0] != 0x54) {
        buffer_.erase(buffer_.begin());
        ++discarded_bytes_;
        continue;
      }
      if (buffer_.size() < 2) break;
      if (buffer_[1] != 0x2c) {
        buffer_.erase(buffer_.begin());
        ++discarded_bytes_;
        continue;
      }
      if (buffer_.size() < kLd19FrameSize) break;
      Ld19Bytes raw{};
      std::copy_n(buffer_.begin(), kLd19FrameSize, raw.begin());
      Ld19Packet packet;
      const auto status = decode_ld19(raw, packet);
      Ld19ReceivedFrame frame{raw, status, std::nullopt, receipt};
      if (status == Ld19FrameStatus::valid) {
        frame.packet = packet;
        buffer_.erase(buffer_.begin(), buffer_.begin() + kLd19FrameSize);
      } else {
        buffer_.erase(buffer_.begin());
        ++discarded_bytes_;
      }
      output.push_back(std::move(frame));
    }
  }
  return output;
}

void Ld19StreamParser::reset() noexcept {
  buffer_.clear();
  active_receive_clock_.reset();
}

Ld19CounterUnwrapper::Ld19CounterUnwrapper(std::uint16_t max_gap_ms)
    : max_gap_ms_(max_gap_ms) {
  if (max_gap_ms == 0 || max_gap_ms >= 1000) {
    throw std::invalid_argument("LD19 maximum interpacket gap must be in [1,999] ms");
  }
}

Ld19CounterReading Ld19CounterUnwrapper::observe(std::uint16_t raw_ms) noexcept {
  if (raw_ms > kDevicePeriodMs) { faulted_ = true; return {Ld19CounterStatus::invalid, elapsed_ms_}; }
  if (faulted_) return {Ld19CounterStatus::discontinuity, elapsed_ms_};
  if (!previous_) { previous_ = raw_ms; return {Ld19CounterStatus::first, 0}; }
  bool wrap = false;
  std::uint32_t delta = 0;
  if (raw_ms >= *previous_) {
    delta = raw_ms - *previous_;
  } else if (*previous_ >= kDevicePeriodMs - max_gap_ms_ && raw_ms <= max_gap_ms_) {
    wrap = true;
    delta = static_cast<std::uint32_t>(raw_ms) + kDevicePeriodMs - *previous_;
  } else {
    faulted_ = true;
    return {Ld19CounterStatus::discontinuity, elapsed_ms_};
  }
  if (delta > max_gap_ms_) {
    faulted_ = true;
    return {Ld19CounterStatus::discontinuity, elapsed_ms_};
  }
  previous_ = raw_ms;
  elapsed_ms_ += delta;
  return {wrap ? Ld19CounterStatus::wrapped : Ld19CounterStatus::continuous, elapsed_ms_};
}

void Ld19CounterUnwrapper::reset() noexcept {
  previous_.reset();
  elapsed_ms_ = 0;
  faulted_ = false;
}

std::optional<Ld19RelativeScan> ld19_relative_beams(
    const Ld19Packet& packet, double max_packet_duration_s) noexcept {
  if (packet.speed_degrees_per_second == 0 || !std::isfinite(max_packet_duration_s) ||
      max_packet_duration_s <= 0) return std::nullopt;
  const int span = (static_cast<int>(packet.end_angle_centidegrees) -
                    static_cast<int>(packet.start_angle_centidegrees) + 36000) % 36000;
  if (span == 0) return std::nullopt;
  const double duration_s = static_cast<double>(span) / 100.0 /
                            static_cast<double>(packet.speed_degrees_per_second);
  if (!std::isfinite(duration_s) || duration_s > max_packet_duration_s) return std::nullopt;
  Ld19RelativeScan result;
  for (std::size_t i = 0; i < kLd19PointsPerFrame; ++i) {
    const double fraction = static_cast<double>(i) / (kLd19PointsPerFrame - 1);
    const double cd = std::fmod(packet.start_angle_centidegrees + span * fraction, 36000.0);
    const double offset = (fraction - 1.0) * duration_s * 1e9;
    const auto& p = packet.points[i];
    result.beams[i] = {cd * kRadiansPerCentidegree, p.distance_mm * 0.001,
                       p.intensity, p.distance_mm == 0 ? ReturnKind::invalid : ReturnKind::hit,
                       static_cast<TimeNs>(std::llround(offset))};
  }
  return result;
}

std::optional<std::array<BeamObservation, kLd19PointsPerFrame>> anchor_ld19_beams(
    const Ld19RelativeScan& relative, Ld19TimeAnchor anchor) noexcept {
  if (anchor.quality != Ld19AnchorQuality::provisional &&
      !anchor.physical_reference_validated) return std::nullopt;
  std::array<BeamObservation, kLd19PointsPerFrame> output{};
  for (std::size_t i = 0; i < output.size(); ++i) {
    const auto time = advance(anchor.last_beam_time, DurationNs{relative.beams[i].offset_from_last_beam_ns});
    if (!std::holds_alternative<Timestamp>(time)) return std::nullopt;
    output[i] = {std::get<Timestamp>(time), relative.beams[i].angle_rad,
                 relative.beams[i].range_m, relative.beams[i].kind,
                 anchor.source_id, anchor.calibration_revision,
                 anchor.quality == Ld19AnchorQuality::synthetic_exact ? BeamTimeQuality::synthetic_exact
                   : (anchor.quality == Ld19AnchorQuality::calibrated ? BeamTimeQuality::calibrated
                                                                        : BeamTimeQuality::provisional)};
  }
  return output;
}
} // namespace slam::sensors
