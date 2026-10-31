#pragma once

#include "slam/core/time.hpp"
#include "slam/sensors/scan.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace slam::sensors {
constexpr std::size_t kLd19FrameSize = 47;
constexpr std::size_t kLd19PointsPerFrame = 12;
using Ld19Bytes = std::array<std::uint8_t, kLd19FrameSize>;

enum class Ld19FrameStatus { valid, bad_crc, bad_format };

struct Ld19Point {
  std::uint16_t distance_mm{};
  std::uint8_t intensity{};
};

struct Ld19Packet {
  Ld19Bytes raw{};  // original unmodified UART bytes
  std::uint16_t speed_degrees_per_second{};
  std::uint16_t start_angle_centidegrees{};
  std::uint16_t end_angle_centidegrees{};
  std::uint16_t device_timestamp_ms{};
  std::array<Ld19Point, kLd19PointsPerFrame> points{};
};

// CRC-8, polynomial 0x4D, initial value 0, no reflection or final XOR.
[[nodiscard]] std::uint8_t ld19_crc8(std::span<const std::uint8_t> input) noexcept;
[[nodiscard]] Ld19FrameStatus decode_ld19(const Ld19Bytes& raw, Ld19Packet& output) noexcept;

struct Ld19ReceivedFrame {
  Ld19Bytes raw{};
  Ld19FrameStatus status{Ld19FrameStatus::bad_format};
  std::optional<Ld19Packet> packet;
  Timestamp last_byte_received_at{}; // host receipt clock, never acquisition time
};

// Supports split UART reads and resynchronization after corruption. Every
// recognized header/version candidate is surfaced, even if CRC is bad.
// The raw byte stream is NOT a lossless recording substitute: store the
// original UART input separately for byte-for-byte forensic replay.
class Ld19StreamParser {
 public:
  [[nodiscard]] std::vector<Ld19ReceivedFrame> push(
      std::span<const std::uint8_t> bytes, Timestamp chunk_last_byte_received_at);
  void reset() noexcept;
  [[nodiscard]] std::uint64_t discarded_bytes() const noexcept { return discarded_bytes_; }

 private:
  std::vector<std::uint8_t> buffer_;
  std::optional<Timestamp> active_receive_clock_;
  std::uint64_t discarded_bytes_{};
};

enum class Ld19CounterStatus { first, continuous, wrapped, discontinuity, invalid };
struct Ld19CounterReading {
  Ld19CounterStatus status{Ld19CounterStatus::invalid};
  std::uint64_t unwrapped_ms{}; // device-relative time; NOT canonical nanoseconds
};

// The nominal LD19 tick counter wraps every 30,000 ms. A jump larger than
// max_gap_ms, a reset or an ambiguous multi-wrap span is fail-closed. Call
// reset() to establish a new device epoch after a discontinuity.
class Ld19CounterUnwrapper {
 public:
  explicit Ld19CounterUnwrapper(std::uint16_t max_gap_ms = 500);
  [[nodiscard]] Ld19CounterReading observe(std::uint16_t raw_ms) noexcept;
  void reset() noexcept;

 private:
  std::uint16_t max_gap_ms_;
  std::optional<std::uint16_t> previous_;
  std::uint64_t elapsed_ms_{};
  bool faulted_{};
};

// Angle interpolation and angular-speed timing are a PROVISIONAL model.
// The LD19 packet timestamp's exact physical reference is not established.
// These offsets refer to the LAST observed beam, not automatically to the
// device timestamp or host receipt timestamp.
struct Ld19RelativeBeam {
  double angle_rad{};
  double range_m{};
  std::uint8_t intensity{};
  ReturnKind kind{ReturnKind::invalid};
  TimeNs offset_from_last_beam_ns{};
};
struct Ld19RelativeScan {
  std::array<Ld19RelativeBeam, kLd19PointsPerFrame> beams{};
  bool reference_unverified{true};
};

[[nodiscard]] std::optional<Ld19RelativeScan> ld19_relative_beams(
    const Ld19Packet& packet, double max_packet_duration_s = 0.2) noexcept;

// Explicit external anchor supplied by a simulator or measured clock model.
// Even with an anchor, mark the time quality: an assumed last-beam physical
// packet reference is not a calibrated measurement.
enum class Ld19AnchorQuality { provisional, calibrated, synthetic_exact };
struct Ld19TimeAnchor {
  Timestamp last_beam_time{};
  Ld19AnchorQuality quality{Ld19AnchorQuality::provisional};
  std::uint64_t source_id{};
  std::uint64_t calibration_revision{};
  bool physical_reference_validated{false}; // must be true for calibrated or synthetic_exact
};
[[nodiscard]] std::optional<std::array<BeamObservation, kLd19PointsPerFrame>>
anchor_ld19_beams(const Ld19RelativeScan& relative, Ld19TimeAnchor anchor) noexcept;

} // namespace slam::sensors
