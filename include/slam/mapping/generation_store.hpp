#pragma once

#include "slam/mapping/map_bundle.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>

namespace slam::mapping {

// Filesystem-backed selection of immutable FUSMAP2D generations.
// One writer is serialized by an advisory flock; on-disk publication uses
// fsync(file) -> rename -> fsync(directory) for crash-recoverable pointers.
// The private directory must not be writable by an untrusted local process.
struct GenerationRef {
  std::uint64_t id{};
  std::uint64_t fingerprint{};
  friend bool operator==(const GenerationRef&, const GenerationRef&) = default;
};
struct GenerationState {
  std::optional<GenerationRef> active;
  std::optional<GenerationRef> previous;
  bool recovered_from_previous{};
};

class GenerationStore final {
 public:
  explicit GenerationStore(std::filesystem::path root);

  // The caller must bootstrap once with expected_active_fingerprint=nullopt.
  // Subsequent publication must explicitly pin the existing incumbent's
  // fingerprint. Nothing is committed when comparison fails.
  [[nodiscard]] GenerationRef publish(
      const MapBundle& candidate,
      std::optional<std::uint64_t> expected_active_fingerprint);

  // Inspects valid files, automatically falls back to a valid previous map
  // when the current file is missing/corrupt, and writes that decision durably.
  // Malformed pointer metadata always fails closed: never guess from orphans.
  [[nodiscard]] GenerationState recover();
  [[nodiscard]] MapBundle load_active();
  [[nodiscard]] GenerationState state();

  // Explicitly re-select previous generation; never overwrites map content.
  [[nodiscard]] GenerationState rollback(std::uint64_t expected_active_fingerprint);

 private:
  std::filesystem::path root_;
};
} // namespace slam::mapping
