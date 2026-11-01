#pragma once

#include "slam/mapping/distance_field.hpp"

#include <cstdint>
#include <iosfwd>
#include <memory>
#include <string>

namespace slam::mapping {

// Coordinate frames are descriptive provenance, NOT proof of calibrated
// map/odom alignment. Consumers must inspect metric_origin_verified.
struct MapProvenance {
  std::string frame_id;
  std::string source_id;
  std::string truth_class;
  bool metric_origin_verified{false};
};

struct MapBundle {
  std::shared_ptr<const MapSnapshot> snapshot;
  MapProvenance provenance;
};

// FUSMAP2D v1: fixed little-endian fields, explicit known mask and IEEE754
// log odds. Footer FNV-1a detects accidental damage, NOT an authenticity MAC.
// Limits: 16 million cells, 256 bytes per metadata field, 150 MiB file.
// Streams throw std::invalid_argument/runtime_error on malformed content.
void write_map_bundle(std::ostream& out, const MapBundle& bundle);
[[nodiscard]] MapBundle read_map_bundle(std::istream& in);

// Visualization exports ONLY. PGM loses authoritative known-mask/log-odds
// fidelity; reload the binary bundle for localization/mapping.
void write_map_pgm(std::ostream& out, const MapSnapshot& snapshot);
void write_map_yaml(std::ostream& out, const MapSnapshot& snapshot,
                    const std::string& pgm_basename);

} // namespace slam::mapping
