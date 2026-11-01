#pragma once
#include "apps/demo/native/json.hpp"
#include "slam/mapping/map_bundle.hpp"

#include <cstddef>
#include <filesystem>
#include <optional>
#include <cstdint>
#include <string>

namespace slam::viewer {
class ViewerData {
public:
  ViewerData(std::filesystem::path maps, std::filesystem::path replays,
             bool example_if_empty);
  Json list_maps() const;
  Json map_metadata(const std::string& name) const;
  Json map_cell(const std::string& name, std::size_t x, std::size_t y) const;
  std::string map_png(const std::string& name) const;
  Json list_replays() const;
  Json replay_info(const std::string& name) const;
  Json replay_frames(const std::string& name) const;
  Json replay_frame(const std::string& name, std::size_t index) const;
private:
  struct ReplaySession {Json data, match_info;std::map<std::size_t,Json> matches;};
  mapping::MapBundle map(const std::string& name) const;
  ReplaySession replay(const std::string& name) const;
  Json registry() const;
  std::filesystem::path maps_, replays_;
  // One verified map and its PNG stay hot while the user explores that map.
  // The single-threaded viewer invalidates this cache on file identity change.
  struct CachedMap {
    std::string name;
    std::uint64_t device{},inode{},bytes{};
    std::int64_t mtime_s{},mtime_ns{},ctime_s{},ctime_ns{};
    mapping::MapBundle bundle;
    std::optional<std::string> png;
  };
  mutable std::optional<CachedMap> map_cache_;
  std::optional<mapping::MapBundle> synthetic_map_;
  std::optional<Json> synthetic_replay_;
};
} // namespace slam::viewer
