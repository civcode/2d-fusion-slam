#pragma once
#include "apps/demo/native/json.hpp"
#include "slam/mapping/map_bundle.hpp"

#include <cstddef>
#include <filesystem>
#include <optional>
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
  Json replay_frame(const std::string& name, std::size_t index) const;
private:
  struct ReplaySession {Json data, match_info;std::map<std::size_t,Json> matches;};
  mapping::MapBundle map(const std::string& name) const;
  ReplaySession replay(const std::string& name) const;
  Json registry() const;
  std::filesystem::path maps_, replays_;
  std::optional<mapping::MapBundle> synthetic_map_;
  std::optional<Json> synthetic_replay_;
};
} // namespace slam::viewer
