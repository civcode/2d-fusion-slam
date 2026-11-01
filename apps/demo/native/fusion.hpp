#pragma once
#include "apps/demo/native/json.hpp"
#include "apps/demo/native/data.hpp"

#include <cstddef>
#include <filesystem>
#include <string>

namespace slam::viewer {

// Independent read-only run browser. Only direct child directories with
// strictly allowed basenames are addressable; no path-controlled filesystem
// writes, no SLAM solver, and no external links from replay JSON.
class FusionRuns {
public:
  explicit FusionRuns(std::filesystem::path root);
  Json index() const;
  Json report(const std::string& run) const;
  Json trajectories(const std::string& run) const;
  Json replay(const std::string& run) const;
  Json frame(const std::string& run,std::size_t index) const;
  Json map_metadata(const std::string& run,const std::string& method) const;
  std::string map_png(const std::string& run,const std::string& method) const;
private:
  struct Session {Json report,poses,trace;};
  std::filesystem::path directory(const std::string& run) const;
  Session load(const std::string& run) const;
  static bool method_name(const std::string& name);
  std::filesystem::path root_;
};

} // namespace slam::viewer
