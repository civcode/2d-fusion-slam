#pragma once

// Human-oriented, read-only review inventory for native experiment exports.
// Explicit Docker host mapping is accepted via SLAM_HOST_OUTPUT_DIR. Nothing
// guesses the host location of /output, and this utility never copies files.
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace slam::cli {
struct ReviewFile {
  std::string role;
  std::filesystem::path path;
  bool upload{true};
};

inline std::string review_json_string(std::string_view value) {
  const char* digits="0123456789abcdef";
  std::string output="\"";
  for(unsigned char c:value) {
    if(c=='"'||c=='\\') { output.push_back('\\');output.push_back(char(c)); }
    else if(c=='\n')output+="\\n";
    else if(c=='\r')output+="\\r";
    else if(c=='\t')output+="\\t";
    else if(c<0x20) {
      output+="\\u00";
      output.push_back(digits[c>>4]);
      output.push_back(digits[c&15]);
    } else output.push_back(char(c));
  }
  return output+"\"";
}
inline std::string review_host_path(const std::filesystem::path& p) {
  const auto absolute=std::filesystem::absolute(p).lexically_normal();
  const char* host=std::getenv("SLAM_HOST_OUTPUT_DIR");
  const char* container=std::getenv("SLAM_CONTAINER_OUTPUT_DIR");
  const std::filesystem::path container_root=
      std::filesystem::path(container&&*container?container:"/output").lexically_normal();
  if(host&&*host) {
    const auto relative=absolute.lexically_relative(container_root);
    if(!relative.empty()&&!relative.is_absolute() &&
       (relative=="." || *relative.begin()!=".."))
      return (std::filesystem::path(host)/relative).lexically_normal().string();
  }
  // /output is a Docker mount, not necessarily a real host directory.
  const auto relative=absolute.lexically_relative(container_root);
  if(!relative.empty()&&!relative.is_absolute() &&
     (relative=="." || *relative.begin()!=".."))
    return "";
  return absolute.string();
}

// Exclusive output: experiment artifacts and their inventory are immutable.
// Always include the manifest itself in the caller's review list.
inline void write_review_manifest(const std::filesystem::path& manifest,
                                  std::string_view experiment,
                                  const std::vector<ReviewFile>& files) {
  if(std::filesystem::exists(manifest))
    throw std::invalid_argument("review manifest already exists; refusing overwrite");
  std::ofstream out(manifest);
  if(!out)throw std::runtime_error("cannot create review manifest");
  out<<"{\n  \"format\": \"SLAM_REVIEW1\",\n  \"experiment\": "
     <<review_json_string(experiment)<<",\n  \"files\": [\n";
  for(std::size_t i=0;i<files.size();++i) {
    const auto& file=files[i];
    const auto absolute=std::filesystem::absolute(file.path).lexically_normal();
    const auto host=review_host_path(file.path);
    if(i)out<<",\n";
    out<<"    {\"role\": "<<review_json_string(file.role)
       <<", \"path\": "<<review_json_string(absolute.string())
       <<", \"host_path\": "<<(host.empty()?"null":review_json_string(host))
       <<", \"upload\": "<<(file.upload?"true":"false")
       <<"}";
  }
  out<<"\n  ]\n}\n";
  if(!out)throw std::runtime_error("failed to write review manifest");
  out.close();
  std::cout<<"\nReview artifacts (upload to ChatGPT):\n";
  for(const auto& file:files) {
    if(!file.upload)continue;
    const auto host=review_host_path(file.path);
    std::cout<<"  "<<file.role<<": "
             <<(host.empty()?std::filesystem::absolute(file.path).string():host);
    if(host.empty())std::cout<<" [container path; host mount not provided]";
    std::cout<<'\n';
  }
  std::cout<<"Other outputs (optional maps/previews):\n";
  for(const auto& file:files) {
    if(file.upload)continue;
    const auto host=review_host_path(file.path);
    std::cout<<"  "<<file.role<<": "
             <<(host.empty()?std::filesystem::absolute(file.path).string():host)
             <<'\n';
  }
  const auto host=review_host_path(manifest);
  std::cout<<"Review manifest: "
           <<(host.empty()?std::filesystem::absolute(manifest).string():host)<<'\n';
  if(host.empty())
    std::cout<<"To print host paths in Docker, pass "
                "-e SLAM_HOST_OUTPUT_DIR=\"$PWD/<mounted-host-directory>\" "
                "with -v \"$PWD/<mounted-host-directory>:/output\".\n";
}
} // namespace slam::cli
