#include "slam/cli/review_artifacts.hpp"
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <stdexcept>

int main() {
  namespace fs=std::filesystem;
  const auto temp=fs::temp_directory_path()/
    ("slam-review-test-"+std::to_string(
       std::chrono::steady_clock::now().time_since_epoch().count()));
  fs::create_directories(temp);
  try {
    const auto output=temp/"container";
    fs::create_directories(output);
    const auto file=output/"fr079.report.json";
    {std::ofstream f(file);f<<"{}";assert(bool(f));}
#ifdef __unix__
    assert(setenv("SLAM_CONTAINER_OUTPUT_DIR",output.c_str(),1)==0);
    assert(setenv("SLAM_HOST_OUTPUT_DIR","/host/maps",1)==0);
#endif
    const auto mapped=slam::cli::review_host_path(file);
    assert(mapped=="/host/maps/fr079.report.json");
    const auto manifest=output/"review.json";
    slam::cli::write_review_manifest(manifest,"fr079_offline",{
        {"report",file,true}, {"occupancy_map",file,false}});
    std::ifstream in(manifest);
    const std::string json{std::istreambuf_iterator<char>(in),{}};
    assert(json.find("\"format\": \"SLAM_REVIEW1\"")!=std::string::npos);
    assert(json.find("\"host_path\": \"/host/maps/fr079.report.json\"")
           !=std::string::npos);
    assert(json.find("\"upload\": false")!=std::string::npos);
    bool refused=false;
    try { slam::cli::write_review_manifest(manifest,"fr079_offline",{}); }
    catch(const std::invalid_argument&) {refused=true;}
    assert(refused);
    assert(slam::cli::review_json_string("a\"\\\n")=="\"a\\\"\\\\\\n\"");
    fs::remove_all(temp);
  }catch(...) { fs::remove_all(temp);throw; }
}
