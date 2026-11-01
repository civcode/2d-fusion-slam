#include "slam/datasets/carmen.hpp"

#include <zlib.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
std::string read_gzip(const char* filename) {
  gzFile stream = gzopen(filename,"rb");
  if (!stream) throw std::runtime_error("cannot open gzip file");
  std::string result;
  char buf[16384];
  int amount;
  while ((amount = gzread(stream,buf,sizeof(buf))) > 0) {
    if (result.size() > 128 * 1024 * 1024) {
      gzclose(stream);
      throw std::runtime_error("dataset exceeds 128 MiB decompressed limit");
    }
    result.append(buf,static_cast<std::size_t>(amount));
  }
  if (amount < 0) { gzclose(stream); throw std::runtime_error("bad gzip stream"); }
  if (gzclose(stream) != Z_OK) throw std::runtime_error("failed gzip close/CRC");
  return result;
}
}
int main(int argc,char** argv) {
  if ((argc != 2 && argc != 3) || std::string(argv[1]) == "--help") {
    std::cout << "Usage: slam_carmen_inspect <FR079 CARMEN .log or .log.gz> [expected_scan_count]\n";
    return argc == 2 ? 0 : 2;
  }
  try {
    const std::string filename=argv[1];
    std::string content;
    if (filename.size() >= 3 && filename.substr(filename.size()-3)==".gz") {
      content=read_gzip(argv[1]);
    } else {
      std::ifstream f(filename);
      if (!f) throw std::runtime_error("cannot open input file");
      std::ostringstream out;out<<f.rdbuf();content=out.str();
      if (content.size() > 128 * 1024 * 1024) throw std::runtime_error("dataset exceeds limit");
    }
    std::istringstream input(content);
    const auto s=slam::datasets::CarmenReader{}.inspect(input);
    if (s.scans==0) throw std::runtime_error("no CARMEN laser scans found");
    if(argc==3 && (std::stoull(argv[2])!=s.scans || s.minimum_beams!=360 || s.maximum_beams!=360))
      throw std::runtime_error("unexpected FR079 scan count/beam shape");
    std::cout << "{\"scans\":" << s.scans << ",\"odometry\":" << s.odometry
              << ",\"beams\":" << s.ranges << ",\"hits\":" << s.hits
              << ",\"invalid\":" << s.invalid_ranges
              << ",\"min_beams\":" << s.minimum_beams
              << ",\"max_beams\":" << s.maximum_beams
              << ",\"first_ipc_seconds\":" << s.first_scan_seconds
              << ",\"last_ipc_seconds\":" << s.last_scan_seconds << "}\n";
  } catch (const std::exception& e) {
    std::cerr << "CARMEN import failed: " << e.what() << '\n'; return 1;
  }
}
