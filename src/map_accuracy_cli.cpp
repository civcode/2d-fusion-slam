#include "slam/evaluation/map_accuracy.hpp"

#include <exception>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: slam_map_accuracy <apartment|warehouse> <generated.slammap>\n"
              << "Writes FUSMAPACC1 JSON to stdout; refuse unverified/unaligned map inputs.\n";
    return 2;
  }
  try {
    const std::string name(argv[1]);
    slam::fixtures::Scene scene;
    if (name == "apartment") scene = slam::fixtures::apartment();
    else if (name == "warehouse") scene = slam::fixtures::warehouse();
    else throw std::invalid_argument("unsupported exact synthetic reference scene");
    std::ifstream file(argv[2], std::ios::binary);
    if (!file) throw std::runtime_error("cannot open generated FUSMAP2D map");
    const auto bundle = slam::mapping::read_map_bundle(file);
    const auto report = slam::evaluation::evaluate_map_accuracy(bundle, scene);
    slam::evaluation::write_map_accuracy_json(std::cout, report, scene);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Map accuracy evaluation refused: " << e.what() << '\n';
    return 1;
  }
}
