#include "slam/cli/parse.hpp"
#include "slam/evaluation/map_accuracy.hpp"

#include <exception>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
  slam::cli::Positionals parsed;
  try {
    parsed=slam::cli::parse_positionals(argc,argv,"slam_map_accuracy",
        "<apartment|apartment_explore|warehouse|warehouse_asymmetric|office|workshop|loft|atrium|industrial> <generated.slammap>",2,2);
  }catch(const std::exception& e) {
    std::cerr<<e.what()<<'\n';return 2;
  }
  if(parsed.help){std::cout<<parsed.usage;return 0;}
  const auto& cli_args=parsed.values;
  try {
    const std::string name(cli_args[0]);
    // Only the independent evaluator gets access to exact scene walls.
    const auto scene=slam::fixtures::named_scene(name);
    std::ifstream file(cli_args[1], std::ios::binary);
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
