#pragma once
#include <cxxopts.hpp>

#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace slam::cli {
struct Positionals {
  bool help{false};
  std::vector<std::string> values;
  std::string usage;
};

// cxxopts-backed parser for utilities whose interface consists entirely of
// positional arguments. Explicit cardinality prevents silently ignored extra
// input; cxxopts performs all tokenization, option recognition and --help.
inline Positionals parse_positionals(
    int argc,char** argv,const std::string& program,
    const std::string& synopsis,std::size_t minimum,std::size_t maximum) {
  cxxopts::Options options(program,synopsis);
  options.add_options()
      ("args","Positional inputs",cxxopts::value<std::vector<std::string>>())
      ("h,help","Print help");
  options.parse_positional({"args"});
  options.positional_help(synopsis);
  const auto parsed=options.parse(argc,argv);
  if(parsed.count("help")&&parsed["help"].as<bool>())return {true,{},options.help()};
  const auto values=parsed.count("args")?
      parsed["args"].as<std::vector<std::string>>():
      std::vector<std::string>{};
  if(values.size()<minimum||values.size()>maximum)
    throw std::invalid_argument("Usage: "+program+" "+synopsis);
  return {false,values,options.help()};
}
} // namespace slam::cli
