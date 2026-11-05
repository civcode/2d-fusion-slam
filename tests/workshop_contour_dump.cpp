// Testing-only native geometry dump for cross-language workshop parity.
#include "slam/fixtures/synthetic.hpp"

#include <iomanip>
#include <iostream>

int main() {
  const auto scene=slam::fixtures::workshop();
  std::cout<<std::setprecision(17);
  for(const auto& edge:scene.walls)
    std::cout<<edge.start.x()<<' '<<edge.start.y()<<' '
             <<edge.end.x()<<' '<<edge.end.y()<<'\n';
}
