#include <cppplot/control/control.hpp>
#include <iostream>
#include <vector>

using namespace cppplot::control::sysid;

int main() {
  std::vector<double> u = {1.0, -1.0, 1.0, -1.0};
  std::vector<double> y = {0.5, -0.5, 0.5, -0.5};
  double Ts = 0.1;

  auto result = coherence(u, y, Ts, 2);
  auto freq = result.first;
  auto coh = result.second;

  std::cout << "Coherence size: " << freq.size() << std::endl;
  return 0;
}
