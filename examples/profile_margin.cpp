// compile with: g++ -std=c++17 -O2 -I../cppplot/include profile_margin.cpp -o profile_margin

#include <chrono>
#include <iostream>

#include <cppplot/control/control.hpp>

using namespace cppplot::control;

int main() {
  // G = 1/(s+1)^10
  TransferFunction G({1.0}, {1.0, 10.0, 45.0, 120.0, 210.0, 252.0, 210.0, 120.0,
                             45.0, 10.0, 1.0});

  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < 1000; ++i) {
    auto m = margin(G);
  }
  auto end = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double, std::milli> ms = end - start;
  std::cout << "cppplot (1000 runs): " << ms.count() << " ms" << std::endl;

  return 0;
}
