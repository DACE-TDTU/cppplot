#include <cassert>
#include <cmath>
#include <cppplot/cppplot.hpp>
#include <iostream>

using namespace cppplot;

void test_fill_alpha() {
  std::cout << "Testing Issue 3: fill_between alpha..." << std::endl;
  figure(800, 600);
  auto x = linspace(0, 10, 10);
  std::vector<double> y1(10, 1.0);
  std::vector<double> y2(10, 2.0);

  // We expect this fill to be semi-transparent, not opaque.
  // SVG inspection is manual, but we can verify it doesn't crash here.
  fill_between(x, y1, y2, opts({{"color", "blue"}, {"alpha", "0.5"}}));
  savefig("test_output_fill_alpha.svg");

  std::cout << "OK." << std::endl;
}

void test_log_scale_ticks() {
  std::cout << "Testing Issue 6: log scale rendering..." << std::endl;
  figure(800, 600);
  auto x = logspace(-2, 2, 50); // 0.01 to 100
  std::vector<double> y;
  for (double xi : x)
    y.push_back(std::sqrt(xi));

  plot(x, y);
  xscale("log"); // apply log scale

  // Check that we can generate the plot without crashing
  // The log calculation happens in drawElements and drawAxis
  savefig("test_output_log_scale.svg");

  std::cout << "OK." << std::endl;
}

int main() {
  std::cout << "Running fixes regression tests..." << std::endl;

  try {
    test_fill_alpha();
    test_log_scale_ticks();

    std::cout << "All fix regression tests passed!" << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Test failed: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
