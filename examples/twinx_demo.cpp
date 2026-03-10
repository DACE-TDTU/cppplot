#include <cmath>
#include <cppplot/cppplot.hpp>


using namespace cppplot;

int main() {
  std::vector<double> x, y1, y2;
  for (int i = 0; i <= 100; ++i) {
    double xi = i * 0.1;
    x.push_back(xi);
    y1.push_back(std::sin(xi));       // Range: -1 to 1
    y2.push_back(std::exp(xi * 0.5)); // Range: 1 to ~148
  }

  figure(800, 600);

  // Primary axis plot
  plot(x, y1, "-b", {{"label", "sin(x)"}});
  ylabel("Primary Y (sin)");
  xlabel("X Axis");

  // Secondary axis plot
  twinx();
  plot(x, y2, "-r", {{"label", "exp(x/2)"}});
  ylabel2("Secondary Y (exp)");

  title("Twin Axes Example");
  legend();

  savefig("output/twinx_example.svg");

  return 0;
}
