#include <cmath>
#include <cppplot/cppplot.hpp>
#include <iostream>
#include <vector>

using namespace cppplot;

int main() {
  std::cout << "Creating Polar Plot Demo..." << std::endl;

  // 1. Archimedean spiral: r = a + b * theta
  std::vector<double> theta1, r1;
  double a = 0.5;
  double b = 1.0;
  for (double t = 0; t <= 4 * M_PI; t += 0.05) {
    theta1.push_back(t);
    r1.push_back(a + b * t);
  }

  // 2. Rose curve: r = cos(k * theta)
  std::vector<double> theta2, r2;
  double k = 3.0; // 3 petals
  for (double t = 0; t <= 2 * M_PI; t += 0.02) {
    theta2.push_back(t);
    r2.push_back(5.0 * std::abs(std::sin(k * t)));
  }

  // Create figure
  figure();

  // Enable polar axes
  polar(true);

  // Set title
  title("Archimedean Spiral and Rose Curve");

  // Plot data
  PlotOptions opts1;
  opts1["color"] = "blue";
  opts1["label"] = "Spiral";
  opts1["linewidth"] = "2";
  plot(theta1, r1, "", opts1);

  PlotOptions opts2;
  opts2["color"] = "red";
  opts2["label"] = "Rose";
  opts2["linewidth"] = "2";
  plot(theta2, r2, "", opts2);

  // Show legend
  legend();

  // Render and save
  savefig("output/polar_demo.svg");

  std::cout << "Demo saved to output/polar_demo.svg" << std::endl;
  return 0;
}
