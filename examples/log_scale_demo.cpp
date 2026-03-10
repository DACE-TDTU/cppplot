#include <cmath>
#include <cppplot/cppplot.hpp>
#include <iostream>
#include <vector>

using namespace cppplot;

int main() {
  std::cout << "Generating log scale demonstration plot..." << std::endl;

  // Create a new figure with 2 subplots (linear vs log)
  figure(1000, 500);

  // Generate frequency data (logarithmically spaced) from 0.01 to 100
  auto x = logspace(-2, 2, 200);
  std::vector<double> y_linear, y_quad, y_sqrt;

  for (double xi : x) {
    y_linear.push_back(xi);          // O(N)
    y_quad.push_back(xi * xi);       // O(N^2)
    y_sqrt.push_back(std::sqrt(xi)); // O(sqrt(N))
  }

  // ---------------------------------------------------------
  // Subplot 1: Linear Scale (To see why log is needed)
  // ---------------------------------------------------------
  subplot(1, 2, 1);
  plot(x, y_linear, "b-", opts({{"label", "O(N)"}, {"linewidth", "2"}}));
  plot(x, y_quad, "r--", opts({{"label", "O(N^2)"}, {"linewidth", "2"}}));
  plot(x, y_sqrt, "g-.", opts({{"label", "O(sqrt(N))"}, {"linewidth", "2"}}));

  xlabel("X (Linear Scale)");
  ylabel("Y");
  title("Standard Linear Plot");
  grid(true);
  legend(true);

  // ---------------------------------------------------------
  // Subplot 2: Log-Log Scale
  // ---------------------------------------------------------
  subplot(1, 2, 2);
  plot(x, y_linear, "b-", opts({{"label", "O(N)"}, {"linewidth", "2"}}));
  plot(x, y_quad, "r--", opts({{"label", "O(N^2)"}, {"linewidth", "2"}}));
  plot(x, y_sqrt, "g-.", opts({{"label", "O(sqrt(N))"}, {"linewidth", "2"}}));

  // Apply log scale to both axes
  xscale("log");
  yscale("log");

  xlabel("X (Log Scale)");
  ylabel("Y (Log Scale)");
  title("Log-Log Plot (Native)");
  grid(true);
  // You can customize the minor grid or tick labels, but log scale takes care
  // of spacing
  legend(true);

  // Save the figure
  savefig("log_scale_demo.svg");
  std::cout << "Done! Saved as log_scale_demo.svg" << std::endl;

  return 0;
}
