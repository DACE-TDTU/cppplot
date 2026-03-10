#include <cppplot/cppplot.hpp>
#include <iostream>
#include <vector>

using namespace cppplot;

int main() {
  std::cout << "Running Box Plot with Custom Labels Demo..." << std::endl;

  // Generate normal distributions for different groups
  // Group A: mean=50, std=10
  // Group B: mean=75, std=15
  // Group C: mean=60, std=5
  // Group D: mean=45, std=20
  std::vector<std::vector<double>> data = {
      cppplot::randn(100, 50.0, 10.0), cppplot::randn(120, 75.0, 15.0),
      cppplot::randn(90, 60.0, 5.0), cppplot::randn(150, 45.0, 20.0)};

  std::vector<double> positions = {1.0, 2.0, 3.0, 4.0};

  // Draw the boxplot
  boxplot(data, positions, opts({{"width", "0.5"}, {"color", "teal"}}));

  // Apply custom X-axis text labels instead of raw numbers
  std::vector<std::string> labels = {"Group A", "Group B", "Group C",
                                     "Group D"};
  gca().set_xticklabels(labels, positions);

  // Apply a custom Y-axis label list for specific ticks just to demonstrate the
  // API
  std::vector<double> y_ticks = {0.0, 25.0, 50.0, 75.0, 100.0, 125.0};
  std::vector<std::string> y_labels = {"0 (Min)", "25",  "50 (Avg)",
                                       "75",      "100", "125 (Max)"};
  gca().set_yticklabels(y_labels, y_ticks);

  title("Performance by Group");
  xlabel("Test Groups");
  ylabel("Scores");
  grid(true);

  // Save output
  savefig("output/boxplot_labels.svg");
  std::cout << "Saved plot to output/boxplot_labels.svg" << std::endl;

  return 0;
}
