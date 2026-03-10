/**
 * @file boxplot_demo.cpp
 * @brief Example of drawing boxplots with cppplot
 */

#include <cppplot/core/utils.hpp>
#include <cppplot/cppplot.hpp>
#include <iostream>


using namespace cppplot;

int main() {
  std::cout << "Creating boxplot demonstration..." << std::endl;

  // Generate some random data representing 3 different groups
  std::vector<std::vector<double>> data;

  // Group 1: Normal distribution N(0, 1)
  data.push_back(cppplot::randn(100, 0.0, 1.0));

  // Group 2: Normal distribution N(2, 1.5)
  data.push_back(cppplot::randn(150, 2.0, 1.5));

  // Group 3: Normal distribution N(-1, 0.5)
  data.push_back(cppplot::randn(80, -1.0, 0.5));

  // Create figure
  figure(800, 600);

  // Add title
  title("Boxplot Demonstration", opts({{"fontsize", "20"}}));

  // Create boxplots
  boxplot(data, {1.0, 2.0, 3.0}, opts({{"width", "0.6"}, {"color", "blue"}}));

  // Labels
  xlabel("Groups");
  ylabel("Values");

  // Set custom XTicks explicitly if supported (optional via axes)
  // gca().set_xticklabels({"Group 1", "Group 2", "Group 3"}); // if supported

  // Show grid
  grid(true);

  // Save the plot
  std::cout << "Saving as boxplot_demo.svg..." << std::endl;
  savefig("boxplot_demo.svg");

  std::cout << "Done!" << std::endl;
  return 0;
}
