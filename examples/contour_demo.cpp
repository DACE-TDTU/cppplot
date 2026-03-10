#include <cmath>
#include <cppplot/cppplot.hpp>
#include <vector>


using namespace cppplot;

int main() {
  // Grid sizing
  int N = 50;
  std::vector<double> x, y;
  for (int i = 0; i < N; ++i) {
    x.push_back(-3.0 + 6.0 * i / (N - 1));
    y.push_back(-3.0 + 6.0 * i / (N - 1));
  }

  // Scalar field Z = sin(x) * cos(y)
  std::vector<std::vector<double>> z(N, std::vector<double>(N));
  for (int r = 0; r < N; ++r) {
    for (int c = 0; c < N; ++c) {
      // z(r,c) maps to y(r), x(c)
      z[r][c] = std::sin(x[c]) * std::cos(y[r]);
    }
  }

  // Contour levels
  std::vector<double> levels = {-0.8, -0.5, -0.2, 0.0, 0.2, 0.5, 0.8};

  figure(800, 600);

  // Draw contour plot
  contour(x, y, z, levels);

  title("Contour Plot: sin(x) * cos(y)");
  xlabel("X Axis");
  ylabel("Y Axis");

  savefig("output/contour_demo.svg");

  return 0;
}
