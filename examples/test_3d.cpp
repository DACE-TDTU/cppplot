#include "cppplot.hpp"
#include <cmath>
#include <vector>

void test_surf() {
  using namespace cppplot;

  // Create 3D mesh
  int n = 40;
  std::vector<std::vector<double>> X(n, std::vector<double>(n));
  std::vector<std::vector<double>> Y(n, std::vector<double>(n));
  std::vector<std::vector<double>> Z(n, std::vector<double>(n));

  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      double x = -3.0 + 6.0 * i / (n - 1);
      double y = -3.0 + 6.0 * j / (n - 1);
      X[i][j] = x;
      Y[i][j] = y;
      Z[i][j] = std::sin(std::sqrt(x * x + y * y));
    }
  }

  figure(800, 600);
  view_init(30, -60);
  surf(X, Y, Z, {{"cmap", "viridis"}, {"alpha", "0.8"}});
  title("3D Surface Plot");
  xlabel("X-axis");
  ylabel("Y-axis");
  zlabel("Z-axis");
  savefig("test_surf.svg");
}

void test_plot3() {
  using namespace cppplot;

  int n = 1000;
  std::vector<double> x, y, z;
  for (int i = 0; i < n; ++i) {
    double t = 0.1 * i;
    x.push_back(std::sin(t));
    y.push_back(std::cos(t));
    z.push_back(t * 0.1);
  }

  figure(800, 600);
  view_init(15, -45);
  plot3(x, y, z, "b-");
  title("3D Helix");
  savefig("test_plot3.svg");
}

int main() {
  test_surf();
  test_plot3();
  return 0;
}
