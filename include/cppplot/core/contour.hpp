/**
 * @file contour.hpp
 * @brief Marching Squares algorithm for contour plots
 */

#ifndef CPPPLOT_CORE_CONTOUR_HPP
#define CPPPLOT_CORE_CONTOUR_HPP

#include "types.hpp"
#include <utility>
#include <vector>


namespace cppplot {

struct ContourLine {
  double level;
  std::vector<std::pair<Point, Point>> segments;
};

/**
 * @brief Implement the Marching Squares algorithm to find isolines on a 2D
 * grid.
 *
 * @param x 1D vector of x coordinates (size cols)
 * @param y 1D vector of y coordinates (size rows)
 * @param z 2D vector of z values (rows x cols)
 * @param levels vector of z levels to compute contours for
 * @return std::vector<ContourLine> computed contour segments per level
 */
inline std::vector<ContourLine>
marching_squares(const std::vector<double> &x, const std::vector<double> &y,
                 const std::vector<std::vector<double>> &z,
                 const std::vector<double> &levels) {
  std::vector<ContourLine> contours;
  if (x.empty() || y.empty() || z.empty() || z[0].empty() || levels.empty()) {
    return contours;
  }

  size_t rows = y.size();
  size_t cols = x.size();

  if (z.size() != rows || z[0].size() != cols) {
    return contours;
  }

  // Pre-allocate contours
  for (double level : levels) {
    ContourLine c;
    c.level = level;
    contours.push_back(c);
  }

  // Edge interpolation helper
  auto interp = [](double level, double z1, double z2, const Point &p1,
                   const Point &p2) {
    if (z1 == z2)
      return p1;
    double t = (level - z1) / (z2 - z1);
    return Point(p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y));
  };

  for (size_t r = 0; r < rows - 1; ++r) {
    for (size_t c = 0; c < cols - 1; ++c) {
      double z1 = z[r][c];
      double z2 = z[r][c + 1];
      double z3 = z[r + 1][c + 1];
      double z4 = z[r + 1][c];

      Point p1(x[c], y[r]);
      Point p2(x[c + 1], y[r]);
      Point p3(x[c + 1], y[r + 1]);
      Point p4(x[c], y[r + 1]);

      for (size_t i = 0; i < levels.size(); ++i) {
        double level = levels[i];

        int b1 = (z1 > level) ? 1 : 0;
        int b2 = (z2 > level) ? 1 : 0;
        int b3 = (z3 > level) ? 1 : 0;
        int b4 = (z4 > level) ? 1 : 0;

        int state = b1 * 8 + b2 * 4 + b3 * 2 + b4 * 1;

        std::vector<Point> pts;

        switch (state) {
        case 1:
        case 14:
          pts.push_back(interp(level, z4, z3, p4, p3));
          pts.push_back(interp(level, z1, z4, p1, p4));
          break;
        case 2:
        case 13:
          pts.push_back(interp(level, z2, z3, p2, p3));
          pts.push_back(interp(level, z4, z3, p4, p3));
          break;
        case 3:
        case 12:
          pts.push_back(interp(level, z2, z3, p2, p3));
          pts.push_back(interp(level, z1, z4, p1, p4));
          break;
        case 4:
        case 11:
          pts.push_back(interp(level, z1, z2, p1, p2));
          pts.push_back(interp(level, z2, z3, p2, p3));
          break;
        case 5:
          pts.push_back(interp(level, z1, z2, p1, p2));
          pts.push_back(interp(level, z2, z3, p2, p3));
          pts.push_back(interp(level, z4, z3, p4, p3));
          pts.push_back(interp(level, z1, z4, p1, p4));
          break;
        case 6:
        case 9:
          pts.push_back(interp(level, z1, z2, p1, p2));
          pts.push_back(interp(level, z4, z3, p4, p3));
          break;
        case 7:
        case 8:
          pts.push_back(interp(level, z1, z2, p1, p2));
          pts.push_back(interp(level, z1, z4, p1, p4));
          break;
        case 10:
          pts.push_back(interp(level, z1, z2, p1, p2));
          pts.push_back(interp(level, z1, z4, p1, p4));
          pts.push_back(interp(level, z2, z3, p2, p3));
          pts.push_back(interp(level, z4, z3, p4, p3));
          break;
        default:
          break;
        }

        if (pts.size() >= 2) {
          contours[i].segments.push_back({pts[0], pts[1]});
        }
        if (pts.size() == 4) {
          contours[i].segments.push_back({pts[2], pts[3]});
        }
      }
    }
  }

  return contours;
}

} // namespace cppplot

#endif
