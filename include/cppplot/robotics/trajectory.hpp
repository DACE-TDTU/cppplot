/**
 * @file robotics/trajectory.hpp
 * @brief Lightweight 2D path utilities for mobile robot examples.
 */

#ifndef CPPPLOT_ROBOTICS_TRAJECTORY_HPP
#define CPPPLOT_ROBOTICS_TRAJECTORY_HPP

#include "types.hpp"
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

namespace cppplot {
namespace robotics {

class Path2D {
public:
  Path2D() = default;
  explicit Path2D(std::vector<Point2D> points) : points_(std::move(points)) {}

  const std::vector<Point2D> &points() const { return points_; }
  std::vector<Point2D> &points() { return points_; }
  bool empty() const { return points_.empty(); }
  size_t size() const { return points_.size(); }

  void push_back(const Point2D &p) { points_.push_back(p); }

  double length() const {
    double total = 0.0;
    for (size_t i = 1; i < points_.size(); ++i)
      total += distance(points_[i - 1], points_[i]);
    return total;
  }

  size_t nearest_index(const Point2D &query) const {
    require_nonempty();
    size_t best = 0;
    double best_dist = distance(query, points_[0]);
    for (size_t i = 1; i < points_.size(); ++i) {
      const double d = distance(query, points_[i]);
      if (d < best_dist) {
        best_dist = d;
        best = i;
      }
    }
    return best;
  }

  Point2D lookahead_point(const Point2D &query, double lookahead) const {
    require_nonempty();
    const size_t nearest = nearest_index(query);

    double accumulated = 0.0;
    for (size_t i = nearest + 1; i < points_.size(); ++i) {
      accumulated += distance(points_[i - 1], points_[i]);
      if (accumulated >= lookahead)
        return points_[i];
    }

    return points_.back();
  }

private:
  std::vector<Point2D> points_;

  void require_nonempty() const {
    if (points_.empty())
      throw std::runtime_error("Path2D: path is empty");
  }
};

inline Path2D make_line_path(Point2D start, Point2D end, int samples) {
  if (samples < 2)
    throw std::runtime_error("make_line_path: samples must be >= 2");

  Path2D path;
  path.points().reserve(static_cast<size_t>(samples));
  for (int i = 0; i < samples; ++i) {
    const double a = static_cast<double>(i) / static_cast<double>(samples - 1);
    path.push_back(Point2D(start.x + a * (end.x - start.x),
                           start.y + a * (end.y - start.y)));
  }
  return path;
}

inline Path2D make_circle_path(Point2D center, double radius, int samples) {
  if (radius <= 0.0)
    throw std::runtime_error("make_circle_path: radius must be > 0");
  if (samples < 3)
    throw std::runtime_error("make_circle_path: samples must be >= 3");

  Path2D path;
  path.points().reserve(static_cast<size_t>(samples) + 1);
  for (int i = 0; i <= samples; ++i) {
    const double a = 2.0 * kPi * static_cast<double>(i) /
                     static_cast<double>(samples);
    path.push_back(Point2D(center.x + radius * std::cos(a),
                           center.y + radius * std::sin(a)));
  }
  return path;
}

} // namespace robotics
} // namespace cppplot

#endif // CPPPLOT_ROBOTICS_TRAJECTORY_HPP
