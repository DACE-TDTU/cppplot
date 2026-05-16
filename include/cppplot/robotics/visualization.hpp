/**
 * @file robotics/visualization.hpp
 * @brief Plot helpers for robotics demos.
 */

#ifndef CPPPLOT_ROBOTICS_VISUALIZATION_HPP
#define CPPPLOT_ROBOTICS_VISUALIZATION_HPP

#include "../pyplot.hpp"
#include "mobile_base.hpp"
#include "trajectory.hpp"
#include <algorithm>
#include <string>
#include <vector>

namespace cppplot {
namespace robotics {

inline std::vector<double> xs(const std::vector<Point2D> &points) {
  std::vector<double> out;
  out.reserve(points.size());
  for (const auto &p : points)
    out.push_back(p.x);
  return out;
}

inline std::vector<double> ys(const std::vector<Point2D> &points) {
  std::vector<double> out;
  out.reserve(points.size());
  for (const auto &p : points)
    out.push_back(p.y);
  return out;
}

inline std::vector<double> xs(const std::vector<Pose2D> &poses) {
  std::vector<double> out;
  out.reserve(poses.size());
  for (const auto &p : poses)
    out.push_back(p.x);
  return out;
}

inline std::vector<double> ys(const std::vector<Pose2D> &poses) {
  std::vector<double> out;
  out.reserve(poses.size());
  for (const auto &p : poses)
    out.push_back(p.y);
  return out;
}

inline void plot_path(const Path2D &path, const std::string &label = "path") {
  plot(xs(path.points()), ys(path.points()), "k--", {{"label", label}});
}

inline void plot_trajectory(const std::vector<Pose2D> &poses,
                            const std::string &label = "trajectory") {
  plot(xs(poses), ys(poses), "b-", {{"label", label}});
}

inline void draw_robot(const Pose2D &pose,
                       const DifferentialDriveParams &params = {},
                       const std::string &color = "red") {
  const double body = std::max(params.track_width, params.wheel_radius * 4.0);
  const double nose = body * 0.8;
  const double half = body * 0.35;
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);

  auto transform = [&](double local_x, double local_y) {
    return Point2D(pose.x + c * local_x - s * local_y,
                   pose.y + s * local_x + c * local_y);
  };

  const Point2D p0 = transform(nose, 0.0);
  const Point2D p1 = transform(-0.5 * nose, half);
  const Point2D p2 = transform(-0.5 * nose, -half);

  std::vector<double> rx = {p0.x, p1.x, p2.x, p0.x};
  std::vector<double> ry = {p0.y, p1.y, p2.y, p0.y};
  plot(rx, ry, "-", {{"color", color}, {"linewidth", "2"}});
}

inline void plot_tracking_errors(const std::vector<TrackingSample> &samples) {
  std::vector<double> t, dist, heading;
  t.reserve(samples.size());
  dist.reserve(samples.size());
  heading.reserve(samples.size());

  for (const auto &s : samples) {
    t.push_back(s.t);
    dist.push_back(s.distance_error);
    heading.push_back(s.heading_error);
  }

  plot(t, dist, "b-", {{"label", "distance error"}});
  plot(t, heading, "r--", {{"label", "heading error"}});
}

} // namespace robotics
} // namespace cppplot

#endif // CPPPLOT_ROBOTICS_VISUALIZATION_HPP
