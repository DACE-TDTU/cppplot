/**
 * @file robotics/types.hpp
 * @brief Basic robotics data types and small math helpers.
 */

#ifndef CPPPLOT_ROBOTICS_TYPES_HPP
#define CPPPLOT_ROBOTICS_TYPES_HPP

#include <cmath>
#include <vector>

namespace cppplot {
namespace robotics {

constexpr double kPi = 3.141592653589793238462643383279502884;

struct Point2D {
  double x = 0.0;
  double y = 0.0;

  Point2D() = default;
  Point2D(double x_, double y_) : x(x_), y(y_) {}
};

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;

  Pose2D() = default;
  Pose2D(double x_, double y_, double theta_)
      : x(x_), y(y_), theta(theta_) {}
};

struct Twist2D {
  double v = 0.0;
  double omega = 0.0;

  Twist2D() = default;
  Twist2D(double v_, double omega_) : v(v_), omega(omega_) {}
};

struct WheelVelocity {
  double left = 0.0;
  double right = 0.0;

  WheelVelocity() = default;
  WheelVelocity(double left_, double right_) : left(left_), right(right_) {}
};

struct TrackingSample {
  double t = 0.0;
  Pose2D pose;
  Twist2D command;
  double distance_error = 0.0;
  double heading_error = 0.0;
};

inline double clamp(double value, double lo, double hi) {
  return value < lo ? lo : (value > hi ? hi : value);
}

inline double normalize_angle(double angle) {
  while (angle > kPi)
    angle -= 2.0 * kPi;
  while (angle < -kPi)
    angle += 2.0 * kPi;
  return angle;
}

inline double distance(const Point2D &a, const Point2D &b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

inline double distance(const Pose2D &a, const Point2D &b) {
  return distance(Point2D(a.x, a.y), b);
}

inline Point2D point(const Pose2D &pose) { return Point2D(pose.x, pose.y); }

} // namespace robotics
} // namespace cppplot

#endif // CPPPLOT_ROBOTICS_TYPES_HPP
