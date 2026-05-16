/**
 * @file robotics/controllers.hpp
 * @brief Basic robotics controllers.
 */

#ifndef CPPPLOT_ROBOTICS_CONTROLLERS_HPP
#define CPPPLOT_ROBOTICS_CONTROLLERS_HPP

#include "trajectory.hpp"
#include <cmath>
#include <stdexcept>
#include <utility>

namespace cppplot {
namespace robotics {

struct PurePursuitConfig {
  double lookahead = 0.5;
  double speed = 0.5;
  double max_speed = 1.0;
  double max_omega = 2.0;
};

class PurePursuitController {
public:
  PurePursuitController(Path2D path, PurePursuitConfig config = {})
      : path_(std::move(path)), config_(config) {
    if (config_.lookahead <= 0.0)
      throw std::runtime_error("PurePursuitController: lookahead must be > 0");
    if (config_.max_speed <= 0.0)
      throw std::runtime_error("PurePursuitController: max_speed must be > 0");
    if (config_.max_omega <= 0.0)
      throw std::runtime_error("PurePursuitController: max_omega must be > 0");
  }

  Twist2D compute(const Pose2D &pose) const {
    const Point2D goal = path_.lookahead_point(point(pose), config_.lookahead);
    const double dx = goal.x - pose.x;
    const double dy = goal.y - pose.y;
    const double alpha = normalize_angle(std::atan2(dy, dx) - pose.theta);
    const double curvature = 2.0 * std::sin(alpha) / config_.lookahead;

    const double v = clamp(config_.speed, -config_.max_speed, config_.max_speed);
    const double omega =
        clamp(v * curvature, -config_.max_omega, config_.max_omega);
    return Twist2D(v, omega);
  }

  const Path2D &path() const { return path_; }
  const PurePursuitConfig &config() const { return config_; }

private:
  Path2D path_;
  PurePursuitConfig config_;
};

class PID {
public:
  PID(double kp, double ki, double kd, double min_output = -1e9,
      double max_output = 1e9)
      : kp_(kp), ki_(ki), kd_(kd), min_output_(min_output),
        max_output_(max_output) {}

  double compute(double reference, double measurement, double dt) {
    const double error = reference - measurement;
    integral_ += error * dt;
    const double derivative = first_ ? 0.0 : (error - previous_error_) / dt;
    first_ = false;
    previous_error_ = error;

    return clamp(kp_ * error + ki_ * integral_ + kd_ * derivative,
                 min_output_, max_output_);
  }

  void reset() {
    integral_ = 0.0;
    previous_error_ = 0.0;
    first_ = true;
  }

private:
  double kp_;
  double ki_;
  double kd_;
  double min_output_;
  double max_output_;
  double integral_ = 0.0;
  double previous_error_ = 0.0;
  bool first_ = true;
};

} // namespace robotics
} // namespace cppplot

#endif // CPPPLOT_ROBOTICS_CONTROLLERS_HPP
