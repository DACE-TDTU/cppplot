/**
 * @file robotics/simulation.hpp
 * @brief Simple deterministic simulation helpers for 2D mobile robots.
 */

#ifndef CPPPLOT_ROBOTICS_SIMULATION_HPP
#define CPPPLOT_ROBOTICS_SIMULATION_HPP

#include "types.hpp"
#include <cmath>
#include <functional>
#include <vector>

namespace cppplot {
namespace robotics {

inline Pose2D integrate_euler(const Pose2D &pose, const Twist2D &twist,
                              double dt) {
  Pose2D next;
  next.x = pose.x + twist.v * std::cos(pose.theta) * dt;
  next.y = pose.y + twist.v * std::sin(pose.theta) * dt;
  next.theta = normalize_angle(pose.theta + twist.omega * dt);
  return next;
}

inline Pose2D integrate_exact(const Pose2D &pose, const Twist2D &twist,
                              double dt) {
  if (std::abs(twist.omega) < 1e-12)
    return integrate_euler(pose, twist, dt);

  const double dtheta = twist.omega * dt;
  const double radius = twist.v / twist.omega;

  Pose2D next;
  next.x = pose.x + radius * (std::sin(pose.theta + dtheta) -
                              std::sin(pose.theta));
  next.y = pose.y - radius * (std::cos(pose.theta + dtheta) -
                              std::cos(pose.theta));
  next.theta = normalize_angle(pose.theta + dtheta);
  return next;
}

inline std::vector<Pose2D>
simulate_unicycle(Pose2D initial, double dt, int steps,
                  const std::function<Twist2D(const Pose2D &, double)> &policy) {
  std::vector<Pose2D> poses;
  poses.reserve(static_cast<size_t>(steps) + 1);
  poses.push_back(initial);

  Pose2D pose = initial;
  for (int k = 0; k < steps; ++k) {
    const double t = k * dt;
    const Twist2D command = policy(pose, t);
    pose = integrate_exact(pose, command, dt);
    poses.push_back(pose);
  }

  return poses;
}

} // namespace robotics
} // namespace cppplot

#endif // CPPPLOT_ROBOTICS_SIMULATION_HPP
