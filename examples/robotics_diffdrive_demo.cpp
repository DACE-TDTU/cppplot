/**
 * @file robotics_diffdrive_demo.cpp
 * @brief Differential-drive pure-pursuit demo.
 */

#include <cppplot/robotics/robotics.hpp>
#include <iostream>

using namespace cppplot;
using namespace cppplot::robotics;

int main() {
  DifferentialDriveParams params(0.05, 0.30);
  DifferentialDrive robot(params);

  Path2D path = make_line_path(Point2D(0.0, 0.0), Point2D(5.0, 2.0), 80);
  PurePursuitConfig cfg;
  cfg.lookahead = 0.45;
  cfg.speed = 0.55;
  cfg.max_omega = 2.0;
  PurePursuitController controller(path, cfg);

  const double dt = 0.02;
  const int steps = 550;
  Pose2D pose(0.0, -0.6, 0.15);

  std::vector<Pose2D> poses;
  std::vector<TrackingSample> samples;
  poses.reserve(steps + 1);
  samples.reserve(steps);
  poses.push_back(pose);

  for (int k = 0; k < steps; ++k) {
    const double t = k * dt;
    const Twist2D cmd = controller.compute(pose);
    const Point2D nearest = path.points()[path.nearest_index(point(pose))];

    TrackingSample sample;
    sample.t = t;
    sample.pose = pose;
    sample.command = cmd;
    sample.distance_error = distance(pose, nearest);
    sample.heading_error =
        normalize_angle(std::atan2(nearest.y - pose.y, nearest.x - pose.x) -
                        pose.theta);
    samples.push_back(sample);

    WheelVelocity wheels = robot.inverse(cmd);
    pose = integrate_exact(pose, robot.forward(wheels), dt);
    poses.push_back(pose);
  }

  figure(1000, 420);
  subplot(1, 2, 1);
  plot_path(path, "reference path");
  plot_trajectory(poses, "robot trajectory");
  draw_robot(poses.back(), params);
  title("Differential-Drive Pure Pursuit");
  xlabel("x (m)");
  ylabel("y (m)");
  grid(true);
  legend(true);

  subplot(1, 2, 2);
  plot_tracking_errors(samples);
  title("Tracking Errors");
  xlabel("time (s)");
  ylabel("error");
  grid(true);
  legend(true);

  savefig("robotics_diffdrive_demo.svg");
  std::cout << "Generated robotics_diffdrive_demo.svg\n";
  return 0;
}
