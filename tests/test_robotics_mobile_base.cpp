#include <cppplot/robotics/robotics.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace cppplot::robotics;

namespace {

void require_close(double a, double b, double tol, const char *message) {
  if (std::abs(a - b) > tol)
    throw std::runtime_error(message);
}

void test_forward_inverse_consistency() {
  DifferentialDrive robot({0.05, 0.30});
  Twist2D twist(0.6, 1.2);
  WheelVelocity wheels = robot.inverse(twist);
  Twist2D recovered = robot.forward(wheels);
  require_close(recovered.v, twist.v, 1e-12, "forward/inverse v mismatch");
  require_close(recovered.omega, twist.omega, 1e-12,
                "forward/inverse omega mismatch");
}

void test_straight_motion() {
  DifferentialDrive robot({0.10, 0.50});
  Twist2D twist = robot.forward(5.0, 5.0);
  require_close(twist.v, 0.5, 1e-12, "straight v mismatch");
  require_close(twist.omega, 0.0, 1e-12, "straight omega mismatch");

  Pose2D next = integrate_exact(Pose2D(0.0, 0.0, 0.0), twist, 2.0);
  require_close(next.x, 1.0, 1e-12, "straight x mismatch");
  require_close(next.y, 0.0, 1e-12, "straight y mismatch");
  require_close(next.theta, 0.0, 1e-12, "straight theta mismatch");
}

void test_in_place_rotation() {
  DifferentialDrive robot({0.10, 0.50});
  Twist2D twist = robot.forward(-5.0, 5.0);
  require_close(twist.v, 0.0, 1e-12, "rotation v mismatch");
  require_close(twist.omega, 2.0, 1e-12, "rotation omega mismatch");

  Pose2D next = integrate_exact(Pose2D(0.0, 0.0, 0.0), twist, 0.5);
  require_close(next.x, 0.0, 1e-12, "rotation x mismatch");
  require_close(next.y, 0.0, 1e-12, "rotation y mismatch");
  require_close(next.theta, 1.0, 1e-12, "rotation theta mismatch");
}

void test_path_helpers() {
  Path2D path = make_line_path(Point2D(0.0, 0.0), Point2D(1.0, 0.0), 11);
  require_close(path.length(), 1.0, 1e-12, "path length mismatch");
  if (path.nearest_index(Point2D(0.49, 0.2)) != 5)
    throw std::runtime_error("nearest_index mismatch");
  Point2D lookahead = path.lookahead_point(Point2D(0.0, 0.0), 0.35);
  require_close(lookahead.x, 0.4, 1e-12, "lookahead x mismatch");
}

} // namespace

int main() {
  try {
    test_forward_inverse_consistency();
    test_straight_motion();
    test_in_place_rotation();
    test_path_helpers();
  } catch (const std::exception &e) {
    std::cerr << "test_robotics_mobile_base FAILED: " << e.what() << "\n";
    return 1;
  }

  std::cout << "test_robotics_mobile_base PASSED\n";
  return 0;
}
