/**
 * @file robotics/mobile_base.hpp
 * @brief Differential-drive mobile-base kinematics.
 */

#ifndef CPPPLOT_ROBOTICS_MOBILE_BASE_HPP
#define CPPPLOT_ROBOTICS_MOBILE_BASE_HPP

#include "types.hpp"
#include <stdexcept>

namespace cppplot {
namespace robotics {

struct DifferentialDriveParams {
  double wheel_radius = 0.05;
  double track_width = 0.30;

  DifferentialDriveParams() = default;
  DifferentialDriveParams(double wheel_radius_, double track_width_)
      : wheel_radius(wheel_radius_), track_width(track_width_) {}
};

class DifferentialDrive {
public:
  explicit DifferentialDrive(const DifferentialDriveParams &params = {})
      : params_(params) {
    validate();
  }

  const DifferentialDriveParams &params() const { return params_; }

  Twist2D forward(double omega_left, double omega_right) const {
    const double r = params_.wheel_radius;
    const double L = params_.track_width;
    return Twist2D(0.5 * r * (omega_right + omega_left),
                   r * (omega_right - omega_left) / L);
  }

  Twist2D forward(const WheelVelocity &wheels) const {
    return forward(wheels.left, wheels.right);
  }

  WheelVelocity inverse(double v, double omega) const {
    const double r = params_.wheel_radius;
    const double L = params_.track_width;
    return WheelVelocity((v - 0.5 * omega * L) / r,
                         (v + 0.5 * omega * L) / r);
  }

  WheelVelocity inverse(const Twist2D &twist) const {
    return inverse(twist.v, twist.omega);
  }

private:
  DifferentialDriveParams params_;

  void validate() const {
    if (params_.wheel_radius <= 0.0)
      throw std::runtime_error("DifferentialDrive: wheel_radius must be > 0");
    if (params_.track_width <= 0.0)
      throw std::runtime_error("DifferentialDrive: track_width must be > 0");
  }
};

} // namespace robotics
} // namespace cppplot

#endif // CPPPLOT_ROBOTICS_MOBILE_BASE_HPP
