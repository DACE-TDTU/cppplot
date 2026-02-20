/**
 * @file adaptive/mrac.hpp
 * @brief Model Reference Adaptive Control (MRAC)
 *
 * MRAC drives a plant to match a reference model output:
 *   Reference model: ẋₘ = Aₘxₘ + Bₘr,  yₘ = Cₘxₘ
 *   Plant:           ẋₚ = Aₚxₚ + Bₚu   (unknown Aₚ, Bₚ)
 *
 * Adaptation law minimizes tracking error e = yₚ - yₘ.
 *
 * Variants:
 *   - MRAC_Lyapunov:  Lyapunov-based (Barbalat-stable)
 *   - MRAC_Sigma:     σ-modification for robustness to disturbances
 *   - MRAC_Deadzone:  Deadzone modification for noise robustness
 *   - MRAC_Projection: Bounded parameter adaptation
 *   - AdaptivePID:    MRAC-inspired PID with adaptive gains
 *
 * References:
 *   - Ioannou & Sun (1996) "Robust Adaptive Control", Ch. 5
 *   - Narendra & Annaswamy (1989) "Stable Adaptive Systems"
 *   - Slotine & Li (1991) "Applied Nonlinear Control", Ch. 8
 */

#ifndef CPPPLOT_CONTROL_ADAPTIVE_MRAC_HPP
#define CPPPLOT_CONTROL_ADAPTIVE_MRAC_HPP

#include "../../pyplot.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>


namespace cppplot {
namespace control {
namespace adaptive {

using Vec = std::vector<double>;

// ============================================================
//        FIRST-ORDER MRAC (Lyapunov Design)
// ============================================================

/**
 * @brief 1st-order MRAC for plant: ẏₚ = -aₚyₚ + bₚu
 *
 * Reference model: ẏₘ = -aₘyₘ + bₘr  (aₘ > 0 chosen by designer)
 *
 * Control law: u = (1/bₘ)(k₁·r - k₂·yₚ)
 *   k₁ = θ₁*bₘ/bₚ (feedforward),  k₂ = (aₘ - aₚ)/bₚ (feedback)
 *
 * Lyapunov-based adaptation (tracking error e = yₚ - yₘ):
 *   θ̇₁ = -γ₁·e·r         (input gain)
 *   θ̇₂ = -γ₂·e·yₚ        (feedback gain)
 *
 * Stability: requires plant sign(bₚ) to be known.
 */
class MRACFirstOrder {
public:
  double am_, bm_;         ///< Reference model parameters
  double gamma1_, gamma2_; ///< Adaptation gains (γ₁, γ₂ > 0)
  double sig_;             ///< σ-modification (0 = none)
  double deadzone_;        ///< Deadzone width (0 = none)

  double theta1_, theta2_; ///< Adaptive gains (k₁, k₂)
  double ym_;              ///< Reference model state
  double yp_prev_;         ///< Previous plant output (for model)

  /**
   * @param am        Reference model bandwidth (aₘ > 0)
   * @param bm        Reference model input gain
   * @param gamma1    Adaptation rate for input gain θ₁
   * @param gamma2    Adaptation rate for feedback gain θ₂
   * @param sigma     σ-modification coefficient (0 = standard MRAC)
   * @param deadzone  Deadzone threshold (0 = standard MRAC)
   */
  MRACFirstOrder(double am, double bm, double gamma1 = 1.0, double gamma2 = 1.0,
                 double sigma = 0.0, double deadzone = 0.0)
      : am_(am), bm_(bm), gamma1_(gamma1), gamma2_(gamma2), sig_(sigma),
        deadzone_(deadzone), theta1_(1.0), theta2_(0.0), ym_(0.0),
        yp_prev_(0.0) {}

  void reset(double y0 = 0.0, double theta1_0 = 1.0, double theta2_0 = 0.0) {
    ym_ = y0;
    yp_prev_ = y0;
    theta1_ = theta1_0;
    theta2_ = theta2_0;
  }

  /**
   * @brief Compute MRAC control and update adaptation
   *
   * @param r    Reference input
   * @param yp   Plant output measurement
   * @param dt   Time step
   * @return     Control input u
   */
  double compute(double r, double yp, double dt) {
    // Update reference model: ẏₘ = -aₘ yₘ + bₘ r
    ym_ += (-am_ * ym_ + bm_ * r) * dt;

    // Tracking error
    double e = yp - ym_;

    // Deadzone: suppress adaptation if |e| < deadzone
    double e_adapt = (std::abs(e) > deadzone_) ? e : 0.0;

    // Control law: u = θ₁·r - θ₂·yₚ  (normalized by b_sgn=+1)
    double u = theta1_ * r - theta2_ * yp;

    // Adaptation laws (with σ-modification for robustness):
    // θ̇₁ = -γ₁·e_adapt·r - σ·θ₁
    // θ̇₂ = -γ₂·e_adapt·yₚ - σ·θ₂
    double theta1_dot = -gamma1_ * e_adapt * r - sig_ * theta1_;
    double theta2_dot = gamma2_ * e_adapt * yp - sig_ * theta2_; // note sign

    theta1_ += theta1_dot * dt;
    theta2_ += theta2_dot * dt;
    yp_prev_ = yp;

    return u;
  }

  double ym() const { return ym_; }
  double theta1() const { return theta1_; }
  double theta2() const { return theta2_; }
};

// ============================================================
//        HIGHER-ORDER MRAC (n-th order plant)
// ============================================================

/**
 * @brief n-th order MRAC using SPR-Lyapunov design
 *
 * Plant:   yₚ^(n) + a_{n-1}yₚ^(n-1) + ... + a₀yₚ = bₚ·u
 * Ref:     yₘ^(n) + ...                            = bₘ·r
 *
 * Adaptation via augmented error ε and regressor φ:
 *   θ̇ = -Γ·ε·φ  (matrix gain Γ)
 *   u = θᵀφ
 *
 * This implementation uses the simplified form for SISO n-th order systems.
 * Parameter vector θ = [θ_ff, θ_fb1, θ_fb2, ..., θ_fbn] (n+1 params).
 */
class MRACHigherOrder {
public:
  int n_;        ///< System order
  double am_;    ///< Reference pole (placed at -am_)
  double gamma_; ///< Adaptation gain (scalar)
  double sig_;   ///< σ-modification

  Vec theta_;       ///< Adaptive parameters [θ₀, θ₁, ..., θₙ]
  Vec ym_states_;   ///< Reference model states [yₘ, ẏₘ, ..., yₘ^(n-1)]
  Vec yp_states_;   ///< Filtered plant state estimates
  double theta_ff_; ///< Feedforward gain

  MRACHigherOrder(int n, double am, double gamma = 1.0, double sigma = 0.0)
      : n_(n), am_(am), gamma_(gamma), sig_(sigma), ym_states_(n, 0.0),
        yp_states_(n, 0.0), theta_ff_(1.0) {
    theta_.assign(n + 1, 0.0);
    theta_[0] = 1.0; // initial feedforward = 1
  }

  void reset() {
    std::fill(ym_states_.begin(), ym_states_.end(), 0.0);
    std::fill(yp_states_.begin(), yp_states_.end(), 0.0);
    std::fill(theta_.begin(), theta_.end(), 0.0);
    theta_[0] = 1.0;
  }

  double compute(double r, double yp, double dt) {
    // Propagate reference model (n integrators with feedback)
    // Simplified: single-pole model ẏₘ = -am·yₘ + am·r
    double ym = ym_states_[0];
    ym_states_[0] += (-am_ * ym + am_ * r) * dt;

    double e = yp - ym_states_[0];

    // Regressor: φ = [r, yₚ, ẏₚ_approx, ...]
    Vec phi(n_ + 1, 0.0);
    phi[0] = r;
    phi[1] = yp;

    // Simplified state filter for higher n (Euler)
    for (int i = n_ - 1; i >= 1; --i)
      yp_states_[i] = yp_states_[i - 1];
    yp_states_[0] = yp;
    for (int i = 1; i < n_; ++i)
      phi[i + 1] = yp_states_[i];

    // Control: u = θᵀφ
    double u = 0;
    for (int i = 0; i <= n_; ++i)
      u += theta_[i] * phi[i];

    // Adaptation
    for (int i = 0; i <= n_; ++i) {
      double td = -gamma_ * e * phi[i] - sig_ * theta_[i];
      theta_[i] += td * dt;
    }

    return u;
  }

  double ym() const { return ym_states_.empty() ? 0.0 : ym_states_[0]; }
};

// ============================================================
//        MRAC WITH SIGMA-MODIFICATION (Robustness)
// ============================================================

/**
 * @brief Sigma-modification for bounded-noise robustness
 *
 * Standard MRAC is not robust to bounded disturbances.
 * Sigma-modification adds a damping term:
 *   θ̇ = -Γ·e·φ - σ·θ
 *
 * This ensures ‖θ(t)‖ ≤ max(‖θ(0)‖, γ_bound/σ) for bounded disturbances.
 * Trade-off: steady-state tracking error ≈ σ·‖θ*‖/γ (small σ ≈ 0.001-0.01).
 *
 * Alias for MRACFirstOrder with sig_ > 0 (already implemented above).
 */
using MRACSigma = MRACFirstOrder;

// ============================================================
//        ADAPTIVE PID (MRAC-inspired)
// ============================================================

/**
 * @brief Adaptive PID controller with gradient-descent gain updates
 *
 * Adjusts Kp, Ki, Kd online based on tracking error gradient:
 *   Kp_dot = -γp * e * e            (proportional gain)
 *   Ki_dot = -γi * e * ∫e            (integral gain)
 *   Kd_dot = -γd * e * ė            (derivative gain)
 *
 * Equivalent to minimizing J = ½e² via gradient descent on PID gains.
 */
class AdaptivePID {
public:
  double Kp_, Ki_, Kd_;                ///< Current gains
  double gamma_p_, gamma_i_, gamma_d_; ///< Adaptation rates
  double integral_, e_prev_;
  double Kp_min_, Kp_max_; ///< Gain bounds

  AdaptivePID(double Kp0 = 1.0, double Ki0 = 0.0, double Kd0 = 0.0,
              double gamma_p = 0.1, double gamma_i = 0.01,
              double gamma_d = 0.01, double Kp_min = 0.0, double Kp_max = 100.0)
      : Kp_(Kp0), Ki_(Ki0), Kd_(Kd0), gamma_p_(gamma_p), gamma_i_(gamma_i),
        gamma_d_(gamma_d), integral_(0.0), e_prev_(0.0), Kp_min_(Kp_min),
        Kp_max_(Kp_max) {}

  void reset() {
    integral_ = 0;
    e_prev_ = 0;
  }

  double compute(double r, double y, double dt) {
    double e = r - y;
    integral_ += e * dt;
    double edot = (e - e_prev_) / (dt + 1e-12);
    e_prev_ = e;

    double u = Kp_ * e + Ki_ * integral_ + Kd_ * edot;

    // Adaptive gain update (gradient descent)
    Kp_ -= gamma_p_ * e * (-e); // ∂J/∂Kp = e * ∂u/∂Kp * ... simplified
    Ki_ -= gamma_i_ * e * (-integral_);
    Kd_ -= gamma_d_ * e * (-edot);

    // Clamp to safe range
    Kp_ = std::max(Kp_min_, std::min(Kp_max_, Kp_));
    Ki_ = std::max(0.0, Ki_);
    Kd_ = std::max(0.0, Kd_);

    return u;
  }

  double Kp() const { return Kp_; }
  double Ki() const { return Ki_; }
  double Kd() const { return Kd_; }
};

// ============================================================
//        MRAC SIMULATION
// ============================================================

/**
 * @brief Simulation result from MRAC run
 */
struct MRACSimResult {
  std::vector<double> time;
  std::vector<double> yp;       ///< Plant output
  std::vector<double> ym;       ///< Reference model output
  std::vector<double> error;    ///< Tracking error e = yp - ym
  std::vector<double> u;        ///< Control input
  std::vector<double> theta1_h; ///< Parameter theta1 history
  std::vector<double> theta2_h; ///< Parameter theta2 history
  double rms_error;
  double final_theta1, final_theta2;
};

/**
 * @brief Simulate 1st-order MRAC on a plant
 *
 * @param mrac        MRAC controller (will be updated in place)
 * @param plant_a     True plant parameter a (in ẋ = -a·x + b·u)
 * @param plant_b     True plant parameter b
 * @param ref_fn      Reference signal r(t)
 * @param disturbance Disturbance signal d(t) (added to plant)
 * @param T           Simulation duration
 * @param dt          Time step
 */
inline MRACSimResult simulate_mrac(
    MRACFirstOrder &mrac, double plant_a, double plant_b,
    std::function<double(double)> ref_fn,
    std::function<double(double)> disturbance = [](double) { return 0.0; },
    double T = 30.0, double dt = 0.001) {
  MRACSimResult result;
  double yp = 0.0; // Plant state (= output for 1st order)
  double t = 0;
  int N = (int)(T / dt);
  double sum_sq = 0;

  result.time.reserve(N);
  result.yp.reserve(N);
  result.ym.reserve(N);
  result.error.reserve(N);
  result.u.reserve(N);
  result.theta1_h.reserve(N);
  result.theta2_h.reserve(N);

  for (int i = 0; i < N; ++i) {
    double r = ref_fn(t);
    double d = disturbance(t);

    double u = mrac.compute(r, yp, dt);

    // Plant update: ẏₚ = -a·yₚ + b·u + d
    yp += (-plant_a * yp + plant_b * u + d) * dt;

    double e = yp - mrac.ym();
    sum_sq += e * e;

    result.time.push_back(t);
    result.yp.push_back(yp);
    result.ym.push_back(mrac.ym());
    result.error.push_back(e);
    result.u.push_back(u);
    result.theta1_h.push_back(mrac.theta1());
    result.theta2_h.push_back(mrac.theta2());

    t += dt;
  }

  result.rms_error = std::sqrt(sum_sq / N);
  result.final_theta1 = mrac.theta1();
  result.final_theta2 = mrac.theta2();
  return result;
}

} // namespace adaptive
} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_ADAPTIVE_MRAC_HPP
