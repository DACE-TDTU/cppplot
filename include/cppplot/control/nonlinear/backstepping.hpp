/**
 * @file nonlinear/backstepping.hpp
 * @brief Recursive Backstepping Controller Design
 *
 * Recursive backstepping for strict-feedback nonlinear systems:
 *   ẋ₁ = f₁(x₁) + g₁(x₁)·x₂
 *   ẋ₂ = f₂(x₁,x₂) + g₂(x₁,x₂)·x₃
 *   ...
 *   ẋₙ = fₙ(x) + gₙ(x)·u
 *
 * References:
 *   - Krstic, Kanellakopoulos, Kokotovic (1995) "Nonlinear and Adaptive Control
 * Design"
 *   - Khalil (2002) "Nonlinear Systems", Ch. 14
 *   - Farrell & Polycarpou (2006) "Adaptive Approximation Based Control"
 */

#ifndef CPPPLOT_CONTROL_NONLINEAR_BACKSTEPPING_HPP
#define CPPPLOT_CONTROL_NONLINEAR_BACKSTEPPING_HPP

#include "../../pyplot.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <vector>


namespace cppplot {
namespace control {
namespace nonlinear {

using Vec = std::vector<double>;

// ============================================================
//   STEP RESULT (single backstepping step)
// ============================================================

/**
 * @brief Result from one backstepping step
 */
struct BacksteppingStep {
  double alpha;  ///< Virtual control α_k to make z_k dynamics stable
  double V;      ///< Lyapunov function contribution V_k = z_k²/2
  double z;      ///< Error z_k = x_k - α_{k-1}
  double k_gain; ///< Damping gain used at this step
};

// ============================================================
//   BACKSTEPPING DESIGNER (n-th order strict-feedback chain)
// ============================================================

/**
 * @brief Recursive backstepping for a pure-integrator chain with nonlinear
 * perturbations
 *
 * System form:
 *   ẋᵢ = fᵢ(x₁..xᵢ) + gᵢ(x₁..xᵢ) * x_{i+1},   i = 1..n-1
 *   ẋₙ = fₙ(x) + gₙ(x) * u
 *
 * The designer computes virtual controls α₁, α₂, ..., αₙ₋₁ and final u.
 * Gains [c₁, c₂, ..., cₙ] are tuning parameters (larger → faster but more
 * aggressive).
 */
class BacksteppingDesigner {
public:
  /// f functions: fᵢ(state) → scalar, i=0..n-1 (0-indexed)
  std::vector<std::function<double(Vec)>> f_;
  /// g functions: gᵢ(state) → scalar, i=0..n-1 (0-indexed)
  std::vector<std::function<double(Vec)>> g_;
  /// Damping gains cᵢ for each step
  std::vector<double> gains_;
  int n_; ///< System order

  /**
   * @param f_list   List of drift functions [f₁, f₂, ..., fₙ], each
   * f_i(full_state)
   * @param g_list   List of gain functions  [g₁, g₂, ..., gₙ], each
   * g_i(full_state)
   * @param gains    Damping gains c₁..cₙ (all positive, default = 1.0 each)
   */
  BacksteppingDesigner(std::vector<std::function<double(Vec)>> f_list,
                       std::vector<std::function<double(Vec)>> g_list,
                       std::vector<double> gains = {})
      : f_(f_list), g_(g_list), n_((int)f_list.size()) {
    if (g_list.size() != f_list.size())
      throw std::runtime_error(
          "BacksteppingDesigner: f and g lists must match in size");
    gains_ = gains.empty() ? std::vector<double>(n_, 1.0) : gains;
    if ((int)gains_.size() < n_)
      gains_.resize(n_, 1.0);
  }

  /**
   * @brief Compute backstepping control and Lyapunov value
   *
   * @param x       Full state [x₁, x₂, ..., xₙ]
   * @param y_ref   Reference for x₁ (output reference)
   * @param yd_ref  First derivative of reference
   * @param yd2_ref Second derivative (if needed for n>2)
   * @return        {control u, total Lyapunov V, vector of steps}
   */
  struct ComputeResult {
    double u;
    double V_total;
    std::vector<BacksteppingStep> steps;
  };

  ComputeResult compute(const Vec &x, double y_ref = 0.0, double yd_ref = 0.0,
                        double yd2_ref = 0.0) {
    if ((int)x.size() < n_)
      throw std::runtime_error("BacksteppingDesigner: state size mismatch");

    std::vector<BacksteppingStep> steps(n_);
    std::vector<double> alpha(n_ + 1, 0.0);
    alpha[0] = y_ref; // tracking: α₀ = y_ref

    double V_total = 0;

    // Reference derivatives (pre-define for up to 3rd order)
    std::vector<double> alpha_dot(n_ + 1, 0.0);
    alpha_dot[0] = yd_ref;

    for (int k = 0; k < n_; ++k) {
      // Step k: error zₖ₊₁ = x_{k+1} - αₖ
      double zk = x[k] - alpha[k];
      double Vk = 0.5 * zk * zk;
      V_total += Vk;

      // Virtual control for next step (or final u):
      // αₖ₊₁ = (1/gₖ) * (-fₖ - cₖ*zₖ - z_{k-1} + α̇ₖ)
      // The -z_{k-1} term is the back-coupling to previous step
      double z_prev = (k > 0) ? (x[k - 1] - alpha[k - 1]) : 0.0;

      double f_k = f_[k](x);
      double g_k = g_[k](x);

      if (std::abs(g_k) < 1e-10)
        throw std::runtime_error("BacksteppingDesigner: g = 0 at step " +
                                 std::to_string(k + 1));

      double alpha_dot_k = alpha_dot[k];

      // Virtual control: αₖ₊₁ = (1/gₖ)(-fₖ - cₖzₖ - z_{prev} + α̇ₖ)
      double alpha_next =
          (1.0 / g_k) * (-f_k - gains_[k] * zk - z_prev + alpha_dot_k);
      alpha[k + 1] = alpha_next;

      // Approximate α̇ₖ₊₁ for next step (first-order approximation)
      // For production code, use command filtering or analytic differentiation
      if (k < n_ - 1) {
        // Numeric differentiation approximation: α̇_{k+1} ≈ 0 (simplified)
        // Command-filtered version handles this properly
        alpha_dot[k + 1] = 0.0;
      }

      steps[k] = BacksteppingStep{alpha_next, Vk, zk, gains_[k]};
    }

    // Final control u = αₙ
    double u = alpha[n_];

    return ComputeResult{u, V_total, steps};
  }
};

// ============================================================
//   COMMAND-FILTERED BACKSTEPPING
// ============================================================

/**
 * @brief Command Filter (low-pass filter to smooth virtual controls)
 *
 * Replaces analytic differentiation of virtual controls with filtered versions.
 * ẋ_f = -ωc*(x_f - α)  (first-order command filter)
 *
 * This eliminates the "explosion of terms" problem in high-order backstepping.
 */
class CommandFilter {
public:
  double omega_c_;   ///< Filter bandwidth [rad/s]
  double state_;     ///< Filter state xf (tracks virtual control α)
  double state_dot_; ///< Filter output derivative

  explicit CommandFilter(double omega_c = 10.0)
      : omega_c_(omega_c), state_(0.0), state_dot_(0.0) {}

  void reset(double initial_value) {
    state_ = initial_value;
    state_dot_ = 0;
  }

  /**
   * @brief Update filter and return filtered value + derivative
   * @param alpha  Desired virtual control (unfiltered)
   * @param dt     Time step
   * @return       {filtered_alpha, filtered_alpha_dot}
   */
  std::pair<double, double> update(double alpha, double dt) {
    // ẋ_f = ωc (α - x_f)
    double xf_dot = omega_c_ * (alpha - state_);
    state_ += xf_dot * dt;
    state_dot_ = xf_dot;
    return {state_, state_dot_};
  }
};

/**
 * @brief Command-Filtered Backstepping Controller (n-th order)
 *
 * Uses command filters to avoid analytic differentiation of virtual controls.
 * States:
 *   x₁, ..., xₙ  — original states
 *   ξ₁, ..., ξₙ₋₁ — filtered virtual controls (filter states)
 */
class CommandFilteredBackstepping {
public:
  std::vector<std::function<double(Vec)>> f_, g_;
  std::vector<double> gains_;
  std::vector<CommandFilter> filters_;
  int n_;

  CommandFilteredBackstepping(std::vector<std::function<double(Vec)>> f_list,
                              std::vector<std::function<double(Vec)>> g_list,
                              std::vector<double> gains = {},
                              double filter_bandwidth = 20.0)
      : f_(f_list), g_(g_list), n_((int)f_list.size()) {
    gains_ = gains.empty() ? std::vector<double>(n_, 2.0) : gains;
    if ((int)gains_.size() < n_)
      gains_.resize(n_, 2.0);
    // One filter per step except the last
    for (int i = 0; i < n_ - 1; ++i)
      filters_.emplace_back(filter_bandwidth);
  }

  void reset(const Vec &x0, double y_ref = 0.0) {
    // Initialize filters at equilibrium
    for (auto &f : filters_)
      f.reset(y_ref);
  }

  /**
   * @brief Compute control and update filter states
   *
   * @param x    Current state
   * @param yr   Reference output
   * @param dt   Time step (for filter update)
   * @return     Control input u
   */
  double compute(const Vec &x, double yr, double dt) {
    if ((int)x.size() < n_)
      throw std::runtime_error(
          "CommandFilteredBackstepping: state size mismatch");

    // Step 1: z₁ = x₁ - yr, α₁ design
    double z1 = x[0] - yr;
    double f1 = f_[0](x), g1 = g_[0](x);
    double alpha1 = (1.0 / g1) * (-f1 - gains_[0] * z1);

    // Update filter 0: gives filtered α₁ and its derivative
    double alpha1_f, alpha1_fdot;
    if (!filters_.empty()) {
      auto [af, adot] = filters_[0].update(alpha1, dt);
      alpha1_f = af;
      alpha1_fdot = adot;
    } else {
      alpha1_f = alpha1;
      alpha1_fdot = 0;
    }

    if (n_ == 1)
      return alpha1;

    // Step 2: z₂ = x₂ - α₁_f
    double z2 = x[1] - alpha1_f;
    double f2 = f_[1](x), g2 = g_[1](x);
    double alpha2 = (1.0 / g2) * (-f2 - gains_[1] * z2 - z1 + alpha1_fdot);

    if (n_ == 2)
      return alpha2;

    // Update filter 1
    double alpha2_f, alpha2_fdot;
    if ((int)filters_.size() > 1) {
      auto [af, adot] = filters_[1].update(alpha2, dt);
      alpha2_f = af;
      alpha2_fdot = adot;
    } else {
      alpha2_f = alpha2;
      alpha2_fdot = 0;
    }

    // Step 3: z₃ = x₃ - α₂_f
    double z3 = x[2] - alpha2_f;
    double f3 = f_[2](x), g3 = g_[2](x);
    double alpha3 = (1.0 / g3) * (-f3 - gains_[2] * z3 - z2 + alpha2_fdot);

    if (n_ == 3)
      return alpha3;

    // For higher order, continue the pattern (up to order n_)
    // (Simplified: returns alpha3 for now; extend for n>3)
    return alpha3;
  }
};

// ============================================================
//   ADAPTIVE BACKSTEPPING (with parameter estimation)
// ============================================================

/**
 * @brief Adaptive Backstepping for SISO 2nd-order system with unknown parameter
 *
 * System: ẋ₁ = x₂,  ẋ₂ = θ·φ(x) + u
 * where θ is unknown constant, φ(x) is known regressor.
 *
 * Combined controller + adaptation law (Lyapunov-based):
 *   u = -(c₁+c₂)z₂ - c₁c₂x₁ - θ̂·φ(x) + c₁x₂
 *   θ̂̇ = γ · z₂ · φ(x)     (gradient update on z₂)
 */
class AdaptiveBackstepping2 {
public:
  double c1_, c2_;   ///< Backstepping gains
  double gamma_;     ///< Adaptation rate
  double theta_hat_; ///< Parameter estimate
  double sigma_;     ///< σ-modification coefficient (0 = no modification)

  AdaptiveBackstepping2(double c1 = 1.0, double c2 = 1.0, double gamma = 1.0,
                        double theta_hat0 = 0.0, double sigma = 0.0)
      : c1_(c1), c2_(c2), gamma_(gamma), theta_hat_(theta_hat0), sigma_(sigma) {
  }

  /**
   * @brief Compute control and update parameter estimate
   *
   * @param x     State [x₁, x₂]
   * @param phi   Regressor function φ(x) evaluated at current x
   * @param yr    Reference
   * @param dt    Time step
   * @return      Control input u
   */
  double compute(const Vec &x, double phi, double yr = 0.0, double dt = 0.01) {
    // Backstepping errors
    double z1 = x[0] - yr;
    double alpha1 = -c1_ * z1; // virtual control
    double z2 = x[1] - alpha1;

    // alpha1_dot ≈ -c1 * x2 (chain rule, ignoring yr_dot for simplicity)
    double alpha1_dot = -c1_ * x[1];

    // Final control
    double u = -c2_ * z2 - z1 - theta_hat_ * phi + alpha1_dot;

    // Parameter adaptation: θ̂̇ = γ z₂ φ - σ θ̂  (σ-modification for robustness)
    double theta_dot = gamma_ * z2 * phi - sigma_ * theta_hat_;
    theta_hat_ += theta_dot * dt;

    return u;
  }

  double theta_hat() const { return theta_hat_; }
};

// ============================================================
//   SIMULATION HELPER
// ============================================================

/**
 * @brief Simulate backstepping-controlled system
 */
struct BacksteppingSimResult {
  std::vector<double> time, x1, x2, x3, u, V;
  double rms_error;
};

inline BacksteppingSimResult
simulate_backstepping(BacksteppingDesigner &bs,
                      std::vector<std::function<double(Vec, double)>>
                          plant_f, // ẋᵢ = f_plant(x,u)
                      Vec x0, double yr, double T = 10.0, double dt = 0.01) {
  BacksteppingSimResult result;
  Vec x = x0;
  double t = 0;
  int N = (int)(T / dt);
  double sum_sq = 0;
  result.time.reserve(N);
  result.x1.reserve(N);
  result.x2.reserve(N);
  result.u.reserve(N);
  result.V.reserve(N);

  for (int i = 0; i < N; ++i) {
    auto cr = bs.compute(x, yr);
    double u = cr.u;
    double e = x[0] - yr;
    sum_sq += e * e;

    result.time.push_back(t);
    result.x1.push_back(x[0]);
    if (x.size() > 1)
      result.x2.push_back(x[1]);
    if (x.size() > 2)
      result.x3.push_back(x[2]);
    result.u.push_back(u);
    result.V.push_back(cr.V_total);

    // RK4
    auto sys = [&](Vec xs) -> Vec {
      Vec dx(xs.size());
      for (size_t j = 0; j < std::min(xs.size(), plant_f.size()); ++j)
        dx[j] = plant_f[j](xs, u);
      return dx;
    };
    Vec k1 = sys(x);
    Vec xk2(x.size());
    for (size_t j = 0; j < x.size(); ++j)
      xk2[j] = x[j] + 0.5 * dt * k1[j];
    Vec k2 = sys(xk2);
    Vec xk3(x.size());
    for (size_t j = 0; j < x.size(); ++j)
      xk3[j] = x[j] + 0.5 * dt * k2[j];
    Vec k3 = sys(xk3);
    Vec xk4(x.size());
    for (size_t j = 0; j < x.size(); ++j)
      xk4[j] = x[j] + dt * k3[j];
    Vec k4 = sys(xk4);
    for (size_t j = 0; j < x.size(); ++j)
      x[j] += dt / 6.0 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);
    t += dt;
  }
  result.rms_error = std::sqrt(sum_sq / N);
  return result;
}

} // namespace nonlinear
} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_NONLINEAR_BACKSTEPPING_HPP
