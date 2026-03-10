/**
 * @file nonlinear/cbf.hpp
 * @brief Control Barrier Functions (CBF) for Safety-Critical Control
 *
 * A Control Barrier Function (CBF) h(x) ≥ 0 defines a safe set S.
 * The CBF condition enforces forward invariance of S by requiring:
 *   ḣ(x, u) + α·h(x) ≥ 0   (CBF constraint)
 *
 * This file provides:
 *   - CBF-QP Safety Filter: min-modification of any nominal controller
 *   - Multi-CBF Filter: intersection of multiple safe sets
 *   - CLF + CBF unified controller (stability + safety)
 *   - Factory functions for common safe set shapes
 *   - Monte Carlo safety verification
 *
 * References:
 *   - Ames et al. (2019) "Control barrier functions: Theory and applications"
 *   - Ames et al. (2017) "Control Barrier Function Based Quadratic Programs"
 *   - Prajna & Jadbabaie (2004) "Safety verification of hybrid systems"
 *
 * Usage:
 * @code
 * #include <cppplot/control/nonlinear/cbf.hpp>
 * using namespace cppplot::control::nonlinear;
 *
 * // Safe set: |x| ≤ 1.5 → h(x) = 1.5² - x²
 * auto h    = [](Vec x)->double { return 2.25 - x[0]*x[0]; };
 * auto Lfh  = [](Vec x)->double { return -2*x[0]*x[1]; };
 * auto Lgh  = [](Vec x)->double { return -2*x[0]; };
 *
 * CBFQPFilter safe(h, Lfh, Lgh, 1.0);
 * double u_safe = safe.filter(x, u_nominal);
 * @endcode
 */

#ifndef CPPPLOT_CONTROL_NONLINEAR_CBF_HPP
#define CPPPLOT_CONTROL_NONLINEAR_CBF_HPP

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
namespace nonlinear {

using Vec = std::vector<double>;

// ============================================================
//          CONTROL BARRIER FUNCTION WRAPPER
// ============================================================

/**
 * @brief Control Barrier Function descriptor
 *
 * Encapsulates h(x) along with its Lie derivatives along f and g:
 *   L_f h(x) = ∇h · f(x)   (drift component)
 *   L_g h(x) = ∇h · g(x)   (input component)
 *
 * For affine system ẋ = f(x) + g(x)u:
 *   ḣ = L_f h(x) + L_g h(x) · u
 *
 * CBF condition: L_f h(x) + L_g h(x)·u + α(h(x)) ≥ 0
 * where α is a class K function (e.g., α(s) = k·s).
 */
struct ControlBarrierFunction {
  std::function<double(Vec)> h;   ///< Barrier function: h(x) ≥ 0 = safe set
  std::function<double(Vec)> Lfh; ///< L_f h(x): Lie derivative of h along f
  std::function<double(Vec)> Lgh; ///< L_g h(x): Lie derivative of h along g

  double alpha; ///< Class K coefficient: α(s) = alpha·s (linear)
  std::string name = "CBF";

  /**
   * @param h_fn     Barrier function
   * @param Lfh_fn   Lie derivative of h along f
   * @param Lgh_fn   Lie derivative of h along g
   * @param alpha_   Class K function gain (α(s) = alpha_·s)
   */
  ControlBarrierFunction(std::function<double(Vec)> h_fn,
                         std::function<double(Vec)> Lfh_fn,
                         std::function<double(Vec)> Lgh_fn, double alpha_ = 1.0)
      : h(h_fn), Lfh(Lfh_fn), Lgh(Lgh_fn), alpha(alpha_) {}

  /// Evaluate CBF value at x
  double value(const Vec &x) const { return h(x); }

  /// Check if x is in safe set
  bool is_safe(const Vec &x) const { return h(x) >= 0; }

  /// CBF constraint: L_f h + L_g h · u ≥ -α·h
  /// Returns the minimum required u when L_g h > 0
  double min_safe_u(const Vec &x) const {
    double lgv = Lgh(x);
    if (std::abs(lgv) < 1e-12)
      return -1e10; // constraint inactive
    return -(Lfh(x) + alpha * h(x)) / lgv;
  }

  /// Check if constraint is active (L_g h ≠ 0)
  bool is_active(const Vec &x) const { return std::abs(Lgh(x)) > 1e-10; }
};

// ============================================================
//          CBF-QP SAFETY FILTER (SISO/SCALAR CONTROL)
// ============================================================

/**
 * @brief CBF-QP Safety Filter for scalar control input
 *
 * Minimally modifies a nominal control u_nom to satisfy CBF constraint:
 *   min ‖u - u_nom‖²
 *   s.t. L_f h(x) + L_g h(x)·u + α·h(x) ≥ 0
 *
 * For SISO (one CBF, scalar u), closed-form solution:
 *   if L_g h·u_nom + L_f h + α·h ≥ 0: u_safe = u_nom (safe)
 *   else: u_safe = u_nom - (L_g h·u_nom + L_f h + α·h) / (L_g h)²  · L_g h
 *               = -(L_f h + α·h) / L_g h  (minimum modification)
 */
class CBFQPFilter {
public:
  ControlBarrierFunction cbf_;
  double u_min_, u_max_;    ///< Optional control bounds
  mutable bool was_active_; ///< Was CBF active at last call?
  mutable double last_h_;   ///< Last h(x) value

  CBFQPFilter(ControlBarrierFunction cbf, double u_min = -1e6,
              double u_max = 1e6)
      : cbf_(cbf), u_min_(u_min), u_max_(u_max), was_active_(false),
        last_h_(0.0) {}

  /**
   * @brief Apply CBF safety filter
   *
   * @param x      Current state
   * @param u_nom  Nominal (unsafe) control
   * @return       Safe control u_safe
   */
  double filter(const Vec &x, double u_nom) const {
    last_h_ = cbf_.h(x);
    double Lf = cbf_.Lfh(x);
    double Lg = cbf_.Lgh(x);
    double hx = last_h_;

    // CBF constraint value at u_nom: Lf_h + Lg_h·u_nom + α·h
    double cbf_constraint = Lf + Lg * u_nom + cbf_.alpha * hx;

    if (cbf_constraint >= 0) {
      was_active_ = false;
      return std::max(u_min_, std::min(u_max_, u_nom));
    }

    // Constraint violated: project to constraint boundary
    was_active_ = true;
    if (std::abs(Lg) < 1e-10) {
      // L_g h = 0: can't satisfy constraint with input
      // Return nominal (nothing we can do)
      return std::max(u_min_, std::min(u_max_, u_nom));
    }

    // Minimum-norm solution: u_safe = u_nom + λ*Lg where
    // λ = -(Lf + Lg*u_nom + α*h) / (Lg²)
    double lambda = -cbf_constraint / (Lg * Lg);
    double u_safe = u_nom + lambda * Lg;
    return std::max(u_min_, std::min(u_max_, u_safe));
  }

  bool was_active() const { return was_active_; }
  double barrier_value() const { return last_h_; }
};

// ============================================================
//          MULTI-CBF FILTER (multiple simultaneous constraints)
// ============================================================

/**
 * @brief Safety filter with multiple CBF constraints (set intersection)
 *
 * Handles m CBF constraints simultaneously via a simple QP:
 *   min ‖u - u_nom‖²
 *   s.t. L_f hᵢ + L_g hᵢ·u + αᵢ·hᵢ ≥ 0,  i = 1..m
 *
 * For SISO scalar u, uses iterative projection (Dykstra's algorithm).
 * Each projection onto a half-line: {u : aᵢ·u ≥ bᵢ}.
 */
class MultiCBFFilter {
public:
  std::vector<ControlBarrierFunction> cbfs_;
  double u_min_, u_max_;

  explicit MultiCBFFilter(double u_min = -1e6, double u_max = 1e6)
      : u_min_(u_min), u_max_(u_max) {}

  void add_cbf(ControlBarrierFunction cbf) { cbfs_.push_back(std::move(cbf)); }

  /**
   * @brief Filter nominal control to satisfy all CBF constraints
   *
   * @param x      Current state
   * @param u_nom  Nominal control
   * @return       Safe control (satisfies all CBF constraints)
   */
  double filter(const Vec &x, double u_nom) const {
    double u = u_nom;

    // Iterative projection: project onto each constraint in sequence
    // Repeat until convergence (typically 10-20 iterations)
    for (int iter = 0; iter < 30; ++iter) {
      double u_prev = u;
      for (const auto &cbf : cbfs_) {
        double Lf = cbf.Lfh(x);
        double Lg = cbf.Lgh(x);
        double hx = cbf.h(x);
        double constraint = Lf + Lg * u + cbf.alpha * hx;
        if (constraint < 0 && std::abs(Lg) > 1e-10) {
          // Project: u ← u - constraint/Lg² * Lg
          u = u - constraint / (Lg * Lg) * Lg;
        }
      }
      // Clamp to control bounds
      u = std::max(u_min_, std::min(u_max_, u));
      if (std::abs(u - u_prev) < 1e-8)
        break; // converged
    }
    return u;
  }

  /**
   * @brief Check which constraints are violated at current state/control
   */
  std::vector<bool> check_violations(const Vec &x, double u) const {
    std::vector<bool> violated;
    for (const auto &cbf : cbfs_) {
      double val = cbf.Lfh(x) + cbf.Lgh(x) * u + cbf.alpha * cbf.h(x);
      violated.push_back(val < -1e-8);
    }
    return violated;
  }
};

// ============================================================
//          CLF + CBF UNIFIED CONTROLLER
// ============================================================

/**
 * @brief CLF-CBF QP Controller (stability + safety)
 *
 * Simultaneously satisfies:
 *   CLF constraint: L_f V + L_g V·u + γ·V ≤ δ  (relaxed: δ ≥ 0)
 *   CBF constraint: L_f h + L_g h·u + α·h ≥ 0  (hard: safety)
 *
 * Objective: min ½‖u - u_nom‖² + p·δ²
 * For SISO with analytic solution (when CLF and CBF intersect).
 */
class CLFCBFController {
public:
  /// CLF: V(x), L_f V, L_g V; CLF gain γ
  std::function<double(Vec)> V_, LfV_, LgV_;
  double clf_gamma_, clf_penalty_;

  /// CBF:
  ControlBarrierFunction cbf_;

  double u_min_, u_max_;

  CLFCBFController(std::function<double(Vec)> V, std::function<double(Vec)> LfV,
                   std::function<double(Vec)> LgV, double clf_gamma,
                   ControlBarrierFunction cbf, double clf_penalty = 100.0,
                   double u_min = -1e6, double u_max = 1e6)
      : V_(V), LfV_(LfV), LgV_(LgV), clf_gamma_(clf_gamma),
        clf_penalty_(clf_penalty), cbf_(cbf), u_min_(u_min), u_max_(u_max) {}

  /**
   * @brief Compute safe stabilizing control
   *
   * Uses analytic solution to the 2-constraint QP (SISO case).
   * CBF is always hard-enforced; CLF is soft (with penalty on slack).
   *
   * @param x   Current state
   * @return    {u_safe, clf_satisfied, cbf_satisfied}
   */
  struct CLFCBFResult {
    double u;
    bool clf_active; ///< CLF constraint was active
    bool cbf_active; ///< CBF constraint was active
    double V_val;    ///< Current Lyapunov value
    double h_val;    ///< Current barrier value
  };

  CLFCBFResult compute(const Vec &x) const {
    CLFCBFResult res;
    res.V_val = V_(x);
    res.h_val = cbf_.h(x);

    double LfV = LfV_(x), LgV = LgV_(x);
    double Lfh = cbf_.Lfh(x), Lgh = cbf_.Lgh(x);

    // Start with CLF-optimal u (ignore CBF first)
    double u_clf = 0.0;
    if (std::abs(LgV) > 1e-10) {
      // Sontag's formula for CLF
      double num = LfV + clf_gamma_ * res.V_val;
      if (num > 0)
        u_clf =
            -(num + std::sqrt(num * num + LgV * LgV * LgV * LgV)) / (2 * LgV);
    }

    res.clf_active = false;
    res.cbf_active = false;

    // Check CBF constraint at u_clf
    double cbf_val = Lfh + Lgh * u_clf + cbf_.alpha * res.h_val;
    if (cbf_val >= 0) {
      // CLF and CBF both satisfied
      res.u = std::max(u_min_, std::min(u_max_, u_clf));
      res.clf_active = true;
      return res;
    }

    // CBF violated: find point on CBF boundary
    if (std::abs(Lgh) < 1e-10) {
      res.u = std::max(u_min_, std::min(u_max_, u_clf));
      return res;
    }

    // Minimum-norm u satisfying CBF (ignoring CLF)
    double u_cbf = -(Lfh + cbf_.alpha * res.h_val) / Lgh;
    res.cbf_active = true;

    // Check if CLF can still be satisfied with u_cbf
    bool clf_ok = (LfV + LgV * u_cbf + clf_gamma_ * res.V_val <= 0);
    res.clf_active = clf_ok;
    res.u = std::max(u_min_, std::min(u_max_, u_cbf));
    return res;
  }
};

// ============================================================
//          FACTORY: COMMON SAFE SET SHAPES
// ============================================================

/**
 * @brief Create CBF for a 1D interval safe set: a ≤ x₁ ≤ b
 *
 * h_lower(x) = x₁ - a
 * h_upper(x) = b - x₁
 *
 * @param state_index  Which state to constrain (0-indexed)
 * @param a, b         Lower and upper bounds
 * @param alpha        Class K coefficient
 */
inline std::vector<ControlBarrierFunction>
cbf_from_interval(int state_index, double a, double b, double alpha = 1.0,
                  std::function<Vec(Vec)> f = nullptr,
                  std::function<Vec(Vec)> g = nullptr) {
  // Lower bound: h₁ = x - a
  auto Lfh_lower = [state_index, f](Vec x) -> double {
    if (!f)
      return 0.0;
    Vec fv = f(x);
    return (state_index < (int)fv.size()) ? fv[state_index] : 0.0;
  };
  auto Lgh_lower = [state_index, g](Vec x) -> double {
    if (!g)
      return 1.0;
    Vec gv = g(x);
    return (state_index < (int)gv.size()) ? gv[state_index] : 0.0;
  };

  ControlBarrierFunction h_lower(
      [state_index, a](Vec x) {
        return (state_index < (int)x.size()) ? x[state_index] - a : 0.0;
      },
      Lfh_lower, Lgh_lower, alpha);
  h_lower.name = "lower_bound";

  // Upper bound: h₂ = b - x
  ControlBarrierFunction h_upper(
      [state_index, b](Vec x) {
        return (state_index < (int)x.size()) ? b - x[state_index] : 0.0;
      },
      [state_index, f](Vec x) -> double {
        if (!f)
          return 0.0;
        Vec fv = f(x);
        return (state_index < (int)fv.size()) ? -fv[state_index] : 0.0;
      },
      [state_index, g](Vec x) -> double {
        if (!g)
          return -1.0;
        Vec gv = g(x);
        return (state_index < (int)gv.size()) ? -gv[state_index] : 0.0;
      },
      alpha);
  h_upper.name = "upper_bound";

  return {h_lower, h_upper};
}

/**
 * @brief Create CBF for a circular safe set: ‖x - center‖ ≤ r
 *
 * h(x) = r² - (x₁-cx)² - (x₂-cy)²
 * Lfh = -2(x₁-cx)f₁ - 2(x₂-cy)f₂
 * Lgh = -2(x₁-cx)g₁ - 2(x₂-cy)g₂  (for SISO, g is a 2-vector)
 *
 * @param center_x, center_y  Center of safe circle
 * @param radius               Safe radius
 * @param f                    Drift field (optional for Lfh)
 * @param g                    Input field (optional for Lgh)
 * @param alpha                Class K coefficient
 */
inline ControlBarrierFunction
cbf_from_circle(double center_x, double center_y, double radius,
                std::function<Vec(Vec)> f = nullptr,
                std::function<Vec(Vec)> g = nullptr, double alpha = 1.0) {
  double r2 = radius * radius;

  auto h = [center_x, center_y, r2](Vec x) -> double {
    double dx = x[0] - center_x, dy = x[1] - center_y;
    return r2 - dx * dx - dy * dy;
  };

  auto Lfh = [center_x, center_y, f](Vec x) -> double {
    if (!f)
      return 0.0;
    Vec fv = f(x);
    double dx = x[0] - center_x, dy = (x.size() > 1) ? x[1] - center_y : 0.0;
    double val = 0;
    if (fv.size() > 0)
      val -= 2 * dx * fv[0];
    if (fv.size() > 1)
      val -= 2 * dy * fv[1];
    return val;
  };

  auto Lgh = [center_x, center_y, g](Vec x) -> double {
    if (!g) {
      // Default: assume g affects x₁ only (SISO)
      return -2 * (x[0] - center_x);
    }
    Vec gv = g(x);
    double dx = x[0] - center_x, dy = (x.size() > 1) ? x[1] - center_y : 0.0;
    double val = 0;
    if (gv.size() > 0)
      val -= 2 * dx * gv[0];
    if (gv.size() > 1)
      val -= 2 * dy * gv[1];
    return val;
  };

  ControlBarrierFunction cbf(h, Lfh, Lgh, alpha);
  cbf.name = "circle";
  return cbf;
}

// ============================================================
//          SAFETY VERIFICATION (MONTE CARLO)
// ============================================================

/**
 * @brief Monte Carlo safety verification for a CBF-filtered system
 *
 * Runs N random simulations and checks if h(x(t)) ≥ 0 at all times.
 *
 * @param f_closed   Closed-loop dynamics ẋ = f_closed(x, t)
 * @param cbf        Barrier function to check
 * @param x0_sampler Function to sample initial conditions
 * @param N          Number of Monte Carlo trials
 * @param T          Simulation horizon
 * @param dt         Time step
 */
struct SafetyVerificationResult {
  int N_total;
  int N_safe;         ///< Trials where h(x(t)) ≥ 0 always
  int N_violated;     ///< Trials with a violation
  double min_h;       ///< Minimum h seen across all trials (most violated)
  double safety_rate; ///< N_safe / N_total

  void print() const {
    std::cout << "== Safety Verification (" << N_total << " trials) ==\n";
    std::cout << "  Safe trials:    " << N_safe << "\n";
    std::cout << "  Violated:       " << N_violated << "\n";
    std::cout << "  Safety rate:    " << 100.0 * safety_rate << "%\n";
    std::cout << "  Min h observed: " << min_h << "\n";
  }
};

inline SafetyVerificationResult
verify_safety_monte_carlo(std::function<Vec(Vec, double)> f_closed,
                          const ControlBarrierFunction &cbf,
                          std::function<Vec()> x0_sampler, int N = 500,
                          double T = 10.0, double dt = 0.01) {
  SafetyVerificationResult result{N, 0, 0, 1e9, 0.0};

  for (int trial = 0; trial < N; ++trial) {
    Vec x = x0_sampler();
    bool violated = false;
    double t = 0;
    int steps = (int)(T / dt);

    for (int k = 0; k < steps && !violated; ++k) {
      double hv = cbf.h(x);
      result.min_h = std::min(result.min_h, hv);
      if (hv < -1e-6) {
        violated = true;
        break;
      }

      Vec k1 = f_closed(x, t);
      Vec xk2(x.size());
      for (size_t j = 0; j < x.size(); ++j)
        xk2[j] = x[j] + 0.5 * dt * k1[j];
      Vec k2 = f_closed(xk2, t + 0.5 * dt);
      Vec xk3(x.size());
      for (size_t j = 0; j < x.size(); ++j)
        xk3[j] = x[j] + 0.5 * dt * k2[j];
      Vec k3 = f_closed(xk3, t + 0.5 * dt);
      Vec xk4(x.size());
      for (size_t j = 0; j < x.size(); ++j)
        xk4[j] = x[j] + dt * k3[j];
      Vec k4 = f_closed(xk4, t + dt);
      for (size_t j = 0; j < x.size(); ++j)
        x[j] += dt / 6.0 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);
      t += dt;
    }

    if (!violated)
      result.N_safe++;
    else
      result.N_violated++;
  }

  result.safety_rate = (double)result.N_safe / N;
  return result;
}

} // namespace nonlinear
} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_NONLINEAR_CBF_HPP
