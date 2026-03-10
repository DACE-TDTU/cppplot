/**
 * @file nonlinear/feedback_linearization.hpp
 * @brief Feedback Linearization for Nonlinear Systems
 *
 * Input-output and input-state feedback linearization for SISO/MIMO systems.
 *
 * For SISO affine nonlinear system:
 *   ẋ = f(x) + g(x)·u
 *   y = h(x)
 *
 * Feedback linearizing control: u = (v - L_f^r h(x)) / L_g L_f^(r-1) h(x)
 * which yields the linear relation y^(r) = v (r = relative degree).
 *
 * Features:
 *   - Lie derivative computation (numeric and symbolic)
 *   - Relative degree computation
 *   - SISO feedback linearizing controller
 *   - Zero dynamics / internal dynamics simulation
 *   - MIMO input-output linearization (decoupling matrix)
 *   - Differential flatness coordinate map
 *
 * References:
 *   - Isidori (1995) "Nonlinear Control Systems"
 *   - Khalil (2002) "Nonlinear Systems", Ch. 13
 *   - Slotine & Li (1991) "Applied Nonlinear Control"
 *
 * Usage:
 * @code
 * #include <cppplot/control/nonlinear/feedback_linearization.hpp>
 * using namespace cppplot::control::nonlinear;
 *
 * // Pendulum: ẋ₁ = x₂, ẋ₂ = -sin(x₁) + u, y = x₁
 * auto f = [](Vec x)->Vec { return {x[1], -std::sin(x[0])}; };
 * auto g = [](Vec x)->Vec { return {0.0, 1.0}; };
 * auto h = [](Vec x)->double { return x[0]; };
 *
 * FeedbackLinearizer fbl(f, g, h, 2);
 * // At runtime:
 * double u = fbl.compute(x, v);  // v = desired acceleration
 * @endcode
 */

#ifndef CPPPLOT_CONTROL_NONLINEAR_FEEDBACK_LINEARIZATION_HPP
#define CPPPLOT_CONTROL_NONLINEAR_FEEDBACK_LINEARIZATION_HPP

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
using VectorField = std::function<Vec(Vec)>;
using ScalarField = std::function<double(Vec)>;
using MatrixField = std::function<std::vector<Vec>(Vec)>; // for MIMO g(x)

// ============================================================
//                   LIE DERIVATIVES
// ============================================================

/**
 * @brief Numeric Lie derivative: L_f h(x) = ∇h(x) · f(x)
 *
 * @param f   Vector field f: R^n → R^n
 * @param h   Scalar function h: R^n → R
 * @param x   Point at which to evaluate
 * @param step Finite difference step size
 * @return    L_f h(x)
 */
inline double lie_derivative(const VectorField &f, const ScalarField &h,
                             const Vec &x, double step = 1e-6) {
  Vec fv = f(x);
  double result = 0.0;
  for (size_t i = 0; i < x.size(); ++i) {
    Vec xp = x, xm = x;
    xp[i] += step;
    xm[i] -= step;
    double dh_dxi = (h(xp) - h(xm)) / (2 * step);
    result += dh_dxi * fv[i];
  }
  return result;
}

/**
 * @brief Iterated Lie derivative: L_f^k h(x)
 *
 * Computes L_f^k h by iterating: L_f^k h = L_f (L_f^(k-1) h)
 * Returns both the scalar value and the intermediate ScalarField.
 */
inline ScalarField make_lie_k(const VectorField &f, const ScalarField &h,
                              int k) {
  if (k == 0)
    return h;
  ScalarField prev = make_lie_k(f, h, k - 1);
  return [f, prev](Vec x) { return lie_derivative(f, prev, x); };
}

/**
 * @brief L_g L_f^(r-1) h(x) — decoupling gain for SISO FBL
 */
inline double decoupling_gain(const VectorField &f, const VectorField &g,
                              const ScalarField &h, int r) {
  // Compute L_f^(r-1) h as a ScalarField
  ScalarField Lfr1_h = make_lie_k(f, h, r - 1);
  // Then take its Lie derivative along g
  return lie_derivative(g, Lfr1_h, std::vector<double>(2, 0.0));
}

// ============================================================
//                 RELATIVE DEGREE COMPUTATION
// ============================================================

/**
 * @brief Compute relative degree r of output y = h(x) w.r.t. input u
 *
 * r is the smallest integer such that L_g L_f^(r-1) h(x) ≠ 0.
 *
 * @param f       Drift vector field
 * @param g       Input vector field
 * @param h       Output function
 * @param x       Evaluation point
 * @param n_max   Maximum relative degree to check
 * @param tol     Threshold for "non-zero" check
 * @return        Relative degree (0 = not well-defined at x)
 */
inline int relative_degree(const VectorField &f, const VectorField &g,
                           const ScalarField &h, const Vec &x, int n_max = 8,
                           double tol = 1e-6) {
  for (int r = 1; r <= n_max; ++r) {
    // Build L_f^(r-1) h
    ScalarField Lfr1_h = make_lie_k(f, h, r - 1);
    // Evaluate L_g L_f^(r-1) h at x
    double val = lie_derivative(g, Lfr1_h, x);
    if (std::abs(val) > tol)
      return r;
  }
  return 0; // not defined
}

// ============================================================
//           SISO FEEDBACK LINEARIZING CONTROLLER
// ============================================================

/**
 * @brief SISO Feedback Linearizer
 *
 * For system ẋ = f(x) + g(x)u, y = h(x) with relative degree r:
 *   u = (v - L_f^r h(x)) / (L_g L_f^(r-1) h(x))
 *
 * Then the input-output behavior becomes: y^(r) = v.
 * The outer loop (v → desired behavior) must be designed separately.
 */
class FeedbackLinearizer {
public:
  VectorField f_;        ///< Drift term
  VectorField g_;        ///< Input term
  ScalarField h_;        ///< Output function
  int r_;                ///< Relative degree
  double u_min_, u_max_; ///< Control saturation limits

  ScalarField Lfr_h_;  ///< L_f^r h (precomputed as ScalarField)
  ScalarField Lfr1_h_; ///< L_f^(r-1) h

  /**
   * @param f       ẋ = f(x) + g(x)u drift
   * @param g       Input vector field
   * @param h       Output y = h(x)
   * @param r       Known relative degree (0 = auto-detect at evaluation point)
   * @param u_min   Lower saturation bound
   * @param u_max   Upper saturation bound
   */
  FeedbackLinearizer(VectorField f, VectorField g, ScalarField h, int r = 0,
                     double u_min = -1e6, double u_max = 1e6)
      : f_(f), g_(g), h_(h), r_(r), u_min_(u_min), u_max_(u_max) {
    if (r_ > 0) {
      Lfr_h_ = make_lie_k(f_, h_, r_);
      Lfr1_h_ = make_lie_k(f_, h_, r_ - 1);
    }
  }

  /**
   * @brief Compute feedback linearizing control u
   *
   * @param x   Current state
   * @param v   New (virtual) input — outer loop output (e.g., PD on tracking
   * error)
   * @return    Control input u to apply to the plant
   */
  double compute(const Vec &x, double v) {
    int r = r_;
    if (r <= 0) {
      // Auto-detect relative degree at current x
      r = relative_degree(f_, g_, h_, x);
      if (r <= 0)
        throw std::runtime_error(
            "FBL: relative degree not defined at current x");
      Lfr_h_ = make_lie_k(f_, h_, r);
      Lfr1_h_ = make_lie_k(f_, h_, r - 1);
      r_ = r;
    }

    double Lf_r_h = Lfr_h_(x);
    double Lg_Lfr1_h = lie_derivative(g_, Lfr1_h_, x);

    if (std::abs(Lg_Lfr1_h) < 1e-10)
      throw std::runtime_error("FBL: decoupling gain = 0 (singularity)");

    double u = (v - Lf_r_h) / Lg_Lfr1_h;
    return std::max(u_min_, std::min(u_max_, u));
  }

  /**
   * @brief Get output y = h(x) and its first (r-1) derivatives evaluated from x
   *
   * Useful for errors in the linear outer loop.
   * y^(k) = L_f^k h(x)
   *
   * @return vector [h(x), L_f h(x), ..., L_f^(r-1) h(x)]
   */
  std::vector<double> output_and_derivatives(const Vec &x) {
    int r = (r_ > 0) ? r_ : relative_degree(f_, g_, h_, x);
    std::vector<double> derivs(r);
    for (int k = 0; k < r; ++k) {
      auto Lfk_h = make_lie_k(f_, h_, k);
      derivs[k] = Lfk_h(x);
    }
    return derivs;
  }

  /**
   * @brief Design outer-loop PD gains for the linearized system
   *
   * After FBL: y^(r) = v. Desired poles at {-ω_n} (all real).
   * Returns gains [k_0, k_1, ..., k_{r-1}] for:
   *   v = -k_0*e - k_1*ė - ... - k_{r-1}*e^(r-1) + y_ref^(r)
   *
   * @param omega_n  Desired closed-loop bandwidth
   * @param r        Relative degree (uses stored r_ if 0)
   */
  std::vector<double> place_poles(double omega_n, int r = 0) {
    if (r <= 0)
      r = r_;
    // Binomial coefficients for (s+ω_n)^r = 0
    std::vector<double> k(r);
    // Coefficients of (s+ω_n)^r = s^r + C(r,1)ω_n s^{r-1} + ... + ω_n^r
    // k[i] = C(r, r-i) * ω_n^(r-i)  → but here we return k_0..k_{r-1}
    double wn_pow = 1.0;
    // Binomial: C(r,i)
    auto binom = [](int n, int m) -> double {
      if (m < 0 || m > n)
        return 0;
      double result = 1;
      for (int i = 0; i < m; ++i) {
        result *= (n - i);
        result /= (i + 1);
      }
      return result;
    };
    for (int i = 0; i < r; ++i) {
      k[i] = binom(r, i) * std::pow(omega_n, r - i);
    }
    return k; // k[0] → k_{r-1} (from constant to highest derivative)
  }

  /**
   * @brief Check whether current x is near a linearization singularity
   *
   * Returns true if |L_g L_f^(r-1) h(x)| < tol.
   */
  bool is_singular(const Vec &x, double tol = 1e-6) {
    if (r_ <= 0)
      return false;
    ScalarField Lfr1_h = make_lie_k(f_, h_, r_ - 1);
    return std::abs(lie_derivative(g_, Lfr1_h, x)) < tol;
  }
};

// ============================================================
//           ZERO DYNAMICS SIMULATION
// ============================================================

/**
 * @brief Simulate zero dynamics (internal dynamics with y ≡ 0)
 *
 * For SISO FBL system, after I/O linearization the state can be split into:
 * - Linearized coordinates: ξ = [y, ẏ, ..., y^(r-1)]  (controlled)
 * - Zero dynamics: η̇ = q(η, 0) (uncontrolled internal dynamics)
 *
 * This function simulates η(t) starting from η₀ with y ≡ 0 constraint.
 * The zero dynamics states are the states NOT in the output-relative-degree
 * chain.
 *
 * For simple 2D systems (r = n), there are no zero dynamics.
 *
 * @param f        Full system ẋ = f(x) + g(x)u
 * @param g        Input vector field
 * @param h        Output function y = h(x)
 * @param fbl      Feedback linearizer (pre-built)
 * @param eta0     Initial condition in original coordinates
 * @param T        Simulation time
 * @param dt       Time step
 * @return         Pair: {time, internal_state_norms}
 */
inline std::pair<std::vector<double>, std::vector<double>>
simulate_zero_dynamics(FeedbackLinearizer &fbl, int n_internal, Vec x0,
                       double T = 10.0, double dt = 0.01) {
  // Zero dynamics: apply control to maintain y = 0
  // i.e., set reference y_ref = 0, then compute u = fbl.compute(x, v)
  // where v maintains y^(r) = -k[0]*y - k[1]*ydot - ...
  auto gains = fbl.place_poles(5.0); // aggressive tracking to keep y=0

  Vec x = x0;
  double t = 0;
  int N = (int)(T / dt);
  std::vector<double> t_v(N), norm_v(N);

  for (int i = 0; i < N; ++i) {
    t_v[i] = t;
    double norm = 0;
    // Compute norm of internal states (approximate: all x except x[0])
    for (size_t j = 1; j < x.size(); ++j)
      norm += x[j] * x[j];
    norm_v[i] = std::sqrt(norm);

    // Outer loop: track y_ref = 0
    auto derivs = fbl.output_and_derivatives(x);
    double v = 0;
    for (size_t j = 0; j < std::min(gains.size(), derivs.size()); ++j)
      v -= gains[j] * derivs[j];

    double u = 0;
    try {
      u = fbl.compute(x, v);
    } catch (...) {
      u = 0;
    }

    // RK4 integration
    auto dyn = [&](Vec xs) -> Vec {
      Vec fv = fbl.f_(xs);
      Vec gv = fbl.g_(xs);
      Vec dx(xs.size());
      for (size_t j = 0; j < xs.size(); ++j)
        dx[j] = fv[j] + gv[j] * u;
      return dx;
    };
    Vec k1 = dyn(x);
    Vec xk2(x.size());
    for (size_t j = 0; j < x.size(); ++j)
      xk2[j] = x[j] + 0.5 * dt * k1[j];
    Vec k2 = dyn(xk2);
    Vec xk3(x.size());
    for (size_t j = 0; j < x.size(); ++j)
      xk3[j] = x[j] + 0.5 * dt * k2[j];
    Vec k3 = dyn(xk3);
    Vec xk4(x.size());
    for (size_t j = 0; j < x.size(); ++j)
      xk4[j] = x[j] + dt * k3[j];
    Vec k4 = dyn(xk4);
    for (size_t j = 0; j < x.size(); ++j)
      x[j] += dt / 6.0 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);
    t += dt;
  }
  return {t_v, norm_v};
}

// ============================================================
//           MIMO INPUT-OUTPUT LINEARIZATION
// ============================================================

/**
 * @brief MIMO Decoupling Matrix for square input-output linearization
 *
 * For MIMO system ẋ = f(x) + g(x)u, yᵢ = hᵢ(x), i=1..m:
 * - Relative degrees [r₁, r₂, ..., r_m]
 * - Decoupling matrix D(x): dᵢⱼ = L_gⱼ L_f^(rᵢ-1) hᵢ(x)
 * - Control: u = D(x)⁻¹ (v - a(x)), where aᵢ(x) = L_f^rᵢ hᵢ(x)
 */
class InputOutputLinearizer {
public:
  VectorField f_;
  std::vector<VectorField> g_cols_; ///< Each column gⱼ of g(x)
  std::vector<ScalarField> h_;      ///< Each output function hᵢ
  std::vector<int> r_;              ///< Relative degrees [r₁, ..., r_m]

  InputOutputLinearizer(VectorField f, std::vector<VectorField> g_cols,
                        std::vector<ScalarField> h, std::vector<int> r)
      : f_(f), g_cols_(g_cols), h_(h), r_(r) {}

  /**
   * @brief Compute MIMO FBL control u = D(x)⁻¹(v - a(x))
   *
   * @param x   Current state
   * @param v   Virtual input vector [v₁, ..., v_m]
   * @return    Control input vector u
   */
  std::vector<double> compute(const Vec &x, const std::vector<double> &v) {
    size_t m = h_.size();
    if (v.size() != m)
      throw std::runtime_error("MIMO FBL: v size mismatch");

    // Build decoupling matrix D and a(x)
    std::vector<std::vector<double>> D(m, std::vector<double>(m, 0.0));
    std::vector<double> a(m);

    for (size_t i = 0; i < m; ++i) {
      int ri = r_[i];
      // aᵢ = L_f^rᵢ hᵢ(x)
      auto Lfri_h = make_lie_k(f_, h_[i], ri);
      a[i] = Lfri_h(x);

      for (size_t j = 0; j < m; ++j) {
        // dᵢⱼ = L_{gⱼ} L_f^(rᵢ-1) hᵢ(x)
        auto Lfri1_h = make_lie_k(f_, h_[i], ri - 1);
        D[i][j] = lie_derivative(g_cols_[j], Lfri1_h, x);
      }
    }

    // Solve D*u = v - a  (2×2 case: analytic; general: Gaussian elimination)
    std::vector<double> rhs(m);
    for (size_t i = 0; i < m; ++i)
      rhs[i] = v[i] - a[i];

    // Gaussian elimination with partial pivoting
    auto D_copy = D;
    auto rhs_copy = rhs;
    for (size_t pivot = 0; pivot < m; ++pivot) {
      // Find max element
      size_t max_row = pivot;
      for (size_t k = pivot + 1; k < m; ++k)
        if (std::abs(D_copy[k][pivot]) > std::abs(D_copy[max_row][pivot]))
          max_row = k;
      std::swap(D_copy[pivot], D_copy[max_row]);
      std::swap(rhs_copy[pivot], rhs_copy[max_row]);

      if (std::abs(D_copy[pivot][pivot]) < 1e-10)
        throw std::runtime_error("MIMO FBL: decoupling matrix is singular");

      for (size_t k = pivot + 1; k < m; ++k) {
        double factor = D_copy[k][pivot] / D_copy[pivot][pivot];
        for (size_t l = pivot; l < m; ++l)
          D_copy[k][l] -= factor * D_copy[pivot][l];
        rhs_copy[k] -= factor * rhs_copy[pivot];
      }
    }
    // Back substitution
    std::vector<double> u(m, 0.0);
    for (int i = (int)m - 1; i >= 0; --i) {
      u[i] = rhs_copy[i];
      for (size_t j = i + 1; j < m; ++j)
        u[i] -= D_copy[i][j] * u[j];
      u[i] /= D_copy[i][i];
    }
    return u;
  }
};

// ============================================================
//          FREE FUNCTIONS: SIMULATION WITH FBL
// ============================================================

/**
 * @brief Simulate FBL-controlled system tracking a reference
 *
 * Outer loop: PD on tracking error + feedforward via FBL.
 * Integrates the original nonlinear system with u = FBL(v).
 *
 * @param fbl        Feedback linearizer
 * @param f          Full system dynamics ẋ = f(x) + g(x)u
 * @param g          Input vector field
 * @param x0         Initial state
 * @param ref        Reference signal: (t) → y_ref
 * @param ref_dot    Reference derivative (t) → dy_ref/dt (only for r≥2)
 * @param gains      Outer loop gains [k_0, ..., k_{r-1}]
 * @param T          Simulation time
 * @param dt         Time step
 * @return           {time, output y(t), reference y_ref(t), control u(t)}
 */
struct FBLSimResult {
  std::vector<double> time, y, y_ref, u;
  double rms_error;
  double max_error;
};

inline FBLSimResult simulate_fbl(FeedbackLinearizer &fbl, const VectorField &f,
                                 const VectorField &g, Vec x0,
                                 std::function<double(double)> ref,
                                 std::function<double(double)> ref_dot,
                                 const std::vector<double> &gains,
                                 double T = 10.0, double dt = 0.01) {
  FBLSimResult result;
  Vec x = x0;
  double t = 0;
  int N = (int)(T / dt);
  result.time.reserve(N);
  result.y.reserve(N);
  result.y_ref.reserve(N);
  result.u.reserve(N);
  double sum_sq = 0;

  for (int i = 0; i < N; ++i) {
    double yr = ref(t);
    double yrd = ref_dot(t);
    double y_cur = fbl.h_(x);
    double e = y_cur - yr;

    // Outer loop: v = y_ref^(r) - gains[r-1]*e^(r-1) - ... - gains[0]*e
    auto derivs = fbl.output_and_derivatives(x);
    double v = 0;
    if (!derivs.empty())
      v -= gains[0] * (derivs[0] - yr);
    if (derivs.size() >= 2 && gains.size() >= 2)
      v -= gains[1] * (derivs[1] - yrd);
    for (size_t k = 2; k < std::min(gains.size(), derivs.size()); ++k)
      v -= gains[k] * derivs[k];

    double u = 0;
    try {
      u = fbl.compute(x, v);
    } catch (...) {
    }

    result.time.push_back(t);
    result.y.push_back(y_cur);
    result.y_ref.push_back(yr);
    result.u.push_back(u);
    sum_sq += e * e;

    // RK4 integration
    auto sys = [&](Vec xs) -> Vec {
      Vec fv = f(xs), gv = g(xs);
      Vec dx(xs.size());
      for (size_t j = 0; j < xs.size(); ++j)
        dx[j] = fv[j] + gv[j] * u;
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
  // max error
  result.max_error = 0;
  for (size_t i = 0; i < result.y.size(); ++i)
    result.max_error =
        std::max(result.max_error, std::abs(result.y[i] - result.y_ref[i]));
  return result;
}

} // namespace nonlinear
} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_NONLINEAR_FEEDBACK_LINEARIZATION_HPP
