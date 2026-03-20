/**
 * @file nonlinear/lyapunov.hpp
 * @brief Lyapunov Stability Analysis Tools
 *
 * General-purpose Lyapunov-based analysis for nonlinear systems:
 *   ẋ = f(x, u)
 *
 * Features:
 *   - Control Lyapunov Function (CLF) conditions
 *   - Region of Attraction (ROA) estimation via sublevel sets
 *   - Lyapunov equation solver for linear systems (AᵀP + PA = -Q)
 *   - Barrier certificate verification
 *   - Level set visualization
 *   - LaSalle invariance principle applications
 *
 * References:
 *   - Khalil (2002) "Nonlinear Systems", Ch. 4, 5
 *   - Sontag (1989) "A 'universal' construction of Artstein's theorem"
 *   - Prajna et al. (2007) "Framework for worst-case and stochastic safety"
 *
 * Usage:
 * @code
 * #include <cppplot/control/nonlinear/lyapunov.hpp>
 * using namespace cppplot::control::nonlinear;
 *
 * // Quadratic CLF: V(x) = x'Px for inverted pendulum
 * auto V    = [](Vec x) { return x[0]*x[0] + x[1]*x[1]; };
 * auto Vdot = [](Vec x, double u) {
 *     return 2*x[0]*x[1] + 2*x[1]*(-std::sin(x[0]) + u);
 * };
 *
 * LyapunovAnalysis la(V, Vdot);
 * double roa_level = la.estimate_roa_level({-3,3}, {-3,3});
 * la.plot_level_sets({-3,3}, {-3,3});
 * @endcode
 */

#ifndef CPPPLOT_CONTROL_NONLINEAR_LYAPUNOV_HPP
#define CPPPLOT_CONTROL_NONLINEAR_LYAPUNOV_HPP

#include "../../pyplot.hpp"
#include "../state_space.hpp"
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
using VectorField2D = std::function<Vec(Vec)>;

/// Scalar Lyapunov function V: R^n → R
using LyapunovFunc = std::function<double(Vec)>;
/// V̇ = ∂V/∂x · f(x,u): depends on state and control input
using LyapunovDot = std::function<double(Vec, double)>;
/// Autonomous V̇ = ∂V/∂x · f(x): depends only on state (no-input version)
using LyapunovDotAuto = std::function<double(Vec)>;

// ============================================================
//               NUMERIC GRADIENT UTILITY
// ============================================================

/**
 * @brief Numeric gradient of scalar function V(x) at point x
 * Uses central finite differences with step h.
 */
inline Vec gradient(const LyapunovFunc &V, const Vec &x, double h = 1e-5) {
  Vec grad(x.size());
  for (size_t i = 0; i < x.size(); ++i) {
    Vec xp = x, xm = x;
    xp[i] += h;
    xm[i] -= h;
    grad[i] = (V(xp) - V(xm)) / (2 * h);
  }
  return grad;
}

/**
 * @brief Lie derivative: L_f V(x) = ∇V · f(x)
 */
inline double lie_derivative_V(const LyapunovFunc &V,
                               const std::function<Vec(Vec)> &f, const Vec &x,
                               double h = 1e-5) {
  Vec grad = gradient(V, x, h);
  Vec fv = f(x);
  double result = 0.0;
  for (size_t i = 0; i < std::min(grad.size(), fv.size()); ++i)
    result += grad[i] * fv[i];
  return result;
}

// ============================================================
//               LYAPUNOV EQUATION SOLVER
// ============================================================

/**
 * @brief Solve continuous Lyapunov equation: AᵀP + PA = -Q
 *
 * Uses the Bartels-Stewart method (iterative) for small matrices,
 * or closed-form by vectorization: (A⊗I + I⊗A)vec(P) = -vec(Q)
 *
 * @param A  n×n stable system matrix (eigenvalues must have Re < 0)
 * @param Q  n×n positive definite matrix (typically identity)
 * @return   P  n×n positive definite solution
 * @throws   std::runtime_error if A is not stable
 */
inline Matrix lyapunov(const Matrix &A, const Matrix &Q) {
  size_t n = A.rows;
  if (n != A.cols || n != Q.rows || n != Q.cols)
    throw std::runtime_error("lyapunov_equation: dimension mismatch");

  // Check stability
  auto eigs = A.eigenvalues();
  for (const auto &ev : eigs) {
    if (ev.real() >= 0)
      throw std::runtime_error(
          "lyapunov_equation: A must be stable (all Re(λ) < 0)");
  }

  // Vectorization approach: solve (A^T ⊗ I + I ⊗ A) p = -q
  // For n×n, this gives n² equations. We solve via Kronecker product.
  // Build n²×n² system matrix: M = A^T ⊗ I_n + I_n ⊗ A
  size_t n2 = n * n;
  Matrix M(n2, n2, 0.0);
  Matrix I = Matrix::eye(n);
  Matrix AT = A.T();

  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < n; ++j) {
      // I ⊗ A block: block (i,j) = A * delta(i,j)
      for (size_t k = 0; k < n; ++k)
        for (size_t l = 0; l < n; ++l)
          M(i * n + k, j * n + l) += (i == j ? 1.0 : 0.0) * A(k, l);

      // AT ⊗ I block: M(i*n+k, j*n+k) += AT(i,j)
      for (size_t k = 0; k < n; ++k)
        M(i * n + k, j * n + k) += AT(i, j);
    }
  }

  // RHS: -vec(Q)
  Matrix rhs(n2, 1, 0.0);
  for (size_t i = 0; i < n; ++i)
    for (size_t j = 0; j < n; ++j)
      rhs(i * n + j, 0) = -Q(i, j);

  // Solve: M * vec(P) = rhs
  Matrix p_vec = M.solve(rhs);

  // Reshape to n×n
  Matrix P(n, n, 0.0);
  for (size_t i = 0; i < n; ++i)
    for (size_t j = 0; j < n; ++j)
      P(i, j) = p_vec(i * n + j, 0);

  return P;
}

// ============================================================
//               CLF CONDITION CHECK
// ============================================================

/**
 * @brief Check if CLF condition is satisfiable at state x
 *
 * For CLF V with ∂V/∂x · g(x) ≠ 0, there exists a control u such that
 * V̇ < 0. Uses Sontag's condition for affine systems: ẋ = f(x) + g(x)u.
 *
 * @param Lf_V    L_f V(x) = ∇V · f(x)
 * @param Lg_V    L_g V(x) = ∇V · g(x)
 * @param V_x     V(x) current value
 * @param alpha   Class K function: need V̇ ≤ -α(V)
 * @return        true if CLF condition can be satisfied
 */
inline bool clf_condition(double Lf_V, double Lg_V, double V_x,
                          double alpha = 1.0) {
  // If L_g V = 0: need L_f V < 0 already
  if (std::abs(Lg_V) < 1e-10)
    return Lf_V < -alpha * V_x;
  // Otherwise: u = -Lf_V/Lg_V -alpha*V_x/Lg_V achieves V̇ ≤ -alpha*V
  return true; // always satisfiable when Lg_V ≠ 0
}

/**
 * @brief Sontag's universal CLF control law
 *
 * u = -(Lf_V + sqrt((Lf_V)^2 + (Lg_V)^4)) / Lg_V  when Lg_V ≠ 0
 *
 * Guarantees V̇ ≤ -‖Lg_V‖² (strictly decreasing).
 */
inline double sontag_clf_control(double Lf_V, double Lg_V,
                                 double epsilon = 0.0) {
  if (std::abs(Lg_V) < 1e-9)
    return 0.0;
  double num = Lf_V + std::sqrt(Lf_V * Lf_V + std::pow(Lg_V, 4));
  return -num / Lg_V;
}

// ============================================================
//               REGION OF ATTRACTION ESTIMATION
// ============================================================

/**
 * @brief ROA estimation result
 */
struct ROAResult {
  double level;           ///< Estimated sublevel set {V(x) ≤ level}
  double volume_estimate; ///< 2D: area of estimated ROA
  bool verified;          ///< Was V̇ < 0 verified in entire sublevel set?
  std::string method;
};

/**
 * @brief Estimate ROA by finding largest sublevel set {V ≤ c} where V̇ < 0
 *
 * Binary search over level c. Checks a fine grid within the level set.
 *
 * @param V          Lyapunov function V(x)
 * @param Vdot       Autonomous V̇(x) = ∇V·f(x)
 * @param x1_range   Search range for x₁
 * @param x2_range   Search range for x₂
 * @param resolution Grid resolution for verification
 * @param max_level  Maximum level set value to search
 */
inline ROAResult estimate_roa(const LyapunovFunc &V,
                              const LyapunovDotAuto &Vdot,
                              std::pair<double, double> x1_range,
                              std::pair<double, double> x2_range,
                              int resolution = 50, double max_level = 10.0) {
  ROAResult result;
  result.method = "sublevel_set_binary_search";

  // Check if V is actually positive definite at origin
  // and V̇ is negative definite in a neighborhood
  auto check_level = [&](double c) -> bool {
    // Check all grid points with V(x) ≤ c: must have Vdot(x) < 0
    double dx = (x1_range.second - x1_range.first) / resolution;
    double dy = (x2_range.second - x2_range.first) / resolution;
    for (int i = 0; i <= resolution; ++i) {
      for (int j = 0; j <= resolution; ++j) {
        Vec x = {x1_range.first + i * dx, x2_range.first + j * dy};
        double v = V(x);
        if (v < 1e-10 || v > c)
          continue; // skip origin & outside set
        double vd = Vdot(x);
        if (vd >= 0)
          return false; // V̇ ≥ 0: not inside ROA
      }
    }
    return true;
  };

  // Binary search on level c
  double lo = 0.0, hi = max_level;
  for (int iter = 0; iter < 30; ++iter) {
    double mid = 0.5 * (lo + hi);
    if (check_level(mid))
      lo = mid;
    else
      hi = mid;
  }

  result.level = lo;
  result.verified = check_level(lo);

  // Estimate ROA area (2D: count grid points inside {V ≤ c})
  if (x1_range.first < x1_range.second) {
    double dx = (x1_range.second - x1_range.first) / resolution;
    double dy = (x2_range.second - x2_range.first) / resolution;
    int count = 0;
    for (int i = 0; i <= resolution; ++i) {
      for (int j = 0; j <= resolution; ++j) {
        Vec x = {x1_range.first + i * dx, x2_range.first + j * dy};
        if (V(x) <= lo)
          count++;
      }
    }
    result.volume_estimate = count * dx * dy;
  }

  return result;
}

// ============================================================
//               BARRIER CERTIFICATE
// ============================================================

/**
 * @brief Barrier certificate conditions for safety verification
 *
 * Given a barrier function B(x) ≥ 0 defining safe set S = {x: B(x) ≥ 0}:
 *   1. B(x₀) ≥ 0 for all x₀ in initial set X₀
 *   2. B(x) < 0 for all x in unsafe set Xu
 *   3. Ḃ(x) ≥ 0 (or ≥ -λB) for all x on boundary {B=0}
 *
 * If all conditions hold → safe set is forward invariant.
 */
struct BarrierCertificateResult {
  bool condition1_ok; ///< All initial states safe
  bool condition2_ok; ///< All unsafe states correctly labeled
  bool condition3_ok; ///< Ḃ ≥ -λB on boundary
  bool certified;     ///< All three conditions satisfied

  void print() const {
    std::cout << "== Barrier Certificate Verification ==\n";
    std::cout << "  Cond 1 (X₀ ⊆ safe):     " << (condition1_ok ? "✓" : "✗")
              << "\n";
    std::cout << "  Cond 2 (Xu ⊆ unsafe):   " << (condition2_ok ? "✓" : "✗")
              << "\n";
    std::cout << "  Cond 3 (Ḃ ≥ -λB on ∂S): " << (condition3_ok ? "✓" : "✗")
              << "\n";
    std::cout << "  Safety Certified:        " << (certified ? "YES" : "NO")
              << "\n";
  }
};

/**
 * @brief Verify barrier certificate numerically on a grid
 *
 * @param B           Barrier function B(x) ≥ 0 defines safe set
 * @param Bdot        Ḃ(x) = ∇B · f(x)
 * @param initial_set Predicate: x ∈ X₀?
 * @param unsafe_set  Predicate: x ∈ Xu?
 * @param x1_range    Grid range x₁
 * @param x2_range    Grid range x₂
 * @param lambda      Relaxed condition constant (Ḃ ≥ -λB)
 * @param resolution  Grid resolution
 */
inline BarrierCertificateResult
verify_barrier_certificate(const LyapunovFunc &B, const LyapunovDotAuto &Bdot,
                           const std::function<bool(Vec)> &initial_set,
                           const std::function<bool(Vec)> &unsafe_set,
                           std::pair<double, double> x1_range,
                           std::pair<double, double> x2_range,
                           double lambda = 0.0, int resolution = 50) {
  BarrierCertificateResult result{true, true, true, false};

  double dx = (x1_range.second - x1_range.first) / resolution;
  double dy = (x2_range.second - x2_range.first) / resolution;
  // Note: y2 → x2 fix
  dy = (x2_range.second - x2_range.first) / resolution;

  for (int i = 0; i <= resolution; ++i) {
    for (int j = 0; j <= resolution; ++j) {
      Vec x = {x1_range.first + i * dx, x2_range.first + j * dy};
      double bv = B(x);
      double bdv = Bdot(x);

      // Condition 1: initial set ⊆ {B ≥ 0}
      if (initial_set(x) && bv < 0)
        result.condition1_ok = false;

      // Condition 2: unsafe set ⊆ {B < 0}
      if (unsafe_set(x) && bv >= 0)
        result.condition2_ok = false;

      // Condition 3: on boundary |B| small → Ḃ ≥ -λB
      if (std::abs(bv) < 0.1 * std::max(dx, dy)) {
        if (bdv < -lambda * bv - 1e-6)
          result.condition3_ok = false;
      }
    }
  }

  result.certified =
      result.condition1_ok && result.condition2_ok && result.condition3_ok;
  return result;
}

// ============================================================
//             LYAPUNOV ANALYSIS CLASS
// ============================================================

/**
 * @brief Container for Lyapunov function analysis (stateful)
 *
 * Holds V and V̇ and provides analysis + visualization methods.
 * Works for both autonomous (V̇ = f(x)) and controlled (V̇ = f(x,u)) systems.
 */
class LyapunovAnalysis {
public:
  LyapunovFunc V_;   ///< Lyapunov function
  LyapunovDot Vdot_; ///< V̇ with control input

  /**
   * @param V     Lyapunov function: V(x) ≥ 0, V(0)=0
   * @param Vdot  Time derivative: Vdot(x, u) = ∇V · f(x,u)
   */
  LyapunovAnalysis(LyapunovFunc V, LyapunovDot Vdot) : V_(V), Vdot_(Vdot) {}

  /// Autonomous version: Vdot(x) with u=0 internally
  LyapunovAnalysis(LyapunovFunc V, LyapunovDotAuto Vdot_auto) : V_(V) {
    Vdot_ = [Vdot_auto](Vec x, double) { return Vdot_auto(x); };
  }

  /// Evaluate V at a state
  double V(const Vec &x) const { return V_(x); }

  /// Evaluate V̇ at state x with control u
  double Vdot(const Vec &x, double u = 0.0) const { return Vdot_(x, u); }

  /**
   * @brief Estimate Region of Attraction (2D only)
   *
   * Finds largest sublevel set {V(x) ≤ c} where V̇(x, 0) < 0.
   */
  ROAResult estimate_roa_level(std::pair<double, double> x1_range,
                               std::pair<double, double> x2_range,
                               int resolution = 60) const {
    LyapunovDotAuto Vdot_auto = [this](Vec x) { return Vdot_(x, 0.0); };
    return estimate_roa(V_, Vdot_auto, x1_range, x2_range, resolution);
  }

  /**
   * @brief Plot level sets of V on a 2D grid
   *
   * @param x1_range     Plot range for x₁
   * @param x2_range     Plot range for x₂
   * @param levels       Specific level values to draw (empty → auto-choose)
   * @param resolution   Grid resolution
   */
  void plot_level_sets(std::pair<double, double> x1_range,
                       std::pair<double, double> x2_range,
                       std::vector<double> levels = {},
                       int resolution = 100) const {
    double dx = (x1_range.second - x1_range.first) / resolution;
    double dy = (x2_range.second - x2_range.first) / resolution;

    // Evaluate V on grid
    std::vector<double> V_vals;
    double V_max = 0;
    for (int i = 0; i <= resolution; ++i) {
      for (int j = 0; j <= resolution; ++j) {
        Vec x = {x1_range.first + i * dx, x2_range.first + j * dy};
        double v = V_(x);
        V_vals.push_back(v);
        V_max = std::max(V_max, v);
      }
    }

    if (levels.empty()) {
      // Auto-choose 5 logarithmically spaced levels
      for (int k = 1; k <= 5; ++k)
        levels.push_back(V_max * k / 6.0);
    }

    // Draw each level set by scanning for sign changes
    std::vector<std::string> colors = {"#1f77b4", "#ff7f0e", "#2ca02c",
                                       "#d62728", "#9467bd"};
    for (size_t li = 0; li < levels.size(); ++li) {
      double c = levels[li];
      std::vector<double> lx1, lx2;
      for (int i = 0; i <= resolution; ++i) {
        for (int j = 0; j < resolution; ++j) {
          Vec xa = {x1_range.first + i * dx, x2_range.first + j * dy};
          Vec xb = {x1_range.first + i * dx, x2_range.first + (j + 1) * dy};
          double va = V_(xa) - c, vb = V_(xb) - c;
          if (va * vb <= 0) {
            lx1.push_back(xa[0]);
            lx2.push_back(0.5 * (xa[1] + xb[1]));
          }
        }
      }
      std::string col = colors[li % colors.size()];
      if (!lx1.empty())
        cppplot::plot(
            lx1, lx2, col + ".",
            {{"markersize", "1"}, {"label", "V=" + std::to_string((int)c)}});
    }
  }

  /**
   * @brief Check if V̇ < 0 throughout a region for u=0
   *
   * @return true if V̇ < 0 everywhere in the grid
   */
  bool verify_negative_definite(std::pair<double, double> x1_range,
                                std::pair<double, double> x2_range,
                                int resolution = 40) const {
    double dx = (x1_range.second - x1_range.first) / resolution;
    double dy = (x2_range.second - x2_range.first) / resolution;
    for (int i = 0; i <= resolution; ++i) {
      for (int j = 0; j <= resolution; ++j) {
        Vec x = {x1_range.first + i * dx, x2_range.first + j * dy};
        // Skip origin
        if (std::abs(x[0]) < 1e-8 && std::abs(x[1]) < 1e-8)
          continue;
        if (Vdot_(x, 0.0) >= 0)
          return false;
      }
    }
    return true;
  }
};

// ============================================================
//          FREE FUNCTIONS: COMMON LYAPUNOV FORMS
// ============================================================

/**
 * @brief Quadratic CLF: V(x) = xᵀPx
 *
 * @param P  Positive definite matrix (comes from lyapunov_equation)
 * @return   (V, ∇V) pair: Lyapunov function and its gradient
 */
inline std::pair<LyapunovFunc, std::function<Vec(Vec)>>
quadratic_clf(const Matrix &P) {
  auto V = [P](Vec x) -> double {
    double val = 0;
    for (size_t i = 0; i < x.size(); ++i)
      for (size_t j = 0; j < x.size(); ++j)
        val += x[i] * P(i, j) * x[j];
    return val;
  };
  auto gradV = [P](Vec x) -> Vec {
    size_t n = x.size();
    Vec g(n, 0);
    for (size_t i = 0; i < n; ++i)
      for (size_t j = 0; j < n; ++j)
        g[i] += 2 * P(i, j) * x[j];
    return g;
  };
  return {V, gradV};
}

/**
 * @brief Check LaSalle invariance conditions
 *
 * V̇ ≤ 0 (not strictly), but the only trajectory for which V̇ = 0 is x = 0.
 * Verifies this numerically on a grid.
 *
 * @param f      Autonomous system ẋ = f(x)
 * @param V      Lyapunov function
 * @param Vdot   V̇(x) = ∇V · f(x)
 * @param x1_range, x2_range  Domain
 * @param tol    Tolerance for V̇ ≈ 0 detection
 * @return  Set of points where V̇ = 0 (should only be origin)
 */
inline std::vector<Vec>
lasalle_invariant_set(const LyapunovFunc &V, const LyapunovDotAuto &Vdot,
                      std::pair<double, double> x1_range,
                      std::pair<double, double> x2_range, double tol = 0.01,
                      int resolution = 40) {
  std::vector<Vec> set_E;
  double dx = (x1_range.second - x1_range.first) / resolution;
  double dy = (x2_range.second - x2_range.first) / resolution;
  for (int i = 0; i <= resolution; ++i) {
    for (int j = 0; j <= resolution; ++j) {
      Vec x = {x1_range.first + i * dx, x2_range.first + j * dy};
      if (std::abs(x[0]) < 1e-6 && std::abs(x[1]) < 1e-6)
        continue;
      if (std::abs(Vdot(x)) < tol)
        set_E.push_back(x);
    }
  }
  return set_E;
}

} // namespace nonlinear
} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_NONLINEAR_LYAPUNOV_HPP
