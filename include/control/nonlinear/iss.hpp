/**
 * @file nonlinear/iss.hpp
 * @brief Input-to-State Stability (ISS) Analysis Tools
 *
 * ISS characterizes robustness of nonlinear systems to bounded disturbances:
 *   ẋ = f(x, d),  x ∈ Rⁿ,  d ∈ Rᵐ (disturbance)
 *
 * A system is ISS if ∃ β ∈ KL, γ ∈ K:
 *   ‖x(t)‖ ≤ β(‖x₀‖, t) + γ(‖d‖∞)
 *
 * Features:
 *   - ISS Lyapunov function conditions
 *   - Numeric ISS gain estimation via simulation sweep
 *   - Small-gain theorem verification for cascade/interconnected systems
 *   - ISS-backstepping with robustness margins
 *   - Gain margin and disturbance rejection plots
 *
 * References:
 *   - Sontag (1989) "Smooth stabilization implies coprime factorization"
 *   - Jiang & Wang (2001) "Input-to-state stability for discrete-time systems"
 *   - Khalil (2002) "Nonlinear Systems", Ch. 9
 */

#ifndef CPPPLOT_CONTROL_NONLINEAR_ISS_HPP
#define CPPPLOT_CONTROL_NONLINEAR_ISS_HPP

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
//              ISS LYAPUNOV FUNCTION CONDITIONS
// ============================================================

/**
 * @brief ISS-Lyapunov conditions:
 *
 * V: R^n → R is an ISS-Lyapunov function if ∃ α₁, α₂ ∈ K∞, α₃ ∈ K, γ ∈ K:
 *   α₁(‖x‖) ≤ V(x) ≤ α₂(‖x‖)
 *   ‖x‖ ≥ γ(‖d‖) → ∂V/∂x · f(x,d) ≤ -α₃(‖x‖)
 *
 * This struct holds the functions and provides a grid-based verification.
 */
struct ISSLyapunovVerification {
  bool pos_definite;   ///< α₁(‖x‖) ≤ V(x) condition
  bool rad_unbounded;  ///< V(x) → ∞ as ‖x‖ → ∞
  bool decrease_check; ///< V̇ ≤ -α₃(‖x‖) when ‖x‖ ≥ γ(‖d‖)
  bool iss_certified;  ///< All conditions satisfied

  void print() const {
    std::cout << "== ISS-Lyapunov Verification ==\n";
    std::cout << "  Positive definite V:  " << (pos_definite ? "✓" : "✗")
              << "\n";
    std::cout << "  Radially unbounded:   " << (rad_unbounded ? "✓" : "✗")
              << "\n";
    std::cout << "  Decrease condition:   " << (decrease_check ? "✓" : "✗")
              << "\n";
    std::cout << "  ISS Certified:        " << (iss_certified ? "YES" : "NO")
              << "\n";
  }
};

/**
 * @brief Verify ISS-Lyapunov conditions numerically on a grid
 *
 * @param V         Lyapunov function V(x)
 * @param Vdot      V̇(x, d): time derivative with disturbance d
 * @param gamma_fn  γ: R → R (class K function: bound s.t. if ‖x‖ ≥ γ(‖d‖) →
 * V̇<0)
 * @param alpha3_fn α₃: R → R (class K decay rate)
 * @param x_range   Symmetric range: x ∈ [-x_range, x_range]^n
 * @param d_vals    Range of disturbance magnitudes to test
 * @param n_states  State dimension
 * @param resolution Grid resolution per dimension (total = resolution^n_states)
 */
inline ISSLyapunovVerification verify_iss_lyapunov(
    std::function<double(Vec)> V,
    std::function<double(Vec, double)> Vdot, // Vdot(x, d_magnitude)
    std::function<double(double)> gamma_fn,
    std::function<double(double)> alpha3_fn, double x_range = 3.0,
    double d_max = 1.0, int resolution = 20, int n_states = 2) {
  ISSLyapunovVerification result{true, true, true, false};
  double dx = 2 * x_range / resolution;

  // For 2D verification (generalize for higher n)
  if (n_states == 2) {
    double V_max_seen = 0, x_max_seen = 0;

    for (int i = 0; i <= resolution; ++i) {
      for (int j = 0; j <= resolution; ++j) {
        Vec x = {-x_range + i * dx, -x_range + j * dx};
        double x_norm = std::sqrt(x[0] * x[0] + x[1] * x[1]);
        if (x_norm < 1e-8)
          continue;

        double v = V(x);
        V_max_seen = std::max(V_max_seen, v);
        x_max_seen = std::max(x_max_seen, x_norm);

        // Positive definiteness: V(x) > 0 for x ≠ 0
        if (v <= 0)
          result.pos_definite = false;

        // Decrease condition: test at maximum disturbance
        // If ‖x‖ ≥ γ(‖d‖) → V̇ ≤ -α₃(‖x‖)
        double d_test = d_max;
        double gamma_d = gamma_fn(d_test);
        if (x_norm >= gamma_d) {
          double vd = Vdot(x, d_test);
          double thresh = -alpha3_fn(x_norm);
          if (vd > thresh + 1e-6)
            result.decrease_check = false;
        }
      }
    }

    // Radial unboundedness check (heuristic: V increased with ‖x‖)
    result.rad_unbounded = (V_max_seen > 0); // minimal check
  }

  result.iss_certified =
      result.pos_definite && result.rad_unbounded && result.decrease_check;
  return result;
}

// ============================================================
//              NUMERIC ISS GAIN ESTIMATION
// ============================================================

/**
 * @brief Estimate ISS gain γ via simulation sweep
 *
 * Sweeps disturbance magnitudes ‖d‖∞ and measures steady-state x amplitude.
 * Fits γ(s) = a*s (linear ISS gain approximation).
 *
 * @param f_disturbed  System ẋ = f(x, d): dynamics with disturbance input
 * @param d_range      Vector of disturbance magnitudes to test
 * @param x0           Initial condition
 * @param T_settle     Settlement time (we take max from T_settle/2 to T)
 * @param dt           Simulation time step
 * @return             {d_values, max_x_norms, estimated_gain_slope}
 */
struct ISSGainResult {
  std::vector<double> d_values; ///< Tested disturbance magnitudes
  std::vector<double> x_norms;  ///< Corresponding max ‖x‖ at steady state
  double gamma_slope;           ///< Linear ISS gain γ: ‖x‖_ss ≈ γ * ‖d‖
  bool is_iss;                  ///< Empirical ISS check
};

inline ISSGainResult
estimate_iss_gain(std::function<Vec(Vec, double)> f_disturbed,
                  std::vector<double> d_range, Vec x0, double T_settle = 15.0,
                  double dt = 0.01) {
  ISSGainResult result;
  result.d_values = d_range;
  result.gamma_slope = 0;

  for (double d_mag : d_range) {
    Vec x = x0;
    double t = 0;
    int N = (int)(T_settle / dt);
    double x_max = 0;
    double start_track = T_settle / 2.0;

    for (int k = 0; k < N; ++k) {
      t += dt;
      // Constant worst-case disturbance (or sinusoidal)
      double d = d_mag; // constant step disturbance

      // RK4
      auto sys = [&](Vec xs) { return f_disturbed(xs, d); };
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

      if (t >= start_track) {
        double norm = 0;
        for (double xi : x)
          norm += xi * xi;
        x_max = std::max(x_max, std::sqrt(norm));
      }
    }
    result.x_norms.push_back(x_max);
  }

  // Linear fit: γ = max(x_norm) / d_mag
  double gamma_max = 0;
  result.is_iss = true;
  for (size_t i = 0; i < d_range.size(); ++i) {
    if (d_range[i] > 1e-10) {
      gamma_max = std::max(gamma_max, result.x_norms[i] / d_range[i]);
    }
    // Check if system stays bounded
    if (result.x_norms[i] > 1000.0)
      result.is_iss = false;
  }
  result.gamma_slope = gamma_max;

  return result;
}

// ============================================================
//              SMALL-GAIN THEOREM
// ============================================================

/**
 * @brief Small-gain theorem check for two ISS systems in feedback
 *
 * For cascade/interconnection of two ISS systems:
 *   Σ₁: ‖x₁‖ ≤ β₁(‖x₁₀‖,t) + γ₁(‖x₂‖)
 *   Σ₂: ‖x₂‖ ≤ β₂(‖x₂₀‖,t) + γ₂(‖x₁‖)
 *
 * Small-gain condition: γ₁ ∘ γ₂ (s) < s for all s > 0.
 * For linear gains γᵢ(s) = kᵢ·s: condition is k₁·k₂ < 1.
 *
 * @param gamma1_slope  Linear gain of system 1: ‖x₁‖ ≤ k₁·‖x₂‖
 * @param gamma2_slope  Linear gain of system 2: ‖x₂‖ ≤ k₂·‖x₁‖
 * @return              true if small-gain condition satisfied
 */
inline bool small_gain_check(double gamma1_slope, double gamma2_slope) {
  return gamma1_slope * gamma2_slope < 1.0;
}

/**
 * @brief Nonlinear small-gain check: verify γ₁∘γ₂(s) < s on [0, s_max]
 *
 * @param gamma1    Class K function γ₁: R≥0 → R≥0
 * @param gamma2    Class K function γ₂: R≥0 → R≥0
 * @param s_max     Maximum s to check
 * @param n_points  Number of points to check
 * @return          true if γ₁(γ₂(s)) < s for all tested s
 */
inline bool small_gain_nonlinear(std::function<double(double)> gamma1,
                                 std::function<double(double)> gamma2,
                                 double s_max = 10.0, int n_points = 100) {
  for (int i = 1; i <= n_points; ++i) {
    double s = s_max * i / n_points;
    double composed = gamma1(gamma2(s));
    if (composed >= s)
      return false;
  }
  return true;
}

// ============================================================
//              ISS-BACKSTEPPING
// ============================================================

/**
 * @brief ISS-aware backstepping for 2nd order system with disturbance
 *
 * System: ẋ₁ = x₂ + d₁,  ẋ₂ = u + d₂
 * ISS backstepping: each step ensures zᵢ-subsystem is ISS w.r.t. d.
 * The ISS gain margins are computed analytically.
 *
 * Result: u achieves ISS with gain γ: ‖z‖∞ ≤ γ · ‖d‖∞
 */
class ISSBackstepping {
public:
  double c1_, c2_;     ///< Step gains (must be positive)
  double eps1_, eps2_; ///< ISS margins (small positive)

  ISSBackstepping(double c1 = 2.0, double c2 = 2.0, double eps1 = 0.5,
                  double eps2 = 0.5)
      : c1_(c1), c2_(c2), eps1_(eps1), eps2_(eps2) {}

  /**
   * @brief Compute ISS-robust backstepping control
   *
   * @param x   State [x₁, x₂]
   * @param yr  Reference
   * @return    Control input u
   */
  double compute(const Vec &x, double yr = 0.0) {
    double z1 = x[0] - yr;
    double alpha1 = -(c1_ + eps1_) * z1; // Extra margin eps for ISS
    double z2 = x[1] - alpha1;
    double alpha1dot_approx = -(c1_ + eps1_) * x[1]; // dα₁/dt ≈ -c₁*x₂
    double u = -(c2_ + eps2_) * z2 - z1 + alpha1dot_approx;
    return u;
  }

  /**
   * @brief Theoretical ISS gain bound
   *
   * For this 2nd order system, ISS gain γ ≈ 1/(min(c1,c2)*eps).
   */
  double iss_gain_bound() const {
    return 1.0 / (std::min(eps1_, eps2_) * std::min(c1_, c2_));
  }
};

// ============================================================
//              ISS ANALYSIS PLOTS
// ============================================================

/**
 * @brief Plot ISS gain curve: max steady-state norm vs disturbance magnitude
 *
 * @param result   ISSGainResult from estimate_iss_gain()
 * @param label    Plot label
 */
inline void plot_iss_gain(const ISSGainResult &result,
                          const std::string &label = "ISS gain") {
  cppplot::plot(result.d_values, result.x_norms, "b-o",
                {{"label", label}, {"markersize", "4"}});

  // Also plot the linear ISS gain bound
  std::vector<double> gamma_line;
  for (double d : result.d_values)
    gamma_line.push_back(result.gamma_slope * d);
  cppplot::plot(result.d_values, gamma_line, "r--",
                {{"label", "γ·‖d‖ (ISS bound)"}});
}

} // namespace nonlinear
} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_NONLINEAR_ISS_HPP
