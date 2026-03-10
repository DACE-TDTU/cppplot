/**
 * @file adaptive/rls.hpp
 * @brief Recursive Least Squares (RLS) Parameter Estimation
 *
 * Online parameter estimator for the linear regression model:
 *   y = φᵀ(t) · θ + ε
 *
 * where y is the measured output, φ is the regressor vector, θ is the
 * unknown parameter vector, and ε is bounded noise.
 *
 * Variants:
 *   - RLS:           Standard with forgetting factor λ ∈ (0, 1]
 *   - RLS_Projection: RLS with parameter projection onto convex set Ω
 *   - RLS_CV:        Variable-format RLS with covariance resetting
 *
 * Applications:
 *   - System identification
 *   - Adaptive control (certainty equivalence)
 *   - Online model learning
 *
 * References:
 *   - Ioannou & Sun (1996) "Robust Adaptive Control", Ch. 3
 *   - Krstic et al. (1995) "Nonlinear and Adaptive Control Design", Ch. 1
 *   - Ljung (1987) "System Identification: Theory for the User"
 */

#ifndef CPPPLOT_CONTROL_ADAPTIVE_RLS_HPP
#define CPPPLOT_CONTROL_ADAPTIVE_RLS_HPP

#include "cppplot/core/matrix.hpp"
#include "cppplot/pyplot.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace cppplot {
namespace control {
namespace adaptive {

using Vec = std::vector<double>;

// ============================================================
//                 STANDARD RLS ESTIMATOR
// ============================================================

/**
 * @brief Recursive Least Squares with forgetting factor
 *
 * Algorithm:
 *   K(t)    = P(t-1)φ(t) / (λ + φᵀ(t)P(t-1)φ(t))    [Kalman gain]
 *   θ̂(t)   = θ̂(t-1) + K(t)(y(t) - φᵀ(t)θ̂(t-1))     [parameter update]
 *   P(t)    = (I - K(t)φᵀ(t)) P(t-1) / λ              [covariance update]
 *
 * Forgetting factor λ ∈ (0,1]: λ=1 gives standard LS; λ<1 gives
 * higher weight to recent data (use λ ≈ 0.95-0.99 for time-varying systems).
 *
 * @tparam N  Number of parameters to estimate (compile-time if >0, runtime if
 * 0)
 */
class RLS {
public:
  int n_params_;  ///< Number of parameters
  double lambda_; ///< Forgetting factor λ ∈ (0,1]
  double P0_;     ///< Initial covariance scale (large = more uncertainty)

  Vec theta_hat_;    ///< Current parameter estimate
  Matrix P_;         ///< Covariance matrix
  Vec phi_prev_;     ///< Previous regressor (for prediction error)
  double y_pred_;    ///< Predicted output
  size_t n_samples_; ///< Number of updates performed

  /**
   * @param n_params     Number of parameters
   * @param lambda       Forgetting factor (0 < λ ≤ 1)
   * @param P0           Initial covariance diagonal (large = high uncertainty)
   * @param theta0       Optional initial parameter estimate
   */
  RLS(int n_params, double lambda = 1.0, double P0 = 1e4,
      const Vec &theta0 = {})
      : n_params_(n_params), lambda_(lambda), P0_(P0),
        P_(n_params, n_params, 0.0), n_samples_(0), y_pred_(0.0) {
    // Initialize covariance as P0 * I
    for (int i = 0; i < n_params; ++i)
      P_(i, i) = P0;

    // Initialize parameter estimate
    theta_hat_.assign(n_params, 0.0);
    if (!theta0.empty())
      for (int i = 0; i < std::min(n_params, (int)theta0.size()); ++i)
        theta_hat_[i] = theta0[i];

    phi_prev_.assign(n_params, 0.0);
  }

  /**
   * @brief Update parameter estimate with new measurement
   *
   * @param phi   Regressor vector φ(t) — [n_params] vector
   * @param y     New measurement y(t)
   * @return      Updated parameter estimate θ̂(t)
   */
  const Vec &update(const Vec &phi, double y) {
    if ((int)phi.size() != n_params_)
      throw std::runtime_error("RLS.update: phi dimension mismatch");

    ++n_samples_;
    phi_prev_ = phi;

    // Prediction error: e = y - φᵀθ̂
    double e = y;
    for (int i = 0; i < n_params_; ++i)
      e -= phi[i] * theta_hat_[i];
    y_pred_ = y - e;

    // Kalman gain: K = Pφ / (λ + φᵀPφ)
    Vec Pphi(n_params_, 0.0);
    for (int i = 0; i < n_params_; ++i)
      for (int j = 0; j < n_params_; ++j)
        Pphi[i] += P_(i, j) * phi[j];

    double denom = lambda_;
    for (int i = 0; i < n_params_; ++i)
      denom += phi[i] * Pphi[i];
    if (std::abs(denom) < 1e-12)
      denom = 1e-12;

    Vec K(n_params_);
    for (int i = 0; i < n_params_; ++i)
      K[i] = Pphi[i] / denom;

    // Parameter update: θ̂ ← θ̂ + K·e
    for (int i = 0; i < n_params_; ++i)
      theta_hat_[i] += K[i] * e;

    // Covariance update: P ← (I - Kφᵀ)P / λ
    // P_new(i,j) = [P(i,j) - K(i)*Pphi(j)] / λ
    for (int i = 0; i < n_params_; ++i)
      for (int j = 0; j < n_params_; ++j)
        P_(i, j) = (P_(i, j) - K[i] * Pphi[j]) / lambda_;

    return theta_hat_;
  }

  /// Prediction error at last update: e = y - φᵀθ̂_prev
  double prediction_error(const Vec &phi, double y) const {
    double e = y;
    for (int i = 0; i < n_params_; ++i)
      e -= phi[i] * theta_hat_[i];
    return e;
  }

  /// Covariance trace (monitor for blow-up)
  double covariance_trace() const { return P_.trace(); }

  /// Minimum eigenvalue of P (indicator of PE condition)
  double min_covariance_eigenvalue() const {
    // For PE systems, min eigenvalue of P should go to 0 (converged)
    // For non-PE, covariance grows → min eigenvalue → ∞
    auto evs = P_.eigenvalues();
    double min_re = 1e9;
    for (const auto &ev : evs)
      min_re = std::min(min_re, ev.real());
    return min_re;
  }

  const Vec &theta() const { return theta_hat_; }
  int num_samples() const { return (int)n_samples_; }

  void print() const {
    std::cout << "RLS Estimator: n=" << n_params_ << ", λ=" << lambda_
              << ", samples=" << n_samples_ << "\n";
    std::cout << "  θ̂ = [";
    for (int i = 0; i < n_params_; ++i)
      std::cout << theta_hat_[i] << (i < n_params_ - 1 ? ", " : "");
    std::cout << "]\n";
    std::cout << "  tr(P) = " << covariance_trace() << "\n";
  }
};

// ============================================================
//           RLS WITH PARAMETER PROJECTION
// ============================================================

/**
 * @brief RLS with parameter projection onto a convex parameter set Ω
 *
 * After each update, if θ̂ ∉ Ω, projects back to nearest point in Ω.
 * Typical Ω = {θ: θ_min ≤ θᵢ ≤ θ_max} (box constraint per component).
 *
 * Projection prevents parameter drift and ensures robustness in noisy
 * conditions.
 */
class RLSProjection : public RLS {
public:
  Vec theta_min_, theta_max_; ///< Box constraint bounds

  /**
   * @param n_params    Parameter count
   * @param theta_min   Lower bounds (use -∞ for no bound)
   * @param theta_max   Upper bounds (use +∞ for no bound)
   * @param lambda      Forgetting factor
   */
  RLSProjection(int n_params, const Vec &theta_min, const Vec &theta_max,
                double lambda = 1.0, double P0 = 1e4)
      : RLS(n_params, lambda, P0), theta_min_(theta_min),
        theta_max_(theta_max) {
    if ((int)theta_min_.size() < n_params)
      theta_min_.resize(n_params, -1e6);
    if ((int)theta_max_.size() < n_params)
      theta_max_.resize(n_params, 1e6);
  }

  /**
   * @brief Update with projection
   */
  const Vec &update(const Vec &phi, double y) {
    RLS::update(phi, y);
    // Project: θ̂ = clamp(θ̂, θ_min, θ_max)
    for (int i = 0; i < n_params_; ++i) {
      theta_hat_[i] =
          std::max(theta_min_[i], std::min(theta_max_[i], theta_hat_[i]));
    }
    return theta_hat_;
  }
};

// ============================================================
//         PERSISTENT EXCITATION CHECK
// ============================================================

/**
 * @brief Compute Persistent Excitation index from regressor history
 *
 * PE condition: ∃ T, α₁, α₂ > 0:
 *   α₁ I ≤ ∫_{t}^{t+T} φ(τ)φᵀ(τ) dτ ≤ α₂ I
 *
 * Returns min eigenvalue of the Gramian G = Σ φᵢφᵢᵀ / N.
 * A positive return → PE condition satisfied.
 *
 * @param phi_history   Matrix of regressor vectors (N × n_params)
 * @return              Min eigenvalue of empirical Gramian / N
 */
inline double persistent_excitation_index(const std::vector<Vec> &phi_history) {
  if (phi_history.empty())
    return 0.0;
  int N = (int)phi_history.size();
  int n = (int)phi_history[0].size();

  // Build Gramian G = (1/N) Σ φᵢφᵢᵀ
  Matrix G(n, n, 0.0);
  for (const auto &phi : phi_history) {
    for (int i = 0; i < n; ++i)
      for (int j = 0; j < n; ++j)
        G(i, j) += phi[i] * phi[j];
  }
  // Normalize
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j)
      G(i, j) /= N;

  // Min eigenvalue
  auto evs = G.eigenvalues();
  double min_ev = 1e9;
  for (const auto &ev : evs)
    min_ev = std::min(min_ev, ev.real());
  return min_ev;
}

// ============================================================
//         SIMULATION: RLS ON A KNOWN SYSTEM
// ============================================================

/**
 * @brief Simulate RLS on a 1st or 2nd order system with known true parameters
 *
 * System: y(t) = θ₁·φ₁(t) + θ₂·φ₂(t) (linear regression)
 * Generates time series of parameter estimates.
 */
struct RLSSimResult {
  std::vector<double> time;
  std::vector<Vec> theta_history; ///< θ̂(t) at each step
  std::vector<double> error;      ///< ‖θ̂ - θ_true‖
  std::vector<double> cov_trace;  ///< tr(P(t))
  double final_error;
  double pe_index;
};

inline RLSSimResult
simulate_rls(RLS &rls,
             std::function<Vec(double)> phi_fn, ///< Regressor as function of t
             std::function<double(double)>
                 y_fn, ///< True output y(t) = φᵀ(t)θ_true + noise
             const Vec &theta_true, double T = 30.0, double dt = 0.1) {
  RLSSimResult result;
  std::vector<Vec> phi_history;

  for (double t = 0; t <= T; t += dt) {
    Vec phi = phi_fn(t);
    double y = y_fn(t);

    rls.update(phi, y);
    phi_history.push_back(phi);

    double err = 0;
    for (size_t i = 0; i < std::min(theta_true.size(), rls.theta_hat_.size());
         ++i)
      err += std::pow(rls.theta_hat_[i] - theta_true[i], 2);

    result.time.push_back(t);
    result.theta_history.push_back(rls.theta_hat_);
    result.error.push_back(std::sqrt(err));
    result.cov_trace.push_back(rls.covariance_trace());
  }

  result.final_error = result.error.empty() ? 0.0 : result.error.back();
  result.pe_index = persistent_excitation_index(phi_history);
  return result;
}

} // namespace adaptive
} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_ADAPTIVE_RLS_HPP
