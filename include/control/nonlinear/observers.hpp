/**
 * @file nonlinear/observers.hpp
 * @brief Nonlinear State Observers
 *
 * A unified collection of nonlinear observer designs for systems:
 *   ẋ = f(x, u)
 *   y = h(x)
 *
 * All observers share a common interface:
 *   update(y, u, dt) → estimated state vector x̂
 *
 * Included:
 *   - High-Gain Observer (HGO): parameterized by small ε
 *   - Extended State Observer (ESO/ADRC): estimates total disturbance
 *   - Sliding Mode Observer (SMO / Exact Differentiator)
 *   - Nonlinear Luenberger Observer
 *   - Observer Bank (multiple-model for fault detection)
 *
 * References:
 *   - Khalil (2017) "High-Gain Observers in Nonlinear Feedback Control"
 *   - Han (2009) "From PID to Active Disturbance Rejection Control"
 *   - Fridman et al. (2008) "Higher-order sliding mode observers"
 *   - Gauthier & Kupka (2001) "Deterministic Observation Theory"
 */

#ifndef CPPPLOT_CONTROL_NONLINEAR_OBSERVERS_HPP
#define CPPPLOT_CONTROL_NONLINEAR_OBSERVERS_HPP

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
//        HIGH GAIN OBSERVER (HGO)
// ============================================================

/**
 * @brief High-Gain Observer for systems in observability normal form
 *
 * System: ẋᵢ = xᵢ₊₁, i=1..n-1,  ẋₙ = φ(x,u),  y = x₁
 * (n-th order chain of integrators with nonlinear last equation)
 *
 * Observer: d/dt x̂ᵢ = x̂ᵢ₊₁ + aᵢ/εⁱ · (y - x̂₁),  i=1..n
 * where aᵢ > 0 are chosen so that s^n + a₁s^{n-1}+...+aₙ is Hurwitz
 * and ε ∈ (0,1) is a small peaking parameter.
 *
 * Key property: estimates converge in O(ε) neighborhood of true state.
 * Peaking phenomenon: initial transient is O(1/εⁱ) for state i.
 */
class HighGainObserver {
public:
  int n_;                    ///< Observer order
  double eps_;               ///< Peaking parameter ε ∈ (0,1)
  std::vector<double> a_;    ///< Observer gains [a₁, a₂, ..., aₙ]
  std::vector<double> xhat_; ///< Current state estimate

  /// Optional nonlinear term φ(x̂, u) for last state
  std::function<double(Vec, double)> phi_;

  /**
   * @param n          Observer order (= system order)
   * @param epsilon    Small peaking parameter (0 < ε < 1, smaller = faster)
   * @param hurwitz_a  Coefficients of Hurwitz polynomial (default: [n+1 choose
   * k] Butterworth)
   * @param phi        Optional: model of fₙ(x,u) for last state (use 0 if
   * unknown)
   */
  HighGainObserver(int n, double epsilon, std::vector<double> hurwitz_a = {},
                   std::function<double(Vec, double)> phi = nullptr)
      : n_(n), eps_(epsilon), xhat_(n, 0.0), phi_(phi) {
    if (hurwitz_a.empty()) {
      // Default: Hurwitz polynomial coefficients for (s+1)^n (all real at -1)
      // Binomial coefficients: C(n,k)
      a_.resize(n);
      for (int k = 0; k < n; ++k) {
        // C(n, k+1): binomial coefficient
        double binom = 1.0;
        for (int j = 0; j <= k; ++j)
          binom = binom * (n - j) / (j + 1);
        a_[k] = binom;
      }
    } else {
      a_ = hurwitz_a;
      if ((int)a_.size() < n_)
        a_.resize(n_, 1.0);
    }
  }

  void reset(const Vec &x0 = {}) {
    xhat_.assign(n_, 0.0);
    if (!x0.empty()) {
      for (int i = 0; i < std::min(n_, (int)x0.size()); ++i)
        xhat_[i] = x0[i];
    }
  }

  /**
   * @brief Update observer
   *
   * @param y   Measured output y = x₁
   * @param u   Control input
   * @param dt  Time step
   * @return    Updated state estimate x̂
   */
  const Vec &update(double y, double u, double dt) {
    double e_y = y - xhat_[0]; // Output injection error

    // Euler integration of observer equations
    Vec xhat_dot(n_);
    double eps_pow = eps_; // εⁱ

    for (int i = 0; i < n_ - 1; ++i) {
      xhat_dot[i] = xhat_[i + 1] + (a_[i] / eps_pow) * e_y;
      eps_pow *= eps_;
    }

    // Last state: + φ term (if model available)
    double phi_val = phi_ ? phi_(xhat_, u) : 0.0;
    xhat_dot[n_ - 1] = phi_val + (a_[n_ - 1] / eps_pow) * e_y;

    for (int i = 0; i < n_; ++i)
      xhat_[i] += xhat_dot[i] * dt;

    return xhat_;
  }

  const Vec &state() const { return xhat_; }
  double output() const { return xhat_.empty() ? 0.0 : xhat_[0]; }
};

// ============================================================
//        EXTENDED STATE OBSERVER (ESO / ADRC)
// ============================================================

/**
 * @brief Extended State Observer for Active Disturbance Rejection Control
 *
 * Augments system with "total disturbance" as an extra state.
 * For n-th order system: ẋ₁=x₂, ..., ẋₙ=f(x)+d(t)+b₀u, y=x₁
 * ESO estimates [x₁, ..., xₙ, f(x)+d(t)] as [ẑ₁, ..., ẑₙ, ẑₙ₊₁]
 *
 * ESO equations (linear parameterization):
 *   ḑ₁ = ẑ₂ + β₁·(y - ẑ₁)
 *   ḑ₂ = ẑ₃ + β₂·(y - ẑ₁)
 *   ...
 *   ḑₙ = ẑₙ₊₁ + βₙ·(y - ẑ₁) + b₀·u
 *   ḑₙ₊₁ = βₙ₊₁·(y - ẑ₁)
 *
 * Gains: βᵢ = C(n+1,i) · ωₒⁱ  (bandwidth parameterization)
 */
class ExtendedStateObserver {
public:
  int n_;                    ///< System order
  double omega_o_;           ///< Observer bandwidth [rad/s]
  double b0_;                ///< Input gain estimate b₀
  std::vector<double> beta_; ///< Gains [β₁, ..., βₙ₊₁]
  std::vector<double> zhat_; ///< ESO state [ẑ₁, ..., ẑₙ, ẑₙ₊₁]

  /**
   * @param n         System order
   * @param omega_o   Observer bandwidth (larger = faster, but sensitive to
   * noise)
   * @param b0        Input channel gain estimate (from model or tuning)
   */
  ExtendedStateObserver(int n, double omega_o, double b0 = 1.0)
      : n_(n), omega_o_(omega_o), b0_(b0), zhat_(n + 1, 0.0) {
    // Bandwidth parameterization: βᵢ = C(n+1, i) · ωₒⁱ
    beta_.resize(n + 1);
    double wo_pow = omega_o_;
    for (int i = 0; i < n + 1; ++i) {
      // Binomial C(n+1, i+1)
      double binom = 1.0;
      for (int j = 0; j <= i; ++j)
        binom = binom * (n + 1 - j) / (j + 1);
      beta_[i] = binom * wo_pow;
      wo_pow *= omega_o_;
    }
  }

  void reset(double y0 = 0.0) {
    zhat_.assign(n_ + 1, 0.0);
    zhat_[0] = y0;
  }

  /**
   * @brief Update ESO
   *
   * @param y   Output measurement y = x₁
   * @param u   Control input
   * @param dt  Time step
   * @return    State estimate [ẑ₁, ..., ẑₙ, ẑₙ₊₁ (total disturbance)]
   */
  const Vec &update(double y, double u, double dt) {
    double e = y - zhat_[0]; // innovation

    Vec zdot(n_ + 1);
    for (int i = 0; i < n_ - 1; ++i)
      zdot[i] = zhat_[i + 1] + beta_[i] * e;
    zdot[n_ - 1] = zhat_[n_] + beta_[n_ - 1] * e + b0_ * u;
    zdot[n_] = beta_[n_] * e; // disturbance rate = 0 (unknown)

    for (int i = 0; i <= n_; ++i)
      zhat_[i] += zdot[i] * dt;

    return zhat_;
  }

  const Vec &state() const { return zhat_; }

  /// Estimated total disturbance f(x) + d(t)
  double disturbance_estimate() const {
    return zhat_.empty() ? 0.0 : zhat_.back();
  }

  /**
   * @brief ADRC control law using ESO
   *
   * u = (u₀ - ẑₙ₊₁) / b₀
   * where u₀ is a PD/reference-tracking term: u₀ = kp*(r-ẑ₁) + kd*(ṙ-ẑ₂)
   *
   * @param r      Reference
   * @param rdot   Reference derivative
   * @param kp, kd PD gains for the linearized system
   */
  double adrc_control(double r, double rdot, double kp, double kd) const {
    if (zhat_.size() < 3)
      return 0.0;
    double u0 = kp * (r - zhat_[0]) + kd * (rdot - zhat_[1]);
    return (u0 - zhat_[n_]) / b0_;
  }
};

// ============================================================
//        SLIDING MODE OBSERVER (Exact Differentiator)
// ============================================================

/**
 * @brief Super-Twisting Exact Differentiator (Levant 2003)
 *
 * Given signal y(t), estimates y̆ and ẏ exactly in finite time:
 *   ż₁ = -λ₁ |z₁ - y|^{1/2} sign(z₁ - y) + z₂
 *   ż₂ = -λ₂ sign(z₁ - y)
 *
 * Can be cascaded n-1 times to get the nth differentiator.
 * This implementation provides a 2nd order differentiator (n=2: y, ẏ).
 *
 * Reference: Levant (2003) "Higher-order sliding modes, differentiation and
 *            output-feedback control"
 */
class SlidingModeObserver {
public:
  double lambda1_,
      lambda2_;      ///< Gains (must satisfy λ₁>2√L, λ₂>L for Lipschitz L)
  double z1_, z2_;   ///< Observer states: z₁ ≈ y, z₂ ≈ ẏ
  double sat_level_; ///< Saturation for sign function approximation

  SlidingModeObserver(double lambda1 = 1.5, double lambda2 = 1.1,
                      double sat = 1e6)
      : lambda1_(lambda1), lambda2_(lambda2), z1_(0.0), z2_(0.0),
        sat_level_(sat) {}

  void reset(double y0 = 0.0) {
    z1_ = y0;
    z2_ = 0.0;
  }

  /**
   * @brief Update and return [estimated_y, estimated_ydot]
   *
   * @param y   Noisy measurement of y
   * @param dt  Time step
   * @return    {y_estimate, ydot_estimate}
   */
  std::pair<double, double> update(double y, double dt) {
    double e = z1_ - y;
    double e_sqrt = std::sqrt(std::abs(e));
    double sgn_e = (e > 0) ? 1.0 : ((e < 0) ? -1.0 : 0.0);

    double z1_dot = -lambda1_ * e_sqrt * sgn_e + z2_;
    double z2_dot = -lambda2_ * sgn_e;

    // Euler integration (use smaller dt for accuracy)
    z1_ += z1_dot * dt;
    z2_ += z2_dot * dt;

    return {z1_, z2_};
  }

  double y_estimate() const { return z1_; }
  double ydot_estimate() const { return z2_; }
};

// ============================================================
//        NONLINEAR LUENBERGER OBSERVER
// ============================================================

/**
 * @brief Nonlinear Luenberger Observer
 *
 * For system: ẋ = f(x, u),  y = h(x)
 *
 * Observer: ẋ̂ = f(x̂, u) + L(y - h(x̂))
 * where L is the observer gain vector/matrix.
 *
 * Convergence requires the error dynamics to be asymptotically stable.
 * Gain L is usually designed via linearization at an operating point.
 */
class NonlinearLuenberger {
public:
  std::function<Vec(Vec, double)> f_; ///< System dynamics ẋ = f(x, u)
  std::function<double(Vec)> h_;      ///< Output y = h(x)
  std::vector<double> L_;             ///< Observer gain vector
  Vec xhat_;                          ///< State estimate

  NonlinearLuenberger(std::function<Vec(Vec, double)> f,
                      std::function<double(Vec)> h, std::vector<double> L)
      : f_(f), h_(h), L_(L), xhat_(L.size(), 0.0) {}

  void reset(const Vec &x0 = {}) {
    xhat_.assign(L_.size(), 0.0);
    for (size_t i = 0; i < std::min(x0.size(), xhat_.size()); ++i)
      xhat_[i] = x0[i];
  }

  /**
   * @brief Update observer (RK4)
   *
   * @param y    Measurement y = h(x)
   * @param u    Control input
   * @param dt   Time step
   * @return     Updated state estimate x̂
   */
  const Vec &update(double y, double u, double dt) {
    double innovation = y - h_(xhat_); // y - h(x̂)

    auto obs_dyn = [&](Vec xh) -> Vec {
      Vec fv = f_(xh, u);
      for (size_t i = 0; i < std::min(fv.size(), L_.size()); ++i)
        fv[i] += L_[i] * innovation;
      return fv;
    };

    Vec k1 = obs_dyn(xhat_);
    Vec xk2(xhat_.size());
    for (size_t i = 0; i < xhat_.size(); ++i)
      xk2[i] = xhat_[i] + 0.5 * dt * k1[i];
    Vec k2 = obs_dyn(xk2);
    Vec xk3(xhat_.size());
    for (size_t i = 0; i < xhat_.size(); ++i)
      xk3[i] = xhat_[i] + 0.5 * dt * k2[i];
    Vec k3 = obs_dyn(xk3);
    Vec xk4(xhat_.size());
    for (size_t i = 0; i < xhat_.size(); ++i)
      xk4[i] = xhat_[i] + dt * k3[i];
    Vec k4 = obs_dyn(xk4);
    for (size_t i = 0; i < xhat_.size(); ++i)
      xhat_[i] += dt / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);

    return xhat_;
  }

  const Vec &state() const { return xhat_; }
};

// ============================================================
//        OBSERVER BANK (Multiple-Model)
// ============================================================

/**
 * @brief Observer Bank for fault detection / isolation
 *
 * Runs N ESO observers in parallel (e.g., for N possible fault modes).
 * Identifies the best-matching model by minimum innovation norm.
 */
class ObserverBank {
public:
  std::vector<ExtendedStateObserver> observers_;
  std::vector<std::string> labels_;
  std::vector<double> innovations_; ///< Latest innovation |y - ŷ|

  ObserverBank() = default;

  void add(ExtendedStateObserver obs, const std::string &label = "") {
    observers_.push_back(std::move(obs));
    labels_.push_back(
        label.empty() ? "Model_" + std::to_string(observers_.size()) : label);
    innovations_.push_back(0.0);
  }

  /**
   * @brief Update all observers and return index of best match
   *
   * @return Index of observer with minimum innovation
   */
  int update(double y, double u, double dt) {
    for (size_t i = 0; i < observers_.size(); ++i) {
      double y_prev =
          observers_[i].state().empty() ? 0.0 : observers_[i].state()[0];
      observers_[i].update(y, u, dt);
      innovations_[i] = std::abs(y - observers_[i].state()[0]);
    }
    return (int)(std::min_element(innovations_.begin(), innovations_.end()) -
                 innovations_.begin());
  }

  const ExtendedStateObserver &best_observer() const {
    size_t idx = std::min_element(innovations_.begin(), innovations_.end()) -
                 innovations_.begin();
    return observers_[idx];
  }
};

// ============================================================
//        OBSERVABILITY RANK (LOCAL)
// ============================================================

/**
 * @brief Compute local observability rank for SISO nonlinear system
 *
 * Constructs the observability codistribution matrix:
 * O = [dh; d(L_f h); d(L_f^2 h); ...]  (n rows, each a gradient)
 * System is locally observable at x if rank(O(x)) = n.
 *
 * @param f        System drift ẋ = f(x)
 * @param h        Output y = h(x)
 * @param x        Evaluation point
 * @param n        System order (number of states)
 * @return         Computed rank (integer 0..n)
 */
inline int observability_rank(std::function<Vec(Vec)> f,
                              std::function<double(Vec)> h, const Vec &x,
                              int n = -1) {
  int dim = (n > 0) ? n : (int)x.size();
  double step = 1e-5;

  // Build observability matrix rows: [∇(L_f^k h)]ᵀ for k = 0..dim-1
  // Each row has dim entries (gradient)
  std::vector<std::vector<double>> O_rows;

  std::function<double(Vec)> Lfk_h = h;
  for (int k = 0; k < dim; ++k) {
    // Numeric gradient of Lfk_h at x
    std::vector<double> row(dim);
    for (int i = 0; i < dim; ++i) {
      Vec xp = x, xm = x;
      xp[i] += step;
      xm[i] -= step;
      row[i] = (Lfk_h(xp) - Lfk_h(xm)) / (2 * step);
    }
    O_rows.push_back(row);

    // Compute L_f^(k+1) h = ∇(L_f^k h) · f(x) (new ScalarField for next)
    auto prev = Lfk_h;
    Lfk_h = [prev, f, step](Vec xv) -> double {
      Vec fv = f(xv);
      double val = 0;
      for (size_t i = 0; i < xv.size(); ++i) {
        Vec xp = xv, xm = xv;
        xp[i] += step;
        xm[i] -= step;
        val += (prev(xp) - prev(xm)) / (2 * step) * fv[i];
      }
      return val;
    };
  }

  // Compute rank via Gaussian elimination
  auto rows = O_rows; // copy
  int rank = 0;
  double tol = 1e-8;

  for (int col = 0; col < dim && rank < dim; ++col) {
    // Find pivot
    int pivot = -1;
    for (int row = rank; row < (int)rows.size(); ++row) {
      if (std::abs(rows[row][col]) > tol) {
        pivot = row;
        break;
      }
    }
    if (pivot < 0)
      continue;
    std::swap(rows[rank], rows[pivot]);
    double scale = rows[rank][col];
    for (int j = col; j < dim; ++j)
      rows[rank][j] /= scale;
    for (int row = 0; row < (int)rows.size(); ++row) {
      if (row == rank || std::abs(rows[row][col]) < tol)
        continue;
      double factor = rows[row][col];
      for (int j = col; j < dim; ++j)
        rows[row][j] -= factor * rows[rank][j];
    }
    ++rank;
  }
  return rank;
}

} // namespace nonlinear
} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_NONLINEAR_OBSERVERS_HPP
