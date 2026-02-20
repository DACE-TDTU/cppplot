/**
 * @file adaptive/adaptive_nonlinear.hpp
 * @brief Adaptive Control for Nonlinear Systems
 *
 * Combines adaptive estimation with nonlinear control design:
 *   - Slotine-Li adaptive computed-torque for robot manipulators
 *   - Certainty Equivalence Principle controller (plug-in adapter)
 *   - RBF Neural Network adaptive controller
 *   - Composite Adaptation (error + prediction error)
 *   - Adaptive Backstepping with online parameter estimation
 *
 * The key theme (Ch08-Ch09): separate the control structure from the
 * parameter estimator. "Modular" design means any estimator can be
 * combined with any stabilizing controller via the CE principle.
 *
 * References:
 *   - Slotine & Li (1987) "On the Adaptive Control of Robot Manipulators"
 *   - Sontag (1992) "Neural nets as systems models and controllers"
 *   - Ioannou & Fidan (2006) "Adaptive Control Tutorial"
 *   - Khalil (2002) "Nonlinear Systems", Ch. 11
 */

#ifndef CPPPLOT_CONTROL_ADAPTIVE_ADAPTIVE_NONLINEAR_HPP
#define CPPPLOT_CONTROL_ADAPTIVE_ADAPTIVE_NONLINEAR_HPP

#include "../../pyplot.hpp"
#include "rls.hpp"
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
//        SLOTINE-LI ADAPTIVE COMPUTED-TORQUE (ROBOT)
// ============================================================

/**
 * @brief Slotine-Li Adaptive Controller for Robot Manipulators
 *
 * Robot dynamics: M(q)q̈ + C(q,q̇)q̇ + G(q) = τ
 *
 * Key insight: dynamics are LINEAR in a set of physical parameters θ:
 *   M(q)q̈ + C(q,q̇)q̇ + G(q) = Y(q, q̇, q̈) · θ
 *
 * Slotine-Li control:
 *   τ = Y(q, q̇, q̇r, r) · θ̂ - Kd·s
 * where s = ė + Λ·e (sliding surface), q̇r = q̇d - Λ·e
 *
 * Adaptation Law (Lyapunov-based):
 *   θ̂̇ = -Γ·Yᵀ·s
 *
 * This is the "regressor-based" modular adaptive control approach.
 */
class SlotineLiController {
public:
  int n_dof_;      ///< Degrees of freedom
  int n_params_;   ///< Number of physical parameters
  double lambda_;  ///< Sliding surface gain Λ
  double kd_;      ///< Damping gain Kd
  Vec theta_hat_;  ///< Parameter estimates
  Vec Gamma_diag_; ///< Adaptation gain matrix (diagonal)

  /**
   * @param n_dof      Robot degrees of freedom
   * @param n_params   Number of physical parameters (e.g., masses, inertias)
   * @param lambda     Sliding variable gain Λ > 0
   * @param kd         Robust damping gain Kd > 0
   * @param gamma_vals Adaptation gains Γᵢ for each parameter
   * @param theta0     Initial parameter estimates
   */
  SlotineLiController(int n_dof, int n_params, double lambda, double kd,
                      const Vec &gamma_vals = {}, const Vec &theta0 = {})
      : n_dof_(n_dof), n_params_(n_params), lambda_(lambda), kd_(kd) {
    theta_hat_ = theta0.empty() ? Vec(n_params, 1.0) : theta0;
    if ((int)theta_hat_.size() < n_params)
      theta_hat_.resize(n_params, 1.0);

    Gamma_diag_ = gamma_vals.empty() ? Vec(n_params, 1.0) : gamma_vals;
    if ((int)Gamma_diag_.size() < n_params)
      Gamma_diag_.resize(n_params, 1.0);
  }

  /**
   * @brief Compute adaptive control torque
   *
   * @param q       Joint positions [n_dof]
   * @param qdot    Joint velocities [n_dof]
   * @param qd      Desired trajectory [n_dof]
   * @param qddot   Desired velocity [n_dof]
   * @param qd_ddot Desired acceleration [n_dof]
   * @param Y_fn    Regressor function Y(q,qdot,qr,rdot) → [n_dof × n_params]
   * matrix (provided as flat vector, row-major)
   * @param dt      Time step for parameter update
   * @return        Torque vector τ [n_dof]
   */
  Vec compute(const Vec &q, const Vec &qdot, const Vec &qd, const Vec &qdot_d,
              const Vec &qddot_d,
              std::function<std::vector<double>(Vec, Vec, Vec, Vec)> Y_fn,
              double dt) {
    // Position and velocity errors
    Vec e(n_dof_), edot(n_dof_);
    for (int i = 0; i < n_dof_; ++i) {
      e[i] = q[i] - qd[i];
      edot[i] = qdot[i] - qdot_d[i];
    }

    // Sliding variable: s = ė + Λ·e
    Vec s(n_dof_);
    for (int i = 0; i < n_dof_; ++i)
      s[i] = edot[i] + lambda_ * e[i];

    // Reference velocity: q̇r = q̇d - Λ·e
    Vec qdot_r(n_dof_);
    for (int i = 0; i < n_dof_; ++i)
      qdot_r[i] = qdot_d[i] - lambda_ * e[i];

    // Reference acceleration: q̈r = q̈d - Λ·ė
    Vec qddot_r(n_dof_);
    for (int i = 0; i < n_dof_; ++i)
      qddot_r[i] = qddot_d[i] - lambda_ * edot[i];

    // Regressor matrix Y (flat, row-major: [n_dof × n_params])
    std::vector<double> Y_flat = Y_fn(q, qdot, qdot_r, qddot_r);

    // Control: τ = Y·θ̂ - Kd·s
    Vec tau(n_dof_, 0.0);
    for (int i = 0; i < n_dof_; ++i) {
      for (int j = 0; j < n_params_; ++j)
        tau[i] += Y_flat[i * n_params_ + j] * theta_hat_[j];
      tau[i] -= kd_ * s[i];
    }

    // Adaptation law: θ̂̇ = -Γ·Yᵀ·s
    Vec theta_dot(n_params_, 0.0);
    for (int j = 0; j < n_params_; ++j) {
      double yt_s = 0;
      for (int i = 0; i < n_dof_; ++i)
        yt_s += Y_flat[i * n_params_ + j] * s[i];
      theta_dot[j] = -Gamma_diag_[j] * yt_s;
      theta_hat_[j] += theta_dot[j] * dt;
    }

    return tau;
  }

  const Vec &theta() const { return theta_hat_; }
};

// ============================================================
//        CERTAINTY EQUIVALENCE PRINCIPLE (MODULAR)
// ============================================================

/**
 * @brief Certainty Equivalence Controller (plug-in adapter)
 *
 * Combines any stabilizing controller (designed for known θ) with
 * any parameter estimator (RLS, MRAC, etc.) via the CE principle:
 *   u(t) = u_nominal(x, θ̂(t))
 *
 * Note: CE does not guarantee stability in general, but is provably
 * stable when combined with suitable estimators satisfying SPR conditions.
 */
class CertaintyEquivalenceController {
public:
  /// Nominal controller: (state, parameters) → control
  std::function<double(Vec, Vec)> controller_fn_;
  /// Parameter estimator update function: (phi, y) → theta_hat
  std::function<Vec(Vec, double)> estimator_fn_;
  Vec theta_hat_;

  CertaintyEquivalenceController(std::function<double(Vec, Vec)> controller,
                                 std::function<Vec(Vec, double)> estimator,
                                 Vec theta_init = {})
      : controller_fn_(controller), estimator_fn_(estimator),
        theta_hat_(theta_init) {}

  double compute(const Vec &x, const Vec &phi, double y) {
    // Update parameter estimate
    theta_hat_ = estimator_fn_(phi, y);
    // Apply nominal controller with estimated parameters
    return controller_fn_(x, theta_hat_);
  }

  const Vec &theta() const { return theta_hat_; }
};

// ============================================================
//        RBF NEURAL NETWORK ADAPTIVE CONTROLLER
// ============================================================

/**
 * @brief Radial Basis Function Network: Ŷ(x) = Ŵᵀφ(x)
 *
 * φᵢ(x) = exp(-‖x - cᵢ‖² / (2σᵢ²))  (Gaussian RBF)
 *
 * Used as a universal approximator for unknown nonlinear functions.
 * Online weight adaptation: Ẇᵢ = -γ·eᵢ·φ(x)  (gradient on tracking error)
 */
class RBFNetwork {
public:
  int n_inputs_;  ///< Input dimension
  int n_centers_; ///< Number of RBF centers

  std::vector<Vec> centers_;    ///< RBF centers [n_centers × n_inputs]
  std::vector<double> sigmas_;  ///< RBF widths [n_centers]
  std::vector<double> weights_; ///< Adaptive weights Ŵ [n_centers]
  double gamma_;                ///< Adaptation gain

  /**
   * @param centers  RBF center locations
   * @param sigmas   RBF widths (one per center, or scalar repeated)
   * @param gamma    Adaptation gain
   */
  RBFNetwork(const std::vector<Vec> &centers, const std::vector<double> &sigmas,
             double gamma = 1.0)
      : n_inputs_((int)(centers.empty() ? 1 : centers[0].size())),
        n_centers_((int)centers.size()), centers_(centers), sigmas_(sigmas),
        gamma_(gamma) {
    weights_.assign(n_centers_, 0.0);
    if ((int)sigmas_.size() < n_centers_) {
      double default_sigma = (sigmas.empty() ? 1.0 : sigmas[0]);
      sigmas_.resize(n_centers_, default_sigma);
    }
  }

  /**
   * @brief Evaluate basis functions φ(x)
   */
  Vec evaluate_basis(const Vec &x) const {
    Vec phi(n_centers_);
    for (int k = 0; k < n_centers_; ++k) {
      double dist_sq = 0;
      for (int i = 0; i < std::min((int)x.size(), n_inputs_); ++i) {
        double d = x[i] - centers_[k][i];
        dist_sq += d * d;
      }
      phi[k] = std::exp(-dist_sq / (2 * sigmas_[k] * sigmas_[k]));
    }
    return phi;
  }

  /**
   * @brief Compute approximation Ŷ = Ŵᵀφ(x)
   */
  double approximate(const Vec &x) const {
    Vec phi = evaluate_basis(x);
    double val = 0;
    for (int k = 0; k < n_centers_; ++k)
      val += weights_[k] * phi[k];
    return val;
  }

  /**
   * @brief Update weights online: Ẇᵢ = -γ·e·φᵢ(x)
   *
   * @param x    Current state
   * @param e    Tracking/prediction error
   * @param dt   Time step
   */
  void update_weights(const Vec &x, double e, double dt) {
    Vec phi = evaluate_basis(x);
    for (int k = 0; k < n_centers_; ++k)
      weights_[k] -= gamma_ * e * phi[k] * dt;
  }

  const std::vector<double> &weights() const { return weights_; }
  double weight_norm() const {
    double norm = 0;
    for (double w : weights_)
      norm += w * w;
    return std::sqrt(norm);
  }
};

/**
 * @brief Factory: Create RBF centers on a regular grid
 *
 * @param x_range   {min, max} per input dimension
 * @param n_per_dim Number of centers per dimension
 * @param sigma     Width of each Gaussian
 */
inline RBFNetwork
make_rbf_grid(const std::vector<std::pair<double, double>> &x_range,
              int n_per_dim = 5, double sigma = 1.0, double gamma = 1.0) {
  int dim = (int)x_range.size();
  std::vector<Vec> centers;

  // Simplified for 1D and 2D (most common cases)
  if (dim == 1) {
    double lo = x_range[0].first, hi = x_range[0].second;
    for (int i = 0; i < n_per_dim; ++i) {
      double c = lo + (hi - lo) * i / (n_per_dim - 1);
      centers.push_back({c});
    }
  } else if (dim == 2) {
    for (int i = 0; i < n_per_dim; ++i) {
      for (int j = 0; j < n_per_dim; ++j) {
        double c1 = x_range[0].first + (x_range[0].second - x_range[0].first) *
                                           i / (n_per_dim - 1);
        double c2 = x_range[1].first + (x_range[1].second - x_range[1].first) *
                                           j / (n_per_dim - 1);
        centers.push_back({c1, c2});
      }
    }
  }

  std::vector<double> sigmas((int)centers.size(), sigma);
  return RBFNetwork(centers, sigmas, gamma);
}

// ============================================================
//        NEURAL NETWORK ADAPTIVE CONTROLLER
// ============================================================

/**
 * @brief NN-based adaptive control for unknown nonlinearities
 *
 * For system: ẋ = f(x) + g(x)u where f(x) is unknown:
 *   Approximate: f(x) ≈ Ŵᵀφ(x) using online-adapted RBF network
 *   Control: u = (1/g₀)(-Ŵᵀφ(x) + ẍd + k₁ė + k₂e)
 *
 * Online weight adaptation using tracking error:
 *   Ẇ = -Γ·φ(x)·s  where s = ė + λ·e
 */
class NNAdaptiveController {
public:
  RBFNetwork nn_;  ///< Approximator for unknown f(x)
  double k1_, k2_; ///< PD-like gains
  double lambda_;  ///< Sliding surface gain
  double g0_;      ///< Known input gain g(x) ≈ g₀

  NNAdaptiveController(RBFNetwork nn, double k1 = 2.0, double k2 = 1.0,
                       double lambda = 1.0, double g0 = 1.0)
      : nn_(nn), k1_(k1), k2_(k2), lambda_(lambda), g0_(g0) {}

  double compute(const Vec &x, double xd, double xdot, double xddot_d,
                 double dt) {
    double e = x[0] - xd;
    double edot = (x.size() > 1) ? (x[1] - xdot) : 0.0;
    double s = edot + lambda_ * e;

    // NN approximation of unknown f(x)
    double f_hat = nn_.approximate(x);

    // Control law
    double u = (1.0 / g0_) * (-f_hat + xddot_d - k1_ * e - k2_ * edot);

    // Weight update
    nn_.update_weights(x, s, dt);

    return u;
  }
};

// ============================================================
//        COMPOSITE ADAPTATION
// ============================================================

/**
 * @brief Composite Adaptation: tracking error + prediction error
 *
 * Standard MRAC only uses tracking error. Composite adaptation also
 * uses prediction error from an identifier running in parallel:
 *   θ̂̇ = -γ₁·eᵀ·Yᵀ  (tracking error term)
 *        -γ₂·ε·Yᵀ   (prediction error term)
 *
 * Achieves faster parameter convergence without PE requirement.
 * Reference: Slotine & Li (1987).
 */
class CompositeAdaptation {
public:
  double gamma_tracking_; ///< Gain for tracking error term
  double gamma_id_;       ///< Gain for identification error term
  Vec theta_hat_;         ///< Parameter estimate
  Vec theta_id_;          ///< Identifier parallel estimate

  CompositeAdaptation(int n_params, double gamma_tracking = 1.0,
                      double gamma_id = 2.0, const Vec &theta0 = {})
      : gamma_tracking_(gamma_tracking), gamma_id_(gamma_id) {
    theta_hat_ = theta0.empty() ? Vec(n_params, 0.0) : theta0;
    theta_id_.assign(n_params, 0.0);
  }

  /**
   * @brief Update with both tracking error and prediction error
   *
   * @param Y_phi    Regressor vector Yϕ (n_params)
   * @param e        Tracking error
   * @param epsilon  Prediction error (from parallel identifier)
   * @param dt       Time step
   */
  void update(const Vec &Y_phi, double e, double epsilon, double dt) {
    for (size_t i = 0; i < std::min(theta_hat_.size(), Y_phi.size()); ++i) {
      double td =
          -gamma_tracking_ * e * Y_phi[i] - gamma_id_ * epsilon * Y_phi[i];
      theta_hat_[i] += td * dt;
    }
  }

  const Vec &theta() const { return theta_hat_; }
};

} // namespace adaptive
} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_ADAPTIVE_ADAPTIVE_NONLINEAR_HPP
