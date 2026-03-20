/**
 * @file mpc.hpp
 * @brief Model Predictive Control (MPC) Implementation
 *
 * MPC solves an optimization problem at each time step:
 *   min  Σ_{k=0}^{N-1} (x_k'Qx_k + u_k'Ru_k) + x_N'P_f*x_N
 *   s.t. x_{k+1} = Ax_k + Bu_k
 *        u_min ≤ u_k ≤ u_max
 *        x_min ≤ x_k ≤ x_max
 *
 * Features:
 * - Unconstrained MPC (analytical solution)
 * - Constrained MPC with simple box constraints
 * - Tracking MPC for reference following
 * - Terminal cost with LQR for stability guarantee
 *
 * @author CppPlot Control Systems Library
 * @date January 2026
 *
 * References:
 * [1] Maciejowski - "Predictive Control with Constraints"
 * [2] Rawlings & Mayne - "Model Predictive Control: Theory and Design"
 * [3] Borrelli, Bemporad, Morari - "Predictive Control"
 */

#ifndef CPPPLOT_CONTROL_MPC_HPP
#define CPPPLOT_CONTROL_MPC_HPP

#include "controller_design.hpp"
#include "state_space.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace cppplot {
namespace control {

/**
 * @brief MPC Solver Type
 */
enum class MPCSolver { PGD, ADMM };

/**
 * @brief MPC Configuration structure
 */
struct MPCConfig {
  size_t horizon = 10; // Prediction horizon N

  // Standard Weights
  Matrix Q;          // State weight (n x n)
  Matrix R;          // Input weight (m x m)
  Matrix P_terminal; // Terminal state weight (n x n)

  // Constraints
  std::vector<double> u_min; // Input lower bounds
  std::vector<double> u_max; // Input upper bounds
  std::vector<double> x_min; // State lower bounds
  std::vector<double> x_max; // State upper bounds

  // Reference tracking
  Matrix x_ref; // State reference (n x 1)
  Matrix u_ref; // Input reference (m x 1)

  // Solver Setup
  MPCSolver solver_type = MPCSolver::PGD;
  bool use_terminal_cost = true; // Use LQR terminal cost

  // PGD Parameters
  size_t max_iter = 100;
  double tolerance = 1e-4;

  // ADMM Parameters
  double rho = 1.0;           ///< ADMM step size (penalty parameter)
  double eps_abs = 1e-4;      ///< ADMM absolute convergence tolerance
  double eps_rel = 1e-3;      ///< ADMM relative convergence tolerance
  size_t admm_max_iter = 100; ///< ADMM hard iteration cap
  size_t check_every = 5;     ///< ADMM convergence check frequency
  bool warm_start = true;     ///< Enable ADMM warm-starting

  /**
   * @brief Default constructor
   */
  MPCConfig() {}

  /**
   * @brief Create simple config with weights only
   */
  MPCConfig(size_t N, const Matrix &Q_, const Matrix &R_)
      : horizon(N), Q(Q_), R(R_) {}
};

/**
 * @brief MPC Solution structure
 */
struct MPCSolution {
  Matrix u_opt;   // Optimal control sequence (m*N x 1)
  Matrix x_pred;  // Predicted state trajectory (n*(N+1) x 1)
  double cost;    // Optimal cost
  int iterations; // Number of solver iterations
  bool converged; // Convergence status

  /**
   * @brief Get first control action (to apply to plant)
   */
  Matrix getFirstControl(size_t m) const {
    Matrix u0(m, 1);
    for (size_t i = 0; i < m; ++i) {
      u0(i, 0) = u_opt(i, 0);
    }
    return u0;
  }

  /**
   * @brief Get control at step k
   */
  Matrix getControl(size_t k, size_t m) const {
    Matrix uk(m, 1);
    for (size_t i = 0; i < m; ++i) {
      uk(i, 0) = u_opt(k * m + i, 0);
    }
    return uk;
  }

  /**
   * @brief Get predicted state at step k
   */
  Matrix getState(size_t k, size_t n) const {
    Matrix xk(n, 1);
    for (size_t i = 0; i < n; ++i) {
      xk(i, 0) = x_pred(k * n + i, 0);
    }
    return xk;
  }
};

// ============================================================
//  Dynamic Cholesky — mirrors qp::cholesky / qp::chol_solve
//  but operates on cppplot::Matrix (dynamic size).
//  Used by MPCController::solveConstrainedADMM() to avoid
//  the O(N^3) dense inverse H_admm.inv() (P1 fix).
// ============================================================

/**
 * @brief In-place Cholesky: A = L*L', stores L in lower triangle.
 * @return true if A is positive definite, false otherwise.
 */
inline bool cholesky_dynamic(Matrix &L, const Matrix &A) {
  size_t n = A.rows;
  L = A;
  for (size_t j = 0; j < n; ++j) {
    double s = L(j, j);
    for (size_t k = 0; k < j; ++k)
      s -= L(j, k) * L(j, k);
    if (s < 1e-14)
      return false;
    L(j, j) = std::sqrt(s);
    for (size_t i = j + 1; i < n; ++i) {
      double t = L(i, j);
      for (size_t k = 0; k < j; ++k)
        t -= L(i, k) * L(j, k);
      L(i, j) = t / L(j, j);
    }
  }
  return true;
}

/**
 * @brief Solve L*L'*x = b using pre-computed lower-triangular L.
 * Two triangular solves: forward then backward. O(N^2).
 */
inline Matrix chol_solve_dynamic(const Matrix &L, const Matrix &b) {
  size_t n = L.rows;
  Matrix y = Matrix::zeros(n, 1);
  Matrix x = Matrix::zeros(n, 1);

  // Forward substitution: L*y = b
  for (size_t i = 0; i < n; ++i) {
    double s = b(i, 0);
    for (size_t k = 0; k < i; ++k)
      s -= L(i, k) * y(k, 0);
    y(i, 0) = s / L(i, i);
  }

  // Backward substitution: L'*x = y
  for (int i = static_cast<int>(n) - 1; i >= 0; --i) {
    double s = y(i, 0);
    for (size_t k = static_cast<size_t>(i) + 1; k < n; ++k)
      s -= L(k, static_cast<size_t>(i)) * x(k, 0);
    x(static_cast<size_t>(i), 0) = s / L(static_cast<size_t>(i),
                                          static_cast<size_t>(i));
  }

  return x;
}

/**
 * @brief Model Predictive Controller
 *
 * Implements MPC for linear discrete-time systems:
 *   x(k+1) = A*x(k) + B*u(k)
 *   y(k)   = C*x(k)
 */
class MPCController {
private:
  Matrix A_, B_, C_;
  size_t n_, m_, p_; // state, input, output dimensions
  MPCConfig config_;

  // Pre-computed matrices for unconstrained MPC
  Matrix Phi_;     // Prediction matrix for states from x0
  Matrix Gamma_;   // Prediction matrix for states from U
  Matrix H_;       // Hessian for QP: H = Gamma'*Q_bar*Gamma + R_bar
  Matrix L_admm_;  // Cholesky factor of (H + rho*I) — replaces dense inverse
  Matrix F_;       // Linear term: F = Gamma'*Q_bar*Phi

  bool matrices_computed_ = false;

  // ADMM warm-start state (persists across solve() calls)
  Matrix admm_x_, admm_z_, admm_u_;
  bool admm_initialized_ = false;

public:
  /**
   * @brief Construct MPC controller
   *
   * @param A System matrix (discrete-time)
   * @param B Input matrix
   * @param C Output matrix
   * @param config MPC configuration
   */
  MPCController(const Matrix &A, const Matrix &B, const Matrix &C,
                const MPCConfig &config)
      : A_(A), B_(B), C_(C), config_(config) {
    n_ = A.rows;
    m_ = B.cols;
    p_ = C.rows;

    // Validate dimensions
    if (config_.Q.rows == 0) {
      config_.Q = Matrix::eye(n_);
    }
    if (config_.R.rows == 0) {
      config_.R = Matrix::eye(m_);
    }

    // Compute terminal cost using DARE (discrete Riccati) if requested
    if (config_.use_terminal_cost && config_.P_terminal.rows == 0) {
      config_.P_terminal = dare(A_, B_, config_.Q, config_.R);
    }

    computePredictionMatrices();
  }

  /**
   * @brief Solve MPC problem for current state
   *
   * @param x0 Current state (n x 1)
   * @return MPC solution with optimal control sequence
   */
  MPCSolution solve(const Matrix &x0) {
    // Check for constraints
    bool has_input_constraints =
        !config_.u_min.empty() || !config_.u_max.empty();
    bool has_state_constraints =
        !config_.x_min.empty() || !config_.x_max.empty();

    if (has_input_constraints || has_state_constraints) {
      return solveConstrained(x0);
    } else {
      return solveUnconstrained(x0);
    }
  }

  /**
   * @brief Get configuration
   */
  const MPCConfig &getConfig() const { return config_; }

  /**
   * @brief Update configuration
   */
  void setConfig(const MPCConfig &config) {
    config_ = config;
    matrices_computed_ = false;
    admm_initialized_ = false; // invalidate warm-start on config change
    computePredictionMatrices();
  }

  /**
   * @brief Update reference for tracking MPC
   */
  void setReference(const Matrix &x_ref, const Matrix &u_ref = Matrix()) {
    config_.x_ref = x_ref;
    if (u_ref.rows > 0) {
      config_.u_ref = u_ref;
    }
  }

  /**
   * @brief Reset ADMM warm-start state to zero.
   *
   * Call this when there is a large change in operating point,
   * reference step, or after a fault — to avoid stale warm-start
   * degrading convergence.
   */
  void resetADMM() { admm_initialized_ = false; }

private:
  /**
   * @brief Compute prediction matrices for batch MPC formulation
   *
   * State predictions: X = Phi*x0 + Gamma*U
   * where:
   *   X = [x1; x2; ...; x_N]       (n*N x 1)
   *   U = [u0; u1; ...; u_{N-1}]   (m*N x 1)
   *
   *   Phi = [A; A²; A³; ...; A^N]  (n*N x n)
   *
   *   Gamma = [B       0       0    ...  0   ]  (n*N x m*N)
   *           [AB      B       0    ...  0   ]
   *           [A²B     AB      B    ...  0   ]
   *           [...     ...     ...  ...  ... ]
   *           [A^{N-1}B A^{N-2}B ... AB   B  ]
   */
  void computePredictionMatrices() {
    if (matrices_computed_)
      return;

    size_t N = config_.horizon;

    // Build Phi matrix
    Phi_ = Matrix::zeros(n_ * N, n_);
    Matrix A_power = A_;
    for (size_t i = 0; i < N; ++i) {
      for (size_t r = 0; r < n_; ++r) {
        for (size_t c = 0; c < n_; ++c) {
          Phi_(i * n_ + r, c) = A_power(r, c);
        }
      }
      A_power = A_power * A_;
    }

    // Build Gamma matrix
    Gamma_ = Matrix::zeros(n_ * N, m_ * N);
    for (size_t i = 0; i < N; ++i) {
      Matrix A_power_B = B_;
      for (size_t j = 0; j <= i; ++j) {
        size_t row_start = i * n_;
        size_t col_start = (i - j) * m_;

        // Gamma(row_start:row_start+n_, col_start:col_start+m_) = A^j * B
        for (size_t r = 0; r < n_; ++r) {
          for (size_t c = 0; c < m_; ++c) {
            Gamma_(row_start + r, col_start + c) = A_power_B(r, c);
          }
        }

        A_power_B = A_ * A_power_B;
      }
    }

    // Build Q_bar (block diagonal)
    Matrix Q_bar = Matrix::zeros(n_ * N, n_ * N);
    for (size_t i = 0; i < N; ++i) {
      // Use terminal cost for last step if available
      const Matrix &Qi = (i == N - 1 && config_.P_terminal.rows > 0)
                             ? config_.P_terminal
                             : config_.Q;

      for (size_t r = 0; r < n_; ++r) {
        for (size_t c = 0; c < n_; ++c) {
          Q_bar(i * n_ + r, i * n_ + c) = Qi(r, c);
        }
      }
    }

    // Build R_bar (block diagonal)
    Matrix R_bar = Matrix::zeros(m_ * N, m_ * N);
    for (size_t i = 0; i < N; ++i) {
      for (size_t r = 0; r < m_; ++r) {
        for (size_t c = 0; c < m_; ++c) {
          R_bar(i * m_ + r, i * m_ + c) = config_.R(r, c);
        }
      }
    }

    // Compute Hessian: H = Gamma'*Q_bar*Gamma + R_bar
    Matrix GtQ = Gamma_.T() * Q_bar;
    H_ = GtQ * Gamma_ + R_bar;

    // Cholesky-factorize (H + rho*I) once — reused every solve() call.
    // Replaces the O(N^3) dense inverse with O(N^2) triangular solves (P1 fix).
    Matrix H_admm = H_;
    for (size_t i = 0; i < m_ * config_.horizon; ++i)
      H_admm(i, i) += config_.rho;
    bool chol_ok = cholesky_dynamic(L_admm_, H_admm);
    if (!chol_ok) {
      // Fallback: increase regularization and retry once
      for (size_t i = 0; i < m_ * config_.horizon; ++i)
        H_admm(i, i) += 1e-6;
      cholesky_dynamic(L_admm_, H_admm);
    }
    // Reset warm-start state whenever matrices are recomputed
    admm_initialized_ = false;

    // Compute F: F = Gamma'*Q_bar*Phi
    F_ = GtQ * Phi_;

    matrices_computed_ = true;
  }

  /**
   * @brief Solve unconstrained MPC (analytical solution)
   *
   * Cost: J = X'*Q_bar*X + U'*R_bar*U
   *       = (Phi*x0 + Gamma*U)'*Q_bar*(Phi*x0 + Gamma*U) + U'*R_bar*U
   *       = U'*H*U + 2*x0'*F'*U + const
   *
   * Optimal: U* = -H^{-1}*F*x0
   */
  MPCSolution solveUnconstrained(const Matrix &x0) {
    MPCSolution sol;
    size_t N = config_.horizon;

    // Solve: U* = -H^(-1) * F * x0
    Matrix Fx0 = F_ * x0;

    // Add regularization for numerical stability
    Matrix H_reg = H_;
    double reg = 1e-8;
    for (size_t i = 0; i < m_ * N; ++i) {
      H_reg(i, i) += reg;
    }

    // Use direct inverse
    Matrix H_inv = H_reg.inv();

    // Compute U* = -H^{-1} * F * x0
    sol.u_opt = H_inv * Fx0;
    for (size_t i = 0; i < m_ * N; ++i) {
      sol.u_opt(i, 0) = -sol.u_opt(i, 0);
    }

    // Compute predicted states
    sol.x_pred = Matrix::zeros(n_ * (N + 1), 1);
    for (size_t i = 0; i < n_; ++i) {
      sol.x_pred(i, 0) = x0(i, 0);
    }

    Matrix Phi_x0 = Phi_ * x0;
    Matrix Gamma_U = Gamma_ * sol.u_opt;
    for (size_t k = 0; k < N; ++k) {
      for (size_t i = 0; i < n_; ++i) {
        sol.x_pred((k + 1) * n_ + i, 0) =
            Phi_x0(k * n_ + i, 0) + Gamma_U(k * n_ + i, 0);
      }
    }

    // Compute cost
    Matrix X = Phi_x0 + Gamma_U;
    // Build Q_bar again for cost computation
    double cost = 0;
    for (size_t k = 0; k < N; ++k) {
      const Matrix &Qk = (k == N - 1 && config_.P_terminal.rows > 0)
                             ? config_.P_terminal
                             : config_.Q;
      // x_k' * Q * x_k
      for (size_t i = 0; i < n_; ++i) {
        for (size_t j = 0; j < n_; ++j) {
          cost += X(k * n_ + i, 0) * Qk(i, j) * X(k * n_ + j, 0);
        }
      }
      // u_k' * R * u_k
      for (size_t i = 0; i < m_; ++i) {
        for (size_t j = 0; j < m_; ++j) {
          cost += sol.u_opt(k * m_ + i, 0) * config_.R(i, j) *
                  sol.u_opt(k * m_ + j, 0);
        }
      }
    }
    sol.cost = cost;
    sol.iterations = 1;
    sol.converged = true;

    return sol;
  }

  /**
   * @brief Solve constrained MPC using projected gradient descent
   *
   * Simple box-constrained QP solver:
   *   min  0.5*U'*H*U + g'*U
   *   s.t. u_min ≤ u ≤ u_max
   *
   * Algorithm: Projected gradient descent
   *   U(k+1) = project(U(k) - alpha * (H*U(k) + g))
   */
  MPCSolution solveConstrained(const Matrix &x0) {
    if (config_.solver_type == MPCSolver::ADMM) {
      return solveConstrainedADMM(x0);
    }
    return solveConstrainedPGD(x0);
  }

  /**
   * @brief Solve constrained MPC using projected gradient descent
   *
   * Simple box-constrained QP solver:
   *   min  0.5*U'*H*U + g'*U
   *   s.t. u_min ≤ u ≤ u_max
   *
   * Algorithm: Projected gradient descent
   *   U(k+1) = project(U(k) - alpha * (H*U(k) + g))
   */
  MPCSolution solveConstrainedPGD(const Matrix &x0) {
    MPCSolution sol;
    size_t N = config_.horizon;

    // Linear term: g = F * x0
    Matrix g = F_ * x0;

    // Initialize with unconstrained solution, then project
    Matrix U = Matrix::zeros(m_ * N, 1);

    // Gradient descent with projection
    double alpha = 0.01; // Step size (could use line search)

    for (size_t iter = 0; iter < config_.max_iter; ++iter) {
      // Gradient: grad = H*U + g
      Matrix grad = H_ * U + g;

      // Check convergence
      double grad_norm = 0;
      for (size_t i = 0; i < m_ * N; ++i) {
        grad_norm += grad(i, 0) * grad(i, 0);
      }
      grad_norm = std::sqrt(grad_norm);

      if (grad_norm < config_.tolerance) {
        sol.iterations = static_cast<int>(iter);
        sol.converged = true;
        break;
      }

      // Update with adaptive step size
      // Use Barzilai-Borwein step size approximation
      Matrix U_new = U;
      for (size_t i = 0; i < m_ * N; ++i) {
        U_new(i, 0) = U(i, 0) - alpha * grad(i, 0);
      }

      // Project onto constraints
      projectOntoConstraints(U_new);

      // Update step size based on progress
      Matrix dU = U_new - U;
      double dU_norm = 0;
      for (size_t i = 0; i < m_ * N; ++i) {
        dU_norm += dU(i, 0) * dU(i, 0);
      }
      if (dU_norm > 1e-12) {
        Matrix H_dU = H_ * dU;
        double dU_H_dU = 0;
        for (size_t i = 0; i < m_ * N; ++i) {
          dU_H_dU += dU(i, 0) * H_dU(i, 0);
        }
        if (std::abs(dU_H_dU) > 1e-12) {
          alpha = dU_norm / dU_H_dU;
          alpha = std::max(0.001, std::min(1.0, alpha));
        }
      }

      U = U_new;
      sol.iterations = static_cast<int>(iter + 1);
    }

    sol.u_opt = U;

    // Compute predicted states
    sol.x_pred = Matrix::zeros(n_ * (N + 1), 1);
    for (size_t i = 0; i < n_; ++i) {
      sol.x_pred(i, 0) = x0(i, 0);
    }

    Matrix Phi_x0 = Phi_ * x0;
    Matrix Gamma_U = Gamma_ * U;
    for (size_t k = 0; k < N; ++k) {
      for (size_t i = 0; i < n_; ++i) {
        sol.x_pred((k + 1) * n_ + i, 0) =
            Phi_x0(k * n_ + i, 0) + Gamma_U(k * n_ + i, 0);
      }
    }

    // Compute cost
    sol.cost = computeCost(x0, U);

    return sol;
  }

  /**
   * @brief Solve constrained MPC using ADMM
   *
   * Solves box-constrained Quadratic Program:
   *   min  0.5*U'*H*U + g'*U
   *   s.t. u_min ≤ u ≤ u_max
   *
   * ADMM updates:
   *   x-update: x^{k+1} = H_{inv} * (\rho z^k - u^k - g)
   *   z-update: z^{k+1} = clip(x^{k+1} + u^k/\rho, lb, ub)
   *   u-update: u^{k+1} = u^k + \rho(x^{k+1} - z^{k+1})
   */
  MPCSolution solveConstrainedADMM(const Matrix &x0) {
    MPCSolution sol;
    size_t N = config_.horizon;
    size_t U_size = m_ * N;

    // Linear term: g = F * x0
    Matrix g = F_ * x0;

    double rho = config_.rho;

    // ── P3 fix: warm-start ───────────────────────────────────
    // admm_x_, admm_z_, admm_u_ persist across solve() calls.
    // On first call (or after setConfig/reset), initialize to zero.
    if (!config_.warm_start || !admm_initialized_) {
      admm_x_ = Matrix::zeros(U_size, 1);
      admm_z_ = Matrix::zeros(U_size, 1);
      admm_u_ = Matrix::zeros(U_size, 1);
      admm_initialized_ = true;
    }

    bool converged = false;
    size_t iters = 0;

    for (size_t k = 0; k < config_.admm_max_iter; ++k) {
      // ── x-update ─────────────────────────────────────────
      // x = (H + rho*I)^{-1} * (rho*z - u - g)
      // P1 fix: use chol_solve_dynamic instead of H_inv_admm_ * rhs
      Matrix rhs = Matrix::zeros(U_size, 1);
      for (size_t i = 0; i < U_size; ++i)
        rhs(i, 0) = rho * admm_z_(i, 0) - admm_u_(i, 0) - g(i, 0);
      admm_x_ = chol_solve_dynamic(L_admm_, rhs);

      // ── z-update ─────────────────────────────────────────
      // z = clip(x + u/rho, lb, ub)
      Matrix z_prev = admm_z_;
      for (size_t i = 0; i < U_size; ++i) {
        admm_z_(i, 0) = admm_x_(i, 0) + admm_u_(i, 0) / rho;
      }
      projectOntoConstraints(admm_z_);

      // ── u-update ─────────────────────────────────────────
      // u += rho * (x - z)
      for (size_t i = 0; i < U_size; ++i)
        admm_u_(i, 0) += rho * (admm_x_(i, 0) - admm_z_(i, 0));

      // ── Convergence check ─────────────────────────────────
      if ((k + 1) % config_.check_every == 0) {
        double r_norm = 0.0, s_norm = 0.0;
        double x_norm = 0.0, z_norm = 0.0, u_norm = 0.0;
        for (size_t i = 0; i < U_size; ++i) {
          double r_i = admm_x_(i, 0) - admm_z_(i, 0);
          double s_i = rho * (admm_z_(i, 0) - z_prev(i, 0));
          r_norm += r_i * r_i;
          s_norm += s_i * s_i;
          x_norm += admm_x_(i, 0) * admm_x_(i, 0);
          z_norm += admm_z_(i, 0) * admm_z_(i, 0);
          u_norm += admm_u_(i, 0) * admm_u_(i, 0);
        }
        r_norm = std::sqrt(r_norm);
        s_norm = std::sqrt(s_norm);

        double eps_prim = config_.eps_abs * std::sqrt(static_cast<double>(U_size))
                        + config_.eps_rel * std::max(std::sqrt(x_norm),
                                                     std::sqrt(z_norm));
        // W2 fix: eps_dual must include rho factor (OSQP convention)
        double eps_dual = config_.eps_abs * std::sqrt(static_cast<double>(U_size))
                        + config_.eps_rel * rho * std::sqrt(u_norm);

        if (r_norm < eps_prim && s_norm < eps_dual) {
          converged = true;
          iters = k + 1;
          break;
        }
      }
    }

    if (!converged)
      iters = config_.admm_max_iter;

    sol.u_opt = admm_z_;
    sol.iterations = static_cast<int>(iters);
    sol.converged = converged;

    // Compute predicted states
    sol.x_pred = Matrix::zeros(n_ * (N + 1), 1);
    for (size_t i = 0; i < n_; ++i)
      sol.x_pred(i, 0) = x0(i, 0);

    Matrix Phi_x0 = Phi_ * x0;
    Matrix Gamma_U = Gamma_ * sol.u_opt;
    for (size_t k = 0; k < N; ++k)
      for (size_t i = 0; i < n_; ++i)
        sol.x_pred((k + 1) * n_ + i, 0) =
            Phi_x0(k * n_ + i, 0) + Gamma_U(k * n_ + i, 0);

    sol.cost = computeCost(x0, sol.u_opt);
    return sol;
  }

  /**
   * @brief Project control sequence onto box constraints
   */
  void projectOntoConstraints(Matrix &U) {
    size_t N = config_.horizon;

    for (size_t k = 0; k < N; ++k) {
      for (size_t i = 0; i < m_; ++i) {
        double &u = U(k * m_ + i, 0);

        if (!config_.u_min.empty() && i < config_.u_min.size()) {
          u = std::max(u, config_.u_min[i]);
        }
        if (!config_.u_max.empty() && i < config_.u_max.size()) {
          u = std::min(u, config_.u_max[i]);
        }
      }
    }
  }

  /**
   * @brief Compute MPC cost for given control sequence
   */
  double computeCost(const Matrix &x0, const Matrix &U) {
    size_t N = config_.horizon;

    Matrix Phi_x0 = Phi_ * x0;
    Matrix Gamma_U = Gamma_ * U;
    Matrix X = Phi_x0 + Gamma_U;

    double cost = 0;
    for (size_t k = 0; k < N; ++k) {
      const Matrix &Qk = (k == N - 1 && config_.P_terminal.rows > 0)
                             ? config_.P_terminal
                             : config_.Q;

      for (size_t i = 0; i < n_; ++i) {
        for (size_t j = 0; j < n_; ++j) {
          cost += X(k * n_ + i, 0) * Qk(i, j) * X(k * n_ + j, 0);
        }
      }

      for (size_t i = 0; i < m_; ++i) {
        for (size_t j = 0; j < m_; ++j) {
          cost += U(k * m_ + i, 0) * config_.R(i, j) * U(k * m_ + j, 0);
        }
      }
    }

    return cost;
  }
};

// ============================================================
//                    UTILITY FUNCTIONS
// ============================================================

/**
 * @brief Create MPC controller from continuous-time system
 *
 * @param A Continuous A matrix
 * @param B Continuous B matrix
 * @param C Output matrix
 * @param dt Sampling time
 * @param config MPC configuration
 * @return MPC controller with discretized system
 */
inline MPCController mpcFromContinuous(const Matrix &A, const Matrix &B,
                                       const Matrix &C, double dt,
                                       const MPCConfig &config) {
  // Simple Euler discretization
  // A_d ≈ I + A*dt
  // B_d ≈ B*dt

  size_t n = A.rows;
  Matrix Ad = Matrix::eye(n);
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < n; ++j) {
      Ad(i, j) += A(i, j) * dt;
    }
  }

  Matrix Bd = B;
  for (size_t i = 0; i < B.rows; ++i) {
    for (size_t j = 0; j < B.cols; ++j) {
      Bd(i, j) *= dt;
    }
  }

  return MPCController(Ad, Bd, C, config);
}

/**
 * @brief Simulate closed-loop MPC system
 *
 * @param mpc MPC controller
 * @param A System A matrix
 * @param B System B matrix
 * @param x0 Initial state
 * @param num_steps Number of simulation steps
 * @return Pair of (state trajectory, control trajectory)
 */
inline std::pair<std::vector<Matrix>, std::vector<Matrix>>
simulateMPC(MPCController &mpc, const Matrix &A, const Matrix &B,
            const Matrix &x0, size_t num_steps) {

  std::vector<Matrix> x_traj, u_traj;
  Matrix x = x0;
  x_traj.push_back(x);

  for (size_t k = 0; k < num_steps; ++k) {
    // Solve MPC
    auto sol = mpc.solve(x);

    // Extract first control
    Matrix u = sol.getFirstControl(B.cols);
    u_traj.push_back(u);

    // Apply control
    x = A * x + B * u;
    x_traj.push_back(x);
  }

  return {x_traj, u_traj};
}

/**
 * @brief Compare MPC with LQR for regulation problem
 */
struct MPCvsLQRResult {
  double mpc_cost;
  double lqr_cost;
  std::vector<Matrix> mpc_states;
  std::vector<Matrix> lqr_states;
  std::vector<Matrix> mpc_controls;
  std::vector<Matrix> lqr_controls;
};

inline MPCvsLQRResult compareMPCwithLQR(const Matrix &A, const Matrix &B,
                                        const Matrix &C, const Matrix &Q,
                                        const Matrix &R, const Matrix &x0,
                                        size_t num_steps,
                                        size_t mpc_horizon = 10) {
  MPCvsLQRResult result;

  // Setup MPC
  MPCConfig config(mpc_horizon, Q, R);
  MPCController mpc(A, B, C, config);

  // Setup discrete LQR (DLQR)
  Matrix K = dlqr(A, B, Q, R);

  // Simulate MPC
  Matrix x = x0;
  result.mpc_states.push_back(x);
  result.mpc_cost = 0;

  for (size_t k = 0; k < num_steps; ++k) {
    auto sol = mpc.solve(x);
    Matrix u = sol.getFirstControl(B.cols);
    result.mpc_controls.push_back(u);

    // Cost
    for (size_t i = 0; i < Q.rows; ++i) {
      for (size_t j = 0; j < Q.cols; ++j) {
        result.mpc_cost += x(i, 0) * Q(i, j) * x(j, 0);
      }
    }
    for (size_t i = 0; i < R.rows; ++i) {
      for (size_t j = 0; j < R.cols; ++j) {
        result.mpc_cost += u(i, 0) * R(i, j) * u(j, 0);
      }
    }

    x = A * x + B * u;
    result.mpc_states.push_back(x);
  }

  // Simulate LQR
  x = x0;
  result.lqr_states.push_back(x);
  result.lqr_cost = 0;

  for (size_t k = 0; k < num_steps; ++k) {
    // u = -K*x
    Matrix u = K * x;
    for (size_t i = 0; i < u.rows; ++i) {
      u(i, 0) = -u(i, 0);
    }
    result.lqr_controls.push_back(u);

    // Cost
    for (size_t i = 0; i < Q.rows; ++i) {
      for (size_t j = 0; j < Q.cols; ++j) {
        result.lqr_cost += x(i, 0) * Q(i, j) * x(j, 0);
      }
    }
    for (size_t i = 0; i < R.rows; ++i) {
      for (size_t j = 0; j < R.cols; ++j) {
        result.lqr_cost += u(i, 0) * R(i, j) * u(j, 0);
      }
    }

    x = A * x + B * u;
    result.lqr_states.push_back(x);
  }

  return result;
}

} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_MPC_HPP
