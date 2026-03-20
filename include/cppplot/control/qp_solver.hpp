/**
 * @file qp_solver.hpp
 * @brief ADMM-based QP Solver — Header-Only, Zero Dynamic Memory
 *
 * Solves box-constrained Quadratic Program:
 *
 *   min   0.5 * x' * P * x + q' * x
 *   s.t.  lb <= x <= ub
 *
 * Algorithm: ADMM (Alternating Direction Method of Multipliers)
 *   - Pre-factorizes (P + rho*I) via Cholesky once at setup
 *   - Each solve: O(n^2) chol_solve + O(n) clip projection
 *   - Fixed max_iter cap → bounded WCET for real-time use
 *   - Warm-start from previous solution (closed-loop efficiency)
 *
 * Design goals (DCAS Lab — L0 Core Control Stack):
 *   - Zero heap allocation (stack-only via fixed-size templates)
 *   - C++14 header-only, no dependencies beyond <cmath>
 *   - Target: WCET <= 10ms @ ARM Cortex-A (Allwinner T113-S3)
 *   - Compatible with cppplot::control::Matrix (dynamic wrapper)
 *
 * References:
 *   [1] Stellato et al. "OSQP: An Operator Splitting Solver for QPs"
 *       IEEE Transactions on Automatic Control, 2020.
 *   [2] Amos & Kolter. "OptNet: Differentiable Optimization as a
 *       Layer in Neural Networks." ICML 2017.
 *   [3] Convex Optimization, Boyd & Vandenberghe, Ch. 5 (ADMM).
 *
 * @author DCAS Lab, TDTU — TS. Trí Viễn Vũ
 * @date March 2026
 */

#ifndef CPPPLOT_CONTROL_QP_SOLVER_HPP
#define CPPPLOT_CONTROL_QP_SOLVER_HPP

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace cppplot {
namespace control {
namespace qp {

// ============================================================
//                  FIXED-SIZE LINEAR ALGEBRA
// ============================================================

/**
 * @brief Fixed-size vector — stack allocated, zero heap
 */
template <int N> struct Vec {
  double data[N];

  Vec() { std::fill(data, data + N, 0.0); }

  double &operator[](int i) { return data[i]; }
  const double &operator[](int i) const { return data[i]; }

  double norm2() const {
    double s = 0.0;
    for (int i = 0; i < N; ++i)
      s += data[i] * data[i];
    return std::sqrt(s);
  }

  double dot(const Vec<N> &b) const {
    double s = 0.0;
    for (int i = 0; i < N; ++i)
      s += data[i] * b[i];
    return s;
  }

  Vec<N> operator+(const Vec<N> &b) const {
    Vec<N> r;
    for (int i = 0; i < N; ++i)
      r[i] = data[i] + b[i];
    return r;
  }

  Vec<N> operator-(const Vec<N> &b) const {
    Vec<N> r;
    for (int i = 0; i < N; ++i)
      r[i] = data[i] - b[i];
    return r;
  }

  Vec<N> operator*(double s) const {
    Vec<N> r;
    for (int i = 0; i < N; ++i)
      r[i] = data[i] * s;
    return r;
  }
};

/**
 * @brief Fixed-size square matrix — row-major, stack allocated
 */
template <int N> struct Mat {
  double data[N][N];

  Mat() { std::memset(data, 0, sizeof(data)); }

  double &operator()(int r, int c) { return data[r][c]; }
  const double &operator()(int r, int c) const { return data[r][c]; }

  /** Matrix-vector multiply: y = A * x */
  Vec<N> matvec(const Vec<N> &x) const {
    Vec<N> y;
    for (int i = 0; i < N; ++i)
      for (int j = 0; j < N; ++j)
        y[i] += data[i][j] * x[j];
    return y;
  }
};

// ============================================================
//                    CHOLESKY FACTORIZATION
// ============================================================

/**
 * @brief In-place Cholesky decomposition: A -> L  (A = L * L')
 *
 * Input  A: symmetric positive definite matrix
 * Output L: lower triangular factor (stored in-place, upper ignored)
 *
 * @return true if successful, false if A is not positive definite
 */
template <int N> bool cholesky(Mat<N> &L, const Mat<N> &A) {
  // Copy A into L first
  for (int i = 0; i < N; ++i)
    for (int j = 0; j < N; ++j)
      L(i, j) = A(i, j);

  for (int j = 0; j < N; ++j) {
    // Diagonal element
    double s = L(j, j);
    for (int k = 0; k < j; ++k)
      s -= L(j, k) * L(j, k);
    if (s < 1e-14)
      return false; // Not positive definite
    L(j, j) = std::sqrt(s);

    // Column below diagonal
    for (int i = j + 1; i < N; ++i) {
      double t = L(i, j);
      for (int k = 0; k < j; ++k)
        t -= L(i, k) * L(j, k);
      L(i, j) = t / L(j, j);
    }
  }
  return true;
}

/**
 * @brief Solve L * L' * x = b using stored Cholesky factor L
 *
 * Two triangular solves: forward (L*y=b) then backward (L'*x=y)
 * Complexity: O(N^2)
 */
template <int N> Vec<N> chol_solve(const Mat<N> &L, const Vec<N> &b) {
  Vec<N> y, x;

  // Forward substitution: L * y = b
  for (int i = 0; i < N; ++i) {
    double s = b[i];
    for (int k = 0; k < i; ++k)
      s -= L(i, k) * y[k];
    y[i] = s / L(i, i);
  }

  // Backward substitution: L' * x = y
  for (int i = N - 1; i >= 0; --i) {
    double s = y[i];
    for (int k = i + 1; k < N; ++k)
      s -= L(k, i) * x[k];
    x[i] = s / L(i, i);
  }

  return x;
}

// ============================================================
//                   ADMM QP SOLVER
// ============================================================

/**
 * @brief ADMM solver parameters
 */
struct Params {
  double rho = 1.0;       ///< ADMM step size (penalty parameter)
  double eps_abs = 1e-4;  ///< Absolute convergence tolerance
  double eps_rel = 1e-3;  ///< Relative convergence tolerance
  int max_iter = 100;     ///< Hard iteration cap (WCET bound)
  bool warm_start = true; ///< Reuse previous x, z, u variables
  int check_every = 5;    ///< Check convergence every N iterations
};

/**
 * @brief QP Solution
 */
template <int N> struct Solution {
  Vec<N> x;          ///< Primal solution (feasible: lb <= x <= ub)
  double cost;       ///< Optimal objective value
  int iterations;    ///< Actual iterations used
  bool converged;    ///< True if tolerance met before max_iter
  double primal_res; ///< Final primal residual ||x - z||
  double dual_res;   ///< Final dual residual rho*||z_k - z_{k-1}||

  Solution()
      : cost(0.0), iterations(0), converged(false), primal_res(0.0),
        dual_res(0.0) {}
};

/**
 * @brief ADMM-based solver for box-constrained QP
 *
 * Problem:
 *   min   0.5 * x' * P * x + q' * x
 *   s.t.  lb <= x <= ub
 *
 * ADMM splitting (x = z, lb <= z <= ub) — UNSCALED dual (u_ stores λ):
 *   x-update: x^{k+1} = (P + rho*I)^{-1} * (rho*z^k - u^k - q)
 *   z-update: z^{k+1} = clip(x^{k+1} + u^k/rho, lb, ub)   [u^k = λ^k]
 *   u-update: u^{k+1} = u^k + rho * (x^{k+1} - z^{k+1})   [λ update]
 *
 * Note: u_ stores the UNSCALED dual variable λ (Lagrange multiplier).
 * Some ADMM references use scaled dual s = λ/ρ where z-update = clip(x+s).
 * Here: z-update = clip(x + u_/rho) — equivalent, u_ = λ.
 *
 * Key: (P + rho*I) is factorized ONCE in setup() and reused.
 *
 * @tparam N  Number of decision variables (compile-time)
 */
template <int N> class ADMMSolver {
public:
  /**
   * @brief Setup solver: factorize (P + rho*I)
   *
   * Call once when P changes (e.g., in MPCController constructor).
   * For MPC, P = H = pre-computed Hessian (never changes).
   *
   * @param P  Symmetric PSD Hessian matrix
   * @param p  Solver parameters (rho used here)
   * @return true if factorization succeeded
   */
  bool setup(const Mat<N> &P, const Params &p = Params()) {
    params_ = p;
    P_ = P; // Store P for cost calculation

    // Regularize: M = P + rho * I
    Mat<N> M = P;
    for (int i = 0; i < N; ++i)
      M(i, i) += params_.rho;

    // Cholesky factorize
    bool ok = cholesky(L_, M);
    if (!ok) {
      // Increase regularization and retry
      for (int i = 0; i < N; ++i)
        M(i, i) += 1e-6;
      ok = cholesky(L_, M);
    }

    setup_done_ = ok;

    // Reset warm-start variables
    x_ = Vec<N>();
    z_ = Vec<N>();
    u_ = Vec<N>();

    return ok;
  }

  /**
   * @brief Solve QP for given linear term q and bounds
   *
   * This is the hot path — called every control step (100Hz).
   * Only chol_solve + clip per iteration, no memory allocation.
   *
   * @param q   Linear term (changes every solve, q = F*x0 in MPC)
   * @param lb  Lower bounds
   * @param ub  Upper bounds
   * @return    Solution struct
   */
  Solution<N> solve(const Vec<N> &q, const Vec<N> &lb, const Vec<N> &ub) {
    Solution<N> sol;

    if (!setup_done_)
      return sol;

    // Initialize warm variables if not warm-starting
    if (!params_.warm_start) {
      x_ = Vec<N>();
      z_ = Vec<N>();
      u_ = Vec<N>();
    }

    double rho = params_.rho;

    for (int k = 0; k < params_.max_iter; ++k) {
      // ── x-update ────────────────────────────────────────────
      // x = (P + rho*I)^{-1} * (rho*z - u - q)
      Vec<N> rhs;
      for (int i = 0; i < N; ++i)
        rhs[i] = rho * z_[i] - u_[i] - q[i];
      x_ = chol_solve(L_, rhs);

      // ── z-update ────────────────────────────────────────────
      // z = clip(x + u/rho, lb, ub)
      Vec<N> z_prev = z_;
      for (int i = 0; i < N; ++i) {
        double v = x_[i] + u_[i] / rho;
        z_[i] = std::max(lb[i], std::min(ub[i], v));
      }

      // ── u-update ────────────────────────────────────────────
      // u += rho * (x - z)
      for (int i = 0; i < N; ++i)
        u_[i] += rho * (x_[i] - z_[i]);

      // ── Convergence check (every check_every iterations) ────
      if ((k + 1) % params_.check_every == 0) {
        Vec<N> r = x_ - z_;     // primal residual
        Vec<N> d = z_ - z_prev; // dual direction
        sol.primal_res = r.norm2();
        sol.dual_res = rho * d.norm2();

        double eps_prim = params_.eps_abs * std::sqrt(N) +
                          params_.eps_rel * std::max(x_.norm2(), z_.norm2());
        double eps_dual = params_.eps_abs * std::sqrt(N) +
                          params_.eps_rel * (rho * u_.norm2());

        if (sol.primal_res < eps_prim && sol.dual_res < eps_dual) {
          sol.converged = true;
          sol.iterations = k + 1;
          break;
        }
      }
    }

    if (!sol.converged)
      sol.iterations = params_.max_iter;

    // Return z (feasible) as solution
    sol.x = z_;

    // Compute objective value: 0.5 * z' * P * z + q' * z
    sol.cost = 0.0;
    Vec<N> Pz = P_.matvec(z_);
    for (int i = 0; i < N; ++i) {
      sol.cost += 0.5 * Pz[i] * z_[i] + q[i] * z_[i];
    }

    return sol;
  }

  /**
   * @brief Update rho and re-factorize (adaptive rho strategy)
   *
   * Called when primal/dual residual ratio is too large.
   * Heuristic from OSQP: if ||r_prim|| > mu * ||r_dual||, increase rho.
   */
  bool update_rho(const Mat<N> &P, double new_rho) {
    params_.rho = new_rho;
    return setup(P, params_);
  }

  /** @brief Reset warm-start variables to zero */
  void reset() {
    x_ = Vec<N>();
    z_ = Vec<N>();
    u_ = Vec<N>();
  }

  /** @brief Get current parameters */
  const Params &get_params() const { return params_; }

private:
  Mat<N> P_;         ///< Original Hessian matrix P
  Mat<N> L_;         ///< Cholesky factor of (P + rho*I)
  Vec<N> x_, z_, u_; ///< ADMM primal/dual variables (warm-start)
  Params params_;
  bool setup_done_ = false;
};

// ============================================================
//        BRIDGE: cppplot::Matrix  <->  qp::Vec / Mat
//
//  Excluded when CPPPLOT_QP_STANDALONE is defined.
//  Define this macro when using qp_solver.hpp in embedded
//  targets that do not have the full cppplot Matrix class.
//
//  mpc_embedded.hpp defines CPPPLOT_QP_STANDALONE automatically
//  before including qp_solver.hpp.
// ============================================================

#ifndef CPPPLOT_QP_STANDALONE

/**
 * @brief Copy cppplot::Matrix (dynamic) into fixed-size Mat<N>
 *
 * Use this in MPCController::computePredictionMatrices()
 * to transfer H (n*N x n*N) into the fixed-size solver.
 */
template <int N> Mat<N> to_fixed_mat(const Matrix &M) {
  Mat<N> out;
  for (int i = 0; i < N && i < static_cast<int>(M.rows); ++i)
    for (int j = 0; j < N && j < static_cast<int>(M.cols); ++j)
      out(i, j) = M(i, j);
  return out;
}

/**
 * @brief Copy cppplot::Matrix column vector into Vec<N>
 */
template <int N> Vec<N> to_fixed_vec(const Matrix &v) {
  Vec<N> out;
  for (int i = 0; i < N && i < static_cast<int>(v.rows); ++i)
    out[i] = v(i, 0);
  return out;
}

/**
 * @brief Fill Vec<N> from std::vector<double> (for bounds)
 *
 * Repeats the pattern across N elements (e.g., u_min repeated N times)
 */
template <int N>
Vec<N> to_fixed_vec(const std::vector<double> &v, int m, int horizon,
                    double default_val) {
  Vec<N> out;
  for (int i = 0; i < N; ++i)
    out[i] = default_val;
  for (int k = 0; k < horizon; ++k)
    for (int j = 0; j < m && k * m + j < N; ++j)
      out[k * m + j] = (j < static_cast<int>(v.size())) ? v[j] : default_val;
  return out;
}

/**
 * @brief Copy Vec<N> back into cppplot::Matrix
 */
template <int N> Matrix to_matrix(const Vec<N> &v) {
  Matrix out = Matrix::zeros(N, 1);
  for (int i = 0; i < N; ++i)
    out(i, 0) = v[i];
  return out;
}

#endif // CPPPLOT_QP_STANDALONE

// ============================================================
//        BENCHMARK UTILITIES
// ============================================================

/**
 * @brief Timing structure for WCET analysis
 */
struct SolveStats {
  double mean_us;   ///< Mean solve time (microseconds)
  double max_us;    ///< Worst-case solve time
  double min_us;    ///< Best-case solve time
  double std_us;    ///< Standard deviation
  double mean_iter; ///< Mean iterations to convergence
  int n_failed;     ///< Number of non-converged solves
};

} // namespace qp
} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_QP_SOLVER_HPP
