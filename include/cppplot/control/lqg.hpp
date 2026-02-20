/**
 * @file lqg.hpp
 * @brief Linear Quadratic Gaussian (LQG) Controller
 *
 * LQG combines:
 * - LQR (Linear Quadratic Regulator) for optimal state feedback
 * - Kalman Filter for optimal state estimation
 *
 * This creates a separation principle: design observer and controller
 * independently.
 *
 * @author CppPlot Control Systems Library
 * @date January 2026
 *
 * References:
 * [1] Anderson & Moore - "Optimal Control: Linear Quadratic Methods"
 * [2] Doyle (1978) - "Guaranteed Margins for LQG Regulators"
 *
 * @note LQG does NOT guarantee robustness margins!
 *       The famous Doyle paper showed LQG can have arbitrarily small margins.
 *       For robust control, consider LQG/LTR or H∞ methods.
 */

#ifndef CPPPLOT_CONTROL_LQG_HPP
#define CPPPLOT_CONTROL_LQG_HPP

#include "controller_design.hpp"
#include "kalman.hpp"
#include "state_space.hpp"


namespace cppplot {
namespace control {

/**
 * @brief LQG Controller result structure
 */
struct LQGResult {
  Matrix K; // State feedback gains (1×n row matrix)
  Matrix L; // Kalman filter gains
  Matrix P; // Riccati solution (controller)
  Matrix S; // Riccati solution (estimator)

  // Closed-loop system matrices
  Matrix Acl;            // Closed-loop A matrix
  StateSpace closedLoop; // Full closed-loop system
};

/**
 * @brief Design LQG controller
 *
 * The LQG problem:
 *
 * Plant:
 *   ẋ = Ax + Bu + Gw      (w = process noise, w ~ N(0, W))
 *   y = Cx + v            (v = measurement noise, v ~ N(0, V))
 *
 * Cost:
 *   J = E[ ∫(x'Qx + u'Ru)dt ]
 *
 * Solution (Separation Principle):
 *   1. Design LQR: K = R^(-1)B'P where P solves ARE
 *   2. Design Kalman: L = SC'V^(-1) where S solves dual ARE
 *   3. Controller: u = -K*x̂ where x̂ is Kalman estimate
 *
 * @param A System matrix (n x n)
 * @param B Input matrix (n x m)
 * @param C Output matrix (p x n)
 * @param Q State weighting (n x n)
 * @param R Input weighting (m x m)
 * @param W Process noise covariance (n x n) or intensity matrix G*Qn*G'
 * @param V Measurement noise covariance (p x p)
 * @return LQG controller gains and closed-loop system
 *
 * @example
 * ```cpp
 * // Double integrator with position measurement
 * Matrix A = {{0, 1}, {0, 0}};
 * Matrix B = {{0}, {1}};
 * Matrix C = {{1, 0}};
 *
 * // LQR weights
 * Matrix Q = Matrix::eye(2);
 * Matrix R = Matrix::eye(1) * 0.1;
 *
 * // Noise covariances
 * Matrix W = Matrix::eye(2) * 0.01;  // Process noise
 * Matrix V = Matrix::eye(1) * 0.1;   // Measurement noise
 *
 * auto lqg = designLQG(A, B, C, Q, R, W, V);
 * std::cout << "LQR gains: K = " << lqg.K << std::endl;
 * std::cout << "Kalman gains: L = " << lqg.L << std::endl;
 * ```
 */
inline LQGResult designLQG(const Matrix &A, const Matrix &B, const Matrix &C,
                           const Matrix &Q, const Matrix &R, const Matrix &W,
                           const Matrix &V) {
  LQGResult result;
  size_t n = A.rows;
  size_t m = B.cols;
  size_t p = C.rows;

  // Step 1: Design LQR controller
  // Solve: A'P + PA - PBR^(-1)B'P + Q = 0
  result.P = care(A, B, Q, R);
  result.K = lqr(A, B, Q, R);

  // Step 2: Design Kalman filter (dual problem)
  // Solve: AS + SA' - SC'V^(-1)CS + W = 0
  // This is equivalent to: A'S + SA - SC'V^(-1)CS + W = 0 for A' -> A
  // Actually the dual Riccati: AS + SA' - SC'V^(-1)CS + W = 0
  // We can solve it using care with transposed system:
  // care(A', C', W, V) gives the solution

  Matrix AT = A.T();
  Matrix CT = C.T();
  result.S = care(AT, CT, W, V);

  // Kalman gain: L = S*C'*V^(-1)
  Matrix V_inv = V.inv();
  result.L = result.S * CT * V_inv;

  // Step 3: Form closed-loop system
  // State: [x; x̂] where x̂ = estimated state
  //
  // [ẋ ]   [A       -BK  ] [x ]   [I] w   [0] v
  // [x̂̇] = [LC  A-BK-LC] [x̂] + [0]   + [L]
  //
  // Or in terms of error e = x - x̂:
  // [ẋ]   [A    -BK ] [x]   [I] w   [0] v
  // [ė] = [0   A-LC] [e] + [I]   + [-L]
  //
  // Closed-loop eigenvalues: eig(A-BK) ∪ eig(A-LC)

  // Build K matrix for state feedback (result.K is already a 1×n Matrix)
  Matrix BK = B * result.K;

  // Closed-loop A matrix (2n x 2n)
  result.Acl = Matrix::zeros(2 * n, 2 * n);

  // Top-left: A
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < n; ++j) {
      result.Acl(i, j) = A(i, j);
    }
  }

  // Top-right: -B*K
  // BK already computed above
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < n; ++j) {
      result.Acl(i, n + j) = -BK(i, j);
    }
  }

  // Bottom-left: L*C
  Matrix LC = result.L * C;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < n; ++j) {
      result.Acl(n + i, j) = LC(i, j);
    }
  }

  // Bottom-right: A - B*K - L*C
  Matrix ABKLC = A - BK - LC;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < n; ++j) {
      result.Acl(n + i, n + j) = ABKLC(i, j);
    }
  }

  return result;
}

/**
 * @brief LQG Controller class for simulation
 *
 * Implements the LQG controller as a dynamic system that can be simulated.
 */
class LQGController {
private:
  Matrix A_, B_, C_;
  Matrix K_;
  KalmanFilter *kf_;
  size_t n_, m_, p_;

public:
  /**
   * @brief Construct LQG controller
   */
  LQGController(const Matrix &A, const Matrix &B, const Matrix &C,
                const Matrix &Q, const Matrix &R, const Matrix &W,
                const Matrix &V)
      : A_(A), B_(B), C_(C) {
    n_ = A.rows;
    m_ = B.cols;
    p_ = C.rows;

    // Design LQR
    K_ = lqr(A, B, Q, R);

    // Create Kalman filter
    kf_ = new KalmanFilter(A, B, C, W, V);
  }

  ~LQGController() { delete kf_; }

  /**
   * @brief Compute control input given measurement
   *
   * @param y Measurement
   * @param u_prev Previous control input (for Kalman prediction)
   * @return Control input u = -K*x̂
   */
  Matrix computeControl(const Matrix &y, const Matrix &u_prev) {
    // Update Kalman filter
    auto est = kf_->update(y, u_prev);

    // Compute control: u = -K*x̂
    Matrix u = K_ * est.x_hat;
    for (size_t i = 0; i < u.rows; ++i) {
      u(i, 0) = -u(i, 0);
    }

    return u;
  }

  /**
   * @brief Get state estimate
   */
  Matrix getStateEstimate() const { return kf_->getState(); }

  /**
   * @brief Set initial state estimate
   */
  void setInitialEstimate(const Matrix &x0, const Matrix &P0) {
    kf_->setInitialState(x0, P0);
  }

  /**
   * @brief Get LQR gains
   */
  Matrix getGains() const { return K_; }

  /**
   * @brief Reset controller
   */
  void reset() { kf_->reset(); }
};

/**
 * @brief LQG with Loop Transfer Recovery (LQG/LTR)
 *
 * LQG/LTR recovers the robustness properties of LQR by:
 * 1. At plant input: increase process noise (make Kalman faster)
 * 2. At plant output: increase state weight (make LQR faster)
 *
 * As q → ∞ (or ρ → 0), the loop transfer function approaches
 * the target loop transfer (LQR at input, Kalman at output).
 *
 * @param A System matrix
 * @param B Input matrix
 * @param C Output matrix
 * @param Q Base state weight
 * @param R Input weight
 * @param W Base process noise
 * @param V Measurement noise
 * @param rho Recovery parameter (smaller = better recovery, but faster
 * observer)
 * @return LQG result with recovered robustness
 *
 * @note For input LTR: W_new = W + (1/ρ)*B*B'
 * @note For output LTR: Q_new = Q + (1/ρ)*C'*C
 */
inline LQGResult designLQG_LTR(const Matrix &A, const Matrix &B,
                               const Matrix &C, const Matrix &Q,
                               const Matrix &R, const Matrix &W,
                               const Matrix &V, double rho = 0.01,
                               bool input_recovery = true) {
  size_t n = A.rows;

  if (input_recovery) {
    // Input LTR: Increase process noise to speed up observer
    // This recovers LQR robustness at plant input
    Matrix BT = B.T();
    Matrix BBT = B * BT;

    Matrix W_ltr = W;
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < n; ++j) {
        W_ltr(i, j) += BBT(i, j) / rho;
      }
    }

    return designLQG(A, B, C, Q, R, W_ltr, V);
  } else {
    // Output LTR: Increase state weight to speed up controller
    // This recovers Kalman robustness at plant output
    Matrix CT = C.T();
    Matrix CTC = CT * C;

    Matrix Q_ltr = Q;
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < n; ++j) {
        Q_ltr(i, j) += CTC(i, j) / rho;
      }
    }

    return designLQG(A, B, C, Q_ltr, R, W, V);
  }
}

/**
 * @brief Compute loop transfer function for LQG analysis
 *
 * For robustness analysis, we need to examine:
 * - Loop transfer at plant input: L_u = K(sI-A+BK+LC)^(-1)L C (sI-A)^(-1) B
 * - Loop transfer at plant output: L_y = C(sI-A)^(-1) B K(sI-A+BK+LC)^(-1)L
 *
 * @param A System matrix
 * @param B Input matrix
 * @param C Output matrix
 * @param K LQR gain
 * @param L Kalman gain
 * @return StateSpace representation of compensator K(sI-A+BK+LC)^(-1)L
 */
inline StateSpace lqgCompensator(const Matrix &A, const Matrix &B,
                                 const Matrix &C, const Matrix &K,
                                 const Matrix &L) {
  size_t n = A.rows;
  size_t m = B.cols;
  size_t p = C.rows;

  // Compensator: K(sI - A + BK + LC)^(-1) L
  // State equation: x̂̇ = (A - BK - LC)x̂ + Ly
  // Output equation: u = -K*x̂

  Matrix Ac = A - B * K - L * C;
  Matrix Bc = L;

  // Output is -K*x̂
  Matrix Cc(m, n);
  for (size_t i = 0; i < m; ++i) {
    for (size_t j = 0; j < n; ++j) {
      Cc(i, j) = -K(0, j); // Only works for SISO
    }
  }

  Matrix Dc = Matrix::zeros(m, p);

  return StateSpace(Ac, Bc, Cc, Dc);
}

// ============================================================
//                    DISCRETE-TIME LQG
// ============================================================

/**
 * @brief Design discrete-time LQG controller
 *
 * For discrete-time system:
 *   x(k+1) = A*x(k) + B*u(k) + w(k)
 *   y(k)   = C*x(k) + v(k)
 *
 * Cost: J = E[ Σ(x'Qx + u'Ru) ]
 */
inline LQGResult designLQG_discrete(const Matrix &A, const Matrix &B,
                                    const Matrix &C, const Matrix &Q,
                                    const Matrix &R, const Matrix &W,
                                    const Matrix &V) {
  // For discrete-time, we solve DARE instead of CARE
  // The implementation is similar but uses discrete Riccati equations

  // For now, use continuous-time approximation
  // TODO: Implement proper discrete Riccati solver
  return designLQG(A, B, C, Q, R, W, V);
}

} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_LQG_HPP
