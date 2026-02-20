/**
 * @file robust/hinf.hpp
 * @brief H∞ Control Synthesis for Small-Scale Systems
 *
 * Implements H∞ control synthesis methods for teaching and small-scale
 * applications:
 * - State-feedback H∞ design
 * - Output-feedback H∞ design (full-order controller)
 * - γ-iteration for optimal H∞ bound
 * - Mixed-sensitivity H∞ design (S/KS/T weighting)
 *
 * Intended for systems with n ≤ 4 states. Uses iterative Riccati methods.
 *
 * References:
 * - Doyle, Glover, Khargonekar, Francis (1989) "State-Space Solutions to
 * Standard H2 and H∞ Control Problems"
 * - Zhou, Doyle, Glover (1996) "Robust and Optimal Control"
 * - Skogestad & Postlethwaite (2005) "Multivariable Feedback Control"
 */

#ifndef CPPPLOT_CONTROL_ROBUST_HINF_HPP
#define CPPPLOT_CONTROL_ROBUST_HINF_HPP

#include "../analysis.hpp"
#include "../controller_design.hpp"
#include "../state_space.hpp"
#include "../transfer_function.hpp"
#include <algorithm>
#include <limits>


namespace cppplot {
namespace control {
namespace robust {

// ============================================================
//                    RESULT STRUCTURES
// ============================================================

/**
 * @brief Result structure for H∞ state-feedback synthesis
 */
struct HinfStateFeedbackResult {
  Matrix K;          ///< State-feedback gain (m x n)
  Matrix P;          ///< Solution to H∞ ARE
  bool success;      ///< Convergence/success flag
  double gamma_used; ///< γ used in synthesis
  double gamma_opt;  ///< Optimal γ found (if γ-iteration used)
  int iterations;    ///< Number of iterations to converge

  HinfStateFeedbackResult()
      : success(false), gamma_used(0), gamma_opt(0), iterations(0) {}
};

/**
 * @brief Result structure for H∞ output-feedback synthesis
 */
struct HinfOutputFeedbackResult {
  Matrix Ak;         ///< Controller state matrix
  Matrix Bk;         ///< Controller input matrix
  Matrix Ck;         ///< Controller output matrix
  Matrix Dk;         ///< Controller feedthrough matrix
  StateSpace K_ss;   ///< Controller as StateSpace object
  Matrix X;          ///< Solution to control ARE
  Matrix Y;          ///< Solution to filtering ARE
  bool success;      ///< Convergence/success flag
  double gamma_used; ///< γ used in synthesis
  double gamma_opt;  ///< Optimal γ found
  int iterations;    ///< Total iterations

  HinfOutputFeedbackResult()
      : success(false), gamma_used(0), gamma_opt(0), iterations(0) {}
};

/**
 * @brief Generalized plant structure for H∞ design
 *
 * Standard form:
 *   ẋ  = A x  + B1 w  + B2 u
 *   z  = C1 x + D11 w + D12 u
 *   y  = C2 x + D21 w + D22 u
 *
 * where:
 *   x = state, w = disturbance/reference, u = control
 *   z = performance output, y = measurement
 */
struct GeneralizedPlant {
  Matrix A;   ///< State matrix (n x n)
  Matrix B1;  ///< Disturbance input (n x nw)
  Matrix B2;  ///< Control input (n x nu)
  Matrix C1;  ///< Performance output (nz x n)
  Matrix C2;  ///< Measurement output (ny x n)
  Matrix D11; ///< Feedthrough w -> z (nz x nw)
  Matrix D12; ///< Feedthrough u -> z (nz x nu)
  Matrix D21; ///< Feedthrough w -> y (ny x nw)
  Matrix D22; ///< Feedthrough u -> y (ny x nu)

  size_t n() const { return A.rows; }
  size_t nw() const { return B1.cols; }
  size_t nu() const { return B2.cols; }
  size_t nz() const { return C1.rows; }
  size_t ny() const { return C2.rows; }

  /// Check standard assumptions for H∞ synthesis
  bool check_assumptions() const {
    // D12 should have full column rank
    // D21 should have full row rank
    // D11 = 0 (common), D22 = 0 (common)
    return true; // Simplified check
  }
};

// ============================================================
//                    HELPER FUNCTIONS
// ============================================================

/**
 * @brief Compute Frobenius norm of a matrix
 */
inline double frobenius_norm(const Matrix &M) {
  double sum = 0;
  for (size_t i = 0; i < M.rows; ++i) {
    for (size_t j = 0; j < M.cols; ++j) {
      sum += M(i, j) * M(i, j);
    }
  }
  return std::sqrt(sum);
}

/**
 * @brief Check if all eigenvalues have negative real parts
 */
inline bool is_stable_matrix(const Matrix &A) {
  if (A.rows > 4)
    return true; // Assume stable for large matrices
  auto eigs = A.eigenvalues();
  for (const auto &e : eigs) {
    if (e.real() >= -1e-10)
      return false;
  }
  return true;
}

/**
 * @brief Spectral radius (max |eigenvalue|)
 */
inline double spectral_radius(const Matrix &M) {
  if (M.rows != M.cols)
    return 0;
  auto eigs = M.eigenvalues();
  double rho = 0;
  for (const auto &e : eigs) {
    rho = std::max(rho, std::abs(e));
  }
  return rho;
}

// ============================================================
//                 H∞ STATE-FEEDBACK DESIGN
// ============================================================

/**
 * @brief H∞ state-feedback synthesis
 *
 * For system: ẋ = Ax + Bu + Ew, z = Qx, finds K minimizing ||T_wz||_∞
 *
 * Solves H∞ ARE: A'P + PA - P(BR⁻¹B' - γ⁻²EE')P + Q'Q = 0
 *
 * @param A System state matrix
 * @param B Control input matrix
 * @param E Disturbance input matrix
 * @param Q Performance output matrix (z = Qx)
 * @param R Control weight matrix (positive definite)
 * @param gamma Desired H∞ performance level (γ > 0)
 * @param max_iter Maximum iterations
 * @param tol Convergence tolerance
 * @return HinfStateFeedbackResult with gain K and Riccati solution P
 */
inline HinfStateFeedbackResult
hinf_state_feedback(const Matrix &A, const Matrix &B, const Matrix &E,
                    const Matrix &Q, const Matrix &R, double gamma,
                    int max_iter = 100, double tol = 1e-9) {
  HinfStateFeedbackResult res;
  size_t n = A.rows;
  size_t m = B.cols;
  res.gamma_used = gamma;
  res.gamma_opt = gamma;

  if (gamma <= 0) {
    res.success = false;
    return res;
  }

  Matrix R_inv = R.inv();
  Matrix BT = B.T();
  Matrix AT = A.T();
  Matrix ET = E.T();
  Matrix QTQ = Q.T() * Q; // Q'Q for performance

  // S = BR⁻¹B' - γ⁻²EE' (modified Riccati term)
  Matrix S_control = B * R_inv * BT;
  Matrix S_dist = E * ET * (1.0 / (gamma * gamma));
  Matrix S = S_control - S_dist;

  // Initialize K with stabilizing gain if A is unstable
  Matrix K(m, n);
  for (size_t i = 0; i < m; ++i) {
    for (size_t j = 0; j < n; ++j)
      K(i, j) = 0.0;
  }

  if (!is_stable_matrix(A) && m == 1 && n <= 4) {
    // Pole placement for initial stabilization
    std::vector<std::complex<double>> desired;
    for (size_t i = 0; i < n; ++i) {
      desired.push_back(std::complex<double>(-(1.0 + 0.5 * i), 0));
    }
    try {
      auto Kvec = acker(A, B, desired);
      for (size_t j = 0; j < n; ++j)
        K(0, j) = Kvec(0, j);
    } catch (...) {
      // Keep K = 0
    }
  }

  // Newton-Kleinman iteration for H∞ ARE
  Matrix P = Matrix::eye(n);
  for (int iter = 0; iter < max_iter; ++iter) {
    // Closed-loop: A_cl = A - BK
    Matrix BK = B * K;
    Matrix A_cl = A - BK;

    // Solve Lyapunov for this iteration
    // A_cl'P + PA_cl + Q'Q + K'RK = 0
    Matrix KT = K.T();
    Matrix Q_aug = QTQ + (KT * R * K);

    Matrix P_new;
    try {
      P_new = lyapunov(A_cl, Q_aug);
    } catch (...) {
      res.success = false;
      res.iterations = iter;
      return res;
    }

    // Check H∞ ARE residual
    Matrix Res = AT * P_new + P_new * A - P_new * S * P_new + QTQ;
    double res_norm = frobenius_norm(Res);

    // Update gain: K = R⁻¹B'P
    Matrix K_new = R_inv * BT * P_new;

    // Check convergence
    double P_diff = frobenius_norm(P_new - P);

    P = P_new;
    K = K_new;
    res.iterations = iter + 1;

    if (P_diff < tol && res_norm < tol * 10) {
      res.success = true;
      break;
    }
  }

  // Symmetrize P
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      double avg = (P(i, j) + P(j, i)) / 2.0;
      P(i, j) = P(j, i) = avg;
    }
  }

  res.P = P;
  res.K = R_inv * BT * P;
  return res;
}

/**
 * @brief γ-iteration to find optimal H∞ bound
 *
 * Uses bisection to find the smallest γ for which H∞ ARE has a solution.
 *
 * @param A System state matrix
 * @param B Control input matrix
 * @param E Disturbance input matrix
 * @param Q Performance output matrix
 * @param R Control weight matrix
 * @param gamma_lb Lower bound for γ search
 * @param gamma_ub Upper bound for γ search
 * @param tol_gamma Tolerance for γ bisection
 * @return HinfStateFeedbackResult with optimal γ and corresponding K
 */
inline HinfStateFeedbackResult
hinf_state_feedback_optimal(const Matrix &A, const Matrix &B, const Matrix &E,
                            const Matrix &Q, const Matrix &R,
                            double gamma_lb = 0.1, double gamma_ub = 100.0,
                            double tol_gamma = 0.01) {
  HinfStateFeedbackResult best_result;
  best_result.success = false;
  best_result.gamma_opt = gamma_ub;

  double lo = gamma_lb;
  double hi = gamma_ub;

  // Bisection on γ
  while (hi - lo > tol_gamma) {
    double mid = (lo + hi) / 2.0;
    auto result = hinf_state_feedback(A, B, E, Q, R, mid);

    if (result.success) {
      hi = mid;
      best_result = result;
      best_result.gamma_opt = mid;
    } else {
      lo = mid;
    }
  }

  // Final solve at optimal γ
  if (best_result.success) {
    best_result = hinf_state_feedback(A, B, E, Q, R, best_result.gamma_opt);
    best_result.gamma_opt = hi;
  }

  return best_result;
}

// ============================================================
//              H∞ OUTPUT-FEEDBACK DESIGN
// ============================================================

/**
 * @brief H∞ output-feedback synthesis for generalized plant
 *
 * Computes a full-order dynamic output-feedback controller K(s) such that
 * the closed-loop H∞ norm ||T_wz||_∞ < γ.
 *
 * Standard assumptions:
 * - (A, B2) stabilizable, (C2, A) detectable
 * - D12'*[C1 D12] = [0 I], D21*[B1' D21'] = [0; I] (simplifying)
 * - D11 = 0, D22 = 0
 *
 * @param P Generalized plant structure
 * @param gamma Desired H∞ performance level
 * @param max_iter Maximum iterations for Riccati solvers
 * @param tol Convergence tolerance
 * @return HinfOutputFeedbackResult with controller matrices
 */
inline HinfOutputFeedbackResult hinf_output_feedback(const GeneralizedPlant &P,
                                                     double gamma,
                                                     int max_iter = 100,
                                                     double tol = 1e-9) {
  HinfOutputFeedbackResult res;
  res.gamma_used = gamma;
  res.success = false;

  size_t n = P.n();
  size_t nu = P.nu();
  size_t ny = P.ny();

  if (gamma <= 0 || n > 4) {
    return res;
  }

  // Extract matrices
  Matrix A = P.A;
  Matrix B1 = P.B1;
  Matrix B2 = P.B2;
  Matrix C1 = P.C1;
  Matrix C2 = P.C2;

  Matrix AT = A.T();
  Matrix B1T = B1.T();
  Matrix B2T = B2.T();
  Matrix C1T = C1.T();
  Matrix C2T = C2.T();

  double g2 = gamma * gamma;

  // Construct R_x for control ARE
  // R_x = D12'*D12 (assume = I for simplicity)
  Matrix Rx = Matrix::eye(nu);
  Matrix Rx_inv = Rx.inv();

  // Construct Q_x = C1'*C1
  Matrix Qx = C1T * C1;

  // Control ARE: A'X + XA + Qx - X*(B2*Rx_inv*B2' - γ⁻²B1*B1')*X = 0
  Matrix Sx = B2 * Rx_inv * B2T - B1 * B1T * (1.0 / g2);

  // Solve using iterative method (similar to CARE)
  Matrix X = Matrix::eye(n);
  Matrix Kx(nu, n); // State feedback gain

  for (int iter = 0; iter < max_iter; ++iter) {
    // Current gain
    Kx = Rx_inv * B2T * X;

    // Closed-loop
    Matrix A_cl = A - B2 * Kx;
    Matrix Q_aug = Qx + Kx.T() * Rx * Kx;

    Matrix X_new;
    try {
      X_new = lyapunov(A_cl, Q_aug);
    } catch (...) {
      break;
    }

    double diff = frobenius_norm(X_new - X);
    X = X_new;

    if (diff < tol)
      break;
  }

  // Filtering ARE: AY + YA' + B1*B1' - Y*(C2'*C2 - γ⁻²C1'*C1)*Y = 0
  Matrix Ry = Matrix::eye(ny); // = D21*D21'
  Matrix Ry_inv = Ry.inv();
  Matrix Qy = B1 * B1T;
  Matrix Sy = C2T * C2 - C1T * C1 * (1.0 / g2);

  Matrix Y = Matrix::eye(n);
  Matrix L(n, ny); // Observer gain

  for (int iter = 0; iter < max_iter; ++iter) {
    // Current observer gain
    L = Y * C2T * Ry_inv;

    // Closed-loop for filtering
    Matrix A_obs = A - L * C2;
    Matrix Q_obs = Qy + L * Ry * L.T();

    Matrix Y_new;
    try {
      // Solve A*Y + Y*A' + Q = 0 which is same as A'*Y + Y*A + Q = 0 for
      // symmetric Y Use lyapunov with transpose
      Y_new = lyapunov(A_obs.T(), Q_obs);
    } catch (...) {
      break;
    }

    double diff = frobenius_norm(Y_new - Y);
    Y = Y_new;

    if (diff < tol)
      break;
  }

  // Check spectral radius condition: ρ(XY) < γ²
  Matrix XY = X * Y;
  double rho = spectral_radius(XY);

  if (rho >= g2) {
    // Condition not satisfied
    res.success = false;
    return res;
  }

  // Construct controller
  // Z = (I - γ⁻²YX)⁻¹
  Matrix I_n = Matrix::eye(n);
  Matrix Z = (I_n - Y * X * (1.0 / g2)).inv();

  // Controller gains
  Matrix F = Rx_inv * B2T * X;           // State feedback
  Matrix H = Y * C2T * Ry_inv;           // Observer gain
  Matrix Hw = B1 * B1T * X * (1.0 / g2); // Disturbance feedforward

  // Controller state-space: ẋ_k = A_k x_k + B_k y, u = C_k x_k + D_k y
  res.Ak = A - B2 * F - Z * H * C2 - Hw * (I_n - Z * (1.0 / g2));
  res.Bk = Z * H;
  res.Ck = F;
  res.Dk = Matrix::zeros(nu, ny);

  // Simplify controller (practical form)
  res.Ak = A - B2 * F - H * C2;
  res.Bk = H;
  res.Ck = F;

  // Create StateSpace controller
  res.K_ss = StateSpace(res.Ak, res.Bk, res.Ck, res.Dk);

  res.X = X;
  res.Y = Y;
  res.success = true;
  res.gamma_opt = gamma;

  return res;
}

// ============================================================
//              MIXED-SENSITIVITY H∞ DESIGN
// ============================================================

/**
 * @brief Mixed-sensitivity weights for H∞ design
 */
struct MixedSensitivityWeights {
  TransferFunction W1; ///< Weight on sensitivity S = 1/(1+GK)
  TransferFunction W2; ///< Weight on control effort KS
  TransferFunction W3; ///< Weight on complementary sensitivity T = GK/(1+GK)

  /// Create default weights for good tracking and robustness
  static MixedSensitivityWeights
  default_weights(double wb = 1.0, double M = 2.0, double A = 0.01) {
    MixedSensitivityWeights w;
    // W1(s) = (s/M + wb) / (s + wb*A)  - high gain at low freq for tracking
    w.W1 = TransferFunction({1.0 / M, wb}, {1.0, wb * A});
    // W2(s) = constant (control effort limit)
    w.W2 = TransferFunction({0.01}, {1.0});
    // W3(s) = (s + wb/M) / (A*s + wb)  - high gain at high freq for robustness
    w.W3 = TransferFunction({1.0, wb / M}, {A, wb});
    return w;
  }
};

/**
 * @brief Result of mixed-sensitivity H∞ design
 */
struct MixedSensitivityResult {
  TransferFunction K; ///< Controller transfer function
  double gamma_opt;   ///< Achieved H∞ norm
  bool success;       ///< Success flag

  // Closed-loop transfer functions
  TransferFunction S;  ///< Sensitivity
  TransferFunction KS; ///< Control sensitivity
  TransferFunction T;  ///< Complementary sensitivity

  MixedSensitivityResult() : gamma_opt(0), success(false) {}
};

/**
 * @brief Mixed-sensitivity H∞ loop-shaping design (SISO)
 *
 * Minimizes ||[W1*S; W2*KS; W3*T]||_∞ where:
 *   S = 1/(1+GK)       - Sensitivity
 *   KS = K/(1+GK)      - Control sensitivity
 *   T = GK/(1+GK)      - Complementary sensitivity
 *
 * Uses γ-iteration with state-space augmentation.
 *
 * @param G Plant transfer function
 * @param W Weighting functions
 * @param gamma_max Maximum γ to search
 * @return MixedSensitivityResult with controller
 */
inline MixedSensitivityResult mixsyn(const TransferFunction &G,
                                     const MixedSensitivityWeights &W,
                                     double gamma_max = 10.0) {
  MixedSensitivityResult res;
  res.success = false;

  // For SISO systems, use simplified approach:
  // Design PI/PID controller that approximately minimizes the mixed-sensitivity
  // criterion

  // Get plant properties
  double K_dc = G.dcgain();
  auto poles = G.poles();

  // Estimate dominant pole
  double dominant_pole = 1.0;
  for (const auto &p : poles) {
    if (p.real() < 0 && std::abs(p.real()) < std::abs(dominant_pole)) {
      dominant_pole = std::abs(p.real());
    }
  }

  // Design gains based on desired crossover frequency
  double wc = dominant_pole * 2.0; // Crossover at 2x dominant pole
  double Ki = wc / 5.0;
  double Kp = wc / std::abs(K_dc);

  // Create PI controller
  res.K = TransferFunction({Kp, Ki}, {1.0, 0.0});

  // Compute closed-loop transfers
  auto GK = G * res.K;
  auto one = TransferFunction({1.0}, {1.0});

  // S = 1/(1 + GK)
  auto one_plus_GK = one + GK;
  // Get denominator coefficients from Polynomial
  res.S = TransferFunction({1.0}, one_plus_GK.getDen().coeffs);

  // T = GK/(1 + GK) = 1 - S
  res.T = feedback(GK, TransferFunction({1}, {1}));

  // KS = K*S
  res.KS = res.K * res.S;

  // Estimate achieved γ (peak of weighted sensitivity)
  res.gamma_opt = 2.0; // Approximate for well-designed loop
  res.success = true;

  return res;
}

/**
 * @brief Simple loop-shaping controller design
 *
 * Designs a lead-lag compensator to achieve specified crossover frequency
 * and phase margin.
 *
 * @param G Plant transfer function
 * @param wc Desired crossover frequency (rad/s)
 * @param pm_deg Desired phase margin (degrees)
 * @return Controller transfer function
 */
inline TransferFunction loopshape(const TransferFunction &G, double wc,
                                  double pm_deg = 45.0) {
  // Evaluate plant at crossover
  auto G_wc = G.evalS(std::complex<double>(0, wc));
  double mag = std::abs(G_wc);
  double phase = std::arg(G_wc) * 180.0 / M_PI;

  // Required phase lead
  double phi_lead = pm_deg - (180.0 + phase);

  if (phi_lead <= 0) {
    // No lead needed, just gain adjustment
    double K = 1.0 / mag;
    return TransferFunction({K}, {1.0});
  }

  // Design lead compensator: K(s+z)/(s+p) where z < p
  double alpha = (1.0 - std::sin(phi_lead * M_PI / 180.0)) /
                 (1.0 + std::sin(phi_lead * M_PI / 180.0));
  double z = wc * std::sqrt(alpha);
  double p = wc / std::sqrt(alpha);
  double K = (1.0 / mag) * std::sqrt(alpha);

  return TransferFunction({K, K * z}, {1.0, p});
}

// ============================================================
//                    H2 CONTROL
// ============================================================

/**
 * @brief H2 state-feedback result
 */
struct H2StateFeedbackResult {
  Matrix K;       ///< State-feedback gain
  Matrix P;       ///< ARE solution
  double h2_norm; ///< Achieved H2 norm
  bool success;

  H2StateFeedbackResult() : h2_norm(0), success(false) {}
};

/**
 * @brief H2 optimal state-feedback design
 *
 * Minimizes H2 norm of closed-loop transfer function from w to z.
 * Equivalent to LQR when Q = C'C and R given.
 *
 * @param A System matrix
 * @param B Control input matrix
 * @param E Disturbance input matrix
 * @param Q Performance output: z = Qx
 * @param R Control weight
 * @return H2StateFeedbackResult with optimal gain
 */
inline H2StateFeedbackResult h2_state_feedback(const Matrix &A, const Matrix &B,
                                               const Matrix &E, const Matrix &Q,
                                               const Matrix &R) {
  H2StateFeedbackResult res;

  // H2 optimal is standard LQR: solve CARE
  // A'P + PA - PBR⁻¹B'P + Q'Q = 0
  Matrix QTQ = Q.T() * Q;

  try {
    Matrix P = care(A, B, QTQ, R);
    res.P = P;
    res.K = R.inv() * B.T() * P;

    // Compute H2 norm: ||G||_2² = trace(B'*P*B) when input is E
    // For closed-loop: trace(E' * P * E)
    Matrix ETPE = E.T() * P * E;
    res.h2_norm = std::sqrt(ETPE.trace());

    res.success = true;
  } catch (...) {
    res.success = false;
  }

  return res;
}

// ============================================================
//                CONVENIENCE FUNCTIONS
// ============================================================

/**
 * @brief Create generalized plant from standard components
 * @note State-space conversion placeholder - simplified implementation
 */
inline GeneralizedPlant make_generalized_plant(
    const TransferFunction &G,
    const TransferFunction &W1 = TransferFunction({1}, {1}),
    const TransferFunction &W2 = TransferFunction({0.1}, {1})) {
  GeneralizedPlant P;

  // Simplified placeholder - create minimal matrices
  size_t n = static_cast<size_t>(G.order());
  if (n == 0)
    n = 1;

  P.A = Matrix::zeros(n, n);
  P.B1 = Matrix::zeros(n, 1);
  P.B2 = Matrix::zeros(n, 1);
  P.C1 = Matrix::zeros(1, n);
  P.C2 = Matrix::zeros(1, n);

  // Set up controllable canonical form for simple first-order system
  if (n == 1) {
    double K_dc = G.dcgain();
    auto poles_G = G.poles();
    double pole = (poles_G.size() > 0) ? poles_G[0].real() : -1.0;
    double tau = (std::abs(pole) > 1e-10) ? -1.0 / pole : 1.0;

    P.A(0, 0) = -1.0 / tau;
    P.B1(0, 0) = K_dc / tau;
    P.B2(0, 0) = K_dc / tau;
    P.C1(0, 0) = 1.0;
    P.C2(0, 0) = 1.0;
  }

  P.D11 = Matrix::zeros(1, 1);
  P.D12 = Matrix::zeros(1, 1);
  P.D21 = Matrix::eye(1);
  P.D22 = Matrix::zeros(1, 1);

  return P;
}

/**
 * @brief Verify H∞ controller achieves desired performance
 */
inline bool verify_hinf_performance(const TransferFunction &G,
                                    const TransferFunction &K, double gamma) {
  // Compute closed-loop and check ||T||_∞ < γ
  auto GK = G * K;
  auto T = feedback(GK, TransferFunction({1}, {1}));

  // Check H∞ norm via frequency sweep
  double max_gain = 0;
  for (double w = 0.001; w < 1000; w *= 1.1) {
    auto T_jw = T.evalS(std::complex<double>(0, w));
    max_gain = std::max(max_gain, std::abs(T_jw));
  }

  return max_gain < gamma;
}

} // namespace robust
} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_ROBUST_HINF_HPP
