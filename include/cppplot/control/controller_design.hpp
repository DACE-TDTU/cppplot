/**
 * @file controller_design.hpp
 * @brief Controller design tools: Pole placement, LQR, Observers
 *
 * Provides MATLAB-like functions for controller and observer design
 */

#ifndef CPPPLOT_CONTROL_CONTROLLER_DESIGN_HPP
#define CPPPLOT_CONTROL_CONTROLLER_DESIGN_HPP

// Define M_PI for Windows compatibility
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "../pyplot.hpp"
#include "state_space.hpp"
#include "transfer_function.hpp"
#include <cmath>
#include <complex>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace cppplot {
// NOTE: Riccati helpers are defined in core/riccati.hpp (Eigen-backed).
// We forward-declare here to avoid pulling Eigen into the default control headers.
Matrix care(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R);
Matrix dare(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R);

namespace control {

// Use pyplot functions from parent namespace
using cppplot::axhline;
using cppplot::figure;
using cppplot::grid;
using cppplot::legend;
using cppplot::opts;
using cppplot::plot;
using cppplot::title;
using cppplot::xlabel;
using cppplot::xscale;
using cppplot::ylabel;
using cppplot::care;    // Laub Hamiltonian Schur method (riccati.hpp)
using cppplot::dare;    // Van Dooren Symplectic Schur method (riccati.hpp)


// ============================================================
//                    POLE PLACEMENT
// ============================================================

/**
 * @brief Compute Ackermann's formula for pole placement
 *
 * For system ẋ = Ax + Bu, find K such that eig(A - BK) = desired_poles
 *
 * K = [0 0 ... 0 1] * inv(Wc) * phi(A)
 * where phi(s) = (s - p1)(s - p2)...(s - pn) evaluated at A
 *
 * @param A System matrix
 * @param B Input matrix
 * @param poles Desired closed-loop poles
 * @return State feedback gain vector K
 */
inline Matrix acker(const Matrix &A, const Matrix &B,
                    const std::vector<std::complex<double>> &poles) {
  size_t n = A.rows;

  if (B.cols > 1) {
    throw std::runtime_error("Ackermann's formula only supports SISO systems "
                             "(B must have 1 column)");
  }

  if (poles.size() != n) {
    throw std::runtime_error("Number of poles must equal system order");
  }

  // Build controllability matrix
  Matrix Wc(n, n);
  Matrix Ak = Matrix::eye(n);
  for (size_t k = 0; k < n; ++k) {
    Matrix AkB = Ak * B;
    for (size_t i = 0; i < n; ++i) {
      Wc(i, k) = AkB(i, 0);
    }
    Ak = Ak * A;
  }

  // Check controllability
  if (std::abs(Wc.det()) < 1e-10) {
    throw std::runtime_error("System is not controllable");
  }

  // Compute characteristic polynomial from desired poles
  // phi(s) = (s - p1)(s - p2)...(s - pn)
  std::vector<std::complex<double>> char_coeffs(n + 1);
  char_coeffs[0] = 1.0;

  for (size_t i = 0; i < n; ++i) {
    // Multiply by (s - p_i)
    std::vector<std::complex<double>> new_coeffs(n + 1, 0);
    for (size_t j = 0; j <= i; ++j) {
      new_coeffs[j] += char_coeffs[j];                // s term
      new_coeffs[j + 1] -= poles[i] * char_coeffs[j]; // -p_i term
    }
    char_coeffs = new_coeffs;
  }

  // Compute phi(A) = A^n + c1*A^(n-1) + ... + cn*I
  Matrix phiA = Matrix::zeros(n, n);
  Ak = Matrix::eye(n);

  for (int k = n; k >= 0; --k) {
    double coeff = char_coeffs[n - k].real();
    phiA = phiA + Ak * coeff;
    if (k > 0)
      Ak = Ak * A;
  }

  // K = e_n^T * Wc^(-1) * phi(A)
  // where e_n = [0, 0, ..., 1]^T
  Matrix Wc_inv = Wc.inv();
  Matrix result = Wc_inv * phiA;

  // Extract last row as 1×n row matrix
  Matrix K(1, n);
  for (size_t i = 0; i < n; ++i) {
    K(0, i) = result(n - 1, i);
  }

  return K;
}

/**
 * @brief Place poles using Ackermann's formula (simplified interface)
 */
inline Matrix place(const Matrix &A, const Matrix &B,
                    const std::vector<double> &real_poles) {
  std::vector<std::complex<double>> poles;
  for (double p : real_poles) {
    poles.push_back(std::complex<double>(p, 0));
  }
  return acker(A, B, poles);
}

/**
 * @brief Place poles with complex conjugate pairs
 */
inline Matrix place_complex(const Matrix &A, const Matrix &B,
                            const std::vector<std::complex<double>> &poles) {
  return acker(A, B, poles);
}

// ============================================================
//                    LQR CONTROLLER
// ============================================================

// lyapunov() removed: use Matrix::lyapunov(A, Q) from matrix.hpp
// (Bartels-Stewart algorithm — numerically correct)

// care() reimplemented: thin wrapper → cppplot::care() in riccati.hpp
// (Laub 1979 Hamiltonian Schur method, Eigen3 backed — replaces Kleinman iteration)
// Signature kept identical for backward compatibility.

/**
 * @brief Linear Quadratic Regulator design
 *
 * Minimize J = ∫(x'Qx + u'Ru)dt subject to ẋ = Ax + Bu
 *
 * @param A System matrix
 * @param B Input matrix
 * @param Q State weighting matrix
 * @param R Input weighting matrix
 * @return LQR gain K such that u = -Kx
 */
inline Matrix lqr(const Matrix &A, const Matrix &B, const Matrix &Q,
                  const Matrix &R) {
  // Solve ARE for P
  Matrix P = care(A, B, Q, R);

  // K = R^(-1) * B' * P
  Matrix R_inv = R.inv();
  Matrix BT = B.T();
  Matrix K = R_inv * BT * P;

  return K;
}

/**
 * @brief LQR with diagonal weighting (simplified interface)
 */
inline Matrix lqr_simple(const Matrix &A, const Matrix &B,
                         const std::vector<double> &q_diag, double r) {
  size_t n = A.rows;

  Matrix Q(n, n, 0);
  for (size_t i = 0; i < std::min(n, q_diag.size()); ++i) {
    Q(i, i) = q_diag[i];
  }

  Matrix R(1, 1);
  R(0, 0) = r;

  return lqr(A, B, Q, R);
}

// ============================================================
//                    OBSERVER DESIGN
// ============================================================

/**
 * @brief Design Luenberger observer using pole placement
 *
 * Observer: x̂̇ = Ax̂ + Bu + L(y - Cx̂)
 *         = (A - LC)x̂ + Bu + Ly
 *
 * L is chosen such that eig(A - LC) = observer_poles
 *
 * Using duality: L' = acker(A', C', poles)
 */
inline Matrix observer_gain(const Matrix &A, const Matrix &C,
                            const std::vector<std::complex<double>> &poles) {
  // Observer design uses duality: place poles of (A - LC)
  // Same as placing poles of (A' - C'L')
  // So L' = acker(A', C', poles), then L = (L')'

  Matrix AT = A.T();
  Matrix CT = C.T();

  Matrix L_T = acker(AT, CT, poles); // 1×n row matrix

  // L is the transpose: n×1 column matrix
  return L_T.T();
}

/**
 * @brief Design observer with real poles (simplified)
 */
inline Matrix place_observer(const Matrix &A, const Matrix &C,
                             const std::vector<double> &poles) {
  std::vector<std::complex<double>> cpoles;
  for (double p : poles) {
    cpoles.push_back(std::complex<double>(p, 0));
  }
  return observer_gain(A, C, cpoles);
}

// ============================================================
//                    SENSITIVITY FUNCTIONS
// ============================================================

/**
 * @brief Calculate sensitivity function S(s) = 1/(1 + G(s)K(s))
 * and complementary sensitivity T(s) = G(s)K(s)/(1 + G(s)K(s))
 */
struct SensitivityFunctions {
  TransferFunction S;  // Sensitivity: 1/(1+GK)
  TransferFunction T;  // Complementary: GK/(1+GK)
  TransferFunction CS; // Control sensitivity: K/(1+GK)
  TransferFunction PS; // Plant sensitivity: G/(1+GK)
};

inline SensitivityFunctions sensitivity(const TransferFunction &G, // Plant
                                        const TransferFunction &K  // Controller
) {
  SensitivityFunctions sf;

  // L = G*K (loop transfer function)
  TransferFunction L = G * K;

  // S = 1/(1+L)
  TransferFunction one({1}, {1});
  sf.S = feedback(one, L);

  // T = L/(1+L) = 1 - S
  sf.T = feedback(L, one);

  // CS = K*S = K/(1+L)
  sf.CS = K * sf.S;

  // PS = G*S = G/(1+L)
  sf.PS = G * sf.S;

  return sf;
}

/**
 * @brief Plot sensitivity functions
 */
inline void sensitivity_plot(const TransferFunction &G,
                             const TransferFunction &K, double omega_min = 0.01,
                             double omega_max = 1000, int num_points = 200) {
  auto sf = sensitivity(G, K);

  // Generate frequency vector
  std::vector<double> omega;
  double ratio = std::pow(omega_max / omega_min, 1.0 / (num_points - 1));
  for (int i = 0; i < num_points; ++i) {
    omega.push_back(omega_min * std::pow(ratio, i));
  }

  // Calculate magnitudes
  std::vector<double> S_mag, T_mag, CS_mag, PS_mag;
  for (double w : omega) {
    S_mag.push_back(sf.S.mag_dB(w));
    T_mag.push_back(sf.T.mag_dB(w));
    CS_mag.push_back(sf.CS.mag_dB(w));
    PS_mag.push_back(sf.PS.mag_dB(w));
  }

  figure(900, 600);

  plot(omega, S_mag, "-",
       opts({{"color", "#1f77b4"},
             {"linewidth", "2"},
             {"label", "S (Sensitivity)"}}));
  plot(omega, T_mag, "-",
       opts({{"color", "#ff7f0e"},
             {"linewidth", "2"},
             {"label", "T (Comp. Sensitivity)"}}));
  plot(omega, CS_mag, "-",
       opts({{"color", "#2ca02c"},
             {"linewidth", "2"},
             {"label", "CS (Control Sens.)"}}));
  plot(omega, PS_mag, "-",
       opts({{"color", "#d62728"},
             {"linewidth", "2"},
             {"label", "PS (Plant Sens.)"}}));

  xscale("log");
  axhline(0,
          opts({{"color", "black"}, {"linestyle", "-"}, {"linewidth", "0.5"}}));
  axhline(6, opts({{"color", "red"},
                   {"linestyle", ":"},
                   {"linewidth", "1"}})); // M_s = 2

  xlabel("Frequency (rad/s)");
  ylabel("Magnitude (dB)");
  title("Sensitivity Functions");
  legend(true);
  grid(true);
}

// ============================================================
//                    PID TUNING
// ============================================================

/**
 * @brief Ziegler-Nichols tuning based on ultimate gain and period
 */
struct PIDGains {
  double Kp;
  double Ki;
  double Kd;
};

inline PIDGains ziegler_nichols(double Ku, double Tu,
                                std::string type = "PID") {
  PIDGains gains;

  if (type == "P") {
    gains.Kp = 0.5 * Ku;
    gains.Ki = 0;
    gains.Kd = 0;
  } else if (type == "PI") {
    gains.Kp = 0.45 * Ku;
    gains.Ki = 1.2 * gains.Kp / Tu;
    gains.Kd = 0;
  } else if (type == "PID") {
    gains.Kp = 0.6 * Ku;
    gains.Ki = 2.0 * gains.Kp / Tu;
    gains.Kd = gains.Kp * Tu / 8.0;
  } else if (type == "PID_no_overshoot") {
    gains.Kp = 0.2 * Ku;
    gains.Ki = 0.4 * gains.Kp / Tu;
    gains.Kd = gains.Kp * Tu / 15.0;
  } else {
    throw std::runtime_error("Unknown PID type: " + type);
  }

  return gains;
}

/**
 * @brief Cohen-Coon tuning for first-order plus dead time (FOPDT)
 * G(s) = K * e^(-Ls) / (Ts + 1)
 */
inline PIDGains cohen_coon(double K, double T, double L,
                           std::string type = "PID") {
  PIDGains gains;
  double r = L / T;

  if (type == "P") {
    gains.Kp = (1.0 / K) * (T / L) * (1 + r / 3.0);
    gains.Ki = 0;
    gains.Kd = 0;
  } else if (type == "PI") {
    gains.Kp = (1.0 / K) * (T / L) * (0.9 + r / 12.0);
    double Ti = L * (30 + 3 * r) / (9 + 20 * r);
    gains.Ki = gains.Kp / Ti;
    gains.Kd = 0;
  } else if (type == "PID") {
    gains.Kp = (1.0 / K) * (T / L) * (4.0 / 3.0 + r / 4.0);
    double Ti = L * (32 + 6 * r) / (13 + 8 * r);
    double Td = L * 4.0 / (11 + 2 * r);
    gains.Ki = gains.Kp / Ti;
    gains.Kd = gains.Kp * Td;
  } else {
    throw std::runtime_error("Unknown PID type: " + type);
  }

  return gains;
}

/**
 * @brief Create PID transfer function from gains
 */
inline TransferFunction pid_tf(const PIDGains &gains) {
  // PID: G(s) = Kp + Ki/s + Kd*s = (Kd*s^2 + Kp*s + Ki) / s
  return TransferFunction({gains.Kd, gains.Kp, gains.Ki}, {1, 0});
}

// ============================================================
//                    LEAD/LAG COMPENSATOR DESIGN
// ============================================================

/**
 * @brief Design lead compensator for phase margin improvement
 *
 * Lead compensator: Gc(s) = Kc * (s + z) / (s + p) where z < p
 *
 * @param phi_max Maximum phase lead required (degrees)
 * @param wc Desired crossover frequency
 * @return Transfer function of lead compensator
 */
inline TransferFunction design_lead(double phi_max, double wc,
                                    double Kc = 1.0) {
  double phi_rad = phi_max * M_PI / 180.0;

  // alpha = (1 - sin(phi_max)) / (1 + sin(phi_max))
  double alpha = (1 - std::sin(phi_rad)) / (1 + std::sin(phi_rad));

  // Maximum phase occurs at wm = sqrt(z*p) = wc
  // With z = wc*sqrt(alpha), p = wc/sqrt(alpha)
  double z = wc * std::sqrt(alpha);
  double p = wc / std::sqrt(alpha);

  // Gc(s) = Kc * (s + z) / (s + p)
  return TransferFunction({Kc, Kc * z}, {1, p});
}

/**
 * @brief Design lag compensator for steady-state error improvement
 *
 * Lag compensator: Gc(s) = Kc * (s + z) / (s + p) where z > p
 *
 * @param gain_increase Required DC gain increase
 * @param wc Desired crossover frequency
 * @return Transfer function of lag compensator
 */
inline TransferFunction design_lag(double gain_increase, double wc,
                                   double Kc = 1.0) {
  // beta = gain_increase (ratio of z/p)
  double beta = gain_increase;

  // Place zero at wc/10, pole at wc/(10*beta)
  double z = wc / 10.0;
  double p = z / beta;

  // Gc(s) = Kc * (s + z) / (s + p)
  return TransferFunction({Kc, Kc * z}, {1, p});
}

// ============================================================
//              DISCRETE ALGEBRAIC RICCATI EQUATION
// ============================================================

// dare() reimplemented: thin wrapper → cppplot::dare() in riccati.hpp
// (Van Dooren 1981 Symplectic Schur method, Eigen3 backed — replaces value iteration)
// Signature kept identical for backward compatibility.

/**
 * @brief Design Discrete LQR controller
 *
 * Minimizes: J = Σ_{k=0}^∞ (x_k'Qx_k + u_k'Ru_k)
 * Subject to: x_{k+1} = Ax_k + Bu_k
 *
 * Optimal control: u_k = -K*x_k
 * where K = (R + B'PB)^(-1) * B'PA
 * and P solves the DARE
 *
 * @param A System matrix (discrete-time)
 * @param B Input matrix
 * @param Q State weight
 * @param R Input weight
 * @return Optimal gain vector K
 */
inline Matrix dlqr(const Matrix &A, const Matrix &B, const Matrix &Q,
                   const Matrix &R) {
  size_t n = A.rows;
  size_t m = B.cols;

  // Solve DARE
  Matrix P = dare(A, B, Q, R);

  // Compute gain K = (R + B'PB)^(-1) * B'PA
  Matrix BT = B.T();
  Matrix BTP = BT * P;
  Matrix BTPB = BTP * B;
  Matrix S = R;
  for (size_t i = 0; i < m; ++i) {
    for (size_t j = 0; j < m; ++j) {
      S(i, j) += BTPB(i, j);
    }
  }

  Matrix S_inv = S.inv();
  Matrix BTPA = BTP * A;
  Matrix K = S_inv * BTPA;

  return K;
}

// ============================================================
//                    PRINT FUNCTIONS
// ============================================================

inline void print_controller_gains(const std::string &name,
                                   const std::vector<double> &K) {
  std::cout << name << " gains K = [";
  for (size_t i = 0; i < K.size(); ++i) {
    std::cout << std::fixed << std::setprecision(4) << K[i];
    if (i < K.size() - 1)
      std::cout << ", ";
  }
  std::cout << "]" << std::endl;
}

inline void print_pid_gains(const std::string &name, const PIDGains &gains) {
  std::cout << name << " PID: Kp=" << std::fixed << std::setprecision(4)
            << gains.Kp << ", Ki=" << gains.Ki << ", Kd=" << gains.Kd
            << std::endl;
}

} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_CONTROLLER_DESIGN_HPP
