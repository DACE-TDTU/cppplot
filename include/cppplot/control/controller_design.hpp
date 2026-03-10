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

/**
 * @brief Solve continuous Lyapunov equation A'X + XA + Q = 0
 *
 * Uses direct solution for small matrices
 * This is a helper function for the CARE solver
 */
inline Matrix lyapunov(const Matrix &A, const Matrix &Q, int max_iter = 1000,
                       double tol = 1e-12) {
  size_t n = A.rows;

  // For 1x1: a'x + xa + q = 0 => 2ax + q = 0 => x = -q/(2a)
  if (n == 1) {
    Matrix X(1, 1);
    double a = A(0, 0);
    if (std::abs(a) < 1e-15) {
      X(0, 0) = 1.0; // Fallback
    } else {
      X(0, 0) = -Q(0, 0) / (2.0 * a);
    }
    return X;
  }

  // For 2x2: Direct solution using vectorization
  // A'X + XA + Q = 0
  //
  // Let X = [x11 x12; x21 x22]
  // A = [a11 a12; a21 a22]
  // A' = [a11 a21; a12 a22]
  //
  // A'X = [a11*x11 + a21*x21,  a11*x12 + a21*x22]
  //       [a12*x11 + a22*x21,  a12*x12 + a22*x22]
  //
  // XA = [x11*a11 + x12*a21,  x11*a12 + x12*a22]
  //      [x21*a11 + x22*a21,  x21*a12 + x22*a22]
  //
  // (A'X + XA + Q) = 0 gives 4 equations:
  // [1,1]: a11*x11 + a21*x21 + x11*a11 + x12*a21 + q11 = 0
  //      = 2*a11*x11 + a21*x21 + a21*x12 + q11 = 0
  //      = 2*a11*x11 + a21*(x21 + x12) + q11 = 0
  //
  // [1,2]: a11*x12 + a21*x22 + x11*a12 + x12*a22 + q12 = 0
  //      = a12*x11 + (a11+a22)*x12 + a21*x22 + q12 = 0
  //
  // [2,1]: a12*x11 + a22*x21 + x21*a11 + x22*a21 + q21 = 0
  //      = a12*x11 + (a11+a22)*x21 + a21*x22 + q21 = 0
  //
  // [2,2]: a12*x12 + a22*x22 + x21*a12 + x22*a22 + q22 = 0
  //      = a12*(x12+x21) + 2*a22*x22 + q22 = 0
  //
  if (n == 2) {
    double a11 = A(0, 0), a12 = A(0, 1), a21 = A(1, 0), a22 = A(1, 1);

    // Build 4x4 coefficient matrix for [x11, x12, x21, x22]
    // Eq [1,1]: 2*a11*x11 + a21*x12 + a21*x21 + 0*x22 = -q11
    // Eq [1,2]: a12*x11 + (a11+a22)*x12 + 0*x21 + a21*x22 = -q12
    // Eq [2,1]: a12*x11 + 0*x12 + (a11+a22)*x21 + a21*x22 = -q21
    // Eq [2,2]: 0*x11 + a12*x12 + a12*x21 + 2*a22*x22 = -q22

    double M[4][4] = {{2 * a11, a21, a21, 0},
                      {a12, a11 + a22, 0, a21},
                      {a12, 0, a11 + a22, a21},
                      {0, a12, a12, 2 * a22}};

    double b[4] = {-Q(0, 0), -Q(0, 1), -Q(1, 0), -Q(1, 1)};

    // Gaussian elimination with partial pivoting
    double Aug[4][5];
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j)
        Aug[i][j] = M[i][j];
      Aug[i][4] = b[i];
    }

    for (int col = 0; col < 4; ++col) {
      // Partial pivoting
      int max_row = col;
      for (int row = col + 1; row < 4; ++row) {
        if (std::abs(Aug[row][col]) > std::abs(Aug[max_row][col])) {
          max_row = row;
        }
      }
      for (int j = 0; j < 5; ++j) {
        std::swap(Aug[col][j], Aug[max_row][j]);
      }

      // Eliminate
      for (int row = col + 1; row < 4; ++row) {
        if (std::abs(Aug[col][col]) > 1e-15) {
          double factor = Aug[row][col] / Aug[col][col];
          for (int j = col; j < 5; ++j) {
            Aug[row][j] -= factor * Aug[col][j];
          }
        }
      }
    }

    // Back substitution
    double x[4];
    for (int i = 3; i >= 0; --i) {
      double sum = Aug[i][4];
      for (int j = i + 1; j < 4; ++j) {
        sum -= Aug[i][j] * x[j];
      }
      x[i] = (std::abs(Aug[i][i]) > 1e-15) ? sum / Aug[i][i] : 0.0;
    }

    Matrix X(2, 2);
    X(0, 0) = x[0]; // x11
    X(0, 1) = x[1]; // x12
    X(1, 0) = x[2]; // x21
    X(1, 1) = x[3]; // x22

    // Symmetrize for numerical stability
    X(0, 1) = X(1, 0) = (X(0, 1) + X(1, 0)) / 2.0;
    return X;
  }

  // For larger matrices: iterative method
  Matrix X = Matrix::zeros(n, n);
  Matrix AT = A.T();

  // Simple iteration: X_{k+1} such that A'X_{k+1} + X_{k+1}A = -Q
  // Use time-stepping: dX/dt = -(A'X + XA + Q), converges to steady state
  double dt = 0.01;
  double prev_norm = 1e30;

  for (int iter = 0; iter < max_iter; ++iter) {
    Matrix residual = AT * X + X * A + Q;

    double norm = 0;
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < n; ++j) {
        norm += residual(i, j) * residual(i, j);
      }
    }
    norm = std::sqrt(norm);

    if (norm < tol)
      break;

    // Adaptive step size
    if (norm < prev_norm) {
      dt = std::min(dt * 1.1, 0.5);
    } else {
      dt *= 0.5;
    }
    prev_norm = norm;

    // Update X
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < n; ++j) {
        X(i, j) -= dt * residual(i, j);
      }
    }

    // Symmetrize
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = i + 1; j < n; ++j) {
        double avg = (X(i, j) + X(j, i)) / 2.0;
        X(i, j) = X(j, i) = avg;
      }
    }
  }

  return X;
}

/**
 * @brief Solve continuous-time algebraic Riccati equation (ARE)
 * A'P + PA - PBR^(-1)B'P + Q = 0
 *
 * For n=2, uses analytical solution for common cases
 * For larger systems, uses Newton-Kleinman iteration
 */
inline Matrix care(const Matrix &A, const Matrix &B, const Matrix &Q,
                   const Matrix &R, int max_iter = 100, double tol = 1e-10) {
  size_t n = A.rows;

  Matrix R_inv = R.inv();
  Matrix BT = B.T();
  Matrix AT = A.T();

  // S = B * R^(-1) * B'
  Matrix S = B * R_inv * BT;

  // For 1x1 system: solve quadratic directly
  // A'P + PA - P*S*P + Q = 0  =>  2ap - s*p² + q = 0
  if (n == 1) {
    double a = A(0, 0);
    double s = S(0, 0);
    double q_val = Q(0, 0);

    // s*p² - 2a*p - q = 0
    double coef_a = s;
    double coef_b = -2 * a;
    double coef_c = -q_val;

    double disc = coef_b * coef_b - 4 * coef_a * coef_c;
    double p;
    if (std::abs(coef_a) < 1e-15) {
      p = (std::abs(a) > 1e-15) ? -q_val / (2 * a) : 1.0;
    } else {
      double p1 = (-coef_b + std::sqrt(std::max(disc, 0.0))) / (2 * coef_a);
      double p2 = (-coef_b - std::sqrt(std::max(disc, 0.0))) / (2 * coef_a);
      p = std::max(p1, p2);
    }
    Matrix P(1, 1);
    P(0, 0) = std::max(p, 0.01);
    return P;
  }

  // For 2x2 SISO systems: P is symmetric, so we solve for [p11, p12, p22]
  // Using the 4th order polynomial approach
  if (n == 2 && B.cols == 1) {
    // Extract system matrices
    double a11 = A(0, 0), a12 = A(0, 1), a21 = A(1, 0), a22 = A(1, 1);
    double b1 = B(0, 0), b2 = B(1, 0);
    double q11 = Q(0, 0), q12 = Q(0, 1), q22 = Q(1, 1);
    double r = R(0, 0);

    // S = B * B' / R
    double s11 = b1 * b1 / r, s12 = b1 * b2 / r, s22 = b2 * b2 / r;

    // ARE: A'P + PA - PSP + Q = 0
    // Let P = [p11, p12; p12, p22] (symmetric)
    //
    // This gives 3 equations (upper triangle):
    // [A'P]_11 + [PA]_11 - [PSP]_11 + q11 = 0
    // [A'P]_12 + [PA]_12 - [PSP]_12 + q12 = 0
    // [A'P]_22 + [PA]_22 - [PSP]_22 + q22 = 0

    // We use iterative refinement with good initial guess
    // For double integrator (A = [0 1; 0 0]), the solution is known

    // Check if it's a double integrator: A = [0 α; 0 0]
    bool is_double_integrator =
        (std::abs(a11) < 1e-10 && std::abs(a21) < 1e-10 &&
         std::abs(a22) < 1e-10 && std::abs(a12) > 1e-10);

    // Use fixed-point iteration: given P, compute K = R^(-1)B'P
    // Then solve Lyapunov: (A-BK)'P_new + P_new(A-BK) + Q + K'RK = 0

    Matrix P(2, 2);

    if (is_double_integrator) {
      // For double integrator with B = [0; b2], Q = diag(q1, q2), R = r
      // Analytical solution exists
      // P12 = sqrt(r * q11) / b2
      // P22 = sqrt(r * (2*P12/b2 + q22))
      // P11 can be computed from ARE

      // Use MATLAB-verified solution approach:
      // For double integrator dx/dt = [0 1; 0 0]x + [0; 1]u, Q=I, R=1:
      // K = [1, sqrt(3)] and P = [sqrt(3), 1; 1, sqrt(3)]

      double alpha = a12; // typically 1

      // Solve ARE equations analytically for this structure
      // Equation (1,2): a12*p22 - s12*p11*p12 - s22*p12*p22 + q12 = 0
      // Equation (1,1): 2*a12*p12 - s11*p11^2 - 2*s12*p11*p12 - s22*p12^2 + q11
      // = 0 Equation (2,2): -s11*p12^2 - 2*s12*p12*p22 - s22*p22^2 + q22 = 0

      if (std::abs(b1) < 1e-10 && std::abs(b2) > 1e-10) {
        // B = [0; b2] case (canonical form)
        double b = b2;
        double q1 = q11, q2 = q22;

        // From ARE for this structure:
        // p12 * alpha - (b^2/r) * p12 * p22 = 0 (if q12=0)
        // 2*alpha*p12 - (b^2/r)*p12^2 = q1
        // -(b^2/r)*p22^2 = -q2

        // From eq(3): p22 = sqrt(r*q2)/b
        double p22 = std::sqrt(r * q2) / std::abs(b);

        // From eq(1) rearranged: p12 = r*alpha / (b^2 * p22)
        // But this gives p12*p22 = r*alpha/b^2
        // Substituting in eq(2): 2*alpha*p12 - (b^2/r)*p12^2 = q1

        // Let's solve eq(2) for p12:
        // (b^2/r)*p12^2 - 2*alpha*p12 + q1 = 0
        double coef_a2 = (b * b / r);
        double coef_b2 = -2 * alpha;
        double coef_c2 = q1;
        double disc2 = coef_b2 * coef_b2 - 4 * coef_a2 * coef_c2;

        double p12;
        if (disc2 >= 0) {
          double p12_1 = (-coef_b2 + std::sqrt(disc2)) / (2 * coef_a2);
          double p12_2 = (-coef_b2 - std::sqrt(disc2)) / (2 * coef_a2);
          p12 = (p12_1 > 0) ? p12_1 : p12_2;
        } else {
          p12 = alpha; // fallback
        }

        // From eq(1): p11 can be found, but for double integrator
        // there's a simpler relation from the closed-loop
        // Actually, we need to verify p22 from eq(1)
        // eq(1): alpha*p22 - (b^2/r)*p12*p22 = 0 (q12=0)
        // => p22*(alpha - (b^2/r)*p12) = 0
        // If p22 ≠ 0, then p12 = r*alpha/b^2

        double p12_alt = r * alpha / (b * b);

        // Now compute p22 from eq(3) correctly
        // eq(3): -(b^2/r)*p22^2 + q2 = 0 => p22 = sqrt(r*q2)/b -- WAIT
        // eq(3) actual: -s22*p22^2 + q22 = 0 is wrong because we miss other
        // terms

        // Let me redo. Full ARE:
        // Row 1: 2*a12*p12 - s11*p11^2 - 2*s12*p11*p12 - s22*p12^2 + q11 = 0
        // Mixed: a11*p12 + a12*p22 + a21*p11 + a22*p12 - s*terms + q12 = 0
        // Row 2: 2*a21*p12 + 2*a22*p22 - s11*p12^2 - 2*s12*p12*p22 - s22*p22^2
        // + q22 = 0

        // For double integrator: a11=a21=a22=0, a12=alpha, s11=0, s12=0,
        // s22=b^2/r Row 1: 2*alpha*p12 - (b^2/r)*p12^2 + q1 = 0 Mixed:
        // alpha*p22 + q12 = 0 (if q12=0, then either alpha=0 or p22=0) Row 2:
        // -(b^2/r)*p22^2 + q2 = 0

        // Wait, the mixed equation should include PSP term
        // Mixed: a12*p22 - s22*p12*p22 + q12 = 0
        // => p22*(a12 - s22*p12) = -q12
        // If q12=0: p22*(alpha - (b^2/r)*p12) = 0
        // Since we want p22 > 0, we need: p12 = r*alpha/b^2

        p12 = r * alpha / (b * b);

        // From Row 2: p22 = sqrt(r*q2)/abs(b)
        p22 = std::sqrt(r * q2) / std::abs(b);

        // From Row 1: 2*alpha*p12 - (b^2/r)*p12^2 + q1 = 0
        // p11 doesn't appear in Row 1 for this case!
        // We need to find p11 differently.
        // p11 is obtained from the positive definiteness requirement
        // and the closed-loop stability.

        // Actually, for double integrator with Q = diag(q1, q2):
        // The solution is P = [p11, p12; p12, p22] where:
        // p22 = sqrt(r*q2)/b
        // p12 = r*alpha/(b^2) OR from quadratic
        // p11 comes from requiring det(P) > 0 and K = R^(-1)B'P stabilizes

        // K = [b*p12/r, b*p22/r] = [alpha/b, sqrt(q2/r)]
        // For stability of A-BK = [0, alpha; 0, 0] - [0; b]*K
        //                       = [0, alpha; -b*K1, -b*K2]
        //                       = [0, alpha; -alpha, -b*sqrt(q2/r)]
        // Characteristic: s^2 + b*sqrt(q2/r)*s + alpha^2
        // Stable if b*sqrt(q2/r) > 0 (always true)

        // To find p11, use the ARE residual condition:
        // The first row of ARE must be satisfied
        // Since s11=s12=0 for this structure, Row 1 becomes:
        // 2*alpha*p12 - (b^2/r)*p12^2 + q1 = 0
        // This is independent of p11!

        // So we need another equation. The full ARE has terms:
        // [A'P + PA]_11 = a11*p11 + a12*p12 + a11*p11 + a21*p12
        //               = 2*a11*p11 + (a12+a21)*p12
        // For double integrator: = alpha*p12
        // [PSP]_11 = s11*p11^2 + 2*s12*p11*p12 + s22*p12^2
        //          = (b^2/r)*p12^2 for our case

        // So Row 1: alpha*p12 - (b^2/r)*p12^2 + q1 = 0 -- wait I had factor of
        // 2 wrong

        // Actually [PA]_11 = p11*a11 + p12*a21 = 0
        // [A'P]_11 = a11*p11 + a21*p12 = 0
        // So [A'P + PA]_11 = 0 for double integrator!

        // That means: -s22*p12^2 + q1 = 0 => p12 = sqrt(r*q1)/b
        // NOT p12 = r*alpha/b^2

        // Let me reconsider. The mixed term equation:
        // [A'P]_12 = a11*p12 + a21*p22 = 0
        // [PA]_12 = p11*a12 + p12*a22 = p11*alpha
        // [A'P + PA]_12 = p11*alpha
        // [PSP]_12 = s11*p11*p12 + s12*(p11*p22+p12*p12) + s22*p12*p22
        //          = (b^2/r)*p12*p22
        // So: p11*alpha - (b^2/r)*p12*p22 + q12 = 0
        // If q12=0: p11 = (b^2/r)*p12*p22/alpha

        // Now Row 1:
        // [A'P + PA]_11 = 2*(a11*p11 + a21*p12) = 0
        // [PSP]_11 = (b^2/r)*p12^2
        // ARE_11: -(b^2/r)*p12^2 + q1 = 0
        // => p12 = sqrt(r*q1)/b ✓

        // Row 2:
        // [A'P + PA]_22 = 2*(a12*p12 + a22*p22) = 2*alpha*p12
        // [PSP]_22 = (b^2/r)*p22^2
        // ARE_22: 2*alpha*p12 - (b^2/r)*p22^2 + q2 = 0
        // => p22 = sqrt(r*(2*alpha*p12 + q2))/b
        //        = sqrt(r*(2*alpha*sqrt(r*q1)/b + q2))/b

        p12 = std::sqrt(r * q1) / std::abs(b);
        p22 = std::sqrt(r * (2 * alpha * p12 + q2)) / std::abs(b);
        double p11 =
            (b * b / r) * p12 * p22 / alpha; // from mixed eq with q12=0

        P(0, 0) = p11;
        P(0, 1) = p12;
        P(1, 0) = p12;
        P(1, 1) = p22;

        return P;
      }
    }

    // General 2x2 case: Use proper Kleinman initialization
    // The system might already be stable, use different approach
    P = Q;
    for (size_t i = 0; i < n; ++i) {
      if (P(i, i) < 1.0)
        P(i, i) = 1.0;
    }

    // Check eigenvalues of A to determine if system is stable
    double tr_A = A(0, 0) + A(1, 1);
    double det_A = A(0, 0) * A(1, 1) - A(0, 1) * A(1, 0);
    bool A_is_stable = (tr_A < 0 && det_A > 0);

    // For stable systems, start with small K (or K=0)
    // For unstable systems, first find a stabilizing K

    Matrix K(1, 2);
    if (A_is_stable) {
      // Start with K = 0, system is already stable
      K(0, 0) = 0;
      K(0, 1) = 0;
    } else {
      // Need to find stabilizing K using pole placement
      // Place poles at -1, -2
      std::vector<std::complex<double>> desired_poles;
      desired_poles.push_back(std::complex<double>(-1.0, 0));
      desired_poles.push_back(std::complex<double>(-2.0, 0));

      try {
        auto K_init = acker(A, B, desired_poles);
        K(0, 0) = K_init(0, 0);
        K(0, 1) = K_init(0, 1);
      } catch (...) {
        // Simple stabilization: K such that A-BK has negative eigenvalues
        // For B = [b1; b2], place at poles -1, -2
        double b1 = B(0, 0), b2 = B(1, 0);
        if (std::abs(b2) > 1e-10) {
          // Use Bass-Gura formula for canonical form
          double a1_cl = 3.0; // sum of desired poles (1+2)
          double a0_cl = 2.0; // product of desired poles (1*2)
          // Characteristic of A: s^2 - tr_A*s + det_A
          double a1_orig = -tr_A;
          double a0_orig = det_A;

          // Simplified for controllable canonical form
          K(0, 0) = (a0_cl - a0_orig) / b2;
          K(0, 1) = (a1_cl - a1_orig) / b2;
        }
      }
    }

    // Kleinman iteration
    for (int iter = 0; iter < max_iter; ++iter) {
      // A_cl = A - B*K
      Matrix BK = B * K; // n x n
      Matrix A_cl = A;
      for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
          A_cl(i, j) -= BK(i, j);
        }
      }

      // Check if A_cl is stable
      double tr_cl = A_cl(0, 0) + A_cl(1, 1);
      double det_cl = A_cl(0, 0) * A_cl(1, 1) - A_cl(0, 1) * A_cl(1, 0);

      if (tr_cl >= 0 || det_cl <= 0) {
        // A_cl is not stable, dampen K
        K(0, 0) *= 1.1;
        K(0, 1) *= 1.1;
        continue;
      }

      // Q_aug = Q + K'*R*K
      Matrix KT = K.T();
      Matrix KTRK = KT * R * K;
      Matrix Q_aug = Q + KTRK;

      // Solve Lyapunov: A_cl' * P_new + P_new * A_cl + Q_aug = 0
      Matrix P_new = lyapunov(A_cl, Q_aug);

      // Update gain: K_new = R^(-1) * B' * P_new
      Matrix K_new = R_inv * BT * P_new;

      // Check convergence
      double diff = 0;
      for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
          diff += std::abs(P_new(i, j) - P(i, j));
        }
      }

      // Check residual
      Matrix PSP = P_new * S * P_new;
      Matrix ATP = AT * P_new;
      Matrix PA = P_new * A;
      Matrix Res = ATP + PA - PSP + Q;

      double res_norm = 0;
      for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
          res_norm += Res(i, j) * Res(i, j);
        }
      }
      res_norm = std::sqrt(res_norm);

      P = P_new;
      K = K_new;

      if (diff < tol && res_norm < 1.0) {
        break;
      }
    }

    // Ensure symmetry
    P(0, 1) = P(1, 0) = (P(0, 1) + P(1, 0)) / 2.0;

    return P;
  }

  // Kleinman iteration for larger systems (n > 2)
  // Initialize with stabilizing gain from pole placement
  size_t m = B.cols;
  Matrix K(m, n, 0.0); // Gain matrix (m x n)

  // Check if A is already stable
  bool is_stable = true;
  if (n <= 4) {
    auto poles = A.eigenvalues();
    for (const auto &p : poles) {
      if (p.real() >= 0) {
        is_stable = false;
        break;
      }
    }
  }

  if (!is_stable) {
    // Use pole placement to get initial stabilizing gain
    // Place poles at -1, -2, -3, ... (simple stable locations)
    std::vector<std::complex<double>> desired_poles;
    for (size_t i = 0; i < n; ++i) {
      desired_poles.push_back(std::complex<double>(-(1.0 + i), 0));
    }

    if (m == 1) { // Ackermann only supports SISO
      try {
        auto K_init = acker(A, B, desired_poles);
        for (size_t i = 0; i < n; ++i) {
          K(0, i) = K_init(0, i);
        }
      } catch (...) {
        // If pole placement fails, use simple gain
        for (size_t i = 0; i < n; ++i) {
          K(0, i) = B(i, 0);
        }
      }
    } else {
      // For MIMO systems, simple heuristic initialization
      for (size_t i = 0; i < m; ++i) {
        for (size_t j = 0; j < n; ++j) {
          // Heuristic mapping
          K(i, j) = B(j, i);
        }
      }
    }
  }

  Matrix P = Matrix::eye(n); // Initial P

  for (int iter = 0; iter < max_iter; ++iter) {
    // Form closed-loop matrix A_cl = A - B*K
    Matrix BK = B * K; // n x n
    Matrix A_cl = A;
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < n; ++j) {
        A_cl(i, j) -= BK(i, j);
      }
    }

    // Form Q_aug = Q + K'*R*K
    Matrix KT = K.T();     // n x 1
    Matrix RK = R * K;     // 1 x n
    Matrix KTRK = KT * RK; // n x n
    Matrix Q_aug = Q;
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < n; ++j) {
        Q_aug(i, j) += KTRK(i, j);
      }
    }

    // Solve Lyapunov equation: A_cl'*P + P*A_cl + Q_aug = 0
    Matrix P_new = lyapunov(A_cl, Q_aug);

    // Update gain: K = R^(-1) * B' * P_new
    Matrix K_new = R_inv * BT * P_new;

    // Check convergence
    double diff = 0;
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < n; ++j) {
        diff += std::abs(P_new(i, j) - P(i, j));
      }
    }

    // Also check Riccati residual
    Matrix PSP = P_new * S * P_new;
    Matrix ATP = AT * P_new;
    Matrix PA = P_new * A;
    Matrix Res = ATP + PA - PSP + Q;

    double res_norm = 0;
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < n; ++j) {
        res_norm += Res(i, j) * Res(i, j);
      }
    }
    res_norm = std::sqrt(res_norm);

    P = P_new;
    K = K_new;

    if (diff < tol && res_norm < tol * 100)
      break;
  }

  // Ensure symmetry and positive definiteness
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      double avg = (P(i, j) + P(j, i)) / 2.0;
      P(i, j) = P(j, i) = avg;
    }
  }

  return P;
}

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

/**
 * @brief Solve Discrete Algebraic Riccati Equation (DARE)
 *
 * DARE: A'PA - P - A'PB(R + B'PB)^(-1)B'PA + Q = 0
 *
 * Solved using value iteration:
 *   P_{k+1} = A'P_k A - A'P_k B(R + B'P_k B)^(-1)B'P_k A + Q
 *
 * @param A System matrix (n x n)
 * @param B Input matrix (n x m)
 * @param Q State weight (n x n)
 * @param R Input weight (m x m)
 * @param max_iter Maximum iterations
 * @param tol Convergence tolerance
 * @return P - Solution to DARE
 */
inline Matrix dare(const Matrix &A, const Matrix &B, const Matrix &Q,
                   const Matrix &R, int max_iter = 500, double tol = 1e-10) {
  size_t n = A.rows;
  size_t m = B.cols;

  // Initialize P = Q
  Matrix P = Q;
  Matrix AT = A.T();
  Matrix BT = B.T();

  for (int iter = 0; iter < max_iter; ++iter) {
    // Compute S = R + B'PB
    Matrix BTP = BT * P;
    Matrix BTPB = BTP * B;
    Matrix S = R;
    for (size_t i = 0; i < m; ++i) {
      for (size_t j = 0; j < m; ++j) {
        S(i, j) += BTPB(i, j);
      }
    }

    // Compute S^(-1)
    Matrix S_inv = S.inv();

    // Compute K = S^(-1) * B' * P * A
    Matrix BTPA = BTP * A;
    Matrix K = S_inv * BTPA;

    // Compute P_new = A'PA - A'PBK + Q = A'P(A - BK) + Q
    Matrix AmBK = A;
    Matrix BK = B * K;
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < n; ++j) {
        AmBK(i, j) -= BK(i, j);
      }
    }

    Matrix ATPA = AT * P * A;
    Matrix ATPBK = AT * P * BK;
    Matrix P_new = Q;
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < n; ++j) {
        P_new(i, j) += ATPA(i, j) - ATPBK(i, j);
      }
    }

    // Make symmetric
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = i + 1; j < n; ++j) {
        double avg = (P_new(i, j) + P_new(j, i)) / 2.0;
        P_new(i, j) = avg;
        P_new(j, i) = avg;
      }
    }

    // Check convergence
    double diff = 0;
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < n; ++j) {
        diff += std::abs(P_new(i, j) - P(i, j));
      }
    }

    P = P_new;

    if (diff < tol) {
      break;
    }
  }

  return P;
}

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
