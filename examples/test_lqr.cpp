/**
 * Test LQR/CARE solver
 */

#include "cppplot/control/control.hpp"
#include <iomanip>
#include <iostream>


using namespace cppplot::control;

int main() {
  std::cout << "=== LQR/CARE Solver Test ===" << std::endl;

  // Simple integrator system: x' = u
  // A = 0, B = 1
  std::cout << "\n[1] Simple Integrator (1x1)" << std::endl;
  {
    Matrix A(1, 1, 0);
    Matrix B(1, 1);
    B(0, 0) = 1.0;
    Matrix Q(1, 1);
    Q(0, 0) = 1.0;
    Matrix R(1, 1);
    R(0, 0) = 1.0;

    auto K = lqr(A, B, Q, R);
    std::cout << "  A = [0], B = [1], Q = [1], R = [1]" << std::endl;
    std::cout << "  LQR gain K = " << K(0, 0) << std::endl;
    std::cout << "  Expected: K = 1 (analytical)" << std::endl;
  }

  // Double integrator: x1' = x2, x2' = u
  // A = [0, 1; 0, 0], B = [0; 1]
  std::cout << "\n[2] Double Integrator (2x2)" << std::endl;
  {
    Matrix A = {{0, 1}, {0, 0}};
    Matrix B = {{0}, {1}};
    Matrix Q = {{1, 0}, {0, 1}};
    Matrix R(1, 1);
    R(0, 0) = 1.0;

    auto K = lqr(A, B, Q, R);
    std::cout << "  A = [0 1; 0 0], B = [0; 1], Q = I, R = 1" << std::endl;
    std::cout << "  LQR gain K = [" << K(0, 0) << ", " << K(0, 1) << "]"
              << std::endl;
    std::cout << "  Expected: K ≈ [1, sqrt(3)] = [1, 1.732]" << std::endl;
  }

  // Mass-spring-damper: m=1, c=0.5, k=2
  std::cout << "\n[3] Mass-Spring-Damper" << std::endl;
  {
    double m = 1.0, c = 0.5, k = 2.0;
    Matrix A = {{0, 1}, {-k / m, -c / m}};
    Matrix B = {{0}, {1 / m}};
    Matrix Q = {{10, 0}, {0, 1}};
    Matrix R(1, 1);
    R(0, 0) = 0.1;

    std::cout << "  A = [0 1; " << -k / m << " " << -c / m << "]" << std::endl;
    std::cout << "  B = [0; 1], Q = diag(10, 1), R = 0.1" << std::endl;

    // Test CARE directly
    auto P = care(A, B, Q, R);
    std::cout << "\n  CARE solution P:" << std::endl;
    std::cout << "    [" << P(0, 0) << ", " << P(0, 1) << "]" << std::endl;
    std::cout << "    [" << P(1, 0) << ", " << P(1, 1) << "]" << std::endl;

    auto K = lqr(A, B, Q, R);
    std::cout << "\n  LQR gain K = [" << K(0, 0) << ", " << K(0, 1) << "]"
              << std::endl;

    // Check Riccati residual
    Matrix R_inv = R.inv();
    Matrix BT = B.T();
    Matrix AT = A.T();
    Matrix S = B * R_inv * BT;
    Matrix PSP = P * S * P;
    Matrix ATP = AT * P;
    Matrix PA = P * A;
    Matrix Res = ATP + PA - PSP + Q;

    double res_norm = 0;
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        res_norm += Res(i, j) * Res(i, j);
      }
    }
    res_norm = std::sqrt(res_norm);
    std::cout << "\n  Riccati residual norm: " << res_norm << std::endl;
  }

  // Test Lyapunov solver directly
  std::cout << "\n[4] Lyapunov Solver Test" << std::endl;
  {
    // A'X + XA + Q = 0 where A is stable
    Matrix A = {{-1, 0}, {0, -2}}; // Stable diagonal matrix
    Matrix Q = {{1, 0}, {0, 1}};

    std::cout << "  A = [-1 0; 0 -2], Q = I" << std::endl;
    std::cout << "  Analytical: X = diag(0.5, 0.25)" << std::endl;

    Matrix X = lyapunov(A, Q);
    std::cout << "  Computed X:" << std::endl;
    std::cout << "    [" << X(0, 0) << ", " << X(0, 1) << "]" << std::endl;
    std::cout << "    [" << X(1, 0) << ", " << X(1, 1) << "]" << std::endl;
  }

  std::cout << "\n=== Done ===" << std::endl;
  return 0;
}
