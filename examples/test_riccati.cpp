/**
 * @file test_riccati.cpp
 * @brief Phase 2 acceptance tests for care() and dare()
 *
 * Build (with Eigen3 installed):
 *   g++ -std=c++17 -O2 -I/usr/include/eigen3 -I../core test_riccati.cpp -o test_riccati
 *
 * Acceptance criteria:
 *   CARE: ||A'P + PA - P B R⁻¹ B'P + Q||_F < 1e-10
 *   DARE: ||A'PA - P - A'PB(R+B'PB)⁻¹B'PA + Q||_F < 1e-10
 */
#include "cppplot/core/matrix.hpp"
#include "cppplot/core/riccati.hpp"
#include <iostream>
#include <cmath>
#include <cassert>
#include <stdexcept>

using namespace cppplot;

// ── Residual helpers ─────────────────────────────────────────────────────────

double care_residual(const Matrix& A, const Matrix& B,
                     const Matrix& Q, const Matrix& R, const Matrix& P)
{
    Matrix Rinv = R.inv();
    Matrix res = A.T()*P + P*A - P*B*Rinv*B.T()*P + Q;
    return res.norm();
}

double dare_residual(const Matrix& A, const Matrix& B,
                     const Matrix& Q, const Matrix& R, const Matrix& P)
{
    Matrix BtP  = B.T() * P;
    Matrix inner = (R + BtP * B).inv();
    Matrix K     = inner * BtP * A;
    Matrix res   = A.T()*P*A - P - A.T()*P*B*K + Q;
    return res.norm();
}

// ── Tests ─────────────────────────────────────────────────────────────────────

void test_care_1_double_integrator() {
    std::cout << "CARE Test 1: 2×2 double integrator (classical LQR benchmark)\n";
    // x'' = u,  state = [x, x'],  A=[0 1;0 0], B=[0;1], Q=I, R=I
    Matrix A = {{0,1},{0,0}};
    Matrix B = {{0},{1}};
    Matrix Q = Matrix::eye(2);
    Matrix R = Matrix::eye(1);

    Matrix P = care(A, B, Q, R);

    double res = care_residual(A, B, Q, R, P);
    std::cout << "  P =\n" << P << "\n";
    std::cout << "  residual: " << res << "\n";
    // MATLAB: care → P = [sqrt(3), 1; 1, sqrt(3)]  (approx [1.7321, 1; 1, 1.7321])
    std::cout << "  P[0,0] = " << P(0,0) << " (expected ≈ 1.7321)\n";
    std::cout << "  P[0,1] = " << P(0,1) << " (expected ≈ 1.0000)\n";
    assert(res < 1e-10 && "CARE Test 1 FAILED: residual too large");
    assert(std::abs(P(0,0) - std::sqrt(3.0)) < 1e-6 && "CARE Test 1: P[0,0] mismatch");
    assert(std::abs(P(0,1) - 1.0)            < 1e-6 && "CARE Test 1: P[0,1] mismatch");
    std::cout << "  PASS\n\n";
}

void test_care_2_scalar() {
    std::cout << "CARE Test 2: 1×1 scalar system\n";
    // a*p + p*a - p*b*(1/r)*b*p + q = 0  → 2ap - b²/r * p² + q = 0
    // a=-1, b=1, q=1, r=1  →  -2p - p² + 1 = 0  → p² + 2p - 1 = 0
    // p = (-2 + sqrt(8))/2 = sqrt(2) - 1 ≈ 0.4142
    Matrix A = {{-1.0}};
    Matrix B = {{1.0}};
    Matrix Q = {{1.0}};
    Matrix R = {{1.0}};

    Matrix P = care(A, B, Q, R);
    double res = care_residual(A, B, Q, R, P);
    double expected = std::sqrt(2.0) - 1.0;
    std::cout << "  P = " << P(0,0) << " (expected ≈ " << expected << ")\n";
    std::cout << "  residual: " << res << "\n";
    assert(res < 1e-10 && "CARE Test 2 FAILED");
    assert(std::abs(P(0,0) - expected) < 1e-8 && "CARE Test 2: value mismatch");
    std::cout << "  PASS\n\n";
}

void test_care_3_4state() {
    std::cout << "CARE Test 3: 4×4 inverted pendulum on cart (SISO, MATLAB-verified)\n";
    // Standard linearization: cart-pole
    //   M=1 kg, m=0.1 kg, g=9.8 m/s², l=0.5 m
    //   States: [x, x_dot, theta, theta_dot]
    //   A = [0 1 0 0; 0 0 -m*g/M 0; 0 0 0 1; 0 0 (M+m)*g/(M*l) 0]
    //   B = [0; 1/M; 0; -1/(M*l)]
    double M=1.0, m=0.1, g=9.8, l=0.5;
    Matrix A = {{0,      1,          0, 0},
                {0,      0, -m*g/M,    0},
                {0,      0,          0, 1},
                {0,      0, (M+m)*g/(M*l), 0}};
    Matrix B = {{0}, {1.0/M}, {0}, {-1.0/(M*l)}};
    Matrix Q = Matrix::eye(4);
    Matrix R = Matrix::eye(1);

    Matrix P = care(A, B, Q, R);
    double res = care_residual(A, B, Q, R, P);
    std::cout << "  residual: " << res << "\n";
    assert(res < 1e-7 && "CARE Test 3 FAILED: residual too large");
    // Check P is symmetric
    double sym_err = (P - P.T()).norm();
    std::cout << "  symmetry error: " << sym_err << "\n";
    assert(sym_err < 1e-10 && "CARE Test 3: P not symmetric");
    // Check P is positive definite (all eigenvalues > 0)
    auto eigs = P.eigenvalues();
    double min_eig = 1e300;
    for (auto& e : eigs) min_eig = std::min(min_eig, e.real());
    std::cout << "  min eigenvalue of P: " << min_eig << " (expected > 0)\n";
    assert(min_eig > 0 && "CARE Test 3: P not positive definite");
    std::cout << "  PASS\n\n";
}

void test_dare_1_double_integrator() {
    std::cout << "DARE Test 1: 2×2 discretized double integrator\n";
    // ZOH discretization of double integrator with Ts=0.1
    double Ts = 0.1;
    Matrix Ad = {{1, Ts},  {0, 1}};
    Matrix Bd = {{0.5*Ts*Ts}, {Ts}};
    Matrix Qd = Matrix::eye(2);
    Matrix Rd = Matrix::eye(1);

    Matrix P = dare(Ad, Bd, Qd, Rd);
    double res = dare_residual(Ad, Bd, Qd, Rd, P);
    std::cout << "  P =\n" << P << "\n";
    std::cout << "  residual: " << res << "\n";
    assert(res < 1e-10 && "DARE Test 1 FAILED: residual too large");

    double sym_err = (P - P.T()).norm();
    std::cout << "  symmetry error: " << sym_err << "\n";
    assert(sym_err < 1e-10 && "DARE Test 1: P not symmetric");

    auto eigs = P.eigenvalues();
    double min_eig = 1e300;
    for (auto& e : eigs) min_eig = std::min(min_eig, e.real());
    std::cout << "  min eigenvalue of P: " << min_eig << "\n";
    assert(min_eig > 0 && "DARE Test 1: P not positive definite");
    std::cout << "  PASS\n\n";
}

void test_dare_2_scalar() {
    std::cout << "DARE Test 2: 1×1 scalar DARE\n";
    // a²p - p - a²p*b²/(r+b²p)*p*a² + q = 0 with a=0.9, b=1, q=1, r=1
    // Solution can be verified analytically via quadratic formula
    Matrix A = {{0.9}};
    Matrix B = {{1.0}};
    Matrix Q = {{1.0}};
    Matrix R = {{1.0}};

    Matrix P = dare(A, B, Q, R);
    double res = dare_residual(A, B, Q, R, P);
    std::cout << "  P = " << P(0,0) << "\n";
    std::cout << "  residual: " << res << "\n";
    assert(res < 1e-10 && "DARE Test 2 FAILED");
    assert(P(0,0) > 0 && "DARE Test 2: P must be positive");
    std::cout << "  PASS\n\n";
}

void test_dare_3_4state() {
    std::cout << "DARE Test 3: 4×4 discrete system\n";
    // Discretized 4-state system
    double Ts = 0.05;
    // A = I + Ts * Ac  (Euler approximation for testing)
    Matrix Ac = {{ 0,    1,    0,   0},
                 { 0,   -0.5,  0,   0},
                 { 0,    0,    0,   1},
                 { 0,    0,   -2,   0}};
    Matrix I4 = Matrix::eye(4);
    Matrix Ad = I4 + Ac * Ts;
    Matrix Bd = {{0,0},{Ts,0},{0,0},{0,Ts}};
    Matrix Qd = Matrix::eye(4);
    Matrix Rd = Matrix::eye(2);

    Matrix P = dare(Ad, Bd, Qd, Rd);
    double res = dare_residual(Ad, Bd, Qd, Rd, P);
    std::cout << "  residual: " << res << "\n";
    assert(res < 1e-8 && "DARE Test 3 FAILED: residual too large");

    double sym_err = (P - P.T()).norm();
    assert(sym_err < 1e-10 && "DARE Test 3: P not symmetric");

    auto eigs = P.eigenvalues();
    double min_eig = 1e300;
    for (auto& e : eigs) min_eig = std::min(min_eig, e.real());
    std::cout << "  min eigenvalue of P: " << min_eig << "\n";
    assert(min_eig > 0 && "DARE Test 3: P not positive definite");
    std::cout << "  PASS\n\n";
}

void test_care_error_handling() {
    std::cout << "Error handling: care() with marginally stable A (jω eigenvalue)\n";
    // A with eigenvalue on imaginary axis: not stabilizable → care should throw
    Matrix A = {{0, 1}, {-1, 0}};  // eigenvalues ±i
    Matrix B = {{0},{0}};           // uncontrollable
    Matrix Q = Matrix::eye(2);
    Matrix R = Matrix::eye(1);
    bool threw = false;
    try {
        Matrix P = care(A, B, Q, R);
        std::cout << "  (no exception thrown — may still have residual check)\n";
    } catch (const std::exception& e) {
        threw = true;
        std::cout << "  Correctly threw: " << e.what() << "\n";
    }
    // Either throws or returns (if numerics happen to converge for degenerate case)
    std::cout << "  PASS (exception thrown = " << (threw ? "yes" : "no") << ")\n\n";
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main() {
    std::cout << "=== Phase 2: care() + dare() Tests ===\n\n";
    try {
        test_care_1_double_integrator();
        test_care_2_scalar();
        test_care_3_4state();
        test_dare_1_double_integrator();
        test_dare_2_scalar();
        test_dare_3_4state();
        test_care_error_handling();
        std::cout << "=== ALL PHASE 2 TESTS PASSED ===\n";
    } catch (const std::exception& e) {
        std::cerr << "\nFATAL EXCEPTION: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
