/**
 * @file test_matrix.cpp
 * @brief Comprehensive tests for the new standalone Matrix class
 * 
 * Tests: basic ops, solve, eigenvalues (small & large), expm, Schur, QR, Lyapunov
 * 
 * Compile:
 *   g++ -std=c++17 -D_USE_MATH_DEFINES -I../include test_matrix.cpp -o test_matrix
 */

#include <cppplot/core/matrix.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <cmath>
#include <cassert>

using namespace cppplot;

int passed = 0, failed = 0;

void pass() { std::cout << "PASSED\n"; passed++; }
void fail(const std::string& msg) { std::cout << "FAILED: " << msg << "\n"; failed++; }

bool near(double a, double b, double tol = 1e-8) {
    return std::abs(a - b) < tol;
}

bool cnear(std::complex<double> a, std::complex<double> b, double tol = 1e-8) {
    return std::abs(a - b) < tol;
}

int main() {
    std::cout << "====== Matrix Class Tests ======\n\n";

    // --- Basic Operations ---
    std::cout << "TEST: Construction & access ... ";
    {
        Matrix A = {{1, 2}, {3, 4}};
        if (A(0, 0) == 1 && A(1, 1) == 4 && A.rows == 2 && A.cols == 2) pass();
        else fail("wrong values");
    }

    std::cout << "TEST: eye, zeros, diag ... ";
    {
        Matrix I = Matrix::eye(3);
        Matrix Z = Matrix::zeros(2, 3);
        Matrix D = Matrix::diag({1, 2, 3});
        if (I(0, 0) == 1 && I(1, 0) == 0 && Z(1, 2) == 0 && D(1, 1) == 2) pass();
        else fail("wrong factory");
    }

    std::cout << "TEST: Addition & subtraction ... ";
    {
        Matrix A = {{1, 2}, {3, 4}};
        Matrix B = {{5, 6}, {7, 8}};
        Matrix C = A + B;
        Matrix D = A - B;
        if (C(0, 0) == 6 && C(1, 1) == 12 && D(0, 0) == -4) pass();
        else fail("arithmetic");
    }

    std::cout << "TEST: Multiplication ... ";
    {
        Matrix A = {{1, 2}, {3, 4}};
        Matrix B = {{5, 6}, {7, 8}};
        Matrix C = A * B;
        if (C(0, 0) == 19 && C(0, 1) == 22 && C(1, 0) == 43 && C(1, 1) == 50) pass();
        else fail("mult wrong");
    }

    std::cout << "TEST: Scalar multiply & divide ... ";
    {
        Matrix A = {{2, 4}, {6, 8}};
        Matrix B = A * 0.5;
        Matrix C = A / 2.0;
        if (near(B(0, 0), 1) && near(C(1, 1), 4)) pass();
        else fail("scalar ops");
    }

    std::cout << "TEST: Transpose ... ";
    {
        Matrix A = {{1, 2, 3}, {4, 5, 6}};
        Matrix AT = A.T();
        if (AT.rows == 3 && AT.cols == 2 && AT(2, 1) == 6) pass();
        else fail("transpose");
    }

    std::cout << "TEST: Norms ... ";
    {
        Matrix A = {{3, -4}, {0, 5}};
        if (near(A.norm(), std::sqrt(50)) && near(A.normInf(), 7) && near(A.norm1(), 9)) pass();
        else fail("norms");
    }

    // --- Determinant & Inverse ---
    std::cout << "TEST: Determinant 2x2 ... ";
    {
        Matrix A = {{1, 2}, {3, 4}};
        if (near(A.det(), -2.0)) pass();
        else fail("det = " + std::to_string(A.det()));
    }

    std::cout << "TEST: Determinant 4x4 ... ";
    {
        Matrix A = {{1, 2, 3, 4}, {5, 6, 7, 8}, {2, 6, 4, 8}, {3, 1, 1, 2}};
        double d = A.det();
        if (near(d, 72.0, 1e-6)) pass();
        else fail("det4x4 = " + std::to_string(d));
    }

    std::cout << "TEST: Inverse 2x2 ... ";
    {
        Matrix A = {{1, 2}, {3, 4}};
        Matrix Ainv = A.inv();
        Matrix I = A * Ainv;
        if (near(I(0,0), 1) && near(I(0,1), 0) && near(I(1,0), 0) && near(I(1,1), 1)) pass();
        else fail("inverse");
    }

    std::cout << "TEST: Inverse 4x4 ... ";
    {
        Matrix A = {{1, 2, 3, 4}, {5, 6, 7, 8}, {2, 6, 4, 8}, {3, 1, 1, 2}};
        Matrix Ainv = A.inv();
        Matrix I = A * Ainv;
        bool ok = true;
        for (size_t i = 0; i < 4; ++i)
            for (size_t j = 0; j < 4; ++j)
                if (!near(I(i, j), (i == j) ? 1.0 : 0.0, 1e-8)) ok = false;
        if (ok) pass();
        else fail("inv4x4");
    }

    // --- Solve ---
    std::cout << "TEST: Solve Ax=b 3x3 ... ";
    {
        Matrix A = {{2, 1, -1}, {-3, -1, 2}, {-2, 1, 2}};
        Matrix b = {{8}, {-11}, {-3}};
        Matrix x = A.solve(b);
        // Expected: x = [2, 3, -1]
        if (near(x(0,0), 2) && near(x(1,0), 3) && near(x(2,0), -1)) pass();
        else fail("solve3x3: " + std::to_string(x(0,0)) + " " + std::to_string(x(1,0)) + " " + std::to_string(x(2,0)));
    }

    // --- QR Decomposition (Householder) ---
    std::cout << "TEST: QR decomposition ... ";
    {
        Matrix A = {{1, 2, 3}, {4, 5, 6}, {7, 8, 10}};
        auto qr = A.qr();
        Matrix QR = qr.Q * qr.R;
        bool ok = true;
        for (size_t i = 0; i < 3; ++i)
            for (size_t j = 0; j < 3; ++j)
                if (!near(QR(i, j), A(i, j), 1e-8)) ok = false;
        // Check Q is orthogonal: Q^T * Q = I
        Matrix QTQ = qr.Q.T() * qr.Q;
        for (size_t i = 0; i < 3; ++i)
            for (size_t j = 0; j < 3; ++j)
                if (!near(QTQ(i, j), (i == j) ? 1.0 : 0.0, 1e-8)) ok = false;
        if (ok) pass();
        else fail("QR");
    }

    // --- Eigenvalues ---
    std::cout << "TEST: Eigenvalues 2x2 real ... ";
    {
        Matrix A = {{2, 1}, {1, 2}};
        auto eigs = A.eigenvalues();
        // Eigenvalues: 3, 1
        bool found3 = false, found1 = false;
        for (auto& e : eigs) {
            if (near(e.real(), 3, 1e-8) && near(e.imag(), 0, 1e-8)) found3 = true;
            if (near(e.real(), 1, 1e-8) && near(e.imag(), 0, 1e-8)) found1 = true;
        }
        if (found3 && found1) pass();
        else fail("eig2x2");
    }

    std::cout << "TEST: Eigenvalues 2x2 complex ... ";
    {
        Matrix A = {{0, -1}, {1, 0}};
        auto eigs = A.eigenvalues();
        // Eigenvalues: +i, -i
        bool ok = (eigs.size() == 2);
        if (ok) {
            bool found_pi = false, found_ni = false;
            for (auto& e : eigs) {
                if (near(e.real(), 0, 1e-8) && near(e.imag(), 1, 1e-8)) found_pi = true;
                if (near(e.real(), 0, 1e-8) && near(e.imag(), -1, 1e-8)) found_ni = true;
            }
            ok = found_pi && found_ni;
        }
        if (ok) pass();
        else fail("eig2x2_complex");
    }

    std::cout << "TEST: Eigenvalues 3x3 ... ";
    {
        // A with known eigenvalues: construct A = P*D*P^(-1)
        // D = diag(1, 2, 3)
        Matrix A = {{1, 0, 0}, {0, 2, 0}, {0, 0, 3}};
        auto eigs = A.eigenvalues();
        bool found1 = false, found2 = false, found3 = false;
        for (auto& e : eigs) {
            if (near(e.real(), 1, 1e-6)) found1 = true;
            if (near(e.real(), 2, 1e-6)) found2 = true;
            if (near(e.real(), 3, 1e-6)) found3 = true;
        }
        if (found1 && found2 && found3) pass();
        else fail("eig3x3");
    }

    std::cout << "TEST: Eigenvalues 5x5 (NEW - was broken before) ... ";
    {
        // Companion matrix for polynomial (x-1)(x-2)(x-3)(x-4)(x-5)
        Matrix A = {{0, 0, 0, 0, 120}, {1, 0, 0, 0, -274}, {0, 1, 0, 0, 225},
                     {0, 0, 1, 0, -85}, {0, 0, 0, 1, 15}};
        auto eigs = A.eigenvalues();
        bool found[5] = {false};
        for (auto& e : eigs) {
            for (int k = 1; k <= 5; k++) {
                if (near(e.real(), k, 1e-4) && near(e.imag(), 0, 1e-4))
                    found[k-1] = true;
            }
        }
        bool all = true;
        for (int k = 0; k < 5; k++) all = all && found[k];
        if (all) pass();
        else {
            std::string msg = "eig5x5: ";
            for (auto& e : eigs) msg += "(" + std::to_string(e.real()) + "+" + std::to_string(e.imag()) + "i) ";
            fail(msg);
        }
    }

    std::cout << "TEST: Eigenvalues 8x8 diagonal ... ";
    {
        Matrix A(8, 8, 0);
        for (int i = 0; i < 8; i++) A(i, i) = (i + 1) * 1.0;
        auto eigs = A.eigenvalues();
        bool ok = (eigs.size() == 8);
        for (int k = 1; k <= 8 && ok; k++) {
            bool found = false;
            for (auto& e : eigs)
                if (near(e.real(), k, 1e-6) && near(e.imag(), 0, 1e-6)) found = true;
            ok = ok && found;
        }
        if (ok) pass();
        else fail("eig8x8");
    }

    // --- Schur Decomposition ---
    std::cout << "TEST: Schur decomposition 3x3 ... ";
    {
        Matrix A = {{1, 2, 3}, {4, 5, 6}, {7, 8, 10}};
        auto schur = A.schur();
        // Verify A = Q * T * Q^T
        Matrix recon = schur.Q * schur.T * schur.Q.T();
        bool ok = true;
        for (size_t i = 0; i < 3; ++i)
            for (size_t j = 0; j < 3; ++j)
                if (!near(recon(i, j), A(i, j), 1e-8)) ok = false;
        // Verify Q is orthogonal
        Matrix QTQ = schur.Q.T() * schur.Q;
        for (size_t i = 0; i < 3; ++i)
            for (size_t j = 0; j < 3; ++j)
                if (!near(QTQ(i, j), (i == j) ? 1.0 : 0.0, 1e-8)) ok = false;
        if (ok) pass();
        else fail("schur3x3");
    }

    // --- Matrix Exponential ---
    std::cout << "TEST: expm(zero) = I ... ";
    {
        Matrix Z = Matrix::zeros(3, 3);
        Matrix E = Z.expm();
        bool ok = true;
        for (size_t i = 0; i < 3; ++i)
            for (size_t j = 0; j < 3; ++j)
                if (!near(E(i, j), (i == j) ? 1.0 : 0.0, 1e-10)) ok = false;
        if (ok) pass();
        else fail("expm zero");
    }

    std::cout << "TEST: expm(diagonal) ... ";
    {
        Matrix A = {{1, 0}, {0, 2}};
        Matrix E = A.expm();
        if (near(E(0, 0), std::exp(1), 1e-8) && near(E(1, 1), std::exp(2), 1e-8)
            && near(E(0, 1), 0, 1e-8) && near(E(1, 0), 0, 1e-8)) pass();
        else fail("expm diag: " + std::to_string(E(0,0)) + " " + std::to_string(E(1,1)));
    }

    std::cout << "TEST: expm(nilpotent) ... ";
    {
        // A = [[0, 1], [0, 0]], expm(A) = [[1, 1], [0, 1]]
        Matrix A = {{0, 1}, {0, 0}};
        Matrix E = A.expm();
        if (near(E(0, 0), 1) && near(E(0, 1), 1) && near(E(1, 0), 0) && near(E(1, 1), 1)) pass();
        else fail("expm nilpotent");
    }

    std::cout << "TEST: expm(rotation) ... ";
    {
        // A = [[0, -pi/2], [pi/2, 0]], expm(A) should be ~rotation by pi/2
        double angle = M_PI / 2;
        Matrix A = {{0, -angle}, {angle, 0}};
        Matrix E = A.expm();
        // Expected: [[cos(pi/2), -sin(pi/2)], [sin(pi/2), cos(pi/2)]] = [[0, -1], [1, 0]]
        if (near(E(0, 0), 0, 1e-6) && near(E(0, 1), -1, 1e-6) 
            && near(E(1, 0), 1, 1e-6) && near(E(1, 1), 0, 1e-6)) pass();
        else fail("expm rotation: " + E.toString());
    }

    // --- Backward compatibility with control module ---
    std::cout << "TEST: Control: StateSpace uses new Matrix ... ";
    {
        using namespace cppplot::control;
        Matrix A_ctrl = {{0, 1}, {-2, -3}};
        Matrix B_ctrl = {{0}, {1}};
        Matrix C_ctrl = {{1, 0}};
        Matrix D_ctrl = {{0}};
        StateSpace sys(A_ctrl, B_ctrl, C_ctrl, D_ctrl);
        auto poles = sys.poles();
        // Poles of s^2 + 3s + 2 = (s+1)(s+2) => -1, -2
        bool found1 = false, found2 = false;
        for (auto& p : poles) {
            if (near(p.real(), -1, 1e-6)) found1 = true;
            if (near(p.real(), -2, 1e-6)) found2 = true;
        }
        if (found1 && found2 && sys.isStable() && sys.isControllable()) pass();
        else fail("StateSpace compat");
    }

    std::cout << "TEST: Control: lsim RK4 integration ... ";
    {
        using namespace cppplot::control;
        // Simple integrator: dx/dt = u, y = x
        Matrix A_i = {{0}};
        Matrix B_i = {{1}};
        Matrix C_i = {{1}};
        StateSpace sys(A_i, B_i, C_i, 0.0);
        
        int N = 1000;
        std::vector<double> t(N), u(N, 1.0); // unit step
        for (int i = 0; i < N; i++) t[i] = i * 0.01; // dt = 0.01, total = 10s
        
        auto result = lsim(sys, u, t);
        auto y = result.first;
        // At t=10, y should be ~10 (integral of 1 over 10 seconds)
        if (near(y.back(), 9.99, 0.02)) pass();  // RK4 should be very accurate
        else fail("lsim RK4: y(10) = " + std::to_string(y.back()));
    }

    // --- Sylvester & Lyapunov ---
    std::cout << "TEST: Lyapunov equation: A*P + P*A^T + Q = 0 ... ";
    {
        Matrix A = {{-1, 0}, {0, -2}};
        Matrix Q = {{1, 0}, {0, 1}};
        Matrix P = Matrix::lyapunov(A, Q);
        // Verify: A*P + P*A^T + Q should be ~0
        Matrix residual = A * P + P * A.T() + Q;
        bool ok = true;
        for (size_t i = 0; i < 2; ++i)
            for (size_t j = 0; j < 2; ++j)
                if (!near(residual(i, j), 0, 1e-6)) ok = false;
        if (ok) pass();
        else fail("lyapunov residual: " + residual.toString());
    }

    // --- New Matrix utilities ---
    std::cout << "TEST: Condition number ... ";
    {
        // Well-conditioned matrix
        Matrix I = Matrix::eye(3);
        double c_I = I.cond();
        // Identity has cond = 1
        if (near(c_I, 1.0, 0.1)) pass();
        else fail("cond(I) = " + std::to_string(c_I));
    }

    std::cout << "TEST: Spectral radius ... ";
    {
        Matrix A = {{2, 0}, {0, 3}};
        double sr = A.spectralRadius();
        if (near(sr, 3.0, 0.01)) pass();
        else fail("spectralRadius = " + std::to_string(sr));
    }

    std::cout << "TEST: isSymmetric ... ";
    {
        Matrix S = {{1, 2}, {2, 4}};
        Matrix N = {{1, 2}, {3, 4}};
        if (S.isSymmetric() && !N.isSymmetric()) pass();
        else fail("isSymmetric");
    }

    std::cout << "TEST: isPositiveDefinite ... ";
    {
        Matrix P = {{4, 2}, {2, 4}};  // eigenvalues: 2, 6 (positive)
        Matrix N = {{1, 2}, {2, 1}};   // eigenvalues: -1, 3 (not PD)
        if (P.isPositiveDefinite() && !N.isPositiveDefinite()) pass();
        else fail("isPositiveDefinite");
    }

    std::cout << "TEST: Cholesky decomposition ... ";
    {
        Matrix A = {{4, 2}, {2, 5}};
        Matrix L = A.cholesky();
        Matrix recon = L * L.T();
        bool ok = true;
        for (size_t i = 0; i < 2; ++i)
            for (size_t j = 0; j < 2; ++j)
                if (!near(recon(i, j), A(i, j), 1e-10)) ok = false;
        if (ok) pass();
        else fail("cholesky: L*L^T != A");
    }

    // --- Summary ---
    std::cout << "\n====== Results: " << passed << " passed, " << failed << " failed ======\n";
    return failed;
}
