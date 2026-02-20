/**
 * @file state_space.hpp
 * @brief State-Space representation for control systems
 * 
 * State-space model: 
 *   ẋ = Ax + Bu
 *   y = Cx + Du
 * 
 * The Matrix class is now in core/matrix.hpp and provides:
 *   - Householder QR decomposition
 *   - QR iteration with Wilkinson shift (eigenvalues for any size)
 *   - Real Schur decomposition
 *   - Matrix exponential (Padé + scaling-and-squaring)
 *   - Linear system solver (LU with partial pivoting)
 *   - Sylvester / Lyapunov equation solvers
 */

#ifndef CPPPLOT_CONTROL_STATE_SPACE_HPP
#define CPPPLOT_CONTROL_STATE_SPACE_HPP

#include "../core/matrix.hpp"
#include "transfer_function.hpp"
#include <vector>
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <iostream>

namespace cppplot {
namespace control {

// Bring Matrix into the control namespace for backward compatibility
using cppplot::Matrix;

/**
 * @class StateSpace
 * @brief State-space representation: ẋ = Ax + Bu, y = Cx + Du
 */
class StateSpace {
public:
    Matrix A;  // State matrix (n x n)
    Matrix B;  // Input matrix (n x m)
    Matrix C;  // Output matrix (p x n)
    Matrix D;  // Feedthrough matrix (p x m)
    
    size_t n_states;   // Number of states
    size_t n_inputs;   // Number of inputs
    size_t n_outputs;  // Number of outputs
    
    // ============ Constructors ============
    
    StateSpace() : n_states(0), n_inputs(0), n_outputs(0) {}
    
    StateSpace(const Matrix& A_, const Matrix& B_, 
               const Matrix& C_, const Matrix& D_)
        : A(A_), B(B_), C(C_), D(D_) {
        n_states = A.rows;
        n_inputs = B.cols;
        n_outputs = C.rows;
        
        // Validate dimensions
        if (A.rows != A.cols) throw std::runtime_error("A must be square");
        if (B.rows != n_states) throw std::runtime_error("B row count must match A");
        if (C.cols != n_states) throw std::runtime_error("C column count must match A");
        if (D.rows != n_outputs || D.cols != n_inputs) {
            throw std::runtime_error("D dimensions must match B and C");
        }
    }
    
    // SISO constructor with scalar D
    StateSpace(const Matrix& A_, const Matrix& B_, 
               const Matrix& C_, double d = 0.0)
        : A(A_), B(B_), C(C_) {
        n_states = A.rows;
        n_inputs = B.cols;
        n_outputs = C.rows;
        D = Matrix(n_outputs, n_inputs, d);
    }
    
    // ============ Properties ============
    
    size_t order() const { return n_states; }
    
    bool isSISO() const { return n_inputs == 1 && n_outputs == 1; }
    
    // ============ Poles (eigenvalues of A) ============
    
    std::vector<std::complex<double>> poles() const {
        // Use the improved eigenvalue computation from Matrix class
        // which supports QR iteration for matrices up to 4x4
        // and falls back to characteristic polynomial roots for larger matrices
        return A.eigenvalues();
    }
    
    // Characteristic polynomial using Faddeev-LeVerrier
    Polynomial characteristic_polynomial() const {
        std::vector<double> coeffs(n_states + 1);
        coeffs[0] = 1.0;  // Leading coefficient
        
        Matrix M = Matrix::eye(n_states);
        Matrix AM(n_states, n_states);
        
        for (size_t k = 1; k <= n_states; ++k) {
            AM = A * M;
            coeffs[k] = -AM.trace() / k;
            if (k < n_states) {
                M = AM + Matrix::eye(n_states) * coeffs[k];
            }
        }
        
        return Polynomial(coeffs);
    }
    
    // ============ Stability ============
    
    bool isStable() const {
        auto p = poles();
        for (const auto& pole : p) {
            if (pole.real() >= 0) return false;
        }
        return true;
    }
    
    // ============ Controllability ============
    
    Matrix controllability_matrix() const {
        // Controllability matrix: [B, AB, A²B, ..., A^(n-1)B]
        Matrix Wc(n_states, n_states * n_inputs);
        Matrix Ak = Matrix::eye(n_states);
        
        for (size_t k = 0; k < n_states; ++k) {
            Matrix AkB = Ak * B;
            for (size_t i = 0; i < n_states; ++i) {
                for (size_t j = 0; j < n_inputs; ++j) {
                    Wc(i, k * n_inputs + j) = AkB(i, j);
                }
            }
            Ak = Ak * A;
        }
        return Wc;
    }
    
    bool isControllable() const {
        Matrix Wc = controllability_matrix();
        // System is controllable if rank(Wc) = n_states
        // Use numerical rank computation with tolerance
        return Wc.rank(1e-10) >= n_states;
    }
    
    // ============ Observability ============
    
    Matrix observability_matrix() const {
        // Observability matrix: [C; CA; CA²; ...; CA^(n-1)]
        Matrix Wo(n_states * n_outputs, n_states);
        Matrix Ak = Matrix::eye(n_states);
        
        for (size_t k = 0; k < n_states; ++k) {
            Matrix CAk = C * Ak;
            for (size_t i = 0; i < n_outputs; ++i) {
                for (size_t j = 0; j < n_states; ++j) {
                    Wo(k * n_outputs + i, j) = CAk(i, j);
                }
            }
            Ak = A * Ak;
        }
        return Wo;
    }
    
    bool isObservable() const {
        Matrix Wo = observability_matrix();
        // System is observable if rank(Wo) = n_states
        // Use numerical rank computation with tolerance
        return Wo.rank(1e-10) >= n_states;
    }
    
    // ============ String Representation ============
    
    std::string toString() const {
        std::ostringstream oss;
        oss << "State-Space System (n=" << n_states 
            << ", m=" << n_inputs << ", p=" << n_outputs << ")\n\n";
        oss << "A =\n" << A.toString() << "\n\n";
        oss << "B =\n" << B.toString() << "\n\n";
        oss << "C =\n" << C.toString() << "\n\n";
        oss << "D =\n" << D.toString();
        return oss.str();
    }
};

// ============ Conversion Functions ============

/**
 * @brief Convert transfer function to controllable canonical form state-space
 * 
 * For G(s) = (b_m s^m + ... + b_0) / (s^n + a_{n-1} s^{n-1} + ... + a_0)
 * 
 * Controllable canonical form:
 * A = [0  1  0 ... 0 ]    B = [0]
 *     [0  0  1 ... 0 ]        [0]
 *     [    ...      ]        [.]
 *     [-a0 -a1 ... -an-1]     [1]
 */
inline StateSpace tf2ss(const TransferFunction& G) {
    int n = G.den.degree();
    int m = G.num.degree();
    
    if (n == 0) {
        // Static gain
        Matrix A(1, 1, 0);
        Matrix B(1, 1, 0);
        Matrix C(1, 1, 0);
        double d = G.num.coeffs[0] / G.den.coeffs[0];
        return StateSpace(A, B, C, d);
    }
    
    // Normalize denominator (make leading coeff = 1)
    std::vector<double> a(n);
    double lead = G.den.coeffs[0];
    for (int i = 0; i < n; ++i) {
        a[i] = G.den.coeffs[n - i] / lead;  // a[0] = a_0, a[n-1] = a_{n-1}
    }
    
    // Normalize numerator
    std::vector<double> b(n, 0);
    for (int i = 0; i <= m; ++i) {
        b[n - 1 - m + i] = G.num.coeffs[i] / lead;
    }
    
    // Build A matrix (controllable canonical form)
    Matrix A(n, n, 0);
    for (int i = 0; i < n - 1; ++i) {
        A(i, i + 1) = 1.0;
    }
    for (int i = 0; i < n; ++i) {
        A(n - 1, i) = -a[i];
    }
    
    // Build B matrix
    Matrix B(n, 1, 0);
    B(n - 1, 0) = 1.0;
    
    // Build C matrix
    Matrix C(1, n, 0);
    for (int i = 0; i < n; ++i) {
        C(0, i) = b[i];
    }
    
    // D matrix (feedthrough)
    double d = 0;
    if (m >= n) {
        d = G.num.coeffs[0] / lead;
    }
    
    return StateSpace(A, B, C, d);
}

/**
 * @brief Convert state-space to transfer function (SISO only)
 * G(s) = C(sI - A)^(-1)B + D
 * 
 * This implementation works for any system order by using:
 * - Denominator = characteristic polynomial of A
 * - Numerator computed using Leverrier-Faddeev algorithm
 * 
 * For system: ẋ = Ax + Bu, y = Cx + Du
 * G(s) = C * adj(sI - A) * B / det(sI - A) + D
 */
inline TransferFunction ss2tf(const StateSpace& sys) {
    if (!sys.isSISO()) {
        throw std::runtime_error("ss2tf only supports SISO systems");
    }
    
    int n = static_cast<int>(sys.n_states);
    
    // Handle zero-state (static gain) case
    if (n == 0) {
        return TransferFunction({sys.D(0, 0)}, {1.0});
    }
    
    // Denominator = characteristic polynomial of A
    // Using Faddeev-LeVerrier: det(sI - A) = s^n + c1*s^(n-1) + ... + cn
    std::vector<double> char_coeffs(n + 1);
    char_coeffs[0] = 1.0;  // Leading coefficient
    
    Matrix M = Matrix::eye(n);
    for (int k = 1; k <= n; ++k) {
        Matrix AM = sys.A * M;
        char_coeffs[k] = -AM.trace() / k;
        if (k < n) {
            M = AM + Matrix::eye(n) * char_coeffs[k];
        }
    }
    
    Polynomial den(char_coeffs);
    
    // Numerator computation using Leverrier-Faddeev algorithm
    // adj(sI - A) = s^(n-1)*I + s^(n-2)*N1 + ... + N_{n-1}
    // where N_k matrices are computed iteratively
    // 
    // The numerator of C * adj(sI - A) * B is a polynomial of degree ≤ n-1:
    // num(s) = s^(n-1)*(C*B) + s^(n-2)*(C*N1*B) + ... + (C*N_{n-1}*B)
    
    // Compute N matrices and numerator coefficients
    std::vector<double> num_coeffs(n + 1, 0.0);  // May need n+1 for D contribution
    
    // Start with N0 = I
    Matrix Nk = Matrix::eye(n);
    
    for (int k = 0; k < n; ++k) {
        // Coefficient of s^(n-1-k) in numerator (before adding D)
        Matrix CNk = sys.C * Nk;   // 1 x n
        Matrix CNkB = CNk * sys.B; // 1 x 1
        num_coeffs[k + 1] = CNkB(0, 0);  // Store in reverse order (high power first)
        
        // Update: N_{k+1} = A*N_k + c_{k+1}*I
        if (k < n - 1) {
            Nk = sys.A * Nk + Matrix::eye(n) * char_coeffs[k + 1];
        }
    }
    
    // Shift coefficients to correct positions
    // num_coeffs[1] = C*I*B (coeff of s^(n-1))
    // num_coeffs[2] = C*N1*B (coeff of s^(n-2))
    // etc.
    std::vector<double> proper_num(n);
    for (int i = 0; i < n; ++i) {
        proper_num[i] = num_coeffs[i + 1];
    }
    
    // Add D contribution: G(s) = num/den + D = (num + D*den)/den
    double d = sys.D(0, 0);
    
    if (std::abs(d) > 1e-15) {
        // New numerator = proper_num(s) + D * den(s)
        // proper_num is degree n-1, den is degree n
        // Result is degree n
        std::vector<double> full_num(n + 1);
        
        // Add D * den(s)
        for (int i = 0; i <= n; ++i) {
            full_num[i] = d * char_coeffs[i];
        }
        
        // Add proper_num(s) (shifted to align powers)
        for (int i = 0; i < n; ++i) {
            full_num[i + 1] += proper_num[i];
        }
        
        // Remove leading zeros
        while (full_num.size() > 1 && std::abs(full_num.front()) < 1e-15) {
            full_num.erase(full_num.begin());
        }
        
        return TransferFunction(Polynomial(full_num), den);
    }
    
    // Strictly proper case (D = 0)
    // Remove leading zeros from numerator
    while (proper_num.size() > 1 && std::abs(proper_num.front()) < 1e-15) {
        proper_num.erase(proper_num.begin());
    }
    
    return TransferFunction(Polynomial(proper_num), den);
}

// ============ Time Response Simulation ============

/**
 * @brief Simulate state-space response with arbitrary input
 * @param sys State-space system
 * @param u Input signal (vector of values, one per time step)
 * @param t Time vector
 * @param x0 Initial state (optional)
 * @return Output vector y
 */
inline std::pair<std::vector<double>, std::vector<std::vector<double>>> 
lsim(const StateSpace& sys, 
     const std::vector<double>& u, 
     const std::vector<double>& t,
     const std::vector<double>& x0 = {}) 
{
    size_t N = t.size();
    size_t n = sys.n_states;
    
    // Initialize state
    std::vector<double> x(n, 0.0);
    if (!x0.empty()) {
        for (size_t i = 0; i < std::min(n, x0.size()); ++i) {
            x[i] = x0[i];
        }
    }
    
    std::vector<double> y(N);
    std::vector<std::vector<double>> x_history(N, std::vector<double>(n));
    
    // Helper lambda: compute ẋ = Ax + Bu for given state and input
    auto xdot = [&](const std::vector<double>& state, double input) {
        std::vector<double> dx(n);
        for (size_t i = 0; i < n; ++i) {
            double sum = 0;
            for (size_t j = 0; j < n; ++j)
                sum += sys.A(i, j) * state[j];
            dx[i] = sum + sys.B(i, 0) * input;
        }
        return dx;
    };
    
    for (size_t k = 0; k < N; ++k) {
        // Store state
        x_history[k] = x;
        
        // Output: y = Cx + Du
        y[k] = 0;
        for (size_t i = 0; i < n; ++i) {
            y[k] += sys.C(0, i) * x[i];
        }
        if (sys.D.rows > 0 && sys.D.cols > 0) {
            y[k] += sys.D(0, 0) * u[k];
        }
        
        // State update using RK4 integration (4th-order Runge-Kutta)
        if (k < N - 1) {
            double dt = t[k + 1] - t[k];
            double u_k = u[k];
            double u_k1 = u[k + 1];
            double u_mid = 0.5 * (u_k + u_k1); // Linear input interpolation

            auto k1 = xdot(x, u_k);

            std::vector<double> x_tmp(n);
            for (size_t i = 0; i < n; ++i)
                x_tmp[i] = x[i] + 0.5 * dt * k1[i];
            auto k2 = xdot(x_tmp, u_mid);

            for (size_t i = 0; i < n; ++i)
                x_tmp[i] = x[i] + 0.5 * dt * k2[i];
            auto k3 = xdot(x_tmp, u_mid);

            for (size_t i = 0; i < n; ++i)
                x_tmp[i] = x[i] + dt * k3[i];
            auto k4 = xdot(x_tmp, u_k1);

            for (size_t i = 0; i < n; ++i)
                x[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
    }
    
    return std::make_pair(y, x_history);
}

/**
 * @brief Step response of state-space system
 */
inline std::pair<std::vector<double>, std::vector<double>>
ss_step(const StateSpace& sys, double T = 10.0, int N = 500) {
    std::vector<double> t(N), u(N, 1.0);
    for (int i = 0; i < N; ++i) {
        t[i] = i * T / (N - 1);
    }
    
    auto result = lsim(sys, u, t);
    return std::make_pair(t, result.first);
}

/**
 * @brief Initial condition response
 */
inline std::pair<std::vector<double>, std::vector<double>>
initial(const StateSpace& sys, const std::vector<double>& x0,
        double T = 10.0, int N = 500) {
    std::vector<double> t(N), u(N, 0.0);  // Zero input
    for (int i = 0; i < N; ++i) {
        t[i] = i * T / (N - 1);
    }
    
    auto result = lsim(sys, u, t, x0);
    return std::make_pair(t, result.first);
}

// ============ Printing ============

inline void ssinfo(const StateSpace& sys) {
    std::cout << "========== State-Space System ==========" << std::endl;
    std::cout << sys.toString() << std::endl;
    std::cout << "\nProperties:" << std::endl;
    std::cout << "  States: " << sys.n_states << std::endl;
    std::cout << "  Inputs: " << sys.n_inputs << std::endl;
    std::cout << "  Outputs: " << sys.n_outputs << std::endl;
    std::cout << "  Stable: " << (sys.isStable() ? "Yes" : "No") << std::endl;
    std::cout << "  Controllable: " << (sys.isControllable() ? "Yes" : "No") << std::endl;
    std::cout << "  Observable: " << (sys.isObservable() ? "Yes" : "No") << std::endl;
    
    auto p = sys.poles();
    std::cout << "\nPoles:" << std::endl;
    for (size_t i = 0; i < p.size(); ++i) {
        std::cout << "  p" << (i+1) << " = " << std::fixed << std::setprecision(4)
                  << p[i].real();
        if (std::abs(p[i].imag()) > 1e-10) {
            std::cout << (p[i].imag() >= 0 ? " + " : " - ") 
                      << std::abs(p[i].imag()) << "j";
        }
        std::cout << std::endl;
    }
    std::cout << "=========================================" << std::endl;
}

} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_STATE_SPACE_HPP
