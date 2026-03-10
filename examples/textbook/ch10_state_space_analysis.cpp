/**
 * @file ch10_state_space_analysis.cpp
 * @brief Chapter 10: State-Space Analysis
 * 
 * Learning Outcomes:
 * - Calculate state transition matrix
 * - Test controllability and observability
 * - Compute system eigenvalues and modes
 * - Understand canonical forms
 * 
 * Physical Systems: DC Motor, Tank System, Aircraft
 * 
 * Build: g++ -std=c++14 -I "../../include" ch10_state_space_analysis.cpp -o ch10_state_space_analysis
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>

using namespace cppplot;
using namespace cppplot::control;

// ============================================================================
// HELPER: Compute matrix exponential using Taylor series
// ============================================================================
Matrix matrix_exp(const Matrix& A, double t, int terms = 50) {
    size_t n = A.rows;
    Matrix result = Matrix::eye(n);
    Matrix Ak = Matrix::eye(n);
    double tk = 1.0;
    double factorial = 1.0;
    
    for (int k = 1; k < terms; ++k) {
        Ak = Ak * A;
        tk *= t;
        factorial *= k;
        result = result + Ak * (tk / factorial);
    }
    return result;
}

// ============================================================================
// HELPER: Build controllability matrix
// ============================================================================
Matrix controllability_matrix(const Matrix& A, const Matrix& B) {
    size_t n = A.rows;
    Matrix Wc(n, n);
    Matrix Ak = Matrix::eye(n);
    
    for (size_t k = 0; k < n; ++k) {
        Matrix AkB = Ak * B;
        for (size_t i = 0; i < n; ++i) {
            Wc(i, k) = AkB(i, 0);
        }
        Ak = Ak * A;
    }
    return Wc;
}

// ============================================================================
// HELPER: Build observability matrix
// ============================================================================
Matrix observability_matrix(const Matrix& A, const Matrix& C) {
    size_t n = A.rows;
    Matrix Wo(n, n);
    Matrix Ak = Matrix::eye(n);
    
    for (size_t k = 0; k < n; ++k) {
        Matrix CAk = C * Ak;
        for (size_t j = 0; j < n; ++j) {
            Wo(k, j) = CAk(0, j);
        }
        Ak = A * Ak;
    }
    return Wo;
}

// ============================================================================
// HELPER: Simulate state trajectory
// ============================================================================
void simulate_state_trajectory(
    const Matrix& A, const Matrix& B, const Matrix& C,
    const Matrix& x0, double u_input, double t_final, double dt,
    std::vector<double>& t_out,
    std::vector<std::vector<double>>& x_out,
    std::vector<double>& y_out
) {
    size_t n = A.rows;
    Matrix x = x0;
    
    t_out.clear();
    x_out.clear();
    y_out.clear();
    
    // Discretize using matrix exponential
    Matrix Ad = matrix_exp(A, dt);
    
    // Bd ≈ A^(-1) * (e^(A*dt) - I) * B, but for simplicity use first-order
    Matrix Bd = B * dt;
    
    for (double t = 0; t <= t_final; t += dt) {
        // Store state
        t_out.push_back(t);
        std::vector<double> state_vec(n);
        for (size_t i = 0; i < n; ++i) {
            state_vec[i] = x(i, 0);
        }
        x_out.push_back(state_vec);
        
        // Output y = Cx
        Matrix y_mat = C * x;
        y_out.push_back(y_mat(0, 0));
        
        // State update: x(k+1) = Ad*x(k) + Bd*u
        Matrix u_mat(1, 1);
        u_mat(0, 0) = u_input;
        x = Ad * x + Bd * u_mat;
    }
}

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║     Chapter 10: State-Space Analysis                         ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    figure(1600, 1200);
    
    // ========================================================================
    // Example 1: DC Motor - Controllability and Observability
    // ========================================================================
    std::cout << "\n▶ Example 1: DC Motor Controllability/Observability Analysis" << std::endl;
    std::cout << "  ────────────────────────────────────────────────────────────" << std::endl;
    
    // DC Motor parameters
    double R = 2.0;      // Ohms
    double L = 0.5;      // H
    double Kt = 0.1;     // Nm/A
    double Kb = 0.1;     // V/(rad/s)
    double J = 0.01;     // kg·m²
    double b = 0.1;      // N·m·s
    
    // State: x = [i, ω]  (current, angular velocity)
    // Input: u = V (voltage)
    // Output: y = ω (angular velocity)
    
    Matrix A_motor = {{-R/L, -Kb/L}, {Kt/J, -b/J}};
    Matrix B_motor = {{1/L}, {0}};
    Matrix C_motor = {{0, 1}};
    
    std::cout << "\n  System Matrices:" << std::endl;
    std::cout << "  A = [" << A_motor(0,0) << ", " << A_motor(0,1) << "]" << std::endl;
    std::cout << "      [" << A_motor(1,0) << ", " << A_motor(1,1) << "]" << std::endl;
    std::cout << "  B = [" << B_motor(0,0) << "]" << std::endl;
    std::cout << "      [" << B_motor(1,0) << "]" << std::endl;
    std::cout << "  C = [" << C_motor(0,0) << ", " << C_motor(0,1) << "]" << std::endl;
    
    // Controllability
    Matrix Wc = controllability_matrix(A_motor, B_motor);
    double det_Wc = Wc.det();
    size_t rank_Wc = Wc.rank();
    
    std::cout << "\n  Controllability Matrix:" << std::endl;
    std::cout << "  Wc = [" << Wc(0,0) << ", " << Wc(0,1) << "]" << std::endl;
    std::cout << "       [" << Wc(1,0) << ", " << Wc(1,1) << "]" << std::endl;
    std::cout << "  det(Wc) = " << det_Wc << std::endl;
    std::cout << "  rank(Wc) = " << rank_Wc << " (n = 2)" << std::endl;
    std::cout << "  ✓ System is " << (rank_Wc == 2 ? "CONTROLLABLE" : "NOT controllable") << std::endl;
    
    // Observability
    Matrix Wo = observability_matrix(A_motor, C_motor);
    double det_Wo = Wo.det();
    size_t rank_Wo = Wo.rank();
    
    std::cout << "\n  Observability Matrix:" << std::endl;
    std::cout << "  Wo = [" << Wo(0,0) << ", " << Wo(0,1) << "]" << std::endl;
    std::cout << "       [" << Wo(1,0) << ", " << Wo(1,1) << "]" << std::endl;
    std::cout << "  det(Wo) = " << det_Wo << std::endl;
    std::cout << "  rank(Wo) = " << rank_Wo << " (n = 2)" << std::endl;
    std::cout << "  ✓ System is " << (rank_Wo == 2 ? "OBSERVABLE" : "NOT observable") << std::endl;
    
    // Eigenvalues (system modes)
    auto eigs = A_motor.eigenvalues();
    std::cout << "\n  System Eigenvalues (Modes):" << std::endl;
    for (size_t i = 0; i < eigs.size(); ++i) {
        std::cout << "  λ" << (i+1) << " = " << std::fixed << std::setprecision(4) 
                  << eigs[i].real();
        if (std::abs(eigs[i].imag()) > 1e-10) {
            std::cout << (eigs[i].imag() >= 0 ? " + " : " - ") 
                      << std::abs(eigs[i].imag()) << "j";
        }
        std::cout << std::endl;
    }
    
    // State transition matrix at t = 0.1s
    double t_eval = 0.1;
    Matrix Phi = matrix_exp(A_motor, t_eval);
    std::cout << "\n  State Transition Matrix Φ(" << t_eval << "):" << std::endl;
    std::cout << "  Φ = [" << std::fixed << std::setprecision(4) 
              << Phi(0,0) << ", " << Phi(0,1) << "]" << std::endl;
    std::cout << "      [" << Phi(1,0) << ", " << Phi(1,1) << "]" << std::endl;
    
    // Simulate step response
    Matrix x0_motor(2, 1);
    x0_motor(0, 0) = 0;  // Initial current
    x0_motor(1, 0) = 0;  // Initial velocity
    
    std::vector<double> t_motor, y_motor;
    std::vector<std::vector<double>> x_motor;
    simulate_state_trajectory(A_motor, B_motor, C_motor, x0_motor, 12.0, 1.0, 0.001,
                              t_motor, x_motor, y_motor);
    
    // Extract state trajectories
    std::vector<double> current, velocity;
    for (const auto& x : x_motor) {
        current.push_back(x[0]);
        velocity.push_back(x[1]);
    }
    
    // Plot 1: DC Motor state trajectories
    subplot(2, 3, 1);
    plot(t_motor, current, "b-", opts({{"linewidth", "2"}, {"label", "Current i(t)"}}));
    xlabel("Time (s)");
    ylabel("Current (A)");
    title("DC Motor: State x₁ = Current");
    grid(true);
    
    subplot(2, 3, 2);
    plot(t_motor, velocity, "r-", opts({{"linewidth", "2"}, {"label", "Velocity ω(t)"}}));
    xlabel("Time (s)");
    ylabel("Velocity (rad/s)");
    title("DC Motor: State x₂ = Angular Velocity");
    grid(true);
    
    // Phase portrait
    subplot(2, 3, 3);
    plot(current, velocity, "g-", opts({{"linewidth", "2"}}));
    scatter({current[0]}, {velocity[0]}, opts({{"s", "50"}, {"color", "blue"}, {"marker", "o"}}));
    scatter({current.back()}, {velocity.back()}, opts({{"s", "50"}, {"color", "red"}, {"marker", "x"}}));
    xlabel("Current i (A)");
    ylabel("Velocity ω (rad/s)");
    title("Phase Portrait");
    grid(true);
    
    // ========================================================================
    // Example 2: Uncontrollable System
    // ========================================================================
    std::cout << "\n▶ Example 2: Uncontrollable System Example" << std::endl;
    std::cout << "  ─────────────────────────────────────────" << std::endl;
    
    // System where B is aligned with an eigenvector
    Matrix A_uncontrollable = {{-1, 0}, {0, -2}};  // Diagonal (decoupled modes)
    Matrix B_uncontrollable = {{1}, {0}};          // Only affects first mode
    Matrix C_uncontrollable = {{1, 1}};            // Measures sum
    
    std::cout << "\n  System with decoupled modes:" << std::endl;
    std::cout << "  A = [-1,  0]    (λ₁ = -1, λ₂ = -2)" << std::endl;
    std::cout << "      [ 0, -2]" << std::endl;
    std::cout << "  B = [1]         (Input only affects x₁)" << std::endl;
    std::cout << "      [0]" << std::endl;
    
    Matrix Wc_uncont = controllability_matrix(A_uncontrollable, B_uncontrollable);
    std::cout << "\n  Controllability Matrix:" << std::endl;
    std::cout << "  Wc = [B, AB] = [" << Wc_uncont(0,0) << ", " << Wc_uncont(0,1) << "]" << std::endl;
    std::cout << "                 [" << Wc_uncont(1,0) << ", " << Wc_uncont(1,1) << "]" << std::endl;
    std::cout << "  det(Wc) = " << Wc_uncont.det() << std::endl;
    std::cout << "  rank(Wc) = " << Wc_uncont.rank() << " < n = 2" << std::endl;
    std::cout << "  ✗ System is NOT controllable!" << std::endl;
    std::cout << "    → Mode λ₂ = -2 cannot be controlled" << std::endl;
    
    // ========================================================================
    // Example 3: State Transition Matrix Properties
    // ========================================================================
    std::cout << "\n▶ Example 3: State Transition Matrix Properties" << std::endl;
    std::cout << "  ──────────────────────────────────────────────" << std::endl;
    
    // Verify Φ(0) = I
    Matrix Phi_0 = matrix_exp(A_motor, 0);
    std::cout << "\n  Property 1: Φ(0) = I" << std::endl;
    std::cout << "  Φ(0) = [" << Phi_0(0,0) << ", " << Phi_0(0,1) << "]" << std::endl;
    std::cout << "         [" << Phi_0(1,0) << ", " << Phi_0(1,1) << "]  ✓" << std::endl;
    
    // Verify Φ(t₁)Φ(t₂) = Φ(t₁+t₂)
    double t1 = 0.1, t2 = 0.2;
    Matrix Phi_t1 = matrix_exp(A_motor, t1);
    Matrix Phi_t2 = matrix_exp(A_motor, t2);
    Matrix Phi_t1_t2 = matrix_exp(A_motor, t1 + t2);
    Matrix Phi_product = Phi_t1 * Phi_t2;
    
    std::cout << "\n  Property 2: Φ(t₁)·Φ(t₂) = Φ(t₁+t₂)" << std::endl;
    std::cout << "  Φ(0.1)·Φ(0.2) = [" << std::fixed << std::setprecision(4)
              << Phi_product(0,0) << ", " << Phi_product(0,1) << "]" << std::endl;
    std::cout << "                  [" << Phi_product(1,0) << ", " << Phi_product(1,1) << "]" << std::endl;
    std::cout << "  Φ(0.3)        = [" << Phi_t1_t2(0,0) << ", " << Phi_t1_t2(0,1) << "]  ✓" << std::endl;
    std::cout << "                  [" << Phi_t1_t2(1,0) << ", " << Phi_t1_t2(1,1) << "]" << std::endl;
    
    // ========================================================================
    // Example 4: Two-Tank System (Observability Analysis)
    // ========================================================================
    std::cout << "\n▶ Example 4: Two-Tank System - Observability Analysis" << std::endl;
    std::cout << "  ───────────────────────────────────────────────────" << std::endl;
    
    // Two tanks connected in series
    // h₁̇ = -a₁·h₁ + u
    // h₂̇ = a₁·h₁ - a₂·h₂
    // y = h₂ (only second tank measured)
    
    double a1 = 0.5, a2 = 0.3;
    Matrix A_tanks = {{-a1, 0}, {a1, -a2}};
    Matrix B_tanks = {{1}, {0}};
    Matrix C_tanks = {{0, 1}};  // Only measure h₂
    
    std::cout << "\n  Two tanks in series, sensor on Tank 2 only:" << std::endl;
    std::cout << "  A = [" << A_tanks(0,0) << ", " << A_tanks(0,1) << "]" << std::endl;
    std::cout << "      [" << A_tanks(1,0) << ", " << A_tanks(1,1) << "]" << std::endl;
    std::cout << "  C = [" << C_tanks(0,0) << ", " << C_tanks(0,1) << "]" << std::endl;
    
    Matrix Wo_tanks = observability_matrix(A_tanks, C_tanks);
    std::cout << "\n  Observability Matrix:" << std::endl;
    std::cout << "  Wo = [C  ] = [" << Wo_tanks(0,0) << ", " << Wo_tanks(0,1) << "]" << std::endl;
    std::cout << "       [CA]   [" << Wo_tanks(1,0) << ", " << Wo_tanks(1,1) << "]" << std::endl;
    std::cout << "  det(Wo) = " << Wo_tanks.det() << std::endl;
    std::cout << "  rank(Wo) = " << Wo_tanks.rank() << std::endl;
    std::cout << "  ✓ System is OBSERVABLE" << std::endl;
    std::cout << "    → h₁ can be reconstructed from measurements of h₂" << std::endl;
    
    // Simulate tanks
    Matrix x0_tanks(2, 1);
    x0_tanks(0, 0) = 1.0;  // Initial level tank 1
    x0_tanks(1, 0) = 0.0;  // Initial level tank 2
    
    std::vector<double> t_tanks, y_tanks;
    std::vector<std::vector<double>> x_tanks;
    simulate_state_trajectory(A_tanks, B_tanks, C_tanks, x0_tanks, 0.5, 15.0, 0.01,
                              t_tanks, x_tanks, y_tanks);
    
    std::vector<double> h1, h2;
    for (const auto& x : x_tanks) {
        h1.push_back(x[0]);
        h2.push_back(x[1]);
    }
    
    subplot(2, 3, 4);
    plot(t_tanks, h1, "b-", opts({{"linewidth", "2"}, {"label", "h₁ (unmeasured)"}}));
    plot(t_tanks, h2, "r-", opts({{"linewidth", "2"}, {"label", "h₂ (measured)"}}));
    xlabel("Time (s)");
    ylabel("Level (m)");
    title("Two-Tank System Response");
    legend(true);
    grid(true);
    
    // ========================================================================
    // Example 5: Kalman Decomposition Concept
    // ========================================================================
    std::cout << "\n▶ Example 5: Controllability/Observability Summary" << std::endl;
    std::cout << "  ──────────────────────────────────────────────────" << std::endl;
    
    std::cout << R"(
  ┌────────────────────────────────────────────────────────────────┐
  │                    KALMAN DECOMPOSITION                         │
  ├────────────────────────────────────────────────────────────────┤
  │                                                                 │
  │  Any LTI system can be decomposed into 4 subsystems:           │
  │                                                                 │
  │  ┌─────────────────────────────────────────────────┐           │
  │  │   Controllable &        Controllable &          │           │
  │  │   Observable (co)       Unobservable (c̄o)       │           │
  │  │                                                 │           │
  │  │   These modes can be:   These modes can be      │           │
  │  │   - Controlled          moved but not seen     │           │
  │  │   - Estimated                                   │           │
  │  │   - Stabilized                                  │           │
  │  ├─────────────────────────────────────────────────┤           │
  │  │   Uncontrollable &      Uncontrollable &        │           │
  │  │   Observable (c̄o)       Unobservable (c̄ō)       │           │
  │  │                                                 │           │
  │  │   These modes can be    "Hidden modes"          │           │
  │  │   seen but not moved    Dangerous if unstable!  │           │
  │  └─────────────────────────────────────────────────┘           │
  │                                                                 │
  │  Transfer Function = Only controllable AND observable part     │
  │                                                                 │
  └────────────────────────────────────────────────────────────────┘
)" << std::endl;
    
    // ========================================================================
    // Example 6: Eigenvalue-Based Mode Analysis
    // ========================================================================
    std::cout << "\n▶ Example 6: Eigenvalue-Based Mode Analysis" << std::endl;
    std::cout << "  ───────────────────────────────────────────" << std::endl;
    
    // Second-order underdamped system
    double wn = 2.0, zeta = 0.2;
    Matrix A_second = {{0, 1}, {-wn*wn, -2*zeta*wn}};
    
    auto eigs_second = A_second.eigenvalues();
    std::cout << "\n  Second-order system: ωn = " << wn << ", ζ = " << zeta << std::endl;
    std::cout << "  A = [0, 1]" << std::endl;
    std::cout << "      [-ωn², -2ζωn]" << std::endl;
    
    for (size_t i = 0; i < eigs_second.size(); ++i) {
        double sigma = eigs_second[i].real();
        double omega_d = std::abs(eigs_second[i].imag());
        std::cout << "  λ" << (i+1) << " = " << std::fixed << std::setprecision(4) 
                  << sigma << " ± " << omega_d << "j" << std::endl;
    }
    
    double tau = -1.0 / eigs_second[0].real();
    double omega_d = std::abs(eigs_second[0].imag());
    std::cout << "\n  Time constant: τ = " << std::fixed << std::setprecision(4) << tau << " s" << std::endl;
    std::cout << "  Damped frequency: ωd = " << omega_d << " rad/s" << std::endl;
    std::cout << "  Period: T = " << 2*M_PI/omega_d << " s" << std::endl;
    
    // Simulate second-order system
    Matrix B_second = {{0}, {1}};
    Matrix C_second = {{1, 0}};
    Matrix x0_second(2, 1);
    x0_second(0, 0) = 0;  // Initial position
    x0_second(1, 0) = 0;  // Initial velocity
    
    std::vector<double> t_second, y_second;
    std::vector<std::vector<double>> x_second;
    simulate_state_trajectory(A_second, B_second, C_second, x0_second, 1.0, 10.0, 0.01,
                              t_second, x_second, y_second);
    
    subplot(2, 3, 5);
    plot(t_second, y_second, "b-", opts({{"linewidth", "2"}}));
    xlabel("Time (s)");
    ylabel("Position");
    title("Underdamped Mode Response (ζ = 0.2)");
    grid(true);
    
    // Plot eigenvalues in s-plane
    subplot(2, 3, 6);
    std::vector<double> eig_re, eig_im;
    for (const auto& e : eigs_second) {
        eig_re.push_back(e.real());
        eig_im.push_back(e.imag());
    }
    scatter(eig_re, eig_im, opts({{"s", "80"}, {"color", "red"}, {"marker", "x"}}));
    axhline(0, opts({{"color", "black"}, {"linewidth", "0.5"}}));
    axvline(0, opts({{"color", "black"}, {"linewidth", "0.5"}}));
    xlabel("Real (σ)");
    ylabel("Imaginary (jω)");
    title("Pole Locations in s-Plane");
    grid(true);
    
    // Add constant damping ratio line
    std::vector<double> line_re, line_im;
    for (double s = 0; s <= 3; s += 0.1) {
        line_re.push_back(-zeta * s);
        line_im.push_back(std::sqrt(1 - zeta*zeta) * s);
    }
    plot(line_re, line_im, "g--", opts({{"linewidth", "1"}, {"alpha", "0.5"}}));
    std::vector<double> line_im_neg;
    for (double im : line_im) line_im_neg.push_back(-im);
    plot(line_re, line_im_neg, "g--", opts({{"linewidth", "1"}, {"alpha", "0.5"}}));
    
    savefig("ch10_state_space_analysis.svg");
    
    std::cout << "\n════════════════════════════════════════════════════════════════" << std::endl;
    std::cout << "  Saved: ch10_state_space_analysis.svg" << std::endl;
    std::cout << "════════════════════════════════════════════════════════════════" << std::endl;
    
    return 0;
}
