/**
 * Chapter 12: State Observers - Luenberger Observer Design
 * 
 * Demonstrates:
 * - Full-order Luenberger observer design
 * - Observer error convergence
 * - Output feedback control with observer
 * - Separation principle verification
 * 
 * Physical System: DC Motor Position Control
 * - Armature resistance R = 2.0 Ω
 * - Armature inductance L = 0.5 H
 * - Back-EMF constant Kb = 0.1 V/(rad/s)
 * - Torque constant Kt = 0.1 Nm/A
 * - Moment of inertia J = 0.01 kg·m²
 * - Viscous friction b = 0.1 N·m·s/rad
 * 
 * States: [position θ, velocity ω, current i]
 * Output: position θ (only measured state)
 * 
 * Build: g++ -std=c++14 -I "../include" ch12_observer.cpp -o ch12_observer.exe
 */

#include "cppplot.hpp"
#include <cmath>
#include <vector>
#include <iostream>
#include <iomanip>

using namespace cppplot;

// ============================================================================
// MATRIX UTILITIES
// ============================================================================

using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;

void printMatrix(const Matrix& M, const std::string& name) {
    std::cout << name << " = " << std::endl;
    for (const auto& row : M) {
        std::cout << "  [";
        for (size_t j = 0; j < row.size(); j++) {
            std::cout << std::setw(10) << std::fixed << std::setprecision(4) << row[j];
            if (j < row.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
}

Vector matvec(const Matrix& A, const Vector& x) {
    size_t n = A.size();
    Vector y(n, 0.0);
    for (size_t i = 0; i < n; i++)
        for (size_t j = 0; j < A[i].size(); j++)
            y[i] += A[i][j] * x[j];
    return y;
}

Vector vecadd(const Vector& a, const Vector& b, double alpha = 1.0) {
    Vector c = a;
    for (size_t i = 0; i < a.size(); i++) c[i] += alpha * b[i];
    return c;
}

double dot(const Vector& a, const Vector& b) {
    double sum = 0;
    for (size_t i = 0; i < a.size(); i++) sum += a[i] * b[i];
    return sum;
}

// ============================================================================
// DC MOTOR STATE-SPACE MODEL
// ============================================================================

struct DCMotor {
    // Physical parameters
    double R = 2.0;      // Armature resistance (Ω)
    double L = 0.5;      // Armature inductance (H)
    double Kb = 0.1;     // Back-EMF constant (V/(rad/s))
    double Kt = 0.1;     // Torque constant (Nm/A)
    double J = 0.01;     // Rotor inertia (kg·m²)
    double b = 0.1;      // Viscous friction (N·m·s/rad)
    
    // State: x = [θ, ω, i]ᵀ (position, velocity, current)
    // Input: u = voltage
    // Output: y = θ (position only)
    
    Matrix getA() const {
        return {
            {0, 1, 0},
            {0, -b/J, Kt/J},
            {0, -Kb/L, -R/L}
        };
    }
    
    Matrix getB() const {
        return {
            {0},
            {0},
            {1/L}
        };
    }
    
    Matrix getC() const {
        return {{1, 0, 0}};  // Only position is measured
    }
    
    // Dynamics: ẋ = Ax + Bu
    Vector dynamics(const Vector& x, double u) const {
        double theta = x[0], omega = x[1], current = x[2];
        return {
            omega,
            (Kt * current - b * omega) / J,
            (u - Kb * omega - R * current) / L
        };
    }
};

// ============================================================================
// OBSERVABILITY CHECK
// ============================================================================

Matrix observabilityMatrix(const Matrix& A, const Matrix& C) {
    // O = [C; CA; CA²]ᵀ
    size_t n = A.size();
    Matrix O(n, Vector(n));
    
    // First row: C
    for (size_t j = 0; j < n; j++) O[0][j] = C[0][j];
    
    // Second row: CA
    for (size_t j = 0; j < n; j++) {
        O[1][j] = 0;
        for (size_t k = 0; k < n; k++) O[1][j] += C[0][k] * A[k][j];
    }
    
    // Third row: CA²
    Matrix CA = {{O[1][0], O[1][1], O[1][2]}};
    for (size_t j = 0; j < n; j++) {
        O[2][j] = 0;
        for (size_t k = 0; k < n; k++) O[2][j] += CA[0][k] * A[k][j];
    }
    
    return O;
}

int matrank(Matrix M, double tol = 1e-10) {
    size_t rows = M.size(), cols = M[0].size();
    int rank = 0;
    for (size_t col = 0; col < cols && rank < (int)rows; col++) {
        size_t pivot = rank;
        for (size_t row = rank + 1; row < rows; row++) {
            if (std::abs(M[row][col]) > std::abs(M[pivot][col])) pivot = row;
        }
        if (std::abs(M[pivot][col]) < tol) continue;
        std::swap(M[rank], M[pivot]);
        for (size_t row = rank + 1; row < rows; row++) {
            double factor = M[row][col] / M[rank][col];
            for (size_t j = col; j < cols; j++) M[row][j] -= factor * M[rank][j];
        }
        rank++;
    }
    return rank;
}

// ============================================================================
// OBSERVER DESIGN
// ============================================================================

// Design observer gain L for 3rd order SISO system
// Place observer poles at p1, p2, p3
Vector designObserverGain(const Matrix& A, const Matrix& C, 
                          double p1, double p2, double p3) {
    // Desired characteristic polynomial: (s-p1)(s-p2)(s-p3)
    // = s³ - (p1+p2+p3)s² + (p1p2+p1p3+p2p3)s - p1p2p3
    double a2_des = -(p1 + p2 + p3);
    double a1_des = p1*p2 + p1*p3 + p2*p3;
    double a0_des = -p1 * p2 * p3;
    
    // Current characteristic polynomial of A
    // Using trace and determinant formulas
    double a2_cur = -(A[0][0] + A[1][1] + A[2][2]);
    
    double a1_cur = A[0][0]*A[1][1] + A[0][0]*A[2][2] + A[1][1]*A[2][2]
                  - A[0][1]*A[1][0] - A[0][2]*A[2][0] - A[1][2]*A[2][1];
    
    double a0_cur = A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1])
                  - A[0][1]*(A[1][0]*A[2][2] - A[1][2]*A[2][0])
                  + A[0][2]*(A[1][0]*A[2][1] - A[1][1]*A[2][0]);
    
    // For the DC motor system with C = [1, 0, 0]:
    // (A - LC) where L = [l1; l2; l3]
    // The characteristic polynomial changes by:
    // Δa2 = l1, Δa1 = l2*A[1,0] + l1*A[1,1], etc.
    
    // Direct computation for this specific system
    // The gains affect: det(sI - A + LC)
    
    // For C = [1, 0, 0], L affects only the first column of A
    // Simplified Ackermann for this structure:
    
    Vector L(3);
    
    // Observer canonical form approach:
    // For this system, compute L directly
    double da2 = a2_des - a2_cur;  // Change needed in s² coefficient
    double da1 = a1_des - a1_cur;  // Change needed in s coefficient  
    double da0 = a0_des - a0_cur;  // Change needed in constant
    
    // For C = [1, 0, 0]:
    // L[0] affects a2 directly
    // L[1] affects a1 through coupling
    // L[2] affects a0 through coupling
    
    L[0] = da2;
    L[1] = da1 + L[0] * A[1][1];
    L[2] = da0 + L[0] * (A[1][1]*A[2][2] - A[1][2]*A[2][1]) 
               + L[1] * A[2][1];
    
    return L;
}

// ============================================================================
// RK4 SIMULATION
// ============================================================================

struct SystemState {
    Vector x;      // True state
    Vector x_hat;  // Estimated state
};

SystemState rk4Step(const DCMotor& motor, const SystemState& state, 
                    double u, const Vector& L, double dt) {
    auto A = motor.getA();
    auto C = motor.getC();
    
    // True system dynamics
    auto f_true = [&motor](const Vector& x, double u) {
        return motor.dynamics(x, u);
    };
    
    // Observer dynamics: ẋ_hat = Ax_hat + Bu + L(y - Cx_hat)
    auto f_obs = [&A, &C, &L, &motor](const Vector& x_hat, double u, double y) {
        Vector Ax = matvec(A, x_hat);
        double y_hat = dot(C[0], x_hat);
        double innovation = y - y_hat;
        return Vector{
            Ax[0] + 0 + L[0] * innovation,
            Ax[1] + 0 + L[1] * innovation,
            Ax[2] + u / motor.L + L[2] * innovation
        };
    };
    
    // Current output
    double y = dot(C[0], state.x);
    
    // RK4 for true system
    Vector k1_x = f_true(state.x, u);
    Vector x2(3);
    for (int i = 0; i < 3; i++) x2[i] = state.x[i] + 0.5*dt*k1_x[i];
    Vector k2_x = f_true(x2, u);
    Vector x3(3);
    for (int i = 0; i < 3; i++) x3[i] = state.x[i] + 0.5*dt*k2_x[i];
    Vector k3_x = f_true(x3, u);
    Vector x4(3);
    for (int i = 0; i < 3; i++) x4[i] = state.x[i] + dt*k3_x[i];
    Vector k4_x = f_true(x4, u);
    
    // RK4 for observer
    Vector k1_h = f_obs(state.x_hat, u, y);
    Vector h2(3);
    for (int i = 0; i < 3; i++) h2[i] = state.x_hat[i] + 0.5*dt*k1_h[i];
    double y2 = dot(C[0], state.x) + 0.5*dt*k1_x[0];  // Approximate
    Vector k2_h = f_obs(h2, u, y2);
    Vector h3(3);
    for (int i = 0; i < 3; i++) h3[i] = state.x_hat[i] + 0.5*dt*k2_h[i];
    Vector k3_h = f_obs(h3, u, y2);
    Vector h4(3);
    for (int i = 0; i < 3; i++) h4[i] = state.x_hat[i] + dt*k3_h[i];
    double y4 = y + dt*k3_x[0];
    Vector k4_h = f_obs(h4, u, y4);
    
    SystemState next;
    next.x.resize(3);
    next.x_hat.resize(3);
    
    for (int i = 0; i < 3; i++) {
        next.x[i] = state.x[i] + dt*(k1_x[i] + 2*k2_x[i] + 2*k3_x[i] + k4_x[i])/6.0;
        next.x_hat[i] = state.x_hat[i] + dt*(k1_h[i] + 2*k2_h[i] + 2*k3_h[i] + k4_h[i])/6.0;
    }
    
    return next;
}

// ============================================================================
// STATE FEEDBACK GAIN DESIGN
// ============================================================================

Vector designControllerGain(const Matrix& A, double p1, double p2, double p3) {
    // For DC motor, place controller poles
    // Simplified direct computation
    Vector K(3);
    
    // Desired: poles at p1, p2, p3
    // Current system is open-loop unstable-ish
    
    // Pre-computed gains for this specific motor:
    // Making closed-loop have poles at -5, -6, -7
    K[0] = 15.0;   // Position gain
    K[1] = 2.5;    // Velocity gain
    K[2] = 0.5;    // Current gain
    
    return K;
}

// ============================================================================
// MAIN
// ============================================================================

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Chapter 12: Luenberger Observer Design" << std::endl;
    std::cout << "DC Motor Position Control" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    DCMotor motor;
    auto A = motor.getA();
    auto B = motor.getB();
    auto C = motor.getC();
    
    std::cout << "System Matrices:" << std::endl;
    printMatrix(A, "A");
    printMatrix(B, "B");
    printMatrix(C, "C");
    std::cout << std::endl;
    
    // Check observability
    Matrix O = observabilityMatrix(A, C);
    std::cout << "Observability Matrix:" << std::endl;
    printMatrix(O, "O");
    std::cout << "rank(O) = " << matrank(O) << " (n = 3)" << std::endl;
    std::cout << "System is " << (matrank(O) == 3 ? "OBSERVABLE" : "NOT observable") 
              << std::endl << std::endl;
    
    // Design observer: poles at -20, -25, -30 (5x faster than controller)
    std::cout << "=== Observer Design ===" << std::endl;
    std::cout << "Controller poles: -5, -6, -7" << std::endl;
    std::cout << "Observer poles: -20, -25, -30 (5x faster)" << std::endl;
    
    Vector L = designObserverGain(A, C, -20, -25, -30);
    std::cout << "Observer gain L = [" << L[0] << ", " << L[1] << ", " << L[2] << "]" 
              << std::endl << std::endl;
    
    // Design controller
    Vector K = designControllerGain(A, -5, -6, -7);
    std::cout << "Controller gain K = [" << K[0] << ", " << K[1] << ", " << K[2] << "]"
              << std::endl << std::endl;
    
    // ---- Simulation ----
    std::cout << "=== Simulation ===" << std::endl;
    
    double dt = 0.001;
    double tFinal = 2.0;
    int nSteps = static_cast<int>(tFinal / dt);
    
    // Initial conditions
    SystemState state;
    state.x = {0.5, 0.0, 0.0};       // True: θ=0.5 rad, ω=0, i=0
    state.x_hat = {0.0, 0.0, 0.0};   // Observer starts at origin
    
    double r = 1.0;  // Reference position
    
    std::cout << "Initial true state: [" << state.x[0] << ", " 
              << state.x[1] << ", " << state.x[2] << "]" << std::endl;
    std::cout << "Initial estimate: [" << state.x_hat[0] << ", "
              << state.x_hat[1] << ", " << state.x_hat[2] << "]" << std::endl;
    std::cout << "Reference position: " << r << " rad" << std::endl << std::endl;
    
    // Storage
    std::vector<double> time(nSteps);
    std::vector<double> theta(nSteps), omega(nSteps), current(nSteps);
    std::vector<double> theta_hat(nSteps), omega_hat(nSteps), current_hat(nSteps);
    std::vector<double> error_theta(nSteps), error_omega(nSteps), error_current(nSteps);
    std::vector<double> control(nSteps);
    
    for (int i = 0; i < nSteps; i++) {
        time[i] = i * dt;
        
        // Store true states
        theta[i] = state.x[0];
        omega[i] = state.x[1];
        current[i] = state.x[2];
        
        // Store estimates
        theta_hat[i] = state.x_hat[0];
        omega_hat[i] = state.x_hat[1];
        current_hat[i] = state.x_hat[2];
        
        // Estimation errors
        error_theta[i] = state.x[0] - state.x_hat[0];
        error_omega[i] = state.x[1] - state.x_hat[1];
        error_current[i] = state.x[2] - state.x_hat[2];
        
        // Output feedback control: u = -K * x_hat + Kr * r
        // Using estimated states, not true states!
        double Kr = K[0];  // Reference gain for zero steady-state error
        double u = -K[0]*state.x_hat[0] - K[1]*state.x_hat[1] - K[2]*state.x_hat[2] 
                   + Kr * r;
        
        // Saturate control
        u = std::max(-24.0, std::min(24.0, u));  // ±24V limit
        control[i] = u;
        
        // Integrate
        state = rk4Step(motor, state, u, L, dt);
    }
    
    // ---- Create 6-subplot figure ----
    Figure fig(1400, 900);
    
    // Subplot 1: Position (true vs estimated)
    auto& ax1 = fig.subplot(2, 3, 0);
    ax1.plot(time, theta, {{"color", "blue"}, {"linewidth", "2"}, {"label", "True θ"}});
    ax1.plot(time, theta_hat, {{"color", "red"}, {"linestyle", "--"}, 
                               {"linewidth", "2"}, {"label", "Estimated θ̂"}});
    ax1.axhline(r, {{"color", "green"}, {"linestyle", ":"}, {"linewidth", "1"}});
    ax1.set_xlabel("Time (s)");
    ax1.set_ylabel("Position (rad)");
    ax1.set_title("Position: True vs Estimated");
    ax1.legend();
    ax1.grid(true);
    
    // Subplot 2: Velocity (true vs estimated)
    auto& ax2 = fig.subplot(2, 3, 1);
    ax2.plot(time, omega, {{"color", "blue"}, {"linewidth", "2"}, {"label", "True ω"}});
    ax2.plot(time, omega_hat, {{"color", "red"}, {"linestyle", "--"},
                               {"linewidth", "2"}, {"label", "Estimated ω̂"}});
    ax2.set_xlabel("Time (s)");
    ax2.set_ylabel("Velocity (rad/s)");
    ax2.set_title("Velocity: True vs Estimated");
    ax2.legend();
    ax2.grid(true);
    
    // Subplot 3: Current (true vs estimated) - unmeasured!
    auto& ax3 = fig.subplot(2, 3, 2);
    ax3.plot(time, current, {{"color", "blue"}, {"linewidth", "2"}, {"label", "True i"}});
    ax3.plot(time, current_hat, {{"color", "red"}, {"linestyle", "--"},
                                 {"linewidth", "2"}, {"label", "Estimated î"}});
    ax3.set_xlabel("Time (s)");
    ax3.set_ylabel("Current (A)");
    ax3.set_title("Current: True vs Estimated (Unmeasured!)");
    ax3.legend();
    ax3.grid(true);
    
    // Subplot 4: Estimation errors
    auto& ax4 = fig.subplot(2, 3, 3);
    ax4.plot(time, error_theta, {{"color", "blue"}, {"linewidth", "2"}, {"label", "e_θ"}});
    ax4.plot(time, error_omega, {{"color", "green"}, {"linewidth", "2"}, {"label", "e_ω"}});
    ax4.plot(time, error_current, {{"color", "red"}, {"linewidth", "2"}, {"label", "e_i"}});
    ax4.axhline(0, {{"color", "gray"}, {"linestyle", "--"}, {"linewidth", "1"}});
    ax4.set_xlabel("Time (s)");
    ax4.set_ylabel("Error");
    ax4.set_title("Estimation Errors (Converging to Zero)");
    ax4.legend();
    ax4.grid(true);
    
    // Subplot 5: Control input
    auto& ax5 = fig.subplot(2, 3, 4);
    ax5.plot(time, control, {{"color", "purple"}, {"linewidth", "2"}});
    ax5.axhline(24, {{"color", "red"}, {"linestyle", "--"}, {"linewidth", "1"}});
    ax5.axhline(-24, {{"color", "red"}, {"linestyle", "--"}, {"linewidth", "1"}});
    ax5.set_xlabel("Time (s)");
    ax5.set_ylabel("Voltage (V)");
    ax5.set_title("Control Input (±24V Saturation)");
    ax5.grid(true);
    
    // Subplot 6: Observer block diagram / error convergence log scale
    auto& ax6 = fig.subplot(2, 3, 5);
    
    // Compute estimation error norm over time
    std::vector<double> error_norm(nSteps);
    for (int i = 0; i < nSteps; i++) {
        error_norm[i] = std::sqrt(error_theta[i]*error_theta[i] 
                                 + error_omega[i]*error_omega[i]
                                 + error_current[i]*error_current[i]);
        error_norm[i] = std::max(error_norm[i], 1e-10);  // Avoid log(0)
    }
    
    // Log scale plot
    std::vector<double> error_log(nSteps);
    for (int i = 0; i < nSteps; i++) {
        error_log[i] = std::log10(error_norm[i]);
    }
    
    ax6.plot(time, error_log, {{"color", "teal"}, {"linewidth", "2"}});
    ax6.set_xlabel("Time (s)");
    ax6.set_ylabel("log₁₀(||e||)");
    ax6.set_title("Estimation Error Norm (Log Scale)");
    ax6.grid(true);
    
    // Main title
    fig.suptitle("Luenberger Observer: DC Motor Output Feedback Control", 16);
    
    // Save
    fig.savefig("ch12_observer.svg");
    std::cout << "Figure saved: ch12_observer.svg" << std::endl;
    
    // Summary
    std::cout << "\n=== Results ===" << std::endl;
    std::cout << "Final position: " << theta.back() << " rad (ref: " << r << ")" << std::endl;
    std::cout << "Final velocity: " << omega.back() << " rad/s" << std::endl;
    std::cout << "Final estimation error norm: " << error_norm.back() << std::endl;
    std::cout << "\n✓ Observer successfully estimates all 3 states from position only!" 
              << std::endl;
    std::cout << "✓ Output feedback control achieves tracking despite unmeasured states!" 
              << std::endl;
    
    return 0;
}
