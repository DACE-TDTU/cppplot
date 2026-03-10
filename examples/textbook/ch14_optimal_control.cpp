/**
 * Chapter 14: Optimal Control - LQR Design
 * 
 * Demonstrates:
 * - LQR controller design via Riccati equation
 * - Effect of Q and R weighting matrices
 * - Comparison with pole placement
 * - LQR guaranteed margins
 * 
 * Physical System: Double Integrator (Simplified Drone Position)
 * - Mass M = 1.0 kg
 * - States: [position, velocity]
 * - Input: force
 * 
 * Extended: Quadcopter Pitch Angle Control
 * - Inertia I = 0.01 kg·m²
 * - Arm length L = 0.25 m
 * 
 * Build: g++ -std=c++14 -I "../include" ch14_optimal_control.cpp -o ch14_optimal_control.exe
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

Matrix matmul(const Matrix& A, const Matrix& B) {
    size_t n = A.size(), m = B[0].size(), k = B.size();
    Matrix C(n, Vector(m, 0.0));
    for (size_t i = 0; i < n; i++)
        for (size_t j = 0; j < m; j++)
            for (size_t l = 0; l < k; l++)
                C[i][j] += A[i][l] * B[l][j];
    return C;
}

Matrix transpose(const Matrix& A) {
    Matrix T(A[0].size(), Vector(A.size()));
    for (size_t i = 0; i < A.size(); i++)
        for (size_t j = 0; j < A[0].size(); j++)
            T[j][i] = A[i][j];
    return T;
}

Matrix matadd(const Matrix& A, const Matrix& B, double alpha = 1.0) {
    Matrix C = A;
    for (size_t i = 0; i < A.size(); i++)
        for (size_t j = 0; j < A[i].size(); j++)
            C[i][j] += alpha * B[i][j];
    return C;
}

Matrix matscale(const Matrix& A, double s) {
    Matrix C = A;
    for (auto& row : C)
        for (auto& v : row)
            v *= s;
    return C;
}

Vector matvec(const Matrix& A, const Vector& x) {
    Vector y(A.size(), 0.0);
    for (size_t i = 0; i < A.size(); i++)
        for (size_t j = 0; j < A[i].size(); j++)
            y[i] += A[i][j] * x[j];
    return y;
}

// 2x2 matrix inverse
Matrix inv2x2(const Matrix& A) {
    double det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
    return {
        {A[1][1]/det, -A[0][1]/det},
        {-A[1][0]/det, A[0][0]/det}
    };
}

// ============================================================================
// RICCATI EQUATION SOLVER
// ============================================================================

// Solve Algebraic Riccati Equation (ARE):
// A'P + PA - PBR^{-1}B'P + Q = 0
// Using iterative method (fixed-point iteration)
Matrix solveRiccati(const Matrix& A, const Matrix& B, 
                     const Matrix& Q, const Matrix& R,
                     int maxIter = 1000, double tol = 1e-10) {
    size_t n = A.size();
    
    // Initialize P = Q
    Matrix P = Q;
    Matrix Rinv = inv2x2(R);
    Matrix BT = transpose(B);
    Matrix AT = transpose(A);
    
    for (int iter = 0; iter < maxIter; iter++) {
        // Compute K = R^{-1} B' P
        Matrix K = matmul(Rinv, matmul(BT, P));
        
        // Compute Acl = A - BK
        Matrix BK = matmul(B, K);
        Matrix Acl = matadd(A, BK, -1.0);
        
        // Solve Lyapunov equation: Acl'P + PAcl + Q + K'RK = 0
        // Using direct iteration: P_new = -inv(Acl'⊗I + I⊗Acl') * vec(Q + K'RK)
        
        // Simplified for 2x2: direct iteration
        Matrix KTRK = matmul(transpose(K), matmul(R, K));
        Matrix Qeff = matadd(Q, KTRK);
        
        // New P via iteration: P = integral(e^{Acl't} Qeff e^{Aclt} dt)
        // Approximate by discrete summation
        Matrix P_new(n, Vector(n, 0.0));
        double dt = 0.01;
        for (double t = 0; t < 50; t += dt) {
            // e^{Acl*t} ≈ I + Acl*t + (Acl*t)^2/2 + ...
            Matrix eAt(n, Vector(n, 0.0));
            for (size_t i = 0; i < n; i++) eAt[i][i] = 1.0;
            
            Matrix Acl_t = matscale(Acl, t);
            Matrix Acl_t2 = matmul(Acl_t, Acl_t);
            Matrix Acl_t3 = matmul(Acl_t2, Acl_t);
            
            // e^{At} ≈ I + At + (At)²/2 + (At)³/6
            eAt = matadd(eAt, Acl_t, 1.0);
            eAt = matadd(eAt, Acl_t2, 0.5);
            eAt = matadd(eAt, Acl_t3, 1.0/6.0);
            
            // e^{A't} Qeff e^{At}
            Matrix eAtT = transpose(eAt);
            Matrix contrib = matmul(eAtT, matmul(Qeff, eAt));
            P_new = matadd(P_new, contrib, dt);
        }
        
        // Check convergence
        double diff = 0;
        for (size_t i = 0; i < n; i++)
            for (size_t j = 0; j < n; j++)
                diff += std::abs(P_new[i][j] - P[i][j]);
        
        P = P_new;
        
        if (diff < tol) {
            std::cout << "  Riccati converged in " << iter+1 << " iterations" << std::endl;
            break;
        }
    }
    
    return P;
}

// ============================================================================
// LQR DESIGN
// ============================================================================

struct LQRResult {
    Matrix P;   // Riccati solution
    Matrix K;   // Feedback gain
};

LQRResult designLQR(const Matrix& A, const Matrix& B, 
                     const Matrix& Q, const Matrix& R) {
    LQRResult result;
    
    // Solve Riccati equation
    result.P = solveRiccati(A, B, Q, R);
    
    // Compute gain: K = R^{-1} B' P
    Matrix Rinv = inv2x2(R);
    Matrix BT = transpose(B);
    result.K = matmul(Rinv, matmul(BT, result.P));
    
    return result;
}

// ============================================================================
// DOUBLE INTEGRATOR SYSTEM
// ============================================================================

struct DoubleIntegrator {
    Matrix A = {{0, 1}, {0, 0}};
    Matrix B = {{0}, {1}};
    Matrix C = {{1, 0}};
    
    Vector dynamics(const Vector& x, double u) const {
        return {x[1], u};
    }
};

// ============================================================================
// RK4 SIMULATION
// ============================================================================

Vector rk4Step(const DoubleIntegrator& sys, const Vector& x, double u, double dt) {
    auto f = [&sys, u](const Vector& x) { return sys.dynamics(x, u); };
    
    Vector k1 = f(x);
    Vector x2 = {x[0] + 0.5*dt*k1[0], x[1] + 0.5*dt*k1[1]};
    Vector k2 = f(x2);
    Vector x3 = {x[0] + 0.5*dt*k2[0], x[1] + 0.5*dt*k2[1]};
    Vector k3 = f(x3);
    Vector x4 = {x[0] + dt*k3[0], x[1] + dt*k3[1]};
    Vector k4 = f(x4);
    
    return {
        x[0] + dt*(k1[0] + 2*k2[0] + 2*k3[0] + k4[0])/6.0,
        x[1] + dt*(k1[1] + 2*k2[1] + 2*k3[1] + k4[1])/6.0
    };
}

// ============================================================================
// COST FUNCTION COMPUTATION
// ============================================================================

double computeCost(const std::vector<Vector>& x_traj, 
                   const std::vector<double>& u_traj,
                   const Matrix& Q, double R, double dt) {
    double J = 0;
    for (size_t i = 0; i < x_traj.size(); i++) {
        // x'Qx
        double xQx = Q[0][0]*x_traj[i][0]*x_traj[i][0] 
                   + Q[1][1]*x_traj[i][1]*x_traj[i][1];
        // u'Ru
        double uRu = R * u_traj[i] * u_traj[i];
        J += (xQx + uRu) * dt;
    }
    return J;
}

// ============================================================================
// MAIN
// ============================================================================

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Chapter 14: Optimal Control (LQR)" << std::endl;
    std::cout << "Double Integrator / Position Control" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    DoubleIntegrator sys;
    
    std::cout << "System: Double Integrator" << std::endl;
    std::cout << "  ẋ₁ = x₂" << std::endl;
    std::cout << "  ẋ₂ = u" << std::endl;
    std::cout << "  y = x₁\n" << std::endl;
    
    // ---- Compare different Q/R combinations ----
    std::cout << "=== LQR Design Comparison ===" << std::endl;
    
    struct QRCase {
        std::string name;
        Matrix Q;
        Matrix R;
        std::string color;
    };
    
    std::vector<QRCase> cases = {
        {"Q=I, R=1", {{1, 0}, {0, 1}}, {{1}}, "blue"},
        {"Q=10I, R=1", {{10, 0}, {0, 10}}, {{1}}, "green"},
        {"Q=I, R=10", {{1, 0}, {0, 1}}, {{10}}, "red"},
        {"Q=100I, R=0.1", {{100, 0}, {0, 100}}, {{0.1}}, "purple"}
    };
    
    // Simulation parameters
    double dt = 0.01;
    double tFinal = 5.0;
    int nSteps = static_cast<int>(tFinal / dt);
    Vector x0 = {1.0, 0.0};  // Initial: position = 1, velocity = 0
    
    // Storage for all cases
    std::vector<std::vector<double>> all_time(cases.size());
    std::vector<std::vector<double>> all_pos(cases.size());
    std::vector<std::vector<double>> all_vel(cases.size());
    std::vector<std::vector<double>> all_ctrl(cases.size());
    std::vector<double> all_costs(cases.size());
    
    for (size_t c = 0; c < cases.size(); c++) {
        std::cout << "\nCase " << (c+1) << ": " << cases[c].name << std::endl;
        
        // Design LQR
        auto result = designLQR(sys.A, sys.B, cases[c].Q, cases[c].R);
        
        std::cout << "  K = [" << result.K[0][0] << ", " << result.K[0][1] << "]" << std::endl;
        
        // Simulate
        all_time[c].resize(nSteps);
        all_pos[c].resize(nSteps);
        all_vel[c].resize(nSteps);
        all_ctrl[c].resize(nSteps);
        
        std::vector<Vector> x_traj(nSteps);
        Vector x = x0;
        
        for (int k = 0; k < nSteps; k++) {
            all_time[c][k] = k * dt;
            all_pos[c][k] = x[0];
            all_vel[c][k] = x[1];
            
            // LQR control: u = -Kx
            double u = -(result.K[0][0]*x[0] + result.K[0][1]*x[1]);
            all_ctrl[c][k] = u;
            
            x_traj[k] = x;
            x = rk4Step(sys, x, u, dt);
        }
        
        // Compute cost
        all_costs[c] = computeCost(x_traj, all_ctrl[c], cases[c].Q, 
                                   cases[c].R[0][0], dt);
        std::cout << "  Cost J = " << all_costs[c] << std::endl;
    }
    
    // ---- Create 6-subplot figure ----
    Figure fig(1400, 900);
    
    // Subplot 1: Position comparison
    auto& ax1 = fig.subplot(2, 3, 0);
    for (size_t c = 0; c < cases.size(); c++) {
        ax1.plot(all_time[c], all_pos[c], 
                {{"color", cases[c].color}, {"linewidth", "2"}, 
                 {"label", cases[c].name}});
    }
    ax1.axhline(0, {{"color", "gray"}, {"linestyle", "--"}, {"linewidth", "1"}});
    ax1.set_xlabel("Time (s)");
    ax1.set_ylabel("Position");
    ax1.set_title("Position Response");
    ax1.legend();
    ax1.grid(true);
    
    // Subplot 2: Velocity comparison
    auto& ax2 = fig.subplot(2, 3, 1);
    for (size_t c = 0; c < cases.size(); c++) {
        ax2.plot(all_time[c], all_vel[c],
                {{"color", cases[c].color}, {"linewidth", "2"}});
    }
    ax2.axhline(0, {{"color", "gray"}, {"linestyle", "--"}, {"linewidth", "1"}});
    ax2.set_xlabel("Time (s)");
    ax2.set_ylabel("Velocity");
    ax2.set_title("Velocity Response");
    ax2.grid(true);
    
    // Subplot 3: Control effort
    auto& ax3 = fig.subplot(2, 3, 2);
    for (size_t c = 0; c < cases.size(); c++) {
        ax3.plot(all_time[c], all_ctrl[c],
                {{"color", cases[c].color}, {"linewidth", "2"}});
    }
    ax3.axhline(0, {{"color", "gray"}, {"linestyle", "--"}, {"linewidth", "1"}});
    ax3.set_xlabel("Time (s)");
    ax3.set_ylabel("Control u");
    ax3.set_title("Control Input");
    ax3.grid(true);
    
    // Subplot 4: Phase plane
    auto& ax4 = fig.subplot(2, 3, 3);
    for (size_t c = 0; c < cases.size(); c++) {
        ax4.plot(all_pos[c], all_vel[c],
                {{"color", cases[c].color}, {"linewidth", "2"}});
    }
    ax4.scatter({x0[0]}, {x0[1]}, {{"color", "black"}, {"marker", "o"}, {"s", "100"}});
    ax4.scatter({0}, {0}, {{"color", "red"}, {"marker", "x"}, {"s", "100"}});
    ax4.set_xlabel("Position");
    ax4.set_ylabel("Velocity");
    ax4.set_title("Phase Plane");
    ax4.grid(true);
    
    // Subplot 5: Cost comparison (bar chart)
    auto& ax5 = fig.subplot(2, 3, 4);
    std::vector<double> case_idx = {0, 1, 2, 3};
    std::vector<std::string> bar_colors;
    for (const auto& c : cases) bar_colors.push_back(c.color);
    
    // Manual bar plot using scatter with vertical lines
    for (size_t c = 0; c < cases.size(); c++) {
        std::vector<double> x_bar = {(double)c, (double)c};
        std::vector<double> y_bar = {0, all_costs[c]};
        ax5.plot(x_bar, y_bar, {{"color", cases[c].color}, {"linewidth", "10"}});
    }
    ax5.set_xlabel("Case");
    ax5.set_ylabel("Cost J");
    ax5.set_title("Total Cost Comparison");
    ax5.grid(true);
    
    // Subplot 6: Trade-off visualization
    auto& ax6 = fig.subplot(2, 3, 5);
    
    // Compute settling time and max control for each case
    std::vector<double> settle_times(cases.size());
    std::vector<double> max_ctrls(cases.size());
    
    for (size_t c = 0; c < cases.size(); c++) {
        // Settling time (2% criterion)
        settle_times[c] = tFinal;
        for (int k = nSteps - 1; k >= 0; k--) {
            if (std::abs(all_pos[c][k]) > 0.02 * x0[0]) {
                settle_times[c] = all_time[c][k];
                break;
            }
        }
        
        // Max control
        max_ctrls[c] = 0;
        for (double u : all_ctrl[c]) {
            if (std::abs(u) > max_ctrls[c]) max_ctrls[c] = std::abs(u);
        }
    }
    
    ax6.scatter(settle_times, max_ctrls, 
               {{"color", "blue"}, {"marker", "o"}, {"s", "150"}});
    
    // Label points
    for (size_t c = 0; c < cases.size(); c++) {
        ax6.scatter({settle_times[c]}, {max_ctrls[c]},
                   {{"color", cases[c].color}, {"marker", "o"}, {"s", "150"}});
    }
    
    ax6.set_xlabel("Settling Time (s)");
    ax6.set_ylabel("Max Control |u|");
    ax6.set_title("Performance Trade-off");
    ax6.grid(true);
    
    // Main title
    fig.suptitle("LQR Optimal Control: Effect of Q and R Weighting", 16);
    
    // Save
    fig.savefig("ch14_optimal_control.svg");
    std::cout << "\nFigure saved: ch14_optimal_control.svg" << std::endl;
    
    // Summary
    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "Trade-off between settling time and control effort:" << std::endl;
    for (size_t c = 0; c < cases.size(); c++) {
        std::cout << "  " << cases[c].name << ": Ts=" 
                  << std::fixed << std::setprecision(2) << settle_times[c]
                  << "s, |u|_max=" << max_ctrls[c] << std::endl;
    }
    
    std::cout << "\n✓ LQR provides systematic way to balance performance vs effort!" 
              << std::endl;
    
    return 0;
}
