/**
 * Chapter 16: Nonlinear Control - Phase Plane Analysis
 * 
 * Demonstrates:
 * - Phase plane analysis and trajectories
 * - Equilibrium point classification
 * - Linearization around equilibrium
 * - Van der Pol oscillator (limit cycle)
 * - Pendulum (multiple equilibria)
 * 
 * Physical Systems:
 * 1. Nonlinear Pendulum: θ̈ + (g/L)sin(θ) = 0
 * 2. Van der Pol Oscillator: ẍ - μ(1-x²)ẋ + x = 0
 * 
 * Build: g++ -std=c++14 -I "../include" ch16_nonlinear_control.cpp -o ch16_nonlinear_control.exe
 */

#include "cppplot.hpp"
#include <cmath>
#include <vector>
#include <iostream>
#include <iomanip>

using namespace cppplot;

// ============================================================================
// NONLINEAR PENDULUM
// ============================================================================

struct NonlinearPendulum {
    double g = 9.81;  // Gravity (m/s²)
    double L = 1.0;   // Length (m)
    double b = 0.1;   // Damping
    
    // State: x = [θ, ω] (angle, angular velocity)
    std::pair<double, double> dynamics(double theta, double omega) const {
        return {
            omega,
            -(g/L) * std::sin(theta) - b * omega
        };
    }
    
    // Linearized around θ = 0
    std::pair<double, double> linearizedDynamics(double theta, double omega) const {
        return {
            omega,
            -(g/L) * theta - b * omega
        };
    }
};

// ============================================================================
// VAN DER POL OSCILLATOR
// ============================================================================

struct VanDerPol {
    double mu = 1.0;  // Nonlinearity parameter
    
    // ẍ - μ(1-x²)ẋ + x = 0
    // State: [x, ẋ]
    std::pair<double, double> dynamics(double x, double v) const {
        return {
            v,
            mu * (1 - x*x) * v - x
        };
    }
};

// ============================================================================
// DUFFING OSCILLATOR (for comparison)
// ============================================================================

struct DuffingOscillator {
    double alpha = 1.0;   // Linear stiffness
    double beta = 0.2;    // Cubic stiffness
    double delta = 0.1;   // Damping
    
    // ẍ + δẋ + αx + βx³ = 0
    std::pair<double, double> dynamics(double x, double v) const {
        return {
            v,
            -delta * v - alpha * x - beta * x * x * x
        };
    }
};

// ============================================================================
// RK4 INTEGRATION
// ============================================================================

template<typename System>
void rk4Step(const System& sys, double& x1, double& x2, double dt) {
    auto [k1_1, k1_2] = sys.dynamics(x1, x2);
    auto [k2_1, k2_2] = sys.dynamics(x1 + 0.5*dt*k1_1, x2 + 0.5*dt*k1_2);
    auto [k3_1, k3_2] = sys.dynamics(x1 + 0.5*dt*k2_1, x2 + 0.5*dt*k2_2);
    auto [k4_1, k4_2] = sys.dynamics(x1 + dt*k3_1, x2 + dt*k3_2);
    
    x1 += dt * (k1_1 + 2*k2_1 + 2*k3_1 + k4_1) / 6.0;
    x2 += dt * (k1_2 + 2*k2_2 + 2*k3_2 + k4_2) / 6.0;
}

// ============================================================================
// PHASE PORTRAIT GENERATION
// ============================================================================

template<typename System>
void generateTrajectory(const System& sys, double x1_0, double x2_0,
                         double tFinal, double dt,
                         std::vector<double>& x1_traj,
                         std::vector<double>& x2_traj) {
    int nSteps = static_cast<int>(tFinal / dt);
    x1_traj.resize(nSteps);
    x2_traj.resize(nSteps);
    
    double x1 = x1_0, x2 = x2_0;
    
    for (int i = 0; i < nSteps; i++) {
        x1_traj[i] = x1;
        x2_traj[i] = x2;
        rk4Step(sys, x1, x2, dt);
    }
}

// Generate vector field
template<typename System>
void generateVectorField(const System& sys,
                          double x1_min, double x1_max, int n1,
                          double x2_min, double x2_max, int n2,
                          std::vector<double>& X1, std::vector<double>& X2,
                          std::vector<double>& U, std::vector<double>& V) {
    X1.clear(); X2.clear(); U.clear(); V.clear();
    
    for (int i = 0; i < n1; i++) {
        for (int j = 0; j < n2; j++) {
            double x1 = x1_min + i * (x1_max - x1_min) / (n1 - 1);
            double x2 = x2_min + j * (x2_max - x2_min) / (n2 - 1);
            
            auto [dx1, dx2] = sys.dynamics(x1, x2);
            
            // Normalize for visualization
            double mag = std::sqrt(dx1*dx1 + dx2*dx2);
            if (mag > 0.001) {
                dx1 /= mag;
                dx2 /= mag;
            }
            
            X1.push_back(x1);
            X2.push_back(x2);
            U.push_back(dx1 * 0.2);
            V.push_back(dx2 * 0.2);
        }
    }
}

// ============================================================================
// LYAPUNOV ANALYSIS
// ============================================================================

// Compute V = x² + ω² (energy-like function for pendulum)
std::vector<double> computeLyapunov(const std::vector<double>& x1,
                                     const std::vector<double>& x2,
                                     double g_over_L) {
    std::vector<double> V(x1.size());
    for (size_t i = 0; i < x1.size(); i++) {
        // Total energy: V = (1/2)ω² + g/L(1 - cos(θ))
        V[i] = 0.5 * x2[i] * x2[i] + g_over_L * (1 - std::cos(x1[i]));
    }
    return V;
}

// ============================================================================
// MAIN
// ============================================================================

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Chapter 16: Nonlinear Control" << std::endl;
    std::cout << "Phase Plane Analysis" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    // Systems
    NonlinearPendulum pendulum;
    VanDerPol vdp;
    vdp.mu = 1.5;
    DuffingOscillator duffing;
    
    std::cout << "System 1: Nonlinear Pendulum" << std::endl;
    std::cout << "  θ̈ + (g/L)sin(θ) + bω = 0" << std::endl;
    std::cout << "  Equilibria: θ = nπ (n = 0, ±1, ±2, ...)\n" << std::endl;
    
    std::cout << "System 2: Van der Pol Oscillator" << std::endl;
    std::cout << "  ẍ - μ(1-x²)ẋ + x = 0, μ = " << vdp.mu << std::endl;
    std::cout << "  Features: Limit cycle for μ > 0\n" << std::endl;
    
    // Simulation parameters
    double dt = 0.01;
    double tFinal = 20.0;
    
    // ---- Create 6-subplot figure ----
    Figure fig(1400, 900);
    
    // Subplot 1: Pendulum phase portrait
    auto& ax1 = fig.subplot(2, 3, 0);
    
    // Multiple initial conditions
    std::vector<std::pair<double, double>> pendulum_ics = {
        {0.5, 0}, {1.0, 0}, {1.5, 0}, {2.0, 0}, {2.5, 0},
        {3.0, 0}, {0, 2}, {0, 3}, {0, 4},
        {M_PI - 0.1, 0.1}, {M_PI + 0.1, -0.1}
    };
    
    std::vector<std::string> colors = {"blue", "green", "red", "purple", "orange",
                                        "brown", "teal", "magenta", "cyan",
                                        "olive", "pink"};
    
    for (size_t i = 0; i < pendulum_ics.size(); i++) {
        std::vector<double> theta, omega;
        generateTrajectory(pendulum, pendulum_ics[i].first, pendulum_ics[i].second,
                           tFinal, dt, theta, omega);
        ax1.plot(theta, omega, {{"color", colors[i % colors.size()]}, {"linewidth", "1"}});
    }
    
    // Mark equilibrium points
    ax1.scatter({0, M_PI, -M_PI}, {0, 0, 0}, 
               {{"color", "red"}, {"marker", "o"}, {"s", "100"}});
    
    ax1.set_xlabel("θ (rad)");
    ax1.set_ylabel("ω (rad/s)");
    ax1.set_title("Pendulum Phase Portrait");
    ax1.grid(true);
    
    // Subplot 2: Van der Pol phase portrait (limit cycle)
    auto& ax2 = fig.subplot(2, 3, 1);
    
    std::vector<std::pair<double, double>> vdp_ics = {
        {0.1, 0}, {0.5, 0}, {1.0, 0}, {3.0, 0}, {0, 0.1}, {0, 3.0}
    };
    
    for (size_t i = 0; i < vdp_ics.size(); i++) {
        std::vector<double> x, v;
        generateTrajectory(vdp, vdp_ics[i].first, vdp_ics[i].second,
                           30.0, dt, x, v);
        ax2.plot(x, v, {{"color", colors[i % colors.size()]}, {"linewidth", "1.5"}});
    }
    
    ax2.scatter({0}, {0}, {{"color", "red"}, {"marker", "x"}, {"s", "100"}});
    ax2.set_xlabel("x");
    ax2.set_ylabel("ẋ");
    ax2.set_title("Van der Pol: Limit Cycle (μ=1.5)");
    ax2.grid(true);
    
    // Subplot 3: Time response comparison (linear vs nonlinear pendulum)
    auto& ax3 = fig.subplot(2, 3, 2);
    
    // Large angle initial condition
    double theta0 = 2.5;  // ~143 degrees
    double omega0 = 0;
    
    std::vector<double> t_pend, theta_nl, theta_lin;
    int nSteps = static_cast<int>(10.0 / dt);
    t_pend.resize(nSteps);
    theta_nl.resize(nSteps);
    theta_lin.resize(nSteps);
    
    double th_nl = theta0, om_nl = omega0;
    double th_lin = theta0, om_lin = omega0;
    
    NonlinearPendulum pendulum_undamped;
    pendulum_undamped.b = 0;
    
    for (int i = 0; i < nSteps; i++) {
        t_pend[i] = i * dt;
        theta_nl[i] = th_nl;
        theta_lin[i] = th_lin;
        
        rk4Step(pendulum_undamped, th_nl, om_nl, dt);
        
        // Linearized: θ̈ + (g/L)θ = 0
        auto [dth, dom] = pendulum.linearizedDynamics(th_lin, om_lin);
        th_lin += om_lin * dt;
        om_lin += dom * dt;
    }
    
    ax3.plot(t_pend, theta_nl, {{"color", "blue"}, {"linewidth", "2"}, {"label", "Nonlinear"}});
    ax3.plot(t_pend, theta_lin, {{"color", "red"}, {"linestyle", "--"}, 
                                  {"linewidth", "2"}, {"label", "Linearized"}});
    ax3.set_xlabel("Time (s)");
    ax3.set_ylabel("θ (rad)");
    ax3.set_title("Linearization Error (θ₀=2.5 rad)");
    ax3.legend();
    ax3.grid(true);
    
    // Subplot 4: Lyapunov function evolution (pendulum with damping)
    auto& ax4 = fig.subplot(2, 3, 3);
    
    std::vector<double> theta_damp, omega_damp;
    generateTrajectory(pendulum, 2.0, 1.0, 15.0, dt, theta_damp, omega_damp);
    
    std::vector<double> t_lyap(theta_damp.size());
    for (size_t i = 0; i < t_lyap.size(); i++) t_lyap[i] = i * dt;
    
    auto V = computeLyapunov(theta_damp, omega_damp, pendulum.g / pendulum.L);
    
    ax4.plot(t_lyap, V, {{"color", "blue"}, {"linewidth", "2"}});
    ax4.set_xlabel("Time (s)");
    ax4.set_ylabel("V(x) (Energy)");
    ax4.set_title("Lyapunov Function Decreasing");
    ax4.grid(true);
    
    // Subplot 5: Vector field for Van der Pol
    auto& ax5 = fig.subplot(2, 3, 4);
    
    // Plot one limit cycle trajectory clearly
    std::vector<double> x_lc, v_lc;
    generateTrajectory(vdp, 0.1, 0.0, 50.0, dt, x_lc, v_lc);
    
    // Take only last portion (steady state limit cycle)
    size_t start_idx = x_lc.size() * 2 / 3;
    std::vector<double> x_lc_ss(x_lc.begin() + start_idx, x_lc.end());
    std::vector<double> v_lc_ss(v_lc.begin() + start_idx, v_lc.end());
    
    ax5.plot(x_lc_ss, v_lc_ss, {{"color", "blue"}, {"linewidth", "3"}});
    
    // Arrow direction
    size_t mid = x_lc_ss.size() / 2;
    ax5.scatter({x_lc_ss[mid]}, {v_lc_ss[mid]}, 
               {{"color", "red"}, {"marker", ">"}, {"s", "150"}});
    
    ax5.scatter({0}, {0}, {{"color", "red"}, {"marker", "x"}, {"s", "100"}});
    ax5.set_xlabel("x");
    ax5.set_ylabel("ẋ");
    ax5.set_title("Van der Pol Limit Cycle (Steady State)");
    ax5.grid(true);
    
    // Subplot 6: Equilibrium point classification
    auto& ax6 = fig.subplot(2, 3, 5);
    
    // Show different equilibrium types
    // Stable node: λ = -1, -2
    // Saddle: λ = -1, +1
    // Stable focus: λ = -0.5 ± 2j
    // Center: λ = ±2j
    
    std::vector<std::string> eq_types = {"Stable Node", "Saddle", "Stable Focus", "Center"};
    std::vector<std::string> eq_colors_list = {"green", "red", "blue", "purple"};
    
    // Simple systems for each type
    struct LinearSystem {
        double a11, a12, a21, a22;
        std::pair<double, double> dynamics(double x, double y) const {
            return {a11*x + a12*y, a21*x + a22*y};
        }
    };
    
    std::vector<LinearSystem> linear_systems = {
        {-1, 0, 0, -2},      // Stable node
        {1, 0, 0, -1},       // Saddle
        {-0.5, 2, -2, -0.5}, // Stable focus
        {0, 2, -2, 0}        // Center
    };
    
    // Plot eigenvalue locations
    std::vector<std::pair<double, double>> eigenvalues = {
        {-1.5, 0},           // Stable node (average position)
        {0, 0},              // Saddle (between)
        {-0.5, 0},           // Stable focus (real part)
        {0, 0}               // Center
    };
    
    // Draw regions
    ax6.axvline(0, {{"color", "black"}, {"linewidth", "2"}});
    ax6.axhline(0, {{"color", "gray"}, {"linestyle", "--"}, {"linewidth", "1"}});
    
    // Eigenvalue markers
    std::vector<double> ev_re = {-1.5, 1, -0.5, 0};
    std::vector<double> ev_im = {0, 0, 2, 2};
    
    for (size_t i = 0; i < eq_types.size(); i++) {
        ax6.scatter({ev_re[i]}, {ev_im[i]}, 
                   {{"color", eq_colors_list[i]}, {"marker", "o"}, {"s", "200"}});
        ax6.scatter({ev_re[i]}, {-ev_im[i]}, 
                   {{"color", eq_colors_list[i]}, {"marker", "o"}, {"s", "200"}});
    }
    
    ax6.set_xlabel("Re(λ)");
    ax6.set_ylabel("Im(λ)");
    ax6.set_title("Equilibrium Classification by Eigenvalues");
    ax6.grid(true);
    
    // Main title
    fig.suptitle("Nonlinear Control: Phase Plane & Stability Analysis", 16);
    
    // Save
    fig.savefig("ch16_nonlinear_control.svg");
    std::cout << "Figure saved: ch16_nonlinear_control.svg" << std::endl;
    
    // Summary
    std::cout << "\n=== Key Observations ===" << std::endl;
    std::cout << "1. Pendulum: Multiple equilibria (θ=0 stable, θ=π saddle)" << std::endl;
    std::cout << "2. Van der Pol: Self-sustained oscillation (limit cycle)" << std::endl;
    std::cout << "3. Linearization error grows with angle" << std::endl;
    std::cout << "4. Lyapunov function decreases → stability" << std::endl;
    
    std::cout << "\n✓ Phase plane reveals global nonlinear behavior!" << std::endl;
    std::cout << "✓ Linearization valid only near equilibrium!" << std::endl;
    
    return 0;
}
