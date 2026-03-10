/**
 * @file ch09_state_space_intro.cpp
 * @brief Chapter 9: State-Space Representation Fundamentals
 * 
 * Learning Outcomes:
 * - Define state variables and state equations
 * - Convert between transfer function and state-space
 * - Visualize state trajectories in phase plane
 * 
 * Physical System: Mass-Spring-Damper & Inverted Pendulum
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>

using namespace cppplot;
using namespace cppplot::control;

// Simple 4th order Runge-Kutta integrator for state-space systems
void simulate_ss(
    const std::vector<std::vector<double>>& A,
    const std::vector<double>& B,
    const std::vector<double>& C,
    const std::vector<double>& x0,
    double u_input,
    double t_final,
    double dt,
    std::vector<double>& t_out,
    std::vector<std::vector<double>>& x_out,
    std::vector<double>& y_out
) {
    int n = A.size();
    std::vector<double> x = x0;
    
    t_out.clear();
    x_out.clear();
    y_out.clear();
    
    for (double t = 0; t <= t_final; t += dt) {
        // Store current state
        t_out.push_back(t);
        x_out.push_back(x);
        
        // Compute output y = Cx + Du (D=0 assumed)
        double y = 0;
        for (int i = 0; i < n; ++i) {
            y += C[i] * x[i];
        }
        y_out.push_back(y);
        
        // RK4 integration
        auto xdot = [&](const std::vector<double>& state, double input) {
            std::vector<double> dx(n);
            for (int i = 0; i < n; ++i) {
                dx[i] = B[i] * input;
                for (int j = 0; j < n; ++j) {
                    dx[i] += A[i][j] * state[j];
                }
            }
            return dx;
        };
        
        auto add_vec = [](const std::vector<double>& a, const std::vector<double>& b, double scale) {
            std::vector<double> result(a.size());
            for (size_t i = 0; i < a.size(); ++i) {
                result[i] = a[i] + scale * b[i];
            }
            return result;
        };
        
        auto k1 = xdot(x, u_input);
        auto k2 = xdot(add_vec(x, k1, dt/2), u_input);
        auto k3 = xdot(add_vec(x, k2, dt/2), u_input);
        auto k4 = xdot(add_vec(x, k3, dt), u_input);
        
        for (int i = 0; i < n; ++i) {
            x[i] += dt/6 * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
        }
    }
}

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║     Chapter 9: State-Space Introduction                      ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    figure(1600, 1200);
    
    // ========================================================================
    // Example 1: Mass-Spring-Damper System
    // ========================================================================
    std::cout << "\n▶ Example 1: Mass-Spring-Damper System" << std::endl;
    std::cout << "  ─────────────────────────────────────────" << std::endl;
    
    // Physical parameters
    double m = 1.0;    // kg
    double k = 4.0;    // N/m
    double b = 0.5;    // N·s/m
    
    std::cout << "  m = " << m << " kg, k = " << k << " N/m, b = " << b << " N·s/m" << std::endl;
    
    // State-space representation
    // x1 = position, x2 = velocity
    // dx1/dt = x2
    // dx2/dt = -k/m*x1 - b/m*x2 + 1/m*F
    
    std::vector<std::vector<double>> A_msd = {
        {0, 1},
        {-k/m, -b/m}
    };
    std::vector<double> B_msd = {0, 1/m};
    std::vector<double> C_msd = {1, 0};  // Output = position
    
    std::cout << "\n  State-Space Matrices:" << std::endl;
    std::cout << "  A = [ " << A_msd[0][0] << ", " << A_msd[0][1] << " ]" << std::endl;
    std::cout << "      [ " << A_msd[1][0] << ", " << A_msd[1][1] << " ]" << std::endl;
    std::cout << "  B = [ " << B_msd[0] << " ]" << std::endl;
    std::cout << "      [ " << B_msd[1] << " ]" << std::endl;
    std::cout << "  C = [ " << C_msd[0] << ", " << C_msd[1] << " ]" << std::endl;
    
    // Calculate eigenvalues (natural frequencies)
    // det(sI - A) = s² + (b/m)s + k/m = 0
    double wn = std::sqrt(k/m);
    double zeta = b / (2*std::sqrt(k*m));
    
    std::cout << "\n  Natural frequency: ωn = " << wn << " rad/s" << std::endl;
    std::cout << "  Damping ratio: ζ = " << zeta << std::endl;
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 1: State Trajectory (Phase Plane)
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 1);
    
    // Simulate from different initial conditions
    std::vector<std::vector<double>> ics = {{1, 0}, {0, 2}, {-1, 1}, {1, -1}};
    std::vector<std::string> colors = {"#E74C3C", "#3498DB", "#2ECC71", "#9B59B6"};
    
    for (size_t i = 0; i < ics.size(); ++i) {
        std::vector<double> t, y;
        std::vector<std::vector<double>> x;
        
        simulate_ss(A_msd, B_msd, C_msd, ics[i], 0, 10, 0.01, t, x, y);
        
        std::vector<double> x1, x2;
        for (auto& state : x) {
            x1.push_back(state[0]);
            x2.push_back(state[1]);
        }
        
        std::ostringstream label;
        label << "x0=[" << ics[i][0] << "," << ics[i][1] << "]";
        plot(x1, x2, "-", {{"color", colors[i]}, {"linewidth", "1.5"}, {"label", label.str()}});
        
        // Mark start point
        scatter({ics[i][0]}, {ics[i][1]}, {{"color", colors[i]}, {"s", "80"}, {"marker", "o"}});
    }
    
    // Equilibrium point
    scatter({0}, {0}, {{"color", "black"}, {"s", "150"}, {"marker", "x"}, {"label", "Equilibrium"}});
    
    xlabel("Position x₁");
    ylabel("Velocity x₂");
    title("(a) Phase Plane Trajectories");
    legend({{"fontsize", "6"}, {"loc", "upper right"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 2: Step Response from State-Space
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 2);
    
    std::vector<double> t_step, y_step;
    std::vector<std::vector<double>> x_step;
    
    simulate_ss(A_msd, B_msd, C_msd, {0, 0}, 1.0, 15, 0.01, t_step, x_step, y_step);
    
    plot(t_step, y_step, "b-", {{"linewidth", "2"}, {"label", "Position (y)"}});
    
    // Also plot velocity
    std::vector<double> vel;
    for (auto& state : x_step) vel.push_back(state[1]);
    plot(t_step, vel, "r--", {{"linewidth", "1.5"}, {"label", "Velocity (x₂)"}});
    
    axhline(1/k, {{"color", "gray"}, {"linestyle", ":"}, {"alpha", "0.5"}, {"label", "Steady-state"}});
    
    xlabel("Time [s]");
    ylabel("Response");
    title("(b) Step Response");
    legend({{"fontsize", "7"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 3: State Variables vs Time
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 3);
    
    std::vector<double> pos, vel_plot;
    for (auto& state : x_step) {
        pos.push_back(state[0]);
        vel_plot.push_back(state[1]);
    }
    
    plot(t_step, pos, "b-", {{"linewidth", "2"}, {"label", "x₁ (position)"}});
    plot(t_step, vel_plot, "r-", {{"linewidth", "2"}, {"label", "x₂ (velocity)"}});
    
    xlabel("Time [s]");
    ylabel("State Variables");
    title("(c) State Variables vs Time");
    legend({{"fontsize", "8"}});
    grid(true);
    
    // ========================================================================
    // Example 2: Inverted Pendulum
    // ========================================================================
    std::cout << "\n▶ Example 2: Inverted Pendulum (Linearized)" << std::endl;
    std::cout << "  ──────────────────────────────────────────────" << std::endl;
    
    // Parameters
    double M = 1.0;     // Cart mass [kg]
    double mp = 0.1;    // Pendulum mass [kg]
    double l = 0.5;     // Pendulum length [m]
    double g = 9.81;    // Gravity [m/s²]
    
    std::cout << "  M = " << M << " kg (cart)" << std::endl;
    std::cout << "  m = " << mp << " kg (pendulum)" << std::endl;
    std::cout << "  l = " << l << " m (length)" << std::endl;
    
    // Linearized state-space around θ = 0
    // x = [cart_pos, cart_vel, angle, angular_vel]
    
    double denom = M + mp;
    std::vector<std::vector<double>> A_pend = {
        {0, 1, 0, 0},
        {0, 0, -mp*g/denom, 0},
        {0, 0, 0, 1},
        {0, 0, (M+mp)*g/(denom*l), 0}
    };
    std::vector<double> B_pend = {0, 1/denom, 0, -1/(denom*l)};
    std::vector<double> C_angle = {0, 0, 1, 0};  // Output = angle
    
    std::cout << "\n  State: x = [cart_pos, cart_vel, angle, angular_vel]ᵀ" << std::endl;
    std::cout << "  This is an UNSTABLE system (positive eigenvalue)" << std::endl;
    
    // Calculate eigenvalues
    // The characteristic polynomial for this linearized pendulum has a positive root
    double lambda_unstable = std::sqrt((M+mp)*g/(denom*l));
    std::cout << "  Unstable eigenvalue: λ ≈ " << lambda_unstable << std::endl;
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 4: Pendulum Phase Plane (θ vs θ_dot)
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 4);
    
    // Plot vector field for pendulum angle dynamics
    std::vector<double> theta_range, theta_dot_range;
    for (double th = -0.3; th <= 0.3; th += 0.05) {
        for (double thd = -1; thd <= 1; thd += 0.2) {
            theta_range.push_back(th);
            theta_dot_range.push_back(thd);
        }
    }
    
    // Simulate pendulum without control (will diverge)
    std::vector<std::vector<double>> ics_pend = {{0, 0, 0.1, 0}, {0, 0, -0.1, 0}, {0, 0, 0.05, 0.2}};
    
    for (size_t i = 0; i < ics_pend.size(); ++i) {
        std::vector<double> t, y;
        std::vector<std::vector<double>> x;
        
        simulate_ss(A_pend, B_pend, C_angle, ics_pend[i], 0, 2, 0.01, t, x, y);
        
        std::vector<double> theta, theta_dot;
        for (auto& state : x) {
            theta.push_back(state[2]);
            theta_dot.push_back(state[3]);
        }
        
        plot(theta, theta_dot, "-", {{"color", colors[i]}, {"linewidth", "1.5"}});
        scatter({ics_pend[i][2]}, {ics_pend[i][3]}, {{"color", colors[i]}, {"s", "60"}, {"marker", "o"}});
    }
    
    scatter({0}, {0}, {{"color", "red"}, {"s", "150"}, {"marker", "x"}, {"label", "Unstable eq."}});
    
    xlabel("Angle θ [rad]");
    ylabel("Angular velocity θ̇ [rad/s]");
    title("(d) Pendulum Phase Plane (Unstable)");
    legend({{"fontsize", "7"}});
    grid(true);
    xlim(-0.5, 0.5);
    ylim(-3, 3);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 5: Pendulum Angle vs Time (Open-Loop Unstable)
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 5);
    
    for (size_t i = 0; i < ics_pend.size(); ++i) {
        std::vector<double> t, y;
        std::vector<std::vector<double>> x;
        
        simulate_ss(A_pend, B_pend, C_angle, ics_pend[i], 0, 2, 0.01, t, x, y);
        
        // Clip for visualization
        std::vector<double> y_clip;
        for (double val : y) {
            y_clip.push_back(std::max(-1.0, std::min(1.0, val)));
        }
        
        std::ostringstream label;
        label << "θ₀ = " << std::fixed << std::setprecision(2) << ics_pend[i][2] << " rad";
        plot(t, y_clip, "-", {{"color", colors[i]}, {"linewidth", "2"}, {"label", label.str()}});
    }
    
    axhline(0, {{"color", "gray"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    
    xlabel("Time [s]");
    ylabel("Angle θ [rad]");
    title("(e) Pendulum Angle (Uncontrolled)");
    legend({{"fontsize", "7"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 6: Transfer Function vs State-Space Comparison
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 6);
    
    // Compare TF and SS representations for mass-spring-damper
    // G(s) = 1/m / (s² + b/m*s + k/m)
    TransferFunction G_msd({1/m}, {1, b/m, k/m});
    
    auto [t_tf, y_tf] = step(G_msd, 15.0);
    
    plot(t_tf, y_tf, "b-", {{"linewidth", "2"}, {"label", "Transfer Function"}});
    plot(t_step, y_step, "r--", {{"linewidth", "2"}, {"label", "State-Space"}});
    
    xlabel("Time [s]");
    ylabel("Response");
    title("(f) TF vs SS: Same Response");
    legend({{"fontsize", "8"}});
    grid(true);
    
    savefig("ch09_state_space_intro.svg");
    std::cout << "\n✓ Saved ch09_state_space_intro.svg" << std::endl;
    
    // ========================================================================
    // Summary
    // ========================================================================
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║              STATE-SPACE INTRODUCTION SUMMARY                ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  State Equations:                                            ║" << std::endl;
    std::cout << "║  ẋ = Ax + Bu   (state equation)                              ║" << std::endl;
    std::cout << "║  y = Cx + Du   (output equation)                             ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Key Concepts:                                               ║" << std::endl;
    std::cout << "║  • State variables store system 'memory' (energy)            ║" << std::endl;
    std::cout << "║  • n states = n-th order system                              ║" << std::endl;
    std::cout << "║  • Eigenvalues of A = system poles                           ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Advantages over Transfer Functions:                         ║" << std::endl;
    std::cout << "║  • Handles MIMO systems naturally                            ║" << std::endl;
    std::cout << "║  • Includes initial conditions                               ║" << std::endl;
    std::cout << "║  • Access to internal variables                              ║" << std::endl;
    std::cout << "║  • Foundation for modern control                             ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  TF ↔ SS Conversion:                                        ║" << std::endl;
    std::cout << "║  G(s) = C(sI - A)⁻¹B + D                                     ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    return 0;
}
