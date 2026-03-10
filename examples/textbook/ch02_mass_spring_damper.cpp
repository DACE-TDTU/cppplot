/**
 * @file ch02_mass_spring_damper.cpp
 * @brief Chapter 2: Mass-Spring-Damper System Modeling
 * 
 * Learning Outcomes:
 * - Derive transfer function from physical parameters
 * - Analyze effect of damping ratio on response
 * - Understand pole locations and their physical meaning
 * 
 * Physical System → Mathematical Model → CppPlot Visualization
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║     Chapter 2: Mass-Spring-Damper System Modeling            ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // ========================================================================
    // STEP 1: Define Physical Parameters
    // ========================================================================
    std::cout << "\n▶ STEP 1: Physical Parameters" << std::endl;
    std::cout << "  ─────────────────────────────" << std::endl;
    
    double m = 1.0;     // Mass [kg]
    double b = 0.5;     // Damping coefficient [N·s/m]
    double k = 4.0;     // Spring constant [N/m]
    
    std::cout << "  Mass         m = " << m << " kg" << std::endl;
    std::cout << "  Damping      b = " << b << " N·s/m" << std::endl;
    std::cout << "  Spring       k = " << k << " N/m" << std::endl;
    
    std::cout << "\n  Physical System:" << std::endl;
    std::cout << "         x(t) displacement" << std::endl;
    std::cout << "           │" << std::endl;
    std::cout << "    ┌──────┴──────┐" << std::endl;
    std::cout << "    │     Mass    │◀─────── F(t) applied force" << std::endl;
    std::cout << "    │      M      │" << std::endl;
    std::cout << "    └──────┬──────┘" << std::endl;
    std::cout << "           │" << std::endl;
    std::cout << "       ════╪════  Spring k" << std::endl;
    std::cout << "       ────╫────  Damper b" << std::endl;
    std::cout << "       ────┴────  Fixed wall" << std::endl;
    
    // ========================================================================
    // STEP 2: Derive Mathematical Model
    // ========================================================================
    std::cout << "\n▶ STEP 2: Mathematical Model (Newton's 2nd Law)" << std::endl;
    std::cout << "  ──────────────────────────────────────────────" << std::endl;
    
    std::cout << "  Sum of forces: F(t) - kx - b·dx/dt = m·d²x/dt²" << std::endl;
    std::cout << "  Standard form: m·x'' + b·x' + kx = F(t)" << std::endl;
    
    // ========================================================================
    // STEP 3: Derive Transfer Function
    // ========================================================================
    std::cout << "\n▶ STEP 3: Transfer Function (Laplace Transform)" << std::endl;
    std::cout << "  ─────────────────────────────────────────────" << std::endl;
    
    std::cout << "  Laplace: (ms² + bs + k)X(s) = F(s)" << std::endl;
    std::cout << "  G(s) = X(s)/F(s) = 1/(ms² + bs + k)" << std::endl;
    std::cout << "\n  G(s) = 1/(" << m << "s² + " << b << "s + " << k << ")" << std::endl;
    
    // Create transfer function
    TransferFunction G({1}, {m, b, k});
    
    // ========================================================================
    // STEP 4: Analyze System Characteristics
    // ========================================================================
    std::cout << "\n▶ STEP 4: System Characteristics" << std::endl;
    std::cout << "  ────────────────────────────────" << std::endl;
    
    double wn = std::sqrt(k / m);               // Natural frequency [rad/s]
    double zeta = b / (2 * std::sqrt(k * m));   // Damping ratio
    double wd = wn * std::sqrt(1 - zeta*zeta);  // Damped frequency (if underdamped)
    
    std::cout << "  Natural frequency  ωn = √(k/m) = " << wn << " rad/s" << std::endl;
    std::cout << "  Damping ratio      ζ = b/(2√(km)) = " << zeta << std::endl;
    
    std::string response_type;
    if (zeta < 1) {
        response_type = "Underdamped (oscillatory)";
        std::cout << "  Damped frequency   ωd = ωn√(1-ζ²) = " << wd << " rad/s" << std::endl;
    }
    else if (std::abs(zeta - 1) < 1e-6) {
        response_type = "Critically damped";
    }
    else {
        response_type = "Overdamped (sluggish)";
    }
    std::cout << "\n  Response type: " << response_type << std::endl;
    
    // Poles
    std::cout << "\n  Poles (roots of ms² + bs + k = 0):" << std::endl;
    auto poles = G.poles();
    for (size_t i = 0; i < poles.size(); ++i) {
        std::cout << "    s" << i+1 << " = " << std::fixed << std::setprecision(3)
                  << poles[i].real();
        if (std::abs(poles[i].imag()) > 1e-6) {
            std::cout << " ± j" << std::abs(poles[i].imag());
        }
        std::cout << std::endl;
    }
    
    // ========================================================================
    // STEP 5: Simulate and Visualize
    // ========================================================================
    std::cout << "\n▶ STEP 5: Simulation" << std::endl;
    std::cout << "  ─────────────────────" << std::endl;
    
    double t_final = 15.0;
    
    figure(1200, 900);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 1: Effect of Damping Ratio on Step Response
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 2, 1);
    
    std::vector<double> damping_values = {0.1, 0.3, 0.5, 0.707, 1.0, 2.0};
    std::vector<std::string> colors = {"#E74C3C", "#E67E22", "#F1C40F", "#2ECC71", "#3498DB", "#9B59B6"};
    
    for (size_t i = 0; i < damping_values.size(); ++i) {
        double zeta_i = damping_values[i];
        double b_i = 2 * zeta_i * std::sqrt(k * m);
        TransferFunction G_i({1}, {m, b_i, k});
        auto [t_i, x_i] = step(G_i, t_final);
        
        std::ostringstream label;
        label << "ζ = " << std::fixed << std::setprecision(3) << zeta_i;
        plot(t_i, x_i, "-", {{"color", colors[i]}, {"linewidth", "2"}, {"label", label.str()}});
    }
    
    axhline(1.0/k, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.5"}, {"label", "x_ss = 1/k"}});
    xlabel("Time [s]");
    ylabel("Displacement x [m]");
    title("Effect of Damping Ratio on Step Response");
    legend({{"loc", "lower right"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 2: Pole Locations in s-plane
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 2, 2);
    
    // Draw ωn circle (left half)
    std::vector<double> circle_re, circle_im;
    for (int i = 0; i <= 100; ++i) {
        double angle = M_PI/2 + i * M_PI / 100;
        circle_re.push_back(wn * std::cos(angle));
        circle_im.push_back(wn * std::sin(angle));
    }
    plot(circle_re, circle_im, "k--", {{"alpha", "0.3"}, {"label", "ωn circle"}});
    
    // Draw constant zeta lines
    for (double z : {0.3, 0.5, 0.707}) {
        std::vector<double> line_re, line_im;
        double angle = std::acos(z);
        for (double r = 0; r <= 3; r += 0.1) {
            line_re.push_back(-r * std::cos(M_PI - angle));
            line_im.push_back(r * std::sin(M_PI - angle));
        }
        plot(line_re, line_im, ":", {{"color", "gray"}, {"alpha", "0.5"}});
    }
    
    // Plot poles for each damping value
    for (size_t i = 0; i < damping_values.size(); ++i) {
        double zeta_i = damping_values[i];
        double b_i = 2 * zeta_i * std::sqrt(k * m);
        TransferFunction G_i({1}, {m, b_i, k});
        auto poles_i = G_i.poles();
        
        std::vector<double> re, im;
        for (auto& p : poles_i) {
            re.push_back(p.real());
            im.push_back(p.imag());
        }
        
        scatter(re, im, {{"color", colors[i]}, {"s", "100"}});
    }
    
    axhline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    axvline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    xlabel("Real Part σ");
    ylabel("Imaginary Part jω");
    title("Pole Locations in s-plane");
    grid(true);
    xlim(-5, 1);
    ylim(-3, 3);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 3: Impulse Response (physical meaning: sudden impact)
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 2, 3);
    
    auto [t_imp, x_imp] = impulse(G, t_final);
    plot(t_imp, x_imp, "b-", {{"linewidth", "2"}, {"label", "Impulse response"}});
    
    // Show envelope for underdamped case
    if (zeta < 1) {
        std::vector<double> t_env, env_pos, env_neg;
        for (double ti : t_imp) {
            t_env.push_back(ti);
            double env = std::exp(-zeta * wn * ti) / (m * wd);
            env_pos.push_back(env);
            env_neg.push_back(-env);
        }
        plot(t_env, env_pos, "r--", {{"linewidth", "1"}, {"label", "Envelope"}});
        plot(t_env, env_neg, "r--", {{"linewidth", "1"}});
    }
    
    xlabel("Time [s]");
    ylabel("Displacement x [m]");
    title("Impulse Response (sudden impact F·δ(t))");
    legend();
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 4: Free Vibration (x(0) = 1, v(0) = 0)
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 2, 4);
    
    double x0 = 1.0;  // Initial displacement
    std::vector<double> t_free, x_free;
    
    if (zeta < 1) {
        // Underdamped: x(t) = x0 * e^(-ζωn*t) * [cos(ωd*t) + (ζ/√(1-ζ²))*sin(ωd*t)]
        double phi = std::atan(zeta / std::sqrt(1 - zeta*zeta));
        double A = x0 / std::cos(phi);
        
        for (double ti = 0; ti <= t_final; ti += 0.02) {
            t_free.push_back(ti);
            double decay = std::exp(-zeta * wn * ti);
            x_free.push_back(A * decay * std::cos(wd * ti + phi));
        }
    }
    else if (std::abs(zeta - 1) < 1e-6) {
        // Critically damped: x(t) = x0 * (1 + ωn*t) * e^(-ωn*t)
        for (double ti = 0; ti <= t_final; ti += 0.02) {
            t_free.push_back(ti);
            x_free.push_back(x0 * (1 + wn * ti) * std::exp(-wn * ti));
        }
    }
    else {
        // Overdamped: x(t) = C1*e^(s1*t) + C2*e^(s2*t)
        double s1 = -zeta * wn + wn * std::sqrt(zeta*zeta - 1);
        double s2 = -zeta * wn - wn * std::sqrt(zeta*zeta - 1);
        double C1 = x0 * s2 / (s2 - s1);
        double C2 = -x0 * s1 / (s2 - s1);
        
        for (double ti = 0; ti <= t_final; ti += 0.02) {
            t_free.push_back(ti);
            x_free.push_back(C1 * std::exp(s1 * ti) + C2 * std::exp(s2 * ti));
        }
    }
    
    plot(t_free, x_free, "b-", {{"linewidth", "2"}, {"label", "Free response"}});
    
    // Envelope
    if (zeta < 1) {
        std::vector<double> env_p, env_n;
        for (double ti : t_free) {
            double env = x0 * std::exp(-zeta * wn * ti);
            env_p.push_back(env);
            env_n.push_back(-env);
        }
        plot(t_free, env_p, "r--", {{"alpha", "0.5"}, {"label", "Envelope"}});
        plot(t_free, env_n, "r--", {{"alpha", "0.5"}});
    }
    
    xlabel("Time [s]");
    ylabel("Displacement x [m]");
    title("Free Vibration from x(0) = 1 m");
    legend();
    grid(true);
    
    savefig("ch02_mass_spring_damper.svg");
    std::cout << "  ✓ Generated: ch02_mass_spring_damper.svg" << std::endl;
    
    // ========================================================================
    // STEP 6: Physical Interpretation
    // ========================================================================
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║               PHYSICAL INTERPRETATION                        ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  How physical parameters affect response:                    ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  • ↑ Mass m    → ↓ ωn (slower response, more inertia)       ║" << std::endl;
    std::cout << "║  • ↑ Spring k  → ↑ ωn (faster oscillation, stiffer)         ║" << std::endl;
    std::cout << "║  • ↑ Damping b → ↑ ζ  (less overshoot, more energy loss)    ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Response Types:                                             ║" << std::endl;
    std::cout << "║  • ζ < 1   : Underdamped - oscillates with decaying envelope ║" << std::endl;
    std::cout << "║  • ζ = 1   : Critical - fastest without oscillation          ║" << std::endl;
    std::cout << "║  • ζ > 1   : Overdamped - slow, no oscillation               ║" << std::endl;
    std::cout << "║  • ζ = 0.707: Optimal (Butterworth) - fast with 4.3% OS     ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    return 0;
}
