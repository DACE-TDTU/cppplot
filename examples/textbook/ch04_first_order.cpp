/**
 * @file ch04_first_order.cpp
 * @brief Chapter 4: First-order system analysis
 * 
 * This example demonstrates:
 * - Creating first-order transfer functions
 * - Computing step response
 * - Understanding time constant effects
 * - Comparing theoretical vs simulated results
 * 
 * Compile: g++ -std=c++14 -I "../../include" ch04_first_order.cpp -o ch04_first_order.exe
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
    std::cout << "║     Chapter 4: First-Order System Time-Domain Analysis       ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;

    // ========================================================================
    // Part 1: Basic First-Order System
    // ========================================================================
    std::cout << "\n▶ PART 1: Basic First-Order System" << std::endl;
    std::cout << "   ────────────────────────────────" << std::endl;
    
    // System parameters
    double K = 2.0;    // DC gain
    double tau = 3.0;  // Time constant [s]
    
    // Create transfer function: G(s) = K / (τs + 1)
    // In polynomial form: num = [K], den = [τ, 1]
    TransferFunction G({K}, {tau, 1});
    
    std::cout << "\n   Transfer Function: G(s) = " << K << " / (" 
              << tau << "s + 1)" << std::endl;
    std::cout << "   DC Gain K = " << K << std::endl;
    std::cout << "   Time Constant τ = " << tau << " s" << std::endl;
    
    // Generate step response
    double t_final = 5 * tau;  // Simulate for 5 time constants
    auto [t, y] = step(G, t_final);
    
    // Calculate theoretical values at key time points
    std::cout << "\n   ┌─────────────────────────────────────────────────────┐" << std::endl;
    std::cout << "   │ Time        │ Theory          │ Simulated │ % Final │" << std::endl;
    std::cout << "   ├─────────────┼─────────────────┼───────────┼─────────┤" << std::endl;
    
    std::vector<std::pair<double, std::string>> checkpoints = {
        {1.0 * tau, "τ"},
        {2.0 * tau, "2τ"},
        {3.0 * tau, "3τ"},
        {4.0 * tau, "4τ"},
        {5.0 * tau, "5τ"}
    };
    
    for (auto& [time, label] : checkpoints) {
        double y_theory = K * (1.0 - std::exp(-time / tau));
        double pct = y_theory / K * 100.0;
        
        // Find simulated value
        double y_sim = 0;
        for (size_t i = 0; i < t.size() - 1; ++i) {
            if (t[i] <= time && t[i+1] > time) {
                y_sim = y[i];
                break;
            }
        }
        
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "   │ t = " << std::setw(5) << label << " = " << std::setw(4) << time 
                  << " │ y = " << std::setw(6) << y_theory << " │ "
                  << std::setw(9) << y_sim << " │ " 
                  << std::setw(6) << pct << "% │" << std::endl;
    }
    std::cout << "   └─────────────────────────────────────────────────────┘" << std::endl;
    
    // ========================================================================
    // Part 2: Effect of Time Constant
    // ========================================================================
    std::cout << "\n▶ PART 2: Effect of Time Constant" << std::endl;
    std::cout << "   ─────────────────────────────────" << std::endl;
    
    std::vector<double> time_constants = {0.5, 1.0, 2.0, 5.0};
    double K_fixed = 1.0;
    
    figure(1000, 600);
    
    std::cout << "\n   Comparing systems with τ = ";
    for (size_t i = 0; i < time_constants.size(); ++i) {
        std::cout << time_constants[i];
        if (i < time_constants.size() - 1) std::cout << ", ";
    }
    std::cout << " seconds" << std::endl;
    
    std::vector<std::string> colors = {"blue", "green", "orange", "red"};
    
    for (size_t i = 0; i < time_constants.size(); ++i) {
        double tc = time_constants[i];
        TransferFunction G_i({K_fixed}, {tc, 1});
        auto [t_i, y_i] = step(G_i, 15.0);
        
        std::string label = "τ = " + std::to_string(tc).substr(0, 3) + " s";
        plot(t_i, y_i, "-", {{"color", colors[i]}, {"linewidth", "2"}, {"label", label}});
    }
    
    // Add reference line
    axhline(K_fixed, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.5"}, {"label", "Final value K"}});
    
    xlabel("Time [s]");
    ylabel("Output y(t)");
    title("First-Order Step Response: Effect of Time Constant");
    legend();
    grid(true);
    
    savefig("ch04_time_constant_effect.svg");
    std::cout << "\n   ✓ Plot saved: ch04_time_constant_effect.svg" << std::endl;
    
    // ========================================================================
    // Part 3: Effect of DC Gain
    // ========================================================================
    std::cout << "\n▶ PART 3: Effect of DC Gain" << std::endl;
    std::cout << "   ─────────────────────────" << std::endl;
    
    std::vector<double> gains = {0.5, 1.0, 2.0, 4.0};
    double tau_fixed = 2.0;
    
    figure(1000, 600);
    
    for (size_t i = 0; i < gains.size(); ++i) {
        double gain = gains[i];
        TransferFunction G_i({gain}, {tau_fixed, 1});
        auto [t_i, y_i] = step(G_i, 12.0);
        
        std::string label = "K = " + std::to_string(gain).substr(0, 3);
        plot(t_i, y_i, "-", {{"color", colors[i]}, {"linewidth", "2"}, {"label", label}});
    }
    
    xlabel("Time [s]");
    ylabel("Output y(t)");
    title("First-Order Step Response: Effect of DC Gain (τ = 2s)");
    legend();
    grid(true);
    
    savefig("ch04_dc_gain_effect.svg");
    std::cout << "\n   ✓ Plot saved: ch04_dc_gain_effect.svg" << std::endl;
    
    // ========================================================================
    // Part 4: Step Response Characteristics
    // ========================================================================
    std::cout << "\n▶ PART 4: Step Response Characteristics" << std::endl;
    std::cout << "   ──────────────────────────────────────" << std::endl;
    
    // Use the original system
    auto info = stepinfo(t, y);
    
    std::cout << "\n   For G(s) = " << K << "/(" << tau << "s + 1):" << std::endl;
    std::cout << "\n   ┌───────────────────────────────────────────┐" << std::endl;
    std::cout << "   │ Parameter          │ Value      │ Formula  │" << std::endl;
    std::cout << "   ├────────────────────┼────────────┼──────────┤" << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "   │ Rise Time (10-90%) │ " << std::setw(8) << info.rise_time 
              << " s │ 2.2τ     │" << std::endl;
    std::cout << "   │ Settling Time (2%) │ " << std::setw(8) << info.settling_time 
              << " s │ 4τ       │" << std::endl;
    std::cout << "   │ Peak Value         │ " << std::setw(10) << info.peak_value 
              << " │ K        │" << std::endl;
    std::cout << "   │ Overshoot          │ " << std::setw(8) << info.overshoot 
              << " % │ 0        │" << std::endl;
    std::cout << "   └───────────────────────────────────────────┘" << std::endl;
    
    std::cout << "\n   Theoretical values:" << std::endl;
    std::cout << "   - Rise time:     2.2 × " << tau << " = " << 2.2*tau << " s" << std::endl;
    std::cout << "   - Settling time: 4 × " << tau << " = " << 4.0*tau << " s" << std::endl;
    
    // ========================================================================
    // Part 5: Different Input Types
    // ========================================================================
    std::cout << "\n▶ PART 5: Response to Different Inputs" << std::endl;
    std::cout << "   ──────────────────────────────────────" << std::endl;
    
    figure(1200, 800);
    
    // Unit Step
    subplot(2, 2, 1);
    plot(t, y, "b-", {{"linewidth", "2"}});
    axhline(K, {{"color", "red"}, {"linestyle", "--"}, {"alpha", "0.5"}});
    xlabel("Time [s]");
    ylabel("Output");
    title("Step Response");
    grid(true);
    
    // Impulse
    subplot(2, 2, 2);
    auto [t_imp, y_imp] = impulse(G, t_final);
    plot(t_imp, y_imp, "g-", {{"linewidth", "2"}});
    xlabel("Time [s]");
    ylabel("Output");
    title("Impulse Response");
    grid(true);
    
    // Ramp Response (using lsim)
    subplot(2, 2, 3);
    std::vector<double> t_ramp, u_ramp;
    for (double ti = 0; ti <= t_final; ti += 0.05) {
        t_ramp.push_back(ti);
        u_ramp.push_back(ti);  // ramp: u(t) = t
    }
    auto [t_out, y_ramp] = lsim(G, u_ramp, t_ramp);
    plot(t_out, y_ramp, "m-", {{"linewidth", "2"}, {"label", "Output"}});
    plot(t_ramp, u_ramp, "k--", {{"linewidth", "1"}, {"label", "Input (ramp)"}});
    xlabel("Time [s]");
    ylabel("Output");
    title("Ramp Response");
    legend();
    grid(true);
    
    // Sinusoidal Response
    subplot(2, 2, 4);
    std::vector<double> t_sin, u_sin;
    double freq = 0.5;  // Hz
    for (double ti = 0; ti <= 20.0; ti += 0.05) {
        t_sin.push_back(ti);
        u_sin.push_back(std::sin(2 * M_PI * freq * ti));
    }
    auto [t_sin_out, y_sin] = lsim(G, u_sin, t_sin);
    plot(t_sin, u_sin, "k--", {{"linewidth", "1"}, {"label", "Input"}});
    plot(t_sin_out, y_sin, "b-", {{"linewidth", "2"}, {"label", "Output"}});
    xlabel("Time [s]");
    ylabel("Output");
    title("Sinusoidal Response (f = 0.5 Hz)");
    legend();
    grid(true);
    
    savefig("ch04_first_order_responses.svg");
    std::cout << "\n   ✓ Plot saved: ch04_first_order_responses.svg" << std::endl;
    
    // ========================================================================
    // Summary
    // ========================================================================
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                         SUMMARY                              ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ First-order system: G(s) = K/(τs + 1)                        ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║ Key characteristics:                                         ║" << std::endl;
    std::cout << "║ • Time constant τ: time to reach 63.2% of final value       ║" << std::endl;
    std::cout << "║ • DC gain K: steady-state value for unit step               ║" << std::endl;
    std::cout << "║ • No overshoot (exponential approach)                       ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║ Design formulas:                                             ║" << std::endl;
    std::cout << "║ • Rise time (10-90%):  tr ≈ 2.2τ                             ║" << std::endl;
    std::cout << "║ • Settling time (2%):  ts ≈ 4τ                               ║" << std::endl;
    std::cout << "║ • Settling time (5%):  ts ≈ 3τ                               ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    return 0;
}
