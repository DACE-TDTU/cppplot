/**
 * @file ch04_first_order_systems.cpp
 * @brief Chapter 4: First-Order System Analysis
 * 
 * Learning Outcomes:
 * - Calculate time constant from physical parameters
 * - Analyze step and ramp responses
 * - Understand the meaning of 63.2% and 5τ settling
 * 
 * Physical System: Thermal System (heated metal block)
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
    std::cout << "║     Chapter 4: First-Order System Analysis                   ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // ========================================================================
    // PHYSICAL SYSTEM: Heated Metal Block
    // ========================================================================
    std::cout << "\n▶ Physical System: Heated Metal Block" << std::endl;
    std::cout << "  ─────────────────────────────────────" << std::endl;
    
    // Physical parameters
    double M = 0.5;       // Mass [kg]
    double c = 900;       // Specific heat of aluminum [J/(kg·K)]
    double h = 10;        // Convection coefficient [W/(m²·K)]
    double A = 0.06;      // Surface area [m²]
    double T_amb = 25;    // Ambient temperature [°C]
    
    std::cout << "  Physical Parameters:" << std::endl;
    std::cout << "    Mass M = " << M << " kg" << std::endl;
    std::cout << "    Specific heat c = " << c << " J/(kg·K)" << std::endl;
    std::cout << "    Convection h = " << h << " W/(m²·K)" << std::endl;
    std::cout << "    Surface area A = " << A << " m²" << std::endl;
    std::cout << "    Ambient temp T_amb = " << T_amb << " °C" << std::endl;
    
    // Derived parameters
    double tau = M * c / (h * A);           // Time constant [s]
    double K = 1.0 / (h * A);               // DC gain [K/W]
    
    std::cout << "\n  Derived Parameters:" << std::endl;
    std::cout << "    Time constant τ = Mc/(hA) = " << tau << " s = " << tau/60 << " min" << std::endl;
    std::cout << "    DC gain K = 1/(hA) = " << K << " K/W" << std::endl;
    std::cout << "    Pole location: s = -1/τ = " << -1/tau << std::endl;
    
    // ========================================================================
    // TRANSFER FUNCTION
    // ========================================================================
    std::cout << "\n▶ Transfer Function" << std::endl;
    std::cout << "  ─────────────────────" << std::endl;
    std::cout << "  G(s) = K/(τs + 1) = " << K << "/(" << tau << "s + 1)" << std::endl;
    
    TransferFunction G({K}, {tau, 1});
    
    // ========================================================================
    // ANALYSIS: Step Response
    // ========================================================================
    std::cout << "\n▶ Step Response Analysis" << std::endl;
    std::cout << "  ───────────────────────" << std::endl;
    
    double Q_step = 100;  // 100W heater power
    double T_ss = Q_step * K;  // Steady-state temperature rise
    
    std::cout << "  Input: Q = " << Q_step << " W (step heater power)" << std::endl;
    std::cout << "  Final temperature rise: ΔT_ss = K·Q = " << T_ss << " °C" << std::endl;
    std::cout << "  Final absolute temp: T_final = T_amb + ΔT_ss = " << T_amb + T_ss << " °C" << std::endl;
    
    // Performance metrics
    std::cout << "\n  Time Response Metrics:" << std::endl;
    std::cout << "    At t = τ:   T reaches 63.2% of final value" << std::endl;
    std::cout << "    At t = 3τ:  T reaches 95.0% of final value" << std::endl;
    std::cout << "    At t = 5τ:  T reaches 99.3% of final value (settling time)" << std::endl;
    std::cout << "\n    τ = " << tau << " s" << std::endl;
    std::cout << "    3τ = " << 3*tau << " s" << std::endl;
    std::cout << "    5τ = " << 5*tau << " s = " << 5*tau/60 << " min" << std::endl;
    
    // ========================================================================
    // SIMULATION
    // ========================================================================
    double t_final = 6 * tau;
    auto [t, delta_T] = step(G * Q_step, t_final);
    
    // Convert to absolute temperature
    std::vector<double> T_actual;
    for (double dT : delta_T) {
        T_actual.push_back(T_amb + dT);
    }
    
    figure(1400, 1000);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 1: Step Response with Key Points
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 2, 1);
    
    plot(t, T_actual, "b-", {{"linewidth", "2"}, {"label", "Temperature T(t)"}});
    
    // Final value
    double T_final = T_amb + T_ss;
    axhline(T_final, {{"color", "green"}, {"linestyle", "--"}, {"label", "Final: " + std::to_string(int(T_final)) + "°C"}});
    axhline(T_amb, {{"color", "gray"}, {"linestyle", ":"}, {"label", "Ambient"}});
    
    // Key time points
    double T_63 = T_amb + 0.632 * T_ss;
    double T_95 = T_amb + 0.95 * T_ss;
    double T_99 = T_amb + 0.993 * T_ss;
    
    scatter({tau}, {T_63}, {{"color", "red"}, {"s", "100"}, {"zorder", "5"}});
    scatter({3*tau}, {T_95}, {{"color", "orange"}, {"s", "100"}, {"zorder", "5"}});
    scatter({5*tau}, {T_99}, {{"color", "green"}, {"s", "100"}, {"zorder", "5"}});
    
    // Annotations
    axvline(tau, {{"color", "red"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    axvline(3*tau, {{"color", "orange"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    axvline(5*tau, {{"color", "green"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    
    xlabel("Time [s]");
    ylabel("Temperature [°C]");
    title("Thermal System Step Response (Q = 100W)");
    legend({{"loc", "lower right"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 2: Normalized Response for Different τ
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 2, 2);
    
    std::vector<double> tau_values = {100, 200, 500, 750};
    std::vector<std::string> colors = {"#E74C3C", "#E67E22", "#2ECC71", "#3498DB"};
    
    for (size_t i = 0; i < tau_values.size(); ++i) {
        double tau_i = tau_values[i];
        TransferFunction G_i({1}, {tau_i, 1});  // Normalized (K=1)
        auto [t_i, y_i] = step(G_i, 4000);
        
        std::ostringstream label;
        label << "τ = " << tau_i << " s";
        plot(t_i, y_i, "-", {{"color", colors[i]}, {"linewidth", "2"}, {"label", label.str()}});
    }
    
    axhline(0.632, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.5"}, {"label", "63.2%"}});
    axhline(1.0, {{"color", "black"}, {"linestyle", ":"}, {"alpha", "0.3"}});
    
    xlabel("Time [s]");
    ylabel("Normalized Response y/K");
    title("Effect of Time Constant τ");
    legend({{"fontsize", "9"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 3: Ramp Response
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 2, 3);
    
    // Ramp input: u(t) = r·t, where r = ramp rate
    // For ramp input: U(s) = r/s²
    // Y(s) = G(s)·r/s² = Kr/[s²(τs+1)]
    // y(t) = Kr[t - τ + τ·e^(-t/τ)] = Kr[t - τ(1 - e^(-t/τ))]
    
    double ramp_rate = 10.0;  // W/s (heater power ramp)
    std::vector<double> t_ramp, y_ramp, u_ramp, error_ramp;
    
    for (double ti = 0; ti <= 4*tau; ti += tau/100) {
        t_ramp.push_back(ti);
        double u = ramp_rate * ti;  // Input: linearly increasing power
        u_ramp.push_back(K * u);    // Desired output (if no lag)
        
        // Ramp response: y(t) = K·r·[t - τ(1 - e^(-t/τ))]
        double y = K * ramp_rate * (ti - tau * (1 - std::exp(-ti/tau)));
        y_ramp.push_back(y);
        
        // Steady-state error = τ (lag behind ideal)
        double error = K * ramp_rate * tau * (1 - std::exp(-ti/tau));
        error_ramp.push_back(error);
    }
    
    plot(t_ramp, u_ramp, "g--", {{"linewidth", "1.5"}, {"label", "Ideal (no lag)"}});
    plot(t_ramp, y_ramp, "b-", {{"linewidth", "2"}, {"label", "Actual response"}});
    
    xlabel("Time [s]");
    ylabel("Temperature Rise [°C]");
    title("Ramp Response (Q increases at " + std::to_string(int(ramp_rate)) + " W/s)");
    legend();
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 4: Steady-State Error for Ramp
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 2, 4);
    
    plot(t_ramp, error_ramp, "r-", {{"linewidth", "2"}, {"label", "Tracking error"}});
    
    double ss_error = K * ramp_rate * tau;
    axhline(ss_error, {{"color", "black"}, {"linestyle", "--"}, {"label", "SS error = K·r·τ = " + std::to_string(ss_error).substr(0,5) + "°C"}});
    
    xlabel("Time [s]");
    ylabel("Error [°C]");
    title("Ramp Tracking Error");
    legend();
    grid(true);
    
    savefig("ch04_first_order_systems.svg");
    std::cout << "\n✓ Saved ch04_first_order_systems.svg" << std::endl;
    
    // ========================================================================
    // DESIGN INSIGHTS
    // ========================================================================
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║               FIRST-ORDER SYSTEM SUMMARY                     ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Standard Form: G(s) = K/(τs + 1)                            ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Step Response: y(t) = K·u_step·(1 - e^(-t/τ))              ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Key Time Points:                                            ║" << std::endl;
    std::cout << "║  ┌──────────┬────────────┬─────────────────────────────┐    ║" << std::endl;
    std::cout << "║  │  Time    │  % Final   │  Notes                      │    ║" << std::endl;
    std::cout << "║  ├──────────┼────────────┼─────────────────────────────┤    ║" << std::endl;
    std::cout << "║  │   τ      │   63.2%    │  Definition of time const.  │    ║" << std::endl;
    std::cout << "║  │   2τ     │   86.5%    │                             │    ║" << std::endl;
    std::cout << "║  │   3τ     │   95.0%    │  Often used for t_s (5%)    │    ║" << std::endl;
    std::cout << "║  │   4τ     │   98.2%    │  Often used for t_s (2%)    │    ║" << std::endl;
    std::cout << "║  │   5τ     │   99.3%    │  Settling time (1%)         │    ║" << std::endl;
    std::cout << "║  └──────────┴────────────┴─────────────────────────────┘    ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Ramp Response Error: e_ss = K·r·τ                          ║" << std::endl;
    std::cout << "║  (system lags behind by τ seconds)                          ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    return 0;
}
