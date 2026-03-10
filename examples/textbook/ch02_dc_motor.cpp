/**
 * @file ch02_dc_motor.cpp
 * @brief Chapter 2: DC Motor Modeling - Electromechanical System
 * 
 * Learning Outcomes:
 * - Model coupled electrical-mechanical systems
 * - Derive transfer function from physical equations
 * - Understand motor operating characteristics
 * 
 * Physical System: DC Motor
 * Electrical: V = La*di/dt + Ra*i + Kb*ω
 * Mechanical: J*dω/dt + b*ω = Kt*i
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
    std::cout << "║         Chapter 2: DC Motor Modeling                         ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // ========================================================================
    // STEP 1: Physical System Description
    // ========================================================================
    std::cout << "\n▶ STEP 1: Physical System" << std::endl;
    std::cout << "  ────────────────────────" << std::endl;
    
    std::cout << R"(
    DC Motor Cross-Section:
    
        V(t) ──┬──[Ra]──[La]──┬──▶  ┌─────────┐
        input  │              │     │ ╭─────╮ │  ω(t) speed
               │             e_b    │ │Motor│ │──────▶
               │            (back   │ │Core │ │  θ(t) position
               │             EMF)   │ ╰─────╯ │
               └───────────────────┤└────┬────┘
                                   │     │
                             GND ──┴─────┘
    
    Electrical Circuit:           Mechanical Shaft:
    ┌───[Ra]───[La]───┐          ┌──────────────┐
    │                 │          │   ╭────╮     │
    V(t)             e_b = Kb·ω  │   │ J  │───▶ ω(t)
    │                 │          │   ╰────╯     │
    └─────────────────┘          │ Friction: b·ω│
                                 └──────────────┘
)" << std::endl;

    // ========================================================================
    // STEP 2: Define Physical Parameters
    // ========================================================================
    std::cout << "\n▶ STEP 2: Physical Parameters" << std::endl;
    std::cout << "  ────────────────────────────" << std::endl;
    
    // Typical small DC motor parameters
    double Ra = 2.0;      // Armature resistance [Ω]
    double La = 0.01;     // Armature inductance [H]
    double J = 0.01;      // Rotor inertia [kg·m²]
    double b = 0.001;     // Viscous friction [N·m·s/rad]
    double Kt = 0.1;      // Torque constant [N·m/A]
    double Kb = 0.1;      // Back-EMF constant [V·s/rad]
    
    std::cout << "  Electrical:" << std::endl;
    std::cout << "    Armature resistance  Ra = " << Ra << " Ω" << std::endl;
    std::cout << "    Armature inductance  La = " << La*1000 << " mH" << std::endl;
    std::cout << "\n  Mechanical:" << std::endl;
    std::cout << "    Rotor inertia        J  = " << J << " kg·m²" << std::endl;
    std::cout << "    Viscous friction     b  = " << b << " N·m·s/rad" << std::endl;
    std::cout << "\n  Coupling Constants:" << std::endl;
    std::cout << "    Torque constant      Kt = " << Kt << " N·m/A" << std::endl;
    std::cout << "    Back-EMF constant    Kb = " << Kb << " V·s/rad" << std::endl;
    
    // ========================================================================
    // STEP 3: Derive Governing Equations
    // ========================================================================
    std::cout << "\n▶ STEP 3: Governing Equations" << std::endl;
    std::cout << "  ─────────────────────────────" << std::endl;
    
    std::cout << "  Electrical (KVL):" << std::endl;
    std::cout << "    V(t) = La·(di/dt) + Ra·i + e_b" << std::endl;
    std::cout << "    where e_b = Kb·ω (back-EMF)" << std::endl;
    std::cout << "\n  Mechanical (Newton for rotation):" << std::endl;
    std::cout << "    J·(dω/dt) + b·ω = τ_motor = Kt·i" << std::endl;
    
    // ========================================================================
    // STEP 4: Derive Transfer Functions
    // ========================================================================
    std::cout << "\n▶ STEP 4: Transfer Functions" << std::endl;
    std::cout << "  ────────────────────────────" << std::endl;
    
    std::cout << "  Laplace Transform (zero ICs):" << std::endl;
    std::cout << "    V(s) = (La·s + Ra)·I(s) + Kb·Ω(s)" << std::endl;
    std::cout << "    (J·s + b)·Ω(s) = Kt·I(s)" << std::endl;
    
    // Full model coefficients: G(s) = Kt / [(La·s + Ra)(J·s + b) + Kt·Kb]
    //                               = Kt / [La·J·s² + (La·b + Ra·J)·s + (Ra·b + Kt·Kb)]
    double a2 = La * J;
    double a1 = La * b + Ra * J;
    double a0 = Ra * b + Kt * Kb;
    
    std::cout << "\n  Speed TF: Ω(s)/V(s) = Kt / [(La·s + Ra)(J·s + b) + Kt·Kb]" << std::endl;
    std::cout << "                      = " << Kt << " / (" 
              << a2 << "s² + " << a1 << "s + " << a0 << ")" << std::endl;
    
    // Create transfer functions
    TransferFunction G_speed_full({Kt}, {a2, a1, a0});
    TransferFunction G_position_full({Kt}, {a2, a1, a0, 0});  // Integrate for position
    
    // ========================================================================
    // STEP 5: Simplified Model (La ≈ 0)
    // ========================================================================
    std::cout << "\n▶ STEP 5: Simplified Model (La ≈ 0)" << std::endl;
    std::cout << "  ────────────────────────────────────" << std::endl;
    
    std::cout << "  Since electrical time constant τe = La/Ra << τm (mechanical)" << std::endl;
    std::cout << "  We can neglect La to get first-order model:" << std::endl;
    
    double Km = Kt / (Ra * b + Kt * Kb);            // Motor gain
    double tau_m = J * Ra / (Ra * b + Kt * Kb);     // Mechanical time constant
    
    std::cout << "\n  G_speed(s) = Km/(τm·s + 1)" << std::endl;
    std::cout << "  where:" << std::endl;
    std::cout << "    Km  = Kt/(Ra·b + Kt·Kb) = " << std::fixed << std::setprecision(4) << Km << " rad/(V·s)" << std::endl;
    std::cout << "    τm  = J·Ra/(Ra·b + Kt·Kb) = " << tau_m*1000 << " ms" << std::endl;
    
    TransferFunction G_speed_simple({Km}, {tau_m, 1});
    TransferFunction G_position_simple({Km}, {tau_m, 1, 0});
    
    // ========================================================================
    // STEP 6: Analyze Poles
    // ========================================================================
    std::cout << "\n▶ STEP 6: Pole Analysis" << std::endl;
    std::cout << "  ───────────────────────" << std::endl;
    
    auto poles_full = G_speed_full.poles();
    std::cout << "  Full model poles:" << std::endl;
    for (size_t i = 0; i < poles_full.size(); ++i) {
        std::cout << "    p" << i+1 << " = " << std::fixed << std::setprecision(3) 
                  << poles_full[i].real();
        if (std::abs(poles_full[i].imag()) > 1e-6) {
            std::cout << " + j" << poles_full[i].imag();
        }
        std::cout << std::endl;
    }
    
    std::cout << "\n  Simplified model pole:" << std::endl;
    std::cout << "    p = -1/τm = " << -1/tau_m << std::endl;
    
    // ========================================================================
    // STEP 7: Simulate with Step Input
    // ========================================================================
    std::cout << "\n▶ STEP 7: Simulation" << std::endl;
    std::cout << "  ─────────────────────" << std::endl;
    
    double V_step = 12.0;  // 12V step input
    double t_final = 0.5;
    
    std::cout << "  Input: " << V_step << " V step" << std::endl;
    
    // Speed responses
    auto [t1, speed_full] = step(G_speed_full * V_step, t_final);
    auto [t2, speed_simple] = step(G_speed_simple * V_step, t_final);
    
    // Position responses
    auto [t3, pos_full] = step(G_position_full * V_step, t_final);
    auto [t4, pos_simple] = step(G_position_simple * V_step, t_final);
    
    // Calculate current: i = (V - Kb·ω)/Ra
    std::vector<double> current_full, current_simple;
    for (double w : speed_full) {
        current_full.push_back((V_step - Kb * w) / Ra);
    }
    for (double w : speed_simple) {
        current_simple.push_back((V_step - Kb * w) / Ra);
    }
    
    // Calculate torque: τ = Kt·i
    std::vector<double> torque_full, torque_simple;
    for (double i : current_full) {
        torque_full.push_back(Kt * i);
    }
    for (double i : current_simple) {
        torque_simple.push_back(Kt * i);
    }
    
    // ========================================================================
    // STEP 8: Generate Plots
    // ========================================================================
    figure(1400, 1000);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 1: Speed Response
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 3, 1);
    plot(t1, speed_full, "b-", {{"linewidth", "2"}, {"label", "Full model"}});
    plot(t2, speed_simple, "r--", {{"linewidth", "2"}, {"label", "Simplified (La=0)"}});
    
    double omega_ss = Km * V_step;
    axhline(omega_ss, {{"color", "green"}, {"linestyle", ":"}, {"label", "Steady state"}});
    
    xlabel("Time [s]");
    ylabel("Speed ω [rad/s]");
    title("Motor Speed Response (V = 12V)");
    legend();
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 2: Position Response
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 3, 2);
    plot(t3, pos_full, "b-", {{"linewidth", "2"}, {"label", "Full model"}});
    plot(t4, pos_simple, "r--", {{"linewidth", "2"}, {"label", "Simplified"}});
    xlabel("Time [s]");
    ylabel("Position θ [rad]");
    title("Motor Position Response");
    legend();
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 3: Current Response
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 3, 3);
    plot(t1, current_full, "b-", {{"linewidth", "2"}, {"label", "Full model"}});
    plot(t2, current_simple, "r--", {{"linewidth", "2"}, {"label", "Simplified"}});
    
    double i_stall = V_step / Ra;
    axhline(i_stall, {{"color", "orange"}, {"linestyle", ":"}, {"label", "Stall current"}});
    
    xlabel("Time [s]");
    ylabel("Current i [A]");
    title("Armature Current");
    legend();
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 4: Torque Response
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 3, 4);
    plot(t1, torque_full, "b-", {{"linewidth", "2"}, {"label", "Full model"}});
    plot(t2, torque_simple, "r--", {{"linewidth", "2"}, {"label", "Simplified"}});
    
    double tau_stall = Kt * i_stall;
    axhline(tau_stall, {{"color", "orange"}, {"linestyle", ":"}, {"label", "Stall torque"}});
    
    xlabel("Time [s]");
    ylabel("Torque τ [N·m]");
    title("Motor Torque");
    legend();
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 5: Torque-Speed Characteristic
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 3, 5);
    
    // Motor characteristic curve: τ = τ_stall * (1 - ω/ω_no_load)
    double omega_no_load = V_step * Kt / (Kt * Kb);  // When τ = 0
    std::vector<double> omega_curve, tau_curve;
    for (double w = 0; w <= omega_no_load * 1.1; w += omega_no_load/50) {
        omega_curve.push_back(w);
        double tau = tau_stall * (1 - w / omega_no_load);
        tau_curve.push_back(tau);
    }
    
    plot(omega_curve, tau_curve, "b-", {{"linewidth", "2"}, {"label", "Torque-Speed curve"}});
    scatter({0}, {tau_stall}, {{"color", "red"}, {"s", "100"}, {"label", "Stall point"}});
    scatter({omega_no_load}, {0}, {{"color", "green"}, {"s", "100"}, {"label", "No-load point"}});
    
    xlabel("Speed ω [rad/s]");
    ylabel("Torque τ [N·m]");
    title("Motor Characteristic Curve");
    legend();
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 6: Power Curves
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 3, 6);
    
    // Mechanical power: P_mech = τ * ω
    // Electrical power: P_elec = V * i
    std::vector<double> P_mech, P_elec, efficiency;
    for (size_t i = 0; i < t1.size(); ++i) {
        double p_m = torque_full[i] * speed_full[i];
        double p_e = V_step * current_full[i];
        P_mech.push_back(p_m);
        P_elec.push_back(p_e);
        if (p_e > 0.01) {
            efficiency.push_back(100 * p_m / p_e);
        } else {
            efficiency.push_back(0);
        }
    }
    
    plot(t1, P_elec, "r-", {{"linewidth", "2"}, {"label", "P_electrical"}});
    plot(t1, P_mech, "b-", {{"linewidth", "2"}, {"label", "P_mechanical"}});
    
    xlabel("Time [s]");
    ylabel("Power [W]");
    title("Power Analysis");
    legend();
    grid(true);
    
    savefig("ch02_dc_motor.svg");
    std::cout << "  ✓ Generated: ch02_dc_motor.svg" << std::endl;
    
    // ========================================================================
    // STEP 9: Physical Interpretation
    // ========================================================================
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║               MOTOR OPERATING POINTS                         ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  For V = " << std::fixed << std::setprecision(0) << V_step << " V input:                                          ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Stall Condition (ω = 0):                                    ║" << std::endl;
    std::cout << "║    • Current:  i_stall = V/Ra = " << std::setprecision(1) << i_stall << " A                    ║" << std::endl;
    std::cout << "║    • Torque:   τ_stall = Kt·i = " << tau_stall << " N·m                  ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  No-Load Condition (τ = 0):                                  ║" << std::endl;
    std::cout << "║    • Speed:    ω_no_load = " << std::setprecision(1) << omega_ss << " rad/s                ║" << std::endl;
    std::cout << "║    • Current:  i ≈ 0 A (only friction)                       ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Time Constant:                                              ║" << std::endl;
    std::cout << "║    • τm = " << std::setprecision(1) << tau_m*1000 << " ms                                           ║" << std::endl;
    std::cout << "║    • 63% of final speed in " << tau_m*1000 << " ms                       ║" << std::endl;
    std::cout << "║    • 95% of final speed in 3τm = " << 3*tau_m*1000 << " ms                  ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // ========================================================================
    // Design Insights
    // ========================================================================
    std::cout << "\n┌────────────────────────────────────────────────────────────────┐" << std::endl;
    std::cout << "│                    DESIGN INSIGHTS                             │" << std::endl;
    std::cout << "├────────────────────────────────────────────────────────────────┤" << std::endl;
    std::cout << "│                                                                │" << std::endl;
    std::cout << "│  To increase motor speed:                                      │" << std::endl;
    std::cout << "│    • ↑ Applied voltage V                                       │" << std::endl;
    std::cout << "│    • ↓ Armature resistance Ra (better winding)                 │" << std::endl;
    std::cout << "│                                                                │" << std::endl;
    std::cout << "│  To increase motor torque:                                     │" << std::endl;
    std::cout << "│    • ↑ Current i (limited by thermal capacity)                 │" << std::endl;
    std::cout << "│    • ↑ Torque constant Kt (stronger magnets)                   │" << std::endl;
    std::cout << "│                                                                │" << std::endl;
    std::cout << "│  To speed up response:                                         │" << std::endl;
    std::cout << "│    • ↓ Rotor inertia J (lighter rotor)                         │" << std::endl;
    std::cout << "│    • ↓ Armature resistance Ra                                  │" << std::endl;
    std::cout << "│                                                                │" << std::endl;
    std::cout << "└────────────────────────────────────────────────────────────────┘" << std::endl;
    
    return 0;
}
