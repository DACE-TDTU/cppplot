/**
 * @file ch02_rlc_circuit.cpp
 * @brief Chapter 2: RLC Circuit Modeling and Electrical-Mechanical Analogy
 * 
 * Learning Outcomes:
 * - Apply Kirchhoff's laws to derive circuit equations
 * - Understand electrical-mechanical system analogies
 * - Analyze resonance in second-order systems
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <complex>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║         Chapter 2: RLC Circuit Modeling                      ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // ========================================================================
    // STEP 1: Physical System
    // ========================================================================
    std::cout << "\n▶ STEP 1: Physical System" << std::endl;
    std::cout << "  ────────────────────────" << std::endl;
    
    std::cout << R"(
    Series RLC Circuit:
    
         V_in(t)
           │
           ▼
    ┌──────┴──────┐
    │             │
   ═╪═ R         │  i(t)
   ═╪═            │   →
    │             │
   ┌┴┐            │
   │L│            │
   └┬┘            │
    │             │
    ├─────────────┤
   ═╪═           │
    C   V_out(t)  │
   ═╪═           │
    │             │
    ──────────────┴──────────── GND
    
)" << std::endl;

    // ========================================================================
    // STEP 2: Define Physical Parameters
    // ========================================================================
    std::cout << "\n▶ STEP 2: Physical Parameters" << std::endl;
    std::cout << "  ────────────────────────────" << std::endl;
    
    double R = 10;         // Resistance [Ω]
    double L = 0.1;        // Inductance [H]
    double C = 100e-6;     // Capacitance [F] = 100 μF
    
    std::cout << "  Resistance   R = " << R << " Ω" << std::endl;
    std::cout << "  Inductance   L = " << L*1000 << " mH" << std::endl;
    std::cout << "  Capacitance  C = " << C*1e6 << " μF" << std::endl;
    
    // ========================================================================
    // STEP 3: Apply Kirchhoff's Voltage Law
    // ========================================================================
    std::cout << "\n▶ STEP 3: Kirchhoff's Voltage Law (KVL)" << std::endl;
    std::cout << "  ──────────────────────────────────────" << std::endl;
    
    std::cout << "  Around the loop: V_in = V_R + V_L + V_C" << std::endl;
    std::cout << "  V_R = R·i" << std::endl;
    std::cout << "  V_L = L·(di/dt)" << std::endl;
    std::cout << "  V_C = (1/C)·∫i dt" << std::endl;
    std::cout << "\n  Since i = C·(dV_C/dt):" << std::endl;
    std::cout << "  V_in = LC·(d²V_C/dt²) + RC·(dV_C/dt) + V_C" << std::endl;
    
    // ========================================================================
    // STEP 4: Derive Transfer Function
    // ========================================================================
    std::cout << "\n▶ STEP 4: Transfer Function" << std::endl;
    std::cout << "  ──────────────────────────" << std::endl;
    
    std::cout << "  Laplace Transform:" << std::endl;
    std::cout << "  V_out(s)/V_in(s) = (1/LC) / [s² + (R/L)s + (1/LC)]" << std::endl;
    
    double wn_sq = 1.0 / (L * C);
    double two_zeta_wn = R / L;
    double wn = std::sqrt(wn_sq);
    double zeta = two_zeta_wn / (2 * wn);
    double wd = wn * std::sqrt(1 - zeta*zeta);
    double f_resonance = wn / (2 * M_PI);
    
    std::cout << "\n  G(s) = " << wn_sq << " / (s² + " << two_zeta_wn << "s + " << wn_sq << ")" << std::endl;
    std::cout << "\n  Characteristic parameters:" << std::endl;
    std::cout << "    Natural frequency ωn = 1/√(LC) = " << wn << " rad/s" << std::endl;
    std::cout << "    Resonant frequency f = " << f_resonance << " Hz" << std::endl;
    std::cout << "    Damping ratio ζ = (R/2)·√(C/L) = " << zeta << std::endl;
    
    // Create transfer function
    TransferFunction G_rlc({wn_sq}, {1, two_zeta_wn, wn_sq});
    
    // ========================================================================
    // STEP 5: Electrical-Mechanical Analogy
    // ========================================================================
    std::cout << "\n▶ STEP 5: Electrical-Mechanical Analogy" << std::endl;
    std::cout << "  ───────────────────────────────────────" << std::endl;
    
    std::cout << R"(
    ┌─────────────────────────────────────────────────────────────┐
    │           FORCE-VOLTAGE ANALOGY                             │
    ├──────────────────┬──────────────────┬───────────────────────┤
    │    Mechanical    │    Electrical    │    Common Role        │
    ├──────────────────┼──────────────────┼───────────────────────┤
    │    Force F       │    Voltage V     │    Effort variable    │
    │    Velocity v    │    Current i     │    Flow variable      │
    │    Mass m        │    Inductance L  │    Inertia            │
    │    Damping b     │    Resistance R  │    Dissipation        │
    │    Spring 1/k    │    Capacitance C │    Compliance         │
    │    Displacement x│    Charge q      │    Through integral   │
    └──────────────────┴──────────────────┴───────────────────────┘
)" << std::endl;
    
    // Equivalent mechanical system
    double m_eq = L;       // Inductance ↔ Mass
    double b_eq = R;       // Resistance ↔ Damping
    double k_eq = 1/C;     // 1/Capacitance ↔ Spring constant
    
    std::cout << "  Equivalent mechanical system:" << std::endl;
    std::cout << "    m_eq = L = " << m_eq << " kg" << std::endl;
    std::cout << "    b_eq = R = " << b_eq << " N·s/m" << std::endl;
    std::cout << "    k_eq = 1/C = " << k_eq << " N/m" << std::endl;
    
    TransferFunction G_mech({1}, {m_eq, b_eq, k_eq});
    
    std::cout << "\n  Both have same form: G(s) = K / (s² + 2ζωn·s + ωn²)" << std::endl;
    std::cout << "  This is the FUNDAMENTAL second-order system!" << std::endl;
    
    // ========================================================================
    // STEP 6: Analyze Resonance
    // ========================================================================
    std::cout << "\n▶ STEP 6: Resonance Analysis" << std::endl;
    std::cout << "  ───────────────────────────" << std::endl;
    
    // Quality factor
    double Q = 1 / (2 * zeta);
    double bandwidth = wn / Q;
    
    std::cout << "  Quality factor Q = 1/(2ζ) = " << Q << std::endl;
    std::cout << "  Bandwidth BW = ωn/Q = " << bandwidth << " rad/s" << std::endl;
    std::cout << "  Peak magnitude at resonance ≈ " << Q << " (for ζ << 1)" << std::endl;
    
    // ========================================================================
    // STEP 7: Simulation and Plotting
    // ========================================================================
    std::cout << "\n▶ STEP 7: Simulation" << std::endl;
    std::cout << "  ─────────────────────" << std::endl;
    
    double t_final = 0.05;  // 50 ms
    
    figure(1400, 1000);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 1: Step Response - Capacitor Voltage
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 2, 1);
    
    auto [t, v_out] = step(G_rlc, t_final);
    plot(t, v_out, "b-", {{"linewidth", "2"}, {"label", "V_C(t)"}});
    axhline(1.0, {{"color", "green"}, {"linestyle", "--"}, {"label", "V_in (1V step)"}});
    
    // Show envelope
    std::vector<double> env_upper, env_lower;
    for (double ti : t) {
        double env = std::exp(-zeta * wn * ti);
        env_upper.push_back(1 + env);
        env_lower.push_back(1 - env);
    }
    plot(t, env_upper, "r--", {{"alpha", "0.5"}, {"label", "Envelope"}});
    plot(t, env_lower, "r--", {{"alpha", "0.5"}});
    
    xlabel("Time [s]");
    ylabel("Capacitor Voltage [V]");
    title("Step Response: Capacitor Voltage");
    legend();
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 2: Current Response
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 2, 2);
    
    // Current = C * dV_C/dt (approximate using finite difference)
    std::vector<double> current;
    current.push_back(0);
    for (size_t i = 1; i < v_out.size(); ++i) {
        double dv = v_out[i] - v_out[i-1];
        double dt = t[i] - t[i-1];
        current.push_back(C * dv / dt);
    }
    
    plot(t, current, "m-", {{"linewidth", "2"}, {"label", "i(t) = C·dV_C/dt"}});
    xlabel("Time [s]");
    ylabel("Current [A]");
    title("Circuit Current");
    legend();
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 3: Bode Magnitude Plot
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 2, 3);
    
    std::vector<double> freq, mag_db;
    for (double f = 10; f <= 10000; f *= 1.1) {
        double w = 2 * M_PI * f;
        std::complex<double> s(0, w);
        std::complex<double> H = wn_sq / (s*s + two_zeta_wn*s + wn_sq);
        double mag = 20 * std::log10(std::abs(H));
        freq.push_back(f);
        mag_db.push_back(mag);
    }
    
    plot(freq, mag_db, "b-", {{"linewidth", "2"}, {"label", "Magnitude"}});
    axvline(f_resonance, {{"color", "red"}, {"linestyle", "--"}, {"label", "Resonant frequency"}});
    axhline(0, {{"color", "gray"}, {"linestyle", ":"}});
    axhline(-3, {{"color", "green"}, {"linestyle", ":"}, {"label", "-3 dB"}});
    
    xlabel("Frequency [Hz]");
    ylabel("Magnitude [dB]");
    title("Bode Magnitude Plot");
    legend();
    grid(true);
    xscale("log");
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 4: Effect of Damping (Different R values)
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 2, 4);
    
    std::vector<double> R_values = {5, 10, 31.6, 50, 100};  // 31.6 ≈ critical damping
    std::vector<std::string> colors = {"#E74C3C", "#E67E22", "#2ECC71", "#3498DB", "#9B59B6"};
    
    double R_critical = 2 * std::sqrt(L / C);  // Critical damping resistance
    
    for (size_t i = 0; i < R_values.size(); ++i) {
        double R_i = R_values[i];
        double zeta_i = (R_i / 2) * std::sqrt(C / L);
        
        TransferFunction G_i({wn_sq}, {1, R_i/L, wn_sq});
        auto [t_i, v_i] = step(G_i, t_final);
        
        std::ostringstream label;
        label << "R=" << R_i << "Ω (ζ=" << std::fixed << std::setprecision(2) << zeta_i << ")";
        plot(t_i, v_i, "-", {{"color", colors[i]}, {"linewidth", "2"}, {"label", label.str()}});
    }
    
    xlabel("Time [s]");
    ylabel("Voltage [V]");
    title("Effect of Resistance on Response");
    legend({{"loc", "lower right"}});
    grid(true);
    
    savefig("ch02_rlc_circuit.svg");
    std::cout << "  ✓ Generated: ch02_rlc_circuit.svg" << std::endl;
    
    // ========================================================================
    // Additional: Frequency Response for Different Q Values
    // ========================================================================
    figure(800, 600);
    
    std::vector<double> Q_values = {0.5, 1.0, 2.0, 5.0, 10.0};
    std::vector<std::string> q_colors = {"#3498DB", "#2ECC71", "#F1C40F", "#E67E22", "#E74C3C"};
    
    for (size_t i = 0; i < Q_values.size(); ++i) {
        double Q_i = Q_values[i];
        double zeta_i = 1 / (2 * Q_i);
        
        std::vector<double> freq_q, mag_q;
        for (double w_ratio = 0.1; w_ratio <= 10; w_ratio *= 1.05) {
            double w = w_ratio * wn;
            std::complex<double> s(0, w);
            std::complex<double> H = wn_sq / (s*s + 2*zeta_i*wn*s + wn_sq);
            double mag = 20 * std::log10(std::abs(H));
            freq_q.push_back(w_ratio);
            mag_q.push_back(mag);
        }
        
        std::ostringstream label;
        label << "Q = " << Q_i;
        plot(freq_q, mag_q, "-", {{"color", q_colors[i]}, {"linewidth", "2"}, {"label", label.str()}});
    }
    
    axvline(1.0, {{"color", "gray"}, {"linestyle", "--"}, {"alpha", "0.5"}});
    axhline(0, {{"color", "black"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    
    xlabel("Normalized Frequency ω/ωn");
    ylabel("Magnitude [dB]");
    title("Resonance Peak vs Quality Factor");
    legend();
    grid(true);
    xscale("log");
    
    savefig("ch02_quality_factor.svg");
    std::cout << "  ✓ Generated: ch02_quality_factor.svg" << std::endl;
    
    // ========================================================================
    // Physical Interpretation
    // ========================================================================
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║               PHYSICAL INTERPRETATION                        ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Energy in RLC Circuit:                                      ║" << std::endl;
    std::cout << "║  • Capacitor stores electric energy: E_C = ½CV²              ║" << std::endl;
    std::cout << "║  • Inductor stores magnetic energy:  E_L = ½Li²              ║" << std::endl;
    std::cout << "║  • Resistor dissipates energy:       P_R = i²R               ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  At resonance (ω = ωn):                                      ║" << std::endl;
    std::cout << "║  • Energy oscillates between L and C                         ║" << std::endl;
    std::cout << "║  • Impedance of L and C cancel                               ║" << std::endl;
    std::cout << "║  • Circuit acts purely resistive                             ║" << std::endl;
    std::cout << "║  • Maximum current flows through circuit                     ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Critical damping R_c = 2√(L/C) = " << std::fixed << std::setprecision(1) 
              << R_critical << " Ω              ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // ========================================================================
    // Design Rules
    // ========================================================================
    std::cout << "\n┌────────────────────────────────────────────────────────────────┐" << std::endl;
    std::cout << "│                      DESIGN RULES                              │" << std::endl;
    std::cout << "├────────────────────────────────────────────────────────────────┤" << std::endl;
    std::cout << "│                                                                │" << std::endl;
    std::cout << "│  For Sharp Resonance (High Q filter):                          │" << std::endl;
    std::cout << "│    • Use low R (small resistance)                              │" << std::endl;
    std::cout << "│    • Q = (1/R)·√(L/C)                                          │" << std::endl;
    std::cout << "│                                                                │" << std::endl;
    std::cout << "│  For Wide Bandwidth (Low Q filter):                            │" << std::endl;
    std::cout << "│    • Use high R (large resistance)                             │" << std::endl;
    std::cout << "│    • Bandwidth = R/L                                           │" << std::endl;
    std::cout << "│                                                                │" << std::endl;
    std::cout << "│  To shift resonant frequency:                                  │" << std::endl;
    std::cout << "│    • ωn = 1/√(LC)                                              │" << std::endl;
    std::cout << "│    • ↑ L or ↑ C → ↓ ωn (lower frequency)                       │" << std::endl;
    std::cout << "│    • ↓ L or ↓ C → ↑ ωn (higher frequency)                      │" << std::endl;
    std::cout << "│                                                                │" << std::endl;
    std::cout << "└────────────────────────────────────────────────────────────────┘" << std::endl;
    
    return 0;
}
