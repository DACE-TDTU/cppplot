/**
 * @file ch06_bode_basics.cpp
 * @brief Chapter 6: Bode Plot Fundamentals
 * 
 * Learning Outcomes:
 * - Draw Bode plots for basic transfer function elements
 * - Combine elements using asymptotic approximation
 * - Understand magnitude and phase relationships
 * 
 * Physical System: Various control system elements
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║     Chapter 6: Bode Plot Fundamentals                        ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    figure(1800, 1200);
    
    double w_min = 0.01, w_max = 1000;
    int n_points = 500;
    
    // ========================================================================
    // Basic Elements
    // ========================================================================
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 1: Constant Gain K
    // ─────────────────────────────────────────────────────────────────
    subplot(3, 4, 1);
    
    std::vector<double> K_values = {0.1, 1, 10};
    std::vector<std::string> colors = {"#3498DB", "#2ECC71", "#E74C3C"};
    
    for (size_t i = 0; i < K_values.size(); ++i) {
        TransferFunction G_k({K_values[i]}, {1});
        auto [freq, mag, phase] = bode(G_k, w_min, w_max, n_points);
        
        std::vector<double> mag_db;
        for (double m : mag) mag_db.push_back(20 * std::log10(m));
        
        std::ostringstream label;
        label << "K = " << K_values[i];
        semilogx(freq, mag_db, "-", {{"color", colors[i]}, {"linewidth", "2"}, {"label", label.str()}});
    }
    
    xlabel("ω [rad/s]");
    ylabel("Mag [dB]");
    title("(a) Constant Gain");
    legend({{"fontsize", "7"}});
    grid(true);
    ylim(-25, 25);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 2: Integrator 1/s
    // ─────────────────────────────────────────────────────────────────
    subplot(3, 4, 2);
    
    TransferFunction G_int({1}, {1, 0});  // 1/s
    auto [freq_int, mag_int, phase_int] = bode(G_int, w_min, w_max, n_points);
    
    std::vector<double> mag_int_db;
    for (double m : mag_int) mag_int_db.push_back(20 * std::log10(m));
    
    semilogx(freq_int, mag_int_db, "b-", {{"linewidth", "2"}, {"label", "1/s"}});
    
    // -20 dB/decade reference line
    std::vector<double> ref_freq = {0.1, 10};
    std::vector<double> ref_mag = {20, -20};
    semilogx(ref_freq, ref_mag, "r--", {{"alpha", "0.5"}, {"label", "-20 dB/dec"}});
    
    xlabel("ω [rad/s]");
    ylabel("Mag [dB]");
    title("(b) Integrator 1/s");
    legend({{"fontsize", "7"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 3: Differentiator s
    // ─────────────────────────────────────────────────────────────────
    subplot(3, 4, 3);
    
    TransferFunction G_diff({1, 0}, {1});  // s
    auto [freq_diff, mag_diff, phase_diff] = bode(G_diff, w_min, w_max, n_points);
    
    std::vector<double> mag_diff_db;
    for (double m : mag_diff) mag_diff_db.push_back(20 * std::log10(m));
    
    semilogx(freq_diff, mag_diff_db, "b-", {{"linewidth", "2"}, {"label", "s"}});
    
    std::vector<double> ref_mag_p = {-20, 20};
    semilogx(ref_freq, ref_mag_p, "r--", {{"alpha", "0.5"}, {"label", "+20 dB/dec"}});
    
    xlabel("ω [rad/s]");
    ylabel("Mag [dB]");
    title("(c) Differentiator s");
    legend({{"fontsize", "7"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 4: Phase of Integrator and Differentiator
    // ─────────────────────────────────────────────────────────────────
    subplot(3, 4, 4);
    
    semilogx(freq_int, phase_int, "b-", {{"linewidth", "2"}, {"label", "1/s: -90°"}});
    semilogx(freq_diff, phase_diff, "r-", {{"linewidth", "2"}, {"label", "s: +90°"}});
    axhline(0, {{"color", "black"}, {"linestyle", ":"}, {"alpha", "0.3"}});
    
    xlabel("ω [rad/s]");
    ylabel("Phase [deg]");
    title("(d) Phase: Integrator & Diff");
    legend({{"fontsize", "7"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 5: First-Order Pole: 1/(1+s/ωp)
    // ─────────────────────────────────────────────────────────────────
    subplot(3, 4, 5);
    
    double wp = 10.0;  // Corner frequency
    TransferFunction G_pole({1}, {1.0/wp, 1});  // 1/(s/wp + 1) = wp/(s+wp)
    auto [freq_p, mag_p, phase_p] = bode(G_pole, w_min, w_max, n_points);
    
    std::vector<double> mag_p_db;
    for (double m : mag_p) mag_p_db.push_back(20 * std::log10(m));
    
    semilogx(freq_p, mag_p_db, "b-", {{"linewidth", "2"}, {"label", "Actual"}});
    
    // Asymptotic approximation
    std::vector<double> asym_freq = {0.01, wp, 1000};
    std::vector<double> asym_mag = {0, 0, -40};  // -20 dB/dec after corner
    semilogx(asym_freq, asym_mag, "r--", {{"linewidth", "1.5"}, {"label", "Asymptote"}});
    
    // Mark corner frequency
    axvline(wp, {{"color", "green"}, {"linestyle", ":"}, {"alpha", "0.7"}});
    scatter({wp}, {-3.0}, {{"color", "green"}, {"s", "100"}, {"marker", "o"}, {"label", "ωp=10 (-3dB)"}});
    
    xlabel("ω [rad/s]");
    ylabel("Mag [dB]");
    title("(e) First-Order Pole");
    legend({{"fontsize", "6"}, {"loc", "lower left"}});
    grid(true);
    ylim(-45, 5);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 6: Phase of First-Order Pole
    // ─────────────────────────────────────────────────────────────────
    subplot(3, 4, 6);
    
    semilogx(freq_p, phase_p, "b-", {{"linewidth", "2"}, {"label", "Actual"}});
    
    // Asymptotic phase
    std::vector<double> asym_phase_freq = {0.01, wp/10, wp, wp*10, 1000};
    std::vector<double> asym_phase_val = {0, 0, -45, -90, -90};
    semilogx(asym_phase_freq, asym_phase_val, "r--", {{"linewidth", "1.5"}, {"label", "Asymptote"}});
    
    axvline(wp, {{"color", "green"}, {"linestyle", ":"}, {"alpha", "0.7"}});
    scatter({wp}, {-45.0}, {{"color", "green"}, {"s", "100"}, {"marker", "o"}, {"label", "ωp (-45°)"}});
    
    xlabel("ω [rad/s]");
    ylabel("Phase [deg]");
    title("(f) Phase of First-Order Pole");
    legend({{"fontsize", "6"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 7: First-Order Zero: (1+s/ωz)
    // ─────────────────────────────────────────────────────────────────
    subplot(3, 4, 7);
    
    double wz = 10.0;
    TransferFunction G_zero({1.0/wz, 1}, {1});  // s/wz + 1
    auto [freq_z, mag_z, phase_z] = bode(G_zero, w_min, w_max, n_points);
    
    std::vector<double> mag_z_db;
    for (double m : mag_z) mag_z_db.push_back(20 * std::log10(m));
    
    semilogx(freq_z, mag_z_db, "b-", {{"linewidth", "2"}, {"label", "Actual"}});
    
    // Asymptotic
    std::vector<double> asym_z_freq = {0.01, wz, 1000};
    std::vector<double> asym_z_mag = {0, 0, 40};
    semilogx(asym_z_freq, asym_z_mag, "r--", {{"linewidth", "1.5"}, {"label", "Asymptote"}});
    
    axvline(wz, {{"color", "green"}, {"linestyle", ":"}, {"alpha", "0.7"}});
    scatter({wz}, {3.0}, {{"color", "green"}, {"s", "100"}, {"marker", "o"}, {"label", "ωz=10 (+3dB)"}});
    
    xlabel("ω [rad/s]");
    ylabel("Mag [dB]");
    title("(g) First-Order Zero");
    legend({{"fontsize", "6"}, {"loc", "upper left"}});
    grid(true);
    ylim(-5, 45);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 8: Phase of First-Order Zero
    // ─────────────────────────────────────────────────────────────────
    subplot(3, 4, 8);
    
    semilogx(freq_z, phase_z, "b-", {{"linewidth", "2"}, {"label", "Actual"}});
    
    // Asymptotic phase
    std::vector<double> asym_pz_val = {0, 0, 45, 90, 90};
    semilogx(asym_phase_freq, asym_pz_val, "r--", {{"linewidth", "1.5"}, {"label", "Asymptote"}});
    
    axvline(wz, {{"color", "green"}, {"linestyle", ":"}, {"alpha", "0.7"}});
    scatter({wz}, {45.0}, {{"color", "green"}, {"s", "100"}, {"marker", "o"}, {"label", "ωz (+45°)"}});
    
    xlabel("ω [rad/s]");
    ylabel("Phase [deg]");
    title("(h) Phase of First-Order Zero");
    legend({{"fontsize", "6"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 9: Second-Order System (varying ζ)
    // ─────────────────────────────────────────────────────────────────
    subplot(3, 4, 9);
    
    double wn = 10.0;
    std::vector<double> zeta_vals = {0.1, 0.3, 0.5, 0.707, 1.0};
    std::vector<std::string> zeta_colors = {"#E74C3C", "#E67E22", "#F1C40F", "#2ECC71", "#3498DB"};
    
    for (size_t i = 0; i < zeta_vals.size(); ++i) {
        double z = zeta_vals[i];
        // G(s) = ωn²/(s² + 2ζωns + ωn²)
        TransferFunction G_2nd({wn*wn}, {1, 2*z*wn, wn*wn});
        auto [freq_2, mag_2, phase_2] = bode(G_2nd, w_min, w_max, n_points);
        
        std::vector<double> mag_2_db;
        for (double m : mag_2) mag_2_db.push_back(20 * std::log10(m));
        
        std::ostringstream label;
        label << "ζ = " << std::fixed << std::setprecision(2) << z;
        semilogx(freq_2, mag_2_db, "-", {{"color", zeta_colors[i]}, {"linewidth", "2"}, {"label", label.str()}});
    }
    
    axvline(wn, {{"color", "gray"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    
    xlabel("ω [rad/s]");
    ylabel("Mag [dB]");
    title("(i) Second-Order: Magnitude");
    legend({{"fontsize", "6"}, {"loc", "lower left"}});
    grid(true);
    ylim(-60, 20);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 10: Second-Order Phase
    // ─────────────────────────────────────────────────────────────────
    subplot(3, 4, 10);
    
    for (size_t i = 0; i < zeta_vals.size(); ++i) {
        double z = zeta_vals[i];
        TransferFunction G_2nd({wn*wn}, {1, 2*z*wn, wn*wn});
        auto [freq_2, mag_2, phase_2] = bode(G_2nd, w_min, w_max, n_points);
        
        std::ostringstream label;
        label << "ζ = " << std::fixed << std::setprecision(2) << z;
        semilogx(freq_2, phase_2, "-", {{"color", zeta_colors[i]}, {"linewidth", "2"}, {"label", label.str()}});
    }
    
    axvline(wn, {{"color", "gray"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    axhline(-90, {{"color", "gray"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    
    xlabel("ω [rad/s]");
    ylabel("Phase [deg]");
    title("(j) Second-Order: Phase");
    legend({{"fontsize", "6"}, {"loc", "lower left"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 11: Combined System Example
    // ─────────────────────────────────────────────────────────────────
    subplot(3, 4, 11);
    
    // G(s) = 10(s+5) / [s(s+1)(s+50)]
    // K = 10, zero at s=-5, poles at s=0,-1,-50
    TransferFunction G_combined({10, 50}, {1, 51, 50, 0});
    auto [freq_c, mag_c, phase_c] = bode(G_combined, w_min, w_max, n_points);
    
    std::vector<double> mag_c_db;
    for (double m : mag_c) mag_c_db.push_back(20 * std::log10(m));
    
    semilogx(freq_c, mag_c_db, "b-", {{"linewidth", "2"}, {"label", "G(s)"}});
    
    // Mark corner frequencies
    axvline(1, {{"color", "red"}, {"linestyle", ":"}, {"alpha", "0.5"}, {"label", "ωp=1"}});
    axvline(5, {{"color", "green"}, {"linestyle", ":"}, {"alpha", "0.5"}, {"label", "ωz=5"}});
    axvline(50, {{"color", "orange"}, {"linestyle", ":"}, {"alpha", "0.5"}, {"label", "ωp=50"}});
    
    xlabel("ω [rad/s]");
    ylabel("Mag [dB]");
    title("(k) Combined: 10(s+5)/[s(s+1)(s+50)]");
    legend({{"fontsize", "5"}, {"loc", "lower left"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 12: Combined System Phase
    // ─────────────────────────────────────────────────────────────────
    subplot(3, 4, 12);
    
    semilogx(freq_c, phase_c, "b-", {{"linewidth", "2"}, {"label", "Phase"}});
    
    axvline(1, {{"color", "red"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    axvline(5, {{"color", "green"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    axvline(50, {{"color", "orange"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    
    xlabel("ω [rad/s]");
    ylabel("Phase [deg]");
    title("(l) Combined System: Phase");
    legend({{"fontsize", "7"}});
    grid(true);
    
    savefig("ch06_bode_basics.svg");
    std::cout << "\n✓ Saved ch06_bode_basics.svg" << std::endl;
    
    // ========================================================================
    // Summary
    // ========================================================================
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                  BODE PLOT ELEMENT SUMMARY                   ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Element              Magnitude            Phase             ║" << std::endl;
    std::cout << "║  ──────────────────────────────────────────────────────────  ║" << std::endl;
    std::cout << "║  K (constant)         20log|K| dB (flat)   0° or 180°        ║" << std::endl;
    std::cout << "║  s (diff)             +20 dB/decade        +90°              ║" << std::endl;
    std::cout << "║  1/s (integ)          -20 dB/decade        -90°              ║" << std::endl;
    std::cout << "║  1/(1+s/ωp)           -20 dB/dec (ω>ωp)    0→-90°            ║" << std::endl;
    std::cout << "║  1+s/ωz               +20 dB/dec (ω>ωz)    0→+90°            ║" << std::endl;
    std::cout << "║  ωn²/(s²+2ζωns+ωn²)   -40 dB/dec (ω>ωn)    0→-180°           ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Key Points:                                                 ║" << std::endl;
    std::cout << "║  • Products in G(s) → Sum in dB                              ║" << std::endl;
    std::cout << "║  • Corner freq: -3dB (pole) or +3dB (zero)                   ║" << std::endl;
    std::cout << "║  • Phase at corner: ±45°                                     ║" << std::endl;
    std::cout << "║  • Resonance peak when ζ < 0.707                             ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    return 0;
}
