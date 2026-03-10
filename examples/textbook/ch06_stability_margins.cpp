/**
 * @file ch06_stability_margins.cpp
 * @brief Chapter 6: Gain Margin and Phase Margin Analysis
 * 
 * Learning Outcomes:
 * - Calculate gain and phase margins from Bode plots
 * - Understand the relationship between margins and stability
 * - Relate phase margin to transient response
 * 
 * Physical System: DC Motor Position Control
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <algorithm>

using namespace cppplot;
using namespace cppplot::control;

// Find gain crossover frequency (where |G| = 1)
double findGainCrossover(const std::vector<double>& freq, const std::vector<double>& mag) {
    for (size_t i = 1; i < freq.size(); ++i) {
        if ((mag[i-1] >= 1.0 && mag[i] < 1.0) || (mag[i-1] <= 1.0 && mag[i] > 1.0)) {
            // Linear interpolation
            double ratio = (1.0 - mag[i-1]) / (mag[i] - mag[i-1]);
            return freq[i-1] + ratio * (freq[i] - freq[i-1]);
        }
    }
    return -1;  // Not found
}

// Find phase crossover frequency (where phase = -180)
double findPhaseCrossover(const std::vector<double>& freq, const std::vector<double>& phase) {
    for (size_t i = 1; i < freq.size(); ++i) {
        if ((phase[i-1] >= -180 && phase[i] < -180) || (phase[i-1] <= -180 && phase[i] > -180)) {
            double ratio = (-180 - phase[i-1]) / (phase[i] - phase[i-1]);
            return freq[i-1] + ratio * (freq[i] - freq[i-1]);
        }
    }
    return -1;  // Not found (infinite GM)
}

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║     Chapter 6: Stability Margins                             ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // ========================================================================
    // System Definition: G(s) = K/[s(s+2)(s+5)]
    // ========================================================================
    std::cout << "\n▶ Open-Loop System: G(s) = K/[s(s+2)(s+5)]" << std::endl;
    std::cout << "  ──────────────────────────────────────────────" << std::endl;
    
    // This is a Type-1 system with three poles
    // Critical: This system CAN become unstable for large K
    
    figure(1600, 1200);
    
    double w_min = 0.01, w_max = 100;
    int n_points = 1000;
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 1: Bode Magnitude for Different K
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 1);
    
    std::vector<double> K_values = {5, 20, 70, 200};
    std::vector<std::string> colors = {"#2ECC71", "#3498DB", "#E67E22", "#E74C3C"};
    
    for (size_t i = 0; i < K_values.size(); ++i) {
        double K = K_values[i];
        
        // G(s) = K/[s(s+2)(s+5)] = K/(s³ + 7s² + 10s)
        TransferFunction G({K}, {1, 7, 10, 0});
        auto [freq, mag, phase] = bode(G, w_min, w_max, n_points);
        
        std::vector<double> mag_db;
        for (double m : mag) mag_db.push_back(20 * std::log10(m));
        
        std::ostringstream label;
        label << "K = " << int(K);
        semilogx(freq, mag_db, "-", {{"color", colors[i]}, {"linewidth", "2"}, {"label", label.str()}});
    }
    
    axhline(0, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.5"}, {"label", "0 dB line"}});
    
    xlabel("Frequency [rad/s]");
    ylabel("Magnitude [dB]");
    title("(a) Open-Loop Magnitude");
    legend({{"fontsize", "7"}});
    grid(true);
    ylim(-60, 60);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 2: Bode Phase for Different K
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 2);
    
    for (size_t i = 0; i < K_values.size(); ++i) {
        double K = K_values[i];
        TransferFunction G({K}, {1, 7, 10, 0});
        auto [freq, mag, phase] = bode(G, w_min, w_max, n_points);
        
        std::ostringstream label;
        label << "K = " << int(K);
        semilogx(freq, phase, "-", {{"color", colors[i]}, {"linewidth", "2"}, {"label", label.str()}});
    }
    
    axhline(-180, {{"color", "red"}, {"linestyle", "--"}, {"alpha", "0.7"}, {"label", "-180° line"}});
    
    xlabel("Frequency [rad/s]");
    ylabel("Phase [deg]");
    title("(b) Open-Loop Phase");
    legend({{"fontsize", "7"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 3: Detailed Margin Analysis (K = 20)
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 3);
    
    double K_design = 20.0;
    TransferFunction G_design({K_design}, {1, 7, 10, 0});
    auto [freq_d, mag_d, phase_d] = bode(G_design, w_min, w_max, n_points);
    
    std::vector<double> mag_d_db;
    for (double m : mag_d) mag_d_db.push_back(20 * std::log10(m));
    
    // Find crossover frequencies
    double wgc = findGainCrossover(freq_d, mag_d);
    double wpc = findPhaseCrossover(freq_d, phase_d);
    
    // Find magnitude at phase crossover
    double mag_at_wpc = -999;
    for (size_t i = 0; i < freq_d.size(); ++i) {
        if (std::abs(freq_d[i] - wpc) < 0.1) {
            mag_at_wpc = 20 * std::log10(mag_d[i]);
            break;
        }
    }
    
    // Find phase at gain crossover
    double phase_at_wgc = -999;
    for (size_t i = 0; i < freq_d.size(); ++i) {
        if (std::abs(freq_d[i] - wgc) < 0.1) {
            phase_at_wgc = phase_d[i];
            break;
        }
    }
    
    double GM = -mag_at_wpc;  // dB
    double PM = 180 + phase_at_wgc;  // degrees
    
    std::cout << "\n  For K = " << K_design << ":" << std::endl;
    std::cout << "  • Gain crossover: ωgc = " << std::fixed << std::setprecision(2) << wgc << " rad/s" << std::endl;
    std::cout << "  • Phase crossover: ωpc = " << wpc << " rad/s" << std::endl;
    std::cout << "  • Gain Margin: GM = " << GM << " dB" << std::endl;
    std::cout << "  • Phase Margin: PM = " << PM << "°" << std::endl;
    
    semilogx(freq_d, mag_d_db, "b-", {{"linewidth", "2"}, {"label", "Magnitude"}});
    
    axhline(0, {{"color", "gray"}, {"linestyle", "--"}, {"alpha", "0.5"}});
    
    // Mark gain margin
    if (wpc > 0 && mag_at_wpc > -100) {
        axvline(wpc, {{"color", "red"}, {"linestyle", ":"}, {"alpha", "0.7"}});
        scatter({wpc}, {mag_at_wpc}, {{"color", "red"}, {"s", "100"}, {"marker", "v"}});
        // GM arrow (approximate)
        plot({wpc, wpc}, {mag_at_wpc, 0}, "r-", {{"linewidth", "2"}, {"label", "GM"}});
    }
    
    // Mark gain crossover
    if (wgc > 0) {
        axvline(wgc, {{"color", "green"}, {"linestyle", ":"}, {"alpha", "0.7"}});
        scatter({wgc}, {0}, {{"color", "green"}, {"s", "100"}, {"marker", "o"}, {"label", "ωgc"}});
    }
    
    xlabel("Frequency [rad/s]");
    ylabel("Magnitude [dB]");
    title("(c) Gain Margin (K=20)");
    legend({{"fontsize", "6"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 4: Phase Margin Detail
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 4);
    
    semilogx(freq_d, phase_d, "b-", {{"linewidth", "2"}, {"label", "Phase"}});
    
    axhline(-180, {{"color", "red"}, {"linestyle", "--"}, {"alpha", "0.7"}});
    
    // Mark phase margin
    if (wgc > 0 && phase_at_wgc > -300) {
        axvline(wgc, {{"color", "green"}, {"linestyle", ":"}, {"alpha", "0.7"}});
        scatter({wgc}, {phase_at_wgc}, {{"color", "green"}, {"s", "100"}, {"marker", "o"}});
        // PM arrow
        plot({wgc, wgc}, {phase_at_wgc, -180}, "g-", {{"linewidth", "2"}, {"label", "PM"}});
    }
    
    // Mark phase crossover
    if (wpc > 0) {
        axvline(wpc, {{"color", "red"}, {"linestyle", ":"}, {"alpha", "0.7"}});
        scatter({wpc}, {-180}, {{"color", "red"}, {"s", "100"}, {"marker", "v"}, {"label", "ωpc"}});
    }
    
    xlabel("Frequency [rad/s]");
    ylabel("Phase [deg]");
    title("(d) Phase Margin (K=20)");
    legend({{"fontsize", "6"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 5: Step Response vs K
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 5);
    
    double t_sim = 10.0;
    
    for (size_t i = 0; i < K_values.size(); ++i) {
        double K = K_values[i];
        TransferFunction G({K}, {1, 7, 10, 0});
        TransferFunction Gcl = feedback(G);
        auto [t, y] = step(Gcl, t_sim);
        
        // Clip unstable responses
        std::vector<double> y_clip;
        for (double val : y) {
            y_clip.push_back(std::max(-0.5, std::min(2.5, val)));
        }
        
        std::ostringstream label;
        label << "K = " << int(K);
        if (K >= 70) label << " (unstable)";
        plot(t, y_clip, "-", {{"color", colors[i]}, {"linewidth", "2"}, {"label", label.str()}});
    }
    
    axhline(1.0, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.3"}});
    
    xlabel("Time [s]");
    ylabel("Response");
    title("(e) Closed-Loop Step Response");
    legend({{"fontsize", "7"}});
    grid(true);
    ylim(-0.5, 2.5);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 6: Margin vs K
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 6);
    
    std::vector<double> K_range, GM_range, PM_range;
    
    for (double K = 1; K <= 100; K += 2) {
        TransferFunction G({K}, {1, 7, 10, 0});
        auto [freq, mag, phase] = bode(G, w_min, w_max, n_points);
        
        double wgc_k = findGainCrossover(freq, mag);
        double wpc_k = findPhaseCrossover(freq, phase);
        
        // Find GM
        double mag_at_wpc_k = 0;
        for (size_t i = 0; i < freq.size(); ++i) {
            if (wpc_k > 0 && std::abs(freq[i] - wpc_k) < 0.1) {
                mag_at_wpc_k = mag[i];
                break;
            }
        }
        double GM_k = (wpc_k > 0 && mag_at_wpc_k > 0) ? -20 * std::log10(mag_at_wpc_k) : 50;
        
        // Find PM
        double phase_at_wgc_k = -90;
        for (size_t i = 0; i < freq.size(); ++i) {
            if (wgc_k > 0 && std::abs(freq[i] - wgc_k) < 0.1) {
                phase_at_wgc_k = phase[i];
                break;
            }
        }
        double PM_k = (wgc_k > 0) ? 180 + phase_at_wgc_k : 90;
        
        K_range.push_back(K);
        GM_range.push_back(std::min(GM_k, 40.0));
        PM_range.push_back(std::max(PM_k, -30.0));
    }
    
    plot(K_range, GM_range, "r-", {{"linewidth", "2"}, {"label", "Gain Margin"}});
    plot(K_range, PM_range, "b-", {{"linewidth", "2"}, {"label", "Phase Margin"}});
    
    axhline(0, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.5"}, {"label", "Stability limit"}});
    axhline(6, {{"color", "orange"}, {"linestyle", ":"}, {"alpha", "0.5"}, {"label", "6 dB / 45° typical"}});
    axhline(45, {{"color", "orange"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    
    // Critical K (Routh-Hurwitz: K_crit = 70)
    double K_crit = 70.0;  // From s³ + 7s² + 10s + K = 0 stability analysis
    axvline(K_crit, {{"color", "red"}, {"linestyle", "-"}, {"linewidth", "2"}, {"alpha", "0.7"}, {"label", "K_crit=70"}});
    
    xlabel("Gain K");
    ylabel("Margin [dB or deg]");
    title("(f) Stability Margins vs K");
    legend({{"fontsize", "6"}});
    grid(true);
    xlim(0, 100);
    ylim(-30, 50);
    
    savefig("ch06_stability_margins.svg");
    std::cout << "\n✓ Saved ch06_stability_margins.svg" << std::endl;
    
    // ========================================================================
    // Routh-Hurwitz Verification
    // ========================================================================
    std::cout << "\n▶ Routh-Hurwitz Stability Analysis" << std::endl;
    std::cout << "  ──────────────────────────────────────" << std::endl;
    std::cout << "  Characteristic equation: s³ + 7s² + 10s + K = 0" << std::endl;
    std::cout << "\n  Routh array:" << std::endl;
    std::cout << "  s³ |  1    10" << std::endl;
    std::cout << "  s² |  7    K" << std::endl;
    std::cout << "  s¹ | (70-K)/7" << std::endl;
    std::cout << "  s⁰ |  K" << std::endl;
    std::cout << "\n  For stability: K > 0 AND K < 70" << std::endl;
    std::cout << "  → Critical gain: K_crit = 70" << std::endl;
    
    // ========================================================================
    // Summary
    // ========================================================================
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║              STABILITY MARGIN SUMMARY                        ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Gain Margin (GM):                                           ║" << std::endl;
    std::cout << "║  • Find ωpc where phase = -180°                              ║" << std::endl;
    std::cout << "║  • GM = -20log|G(jωpc)|                                      ║" << std::endl;
    std::cout << "║  • GM > 0: Stable | GM < 0: Unstable                         ║" << std::endl;
    std::cout << "║  • Typical requirement: GM ≥ 6 dB                            ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Phase Margin (PM):                                          ║" << std::endl;
    std::cout << "║  • Find ωgc where |G| = 1 (0 dB)                             ║" << std::endl;
    std::cout << "║  • PM = 180° + ∠G(jωgc)                                      ║" << std::endl;
    std::cout << "║  • PM > 0: Stable | PM < 0: Unstable                         ║" << std::endl;
    std::cout << "║  • Typical requirement: PM ≥ 45°                             ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Relationship to Time Domain:                                ║" << std::endl;
    std::cout << "║  • ζ ≈ PM/100° (for dominant second-order)                   ║" << std::endl;
    std::cout << "║  • PM = 45° → ζ ≈ 0.45 → Mp ≈ 23%                           ║" << std::endl;
    std::cout << "║  • PM = 60° → ζ ≈ 0.60 → Mp ≈ 10%                           ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    return 0;
}
