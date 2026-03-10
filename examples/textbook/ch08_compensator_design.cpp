/**
 * @file ch08_compensator_design.cpp
 * @brief Chapter 8: Lead, Lag, and Lead-Lag Compensator Design
 * 
 * Learning Outcomes:
 * - Design lead compensator for phase margin improvement
 * - Design lag compensator for steady-state error reduction
 * - Combine lead-lag for comprehensive performance
 * 
 * Physical System: Antenna Positioning System
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>

using namespace cppplot;
using namespace cppplot::control;

// Find gain crossover frequency
double findGainCrossover(const std::vector<double>& freq, const std::vector<double>& mag) {
    for (size_t i = 1; i < freq.size(); ++i) {
        if ((mag[i-1] >= 1.0 && mag[i] < 1.0)) {
            double ratio = (1.0 - mag[i-1]) / (mag[i] - mag[i-1]);
            return freq[i-1] + ratio * (freq[i] - freq[i-1]);
        }
    }
    return -1;
}

// Find phase at specific frequency
double findPhaseAtFreq(const std::vector<double>& freq, const std::vector<double>& phase, double target_freq) {
    for (size_t i = 1; i < freq.size(); ++i) {
        if (freq[i] >= target_freq) {
            double ratio = (target_freq - freq[i-1]) / (freq[i] - freq[i-1]);
            return phase[i-1] + ratio * (phase[i] - phase[i-1]);
        }
    }
    return phase.back();
}

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║     Chapter 8: Compensator Design                            ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // ========================================================================
    // Plant Definition: Antenna Positioning System
    // ========================================================================
    std::cout << "\n▶ System: Antenna Position Control" << std::endl;
    std::cout << "  ────────────────────────────────────" << std::endl;
    
    // G(s) = 10/[s(s+2)] - Type-1 system
    TransferFunction G({10}, {1, 2, 0});
    
    std::cout << "  Plant: G(s) = 10/[s(s+2)]" << std::endl;
    std::cout << "  Type: 1 (one integrator)" << std::endl;
    std::cout << "  Current Kv = lim s→0 sG(s) = 5" << std::endl;
    
    figure(1800, 1200);
    
    double w_min = 0.01, w_max = 1000;
    int n_points = 500;
    
    // ========================================================================
    // Analyze Uncompensated System
    // ========================================================================
    auto [freq_u, mag_u, phase_u] = bode(G, w_min, w_max, n_points);
    
    double wgc_u = findGainCrossover(freq_u, mag_u);
    double phase_at_wgc_u = findPhaseAtFreq(freq_u, phase_u, wgc_u);
    double PM_u = 180 + phase_at_wgc_u;
    
    std::cout << "\n  Uncompensated Analysis:" << std::endl;
    std::cout << "  • Gain crossover: ωgc = " << std::fixed << std::setprecision(2) << wgc_u << " rad/s" << std::endl;
    std::cout << "  • Phase margin: PM = " << PM_u << "°" << std::endl;
    
    // ========================================================================
    // Design Specifications
    // ========================================================================
    std::cout << "\n▶ Design Requirements:" << std::endl;
    std::cout << "  ─────────────────────────" << std::endl;
    std::cout << "  • Phase margin: PM ≥ 50°" << std::endl;
    std::cout << "  • Velocity constant: Kv ≥ 20" << std::endl;
    std::cout << "  • Bandwidth: ωgc ≥ 5 rad/s" << std::endl;
    
    // ========================================================================
    // LEAD COMPENSATOR DESIGN
    // ========================================================================
    std::cout << "\n▶ Lead Compensator Design" << std::endl;
    std::cout << "  ──────────────────────────────" << std::endl;
    
    // Step 1: Determine required phase lead
    double PM_desired = 50.0;
    double wgc_new = 5.0;
    
    // Get current phase at new crossover
    double phase_at_new_wgc = findPhaseAtFreq(freq_u, phase_u, wgc_new);
    double PM_at_new_wgc = 180 + phase_at_new_wgc;
    
    // Need additional phase (with safety margin for gain increase)
    double phi_m = PM_desired - PM_at_new_wgc + 10;  // 10° safety margin
    double phi_m_rad = phi_m * M_PI / 180;
    
    std::cout << "  Step 1: Phase lead needed = " << phi_m << "°" << std::endl;
    
    // Step 2: Calculate alpha
    double alpha = (1 - std::sin(phi_m_rad)) / (1 + std::sin(phi_m_rad));
    std::cout << "  Step 2: α = " << std::setprecision(3) << alpha << std::endl;
    
    // Step 3: Calculate zero and pole (place ωm = ωgc_new)
    double zc = wgc_new * std::sqrt(alpha);
    double pc = wgc_new / std::sqrt(alpha);
    
    std::cout << "  Step 3: zc = " << zc << ", pc = " << pc << std::endl;
    
    // Step 4: Create lead compensator
    TransferFunction Gc_lead({1, zc}, {1, pc});
    
    // Step 5: Adjust gain to achieve |GcG| = 1 at ωgc_new
    auto [freq_lead_test, mag_lead_test, phase_lead_test] = bode(Gc_lead * G, w_min, w_max, n_points);
    double mag_at_wgc = 1;
    for (size_t i = 0; i < freq_lead_test.size(); ++i) {
        if (freq_lead_test[i] >= wgc_new) {
            mag_at_wgc = mag_lead_test[i];
            break;
        }
    }
    double Kc_lead = 1.0 / mag_at_wgc;
    
    std::cout << "  Step 4: Kc = " << Kc_lead << std::endl;
    
    TransferFunction Gc_lead_final = Kc_lead * Gc_lead;
    TransferFunction Go_lead = Gc_lead_final * G;
    
    // Verify
    auto [freq_lead, mag_lead, phase_lead] = bode(Go_lead, w_min, w_max, n_points);
    double wgc_lead = findGainCrossover(freq_lead, mag_lead);
    double PM_lead = 180 + findPhaseAtFreq(freq_lead, phase_lead, wgc_lead);
    double Kv_lead = Kc_lead * 10 / 2;  // lim s→0 sGcG = Kc * 10/2
    
    std::cout << "\n  Lead Compensated Results:" << std::endl;
    std::cout << "  • New ωgc = " << wgc_lead << " rad/s ✓" << std::endl;
    std::cout << "  • New PM = " << PM_lead << "° " << (PM_lead >= 50 ? "✓" : "✗") << std::endl;
    std::cout << "  • Kv = " << Kv_lead << (Kv_lead >= 20 ? " ✓" : " (need lag to increase)") << std::endl;
    
    // ========================================================================
    // LAG COMPENSATOR DESIGN (to increase Kv)
    // ========================================================================
    std::cout << "\n▶ Lag Compensator Design (for Kv)" << std::endl;
    std::cout << "  ───────────────────────────────────" << std::endl;
    
    // Need to increase Kv from current value to 20
    double Kv_required = 20.0;
    double Kv_current = Kv_lead;
    double beta = Kv_required / Kv_current;
    
    std::cout << "  β = Kv_req / Kv_curr = " << beta << std::endl;
    
    // Place lag compensator well below ωgc (at ωgc/10)
    double zc_lag = wgc_lead / 10;
    double pc_lag = zc_lag / beta;
    
    std::cout << "  zc_lag = " << zc_lag << ", pc_lag = " << pc_lag << std::endl;
    
    TransferFunction Gc_lag({1, zc_lag}, {1, pc_lag});
    
    // ========================================================================
    // LEAD-LAG COMPENSATOR
    // ========================================================================
    std::cout << "\n▶ Combined Lead-Lag Compensator" << std::endl;
    std::cout << "  ─────────────────────────────────" << std::endl;
    
    TransferFunction Gc_leadlag = Gc_lead_final * Gc_lag;
    TransferFunction Go_leadlag = Gc_leadlag * G;
    
    auto [freq_ll, mag_ll, phase_ll] = bode(Go_leadlag, w_min, w_max, n_points);
    double wgc_ll = findGainCrossover(freq_ll, mag_ll);
    double PM_ll = 180 + findPhaseAtFreq(freq_ll, phase_ll, wgc_ll);
    double Kv_ll = Kc_lead * beta * 10 / 2;
    
    std::cout << "  Final Results:" << std::endl;
    std::cout << "  • ωgc = " << wgc_ll << " rad/s" << std::endl;
    std::cout << "  • PM = " << PM_ll << "°" << std::endl;
    std::cout << "  • Kv = " << Kv_ll << std::endl;
    
    // ========================================================================
    // PLOTS
    // ========================================================================
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 1: Lead Compensator Bode
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 1);
    
    auto [freq_c, mag_c, phase_c] = bode(Gc_lead_final, w_min, w_max, n_points);
    std::vector<double> mag_c_db;
    for (double m : mag_c) mag_c_db.push_back(20 * std::log10(m));
    
    semilogx(freq_c, mag_c_db, "b-", {{"linewidth", "2"}, {"label", "Lead Comp"}});
    
    axvline(zc, {{"color", "green"}, {"linestyle", ":"}, {"label", "zero"}});
    axvline(pc, {{"color", "red"}, {"linestyle", ":"}, {"label", "pole"}});
    axvline(wgc_new, {{"color", "orange"}, {"linestyle", "--"}, {"label", "ωm"}});
    
    xlabel("Frequency [rad/s]");
    ylabel("Magnitude [dB]");
    title("(a) Lead Compensator");
    legend({{"fontsize", "6"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 2: Lead Compensator Phase
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 2);
    
    semilogx(freq_c, phase_c, "b-", {{"linewidth", "2"}, {"label", "Phase"}});
    axhline(phi_m, {{"color", "red"}, {"linestyle", "--"}, {"label", "φmax"}});
    axvline(wgc_new, {{"color", "orange"}, {"linestyle", "--"}, {"label", "ωm"}});
    
    xlabel("Frequency [rad/s]");
    ylabel("Phase [deg]");
    title("(b) Lead Phase Contribution");
    legend({{"fontsize", "7"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 3: Lag Compensator Bode
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 3);
    
    auto [freq_lag, mag_lag, phase_lag] = bode(Gc_lag, w_min, w_max, n_points);
    std::vector<double> mag_lag_db;
    for (double m : mag_lag) mag_lag_db.push_back(20 * std::log10(m));
    
    semilogx(freq_lag, mag_lag_db, "r-", {{"linewidth", "2"}, {"label", "Magnitude"}});
    
    axvline(zc_lag, {{"color", "green"}, {"linestyle", ":"}, {"label", "zero"}});
    axvline(pc_lag, {{"color", "red"}, {"linestyle", ":"}, {"label", "pole"}});
    
    xlabel("Frequency [rad/s]");
    ylabel("Magnitude [dB]");
    title("(c) Lag Compensator");
    legend({{"fontsize", "7"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 4: Open-Loop Bode Comparison (Magnitude)
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 4);
    
    std::vector<double> mag_u_db, mag_lead_db, mag_ll_db;
    for (double m : mag_u) mag_u_db.push_back(20 * std::log10(m));
    for (double m : mag_lead) mag_lead_db.push_back(20 * std::log10(m));
    for (double m : mag_ll) mag_ll_db.push_back(20 * std::log10(m));
    
    semilogx(freq_u, mag_u_db, "k--", {{"linewidth", "1.5"}, {"label", "Uncompensated"}});
    semilogx(freq_lead, mag_lead_db, "b-", {{"linewidth", "2"}, {"label", "Lead"}});
    semilogx(freq_ll, mag_ll_db, "r-", {{"linewidth", "2"}, {"label", "Lead-Lag"}});
    
    axhline(0, {{"color", "gray"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    
    xlabel("Frequency [rad/s]");
    ylabel("Magnitude [dB]");
    title("(d) Open-Loop Magnitude");
    legend({{"fontsize", "7"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 5: Open-Loop Bode Comparison (Phase)
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 5);
    
    semilogx(freq_u, phase_u, "k--", {{"linewidth", "1.5"}, {"label", "Uncompensated"}});
    semilogx(freq_lead, phase_lead, "b-", {{"linewidth", "2"}, {"label", "Lead"}});
    semilogx(freq_ll, phase_ll, "r-", {{"linewidth", "2"}, {"label", "Lead-Lag"}});
    
    axhline(-180, {{"color", "red"}, {"linestyle", "-"}, {"linewidth", "1"}, {"alpha", "0.5"}});
    axhline(-180 + PM_desired, {{"color", "green"}, {"linestyle", ":"}, {"label", "PM=50° target"}});
    
    xlabel("Frequency [rad/s]");
    ylabel("Phase [deg]");
    title("(e) Open-Loop Phase");
    legend({{"fontsize", "6"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 6: Closed-Loop Step Response
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 6);
    
    TransferFunction Gcl_u = feedback(G);
    TransferFunction Gcl_lead = feedback(Go_lead);
    TransferFunction Gcl_ll = feedback(Go_leadlag);
    
    double t_sim = 5.0;
    
    auto [t_u, y_u] = step(Gcl_u, t_sim);
    auto [t_lead, y_lead] = step(Gcl_lead, t_sim);
    auto [t_ll, y_ll] = step(Gcl_ll, t_sim);
    
    plot(t_u, y_u, "k--", {{"linewidth", "1.5"}, {"label", "Uncompensated"}});
    plot(t_lead, y_lead, "b-", {{"linewidth", "2"}, {"label", "Lead"}});
    plot(t_ll, y_ll, "r-", {{"linewidth", "2"}, {"label", "Lead-Lag"}});
    
    axhline(1.0, {{"color", "gray"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    
    xlabel("Time [s]");
    ylabel("Response");
    title("(f) Closed-Loop Step Response");
    legend({{"fontsize", "7"}});
    grid(true);
    
    savefig("ch08_compensator_design.svg");
    std::cout << "\n✓ Saved ch08_compensator_design.svg" << std::endl;
    
    // ========================================================================
    // Summary
    // ========================================================================
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║              COMPENSATOR DESIGN SUMMARY                      ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  LEAD COMPENSATOR: Gc(s) = Kc(s+zc)/(s+pc), pc > zc         ║" << std::endl;
    std::cout << "║  • Purpose: Increase phase margin, increase bandwidth        ║" << std::endl;
    std::cout << "║  • φmax = arcsin[(1-α)/(1+α)] at ωm = √(zc·pc)              ║" << std::endl;
    std::cout << "║  • Design: Place ωm at desired ωgc                          ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  LAG COMPENSATOR: Gc(s) = Kc(s+zc)/(s+pc), pc < zc          ║" << std::endl;
    std::cout << "║  • Purpose: Increase low-freq gain, reduce steady-state err  ║" << std::endl;
    std::cout << "║  • Place zc, pc << ωgc to minimize phase impact             ║" << std::endl;
    std::cout << "║  • β = zc/pc determines gain increase                        ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  LEAD-LAG: Combines both benefits                            ║" << std::endl;
    std::cout << "║  • Design lag first for steady-state                         ║" << std::endl;
    std::cout << "║  • Design lead for phase margin                              ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Results:                        Uncomp   Lead    Lead-Lag   ║" << std::endl;
    std::cout << "║  ─────────────────────────────────────────────────────────   ║" << std::endl;
    std::cout << "║  Phase Margin [°]              " << std::setw(6) << std::setprecision(1) << PM_u 
              << "   " << std::setw(6) << PM_lead << "  " << std::setw(6) << PM_ll << "     ║" << std::endl;
    std::cout << "║  Gain Crossover [rad/s]        " << std::setw(6) << wgc_u 
              << "   " << std::setw(6) << wgc_lead << "  " << std::setw(6) << wgc_ll << "     ║" << std::endl;
    std::cout << "║  Kv                            " << std::setw(6) << 5.0 
              << "   " << std::setw(6) << Kv_lead << "  " << std::setw(6) << Kv_ll << "     ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    return 0;
}
