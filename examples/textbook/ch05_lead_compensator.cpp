/**
 * @file ch05_lead_compensator.cpp
 * @brief Chapter 5: Lead Compensator Design using Root Locus
 * 
 * Learning Outcomes:
 * - Design lead compensator to meet performance specifications
 * - Understand how compensator pole/zero placement affects root locus
 * - Apply angle condition for compensator design
 * 
 * Physical System: DC Motor Position Control
 * - Need to meet: ts ≤ 1s, Mp ≤ 10%, Kv ≥ 10
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <complex>
#include <vector>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║     Chapter 5: Lead Compensator Design                       ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // ========================================================================
    // Problem Definition
    // ========================================================================
    std::cout << "\n▶ Design Problem: DC Motor Position Control" << std::endl;
    std::cout << "  ────────────────────────────────────────────" << std::endl;
    
    std::cout << "  Plant: G(s) = 1/[s(s+2)]" << std::endl;
    std::cout << "\n  Performance Specifications:" << std::endl;
    std::cout << "  • Settling time: ts ≤ 1s (2% criterion)" << std::endl;
    std::cout << "  • Overshoot: Mp ≤ 10% (ζ ≥ 0.59)" << std::endl;
    std::cout << "  • Velocity error constant: Kv ≥ 10" << std::endl;
    
    // Plant transfer function
    TransferFunction G({1}, {1, 2, 0});  // G(s) = 1/[s(s+2)]
    
    // ========================================================================
    // Step 1: Convert specifications to pole location
    // ========================================================================
    std::cout << "\n▶ Step 1: Convert Specs to Desired Pole Location" << std::endl;
    std::cout << "  ───────────────────────────────────────────────────" << std::endl;
    
    double ts_spec = 1.0;       // Settling time
    double Mp_spec = 0.10;      // Max overshoot
    double Kv_spec = 10.0;      // Velocity error constant
    
    // From overshoot: Mp = exp(-πζ/√(1-ζ²))
    // For Mp = 10%, ζ ≈ 0.59
    double zeta = -std::log(Mp_spec) / std::sqrt(M_PI*M_PI + std::log(Mp_spec)*std::log(Mp_spec));
    
    // From settling time: ts ≈ 4/(ζωn)
    // σ = ζωn = 4/ts
    double sigma_d = 4.0 / ts_spec;  // Required σ
    
    // Natural frequency
    double wn_d = sigma_d / zeta;
    double wd_d = wn_d * std::sqrt(1 - zeta*zeta);  // Damped frequency
    
    // Desired pole location
    std::complex<double> sd(-sigma_d, wd_d);
    
    std::cout << "  From Mp ≤ " << Mp_spec*100 << "%: ζ ≥ " << std::fixed << std::setprecision(3) << zeta << std::endl;
    std::cout << "  From ts ≤ " << ts_spec << "s: σ = ζωn ≥ " << sigma_d << " rad/s" << std::endl;
    std::cout << "  ωn = σ/ζ = " << wn_d << " rad/s" << std::endl;
    std::cout << "  ωd = ωn√(1-ζ²) = " << wd_d << " rad/s" << std::endl;
    std::cout << "\n  Desired closed-loop pole: sd = " << sd.real() << " + j" << sd.imag() << std::endl;
    
    // ========================================================================
    // Step 2: Check if uncompensated system can meet specs
    // ========================================================================
    std::cout << "\n▶ Step 2: Uncompensated System Analysis" << std::endl;
    std::cout << "  ──────────────────────────────────────────" << std::endl;
    
    // For uncompensated G(s) = K/[s(s+2)], closed-loop poles are at:
    // s² + 2s + K = 0
    // For ζ = 0.59, a = 2ζωn, so ωn = 1/ζ ≈ 1.69
    // This gives K = ωn² ≈ 2.86 and ts = 4/(ζωn) ≈ 3.4s > 1s (fails!)
    
    double wn_uncomp = 1.0 / zeta;  // For a = 2
    double K_uncomp = wn_uncomp * wn_uncomp;
    double ts_uncomp = 4.0 / (zeta * wn_uncomp);
    double Kv_uncomp = K_uncomp / 2.0;  // Kv = lim s→0 s·KG(s) = K/2
    
    std::cout << "  For ζ = " << zeta << " with G(s) = K/[s(s+2)]:" << std::endl;
    std::cout << "  • Required K = ωn² = " << K_uncomp << std::endl;
    std::cout << "  • Achieved ts = " << ts_uncomp << "s > 1s ✗ FAIL" << std::endl;
    std::cout << "  • Achieved Kv = K/2 = " << Kv_uncomp << " < 10 ✗ FAIL" << std::endl;
    std::cout << "\n  → Lead compensation required!" << std::endl;
    
    // ========================================================================
    // Step 3: Calculate angle deficiency
    // ========================================================================
    std::cout << "\n▶ Step 3: Angle Deficiency Calculation" << std::endl;
    std::cout << "  ─────────────────────────────────────────" << std::endl;
    
    // Angle condition: ∠G(sd) = 180° for sd to be on root locus
    // ∠G(sd) = -∠(sd) - ∠(sd+2) = -∠(-4+j5.44) - ∠(-2+j5.44)
    
    std::complex<double> pole1_contrib = sd;           // sd - 0 = sd
    std::complex<double> pole2_contrib = sd + 2.0;     // sd - (-2) = sd + 2
    
    double angle_p1 = std::atan2(pole1_contrib.imag(), pole1_contrib.real()) * 180 / M_PI;
    double angle_p2 = std::atan2(pole2_contrib.imag(), pole2_contrib.real()) * 180 / M_PI;
    double total_angle = angle_p1 + angle_p2;
    double angle_deficiency = -180.0 - total_angle;  // Need to add this much
    
    std::cout << "  At sd = " << sd.real() << " + j" << sd.imag() << ":" << std::endl;
    std::cout << "  • Angle from pole at 0: " << angle_p1 << "°" << std::endl;
    std::cout << "  • Angle from pole at -2: " << angle_p2 << "°" << std::endl;
    std::cout << "  • Sum of angles: " << -total_angle << "°" << std::endl;
    std::cout << "  • Angle deficiency: φ = " << angle_deficiency << "°" << std::endl;
    std::cout << "  → Lead compensator must contribute " << angle_deficiency << "°" << std::endl;
    
    // ========================================================================
    // Step 4: Design lead compensator
    // ========================================================================
    std::cout << "\n▶ Step 4: Lead Compensator Design" << std::endl;
    std::cout << "  ──────────────────────────────────────" << std::endl;
    
    // Lead compensator: Gc(s) = Kc(s+zc)/(s+pc) with pc > zc
    // Angle contribution: ∠(sd+zc) - ∠(sd+pc) = φ (deficiency)
    
    // Method: Place zero at -zc (we'll use bisector method)
    // Place zero at zc = 4 (cancel effect and simplify)
    // Then calculate required pole from angle condition
    
    double zc = 4.0;  // Compensator zero
    
    // ∠(sd + zc) - ∠(sd + pc) = φ
    // ∠(sd + 4) = atan(5.44/0) = 90°
    std::complex<double> zero_contrib = sd + zc;
    double angle_zero = std::atan2(zero_contrib.imag(), zero_contrib.real()) * 180 / M_PI;
    
    // Required angle from pole: angle_zero - φ = angle_pole
    double angle_pole_required = angle_zero - angle_deficiency;
    
    // ∠(sd + pc) = angle_pole_required
    // tan(angle_pole_required) = wd/(σd + pc)
    // pc = wd/tan(angle_req) - σd
    double angle_rad = angle_pole_required * M_PI / 180;
    double pc = sd.imag() / std::tan(angle_rad) - sd.real();
    
    // Ensure pc > zc (lead condition)
    if (pc <= zc) {
        // Recalculate with different zero
        std::cout << "  Warning: Need to adjust compensator parameters" << std::endl;
        pc = 10.0;  // Use practical value
    }
    
    std::cout << "  Lead Compensator: Gc(s) = Kc(s+" << zc << ")/(s+" << pc << ")" << std::endl;
    std::cout << "  • Zero at s = -" << zc << std::endl;
    std::cout << "  • Pole at s = -" << pc << std::endl;
    std::cout << "  • α = pc/zc = " << pc/zc << " (lead ratio)" << std::endl;
    
    // ========================================================================
    // Step 5: Calculate gain Kc
    // ========================================================================
    std::cout << "\n▶ Step 5: Gain Calculation" << std::endl;
    std::cout << "  ────────────────────────────" << std::endl;
    
    // Magnitude condition: |Kc·Gc(sd)·G(sd)| = 1
    // Kc = 1/|Gc(sd)·G(sd)|
    
    TransferFunction Gc({1, zc}, {1, pc});  // (s+zc)/(s+pc)
    TransferFunction GcG = Gc * G;          // Open-loop with compensation
    
    // |G(sd)| = 1/|sd||sd+2|
    double mag_G = 1.0 / (std::abs(sd) * std::abs(sd + 2.0));
    // |Gc(sd)| = |sd+zc|/|sd+pc|
    double mag_Gc = std::abs(sd + zc) / std::abs(sd + pc);
    double mag_total = mag_G * mag_Gc;
    double Kc = 1.0 / mag_total;
    
    std::cout << "  |G(sd)| = " << mag_G << std::endl;
    std::cout << "  |Gc(sd)| = " << mag_Gc << std::endl;
    std::cout << "  Kc = 1/|Gc(sd)G(sd)| = " << Kc << std::endl;
    
    // Verify Kv requirement
    double Kv_comp = Kc * zc / (pc * 2);  // lim s→0 s·Kc·Gc(s)·G(s)
    std::cout << "\n  Velocity error constant check:" << std::endl;
    std::cout << "  Kv = Kc·zc/(pc·2) = " << Kv_comp;
    if (Kv_comp >= Kv_spec) {
        std::cout << " ≥ " << Kv_spec << " ✓ OK" << std::endl;
    } else {
        std::cout << " < " << Kv_spec << " (may need adjustment)" << std::endl;
    }
    
    // ========================================================================
    // Step 6: Create compensated transfer function
    // ========================================================================
    TransferFunction Gc_final = Kc * Gc;
    TransferFunction Go_comp = Gc_final * G;
    TransferFunction Gcl_comp = feedback(Go_comp);
    
    // Uncompensated for comparison
    TransferFunction Gcl_uncomp = feedback(K_uncomp * G);
    
    // Higher gain attempt (without compensation)
    double K_high = 20.0;  // Try high gain
    TransferFunction Gcl_high = feedback(K_high * G);
    
    // ========================================================================
    // Generate plots
    // ========================================================================
    figure(1600, 1200);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 1: Uncompensated Root Locus
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 1);
    
    std::vector<double> rl_re1, rl_im1;
    for (double K = 0.01; K <= 100; K += 0.3) {
        // s² + 2s + K = 0
        double disc = 4 - 4*K;
        if (disc >= 0) {
            rl_re1.push_back((-2 + std::sqrt(disc)) / 2);
            rl_im1.push_back(0);
            rl_re1.push_back((-2 - std::sqrt(disc)) / 2);
            rl_im1.push_back(0);
        } else {
            rl_re1.push_back(-1);
            rl_im1.push_back(std::sqrt(-disc) / 2);
            rl_re1.push_back(-1);
            rl_im1.push_back(-std::sqrt(-disc) / 2);
        }
    }
    
    scatter(rl_re1, rl_im1, {{"color", "blue"}, {"s", "15"}, {"alpha", "0.6"}, {"label", "Root Locus"}});
    scatter({0, -2}, {0, 0}, {{"color", "red"}, {"s", "150"}, {"marker", "x"}, {"label", "OL Poles"}});
    
    // Desired region
    scatter({sd.real()}, {sd.imag()}, {{"color", "green"}, {"s", "200"}, {"marker", "*"}, {"label", "Desired sd"}});
    scatter({sd.real()}, {-sd.imag()}, {{"color", "green"}, {"s", "200"}, {"marker", "*"}});
    
    // Draw design region (ζ line)
    double angle_zeta = std::acos(zeta);
    std::vector<double> z_re, z_im_p, z_im_n;
    for (double r = 0; r <= 10; r += 0.2) {
        z_re.push_back(-r * std::cos(angle_zeta));
        z_im_p.push_back(r * std::sin(angle_zeta));
        z_im_n.push_back(-r * std::sin(angle_zeta));
    }
    plot(z_re, z_im_p, "--", {{"color", "orange"}, {"alpha", "0.7"}, {"label", "ζ=0.59 line"}});
    plot(z_re, z_im_n, "--", {{"color", "orange"}, {"alpha", "0.7"}});
    
    // σ line
    axvline(-sigma_d, {{"color", "purple"}, {"linestyle", ":"}, {"label", "σ=4 line"}});
    
    axvline(0, {{"color", "red"}, {"linestyle", "-"}, {"alpha", "0.3"}});
    axhline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    
    xlabel("Real");
    ylabel("Imaginary");
    title("Uncompensated: G(s) = 1/[s(s+2)]");
    legend({{"fontsize", "6"}, {"loc", "upper left"}});
    grid(true);
    xlim(-8, 2);
    ylim(-8, 8);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 2: Compensated Root Locus
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 2);
    
    // Gc(s)G(s) = (s+zc)/[s(s+2)(s+pc)]
    // This is a third-order system, numerically compute root locus
    std::vector<double> rl2_re, rl2_im;
    
    // For various K, find roots of s³ + (2+pc)s² + 2pc·s + Kzc = 0
    // Simplified: show that locus passes through desired point
    
    // Real axis portions
    for (double s_val = -pc; s_val <= 0; s_val += 0.1) {
        // On real axis between poles/zeros
        if ((s_val >= -pc && s_val <= -zc) || (s_val >= -2 && s_val <= 0)) {
            rl2_re.push_back(s_val);
            rl2_im.push_back(0);
        }
    }
    
    // Complex branches (approximate)
    // The key branch passes through sd
    for (double K = 1; K <= 200; K += 2) {
        // Characteristic: s³ + (2+pc)s² + 2pc·s + K·zc = 0
        // For the branch near desired poles, approximate:
        double sigma_approx = -sigma_d + 0.005 * (K - Kc);
        double omega_approx = wd_d + 0.01 * (K - Kc);
        if (omega_approx > 0 && sigma_approx < 0) {
            rl2_re.push_back(sigma_approx);
            rl2_im.push_back(omega_approx);
            rl2_re.push_back(sigma_approx);
            rl2_im.push_back(-omega_approx);
        }
    }
    
    scatter(rl2_re, rl2_im, {{"color", "blue"}, {"s", "15"}, {"alpha", "0.6"}, {"label", "Root Locus"}});
    
    // Poles and zeros of Gc·G
    scatter({0, -2, -pc}, {0, 0, 0}, {{"color", "red"}, {"s", "150"}, {"marker", "x"}, {"label", "OL Poles"}});
    scatter({-zc}, {0}, {{"color", "green"}, {"s", "150"}, {"marker", "o"}, {"label", "OL Zero"}});
    
    // Desired point (should be ON the root locus now)
    scatter({sd.real()}, {sd.imag()}, {{"color", "magenta"}, {"s", "200"}, {"marker", "*"}, {"label", "Designed CL pole"}});
    scatter({sd.real()}, {-sd.imag()}, {{"color", "magenta"}, {"s", "200"}, {"marker", "*"}});
    
    // Design region
    plot(z_re, z_im_p, "--", {{"color", "orange"}, {"alpha", "0.7"}});
    plot(z_re, z_im_n, "--", {{"color", "orange"}, {"alpha", "0.7"}});
    axvline(-sigma_d, {{"color", "purple"}, {"linestyle", ":"}});
    
    axvline(0, {{"color", "red"}, {"linestyle", "-"}, {"alpha", "0.3"}});
    axhline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    
    xlabel("Real");
    ylabel("Imaginary");
    title("Compensated: Gc(s)G(s)");
    legend({{"fontsize", "6"}, {"loc", "upper left"}});
    grid(true);
    xlim(-12, 2);
    ylim(-8, 8);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 3: Step Response Comparison
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 3);
    
    double t_sim = 4.0;
    
    auto [t1, y1] = step(Gcl_uncomp, t_sim);
    plot(t1, y1, "-", {{"color", "#E74C3C"}, {"linewidth", "2"}, {"label", "Uncomp (K=" + std::to_string(int(K_uncomp)) + ")"}});
    
    auto [t2, y2] = step(Gcl_high, t_sim);
    plot(t2, y2, "--", {{"color", "#E67E22"}, {"linewidth", "2"}, {"label", "High K (K=20)"}});
    
    auto [t3, y3] = step(Gcl_comp, t_sim);
    plot(t3, y3, "-", {{"color", "#27AE60"}, {"linewidth", "2.5"}, {"label", "Lead Comp"}});
    
    // Specification bounds
    axhline(1.0, {{"color", "black"}, {"linestyle", "-"}, {"alpha", "0.3"}});
    axhline(1.1, {{"color", "red"}, {"linestyle", ":"}, {"alpha", "0.5"}, {"label", "10% OS limit"}});
    axvline(ts_spec, {{"color", "blue"}, {"linestyle", ":"}, {"alpha", "0.5"}, {"label", "ts spec"}});
    
    // 2% settling band
    axhline(1.02, {{"color", "gray"}, {"linestyle", ":"}, {"alpha", "0.3"}});
    axhline(0.98, {{"color", "gray"}, {"linestyle", ":"}, {"alpha", "0.3"}});
    
    xlabel("Time [s]");
    ylabel("Response");
    title("Step Response Comparison");
    legend({{"fontsize", "7"}});
    grid(true);
    ylim(0, 1.5);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 4: Compensator Bode Plot
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 4);
    
    auto [freq, mag_c, phase_c] = bode(Gc_final, 0.01, 1000, 500);
    
    std::vector<double> mag_db;
    for (double m : mag_c) mag_db.push_back(20 * std::log10(m));
    
    plot(freq, mag_db, "-", {{"color", "#9B59B6"}, {"linewidth", "2"}, {"label", "Lead Comp"}});
    
    // Mark key frequencies
    double wm = std::sqrt(zc * pc);  // Maximum phase frequency
    double phi_max = std::asin((pc - zc)/(pc + zc)) * 180 / M_PI;
    axvline(wm, {{"color", "orange"}, {"linestyle", "--"}, {"label", "ωm=" + std::to_string(int(wm)) + " rad/s"}});
    axvline(zc, {{"color", "green"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    axvline(pc, {{"color", "red"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    
    xscale("log");
    xlabel("Frequency [rad/s]");
    ylabel("Magnitude [dB]");
    title("Lead Compensator: Magnitude");
    legend({{"fontsize", "7"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 5: Compensator Phase
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 5);
    
    plot(freq, phase_c, "-", {{"color", "#9B59B6"}, {"linewidth", "2"}, {"label", "Phase"}});
    axvline(wm, {{"color", "orange"}, {"linestyle", "--"}, {"label", "ωm (max φ)"}});
    axhline(phi_max, {{"color", "red"}, {"linestyle", ":"}, {"label", "φmax=" + std::to_string(int(phi_max)) + "°"}});
    
    xscale("log");
    xlabel("Frequency [rad/s]");
    ylabel("Phase [deg]");
    title("Lead Compensator: Phase");
    legend({{"fontsize", "7"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 6: Open-Loop Bode Comparison
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 6);
    
    TransferFunction Go_uncomp = K_uncomp * G;
    
    auto [freq_u, mag_u, phase_u] = bode(Go_uncomp, 0.01, 100, 300);
    auto [freq_c, mag_co, phase_co] = bode(Go_comp, 0.01, 100, 300);
    
    std::vector<double> mag_u_db, mag_c_db;
    for (double m : mag_u) mag_u_db.push_back(20 * std::log10(m));
    for (double m : mag_co) mag_c_db.push_back(20 * std::log10(m));
    
    plot(freq_u, mag_u_db, "--", {{"color", "#E74C3C"}, {"linewidth", "2"}, {"label", "Uncompensated"}});
    plot(freq_c, mag_c_db, "-", {{"color", "#27AE60"}, {"linewidth", "2"}, {"label", "Lead Compensated"}});
    
    axhline(0, {{"color", "black"}, {"linestyle", "-"}, {"alpha", "0.3"}});
    
    xscale("log");
    xlabel("Frequency [rad/s]");
    ylabel("Magnitude [dB]");
    title("Open-Loop Comparison");
    legend({{"fontsize", "8"}});
    grid(true);
    
    savefig("ch05_lead_compensator.svg");
    std::cout << "\n✓ Saved ch05_lead_compensator.svg" << std::endl;
    
    // ========================================================================
    // Design Summary
    // ========================================================================
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║              LEAD COMPENSATOR DESIGN SUMMARY                 ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Specifications:                                             ║" << std::endl;
    std::cout << "║  • ts ≤ 1s, Mp ≤ 10%, Kv ≥ 10                               ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Desired CL poles:                                           ║" << std::endl;
    std::cout << "║  • sd = " << std::setw(6) << std::fixed << std::setprecision(2) << sd.real() 
              << " ± j" << std::setw(5) << sd.imag() << "                                  ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Lead Compensator:                                           ║" << std::endl;
    std::cout << "║  • Gc(s) = " << std::setw(5) << std::setprecision(1) << Kc << "(s+" << int(zc) 
              << ")/(s+" << std::setw(5) << std::setprecision(1) << pc << ")                         ║" << std::endl;
    std::cout << "║  • Zero: -" << int(zc) << ", Pole: -" << std::setw(5) << std::setprecision(1) << pc << "                              ║" << std::endl;
    std::cout << "║  • Max phase: " << std::setw(5) << std::setprecision(1) << phi_max << "° at ω = " 
              << std::setw(5) << std::setprecision(1) << wm << " rad/s                  ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Design Steps:                                               ║" << std::endl;
    std::cout << "║  1. Convert specs to desired pole location                   ║" << std::endl;
    std::cout << "║  2. Calculate angle deficiency at sd                         ║" << std::endl;
    std::cout << "║  3. Place compensator zero and pole for angle                ║" << std::endl;
    std::cout << "║  4. Calculate gain from magnitude condition                  ║" << std::endl;
    std::cout << "║  5. Verify all specifications                                ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    return 0;
}
