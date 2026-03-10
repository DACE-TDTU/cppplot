/**
 * @file ch07_nyquist_basics.cpp
 * @brief Chapter 7: Nyquist Plot Fundamentals
 * 
 * Learning Outcomes:
 * - Draw Nyquist plots for various transfer functions
 * - Understand the relationship between encirclements and stability
 * - Apply Nyquist criterion to assess closed-loop stability
 * 
 * Physical System: Various control system examples
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <complex>

using namespace cppplot;
using namespace cppplot::control;

// Compute G(jω) for a transfer function
std::complex<double> evalTF(const TransferFunction& G, double omega) {
    std::complex<double> s(0, omega);
    
    // Evaluate numerator
    std::complex<double> num(0, 0);
    auto num_coeffs = G.num();
    for (size_t i = 0; i < num_coeffs.size(); ++i) {
        num += num_coeffs[i] * std::pow(s, num_coeffs.size() - 1 - i);
    }
    
    // Evaluate denominator
    std::complex<double> den(0, 0);
    auto den_coeffs = G.den();
    for (size_t i = 0; i < den_coeffs.size(); ++i) {
        den += den_coeffs[i] * std::pow(s, den_coeffs.size() - 1 - i);
    }
    
    return num / den;
}

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║     Chapter 7: Nyquist Plot Fundamentals                     ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    figure(1600, 1200);
    
    // ========================================================================
    // Example 1: First-Order System G(s) = K/(s+a)
    // ========================================================================
    std::cout << "\n▶ Example 1: First-Order System G(s) = 2/(s+1)" << std::endl;
    std::cout << "  ─────────────────────────────────────────────────" << std::endl;
    
    subplot(2, 3, 1);
    
    TransferFunction G1({2}, {1, 1});  // G(s) = 2/(s+1)
    
    std::vector<double> re1_pos, im1_pos, re1_neg, im1_neg;
    
    // ω: 0 → ∞ (positive frequencies)
    for (double w = 0.001; w <= 100; w *= 1.05) {
        std::complex<double> G_jw = evalTF(G1, w);
        re1_pos.push_back(G_jw.real());
        im1_pos.push_back(G_jw.imag());
    }
    
    // ω: -∞ → 0 (negative frequencies - mirror)
    for (double w = -100; w <= -0.001; w /= 1.05) {
        std::complex<double> G_jw = evalTF(G1, w);
        re1_neg.push_back(G_jw.real());
        im1_neg.push_back(G_jw.imag());
    }
    
    plot(re1_pos, im1_pos, "b-", {{"linewidth", "2"}, {"label", "ω > 0"}});
    plot(re1_neg, im1_neg, "b--", {{"linewidth", "1.5"}, {"label", "ω < 0"}});
    
    // Mark key points
    scatter({2.0}, {0}, {{"color", "green"}, {"s", "100"}, {"marker", "o"}, {"label", "ω=0"}});
    scatter({0}, {0}, {{"color", "red"}, {"s", "100"}, {"marker", "s"}, {"label", "ω→∞"}});
    
    // Critical point -1
    scatter({-1}, {0}, {{"color", "red"}, {"s", "150"}, {"marker", "x"}, {"label", "-1+j0"}});
    
    axhline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    axvline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    
    xlabel("Real");
    ylabel("Imaginary");
    title("(a) G(s) = 2/(s+1)");
    legend({{"fontsize", "6"}, {"loc", "lower right"}});
    grid(true);
    xlim(-1.5, 2.5);
    ylim(-1.5, 1.5);
    
    std::cout << "  P = 0 (no RHP poles)" << std::endl;
    std::cout << "  Encirclements of -1: N = 0" << std::endl;
    std::cout << "  Z = N + P = 0 → STABLE for all K > 0" << std::endl;
    
    // ========================================================================
    // Example 2: Type-1 System G(s) = K/[s(s+a)]
    // ========================================================================
    std::cout << "\n▶ Example 2: Type-1 System G(s) = 1/[s(s+1)]" << std::endl;
    std::cout << "  ──────────────────────────────────────────────────" << std::endl;
    
    subplot(2, 3, 2);
    
    TransferFunction G2({1}, {1, 1, 0});  // G(s) = 1/[s(s+1)]
    
    std::vector<double> re2_pos, im2_pos, re2_neg, im2_neg;
    
    // ω: 0+ → ∞
    for (double w = 0.01; w <= 100; w *= 1.03) {
        std::complex<double> G_jw = evalTF(G2, w);
        re2_pos.push_back(std::max(-10.0, std::min(10.0, G_jw.real())));
        im2_pos.push_back(std::max(-10.0, std::min(10.0, G_jw.imag())));
    }
    
    // ω: -∞ → 0-
    for (double w = -100; w <= -0.01; w /= 1.03) {
        std::complex<double> G_jw = evalTF(G2, w);
        re2_neg.push_back(std::max(-10.0, std::min(10.0, G_jw.real())));
        im2_neg.push_back(std::max(-10.0, std::min(10.0, G_jw.imag())));
    }
    
    plot(re2_pos, im2_pos, "b-", {{"linewidth", "2"}, {"label", "ω > 0"}});
    plot(re2_neg, im2_neg, "b--", {{"linewidth", "1.5"}, {"label", "ω < 0"}});
    
    // Draw semicircle around origin (for pole at s=0)
    std::vector<double> semi_re, semi_im;
    double R = 5;  // Large radius to show infinite semicircle
    for (int i = 0; i <= 50; ++i) {
        double angle = -M_PI/2 + i * M_PI / 50;
        semi_re.push_back(R * std::cos(angle));
        semi_im.push_back(R * std::sin(angle));
    }
    plot(semi_re, semi_im, "g--", {{"linewidth", "1"}, {"alpha", "0.5"}, {"label", "s=0 detour"}});
    
    // Critical point -1
    scatter({-1}, {0}, {{"color", "red"}, {"s", "150"}, {"marker", "x"}, {"label", "-1+j0"}});
    
    // Arrow showing direction
    scatter({-0.5}, {-1.5}, {{"color", "blue"}, {"s", "80"}, {"marker", ">"}});
    
    axhline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    axvline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    
    xlabel("Real");
    ylabel("Imaginary");
    title("(b) G(s) = 1/[s(s+1)]");
    legend({{"fontsize", "6"}, {"loc", "lower left"}});
    grid(true);
    xlim(-3, 3);
    ylim(-5, 5);
    
    std::cout << "  P = 0 (pole at s=0 is ON contour, not in RHP)" << std::endl;
    std::cout << "  Comes from -j∞, goes to 0, semicircle to +j∞" << std::endl;
    std::cout << "  N = 0 → STABLE for all K > 0" << std::endl;
    
    // ========================================================================
    // Example 3: Third-Order System (Can be Unstable)
    // ========================================================================
    std::cout << "\n▶ Example 3: Third-Order G(s) = K/[s(s+1)(s+2)]" << std::endl;
    std::cout << "  ──────────────────────────────────────────────────────" << std::endl;
    
    subplot(2, 3, 3);
    
    double K3 = 3.0;  // Below critical
    TransferFunction G3({K3}, {1, 3, 2, 0});
    
    std::vector<double> re3_pos, im3_pos;
    
    for (double w = 0.01; w <= 100; w *= 1.02) {
        std::complex<double> G_jw = evalTF(G3, w);
        re3_pos.push_back(std::max(-5.0, std::min(5.0, G_jw.real())));
        im3_pos.push_back(std::max(-5.0, std::min(5.0, G_jw.imag())));
    }
    
    // Mirror for negative frequencies
    std::vector<double> re3_neg, im3_neg;
    for (size_t i = 0; i < re3_pos.size(); ++i) {
        re3_neg.push_back(re3_pos[i]);
        im3_neg.push_back(-im3_pos[i]);
    }
    
    plot(re3_pos, im3_pos, "b-", {{"linewidth", "2"}, {"label", "K=3 (stable)"}});
    plot(re3_neg, im3_neg, "b--", {{"linewidth", "1.5"}});
    
    // Higher K (unstable)
    K3 = 8.0;
    TransferFunction G3_high({K3}, {1, 3, 2, 0});
    
    std::vector<double> re3h_pos, im3h_pos;
    for (double w = 0.01; w <= 100; w *= 1.02) {
        std::complex<double> G_jw = evalTF(G3_high, w);
        re3h_pos.push_back(std::max(-5.0, std::min(5.0, G_jw.real())));
        im3h_pos.push_back(std::max(-5.0, std::min(5.0, G_jw.imag())));
    }
    
    std::vector<double> re3h_neg, im3h_neg;
    for (size_t i = 0; i < re3h_pos.size(); ++i) {
        re3h_neg.push_back(re3h_pos[i]);
        im3h_neg.push_back(-im3h_pos[i]);
    }
    
    plot(re3h_pos, im3h_pos, "r-", {{"linewidth", "2"}, {"label", "K=8 (unstable)"}});
    plot(re3h_neg, im3h_neg, "r--", {{"linewidth", "1.5"}});
    
    // Critical point
    scatter({-1}, {0}, {{"color", "black"}, {"s", "200"}, {"marker", "x"}});
    text(-1.1, 0.3, "-1", {{"fontsize", "10"}});
    
    axhline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    axvline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    
    xlabel("Real");
    ylabel("Imaginary");
    title("(c) G(s) = K/[s(s+1)(s+2)]");
    legend({{"fontsize", "7"}});
    grid(true);
    xlim(-3, 1);
    ylim(-2, 2);
    
    // Critical K by Routh: K_crit = 6
    std::cout << "  Routh-Hurwitz: K_crit = 6" << std::endl;
    std::cout << "  K = 3: Does not encircle -1 → STABLE" << std::endl;
    std::cout << "  K = 8: Encircles -1 clockwise → UNSTABLE" << std::endl;
    
    // ========================================================================
    // Example 4: System with RHP Pole
    // ========================================================================
    std::cout << "\n▶ Example 4: Unstable Plant G(s) = 1/[s(s-1)]" << std::endl;
    std::cout << "  ────────────────────────────────────────────────" << std::endl;
    
    subplot(2, 3, 4);
    
    TransferFunction G4({1}, {1, -1, 0});  // G(s) = 1/[s(s-1)]
    
    std::vector<double> re4_pos, im4_pos;
    for (double w = 0.01; w <= 100; w *= 1.02) {
        std::complex<double> G_jw = evalTF(G4, w);
        re4_pos.push_back(std::max(-5.0, std::min(5.0, G_jw.real())));
        im4_pos.push_back(std::max(-5.0, std::min(5.0, G_jw.imag())));
    }
    
    std::vector<double> re4_neg, im4_neg;
    for (size_t i = 0; i < re4_pos.size(); ++i) {
        re4_neg.push_back(re4_pos[i]);
        im4_neg.push_back(-im4_pos[i]);
    }
    
    plot(re4_pos, im4_pos, "b-", {{"linewidth", "2"}, {"label", "ω > 0"}});
    plot(re4_neg, im4_neg, "b--", {{"linewidth", "1.5"}, {"label", "ω < 0"}});
    
    // Critical point
    scatter({-1}, {0}, {{"color", "red"}, {"s", "150"}, {"marker", "x"}, {"label", "-1+j0"}});
    
    axhline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    axvline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    
    xlabel("Real");
    ylabel("Imaginary");
    title("(d) G(s) = 1/[s(s-1)] (P=1)");
    legend({{"fontsize", "7"}});
    grid(true);
    xlim(-3, 1);
    ylim(-3, 3);
    
    std::cout << "  P = 1 (pole at s = +1)" << std::endl;
    std::cout << "  Need N = 1 CCW encirclement of -1 for stability" << std::endl;
    std::cout << "  For small K: N = 0 → Z = 1 → UNSTABLE" << std::endl;
    std::cout << "  Need K > 1 for N = 1 CCW → Z = 0 → STABLE" << std::endl;
    
    // ========================================================================
    // Example 5: Gain and Phase Margin on Nyquist
    // ========================================================================
    std::cout << "\n▶ Example 5: Margins on Nyquist Plot" << std::endl;
    std::cout << "  ─────────────────────────────────────────" << std::endl;
    
    subplot(2, 3, 5);
    
    double K5 = 10.0;
    TransferFunction G5({K5}, {1, 6, 11, 6, 0});  // G(s) = 10/[s(s+1)(s+2)(s+3)]
    
    std::vector<double> re5_pos, im5_pos;
    double wgc = -1, wpc = -1;
    double re_at_wpc = 0;
    
    for (double w = 0.01; w <= 100; w *= 1.02) {
        std::complex<double> G_jw = evalTF(G5, w);
        double re = G_jw.real();
        double im = G_jw.imag();
        
        re5_pos.push_back(re);
        im5_pos.push_back(im);
        
        // Check for |G| = 1 crossover
        double mag = std::abs(G_jw);
        if (wgc < 0 && mag < 1 && w > 0.1) {
            wgc = w;
        }
        
        // Check for phase = -180° crossover (imaginary part crosses zero from below)
        if (im > -0.01 && im < 0.01 && re < 0 && w > 0.5) {
            if (wpc < 0) {
                wpc = w;
                re_at_wpc = re;
            }
        }
    }
    
    std::vector<double> re5_neg, im5_neg;
    for (size_t i = 0; i < re5_pos.size(); ++i) {
        re5_neg.push_back(re5_pos[i]);
        im5_neg.push_back(-im5_pos[i]);
    }
    
    plot(re5_pos, im5_pos, "b-", {{"linewidth", "2"}, {"label", "Nyquist"}});
    plot(re5_neg, im5_neg, "b--", {{"linewidth", "1.5"}});
    
    // Unit circle
    std::vector<double> unit_re, unit_im;
    for (int i = 0; i <= 100; ++i) {
        double angle = i * 2 * M_PI / 100;
        unit_re.push_back(std::cos(angle));
        unit_im.push_back(std::sin(angle));
    }
    plot(unit_re, unit_im, "g--", {{"linewidth", "1"}, {"alpha", "0.5"}, {"label", "|G|=1 circle"}});
    
    // Mark -1 point
    scatter({-1}, {0}, {{"color", "red"}, {"s", "150"}, {"marker", "x"}, {"label", "-1+j0"}});
    
    // Gain margin (distance from -1 to where curve crosses negative real axis)
    if (re_at_wpc < 0) {
        plot({-1, re_at_wpc}, {0, 0}, "r-", {{"linewidth", "2"}, {"label", "GM"}});
        scatter({re_at_wpc}, {0}, {{"color", "red"}, {"s", "80"}, {"marker", "o"}});
    }
    
    // Phase margin (angle from negative real axis to |G|=1 point)
    std::complex<double> G_at_wgc = evalTF(G5, wgc);
    if (wgc > 0) {
        plot({0, G_at_wgc.real()}, {0, G_at_wgc.imag()}, "m-", {{"linewidth", "2"}, {"label", "PM"}});
        scatter({G_at_wgc.real()}, {G_at_wgc.imag()}, {{"color", "magenta"}, {"s", "80"}, {"marker", "o"}});
    }
    
    axhline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    axvline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    
    xlabel("Real");
    ylabel("Imaginary");
    title("(e) Margins on Nyquist");
    legend({{"fontsize", "6"}, {"loc", "upper left"}});
    grid(true);
    xlim(-2, 1);
    ylim(-1.5, 1.5);
    
    double GM = (re_at_wpc != 0) ? -1.0 / re_at_wpc : 999;
    double GM_dB = 20 * std::log10(GM);
    double PM = 180 + std::atan2(G_at_wgc.imag(), G_at_wgc.real()) * 180 / M_PI;
    
    std::cout << "  Gain crossover: ωgc ≈ " << std::fixed << std::setprecision(2) << wgc << " rad/s" << std::endl;
    std::cout << "  Phase crossover: ωpc ≈ " << wpc << " rad/s" << std::endl;
    std::cout << "  Gain Margin: " << GM_dB << " dB" << std::endl;
    std::cout << "  Phase Margin: " << PM << "°" << std::endl;
    
    // ========================================================================
    // Example 6: Closed-Loop Response Comparison
    // ========================================================================
    subplot(2, 3, 6);
    
    std::cout << "\n▶ Example 6: Closed-Loop Step Response Verification" << std::endl;
    std::cout << "  ───────────────────────────────────────────────────────" << std::endl;
    
    std::vector<double> K_test = {3, 6, 9};  // Below, at, above critical
    std::vector<std::string> colors = {"#2ECC71", "#F1C40F", "#E74C3C"};
    
    TransferFunction G_base({1}, {1, 3, 2, 0});  // 1/[s(s+1)(s+2)]
    
    for (size_t i = 0; i < K_test.size(); ++i) {
        double K = K_test[i];
        TransferFunction G_cl = feedback(K * G_base);
        auto [t, y] = step(G_cl, 15.0);
        
        // Clip for visualization
        std::vector<double> y_clip;
        for (double val : y) {
            y_clip.push_back(std::max(-1.0, std::min(3.0, val)));
        }
        
        std::ostringstream label;
        label << "K = " << int(K);
        if (K == 6) label << " (critical)";
        else if (K > 6) label << " (unstable)";
        plot(t, y_clip, "-", {{"color", colors[i]}, {"linewidth", "2"}, {"label", label.str()}});
    }
    
    axhline(1.0, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.3"}});
    
    xlabel("Time [s]");
    ylabel("Response");
    title("(f) Step Response Verification");
    legend({{"fontsize", "7"}});
    grid(true);
    ylim(-1, 3);
    
    std::cout << "  K = 3: Below critical → Stable" << std::endl;
    std::cout << "  K = 6: Critical → Marginally stable" << std::endl;
    std::cout << "  K = 9: Above critical → Unstable" << std::endl;
    
    savefig("ch07_nyquist_basics.svg");
    std::cout << "\n✓ Saved ch07_nyquist_basics.svg" << std::endl;
    
    // ========================================================================
    // Summary
    // ========================================================================
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                NYQUIST CRITERION SUMMARY                     ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Nyquist Stability Criterion:                                ║" << std::endl;
    std::cout << "║  Z = N + P                                                   ║" << std::endl;
    std::cout << "║  • Z = # of RHP closed-loop poles (want Z = 0)              ║" << std::endl;
    std::cout << "║  • N = # of CW encirclements of -1 (CCW = negative)         ║" << std::endl;
    std::cout << "║  • P = # of RHP open-loop poles (known)                     ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Special Cases:                                              ║" << std::endl;
    std::cout << "║  • P = 0 (stable OL): Need N = 0 (no encirclements)         ║" << std::endl;
    std::cout << "║  • P > 0 (unstable OL): Need N = -P (P CCW encirclements)   ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Margins from Nyquist:                                       ║" << std::endl;
    std::cout << "║  • GM: Distance from -1 to real-axis crossing               ║" << std::endl;
    std::cout << "║  • PM: Angle from -Re axis to |G|=1 crossing                ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Advantages:                                                 ║" << std::endl;
    std::cout << "║  • Works for systems with time delay                         ║" << std::endl;
    std::cout << "║  • Can use measured frequency response data                  ║" << std::endl;
    std::cout << "║  • Handles non-minimum phase systems                         ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    return 0;
}
