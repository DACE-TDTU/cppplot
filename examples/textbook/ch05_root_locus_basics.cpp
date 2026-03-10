/**
 * @file ch05_root_locus_basics.cpp
 * @brief Chapter 5: Root Locus Fundamentals
 * 
 * Learning Outcomes:
 * - Visualize how closed-loop poles move with gain K
 * - Understand the relationship between pole location and performance
 * - Design controller gain for specified damping ratio
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

// Function to compute closed-loop poles analytically
std::vector<std::complex<double>> computeClosedLoopPoles(
    const std::vector<double>& open_poles, double K) {
    // For G(s) = 1/[(s-p1)(s-p2)], closed-loop is s² + (|p1|+|p2|)s + |p1||p2| + K = 0
    // This is a simplified computation for second-order systems
    
    std::vector<std::complex<double>> cl_poles;
    
    if (open_poles.size() == 2) {
        double a = -(open_poles[0] + open_poles[1]);  // sum of pole magnitudes
        double b = open_poles[0] * open_poles[1] + K;  // product + K
        
        double disc = a*a - 4*b;
        if (disc >= 0) {
            cl_poles.push_back(std::complex<double>((-a + std::sqrt(disc))/2, 0));
            cl_poles.push_back(std::complex<double>((-a - std::sqrt(disc))/2, 0));
        } else {
            cl_poles.push_back(std::complex<double>(-a/2, std::sqrt(-disc)/2));
            cl_poles.push_back(std::complex<double>(-a/2, -std::sqrt(-disc)/2));
        }
    }
    
    return cl_poles;
}

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║     Chapter 5: Root Locus Fundamentals                       ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // ========================================================================
    // System 1: G(s) = 1/[s(s+a)] - Position control
    // ========================================================================
    std::cout << "\n▶ System 1: Position Control" << std::endl;
    std::cout << "  ────────────────────────────" << std::endl;
    
    double a = 4.0;  // Pole at s = -a
    
    std::cout << "  G(s) = 1/[s(s+" << a << ")]" << std::endl;
    std::cout << "  Open-loop poles: s = 0, s = -" << a << std::endl;
    std::cout << "  Open-loop zeros: none" << std::endl;
    
    // Root locus rules
    std::cout << "\n  Root Locus Rules:" << std::endl;
    std::cout << "  • 2 branches (n = 2 poles)" << std::endl;
    std::cout << "  • Start at s = 0 and s = -" << a << " (K = 0)" << std::endl;
    std::cout << "  • End at ∞ (no zeros)" << std::endl;
    std::cout << "  • 2 asymptotes at 90° and 270°" << std::endl;
    std::cout << "  • Centroid: σ = (0 + (-" << a << "))/2 = -" << a/2 << std::endl;
    std::cout << "  • Real axis segment: [" << -a << ", 0]" << std::endl;
    std::cout << "  • Breakaway point: s = -" << a/2 << " (K = " << a*a/4 << ")" << std::endl;
    
    TransferFunction G1({1}, {1, a, 0});
    
    figure(1600, 1200);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 1: Root Locus for System 1
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 1);
    
    std::vector<double> rl_re1, rl_im1, rl_re2, rl_im2;
    std::vector<double> K_values;
    
    for (double K = 0.01; K <= 100; K += 0.2) {
        K_values.push_back(K);
        
        // Characteristic eq: s² + as + K = 0
        double disc = a*a - 4*K;
        if (disc >= 0) {
            rl_re1.push_back((-a + std::sqrt(disc)) / 2);
            rl_im1.push_back(0);
            rl_re2.push_back((-a - std::sqrt(disc)) / 2);
            rl_im2.push_back(0);
        } else {
            rl_re1.push_back(-a / 2);
            rl_im1.push_back(std::sqrt(-disc) / 2);
            rl_re2.push_back(-a / 2);
            rl_im2.push_back(-std::sqrt(-disc) / 2);
        }
    }
    
    plot(rl_re1, rl_im1, "b-", {{"linewidth", "2"}, {"label", "Root Locus"}});
    plot(rl_re2, rl_im2, "b-", {{"linewidth", "2"}});
    
    // Open-loop poles
    scatter({0, -a}, {0, 0}, {{"color", "red"}, {"s", "150"}, {"marker", "x"}, {"label", "OL Poles (×)"}});
    
    // Breakaway point
    double K_breakaway = a*a / 4;
    scatter({-a/2}, {0}, {{"color", "green"}, {"s", "100"}, {"marker", "o"}, {"label", "Breakaway (K=" + std::to_string(int(K_breakaway)) + ")"}});
    
    // Draw asymptotes
    plot({-a/2, -a/2}, {-8.0, 8.0}, "--", {{"color", "gray"}, {"alpha", "0.5"}, {"label", "Asymptotes"}});
    
    // Mark K values on root locus
    std::vector<double> K_markers = {1, 4, 16, 36};
    for (double Km : K_markers) {
        double disc = a*a - 4*Km;
        double re, im;
        if (disc >= 0) {
            re = (-a + std::sqrt(disc)) / 2;
            im = 0;
        } else {
            re = -a / 2;
            im = std::sqrt(-disc) / 2;
        }
        scatter({re}, {im}, {{"color", "orange"}, {"s", "60"}});
    }
    
    axvline(0, {{"color", "red"}, {"linestyle", ":"}, {"alpha", "0.5"}, {"label", "jω axis"}});
    axhline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    
    xlabel("Real (σ)");
    ylabel("Imaginary (jω)");
    title("Root Locus: G(s) = 1/[s(s+4)]");
    legend({{"fontsize", "7"}, {"loc", "upper left"}});
    grid(true);
    xlim(-8, 2);
    ylim(-8, 8);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 2: Step Response for Different K
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 2);
    
    std::vector<double> K_demo = {1, 4, 9, 16, 36};
    std::vector<std::string> colors = {"#E74C3C", "#E67E22", "#F1C40F", "#2ECC71", "#3498DB"};
    
    double t_final = 6.0;
    for (size_t i = 0; i < K_demo.size(); ++i) {
        double K = K_demo[i];
        TransferFunction G_cl = feedback(K * G1);
        auto [t, y] = step(G_cl, t_final);
        
        std::ostringstream label;
        label << "K = " << int(K);
        plot(t, y, "-", {{"color", colors[i]}, {"linewidth", "2"}, {"label", label.str()}});
    }
    
    axhline(1.0, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.3"}});
    xlabel("Time [s]");
    ylabel("Response");
    title("Step Response vs Gain K");
    legend({{"fontsize", "8"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 3: Damping Ratio vs K
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 3);
    
    std::vector<double> K_range, zeta_range, wn_range;
    for (double K = 0.1; K <= 50; K += 0.2) {
        K_range.push_back(K);
        
        double wn = std::sqrt(K);
        double zeta = a / (2 * wn);
        
        zeta_range.push_back(std::min(zeta, 3.0));  // Clip for display
        wn_range.push_back(wn);
    }
    
    plot(K_range, zeta_range, "b-", {{"linewidth", "2"}, {"label", "ζ = a/(2√K)"}});
    
    axhline(1.0, {{"color", "red"}, {"linestyle", "--"}, {"label", "Critical (ζ=1)"}});
    axhline(0.707, {{"color", "green"}, {"linestyle", "--"}, {"label", "Optimal (ζ=0.707)"}});
    axhline(0.5, {{"color", "orange"}, {"linestyle", ":"}, {"label", "ζ=0.5"}});
    
    // Mark K values for specific ζ
    double K_critical = (a / 2) * (a / 2);  // ζ = 1
    double K_optimal = (a / (2 * 0.707)) * (a / (2 * 0.707));  // ζ = 0.707
    
    axvline(K_critical, {{"color", "red"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    axvline(K_optimal, {{"color", "green"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    
    xlabel("Gain K");
    ylabel("Damping Ratio ζ");
    title("Damping Ratio vs Controller Gain");
    legend({{"fontsize", "7"}});
    grid(true);
    xlim(0, 50);
    ylim(0, 3);
    
    // ========================================================================
    // System 2: G(s) = 1/[s(s+2)(s+4)] - Third-order system
    // ========================================================================
    std::cout << "\n▶ System 2: Third-Order System (Unstable for High K)" << std::endl;
    std::cout << "  ──────────────────────────────────────────────────────" << std::endl;
    
    std::cout << "  G(s) = 1/[s(s+2)(s+4)]" << std::endl;
    std::cout << "  Open-loop poles: s = 0, -2, -4" << std::endl;
    std::cout << "\n  Root Locus Rules:" << std::endl;
    std::cout << "  • 3 branches, 3 asymptotes" << std::endl;
    std::cout << "  • Asymptote angles: 60°, 180°, 300°" << std::endl;
    std::cout << "  • Centroid: (0 - 2 - 4)/3 = -2" << std::endl;
    std::cout << "  • Root locus CROSSES jω axis → System can be UNSTABLE!" << std::endl;
    
    TransferFunction G2({1}, {1, 6, 8, 0});  // s³ + 6s² + 8s
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 4: Root Locus for Third-Order System
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 4);
    
    // Simplified root locus computation (numerical)
    // For this system, crossing happens at K = 48 (Routh-Hurwitz)
    // Characteristic: s³ + 6s² + 8s + K = 0
    
    // We'll compute a few points numerically
    std::vector<double> rl3_re, rl3_im;
    
    // Real axis portion (K small)
    for (double s_val = -4; s_val <= 0; s_val += 0.05) {
        // K = -s(s+2)(s+4) for real axis points
        double K = -s_val * (s_val + 2) * (s_val + 4);
        if (K >= 0) {
            rl3_re.push_back(s_val);
            rl3_im.push_back(0);
        }
    }
    
    // Complex branches (approximate using dominant pole behavior)
    for (double K = 1; K <= 100; K += 1) {
        // Approximate: for K > 48, one real pole, two complex
        // For K < 48, use numerical methods or approximation
        
        // At K = 48: s = ±j√8 (crossing point)
        // For simplicity, show the trend
        if (K < 48) {
            // Underdamped region
            double sigma_approx = -2 - 0.02 * K;
            double omega_approx = std::sqrt(std::max(0.0, (K - 4.0) / 3.0));
            if (omega_approx > 0.1) {
                rl3_re.push_back(sigma_approx);
                rl3_im.push_back(omega_approx);
                rl3_re.push_back(sigma_approx);
                rl3_im.push_back(-omega_approx);
            }
        }
    }
    
    scatter(rl3_re, rl3_im, {{"color", "blue"}, {"s", "10"}, {"alpha", "0.5"}});
    
    // Open-loop poles
    scatter({0, -2, -4}, {0, 0, 0}, {{"color", "red"}, {"s", "150"}, {"marker", "x"}, {"label", "OL Poles"}});
    
    // Centroid and asymptotes
    scatter({-2}, {0}, {{"color", "green"}, {"s", "100"}, {"marker", "s"}, {"label", "Centroid"}});
    
    // Draw asymptotes
    for (double angle : {60.0, 180.0, 300.0}) {
        double rad = angle * M_PI / 180;
        std::vector<double> asym_re, asym_im;
        for (double r = 0; r <= 6; r += 0.5) {
            asym_re.push_back(-2 + r * std::cos(rad));
            asym_im.push_back(r * std::sin(rad));
        }
        plot(asym_re, asym_im, "--", {{"color", "gray"}, {"alpha", "0.4"}});
    }
    
    // jω axis crossing
    double w_cross = std::sqrt(8);  // At K = 48
    scatter({0, 0}, {w_cross, -w_cross}, {{"color", "orange"}, {"s", "100"}, {"marker", "^"}, {"label", "K=48 crossing"}});
    
    axvline(0, {{"color", "red"}, {"linestyle", "-"}, {"linewidth", "2"}, {"alpha", "0.7"}});
    axhline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    
    xlabel("Real");
    ylabel("Imaginary");
    title("Root Locus: G(s) = 1/[s(s+2)(s+4)]");
    legend({{"fontsize", "7"}});
    grid(true);
    xlim(-6, 2);
    ylim(-5, 5);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 5: Step Response Showing Instability
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 5);
    
    std::vector<double> K_demo2 = {10, 30, 48, 60};
    std::vector<std::string> colors2 = {"#2ECC71", "#F1C40F", "#E67E22", "#E74C3C"};
    
    for (size_t i = 0; i < K_demo2.size(); ++i) {
        double K = K_demo2[i];
        TransferFunction G_cl2 = feedback(K * G2);
        auto [t, y] = step(G_cl2, 8.0);
        
        // Clip unstable response for visualization
        std::vector<double> y_clip;
        for (double val : y) {
            y_clip.push_back(std::max(-3.0, std::min(3.0, val)));
        }
        
        std::ostringstream label;
        label << "K = " << int(K);
        if (K >= 48) label << " (unstable)";
        plot(t, y_clip, "-", {{"color", colors2[i]}, {"linewidth", "2"}, {"label", label.str()}});
    }
    
    axhline(1.0, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.3"}});
    axhline(0, {{"color", "gray"}, {"linestyle", ":"}, {"alpha", "0.3"}});
    
    xlabel("Time [s]");
    ylabel("Response (clipped)");
    title("Third-Order System: Stability Limit");
    legend({{"fontsize", "8"}});
    grid(true);
    ylim(-2, 3);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 6: Design Chart - S-Plane Regions
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 3, 6);
    
    // Draw constant ζ lines
    for (double zeta : {0.3, 0.5, 0.707, 0.9}) {
        double angle = std::acos(zeta);
        std::vector<double> z_re, z_im_p, z_im_n;
        for (double r = 0; r <= 10; r += 0.2) {
            z_re.push_back(-r * std::cos(angle));
            z_im_p.push_back(r * std::sin(angle));
            z_im_n.push_back(-r * std::sin(angle));
        }
        plot(z_re, z_im_p, ":", {{"color", "blue"}, {"alpha", "0.4"}});
        plot(z_re, z_im_n, ":", {{"color", "blue"}, {"alpha", "0.4"}});
    }
    
    // Draw constant ωn circles
    for (double wn : {2.0, 4.0, 6.0, 8.0}) {
        std::vector<double> wn_re, wn_im;
        for (int j = 0; j <= 50; ++j) {
            double ang = M_PI/2 + j * M_PI / 50;
            wn_re.push_back(wn * std::cos(ang));
            wn_im.push_back(wn * std::sin(ang));
        }
        plot(wn_re, wn_im, "--", {{"color", "green"}, {"alpha", "0.3"}});
    }
    
    // Draw constant settling time lines (σ = 4/ts)
    for (double ts : {0.5, 1.0, 2.0}) {
        double sigma = 4.0 / ts;
        axvline(-sigma, {{"color", "red"}, {"linestyle", ":"}, {"alpha", "0.4"}});
    }
    
    // Annotations
    text(-1, 7, "ζ lines", {{"fontsize", "8"}, {"color", "blue"}});
    text(-7, 3, "ωn circles", {{"fontsize", "8"}, {"color", "green"}});
    text(-9, 7, "ts lines", {{"fontsize", "8"}, {"color", "red"}});
    
    axvline(0, {{"color", "red"}, {"linewidth", "2"}, {"alpha", "0.7"}});
    axhline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    
    // Shade stable region
    fill_between({-10, 0, 0, -10}, {-10, -10, 10, 10}, {{"color", "green"}, {"alpha", "0.05"}});
    
    xlabel("Real (σ)");
    ylabel("Imaginary (jω)");
    title("S-Plane Design Regions");
    grid(true);
    xlim(-10, 2);
    ylim(-8, 8);
    
    savefig("ch05_root_locus_basics.svg");
    std::cout << "\n✓ Saved ch05_root_locus_basics.svg" << std::endl;
    
    // ========================================================================
    // Design Summary
    // ========================================================================
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                ROOT LOCUS DESIGN SUMMARY                     ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Key Rules:                                                  ║" << std::endl;
    std::cout << "║  1. Branches start at OL poles (K=0)                         ║" << std::endl;
    std::cout << "║  2. Branches end at OL zeros or ∞ (K→∞)                     ║" << std::endl;
    std::cout << "║  3. Real axis: odd count of poles+zeros to the right        ║" << std::endl;
    std::cout << "║  4. Asymptotes: angles = (2k+1)·180°/(n-m)                   ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Design Process:                                             ║" << std::endl;
    std::cout << "║  1. Sketch root locus                                        ║" << std::endl;
    std::cout << "║  2. Identify desired pole region (ζ, ωn specs)              ║" << std::endl;
    std::cout << "║  3. Find K where root locus passes through region           ║" << std::endl;
    std::cout << "║  4. If no K works, add compensator to reshape locus         ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  For G(s) = 1/[s(s+a)]:                                      ║" << std::endl;
    std::cout << "║  • ζ = a/(2√K)  →  K = (a/(2ζ))²                           ║" << std::endl;
    std::cout << "║  • ωn = √K                                                  ║" << std::endl;
    std::cout << "║  • Always stable (locus stays in LHP)                       ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    return 0;
}
