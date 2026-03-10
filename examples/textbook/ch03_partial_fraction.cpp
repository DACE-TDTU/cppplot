/**
 * @file ch03_partial_fraction.cpp
 * @brief Chapter 3: Partial Fraction Expansion and Inverse Laplace Transform
 * 
 * Learning Outcomes:
 * - Decompose transfer functions using partial fractions
 * - Compute inverse Laplace transform
 * - Understand the contribution of each mode to system response
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
    std::cout << "║   Chapter 3: Partial Fraction Expansion                      ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // ========================================================================
    // Example 1: Distinct Real Poles
    // ========================================================================
    std::cout << "\n▶ Example 1: Distinct Real Poles" << std::endl;
    std::cout << "  ─────────────────────────────────" << std::endl;
    
    std::cout << R"(
    Given: G(s) = 2(s+3) / [(s+1)(s+2)]
    
    Step Response: Y(s) = G(s)/s = 2(s+3) / [s(s+1)(s+2)]
    
    Partial Fractions:
    Y(s) = A/s + B/(s+1) + C/(s+2)
    
    Finding residues:
    A = s·Y(s)|_{s=0} = 2(0+3)/[(0+1)(0+2)] = 6/2 = 3
    B = (s+1)·Y(s)|_{s=-1} = 2(-1+3)/[(-1)(-1+2)] = 4/(-1) = -4
    C = (s+2)·Y(s)|_{s=-2} = 2(-2+3)/[(-2)(-2+1)] = 2/2 = 1
    
    Therefore: Y(s) = 3/s - 4/(s+1) + 1/(s+2)
    
    Inverse Laplace: y(t) = 3 - 4e^(-t) + e^(-2t)
)" << std::endl;

    TransferFunction G1({2, 6}, {1, 3, 2});  // 2(s+3)/(s^2+3s+2)
    
    std::cout << "  Poles: ";
    for (auto& p : G1.poles()) {
        std::cout << p << " ";
    }
    std::cout << std::endl;
    
    double t_final = 6.0;
    auto [t1, y1_sim] = step(G1, t_final);
    
    // Analytical components
    std::vector<double> y1_dc, y1_mode1, y1_mode2, y1_analytical;
    for (double ti : t1) {
        double dc = 3.0;
        double mode1 = -4.0 * std::exp(-ti);
        double mode2 = 1.0 * std::exp(-2*ti);
        y1_dc.push_back(dc);
        y1_mode1.push_back(mode1);
        y1_mode2.push_back(mode2);
        y1_analytical.push_back(dc + mode1 + mode2);
    }
    
    figure(1400, 900);
    
    // Plot 1: Simulation vs Analytical
    subplot(2, 3, 1);
    plot(t1, y1_sim, "b-", {{"linewidth", "3"}, {"label", "Simulation"}});
    plot(t1, y1_analytical, "r--", {{"linewidth", "2"}, {"label", "Analytical"}});
    xlabel("Time [s]");
    ylabel("y(t)");
    title("Example 1: Step Response Verification");
    legend();
    grid(true);
    
    // Plot 2: Component breakdown
    subplot(2, 3, 2);
    plot(t1, y1_dc, "--", {{"color", "green"}, {"linewidth", "1.5"}, {"label", "DC: 3"}});
    plot(t1, y1_mode1, "--", {{"color", "red"}, {"linewidth", "1.5"}, {"label", "Mode 1: -4e^(-t)"}});
    plot(t1, y1_mode2, "--", {{"color", "blue"}, {"linewidth", "1.5"}, {"label", "Mode 2: e^(-2t)"}});
    plot(t1, y1_analytical, "k-", {{"linewidth", "2"}, {"label", "Sum"}});
    axhline(0, {{"color", "gray"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    xlabel("Time [s]");
    ylabel("Component");
    title("Partial Fraction Components");
    legend({{"fontsize", "8"}});
    grid(true);
    
    // ========================================================================
    // Example 2: Complex Conjugate Poles
    // ========================================================================
    std::cout << "\n▶ Example 2: Complex Conjugate Poles" << std::endl;
    std::cout << "  ─────────────────────────────────────" << std::endl;
    
    std::cout << R"(
    Given: G(s) = 4 / (s² + 2s + 5)
    
    Poles: s = -1 ± j2  (ωn=√5, ζ=1/√5≈0.447)
    
    Step Response: Y(s) = 4 / [s(s² + 2s + 5)]
    
    Partial Fractions:
    Y(s) = A/s + (Bs + C)/(s² + 2s + 5)
    
    A = 4/5 = 0.8
    
    Completing the square: s² + 2s + 5 = (s+1)² + 4
    
    y(t) = 0.8[1 - e^(-t)(cos(2t) + 0.5·sin(2t))]
         = 0.8[1 - 1.118·e^(-t)·cos(2t - 0.464)]
)" << std::endl;

    TransferFunction G2({4}, {1, 2, 5});
    
    std::cout << "  Poles: ";
    for (auto& p : G2.poles()) {
        std::cout << p << " ";
    }
    std::cout << std::endl;
    
    auto [t2, y2_sim] = step(G2, t_final);
    
    // Analytical solution
    double dc_gain2 = 4.0 / 5.0;  // G(0) = 4/5 = 0.8
    std::vector<double> y2_analytical, y2_dc, y2_transient;
    for (double ti : t2) {
        double dc = dc_gain2;
        double transient = -dc_gain2 * std::exp(-ti) * (std::cos(2*ti) + 0.5*std::sin(2*ti));
        y2_dc.push_back(dc);
        y2_transient.push_back(transient);
        y2_analytical.push_back(dc + transient);
    }
    
    subplot(2, 3, 3);
    plot(t2, y2_sim, "b-", {{"linewidth", "3"}, {"label", "Simulation"}});
    plot(t2, y2_analytical, "r--", {{"linewidth", "2"}, {"label", "Analytical"}});
    xlabel("Time [s]");
    ylabel("y(t)");
    title("Example 2: Complex Poles");
    legend();
    grid(true);
    
    subplot(2, 3, 4);
    plot(t2, y2_dc, "--", {{"color", "green"}, {"linewidth", "1.5"}, {"label", "DC: 0.8"}});
    plot(t2, y2_transient, "--", {{"color", "red"}, {"linewidth", "1.5"}, {"label", "Transient"}});
    plot(t2, y2_analytical, "k-", {{"linewidth", "2"}, {"label", "Sum"}});
    
    // Draw envelope
    std::vector<double> env_upper, env_lower;
    for (double ti : t2) {
        double env = dc_gain2 * 1.118 * std::exp(-ti);
        env_upper.push_back(dc_gain2 + env);
        env_lower.push_back(dc_gain2 - env);
    }
    plot(t2, env_upper, ":", {{"color", "orange"}, {"alpha", "0.7"}});
    plot(t2, env_lower, ":", {{"color", "orange"}, {"alpha", "0.7"}, {"label", "Envelope"}});
    
    axhline(dc_gain2, {{"color", "gray"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    xlabel("Time [s]");
    ylabel("Component");
    title("Complex Pole Decomposition");
    legend({{"fontsize", "8"}});
    grid(true);
    
    // ========================================================================
    // Example 3: Repeated Poles
    // ========================================================================
    std::cout << "\n▶ Example 3: Repeated Poles" << std::endl;
    std::cout << "  ───────────────────────────" << std::endl;
    
    std::cout << R"(
    Given: G(s) = 1 / (s+1)²
    
    Step Response: Y(s) = 1 / [s(s+1)²]
    
    Partial Fractions:
    Y(s) = A/s + B/(s+1)² + C/(s+1)
    
    A = [s·Y(s)]|_{s=0} = 1/1 = 1
    B = [(s+1)²·Y(s)]|_{s=-1} = 1/(-1) = -1
    C = d/ds[(s+1)²·Y(s)]|_{s=-1} = d/ds[1/s]|_{s=-1} = -1/s²|_{s=-1} = -1
    
    Y(s) = 1/s - 1/(s+1)² - 1/(s+1)
    
    y(t) = 1 - t·e^(-t) - e^(-t) = 1 - (1+t)e^(-t)
)" << std::endl;

    TransferFunction G3({1}, {1, 2, 1});  // 1/(s+1)² = 1/(s²+2s+1)
    
    std::cout << "  Poles: ";
    for (auto& p : G3.poles()) {
        std::cout << p << " (repeated) ";
    }
    std::cout << std::endl;
    
    auto [t3, y3_sim] = step(G3, t_final);
    
    // Analytical components
    std::vector<double> y3_dc, y3_mode_te, y3_mode_e, y3_analytical;
    for (double ti : t3) {
        double dc = 1.0;
        double mode_te = -ti * std::exp(-ti);
        double mode_e = -std::exp(-ti);
        y3_dc.push_back(dc);
        y3_mode_te.push_back(mode_te);
        y3_mode_e.push_back(mode_e);
        y3_analytical.push_back(dc + mode_te + mode_e);
    }
    
    subplot(2, 3, 5);
    plot(t3, y3_sim, "b-", {{"linewidth", "3"}, {"label", "Simulation"}});
    plot(t3, y3_analytical, "r--", {{"linewidth", "2"}, {"label", "Analytical"}});
    xlabel("Time [s]");
    ylabel("y(t)");
    title("Example 3: Repeated Pole");
    legend();
    grid(true);
    
    subplot(2, 3, 6);
    plot(t3, y3_dc, "--", {{"color", "green"}, {"linewidth", "1.5"}, {"label", "DC: 1"}});
    plot(t3, y3_mode_te, "--", {{"color", "red"}, {"linewidth", "1.5"}, {"label", "-t·e^(-t)"}});
    plot(t3, y3_mode_e, "--", {{"color", "blue"}, {"linewidth", "1.5"}, {"label", "-e^(-t)"}});
    plot(t3, y3_analytical, "k-", {{"linewidth", "2"}, {"label", "Sum"}});
    axhline(0, {{"color", "gray"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    xlabel("Time [s]");
    ylabel("Component");
    title("Repeated Pole Components");
    legend({{"fontsize", "8"}});
    grid(true);
    
    savefig("ch03_partial_fraction.svg");
    std::cout << "\n✓ Saved ch03_partial_fraction.svg" << std::endl;
    
    // ========================================================================
    // Summary Table
    // ========================================================================
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║            INVERSE LAPLACE TRANSFORM TABLE                   ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  F(s)                      f(t)                              ║" << std::endl;
    std::cout << "║  ─────────────────────────────────────────────────────────   ║" << std::endl;
    std::cout << "║  A/s                       A                                 ║" << std::endl;
    std::cout << "║  A/(s+a)                   A·e^(-at)                         ║" << std::endl;
    std::cout << "║  A/(s+a)²                  A·t·e^(-at)                       ║" << std::endl;
    std::cout << "║  Aω/[(s+a)²+ω²]           A·e^(-at)·sin(ωt)                 ║" << std::endl;
    std::cout << "║  A(s+a)/[(s+a)²+ω²]       A·e^(-at)·cos(ωt)                 ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Residue Formulas:                                           ║" << std::endl;
    std::cout << "║  ─────────────────────────────────────────────────────────   ║" << std::endl;
    std::cout << "║  Simple pole p:    A = (s-p)·F(s)|_{s=p}                    ║" << std::endl;
    std::cout << "║  Repeated pole:    Need derivative method                   ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    return 0;
}
