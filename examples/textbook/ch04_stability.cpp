/**
 * @file ch04_stability.cpp
 * @brief Chapter 4: Stability analysis
 * 
 * This example demonstrates:
 * - BIBO stability concept
 * - Pole location analysis
 * - Routh-Hurwitz criterion
 * - Marginal stability and oscillations
 * 
 * Compile: g++ -std=c++14 -I "../../include" ch04_stability.cpp -o ch04_stability.exe
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

// Compute Routh table
std::vector<std::vector<double>> routhTable(const std::vector<double>& coeffs) {
    int n = coeffs.size();
    int rows = n;
    int cols = (n + 1) / 2;
    
    std::vector<std::vector<double>> table(rows, std::vector<double>(cols, 0.0));
    
    // Fill first two rows
    for (int i = 0; i < n; i += 2) {
        table[0][i / 2] = coeffs[i];
    }
    for (int i = 1; i < n; i += 2) {
        table[1][i / 2] = coeffs[i];
    }
    
    // Fill remaining rows
    for (int i = 2; i < rows; ++i) {
        for (int j = 0; j < cols - 1; ++j) {
            double a = table[i - 1][0];
            if (std::abs(a) < 1e-10) {
                // Handle special case: zero in first column
                a = 1e-10;
            }
            table[i][j] = (table[i - 1][0] * table[i - 2][j + 1] - 
                          table[i - 2][0] * table[i - 1][j + 1]) / a;
        }
    }
    
    return table;
}

// Print Routh table
void printRouthTable(const std::vector<std::vector<double>>& table, 
                     const std::vector<std::string>& row_labels) {
    int rows = table.size();
    int cols = table[0].size();
    
    std::cout << "   ┌─────────";
    for (int j = 0; j < cols; ++j) std::cout << "┬───────────";
    std::cout << "┐" << std::endl;
    
    std::cout << "   │ Row     ";
    for (int j = 0; j < cols; ++j) {
        std::cout << "│    c" << j << "     ";
    }
    std::cout << "│" << std::endl;
    
    std::cout << "   ├─────────";
    for (int j = 0; j < cols; ++j) std::cout << "┼───────────";
    std::cout << "┤" << std::endl;
    
    for (int i = 0; i < rows; ++i) {
        std::cout << "   │ " << std::setw(7) << row_labels[i] << " ";
        for (int j = 0; j < cols; ++j) {
            std::cout << "│ " << std::setw(9) << std::fixed << std::setprecision(3) 
                      << table[i][j] << " ";
        }
        std::cout << "│" << std::endl;
    }
    
    std::cout << "   └─────────";
    for (int j = 0; j < cols; ++j) std::cout << "┴───────────";
    std::cout << "┘" << std::endl;
}

// Count sign changes in first column
int countSignChanges(const std::vector<std::vector<double>>& table) {
    int changes = 0;
    double prev_sign = (table[0][0] >= 0) ? 1.0 : -1.0;
    
    for (size_t i = 1; i < table.size(); ++i) {
        double curr_sign = (table[i][0] >= 0) ? 1.0 : -1.0;
        if (curr_sign != prev_sign) {
            changes++;
        }
        prev_sign = curr_sign;
    }
    
    return changes;
}

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║          Chapter 4: Stability Analysis                       ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // ========================================================================
    // Part 1: Stability Based on Pole Location
    // ========================================================================
    std::cout << "\n▶ PART 1: Stability Based on Pole Location" << std::endl;
    std::cout << "   ──────────────────────────────────────────" << std::endl;
    
    std::cout << "\n   BIBO Stability Theorem:" << std::endl;
    std::cout << "   A LTI system is stable if and only if ALL poles have" << std::endl;
    std::cout << "   negative real parts (lie in the left half-plane)." << std::endl;
    
    // Create systems with different pole locations
    struct SystemExample {
        std::string name;
        std::vector<double> num;
        std::vector<double> den;
        std::string stability;
    };
    
    std::vector<SystemExample> systems = {
        {"Stable (real poles)", {1}, {1, 3, 2}, "Stable"},           // (s+1)(s+2)
        {"Stable (complex)", {4}, {1, 2, 4}, "Stable"},              // s² + 2s + 4
        {"Marginally stable", {1}, {1, 0, 1}, "Marginally"},         // s² + 1 (poles at ±j)
        {"Unstable (one RHP)", {1}, {1, 0, -1}, "Unstable"},         // s² - 1 = (s-1)(s+1)
        {"Unstable (complex RHP)", {4}, {1, -2, 4}, "Unstable"}      // s² - 2s + 4
    };
    
    std::cout << "\n   ┌────────────────────────┬────────────────────────────┬─────────────┐" << std::endl;
    std::cout << "   │ System                 │ Poles                      │ Stability   │" << std::endl;
    std::cout << "   ├────────────────────────┼────────────────────────────┼─────────────┤" << std::endl;
    
    for (auto& sys : systems) {
        TransferFunction G(sys.num, sys.den);
        auto poles = G.poles();
        
        std::ostringstream poles_str;
        for (size_t i = 0; i < poles.size(); ++i) {
            if (i > 0) poles_str << ", ";
            poles_str << std::fixed << std::setprecision(2);
            if (std::abs(poles[i].imag()) < 1e-6) {
                poles_str << poles[i].real();
            } else {
                poles_str << poles[i].real() << (poles[i].imag() >= 0 ? "+" : "") 
                          << poles[i].imag() << "j";
            }
        }
        
        std::cout << "   │ " << std::setw(22) << sys.name << " │ " 
                  << std::setw(26) << poles_str.str() << " │ "
                  << std::setw(11) << sys.stability << " │" << std::endl;
    }
    std::cout << "   └────────────────────────┴────────────────────────────┴─────────────┘" << std::endl;
    
    // Plot pole locations
    figure(800, 600);
    
    // Draw stability region boundary (imaginary axis)
    std::vector<double> im_axis_x = {0, 0};
    std::vector<double> im_axis_y = {-5, 5};
    plot(im_axis_x, im_axis_y, "k-", {{"linewidth", "2"}});
    
    // Shade stable region
    fill({-5, 0, 0, -5}, {-5, -5, 5, 5}, {{"color", "lightgreen"}, {"alpha", "0.2"}});
    text(-2.5, 4, "STABLE", {{"fontsize", "14"}, {"color", "green"}});
    
    // Shade unstable region  
    fill({0, 5, 5, 0}, {-5, -5, 5, 5}, {{"color", "lightsalmon"}, {"alpha", "0.2"}});
    text(2, 4, "UNSTABLE", {{"fontsize", "14"}, {"color", "red"}});
    
    // Plot poles for each system
    std::vector<std::string> markers = {"o", "s", "^", "d", "v"};
    std::vector<std::string> colors = {"blue", "green", "orange", "red", "purple"};
    
    for (size_t i = 0; i < systems.size(); ++i) {
        TransferFunction G(systems[i].num, systems[i].den);
        auto poles = G.poles();
        
        std::vector<double> re, im;
        for (auto& p : poles) {
            re.push_back(p.real());
            im.push_back(p.imag());
        }
        
        scatter(re, im, {{"color", colors[i]}, {"markersize", "12"}, {"label", systems[i].name}});
    }
    
    axhline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    axvline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    
    xlabel("Real Part [σ]");
    ylabel("Imaginary Part [jω]");
    title("Pole Locations and Stability Regions");
    legend();
    grid(true);
    xlim(-4, 4);
    ylim(-4, 4);
    
    savefig("ch04_stability_regions.svg");
    std::cout << "\n   ✓ Plot saved: ch04_stability_regions.svg" << std::endl;
    
    // ========================================================================
    // Part 2: Step Response Comparison
    // ========================================================================
    std::cout << "\n▶ PART 2: Step Response Comparison" << std::endl;
    std::cout << "   ──────────────────────────────────" << std::endl;
    
    figure(1000, 700);
    
    // Only plot stable and marginally stable systems
    // (Unstable systems diverge)
    
    subplot(2, 2, 1);
    {
        TransferFunction G({1}, {1, 3, 2});
        auto [t, y] = step(G, 10.0);
        plot(t, y, "b-", {{"linewidth", "2"}});
        xlabel("Time [s]");
        ylabel("Output");
        title("Stable: Poles at -1, -2");
        grid(true);
    }
    
    subplot(2, 2, 2);
    {
        TransferFunction G({4}, {1, 2, 4});
        auto [t, y] = step(G, 10.0);
        plot(t, y, "g-", {{"linewidth", "2"}});
        xlabel("Time [s]");
        ylabel("Output");
        title("Stable: Poles at -1±j√3");
        grid(true);
    }
    
    subplot(2, 2, 3);
    {
        TransferFunction G({1}, {1, 0, 1});
        auto [t, y] = step(G, 30.0);
        plot(t, y, "orange", {{"linewidth", "2"}});
        xlabel("Time [s]");
        ylabel("Output");
        title("Marginally Stable: Poles at ±j");
        grid(true);
    }
    
    subplot(2, 2, 4);
    {
        // Simulate unstable for short time
        TransferFunction G({1}, {1, 0, -1});
        auto [t, y] = step(G, 3.0, 100);
        plot(t, y, "r-", {{"linewidth", "2"}});
        xlabel("Time [s]");
        ylabel("Output");
        title("Unstable: Poles at ±1");
        grid(true);
    }
    
    savefig("ch04_stability_responses.svg");
    std::cout << "\n   ✓ Plot saved: ch04_stability_responses.svg" << std::endl;
    
    // ========================================================================
    // Part 3: Routh-Hurwitz Criterion
    // ========================================================================
    std::cout << "\n▶ PART 3: Routh-Hurwitz Criterion" << std::endl;
    std::cout << "   ─────────────────────────────────" << std::endl;
    
    std::cout << "\n   The Routh-Hurwitz criterion determines stability" << std::endl;
    std::cout << "   without computing poles directly." << std::endl;
    
    // Example 1: Stable system
    std::cout << "\n   Example 1: s³ + 6s² + 11s + 6 = 0" << std::endl;
    std::cout << "              (Known roots: s = -1, -2, -3)" << std::endl;
    
    std::vector<double> coeffs1 = {1, 6, 11, 6};
    auto table1 = routhTable(coeffs1);
    std::vector<std::string> labels1 = {"s³", "s²", "s¹", "s⁰"};
    
    std::cout << "\n   Routh Table:" << std::endl;
    printRouthTable(table1, labels1);
    
    int changes1 = countSignChanges(table1);
    std::cout << "\n   Sign changes in first column: " << changes1 << std::endl;
    std::cout << "   → System is " << (changes1 == 0 ? "STABLE" : "UNSTABLE") << std::endl;
    
    // Example 2: Unstable system
    std::cout << "\n   ─────────────────────────────────────────────" << std::endl;
    std::cout << "\n   Example 2: s³ + 2s² - s - 2 = 0" << std::endl;
    std::cout << "              (Known roots: s = -2, -1, +1)" << std::endl;
    
    std::vector<double> coeffs2 = {1, 2, -1, -2};
    auto table2 = routhTable(coeffs2);
    
    std::cout << "\n   Routh Table:" << std::endl;
    printRouthTable(table2, labels1);
    
    int changes2 = countSignChanges(table2);
    std::cout << "\n   Sign changes in first column: " << changes2 << std::endl;
    std::cout << "   → System has " << changes2 << " pole(s) in RHP → UNSTABLE" << std::endl;
    
    // Example 3: Design with parameter K
    std::cout << "\n   ─────────────────────────────────────────────" << std::endl;
    std::cout << "\n   Example 3: Find K for stability" << std::endl;
    std::cout << "              Closed-loop: s³ + 3s² + 2s + K = 0" << std::endl;
    
    std::cout << "\n   Routh Table (symbolic):" << std::endl;
    std::cout << "   ┌─────────┬───────────┬───────────┐" << std::endl;
    std::cout << "   │ Row     │    c0     │    c1     │" << std::endl;
    std::cout << "   ├─────────┼───────────┼───────────┤" << std::endl;
    std::cout << "   │    s³   │     1     │     2     │" << std::endl;
    std::cout << "   │    s²   │     3     │     K     │" << std::endl;
    std::cout << "   │    s¹   │ (6-K)/3   │     0     │" << std::endl;
    std::cout << "   │    s⁰   │     K     │     0     │" << std::endl;
    std::cout << "   └─────────┴───────────┴───────────┘" << std::endl;
    
    std::cout << "\n   For stability, all elements in first column must be > 0:" << std::endl;
    std::cout << "   • 1 > 0       ✓ (always)" << std::endl;
    std::cout << "   • 3 > 0       ✓ (always)" << std::endl;
    std::cout << "   • (6-K)/3 > 0 → K < 6" << std::endl;
    std::cout << "   • K > 0" << std::endl;
    std::cout << "\n   → Stability range: 0 < K < 6" << std::endl;
    
    // Verify with simulation
    std::cout << "\n   Verification by simulation:" << std::endl;
    
    figure(1200, 400);
    
    std::vector<double> K_values = {1, 3, 5, 6, 8};
    std::vector<std::string> k_colors = {"blue", "green", "orange", "red", "darkred"};
    
    for (size_t i = 0; i < K_values.size(); ++i) {
        double K = K_values[i];
        TransferFunction G({K}, {1, 3, 2, K});
        auto [t, y] = step(G, 20.0);
        
        std::string label = "K = " + std::to_string((int)K);
        plot(t, y, "-", {{"color", k_colors[i]}, {"linewidth", "2"}, {"label", label}});
        
        // Check stability
        auto poles = G.poles();
        bool stable = true;
        for (auto& p : poles) {
            if (p.real() >= 0) stable = false;
        }
        std::cout << "   K = " << K << ": " << (stable ? "Stable" : "Unstable") << std::endl;
    }
    
    axhline(1.0, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.3"}});
    xlabel("Time [s]");
    ylabel("Output");
    title("Closed-Loop Step Response: G(s) = K/(s³ + 3s² + 2s + K)");
    legend();
    grid(true);
    ylim(-2, 5);
    
    savefig("ch04_routh_verification.svg");
    std::cout << "\n   ✓ Plot saved: ch04_routh_verification.svg" << std::endl;
    
    // ========================================================================
    // Part 4: Root Locus Preview
    // ========================================================================
    std::cout << "\n▶ PART 4: Stability Boundary (Root Locus Preview)" << std::endl;
    std::cout << "   ──────────────────────────────────────────────────" << std::endl;
    
    figure(800, 600);
    
    // Track pole locations as K varies
    std::vector<double> K_range;
    for (double K = 0.1; K <= 10; K += 0.1) {
        K_range.push_back(K);
    }
    
    // Store all pole locations
    std::vector<std::vector<double>> pole_real(3), pole_imag(3);
    
    for (double K : K_range) {
        TransferFunction G({K}, {1, 3, 2, K});
        auto poles = G.poles();
        
        for (size_t i = 0; i < 3; ++i) {
            pole_real[i].push_back(poles[i].real());
            pole_imag[i].push_back(poles[i].imag());
        }
    }
    
    // Plot root locus
    for (size_t i = 0; i < 3; ++i) {
        plot(pole_real[i], pole_imag[i], "-", {{"linewidth", "1.5"}});
    }
    
    // Mark critical point (K = 6, poles on imaginary axis)
    TransferFunction G_crit({6}, {1, 3, 2, 6});
    auto poles_crit = G_crit.poles();
    std::vector<double> crit_re, crit_im;
    for (auto& p : poles_crit) {
        crit_re.push_back(p.real());
        crit_im.push_back(p.imag());
    }
    scatter(crit_re, crit_im, {{"color", "red"}, {"markersize", "15"}, {"label", "K = 6 (critical)"}});
    
    axhline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    axvline(0, {{"color", "black"}, {"linewidth", "2"}, {"label", "Stability boundary"}});
    
    xlabel("Real Part");
    ylabel("Imaginary Part");
    title("Pole Migration: G(s) = K/(s³ + 3s² + 2s + K), K: 0.1 → 10");
    legend();
    grid(true);
    xlim(-4, 2);
    ylim(-3, 3);
    
    savefig("ch04_pole_migration.svg");
    std::cout << "\n   ✓ Plot saved: ch04_pole_migration.svg" << std::endl;
    
    // ========================================================================
    // Summary
    // ========================================================================
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                         SUMMARY                              ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Stability Conditions:                                        ║" << std::endl;
    std::cout << "║ • Stable:     All poles in LHP (Re(s) < 0)                   ║" << std::endl;
    std::cout << "║ • Marginally: Poles on jω-axis (sustainted oscillation)      ║" << std::endl;
    std::cout << "║ • Unstable:   Any pole in RHP (Re(s) > 0)                    ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║ Routh-Hurwitz Criterion:                                     ║" << std::endl;
    std::cout << "║ • Number of sign changes in first column = RHP poles        ║" << std::endl;
    std::cout << "║ • Useful for finding parameter ranges for stability         ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║ Practical Guidelines:                                        ║" << std::endl;
    std::cout << "║ • All coefficients must have same sign (necessary)          ║" << std::endl;
    std::cout << "║ • No coefficients can be zero (necessary for n ≥ 2)         ║" << std::endl;
    std::cout << "║ • Use Routh table for sufficient condition                  ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    return 0;
}
