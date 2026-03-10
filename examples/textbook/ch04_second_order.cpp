/**
 * @file ch04_second_order.cpp
 * @brief Chapter 4: Second-order system analysis
 * 
 * This example demonstrates:
 * - Creating second-order transfer functions
 * - Effect of damping ratio on system response
 * - Computing performance specifications
 * - Relationship between pole location and transient behavior
 * 
 * Compile: g++ -std=c++14 -I "../../include" ch04_second_order.cpp -o ch04_second_order.exe
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <complex>

using namespace cppplot;
using namespace cppplot::control;

// Helper function: calculate overshoot from damping ratio
double overshoot_from_zeta(double zeta) {
    if (zeta >= 1.0) return 0.0;
    return 100.0 * std::exp(-M_PI * zeta / std::sqrt(1.0 - zeta * zeta));
}

// Helper function: calculate damping ratio from overshoot
double zeta_from_overshoot(double os_percent) {
    if (os_percent <= 0) return 1.0;
    double ln_os = std::log(os_percent / 100.0);
    return -ln_os / std::sqrt(M_PI * M_PI + ln_os * ln_os);
}

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║    Chapter 4: Second-Order System Time-Domain Analysis       ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // ========================================================================
    // Part 1: Standard Second-Order Form
    // ========================================================================
    std::cout << "\n▶ PART 1: Standard Second-Order Form" << std::endl;
    std::cout << "   ────────────────────────────────────" << std::endl;
    
    std::cout << "\n   Standard form: G(s) = ωn² / (s² + 2ζωn·s + ωn²)" << std::endl;
    std::cout << "\n   Where:" << std::endl;
    std::cout << "   • ωn = natural frequency [rad/s]" << std::endl;
    std::cout << "   • ζ  = damping ratio [dimensionless]" << std::endl;
    
    double wn = 2.0;  // Natural frequency
    
    // Different damping ratios
    std::vector<double> zetas = {0.1, 0.3, 0.5, 0.707, 1.0, 2.0};
    std::vector<std::string> zeta_labels = {"0.1", "0.3", "0.5", "0.707", "1.0", "2.0"};
    
    // ========================================================================
    // Part 2: Effect of Damping Ratio on Step Response
    // ========================================================================
    std::cout << "\n▶ PART 2: Effect of Damping Ratio" << std::endl;
    std::cout << "   ─────────────────────────────────" << std::endl;
    
    figure(1200, 500);
    
    std::vector<std::string> colors = {"red", "orange", "gold", "green", "blue", "purple"};
    
    std::cout << "\n   ┌─────────────────────────────────────────────────────────────┐" << std::endl;
    std::cout << "   │   ζ   │  Response Type  │ Poles                              │" << std::endl;
    std::cout << "   ├───────┼─────────────────┼────────────────────────────────────┤" << std::endl;
    
    for (size_t i = 0; i < zetas.size(); ++i) {
        double zeta = zetas[i];
        
        // Create transfer function: G(s) = wn²/(s² + 2ζωn·s + ωn²)
        TransferFunction G({wn * wn}, {1, 2 * zeta * wn, wn * wn});
        
        auto [t, y] = step(G, 15.0);
        
        std::string label = "ζ = " + zeta_labels[i];
        plot(t, y, "-", {{"color", colors[i]}, {"linewidth", "2"}, {"label", label}});
        
        // Determine response type and poles
        std::string response_type;
        std::string poles_str;
        
        double sigma = zeta * wn;
        
        if (zeta < 1.0) {
            response_type = "Underdamped";
            double wd = wn * std::sqrt(1.0 - zeta * zeta);
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2);
            oss << -sigma << " ± j" << wd;
            poles_str = oss.str();
        } else if (std::abs(zeta - 1.0) < 1e-6) {
            response_type = "Critically damped";
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2);
            oss << -sigma << " (repeated)";
            poles_str = oss.str();
        } else {
            response_type = "Overdamped";
            double p1 = -zeta * wn + wn * std::sqrt(zeta * zeta - 1);
            double p2 = -zeta * wn - wn * std::sqrt(zeta * zeta - 1);
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2);
            oss << p1 << ", " << p2;
            poles_str = oss.str();
        }
        
        std::cout << "   │ " << std::setw(5) << zeta_labels[i] << " │ " 
                  << std::setw(15) << response_type << " │ " 
                  << std::setw(34) << poles_str << " │" << std::endl;
    }
    
    std::cout << "   └─────────────────────────────────────────────────────────────┘" << std::endl;
    
    axhline(1.0, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.3"}});
    
    xlabel("Time [s]");
    ylabel("Output y(t)");
    title("Second-Order Step Response (ωn = 2 rad/s): Effect of Damping Ratio");
    legend();
    grid(true);
    
    savefig("ch04_damping_effect.svg");
    std::cout << "\n   ✓ Plot saved: ch04_damping_effect.svg" << std::endl;
    
    // ========================================================================
    // Part 3: Pole Locations in s-Plane
    // ========================================================================
    std::cout << "\n▶ PART 3: Pole Locations in s-Plane" << std::endl;
    std::cout << "   ───────────────────────────────────" << std::endl;
    
    figure(800, 700);
    
    // Draw circles for constant |s| = wn
    std::vector<double> theta_circle, x_circle, y_circle;
    for (int i = 0; i <= 100; ++i) {
        double th = M_PI/2 + i * M_PI / 100;  // Left half only
        x_circle.push_back(wn * std::cos(th));
        y_circle.push_back(wn * std::sin(th));
    }
    plot(x_circle, y_circle, "k--", {{"alpha", "0.3"}, {"label", "|s| = ωn"}});
    
    // Draw lines for constant damping ratio
    std::vector<double> zeta_lines = {0.3, 0.5, 0.707};
    for (double z : zeta_lines) {
        double angle = std::acos(z);
        std::vector<double> xline = {0, -3 * std::cos(angle)};
        std::vector<double> yline = {0, 3 * std::sin(angle)};
        plot(xline, yline, ":", {{"color", "gray"}, {"alpha", "0.5"}});
        
        // Also draw in lower half
        yline = {0, -3 * std::sin(angle)};
        plot(xline, yline, ":", {{"color", "gray"}, {"alpha", "0.5"}});
    }
    
    // Plot poles for each damping ratio
    for (size_t i = 0; i < zetas.size(); ++i) {
        double zeta = zetas[i];
        double sigma = zeta * wn;
        
        if (zeta < 1.0) {
            // Complex conjugate poles
            double wd = wn * std::sqrt(1.0 - zeta * zeta);
            scatter({-sigma, -sigma}, {wd, -wd}, 
                    {{"color", colors[i]}, {"markersize", "12"}, 
                     {"label", "ζ = " + zeta_labels[i]}});
        } else if (std::abs(zeta - 1.0) < 1e-6) {
            // Repeated real poles
            scatter({-sigma}, {0.0}, 
                    {{"color", colors[i]}, {"markersize", "15"}, 
                     {"label", "ζ = 1 (critical)"}});
        } else {
            // Distinct real poles
            double p1 = -zeta * wn + wn * std::sqrt(zeta * zeta - 1);
            double p2 = -zeta * wn - wn * std::sqrt(zeta * zeta - 1);
            scatter({p1, p2}, {0.0, 0.0}, 
                    {{"color", colors[i]}, {"markersize", "12"}, 
                     {"label", "ζ = " + zeta_labels[i]}});
        }
    }
    
    // Draw axes
    axhline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    axvline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    
    xlabel("Real Part [σ]");
    ylabel("Imaginary Part [jω]");
    title("Pole Locations in s-Plane (ωn = 2 rad/s)");
    legend();
    grid(true);
    xlim(-5, 1);
    ylim(-3, 3);
    
    savefig("ch04_pole_locations.svg");
    std::cout << "\n   ✓ Plot saved: ch04_pole_locations.svg" << std::endl;
    
    // ========================================================================
    // Part 4: Performance Specifications
    // ========================================================================
    std::cout << "\n▶ PART 4: Performance Specifications" << std::endl;
    std::cout << "   ────────────────────────────────────" << std::endl;
    
    std::cout << "\n   ┌───────────────────────────────────────────────────────────────────┐" << std::endl;
    std::cout << "   │   ζ    │  tr [s]  │  ts [s]  │  tp [s]  │  OS [%]  │ OS theory │" << std::endl;
    std::cout << "   ├────────┼──────────┼──────────┼──────────┼──────────┼───────────┤" << std::endl;
    
    std::vector<double> zeta_perf, rise_times, settling_times, overshoots;
    
    for (size_t i = 0; i < zetas.size(); ++i) {
        double zeta = zetas[i];
        TransferFunction G({wn * wn}, {1, 2 * zeta * wn, wn * wn});
        auto [t, y] = step(G, 20.0, 1000);
        auto info = stepinfo(t, y);
        
        double os_theory = overshoot_from_zeta(zeta);
        
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "   │  " << std::setw(5) << zeta_labels[i] << " │ "
                  << std::setw(8) << info.rise_time << " │ "
                  << std::setw(8) << info.settling_time << " │ "
                  << std::setw(8) << info.peak_time << " │ "
                  << std::setw(8) << info.overshoot << " │ "
                  << std::setw(9) << os_theory << " │" << std::endl;
        
        zeta_perf.push_back(zeta);
        rise_times.push_back(info.rise_time);
        settling_times.push_back(info.settling_time);
        overshoots.push_back(info.overshoot);
    }
    std::cout << "   └───────────────────────────────────────────────────────────────────┘" << std::endl;
    
    // ========================================================================
    // Part 5: Performance vs Damping Ratio Curves
    // ========================================================================
    std::cout << "\n▶ PART 5: Performance vs Damping Ratio" << std::endl;
    std::cout << "   ──────────────────────────────────────" << std::endl;
    
    figure(1200, 500);
    
    // Generate continuous curves
    std::vector<double> zeta_cont, tr_cont, ts_cont, os_cont;
    
    for (double z = 0.05; z <= 2.0; z += 0.02) {
        TransferFunction G({wn * wn}, {1, 2 * z * wn, wn * wn});
        auto [t, y] = step(G, 30.0, 500);
        auto info = stepinfo(t, y);
        
        zeta_cont.push_back(z);
        tr_cont.push_back(info.rise_time);
        ts_cont.push_back(info.settling_time);
        os_cont.push_back(info.overshoot);
    }
    
    // Plot time specifications
    subplot(1, 2, 1);
    plot(zeta_cont, tr_cont, "b-", {{"linewidth", "2"}, {"label", "Rise Time"}});
    plot(zeta_cont, ts_cont, "r-", {{"linewidth", "2"}, {"label", "Settling Time"}});
    
    // Mark optimal region
    axvline(0.707, {{"color", "green"}, {"linestyle", "--"}, {"alpha", "0.7"}, {"label", "ζ = 0.707"}});
    
    xlabel("Damping Ratio ζ");
    ylabel("Time [s]");
    title("Time Specifications vs Damping Ratio");
    legend();
    grid(true);
    ylim(0, 15);
    
    // Plot overshoot
    subplot(1, 2, 2);
    plot(zeta_cont, os_cont, "g-", {{"linewidth", "2"}, {"label", "Simulated"}});
    
    // Theoretical curve
    std::vector<double> os_theory_cont;
    for (double z : zeta_cont) {
        os_theory_cont.push_back(overshoot_from_zeta(z));
    }
    plot(zeta_cont, os_theory_cont, "r--", {{"linewidth", "1.5"}, {"label", "Theory: exp(-πζ/√(1-ζ²))"}});
    
    axvline(0.707, {{"color", "green"}, {"linestyle", "--"}, {"alpha", "0.7"}});
    
    xlabel("Damping Ratio ζ");
    ylabel("Overshoot [%]");
    title("Percent Overshoot vs Damping Ratio");
    legend();
    grid(true);
    
    savefig("ch04_performance_curves.svg");
    std::cout << "\n   ✓ Plot saved: ch04_performance_curves.svg" << std::endl;
    
    // ========================================================================
    // Part 6: Design Example
    // ========================================================================
    std::cout << "\n▶ PART 6: Design Example" << std::endl;
    std::cout << "   ────────────────────────" << std::endl;
    
    // Design requirements
    double os_spec = 10.0;   // Max overshoot: 10%
    double ts_spec = 2.0;    // Max settling time: 2s
    
    std::cout << "\n   Design Specifications:" << std::endl;
    std::cout << "   • Maximum Overshoot: " << os_spec << "%" << std::endl;
    std::cout << "   • Maximum Settling Time: " << ts_spec << " s" << std::endl;
    
    // Calculate required parameters
    double zeta_req = zeta_from_overshoot(os_spec);
    double sigma_req = 4.0 / ts_spec;  // σ = 4/ts for 2% criterion
    double wn_req = sigma_req / zeta_req;
    
    std::cout << "\n   Design Calculations:" << std::endl;
    std::cout << "   • Required ζ from OS = 10%: ζ = " << std::fixed << std::setprecision(3) << zeta_req << std::endl;
    std::cout << "   • Required σ from ts = 2s:  σ = 4/ts = " << sigma_req << std::endl;
    std::cout << "   • Required ωn = σ/ζ = " << wn_req << " rad/s" << std::endl;
    
    // Create designed system
    TransferFunction G_designed({wn_req * wn_req}, {1, 2 * zeta_req * wn_req, wn_req * wn_req});
    
    // Verify design
    auto [t_d, y_d] = step(G_designed, 5.0);
    auto info_d = stepinfo(t_d, y_d);
    
    std::cout << "\n   Verification:" << std::endl;
    std::cout << "   • Actual Overshoot: " << info_d.overshoot << "% (spec: ≤" << os_spec << "%)" << std::endl;
    std::cout << "   • Actual Settling Time: " << info_d.settling_time << " s (spec: ≤" << ts_spec << " s)" << std::endl;
    
    if (info_d.overshoot <= os_spec && info_d.settling_time <= ts_spec) {
        std::cout << "\n   ✓ Design meets specifications!" << std::endl;
    } else {
        std::cout << "\n   ✗ Design needs refinement." << std::endl;
    }
    
    // Plot designed system response
    figure(800, 500);
    
    plot(t_d, y_d, "b-", {{"linewidth", "2"}, {"label", "Designed System"}});
    axhline(1.0, {{"color", "black"}, {"linestyle", "-"}, {"alpha", "0.3"}});
    axhline(1.0 + os_spec/100, {{"color", "red"}, {"linestyle", "--"}, {"label", "OS limit"}});
    axvline(ts_spec, {{"color", "green"}, {"linestyle", "--"}, {"label", "ts limit"}});
    
    xlabel("Time [s]");
    ylabel("Output");
    std::ostringstream title_oss;
    title_oss << std::fixed << std::setprecision(2);
    title_oss << "Designed System: ωn = " << wn_req << " rad/s, ζ = " << zeta_req;
    title(title_oss.str());
    legend();
    grid(true);
    
    savefig("ch04_design_example.svg");
    std::cout << "\n   ✓ Plot saved: ch04_design_example.svg" << std::endl;
    
    // ========================================================================
    // Summary
    // ========================================================================
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                         SUMMARY                              ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Second-order system: G(s) = ωn²/(s² + 2ζωn·s + ωn²)          ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║ Response types:                                              ║" << std::endl;
    std::cout << "║ • ζ < 1: Underdamped (oscillatory)                           ║" << std::endl;
    std::cout << "║ • ζ = 1: Critically damped (fastest non-oscillatory)         ║" << std::endl;
    std::cout << "║ • ζ > 1: Overdamped (slow, no oscillation)                   ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║ Design formulas (underdamped):                               ║" << std::endl;
    std::cout << "║ • Overshoot: OS = exp(-πζ/√(1-ζ²)) × 100%                    ║" << std::endl;
    std::cout << "║ • Peak time: tp = π/ωd = π/(ωn√(1-ζ²))                       ║" << std::endl;
    std::cout << "║ • Settling time (2%): ts ≈ 4/(ζωn)                           ║" << std::endl;
    std::cout << "║ • Rise time: tr ≈ (1.8)/ωn  (approximate)                    ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║ Optimal damping: ζ ≈ 0.707 balances speed and overshoot      ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    return 0;
}
