/**
 * @file ch04_second_order_systems.cpp
 * @brief Chapter 4: Second-Order System Analysis and Performance Specifications
 * 
 * Learning Outcomes:
 * - Relate physical parameters to ωn and ζ
 * - Calculate performance specs: rise time, overshoot, settling time
 * - Design for specified performance
 * 
 * Physical System: Mass-Spring-Damper (Car Suspension)
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
    std::cout << "║     Chapter 4: Second-Order System Analysis                  ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // ========================================================================
    // PHYSICAL SYSTEM: Quarter-Car Suspension Model
    // ========================================================================
    std::cout << "\n▶ Physical System: Quarter-Car Suspension" << std::endl;
    std::cout << "  ─────────────────────────────────────────" << std::endl;
    
    std::cout << R"(
        ┌──────────────────┐
        │   Car Body (m)   │  x(t): body displacement
        └────────┬─────────┘
             ╔═══╧═══╗
             ║ Spring ║  k (stiffness)
             ║   k    ║
             ╚═══╤═══╝
             ┌───┴───┐
             │Damper │  b (damping coefficient)
             │   b   │
             └───┬───┘
                 │
    ─────────────┴───────────────  Road: r(t) input
)" << std::endl;

    // Physical parameters (typical passenger car quarter)
    double m = 250;       // Sprung mass (quarter car body) [kg]
    double k = 15000;     // Spring stiffness [N/m]
    double b = 1500;      // Damping coefficient [N·s/m]
    
    std::cout << "  Physical Parameters:" << std::endl;
    std::cout << "    Body mass m = " << m << " kg" << std::endl;
    std::cout << "    Spring k = " << k << " N/m" << std::endl;
    std::cout << "    Damper b = " << b << " N·s/m" << std::endl;
    
    // ========================================================================
    // DERIVE SYSTEM PARAMETERS
    // ========================================================================
    std::cout << "\n▶ Derived Parameters" << std::endl;
    std::cout << "  ────────────────────" << std::endl;
    
    double wn = std::sqrt(k / m);               // Natural frequency [rad/s]
    double fn = wn / (2 * M_PI);                // Natural frequency [Hz]
    double zeta = b / (2 * std::sqrt(k * m));   // Damping ratio
    double wd = wn * std::sqrt(1 - zeta*zeta);  // Damped frequency
    double fd = wd / (2 * M_PI);                // Damped frequency [Hz]
    
    std::cout << "  Natural frequency ωn = √(k/m) = " << std::fixed << std::setprecision(2) 
              << wn << " rad/s (" << fn << " Hz)" << std::endl;
    std::cout << "  Damping ratio ζ = b/(2√(km)) = " << zeta << std::endl;
    std::cout << "  Damped frequency ωd = ωn√(1-ζ²) = " << wd << " rad/s (" << fd << " Hz)" << std::endl;
    
    // Critical damping
    double b_critical = 2 * std::sqrt(k * m);
    std::cout << "\n  Critical damping b_c = 2√(km) = " << b_critical << " N·s/m" << std::endl;
    std::cout << "  Current damping is " << (zeta < 1 ? "under" : (zeta > 1 ? "over" : "critically")) << "damped" << std::endl;
    
    // ========================================================================
    // TRANSFER FUNCTION
    // ========================================================================
    std::cout << "\n▶ Transfer Function" << std::endl;
    std::cout << "  ─────────────────────" << std::endl;
    
    // Standard form: G(s) = ωn²/(s² + 2ζωn·s + ωn²)
    std::cout << "  Standard form: G(s) = ωn²/(s² + 2ζωn·s + ωn²)" << std::endl;
    std::cout << "  G(s) = " << wn*wn << "/(s² + " << 2*zeta*wn << "s + " << wn*wn << ")" << std::endl;
    
    TransferFunction G({wn*wn}, {1, 2*zeta*wn, wn*wn});
    
    // ========================================================================
    // PERFORMANCE SPECIFICATIONS
    // ========================================================================
    std::cout << "\n▶ Performance Specifications" << std::endl;
    std::cout << "  ────────────────────────────" << std::endl;
    
    // For underdamped systems (0 < ζ < 1):
    // Rise time (0-100%): tr ≈ (1.8/ωn) for ζ = 0.5
    // Overshoot: Mp = exp(-πζ/√(1-ζ²))
    // Peak time: tp = π/ωd
    // Settling time (2%): ts ≈ 4/(ζωn)
    
    double Mp = std::exp(-M_PI * zeta / std::sqrt(1 - zeta*zeta)) * 100;  // Percent overshoot
    double tp = M_PI / wd;                                                 // Peak time
    double ts_2pct = 4 / (zeta * wn);                                      // 2% settling time
    double ts_5pct = 3 / (zeta * wn);                                      // 5% settling time
    double tr = (1.0 + 1.1*zeta + 1.4*zeta*zeta) / wn;                     // Rise time approximation
    
    std::cout << "  Peak time tp = π/ωd = " << tp << " s" << std::endl;
    std::cout << "  Rise time tr ≈ " << tr << " s" << std::endl;
    std::cout << "  Overshoot Mp = " << Mp << "%" << std::endl;
    std::cout << "  Settling time (2%) ts ≈ 4/(ζωn) = " << ts_2pct << " s" << std::endl;
    std::cout << "  Settling time (5%) ts ≈ 3/(ζωn) = " << ts_5pct << " s" << std::endl;
    
    // ========================================================================
    // SIMULATION
    // ========================================================================
    double t_final = 2.0;  // 2 seconds
    auto [t, y] = step(G, t_final);
    
    figure(1600, 1200);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 1: Step Response with Performance Metrics
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 3, 1);
    
    plot(t, y, "b-", {{"linewidth", "2"}, {"label", "Response"}});
    
    // Steady state and bounds
    axhline(1.0, {{"color", "green"}, {"linestyle", "--"}, {"alpha", "0.7"}});
    axhline(1.02, {{"color", "red"}, {"linestyle", ":", {"alpha", "0.5"}, {"label", "±2% bounds"}}});
    axhline(0.98, {{"color", "red"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    
    // Peak point
    double y_peak = 1 + Mp/100;
    scatter({tp}, {y_peak}, {{"color", "red"}, {"s", "100"}, {"zorder", "5"}});
    
    // Settling time indicator
    axvline(ts_2pct, {{"color", "orange"}, {"linestyle", "--"}, {"alpha", "0.7"}, {"label", "ts (2%)"}});
    
    xlabel("Time [s]");
    ylabel("Displacement");
    title("Step Response: Quarter-Car Suspension");
    legend({{"fontsize", "8"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 2: Effect of Damping Ratio
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 3, 2);
    
    std::vector<double> zeta_sweep = {0.1, 0.3, 0.5, 0.707, 1.0, 1.5};
    std::vector<std::string> colors = {"#E74C3C", "#E67E22", "#F1C40F", "#2ECC71", "#3498DB", "#9B59B6"};
    
    for (size_t i = 0; i < zeta_sweep.size(); ++i) {
        double z = zeta_sweep[i];
        TransferFunction Gz({wn*wn}, {1, 2*z*wn, wn*wn});
        auto [tz, yz] = step(Gz, t_final);
        
        std::ostringstream label;
        label << "ζ=" << std::fixed << std::setprecision(2) << z;
        plot(tz, yz, "-", {{"color", colors[i]}, {"linewidth", "2"}, {"label", label.str()}});
    }
    
    axhline(1.0, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.3"}});
    xlabel("Time [s]");
    ylabel("Response");
    title("Effect of Damping Ratio (ωn = " + std::to_string(int(wn)) + " rad/s)");
    legend({{"fontsize", "8"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 3: Overshoot vs Damping Ratio
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 3, 3);
    
    std::vector<double> z_range, Mp_range;
    for (double z = 0.05; z <= 1.0; z += 0.01) {
        z_range.push_back(z);
        double overshoot = std::exp(-M_PI * z / std::sqrt(1 - z*z)) * 100;
        Mp_range.push_back(overshoot);
    }
    
    plot(z_range, Mp_range, "b-", {{"linewidth", "2"}});
    
    // Mark common values
    scatter({0.5}, {std::exp(-M_PI * 0.5 / std::sqrt(1 - 0.25)) * 100}, {{"color", "red"}, {"s", "80"}});
    scatter({0.707}, {std::exp(-M_PI * 0.707 / std::sqrt(1 - 0.5)) * 100}, {{"color", "green"}, {"s", "80"}});
    
    // Typical spec lines
    axhline(16.3, {{"color", "orange"}, {"linestyle", "--"}, {"label", "ζ=0.5: 16.3%"}});
    axhline(4.3, {{"color", "green"}, {"linestyle", "--"}, {"label", "ζ=0.707: 4.3%"}});
    
    xlabel("Damping Ratio ζ");
    ylabel("Percent Overshoot [%]");
    title("Overshoot vs Damping Ratio");
    legend({{"fontsize", "8"}});
    grid(true);
    xlim(0, 1);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 4: Effect of Natural Frequency
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 3, 4);
    
    std::vector<double> wn_values = {4, 6, 8, 10, 15};
    
    for (size_t i = 0; i < wn_values.size(); ++i) {
        double w = wn_values[i];
        TransferFunction Gw({w*w}, {1, 2*0.5*w, w*w});  // Fixed ζ = 0.5
        auto [tw, yw] = step(Gw, t_final);
        
        std::ostringstream label;
        label << "ωn=" << w << " rad/s";
        plot(tw, yw, "-", {{"linewidth", "2"}, {"label", label.str()}});
    }
    
    axhline(1.0, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.3"}});
    xlabel("Time [s]");
    ylabel("Response");
    title("Effect of Natural Frequency (ζ = 0.5)");
    legend({{"fontsize", "8"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 5: S-Plane Design Chart
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 3, 5);
    
    // Constant ζ lines (radial)
    for (double z : {0.3, 0.5, 0.707, 0.9}) {
        double angle = std::acos(z);
        std::vector<double> line_re, line_im;
        for (double r = 0; r <= 20; r += 0.5) {
            line_re.push_back(-r * std::cos(angle));
            line_im.push_back(r * std::sin(angle));
        }
        plot(line_re, line_im, ":", {{"color", "gray"}, {"alpha", "0.6"}});
        
        // Negative imaginary part
        std::vector<double> line_im_neg;
        for (double im : line_im) line_im_neg.push_back(-im);
        plot(line_re, line_im_neg, ":", {{"color", "gray"}, {"alpha", "0.6"}});
    }
    
    // Constant ωn circles
    for (double wn_c : {5.0, 10.0, 15.0}) {
        std::vector<double> circ_re, circ_im;
        for (int j = 0; j <= 50; ++j) {
            double ang = M_PI/2 + j * M_PI / 50;
            circ_re.push_back(wn_c * std::cos(ang));
            circ_im.push_back(wn_c * std::sin(ang));
        }
        plot(circ_re, circ_im, "--", {{"color", "blue"}, {"alpha", "0.4"}});
    }
    
    // Constant settling time lines (vertical)
    for (double ts : {0.5, 1.0, 1.5}) {
        double sigma = 4.0 / ts;  // For 2% settling
        axvline(-sigma, {{"color", "red"}, {"linestyle", "--"}, {"alpha", "0.3"}});
    }
    
    // Current system poles
    double sigma_p = zeta * wn;
    scatter({-sigma_p, -sigma_p}, {wd, -wd}, {{"color", "red"}, {"s", "120"}, {"marker", "x"}, {"zorder", "5"}});
    
    axhline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    axvline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    
    xlabel("Real (σ)");
    ylabel("Imaginary (jω)");
    title("S-Plane Design Chart");
    xlim(-15, 2);
    ylim(-15, 15);
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────────
    // Plot 6: Design Formulas Summary
    // ─────────────────────────────────────────────────────────────────────
    subplot(2, 3, 6);
    
    // Plot settling time vs damping (for fixed ωn)
    std::vector<double> z_ts, ts_values;
    for (double z = 0.1; z <= 1.0; z += 0.02) {
        z_ts.push_back(z);
        ts_values.push_back(4.0 / (z * wn));
    }
    
    plot(z_ts, ts_values, "b-", {{"linewidth", "2"}, {"label", "ts (2%) = 4/(ζωn)"}});
    
    // Mark current operating point
    scatter({zeta}, {ts_2pct}, {{"color", "red"}, {"s", "100"}, {"zorder", "5"}});
    
    xlabel("Damping Ratio ζ");
    ylabel("Settling Time [s]");
    title("Settling Time vs Damping (ωn = " + std::to_string(int(wn)) + ")");
    legend();
    grid(true);
    
    savefig("ch04_second_order_systems.svg");
    std::cout << "\n✓ Saved ch04_second_order_systems.svg" << std::endl;
    
    // ========================================================================
    // DESIGN EXAMPLE: Meet Specifications
    // ========================================================================
    std::cout << "\n▶ Design Example" << std::endl;
    std::cout << "  ────────────────" << std::endl;
    
    // Specs: Mp ≤ 10%, ts ≤ 1.0 s
    std::cout << "  Specifications: Mp ≤ 10%, ts (2%) ≤ 1.0 s" << std::endl;
    
    // From Mp ≤ 10%: ζ ≥ 0.59
    double zeta_min = -std::log(0.10) / std::sqrt(M_PI*M_PI + std::log(0.10)*std::log(0.10));
    std::cout << "  From Mp ≤ 10%: ζ ≥ " << std::fixed << std::setprecision(3) << zeta_min << std::endl;
    
    // From ts ≤ 1.0s: ζωn ≥ 4
    std::cout << "  From ts ≤ 1.0s: ζωn ≥ 4.0 rad/s" << std::endl;
    
    // Choosing ζ = 0.6, we need ωn ≥ 4/0.6 = 6.67 rad/s
    double zeta_design = 0.6;
    double wn_design = 4.0 / zeta_design;
    std::cout << "\n  Design choice: ζ = " << zeta_design << ", ωn = " << wn_design << " rad/s" << std::endl;
    
    // Calculate required physical parameters
    double k_design = wn_design * wn_design * m;
    double b_design = 2 * zeta_design * std::sqrt(k_design * m);
    
    std::cout << "  Required parameters:" << std::endl;
    std::cout << "    Spring k = " << k_design << " N/m" << std::endl;
    std::cout << "    Damper b = " << b_design << " N·s/m" << std::endl;
    
    // ========================================================================
    // SUMMARY
    // ========================================================================
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║          SECOND-ORDER SYSTEM DESIGN FORMULAS                 ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Standard Form: G(s) = ωn²/(s² + 2ζωn·s + ωn²)              ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Performance Specifications (0 < ζ < 1):                     ║" << std::endl;
    std::cout << "║  ┌──────────────────┬───────────────────────────────────┐   ║" << std::endl;
    std::cout << "║  │ Metric           │ Formula                            │   ║" << std::endl;
    std::cout << "║  ├──────────────────┼───────────────────────────────────┤   ║" << std::endl;
    std::cout << "║  │ Rise time        │ tr ≈ (1.8)/ωn (for ζ≈0.5)         │   ║" << std::endl;
    std::cout << "║  │ Peak time        │ tp = π/ωd                          │   ║" << std::endl;
    std::cout << "║  │ Overshoot        │ Mp = e^(-πζ/√(1-ζ²))              │   ║" << std::endl;
    std::cout << "║  │ Settling (2%)    │ ts ≈ 4/(ζωn)                       │   ║" << std::endl;
    std::cout << "║  │ Settling (5%)    │ ts ≈ 3/(ζωn)                       │   ║" << std::endl;
    std::cout << "║  └──────────────────┴───────────────────────────────────┘   ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Design Guidelines:                                          ║" << std::endl;
    std::cout << "║  • ↑ ωn → faster response, faster settling                  ║" << std::endl;
    std::cout << "║  • ↑ ζ  → less overshoot, but slower                        ║" << std::endl;
    std::cout << "║  • ζ = 0.707: Good balance (4.3% overshoot)                 ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    return 0;
}
