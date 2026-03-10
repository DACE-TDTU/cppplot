/**
 * @file ch03_pole_zero.cpp
 * @brief Chapter 3: Pole-Zero Analysis and Time Response Relationship
 * 
 * Learning Outcomes:
 * - Visualize pole locations in the s-plane
 * - Understand how pole positions affect time response
 * - Analyze stability from pole locations
 * - Observe effects of zeros on system response
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
    std::cout << "║   Chapter 3: Pole-Zero Analysis and Time Response            ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    double t_final = 8.0;
    
    // ========================================================================
    // Part 1: Effect of Real Pole Location
    // ========================================================================
    std::cout << "\n▶ Part 1: Real Pole Location" << std::endl;
    std::cout << "  ───────────────────────────" << std::endl;
    
    std::cout << "  For a first-order system G(s) = 1/(τs+1) = 1/((1/p)s+1)" << std::endl;
    std::cout << "  Pole location p determines time constant τ = -1/p" << std::endl;
    
    figure(1400, 1000);
    
    subplot(2, 3, 1);
    
    std::vector<double> real_poles = {-0.5, -1, -2, -5};
    std::vector<std::string> colors = {"#E74C3C", "#E67E22", "#2ECC71", "#3498DB"};
    
    for (size_t i = 0; i < real_poles.size(); ++i) {
        double p = real_poles[i];
        double tau = -1.0 / p;
        
        // G(s) = |p|/(s+|p|) to have DC gain = 1
        TransferFunction G({-p}, {1, -p});
        auto [t, y] = step(G, t_final);
        
        std::ostringstream label;
        label << "p=" << p << " (τ=" << std::fixed << std::setprecision(1) << tau << "s)";
        plot(t, y, "-", {{"color", colors[i]}, {"linewidth", "2"}, {"label", label.str()}});
        
        std::cout << "  Pole at s = " << p << ": τ = " << tau << " s" << std::endl;
    }
    
    axhline(1.0, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.3"}});
    axhline(0.632, {{"color", "gray"}, {"linestyle", ":"}, {"alpha", "0.5"}, {"label", "63.2%"}});
    xlabel("Time [s]");
    ylabel("Response");
    title("Effect of Real Pole Location");
    legend({{"fontsize", "8"}, {"loc", "lower right"}});
    grid(true);
    
    // ========================================================================
    // Part 2: Complex Poles - Damping Effect
    // ========================================================================
    std::cout << "\n▶ Part 2: Complex Poles - Effect of Damping" << std::endl;
    std::cout << "  ──────────────────────────────────────────" << std::endl;
    
    subplot(2, 3, 2);
    
    double wn = 2.0;  // Natural frequency
    std::vector<double> zeta_values = {0.1, 0.3, 0.5, 0.707, 1.0};
    std::vector<std::string> zeta_colors = {"#E74C3C", "#E67E22", "#F1C40F", "#2ECC71", "#3498DB"};
    
    for (size_t i = 0; i < zeta_values.size(); ++i) {
        double z = zeta_values[i];
        
        // G(s) = ωn²/(s² + 2ζωn·s + ωn²)
        TransferFunction G({wn*wn}, {1, 2*z*wn, wn*wn});
        auto [t, y] = step(G, t_final);
        
        std::ostringstream label;
        label << "ζ = " << std::fixed << std::setprecision(2) << z;
        plot(t, y, "-", {{"color", zeta_colors[i]}, {"linewidth", "2"}, {"label", label.str()}});
        
        // Calculate pole locations
        if (z < 1) {
            double sigma = z * wn;
            double wd = wn * std::sqrt(1 - z*z);
            std::cout << "  ζ = " << z << ": poles at s = -" << sigma << " ± j" << wd << std::endl;
        } else {
            std::cout << "  ζ = " << z << ": two real poles" << std::endl;
        }
    }
    
    axhline(1.0, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.3"}});
    xlabel("Time [s]");
    ylabel("Response");
    title("Effect of Damping Ratio (ωn = 2 rad/s)");
    legend({{"fontsize", "8"}});
    grid(true);
    
    // ========================================================================
    // Part 3: S-Plane Visualization
    // ========================================================================
    std::cout << "\n▶ Part 3: S-Plane Map" << std::endl;
    std::cout << "  ─────────────────────" << std::endl;
    
    subplot(2, 3, 3);
    
    // Stability boundary (imaginary axis)
    axvline(0, {{"color", "red"}, {"linestyle", "--"}, {"linewidth", "2"}, {"alpha", "0.8"}});
    
    // Axes
    axhline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    
    // Constant damping ratio lines (ζ = cos(θ))
    std::vector<double> zeta_lines = {0.3, 0.5, 0.707, 0.9};
    for (double zl : zeta_lines) {
        double angle = std::acos(zl);
        std::vector<double> line_re, line_im_p, line_im_n;
        for (double r = 0; r <= 4; r += 0.1) {
            line_re.push_back(-r * std::cos(angle));
            line_im_p.push_back(r * std::sin(angle));
            line_im_n.push_back(-r * std::sin(angle));
        }
        plot(line_re, line_im_p, ":", {{"color", "gray"}, {"alpha", "0.5"}});
        plot(line_re, line_im_n, ":", {{"color", "gray"}, {"alpha", "0.5"}});
    }
    
    // Constant ωn circles
    for (double wn_c : {1.0, 2.0, 3.0}) {
        std::vector<double> circ_re, circ_im;
        for (int j = 0; j <= 50; ++j) {
            double ang = M_PI/2 + j * M_PI / 50;
            circ_re.push_back(wn_c * std::cos(ang));
            circ_im.push_back(wn_c * std::sin(ang));
        }
        plot(circ_re, circ_im, ":", {{"color", "blue"}, {"alpha", "0.3"}});
    }
    
    // Plot poles for the damping sweep
    for (size_t i = 0; i < zeta_values.size(); ++i) {
        double z = zeta_values[i];
        if (z < 1) {
            double sigma = -z * wn;
            double wd = wn * std::sqrt(1 - z*z);
            scatter({sigma, sigma}, {wd, -wd}, {{"color", zeta_colors[i]}, {"s", "80"}, {"marker", "x"}});
        } else {
            // Two real poles
            double p1 = -z * wn + wn * std::sqrt(z*z - 1);
            double p2 = -z * wn - wn * std::sqrt(z*z - 1);
            scatter({p1, p2}, {0.0, 0.0}, {{"color", zeta_colors[i]}, {"s", "80"}, {"marker", "x"}});
        }
    }
    
    // Annotate regions
    text(-2.5, 2, "STABLE", {{"fontsize", "10"}, {"color", "green"}});
    text(0.5, 2, "UNSTABLE", {{"fontsize", "10"}, {"color", "red"}});
    
    xlabel("Real (σ)");
    ylabel("Imaginary (jω)");
    title("S-Plane: Pole Map");
    xlim(-5, 2);
    ylim(-4, 4);
    grid(true);
    
    // ========================================================================
    // Part 4: Effect of Zero Location
    // ========================================================================
    std::cout << "\n▶ Part 4: Effect of Zeros" << std::endl;
    std::cout << "  ────────────────────────" << std::endl;
    
    subplot(2, 3, 4);
    
    // Base system: poles at -1 ± j1 (ωn=√2, ζ=0.707)
    double base_wn = std::sqrt(2);
    double base_zeta = 0.707;
    
    std::cout << "  Base system: poles at s = -1 ± j1" << std::endl;
    
    // No zero
    TransferFunction G0({2}, {1, 2, 2});
    auto [t0, y0] = step(G0, t_final);
    plot(t0, y0, "k-", {{"linewidth", "2"}, {"label", "No zero"}});
    
    // Zero in LHP (speeds up response)
    TransferFunction G_lhp({2, 8}, {1, 2, 2});  // Zero at s = -4
    auto [t1, y1] = step(G_lhp, t_final);
    plot(t1, y1, "b-", {{"linewidth", "2"}, {"label", "Zero at s=-4 (LHP)"}});
    std::cout << "  LHP zero at s=-4: Speeds up response" << std::endl;
    
    // Zero near origin (increases overshoot)  
    TransferFunction G_near({2, 1}, {1, 2, 2});  // Zero at s = -0.5
    auto [t2, y2] = step(G_near, t_final);
    plot(t2, y2, "g-", {{"linewidth", "2"}, {"label", "Zero at s=-0.5"}});
    std::cout << "  Near-origin zero at s=-0.5: Increases overshoot" << std::endl;
    
    // Zero in RHP (non-minimum phase, initial undershoot)
    TransferFunction G_rhp({2, -2}, {1, 2, 2});  // Zero at s = +1
    auto [t3, y3] = step(G_rhp, t_final);
    plot(t3, y3, "r-", {{"linewidth", "2"}, {"label", "Zero at s=+1 (RHP)"}});
    std::cout << "  RHP zero at s=+1: Non-minimum phase (undershoot)" << std::endl;
    
    axhline(1.0, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.3"}});
    axhline(0, {{"color", "gray"}, {"linestyle", ":"}, {"alpha", "0.3"}});
    xlabel("Time [s]");
    ylabel("Response");
    title("Effect of Zero Location");
    legend({{"fontsize", "8"}});
    grid(true);
    
    // ========================================================================
    // Part 5: Stability Examples
    // ========================================================================
    std::cout << "\n▶ Part 5: Stability Comparison" << std::endl;
    std::cout << "  ─────────────────────────────" << std::endl;
    
    subplot(2, 3, 5);
    
    // Stable: pole at s = -1
    TransferFunction G_stable({1}, {1, 1});
    auto [ts, ys] = step(G_stable, 5.0);
    plot(ts, ys, "g-", {{"linewidth", "2"}, {"label", "Stable (p=-1)"}});
    std::cout << "  Stable: pole at s=-1 → exponential decay to steady state" << std::endl;
    
    // Marginally stable: pole at s = 0 (integrator)
    std::vector<double> t_int, y_int;
    for (double ti = 0; ti <= 5; ti += 0.02) {
        t_int.push_back(ti);
        y_int.push_back(ti);  // Ramp for step input to integrator
    }
    plot(t_int, y_int, "-", {{"color", "orange"}, {"linewidth", "2"}, {"label", "Integrator (p=0)"}});
    std::cout << "  Marginal: pole at s=0 → unbounded growth (ramp)" << std::endl;
    
    // Unstable: pole at s = +0.5
    TransferFunction G_unstable({1}, {1, -0.5});
    auto [tu, yu] = step(G_unstable, 5.0);
    std::vector<double> yu_clip;
    for (double val : yu) yu_clip.push_back(std::min(val, 15.0));
    plot(tu, yu_clip, "r-", {{"linewidth", "2"}, {"label", "Unstable (p=+0.5)"}});
    std::cout << "  Unstable: pole at s=+0.5 → exponential growth" << std::endl;
    
    // Oscillatory marginal: poles on jω axis
    std::vector<double> t_osc, y_osc;
    for (double ti = 0; ti <= 5; ti += 0.02) {
        t_osc.push_back(ti);
        y_osc.push_back(1 - std::cos(2*ti));  // Sustained oscillation
    }
    plot(t_osc, y_osc, "m-", {{"linewidth", "2"}, {"label", "Oscillatory (p=±j2)"}});
    std::cout << "  Oscillatory: poles on jω axis → sustained oscillation" << std::endl;
    
    xlabel("Time [s]");
    ylabel("Response");
    title("Stability Classification");
    legend({{"fontsize", "8"}});
    grid(true);
    ylim(-1, 10);
    
    // ========================================================================
    // Part 6: Dominant Poles
    // ========================================================================
    std::cout << "\n▶ Part 6: Dominant Poles" << std::endl;
    std::cout << "  ────────────────────────" << std::endl;
    
    subplot(2, 3, 6);
    
    // System with dominant and fast poles
    // Dominant poles at -1±j1, fast pole at -10
    // G(s) = 20/(s+10) * 2/((s+1)²+1) = 40/[(s+10)(s²+2s+2)]
    
    // Full system
    TransferFunction G_full({40}, {1, 12, 22, 20});
    auto [tf, yf] = step(G_full, t_final);
    plot(tf, yf, "b-", {{"linewidth", "2"}, {"label", "Full system"}});
    
    // Dominant approximation (ignoring fast pole)
    TransferFunction G_dom({2}, {1, 2, 2});
    auto [td, yd] = step(G_dom, t_final);
    plot(td, yd, "r--", {{"linewidth", "2"}, {"label", "Dominant approx."}});
    
    std::cout << "  Full system: poles at -1±j1 (dominant), -10 (fast)" << std::endl;
    std::cout << "  Dominant approximation neglects fast pole (5x rule)" << std::endl;
    
    axhline(1.0, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.3"}});
    xlabel("Time [s]");
    ylabel("Response");
    title("Dominant Pole Approximation");
    legend();
    grid(true);
    
    savefig("ch03_pole_zero_analysis.svg");
    std::cout << "\n✓ Saved ch03_pole_zero_analysis.svg" << std::endl;
    
    // ========================================================================
    // Summary
    // ========================================================================
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                    KEY INSIGHTS                               ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  POLE LOCATION → STABILITY                                   ║" << std::endl;
    std::cout << "║  • Left of jω-axis (σ < 0): STABLE                          ║" << std::endl;
    std::cout << "║  • On jω-axis (σ = 0): MARGINALLY STABLE                    ║" << std::endl;
    std::cout << "║  • Right of jω-axis (σ > 0): UNSTABLE                       ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  POLE LOCATION → RESPONSE SPEED                              ║" << std::endl;
    std::cout << "║  • Further left → Faster response                           ║" << std::endl;
    std::cout << "║  • τ = 1/|σ| is the time constant                           ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  POLE LOCATION → OSCILLATION                                 ║" << std::endl;
    std::cout << "║  • Pure real poles: No oscillation                          ║" << std::endl;
    std::cout << "║  • Complex poles: Oscillation at ωd = |Im(pole)|            ║" << std::endl;
    std::cout << "║  • ζ = cos(angle from negative real axis)                   ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  ZERO LOCATION → RESPONSE SHAPE                              ║" << std::endl;
    std::cout << "║  • LHP zeros: Can speed up response                         ║" << std::endl;
    std::cout << "║  • Near-origin zeros: Increase overshoot                    ║" << std::endl;
    std::cout << "║  • RHP zeros: Cause initial undershoot (non-min phase)      ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    return 0;
}
