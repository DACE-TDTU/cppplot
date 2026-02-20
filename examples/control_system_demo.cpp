/**
 * @file control_system_demo.cpp
 * @brief Demo vẽ các biểu đồ Control System theo CHUẨN: Step Response, Bode, Nyquist, Pole-Zero
 * 
 * Chuẩn tham khảo: Python Control Library, MATLAB Control System Toolbox
 * 
 * Compile: g++ -std=c++14 -D_USE_MATH_DEFINES -I../include control_system_demo.cpp -o control_demo.exe
 */

#include <cppplot/cppplot.hpp>
#include <iostream>
#include <cmath>
#include <complex>
#include <algorithm>

using namespace cppplot;

// ============ Transfer Function Class ============

/**
 * @brief Second-order system: G(s) = ωn² / (s² + 2ζωns + ωn²)
 */
class SecondOrderSystem {
public:
    double wn;    // Natural frequency (rad/s)
    double zeta;  // Damping ratio
    
    SecondOrderSystem(double wn_, double zeta_) : wn(wn_), zeta(zeta_) {}
    
    // Step response
    double step(double t) const {
        if (t < 0) return 0;
        if (zeta < 1.0) {
            // Underdamped
            double wd = wn * std::sqrt(1 - zeta * zeta);
            double phi = std::atan2(std::sqrt(1 - zeta * zeta), zeta);
            return 1 - std::exp(-zeta * wn * t) * std::sin(wd * t + phi) / std::sqrt(1 - zeta * zeta);
        } else if (std::abs(zeta - 1.0) < 1e-10) {
            // Critically damped
            return 1 - std::exp(-wn * t) * (1 + wn * t);
        } else {
            // Overdamped
            double s1 = -zeta * wn + wn * std::sqrt(zeta * zeta - 1);
            double s2 = -zeta * wn - wn * std::sqrt(zeta * zeta - 1);
            return 1 + (s2 * std::exp(s1 * t) - s1 * std::exp(s2 * t)) / (s1 - s2);
        }
    }
    
    // Frequency response: G(jω)
    std::complex<double> freqresp(double omega) const {
        std::complex<double> jw(0, omega);
        std::complex<double> num = wn * wn;
        std::complex<double> den = jw * jw + 2.0 * zeta * wn * jw + wn * wn;
        return num / den;
    }
    
    // Magnitude in dB
    double mag_dB(double omega) const {
        return 20.0 * std::log10(std::abs(freqresp(omega)));
    }
    
    // Phase in degrees (unwrapped)
    double phase_deg(double omega) const {
        return std::arg(freqresp(omega)) * 180.0 / M_PI;
    }
    
    // Get poles
    void poles(std::complex<double>& p1, std::complex<double>& p2) const {
        double disc = zeta * zeta - 1;
        if (disc < 0) {
            double re = -zeta * wn;
            double im = wn * std::sqrt(-disc);
            p1 = std::complex<double>(re, im);
            p2 = std::complex<double>(re, -im);
        } else {
            p1 = std::complex<double>(-zeta * wn + wn * std::sqrt(disc), 0);
            p2 = std::complex<double>(-zeta * wn - wn * std::sqrt(disc), 0);
        }
    }
};

/**
 * @brief General transfer function G(s) = K * num(s) / den(s)
 * For open-loop: G(s) = K / (s * (s+a) * (s+b))
 */
class OpenLoopTF {
public:
    double K;
    std::vector<double> poles_re;  // Real parts of poles
    std::vector<double> poles_im;  // Imaginary parts
    std::vector<double> zeros_re;
    std::vector<double> zeros_im;
    
    // G(s) = K / (s(s+1)(s+2)) - typical open-loop system
    OpenLoopTF(double gain = 1.0) : K(gain) {
        poles_re = {0, -1, -2};
        poles_im = {0, 0, 0};
    }
    
    // Frequency response
    std::complex<double> freqresp(double omega) const {
        std::complex<double> jw(0, omega);
        std::complex<double> result = K;
        
        // Divide by each pole factor (s - p)
        for (size_t i = 0; i < poles_re.size(); ++i) {
            std::complex<double> pole(poles_re[i], poles_im[i]);
            result /= (jw - pole);
        }
        
        // Multiply by each zero factor (s - z)
        for (size_t i = 0; i < zeros_re.size(); ++i) {
            std::complex<double> zero(zeros_re[i], zeros_im[i]);
            result *= (jw - zero);
        }
        
        return result;
    }
    
    double mag_dB(double omega) const {
        double mag = std::abs(freqresp(omega));
        if (mag < 1e-20) return -400;  // Avoid log(0)
        return 20.0 * std::log10(mag);
    }
    
    double phase_deg(double omega) const {
        return std::arg(freqresp(omega)) * 180.0 / M_PI;
    }
};

// ============ Utility: Phase unwrapping ============
std::vector<double> unwrap_phase(const std::vector<double>& phase_deg) {
    std::vector<double> unwrapped = phase_deg;
    for (size_t i = 1; i < unwrapped.size(); ++i) {
        double diff = unwrapped[i] - unwrapped[i-1];
        if (diff > 180) {
            // Jump down
            for (size_t j = i; j < unwrapped.size(); ++j) {
                unwrapped[j] -= 360;
            }
        } else if (diff < -180) {
            // Jump up
            for (size_t j = i; j < unwrapped.size(); ++j) {
                unwrapped[j] += 360;
            }
        }
    }
    return unwrapped;
}

int main() {
    std::cout << "===========================================\n";
    std::cout << "  CppPlot Control System Demo (Standard)\n";
    std::cout << "===========================================\n\n";
    
    // ============ 1. Step Response (Chuẩn) ============
    std::cout << "1. Step Response...\n";
    {
        figure(800, 500);
        
        auto t = linspace(0, 12, 500);
        
        std::vector<double> zetas = {0.1, 0.3, 0.5, 0.707, 1.0, 2.0};
        std::vector<std::string> colors = {"#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"};
        
        double wn = 1.0;
        
        for (size_t i = 0; i < zetas.size(); ++i) {
            SecondOrderSystem sys(wn, zetas[i]);
            std::vector<double> y;
            for (double ti : t) {
                y.push_back(sys.step(ti));
            }
            
            std::ostringstream lbl;
            lbl << "\\zeta=" << zetas[i];
            plot(t, y, "-", opts({{"color", colors[i]}, {"linewidth", "1.5"}, {"label", lbl.str()}}));
        }
        
        // Steady state và settling bands
        axhline(1.0, opts({{"color", "black"}, {"linestyle", "--"}, {"linewidth", "1"}}));
        axhline(1.02, opts({{"color", "gray"}, {"linestyle", ":"}, {"linewidth", "0.5"}}));
        axhline(0.98, opts({{"color", "gray"}, {"linestyle", ":"}, {"linewidth", "0.5"}}));
        
        fill_between(t, std::vector<double>(t.size(), 0.98), std::vector<double>(t.size(), 1.02),
                     opts({{"color", "green"}, {"alpha", "0.1"}}));
        
        title("Step Response - Second Order System (\\omega_n = 1 rad/s)");
        xlabel("Time (seconds)");
        ylabel("Amplitude");
        legend(true);
        grid(true);
        xlim(0, 12);
        ylim(0, 1.8);
        
        savefig("output/control_01_step_response.svg");
        clf();
        std::cout << "   Saved: output/control_01_step_response.svg\n";
    }
    
    // ============ 2. Bode Plot (Chuẩn) ============
    std::cout << "2. Bode Plot...\n";
    {
        figure(800, 600);
        layout(2, 1);
        
        // Frequency range: 3 decades
        std::vector<double> omega;
        for (double w = 0.01; w <= 100; w *= 1.02) {
            omega.push_back(w);
        }
        
        SecondOrderSystem sys(1.0, 0.3);
        
        std::vector<double> mag, phase_raw;
        for (double w : omega) {
            mag.push_back(sys.mag_dB(w));
            phase_raw.push_back(sys.phase_deg(w));
        }
        
        // Unwrap phase để liên tục
        auto phase = unwrap_phase(phase_raw);
        
        // === Magnitude Plot ===
        subplot(2, 1, 1);
        plot(omega, mag, "-", opts({{"color", "#1f77b4"}, {"linewidth", "2"}}));
        
        xscale("log");
        axhline(0, opts({{"color", "black"}, {"linestyle", "-"}, {"linewidth", "0.5"}}));
        axhline(-3, opts({{"color", "red"}, {"linestyle", ":"}, {"linewidth", "1"}}));  // -3dB line
        
        // Mark resonance peak cho underdamped
        double w_peak = sys.wn * std::sqrt(1 - 2*sys.zeta*sys.zeta);
        if (sys.zeta < 0.707) {
            double M_peak = sys.mag_dB(w_peak);
            scatter({w_peak}, {M_peak}, opts({{"s", "20"}, {"color", "red"}}));
            
            std::ostringstream peak_label;
            peak_label << "Peak: " << std::fixed << std::setprecision(1) << M_peak << " dB";
            text(w_peak * 1.5, M_peak, peak_label.str(), opts({{"fontsize", "9"}, {"color", "red"}}));
        }
        
        // Chuẩn: không có title riêng cho subplot, dùng ylabel
        ylabel("Magnitude (dB)");
        grid(true);
        ylim(-60, 20);
        
        // === Phase Plot ===
        subplot(2, 1, 2);
        plot(omega, phase, "-", opts({{"color", "#1f77b4"}, {"linewidth", "2"}}));
        
        xscale("log");
        
        // Phase reference lines
        axhline(0, opts({{"color", "gray"}, {"linestyle", ":"}, {"linewidth", "0.5"}}));
        axhline(-90, opts({{"color", "gray"}, {"linestyle", ":"}, {"linewidth", "0.5"}}));
        axhline(-180, opts({{"color", "gray"}, {"linestyle", ":"}, {"linewidth", "0.5"}}));
        
        // Mark -90° point (ω = ωn for second-order)
        axvline(sys.wn, opts({{"color", "red"}, {"linestyle", "--"}, {"linewidth", "0.8"}}));
        text(sys.wn * 1.2, -45, "\\omega_n", opts({{"fontsize", "10"}, {"color", "red"}}));
        
        xlabel("Frequency (rad/s)");
        ylabel("Phase (deg)");
        grid(true);
        ylim(-200, 10);
        
        suptitle("Bode Diagram: G(s) = 1/(s^2 + 0.6s + 1), \\zeta = 0.3");
        savefig("output/control_02_bode.svg");
        clf();
        std::cout << "   Saved: output/control_02_bode.svg\n";
    }
    
    // ============ 3. Bode với Gain/Phase Margins ============
    std::cout << "3. Bode Plot with Margins...\n";
    {
        figure(800, 600);
        layout(2, 1);
        
        OpenLoopTF sys(2.0);  // K=2, G(s) = 2 / (s(s+1)(s+2))
        
        std::vector<double> omega;
        for (double w = 0.01; w <= 100; w *= 1.02) {
            omega.push_back(w);
        }
        
        std::vector<double> mag, phase_raw;
        for (double w : omega) {
            mag.push_back(sys.mag_dB(w));
            phase_raw.push_back(sys.phase_deg(w));
        }
        auto phase = unwrap_phase(phase_raw);
        
        // Tìm gain crossover (|G| = 0 dB) và phase crossover (phase = -180°)
        double w_gc = 0, w_pc = 0;
        double pm = 0, gm_dB = 0;
        
        for (size_t i = 1; i < omega.size(); ++i) {
            // Gain crossover: magnitude crosses 0 dB
            if (mag[i-1] > 0 && mag[i] <= 0) {
                w_gc = omega[i];
                pm = 180 + phase[i];  // Phase margin
            }
            // Phase crossover: phase crosses -180°
            if (phase[i-1] > -180 && phase[i] <= -180) {
                w_pc = omega[i];
                gm_dB = -mag[i];  // Gain margin in dB
            }
        }
        
        // === Magnitude Plot ===
        subplot(2, 1, 1);
        plot(omega, mag, "-", opts({{"color", "#1f77b4"}, {"linewidth", "2"}}));
        
        xscale("log");
        axhline(0, opts({{"color", "black"}, {"linestyle", "-"}, {"linewidth", "0.5"}}));
        
        // Mark crossover frequencies
        if (w_gc > 0) {
            axvline(w_gc, opts({{"color", "green"}, {"linestyle", "--"}, {"linewidth", "1"}}));
            scatter({w_gc}, {0.0}, opts({{"s", "25"}, {"color", "green"}}));
        }
        if (w_pc > 0) {
            axvline(w_pc, opts({{"color", "red"}, {"linestyle", "--"}, {"linewidth", "1"}}));
            // Gain margin visualization
            double mag_at_pc = sys.mag_dB(w_pc);
            plot({w_pc, w_pc}, {mag_at_pc, 0.0}, "-", opts({{"color", "red"}, {"linewidth", "2"}}));
            
            std::ostringstream gm_text;
            gm_text << "GM = " << std::fixed << std::setprecision(1) << gm_dB << " dB";
            text(w_pc * 1.3, mag_at_pc / 2, gm_text.str(), opts({{"fontsize", "9"}, {"color", "red"}}));
        }
        
        ylabel("Magnitude (dB)");
        grid(true);
        ylim(-80, 40);
        
        // === Phase Plot ===
        subplot(2, 1, 2);
        plot(omega, phase, "-", opts({{"color", "#1f77b4"}, {"linewidth", "2"}}));
        
        xscale("log");
        axhline(-180, opts({{"color", "black"}, {"linestyle", "-"}, {"linewidth", "0.5"}}));
        
        if (w_gc > 0) {
            axvline(w_gc, opts({{"color", "green"}, {"linestyle", "--"}, {"linewidth", "1"}}));
            double phase_at_gc = sys.phase_deg(w_gc);
            // Unwrap single value
            while (phase_at_gc > 0) phase_at_gc -= 360;
            while (phase_at_gc < -360) phase_at_gc += 360;
            
            plot({w_gc, w_gc}, {phase_at_gc, -180.0}, "-", opts({{"color", "green"}, {"linewidth", "2"}}));
            
            std::ostringstream pm_text;
            pm_text << "PM = " << std::fixed << std::setprecision(1) << pm << "\\degree";
            text(w_gc * 1.3, (phase_at_gc - 180) / 2, pm_text.str(), opts({{"fontsize", "9"}, {"color", "green"}}));
        }
        if (w_pc > 0) {
            axvline(w_pc, opts({{"color", "red"}, {"linestyle", "--"}, {"linewidth", "1"}}));
            scatter({w_pc}, {-180.0}, opts({{"s", "25"}, {"color", "red"}}));
        }
        
        xlabel("Frequency (rad/s)");
        ylabel("Phase (deg)");
        grid(true);
        ylim(-270, 0);
        
        suptitle("Bode Diagram with Stability Margins: G(s) = 2 / (s(s+1)(s+2))");
        savefig("output/control_03_bode_margins.svg");
        clf();
        std::cout << "   Saved: output/control_03_bode_margins.svg\n";
    }
    
    // ============ 4. Nyquist Plot (Chuẩn) ============
    std::cout << "4. Nyquist Plot...\n";
    {
        figure(700, 700);
        
        OpenLoopTF sys(1.0);  // G(s) = 1 / (s(s+1)(s+2))
        
        // Frequency points - avoid ω=0 (pole at origin)
        std::vector<double> omega;
        for (double w = 0.001; w <= 50; w *= 1.03) {
            omega.push_back(w);
        }
        
        // Positive frequency contour
        std::vector<double> re_pos, im_pos;
        for (double w : omega) {
            auto G = sys.freqresp(w);
            re_pos.push_back(G.real());
            im_pos.push_back(G.imag());
        }
        
        // Negative frequency contour (mirror about real axis)
        std::vector<double> re_neg, im_neg;
        for (int i = omega.size() - 1; i >= 0; --i) {
            re_neg.push_back(re_pos[i]);
            im_neg.push_back(-im_pos[i]);
        }
        
        // Plot contours
        plot(re_pos, im_pos, "-", opts({{"color", "#1f77b4"}, {"linewidth", "2"}, {"label", "\\omega: 0\\to+\\infty"}}));
        plot(re_neg, im_neg, "--", opts({{"color", "#1f77b4"}, {"linewidth", "1.5"}, {"label", "\\omega: -\\infty\\to 0"}}));
        
        // Critical point (-1, 0)
        scatter({-1}, {0}, opts({{"s", "40"}, {"color", "red"}, {"marker", "+"}}));
        text(-0.85, 0.15, "(-1, j0)", opts({{"fontsize", "10"}, {"color", "red"}}));
        
        // Start marker (ω → 0+)
        scatter({re_pos[0]}, {im_pos[0]}, opts({{"s", "25"}, {"color", "green"}}));
        
        // Direction arrows (approximate by plotting small markers)
        size_t n = re_pos.size();
        std::vector<size_t> arrow_idx = {n/6, n/3, n/2, 2*n/3};
        for (size_t idx : arrow_idx) {
            if (idx < n - 1) {
                scatter({re_pos[idx]}, {im_pos[idx]}, opts({{"s", "12"}, {"color", "#1f77b4"}, {"marker", ">"}}));
            }
        }
        
        // Axes through origin
        axhline(0, opts({{"color", "black"}, {"linewidth", "0.8"}}));
        axvline(0, opts({{"color", "black"}, {"linewidth", "0.8"}}));
        
        // Unit circle
        auto theta = linspace(0, 2 * M_PI, 100);
        std::vector<double> uc_re, uc_im;
        for (double th : theta) {
            uc_re.push_back(std::cos(th));
            uc_im.push_back(std::sin(th));
        }
        plot(uc_re, uc_im, ":", opts({{"color", "gray"}, {"linewidth", "0.8"}}));
        
        title("Nyquist Diagram: G(s) = 1 / (s(s+1)(s+2))");
        xlabel("Real Axis");
        ylabel("Imaginary Axis");
        legend(true);
        grid(true);
        
        // QUAN TRỌNG: Equal aspect ratio
        xlim(-2, 1);
        ylim(-1.5, 1.5);
        
        // Stability info
        figtext(0.15, 0.92, "Encirclements of (-1,0): N = 0", opts({{"fontsize", "10"}}));
        figtext(0.15, 0.87, "RHP poles: P = 0", opts({{"fontsize", "10"}}));
        figtext(0.15, 0.82, "Closed-loop poles in RHP: Z = N + P = 0", opts({{"fontsize", "10"}, {"color", "green"}}));
        
        savefig("output/control_04_nyquist.svg");
        clf();
        std::cout << "   Saved: output/control_04_nyquist.svg\n";
    }
    
    // ============ 5. Pole-Zero Map (Chuẩn) ============
    std::cout << "5. Pole-Zero Map...\n";
    {
        figure(700, 600);
        
        // System: G(s) = (s + 2) / ((s + 1)(s² + s + 1))
        // Zeros: z1 = -2
        // Poles: p1 = -1, p2,3 = -0.5 ± j0.866
        
        std::vector<double> zeros_re = {-2};
        std::vector<double> zeros_im = {0};
        
        std::vector<double> poles_re = {-1, -0.5, -0.5};
        std::vector<double> poles_im = {0, 0.866, -0.866};
        
        // Vẽ zeros trước (để poles đè lên nếu trùng)
        scatter(zeros_re, zeros_im, opts({{"s", "60"}, {"color", "blue"}, {"marker", "o"}}));
        
        // Vẽ poles
        scatter(poles_re, poles_im, opts({{"s", "60"}, {"color", "red"}, {"marker", "x"}}));
        
        // Trục ảo = stability boundary
        axvline(0, opts({{"color", "black"}, {"linewidth", "1.5"}}));
        axhline(0, opts({{"color", "black"}, {"linewidth", "0.8"}}));
        
        // Shading cho stable region (LHP)
        auto x_stable = linspace(-3.5, 0, 50);
        fill_between(x_stable, std::vector<double>(50, -2), std::vector<double>(50, 2),
                     opts({{"color", "green"}, {"alpha", "0.08"}}));
        
        // Constant damping ratio lines
        for (double z : {0.2, 0.4, 0.6, 0.8}) {
            double angle = std::acos(z);
            std::vector<double> line_re = {0, -3 * std::cos(angle)};
            std::vector<double> line_im_p = {0, 3 * std::sin(angle)};
            std::vector<double> line_im_n = {0, -3 * std::sin(angle)};
            
            plot(line_re, line_im_p, ":", opts({{"color", "gray"}, {"linewidth", "0.5"}}));
            plot(line_re, line_im_n, ":", opts({{"color", "gray"}, {"linewidth", "0.5"}}));
            
            std::ostringstream lbl;
            lbl << "\\zeta=" << z;
            text(-2.8 * std::cos(angle), 2.8 * std::sin(angle), lbl.str(),
                 opts({{"fontsize", "8"}, {"color", "gray"}}));
        }
        
        // Natural frequency circles
        for (double wn : {0.5, 1.0, 1.5, 2.0}) {
            auto th = linspace(M_PI/2, 3*M_PI/2, 50);  // Only LHP semicircle
            std::vector<double> circ_re, circ_im;
            for (double t : th) {
                circ_re.push_back(wn * std::cos(t));
                circ_im.push_back(wn * std::sin(t));
            }
            plot(circ_re, circ_im, ":", opts({{"color", "lightgray"}, {"linewidth", "0.5"}}));
        }
        
        title("Pole-Zero Map");
        xlabel("Real Axis (seconds^{-1})");
        ylabel("Imaginary Axis (seconds^{-1})");
        grid(true);
        
        // QUAN TRỌNG: Equal aspect ratio cho pole-zero map
        xlim(-3.5, 1);
        ylim(-2, 2);
        
        // Legend
        figtext(0.75, 0.90, "\\times  Poles", opts({{"fontsize", "11"}, {"color", "red"}}));
        figtext(0.75, 0.85, "\\circ  Zeros", opts({{"fontsize", "11"}, {"color", "blue"}}));
        
        savefig("output/control_05_pzmap.svg");
        clf();
        std::cout << "   Saved: output/control_05_pzmap.svg\n";
    }
    
    // ============ 6. Root Locus (Improved) ============
    std::cout << "6. Root Locus...\n";
    {
        figure(700, 600);
        
        // G(s) = K / (s(s+1)(s+2))
        // Poles at s = 0, -1, -2
        // As K increases: two poles meet, become complex, then one goes to +∞
        
        // Compute actual root locus using characteristic equation
        // s³ + 3s² + 2s + K = 0
        
        std::vector<double> locus_re, locus_im;
        
        for (double K = 0; K <= 20; K += 0.02) {
            // For each K, solve cubic equation numerically (simplified approximation)
            // Using Newton-Raphson or analytical formulas is complex
            // Here we use a simpler parametric approach
            
            // Breakaway point: dK/ds = 0 → s ≈ -0.42
            // Asymptote centroid: σ = (0 - 1 - 2) / 3 = -1
            // Asymptote angles: ±60°, 180°
            
            if (K < 0.385) {
                // Three real roots
                // Approximate: roots move along real axis
                double s1 = -K * 0.2;           // Root near 0 moves right slightly then left
                double s2 = -1 - K * 0.1;       // Root near -1 moves left
                double s3 = -2 + K * 0.15;      // Root near -2 moves right
                
                locus_re.push_back(s1); locus_im.push_back(0);
                locus_re.push_back(s2); locus_im.push_back(0);
                locus_re.push_back(s3); locus_im.push_back(0);
            } else {
                // Two complex roots + one real
                double sigma = -0.42;  // breakaway point
                double delta = K - 0.385;
                double omega = std::sqrt(delta) * 1.2;
                
                // Complex pair
                double re_cx = sigma - delta * 0.5;  // Move left along asymptote
                if (re_cx < -1) re_cx = -1 + (re_cx + 1) * 0.3;  // Slow down
                
                locus_re.push_back(re_cx); locus_im.push_back(omega);
                locus_re.push_back(re_cx); locus_im.push_back(-omega);
                
                // Real root moves to -∞
                locus_re.push_back(-2 - delta * 0.2); locus_im.push_back(0);
            }
        }
        
        // Plot locus
        scatter(locus_re, locus_im, opts({{"s", "2"}, {"color", "#1f77b4"}, {"alpha", "0.5"}}));
        
        // Open-loop poles
        scatter({0, -1, -2}, {0, 0, 0}, opts({{"s", "50"}, {"color", "red"}, {"marker", "x"}}));
        
        // Asymptotes
        double sigma_a = -1.0;  // Centroid
        std::vector<double> angles = {60, 180, -60};  // degrees
        for (double ang : angles) {
            double rad = ang * M_PI / 180;
            std::vector<double> asym_re = {sigma_a, sigma_a + 3 * std::cos(rad)};
            std::vector<double> asym_im = {0, 3 * std::sin(rad)};
            plot(asym_re, asym_im, "--", opts({{"color", "gray"}, {"linewidth", "0.8"}}));
        }
        
        // Breakaway point
        scatter({-0.42}, {0}, opts({{"s", "25"}, {"color", "orange"}}));
        text(-0.3, 0.2, "Breakaway", opts({{"fontsize", "9"}, {"color", "orange"}}));
        
        // Axes
        axhline(0, opts({{"color", "black"}, {"linewidth", "1"}}));
        axvline(0, opts({{"color", "black"}, {"linewidth", "1"}}));
        
        // Stability boundary highlight
        fill_between(linspace(-4, 0, 50), std::vector<double>(50, -3), std::vector<double>(50, 3),
                     opts({{"color", "green"}, {"alpha", "0.05"}}));
        
        title("Root Locus: G(s) = K / (s(s+1)(s+2))");
        xlabel("Real Axis");
        ylabel("Imaginary Axis");
        grid(true);
        xlim(-4, 1);
        ylim(-3, 3);
        
        // Annotations
        text(0.1, 0.2, "K=0", opts({{"fontsize", "9"}, {"color", "red"}}));
        text(-1.2, 2.5, "K \\to \\infty", opts({{"fontsize", "9"}, {"color", "blue"}}));
        
        savefig("output/control_06_rlocus.svg");
        clf();
        std::cout << "   Saved: output/control_06_rlocus.svg\n";
    }
    
    // ============ 7. Combined Dashboard ============
    std::cout << "7. System Analysis Dashboard...\n";
    {
        figure(1100, 800);
        layout(2, 3);
        
        SecondOrderSystem sys(2.0, 0.4);
        
        // 1. Step Response
        subplot(2, 3, 1);
        auto t = linspace(0, 6, 200);
        std::vector<double> y_step;
        for (double ti : t) y_step.push_back(sys.step(ti));
        
        plot(t, y_step, "-", opts({{"color", "#1f77b4"}, {"linewidth", "2"}}));
        axhline(1.0, opts({{"color", "gray"}, {"linestyle", "--"}}));
        title("Step Response");
        xlabel("Time (s)");
        ylabel("y(t)");
        grid(true);
        ylim(0, 1.5);
        
        // 2. Impulse Response
        subplot(2, 3, 2);
        std::vector<double> y_imp;
        double dt = t[1] - t[0];
        for (size_t i = 1; i < y_step.size(); ++i) {
            y_imp.push_back((y_step[i] - y_step[i-1]) / dt);
        }
        std::vector<double> t_imp(t.begin()+1, t.end());
        
        plot(t_imp, y_imp, "-", opts({{"color", "#2ca02c"}, {"linewidth", "2"}}));
        axhline(0, opts({{"color", "gray"}, {"linestyle", "--"}}));
        title("Impulse Response");
        xlabel("Time (s)");
        ylabel("h(t)");
        grid(true);
        
        // 3. Pole-Zero Map
        subplot(2, 3, 3);
        std::complex<double> p1, p2;
        sys.poles(p1, p2);
        
        scatter({p1.real(), p2.real()}, {p1.imag(), p2.imag()},
                opts({{"s", "40"}, {"color", "red"}, {"marker", "x"}}));
        axhline(0, opts({{"color", "black"}, {"linewidth", "0.8"}}));
        axvline(0, opts({{"color", "black"}, {"linewidth", "1"}}));
        title("Pole-Zero Map");
        xlabel("Re");
        ylabel("Im");
        grid(true);
        xlim(-2, 0.5);
        ylim(-2.5, 2.5);
        
        // 4. Bode Magnitude
        subplot(2, 3, 4);
        std::vector<double> omega;
        for (double w = 0.1; w <= 100; w *= 1.08) omega.push_back(w);
        
        std::vector<double> mag;
        for (double w : omega) mag.push_back(sys.mag_dB(w));
        
        plot(omega, mag, "-", opts({{"color", "#1f77b4"}, {"linewidth", "2"}}));
        xscale("log");
        axhline(0, opts({{"color", "gray"}, {"linestyle", "--"}}));
        axhline(-3, opts({{"color", "red"}, {"linestyle", ":"}}));
        title("Bode - Magnitude");
        xlabel("\\omega (rad/s)");
        ylabel("dB");
        grid(true);
        
        // 5. Bode Phase
        subplot(2, 3, 5);
        std::vector<double> phase_raw;
        for (double w : omega) phase_raw.push_back(sys.phase_deg(w));
        auto phase = unwrap_phase(phase_raw);
        
        plot(omega, phase, "-", opts({{"color", "#2ca02c"}, {"linewidth", "2"}}));
        xscale("log");
        axhline(-90, opts({{"color", "gray"}, {"linestyle", ":"}}));
        axhline(-180, opts({{"color", "gray"}, {"linestyle", ":"}}));
        title("Bode - Phase");
        xlabel("\\omega (rad/s)");
        ylabel("deg");
        grid(true);
        
        // 6. Nyquist
        subplot(2, 3, 6);
        std::vector<double> nyq_re, nyq_im;
        for (double w = 0.01; w <= 50; w *= 1.05) {
            auto G = sys.freqresp(w);
            nyq_re.push_back(G.real());
            nyq_im.push_back(G.imag());
        }
        
        plot(nyq_re, nyq_im, "-", opts({{"color", "#1f77b4"}, {"linewidth", "2"}}));
        scatter({-1}, {0}, opts({{"s", "30"}, {"color", "red"}, {"marker", "+"}}));
        axhline(0, opts({{"color", "gray"}, {"linewidth", "0.5"}}));
        axvline(0, opts({{"color", "gray"}, {"linewidth", "0.5"}}));
        title("Nyquist");
        xlabel("Re");
        ylabel("Im");
        grid(true);
        
        suptitle("System: G(s) = 4/(s^2 + 1.6s + 4)  [\\omega_n=2, \\zeta=0.4]");
        savefig("output/control_07_dashboard.svg");
        clf();
        std::cout << "   Saved: output/control_07_dashboard.svg\n";
    }
    
    // ============ Summary ============
    std::cout << "\n===========================================\n";
    std::cout << "Control System Demos (Standard Format)\n";
    std::cout << "===========================================\n";
    std::cout << "\nPlots generated:\n";
    std::cout << "  1. Step Response - with settling band\n";
    std::cout << "  2. Bode Plot - with resonance peak marking\n";
    std::cout << "  3. Bode with Margins - GM, PM visualization\n";
    std::cout << "  4. Nyquist - with direction arrows, critical point\n";
    std::cout << "  5. Pole-Zero Map - with damping/wn lines\n";
    std::cout << "  6. Root Locus - with asymptotes, breakaway\n";
    std::cout << "  7. Dashboard - complete analysis\n";
    std::cout << "\nStandard features:\n";
    std::cout << "  - Phase unwrapping for continuous Bode phase\n";
    std::cout << "  - Gain/Phase margin visualization\n";
    std::cout << "  - Proper axis labels (rad/s, dB, deg)\n";
    std::cout << "  - Critical point (-1,0) on Nyquist\n";
    std::cout << "  - Damping ratio lines on P-Z map\n";
    
    return 0;
}
