/**
 * @file bode.hpp
 * @brief Bode plot functions following Python Control standards
 */

#ifndef CPPPLOT_CONTROL_BODE_HPP
#define CPPPLOT_CONTROL_BODE_HPP

// Define M_PI for Windows compatibility
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "transfer_function.hpp"
#include "analysis.hpp"
#include "../pyplot.hpp"
#include <vector>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace cppplot {
namespace control {

/**
 * @struct BodeOptions
 * @brief Options for Bode plot
 */
struct BodeOptions {
    double omega_min = 0;       // Min frequency (0 = auto)
    double omega_max = 0;       // Max frequency (0 = auto)
    int num_points = 200;       // Number of frequency points
    bool dB = true;             // Magnitude in dB
    bool deg = true;            // Phase in degrees
    bool Hz = false;            // Frequency in Hz (default rad/s)
    bool margins = false;       // Show gain/phase margins
    bool grid = true;           // Show grid
    std::string color = "";     // Line color (empty = auto)
    std::string linestyle = "-";
    double linewidth = 2.0;
    std::string label = "";
};

/**
 * @brief Generate frequency vector for Bode plot
 */
inline std::vector<double> bode_frequencies(
    const TransferFunction& G,
    double omega_min = 0,
    double omega_max = 0,
    int num_points = 200
) {
    // Auto-determine frequency range from poles and zeros
    if (omega_min <= 0 || omega_max <= 0) {
        auto p = G.poles();
        auto z = G.zeros();
        
        double w_min = 0.1, w_max = 10.0;
        
        for (const auto& pole : p) {
            double w = std::abs(pole);
            if (w > 1e-3) {
                w_min = std::min(w_min, w / 10);
                w_max = std::max(w_max, w * 10);
            }
        }
        for (const auto& zero : z) {
            double w = std::abs(zero);
            if (w > 1e-3) {
                w_min = std::min(w_min, w / 10);
                w_max = std::max(w_max, w * 10);
            }
        }
        
        if (omega_min <= 0) omega_min = w_min;
        if (omega_max <= 0) omega_max = w_max;
    }
    
    // Generate log-spaced frequencies
    std::vector<double> omega;
    double log_min = std::log10(omega_min);
    double log_max = std::log10(omega_max);
    double step = (log_max - log_min) / (num_points - 1);
    
    for (int i = 0; i < num_points; ++i) {
        omega.push_back(std::pow(10, log_min + i * step));
    }
    
    return omega;
}

/**
 * @brief Unwrap phase to be continuous
 */
inline std::vector<double> unwrap_phase_deg(const std::vector<double>& phase) {
    std::vector<double> unwrapped = phase;
    for (size_t i = 1; i < unwrapped.size(); ++i) {
        while (unwrapped[i] - unwrapped[i-1] > 180) unwrapped[i] -= 360;
        while (unwrapped[i] - unwrapped[i-1] < -180) unwrapped[i] += 360;
    }
    return unwrapped;
}

/**
 * @brief Plot Bode diagram for a transfer function
 * @param G Transfer function
 * @param options Bode plot options
 */
inline void bode(const TransferFunction& G, const BodeOptions& options = BodeOptions()) {
    auto omega = bode_frequencies(G, options.omega_min, options.omega_max, options.num_points);
    
    // Calculate frequency response
    std::vector<double> mag, phase_raw;
    for (double w : omega) {
        if (options.dB) {
            mag.push_back(G.mag_dB(w));
        } else {
            mag.push_back(G.mag(w));
        }
        
        if (options.deg) {
            phase_raw.push_back(G.phase_deg(w));
        } else {
            phase_raw.push_back(G.phase(w));
        }
    }
    
    // Unwrap phase
    auto phase = options.deg ? unwrap_phase_deg(phase_raw) : phase_raw;
    
    // Convert to Hz if requested
    std::vector<double> freq = omega;
    if (options.Hz) {
        for (double& f : freq) f /= (2 * M_PI);
    }
    
    // Setup figure
    figure(800, 600);
    layout(2, 1);
    
    // Determine color
    std::string color = options.color.empty() ? "#1f77b4" : options.color;
    
    // === Magnitude Plot ===
    subplot(2, 1, 1);
    
    std::ostringstream style_mag;
    style_mag << "{\"color\": \"" << color << "\", "
              << "\"linestyle\": \"" << options.linestyle << "\", "
              << "\"linewidth\": \"" << options.linewidth << "\"";
    if (!options.label.empty()) {
        style_mag << ", \"label\": \"" << options.label << "\"";
    }
    style_mag << "}";
    
    plot(freq, mag, options.linestyle, opts({
        {"color", color},
        {"linewidth", std::to_string(options.linewidth)},
        {"label", options.label}
    }));
    
    xscale("log");
    
    // Reference lines
    if (options.dB) {
        axhline(0, opts({{"color", "black"}, {"linestyle", "-"}, {"linewidth", "0.5"}}));
        axhline(-3, opts({{"color", "gray"}, {"linestyle", ":"}, {"linewidth", "0.8"}}));
        ylabel("Magnitude (dB)");
    } else {
        axhline(1, opts({{"color", "gray"}, {"linestyle", ":"}, {"linewidth", "0.8"}}));
        ylabel("Magnitude");
    }
    
    if (options.grid) grid(true);
    
    // === Phase Plot ===
    subplot(2, 1, 2);
    
    plot(freq, phase, options.linestyle, opts({
        {"color", color},
        {"linewidth", std::to_string(options.linewidth)}
    }));
    
    xscale("log");
    
    // Phase reference lines
    if (options.deg) {
        axhline(0, opts({{"color", "gray"}, {"linestyle", ":"}, {"linewidth", "0.5"}}));
        axhline(-90, opts({{"color", "gray"}, {"linestyle", ":"}, {"linewidth", "0.5"}}));
        axhline(-180, opts({{"color", "gray"}, {"linestyle", ":"}, {"linewidth", "0.5"}}));
        axhline(-270, opts({{"color", "gray"}, {"linestyle", ":"}, {"linewidth", "0.5"}}));
        ylabel("Phase (deg)");
    } else {
        ylabel("Phase (rad)");
    }
    
    xlabel(options.Hz ? "Frequency (Hz)" : "Frequency (rad/s)");
    
    if (options.grid) grid(true);
    
    // Add margins if requested
    if (options.margins) {
        auto m = margin(G);
        
        // Mark gain crossover
        if (m.Wgc > 0) {
            subplot(2, 1, 1);
            axvline(m.Wgc, opts({{"color", "green"}, {"linestyle", "--"}, {"linewidth", "1"}}));
            scatter({m.Wgc}, {0.0}, opts({{"s", "25"}, {"color", "green"}}));
            
            subplot(2, 1, 2);
            axvline(m.Wgc, opts({{"color", "green"}, {"linestyle", "--"}, {"linewidth", "1"}}));
            double phase_gc = -180 + m.Pm;
            scatter({m.Wgc}, {phase_gc}, opts({{"s", "25"}, {"color", "green"}}));
            
            std::ostringstream pm_label;
            pm_label << "PM = " << std::fixed << std::setprecision(1) << m.Pm << "°";
            text(m.Wgc * 1.2, phase_gc + 10, pm_label.str(), opts({{"fontsize", "9"}, {"color", "green"}}));
        }
        
        // Mark phase crossover
        if (m.Wpc > 0) {
            subplot(2, 1, 1);
            axvline(m.Wpc, opts({{"color", "red"}, {"linestyle", "--"}, {"linewidth", "1"}}));
            double mag_pc = -m.Gm_dB;
            scatter({m.Wpc}, {mag_pc}, opts({{"s", "25"}, {"color", "red"}}));
            
            std::ostringstream gm_label;
            gm_label << "GM = " << std::fixed << std::setprecision(1) << m.Gm_dB << " dB";
            text(m.Wpc * 1.2, mag_pc + 3, gm_label.str(), opts({{"fontsize", "9"}, {"color", "red"}}));
            
            subplot(2, 1, 2);
            axvline(m.Wpc, opts({{"color", "red"}, {"linestyle", "--"}, {"linewidth", "1"}}));
            scatter({m.Wpc}, {-180.0}, opts({{"s", "25"}, {"color", "red"}}));
        }
    }
    
    suptitle("Bode Diagram");
}

/**
 * @brief Plot Bode for multiple systems
 */
inline void bode(
    const std::vector<TransferFunction>& systems,
    const std::vector<std::string>& labels = {},
    const BodeOptions& options = BodeOptions()
) {
    std::vector<std::string> colors = {
        "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", 
        "#9467bd", "#8c564b", "#e377c2", "#7f7f7f"
    };
    
    // Find common frequency range
    double w_min = 1e10, w_max = 1e-10;
    for (const auto& G : systems) {
        auto omega = bode_frequencies(G);
        w_min = std::min(w_min, omega.front());
        w_max = std::max(w_max, omega.back());
    }
    
    auto omega = bode_frequencies(systems[0], w_min, w_max, options.num_points);
    
    figure(800, 600);
    layout(2, 1);
    
    for (size_t i = 0; i < systems.size(); ++i) {
        std::vector<double> mag, phase_raw;
        for (double w : omega) {
            mag.push_back(systems[i].mag_dB(w));
            phase_raw.push_back(systems[i].phase_deg(w));
        }
        auto phase = unwrap_phase_deg(phase_raw);
        
        std::string color = colors[i % colors.size()];
        std::string label = (i < labels.size()) ? labels[i] : "";
        
        subplot(2, 1, 1);
        plot(omega, mag, "-", opts({
            {"color", color},
            {"linewidth", "2"},
            {"label", label}
        }));
        
        subplot(2, 1, 2);
        plot(omega, phase, "-", opts({{"color", color}, {"linewidth", "2"}}));
    }
    
    subplot(2, 1, 1);
    xscale("log");
    ylabel("Magnitude (dB)");
    axhline(0, opts({{"color", "black"}, {"linestyle", "-"}, {"linewidth", "0.5"}}));
    if (!labels.empty()) legend(true);
    if (options.grid) grid(true);
    
    subplot(2, 1, 2);
    xscale("log");
    xlabel("Frequency (rad/s)");
    ylabel("Phase (deg)");
    axhline(-180, opts({{"color", "gray"}, {"linestyle", ":"}, {"linewidth", "0.5"}}));
    if (options.grid) grid(true);
    
    suptitle("Bode Diagram");
}

/**
 * @brief Convenience function: bode with custom frequency vector
 */
inline void bode(const TransferFunction& G, const std::vector<double>& omega) {
    BodeOptions opts;
    opts.omega_min = omega.front();
    opts.omega_max = omega.back();
    opts.num_points = omega.size();
    bode(G, opts);
}

} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_BODE_HPP
