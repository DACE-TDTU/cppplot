/**
 * @file nichols.hpp
 * @brief Nichols chart plotting functions
 * 
 * Nichols chart plots gain (dB) vs phase (deg) with M-circles and N-circles
 */

#ifndef CPPPLOT_CONTROL_NICHOLS_HPP
#define CPPPLOT_CONTROL_NICHOLS_HPP

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
 * @struct NicholsOptions
 * @brief Options for Nichols chart
 */
struct NicholsOptions {
    double omega_min = 0;       // Min frequency (0 = auto)
    double omega_max = 0;       // Max frequency (0 = auto)
    int num_points = 200;       // Number of frequency points
    bool grid = true;           // Show grid
    bool show_m_circles = true; // Show M-circles (constant CL magnitude)
    bool show_n_circles = false; // Show N-circles (constant CL phase)
    bool show_margins = false;  // Show gain/phase margins
    std::string color = "#1f77b4";
    double linewidth = 2.0;
    std::string label = "";
};

/**
 * @brief Generate M-circle (constant closed-loop magnitude) for Nichols chart
 * M = |T| = |G/(1+G)| = constant
 * 
 * In Nichols coordinates (phase, magnitude dB):
 * The M-circle transforms to a specific curve
 */
inline std::pair<std::vector<double>, std::vector<double>> nichols_m_circle(double M_dB) {
    std::vector<double> phase, mag;
    
    double M = std::pow(10, M_dB / 20.0);  // Convert to linear
    
    // M-circle in Nichols chart
    // For |G/(1+G)| = M, we parameterize G = re^(jθ) and solve
    // The locus in (phase, |G|_dB) coordinates forms a closed curve
    
    if (std::abs(M - 1.0) < 0.001) {
        // M = 0 dB (unity) - vertical line at -180°
        for (double g_dB = -40; g_dB <= 40; g_dB += 1) {
            phase.push_back(-180);
            mag.push_back(g_dB);
        }
    } else {
        // General M-circle
        double M2 = M * M;
        
        for (int i = 0; i <= 360; ++i) {
            double theta = i * M_PI / 180.0;  // Parameter angle
            
            // Radius of M-circle in G-plane
            double r = M / std::abs(M2 - 1);
            double center = M2 / (M2 - 1);
            
            // Point on M-circle in G-plane
            double G_re = center + r * std::cos(theta);
            double G_im = r * std::sin(theta);
            
            // Convert to Nichols coordinates
            double G_mag = std::sqrt(G_re * G_re + G_im * G_im);
            double G_phase = std::atan2(G_im, G_re) * 180.0 / M_PI;
            
            if (G_mag > 0.001 && G_mag < 1000) {
                phase.push_back(G_phase);
                mag.push_back(20 * std::log10(G_mag));
            }
        }
    }
    
    return std::make_pair(phase, mag);
}

/**
 * @brief Generate N-circle (constant closed-loop phase) for Nichols chart
 * N = arg(G/(1+G)) = constant
 */
inline std::pair<std::vector<double>, std::vector<double>> nichols_n_circle(double N_deg) {
    std::vector<double> phase, mag;
    
    double N = N_deg * M_PI / 180.0;  // Convert to radians
    
    // N-circle: arg(G/(1+G)) = N
    // Center at (-0.5, 0.5*cot(N)), radius = 0.5/|sin(N)|
    
    if (std::abs(std::sin(N)) < 0.01) {
        // N ≈ 0 or 180 - horizontal line
        return std::make_pair(phase, mag);
    }
    
    double center_re = -0.5;
    double center_im = 0.5 / std::tan(N);
    double radius = std::abs(0.5 / std::sin(N));
    
    for (int i = 0; i <= 360; ++i) {
        double theta = i * M_PI / 180.0;
        
        double G_re = center_re + radius * std::cos(theta);
        double G_im = center_im + radius * std::sin(theta);
        
        double G_mag = std::sqrt(G_re * G_re + G_im * G_im);
        double G_phase = std::atan2(G_im, G_re) * 180.0 / M_PI;
        
        if (G_mag > 0.001 && G_mag < 1000 && G_phase >= -360 && G_phase <= 0) {
            phase.push_back(G_phase);
            mag.push_back(20 * std::log10(G_mag));
        }
    }
    
    return std::make_pair(phase, mag);
}

/**
 * @brief Plot Nichols chart
 * @param G Open-loop transfer function
 * @param options Plot options
 */
inline void nichols(const TransferFunction& G, const NicholsOptions& options = NicholsOptions()) {
    // Generate frequency points
    auto omega = bode_frequencies(G, options.omega_min, options.omega_max, options.num_points);
    
    // Calculate frequency response
    std::vector<double> mag_dB, phase_deg;
    for (double w : omega) {
        mag_dB.push_back(G.mag_dB(w));
        phase_deg.push_back(G.phase_deg(w));
    }
    
    // Unwrap phase
    for (size_t i = 1; i < phase_deg.size(); ++i) {
        while (phase_deg[i] - phase_deg[i-1] > 180) phase_deg[i] -= 360;
        while (phase_deg[i] - phase_deg[i-1] < -180) phase_deg[i] += 360;
    }
    
    figure(800, 700);
    
    // Draw M-circles (constant closed-loop magnitude)
    if (options.show_m_circles) {
        std::vector<double> M_values = {-12, -6, -3, -1, 0, 0.25, 0.5, 1, 3, 6, 12};
        for (double M_dB : M_values) {
            auto m_result = nichols_m_circle(M_dB);
            std::vector<double> m_phase = m_result.first;
            std::vector<double> m_mag = m_result.second;
            if (!m_phase.empty()) {
                plot(m_phase, m_mag, "-", opts({
                    {"color", "lightgray"},
                    {"linewidth", "0.5"},
                    {"alpha", "0.7"}
                }));
            }
        }
    }
    
    // Draw N-circles (constant closed-loop phase)
    if (options.show_n_circles) {
        std::vector<double> N_values = {-210, -180, -150, -120, -90, -60, -30, -15, -5, 5, 15, 30};
        for (double N_deg : N_values) {
            auto n_result = nichols_n_circle(N_deg);
            std::vector<double> n_phase = n_result.first;
            std::vector<double> n_mag = n_result.second;
            if (!n_phase.empty()) {
                plot(n_phase, n_mag, "-", opts({
                    {"color", "lightblue"},
                    {"linewidth", "0.5"},
                    {"alpha", "0.5"}
                }));
            }
        }
    }
    
    // Critical point (-180°, 0 dB)
    scatter({-180}, {0}, opts({{"s", "40"}, {"color", "red"}, {"marker", "+"}}));
    text(-175, 2, "(-180, 0dB)", opts({{"fontsize", "9"}, {"color", "red"}}));
    
    // Plot open-loop frequency response
    plot(phase_deg, mag_dB, "-", opts({
        {"color", options.color},
        {"linewidth", std::to_string(options.linewidth)},
        {"label", options.label.empty() ? "G(jw)" : options.label}
    }));
    
    // Direction arrows
    size_t n = phase_deg.size();
    std::vector<size_t> arrow_idx = {n/6, n/3, n/2, 2*n/3};
    for (size_t idx : arrow_idx) {
        if (idx < n) {
            scatter({phase_deg[idx]}, {mag_dB[idx]}, opts({
                {"s", "15"},
                {"color", options.color},
                {"marker", ">"}
            }));
        }
    }
    
    // Start/end markers
    scatter({phase_deg.front()}, {mag_dB.front()}, opts({
        {"s", "30"}, {"color", "green"}
    }));
    text(phase_deg.front() + 5, mag_dB.front() + 2, "w->0", 
         opts({{"fontsize", "9"}, {"color", "green"}}));
    
    // Add margins if requested
    if (options.show_margins) {
        auto m = margin(G);
        
        // Phase margin at gain crossover
        if (m.Wgc > 0) {
            double phase_gc = -180 + m.Pm;
            scatter({phase_gc}, {0.0}, opts({{"s", "30"}, {"color", "green"}}));
            
            // Draw PM arc
            std::vector<double> pm_x = {phase_gc, -180};
            std::vector<double> pm_y = {0, 0};
            plot(pm_x, pm_y, "-", opts({{"color", "green"}, {"linewidth", "2"}}));
            
            std::ostringstream pm_label;
            pm_label << "PM=" << std::fixed << std::setprecision(1) << m.Pm << " deg";
            text((phase_gc - 180) / 2, 2, pm_label.str(), opts({{"fontsize", "9"}, {"color", "green"}}));
        }
        
        // Gain margin at phase crossover
        if (m.Wpc > 0 && m.Gm_dB < 100) {
            double mag_pc = -m.Gm_dB;
            scatter({-180.0}, {mag_pc}, opts({{"s", "30"}, {"color", "red"}}));
            
            // Draw GM line
            std::vector<double> gm_x = {-180, -180};
            std::vector<double> gm_y = {mag_pc, 0};
            plot(gm_x, gm_y, "-", opts({{"color", "red"}, {"linewidth", "2"}}));
            
            std::ostringstream gm_label;
            gm_label << "GM=" << std::fixed << std::setprecision(1) << m.Gm_dB << "dB";
            text(-175, mag_pc / 2, gm_label.str(), opts({{"fontsize", "9"}, {"color", "red"}}));
        }
    }
    
    xlabel("Open-Loop Phase (deg)");
    ylabel("Open-Loop Gain (dB)");
    title("Nichols Chart");
    
    if (options.grid) grid(true);
    legend(true);
    
    // Set axis limits
    xlim(-360, 0);
}

/**
 * @brief Plot Nichols chart for multiple systems
 */
inline void nichols(
    const std::vector<TransferFunction>& systems,
    const std::vector<std::string>& labels,
    const NicholsOptions& options = NicholsOptions()
) {
    std::vector<std::string> colors = {
        "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", 
        "#9467bd", "#8c564b", "#e377c2", "#7f7f7f"
    };
    
    figure(800, 700);
    
    // Draw M-circles
    if (options.show_m_circles) {
        std::vector<double> M_values = {-12, -6, -3, -1, 0, 1, 3, 6, 12};
        for (double M_dB : M_values) {
            auto m_result = nichols_m_circle(M_dB);
            std::vector<double> m_phase = m_result.first;
            std::vector<double> m_mag = m_result.second;
            if (!m_phase.empty()) {
                plot(m_phase, m_mag, "-", opts({
                    {"color", "lightgray"},
                    {"linewidth", "0.5"}
                }));
            }
        }
    }
    
    // Critical point
    scatter({-180}, {0}, opts({{"s", "40"}, {"color", "red"}, {"marker", "+"}}));
    
    // Plot each system
    for (size_t i = 0; i < systems.size(); ++i) {
        auto omega = bode_frequencies(systems[i], options.omega_min, options.omega_max, options.num_points);
        
        std::vector<double> mag_dB, phase_deg;
        for (double w : omega) {
            mag_dB.push_back(systems[i].mag_dB(w));
            phase_deg.push_back(systems[i].phase_deg(w));
        }
        
        // Unwrap phase
        for (size_t j = 1; j < phase_deg.size(); ++j) {
            while (phase_deg[j] - phase_deg[j-1] > 180) phase_deg[j] -= 360;
            while (phase_deg[j] - phase_deg[j-1] < -180) phase_deg[j] += 360;
        }
        
        std::string color = colors[i % colors.size()];
        std::string label = (i < labels.size()) ? labels[i] : "";
        
        plot(phase_deg, mag_dB, "-", opts({
            {"color", color},
            {"linewidth", "2"},
            {"label", label}
        }));
    }
    
    xlabel("Open-Loop Phase (deg)");
    ylabel("Open-Loop Gain (dB)");
    title("Nichols Chart");
    
    if (!labels.empty()) legend(true);
    if (options.grid) grid(true);
    
    xlim(-360, 0);
}

} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_NICHOLS_HPP
