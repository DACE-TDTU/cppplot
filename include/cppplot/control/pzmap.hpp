/**
 * @file pzmap.hpp
 * @brief Pole-Zero map plotting functions
 */

#ifndef CPPPLOT_CONTROL_PZMAP_HPP
#define CPPPLOT_CONTROL_PZMAP_HPP

#include "transfer_function.hpp"
#include "../pyplot.hpp"
#include <vector>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <iostream>

namespace cppplot {
namespace control {

/**
 * @struct PZMapOptions
 * @brief Options for pole-zero map
 */
struct PZMapOptions {
    bool damping_lines = true;      // Show constant damping ratio lines
    bool natural_freq_lines = true; // Show constant ωn circles
    std::vector<double> zeta_lines = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
    bool grid = true;
    bool stability_region = true;   // Shade LHP
    int marker_size = 60;
    std::string pole_color = "red";
    std::string zero_color = "blue";
};

/**
 * @brief Plot pole-zero map
 * @param G Transfer function
 * @param options Plot options
 */
inline void pzmap(const TransferFunction& G, const PZMapOptions& options = PZMapOptions()) {
    auto p = G.poles();
    auto z = G.zeros();
    
    // Separate into real and imaginary parts
    std::vector<double> poles_re, poles_im;
    std::vector<double> zeros_re, zeros_im;
    
    for (const auto& pole : p) {
        poles_re.push_back(pole.real());
        poles_im.push_back(pole.imag());
    }
    
    for (const auto& zero : z) {
        zeros_re.push_back(zero.real());
        zeros_im.push_back(zero.imag());
    }
    
    // Determine axis limits
    double max_re = 1, min_re = -1;
    double max_im = 1;
    
    for (const auto& pole : p) {
        max_re = std::max(max_re, std::abs(pole.real()) * 1.2);
        min_re = std::min(min_re, -std::abs(pole.real()) * 1.2);
        max_im = std::max(max_im, std::abs(pole.imag()) * 1.2);
    }
    for (const auto& zero : z) {
        max_re = std::max(max_re, std::abs(zero.real()) * 1.2);
        min_re = std::min(min_re, -std::abs(zero.real()) * 1.2);
        max_im = std::max(max_im, std::abs(zero.imag()) * 1.2);
    }
    
    figure(700, 700);
    
    // Shade stability region (LHP)
    if (options.stability_region) {
        std::vector<double> lhp_x = {min_re, 0, 0, min_re};
        std::vector<double> lhp_y = {-max_im, -max_im, max_im, max_im};
        fill_between(
            std::vector<double>{min_re, 0},
            std::vector<double>{-max_im, -max_im},
            std::vector<double>{max_im, max_im},
            opts({{"color", "green"}, {"alpha", "0.05"}})
        );
    }
    
    // Draw constant damping ratio lines
    if (options.damping_lines) {
        for (double zeta : options.zeta_lines) {
            // Line from origin: Im/Re = -sqrt(1-zeta^2)/zeta
            double angle = std::acos(zeta);
            std::vector<double> line_re = {0, min_re};
            std::vector<double> line_im_pos = {0, -min_re * std::tan(angle)};
            std::vector<double> line_im_neg = {0, min_re * std::tan(angle)};
            
            plot(line_re, line_im_pos, "-", opts({
                {"color", "gray"},
                {"linewidth", "0.5"},
                {"alpha", "0.5"}
            }));
            plot(line_re, line_im_neg, "-", opts({
                {"color", "gray"},
                {"linewidth", "0.5"},
                {"alpha", "0.5"}
            }));
            
            // Label
            double label_x = min_re * 0.3;
            double label_y = -label_x * std::tan(angle);
            std::ostringstream lbl;
            lbl << std::fixed << std::setprecision(1) << zeta;
            text(label_x - 0.05, label_y, lbl.str(), opts({{"fontsize", "7"}, {"color", "gray"}}));
        }
    }
    
    // Draw constant natural frequency circles
    if (options.natural_freq_lines) {
        double max_wn = std::sqrt(max_re * max_re + max_im * max_im);
        std::vector<double> wn_values;
        
        // Generate reasonable wn values
        double wn = 0.5;
        while (wn < max_wn) {
            wn_values.push_back(wn);
            wn *= 2;
        }
        
        for (double wn : wn_values) {
            std::vector<double> arc_x, arc_y;
            // Only draw in LHP
            for (int i = 90; i <= 270; ++i) {
                double theta = i * M_PI / 180;
                arc_x.push_back(wn * std::cos(theta));
                arc_y.push_back(wn * std::sin(theta));
            }
            plot(arc_x, arc_y, "-", opts({
                {"color", "lightgray"},
                {"linewidth", "0.5"},
                {"alpha", "0.5"}
            }));
        }
    }
    
    // Axes
    axhline(0, opts({{"color", "black"}, {"linewidth", "0.8"}}));
    axvline(0, opts({{"color", "black"}, {"linewidth", "0.8"}}));
    
    // Plot zeros first (so poles appear on top)
    if (!zeros_re.empty()) {
        scatter(zeros_re, zeros_im, opts({
            {"s", std::to_string(options.marker_size)},
            {"color", options.zero_color},
            {"marker", "o"}
        }));
    }
    
    // Plot poles
    if (!poles_re.empty()) {
        scatter(poles_re, poles_im, opts({
            {"s", std::to_string(options.marker_size)},
            {"color", options.pole_color},
            {"marker", "x"}
        }));
    }
    
    xlabel("Real Axis (\\sigma)");
    ylabel("Imaginary Axis (j\\omega)");
    title("Pole-Zero Map");
    
    if (options.grid) grid(true);
    
    xlim(min_re, max_re);
    ylim(-max_im, max_im);
}

/**
 * @brief Plot pole-zero map for multiple systems
 */
inline void pzmap(
    const std::vector<TransferFunction>& systems,
    const std::vector<std::string>& labels = {},
    const PZMapOptions& options = PZMapOptions()
) {
    std::vector<std::string> colors = {
        "red", "blue", "green", "orange", "purple", "brown"
    };
    
    // Find overall limits
    double max_re = 1, min_re = -1, max_im = 1;
    
    for (const auto& G : systems) {
        for (const auto& pole : G.poles()) {
            max_re = std::max(max_re, std::abs(pole.real()) * 1.2);
            min_re = std::min(min_re, -std::abs(pole.real()) * 1.2);
            max_im = std::max(max_im, std::abs(pole.imag()) * 1.2);
        }
    }
    
    figure(700, 700);
    
    for (size_t i = 0; i < systems.size(); ++i) {
        auto p = systems[i].poles();
        auto z = systems[i].zeros();
        
        std::vector<double> poles_re, poles_im, zeros_re, zeros_im;
        
        for (const auto& pole : p) {
            poles_re.push_back(pole.real());
            poles_im.push_back(pole.imag());
        }
        for (const auto& zero : z) {
            zeros_re.push_back(zero.real());
            zeros_im.push_back(zero.imag());
        }
        
        std::string color = colors[i % colors.size()];
        std::string label = (i < labels.size()) ? labels[i] : "";
        
        if (!poles_re.empty()) {
            scatter(poles_re, poles_im, opts({
                {"s", std::to_string(options.marker_size)},
                {"color", color},
                {"marker", "x"},
                {"label", label + " poles"}
            }));
        }
        if (!zeros_re.empty()) {
            scatter(zeros_re, zeros_im, opts({
                {"s", std::to_string(options.marker_size)},
                {"color", color},
                {"marker", "o"},
                {"label", label + " zeros"}
            }));
        }
    }
    
    axhline(0, opts({{"color", "black"}, {"linewidth", "0.8"}}));
    axvline(0, opts({{"color", "black"}, {"linewidth", "0.8"}}));
    
    xlabel("Real Axis");
    ylabel("Imaginary Axis");
    title("Pole-Zero Map");
    
    if (!labels.empty()) legend(true);
    if (options.grid) grid(true);
}

/**
 * @brief Print pole-zero information to console
 */
inline void pzmap_info(const TransferFunction& G) {
    std::cout << "=== Pole-Zero Analysis ===" << std::endl;
    
    auto p = G.poles();
    auto z = G.zeros();
    
    std::cout << "\nPoles (" << p.size() << "):" << std::endl;
    for (size_t i = 0; i < p.size(); ++i) {
        double wn = std::abs(p[i]);
        double zeta = (wn > 1e-10) ? -p[i].real() / wn : 1.0;
        
        std::cout << "  p" << (i+1) << " = " 
                  << std::fixed << std::setprecision(4)
                  << p[i].real();
        if (std::abs(p[i].imag()) > 1e-10) {
            std::cout << (p[i].imag() >= 0 ? " + " : " - ")
                      << std::abs(p[i].imag()) << "j";
        }
        std::cout << "  |  ωn = " << wn << ", ζ = " << zeta << std::endl;
    }
    
    std::cout << "\nZeros (" << z.size() << "):" << std::endl;
    for (size_t i = 0; i < z.size(); ++i) {
        std::cout << "  z" << (i+1) << " = "
                  << std::fixed << std::setprecision(4)
                  << z[i].real();
        if (std::abs(z[i].imag()) > 1e-10) {
            std::cout << (z[i].imag() >= 0 ? " + " : " - ")
                      << std::abs(z[i].imag()) << "j";
        }
        std::cout << std::endl;
    }
    
    std::cout << "\nStability: " << (G.isStable() ? "STABLE" : "UNSTABLE") << std::endl;
}

} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_PZMAP_HPP
