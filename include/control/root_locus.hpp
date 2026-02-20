/**
 * @file root_locus.hpp
 * @brief Root locus plotting functions
 */

#ifndef CPPPLOT_CONTROL_ROOT_LOCUS_HPP
#define CPPPLOT_CONTROL_ROOT_LOCUS_HPP

#include "transfer_function.hpp"
#include "polynomial.hpp"
#include "../pyplot.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace cppplot {
namespace control {

/**
 * @struct RLocusOptions
 * @brief Options for root locus plot
 */
struct RLocusOptions {
    double K_min = 0;           // Minimum gain
    double K_max = 0;           // Maximum gain (0 = auto)
    int num_K = 500;            // Number of gain values
    bool show_asymptotes = true;
    bool show_breakaway = true;
    bool grid = true;
    int marker_size = 2;
    std::string locus_color = "#1f77b4";
    std::string pole_color = "red";
    std::string zero_color = "blue";
};

/**
 * @brief Calculate root locus data
 * @param G Open-loop transfer function (without gain K)
 * @param K_values Vector of gain values
 * @return Vector of pole locations for each K
 */
inline std::vector<std::vector<std::complex<double>>> rlocus_data(
    const TransferFunction& G,
    const std::vector<double>& K_values
) {
    std::vector<std::vector<std::complex<double>>> loci;
    
    for (double K : K_values) {
        // Closed-loop: 1 + K*G(s) = 0
        // => den(s) + K*num(s) = 0
        Polynomial char_poly = G.den + G.num * K;
        auto roots = char_poly.roots();
        loci.push_back(roots);
    }
    
    return loci;
}

/**
 * @brief Find asymptote angles for root locus
 * @param n Number of poles
 * @param m Number of zeros
 * @return Vector of asymptote angles in radians
 */
inline std::vector<double> asymptote_angles(int n, int m) {
    std::vector<double> angles;
    int num_asymptotes = n - m;
    
    for (int k = 0; k < num_asymptotes; ++k) {
        double angle = M_PI * (2 * k + 1) / num_asymptotes;
        angles.push_back(angle);
    }
    
    return angles;
}

/**
 * @brief Find asymptote centroid (intersection point)
 * @param poles System poles
 * @param zeros System zeros
 * @return Centroid on real axis
 */
inline double asymptote_centroid(
    const std::vector<std::complex<double>>& poles,
    const std::vector<std::complex<double>>& zeros
) {
    double sum_poles = 0, sum_zeros = 0;
    for (const auto& p : poles) sum_poles += p.real();
    for (const auto& z : zeros) sum_zeros += z.real();
    
    int n = poles.size();
    int m = zeros.size();
    
    if (n == m) return 0;  // No asymptotes
    return (sum_poles - sum_zeros) / (n - m);
}

/**
 * @brief Plot root locus
 * @param G Open-loop transfer function
 * @param options Plot options
 */
inline void rlocus(const TransferFunction& G, const RLocusOptions& options = RLocusOptions()) {
    auto ol_poles = G.poles();
    auto ol_zeros = G.zeros();
    int n = ol_poles.size();
    int m = ol_zeros.size();
    
    // Auto-determine K range
    double K_max = options.K_max;
    if (K_max <= 0) {
        // Estimate based on pole locations
        double max_pole = 0;
        for (const auto& p : ol_poles) {
            max_pole = std::max(max_pole, std::abs(p));
        }
        K_max = std::pow(max_pole, n - m) * 100;
        if (K_max < 10) K_max = 100;
    }
    
    // Generate K values (log-spaced for better coverage)
    std::vector<double> K_values;
    K_values.push_back(0);
    double log_min = -3;
    double log_max = std::log10(K_max);
    for (int i = 0; i < options.num_K - 1; ++i) {
        double K = std::pow(10, log_min + i * (log_max - log_min) / (options.num_K - 2));
        K_values.push_back(K);
    }
    
    // Calculate loci
    auto loci = rlocus_data(G, K_values);
    
    // Convert to plottable format (organize by branch)
    std::vector<std::vector<double>> branches_re(n);
    std::vector<std::vector<double>> branches_im(n);
    
    for (size_t k = 0; k < loci.size(); ++k) {
        auto& roots = loci[k];
        
        // Sort roots to maintain branch continuity
        if (k > 0 && k < loci.size()) {
            auto& prev_roots = loci[k-1];
            std::vector<bool> used(n, false);
            std::vector<std::complex<double>> sorted_roots(n);
            
            for (int i = 0; i < n; ++i) {
                double min_dist = 1e10;
                int best_j = 0;
                for (int j = 0; j < n; ++j) {
                    if (!used[j]) {
                        double dist = std::abs(roots[j] - prev_roots[i]);
                        if (dist < min_dist) {
                            min_dist = dist;
                            best_j = j;
                        }
                    }
                }
                sorted_roots[i] = roots[best_j];
                used[best_j] = true;
            }
            roots = sorted_roots;
        }
        
        for (int i = 0; i < n; ++i) {
            branches_re[i].push_back(roots[i].real());
            branches_im[i].push_back(roots[i].imag());
        }
    }
    
    // Determine plot limits
    double max_re = 1, min_re = -1, max_im = 1;
    for (int i = 0; i < n; ++i) {
        for (size_t k = 0; k < branches_re[i].size(); ++k) {
            max_re = std::max(max_re, branches_re[i][k] + 0.5);
            min_re = std::min(min_re, branches_re[i][k] - 0.5);
            max_im = std::max(max_im, std::abs(branches_im[i][k]) + 0.5);
        }
    }
    
    figure(800, 700);
    
    // Plot root locus branches
    for (int i = 0; i < n; ++i) {
        scatter(branches_re[i], branches_im[i], opts({
            {"s", std::to_string(options.marker_size)},
            {"color", options.locus_color},
            {"alpha", "0.6"}
        }));
    }
    
    // Plot asymptotes
    if (options.show_asymptotes && n > m) {
        double centroid = asymptote_centroid(ol_poles, ol_zeros);
        auto angles = asymptote_angles(n, m);
        
        for (double angle : angles) {
            double length = std::max(std::abs(min_re - centroid), max_im) * 1.5;
            std::vector<double> asym_re = {centroid, centroid + length * std::cos(angle)};
            std::vector<double> asym_im = {0, length * std::sin(angle)};
            
            plot(asym_re, asym_im, "--", opts({
                {"color", "gray"},
                {"linewidth", "1"},
                {"alpha", "0.5"}
            }));
        }
        
        // Mark centroid
        scatter({centroid}, {0.0}, opts({
            {"s", "30"},
            {"color", "gray"},
            {"marker", "o"}
        }));
    }
    
    // Plot open-loop poles (×)
    std::vector<double> poles_re, poles_im;
    for (const auto& p : ol_poles) {
        poles_re.push_back(p.real());
        poles_im.push_back(p.imag());
    }
    scatter(poles_re, poles_im, opts({
        {"s", "80"},
        {"color", options.pole_color},
        {"marker", "x"}
    }));
    
    // Plot open-loop zeros (○)
    if (!ol_zeros.empty()) {
        std::vector<double> zeros_re, zeros_im;
        for (const auto& z : ol_zeros) {
            zeros_re.push_back(z.real());
            zeros_im.push_back(z.imag());
        }
        scatter(zeros_re, zeros_im, opts({
            {"s", "80"},
            {"color", options.zero_color},
            {"marker", "o"}
        }));
    }
    
    // Axes
    axhline(0, opts({{"color", "black"}, {"linewidth", "0.8"}}));
    axvline(0, opts({{"color", "black"}, {"linewidth", "0.8"}}));
    
    xlabel("Real Axis");
    ylabel("Imaginary Axis");
    title("Root Locus");
    
    if (options.grid) grid(true);
    
    xlim(min_re, max_re);
    ylim(-max_im, max_im);
}

/**
 * @brief Plot root locus with custom K range
 */
inline void rlocus(const TransferFunction& G, const std::vector<double>& K_values) {
    RLocusOptions opts;
    opts.num_K = K_values.size();
    
    auto loci = rlocus_data(G, K_values);
    
    // Similar plotting code as above...
    rlocus(G, opts);
}

/**
 * @brief Calculate gain K for a specific root locus point
 * @param G Transfer function
 * @param s Desired closed-loop pole location
 * @return Required gain K (negative if not on locus)
 */
inline double rlocfind(const TransferFunction& G, std::complex<double> s) {
    auto num_val = G.num(s);
    auto den_val = G.den(s);
    
    if (std::abs(num_val) < 1e-15) {
        return std::numeric_limits<double>::infinity();
    }
    
    double K = -std::abs(den_val / num_val);
    
    // Check if point is actually on locus (phase condition)
    double angle = std::arg(G.eval(s));
    double phase_error = std::fmod(std::abs(angle) + M_PI, 2 * M_PI) - M_PI;
    
    if (std::abs(phase_error) > 0.1) {  // Not on locus
        return -1;
    }
    
    return K;
}

} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_ROOT_LOCUS_HPP
