/**
 * @file nyquist.hpp
 * @brief Nyquist plot functions following Python Control standards
 */

#ifndef CPPPLOT_CONTROL_NYQUIST_HPP
#define CPPPLOT_CONTROL_NYQUIST_HPP

// Define M_PI for Windows compatibility
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "../pyplot.hpp"
#include "transfer_function.hpp"
#include <cmath>
#include <sstream>
#include <vector>

namespace cppplot {
namespace control {

/**
 * @struct NyquistOptions
 * @brief Options for Nyquist plot
 */
struct NyquistOptions {
  double omega_min = 0.001;   // Min frequency
  double omega_max = 1000;    // Max frequency
  int num_points = 500;       // Number of points
  bool arrows = true;         // Show direction arrows
  bool mirror = true;         // Show negative frequency (conjugate)
  bool unit_circle = false;   // Show unit circle
  bool critical_point = true; // Mark (-1, 0)
  bool grid = true;
  std::string color = "#1f77b4";
  double linewidth = 2.0;
  std::string label = "";
};

/**
 * @brief Plot Nyquist diagram
 * @param G Open-loop transfer function
 * @param options Plot options
 */
inline void nyquist(const TransferFunction &G,
                    const NyquistOptions &options = NyquistOptions()) {
  // Generate frequency points
  std::vector<double> omega;
  double log_min = std::log10(options.omega_min);
  double log_max = std::log10(options.omega_max);
  double step = (log_max - log_min) / (options.num_points - 1);

  for (int i = 0; i < options.num_points; ++i) {
    omega.push_back(std::pow(10, log_min + i * step));
  }

  // Calculate frequency response
  std::vector<double> re_pos, im_pos;
  std::vector<double> re_neg, im_neg;

  double max_extent = 0.0;

  for (double w : omega) {
    auto H = G.freqresp(w);
    double r = H.real();
    double i = H.imag();

    // Guard against NaN or Infinity from pure integrators at w->0
    if (!std::isfinite(r) || !std::isfinite(i))
      continue;

    // Skip excessively large points to prevent breaking auto-scaling
    if (std::abs(r) > 100.0 || std::abs(i) > 100.0)
      continue;

    re_pos.push_back(r);
    im_pos.push_back(i);
    max_extent = std::max({max_extent, std::abs(r), std::abs(i)});

    if (options.mirror) {
      re_neg.push_back(r);
      im_neg.push_back(-i); // Conjugate
    }
  }

  // Create figure
  figure(700, 700);

  // Plot positive frequency (ω > 0)
  plot(
      re_pos, im_pos, "-",
      opts({{"color", options.color},
            {"linewidth", std::to_string(options.linewidth)},
            {"label", options.label.empty() ? "\\omega > 0" : options.label}}));

  // Plot negative frequency (ω < 0) - dashed
  if (options.mirror) {
    plot(re_neg, im_neg, "--",
         opts({{"color", options.color},
               {"linewidth", std::to_string(options.linewidth * 0.7)},
               {"alpha", "0.6"},
               {"label", "\\omega < 0"}}));
  }

  // Direction arrows
  if (options.arrows) {
    size_t n = re_pos.size();
    std::vector<size_t> arrow_idx = {n / 6, n / 3, n / 2, 2 * n / 3, 5 * n / 6};

    for (size_t idx : arrow_idx) {
      if (idx > 0 && idx < n - 1) {
        // Calculate direction
        double dx = re_pos[idx + 1] - re_pos[idx - 1];
        double dy = im_pos[idx + 1] - im_pos[idx - 1];
        double mag = std::sqrt(dx * dx + dy * dy);

        if (mag > 1e-10) {
          scatter(
              {re_pos[idx]}, {im_pos[idx]},
              opts({{"s", "12"}, {"color", options.color}, {"marker", ">"}}));
        }
      }
    }
  }

  // Critical point (-1, 0)
  if (options.critical_point) {
    scatter({-1}, {0}, opts({{"s", "40"}, {"color", "red"}, {"marker", "+"}}));
    text(-0.85, 0.1, "(-1, j0)", opts({{"fontsize", "10"}, {"color", "red"}}));
  }

  // Unit circle
  if (options.unit_circle) {
    std::vector<double> circle_x, circle_y;
    for (int i = 0; i <= 100; ++i) {
      double theta = 2 * M_PI * i / 100;
      circle_x.push_back(std::cos(theta));
      circle_y.push_back(std::sin(theta));
    }
    plot(circle_x, circle_y, "--",
         opts({{"color", "gray"}, {"linewidth", "0.8"}, {"alpha", "0.5"}}));
  }

  // Axes
  axhline(0, opts({{"color", "black"}, {"linewidth", "0.5"}}));
  axvline(0, opts({{"color", "black"}, {"linewidth", "0.5"}}));

  // Start marker
  scatter({re_pos[0]}, {im_pos[0]}, opts({{"s", "25"}, {"color", "green"}}));
  text(re_pos[0] + 0.05, im_pos[0] + 0.05, "\\omega \\to 0",
       opts({{"fontsize", "9"}, {"color", "green"}}));

  xlabel("Real Axis");
  ylabel("Imaginary Axis");
  title("Nyquist Diagram");

  if (options.grid)
    grid(true);
  legend(true);

  // Auto-scale to keep critical point in view and clip integrator infinities
  double limit = std::max(1.5, max_extent * 1.1);
  limit = std::min(limit, 20.0); // Hardcap to prevent excessive zoom out
  xlim(-limit, limit);
  ylim(-limit, limit);
}

/**
 * @brief Plot Nyquist for multiple systems
 */
inline void nyquist(const std::vector<TransferFunction> &systems,
                    const std::vector<std::string> &labels = {},
                    const NyquistOptions &options = NyquistOptions()) {
  std::vector<std::string> colors = {"#1f77b4", "#ff7f0e", "#2ca02c",
                                     "#d62728", "#9467bd", "#8c564b",
                                     "#e377c2", "#7f7f7f"};

  figure(700, 700);

  double overall_max_extent = 0.0;

  for (size_t i = 0; i < systems.size(); ++i) {
    std::vector<double> omega;
    double log_min = std::log10(options.omega_min);
    double log_max = std::log10(options.omega_max);
    double step = (log_max - log_min) / (options.num_points - 1);

    for (int j = 0; j < options.num_points; ++j) {
      omega.push_back(std::pow(10, log_min + j * step));
    }

    std::vector<double> re, im;
    for (double w : omega) {
      auto H = systems[i].freqresp(w);
      double r = H.real();
      double im_val = H.imag();

      if (!std::isfinite(r) || !std::isfinite(im_val))
        continue;
      if (std::abs(r) > 100.0 || std::abs(im_val) > 100.0)
        continue;

      re.push_back(r);
      im.push_back(im_val);
      overall_max_extent =
          std::max({overall_max_extent, std::abs(r), std::abs(im_val)});
    }

    std::string color = colors[i % colors.size()];
    std::string label = (i < labels.size()) ? labels[i] : "";

    plot(re, im, "-",
         opts({{"color", color}, {"linewidth", "2"}, {"label", label}}));
  }

  // Critical point
  scatter({-1}, {0}, opts({{"s", "40"}, {"color", "red"}, {"marker", "+"}}));

  axhline(0, opts({{"color", "black"}, {"linewidth", "0.5"}}));
  axvline(0, opts({{"color", "black"}, {"linewidth", "0.5"}}));

  xlabel("Real Axis");
  ylabel("Imaginary Axis");
  title("Nyquist Diagram");

  if (!labels.empty())
    legend(true);
  if (options.grid)
    grid(true);

  double limit = std::max(1.5, overall_max_extent * 1.1);
  limit = std::min(limit, 20.0);
  xlim(-limit, limit);
  ylim(-limit, limit);
}

} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_NYQUIST_HPP
