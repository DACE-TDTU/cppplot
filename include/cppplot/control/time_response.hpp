/**
 * @file time_response.hpp
 * @brief Time domain response functions: step, impulse, lsim
 */

#ifndef CPPPLOT_CONTROL_TIME_RESPONSE_HPP
#define CPPPLOT_CONTROL_TIME_RESPONSE_HPP

#include "../pyplot.hpp"
#include "analysis.hpp"
#include "transfer_function.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <vector>

namespace cppplot {
namespace control {

/**
 * @struct TimeResponse
 * @brief Named return type for time-domain response data
 * Replaces std::pair to avoid requiring C++17 structured bindings.
 */
struct TimeResponse {
  std::vector<double> t; ///< Time vector
  std::vector<double> y; ///< Response vector
};

/**
 * @struct TimeResponseOptions
 * @brief Options for time response plots
 */
struct TimeResponseOptions {
  double t_final = 0;        // Final time (0 = auto)
  int num_points = 500;      // Number of time points
  bool settling_band = true; // Show ±2% settling band
  bool grid = true;
  std::string color = "#1f77b4";
  double linewidth = 2.0;
  std::string label = "";
};

/**
 * @brief Calculate step response data
 * @param G Transfer function
 * @param t Time vector (empty for auto)
 * @return TimeResponse with .t and .y vectors
 */
inline TimeResponse step_data(const TransferFunction &G,
                              std::vector<double> t = {}) {
  // Auto-generate time vector if not provided
  if (t.empty()) {
    auto p = G.poles();
    double dominant_re = -1;
    for (const auto &pole : p) {
      if (pole.real() < 0 && pole.real() > dominant_re) {
        dominant_re = pole.real();
      }
    }
    double t_final = (dominant_re < -0.01) ? -5.0 / dominant_re : 10.0;
    if (t_final > 50)
      t_final = 50;

    t = linspace(0, t_final, 500);
  }

  std::vector<double> y;

  // Prefer exact step response via inverse Laplace when available
  bool used_exact = false;
  for (double ti : t) {
    double val = inverse_laplace_step(G, ti);
    if (!std::isnan(val)) {
      y.push_back(val);
      used_exact = true;
    } else {
      // Fallback: compute later using approximation path
      used_exact = false;
      break;
    }
  }

  if (!used_exact) {
    y.clear();
    // For 2nd order systems, use analytical formula
    if (G.order() == 2 && G.numZeros() == 0) {
      for (double ti : t) {
        y.push_back(G.stepResponse(ti));
      }
    } else {
      // Numerical approximation using dominant pole behavior
      double ss_value = G.dcgain();

      auto p = G.poles();
      std::complex<double> dominant = p[0];
      for (const auto &pole : p) {
        if (pole.real() > dominant.real()) {
          dominant = pole;
        }
      }

      double sigma = dominant.real();
      double omega = dominant.imag();

      for (double ti : t) {
        double yi;
        if (std::abs(omega) < 1e-10) {
          yi = ss_value * (1 - std::exp(sigma * ti));
        } else {
          double wd = std::abs(omega);
          double zeta = -sigma / std::sqrt(sigma * sigma + omega * omega);
          double phi = std::atan2(std::sqrt(1 - zeta * zeta), zeta);
          yi = ss_value * (1 - std::exp(sigma * ti) * std::sin(wd * ti + phi) /
                                   std::sqrt(1 - zeta * zeta));
        }
        y.push_back(yi);
      }
    }
  }

  TimeResponse result;
  result.t = t;
  result.y = y;
  return result;
}

/**
 * @brief Calculate step response data (convenience overload with final time)
 * @param G Transfer function
 * @param t_final Final simulation time
 * @param num_points Number of time points (default 500)
 * @return TimeResponse with .t and .y vectors
 */
inline TimeResponse step_data(const TransferFunction &G, double t_final,
                              int num_points = 500) {
  return step_data(G, linspace(0, t_final, num_points));
}

/**
 * @brief Plot step response
 * @param G Transfer function
 * @param options Plot options
 */
inline void step(const TransferFunction &G,
                 const TimeResponseOptions &options = TimeResponseOptions()) {
  std::vector<double> t;
  if (options.t_final > 0) {
    t = linspace(0, options.t_final, options.num_points);
  }

  auto result = step_data(G, t);
  std::vector<double> time_vec = result.t;
  std::vector<double> response = result.y;

  figure(800, 500);

  plot(time_vec, response, "-",
       opts({{"color", options.color},
             {"linewidth", std::to_string(options.linewidth)},
             {"label", options.label}}));

  // Steady-state value
  double ss = G.dcgain();
  if (std::isfinite(ss)) {
    axhline(
        ss,
        opts({{"color", "black"}, {"linestyle", "--"}, {"linewidth", "1"}}));

    // Settling band
    if (options.settling_band) {
      axhline(
          ss * 1.02,
          opts({{"color", "gray"}, {"linestyle", ":"}, {"linewidth", "0.5"}}));
      axhline(
          ss * 0.98,
          opts({{"color", "gray"}, {"linestyle", ":"}, {"linewidth", "0.5"}}));

      fill_between(time_vec, std::vector<double>(time_vec.size(), ss * 0.98),
                   std::vector<double>(time_vec.size(), ss * 1.02),
                   opts({{"color", "green"}, {"alpha", "0.1"}}));
    }
  }

  xlabel("Time (seconds)");
  ylabel("Amplitude");
  title("Step Response");

  if (options.grid)
    grid(true);
  if (!options.label.empty())
    legend(true);
}

/**
 * @brief Plot step response for multiple systems
 */
inline void step(const std::vector<TransferFunction> &systems,
                 const std::vector<std::string> &labels = {},
                 const TimeResponseOptions &options = TimeResponseOptions()) {
  std::vector<std::string> colors = {"#1f77b4", "#ff7f0e", "#2ca02c",
                                     "#d62728", "#9467bd", "#8c564b",
                                     "#e377c2", "#7f7f7f"};

  // Find common time range
  double t_max = 0;
  for (const auto &G : systems) {
    auto res = step_data(G);
    t_max = std::max(t_max, res.t.back());
  }

  auto time_vec = linspace(0, t_max, options.num_points);

  figure(800, 500);

  for (size_t i = 0; i < systems.size(); ++i) {
    auto res = step_data(systems[i], time_vec);
    std::vector<double> t = res.t;
    std::vector<double> y = res.y;

    std::string color = colors[i % colors.size()];
    std::string label = (i < labels.size()) ? labels[i] : "";

    plot(t, y, "-",
         opts({{"color", color}, {"linewidth", "2"}, {"label", label}}));
  }

  xlabel("Time (seconds)");
  ylabel("Amplitude");
  title("Step Response");

  if (!labels.empty())
    legend(true);
  if (options.grid)
    grid(true);
}

/**
 * @brief Calculate impulse response data
 */
inline TimeResponse impulse_data(const TransferFunction &G,
                                 std::vector<double> t = {}) {
  // Build time vector if needed
  if (t.empty()) {
    auto p = G.poles();
    double dominant_re = -1;
    for (const auto &pole : p) {
      if (pole.real() < 0 && pole.real() > dominant_re) {
        dominant_re = pole.real();
      }
    }
    double t_final = (dominant_re < -0.01) ? -5.0 / dominant_re : 10.0;
    t = linspace(0, t_final, 500);
  }

  // Try exact impulse via inverse Laplace; fallback to derivative of step
  std::vector<double> y;
  bool used_exact = true;
  for (double ti : t) {
    double val = inverse_laplace_impulse(G, ti);
    if (!std::isnan(val)) {
      y.push_back(val);
    } else {
      used_exact = false;
      break;
    }
  }

  if (!used_exact) {
    // Fallback: numerical derivative of step response
    auto step_res = step_data(G, t);
    std::vector<double> y_step = step_res.y;
    y.clear();
    y.push_back(0);
    for (size_t i = 1; i < t.size(); ++i) {
      double dt = t[i] - t[i - 1];
      double dy = (y_step[i] - y_step[i - 1]) / dt;
      y.push_back(dy);
    }
  }

  TimeResponse result;
  result.t = t;
  result.y = y;
  return result;
}

/**
 * @brief Calculate impulse response data (convenience overload with final time)
 * @param G Transfer function
 * @param t_final Final simulation time
 * @param num_points Number of time points (default 500)
 * @return Pair of (time, response) vectors
 */
inline TimeResponse impulse_data(const TransferFunction &G, double t_final,
                                 int num_points = 500) {
  return impulse_data(G, linspace(0, t_final, num_points));
}

/**
 * @brief Plot impulse response
 */
inline void
impulse(const TransferFunction &G,
        const TimeResponseOptions &options = TimeResponseOptions()) {
  std::vector<double> t;
  if (options.t_final > 0) {
    t = linspace(0, options.t_final, options.num_points);
  }

  auto imp_res = impulse_data(G, t);
  std::vector<double> time_vec = imp_res.t;
  std::vector<double> response = imp_res.y;

  figure(800, 500);

  plot(time_vec, response, "-",
       opts({{"color", options.color},
             {"linewidth", std::to_string(options.linewidth)},
             {"label", options.label}}));

  axhline(0,
          opts({{"color", "black"}, {"linestyle", "-"}, {"linewidth", "0.5"}}));

  xlabel("Time (seconds)");
  ylabel("Amplitude");
  title("Impulse Response");

  if (options.grid)
    grid(true);
  if (!options.label.empty())
    legend(true);
}

/**
 * @brief Plot initial condition response (for state-space, simplified)
 */
inline void
initial(const TransferFunction &G, double x0 = 1.0,
        const TimeResponseOptions &options = TimeResponseOptions()) {
  // Simplified: for 2nd order, x(t) = x0 * e^(σt) * cos(ωdt + φ)
  std::vector<double> t;
  if (options.t_final > 0) {
    t = linspace(0, options.t_final, options.num_points);
  } else {
    auto p = G.poles();
    double dominant_re = -1;
    for (const auto &pole : p) {
      if (pole.real() < 0 && pole.real() > dominant_re) {
        dominant_re = pole.real();
      }
    }
    double t_final = (dominant_re < -0.01) ? -5.0 / dominant_re : 10.0;
    t = linspace(0, t_final, options.num_points);
  }

  std::vector<double> y;
  auto p = G.poles();

  if (!p.empty()) {
    double sigma = p[0].real();
    double omega = p[0].imag();

    for (double ti : t) {
      double yi;
      if (std::abs(omega) < 1e-10) {
        yi = x0 * std::exp(sigma * ti);
      } else {
        yi = x0 * std::exp(sigma * ti) * std::cos(omega * ti);
      }
      y.push_back(yi);
    }
  }

  figure(800, 500);

  plot(t, y, "-",
       opts({{"color", options.color},
             {"linewidth", std::to_string(options.linewidth)}}));

  axhline(0,
          opts({{"color", "black"}, {"linestyle", "-"}, {"linewidth", "0.5"}}));

  xlabel("Time (seconds)");
  ylabel("Amplitude");
  title("Initial Condition Response");

  if (options.grid)
    grid(true);
}

} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_TIME_RESPONSE_HPP
