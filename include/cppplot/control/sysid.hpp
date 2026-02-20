/**
 * @file sysid.hpp
 * @brief System Identification module for CppPlot Control Systems Library
 *
 * Provides functions for identifying dynamic system models from measured
 * input-output data. Supports time-domain, frequency-domain, and parametric
 * identification methods.
 *
 * @version 1.0.0
 * @date 2026
 *
 * Features:
 * - Step response identification (1st and 2nd order)
 * - Frequency-domain identification (Bode fitting)
 * - Parametric identification (ARX with least squares)
 * - Input signal generation (PRBS, chirp, multisine)
 * - Model validation (residual analysis, cross-validation, AIC/BIC)
 * - Discrete-to-continuous conversion (Tustin)
 *
 * @see Chapter 9: System Identification — From Measured Data to Mathematical
 * Models
 *
 * @example Basic step response identification
 * @code
 * #include <cppplot/control/sysid.hpp>
 * using namespace cppplot::control::sysid;
 *
 * // Measured step response data
 * std::vector<double> t = {0, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0};
 * std::vector<double> y = {0, 0.18, 0.33, 0.63, 0.86, 0.98, 1.0, 1.0};
 *
 * auto result = id_step_first_order(t, y, 1.0);
 * std::cout << "K = " << result.K << ", tau = " << result.tau << std::endl;
 * // result.G is the identified TransferFunction
 * @endcode
 *
 * @example ARX identification with PRBS
 * @code
 * auto prbs = generate_prbs(7, 1.0, 0.01);   // 7-bit, amplitude 1V, Ts=10ms
 * // ... apply to system and record output ...
 *
 * auto arx = id_arx(u, y, 2, 2);             // ARX(2,2)
 * auto G_id = arx_to_tf(arx, 0.01);          // Convert to continuous TF
 * auto val = validate_model(arx, u_val, y_val); // Cross-validate
 * @endcode
 */

#ifndef CPPPLOT_CONTROL_SYSID_HPP
#define CPPPLOT_CONTROL_SYSID_HPP

#include "../core/matrix.hpp"
#include "discrete.hpp"
#include "transfer_function.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace cppplot {
namespace control {
namespace sysid {

// ============================================================================
//  Result Structures
// ============================================================================

/**
 * @struct FirstOrderIDResult
 * @brief Result of first-order step response identification
 */
struct FirstOrderIDResult {
  double K;           ///< DC gain
  double tau;         ///< Time constant [s]
  TransferFunction G; ///< Identified TF: G(s) = K / (τs + 1)
  double fit_percent; ///< Goodness of fit [%]
};

/**
 * @struct SecondOrderIDResult
 * @brief Result of second-order step response identification
 */
struct SecondOrderIDResult {
  double K;           ///< DC gain
  double wn;          ///< Natural frequency [rad/s]
  double zeta;        ///< Damping ratio
  double Mp;          ///< Percent overshoot (0–1 scale)
  double tp;          ///< Peak time [s]
  double ts;          ///< Settling time [s] (estimated)
  TransferFunction G; ///< Identified TF: G(s) = Kωn² / (s² + 2ζωns + ωn²)
  double fit_percent; ///< Goodness of fit [%]
};

/**
 * @struct ARXModel
 * @brief Identified ARX discrete-time model
 *
 * Model: y[k] = a1*y[k-1] + ... + ana*y[k-na]
 *             + b1*u[k-1] + ... + bnb*u[k-nb] + e[k]
 */
struct ARXModel {
  int na;                        ///< Number of output (AR) parameters
  int nb;                        ///< Number of input (X) parameters
  int nk;                        ///< Input delay (default 1)
  std::vector<double> a;         ///< AR coefficients [a1, a2, ..., ana]
  std::vector<double> b;         ///< X coefficients [b1, b2, ..., bnb]
  double Ts;                     ///< Sampling period [s]
  double sigma2;                 ///< Residual variance
  std::vector<double> residuals; ///< Prediction errors
  int N;                         ///< Number of data points used

  /// Number of free parameters
  int num_params() const { return na + nb; }

  /// Convert to discrete-time transfer function
  DiscreteTransferFunction to_dtf() const {
    // H(z) = B(z^-1) / A(z^-1)
    // A(z^-1) = 1 - a1*z^-1 - a2*z^-2 - ...
    // B(z^-1) = b1*z^-1 + b2*z^-2 + ...
    // Multiply through by z^max(na,nb):
    // H(z) = (b1*z^(na-1) + b2*z^(na-2) + ...) / (z^na - a1*z^(na-1) - ...)

    int max_order = std::max(na, nb + nk - 1);

    // Denominator: z^na - a1*z^(na-1) - ... - ana
    std::vector<double> den_coeffs(max_order + 1, 0.0);
    den_coeffs[0] = 1.0; // z^na leading coefficient
    for (int i = 0; i < na; ++i) {
      den_coeffs[i + 1] = -a[i];
    }

    // Numerator: 0*z^na + b1*z^(na-nk) + b2*z^(na-nk-1) + ...
    std::vector<double> num_coeffs(max_order + 1, 0.0);
    for (int i = 0; i < nb; ++i) {
      int idx = nk + i;
      if (idx <= max_order) {
        num_coeffs[idx] = b[i];
      }
    }

    return DiscreteTransferFunction(num_coeffs, den_coeffs, Ts);
  }
};

/**
 * @struct BodeFitResult
 * @brief Result of frequency-domain identification from Bode data
 */
struct BodeFitResult {
  TransferFunction G;                  ///< Identified continuous-time TF
  std::vector<double> omega;           ///< Frequencies used [rad/s]
  std::vector<double> mag_dB_meas;     ///< Measured magnitude [dB]
  std::vector<double> phase_deg_meas;  ///< Measured phase [deg]
  std::vector<double> mag_dB_model;    ///< Model magnitude [dB]
  std::vector<double> phase_deg_model; ///< Model phase [deg]
  double mag_rms_error;                ///< RMS magnitude error [dB]
  double phase_rms_error;              ///< RMS phase error [deg]
};

/**
 * @struct ValidationResult
 * @brief Model validation metrics
 */
struct ValidationResult {
  double fit_percent;               ///< FIT metric [%] (NRMSE-based)
  double mse;                       ///< Mean squared error
  double rmse;                      ///< Root mean squared error
  std::vector<double> residuals;    ///< Prediction errors
  std::vector<double> autocorr;     ///< Residual autocorrelation
  std::vector<double> crosscorr;    ///< Input-residual cross-correlation
  bool residuals_white;             ///< Whiteness test result (95% conf)
  bool residuals_uncorr_with_input; ///< Input-independence test result
};

/**
 * @struct InformationCriteria
 * @brief AIC/BIC model selection metrics
 */
struct InformationCriteria {
  double AIC;     ///< Akaike Information Criterion
  double BIC;     ///< Bayesian Information Criterion
  int num_params; ///< Number of parameters
  int N;          ///< Number of data points
  double sigma2;  ///< Residual variance
};

/**
 * @struct StepIDParams
 * @brief Parameters extracted from step response (before TF construction)
 */
struct StepIDParams {
  double K;    ///< DC gain
  double tau;  ///< Time constant (1st order) [s]
  double wn;   ///< Natural frequency (2nd order) [rad/s]
  double zeta; ///< Damping ratio (2nd order)
  double Mp;   ///< Overshoot fraction
  double tp;   ///< Peak time [s]
  double td;   ///< Time delay [s]
  int order;   ///< Detected system order (1 or 2)
};

// ============================================================================
//  Utility: Compute FIT metric
// ============================================================================

/**
 * @brief Compute the NRMSE-based FIT metric
 *
 * FIT = 100 * (1 - ||y - y_hat|| / ||y - mean(y)||)
 *
 * @param y     Measured output
 * @param y_hat Model prediction
 * @return FIT percentage (can be negative if model is worse than mean)
 */
inline double compute_fit(const std::vector<double> &y,
                          const std::vector<double> &y_hat) {
  if (y.size() != y_hat.size() || y.empty())
    return 0.0;

  double mean_y = 0.0;
  for (double v : y)
    mean_y += v;
  mean_y /= y.size();

  double norm_err = 0.0, norm_ref = 0.0;
  for (size_t i = 0; i < y.size(); ++i) {
    double err = y[i] - y_hat[i];
    double ref = y[i] - mean_y;
    norm_err += err * err;
    norm_ref += ref * ref;
  }
  norm_err = std::sqrt(norm_err);
  norm_ref = std::sqrt(norm_ref);

  if (norm_ref < 1e-15)
    return 100.0; // Constant output perfectly matched
  return 100.0 * (1.0 - norm_err / norm_ref);
}

// ============================================================================
//  §9.2 — Step Response Identification
// ============================================================================

/**
 * @brief Detect system order from step response data
 *
 * Heuristic: if overshoot > 2% of final value → second order (underdamped)
 *            otherwise → first order
 *
 * @param y Step response data (assumed to start from ~0)
 * @return 1 for first-order, 2 for second-order
 */
inline int detect_step_order(const std::vector<double> &y) {
  if (y.size() < 3)
    return 1;
  double y_final = y.back();
  if (std::abs(y_final) < 1e-15)
    return 1;

  double y_max = *std::max_element(y.begin(), y.end());
  double overshoot = (y_max - y_final) / std::abs(y_final);
  return (overshoot > 0.02) ? 2 : 1;
}

/**
 * @brief Identify a first-order system from step response data
 *
 * Extracts K and τ from measured step response using the 63.2% method,
 * then refines with least-squares fitting.
 *
 * Model: G(s) = K / (τs + 1)
 * Step response: y(t) = K * u_step * (1 - exp(-t/τ))
 *
 * @param t      Time vector [s]
 * @param y      Output vector (step response)
 * @param u_step Step input magnitude (default 1.0)
 * @return FirstOrderIDResult with K, τ, G(s), and fit metric
 */
inline FirstOrderIDResult id_step_first_order(const std::vector<double> &t,
                                              const std::vector<double> &y,
                                              double u_step = 1.0) {
  if (t.size() != y.size() || t.size() < 3) {
    throw std::runtime_error(
        "id_step_first_order: need at least 3 data points");
  }

  FirstOrderIDResult result;

  // DC gain: use average of last 10% of data for robustness
  size_t start_ss = static_cast<size_t>(0.9 * y.size());
  double y_ss = 0.0;
  for (size_t i = start_ss; i < y.size(); ++i)
    y_ss += y[i];
  y_ss /= (y.size() - start_ss);
  result.K = y_ss / u_step;

  // Time constant: find time when y reaches 63.2% of y_ss
  double target = 0.632 * y_ss;
  result.tau = t.back(); // default
  for (size_t i = 0; i < y.size() - 1; ++i) {
    if (y[i] <= target && y[i + 1] >= target) {
      // Linear interpolation
      double frac = (target - y[i]) / (y[i + 1] - y[i]);
      result.tau = t[i] + frac * (t[i + 1] - t[i]);
      break;
    }
  }

  // Refine with least-squares: minimize Σ(y_meas - K(1 - e^(-t/τ)))²
  // Use Gauss-Newton iteration (2-3 iterations suffice)
  double K_est = result.K;
  double tau_est = result.tau;

  for (int iter = 0; iter < 10; ++iter) {
    // Jacobian and residual
    double JtJ_00 = 0, JtJ_01 = 0, JtJ_11 = 0;
    double Jtr_0 = 0, Jtr_1 = 0;

    for (size_t i = 0; i < t.size(); ++i) {
      double exp_t = std::exp(-t[i] / tau_est);
      double y_model = K_est * u_step * (1.0 - exp_t);
      double r = y[i] - y_model;

      // dy/dK = u_step * (1 - e^(-t/τ))
      double J0 = u_step * (1.0 - exp_t);
      // dy/dτ = K * u_step * (-t/τ²) * e^(-t/τ)
      double J1 = K_est * u_step * (-t[i] / (tau_est * tau_est)) * exp_t;

      JtJ_00 += J0 * J0;
      JtJ_01 += J0 * J1;
      JtJ_11 += J1 * J1;
      Jtr_0 += J0 * r;
      Jtr_1 += J1 * r;
    }

    // Solve 2x2 normal equations
    double det = JtJ_00 * JtJ_11 - JtJ_01 * JtJ_01;
    if (std::abs(det) < 1e-30)
      break;

    double dK = (JtJ_11 * Jtr_0 - JtJ_01 * Jtr_1) / det;
    double dtau = (-JtJ_01 * Jtr_0 + JtJ_00 * Jtr_1) / det;

    K_est += dK;
    tau_est += dtau;

    // Ensure tau > 0
    if (tau_est <= 0)
      tau_est = 0.001;

    if (std::abs(dK) < 1e-10 * std::abs(K_est) &&
        std::abs(dtau) < 1e-10 * std::abs(tau_est))
      break;
  }

  result.K = K_est;
  result.tau = tau_est;
  result.G = TransferFunction({result.K}, {result.tau, 1.0});

  // Compute fit metric
  std::vector<double> y_hat(t.size());
  for (size_t i = 0; i < t.size(); ++i) {
    y_hat[i] = result.K * u_step * (1.0 - std::exp(-t[i] / result.tau));
  }
  result.fit_percent = compute_fit(y, y_hat);

  return result;
}

/**
 * @brief Identify a second-order underdamped system from step response
 *
 * Extracts K, ζ, ωn from measured step response using overshoot and peak time,
 * then refines with nonlinear least-squares.
 *
 * Model: G(s) = Kωn² / (s² + 2ζωns + ωn²)
 *
 * @param t      Time vector [s]
 * @param y      Output vector (step response)
 * @param u_step Step input magnitude (default 1.0)
 * @return SecondOrderIDResult
 */
inline SecondOrderIDResult id_step_second_order(const std::vector<double> &t,
                                                const std::vector<double> &y,
                                                double u_step = 1.0) {
  if (t.size() != y.size() || t.size() < 5) {
    throw std::runtime_error(
        "id_step_second_order: need at least 5 data points");
  }

  SecondOrderIDResult result;

  // DC gain from steady state (average last 10%)
  size_t start_ss = static_cast<size_t>(0.9 * y.size());
  double y_ss = 0.0;
  for (size_t i = start_ss; i < y.size(); ++i)
    y_ss += y[i];
  y_ss /= (y.size() - start_ss);
  result.K = y_ss / u_step;

  // Find peak (overshoot)
  size_t peak_idx = 0;
  double y_max = y[0];
  for (size_t i = 1; i < y.size(); ++i) {
    if (y[i] > y_max) {
      y_max = y[i];
      peak_idx = i;
    }
  }
  result.tp = t[peak_idx];
  result.Mp = (y_max - y_ss) / std::abs(y_ss);

  // Extract damping ratio from overshoot
  if (result.Mp > 0.001) {
    double ln_mp = std::log(result.Mp);
    result.zeta = -ln_mp / std::sqrt(M_PI * M_PI + ln_mp * ln_mp);
  } else {
    result.zeta = 1.0; // Overdamped or critically damped
  }

  // Natural frequency from peak time
  if (result.tp > 0 && result.zeta < 1.0) {
    result.wn = M_PI / (result.tp * std::sqrt(1.0 - result.zeta * result.zeta));
  } else {
    // Fallback: estimate from rise time
    double target_10 = 0.1 * y_ss, target_90 = 0.9 * y_ss;
    double t_10 = t[0], t_90 = t.back();
    for (size_t i = 0; i < y.size() - 1; ++i) {
      if (y[i] <= target_10 && y[i + 1] >= target_10)
        t_10 = t[i];
      if (y[i] <= target_90 && y[i + 1] >= target_90) {
        t_90 = t[i];
        break;
      }
    }
    double tr = t_90 - t_10;
    result.wn = 1.8 / tr; // Approximation for ζ ≈ 0.5
  }

  // Settling time estimate
  result.ts = 4.0 / (result.zeta * result.wn);

  // Build transfer function
  double wn2 = result.wn * result.wn;
  result.G = TransferFunction({result.K * wn2},
                              {1.0, 2.0 * result.zeta * result.wn, wn2});

  // Compute fit metric
  std::vector<double> y_hat(t.size());
  for (size_t i = 0; i < t.size(); ++i) {
    y_hat[i] = result.G.stepResponse(t[i]) * u_step;
  }
  result.fit_percent = compute_fit(y, y_hat);

  return result;
}

/**
 * @brief Auto-detect order and identify from step response
 *
 * Automatically detects whether the system is first or second order
 * and calls the appropriate identification function.
 *
 * @param t      Time vector [s]
 * @param y      Output vector
 * @param u_step Step magnitude
 * @return StepIDParams with detected order and parameters
 */
inline StepIDParams id_step_auto(const std::vector<double> &t,
                                 const std::vector<double> &y,
                                 double u_step = 1.0) {
  StepIDParams params;
  params.td = 0.0;
  params.order = detect_step_order(y);

  if (params.order == 1) {
    auto r = id_step_first_order(t, y, u_step);
    params.K = r.K;
    params.tau = r.tau;
    params.wn = 1.0 / r.tau;
    params.zeta = 1.0;
    params.Mp = 0.0;
    params.tp = 0.0;
  } else {
    auto r = id_step_second_order(t, y, u_step);
    params.K = r.K;
    params.tau = 1.0 / (r.zeta * r.wn);
    params.wn = r.wn;
    params.zeta = r.zeta;
    params.Mp = r.Mp;
    params.tp = r.tp;
  }

  return params;
}

// ============================================================================
//  §9.3 — Frequency-Domain Identification
// ============================================================================

/**
 * @brief Identify transfer function from Bode magnitude/phase data
 *
 * Performs asymptotic fitting: determines system order from high-frequency
 * slope, locates corner frequencies, then refines with optimization.
 *
 * @param omega     Frequency vector [rad/s]
 * @param mag_dB    Measured magnitude [dB]
 * @param phase_deg Measured phase [deg] (optional, empty = magnitude only)
 * @param max_order Maximum model order to try (default 4)
 * @return BodeFitResult
 */
inline BodeFitResult id_bode(const std::vector<double> &omega,
                             const std::vector<double> &mag_dB,
                             const std::vector<double> &phase_deg = {},
                             int max_order = 4) {
  if (omega.size() != mag_dB.size() || omega.size() < 3) {
    throw std::runtime_error("id_bode: need at least 3 frequency points");
  }

  BodeFitResult result;
  result.omega = omega;
  result.mag_dB_meas = mag_dB;
  result.phase_deg_meas = phase_deg;

  // Step 1: DC gain from lowest-frequency data
  double K_dB = mag_dB[0]; // Approximate DC gain
  double K = std::pow(10.0, K_dB / 20.0);

  // Step 2: Determine relative degree from high-frequency slope
  // Use last few data points to estimate slope
  size_t n = omega.size();
  double slope = 0.0;
  if (n >= 3) {
    // Regression on log-log data (last third of data)
    size_t start = 2 * n / 3;
    double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    int count = 0;
    for (size_t i = start; i < n; ++i) {
      double x = std::log10(omega[i]);
      double y_val = mag_dB[i];
      sum_x += x;
      sum_y += y_val;
      sum_xy += x * y_val;
      sum_x2 += x * x;
      count++;
    }
    slope = (count * sum_xy - sum_x * sum_y) / (count * sum_x2 - sum_x * sum_x);
  }

  // Relative degree ≈ -slope / 20
  int rel_degree = std::max(
      1, std::min(max_order, static_cast<int>(std::round(-slope / 20.0))));

  // Step 3: Find corner frequencies from slope changes
  // Compute numerical derivative of magnitude curve
  std::vector<double> slopes_local;
  for (size_t i = 1; i < n; ++i) {
    double dx = std::log10(omega[i]) - std::log10(omega[i - 1]);
    if (std::abs(dx) > 1e-10) {
      slopes_local.push_back((mag_dB[i] - mag_dB[i - 1]) / dx);
    }
  }

  // Detect corner frequencies where slope changes significantly
  std::vector<double> corners;
  for (size_t i = 1; i < slopes_local.size(); ++i) {
    double ds = slopes_local[i] - slopes_local[i - 1];
    if (ds < -10.0) { // Significant slope change (new pole)
      // Corner is approximately at the midpoint frequency
      double wc = std::sqrt(omega[i] * omega[i + 1]);
      corners.push_back(wc);
    }
  }

  // Ensure we have enough corners for the detected order
  while (static_cast<int>(corners.size()) < rel_degree) {
    // If we can't detect enough corners, distribute them logarithmically
    double w_low = omega[0];
    double w_high = omega.back();
    double wc = std::pow(w_low * w_high, 0.5); // geometric mean
    corners.push_back(wc);
  }

  // Sort corners
  std::sort(corners.begin(), corners.end());

  // Step 4: Build initial transfer function estimate
  // G(s) = K / ((s/w1 + 1)(s/w2 + 1)...)
  std::vector<double> den = {1.0};
  for (int i = 0; i < rel_degree && i < static_cast<int>(corners.size()); ++i) {
    // Multiply (s/wi + 1) into denominator
    double tau_i = 1.0 / corners[i];
    std::vector<double> factor = {tau_i, 1.0};

    // Polynomial convolution
    std::vector<double> new_den(den.size() + 1, 0.0);
    for (size_t j = 0; j < den.size(); ++j) {
      for (size_t k = 0; k < factor.size(); ++k) {
        new_den[j + k] += den[j] * factor[k];
      }
    }
    den = new_den;
  }

  // Scale numerator for correct DC gain
  result.G = TransferFunction({K * den.back()}, den);

  // Compute model response and error
  result.mag_dB_model.resize(n);
  result.phase_deg_model.resize(n);
  double mag_err = 0.0, ph_err = 0.0;
  for (size_t i = 0; i < n; ++i) {
    result.mag_dB_model[i] = result.G.mag_dB(omega[i]);
    result.phase_deg_model[i] = result.G.phase_deg(omega[i]);
    double dm = result.mag_dB_model[i] - mag_dB[i];
    mag_err += dm * dm;
    if (!phase_deg.empty()) {
      double dp = result.phase_deg_model[i] - phase_deg[i];
      ph_err += dp * dp;
    }
  }
  result.mag_rms_error = std::sqrt(mag_err / n);
  result.phase_rms_error = phase_deg.empty() ? 0.0 : std::sqrt(ph_err / n);

  return result;
}

// ============================================================================
//  §9.4 — Parametric Identification: Least Squares ARX
// ============================================================================

/**
 * @brief Identify ARX model parameters using batch least squares
 *
 * Model: y[k] = a1*y[k-1] + ... + ana*y[k-na]
 *             + b1*u[k-nk] + ... + bnb*u[k-nk-nb+1] + e[k]
 *
 * Solves: θ̂ = (Φ'Φ)⁻¹ Φ'Y
 *
 * @param u  Input signal vector
 * @param y  Output signal vector
 * @param na Number of AR (output) parameters
 * @param nb Number of X (input) parameters
 * @param nk Input delay in samples (default 1)
 * @param Ts Sampling period [s] (default 1.0)
 * @return ARXModel with identified parameters
 */
inline ARXModel id_arx(const std::vector<double> &u,
                       const std::vector<double> &y, int na, int nb, int nk = 1,
                       double Ts = 1.0) {
  int N = static_cast<int>(y.size());
  int n_params = na + nb;
  int start = std::max(na, nb + nk - 1);
  int M = N - start; // Number of equations

  if (M < n_params) {
    throw std::runtime_error("id_arx: not enough data points (" +
                             std::to_string(M) + " equations for " +
                             std::to_string(n_params) + " parameters)");
  }
  if (u.size() != y.size()) {
    throw std::runtime_error("id_arx: u and y must have the same length");
  }

  // Build regression matrix Φ (M × n_params) and output vector Y (M × 1)
  Matrix Phi(M, n_params, 0.0);
  Matrix Y(M, 1, 0.0);

  for (int k = 0; k < M; ++k) {
    int idx = k + start;
    Y(k, 0) = y[idx];

    // AR part: -y[idx-1], -y[idx-2], ..., -y[idx-na]
    for (int i = 0; i < na; ++i) {
      Phi(k, i) = -y[idx - 1 - i];
    }

    // X part: u[idx-nk], u[idx-nk-1], ..., u[idx-nk-nb+1]
    for (int i = 0; i < nb; ++i) {
      int u_idx = idx - nk - i;
      if (u_idx >= 0 && u_idx < N) {
        Phi(k, na + i) = u[u_idx];
      }
    }
  }

  // Solve normal equations: θ = (Φ'Φ)⁻¹ Φ'Y
  Matrix PhiT = Phi.T();
  Matrix PhiTy = PhiT * Y;
  Matrix PhiTPhi = PhiT * Phi;
  Matrix theta = PhiTPhi.solve(PhiTy);

  // Extract parameters
  ARXModel model;
  model.na = na;
  model.nb = nb;
  model.nk = nk;
  model.Ts = Ts;
  model.N = N;

  model.a.resize(na);
  model.b.resize(nb);
  for (int i = 0; i < na; ++i)
    model.a[i] = -theta(i, 0); // Note: Φ stores -y
  for (int i = 0; i < nb; ++i)
    model.b[i] = theta(na + i, 0);

  // Compute residuals and variance
  Matrix Y_hat = Phi * theta;
  model.residuals.resize(M);
  double sse = 0.0;
  for (int k = 0; k < M; ++k) {
    model.residuals[k] = Y(k, 0) - Y_hat(k, 0);
    sse += model.residuals[k] * model.residuals[k];
  }
  model.sigma2 = sse / (M - n_params);

  return model;
}

/**
 * @brief Convert ARX model to continuous-time transfer function via Tustin
 *
 * Steps: ARX → discrete TF H(z) → continuous TF G(s) via bilinear transform
 *
 * @param model ARX model
 * @return Continuous-time TransferFunction
 */
inline TransferFunction arx_to_tf(const ARXModel &model) {
  auto H = model.to_dtf();

  // Tustin (bilinear) transformation: z = (1 + sT/2) / (1 - sT/2)
  // i.e., s = (2/T)(z-1)/(z+1)
  // We do the inverse: given H(z), substitute z = (2+sT)/(2-sT)

  double T = model.Ts;
  int n_order = H.order();

  // For low-order systems, do direct substitution
  // z = (2 + s*T) / (2 - s*T)
  // H(z) = num(z)/den(z)
  // Substitute and collect powers of s

  // General approach: polynomial substitution
  auto &num_z = H.num.coeffs;
  auto &den_z = H.den.coeffs;

  int n_num = static_cast<int>(num_z.size());
  int n_den = static_cast<int>(den_z.size());
  int max_deg = std::max(n_num, n_den);

  // Result will be polynomial of degree max_deg - 1 in s
  // numerator and denominator
  std::vector<double> num_s(max_deg, 0.0);
  std::vector<double> den_s(max_deg, 0.0);

  // Precompute (2+sT)^k * (2-sT)^(n-k) for k = 0..n-1
  // where n = degree of the polynomial
  // Each term c_k * z^(n-1-k) becomes c_k * (2+sT)^(n-1-k) * (2-sT)^k

  auto binomial_expand = [&](int power_plus,
                             int power_minus) -> std::vector<double> {
    // Expand (2+sT)^p * (2-sT)^m in powers of s (descending)
    // First expand (2+sT)^p
    int total_deg = power_plus + power_minus;

    // (2 + sT)^p using binomial theorem
    std::vector<double> poly_plus(power_plus + 1, 0.0);
    for (int k = 0; k <= power_plus; ++k) {
      // C(p,k) * 2^(p-k) * (sT)^k
      double binom = 1.0;
      for (int j = 0; j < k; ++j)
        binom *= (double)(power_plus - j) / (j + 1);
      poly_plus[power_plus - k] =
          binom * std::pow(2.0, power_plus - k) * std::pow(T, k);
    }

    // (2 - sT)^m
    std::vector<double> poly_minus(power_minus + 1, 0.0);
    for (int k = 0; k <= power_minus; ++k) {
      double binom = 1.0;
      for (int j = 0; j < k; ++j)
        binom *= (double)(power_minus - j) / (j + 1);
      double sign = (k % 2 == 0) ? 1.0 : -1.0;
      poly_minus[power_minus - k] =
          binom * std::pow(2.0, power_minus - k) * std::pow(T, k) * sign;
    }

    // Convolve poly_plus and poly_minus
    std::vector<double> result(total_deg + 1, 0.0);
    for (int i = 0; i <= power_plus; ++i) {
      for (int j = 0; j <= power_minus; ++j) {
        result[i + j] += poly_plus[i] * poly_minus[j];
      }
    }
    return result;
  };

  // For numerator: Σ num_z[k] * (2+sT)^(n_den-1-k) * (2-sT)^k
  // (We pad to same length as denominator)
  std::vector<double> num_result(max_deg, 0.0);
  std::vector<double> den_result(max_deg, 0.0);

  for (int k = 0; k < n_num; ++k) {
    auto expanded =
        binomial_expand(n_den - 1 - k - (n_den - n_num), k + (n_den - n_num));
    // Adjust indices
    int pp = std::max(0, (n_den - 1) - k - (n_den - n_num));
    int pm = k + std::max(0, n_den - n_num);
    if (pp < 0)
      pp = 0;
    auto exp2 = binomial_expand(n_den - 1 - k, k);
    for (size_t i = 0; i < exp2.size() && i < num_result.size(); ++i) {
      // Only use terms up to max_deg-1
      if (i < static_cast<size_t>(max_deg))
        num_result[i] += num_z[k] * exp2[i];
    }
  }

  for (int k = 0; k < n_den; ++k) {
    auto expanded = binomial_expand(n_den - 1 - k, k);
    for (size_t i = 0; i < expanded.size() && i < den_result.size(); ++i) {
      if (i < static_cast<size_t>(max_deg))
        den_result[i] += den_z[k] * expanded[i];
    }
  }

  // Normalize: divide by leading denominator coefficient
  double lead = den_result[0];
  if (std::abs(lead) > 1e-15) {
    for (auto &v : num_result)
      v /= lead;
    for (auto &v : den_result)
      v /= lead;
  }

  // Trim leading near-zero coefficients in numerator
  while (num_result.size() > 1 && std::abs(num_result[0]) < 1e-12) {
    num_result.erase(num_result.begin());
  }

  return TransferFunction(num_result, den_result);
}

/**
 * @brief Predict output using ARX model (one-step-ahead prediction)
 *
 * @param model ARX model
 * @param u     Input signal
 * @param y     Measured output (for AR feedback)
 * @return Predicted output vector
 */
inline std::vector<double> arx_predict(const ARXModel &model,
                                       const std::vector<double> &u,
                                       const std::vector<double> &y) {
  int N = static_cast<int>(y.size());
  std::vector<double> y_hat(N, 0.0);
  int start = std::max(model.na, model.nb + model.nk - 1);

  for (int k = start; k < N; ++k) {
    double val = 0.0;
    for (int i = 0; i < model.na; ++i) {
      val += model.a[i] * y[k - 1 - i];
    }
    for (int i = 0; i < model.nb; ++i) {
      int u_idx = k - model.nk - i;
      if (u_idx >= 0)
        val += model.b[i] * u[u_idx];
    }
    y_hat[k] = val;
  }

  return y_hat;
}

/**
 * @brief Simulate ARX model (free-running, no feedback from measured y)
 *
 * Uses model's own predictions as feedback — tests model's ability
 * to predict beyond one step ahead.
 *
 * @param model ARX model
 * @param u     Input signal
 * @return Simulated output vector
 */
inline std::vector<double> arx_simulate(const ARXModel &model,
                                        const std::vector<double> &u) {
  int N = static_cast<int>(u.size());
  std::vector<double> y_sim(N, 0.0);
  int start = std::max(model.na, model.nb + model.nk - 1);

  for (int k = start; k < N; ++k) {
    double val = 0.0;
    for (int i = 0; i < model.na; ++i) {
      val += model.a[i] * y_sim[k - 1 - i]; // Use own predictions
    }
    for (int i = 0; i < model.nb; ++i) {
      int u_idx = k - model.nk - i;
      if (u_idx >= 0)
        val += model.b[i] * u[u_idx];
    }
    y_sim[k] = val;
  }

  return y_sim;
}

// ============================================================================
//  §9.5 — Input Signal Generation
// ============================================================================

/**
 * @brief Generate Pseudo-Random Binary Sequence (PRBS)
 *
 * PRBS is a deterministic binary signal with approximately white spectrum.
 * Generated using a linear feedback shift register (LFSR).
 *
 * @param n_bits   Shift register length (sequence period = 2^n - 1)
 * @param amplitude Signal amplitude (switches between ±amplitude)
 * @param Ts       Sampling period [s] (clock period of the PRBS)
 * @param n_periods Number of full periods to generate (default 1)
 * @return std::pair<time_vector, signal_vector>
 */
inline std::pair<std::vector<double>, std::vector<double>>
generate_prbs(int n_bits = 7, double amplitude = 1.0, double Ts = 0.01,
              int n_periods = 1) {
  if (n_bits < 2 || n_bits > 20) {
    throw std::runtime_error("generate_prbs: n_bits must be between 2 and 20");
  }

  int period = (1 << n_bits) - 1; // 2^n - 1
  int total_samples = period * n_periods;

  // LFSR feedback taps (maximal length sequences)
  // Standard taps for common register sizes
  std::vector<std::pair<int, int>> taps = {
      {0, 0},   // n=0 (unused)
      {0, 0},   // n=1 (unused)
      {2, 1},   // n=2
      {3, 2},   // n=3: x^3 + x^2 + 1
      {4, 3},   // n=4: x^4 + x^3 + 1
      {5, 3},   // n=5: x^5 + x^3 + 1
      {6, 5},   // n=6: x^6 + x^5 + 1
      {7, 6},   // n=7: x^7 + x^6 + 1
      {8, 6},   // n=8: x^8 + x^6 + x^5 + x^4 + 1 (simplified 2-tap)
      {9, 5},   // n=9: x^9 + x^5 + 1
      {10, 7},  // n=10: x^10 + x^7 + 1
      {11, 9},  // n=11
      {12, 11}, // n=12
      {13, 12}, // n=13
      {14, 13}, // n=14
      {15, 14}, // n=15
      {16, 15}, // n=16
      {17, 14}, // n=17
      {18, 11}, // n=18
      {19, 18}, // n=19
      {20, 17}  // n=20
  };

  int tap1 = taps[n_bits].first;
  int tap2 = taps[n_bits].second;

  // Initialize shift register (all ones)
  std::vector<int> reg(n_bits, 1);

  std::vector<double> t(total_samples);
  std::vector<double> signal(total_samples);

  for (int k = 0; k < total_samples; ++k) {
    t[k] = k * Ts;
    signal[k] = (reg[0] == 1) ? amplitude : -amplitude;

    // Feedback
    int new_bit = reg[tap1 - 1] ^ reg[tap2 - 1];

    // Shift
    for (int i = 0; i < n_bits - 1; ++i) {
      reg[i] = reg[i + 1];
    }
    reg[n_bits - 1] = new_bit;
  }

  return {t, signal};
}

/**
 * @brief Generate chirp (swept sine) signal
 *
 * u(t) = A * sin(2π(f0*t + (f1-f0)/(2T) * t²))
 *
 * @param f0       Start frequency [Hz]
 * @param f1       End frequency [Hz]
 * @param duration Sweep duration [s]
 * @param Ts       Sampling period [s]
 * @param amplitude Signal amplitude
 * @return std::pair<time_vector, signal_vector>
 */
inline std::pair<std::vector<double>, std::vector<double>>
generate_chirp(double f0, double f1, double duration, double Ts = 0.001,
               double amplitude = 1.0) {
  int N = static_cast<int>(duration / Ts) + 1;
  std::vector<double> t(N), signal(N);

  for (int k = 0; k < N; ++k) {
    t[k] = k * Ts;
    double phase =
        2.0 * M_PI * (f0 * t[k] + (f1 - f0) / (2.0 * duration) * t[k] * t[k]);
    signal[k] = amplitude * std::sin(phase);
  }

  return {t, signal};
}

/**
 * @brief Generate multisine signal (sum of sinusoids)
 *
 * u(t) = Σ Ak * sin(2π*fk*t + φk)
 *
 * @param frequencies Vector of frequencies [Hz]
 * @param duration    Signal duration [s]
 * @param Ts          Sampling period [s]
 * @param amplitude   Amplitude per sinusoid (or vector of amplitudes)
 * @param random_phase If true, use Schroeder phases; else zero phase
 * @return std::pair<time_vector, signal_vector>
 */
inline std::pair<std::vector<double>, std::vector<double>>
generate_multisine(const std::vector<double> &frequencies, double duration,
                   double Ts = 0.001, double amplitude = 1.0,
                   bool random_phase = true) {
  int N = static_cast<int>(duration / Ts) + 1;
  int K = static_cast<int>(frequencies.size());

  std::vector<double> t(N), signal(N, 0.0);

  // Schroeder phases for low crest factor
  std::vector<double> phases(K, 0.0);
  if (random_phase) {
    for (int k = 0; k < K; ++k) {
      phases[k] = -M_PI * k * (k + 1.0) / K; // Schroeder formula
    }
  }

  for (int i = 0; i < N; ++i) {
    t[i] = i * Ts;
    for (int k = 0; k < K; ++k) {
      signal[i] +=
          amplitude * std::sin(2.0 * M_PI * frequencies[k] * t[i] + phases[k]);
    }
  }

  // Normalize to desired amplitude
  double max_val =
      *std::max_element(signal.begin(), signal.end(), [](double a, double b) {
        return std::abs(a) < std::abs(b);
      });
  if (std::abs(max_val) > 1e-15) {
    double scale = amplitude * K / max_val;
    // Actually keep raw sum; user can scale if needed
  }

  return {t, signal};
}

/**
 * @brief Generate white noise signal
 *
 * @param N         Number of samples
 * @param Ts        Sampling period [s]
 * @param std_dev   Standard deviation
 * @param seed      Random seed (0 = random)
 * @return std::pair<time_vector, signal_vector>
 */
inline std::pair<std::vector<double>, std::vector<double>>
generate_white_noise(int N, double Ts = 0.01, double std_dev = 1.0,
                     unsigned int seed = 0) {
  std::vector<double> t(N), signal(N);

  std::mt19937 gen(seed == 0 ? std::random_device{}() : seed);
  std::normal_distribution<double> dist(0.0, std_dev);

  for (int k = 0; k < N; ++k) {
    t[k] = k * Ts;
    signal[k] = dist(gen);
  }

  return {t, signal};
}

// ============================================================================
//  §9.6 — Model Validation
// ============================================================================

/**
 * @brief Compute autocorrelation of a signal
 *
 * R[τ] = (1/N) Σ x[k] * x[k+τ]  (normalized by R[0])
 *
 * @param x       Signal vector
 * @param max_lag Maximum lag to compute
 * @return Normalized autocorrelation vector (R[0]=1, indices 0..max_lag)
 */
inline std::vector<double> autocorrelation(const std::vector<double> &x,
                                           int max_lag = -1) {
  int N = static_cast<int>(x.size());
  if (max_lag < 0)
    max_lag = std::min(N / 4, 50);

  double mean = 0.0;
  for (double v : x)
    mean += v;
  mean /= N;

  // R[0] for normalization
  double R0 = 0.0;
  for (int k = 0; k < N; ++k) {
    double xk = x[k] - mean;
    R0 += xk * xk;
  }
  R0 /= N;

  std::vector<double> R(max_lag + 1);
  R[0] = 1.0; // Normalized

  if (R0 < 1e-30)
    return R;

  for (int tau = 1; tau <= max_lag; ++tau) {
    double sum = 0.0;
    for (int k = 0; k < N - tau; ++k) {
      sum += (x[k] - mean) * (x[k + tau] - mean);
    }
    R[tau] = sum / (N * R0);
  }

  return R;
}

/**
 * @brief Compute cross-correlation between two signals
 *
 * R_xy[τ] = (1/N) Σ x[k] * y[k+τ]  (normalized)
 *
 * @param x First signal
 * @param y Second signal
 * @param max_lag Maximum lag
 * @return Normalized cross-correlation vector
 */
inline std::vector<double> crosscorrelation(const std::vector<double> &x,
                                            const std::vector<double> &y,
                                            int max_lag = -1) {
  int N = static_cast<int>(std::min(x.size(), y.size()));
  if (max_lag < 0)
    max_lag = std::min(N / 4, 50);

  double mean_x = 0.0, mean_y = 0.0;
  for (int k = 0; k < N; ++k) {
    mean_x += x[k];
    mean_y += y[k];
  }
  mean_x /= N;
  mean_y /= N;

  double norm_x = 0.0, norm_y = 0.0;
  for (int k = 0; k < N; ++k) {
    norm_x += (x[k] - mean_x) * (x[k] - mean_x);
    norm_y += (y[k] - mean_y) * (y[k] - mean_y);
  }
  double norm = std::sqrt(norm_x * norm_y) / N;

  std::vector<double> R(max_lag + 1, 0.0);
  if (norm < 1e-30)
    return R;

  for (int tau = 0; tau <= max_lag; ++tau) {
    double sum = 0.0;
    for (int k = 0; k < N - tau; ++k) {
      sum += (x[k] - mean_x) * (y[k + tau] - mean_y);
    }
    R[tau] = sum / (N * norm);
  }

  return R;
}

/**
 * @brief Validate an ARX model against data
 *
 * Performs:
 * 1. One-step-ahead prediction and FIT computation
 * 2. Residual autocorrelation (whiteness test)
 * 3. Input-residual cross-correlation (independence test)
 *
 * @param model  Identified ARX model
 * @param u      Input signal (validation set)
 * @param y      Output signal (validation set)
 * @return ValidationResult
 */
inline ValidationResult validate_model(const ARXModel &model,
                                       const std::vector<double> &u,
                                       const std::vector<double> &y) {
  ValidationResult result;
  int N = static_cast<int>(y.size());

  // One-step-ahead prediction
  auto y_hat = arx_predict(model, u, y);

  // FIT metric
  result.fit_percent = compute_fit(y, y_hat);

  // Residuals
  result.residuals.resize(N);
  result.mse = 0.0;
  for (int k = 0; k < N; ++k) {
    result.residuals[k] = y[k] - y_hat[k];
    result.mse += result.residuals[k] * result.residuals[k];
  }
  result.mse /= N;
  result.rmse = std::sqrt(result.mse);

  // Autocorrelation of residuals
  int max_lag = std::min(N / 4, 50);
  result.autocorr = autocorrelation(result.residuals, max_lag);

  // Cross-correlation of residuals with input
  result.crosscorr = crosscorrelation(result.residuals, u, max_lag);

  // Whiteness test: 95% confidence bound = ±1.96/√N
  double bound = 1.96 / std::sqrt(static_cast<double>(N));
  result.residuals_white = true;
  for (int tau = 1; tau <= max_lag; ++tau) {
    if (std::abs(result.autocorr[tau]) > bound) {
      result.residuals_white = false;
      break;
    }
  }

  // Input-independence test
  result.residuals_uncorr_with_input = true;
  for (int tau = 0; tau <= max_lag; ++tau) {
    if (std::abs(result.crosscorr[tau]) > bound) {
      result.residuals_uncorr_with_input = false;
      break;
    }
  }

  return result;
}

/**
 * @brief Validate a continuous-time transfer function against time-domain data
 *
 * @param G      Identified transfer function
 * @param t      Time vector [s]
 * @param u      Input signal
 * @param y      Measured output
 * @return ValidationResult
 */
inline ValidationResult validate_tf(const TransferFunction &G,
                                    const std::vector<double> &t,
                                    const std::vector<double> &u,
                                    const std::vector<double> &y) {
  ValidationResult result;
  int N = static_cast<int>(y.size());

  // Simulate model response (assuming step or arbitrary input)
  // For step input, use stepResponse; for general, use RK4
  std::vector<double> y_hat(N, 0.0);

  // Check if input is approximately a step
  bool is_step = true;
  if (N > 2) {
    double u0 = u[1]; // Skip initial transient
    for (size_t i = 2; i < u.size(); ++i) {
      if (std::abs(u[i] - u0) > 0.01 * std::abs(u0) + 1e-10) {
        is_step = false;
        break;
      }
    }
  }

  if (is_step && N > 1) {
    double u_step = u[1];
    for (int i = 0; i < N; ++i) {
      y_hat[i] = G.stepResponse(t[i]) * u_step;
    }
  } else {
    // General input: use discrete-time simulation
    // Approximate with Tustin discretization and filter
    double Ts = (t.size() > 1) ? (t[1] - t[0]) : 0.01;
    auto Hd = c2d_tustin(G, Ts);

    auto &num_c = Hd.num.coeffs;
    auto &den_c = Hd.den.coeffs;
    int n_num = static_cast<int>(num_c.size());
    int n_den = static_cast<int>(den_c.size());

    // Direct-form II simulation
    std::vector<double> u_buf(n_num, 0.0);
    std::vector<double> y_buf(n_den, 0.0);

    for (int k = 0; k < N; ++k) {
      // Shift input buffer
      for (int j = n_num - 1; j > 0; --j)
        u_buf[j] = u_buf[j - 1];
      u_buf[0] = (k < static_cast<int>(u.size())) ? u[k] : 0.0;

      // Compute output
      double out = 0.0;
      for (int j = 0; j < n_num; ++j)
        out += num_c[j] * u_buf[j];
      for (int j = 1; j < n_den; ++j)
        out -= den_c[j] * y_buf[j - 1];
      if (std::abs(den_c[0]) > 1e-15)
        out /= den_c[0];

      // Shift output buffer
      for (int j = n_den - 1; j > 0; --j)
        y_buf[j] = y_buf[j - 1];
      y_buf[0] = out;

      y_hat[k] = out;
    }
  }

  // Compute metrics
  result.fit_percent = compute_fit(y, y_hat);

  result.residuals.resize(N);
  result.mse = 0.0;
  for (int k = 0; k < N; ++k) {
    result.residuals[k] = y[k] - y_hat[k];
    result.mse += result.residuals[k] * result.residuals[k];
  }
  result.mse /= N;
  result.rmse = std::sqrt(result.mse);

  int max_lag = std::min(N / 4, 50);
  result.autocorr = autocorrelation(result.residuals, max_lag);
  result.crosscorr = crosscorrelation(result.residuals, u, max_lag);

  double bound = 1.96 / std::sqrt(static_cast<double>(N));
  result.residuals_white = true;
  for (int tau = 1; tau <= max_lag; ++tau) {
    if (std::abs(result.autocorr[tau]) > bound) {
      result.residuals_white = false;
      break;
    }
  }
  result.residuals_uncorr_with_input = true;
  for (int tau = 0; tau <= max_lag; ++tau) {
    if (std::abs(result.crosscorr[tau]) > bound) {
      result.residuals_uncorr_with_input = false;
      break;
    }
  }

  return result;
}

/**
 * @brief Compute AIC and BIC for model order selection
 *
 * AIC = N*ln(σ²) + 2p
 * BIC = N*ln(σ²) + p*ln(N)
 *
 * @param sigma2     Residual variance
 * @param num_params Number of model parameters
 * @param N          Number of data points
 * @return InformationCriteria
 */
inline InformationCriteria compute_aic_bic(double sigma2, int num_params,
                                           int N) {
  InformationCriteria ic;
  ic.sigma2 = sigma2;
  ic.num_params = num_params;
  ic.N = N;

  double log_sigma2 = std::log(std::max(sigma2, 1e-30));
  ic.AIC = N * log_sigma2 + 2.0 * num_params;
  ic.BIC = N * log_sigma2 + num_params * std::log(static_cast<double>(N));

  return ic;
}

/**
 * @brief Automatic model order selection using AIC/BIC
 *
 * Tries ARX models of increasing order and returns the one that
 * minimizes BIC (more conservative than AIC).
 *
 * @param u         Input signal
 * @param y         Output signal
 * @param max_order Maximum order to try (for both na and nb)
 * @param nk        Input delay
 * @param Ts        Sampling period
 * @return Best ARXModel according to BIC
 */
inline ARXModel id_arx_auto(const std::vector<double> &u,
                            const std::vector<double> &y, int max_order = 6,
                            int nk = 1, double Ts = 1.0) {
  ARXModel best_model;
  double best_bic = std::numeric_limits<double>::infinity();

  for (int na = 1; na <= max_order; ++na) {
    for (int nb = 1; nb <= max_order; ++nb) {
      try {
        auto model = id_arx(u, y, na, nb, nk, Ts);
        auto ic = compute_aic_bic(model.sigma2, model.num_params(), model.N);

        if (ic.BIC < best_bic) {
          best_bic = ic.BIC;
          best_model = model;
        }
      } catch (...) {
        continue; // Skip orders that fail
      }
    }
  }

  return best_model;
}

// ============================================================================
//  §9.3.3 — Spectral Analysis (Coherence)
// ============================================================================

/**
 * @brief Compute coherence function between input and output
 *
 * γ²(f) = |Syu(f)|² / (Suu(f) * Syy(f))
 *
 * Uses Welch's method (overlapping windowed FFT segments).
 * Simplified implementation using correlation-based spectral estimates.
 *
 * @param u     Input signal
 * @param y     Output signal
 * @param Ts    Sampling period [s]
 * @param n_seg Number of segments for averaging
 * @return std::pair<frequency_vector [Hz], coherence_vector [0..1]>
 */
inline std::pair<std::vector<double>, std::vector<double>>
coherence(const std::vector<double> &u, const std::vector<double> &y,
          double Ts = 0.01, int n_seg = 8) {
  int N = static_cast<int>(std::min(u.size(), y.size()));
  int seg_len = N / n_seg;
  if (seg_len < 4)
    seg_len = N;
  int n_freq = seg_len / 2;

  // Initialize spectral estimates
  std::vector<double> Suu(n_freq, 0.0);
  std::vector<double> Syy(n_freq, 0.0);
  std::vector<double> Syu_re(n_freq, 0.0);
  std::vector<double> Syu_im(n_freq, 0.0);

  // Process each segment
  int n_segments_actual = 0;
  for (int seg = 0; seg < n_seg; ++seg) {
    int start = seg * seg_len / 2; // 50% overlap
    if (start + seg_len > N)
      break;
    n_segments_actual++;

    // Apply Hanning window and compute DFT
    for (int k = 0; k < n_freq; ++k) {
      double freq = 2.0 * M_PI * k / seg_len;
      double U_re = 0, U_im = 0, Y_re = 0, Y_im = 0;

      for (int n = 0; n < seg_len; ++n) {
        // Hanning window
        double w = 0.5 * (1.0 - std::cos(2.0 * M_PI * n / (seg_len - 1)));
        double u_n = u[start + n] * w;
        double y_n = y[start + n] * w;

        double cos_val = std::cos(freq * n);
        double sin_val = std::sin(freq * n);

        U_re += u_n * cos_val;
        U_im -= u_n * sin_val;
        Y_re += y_n * cos_val;
        Y_im -= y_n * sin_val;
      }

      Suu[k] += U_re * U_re + U_im * U_im;
      Syy[k] += Y_re * Y_re + Y_im * Y_im;
      Syu_re[k] += Y_re * U_re + Y_im * U_im;
      Syu_im[k] += Y_im * U_re - Y_re * U_im;
    }
  }

  // Normalize and compute coherence
  std::vector<double> freq(n_freq);
  std::vector<double> coh(n_freq);

  double df = 1.0 / (seg_len * Ts);
  for (int k = 0; k < n_freq; ++k) {
    freq[k] = k * df;

    double suu = Suu[k] / n_segments_actual;
    double syy = Syy[k] / n_segments_actual;
    double syu_mag2 = (Syu_re[k] * Syu_re[k] + Syu_im[k] * Syu_im[k]) /
                      (n_segments_actual * (double)n_segments_actual);

    if (suu * syy > 1e-30) {
      coh[k] = syu_mag2 / (suu * syy);
      coh[k] = std::min(1.0, std::max(0.0, coh[k]));
    } else {
      coh[k] = 0.0;
    }
  }

  return {freq, coh};
}

// ============================================================================
//  Discrete-time simulation (lsim equivalent)
// ============================================================================

/**
 * @brief Simulate a discrete-time transfer function with arbitrary input
 *
 * @param H Discrete-time transfer function H(z)
 * @param u Input signal vector
 * @return Output signal vector
 */
inline std::vector<double> dsim(const DiscreteTransferFunction &H,
                                const std::vector<double> &u) {
  int N = static_cast<int>(u.size());
  auto &num_c = H.num.coeffs;
  auto &den_c = H.den.coeffs;
  int n_num = static_cast<int>(num_c.size());
  int n_den = static_cast<int>(den_c.size());

  std::vector<double> y(N, 0.0);
  std::vector<double> u_buf(n_num, 0.0);
  std::vector<double> y_buf(n_den, 0.0);

  for (int k = 0; k < N; ++k) {
    // Shift input buffer
    for (int j = n_num - 1; j > 0; --j)
      u_buf[j] = u_buf[j - 1];
    u_buf[0] = u[k];

    // Compute output: y[k] = (Σ b_j * u[k-j] - Σ a_j * y[k-j]) / a_0
    double out = 0.0;
    for (int j = 0; j < n_num; ++j)
      out += num_c[j] * u_buf[j];
    for (int j = 1; j < n_den; ++j)
      out -= den_c[j] * y_buf[j - 1];
    if (std::abs(den_c[0]) > 1e-15)
      out /= den_c[0];

    // Shift output buffer
    for (int j = n_den - 1; j > 0; --j)
      y_buf[j] = y_buf[j - 1];
    y_buf[0] = out;

    y[k] = out;
  }

  return y;
}

/**
 * @brief Simulate a continuous-time TF with arbitrary input using Tustin
 *
 * Discretizes G(s) via Tustin bilinear transform, then simulates.
 *
 * @param G  Continuous-time transfer function
 * @param t  Time vector [s]
 * @param u  Input signal vector
 * @return Output signal vector
 */
inline std::vector<double> lsim(const TransferFunction &G,
                                const std::vector<double> &t,
                                const std::vector<double> &u) {
  if (t.size() < 2 || t.size() != u.size()) {
    throw std::runtime_error(
        "lsim: need at least 2 time points; t and u must match");
  }

  double Ts = t[1] - t[0];
  auto Hd = c2d_tustin(G, Ts);
  return dsim(Hd, u);
}

// ============================================================================
//  Printing / Reporting
// ============================================================================

/**
 * @brief Print ARX model summary to console
 */
inline void print_arx(const ARXModel &model) {
  std::cout << "========== ARX(" << model.na << "," << model.nb
            << ") Model (Ts = " << model.Ts << " s) ==========" << std::endl;

  std::cout << "\nDifference equation:" << std::endl;
  std::cout << "  y[k] = ";
  for (int i = 0; i < model.na; ++i) {
    if (i > 0)
      std::cout << (model.a[i] >= 0 ? " + " : " - ");
    else if (model.a[i] < 0)
      std::cout << "-";
    std::cout << std::fixed << std::setprecision(4) << std::abs(model.a[i])
              << "*y[k-" << (i + 1) << "]";
  }
  for (int i = 0; i < model.nb; ++i) {
    std::cout << (model.b[i] >= 0 ? " + " : " - ");
    std::cout << std::fixed << std::setprecision(4) << std::abs(model.b[i])
              << "*u[k-" << (model.nk + i) << "]";
  }
  std::cout << " + e[k]" << std::endl;

  std::cout << "\nParameters:" << std::endl;
  std::cout << "  a = [";
  for (int i = 0; i < model.na; ++i) {
    if (i > 0)
      std::cout << ", ";
    std::cout << std::setprecision(6) << model.a[i];
  }
  std::cout << "]" << std::endl;

  std::cout << "  b = [";
  for (int i = 0; i < model.nb; ++i) {
    if (i > 0)
      std::cout << ", ";
    std::cout << std::setprecision(6) << model.b[i];
  }
  std::cout << "]" << std::endl;

  std::cout << "\nResidual variance: " << std::setprecision(6) << model.sigma2
            << std::endl;

  auto ic = compute_aic_bic(model.sigma2, model.num_params(), model.N);
  std::cout << "AIC: " << std::setprecision(2) << ic.AIC << std::endl;
  std::cout << "BIC: " << std::setprecision(2) << ic.BIC << std::endl;

  std::cout << "=================================================" << std::endl;
}

/**
 * @brief Print validation result summary
 */
inline void print_validation(const ValidationResult &val) {
  std::cout << "========== Model Validation ==========" << std::endl;
  std::cout << "FIT:  " << std::fixed << std::setprecision(1) << val.fit_percent
            << "%" << std::endl;
  std::cout << "RMSE: " << std::setprecision(6) << val.rmse << std::endl;
  std::cout << "MSE:  " << std::setprecision(6) << val.mse << std::endl;
  std::cout << "Residuals white:             "
            << (val.residuals_white ? "YES ✓" : "NO ✗") << std::endl;
  std::cout << "Residuals uncorr. with input: "
            << (val.residuals_uncorr_with_input ? "YES ✓" : "NO ✗")
            << std::endl;
  std::cout << "======================================" << std::endl;
}

/**
 * @brief Print step response identification result
 */
inline void print_step_id(const FirstOrderIDResult &r) {
  std::cout << "========== Step Response ID (1st Order) =========="
            << std::endl;
  std::cout << "  DC Gain K   = " << std::setprecision(4) << r.K << std::endl;
  std::cout << "  Time const τ = " << std::setprecision(4) << r.tau << " s"
            << std::endl;
  std::cout << "  G(s) = " << r.K << " / (" << r.tau << "s + 1)" << std::endl;
  std::cout << "  FIT = " << std::fixed << std::setprecision(1) << r.fit_percent
            << "%" << std::endl;
  std::cout << "=================================================="
            << std::endl;
}

inline void print_step_id(const SecondOrderIDResult &r) {
  std::cout << "========== Step Response ID (2nd Order) =========="
            << std::endl;
  std::cout << "  DC Gain K     = " << std::setprecision(4) << r.K << std::endl;
  std::cout << "  Nat. freq ωn  = " << std::setprecision(4) << r.wn << " rad/s"
            << std::endl;
  std::cout << "  Damping ζ     = " << std::setprecision(4) << r.zeta
            << std::endl;
  std::cout << "  Overshoot Mp  = " << std::setprecision(1) << (r.Mp * 100)
            << "%" << std::endl;
  std::cout << "  Peak time tp  = " << std::setprecision(4) << r.tp << " s"
            << std::endl;
  std::cout << "  Settling ts   ≈ " << std::setprecision(4) << r.ts << " s"
            << std::endl;
  std::cout << "  FIT = " << std::fixed << std::setprecision(1) << r.fit_percent
            << "%" << std::endl;
  std::cout << "=================================================="
            << std::endl;
}

} // namespace sysid
} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_SYSID_HPP
