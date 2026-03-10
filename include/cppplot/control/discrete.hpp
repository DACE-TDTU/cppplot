/**
 * @file discrete.hpp
 * @brief Discrete-time transfer function and analysis
 *
 * Supports z-domain transfer functions: H(z) = num(z) / den(z)
 */

#ifndef CPPPLOT_CONTROL_DISCRETE_HPP
#define CPPPLOT_CONTROL_DISCRETE_HPP

// Define M_PI for Windows compatibility
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "../pyplot.hpp"
#include "polynomial.hpp"
#include "state_space.hpp"
#include "transfer_function.hpp"
#include <cmath>
#include <complex>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

namespace cppplot {
namespace control {

/**
 * @class DiscreteTransferFunction
 * @brief Discrete-time transfer function H(z) = num(z)/den(z)
 *
 * @example
 * // H(z) = (0.5z + 0.5) / (z - 0.8)
 * DiscreteTransferFunction H({0.5, 0.5}, {1, -0.8}, 0.1);  // Ts = 0.1s
 */
class DiscreteTransferFunction {
public:
  Polynomial num; // Numerator polynomial in z
  Polynomial den; // Denominator polynomial in z
  double Ts;      // Sample time (seconds)

  // ============ Constructors ============

  /// Default: H(z) = 1, Ts = 1
  DiscreteTransferFunction() : num({1}), den({1}), Ts(1.0) {}

  /// From polynomials and sample time
  DiscreteTransferFunction(const Polynomial &n, const Polynomial &d,
                           double ts = 1.0)
      : num(n), den(d), Ts(ts) {
    if (den.isZero())
      throw std::runtime_error("Denominator cannot be zero");
  }

  /// From coefficient vectors
  DiscreteTransferFunction(const std::vector<double> &n,
                           const std::vector<double> &d, double ts = 1.0)
      : num(n), den(d), Ts(ts) {
    if (den.isZero())
      throw std::runtime_error("Denominator cannot be zero");
  }

  /// From initializer lists
  DiscreteTransferFunction(std::initializer_list<double> n,
                           std::initializer_list<double> d, double ts = 1.0)
      : num(n), den(d), Ts(ts) {
    if (den.isZero())
      throw std::runtime_error("Denominator cannot be zero");
  }

  // ============ Properties ============

  /// Order of the system
  int order() const { return den.degree(); }

  /// DC gain: H(1)
  double dcgain() const {
    double d1 = den(1.0);
    if (std::abs(d1) < 1e-15)
      return std::numeric_limits<double>::infinity();
    return num(1.0) / d1;
  }

  /// Sample frequency (rad/s)
  double omega_s() const { return 2 * M_PI / Ts; }

  /// Nyquist frequency (rad/s)
  double omega_nyquist() const { return M_PI / Ts; }

  // ============ Poles and Zeros ============

  /// Get poles (roots of denominator in z-plane)
  std::vector<std::complex<double>> poles() const { return den.roots(); }

  /// Get zeros (roots of numerator in z-plane)
  std::vector<std::complex<double>> zeros() const { return num.roots(); }

  // ============ Stability ============

  /// Check if system is stable (all poles inside unit circle)
  bool isStable() const {
    auto p = poles();
    for (const auto &pole : p) {
      if (std::abs(pole) >= 1.0)
        return false;
    }
    return true;
  }

  // ============ Frequency Response ============

  /// Evaluate at z
  std::complex<double> eval(std::complex<double> z) const {
    return num(z) / den(z);
  }

  /// Evaluate at z = e^(jωT) (frequency response)
  std::complex<double> freqresp(double omega) const {
    std::complex<double> z = std::polar(1.0, omega * Ts);
    return eval(z);
  }

  /// Magnitude at frequency ω (linear)
  double mag(double omega) const { return std::abs(freqresp(omega)); }

  /// Magnitude in dB
  double mag_dB(double omega) const {
    double m = mag(omega);
    if (m < 1e-20)
      return -400;
    return 20.0 * std::log10(m);
  }

  /// Phase in radians
  double phase(double omega) const { return std::arg(freqresp(omega)); }

  /// Phase in degrees
  double phase_deg(double omega) const { return phase(omega) * 180.0 / M_PI; }

  // ============ Arithmetic ============

  DiscreteTransferFunction
  operator*(const DiscreteTransferFunction &other) const {
    if (std::abs(Ts - other.Ts) > 1e-10) {
      throw std::runtime_error("Sample times must match for series connection");
    }
    return DiscreteTransferFunction(num * other.num, den * other.den, Ts);
  }

  DiscreteTransferFunction
  operator+(const DiscreteTransferFunction &other) const {
    if (std::abs(Ts - other.Ts) > 1e-10) {
      throw std::runtime_error(
          "Sample times must match for parallel connection");
    }
    Polynomial newNum = num * other.den + other.num * den;
    Polynomial newDen = den * other.den;
    return DiscreteTransferFunction(newNum, newDen, Ts);
  }

  DiscreteTransferFunction operator*(double K) const {
    return DiscreteTransferFunction(num * K, den, Ts);
  }

  // ============ String Representation ============

  std::string toString() const {
    std::ostringstream oss;
    std::string numStr = num.toString("z");
    std::string denStr = den.toString("z");

    size_t width = std::max(numStr.length(), denStr.length());

    size_t numPad = (width - numStr.length()) / 2;
    oss << std::string(numPad, ' ') << numStr << "\n";
    oss << std::string(width, '-') << "\n";
    size_t denPad = (width - denStr.length()) / 2;
    oss << std::string(denPad, ' ') << denStr;
    oss << "\n\nTs = " << Ts << " s";

    return oss.str();
  }
};

// ============ Discretization Methods ============

/**
 * @brief Perform polynomial substitution: P(s) where s = (2/T)*(z-1)/(z+1)
 *
 * This is the core of Tustin transform. Given polynomial P(s) = a_n*s^n + ... +
 * a_1*s + a_0 We substitute s = (2/T)*(z-1)/(z+1) to get a rational function in
 * z.
 *
 * Returns numerator and denominator polynomials in z.
 */
inline std::pair<Polynomial, Polynomial> tustin_substitute(const Polynomial &P,
                                                           double Ts) {
  int n = P.degree();
  double k = 2.0 / Ts;

  // Result: P(s) with s=(2/T)*(z-1)/(z+1) = sum_i a_i * (k*(z-1))^i / (z+1)^i
  // Numerator = sum_i a_i * k^i * (z-1)^i * (z+1)^(n-i)
  // Denominator = (z+1)^n

  // Build (z-1)^i and (z+1)^i powers
  std::vector<Polynomial> z_minus_1_pow(n + 1);
  std::vector<Polynomial> z_plus_1_pow(n + 1);

  z_minus_1_pow[0] = Polynomial({1.0}); // (z-1)^0 = 1
  z_plus_1_pow[0] = Polynomial({1.0});  // (z+1)^0 = 1

  Polynomial z_minus_1({1.0, -1.0}); // z - 1
  Polynomial z_plus_1({1.0, 1.0});   // z + 1

  for (int i = 1; i <= n; ++i) {
    z_minus_1_pow[i] = z_minus_1_pow[i - 1] * z_minus_1;
    z_plus_1_pow[i] = z_plus_1_pow[i - 1] * z_plus_1;
  }

  // Build numerator: sum_i a_i * k^i * (z-1)^i * (z+1)^(n-i)
  Polynomial num_result({0.0});
  double ki = 1.0; // k^i

  for (int i = 0; i <= n; ++i) {
    // Coefficient a_i (from highest to lowest power in original polynomial)
    // P.coeffs[0] is coefficient of s^n, P.coeffs[n] is coefficient of s^0
    double a_i = P.coeffs[n - i]; // coefficient of s^i

    // Term: a_i * k^i * (z-1)^i * (z+1)^(n-i)
    Polynomial term = z_minus_1_pow[i] * z_plus_1_pow[n - i];
    term = term * (a_i * ki);

    num_result = num_result + term;
    ki *= k;
  }

  // Denominator: (z+1)^n
  Polynomial den_result = z_plus_1_pow[n];

  return std::make_pair(num_result, den_result);
}

/**
 * @brief Convert continuous TF to discrete using Tustin (bilinear) transform
 * s = (2/T) * (z-1)/(z+1)
 *
 * This implementation handles general transfer functions with any order
 * and any number of zeros (numerator terms).
 *
 * @param Gc Continuous transfer function G(s)
 * @param Ts Sample time
 * @return Discrete transfer function H(z)
 */
inline DiscreteTransferFunction c2d_tustin(const TransferFunction &Gc,
                                           double Ts) {
  // Apply Tustin substitution to both numerator and denominator
  std::pair<Polynomial, Polynomial> num_result = tustin_substitute(Gc.num, Ts);
  std::pair<Polynomial, Polynomial> den_result = tustin_substitute(Gc.den, Ts);

  Polynomial num_z_num = num_result.first;
  Polynomial num_z_den = num_result.second;
  Polynomial den_z_num = den_result.first;
  Polynomial den_z_den = den_result.second;

  // H(z) = [num_z_num/num_z_den] / [den_z_num/den_z_den]
  //      = (num_z_num * den_z_den) / (den_z_num * num_z_den)

  Polynomial final_num = num_z_num * den_z_den;
  Polynomial final_den = den_z_num * num_z_den;

  // Normalize so leading coefficient of denominator is 1
  double scale = final_den.coeffs[0];
  if (std::abs(scale) > 1e-15) {
    for (double &c : final_num.coeffs)
      c /= scale;
    for (double &c : final_den.coeffs)
      c /= scale;
  }

  // Remove near-zero leading coefficients
  while (final_num.coeffs.size() > 1 &&
         std::abs(final_num.coeffs.front()) < 1e-15) {
    final_num.coeffs.erase(final_num.coeffs.begin());
  }
  while (final_den.coeffs.size() > 1 &&
         std::abs(final_den.coeffs.front()) < 1e-15) {
    final_den.coeffs.erase(final_den.coeffs.begin());
  }

  return DiscreteTransferFunction(final_num, final_den, Ts);
}

/**
 * @brief Convert continuous TF to discrete using Zero-Order Hold (ZOH)
 *
 * For G(s) = B(s)/A(s), the ZOH equivalent is:
 * H(z) = Z{ (1-z^(-1)) * L^(-1){ G(s)/s } }
 *
 * This implementation handles:
 * - First order systems: G(s) = K / (τs + 1)
 * - Second order underdamped: complex conjugate poles
 * - Second order overdamped: two real poles
 * - Second order critically damped: repeated real poles
 * - Falls back to step-invariant approximation for higher orders
 */
inline DiscreteTransferFunction c2d_zoh(const TransferFunction &Gc, double Ts) {
  // For first-order strictly proper system without zeros: G(s) = K / (τs + 1)
  if (Gc.order() == 1 && Gc.num.degree() == 0) {
    double K = Gc.dcgain();
    double tau = -1.0 / Gc.poles()[0].real();

    double alpha = std::exp(-Ts / tau);
    double b0 = K * (1 - alpha);

    return DiscreteTransferFunction({b0}, {1, -alpha}, Ts);
  }

  // For general systems (any order, with or without zeros)
  // We use the exact State-Space approach:
  // Ad = e^(A*Ts)
  // Bd = ∫_0^Ts e^(At) dt * B
  // Computed via block matrix exponential: M = [A B; 0 0]*Ts -> expm(M) = [Ad
  // Bd; 0 I]
  if (Gc.order() >= 1) {
    // Convert to Continuous StateSpace
    StateSpace sys_c = tf2ss(Gc);
    size_t n = sys_c.n_states;
    size_t m = sys_c.n_inputs;

    // Form augmented matrix M = [A B; 0 0] * Ts
    size_t nm = n + m;
    Matrix M = Matrix::zeros(nm, nm);
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < n; ++j) {
        M(i, j) = sys_c.A(i, j) * Ts;
      }
      for (size_t j = 0; j < m; ++j) {
        M(i, n + j) = sys_c.B(i, j) * Ts;
      }
    }

    // Compute matrix exponential M_exp = expm(M)
    Matrix M_exp = M.expm();

    // Extract Ad and Bd
    Matrix Ad = Matrix::zeros(n, n);
    Matrix Bd = Matrix::zeros(n, m);
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < n; ++j) {
        Ad(i, j) = M_exp(i, j);
      }
      for (size_t j = 0; j < m; ++j) {
        Bd(i, j) = M_exp(i, n + j);
      }
    }

    // Discrete state-space representation
    StateSpace sys_d(Ad, Bd, sys_c.C, sys_c.D);

    // Convert back to Transfer Function
    TransferFunction Gz = ss2tf(sys_d);

    // Clean up small numerical noise coefficients near zero
    std::vector<double> num_clean, den_clean;
    for (double c : Gz.num.coeffs) {
      if (std::abs(c) < 1e-12)
        num_clean.push_back(0.0);
      else
        num_clean.push_back(c);
    }
    for (double c : Gz.den.coeffs) {
      if (std::abs(c) < 1e-12)
        den_clean.push_back(0.0);
      else
        den_clean.push_back(c);
    }

    // Ensure numerator is clean and trimmed
    while (num_clean.size() > 1 && std::abs(num_clean.front()) < 1e-15) {
      num_clean.erase(num_clean.begin());
    }

    return DiscreteTransferFunction(num_clean, den_clean, Ts);
  }

  // Fallback to Tustin for zero order/unsupported cases
  return c2d_tustin(Gc, Ts);
}

// ============ Discrete-Time Factory Functions ============

/**
 * @brief Create discrete integrator: H(z) = Ts*z / (z-1) (forward Euler)
 */
inline DiscreteTransferFunction dtf_integrator(double Ts) {
  return DiscreteTransferFunction({Ts, 0}, {1, -1}, Ts);
}

/**
 * @brief Create discrete integrator (Tustin): H(z) = Ts/2 * (z+1)/(z-1)
 */
inline DiscreteTransferFunction dtf_integrator_tustin(double Ts) {
  return DiscreteTransferFunction({Ts / 2, Ts / 2}, {1, -1}, Ts);
}

/**
 * @brief Create discrete first-order low-pass: H(z) = (1-α)/(z-α) where α =
 * e^(-Ts/τ)
 */
inline DiscreteTransferFunction dtf_lowpass(double tau, double Ts) {
  double alpha = std::exp(-Ts / tau);
  return DiscreteTransferFunction({1 - alpha}, {1, -alpha}, Ts);
}

// ============ Plotting Functions ============

/**
 * @brief Plot discrete Bode diagram
 */
inline void dbode(const DiscreteTransferFunction &H) {
  // Frequency range: 0 to Nyquist
  std::vector<double> omega;
  double w_nyq = H.omega_nyquist();

  for (double w = w_nyq / 1000; w <= w_nyq; w *= 1.03) {
    omega.push_back(w);
  }

  std::vector<double> mag, phase_raw;
  for (double w : omega) {
    mag.push_back(H.mag_dB(w));
    phase_raw.push_back(H.phase_deg(w));
  }

  // Unwrap phase
  std::vector<double> phase = phase_raw;
  for (size_t i = 1; i < phase.size(); ++i) {
    while (phase[i] - phase[i - 1] > 180)
      phase[i] -= 360;
    while (phase[i] - phase[i - 1] < -180)
      phase[i] += 360;
  }

  figure(800, 600);
  layout(2, 1);

  subplot(2, 1, 1);
  plot(omega, mag, "-", opts({{"color", "#1f77b4"}, {"linewidth", "2"}}));
  xscale("log");
  axhline(0,
          opts({{"color", "black"}, {"linestyle", "-"}, {"linewidth", "0.5"}}));
  axhline(-3,
          opts({{"color", "gray"}, {"linestyle", ":"}, {"linewidth", "0.8"}}));
  axvline(w_nyq,
          opts({{"color", "red"}, {"linestyle", "--"}, {"linewidth", "1"}}));
  text(w_nyq * 0.7, 5, "Nyquist", opts({{"fontsize", "9"}, {"color", "red"}}));
  ylabel("Magnitude (dB)");
  grid(true);

  subplot(2, 1, 2);
  plot(omega, phase, "-", opts({{"color", "#1f77b4"}, {"linewidth", "2"}}));
  xscale("log");
  axhline(-180,
          opts({{"color", "gray"}, {"linestyle", ":"}, {"linewidth", "0.5"}}));
  axvline(w_nyq,
          opts({{"color", "red"}, {"linestyle", "--"}, {"linewidth", "1"}}));
  xlabel("Frequency (rad/s)");
  ylabel("Phase (deg)");
  grid(true);

  std::ostringstream title_str;
  title_str << "Discrete Bode Diagram (Ts = " << H.Ts << " s)";
  suptitle(title_str.str());
}

/**
 * @brief Plot discrete pole-zero map (z-plane with unit circle)
 */
inline void dzpmap(const DiscreteTransferFunction &H) {
  auto p = H.poles();
  auto z = H.zeros();

  std::vector<double> poles_re, poles_im;
  std::vector<double> zeros_re, zeros_im;

  for (const auto &pole : p) {
    poles_re.push_back(pole.real());
    poles_im.push_back(pole.imag());
  }
  for (const auto &zero : z) {
    zeros_re.push_back(zero.real());
    zeros_im.push_back(zero.imag());
  }

  figure(700, 700);

  // Unit circle (stability boundary)
  std::vector<double> circle_x, circle_y;
  for (int i = 0; i <= 360; ++i) {
    double theta = i * M_PI / 180.0;
    circle_x.push_back(std::cos(theta));
    circle_y.push_back(std::sin(theta));
  }
  plot(circle_x, circle_y, "-",
       opts({{"color", "gray"}, {"linewidth", "1.5"}, {"linestyle", "--"}}));

  // Fill unstable region (outside unit circle)
  // Shade stable region (inside)
  fill_between(std::vector<double>{-1.5, 1.5}, std::vector<double>{-1.5, -1.5},
               std::vector<double>{1.5, 1.5},
               opts({{"color", "red"}, {"alpha", "0.05"}}));

  // Axes
  axhline(0, opts({{"color", "black"}, {"linewidth", "0.8"}}));
  axvline(0, opts({{"color", "black"}, {"linewidth", "0.8"}}));

  // Plot zeros (o)
  if (!zeros_re.empty()) {
    scatter(zeros_re, zeros_im,
            opts({{"s", "60"}, {"color", "blue"}, {"marker", "o"}}));
  }

  // Plot poles (x)
  if (!poles_re.empty()) {
    scatter(poles_re, poles_im,
            opts({{"s", "60"}, {"color", "red"}, {"marker", "x"}}));
  }

  xlabel("Real Axis");
  ylabel("Imaginary Axis");
  title("Discrete Pole-Zero Map (z-plane)");
  grid(true);

  xlim(-1.5, 1.5);
  ylim(-1.5, 1.5);
}

/**
 * @brief Plot discrete step response
 */
inline void dstep(const DiscreteTransferFunction &H, int num_samples = 50) {
  // Simulate step response using difference equation
  // y[k] = -a1*y[k-1] - a2*y[k-2] - ... + b0*u[k] + b1*u[k-1] + ...

  int n_num = H.num.degree() + 1;
  int n_den = H.den.degree() + 1;

  std::vector<double> a = H.den.coeffs; // Denominator coeffs
  std::vector<double> b = H.num.coeffs; // Numerator coeffs

  // Normalize by a[0]
  double a0 = a[0];
  for (double &ai : a)
    ai /= a0;
  for (double &bi : b)
    bi /= a0;

  std::vector<double> y(num_samples, 0); // Output
  std::vector<double> u(num_samples, 1); // Step input (all 1s)

  // Simulation
  for (int k = 0; k < num_samples; ++k) {
    double yk = 0;

    // Add input terms
    for (int i = 0; i < n_num && i <= k; ++i) {
      yk += b[i] * u[k - i];
    }

    // Subtract output feedback terms
    for (int i = 1; i < n_den && i <= k; ++i) {
      yk -= a[i] * y[k - i];
    }

    y[k] = yk;
  }

  // Time vector
  std::vector<double> t(num_samples);
  for (int k = 0; k < num_samples; ++k) {
    t[k] = k * H.Ts;
  }

  figure(800, 500);

  // Stem plot (discrete)
  for (int k = 0; k < num_samples; ++k) {
    std::vector<double> stem_x = {t[k], t[k]};
    std::vector<double> stem_y = {0, y[k]};
    plot(stem_x, stem_y, "-", opts({{"color", "#1f77b4"}, {"linewidth", "1"}}));
  }
  scatter(t, y, opts({{"s", "20"}, {"color", "#1f77b4"}}));

  // Steady-state reference
  double ss = H.dcgain();
  if (std::isfinite(ss)) {
    axhline(ss,
            opts({{"color", "gray"}, {"linestyle", "--"}, {"linewidth", "1"}}));
  }
  axhline(0,
          opts({{"color", "black"}, {"linestyle", "-"}, {"linewidth", "0.5"}}));

  xlabel("Time (s)");
  ylabel("Amplitude");

  std::ostringstream title_str;
  title_str << "Discrete Step Response (Ts = " << H.Ts << " s)";
  title(title_str.str());

  grid(true);
}

/**
 * @brief Print discrete system info
 */
inline void dsysinfo(const DiscreteTransferFunction &H) {
  std::cout << "======== Discrete System Info ========" << std::endl;
  std::cout << "\nTransfer Function H(z):" << std::endl;
  std::cout << H.toString() << std::endl;

  std::cout << "\nProperties:" << std::endl;
  std::cout << "  Order: " << H.order() << std::endl;
  std::cout << "  Sample Time: " << H.Ts << " s" << std::endl;
  std::cout << "  Sample Freq: " << H.omega_s() << " rad/s" << std::endl;
  std::cout << "  Nyquist Freq: " << H.omega_nyquist() << " rad/s" << std::endl;
  std::cout << "  DC Gain: " << H.dcgain() << std::endl;
  std::cout << "  Stable: " << (H.isStable() ? "Yes" : "No") << std::endl;

  auto p = H.poles();
  std::cout << "\nPoles:" << std::endl;
  for (size_t i = 0; i < p.size(); ++i) {
    double mag = std::abs(p[i]);
    std::cout << "  z" << (i + 1) << " = " << std::fixed << std::setprecision(4)
              << p[i].real();
    if (std::abs(p[i].imag()) > 1e-10) {
      std::cout << (p[i].imag() >= 0 ? " + " : " - ") << std::abs(p[i].imag())
                << "j";
    }
    std::cout << "  |z| = " << mag << (mag >= 1 ? " (UNSTABLE)" : "")
              << std::endl;
  }

  std::cout << "======================================" << std::endl;
}

} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_DISCRETE_HPP
