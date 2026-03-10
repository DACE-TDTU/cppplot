/**
 * @file transfer_function.hpp
 * @brief Transfer Function class for control systems
 *
 * Represents G(s) = num(s) / den(s) = K * (s-z1)(s-z2)... / (s-p1)(s-p2)...
 */

#ifndef CPPPLOT_CONTROL_TRANSFER_FUNCTION_HPP
#define CPPPLOT_CONTROL_TRANSFER_FUNCTION_HPP

// Define M_PI for Windows compatibility
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "polynomial.hpp"
#include <cmath>
#include <complex>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace cppplot {
namespace control {

/**
 * @class TransferFunction
 * @brief Continuous-time transfer function G(s) = num(s)/den(s)
 *
 * @example
 * // G(s) = (s + 1) / (s^2 + 2s + 1)
 * TransferFunction G({1, 1}, {1, 2, 1});
 *
 * // Evaluate at s = jω
 * auto H = G.freqresp(1.0);  // H(j1)
 */
class TransferFunction {
public:
  Polynomial num; // Numerator polynomial
  Polynomial den; // Denominator polynomial

  // ============ Constructors ============

  /// Default: G(s) = 1
  TransferFunction() : num({1}), den({1}) {}

  /// From num/den polynomials
  TransferFunction(const Polynomial &n, const Polynomial &d) : num(n), den(d) {
    if (den.isZero()) {
      throw std::runtime_error("Denominator cannot be zero");
    }
    normalize();
  }

  /// From coefficient vectors
  TransferFunction(const std::vector<double> &n, const std::vector<double> &d)
      : num(n), den(d) {
    if (den.isZero()) {
      throw std::runtime_error("Denominator cannot be zero");
    }
    normalize();
  }

  /// From initializer lists (most convenient)
  TransferFunction(std::initializer_list<double> n,
                   std::initializer_list<double> d)
      : num(n), den(d) {
    if (den.isZero()) {
      throw std::runtime_error("Denominator cannot be zero");
    }
    normalize();
  }

  /// Constant gain
  explicit TransferFunction(double K) : num({K}), den({1}) {}

  // ============ Properties ============

  /// DC gain: G(0)
  double dcgain() const {
    double d0 = den(0.0);
    if (std::abs(d0) < 1e-15) {
      return std::numeric_limits<double>::infinity();
    }
    return num(0.0) / d0;
  }

  /// Order of the system
  int order() const { return den.degree(); }

  /// Number of poles
  int numPoles() const { return den.degree(); }

  /// Number of zeros
  int numZeros() const { return num.degree(); }

  /// Is proper (degree(num) <= degree(den))
  bool isProper() const { return num.degree() <= den.degree(); }

  /// Is strictly proper (degree(num) < degree(den))
  bool isStrictlyProper() const { return num.degree() < den.degree(); }

  // ============ Poles and Zeros ============

  /// Get poles (roots of denominator)
  std::vector<std::complex<double>> poles() const { return den.roots(); }

  /// Get zeros (roots of numerator)
  std::vector<std::complex<double>> zeros() const { return num.roots(); }

  /// Get gain (leading coefficient ratio)
  double gain() const { return num.leading() / den.leading(); }

  // ============ Accessor Aliases (for API compatibility) ============

  /// Get numerator polynomial
  const Polynomial &getNum() const { return num; }

  /// Get denominator polynomial
  const Polynomial &getDen() const { return den; }

  // ============ Frequency Response ============

  /// Evaluate at s
  std::complex<double> eval(std::complex<double> s) const {
    return num(s) / den(s);
  }

  /// Alias for eval() - API compatibility
  std::complex<double> evalS(std::complex<double> s) const { return eval(s); }

  /// Evaluate at s = jω (frequency response)
  std::complex<double> freqresp(double omega) const {
    std::complex<double> jw(0, omega);
    return eval(jw);
  }

  /// Magnitude at frequency ω (linear)
  double mag(double omega) const { return std::abs(freqresp(omega)); }

  /// Magnitude in dB at frequency ω
  double mag_dB(double omega) const {
    double m = mag(omega);
    if (m < 1e-20)
      return -400; // Avoid log(0)
    return 20.0 * std::log10(m);
  }

  /// Phase in radians at frequency ω
  double phase(double omega) const { return std::arg(freqresp(omega)); }

  /// Phase in degrees at frequency ω
  double phase_deg(double omega) const { return phase(omega) * 180.0 / M_PI; }

  // ============ Stability Analysis ============

  /// Check if system is stable (all poles in LHP)
  bool isStable() const {
    auto p = poles();
    for (const auto &pole : p) {
      if (pole.real() >= 0)
        return false;
    }
    return true;
  }

  /// Check if system is marginally stable
  bool isMarginallyStable() const {
    auto p = poles();
    bool hasJwPole = false;
    std::vector<std::complex<double>> jwPoles;
    for (const auto &pole : p) {
      if (pole.real() > 1e-10)
        return false; // Unstable
      if (std::abs(pole.real()) < 1e-10) {
        hasJwPole = true;
        // Check for multiplicity on jw axis
        for (const auto &existing : jwPoles) {
          if (std::abs(pole.imag() - existing.imag()) < 1e-5) {
            return false; // Repeated jw pole -> unstable
          }
        }
        jwPoles.push_back(pole);
      }
    }
    return hasJwPole;
  }

  // ============ Time Response ============

  /**
   * @brief Step response using numerical simulation (RK4 method)
   *
   * Works for any order system by converting to state-space
   * and simulating ẋ = Ax + Bu with u = 1 (step input)
   *
   * Uses controllable canonical form:
   * A = [0  1  0 ... 0 ]    B = [0]
   *     [0  0  1 ... 0 ]        [0]
   *     [    ...      ]        [.]
   *     [-a0 -a1 ... -an-1]     [1]
   *
   * @param t Time point
   * @return y(t) step response value
   */
  double stepResponse(double t) const {
    if (t < 0)
      return 0;
    if (t == 0)
      return 0; // Assuming strictly proper system

    int n = order();
    if (n == 0) {
      // Static gain
      return dcgain();
    }

    // Fast path for 1st order: G(s) = K / (τs + 1)
    if (n == 1) {
      double K = dcgain();
      double pole = poles()[0].real();
      double tau = -1.0 / pole;
      return K * (1.0 - std::exp(-t / tau));
    }

    // Fast path for 2nd order (analytical solution) - ONLY for systems without
    // zeros
    if (n == 2 && num.degree() == 0) {
      double a2 = den.coeffs[0];
      double a1 = den.coeffs[1];
      double a0 = den.coeffs[2];

      double wn = std::sqrt(a0 / a2);
      double zeta = a1 / (2 * wn * a2);
      double K = num(0.0) / a0; // DC gain

      if (zeta < 1.0 - 1e-10) {
        // Underdamped
        double wd = wn * std::sqrt(1 - zeta * zeta);
        double phi = std::atan2(std::sqrt(1 - zeta * zeta), zeta);
        return K * (1 - std::exp(-zeta * wn * t) * std::sin(wd * t + phi) /
                            std::sqrt(1 - zeta * zeta));
      } else if (zeta > 1.0 + 1e-10) {
        // Overdamped
        double s1 = -zeta * wn + wn * std::sqrt(zeta * zeta - 1);
        double s2 = -zeta * wn - wn * std::sqrt(zeta * zeta - 1);
        return K * (1 + (s2 * std::exp(s1 * t) - s1 * std::exp(s2 * t)) /
                            (s1 - s2));
      } else {
        // Critically damped
        return K * (1 - std::exp(-wn * t) * (1 + wn * t));
      }
    }

    // For higher order systems: use RK4 numerical integration
    // Convert to controllable canonical form state-space

    // Normalize denominator coefficients
    std::vector<double> a(n);
    double lead = den.coeffs[0];
    for (int i = 0; i < n; ++i) {
      a[i] = den.coeffs[n - i] / lead; // a[0] = a_0, a[n-1] = a_{n-1}
    }

    // Normalize numerator coefficients
    int m = num.degree();
    std::vector<double> b(n, 0.0);
    for (int i = 0; i <= m; ++i) {
      b[n - 1 - m + i] = num.coeffs[i] / lead;
    }

    // State vector x (initialized to 0)
    std::vector<double> x(n, 0.0);

    // RK4 integration parameters
    double dt = 0.001; // Integration step
    int steps = static_cast<int>(t / dt) + 1;
    dt = t / steps; // Adjust to hit exactly t

    // State derivative function: ẋ = Ax + Bu
    // For controllable canonical form with u = 1:
    // x_dot[0..n-2] = x[1..n-1]
    // x_dot[n-1] = -a[0]*x[0] - a[1]*x[1] - ... - a[n-1]*x[n-1] + 1
    auto state_derivative =
        [&](const std::vector<double> &state) -> std::vector<double> {
      std::vector<double> dx(n);
      for (int i = 0; i < n - 1; ++i) {
        dx[i] = state[i + 1];
      }
      dx[n - 1] = 1.0; // Bu term (u = 1, B = [0...0,1]')
      for (int i = 0; i < n; ++i) {
        dx[n - 1] -= a[i] * state[i];
      }
      return dx;
    };

    // RK4 integration
    for (int step = 0; step < steps; ++step) {
      std::vector<double> k1 = state_derivative(x);

      std::vector<double> x_temp(n);
      for (int i = 0; i < n; ++i)
        x_temp[i] = x[i] + 0.5 * dt * k1[i];
      std::vector<double> k2 = state_derivative(x_temp);

      for (int i = 0; i < n; ++i)
        x_temp[i] = x[i] + 0.5 * dt * k2[i];
      std::vector<double> k3 = state_derivative(x_temp);

      for (int i = 0; i < n; ++i)
        x_temp[i] = x[i] + dt * k3[i];
      std::vector<double> k4 = state_derivative(x_temp);

      for (int i = 0; i < n; ++i) {
        x[i] += dt * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0;
      }
    }

    // Output y = Cx + Du = b'x + d*u
    // For strictly proper (m < n), D = 0
    // C = [b[0], b[1], ..., b[n-1]]
    double y = 0;
    for (int i = 0; i < n; ++i) {
      y += b[i] * x[i];
    }

    return y;
  }

  /**
   * @brief Generate step response data for plotting
   * @param t_final Final time
   * @param num_points Number of points
   * @return Pair of time vector and response vector
   */
  std::pair<std::vector<double>, std::vector<double>>
  stepResponseData(double t_final = 10.0, int num_points = 500) const {
    std::vector<double> t(num_points), y(num_points);
    for (int i = 0; i < num_points; ++i) {
      t[i] = t_final * i / (num_points - 1);
      y[i] = stepResponse(t[i]);
    }
    return {t, y};
  }

  // ============ Arithmetic Operations ============

  /// Series connection: G1 * G2
  TransferFunction operator*(const TransferFunction &other) const {
    return TransferFunction(num * other.num, den * other.den);
  }

  TransferFunction operator*(double K) const {
    return TransferFunction(num * K, den);
  }

  /// Parallel connection: G1 + G2
  TransferFunction operator+(const TransferFunction &other) const {
    Polynomial newNum = num * other.den + other.num * den;
    Polynomial newDen = den * other.den;
    return TransferFunction(newNum, newDen);
  }

  /// Negation
  TransferFunction operator-() const {
    return TransferFunction(num * (-1.0), den);
  }

  TransferFunction operator-(const TransferFunction &other) const {
    return *this + (-other);
  }

  /// Division: G1 / G2
  TransferFunction operator/(const TransferFunction &other) const {
    return TransferFunction(num * other.den, den * other.num);
  }

  // ============ String Representation ============

  std::string toString() const {
    std::ostringstream oss;
    std::string numStr = num.toString();
    std::string denStr = den.toString();

    size_t width = std::max(numStr.length(), denStr.length());

    // Center numerator
    size_t numPad = (width - numStr.length()) / 2;
    oss << std::string(numPad, ' ') << numStr << "\n";

    // Fraction line
    oss << std::string(width, '-') << "\n";

    // Center denominator
    size_t denPad = (width - denStr.length()) / 2;
    oss << std::string(denPad, ' ') << denStr;

    return oss.str();
  }

private:
  /// Normalize (remove common factors, ensure den has positive leading coeff)
  void normalize() {
    if (den.leading() < 0) {
      num = num * (-1.0);
      den = den * (-1.0);
    }
  }
};

// ============ Factory Functions ============

/**
 * @brief Create transfer function from zeros, poles, and gain
 * G(s) = K * (s-z1)(s-z2)... / (s-p1)(s-p2)...
 */
inline TransferFunction zpk(const std::vector<std::complex<double>> &zeros,
                            const std::vector<std::complex<double>> &poles,
                            double gain = 1.0) {
  Polynomial num = poly(zeros) * gain;
  Polynomial den = poly(poles);
  return TransferFunction(num, den);
}

/// Overload for real zeros and poles
inline TransferFunction zpk(const std::vector<double> &zeros,
                            const std::vector<double> &poles,
                            double gain = 1.0) {
  Polynomial num = poly(zeros) * gain;
  Polynomial den = poly(poles);
  return TransferFunction(num, den);
}

/**
 * @brief Create transfer function: G(s) = 1/s (integrator)
 */
inline TransferFunction tf_integrator() {
  return TransferFunction({1}, {1, 0});
}

/**
 * @brief Create transfer function: G(s) = s (differentiator)
 */
inline TransferFunction tf_differentiator() {
  return TransferFunction({1, 0}, {1});
}

/**
 * @brief Create first-order system: G(s) = K / (τs + 1)
 */
inline TransferFunction tf_first_order(double K, double tau) {
  return TransferFunction({K}, {tau, 1});
}

/**
 * @brief Create second-order system: G(s) = K*wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
 */
inline TransferFunction tf_second_order(double wn, double zeta,
                                        double K = 1.0) {
  double wn2 = wn * wn;
  return TransferFunction({K * wn2}, {1, 2 * zeta * wn, wn2});
}

/**
 * @brief Feedback connection
 * @param G Forward path
 * @param H Feedback path (default = 1 for unity feedback)
 * @param sign +1 for positive feedback, -1 for negative feedback (default)
 * @return Closed-loop transfer function G/(1+GH) or G/(1-GH)
 */
inline TransferFunction
feedback(const TransferFunction &G,
         const TransferFunction &H = TransferFunction(1.0), int sign = -1) {
  // Closed-loop: G / (1 + G*H) for negative feedback
  //              G / (1 - G*H) for positive feedback
  TransferFunction GH = G * H;

  Polynomial num = G.num * H.den;
  Polynomial den = G.den * H.den;

  if (sign < 0) {
    den = den + G.num * H.num; // 1 + GH
  } else {
    den = den - G.num * H.num; // 1 - GH
  }

  return TransferFunction(num, den);
}

/**
 * @brief Series connection of multiple systems
 */
inline TransferFunction series(const std::vector<TransferFunction> &systems) {
  TransferFunction result(1.0);
  for (const auto &sys : systems) {
    result = result * sys;
  }
  return result;
}

/**
 * @brief Parallel connection of multiple systems
 */
inline TransferFunction parallel(const std::vector<TransferFunction> &systems) {
  if (systems.empty())
    return TransferFunction(0.0);
  TransferFunction result = systems[0];
  for (size_t i = 1; i < systems.size(); ++i) {
    result = result + systems[i];
  }
  return result;
}

// ============ Non-member operators ============

inline TransferFunction operator*(double K, const TransferFunction &G) {
  return G * K;
}

/// Stream output: cout << G prints the transfer function
inline std::ostream &operator<<(std::ostream &os, const TransferFunction &G) {
  return os << G.toString();
}

} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_TRANSFER_FUNCTION_HPP
