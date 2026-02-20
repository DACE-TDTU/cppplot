/**
 * @file control.hpp
 * @brief CppPlot Control Systems Module
 *
 * A comprehensive control systems library following Python Control / MATLAB
 * standards.
 *
 * @version 2.0.0
 * @date 2026
 *
 * Features:
 * - Transfer function representation and arithmetic
 * - Frequency response analysis (Bode, Nyquist, Nichols)
 * - Time domain analysis (step, impulse, lsim)
 * - Stability analysis (poles, zeros, margins)
 * - Root locus plotting, pole-zero maps
 * - State-space representation and conversion
 * - LQR/LQG design, Kalman filter, MPC
 * - Robust control: H∞, μ-analysis
 * - Nonlinear control: Sliding Mode (incl. Super-Twisting, ISS, DOBSMC)
 * - Nonlinear analysis: Phase portrait, Lyapunov, ISS, CBF
 * - Nonlinear design: Feedback Linearization, Backstepping, Observers
 * - Adaptive control: RLS, MRAC (σ-mod, deadzone), NN Adaptive
 *
 * @example Basic Usage
 * @code
 * #include <cppplot/control/control.hpp>
 * using namespace cppplot::control;
 *
 * // Create a second-order system
 * auto G = tf_second_order(10.0, 0.3);  // wn=10, zeta=0.3
 *
 * // One-liner plots
 * bode(G);
 * savefig("bode.svg");
 *
 * step(G);
 * savefig("step.svg");
 *
 * nyquist(G);
 * savefig("nyquist.svg");
 *
 * pzmap(G);
 * savefig("pzmap.svg");
 * @endcode
 *
 * @example Transfer Function Operations
 * @code
 * // Create from coefficients: G(s) = (s+1)/(s^2+2s+1)
 * TransferFunction G({1, 1}, {1, 2, 1});
 *
 * // Create from zeros, poles, gain
 * auto G2 = zpk({-1}, {-1, -2}, 5.0);
 *
 * // Connections
 * auto G_series = G * G2;           // Series
 * auto G_parallel = G + G2;         // Parallel
 * auto G_closed = feedback(G, 1.0); // Unity feedback
 *
 * // Analysis
 * auto m = margin(G);               // Gain/phase margins
 * auto info = stepinfo(G);          // Rise time, settling, etc.
 * bool stable = isstable(G);        // Stability check
 * @endcode
 */

#ifndef CPPPLOT_CONTROL_HPP
#define CPPPLOT_CONTROL_HPP

#include <iostream>

// Core components
#include "analysis.hpp"
#include "polynomial.hpp"
#include "transfer_function.hpp"


// State-space representation
#include "state_space.hpp"

// Controller design
#include "block_diagram.hpp"
#include "controller_design.hpp"


// State estimation
#include "kalman.hpp"
#include "lqg.hpp"
#include "mpc.hpp"

// Robust control
#include "robust/hinf.hpp"
#include "robust/mu_analysis.hpp"
#include "robust/uncertainty.hpp"


// Nonlinear control — analysis
#include "nonlinear/iss.hpp"            // ISS gain, small-gain theorem
#include "nonlinear/lyapunov.hpp"       // CLF, ROA, Lyapunov equation
#include "nonlinear/phase_portrait.hpp" // Phase portrait, equilibria, limit cycles
#include "nonlinear/sliding_mode.hpp" // SMC variants, Super-Twisting, DOBSMC


// Nonlinear control — design
#include "nonlinear/backstepping.hpp" // Recursive backstepping, command-filtered
#include "nonlinear/cbf.hpp"          // CBF-QP filter, CLF+CBF, multi-CBF
#include "nonlinear/feedback_linearization.hpp" // Lie derivatives, FBL, zero dynamics
#include "nonlinear/observers.hpp" // HGO, ESO/ADRC, SMO, Luenberger


// Adaptive control
#include "adaptive/adaptive_nonlinear.hpp" // Slotine-Li, RBF-NN, CE principle
#include "adaptive/mrac.hpp"               // MRAC variants, adaptive PID
#include "adaptive/rls.hpp" // RLS estimation, projection, PE index


// Plotting functions
#include "bode.hpp"
#include "nichols.hpp"
#include "nyquist.hpp"
#include "pzmap.hpp"
#include "root_locus.hpp"
#include "time_response.hpp"


// Discrete-time support
#include "discrete.hpp"

// System identification
#include "sysid.hpp"

namespace cppplot {
namespace control {

/**
 * @brief Print system information to console
 */
inline void sysinfo(const TransferFunction &G) {
  std::cout << "=============== System Information ==============="
            << std::endl;
  std::cout << "\nTransfer Function:" << std::endl;
  std::cout << G.toString() << std::endl;

  std::cout << "\nProperties:" << std::endl;
  std::cout << "  Order: " << G.order() << std::endl;
  std::cout << "  DC Gain: " << G.dcgain() << std::endl;
  std::cout << "  Proper: " << (G.isProper() ? "Yes" : "No") << std::endl;
  std::cout << "  Stable: " << (G.isStable() ? "Yes" : "No") << std::endl;

  // Poles
  auto p = G.poles();
  std::cout << "\nPoles:" << std::endl;
  for (size_t i = 0; i < p.size(); ++i) {
    std::cout << "  p" << (i + 1) << " = " << p[i].real();
    if (std::abs(p[i].imag()) > 1e-10) {
      std::cout << (p[i].imag() >= 0 ? " + " : " - ") << std::abs(p[i].imag())
                << "j";
    }
    std::cout << std::endl;
  }

  // Zeros
  auto z = G.zeros();
  if (!z.empty()) {
    std::cout << "\nZeros:" << std::endl;
    for (size_t i = 0; i < z.size(); ++i) {
      std::cout << "  z" << (i + 1) << " = " << z[i].real();
      if (std::abs(z[i].imag()) > 1e-10) {
        std::cout << (z[i].imag() >= 0 ? " + " : " - ") << std::abs(z[i].imag())
                  << "j";
      }
      std::cout << std::endl;
    }
  }

  // Margins (if system appears to be open-loop)
  auto m = margin(G);
  std::cout << "\nStability Margins:" << std::endl;
  std::cout << "  Gain Margin: " << m.Gm_dB << " dB at " << m.Wpc << " rad/s"
            << std::endl;
  std::cout << "  Phase Margin: " << m.Pm << " deg at " << m.Wgc << " rad/s"
            << std::endl;

  std::cout << "=================================================" << std::endl;
}

/**
 * @brief Create standard test systems
 */
namespace systems {

/// First-order lag: G(s) = K / (τs + 1)
inline TransferFunction first_order(double K = 1.0, double tau = 1.0) {
  return tf_first_order(K, tau);
}

/// Second-order with damping: G(s) = Kωn² / (s² + 2ζωns + ωn²)
inline TransferFunction second_order(double wn = 1.0, double zeta = 0.5,
                                     double K = 1.0) {
  return tf_second_order(wn, zeta, K);
}

/// Integrator: G(s) = K/s
inline TransferFunction integrator(double K = 1.0) {
  return TransferFunction({K}, {1, 0});
}

/// Double integrator: G(s) = K/s²
inline TransferFunction double_integrator(double K = 1.0) {
  return TransferFunction({K}, {1, 0, 0});
}

/// Type 1 system: G(s) = K / (s(s+a))
inline TransferFunction type1(double K = 1.0, double a = 1.0) {
  return TransferFunction({K}, {1, a, 0});
}

/// Type 1 with two poles: G(s) = K / (s(s+a)(s+b))
inline TransferFunction type1_3pole(double K = 1.0, double a = 1.0,
                                    double b = 2.0) {
  // (s+a)(s+b) = s² + (a+b)s + ab
  return TransferFunction({K}, {1, a + b, a * b, 0});
}

/// Lead compensator: G(s) = K(s+z)/(s+p) where z < p
inline TransferFunction lead(double z, double p, double K = 1.0) {
  return TransferFunction({K, K * z}, {1, p});
}

/// Lag compensator: G(s) = K(s+z)/(s+p) where z > p
inline TransferFunction lag(double z, double p, double K = 1.0) {
  return TransferFunction({K, K * z}, {1, p});
}

/// PID controller: G(s) = Kp + Ki/s + Kd*s
inline TransferFunction pid(double Kp, double Ki, double Kd) {
  // (Kd*s² + Kp*s + Ki) / s
  return TransferFunction({Kd, Kp, Ki}, {1, 0});
}

/// PI controller: G(s) = Kp + Ki/s
inline TransferFunction pi(double Kp, double Ki) {
  return TransferFunction({Kp, Ki}, {1, 0});
}

/// PD controller: G(s) = Kp + Kd*s
inline TransferFunction pd(double Kp, double Kd) {
  return TransferFunction({Kd, Kp}, {1});
}

} // namespace systems

} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_HPP
