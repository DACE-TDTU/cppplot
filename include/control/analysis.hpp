/**
 * @file analysis.hpp
 * @brief Control system analysis functions
 * 
 * Functions: margin, stepinfo, bandwidth, poles, zeros, isstable
 */

#ifndef CPPPLOT_CONTROL_ANALYSIS_HPP
#define CPPPLOT_CONTROL_ANALYSIS_HPP

#include "transfer_function.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace cppplot {
namespace control {

// ============ Margin Analysis ============

/**
 * @struct MarginInfo
 * @brief Gain and phase margin information
 */
struct MarginInfo {
    double Gm;      // Gain margin (linear)
    double Gm_dB;   // Gain margin (dB)
    double Pm;      // Phase margin (degrees)
    double Wgc;     // Gain crossover frequency (rad/s)
    double Wpc;     // Phase crossover frequency (rad/s)
    bool stable;    // Closed-loop stability with unity feedback
};

/**
 * @brief Calculate gain and phase margins
 * @param G Open-loop transfer function
 * @param omega_min Minimum frequency for search
 * @param omega_max Maximum frequency for search
 * @return MarginInfo structure
 */
inline MarginInfo margin(
    const TransferFunction& G,
    double omega_min = 0.001,
    double omega_max = 1000.0
) {
    MarginInfo info;
    info.Gm = std::numeric_limits<double>::infinity();
    info.Gm_dB = std::numeric_limits<double>::infinity();
    info.Pm = 180.0;
    info.Wgc = 0;
    info.Wpc = 0;
    info.stable = true;
    
    // Generate frequency points (logarithmic spacing)
    std::vector<double> omega;
    for (double w = omega_min; w <= omega_max; w *= 1.02) {
        omega.push_back(w);
    }
    
    // Calculate magnitude and phase
    std::vector<double> mag_dB, phase_deg;
    for (double w : omega) {
        mag_dB.push_back(G.mag_dB(w));
        phase_deg.push_back(G.phase_deg(w));
    }
    
    // Improved phase unwrapping algorithm
    // Handles both phase lead and phase lag systems
    // Works by detecting discontinuities > 180° and adjusting
    for (size_t i = 1; i < phase_deg.size(); ++i) {
        double diff = phase_deg[i] - phase_deg[i-1];
        
        // Detect and correct for +360° jumps (phase wrapping from -180 to +180)
        while (diff > 180.0) {
            phase_deg[i] -= 360.0;
            diff = phase_deg[i] - phase_deg[i-1];
        }
        
        // Detect and correct for -360° jumps (phase wrapping from +180 to -180)
        while (diff < -180.0) {
            phase_deg[i] += 360.0;
            diff = phase_deg[i] - phase_deg[i-1];
        }
    }
    
    // For systems starting with positive phase (lead systems), 
    // check if we need to offset the entire phase curve
    // Most control systems have phase starting near 0° or negative
    // If starting phase is large positive, it's likely a lead system
    if (phase_deg[0] > 90.0) {
        // Possibly a differentiator or lead network at low frequency
        // Shift to start from expected range
        double offset = std::round(phase_deg[0] / 360.0) * 360.0;
        for (size_t i = 0; i < phase_deg.size(); ++i) {
            phase_deg[i] -= offset;
        }
    }
    
    // Find gain crossover (|G| = 0 dB) - may have multiple crossovers
    std::vector<double> gain_crossovers;
    std::vector<double> phase_at_gc;
    
    for (size_t i = 1; i < omega.size(); ++i) {
        if ((mag_dB[i-1] > 0 && mag_dB[i] <= 0) ||
            (mag_dB[i-1] >= 0 && mag_dB[i] < 0)) {
            // Linear interpolation for gain crossover frequency
            double w_gc = omega[i-1] + (0 - mag_dB[i-1]) * 
                         (omega[i] - omega[i-1]) / (mag_dB[i] - mag_dB[i-1]);
            
            // Phase at gain crossover
            double phase_gc = phase_deg[i-1] + (w_gc - omega[i-1]) * 
                             (phase_deg[i] - phase_deg[i-1]) / (omega[i] - omega[i-1]);
            
            gain_crossovers.push_back(w_gc);
            phase_at_gc.push_back(phase_gc);
        }
    }
    
    // Use the first (lowest frequency) gain crossover for primary margin
    if (!gain_crossovers.empty()) {
        info.Wgc = gain_crossovers[0];
        info.Pm = 180.0 + phase_at_gc[0];
    }
    
    // Find phase crossover (phase = -180°) - may have multiple crossovers
    std::vector<double> phase_crossovers;
    std::vector<double> mag_at_pc;
    
    for (size_t i = 1; i < omega.size(); ++i) {
        if ((phase_deg[i-1] > -180 && phase_deg[i] <= -180) ||
            (phase_deg[i-1] >= -180 && phase_deg[i] < -180)) {
            // Linear interpolation for phase crossover frequency
            double w_pc = omega[i-1] + (-180 - phase_deg[i-1]) * 
                         (omega[i] - omega[i-1]) / (phase_deg[i] - phase_deg[i-1]);
            
            // Magnitude at phase crossover
            double mag_pc = mag_dB[i-1] + (w_pc - omega[i-1]) * 
                           (mag_dB[i] - mag_dB[i-1]) / (omega[i] - omega[i-1]);
            
            phase_crossovers.push_back(w_pc);
            mag_at_pc.push_back(mag_pc);
        }
    }
    
    // Use the first (lowest frequency) phase crossover for primary margin
    if (!phase_crossovers.empty()) {
        info.Wpc = phase_crossovers[0];
        info.Gm_dB = -mag_at_pc[0];
        info.Gm = std::pow(10, info.Gm_dB / 20.0);
    }
    
    // Determine stability: both margins must be positive
    info.stable = (info.Gm > 1.0) && (info.Pm > 0);
    
    return info;
}

// ============ Step Response Analysis ============

/**
 * @struct StepInfo
 * @brief Step response characteristics
 */
struct StepInfo {
    double RiseTime;        // Time from 10% to 90% of final value
    double SettlingTime;    // Time to reach and stay within 2% of final value
    double Overshoot;       // Peak overshoot (percent)
    double Peak;            // Peak value
    double PeakTime;        // Time of peak
    double SteadyState;     // Final value
    double Undershoot;      // Maximum undershoot (percent)
};

/**
 * @brief Analyze step response characteristics
 * @param G Transfer function
 * @param t_final Final time for simulation
 * @param num_points Number of time points
 * @return StepInfo structure
 */
inline StepInfo stepinfo(
    const TransferFunction& G,
    double t_final = 0,  // 0 = auto
    int num_points = 1000
) {
    StepInfo info = {0, 0, 0, 0, 0, 1, 0};
    
    // Auto-determine simulation time based on dominant pole
    if (t_final <= 0) {
        auto p = G.poles();
        double dominant_re = -1;  // Default
        for (const auto& pole : p) {
            if (pole.real() < 0 && pole.real() > dominant_re) {
                dominant_re = pole.real();
            }
        }
        t_final = -5.0 / dominant_re;  // 5 time constants
        if (t_final > 100) t_final = 100;
        if (t_final < 1) t_final = 10;
    }
    
    // Get steady-state value (DC gain)
    info.SteadyState = G.dcgain();
    if (!std::isfinite(info.SteadyState)) {
        info.SteadyState = 1.0;  // Assume normalized
    }
    
    // Generate time vector
    std::vector<double> t(num_points);
    std::vector<double> y(num_points);
    double dt = t_final / (num_points - 1);
    
    for (int i = 0; i < num_points; ++i) {
        t[i] = i * dt;
        try {
            y[i] = G.stepResponse(t[i]);
        } catch (...) {
            // If stepResponse fails, use numerical approximation
            y[i] = info.SteadyState;  // Placeholder
        }
    }
    
    // Find peak
    double y_max = y[0];
    int i_max = 0;
    for (int i = 1; i < num_points; ++i) {
        if (y[i] > y_max) {
            y_max = y[i];
            i_max = i;
        }
    }
    info.Peak = y_max;
    info.PeakTime = t[i_max];
    
    // Overshoot (relative to steady state)
    if (info.SteadyState > 0) {
        info.Overshoot = std::max(0.0, (y_max - info.SteadyState) / info.SteadyState * 100);
    }
    
    // Rise time (10% to 90%)
    double y_10 = 0.1 * info.SteadyState;
    double y_90 = 0.9 * info.SteadyState;
    double t_10 = 0, t_90 = 0;
    
    for (int i = 1; i < num_points; ++i) {
        if (t_10 == 0 && y[i-1] < y_10 && y[i] >= y_10) {
            t_10 = t[i-1] + (y_10 - y[i-1]) * dt / (y[i] - y[i-1]);
        }
        if (t_90 == 0 && y[i-1] < y_90 && y[i] >= y_90) {
            t_90 = t[i-1] + (y_90 - y[i-1]) * dt / (y[i] - y[i-1]);
            break;
        }
    }
    info.RiseTime = t_90 - t_10;
    
    // Settling time (2% band)
    double tolerance = 0.02 * std::abs(info.SteadyState);
    double t_settle = t_final;
    
    for (int i = num_points - 1; i >= 0; --i) {
        if (std::abs(y[i] - info.SteadyState) > tolerance) {
            t_settle = (i < num_points - 1) ? t[i + 1] : t[i];
            break;
        }
    }
    info.SettlingTime = t_settle;
    
    // Undershoot
    double y_min = y[0];
    for (int i = 1; i < num_points; ++i) {
        if (y[i] < y_min) y_min = y[i];
    }
    if (info.SteadyState > 0 && y_min < 0) {
        info.Undershoot = -y_min / info.SteadyState * 100;
    }
    
    return info;
}

// ============ Bandwidth ============

/**
 * @brief Calculate -3dB bandwidth
 * @param G Transfer function
 * @param omega_min Minimum frequency for search
 * @param omega_max Maximum frequency for search
 * @return Bandwidth in rad/s (0 if not found)
 */
inline double bandwidth(
    const TransferFunction& G,
    double omega_min = 0.001,
    double omega_max = 10000.0
) {
    // Reference magnitude at DC or low frequency
    double mag_ref = G.mag(omega_min);
    double mag_3dB = mag_ref / std::sqrt(2);  // -3dB point
    
    // Search for bandwidth
    double w = omega_min;
    double prev_mag = mag_ref;
    
    while (w <= omega_max) {
        double curr_mag = G.mag(w);
        
        if (prev_mag > mag_3dB && curr_mag <= mag_3dB) {
            // Linear interpolation
            double w_prev = w / 1.02;
            return w_prev + (mag_3dB - prev_mag) * (w - w_prev) / (curr_mag - prev_mag);
        }
        
        prev_mag = curr_mag;
        w *= 1.02;
    }
    
    return 0;  // Not found (system may be all-pass or have no roll-off)
}

// ============ Utility Functions ============

/**
 * @brief Check if system is stable
 */
inline bool isstable(const TransferFunction& G) {
    return G.isStable();
}

/**
 * @brief Get poles of system
 */
inline std::vector<std::complex<double>> poles(const TransferFunction& G) {
    return G.poles();
}

/**
 * @brief Get zeros of system
 */
inline std::vector<std::complex<double>> zeros(const TransferFunction& G) {
    return G.zeros();
}

/**
 * @brief Get DC gain
 */
inline double dcgain(const TransferFunction& G) {
    return G.dcgain();
}

// ============ Inverse Laplace (Rational, Simple Poles) ============

/**
 * @brief Check if all poles are simple (no repeats within tolerance)
 */
inline bool has_simple_poles(const std::vector<std::complex<double>>& poles, double tol = 1e-8) {
    for (size_t i = 0; i < poles.size(); ++i) {
        for (size_t j = i + 1; j < poles.size(); ++j) {
            if (std::abs(poles[i] - poles[j]) < tol) return false;
        }
    }
    return true;
}

/**
 * @brief Derivative of a polynomial
 */
inline Polynomial poly_derivative(const Polynomial& p) {
    int deg = p.degree();
    if (deg <= 0) return Polynomial({0});
    std::vector<double> d(deg);
    for (int i = 0; i < deg; ++i) {
        // coeffs are in descending order; power = deg - i
        int power = deg - i;
        d[i] = p.coeffs[i] * power;
    }
    return Polynomial(d);
}

/**
 * @brief Compute simple residues r_k and poles p_k for proper N(s)/D(s)
 *        r_k = N(p_k) / D'(p_k), assuming simple poles
 */
inline std::vector<std::pair<std::complex<double>, std::complex<double>>>
residues_simple(const Polynomial& num, const Polynomial& den) {
    std::vector<std::pair<std::complex<double>, std::complex<double>>> result;
    auto poles = den.roots();
    auto dprime = poly_derivative(den);
    for (const auto& p : poles) {
        std::complex<double> r = num(p) / dprime(p);
        result.push_back({r, p});
    }
    return result;
}

/**
 * @brief Evaluate inverse Laplace of a proper rational G(s)=N(s)/D(s) at time t (impulse response)
 *        Uses simple-pole residue expansion. Returns NaN if conditions not met.
 */
inline double inverse_laplace_impulse(const TransferFunction& G, double t) {
    if (t < 0) return 0.0;
    if (!G.isProper()) return std::numeric_limits<double>::quiet_NaN();
    auto p = G.poles();
    if (p.empty()) return 0.0;
    if (!has_simple_poles(p)) return std::numeric_limits<double>::quiet_NaN();

    // Properize numerator by removing polynomial part (which maps to delta terms at t=0)
    Polynomial q = G.num / G.den;             // quotient (ignored for t>0)
    Polynomial r = G.num - (q * G.den);       // remainder (proper part)

    auto res = residues_simple(r, G.den);
    std::complex<double> sum(0.0, 0.0);
    for (const auto& rp : res) {
        const auto& R = rp.first;
        const auto& pole = rp.second;
        sum += R * std::exp(pole * t);
    }
    return sum.real();  // result is real for real-coefficient systems
}

/**
 * @brief Evaluate inverse Laplace of G(s)/s at time t (step response) using residues
 *        Returns NaN if conditions not met.
 */
inline double inverse_laplace_step(const TransferFunction& G, double t) {
    if (t < 0) return 0.0;
    // Build H(s) = G(s)/s by multiplying denominator with s
    Polynomial den_s = G.den * Polynomial({1, 0});
    TransferFunction H(G.num, den_s);
    auto p = H.poles();
    if (!H.isProper() || p.empty() || !has_simple_poles(p)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    // No quotient (H is strictly proper when G is proper)
    auto res = residues_simple(H.num, H.den);
    std::complex<double> sum(0.0, 0.0);
    for (const auto& rp : res) {
        sum += rp.first * std::exp(rp.second * t);
    }
    return sum.real();
}

// ============ Damping Analysis ============

/**
 * @struct PoleInfo
 * @brief Information about a pole
 */
struct PoleInfo {
    std::complex<double> value;
    double damping;         // Damping ratio (zeta)
    double naturalFreq;     // Natural frequency (wn)
    double timeConstant;    // Time constant (tau = -1/Re(p))
};

/**
 * @brief Get damping info for all poles
 */
inline std::vector<PoleInfo> damp(const TransferFunction& G) {
    auto p = G.poles();
    std::vector<PoleInfo> info;
    
    for (const auto& pole : p) {
        PoleInfo pi;
        pi.value = pole;
        
        double sigma = pole.real();
        double omega = std::abs(pole.imag());
        double wn = std::abs(pole);
        
        pi.naturalFreq = wn;
        pi.damping = (wn > 1e-10) ? -sigma / wn : 1.0;
        pi.timeConstant = (std::abs(sigma) > 1e-10) ? -1.0 / sigma : std::numeric_limits<double>::infinity();
        
        info.push_back(pi);
    }
    
    return info;
}

} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_ANALYSIS_HPP
