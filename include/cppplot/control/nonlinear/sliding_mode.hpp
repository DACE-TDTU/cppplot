/**
 * @file nonlinear/sliding_mode.hpp
 * @brief Sliding Mode Control (SMC) Module
 *
 * Comprehensive implementation of Sliding Mode Control:
 * - Conventional Sliding Mode Control
 * - Super-Twisting Algorithm (STA)
 * - Integral Sliding Mode Control (ISMC)
 * - Higher-Order Sliding Mode (HOSM)
 * - Fixed-Time SMC (FxTSMC)
 * - Event-Triggered SMC (ETSMC)
 * - Barrier Function-based SMC
 * - Disturbance Observer-based SMC (DOBSMC)
 * - Chattering Reduction Techniques
 *
 * Key concepts:
 * - Sliding surface design: s(x) = 0
 * - Reaching condition: s·ṡ < 0
 * - Equivalent control: keeps system on surface
 * - Switching control: drives system to surface
 *
 * References:
 * - Utkin (1977) "Variable Structure Systems with Sliding Modes"
 * - Levant (1993) "Sliding order and sliding accuracy"
 * - Edwards & Spurgeon (1998) "Sliding Mode Control"
 * - Shtessel et al. (2014) "Sliding Mode Control and Observation"
 * - Polyakov (2012) "Nonlinear feedback design for fixed-time stabilization"
 * - Basin et al. (2017) "Continuous fixed-time controller design"
 * - Heemels & Donkers (2012) "Event-triggered control"
 * - Tee et al. (2009) "Barrier Lyapunov functions"
 */

#ifndef CPPPLOT_CONTROL_NONLINEAR_SLIDING_MODE_HPP
#define CPPPLOT_CONTROL_NONLINEAR_SLIDING_MODE_HPP

#include "../state_space.hpp"
#include "../transfer_function.hpp"
#include "../../pyplot.hpp"
#include <vector>
#include <complex>
#include <cmath>
#include <functional>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <iomanip>

namespace cppplot {
namespace control {
namespace nonlinear {

// ============================================================
//                  SLIDING SURFACE TYPES
// ============================================================

/**
 * @brief Types of sliding surfaces
 */
enum class SurfaceType {
    LINEAR,         ///< s = Cx (linear combination of states)
    INTEGRAL,       ///< s = Cx + ∫e dt (integral action)
    TERMINAL,       ///< s = ẋ + βx^(q/p) (terminal sliding mode)
    PID_LIKE        ///< s = Cx + Ki∫e + Kd·ė (PID-like surface)
};

/**
 * @brief Types of reaching laws
 */
enum class ReachingLaw {
    CONSTANT_RATE,      ///< ṡ = -K·sign(s)
    CONSTANT_PLUS_PROP, ///< ṡ = -K·sign(s) - λs
    POWER_RATE,         ///< ṡ = -K|s|^α·sign(s)
    EXPONENTIAL         ///< ṡ = -K·sign(s)·exp(-β|s|)
};

/**
 * @brief Types of SMC controllers
 */
enum class SMCType {
    CONVENTIONAL,       ///< Basic sign(s) switching
    SUPER_TWISTING,     ///< Second-order STA
    INTEGRAL,           ///< Integral SMC
    QUASI_CONTINUOUS,   ///< Quasi-continuous HOSM
    PRESCRIBED_TIME,    ///< Prescribed-time convergence
    FIXED_TIME,         ///< Fixed-time convergence (independent of initial conditions)
    EVENT_TRIGGERED,    ///< Event-triggered SMC (resource-efficient)
    BARRIER_FUNCTION,   ///< Barrier function-based (state constraints)
    DISTURBANCE_OBSERVER ///< Disturbance observer-based SMC
};

// ============================================================
//                  CONFIGURATION STRUCTURES
// ============================================================

/**
 * @brief Configuration for sliding surface
 */
struct SlidingSurfaceConfig {
    SurfaceType type = SurfaceType::LINEAR;
    std::vector<double> C;              ///< Surface coefficients [c1, c2, ..., cn-1, 1]
    double integral_gain = 0.0;         ///< Ki for integral surface
    double derivative_gain = 0.0;       ///< Kd for PID-like surface
    double terminal_beta = 1.0;         ///< β for terminal sliding mode
    double terminal_p = 5.0;            ///< p for terminal (odd integer)
    double terminal_q = 3.0;            ///< q for terminal (odd integer, q < p)
    
    /// Create linear surface s = c1·x1 + c2·x2 + ... + xn
    static SlidingSurfaceConfig linear(const std::vector<double>& coeffs) {
        SlidingSurfaceConfig cfg;
        cfg.type = SurfaceType::LINEAR;
        cfg.C = coeffs;
        return cfg;
    }
    
    /// Create integral surface s = Cx + Ki·∫e
    static SlidingSurfaceConfig integral(const std::vector<double>& coeffs, double Ki) {
        SlidingSurfaceConfig cfg;
        cfg.type = SurfaceType::INTEGRAL;
        cfg.C = coeffs;
        cfg.integral_gain = Ki;
        return cfg;
    }
    
    /// Create PID-like surface
    static SlidingSurfaceConfig pidLike(double Kp, double Ki, double Kd) {
        SlidingSurfaceConfig cfg;
        cfg.type = SurfaceType::PID_LIKE;
        cfg.C = {Kp};
        cfg.integral_gain = Ki;
        cfg.derivative_gain = Kd;
        return cfg;
    }
};

/**
 * @brief Configuration for SMC controller
 */
struct SMCConfig {
    SMCType type = SMCType::CONVENTIONAL;
    ReachingLaw reaching_law = ReachingLaw::CONSTANT_RATE;
    
    double K = 10.0;                    ///< Switching gain
    double lambda = 5.0;                ///< Proportional gain (for constant+prop)
    double eta = 0.1;                   ///< Reaching margin (ṡ·s < -η|s|)
    double alpha = 0.5;                 ///< Power for power rate reaching
    
    // Chattering reduction
    bool use_boundary_layer = false;    ///< Use saturation instead of sign
    double boundary_thickness = 0.1;    ///< φ: boundary layer thickness
    
    // Super-twisting parameters
    double sta_alpha = 1.5;             ///< STA gain α (typically 1.1-1.5×√L)
    double sta_beta = 1.1;              ///< STA gain β (typically 1.1-1.5×L)
    
    // Control limits
    double u_min = -100.0;
    double u_max = 100.0;
    
    // Fixed-time SMC parameters (Polyakov, 2012)
    double fxt_p = 0.5;                 ///< p parameter (0 < p < 1)
    double fxt_q = 1.5;                 ///< q parameter (q > 1)
    double fxt_k1 = 5.0;                ///< Gain for |s|^p term
    double fxt_k2 = 5.0;                ///< Gain for |s|^q term
    
    // Event-triggered SMC parameters
    double et_threshold = 0.1;          ///< Event trigger threshold
    double et_sigma = 0.5;              ///< Relative threshold (0 < σ < 1)
    double et_min_inter_event = 0.001;  ///< Minimum inter-event time (Zeno prevention)
    
    // Barrier function parameters
    double bf_kb = 1.0;                 ///< Barrier gain
    double bf_state_bound = 10.0;       ///< State constraint |x| < bound
    double bf_barrier_type = 0;         ///< 0: log barrier, 1: reciprocal barrier
    
    // Disturbance observer parameters
    double dob_gain = 50.0;             ///< Observer gain
    double dob_filter_freq = 100.0;     ///< Low-pass filter cutoff frequency
    
    /// Apply control saturation
    double saturate(double u) const {
        return std::max(u_min, std::min(u_max, u));
    }
};

/**
 * @brief Result of SMC simulation
 */
struct SMCSimulationResult {
    std::vector<double> time;
    std::vector<std::vector<double>> states;    ///< State trajectories
    std::vector<double> sliding_surface;        ///< s(t)
    std::vector<double> control;                ///< u(t)
    std::vector<double> equivalent_control;     ///< u_eq(t)
    std::vector<double> switching_control;      ///< u_sw(t)
    std::vector<double> reference;              ///< r(t)
    
    double reaching_time = 0.0;                 ///< Time to reach sliding surface
    double settling_time = 0.0;                 ///< Settling time
    double max_chattering = 0.0;                ///< Maximum control oscillation
    bool reached_surface = false;
    
    /// Get state trajectory for state i
    std::vector<double> getState(size_t i) const {
        std::vector<double> xi;
        for (const auto& x : states) {
            xi.push_back(x[i]);
        }
        return xi;
    }
};

// ============================================================
//                  SLIDING SURFACE CLASS
// ============================================================

/**
 * @brief Sliding Surface for SMC design
 */
class SlidingSurface {
public:
    SlidingSurfaceConfig config_;
    size_t n_states_;
    double integral_state_ = 0.0;
    
    SlidingSurface(const SlidingSurfaceConfig& cfg, size_t n_states)
        : config_(cfg), n_states_(n_states) {
        // Ensure C has correct size
        if (config_.C.size() < n_states_) {
            config_.C.resize(n_states_, 0.0);
            config_.C.back() = 1.0;
        }
    }
    
    /// Compute sliding variable s
    double compute(const std::vector<double>& x, double error = 0.0) {
        double s = 0.0;
        
        switch (config_.type) {
            case SurfaceType::LINEAR:
                for (size_t i = 0; i < std::min(x.size(), config_.C.size()); ++i) {
                    s += config_.C[i] * x[i];
                }
                break;
                
            case SurfaceType::INTEGRAL:
                for (size_t i = 0; i < std::min(x.size(), config_.C.size()); ++i) {
                    s += config_.C[i] * x[i];
                }
                s += config_.integral_gain * integral_state_;
                break;
                
            case SurfaceType::TERMINAL:
                if (x.size() >= 2) {
                    double x1 = x[0];
                    double x2 = x[1];
                    double qp = config_.terminal_q / config_.terminal_p;
                    s = x2 + config_.terminal_beta * std::pow(std::abs(x1), qp) * sign(x1);
                }
                break;
                
            case SurfaceType::PID_LIKE:
                if (!x.empty()) {
                    s = config_.C[0] * error;
                    s += config_.integral_gain * integral_state_;
                    if (x.size() >= 2) {
                        s += config_.derivative_gain * x[1];  // ė ≈ x2 for tracking
                    }
                }
                break;
        }
        
        return s;
    }
    
    /// Update integral state
    void updateIntegral(double error, double dt) {
        integral_state_ += error * dt;
    }
    
    /// Reset integral state
    void reset() {
        integral_state_ = 0.0;
    }
    
    /// Sign function with deadzone
    static double sign(double x, double tol = 1e-10) {
        if (x > tol) return 1.0;
        if (x < -tol) return -1.0;
        return 0.0;
    }
    
    /// Saturation function (for boundary layer)
    static double sat(double x, double phi = 1.0) {
        if (std::abs(x) <= phi) {
            return x / phi;
        }
        return sign(x);
    }
};

// ============================================================
//              SLIDING MODE CONTROLLER CLASS
// ============================================================

/**
 * @brief Sliding Mode Controller
 *
 * Implements various SMC algorithms for second-order systems:
 *   ẋ₁ = x₂
 *   ẋ₂ = f(x) + g(x)·u + d(t)
 *
 * where f(x) is known dynamics, g(x) is control gain, d(t) is disturbance
 */
class SlidingModeController {
public:
    using DynamicsFunc = std::function<double(const std::vector<double>&)>;
    using DisturbanceFunc = std::function<double(double t)>;
    
private:
    SMCConfig config_;
    SlidingSurface surface_;
    
    // System model (for equivalent control computation)
    DynamicsFunc f_;      // f(x): drift term
    DynamicsFunc g_;      // g(x): control effectiveness (usually constant > 0)
    double D_max_ = 0.0;  // Upper bound on |d(t)|
    
    // Internal states for super-twisting
    double sta_integral_ = 0.0;
    
    // Internal states for event-triggered SMC
    double last_control_ = 0.0;
    double last_trigger_time_ = 0.0;
    double current_time_ = 0.0;
    bool event_triggered_ = true;
    
    // Internal states for disturbance observer
    double dob_estimated_disturbance_ = 0.0;
    double dob_aux_state_ = 0.0;
    double dob_filtered_disturbance_ = 0.0;  // Low-pass filtered estimate
    
    // Prescribed-time elapsed
    double prescribed_time_elapsed_ = 0.0;
    
public:
    SlidingModeController(
        const SMCConfig& config,
        const SlidingSurfaceConfig& surface_config,
        size_t n_states = 2
    ) : config_(config), surface_(surface_config, n_states) {
        // Default dynamics: ẋ₂ = u (double integrator)
        f_ = [](const std::vector<double>&) { return 0.0; };
        g_ = [](const std::vector<double>&) { return 1.0; };
    }
    
    /// Set system dynamics f(x) and g(x) for ẋ₂ = f(x) + g(x)·u
    void setDynamics(DynamicsFunc f, DynamicsFunc g, double D_max = 0.0) {
        f_ = f;
        g_ = g;
        D_max_ = D_max;
    }
    
    /// Compute control signal
    double compute(const std::vector<double>& x, double reference = 0.0, double dt = 0.01) {
        double error = reference - x[0];
        double s = surface_.compute(x, error);
        
        // Update integral state if needed
        if (surface_.config_.type == SurfaceType::INTEGRAL ||
            surface_.config_.type == SurfaceType::PID_LIKE) {
            surface_.updateIntegral(error, dt);
        }
        
        double u = 0.0;
        
        switch (config_.type) {
            case SMCType::CONVENTIONAL:
                u = computeConventional(x, s);
                break;
            case SMCType::SUPER_TWISTING:
                u = computeSuperTwisting(x, s, dt);
                break;
            case SMCType::INTEGRAL:
                u = computeIntegralSMC(x, s, error);
                break;
            case SMCType::QUASI_CONTINUOUS:
                u = computeQuasiContinuous(x, s);
                break;
            case SMCType::PRESCRIBED_TIME:
                u = computePrescribedTime(x, s, dt);
                break;
            case SMCType::FIXED_TIME:
                u = computeFixedTime(x, s);
                break;
            case SMCType::EVENT_TRIGGERED:
                u = computeEventTriggered(x, s, dt);
                break;
            case SMCType::BARRIER_FUNCTION:
                u = computeBarrierFunction(x, s);
                break;
            case SMCType::DISTURBANCE_OBSERVER:
                u = computeDisturbanceObserver(x, s, dt);
                break;
        }
        
        return config_.saturate(u);
    }
    
    /// Get current sliding surface value
    double getSlidingSurface(const std::vector<double>& x, double error = 0.0) {
        return surface_.compute(x, error);
    }
    
    /// Reset controller states
    void reset() {
        surface_.reset();
        sta_integral_ = 0.0;
        last_control_ = 0.0;
        last_trigger_time_ = 0.0;
        current_time_ = 0.0;
        event_triggered_ = true;
        dob_estimated_disturbance_ = 0.0;
        dob_aux_state_ = 0.0;
        dob_filtered_disturbance_ = 0.0;
        prescribed_time_elapsed_ = 0.0;
    }
    
    /// Get estimated disturbance (for DOBSMC)
    double getEstimatedDisturbance() const {
        return dob_estimated_disturbance_;
    }
    
    /// Check if event was triggered (for ETSMC)
    bool wasEventTriggered() const {
        return event_triggered_;
    }
    
private:
    /**
     * @brief Conventional SMC: u = u_eq + u_sw
     * 
     * u_eq = -f(x)/g(x)  (equivalent control)
     * u_sw = -(K + η)/g(x) · switch(s)  (switching control)
     */
    double computeConventional(const std::vector<double>& x, double s) {
        double gx = g_(x);
        if (std::abs(gx) < 1e-10) gx = 1.0;  // Avoid division by zero
        
        double fx = f_(x);
        
        // Equivalent control
        double u_eq = -fx / gx;
        
        // Switching control based on reaching law
        double u_sw = 0.0;
        double switch_term = config_.use_boundary_layer ? 
                            SlidingSurface::sat(s, config_.boundary_thickness) :
                            SlidingSurface::sign(s);
        
        switch (config_.reaching_law) {
            case ReachingLaw::CONSTANT_RATE:
                // ṡ = -K·sign(s)
                u_sw = -(config_.K + config_.eta + D_max_) * switch_term / gx;
                break;
                
            case ReachingLaw::CONSTANT_PLUS_PROP:
                // ṡ = -K·sign(s) - λs
                u_sw = -(config_.K * switch_term + config_.lambda * s) / gx;
                break;
                
            case ReachingLaw::POWER_RATE:
                // ṡ = -K|s|^α·sign(s)
                u_sw = -config_.K * std::pow(std::abs(s), config_.alpha) * switch_term / gx;
                break;
                
            case ReachingLaw::EXPONENTIAL:
                // ṡ = -K·sign(s)·exp(-β|s|)
                u_sw = -config_.K * switch_term * std::exp(-config_.lambda * std::abs(s)) / gx;
                break;
        }
        
        return u_eq + u_sw;
    }
    
    /**
     * @brief Super-Twisting Algorithm (STA)
     * 
     * u = -α|s|^(1/2)·sign(s) + v
     * v̇ = -β·sign(s)
     * 
     * Provides continuous control with finite-time convergence
     
    double computeSuperTwisting(double s, double dt) {
        double alpha = config_.sta_alpha;
        double beta = config_.sta_beta;
        
        // Super-twisting control law
        double u = -alpha * std::sqrt(std::abs(s)) * SlidingSurface::sign(s) + sta_integral_;
        
        // Update integral term
        sta_integral_ -= beta * SlidingSurface::sign(s) * dt;
        
        return u;
    }
    */
double computeSuperTwisting(const std::vector<double>& x, double s, double dt) {
    // Thêm equivalent control
    double gx = g_(x);
    if (std::abs(gx) < 1e-10) gx = 1.0;
    double u_eq = -f_(x) / gx;          // ← THÊM DÒNG NÀY
    
    double alpha = config_.sta_alpha;
    double beta = config_.sta_beta;

    double u_sta = -alpha * std::sqrt(std::abs(s)) * SlidingSurface::sign(s) + sta_integral_;
    sta_integral_ -= beta * SlidingSurface::sign(s) * dt;
    
    return u_eq + u_sta;                 // ← SỬA DÒNG NÀY
}

    /**
     * @brief Integral Sliding Mode Control
     * 
     * Eliminates reaching phase by designing surface through initial condition
     */
    double computeIntegralSMC(const std::vector<double>& x, double s, double error) {
        double gx = g_(x);
        if (std::abs(gx) < 1e-10) gx = 1.0;
        
        double fx = f_(x);
        
        // Nominal control (from linear design)
        double u_nom = -fx / gx;
        
        // Discontinuous compensation
        double switch_term = config_.use_boundary_layer ?
                            SlidingSurface::sat(s, config_.boundary_thickness) :
                            SlidingSurface::sign(s);
        
        double u_disc = -(config_.K + D_max_) * switch_term / gx;
        
        return u_nom + u_disc;
    }
    
    /**
     * @brief Quasi-Continuous Higher-Order SMC
     * 
     * Provides smoother control while maintaining robustness
     */
    double computeQuasiContinuous(const std::vector<double>& x, double s) {
        double gx = g_(x);
        if (std::abs(gx) < 1e-10) gx = 1.0;
        
        double fx = f_(x);
        
        // Equivalent control
        double u_eq = -fx / gx;
        
        // Quasi-continuous term: u = -K · s / (|s| + ε)
        double epsilon = config_.boundary_thickness;
        double u_qc = -config_.K * s / (std::abs(s) + epsilon) / gx;
        
        return u_eq + u_qc;
    }
    
    /**
     * @brief Prescribed-Time SMC
     * 
     * Guarantees convergence within prescribed time T
     */
    double computePrescribedTime(const std::vector<double>& x, double s, double dt) {
        double gx = g_(x);
        if (std::abs(gx) < 1e-10) gx = 1.0;
        
        prescribed_time_elapsed_ += dt;
        
        double T_prescribed = 2.0;  // Prescribed settling time
        double tau = T_prescribed - prescribed_time_elapsed_;
        
        if (tau <= 0.01) tau = 0.01;  // Prevent singularity
        
        // Time-varying gain
        double K_t = config_.K / tau;
        
        double switch_term = config_.use_boundary_layer ?
                            SlidingSurface::sat(s, config_.boundary_thickness) :
                            SlidingSurface::sign(s);
        
        double fx = f_(x);
        double u = (-fx - K_t * switch_term) / gx;
        
        return u;
    }
    
    /**
     * @brief Fixed-Time SMC (FxTSMC)
     * 
     * Based on Polyakov (2012): "Nonlinear feedback design for fixed-time stabilization"
     * 
     * Control law: u = u_eq - (k1|s|^p·sign(s) + k2|s|^q·sign(s)) / g(x)
     * 
     * where 0 < p < 1 and q > 1
     * 
     * Fixed settling time bound: T_max = 1/(k1(1-p)) + 1/(k2(q-1))
     * 
     * Key advantage: Convergence time is bounded independent of initial conditions
     */
    double computeFixedTime(const std::vector<double>& x, double s) {
        double gx = g_(x);
        if (std::abs(gx) < 1e-10) gx = 1.0;
        
        double fx = f_(x);
        
        // Equivalent control
        double u_eq = -fx / gx;
        
        // Fixed-time reaching law parameters
        double p = config_.fxt_p;      // 0 < p < 1 (fast convergence far from surface)
        double q = config_.fxt_q;      // q > 1 (fast convergence near surface)
        double k1 = config_.fxt_k1;
        double k2 = config_.fxt_k2;
        
        // Fixed-time control term
        double sign_s = SlidingSurface::sign(s);
        double abs_s = std::abs(s);
        
        // Bi-power reaching law: ṡ = -k1|s|^p·sign(s) - k2|s|^q·sign(s)
        double u_fxt = -(k1 * std::pow(abs_s + 1e-10, p) * sign_s + 
                         k2 * std::pow(abs_s + 1e-10, q) * sign_s + 
                         D_max_ * sign_s) / gx;
        
        // Apply boundary layer if enabled
        if (config_.use_boundary_layer && abs_s < config_.boundary_thickness) {
            double sat_s = SlidingSurface::sat(s, config_.boundary_thickness);
            u_fxt = -(k1 * std::pow(config_.boundary_thickness, p) * sat_s + 
                      k2 * std::pow(config_.boundary_thickness, q) * sat_s) / gx;
        }
        
        return u_eq + u_fxt;
    }
    
    /**
     * @brief Event-Triggered SMC (ETSMC)
     * 
     * Based on Heemels & Donkers (2012), adapted for SMC
     * 
     * Control is only updated when trigger condition is violated:
     *   ||s(t) - s(tk)|| > σ||s(tk)|| + threshold
     * 
     * Benefits:
     * - Reduces computational load
     * - Reduces actuator wear
     * - Saves energy in networked systems
     */
    double computeEventTriggered(const std::vector<double>& x, double s, double dt) {
        current_time_ += dt;
        
        double gx = g_(x);
        if (std::abs(gx) < 1e-10) gx = 1.0;
        
        double fx = f_(x);
        
        // Check if event should be triggered
        double time_since_trigger = current_time_ - last_trigger_time_;
        
        // Relative trigger condition
        double trigger_error = std::abs(s);
        double trigger_threshold = config_.et_sigma * std::abs(last_control_) + config_.et_threshold;
        
        // Zeno prevention: minimum inter-event time
        bool min_time_passed = time_since_trigger >= config_.et_min_inter_event;
        
        // Event trigger condition
        event_triggered_ = (trigger_error > trigger_threshold && min_time_passed) || 
                          (time_since_trigger > 0.1);  // Maximum hold time
        
        if (event_triggered_ || last_trigger_time_ == 0.0) {
            // Compute new control
            double u_eq = -fx / gx;
            double switch_term = config_.use_boundary_layer ?
                                SlidingSurface::sat(s, config_.boundary_thickness) :
                                SlidingSurface::sign(s);
            double u_sw = -(config_.K + D_max_) * switch_term / gx;
            
            last_control_ = u_eq + u_sw;
            last_trigger_time_ = current_time_;
        }
        
        return last_control_;
    }
    
    /**
     * @brief Barrier Function-based SMC
     * 
     * Based on Tee et al. (2009): "Barrier Lyapunov Functions"
     * 
     * Ensures state constraints |x| < kc are never violated
     * 
     * Barrier Lyapunov function: V = (1/2)log(kc²/(kc² - x²)) + (1/2)s²
     * 
     * Benefits:
     * - Guaranteed constraint satisfaction
     * - No explicit constraint handling in optimization
     */
    double computeBarrierFunction(const std::vector<double>& x, double s) {
        double gx = g_(x);
        if (std::abs(gx) < 1e-10) gx = 1.0;
        
        double fx = f_(x);
        
        // State constraint bound
        double kc = config_.bf_state_bound;
        double kb = config_.bf_kb;
        
        // Current state (position)
        double x1 = x.empty() ? 0.0 : x[0];
        
        // Barrier function term
        double barrier_term = 0.0;
        
        // Ensure we're within bounds
        double margin = 0.99;  // Stay within 99% of bound
        double x1_safe = std::max(-kc * margin, std::min(kc * margin, x1));
        
        if (config_.bf_barrier_type == 0) {
            // Log barrier: B(x) = log(kc² / (kc² - x²))
            double denom = kc * kc - x1_safe * x1_safe;
            if (denom > 1e-10) {
                // ∂B/∂x = 2x / (kc² - x²)
                double dB_dx = 2.0 * x1_safe / denom;
                barrier_term = kb * dB_dx;
            }
        } else {
            // Reciprocal barrier: B(x) = 1 / (kc - |x|)
            double dist_to_bound = kc - std::abs(x1_safe);
            if (dist_to_bound > 1e-10) {
                barrier_term = kb * SlidingSurface::sign(x1_safe) / (dist_to_bound * dist_to_bound);
            }
        }
        
        // Equivalent control
        double u_eq = -fx / gx;
        
        // Switching control with barrier modification
        double switch_term = config_.use_boundary_layer ?
                            SlidingSurface::sat(s, config_.boundary_thickness) :
                            SlidingSurface::sign(s);
        double u_sw = -(config_.K + D_max_) * switch_term / gx;
        
        // Add barrier compensation
        double u_barrier = -barrier_term / gx;
        
        return u_eq + u_sw + u_barrier;
    }
    
    /**
     * @brief Disturbance Observer-based SMC (DOBSMC)
     * 
     * Combines SMC with disturbance observer for improved performance
     * 
     * Observer: d̂ = z + L·x
     *           ż = -L·(f(x) + g(x)·u + d̂)
     * 
     * Benefits:
     * - Reduced switching gain (less chattering)
     * - Better disturbance rejection
     * - Active disturbance estimation
     */
    double computeDisturbanceObserver(const std::vector<double>& x, double s, double dt) {
        double gx = g_(x);
        if (std::abs(gx) < 1e-10) gx = 1.0;
        
        double fx = f_(x);
        
        // Observer gain
        double L = config_.dob_gain;
        
        // Get velocity state for observer
        double x2 = (x.size() > 1) ? x[1] : 0.0;
        
        // Disturbance observer update (first-order filter structure)
        // d̂ = z + L·x2
        dob_estimated_disturbance_ = dob_aux_state_ + L * x2;
        
        // Auxiliary state dynamics
        // ż = -L·(f(x) + g(x)·u_prev + d̂) = -L·ẋ2_estimated
        // Simplified: ż = -L·d̂ (assuming steady state)
        double z_dot = -L * (dob_estimated_disturbance_);
        dob_aux_state_ += z_dot * dt;
        
        // Low-pass filter on estimated disturbance (using member variable)
        double wc = config_.dob_filter_freq;
        dob_filtered_disturbance_ += wc * (dob_estimated_disturbance_ - dob_filtered_disturbance_) * dt;
        
        // Equivalent control with disturbance compensation
        double u_eq = (-fx - dob_filtered_disturbance_) / gx;
        
        // Reduced switching control (disturbance already compensated)
        // Can use smaller gain since DOB handles most of the disturbance
        double K_reduced = config_.K * 0.3;  // Reduced gain
        
        double switch_term = config_.use_boundary_layer ?
                            SlidingSurface::sat(s, config_.boundary_thickness) :
                            SlidingSurface::sign(s);
        double u_sw = -K_reduced * switch_term / gx;
        
        return u_eq + u_sw;
    }
};

// ============================================================
//                  SMC SIMULATOR
// ============================================================

/**
 * @brief Simulator for SMC systems
 */
class SMCSimulator {
public:
    using SystemDynamics = std::function<std::vector<double>(
        double t, const std::vector<double>& x, double u)>;
    using ReferenceFunc = std::function<double(double t)>;
    using DisturbanceFunc = std::function<double(double t)>;
    
private:
    SystemDynamics dynamics_;
    SlidingModeController& controller_;
    size_t n_states_;
    
public:
    SMCSimulator(SlidingModeController& controller, SystemDynamics dynamics, size_t n_states)
        : dynamics_(dynamics), controller_(controller), n_states_(n_states) {}
    
    /**
     * @brief Run simulation
     */
    SMCSimulationResult simulate(
        const std::vector<double>& x0,
        double t_final,
        double dt,
        ReferenceFunc reference = [](double) { return 0.0; },
        DisturbanceFunc disturbance = [](double) { return 0.0; }
    ) {
        SMCSimulationResult result;
        
        std::vector<double> x = x0;
        double t = 0.0;
        
        controller_.reset();
        
        bool surface_reached = false;
        double surface_threshold = 0.01;
        
        while (t <= t_final) {
            double ref = reference(t);
            double dist = disturbance(t);
            
            // Compute control
            double u = controller_.compute(x, ref, dt);
            double s = controller_.getSlidingSurface(x, ref - x[0]);
            
            // Store results
            result.time.push_back(t);
            result.states.push_back(x);
            result.sliding_surface.push_back(s);
            result.control.push_back(u);
            result.reference.push_back(ref);
            
            // Check if surface reached
            if (!surface_reached && std::abs(s) < surface_threshold) {
                surface_reached = true;
                result.reaching_time = t;
            }
            
            // RK4 integration
            x = rk4Step(t, x, u + dist, dt);
            t += dt;
        }
        
        result.reached_surface = surface_reached;
        
        // Compute chattering metric
        if (result.control.size() > 2) {
            double max_diff = 0;
            for (size_t i = 2; i < result.control.size(); ++i) {
                double diff = std::abs(result.control[i] - result.control[i-1]);
                max_diff = std::max(max_diff, diff);
            }
            result.max_chattering = max_diff / dt;
        }
        
        return result;
    }
    
private:
    std::vector<double> rk4Step(double t, const std::vector<double>& x, double u, double dt) {
        auto k1 = dynamics_(t, x, u);
        
        std::vector<double> x2(n_states_);
        for (size_t i = 0; i < n_states_; ++i) {
            x2[i] = x[i] + 0.5 * dt * k1[i];
        }
        auto k2 = dynamics_(t + 0.5*dt, x2, u);
        
        std::vector<double> x3(n_states_);
        for (size_t i = 0; i < n_states_; ++i) {
            x3[i] = x[i] + 0.5 * dt * k2[i];
        }
        auto k3 = dynamics_(t + 0.5*dt, x3, u);
        
        std::vector<double> x4(n_states_);
        for (size_t i = 0; i < n_states_; ++i) {
            x4[i] = x[i] + dt * k3[i];
        }
        auto k4 = dynamics_(t + dt, x4, u);
        
        std::vector<double> x_new(n_states_);
        for (size_t i = 0; i < n_states_; ++i) {
            x_new[i] = x[i] + (dt / 6.0) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
        }
        
        return x_new;
    }
};

// ============================================================
//                  VISUALIZATION FUNCTIONS
// ============================================================

/**
 * @brief Plot SMC simulation results
 */
inline void plotSMCResponse(
    const SMCSimulationResult& result,
    const std::string& filename = "smc_response"
) {
    using namespace cppplot;
    
    figure(1000, 800);
    suptitle("Sliding Mode Control Response");
    layout(2, 2);
    
    // Plot 1: State trajectories
    subplot(2, 2, 1);
    auto x1 = result.getState(0);
    auto x2 = result.getState(1);
    plot(result.time, x1, "-", opts({{"color", "blue"}, {"linewidth", "2"}, {"label", "x₁ (position)"}}));
    plot(result.time, result.reference, "--", opts({{"color", "green"}, {"linewidth", "1.5"}, {"label", "Reference"}}));
    xlabel("Time (s)");
    ylabel("State");
    title("State Response");
    legend(true);
    grid(true);
    
    // Plot 2: Sliding surface
    subplot(2, 2, 2);
    plot(result.time, result.sliding_surface, "-", opts({{"color", "red"}, {"linewidth", "2"}}));
    axhline(0.0, opts({{"color", "black"}, {"linestyle", "--"}, {"linewidth", "1"}}));
    xlabel("Time (s)");
    ylabel("s(x)");
    std::ostringstream title_s;
    title_s << "Sliding Surface (t_reach = " << std::fixed << std::setprecision(3) 
            << result.reaching_time << " s)";
    title(title_s.str());
    grid(true);
    
    // Plot 3: Control signal
    subplot(2, 2, 3);
    plot(result.time, result.control, "-", opts({{"color", "orange"}, {"linewidth", "1.5"}}));
    xlabel("Time (s)");
    ylabel("u(t)");
    title("Control Signal");
    grid(true);
    
    // Plot 4: Phase portrait
    subplot(2, 2, 4);
    plot(x1, x2, "-", opts({{"color", "purple"}, {"linewidth", "1.5"}}));
    scatter({x1.front()}, {x2.front()}, opts({{"s", "100"}, {"color", "green"}, {"marker", "o"}}));
    scatter({x1.back()}, {x2.back()}, opts({{"s", "100"}, {"color", "red"}, {"marker", "x"}}));
    xlabel("x₁");
    ylabel("x₂");
    title("Phase Portrait");
    grid(true);
    
    savefig(filename + ".svg");
    savefig(filename + ".png");
    std::cout << "Saved: " << filename << ".svg, " << filename << ".png\n";
}

/**
 * @brief Plot sliding surface analysis
 */
inline void plotSlidingSurfaceAnalysis(
    const SMCSimulationResult& result,
    const std::string& filename = "smc_surface_analysis"
) {
    using namespace cppplot;
    
    figure(900, 600);
    suptitle("Sliding Surface Analysis");
    layout(1, 2);
    
    // Plot 1: s(t) and ṡ(t)
    subplot(1, 2, 1);
    plot(result.time, result.sliding_surface, "-", 
         opts({{"color", "blue"}, {"linewidth", "2"}, {"label", "s(t)"}}));
    
    // Estimate ṡ
    std::vector<double> s_dot;
    for (size_t i = 1; i < result.sliding_surface.size(); ++i) {
        double ds = (result.sliding_surface[i] - result.sliding_surface[i-1]) / 
                    (result.time[i] - result.time[i-1]);
        s_dot.push_back(ds);
    }
    s_dot.insert(s_dot.begin(), s_dot.front());
    
    plot(result.time, s_dot, "--", 
         opts({{"color", "red"}, {"linewidth", "1.5"}, {"label", "ṡ(t)"}}));
    axhline(0.0, opts({{"color", "black"}, {"linestyle", ":"}}));
    
    xlabel("Time (s)");
    ylabel("Surface / Derivative");
    title("Sliding Variable Dynamics");
    legend(true);
    grid(true);
    
    // Plot 2: Phase plane with sliding surface
    subplot(1, 2, 2);
    auto x1 = result.getState(0);
    auto x2 = result.getState(1);
    
    // Plot sliding line (for linear surface s = c1*x1 + x2)
    double x1_min = *std::min_element(x1.begin(), x1.end());
    double x1_max = *std::max_element(x1.begin(), x1.end());
    std::vector<double> surface_x1, surface_x2;
    for (double xi = x1_min - 0.5; xi <= x1_max + 0.5; xi += 0.1) {
        surface_x1.push_back(xi);
        surface_x2.push_back(-xi);  // Assuming s = x1 + x2 = 0 → x2 = -x1
    }
    
    plot(surface_x1, surface_x2, "--", 
         opts({{"color", "green"}, {"linewidth", "2"}, {"label", "Sliding surface s=0"}}));
    plot(x1, x2, "-", 
         opts({{"color", "blue"}, {"linewidth", "1.5"}, {"label", "Trajectory"}}));
    
    scatter({x1.front()}, {x2.front()}, opts({{"s", "120"}, {"color", "green"}, {"marker", "o"}}));
    scatter({0.0}, {0.0}, opts({{"s", "120"}, {"color", "red"}, {"marker", "x"}}));
    
    xlabel("x₁");
    ylabel("x₂");
    title("Phase Plane with Sliding Surface");
    legend(true);
    grid(true);
    
    savefig(filename + ".svg");
    savefig(filename + ".png");
    std::cout << "Saved: " << filename << ".svg, " << filename << ".png\n";
}

/**
 * @brief Compare different SMC algorithms
 */
inline void plotSMCComparison(
    const std::vector<SMCSimulationResult>& results,
    const std::vector<std::string>& labels,
    const std::string& filename = "smc_comparison"
) {
    using namespace cppplot;
    
    if (results.empty() || results.size() != labels.size()) return;
    
    figure(1000, 700);
    suptitle("SMC Algorithm Comparison");
    layout(2, 2);
    
    std::vector<std::string> colors = {"blue", "red", "green", "orange", "purple"};
    
    // Plot 1: Output response
    subplot(2, 2, 1);
    for (size_t i = 0; i < results.size(); ++i) {
        auto x1 = results[i].getState(0);
        plot(results[i].time, x1, "-", 
             opts({{"color", colors[i % colors.size()]}, {"linewidth", "2"}, {"label", labels[i]}}));
    }
    plot(results[0].time, results[0].reference, "--", 
         opts({{"color", "gray"}, {"linewidth", "1"}, {"label", "Reference"}}));
    xlabel("Time (s)");
    ylabel("Output");
    title("Output Response");
    legend(true);
    grid(true);
    
    // Plot 2: Sliding surface
    subplot(2, 2, 2);
    for (size_t i = 0; i < results.size(); ++i) {
        plot(results[i].time, results[i].sliding_surface, "-",
             opts({{"color", colors[i % colors.size()]}, {"linewidth", "1.5"}, {"label", labels[i]}}));
    }
    axhline(0.0, opts({{"color", "black"}, {"linestyle", "--"}}));
    xlabel("Time (s)");
    ylabel("s(t)");
    title("Sliding Surface");
    legend(true);
    grid(true);
    
    // Plot 3: Control signal
    subplot(2, 2, 3);
    for (size_t i = 0; i < results.size(); ++i) {
        plot(results[i].time, results[i].control, "-",
             opts({{"color", colors[i % colors.size()]}, {"linewidth", "1"}, {"label", labels[i]}}));
    }
    xlabel("Time (s)");
    ylabel("u(t)");
    title("Control Signal");
    legend(true);
    grid(true);
    
    // Plot 4: Performance metrics
    subplot(2, 2, 4);
    std::vector<double> reaching_times, chattering;
    for (const auto& r : results) {
        reaching_times.push_back(r.reaching_time);
        chattering.push_back(r.max_chattering);
    }
    
    std::vector<double> x_pos;
    for (size_t i = 0; i < results.size(); ++i) {
        x_pos.push_back(static_cast<double>(i));
    }
    
    // Create text annotation for metrics
    std::ostringstream metrics;
    metrics << std::fixed << std::setprecision(3);
    metrics << "Performance Metrics:\n";
    for (size_t i = 0; i < labels.size(); ++i) {
        metrics << labels[i] << ": t_r=" << reaching_times[i] << "s";
        if (chattering[i] > 0) {
            metrics << ", chatter=" << std::setprecision(1) << chattering[i];
        }
        metrics << "\n";
    }
    
    text(0.1, 0.5, metrics.str(), opts({{"fontsize", "10"}}));
    title("Performance Comparison");
    
    savefig(filename + ".svg");
    savefig(filename + ".png");
    std::cout << "Saved: " << filename << ".svg, " << filename << ".png\n";
}

/**
 * @brief Plot chattering analysis
 */
inline void plotChatteringAnalysis(
    const SMCSimulationResult& result_sign,
    const SMCSimulationResult& result_sat,
    const std::string& filename = "smc_chattering"
) {
    using namespace cppplot;
    
    figure(900, 500);
    suptitle("Chattering Reduction Analysis");
    layout(1, 2);
    
    // Control signal with sign function
    subplot(1, 2, 1);
    plot(result_sign.time, result_sign.control, "-", 
         opts({{"color", "blue"}, {"linewidth", "1"}}));
    xlabel("Time (s)");
    ylabel("u(t)");
    std::ostringstream t1;
    t1 << "Conventional SMC (sign)\nMax rate: " << std::fixed << std::setprecision(1) 
       << result_sign.max_chattering;
    title(t1.str());
    grid(true);
    
    // Control signal with saturation
    subplot(1, 2, 2);
    plot(result_sat.time, result_sat.control, "-", 
         opts({{"color", "green"}, {"linewidth", "1.5"}}));
    xlabel("Time (s)");
    ylabel("u(t)");
    std::ostringstream t2;
    t2 << "Boundary Layer SMC (sat)\nMax rate: " << std::fixed << std::setprecision(1) 
       << result_sat.max_chattering;
    title(t2.str());
    grid(true);
    
    savefig(filename + ".svg");
    savefig(filename + ".png");
    std::cout << "Saved: " << filename << ".svg, " << filename << ".png\n";
}

// ============================================================
//              FACTORY FUNCTIONS FOR COMMON SYSTEMS
// ============================================================

/**
 * @brief Create SMC for second-order system (double integrator with disturbance)
 */
inline std::pair<SlidingModeController, SMCSimulator::SystemDynamics> 
createDoubleIntegratorSMC(
    SMCType type = SMCType::CONVENTIONAL,
    double lambda = 5.0,
    double K = 10.0,
    bool use_boundary_layer = false
) {
    // Surface: s = λe + ė = λ(r-x₁) + (0-x₂) for tracking
    // Simplified: s = λx₁ + x₂ for regulation to origin
    SlidingSurfaceConfig surface_cfg;
    surface_cfg.type = SurfaceType::LINEAR;
    surface_cfg.C = {lambda, 1.0};
    
    SMCConfig smc_cfg;
    smc_cfg.type = type;
    smc_cfg.K = K;
    smc_cfg.lambda = lambda;
    smc_cfg.use_boundary_layer = use_boundary_layer;
    smc_cfg.boundary_thickness = 0.1;
    
    // Super-twisting gains
    if (type == SMCType::SUPER_TWISTING) {
        smc_cfg.sta_alpha = 1.5 * std::sqrt(K);
        smc_cfg.sta_beta = 1.1 * K;
    }
    
    SlidingModeController controller(smc_cfg, surface_cfg, 2);
    
    // Double integrator: ẋ₁ = x₂, ẋ₂ = u
    auto dynamics = [](double t, const std::vector<double>& x, double u) {
        return std::vector<double>{x[1], u};
    };
    
    return {controller, dynamics};
}

/**
 * @brief Create SMC for mass-spring-damper system
 */
inline std::pair<SlidingModeController, SMCSimulator::SystemDynamics>
createMassSpringDamperSMC(
    double m = 1.0,
    double c = 0.5,
    double k = 1.0,
    SMCType type = SMCType::CONVENTIONAL,
    double K_smc = 20.0
) {
    // System: mẍ + cẋ + kx = u
    // State form: ẋ₁ = x₂, ẋ₂ = -k/m x₁ - c/m x₂ + 1/m u
    
    double a1 = k / m;
    double a2 = c / m;
    double b = 1.0 / m;
    
    SlidingSurfaceConfig surface_cfg;
    surface_cfg.type = SurfaceType::LINEAR;
    surface_cfg.C = {5.0, 1.0};  // s = 5x₁ + x₂
    
    SMCConfig smc_cfg;
    smc_cfg.type = type;
    smc_cfg.K = K_smc;
    smc_cfg.use_boundary_layer = true;
    smc_cfg.boundary_thickness = 0.05;
    
    SlidingModeController controller(smc_cfg, surface_cfg, 2);
    
    // Set dynamics for equivalent control computation
    controller.setDynamics(
        [a1, a2](const std::vector<double>& x) { return -a1*x[0] - a2*x[1]; },
        [b](const std::vector<double>&) { return b; },
        0.5  // D_max: max disturbance
    );
    
    // Full dynamics
    auto dynamics = [a1, a2, b](double t, const std::vector<double>& x, double u) {
        return std::vector<double>{
            x[1],
            -a1*x[0] - a2*x[1] + b*u
        };
    };
    
    return {controller, dynamics};
}

/**
 * @brief Create SMC for DC motor speed control
 */
inline std::pair<SlidingModeController, SMCSimulator::SystemDynamics>
createDCMotorSMC(
    double J = 0.01,
    double B = 0.001,
    double Km = 0.05,
    double R = 2.5,
    SMCType type = SMCType::SUPER_TWISTING,
    double K_smc = 50.0
) {
    // DC Motor (simplified): J*ω̇ + B*ω = Km/R * V
    // State: x = ω (angular velocity)
    // For 2nd order SMC, we use x = [∫ω, ω]
    
    double a = B / J;
    double b = Km / (J * R);
    
    SlidingSurfaceConfig surface_cfg;
    surface_cfg.type = SurfaceType::INTEGRAL;
    surface_cfg.C = {1.0, 0.0};
    surface_cfg.integral_gain = 10.0;
    
    SMCConfig smc_cfg;
    smc_cfg.type = type;
    smc_cfg.K = K_smc;
    smc_cfg.sta_alpha = 15.0;
    smc_cfg.sta_beta = 100.0;
    smc_cfg.use_boundary_layer = true;
    smc_cfg.boundary_thickness = 0.01;
    smc_cfg.u_min = -24.0;  // Voltage limits
    smc_cfg.u_max = 24.0;
    
    SlidingModeController controller(smc_cfg, surface_cfg, 2);
    
    controller.setDynamics(
        [a](const std::vector<double>& x) { return -a * x[1]; },
        [b](const std::vector<double>&) { return b; },
        5.0  // Load torque disturbance
    );
    
    // Dynamics: ẋ₁ = x₂ (∫ω = θ), ẋ₂ = -a*x₂ + b*u
    auto dynamics = [a, b](double t, const std::vector<double>& x, double u) {
        return std::vector<double>{
            x[1],
            -a*x[1] + b*u
        };
    };
    
    return {controller, dynamics};
}

/**
 * @brief Create Fixed-Time SMC controller
 * 
 * Fixed-time convergence: T_max = 1/(k1(1-p)) + 1/(k2(q-1))
 * Independent of initial conditions!
 */
inline std::pair<SlidingModeController, SMCSimulator::SystemDynamics>
createFixedTimeSMC(
    double lambda = 5.0,
    double k1 = 5.0,
    double k2 = 5.0,
    double p = 0.5,
    double q = 1.5
) {
    SlidingSurfaceConfig surface_cfg;
    surface_cfg.type = SurfaceType::LINEAR;
    surface_cfg.C = {lambda, 1.0};
    
    SMCConfig smc_cfg;
    smc_cfg.type = SMCType::FIXED_TIME;
    smc_cfg.fxt_k1 = k1;
    smc_cfg.fxt_k2 = k2;
    smc_cfg.fxt_p = p;
    smc_cfg.fxt_q = q;
    smc_cfg.use_boundary_layer = true;
    smc_cfg.boundary_thickness = 0.05;
    
    SlidingModeController controller(smc_cfg, surface_cfg, 2);
    
    auto dynamics = [](double t, const std::vector<double>& x, double u) {
        (void)t;  // Unused
        return std::vector<double>{x[1], u};
    };
    
    return {controller, dynamics};
}

/**
 * @brief Create Event-Triggered SMC controller
 * 
 * Resource-efficient SMC with reduced communication/computation
 */
inline std::pair<SlidingModeController, SMCSimulator::SystemDynamics>
createEventTriggeredSMC(
    double lambda = 5.0,
    double K = 15.0,
    double sigma = 0.3,
    double threshold = 0.05
) {
    SlidingSurfaceConfig surface_cfg;
    surface_cfg.type = SurfaceType::LINEAR;
    surface_cfg.C = {lambda, 1.0};
    
    SMCConfig smc_cfg;
    smc_cfg.type = SMCType::EVENT_TRIGGERED;
    smc_cfg.K = K;
    smc_cfg.et_sigma = sigma;
    smc_cfg.et_threshold = threshold;
    smc_cfg.et_min_inter_event = 0.001;
    smc_cfg.use_boundary_layer = true;
    smc_cfg.boundary_thickness = 0.1;
    
    SlidingModeController controller(smc_cfg, surface_cfg, 2);
    
    auto dynamics = [](double t, const std::vector<double>& x, double u) {
        (void)t;  // Unused
        return std::vector<double>{x[1], u};
    };
    
    return {controller, dynamics};
}

/**
 * @brief Create Barrier Function-based SMC controller
 * 
 * Guarantees state constraints |x| < bound
 */
inline std::pair<SlidingModeController, SMCSimulator::SystemDynamics>
createBarrierFunctionSMC(
    double lambda = 5.0,
    double K = 10.0,
    double state_bound = 5.0,
    double barrier_gain = 1.0
) {
    SlidingSurfaceConfig surface_cfg;
    surface_cfg.type = SurfaceType::LINEAR;
    surface_cfg.C = {lambda, 1.0};
    
    SMCConfig smc_cfg;
    smc_cfg.type = SMCType::BARRIER_FUNCTION;
    smc_cfg.K = K;
    smc_cfg.bf_state_bound = state_bound;
    smc_cfg.bf_kb = barrier_gain;
    smc_cfg.bf_barrier_type = 0;  // Log barrier
    smc_cfg.use_boundary_layer = true;
    smc_cfg.boundary_thickness = 0.1;
    
    SlidingModeController controller(smc_cfg, surface_cfg, 2);
    
    auto dynamics = [](double t, const std::vector<double>& x, double u) {
        (void)t;  // Unused
        return std::vector<double>{x[1], u};
    };
    
    return {controller, dynamics};
}

/**
 * @brief Create Disturbance Observer-based SMC controller
 * 
 * Reduced chattering through active disturbance estimation
 */
inline std::pair<SlidingModeController, SMCSimulator::SystemDynamics>
createDisturbanceObserverSMC(
    double lambda = 5.0,
    double K = 10.0,
    double observer_gain = 50.0,
    double D_max = 5.0
) {
    SlidingSurfaceConfig surface_cfg;
    surface_cfg.type = SurfaceType::LINEAR;
    surface_cfg.C = {lambda, 1.0};
    
    SMCConfig smc_cfg;
    smc_cfg.type = SMCType::DISTURBANCE_OBSERVER;
    smc_cfg.K = K;
    smc_cfg.dob_gain = observer_gain;
    smc_cfg.dob_filter_freq = 100.0;
    smc_cfg.use_boundary_layer = true;
    smc_cfg.boundary_thickness = 0.05;
    
    SlidingModeController controller(smc_cfg, surface_cfg, 2);
    controller.setDynamics(
        [](const std::vector<double>&) { return 0.0; },
        [](const std::vector<double>&) { return 1.0; },
        D_max
    );
    
    auto dynamics = [](double t, const std::vector<double>& x, double u) {
        (void)t;  // Unused
        return std::vector<double>{x[1], u};
    };
    
    return {controller, dynamics};
}

// ============================================================
//                  SUMMARY PRINTER
// ============================================================

/**
 * @brief Print SMC design summary
 */
inline void printSMCSummary(
    const SMCConfig& config,
    const SlidingSurfaceConfig& surface,
    const SMCSimulationResult& result
) {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║           SLIDING MODE CONTROL DESIGN SUMMARY                ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
    
    // Controller type
    std::cout << "║ Controller Type: ";
    switch (config.type) {
        case SMCType::CONVENTIONAL: std::cout << "Conventional SMC"; break;
        case SMCType::SUPER_TWISTING: std::cout << "Super-Twisting Algorithm"; break;
        case SMCType::INTEGRAL: std::cout << "Integral SMC"; break;
        case SMCType::QUASI_CONTINUOUS: std::cout << "Quasi-Continuous HOSM"; break;
        case SMCType::PRESCRIBED_TIME: std::cout << "Prescribed-Time SMC"; break;
        case SMCType::FIXED_TIME: std::cout << "Fixed-Time SMC"; break;
        case SMCType::EVENT_TRIGGERED: std::cout << "Event-Triggered SMC"; break;
        case SMCType::BARRIER_FUNCTION: std::cout << "Barrier Function SMC"; break;
        case SMCType::DISTURBANCE_OBSERVER: std::cout << "Disturbance Observer SMC"; break;
    }
    std::cout << std::setw(25) << " " << "║\n";
    
    // Surface type
    std::cout << "║ Surface Type: ";
    switch (surface.type) {
        case SurfaceType::LINEAR: std::cout << "Linear (s = Cx)"; break;
        case SurfaceType::INTEGRAL: std::cout << "Integral (s = Cx + Ki∫e)"; break;
        case SurfaceType::TERMINAL: std::cout << "Terminal"; break;
        case SurfaceType::PID_LIKE: std::cout << "PID-Like"; break;
    }
    std::cout << std::setw(30) << " " << "║\n";
    
    // Parameters
    std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ Parameters:                                                  ║\n";
    std::cout << "║   Switching Gain K = " << std::setw(10) << std::fixed << std::setprecision(2) 
              << config.K << std::setw(30) << " " << "║\n";
    
    if (config.use_boundary_layer) {
        std::cout << "║   Boundary Layer φ = " << std::setw(10) << config.boundary_thickness 
                  << " (chattering reduction)" << std::setw(7) << " " << "║\n";
    }
    
    if (config.type == SMCType::SUPER_TWISTING) {
        std::cout << "║   STA α = " << std::setw(8) << config.sta_alpha 
                  << ", β = " << std::setw(8) << config.sta_beta << std::setw(22) << " " << "║\n";
    }
    
    if (config.type == SMCType::FIXED_TIME) {
        std::cout << "║   FxT k1 = " << std::setw(6) << config.fxt_k1 
                  << ", k2 = " << std::setw(6) << config.fxt_k2 
                  << ", p = " << std::setw(4) << config.fxt_p 
                  << ", q = " << std::setw(4) << config.fxt_q << std::setw(5) << " " << "║\n";
        double T_max = 1.0/(config.fxt_k1*(1-config.fxt_p)) + 1.0/(config.fxt_k2*(config.fxt_q-1));
        std::cout << "║   T_max (bound) = " << std::setw(8) << std::setprecision(3) << T_max 
                  << " s" << std::setw(32) << " " << "║\n";
    }
    
    if (config.type == SMCType::EVENT_TRIGGERED) {
        std::cout << "║   ET σ = " << std::setw(6) << config.et_sigma 
                  << ", threshold = " << std::setw(6) << config.et_threshold << std::setw(18) << " " << "║\n";
    }
    
    if (config.type == SMCType::BARRIER_FUNCTION) {
        std::cout << "║   State Bound = " << std::setw(8) << config.bf_state_bound 
                  << ", kb = " << std::setw(6) << config.bf_kb << std::setw(20) << " " << "║\n";
    }
    
    if (config.type == SMCType::DISTURBANCE_OBSERVER) {
        std::cout << "║   DOB Gain L = " << std::setw(8) << config.dob_gain << std::setw(35) << " " << "║\n";
    }
    
    // Performance
    std::cout << "╠══════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ Performance:                                                 ║\n";
    std::cout << "║   Reaching Time = " << std::setw(8) << std::setprecision(3) 
              << result.reaching_time << " s" << std::setw(32) << " " << "║\n";
    std::cout << "║   Surface Reached: " << (result.reached_surface ? "YES ✓" : "NO ✗") 
              << std::setw(36) << " " << "║\n";
    std::cout << "║   Max Chattering = " << std::setw(8) << std::setprecision(1) 
              << result.max_chattering << std::setw(33) << " " << "║\n";
    
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n";
}

} // namespace nonlinear
} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_NONLINEAR_SLIDING_MODE_HPP
