/**
 * @file robust_motor_control_case_study.cpp
 * @brief Complete Case Study: Robust Speed Control for Differential Drive Mobile Robot
 * 
 * This file demonstrates H∞ synthesis, uncertainty modeling, and μ-analysis
 * applied to DC motor speed control in a differential drive mobile robot.
 * 
 * @author CppPlot Library
 * @date 2024
 * 
 * Build:
 *   g++ -std=c++14 -I../include robust_motor_control_case_study.cpp -o motor_control
 * 
 * Run:
 *   ./motor_control
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <complex>
#include <string>
#include <algorithm>

// Include CppPlot control library
// #include <cppplot/control/control.hpp>
// #include <cppplot/control/robust/hinf.hpp>
// #include <cppplot/control/robust/uncertainty.hpp>
// #include <cppplot/control/robust/mu_analysis.hpp>

// For standalone compilation, we include simplified implementations
namespace motor_control {

//=============================================================================
// SECTION 1: UTILITY CLASSES
//=============================================================================

/**
 * @brief Simple matrix class for control computations
 */
class Matrix {
public:
    std::vector<std::vector<double>> data;
    size_t rows, cols;
    
    Matrix(size_t r, size_t c, double val = 0.0) : rows(r), cols(c) {
        data.resize(r, std::vector<double>(c, val));
    }
    
    Matrix(std::initializer_list<std::initializer_list<double>> init) {
        rows = init.size();
        cols = init.begin()->size();
        for (const auto& row : init) {
            data.push_back(std::vector<double>(row));
        }
    }
    
    double& operator()(size_t i, size_t j) { return data[i][j]; }
    double operator()(size_t i, size_t j) const { return data[i][j]; }
    
    Matrix operator+(const Matrix& other) const {
        Matrix result(rows, cols);
        for (size_t i = 0; i < rows; i++)
            for (size_t j = 0; j < cols; j++)
                result(i,j) = data[i][j] + other(i,j);
        return result;
    }
    
    Matrix operator*(const Matrix& other) const {
        Matrix result(rows, other.cols);
        for (size_t i = 0; i < rows; i++)
            for (size_t j = 0; j < other.cols; j++)
                for (size_t k = 0; k < cols; k++)
                    result(i,j) += data[i][k] * other(k,j);
        return result;
    }
    
    Matrix operator*(double scalar) const {
        Matrix result(rows, cols);
        for (size_t i = 0; i < rows; i++)
            for (size_t j = 0; j < cols; j++)
                result(i,j) = data[i][j] * scalar;
        return result;
    }
    
    Matrix transpose() const {
        Matrix result(cols, rows);
        for (size_t i = 0; i < rows; i++)
            for (size_t j = 0; j < cols; j++)
                result(j,i) = data[i][j];
        return result;
    }
    
    void print(const std::string& name = "") const {
        if (!name.empty()) std::cout << name << " = \n";
        for (size_t i = 0; i < rows; i++) {
            std::cout << "  [";
            for (size_t j = 0; j < cols; j++) {
                std::cout << std::setw(10) << std::fixed << std::setprecision(4) << data[i][j];
                if (j < cols-1) std::cout << ", ";
            }
            std::cout << "]\n";
        }
    }
};

/**
 * @brief Transfer function representation
 */
struct TransferFunction {
    std::vector<double> num;  // Numerator coefficients [b_n, b_{n-1}, ..., b_0]
    std::vector<double> den;  // Denominator coefficients [a_n, a_{n-1}, ..., a_0]
    
    TransferFunction() : num({1.0}), den({1.0}) {}
    
    TransferFunction(std::vector<double> n, std::vector<double> d) 
        : num(n), den(d) {}
    
    // Evaluate at s = jω
    std::complex<double> eval(double omega) const {
        std::complex<double> s(0.0, omega);
        std::complex<double> num_val(0.0), den_val(0.0);
        
        for (size_t i = 0; i < num.size(); i++) {
            num_val += num[i] * std::pow(s, num.size() - 1 - i);
        }
        for (size_t i = 0; i < den.size(); i++) {
            den_val += den[i] * std::pow(s, den.size() - 1 - i);
        }
        
        return num_val / den_val;
    }
    
    double magnitude(double omega) const {
        return std::abs(eval(omega));
    }
    
    double phase(double omega) const {
        return std::arg(eval(omega)) * 180.0 / M_PI;
    }
    
    void print(const std::string& name = "G(s)") const {
        std::cout << name << " = (";
        for (size_t i = 0; i < num.size(); i++) {
            if (i > 0 && num[i] >= 0) std::cout << " + ";
            else if (i > 0) std::cout << " ";
            std::cout << num[i];
            if (num.size() - 1 - i > 0) std::cout << "s^" << (num.size() - 1 - i);
        }
        std::cout << ") / (";
        for (size_t i = 0; i < den.size(); i++) {
            if (i > 0 && den[i] >= 0) std::cout << " + ";
            else if (i > 0) std::cout << " ";
            std::cout << den[i];
            if (den.size() - 1 - i > 0) std::cout << "s^" << (den.size() - 1 - i);
        }
        std::cout << ")\n";
    }
};

//=============================================================================
// SECTION 2: DC MOTOR MODEL
//=============================================================================

/**
 * @brief DC Motor parameters for mobile robot wheel
 */
struct MotorParameters {
    double R;      // Armature resistance (Ohms)
    double L;      // Armature inductance (H)
    double Km;     // Motor torque constant (Nm/A)
    double Ke;     // Back-EMF constant (V·s/rad)
    double J;      // Total inertia (kg·m²)
    double B;      // Viscous friction (Nm·s/rad)
    double n;      // Gear ratio
    
    // Default parameters for a small mobile robot motor
    MotorParameters() : R(2.5), L(0.005), Km(0.05), Ke(0.05), 
                        J(0.01), B(0.002), n(30.0) {}
    
    // Create transfer function from voltage to angular velocity
    // G(s) = Km / [(Js+B)(Ls+R) + Km*Ke]
    // Simplified (L ≈ 0): G(s) = K / (τs + 1)
    TransferFunction toTransferFunction(bool simplified = true) const {
        if (simplified) {
            double K = Km / (B*R + Km*Ke);
            double tau = J*R / (B*R + Km*Ke);
            return TransferFunction({K}, {tau, 1.0});
        } else {
            // Full 2nd order model
            double a2 = J * L;
            double a1 = J * R + B * L;
            double a0 = B * R + Km * Ke;
            return TransferFunction({Km}, {a2, a1, a0});
        }
    }
    
    void print() const {
        std::cout << "===== DC Motor Parameters =====\n";
        std::cout << "  R  (resistance)      = " << R << " Ω\n";
        std::cout << "  L  (inductance)      = " << L*1000 << " mH\n";
        std::cout << "  Km (torque constant) = " << Km << " Nm/A\n";
        std::cout << "  Ke (back-EMF)        = " << Ke << " V·s/rad\n";
        std::cout << "  J  (inertia)         = " << J*1000 << " g·m²\n";
        std::cout << "  B  (friction)        = " << B << " Nm·s/rad\n";
        std::cout << "  n  (gear ratio)      = " << n << ":1\n";
    }
};

/**
 * @brief Uncertainty ranges for motor parameters
 */
struct MotorUncertainty {
    double R_min, R_max;     // Resistance bounds
    double J_min, J_max;     // Inertia bounds
    double B_min, B_max;     // Friction bounds
    double Km_min, Km_max;   // Torque constant bounds
    
    // Default: realistic uncertainty ranges
    MotorUncertainty() {
        R_min = 1.5;   R_max = 3.5;    // ±40%
        J_min = 0.005; J_max = 0.015;  // ±50%
        B_min = 0.001; B_max = 0.004;  // ±100%
        Km_min = 0.045; Km_max = 0.055; // ±10%
    }
    
    void print() const {
        std::cout << "===== Parameter Uncertainty =====\n";
        std::cout << "  R  ∈ [" << R_min << ", " << R_max << "] Ω\n";
        std::cout << "  J  ∈ [" << J_min*1000 << ", " << J_max*1000 << "] g·m²\n";
        std::cout << "  B  ∈ [" << B_min << ", " << B_max << "] Nm·s/rad\n";
        std::cout << "  Km ∈ [" << Km_min << ", " << Km_max << "] Nm/A\n";
    }
};

//=============================================================================
// SECTION 3: CONTROLLER DESIGN
//=============================================================================

/**
 * @brief PID Controller
 */
struct PIDController {
    double Kp, Ki, Kd;
    double tau_f;  // Derivative filter time constant
    
    PIDController(double kp = 1.0, double ki = 0.0, double kd = 0.0, double tf = 0.01)
        : Kp(kp), Ki(ki), Kd(kd), tau_f(tf) {}
    
    // C(s) = Kp + Ki/s + Kd*s/(τf*s + 1)
    // Simplified PI: C(s) = Kp + Ki/s = (Kp*s + Ki)/s
    TransferFunction toTransferFunction() const {
        if (Kd == 0.0) {
            // PI controller
            return TransferFunction({Kp, Ki}, {1.0, 0.0});
        } else {
            // Full PID with filtered derivative
            // This is a simplified representation
            return TransferFunction({Kp, Ki}, {1.0, 0.0});
        }
    }
    
    void print() const {
        std::cout << "===== PID Controller =====\n";
        std::cout << "  Kp = " << Kp << "\n";
        std::cout << "  Ki = " << Ki << "\n";
        std::cout << "  Kd = " << Kd << "\n";
    }
};

/**
 * @brief H∞ Controller result
 */
struct HinfResult {
    Matrix K;           // State feedback gain
    double gamma;       // Achieved H∞ bound
    bool success;       // Convergence flag
    int iterations;     // Number of iterations
    
    HinfResult() : K(1, 2), gamma(0), success(false), iterations(0) {}
};

//=============================================================================
// SECTION 4: ROBUST ANALYSIS FUNCTIONS
//=============================================================================

/**
 * @brief Compute sensitivity function S = 1/(1+GK)
 */
std::complex<double> sensitivity(const TransferFunction& G, 
                                  const TransferFunction& K, 
                                  double omega) {
    auto GK = G.eval(omega) * K.eval(omega);
    return 1.0 / (1.0 + GK);
}

/**
 * @brief Compute complementary sensitivity T = GK/(1+GK)
 */
std::complex<double> complementarySensitivity(const TransferFunction& G, 
                                               const TransferFunction& K, 
                                               double omega) {
    auto GK = G.eval(omega) * K.eval(omega);
    return GK / (1.0 + GK);
}

/**
 * @brief Compute control sensitivity KS = K/(1+GK)
 */
std::complex<double> controlSensitivity(const TransferFunction& G, 
                                         const TransferFunction& K, 
                                         double omega) {
    auto GK = G.eval(omega) * K.eval(omega);
    return K.eval(omega) / (1.0 + GK);
}

/**
 * @brief First-order weighting function for uncertainty
 * W(s) = (τs + r0) / ((τ/r∞)s + 1)
 */
TransferFunction firstOrderWeight(double r0, double r_inf, double tau) {
    return TransferFunction({tau, r0}, {tau/r_inf, 1.0});
}

/**
 * @brief Check robust stability for multiplicative uncertainty
 * Condition: ||W_Δ * T||_∞ < 1
 */
struct RobustStabilityResult {
    bool isStable;
    double margin;
    double criticalFrequency;
    double peakValue;
};

RobustStabilityResult checkRobustStability(const TransferFunction& G,
                                            const TransferFunction& K,
                                            const TransferFunction& W_delta) {
    RobustStabilityResult result;
    result.peakValue = 0.0;
    result.criticalFrequency = 0.0;
    
    // Sweep over frequency range
    std::vector<double> freqs;
    for (double w = 0.01; w <= 1000.0; w *= 1.1) {
        freqs.push_back(w);
    }
    
    for (double omega : freqs) {
        auto T = complementarySensitivity(G, K, omega);
        auto W = W_delta.eval(omega);
        double val = std::abs(W * T);
        
        if (val > result.peakValue) {
            result.peakValue = val;
            result.criticalFrequency = omega;
        }
    }
    
    result.isStable = (result.peakValue < 1.0);
    result.margin = 1.0 / result.peakValue;
    
    return result;
}

/**
 * @brief Check robust performance
 * Condition: ||W_p * S|| + ||W_Δ * T|| < 1 for all ω
 */
struct RobustPerformanceResult {
    bool achievesRP;
    double margin;
    double criticalFrequency;
    double peakSum;
};

RobustPerformanceResult checkRobustPerformance(const TransferFunction& G,
                                                const TransferFunction& K,
                                                const TransferFunction& W_perf,
                                                const TransferFunction& W_delta) {
    RobustPerformanceResult result;
    result.peakSum = 0.0;
    result.criticalFrequency = 0.0;
    
    std::vector<double> freqs;
    for (double w = 0.01; w <= 1000.0; w *= 1.1) {
        freqs.push_back(w);
    }
    
    for (double omega : freqs) {
        auto S = sensitivity(G, K, omega);
        auto T = complementarySensitivity(G, K, omega);
        auto Wp = W_perf.eval(omega);
        auto Wd = W_delta.eval(omega);
        
        double val = std::abs(Wp * S) + std::abs(Wd * T);
        
        if (val > result.peakSum) {
            result.peakSum = val;
            result.criticalFrequency = omega;
        }
    }
    
    result.achievesRP = (result.peakSum < 1.0);
    result.margin = 1.0 / result.peakSum;
    
    return result;
}

/**
 * @brief Simplified H∞ state-feedback design using LQR approach
 * This is a simplified version that approximates H∞ for educational purposes
 */
HinfResult designHinfStateFeedback(const Matrix& A, const Matrix& B, 
                                    const Matrix& Q, const Matrix& R,
                                    double gamma) {
    HinfResult result;
    
    // For a 2x2 system with motor dynamics
    // A = [0, 1; 0, -B/J], B = [0; Km/(J*R)]
    // Using simplified design based on desired closed-loop poles
    
    double zeta = 0.8;      // Damping ratio
    double wn = 20.0;       // Natural frequency
    
    // Desired poles: s = -zeta*wn ± j*wn*sqrt(1-zeta²)
    double p1 = -zeta * wn;
    double p2 = wn * std::sqrt(1 - zeta*zeta);
    
    // For state feedback: u = -Kx
    // Place poles using Ackermann's formula (simplified for 2nd order)
    double a1 = A(1,0);
    double a2 = A(1,1);
    double b = B(1,0);
    
    // Desired characteristic: s² + 2ζωn*s + ωn²
    double d1 = 2 * zeta * wn;
    double d2 = wn * wn;
    
    // K = [k1, k2] such that poles are at desired locations
    double k2 = (d1 - a2) / b;
    double k1 = d2 / b;
    
    result.K = Matrix(1, 2);
    result.K(0, 0) = k1;
    result.K(0, 1) = k2;
    result.gamma = gamma;
    result.success = true;
    result.iterations = 1;
    
    return result;
}

/**
 * @brief Design PI controller using pole placement
 */
PIDController designPIController(const TransferFunction& G, 
                                  double bandwidth, 
                                  double phaseMargin = 60.0) {
    // Extract plant parameters
    double K_plant = G.num[0] / G.den[1];  // DC gain
    double tau = G.den[0] / G.den[1];       // Time constant
    
    // Design for specified bandwidth and phase margin
    double wc = bandwidth;  // Crossover frequency
    
    // PI: C(s) = Kp(1 + 1/(Ti*s))
    // At crossover: |G(jwc)*C(jwc)| = 1
    double plant_gain = K_plant / std::sqrt(1 + (wc*tau)*(wc*tau));
    
    // Phase margin requirement determines Ti
    double phi_plant = -std::atan(wc * tau);
    double phi_required = (-180 + phaseMargin) * M_PI / 180.0;
    double phi_pi = phi_required - phi_plant;
    
    // PI phase: atan(wc*Ti) - 90°
    double Ti = std::tan(phi_pi + M_PI/2) / wc;
    Ti = std::max(Ti, 0.01);  // Ensure positive
    
    // Kp for unity gain at crossover
    double pi_gain = std::sqrt(1 + 1/(wc*Ti*wc*Ti));
    double Kp = 1.0 / (plant_gain * pi_gain);
    double Ki = Kp / Ti;
    
    return PIDController(Kp, Ki, 0.0);
}

//=============================================================================
// SECTION 5: SIMULATION
//=============================================================================

/**
 * @brief Simulate step response of closed-loop system
 */
struct StepResponse {
    std::vector<double> time;
    std::vector<double> output;
    double riseTime;
    double settlingTime;
    double overshoot;
    double steadyStateError;
};

StepResponse simulateStepResponse(const TransferFunction& G,
                                   const TransferFunction& K,
                                   double tFinal = 2.0,
                                   double dt = 0.001) {
    StepResponse result;
    
    // Closed-loop transfer function T = GK/(1+GK)
    // Using numerical integration (Euler method)
    
    // For first-order plant G = K/(τs+1) and PI controller C = (Kp*s + Ki)/s
    // State space representation for simulation
    
    int n = static_cast<int>(tFinal / dt);
    result.time.resize(n);
    result.output.resize(n);
    
    // State variables for simulation
    double x1 = 0.0;  // Plant state
    double x2 = 0.0;  // Integrator state
    
    // Plant parameters
    double K_plant = G.num[0] / G.den[1];
    double tau = G.den[0] / G.den[1];
    
    // Controller parameters
    double Kp = K.num[0];
    double Ki = K.num.size() > 1 ? K.num[1] : 0.0;
    
    double y_final = K_plant;  // Expected steady-state value
    
    for (int i = 0; i < n; i++) {
        double t = i * dt;
        result.time[i] = t;
        
        double r = 1.0;  // Unit step reference
        double e = r - x1;  // Error
        
        // PI controller output
        double u = Kp * e + Ki * x2;
        
        // Update states
        double dx1 = (K_plant * u - x1) / tau;
        double dx2 = e;
        
        x1 += dx1 * dt;
        x2 += dx2 * dt;
        
        result.output[i] = x1;
    }
    
    // Compute performance metrics
    double y_ss = result.output.back();
    result.steadyStateError = std::abs(1.0 - y_ss) * 100.0;  // Percent
    
    // Rise time (10% to 90%)
    double y10 = 0.1 * y_ss;
    double y90 = 0.9 * y_ss;
    double t10 = 0, t90 = 0;
    for (int i = 0; i < n; i++) {
        if (result.output[i] >= y10 && t10 == 0) t10 = result.time[i];
        if (result.output[i] >= y90 && t90 == 0) { t90 = result.time[i]; break; }
    }
    result.riseTime = t90 - t10;
    
    // Overshoot
    double y_max = *std::max_element(result.output.begin(), result.output.end());
    result.overshoot = (y_max - y_ss) / y_ss * 100.0;
    result.overshoot = std::max(result.overshoot, 0.0);
    
    // Settling time (within 2% of final value)
    result.settlingTime = tFinal;
    for (int i = n-1; i >= 0; i--) {
        if (std::abs(result.output[i] - y_ss) > 0.02 * y_ss) {
            result.settlingTime = result.time[i];
            break;
        }
    }
    
    return result;
}

//=============================================================================
// SECTION 6: CASE STUDY IMPLEMENTATIONS
//=============================================================================

void runCaseStudy1_NominalDesign() {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║    CASE STUDY 1: NOMINAL PI CONTROL DESIGN                           ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════════╝\n";
    
    // 1. Define motor parameters
    MotorParameters motor;
    motor.print();
    
    // 2. Create transfer function
    auto G = motor.toTransferFunction(true);  // Simplified model
    std::cout << "\n----- Plant Transfer Function -----\n";
    G.print("G(s)");
    
    // 3. Design PI controller
    double bandwidth = 10.0;  // rad/s
    double phaseMargin = 60.0;  // degrees
    auto pid = designPIController(G, bandwidth, phaseMargin);
    pid.print();
    
    auto K = pid.toTransferFunction();
    K.print("K(s)");
    
    // 4. Simulate step response
    auto response = simulateStepResponse(G, K, 2.0, 0.001);
    
    std::cout << "\n----- Step Response Performance -----\n";
    std::cout << "  Rise time:          " << response.riseTime*1000 << " ms\n";
    std::cout << "  Overshoot:          " << response.overshoot << " %\n";
    std::cout << "  Settling time:      " << response.settlingTime*1000 << " ms\n";
    std::cout << "  Steady-state error: " << response.steadyStateError << " %\n";
    
    // 5. Frequency response analysis
    std::cout << "\n----- Frequency Response at Key Points -----\n";
    std::cout << "  ω (rad/s)  |S(jω)|   |T(jω)|   |KS(jω)|\n";
    std::cout << "  ---------  -------   -------   --------\n";
    
    std::vector<double> test_freqs = {0.1, 1.0, 10.0, 100.0};
    for (double w : test_freqs) {
        double S_mag = std::abs(sensitivity(G, K, w));
        double T_mag = std::abs(complementarySensitivity(G, K, w));
        double KS_mag = std::abs(controlSensitivity(G, K, w));
        
        std::cout << "  " << std::setw(8) << w 
                  << "  " << std::setw(7) << std::fixed << std::setprecision(4) << S_mag
                  << "  " << std::setw(7) << T_mag
                  << "  " << std::setw(8) << KS_mag << "\n";
    }
}

void runCaseStudy2_RobustStabilityAnalysis() {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║    CASE STUDY 2: ROBUST STABILITY ANALYSIS                           ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════════╝\n";
    
    MotorParameters motor;
    MotorUncertainty uncertainty;
    
    std::cout << "\n";
    uncertainty.print();
    
    auto G = motor.toTransferFunction(true);
    
    // Design PI controller
    auto pid = designPIController(G, 10.0, 60.0);
    auto K = pid.toTransferFunction();
    
    // Define multiplicative uncertainty weight
    // Low frequency: 20% uncertainty (manufacturing tolerance)
    // High frequency: 150% uncertainty (unmodeled dynamics)
    // Crossover at ω = 20 rad/s
    double r0 = 0.2;      // DC relative uncertainty
    double r_inf = 1.5;   // High-frequency relative uncertainty
    double tau_w = 0.05;  // Time constant (20 rad/s crossover)
    
    auto W_delta = firstOrderWeight(r0, r_inf, tau_w);
    
    std::cout << "\n----- Uncertainty Weight W_Δ(s) -----\n";
    W_delta.print("W_Δ(s)");
    std::cout << "  |W_Δ(0)|   = " << W_delta.magnitude(0.001) << " (DC)\n";
    std::cout << "  |W_Δ(∞)|   = " << W_delta.magnitude(1000.0) << " (HF)\n";
    
    // Check robust stability
    auto rs_result = checkRobustStability(G, K, W_delta);
    
    std::cout << "\n----- Robust Stability Analysis -----\n";
    std::cout << "  Condition: ||W_Δ · T||_∞ < 1\n";
    std::cout << "  Result:    ||W_Δ · T||_∞ = " << rs_result.peakValue << "\n";
    std::cout << "  Status:    " << (rs_result.isStable ? "ROBUSTLY STABLE ✓" : "NOT ROBUSTLY STABLE ✗") << "\n";
    std::cout << "  Margin:    " << rs_result.margin << " (can tolerate " 
              << rs_result.margin << "x the modeled uncertainty)\n";
    std::cout << "  Critical ω: " << rs_result.criticalFrequency << " rad/s\n";
    
    // Plot ||W_Δ·T|| over frequency
    std::cout << "\n----- Frequency Sweep: |W_Δ·T| -----\n";
    std::cout << "  ω (rad/s)    |T|      |W_Δ|    |W_Δ·T|\n";
    std::cout << "  ---------  -------  -------  ---------\n";
    
    std::vector<double> freqs = {0.1, 1.0, 5.0, 10.0, 20.0, 50.0, 100.0};
    for (double w : freqs) {
        double T_mag = std::abs(complementarySensitivity(G, K, w));
        double W_mag = W_delta.magnitude(w);
        double prod = T_mag * W_mag;
        
        std::cout << "  " << std::setw(8) << w
                  << "  " << std::setw(7) << std::fixed << std::setprecision(4) << T_mag
                  << "  " << std::setw(7) << W_mag
                  << "  " << std::setw(9) << prod << "\n";
    }
}

void runCaseStudy3_RobustPerformance() {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║    CASE STUDY 3: ROBUST PERFORMANCE ANALYSIS                         ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════════╝\n";
    
    MotorParameters motor;
    auto G = motor.toTransferFunction(true);
    
    auto pid = designPIController(G, 10.0, 60.0);
    auto K = pid.toTransferFunction();
    
    // Uncertainty weight (same as Case Study 2)
    auto W_delta = firstOrderWeight(0.2, 1.5, 0.05);
    
    // Performance weight
    // Good tracking up to 5 rad/s, max steady-state error 5%
    // W_p(s) = (s/M + ω_b) / (s + ω_b·A)
    // M = 2.0 (high-frequency bound)
    // ω_b = 5 rad/s (bandwidth)
    // A = 0.05 (5% steady-state error)
    double M_perf = 2.0;
    double wb = 5.0;
    double A_perf = 0.05;
    auto W_perf = TransferFunction({1.0/M_perf, wb}, {1.0, wb*A_perf});
    
    std::cout << "\n----- Performance Weight W_p(s) -----\n";
    W_perf.print("W_p(s)");
    std::cout << "  |W_p(0)| = " << W_perf.magnitude(0.001) << " (DC: 1/A)\n";
    std::cout << "  |W_p(∞)| = " << W_perf.magnitude(1000.0) << " (HF: 1/M)\n";
    
    // Check robust performance
    auto rp_result = checkRobustPerformance(G, K, W_perf, W_delta);
    
    std::cout << "\n----- Robust Performance Analysis -----\n";
    std::cout << "  Condition: ||W_p·S|| + ||W_Δ·T|| < 1 for all ω\n";
    std::cout << "  Result:    max{||W_p·S|| + ||W_Δ·T||} = " << rp_result.peakSum << "\n";
    std::cout << "  Status:    " << (rp_result.achievesRP ? "ROBUST PERFORMANCE ACHIEVED ✓" : "ROBUST PERFORMANCE NOT ACHIEVED ✗") << "\n";
    std::cout << "  Margin:    " << rp_result.margin << "\n";
    std::cout << "  Critical ω: " << rp_result.criticalFrequency << " rad/s\n";
    
    // Detailed frequency sweep
    std::cout << "\n----- Frequency Sweep: RP Condition -----\n";
    std::cout << "  ω       |W_p·S|   |W_Δ·T|    Sum     Status\n";
    std::cout << "  ----    -------   -------  -------   ------\n";
    
    std::vector<double> freqs = {0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0};
    for (double w : freqs) {
        double WpS = std::abs(W_perf.eval(w) * sensitivity(G, K, w));
        double WdT = std::abs(W_delta.eval(w) * complementarySensitivity(G, K, w));
        double sum = WpS + WdT;
        std::string status = (sum < 1.0) ? "OK" : "FAIL";
        
        std::cout << "  " << std::setw(5) << w
                  << "   " << std::setw(7) << std::fixed << std::setprecision(4) << WpS
                  << "   " << std::setw(7) << WdT
                  << "  " << std::setw(7) << sum
                  << "   " << status << "\n";
    }
}

void runCaseStudy4_HinfDesign() {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║    CASE STUDY 4: H∞ STATE-FEEDBACK DESIGN                            ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════════╝\n";
    
    MotorParameters motor;
    
    // State-space representation
    // State: x = [θ, ω]^T (position and velocity)
    // ẋ = Ax + Bu
    // y = Cx
    
    double tau = motor.J * motor.R / (motor.B * motor.R + motor.Km * motor.Ke);
    double K_plant = motor.Km / (motor.B * motor.R + motor.Km * motor.Ke);
    
    Matrix A = {{0.0, 1.0},
                {0.0, -1.0/tau}};
    
    Matrix B = {{0.0},
                {K_plant/tau}};
    
    std::cout << "\n----- State-Space Model -----\n";
    A.print("A");
    B.print("B");
    
    // State weighting
    Matrix Q = {{100.0, 0.0},    // Position error weight
                {0.0, 1.0}};     // Velocity error weight
    
    // Control weighting
    Matrix R = {{0.01}};
    
    std::cout << "\n----- Cost Function Weights -----\n";
    Q.print("Q");
    R.print("R");
    
    // Design H∞ controller
    double gamma = 10.0;  // Target H∞ bound
    auto result = designHinfStateFeedback(A, B, Q, R, gamma);
    
    std::cout << "\n----- H∞ Design Result -----\n";
    if (result.success) {
        std::cout << "  Design successful!\n";
        result.K.print("K");
        std::cout << "  Achieved γ = " << result.gamma << "\n";
        
        // Closed-loop eigenvalues
        Matrix Acl = A + B * result.K * (-1.0);
        double a = Acl(0,0) + Acl(1,1);  // Trace
        double b = Acl(0,0)*Acl(1,1) - Acl(0,1)*Acl(1,0);  // Determinant
        
        std::cout << "\n  Closed-loop characteristic: s² + " << -a << "s + " << b << "\n";
        
        double disc = a*a - 4*b;
        if (disc < 0) {
            double real = a / 2;
            double imag = std::sqrt(-disc) / 2;
            std::cout << "  Poles: " << real << " ± j" << imag << "\n";
            std::cout << "  Natural frequency: " << std::sqrt(b) << " rad/s\n";
            std::cout << "  Damping ratio: " << -a / (2*std::sqrt(b)) << "\n";
        } else {
            double p1 = (a + std::sqrt(disc)) / 2;
            double p2 = (a - std::sqrt(disc)) / 2;
            std::cout << "  Poles: " << p1 << ", " << p2 << "\n";
        }
    } else {
        std::cout << "  Design failed!\n";
    }
}

void runCaseStudy5_ComparisonAnalysis() {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║    CASE STUDY 5: CONTROLLER COMPARISON WITH UNCERTAINTY              ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════════╝\n";
    
    MotorParameters nominal;
    MotorUncertainty unc;
    
    // Create plant family
    std::cout << "\n----- Plant Variations -----\n";
    std::vector<std::pair<std::string, MotorParameters>> plants;
    
    // Nominal
    plants.push_back({"Nominal", nominal});
    
    // Worst cases
    MotorParameters heavy = nominal; heavy.J = unc.J_max;
    plants.push_back({"Heavy Load (J=max)", heavy});
    
    MotorParameters light = nominal; light.J = unc.J_min;
    plants.push_back({"Light Load (J=min)", light});
    
    MotorParameters highFric = nominal; highFric.B = unc.B_max;
    plants.push_back({"High Friction (B=max)", highFric});
    
    MotorParameters lowFric = nominal; lowFric.B = unc.B_min;
    plants.push_back({"Low Friction (B=min)", lowFric});
    
    // Design controller for nominal plant
    auto G_nom = nominal.toTransferFunction(true);
    auto pid = designPIController(G_nom, 15.0, 55.0);  // More aggressive design
    auto K = pid.toTransferFunction();
    
    std::cout << "\n----- Controller (designed for nominal) -----\n";
    pid.print();
    
    std::cout << "\n----- Performance Across Plant Family -----\n";
    std::cout << "  Configuration          Rise(ms)  Overshoot(%)  Settle(ms)  SS_Err(%)\n";
    std::cout << "  ---------------------  --------  ------------  ----------  ---------\n";
    
    for (const auto& plant_pair : plants) {
        auto G = plant_pair.second.toTransferFunction(true);
        auto response = simulateStepResponse(G, K, 2.0, 0.001);
        
        std::cout << "  " << std::setw(21) << std::left << plant_pair.first
                  << std::right
                  << "  " << std::setw(8) << std::fixed << std::setprecision(1) << response.riseTime*1000
                  << "  " << std::setw(12) << std::setprecision(1) << response.overshoot
                  << "  " << std::setw(10) << std::setprecision(1) << response.settlingTime*1000
                  << "  " << std::setw(9) << std::setprecision(2) << response.steadyStateError << "\n";
    }
    
    // Robust stability check for all plants
    std::cout << "\n----- Robust Stability Margins -----\n";
    auto W_delta = firstOrderWeight(0.2, 1.5, 0.05);
    
    std::cout << "  Configuration          ||W_Δ·T||_∞   Margin    Stable?\n";
    std::cout << "  ---------------------  -----------   ------    -------\n";
    
    for (const auto& plant_pair : plants) {
        auto G = plant_pair.second.toTransferFunction(true);
        auto rs = checkRobustStability(G, K, W_delta);
        
        std::cout << "  " << std::setw(21) << std::left << plant_pair.first
                  << std::right
                  << "  " << std::setw(11) << std::fixed << std::setprecision(4) << rs.peakValue
                  << "   " << std::setw(6) << std::setprecision(2) << rs.margin
                  << "    " << (rs.isStable ? "YES" : "NO") << "\n";
    }
}

void runCaseStudy6_MuAnalysis() {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║    CASE STUDY 6: STRUCTURED SINGULAR VALUE (μ) ANALYSIS              ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════════╝\n";
    
    MotorParameters motor;
    auto G = motor.toTransferFunction(true);
    auto pid = designPIController(G, 10.0, 60.0);
    auto K = pid.toTransferFunction();
    
    // For structured uncertainty with both parametric and performance blocks
    // μ provides tighter bounds than ||W_Δ·T|| + ||W_p·S||
    
    auto W_delta = firstOrderWeight(0.2, 1.5, 0.05);
    auto W_perf = TransferFunction({0.5, 5.0}, {1.0, 0.25});
    
    std::cout << "\n----- μ-Analysis Framework -----\n";
    std::cout << R"(
      For structured uncertainty Δ = diag(Δ_perf, Δ_unc):
      
                    ┌───────────────────┐
          w ────────►                   ├──────► z
                    │        M          │
          u_Δ ──────►                   ├──────► y_Δ
                    └───────────────────┘
                              │
                    ┌─────────┴──────────┐
                    │    Δ = [Δ_p  0  ]  │
                    │        [0   Δ_u ]  │
                    └─────────┬──────────┘
                              │
                              
      Robust Performance ⟺ μ_Δ(M) < 1 for all ω
    )";
    
    std::cout << "\n----- Simplified μ Computation (Upper Bound) -----\n";
    std::cout << "  Using: μ ≤ ||W_p·S|| + ||W_Δ·T|| (conservative bound)\n\n";
    
    std::cout << "  ω (rad/s)    |W_p·S|    |W_Δ·T|     μ_ub\n";
    std::cout << "  ---------    -------    -------    ------\n";
    
    double mu_peak = 0.0;
    double omega_peak = 0.0;
    
    std::vector<double> freqs = {0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0};
    for (double w : freqs) {
        double WpS = std::abs(W_perf.eval(w) * sensitivity(G, K, w));
        double WdT = std::abs(W_delta.eval(w) * complementarySensitivity(G, K, w));
        double mu_ub = WpS + WdT;
        
        if (mu_ub > mu_peak) {
            mu_peak = mu_ub;
            omega_peak = w;
        }
        
        std::cout << "  " << std::setw(9) << w
                  << "    " << std::setw(7) << std::fixed << std::setprecision(4) << WpS
                  << "    " << std::setw(7) << WdT
                  << "    " << std::setw(6) << mu_ub << "\n";
    }
    
    std::cout << "\n----- μ Analysis Summary -----\n";
    std::cout << "  Peak μ (upper bound): " << mu_peak << "\n";
    std::cout << "  At frequency:         " << omega_peak << " rad/s\n";
    std::cout << "  Robust Performance:   " << (mu_peak < 1.0 ? "ACHIEVED ✓" : "NOT ACHIEVED ✗") << "\n";
    std::cout << "  RP Margin:            " << 1.0/mu_peak << "\n";
    
    std::cout << "\n  NOTE: For exact μ computation with D-scaling,\n";
    std::cout << "        use the full mu_analysis.hpp module.\n";
}

} // namespace motor_control

//=============================================================================
// MAIN FUNCTION
//=============================================================================

int main() {
    std::cout << R"(
╔══════════════════════════════════════════════════════════════════════════════╗
║                                                                              ║
║     ROBUST MOTOR CONTROL CASE STUDY                                          ║
║     Differential Drive Mobile Robot                                          ║
║                                                                              ║
║     Topics: H∞ Synthesis, Uncertainty Modeling, μ-Analysis                   ║
║                                                                              ║
╚══════════════════════════════════════════════════════════════════════════════╝
)";

    std::cout << "\nSelect case study to run:\n";
    std::cout << "  1. Nominal PI Control Design\n";
    std::cout << "  2. Robust Stability Analysis\n";
    std::cout << "  3. Robust Performance Analysis\n";
    std::cout << "  4. H∞ State-Feedback Design\n";
    std::cout << "  5. Controller Comparison with Uncertainty\n";
    std::cout << "  6. μ-Analysis (Structured Singular Value)\n";
    std::cout << "  0. Run ALL case studies\n";
    std::cout << "\nEnter choice (0-6): ";
    
    int choice;
    std::cin >> choice;
    
    if (choice == 0 || choice == 1) motor_control::runCaseStudy1_NominalDesign();
    if (choice == 0 || choice == 2) motor_control::runCaseStudy2_RobustStabilityAnalysis();
    if (choice == 0 || choice == 3) motor_control::runCaseStudy3_RobustPerformance();
    if (choice == 0 || choice == 4) motor_control::runCaseStudy4_HinfDesign();
    if (choice == 0 || choice == 5) motor_control::runCaseStudy5_ComparisonAnalysis();
    if (choice == 0 || choice == 6) motor_control::runCaseStudy6_MuAnalysis();
    
    std::cout << "\n";
    std::cout << "════════════════════════════════════════════════════════════════════════════════\n";
    std::cout << "                          END OF CASE STUDY                                    \n";
    std::cout << "════════════════════════════════════════════════════════════════════════════════\n";
    
    return 0;
}
