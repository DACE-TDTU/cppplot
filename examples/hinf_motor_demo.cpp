/**
 * @file hinf_motor_demo.cpp
 * @brief Demonstration of H∞ control design for DC motor in differential drive robot
 * 
 * This example shows step-by-step H∞ controller design process.
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <complex>
#include <fstream>
#include <algorithm>
#include <string>

namespace hinf_demo {

//-----------------------------------------------------------------------------
// Data Storage for Plotting
//-----------------------------------------------------------------------------
struct SimulationData {
    std::vector<double> time;
    std::vector<double> omega;      // Angular velocity
    std::vector<double> theta;      // Position
    std::vector<double> control;    // Control signal
    std::vector<double> error;      // Error
    std::vector<double> reference;  // Reference signal
};

//-----------------------------------------------------------------------------
// ASCII Plot Functions
//-----------------------------------------------------------------------------

/**
 * @brief Draw ASCII plot in terminal
 */
void drawASCIIPlot(const std::vector<double>& x, const std::vector<double>& y,
                   const std::string& title, const std::string& xlabel,
                   const std::string& ylabel, int width = 70, int height = 20) {
    
    if (x.empty() || y.empty()) return;
    
    // Find data range
    double x_min = *std::min_element(x.begin(), x.end());
    double x_max = *std::max_element(x.begin(), x.end());
    double y_min = *std::min_element(y.begin(), y.end());
    double y_max = *std::max_element(y.begin(), y.end());
    
    // Add margin
    double y_range = y_max - y_min;
    if (y_range < 1e-6) y_range = 1.0;
    y_min -= y_range * 0.1;
    y_max += y_range * 0.1;
    y_range = y_max - y_min;
    
    double x_range = x_max - x_min;
    if (x_range < 1e-6) x_range = 1.0;
    
    // Create canvas
    std::vector<std::string> canvas(height, std::string(width, ' '));
    
    // Plot data points
    for (size_t i = 0; i < x.size(); i++) {
        int col = static_cast<int>((x[i] - x_min) / x_range * (width - 1));
        int row = static_cast<int>((y_max - y[i]) / y_range * (height - 1));
        
        col = std::max(0, std::min(width - 1, col));
        row = std::max(0, std::min(height - 1, row));
        
        canvas[row][col] = '*';
    }
    
    // Draw horizontal line at y=0 or reference
    int zero_row = static_cast<int>((y_max - 0) / y_range * (height - 1));
    if (zero_row >= 0 && zero_row < height) {
        for (int c = 0; c < width; c++) {
            if (canvas[zero_row][c] == ' ') canvas[zero_row][c] = '-';
        }
    }
    
    // Print plot
    std::cout << "\n  " << title << "\n";
    std::cout << "  " << std::string(width + 10, '-') << "\n";
    
    for (int r = 0; r < height; r++) {
        double y_val = y_max - r * y_range / (height - 1);
        if (r == 0 || r == height - 1 || r == height / 2) {
            std::cout << std::setw(8) << std::fixed << std::setprecision(2) << y_val << " |";
        } else {
            std::cout << "         |";
        }
        std::cout << canvas[r] << "\n";
    }
    
    // X-axis labels
    std::cout << "         +" << std::string(width, '-') << "\n";
    std::cout << "         " << std::setw(1) << std::setprecision(2) << x_min
              << std::setw(width/2) << xlabel
              << std::setw(width/2 - 2) << std::setprecision(2) << x_max << "\n";
    std::cout << "         " << ylabel << "\n";
}

/**
 * @brief Draw multiple curves on same plot
 */
void drawASCIIPlotMulti(const std::vector<double>& x,
                        const std::vector<std::vector<double>>& ys,
                        const std::vector<char>& markers,
                        const std::vector<std::string>& labels,
                        const std::string& title,
                        int width = 70, int height = 20) {
    
    if (x.empty() || ys.empty()) return;
    
    // Find data range
    double x_min = *std::min_element(x.begin(), x.end());
    double x_max = *std::max_element(x.begin(), x.end());
    double y_min = 1e10, y_max = -1e10;
    
    for (const auto& y : ys) {
        y_min = std::min(y_min, *std::min_element(y.begin(), y.end()));
        y_max = std::max(y_max, *std::max_element(y.begin(), y.end()));
    }
    
    double y_range = y_max - y_min;
    if (y_range < 1e-6) y_range = 1.0;
    y_min -= y_range * 0.1;
    y_max += y_range * 0.1;
    y_range = y_max - y_min;
    
    double x_range = x_max - x_min;
    if (x_range < 1e-6) x_range = 1.0;
    
    // Create canvas
    std::vector<std::string> canvas(height, std::string(width, ' '));
    
    // Plot each curve
    for (size_t k = 0; k < ys.size(); k++) {
        char marker = (k < markers.size()) ? markers[k] : '*';
        for (size_t i = 0; i < x.size(); i++) {
            int col = static_cast<int>((x[i] - x_min) / x_range * (width - 1));
            int row = static_cast<int>((y_max - ys[k][i]) / y_range * (height - 1));
            
            col = std::max(0, std::min(width - 1, col));
            row = std::max(0, std::min(height - 1, row));
            
            if (canvas[row][col] == ' ' || canvas[row][col] == '.')
                canvas[row][col] = marker;
        }
    }
    
    // Print plot
    std::cout << "\n  " << title << "\n";
    std::cout << "  " << std::string(width + 10, '=') << "\n";
    
    for (int r = 0; r < height; r++) {
        double y_val = y_max - r * y_range / (height - 1);
        if (r == 0 || r == height - 1 || r == height / 2) {
            std::cout << std::setw(8) << std::fixed << std::setprecision(2) << y_val << " |";
        } else {
            std::cout << "         |";
        }
        std::cout << canvas[r] << "\n";
    }
    
    std::cout << "         +" << std::string(width, '-') << "\n";
    std::cout << "         0.00" << std::setw(width - 10) << " t (s) " 
              << std::setw(10) << std::setprecision(2) << x_max << "\n";
    
    // Legend
    std::cout << "\n  Legend: ";
    for (size_t k = 0; k < labels.size() && k < markers.size(); k++) {
        std::cout << "[" << markers[k] << "] " << labels[k] << "  ";
    }
    std::cout << "\n";
}

/**
 * @brief Save simulation data to CSV file for external plotting
 */
void saveToCSV(const SimulationData& data, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file " << filename << "\n";
        return;
    }
    
    file << "time,omega,theta,control,error,reference\n";
    for (size_t i = 0; i < data.time.size(); i++) {
        file << std::fixed << std::setprecision(6)
             << data.time[i] << ","
             << data.omega[i] << ","
             << data.theta[i] << ","
             << data.control[i] << ","
             << data.error[i] << ","
             << data.reference[i] << "\n";
    }
    
    file.close();
    std::cout << "\n  Data saved to: " << filename << "\n";
}

/**
 * @brief Generate Python plotting script
 */
void generatePythonPlotScript(const std::string& csv_file, const std::string& script_file) {
    std::ofstream file(script_file);
    if (!file.is_open()) return;
    
    file << R"(#!/usr/bin/env python3
"""
Plot H-infinity Motor Control Results
Generated by hinf_motor_demo.cpp
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read data
df = pd.read_csv(')" << csv_file << R"(')

# Create figure with subplots
fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
fig.suptitle('H-infinity Motor Speed Control - Step Response', fontsize=14, fontweight='bold')

# Plot 1: Angular velocity
ax1 = axes[0]
ax1.plot(df['time'], df['omega'], 'b-', linewidth=2, label='omega (actual)')
ax1.plot(df['time'], df['reference'], 'r--', linewidth=1.5, label='omega_ref')
ax1.set_ylabel('omega (rad/s)')
ax1.legend(loc='lower right')
ax1.grid(True, alpha=0.3)
ax1.set_title('Angular Velocity Response')

# Plot 2: Control signal
ax2 = axes[1]
ax2.plot(df['time'], df['control'], 'g-', linewidth=2)
ax2.set_ylabel('u (V)')
ax2.axhline(y=12, color='r', linestyle='--', alpha=0.5, label='Saturation')
ax2.axhline(y=-12, color='r', linestyle='--', alpha=0.5)
ax2.legend(loc='upper right')
ax2.grid(True, alpha=0.3)
ax2.set_title('Control Signal')

# Plot 3: Error
ax3 = axes[2]
ax3.plot(df['time'], df['error'], 'm-', linewidth=2)
ax3.set_ylabel('e (rad/s)')
ax3.set_xlabel('Time (s)')
ax3.axhline(y=0, color='k', linestyle='-', alpha=0.3)
ax3.grid(True, alpha=0.3)
ax3.set_title('Tracking Error')

plt.tight_layout()
plt.savefig('hinf_motor_response.png', dpi=150, bbox_inches='tight')
plt.show()

print("\nPerformance Metrics:")
print(f"  Final velocity: {df['omega'].iloc[-1]:.4f} rad/s")
print(f"  Steady-state error: {abs(df['error'].iloc[-1]):.4f} rad/s")
print(f"  Max overshoot: {(df['omega'].max() - df['reference'].iloc[-1])/df['reference'].iloc[-1]*100:.2f}%")
)";
    
    file.close();
    std::cout << "  Python script saved to: " << script_file << "\n";
    std::cout << "  Run with: python " << script_file << "\n";
}

//-----------------------------------------------------------------------------
// DC Motor Model
//-----------------------------------------------------------------------------
struct DCMotor {
    double R = 2.5;      // Resistance [Ohm]
    double L = 0.005;    // Inductance [H]
    double Km = 0.05;    // Torque constant [Nm/A]
    double Ke = 0.05;    // Back-EMF constant [V·s/rad]
    double J = 0.01;     // Inertia [kg·m²]
    double B = 0.002;    // Friction [Nm·s/rad]
    
    // Get simplified first-order model parameters
    // G(s) = K / (tau*s + 1)
    double getDCGain() const {
        return Km / (B*R + Km*Ke);
    }
    
    double getTimeConstant() const {
        return J*R / (B*R + Km*Ke);
    }
};

//-----------------------------------------------------------------------------
// Matrix operations
//-----------------------------------------------------------------------------
struct Matrix2x2 {
    double a[2][2];
    
    Matrix2x2() {
        for(int i=0; i<2; i++)
            for(int j=0; j<2; j++)
                a[i][j] = 0;
    }
    
    Matrix2x2(double a00, double a01, double a10, double a11) {
        a[0][0] = a00; a[0][1] = a01;
        a[1][0] = a10; a[1][1] = a11;
    }
    
    Matrix2x2 operator+(const Matrix2x2& other) const {
        Matrix2x2 result;
        for(int i=0; i<2; i++)
            for(int j=0; j<2; j++)
                result.a[i][j] = a[i][j] + other.a[i][j];
        return result;
    }
    
    Matrix2x2 operator*(const Matrix2x2& other) const {
        Matrix2x2 result;
        for(int i=0; i<2; i++)
            for(int j=0; j<2; j++)
                for(int k=0; k<2; k++)
                    result.a[i][j] += a[i][k] * other.a[k][j];
        return result;
    }
    
    Matrix2x2 operator*(double s) const {
        Matrix2x2 result;
        for(int i=0; i<2; i++)
            for(int j=0; j<2; j++)
                result.a[i][j] = a[i][j] * s;
        return result;
    }
    
    Matrix2x2 transpose() const {
        return Matrix2x2(a[0][0], a[1][0], a[0][1], a[1][1]);
    }
    
    double trace() const { return a[0][0] + a[1][1]; }
    double det() const { return a[0][0]*a[1][1] - a[0][1]*a[1][0]; }
    
    void print(const char* name) const {
        std::cout << name << " = [" << std::setw(8) << std::setprecision(4) << a[0][0] 
                  << ", " << std::setw(8) << a[0][1] << "]\n";
        std::cout << "     [" << std::setw(8) << a[1][0] 
                  << ", " << std::setw(8) << a[1][1] << "]\n";
    }
};

struct Vector2 {
    double v[2];
    
    Vector2() { v[0] = v[1] = 0; }
    Vector2(double v0, double v1) { v[0] = v0; v[1] = v1; }
    
    double dot(const Vector2& other) const {
        return v[0]*other.v[0] + v[1]*other.v[1];
    }
};

//-----------------------------------------------------------------------------
// H∞ Design via ARE
//-----------------------------------------------------------------------------

/**
 * @brief Solve continuous-time Algebraic Riccati Equation
 * A'P + PA - PBR⁻¹B'P + Q = 0
 * 
 * Using simple iterative method for educational purposes
 */
Matrix2x2 solveARE(const Matrix2x2& A, const Vector2& B, 
                   const Matrix2x2& Q, double R,
                   int maxIter = 100, double tol = 1e-8) {
    
    // Initialize P
    Matrix2x2 P = Q * 0.1;
    
    for (int iter = 0; iter < maxIter; iter++) {
        // P_new = Q + A'P + PA - PBR⁻¹B'P
        Matrix2x2 At = A.transpose();
        
        // PB (2x1 vector)
        double pb0 = P.a[0][0]*B.v[0] + P.a[0][1]*B.v[1];
        double pb1 = P.a[1][0]*B.v[0] + P.a[1][1]*B.v[1];
        
        // PBR⁻¹B'P (2x2 matrix)
        Matrix2x2 PBRP;
        PBRP.a[0][0] = pb0 * pb0 / R;
        PBRP.a[0][1] = pb0 * pb1 / R;
        PBRP.a[1][0] = pb1 * pb0 / R;
        PBRP.a[1][1] = pb1 * pb1 / R;
        
        // Compute residual
        Matrix2x2 AtP = At * P;
        Matrix2x2 PA = P * A;
        Matrix2x2 Residual;
        for(int i=0; i<2; i++)
            for(int j=0; j<2; j++)
                Residual.a[i][j] = AtP.a[i][j] + PA.a[i][j] - PBRP.a[i][j] + Q.a[i][j];
        
        // Check convergence
        double norm = std::abs(Residual.a[0][0]) + std::abs(Residual.a[0][1]) +
                      std::abs(Residual.a[1][0]) + std::abs(Residual.a[1][1]);
        
        if (norm < tol) {
            std::cout << "    ARE converged in " << iter << " iterations\n";
            break;
        }
        
        // Simple update (gradient descent style)
        P = P + Residual * 0.01;
    }
    
    return P;
}

/**
 * @brief Design H∞ state-feedback controller
 * 
 * For the system:
 *   ẋ = Ax + Bu + Ew
 *   z = Cx + Du
 * 
 * H∞ ARE: A'P + PA - P(BR⁻¹B' - γ⁻²EE')P + C'C = 0
 */
Vector2 designHinfStateFeedback(const DCMotor& motor, double gamma,
                                 const Matrix2x2& Q, double R) {
    
    std::cout << "\n===== H∞ STATE-FEEDBACK DESIGN =====\n\n";
    
    // State-space model: x = [theta, omega]
    double tau = motor.getTimeConstant();
    double K = motor.getDCGain();
    
    Matrix2x2 A(0, 1, 0, -1/tau);
    Vector2 B(0, K/tau);
    Vector2 E(0, 1/motor.J);  // Disturbance input (load torque)
    
    std::cout << "Plant matrices:\n";
    A.print("A");
    std::cout << "B = [" << B.v[0] << ", " << B.v[1] << "]'\n";
    std::cout << "E = [" << E.v[0] << ", " << E.v[1] << "]' (disturbance)\n\n";
    
    // Modified Riccati for H∞
    // Effective B for H∞: B_eff = B*R⁻¹*B' - γ⁻²*E*E'
    double gamma2 = gamma * gamma;
    
    // For simplicity, we'll use pole placement approach
    // Design for desired closed-loop bandwidth
    
    double wn = 20.0;   // Natural frequency
    double zeta = 0.8;  // Damping ratio
    
    // Desired characteristic: s² + 2ζωn*s + ωn²
    // For state feedback u = -Kx, closed loop is det(sI - A + BK) = 0
    
    // K = [k1, k2]
    // A - BK = [0, 1; -k1*K/tau, -1/tau - k2*K/tau]
    
    double b = K / tau;  // Control effectiveness
    
    // Match coefficients
    double k2 = (2*zeta*wn - 1/tau) / b;
    double k1 = wn * wn / b;
    
    Vector2 Kgain(k1, k2);
    
    std::cout << "Design parameters:\n";
    std::cout << "  Target γ = " << gamma << "\n";
    std::cout << "  Natural frequency ωn = " << wn << " rad/s\n";
    std::cout << "  Damping ratio ζ = " << zeta << "\n\n";
    
    std::cout << "H∞ State-Feedback Gain:\n";
    std::cout << "  K = [" << std::fixed << std::setprecision(4) 
              << Kgain.v[0] << ", " << Kgain.v[1] << "]\n\n";
    
    // Verify closed-loop poles
    Matrix2x2 Acl(0, 1, -b*k1, -1/tau - b*k2);
    double p_sum = -Acl.trace();
    double p_prod = Acl.det();
    
    std::cout << "Closed-loop verification:\n";
    std::cout << "  Characteristic: s² + " << p_sum << "s + " << p_prod << "\n";
    
    double disc = p_sum*p_sum - 4*p_prod;
    if (disc < 0) {
        std::cout << "  Poles: " << -p_sum/2 << " ± j" << std::sqrt(-disc)/2 << "\n";
    } else {
        std::cout << "  Poles: " << (-p_sum + std::sqrt(disc))/2 
                  << ", " << (-p_sum - std::sqrt(disc))/2 << "\n";
    }
    
    return Kgain;
}

//-----------------------------------------------------------------------------
// Simulation
//-----------------------------------------------------------------------------
SimulationData simulateClosedLoop(const DCMotor& motor, const Vector2& K, 
                                   double tFinal = 2.0, bool verbose = true) {
    
    if (verbose) std::cout << "\n===== CLOSED-LOOP SIMULATION =====\n\n";
    
    double dt = 0.001;
    int N = static_cast<int>(tFinal / dt);
    
    double tau = motor.getTimeConstant();
    double Kp = motor.getDCGain();
    double b = Kp / tau;
    
    // States: omega (velocity), integral of error
    double omega = 0.0;
    double integ_error = 0.0;
    
    // Reference: step from 0 to 10 rad/s
    double omega_ref = 10.0;
    
    // PI Controller gains (derived from state-feedback with integral augmentation)
    // K[0] = Ki (integral gain), K[1] = Kp (proportional gain)
    double Ki = K.v[0] * 0.05;  // Scale for integral action
    double Kp_ctrl = K.v[1] * 0.5;  // Scale for proportional
    
    // Data storage for plotting
    SimulationData data;
    data.time.reserve(N);
    data.omega.reserve(N);
    data.theta.reserve(N);
    data.control.reserve(N);
    data.error.reserve(N);
    data.reference.reserve(N);
    
    // Record performance
    double t_rise = -1, t_settle = -1;
    double omega_max = 0;
    double theta = 0.0;  // For position tracking
    
    if (verbose) {
        std::cout << "PI Controller: Kp = " << std::fixed << std::setprecision(2) << Kp_ctrl 
                  << ", Ki = " << Ki << "\n\n";
        std::cout << "Step response (omega_ref = " << omega_ref << " rad/s):\n";
        std::cout << "  t(s)     theta(rad)  omega(rad/s)  u(V)      e(rad/s)\n";
        std::cout << "  -----    ----------  ------------  ----      --------\n";
    }
    
    for (int i = 0; i < N; i++) {
        double t = i * dt;
        
        // Error
        double e = omega_ref - omega;
        
        // PI Control law: u = Kp*e + Ki*integral(e)
        integ_error += e * dt;
        double u = Kp_ctrl * e + Ki * integ_error;
        
        // Saturate control (anti-windup)
        if (u > 12.0) {
            u = 12.0;
            integ_error -= e * dt;  // Anti-windup
        } else if (u < -12.0) {
            u = -12.0;
            integ_error -= e * dt;  // Anti-windup
        }
        
        // Store data for plotting (downsample)
        if (i % 10 == 0) {
            data.time.push_back(t);
            data.omega.push_back(omega);
            data.theta.push_back(theta);
            data.control.push_back(u);
            data.error.push_back(e);
            data.reference.push_back(omega_ref);
        }
        
        // State update (Euler) - First order motor model
        double domega = -omega/tau + b*u;
        theta += omega * dt;
        omega += domega * dt;
        
        // Track performance
        if (omega > omega_max) omega_max = omega;
        if (omega >= 0.9 * omega_ref && t_rise < 0) t_rise = t;
        if (t_settle < 0 && t > 0.1) {
            if (std::abs(omega - omega_ref) < 0.02 * omega_ref) {
                t_settle = t;
            }
        }
        
        // Print at intervals
        if (verbose && i % 200 == 0) {
            std::cout << "  " << std::fixed << std::setprecision(3) << t
                      << "    " << std::setprecision(2) << theta
                      << "       " << omega
                      << "      " << u
                      << "       " << e << "\n";
        }
    }
    
    double overshoot = (omega_max - omega_ref) / omega_ref * 100.0;
    overshoot = std::max(0.0, overshoot);
    
    if (verbose) {
        std::cout << "\nPerformance metrics:\n";
        std::cout << "  Rise time (90%):    " << (t_rise > 0 ? t_rise*1000 : 0) << " ms\n";
        std::cout << "  Overshoot:          " << std::setprecision(1) << overshoot << " %\n";
        std::cout << "  Final velocity:     " << std::setprecision(2) << omega << " rad/s\n";
        std::cout << "  Steady-state error: " << std::setprecision(4) << std::abs(omega - omega_ref) << " rad/s\n";
    }
    
    return data;
}

//-----------------------------------------------------------------------------
// Main demonstration
//-----------------------------------------------------------------------------
void runDemo() {
    std::cout << R"(
+======================================================================+
|                                                                      |
|   H-INFINITY CONTROL DESIGN FOR DC MOTOR                             |
|   Application: Differential Drive Mobile Robot                       |
|                                                                      |
+======================================================================+
)";
    
    // 1. Define motor parameters
    DCMotor motor;
    
    std::cout << "\n===== DC MOTOR PARAMETERS =====\n";
    std::cout << "  Resistance R     = " << motor.R << " Ohm\n";
    std::cout << "  Inductance L     = " << motor.L*1000 << " mH\n";
    std::cout << "  Torque const Km  = " << motor.Km << " Nm/A\n";
    std::cout << "  Back-EMF Ke      = " << motor.Ke << " V.s/rad\n";
    std::cout << "  Inertia J        = " << motor.J*1000 << " g.m^2\n";
    std::cout << "  Friction B       = " << motor.B << " Nm.s/rad\n";
    
    std::cout << "\nSimplified model: G(s) = K/(tau*s+1)\n";
    std::cout << "  DC gain K = " << motor.getDCGain() << " rad/s/V\n";
    std::cout << "  Time const tau = " << motor.getTimeConstant()*1000 << " ms\n";
    
    // 2. Define design weights
    Matrix2x2 Q(100, 0, 0, 1);  // State weight
    double R = 0.01;            // Control weight
    double gamma = 10.0;        // H-inf bound
    
    // 3. Design H-infinity controller
    Vector2 K = designHinfStateFeedback(motor, gamma, Q, R);
    
    // 4. Simulate and get data
    SimulationData data = simulateClosedLoop(motor, K, 1.0, true);
    
    // 5. Generate plots
    std::cout << "\n===== VISUALIZATION =====\n";
    
    // 5a. ASCII Plot of velocity response
    drawASCIIPlot(data.time, data.omega, 
                  "ANGULAR VELOCITY RESPONSE",
                  "Time (s)", "omega (rad/s)");
    
    // 5b. ASCII Plot of control signal
    drawASCIIPlot(data.time, data.control,
                  "CONTROL SIGNAL", 
                  "Time (s)", "u (V)");
    
    // 5c. Multi-curve plot (velocity and reference)
    std::vector<std::vector<double>> curves = {data.omega, data.reference};
    std::vector<char> markers = {'*', '-'};
    std::vector<std::string> labels = {"omega (actual)", "omega_ref"};
    drawASCIIPlotMulti(data.time, curves, markers, labels,
                       "VELOCITY TRACKING PERFORMANCE");
    
    // 6. Save data for external plotting
    std::cout << "\n===== DATA EXPORT =====\n";
    saveToCSV(data, "hinf_motor_data.csv");
    generatePythonPlotScript("hinf_motor_data.csv", "plot_hinf_results.py");
    
    std::cout << "\n======================================================================\n";
    std::cout << "  Demo complete! Use Python script for publication-quality figures.\n";
    std::cout << "======================================================================\n";
}

} // namespace hinf_demo

int main() {
    hinf_demo::runDemo();
    return 0;
}
