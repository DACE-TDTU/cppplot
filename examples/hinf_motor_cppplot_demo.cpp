/**
 * @file hinf_motor_cppplot_demo.cpp
 * @brief H-infinity control demo using CppPlot library for visualization
 * 
 * This example demonstrates H∞ controller design for DC motor in
 * differential drive robot with native C++ plotting using CppPlot.
 * 
 * Build: g++ -std=c++14 -I../include hinf_motor_cppplot_demo.cpp -o hinf_motor_cppplot_demo
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace cppplot;
using namespace cppplot::control;

//=============================================================================
// DC Motor Model
//=============================================================================
struct DCMotor {
    double R = 2.5;      // Resistance [Ohm]
    double L = 0.005;    // Inductance [H]
    double Km = 0.05;    // Torque constant [Nm/A]
    double Ke = 0.05;    // Back-EMF constant [V·s/rad]
    double J = 0.01;     // Inertia [kg·m²]
    double B = 0.002;    // Friction [Nm·s/rad]
    
    // DC gain: K = Km / (B*R + Km*Ke)
    double getK() const { return Km / (B*R + Km*Ke); }
    
    // Time constant: tau = J*R / (B*R + Km*Ke)
    double getTau() const { return J*R / (B*R + Km*Ke); }
    
    // Get transfer function: G(s) = K / (tau*s + 1)
    TransferFunction toTF() const {
        double K_val = getK();
        double tau = getTau();
        return TransferFunction({K_val}, {tau, 1.0});
    }
    
    void print() const {
        std::cout << "\n=== DC Motor Parameters ===\n";
        std::cout << "  R (resistance)  = " << R << " Ohm\n";
        std::cout << "  L (inductance)  = " << L*1000 << " mH\n";
        std::cout << "  Km (torque)     = " << Km << " Nm/A\n";
        std::cout << "  Ke (back-EMF)   = " << Ke << " V.s/rad\n";
        std::cout << "  J (inertia)     = " << J*1000 << " g.m^2\n";
        std::cout << "  B (friction)    = " << B << " Nm.s/rad\n";
        std::cout << "\n  Simplified: G(s) = " << getK() << " / (" 
                  << getTau() << "s + 1)\n";
    }
};

//=============================================================================
// PI Controller Design
//=============================================================================
struct PIController {
    double Kp = 0;
    double Ki = 0;
    
    TransferFunction toTF() const {
        // PI: C(s) = Kp + Ki/s = (Kp*s + Ki) / s
        return TransferFunction({Kp, Ki}, {1.0, 0.0});
    }
    
    void print() const {
        std::cout << "\n=== PI Controller ===\n";
        std::cout << "  Kp = " << Kp << "\n";
        std::cout << "  Ki = " << Ki << "\n";
        std::cout << "  C(s) = " << Kp << " + " << Ki << "/s\n";
    }
};

/**
 * @brief Design PI controller for desired phase margin
 */
PIController designPI(const DCMotor& motor, double wc, double pm_deg) {
    double tau = motor.getTau();
    double K_plant = motor.getK();
    
    // Plant gain and phase at crossover
    double plant_gain_wc = K_plant / std::sqrt(1 + (wc*tau)*(wc*tau));
    double plant_phase_wc = -std::atan(wc*tau) * 180.0 / M_PI;
    
    // PI phase contribution needed
    double phi_needed = -180 + pm_deg - plant_phase_wc;
    double Ti = std::tan((phi_needed + 90) * M_PI / 180.0) / wc;
    Ti = std::max(0.01, Ti);
    
    // Gain for |L(jwc)| = 1
    double pi_gain_wc = std::sqrt(1 + 1/(wc*Ti*wc*Ti));
    double Kp_val = 1.0 / (plant_gain_wc * pi_gain_wc);
    
    PIController pi;
    pi.Kp = Kp_val;
    pi.Ki = Kp_val / Ti;
    
    return pi;
}

//=============================================================================
// Simulation
//=============================================================================
struct SimData {
    std::vector<double> t, y, u, e, r;
};

SimData simulate(const DCMotor& motor, const PIController& pi, 
                 double omega_ref, double t_final, double dt = 0.001) {
    
    SimData data;
    int N = static_cast<int>(t_final / dt);
    
    double tau = motor.getTau();
    double K_motor = motor.getK();
    double b = K_motor / tau;
    
    double omega = 0.0;
    double integ_e = 0.0;
    
    for (int i = 0; i < N; i++) {
        double t_val = i * dt;
        double err = omega_ref - omega;
        
        // PI control
        integ_e += err * dt;
        double u_val = pi.Kp * err + pi.Ki * integ_e;
        
        // Saturation with anti-windup
        if (u_val > 12.0) {
            u_val = 12.0;
            integ_e -= err * dt;
        } else if (u_val < -12.0) {
            u_val = -12.0;
            integ_e -= err * dt;
        }
        
        // Store data (downsample)
        if (i % 10 == 0) {
            data.t.push_back(t_val);
            data.y.push_back(omega);
            data.u.push_back(u_val);
            data.e.push_back(err);
            data.r.push_back(omega_ref);
        }
        
        // Update state
        double domega = -omega/tau + b*u_val;
        omega += domega * dt;
    }
    
    return data;
}

//=============================================================================
// Main Demo
//=============================================================================
int main() {
    std::cout << R"(
+======================================================================+
|  H-INFINITY MOTOR CONTROL DEMO - Using CppPlot Library               |
|  Application: Differential Drive Mobile Robot                        |
+======================================================================+
)" << std::endl;

    // 1. Define motor
    DCMotor motor;
    motor.print();
    
    // 2. Create transfer function using CppPlot
    auto G = motor.toTF();
    std::cout << "\nPlant Transfer Function:\n";
    std::cout << "  G(s) = " << G.toString() << "\n";
    
    // 3. Design PI controller
    double wc = 10.0;   // Crossover frequency [rad/s]
    double pm = 60.0;   // Phase margin [deg]
    auto pi = designPI(motor, wc, pm);
    pi.print();
    
    auto C = pi.toTF();
    std::cout << "  C(s) = " << C.toString() << "\n";
    
    // 4. Closed-loop analysis
    auto L = G * C;
    auto T = feedback(L);      // Complementary sensitivity
    auto one = TransferFunction({1}, {1});
    auto S = one - T;  // Sensitivity
    
    std::cout << "\n=== Closed-Loop Transfer Functions ===\n";
    std::cout << "  Loop L(s) = G*C:\n    " << L.toString() << "\n";
    std::cout << "  T(s) = L/(1+L):\n    " << T.toString() << "\n";
    
    // 5. Stability analysis using margin() function
    auto margins_result = margin(L);
    std::cout << "\n=== Stability Margins ===\n";
    std::cout << "  Gain margin:  " << margins_result.Gm << " (" 
              << margins_result.Gm_dB << " dB)\n";
    std::cout << "  Phase margin: " << margins_result.Pm << " deg\n";
    std::cout << "  Crossover:    " << margins_result.Wgc << " rad/s\n";
    
    // 6. Simulate step response
    std::cout << "\n=== Simulating Step Response ===\n";
    double omega_ref = 10.0;  // Reference [rad/s]
    auto data = simulate(motor, pi, omega_ref, 2.0);
    
    // Performance metrics
    double y_final = data.y.back();
    double y_max = *std::max_element(data.y.begin(), data.y.end());
    double overshoot = std::max(0.0, (y_max - omega_ref) / omega_ref * 100);
    double ss_error = std::abs(y_final - omega_ref);
    
    std::cout << "  Final velocity:     " << y_final << " rad/s\n";
    std::cout << "  Steady-state error: " << ss_error << " rad/s (" 
              << ss_error/omega_ref*100 << "%)\n";
    std::cout << "  Overshoot:          " << overshoot << "%\n";
    
    //=========================================================================
    // PLOTTING WITH CPPPLOT
    //=========================================================================
    std::cout << "\n=== Generating Plots with CppPlot ===\n";
    
    // Figure 1: Step Response (3 subplots)
    figure(1000, 800);
    suptitle("H-infinity Motor Speed Control - Step Response");
    layout(3, 1);
    
    // Subplot 1: Velocity response
    subplot(3, 1, 1);
    plot(data.t, data.y, "-", opts({{"color", "blue"}, {"linewidth", "2"}, {"label", "omega (actual)"}}));
    plot(data.t, data.r, "--", opts({{"color", "red"}, {"linewidth", "1.5"}, {"label", "omega_ref"}}));
    ylabel("omega (rad/s)");
    title("Angular Velocity Response");
    legend(true);
    grid(true);
    
    // Add 2% settling band
    std::vector<double> band_upper(data.t.size(), omega_ref * 1.02);
    std::vector<double> band_lower(data.t.size(), omega_ref * 0.98);
    plot(data.t, band_upper, ":", opts({{"color", "green"}, {"alpha", "0.5"}}));
    plot(data.t, band_lower, ":", opts({{"color", "green"}, {"alpha", "0.5"}}));
    
    // Subplot 2: Control signal
    subplot(3, 1, 2);
    plot(data.t, data.u, "-", opts({{"color", "green"}, {"linewidth", "2"}, {"label", "u (V)"}}));
    axhline(12, opts({{"color", "red"}, {"linestyle", "--"}, {"alpha", "0.5"}}));
    axhline(-12, opts({{"color", "red"}, {"linestyle", "--"}, {"alpha", "0.5"}}));
    ylabel("u (V)");
    title("Control Signal");
    legend(true);
    grid(true);
    
    // Subplot 3: Tracking error
    subplot(3, 1, 3);
    plot(data.t, data.e, "-", opts({{"color", "magenta"}, {"linewidth", "2"}}));
    axhline(0, opts({{"color", "black"}, {"alpha", "0.3"}}));
    xlabel("Time (s)");
    ylabel("e (rad/s)");
    title("Tracking Error");
    grid(true);
    
    savefig("hinf_step_response.svg");
    savefig("hinf_step_response.png");
    std::cout << "  Saved: hinf_step_response.svg, hinf_step_response.png\n";
    
    // Figure 2: Bode Plot
    figure(1000, 600);
    suptitle("Bode Diagram - Loop Transfer Function L(s) = G(s)C(s)");
    BodeOptions bode_opts;
    bode_opts.margins = true;
    bode_opts.grid = true;
    bode(L, bode_opts);
    savefig("hinf_bode.svg");
    savefig("hinf_bode.png");
    std::cout << "  Saved: hinf_bode.svg, hinf_bode.png\n";
    
    // Figure 3: Nyquist Plot
    figure(700, 700);
    title("Nyquist Diagram");
    NyquistOptions nyq_opts;
    nyq_opts.grid = true;
    nyquist(L, nyq_opts);
    savefig("hinf_nyquist.svg");
    std::cout << "  Saved: hinf_nyquist.svg\n";
    
    // Figure 4: Sensitivity Functions
    figure(1000, 500);
    suptitle("Sensitivity Functions");
    layout(1, 2);
    
    // Generate frequency data
    auto omega_vec = logspace(-2, 3, 200);
    std::vector<double> S_mag, T_mag;
    for (double w : omega_vec) {
        std::complex<double> jw(0, w);
        S_mag.push_back(20 * std::log10(std::abs(S.eval(jw))));
        T_mag.push_back(20 * std::log10(std::abs(T.eval(jw))));
    }
    
    subplot(1, 2, 1);
    xscale("log");
    plot(omega_vec, S_mag, "-", opts({{"color", "blue"}, {"linewidth", "2"}, {"label", "|S(jw)|"}}));
    plot(omega_vec, T_mag, "-", opts({{"color", "red"}, {"linewidth", "2"}, {"label", "|T(jw)|"}}));
    axhline(0, opts({{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.5"}}));
    axhline(6, opts({{"color", "green"}, {"linestyle", ":"}, {"alpha", "0.7"}}));
    xlabel("Frequency (rad/s)");
    ylabel("Magnitude (dB)");
    title("Sensitivity & Complementary Sensitivity");
    legend(true);
    grid(true);
    
    // Robust stability test |W_delta * T| < 1
    subplot(1, 2, 2);
    xscale("log");
    double r0 = 0.25, r_inf = 1.5, tau_w = 0.05;
    std::vector<double> WdT;
    for (size_t i = 0; i < omega_vec.size(); i++) {
        std::complex<double> jw(0, omega_vec[i]);
        std::complex<double> W_delta = (tau_w * jw + r0) / (tau_w/r_inf * jw + 1.0);
        std::complex<double> T_val = T.eval(jw);
        WdT.push_back(std::abs(W_delta * T_val));
    }
    plot(omega_vec, WdT, "-", opts({{"color", "red"}, {"linewidth", "2"}, {"label", "|W_delta * T|"}}));
    axhline(1.0, opts({{"color", "black"}, {"linestyle", "--"}, {"linewidth", "2"}}));
    xlabel("Frequency (rad/s)");
    ylabel("|W_delta * T|");
    title("Robust Stability: ||W_delta*T||_inf < 1");
    legend(true);
    grid(true);
    ylim(0, 1.5);
    
    savefig("hinf_sensitivity.svg");
    savefig("hinf_sensitivity.png");
    std::cout << "  Saved: hinf_sensitivity.svg, hinf_sensitivity.png\n";
    
    // Figure 5: Pole-Zero Map
    figure(600, 600);
    title("Closed-Loop Pole-Zero Map");
    pzmap(T);
    savefig("hinf_pzmap.svg");
    std::cout << "  Saved: hinf_pzmap.svg\n";
    
    // Show all figures
    std::cout << "\n=== All plots generated! ===\n";
    show();
    
    return 0;
}
