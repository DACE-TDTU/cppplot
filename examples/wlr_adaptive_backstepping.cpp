

// Compile: g++ -I../include wlr_adaptive_backstepping.cpp -o wlr_adaptive_backstepping.exe
// /.wlr_adaptive_backstepping.exe

#include <cmath>
#include <vector>
#include <array>
#include <iostream>
#include <iomanip>
#include <string>
#include <algorithm>
#include "cppplot/cppplot.hpp"

using namespace cppplot;

// ═══════════════════════════════════════════════════════════════════════════
// System & Controller Parameters
// ═══════════════════════════════════════════════════════════════════════════

struct RobotParams {
    double theta_star = 9.81; // True unmeasurable gravity/payload parameter
    double c = 2.0;           // Drive train coefficient
    double a = 1.0;           // Motor friction coefficient
};

struct ControlGains {
    double k1 = 4.0;          // Kinematic layer gain
    double k2 = 5.0;          // Dynamic layer gain
    double k3 = 8.0;          // Hardware layer gain
    double gamma = 15.0;      // Adaptation learning rate
};

// ── Diagnostic Data Storage ───────────────────────────────────────────────
struct SimResult {
    std::vector<double> t, x1, x2, x3, u, theta_hat, theta_true;
};

// ── The Main Simulation & Control Loop ────────────────────────────────────
SimResult run_wlr_sim(const RobotParams& plant, const ControlGains& gains, 
                      double t_end = 10.0, double dt = 0.001) {
    SimResult res;
    int N = static_cast<int>(t_end / dt);
    
    // Pre-allocate memory for fast execution
    res.t.reserve(N); res.x1.reserve(N); res.x2.reserve(N); 
    res.x3.reserve(N); res.u.reserve(N); 
    res.theta_hat.reserve(N); res.theta_true.reserve(N);

    // ── Initial Conditions ──
    double x1 = M_PI / 4.0; // Starts at 45 degrees tilt!
    double x2 = 0.0;
    double x3 = 0.0;
    double theta_hat = 0.0; // Controller has NO knowledge of true theta*
    
    double alpha_2_prev = 0.0;

    for (int k = 0; k < N; ++k) {
        double t = k * dt;
        
        // 1. Data Logging
        res.t.push_back(t);
        res.x1.push_back(x1);
        res.x2.push_back(x2);
        res.x3.push_back(x3);
        res.theta_hat.push_back(theta_hat);
        res.theta_true.push_back(plant.theta_star);

        // ═══════════════════════════════════════════════════════════════════
        // EMBEDDED CONTROL ALGORITHM (Portable to MCU)
        // ═══════════════════════════════════════════════════════════════════
        
        // --- Layer 1: Kinematic Tracking ---
        double alpha_1 = -gains.k1 * x1;
        double z2 = x2 - alpha_1;

        // --- Layer 2: Adaptive Dynamics ---
        double alpha_2 = (1.0 / plant.c) * (-x1 - theta_hat * std::sin(x1) - gains.k1 * x2 - gains.k2 * z2);
        
        // Dirty derivative for alpha_2 (in hardware, pass through a low-pass filter)
        double d_alpha_2 = (k > 0) ? (alpha_2 - alpha_2_prev) / dt : 0.0;
        alpha_2_prev = alpha_2;
        
        double z3 = x3 - alpha_2;
        
        // Parameter Adaptation Law
        double d_theta_hat = gains.gamma * z2 * std::sin(x1);

        // --- Layer 3: Hardware Actuator & Control Law ---
        double u = -plant.c * z2 + plant.a * x3 + d_alpha_2 - gains.k3 * z3;
        
        res.u.push_back(std::clamp(u, -48.0, 48.0));

        // ═══════════════════════════════════════════════════════════════════
        // PLANT DYNAMICS (Simulation Only)
        // ═══════════════════════════════════════════════════════════════════
        double dx1 = x2;
        double dx2 = plant.theta_star * std::sin(x1) + plant.c * x3;
        double dx3 = -plant.a * x3 + u;

        // Euler Integration
        x1 += dx1 * dt;
        x2 += dx2 * dt;
        x3 += dx3 * dt;
        theta_hat += d_theta_hat * dt;
    }
    return res;
}
int main(){
    // exec
// ═══════════════════════════════════════════════════════════════════════════
// RUN SIMULATION AND PLOT RESULTS
// ═══════════════════════════════════════════════════════════════════════════

RobotParams plant;
ControlGains gains;

// Run the C++ simulation
auto res = run_wlr_sim(plant, gains, 3.0, 0.001);

std::printf("\n╔══════════════════════════════════════════════════════════════╗\n");
std::printf(  "║  SIMULATION COMPLETE: Adaptive Backstepping for WLR          ║\n");
std::printf(  "╚══════════════════════════════════════════════════════════════╝\n");
std::printf("  Initial Angle    : %.2f deg\n", 45.0);
std::printf("  True Theta* : %.3f\n", plant.theta_star);
std::printf("  Estimated Theta  : %.3f (Final)\n\n", res.theta_hat.back());

// ── Generate IEEE Standard 3-Panel Plot ───────────────────────────────────
figure(1000, 1000);

// Panel 1: States
subplot(3, 1, 1);
plot(res.t, res.x1, "b-",  {{"linewidth", "2.0"}, {"label", "Pitch Angle x_1 (rad)"}});
plot(res.t, res.x2, "r--", {{"linewidth", "2.0"}, {"label", "Pitch Rate x_2 (rad/s)"}});
xlabel("Time [s]");
ylabel("States");
title("System State Convergence");
legend(); grid(true);

// Panel 2: Control Effort
subplot(3, 1, 2);
plot(res.t, res.u, "g-", {{"linewidth", "2.0"}, {"label", "Control Voltage u(t)"}});
xlabel("Time [s]");
ylabel("Control Input [V]");
title("Control Effort (Chattering-Free)");
legend(); grid(true);

// Panel 3: Adaptive Parameter
subplot(3, 1, 3);
plot(res.t, res.theta_hat, "m-", {{"linewidth", "2.0"}, {"label", "Estimated \\hat{\\theta}"}});
plot(res.t, res.theta_true, "k:", {{"linewidth", "2.0"}, {"label", "True \\theta^*"}});
xlabel("Time [s]");
ylabel("Parameter Value");
title("Adaptive Parameter Convergence (Note: PE condition)");
legend(); grid(true);

// Save for publication
savefig("wlr_adaptive_backstepping.svg");
std::cout << "Saved: wlr_adaptive_backstepping.svg\n";

}