/**
 * @file control_design_demo.cpp
 * @brief Comprehensive demo of control system design tools
 * 
 * Demonstrates:
 * - State-space systems and conversions
 * - Pole placement (Ackermann's formula)
 * - LQR controller design
 * - Observer design
 * - Sensitivity functions
 * - PID tuning methods
 * - Lead/Lag compensator design
 * 
 * Compile:
 *   g++ -std=c++14 -D_USE_MATH_DEFINES -I../include control_design_demo.cpp -o control_design_demo.exe
 */

#include <cppplot/control/control.hpp>
#include <iostream>
#include <iomanip>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    std::cout << "================================================" << std::endl;
    std::cout << " CppPlot Control Design Tools - Comprehensive Demo" << std::endl;
    std::cout << "================================================" << std::endl;
    
    // ========================================
    // PART 1: STATE-SPACE SYSTEMS
    // ========================================
    std::cout << "\n[1] STATE-SPACE SYSTEMS" << std::endl;
    std::cout << "------------------------" << std::endl;
    
    // DC Motor model: J*theta'' + b*theta' = K*u
    // States: x1 = theta, x2 = theta'
    // A = [0, 1; 0, -b/J], B = [0; K/J], C = [1, 0]
    
    double J = 0.01;   // Moment of inertia
    double b = 0.1;    // Damping
    double K = 0.01;   // Motor constant
    
    Matrix A = {{0, 1}, {0, -b/J}};
    Matrix B = {{0}, {K/J}};
    Matrix C = {{1, 0}};
    Matrix D = {{0}};
    
    StateSpace motor(A, B, C, D);
    
    std::cout << "\n--- DC Motor State-Space Model ---" << std::endl;
    ssinfo(motor);
    
    // Convert to transfer function
    std::cout << "\n--- Conversion to Transfer Function ---" << std::endl;
    try {
        auto G_motor = ss2tf(motor);
        std::cout << "G(s) = " << std::endl << G_motor.toString() << std::endl;
    } catch (const std::exception& e) {
        std::cout << "Note: " << e.what() << std::endl;
    }
    
    // Create a simple second-order system for demonstrations
    // G(s) = 1 / (s^2 + 2s + 1) - Double integrator with damping
    TransferFunction G({1}, {1, 2, 1});
    std::cout << "\n--- Test System ---" << std::endl;
    std::cout << "G(s) = 1 / (s^2 + 2s + 1)" << std::endl;
    
    // Convert TF to state-space
    auto sys = tf2ss(G);
    std::cout << "\n--- TF to State-Space ---" << std::endl;
    ssinfo(sys);
    
    // ========================================
    // PART 2: POLE PLACEMENT
    // ========================================
    std::cout << "\n\n[2] POLE PLACEMENT (Ackermann's Formula)" << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    
    // Design for DC motor position control
    // Open-loop poles are at 0 and -b/J = -10
    // Desired closed-loop poles: -5 ± 5j (faster, some overshoot)
    
    std::vector<std::complex<double>> desired_poles = {
        std::complex<double>(-5, 5),
        std::complex<double>(-5, -5)
    };
    
    std::cout << "\nDesired poles: -5 + 5j, -5 - 5j" << std::endl;
    std::cout << "  Damping ratio: " << std::fixed << std::setprecision(3) 
              << 5.0 / std::sqrt(50) << std::endl;
    std::cout << "  Natural frequency: " << std::sqrt(50) << " rad/s" << std::endl;
    
    try {
        auto K_fb = place_complex(motor.A, motor.B, desired_poles);
        print_controller_gains("Pole Placement", K_fb);
        
        // Calculate closed-loop A matrix: A - BK
        Matrix K_mat(1, 2);
        K_mat(0, 0) = K_fb[0];
        K_mat(0, 1) = K_fb[1];
        Matrix A_cl = motor.A - motor.B * K_mat;
        
        std::cout << "\nClosed-loop A matrix:" << std::endl;
        std::cout << A_cl.toString() << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }
    
    // ========================================
    // PART 3: LQR CONTROLLER
    // ========================================
    std::cout << "\n\n[3] LQR CONTROLLER DESIGN" << std::endl;
    std::cout << "--------------------------" << std::endl;
    
    // LQR for mass-spring-damper: m*x'' + c*x' + k*x = u
    double m = 1.0, c = 0.5, k = 2.0;
    Matrix A_msd = {{0, 1}, {-k/m, -c/m}};
    Matrix B_msd = {{0}, {1/m}};
    
    std::cout << "\nMass-Spring-Damper System:" << std::endl;
    std::cout << "  m=" << m << ", c=" << c << ", k=" << k << std::endl;
    
    // LQR weights: penalize position error and control effort
    std::vector<double> Q_diag = {10.0, 1.0};  // Position more important
    double R_val = 0.1;  // Allow more control effort
    
    std::cout << "\nLQR weights: Q = diag(" << Q_diag[0] << ", " << Q_diag[1] 
              << "), R = " << R_val << std::endl;
    
    auto K_lqr = lqr_simple(A_msd, B_msd, Q_diag, R_val);
    print_controller_gains("LQR", K_lqr);
    
    // ========================================
    // PART 4: OBSERVER DESIGN
    // ========================================
    std::cout << "\n\n[4] OBSERVER (LUENBERGER) DESIGN" << std::endl;
    std::cout << "---------------------------------" << std::endl;
    
    Matrix C_obs = {{1, 0}};  // Only position is measured
    
    // Observer poles should be 2-5x faster than controller poles
    std::vector<double> obs_poles = {-20, -25};
    
    std::cout << "\nObserver poles: " << obs_poles[0] << ", " << obs_poles[1] << std::endl;
    std::cout << "(2-5x faster than controller)" << std::endl;
    
    try {
        auto L = place_observer(A_msd, C_obs, obs_poles);
        print_controller_gains("Observer", L);
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }
    
    // ========================================
    // PART 5: PID TUNING
    // ========================================
    std::cout << "\n\n[5] PID TUNING METHODS" << std::endl;
    std::cout << "----------------------" << std::endl;
    
    // Plant: First-order plus dead time (FOPDT)
    // G(s) ≈ K * e^(-Ls) / (Ts + 1)
    double Kp_plant = 2.0;   // Process gain
    double T_plant = 5.0;    // Time constant
    double L_plant = 1.0;    // Dead time
    
    std::cout << "\nFOPDT Plant: K=" << Kp_plant << ", T=" << T_plant 
              << ", L=" << L_plant << std::endl;
    
    // Ziegler-Nichols (requires Ku and Tu from experiment)
    // Assuming Ku = 5, Tu = 4 from ultimate cycling
    double Ku = 5.0, Tu = 4.0;
    std::cout << "\nZiegler-Nichols (Ku=" << Ku << ", Tu=" << Tu << "):" << std::endl;
    
    auto pid_zn = ziegler_nichols(Ku, Tu, "PID");
    print_pid_gains("  Z-N PID", pid_zn);
    
    auto pid_zn_no = ziegler_nichols(Ku, Tu, "PID_no_overshoot");
    print_pid_gains("  Z-N No-Overshoot", pid_zn_no);
    
    // Cohen-Coon method
    std::cout << "\nCohen-Coon method:" << std::endl;
    
    auto pid_cc = cohen_coon(Kp_plant, T_plant, L_plant, "PID");
    print_pid_gains("  Cohen-Coon PID", pid_cc);
    
    // ========================================
    // PART 6: LEAD/LAG COMPENSATOR
    // ========================================
    std::cout << "\n\n[6] LEAD/LAG COMPENSATOR DESIGN" << std::endl;
    std::cout << "--------------------------------" << std::endl;
    
    // Plant: G(s) = 10 / (s(s+2))
    TransferFunction G_plant({10}, {1, 2, 0});
    
    std::cout << "\nPlant: G(s) = 10 / (s(s+2))" << std::endl;
    
    auto margins_ol = margin(G_plant);
    std::cout << "Open-loop Phase Margin: " << std::fixed << std::setprecision(1) 
              << margins_ol.Pm << " deg" << std::endl;
    
    // Design lead compensator for PM = 45 deg at wc = 5 rad/s
    double target_PM = 45;
    double wc = 5.0;
    double current_phase = G_plant.phase_deg(wc);
    double phi_lead = target_PM - (180 + current_phase) + 10;  // +10 deg margin
    
    std::cout << "\nTarget: PM = " << target_PM << " deg at wc = " << wc << " rad/s" << std::endl;
    std::cout << "Current phase at wc: " << current_phase << " deg" << std::endl;
    std::cout << "Required phase lead: " << phi_lead << " deg" << std::endl;
    
    auto Gc_lead = design_lead(phi_lead, wc);
    std::cout << "\nLead Compensator:" << std::endl;
    std::cout << Gc_lead.toString() << std::endl;
    
    // Check new margins
    auto G_comp = Gc_lead * G_plant;
    auto margins_comp = margin(G_comp);
    std::cout << "Compensated Phase Margin: " << margins_comp.Pm << " deg" << std::endl;
    
    // ========================================
    // PART 7: SENSITIVITY FUNCTIONS
    // ========================================
    std::cout << "\n\n[7] SENSITIVITY FUNCTIONS" << std::endl;
    std::cout << "-------------------------" << std::endl;
    
    // Controller: PID tuned by Ziegler-Nichols
    auto C_pid = pid_tf(pid_zn);
    
    std::cout << "\nPlant: G(s) = 10/(s(s+2))" << std::endl;
    std::cout << "Controller: Z-N PID" << std::endl;
    
    // Plot sensitivity functions
    sensitivity_plot(G_plant, C_pid);
    savefig("des_07_sensitivity.svg");
    std::cout << "\n✓ Saved: des_07_sensitivity.svg" << std::endl;
    
    // ========================================
    // PART 8: CLOSED-LOOP COMPARISON
    // ========================================
    std::cout << "\n\n[8] CLOSED-LOOP COMPARISON" << std::endl;
    std::cout << "--------------------------" << std::endl;
    
    // Compare different controllers
    TransferFunction G_simple({1}, {1, 1, 0});  // 1/(s(s+1))
    
    // P controller
    TransferFunction C_P({5}, {1});
    auto T_P = closed_loop(G_simple, C_P);
    
    // PI controller
    auto C_PI = systems::pi(5, 2);
    auto T_PI = closed_loop(G_simple, C_PI);
    
    // PID controller
    auto C_PID = systems::pid(5, 2, 1);
    auto T_PID = closed_loop(G_simple, C_PID);
    
    std::cout << "\nPlant: G(s) = 1/(s(s+1))" << std::endl;
    std::cout << "\nControllers:" << std::endl;
    std::cout << "  P:   Kp = 5" << std::endl;
    std::cout << "  PI:  Kp = 5, Ki = 2" << std::endl;
    std::cout << "  PID: Kp = 5, Ki = 2, Kd = 1" << std::endl;
    
    // Step response comparison
    figure(800, 500);
    
    auto t = linspace(0, 10, 500);
    
    auto result_P = step_data(T_P, t);
    auto result_PI = step_data(T_PI, t);
    auto result_PID = step_data(T_PID, t);
    
    auto y_P = result_P.second;
    auto y_PI = result_PI.second;
    auto y_PID = result_PID.second;
    
    plot(t, y_P, "-", opts({{"color", "#1f77b4"}, {"linewidth", "2"}, {"label", "P"}}));
    plot(t, y_PI, "-", opts({{"color", "#ff7f0e"}, {"linewidth", "2"}, {"label", "PI"}}));
    plot(t, y_PID, "-", opts({{"color", "#2ca02c"}, {"linewidth", "2"}, {"label", "PID"}}));
    
    axhline(1.0, opts({{"color", "black"}, {"linestyle", "--"}, {"linewidth", "0.8"}}));
    
    xlabel("Time (s)");
    ylabel("Output");
    title("Step Response Comparison: P vs PI vs PID");
    legend(true);
    grid(true);
    savefig("des_08_pid_comparison.svg");
    std::cout << "\n✓ Saved: des_08_pid_comparison.svg" << std::endl;
    
    // ========================================
    // PART 9: BODE PLOTS FOR DESIGNED SYSTEM
    // ========================================
    std::cout << "\n\n[9] BODE PLOTS FOR COMPENSATED SYSTEM" << std::endl;
    std::cout << "--------------------------------------" << std::endl;
    
    std::vector<TransferFunction> systems = {G_plant, G_comp};
    std::vector<std::string> labels = {"Original", "With Lead Comp."};
    
    bode(systems, labels);
    savefig("des_09_bode_comparison.svg");
    std::cout << "✓ Saved: des_09_bode_comparison.svg" << std::endl;
    
    // ========================================
    // SUMMARY
    // ========================================
    std::cout << "\n================================================" << std::endl;
    std::cout << " DEMO COMPLETE - 3 plots generated" << std::endl;
    std::cout << "================================================" << std::endl;
    std::cout << "\nFiles generated:" << std::endl;
    std::cout << "  1. des_07_sensitivity.svg   - Sensitivity functions" << std::endl;
    std::cout << "  2. des_08_pid_comparison.svg - P/PI/PID comparison" << std::endl;
    std::cout << "  3. des_09_bode_comparison.svg - Bode with/without compensator" << std::endl;
    
    std::cout << "\n\nKey Features Demonstrated:" << std::endl;
    std::cout << "  - State-space representation and conversions" << std::endl;
    std::cout << "  - Pole placement using Ackermann's formula" << std::endl;
    std::cout << "  - LQR optimal control design" << std::endl;
    std::cout << "  - Luenberger observer design" << std::endl;
    std::cout << "  - PID tuning (Ziegler-Nichols, Cohen-Coon)" << std::endl;
    std::cout << "  - Lead/Lag compensator design" << std::endl;
    std::cout << "  - Sensitivity function analysis" << std::endl;
    
    return 0;
}
