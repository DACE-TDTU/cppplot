/**
 * @file control_advanced_demo.cpp
 * @brief Demo for Nichols chart and Discrete-time systems
 * 
 * Compile:
 *   g++ -std=c++14 -D_USE_MATH_DEFINES -I../include control_advanced_demo.cpp -o control_advanced_demo.exe
 */

#include <cppplot/control/control.hpp>
#include <iostream>
#include <iomanip>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << " CppPlot Control Module - Advanced Demo" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // ========== Part 1: Nichols Chart ==========
    std::cout << "\n[1] NICHOLS CHART DEMO" << std::endl;
    std::cout << "----------------------" << std::endl;
    
    // Classic control system for Nichols chart
    auto G1 = TransferFunction({10}, {1, 3, 2, 0});  // 10 / (s³ + 3s² + 2s)
    
    std::cout << "Open-loop system G(s) = 10 / (s³ + 3s² + 2s)" << std::endl;
    
    auto m1 = margin(G1);
    std::cout << "Gain Margin: " << std::fixed << std::setprecision(2) << m1.Gm_dB << " dB" << std::endl;
    std::cout << "Phase Margin: " << m1.Pm << " deg" << std::endl;
    
    // Single system Nichols chart
    NicholsOptions nopt;
    nopt.show_m_circles = true;
    nopt.show_n_circles = true;
    nopt.show_margins = true;
    nichols(G1, nopt);
    savefig("adv_01_nichols_single.svg");
    std::cout << "✓ Saved: adv_01_nichols_single.svg" << std::endl;
    
    // Multiple systems comparison
    auto G2 = TransferFunction({20}, {1, 3, 2, 0});  // Higher gain
    auto G3 = TransferFunction({5}, {1, 3, 2, 0});   // Lower gain
    
    std::vector<TransferFunction> systems = {G1, G2, G3};
    std::vector<std::string> labels = {"K=10", "K=20 (unstable)", "K=5"};
    
    NicholsOptions nopt2;
    nopt2.show_m_circles = true;
    nichols(systems, labels, nopt2);
    savefig("adv_02_nichols_comparison.svg");
    std::cout << "✓ Saved: adv_02_nichols_comparison.svg" << std::endl;
    
    // ========== Part 2: Discrete-Time Systems ==========
    std::cout << "\n[2] DISCRETE-TIME SYSTEMS" << std::endl;
    std::cout << "-------------------------" << std::endl;
    
    // Create discrete transfer function directly
    // H(z) = 0.2z / (z² - 1.5z + 0.7)
    DiscreteTransferFunction H1({0.2, 0}, {1, -1.5, 0.7}, 0.1);  // Ts = 0.1s
    
    std::cout << "\n--- Discrete TF H(z) ---" << std::endl;
    dsysinfo(H1);
    
    // Discrete Bode diagram
    dbode(H1);
    savefig("adv_03_discrete_bode.svg");
    std::cout << "\n✓ Saved: adv_03_discrete_bode.svg" << std::endl;
    
    // Discrete Pole-Zero map (z-plane)
    dzpmap(H1);
    savefig("adv_04_discrete_pzmap.svg");
    std::cout << "✓ Saved: adv_04_discrete_pzmap.svg" << std::endl;
    
    // Discrete Step Response
    dstep(H1, 60);
    savefig("adv_05_discrete_step.svg");
    std::cout << "✓ Saved: adv_05_discrete_step.svg" << std::endl;
    
    // ========== Part 3: Discretization (c2d) ==========
    std::cout << "\n[3] DISCRETIZATION (Continuous to Discrete)" << std::endl;
    std::cout << "--------------------------------------------" << std::endl;
    
    // Start with continuous first-order system
    auto Gc1 = tf_first_order(1.0, 1.0);  // G(s) = 1 / (s+1)
    std::cout << "\nContinuous: G(s) = 1 / (s + 1)" << std::endl;
    
    double Ts = 0.5;  // 500ms sample time
    
    // Discretize using Tustin method
    auto Hd_tustin = c2d_tustin(Gc1, Ts);
    std::cout << "\nTustin discretization (Ts = " << Ts << "s):" << std::endl;
    std::cout << Hd_tustin.toString() << std::endl;
    
    // Discretize using ZOH method
    auto Hd_zoh = c2d_zoh(Gc1, Ts);
    std::cout << "\nZOH discretization (Ts = " << Ts << "s):" << std::endl;
    std::cout << Hd_zoh.toString() << std::endl;
    
    // Compare step responses
    figure(800, 500);
    
    // Continuous step response (approximated)
    std::vector<double> t_cont, y_cont;
    for (double t = 0; t <= 5; t += 0.05) {
        t_cont.push_back(t);
        y_cont.push_back(1.0 - std::exp(-t));  // Known solution for 1/(s+1)
    }
    plot(t_cont, y_cont, "-", opts({{"color", "black"}, {"linewidth", "2"}, {"label", "Continuous"}}));
    
    // Tustin discrete response
    std::vector<double> t_tustin, y_tustin;
    double y_t = 0;
    double alpha_t = -Hd_tustin.den.coeffs[1];
    double b0_t = Hd_tustin.num.coeffs[0];
    double b1_t = Hd_tustin.num.coeffs[1];
    for (int k = 0; k < 11; ++k) {
        double u_k = 1.0;
        double u_km1 = (k > 0) ? 1.0 : 0.0;
        double y_km1 = (k > 0) ? y_tustin.back() : 0;
        double y_k = alpha_t * y_km1 + b0_t * u_k + b1_t * u_km1;
        t_tustin.push_back(k * Ts);
        y_tustin.push_back(y_k);
        y_t = y_k;
    }
    scatter(t_tustin, y_tustin, opts({{"s", "40"}, {"color", "blue"}, {"marker", "o"}, {"label", "Tustin"}}));
    
    // ZOH discrete response (analytical)
    std::vector<double> t_zoh, y_zoh;
    double alpha_z = std::exp(-Ts);
    for (int k = 0; k <= 10; ++k) {
        double y_k = 1.0 - std::pow(alpha_z, k);
        t_zoh.push_back(k * Ts);
        y_zoh.push_back(y_k);
    }
    scatter(t_zoh, y_zoh, opts({{"s", "30"}, {"color", "red"}, {"marker", "x"}, {"label", "ZOH"}}));
    
    xlabel("Time (s)");
    ylabel("Amplitude");
    title("Continuous vs Discrete Step Response");
    legend();
    grid(true);
    savefig("adv_06_c2d_comparison.svg");
    std::cout << "\n✓ Saved: adv_06_c2d_comparison.svg" << std::endl;
    
    // ========== Part 4: Second-order Discretization ==========
    std::cout << "\n[4] SECOND-ORDER DISCRETIZATION" << std::endl;
    std::cout << "--------------------------------" << std::endl;
    
    // Continuous second-order underdamped
    auto Gc2 = tf_second_order(5.0, 0.3);  // wn=5, zeta=0.3
    std::cout << "\nContinuous: G(s) = 25 / (s² + 3s + 25)" << std::endl;
    std::cout << "  wn = 5 rad/s, zeta = 0.3" << std::endl;
    
    Ts = 0.05;  // 50ms sample time
    auto Hd2 = c2d_tustin(Gc2, Ts);
    
    std::cout << "\nDiscrete (Tustin, Ts = " << Ts << "s):" << std::endl;
    std::cout << Hd2.toString() << std::endl;
    
    std::cout << "  Stable: " << (Hd2.isStable() ? "Yes" : "No") << std::endl;
    std::cout << "  Poles: ";
    auto dp = Hd2.poles();
    for (const auto& p : dp) {
        std::cout << "|z|=" << std::abs(p) << " ";
    }
    std::cout << std::endl;
    
    dstep(Hd2, 100);
    savefig("adv_07_second_order_discrete.svg");
    std::cout << "✓ Saved: adv_07_second_order_discrete.svg" << std::endl;
    
    // ========== Part 5: Discrete System Design ==========
    std::cout << "\n[5] DISCRETE LOWPASS FILTER" << std::endl;
    std::cout << "----------------------------" << std::endl;
    
    // Design a discrete lowpass filter
    double tau = 0.1;  // Time constant
    Ts = 0.01;         // 10ms sample
    
    auto Hlp = dtf_lowpass(tau, Ts);
    std::cout << "\nDiscrete Lowpass Filter (τ=" << tau << "s, Ts=" << Ts << "s):" << std::endl;
    std::cout << Hlp.toString() << std::endl;
    
    dbode(Hlp);
    savefig("adv_08_discrete_lowpass_bode.svg");
    std::cout << "✓ Saved: adv_08_discrete_lowpass_bode.svg" << std::endl;
    
    // ========== Summary ==========
    std::cout << "\n========================================" << std::endl;
    std::cout << " DEMO COMPLETE - 8 plots generated" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "\nGenerated files:" << std::endl;
    std::cout << "  1. adv_01_nichols_single.svg" << std::endl;
    std::cout << "  2. adv_02_nichols_comparison.svg" << std::endl;
    std::cout << "  3. adv_03_discrete_bode.svg" << std::endl;
    std::cout << "  4. adv_04_discrete_pzmap.svg" << std::endl;
    std::cout << "  5. adv_05_discrete_step.svg" << std::endl;
    std::cout << "  6. adv_06_c2d_comparison.svg" << std::endl;
    std::cout << "  7. adv_07_second_order_discrete.svg" << std::endl;
    std::cout << "  8. adv_08_discrete_lowpass_bode.svg" << std::endl;
    
    return 0;
}
