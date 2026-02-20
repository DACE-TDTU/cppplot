/**
 * @file control_module_demo.cpp
 * @brief Demo showcasing the CppPlot Control Systems Module
 * 
 * This demonstrates the new one-liner API similar to Python Control.
 * 
 * Compile: g++ -std=c++14 -D_USE_MATH_DEFINES -I../include control_module_demo.cpp -o control_module_demo.exe
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <iomanip>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    std::cout << "=============================================" << std::endl;
    std::cout << "  CppPlot Control Systems Module Demo" << std::endl;
    std::cout << "  Version 1.5.0" << std::endl;
    std::cout << "=============================================" << std::endl;
    
    // ============================================================
    // 1. Transfer Function Basics
    // ============================================================
    std::cout << "\n1. Transfer Function Basics\n";
    std::cout << "----------------------------\n";
    
    // Create from coefficients: G(s) = 100 / (s² + 6s + 100)
    // This is wn=10, zeta=0.3
    TransferFunction G1({100}, {1, 6, 100});
    std::cout << "G1(s) from coefficients:\n" << G1.toString() << "\n\n";
    
    // Create using factory function
    auto G2 = tf_second_order(10.0, 0.3);  // wn=10, zeta=0.3
    std::cout << "G2(s) from tf_second_order(10, 0.3):\n" << G2.toString() << "\n\n";
    
    // Create from zeros, poles, gain - use explicit vector
    std::vector<double> zeros_vec = {-5.0};
    std::vector<double> poles_vec = {-1.0, -2.0};
    auto G3 = zpk(zeros_vec, poles_vec, 2.0);  // K(s+5)/((s+1)(s+2))
    std::cout << "G3(s) from zpk({-5}, {-1,-2}, 2):\n" << G3.toString() << "\n\n";
    
    // ============================================================
    // 2. System Properties
    // ============================================================
    std::cout << "\n2. System Properties\n";
    std::cout << "--------------------\n";
    
    std::cout << "G2 Analysis:\n";
    std::cout << "  Order: " << G2.order() << "\n";
    std::cout << "  DC Gain: " << G2.dcgain() << "\n";
    std::cout << "  Stable: " << (G2.isStable() ? "Yes" : "No") << "\n";
    
    auto p = G2.poles();
    std::cout << "  Poles:\n";
    for (size_t i = 0; i < p.size(); ++i) {
        std::cout << "    p" << (i+1) << " = " << std::fixed << std::setprecision(3)
                  << p[i].real() << " + " << p[i].imag() << "j\n";
    }
    
    // ============================================================
    // 3. Stability Margins
    // ============================================================
    std::cout << "\n3. Stability Margins\n";
    std::cout << "--------------------\n";
    
    // Open-loop system for margin analysis
    auto G_ol = systems::type1_3pole(2.0, 1.0, 2.0);  // 2/(s(s+1)(s+2))
    std::cout << "Open-loop G(s) = 2/(s(s+1)(s+2)):\n";
    
    auto m = margin(G_ol);
    std::cout << "  Gain Margin: " << std::fixed << std::setprecision(2) 
              << m.Gm_dB << " dB at " << m.Wpc << " rad/s\n";
    std::cout << "  Phase Margin: " << m.Pm << " deg at " << m.Wgc << " rad/s\n";
    std::cout << "  Closed-loop stable: " << (m.stable ? "Yes" : "No") << "\n";
    
    // ============================================================
    // 4. Step Response Analysis  
    // ============================================================
    std::cout << "\n4. Step Response Analysis\n";
    std::cout << "-------------------------\n";
    
    auto info = stepinfo(G2);
    std::cout << "G2 Step Response:\n";
    std::cout << "  Rise Time: " << std::fixed << std::setprecision(3) 
              << info.RiseTime << " s\n";
    std::cout << "  Settling Time: " << info.SettlingTime << " s\n";
    std::cout << "  Overshoot: " << std::setprecision(1) << info.Overshoot << "%\n";
    std::cout << "  Peak: " << std::setprecision(3) << info.Peak << "\n";
    std::cout << "  Steady State: " << info.SteadyState << "\n";
    
    // ============================================================
    // 5. Generate Plots
    // ============================================================
    std::cout << "\n5. Generating Plots\n";
    std::cout << "-------------------\n";
    
    // --- Step Response ---
    std::cout << "  [1/6] Step Response...\n";
    step(G2);
    title("Step Response - Second Order System (\\omega_n=10, \\zeta=0.3)");
    savefig("output/ctrl_01_step.svg");
    clf();
    
    // --- Multiple Step Responses ---
    std::cout << "  [2/6] Comparing Step Responses...\n";
    std::vector<TransferFunction> systems = {
        tf_second_order(10, 0.1),
        tf_second_order(10, 0.3),
        tf_second_order(10, 0.5),
        tf_second_order(10, 0.707),
        tf_second_order(10, 1.0)
    };
    std::vector<std::string> labels = {
        "\\zeta=0.1", "\\zeta=0.3", "\\zeta=0.5", 
        "\\zeta=0.707", "\\zeta=1.0"
    };
    step(systems, labels);
    title("Step Response Comparison");
    savefig("output/ctrl_02_step_compare.svg");
    clf();
    
    // --- Bode Plot ---
    std::cout << "  [3/6] Bode Plot...\n";
    BodeOptions bode_opts;
    bode_opts.margins = true;
    bode(G_ol, bode_opts);
    suptitle("Bode Plot - G(s) = 2/(s(s+1)(s+2))");
    savefig("output/ctrl_03_bode.svg");
    clf();
    
    // --- Nyquist Plot ---
    std::cout << "  [4/6] Nyquist Plot...\n";
    nyquist(G_ol);
    savefig("output/ctrl_04_nyquist.svg");
    clf();
    
    // --- Pole-Zero Map ---
    std::cout << "  [5/6] Pole-Zero Map...\n";
    pzmap(G2);
    savefig("output/ctrl_05_pzmap.svg");
    clf();
    
    // --- Root Locus ---
    std::cout << "  [6/6] Root Locus...\n";
    // G(s) = 1/(s(s+1)(s+2)) for root locus
    auto G_rl = TransferFunction({1}, {1, 3, 2, 0});
    rlocus(G_rl);
    savefig("output/ctrl_06_rlocus.svg");
    clf();
    
    // ============================================================
    // 6. System Connections Demo
    // ============================================================
    std::cout << "\n6. System Connections\n";
    std::cout << "---------------------\n";
    
    auto G_plant = tf_second_order(5, 0.5);
    auto G_controller = systems::pi(2.0, 1.0);  // PI controller
    
    std::cout << "Plant G(s):\n" << G_plant.toString() << "\n\n";
    std::cout << "Controller C(s) = PI(2, 1):\n" << G_controller.toString() << "\n\n";
    
    // Open-loop
    auto G_open = G_controller * G_plant;
    std::cout << "Open-loop L(s) = C(s)*G(s):\n" << G_open.toString() << "\n\n";
    
    // Closed-loop with unity feedback
    auto G_closed = feedback(G_open);
    std::cout << "Closed-loop T(s) = L/(1+L):\n";
    std::cout << "  Poles: ";
    for (const auto& pole : G_closed.poles()) {
        std::cout << pole.real();
        if (std::abs(pole.imag()) > 1e-10) {
            std::cout << (pole.imag() >= 0 ? "+" : "-") << std::abs(pole.imag()) << "j";
        }
        std::cout << "  ";
    }
    std::cout << "\n";
    std::cout << "  Stable: " << (G_closed.isStable() ? "Yes" : "No") << "\n";
    
    // ============================================================
    // Summary
    // ============================================================
    std::cout << "\n=============================================" << std::endl;
    std::cout << "  Demo Complete!" << std::endl;
    std::cout << "=============================================" << std::endl;
    std::cout << "\nGenerated files in output/:" << std::endl;
    std::cout << "  ctrl_01_step.svg       - Single step response" << std::endl;
    std::cout << "  ctrl_02_step_compare.svg - Multiple damping ratios" << std::endl;
    std::cout << "  ctrl_03_bode.svg       - Bode plot with margins" << std::endl;
    std::cout << "  ctrl_04_nyquist.svg    - Nyquist diagram" << std::endl;
    std::cout << "  ctrl_05_pzmap.svg      - Pole-zero map" << std::endl;
    std::cout << "  ctrl_06_rlocus.svg     - Root locus" << std::endl;
    
    return 0;
}
