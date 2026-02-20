/**
 * @file advanced_smc_demo.cpp
 * @brief Demonstration of Advanced Sliding Mode Control algorithms
 *
 * This example demonstrates the newest SMC algorithms:
 * 1. Fixed-Time SMC (FxTSMC) - Polyakov (2012)
 * 2. Event-Triggered SMC (ETSMC) - Resource-efficient control
 * 3. Barrier Function SMC - State constraint satisfaction
 * 4. Disturbance Observer SMC (DOBSMC) - Active disturbance rejection
 *
 * Compile:
 *   g++ -std=c++14 -O2 -Wno-unused-parameter -I../include advanced_smc_demo.cpp -o advanced_smc_demo
 *
 * Run:
 *   ./advanced_smc_demo
 */

#include <cppplot/control/control.hpp>
#include <iostream>
#include <iomanip>

using namespace cppplot;
using namespace cppplot::control;
using namespace cppplot::control::nonlinear;

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║       ADVANCED SLIDING MODE CONTROL DEMONSTRATION                ║\n";
    std::cout << "║   Fixed-Time | Event-Triggered | Barrier | Disturbance Observer  ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════╝\n\n";

    // Common setup
    std::vector<double> x0 = {3.0, 1.0};  // Start at (3, 1)
    double t_final = 5.0;
    double dt = 0.001;

    auto reference = [](double t) { return 0.0; };
    
    // Matched disturbance
    auto disturbance = [](double t) { 
        return 0.5 * std::sin(3.0 * t) + 0.2 * std::cos(7.0 * t);
    };

    // ================================================================
    // Example 1: Fixed-Time SMC (FxTSMC)
    // ================================================================
    std::cout << "═══════════════════════════════════════════════════════════════════\n";
    std::cout << "Example 1: Fixed-Time SMC (FxTSMC)\n";
    std::cout << "   Based on Polyakov (2012)\n";
    std::cout << "   Convergence time bounded INDEPENDENT of initial conditions!\n";
    std::cout << "   Control: u = -k1|s|^p·sign(s) - k2|s|^q·sign(s)\n";
    std::cout << "   T_max = 1/(k1(1-p)) + 1/(k2(q-1))\n";
    std::cout << "═══════════════════════════════════════════════════════════════════\n\n";

    // Test with different initial conditions to show fixed-time property
    std::vector<std::vector<double>> x0_tests = {
        {1.0, 0.5},   // Small IC
        {3.0, 1.0},   // Medium IC
        {5.0, 2.0},   // Large IC
        {10.0, 3.0}   // Very large IC
    };
    
    std::vector<SMCSimulationResult> fxt_results;
    std::vector<std::string> fxt_labels;
    
    double k1 = 5.0, k2 = 5.0, p = 0.5, q = 1.5;
    double T_max_theoretical = 1.0/(k1*(1.0-p)) + 1.0/(k2*(q-1.0));
    
    std::cout << "Fixed-Time Parameters:\n";
    std::cout << "  k1 = " << k1 << ", k2 = " << k2 << ", p = " << p << ", q = " << q << "\n";
    std::cout << "  T_max (theoretical) = " << std::fixed << std::setprecision(3) 
              << T_max_theoretical << " s\n\n";
    
    std::cout << "Testing with different initial conditions:\n";
    std::cout << "─────────────────────────────────────────────────────────────────\n";
    std::cout << std::setw(20) << "Initial Condition" << std::setw(20) << "Reaching Time" 
              << std::setw(15) << "Status" << "\n";
    std::cout << "─────────────────────────────────────────────────────────────────\n";
    
    for (size_t i = 0; i < x0_tests.size(); ++i) {
        auto pair_fxt = createFixedTimeSMC(5.0, k1, k2, p, q);
        auto ctrl_fxt = pair_fxt.first;
        auto dyn_fxt = pair_fxt.second;
        
        SMCSimulator sim_fxt(ctrl_fxt, dyn_fxt, 2);
        auto result = sim_fxt.simulate(x0_tests[i], t_final, dt, reference, disturbance);
        fxt_results.push_back(result);
        
        std::ostringstream label;
        label << "x0=(" << x0_tests[i][0] << "," << x0_tests[i][1] << ")";
        fxt_labels.push_back(label.str());
        
        std::cout << std::setw(20) << label.str() 
                  << std::setw(17) << std::setprecision(4) << result.reaching_time << " s"
                  << std::setw(15) << (result.reaching_time < T_max_theoretical ? "✓ PASS" : "✗ FAIL") 
                  << "\n";
    }
    std::cout << "─────────────────────────────────────────────────────────────────\n";
    std::cout << "All reaching times should be < T_max = " << T_max_theoretical << " s\n\n";

    // Print detailed summary for one case
    SMCConfig cfg_fxt;
    cfg_fxt.type = SMCType::FIXED_TIME;
    cfg_fxt.fxt_k1 = k1;
    cfg_fxt.fxt_k2 = k2;
    cfg_fxt.fxt_p = p;
    cfg_fxt.fxt_q = q;
    cfg_fxt.use_boundary_layer = true;
    cfg_fxt.boundary_thickness = 0.05;
    printSMCSummary(cfg_fxt, SlidingSurfaceConfig::linear({5.0, 1.0}), fxt_results[1]);

    // Plot comparison for FxTSMC
    plotSMCComparison(fxt_results, fxt_labels, "smc_fixedtime_comparison");
    plotSMCResponse(fxt_results[2], "smc_fixedtime_response");

    // ================================================================
    // Example 2: Event-Triggered SMC (ETSMC)
    // ================================================================
    std::cout << "\n═══════════════════════════════════════════════════════════════════\n";
    std::cout << "Example 2: Event-Triggered SMC (ETSMC)\n";
    std::cout << "   Control updated only when trigger condition violated\n";
    std::cout << "   Benefits: Reduced computation, communication, actuator wear\n";
    std::cout << "   Trigger: ||s(t) - s(tk)|| > σ||u(tk)|| + threshold\n";
    std::cout << "═══════════════════════════════════════════════════════════════════\n\n";

    // Compare conventional vs event-triggered
    auto pair_conv = createDoubleIntegratorSMC(SMCType::CONVENTIONAL, 5.0, 15.0, true);
    auto ctrl_conv = pair_conv.first;
    auto dyn_conv = pair_conv.second;
    SMCSimulator sim_conv(ctrl_conv, dyn_conv, 2);
    auto result_conv = sim_conv.simulate(x0, t_final, dt, reference, disturbance);

    auto pair_et = createEventTriggeredSMC(5.0, 15.0, 0.3, 0.05);
    auto ctrl_et = pair_et.first;
    auto dyn_et = pair_et.second;
    SMCSimulator sim_et(ctrl_et, dyn_et, 2);
    auto result_et = sim_et.simulate(x0, t_final, dt, reference, disturbance);

    std::cout << "Comparison: Conventional SMC vs Event-Triggered SMC\n";
    std::cout << "─────────────────────────────────────────────────────────────────\n";
    std::cout << std::setw(25) << "Metric" << std::setw(20) << "Conventional" 
              << std::setw(20) << "Event-Triggered" << "\n";
    std::cout << "─────────────────────────────────────────────────────────────────\n";
    std::cout << std::setw(25) << "Reaching Time (s)" 
              << std::setw(20) << std::setprecision(4) << result_conv.reaching_time
              << std::setw(20) << result_et.reaching_time << "\n";
    std::cout << std::setw(25) << "Max Chattering" 
              << std::setw(20) << std::setprecision(1) << result_conv.max_chattering
              << std::setw(20) << result_et.max_chattering << "\n";
    
    // Count control updates (approximation)
    int conv_updates = static_cast<int>(result_conv.time.size());
    int et_updates = 0;
    for (size_t i = 1; i < result_et.control.size(); ++i) {
        if (std::abs(result_et.control[i] - result_et.control[i-1]) > 1e-6) {
            et_updates++;
        }
    }
    std::cout << std::setw(25) << "Control Updates"
              << std::setw(20) << conv_updates
              << std::setw(20) << et_updates << "\n";
    double reduction = 100.0 * (1.0 - static_cast<double>(et_updates) / conv_updates);
    std::cout << std::setw(25) << "Update Reduction"
              << std::setw(20) << "baseline"
              << std::setw(19) << std::setprecision(1) << reduction << "%" << "\n";
    std::cout << "─────────────────────────────────────────────────────────────────\n\n";

    SMCConfig cfg_et;
    cfg_et.type = SMCType::EVENT_TRIGGERED;
    cfg_et.K = 15.0;
    cfg_et.et_sigma = 0.3;
    cfg_et.et_threshold = 0.05;
    cfg_et.use_boundary_layer = true;
    printSMCSummary(cfg_et, SlidingSurfaceConfig::linear({5.0, 1.0}), result_et);

    plotSMCComparison({result_conv, result_et}, {"Conventional", "Event-Triggered"}, 
                      "smc_event_triggered_comparison");

    // ================================================================
    // Example 3: Barrier Function SMC
    // ================================================================
    std::cout << "\n═══════════════════════════════════════════════════════════════════\n";
    std::cout << "Example 3: Barrier Function-based SMC\n";
    std::cout << "   Guarantees state constraints |x| < bound\n";
    std::cout << "   Based on Barrier Lyapunov Functions (Tee et al., 2009)\n";
    std::cout << "   V = (1/2)log(kc²/(kc² - x²)) prevents constraint violation\n";
    std::cout << "═══════════════════════════════════════════════════════════════════\n\n";

    double state_bound = 4.0;  // Constraint: |x| < 4
    
    // Start near the constraint
    std::vector<double> x0_constrained = {3.5, 0.5};
    
    // Without barrier (conventional)
    auto pair_no_barrier = createDoubleIntegratorSMC(SMCType::CONVENTIONAL, 5.0, 10.0, true);
    auto ctrl_no_barrier = pair_no_barrier.first;
    auto dyn_no_barrier = pair_no_barrier.second;
    SMCSimulator sim_no_barrier(ctrl_no_barrier, dyn_no_barrier, 2);
    
    // Strong disturbance trying to push state beyond constraint
    auto strong_dist = [](double t) { return 3.0 * std::sin(2.0 * t); };
    auto result_no_barrier = sim_no_barrier.simulate(x0_constrained, 4.0, dt, reference, strong_dist);

    // With barrier function
    auto pair_barrier = createBarrierFunctionSMC(5.0, 10.0, state_bound, 2.0);
    auto ctrl_barrier = pair_barrier.first;
    auto dyn_barrier = pair_barrier.second;
    SMCSimulator sim_barrier(ctrl_barrier, dyn_barrier, 2);
    auto result_barrier = sim_barrier.simulate(x0_constrained, 4.0, dt, reference, strong_dist);

    // Check constraint violations
    auto x1_no_barrier = result_no_barrier.getState(0);
    auto x1_barrier = result_barrier.getState(0);
    
    double max_no_barrier = *std::max_element(x1_no_barrier.begin(), x1_no_barrier.end(),
        [](double a, double b) { return std::abs(a) < std::abs(b); });
    double max_barrier = *std::max_element(x1_barrier.begin(), x1_barrier.end(),
        [](double a, double b) { return std::abs(a) < std::abs(b); });
    max_no_barrier = std::abs(max_no_barrier);
    max_barrier = std::abs(max_barrier);

    std::cout << "State Constraint Test: |x| < " << state_bound << "\n";
    std::cout << "Strong disturbance: d(t) = 3·sin(2t)\n";
    std::cout << "─────────────────────────────────────────────────────────────────\n";
    std::cout << std::setw(25) << "Controller" << std::setw(20) << "Max |x|" 
              << std::setw(20) << "Constraint" << "\n";
    std::cout << "─────────────────────────────────────────────────────────────────\n";
    std::cout << std::setw(25) << "Conventional SMC" 
              << std::setw(20) << std::setprecision(3) << max_no_barrier
              << std::setw(20) << (max_no_barrier < state_bound ? "✓ Satisfied" : "✗ VIOLATED") << "\n";
    std::cout << std::setw(25) << "Barrier Function SMC" 
              << std::setw(20) << max_barrier
              << std::setw(20) << (max_barrier < state_bound ? "✓ Satisfied" : "✗ VIOLATED") << "\n";
    std::cout << "─────────────────────────────────────────────────────────────────\n\n";

    SMCConfig cfg_bf;
    cfg_bf.type = SMCType::BARRIER_FUNCTION;
    cfg_bf.K = 10.0;
    cfg_bf.bf_state_bound = state_bound;
    cfg_bf.bf_kb = 2.0;
    cfg_bf.use_boundary_layer = true;
    printSMCSummary(cfg_bf, SlidingSurfaceConfig::linear({5.0, 1.0}), result_barrier);

    // Custom plot showing constraint
    using namespace cppplot;
    figure(1000, 600);
    suptitle("Barrier Function SMC - State Constraint Enforcement");
    layout(1, 2);
    
    subplot(1, 2, 1);
    plot(result_no_barrier.time, x1_no_barrier, "-", 
         opts({{"color", "red"}, {"linewidth", "2"}, {"label", "Conventional SMC"}}));
    plot(result_barrier.time, x1_barrier, "-", 
         opts({{"color", "blue"}, {"linewidth", "2"}, {"label", "Barrier SMC"}}));
    axhline(state_bound, opts({{"color", "black"}, {"linestyle", "--"}, {"linewidth", "1.5"}}));
    axhline(-state_bound, opts({{"color", "black"}, {"linestyle", "--"}, {"linewidth", "1.5"}}));
    xlabel("Time (s)");
    ylabel("x₁ (position)");
    std::ostringstream title_str;
    title_str << "State Response (Constraint: |x| < " << state_bound << ")";
    title(title_str.str());
    legend(true);
    grid(true);
    
    subplot(1, 2, 2);
    plot(result_no_barrier.time, result_no_barrier.control, "-", 
         opts({{"color", "red"}, {"linewidth", "1"}, {"label", "Conventional"}}));
    plot(result_barrier.time, result_barrier.control, "-", 
         opts({{"color", "blue"}, {"linewidth", "1"}, {"label", "Barrier"}}));
    xlabel("Time (s)");
    ylabel("u(t)");
    title("Control Signal");
    legend(true);
    grid(true);
    
    savefig("smc_barrier_function.svg");
    savefig("smc_barrier_function.png");
    std::cout << "Saved: smc_barrier_function.svg, smc_barrier_function.png\n";

    // ================================================================
    // Example 4: Disturbance Observer-based SMC (DOBSMC)
    // ================================================================
    std::cout << "\n═══════════════════════════════════════════════════════════════════\n";
    std::cout << "Example 4: Disturbance Observer-based SMC (DOBSMC)\n";
    std::cout << "   Active disturbance estimation and compensation\n";
    std::cout << "   Observer: d̂ = z + L·x, ż = -L·(f + gu + d̂)\n";
    std::cout << "   Benefits: Reduced switching gain, less chattering\n";
    std::cout << "═══════════════════════════════════════════════════════════════════\n\n";

    // Compare with conventional under strong disturbance
    double D_max = 5.0;
    auto severe_dist = [D_max](double t) { 
        return D_max * std::sin(5.0 * t) + 0.5 * D_max * std::cos(11.0 * t);
    };

    // Conventional (needs high gain for robustness)
    auto pair_high_gain = createDoubleIntegratorSMC(SMCType::CONVENTIONAL, 5.0, 25.0, true);
    auto ctrl_high_gain = pair_high_gain.first;
    auto dyn_high_gain = pair_high_gain.second;
    SMCSimulator sim_high_gain(ctrl_high_gain, dyn_high_gain, 2);
    auto result_high_gain = sim_high_gain.simulate(x0, t_final, dt, reference, severe_dist);

    // DOBSMC (can use lower gain due to disturbance compensation)
    auto pair_dob = createDisturbanceObserverSMC(5.0, 25.0, 50.0, D_max);
    auto ctrl_dob = pair_dob.first;
    auto dyn_dob = pair_dob.second;
    SMCSimulator sim_dob(ctrl_dob, dyn_dob, 2);
    auto result_dob = sim_dob.simulate(x0, t_final, dt, reference, severe_dist);

    std::cout << "Comparison under severe disturbance: d(t) = 5·sin(5t) + 2.5·cos(11t)\n";
    std::cout << "─────────────────────────────────────────────────────────────────\n";
    std::cout << std::setw(25) << "Metric" << std::setw(20) << "High-Gain SMC" 
              << std::setw(20) << "DOBSMC" << "\n";
    std::cout << "─────────────────────────────────────────────────────────────────\n";
    std::cout << std::setw(25) << "Reaching Time (s)" 
              << std::setw(20) << std::setprecision(4) << result_high_gain.reaching_time
              << std::setw(20) << result_dob.reaching_time << "\n";
    std::cout << std::setw(25) << "Max Chattering" 
              << std::setw(20) << std::setprecision(1) << result_high_gain.max_chattering
              << std::setw(20) << result_dob.max_chattering << "\n";
    double chatter_reduction = 100.0 * (1.0 - result_dob.max_chattering / result_high_gain.max_chattering);
    std::cout << std::setw(25) << "Chattering Reduction"
              << std::setw(20) << "baseline"
              << std::setw(19) << std::setprecision(1) << chatter_reduction << "%" << "\n";
    std::cout << "─────────────────────────────────────────────────────────────────\n\n";

    SMCConfig cfg_dob;
    cfg_dob.type = SMCType::DISTURBANCE_OBSERVER;
    cfg_dob.K = 25.0;
    cfg_dob.dob_gain = 50.0;
    cfg_dob.use_boundary_layer = true;
    printSMCSummary(cfg_dob, SlidingSurfaceConfig::linear({5.0, 1.0}), result_dob);

    plotSMCComparison({result_high_gain, result_dob}, {"High-Gain SMC", "DOBSMC"}, 
                      "smc_disturbance_observer_comparison");
    plotChatteringAnalysis(result_high_gain, result_dob, "smc_dob_chattering");

    // ================================================================
    // Grand Comparison: All Advanced Algorithms
    // ================================================================
    std::cout << "\n═══════════════════════════════════════════════════════════════════\n";
    std::cout << "Grand Comparison: All Advanced SMC Algorithms\n";
    std::cout << "═══════════════════════════════════════════════════════════════════\n\n";

    // Regenerate with same conditions for fair comparison
    auto result_sta_new = sim_conv.simulate(x0, t_final, dt, reference, disturbance);
    
    auto pair_fxt_new = createFixedTimeSMC(5.0, 5.0, 5.0, 0.5, 1.5);
    auto ctrl_fxt_new = pair_fxt_new.first;
    SMCSimulator sim_fxt_new(ctrl_fxt_new, pair_fxt_new.second, 2);
    auto result_fxt_new = sim_fxt_new.simulate(x0, t_final, dt, reference, disturbance);

    auto result_et_new = sim_et.simulate(x0, t_final, dt, reference, disturbance);
    auto result_barrier_new = sim_barrier.simulate(x0, t_final, dt, reference, disturbance);
    auto result_dob_new = sim_dob.simulate(x0, t_final, dt, reference, disturbance);

    plotSMCComparison(
        {result_sta_new, result_fxt_new, result_et_new, result_barrier_new, result_dob_new},
        {"Conventional", "Fixed-Time", "Event-Triggered", "Barrier", "DOB"},
        "smc_all_advanced_comparison"
    );

    // ================================================================
    // Summary
    // ================================================================
    std::cout << "\n╔════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                       GENERATED PLOTS                              ║\n";
    std::cout << "╠════════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ 1. smc_fixedtime_comparison.svg/png                                ║\n";
    std::cout << "║ 2. smc_fixedtime_response.svg/png                                  ║\n";
    std::cout << "║ 3. smc_event_triggered_comparison.svg/png                          ║\n";
    std::cout << "║ 4. smc_barrier_function.svg/png                                    ║\n";
    std::cout << "║ 5. smc_disturbance_observer_comparison.svg/png                     ║\n";
    std::cout << "║ 6. smc_dob_chattering.svg/png                                      ║\n";
    std::cout << "║ 7. smc_all_advanced_comparison.svg/png                             ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════════╝\n";

    std::cout << "\n╔════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                  ALGORITHM SELECTION GUIDE                         ║\n";
    std::cout << "╠════════════════════════════════════════════════════════════════════╣\n";
    std::cout << "║ Fixed-Time SMC:      When convergence time must be guaranteed      ║\n";
    std::cout << "║                      regardless of initial conditions              ║\n";
    std::cout << "║                                                                    ║\n";
    std::cout << "║ Event-Triggered:     Resource-constrained systems, networked       ║\n";
    std::cout << "║                      control, battery-powered applications         ║\n";
    std::cout << "║                                                                    ║\n";
    std::cout << "║ Barrier Function:    Safety-critical systems with hard state       ║\n";
    std::cout << "║                      constraints (robot joint limits, etc.)        ║\n";
    std::cout << "║                                                                    ║\n";
    std::cout << "║ DOBSMC:              Systems with significant unknown disturbances ║\n";
    std::cout << "║                      where chattering must be minimized            ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════════╝\n";

    return 0;
}
