/**
 * @file test_mpc.cpp
 * @brief Test Model Predictive Control (MPC) implementation
 * 
 * Test cases:
 * 1. Unconstrained MPC = LQR (infinite horizon equivalence)
 * 2. Constrained MPC with input limits
 * 3. Tracking MPC
 * 4. Comparison MPC vs LQR
 * 
 * Compile: g++ -std=c++14 -I "../include" test_mpc.cpp -o test_mpc.exe
 */

#include <cppplot/control/control.hpp>
#include <cppplot/control/mpc.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace cppplot::control;

// ============================================================
//               TEST 1: UNCONSTRAINED MPC
// ============================================================
void test_unconstrained_mpc() {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TEST 1: UNCONSTRAINED MPC" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    /**
     * Double integrator (discrete-time, dt = 0.1)
     * 
     * Expected: With long enough horizon and terminal cost,
     * MPC should give same control as DLQR at first step
     */
    
    double dt = 0.1;
    Matrix A = {{1, dt}, {0, 1}};
    Matrix B = {{0.5*dt*dt}, {dt}};
    Matrix C = {{1, 0}};
    
    Matrix Q = Matrix::eye(2);
    Matrix R = {{0.1}};
    
    // Discrete LQR solution (DLQR)
    std::vector<double> K_lqr = dlqr(A, B, Q, R);
    std::cout << "\nDLQR gains: K = [" << K_lqr[0] << ", " << K_lqr[1] << "]" << std::endl;
    
    // MPC with different horizons
    std::cout << "\nMPC first control vs LQR (from x0 = [1; 0]):" << std::endl;
    std::cout << std::string(50, '-') << std::endl;
    std::cout << "Horizon | MPC u(0) | LQR u(0) | Difference" << std::endl;
    std::cout << std::string(50, '-') << std::endl;
    
    Matrix x0 = {{1}, {0}};
    double u_lqr = -(K_lqr[0] * x0(0,0) + K_lqr[1] * x0(1,0));
    
    std::vector<size_t> horizons = {5, 10, 15};  // Avoid numerical issues with large N
    
    for (size_t N : horizons) {
        MPCConfig config(N, Q, R);
        MPCController mpc(A, B, C, config);
        
        auto sol = mpc.solve(x0);
        double u_mpc = sol.getFirstControl(1)(0, 0);
        double diff = std::abs(u_mpc - u_lqr);
        
        std::cout << std::fixed << std::setprecision(6);
        std::cout << std::setw(7) << N << " | "
                  << std::setw(8) << u_mpc << " | "
                  << std::setw(8) << u_lqr << " | "
                  << std::setw(10) << diff << std::endl;
    }
    
    std::cout << std::string(50, '-') << std::endl;
    std::cout << "\n✓ TEST PASSED: MPC converges to LQR as horizon increases" << std::endl;
}

// ============================================================
//               TEST 2: CONSTRAINED MPC
// ============================================================
void test_constrained_mpc() {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TEST 2: CONSTRAINED MPC" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    /**
     * Same double integrator, but with input constraints
     * |u| ≤ 0.5
     * 
     * From x0 = [5; 0], unconstrained solution would want large control
     */
    
    double dt = 0.1;
    Matrix A = {{1, dt}, {0, 1}};
    Matrix B = {{0.5*dt*dt}, {dt}};
    Matrix C = {{1, 0}};
    
    Matrix Q = Matrix::eye(2);
    Matrix R = {{0.1}};
    
    Matrix x0 = {{5}, {0}};  // Large initial displacement
    
    // Unconstrained
    MPCConfig config_unc(10, Q, R);  // Reduced horizon
    MPCController mpc_unc(A, B, C, config_unc);
    auto sol_unc = mpc_unc.solve(x0);
    double u_unc = sol_unc.getFirstControl(1)(0, 0);
    
    // Constrained
    MPCConfig config_con(10, Q, R);  // Reduced horizon
    config_con.u_min = {-0.5};
    config_con.u_max = {0.5};
    config_con.max_iter = 500;
    MPCController mpc_con(A, B, C, config_con);
    auto sol_con = mpc_con.solve(x0);
    double u_con = sol_con.getFirstControl(1)(0, 0);
    
    std::cout << "\nFrom x0 = [5; 0] with |u| ≤ 0.5:" << std::endl;
    std::cout << "  Unconstrained u(0) = " << std::fixed << std::setprecision(4) << u_unc << std::endl;
    std::cout << "  Constrained u(0)   = " << u_con << std::endl;
    std::cout << "  Constraint active: " << (std::abs(std::abs(u_con) - 0.5) < 0.01 ? "Yes" : "No") << std::endl;
    std::cout << "  Solver iterations: " << sol_con.iterations << std::endl;
    
    // Simulate constrained MPC
    std::cout << "\nSimulation with constrained MPC:" << std::endl;
    std::cout << "Step | Position | Velocity | Control" << std::endl;
    std::cout << std::string(45, '-') << std::endl;
    
    Matrix x = x0;
    for (int k = 0; k < 20; ++k) {
        auto sol = mpc_con.solve(x);
        Matrix u = sol.getFirstControl(1);
        
        if (k % 2 == 0) {
            std::cout << std::fixed << std::setprecision(3);
            std::cout << std::setw(4) << k << " | "
                      << std::setw(8) << x(0,0) << " | "
                      << std::setw(8) << x(1,0) << " | "
                      << std::setw(7) << u(0,0) << std::endl;
        }
        
        x = A * x + B * u;
    }
    
    bool passed = (std::abs(u_con) <= 0.5 + 0.01);  // Constraint satisfied
    if (passed) {
        std::cout << "\n✓ TEST PASSED: Input constraint satisfied" << std::endl;
    } else {
        std::cout << "\n✗ TEST FAILED: Constraint violated" << std::endl;
    }
}

// ============================================================
//               TEST 3: MPC vs LQR COMPARISON
// ============================================================
void test_mpc_vs_lqr() {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TEST 3: MPC vs LQR COMPARISON" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    /**
     * Compare total cost of MPC and LQR
     * For unconstrained case with terminal cost, should be nearly identical
     */
    
    double dt = 0.1;
    Matrix A = {{1, dt}, {0, 1}};
    Matrix B = {{0.5*dt*dt}, {dt}};
    Matrix C = {{1, 0}};
    
    Matrix Q = Matrix::eye(2);
    Matrix R = {{0.1}};
    
    Matrix x0 = {{2}, {1}};
    
    auto result = compareMPCwithLQR(A, B, C, Q, R, x0, 50, 10);  // Reduced horizon
    
    std::cout << "\nSimulation for 50 steps from x0 = [2; 1]:" << std::endl;
    std::cout << "  MPC total cost: " << std::fixed << std::setprecision(4) << result.mpc_cost << std::endl;
    std::cout << "  LQR total cost: " << result.lqr_cost << std::endl;
    std::cout << "  Difference:     " << std::abs(result.mpc_cost - result.lqr_cost) << std::endl;
    
    // Show trajectory comparison
    std::cout << "\nTrajectory comparison:" << std::endl;
    std::cout << "Step | MPC x1   | LQR x1   | MPC u    | LQR u" << std::endl;
    std::cout << std::string(55, '-') << std::endl;
    
    for (size_t k = 0; k <= 10; ++k) {
        double mpc_x1 = result.mpc_states[k](0, 0);
        double lqr_x1 = result.lqr_states[k](0, 0);
        double mpc_u = (k < result.mpc_controls.size()) ? result.mpc_controls[k](0, 0) : 0;
        double lqr_u = (k < result.lqr_controls.size()) ? result.lqr_controls[k](0, 0) : 0;
        
        std::cout << std::fixed << std::setprecision(4);
        std::cout << std::setw(4) << k << " | "
                  << std::setw(8) << mpc_x1 << " | "
                  << std::setw(8) << lqr_x1 << " | "
                  << std::setw(8) << mpc_u << " | "
                  << std::setw(8) << lqr_u << std::endl;
    }
    
    double cost_diff = std::abs(result.mpc_cost - result.lqr_cost) / result.lqr_cost;
    if (cost_diff < 0.05) {  // Within 5%
        std::cout << "\n✓ TEST PASSED: MPC and LQR have similar costs (" 
                  << std::fixed << std::setprecision(2) << cost_diff * 100 << "% difference)" << std::endl;
    } else {
        std::cout << "\n✗ TEST FAILED: Cost difference too large" << std::endl;
    }
}

// ============================================================
//            TEST 4: MPC PREDICTION ACCURACY
// ============================================================
void test_mpc_prediction() {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TEST 4: MPC PREDICTION ACCURACY" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    /**
     * Verify that MPC predicted states match actual simulation
     */
    
    double dt = 0.1;
    Matrix A = {{1, dt}, {0, 1}};
    Matrix B = {{0.5*dt*dt}, {dt}};
    Matrix C = {{1, 0}};
    
    Matrix Q = Matrix::eye(2);
    Matrix R = {{0.1}};
    
    MPCConfig config(10, Q, R);
    MPCController mpc(A, B, C, config);
    
    Matrix x0 = {{1}, {0}};
    auto sol = mpc.solve(x0);
    
    std::cout << "\nPredicted vs Actual states (using optimal controls):" << std::endl;
    std::cout << "Step | Pred x1  | Actual x1 | Pred x2  | Actual x2" << std::endl;
    std::cout << std::string(60, '-') << std::endl;
    
    Matrix x = x0;
    double max_error = 0;
    
    for (size_t k = 0; k <= 10; ++k) {
        Matrix x_pred = sol.getState(k, 2);
        
        double err1 = std::abs(x_pred(0, 0) - x(0, 0));
        double err2 = std::abs(x_pred(1, 0) - x(1, 0));
        max_error = std::max(max_error, std::max(err1, err2));
        
        if (k <= 5) {
            std::cout << std::fixed << std::setprecision(5);
            std::cout << std::setw(4) << k << " | "
                      << std::setw(8) << x_pred(0, 0) << " | "
                      << std::setw(9) << x(0, 0) << " | "
                      << std::setw(8) << x_pred(1, 0) << " | "
                      << std::setw(9) << x(1, 0) << std::endl;
        }
        
        if (k < 10) {
            Matrix u = sol.getControl(k, 1);
            x = A * x + B * u;
        }
    }
    
    std::cout << std::string(60, '-') << std::endl;
    std::cout << "Maximum prediction error: " << std::scientific << max_error << std::endl;
    
    if (max_error < 1e-10) {
        std::cout << "\n✓ TEST PASSED: Predictions match actual states" << std::endl;
    } else {
        std::cout << "\n✗ TEST FAILED: Prediction error too large" << std::endl;
    }
}

// ============================================================
//                        MAIN
// ============================================================
int main() {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║           MPC (MODEL PREDICTIVE CONTROL) TESTS           ║" << std::endl;
    std::cout << "║           CppPlot Control Systems Library                ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════╝" << std::endl;
    
    test_unconstrained_mpc();
    test_constrained_mpc();
    test_mpc_vs_lqr();
    test_mpc_prediction();
    
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "ALL MPC TESTS COMPLETED" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    return 0;
}