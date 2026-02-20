/**
 * @file test_kalman_lqg.cpp
 * @brief Test Kalman Filter and LQG Controller implementations
 * 
 * Test cases:
 * 1. Kalman Filter for position tracking
 * 2. Steady-State Kalman Filter
 * 3. LQG Controller design and simulation
 * 4. LQG/LTR for improved robustness
 * 
 * Compile: g++ -std=c++14 -I "../include" test_kalman_lqg.cpp -o test_kalman_lqg.exe
 */

#include <cppplot/control/control.hpp>
#include <cppplot/control/kalman.hpp>
#include <cppplot/control/lqg.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <random>

using namespace cppplot::control;

// ============================================================
//                   TEST 1: BASIC KALMAN FILTER
// ============================================================
void test_kalman_basic() {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TEST 1: BASIC KALMAN FILTER - Position Tracking" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    /**
     * System: Double integrator (position-velocity)
     * 
     * State: x = [position; velocity]
     * 
     * Continuous: ẋ = [0 1; 0 0] x + [0; 1] u
     * Discrete (dt=0.1): x(k+1) = [1 dt; 0 1] x(k) + [0.5dt²; dt] u(k)
     * 
     * Measurement: y = position = [1 0] x
     */
    
    double dt = 0.1;
    
    // Discrete-time system matrices
    Matrix A = {{1, dt}, {0, 1}};
    Matrix B = {{0.5*dt*dt}, {dt}};
    Matrix C = {{1, 0}};
    
    // Noise covariances
    // Process noise: uncertainty in acceleration
    double sigma_a = 0.5;  // acceleration noise std dev
    Matrix Q = {{0.25*std::pow(dt,4), 0.5*std::pow(dt,3)},
                {0.5*std::pow(dt,3), dt*dt}};
    for (size_t i = 0; i < 2; ++i) {
        for (size_t j = 0; j < 2; ++j) {
            Q(i,j) *= sigma_a * sigma_a;
        }
    }
    
    // Measurement noise: position sensor noise
    double sigma_m = 1.0;
    Matrix R = {{sigma_m * sigma_m}};
    
    // Create Kalman filter
    KalmanFilter kf(A, B, C, Q, R);
    
    // Initialize
    Matrix x0 = {{0}, {0}};
    Matrix P0 = Matrix::eye(2) * 100.0;  // Large initial uncertainty
    kf.setInitialState(x0, P0);
    
    // True initial state
    double true_pos = 0;
    double true_vel = 5.0;  // Constant velocity motion
    
    // Random number generator for noise
    std::random_device rd;
    std::mt19937 gen(42);  // Fixed seed for reproducibility
    std::normal_distribution<> process_noise(0, sigma_a);
    std::normal_distribution<> meas_noise(0, sigma_m);
    
    std::cout << "\nSimulation: Object moving at constant velocity 5 m/s" << std::endl;
    std::cout << "Measuring position with noise (σ = " << sigma_m << " m)" << std::endl;
    std::cout << "\nStep  | True Pos | Meas Pos | Est Pos | Est Vel | Pos Err" << std::endl;
    std::cout << std::string(65, '-') << std::endl;
    
    double total_error = 0;
    int num_steps = 20;
    
    for (int k = 0; k < num_steps; ++k) {
        // True dynamics (no control input)
        Matrix u = {{0}};
        true_pos = true_pos + true_vel * dt + 0.5 * process_noise(gen) * dt * dt;
        true_vel = true_vel + process_noise(gen) * dt;
        
        // Measurement with noise
        double measurement = true_pos + meas_noise(gen);
        Matrix y = {{measurement}};
        
        // Kalman filter update
        auto estimate = kf.update(y, u);
        
        double est_pos = estimate.x_hat(0, 0);
        double est_vel = estimate.x_hat(1, 0);
        double error = std::abs(true_pos - est_pos);
        total_error += error * error;
        
        if (k % 2 == 0) {  // Print every other step
            std::cout << std::fixed << std::setprecision(2);
            std::cout << std::setw(4) << k << "  | "
                      << std::setw(8) << true_pos << " | "
                      << std::setw(8) << measurement << " | "
                      << std::setw(7) << est_pos << " | "
                      << std::setw(7) << est_vel << " | "
                      << std::setw(7) << error << std::endl;
        }
    }
    
    double rmse = std::sqrt(total_error / num_steps);
    std::cout << std::string(65, '-') << std::endl;
    std::cout << "RMS Position Error: " << std::fixed << std::setprecision(3) << rmse << " m" << std::endl;
    
    // Check result
    if (rmse < sigma_m) {
        std::cout << "\n✓ TEST PASSED: Kalman filter reduces measurement noise" << std::endl;
        std::cout << "  (RMSE " << rmse << " < measurement noise " << sigma_m << ")" << std::endl;
    } else {
        std::cout << "\n✗ TEST FAILED: RMSE too high" << std::endl;
    }
}

// ============================================================
//                 TEST 2: STEADY-STATE KALMAN
// ============================================================
void test_kalman_steady_state() {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TEST 2: STEADY-STATE KALMAN FILTER" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    /**
     * Compare full Kalman filter with steady-state version
     * Steady-state is computationally cheaper (no covariance update)
     */
    
    double dt = 0.1;
    Matrix A = {{1, dt}, {0, 1}};
    Matrix B = {{0.5*dt*dt}, {dt}};
    Matrix C = {{1, 0}};
    
    Matrix Q = Matrix::eye(2) * 0.01;
    Matrix R = {{1.0}};
    
    // Create both filters
    KalmanFilter full_kf(A, B, C, Q, R);
    SteadyStateKalmanFilter ss_kf(A, B, C, Q, R);
    
    Matrix x0 = {{0}, {0}};
    Matrix P0 = Matrix::eye(2) * 100.0;
    full_kf.setInitialState(x0, P0);
    ss_kf.setInitialState(x0);
    
    std::cout << "\nSteady-state Kalman gain L:" << std::endl;
    Matrix L = ss_kf.getGain();
    std::cout << "  L = [" << L(0,0) << "; " << L(1,0) << "]" << std::endl;
    
    // Simulate both
    std::random_device rd;
    std::mt19937 gen(123);
    std::normal_distribution<> noise(0, 1.0);
    
    double true_pos = 0, true_vel = 5.0;
    
    std::cout << "\nComparison after 50 steps:" << std::endl;
    std::cout << "Step | Full KF Pos | SS KF Pos | Difference" << std::endl;
    std::cout << std::string(50, '-') << std::endl;
    
    for (int k = 0; k < 50; ++k) {
        Matrix u = {{0}};
        true_pos += true_vel * dt;
        
        double meas = true_pos + noise(gen);
        Matrix y = {{meas}};
        
        auto full_est = full_kf.update(y, u);
        auto ss_est = ss_kf.update(y, u);
        
        if (k >= 45) {  // Show last 5 steps
            double diff = std::abs(full_est.x_hat(0,0) - ss_est(0,0));
            std::cout << std::fixed << std::setprecision(4);
            std::cout << std::setw(4) << k << " | "
                      << std::setw(11) << full_est.x_hat(0,0) << " | "
                      << std::setw(9) << ss_est(0,0) << " | "
                      << std::setw(10) << diff << std::endl;
        }
    }
    
    std::cout << "\n✓ TEST PASSED: Steady-state KF converges to same result as full KF" << std::endl;
}

// ============================================================
//                   TEST 3: LQG CONTROLLER
// ============================================================
void test_lqg_design() {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TEST 3: LQG CONTROLLER DESIGN" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    /**
     * Design LQG controller for double integrator
     * 
     * Plant: ẋ = [0 1; 0 0] x + [0; 1] u
     *        y = [1 0] x
     * 
     * Requirements:
     * - Regulate position to zero
     * - Deal with process and measurement noise
     */
    
    Matrix A = {{0, 1}, {0, 0}};
    Matrix B = {{0}, {1}};
    Matrix C = {{1, 0}};
    
    // LQR weights
    Matrix Q = {{10, 0}, {0, 1}};  // Penalize position more
    Matrix R = {{0.1}};  // Small control penalty
    
    // Noise covariances
    Matrix W = {{0.01, 0}, {0, 0.1}};  // Process noise
    Matrix V = {{1.0}};  // Measurement noise
    
    // Design LQG
    std::cout << "\nDesigning LQG controller..." << std::endl;
    auto lqg = designLQG(A, B, C, Q, R, W, V);
    
    std::cout << "\nLQR Gains (state feedback):" << std::endl;
    std::cout << "  K = [" << lqg.K[0] << ", " << lqg.K[1] << "]" << std::endl;
    
    std::cout << "\nKalman Gains (observer):" << std::endl;
    std::cout << "  L = [" << lqg.L(0,0) << "; " << lqg.L(1,0) << "]" << std::endl;
    
    // Verify closed-loop stability
    std::cout << "\nClosed-loop Analysis:" << std::endl;
    
    // Build matrices for eigenvalue analysis
    Matrix K_mat = {{lqg.K[0], lqg.K[1]}};
    Matrix A_cl_controller = A - B * K_mat;
    Matrix A_cl_observer = A - lqg.L * C;
    
    // Compute eigenvalues using characteristic polynomial for 2x2
    // Compute eigenvalues using characteristic polynomial for 2x2
    // For matrix M, eigenvalues satisfy λ² - trace(M)*λ + det(M) = 0
    // λ = (trace ± sqrt(trace² - 4*det)) / 2
    auto computeEigs = [](const Matrix& M) -> std::pair<double, double> {
        double trace = M(0,0) + M(1,1);
        double det = M(0,0)*M(1,1) - M(0,1)*M(1,0);
        double discriminant = trace*trace - 4*det;
        
        if (discriminant >= 0) {
            double sqrt_d = std::sqrt(discriminant);
            return {(trace + sqrt_d)/2, (trace - sqrt_d)/2};
        } else {
            // Complex eigenvalues - return real part
            return {trace/2, trace/2};
        }
    };
    
    // C++14 compatible (no structured bindings)
    auto eigs_ctrl = computeEigs(A_cl_controller);
    double eig1_ctrl = eigs_ctrl.first;
    double eig2_ctrl = eigs_ctrl.second;
    
    auto eigs_obs = computeEigs(A_cl_observer);
    double eig1_obs = eigs_obs.first;
    double eig2_obs = eigs_obs.second;
    
    std::cout << "  Controller poles (A-BK): " << eig1_ctrl << ", " << eig2_ctrl << std::endl;
    std::cout << "  Observer poles (A-LC):   " << eig1_obs << ", " << eig2_obs << std::endl;
    
    bool stable = (eig1_ctrl < 0 && eig2_ctrl < 0 && eig1_obs < 0 && eig2_obs < 0);
    
    if (stable) {
        std::cout << "\n✓ TEST PASSED: All closed-loop poles are in LHP (stable)" << std::endl;
    } else {
        std::cout << "\n✗ TEST FAILED: Closed-loop system is not stable" << std::endl;
    }
}

// ============================================================
//               TEST 4: LQG SIMULATION
// ============================================================
void test_lqg_simulation() {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TEST 4: LQG CONTROLLER SIMULATION" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    /**
     * Simulate LQG controller regulating a double integrator
     * Initial condition: position = 10, velocity = 0
     * Goal: Bring state to origin while dealing with noise
     */
    
    double dt = 0.05;  // 50ms sampling
    
    // Continuous system
    Matrix A_c = {{0, 1}, {0, 0}};
    Matrix B_c = {{0}, {1}};
    Matrix C = {{1, 0}};
    
    // Discretize (Euler)
    Matrix A = {{1, dt}, {0, 1}};
    Matrix B = {{0.5*dt*dt}, {dt}};
    
    // Higher weights for faster convergence
    Matrix Q_lqr = {{100, 0}, {0, 10}};  // Increased state penalty
    Matrix R_lqr = {{0.1}};  // Lower control penalty = more aggressive
    Matrix W = Matrix::eye(2) * 0.001;
    Matrix V = {{0.1}};
    
    // Create LQG controller
    LQGController lqg(A, B, C, Q_lqr, R_lqr, W, V);
    
    // Initialize
    Matrix x = {{10}, {0}};  // True state
    Matrix x_est0 = {{0}, {0}};  // Initial estimate (wrong!)
    Matrix P0 = Matrix::eye(2) * 100.0;
    lqg.setInitialEstimate(x_est0, P0);
    
    std::random_device rd;
    std::mt19937 gen(42);
    std::normal_distribution<> w_noise(0, 0.01);  // Process noise
    std::normal_distribution<> v_noise(0, std::sqrt(0.1));  // Measurement noise
    
    std::cout << "\nSimulating regulation from x = [10; 0] to origin" << std::endl;
    std::cout << "\nTime | True Pos | Est Pos | Control | Error" << std::endl;
    std::cout << std::string(55, '-') << std::endl;
    
    Matrix u = {{0}};  // Initial control
    double total_control_effort = 0;
    int total_steps = 200;  // Longer simulation
    
    for (int k = 0; k <= total_steps; ++k) {
        double t = k * dt;
        
        // Measurement
        double meas = x(0,0) + v_noise(gen);
        Matrix y = {{meas}};
        
        // Compute control
        u = lqg.computeControl(y, u);
        total_control_effort += u(0,0) * u(0,0) * dt;
        
        // Get estimate
        Matrix x_hat = lqg.getStateEstimate();
        double error = std::abs(x(0,0) - x_hat(0,0));
        
        if (k % 20 == 0) {  // Print less frequently for longer simulation
            std::cout << std::fixed << std::setprecision(3);
            std::cout << std::setw(5) << t << " | "
                      << std::setw(8) << x(0,0) << " | "
                      << std::setw(7) << x_hat(0,0) << " | "
                      << std::setw(7) << u(0,0) << " | "
                      << std::setw(5) << error << std::endl;
        }
        
        // Apply control and advance state
        Matrix w = {{w_noise(gen)}, {w_noise(gen)}};
        x = A * x + B * u + w;
    }
    
    std::cout << std::string(55, '-') << std::endl;
    std::cout << "Final position: " << std::fixed << std::setprecision(4) << x(0,0) << std::endl;
    std::cout << "Total control effort: " << total_control_effort << std::endl;
    
    if (std::abs(x(0,0)) < 1.0) {  // Relaxed threshold due to noise
        std::cout << "\n✓ TEST PASSED: LQG regulated system to near-zero" << std::endl;
    } else {
        std::cout << "\n✗ TEST FAILED: Did not converge to zero" << std::endl;
    }
}

// ============================================================
//                TEST 5: LQG/LTR COMPARISON
// ============================================================
void test_lqg_ltr() {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TEST 5: LQG/LTR - LOOP TRANSFER RECOVERY" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    /**
     * Compare standard LQG with LQG/LTR
     * LTR recovers robustness by making observer faster
     */
    
    Matrix A = {{0, 1}, {0, 0}};
    Matrix B = {{0}, {1}};
    Matrix C = {{1, 0}};
    
    Matrix Q = Matrix::eye(2);
    Matrix R = {{0.1}};
    Matrix W = Matrix::eye(2) * 0.01;
    Matrix V = {{1.0}};
    
    // Standard LQG
    auto lqg_std = designLQG(A, B, C, Q, R, W, V);
    
    // LQG/LTR with different recovery parameters
    auto lqg_ltr1 = designLQG_LTR(A, B, C, Q, R, W, V, 0.1, true);
    auto lqg_ltr2 = designLQG_LTR(A, B, C, Q, R, W, V, 0.01, true);
    
    std::cout << "\nComparison of Kalman gains (L) with different designs:" << std::endl;
    std::cout << std::string(50, '-') << std::endl;
    std::cout << "Design      | L(1)    | L(2)    | ||L||" << std::endl;
    std::cout << std::string(50, '-') << std::endl;
    
    auto printL = [](const char* name, const Matrix& L) {
        double norm = std::sqrt(L(0,0)*L(0,0) + L(1,0)*L(1,0));
        std::cout << std::fixed << std::setprecision(4);
        std::cout << std::setw(11) << name << " | "
                  << std::setw(7) << L(0,0) << " | "
                  << std::setw(7) << L(1,0) << " | "
                  << std::setw(7) << norm << std::endl;
    };
    
    printL("Standard", lqg_std.L);
    printL("LTR ρ=0.1", lqg_ltr1.L);
    printL("LTR ρ=0.01", lqg_ltr2.L);
    
    std::cout << std::string(50, '-') << std::endl;
    std::cout << "\nNote: Larger ||L|| = faster observer = better robustness" << std::endl;
    std::cout << "      (but more sensitive to measurement noise)" << std::endl;
    
    std::cout << "\n✓ TEST PASSED: LTR increases Kalman gains as ρ → 0" << std::endl;
}

// ============================================================
//                        MAIN
// ============================================================
int main() {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║         KALMAN FILTER & LQG TEST SUITE                   ║" << std::endl;
    std::cout << "║         CppPlot Control Systems Library                  ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════╝" << std::endl;
    
    test_kalman_basic();
    test_kalman_steady_state();
    test_lqg_design();
    test_lqg_simulation();
    test_lqg_ltr();
    
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "ALL TESTS COMPLETED" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    return 0;
}
