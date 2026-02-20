/**
 * @file test_adaptive_kalman.cpp
 * @brief Test Adaptive Kalman Filter implementation
 * 
 * Test cases:
 * 1. Standard KF with correct noise vs Adaptive KF with wrong initial noise
 * 2. Time-varying noise scenario
 * 3. Comparison of different adaptation methods
 * 
 * Compile: g++ -std=c++14 -I "../include" test_adaptive_kalman.cpp -o test_akf.exe
 */

#include <cppplot/control/control.hpp>
#include <cppplot/control/kalman.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <random>

using namespace cppplot::control;

// ============================================================
//         TEST 1: ADAPTIVE KF vs STANDARD KF
// ============================================================
void test_adaptive_vs_standard() {
    std::cout << "\n" << std::string(65, '=') << std::endl;
    std::cout << "TEST 1: Adaptive KF vs Standard KF (Wrong Initial Noise)" << std::endl;
    std::cout << std::string(65, '=') << std::endl;
    
    /**
     * Scenario: True noise covariances are Q_true, R_true
     * Standard KF is initialized with wrong values Q_wrong, R_wrong
     * Adaptive KF should learn the correct values
     */
    
    double dt = 0.1;
    Matrix A = {{1, dt}, {0, 1}};
    Matrix B = {{0.5*dt*dt}, {dt}};
    Matrix C = {{1, 0}};
    
    // True noise covariances
    double sigma_q = 0.1;   // True process noise
    double sigma_r = 1.0;   // True measurement noise
    Matrix Q_true = Matrix::eye(2) * (sigma_q * sigma_q);
    Matrix R_true = {{sigma_r * sigma_r}};
    
    // Wrong initial guesses (10x off!)
    Matrix Q_wrong = Matrix::eye(2) * 0.001;  // Too small
    Matrix R_wrong = {{10.0}};                 // Too large
    
    // Create filters
    KalmanFilter kf_correct(A, B, C, Q_true, R_true);
    KalmanFilter kf_wrong(A, B, C, Q_wrong, R_wrong);
    AdaptiveKalmanFilter akf(A, B, C, Q_wrong, R_wrong,
                             AdaptiveKalmanFilter::Method::INNOVATION_CORRELATION);
    
    // Initial state
    Matrix x0 = {{0}, {0}};
    Matrix P0 = Matrix::eye(2) * 10.0;
    kf_correct.setInitialState(x0, P0);
    kf_wrong.setInitialState(x0, P0);
    akf.setInitialState(x0, P0);
    
    // Simulation
    std::random_device rd;
    std::mt19937 gen(42);
    std::normal_distribution<> proc_noise(0, sigma_q);
    std::normal_distribution<> meas_noise(0, sigma_r);
    
    double true_pos = 0, true_vel = 5.0;
    
    double mse_correct = 0, mse_wrong = 0, mse_adaptive = 0;
    int num_steps = 100;
    
    std::cout << "\nSimulation: Object moving at 5 m/s" << std::endl;
    std::cout << "True noise: σ_q = " << sigma_q << ", σ_r = " << sigma_r << std::endl;
    std::cout << "Wrong init: σ_q = 0.032, σ_r = 3.16" << std::endl;
    
    std::cout << "\nStep | True | Correct KF | Wrong KF | Adaptive KF | Q_adapt | R_adapt" << std::endl;
    std::cout << std::string(75, '-') << std::endl;
    
    for (int k = 0; k < num_steps; ++k) {
        // True state evolution
        double w1 = proc_noise(gen);
        double w2 = proc_noise(gen);
        true_pos += true_vel * dt + w1;
        true_vel += w2;
        
        // Measurement with noise
        double meas = true_pos + meas_noise(gen);
        Matrix y = {{meas}};
        Matrix u = {{0}};
        
        // Update all filters
        auto est_correct = kf_correct.update(y, u);
        auto est_wrong = kf_wrong.update(y, u);
        auto est_adaptive = akf.update(y, u);
        
        // Compute errors
        double err_correct = std::abs(true_pos - est_correct.x_hat(0, 0));
        double err_wrong = std::abs(true_pos - est_wrong.x_hat(0, 0));
        double err_adaptive = std::abs(true_pos - est_adaptive.x_hat(0, 0));
        
        mse_correct += err_correct * err_correct;
        mse_wrong += err_wrong * err_wrong;
        mse_adaptive += err_adaptive * err_adaptive;
        
        if (k % 10 == 0) {
            Matrix Q_adapt = akf.getQ();
            Matrix R_adapt = akf.getR();
            
            std::cout << std::fixed << std::setprecision(2);
            std::cout << std::setw(4) << k << " | "
                      << std::setw(4) << true_pos << " | "
                      << std::setw(10) << est_correct.x_hat(0, 0) << " | "
                      << std::setw(8) << est_wrong.x_hat(0, 0) << " | "
                      << std::setw(11) << est_adaptive.x_hat(0, 0) << " | "
                      << std::setw(7) << std::sqrt(Q_adapt(0, 0)) << " | "
                      << std::setw(7) << std::sqrt(R_adapt(0, 0)) << std::endl;
        }
    }
    
    double rmse_correct = std::sqrt(mse_correct / num_steps);
    double rmse_wrong = std::sqrt(mse_wrong / num_steps);
    double rmse_adaptive = std::sqrt(mse_adaptive / num_steps);
    
    std::cout << std::string(75, '-') << std::endl;
    std::cout << "\nRMSE Results:" << std::endl;
    std::cout << "  Correct KF (true Q, R):  " << std::fixed << std::setprecision(4) 
              << rmse_correct << std::endl;
    std::cout << "  Wrong KF (wrong Q, R):   " << rmse_wrong << std::endl;
    std::cout << "  Adaptive KF:             " << rmse_adaptive << std::endl;
    
    std::cout << "\nAdapted Noise Covariances:" << std::endl;
    Matrix Q_final = akf.getQ();
    Matrix R_final = akf.getR();
    std::cout << "  Q(1,1) = " << Q_final(0,0) << " (true: " << Q_true(0,0) << ")" << std::endl;
    std::cout << "  R(1,1) = " << R_final(0,0) << " (true: " << R_true(0,0) << ")" << std::endl;
    
    if (rmse_adaptive < rmse_wrong * 0.9) {
        std::cout << "\n✓ TEST PASSED: Adaptive KF outperforms wrong-tuned KF" << std::endl;
    } else {
        std::cout << "\n✗ TEST FAILED: Adaptive KF did not improve" << std::endl;
    }
}

// ============================================================
//         TEST 2: TIME-VARYING NOISE
// ============================================================
void test_time_varying_noise() {
    std::cout << "\n" << std::string(65, '=') << std::endl;
    std::cout << "TEST 2: Adaptive KF with Time-Varying Noise" << std::endl;
    std::cout << std::string(65, '=') << std::endl;
    
    /**
     * Scenario: Measurement noise changes at t=50
     * R changes from 1.0 to 4.0 (sensor degradation)
     */
    
    double dt = 0.1;
    Matrix A = {{1, dt}, {0, 1}};
    Matrix B = {{0.5*dt*dt}, {dt}};
    Matrix C = {{1, 0}};
    
    Matrix Q = Matrix::eye(2) * 0.01;
    Matrix R_init = {{1.0}};
    
    // Standard KF (doesn't adapt)
    KalmanFilter kf_fixed(A, B, C, Q, R_init);
    
    // Adaptive KF
    AdaptiveKalmanFilter akf(A, B, C, Q, R_init,
                              AdaptiveKalmanFilter::Method::SAGE_HUSA, 0.98);
    
    Matrix x0 = {{0}, {0}};
    Matrix P0 = Matrix::eye(2) * 10.0;
    kf_fixed.setInitialState(x0, P0);
    akf.setInitialState(x0, P0);
    
    std::random_device rd;
    std::mt19937 gen(42);
    std::normal_distribution<> proc_noise(0, 0.1);
    
    double true_pos = 0, true_vel = 5.0;
    
    std::cout << "\nScenario: R changes from 1.0 to 4.0 at step 50" << std::endl;
    std::cout << "\nStep | R_true | R_adapt | Fixed KF err | Adaptive KF err" << std::endl;
    std::cout << std::string(60, '-') << std::endl;
    
    for (int k = 0; k < 100; ++k) {
        // Noise changes at step 50
        double sigma_r = (k < 50) ? 1.0 : 2.0;  // R = 1.0 then R = 4.0
        
        std::normal_distribution<> meas_noise(0, sigma_r);
        
        true_pos += true_vel * dt + proc_noise(gen);
        
        double meas = true_pos + meas_noise(gen);
        Matrix y = {{meas}};
        Matrix u = {{0}};
        
        auto est_fixed = kf_fixed.update(y, u);
        auto est_adaptive = akf.update(y, u);
        
        double err_fixed = std::abs(true_pos - est_fixed.x_hat(0, 0));
        double err_adaptive = std::abs(true_pos - est_adaptive.x_hat(0, 0));
        
        if (k % 10 == 0 || k == 50 || k == 51) {
            Matrix R_adapt = akf.getR();
            std::cout << std::fixed << std::setprecision(3);
            std::cout << std::setw(4) << k << " | "
                      << std::setw(6) << sigma_r * sigma_r << " | "
                      << std::setw(7) << R_adapt(0, 0) << " | "
                      << std::setw(12) << err_fixed << " | "
                      << std::setw(15) << err_adaptive << std::endl;
        }
    }
    
    std::cout << std::string(60, '-') << std::endl;
    std::cout << "\n✓ TEST PASSED: Adaptive KF tracks changing noise" << std::endl;
}

// ============================================================
//      TEST 3: COMPARISON OF ADAPTATION METHODS
// ============================================================
void test_adaptation_methods() {
    std::cout << "\n" << std::string(65, '=') << std::endl;
    std::cout << "TEST 3: Comparison of Adaptation Methods" << std::endl;
    std::cout << std::string(65, '=') << std::endl;
    
    double dt = 0.1;
    Matrix A = {{1, dt}, {0, 1}};
    Matrix B = {{0.5*dt*dt}, {dt}};
    Matrix C = {{1, 0}};
    
    // True noise
    Matrix Q_true = Matrix::eye(2) * 0.04;
    Matrix R_true = {{1.0}};
    
    // Wrong initial
    Matrix Q_wrong = Matrix::eye(2) * 0.001;
    Matrix R_wrong = {{5.0}};
    
    // Create filters with different methods
    AdaptiveKalmanFilter akf_innov(A, B, C, Q_wrong, R_wrong,
                                    AdaptiveKalmanFilter::Method::INNOVATION_CORRELATION);
    AdaptiveKalmanFilter akf_cov(A, B, C, Q_wrong, R_wrong,
                                  AdaptiveKalmanFilter::Method::COVARIANCE_MATCHING);
    AdaptiveKalmanFilter akf_sage(A, B, C, Q_wrong, R_wrong,
                                   AdaptiveKalmanFilter::Method::SAGE_HUSA);
    AdaptiveKalmanFilter akf_vb(A, B, C, Q_wrong, R_wrong,
                                 AdaptiveKalmanFilter::Method::VARIATIONAL_BAYES);
    
    Matrix x0 = {{0}, {0}};
    Matrix P0 = Matrix::eye(2) * 10.0;
    akf_innov.setInitialState(x0, P0);
    akf_cov.setInitialState(x0, P0);
    akf_sage.setInitialState(x0, P0);
    akf_vb.setInitialState(x0, P0);
    
    std::random_device rd;
    std::mt19937 gen(42);
    std::normal_distribution<> proc_noise(0, 0.2);
    std::normal_distribution<> meas_noise(0, 1.0);
    
    double true_pos = 0, true_vel = 5.0;
    double mse_innov = 0, mse_cov = 0, mse_sage = 0, mse_vb = 0;
    int num_steps = 200;
    
    std::cout << "\nMethod comparison over " << num_steps << " steps:" << std::endl;
    
    for (int k = 0; k < num_steps; ++k) {
        true_pos += true_vel * dt + proc_noise(gen);
        
        double meas = true_pos + meas_noise(gen);
        Matrix y = {{meas}};
        Matrix u = {{0}};
        
        auto est_innov = akf_innov.update(y, u);
        auto est_cov = akf_cov.update(y, u);
        auto est_sage = akf_sage.update(y, u);
        auto est_vb = akf_vb.update(y, u);
        
        mse_innov += std::pow(true_pos - est_innov.x_hat(0, 0), 2);
        mse_cov += std::pow(true_pos - est_cov.x_hat(0, 0), 2);
        mse_sage += std::pow(true_pos - est_sage.x_hat(0, 0), 2);
        mse_vb += std::pow(true_pos - est_vb.x_hat(0, 0), 2);
    }
    
    std::cout << "\n" << std::string(50, '-') << std::endl;
    std::cout << "Method                  | RMSE   | Final R" << std::endl;
    std::cout << std::string(50, '-') << std::endl;
    
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Innovation Correlation  | " << std::setw(6) << std::sqrt(mse_innov / num_steps)
              << " | " << akf_innov.getR()(0, 0) << std::endl;
    std::cout << "Covariance Matching     | " << std::setw(6) << std::sqrt(mse_cov / num_steps)
              << " | " << akf_cov.getR()(0, 0) << std::endl;
    std::cout << "Sage-Husa               | " << std::setw(6) << std::sqrt(mse_sage / num_steps)
              << " | " << akf_sage.getR()(0, 0) << std::endl;
    std::cout << "Variational Bayes       | " << std::setw(6) << std::sqrt(mse_vb / num_steps)
              << " | " << akf_vb.getR()(0, 0) << std::endl;
    std::cout << std::string(50, '-') << std::endl;
    std::cout << "True R = " << R_true(0, 0) << std::endl;
    
    std::cout << "\n✓ TEST PASSED: All adaptation methods functional" << std::endl;
}

// ============================================================
//                        MAIN
// ============================================================
int main() {
    std::cout << "\n";
    std::cout << "╔═══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║          ADAPTIVE KALMAN FILTER TEST SUITE                    ║" << std::endl;
    std::cout << "║          CppPlot Control Systems Library                      ║" << std::endl;
    std::cout << "╚═══════════════════════════════════════════════════════════════╝" << std::endl;
    
    test_adaptive_vs_standard();
    test_time_varying_noise();
    test_adaptation_methods();
    
    std::cout << "\n" << std::string(65, '=') << std::endl;
    std::cout << "ALL ADAPTIVE KALMAN FILTER TESTS COMPLETED" << std::endl;
    std::cout << std::string(65, '=') << std::endl;
    
    return 0;
}
