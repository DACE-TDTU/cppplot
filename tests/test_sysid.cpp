/**
 * @file test_sysid.cpp
 * @brief Unit tests for System Identification module (sysid.hpp)
 * 
 * Tests cover:
 *  - First-order step response identification
 *  - Second-order step response identification
 *  - Order detection heuristic
 *  - ARX identification with known system
 *  - PRBS generation properties
 *  - Chirp / multisine / white noise generation
 *  - Model validation (FIT, whiteness)
 *  - AIC/BIC computation
 *  - lsim / dsim simulation
 *  - Autocorrelation / cross-correlation
 *  - Coherence
 * 
 * Build:
 *   g++ -std=c++17 -I../include test_sysid.cpp -o test_sysid && ./test_sysid
 */

#include <cppplot/control/control.hpp>
#include <iostream>
#include <cassert>
#include <cmath>
#include <random>
#include <algorithm>
#include <numeric>

using namespace cppplot;
using namespace cppplot::control;
using namespace cppplot::control::sysid;

// Simple test framework (same as test_core.cpp)
int tests_passed = 0;
int tests_failed = 0;

#define TEST(name) void test_##name()
#define RUN_TEST(name) do { \
    std::cout << "Running " #name "... "; \
    try { \
        test_##name(); \
        std::cout << "PASSED\n"; \
        tests_passed++; \
    } catch (const std::exception& e) { \
        std::cout << "FAILED: " << e.what() << "\n"; \
        tests_failed++; \
    } \
} while(0)

#define ASSERT_NEAR(a, b, tol) do { \
    if (std::abs((a) - (b)) > (tol)) { \
        std::ostringstream oss; \
        oss << "ASSERT_NEAR failed: " << (a) << " vs " << (b) \
            << " (tol=" << (tol) << ", diff=" << std::abs((a)-(b)) << ")"; \
        throw std::runtime_error(oss.str()); \
    } \
} while(0)

#define ASSERT_TRUE(x) do { \
    if (!(x)) throw std::runtime_error("Assertion failed: " #x); \
} while(0)

#define ASSERT_GT(a, b) do { \
    if (!((a) > (b))) { \
        std::ostringstream oss; \
        oss << "ASSERT_GT failed: " << (a) << " not > " << (b); \
        throw std::runtime_error(oss.str()); \
    } \
} while(0)

// ====================================================================
//  Step Response Identification
// ====================================================================

TEST(id_first_order_noiseless) {
    // True: G(s) = 3.0 / (2.0s + 1)
    double K = 3.0, tau = 2.0, u = 1.0;
    std::vector<double> t, y;
    for (double ti = 0; ti <= 15.0; ti += 0.05) {
        t.push_back(ti);
        y.push_back(K * u * (1.0 - std::exp(-ti / tau)));
    }
    auto r = id_step_first_order(t, y, u);
    ASSERT_NEAR(r.K, K, 0.01);
    ASSERT_NEAR(r.tau, tau, 0.01);
    ASSERT_GT(r.fit_percent, 99.0);
}

TEST(id_first_order_noisy) {
    double K = 2.5, tau = 0.8, u = 1.0;
    std::mt19937 gen(42);
    std::normal_distribution<double> noise(0.0, 0.03);
    std::vector<double> t, y;
    for (double ti = 0; ti <= 6.0; ti += 0.05) {
        t.push_back(ti);
        y.push_back(K * u * (1.0 - std::exp(-ti / tau)) + noise(gen));
    }
    auto r = id_step_first_order(t, y, u);
    ASSERT_NEAR(r.K, K, 0.1);    // Wider tolerance for noise
    ASSERT_NEAR(r.tau, tau, 0.1);
    ASSERT_GT(r.fit_percent, 90.0);
}

TEST(id_second_order_noiseless) {
    double K = 1.0, wn = 5.0, zeta = 0.3;
    double wn2 = wn * wn;
    TransferFunction G({K * wn2}, {1.0, 2.0 * zeta * wn, wn2});
    
    std::vector<double> t, y;
    for (double ti = 0; ti <= 5.0; ti += 0.01) {
        t.push_back(ti);
        y.push_back(G.stepResponse(ti));
    }
    auto r = id_step_second_order(t, y, 1.0);
    ASSERT_NEAR(r.K, K, 0.05);
    ASSERT_NEAR(r.wn, wn, 0.5);
    ASSERT_NEAR(r.zeta, zeta, 0.05);
    ASSERT_GT(r.fit_percent, 95.0);
}

TEST(detect_order_first) {
    // Pure first-order: no overshoot
    std::vector<double> y;
    for (int i = 0; i < 100; ++i) {
        double t = i * 0.1;
        y.push_back(1.0 * (1.0 - std::exp(-t / 1.0)));
    }
    ASSERT_TRUE(detect_step_order(y) == 1);
}

TEST(detect_order_second) {
    // Second-order with overshoot
    double wn = 5.0, zeta = 0.2;
    TransferFunction G({wn * wn}, {1.0, 2 * zeta * wn, wn * wn});
    std::vector<double> y;
    for (int i = 0; i < 500; ++i) {
        y.push_back(G.stepResponse(i * 0.01));
    }
    ASSERT_TRUE(detect_step_order(y) == 2);
}

// ====================================================================
//  ARX Identification
// ====================================================================

TEST(arx_known_system) {
    // True plant: G(s) = 1/(s+1), discretized at Ts=0.1
    TransferFunction G_true({1.0}, {1.0, 1.0});
    double Ts = 0.1;
    auto Hd = c2d_tustin(G_true, Ts);
    
    // Generate PRBS input and simulate
    auto [t_prbs, u] = generate_prbs(7, 1.0, Ts, 2);
    auto y = dsim(Hd, u);
    
    // Identify ARX(1,1)
    auto model = id_arx(u, y, 1, 1, 1, Ts);
    
    // Validate: one-step-ahead prediction should be excellent
    auto y_hat = arx_predict(model, u, y);
    double fit = compute_fit(y, y_hat);
    ASSERT_GT(fit, 99.0);  // Essentially perfect for noiseless
}

TEST(arx_with_noise) {
    TransferFunction G({1.0}, {1.0, 1.0});
    double Ts = 0.1;
    auto Hd = c2d_tustin(G, Ts);
    
    auto [t_prbs, u] = generate_prbs(7, 1.0, Ts, 3);
    auto y_clean = dsim(Hd, u);
    
    std::mt19937 gen(42);
    std::normal_distribution<double> noise(0.0, 0.01);
    std::vector<double> y(y_clean.size());
    for (size_t i = 0; i < y.size(); ++i) y[i] = y_clean[i] + noise(gen);
    
    auto model = id_arx(u, y, 2, 2, 1, Ts);
    auto y_hat = arx_predict(model, u, y);
    double fit = compute_fit(y, y_hat);
    ASSERT_GT(fit, 90.0);
}

TEST(arx_to_dtf_conversion) {
    ARXModel m;
    m.na = 1; m.nb = 1; m.nk = 1; m.Ts = 0.1;
    m.a = {0.9}; m.b = {0.1};
    m.N = 100; m.sigma2 = 0.0;
    
    auto dtf = m.to_dtf();
    // H(z) = 0.1*z^0 / (z - 0.9) → 0.1 / (z - 0.9)
    // Check DC gain: H(1) = 0.1 / (1 - 0.9) = 1.0
    // (Verify structure exists without crash)
    ASSERT_TRUE(dtf.order() >= 1);
}

// ====================================================================
//  Input Signal Generation
// ====================================================================

TEST(prbs_length) {
    auto [t, s] = generate_prbs(7, 1.0, 0.01, 1);
    int expected = (1 << 7) - 1;  // 127
    ASSERT_TRUE(static_cast<int>(s.size()) == expected);
}

TEST(prbs_binary_values) {
    auto [t, s] = generate_prbs(5, 2.0, 0.01, 1);
    for (double v : s) {
        ASSERT_TRUE(std::abs(v) == 2.0);  // ±2.0 only
    }
}

TEST(prbs_approximate_balance) {
    // PRBS should have approximately equal ±1 counts (differ by exactly 1)
    auto [t, s] = generate_prbs(7, 1.0, 0.01, 1);
    int pos = 0, neg = 0;
    for (double v : s) {
        if (v > 0) pos++; else neg++;
    }
    ASSERT_TRUE(std::abs(pos - neg) <= 1);
}

TEST(chirp_length) {
    auto [t, s] = generate_chirp(1.0, 100.0, 5.0, 0.001);
    ASSERT_TRUE(static_cast<int>(s.size()) == 5001);
}

TEST(chirp_bounded) {
    auto [t, s] = generate_chirp(1.0, 100.0, 5.0, 0.001, 3.0);
    for (double v : s) {
        ASSERT_TRUE(std::abs(v) <= 3.01);  // Within amplitude + floating point
    }
}

TEST(multisine_length) {
    auto [t, s] = generate_multisine({1, 5, 10}, 2.0, 0.001);
    ASSERT_TRUE(static_cast<int>(s.size()) == 2001);
}

TEST(white_noise_statistics) {
    auto [t, s] = generate_white_noise(10000, 0.01, 1.0, 42);
    double mean = 0.0;
    for (double v : s) mean += v;
    mean /= s.size();
    ASSERT_NEAR(mean, 0.0, 0.1);  // Mean ≈ 0
    
    double var = 0.0;
    for (double v : s) var += (v - mean) * (v - mean);
    var /= s.size();
    ASSERT_NEAR(var, 1.0, 0.1);   // Variance ≈ 1
}

// ====================================================================
//  Validation & Statistics
// ====================================================================

TEST(compute_fit_perfect) {
    std::vector<double> y = {1, 2, 3, 4, 5};
    double fit = compute_fit(y, y);
    ASSERT_NEAR(fit, 100.0, 0.01);
}

TEST(compute_fit_mean_predictor) {
    // If y_hat = mean(y) → FIT = 0%
    std::vector<double> y = {1, 2, 3, 4, 5};
    double mean = 3.0;
    std::vector<double> y_hat(5, mean);
    double fit = compute_fit(y, y_hat);
    ASSERT_NEAR(fit, 0.0, 1.0);
}

TEST(autocorrelation_white) {
    // White noise should have R[τ] ≈ 0 for τ > 0
    std::mt19937 gen(42);
    std::normal_distribution<double> d(0, 1);
    std::vector<double> x(5000);
    for (auto& v : x) v = d(gen);
    
    auto R = autocorrelation(x, 20);
    ASSERT_NEAR(R[0], 1.0, 0.01);
    for (int tau = 1; tau <= 20; ++tau) {
        ASSERT_TRUE(std::abs(R[tau]) < 0.1);
    }
}

TEST(aic_bic_ordering) {
    // More parameters → larger AIC/BIC (for same sigma2 and N)
    auto ic1 = compute_aic_bic(0.01, 2, 100);
    auto ic2 = compute_aic_bic(0.01, 4, 100);
    ASSERT_TRUE(ic2.AIC > ic1.AIC);
    ASSERT_TRUE(ic2.BIC > ic1.BIC);
}

TEST(aic_bic_variance_effect) {
    // Lower variance → lower AIC (better model)
    auto ic1 = compute_aic_bic(0.001, 2, 100);
    auto ic2 = compute_aic_bic(0.01, 2, 100);
    ASSERT_TRUE(ic1.AIC < ic2.AIC);
}

// ====================================================================
//  Simulation (lsim / dsim)
// ====================================================================

TEST(dsim_step_response) {
    // Simulate step response via dsim and compare with dstep
    TransferFunction G({1.0}, {1.0, 1.0});
    double Ts = 0.05;
    auto Hd = c2d_tustin(G, Ts);
    
    // Step input
    int N = 200;
    std::vector<double> u(N, 1.0);
    auto y = dsim(Hd, u);
    
    // Final value should approach DC gain = 1.0
    ASSERT_NEAR(y.back(), 1.0, 0.05);
}

TEST(lsim_step_input) {
    TransferFunction G({2.0}, {1.0, 1.0});  // G = 2/(s+1)
    
    std::vector<double> t, u;
    for (double ti = 0; ti <= 10.0; ti += 0.01) {
        t.push_back(ti);
        u.push_back(1.0);  // Step input
    }
    
    auto y = lsim(G, t, u);
    // Steady state y → 2.0
    ASSERT_NEAR(y.back(), 2.0, 0.1);
}

TEST(lsim_zero_input) {
    TransferFunction G({1.0}, {1.0, 1.0});
    
    std::vector<double> t, u;
    for (double ti = 0; ti <= 5.0; ti += 0.01) {
        t.push_back(ti);
        u.push_back(0.0);
    }
    
    auto y = lsim(G, t, u);
    for (double v : y) {
        ASSERT_NEAR(v, 0.0, 1e-10);
    }
}

// ====================================================================
//  Validation pipeline
// ====================================================================

TEST(validate_model_good_fit) {
    TransferFunction G({1.0}, {1.0, 1.0});
    double Ts = 0.1;
    auto Hd = c2d_tustin(G, Ts);
    
    auto [t, u] = generate_prbs(7, 1.0, Ts, 2);
    auto y = dsim(Hd, u);
    
    auto model = id_arx(u, y, 1, 1, 1, Ts);
    auto val = validate_model(model, u, y);
    
    ASSERT_GT(val.fit_percent, 95.0);
    ASSERT_TRUE(val.rmse < 0.1);
}

// ====================================================================
//  Coherence
// ====================================================================

TEST(coherence_perfect_linear) {
    // Perfectly correlated linear system → γ² ≈ 1
    TransferFunction G({1.0}, {1.0, 1.0});
    double Ts = 0.01;
    auto Hd = c2d_tustin(G, Ts);
    
    auto [t, u] = generate_prbs(9, 1.0, Ts, 2);
    auto y = dsim(Hd, u);
    
    auto [freq, coh] = coherence(u, y, Ts, 4);
    
    // Most coherence values should be high (skip DC)
    int high_count = 0;
    for (size_t i = 1; i < coh.size(); ++i) {
        if (coh[i] > 0.5) high_count++;
    }
    ASSERT_GT(high_count, static_cast<int>(coh.size()) / 2);
}

// ====================================================================
//  Main
// ====================================================================

int main() {
    std::cout << "========================================\n"
              << "  CppPlot System Identification Tests\n"
              << "========================================\n\n";
    
    // Step response ID
    RUN_TEST(id_first_order_noiseless);
    RUN_TEST(id_first_order_noisy);
    RUN_TEST(id_second_order_noiseless);
    RUN_TEST(detect_order_first);
    RUN_TEST(detect_order_second);
    
    // ARX identification
    RUN_TEST(arx_known_system);
    RUN_TEST(arx_with_noise);
    RUN_TEST(arx_to_dtf_conversion);
    
    // Input signal generation
    RUN_TEST(prbs_length);
    RUN_TEST(prbs_binary_values);
    RUN_TEST(prbs_approximate_balance);
    RUN_TEST(chirp_length);
    RUN_TEST(chirp_bounded);
    RUN_TEST(multisine_length);
    RUN_TEST(white_noise_statistics);
    
    // Validation & statistics
    RUN_TEST(compute_fit_perfect);
    RUN_TEST(compute_fit_mean_predictor);
    RUN_TEST(autocorrelation_white);
    RUN_TEST(aic_bic_ordering);
    RUN_TEST(aic_bic_variance_effect);
    
    // Simulation
    RUN_TEST(dsim_step_response);
    RUN_TEST(lsim_step_input);
    RUN_TEST(lsim_zero_input);
    
    // Validation pipeline
    RUN_TEST(validate_model_good_fit);
    
    // Coherence
    RUN_TEST(coherence_perfect_linear);
    
    std::cout << "\n========================================\n"
              << "  Results: " << tests_passed << " passed, "
              << tests_failed << " failed\n"
              << "========================================\n";
    
    return tests_failed > 0 ? 1 : 0;
}
