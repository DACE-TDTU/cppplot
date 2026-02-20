/**
 * @file sysid_demo.cpp
 * @brief Chapter 9 — System Identification Demo
 *
 * Demonstrates:
 *  1. Step response identification (1st & 2nd order)
 *  2. ARX parametric identification with PRBS input
 *  3. Model validation (FIT, residual analysis, AIC/BIC)
 *  4. Input signal generation (PRBS, chirp)
 *  5. Frequency-domain identification from Bode data
 *  6. Automatic model order selection
 *
 * Build (from cppplot/build):
 *   cmake .. && cmake --build . --target sysid_demo
 *
 * Or standalone:
 *   g++ -std=c++17 -I../include sysid_demo.cpp -o sysid_demo
 */

#include <cmath>
#include <cppplot/control/control.hpp>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

using namespace cppplot;
using namespace cppplot::control;
using namespace cppplot::control::sysid;

// ---------------------------------------------------------------------------
//  Helper: simulate true plant and add noise
// ---------------------------------------------------------------------------
std::vector<double> simulate_true_first_order(const std::vector<double> &t,
                                              double K, double tau,
                                              double u_step, double noise_std,
                                              unsigned seed = 42) {
  std::mt19937 gen(seed);
  std::normal_distribution<double> noise(0.0, noise_std);
  std::vector<double> y(t.size());
  for (size_t i = 0; i < t.size(); ++i) {
    y[i] = K * u_step * (1.0 - std::exp(-t[i] / tau)) + noise(gen);
  }
  return y;
}

std::vector<double> simulate_true_second_order(const std::vector<double> &t,
                                               double K, double wn, double zeta,
                                               double u_step, double noise_std,
                                               unsigned seed = 42) {
  // Use TF step response
  double wn2 = wn * wn;
  TransferFunction G({K * wn2}, {1.0, 2.0 * zeta * wn, wn2});

  std::mt19937 gen(seed);
  std::normal_distribution<double> noise(0.0, noise_std);
  std::vector<double> y(t.size());
  for (size_t i = 0; i < t.size(); ++i) {
    y[i] = G.stepResponse(t[i]) * u_step + noise(gen);
  }
  return y;
}

// ===========================================================================
int main() {
  std::cout << R"(
╔══════════════════════════════════════════════════════════════╗
║     Chapter 9 — System Identification Demo (CppPlot)        ║
╚══════════════════════════════════════════════════════════════╝
)" << std::endl;

  // -----------------------------------------------------------------------
  //  DEMO 1: First-order step response identification
  // -----------------------------------------------------------------------
  std::cout << "\n▶ DEMO 1: First-Order Step Response Identification\n"
            << std::string(55, '-') << std::endl;

  // True plant: G(s) = 2.5 / (0.8s + 1)
  double K_true = 2.5, tau_true = 0.8, u_step = 1.0;

  // Generate time vector and noisy response
  std::vector<double> t1;
  for (double t = 0.0; t <= 6.0; t += 0.05)
    t1.push_back(t);
  auto y1 = simulate_true_first_order(t1, K_true, tau_true, u_step, 0.02);

  // Identify
  auto id1 = id_step_first_order(t1, y1, u_step);
  print_step_id(id1);

  std::cout << "  True:  K = " << K_true << ", τ = " << tau_true << std::endl;
  std::cout << "  Error: ΔK = " << std::abs(id1.K - K_true)
            << ", Δτ = " << std::abs(id1.tau - tau_true) << "\n";

  // -----------------------------------------------------------------------
  //  DEMO 2: Second-order step response identification
  // -----------------------------------------------------------------------
  std::cout << "\n▶ DEMO 2: Second-Order Step Response Identification\n"
            << std::string(55, '-') << std::endl;

  double K2_true = 1.0, wn_true = 5.0, zeta_true = 0.3;

  std::vector<double> t2;
  for (double t = 0.0; t <= 5.0; t += 0.01)
    t2.push_back(t);
  auto y2 =
      simulate_true_second_order(t2, K2_true, wn_true, zeta_true, u_step, 0.01);

  auto id2 = id_step_second_order(t2, y2, u_step);
  print_step_id(id2);

  std::cout << "  True:  K = " << K2_true << ", ωn = " << wn_true
            << ", ζ = " << zeta_true << std::endl;
  std::cout << "  Error: ΔK = " << std::abs(id2.K - K2_true)
            << ", Δωn = " << std::abs(id2.wn - wn_true)
            << ", Δζ = " << std::abs(id2.zeta - zeta_true) << "\n";

  // -----------------------------------------------------------------------
  //  DEMO 3: ARX Identification with PRBS Input
  // -----------------------------------------------------------------------
  std::cout << "\n▶ DEMO 3: ARX Identification with PRBS Input\n"
            << std::string(55, '-') << std::endl;

  // True plant: G(s) = 1 / (s + 1) discretized at Ts = 0.1
  double Ts = 0.1;
  TransferFunction G_true({1.0}, {1.0, 1.0});
  auto Hd_true = c2d_tustin(G_true, Ts);

  // Generate PRBS input
  auto prbs_data = generate_prbs(7, 1.0, Ts, 3);
  auto t_prbs = prbs_data.first;
  auto u_prbs = prbs_data.second;
  int N = static_cast<int>(u_prbs.size());

  // Simulate plant output with noise
  auto y_clean = dsim(Hd_true, u_prbs);
  std::mt19937 gen(123);
  std::normal_distribution<double> noise(0.0, 0.01);
  std::vector<double> y_noisy(N);
  for (int k = 0; k < N; ++k)
    y_noisy[k] = y_clean[k] + noise(gen);

  // Split data: 70% training, 30% validation
  int N_train = static_cast<int>(0.7 * N);
  std::vector<double> u_train(u_prbs.begin(), u_prbs.begin() + N_train);
  std::vector<double> y_train(y_noisy.begin(), y_noisy.begin() + N_train);
  std::vector<double> u_val(u_prbs.begin() + N_train, u_prbs.end());
  std::vector<double> y_val(y_noisy.begin() + N_train, y_noisy.end());

  // Identify ARX(2,2) model
  auto arx_model = id_arx(u_train, y_train, 2, 2, 1, Ts);
  print_arx(arx_model);

  // Validate on held-out data
  auto val = validate_model(arx_model, u_val, y_val);
  print_validation(val);

  // Convert to continuous TF
  auto G_id = arx_to_tf(arx_model);
  std::cout << "\nIdentified continuous TF: " << G_id.toString() << std::endl;
  std::cout << "True continuous TF:       " << G_true.toString() << "\n";

  // -----------------------------------------------------------------------
  //  DEMO 4: Automatic Model Order Selection
  // -----------------------------------------------------------------------
  std::cout << "\n▶ DEMO 4: Automatic Model Order Selection (AIC/BIC)\n"
            << std::string(55, '-') << std::endl;

  std::cout << std::setw(6) << "na" << std::setw(6) << "nb" << std::setw(14)
            << "AIC" << std::setw(14) << "BIC" << std::setw(12) << "σ²"
            << std::endl;
  std::cout << std::string(52, '-') << std::endl;

  for (int na = 1; na <= 4; ++na) {
    for (int nb = 1; nb <= 4; ++nb) {
      try {
        auto m = id_arx(u_train, y_train, na, nb, 1, Ts);
        auto ic = compute_aic_bic(m.sigma2, m.num_params(), m.N);
        std::cout << std::setw(6) << na << std::setw(6) << nb << std::setw(14)
                  << std::fixed << std::setprecision(2) << ic.AIC
                  << std::setw(14) << ic.BIC << std::setw(12) << std::scientific
                  << std::setprecision(3) << m.sigma2 << std::endl;
      } catch (...) {
        continue;
      }
    }
  }

  auto best = id_arx_auto(u_train, y_train, 4, 1, Ts);
  std::cout << "\nBest model (BIC): ARX(" << best.na << "," << best.nb << ")\n";

  // -----------------------------------------------------------------------
  //  DEMO 5: Input Signal Generation
  // -----------------------------------------------------------------------
  std::cout << "\n▶ DEMO 5: Input Signal Generation\n"
            << std::string(55, '-') << std::endl;

  // PRBS
  auto prbs_res = generate_prbs(5, 2.0, 0.01, 1);
  auto t_p = prbs_res.first;
  auto s_p = prbs_res.second;
  std::cout << "PRBS-5:  " << s_p.size()
            << " samples, period = " << ((1 << 5) - 1) << std::endl;

  // Chirp
  auto chirp_res = generate_chirp(0.1, 50.0, 10.0, 0.001);
  auto t_c = chirp_res.first;
  auto s_c = chirp_res.second;
  std::cout << "Chirp:   " << s_c.size() << " samples, 0.1-50 Hz over 10 s\n";

  // Multisine
  auto multi_res = generate_multisine({1, 2, 5, 10, 20}, 2.0, 0.001);
  auto t_m = multi_res.first;
  auto s_m = multi_res.second;
  std::cout << "Multisine: " << s_m.size() << " samples, 5 frequencies\n";

  // White noise
  auto white_res = generate_white_noise(500, 0.01, 0.5, 42);
  auto t_w = white_res.first;
  auto s_w = white_res.second;
  std::cout << "White noise: " << s_w.size() << " samples, σ = 0.5\n";

  // -----------------------------------------------------------------------
  //  DEMO 6: Frequency-Domain Identification
  // -----------------------------------------------------------------------
  std::cout << "\n▶ DEMO 6: Frequency-Domain Identification\n"
            << std::string(55, '-') << std::endl;

  // Generate "measured" Bode data from true system G(s) = 10/(s+2)
  TransferFunction G_bode_true({10.0}, {1.0, 2.0});

  std::vector<double> omega;
  for (double w = 0.01; w <= 1000.0; w *= 1.2)
    omega.push_back(w);

  std::vector<double> mag_dB_meas(omega.size());
  std::vector<double> phase_deg_meas(omega.size());
  std::mt19937 gen2(99);
  std::normal_distribution<double> bode_noise(0.0, 0.5);

  for (size_t i = 0; i < omega.size(); ++i) {
    mag_dB_meas[i] = G_bode_true.mag_dB(omega[i]) + bode_noise(gen2);
    phase_deg_meas[i] = G_bode_true.phase_deg(omega[i]) + bode_noise(gen2);
  }

  auto bode_result = id_bode(omega, mag_dB_meas, phase_deg_meas);

  std::cout << "Identified TF:  " << bode_result.G.toString() << std::endl;
  std::cout << "True TF:        " << G_bode_true.toString() << std::endl;
  std::cout << "Mag RMS error:  " << std::fixed << std::setprecision(2)
            << bode_result.mag_rms_error << " dB" << std::endl;
  std::cout << "Phase RMS error: " << bode_result.phase_rms_error << " deg"
            << std::endl;

  // -----------------------------------------------------------------------
  //  DEMO 7: lsim — General Input Simulation
  // -----------------------------------------------------------------------
  std::cout << "\n▶ DEMO 7: lsim — General Input Simulation\n"
            << std::string(55, '-') << std::endl;

  TransferFunction G_sim({5.0}, {1.0, 3.0, 2.0}); // G = 5/(s²+3s+2)

  std::vector<double> t_sim, u_sim;
  for (double t = 0; t <= 10.0; t += 0.01) {
    t_sim.push_back(t);
    u_sim.push_back(std::sin(t)); // Sinusoidal input
  }

  auto y_sim = lsim(G_sim, t_sim, u_sim);

  std::cout << "System: G(s) = 5 / (s² + 3s + 2)" << std::endl;
  std::cout << "Input:  u(t) = sin(t)" << std::endl;
  std::cout << "Output samples:" << std::endl;
  for (int i = 0; i <= 10; ++i) {
    int idx = i * 100;
    if (idx < static_cast<int>(y_sim.size())) {
      std::cout << "  t=" << std::setw(5) << std::fixed << std::setprecision(1)
                << t_sim[idx] << " s → y=" << std::setprecision(4) << y_sim[idx]
                << std::endl;
    }
  }

  // -----------------------------------------------------------------------
  //  DEMO 8: Coherence Analysis
  // -----------------------------------------------------------------------
  std::cout << "\n▶ DEMO 8: Coherence Analysis\n"
            << std::string(55, '-') << std::endl;

  // Use PRBS→plant→noisy output
  auto coh_result = coherence(u_prbs, y_noisy, Ts, 4);
  auto coh_freq = coh_result.first;
  auto coh_val = coh_result.second;

  std::cout << "Coherence γ²(f) at selected frequencies:" << std::endl;
  int step = std::max(1, static_cast<int>(coh_freq.size()) / 10);
  for (size_t i = 0; i < coh_freq.size(); i += step) {
    std::cout << "  f = " << std::setw(6) << std::fixed << std::setprecision(2)
              << coh_freq[i] << " Hz → γ² = " << std::setprecision(4)
              << coh_val[i] << std::endl;
  }

  std::cout << "\n✓ All demos completed successfully.\n" << std::endl;
  return 0;
}
