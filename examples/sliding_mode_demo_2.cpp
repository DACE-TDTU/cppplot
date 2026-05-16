/**
 * @file sliding_mode_demo.cpp
 * @brief Demonstration of Sliding Mode Control algorithms
 *
 * This example demonstrates:
 * 1. Conventional SMC with sign function
 * 2. Super-Twisting Algorithm (STA)
 * 3. Boundary layer for chattering reduction
 * 4. Application to different systems
 *
 * Compile:
 * g++ -std=c++17 -I"../include" sliding_mode_demo.cpp -o
 * output/sliding_mode_demo.exe Run:
 *   ./output/sliding_mode_demo.exe
 */

#include <cppplot/control/control.hpp>
#include <cppplot/control/nonlinear/sliding_mode.hpp>
#include <iomanip>
#include <iostream>

using namespace cppplot;
using namespace cppplot::control;
using namespace cppplot::control::nonlinear;

int main() {
  std::cout
      << "╔══════════════════════════════════════════════════════════════╗\n";
  std::cout
      << "║         SLIDING MODE CONTROL DEMONSTRATION                   ║\n";
  std::cout
      << "╚══════════════════════════════════════════════════════════════╝\n\n";

  // ================================================================
  // Example 1: Double Integrator - Conventional vs Super-Twisting
  // ================================================================
  std::cout
      << "═══════════════════════════════════════════════════════════════\n";
  std::cout << "Example 1: Double Integrator System\n";
  std::cout << "   ẋ₁ = x₂\n";
  std::cout << "   ẋ₂ = u + d(t)   (d = disturbance)\n";
  std::cout
      << "═══════════════════════════════════════════════════════════════\n\n";

  // Initial conditions
  std::vector<double> x0 = {2.0, 1.0}; // Start at (2, 1)
  double t_final = 5.0;
  double dt = 0.001;

  // Reference signal (step to 0)
  auto reference = [](double t) { return 0.0; };

  // Disturbance (matched uncertainty)
  auto disturbance = [](double t) {
    return 0.5 * std::sin(2 * t); // Sinusoidal disturbance
  };

  // ---- Conventional SMC (with sign function) ----
  std::cout << "1.1 Conventional SMC (sign function):\n";
  auto pair_conv =
      createDoubleIntegratorSMC(SMCType::CONVENTIONAL, 5.0, 15.0, false);
  auto ctrl_conv = pair_conv.first;
  auto dyn_conv = pair_conv.second;

  SMCSimulator sim_conv(ctrl_conv, dyn_conv, 2);
  auto result_conv = sim_conv.simulate(x0, t_final, dt, reference, disturbance);

  printSMCSummary(
      SMCConfig{SMCType::CONVENTIONAL, ReachingLaw::CONSTANT_RATE, 15.0},
      SlidingSurfaceConfig::linear({5.0, 1.0}), result_conv);

  // ---- Super-Twisting Algorithm ----
  std::cout << "\n1.2 Super-Twisting Algorithm:\n";
  auto pair_sta =
      createDoubleIntegratorSMC(SMCType::SUPER_TWISTING, 5.0, 15.0, false);
  auto ctrl_sta = pair_sta.first;
  auto dyn_sta = pair_sta.second;

  SMCSimulator sim_sta(ctrl_sta, dyn_sta, 2);
  auto result_sta = sim_sta.simulate(x0, t_final, dt, reference, disturbance);

  printSMCSummary(SMCConfig{SMCType::SUPER_TWISTING},
                  SlidingSurfaceConfig::linear({5.0, 1.0}), result_sta);

  // ---- Boundary Layer (chattering reduction) ----
  std::cout << "\n1.3 Conventional SMC with Boundary Layer:\n";
  auto pair_bl =
      createDoubleIntegratorSMC(SMCType::CONVENTIONAL, 5.0, 15.0, true);
  auto ctrl_bl = pair_bl.first;
  auto dyn_bl = pair_bl.second;

  SMCSimulator sim_bl(ctrl_bl, dyn_bl, 2);
  auto result_bl = sim_bl.simulate(x0, t_final, dt, reference, disturbance);

  SMCConfig cfg_bl;
  cfg_bl.type = SMCType::CONVENTIONAL;
  cfg_bl.use_boundary_layer = true;
  cfg_bl.boundary_thickness = 0.1;
  printSMCSummary(cfg_bl, SlidingSurfaceConfig::linear({5.0, 1.0}), result_bl);

  // Plot comparison
  std::cout << "\nGenerating comparison plots...\n";
  plotSMCComparison({result_conv, result_sta, result_bl},
                    {"Conventional (sign)", "Super-Twisting", "Boundary Layer"},
                    "smc_double_integrator_comparison");

  // Plot individual responses
  plotSMCResponse(result_conv, "smc_conventional_response");
  plotSMCResponse(result_sta, "smc_supertwisting_response");

  // Plot chattering analysis
  plotChatteringAnalysis(result_conv, result_bl, "smc_chattering_comparison");

  // ================================================================
  // Example 2: Mass-Spring-Damper System
  // ================================================================
  std::cout
      << "\n═══════════════════════════════════════════════════════════════\n";
  std::cout << "Example 2: Mass-Spring-Damper System\n";
  std::cout << "   mẍ + cẋ + kx = u\n";
  std::cout << "   m=1, c=0.5, k=1\n";
  std::cout
      << "═══════════════════════════════════════════════════════════════\n\n";

  auto pair_msd =
      createMassSpringDamperSMC(1.0, 0.5, 1.0, SMCType::SUPER_TWISTING, 25.0);
  auto ctrl_msd = pair_msd.first;
  auto dyn_msd = pair_msd.second;

  SMCSimulator sim_msd(ctrl_msd, dyn_msd, 2);
  std::vector<double> x0_msd = {1.5, 0.5}; // Initial displacement and velocity

  auto result_msd =
      sim_msd.simulate(x0_msd, 8.0, 0.001, reference, [](double t) {
        return 0.3 * std::sin(5 * t);
      }); // Higher freq disturbance

  std::cout << "Mass-Spring-Damper with Super-Twisting SMC:\n";
  SMCConfig cfg_msd;
  cfg_msd.type = SMCType::SUPER_TWISTING;
  cfg_msd.use_boundary_layer = true;
  printSMCSummary(cfg_msd, SlidingSurfaceConfig::linear({5.0, 1.0}),
                  result_msd);

  plotSMCResponse(result_msd, "smc_mass_spring_damper");
  plotSlidingSurfaceAnalysis(result_msd, "smc_msd_surface_analysis");

  // ================================================================
  // Example 3: DC Motor Speed Control
  // ================================================================
  std::cout
      << "\n═══════════════════════════════════════════════════════════════\n";
  std::cout << "Example 3: DC Motor Speed Control\n";
  std::cout << "   Jω̇ + Bω = (Km/R)V\n";
  std::cout << "   J=0.01, B=0.001, Km=0.05, R=2.5\n";
  std::cout
      << "═══════════════════════════════════════════════════════════════\n\n";

  auto pair_motor =
      createDCMotorSMC(0.01, 0.001, 0.05, 2.5, SMCType::SUPER_TWISTING, 50.0);
  auto ctrl_motor = pair_motor.first;
  auto dyn_motor = pair_motor.second;

  SMCSimulator sim_motor(ctrl_motor, dyn_motor, 2);
  std::vector<double> x0_motor = {0.0, 0.0}; // Start from rest

  // Step reference for speed
  auto speed_ref = [](double t) {
    if (t < 1.0)
      return 0.0;
    if (t < 3.0)
      return 100.0; // 100 rad/s
    return 50.0;    // Step down to 50 rad/s
  };

  // Load torque disturbance
  auto load_disturbance = [](double t) {
    if (t > 2.0 && t < 2.5)
      return 5.0; // Sudden load
    return 0.0;
  };

  auto result_motor =
      sim_motor.simulate(x0_motor, 5.0, 0.0005, speed_ref, load_disturbance);

  std::cout << "DC Motor Speed Control with Integral SMC:\n";
  SMCConfig cfg_motor;
  cfg_motor.type = SMCType::SUPER_TWISTING;
  SlidingSurfaceConfig surf_motor;
  surf_motor.type = SurfaceType::INTEGRAL;
  printSMCSummary(cfg_motor, surf_motor, result_motor);

  plotSMCResponse(result_motor, "smc_dc_motor_speed");

  // ================================================================
  // Example 4: Tracking Control
  // ================================================================
  std::cout
      << "\n═══════════════════════════════════════════════════════════════\n";
  std::cout << "Example 4: Sinusoidal Tracking\n";
  std::cout
      << "═══════════════════════════════════════════════════════════════\n\n";

  // Create controller for tracking
  SlidingSurfaceConfig track_surf;
  track_surf.type = SurfaceType::LINEAR;
  track_surf.C = {10.0, 1.0}; // Higher λ for faster tracking

  SMCConfig track_cfg;
  track_cfg.type = SMCType::QUASI_CONTINUOUS;
  track_cfg.K = 20.0;
  track_cfg.boundary_thickness = 0.05;

  SlidingModeController ctrl_track(track_cfg, track_surf, 2);

  auto dyn_track = [](double t, const std::vector<double> &x, double u) {
    return std::vector<double>{x[1], u};
  };

  SMCSimulator sim_track(ctrl_track, dyn_track, 2);

  // Sinusoidal reference
  auto sin_ref = [](double t) { return std::sin(2.0 * t); };

  auto result_track = sim_track.simulate({0.0, 0.0}, 6.0, 0.001, sin_ref);

  std::cout << "Tracking with Quasi-Continuous SMC:\n";
  printSMCSummary(track_cfg, track_surf, result_track);

  // Custom plot for tracking
  using namespace cppplot;
  figure(900, 600);
  suptitle("Sinusoidal Tracking with Quasi-Continuous SMC");
  layout(2, 1);

  subplot(2, 1, 1);
  auto x1_track = result_track.getState(0);
  plot(result_track.time, x1_track, "-",
       opts({{"color", "blue"}, {"linewidth", "2"}, {"label", "Output"}}));
  plot(result_track.time, result_track.reference, "--",
       opts({{"color", "red"}, {"linewidth", "1.5"}, {"label", "Reference"}}));
  xlabel("Time (s)");
  ylabel("Position");
  title("Output Tracking");
  legend(true);
  grid(true);

  subplot(2, 1, 2);
  std::vector<double> tracking_error;
  for (size_t i = 0; i < x1_track.size(); ++i) {
    tracking_error.push_back(result_track.reference[i] - x1_track[i]);
  }
  plot(result_track.time, tracking_error, "-",
       opts({{"color", "green"}, {"linewidth", "1.5"}}));
  axhline(0.0, opts({{"color", "black"}, {"linestyle", "--"}}));
  xlabel("Time (s)");
  ylabel("Error");
  title("Tracking Error");
  grid(true);

  savefig("smc_tracking_sinusoid.svg");
  savefig("smc_tracking_sinusoid.png");

  // ================================================================
  // Summary
  // ================================================================
  std::cout
      << "\n╔══════════════════════════════════════════════════════════════╗\n";
  std::cout
      << "║                    GENERATED PLOTS                           ║\n";
  std::cout
      << "╠══════════════════════════════════════════════════════════════╣\n";
  std::cout
      << "║ 1. smc_double_integrator_comparison.svg/png                  ║\n";
  std::cout
      << "║ 2. smc_conventional_response.svg/png                         ║\n";
  std::cout
      << "║ 3. smc_supertwisting_response.svg/png                        ║\n";
  std::cout
      << "║ 4. smc_chattering_comparison.svg/png                         ║\n";
  std::cout
      << "║ 5. smc_mass_spring_damper.svg/png                            ║\n";
  std::cout
      << "║ 6. smc_msd_surface_analysis.svg/png                          ║\n";
  std::cout
      << "║ 7. smc_dc_motor_speed.svg/png                                ║\n";
  std::cout
      << "║ 8. smc_tracking_sinusoid.svg/png                             ║\n";
  std::cout
      << "╚══════════════════════════════════════════════════════════════╝\n";

  return 0;
}
