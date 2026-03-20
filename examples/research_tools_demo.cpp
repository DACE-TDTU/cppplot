/**
 * @file research_tools_demo.cpp
 * @brief Comprehensive demonstration of CppPlot Research Tools
 *
 * This demo showcases all the advanced research modules:
 * 1. Kalman Filter (state estimation)
 * 2. LQG Controller (optimal control with noise)
 * 3. MPC Controller (predictive control with constraints)
 *
 * Compile: g++ -std=c++17 -I "../include" research_tools_demo.cpp -o research_demo.exe
 */

#include <cmath>
#include <cppplot/control/control.hpp>
#include <cppplot/control/kalman.hpp>
#include <cppplot/control/lqg.hpp>
#include <cppplot/control/mpc.hpp>
#include <iomanip>
#include <iostream>
#include <random>


using namespace cppplot::control;

void printHeader(const std::string &title) {
  std::cout << "\n" << std::string(70, '=') << std::endl;
  std::cout << "  " << title << std::endl;
  std::cout << std::string(70, '=') << std::endl;
}

// ============================================================
//               DEMO 1: VEHICLE POSITION TRACKING
// ============================================================
void demo_kalman_vehicle_tracking() {
  printHeader("DEMO 1: Kalman Filter - Vehicle Position Tracking");

  /**
   * System: Vehicle moving in 1D with acceleration input
   * State: [position, velocity]
   * Measurement: position (GPS-like, noisy)
   *
   * This demonstrates sensor fusion capability.
   */

  std::cout << "\nScenario: Track vehicle position using noisy GPS"
            << std::endl;
  std::cout << "True initial position: 0 m, velocity: 10 m/s" << std::endl;

  double dt = 0.1; // 100ms sampling

  // Discrete-time model
  Matrix A = {{1, dt}, {0, 1}};
  Matrix B = {{0.5 * dt * dt}, {dt}};
  Matrix C = {{1, 0}}; // Measure position only

  // Noise: σ_accel = 0.5 m/s², σ_gps = 2 m
  double sigma_a = 0.5;
  Matrix Q = {
      {0.25 * std::pow(dt, 4) * sigma_a * sigma_a,
       0.5 * std::pow(dt, 3) * sigma_a * sigma_a},
      {0.5 * std::pow(dt, 3) * sigma_a * sigma_a, dt * dt * sigma_a * sigma_a}};
  Matrix R = {{4.0}}; // GPS noise variance

  KalmanFilter kf(A, B, C, Q, R);
  kf.setInitialState(Matrix({{0}, {0}}), Matrix::eye(2) * 100);

  // Simulation
  std::random_device rd;
  std::mt19937 gen(42);
  std::normal_distribution<> gps_noise(0, 2.0);
  std::normal_distribution<> accel_noise(0, sigma_a);

  double true_pos = 0, true_vel = 10;

  std::cout << "\nTime [s] | True Pos | GPS Meas | Est Pos | Est Vel"
            << std::endl;
  std::cout << std::string(55, '-') << std::endl;

  for (int k = 0; k <= 50; ++k) {
    double t = k * dt;

    // Apply random acceleration (simulate real driving)
    double accel = accel_noise(gen);
    Matrix u = {{accel}};

    true_pos += true_vel * dt + 0.5 * accel * dt * dt;
    true_vel += accel * dt;

    // GPS measurement with noise
    double gps = true_pos + gps_noise(gen);
    Matrix y = {{gps}};

    // Kalman filter update
    auto est = kf.update(y, u);

    if (k % 5 == 0) {
      std::cout << std::fixed << std::setprecision(2);
      std::cout << std::setw(8) << t << " | " << std::setw(8) << true_pos
                << " | " << std::setw(8) << gps << " | " << std::setw(7)
                << est.x_hat(0, 0) << " | " << std::setw(7) << est.x_hat(1, 0)
                << std::endl;
    }
  }

  std::cout << "\n✓ Kalman filter smooths noisy GPS measurements!" << std::endl;
}

// ============================================================
//            DEMO 2: INVERTED PENDULUM LQG
// ============================================================
void demo_lqg_pendulum() {
  printHeader("DEMO 2: LQG Controller - Inverted Pendulum Stabilization");

  /**
   * Linearized inverted pendulum around upright position
   * State: [theta, theta_dot]
   * Control: Force on cart
   * Measurement: Angle (encoder)
   *
   * Parameters: m=0.1kg, l=0.5m, g=9.81m/s²
   */

  std::cout << "\nScenario: Balance inverted pendulum with noisy encoder"
            << std::endl;

  // Linearized system (around theta=0)
  // theta_ddot = (g/l)*theta + (1/ml²)*u
  double g = 9.81, l = 0.5, m = 0.1;
  double a = g / l;             // = 19.62
  double b = 1.0 / (m * l * l); // = 40

  // Continuous: A = [0 1; a 0], B = [0; b]
  // Discretize with dt=0.01
  double dt = 0.01;
  Matrix A = {{1, dt}, {a * dt, 1}};
  Matrix B = {{0.5 * dt * dt * b}, {dt * b}};
  Matrix C = {{1, 0}}; // Measure angle

  // Weights
  Matrix Q_lqr = {{100, 0}, {0, 1}}; // Penalize angle heavily
  Matrix R_lqr = {{0.01}};

  // Noise
  Matrix W = Matrix::eye(2) * 0.001;
  Matrix V = {{0.001}}; // Encoder noise

  // Design LQG
  auto lqg = designLQG(A, B, C, Q_lqr, R_lqr, W, V);

  std::cout << "\nLQG Design Results:" << std::endl;
  std::cout << "  State feedback K = [" << lqg.K(0, 0) << ", " << lqg.K(0, 1)
            << "]" << std::endl;
  std::cout << "  Observer gain  L = [" << lqg.L(0, 0) << "; " << lqg.L(1, 0)
            << "]" << std::endl;

  // Create LQG controller and simulate
  LQGController controller(A, B, C, Q_lqr, R_lqr, W, V);

  // Initial condition: 5 degree tilt
  Matrix x = {{5.0 * M_PI / 180}, {0}}; // 5 degrees
  Matrix x_est0 = {{0}, {0}};
  controller.setInitialEstimate(x_est0, Matrix::eye(2) * 10);

  std::random_device rd;
  std::mt19937 gen(42);
  std::normal_distribution<> w_noise(0, 0.01);
  std::normal_distribution<> v_noise(0, std::sqrt(0.001));

  std::cout << "\nSimulation (initial angle = 5°):" << std::endl;
  std::cout << "Time [s] | Angle [°] | Control [N]" << std::endl;
  std::cout << std::string(40, '-') << std::endl;

  Matrix u = {{0}};
  for (int k = 0; k <= 200; ++k) {
    double t = k * dt;

    double meas = x(0, 0) + v_noise(gen);
    Matrix y = {{meas}};

    u = controller.computeControl(y, u);

    if (k % 20 == 0) {
      std::cout << std::fixed << std::setprecision(3);
      std::cout << std::setw(8) << t << " | " << std::setw(9)
                << x(0, 0) * 180 / M_PI << " | " << std::setw(11) << u(0, 0)
                << std::endl;
    }

    // Update state
    Matrix w = {{w_noise(gen)}, {w_noise(gen)}};
    x = A * x + B * u + w;
  }

  std::cout << "\n✓ LQG stabilizes pendulum from 5° initial tilt!" << std::endl;
}

// ============================================================
//             DEMO 3: ROBOT ARM MPC
// ============================================================
void demo_mpc_robot_arm() {
  printHeader("DEMO 3: MPC Controller - Robot Arm with Torque Limits");

  /**
   * Single-link robot arm with torque constraints
   * State: [angle, angular_velocity]
   * Control: Motor torque
   * Constraint: |torque| ≤ 5 N⋅m
   */

  std::cout << "\nScenario: Move robot arm from 0° to 90° with limited torque"
            << std::endl;

  // System: J*theta_ddot = tau - b*theta_dot
  // J=0.5 kg⋅m², b=0.1 N⋅m⋅s
  double J = 0.5, b = 0.1;
  double dt = 0.05;

  // Continuous: A = [0 1; 0 -b/J], B = [0; 1/J]
  // Discrete:
  Matrix A = {{1, dt}, {0, 1 - b * dt / J}};
  Matrix B = {{0.5 * dt * dt / J}, {dt / J}};
  Matrix C = {{1, 0}};

  // MPC config
  Matrix Q = {{100, 0}, {0, 10}};
  Matrix R = {{1}};

  MPCConfig config(10, Q, R);
  config.u_min = {-5.0}; // Torque limit
  config.u_max = {5.0};
  config.max_iter = 200;

  MPCController mpc(A, B, C, config);

  // Target: 90 degrees = pi/2 rad
  double target_angle = M_PI / 2;

  // Start at 0
  Matrix x = {{0}, {0}};

  std::cout << "\nMPC Trajectory (target = 90°):" << std::endl;
  std::cout << "Time [s] | Angle [°] | Velocity | Torque [N⋅m]" << std::endl;
  std::cout << std::string(50, '-') << std::endl;

  for (int k = 0; k <= 60; ++k) {
    double t = k * dt;

    // Error state (for regulation)
    Matrix x_err = {{x(0, 0) - target_angle}, {x(1, 0)}};

    auto sol = mpc.solve(x_err);
    Matrix u = sol.getFirstControl(1);

    if (k % 5 == 0) {
      std::cout << std::fixed << std::setprecision(3);
      std::cout << std::setw(8) << t << " | " << std::setw(9)
                << x(0, 0) * 180 / M_PI << " | " << std::setw(8) << x(1, 0)
                << " | " << std::setw(12) << u(0, 0) << std::endl;
    }

    x = A * x + B * u;
  }

  std::cout << "\n✓ MPC respects torque constraints while tracking!"
            << std::endl;
  std::cout << "  (Note: Torque always stays within ±5 N⋅m)" << std::endl;
}

// ============================================================
//            DEMO 4: COMPARISON - LQR vs MPC
// ============================================================
void demo_lqr_vs_mpc() {
  printHeader("DEMO 4: Comparison - LQR vs MPC with Constraints");

  std::cout
      << "\nScenario: Same system, compare unconstrained LQR vs constrained MPC"
      << std::endl;

  double dt = 0.1;
  Matrix A = {{1, dt}, {0, 1}};
  Matrix B = {{0.5 * dt * dt}, {dt}};
  Matrix C = {{1, 0}};

  Matrix Q = Matrix::eye(2);
  Matrix R = {{0.1}};

  // DLQR (unconstrained)
  auto K = dlqr(A, B, Q, R);

  // MPC (constrained |u| ≤ 1)
  MPCConfig config(10, Q, R);
  config.u_min = {-1.0};
  config.u_max = {1.0};
  config.max_iter = 200;
  MPCController mpc(A, B, C, config);

  Matrix x0 = {{5}, {0}}; // Start at position 5

  std::cout << "\nFrom x0 = [5; 0]:" << std::endl;
  std::cout << "Step | LQR Pos | MPC Pos | LQR u  | MPC u (constrained)"
            << std::endl;
  std::cout << std::string(60, '-') << std::endl;

  Matrix x_lqr = x0, x_mpc = x0;

  for (int k = 0; k <= 20; ++k) {
    // LQR control
    double u_lqr = -(K(0, 0) * x_lqr(0, 0) + K(0, 1) * x_lqr(1, 0));

    // MPC control
    auto sol = mpc.solve(x_mpc);
    double u_mpc = sol.getFirstControl(1)(0, 0);

    if (k % 2 == 0) {
      std::cout << std::fixed << std::setprecision(3);
      std::cout << std::setw(4) << k << " | " << std::setw(7) << x_lqr(0, 0)
                << " | " << std::setw(7) << x_mpc(0, 0) << " | " << std::setw(6)
                << u_lqr << " | " << std::setw(6) << u_mpc << std::endl;
    }

    // Update
    Matrix u_lqr_mat = {{u_lqr}};
    Matrix u_mpc_mat = {{u_mpc}};
    x_lqr = A * x_lqr + B * u_lqr_mat;
    x_mpc = A * x_mpc + B * u_mpc_mat;
  }

  std::cout << "\n✓ MPC respects |u| ≤ 1 constraint (slower but safe)"
            << std::endl;
  std::cout << "  LQR uses large control initially (unconstrained)"
            << std::endl;
}

// ============================================================
//                         MAIN
// ============================================================
int main() {
  std::cout << "\n";
  std::cout << "╔══════════════════════════════════════════════════════════════"
               "═════════╗"
            << std::endl;
  std::cout << "║           CPPPLOT RESEARCH TOOLS - COMPREHENSIVE DEMO        "
               "         ║"
            << std::endl;
  std::cout << "║                                                              "
               "         ║"
            << std::endl;
  std::cout << "║   Demonstrating advanced control and estimation algorithms:  "
               "        ║"
            << std::endl;
  std::cout << "║   • Kalman Filter (state estimation)                         "
               "         ║"
            << std::endl;
  std::cout << "║   • LQG Controller (optimal control with noise)              "
               "         ║"
            << std::endl;
  std::cout << "║   • MPC Controller (predictive control with constraints)     "
               "         ║"
            << std::endl;
  std::cout << "╚══════════════════════════════════════════════════════════════"
               "═════════╝"
            << std::endl;

  demo_kalman_vehicle_tracking();
  demo_lqg_pendulum();
  demo_mpc_robot_arm();
  demo_lqr_vs_mpc();

  std::cout << "\n" << std::string(70, '=') << std::endl;
  std::cout << "  ALL DEMOS COMPLETED SUCCESSFULLY!" << std::endl;
  std::cout << std::string(70, '=') << std::endl;

  return 0;
}
