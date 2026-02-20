/**
 * @file control_comparison_plots.cpp
 * @brief Visualize and compare different control methods with plots
 *
 * This demo generates SVG plots comparing:
 * 1. Kalman Filter: True state vs Estimated state vs Measurements
 * 2. LQG Controller: Setpoint tracking with control input
 * 3. MPC Controller: Constrained vs Unconstrained control
 * 4. Comparison: LQR vs MPC vs PID responses
 *
 * Compile: g++ -std=c++14 -I "../include" control_comparison_plots.cpp -o
 * control_plots.exe
 */

#include <cmath>
#include <cppplot/control/control.hpp>
#include <cppplot/control/kalman.hpp>
#include <cppplot/control/lqg.hpp>
#include <cppplot/control/mpc.hpp>
#include <cppplot/cppplot.hpp>
#include <iostream>
#include <random>
#include <vector>


using namespace cppplot;
using namespace cppplot::control;

// ============================================================
//       PLOT 1: KALMAN FILTER STATE ESTIMATION
// ============================================================
void plot_kalman_estimation() {
  std::cout << "Generating Kalman Filter Plot..." << std::endl;

  // System: Position tracking with noisy GPS
  double dt = 0.1;
  Matrix A = {{1, dt}, {0, 1}};
  Matrix B = {{0.5 * dt * dt}, {dt}};
  Matrix C = {{1, 0}};

  Matrix Q = Matrix::eye(2) * 0.01;
  Matrix R = {{4.0}}; // GPS noise variance (sigma=2m)

  KalmanFilter kf(A, B, C, Q, R);
  kf.setInitialState(Matrix({{0}, {0}}), Matrix::eye(2) * 100);

  std::mt19937 gen(42);
  std::normal_distribution<> gps_noise(0, 2.0);
  std::normal_distribution<> proc_noise(0, 0.1);

  // Data vectors
  std::vector<double> time, true_pos, measured_pos, estimated_pos;
  std::vector<double> true_vel, estimated_vel;
  std::vector<double> estimation_error;

  double x_true = 0, v_true = 5.0; // True: pos=0, vel=5 m/s

  for (int k = 0; k <= 100; ++k) {
    double t = k * dt;
    time.push_back(t);

    // True state evolution
    x_true += v_true * dt + proc_noise(gen);
    true_pos.push_back(x_true);
    true_vel.push_back(v_true);

    // Noisy measurement
    double meas = x_true + gps_noise(gen);
    measured_pos.push_back(meas);

    // Kalman filter
    Matrix y = {{meas}};
    Matrix u = {{0}};
    auto est = kf.update(y, u);

    estimated_pos.push_back(est.x_hat(0, 0));
    estimated_vel.push_back(est.x_hat(1, 0));
    estimation_error.push_back(std::abs(x_true - est.x_hat(0, 0)));
  }

  // Create figure with subplots
  figure(1200, 800);
  layout(2, 2);

  // Subplot 1: Position comparison
  subplot(2, 2, 1);
  plot(time, true_pos, "b-", {{"label", "True Position"}});
  plot(time, measured_pos, "r.",
       {{"label", "GPS Measurement"}, {"alpha", "0.5"}});
  plot(time, estimated_pos, "g-",
       {{"label", "Kalman Estimate"}, {"linewidth", "2"}});
  xlabel("Time [s]");
  ylabel("Position [m]");
  title("Kalman Filter: Position Estimation");
  legend();
  grid(true);

  // Subplot 2: Velocity estimation
  subplot(2, 2, 2);
  plot(time, true_vel, "b-", {{"label", "True Velocity"}});
  plot(time, estimated_vel, "g--",
       {{"label", "Estimated Velocity"}, {"linewidth", "2"}});
  xlabel("Time [s]");
  ylabel("Velocity [m/s]");
  title("Kalman Filter: Velocity Estimation (Unobserved State)");
  legend();
  grid(true);

  // Subplot 3: Estimation error
  subplot(2, 2, 3);
  plot(time, estimation_error, "m-", {{"label", "Position Error"}});
  axhline(2.0,
          {{"color", "red"}, {"linestyle", "--"}, {"label", "1s GPS Noise"}});
  xlabel("Time [s]");
  ylabel("Error [m]");
  title("Estimation Error (Kalman vs GPS noise)");
  legend();
  grid(true);

  // Subplot 4: Measurement vs Estimate scatter
  subplot(2, 2, 4);
  scatter(true_pos, estimated_pos,
          {{"color", "blue"}, {"alpha", "0.6"}, {"label", "Estimate"}});
  // Perfect tracking line
  std::vector<double> line_x = {0, 60}, line_y = {0, 60};
  plot(line_x, line_y, "k--", {{"label", "Perfect"}});
  xlabel("True Position [m]");
  ylabel("Estimated Position [m]");
  title("True vs Estimated (Correlation)");
  legend();
  grid(true);

  savefig("kalman_estimation.svg");
  std::cout << "  Saved: kalman_estimation.svg" << std::endl;
}

// ============================================================
//       PLOT 2: LQG SETPOINT TRACKING
// ============================================================
void plot_lqg_tracking() {
  std::cout << "Generating LQG Tracking Plot..." << std::endl;

  // Second-order system with integrator
  double dt = 0.05;
  Matrix A = {{1, dt}, {0, 0.95}}; // Slightly damped
  Matrix B = {{0}, {dt}};
  Matrix C = {{1, 0}};

  Matrix Q_lqr = {{10, 0}, {0, 1}};
  Matrix R_lqr = {{0.1}};
  Matrix W = Matrix::eye(2) * 0.001;
  Matrix V = {{0.01}};

  LQGController controller(A, B, C, Q_lqr, R_lqr, W, V);
  controller.setInitialEstimate(Matrix({{0}, {0}}), Matrix::eye(2) * 1);

  std::mt19937 gen(42);
  std::normal_distribution<> noise(0, 0.1);

  // Data vectors
  std::vector<double> time, output, setpoint_vec, control_input, error;

  Matrix x = {{0}, {0}};
  Matrix u = {{0}};

  // Step changes in setpoint
  for (int k = 0; k <= 400; ++k) {
    double t = k * dt;
    time.push_back(t);

    // Setpoint: step from 0 to 5 at t=2, step to 2 at t=10, step to 8 at t=15
    double sp = 0;
    if (t >= 2)
      sp = 5;
    if (t >= 10)
      sp = 2;
    if (t >= 15)
      sp = 8;
    setpoint_vec.push_back(sp);

    // Measurement with noise
    double meas = x(0, 0) + noise(gen);
    Matrix y = {{meas}};

    // Add setpoint tracking (shift estimated state)
    Matrix y_shifted = {{meas - sp}};

    // LQG control
    u = controller.computeControl(y_shifted, u);

    output.push_back(x(0, 0));
    control_input.push_back(u(0, 0));
    error.push_back(sp - x(0, 0));

    // Update state
    x = A * x + B * u;
    x(0, 0) += noise(gen) * 0.01;
  }

  // Create figure
  figure(1200, 600);
  layout(2, 1);

  // Top: Output vs Setpoint
  subplot(2, 1, 1);
  plot(time, setpoint_vec, "r--", {{"label", "Setpoint"}, {"linewidth", "2"}});
  plot(time, output, "b-", {{"label", "Output"}, {"linewidth", "1.5"}});
  xlabel("Time [s]");
  ylabel("Position");
  title("LQG Controller: Setpoint Tracking");
  legend();
  grid(true);
  ylim(-1, 10);

  // Bottom: Control input
  subplot(2, 1, 2);
  plot(time, control_input, "g-",
       {{"label", "Control Input"}, {"linewidth", "1.5"}});
  axhline(0, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.3"}});
  xlabel("Time [s]");
  ylabel("Control Signal");
  title("LQG Control Input");
  legend();
  grid(true);

  savefig("lqg_tracking.svg");
  std::cout << "  Saved: lqg_tracking.svg" << std::endl;
}

// ============================================================
//       PLOT 3: MPC CONSTRAINED VS UNCONSTRAINED
// ============================================================
void plot_mpc_constraints() {
  std::cout << "Generating MPC Constraints Plot..." << std::endl;

  double dt = 0.1;
  Matrix A = {{1, dt}, {0, 1}};
  Matrix B = {{0.5 * dt * dt}, {dt}};
  Matrix C = {{1, 0}};

  Matrix Q = {{10, 0}, {0, 1}};
  Matrix R = {{0.1}};

  // Unconstrained MPC
  MPCConfig config_unc(15, Q, R);
  MPCController mpc_unc(A, B, C, config_unc);

  // Constrained MPC (|u| <= 2)
  MPCConfig config_con(15, Q, R);
  config_con.u_min = {-2.0};
  config_con.u_max = {2.0};
  config_con.max_iter = 200;
  MPCController mpc_con(A, B, C, config_con);

  // Data vectors
  std::vector<double> time;
  std::vector<double> pos_unc, pos_con;
  std::vector<double> vel_unc, vel_con;
  std::vector<double> u_unc, u_con;

  Matrix x_unc = {{10}, {0}}; // Start at position 10, target 0
  Matrix x_con = {{10}, {0}};

  for (int k = 0; k <= 80; ++k) {
    double t = k * dt;
    time.push_back(t);

    pos_unc.push_back(x_unc(0, 0));
    pos_con.push_back(x_con(0, 0));
    vel_unc.push_back(x_unc(1, 0));
    vel_con.push_back(x_con(1, 0));

    // Compute controls
    auto sol_unc = mpc_unc.solve(x_unc);
    auto sol_con = mpc_con.solve(x_con);

    double ctrl_unc = sol_unc.getFirstControl(1)(0, 0);
    double ctrl_con = sol_con.getFirstControl(1)(0, 0);

    u_unc.push_back(ctrl_unc);
    u_con.push_back(ctrl_con);

    // Update states
    Matrix u_mat_unc = {{ctrl_unc}};
    Matrix u_mat_con = {{ctrl_con}};
    x_unc = A * x_unc + B * u_mat_unc;
    x_con = A * x_con + B * u_mat_con;
  }

  // Create figure
  figure(1200, 800);
  layout(2, 2);

  // Position comparison
  subplot(2, 2, 1);
  plot(time, pos_unc, "b-", {{"label", "Unconstrained"}, {"linewidth", "2"}});
  plot(time, pos_con, "r-",
       {{"label", "Constrained |u|<=2"}, {"linewidth", "2"}});
  axhline(0, {{"color", "green"}, {"linestyle", "--"}, {"label", "Target"}});
  xlabel("Time [s]");
  ylabel("Position");
  title("MPC Position Response");
  legend();
  grid(true);

  // Velocity comparison
  subplot(2, 2, 2);
  plot(time, vel_unc, "b-", {{"label", "Unconstrained"}, {"linewidth", "2"}});
  plot(time, vel_con, "r-", {{"label", "Constrained"}, {"linewidth", "2"}});
  xlabel("Time [s]");
  ylabel("Velocity");
  title("MPC Velocity Response");
  legend();
  grid(true);

  // Control input comparison
  subplot(2, 2, 3);
  plot(time, u_unc, "b-", {{"label", "Unconstrained"}, {"linewidth", "2"}});
  plot(time, u_con, "r-", {{"label", "Constrained"}, {"linewidth", "2"}});
  axhline(2.0, {{"color", "gray"}, {"linestyle", "--"}, {"alpha", "0.7"}});
  axhline(-2.0, {{"color", "gray"}, {"linestyle", "--"}, {"alpha", "0.7"}});
  xlabel("Time [s]");
  ylabel("Control Input");
  title("MPC Control Signal (Gray = Constraints)");
  legend();
  grid(true);

  // Phase portrait (position vs velocity)
  subplot(2, 2, 4);
  plot(pos_unc, vel_unc, "b-",
       {{"label", "Unconstrained"}, {"linewidth", "1.5"}});
  plot(pos_con, vel_con, "r-",
       {{"label", "Constrained"}, {"linewidth", "1.5"}});
  scatter({10}, {0},
          {{"color", "green"}, {"markersize", "10"}, {"label", "Start"}});
  scatter({0}, {0},
          {{"color", "black"}, {"markersize", "10"}, {"label", "Target"}});
  xlabel("Position");
  ylabel("Velocity");
  title("Phase Portrait");
  legend();
  grid(true);

  savefig("mpc_constraints.svg");
  std::cout << "  Saved: mpc_constraints.svg" << std::endl;
}

// ============================================================
//       PLOT 4: LQR vs MPC vs PID COMPARISON
// ============================================================
void plot_controller_comparison() {
  std::cout << "Generating Controller Comparison Plot..." << std::endl;

  double dt = 0.05;
  Matrix A = {{1, dt}, {0, 0.98}}; // Slightly damped double integrator
  Matrix B = {{0.5 * dt * dt}, {dt}};
  Matrix C = {{1, 0}};

  Matrix Q = {{10, 0}, {0, 1}};
  Matrix R = {{1}};

  // LQR (DLQR)
  auto K_lqr = dlqr(A, B, Q, R);

  // MPC (constrained)
  MPCConfig config(10, Q, R);
  config.u_min = {-5.0};
  config.u_max = {5.0};
  config.max_iter = 100;
  MPCController mpc(A, B, C, config);

  // PID parameters (tuned for this system)
  double Kp = 3.0, Ki = 0.5, Kd = 1.5;

  // Data vectors
  std::vector<double> time, setpoint_vec;
  std::vector<double> y_lqr, y_mpc, y_pid;
  std::vector<double> u_lqr_vec, u_mpc_vec, u_pid_vec;

  Matrix x_lqr = {{0}, {0}};
  Matrix x_mpc = {{0}, {0}};
  Matrix x_pid = {{0}, {0}};

  double integral = 0, prev_error = 0;
  double sp = 0;

  for (int k = 0; k <= 400; ++k) {
    double t = k * dt;
    time.push_back(t);

    // Setpoint: step to 5 at t=1
    sp = (t >= 1.0) ? 5.0 : 0.0;
    setpoint_vec.push_back(sp);

    // Current outputs
    y_lqr.push_back(x_lqr(0, 0));
    y_mpc.push_back(x_mpc(0, 0));
    y_pid.push_back(x_pid(0, 0));

    // LQR control (with setpoint tracking)
    double ctrl_lqr =
        K_lqr(0, 0) * (sp - x_lqr(0, 0)) - K_lqr(0, 1) * x_lqr(1, 0);

    // MPC control (error state)
    Matrix x_err = {{x_mpc(0, 0) - sp}, {x_mpc(1, 0)}};
    auto sol = mpc.solve(x_err);
    double ctrl_mpc = sol.getFirstControl(1)(0, 0);

    // PID control
    double error = sp - x_pid(0, 0);
    integral += error * dt;
    double derivative = (error - prev_error) / dt;
    double ctrl_pid = Kp * error + Ki * integral + Kd * derivative;
    ctrl_pid = std::max(-5.0, std::min(5.0, ctrl_pid)); // Saturate
    prev_error = error;

    u_lqr_vec.push_back(ctrl_lqr);
    u_mpc_vec.push_back(ctrl_mpc);
    u_pid_vec.push_back(ctrl_pid);

    // Update states
    Matrix u_lqr_mat = {{ctrl_lqr}};
    Matrix u_mpc_mat = {{ctrl_mpc}};
    Matrix u_pid_mat = {{ctrl_pid}};

    x_lqr = A * x_lqr + B * u_lqr_mat;
    x_mpc = A * x_mpc + B * u_mpc_mat;
    x_pid = A * x_pid + B * u_pid_mat;
  }

  // Calculate performance metrics
  double ts_lqr = 0, ts_mpc = 0, ts_pid = 0;
  double os_lqr = 0, os_mpc = 0, os_pid = 0;
  double target = 5.0;

  for (size_t i = 0; i < time.size(); ++i) {
    if (time[i] >= 1.0) {
      os_lqr = std::max(os_lqr, (y_lqr[i] - target) / target * 100);
      os_mpc = std::max(os_mpc, (y_mpc[i] - target) / target * 100);
      os_pid = std::max(os_pid, (y_pid[i] - target) / target * 100);

      if (std::abs(y_lqr[i] - target) > 0.02 * target)
        ts_lqr = time[i] - 1.0;
      if (std::abs(y_mpc[i] - target) > 0.02 * target)
        ts_mpc = time[i] - 1.0;
      if (std::abs(y_pid[i] - target) > 0.02 * target)
        ts_pid = time[i] - 1.0;
    }
  }

  // Create figure
  figure(1400, 900);
  layout(2, 2);

  // Output comparison
  subplot(2, 2, 1);
  plot(time, setpoint_vec, "k--", {{"label", "Setpoint"}, {"linewidth", "2"}});
  plot(time, y_lqr, "b-", {{"label", "LQR"}, {"linewidth", "1.5"}});
  plot(time, y_mpc, "r-", {{"label", "MPC"}, {"linewidth", "1.5"}});
  plot(time, y_pid, "g-", {{"label", "PID"}, {"linewidth", "1.5"}});
  xlabel("Time [s]");
  ylabel("Output");
  title("Step Response Comparison: LQR vs MPC vs PID");
  legend();
  grid(true);
  xlim(0, 10);
  ylim(-0.5, 7);

  // Control input comparison
  subplot(2, 2, 2);
  plot(time, u_lqr_vec, "b-", {{"label", "LQR"}, {"linewidth", "1.5"}});
  plot(time, u_mpc_vec, "r-", {{"label", "MPC"}, {"linewidth", "1.5"}});
  plot(time, u_pid_vec, "g-", {{"label", "PID"}, {"linewidth", "1.5"}});
  axhline(5.0, {{"color", "gray"}, {"linestyle", "--"}, {"alpha", "0.5"}});
  axhline(-5.0, {{"color", "gray"}, {"linestyle", "--"}, {"alpha", "0.5"}});
  xlabel("Time [s]");
  ylabel("Control Input");
  title("Control Effort Comparison");
  legend();
  grid(true);
  xlim(0, 10);

  // Zoom in on transient
  subplot(2, 2, 3);
  plot(time, setpoint_vec, "k--", {{"label", "Setpoint"}, {"linewidth", "2"}});
  plot(time, y_lqr, "b-", {{"label", "LQR"}, {"linewidth", "1.5"}});
  plot(time, y_mpc, "r-", {{"label", "MPC"}, {"linewidth", "1.5"}});
  plot(time, y_pid, "g-", {{"label", "PID"}, {"linewidth", "1.5"}});
  // Add settling band
  axhline(5.0 * 1.02,
          {{"color", "orange"}, {"linestyle", ":"}, {"alpha", "0.7"}});
  axhline(5.0 * 0.98,
          {{"color", "orange"}, {"linestyle", ":"}, {"alpha", "0.7"}});
  xlabel("Time [s]");
  ylabel("Output");
  title("Transient Response (Zoom) with 2% Settling Band");
  legend();
  grid(true);
  xlim(0.8, 4);
  ylim(3, 6);

  // Performance bar chart
  subplot(2, 2, 4);
  std::vector<std::string> methods = {"LQR", "MPC", "PID"};
  std::vector<double> settling_times = {ts_lqr, ts_mpc, ts_pid};
  bar(methods, settling_times, {{"color", "steelblue"}});
  xlabel("Controller");
  ylabel("Settling Time [s]");
  title("Performance: Settling Time (2%)");
  grid(true);

  savefig("controller_comparison.svg");
  std::cout << "  Saved: controller_comparison.svg" << std::endl;

  // Print metrics table
  std::cout << "\n  Performance Metrics:" << std::endl;
  std::cout << "  " << std::string(45, '-') << std::endl;
  std::cout << "  Controller | Settling Time | Overshoot" << std::endl;
  std::cout << "  " << std::string(45, '-') << std::endl;
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "  LQR        | " << std::setw(10) << ts_lqr << " s | "
            << std::setw(7) << os_lqr << " %" << std::endl;
  std::cout << "  MPC        | " << std::setw(10) << ts_mpc << " s | "
            << std::setw(7) << os_mpc << " %" << std::endl;
  std::cout << "  PID        | " << std::setw(10) << ts_pid << " s | "
            << std::setw(7) << os_pid << " %" << std::endl;
  std::cout << "  " << std::string(45, '-') << std::endl;
}

// ============================================================
//       PLOT 5: ADAPTIVE KALMAN FILTER COMPARISON
// ============================================================
void plot_adaptive_kalman() {
  std::cout << "Generating Adaptive Kalman Plot..." << std::endl;

  double dt = 0.1;
  Matrix A = {{1, dt}, {0, 1}};
  Matrix B = {{0.5 * dt * dt}, {dt}};
  Matrix C = {{1, 0}};

  // True noise
  double sigma_r_true = 2.0; // True measurement noise
  Matrix Q_true = Matrix::eye(2) * 0.01;
  Matrix R_true = {{sigma_r_true * sigma_r_true}};

  // Wrong initial guess (5x too large)
  Matrix Q_wrong = Matrix::eye(2) * 0.01;
  Matrix R_wrong = {{25.0}}; // Wrong: 5 instead of 2

  // Create filters
  KalmanFilter kf_correct(A, B, C, Q_true, R_true);
  KalmanFilter kf_wrong(A, B, C, Q_wrong, R_wrong);
  AdaptiveKalmanFilter akf(A, B, C, Q_wrong, R_wrong,
                           AdaptiveKalmanFilter::Method::COVARIANCE_MATCHING);

  Matrix x0 = {{0}, {0}};
  Matrix P0 = Matrix::eye(2) * 100;
  kf_correct.setInitialState(x0, P0);
  kf_wrong.setInitialState(x0, P0);
  akf.setInitialState(x0, P0);

  std::mt19937 gen(42);
  std::normal_distribution<> proc_noise(0, 0.1);
  std::normal_distribution<> meas_noise(0, sigma_r_true);

  // Data vectors
  std::vector<double> time, true_pos, meas_pos;
  std::vector<double> est_correct, est_wrong, est_adaptive;
  std::vector<double> R_adapt_history;

  double x_true = 0, v_true = 5.0;

  for (int k = 0; k <= 150; ++k) {
    double t = k * dt;
    time.push_back(t);

    x_true += v_true * dt + proc_noise(gen);
    true_pos.push_back(x_true);

    double meas = x_true + meas_noise(gen);
    meas_pos.push_back(meas);

    Matrix y = {{meas}};
    Matrix u = {{0}};

    auto e_correct = kf_correct.update(y, u);
    auto e_wrong = kf_wrong.update(y, u);
    auto e_adaptive = akf.update(y, u);

    est_correct.push_back(e_correct.x_hat(0, 0));
    est_wrong.push_back(e_wrong.x_hat(0, 0));
    est_adaptive.push_back(e_adaptive.x_hat(0, 0));

    Matrix R_adapt = akf.getR();
    R_adapt_history.push_back(std::sqrt(R_adapt(0, 0)));
  }

  // Create figure
  figure(1200, 800);
  layout(2, 2);

  // Position comparison
  subplot(2, 2, 1);
  plot(time, true_pos, "k-", {{"label", "True"}, {"linewidth", "2"}});
  plot(time, meas_pos, ".",
       {{"color", "gray"}, {"alpha", "0.3"}, {"label", "Measurement"}});
  plot(time, est_correct, "b-",
       {{"label", "KF (correct R)"}, {"linewidth", "1.5"}});
  plot(time, est_wrong, "r-",
       {{"label", "KF (wrong R)"}, {"linewidth", "1.5"}});
  plot(time, est_adaptive, "g-",
       {{"label", "Adaptive KF"}, {"linewidth", "1.5"}});
  xlabel("Time [s]");
  ylabel("Position [m]");
  title("State Estimation: Fixed vs Adaptive KF");
  legend();
  grid(true);

  // R adaptation
  subplot(2, 2, 2);
  plot(time, R_adapt_history, "g-",
       {{"label", "Adapted sigma_R"}, {"linewidth", "2"}});
  axhline(
      sigma_r_true,
      {{"color", "blue"}, {"linestyle", "--"}, {"label", "True sigma_R = 2"}});
  axhline(5.0, {{"color", "red"},
                {"linestyle", "--"},
                {"label", "Initial sigma_R = 5"}});
  xlabel("Time [s]");
  ylabel("sigma_R (std dev)");
  title("Adaptive KF: R Covariance Learning");
  legend();
  grid(true);
  ylim(0, 6);

  // Error comparison
  std::vector<double> err_correct, err_wrong, err_adaptive;
  for (size_t i = 0; i < time.size(); ++i) {
    err_correct.push_back(std::abs(true_pos[i] - est_correct[i]));
    err_wrong.push_back(std::abs(true_pos[i] - est_wrong[i]));
    err_adaptive.push_back(std::abs(true_pos[i] - est_adaptive[i]));
  }

  subplot(2, 2, 3);
  plot(time, err_correct, "b-",
       {{"label", "KF (correct R)"}, {"alpha", "0.8"}});
  plot(time, err_wrong, "r-", {{"label", "KF (wrong R)"}, {"alpha", "0.8"}});
  plot(time, err_adaptive, "g-", {{"label", "Adaptive KF"}, {"alpha", "0.8"}});
  xlabel("Time [s]");
  ylabel("Absolute Error [m]");
  title("Estimation Error Comparison");
  legend();
  grid(true);

  // RMSE over time (cumulative)
  std::vector<double> rmse_correct, rmse_wrong, rmse_adaptive;
  double sum_correct = 0, sum_wrong = 0, sum_adaptive = 0;
  for (size_t i = 0; i < time.size(); ++i) {
    sum_correct += err_correct[i] * err_correct[i];
    sum_wrong += err_wrong[i] * err_wrong[i];
    sum_adaptive += err_adaptive[i] * err_adaptive[i];
    rmse_correct.push_back(std::sqrt(sum_correct / (i + 1)));
    rmse_wrong.push_back(std::sqrt(sum_wrong / (i + 1)));
    rmse_adaptive.push_back(std::sqrt(sum_adaptive / (i + 1)));
  }

  subplot(2, 2, 4);
  plot(time, rmse_correct, "b-",
       {{"label", "KF (correct R)"}, {"linewidth", "2"}});
  plot(time, rmse_wrong, "r-", {{"label", "KF (wrong R)"}, {"linewidth", "2"}});
  plot(time, rmse_adaptive, "g-",
       {{"label", "Adaptive KF"}, {"linewidth", "2"}});
  xlabel("Time [s]");
  ylabel("RMSE [m]");
  title("Cumulative RMSE");
  legend();
  grid(true);

  savefig("adaptive_kalman.svg");
  std::cout << "  Saved: adaptive_kalman.svg" << std::endl;
}

// ============================================================
//                         MAIN
// ============================================================
int main() {
  std::cout << "\n";
  std::cout << "==============================================================="
               "========="
            << std::endl;
  std::cout << "          CONTROL SYSTEMS VISUALIZATION - CppPlot              "
               "        "
            << std::endl;
  std::cout << "                                                               "
               "        "
            << std::endl;
  std::cout << "   Generating comparison plots for different control methods   "
               "        "
            << std::endl;
  std::cout << "==============================================================="
               "========="
            << std::endl;
  std::cout << "\n";

  plot_kalman_estimation();
  plot_lqg_tracking();
  plot_mpc_constraints();
  plot_controller_comparison();
  plot_adaptive_kalman();

  std::cout << "\n" << std::string(70, '=') << std::endl;
  std::cout << "  ALL PLOTS GENERATED SUCCESSFULLY!" << std::endl;
  std::cout << "\n  Generated files:" << std::endl;
  std::cout
      << "    1. kalman_estimation.svg    - Kalman Filter state estimation"
      << std::endl;
  std::cout << "    2. lqg_tracking.svg         - LQG setpoint tracking"
            << std::endl;
  std::cout
      << "    3. mpc_constraints.svg      - MPC constrained vs unconstrained"
      << std::endl;
  std::cout << "    4. controller_comparison.svg - LQR vs MPC vs PID"
            << std::endl;
  std::cout
      << "    5. adaptive_kalman.svg      - Adaptive vs Fixed Kalman Filter"
      << std::endl;
  std::cout << std::string(70, '=') << std::endl;

  return 0;
}
