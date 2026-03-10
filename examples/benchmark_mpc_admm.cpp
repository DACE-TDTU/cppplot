#include <chrono>
#include <cppplot/control/mpc.hpp>
#include <cppplot/control/state_space.hpp>
#include <iostream>


using namespace cppplot;
using namespace cppplot::control;

int main() {
  // Simple 2D double integrator
  Matrix A = {{1.0, 0.1}, {0.0, 1.0}};
  Matrix B = {{0.005}, {0.1}};
  Matrix C = {{1.0, 0.0}};

  size_t N = 20; // Horizon

  // Setup Config
  MPCConfig config;
  config.horizon = N;
  config.Q = Matrix::eye(2) * 10;
  config.R = Matrix::eye(1) * 1;
  config.u_min = {-1.0};
  config.u_max = {1.0};

  // State
  Matrix x0 = {{5.0}, {0.0}};

  // 1. PGD Solver
  config.solver_type = MPCSolver::PGD;
  config.max_iter = 2000;
  MPCController mpc_pgd(A, B, C, config);

  auto start = std::chrono::high_resolution_clock::now();
  MPCSolution sol_pgd = mpc_pgd.solve(x0);
  auto end = std::chrono::high_resolution_clock::now();
  double t_pgd = std::chrono::duration<double, std::milli>(end - start).count();

  // 2. ADMM Solver
  config.solver_type = MPCSolver::ADMM;
  config.rho = 1.0;
  config.admm_max_iter = 2000;
  MPCController mpc_admm(A, B, C, config);

  start = std::chrono::high_resolution_clock::now();
  MPCSolution sol_admm = mpc_admm.solve(x0);
  end = std::chrono::high_resolution_clock::now();
  double t_admm =
      std::chrono::duration<double, std::milli>(end - start).count();

  // Report
  std::cout << "--- MPC Solver Test ---\n";
  std::cout << "PGD : Cost = " << sol_pgd.cost
            << ", Iters = " << sol_pgd.iterations << ", Time = " << t_pgd
            << " ms (u0 = " << sol_pgd.u_opt(0, 0) << ")\n";
  std::cout << "ADMM: Cost = " << sol_admm.cost
            << ", Iters = " << sol_admm.iterations << ", Time = " << t_admm
            << " ms (u0 = " << sol_admm.u_opt(0, 0) << ")\n";

  // Calculate difference
  double diff = (sol_pgd.u_opt - sol_admm.u_opt).norm();
  std::cout << "L2 Difference between U_opt vectors: " << diff << "\n";

  return 0;
}
