/**
 * @file demo_qp_mpc.cpp
 * @brief Simulation & Benchmark Demo — ADMM QP Solver + MPC
 *
 * Demonstrates the new qp_solver.hpp integrated into MPC for AGV control.
 *
 * Experiments:
 *   EXP 1 — ADMM Solver Correctness vs Projected Gradient Descent
 *   EXP 2 — Convergence profile (residuals vs iterations)
 *   EXP 3 — AGV double integrator closed-loop with constraints
 *   EXP 4 — WCET benchmark (1000 solves, timing histogram)
 *   EXP 5 — Warm-start benefit (cold vs warm iterations)
 *
 * Outputs (CSV for plotting):
 *   qp_exp1_comparison.csv
 *   qp_exp2_convergence.csv
 *   qp_exp3_trajectory.csv
 *   qp_exp4_timing.csv
 *   qp_exp5_warmstart.csv
 *
 * Compile:
 *   g++ -std=c++14 -O2 -I"../include" demo_qp_mpc.cpp -o demo_qp_mpc
 *   g++ -std=c++14 -O3 -I"../include" demo_qp_mpc.cpp -o demo_qp_mpc  # benchmark
 *
 * Run:
 *   ./demo_qp_mpc
 *   python3 plot_qp_results.py   # generate all plots
 */

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

// ── cppplot / DCAS Lab headers ──────────────────────────────
#include <cppplot/control/control.hpp>   // Matrix, dlqr, dare
#include <cppplot/control/mpc.hpp>       // MPCController, MPCConfig
#include <cppplot/control/qp_solver.hpp> // ADMMSolver, Vec, Mat

using namespace cppplot::control;
using namespace cppplot::control::qp;
using Clock     = std::chrono::high_resolution_clock;
using Microsecs = std::chrono::microseconds;

// ============================================================
//              STANDALONE ADMM (no cppplot Matrix dep)
// ============================================================
// For benchmark purity we also implement a self-contained version
// that doesn't go through Matrix bridging.

/**
 * @brief Projected Gradient Descent solver (current baseline in mpc.hpp)
 *
 * Replicated here for fair comparison without modifying mpc.hpp.
 */
template <int N>
struct PGDSolver {
  struct Result { Vec<N> x; int iters; bool converged; };

  Result solve(const Mat<N>& H, const Vec<N>& g,
               const Vec<N>& lb, const Vec<N>& ub,
               int max_iter = 500, double tol = 1e-8) {
    Result r;
    Vec<N> U;  // zero-init
    double alpha = 0.01;

    for (int k = 0; k < max_iter; ++k) {
      // grad = H*U + g
      Vec<N> grad = H.matvec(U) + g;
      double gnorm = grad.norm2();
      if (gnorm < tol) { r.converged = true; r.iters = k; break; }

      // Update + project
      Vec<N> U_new;
      for (int i = 0; i < N; ++i)
        U_new[i] = std::max(lb[i], std::min(ub[i], U[i] - alpha * grad[i]));

      // Barzilai-Borwein step size
      Vec<N> dU = U_new - U;
      double dU_norm2 = dU.dot(dU);
      if (dU_norm2 > 1e-12) {
        Vec<N> HdU = H.matvec(dU);
        double dUHdU = dU.dot(HdU);
        if (std::abs(dUHdU) > 1e-12)
          alpha = std::max(0.001, std::min(1.0, dU_norm2 / dUHdU));
      }
      U = U_new;
      r.iters = k + 1;
    }
    r.x = U;
    return r;
  }
};

// ============================================================
//              HELPER — CSV writer
// ============================================================
class CSV {
 public:
  explicit CSV(const std::string& fname) : f_(fname) {
    std::cout << "  Writing " << fname << " ..." << std::endl;
  }
  template <typename... Args>
  void row(Args... args) {
    bool first = true;
    auto write = [&](auto v) {
      if (!first) f_ << ",";
      f_ << v;
      first = false;
    };
    (void)std::initializer_list<int>{(write(args), 0)...};
    f_ << "\n";
  }
  void header(const std::string& h) { f_ << h << "\n"; }
 private:
  std::ofstream f_;
};

// ============================================================
//              SYSTEM MODEL — AGV Double Integrator
// ============================================================
/**
 * Discrete-time double integrator  (dt = 0.01s)
 *   x = [position, velocity]'
 *   u = acceleration
 *
 *   x_{k+1} = A*x_k + B*u_k
 *   A = [1  dt; 0  1]
 *   B = [0.5*dt^2; dt]
 */
struct AGVModel {
  static constexpr int NX = 2;   // state dimension
  static constexpr int NU = 1;   // input dimension
  static constexpr double DT = 0.01;

  Mat<2> A, B_2x1;  // just store as arrays
  double A_raw[2][2], B_raw[2][1];

  // cppplot Matrix versions
  Matrix Amat, Bmat, Cmat;

  AGVModel() {
    double dt = DT;
    Amat = Matrix(2, 2);
    Bmat = Matrix(2, 1);
    Cmat = Matrix::eye(2);

    Amat(0,0)=1; Amat(0,1)=dt;
    Amat(1,0)=0; Amat(1,1)=1;
    Bmat(0,0)=0.5*dt*dt;
    Bmat(1,0)=dt;
  }

  // step: returns x_{k+1}
  Matrix step(const Matrix& x, const Matrix& u) {
    return Amat * x + Bmat * u;
  }
};

// ============================================================
//              EXP 1 — Correctness: ADMM vs PGD
// ============================================================
void exp1_correctness() {
  std::cout << "\n" << std::string(60,'=') << std::endl;
  std::cout << "EXP 1 — ADMM Correctness vs Projected Gradient Descent"
            << std::endl;
  std::cout << std::string(60,'=') << std::endl;

  // Double integrator MPC: N=10, m=1 → QP dim = 10
  constexpr int QP_N = 10;

  AGVModel model;

  // MPC weights
  Matrix Q = Matrix::eye(2);
  Matrix R = Matrix(1,1); R(0,0) = 0.1;

  // ── ADMM controller (new solver) ─────────────────────────────
  MPCConfig cfg(QP_N, Q, R);
  cfg.u_min = {-2.0};
  cfg.u_max = {2.0};
  MPCController mpc_admm(model.Amat, model.Bmat, model.Cmat, cfg);

  // ── PGD baseline — uses PGDSolver<N> defined in this file ────
  // Build the MPC prediction matrices manually so we can call
  // PGDSolver directly with the same H and g as the ADMM solver.
  // We need H (10x10) and F (10x2) from the MPC formulation.
  //
  // Strategy: extract H and g indirectly.
  //   Step 1: run ADMM on x0=[1,0] (unconstrained, converges exactly)
  //   Step 2: unconstrained solution = -H^{-1}*g, so g = -H*U*
  //   This is too involved without access to private members.
  //
  // Practical approach: use ADMM as reference (ground truth, tight tol)
  // and PGD as comparison at looser tolerance. Both solve the same QP
  // through MPCController — but PGD must be the STANDALONE PGDSolver,
  // NOT another MPCController instance (which would also use ADMM).
  //
  // To drive PGDSolver<N> directly we need H and g exposed from MPC.
  // Since H/F are private in MPCController, we reconstruct them here
  // analytically for the double integrator (same formula as MPCController).

  // ── Reconstruct H analytically ───────────────────────────────
  // Phi  = [A; A^2; ...; A^N]         (nN x n)
  // Gamma= lower-triangular Toeplitz  (nN x mN)
  // Qbar = blkdiag(Q,...,Q,Pf)        (nN x nN)
  // H    = Gamma'*Qbar*Gamma + Rbar   (mN x mN)
  // g    = (Gamma'*Qbar*Phi)' * x0    (mN)

  const int n = 2, m = 1, N = QP_N;
  double dt = AGVModel::DT;

  // A and B as plain arrays for manual computation
  double A[2][2] = {{1,dt},{0,1}};
  double B[2][1] = {{0.5*dt*dt},{dt}};

  // Build Phi (nN x n) and Gamma (nN x mN) via matrix powers
  // Using Mat<20> and Mat<10> (nN=20, mN=10)
  Mat<20> Phi_mat;     // 20 rows x n=2 → store as 20x20 (use first 2 cols)
  Mat<20> Gamma_mat;   // 20 rows x mN=10 → store as 20x20 (use first 10 cols)

  // A^k arrays
  double Ak[2][2];  // current A^k
  // init A^1
  for(int r=0;r<2;r++) for(int c=0;c<2;c++) Ak[r][c]=A[r][c];

  for(int k=0;k<N;k++){
    // Phi rows 2k, 2k+1 = A^{k+1}
    for(int r=0;r<2;r++) for(int c=0;c<2;c++) Phi_mat(2*k+r, c) = Ak[r][c];

    // Gamma: col j (j=0..k), rows 2k, 2k+1 = A^{k-j} * B
    // A^0 = I, A^1, ... precompute A^{k-j}
    double Apow[2][2] = {{1,0},{0,1}};  // A^0 = I
    for(int j=k;j>=0;j--){
      // Apow = A^{k-j}
      // Gamma(2k:2k+1, j) = Apow * B
      for(int r=0;r<2;r++){
        double val = 0;
        for(int c=0;c<2;c++) val += Apow[r][c]*B[c][0];
        Gamma_mat(2*k+r, j) = val;
      }
      // advance Apow by one more A
      if(j>0){
        double tmp[2][2]={{0,0},{0,0}};
        for(int r=0;r<2;r++) for(int cc=0;cc<2;cc++)
          for(int kk=0;kk<2;kk++) tmp[r][cc]+=A[r][kk]*Apow[kk][cc];
        for(int r=0;r<2;r++) for(int cc=0;cc<2;cc++) Apow[r][cc]=tmp[r][cc];
      }
    }
    // Advance Ak = A^{k+1}
    if(k<N-1){
      double tmp[2][2]={{0,0},{0,0}};
      for(int r=0;r<2;r++) for(int c=0;c<2;c++)
        for(int kk=0;kk<2;kk++) tmp[r][c]+=A[r][kk]*Ak[kk][c];
      for(int r=0;r<2;r++) for(int c=0;c<2;c++) Ak[r][c]=tmp[r][c];
    }
  }

  // Qbar: blkdiag(Q,...,Q) for nN x nN (Pf=Q for simplicity here)
  // Q = I_2, so Qbar = I_20
  double Qbar[20][20]={};
  for(int i=0;i<20;i++) Qbar[i][i]=1.0;
  // Rbar = 0.1 * I_10
  double Rbar_diag = 0.1;

  // H = Gamma'*Qbar*Gamma + Rbar  (10x10)
  Mat<QP_N> H_real;
  for(int i=0;i<N;i++) for(int j=0;j<N;j++){
    double s=0;
    for(int r=0;r<2*N;r++) s += Gamma_mat(r,i)*Gamma_mat(r,j); // Qbar=I
    H_real(i,j)=s + (i==j ? Rbar_diag : 0.0);
  }

  // F' = Gamma'*Qbar*Phi  (mN x n) → F = Phi'*Qbar*Gamma  (n x mN)
  double Ft[10][2]={};  // F'[i][c] = Gamma'*Phi column c row i
  for(int i=0;i<N;i++) for(int c=0;c<2;c++){
    double s=0;
    for(int r=0;r<2*N;r++) s += Gamma_mat(r,i)*Phi_mat(r,c); // Qbar=I
    Ft[i][c]=s;
  }

  // Bounds
  Vec<QP_N> lb, ub;
  for(int i=0;i<N;i++){ lb[i]=-2.0; ub[i]=2.0; }

  // ── PGD solver (standalone, correct) ─────────────────────────
  PGDSolver<QP_N> pgd;

  // ── ADMM solver (standalone, same H) ─────────────────────────
  Params admm_params;
  admm_params.rho       = 1.0;
  admm_params.max_iter  = 200;
  admm_params.eps_abs   = 1e-6;
  admm_params.eps_rel   = 1e-5;
  admm_params.warm_start = false;  // cold start for fair comparison
  ADMMSolver<QP_N> admm_solo;
  admm_solo.setup(H_real, admm_params);

  // Reference: MPCController (ADMM) — cross-check
  CSV csv("qp_exp1_comparison.csv");
  csv.header("test_id,x0_pos,x0_vel,u_admm,u_pgd,u_mpc,abs_diff_admm_pgd,admm_iters,pgd_iters,admm_us,pgd_us");

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "\n  x0_pos  | u_ADMM    | u_PGD     | u_MPC     | diff(A-P) | A_it | P_it\n";
  std::cout << "  " << std::string(70,'-') << std::endl;

  double test_positions[] = {-5,-3,-2,-1,-0.5,0.1,0.5,1,2,3,
                               4,  5, 0.3,-0.3, 1.5,-1.5,0.01,4.9,-4.9,2.7};

  for (int idx = 0; idx < 20; ++idx) {
    double pos = test_positions[idx];
    Matrix x0 = Matrix(2,1);
    x0(0,0) = pos; x0(1,0) = 0.0;

    // g = F' * x0  (mN vector)
    Vec<QP_N> g_vec;
    for(int i=0;i<N;i++) g_vec[i] = Ft[i][0]*pos + Ft[i][1]*0.0;

    // ADMM standalone solve
    admm_solo.reset();  // cold start every time
    auto ta0 = Clock::now();
    auto sol_a = admm_solo.solve(g_vec, lb, ub);
    auto ta1 = Clock::now();
    double u_admm = sol_a.x[0];
    double admm_us = std::chrono::duration_cast<Microsecs>(ta1-ta0).count();

    // PGD standalone solve
    auto tp0 = Clock::now();
    auto sol_p = pgd.solve(H_real, g_vec, lb, ub, 1000, 1e-8);
    auto tp1 = Clock::now();
    double u_pgd = sol_p.x[0];
    double pgd_us = std::chrono::duration_cast<Microsecs>(tp1-tp0).count();

    // MPCController solve (ADMM internally, cross-check)
    auto sol_mpc = mpc_admm.solve(x0);
    double u_mpc = sol_mpc.getFirstControl(1)(0,0);

    double diff = std::abs(u_admm - u_pgd);

    std::cout << "  " << std::setw(7) << pos
              << " | " << std::setw(9) << u_admm
              << " | " << std::setw(9) << u_pgd
              << " | " << std::setw(9) << u_mpc
              << " | " << std::setw(9) << diff
              << " | " << std::setw(4) << sol_a.iterations
              << " | " << std::setw(4) << sol_p.iters
              << std::endl;

    csv.row(idx, pos, 0.0,
            u_admm, u_pgd, u_mpc,
            diff,
            sol_a.iterations, sol_p.iters,
            admm_us, pgd_us);
  }

  std::cout << "\n  ✓ EXP 1 DONE — CSV written" << std::endl;
}

// ============================================================
//              EXP 2 — Convergence Profile
// ============================================================
void exp2_convergence() {
  std::cout << "\n" << std::string(60,'=') << std::endl;
  std::cout << "EXP 2 — ADMM Convergence Profile (residuals vs iterations)"
            << std::endl;
  std::cout << std::string(60,'=') << std::endl;

  constexpr int QP_N = 10;
  const double dt = AGVModel::DT;

  // ── Build REAL H from AGV double integrator MPC ──────────────
  // (same analytical reconstruction as EXP1)
  double A[2][2] = {{1,dt},{0,1}};
  double B[2][1] = {{0.5*dt*dt},{dt}};
  const int N = QP_N;

  // Gamma (20 x 10): lower-triangular Toeplitz
  double Gamma[20][10] = {};
  double Ak[2][2] = {{1,0},{0,1}};  // A^0 = I
  // Row block k (rows 2k, 2k+1), col j = A^{k-j} * B
  // Build column by column using the shift structure
  for(int k=0;k<N;k++){
    double Apow[2][2] = {{1,0},{0,1}};
    for(int j=k;j>=0;j--){
      for(int r=0;r<2;r++){
        double v=0; for(int c=0;c<2;c++) v+=Apow[r][c]*B[c][0];
        Gamma[2*k+r][j]=v;
      }
      if(j>0){
        double tmp[2][2]={};
        for(int r=0;r<2;r++) for(int c=0;c<2;c++)
          for(int kk=0;kk<2;kk++) tmp[r][c]+=A[r][kk]*Apow[kk][c];
        for(int r=0;r<2;r++) for(int c=0;c<2;c++) Apow[r][c]=tmp[r][c];
      }
    }
  }

  // Phi (20 x 2): A^1 ... A^N
  double Phi[20][2] = {};
  double Ak2[2][2] = {{1,dt},{0,1}};  // A^1
  for(int k=0;k<N;k++){
    for(int r=0;r<2;r++) for(int c=0;c<2;c++) Phi[2*k+r][c]=Ak2[r][c];
    if(k<N-1){
      double tmp[2][2]={};
      for(int r=0;r<2;r++) for(int c=0;c<2;c++)
        for(int kk=0;kk<2;kk++) tmp[r][c]+=A[r][kk]*Ak2[kk][c];
      for(int r=0;r<2;r++) for(int c=0;c<2;c++) Ak2[r][c]=tmp[r][c];
    }
  }

  // H = Gamma'*Qbar*Gamma + Rbar  (Qbar=I_20, Rbar=0.1*I_10)
  Mat<QP_N> H_real;
  for(int i=0;i<N;i++) for(int j=0;j<N;j++){
    double s=0;
    for(int r=0;r<2*N;r++) s+=Gamma[r][i]*Gamma[r][j];
    H_real(i,j)=s+(i==j?0.1:0.0);
  }

  // F' = Gamma'*Qbar*Phi  (N x 2)
  double Ft[10][2]={};
  for(int i=0;i<N;i++) for(int c=0;c<2;c++){
    double s=0;
    for(int r=0;r<2*N;r++) s+=Gamma[r][i]*Phi[r][c];
    Ft[i][c]=s;
  }

  // g = F' * x0  for x0 = [3, 0] (large displacement, tests constraint)
  double x0_pos = 3.0, x0_vel = 0.0;
  Vec<QP_N> g_vec;
  for(int i=0;i<N;i++) g_vec[i] = Ft[i][0]*x0_pos + Ft[i][1]*x0_vel;

  // Bounds: |u| <= 2.0
  Vec<QP_N> lb, ub;
  for (int i = 0; i < QP_N; ++i) { lb[i] = -2.0; ub[i] = 2.0; }

  // ── Instrument ADMM manually to log per-iteration residuals ──
  // Convention: u_ stores UNSCALED dual λ (same as qp_solver.hpp)
  //   x-update: rhs = rho*z - λ - g
  //   z-update: v = x + λ/rho, z = clip(v, lb, ub)
  //   u-update: λ += rho*(x - z)
  CSV csv("qp_exp2_convergence.csv");
  csv.header("iteration,primal_res,dual_res,obj_val,converged_at");

  const double rho = 1.0;

  // Factorize (H + rho*I) once
  Mat<QP_N> M_factor = H_real;
  for (int i = 0; i < QP_N; ++i) M_factor(i,i) += rho;
  Mat<QP_N> L_chol;
  cholesky(L_chol, M_factor);

  Vec<QP_N> x, z, lambda;  // all zero-init
  int converged_at = -1;
  const double eps = 1e-4;

  for (int k = 0; k < 150; ++k) {
    // x-update: x = (H + rho*I)^{-1} * (rho*z - lambda - g)
    Vec<QP_N> rhs;
    for (int i = 0; i < QP_N; ++i) rhs[i] = rho*z[i] - lambda[i] - g_vec[i];
    x = chol_solve(L_chol, rhs);

    // z-update: z = clip(x + lambda/rho, lb, ub)  [unscaled dual]
    Vec<QP_N> z_prev = z;
    for (int i = 0; i < QP_N; ++i) {
      double v = x[i] + lambda[i] / rho;
      z[i] = std::max(lb[i], std::min(ub[i], v));
    }

    // lambda-update: lambda += rho*(x - z)
    for (int i = 0; i < QP_N; ++i) lambda[i] += rho*(x[i] - z[i]);

    // Residuals
    double pr = (x - z).norm2();
    double dr = rho * (z - z_prev).norm2();

    // Objective: 0.5*z'*H*z + g'*z  (full, correct)
    Vec<QP_N> Hz = H_real.matvec(z);
    double obj = 0.0;
    for (int i = 0; i < QP_N; ++i) obj += 0.5*Hz[i]*z[i] + g_vec[i]*z[i];

    if (converged_at < 0 && pr < eps*std::sqrt(QP_N) && dr < eps*std::sqrt(QP_N))
      converged_at = k + 1;

    csv.row(k+1, pr, dr, obj, converged_at > 0 ? converged_at : -1);
  }

  std::cout << "  x0 = [" << x0_pos << ", " << x0_vel << "]  "
            << "|u|<=2.0  rho=" << rho << std::endl;
  std::cout << "  Converged at iteration: " << converged_at << std::endl;
  std::cout << "  ✓ EXP 2 DONE — CSV written (real MPC H, correct objective)" << std::endl;
}

// ============================================================
//              EXP 3 — AGV Closed-Loop Simulation
// ============================================================
void exp3_agv_trajectory() {
  std::cout << "\n" << std::string(60,'=') << std::endl;
  std::cout << "EXP 3 — AGV Closed-Loop MPC with Input Constraints"
            << std::endl;
  std::cout << std::string(60,'=') << std::endl;

  AGVModel model;
  int N_SIM = 200;  // 200 steps = 2 seconds

  Matrix Q = Matrix::eye(2);    // state weight
  Matrix R = Matrix(1,1);
  R(0,0) = 0.1;

  // Reference tracking: x_ref = [2.0; 0.0]
  // Initial state: x0 = [0; 0]

  // Three scenarios
  struct Scenario {
    std::string name;
    double x0_pos, x0_vel;
    double u_bound;
    double x_ref;
  };

  std::vector<Scenario> scenarios = {
    {"unconstrained",   0.0, 0.0, 10.0, 2.0},  // effectively unconstrained
    {"light_constraint",0.0, 0.0,  1.0, 2.0},  // |u| <= 1
    {"tight_constraint",0.0, 0.0,  0.5, 2.0},  // |u| <= 0.5 (AGV torque limit)
  };

  CSV csv("qp_exp3_trajectory.csv");
  csv.header("scenario,step,time,position,velocity,control,ref_pos,settle_error");

  for (auto& sc : scenarios) {
    std::cout << "\n  Scenario: " << sc.name
              << "  |u| <= " << sc.u_bound << std::endl;

    MPCConfig cfg(10, Q, R);
    cfg.u_min = {-sc.u_bound};
    cfg.u_max = { sc.u_bound};

    // Reference: shift error formulation
    // x_error = x - x_ref, minimize error
    // Simple: subtract reference from state before passing to MPC
    Matrix x_ref = Matrix(2,1);
    x_ref(0,0) = sc.x_ref;
    x_ref(1,0) = 0.0;

    MPCController mpc(model.Amat, model.Bmat, model.Cmat, cfg);

    Matrix x = Matrix(2,1);
    x(0,0) = sc.x0_pos;
    x(1,0) = sc.x0_vel;

    std::cout << "  step | pos    | vel    | u      | err" << std::endl;
    std::cout << "  " << std::string(44,'-') << std::endl;

    for (int k = 0; k < N_SIM; ++k) {
      // Error state: e = x - x_ref
      // MPC drives e -> 0, i.e. x -> x_ref
      // Optimal: u* = -K*e = -K*(x - x_ref) = K*(x_ref - x)
      // When x=0, x_ref=2: e=-2, u* = -K*(-2) > 0 (accelerate forward) CORRECT
      Matrix x_err = x - x_ref;

      // Solve MPC on error state
      auto sol = mpc.solve(x_err);
      Matrix u = sol.getFirstControl(1);
      // Note: mpc.hpp solveUnconstrained computes U* = -H^{-1}*F*x0
      // so u = -K*x_err = K*(x_ref - x) > 0 when x < x_ref  CORRECT SIGN ✓

      double pos = x(0,0);
      double vel = x(1,0);
      double ctrl = u(0,0);
      double err = std::abs(pos - sc.x_ref);
      double t = k * AGVModel::DT;

      if (k % 20 == 0 || k < 5) {
        std::cout << "  " << std::setw(4) << k
                  << " | " << std::setw(6) << std::fixed << std::setprecision(3) << pos
                  << " | " << std::setw(6) << vel
                  << " | " << std::setw(6) << ctrl
                  << " | " << std::setw(5) << err << std::endl;
      }

      csv.row(sc.name, k, t, pos, vel, ctrl, sc.x_ref, err);

      // Simulate plant
      x = model.step(x, u);
    }
  }

  std::cout << "\n  ✓ EXP 3 DONE — CSV written" << std::endl;
}

// ============================================================
//              EXP 4 — WCET Benchmark
// ============================================================
void exp4_wcet_benchmark() {
  std::cout << "\n" << std::string(60,'=') << std::endl;
  std::cout << "EXP 4 — WCET Benchmark (1000 solves, timing histogram)"
            << std::endl;
  std::cout << std::string(60,'=') << std::endl;

  // ── Build real H analytically (same as EXP1/EXP2) ────────────
  constexpr int QP_N = 10;
  const double dt = AGVModel::DT;
  double A[2][2] = {{1,dt},{0,1}};
  double B[2][1] = {{0.5*dt*dt},{dt}};
  const int N = QP_N;

  double Gamma[20][10] = {};
  for(int k=0;k<N;k++){
    double Apow[2][2] = {{1,0},{0,1}};
    for(int j=k;j>=0;j--){
      for(int r=0;r<2;r++){
        double v=0; for(int c=0;c<2;c++) v+=Apow[r][c]*B[c][0];
        Gamma[2*k+r][j]=v;
      }
      if(j>0){
        double tmp[2][2]={};
        for(int r=0;r<2;r++) for(int c=0;c<2;c++)
          for(int kk=0;kk<2;kk++) tmp[r][c]+=A[r][kk]*Apow[kk][c];
        for(int r=0;r<2;r++) for(int c=0;c<2;c++) Apow[r][c]=tmp[r][c];
      }
    }
  }
  double Phi[20][2] = {};
  double Ak2[2][2] = {{1,dt},{0,1}};
  for(int k=0;k<N;k++){
    for(int r=0;r<2;r++) for(int c=0;c<2;c++) Phi[2*k+r][c]=Ak2[r][c];
    if(k<N-1){
      double tmp[2][2]={};
      for(int r=0;r<2;r++) for(int c=0;c<2;c++)
        for(int kk=0;kk<2;kk++) tmp[r][c]+=A[r][kk]*Ak2[kk][c];
      for(int r=0;r<2;r++) for(int c=0;c<2;c++) Ak2[r][c]=tmp[r][c];
    }
  }
  Mat<QP_N> H_real;
  for(int i=0;i<N;i++) for(int j=0;j<N;j++){
    double s=0; for(int r=0;r<2*N;r++) s+=Gamma[r][i]*Gamma[r][j];
    H_real(i,j)=s+(i==j?0.1:0.0);
  }
  double Ft[10][2]={};
  for(int i=0;i<N;i++) for(int c=0;c<2;c++){
    double s=0; for(int r=0;r<2*N;r++) s+=Gamma[r][i]*Phi[r][c];
    Ft[i][c]=s;
  }
  Vec<QP_N> lb, ub;
  for(int i=0;i<N;i++){ lb[i]=-2.0; ub[i]=2.0; }

  // ── ADMM cold-start solver (no warm-start → true WCET) ───────
  Params admm_cold_params;
  admm_cold_params.rho        = 1.0;
  admm_cold_params.max_iter   = 100;
  admm_cold_params.eps_abs    = 1e-4;
  admm_cold_params.eps_rel    = 1e-3;
  admm_cold_params.warm_start = false;  // MUST be false for WCET measurement
  ADMMSolver<QP_N> admm_cold;
  admm_cold.setup(H_real, admm_cold_params);

  // ── ADMM warm-start solver (realistic closed-loop) ───────────
  Params admm_warm_params = admm_cold_params;
  admm_warm_params.warm_start = true;
  ADMMSolver<QP_N> admm_warm;
  admm_warm.setup(H_real, admm_warm_params);

  // ── PGD baseline (standalone, correct) ───────────────────────
  PGDSolver<QP_N> pgd_solver;

  const int N_BENCH = 1000;
  const int N_WARMUP = 20;  // CPU/cache warmup, results discarded

  CSV csv("qp_exp4_timing.csv");
  csv.header("trial,x0_pos,x0_vel,"
             "admm_cold_us,admm_warm_us,pgd_us,"
             "admm_cold_iters,admm_warm_iters,pgd_iters,"
             "admm_cold_converged");

  std::vector<double> t_cold, t_warm, t_pgd;
  t_cold.reserve(N_BENCH); t_warm.reserve(N_BENCH); t_pgd.reserve(N_BENCH);

  // States: full range for diverse WCET coverage
  double states[] = {-5,-4.5,-4,-3.5,-3,-2.5,-2,-1.5,-1,-0.5,
                      0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5};
  const int n_states = 20;

  // ── CPU warmup: run N_WARMUP iterations, discard ─────────────
  for (int w = 0; w < N_WARMUP; ++w) {
    Vec<QP_N> g_dummy;
    for(int i=0;i<QP_N;i++) g_dummy[i]=Ft[i][0]*states[w%n_states];
    admm_cold.reset();
    admm_cold.solve(g_dummy, lb, ub);
    pgd_solver.solve(H_real, g_dummy, lb, ub, 200, 1e-4);
  }

  // ── Benchmark loop ───────────────────────────────────────────
  for (int trial = 0; trial < N_BENCH; ++trial) {
    double pos = states[trial % n_states];
    double vel = 0.1 * ((trial % 7) - 3);  // vel in {-0.3,...,0.3}

    Vec<QP_N> g_vec;
    for(int i=0;i<QP_N;i++) g_vec[i] = Ft[i][0]*pos + Ft[i][1]*vel;

    // ADMM cold-start (WCET measurement — no warm state)
    admm_cold.reset();
    auto tc0 = Clock::now();
    auto sol_ac = admm_cold.solve(g_vec, lb, ub);
    auto tc1 = Clock::now();
    double us_cold = std::chrono::duration_cast<Microsecs>(tc1-tc0).count();

    // ADMM warm-start (realistic closed-loop scenario)
    auto tw0 = Clock::now();
    auto sol_aw = admm_warm.solve(g_vec, lb, ub);
    auto tw1 = Clock::now();
    double us_warm = std::chrono::duration_cast<Microsecs>(tw1-tw0).count();

    // PGD standalone (correct comparison)
    auto tp0 = Clock::now();
    auto sol_p = pgd_solver.solve(H_real, g_vec, lb, ub, 500, 1e-6);
    auto tp1 = Clock::now();
    double us_pgd = std::chrono::duration_cast<Microsecs>(tp1-tp0).count();

    t_cold.push_back(us_cold);
    t_warm.push_back(us_warm);
    t_pgd.push_back(us_pgd);

    csv.row(trial, pos, vel,
            us_cold, us_warm, us_pgd,
            sol_ac.iterations, sol_aw.iterations, sol_p.iters,
            sol_ac.converged ? 1 : 0);
  }

  // ── Statistics ───────────────────────────────────────────────
  auto stats = [](const std::vector<double>& v) {
    double sum=0, mx=0, mn=1e9;
    for (auto x : v){ sum+=x; mx=std::max(mx,x); mn=std::min(mn,x); }
    double mean=sum/v.size(), var=0;
    for (auto x : v) var+=(x-mean)*(x-mean);
    return std::make_tuple(mean, mx, mn, std::sqrt(var/v.size()));
  };
  auto [cm,cx,cn,cs] = stats(t_cold);
  auto [wm,wx,wn,ws] = stats(t_warm);
  auto [pm,px,pn,ps] = stats(t_pgd);

  std::cout << std::fixed << std::setprecision(1) << std::endl;
  std::cout << "  ┌──────────────────┬──────┬──────┬──────┬─────────┐" << std::endl;
  std::cout << "  │ Solver           │ Mean │  Max │  Min │   Std   │" << std::endl;
  std::cout << "  ├──────────────────┼──────┼──────┼──────┼─────────┤" << std::endl;
  std::cout << "  │ ADMM cold (WCET) │ " << std::setw(4)<<cm <<" │ "<<std::setw(4)<<cx<<" │ "<<std::setw(4)<<cn<<" │ "<<std::setw(7)<<cs<<" │"<<std::endl;
  std::cout << "  │ ADMM warm (loop) │ " << std::setw(4)<<wm <<" │ "<<std::setw(4)<<wx<<" │ "<<std::setw(4)<<wn<<" │ "<<std::setw(7)<<ws<<" │"<<std::endl;
  std::cout << "  │ PGD baseline     │ " << std::setw(4)<<pm <<" │ "<<std::setw(4)<<px<<" │ "<<std::setw(4)<<pn<<" │ "<<std::setw(7)<<ps<<" │"<<std::endl;
  std::cout << "  └──────────────────┴──────┴──────┴──────┴─────────┘" << std::endl;
  std::cout << "\n  WCET (ADMM cold): " << cx << " µs = " << cx/1000.0 << " ms  → "
            << (cx < 10000 ? "✓ PASS (< 10ms)" : "✗ FAIL (> 10ms)") << std::endl;
  std::cout << "  Warm-start speedup: " << cm/wm << "x mean,  "
            << cx/wx << "x WCET" << std::endl;
  std::cout << "  ADMM vs PGD: " << pm/cm << "x mean speedup (cold)" << std::endl;

  std::cout << "\n  ✓ EXP 4 DONE — CSV written" << std::endl;
}

// ============================================================
//              EXP 5 — Warm-Start Benefit
// ============================================================
void exp5_warm_start() {
  std::cout << "\n" << std::string(60,'=') << std::endl;
  std::cout << "EXP 5 — Warm-Start Benefit in Closed-Loop"
            << std::endl;
  std::cout << std::string(60,'=') << std::endl;

  // ── Experiment design ────────────────────────────────────────
  // Goal: measure iteration savings from warm-starting ADMM.
  //
  // Both warm and cold solvers see IDENTICAL state x at each step
  // (plant advances using warm solution — fair state comparison).
  //
  // Cold: ADMMSolver with warm_start=false + explicit reset() each step
  //   → always starts from x=0, z=0, lambda=0
  // Warm: ADMMSolver with warm_start=true
  //   → reuses x,z,lambda from previous solve
  //
  // Task: regulation x → 0 from x0=[4,0]
  //   (x_ref=[0,0], x_err = x - 0 = x, MPC drives x → 0)

  const double dt = AGVModel::DT;
  double A_arr[2][2] = {{1,dt},{0,1}};
  double B_arr[2][1] = {{0.5*dt*dt},{dt}};
  constexpr int QP_N = 10;
  const int N = QP_N;

  // Build H analytically (same as other exps)
  double Gamma[20][10] = {};
  for(int k=0;k<N;k++){
    double Apow[2][2] = {{1,0},{0,1}};
    for(int j=k;j>=0;j--){
      for(int r=0;r<2;r++){
        double v=0; for(int c=0;c<2;c++) v+=Apow[r][c]*B_arr[c][0];
        Gamma[2*k+r][j]=v;
      }
      if(j>0){
        double tmp[2][2]={};
        for(int r=0;r<2;r++) for(int c=0;c<2;c++)
          for(int kk=0;kk<2;kk++) tmp[r][c]+=A_arr[r][kk]*Apow[kk][c];
        for(int r=0;r<2;r++) for(int c=0;c<2;c++) Apow[r][c]=tmp[r][c];
      }
    }
  }
  double Phi[20][2] = {};
  double Ak2[2][2] = {{1,dt},{0,1}};
  for(int k=0;k<N;k++){
    for(int r=0;r<2;r++) for(int c=0;c<2;c++) Phi[2*k+r][c]=Ak2[r][c];
    if(k<N-1){
      double tmp[2][2]={};
      for(int r=0;r<2;r++) for(int c=0;c<2;c++)
        for(int kk=0;kk<2;kk++) tmp[r][c]+=A_arr[r][kk]*Ak2[kk][c];
      for(int r=0;r<2;r++) for(int c=0;c<2;c++) Ak2[r][c]=tmp[r][c];
    }
  }
  Mat<QP_N> H_real;
  for(int i=0;i<N;i++) for(int j=0;j<N;j++){
    double s=0; for(int r=0;r<2*N;r++) s+=Gamma[r][i]*Gamma[r][j];
    H_real(i,j)=s+(i==j?0.1:0.0);
  }
  double Ft[10][2]={};
  for(int i=0;i<N;i++) for(int c=0;c<2;c++){
    double s=0; for(int r=0;r<2*N;r++) s+=Gamma[r][i]*Phi[r][c];
    Ft[i][c]=s;
  }

  // Bounds: |u| <= 1.0  (constrained to make problem non-trivial)
  Vec<QP_N> lb, ub;
  for(int i=0;i<N;i++){ lb[i]=-1.0; ub[i]=1.0; }

  // ── ADMM warm solver ─────────────────────────────────────────
  Params p_warm;
  p_warm.rho=1.0; p_warm.max_iter=100;
  p_warm.eps_abs=1e-4; p_warm.eps_rel=1e-3;
  p_warm.warm_start=true;   // reuse previous solution
  ADMMSolver<QP_N> solver_warm;
  solver_warm.setup(H_real, p_warm);

  // ── ADMM cold solver ─────────────────────────────────────────
  Params p_cold = p_warm;
  p_cold.warm_start=false;  // always cold (reset happens in solve())
  ADMMSolver<QP_N> solver_cold;
  solver_cold.setup(H_real, p_cold);

  // ── Simulation ───────────────────────────────────────────────
  const int N_SIM = 80;
  double pos = 4.0, vel = 0.0;  // initial state

  CSV csv("qp_exp5_warmstart.csv");
  csv.header("step,position,velocity,iters_warm,iters_cold,iter_savings,u_warm");

  std::cout << "\n  step | pos    | vel    | warm_it | cold_it | savings" << std::endl;
  std::cout << "  " << std::string(55,'-') << std::endl;

  double total_warm = 0, total_cold = 0;

  for (int k = 0; k < N_SIM; ++k) {
    // g = F' * [pos, vel]'  (regulation: x_ref=0, x_err=x)
    Vec<QP_N> g_vec;
    for(int i=0;i<QP_N;i++) g_vec[i] = Ft[i][0]*pos + Ft[i][1]*vel;

    // Warm solve (retains x_,z_,u_ from last call)
    auto sol_w = solver_warm.solve(g_vec, lb, ub);

    // Cold solve (solver_cold has warm_start=false, resets internally)
    auto sol_c = solver_cold.solve(g_vec, lb, ub);

    int iw = sol_w.iterations;
    int ic = sol_c.iterations;
    int savings = ic - iw;
    total_warm += iw;
    total_cold += ic;

    double u_warm = sol_w.x[0];  // first control action

    if (k < 15 || k % 10 == 0) {
      std::cout << "  " << std::setw(4) << k
                << " | " << std::setw(6) << std::fixed << std::setprecision(3) << pos
                << " | " << std::setw(6) << vel
                << " | " << std::setw(7) << iw
                << " | " << std::setw(7) << ic
                << " | " << std::setw(7) << savings << std::endl;
    }

    csv.row(k, pos, vel, iw, ic, savings, u_warm);

    // Advance plant using warm control (cold is observer only)
    // x_{k+1} = A*x + B*u
    double pos_new = pos + dt*vel + 0.5*dt*dt*u_warm;
    double vel_new = vel + dt*u_warm;
    pos = pos_new;
    vel = vel_new;
  }

  std::cout << "\n  Average iterations:"
            << "  warm=" << total_warm/N_SIM
            << "  cold=" << total_cold/N_SIM
            << "  savings=" << (total_cold-total_warm)/N_SIM << "/step"
            << std::endl;
  std::cout << "  Warm-start reduces iterations by ~"
            << 100.0*(total_cold-total_warm)/total_cold << "%" << std::endl;

  std::cout << "\n  ✓ EXP 5 DONE — CSV written" << std::endl;
}

// ============================================================
//                        MAIN
// ============================================================
int main() {
  std::cout << "\n";
  std::cout << "╔══════════════════════════════════════════════════════════╗\n";
  std::cout << "║      ADMM QP SOLVER — Simulation & Benchmark Demo       ║\n";
  std::cout << "║      DCAS Lab · L0 Core Control Stack · TDTU            ║\n";
  std::cout << "╚══════════════════════════════════════════════════════════╝\n";
  std::cout << "\n  All results saved to CSV → run plot_qp_results.py to visualize\n";

  exp1_correctness();
  exp2_convergence();
  exp3_agv_trajectory();
  exp4_wcet_benchmark();
  exp5_warm_start();

  std::cout << "\n" << std::string(60,'=') << std::endl;
  std::cout << "ALL EXPERIMENTS COMPLETE" << std::endl;
  std::cout << "  Run: python3 plot_qp_results.py" << std::endl;
  std::cout << std::string(60,'=') << "\n" << std::endl;

  return 0;
}
