/**
 * @file benchmark_qp_solvers.cpp
 * @brief C++ port of benchmark_qp_solvers.py using CppPlot visualization
 *
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  DCAS Lab QP Solver Benchmark — C++17 + CppPlot                ║
 * ║  Replaces benchmark_qp_solvers.py + plot_qp_results.py         ║
 * ╚══════════════════════════════════════════════════════════════════╝
 *
 * Solvers (matching Python benchmark exactly):
 *   1. DCAS-ADMM cold   — qp_solver.hpp ADMMSolver, rho=1.0, cold-start
 *   2. DCAS-ADMM warm   — qp_solver.hpp ADMMSolver, rho=1.0, warm-start
 *   3. DCAS-ADMM tuned  — ADMMSolver, rho = spectral heuristic (auto-tune)
 *   4. DCAS-PGD         — Projected Gradient + Barzilai-Borwein
 *                         (exact port of mpc.hpp::solveConstrained)
 *   5. CG+Proj          — Nonlinear CG with box projection
 *   6. Chol-Direct      — u*=-H^{-1}g then clip (suboptimal under constraints)
 *
 * Three scenarios (identical to Python):
 *   S1 Easy:   Q=I,           R=0.1,   |u|≤2.0  cond(H)≈1
 *   S2 Medium: Q=diag(10,1),  R=0.01,  |u|≤1.0  cond(H)≈5
 *   S3 Hard:   Q=diag(100,1), R=0.001, |u|≤0.3  cond(H)≈44
 *
 * MPC problem: AGV double-integrator, N=10 horizon, QP dim=10
 *   min  0.5*U'*H*U + g'*U    s.t. -u_b ≤ U_i ≤ u_b
 *   H = Γ'*Q̄*Γ + R̄           (built from real dynamics, not synthetic)
 *   g = Γ'*Q̄*Φ * x₀           (changes every solve)
 *
 * CppPlot SVG outputs:
 *   fig_bench_timing.svg      — bar chart: median solve time per scenario
 *   fig_bench_convergence.svg — ADMM primal/dual residuals + objective
 *   fig_bench_accuracy.svg    — ||u - u*||₂ vs x₀ position
 *   fig_bench_tradeoff.svg    — time vs accuracy Pareto scatter
 *   fig_bench_cdf.svg         — CDF of solve times (deadline analysis)
 *   fig_bench_warmstart.svg   — warm vs cold iterations + savings
 *
 * Data outputs:
 *   benchmark_results.csv     — full raw data
 *   benchmark_summary.txt     — LaTeX-ready summary table
 *
 * Compile:
 *   g++ -std=c++17 -O2 -I../include benchmark_qp_solvers.cpp -o bench_qp
 *   g++ -std=c++17 -O3 -I../include benchmark_qp_solvers.cpp -o bench_qp  # timing
 *
 * DCAS Lab · L0 Core Control Stack · TDTU — TS. Trí Viễn Vũ — March 2026
 *
 * ─────────────────────────────────────────────────────────────────────────
 * COMPATIBILITY & BUG NOTES (from source code review):
 *
 * [OK] qp_solver.hpp ADMM algorithm is CORRECT and matches Python replica:
 *      u_ = λ (unscaled dual), z-update = clip(x + u_/rho, lb, ub) ✓
 *
 * [OK] sol.cost in qp_solver.hpp correctly recovers P from L via:
 *      cost = 0.5*(‖L'z‖² - ρ‖z‖²) + q'z  where L is factor of (P+ρI) ✓
 *
 * [OK] Python build_mpc_matrices() Γ construction is CORRECT:
 *      Apow is applied BEFORE update (Python evaluates RHS first) ✓
 *
 * [BUG-C3] mpc.hpp::solveConstrained() uses PGD internally, does NOT
 *      use qp_solver.hpp. MPCController and ADMMSolver are independent.
 *      demo_qp_mpc.cpp works around this by building H analytically.
 *
 * [BUG-C4] mpc.hpp cost in solveUnconstrained() uses P_terminal at k=N-1
 *      inside the cost sum but Phi/Gamma construction uses Q only → cost
 *      slightly overestimated. Does NOT affect control action. ✓
 *
 * [NOTE] check_every=5 in ADMMSolver → reported iterations may be up to
 *      4 higher than actual convergence point (conservative, not wrong).
 *
 * [NOTE] CGProjSolver comment says "Frank-Wolfe style" — incorrect naming.
 *      It is nonlinear CG + projection. Convergence not guaranteed for
 *      constrained QP without proper line search.
 * ─────────────────────────────────────────────────────────────────────────
 */

#define _USE_MATH_DEFINES
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

// ── CppPlot visualization API (confirmed: Figure fig(w,h); fig.subplot(); fig.savefig()) ──
#include <cppplot/cppplot.hpp>

// ── DCAS ADMM QP solver — the primary solver being benchmarked ──────────────
#include <cppplot/control/qp_solver.hpp>

using namespace cppplot;
using namespace cppplot::control::qp;
using Clock    = std::chrono::high_resolution_clock;
using FpMicros = std::chrono::duration<double, std::micro>;

// ══════════════════════════════════════════════════════════════════════════════
//   §1  ADDITIONAL SOLVERS
//   Port of Python benchmark solvers not in qp_solver.hpp
// ══════════════════════════════════════════════════════════════════════════════

/** Unified result type for all solvers */
template <int N>
struct BenchResult {
    Vec<N>  x;
    double  cost      = 0.0;
    int     iters     = 0;
    bool    converged = false;
};

// ── §1.1  Projected Gradient Descent + Barzilai-Borwein ──────────────────────
/**
 * Exact port of Python PGDSolver AND mpc.hpp::solveConstrained().
 * grad = H*U + g
 * U_new = clip(U - α*grad, lb, ub)
 * α_BB  = ‖ΔU‖² / (ΔU'·H·ΔU)
 */
template <int N>
struct PGDSolver {
    static constexpr const char* NAME = "DCAS-PGD";
    int    max_iter = 1000;
    double tol      = 1e-8;   // gradient norm (matches Python)

    BenchResult<N> solve(const Mat<N>& H, const Vec<N>& g,
                         const Vec<N>& lb, const Vec<N>& ub) const {
        BenchResult<N> r{};
        Vec<N> U;             // zero-init
        double alpha = 0.01;

        for (int k = 0; k < max_iter; ++k) {
            Vec<N> grad = H.matvec(U) + g;
            if (grad.norm2() < tol) { r.converged = true; r.iters = k; break; }

            Vec<N> U_new;
            for (int i = 0; i < N; ++i)
                U_new[i] = std::max(lb[i], std::min(ub[i], U[i] - alpha*grad[i]));

            // Barzilai-Borwein adaptive step
            Vec<N> dU   = U_new - U;
            double dU2  = dU.dot(dU);
            if (dU2 > 1e-12) {
                Vec<N> HdU   = H.matvec(dU);
                double dUHdU = dU.dot(HdU);
                if (std::abs(dUHdU) > 1e-12)
                    alpha = std::max(0.001, std::min(1.0, dU2/dUHdU));
            }
            U = U_new;
            r.iters = k + 1;
        }
        r.x = U;
        Vec<N> HU = H.matvec(U);
        for (int i = 0; i < N; ++i) r.cost += 0.5*HU[i]*U[i] + g[i]*U[i];
        return r;
    }
};

// ── §1.2  Nonlinear CG + Box Projection ──────────────────────────────────────
/**
 * Port of Python CGProjSolver (Polak-Ribière β, box projection).
 * Convergence not guaranteed for constrained QP without line search.
 * Note: Python comment says "Frank-Wolfe style" but algorithm is NL-CG.
 */
template <int N>
struct CGProjSolver {
    static constexpr const char* NAME = "CG+Proj";
    int    max_iter = 500;
    double tol      = 1e-8;

    BenchResult<N> solve(const Mat<N>& H, const Vec<N>& g,
                         const Vec<N>& lb, const Vec<N>& ub) const {
        BenchResult<N> r{};
        Vec<N> x;
        Vec<N> grad = H.matvec(x) + g;
        Vec<N> p;
        for (int i = 0; i < N; ++i) p[i] = -grad[i];

        for (int k = 0; k < max_iter; ++k) {
            Vec<N>  Hp  = H.matvec(p);
            double  pHp = p.dot(Hp);
            if (std::abs(pHp) < 1e-15) { r.iters = k; r.converged = true; break; }

            double alpha = -(grad.dot(p)) / pHp;
            Vec<N>  x_new;
            for (int i = 0; i < N; ++i)
                x_new[i] = std::max(lb[i], std::min(ub[i], x[i] + alpha*p[i]));

            Vec<N> grad_new = H.matvec(x_new) + g;
            if (grad_new.norm2() < tol) {
                x = x_new; r.iters = k+1; r.converged = true; break;
            }
            double g2    = grad.dot(grad);
            double g2new = grad_new.dot(grad_new);
            double beta  = (g2 > 1e-15) ? g2new/g2 : 0.0;
            for (int i = 0; i < N; ++i) p[i] = -grad_new[i] + beta*p[i];
            x = x_new; grad = grad_new;
            r.iters = k + 1;
        }
        r.x = x;
        Vec<N> Hx = H.matvec(x);
        for (int i = 0; i < N; ++i) r.cost += 0.5*Hx[i]*x[i] + g[i]*x[i];
        return r;
    }
};

// ── §1.3  Cholesky-Direct: u*=-H^{-1}g then clip ─────────────────────────────
/**
 * Port of Python CholDirectSolver.
 * Analytically solves unconstrained QP then projects onto [lb, ub].
 * SUBOPTIMAL when constraints active (projection ≠ KKT solution).
 * Used as speed baseline and to quantify constraint activity.
 */
template <int N>
struct CholDirectSolver {
    static constexpr const char* NAME = "Chol-Direct";
    Mat<N> L;
    bool   ready = false;

    bool setup(const Mat<N>& H) { return (ready = cholesky(L, H)); }

    BenchResult<N> solve(const Vec<N>& g,
                         const Vec<N>& lb, const Vec<N>& ub) const {
        BenchResult<N> r{};
        r.iters = 1; r.converged = true;
        if (!ready) return r;
        Vec<N> neg_g;
        for (int i = 0; i < N; ++i) neg_g[i] = -g[i];
        Vec<N> u_unc = chol_solve(L, neg_g);
        for (int i = 0; i < N; ++i)
            r.x[i] = std::max(lb[i], std::min(ub[i], u_unc[i]));
        return r;   // cost filled later using known H
    }
};

// ══════════════════════════════════════════════════════════════════════════════
//   §2  MPC PROBLEM CONSTRUCTION
//   Builds real H and g from AGV double-integrator dynamics.
//   C++ port of Python build_mpc_matrices() — verified correct.
// ══════════════════════════════════════════════════════════════════════════════

/**
 * AGV double-integrator:  x = [pos, vel]',  u = acceleration,  dt = 0.01 s
 *   A = [1  dt; 0  1],   B = [0.5*dt²; dt]
 *
 * Prediction matrices built analytically (no cppplot::Matrix bridge needed):
 *   Φ ∈ R^{2N×2}  — free response from x₀
 *   Γ ∈ R^{2N×N}  — forced response (lower-triangular Toeplitz)
 *   H = Γ'Q̄Γ + R̄  — QP Hessian (built once, reused each solve)
 *   g = F'·x₀      — linear term (changes each solve)
 *
 * @tparam QN  QP dimension = horizon N × num_inputs = 10×1 = 10
 */
template <int QN>
struct MPCProblem {
    static constexpr int    NX  = 2;
    static constexpr int    HOR = QN;  // horizon = 10
    static constexpr double DT  = 0.01;

    Mat<QN> H;              // QP Hessian — symmetric PSD
    double  Ft[QN][NX];     // F' = Γ'Q̄Φ;  g = F'·x₀
    double  eig_min = 0.0, eig_max = 0.0;  // Gershgorin bounds

    /**
     * @param Q_diag  Diagonal entries of state weight Q ∈ R^{NX×NX}
     * @param R_val   Scalar input weight R
     */
    void build(const double Q_diag[NX], double R_val) {
        const double dt = DT;
        double A[NX][NX] = {{1.0, dt}, {0.0, 1.0}};
        double B[NX][1]  = {{0.5*dt*dt}, {dt}};

        // ── Build Φ (2N×2) and Γ (2N×N) ──────────────────────────────────
        double Phi[2*QN][NX] = {};
        double Gam[2*QN][QN] = {};
        double Ak[NX][NX]    = {{1.0, dt}, {0.0, 1.0}};  // A^1

        for (int k = 0; k < HOR; ++k) {
            // Φ row-block k = A^{k+1}
            for (int r = 0; r < NX; ++r)
                for (int c = 0; c < NX; ++c)
                    Phi[NX*k+r][c] = Ak[r][c];

            // Γ columns j=k..0: Γ(2k:2k+2, j) = A^{k-j}·B
            double Apow[NX][NX] = {{1,0},{0,1}};   // A^0 = I
            for (int j = k; j >= 0; --j) {
                for (int r = 0; r < NX; ++r) {
                    double v = 0;
                    for (int p = 0; p < NX; ++p) v += Apow[r][p]*B[p][0];
                    Gam[NX*k+r][j] = v;
                }
                if (j > 0) {   // Advance Apow = A·Apow
                    double tmp[NX][NX] = {};
                    for (int r=0;r<NX;++r) for(int c=0;c<NX;++c)
                        for(int p=0;p<NX;++p) tmp[r][c] += A[r][p]*Apow[p][c];
                    for (int r=0;r<NX;++r) for(int c=0;c<NX;++c) Apow[r][c]=tmp[r][c];
                }
            }
            // Advance Ak = A·Ak for next iteration
            if (k < HOR-1) {
                double tmp[NX][NX] = {};
                for (int r=0;r<NX;++r) for(int c=0;c<NX;++c)
                    for(int p=0;p<NX;++p) tmp[r][c] += A[r][p]*Ak[p][c];
                for (int r=0;r<NX;++r) for(int c=0;c<NX;++c) Ak[r][c]=tmp[r][c];
            }
        }

        // ── H = Γ'·Q̄·Γ + R̄  (Q̄ = blkdiag(Q,...), diagonal Q case) ──────
        for (int i = 0; i < HOR; ++i)
            for (int j = 0; j < HOR; ++j) {
                double s = 0.0;
                for (int r = 0; r < 2*HOR; ++r)
                    s += Gam[r][i] * Q_diag[r % NX] * Gam[r][j];
                H(i,j) = s + (i == j ? R_val : 0.0);
            }

        // ── F' = Γ'·Q̄·Φ ∈ R^{N×2} ──────────────────────────────────────
        for (int i = 0; i < HOR; ++i)
            for (int c = 0; c < NX; ++c) {
                double s = 0.0;
                for (int r = 0; r < 2*HOR; ++r)
                    s += Gam[r][i] * Q_diag[r % NX] * Phi[r][c];
                Ft[i][c] = s;
            }

        // ── Eigenvalue bounds (Gershgorin circles) ─────────────────────────
        eig_min = H(0,0); eig_max = H(0,0);
        for (int i = 0; i < HOR; ++i) {
            double off = 0.0;
            for (int j = 0; j < HOR; ++j) if (j!=i) off += std::abs(H(i,j));
            eig_max = std::max(eig_max, H(i,i) + off);
            eig_min = std::min(eig_min, std::max(0.0, H(i,i) - off));
        }
    }

    /** g = F'·x₀ */
    Vec<QN> g_from_state(double pos, double vel) const {
        Vec<QN> g;
        for (int i = 0; i < HOR; ++i)
            g[i] = Ft[i][0]*pos + Ft[i][1]*vel;
        return g;
    }

    /** QP objective: 0.5·x'·H·x + g'·x */
    double cost(const Vec<QN>& x, const Vec<QN>& g_vec) const {
        Vec<QN> Hx = H.matvec(x);
        double c = 0.0;
        for (int i = 0; i < HOR; ++i) c += 0.5*Hx[i]*x[i] + g_vec[i]*x[i];
        return c;
    }

    /** Spectral heuristic rho (matches Python ADMM-tuned) */
    double spectral_rho() const {
        double r = std::max(10.0*eig_max,
                            std::sqrt(std::max(eig_min,1e-10)*eig_max));
        return std::min(r, 1e6);
    }

    /** Condition number estimate */
    double cond() const {
        return (eig_min > 1e-14) ? eig_max/eig_min : 1e9;
    }
};

// ══════════════════════════════════════════════════════════════════════════════
//   §3  BENCHMARK DATA STRUCTURES
// ══════════════════════════════════════════════════════════════════════════════

struct BenchRow {
    std::string scenario;
    std::string solver;
    double x0_pos, x0_vel;
    double u0, u0_ref;
    double u_l2_err;
    double cost_rel_err;
    int    iters;
    bool   converged;
    int    n_active;   // active constraints in reference solution
    double t_min_us, t_median_us, t_p95_us, t_max_us, t_std_us;
};

// ── Statistics ────────────────────────────────────────────────────────────────

static double pct(std::vector<double> v, double p) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    double idx = p/100.0*(static_cast<int>(v.size())-1);
    int lo = static_cast<int>(idx), hi = std::min(lo+1,(int)v.size()-1);
    return v[lo]*(1.0-(idx-lo)) + v[hi]*(idx-lo);
}
static double median_v(std::vector<double> v) { return pct(v,50.0); }
static double mean_v(const std::vector<double>& v) {
    return v.empty() ? 0.0 : std::accumulate(v.begin(),v.end(),0.0)/v.size();
}
static double stddev_v(const std::vector<double>& v) {
    double m = mean_v(v), s = 0.0;
    for (auto x : v) s += (x-m)*(x-m);
    return (v.size()>1) ? std::sqrt(s/(v.size()-1)) : 0.0;
}
template<typename Fn>
static std::vector<double> collect(const std::vector<BenchRow>& rows,
                                    const std::string& sc, const std::string& sv,
                                    Fn fn) {
    std::vector<double> out;
    for (const auto& r : rows)
        if (r.scenario==sc && r.solver==sv) out.push_back(fn(r));
    return out;
}

// ══════════════════════════════════════════════════════════════════════════════
//   §4  BENCHMARK ENGINE
// ══════════════════════════════════════════════════════════════════════════════

static constexpr int QP_N    = 10;
static constexpr int N_REPS  = 50;   // timing reps per (scenario, x0)
static constexpr int N_WARMUP= 10;   // discarded warmup reps

static const std::vector<std::string> SOLVER_ORDER = {
    "ADMM-cold","ADMM-warm","ADMM-tuned","DCAS-PGD","CG+Proj","Chol-Direct"
};

struct Scenario {
    std::string name, label;
    double      Q_diag[2];
    double      R_val;
    double      u_bound;
    std::vector<std::pair<double,double>> x0_list;
};

static std::vector<Scenario> make_scenarios() {
    std::vector<std::pair<double,double>> x0_s1 = {
        {-5,0},{-4,0},{-3,0},{-2,0},{-1,0},{-0.5,0},{0.5,0},{1,0},{2,0},
        {3,0},{4,0},{4.9,0},{-3,.3},{-3,-.3},{2,.2},{2,-.2},
        {1,.5},{-1,-.5},{4.5,.1},{-4.5,-.1}
    };
    std::vector<std::pair<double,double>> x0_s2 = {
        {-2,0},{-1.5,0},{-1,0},{-0.5,0},{-0.2,0},{0.2,0},{0.5,0},{1,0},
        {1.5,0},{2,0},{-1,.1},{-1,-.1},{1,.1},{1,-.1},
        {.5,.3},{.5,-.3},{1.5,.2},{-1.5,-.2},{2,.1},{-2,-.1}
    };
    std::vector<std::pair<double,double>> x0_s3 = {
        {-3,0},{-2,0},{-1,0},{-0.5,0},{-0.2,0},{0.2,0},{0.5,0},{1,0},
        {2,0},{3,0},{-1,.05},{-1,-.05},{1,.05},{1,-.05},
        {.5,.1},{.5,-.1},{2,.05},{-2,-.05},{3,.02},{-3,-.02}
    };
    return {
        {"S1_easy",  "S1: Easy  (Q=I,        R=0.1,   |u|≤2.0)",
         {1.0,1.0},    0.1,   2.0, x0_s1},
        {"S2_medium","S2: Medium (Q=diag(10,1),R=0.01,  |u|≤1.0)",
         {10.0,1.0},   0.01,  1.0, x0_s2},
        {"S3_hard",  "S3: Hard  (Q=diag(100,1),R=0.001,|u|≤0.3)",
         {100.0,1.0},  0.001, 0.3, x0_s3},
    };
}

/** Measure one solver N_REPS times, compute timing stats + accuracy */
template<typename SolveFn>
static BenchRow measure(const std::string& sc, const std::string& sv,
                         double pos, double vel,
                         const Vec<QP_N>& g_vec,
                         const Vec<QP_N>& lb, const Vec<QP_N>& ub,
                         const Vec<QP_N>& u_ref, double cost_ref,
                         int n_active,
                         const MPCProblem<QP_N>& mpc,
                         SolveFn fn) {
    std::vector<double> times; times.reserve(N_REPS);
    BenchResult<QP_N> sol{};
    for (int rep = 0; rep < N_REPS; ++rep) {
        auto t0 = Clock::now();
        sol = fn();
        auto t1 = Clock::now();
        times.push_back(FpMicros(t1-t0).count());
    }

    double u_err = 0.0;
    for (int i = 0; i < QP_N; ++i) u_err += (sol.x[i]-u_ref[i])*(sol.x[i]-u_ref[i]);
    u_err = std::sqrt(u_err);
    double c     = mpc.cost(sol.x, g_vec);
    double c_err = (std::abs(cost_ref) > 1e-12) ? std::abs(c-cost_ref)/std::abs(cost_ref) : 0.0;

    BenchRow r;
    r.scenario = sc; r.solver = sv;
    r.x0_pos = pos; r.x0_vel = vel;
    r.u0 = sol.x[0]; r.u0_ref = u_ref[0];
    r.u_l2_err = u_err; r.cost_rel_err = c_err;
    r.iters = sol.iters; r.converged = sol.converged; r.n_active = n_active;
    r.t_min_us    = *std::min_element(times.begin(),times.end());
    r.t_median_us = median_v(times);
    r.t_p95_us    = pct(times,95.0);
    r.t_max_us    = *std::max_element(times.begin(),times.end());
    r.t_std_us    = stddev_v(times);
    return r;
}

std::vector<BenchRow> run_benchmark() {
    std::vector<BenchRow> all_rows;
    auto scenarios = make_scenarios();

    std::cout << "\n╔══════════════════════════════════════════════════════════════╗\n"
              << "║  DCAS Lab QP Solver Benchmark — C++ with CppPlot            ║\n"
              << "║  TDTU · L0 Core Control Stack · March 2026                  ║\n"
              << "╚══════════════════════════════════════════════════════════════╝\n"
              << "  N_REPS=" << N_REPS << "  N_WARMUP=" << N_WARMUP
              << "  QP_N=" << QP_N << "\n\n";

    for (auto& sc : scenarios) {
        MPCProblem<QP_N> mpc;
        mpc.build(sc.Q_diag, sc.R_val);
        double rho_opt = mpc.spectral_rho();

        std::cout << "══════════════════════════════════════════════════════════════\n"
                  << "Scenario: " << sc.name << "\n  " << sc.label << "\n"
                  << "  cond(H)≈" << std::fixed << std::setprecision(1) << mpc.cond()
                  << "  eig=[" << mpc.eig_min << ", " << mpc.eig_max << "]"
                  << "  rho_opt=" << rho_opt << "\n\n";

        Vec<QP_N> lb, ub;
        for (int i=0;i<QP_N;++i) { lb[i]=-sc.u_bound; ub[i]=sc.u_bound; }

        // ── Setup solvers (Cholesky factorization done ONCE per scenario) ──
        Params p_cold; p_cold.rho=1.0; p_cold.max_iter=200;
        p_cold.eps_abs=1e-6; p_cold.eps_rel=1e-5; p_cold.warm_start=false;
        ADMMSolver<QP_N> admm_cold; admm_cold.setup(mpc.H, p_cold);

        Params p_warm = p_cold; p_warm.warm_start = true;
        ADMMSolver<QP_N> admm_warm; admm_warm.setup(mpc.H, p_warm);

        Params p_tuned = p_cold; p_tuned.rho = rho_opt;
        ADMMSolver<QP_N> admm_tuned; admm_tuned.setup(mpc.H, p_tuned);

        // Reference: extremely tight tolerance (ground truth u*)
        Params p_ref; p_ref.rho=1.0; p_ref.max_iter=5000;
        p_ref.eps_abs=1e-12; p_ref.eps_rel=1e-11; p_ref.warm_start=false;
        ADMMSolver<QP_N> admm_ref; admm_ref.setup(mpc.H, p_ref);

        PGDSolver<QP_N>      pgd;
        CGProjSolver<QP_N>   cg;
        CholDirectSolver<QP_N> chol_d; chol_d.setup(mpc.H);

        // CPU warmup (heat cache and branch predictor, discard results)
        Vec<QP_N> g_dum = mpc.g_from_state(1.0, 0.0);
        for (int w=0; w<N_WARMUP; ++w) {
            admm_cold.reset(); admm_cold.solve(g_dum, lb, ub);
            admm_warm.solve(g_dum, lb, ub);
            pgd.solve(mpc.H, g_dum, lb, ub);
            cg.solve(mpc.H, g_dum, lb, ub);
        }

        // ── Per-x₀ benchmark loop ──────────────────────────────────────────
        for (const auto& x0_pair : sc.x0_list) {
            double pos = x0_pair.first;
            double vel = x0_pair.second;
            Vec<QP_N> g_vec = mpc.g_from_state(pos, vel);

            // Ground-truth reference solution
            admm_ref.reset();
            auto sol_ref = admm_ref.solve(g_vec, lb, ub);
            Vec<QP_N> u_ref   = sol_ref.x;
            double    cost_ref = mpc.cost(u_ref, g_vec);
            int       n_act    = 0;
            for (int i=0;i<QP_N;++i)
                if (u_ref[i]<=lb[i]+1e-6 || u_ref[i]>=ub[i]-1e-6) ++n_act;

            // Helper: Solution<N> → BenchResult<N>
            auto a2b = [](const Solution<QP_N>& s) -> BenchResult<QP_N> {
                BenchResult<QP_N> r;
                r.x=s.x; r.cost=s.cost; r.iters=s.iterations; r.converged=s.converged;
                return r;
            };

            // ADMM cold
            all_rows.push_back(measure(sc.name,"ADMM-cold",pos,vel,g_vec,lb,ub,u_ref,cost_ref,n_act,mpc,
                [&]{ admm_cold.reset(); return a2b(admm_cold.solve(g_vec,lb,ub)); }));

            // ADMM warm (reset per x0 block → fresh closed-loop start per point)
            admm_warm.reset();
            all_rows.push_back(measure(sc.name,"ADMM-warm",pos,vel,g_vec,lb,ub,u_ref,cost_ref,n_act,mpc,
                [&]{ return a2b(admm_warm.solve(g_vec,lb,ub)); }));

            // ADMM tuned
            all_rows.push_back(measure(sc.name,"ADMM-tuned",pos,vel,g_vec,lb,ub,u_ref,cost_ref,n_act,mpc,
                [&]{ admm_tuned.reset(); return a2b(admm_tuned.solve(g_vec,lb,ub)); }));

            // PGD
            all_rows.push_back(measure(sc.name,"DCAS-PGD",pos,vel,g_vec,lb,ub,u_ref,cost_ref,n_act,mpc,
                [&]{ return pgd.solve(mpc.H,g_vec,lb,ub); }));

            // CG+Proj
            all_rows.push_back(measure(sc.name,"CG+Proj",pos,vel,g_vec,lb,ub,u_ref,cost_ref,n_act,mpc,
                [&]{ return cg.solve(mpc.H,g_vec,lb,ub); }));

            // Chol-Direct (cost computed by measure() using mpc.cost)
            all_rows.push_back(measure(sc.name,"Chol-Direct",pos,vel,g_vec,lb,ub,u_ref,cost_ref,n_act,mpc,
                [&]{ return chol_d.solve(g_vec,lb,ub); }));
        }

        // Per-scenario summary
        std::cout << "  " << std::left<<std::setw(14)<<"Solver"
                  << std::right<<std::setw(9)<<"Med(µs)"<<std::setw(9)<<"P95(µs)"
                  <<std::setw(9)<<"Max(µs)"<<std::setw(8)<<"Iters"
                  <<std::setw(12)<<"||u-u*||₂"<<std::setw(9)<<"Conv%\n";
        std::cout << "  " << std::string(68,'-') << "\n";
        for (auto& sn : SOLVER_ORDER) {
            auto tm=collect(all_rows,sc.name,sn,[](const BenchRow&r){return r.t_median_us;});
            auto tp=collect(all_rows,sc.name,sn,[](const BenchRow&r){return r.t_p95_us;});
            auto tx=collect(all_rows,sc.name,sn,[](const BenchRow&r){return r.t_max_us;});
            auto it=collect(all_rows,sc.name,sn,[](const BenchRow&r){return (double)r.iters;});
            auto er=collect(all_rows,sc.name,sn,[](const BenchRow&r){return r.u_l2_err;});
            int tot=(int)tm.size(); if(!tot) continue;
            int cv=0;
            for(auto&r:all_rows) if(r.scenario==sc.name&&r.solver==sn&&r.converged)++cv;
            std::cout<<"  "<<std::left<<std::setw(14)<<sn<<std::right<<std::fixed
                     <<std::setprecision(1)<<std::setw(9)<<median_v(tm)
                     <<std::setw(9)<<median_v(tp)
                     <<std::setw(9)<<*std::max_element(tx.begin(),tx.end())
                     <<std::setw(8)<<(int)median_v(it)
                     <<"  "<<std::scientific<<std::setprecision(2)
                     <<std::setw(10)<<median_v(er)
                     <<std::fixed<<std::setprecision(1)
                     <<std::setw(9)<<100.0*cv/tot<<"%\n";
        }
        std::cout << "\n";
    }
    return all_rows;
}

// ══════════════════════════════════════════════════════════════════════════════
//   §5  CPPPLOT FIGURES
//   6 figures using confirmed CppPlot API:
//     Figure fig(w, h);  auto& ax = fig.subplot(r,c,i);  fig.savefig("f.svg");
//     ax.plot(x,y,"fmt",{{"label",string}});  ax.scatter(x,y,{{"c",string}});
//     ax.bar(x,h);  ax.set_title/xlabel/ylabel(str);  ax.set_xlim/ylim(a,b);
//     ax.grid(bool);  ax.legend(bool);
// ══════════════════════════════════════════════════════════════════════════════

static const std::string SC_NAMES[] = {"S1_easy","S2_medium","S3_hard"};
static const std::string SC_SHORT[] = {"S1: Easy","S2: Medium","S3: Hard"};

// ── Fig 1: Timing bar chart ───────────────────────────────────────────────────
/**
 * 3 subplots (one per scenario), bar heights = median solve time.
 * Solver index on x-axis: 1=ADMM-cold, 2=ADMM-warm, ..., 6=Chol-Direct.
 */
void fig_timing(const std::vector<BenchRow>& rows, const std::string& out) {
    Figure fig(1300, 520);
    for (int si=0; si<3; ++si) {
        auto& ax = fig.subplot(1,3,si+1);
        const std::string& sc = SC_NAMES[si];
        std::vector<double> xs, ys;
        for (int k=0; k<(int)SOLVER_ORDER.size(); ++k) {
            auto t = collect(rows,sc,SOLVER_ORDER[k],[](const BenchRow&r){return r.t_median_us;});
            if (t.empty()) continue;
            xs.push_back(k+1.0);
            ys.push_back(median_v(t));
        }
        ax.bar(xs,ys);
        ax.set_title(SC_SHORT[si] + " — Median Solve Time");
        ax.set_xlabel("Solver  (1=ADMM-cold 2=warm 3=tuned 4=PGD 5=CG 6=Chol)");
        ax.set_ylabel("Median time (us)");
        ax.set_xlim(0.0, (double)SOLVER_ORDER.size()+1.5);
        ax.grid(true);
    }
    fig.savefig(out);
    std::cout << "  -> " << out << "\n";
}

// ── Fig 2: ADMM convergence profile ──────────────────────────────────────────
/**
 * Runs ADMM manually for 150 iterations on S3 hard problem (x0=[3,0]).
 * Left subplot:  primal residual ‖x-z‖ and dual residual ρ‖Δz‖ vs iteration.
 * Right subplot: QP objective value convergence.
 * Replaces Python plot_exp2() and demo_qp_mpc.cpp EXP2.
 */
void fig_convergence(const std::vector<BenchRow>& /*unused*/, const std::string& out) {
    MPCProblem<QP_N> mpc;
    double Q3[2] = {100.0, 1.0};
    mpc.build(Q3, 0.001);  // S3 hard

    Vec<QP_N> lb, ub;
    for (int i=0;i<QP_N;++i) { lb[i]=-0.3; ub[i]=0.3; }

    Vec<QP_N> g_vec = mpc.g_from_state(3.0, 0.0);  // challenging state

    const int    MAX_K = 150;
    const double rho   = 1.0;

    // Factorize (H + rho·I)
    Mat<QP_N> Mreg = mpc.H;
    for (int i=0;i<QP_N;++i) Mreg(i,i) += rho;
    Mat<QP_N> Lfact;
    cholesky(Lfact, Mreg);

    Vec<QP_N> x_k, z_k, u_k;
    std::vector<double> iter_v, prim_v, dual_v, obj_v;

    for (int k=0; k<MAX_K; ++k) {
        Vec<QP_N> rhs;
        for (int i=0;i<QP_N;++i) rhs[i]=rho*z_k[i]-u_k[i]-g_vec[i];
        x_k = chol_solve(Lfact, rhs);

        Vec<QP_N> z_prev = z_k;
        for (int i=0;i<QP_N;++i) {
            double v = x_k[i]+u_k[i]/rho;
            z_k[i] = std::max(lb[i],std::min(ub[i],v));
        }
        for (int i=0;i<QP_N;++i) u_k[i] += rho*(x_k[i]-z_k[i]);

        double pr = (x_k-z_k).norm2();
        double dr = rho*(z_k-z_prev).norm2();
        double obj = mpc.cost(z_k,g_vec);

        iter_v.push_back(k+1.0);
        prim_v.push_back(std::max(pr,1e-14));
        dual_v.push_back(std::max(dr,1e-14));
        obj_v.push_back(obj);
    }

    Figure fig(1100, 460);

    // Left: residuals
    {
        auto& ax = fig.subplot(1,2,1);
        ax.plot(iter_v, prim_v, "b-", {{"label",std::string("Primal ||x-z||")}});
        ax.plot(iter_v, dual_v, "r-", {{"label",std::string("Dual rho*||Dz||")}});
        // eps reference line
        double eps_line = 1e-4 * std::sqrt(QP_N);
        ax.plot({1.0,(double)MAX_K}, {eps_line,eps_line}, "k--",
                {{"label",std::string("eps_abs*sqrt(N)")}});
        ax.set_title("ADMM Residuals  (S3, x0=[3,0], rho=1)");
        ax.set_xlabel("Iteration k");
        ax.set_ylabel("Residual");
        ax.legend(true);
        ax.grid(true);
    }

    // Right: objective
    {
        auto& ax = fig.subplot(1,2,2);
        ax.plot(iter_v, obj_v, "g-");
        ax.set_title("Objective Value vs Iteration");
        ax.set_xlabel("Iteration k");
        ax.set_ylabel("0.5*z'Hz + g'z");
        ax.grid(true);
    }
    fig.savefig(out);
    std::cout << "  -> " << out << "\n";
}

// ── Fig 3: Accuracy vs x₀ position ───────────────────────────────────────────
/**
 * ||u - u*||₂ vs x₀ position for each solver.
 * Replaces Python fig_accuracy().
 */
void fig_accuracy(const std::vector<BenchRow>& rows, const std::string& out) {
    Figure fig(1300, 430);
    const std::vector<std::string> sv_plt = {"ADMM-cold","ADMM-warm","ADMM-tuned","DCAS-PGD","CG+Proj"};
    const std::vector<std::string> fmts   = {"r-","b-","m-","g-","c-"};

    for (int si=0; si<3; ++si) {
        auto& ax = fig.subplot(1,3,si+1);
        const std::string& sc = SC_NAMES[si];

        for (int k=0; k<(int)sv_plt.size(); ++k) {
            std::vector<std::pair<double,double>> pts;
            for (const auto& r : rows)
                if (r.scenario==sc && r.solver==sv_plt[k])
                    pts.emplace_back(r.x0_pos, std::max(r.u_l2_err,1e-14));
            if (pts.empty()) continue;
            std::sort(pts.begin(),pts.end());
            std::vector<double> xs,ys;
            for (std::size_t _i=0;_i<pts.size();++_i){xs.push_back(pts[_i].first);ys.push_back(pts[_i].second);}
            ax.plot(xs,ys,fmts[k].c_str(),{{"label",std::string(sv_plt[k])}});
        }
        // 1e-6 accuracy threshold line
        std::vector<double> ref_xs;
        for (const auto& r : rows)
            if (r.scenario==sc && r.solver=="ADMM-cold") ref_xs.push_back(r.x0_pos);
        if (!ref_xs.empty()) {
            double xmin=*std::min_element(ref_xs.begin(),ref_xs.end());
            double xmax=*std::max_element(ref_xs.begin(),ref_xs.end());
            ax.plot({xmin,xmax},{1e-6,1e-6},"k--");
        }
        ax.set_title(SC_SHORT[si] + " — Solution Accuracy");
        ax.set_xlabel("x0 position (m)");
        ax.set_ylabel("||u - u*||2");
        ax.grid(true);
        if (si==0) ax.legend(true);
    }
    fig.savefig(out);
    std::cout << "  -> " << out << "\n";
}

// ── Fig 4: Time vs accuracy Pareto scatter ────────────────────────────────────
/**
 * Each point = one solver. Lower-left = Pareto-optimal.
 * Replaces Python fig_tradeoff().
 */
void fig_tradeoff(const std::vector<BenchRow>& rows, const std::string& out) {
    Figure fig(1300, 430);
    for (int si=0; si<3; ++si) {
        auto& ax = fig.subplot(1,3,si+1);
        const std::string& sc = SC_NAMES[si];
        std::vector<double> xs,ys;
        for (auto& sn : SOLVER_ORDER) {
            auto t=collect(rows,sc,sn,[](const BenchRow&r){return r.t_median_us;});
            auto e=collect(rows,sc,sn,[](const BenchRow&r){return r.u_l2_err;});
            if (t.empty()) continue;
            xs.push_back(median_v(t));
            ys.push_back(std::max(median_v(e),1e-14));
        }
        if (!xs.empty())
            ax.scatter(xs,ys,{{"c",std::string("blue")}});
        ax.set_title(SC_SHORT[si] + " — Time vs Accuracy");
        ax.set_xlabel("Median solve time (us)");
        ax.set_ylabel("Median ||u - u*||2");
        ax.grid(true);
    }
    fig.savefig(out);
    std::cout << "  -> " << out << "\n";
}

// ── Fig 5: Solve time CDF ─────────────────────────────────────────────────────
/**
 * CDF(t) = fraction of solves completing in ≤ t µs.
 * 10ms deadline marker. Replaces Python fig_cdf().
 */
void fig_cdf(const std::vector<BenchRow>& rows, const std::string& out) {
    Figure fig(1300, 430);
    const std::vector<std::string> cdf_sv   = {"ADMM-cold","ADMM-warm","DCAS-PGD","Chol-Direct"};
    const std::vector<std::string> cdf_fmts = {"r-","b-","g-","k-"};

    for (int si=0; si<3; ++si) {
        auto& ax = fig.subplot(1,3,si+1);
        const std::string& sc = SC_NAMES[si];
        for (int k=0; k<(int)cdf_sv.size(); ++k) {
            auto t=collect(rows,sc,cdf_sv[k],[](const BenchRow&r){return r.t_median_us;});
            if (t.empty()) continue;
            std::sort(t.begin(),t.end());
            std::vector<double> cdf_y;
            for (int i=0;i<(int)t.size();++i)
                cdf_y.push_back((i+1.0)/t.size());
            ax.plot(t,cdf_y,cdf_fmts[k].c_str(),{{"label",std::string(cdf_sv[k])}});
        }
        // 10ms deadline marker
        ax.plot({10000.0,10000.0},{0.0,1.05},"r--",{{"label",std::string("10ms limit")}});
        ax.set_title(SC_SHORT[si] + " — Solve Time CDF");
        ax.set_xlabel("Solve time (us)");
        ax.set_ylabel("CDF P(T<=t)");
        ax.set_ylim(0.0, 1.05);
        ax.grid(true);
        if (si==0) ax.legend(true);
    }
    fig.savefig(out);
    std::cout << "  -> " << out << "\n";
}

// ── Fig 6: Warm-start benefit ─────────────────────────────────────────────────
/**
 * Simulates closed-loop regulation x₀=[4,0] → 0 (S1 problem).
 * Compares ADMM warm vs cold iterations + cumulative savings.
 * Replaces Python demo_exp5() / EXP5 in demo_qp_mpc.cpp.
 */
void fig_warmstart(const std::string& out) {
    MPCProblem<QP_N> mpc;
    double Q1[2] = {1.0, 1.0};
    mpc.build(Q1, 0.1);

    Vec<QP_N> lb,ub;
    for (int i=0;i<QP_N;++i){lb[i]=-1.0;ub[i]=1.0;}

    Params pbase; pbase.rho=1.0; pbase.max_iter=100;
    pbase.eps_abs=1e-4; pbase.eps_rel=1e-3;

    ADMMSolver<QP_N> sw,sc;
    pbase.warm_start=true;  sw.setup(mpc.H,pbase);
    pbase.warm_start=false; sc.setup(mpc.H,pbase);

    const int N_SIM = 80;
    double pos=4.0, vel=0.0, dt=MPCProblem<QP_N>::DT;
    double total_w=0, total_c=0;

    std::vector<double> step_v,pos_v,iw_v,ic_v,cumul_v;
    for (int k=0;k<N_SIM;++k) {
        Vec<QP_N> g = mpc.g_from_state(pos,vel);
        auto solw = sw.solve(g,lb,ub);
        sc.reset();
        auto solc = sc.solve(g,lb,ub);
        int iw=solw.iterations, ic=solc.iterations;
        total_w+=iw; total_c+=ic;
        step_v.push_back(k);
        pos_v.push_back(pos);
        iw_v.push_back(iw); ic_v.push_back(ic);
        cumul_v.push_back(total_c-total_w);
        double u=solw.x[0];
        pos += dt*vel + 0.5*dt*dt*u;
        vel += dt*u;
    }

    double save_pct = (total_c>0) ? 100.0*(total_c-total_w)/total_c : 0.0;
    std::cout << "  Warm-start savings: "
              << std::fixed<<std::setprecision(1)<<save_pct<<"% ("
              << (total_c-total_w)/N_SIM << " iter/step avg)\n";

    Figure fig(1200, 430);
    {
        auto& ax = fig.subplot(1,3,1);
        ax.plot(step_v,ic_v,"r-",{{"label",std::string("Cold start")}});
        ax.plot(step_v,iw_v,"b-",{{"label",std::string("Warm start")}});
        ax.set_title("ADMM Iterations per Step");
        ax.set_xlabel("Control step k");
        ax.set_ylabel("Iterations to convergence");
        ax.legend(true); ax.grid(true);
    }
    {
        auto& ax = fig.subplot(1,3,2);
        ax.plot(step_v,pos_v,"g-");
        ax.plot({0.0,(double)(N_SIM-1)},{0.0,0.0},"k--");
        ax.set_title("AGV Position  (regulation x->0)");
        ax.set_xlabel("Control step k");
        ax.set_ylabel("Position (m)");
        ax.grid(true);
    }
    {
        auto& ax = fig.subplot(1,3,3);
        ax.plot(step_v,cumul_v,"m-");
        ax.set_title("Cumulative Iteration Savings");
        ax.set_xlabel("Control step k");
        ax.set_ylabel("Cumulative iterations saved");
        ax.grid(true);
    }
    fig.savefig(out);
    std::cout << "  -> " << out << "\n";
}

// ══════════════════════════════════════════════════════════════════════════════
//   §6  DATA EXPORT
// ══════════════════════════════════════════════════════════════════════════════

void write_csv(const std::vector<BenchRow>& rows, const std::string& path) {
    std::ofstream f(path);
    if (!f) { std::cerr << "ERROR: cannot write " << path << "\n"; return; }
    f << "scenario,solver,x0_pos,x0_vel,u0,u0_ref,u_l2_err,cost_rel_err,"
         "iters,converged,n_active,t_min_us,t_median_us,t_p95_us,t_max_us,t_std_us\n";
    for (const auto& r : rows)
        f << r.scenario<<","<<r.solver<<","
          <<r.x0_pos<<","<<r.x0_vel<<","
          <<r.u0<<","<<r.u0_ref<<","
          <<r.u_l2_err<<","<<r.cost_rel_err<<","
          <<r.iters<<","<<(int)r.converged<<","
          <<r.n_active<<","
          <<r.t_min_us<<","<<r.t_median_us<<","
          <<r.t_p95_us<<","<<r.t_max_us<<","<<r.t_std_us<<"\n";
    std::cout << "  -> " << path << "  (" << rows.size() << " rows)\n";
}

void write_summary(const std::vector<BenchRow>& rows, const std::string& path) {
    std::ofstream f(path);
    if (!f) { std::cerr << "ERROR: cannot write " << path << "\n"; return; }
    f << "% Auto-generated by benchmark_qp_solvers.cpp\n"
      << "% DCAS Lab QP Benchmark  N_REPS=" << N_REPS << "  QP_N=" << QP_N << "\n\n";
    f << std::left<<std::setw(16)<<"Solver";
    for (auto& s : {"S1_easy","S2_medium","S3_hard"})
        f<<std::right<<std::setw(26)<<(std::string(s)+" Med/P95/Max(us)");
    f<<"\n"<<std::string(94,'-')<<"\n";
    for (auto& sn : SOLVER_ORDER) {
        f<<std::left<<std::setw(16)<<sn;
        for (auto& sc : {"S1_easy","S2_medium","S3_hard"}) {
            auto tm=collect(rows,sc,sn,[](const BenchRow&r){return r.t_median_us;});
            auto tp=collect(rows,sc,sn,[](const BenchRow&r){return r.t_p95_us;});
            auto tx=collect(rows,sc,sn,[](const BenchRow&r){return r.t_max_us;});
            if (tm.empty()){f<<std::setw(26)<<"N/A";continue;}
            std::ostringstream ss;
            ss<<std::fixed<<std::setprecision(0)
              <<median_v(tm)<<"/"<<median_v(tp)<<"/"
              <<*std::max_element(tx.begin(),tx.end());
            f<<std::right<<std::setw(26)<<ss.str();
        }
        f<<"\n";
    }
    auto tb=collect(rows,"S1_easy","ADMM-cold",[](const BenchRow&r){return r.t_median_us;});
    double base=median_v(tb);
    f<<"\nSpeedups vs ADMM-cold (S1 median):\n";
    for (auto& sn : SOLVER_ORDER) {
        if (sn=="ADMM-cold") continue;
        auto t=collect(rows,"S1_easy",sn,[](const BenchRow&r){return r.t_median_us;});
        if (t.empty()) continue;
        f<<"  "<<std::left<<std::setw(14)<<sn<<": "
         <<std::fixed<<std::setprecision(2)<<median_v(t)/base<<"x\n";
    }
    std::cout<<"  -> "<<path<<"\n";
}

// ══════════════════════════════════════════════════════════════════════════════
//   MAIN
// ══════════════════════════════════════════════════════════════════════════════

int main() {
    const std::string OUT = ".";

    std::cout << "\n╔══════════════════════════════════════════════════════════╗\n"
              << "║      DCAS QP Solver Benchmark — C++ + CppPlot           ║\n"
              << "║      AGV/AMR Embedded MPC · N=10 · TDTU March 2026      ║\n"
              << "╚══════════════════════════════════════════════════════════╝\n\n";

    // Print problem info
    for (auto& sc : make_scenarios()) {
        MPCProblem<QP_N> mpc; mpc.build(sc.Q_diag, sc.R_val);
        std::cout << "  " << sc.name
                  << ": cond(H)≈" << std::fixed<<std::setprecision(1)<<mpc.cond()
                  << "  eig=[" << mpc.eig_min << "," << mpc.eig_max << "]"
                  << "  |u|<=" << sc.u_bound << "\n";
    }

    // Run benchmark
    auto rows = run_benchmark();

    // Generate figures
    std::cout << "\nGenerating CppPlot figures...\n";
    fig_timing     (rows, OUT+"/fig_bench_timing.svg");
    fig_convergence(rows, OUT+"/fig_bench_convergence.svg");
    fig_accuracy   (rows, OUT+"/fig_bench_accuracy.svg");
    fig_tradeoff   (rows, OUT+"/fig_bench_tradeoff.svg");
    fig_cdf        (rows, OUT+"/fig_bench_cdf.svg");
    fig_warmstart  (      OUT+"/fig_bench_warmstart.svg");

    // Export data
    std::cout << "\nExporting data...\n";
    write_csv    (rows, OUT+"/benchmark_results.csv");
    write_summary(rows, OUT+"/benchmark_summary.txt");

    // Final console summary
    std::cout << "\n" << std::string(68,'=') << "\n"
              << "FINAL SUMMARY — Median solve time (µs)  [cold-start]\n"
              << std::string(68,'=') << "\n";
    std::cout << std::left<<std::setw(16)<<"Solver"
              <<std::right<<std::setw(10)<<"S1(µs)"<<std::setw(10)<<"S2(µs)"
              <<std::setw(10)<<"S3(µs)"<<std::setw(10)<<"S1 iters\n"
              <<"  "<<std::string(52,'-')<<"\n";
    for (auto& sn : SOLVER_ORDER) {
        std::cout<<std::left<<std::setw(16)<<sn;
        for (auto& sc : {"S1_easy","S2_medium","S3_hard"}) {
            auto t=collect(rows,sc,sn,[](const BenchRow&r){return r.t_median_us;});
            std::cout<<std::right<<std::fixed<<std::setprecision(1)
                     <<std::setw(10)<<(t.empty()?0.0:median_v(t));
        }
        auto it=collect(rows,"S1_easy",sn,[](const BenchRow&r){return (double)r.iters;});
        std::cout<<std::setw(10)<<(int)(it.empty()?0:median_v(it))<<"\n";
    }

    // WCET
    auto wcet_v=collect(rows,"S3_hard","ADMM-cold",[](const BenchRow&r){return r.t_max_us;});
    if (!wcet_v.empty()) {
        double wcet=*std::max_element(wcet_v.begin(),wcet_v.end());
        std::cout<<"\n  WCET ADMM-cold (S3 worst): "
                 <<std::fixed<<std::setprecision(1)
                 <<wcet<<" µs = "<<wcet/1000.0<<" ms  "
                 <<(wcet<10000?"✓ PASS (< 10ms)":"✗ FAIL (> 10ms)")<<"\n";
    }

    std::cout << "\n✓ All outputs saved to: " << OUT << "/\n\n";
    return 0;
}
