/**
 * @file benchmark_qp_solvers_V7.cpp
 * @brief C++ port of benchmark_qp_solvers.py using CppPlot visualization
 *
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  DCAS Lab QP Solver Benchmark — C++17 + CppPlot                ║
 * ║  Replaces benchmark_qp_solvers.py + plot_qp_results.py         ║
 * ╚══════════════════════════════════════════════════════════════════╝
 *
 * ─────────────────────────────────────────────────────────────────────────
 * FIXES APPLIED V7 (2026-03-09):
 *
 * [FIX J1] write_summary and console summary truncate sub-µs values to "0".
 *   Root cause: std::setprecision(0) rounds 0.16µs → "0" in ostringstream.
 *   Impact: benchmark_summary.txt shows Chol-Direct as "0/0/1" on all scenarios
 *   despite actual median 0.16µs (confirmed in CSV t_median_us column).
 *   Fix: setprecision(2) for Med and P95 columns, setprecision(1) for Max.
 *   Location: write_summary() ostringstream, main() console summary loop.
 *
 * [FIX J2] CG-proj S1 median = 0µs despite nanosecond clock fix [H2].
 *   Root cause: CG-proj converges in 2–3 iterations on S1 Easy (near-
 *   unconstrained, u_bound=2.0). Actual solve time ~100ns/call.
 *   BATCH_SLOW=200 × 100ns = 20µs total batch — borderline for platforms
 *   where clock granularity is 5µs (not 1ms). Result: >50% of 60 reps
 *   return 0ns → median=0µs. P95=5µs (one clock tick / 200 = 25ns → 0? No).
 *   Actually: 5000ns clock tick / 200 batch = 25ns → 0.025µs stored → but
 *   CSV shows P95=5.0µs exactly → clock returning 5µs per rep, not per solve.
 *   This means batch total < clock granularity for some reps on this hardware.
 *
 *   Fix: batch_for(solver, scenario) helper — uses BATCH_FAST (10000) for
 *   PGD and CG-proj when scenario == "S1_easy". BATCH_SLOW (200) retained
 *   for S2/S3 where per-solve time is 100–200µs.
 *   Same fix applied to PGD S1 (same fast-convergence behaviour on easy case).
 *   Location: batch_for() function (replaces hardcoded BATCH_* constants at
 *   all measure() call sites).
 *
 * ─────────────────────────────────────────────────────────────────────────
 * FIXES APPLIED V6 (2026-03-09):
 *
 * [FIX I1] fig_timing: replace stem workaround with native boxplot().
 *   CppPlot now supports boxplot(data, positions, opts) — see boxplot_demo.cpp.
 *   V5 used vertical stem lines (2-point ax.plot per solver) as workaround
 *   because ax.bar() + set_yscale("log") rendered bars invisible [FIX F3d],
 *   and boxplot was not yet available. The stem workaround showed only
 *   median + P95 cap, discarding IQR and outlier distribution.
 *
 *   V6 change:
 *   - BenchRow gains raw_times: std::vector<double> (N_REPS timing samples)
 *   - measure() stores raw times in row.raw_times (no stats lost — min/med/
 *     p95/max/std retained for CSV export and console summary)
 *   - fig_timing() collects raw_times per solver across all x0 for a scenario,
 *     builds std::vector<std::vector<double>> data[6], calls boxplot() once.
 *   - Y-axis: set_yscale("log") retained — boxplot renders correctly on log axis.
 *   - Deadline reference lines (1ms, 10ms) retained via ax.plot().
 *   - [FIX F3d] stem code removed — no longer needed.
 *   - [NOTE in V5 header] "CppPlot cannot produce boxplots" — resolved in V6.
 *
 *   Scientific gain: full timing distribution (Q1–Q3 IQR, whiskers, outliers)
 *   now visible per solver, matching Python fig_timing_boxplot() output.
 *   This closes the parity gap between Python and C++ benchmark figures.
 *
 * ─────────────────────────────────────────────────────────────────────────
 * FIXES APPLIED V5 (2026-03-09):
 *
 * [FIX H1] ADMM-tuned still hits iters=200, converged=0 after V4.
 *   Two root causes identified:
 *
 *   (a) Binary not recompiled: V4 CSV shows iters=200 but p_tuned.max_iter=500
 *       in V4 source. When solver doesn't converge, sol.iterations is set to
 *       params_.max_iter (qp_solver.hpp line 343). iters=200 in CSV means the
 *       running binary still uses V2/V3 code with max_iter=200. Always recompile.
 *
 *   (b) Even with correct binary: ADMM-tuned may not converge within 500 iters
 *       if scenario needs more. Fix: set eps_abs and eps_rel EXPLICITLY (do not
 *       rely on Params{} default alone) and increase max_iter to 1000 as safety.
 *       Also explicitly set check_every=10 (less frequent checking → slightly
 *       fewer wasted convergence-check ops per iter).
 *
 *   Fix: Replace implicit Params{} defaults with fully explicit p_tuned config:
 *     p_tuned.eps_abs   = 1e-4;   // explicit, not default-assumed
 *     p_tuned.eps_rel   = 1e-3;   // explicit
 *     p_tuned.max_iter  = 1000;   // was 500; safety margin for hard scenarios
 *     p_tuned.check_every = 10;   // was 5; reduces check overhead
 *   Location: run_benchmark(), ADMM-tuned Params construction.
 *
 * [FIX H2] ADMM-warm and Chol-Direct still show median=0µs in V4.
 *   Root cause: FpMicros = std::chrono::duration<double, std::micro>.
 *   On many platforms, high_resolution_clock internally uses nanosecond ticks.
 *   The duration_cast to microseconds truncates sub-microsecond intervals to 0
 *   BEFORE the floating-point division by batch. Specifically:
 *     duration<double,micro>(t1-t0).count() / BATCH_FAST
 *   When (t1-t0) < 1µs (which happens when BATCH_FAST=10000 loop runs faster
 *   than expected, e.g. CPU boost), count() returns 0.0 → time = 0.
 *
 *   Actually the cast is not truncating — the issue is that duration<double,micro>
 *   should preserve sub-µs precision. The REAL issue: on Linux with gettimeofday
 *   fallback (some embedded builds), clock resolution is 1ms regardless. The
 *   sub-µs batch total (e.g. 300ns × 10000 = 3ms) IS measurable, but the
 *   per-rep timer fires in 1ms steps → many reps round to 0 after /BATCH_FAST.
 *
 *   Fix: Measure in NANOSECONDS, accumulate as int64, then convert to µs as
 *   double AFTER dividing by batch. This avoids any truncation before division:
 *     int64_t ns = duration_cast<nanoseconds>(t1 - t0).count();
 *     times.push_back(static_cast<double>(ns) / batch / 1000.0);  // → µs
 *   The batch total (e.g. 3,000,000 ns for Chol-Direct) is well above any
 *   platform clock granularity, so the nanosecond count is reliable.
 *   Location: measure() inner timing loop.
 *
 * ─────────────────────────────────────────────────────────────────────────
 *
 * [FIX G1] ADMM-tuned still hits iters=200 (converged=0) in V3.
 *   Root cause: p_tuned was copied from p_cold which has eps_abs=1e-6,
 *   eps_rel=1e-5. With rho=0.5 (clamped), ADMM-tuned needs ~800–1200
 *   iterations to satisfy eps=1e-6 on S1/S2/S3 — far beyond max_iter=500.
 *   The tight tolerance was inherited from p_cold (designed for ADMM-cold
 *   reference quality), but ADMM-tuned only needs "good enough" accuracy
 *   for real-time MPC, not reference quality.
 *   Fix: construct p_tuned from scratch (Params default) rather than
 *   copying p_cold. Default tolerance eps_abs=1e-4, eps_rel=1e-3 matches
 *   the recommended MPC config and converges in ~50–150 iterations.
 *   Location: run_benchmark(), ADMM-tuned Params construction.
 *
 * [FIX G2] ADMM-warm and Chol-Direct still show median=0µs in V3.
 *   Root cause: these solvers complete in ~1–3µs per solve.
 *   TIMING_BATCH=1000 × 1.5µs = 1.5ms total / 1000 = 1.5µs per-solve
 *   estimate. But clock returns values in ~1ms steps, so most reps still
 *   land on 0 after division, making median=0.
 *   Fix: per-solver adaptive batch size, passed as parameter to measure().
 *   - BATCH_FAST = 10000  for Chol-Direct, ADMM-warm  (~0.5–3µs/solve)
 *   - BATCH_MED  =  1000  for ADMM-cold, ADMM-tuned   (~5–50µs/solve)
 *   - BATCH_SLOW =   200  for PGD, CG-proj             (~100–300µs/solve)
 *   Total worst-case timing overhead: PGD S3 ~130µs × 200 × 60 × 20 = 31s
 *   per scenario — acceptable.
 *   Location: measure() gains int batch parameter; all call sites updated.
 *
 * ─────────────────────────────────────────────────────────────────────────
 *
 * [FIX F3a] ADMM-tuned still hits max_iter (iters=200, converged=0) in V2.
 *   Root cause: p_tuned inherits max_iter=200 from p_cold, but with
 *   rho=0.5 (clamped) and eps=1e-6/1e-5, convergence requires ~300–400
 *   iterations on S2/S3. Increasing max_iter is cheap — factorization is
 *   done once; each extra iteration costs only O(N²) chol_solve + O(N) clip.
 *   Fix: p_tuned.max_iter = 500.
 *   Location: run_benchmark(), ADMM-tuned setup.
 *
 * [FIX F3b] ADMM-warm timing = 0µs (median, P95) in V2.
 *   Root cause: in V2, admm_warm.reset() was called ONCE before the outer
 *   measure() call. Inside measure(), the TIMING_BATCH=100 inner loop calls
 *   fn() 100 times without reset — warm state from rep 1 fully converges,
 *   so reps 2..100 terminate in 1–2 iterations (~sub-µs) → median = 0.
 *   This makes ADMM-warm look faster than Chol-Direct, which is physically
 *   impossible for a first solve.
 *   Fix: move admm_warm.reset() INSIDE the lambda passed to measure(), so
 *   every single call to fn() (including each of the TIMING_BATCH inner
 *   reps) starts from a cold state. This measures fair single-solve latency.
 *   Note: fig_warmstart() deliberately keeps warm state across its own
 *   simulation loop — that is correct and unchanged.
 *   Location: run_benchmark(), ADMM-warm measure() call.
 *
 * [FIX F3c] Chol-Direct and CG-proj (S1) still show median=0µs in V2.
 *   Root cause: TIMING_BATCH=100 × ~0.3µs/solve = 30µs total — still
 *   within ~1ms clock granularity on some measurements. With 60 reps,
 *   enough measurements round to 0 that median = 0.
 *   Fix: increase TIMING_BATCH from 100 to 1000. Total timing loop per rep:
 *   ~300µs for Chol-Direct → well above clock granularity.
 *   Side effect: PGD and CG-proj (S3, ~100µs/solve) take 1000×100µs = 100ms
 *   per rep × 60 reps × 20 x0 × 3 scenarios → ~36s extra. Acceptable.
 *   Location: TIMING_BATCH constant.
 *
 * [FIX F3d] fig_timing bars invisible when using ax.bar() + set_yscale("log").
 *   Confirmed CppPlot limitation in V1/V2: ax.bar() renders before log scale
 *   is applied, bars at linear y-values become invisible after scale change.
 *   Fix: replace ax.bar(xs, ys) with per-solver ax.plot() vertical stems
 *   from y_floor=0.3µs to ys[k], using marker-only format (no line between
 *   bars). Each stem is drawn as a 2-point plot({x,x},{floor,val}) so it
 *   appears as a vertical line on the log axis.
 *   Location: fig_timing().
 *
 * ─────────────────────────────────────────────────────────────────────────
 * FIXES APPLIED V2 (2026-03-09):
 *
 * [FIX F1] CRITICAL — Clock resolution: all timing values = 0.
 *   High-resolution clock granularity (~1ms on some systems) is coarser than
 *   actual solve time (~1–50µs). Single-shot measurement returns 0 for most
 *   reps → median/P95 = 0, summary shows "0/0/xxx".
 *   Fix: batch timing — each rep executes fn() TIMING_BATCH times, total
 *   time divided by TIMING_BATCH.
 *   Location: measure() function.
 *
 * [FIX F2] CRITICAL — ADMM-tuned never converges (iters=200, converged=0).
 *   spectral_rho() returns rho << 1 for S2/S3 because lambda_max(H) << 1.
 *   Fix: clamp rho_opt >= 0.5 in spectral_rho().
 *   Location: MPCProblem::spectral_rho().
 *
 * ─────────────────────────────────────────────────────────────────────────
 * FIXES APPLIED V1 (2026-03-09):
 *
 * [FIX B1] CRITICAL — Reference solver rho was hardcoded to 1.0.
 *   For S2 (lambda_max≈0.015) and S3 (lambda_max≈0.006), rho=1.0 overshoots
 *   by 68× and 156× respectively. ADMM cannot converge to tight tolerance,
 *   making u_ref wrong → ALL accuracy metrics (u_l2_err, cost_rel_err) wrong.
 *   Fix: p_ref.rho = rho_opt (spectral heuristic, same as ADMM-tuned).
 *   Location: run_benchmark(), "Reference" solver setup ~line 533.
 *
 * [FIX B2] N_REPS mismatch: C++ used 50, Python used 60.
 *   Timing percentiles (median, P95) are not statistically comparable.
 *   Fix: N_REPS = 60.
 *
 * [FIX B3] Solver name inconsistency: C++ used "DCAS-PGD"/"CG+Proj",
 *   Python used "PGD"/"CG-proj". CSV files cannot be merged.
 *   Fix: renamed to canonical "PGD" and "CG-proj" everywhere
 *   (SOLVER_ORDER, NAME constants, measure() calls, fig_accuracy/cdf).
 *
 * [FIX B4] Eigenvalue estimation used Gershgorin circles (overestimates
 *   lambda_max for near-diagonal H), so rho_opt = spectral_rho() was
 *   inaccurate. Fix: power iteration (40 steps, O(N^2)) for lambda_max;
 *   Gershgorin lower bound retained for conservative lambda_min.
 *
 * [FIX P1] fig_timing Y-axis showed -0.5→0.5 (not µs values).
 *   CppPlot ax.bar() has no set_yscale('log'). With a 350× range
 *   (20µs–7000µs), linear scale made all bars the same invisible height.
 *   Fix: pre-transform ys[i] = log10(t_median_µs) before ax.bar().
 *
 * [FIX P2] fig_timing showed only integer positions 1..6 on X axis,
 *   no solver names. CppPlot has no set_xticklabels().
 *   Fix: one ax.plot({x},{y}, fmt, {label=solver}) dummy point per solver
 *   so ax.legend() reveals solver name for each bar position.
 *
 * [FIX P3] fig_accuracy Y-axis was linear → 1e-14 and 1e-5 looked the same.
 *   CppPlot has no set_yscale('log').
 *   Fix: pre-transform ys[i] = log10(max(err, 1e-14)) before ax.plot().
 *   Threshold line moved from raw 1e-6 to log10 value -6.
 *
 * [FIX P4a] fig_tradeoff both axes were linear (-0.5→0.5).
 *   CppPlot has no set_xscale/yscale.
 *   Fix: pre-transform both axes to log10(µs) / log10(err).
 *
 * [FIX P4b] fig_tradeoff all points were same blue, no labels.
 *   ax.scatter(all_xs, all_ys, {{"c","blue"}}) uses a single call for all
 *   solvers — colour cannot vary and labels are impossible.
 *   Fix: one ax.plot({x},{y}, fmt, {label}) call per solver with unique
 *   marker format: "ro" "bs" "m^" "gD" "cx" "k+" (no '-' = no line).
 *
 * [FIX P5] fig_cdf X-axis was linear → 10ms deadline marker at x=10000
 *   was completely off-screen (data range: 20–7000µs, plot range: 20–500µs).
 *   CppPlot has no set_xscale('log').
 *   Fix: pre-transform t_log[i] = log10(t[i]). Deadline at log10(10000)=4.
 *   Added 1ms reference at log10(1000)=3.
 *
 * [RESOLVED in V6 — FIX I1] CppPlot now supports native boxplot().
 *   V1 note: "CppPlot cannot produce boxplots" — fixed via boxplot_demo.cpp API.
 *   fig_timing() in V6 uses ax.boxplot(data, positions, opts) directly.
 * ─────────────────────────────────────────────────────────────────────────
 *
 * Solvers (matching Python benchmark exactly):
 *   1. ADMM-cold   — ADMMSolver, rho=1.0, cold-start
 *   2. ADMM-warm   — ADMMSolver, rho=1.0, warm-start
 *   3. ADMM-tuned  — ADMMSolver, rho = spectral_rho() (auto-tune)
 *   4. PGD         — Projected Gradient + Barzilai-Borwein
 *   5. CG-proj     — Nonlinear CG with box projection
 *   6. Chol-Direct — u*=-H^{-1}g then clip (suboptimal under constraints)
 *
 * Three scenarios (identical to Python):
 *   S1 Easy:   Q=I,           R=0.1,   |u|≤2.0  cond(H)≈1
 *   S2 Medium: Q=diag(10,1),  R=0.01,  |u|≤1.0  cond(H)≈1.5
 *   S3 Hard:   Q=diag(100,1), R=0.001, |u|≤0.3  cond(H)≈6.2
 *
 * Compile:
 *   g++ -std=c++17 -O2 -I../include benchmark_qp_solvers_V7.cpp -o bench_qp
 *   g++ -std=c++17 -O3 -I../include benchmark_qp_solvers_V7.cpp -o bench_qp  # timing
 *
 * ⚠️  ALWAYS recompile from source — never run a stale binary from a prev version.
 *     If iters=200 for ADMM-tuned, binary is from V2/V3 (max_iter was 200 there).
 *
 * DCAS Lab · L0 Core Control Stack · TDTU — TS. Trí Viễn Vũ — March 2026
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

// ── CppPlot visualization API (confirmed: Figure fig(w,h); fig.subplot();
// fig.savefig()) ──
#include <cppplot/cppplot.hpp>

// ── DCAS ADMM QP solver — the primary solver being benchmarked ──────────────
#include <cppplot/control/qp_solver.hpp>

using namespace cppplot;
using namespace cppplot::control::qp;
using Clock = std::chrono::high_resolution_clock;
// [FIX H2] Use nanoseconds (int64) to avoid precision loss before batch division.
// FpMicros (double µs) was susceptible to sub-µs truncation on platforms where
// clock resolution is 1ms: batch total of e.g. 3ms / BATCH_FAST=10000 = 0.3µs,
// but if measured as integer µs first → 3µs / 10000 = 0 (integer truncation).
// Using ns: 3,000,000 ns / 10000 = 300 ns = 0.3µs — no truncation.
using Nanos = std::chrono::nanoseconds;

// ══════════════════════════════════════════════════════════════════════════════
//   §1  ADDITIONAL SOLVERS
//   Port of Python benchmark solvers not in qp_solver.hpp
// ══════════════════════════════════════════════════════════════════════════════

/** Unified result type for all solvers */
template <int N> struct BenchResult {
  Vec<N> x;
  double cost = 0.0;
  int iters = 0;
  bool converged = false;
};

// ── §1.1  Projected Gradient Descent + Barzilai-Borwein ──────────────────────
/**
 * Exact port of Python PGDSolver AND mpc.hpp::solveConstrained().
 * grad = H*U + g
 * U_new = clip(U - α*grad, lb, ub)
 * α_BB  = ‖ΔU‖² / (ΔU'·H·ΔU)
 */
template <int N> struct PGDSolver {
  static constexpr const char *NAME =
      "PGD"; // [FIX B3] was "DCAS-PGD" — canonical name matches Python
  int max_iter = 1000;
  double tol = 1e-8; // gradient norm (matches Python)

  BenchResult<N> solve(const Mat<N> &H, const Vec<N> &g, const Vec<N> &lb,
                       const Vec<N> &ub) const {
    BenchResult<N> r{};
    Vec<N> U; // zero-init
    double alpha = 0.01;

    for (int k = 0; k < max_iter; ++k) {
      Vec<N> grad = H.matvec(U) + g;
      if (grad.norm2() < tol) {
        r.converged = true;
        r.iters = k;
        break;
      }

      Vec<N> U_new;
      for (int i = 0; i < N; ++i)
        U_new[i] = std::max(lb[i], std::min(ub[i], U[i] - alpha * grad[i]));

      // Barzilai-Borwein adaptive step
      Vec<N> dU = U_new - U;
      double dU2 = dU.dot(dU);
      if (dU2 > 1e-12) {
        Vec<N> HdU = H.matvec(dU);
        double dUHdU = dU.dot(HdU);
        if (std::abs(dUHdU) > 1e-12)
          alpha = std::max(0.001, std::min(1.0, dU2 / dUHdU));
      }
      U = U_new;
      r.iters = k + 1;
    }
    r.x = U;
    Vec<N> HU = H.matvec(U);
    for (int i = 0; i < N; ++i)
      r.cost += 0.5 * HU[i] * U[i] + g[i] * U[i];
    return r;
  }
};

// ── §1.2  Nonlinear CG + Box Projection ──────────────────────────────────────
/**
 * Port of Python CGProjSolver (Polak-Ribière β, box projection).
 * Convergence not guaranteed for constrained QP without line search.
 * Note: Python comment says "Frank-Wolfe style" but algorithm is NL-CG.
 */
template <int N> struct CGProjSolver {
  static constexpr const char *NAME =
      "CG-proj"; // [FIX B3] was "CG+Proj" — canonical name matches Python
  int max_iter = 500;
  double tol = 1e-8;

  BenchResult<N> solve(const Mat<N> &H, const Vec<N> &g, const Vec<N> &lb,
                       const Vec<N> &ub) const {
    BenchResult<N> r{};
    Vec<N> x;
    Vec<N> grad = H.matvec(x) + g;
    Vec<N> p;
    for (int i = 0; i < N; ++i)
      p[i] = -grad[i];

    for (int k = 0; k < max_iter; ++k) {
      Vec<N> Hp = H.matvec(p);
      double pHp = p.dot(Hp);
      if (std::abs(pHp) < 1e-15) {
        r.iters = k;
        r.converged = true;
        break;
      }

      double alpha = -(grad.dot(p)) / pHp;
      Vec<N> x_new;
      for (int i = 0; i < N; ++i)
        x_new[i] = std::max(lb[i], std::min(ub[i], x[i] + alpha * p[i]));

      Vec<N> grad_new = H.matvec(x_new) + g;
      if (grad_new.norm2() < tol) {
        x = x_new;
        r.iters = k + 1;
        r.converged = true;
        break;
      }
      double g2 = grad.dot(grad);
      double g2new = grad_new.dot(grad_new);
      double beta = (g2 > 1e-15) ? g2new / g2 : 0.0;
      for (int i = 0; i < N; ++i)
        p[i] = -grad_new[i] + beta * p[i];
      x = x_new;
      grad = grad_new;
      r.iters = k + 1;
    }
    r.x = x;
    Vec<N> Hx = H.matvec(x);
    for (int i = 0; i < N; ++i)
      r.cost += 0.5 * Hx[i] * x[i] + g[i] * x[i];
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
template <int N> struct CholDirectSolver {
  static constexpr const char *NAME = "Chol-Direct";
  Mat<N> L;
  bool ready = false;

  bool setup(const Mat<N> &H) { return (ready = cholesky(L, H)); }

  BenchResult<N> solve(const Vec<N> &g, const Vec<N> &lb,
                       const Vec<N> &ub) const {
    BenchResult<N> r{};
    r.iters = 1;
    r.converged = true;
    if (!ready)
      return r;
    Vec<N> neg_g;
    for (int i = 0; i < N; ++i)
      neg_g[i] = -g[i];
    Vec<N> u_unc = chol_solve(L, neg_g);
    for (int i = 0; i < N; ++i)
      r.x[i] = std::max(lb[i], std::min(ub[i], u_unc[i]));
    return r; // cost filled later using known H
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
template <int QN> struct MPCProblem {
  static constexpr int NX = 2;
  static constexpr int HOR = QN; // horizon = 10
  static constexpr double DT = 0.01;

  Mat<QN> H;         // QP Hessian — symmetric PSD
  double Ft[QN][NX]; // F' = Γ'Q̄Φ;  g = F'·x₀
  double eig_min = 0.0,
         eig_max = 0.0; // accurate eigenvalue estimates (power iteration)

  /**
   * @param Q_diag  Diagonal entries of state weight Q ∈ R^{NX×NX}
   * @param R_val   Scalar input weight R
   */
  void build(const double Q_diag[NX], double R_val) {
    const double dt = DT;
    double A[NX][NX] = {{1.0, dt}, {0.0, 1.0}};
    double B[NX][1] = {{0.5 * dt * dt}, {dt}};

    // ── Build Φ (2N×2) and Γ (2N×N) ──────────────────────────────────
    double Phi[2 * QN][NX] = {};
    double Gam[2 * QN][QN] = {};
    double Ak[NX][NX] = {{1.0, dt}, {0.0, 1.0}}; // A^1

    for (int k = 0; k < HOR; ++k) {
      // Φ row-block k = A^{k+1}
      for (int r = 0; r < NX; ++r)
        for (int c = 0; c < NX; ++c)
          Phi[NX * k + r][c] = Ak[r][c];

      // Γ columns j=k..0: Γ(2k:2k+2, j) = A^{k-j}·B
      double Apow[NX][NX] = {{1, 0}, {0, 1}}; // A^0 = I
      for (int j = k; j >= 0; --j) {
        for (int r = 0; r < NX; ++r) {
          double v = 0;
          for (int p = 0; p < NX; ++p)
            v += Apow[r][p] * B[p][0];
          Gam[NX * k + r][j] = v;
        }
        if (j > 0) { // Advance Apow = A·Apow
          double tmp[NX][NX] = {};
          for (int r = 0; r < NX; ++r)
            for (int c = 0; c < NX; ++c)
              for (int p = 0; p < NX; ++p)
                tmp[r][c] += A[r][p] * Apow[p][c];
          for (int r = 0; r < NX; ++r)
            for (int c = 0; c < NX; ++c)
              Apow[r][c] = tmp[r][c];
        }
      }
      // Advance Ak = A·Ak for next iteration
      if (k < HOR - 1) {
        double tmp[NX][NX] = {};
        for (int r = 0; r < NX; ++r)
          for (int c = 0; c < NX; ++c)
            for (int p = 0; p < NX; ++p)
              tmp[r][c] += A[r][p] * Ak[p][c];
        for (int r = 0; r < NX; ++r)
          for (int c = 0; c < NX; ++c)
            Ak[r][c] = tmp[r][c];
      }
    }

    // ── H = Γ'·Q̄·Γ + R̄  (Q̄ = blkdiag(Q,...), diagonal Q case) ──────
    for (int i = 0; i < HOR; ++i)
      for (int j = 0; j < HOR; ++j) {
        double s = 0.0;
        for (int r = 0; r < 2 * HOR; ++r)
          s += Gam[r][i] * Q_diag[r % NX] * Gam[r][j];
        H(i, j) = s + (i == j ? R_val : 0.0);
      }

    // ── F' = Γ'·Q̄·Φ ∈ R^{N×2} ──────────────────────────────────────
    for (int i = 0; i < HOR; ++i)
      for (int c = 0; c < NX; ++c) {
        double s = 0.0;
        for (int r = 0; r < 2 * HOR; ++r)
          s += Gam[r][i] * Q_diag[r % NX] * Phi[r][c];
        Ft[i][c] = s;
      }

    // ── Eigenvalue estimation ─────────────────────────────────────────
    // [FIX B4] Gershgorin circles significantly overestimate lambda_max for
    // our near-diagonal H (e.g. H≈diag(R,...) + small off-diag terms).
    // Power iteration gives accurate lambda_max in ~30 steps, O(N^2) each.
    // Accurate lambda_max → accurate rho_opt → ADMM-tuned actually converges
    // fast.
    //
    // lambda_max via power iteration:
    {
      double v[QN] = {};
      v[0] = 1.0;
      double lam = 0.0;
      for (int it = 0; it < 40; ++it) {
        double w[QN] = {};
        for (int i = 0; i < HOR; ++i)
          for (int j = 0; j < HOR; ++j)
            w[i] += H(i, j) * v[j];
        double nm = 0.0;
        for (int i = 0; i < HOR; ++i)
          nm += w[i] * w[i];
        nm = std::sqrt(nm);
        if (nm < 1e-14)
          break;
        lam = 0.0;
        for (int i = 0; i < HOR; ++i) {
          v[i] = w[i] / nm;
          lam += v[i] * w[i] / nm;
        }
      }
      eig_max = lam;
    }
    // lambda_min via Gershgorin lower bound (conservative but safe for SPD H):
    {
      double lo = H(0, 0);
      for (int i = 0; i < HOR; ++i) {
        double off = 0.0;
        for (int j = 0; j < HOR; ++j)
          if (j != i)
            off += std::abs(H(i, j));
        lo = std::min(lo, std::max(0.0, H(i, i) - off));
      }
      eig_min = std::max(lo, 1e-12);
    }
  }

  /** g = F'·x₀ */
  Vec<QN> g_from_state(double pos, double vel) const {
    Vec<QN> g;
    for (int i = 0; i < HOR; ++i)
      g[i] = Ft[i][0] * pos + Ft[i][1] * vel;
    return g;
  }

  /** QP objective: 0.5·x'·H·x + g'·x */
  double cost(const Vec<QN> &x, const Vec<QN> &g_vec) const {
    Vec<QN> Hx = H.matvec(x);
    double c = 0.0;
    for (int i = 0; i < HOR; ++i)
      c += 0.5 * Hx[i] * x[i] + g_vec[i] * x[i];
    return c;
  }

  /** Spectral heuristic rho (matches Python ADMM-tuned)
   *
   * [FIX F2] Original formula returns rho << 1 when lambda_max(H) << 1
   * (e.g. S2: rho≈0.15, S3: rho≈0.06). ADMM with rho<<1 has very weak
   * penalty → slow convergence → fails within max_iter budget.
   * Clamp from below at 0.5 ensures meaningful penalty on all scenarios.
   */
  double spectral_rho() const {
    double r =
        std::max(10.0 * eig_max, std::sqrt(std::max(eig_min, 1e-10) * eig_max));
    r = std::max(r, 0.5); // [FIX F2] clamp: rho >= 0.5 always
    return std::min(r, 1e6);
  }

  /** Condition number estimate */
  double cond() const { return (eig_min > 1e-14) ? eig_max / eig_min : 1e9; }
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
  int iters;
  bool converged;
  int n_active; // active constraints in reference solution
  double t_min_us, t_median_us, t_p95_us, t_max_us, t_std_us;
  // [FIX I1] Raw timing samples for boxplot — N_REPS values (µs)
  std::vector<double> raw_times;
};

// ── Statistics
// ────────────────────────────────────────────────────────────────

static double pct(std::vector<double> v, double p) {
  if (v.empty())
    return 0.0;
  std::sort(v.begin(), v.end());
  double idx = p / 100.0 * (static_cast<int>(v.size()) - 1);
  int lo = static_cast<int>(idx), hi = std::min(lo + 1, (int)v.size() - 1);
  return v[lo] * (1.0 - (idx - lo)) + v[hi] * (idx - lo);
}
static double median_v(std::vector<double> v) { return pct(v, 50.0); }
static double mean_v(const std::vector<double> &v) {
  return v.empty() ? 0.0 : std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}
static double stddev_v(const std::vector<double> &v) {
  double m = mean_v(v), s = 0.0;
  for (auto x : v)
    s += (x - m) * (x - m);
  return (v.size() > 1) ? std::sqrt(s / (v.size() - 1)) : 0.0;
}
template <typename Fn>
static std::vector<double> collect(const std::vector<BenchRow> &rows,
                                   const std::string &sc, const std::string &sv,
                                   Fn fn) {
  std::vector<double> out;
  for (const auto &r : rows)
    if (r.scenario == sc && r.solver == sv)
      out.push_back(fn(r));
  return out;
}

// ══════════════════════════════════════════════════════════════════════════════
//   §4  BENCHMARK ENGINE
// ══════════════════════════════════════════════════════════════════════════════

static constexpr int QP_N = 10;
static constexpr int N_REPS =
    60; // [FIX B2] was 50 — changed to 60 to match Python benchmark
static constexpr int N_WARMUP = 10; // discarded warmup reps

// [FIX B3] Canonical solver names — must match Python benchmark for CSV merge.
// Python uses: "PGD", "CG-proj". Old C++ used: "DCAS-PGD", "CG+Proj".
static const std::vector<std::string> SOLVER_ORDER = {
    "ADMM-cold", "ADMM-warm", "ADMM-tuned", "PGD", "CG-proj", "Chol-Direct"};

struct Scenario {
  std::string name, label;
  double Q_diag[2];
  double R_val;
  double u_bound;
  std::vector<std::pair<double, double>> x0_list;
};

static std::vector<Scenario> make_scenarios() {
  std::vector<std::pair<double, double>> x0_s1 = {
      {-5, 0}, {-4, 0},  {-3, 0}, {-2, 0},   {-1, 0},   {-0.5, 0},  {0.5, 0},
      {1, 0},  {2, 0},   {3, 0},  {4, 0},    {4.9, 0},  {-3, .3},   {-3, -.3},
      {2, .2}, {2, -.2}, {1, .5}, {-1, -.5}, {4.5, .1}, {-4.5, -.1}};
  std::vector<std::pair<double, double>> x0_s2 = {
      {-2, 0},   {-1.5, 0}, {-1, 0},     {-0.5, 0}, {-0.2, 0},
      {0.2, 0},  {0.5, 0},  {1, 0},      {1.5, 0},  {2, 0},
      {-1, .1},  {-1, -.1}, {1, .1},     {1, -.1},  {.5, .3},
      {.5, -.3}, {1.5, .2}, {-1.5, -.2}, {2, .1},   {-2, -.1}};
  std::vector<std::pair<double, double>> x0_s3 = {
      {-3, 0},   {-2, 0},    {-1, 0},    {-0.5, 0}, {-0.2, 0},
      {0.2, 0},  {0.5, 0},   {1, 0},     {2, 0},    {3, 0},
      {-1, .05}, {-1, -.05}, {1, .05},   {1, -.05}, {.5, .1},
      {.5, -.1}, {2, .05},   {-2, -.05}, {3, .02},  {-3, -.02}};
  return {
      {"S1_easy",
       "S1: Easy  (Q=I,        R=0.1,   |u|≤2.0)",
       {1.0, 1.0},
       0.1,
       2.0,
       x0_s1},
      {"S2_medium",
       "S2: Medium (Q=diag(10,1),R=0.01,  |u|≤1.0)",
       {10.0, 1.0},
       0.01,
       1.0,
       x0_s2},
      {"S3_hard",
       "S3: Hard  (Q=diag(100,1),R=0.001,|u|≤0.3)",
       {100.0, 1.0},
       0.001,
       0.3,
       x0_s3},
  };
}

// ── Adaptive batch sizes (per-solver, per-scenario) ──────────────────────────
// [FIX G2] Per-solver batch ensures batch total >> clock granularity for all
// solver speed classes. Three tiers (V6):
//   BATCH_FAST = 10000 — Chol-Direct (~0.15µs), ADMM-warm (~18µs)
//   BATCH_MED  =  1000 — ADMM-cold, ADMM-tuned (~18–80µs)
//   BATCH_SLOW =   200 — PGD, CG-proj on S2/S3 (~100–220µs)
//
// [FIX J2] CG-proj and PGD converge in 2–3 iterations on S1 Easy (~100ns/call).
//   BATCH_SLOW=200 × 100ns = 20µs total — borderline for nanosecond clock on
//   some platforms, causing >50% of reps to return 0ns → median=0µs.
//   Pattern in CSV: CG-proj S1 t_median=0.0 for all 20 x0, P95=5µs (clock tick).
//
//   Root cause: S1 is near-unconstrained (u_bound=2.0, optimal u* rarely hits
//   bounds), so CG converges in 1 conjugate-direction step. Solve time ~100ns
//   regardless of x0. BATCH_SLOW was sized for S3 where CG takes 500 iters.
//
//   Fix: use BATCH_FAST (10000) for PGD and CG-proj on S1_easy.
//   BATCH_SLOW (200) retained for S2/S3 where per-solve time is 100–200µs.
//   Helper batch_for(solver, scenario) encodes this mapping.
//
//   Timing budget check after fix:
//     CG-proj  S1: 10000 × 100ns = 1ms/rep × 60 × 20 = 1.2s   ✓
//     PGD      S1: 10000 × 100ns = 1ms/rep × 60 × 20 = 1.2s   ✓
//     CG-proj  S3: 200   × 130µs = 26ms/rep× 60 × 20 = 31s    ✓ (unchanged)
static constexpr int BATCH_FAST = 10000;
static constexpr int BATCH_MED  =  1000;
static constexpr int BATCH_SLOW =   200;

// [FIX J2] Return appropriate batch size per solver × scenario combination.
static int batch_for(const std::string &solver, const std::string &scenario) {
  // Fast solvers always use BATCH_FAST
  if (solver == "Chol-Direct" || solver == "ADMM-warm")
    return BATCH_FAST;
  // Medium solvers
  if (solver == "ADMM-cold" || solver == "ADMM-tuned")
    return BATCH_MED;
  // PGD and CG-proj: fast on S1 (2–3 iters), slow on S2/S3 (100–500 iters)
  // Use BATCH_FAST on S1 to avoid sub-clock-tick batch totals
  if (scenario == "S1_easy")
    return BATCH_FAST;
  return BATCH_SLOW;
}

/** Measure one solver N_REPS times, compute timing stats + accuracy.
 *
 * [FIX F1+G2] Adaptive batch timing: each rep executes fn() `batch` times
 * and divides total elapsed by `batch`. Caller passes the appropriate
 * batch size (BATCH_FAST / BATCH_MED / BATCH_SLOW) per solver category.
 */
template <typename SolveFn>
static BenchRow measure(const std::string &sc, const std::string &sv,
                        double pos, double vel, const Vec<QP_N> &g_vec,
                        const Vec<QP_N> &lb, const Vec<QP_N> &ub,
                        const Vec<QP_N> &u_ref, double cost_ref, int n_active,
                        const MPCProblem<QP_N> &mpc, int batch, SolveFn fn) {
  std::vector<double> times;
  times.reserve(N_REPS);
  BenchResult<QP_N> sol{};

  // [FIX F1+G2] Run `batch` solves per rep, divide total time by batch
  for (int rep = 0; rep < N_REPS; ++rep) {
    auto t0 = Clock::now();
    for (int b = 0; b < batch; ++b)
      sol = fn(); // last result kept for accuracy check
    auto t1 = Clock::now();
    // [FIX H2] Measure in nanoseconds (int64), divide by batch, convert to µs.
    // This preserves sub-µs resolution for fast solvers like Chol-Direct (~300ns).
    int64_t ns = std::chrono::duration_cast<Nanos>(t1 - t0).count();
    times.push_back(static_cast<double>(ns) / batch / 1000.0);
  }

  double u_err = 0.0;
  for (int i = 0; i < QP_N; ++i)
    u_err += (sol.x[i] - u_ref[i]) * (sol.x[i] - u_ref[i]);
  u_err = std::sqrt(u_err);
  double c = mpc.cost(sol.x, g_vec);
  double c_err = (std::abs(cost_ref) > 1e-12)
                     ? std::abs(c - cost_ref) / std::abs(cost_ref)
                     : 0.0;

  BenchRow r;
  r.scenario = sc;
  r.solver = sv;
  r.x0_pos = pos;
  r.x0_vel = vel;
  r.u0 = sol.x[0];
  r.u0_ref = u_ref[0];
  r.u_l2_err = u_err;
  r.cost_rel_err = c_err;
  r.iters = sol.iters;
  r.converged = sol.converged;
  r.n_active = n_active;
  r.t_min_us = *std::min_element(times.begin(), times.end());
  r.t_median_us = median_v(times);
  r.t_p95_us = pct(times, 95.0);
  r.t_max_us = *std::max_element(times.begin(), times.end());
  r.t_std_us = stddev_v(times);
  r.raw_times = times; // [FIX I1] store for boxplot in fig_timing()
  return r;
}

std::vector<BenchRow> run_benchmark() {
  std::vector<BenchRow> all_rows;
  auto scenarios = make_scenarios();

  std::cout
      << "\n╔══════════════════════════════════════════════════════════════╗\n"
      << "║  DCAS Lab QP Solver Benchmark — C++ with CppPlot  [V5]      ║\n"
      << "║  TDTU · L0 Core Control Stack · March 2026                  ║\n"
      << "╚══════════════════════════════════════════════════════════════╝\n"
      << "  N_REPS=" << N_REPS << "  N_WARMUP=" << N_WARMUP
      << "  QP_N=" << QP_N
      << "  BATCH_FAST=" << BATCH_FAST
      << "  BATCH_MED=" << BATCH_MED
      << "  BATCH_SLOW=" << BATCH_SLOW << "\n\n";

  for (auto &sc : scenarios) {
    MPCProblem<QP_N> mpc;
    mpc.build(sc.Q_diag, sc.R_val);
    double rho_opt = mpc.spectral_rho();

    std::cout
        << "══════════════════════════════════════════════════════════════\n"
        << "Scenario: " << sc.name << "\n  " << sc.label << "\n"
        << "  cond(H)≈" << std::fixed << std::setprecision(1) << mpc.cond()
        << "  eig=[" << mpc.eig_min << ", " << mpc.eig_max << "]"
        << "  rho_opt=" << rho_opt
        << "  [F2: 10*eig_max=" << std::setprecision(4) << 10.0*mpc.eig_max
        << ", clamped_to>=0.5]\n\n";

    Vec<QP_N> lb, ub;
    for (int i = 0; i < QP_N; ++i) {
      lb[i] = -sc.u_bound;
      ub[i] = sc.u_bound;
    }

    // ── Setup solvers (Cholesky factorization done ONCE per scenario) ──
    Params p_cold;
    p_cold.rho = 1.0;
    p_cold.max_iter = 200;
    p_cold.eps_abs = 1e-6;
    p_cold.eps_rel = 1e-5;
    p_cold.warm_start = false;
    ADMMSolver<QP_N> admm_cold;
    admm_cold.setup(mpc.H, p_cold);

    Params p_warm = p_cold;
    p_warm.warm_start = true;
    ADMMSolver<QP_N> admm_warm;
    admm_warm.setup(mpc.H, p_warm);

    // [FIX H1] All p_tuned fields set EXPLICITLY — do not rely on Params{} defaults.
    // Root causes of ADMM-tuned still failing in V4:
    //   (a) Stale binary: if iters=200 in CSV, binary was from V2/V3. Recompile!
    //   (b) max_iter=500 may still be insufficient for S2/S3 hard cases.
    // Fix: max_iter=1000 (safety margin), eps explicitly set, check_every=10.
    // With rho_opt >= 0.5 and eps=1e-4/1e-3, convergence in <200 iters expected
    // for all 3 scenarios. max_iter=1000 ensures no edge case exceeds budget.
    Params p_tuned;
    p_tuned.rho         = rho_opt; // spectral heuristic, clamped >= 0.5  [FIX F2]
    p_tuned.eps_abs     = 1e-4;    // [FIX H1] explicit — suitable for MPC (not reference quality)
    p_tuned.eps_rel     = 1e-3;    // [FIX H1] explicit
    p_tuned.max_iter    = 1000;    // [FIX H1] was 500 — safety margin for hard S2/S3
    p_tuned.warm_start  = false;   // cold-start for fair benchmark comparison
    p_tuned.check_every = 10;      // [FIX H1] was 5 — halves convergence-check overhead
    ADMMSolver<QP_N> admm_tuned;
    admm_tuned.setup(mpc.H, p_tuned);

    // Reference: extremely tight tolerance (ground truth u*)
    // [FIX B1] CRITICAL: was rho=1.0 — fails to converge on S2/S3 where
    // lambda_max(H) << 1 (S2: ~0.015, S3: ~0.006).  rho=1.0 overshoots by
    // 68x–156x, making ADMM stall and u_ref WRONG → all accuracy data invalid.
    // Fix: use spectral rho_opt so reference solver always converges.
    Params p_ref;
    p_ref.rho = rho_opt;
    p_ref.max_iter = 10000;
    p_ref.eps_abs = 1e-10;
    p_ref.eps_rel = 1e-9;
    p_ref.warm_start = false;
    ADMMSolver<QP_N> admm_ref;
    admm_ref.setup(mpc.H, p_ref);

    PGDSolver<QP_N> pgd;
    CGProjSolver<QP_N> cg;
    CholDirectSolver<QP_N> chol_d;
    chol_d.setup(mpc.H);

    // CPU warmup (heat cache and branch predictor, discard results)
    Vec<QP_N> g_dum = mpc.g_from_state(1.0, 0.0);
    for (int w = 0; w < N_WARMUP; ++w) {
      admm_cold.reset();
      admm_cold.solve(g_dum, lb, ub);
      admm_warm.solve(g_dum, lb, ub);
      pgd.solve(mpc.H, g_dum, lb, ub);
      cg.solve(mpc.H, g_dum, lb, ub);
    }

    // ── Per-x₀ benchmark loop ──────────────────────────────────────────
    for (const auto &x0_pair : sc.x0_list) {
      double pos = x0_pair.first;
      double vel = x0_pair.second;
      Vec<QP_N> g_vec = mpc.g_from_state(pos, vel);

      // Ground-truth reference solution
      admm_ref.reset();
      auto sol_ref = admm_ref.solve(g_vec, lb, ub);
      Vec<QP_N> u_ref = sol_ref.x;
      double cost_ref = mpc.cost(u_ref, g_vec);
      int n_act = 0;
      for (int i = 0; i < QP_N; ++i)
        if (u_ref[i] <= lb[i] + 1e-6 || u_ref[i] >= ub[i] - 1e-6)
          ++n_act;

      // Helper: Solution<N> → BenchResult<N>
      auto a2b = [](const Solution<QP_N> &s) -> BenchResult<QP_N> {
        BenchResult<QP_N> r;
        r.x = s.x;
        r.cost = s.cost;
        r.iters = s.iterations;
        r.converged = s.converged;
        return r;
      };

      // ADMM cold
      all_rows.push_back(measure(sc.name, "ADMM-cold", pos, vel, g_vec, lb, ub,
                                 u_ref, cost_ref, n_act, mpc,
                                 batch_for("ADMM-cold", sc.name), [&] {
                                   admm_cold.reset();
                                   return a2b(admm_cold.solve(g_vec, lb, ub));
                                 }));

      // ADMM warm — reset inside lambda for fair single-solve timing [FIX F3b]
      all_rows.push_back(measure(
          sc.name, "ADMM-warm", pos, vel, g_vec, lb, ub, u_ref, cost_ref, n_act,
          mpc, batch_for("ADMM-warm", sc.name), [&] {
            admm_warm.reset();
            return a2b(admm_warm.solve(g_vec, lb, ub));
          }));

      // ADMM tuned
      all_rows.push_back(measure(sc.name, "ADMM-tuned", pos, vel, g_vec, lb, ub,
                                 u_ref, cost_ref, n_act, mpc,
                                 batch_for("ADMM-tuned", sc.name), [&] {
                                   admm_tuned.reset();
                                   return a2b(admm_tuned.solve(g_vec, lb, ub));
                                 }));

      // PGD — [FIX J2] batch_for() uses BATCH_FAST on S1 (2–3 iters ~100ns)
      all_rows.push_back(
          measure(sc.name, "PGD", pos, vel, g_vec, lb, ub, u_ref, cost_ref,
                  n_act, mpc, batch_for("PGD", sc.name),
                  [&] { return pgd.solve(mpc.H, g_vec, lb, ub); }));

      // CG-proj — [FIX J2] same rationale as PGD
      all_rows.push_back(
          measure(sc.name, "CG-proj", pos, vel, g_vec, lb, ub, u_ref, cost_ref,
                  n_act, mpc, batch_for("CG-proj", sc.name),
                  [&] { return cg.solve(mpc.H, g_vec, lb, ub); }));

      // Chol-Direct
      all_rows.push_back(measure(sc.name, "Chol-Direct", pos, vel, g_vec, lb,
                                 ub, u_ref, cost_ref, n_act, mpc,
                                 batch_for("Chol-Direct", sc.name),
                                 [&] { return chol_d.solve(g_vec, lb, ub); }));
    }

    // Per-scenario summary
    std::cout << "  " << std::left << std::setw(14) << "Solver" << std::right
              << std::setw(9) << "Med(µs)" << std::setw(9) << "P95(µs)"
              << std::setw(9) << "Max(µs)" << std::setw(8) << "Iters"
              << std::setw(12) << "||u-u*||₂" << std::setw(9) << "Conv%\n";
    std::cout << "  " << std::string(68, '-') << "\n";
    for (auto &sn : SOLVER_ORDER) {
      auto tm = collect(all_rows, sc.name, sn,
                        [](const BenchRow &r) { return r.t_median_us; });
      auto tp = collect(all_rows, sc.name, sn,
                        [](const BenchRow &r) { return r.t_p95_us; });
      auto tx = collect(all_rows, sc.name, sn,
                        [](const BenchRow &r) { return r.t_max_us; });
      auto it = collect(all_rows, sc.name, sn,
                        [](const BenchRow &r) { return (double)r.iters; });
      auto er = collect(all_rows, sc.name, sn,
                        [](const BenchRow &r) { return r.u_l2_err; });
      int tot = (int)tm.size();
      if (!tot)
        continue;
      int cv = 0;
      for (auto &r : all_rows)
        if (r.scenario == sc.name && r.solver == sn && r.converged)
          ++cv;
      std::cout << "  " << std::left << std::setw(14) << sn << std::right
                << std::fixed << std::setprecision(1) << std::setw(9)
                << median_v(tm) << std::setw(9) << median_v(tp) << std::setw(9)
                << *std::max_element(tx.begin(), tx.end()) << std::setw(8)
                << (int)median_v(it) << "  " << std::scientific
                << std::setprecision(2) << std::setw(10) << median_v(er)
                << std::fixed << std::setprecision(1) << std::setw(9)
                << 100.0 * cv / tot << "%\n";
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
//     ax.set_title/xlabel/ylabel(str);  ax.set_xlim/ylim(a,b);
//     ax.set_xscale/yscale("log");  ax.grid(bool);  ax.legend(bool);
//     ax.boxplot(data, positions, opts);  // [V6] native boxplot support
// ══════════════════════════════════════════════════════════════════════════════

static const std::string SC_NAMES[] = {"S1_easy", "S2_medium", "S3_hard"};
static const std::string SC_SHORT[] = {"S1: Easy", "S2: Medium", "S3: Hard"};

// ── Fig 1: Timing distribution — native boxplot  ─────────────────────────────
/**
 * [FIX I1] Replaces stem-line workaround from V5 [FIX F3d].
 *
 * V5 used 2-point ax.plot() stems because:
 *   (a) ax.bar() + set_yscale("log") rendered bars invisible [FIX F3d], and
 *   (b) CppPlot had no boxplot support at that time.
 * The stem approach showed only median + P95 cap — discarding IQR, whiskers,
 * and outlier distribution that matter for WCET analysis.
 *
 * V6: CppPlot now has native boxplot(data, positions, opts) API.
 * - data[k] = all raw timing samples across all x0 for solver k in this scenario
 *             (N_REPS × n_x0 values — 60 × 20 = 1200 samples per solver)
 * - positions = {1.0, 2.0, ..., 6.0} — one box per solver
 * - yscale("log") applied after boxplot — renders correctly
 * - Deadline reference lines (1ms, 10ms) retained via ax.plot()
 * - Legend: one dummy plot per solver for colour identification
 *
 * Scientific gain: full distribution visible — Q1/Q3 IQR shows solver
 * variance, whiskers reveal tail behaviour, outliers flag jitter events.
 * This matches Python fig_timing_boxplot() and closes the parity gap.
 */
void fig_timing(const std::vector<BenchRow> &rows, const std::string &out) {
  Figure fig(1400, 540);

  // Per-solver colours for boxplot fill and legend markers
  const std::vector<std::string> sv_colors = {
      "red",     // ADMM-cold
      "blue",    // ADMM-warm
      "magenta", // ADMM-tuned
      "green",   // PGD
      "cyan",    // CG-proj
      "black"    // Chol-Direct
  };
  // Legend dummy formats (line style so legend shows coloured line)
  const std::vector<std::string> legend_fmts = {
      "r-", "b-", "m-", "g-", "c-", "k-"
  };

  for (int si = 0; si < 3; ++si) {
    auto &ax = fig.subplot(1, 3, si + 1);
    const std::string &sc = SC_NAMES[si];

    // [FIX I1] Collect raw_times across all x0 per solver → one box each
    std::vector<std::vector<double>> box_data;
    std::vector<double> positions;
    double ymax = 0.5;

    for (int k = 0; k < (int)SOLVER_ORDER.size(); ++k) {
      std::vector<double> all_times;
      for (const auto &r : rows) {
        if (r.scenario == sc && r.solver == SOLVER_ORDER[k]) {
          for (double t : r.raw_times)
            all_times.push_back(std::max(t, 0.1)); // floor at 0.1µs for log axis
        }
      }
      if (all_times.empty()) continue;

      box_data.push_back(all_times);
      positions.push_back(k + 1.0);

      double loc_max = *std::max_element(all_times.begin(), all_times.end());
      ymax = std::max(ymax, loc_max);

      // Dummy plot for legend entry (marker only, not rendered as line)
      ax.plot({k + 1.0}, {0.1}, legend_fmts[k].c_str(),
              {{"label", std::string(SOLVER_ORDER[k])}});
    }

    // [FIX I1] Native boxplot — replaces stem workaround from V5
    if (!box_data.empty())
      ax.boxplot(box_data, positions,
                 opts({{"width", "0.6"}, {"color", "blue"}}));

    ax.set_yscale("log");

    // Deadline reference lines — always visible on log axis
    double xlo = 0.3, xhi = (double)SOLVER_ORDER.size() + 0.7;
    ax.plot({xlo, xhi}, {1000.0,  1000.0},  "m:",  {{"label", std::string("1ms")}});
    ax.plot({xlo, xhi}, {10000.0, 10000.0}, "r--", {{"label", std::string("10ms deadline")}});

    ax.set_title(SC_SHORT[si] + " — Solve Time Distribution");
    ax.set_xlabel("Solver  (see legend)");
    ax.set_ylabel("Time (us, log scale)");
    ax.set_xlim(xlo, xhi);
    ax.set_ylim(0.05, std::max(ymax * 5.0, 31600.0));
    ax.grid(true);
    ax.legend(true);
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
void fig_convergence(const std::vector<BenchRow> & /*unused*/,
                     const std::string &out) {
  MPCProblem<QP_N> mpc;
  double Q3[2] = {100.0, 1.0};
  mpc.build(Q3, 0.001); // S3 hard

  Vec<QP_N> lb, ub;
  for (int i = 0; i < QP_N; ++i) {
    lb[i] = -0.3;
    ub[i] = 0.3;
  }

  Vec<QP_N> g_vec = mpc.g_from_state(3.0, 0.0); // challenging state

  const int MAX_K = 150;
  const double rho = 1.0;

  // Factorize (H + rho·I)
  Mat<QP_N> Mreg = mpc.H;
  for (int i = 0; i < QP_N; ++i)
    Mreg(i, i) += rho;
  Mat<QP_N> Lfact;
  cholesky(Lfact, Mreg);

  Vec<QP_N> x_k, z_k, u_k;
  std::vector<double> iter_v, prim_v, dual_v, obj_v;

  for (int k = 0; k < MAX_K; ++k) {
    Vec<QP_N> rhs;
    for (int i = 0; i < QP_N; ++i)
      rhs[i] = rho * z_k[i] - u_k[i] - g_vec[i];
    x_k = chol_solve(Lfact, rhs);

    Vec<QP_N> z_prev = z_k;
    for (int i = 0; i < QP_N; ++i) {
      double v = x_k[i] + u_k[i] / rho;
      z_k[i] = std::max(lb[i], std::min(ub[i], v));
    }
    for (int i = 0; i < QP_N; ++i)
      u_k[i] += rho * (x_k[i] - z_k[i]);

    double pr = (x_k - z_k).norm2();
    double dr = rho * (z_k - z_prev).norm2();
    double obj = mpc.cost(z_k, g_vec);

    iter_v.push_back(k + 1.0);
    prim_v.push_back(std::max(pr, 1e-14));
    dual_v.push_back(std::max(dr, 1e-14));
    obj_v.push_back(obj);
  }

  Figure fig(1100, 460);

  // Left: residuals
  {
    auto &ax = fig.subplot(1, 2, 1);
    ax.plot(iter_v, prim_v, "b-", {{"label", std::string("Primal ||x-z||")}});
    ax.plot(iter_v, dual_v, "r-", {{"label", std::string("Dual rho*||Dz||")}});
    // eps reference line
    double eps_line = 1e-4 * std::sqrt(QP_N);
    ax.plot({1.0, (double)MAX_K}, {eps_line, eps_line}, "k--",
            {{"label", std::string("eps_abs*sqrt(N)")}});
    ax.set_title("ADMM Residuals  (S3, x0=[3,0], rho=1)");
    ax.set_xlabel("Iteration k");
    ax.set_ylabel("Residual");
    ax.legend(true);
    ax.grid(true);
  }

  // Right: objective
  {
    auto &ax = fig.subplot(1, 2, 2);
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
 * [FIX P3] Y-axis was linear → errors ranging 1e-14 to 1 all collapsed near
 * zero. Fix: uses native ax.yscale("log"). Threshold line at 1e-6 (acceptable
 * accuracy for 5cm AGV spec). [FIX B3] also: solver names updated to canonical
 * "PGD" / "CG-proj".
 */
void fig_accuracy(const std::vector<BenchRow> &rows, const std::string &out) {
  Figure fig(1300, 430);
  // [FIX B3] canonical names
  const std::vector<std::string> sv_plt = {"ADMM-cold", "ADMM-warm",
                                           "ADMM-tuned", "PGD", "CG-proj"};
  const std::vector<std::string> fmts = {"r-", "b-", "m-", "g-", "c-"};

  for (int si = 0; si < 3; ++si) {
    auto &ax = fig.subplot(1, 3, si + 1);
    const std::string &sc = SC_NAMES[si];

    for (int k = 0; k < (int)sv_plt.size(); ++k) {
      std::vector<std::pair<double, double>> pts;
      for (const auto &r : rows)
        if (r.scenario == sc && r.solver == sv_plt[k])
          pts.emplace_back(r.x0_pos, std::max(r.u_l2_err, 1e-15));
      if (pts.empty())
        continue;
      std::sort(pts.begin(), pts.end());
      std::vector<double> xs, ys;
      for (auto &p : pts) {
        xs.push_back(p.first);
        ys.push_back(p.second);
      }
      ax.plot(xs, ys, fmts[k].c_str(), {{"label", std::string(sv_plt[k])}});
    }

    ax.set_yscale("log");

    // Threshold line: 1e-6 (acceptable accuracy bound)
    std::vector<double> ref_xs;
    for (const auto &r : rows)
      if (r.scenario == sc && r.solver == "ADMM-cold")
        ref_xs.push_back(r.x0_pos);
    if (!ref_xs.empty()) {
      double xmin = *std::min_element(ref_xs.begin(), ref_xs.end());
      double xmax = *std::max_element(ref_xs.begin(), ref_xs.end());
      ax.plot({xmin, xmax}, {1e-6, 1e-6}, "k--",
              {{"label", std::string("1e-6 threshold")}});
    }

    ax.set_title(SC_SHORT[si] + " — Solution Accuracy");
    ax.set_xlabel("x0 position (m)");
    ax.set_ylabel("||u - u*||2");
    ax.set_ylim(1e-15, 10.0); // covers machine-eps to mild divergence
    ax.grid(true);
    if (si == 0)
      ax.legend(true);
  }
  fig.savefig(out);
  std::cout << "  -> " << out << "\n";
}

// ── Fig 4: Time vs accuracy Pareto scatter
// ────────────────────────────────────
/**
 * [FIX P4a] Both axes use native log scale via xscale("log") and yscale("log").
 *
 * [FIX P4b] All points were same blue color, no labels → solver identity lost.
 *   Root cause: ax.scatter(all_xs, all_ys, {{"c","blue"}}) collects all solvers
 *   into one call with a single color and no label map.
 *   Fix: one ax.plot({x},{y}, fmt, {label}) call per solver with unique marker
 *   format string ("ro", "bs", "m^", "gD", "cx", "k+") and solver label.
 *   No line style ('-') → markers only, which is correct for a scatter plot.
 */
void fig_tradeoff(const std::vector<BenchRow> &rows, const std::string &out) {
  Figure fig(1300, 430);

  // Unique marker per solver (no line — marker only)
  const std::vector<std::string> sv_fmts = {
      "ro", // ADMM-cold    red circle
      "bs", // ADMM-warm    blue square
      "m^", // ADMM-tuned   magenta triangle
      "gD", // PGD          green diamond  (use 's' if 'D' unsupported)
      "cx", // CG-proj      cyan cross
      "k+"  // Chol-Direct  black plus
  };

  for (int si = 0; si < 3; ++si) {
    auto &ax = fig.subplot(1, 3, si + 1);
    const std::string &sc = SC_NAMES[si];

    double xmin_v = 1e9, xmax_v = -1e9, ymin_v = 1e9, ymax_v = -1e9;

    for (int k = 0; k < (int)SOLVER_ORDER.size(); ++k) {
      const std::string &sn = SOLVER_ORDER[k];
      auto t = collect(rows, sc, sn,
                       [](const BenchRow &r) { return r.t_median_us; });
      auto e =
          collect(rows, sc, sn, [](const BenchRow &r) { return r.u_l2_err; });
      if (t.empty())
        continue;

      double x_val = std::max(median_v(t), 0.5);
      double y_val = std::max(median_v(e), 1e-15);

      xmin_v = std::min(xmin_v, x_val);
      xmax_v = std::max(xmax_v, x_val);
      ymin_v = std::min(ymin_v, y_val);
      ymax_v = std::max(ymax_v, y_val);

      // [FIX P4b] per-solver call with unique marker format + label
      ax.plot({x_val}, {y_val}, sv_fmts[k % sv_fmts.size()].c_str(),
              {{"label", std::string(sn)}});
    }

    ax.set_xscale("log");
    ax.set_yscale("log");

    // Add axis padding
    if (xmin_v < xmax_v) {
      ax.set_xlim(xmin_v / 1.5, xmax_v * 1.5);
      ax.set_ylim(ymin_v / 2.0, ymax_v * 2.0);
    }

    ax.set_title(SC_SHORT[si] + " — Time vs Accuracy Pareto");
    ax.set_xlabel("Median time, us");
    ax.set_ylabel("Median ||u - u*||2");
    ax.grid(true);
    if (si == 0)
      ax.legend(true);
  }
  fig.savefig(out);
  std::cout << "  -> " << out << "\n";
}

// ── Fig 5: Solve time CDF
// ─────────────────────────────────────────────────────
/**
 * [FIX P5] X-axis uses native log scale via ax.xscale("log").
 *   Deadline marker at 10000 — always visible.
 *   1ms marker at 1000 added as secondary reference.
 *   [FIX B3] solver name "DCAS-PGD" → "PGD".
 */
void fig_cdf(const std::vector<BenchRow> &rows, const std::string &out) {
  Figure fig(1300, 430);
  // [FIX B3] canonical names
  const std::vector<std::string> cdf_sv = {"ADMM-cold", "ADMM-warm", "PGD",
                                           "Chol-Direct"};
  const std::vector<std::string> cdf_fmts = {"r-", "b-", "g-", "k-"};

  for (int si = 0; si < 3; ++si) {
    auto &ax = fig.subplot(1, 3, si + 1);
    const std::string &sc = SC_NAMES[si];

    double xmax_v = -1e9, xmin_v = 1e9;

    for (int k = 0; k < (int)cdf_sv.size(); ++k) {
      auto t = collect(rows, sc, cdf_sv[k],
                       [](const BenchRow &r) { return r.t_median_us; });
      if (t.empty())
        continue;
      std::sort(t.begin(), t.end());

      std::vector<double> t_raw, cdf_y;
      for (int i = 0; i < (int)t.size(); ++i) {
        double tv = std::max(t[i], 0.5);
        t_raw.push_back(tv);
        cdf_y.push_back((i + 1.0) / t.size());
        xmin_v = std::min(xmin_v, tv);
        xmax_v = std::max(xmax_v, tv);
      }
      ax.plot(t_raw, cdf_y, cdf_fmts[k].c_str(),
              {{"label", std::string(cdf_sv[k])}});
    }

    ax.set_xscale("log");

    // Deadline markers on log axis — now always visible
    ax.plot({10000.0, 10000.0}, {0.0, 1.05}, "r--",
            {{"label", std::string("10ms deadline")}});
    ax.plot({1000.0, 1000.0}, {0.0, 1.05},
            "m:", {{"label", std::string("1ms")}});

    ax.set_title(SC_SHORT[si] + " — Solve Time CDF");
    ax.set_xlabel("Solve time, us");
    ax.set_ylabel("CDF  P(T <= t)");
    ax.set_ylim(0.0, 1.05);
    if (xmin_v < xmax_v)
      ax.set_xlim(std::max(0.1, xmin_v / 1.5), std::max(xmax_v * 1.5, 30000.0));
    ax.grid(true);
    if (si == 0)
      ax.legend(true);
  }
  fig.savefig(out);
  std::cout << "  -> " << out << "\n";
}

// ── Fig 6: Warm-start benefit
// ─────────────────────────────────────────────────
/**
 * Simulates closed-loop regulation x₀=[4,0] → 0 (S1 problem).
 * Compares ADMM warm vs cold iterations + cumulative savings.
 * Replaces Python demo_exp5() / EXP5 in demo_qp_mpc.cpp.
 */
void fig_warmstart(const std::string &out) {
  MPCProblem<QP_N> mpc;
  double Q1[2] = {1.0, 1.0};
  mpc.build(Q1, 0.1);

  Vec<QP_N> lb, ub;
  for (int i = 0; i < QP_N; ++i) {
    lb[i] = -1.0;
    ub[i] = 1.0;
  }

  Params pbase;
  pbase.rho = 1.0;
  pbase.max_iter = 100;
  pbase.eps_abs = 1e-4;
  pbase.eps_rel = 1e-3;

  ADMMSolver<QP_N> sw, sc;
  pbase.warm_start = true;
  sw.setup(mpc.H, pbase);
  pbase.warm_start = false;
  sc.setup(mpc.H, pbase);

  const int N_SIM = 80;
  double pos = 4.0, vel = 0.0, dt = MPCProblem<QP_N>::DT;
  double total_w = 0, total_c = 0;

  std::vector<double> step_v, pos_v, iw_v, ic_v, cumul_v;
  for (int k = 0; k < N_SIM; ++k) {
    Vec<QP_N> g = mpc.g_from_state(pos, vel);
    auto solw = sw.solve(g, lb, ub);
    sc.reset();
    auto solc = sc.solve(g, lb, ub);
    int iw = solw.iterations, ic = solc.iterations;
    total_w += iw;
    total_c += ic;
    step_v.push_back(k);
    pos_v.push_back(pos);
    iw_v.push_back(iw);
    ic_v.push_back(ic);
    cumul_v.push_back(total_c - total_w);
    double u = solw.x[0];
    pos += dt * vel + 0.5 * dt * dt * u;
    vel += dt * u;
  }

  double save_pct = (total_c > 0) ? 100.0 * (total_c - total_w) / total_c : 0.0;
  std::cout << "  Warm-start savings: " << std::fixed << std::setprecision(1)
            << save_pct << "% (" << (total_c - total_w) / N_SIM
            << " iter/step avg)\n";

  Figure fig(1200, 430);
  {
    auto &ax = fig.subplot(1, 3, 1);
    ax.plot(step_v, ic_v, "r-", {{"label", std::string("Cold start")}});
    ax.plot(step_v, iw_v, "b-", {{"label", std::string("Warm start")}});
    ax.set_title("ADMM Iterations per Step");
    ax.set_xlabel("Control step k");
    ax.set_ylabel("Iterations to convergence");
    ax.legend(true);
    ax.grid(true);
  }
  {
    auto &ax = fig.subplot(1, 3, 2);
    ax.plot(step_v, pos_v, "g-");
    ax.plot({0.0, (double)(N_SIM - 1)}, {0.0, 0.0}, "k--");
    ax.set_title("AGV Position  (regulation x->0)");
    ax.set_xlabel("Control step k");
    ax.set_ylabel("Position (m)");
    ax.grid(true);
  }
  {
    auto &ax = fig.subplot(1, 3, 3);
    ax.plot(step_v, cumul_v, "m-");
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

void write_csv(const std::vector<BenchRow> &rows, const std::string &path) {
  std::ofstream f(path);
  if (!f) {
    std::cerr << "ERROR: cannot write " << path << "\n";
    return;
  }
  f << "scenario,solver,x0_pos,x0_vel,u0,u0_ref,u_l2_err,cost_rel_err,"
       "iters,converged,n_active,t_min_us,t_median_us,t_p95_us,t_max_us,t_std_"
       "us\n";
  for (const auto &r : rows)
    f << r.scenario << "," << r.solver << "," << r.x0_pos << "," << r.x0_vel
      << "," << r.u0 << "," << r.u0_ref << "," << r.u_l2_err << ","
      << r.cost_rel_err << "," << r.iters << "," << (int)r.converged << ","
      << r.n_active << "," << r.t_min_us << "," << r.t_median_us << ","
      << r.t_p95_us << "," << r.t_max_us << "," << r.t_std_us << "\n";
  std::cout << "  -> " << path << "  (" << rows.size() << " rows)\n";
}

void write_summary(const std::vector<BenchRow> &rows, const std::string &path) {
  std::ofstream f(path);
  if (!f) {
    std::cerr << "ERROR: cannot write " << path << "\n";
    return;
  }
  f << "% Auto-generated by benchmark_qp_solvers.cpp\n"
    << "% DCAS Lab QP Benchmark  N_REPS=" << N_REPS << "  QP_N=" << QP_N
    << "  [V7 — boxplot + precision fix]\n\n";
  f << std::left << std::setw(16) << "Solver";
  for (auto &s : {"S1_easy", "S2_medium", "S3_hard"})
    f << std::right << std::setw(26) << (std::string(s) + " Med/P95/Max(us)");
  f << "\n" << std::string(94, '-') << "\n";
  for (auto &sn : SOLVER_ORDER) {
    f << std::left << std::setw(16) << sn;
    for (auto &sc : {"S1_easy", "S2_medium", "S3_hard"}) {
      auto tm = collect(rows, sc, sn,
                        [](const BenchRow &r) { return r.t_median_us; });
      auto tp =
          collect(rows, sc, sn, [](const BenchRow &r) { return r.t_p95_us; });
      auto tx =
          collect(rows, sc, sn, [](const BenchRow &r) { return r.t_max_us; });
      if (tm.empty()) {
        f << std::setw(26) << "N/A";
        continue;
      }
      // [FIX J1] setprecision(2) for Med/P95 — prevents sub-µs values (e.g.
      // Chol-Direct 0.16µs) from rounding to "0" with setprecision(0).
      // Max uses setprecision(1) — worst-case timing rarely sub-µs.
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(2) << median_v(tm) << "/"
         << std::setprecision(2) << median_v(tp) << "/"
         << std::setprecision(1) << *std::max_element(tx.begin(), tx.end());
      f << std::right << std::setw(26) << ss.str();
    }
    f << "\n";
  }
  auto tb = collect(rows, "S1_easy", "ADMM-cold",
                    [](const BenchRow &r) { return r.t_median_us; });
  double base = median_v(tb);
  f << "\nSpeedups vs ADMM-cold (S1 median):\n";
  for (auto &sn : SOLVER_ORDER) {
    if (sn == "ADMM-cold")
      continue;
    auto t = collect(rows, "S1_easy", sn,
                     [](const BenchRow &r) { return r.t_median_us; });
    if (t.empty())
      continue;
    f << "  " << std::left << std::setw(14) << sn << ": " << std::fixed
      << std::setprecision(2) << median_v(t) / base << "x\n";
  }
  std::cout << "  -> " << path << "\n";
}

// ══════════════════════════════════════════════════════════════════════════════
//   MAIN
// ══════════════════════════════════════════════════════════════════════════════

int main() {
  const std::string OUT = ".";

  std::cout
      << "\n╔══════════════════════════════════════════════════════════╗\n"
      << "║   DCAS QP Solver Benchmark — C++ + CppPlot  [V7]        ║\n"
      << "║   AGV/AMR Embedded MPC · N=10 · TDTU March 2026         ║\n"
      << "╚══════════════════════════════════════════════════════════╝\n\n";

  // Print problem info
  for (auto &sc : make_scenarios()) {
    MPCProblem<QP_N> mpc;
    mpc.build(sc.Q_diag, sc.R_val);
    std::cout << "  " << sc.name << ": cond(H)≈" << std::fixed
              << std::setprecision(1) << mpc.cond() << "  eig=[" << mpc.eig_min
              << "," << mpc.eig_max << "]"
              << "  |u|<=" << sc.u_bound << "\n";
  }

  // Run benchmark
  auto rows = run_benchmark();

  // Generate figures
  std::cout << "\nGenerating CppPlot figures...\n";
  fig_timing(rows, OUT + "/fig_bench_timing.svg");
  fig_convergence(rows, OUT + "/fig_bench_convergence.svg");
  fig_accuracy(rows, OUT + "/fig_bench_accuracy.svg");
  fig_tradeoff(rows, OUT + "/fig_bench_tradeoff.svg");
  fig_cdf(rows, OUT + "/fig_bench_cdf.svg");
  fig_warmstart(OUT + "/fig_bench_warmstart.svg");

  // Export data
  std::cout << "\nExporting data...\n";
  write_csv(rows, OUT + "/benchmark_results.csv");
  write_summary(rows, OUT + "/benchmark_summary.txt");

  // Final console summary
  std::cout << "\n"
            << std::string(68, '=') << "\n"
            << "FINAL SUMMARY — Median solve time (µs)  [cold-start]\n"
            << std::string(68, '=') << "\n";
  std::cout << std::left << std::setw(16) << "Solver" << std::right
            << std::setw(10) << "S1(µs)" << std::setw(10) << "S2(µs)"
            << std::setw(10) << "S3(µs)" << std::setw(10) << "S1 iters\n"
            << "  " << std::string(52, '-') << "\n";
  for (auto &sn : SOLVER_ORDER) {
    std::cout << std::left << std::setw(16) << sn;
    for (auto &sc : {"S1_easy", "S2_medium", "S3_hard"}) {
      auto t = collect(rows, sc, sn,
                       [](const BenchRow &r) { return r.t_median_us; });
      // [FIX J1] setprecision(2) — prevents 0.16µs rounding to "0"
      std::cout << std::right << std::fixed << std::setprecision(2)
                << std::setw(10) << (t.empty() ? 0.0 : median_v(t));
    }
    auto it = collect(rows, "S1_easy", sn,
                      [](const BenchRow &r) { return (double)r.iters; });
    std::cout << std::setw(10) << (int)(it.empty() ? 0 : median_v(it)) << "\n";
  }

  // WCET
  auto wcet_v = collect(rows, "S3_hard", "ADMM-cold",
                        [](const BenchRow &r) { return r.t_max_us; });
  if (!wcet_v.empty()) {
    double wcet = *std::max_element(wcet_v.begin(), wcet_v.end());
    std::cout << "\n  WCET ADMM-cold (S3 worst): " << std::fixed
              << std::setprecision(1) << wcet << " µs = " << wcet / 1000.0
              << " ms  "
              << (wcet < 10000 ? "✓ PASS (< 10ms)" : "✗ FAIL (> 10ms)") << "\n";
  }

  std::cout << "\n✓ All outputs saved to: " << OUT << "/\n\n";
  return 0;
}
