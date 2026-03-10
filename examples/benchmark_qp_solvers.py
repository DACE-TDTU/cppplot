"""
benchmark_qp_solvers.py
=======================
Comprehensive benchmark: DCAS ADMM QP Solver vs popular alternatives.

Solvers compared:
  1. DCAS-ADMM (cold)  — qp_solver.hpp algorithm, Python replication, cold-start
  2. DCAS-ADMM (warm)  — same, warm-start (closed-loop scenario)
  3. DCAS-PGD          — Projected Gradient Descent w/ Barzilai-Borwein (mpc.hpp baseline)
  4. CG+Proj           — Conjugate Gradient with box projection (Frank-Wolfe style)
  5. scipy SLSQP       — Sequential Least SQuares Programming (active-set, gold standard)
  6. scipy L-BFGS-B    — Limited-memory BFGS with box constraints (quasi-Newton)
  7. Chol-Direct       — Analytical u*=-H^{-1}g + clip (unconstrained projection)

All solve identical QP:
  min   0.5 * U' * H * U + g' * U
  s.t.  lb <= U <= ub

Problem: MPC for AGV double integrator (N=10 horizon, n=10 decision variables)
  H = Gamma'*Qbar*Gamma + Rbar  (real MPC Hessian, not mock)
  g = F' * x0  (changes every solve)

Three test scenarios:
  S1 — Easy:   Q=I,           R=0.1,   |u|<=2.0  cond(H)~1    (rarely active)
  S2 — Medium: Q=diag(10,1),  R=0.01,  |u|<=1.0  cond(H)~5    (some active)
  S3 — Hard:   Q=diag(100,1), R=0.001, |u|<=0.3  cond(H)~44   (all active)

Outputs:
  benchmark_results.csv       — full raw data (all reps)
  fig_bench_timing.png        — box plots of solve time
  fig_bench_iterations.png    — median iterations grouped by scenario
  fig_bench_accuracy.png      — solution accuracy vs tight reference
  fig_bench_tradeoff.png      — time vs accuracy Pareto scatter
  fig_bench_cdf.png           — CDF of solve times (real-time deadline view)

DCAS Lab · L0 Core Control Stack · TDTU
TS. Tri Vien Vu — March 2026
"""

import numpy as np
from scipy.optimize import minimize
from scipy.linalg import cho_factor, cho_solve
import time, csv, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from collections import defaultdict

# ═══════════════════════════════════════════════════════════════════
#   PART 1 — MPC PROBLEM CONSTRUCTION
# ═══════════════════════════════════════════════════════════════════

def build_mpc_matrices(dt=0.01, N=10, Q=None, R=None):
    """Real MPC prediction matrices for AGV double integrator."""
    A = np.array([[1, dt], [0, 1]])
    B = np.array([[0.5*dt**2], [dt]])
    n = 2
    if Q is None: Q = np.eye(2)
    if R is None: R = 0.1 * np.eye(1)

    Phi   = np.zeros((n*N, n))
    Gamma = np.zeros((n*N, N))
    Ak = A.copy()
    for k in range(N):
        Phi[n*k:n*k+n, :] = Ak
        Apow = np.eye(n)
        for j in range(k, -1, -1):
            Gamma[n*k:n*k+n, j:j+1] = Apow @ B
            Apow = A @ Apow
        if k < N-1:
            Ak = A @ Ak

    Qbar = np.kron(np.eye(N), Q)
    Rbar = np.kron(np.eye(N), R)
    H = Gamma.T @ Qbar @ Gamma + Rbar
    F = Gamma.T @ Qbar @ Phi
    return H, F


SCENARIOS = {
    "S1_easy": dict(
        label="S1: Easy\n(Q=I, R=0.1, |u|≤2)",
        Q=np.eye(2), R=0.1*np.eye(1), u_bound=2.0,
        x0_list=[[-5,0],[-4,0],[-3,0],[-2,0],[-1,0],[-0.5,0],
                 [0.5,0],[1,0],[2,0],[3,0],[4,0],[4.9,0],
                 [-3,0.3],[-3,-0.3],[2,0.2],[2,-0.2],
                 [1,0.5],[-1,-0.5],[4.5,0.1],[-4.5,-0.1]],
    ),
    "S2_medium": dict(
        label="S2: Medium\n(Q=diag(10,1), R=0.01, |u|≤1)",
        Q=np.diag([10.0,1.0]), R=0.01*np.eye(1), u_bound=1.0,
        x0_list=[[-2,0],[-1.5,0],[-1,0],[-0.5,0],[-0.2,0],
                 [0.2,0],[0.5,0],[1,0],[1.5,0],[2,0],
                 [-1,0.1],[-1,-0.1],[1,0.1],[1,-0.1],
                 [0.5,0.3],[0.5,-0.3],[1.5,0.2],[-1.5,-0.2],
                 [2,0.1],[-2,-0.1]],
    ),
    "S3_hard": dict(
        label="S3: Hard\n(Q=diag(100,1), R=0.001, |u|≤0.3)",
        Q=np.diag([100.0,1.0]), R=0.001*np.eye(1), u_bound=0.3,
        x0_list=[[-3,0],[-2,0],[-1,0],[-0.5,0],[-0.2,0],
                 [0.2,0],[0.5,0],[1,0],[2,0],[3,0],
                 [-1,0.05],[-1,-0.05],[1,0.05],[1,-0.05],
                 [0.5,0.1],[0.5,-0.1],[2,0.05],[-2,-0.05],
                 [3,0.02],[-3,-0.02]],
    ),
}

# ═══════════════════════════════════════════════════════════════════
#   PART 2 — SOLVER IMPLEMENTATIONS
# ═══════════════════════════════════════════════════════════════════

class ADMMSolver:
    """
    ADMM for box-constrained QP — exact Python replica of qp_solver.hpp.
    UNSCALED dual: u_ stores lambda (Lagrange multiplier).

    x-update:  x = (H+rho*I)^{-1}(rho*z - lambda - g)
    z-update:  z = clip(x + lambda/rho, lb, ub)
    lam-update: lambda += rho*(x - z)
    """
    name = "DCAS-ADMM"

    def __init__(self, H, rho=1.0, max_iter=200, eps_abs=1e-6, eps_rel=1e-5,
                 warm_start=False):
        self.H = H
        self.N = H.shape[0]
        if rho is None:
            eigs = np.linalg.eigvalsh(H)
            rho = float(max(10.0 * eigs.max(), np.sqrt(eigs.min() * eigs.max())))
        self.rho = rho
        self.max_iter = max_iter
        self.eps_abs = eps_abs
        self.eps_rel = eps_rel
        self.warm_start = warm_start
        M = H + rho * np.eye(self.N)
        self.L_factor = cho_factor(M, lower=True)
        self._x = np.zeros(self.N)
        self._z = np.zeros(self.N)
        self._lam = np.zeros(self.N)

    def reset(self):
        self._x[:] = 0; self._z[:] = 0; self._lam[:] = 0

    def solve(self, g, lb, ub):
        if not self.warm_start:
            x = np.zeros(self.N); z = np.zeros(self.N); lam = np.zeros(self.N)
        else:
            x = self._x.copy(); z = self._z.copy(); lam = self._lam.copy()

        rho = self.rho
        converged = False
        iters = self.max_iter

        for k in range(self.max_iter):
            rhs = rho * z - lam - g
            x = cho_solve(self.L_factor, rhs)

            z_prev = z.copy()
            z = np.clip(x + lam / rho, lb, ub)
            lam += rho * (x - z)

            pr = np.linalg.norm(x - z)
            dr = rho * np.linalg.norm(z - z_prev)
            eps_p = (self.eps_abs * np.sqrt(self.N)
                     + self.eps_rel * max(np.linalg.norm(x), np.linalg.norm(z)))
            eps_d = (self.eps_abs * np.sqrt(self.N)
                     + self.eps_rel * rho * np.linalg.norm(lam))
            if pr < eps_p and dr < eps_d:
                converged = True; iters = k + 1; break

        if self.warm_start:
            self._x = x; self._z = z; self._lam = lam

        cost = 0.5 * z @ self.H @ z + g @ z
        return {"x": z, "iters": iters, "converged": converged, "cost": cost}


class PGDSolver:
    """
    Projected Gradient Descent with Barzilai-Borwein step size.
    Mirrors PGDSolver<N> in demo_qp_mpc.cpp.
    """
    name = "DCAS-PGD"

    def __init__(self, H, max_iter=500, tol=1e-8):
        self.H = H
        self.max_iter = max_iter
        self.tol = tol

    def solve(self, g, lb, ub):
        U = np.zeros(len(g))
        alpha = 0.01
        converged = False
        iters = self.max_iter
        for k in range(self.max_iter):
            grad = self.H @ U + g
            if np.linalg.norm(grad) < self.tol:
                converged = True; iters = k; break
            U_new = np.clip(U - alpha * grad, lb, ub)
            dU = U_new - U
            dU2 = dU @ dU
            if dU2 > 1e-12:
                dUHdU = dU @ (self.H @ dU)
                if abs(dUHdU) > 1e-12:
                    alpha = float(np.clip(dU2 / dUHdU, 0.001, 1.0))
            U = U_new
        cost = 0.5 * U @ self.H @ U + g @ U
        return {"x": U, "iters": iters, "converged": converged, "cost": cost}


class CGProjSolver:
    """
    Conjugate Gradient with box projection (spectral projected gradient).
    Exploits H via matrix-vector product, no factorization overhead.
    """
    name = "CG+Proj"

    def __init__(self, H, max_iter=300, tol=1e-8):
        self.H = H
        self.max_iter = max_iter
        self.tol = tol

    def solve(self, g, lb, ub):
        x = np.zeros(len(g))
        grad = self.H @ x + g
        p = -grad.copy()
        converged = False
        iters = self.max_iter
        for k in range(self.max_iter):
            Hp = self.H @ p
            pHp = p @ Hp
            if abs(pHp) < 1e-15:
                iters = k; converged = True; break
            alpha = -(grad @ p) / pHp
            x_new = np.clip(x + alpha * p, lb, ub)
            grad_new = self.H @ x_new + g
            if np.linalg.norm(grad_new) < self.tol:
                x = x_new; iters = k+1; converged = True; break
            beta = (grad_new @ grad_new) / max(grad @ grad, 1e-15)
            p = -grad_new + beta * p
            x = x_new; grad = grad_new
        cost = 0.5 * x @ self.H @ x + g @ x
        return {"x": x, "iters": iters, "converged": converged, "cost": cost}


class CholDirectSolver:
    """
    Analytical: u* = -H^{-1}g, then project onto [lb, ub].
    Fastest possible — O(N^2) triangular solve.
    Note: projection makes solution suboptimal when constraints are active.
    """
    name = "Chol-Direct"

    def __init__(self, H):
        self.H = H
        self.L_factor = cho_factor(H, lower=True)

    def solve(self, g, lb, ub):
        u_unc = -cho_solve(self.L_factor, g)
        x = np.clip(u_unc, lb, ub)
        cost = 0.5 * x @ self.H @ x + g @ x
        n_active = int(np.sum((x <= lb + 1e-9) | (x >= ub - 1e-9)))
        return {"x": x, "iters": 1, "converged": True, "cost": cost,
                "n_active": n_active}


class ScipySLSQPSolver:
    """scipy SLSQP — active-set QP solver (gold standard Python reference)."""
    name = "scipy-SLSQP"

    def __init__(self, H, max_iter=500, ftol=1e-10):
        self.H = H
        self.max_iter = max_iter
        self.ftol = ftol

    def solve(self, g, lb, ub):
        N = len(g)
        res = minimize(
            lambda u: 0.5 * u @ self.H @ u + g @ u,
            np.zeros(N),
            jac=lambda u: self.H @ u + g,
            method='SLSQP',
            bounds=list(zip(lb, ub)),
            options={'ftol': self.ftol, 'maxiter': self.max_iter}
        )
        return {"x": res.x, "iters": res.nit, "converged": res.success,
                "cost": res.fun}


class ScipyLBFGSBSolver:
    """scipy L-BFGS-B — limited-memory quasi-Newton with box constraints."""
    name = "scipy-LBFGSB"

    def __init__(self, H, max_iter=500, gtol=1e-10):
        self.H = H
        self.max_iter = max_iter
        self.gtol = gtol

    def solve(self, g, lb, ub):
        N = len(g)
        res = minimize(
            lambda u: 0.5 * u @ self.H @ u + g @ u,
            np.zeros(N),
            jac=lambda u: self.H @ u + g,
            method='L-BFGS-B',
            bounds=list(zip(lb, ub)),
            options={'maxiter': self.max_iter, 'gtol': self.gtol, 'ftol': 1e-15}
        )
        return {"x": res.x, "iters": res.nit, "converged": res.success,
                "cost": res.fun}


# ═══════════════════════════════════════════════════════════════════
#   PART 3 — BENCHMARK ENGINE
# ═══════════════════════════════════════════════════════════════════

N_REPS   = 60   # timing repetitions per (scenario, x0)
N_WARMUP = 10   # discarded warmup reps per solver

SOLVER_STYLE = {
    "ADMM-cold":    {"color": "#E84040", "marker": "o",  "lw": 2.5, "z": 6,
                     "label": "DCAS-ADMM\n(cold start)"},
    "ADMM-warm":    {"color": "#FF8040", "marker": "s",  "lw": 2.0, "z": 5,
                     "label": "DCAS-ADMM\n(warm start)"},
    "PGD":          {"color": "#4A90E2", "marker": "^",  "lw": 1.5, "z": 3,
                     "label": "DCAS-PGD\n(BB step)"},
    "CG-proj":      {"color": "#7B68EE", "marker": "D",  "lw": 1.5, "z": 3,
                     "label": "CG+Proj\n(spectral)"},
    "scipy-SLSQP":  {"color": "#27AE60", "marker": "v",  "lw": 1.5, "z": 2,
                     "label": "scipy\nSLSQP"},
    "scipy-LBFGSB": {"color": "#8E44AD", "marker": "P",  "lw": 1.5, "z": 2,
                     "label": "scipy\nL-BFGS-B"},
    "ADMM-tuned":   {"color": "#FF4800", "marker": "*",  "lw": 2.5, "z": 5,
                     "label": "DCAS-ADMM\n(cold, ρ=opt)"},
    "Chol-Direct":  {"color": "#95A5A6", "marker": "x",  "lw": 1.2, "z": 1,
                     "label": "Chol Direct\n(no constr.)"},
}

def make_solvers(H):
    return {
        "ADMM-cold":    ADMMSolver(H, rho=1.0, max_iter=200, eps_abs=1e-6,
                                    eps_rel=1e-5, warm_start=False),
        "ADMM-warm":    ADMMSolver(H, rho=1.0, max_iter=200, eps_abs=1e-6,
                                    eps_rel=1e-5, warm_start=True),
        "PGD":          PGDSolver(H, max_iter=1000, tol=1e-8),
        "CG-proj":      CGProjSolver(H, max_iter=500, tol=1e-8),
        "ADMM-tuned":   ADMMSolver(H, rho=None, max_iter=200, eps_abs=1e-6,
                                    eps_rel=1e-5, warm_start=False),
        "scipy-SLSQP":  ScipySLSQPSolver(H, max_iter=500, ftol=1e-10),
        "scipy-LBFGSB": ScipyLBFGSBSolver(H, max_iter=500, gtol=1e-10),
        "Chol-Direct":  CholDirectSolver(H),
    }


def run_benchmark():
    all_rows = []
    summary = {sc: {sv: [] for sv in SOLVER_STYLE} for sc in SCENARIOS}

    for sc_name, sc in SCENARIOS.items():
        print(f"\n{'='*60}")
        print(f"Scenario: {sc_name}  ({len(sc['x0_list'])} states × {N_REPS} reps)")
        cond_H = np.linalg.cond(build_mpc_matrices(Q=sc["Q"], R=sc["R"])[0])
        print(f"  cond(H)={cond_H:.1f}  |u|<={sc['u_bound']}")
        print(f"{'='*60}")

        H, F = build_mpc_matrices(Q=sc["Q"], R=sc["R"])
        ub_val = sc["u_bound"]
        lb = np.full(10, -ub_val)
        ub = np.full(10,  ub_val)

        solvers = make_solvers(H)

        # Reference: tight ADMM for ground-truth solution
        ref = ADMMSolver(H, rho=1.0, max_iter=5000,
                         eps_abs=1e-12, eps_rel=1e-11, warm_start=False)

        # CPU warmup
        g_dummy = F @ np.array([1.0, 0.0])
        for sv in solvers.values():
            for _ in range(N_WARMUP):
                sv.solve(g_dummy, lb, ub)
            if hasattr(sv, 'reset'): sv.reset()

        for x0 in sc["x0_list"]:
            x0a = np.array(x0, dtype=float)
            g   = F @ x0a
            u_ref = ref.solve(g, lb, ub)["x"]
            n_active_ref = int(np.sum((u_ref <= lb+1e-6) | (u_ref >= ub-1e-6)))

            for sv_name, sv in solvers.items():
                if isinstance(sv, ADMMSolver) and sv.warm_start:
                    sv.reset()  # fresh warm-state per x0 block

                times = []
                sol = None
                for rep in range(N_REPS):
                    t0 = time.perf_counter()
                    sol = sv.solve(g, lb, ub)
                    t1 = time.perf_counter()
                    times.append((t1 - t0) * 1e6)

                t_arr = np.array(times)
                u_err = float(np.linalg.norm(sol["x"] - u_ref))
                cost_ref = 0.5 * u_ref @ H @ u_ref + g @ u_ref
                cost_err = abs(sol["cost"] - cost_ref) / (abs(cost_ref) + 1e-12)

                row = {
                    "scenario":      sc_name,
                    "solver":        sv_name,
                    "x0_pos":        x0[0],
                    "x0_vel":        x0[1],
                    "u0":            float(sol["x"][0]),
                    "u0_ref":        float(u_ref[0]),
                    "u_l2_err":      u_err,
                    "cost_rel_err":  cost_err,
                    "iters":         sol["iters"],
                    "converged":     int(sol["converged"]),
                    "n_active":      n_active_ref,
                    "t_min_us":      float(t_arr.min()),
                    "t_median_us":   float(np.median(t_arr)),
                    "t_p95_us":      float(np.percentile(t_arr, 95)),
                    "t_max_us":      float(t_arr.max()),
                    "t_std_us":      float(t_arr.std()),
                }
                all_rows.append(row)
                summary[sc_name][sv_name].append(row)

        # Per-scenario print
        fmt = "  {:<14} {:>8.1f} {:>8.1f} {:>8.1f} {:>7.0f} {:>10.2e} {:>7.1f}%"
        print(f"\n  {'Solver':<14} {'Med(µs)':>8} {'P95(µs)':>8} "
              f"{'Max(µs)':>8} {'Iters':>7} {'u_err':>10} {'Conv':>8}")
        print("  " + "-"*68)
        for sv_name in SOLVER_STYLE:
            rows = summary[sc_name][sv_name]
            med_t  = np.median([r["t_median_us"] for r in rows])
            p95_t  = np.percentile([r["t_p95_us"] for r in rows], 95)
            max_t  = np.max([r["t_max_us"] for r in rows])
            med_it = np.median([r["iters"] for r in rows])
            med_ue = np.median([r["u_l2_err"] for r in rows])
            conv   = 100.0 * np.mean([r["converged"] for r in rows])
            print(fmt.format(sv_name, med_t, p95_t, max_t, med_it, med_ue, conv))

    return all_rows, summary


# ═══════════════════════════════════════════════════════════════════
#   PART 4 — PLOTTING
# ═══════════════════════════════════════════════════════════════════

SC_NAMES   = list(SCENARIOS.keys())
SV_NAMES   = list(SOLVER_STYLE.keys())

def _colors():
    return [SOLVER_STYLE[s]["color"] for s in SV_NAMES]

def _labels():
    return [SOLVER_STYLE[s]["label"] for s in SV_NAMES]


def fig_timing_boxplot(summary, path):
    fig, axes = plt.subplots(1, 3, figsize=(14, 5.5))
    fig.suptitle(
        "QP Solver Timing — DCAS-ADMM vs Alternatives  |  AGV MPC N=10",
        fontsize=12, fontweight='bold', y=1.01)

    for ax, sc in zip(axes, SC_NAMES):
        data   = [[r["t_median_us"] for r in summary[sc][sv]] for sv in SV_NAMES]
        bp = ax.boxplot(data, patch_artist=True, notch=False,
                        medianprops=dict(color='black', lw=1.8),
                        flierprops=dict(marker='.', ms=3, alpha=0.4))
        for patch, c in zip(bp['boxes'], _colors()):
            patch.set_facecolor(c); patch.set_alpha(0.75)

        # Annotate medians
        for i, d in enumerate(data):
            ax.annotate(f"{np.median(d):.0f}",
                        xy=(i+1, np.median(d)), xytext=(0,8),
                        textcoords='offset points', ha='center',
                        fontsize=6.5, color='#222')

        ax.set_xticks(range(1, len(SV_NAMES)+1))
        ax.set_xticklabels(_labels(), fontsize=7)
        ax.set_yscale('log')
        ax.set_title(SCENARIOS[sc]["label"], fontsize=9.5, pad=5)
        if ax == axes[0]: ax.set_ylabel("Solve time (µs)", fontsize=10)
        ax.grid(True, axis='y', alpha=0.3, which='both')

    plt.tight_layout()
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  → {path}")


def fig_iterations(summary, path):
    n_sv  = len([s for s in SV_NAMES if s != "Chol-Direct"])
    sv_sub = [s for s in SV_NAMES if s != "Chol-Direct"]
    x = np.arange(3)
    width = 0.10
    offs  = np.linspace(-(n_sv-1)/2, (n_sv-1)/2, n_sv) * width

    fig, ax = plt.subplots(figsize=(11, 5))
    for i, sv in enumerate(sv_sub):
        meds = [np.median([r["iters"] for r in summary[sc][sv]]) for sc in SC_NAMES]
        iqrs = [np.percentile([r["iters"] for r in summary[sc][sv]], 75) -
                np.percentile([r["iters"] for r in summary[sc][sv]], 25)
                for sc in SC_NAMES]
        ax.bar(x + offs[i], meds, width*0.88,
               label=SOLVER_STYLE[sv]["label"].replace('\n',' '),
               color=SOLVER_STYLE[sv]["color"], alpha=0.8, zorder=3)
        ax.errorbar(x + offs[i], meds, yerr=iqrs,
                    fmt='none', color='#333', capsize=2, lw=0.8)

    ax.set_xticks(x)
    ax.set_xticklabels([SCENARIOS[s]["label"].replace('\n',' ') for s in SC_NAMES],
                       fontsize=9)
    ax.set_ylabel("Iterations (median ± IQR)", fontsize=10)
    ax.set_title("Solver Iterations — DCAS-ADMM vs Alternatives",
                 fontsize=12, fontweight='bold')
    ax.set_yscale('log')
    ax.legend(fontsize=8, ncol=3, loc='upper left')
    ax.grid(True, axis='y', alpha=0.35)
    plt.tight_layout()
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  → {path}")


def fig_accuracy(summary, path):
    sv_sub = [s for s in SV_NAMES if s != "Chol-Direct"]
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5), sharey=False)
    fig.suptitle("Solution Accuracy  ||u – u*||₂  vs Reference (ADMM ε=1e-12)",
                 fontsize=12, fontweight='bold')

    for ax, sc in zip(axes, SC_NAMES):
        for sv in sv_sub:
            rows = sorted(summary[sc][sv], key=lambda r: r["x0_pos"])
            xs   = [r["x0_pos"] for r in rows]
            ys   = [max(r["u_l2_err"], 1e-14) for r in rows]
            st = SOLVER_STYLE[sv]
            ax.semilogy(xs, ys, st["marker"]+'-',
                        color=st["color"], lw=st["lw"], ms=5, alpha=0.85,
                        label=st["label"].replace('\n',' '), zorder=st["z"])
        ax.axhline(1e-6, color='gray', ls='--', lw=0.9, alpha=0.6)
        ax.set_title(SCENARIOS[sc]["label"], fontsize=9.5)
        ax.set_xlabel("x₀ position (m)", fontsize=9)
        if ax == axes[0]: ax.set_ylabel("||u – u*||₂", fontsize=10)
        ax.grid(True, alpha=0.3, which='both')
        ax.tick_params(labelsize=8)
    axes[0].legend(fontsize=7.5, ncol=2, loc='best')
    plt.tight_layout()
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  → {path}")


def fig_tradeoff(summary, path):
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5))
    fig.suptitle("Time–Accuracy Trade-off by Scenario  (lower-left = Pareto optimal)",
                 fontsize=12, fontweight='bold')

    for ax, sc in zip(axes, SC_NAMES):
        for sv in SV_NAMES:
            rows = summary[sc][sv]
            t_med  = np.median([r["t_median_us"] for r in rows])
            u_med  = np.median([r["u_l2_err"]    for r in rows]) + 1e-14
            st = SOLVER_STYLE[sv]
            ax.scatter(t_med, u_med, color=st["color"], marker=st["marker"],
                       s=100, zorder=st["z"], label=st["label"].replace('\n',' '))
            ax.annotate(sv.replace("scipy-","").replace("ADMM-","ADMM\n"),
                        (t_med, u_med), xytext=(5,3),
                        textcoords='offset points', fontsize=6, color='#333')

        ax.set_xscale('log'); ax.set_yscale('log')
        ax.set_xlabel("Median solve time (µs)", fontsize=9)
        if ax == axes[0]: ax.set_ylabel("Median ||u – u*||₂", fontsize=10)
        ax.set_title(SCENARIOS[sc]["label"], fontsize=9.5)
        ax.grid(True, alpha=0.3, which='both')
        ax.tick_params(labelsize=8)
        ax.annotate("← faster  ↓ accurate\n⊕ Pareto optimal",
                    xy=(0.97, 0.97), xycoords='axes fraction',
                    ha='right', va='top', fontsize=6.5, color='gray',
                    bbox=dict(boxstyle='round,pad=0.25', fc='white', alpha=0.7,
                              ec='lightgray'))
    axes[0].legend(fontsize=7, ncol=2, loc='lower right')
    plt.tight_layout()
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  → {path}")


def fig_cdf(summary, path):
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5), sharey=True)
    fig.suptitle("Solve Time CDF — Real-Time Deadline Analysis",
                 fontsize=12, fontweight='bold')

    for ax, sc in zip(axes, SC_NAMES):
        for sv in SV_NAMES:
            rows = summary[sc][sv]
            times = sorted([r["t_median_us"] for r in rows])
            cdf   = np.linspace(0, 1, len(times))
            st = SOLVER_STYLE[sv]
            ax.plot(times, cdf, st["marker"]+'-',
                    color=st["color"], lw=st["lw"], ms=5, alpha=0.85,
                    label=st["label"].replace('\n',' '), zorder=st["z"])

        ax.axvline(10000, color='red',  ls='--', lw=1.3, label='10ms limit')
        ax.axvline( 1000, color='orange', ls=':', lw=1.0, label='1ms limit')
        ax.axhline(0.99,  color='gray', ls=':',  lw=0.8)
        ax.set_xscale('log')
        ax.set_xlabel("Solve time (µs)", fontsize=9)
        if ax == axes[0]: ax.set_ylabel("CDF P(T ≤ t)", fontsize=10)
        ax.set_title(SCENARIOS[sc]["label"], fontsize=9.5)
        ax.grid(True, alpha=0.3, which='both')
        ax.tick_params(labelsize=8)
        ax.set_ylim(0, 1.05)
    axes[0].legend(fontsize=7.5, ncol=1, loc='lower right')
    plt.tight_layout()
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  → {path}")


def write_summary_table(summary, path):
    """Write LaTeX-ready summary table to txt."""
    lines = []
    lines.append("% Auto-generated by benchmark_qp_solvers.py")
    lines.append("% DCAS Lab QP Benchmark Summary")
    lines.append(f"% N_REPS={N_REPS}")
    lines.append("")
    lines.append(f"{'Solver':<14}  "
                 + "  ".join(f"{'Med/P95/Max (µs)':>20}" for _ in SC_NAMES))
    lines.append("-"*80)
    for sv in SV_NAMES:
        row = f"{sv:<14}"
        for sc in SC_NAMES:
            rows = summary[sc][sv]
            med = np.median([r["t_median_us"] for r in rows])
            p95 = np.percentile([r["t_p95_us"] for r in rows], 95)
            mx  = np.max([r["t_max_us"] for r in rows])
            row += f"  {med:6.0f}/{p95:6.0f}/{mx:6.0f}"
        lines.append(row)
    lines.append("")
    lines.append("Speedups vs ADMM-cold (S1 median):")
    admm_med = np.median([r["t_median_us"] for r in summary["S1_easy"]["ADMM-cold"]])
    for sv in SV_NAMES:
        if sv == "ADMM-cold": continue
        other = np.median([r["t_median_us"] for r in summary["S1_easy"][sv]])
        lines.append(f"  {sv:<14}: {other/admm_med:.2f}x")
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f"  → {path}")


# ═══════════════════════════════════════════════════════════════════
#   MAIN
# ═══════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    OUT = "/mnt/user-data/outputs"

    print("╔══════════════════════════════════════════════════════════════╗")
    print("║   DCAS Lab QP Solver Benchmark vs Popular Alternatives      ║")
    print("║   TDTU · L0 Core Control Stack · March 2026                 ║")
    print("╚══════════════════════════════════════════════════════════════╝")
    print(f"\n  N_REPS={N_REPS}  N_WARMUP={N_WARMUP}")

    # Print problem info
    for sc_name, sc in SCENARIOS.items():
        H, _ = build_mpc_matrices(Q=sc["Q"], R=sc["R"])
        eigs = np.linalg.eigvalsh(H)
        print(f"  {sc_name}: cond(H)={eigs.max()/eigs.min():.1f}  "
              f"eig=[{eigs.min():.4f}, {eigs.max():.4f}]  |u|<={sc['u_bound']}")

    rows, summary = run_benchmark()

    print("\n\nGenerating figures...")
    fig_timing_boxplot(summary, f"{OUT}/fig_bench_timing.png")
    fig_iterations    (summary, f"{OUT}/fig_bench_iterations.png")
    fig_accuracy      (summary, f"{OUT}/fig_bench_accuracy.png")
    fig_tradeoff      (summary, f"{OUT}/fig_bench_tradeoff.png")
    fig_cdf           (summary, f"{OUT}/fig_bench_cdf.png")

    # CSV
    keys = list(rows[0].keys())
    with open(f"{OUT}/benchmark_results.csv", 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader(); w.writerows(rows)
    print(f"  → {OUT}/benchmark_results.csv")

    write_summary_table(summary, f"{OUT}/benchmark_summary.txt")

    # Console summary
    print("\n" + "="*72)
    print("FINAL SUMMARY — Median solve time (µs)  [cold-start unless noted]")
    print("="*72)
    fmt = "  {:<14}  {:>8}  {:>8}  {:>8}  {:>8}"
    print(fmt.format("Solver", "S1 med", "S2 med", "S3 med", "S1 iters"))
    print("  " + "-"*55)
    for sv in SV_NAMES:
        meds = [np.median([r["t_median_us"] for r in summary[sc][sv]])
                for sc in SC_NAMES]
        iters_s1 = np.median([r["iters"] for r in summary["S1_easy"][sv]])
        print(fmt.format(sv, f"{meds[0]:.1f}", f"{meds[1]:.1f}",
                         f"{meds[2]:.1f}", f"{iters_s1:.0f}"))

    print("\n  Speedup ADMM-cold vs competitors (S1 median timing):")
    admm_t = np.median([r["t_median_us"] for r in summary["S1_easy"]["ADMM-cold"]])
    for sv in SV_NAMES:
        if sv == "ADMM-cold": continue
        t = np.median([r["t_median_us"] for r in summary["S1_easy"][sv]])
        tag = "FASTER than ADMM" if t < admm_t else f"{t/admm_t:.1f}x slower"
        print(f"    vs {sv:<14}: {tag}")

    print(f"\n✓ All outputs saved to {OUT}")
