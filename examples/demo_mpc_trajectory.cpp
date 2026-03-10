// ============================================================
//  demo_mpc_trajectory.cpp
//  EXP 3 — AGV Double Integrator: MPC Closed-Loop with Input Constraints
//
//  Generates: fig_exp3_trajectory.svg
//  Layout:    3 columns × 3 rows
//             Columns: Unconstrained |u|≤10, Light |u|≤1, Tight |u|≤0.5
//             Rows:    Position + reference, Velocity, Control input
//
//  Plant:     AGV double integrator, n=2, m=1, Ts=0.01s
//             x = [position, velocity]', u = acceleration
//  MPC:       N=10, Q=diag(10,1), R=0.1, Pf=DARE solution
//             x0=[0,0], reference=2.0m, T_sim=150..1500 steps
//
//  Solver:    ADMMSolver<10> with auto-tuned rho*, warm-start=true
//             (same qp_solver.hpp as benchmark_qp_solvers_V7.cpp)
//
//  Compile:   g++ -std=c++17 -O2 -I../include demo_mpc_trajectory.cpp \
//               -o demo_traj && ./demo_traj
//  Output:    results/fig_exp3_trajectory.svg
// ============================================================

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

// ── CppPlot & QP solver ──────────────────────────────────────
#include <cppplot/cppplot.hpp>
#include <cppplot/control/qp_solver.hpp>

using namespace cppplot;

// ============================================================
//  §1  CONSTANTS
// ============================================================
static constexpr int    QP_N  = 10;     // MPC horizon
static constexpr double Ts    = 0.01;   // sampling period [s]
static constexpr double X_REF = 2.0;    // position reference [m]

// ── Plant matrices (ZOH double integrator) ──────────────────
// A = [[1, Ts], [0, 1]],  B = [[0.5*Ts^2], [Ts]]
static constexpr double A11 = 1.0, A12 = Ts,          A21 = 0.0, A22 = 1.0;
static constexpr double B1  = 0.5 * Ts * Ts,           B2  = Ts;

// ── MPC weight matrices ──────────────────────────────────────
// Q = diag(10, 1),  R = 0.1
// Pf = value-iteration DARE solution (pre-computed below)
static constexpr double Q11 = 10.0, Q22 = 1.0, R_val = 0.1;

// ============================================================
//  §2  MINIMAL LINEAR ALGEBRA  (no Eigen dependency)
// ============================================================

// 2×2 symmetric matrix
struct Mat2 {
    double d[2][2] = {};
    double& operator()(int r, int c)       { return d[r][c]; }
    double  operator()(int r, int c) const { return d[r][c]; }
};

// 2-vector
struct Vec2 {
    double v[2] = {};
    double& operator[](int i)       { return v[i]; }
    double  operator[](int i) const { return v[i]; }
};

// N-vector (N = QP_N = 10)
using VecN = std::array<double, QP_N>;

// N×N symmetric matrix stored row-major
struct MatN {
    double d[QP_N][QP_N] = {};
    double& operator()(int r, int c)       { return d[r][c]; }
    double  operator()(int r, int c) const { return d[r][c]; }
};

// ============================================================
//  §3  PLANT SIMULATION
// ============================================================

// One-step plant update:  x_{k+1} = A*x_k + B*u_k
static Vec2 plant_step(const Vec2& x, double u) {
    Vec2 xn;
    xn[0] = A11*x[0] + A12*x[1] + B1*u;
    xn[1] = A21*x[0] + A22*x[1] + B2*u;
    return xn;
}

// ============================================================
//  §4  MPC PROBLEM BUILDER
//      Builds Hessian H and coupling matrix F = Gamma'*Qbar*Phi
//      so that g(x0) = F * x0_shifted  (shifted state = x - ref)
// ============================================================

struct MPCData {
    MatN H;          // QP Hessian  (mN × mN, m=1)
    MatN F_mat;      // F = Gamma'*Qbar*Phi  (mN × n)
    double rho_star; // spectral penalty (auto-tuned)
};

// Value-iteration DARE:  Pf ← Q + A'*Pf*A - A'*Pf*B*(R+B'*Pf*B)^{-1}*B'*Pf*A
static Mat2 dare_solve() {
    Mat2 P;
    P(0,0) = Q11; P(1,1) = Q22;   // init with Q
    for (int iter = 0; iter < 500; ++iter) {
        // S = R + B'*P*B  (scalar since m=1)
        double S = R_val + B1*(P(0,0)*B1 + P(1,0)*B2)
                         + B2*(P(0,1)*B1 + P(1,1)*B2);
        // K = (B'*P*A) / S  (1×2 row vector)
        double K0 = (B1*(P(0,0)*A11 + P(0,1)*A21)
                   + B2*(P(1,0)*A11 + P(1,1)*A21)) / S;
        double K1 = (B1*(P(0,0)*A12 + P(0,1)*A22)
                   + B2*(P(1,0)*A12 + P(1,1)*A22)) / S;
        // Pnew = Q + A'*P*A - A'*P*B*K
        Mat2 Pn;
        // A'*P*A
        double AP00 = A11*(P(0,0)*A11 + P(0,1)*A21) + A21*(P(1,0)*A11 + P(1,1)*A21);
        double AP01 = A11*(P(0,0)*A12 + P(0,1)*A22) + A21*(P(1,0)*A12 + P(1,1)*A22);
        double AP11 = A12*(P(0,0)*A12 + P(0,1)*A22) + A22*(P(1,0)*A12 + P(1,1)*A22);
        // A'*P*B (column: 2×1)
        double APB0 = A11*(P(0,0)*B1 + P(0,1)*B2) + A21*(P(1,0)*B1 + P(1,1)*B2);
        double APB1 = A12*(P(0,0)*B1 + P(0,1)*B2) + A22*(P(1,0)*B1 + P(1,1)*B2);
        Pn(0,0) = Q11 + AP00 - APB0*K0;
        Pn(0,1) = AP01 - APB0*K1;
        Pn(1,0) = Pn(0,1);
        Pn(1,1) = Q22 + AP11 - APB1*K1;
        // convergence check
        double diff = std::abs(Pn(0,0)-P(0,0)) + std::abs(Pn(0,1)-P(0,1))
                    + std::abs(Pn(1,1)-P(1,1));
        P = Pn;
        if (diff < 1e-12) break;
    }
    return P;
}

// Build Phi (nN × n) and Gamma (nN × mN)
// Phi_row_j = A^{j+1}   (2-vector → but we only need coupling to 1D state)
// Gamma[j][k] = A^{j-k} * B  for j >= k,  else 0
// Since m=1, Gamma is (nN × N) and H is (N × N)
static MPCData build_mpc() {
    const Mat2 Pf = dare_solve();

    // Powers of A:  Apow[j] = A^j,  j = 0..N
    std::array<Mat2, QP_N+1> Apow;
    Apow[0](0,0) = 1; Apow[0](1,1) = 1;   // identity
    for (int j = 1; j <= QP_N; ++j) {
        // Apow[j] = A * Apow[j-1]
        Apow[j](0,0) = A11*Apow[j-1](0,0) + A12*Apow[j-1](1,0);
        Apow[j](0,1) = A11*Apow[j-1](0,1) + A12*Apow[j-1](1,1);
        Apow[j](1,0) = A21*Apow[j-1](0,0) + A22*Apow[j-1](1,0);
        Apow[j](1,1) = A21*Apow[j-1](0,1) + A22*Apow[j-1](1,1);
    }

    // Qbar weights:  Q for j=0..N-1, Pf for j=N
    // For each column k of Gamma (k=0..N-1):
    //   Gamma[j][k] = A^{j-k-1} * B,  j-k-1 >= 0
    //   The (i=0,1) row of Gamma[j][k]:  Gamma_col_k[j] = A^{j-k-1} * B

    // H(k,l) = sum_{j=max(k,l)}^{N} Gamma[j][k]' * Qj * Gamma[j][l]  + R*delta(k,l)
    // F(k, :) = sum_{j=k}^{N}  Gamma[j][k]' * Qj * Phi_j
    // Phi_j = A^{j+1} * (first column — for shifted reference offset)

    // gamma_ij(j, k) = A^{j-k-1}*B  (2-vector, j >= k+1; 0 otherwise)
    auto gamma_col = [&](int j, int k) -> Vec2 {
        Vec2 out;
        if (j < k+1) return out;   // zero
        int exp = j - k - 1;       // A^exp * B
        out[0] = Apow[exp](0,0)*B1 + Apow[exp](0,1)*B2;
        out[1] = Apow[exp](1,0)*B1 + Apow[exp](1,1)*B2;
        return out;
    };

    // phi_j = A^{j+1}  (as 2×2 matrix, but we only need rows acting on 2D state)
    auto phi_j = [&](int j) -> Mat2 { return Apow[j+1]; };

    MPCData mpc;

    for (int k = 0; k < QP_N; ++k) {
        for (int l = 0; l < QP_N; ++l) {
            double hkl = 0.0;
            for (int j = std::max(k,l); j < QP_N; ++j) {
                Vec2 gk = gamma_col(j+1, k);   // Gamma[j+1][k]
                Vec2 gl = gamma_col(j+1, l);
                // Qj * gl   (Q = diag(Q11, Q22))
                double Qgl0 = Q11 * gl[0];
                double Qgl1 = Q22 * gl[1];
                hkl += gk[0]*Qgl0 + gk[1]*Qgl1;
            }
            // Terminal term j=N:  Gamma[N][k]' * Pf * Gamma[N][l]
            {
                Vec2 gk = gamma_col(QP_N, k);
                Vec2 gl = gamma_col(QP_N, l);
                double Pgl0 = Pf(0,0)*gl[0] + Pf(0,1)*gl[1];
                double Pgl1 = Pf(1,0)*gl[0] + Pf(1,1)*gl[1];
                hkl += gk[0]*Pgl0 + gk[1]*Pgl1;
            }
            if (k == l) hkl += R_val;
            mpc.H(k, l) = hkl;
        }
    }

    // F(k, :)   — rows: k=0..N-1,  cols: n=2
    for (int k = 0; k < QP_N; ++k) {
        double f0 = 0.0, f1 = 0.0;
        for (int j = k; j < QP_N; ++j) {
            Vec2 gk = gamma_col(j+1, k);
            Mat2 Phj = phi_j(j);
            // gk' * Q * Phi_j   (Q=diag)
            f0 += Q11*gk[0]*Phj(0,0) + Q22*gk[1]*Phj(1,0);
            f1 += Q11*gk[0]*Phj(0,1) + Q22*gk[1]*Phj(1,1);
        }
        // Terminal: gk' * Pf * Phi_N
        {
            Vec2 gk = gamma_col(QP_N, k);
            Mat2 PhN = phi_j(QP_N-1);   // A^N
            f0 += (Pf(0,0)*gk[0] + Pf(1,0)*gk[1]) * PhN(0,0)
                + (Pf(0,1)*gk[0] + Pf(1,1)*gk[1]) * PhN(1,0);
            f1 += (Pf(0,0)*gk[0] + Pf(1,0)*gk[1]) * PhN(0,1)
                + (Pf(0,1)*gk[0] + Pf(1,1)*gk[1]) * PhN(1,1);
        }
        mpc.F_mat(k, 0) = f0;
        mpc.F_mat(k, 1) = f1;
    }

    // Spectral rho*: power iteration for lambda_max, lambda_min of H
    // rho* = max(10*lambda_max, sqrt(lambda_min * lambda_max))
    {
        // Power iteration for lambda_max
        VecN q; q.fill(1.0 / std::sqrt(QP_N));
        double lam_max = 1.0;
        for (int it = 0; it < 60; ++it) {
            VecN Hq{}; 
            for (int i = 0; i < QP_N; ++i)
                for (int j = 0; j < QP_N; ++j)
                    Hq[i] += mpc.H(i,j) * q[j];
            lam_max = 0.0;
            for (int i = 0; i < QP_N; ++i) lam_max += Hq[i]*Hq[i];
            lam_max = std::sqrt(lam_max);
            if (lam_max < 1e-15) break;
            for (int i = 0; i < QP_N; ++i) q[i] = Hq[i] / lam_max;
        }
        // Gershgorin lower bound for lambda_min
        double lam_min = 1e9;
        for (int i = 0; i < QP_N; ++i) {
            double diag = mpc.H(i,i), off = 0.0;
            for (int j = 0; j < QP_N; ++j) if (j != i) off += std::abs(mpc.H(i,j));
            lam_min = std::min(lam_min, diag - off);
        }
        lam_min = std::max(lam_min, 1e-6);
        mpc.rho_star = std::max({10.0 * lam_max,
                                  std::sqrt(lam_min * lam_max),
                                  0.5});
    }

    return mpc;
}

// Compute g vector for shifted state dx = x - [ref, 0]
static VecN compute_g(const MPCData& mpc, const Vec2& x, double ref) {
    Vec2 dx; dx[0] = x[0] - ref; dx[1] = x[1];
    VecN g{};
    for (int k = 0; k < QP_N; ++k)
        g[k] = mpc.F_mat(k, 0) * dx[0] + mpc.F_mat(k, 1) * dx[1];
    return g;
}

// ============================================================
//  §5  CHOLESKY + ADMM SOLVE  (manual, consistent with qp_solver.hpp)
//      We implement inline to stay self-contained; the logic is
//      identical to ADMMSolver<N> in qp_solver.hpp.
// ============================================================

// Lower-triangular Cholesky:  L * L' = M
static void chol(MatN& L, const MatN& M) {
    L = MatN{};
    for (int i = 0; i < QP_N; ++i) {
        for (int j = 0; j <= i; ++j) {
            double s = M(i, j);
            for (int k = 0; k < j; ++k) s -= L(i,k) * L(j,k);
            L(i, j) = (i == j) ? std::sqrt(std::max(s, 1e-18))
                                : s / L(j, j);
        }
    }
}

// Solve  L * L' * x = b
static VecN chol_solve(const MatN& L, const VecN& b) {
    VecN y{}, x{};
    for (int i = 0; i < QP_N; ++i) {
        double s = b[i];
        for (int k = 0; k < i; ++k) s -= L(i,k) * y[k];
        y[i] = s / L(i,i);
    }
    for (int i = QP_N-1; i >= 0; --i) {
        double s = y[i];
        for (int k = i+1; k < QP_N; ++k) s -= L(k,i) * x[k];
        x[i] = s / L(i,i);
    }
    return x;
}

struct ADMMState {
    VecN xv{}, z{}, u{};   // primal, aux, dual (unscaled)
    MatN L;                  // Cholesky factor of (H + rho*I)
    double rho = 1.0;
    bool factored = false;

    void setup(const MatN& H, double rho_) {
        rho = rho_;
        MatN Hreg = H;
        for (int i = 0; i < QP_N; ++i) Hreg(i,i) += rho;
        chol(L, Hreg);
        factored = true;
    }

    void reset() { xv.fill(0); z.fill(0); u.fill(0); }

    // Returns u_opt[0] (first control action)
    double solve(const VecN& g, double lb_val, double ub_val,
                 int max_iter = 200,
                 double eps_abs = 1e-4, double eps_rel = 1e-3) {
        for (int iter = 0; iter < max_iter; ++iter) {
            // x-update: (H + rho*I)*x = rho*z - u - g
            VecN rhs{};
            for (int i = 0; i < QP_N; ++i)
                rhs[i] = rho * z[i] - u[i] - g[i];
            xv = chol_solve(L, rhs);

            // z-update: clip(x + u/rho, lb, ub)
            VecN z_prev = z;
            for (int i = 0; i < QP_N; ++i) {
                double v = xv[i] + u[i] / rho;
                z[i] = std::max(lb_val, std::min(ub_val, v));
            }

            // dual update: u += rho*(x - z)
            for (int i = 0; i < QP_N; ++i)
                u[i] += rho * (xv[i] - z[i]);

            // convergence check (every 5 iters)
            if ((iter+1) % 5 == 0) {
                double prim = 0.0, dual = 0.0;
                for (int i = 0; i < QP_N; ++i) {
                    prim += (xv[i]-z[i])*(xv[i]-z[i]);
                    dual += rho*rho*(z[i]-z_prev[i])*(z[i]-z_prev[i]);
                }
                prim = std::sqrt(prim); dual = std::sqrt(dual);
                double eps_p = eps_abs * std::sqrt(QP_N)
                             + eps_rel * std::max(std::sqrt(
                                   [&]{ double s=0; for(auto v:xv) s+=v*v; return s; }()),
                                   std::sqrt(
                                   [&]{ double s=0; for(auto v:z)  s+=v*v; return s; }()));
                double eps_d = eps_abs * std::sqrt(QP_N)
                             + eps_rel * rho * std::sqrt(
                                   [&]{ double s=0; for(auto v:u)  s+=v*v; return s; }());
                if (prim < eps_p && dual < eps_d) break;
            }
        }
        return z[0];   // first optimal control action (shifted domain)
    }
};

// ============================================================
//  §6  CLOSED-LOOP SIMULATION
// ============================================================

struct SimResult {
    std::vector<double> t_vec, pos_vec, vel_vec, u_vec;
};

static SimResult run_closed_loop(const MPCData& mpc,
                                  double u_lim,
                                  int    N_steps,
                                  Vec2   x0 = {0.0, 0.0},
                                  double ref = X_REF) {
    ADMMState admm;
    admm.setup(mpc.H, mpc.rho_star);
    // warm start = true: do NOT reset between steps

    SimResult res;
    Vec2 x = x0;

    for (int k = 0; k < N_steps; ++k) {
        res.t_vec.push_back(k * Ts);
        res.pos_vec.push_back(x[0]);
        res.vel_vec.push_back(x[1]);

        VecN g = compute_g(mpc, x, ref);
        double u_shift = admm.solve(g, -u_lim, u_lim);
        // u_shift is optimal for shifted state; actual u is same (reference is constant)
        double u_act = std::max(-u_lim, std::min(u_lim, u_shift));
        res.u_vec.push_back(u_act);

        x = plant_step(x, u_act);
    }
    // Final state time point (no control)
    res.t_vec.push_back(N_steps * Ts);
    res.pos_vec.push_back(x[0]);
    res.vel_vec.push_back(x[1]);
    res.u_vec.push_back(0.0);

    return res;
}

// ============================================================
//  §7  FIGURE GENERATION
// ============================================================

void fig_exp3_trajectory(const std::string& out) {
    // ── Build MPC problem once ──────────────────────────────
    printf("Building MPC (DARE + Hessian)...\n");
    MPCData mpc = build_mpc();
    printf("  rho* = %.4f\n", mpc.rho_star);

    // ── Three constraint scenarios ──────────────────────────
    struct Scenario {
        std::string label;   // column title
        double u_lim;        // |u| <= u_lim
        int    N_steps;      // simulation length
        std::string color;   // line color
    };
    const Scenario scenarios[] = {
        {"Unconstrained  |u| ≤ 10", 10.0,  500,  "b"},
        {"Light  |u| ≤ 1",           1.0,  800,  "r"},
        {"Tight  |u| ≤ 0.5",         0.5, 1500,  "m"},
    };

    // ── Run simulations ─────────────────────────────────────
    printf("Running closed-loop simulations...\n");
    std::array<SimResult, 3> sims;
    for (int i = 0; i < 3; ++i) {
        sims[i] = run_closed_loop(mpc, scenarios[i].u_lim, scenarios[i].N_steps);
        printf("  %s: %d steps\n", scenarios[i].label.c_str(), scenarios[i].N_steps);
    }

    // ── Plot: 3 cols × 3 rows ───────────────────────────────
    // Layout: subplot(3, 3, idx)  idx = 1..9  (row-major, top-to-bottom)
    //   Row 0 (pos):   idx = 1, 2, 3   ← top
    //   Row 1 (vel):   idx = 4, 5, 6
    //   Row 2 (ctrl):  idx = 7, 8, 9   ← bottom
    //
    // Confirmed CppPlot API:
    //   Figure fig(w,h); auto& ax = fig.subplot(rows,cols,idx);
    //   ax.plot(x,y,"fmt",{{"label","str"}}); ax.set_title/xlabel/ylabel(str);
    //   ax.set_xlim/ylim(a,b); ax.grid(bool); ax.legend(bool); fig.savefig(str);


    Figure fig(1500, 900);

    // Colors: unconstrained=blue, light=red, tight=magenta
    const std::string fmt_line[] = {"b-", "r-", "m-"};

    // u_lim label with %g to strip trailing zeros: 10, 1, 0.5
    auto lim_str = [](double u) -> std::string {
        char buf[16]; std::snprintf(buf, sizeof(buf), "%g", u);
        return std::string(buf);
    };

    for (int col = 0; col < 3; ++col) {
        const SimResult& s  = sims[col];
        const Scenario&  sc = scenarios[col];
        const double u_lim  = sc.u_lim;
        const double t0     = s.t_vec.front();
        const double tN     = s.t_vec.back();

        // Explicit index: pos=top, vel=middle, ctrl=bottom
        const int idx_pos  = col + 7;   // row 0: 1,2,3
        const int idx_vel  = col + 4;   // row 1: 4,5,6
        const int idx_ctrl = col + 1;   // row 2: 7,8,9

        // Row 0: Position (top row)
        {
            auto& ax = fig.subplot(3, 3, idx_pos);
            ax.plot(s.t_vec, s.pos_vec, fmt_line[col].c_str(),
                    {{"label", std::string("position")}});
            ax.plot({t0, tN}, {X_REF, X_REF}, "y--",
                    {{"label", std::string("reference = 2")}});
            ax.set_title(sc.label);
            ax.set_xlabel("Time [s]");
            ax.set_ylabel("Pos [m]");
            ax.grid(true);
            ax.legend(true);
        }

        // Row 1: Velocity
        {
            auto& ax = fig.subplot(3, 3, idx_vel);
            ax.plot(s.t_vec, s.vel_vec, fmt_line[col].c_str(), {});
            ax.plot({t0, tN}, {0.0, 0.0}, "k-", {});
            ax.set_title("Velocity profile");
            ax.set_xlabel("Time [s]");
            ax.set_ylabel("Vel [m/s]");
            ax.grid(true);
        }

        // Row 2: Control input (bottom row)
        {
            auto& ax = fig.subplot(3, 3, idx_ctrl);
            ax.plot(s.t_vec, s.u_vec, fmt_line[col].c_str(),
                    {{"label", std::string("u")}});
            std::string lim_label = std::string("\xc2\xb1") + lim_str(u_lim);
            ax.plot({t0, tN}, { u_lim,  u_lim}, "y--", {{"label", lim_label}});
            ax.plot({t0, tN}, {-u_lim, -u_lim}, "y--", {});
            ax.set_title("Control input");
            ax.set_xlabel("Time [s]");
            ax.set_ylabel("u [m/s2]");
            ax.set_ylim(-u_lim * 1.3, u_lim * 1.3);
            ax.grid(true);
            ax.legend(true);
        }
    }

    fig.savefig(out);
    printf("  -> %s\n", out.c_str());
}

// ============================================================
//  §8  MAIN
// ============================================================
int main() {
    printf("=== demo_mpc_trajectory.cpp  V2 ===\n");
    printf("EXP 3 -- AGV MPC Closed-Loop Trajectory\n\n");

    fig_exp3_trajectory("fig_exp3_trajectory.svg");

    printf("\nDone.\n");
    return 0;
}
