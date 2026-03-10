/**
 * @file example_agv_empc.cpp
 * @brief AGV double integrator — EmbeddedMPC<2,1,10> demo
 *
 * Plant:  x = [pos, vel]', u = accel, dt = 0.01s
 *   A = [[1, dt], [0, 1]]
 *   B = [[0.5*dt^2], [dt]]
 *
 * MPC:  N=10, Q=diag(10,1), R=0.1, Pf=DARE
 *       |u| <= 1.0 m/s^2
 *
 * Compile:
 *   g++ -std=c++14 -O2 -I../include example_agv_empc.cpp -o agv_empc
 *
 * @author DCAS Lab, TDTU
 */

#include "cppplot/control/mpc_embedded.hpp"
#include <cstdio>

using namespace dcas::control;

// ── System dimensions ─────────────────────────────────────
static constexpr int NX   = 2;   // state: [pos, vel]
static constexpr int NU   = 1;   // input: accel
static constexpr int NHOR = 10;  // prediction horizon

using EMPC = EmbeddedMPC<NX, NU, NHOR>;

// ── Simple DARE approximation (value iteration) ───────────
// Replace with cppplot::control::dare() in full project.
static Mat<NX> dare_approx(const Mat<NX>& A, const double B[NX][NU],
                             const Mat<NX>& Q, const Mat<NU>& R,
                             int iters = 500)
{
    Mat<NX> P = Q;
    for (int it = 0; it < iters; ++it) {
        // S = R + B'*P*B  (NU x NU)
        double S[NU][NU] = {};
        for (int i = 0; i < NU; ++i)
            for (int j = 0; j < NU; ++j)
                for (int r = 0; r < NX; ++r)
                    for (int q = 0; q < NX; ++q)
                        S[i][j] += B[r][i] * P(r, q) * B[q][j];
        S[0][0] += R(0, 0);   // NU=1: scalar

        // K = S^{-1} * B' * P * A  (NU x NX)
        // S^{-1} scalar for NU=1
        double Sinv = 1.0 / S[0][0];
        double K[NU][NX] = {};
        for (int j = 0; j < NX; ++j) {
            double BtPA = 0.0;
            for (int r = 0; r < NX; ++r)
                for (int q = 0; q < NX; ++q)
                    BtPA += B[r][0] * P(r, q) * A(q, j);
            K[0][j] = Sinv * BtPA;
        }

        // P_new = Q + A'*P*A - A'*P*B*K
        Mat<NX> Pnew;
        for (int i = 0; i < NX; ++i)
            for (int j = 0; j < NX; ++j) {
                double s = Q(i, j);
                for (int r = 0; r < NX; ++r)
                    for (int q = 0; q < NX; ++q)
                        s += A(r, i) * P(r, q) * A(q, j);
                for (int r = 0; r < NX; ++r)
                    for (int q = 0; q < NX; ++q)
                        s -= A(r, i) * P(r, 0) * B[0][0] * K[0][j];
                Pnew(i, j) = s;
            }
        P = Pnew;
    }
    return P;
}

int main()
{
    // ── System matrices ────────────────────────────────────
    const double dt = 0.01;
    Mat<NX> A;
    A(0,0)=1.0; A(0,1)=dt;
    A(1,0)=0.0; A(1,1)=1.0;

    double B[NX][NU];
    B[0][0] = 0.5 * dt * dt;
    B[1][0] = dt;

    // ── Weights ────────────────────────────────────────────
    Mat<NX> Q;  Q(0,0)=10.0; Q(1,1)=1.0;
    Mat<NU> R;  R(0,0)=0.1;
    Mat<NX> Pf = dare_approx(A, B, Q, R, 500);

    // ── Bounds ─────────────────────────────────────────────
    EMPC::BoundVec u_lb, u_ub;
    u_lb[0] = -1.0;
    u_ub[0] = +1.0;

    // ── ADMM params ────────────────────────────────────────
    EMPC::Params params;
    params.max_iter   = 200;
    params.eps_abs    = 1e-4;
    params.eps_rel    = 1e-3;
    params.warm_start = true;
    // rho will be auto-tuned by make_embedded_mpc()

    // ── Build controller ───────────────────────────────────
    auto ctrl = make_embedded_mpc<NX, NU, NHOR>(A, B, Q, R, Pf,
                                                 u_lb, u_ub, params);

    printf("rho_opt = %.4f\n", ctrl.getParams().rho);

    // ── Closed-loop simulation (500 steps = 5s) ────────────
    EMPC::StateVec x;
    x[0] = 1.0;   // initial pos = 1m (regulation to origin)
    x[1] = 0.0;

    printf("%-6s %-10s %-10s %-10s %-6s %-8s\n",
           "step", "pos", "vel", "u", "iters", "conv");

    int n_steps = 500;
    int n_failed = 0;

    for (int step = 0; step < n_steps; ++step) {
        auto sol = ctrl.solve(x);

        if (!sol.converged) n_failed++;

        if (step % 50 == 0) {
            printf("%-6d %-10.5f %-10.5f %-10.5f %-6d %-8s\n",
                   step, x[0], x[1], sol.u[0],
                   sol.iterations,
                   sol.converged ? "OK" : "FAIL");
        }

        // Apply u_0, simulate plant: x = A*x + B*u
        double u0 = sol.u[0];
        double x0_new = A(0,0)*x[0] + A(0,1)*x[1] + B[0][0]*u0;
        double x1_new = A(1,0)*x[0] + A(1,1)*x[1] + B[1][0]*u0;
        x[0] = x0_new;
        x[1] = x1_new;
    }

    printf("\nFinal state: pos=%.6f  vel=%.6f\n", x[0], x[1]);
    printf("Non-converged steps: %d / %d\n", n_failed, n_steps);

    return 0;
}
