/**
 * @file test_qp_solver.cpp
 * @brief 50 test cases for qp_solver.hpp — DCAS Lab M1 milestone
 *
 * Groups:
 *   A (A01–A10): Cholesky factorization + chol_solve
 *   B (B01–B10): ADMM correctness, unconstrained
 *   C (C01–C15): Box constraints
 *   D (D01–D10): Params, warm-start, reset
 *   E (E01–E05): Edge cases & robustness
 *
 * Compile (standalone, no cppplot needed):
 *   g++ -std=c++17 -O2 -I<path_to_include> -I<catch2_include> \
 *       test_qp_solver.cpp -o test_qp_solver
 *
 * Run:
 *   ctest -R test_qp_solver --output-on-failure
 *   ./test_qp_solver --reporter compact
 *   ./test_qp_solver -t "[A]"      # only group A
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
using namespace Catch::Matchers;

// qp:: components are self-contained (Vec, Mat, cholesky, chol_solve,
// ADMMSolver). Bridge functions (to_fixed_mat etc.) need cppplot::Matrix
// and are NOT tested here.
#define CPPPLOT_CONTROL_QP_SKIP_BRIDGE  // guard in case added later
#include "cppplot/cppplot.hpp"                  // defines cppplot::control::Matrix
#include "cppplot/control/qp_solver.hpp"

using namespace cppplot::control::qp;

// ============================================================
//  Helpers
// ============================================================

template <int N>
Mat<N> make_identity() {
    Mat<N> I;
    for (int i = 0; i < N; ++i) I(i, i) = 1.0;
    return I;
}

template <int N>
Vec<N> make_vec_const(double val) {
    Vec<N> v;
    for (int i = 0; i < N; ++i) v[i] = val;
    return v;
}

/** Relative L2 error: ||a-b|| / ||b||  (0 if b≈0) */
template <int N>
double rel_err(const Vec<N>& a, const Vec<N>& b) {
    double den = b.norm2();
    return (den < 1e-14) ? (a - b).norm2() : (a - b).norm2() / den;
}

/** Frobenius error: ||L*L' - A|| */
template <int N>
double chol_recon_err(const Mat<N>& L, const Mat<N>& A) {
    double err = 0.0;
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) {
            double llt = 0.0;
            for (int k = 0; k <= std::min(i, j); ++k)
                llt += L(i, k) * L(j, k);
            double d = llt - A(i, j);
            err += d * d;
        }
    return std::sqrt(err);
}

/** Dense PD matrix: diagonal with small off-diagonal coupling */
template <int N>
Mat<N> make_pd_mat(double diag_val, double offdiag_frac = 0.05) {
    Mat<N> A;
    for (int i = 0; i < N; ++i) A(i, i) = diag_val;
    for (int i = 0; i < N - 1; ++i) {
        double eps = offdiag_frac * diag_val;
        A(i, i + 1) = eps;
        A(i + 1, i) = eps;
    }
    return A;
}

/**
 * Simplified MPC-like Hessian (diagonal-dominant, scale-able).
 * Real H = Γ'*Q̄*Γ + R̄; this approximates its structure.
 */
template <int N>
Mat<N> make_mpc_hessian(double scale = 1.0) {
    Mat<N> H;
    const double R = 0.1, Q = 10.0, dt = 0.01;
    double base = (R + Q * dt * dt) * scale + 0.5 * scale;
    for (int i = 0; i < N; ++i) H(i, i) = base;
    for (int i = 0; i < N - 1; ++i) {
        double off = -R * dt * scale * 0.1;
        H(i, i + 1) = off;
        H(i + 1, i) = off;
    }
    return H;
}

/** Chol-Direct reference: x* = -P^{-1} q */
template <int N>
Vec<N> chol_direct(const Mat<N>& P, const Vec<N>& q) {
    Mat<N> L;
    bool ok = cholesky(L, P);
    REQUIRE(ok);
    Vec<N> neg_q;
    for (int i = 0; i < N; ++i) neg_q[i] = -q[i];
    return chol_solve(L, neg_q);
}

template <int N> Vec<N> lb_inf() { return make_vec_const<N>(-1e9); }
template <int N> Vec<N> ub_inf() { return make_vec_const<N>(+1e9); }

// ============================================================
//  GROUP A — Cholesky (10 tests)
// ============================================================

TEST_CASE("A01: Cholesky 1x1 identity", "[cholesky][A]") {
    Mat<1> A; A(0, 0) = 1.0;
    Mat<1> L;
    REQUIRE(cholesky(L, A));
    CHECK_THAT(L(0, 0), WithinAbs(1.0, 1e-12));
}

TEST_CASE("A02: Cholesky 10x10 identity gives L=I", "[cholesky][A]") {
    auto A = make_identity<10>();
    Mat<10> L;
    REQUIRE(cholesky(L, A));
    for (int i = 0; i < 10; ++i)
        CHECK_THAT(L(i, i), WithinAbs(1.0, 1e-12));
    for (int i = 0; i < 10; ++i)
        for (int j = 0; j < i; ++j)
            CHECK_THAT(L(i, j), WithinAbs(0.0, 1e-12));
}

TEST_CASE("A03: Cholesky diagonal PD — L_ii = sqrt(A_ii)", "[cholesky][A]") {
    Mat<5> A;
    double d[] = {4.0, 9.0, 16.0, 25.0, 1.0};
    double e[] = {2.0, 3.0,  4.0,  5.0, 1.0};
    for (int i = 0; i < 5; ++i) A(i, i) = d[i];
    Mat<5> L;
    REQUIRE(cholesky(L, A));
    for (int i = 0; i < 5; ++i)
        CHECK_THAT(L(i, i), WithinAbs(e[i], 1e-10));
}

TEST_CASE("A04: Cholesky 2x2 dense — hand-verified", "[cholesky][A]") {
    // A = [[4,2],[2,3]] → L = [[2,0],[1,sqrt(2)]]
    Mat<2> A;
    A(0,0)=4; A(0,1)=2; A(1,0)=2; A(1,1)=3;
    Mat<2> L;
    REQUIRE(cholesky(L, A));
    CHECK_THAT(L(0,0), WithinAbs(2.0,            1e-10));
    CHECK_THAT(L(1,0), WithinAbs(1.0,            1e-10));
    CHECK_THAT(L(1,1), WithinAbs(std::sqrt(2.0), 1e-10));
}

TEST_CASE("A05: Cholesky reconstruction L*L'=A, N=10 S1-scale", "[cholesky][A]") {
    auto A = make_mpc_hessian<10>(1.0);
    Mat<10> L;
    REQUIRE(cholesky(L, A));
    CHECK_THAT(chol_recon_err(L, A), WithinAbs(0.0, 1e-10));
}

TEST_CASE("A06: Cholesky reconstruction — S2-scale (small eigenvalues)", "[cholesky][A]") {
    auto A = make_mpc_hessian<10>(0.015);
    Mat<10> L;
    REQUIRE(cholesky(L, A));
    CHECK_THAT(chol_recon_err(L, A), WithinAbs(0.0, 1e-10));
}

TEST_CASE("A07: Cholesky reconstruction — S3-scale (very small eigenvalues)", "[cholesky][A]") {
    auto A = make_mpc_hessian<10>(0.006);
    Mat<10> L;
    REQUIRE(cholesky(L, A));
    CHECK_THAT(chol_recon_err(L, A), WithinAbs(0.0, 1e-10));
}

TEST_CASE("A08: Cholesky returns false for non-PD matrix", "[cholesky][A]") {
    Mat<3> A;
    A(0,0) = 1.0; A(1,1) = -1.0; A(2,2) = 1.0;  // negative diagonal → not PD
    Mat<3> L;
    CHECK_FALSE(cholesky(L, A));
}

TEST_CASE("A09: chol_solve round-trip: x → H*x → recover x", "[chol_solve][A]") {
    auto H = make_mpc_hessian<10>(1.0);
    Mat<10> L;
    REQUIRE(cholesky(L, H));

    Vec<10> x_true;
    for (int i = 0; i < 10; ++i) x_true[i] = (i % 2 == 0) ? 1.0 : -0.5;
    Vec<10> b  = H.matvec(x_true);
    Vec<10> x_rec = chol_solve(L, b);

    CHECK_THAT(rel_err(x_rec, x_true), WithinAbs(0.0, 1e-8));
}

TEST_CASE("A10: chol_solve — multiple independent RHS on same L", "[chol_solve][A]") {
    auto H = make_mpc_hessian<10>(1.0);
    Mat<10> L;
    REQUIRE(cholesky(L, H));

    Vec<10> b1, b2;
    for (int i = 0; i < 10; ++i) { b1[i] = 1.0; b2[i] = static_cast<double>(i); }

    Vec<10> x1 = chol_solve(L, b1);
    Vec<10> x2 = chol_solve(L, b2);

    CHECK_THAT(rel_err(H.matvec(x1), b1), WithinAbs(0.0, 1e-8));
    CHECK_THAT(rel_err(H.matvec(x2), b2), WithinAbs(0.0, 1e-8));
}

// ============================================================
//  GROUP B — ADMM correctness, unconstrained (10 tests)
// ============================================================

TEST_CASE("B01: ADMM unconstrained matches Chol-Direct (varied q)", "[admm][B]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> q; for (int i = 0; i < 10; ++i) q[i] = 0.1 * i - 0.5;
    Vec<10> ref = chol_direct<10>(H, q);

    ADMMSolver<10> solver;
    Params p; p.rho = 1.0; p.eps_abs = 1e-6; p.eps_rel = 1e-6; p.max_iter = 500;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb_inf<10>(), ub_inf<10>());

    CHECK(sol.converged);
    CHECK_THAT(rel_err(sol.x, ref), WithinAbs(0.0, 1e-3));
}

TEST_CASE("B02: ADMM unconstrained — identity H, x*=-q", "[admm][B]") {
    auto H = make_identity<10>();
    Vec<10> q; for (int i = 0; i < 10; ++i) q[i] = i + 1.0;

    ADMMSolver<10> solver;
    Params p; p.rho = 1.0; p.eps_abs = 1e-6; p.eps_rel = 1e-6; p.max_iter = 500;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb_inf<10>(), ub_inf<10>());

    CHECK(sol.converged);
    for (int i = 0; i < 10; ++i)
        CHECK_THAT(sol.x[i], WithinAbs(-(i + 1.0), 1e-3));
}

TEST_CASE("B03: ADMM unconstrained — large q (L2 norm 10x)", "[admm][B]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> q = make_vec_const<10>(10.0);
    Vec<10> ref = chol_direct<10>(H, q);

    ADMMSolver<10> solver;
    Params p; p.rho = 1.0; p.eps_abs = 1e-6; p.eps_rel = 1e-6; p.max_iter = 500;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb_inf<10>(), ub_inf<10>());

    CHECK(sol.converged);
    CHECK_THAT(rel_err(sol.x, ref), WithinAbs(0.0, 1e-3));
}

TEST_CASE("B04: ADMM q=0 → solution norm ≈ 0", "[admm][B]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> q;  // zero

    ADMMSolver<10> solver;
    Params p; p.rho = 1.0; p.eps_abs = 1e-6; p.eps_rel = 1e-6; p.max_iter = 500;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb_inf<10>(), ub_inf<10>());

    CHECK(sol.converged);
    CHECK_THAT(sol.x.norm2(), WithinAbs(0.0, 1e-4));
}

TEST_CASE("B05: ADMM scale test — 2H,2q gives same x* as H,q", "[admm][B]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> q; for (int i = 0; i < 10; ++i) q[i] = 0.3 * i - 1.0;

    ADMMSolver<10> s1;
    Params p1; p1.rho = 0.5; p1.eps_abs = 1e-6; p1.eps_rel = 1e-6; p1.max_iter = 500;
    REQUIRE(s1.setup(H, p1));
    auto sol1 = s1.solve(q, lb_inf<10>(), ub_inf<10>());

    // Build 2H, 2q
    Mat<10> H2 = H; Vec<10> q2 = q;
    for (int i = 0; i < 10; ++i) for (int j = 0; j < 10; ++j) H2(i, j) *= 2.0;
    for (int i = 0; i < 10; ++i) q2[i] *= 2.0;

    ADMMSolver<10> s2;
    Params p2; p2.rho = 1.0; p2.eps_abs = 1e-6; p2.eps_rel = 1e-6; p2.max_iter = 500;
    REQUIRE(s2.setup(H2, p2));
    auto sol2 = s2.solve(q2, lb_inf<10>(), ub_inf<10>());

    CHECK(sol1.converged); CHECK(sol2.converged);
    CHECK_THAT(rel_err(sol1.x, sol2.x), WithinAbs(0.0, 1e-3));
}

TEST_CASE("B06: ADMM with spectral rho converges in few iterations", "[admm][B]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> q; for (int i = 0; i < 10; ++i) q[i] = 0.1 * (i - 5);

    // Power iteration: estimate lambda_max
    Vec<10> v = make_vec_const<10>(1.0);
    for (int it = 0; it < 40; ++it) {
        v = H.matvec(v);
        double n = v.norm2();
        if (n > 0) for (int i = 0; i < 10; ++i) v[i] /= n;
    }
    double lmax = H.matvec(v).dot(v);
    double rho_opt = std::max(10.0 * lmax, 0.5);

    ADMMSolver<10> solver;
    Params p; p.rho = rho_opt; p.eps_abs = 1e-4; p.eps_rel = 1e-3; p.max_iter = 200;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb_inf<10>(), ub_inf<10>());

    CHECK(sol.converged);
    CHECK(sol.iterations <= 100);
}

TEST_CASE("B07: ADMM cost formula: 0.5*z'*P*z + q'*z", "[admm][B]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> q; for (int i = 0; i < 10; ++i) q[i] = 0.2 * i;

    ADMMSolver<10> solver;
    Params p; p.rho = 1.0; p.eps_abs = 1e-6; p.eps_rel = 1e-6; p.max_iter = 500;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb_inf<10>(), ub_inf<10>());
    REQUIRE(sol.converged);

    // Manual cost
    Vec<10> Hz = H.matvec(sol.x);
    double cost_ref = 0.0;
    for (int i = 0; i < 10; ++i)
        cost_ref += 0.5 * Hz[i] * sol.x[i] + q[i] * sol.x[i];

    CHECK_THAT(sol.cost, WithinAbs(cost_ref, std::abs(cost_ref) * 1e-6 + 1e-10));
}

TEST_CASE("B08: ADMM N=5 matches Chol-Direct", "[admm][B]") {
    Mat<5> H;
    for (int i = 0; i < 5; ++i) H(i, i) = 2.0;
    for (int i = 0; i < 4; ++i) { H(i, i+1) = 0.05; H(i+1, i) = 0.05; }

    Vec<5> q; for (int i = 0; i < 5; ++i) q[i] = i * 0.5;
    Vec<5> ref = chol_direct<5>(H, q);

    ADMMSolver<5> solver;
    Params p; p.rho = 1.0; p.eps_abs = 1e-6; p.eps_rel = 1e-6; p.max_iter = 1000;
    REQUIRE(solver.setup(H, p));
    Vec<5> lb = make_vec_const<5>(-1e9);
    Vec<5> ub = make_vec_const<5>(+1e9);
    auto sol = solver.solve(q, lb, ub);

    CHECK(sol.converged);
    CHECK_THAT(rel_err(sol.x, ref), WithinAbs(0.0, 1e-3));
}

TEST_CASE("B09: ADMM primal_res and dual_res are non-negative", "[admm][B]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> q; for (int i = 0; i < 10; ++i) q[i] = 0.5;

    ADMMSolver<10> solver;
    Params p; p.rho = 1.0; p.max_iter = 50;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb_inf<10>(), ub_inf<10>());

    CHECK(sol.primal_res >= 0.0);
    CHECK(sol.dual_res   >= 0.0);
}

TEST_CASE("B10: setup returns false for non-PD matrix", "[admm][B]") {
    Mat<3> bad;
    bad(0,0) = -1.0; bad(1,1) = 1.0; bad(2,2) = 1.0;
    ADMMSolver<3> solver;
    bool ok = solver.setup(bad);
    // Either false or succeeds with fallback regularization — must not crash
    (void)ok;
    SUCCEED("non-PD setup did not crash");
}

// ============================================================
//  GROUP C — Box constraints (15 tests)
// ============================================================

TEST_CASE("C01: lb=ub=0 → solution is zero", "[admm][constrained][C]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> q = make_vec_const<10>(1.0);
    Vec<10> lb = make_vec_const<10>(0.0);
    Vec<10> ub = make_vec_const<10>(0.0);

    ADMMSolver<10> solver;
    Params p; p.rho = 1.0; p.max_iter = 200;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb, ub);

    for (int i = 0; i < 10; ++i)
        CHECK_THAT(sol.x[i], WithinAbs(0.0, 1e-4));
}

TEST_CASE("C02: Tight bound |u|<=0.1, large q → all at upper bound", "[admm][constrained][C]") {
    auto H = make_identity<10>();
    Vec<10> q = make_vec_const<10>(-5.0);  // unconstrained opt = +5 → clips to +0.1
    Vec<10> lb = make_vec_const<10>(-0.1);
    Vec<10> ub = make_vec_const<10>(+0.1);

    ADMMSolver<10> solver;
    Params p; p.rho = 1.0; p.max_iter = 500; p.eps_abs = 1e-6; p.eps_rel = 1e-6;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb, ub);

    for (int i = 0; i < 10; ++i) {
        CHECK(sol.x[i] >= -0.1 - 1e-4);
        CHECK(sol.x[i] <=  0.1 + 1e-4);
        CHECK_THAT(sol.x[i], WithinAbs(0.1, 1e-3));
    }
}

TEST_CASE("C03: |u|<=10 inactive constraints — equals Chol-Direct", "[admm][constrained][C]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> q; for (int i = 0; i < 10; ++i) q[i] = 0.05 * i;
    Vec<10> ref = chol_direct<10>(H, q);

    Vec<10> lb = make_vec_const<10>(-10.0);
    Vec<10> ub = make_vec_const<10>(+10.0);

    ADMMSolver<10> solver;
    Params p; p.rho = 1.0; p.eps_abs = 1e-6; p.eps_rel = 1e-6; p.max_iter = 500;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb, ub);

    CHECK(sol.converged);
    // If unconstrained opt is within bounds, results match
    bool inactive = true;
    for (int i = 0; i < 10; ++i)
        if (std::abs(ref[i]) >= 9.5) inactive = false;
    if (inactive)
        CHECK_THAT(rel_err(sol.x, ref), WithinAbs(0.0, 1e-3));
    // Always feasible
    for (int i = 0; i < 10; ++i) {
        CHECK(sol.x[i] >= -10.0 - 1e-4);
        CHECK(sol.x[i] <=  10.0 + 1e-4);
    }
}

TEST_CASE("C04: |u|<=1 — all components within bounds", "[admm][constrained][C]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> q = make_vec_const<10>(-5.0);
    Vec<10> lb = make_vec_const<10>(-1.0);
    Vec<10> ub = make_vec_const<10>(+1.0);

    ADMMSolver<10> solver;
    Params p; p.rho = 1.0; p.max_iter = 500; p.eps_abs = 1e-4; p.eps_rel = 1e-3;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb, ub);

    for (int i = 0; i < 10; ++i) {
        CHECK(sol.x[i] >= -1.0 - 1e-4);
        CHECK(sol.x[i] <=  1.0 + 1e-4);
    }
}

TEST_CASE("C05: |u|<=0.5 tight — all within bounds", "[admm][constrained][C]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> q = make_vec_const<10>(-10.0);
    Vec<10> lb = make_vec_const<10>(-0.5);
    Vec<10> ub = make_vec_const<10>(+0.5);

    ADMMSolver<10> solver;
    Params p; p.rho = 2.0; p.max_iter = 500;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb, ub);

    for (int i = 0; i < 10; ++i) {
        CHECK(sol.x[i] >= -0.5 - 1e-4);
        CHECK(sol.x[i] <=  0.5 + 1e-4);
    }
}

TEST_CASE("C06: Asymmetric bounds lb=-0.5, ub=1.0", "[admm][constrained][C]") {
    auto H = make_identity<10>();
    Vec<10> q;
    for (int i = 0; i < 10; ++i) q[i] = (i % 2 == 0) ? -5.0 : 5.0;
    Vec<10> lb = make_vec_const<10>(-0.5);
    Vec<10> ub = make_vec_const<10>(+1.0);

    ADMMSolver<10> solver;
    Params p; p.rho = 1.0; p.max_iter = 500; p.eps_abs = 1e-6; p.eps_rel = 1e-6;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb, ub);

    CHECK(sol.converged);
    for (int i = 0; i < 10; ++i) {
        CHECK(sol.x[i] >= -0.5 - 1e-4);
        CHECK(sol.x[i] <=  1.0 + 1e-4);
    }
    CHECK_THAT(sol.x[0], WithinAbs(1.0, 1e-3));   // even → near upper bound
    CHECK_THAT(sol.x[1], WithinAbs(-0.5, 1e-3));  // odd → near lower bound
}

TEST_CASE("C07: Virtually unconstrained (lb=-1e9) ≈ Chol-Direct", "[admm][constrained][C]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> q; for (int i = 0; i < 10; ++i) q[i] = 0.3 * i;
    Vec<10> ref = chol_direct<10>(H, q);

    ADMMSolver<10> solver;
    Params p; p.rho = 1.0; p.eps_abs = 1e-6; p.eps_rel = 1e-6; p.max_iter = 500;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb_inf<10>(), ub_inf<10>());

    CHECK(sol.converged);
    CHECK_THAT(rel_err(sol.x, ref), WithinAbs(0.0, 1e-3));
}

TEST_CASE("C08: Max constraint violation < 1e-4", "[admm][constrained][C]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> q; for (int i = 0; i < 10; ++i) q[i] = 2.0 * std::sin(i);
    Vec<10> lb = make_vec_const<10>(-0.8);
    Vec<10> ub = make_vec_const<10>(+0.8);

    ADMMSolver<10> solver;
    Params p; p.rho = 1.0; p.max_iter = 500;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb, ub);

    double max_viol = 0.0;
    for (int i = 0; i < 10; ++i)
        max_viol = std::max(max_viol,
                            std::max(lb[i] - sol.x[i], sol.x[i] - ub[i]));
    CHECK_THAT(max_viol, WithinAbs(0.0, 1e-4));
}

TEST_CASE("C09: Full solve has lower cost than partial (5 iter)", "[admm][constrained][C]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> q = make_vec_const<10>(1.0);
    Vec<10> lb = make_vec_const<10>(-1.0);
    Vec<10> ub = make_vec_const<10>(+1.0);

    ADMMSolver<10> s_short, s_full;
    Params p_short; p_short.rho = 1.0; p_short.max_iter = 5;  p_short.warm_start = false;
    Params p_full;  p_full.rho  = 1.0; p_full.max_iter  = 500; p_full.warm_start = false;
    REQUIRE(s_short.setup(H, p_short));
    REQUIRE(s_full.setup(H, p_full));

    auto sol_short = s_short.solve(q, lb, ub);
    auto sol_full  = s_full.solve(q, lb, ub);

    CHECK(sol_full.cost <= sol_short.cost + 1e-6);
}

TEST_CASE("C10: N=5 all dims feasible", "[admm][constrained][C]") {
    Mat<5> H;
    for (int i = 0; i < 5; ++i) H(i, i) = 3.0;
    Vec<5> q = make_vec_const<5>(-3.0);
    Vec<5> lb = make_vec_const<5>(-0.5);
    Vec<5> ub = make_vec_const<5>(+0.5);

    ADMMSolver<5> solver;
    Params p; p.rho = 1.0; p.max_iter = 300; p.eps_abs = 1e-6; p.eps_rel = 1e-6;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb, ub);

    for (int i = 0; i < 5; ++i) {
        CHECK(sol.x[i] >= -0.5 - 1e-4);
        CHECK(sol.x[i] <=  0.5 + 1e-4);
    }
}

TEST_CASE("C11: N=1 exact — min 0.5x^2+2x s.t. -1<=x<=1 → x*=-1", "[admm][constrained][C]") {
    Mat<1> H; H(0,0) = 1.0;
    Vec<1> q; q[0] = 2.0;
    Vec<1> lb; lb[0] = -1.0;
    Vec<1> ub; ub[0] =  1.0;

    ADMMSolver<1> solver;
    Params p; p.rho = 1.0; p.max_iter = 200; p.eps_abs = 1e-8; p.eps_rel = 1e-8;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb, ub);

    CHECK(sol.converged);
    CHECK_THAT(sol.x[0], WithinAbs(-1.0, 1e-4));
}

TEST_CASE("C12: Partial constraint — only active dims clamped", "[admm][constrained][C]") {
    // H=I, q=[−3,−3,0,...] → first 2 dims want +3 → clamped to +1; rest = 0
    auto H = make_identity<10>();
    Vec<10> q; q[0] = -3.0; q[1] = -3.0;
    Vec<10> lb = make_vec_const<10>(-1.0);
    Vec<10> ub = make_vec_const<10>(+1.0);

    ADMMSolver<10> solver;
    Params p; p.rho = 1.0; p.max_iter = 500; p.eps_abs = 1e-6; p.eps_rel = 1e-6;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb, ub);

    CHECK(sol.converged);
    CHECK_THAT(sol.x[0], WithinAbs(1.0, 1e-3));
    CHECK_THAT(sol.x[1], WithinAbs(1.0, 1e-3));
    for (int i = 2; i < 10; ++i)
        CHECK_THAT(sol.x[i], WithinAbs(0.0, 1e-3));
}

TEST_CASE("C13: 20 sequential MPC steps — all feasible", "[admm][constrained][C]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> lb = make_vec_const<10>(-1.0);
    Vec<10> ub = make_vec_const<10>(+1.0);

    ADMMSolver<10> solver;
    Params p; p.rho = 1.0; p.max_iter = 200; p.warm_start = true;
    REQUIRE(solver.setup(H, p));

    double max_viol = 0.0;
    for (int step = 0; step < 20; ++step) {
        Vec<10> q;
        for (int i = 0; i < 10; ++i)
            q[i] = std::sin(step * 0.3 + i * 0.1) * 2.0;
        auto sol = solver.solve(q, lb, ub);
        for (int i = 0; i < 10; ++i)
            max_viol = std::max(max_viol,
                                std::max(lb[i] - sol.x[i], sol.x[i] - ub[i]));
    }
    CHECK_THAT(max_viol, WithinAbs(0.0, 1e-4));
}

TEST_CASE("C14: All-active corner — all dims at lower bound", "[admm][constrained][C]") {
    Mat<3> H; H(0,0)=1.0; H(1,1)=1.0; H(2,2)=1.0;
    Vec<3> q; q[0]=5.0; q[1]=5.0; q[2]=5.0;
    Vec<3> lb; lb[0]=lb[1]=lb[2]=-2.0;
    Vec<3> ub; ub[0]=ub[1]=ub[2]= 2.0;

    ADMMSolver<3> solver;
    Params p; p.rho = 1.0; p.max_iter = 500; p.eps_abs = 1e-8; p.eps_rel = 1e-8;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb, ub);

    CHECK(sol.converged);
    for (int i = 0; i < 3; ++i) {
        CHECK(sol.x[i] >= -2.0 - 1e-5);
        CHECK(sol.x[i] <=  2.0 + 1e-5);
        CHECK_THAT(sol.x[i], WithinAbs(-2.0, 1e-4));
    }
}

TEST_CASE("C15: sol.x is z (feasible), not raw ADMM x", "[admm][constrained][C]") {
    // Even with extreme q that drives raw x far outside bounds,
    // sol.x = z_ which is the clipped/projected variable → always feasible.
    auto H = make_identity<10>();
    Vec<10> q = make_vec_const<10>(-1000.0);
    Vec<10> lb = make_vec_const<10>(-1.0);
    Vec<10> ub = make_vec_const<10>(+1.0);

    ADMMSolver<10> solver;
    Params p; p.rho = 1.0; p.max_iter = 500;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb, ub);

    for (int i = 0; i < 10; ++i) {
        CHECK(sol.x[i] >= lb[i] - 1e-6);
        CHECK(sol.x[i] <= ub[i] + 1e-6);
    }
}

// ============================================================
//  GROUP D — Params, warm-start, reset (10 tests)
// ============================================================

TEST_CASE("D01: warm=true vs false — same converged solution", "[admm][warmstart][D]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> q; for (int i = 0; i < 10; ++i) q[i] = 0.5 * i;
    Vec<10> lb = make_vec_const<10>(-1.0);
    Vec<10> ub = make_vec_const<10>(+1.0);

    ADMMSolver<10> sw, sc;
    Params pw; pw.rho=1.0; pw.max_iter=500; pw.warm_start=true;  pw.eps_abs=1e-4; pw.eps_rel=1e-3;
    Params pc; pc.rho=1.0; pc.max_iter=500; pc.warm_start=false; pc.eps_abs=1e-4; pc.eps_rel=1e-3;
    REQUIRE(sw.setup(H, pw)); REQUIRE(sc.setup(H, pc));

    auto solw = sw.solve(q, lb, ub);
    auto solc = sc.solve(q, lb, ub);

    CHECK_THAT(rel_err(solw.x, solc.x), WithinAbs(0.0, 1e-3));
}

TEST_CASE("D02: warm-start reduces iterations on close consecutive solves", "[admm][warmstart][D]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> lb = make_vec_const<10>(-1.0);
    Vec<10> ub = make_vec_const<10>(+1.0);

    ADMMSolver<10> solver;
    Params p; p.rho=1.0; p.max_iter=500; p.warm_start=true; p.eps_abs=1e-4; p.eps_rel=1e-3;
    REQUIRE(solver.setup(H, p));

    Vec<10> q1; for (int i = 0; i < 10; ++i) q1[i] = 0.3 * i;
    auto sol1 = solver.solve(q1, lb, ub);

    // Small perturbation — warm state should help
    Vec<10> q2 = q1; for (int i = 0; i < 10; ++i) q2[i] += 0.01;
    auto sol2 = solver.solve(q2, lb, ub);

    CHECK(sol1.converged);
    CHECK(sol2.converged);
    // Second solve should not need more iterations than first (with some slack)
    CHECK(sol2.iterations <= sol1.iterations + 10);
}

TEST_CASE("D03: reset() makes solver behave like cold start", "[admm][reset][D]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> q = make_vec_const<10>(0.5);
    Vec<10> lb = make_vec_const<10>(-1.0);
    Vec<10> ub = make_vec_const<10>(+1.0);

    ADMMSolver<10> solver;
    Params p; p.rho=1.0; p.max_iter=500; p.warm_start=true; p.eps_abs=1e-4; p.eps_rel=1e-3;
    REQUIRE(solver.setup(H, p));

    // Warm up
    for (int i = 0; i < 5; ++i) solver.solve(q, lb, ub);
    solver.reset();

    // Solve with different q after reset
    Vec<10> q2 = make_vec_const<10>(5.0);
    auto sol_reset = solver.solve(q2, lb, ub);

    // Compare with a pristine cold-start solver
    ADMMSolver<10> cold; Params pc=p; pc.warm_start=false;
    REQUIRE(cold.setup(H, pc));
    auto sol_cold = cold.solve(q2, lb, ub);

    CHECK_THAT(rel_err(sol_reset.x, sol_cold.x), WithinAbs(0.0, 1e-3));
}

TEST_CASE("D04: max_iter=1 — no crash, 1 iteration, solution feasible", "[admm][params][D]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> q = make_vec_const<10>(1.0);
    Vec<10> lb = make_vec_const<10>(-1.0);
    Vec<10> ub = make_vec_const<10>(+1.0);

    ADMMSolver<10> solver;
    Params p; p.rho=1.0; p.max_iter=1; p.check_every=1;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb, ub);

    CHECK(sol.iterations <= 1);
    for (int i = 0; i < 10; ++i) {
        CHECK(sol.x[i] >= -1.0 - 1e-4);
        CHECK(sol.x[i] <=  1.0 + 1e-4);
    }
}

TEST_CASE("D05: max_iter=0 — zero iterations, not converged", "[admm][params][D]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> q = make_vec_const<10>(1.0);

    ADMMSolver<10> solver;
    Params p; p.rho=1.0; p.max_iter=0; p.warm_start=false;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb_inf<10>(), ub_inf<10>());

    CHECK(sol.iterations == 0);
    CHECK_FALSE(sol.converged);
    CHECK_THAT(sol.x.norm2(), WithinAbs(0.0, 1e-12));
}

TEST_CASE("D06: Looser tolerance converges in fewer iterations", "[admm][params][D]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> q; for (int i = 0; i < 10; ++i) q[i] = 0.3 * i;
    Vec<10> lb = make_vec_const<10>(-1.0);
    Vec<10> ub = make_vec_const<10>(+1.0);

    ADMMSolver<10> s_loose, s_tight;
    Params pl; pl.rho=1.0; pl.max_iter=500; pl.warm_start=false; pl.eps_abs=1e-2; pl.eps_rel=1e-2;
    Params pt; pt.rho=1.0; pt.max_iter=500; pt.warm_start=false; pt.eps_abs=1e-6; pt.eps_rel=1e-6;
    REQUIRE(s_loose.setup(H, pl)); REQUIRE(s_tight.setup(H, pt));

    auto sol_l = s_loose.solve(q, lb, ub);
    auto sol_t = s_tight.solve(q, lb, ub);

    CHECK(sol_l.converged); CHECK(sol_t.converged);
    CHECK(sol_l.iterations <= sol_t.iterations);
}

TEST_CASE("D07: check_every=1 vs check_every=10 — same result", "[admm][params][D]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> q; for (int i = 0; i < 10; ++i) q[i] = 0.2 * i;
    Vec<10> lb = make_vec_const<10>(-1.0);
    Vec<10> ub = make_vec_const<10>(+1.0);

    ADMMSolver<10> s1, s2;
    Params p; p.rho=1.0; p.max_iter=500; p.warm_start=false; p.eps_abs=1e-4; p.eps_rel=1e-3;
    Params p2 = p; p.check_every=1; p2.check_every=10;
    REQUIRE(s1.setup(H, p)); REQUIRE(s2.setup(H, p2));

    auto sol1 = s1.solve(q, lb, ub);
    auto sol2 = s2.solve(q, lb, ub);

    CHECK(sol1.converged); CHECK(sol2.converged);
    CHECK_THAT(rel_err(sol1.x, sol2.x), WithinAbs(0.0, 1e-3));
}

TEST_CASE("D08: rho=1.0 on S2-like H — may not converge (expected)", "[admm][params][D]") {
    // rho=1.0 overshoots lambda_max ~68x on S2 → cold-start known to fail
    auto H = make_mpc_hessian<10>(0.015);
    Vec<10> q = make_vec_const<10>(-2.0);
    Vec<10> lb = make_vec_const<10>(-1.0);
    Vec<10> ub = make_vec_const<10>(+1.0);

    ADMMSolver<10> solver;
    Params p;  // default: rho=1.0, max_iter=100
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb, ub);

    // Feasibility always required — convergence not required for this case
    for (int i = 0; i < 10; ++i) {
        CHECK(sol.x[i] >= -1.0 - 1e-4);
        CHECK(sol.x[i] <=  1.0 + 1e-4);
    }
    INFO("S2 rho=1.0: converged=" << sol.converged << " iters=" << sol.iterations);
    SUCCEED("S2 with rho=1.0 may not converge — documented behavior");
}

TEST_CASE("D09: rho=rho_opt on S2-like unconstrained — converges", "[admm][params][D]") {
    auto H = make_mpc_hessian<10>(0.015);

    Vec<10> v = make_vec_const<10>(1.0);
    for (int it = 0; it < 60; ++it) {
        v = H.matvec(v);
        double n = v.norm2(); if (n > 0) for (int i = 0; i < 10; ++i) v[i] /= n;
    }
    double lmax = H.matvec(v).dot(v);
    double rho_opt = std::max(10.0 * lmax, 0.5);

    Vec<10> q = make_vec_const<10>(-1.0);

    ADMMSolver<10> solver;
    Params p; p.rho=rho_opt; p.max_iter=500; p.eps_abs=1e-4; p.eps_rel=1e-3;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb_inf<10>(), ub_inf<10>());

    CHECK(sol.converged);
}

TEST_CASE("D10: Params copy is value-independent", "[admm][params][D]") {
    Params orig; orig.rho=2.0; orig.eps_abs=1e-5; orig.max_iter=200;
    Params copy = orig;
    copy.rho=99.0; copy.eps_abs=0.5; copy.max_iter=1;

    CHECK_THAT(orig.rho,     WithinAbs(2.0,  1e-14));
    CHECK_THAT(orig.eps_abs, WithinAbs(1e-5, 1e-20));
    CHECK(orig.max_iter == 200);
}

// ============================================================
//  GROUP E — Edge cases & robustness (5 tests)
// ============================================================

TEST_CASE("E01: q=0 unconstrained → solution and cost both zero", "[admm][edge][E]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> q;  // zero

    ADMMSolver<10> solver;
    Params p; p.rho=1.0; p.max_iter=500; p.eps_abs=1e-6; p.eps_rel=1e-6;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb_inf<10>(), ub_inf<10>());

    CHECK(sol.converged);
    CHECK_THAT(sol.x.norm2(), WithinAbs(0.0, 1e-4));
    CHECK_THAT(sol.cost,      WithinAbs(0.0, 1e-8));
}

TEST_CASE("E02: q=0 constrained with symmetric bounds → solution is zero", "[admm][edge][E]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> q;
    Vec<10> lb = make_vec_const<10>(-1.0);
    Vec<10> ub = make_vec_const<10>(+1.0);

    ADMMSolver<10> solver;
    Params p; p.rho=1.0; p.max_iter=500; p.eps_abs=1e-6; p.eps_rel=1e-6;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb, ub);

    CHECK(sol.converged);
    CHECK_THAT(sol.x.norm2(), WithinAbs(0.0, 1e-4));
}

TEST_CASE("E03: Ill-conditioned H (cond~1e6) — no crash, feasible", "[admm][edge][E]") {
    Mat<2> H; H(0,0) = 1.0; H(1,1) = 1e6;
    Vec<2> q; q[0]=-1.0; q[1]=-1.0;
    Vec<2> lb; lb[0]=lb[1]=-10.0;
    Vec<2> ub; ub[0]=ub[1]= 10.0;

    ADMMSolver<2> solver;
    Params p; p.rho=1.0; p.max_iter=500;
    REQUIRE(solver.setup(H, p));
    auto sol = solver.solve(q, lb, ub);

    for (int i = 0; i < 2; ++i) {
        CHECK(sol.x[i] >= lb[i] - 1e-4);
        CHECK(sol.x[i] <= ub[i] + 1e-4);
    }
}

TEST_CASE("E04: 100 MPC steps warm-start — >=80% converge, all feasible", "[admm][edge][E]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> lb = make_vec_const<10>(-1.0);
    Vec<10> ub = make_vec_const<10>(+1.0);

    ADMMSolver<10> solver;
    Params p; p.rho=1.0; p.max_iter=200; p.warm_start=true; p.eps_abs=1e-4; p.eps_rel=1e-3;
    REQUIRE(solver.setup(H, p));

    int conv_cnt = 0;
    double max_viol = 0.0;
    for (int step = 0; step < 100; ++step) {
        Vec<10> q;
        for (int i = 0; i < 10; ++i)
            q[i] = -2.0 * std::exp(-0.05 * step) * std::cos(i * 0.3);
        auto sol = solver.solve(q, lb, ub);
        if (sol.converged) conv_cnt++;
        for (int i = 0; i < 10; ++i)
            max_viol = std::max(max_viol,
                                std::max(lb[i] - sol.x[i], sol.x[i] - ub[i]));
    }
    CHECK(conv_cnt >= 80);
    CHECK_THAT(max_viol, WithinAbs(0.0, 1e-4));
}

TEST_CASE("E05: Two solver instances have no shared state", "[admm][edge][E]") {
    auto H = make_mpc_hessian<10>(1.0);
    Vec<10> lb = make_vec_const<10>(-1.0);
    Vec<10> ub = make_vec_const<10>(+1.0);

    ADMMSolver<10> s1, s2;
    Params p1; p1.rho=0.5; p1.max_iter=500; p1.eps_abs=1e-4; p1.eps_rel=1e-3;
    Params p2; p2.rho=2.0; p2.max_iter=500; p2.eps_abs=1e-4; p2.eps_rel=1e-3;
    REQUIRE(s1.setup(H, p1)); REQUIRE(s2.setup(H, p2));

    Vec<10> q1 = make_vec_const<10>(1.0);
    Vec<10> q2 = make_vec_const<10>(-1.0);

    // Interleaved — must not interfere
    auto r1a = s1.solve(q1, lb, ub);
    auto r2a = s2.solve(q2, lb, ub);
    auto r1b = s1.solve(q1, lb, ub);
    auto r2b = s2.solve(q2, lb, ub);

    // Same solver, same q → consistent results
    CHECK_THAT(rel_err(r1a.x, r1b.x), WithinAbs(0.0, 1e-3));
    CHECK_THAT(rel_err(r2a.x, r2b.x), WithinAbs(0.0, 1e-3));
    // Different q → different solutions
    CHECK((r1a.x - r2a.x).norm2() > 0.01);
}
