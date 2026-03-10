/**
 * @file mpc_embedded.hpp
 * @brief Embedded Real-Time MPC Controller — Zero Heap, Fixed-Size Template
 *
 * DCAS Lab — L0 Core Control Stack
 * Target: ARM Cortex-A7 @ 100MHz (Allwinner T113-S3)
 * WCET requirement: <= 10ms / solve @ N=10 horizon
 *
 * Design goals:
 *   - Zero dynamic memory allocation (stack-only via templates)
 *   - All matrices pre-allocated at compile time: Mat<MN>, Vec<MN>
 *   - C++14 header-only, no STL containers in hot path
 *   - Warm-start across solve() calls (closed-loop efficiency)
 *   - Drop-in for cppplot::control::MPCController (ADMM path only)
 *
 * Template parameters:
 *   NX   : state dimension  (e.g. 2 for AGV double integrator)
 *   NU   : input dimension  (e.g. 1 for AGV single-axis)
 *   NHOR : prediction horizon N (e.g. 10)
 *   MN   : NU * NHOR — size of stacked control vector U (auto-deduced)
 *
 * Problem solved each control step:
 *   min   0.5 * U' * H * U + g(x0)' * U
 *   s.t.  u_lb <= U_i <= u_ub   (box constraint, per input dim)
 *
 * where:
 *   H  = Gamma' * Q_bar * Gamma + R_bar   (built once at setup)
 *   g  = F * x0                            (updated every step)
 *   F  = Gamma' * Q_bar * Phi              (built once at setup)
 *
 * Usage:
 * @code
 *   // AGV double integrator: x=[pos,vel], u=accel, N=10
 *   using EMPC = EmbeddedMPC<2, 1, 10>;
 *
 *   EMPC::SystemMatrix A, B;
 *   // ... fill A, B ...
 *
 *   EMPC::WeightMatrix Q, R, Pf;
 *   // ... fill Q, R, Pf (terminal cost) ...
 *
 *   EMPC::BoundVec u_lb, u_ub;
 *   // ... fill bounds ...
 *
 *   EMPC::Params params;
 *   params.rho       = 5.0;   // spectral rho for your H
 *   params.max_iter  = 200;
 *   params.warm_start = true;
 *
 *   EMPC ctrl;
 *   ctrl.setup(A, B, Q, R, Pf, u_lb, u_ub, params);
 *
 *   // Hot path — every 10ms:
 *   EMPC::StateVec x0;  // fill current state
 *   auto sol = ctrl.solve(x0);
 *   double u_apply = sol.u[0];  // first control action
 * @endcode
 *
 * References:
 *   [1] Stellato et al. "OSQP: An Operator Splitting Solver for QPs."
 *       IEEE TAC, 2020.
 *   [2] Maciejowski — "Predictive Control with Constraints"
 *   [3] Rawlings & Mayne — "Model Predictive Control: Theory and Design"
 *
 * @author DCAS Lab, TDTU — TS. Trí Viễn Vũ
 * @date March 2026
 */

#ifndef DCAS_MPC_EMBEDDED_HPP
#define DCAS_MPC_EMBEDDED_HPP

#include "cppplot/control/qp_solver.hpp"   // qp::Vec, Mat, ADMMSolver, Params
#include <cmath>
#include <cstring>

namespace dcas {
namespace control {

// ============================================================
//  Convenience aliases (internal)
// ============================================================
template <int N> using Vec = cppplot::control::qp::Vec<N>;
template <int N> using Mat = cppplot::control::qp::Mat<N>;
using QPParams  = cppplot::control::qp::Params;
template <int N> using QPSolution = cppplot::control::qp::Solution<N>;
template <int N> using ADMMSolver = cppplot::control::qp::ADMMSolver<N>;

// ============================================================
//  EmbeddedMPC<NX, NU, NHOR>
// ============================================================

/**
 * @brief Fixed-size MPC for embedded real-time control.
 *
 * @tparam NX   State dimension
 * @tparam NU   Input dimension
 * @tparam NHOR Prediction horizon
 */
template <int NX, int NU, int NHOR>
class EmbeddedMPC {
public:
    // ── Compile-time sizes ────────────────────────────────
    static constexpr int MN  = NU * NHOR;   ///< stacked U size
    static constexpr int XN  = NX * NHOR;   ///< stacked X size

    // ── Public type aliases ───────────────────────────────
    using StateVec    = Vec<NX>;             ///< x0 current state
    using ControlVec  = Vec<NU>;             ///< single control u_k
    using StackedU    = Vec<MN>;             ///< full horizon U
    using StackedX    = Vec<XN>;             ///< full horizon X (no x0)

    using SystemMatrix = Mat<NX>;            ///< A, Q, R, Pf (NX x NX)
    using InputMatrix  = Mat<NU>;            ///< B columns: NX rows — see note
    // Note: B is NX x NU → stored as Mat<NX> with only NU cols used.
    // For simplicity we store B in a MN x NX matrix (Gamma rows).
    // See setup() for how A, B are used to build Phi, Gamma, H, F.

    using WeightMatrix = Mat<NX>;            ///< Q, Pf (state weight)
    using InputWeight  = Mat<NU>;            ///< R (input weight)
    using BoundVec     = Vec<NU>;            ///< u_lb, u_ub per input dim
    using Params       = QPParams;           ///< ADMM solver params

    // ── Solution returned by solve() ─────────────────────
    struct Solution {
        ControlVec  u;            ///< First control action u_0 (apply to plant)
        StackedU    U;            ///< Full horizon U* (for warm-start inspection)
        double      cost;         ///< QP objective value
        int         iterations;   ///< ADMM iterations used
        bool        converged;    ///< True if tolerance met
        double      primal_res;   ///< Primal residual (feasibility indicator)
        double      dual_res;     ///< Dual residual (optimality indicator)
    };

    // ────────────────────────────────────────────────────
    //  setup()
    // ────────────────────────────────────────────────────

    /**
     * @brief One-time setup: build prediction matrices, factorize H.
     *
     * Call once (in constructor or init phase).
     * Internally builds:
     *   Phi   (XN x NX)  — state prediction from x0
     *   Gamma (XN x MN)  — state prediction from U
     *   H     (MN x MN)  — QP Hessian (constant)
     *   F     (MN x NX)  — linear cost term F*x0 = g
     *
     * Then calls ADMMSolver<MN>::setup(H, params).
     *
     * @param A      Discrete system matrix  (NX x NX)
     * @param B      Input matrix            (NX x NU) — stored column-major
     * @param Q      State weight            (NX x NX)
     * @param R      Input weight            (NU x NU)
     * @param Pf     Terminal weight         (NX x NX) — use dare() result
     * @param u_lb   Input lower bounds      (NU x 1)
     * @param u_ub   Input upper bounds      (NU x 1)
     * @param params ADMM solver parameters
     * @return true if Cholesky factorization succeeded
     */
    bool setup(const Mat<NX>& A,
               const double   B[NX][NU],   // raw NX x NU matrix
               const Mat<NX>& Q,
               const Mat<NU>& R,
               const Mat<NX>& Pf,
               const BoundVec& u_lb,
               const BoundVec& u_ub,
               const Params&   params = Params())
    {
        A_  = A;
        std::memcpy(B_, B, sizeof(B_));
        Q_  = Q;
        R_  = R;
        Pf_ = Pf;

        // Expand u_lb / u_ub over full horizon
        for (int k = 0; k < NHOR; ++k)
            for (int j = 0; j < NU; ++j) {
                lb_[k * NU + j] = u_lb[j];
                ub_[k * NU + j] = u_ub[j];
            }

        buildPredictionMatrices_();

        Mat<MN> H = buildHessian_();
        Mat<MN> F_unused;   // F_ stored as member
        buildLinearTerm_();  // builds F_ (MN x NX)

        params_ = params;
        return solver_.setup(H, params_);
    }

    // ────────────────────────────────────────────────────
    //  solve()  ← HOT PATH  (called every control step)
    // ────────────────────────────────────────────────────

    /**
     * @brief Solve MPC QP for current state x0.
     *
     * Hot path: only computes g = F*x0, then calls ADMM solver.
     * No heap allocation. O(MN²) per call.
     *
     * @param x0  Current state (NX x 1)
     * @return    Solution with first control action u[0..NU-1]
     */
    Solution solve(const StateVec& x0)
    {
        // g = F * x0  (MN x 1)
        Vec<MN> g = matvec_FNX_(F_, x0);

        // ADMM solve
        QPSolution<MN> qp = solver_.solve(g, lb_, ub_);

        // Pack result
        Solution sol;
        sol.U           = qp.x;
        sol.cost        = qp.cost;
        sol.iterations  = qp.iterations;
        sol.converged   = qp.converged;
        sol.primal_res  = qp.primal_res;
        sol.dual_res    = qp.dual_res;

        // Extract first control action u_0
        for (int j = 0; j < NU; ++j)
            sol.u[j] = qp.x[j];

        return sol;
    }

    /**
     * @brief Reset ADMM warm-start state (call on large reference changes).
     */
    void reset() { solver_.reset(); }

    /**
     * @brief Update rho and re-factorize (adaptive rho).
     *
     * Useful if online spectral_rho estimation gives a better value.
     */
    bool updateRho(double new_rho) {
        params_.rho = new_rho;
        Mat<MN> H = buildHessian_();
        return solver_.setup(H, params_);
    }

    /**
     * @brief Get current ADMM parameters.
     */
    const Params& getParams() const { return solver_.get_params(); }

    // ────────────────────────────────────────────────────
    //  Accessors for inspection / logging
    // ────────────────────────────────────────────────────

    /** @brief Hessian H (MN x MN) — built once at setup */
    const Mat<MN>& H() const { return H_; }

    /** @brief Linear cost matrix F (MN x NX) — g = F*x0 */
    const double (&F() const)[MN][NX] { return F_; }

    /** @brief Prediction matrix Phi (XN x NX) */
    const double (&Phi() const)[XN][NX] { return Phi_; }

    // ────────────────────────────────────────────────────
    //  Predict full state trajectory (optional, off hot path)
    // ────────────────────────────────────────────────────

    /**
     * @brief Compute predicted state trajectory X = Phi*x0 + Gamma*U.
     *
     * Not on the hot path — use for logging/visualization only.
     *
     * @param x0  Current state
     * @param U   Stacked control sequence (from solve().U)
     * @param X   Output: predicted states X[k] for k=1..NHOR
     */
    void predictStates(const StateVec& x0,
                       const StackedU& U,
                       StateVec        X[NHOR]) const
    {
        // X[k] = A^(k+1)*x0 + sum_{j=0}^{k} A^(k-j)*B*U[j]
        // Computed via Phi and Gamma stored internally.
        for (int k = 0; k < NHOR; ++k) {
            StateVec xk;
            // Phi contribution: Phi[k*NX .. (k+1)*NX-1, :] * x0
            for (int i = 0; i < NX; ++i) {
                double s = 0.0;
                for (int c = 0; c < NX; ++c)
                    s += Phi_[k * NX + i][c] * x0[c];
                xk[i] = s;
            }
            // Gamma contribution: Gamma[k*NX.., :] * U
            for (int i = 0; i < NX; ++i) {
                double s = 0.0;
                for (int c = 0; c < MN; ++c)
                    s += Gamma_[k * NX + i][c] * U[c];
                xk[i] += s;
            }
            X[k] = xk;
        }
    }

private:
    // ── Stored system data ────────────────────────────────
    Mat<NX> A_;
    double  B_[NX][NU];
    Mat<NX> Q_;
    Mat<NU> R_;
    Mat<NX> Pf_;

    // ── Prediction matrices (stack allocated) ────────────
    double Phi_[XN][NX];    ///< (NHOR*NX) x NX
    double Gamma_[XN][MN];  ///< (NHOR*NX) x (NHOR*NU)
    Mat<MN> H_;              ///< QP Hessian (MN x MN)
    double  F_[MN][NX];     ///< Linear cost matrix (MN x NX)

    // ── Bounds ───────────────────────────────────────────
    Vec<MN> lb_, ub_;

    // ── ADMM solver ───────────────────────────────────────
    ADMMSolver<MN> solver_;
    Params         params_;

    // ────────────────────────────────────────────────────
    //  Internal: build Phi, Gamma
    // ────────────────────────────────────────────────────

    /**
     * @brief Phi[k] = A^(k+1), Gamma[k,j] = A^(k-j)*B  for j<=k
     */
    void buildPredictionMatrices_()
    {
        // Compute A^1, A^2, ..., A^NHOR and store rows in Phi_
        // A_pow[k] = A^(k+1)
        double A_pow[NHOR][NX][NX];
        // A^1 = A
        for (int i = 0; i < NX; ++i)
            for (int j = 0; j < NX; ++j)
                A_pow[0][i][j] = A_(i, j);
        // A^k = A^(k-1) * A
        for (int k = 1; k < NHOR; ++k) {
            for (int i = 0; i < NX; ++i)
                for (int j = 0; j < NX; ++j) {
                    double s = 0.0;
                    for (int p = 0; p < NX; ++p)
                        s += A_pow[k-1][i][p] * A_(p, j);
                    A_pow[k][i][j] = s;
                }
        }

        // Fill Phi_: row block k = A^(k+1)
        for (int k = 0; k < NHOR; ++k)
            for (int i = 0; i < NX; ++i)
                for (int j = 0; j < NX; ++j)
                    Phi_[k * NX + i][j] = A_pow[k][i][j];

        // A_pow_B[k][i][j] = A^k * B  (NX x NU), k=0..NHOR-1
        // A^0 * B = B
        double A_pow_B[NHOR][NX][NU];
        for (int i = 0; i < NX; ++i)
            for (int j = 0; j < NU; ++j)
                A_pow_B[0][i][j] = B_[i][j];
        for (int k = 1; k < NHOR; ++k) {
            for (int i = 0; i < NX; ++i)
                for (int j = 0; j < NU; ++j) {
                    double s = 0.0;
                    for (int p = 0; p < NX; ++p)
                        s += A_(i, p) * A_pow_B[k-1][p][j];
                    A_pow_B[k][i][j] = s;
                }
        }

        // Fill Gamma_: Gamma[row_k, col_j] = A^(k-j)*B  for j<=k, else 0
        std::memset(Gamma_, 0, sizeof(Gamma_));
        for (int k = 0; k < NHOR; ++k)       // row block k
            for (int j = 0; j <= k; ++j)     // col block j (causal)
                for (int i = 0; i < NX; ++i)
                    for (int c = 0; c < NU; ++c)
                        Gamma_[k * NX + i][j * NU + c] = A_pow_B[k - j][i][c];
    }

    // ────────────────────────────────────────────────────
    //  Internal: build H = Gamma'*Q_bar*Gamma + R_bar
    // ────────────────────────────────────────────────────

    Mat<MN> buildHessian_()
    {
        // Q_bar: block-diagonal (NHOR*NX x NHOR*NX)
        // Q_bar[k] = Pf for k=NHOR-1, else Q
        // We avoid storing Q_bar explicitly.
        // H = Gamma' * Q_bar * Gamma + R_bar
        // H(i,j) = sum_k Gamma[k*NX..(k+1)*NX-1, i]' * Q_k
        //                               * Gamma[k*NX.., j]
        //        + R_bar(i,j)

        Mat<MN> H;

        // Gamma' * Q_bar * Gamma
        for (int ci = 0; ci < MN; ++ci) {
            for (int cj = 0; cj < MN; ++cj) {
                double s = 0.0;
                for (int k = 0; k < NHOR; ++k) {
                    // Q_k
                    const Mat<NX>& Qk = (k == NHOR - 1) ? Pf_ : Q_;
                    for (int r = 0; r < NX; ++r)
                        for (int q = 0; q < NX; ++q)
                            s += Gamma_[k * NX + r][ci]
                               * Qk(r, q)
                               * Gamma_[k * NX + q][cj];
                }
                H(ci, cj) = s;
            }
        }

        // Add R_bar: block-diagonal R repeated NHOR times
        for (int k = 0; k < NHOR; ++k)
            for (int i = 0; i < NU; ++i)
                for (int j = 0; j < NU; ++j)
                    H(k * NU + i, k * NU + j) += R_(i, j);

        H_ = H;
        return H;
    }

    // ────────────────────────────────────────────────────
    //  Internal: build F = Gamma' * Q_bar * Phi  (MN x NX)
    // ────────────────────────────────────────────────────

    void buildLinearTerm_()
    {
        std::memset(F_, 0, sizeof(F_));
        for (int ci = 0; ci < MN; ++ci) {
            for (int cj = 0; cj < NX; ++cj) {
                double s = 0.0;
                for (int k = 0; k < NHOR; ++k) {
                    const Mat<NX>& Qk = (k == NHOR - 1) ? Pf_ : Q_;
                    for (int r = 0; r < NX; ++r)
                        for (int q = 0; q < NX; ++q)
                            s += Gamma_[k * NX + r][ci]
                               * Qk(r, q)
                               * Phi_[k * NX + q][cj];
                }
                F_[ci][cj] = s;
            }
        }
    }

    // ────────────────────────────────────────────────────
    //  Internal: matvec for F_ (MN x NX) * Vec<NX>
    // ────────────────────────────────────────────────────

    Vec<MN> matvec_FNX_(const double F[MN][NX], const StateVec& x) const
    {
        Vec<MN> out;
        for (int i = 0; i < MN; ++i) {
            double s = 0.0;
            for (int j = 0; j < NX; ++j)
                s += F[i][j] * x[j];
            out[i] = s;
        }
        return out;
    }
};

// ============================================================
//  Spectral rho estimator (power iteration)
//  Use this in setup to auto-tune params.rho.
// ============================================================

/**
 * @brief Estimate spectral rho for ADMMSolver from Hessian H.
 *
 * Uses power iteration (40 steps) to estimate lambda_max(H),
 * then applies: rho_opt = max(10 * lambda_max, sqrt(lmin*lmax))
 * Clamp low: rho >= 0.5.
 *
 * @tparam N  Size of H (= MN = NU*NHOR)
 * @param  H  QP Hessian matrix
 * @return    Recommended rho value
 */
template <int N>
double spectral_rho(const Mat<N>& H)
{
    // Power iteration for lambda_max
    Vec<N> v;
    for (int i = 0; i < N; ++i) v[i] = 1.0;
    for (int it = 0; it < 40; ++it) {
        v = H.matvec(v);
        double n = v.norm2();
        if (n > 1e-14)
            for (int i = 0; i < N; ++i) v[i] /= n;
    }
    double lmax = H.matvec(v).dot(v);

    // Estimate lambda_min via inverse iteration on (H + eps*I)
    // Approximation: lmin ≈ lmax / cond (use diagonal as proxy)
    double diag_min = H(0, 0);
    for (int i = 1; i < N; ++i)
        if (H(i, i) < diag_min) diag_min = H(i, i);
    double lmin = diag_min > 0.0 ? diag_min : lmax * 1e-3;

    double rho = 10.0 * lmax;
    double rho_geom = std::sqrt(lmin * lmax);
    if (rho_geom > rho) rho = rho_geom;
    if (rho < 0.5) rho = 0.5;

    return rho;
}

// ============================================================
//  Convenience factory: make_embedded_mpc<NX, NU, NHOR>()
// ============================================================

/**
 * @brief Factory helper — build params with auto-tuned rho.
 *
 * Constructs EmbeddedMPC, runs spectral_rho on the built H,
 * then re-setups the solver with optimal rho.
 *
 * @code
 *   auto ctrl = make_embedded_mpc<2, 1, 10>(A, B, Q, R, Pf, lb, ub);
 * @endcode
 */
template <int NX, int NU, int NHOR>
EmbeddedMPC<NX, NU, NHOR> make_embedded_mpc(
    const Mat<NX>& A,
    const double   B[NX][NU],
    const Mat<NX>& Q,
    const Mat<NU>& R,
    const Mat<NX>& Pf,
    const cppplot::control::qp::Vec<NU>& u_lb,
    const cppplot::control::qp::Vec<NU>& u_ub,
    QPParams params = QPParams())
{
    EmbeddedMPC<NX, NU, NHOR> ctrl;
    ctrl.setup(A, B, Q, R, Pf, u_lb, u_ub, params);

    // Auto-tune rho from built H
    double rho_opt = spectral_rho<NU * NHOR>(ctrl.H());
    ctrl.updateRho(rho_opt);

    return ctrl;
}

} // namespace control
} // namespace dcas

#endif // DCAS_MPC_EMBEDDED_HPP
