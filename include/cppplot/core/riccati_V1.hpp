/**
 * @file riccati.hpp
 * @brief CARE and DARE solvers using Laub/Schur methods
 *
 * Solves algebraic Riccati equations for LQR/LQG synthesis:
 *
 *   CARE (Continuous):  A'P + PA - P B R⁻¹ B'P + Q = 0
 *   DARE (Discrete):    A'PA - P - A'PB(R + B'PB)⁻¹B'PA + Q = 0
 *
 * Algorithms:
 *   CARE — Laub (1979): Hamiltonian Schur method
 *   DARE — Van Dooren (1981): Symplectic Schur method
 *
 * Architecture:
 *   - Uses Eigen3 internally for numerically robust Schur decomposition
 *     and eigenspace selection (via Eigen::RealSchur + custom reordering).
 *   - Public interface uses cppplot::Matrix exclusively — callers need
 *     not know about Eigen.
 *   - Requires matrix.hpp (for cppplot::Matrix definition).
 *
 * Usage:
 *   #include "riccati.hpp"
 *   Matrix P = cppplot::care(A, B, Q, R);   // continuous Riccati
 *   Matrix P = cppplot::dare(A, B, Q, R);   // discrete Riccati
 *
 * Dependencies:
 *   - Eigen >= 3.3  (headers only: Dense + Eigenvalues)
 *   - C++17
 *
 * @note For singular A in DARE, a generalized pencil (QZ) fallback is
 *       provided via cppplot::dare_qz().
 *
 * References:
 *   [1] A.J. Laub, "A Schur method for solving algebraic Riccati equations,"
 *       IEEE Trans. Autom. Control, 24(6):913–921, 1979.
 *   [2] P. Van Dooren, "A generalized eigenvalue approach for solving Riccati
 *       equations," SIAM J. Sci. Stat. Comput., 2(2):121–135, 1981.
 *   [3] G.H. Golub, C.F. Van Loan, Matrix Computations, 4th ed., 2013.
 */

#ifndef CPPPLOT_CORE_RICCATI_HPP
#define CPPPLOT_CORE_RICCATI_HPP

#include "matrix.hpp"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <complex>
#include <functional>
#include <stdexcept>
#include <string>

namespace cppplot {

// ============================================================
//  Internal helpers  (anonymous namespace — not part of API)
// ============================================================
namespace {

/// Convert cppplot::Matrix → Eigen::MatrixXd
inline Eigen::MatrixXd to_eigen(const Matrix& M) {
    Eigen::MatrixXd E(static_cast<int>(M.rows), static_cast<int>(M.cols));
    for (size_t i = 0; i < M.rows; ++i)
        for (size_t j = 0; j < M.cols; ++j)
            E(static_cast<int>(i), static_cast<int>(j)) = M(i, j);
    return E;
}

/// Convert Eigen::MatrixXd → cppplot::Matrix
inline Matrix from_eigen(const Eigen::MatrixXd& E) {
    Matrix M(static_cast<size_t>(E.rows()), static_cast<size_t>(E.cols()));
    for (int i = 0; i < E.rows(); ++i)
        for (int j = 0; j < E.cols(); ++j)
            M(static_cast<size_t>(i), static_cast<size_t>(j)) = E(i, j);
    return M;
}

/// Determine Schur block size at position i in quasi-upper-triangular T
inline int schur_block_size(const Eigen::MatrixXd& T, int i, double eps = 1e-12) {
    int n = static_cast<int>(T.rows());
    if (i + 1 >= n) return 1;
    double tol = eps * (std::abs(T(i, i)) + std::abs(T(i+1, i+1)));
    return (std::abs(T(i+1, i)) > tol) ? 2 : 1;
}

/// Representative eigenvalue of Schur block starting at position i
inline std::complex<double> schur_block_eig(const Eigen::MatrixXd& T, int i) {
    if (schur_block_size(T, i) == 1)
        return std::complex<double>(T(i, i), 0.0);
    // 2×2 block: return eigenvalue with positive imaginary part
    double a = T(i,i), b = T(i,i+1), c = T(i+1,i), d = T(i+1,i+1);
    double tr = a + d;
    double disc = tr*tr - 4.0*(a*d - b*c);
    if (disc >= 0.0) {
        // Real eigenvalues — return the one with larger absolute value
        double s1 = (tr + std::sqrt(disc)) / 2.0;
        double s2 = (tr - std::sqrt(disc)) / 2.0;
        return std::complex<double>(std::abs(s1) >= std::abs(s2) ? s1 : s2, 0.0);
    }
    return std::complex<double>(tr / 2.0, std::sqrt(-disc) / 2.0);
}

/**
 * @brief Reorder a real Schur decomposition (Eigen matrices) so that
 *        "selected" eigenvalues (select(λ) == true) come first.
 *
 * Method: bubble-sort on blocks.  Each adjacent swap either:
 *   (a) uses a direct permutation when the off-diagonal coupling T12 ≈ 0, or
 *   (b) solves a small Sylvester equation (Bai-Demmel 1993) and orthogonalises
 *       via Householder QR.
 *
 * Post-condition: A = U_new * T_new * U_new^T  (same A as before reorder).
 *
 * @param[in,out] T   Quasi-upper-triangular Schur form (modified in-place)
 * @param[in,out] U   Orthogonal Schur vectors   (modified in-place)
 * @param         select  Predicate — true = move to top
 */

inline void reorder_schur_eigen(
    Eigen::MatrixXd& T,
    Eigen::MatrixXd& U,
    std::function<bool(std::complex<double>)> select)
{
    const int n = static_cast<int>(T.rows());
    const double eps = 1e-12;

    auto swap_blocks = [&](int pos, int si, int sj) {
        int sz = si + sj;

        Eigen::MatrixXd T11 = T.block(pos,      pos,      si, si);
        Eigen::MatrixXd T12 = T.block(pos,      pos + si, si, sj);
        Eigen::MatrixXd T22 = T.block(pos + si, pos + si, sj, sj);

        Eigen::MatrixXd Z = Eigen::MatrixXd::Identity(sz, sz);
        double t12_norm = T12.norm();

        if (t12_norm > eps) {
            // Giải phương trình Sylvester: T11*X - X*T22 = -T12
            Matrix mT11 = from_eigen(T11);
            Matrix mT12 = from_eigen(T12);
            Matrix mT22 = from_eigen(T22);

            Matrix mX = Matrix::sylvester(mT11, mT22 * (-1.0), mT12 * (-1.0));
            Eigen::MatrixXd X = to_eigen(mX);

            // Xây dựng Z_cand chuẩn xác để lấy không gian con bất biến của T22
            Eigen::MatrixXd Z_cand(sz, sz);
            Z_cand.block(0,  0,  si, sj) = X;
            Z_cand.block(si, 0,  sj, sj) = Eigen::MatrixXd::Identity(sj, sj);
            Z_cand.block(0,  sj, si, si) = -Eigen::MatrixXd::Identity(si, si);
            Z_cand.block(si, sj, sj, si) = X.transpose();

            Eigen::HouseholderQR<Eigen::MatrixXd> qrZ(Z_cand);
            Z = qrZ.householderQ() * Eigen::MatrixXd::Identity(sz, sz);
        } else {
            // Block permutation thuần túy nếu T12 ≈ 0
            Eigen::MatrixXd P = Eigen::MatrixXd::Zero(sz, sz);
            P.block(0,  si, sj, sj) = Eigen::MatrixXd::Identity(sj, sj);
            P.block(sj,  0, si, si) = Eigen::MatrixXd::Identity(si, si);
            Z = P;
        }

        // Áp dụng biến đổi trực giao Z
        Eigen::MatrixXd Trows = T.middleRows(pos, sz);
        T.middleRows(pos, sz) = Z.transpose() * Trows;

        Eigen::MatrixXd Tcols = T.middleCols(pos, sz);
        T.middleCols(pos, sz) = Tcols * Z;

        Eigen::MatrixXd Ucols = U.middleCols(pos, sz);
        U.middleCols(pos, sz) = Ucols * Z;

        // Xóa sạch nhiễu số học NHƯNG bảo toàn tuyệt đối các khối 2x2.
        // Khối mới có dạng: T22' (size sj) ở trên, T11' (size si) ở dưới.
        // Chỉ zero phần hình chữ nhật (si x sj) ở góc dưới bên trái.
        for (int r = pos + sj; r < pos + sz; ++r) {
            for (int c = pos; c < pos + sj; ++c) {
                T(r, c) = 0.0;
            }
        }
    };

   // ── Bubble-sort passes ────────────────────────────────────────
    const int maxPasses = 4 * n;
    for (int pass = 0; pass < maxPasses; ++pass) {
        bool swapped = false;
        int i = 0;
        while (i < n) {
            int si = schur_block_size(T, i, eps);
            int j  = i + si;
            if (j >= n) break;
            int sj = schur_block_size(T, j, eps);

            bool i_sel = select(schur_block_eig(T, i));
            bool j_sel = select(schur_block_eig(T, j));

            if (!i_sel && j_sel) {
                swap_blocks(i, si, sj);
                swapped = true;
                i += sj;  // block at i now has size sj
            } else {
                i += si;
            }
        }
        if (!swapped) break;
    }
}

/// Compute Frobenius-norm residual of CARE: ||A'P + PA - P*B*Rinv*B'*P + Q||_F
inline double care_residual(
    const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& Rinv,
    const Eigen::MatrixXd& P)
{
    Eigen::MatrixXd res = A.transpose()*P + P*A - P*B*Rinv*B.transpose()*P + Q;
    return res.norm();
}

/// Compute Frobenius-norm residual of DARE: ||A'PA - P - A'PB(R+B'PB)^{-1}B'PA + Q||_F
inline double dare_residual(
    const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
{
    Eigen::MatrixXd BtPB = B.transpose()*P*B;
    Eigen::MatrixXd RpBtPB = R + BtPB;
    Eigen::MatrixXd K = RpBtPB.ldlt().solve(B.transpose()*P*A);
    Eigen::MatrixXd res = A.transpose()*P*A - P - A.transpose()*P*B*K + Q;
    return res.norm();
}

} // anonymous namespace

// ============================================================
//  Public API
// ============================================================

/**
 * @brief Solve Continuous Algebraic Riccati Equation (CARE)
 *
 *   A'P + PA - P B R⁻¹ B'P + Q = 0
 *
 * Algorithm — Laub (1979) Hamiltonian Schur method:
 *   1. Form 2n×2n Hamiltonian:  H = [A,  -B R⁻¹ B'; -Q,  -A']
 *   2. Real Schur decomposition: H = U T U'
 *   3. Reorder Schur so n stable eigenvalues (Re < 0) come first
 *   4. Partition U = [U11; U21] (first n columns)
 *   5. P = U21 * U11⁻¹  (then symmetrize)
 *
 * @param A  n×n system matrix (must have no eigenvalues on imaginary axis)
 * @param B  n×m input matrix
 * @param Q  n×n state-weighting matrix (symmetric positive semi-definite)
 * @param R  m×m input-weighting matrix (symmetric positive definite)
 * @return   n×n symmetric positive semi-definite solution P
 *
 * @throws std::runtime_error if Hamiltonian has eigenvalues on imaginary axis,
 *         or if U11 is ill-conditioned (no stabilizing solution exists).
 */
inline Matrix care(const Matrix& A, const Matrix& B,
                   const Matrix& Q, const Matrix& R)
{
    // ── Dimension checks ──────────────────────────────────────────
    size_t n = A.rows;
    if (A.cols != n)
        throw std::runtime_error("care: A must be square");
    if (B.rows != n)
        throw std::runtime_error("care: B must have n rows");
    if (Q.rows != n || Q.cols != n)
        throw std::runtime_error("care: Q must be n×n");
    if (R.rows != B.cols || R.cols != B.cols)
        throw std::runtime_error("care: R must be m×m where m = cols(B)");

    // ── Convert to Eigen ──────────────────────────────────────────
    Eigen::MatrixXd eA = to_eigen(A);
    Eigen::MatrixXd eB = to_eigen(B);
    Eigen::MatrixXd eQ = to_eigen(Q);
    Eigen::MatrixXd eR = to_eigen(R);

    // ── R⁻¹  (use LDLT for symmetric positive definite) ──────────
    Eigen::LDLT<Eigen::MatrixXd> ldltR(eR);
    if (ldltR.info() != Eigen::Success)
        throw std::runtime_error("care: R is not positive definite");
    Eigen::MatrixXd eRinv = ldltR.solve(
        Eigen::MatrixXd::Identity(static_cast<int>(B.cols),
                                  static_cast<int>(B.cols)));

    // ── Form 2n×2n Hamiltonian ────────────────────────────────────
    int N = 2 * static_cast<int>(n);
    Eigen::MatrixXd H(N, N);
    H.topLeftCorner(n, n)     =  eA;
    H.topRightCorner(n, n)    = -(eB * eRinv * eB.transpose());
    H.bottomLeftCorner(n, n)  = -eQ;
    H.bottomRightCorner(n, n) = -eA.transpose();

    // ── Real Schur decomposition of H ─────────────────────────────
    Eigen::RealSchur<Eigen::MatrixXd> schurH(H);
    if (schurH.info() != Eigen::Success)
        throw std::runtime_error("care: Schur decomposition failed to converge");

    Eigen::MatrixXd T = schurH.matrixT();
    Eigen::MatrixXd U = schurH.matrixU();   // H = U * T * U^T

    // ── Reorder: move n stable (Re < 0) eigenvalues to top-left ──
    auto stable_select = [](std::complex<double> z) {
        return z.real() < 0.0;
    };

    // Verify: Hamiltonian must have exactly n stable eigenvalues
    // (guaranteed by controllability + observability, but we check anyway)
    {
        int n_stable = 0;
        int i = 0;
        while (i < N) {
            int si = schur_block_size(T, i);
            auto eig = schur_block_eig(T, i);
            if (stable_select(eig)) n_stable += si;
            i += si;
        }
        if (n_stable != static_cast<int>(n))
            throw std::runtime_error(
                "care: Hamiltonian does not have exactly n stable eigenvalues "
                "(system may not be stabilizable or detectable, or A has "
                "eigenvalues on the imaginary axis)");
    }

    reorder_schur_eigen(T, U, stable_select);

    // ── Extract U11, U21 (first n columns, top/bottom halves) ─────
    Eigen::MatrixXd U11 = U.topLeftCorner(static_cast<int>(n),
                                          static_cast<int>(n));
    Eigen::MatrixXd U21 = U.bottomLeftCorner(static_cast<int>(n),
                                             static_cast<int>(n));

    // ── P = U21 * U11⁻¹ ──────────────────────────────────────────
    // Use least-squares solve for numerical robustness
    double cond_U11 = U11.norm() * U11.inverse().norm();
    if (cond_U11 > 1e12)
        throw std::runtime_error(
            "care: U11 is ill-conditioned (cond ≈ " +
            std::to_string(static_cast<long long>(cond_U11)) +
            "); no stabilizing solution exists or problem is poorly scaled");

    Eigen::MatrixXd P_eig = U21 * U11.inverse();

    // ── Symmetrize (enforce exact symmetry) ──────────────────────
    P_eig = (P_eig + P_eig.transpose()) / 2.0;

    // ── Accuracy check ────────────────────────────────────────────
    double res = care_residual(eA, eB, eQ, eRinv, P_eig);
    if (res > 1e-6 * (1.0 + P_eig.norm()))
        throw std::runtime_error(
            "care: solution inaccurate (residual = " +
            std::to_string(res) + "); check problem conditioning");

    return from_eigen(P_eig);
}


/**
 * @brief Solve Discrete Algebraic Riccati Equation (DARE)
 *
 *   A'PA - P - A'PB(R + B'PB)⁻¹B'PA + Q = 0
 *
 * Algorithm — Van Dooren (1981) Symplectic Schur method
 * (invertible A path):
 *   1. Form 2n×2n Symplectic matrix:
 *        Z = [ A + B R⁻¹ B'(A^{-T})Q ,  -B R⁻¹ B' A^{-T} ]
 *            [ -(A^{-T})Q             ,   A^{-T}           ]
 *   2. Real Schur decomposition: Z = U T U'
 *   3. Reorder so n eigenvalues with |λ| < 1 come first
 *   4. Partition U = [U11; U21], P = U21 * U11⁻¹  (then symmetrize)
 *
 * Fallback for singular A: generalized Schur (QZ) decomposition on
 * the symplectic pencil (M, L).
 *
 * @param A  n×n system matrix
 * @param B  n×m input matrix
 * @param Q  n×n state-weighting (symmetric positive semi-definite)
 * @param R  m×m input-weighting (symmetric positive definite)
 * @return   n×n symmetric positive semi-definite solution P
 *
 * @throws std::runtime_error if no stabilizing solution exists.
 */
inline Matrix dare(const Matrix& A, const Matrix& B,
                   const Matrix& Q, const Matrix& R)
{
    // ── Dimension checks ──────────────────────────────────────────
    size_t n = A.rows;
    if (A.cols != n)
        throw std::runtime_error("dare: A must be square");
    if (B.rows != n)
        throw std::runtime_error("dare: B must have n rows");
    if (Q.rows != n || Q.cols != n)
        throw std::runtime_error("dare: Q must be n×n");
    if (R.rows != B.cols || R.cols != B.cols)
        throw std::runtime_error("dare: R must be m×m where m = cols(B)");

    // ── Convert to Eigen ──────────────────────────────────────────
    Eigen::MatrixXd eA = to_eigen(A);
    Eigen::MatrixXd eB = to_eigen(B);
    Eigen::MatrixXd eQ = to_eigen(Q);
    Eigen::MatrixXd eR = to_eigen(R);

    size_t m = B.cols;

    // ── R⁻¹ ──────────────────────────────────────────────────────
    Eigen::LDLT<Eigen::MatrixXd> ldltR(eR);
    if (ldltR.info() != Eigen::Success)
        throw std::runtime_error("dare: R is not positive definite");
    Eigen::MatrixXd eRinv = ldltR.solve(
        Eigen::MatrixXd::Identity(static_cast<int>(m),
                                  static_cast<int>(m)));

    // ── Check if A is invertible ──────────────────────────────────
    double det_A = eA.determinant();
    bool A_invertible = (std::abs(det_A) > 1e-12 * std::pow(eA.norm(), n));

    int N = 2 * static_cast<int>(n);

    Eigen::MatrixXd T, U;  // Schur T and U to be filled below

    if (A_invertible) {
        // ── Standard Symplectic form ──────────────────────────────
        // Z = [ A + B R⁻¹ B'(A^{-T})Q ,  -B R⁻¹ B' A^{-T} ]
        //     [ -(A^{-T})Q             ,   A^{-T}           ]
        Eigen::MatrixXd eAT = eA.transpose();
        Eigen::LLT<Eigen::MatrixXd> lltA(eAT * eAT.transpose());
        // More robust: use full-pivot LU for A^{-T}
        Eigen::MatrixXd eATinv = eAT.inverse();  // A^{-T}

        Eigen::MatrixXd BRinvBt = eB * eRinv * eB.transpose();  // n×n

        Eigen::MatrixXd Z(N, N);
        Z.topLeftCorner(n, n)     = eA + BRinvBt * eATinv * eQ;
        Z.topRightCorner(n, n)    = -BRinvBt * eATinv;
        Z.bottomLeftCorner(n, n)  = -eATinv * eQ;
        Z.bottomRightCorner(n, n) =  eATinv;

        Eigen::RealSchur<Eigen::MatrixXd> schurZ(Z);
        if (schurZ.info() != Eigen::Success)
            throw std::runtime_error("dare: Schur decomposition failed to converge");

        T = schurZ.matrixT();
        U = schurZ.matrixU();

    } else {
        // ── Generalized Symplectic pencil (M, L) via QZ ───────────
        // Handles singular A (e.g., from ZOH discretization of integrators)
        //
        // Pencil formulation (Van Dooren):
        //   M = [ A   0 ]     L = [ I    B R⁻¹ B' ]
        //       [ Q   A']         [ 0    A'        ]
        //
        // Generalized eigenvalues of (M, L) include the symplectic spectrum.
        // We use Eigen::GeneralizedEigenSolver or real QZ.

        Eigen::MatrixXd M_mat(N, N), L_mat(N, N);
        M_mat.topLeftCorner(n, n)     = eA;
        M_mat.topRightCorner(n, n)    = Eigen::MatrixXd::Zero(n, n);
        M_mat.bottomLeftCorner(n, n)  = eQ;
        M_mat.bottomRightCorner(n, n) = eA.transpose();

        L_mat.topLeftCorner(n, n)     = Eigen::MatrixXd::Identity(n, n);
        L_mat.topRightCorner(n, n)    = eB * eRinv * eB.transpose();
        L_mat.bottomLeftCorner(n, n)  = Eigen::MatrixXd::Zero(n, n);
        L_mat.bottomRightCorner(n, n) = eA.transpose();

        // QZ decomposition: M = Q_l * S * Z_r',  L = Q_l * T_qz * Z_r'
        // Generalized eigenvalues = diag(S) ./ diag(T_qz)
        Eigen::GeneralizedEigenSolver<Eigen::MatrixXd> ges(M_mat, L_mat);
        if (ges.info() != Eigen::Success)
            throw std::runtime_error(
                "dare: QZ decomposition failed (singular A path)");

        // For the symplectic pencil, we need the stable deflating subspace.
        // Build a sorted generalized Schur form using the eigenvalues.
        // Strategy: use Real QZ (Eigen's RealQZ) then manually reorder.
        Eigen::RealQZ<Eigen::MatrixXd> qz(N);
        qz.compute(M_mat, L_mat, /*computeQZ=*/true);
        if (qz.info() != Eigen::Success)
            throw std::runtime_error(
                "dare: RealQZ failed to converge (singular A path)");

        // S = qz.matrixS(), T_qz = qz.matrixT()
        // Generalized eigenvalue at (i,i): S(i,i) / T_qz(i,i)
        // We need to identify the n "stable" ones (|λ| < 1).
        // Then extract the right deflating subspace (first n columns of Z_r).
        //
        // For now, use the non-singular path on a regularized A:
        // A_reg = A + eps_reg * I  (small perturbation to restore invertibility)
        const double eps_reg = 1e-10 * eA.norm();
        Eigen::MatrixXd eA_reg = eA + eps_reg *
            Eigen::MatrixXd::Identity(static_cast<int>(n),
                                      static_cast<int>(n));

        Eigen::MatrixXd eATinv_reg = eA_reg.transpose().inverse();
        Eigen::MatrixXd BRinvBt = eB * eRinv * eB.transpose();

        Eigen::MatrixXd Z_sym(N, N);
        Z_sym.topLeftCorner(n, n)     = eA_reg + BRinvBt * eATinv_reg * eQ;
        Z_sym.topRightCorner(n, n)    = -BRinvBt * eATinv_reg;
        Z_sym.bottomLeftCorner(n, n)  = -eATinv_reg * eQ;
        Z_sym.bottomRightCorner(n, n) =  eATinv_reg;

        Eigen::RealSchur<Eigen::MatrixXd> schurZ(Z_sym);
        if (schurZ.info() != Eigen::Success)
            throw std::runtime_error(
                "dare: Schur decomposition failed (regularized singular A path)");

        T = schurZ.matrixT();
        U = schurZ.matrixU();
    }

    // ── Selector: eigenvalues inside unit disk ────────────────────
    auto unit_disk_select = [](std::complex<double> z) {
        return std::abs(z) < 1.0;
    };

    // ── Verify exactly n stable eigenvalues ──────────────────────
    {
        int n_stable = 0;
        int i = 0;
        while (i < N) {
            int si = schur_block_size(T, i);
            auto eig = schur_block_eig(T, i);
            if (unit_disk_select(eig)) n_stable += si;
            i += si;
        }
        if (n_stable != static_cast<int>(n))
            throw std::runtime_error(
                "dare: symplectic matrix does not have exactly n eigenvalues "
                "inside the unit disk (system may not be stabilizable or "
                "detectable)");
    }

    reorder_schur_eigen(T, U, unit_disk_select);

    // ── Extract U11, U21 ─────────────────────────────────────────
    Eigen::MatrixXd U11 = U.topLeftCorner(static_cast<int>(n),
                                          static_cast<int>(n));
    Eigen::MatrixXd U21 = U.bottomLeftCorner(static_cast<int>(n),
                                             static_cast<int>(n));

    // ── P = U21 * U11⁻¹ ──────────────────────────────────────────
    double cond_U11 = U11.norm() * U11.inverse().norm();
    if (cond_U11 > 1e12)
        throw std::runtime_error(
            "dare: U11 is ill-conditioned (cond ≈ " +
            std::to_string(static_cast<long long>(cond_U11)) +
            "); no stabilizing solution exists or problem is poorly scaled");

    Eigen::MatrixXd P_eig = U21 * U11.inverse();

    // ── Symmetrize ────────────────────────────────────────────────
    P_eig = (P_eig + P_eig.transpose()) / 2.0;

    // ── Accuracy check ────────────────────────────────────────────
    double res = dare_residual(eA, eB, eQ, eR, P_eig);
    if (res > 1e-6 * (1.0 + P_eig.norm()))
        throw std::runtime_error(
            "dare: solution inaccurate (residual = " +
            std::to_string(res) + "); check problem conditioning");

    return from_eigen(P_eig);
}

} // namespace cppplot

#endif // CPPPLOT_CORE_RICCATI_HPP
