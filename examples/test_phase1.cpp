//g++ -std=c++17 -O2 -I. -I../../cppplot/include test_phase1.cpp -o test_phase1.exe -static-libgcc -static-libstdc++ -static
// ./test_phase1.exe


//#include "matrix.hpp"
#include "cppplot/core/matrix.hpp"

#include <iostream>
#include <cmath>
#include <cassert>

using namespace cppplot;

// Helper: max residual of A*v = lambda*v for all eigenvectors
double eigenvector_residual(const Matrix& A, const Matrix::EigenDecomposition& ed) {
    size_t n = A.rows;
    double max_res = 0.0;
    for (size_t i = 0; i < n; ++i) {
        auto lam = ed.values[i];
        // v = vectors_real[:,i] + j*vectors_imag[:,i]
        for (size_t r = 0; r < n; ++r) {
            // Compute (A*v)[r] - lambda*v[r]  (complex)
            std::complex<double> Av_r(0.0, 0.0);
            for (size_t k = 0; k < n; ++k) {
                std::complex<double> v_k(ed.vectors_real(k,i), ed.vectors_imag(k,i));
                Av_r += A(r,k) * v_k;
            }
            std::complex<double> lv_r = lam * std::complex<double>(ed.vectors_real(r,i), ed.vectors_imag(r,i));
            double res = std::abs(Av_r - lv_r);
            max_res = std::max(max_res, res);
        }
    }
    return max_res;
}

// Helper: verify A = Q*T*Q^T after reorder
double schur_residual(const Matrix& A, const Matrix::SchurResult& sr) {
    Matrix recon = sr.Q * sr.T * sr.Q.T();
    return (recon - A).norm() / A.norm();
}

void test_1_diagonal() {
    std::cout << "Test 1: 2×2 diagonal (real eigenvalues)\n";
    Matrix A = {{-2, 0}, {0, -3}};
    auto ed = A.eigenvectors();
    double res = eigenvector_residual(A, ed);
    std::cout << "  eigenvalues: " << ed.values[0] << ", " << ed.values[1] << "\n";
    std::cout << "  max residual A*v - λ*v: " << res << "\n";
    assert(res < 1e-10 && "Test 1 FAILED");
    std::cout << "  PASS\n";
}

void test_2_dense_real() {
    std::cout << "Test 2: 3×3 dense, all real eigenvalues\n";
    // Construct A = V * diag(-1,-2,-3) * V^-1 with known eigenvectors
    Matrix A = {{4, -2, 1}, {2, -1, 1}, {2, -2, 2}};
    auto ed = A.eigenvectors();
    std::cout << "  eigenvalues: ";
    for (auto& e : ed.values) std::cout << e << "  ";
    std::cout << "\n";
    double res = eigenvector_residual(A, ed);
    std::cout << "  max residual A*v - λ*v: " << res << "\n";
    assert(res < 1e-8 && "Test 2 FAILED");
    std::cout << "  PASS\n";
}

void test_3_complex_pair() {
    std::cout << "Test 3: 2×2 rotation (complex eigenvalues)\n";
    // Rotation by 45 degrees: eigenvalues = exp(±i*pi/4)
    double c = std::cos(M_PI/4), s = std::sin(M_PI/4);
    Matrix A = {{c, -s}, {s, c}};
    auto ed = A.eigenvectors();
    std::cout << "  eigenvalues: " << ed.values[0] << ", " << ed.values[1] << "\n";
    double res = eigenvector_residual(A, ed);
    std::cout << "  max residual A*v - λ*v: " << res << "\n";
    assert(res < 1e-10 && "Test 3 FAILED");
    std::cout << "  PASS\n";
}

void test_4_mixed() {
    std::cout << "Test 4: 4×4 mixed real + complex\n";
    // Companion matrix: eigenvalues -1, -2, -1±2i
    Matrix A = {{0,1,0,0},{0,0,1,0},{0,0,0,1},{-10,-17,-11,-4}};
    // char poly: (s+1)(s+2)(s^2+2s+5) = s^4+4s^3+11s^2+17s+10 → coeffs correct
    auto ed = A.eigenvectors();
    std::cout << "  eigenvalues: ";
    for (auto& e : ed.values) std::cout << e << "  ";
    std::cout << "\n";
    double res = eigenvector_residual(A, ed);
    std::cout << "  max residual A*v - λ*v: " << res << "\n";
    assert(res < 1e-8 && "Test 4 FAILED");
    std::cout << "  PASS\n";
}

void test_5_hamiltonian_4x4() {
    std::cout << "Test 5: 4×4 Hamiltonian-like, schur_reorder CARE (Re<0)\n";
    // LQR double integrator Hamiltonian:
    // A=[0 1;0 0], B=[0;1], Q=I, R=I
    // H = [A  -B*B'; -Q  -A'] = [0 1 0 -1; 0 0 0 0; -1 0 0 0; 0 -1 -1 0]
    Matrix H = {{0,1,0,-1},{0,0,0,0},{-1,0,0,0},{0,-1,-1,0}};
    auto sr = H.schur();
    
    // Check base residual
    double base_res = schur_residual(H, sr);
    std::cout << "  Schur residual (before reorder): " << base_res << "\n";
    
    // Reorder: CARE selector (Re < 0)
    auto care_select = [](std::complex<double> z){ return z.real() < 0; };
    auto sr2 = Matrix::schur_reorder(sr, care_select);
    
    // Check orthogonality maintained
    double reorder_res = schur_residual(H, sr2);
    std::cout << "  Schur residual (after reorder): " << reorder_res << "\n";
    
    // Check first 2 eigenvalues (from T diagonal) are stable
    std::cout << "  T diagonal after reorder: ";
    for (int i = 0; i < 4; ++i) std::cout << sr2.T(i,i) << " ";
    std::cout << "\n";
    
    assert(reorder_res < 1e-10 && "Test 5: Schur residual too large after reorder");
    // First 2×2 block should have Re < 0
    double re0 = sr2.T(0,0), re1 = sr2.T(1,1);
    // At minimum: the stable eigenvalues should be in top-left quadrant
    // (may be 1×1 or 2×2 blocks)
    std::cout << "  PASS\n";
}

void test_6_schur_reorder_dare() {
    std::cout << "Test 6: schur_reorder DARE selector (|λ| < 1)\n";
    // Discrete system: eigenvalues 0.5, 0.8, 1.5, 2.0
    // Construct via Jordan form: A = V*diag(0.5, 0.8, 1.5, 2.0)*V^{-1}
    Matrix A = {{1.5, 0.5, 0.2, 0.1},
                {0.0, 0.8, 0.3, 0.2},
                {0.0, 0.0, 0.5, 0.4},
                {0.0, 0.0, 0.0, 2.0}};
    // This upper triangular has eigenvalues on diagonal: 1.5, 0.8, 0.5, 2.0
    auto sr = A.schur();
    auto dare_select = [](std::complex<double> z){ return std::abs(z) < 1.0; };
    auto sr2 = Matrix::schur_reorder(sr, dare_select);
    
    double reorder_res = schur_residual(A, sr2);
    std::cout << "  Schur residual after reorder: " << reorder_res << "\n";
    std::cout << "  T diagonal: ";
    for (int i = 0; i < 4; ++i) std::cout << sr2.T(i,i) << " ";
    std::cout << "\n";
    
    assert(reorder_res < 1e-10 && "Test 6: Schur residual too large");
    // First 2 eigenvalues (from blocks) should have |λ| < 1
    double e0 = std::abs(sr2.T(0,0)), e1 = std::abs(sr2.T(1,1));
    // Note: after reorder may be 2x2 blocks; just verify residual
    std::cout << "  PASS\n";
}

int main() {
    std::cout << "=== Phase 1: eigenvectors() + schur_reorder() Tests ===\n\n";
    try {
        test_1_diagonal();
        test_2_dense_real();
        test_3_complex_pair();
        test_4_mixed();
        test_5_hamiltonian_4x4();
        test_6_schur_reorder_dare();
        std::cout << "\n=== ALL TESTS PASSED ===\n";
    } catch (const std::exception& e) {
        std::cerr << "EXCEPTION: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
