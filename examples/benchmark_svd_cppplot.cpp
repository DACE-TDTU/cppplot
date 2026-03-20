/**
 * benchmark_svd_cppplot.cpp
 * DCAS Lab cppplot benchmark — SVD & Pseudo-inverse
 *
 * Build:
 * g++ -std=c++17 -O2 -I../../cppplot/include benchmark_svd_cppplot.cpp -o bench_svd
 * ./bench_svd
 */

#include "cppplot/core/matrix.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace cppplot;

void check_svd(const std::string& name, const Matrix& A) {
    std::cout << "--- " << name << " (" << A.rows << "x" << A.cols << ") ---\n";
    auto svd = A.svd();
    
    // 1. Reconstruction: A_recon = U * S * V^T
    Matrix A_recon = svd.U * svd.S * svd.V.T();
    double recon_err = (A - A_recon).norm();
    std::cout << "Reconstruction err ||A - U*S*V^T|| : " << std::scientific << recon_err << "\n";
    
    // 2. Orthogonality: U^T * U = I, V^T * V = I
    Matrix I_U = Matrix::eye(svd.U.cols);
    double u_orth_err = (svd.U.T() * svd.U - I_U).norm();
    
    Matrix I_V = Matrix::eye(svd.V.cols);
    double v_orth_err = (svd.V.T() * svd.V - I_V).norm();
    std::cout << "Orthogonality err ||U^T*U - I||    : " << u_orth_err << "\n";
    std::cout << "Orthogonality err ||V^T*V - I||    : " << v_orth_err << "\n";
    
    // 3. Singular values
    std::cout << "Singular values: ";
    for (size_t i = 0; i < svd.S.rows; ++i) {
        std::cout << std::fixed << std::setprecision(4) << svd.S(i, i) << " ";
    }
    std::cout << "\n\n";
}

void check_pinv(const std::string& name, const Matrix& A) {
    std::cout << "--- Pseudo-inverse: " << name << " ---\n";
    Matrix A_pinv = A.pinv();
    
    // Moore-Penrose conditions:
    // 1. A * A^+ * A = A
    double err1 = (A * A_pinv * A - A).norm();
    // 2. A^+ * A * A^+ = A^+
    double err2 = (A_pinv * A * A_pinv - A_pinv).norm();
    
    std::cout << "Moore-Penrose cond 1 ||A*A^+*A - A||       : " << std::scientific << err1 << "\n";
    std::cout << "Moore-Penrose cond 2 ||A^+*A*A^+ - A^+||   : " << err2 << "\n\n";
}

int main() {
    std::cout << "=== SVD & Pseudo-inverse Benchmark ===\n\n";

    // Case 1: Square matrix (Rank Deficient)
    Matrix A_sq = {
        {1.0, 2.0, 3.0},
        {4.0, 5.0, 6.0},
        {7.0, 8.0, 9.0} // Hàng 3 = 2*Hàng 2 - Hàng 1 -> Suy biến, Rank 2
    };
    check_svd("Square (Rank Deficient)", A_sq);
    check_pinv("Square (Rank Deficient)", A_sq);

    // Case 2: Tall matrix (m > n)
    Matrix A_tall = {
        {1.0, 2.0},
        {3.0, 4.0},
        {5.0, 6.0},
        {7.0, 8.0}
    };
    check_svd("Tall matrix", A_tall);
    check_pinv("Tall matrix", A_tall);

    // Case 3: Wide matrix (m < n)
    Matrix A_wide = {
        {1.0, 3.0, 5.0, 7.0},
        {2.0, 4.0, 6.0, 8.0}
    };
    check_svd("Wide matrix", A_wide);
    check_pinv("Wide matrix", A_wide);

    return 0;
}