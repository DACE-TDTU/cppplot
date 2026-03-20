/**
 * stress_test_control_logic.cpp
 * DCAS Lab - Kịch bản benchmark "độc hại" cho logic hệ thống
* g++ -std=c++17 -O2 -I../../cppplot/include stress_test_control_logic.cpp -o stress_test_control
*./stress_test_control.exe
 */

#include "cppplot/core/matrix.hpp"
#include "cppplot/control/state_space.hpp"
#include <iostream>
#include <iomanip>

using namespace cppplot;
using namespace cppplot::control;

void evaluate(const std::string& case_name, bool result, bool expected) {
    std::cout << std::left << std::setw(50) << case_name 
              << ": " << (result == expected ? "[ SUCCESS ]" : "[ FAILED ]") 
              << " (Got: " << result << ", Expected: " << expected << ")\n";
}

int main() {
    std::cout << "=== DCAS Lab: System Logic Stress Test (SVD-based) ===\n\n";

    // --- KỊCH BẢN 1: GẦN SUY BIẾN (NEAR SINGULAR) ---
    // Ma trận A có các mode cực gần nhau, gây khó cho việc tách biệt không gian con.
    // A = diag(1, 1 + 1e-13), B = [1; 1]
    // Nếu tol quá lớn, SVD sẽ coi 2 mode này là 1 -> Rank 1 (Uncontrollable)
    // Nếu tol quá nhỏ, nhiễu số học sẽ làm nó tưởng là Rank 2 (Controllable)
    Matrix A1 = {{1.0, 0.0}, {0.0, 1.0000000000001}};
    Matrix B1 = {{1.0}, {1.0}};
    evaluate("Case 1: Near-identical modes (Numerical Resolution)", 
             is_controllable(A1, B1, 1e-15), true);

    // --- KỊCH BẢN 2: CHUỖI TÍCH PHÂN BẬC CAO (VANDERMONDE STRESS) ---
    // A là ma trận Jordan bậc 6, B tác động vào mode cuối.
    // Ma trận ctrb sẽ là ma trận tam giác ngược, cực kỳ khó chịu về mặt số học khi n tăng.
    size_t n6 = 6;
    Matrix A2 = Matrix::zeros(n6, n6);
    for(size_t i = 0; i < n6-1; ++i) A2(i, i+1) = 1.0; 
    Matrix B2 = Matrix::zeros(n6, 1); B2(n6-1, 0) = 1.0;
    evaluate("Case 2: High-order Integrator Chain (Rank 6)", 
             is_controllable(A2, B2), true);

    // --- KỊCH BẢN 3: HỆ THỐNG MẤT TÍNH ĐIỀU KHIỂN DO TRÙNG CỰC VÀ ZERO ---
    // Hệ thống có một mode bị triệt tiêu bởi vị trí của B.
    // A = [1 0 0; 0 2 0; 0 0 3], B = [1; 0; 1] -> Mode 2 (trị riêng 2) không thể điều khiển.
    Matrix A3 = {{1, 0, 0}, {0, 2, 0}, {0, 0, 3}};
    Matrix B3 = {{1}, {0}, {1}};
    evaluate("Case 3: Hidden Uncontrollable Mode (Rank 2/3)", 
             is_controllable(A3, B3), false);

    // --- KỊCH BẢN 4: MA TRẬN A KHÔNG ĐẦY ĐỦ HẠNG (SINGULAR A) ---
    // A suy biến nặng nhưng hệ vẫn có thể điều khiển được (ví dụ hệ deadbeat).
    Matrix A4 = {{0, 1, 0}, {0, 0, 1}, {0, 0, 0}}; // Triple integrator
    Matrix B4 = {{0}, {0}, {1}};
    evaluate("Case 4: Singular A but Controllable", 
             is_controllable(A4, B4), true);

    // --- KỊCH BẢN 5: ĐỐI NGẪU QUAN SÁT (OBSERVABILITY STRESS) ---
    // C có các hàng phụ thuộc tuyến tính nhưng che giấu bởi scale khác nhau.
    Matrix A5 = {{1, 2}, {3, 4}};
    Matrix C5 = {{1, 1}, {1e-12, 1e-12}}; // Hàng 2 gần như bằng 0
    // Về lý thuyết toán học là Unobservable nếu chỉ có C5. 
    // Nhưng obsv = [C; CA], CA sẽ cứu vớt tính quan sát nếu hàng 1 và CA độc lập.
    evaluate("Case 5: Weakly Observable (Scaling Stress)", 
             is_observable(A5, C5), true);

    std::cout << "\n=== Stress test finished ===\n";
    return 0;
}