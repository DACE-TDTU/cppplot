/**
 * benchmark_control_logic.cpp
 * DCAS Lab cppplot benchmark — Controllability & Observability (SVD-based)
 *
 * Build:
 * g++ -std=c++17 -O2 -I../../cppplot/include benchmark_control_logic.cpp -o bench_control
 * ./bench_control
 */

#include "cppplot/core/matrix.hpp"
#include "cppplot/control/state_space.hpp"
#include <iostream>
#include <iomanip>

using namespace cppplot;
using namespace cppplot::control;

void print_result(const std::string& label, bool result) {
    std::cout << std::left << std::setw(45) << label 
              << ": " << (result ? "[ PASS ]" : "[ FAIL ]") << "\n";
}

int main() {
    std::cout << "=== Control Logic Benchmark (SVD-based Rank) ===\n\n";

    // --- Case 1: Standard Controllable System (Double Integrator) ---
    // A = [0 1; 0 0], B = [0; 1] -> Rank(ctrb) = 2
    Matrix A1 = {{0, 1}, {0, 0}};
    Matrix B1 = {{0}, {1}};
    print_result("Case 1: Double Integrator Controllable", is_controllable(A1, B1));

    // --- Case 2: Uncontrollable System (Decoupled Mode) ---
    // A = [1 0; 0 2], B = [1; 0] -> Mode at 2 is not controllable. Rank(ctrb) = 1
    Matrix A2 = {{1, 0}, {0, 2}};
    Matrix B2 = {{1}, {0}};
    print_result("Case 2: Decoupled Mode Uncontrollable", !is_controllable(A2, B2));

    // --- Case 3: Observable System ---
    // A = [0 1; -2 -3], C = [1 0] -> Rank(obsv) = 2
    Matrix A3 = {{0, 1}, {-2, -3}};
    Matrix C3 = {{1, 0}};
    print_result("Case 3: Standard Observable System", is_observable(A3, C3));

    // --- Case 4: Unobservable System ---
    // A = [1 0; 0 1], C = [1 1] -> Rank(obsv) = 1
    Matrix A4 = {{1, 0}, {0, 1}};
    Matrix C4 = {{1, 1}};
    print_result("Case 4: Identity A Unobservable", !is_observable(A4, C4));

    // --- Case 5: Large-scale Numerical Stability (Rank 5 in 10x10) ---
    // Sử dụng ma trận A có các trị riêng phân biệt: A = diag(1, 2, ..., 10)
    // B chỉ tác động vào 5 mode đầu tiên.
    size_t n = 10;
    Matrix A5 = Matrix::zeros(n, n);
    for(size_t i = 0; i < n; ++i) A5(i, i) = static_cast<double>(i + 1); 
    
    Matrix B5 = Matrix::zeros(n, 1);
    for(size_t i = 0; i < 5; ++i) B5(i, 0) = 1.0; 
    
    Matrix Wc = ctrb(A5, B5);
    size_t r = Wc.rank();
    
    std::cout << "\nCase 5: High-dim Rank check (Expected 5): Rank = " << r << "\n";
    print_result("Case 5: Numerical Rank Stability", r == 5);

    return 0;
}