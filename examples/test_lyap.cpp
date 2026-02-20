/**
 * Test Lyapunov solver for Mass-Spring-Damper case
 */

#include <iostream>
#include <iomanip>
#include "cppplot/control/control.hpp"

using namespace cppplot::control;

int main() {
    std::cout << "=== Lyapunov Solver Debug ===" << std::endl << std::flush;
    
    // Mass-spring-damper: A = [0 1; -2 -0.5]
    // This is stable (eigenvalues at -0.25 ± 1.39i)
    
    Matrix A = {{0, 1}, {-2, -0.5}};
    Matrix Q = {{10, 0}, {0, 1}};
    
    std::cout << "A = [" << A(0,0) << " " << A(0,1) << "; "
              << A(1,0) << " " << A(1,1) << "]" << std::endl << std::flush;
    std::cout << "trace(A) = " << (A(0,0) + A(1,1)) << std::endl << std::flush;
    std::cout << "det(A) = " << (A(0,0)*A(1,1) - A(0,1)*A(1,0)) << std::endl << std::flush;
    
    // First test Lyapunov with simple diagonal A
    std::cout << "\n--- Test 1: Diagonal stable A ---" << std::endl << std::flush;
    Matrix A_diag = {{-1, 0}, {0, -2}};
    Matrix Q_diag = {{1, 0}, {0, 1}};
    std::cout << "Solving A'X + XA + Q = 0 with A = diag(-1,-2)" << std::endl << std::flush;
    Matrix X_diag = lyapunov(A_diag, Q_diag, 100, 1e-8);
    std::cout << "X = diag(" << X_diag(0,0) << ", " << X_diag(1,1) << ")" << std::endl << std::flush;
    std::cout << "Expected: diag(0.5, 0.25)" << std::endl << std::flush;
    
    // Now test with mass-spring-damper
    std::cout << "\n--- Test 2: Mass-Spring-Damper A ---" << std::endl << std::flush;
    std::cout << "Solving Lyapunov: A'X + XA + Q = 0" << std::endl << std::flush;
    std::cout << "Calling lyapunov()..." << std::endl << std::flush;
    Matrix X = lyapunov(A, Q, 100, 1e-8);
    std::cout << "Done!" << std::endl << std::flush;
    
    std::cout << "\nX = " << std::endl << std::flush;
    std::cout << "  [" << X(0,0) << ", " << X(0,1) << "]" << std::endl << std::flush;
    std::cout << "  [" << X(1,0) << ", " << X(1,1) << "]" << std::endl << std::flush;
    
    // Check residual
    Matrix AT = A.T();
    Matrix Res = AT * X + X * A + Q;
    std::cout << "\nResidual A'X + XA + Q = " << std::endl << std::flush;
    std::cout << "  [" << Res(0,0) << ", " << Res(0,1) << "]" << std::endl << std::flush;
    std::cout << "  [" << Res(1,0) << ", " << Res(1,1) << "]" << std::endl << std::flush;
    
    double res_norm = 0;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            res_norm += Res(i,j) * Res(i,j);
        }
    }
    std::cout << "\nResidual norm: " << std::sqrt(res_norm) << std::endl << std::flush;
    
    return 0;
}
