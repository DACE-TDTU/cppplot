#include <cppplot/control/control.hpp>
#include <cppplot/cppplot.hpp>
#include <iostream>

using namespace cppplot;
using namespace cppplot::control;

int main() {
  Matrix A = {{0, 1}, {0, 0}};
  Matrix B = {{0}, {1}};
  Matrix Q = {{10, 0}, {0, 1}};
  Matrix R = {{1}};

  // lqr now returns Matrix (1×n row matrix)
  Matrix K = lqr(A, B, Q, R);
  std::cout << "K = [" << K(0, 0) << ", " << K(0, 1) << "]" << std::endl;

  // Direct matrix arithmetic: B*K works directly
  Matrix Acl = A - B * K;

  auto eigs = Acl.eigenvalues();
  std::cout << "Closed-loop poles:" << std::endl;
  for (const auto &e : eigs)
    std::cout << "  " << e.real() << " + " << e.imag() << "j" << std::endl;

  // Test observer_gain (returns n×1 column matrix)
  Matrix C = {{1, 0}};
  Matrix L = observer_gain(
      A, C, {std::complex<double>(-5, 0), std::complex<double>(-6, 0)});
  std::cout << "Observer L = [" << L(0, 0) << "; " << L(1, 0) << "]"
            << std::endl;

  // Test dlqr
  double dt = 0.1;
  Matrix Ad = {{1, dt}, {0, 1}};
  Matrix Bd = {{0.5 * dt * dt}, {dt}};
  Matrix Kd = dlqr(Ad, Bd, Q, R);
  std::cout << "DLQR K = [" << Kd(0, 0) << ", " << Kd(0, 1) << "]" << std::endl;

  // Test acker/place
  Matrix Kp = place(A, B, {-3.0, -4.0});
  std::cout << "Place K = [" << Kp(0, 0) << ", " << Kp(0, 1) << "]"
            << std::endl;

  std::cout << "OK" << std::endl;
  return 0;
}
