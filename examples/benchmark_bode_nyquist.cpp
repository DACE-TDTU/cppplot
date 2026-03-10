#include <cppplot/control/control.hpp>
#include <cppplot/cppplot.hpp>
#include <iostream>


using namespace cppplot;
using namespace cppplot::control;

int main() {
  // 1. Integrator: G(s) = 1/s
  TransferFunction G1({1}, {1, 0});

  // 2. 2nd order underdamped: G(s) = 1 / (s^2 + 0.2s + 1)
  TransferFunction G2({1}, {1, 0.2, 1});

  // 3. Non-minimum phase: G(s) = (1 - s) / (s^2 + 2s + 1)
  TransferFunction G3({-1, 1}, {1, 2, 1});

  std::cout << "Plotting Bode Diagrams..." << std::endl;

  BodeOptions opts;
  opts.margins = true;

  bode(G1, opts);
  savefig("output/bode_integrator.svg");

  bode(G2, opts);
  savefig("output/bode_2nd_order.svg");

  bode(G3, opts);
  savefig("output/bode_nmp.svg");

  std::cout << "Plotting Nyquist Diagrams..." << std::endl;

  NyquistOptions nopts;

  nyquist(G1, nopts);
  savefig("output/nyquist_integrator.svg");

  nyquist(G2, nopts);
  savefig("output/nyquist_2nd_order.svg");

  nyquist(G3, nopts);
  savefig("output/nyquist_nmp.svg");

  std::cout << "Done." << std::endl;
  return 0;
}
