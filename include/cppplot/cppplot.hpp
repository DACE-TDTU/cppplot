/**
 * @file cppplot.hpp
 * @brief Main header file for CppPlot library
 * 
 * CppPlot - A Matplotlib-style Plotting Library for C++
 * 
 * Include this single header to access all CppPlot functionality.
 * 
 * @example
 * #include <cppplot/cppplot.hpp>
 * using namespace cppplot;
 * 
 * int main() {
 *     std::vector<double> x = {1, 2, 3, 4, 5};
 *     std::vector<double> y = {1, 4, 9, 16, 25};
 *     plot(x, y, "b-o");
 *     savefig("plot.svg");
 *     return 0;
 * }
 */

#ifndef CPPPLOT_CPPPLOT_HPP
#define CPPPLOT_CPPPLOT_HPP

#define CPPPLOT_VERSION_MAJOR 1
#define CPPPLOT_VERSION_MINOR 0
#define CPPPLOT_VERSION_PATCH 0
#define CPPPLOT_VERSION "1.0.0"

// Core components
#include "core/types.hpp"
#include "core/color.hpp"
#include "core/style.hpp"
#include "core/utils.hpp"
#include "core/matrix.hpp"

// Figure and Axes
#include "figure.hpp"
#include "axes.hpp"

// Backends
#include "backends/backend.hpp"
#include "backends/svg_backend.hpp"

// High-level API (pyplot-style)
#include "pyplot.hpp"

// Robotics helpers
#include "robotics/robotics.hpp"

#endif // CPPPLOT_CPPPLOT_HPP
