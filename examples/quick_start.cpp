/**
 * @file quick_start.cpp
 * @brief Quick start example - compile and run this first!
 * 
 * Compile (Windows with MinGW):
 *   g++ -std=c++17 -I../include quick_start.cpp -o quick_start
 * 
 * Compile (Linux/Mac):
 *   g++ -std=c++17 -I../include quick_start.cpp -o quick_start
 * 
 * Run:
 *   ./quick_start
 * 
 * This will generate a simple SVG plot file.
 */

#include <cppplot/cppplot.hpp>
#include <iostream>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace cppplot;

int main() {
    std::cout << "=================================\n";
    std::cout << "  CppPlot Quick Start Example\n";
    std::cout << "=================================\n\n";
    
    // ============================================
    // EXAMPLE 1: Most basic plot
    // ============================================
    std::cout << "Creating basic plot...\n";
    
    // Create some data
    std::vector<double> x = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::vector<double> y = {0, 1, 4, 9, 16, 25, 36, 49, 64, 81, 100};
    
    // Plot it!
    plot(x, y, "b-o");      // Blue line with circle markers
    xlabel("X");
    ylabel("Y = X^2");
    title("My First CppPlot!");
    grid(true);
    
    // Save to file
    savefig("quick_start_plot.svg");
    std::cout << "Saved: quick_start_plot.svg\n\n";
    
    // Clear for next plot
    clf();
    
    // ============================================
    // EXAMPLE 2: Sine wave
    // ============================================
    std::cout << "Creating sine wave plot...\n";
    
    // Generate smooth sine wave
    auto t = linspace(0, 4 * M_PI, 200);
    std::vector<double> sine_wave;
    for (double ti : t) {
        sine_wave.push_back(std::sin(ti));
    }
    
    plot(t, sine_wave, "r-");
    xlabel("Time (radians)");
    ylabel("Amplitude");
    title("Sine Wave");
    grid(true);
    savefig("sine_wave.svg");
    std::cout << "Saved: sine_wave.svg\n\n";
    
    clf();
    
    // ============================================
    // EXAMPLE 3: Multiple lines
    // ============================================
    std::cout << "Creating multi-line plot...\n";
    
    std::vector<double> y1, y2, y3;
    for (double ti : t) {
        y1.push_back(std::sin(ti));
        y2.push_back(std::sin(ti + M_PI/4));
        y3.push_back(std::sin(ti + M_PI/2));
    }
    
    plot(t, y1, "r-", {{"label", "Phase 0"}});
    plot(t, y2, "g-", {{"label", "Phase pi/4"}});
    plot(t, y3, "b-", {{"label", "Phase pi/2"}});
    xlabel("Time");
    ylabel("Value");
    title("Phase-shifted Sine Waves");
    legend(true);
    grid(true);
    savefig("phase_shift.svg");
    std::cout << "Saved: phase_shift.svg\n\n";
    
    clf();
    
    // ============================================
    // EXAMPLE 4: Scatter plot
    // ============================================
    std::cout << "Creating scatter plot...\n";
    
    auto scatter_x = random(50, 0, 10);
    auto scatter_y = random(50, 0, 10);
    
    scatter(scatter_x, scatter_y, {
        {"c", "blue"},
        {"s", "40"}
    });
    xlabel("X");
    ylabel("Y");
    title("Random Points");
    savefig("scatter_plot.svg");
    std::cout << "Saved: scatter_plot.svg\n\n";
    
    clf();
    
    // ============================================
    // EXAMPLE 5: Bar chart
    // ============================================
    std::cout << "Creating bar chart...\n";
    
    std::vector<double> categories = {1, 2, 3, 4, 5};
    std::vector<double> values = {20, 35, 30, 45, 25};
    
    bar(categories, values, {{"color", "steelblue"}});
    xlabel("Category");
    ylabel("Value");
    title("Sales Data");
    savefig("bar_chart.svg");
    std::cout << "Saved: bar_chart.svg\n\n";
    
    clf();
    
    // ============================================
    // EXAMPLE 6: Histogram
    // ============================================
    std::cout << "Creating histogram...\n";
    
    auto normal_data = randn(1000, 50, 10);  // Mean=50, StdDev=10
    
    hist(normal_data, 30, {{"color", "coral"}});
    xlabel("Value");
    ylabel("Frequency");
    title("Normal Distribution (mu=50, sigma=10)");
    savefig("histogram.svg");
    std::cout << "Saved: histogram.svg\n\n";
    
    // ============================================
    // Done!
    // ============================================
    std::cout << "=================================\n";
    std::cout << "  All plots created!\n";
    std::cout << "=================================\n";
    std::cout << "\nOpen the .svg files in a web browser to view.\n";
    
    return 0;
}
