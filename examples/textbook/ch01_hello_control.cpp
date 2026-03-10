/**
 * @file ch01_hello_control.cpp
 * @brief First CppPlot program - plotting a sine wave
 * 
 * Compile: g++ -std=c++17 -I "../include" ch01_hello_control.cpp -o ch01_hello
 */

#include <cppplot/cppplot.hpp>
#include <vector>
#include <cmath>

using namespace cppplot;

constexpr double PI = 3.14159265358979323846;

int main() {
    // Generate data: sine wave
    std::vector<double> t, y;
    for (double ti = 0; ti <= 4 * PI; ti += 0.05) {
        t.push_back(ti);
        y.push_back(std::sin(ti));
    }
    
    // Create figure
    figure(800, 500);
    
    // Plot the sine wave
    plot(t, y, "b-", {{"linewidth", "2"}, {"label", "sin(t)"}});
    
    // Add labels and formatting
    xlabel("Time t [rad]");
    ylabel("Amplitude");
    title("My First CppPlot: Sine Wave");
    legend();
    grid(true);
    
    // Save to file
    savefig("ch01_hello_control.svg");
    
    std::cout << "Plot saved to ch01_hello_control.svg" << std::endl;
    
    return 0;
}
