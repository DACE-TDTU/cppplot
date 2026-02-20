/**
 * @file log_scale_demo.cpp
 * @brief Demo cho tính năng log scale
 * 
 * Compile: g++ -std=c++14 -D_USE_MATH_DEFINES -I../include log_scale_demo.cpp -o log_scale_demo.exe
 */

#include <cppplot/cppplot.hpp>
#include <iostream>
#include <cmath>

using namespace cppplot;

int main() {
    std::cout << "CppPlot Log Scale Demo\n";
    std::cout << "======================\n\n";
    
    // Example 1: Basic Log Scale (Y axis)
    std::cout << "1. Basic Log Scale (Y axis)...\n";
    {
        figure(800, 600);
        
        auto x = linspace(1, 10, 100);
        std::vector<double> y_exp, y_quad, y_lin;
        
        for (double xi : x) {
            y_exp.push_back(std::pow(10, xi / 2));  // Exponential
            y_quad.push_back(xi * xi * 10);          // Quadratic
            y_lin.push_back(xi * 100);               // Linear
        }
        
        plot(x, y_exp, "r-", opts({{"linewidth", "2"}, {"label", "10^(x/2)"}}));
        plot(x, y_quad, "b-", opts({{"linewidth", "2"}, {"label", "10x²"}}));
        plot(x, y_lin, "g-", opts({{"linewidth", "2"}, {"label", "100x"}}));
        
        yscale("log");  // Logarithmic Y scale
        
        title("Comparison with Log Y Scale");
        xlabel("X");
        ylabel("Y (log scale)");
        legend(true);
        grid(true);
        
        savefig("output/log_01_basic_y.svg");
        clf();
        std::cout << "   Saved: output/log_01_basic_y.svg\n";
    }
    
    // Example 2: Log Scale (X axis) - Frequency Response
    std::cout << "2. Log Scale (X axis) - Bode Plot style...\n";
    {
        figure(800, 600);
        
        // Frequency response of a low-pass filter
        std::vector<double> freq, magnitude;
        double fc = 100;  // Cutoff frequency
        
        for (double f = 1; f <= 10000; f *= 1.1) {
            freq.push_back(f);
            double gain = 1.0 / std::sqrt(1 + std::pow(f/fc, 2));
            magnitude.push_back(20 * std::log10(gain));  // dB
        }
        
        plot(freq, magnitude, "b-", opts({{"linewidth", "2"}}));
        
        xscale("log");  // Logarithmic X scale
        
        // Add cutoff frequency marker
        axvline(fc, opts({{"color", "red"}, {"linestyle", "--"}, {"linewidth", "1.5"}}));
        axhline(-3, opts({{"color", "gray"}, {"linestyle", ":"}}));
        
        text(fc * 1.5, -2, "fc = 100 Hz", opts({{"color", "red"}}));
        text(1000, -3.5, "-3 dB point", opts({{"color", "gray"}}));
        
        title("Low-Pass Filter Frequency Response (Bode Plot)");
        xlabel("Frequency (Hz)");
        ylabel("Magnitude (dB)");
        grid(true);
        xlim(1, 10000);
        ylim(-40, 5);
        
        savefig("output/log_02_bode.svg");
        clf();
        std::cout << "   Saved: output/log_02_bode.svg\n";
    }
    
    // Example 3: Log-Log Plot - Power Law
    std::cout << "3. Log-Log Plot - Power Law...\n";
    {
        figure(800, 600);
        
        auto x = linspace(1, 1000, 100);
        std::vector<double> y_sq, y_cube, y_sqrt;
        
        for (double xi : x) {
            y_sq.push_back(xi * xi);           // x²
            y_cube.push_back(xi * xi * xi);    // x³
            y_sqrt.push_back(std::sqrt(xi));   // x^0.5
        }
        
        plot(x, y_sq, "r-", opts({{"linewidth", "2"}, {"label", "y = x²"}}));
        plot(x, y_cube, "b-", opts({{"linewidth", "2"}, {"label", "y = x³"}}));
        plot(x, y_sqrt, "g-", opts({{"linewidth", "2"}, {"label", "y = √x"}}));
        
        xscale("log");
        yscale("log");
        
        title("Power Laws in Log-Log Plot");
        xlabel("X (log scale)");
        ylabel("Y (log scale)");
        legend(true);
        grid(true);
        
        // Note: On log-log plot, power laws become straight lines
        // Slope = exponent
        
        savefig("output/log_03_loglog.svg");
        clf();
        std::cout << "   Saved: output/log_03_loglog.svg\n";
    }
    
    // Example 4: Semi-log Plot with Error Bars - Exponential Decay
    std::cout << "4. Semi-log with Error Bars - Exponential Decay...\n";
    {
        figure(800, 600);
        
        // Simulated radioactive decay data
        std::vector<double> t = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
        std::vector<double> N, N_err;
        double N0 = 1000;
        double tau = 3;  // Half-life
        
        for (double ti : t) {
            double n = N0 * std::exp(-ti / tau);
            N.push_back(n);
            N_err.push_back(std::sqrt(n) * 0.1);  // Poisson-like error
        }
        
        // Error bars on log scale
        errorbar(t, N, N_err, opts({{"color", "blue"}, {"capsize", "4"}, {"label", "Measured"}}));
        
        // Theoretical curve
        auto t_theory = linspace(0, 10, 100);
        std::vector<double> N_theory;
        for (double ti : t_theory) {
            N_theory.push_back(N0 * std::exp(-ti / tau));
        }
        plot(t_theory, N_theory, "r--", opts({{"linewidth", "2"}, {"label", "Theory"}}));
        
        yscale("log");
        
        axhline(N0/2, opts({{"color", "green"}, {"linestyle", ":"}}));
        text(5, N0/2 * 1.2, "Half-life point", opts({{"color", "green"}, {"fontsize", "10"}}));
        
        title("Radioactive Decay (Semi-Log Plot)");
        xlabel("Time (s)");
        ylabel("Count (log scale)");
        legend(true);
        grid(true);
        
        savefig("output/log_04_decay.svg");
        clf();
        std::cout << "   Saved: output/log_04_decay.svg\n";
    }
    
    // Example 5: Comparison Linear vs Log
    std::cout << "5. Comparison: Linear vs Log Scale...\n";
    {
        figure(1000, 400);
        layout(1, 2);
        
        auto x = linspace(0.1, 100, 200);
        std::vector<double> y;
        for (double xi : x) {
            y.push_back(std::pow(xi, 2));
        }
        
        // Linear scale
        subplot(1, 2, 1);
        plot(x, y, "b-", opts({{"linewidth", "2"}}));
        title("Linear Scale");
        xlabel("X");
        ylabel("Y = X²");
        grid(true);
        
        // Log scale
        subplot(1, 2, 2);
        plot(x, y, "b-", opts({{"linewidth", "2"}}));
        xscale("log");
        yscale("log");
        title("Log-Log Scale");
        xlabel("X (log)");
        ylabel("Y = X² (log)");
        grid(true);
        
        suptitle("Same Data: Different Scales");
        
        savefig("output/log_05_comparison.svg");
        clf();
        std::cout << "   Saved: output/log_05_comparison.svg\n";
    }
    
    std::cout << "\n======================\n";
    std::cout << "Log Scale demos completed!\n";
    std::cout << "\nUsage:\n";
    std::cout << "  xscale(\"log\");  // Set X axis to log scale\n";
    std::cout << "  yscale(\"log\");  // Set Y axis to log scale\n";
    std::cout << "  xscale(\"linear\");  // Set X axis to linear (default)\n";
    
    return 0;
}
