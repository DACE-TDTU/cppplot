/**
 * @file scientific.cpp
 * @brief Scientific plotting examples
 * 
 * Compile: g++ -std=c++14 -D_USE_MATH_DEFINES -I../include scientific.cpp -o scientific
 */

#include <cppplot/cppplot.hpp>
#include <iostream>
#include <cmath>
#include <numeric>

using namespace cppplot;

// Sigmoid function
double sigmoid(double x) {
    return 1.0 / (1.0 + std::exp(-x));
}

// Gaussian function
double gaussian(double x, double mu, double sigma) {
    double z = (x - mu) / sigma;
    return std::exp(-0.5 * z * z) / (sigma * std::sqrt(2 * M_PI));
}

int main() {
    std::cout << "CppPlot Scientific Examples\n";
    std::cout << "===========================\n\n";
    
    // Example 1: Function comparison
    std::cout << "1. Comparing activation functions...\n";
    {
        auto x = linspace(-6, 6, 100);
        
        std::vector<double> y_sigmoid, y_tanh, y_relu;
        for (double xi : x) {
            y_sigmoid.push_back(sigmoid(xi));
            y_tanh.push_back(std::tanh(xi));
            y_relu.push_back(std::max(0.0, xi));
        }
        
        figure(800, 500);
        plot(x, y_sigmoid, "b-", opts({{"label", "Sigmoid"}}));
        plot(x, y_tanh, "r-", opts({{"label", "Tanh"}}));
        plot(x, y_relu, "g-", opts({{"label", "ReLU"}}));
        
        xlabel("x");
        ylabel("f(x)");
        title("Neural Network Activation Functions");
        legend(true);
        grid(true);
        ylim(-1.5, 6);
        savefig("output/11_activation_functions.svg");
        clf();
        
        std::cout << "   Saved: output/11_activation_functions.svg\n";
    }
    
    // Example 2: Gaussian distributions
    std::cout << "2. Plotting Gaussian distributions...\n";
    {
        auto x = linspace(-5, 5, 200);
        
        std::vector<double> y1, y2, y3;
        for (double xi : x) {
            y1.push_back(gaussian(xi, 0, 0.5));
            y2.push_back(gaussian(xi, 0, 1.0));
            y3.push_back(gaussian(xi, 0, 2.0));
        }
        
        figure(800, 500);
        plot(x, y1, "b-", opts({{"label", "sigma = 0.5"}}));
        plot(x, y2, "r-", opts({{"label", "sigma = 1.0"}}));
        plot(x, y3, "g-", opts({{"label", "sigma = 2.0"}}));
        
        xlabel("x");
        ylabel("Probability Density");
        title("Gaussian Distributions (mu = 0)");
        legend(true);
        grid(true);
        savefig("output/12_gaussian.svg");
        clf();
        
        std::cout << "   Saved: output/12_gaussian.svg\n";
    }
    
    // Example 3: Exponential growth/decay
    std::cout << "3. Plotting exponential functions...\n";
    {
        auto t = linspace(0, 5, 100);
        
        std::vector<double> growth, decay, oscillating;
        for (double ti : t) {
            growth.push_back(std::exp(0.5 * ti));
            decay.push_back(5 * std::exp(-0.5 * ti));
            oscillating.push_back(3 * std::exp(-0.3 * ti) * std::cos(2 * M_PI * ti));
        }
        
        Figure& fig = figure(1000, 400);
        fig.suptitle("Exponential Functions");
        
        {
            Axes& ax = fig.subplot(1, 3, 1);
            ax.plot(t, growth, "r-");
            ax.set_title("Exponential Growth");
            ax.set_xlabel("t");
            ax.set_ylabel("e^(0.5t)");
            ax.grid(true);
        }
        
        {
            Axes& ax = fig.subplot(1, 3, 2);
            ax.plot(t, decay, "b-");
            ax.set_title("Exponential Decay");
            ax.set_xlabel("t");
            ax.set_ylabel("5e^(-0.5t)");
            ax.grid(true);
        }
        
        {
            Axes& ax = fig.subplot(1, 3, 3);
            ax.plot(t, oscillating, "g-");
            ax.set_title("Damped Oscillation");
            ax.set_xlabel("t");
            ax.set_ylabel("Amplitude");
            ax.grid(true);
        }
        
        fig.savefig("output/13_exponential.svg");
        std::cout << "   Saved: output/13_exponential.svg\n";
    }
    
    // Example 4: Polynomial comparison
    std::cout << "4. Comparing polynomials...\n";
    {
        auto x = linspace(-2, 2, 100);
        
        std::vector<double> y1, y2, y3, y4;
        for (double xi : x) {
            y1.push_back(xi);
            y2.push_back(xi * xi);
            y3.push_back(xi * xi * xi);
            y4.push_back(xi * xi * xi * xi);
        }
        
        figure(700, 600);
        plot(x, y1, "b-", opts({{"label", "x"}}));
        plot(x, y2, "r-", opts({{"label", "x^2"}}));
        plot(x, y3, "g-", opts({{"label", "x^3"}}));
        plot(x, y4, "m-", opts({{"label", "x^4"}}));
        
        xlabel("x");
        ylabel("y");
        title("Polynomial Functions");
        legend(true);
        grid(true);
        ylim(-5, 10);
        savefig("output/14_polynomials.svg");
        clf();
        
        std::cout << "   Saved: output/14_polynomials.svg\n";
    }
    
    // Example 5: Data with noise
    std::cout << "5. Plotting noisy data with trend...\n";
    {
        auto x = linspace(0, 10, 50);
        auto noise = randn(50, 0, 0.5);
        
        std::vector<double> y_true, y_noisy;
        for (size_t i = 0; i < x.size(); ++i) {
            double y = 2 * x[i] + 1; // True line
            y_true.push_back(y);
            y_noisy.push_back(y + noise[i]);
        }
        
        figure(700, 500);
        scatter(x, y_noisy, opts({{"c", "blue"}, {"alpha", "0.6"}}));
        plot(x, y_true, "r-", opts({{"label", "True: y = 2x + 1"}}));
        
        xlabel("x");
        ylabel("y");
        title("Noisy Data with Linear Trend");
        legend(true);
        grid(true);
        savefig("output/15_noisy_data.svg");
        clf();
        
        std::cout << "   Saved: output/15_noisy_data.svg\n";
    }
    
    // Example 6: Statistical distribution comparison
    std::cout << "6. Histogram comparison...\n";
    {
        auto data1 = randn(500, 0, 1);
        auto data2 = randn(500, 2, 1.5);
        
        Figure& fig = figure(1000, 400);
        fig.suptitle("Distribution Comparison");
        
        {
            Axes& ax = fig.subplot(1, 2, 1);
            ax.hist(data1, 25, opts({{"color", "steelblue"}}));
            ax.set_title("N(0, 1)");
            ax.set_xlabel("Value");
            ax.set_ylabel("Frequency");
        }
        
        {
            Axes& ax = fig.subplot(1, 2, 2);
            ax.hist(data2, 25, opts({{"color", "coral"}}));
            ax.set_title("N(2, 1.5)");
            ax.set_xlabel("Value");
            ax.set_ylabel("Frequency");
        }
        
        fig.savefig("output/16_histogram_comparison.svg");
        std::cout << "   Saved: output/16_histogram_comparison.svg\n";
    }
    
    std::cout << "\nAll scientific examples completed!\n";
    
    return 0;
}
