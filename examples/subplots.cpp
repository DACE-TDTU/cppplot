/**
 * @file subplots.cpp
 * @brief Subplot examples
 * 
 * Compile: g++ -std=c++17 -I../include subplots.cpp -o subplots
 */

#include <cppplot/cppplot.hpp>
#include <iostream>
#include <cmath>

using namespace cppplot;

int main() {
    std::cout << "CppPlot Subplots Example\n";
    std::cout << "========================\n\n";
    
    // Example 1: 2x2 subplots
    std::cout << "1. Creating 2x2 subplot grid...\n";
    {
        auto fig = figure(1000, 800);
        fig.suptitle("2x2 Subplot Grid");
        
        auto x = linspace(0, 2 * M_PI, 50);
        
        // Subplot 1: sin
        {
            auto& ax = fig.subplot(2, 2, 1);
            std::vector<double> y;
            for (double xi : x) y.push_back(std::sin(xi));
            ax.plot(x, y, "b-");
            ax.set_title("sin(x)");
            ax.grid(true);
        }
        
        // Subplot 2: cos
        {
            auto& ax = fig.subplot(2, 2, 2);
            std::vector<double> y;
            for (double xi : x) y.push_back(std::cos(xi));
            ax.plot(x, y, "r-");
            ax.set_title("cos(x)");
            ax.grid(true);
        }
        
        // Subplot 3: tan (limited range)
        {
            auto& ax = fig.subplot(2, 2, 3);
            std::vector<double> y;
            for (double xi : x) {
                double val = std::tan(xi);
                y.push_back(std::clamp(val, -5.0, 5.0));
            }
            ax.plot(x, y, "g-");
            ax.set_title("tan(x) [clipped]");
            ax.set_ylim(-5, 5);
            ax.grid(true);
        }
        
        // Subplot 4: sin^2 + cos^2
        {
            auto& ax = fig.subplot(2, 2, 4);
            std::vector<double> y;
            for (double xi : x) {
                y.push_back(std::sin(xi)*std::sin(xi) + std::cos(xi)*std::cos(xi));
            }
            ax.plot(x, y, "m-");
            ax.set_title("sin²(x) + cos²(x) = 1");
            ax.grid(true);
        }
        
        fig.savefig("output/08_subplots_2x2.svg");
        std::cout << "   Saved: output/08_subplots_2x2.svg\n";
    }
    
    // Example 2: 1x3 horizontal layout
    std::cout << "2. Creating 1x3 horizontal layout...\n";
    {
        auto fig = figure(1200, 400);
        fig.suptitle("Different Plot Types");
        
        // Line plot
        {
            auto& ax = fig.subplot(1, 3, 1);
            auto x = linspace(0, 10, 50);
            std::vector<double> y;
            for (double xi : x) y.push_back(std::exp(-xi/5) * std::sin(xi));
            ax.plot(x, y, "b-");
            ax.set_title("Line Plot");
            ax.set_xlabel("x");
            ax.set_ylabel("y");
        }
        
        // Scatter plot
        {
            auto& ax = fig.subplot(1, 3, 2);
            auto x = random(30, 0, 10);
            auto y = random(30, 0, 10);
            ax.scatter(x, y);
            ax.set_title("Scatter Plot");
            ax.set_xlabel("x");
            ax.set_ylabel("y");
        }
        
        // Bar chart
        {
            auto& ax = fig.subplot(1, 3, 3);
            std::vector<double> x = {1, 2, 3, 4, 5};
            std::vector<double> values = {3, 7, 5, 9, 4};
            ax.bar(x, values, {{"color", std::string("coral")}});
            ax.set_title("Bar Chart");
            ax.set_xlabel("Category");
            ax.set_ylabel("Value");
        }
        
        fig.savefig("output/09_subplots_1x3.svg");
        std::cout << "   Saved: output/09_subplots_1x3.svg\n";
    }
    
    // Example 3: 3x1 vertical layout
    std::cout << "3. Creating 3x1 vertical layout...\n";
    {
        auto fig = figure(600, 900);
        fig.suptitle("Vertical Layout");
        
        auto x = linspace(0, 10, 100);
        
        // Plot 1
        {
            auto& ax = fig.subplot(3, 1, 1);
            std::vector<double> y;
            for (double xi : x) y.push_back(xi * xi);
            ax.plot(x, y, "b-");
            ax.set_title("y = x²");
            ax.grid(true);
        }
        
        // Plot 2
        {
            auto& ax = fig.subplot(3, 1, 2);
            std::vector<double> y;
            for (double xi : x) y.push_back(std::sqrt(xi));
            ax.plot(x, y, "r-");
            ax.set_title("y = √x");
            ax.grid(true);
        }
        
        // Plot 3
        {
            auto& ax = fig.subplot(3, 1, 3);
            std::vector<double> y;
            for (double xi : x) y.push_back(std::log(xi + 1));
            ax.plot(x, y, "g-");
            ax.set_title("y = ln(x+1)");
            ax.grid(true);
        }
        
        fig.savefig("output/10_subplots_3x1.svg");
        std::cout << "   Saved: output/10_subplots_3x1.svg\n";
    }
    
    std::cout << "\nSubplots examples completed!\n";
    
    return 0;
}
