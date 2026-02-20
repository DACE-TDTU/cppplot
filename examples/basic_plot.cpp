/**
 * @file basic_plot.cpp
 * @brief Basic plotting example
 * 
 * Compile: g++ -std=c++14 -D_USE_MATH_DEFINES -I../include basic_plot.cpp -o basic_plot
 * Run: ./basic_plot
 */

#include <cppplot/cppplot.hpp>
#include <iostream>
#include <cmath>

using namespace cppplot;

int main() {
    std::cout << "CppPlot Basic Example\n";
    std::cout << "=====================\n\n";
    
    // Example 1: Simple line plot
    std::cout << "1. Creating simple line plot...\n";
    {
        std::vector<double> x = {1, 2, 3, 4, 5};
        std::vector<double> y = {1, 4, 9, 16, 25};
        
        plot(x, y, "b-o");
        xlabel("X values");
        ylabel("Y values (x^2)");
        title("Simple Quadratic Plot");
        grid(true);
        savefig("output/01_simple_plot.svg");
        clf();
        
        std::cout << "   Saved: output/01_simple_plot.svg\n";
    }
    
    // Example 2: Multiple lines
    std::cout << "2. Creating multiple line plot...\n";
    {
        auto x = linspace(0, 2 * M_PI, 100);
        std::vector<double> y_sin, y_cos;
        
        for (double xi : x) {
            y_sin.push_back(std::sin(xi));
            y_cos.push_back(std::cos(xi));
        }
        
        plot(x, y_sin, "r-", opts({{"label", "sin(x)"}}));
        plot(x, y_cos, "b--", opts({{"label", "cos(x)"}}));
        xlabel("x");
        ylabel("y");
        title("Trigonometric Functions");
        legend(true);
        grid(true);
        savefig("output/02_multi_line.svg");
        clf();
        
        std::cout << "   Saved: output/02_multi_line.svg\n";
    }
    
    // Example 3: Scatter plot
    std::cout << "3. Creating scatter plot...\n";
    {
        auto x = random(50, 0, 10);
        auto y = random(50, 0, 10);
        
        scatter(x, y, opts({
            {"c", "steelblue"},
            {"s", "50"},
            {"alpha", "0.7"}
        }));
        xlabel("X");
        ylabel("Y");
        title("Random Scatter Plot");
        savefig("output/03_scatter.svg");
        clf();
        
        std::cout << "   Saved: output/03_scatter.svg\n";
    }
    
    // Example 4: Bar chart
    std::cout << "4. Creating bar chart...\n";
    {
        std::vector<double> x = {1, 2, 3, 4, 5};
        std::vector<double> values = {23, 45, 56, 78, 32};
        
        bar(x, values, opts({{"color", "coral"}}));
        xlabel("Category");
        ylabel("Value");
        title("Bar Chart Example");
        savefig("output/04_bar.svg");
        clf();
        
        std::cout << "   Saved: output/04_bar.svg\n";
    }
    
    // Example 5: Histogram
    std::cout << "5. Creating histogram...\n";
    {
        auto data = randn(1000, 0, 1);
        
        hist(data, 30, opts({{"color", "green"}}));
        xlabel("Value");
        ylabel("Frequency");
        title("Normal Distribution Histogram");
        savefig("output/05_histogram.svg");
        clf();
        
        std::cout << "   Saved: output/05_histogram.svg\n";
    }
    
    // Example 6: Different line styles
    std::cout << "6. Creating line styles demo...\n";
    {
        auto x = linspace(0, 10, 50);
        
        std::vector<double> y1, y2, y3, y4;
        for (double xi : x) {
            y1.push_back(xi);
            y2.push_back(xi + 2);
            y3.push_back(xi + 4);
            y4.push_back(xi + 6);
        }
        
        plot(x, y1, "b-", opts({{"label", "solid"}}));
        plot(x, y2, "r--", opts({{"label", "dashed"}}));
        plot(x, y3, "g-.", opts({{"label", "dash-dot"}}));
        plot(x, y4, "m:", opts({{"label", "dotted"}}));
        
        xlabel("X");
        ylabel("Y");
        title("Line Styles");
        legend(true);
        savefig("output/06_line_styles.svg");
        clf();
        
        std::cout << "   Saved: output/06_line_styles.svg\n";
    }
    
    // Example 7: Markers
    std::cout << "7. Creating markers demo...\n";
    {
        std::vector<double> x = {1, 2, 3, 4, 5, 6, 7};
        std::vector<double> y = {1, 1, 1, 1, 1, 1, 1};
        
        figure(800, 300);
        
        // Different markers
        plot({1.0}, {1.0}, "bo");
        plot({2.0}, {1.0}, "rs");
        plot({3.0}, {1.0}, "g^");
        plot({4.0}, {1.0}, "mv");
        plot({5.0}, {1.0}, "cx");
        plot({6.0}, {1.0}, "y+");
        plot({7.0}, {1.0}, "k*");
        
        xlim(0, 8);
        ylim(0, 2);
        title("Marker Types: o, s, ^, v, x, +, *");
        savefig("output/07_markers.svg");
        clf();
        
        std::cout << "   Saved: output/07_markers.svg\n";
    }
    
    std::cout << "\nAll examples completed!\n";
    std::cout << "Check the 'output' folder for SVG files.\n";
    
    return 0;
}
