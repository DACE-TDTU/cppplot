/**
 * @file latex_demo.cpp
 * @brief Demo tính năng LaTeX rendering cho công thức toán học
 * 
 * Compile: g++ -std=c++14 -D_USE_MATH_DEFINES -I../include latex_demo.cpp -o latex_demo.exe
 */

#include <cppplot/cppplot.hpp>
#include <iostream>
#include <cmath>

using namespace cppplot;

int main() {
    std::cout << "CppPlot LaTeX Rendering Demo\n";
    std::cout << "============================\n\n";
    
    // Example 1: Greek Letters
    std::cout << "1. Greek Letters...\n";
    {
        figure(800, 600);
        
        auto x = linspace(0, 2 * M_PI, 100);
        std::vector<double> y_sin, y_cos;
        for (double xi : x) {
            y_sin.push_back(std::sin(xi));
            y_cos.push_back(std::cos(xi));
        }
        
        plot(x, y_sin, "b-", opts({{"linewidth", "2"}, {"label", "sin(θ)"}}));
        plot(x, y_cos, "r-", opts({{"linewidth", "2"}, {"label", "cos(θ)"}}));
        
        // Add Greek letter annotations
        text(M_PI/2, 1.0, "\\theta = \\pi/2", opts({{"fontsize", "12"}, {"ha", "center"}, {"va", "bottom"}}));
        text(M_PI, 0.0, "\\theta = \\pi", opts({{"fontsize", "12"}, {"ha", "center"}, {"va", "bottom"}}));
        text(3*M_PI/2, -1.0, "\\theta = 3\\pi/2", opts({{"fontsize", "12"}, {"ha", "center"}, {"va", "top"}}));
        
        title("Trigonometric Functions with Greek Letters");
        xlabel("\\theta (radians)");
        ylabel("Amplitude");
        legend(true);
        grid(true);
        
        savefig("output/latex_01_greek.svg");
        clf();
        std::cout << "   Saved: output/latex_01_greek.svg\n";
    }
    
    // Example 2: Superscripts and Subscripts
    std::cout << "2. Superscripts and Subscripts...\n";
    {
        figure(800, 600);
        
        auto x = linspace(0, 5, 50);
        std::vector<double> y1, y2, y3;
        for (double xi : x) {
            y1.push_back(xi);           // x
            y2.push_back(xi * xi);       // x²
            y3.push_back(xi * xi * xi);  // x³
        }
        
        plot(x, y1, "b-", opts({{"linewidth", "2"}, {"label", "y = x"}}));
        plot(x, y2, "r-", opts({{"linewidth", "2"}, {"label", "y = x^2"}}));
        plot(x, y3, "g-", opts({{"linewidth", "2"}, {"label", "y = x^3"}}));
        
        // Add annotations with superscripts
        text(4.5, 4.5, "y = x", opts({{"fontsize", "11"}, {"color", "blue"}}));
        text(4.2, 18, "y = x^2", opts({{"fontsize", "11"}, {"color", "red"}}));
        text(3.5, 50, "y = x^3", opts({{"fontsize", "11"}, {"color", "green"}}));
        
        // Subscripts example
        text(1, 100, "x_1, x_2, x_3, ..., x_n", opts({{"fontsize", "12"}}));
        
        title("Power Functions: x^n");
        xlabel("x");
        ylabel("y");
        legend(true);
        grid(true);
        ylim(0, 130);
        
        savefig("output/latex_02_scripts.svg");
        clf();
        std::cout << "   Saved: output/latex_02_scripts.svg\n";
    }
    
    // Example 3: Mathematical Formulas
    std::cout << "3. Mathematical Formulas...\n";
    {
        figure(900, 700);
        
        // Gaussian function
        auto x = linspace(-4, 4, 200);
        std::vector<double> y;
        double sigma = 1.0;
        double mu = 0.0;
        
        for (double xi : x) {
            double val = (1.0 / (sigma * std::sqrt(2 * M_PI))) * 
                         std::exp(-0.5 * std::pow((xi - mu) / sigma, 2));
            y.push_back(val);
        }
        
        fill_between(x, std::vector<double>(x.size(), 0), y, 
                     opts({{"color", "steelblue"}, {"alpha", "0.3"}}));
        plot(x, y, "b-", opts({{"linewidth", "2"}}));
        
        // Add formula
        text(2.0, 0.35, "f(x) = \\frac{1}{\\sigma\\sqrt{2\\pi}} e^{-\\frac{(x-\\mu)^2}{2\\sigma^2}}", 
             opts({{"fontsize", "14"}, {"color", "darkblue"}}));
        
        // Add parameter values
        text(2.0, 0.30, "\\mu = 0, \\sigma = 1", opts({{"fontsize", "12"}}));
        
        // Mark mean
        axvline(0, opts({{"color", "red"}, {"linestyle", "--"}}));
        text(0.1, 0.42, "\\mu", opts({{"fontsize", "14"}, {"color", "red"}}));
        
        // Mark standard deviations
        axvline(1, opts({{"color", "orange"}, {"linestyle", ":"}}));
        axvline(-1, opts({{"color", "orange"}, {"linestyle", ":"}}));
        text(1.1, 0.25, "+\\sigma", opts({{"fontsize", "11"}, {"color", "orange"}}));
        text(-1.4, 0.25, "-\\sigma", opts({{"fontsize", "11"}, {"color", "orange"}}));
        
        title("Normal Distribution (Gaussian)");
        xlabel("x");
        ylabel("Probability Density");
        grid(true);
        
        savefig("output/latex_03_gaussian.svg");
        clf();
        std::cout << "   Saved: output/latex_03_gaussian.svg\n";
    }
    
    // Example 4: Physics Equations
    std::cout << "4. Physics Equations...\n";
    {
        figure(900, 600);
        
        // Projectile motion
        double v0 = 20;  // Initial velocity
        double angle = 45 * M_PI / 180;  // 45 degrees
        double g = 9.8;
        
        std::vector<double> t_vals, x_vals, y_vals;
        double t_max = 2 * v0 * std::sin(angle) / g;
        
        for (double t = 0; t <= t_max; t += 0.05) {
            t_vals.push_back(t);
            x_vals.push_back(v0 * std::cos(angle) * t);
            y_vals.push_back(v0 * std::sin(angle) * t - 0.5 * g * t * t);
        }
        
        plot(x_vals, y_vals, "b-", opts({{"linewidth", "2.5"}}));
        
        // Add equations at figure positions
        figtext(0.7, 0.85, "Projectile Motion", opts({{"fontsize", "14"}, {"fontweight", "bold"}}));
        figtext(0.7, 0.78, "x(t) = v_0 cos(\\theta) t", opts({{"fontsize", "12"}}));
        figtext(0.7, 0.72, "y(t) = v_0 sin(\\theta) t - \\frac{1}{2}gt^2", opts({{"fontsize", "12"}}));
        figtext(0.7, 0.66, "v_0 = 20 m/s", opts({{"fontsize", "11"}}));
        figtext(0.7, 0.61, "\\theta = 45\\degree", opts({{"fontsize", "11"}}));
        figtext(0.7, 0.56, "g = 9.8 m/s^2", opts({{"fontsize", "11"}}));
        
        // Mark max height
        double t_apex = v0 * std::sin(angle) / g;
        double x_apex = v0 * std::cos(angle) * t_apex;
        double y_apex = v0 * std::sin(angle) * t_apex - 0.5 * g * t_apex * t_apex;
        
        scatter({x_apex}, {y_apex}, opts({{"s", "50"}, {"color", "red"}}));
        text(x_apex + 1, y_apex, "h_{max}", opts({{"fontsize", "11"}, {"color", "red"}}));
        
        title("Projectile Motion Trajectory");
        xlabel("Horizontal Distance (m)");
        ylabel("Height (m)");
        grid(true);
        ylim(0, 12);
        
        savefig("output/latex_04_physics.svg");
        clf();
        std::cout << "   Saved: output/latex_04_physics.svg\n";
    }
    
    // Example 5: Calculus Notation
    std::cout << "5. Calculus Notation...\n";
    {
        figure(900, 650);
        
        auto x = linspace(0, 2 * M_PI, 100);
        std::vector<double> y, dy;
        for (double xi : x) {
            y.push_back(std::sin(xi));
            dy.push_back(std::cos(xi));  // Derivative
        }
        
        plot(x, y, "b-", opts({{"linewidth", "2"}, {"label", "f(x) = sin(x)"}}));
        plot(x, dy, "r--", opts({{"linewidth", "2"}, {"label", "f'(x) = cos(x)"}}));
        
        // Add calculus notation
        text(4.5, 0.7, "\\frac{d}{dx}sin(x) = cos(x)", opts({{"fontsize", "13"}}));
        text(4.5, 0.5, "\\int cos(x)dx = sin(x) + C", opts({{"fontsize", "13"}}));
        
        // Integral symbol with limits
        text(0.5, -0.7, "\\int_0^{2\\pi} sin(x)dx = 0", opts({{"fontsize", "12"}}));
        
        title("Derivatives and Integrals");
        xlabel("x");
        ylabel("y");
        legend(true);
        grid(true);
        
        savefig("output/latex_05_calculus.svg");
        clf();
        std::cout << "   Saved: output/latex_05_calculus.svg\n";
    }
    
    // Example 6: Math Symbols Overview
    std::cout << "6. Math Symbols Overview...\n";
    {
        figure(1000, 800);
        
        // Just show symbols, no data
        plot({0, 10}, {0, 10}, "w-");  // Invisible line to set axes
        
        // Greek letters
        figtext(0.1, 0.92, "Greek Letters:", opts({{"fontsize", "14"}, {"fontweight", "bold"}}));
        figtext(0.1, 0.87, "\\alpha \\beta \\gamma \\delta \\epsilon \\zeta \\eta \\theta", opts({{"fontsize", "12"}}));
        figtext(0.1, 0.82, "\\iota \\kappa \\lambda \\mu \\nu \\xi \\pi \\rho", opts({{"fontsize", "12"}}));
        figtext(0.1, 0.77, "\\sigma \\tau \\upsilon \\phi \\chi \\psi \\omega", opts({{"fontsize", "12"}}));
        figtext(0.5, 0.87, "\\Gamma \\Delta \\Theta \\Lambda \\Xi \\Pi \\Sigma \\Phi \\Psi \\Omega", opts({{"fontsize", "12"}}));
        
        // Math operators
        figtext(0.1, 0.68, "Operators:", opts({{"fontsize", "14"}, {"fontweight", "bold"}}));
        figtext(0.1, 0.63, "\\sum \\prod \\int \\partial \\nabla \\infty", opts({{"fontsize", "12"}}));
        figtext(0.5, 0.63, "\\pm \\times \\div \\cdot \\circ \\bullet", opts({{"fontsize", "12"}}));
        
        // Relations
        figtext(0.1, 0.54, "Relations:", opts({{"fontsize", "14"}, {"fontweight", "bold"}}));
        figtext(0.1, 0.49, "\\leq \\geq \\neq \\approx \\equiv \\sim \\propto", opts({{"fontsize", "12"}}));
        figtext(0.5, 0.49, "\\subset \\supset \\in \\ni \\notin", opts({{"fontsize", "12"}}));
        
        // Arrows
        figtext(0.1, 0.40, "Arrows:", opts({{"fontsize", "14"}, {"fontweight", "bold"}}));
        figtext(0.1, 0.35, "\\to \\leftarrow \\Rightarrow \\Leftarrow \\leftrightarrow", opts({{"fontsize", "12"}}));
        
        // Logic
        figtext(0.1, 0.26, "Logic:", opts({{"fontsize", "14"}, {"fontweight", "bold"}}));
        figtext(0.1, 0.21, "\\forall \\exists \\neg \\wedge \\vee \\cap \\cup", opts({{"fontsize", "12"}}));
        
        // Superscripts/Subscripts
        figtext(0.1, 0.12, "Scripts:", opts({{"fontsize", "14"}, {"fontweight", "bold"}}));
        figtext(0.1, 0.07, "x^2  x^{10}  x_1  x_{12}  a^{n+1}  E_k", opts({{"fontsize", "12"}}));
        
        title("LaTeX Math Symbols Reference", opts({{"fontsize", "16"}}));
        
        // Hide axes for this reference chart
        xlim(-1, 11);
        ylim(-1, 11);
        
        savefig("output/latex_06_symbols.svg");
        clf();
        std::cout << "   Saved: output/latex_06_symbols.svg\n";
    }
    
    // Example 7: Complex Formula - Euler's Identity
    std::cout << "7. Famous Equations...\n";
    {
        figure(800, 600);
        
        // Unit circle
        auto theta = linspace(0, 2 * M_PI, 100);
        std::vector<double> x_circle, y_circle;
        for (double t : theta) {
            x_circle.push_back(std::cos(t));
            y_circle.push_back(std::sin(t));
        }
        
        plot(x_circle, y_circle, "b-", opts({{"linewidth", "2"}}));
        
        // Mark e^(i*pi)
        scatter({-1}, {0}, opts({{"s", "100"}, {"color", "red"}}));
        text(-0.9, 0.15, "e^{i\\pi} = -1", opts({{"fontsize", "14"}, {"color", "red"}}));
        
        // Mark e^(i*pi/2)
        scatter({0}, {1}, opts({{"s", "80"}, {"color", "green"}}));
        text(0.1, 1.1, "e^{i\\pi/2} = i", opts({{"fontsize", "12"}, {"color", "green"}}));
        
        // Euler's formula box
        figtext(0.65, 0.85, "Euler's Formula:", opts({{"fontsize", "14"}, {"fontweight", "bold"}}));
        figtext(0.65, 0.78, "e^{i\\theta} = cos(\\theta) + i sin(\\theta)", opts({{"fontsize", "13"}}));
        figtext(0.65, 0.68, "Euler's Identity:", opts({{"fontsize", "14"}, {"fontweight", "bold"}}));
        figtext(0.65, 0.61, "e^{i\\pi} + 1 = 0", opts({{"fontsize", "16"}, {"color", "darkblue"}}));
        
        // Axes
        axhline(0, opts({{"color", "gray"}, {"linewidth", "0.5"}}));
        axvline(0, opts({{"color", "gray"}, {"linewidth", "0.5"}}));
        
        // Labels
        text(1.1, 0, "Re", opts({{"fontsize", "11"}}));
        text(0.05, 1.15, "Im", opts({{"fontsize", "11"}}));
        
        title("The Unit Circle and Euler's Formula");
        xlabel("Real Axis");
        ylabel("Imaginary Axis");
        xlim(-1.5, 1.5);
        ylim(-1.5, 1.5);
        grid(true);
        
        savefig("output/latex_07_euler.svg");
        clf();
        std::cout << "   Saved: output/latex_07_euler.svg\n";
    }
    
    // Example 8: Statistical Formulas
    std::cout << "8. Statistical Formulas...\n";
    {
        figure(900, 600);
        
        // Sample data histogram
        auto data = randn(500, 50, 10);  // Mean=50, StdDev=10
        hist(data, 20, opts({{"color", "steelblue"}, {"alpha", "0.7"}}));
        
        // Add statistical formulas
        figtext(0.65, 0.88, "Statistics:", opts({{"fontsize", "14"}, {"fontweight", "bold"}}));
        figtext(0.65, 0.82, "Mean: \\mu = \\frac{1}{n}\\sum_{i=1}^{n} x_i", opts({{"fontsize", "11"}}));
        figtext(0.65, 0.74, "Variance: \\sigma^2 = \\frac{1}{n}\\sum_{i=1}^{n}(x_i - \\mu)^2", opts({{"fontsize", "11"}}));
        figtext(0.65, 0.66, "Std Dev: \\sigma = \\sqrt{\\sigma^2}", opts({{"fontsize", "11"}}));
        
        // Sample statistics
        double sum = 0, sum_sq = 0;
        for (double x : data) sum += x;
        double mean = sum / data.size();
        for (double x : data) sum_sq += (x - mean) * (x - mean);
        double stddev = std::sqrt(sum_sq / data.size());
        
        std::ostringstream oss;
        oss << "\\mu \\approx " << std::fixed << std::setprecision(1) << mean;
        figtext(0.65, 0.56, oss.str(), opts({{"fontsize", "12"}, {"color", "red"}}));
        
        oss.str("");
        oss << "\\sigma \\approx " << std::fixed << std::setprecision(1) << stddev;
        figtext(0.65, 0.50, oss.str(), opts({{"fontsize", "12"}, {"color", "red"}}));
        
        // Mark mean
        axvline(mean, opts({{"color", "red"}, {"linewidth", "2"}}));
        
        title("Sample Distribution with Statistical Formulas");
        xlabel("Value");
        ylabel("Frequency");
        grid(true);
        
        savefig("output/latex_08_statistics.svg");
        clf();
        std::cout << "   Saved: output/latex_08_statistics.svg\n";
    }
    
    std::cout << "\n============================\n";
    std::cout << "LaTeX demos completed!\n";
    std::cout << "\nSupported LaTeX features:\n";
    std::cout << "  Greek letters:  \\alpha, \\beta, \\gamma, \\Gamma, \\Delta, etc.\n";
    std::cout << "  Superscripts:   x^2, x^{10}, e^{i\\pi}\n";
    std::cout << "  Subscripts:     x_1, x_{12}, a_n\n";
    std::cout << "  Operators:      \\sum, \\prod, \\int, \\partial, \\nabla\n";
    std::cout << "  Relations:      \\leq, \\geq, \\neq, \\approx, \\equiv\n";
    std::cout << "  Arrows:         \\to, \\leftarrow, \\Rightarrow\n";
    std::cout << "  Logic:          \\forall, \\exists, \\wedge, \\vee\n";
    std::cout << "  Misc:           \\infty, \\pm, \\times, \\div, \\sqrt\n";
    std::cout << "\nNew functions:\n";
    std::cout << "  text(x, y, \"\\\\alpha^2\")      - Text at data coords\n";
    std::cout << "  figtext(x, y, \"formula\")      - Text at figure coords (0-1)\n";
    std::cout << "  latex(x, y, \"$...$\")          - Explicit LaTeX rendering\n";
    
    return 0;
}
