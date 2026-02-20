/**
 * @file font_demo.cpp
 * @brief Demo tùy chỉnh font: fontfamily, fontsize, fontweight, color...
 * 
 * Compile: g++ -std=c++14 -D_USE_MATH_DEFINES -I../include font_demo.cpp -o font_demo
 */

#include <cppplot/cppplot.hpp>
#include <iostream>
#include <cmath>

using namespace cppplot;

int main() {
    std::cout << "CppPlot Font Customization Demo\n";
    std::cout << "================================\n\n";
    
    // Example 1: Basic font size customization
    std::cout << "1. Font size customization...\n";
    {
        auto x = linspace(0, 2*M_PI, 100);
        std::vector<double> y;
        for (double xi : x) y.push_back(std::sin(xi));
        
        figure(900, 600);
        plot(x, y, "b-", opts({{"linewidth", "2"}}));
        
        // Customize font sizes using PlotOptions
        title("Large Title (20pt)", opts({{"fontsize", "20"}, {"fontweight", "bold"}}));
        xlabel("X Axis Label (14pt)", opts({{"fontsize", "14"}}));
        ylabel("Y Axis Label (14pt)", opts({{"fontsize", "14"}}));
        
        // Customize tick labels
        tick_params(opts({{"labelsize", "12"}}));
        
        grid(true);
        savefig("output/font_01_sizes.svg");
        clf();
        
        std::cout << "   Saved: output/font_01_sizes.svg\n";
    }
    
    // Example 2: Different font families
    std::cout << "2. Font families demo...\n";
    {
        auto x = linspace(0, 10, 50);
        std::vector<double> y;
        for (double xi : x) y.push_back(xi * xi);
        
        figure(900, 600);
        plot(x, y, "r-", opts({{"linewidth", "2"}, {"label", "y = x^2"}}));
        
        // Use different font families
        title("Times New Roman Title", opts({
            {"fontfamily", "Times New Roman, serif"}, 
            {"fontsize", "18"}
        }));
        xlabel("Georgia X Label", opts({
            {"fontfamily", "Georgia, serif"}, 
            {"fontsize", "14"}
        }));
        ylabel("Courier Y Label", opts({
            {"fontfamily", "Courier New, monospace"}, 
            {"fontsize", "14"}
        }));
        
        legend(true, opts({
            {"fontfamily", "Verdana, sans-serif"},
            {"fontsize", "12"}
        }));
        
        grid(true);
        savefig("output/font_02_families.svg");
        clf();
        
        std::cout << "   Saved: output/font_02_families.svg\n";
    }
    
    // Example 3: Colored text
    std::cout << "3. Colored text demo...\n";
    {
        auto x = linspace(0, 4*M_PI, 200);
        std::vector<double> y_sin, y_cos;
        for (double xi : x) {
            y_sin.push_back(std::sin(xi));
            y_cos.push_back(std::cos(xi));
        }
        
        figure(900, 600);
        plot(x, y_sin, "b-", opts({{"linewidth", "2"}, {"label", "sin(x)"}}));
        plot(x, y_cos, "r-", opts({{"linewidth", "2"}, {"label", "cos(x)"}}));
        
        // Colored title and labels
        title("Trigonometric Functions", opts({
            {"fontsize", "18"}, {"fontweight", "bold"}, {"color", "darkblue"}
        }));
        xlabel("Angle (radians)", opts({
            {"fontsize", "14"}, {"color", "darkgreen"}
        }));
        ylabel("Value", opts({
            {"fontsize", "14"}, {"color", "darkred"}
        }));
        
        // Colored tick labels
        gca().set_xtick_color("purple");
        gca().set_ytick_color("orange");
        
        legend(true, opts({{"fontsize", "11"}, {"color", "navy"}}));
        grid(true);
        savefig("output/font_03_colors.svg");
        clf();
        
        std::cout << "   Saved: output/font_03_colors.svg\n";
    }
    
    // Example 4: Using rc() for global font settings
    std::cout << "4. Global font settings with rc()...\n";
    {
        auto x = linspace(-5, 5, 100);
        std::vector<double> y;
        for (double xi : x) y.push_back(std::exp(-xi*xi/2));
        
        figure(900, 600);
        
        // Set global font family
        rc("font", opts({{"family", "Georgia, serif"}, {"size", "14"}}));
        
        plot(x, y, "m-", opts({{"linewidth", "2.5"}, {"label", "Gaussian"}}));
        
        title("Gaussian Distribution");
        xlabel("x");
        ylabel("f(x) = exp(-x²/2)");
        
        legend(true);
        grid(true);
        savefig("output/font_04_rc_global.svg");
        clf();
        
        std::cout << "   Saved: output/font_04_rc_global.svg\n";
    }
    
    // Example 5: Using rc() for specific element groups
    std::cout << "5. Element-specific font settings with rc()...\n";
    {
        auto x = linspace(0, 10, 100);
        std::vector<double> y;
        for (double xi : x) y.push_back(std::log(xi + 1));
        
        figure(900, 600);
        
        // Configure different font settings for different elements
        rc("axes", opts({{"titlesize", "20"}, {"titleweight", "bold"}, {"labelsize", "14"}}));
        rc("xtick", opts({{"labelsize", "11"}, {"color", "blue"}}));
        rc("ytick", opts({{"labelsize", "11"}, {"color", "red"}}));
        rc("legend", opts({{"fontsize", "12"}}));
        
        plot(x, y, "g-", opts({{"linewidth", "2"}, {"label", "log(x+1)"}}));
        
        title("Logarithmic Function");
        xlabel("X Value");
        ylabel("Y = log(x + 1)");
        
        legend(true);
        grid(true);
        savefig("output/font_05_rc_elements.svg");
        clf();
        
        std::cout << "   Saved: output/font_05_rc_elements.svg\n";
    }
    
    // Example 6: Scientific publication style
    std::cout << "6. Scientific publication style...\n";
    {
        auto x = linspace(0, 100, 50);
        std::vector<double> y1, y2;
        for (double xi : x) {
            y1.push_back(100 * (1 - std::exp(-xi/30)));
            y2.push_back(80 * (1 - std::exp(-xi/50)));
        }
        
        figure(900, 600);
        
        // Set all fonts to a serif family (publication style)
        fontfamily("Times New Roman, serif");
        
        plot(x, y1, "-", opts({
            {"color", "black"}, {"linewidth", "1.5"}, 
            {"marker", "o"}, {"markersize", "5"},
            {"label", "Treatment A"}
        }));
        plot(x, y2, "--", opts({
            {"color", "black"}, {"linewidth", "1.5"},
            {"marker", "s"}, {"markersize", "5"},
            {"label", "Treatment B"}
        }));
        
        title("Dose-Response Curve", opts({{"fontsize", "16"}, {"fontweight", "bold"}}));
        xlabel("Concentration (μM)", opts({{"fontsize", "14"}}));
        ylabel("Response (%)", opts({{"fontsize", "14"}}));
        
        tick_params(opts({{"labelsize", "12"}}));
        legend(true, opts({{"fontsize", "12"}}));
        
        grid(true);
        savefig("output/font_06_publication.svg");
        clf();
        
        std::cout << "   Saved: output/font_06_publication.svg\n";
    }
    
    // Example 7: Poster/presentation style (large fonts)
    std::cout << "7. Poster/presentation style (large fonts)...\n";
    {
        std::vector<double> categories = {1, 2, 3, 4, 5};
        std::vector<double> values = {45, 72, 58, 89, 63};
        
        figure(1000, 700);
        
        bar(categories, values, opts({
            {"color", "steelblue"}, {"edgecolor", "navy"}, {"linewidth", "2"}
        }));
        
        // Large fonts for poster
        title("Quarterly Sales Performance", opts({
            {"fontsize", "28"}, {"fontweight", "bold"}, {"color", "darkblue"}
        }));
        xlabel("Quarter", opts({{"fontsize", "20"}, {"fontweight", "bold"}}));
        ylabel("Sales (K$)", opts({{"fontsize", "20"}, {"fontweight", "bold"}}));
        
        // Large tick labels
        tick_params(opts({{"labelsize", "16"}}));
        
        savefig("output/font_07_poster.svg");
        clf();
        
        std::cout << "   Saved: output/font_07_poster.svg\n";
    }
    
    // Example 8: Mixed fonts for emphasis
    std::cout << "8. Mixed fonts for emphasis...\n";
    {
        auto x = linspace(0, 2*M_PI, 100);
        std::vector<double> y;
        for (double xi : x) y.push_back(std::sin(xi) * std::exp(-xi/10));
        
        figure(900, 600);
        plot(x, y, "r-", opts({{"linewidth", "2.5"}, {"label", "Damped sine wave"}}));
        
        // Bold title with Arial
        title("Damped Oscillation", opts({
            {"fontfamily", "Arial, sans-serif"},
            {"fontsize", "18"},
            {"fontweight", "bold"}
        }));
        
        // Italic-style labels (using oblique fonts)
        xlabel("Time (seconds)", opts({
            {"fontfamily", "Georgia, serif"},
            {"fontsize", "14"}
        }));
        ylabel("Amplitude", opts({
            {"fontfamily", "Georgia, serif"},
            {"fontsize", "14"}
        }));
        
        // Monospace legend
        legend(true, opts({
            {"fontfamily", "Courier New, monospace"},
            {"fontsize", "11"}
        }));
        
        grid(true);
        savefig("output/font_08_mixed.svg");
        clf();
        
        std::cout << "   Saved: output/font_08_mixed.svg\n";
    }
    
    std::cout << "\n================================\n";
    std::cout << "All font demos completed!\n";
    std::cout << "Check 'output' folder for SVG files.\n";
    std::cout << "\nFont customization options:\n";
    std::cout << "\n  title(), xlabel(), ylabel():\n";
    std::cout << "    fontsize  - Font size in points\n";
    std::cout << "    fontfamily/family - Font family (e.g., 'Arial, sans-serif')\n";
    std::cout << "    fontweight/weight - Font weight ('normal', 'bold')\n";
    std::cout << "    color - Text color\n";
    std::cout << "\n  tick_params():\n";
    std::cout << "    labelsize - Tick label font size\n";
    std::cout << "    labelfamily - Tick label font family\n";
    std::cout << "    labelcolor - Tick label color\n";
    std::cout << "\n  legend():\n";
    std::cout << "    fontsize - Legend font size\n";
    std::cout << "    fontfamily/family - Legend font family\n";
    std::cout << "    fontweight - Legend font weight\n";
    std::cout << "    color/labelcolor - Legend text color\n";
    std::cout << "\n  rc(group, opts) - rcParams-like settings:\n";
    std::cout << "    group='font': family, size, weight\n";
    std::cout << "    group='axes': titlesize, labelsize, titleweight, labelweight\n";
    std::cout << "    group='xtick'/'ytick': labelsize, color\n";
    std::cout << "    group='legend': fontsize\n";
    std::cout << "\n  fontfamily(family) - Set all text to same font family\n";
    
    return 0;
}
