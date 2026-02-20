/**
 * @file style_demo.cpp
 * @brief Demo các tính năng tùy chỉnh style: color, linestyle, linewidth, alpha...
 * 
 * Compile: g++ -std=c++14 -D_USE_MATH_DEFINES -I../include style_demo.cpp -o style_demo
 */

#include <cppplot/cppplot.hpp>
#include <iostream>
#include <cmath>

using namespace cppplot;

int main() {
    std::cout << "CppPlot Style Customization Demo\n";
    std::cout << "=================================\n\n";
    
    // Example 1: Different line styles (linestyle/ls)
    std::cout << "1. Line styles demo (linestyle option)...\n";
    {
        auto x = linspace(0, 10, 100);
        std::vector<double> y1, y2, y3, y4;
        for (double xi : x) {
            y1.push_back(std::sin(xi));
            y2.push_back(std::sin(xi) + 2);
            y3.push_back(std::sin(xi) + 4);
            y4.push_back(std::sin(xi) + 6);
        }
        
        figure(900, 600);
        
        // Using linestyle option
        plot(x, y1, "", opts({{"color", "blue"}, {"linestyle", "-"}, {"linewidth", "2"}, {"label", "solid (-)"}}));
        plot(x, y2, "", opts({{"color", "red"}, {"ls", "--"}, {"lw", "2"}, {"label", "dashed (--)"}}));
        plot(x, y3, "", opts({{"color", "green"}, {"linestyle", "-."}, {"linewidth", "2"}, {"label", "dash-dot (-.)"}}));
        plot(x, y4, "", opts({{"color", "purple"}, {"ls", ":"}, {"lw", "2"}, {"label", "dotted (:)"}}));
        
        xlabel("X");
        ylabel("Y");
        title("Line Styles: solid, dashed, dash-dot, dotted");
        legend(true);
        grid(true);
        savefig("output/style_01_linestyles.svg");
        clf();
        
        std::cout << "   Saved: output/style_01_linestyles.svg\n";
    }
    
    // Example 2: Different line widths (linewidth/lw)
    std::cout << "2. Line widths demo (linewidth option)...\n";
    {
        auto x = linspace(0, 10, 100);
        std::vector<double> y1, y2, y3, y4, y5;
        for (double xi : x) {
            y1.push_back(std::cos(xi));
            y2.push_back(std::cos(xi) + 1.5);
            y3.push_back(std::cos(xi) + 3);
            y4.push_back(std::cos(xi) + 4.5);
            y5.push_back(std::cos(xi) + 6);
        }
        
        figure(900, 600);
        
        // Using linewidth option
        plot(x, y1, "b-", opts({{"linewidth", "0.5"}, {"label", "lw=0.5"}}));
        plot(x, y2, "r-", opts({{"lw", "1.5"}, {"label", "lw=1.5"}}));
        plot(x, y3, "g-", opts({{"linewidth", "3"}, {"label", "lw=3"}}));
        plot(x, y4, "m-", opts({{"lw", "5"}, {"label", "lw=5"}}));
        plot(x, y5, "c-", opts({{"linewidth", "8"}, {"label", "lw=8"}}));
        
        xlabel("X");
        ylabel("Y");
        title("Line Widths: 0.5, 1.5, 3, 5, 8");
        legend(true);
        grid(true);
        savefig("output/style_02_linewidths.svg");
        clf();
        
        std::cout << "   Saved: output/style_02_linewidths.svg\n";
    }
    
    // Example 3: Colors - Named colors
    std::cout << "3. Named colors demo...\n";
    {
        auto x = linspace(0, 2*M_PI, 50);
        
        figure(900, 600);
        
        std::vector<std::string> colors = {"red", "orange", "gold", "green", "blue", 
                                            "purple", "pink", "brown", "gray", "black"};
        
        for (size_t i = 0; i < colors.size(); ++i) {
            std::vector<double> y;
            for (double xi : x) {
                y.push_back(std::sin(xi) + i * 1.2);
            }
            plot(x, y, "-", opts({{"color", colors[i]}, {"linewidth", "2.5"}, {"label", colors[i]}}));
        }
        
        xlabel("X");
        ylabel("Y");
        title("Named Colors Demo");
        legend(true);
        savefig("output/style_03_named_colors.svg");
        clf();
        
        std::cout << "   Saved: output/style_03_named_colors.svg\n";
    }
    
    // Example 4: Alpha transparency
    std::cout << "4. Alpha transparency demo...\n";
    {
        auto x = linspace(0, 10, 100);
        std::vector<double> y;
        for (double xi : x) y.push_back(std::sin(xi) * 5);
        
        figure(900, 600);
        
        // Multiple overlapping lines with different alpha values
        for (int i = 0; i < 10; ++i) {
            std::vector<double> yi;
            for (double xi : x) {
                yi.push_back(std::sin(xi + i * 0.3) * 5);
            }
            double alpha = 0.1 + i * 0.1;
            plot(x, yi, "-", opts({{"color", "blue"}, {"linewidth", "3"}, 
                                   {"alpha", str(alpha)}, {"label", "alpha=" + str(alpha)}}));
        }
        
        xlabel("X");
        ylabel("Y");
        title("Alpha Transparency: 0.1 to 1.0");
        legend(true);
        grid(true);
        savefig("output/style_04_alpha.svg");
        clf();
        
        std::cout << "   Saved: output/style_04_alpha.svg\n";
    }
    
    // Example 5: Markers with styles
    std::cout << "5. Marker styles demo...\n";
    {
        std::vector<double> x = {1, 2, 3, 4, 5, 6, 7, 8};
        
        figure(900, 600);
        
        std::vector<std::string> markers = {"o", "s", "^", "v", "x", "+", "*", "d"};
        std::vector<std::string> labels = {"circle (o)", "square (s)", "triangle-up (^)", 
                                           "triangle-down (v)", "x-mark (x)", "plus (+)", 
                                           "star (*)", "diamond (d)"};
        
        for (size_t i = 0; i < markers.size(); ++i) {
            std::vector<double> y(x.size(), static_cast<double>(i + 1));
            plot(x, y, "", opts({
                {"color", "steelblue"},
                {"marker", markers[i]},
                {"markersize", "12"},
                {"linestyle", "-"},
                {"linewidth", "1"},
                {"label", labels[i]}
            }));
        }
        
        xlabel("X");
        ylabel("Marker Type");
        title("Marker Styles");
        legend(true);
        xlim(0, 9);
        ylim(0, 9);
        savefig("output/style_05_markers.svg");
        clf();
        
        std::cout << "   Saved: output/style_05_markers.svg\n";
    }
    
    // Example 6: Marker face and edge colors
    std::cout << "6. Marker face/edge colors demo...\n";
    {
        auto x = linspace(0, 10, 20);
        
        figure(900, 600);
        
        // Different marker color combinations
        std::vector<double> y1, y2, y3, y4;
        for (double xi : x) {
            y1.push_back(std::sin(xi));
            y2.push_back(std::sin(xi) + 2);
            y3.push_back(std::sin(xi) + 4);
            y4.push_back(std::sin(xi) + 6);
        }
        
        plot(x, y1, "-o", opts({
            {"color", "blue"}, {"markerfacecolor", "white"}, {"markeredgecolor", "blue"},
            {"markersize", "10"}, {"markeredgewidth", "2"}, {"label", "white fill, blue edge"}
        }));
        
        plot(x, y2, "-s", opts({
            {"color", "red"}, {"mfc", "red"}, {"mec", "darkred"},
            {"ms", "10"}, {"mew", "2"}, {"label", "red fill, darkred edge"}
        }));
        
        plot(x, y3, "-^", opts({
            {"color", "green"}, {"markerfacecolor", "yellow"}, {"markeredgecolor", "green"},
            {"markersize", "10"}, {"markeredgewidth", "2"}, {"label", "yellow fill, green edge"}
        }));
        
        plot(x, y4, "-d", opts({
            {"color", "purple"}, {"mfc", "cyan"}, {"mec", "purple"},
            {"ms", "10"}, {"mew", "2"}, {"label", "cyan fill, purple edge"}
        }));
        
        xlabel("X");
        ylabel("Y");
        title("Marker Face and Edge Colors");
        legend(true);
        grid(true);
        savefig("output/style_06_marker_colors.svg");
        clf();
        
        std::cout << "   Saved: output/style_06_marker_colors.svg\n";
    }
    
    // Example 7: Scatter plot styles
    std::cout << "7. Scatter plot styles demo...\n";
    {
        auto x1 = random(30, 0, 5);
        auto y1 = random(30, 0, 5);
        auto x2 = random(30, 5, 10);
        auto y2 = random(30, 5, 10);
        auto x3 = random(30, 0, 5);
        auto y3 = random(30, 5, 10);
        
        figure(900, 600);
        
        scatter(x1, y1, opts({
            {"c", "red"}, {"s", "100"}, {"marker", "o"}, 
            {"alpha", "0.7"}, {"label", "Group A"}
        }));
        
        scatter(x2, y2, opts({
            {"color", "blue"}, {"s", "150"}, {"marker", "s"},
            {"alpha", "0.7"}, {"label", "Group B"}
        }));
        
        scatter(x3, y3, opts({
            {"facecolor", "yellow"}, {"edgecolor", "black"}, 
            {"s", "200"}, {"marker", "^"}, {"linewidth", "2"},
            {"alpha", "0.8"}, {"label", "Group C"}
        }));
        
        xlabel("X");
        ylabel("Y");
        title("Scatter Plot with Custom Styles");
        legend(true);
        grid(true);
        savefig("output/style_07_scatter_styles.svg");
        clf();
        
        std::cout << "   Saved: output/style_07_scatter_styles.svg\n";
    }
    
    // Example 8: Bar chart styles
    std::cout << "8. Bar chart styles demo...\n";
    {
        std::vector<double> x1 = {1, 2, 3, 4, 5};
        std::vector<double> h1 = {20, 35, 30, 25, 40};
        std::vector<double> x2 = {1.3, 2.3, 3.3, 4.3, 5.3};
        std::vector<double> h2 = {25, 32, 28, 35, 30};
        
        figure(900, 600);
        
        bar(x1, h1, opts({
            {"color", "steelblue"}, {"edgecolor", "navy"},
            {"linewidth", "2"}, {"alpha", "0.8"}, {"label", "Series A"}
        }));
        
        bar(x2, h2, opts({
            {"color", "coral"}, {"ec", "darkred"},
            {"lw", "2"}, {"alpha", "0.8"}, {"label", "Series B"}
        }));
        
        xlabel("Category");
        ylabel("Value");
        title("Bar Chart with Custom Colors and Styles");
        legend(true);
        savefig("output/style_08_bar_styles.svg");
        clf();
        
        std::cout << "   Saved: output/style_08_bar_styles.svg\n";
    }
    
    // Example 9: Combined demo - scientific plot
    std::cout << "9. Scientific plot demo...\n";
    {
        auto t = linspace(0, 4*M_PI, 200);
        std::vector<double> signal, noise, filtered;
        
        for (double ti : t) {
            signal.push_back(std::sin(ti) + 0.5 * std::sin(3*ti));
        }
        
        auto n = randn(200, 0, 0.3);
        for (size_t i = 0; i < t.size(); ++i) {
            noise.push_back(signal[i] + n[i]);
        }
        
        // Simple moving average filter
        filtered = noise;
        int window = 5;
        for (size_t i = window; i < noise.size() - window; ++i) {
            double sum = 0;
            for (int j = -window; j <= window; ++j) {
                sum += noise[i + j];
            }
            filtered[i] = sum / (2*window + 1);
        }
        
        figure(1000, 600);
        
        // Noisy data - light, semi-transparent
        plot(t, noise, "", opts({
            {"color", "lightgray"}, {"linewidth", "1"}, {"alpha", "0.7"},
            {"label", "Noisy signal"}
        }));
        
        // Original signal - dashed
        plot(t, signal, "", opts({
            {"color", "blue"}, {"linestyle", "--"}, {"linewidth", "2"},
            {"alpha", "0.8"}, {"label", "Original signal"}
        }));
        
        // Filtered signal - solid, bold
        plot(t, filtered, "", opts({
            {"color", "red"}, {"linestyle", "-"}, {"linewidth", "2.5"},
            {"label", "Filtered signal"}
        }));
        
        xlabel("Time (t)");
        ylabel("Amplitude");
        title("Signal Processing: Original, Noisy, and Filtered");
        legend(true);
        grid(true);
        savefig("output/style_09_scientific.svg");
        clf();
        
        std::cout << "   Saved: output/style_09_scientific.svg\n";
    }
    
    std::cout << "\n=================================\n";
    std::cout << "All style demos completed!\n";
    std::cout << "Check 'output' folder for SVG files.\n";
    std::cout << "\nSupported options:\n";
    std::cout << "  Line: color/c, linestyle/ls, linewidth/lw, alpha\n";
    std::cout << "  Marker: marker, markersize/ms, markerfacecolor/mfc,\n";
    std::cout << "          markeredgecolor/mec, markeredgewidth/mew\n";
    std::cout << "  Scatter: c/color, facecolor/fc, edgecolor/ec, s (size),\n";
    std::cout << "           marker, alpha, linewidth/lw\n";
    std::cout << "  Bar: color/facecolor/fc, edgecolor/ec, linewidth/lw, alpha\n";
    std::cout << "  Common: label\n";
    
    return 0;
}
