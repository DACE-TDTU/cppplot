/**
 * @file advanced_demo.cpp
 * @brief Demo các tính năng nâng cao: error bars, fill_between, annotations, scales
 * 
 * Compile: g++ -std=c++14 -D_USE_MATH_DEFINES -I../include advanced_demo.cpp -o advanced_demo
 */

#include <cppplot/cppplot.hpp>
#include <iostream>
#include <cmath>

using namespace cppplot;

int main() {
    std::cout << "CppPlot Advanced Features Demo\n";
    std::cout << "===============================\n\n";
    
    // Example 1: Error Bars
    std::cout << "1. Error Bars...\n";
    {
        figure(800, 600);
        
        // Data with measurement errors
        std::vector<double> x = {1, 2, 3, 4, 5, 6, 7, 8};
        std::vector<double> y = {2.3, 4.1, 5.8, 8.2, 9.9, 12.1, 14.3, 16.0};
        std::vector<double> yerr = {0.5, 0.4, 0.6, 0.5, 0.7, 0.6, 0.8, 0.5};
        
        errorbar(x, y, yerr, opts({{"color", "steelblue"}, {"capsize", "5"}, {"label", "Measured"}}));
        
        // Add theoretical line
        auto x_fit = linspace(0, 9, 100);
        std::vector<double> y_fit;
        for (double xi : x_fit) y_fit.push_back(2 * xi);
        plot(x_fit, y_fit, "r--", opts({{"label", "Theory: y = 2x"}}));
        
        title("Experimental Data with Error Bars");
        xlabel("Time (s)");
        ylabel("Distance (m)");
        legend(true);
        grid(true);
        
        savefig("output/advanced_01_errorbar.svg");
        clf();
        std::cout << "   Saved: output/advanced_01_errorbar.svg\n";
    }
    
    // Example 2: Fill Between (Confidence Interval)
    std::cout << "2. Fill Between (Confidence Interval)...\n";
    {
        figure(800, 600);
        
        auto x = linspace(0, 10, 100);
        std::vector<double> y_mean, y_upper, y_lower;
        
        for (double xi : x) {
            double val = std::sin(xi) * std::exp(-xi * 0.1);
            y_mean.push_back(val);
            y_upper.push_back(val + 0.2 + xi * 0.02);  // Growing uncertainty
            y_lower.push_back(val - 0.2 - xi * 0.02);
        }
        
        // Fill confidence region
        fill_between(x, y_lower, y_upper, opts({{"color", "steelblue"}, {"alpha", "0.3"}, {"label", "95% CI"}}));
        
        // Plot mean line
        plot(x, y_mean, "b-", opts({{"linewidth", "2"}, {"label", "Mean"}}));
        
        title("Signal with Confidence Interval");
        xlabel("Time (s)");
        ylabel("Amplitude");
        legend(true);
        grid(true);
        
        savefig("output/advanced_02_fill_between.svg");
        clf();
        std::cout << "   Saved: output/advanced_02_fill_between.svg\n";
    }
    
    // Example 3: Multiple Fill Regions (Stack Plot)
    std::cout << "3. Stacked Fill Areas...\n";
    {
        figure(800, 600);
        
        auto x = linspace(0, 10, 50);
        std::vector<double> y1, y2, y3;
        std::vector<double> zero(x.size(), 0);
        
        for (double xi : x) {
            y1.push_back(3 + std::sin(xi) * 0.5);
            y2.push_back(2 + std::cos(xi * 0.5) * 0.3);
            y3.push_back(1.5 + std::sin(xi * 2) * 0.2);
        }
        
        // Calculate cumulative
        std::vector<double> c1 = y1;
        std::vector<double> c2(x.size()), c3(x.size());
        for (size_t i = 0; i < x.size(); ++i) {
            c2[i] = c1[i] + y2[i];
            c3[i] = c2[i] + y3[i];
        }
        
        // Stack fills
        fill_between(x, zero, c1, opts({{"color", "coral"}, {"alpha", "0.7"}, {"label", "Product A"}}));
        fill_between(x, c1, c2, opts({{"color", "seagreen"}, {"alpha", "0.7"}, {"label", "Product B"}}));
        fill_between(x, c2, c3, opts({{"color", "steelblue"}, {"alpha", "0.7"}, {"label", "Product C"}}));
        
        title("Stacked Area Chart");
        xlabel("Month");
        ylabel("Sales (M$)");
        legend(true);
        ylim(0, 10);
        grid(true);
        
        savefig("output/advanced_03_stacked_fill.svg");
        clf();
        std::cout << "   Saved: output/advanced_03_stacked_fill.svg\n";
    }
    
    // Example 4: Text Annotations
    std::cout << "4. Text Annotations...\n";
    {
        figure(800, 600);
        
        auto x = linspace(0, 2 * M_PI, 100);
        std::vector<double> y;
        for (double xi : x) y.push_back(std::sin(xi));
        
        plot(x, y, "b-", opts({{"linewidth", "2"}}));
        
        // Add annotations at key points
        text(M_PI/2, 1.0, "Maximum", opts({{"ha", "center"}, {"va", "bottom"}, {"fontsize", "12"}}));
        text(3*M_PI/2, -1.0, "Minimum", opts({{"ha", "center"}, {"va", "top"}, {"fontsize", "12"}}));
        text(M_PI, 0.0, "Zero crossing", opts({{"ha", "left"}, {"va", "bottom"}, {"fontsize", "10"}}));
        
        // Add horizontal reference lines
        axhline(0, opts({{"color", "gray"}, {"linewidth", "0.5"}}));
        axhline(1, opts({{"color", "red"}, {"linewidth", "0.5"}, {"linestyle", "--"}}));
        axhline(-1, opts({{"color", "red"}, {"linewidth", "0.5"}, {"linestyle", "--"}}));
        
        // Add vertical reference lines
        axvline(M_PI/2, opts({{"color", "green"}, {"linewidth", "0.5"}}));
        axvline(3*M_PI/2, opts({{"color", "green"}, {"linewidth", "0.5"}}));
        
        title("Sine Wave with Annotations");
        xlabel("x (radians)");
        ylabel("sin(x)");
        grid(true);
        
        savefig("output/advanced_04_annotations.svg");
        clf();
        std::cout << "   Saved: output/advanced_04_annotations.svg\n";
    }
    
    // Example 5: Scientific Plot with Multiple Features
    std::cout << "5. Scientific Plot (Combined Features)...\n";
    {
        figure(900, 700);
        
        // Simulated experimental data
        std::vector<double> x_data = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0};
        std::vector<double> y_data = {0.8, 2.1, 4.2, 8.5, 16.1, 32.8, 63.5, 128.2, 254.9, 512.3};
        std::vector<double> y_err;
        for (double y : y_data) y_err.push_back(y * 0.1);  // 10% error
        
        // Theoretical model
        auto x_theory = linspace(0, 5.5, 100);
        std::vector<double> y_theory, y_theory_upper, y_theory_lower;
        for (double xi : x_theory) {
            double val = std::pow(2, xi);
            y_theory.push_back(val);
            y_theory_upper.push_back(val * 1.1);
            y_theory_lower.push_back(val * 0.9);
        }
        
        // Plot theory with uncertainty band
        fill_between(x_theory, y_theory_lower, y_theory_upper, 
                     opts({{"color", "red"}, {"alpha", "0.2"}, {"label", "Model ±10%"}}));
        plot(x_theory, y_theory, "r-", opts({{"linewidth", "2"}, {"label", "Model: y = 2^x"}}));
        
        // Plot experimental data with error bars
        errorbar(x_data, y_data, y_err, 
                 opts({{"color", "blue"}, {"capsize", "4"}, {"label", "Experiment"}}));
        
        // Add annotation
        text(4.0, 200, "Exponential growth\nregion", opts({{"fontsize", "10"}, {"ha", "center"}}));
        
        title("Exponential Growth: Experiment vs. Model", opts({{"fontsize", "14"}}));
        xlabel("Time (hours)", opts({{"fontsize", "12"}}));
        ylabel("Population (×10³)", opts({{"fontsize", "12"}}));
        legend(true);
        grid(true);
        
        savefig("output/advanced_05_scientific.svg");
        clf();
        std::cout << "   Saved: output/advanced_05_scientific.svg\n";
    }
    
    // Example 6: Multi-panel Scientific Figure
    std::cout << "6. Multi-panel Scientific Figure...\n";
    {
        figure(1000, 800);
        layout(2, 2);
        
        auto x = linspace(0, 10, 100);
        
        // Panel A: Raw data with fill
        subplot(2, 2, 1);
        std::vector<double> signal, noise_upper, noise_lower;
        for (double xi : x) {
            double s = std::sin(xi);
            signal.push_back(s);
            noise_upper.push_back(s + 0.3);
            noise_lower.push_back(s - 0.3);
        }
        fill_between(x, noise_lower, noise_upper, opts({{"color", "gray"}, {"alpha", "0.3"}}));
        plot(x, signal, "b-", opts({{"linewidth", "2"}}));
        title("(A) Raw Signal");
        xlabel("Time");
        ylabel("Amplitude");
        grid(true);
        
        // Panel B: Scatter with trend line and error bars
        subplot(2, 2, 2);
        std::vector<double> x_pts = {1, 2, 3, 4, 5, 6, 7, 8, 9};
        std::vector<double> y_pts = {2.1, 4.3, 5.8, 7.9, 10.2, 12.1, 14.5, 16.2, 18.3};
        std::vector<double> y_err(9, 0.8);
        
        errorbar(x_pts, y_pts, y_err, opts({{"color", "red"}, {"capsize", "3"}}));
        
        auto trend_x = linspace(0, 10, 50);
        std::vector<double> trend_y;
        for (double xi : trend_x) trend_y.push_back(2 * xi);
        plot(trend_x, trend_y, "k--", opts({{"linewidth", "1"}}));
        
        title("(B) Linear Regression");
        xlabel("X Variable");
        ylabel("Y Variable");
        text(7, 5, "R² = 0.98", opts({{"fontsize", "10"}}));
        grid(true);
        
        // Panel C: Comparison with reference lines
        subplot(2, 2, 3);
        std::vector<double> model1, model2;
        for (double xi : x) {
            model1.push_back(std::exp(-xi * 0.3) * std::cos(xi));
            model2.push_back(std::exp(-xi * 0.5) * std::cos(xi));
        }
        plot(x, model1, "b-", opts({{"linewidth", "2"}, {"label", "Model A"}}));
        plot(x, model2, "r-", opts({{"linewidth", "2"}, {"label", "Model B"}}));
        axhline(0, opts({{"color", "gray"}, {"linewidth", "0.5"}}));
        title("(C) Damped Oscillations");
        xlabel("Time");
        ylabel("Displacement");
        legend(true);
        grid(true);
        
        // Panel D: Histogram with annotations
        subplot(2, 2, 4);
        auto data = randn(500, 0, 1);
        hist(data, 20, opts({{"color", "steelblue"}}));
        axvline(0, opts({{"color", "red"}, {"linewidth", "2"}}));
        text(0.1, 60, "μ = 0", opts({{"fontsize", "10"}, {"color", "red"}}));
        title("(D) Distribution");
        xlabel("Value");
        ylabel("Frequency");
        
        suptitle("Comprehensive Analysis Results");
        savefig("output/advanced_06_multipanel.svg");
        clf();
        std::cout << "   Saved: output/advanced_06_multipanel.svg\n";
    }
    
    // Example 7: Financial-style Chart
    std::cout << "7. Financial-style Chart...\n";
    {
        figure(900, 600);
        
        // Simulated stock price
        auto days = linspace(0, 100, 100);
        std::vector<double> price, ma_20, band_upper, band_lower;
        
        double p = 100;
        for (size_t i = 0; i < days.size(); ++i) {
            p += (randn(1, 0, 2)[0]);
            if (p < 50) p = 50;
            price.push_back(p);
        }
        
        // Moving average and bands
        for (size_t i = 0; i < days.size(); ++i) {
            double sum = 0;
            int count = 0;
            for (int j = std::max(0, (int)i - 20); j <= (int)i; ++j) {
                sum += price[j];
                count++;
            }
            double ma = sum / count;
            ma_20.push_back(ma);
            band_upper.push_back(ma + 10);
            band_lower.push_back(ma - 10);
        }
        
        // Plot Bollinger-style bands
        fill_between(days, band_lower, band_upper, 
                     opts({{"color", "yellow"}, {"alpha", "0.3"}}));
        plot(days, price, "b-", opts({{"linewidth", "1"}, {"label", "Price"}}));
        plot(days, ma_20, "r-", opts({{"linewidth", "2"}, {"label", "20-day MA"}}));
        
        // Add buy/sell signals as text
        text(25, price[25] + 5, "BUY", opts({{"color", "green"}, {"fontsize", "10"}, {"fontweight", "bold"}}));
        text(60, price[60] - 5, "SELL", opts({{"color", "red"}, {"fontsize", "10"}, {"fontweight", "bold"}}));
        
        title("Stock Price with Moving Average Bands");
        xlabel("Days");
        ylabel("Price ($)");
        legend(true);
        grid(true);
        
        savefig("output/advanced_07_financial.svg");
        clf();
        std::cout << "   Saved: output/advanced_07_financial.svg\n";
    }
    
    std::cout << "\n===============================\n";
    std::cout << "Advanced demos completed!\n";
    std::cout << "\nNew functions demonstrated:\n";
    std::cout << "  errorbar(x, y, yerr)           - Error bars\n";
    std::cout << "  errorbar(x, y, xerr, yerr)     - X and Y error bars\n";
    std::cout << "  fill_between(x, y1, y2)        - Fill area between curves\n";
    std::cout << "  fill_between(x, y, baseline)   - Fill to baseline\n";
    std::cout << "  text(x, y, \"text\")             - Text annotation\n";
    std::cout << "  annotate(text, x, y, tx, ty)   - Annotation with position\n";
    std::cout << "  axhline(y)                     - Horizontal line\n";
    std::cout << "  axvline(x)                     - Vertical line\n";
    std::cout << "  xscale(\"log\"), yscale(\"log\")   - Log scale axes\n";
    
    return 0;
}
