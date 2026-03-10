/**
 * @file layout_demo.cpp
 * @brief Demo tính năng layout/subplot linh hoạt như Julia Plots
 *
 * Compile: g++ -std=c++14 -D_USE_MATH_DEFINES -I../include layout_demo.cpp -o
 * layout_demo run: ./layout_demo
 */

#include <cmath>
#include <cppplot/cppplot.hpp>
#include <iostream>

using namespace cppplot;

int main() {
  std::cout << "CppPlot Layout & Subplot Demo\n";
  std::cout << "==============================\n\n";

  // Example 1: Basic grid layout (2x2)
  std::cout << "1. Basic 2x2 grid layout...\n";
  {
    auto x = linspace(0, 2 * M_PI, 100);

    figure(800, 600);

    // Top-left
    subplot(2, 2, 1);
    std::vector<double> y1;
    for (double xi : x)
      y1.push_back(std::sin(xi));
    plot(x, y1, "b-");
    title("sin(x)");

    // Top-right
    subplot(2, 2, 2);
    std::vector<double> y2;
    for (double xi : x)
      y2.push_back(std::cos(xi));
    plot(x, y2, "r-");
    title("cos(x)");

    // Bottom-left
    subplot(2, 2, 3);
    std::vector<double> y3;
    for (double xi : x)
      y3.push_back(std::tan(xi));
    plot(x, y3, "g-");
    title("tan(x)");
    ylim(-5, 5);

    // Bottom-right
    subplot(2, 2, 4);
    std::vector<double> y4;
    for (double xi : x)
      y4.push_back(std::sin(xi) * std::cos(xi));
    plot(x, y4, "m-");
    title("sin(x)*cos(x)");

    suptitle("Basic 2x2 Grid Layout");
    savefig("output/layout_01_basic_2x2.svg");
    clf();

    std::cout << "   Saved: output/layout_01_basic_2x2.svg\n";
  }

  // Example 2: Subplot spanning multiple cells (Julia-style)
  std::cout << "2. Subplot spanning multiple cells...\n";
  {
    auto x = linspace(0, 10, 100);

    figure(900, 600);

    // Top row: one large subplot spanning 2 cells
    subplot(2, 2, {1, 2}); // Span cells 1 and 2
    std::vector<double> y1;
    for (double xi : x)
      y1.push_back(std::sin(xi) + std::sin(2 * xi) / 2);
    plot(x, y1, "b-", opts({{"linewidth", "2"}, {"label", "Combined wave"}}));
    title("Wide Plot Spanning 2 Columns", opts({{"fontsize", "14"}}));
    legend(true);
    grid(true);

    // Bottom-left
    subplot(2, 2, 3);
    std::vector<double> y2;
    for (double xi : x)
      y2.push_back(std::sin(xi));
    plot(x, y2, "r-");
    title("sin(x)");

    // Bottom-right
    subplot(2, 2, 4);
    std::vector<double> y3;
    for (double xi : x)
      y3.push_back(std::sin(2 * xi) / 2);
    plot(x, y3, "g-");
    title("sin(2x)/2");

    suptitle("Subplot Spanning Multiple Cells");
    savefig("output/layout_02_span_cells.svg");
    clf();

    std::cout << "   Saved: output/layout_02_span_cells.svg\n";
  }

  // Example 3: Custom column widths (Julia-style)
  std::cout << "3. Custom column widths...\n";
  {
    figure(900, 500);

    // Set layout with custom widths: 2 rows, 3 cols with widths 1:2:1
    layout(2, 3, {1, 2, 1});

    auto x = linspace(0, 5, 50);

    // Row 1
    subplot(2, 3, 1);
    std::vector<double> y1;
    for (double xi : x)
      y1.push_back(xi);
    plot(x, y1, "b-");
    title("Small");

    subplot(2, 3, 2);
    std::vector<double> y2;
    for (double xi : x)
      y2.push_back(xi * xi);
    plot(x, y2, "r-");
    title("Wide (2x)");

    subplot(2, 3, 3);
    std::vector<double> y3;
    for (double xi : x)
      y3.push_back(std::sqrt(xi));
    plot(x, y3, "g-");
    title("Small");

    // Row 2
    subplot(2, 3, 4);
    std::vector<double> y4;
    for (double xi : x)
      y4.push_back(std::log(xi + 1));
    plot(x, y4, "m-");
    title("Small");

    subplot(2, 3, 5);
    std::vector<double> y5;
    for (double xi : x)
      y5.push_back(std::exp(xi / 5));
    plot(x, y5, "c-");
    title("Wide (2x)");

    subplot(2, 3, 6);
    std::vector<double> y6;
    for (double xi : x)
      y6.push_back(1.0 / (xi + 1));
    plot(x, y6, "y-");
    title("Small");

    suptitle("Custom Column Widths (1:2:1)");
    savefig("output/layout_03_custom_widths.svg");
    clf();

    std::cout << "   Saved: output/layout_03_custom_widths.svg\n";
  }

  // Example 4: Custom row heights and column widths
  std::cout << "4. Custom row heights and column widths...\n";
  {
    figure(800, 700);

    // Layout: 2 rows x 2 cols, widths 2:1, heights 1:2
    layout(2, 2, {2, 1}, {1, 2});

    auto x = linspace(0, 4 * M_PI, 200);

    subplot(2, 2, 1); // Wide, short (top-left)
    std::vector<double> y1;
    for (double xi : x)
      y1.push_back(std::sin(xi));
    plot(x, y1, "b-", opts({{"linewidth", "2"}}));
    title("Wide, Short");
    grid(true);

    subplot(2, 2, 2); // Narrow, short (top-right)
    auto hist_data = randn(200, 0, 1);
    hist(hist_data, 15, opts({{"color", "orange"}}));
    title("Narrow, Short");

    subplot(2, 2, 3); // Wide, tall (bottom-left)
    auto x2 = linspace(0, 10, 100);
    std::vector<double> y3, y3b;
    for (double xi : x2) {
      y3.push_back(xi * xi);
      y3b.push_back(xi * xi * 0.8);
    }
    plot(x2, y3, "r-", opts({{"label", "y = x²"}}));
    plot(x2, y3b, "r--", opts({{"label", "y = 0.8x²"}}));
    title("Wide, Tall");
    legend(true);
    grid(true);

    subplot(2, 2, 4); // Narrow, tall (bottom-right)
    std::vector<double> categories = {1, 2, 3, 4, 5};
    std::vector<double> values = {30, 45, 28, 60, 35};
    bar(categories, values, opts({{"color", "steelblue"}}));
    title("Narrow, Tall");

    suptitle("Custom Widths (2:1) and Heights (1:2)");
    savefig("output/layout_04_custom_both.svg");
    clf();

    std::cout << "   Saved: output/layout_04_custom_both.svg\n";
  }

  // Example 5: Complex layout with spanning
  std::cout << "5. Complex layout with spanning...\n";
  {
    figure(1000, 800);

    // Use GridSpec for fine control
    GridSpec gs(3, 3);
    gs.setWidthRatios({1, 2, 1});
    gs.setHeightRatios({1, 1, 1});
    gs.setSpacing(0.15, 0.15);
    gcf().setLayout(gs);

    auto x = linspace(0, 10, 100);

    // Top row: one large plot spanning all 3 columns
    subplot(3, 3, {1, 2, 3});
    std::vector<double> main_y;
    for (double xi : x)
      main_y.push_back(std::sin(xi) * std::exp(-xi / 10));
    plot(x, main_y, "b-",
         opts({{"linewidth", "2.5"}, {"label", "Damped sine"}}));
    title("Main Plot (spans top row)", opts({{"fontsize", "14"}}));
    legend(true);
    grid(true);

    // Middle row
    subplot(3, 3, 4);
    std::vector<double> y4;
    for (double xi : x)
      y4.push_back(std::sin(xi));
    plot(x, y4, "r-");
    title("Detail 1");

    subplot(3, 3, 5); // This is the middle (wider)
    auto scatter_x = random(30, 0, 10);
    auto scatter_y = random(30, 0, 10);
    scatter(scatter_x, scatter_y, opts({{"c", "green"}, {"s", "50"}}));
    title("Scatter (wider)");

    subplot(3, 3, 6);
    std::vector<double> y6;
    for (double xi : x)
      y6.push_back(std::cos(xi));
    plot(x, y6, "m-");
    title("Detail 2");

    // Bottom row: left side spans 2 cells, right side is 1 cell
    subplot(3, 3, {7, 8}); // Spans bottom-left 2 cells
    auto hist_data = randn(500, 0, 1);
    hist(hist_data, 30, opts({{"color", "coral"}}));
    title("Histogram (spans 2 cells)");
    xlabel("Value");
    ylabel("Frequency");

    subplot(3, 3, 9);
    std::vector<double> bar_x = {1, 2, 3};
    std::vector<double> bar_h = {40, 65, 50};
    bar(bar_x, bar_h, opts({{"color", "teal"}}));
    title("Bar Chart");

    suptitle("Complex Layout with Spanning");
    savefig("output/layout_05_complex.svg");
    clf();

    std::cout << "   Saved: output/layout_05_complex.svg\n";
  }

  // Example 6: Using subplot_span for precise control
  std::cout << "6. Using subplot_span for precise control...\n";
  {
    figure(900, 700);

    // Set up a 3x3 grid
    layout(3, 3);

    auto x = linspace(0, 2 * M_PI, 100);

    // Large plot spanning rows 0-1, cols 0-1 (top-left quadrant)
    subplot_span(0, 0, 1, 1);
    std::vector<double> y1;
    for (double xi : x)
      y1.push_back(std::sin(xi) + std::cos(2 * xi));
    plot(x, y1, "b-", opts({{"linewidth", "2"}}));
    title("Large (2x2)");
    grid(true);

    // Right column plots
    subplot_span(0, 2, 0, 2); // Top-right
    auto d1 = randn(100, 0, 1);
    hist(d1, 15, opts({{"color", "red"}}));
    title("Hist 1");

    subplot_span(1, 2, 1, 2); // Middle-right
    auto d2 = randn(100, 1, 0.5);
    hist(d2, 15, opts({{"color", "green"}}));
    title("Hist 2");

    // Bottom row
    subplot_span(2, 0, 2, 0);
    std::vector<double> b1 = {1, 2, 3};
    std::vector<double> h1 = {10, 20, 15};
    bar(b1, h1, opts({{"color", "orange"}}));
    title("Bar 1");

    subplot_span(2, 1, 2, 1);
    std::vector<double> b2 = {1, 2, 3};
    std::vector<double> h2 = {25, 15, 30};
    bar(b2, h2, opts({{"color", "purple"}}));
    title("Bar 2");

    subplot_span(2, 2, 2, 2);
    std::vector<double> b3 = {1, 2, 3};
    std::vector<double> h3 = {20, 25, 20};
    bar(b3, h3, opts({{"color", "cyan"}}));
    title("Bar 3");

    suptitle("Precise Control with subplot_span()");
    savefig("output/layout_06_subplot_span.svg");
    clf();

    std::cout << "   Saved: output/layout_06_subplot_span.svg\n";
  }

  // Example 7: Dashboard-style layout
  std::cout << "7. Dashboard-style layout...\n";
  {
    figure(1200, 800);

    // Dashboard: 3 rows x 4 cols with custom sizes
    GridSpec gs(3, 4);
    gs.setWidthRatios({1, 1, 1, 1});
    gs.setHeightRatios({1, 2, 1});
    gs.setMargins(0.08, 0.95, 0.08, 0.92);
    gcf().setLayout(gs);

    // Top row: 4 small KPI-style plots
    auto kpi_x = linspace(0, 10, 20);
    std::vector<std::string> colors = {"steelblue", "coral", "seagreen",
                                       "mediumpurple"};

    for (int i = 1; i <= 4; ++i) {
      subplot(3, 4, i);
      std::vector<double> kpi_y;
      for (double xi : kpi_x)
        kpi_y.push_back(10 + std::sin(xi + i) * 5 + xi * 0.5);
      plot(kpi_x, kpi_y, "-",
           opts({{"color", colors[i - 1]}, {"linewidth", "2"}}));
      title("KPI " + std::to_string(i), opts({{"fontsize", "11"}}));
      grid(true);
    }

    // Middle row: main chart spanning all 4 columns
    subplot(3, 4, {5, 6, 7, 8});
    auto main_x = linspace(0, 100, 200);
    std::vector<double> main_y, trend;
    for (double xi : main_x) {
      main_y.push_back(50 + 30 * std::sin(xi / 10) + 10 * std::sin(xi / 3));
      trend.push_back(50 + xi * 0.2);
    }
    plot(
        main_x, main_y, "-",
        opts(
            {{"color", "royalblue"}, {"linewidth", "1.5"}, {"label", "Data"}}));
    plot(main_x, trend, "--",
         opts({{"color", "red"}, {"linewidth", "2"}, {"label", "Trend"}}));
    title("Main Chart - Time Series Analysis", opts({{"fontsize", "14"}}));
    xlabel("Time");
    ylabel("Value");
    legend(true);
    grid(true);

    // Bottom row: 4 detail charts
    subplot(3, 4, 9);
    auto pie_vals = std::vector<double>{1, 2, 3, 4};
    bar(pie_vals, std::vector<double>{25, 30, 20, 25},
        opts({{"color", "gold"}}));
    title("Category A");

    subplot(3, 4, 10);
    bar(pie_vals, std::vector<double>{35, 20, 25, 20},
        opts({{"color", "lightcoral"}}));
    title("Category B");

    subplot(3, 4, 11);
    bar(pie_vals, std::vector<double>{20, 35, 30, 15},
        opts({{"color", "lightgreen"}}));
    title("Category C");

    subplot(3, 4, 12);
    bar(pie_vals, std::vector<double>{30, 25, 20, 25},
        opts({{"color", "lightskyblue"}}));
    title("Category D");

    suptitle("Dashboard Layout");
    savefig("output/layout_07_dashboard.svg");
    clf();

    std::cout << "   Saved: output/layout_07_dashboard.svg\n";
  }

  // Example 8: Inset axes
  std::cout << "8. Inset axes (plot within plot)...\n";
  {
    figure(800, 600);

    auto x = linspace(0, 10, 200);
    std::vector<double> y, y_detail;
    for (double xi : x) {
      y.push_back(std::sin(xi) * std::exp(-xi / 5));
    }

    // Main plot
    subplot(1, 1, 1);
    plot(x, y, "b-",
         opts({{"linewidth", "2"}, {"label", "Damped oscillation"}}));
    title("Main Plot with Inset");
    xlabel("Time");
    ylabel("Amplitude");
    legend(true);
    grid(true);

    // Add inset axes (zoomed view)
    inset_axes(0.55, 0.55, 0.4, 0.35);
    auto x_zoom = linspace(0, 2, 50);
    std::vector<double> y_zoom;
    for (double xi : x_zoom)
      y_zoom.push_back(std::sin(xi) * std::exp(-xi / 5));
    plot(x_zoom, y_zoom, "r-", opts({{"linewidth", "2"}}));
    title("Zoomed: t=0-2", opts({{"fontsize", "10"}}));
    grid(true);

    savefig("output/layout_08_inset.svg");
    clf();

    std::cout << "   Saved: output/layout_08_inset.svg\n";
  }

  // Example 9: Irregular grid (L-shaped layout)
  std::cout << "9. Irregular grid (L-shaped layout)...\n";
  {
    figure(900, 700);

    layout(2, 2);

    auto x = linspace(0, 5, 100);

    // Large L-shape: top-left spanning down
    subplot(2, 2, {1, 3}); // Vertical span
    std::vector<double> y1;
    for (double xi : x)
      y1.push_back(xi * xi * xi);
    plot(x, y1, "b-", opts({{"linewidth", "2.5"}}));
    title("Tall Left Plot", opts({{"fontsize", "12"}}));
    xlabel("x");
    ylabel("y = x³");
    grid(true);

    // Top-right
    subplot(2, 2, 2);
    std::vector<double> y2;
    for (double xi : x)
      y2.push_back(std::sin(xi * 2));
    plot(x, y2, "r-", opts({{"linewidth", "2"}}));
    title("Top Right");

    // Bottom-right
    subplot(2, 2, 4);
    std::vector<double> y3;
    for (double xi : x)
      y3.push_back(std::cos(xi * 2));
    plot(x, y3, "g-", opts({{"linewidth", "2"}}));
    title("Bottom Right");

    suptitle("L-shaped Layout (Vertical Span)");
    savefig("output/layout_09_irregular.svg");
    clf();

    std::cout << "   Saved: output/layout_09_irregular.svg\n";
  }

  // Example 10: Mixed figure with add_axes
  std::cout << "10. Free-form layout with add_axes()...\n";
  {
    figure(1000, 700);

    auto x = linspace(0, 2 * M_PI, 100);

    // Main large plot
    add_axes(0.1, 0.3, 0.55, 0.6);
    std::vector<double> y1;
    for (double xi : x)
      y1.push_back(std::sin(xi));
    plot(x, y1, "b-", opts({{"linewidth", "2.5"}, {"label", "sin(x)"}}));
    title("Main Plot");
    xlabel("x");
    ylabel("y");
    legend(true);
    grid(true);

    // Side panel plot
    add_axes(0.7, 0.3, 0.25, 0.6);
    auto data = randn(200, 0, 1);
    hist(data, 20, opts({{"color", "coral"}}));
    title("Side Panel");

    // Bottom strip
    add_axes(0.1, 0.08, 0.85, 0.15);
    auto x2 = linspace(0, 50, 200);
    std::vector<double> y2;
    for (double xi : x2)
      y2.push_back(std::sin(xi) + 0.5 * std::sin(3 * xi));
    plot(x2, y2, "g-", opts({{"linewidth", "1"}}));
    title("Bottom Strip");

    suptitle("Free-form Layout with add_axes()");
    savefig("output/layout_10_freeform.svg");
    clf();

    std::cout << "   Saved: output/layout_10_freeform.svg\n";
  }

  std::cout << "\n==============================\n";
  std::cout << "All layout demos completed!\n";
  std::cout << "Check 'output' folder for SVG files.\n";
  std::cout << "\nLayout functions available:\n";
  std::cout << "\n  Basic subplot:\n";
  std::cout << "    subplot(rows, cols, index)       - Standard subplot\n";
  std::cout << "    subplot(rows, cols, {indices})   - Span multiple cells "
               "(Julia-style)\n";
  std::cout << "    subplot_at(row, col)             - Position by row/col "
               "(0-based)\n";
  std::cout << "    subplot_span(r1,c1, r2,c2)       - Span from (r1,c1) to "
               "(r2,c2)\n";
  std::cout << "\n  Layout configuration:\n";
  std::cout << "    layout(rows, cols)               - Simple grid\n";
  std::cout << "    layout(rows, cols, widths)       - Custom column widths\n";
  std::cout << "    layout(rows, cols, widths, heights) - Custom both\n";
  std::cout << "    gcf().setLayout(GridSpec(...))   - Full GridSpec control\n";
  std::cout << "\n  Free positioning:\n";
  std::cout
      << "    add_axes(x, y, w, h)             - Position anywhere (0-1)\n";
  std::cout
      << "    inset_axes(x, y, w, h)           - Inset within current axes\n";
  std::cout << "\n  GridSpec class:\n";
  std::cout << "    GridSpec(rows, cols)\n";
  std::cout << "      .setWidthRatios({...})\n";
  std::cout << "      .setHeightRatios({...})\n";
  std::cout << "      .setSpacing(wspace, hspace)\n";
  std::cout << "      .setMargins(left, right, bottom, top)\n";

  return 0;
}
