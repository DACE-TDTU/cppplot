/**
 * @file test_svg.cpp
 * @brief Tests for SVG backend
 */

#include <cppplot/cppplot.hpp>
#include <iostream>
#include <cassert>
#include <fstream>

using namespace cppplot;

// Simple test framework
int tests_passed = 0;
int tests_failed = 0;

#define TEST(name) void test_##name()
#define RUN_TEST(name) do { \
    std::cout << "Running " #name "... "; \
    try { \
        test_##name(); \
        std::cout << "PASSED\n"; \
        tests_passed++; \
    } catch (const std::exception& e) { \
        std::cout << "FAILED: " << e.what() << "\n"; \
        tests_failed++; \
    } \
} while(0)

#define ASSERT_TRUE(x) do { \
    if (!(x)) throw std::runtime_error("Assertion failed: " #x); \
} while(0)

#define ASSERT_CONTAINS(str, substr) do { \
    if ((str).find(substr) == std::string::npos) \
        throw std::runtime_error("Assertion failed: string doesn't contain " #substr); \
} while(0)

// ============ SVG Backend Tests ============

TEST(svg_basic) {
    SVGBackend svg(400, 300);
    svg.clear(Color::white());
    
    std::string output = svg.render();
    ASSERT_CONTAINS(output, "<?xml");
    ASSERT_CONTAINS(output, "<svg");
    ASSERT_CONTAINS(output, "width=\"400\"");
    ASSERT_CONTAINS(output, "height=\"300\"");
}

TEST(svg_line) {
    SVGBackend svg(400, 300);
    svg.clear(Color::white());
    svg.drawLine(10, 20, 100, 200, LineStyle("-", 2, Color::red()));
    
    std::string output = svg.render();
    ASSERT_CONTAINS(output, "<line");
    ASSERT_CONTAINS(output, "x1=\"10\"");
    ASSERT_CONTAINS(output, "y1=\"20\"");
    ASSERT_CONTAINS(output, "x2=\"100\"");
    ASSERT_CONTAINS(output, "y2=\"200\"");
}

TEST(svg_rect) {
    SVGBackend svg(400, 300);
    svg.clear(Color::white());
    svg.drawRect(50, 50, 100, 80, Color::blue(), LineStyle("-", 1, Color::black()));
    
    std::string output = svg.render();
    ASSERT_CONTAINS(output, "<rect");
    ASSERT_CONTAINS(output, "width=\"100\"");
    ASSERT_CONTAINS(output, "height=\"80\"");
}

TEST(svg_circle) {
    SVGBackend svg(400, 300);
    svg.clear(Color::white());
    svg.drawCircle(200, 150, 50, Color::green(), LineStyle("-", 1, Color::black()));
    
    std::string output = svg.render();
    ASSERT_CONTAINS(output, "<circle");
    ASSERT_CONTAINS(output, "cx=\"200\"");
    ASSERT_CONTAINS(output, "cy=\"150\"");
    ASSERT_CONTAINS(output, "r=\"50\"");
}

TEST(svg_polyline) {
    SVGBackend svg(400, 300);
    svg.clear(Color::white());
    
    std::vector<Point> points = {{10, 10}, {100, 50}, {200, 30}, {300, 100}};
    svg.drawPolyline(points, LineStyle("-", 2, Color::blue()));
    
    std::string output = svg.render();
    ASSERT_CONTAINS(output, "<polyline");
    ASSERT_CONTAINS(output, "points=");
}

TEST(svg_text) {
    SVGBackend svg(400, 300);
    svg.clear(Color::white());
    
    TextStyle style;
    style.fontSize = 14;
    style.color = Color::black();
    svg.drawText(100, 100, "Hello World", style);
    
    std::string output = svg.render();
    ASSERT_CONTAINS(output, "<text");
    ASSERT_CONTAINS(output, "Hello World");
}

TEST(svg_polygon) {
    SVGBackend svg(400, 300);
    svg.clear(Color::white());
    
    std::vector<Point> points = {{100, 50}, {150, 150}, {50, 150}};
    svg.drawPolygon(points, Color::yellow(), LineStyle("-", 1, Color::black()));
    
    std::string output = svg.render();
    ASSERT_CONTAINS(output, "<polygon");
}

TEST(svg_clip) {
    SVGBackend svg(400, 300);
    svg.clear(Color::white());
    
    svg.setClipRect(Rect(50, 50, 200, 150));
    svg.drawCircle(100, 100, 100, Color::blue());
    svg.clearClip();
    
    std::string output = svg.render();
    ASSERT_CONTAINS(output, "<clipPath");
    ASSERT_CONTAINS(output, "clip-path=");
}

// ============ Figure Tests ============

TEST(figure_basic) {
    Figure fig(800, 600);
    auto& ax = fig.gca();
    
    std::vector<double> x = {1, 2, 3, 4, 5};
    std::vector<double> y = {1, 4, 9, 16, 25};
    ax.plot(x, y, "b-");
    
    std::string svg = fig.toSVG();
    ASSERT_CONTAINS(svg, "<svg");
    ASSERT_CONTAINS(svg, "width=\"800\"");
}

TEST(figure_subplot) {
    Figure fig(800, 600);
    
    auto& ax1 = fig.subplot(2, 2, 1);
    auto& ax2 = fig.subplot(2, 2, 2);
    auto& ax3 = fig.subplot(2, 2, 3);
    auto& ax4 = fig.subplot(2, 2, 4);
    
    ASSERT_TRUE(fig.getAxes().size() == 4);
}

TEST(figure_save) {
    Figure fig(400, 300);
    auto& ax = fig.gca();
    ax.plot({1, 2, 3}, {1, 4, 9}, "r-");
    ax.set_title("Test");
    
    std::string filename = "test_output.svg";
    fig.savefig(filename);
    
    // Check file exists
    std::ifstream file(filename);
    ASSERT_TRUE(file.good());
    
    // Clean up
    file.close();
    std::remove(filename.c_str());
}

// ============ Axes Tests ============

TEST(axes_plot) {
    Axes ax;
    std::vector<double> x = {0, 1, 2, 3};
    std::vector<double> y = {0, 1, 4, 9};
    
    ax.plot(x, y, "b-o");
    ax.set_xlabel("X");
    ax.set_ylabel("Y");
    ax.set_title("Test Plot");
    
    // Just check it doesn't throw
    ASSERT_TRUE(true);
}

TEST(axes_scatter) {
    Axes ax;
    std::vector<double> x = {1, 2, 3, 4, 5};
    std::vector<double> y = {2, 4, 1, 5, 3};
    
    ax.scatter(x, y, {{"c", std::string("red")}});
    
    ASSERT_TRUE(true);
}

TEST(axes_bar) {
    Axes ax;
    std::vector<double> x = {1, 2, 3};
    std::vector<double> heights = {10, 20, 15};
    
    ax.bar(x, heights);
    
    ASSERT_TRUE(true);
}

TEST(axes_hist) {
    Axes ax;
    std::vector<double> data = {1, 2, 2, 3, 3, 3, 4, 4, 5};
    
    ax.hist(data, 5);
    
    ASSERT_TRUE(true);
}

TEST(axes_limits) {
    Axes ax;
    ax.set_xlim(0, 10);
    ax.set_ylim(-5, 5);
    
    ASSERT_TRUE(true);
}

TEST(axes_grid) {
    Axes ax;
    ax.grid(true);
    
    ASSERT_TRUE(true);
}

// ============ Integration Tests ============

TEST(integration_complete_plot) {
    // Create a complete plot with all elements
    Figure fig(800, 600);
    auto& ax = fig.gca();
    
    // Generate data
    auto x = linspace(0, 10, 50);
    std::vector<double> y1, y2;
    for (double xi : x) {
        y1.push_back(std::sin(xi));
        y2.push_back(std::cos(xi));
    }
    
    // Plot
    ax.plot(x, y1, "b-", {{"label", std::string("sin")}});
    ax.plot(x, y2, "r--", {{"label", std::string("cos")}});
    
    // Labels
    ax.set_xlabel("X axis");
    ax.set_ylabel("Y axis");
    ax.set_title("Complete Test Plot");
    
    // Options
    ax.grid(true);
    ax.legend(true);
    ax.set_xlim(0, 10);
    ax.set_ylim(-1.5, 1.5);
    
    // Render
    std::string svg = fig.toSVG();
    
    // Verify output contains expected elements
    ASSERT_CONTAINS(svg, "X axis");
    ASSERT_CONTAINS(svg, "Y axis");
    ASSERT_CONTAINS(svg, "Complete Test Plot");
    ASSERT_CONTAINS(svg, "<polyline");
}

// ============ Main ============

int main() {
    std::cout << "CppPlot SVG Tests\n";
    std::cout << "=================\n\n";
    
    // SVG Backend tests
    RUN_TEST(svg_basic);
    RUN_TEST(svg_line);
    RUN_TEST(svg_rect);
    RUN_TEST(svg_circle);
    RUN_TEST(svg_polyline);
    RUN_TEST(svg_text);
    RUN_TEST(svg_polygon);
    RUN_TEST(svg_clip);
    
    // Figure tests
    RUN_TEST(figure_basic);
    RUN_TEST(figure_subplot);
    RUN_TEST(figure_save);
    
    // Axes tests
    RUN_TEST(axes_plot);
    RUN_TEST(axes_scatter);
    RUN_TEST(axes_bar);
    RUN_TEST(axes_hist);
    RUN_TEST(axes_limits);
    RUN_TEST(axes_grid);
    
    // Integration tests
    RUN_TEST(integration_complete_plot);
    
    std::cout << "\n=================\n";
    std::cout << "Tests passed: " << tests_passed << "\n";
    std::cout << "Tests failed: " << tests_failed << "\n";
    
    return tests_failed > 0 ? 1 : 0;
}
