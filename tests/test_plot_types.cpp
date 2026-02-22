/**
 * test_plot_types.cpp
 * Tests all plot types using CONFIRMED API from test_svg.cpp:
 *   Figure, Axes (via fig.gca()), ax.plot/scatter/bar/hist
 */
#include <cppplot/cppplot.hpp>
#include <iostream>
#include <cassert>
#include <cmath>
#include <fstream>
#include <stdexcept>

using namespace cppplot;

int tests_passed = 0, tests_failed = 0;

#define RUN_TEST(name) do { \
    std::cout << "Running " #name "... "; \
    try { test_##name(); std::cout << "PASSED\n"; tests_passed++; } \
    catch (const std::exception& e) { \
        std::cout << "FAILED: " << e.what() << "\n"; tests_failed++; } \
} while(0)

#define ASSERT_TRUE(x)     do { if(!(x))  throw std::runtime_error("TRUE: "  #x); } while(0)
#define ASSERT_FALSE(x)    do { if( (x))  throw std::runtime_error("FALSE: " #x); } while(0)
#define ASSERT_EQ(a,b)     do { if((a)!=(b)) throw std::runtime_error("EQ: " #a); } while(0)
#define ASSERT_CONTAINS(s,sub) do { \
    if((s).find(sub)==std::string::npos) \
        throw std::runtime_error(std::string("missing: '") + (sub) + "'"); \
} while(0)

// ═══════════════════════════════════════════
// Line plots — ax.plot()
// ═══════════════════════════════════════════

void test_line_basic() {
    Figure fig(600, 400);
    auto& ax = fig.gca();
    ax.plot({1.0,2.0,3.0,4.0,5.0},
            {1.0,4.0,9.0,16.0,25.0}, "b-");
    std::string svg = fig.toSVG();
    ASSERT_FALSE(svg.empty());
    ASSERT_CONTAINS(svg, "<svg");
}

void test_line_multiple_series() {
    Figure fig(600, 400);
    auto& ax = fig.gca();
    auto x = linspace(0.0, 2*M_PI, 50);
    std::vector<double> y_sin, y_cos;
    for (double xi : x) {
        y_sin.push_back(std::sin(xi));
        y_cos.push_back(std::cos(xi));
    }
    ax.plot(x, y_sin, "b-",  {{"label", std::string("sin")}});
    ax.plot(x, y_cos, "r--", {{"label", std::string("cos")}});
    ax.legend(true);

    std::string svg = fig.toSVG();
    ASSERT_FALSE(svg.empty());
    // 2 series → SVG lớn hơn 1 series
    Figure fig2(600, 400);
    auto& ax2 = fig2.gca();
    ax2.plot(x, y_sin, "b-");
    ASSERT_TRUE(svg.size() > fig2.toSVG().size());
}

void test_line_format_strings() {
    std::vector<std::string> fmts = {"b-","r--","g:","k-.","m-o","c-s"};
    for (auto& fmt : fmts) {
        Figure fig(400, 300);
        auto& ax = fig.gca();
        ax.plot({1.0,2.0,3.0}, {1.0,2.0,3.0}, fmt.c_str());
        std::string svg = fig.toSVG();
        ASSERT_FALSE(svg.empty());
    }
}

void test_line_with_labels() {
    Figure fig(600, 400);
    auto& ax = fig.gca();
    ax.plot({0.0,1.0,2.0}, {0.0,1.0,4.0}, "b-");
    ax.set_xlabel("X Axis");
    ax.set_ylabel("Y Axis");
    ax.set_title("My Title");
    ax.grid(true);
    ax.set_xlim(0, 2);
    ax.set_ylim(0, 5);

    std::string svg = fig.toSVG();
    ASSERT_CONTAINS(svg, "X Axis");
    ASSERT_CONTAINS(svg, "Y Axis");
    ASSERT_CONTAINS(svg, "My Title");
}

// ═══════════════════════════════════════════
// Scatter plot — ax.scatter()
// ═══════════════════════════════════════════

void test_scatter_basic() {
    Figure fig(500, 400);
    auto& ax = fig.gca();
    ax.scatter({1.0,2.0,3.0,4.0,5.0},
               {2.0,4.0,1.0,5.0,3.0},
               {{"c", std::string("red")}});
    std::string svg = fig.toSVG();
    ASSERT_FALSE(svg.empty());
}

void test_scatter_different_colors() {
    std::vector<std::string> colors = {"red","blue","green","black"};
    for (auto& col : colors) {
        Figure fig(400, 300);
        auto& ax = fig.gca();
        ax.scatter({1.0,2.0,3.0}, {1.0,2.0,3.0},
                   {{"c", col}});
        ASSERT_FALSE(fig.toSVG().empty());
    }
}

// ═══════════════════════════════════════════
// Bar chart — ax.bar()
// ═══════════════════════════════════════════

void test_bar_basic() {
    Figure fig(500, 400);
    auto& ax = fig.gca();
    ax.bar({1.0,2.0,3.0,4.0},
           {10.0,25.0,15.0,30.0});
    std::string svg = fig.toSVG();
    ASSERT_FALSE(svg.empty());
    // Bar chart → phải có rect elements
    ASSERT_CONTAINS(svg, "<rect");
}

void test_bar_values() {
    Figure fig(500, 400);
    auto& ax = fig.gca();
    // Giá trị âm (below-zero bars)
    ax.bar({1.0,2.0,3.0}, {-5.0,10.0,-3.0});
    ASSERT_FALSE(fig.toSVG().empty());
}

// ═══════════════════════════════════════════
// Histogram — ax.hist()
// ═══════════════════════════════════════════

void test_hist_basic() {
    Figure fig(500, 400);
    auto& ax = fig.gca();
    std::vector<double> data = {1,2,2,3,3,3,4,4,5,5,5,5};
    ax.hist(data, 5);
    std::string svg = fig.toSVG();
    ASSERT_FALSE(svg.empty());
    ASSERT_CONTAINS(svg, "<rect"); // histogram dùng rects
}

void test_hist_many_bins() {
    Figure fig(600, 400);
    auto& ax = fig.gca();
    std::vector<double> data;
    for (int i = 0; i < 100; i++) data.push_back(i * 0.1);
    ax.hist(data, 20);
    ASSERT_FALSE(fig.toSVG().empty());
}

// ═══════════════════════════════════════════
// Subplots — fig.subplot()
// ═══════════════════════════════════════════

void test_subplot_2x1() {
    Figure fig(600, 600);

    auto& ax1 = fig.subplot(2, 1, 1);
    ax1.plot({0.0,1.0,2.0}, {0.0,1.0,0.0}, "b-");
    ax1.set_title("Top");

    auto& ax2 = fig.subplot(2, 1, 2);
    ax2.plot({0.0,1.0,2.0}, {0.0,-1.0,0.0}, "r-");
    ax2.set_title("Bottom");

    ASSERT_TRUE(fig.getAxes().size() == 2);
    ASSERT_FALSE(fig.toSVG().empty());
}

void test_subplot_2x2() {
    Figure fig(800, 600);
    auto& ax1 = fig.subplot(2, 2, 1);
    auto& ax2 = fig.subplot(2, 2, 2);
    auto& ax3 = fig.subplot(2, 2, 3);
    auto& ax4 = fig.subplot(2, 2, 4);

    ax1.plot({1.0,2.0,3.0}, {1.0,4.0,9.0}, "b-");
    ax2.scatter({1.0,2.0,3.0}, {3.0,1.0,2.0}, {{"c", std::string("red")}});
    ax3.bar({1.0,2.0,3.0}, {5.0,10.0,7.0});
    ax4.hist({1.0,2.0,2.0,3.0,3.0,3.0}, 3);

    ASSERT_EQ(fig.getAxes().size(), (size_t)4);
    std::string svg = fig.toSVG();
    ASSERT_FALSE(svg.empty());
}

// ═══════════════════════════════════════════
// Integration — complete realistic plot
// ═══════════════════════════════════════════

void test_integration_sincos() {
    Figure fig(800, 600);
    auto& ax = fig.gca();

    auto x = linspace(0.0, 10.0, 100);
    std::vector<double> y1, y2;
    for (double xi : x) {
        y1.push_back(std::sin(xi));
        y2.push_back(std::cos(xi));
    }

    ax.plot(x, y1, "b-",  {{"label", std::string("sin(x)")}});
    ax.plot(x, y2, "r--", {{"label", std::string("cos(x)")}});
    ax.set_xlabel("X axis");
    ax.set_ylabel("Y axis");
    ax.set_title("Complete Integration Test");
    ax.grid(true);
    ax.legend(true);
    ax.set_xlim(0, 10);
    ax.set_ylim(-1.5, 1.5);

    std::string svg = fig.toSVG();
    ASSERT_CONTAINS(svg, "X axis");
    ASSERT_CONTAINS(svg, "Y axis");
    ASSERT_CONTAINS(svg, "Complete Integration Test");
    ASSERT_CONTAINS(svg, "<polyline");

    // Save và kiểm tra file
    fig.savefig("test_integration.svg");
    std::ifstream f("test_integration.svg");
    ASSERT_TRUE(f.good());
    f.close();
    std::remove("test_integration.svg");
}

// ═══════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════

int main() {
    std::cout << "CppPlot Plot Types Tests\n";
    std::cout << "=========================\n\n";

    RUN_TEST(line_basic);
    RUN_TEST(line_multiple_series);
    RUN_TEST(line_format_strings);
    RUN_TEST(line_with_labels);
    RUN_TEST(scatter_basic);
    RUN_TEST(scatter_different_colors);
    RUN_TEST(bar_basic);
    RUN_TEST(bar_values);
    RUN_TEST(hist_basic);
    RUN_TEST(hist_many_bins);
    RUN_TEST(subplot_2x1);
    RUN_TEST(subplot_2x2);
    RUN_TEST(integration_sincos);

    std::cout << "\n=========================\n";
    std::cout << "Passed: " << tests_passed << "\n";
    std::cout << "Failed: " << tests_failed << "\n";
    return tests_failed > 0 ? 1 : 0;
}
