/**
 * test_layout.cpp
 * Tests layout system using confirmed API:
 *   fig.subplot(rows,cols,idx), fig.getAxes(),
 *   SVGBackend low-level for precise geometry tests.
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

#define ASSERT_TRUE(x)  do { if(!(x))  throw std::runtime_error("TRUE: "  #x); } while(0)
#define ASSERT_EQ(a,b)  do { if((a)!=(b)) throw std::runtime_error("EQ: " #a); } while(0)
#define ASSERT_FALSE(x) do { if( (x))  throw std::runtime_error("FALSE: " #x); } while(0)
#define ASSERT_CONTAINS(s,sub) do { \
    if((s).find(sub)==std::string::npos) \
        throw std::runtime_error(std::string("missing: '") + (sub) + "'"); \
} while(0)

// ═══════════════════════════════════════════
// subplot — axis count và content
// ═══════════════════════════════════════════

void test_subplot_1x1_is_gca() {
    Figure fig(600, 400);
    auto& ax = fig.gca();
    ax.plot({1.0,2.0,3.0}, {1.0,4.0,9.0}, "b-");
    ASSERT_EQ(fig.getAxes().size(), (size_t)1);
    ASSERT_FALSE(fig.toSVG().empty());
}

void test_subplot_2x1_count() {
    Figure fig(600, 600);
    fig.subplot(2, 1, 1);
    fig.subplot(2, 1, 2);
    ASSERT_EQ(fig.getAxes().size(), (size_t)2);
}

void test_subplot_1x2_count() {
    Figure fig(800, 400);
    fig.subplot(1, 2, 1);
    fig.subplot(1, 2, 2);
    ASSERT_EQ(fig.getAxes().size(), (size_t)2);
}

void test_subplot_2x2_count() {
    Figure fig(800, 600);
    fig.subplot(2, 2, 1);
    fig.subplot(2, 2, 2);
    fig.subplot(2, 2, 3);
    fig.subplot(2, 2, 4);
    ASSERT_EQ(fig.getAxes().size(), (size_t)4);
}

void test_subplot_3x1_count() {
    Figure fig(600, 900);
    fig.subplot(3, 1, 1);
    fig.subplot(3, 1, 2);
    fig.subplot(3, 1, 3);
    ASSERT_EQ(fig.getAxes().size(), (size_t)3);
}

void test_subplot_each_has_content() {
    Figure fig(800, 600);
    auto x = linspace(0.0, 2*M_PI, 30);
    std::vector<double> y_sin, y_cos, y_tan, y_sq;
    for (double xi : x) {
        y_sin.push_back(std::sin(xi));
        y_cos.push_back(std::cos(xi));
        y_tan.push_back(std::tanh(xi));
        y_sq.push_back(xi*xi / 40.0);
    }

    auto& ax1 = fig.subplot(2, 2, 1);
    ax1.plot(x, y_sin, "b-"); ax1.set_title("sin");

    auto& ax2 = fig.subplot(2, 2, 2);
    ax2.plot(x, y_cos, "r-"); ax2.set_title("cos");

    auto& ax3 = fig.subplot(2, 2, 3);
    ax3.plot(x, y_tan, "g-"); ax3.set_title("tanh");

    auto& ax4 = fig.subplot(2, 2, 4);
    ax4.plot(x, y_sq, "m-"); ax4.set_title("x^2");

    std::string svg = fig.toSVG();
    ASSERT_CONTAINS(svg, "sin");
    ASSERT_CONTAINS(svg, "cos");
    ASSERT_CONTAINS(svg, "tanh");
}

void test_subplot_svg_larger_than_single() {
    // 2x2 subplot SVG phải lớn hơn single-axes SVG cùng kích thước
    auto x = linspace(0.0, 1.0, 20);
    std::vector<double> y;
    for (double xi : x) y.push_back(xi*xi);

    Figure fig_single(800, 600);
    fig_single.gca().plot(x, y, "b-");
    size_t sz_single = fig_single.toSVG().size();

    Figure fig_multi(800, 600);
    for (int i = 1; i <= 4; i++) {
        auto& ax = fig_multi.subplot(2, 2, i);
        ax.plot(x, y, "b-");
    }
    size_t sz_multi = fig_multi.toSVG().size();

    ASSERT_TRUE(sz_multi > sz_single);
}

// ═══════════════════════════════════════════
// SVGBackend layout — geometry precision
// ═══════════════════════════════════════════

void test_svgbackend_coordinate_mapping() {
    // Dùng CoordinateTransform để verify pixel mapping
    CoordinateTransform t(
        Rect(0, 0, 10, 10),   // data space
        Rect(0, 0, 100, 100)  // pixel space
    );
    Point p = t.dataToPixel(5.0, 5.0);
    ASSERT_TRUE(std::abs(p.x - 50.0) < 2.0);
    ASSERT_TRUE(std::abs(p.y - 50.0) < 2.0);
}

void test_svgbackend_multiple_rects() {
    // Bar chart = multiple rects → count <rect in SVG
    SVGBackend svg(400, 300);
    svg.clear(Color::white());
    for (int i = 0; i < 5; i++) {
        svg.drawRect(50+i*60, 100, 40, 100+i*20,
                     Color::blue(),
                     LineStyle("-", 1, Color::black()));
    }
    std::string out = svg.render();
    // Đếm số <rect — phải có ít nhất 5
    int count = 0;
    size_t pos = 0;
    while ((pos = out.find("<rect", pos)) != std::string::npos) {
        ++count; pos += 5;
    }
    ASSERT_TRUE(count >= 5);
}

void test_svgbackend_polyline_points_format() {
    SVGBackend svg(400, 300);
    svg.clear(Color::white());
    std::vector<Point> pts = {{10,10},{50,50},{100,20},{200,80}};
    svg.drawPolyline(pts, LineStyle("-",2,Color::blue()));
    std::string out = svg.render();

    // SVG polyline points format: "x1,y1 x2,y2 ..."
    // Phải có "10" và "50" trong points attribute
    ASSERT_CONTAINS(out, "points=");
    ASSERT_TRUE(out.find("10") != std::string::npos);
}

// ═══════════════════════════════════════════
// Mixed plot types trong subplots
// ═══════════════════════════════════════════

void test_mixed_types_in_subplots() {
    Figure fig(900, 600);

    auto& ax1 = fig.subplot(1, 3, 1);
    ax1.plot({1.0,2.0,3.0,4.0,5.0},
             {1.0,4.0,9.0,16.0,25.0}, "b-o");
    ax1.set_title("Line");

    auto& ax2 = fig.subplot(1, 3, 2);
    ax2.scatter({1.0,2.0,3.0,4.0,5.0},
                {5.0,3.0,4.0,1.0,2.0},
                {{"c", std::string("red")}});
    ax2.set_title("Scatter");

    auto& ax3 = fig.subplot(1, 3, 3);
    ax3.bar({1.0,2.0,3.0,4.0},
            {10.0,25.0,15.0,30.0});
    ax3.set_title("Bar");

    ASSERT_EQ(fig.getAxes().size(), (size_t)3);
    std::string svg = fig.toSVG();
    ASSERT_CONTAINS(svg, "Line");
    ASSERT_CONTAINS(svg, "Scatter");
    ASSERT_CONTAINS(svg, "Bar");
    ASSERT_CONTAINS(svg, "<rect");    // bar chart
    ASSERT_CONTAINS(svg, "<polyline"); // line chart
}

// ═══════════════════════════════════════════
// Figure dimensions in SVG
// ═══════════════════════════════════════════

void test_figure_dimensions_in_svg() {
    struct TC { int w, h; };
    for (auto tc : std::vector<TC>{{400,300},{800,600},{1200,400}}) {
        Figure fig(tc.w, tc.h);
        fig.gca().plot({1.0,2.0}, {1.0,2.0}, "b-");
        std::string svg = fig.toSVG();
        ASSERT_CONTAINS(svg, "width=\"" + std::to_string(tc.w) + "\"");
        ASSERT_CONTAINS(svg, "height=\"" + std::to_string(tc.h) + "\"");
    }
}

// ═══════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════

int main() {
    std::cout << "CppPlot Layout Tests\n";
    std::cout << "=====================\n\n";

    RUN_TEST(subplot_1x1_is_gca);
    RUN_TEST(subplot_2x1_count);
    RUN_TEST(subplot_1x2_count);
    RUN_TEST(subplot_2x2_count);
    RUN_TEST(subplot_3x1_count);
    RUN_TEST(subplot_each_has_content);
    RUN_TEST(subplot_svg_larger_than_single);
    RUN_TEST(svgbackend_coordinate_mapping);
    RUN_TEST(svgbackend_multiple_rects);
    RUN_TEST(svgbackend_polyline_points_format);
    RUN_TEST(mixed_types_in_subplots);
    RUN_TEST(figure_dimensions_in_svg);

    std::cout << "\n=====================\n";
    std::cout << "Passed: " << tests_passed << "\n";
    std::cout << "Failed: " << tests_failed << "\n";
    return tests_failed > 0 ? 1 : 0;
}
