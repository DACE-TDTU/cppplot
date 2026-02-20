/**
 * @file test_core.cpp
 * @brief Unit tests for core functionality
 */

#include <cppplot/cppplot.hpp>
#include <iostream>
#include <cassert>
#include <cmath>

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

#define ASSERT_EQ(a, b) do { \
    if ((a) != (b)) throw std::runtime_error("Assertion failed: " #a " != " #b); \
} while(0)

#define ASSERT_NEAR(a, b, tol) do { \
    if (std::abs((a) - (b)) > (tol)) throw std::runtime_error("Assertion failed: " #a " not near " #b); \
} while(0)

#define ASSERT_TRUE(x) do { \
    if (!(x)) throw std::runtime_error("Assertion failed: " #x); \
} while(0)

// ============ Color Tests ============

TEST(color_rgb) {
    Color c(255, 128, 64);
    ASSERT_EQ(c.r(), 255);
    ASSERT_EQ(c.g(), 128);
    ASSERT_EQ(c.b(), 64);
    ASSERT_EQ(c.a(), 255);
}

TEST(color_hex) {
    Color c = Color::fromHex("#FF8040");
    ASSERT_EQ(c.r(), 255);
    ASSERT_EQ(c.g(), 128);
    ASSERT_EQ(c.b(), 64);
}

TEST(color_named) {
    Color red = Color::fromName("red");
    ASSERT_EQ(red.r(), 255);
    ASSERT_EQ(red.g(), 0);
    ASSERT_EQ(red.b(), 0);
    
    Color blue = Color::fromChar('b');
    ASSERT_EQ(blue.r(), 0);
    ASSERT_EQ(blue.g(), 0);
    ASSERT_EQ(blue.b(), 255);
}

TEST(color_to_hex) {
    Color c(255, 128, 64);
    std::string hex = c.toHex();
    ASSERT_EQ(hex, "#ff8040");
}

TEST(color_alpha) {
    Color c = Color::red().withAlpha(0.5);
    ASSERT_EQ(c.r(), 255);
    ASSERT_NEAR(c.aNorm(), 0.5, 0.01);
}

TEST(colormap) {
    Colormap viridis = Colormap::viridis();
    Color c0 = viridis(0.0);
    Color c1 = viridis(1.0);
    // Just check it doesn't crash and returns valid colors
    ASSERT_TRUE(c0.r() >= 0 && c0.r() <= 255);
    ASSERT_TRUE(c1.r() >= 0 && c1.r() <= 255);
}

// ============ Style Tests ============

TEST(plot_style_parse) {
    PlotStyle style = PlotStyle::parse("b-o");
    ASSERT_EQ(style.line.style, "-");
    ASSERT_EQ(style.marker.marker, "o");
    ASSERT_EQ(style.line.color, Color::blue());
}

TEST(plot_style_dashed) {
    PlotStyle style = PlotStyle::parse("r--");
    ASSERT_EQ(style.line.style, "--");
    ASSERT_EQ(style.line.color, Color::red());
}

TEST(line_style_dash_array) {
    LineStyle solid("-");
    ASSERT_EQ(solid.toDashArray(), "");
    
    LineStyle dashed("--");
    ASSERT_EQ(dashed.toDashArray(), "6,4");
    
    LineStyle dotted(":");
    ASSERT_EQ(dotted.toDashArray(), "2,2");
}

// ============ Utils Tests ============

TEST(linspace) {
    auto v = linspace(0, 10, 11);
    ASSERT_EQ(v.size(), 11u);
    ASSERT_NEAR(v[0], 0.0, 0.001);
    ASSERT_NEAR(v[10], 10.0, 0.001);
    ASSERT_NEAR(v[5], 5.0, 0.001);
}

TEST(arange) {
    auto v = arange(0, 5, 1);
    ASSERT_EQ(v.size(), 5u);
    ASSERT_NEAR(v[0], 0.0, 0.001);
    ASSERT_NEAR(v[4], 4.0, 0.001);
}

TEST(histogram) {
    std::vector<double> data = {1, 2, 2, 3, 3, 3, 4, 4, 5};
    auto result = histogram(data, 5, 1, 5);
    ASSERT_EQ(result.counts.size(), 5u);
    ASSERT_EQ(result.binEdges.size(), 6u);
}

TEST(nice_ticks) {
    auto ticks = niceTicks(0, 100, 10);
    ASSERT_TRUE(!ticks.empty());
    // Ticks should be nice round numbers
    for (double t : ticks) {
        ASSERT_TRUE(std::fmod(t, 10) < 0.001 || std::fmod(t, 10) > 9.999);
    }
}

TEST(format_number) {
    ASSERT_EQ(formatNumber(0), "0");
    ASSERT_EQ(formatNumber(100), "100");
    // formatNumber may have different precision for small numbers
    std::string formatted = formatNumber(0.5);
    ASSERT_TRUE(formatted.find("0.5") != std::string::npos || 
                formatted.find(".5") != std::string::npos);
}

TEST(coordinate_transform) {
    CoordinateTransform transform(
        Rect(0, 0, 100, 100),  // data rect
        Rect(0, 0, 800, 600)   // pixel rect
    );
    
    Point p = transform.dataToPixel(50, 50);
    ASSERT_NEAR(p.x, 400, 1);
    ASSERT_NEAR(p.y, 300, 1);
}

// ============ Types Tests ============

TEST(point) {
    Point p1(1, 2);
    Point p2(3, 4);
    
    Point sum = p1 + p2;
    ASSERT_NEAR(sum.x, 4, 0.001);
    ASSERT_NEAR(sum.y, 6, 0.001);
    
    Point diff = p2 - p1;
    ASSERT_NEAR(diff.x, 2, 0.001);
    ASSERT_NEAR(diff.y, 2, 0.001);
}

TEST(rect) {
    Rect r(10, 20, 100, 50);
    ASSERT_NEAR(r.left(), 10, 0.001);
    ASSERT_NEAR(r.right(), 110, 0.001);
    ASSERT_NEAR(r.top(), 20, 0.001);
    ASSERT_NEAR(r.bottom(), 70, 0.001);
    ASSERT_NEAR(r.centerX(), 60, 0.001);
    ASSERT_NEAR(r.centerY(), 45, 0.001);
}

TEST(limits) {
    Limits lim(0, 100);
    ASSERT_NEAR(lim.range(), 100, 0.001);
    ASSERT_NEAR(lim.center(), 50, 0.001);
    
    lim.include(150);
    ASSERT_NEAR(lim.max, 150, 0.001);
}

TEST(data_series) {
    DataSeries<double> ds({1, 2, 3, 4, 5});
    ASSERT_EQ(ds.size(), 5u);
    ASSERT_NEAR(ds.min(), 1.0, 0.001);
    ASSERT_NEAR(ds.max(), 5.0, 0.001);
    ASSERT_NEAR(ds.mean(), 3.0, 0.001);
    ASSERT_NEAR(ds.sum(), 15.0, 0.001);
}

// ============ Main ============

int main() {
    std::cout << "CppPlot Core Tests\n";
    std::cout << "==================\n\n";
    
    // Color tests
    RUN_TEST(color_rgb);
    RUN_TEST(color_hex);
    RUN_TEST(color_named);
    RUN_TEST(color_to_hex);
    RUN_TEST(color_alpha);
    RUN_TEST(colormap);
    
    // Style tests
    RUN_TEST(plot_style_parse);
    RUN_TEST(plot_style_dashed);
    RUN_TEST(line_style_dash_array);
    
    // Utils tests
    RUN_TEST(linspace);
    RUN_TEST(arange);
    RUN_TEST(histogram);
    RUN_TEST(nice_ticks);
    RUN_TEST(format_number);
    RUN_TEST(coordinate_transform);
    
    // Types tests
    RUN_TEST(point);
    RUN_TEST(rect);
    RUN_TEST(limits);
    RUN_TEST(data_series);
    
    std::cout << "\n==================\n";
    std::cout << "Tests passed: " << tests_passed << "\n";
    std::cout << "Tests failed: " << tests_failed << "\n";
    
    return tests_failed > 0 ? 1 : 0;
}
