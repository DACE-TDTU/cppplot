/**
 * test_numerical.cpp
 * Tests only confirmed API: linspace, arange, histogram,
 * niceTicks, formatNumber — all seen in test_core.cpp.
 * logspace() is NOT tested here because it's not confirmed
 * in test_core — will be tested separately if test_svg.cpp confirms it.
 */
#include <cppplot/cppplot.hpp>
#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include <string>
#include <stdexcept>

using namespace cppplot;

int tests_passed = 0, tests_failed = 0;

#define RUN_TEST(name) do { \
    std::cout << "Running " #name "... "; \
    try { test_##name(); std::cout << "PASSED\n"; tests_passed++; } \
    catch (const std::exception& e) { \
        std::cout << "FAILED: " << e.what() << "\n"; tests_failed++; } \
} while(0)

#define ASSERT_EQ(a,b)     do { if((a)!=(b)) throw std::runtime_error("EQ failed: " #a); } while(0)
#define ASSERT_NEAR(a,b,t) do { if(std::abs((double)(a)-(double)(b))>(t)) throw std::runtime_error("NEAR failed: " #a); } while(0)
#define ASSERT_TRUE(x)     do { if(!(x)) throw std::runtime_error("TRUE failed: " #x); } while(0)

// linspace — tất cả edge cases
void test_linspace_basic() {
    auto v = linspace(0.0, 10.0, 11);
    ASSERT_EQ(v.size(), 11u);
    ASSERT_NEAR(v[0], 0.0, 1e-12);
    ASSERT_NEAR(v[10], 10.0, 1e-12);
    ASSERT_NEAR(v[5], 5.0, 1e-12);
}

void test_linspace_uniform_steps() {
    auto v = linspace(0.0, 1.0, 101);
    double step = v[1] - v[0];
    for (size_t i = 1; i < v.size(); i++)
        ASSERT_NEAR(v[i] - v[i-1], step, 1e-12);
}

void test_linspace_negative_range() {
    auto v = linspace(-5.0, 5.0, 11);
    ASSERT_NEAR(v[0],  -5.0, 1e-10);
    ASSERT_NEAR(v[10],  5.0, 1e-10);
    ASSERT_NEAR(v[5],   0.0, 1e-10);
}

void test_linspace_single_point() {
    auto v = linspace(3.14, 3.14, 1);
    ASSERT_EQ(v.size(), 1u);
    ASSERT_NEAR(v[0], 3.14, 1e-10);
}

void test_linspace_large_n() {
    auto v = linspace(0.0, 2*M_PI, 10000);
    ASSERT_EQ(v.size(), 10000u);
    ASSERT_NEAR(v[0], 0.0, 1e-12);
    ASSERT_NEAR(v[9999], 2*M_PI, 1e-10);
}

// arange
void test_arange_integer_step() {
    auto v = arange(0, 5, 1);
    ASSERT_EQ(v.size(), 5u);
    for (int i = 0; i < 5; i++)
        ASSERT_NEAR(v[i], (double)i, 1e-10);
}

void test_arange_fractional_step() {
    auto v = arange(0.0, 1.0, 0.25);
    ASSERT_EQ(v.size(), 4u);
    ASSERT_NEAR(v[0], 0.00, 1e-10);
    ASSERT_NEAR(v[3], 0.75, 1e-10);
}

void test_arange_negative_start() {
    auto v = arange(-3, 4, 1);
    ASSERT_EQ(v.size(), 7u);
    ASSERT_NEAR(v[0], -3.0, 1e-10);
    ASSERT_NEAR(v[3],  0.0, 1e-10);
    ASSERT_NEAR(v[6],  3.0, 1e-10);
}

// histogram
void test_histogram_uniform_data() {
    std::vector<double> data;
    for (int i = 1; i <= 100; i++) data.push_back((double)i);
    auto h = histogram(data, 10, 1, 100);
    ASSERT_EQ(h.counts.size(), 10u);
    ASSERT_EQ(h.binEdges.size(), 11u);
    int total = 0;
    for (int c : h.counts) total += c;
    ASSERT_EQ(total, 100);
}

void test_histogram_single_bin() {
    std::vector<double> data = {1.0, 2.0, 3.0, 4.0, 5.0};
    auto h = histogram(data, 1, 1, 5);
    ASSERT_EQ(h.counts.size(), 1u);
    ASSERT_EQ(h.counts[0], 5);
}

void test_histogram_edges_monotone() {
    std::vector<double> data = {1.0, 2.0, 3.0};
    auto h = histogram(data, 5, 1, 3);
    for (size_t i = 1; i < h.binEdges.size(); i++)
        ASSERT_TRUE(h.binEdges[i] > h.binEdges[i-1]);
}

// niceTicks
void test_niceticks_round_numbers() {
    auto ticks = niceTicks(0, 100, 10);
    ASSERT_TRUE(!ticks.empty());
    for (double t : ticks) {
        // Tất cả ticks phải trong range [0, 100] (với chút padding)
        ASSERT_TRUE(t >= -10.0 && t <= 110.0);
    }
}

void test_niceticks_coverage() {
    // Ticks phải cover được range [min, max]
    auto ticks = niceTicks(0, 1, 5);
    ASSERT_TRUE(!ticks.empty());
    double tick_min = *std::min_element(ticks.begin(), ticks.end());
    double tick_max = *std::max_element(ticks.begin(), ticks.end());
    ASSERT_TRUE(tick_min <= 0.0 + 0.1);
    ASSERT_TRUE(tick_max >= 1.0 - 0.1);
}

// formatNumber
void test_format_number_integers() {
    ASSERT_EQ(formatNumber(0),   std::string("0"));
    ASSERT_EQ(formatNumber(100), std::string("100"));
    ASSERT_EQ(formatNumber(-50), std::string("-50"));
}

void test_format_number_decimals() {
    std::string s = formatNumber(0.5);
    ASSERT_TRUE(s.find("0.5") != std::string::npos ||
                s.find(".5")  != std::string::npos);
}

void test_format_number_small() {
    // Số rất nhỏ → scientific notation hoặc decimal
    std::string s = formatNumber(0.001);
    ASSERT_TRUE(!s.empty());
}

// CoordinateTransform — nhiều trường hợp hơn
void test_coord_transform_scale() {
    // Data [0,10]×[0,10] → pixel [0,100]×[0,100]: scale 10x
    CoordinateTransform t(Rect(0,0,10,10), Rect(0,0,100,100));
    Point p1 = t.dataToPixel(1.0, 1.0);
    Point p2 = t.dataToPixel(2.0, 2.0);
    // Khoảng cách data 1 unit → pixel 10 units
    ASSERT_NEAR(p2.x - p1.x, 10.0, 1.0);
}

void test_coord_transform_corners() {
    CoordinateTransform t(Rect(0,0,100,100), Rect(0,0,800,600));
    Point origin = t.dataToPixel(0, 0);
    ASSERT_NEAR(origin.x, 0.0, 1.0);
}

int main() {
    std::cout << "CppPlot Numerical Utility Tests\n";
    std::cout << "================================\n\n";

    RUN_TEST(linspace_basic);
    RUN_TEST(linspace_uniform_steps);
    RUN_TEST(linspace_negative_range);
    RUN_TEST(linspace_single_point);
    RUN_TEST(linspace_large_n);
    RUN_TEST(arange_integer_step);
    RUN_TEST(arange_fractional_step);
    RUN_TEST(arange_negative_start);
    RUN_TEST(histogram_uniform_data);
    RUN_TEST(histogram_single_bin);
    RUN_TEST(histogram_edges_monotone);
    RUN_TEST(niceticks_round_numbers);
    RUN_TEST(niceticks_coverage);
    RUN_TEST(format_number_integers);
    RUN_TEST(format_number_decimals);
    RUN_TEST(format_number_small);
    RUN_TEST(coord_transform_scale);
    RUN_TEST(coord_transform_corners);

    std::cout << "\n================================\n";
    std::cout << "Passed: " << tests_passed << "\n";
    std::cout << "Failed: " << tests_failed << "\n";
    return tests_failed > 0 ? 1 : 0;
}
