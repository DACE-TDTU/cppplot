/**
 * test_layout.cpp
 * ─────────────────────────────────────────────
 * Kiểm tra layout system: subplot, GridSpec,
 * add_axes, inset_axes, subplot_span.
 */
#include <cppplot/cppplot.hpp>
#include <cassert>
#include <fstream>
#include <iostream>
#include <cmath>

using namespace cppplot;

static bool file_exists(const std::string& p) {
    return std::ifstream(p).good();
}

static size_t file_size(const std::string& p) {
    std::ifstream f(p, std::ios::binary | std::ios::ate);
    return f.is_open() ? (size_t)f.tellg() : 0;
}

// ─────────────────────────────────────────────
// TEST 1: subplot 2x2
// ─────────────────────────────────────────────
void test_subplot_2x2() {
    std::cout << "[TEST] test_subplot_2x2 ... ";

    auto x = linspace(0.0, 2 * M_PI, 50);
    std::vector<double> y1, y2, y3, y4;
    for (double xi : x) {
        y1.push_back(std::sin(xi));
        y2.push_back(std::cos(xi));
        y3.push_back(std::sin(2 * xi));
        y4.push_back(std::cos(2 * xi));
    }

    figure(800, 600);
    subplot(2, 2, 1); plot(x, y1, "b-"); title("sin(x)");
    subplot(2, 2, 2); plot(x, y2, "r-"); title("cos(x)");
    subplot(2, 2, 3); plot(x, y3, "g-"); title("sin(2x)");
    subplot(2, 2, 4); plot(x, y4, "m-"); title("cos(2x)");
    savefig("test_subplot_2x2.svg");

    assert(file_exists("test_subplot_2x2.svg"));

    // 2x2 subplot SVG phải lớn hơn đáng kể so với single plot
    size_t sz = file_size("test_subplot_2x2.svg");
    assert(sz > 500 && "2x2 subplot SVG seems too small");

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 2: subplot 3x1 (Bode-style layout)
// ─────────────────────────────────────────────
void test_subplot_3x1() {
    std::cout << "[TEST] test_subplot_3x1 ... ";

    auto x = linspace(0.0, 10.0, 100);
    std::vector<double> y;
    for (double xi : x) y.push_back(std::sin(xi) * std::exp(-0.2 * xi));

    figure(600, 800);
    subplot(3, 1, 1);
    plot(x, y, "b-"); title("Top"); ylabel("Signal");

    subplot(3, 1, 2);
    std::vector<double> dy;
    for (size_t i = 1; i < y.size(); i++)
        dy.push_back((y[i] - y[i-1]) / (x[i] - x[i-1]));
    std::vector<double> x_dy(x.begin() + 1, x.end());
    plot(x_dy, dy, "r-"); ylabel("Derivative");

    subplot(3, 1, 3);
    // Running integral (trapezoidal)
    std::vector<double> integ;
    double sum = 0;
    for (size_t i = 0; i < y.size(); i++) {
        if (i > 0) sum += 0.5 * (y[i] + y[i-1]) * (x[i] - x[i-1]);
        integ.push_back(sum);
    }
    plot(x, integ, "g-"); xlabel("t"); ylabel("Integral");

    savefig("test_subplot_3x1.svg");
    assert(file_exists("test_subplot_3x1.svg"));
    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 3: add_axes (free-form placement)
// ─────────────────────────────────────────────
void test_add_axes() {
    std::cout << "[TEST] test_add_axes ... ";

    auto x = linspace(0.0, 10.0, 100);
    std::vector<double> y;
    for (double xi : x) y.push_back(std::sin(xi));

    figure(800, 600);
    // Main axes
    add_axes(0.1, 0.3, 0.55, 0.6);
    plot(x, y, "b-"); title("Main Plot"); grid(true);

    // Side panel
    std::vector<double> xh, yh;
    for (int i = 0; i < 30; i++) {
        xh.push_back(i * 0.1 - 1.5);
        yh.push_back(std::exp(-xh.back() * xh.back() / 0.5) * 10.0);
    }
    add_axes(0.7, 0.3, 0.25, 0.6);
    bar(xh, yh, opts({{"color","steelblue"}}));
    title("Distribution");

    savefig("test_add_axes.svg");
    assert(file_exists("test_add_axes.svg"));
    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 4: inset_axes (zoom inset)
// ─────────────────────────────────────────────
void test_inset_axes() {
    std::cout << "[TEST] test_inset_axes ... ";

    auto x = linspace(0.0, 4 * M_PI, 200);
    std::vector<double> y;
    for (double xi : x) y.push_back(std::sin(xi) * std::exp(-0.1 * xi));

    figure(800, 500);
    subplot(1, 1, 1);
    plot(x, y, "b-", opts({{"linewidth","1.5"}}));
    title("Main View with Zoom Inset"); grid(true);

    // Inset: zoom vào đầu tín hiệu
    auto x_zoom = linspace(0.0, 1.0, 50);
    std::vector<double> y_zoom;
    for (double xi : x_zoom)
        y_zoom.push_back(std::sin(xi) * std::exp(-0.1 * xi));

    inset_axes(0.55, 0.5, 0.4, 0.35);
    plot(x_zoom, y_zoom, "r-", opts({{"linewidth","2"}}));
    title("Zoomed [0,1]");

    savefig("test_inset.svg");
    assert(file_exists("test_inset.svg"));
    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 5: GridSpec với custom ratios
// ─────────────────────────────────────────────
void test_gridspec() {
    std::cout << "[TEST] test_gridspec ... ";

    auto x = linspace(0.0, 2 * M_PI, 100);
    std::vector<double> y;
    for (double xi : x) y.push_back(std::sin(xi));

    std::vector<double> xdata, ydata;
    for (int i = 0; i < 50; i++) {
        xdata.push_back(i * 0.1);
        ydata.push_back(std::sin(i * 0.1) + 0.1 * i);
    }

    figure(1000, 700);
    GridSpec gs(2, 3);
    gs.setWidthRatios({2, 1, 1});
    gs.setHeightRatios({1, 1});
    gs.setSpacing(0.08, 0.12);
    gcf().setLayout(gs);

    // Wide plot spanning row 0, col 0-1
    subplot_span(0, 1, 0, 0);
    plot(x, y, "b-"); title("Wide Plot"); grid(true);

    subplot_span(0, 0, 2, 2);
    scatter(xdata, ydata, opts({{"color","red"}, {"s","10"}}));
    title("Scatter");

    subplot_span(1, 1, 0, 2);
    std::vector<std::string> cats = {"A","B","C","D"};
    std::vector<double> vals = {3.0, 7.0, 5.0, 9.0};
    bar(cats, vals, opts({{"color","steelblue"}}));
    title("Bar");

    savefig("test_gridspec.svg");
    assert(file_exists("test_gridspec.svg"));
    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// MAIN
// ─────────────────────────────────────────────
int main() {
    std::cout << "=== test_layout ===\n";

    try {
        test_subplot_2x2();
        test_subplot_3x1();
        test_add_axes();
        test_inset_axes();
        test_gridspec();
    } catch (const std::exception& e) {
        std::cerr << "EXCEPTION: " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "UNKNOWN EXCEPTION\n";
        return 1;
    }

    std::cout << "=== ALL LAYOUT TESTS PASSED ===\n";
    return 0;
}
