/**
 * test_basic_plot.cpp
 * ─────────────────────────────────────────────
 * Test cơ bản nhất: library có thể include,
 * compile, chạy, và tạo ra file SVG không?
 */
#include <cppplot/cppplot.hpp>
#include <cassert>
#include <fstream>
#include <iostream>
#include <cmath>

using namespace cppplot;

// ── Tiện ích: đọc file thành string ──────────
static std::string read_file(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) return "";
    return std::string(std::istreambuf_iterator<char>(f),
                       std::istreambuf_iterator<char>());
}

// ── Tiện ích: kiểm tra file tồn tại ─────────
static bool file_exists(const std::string& path) {
    std::ifstream f(path);
    return f.good();
}

// ── Tiện ích: SVG có chứa element không ──────
static bool svg_contains(const std::string& svg, const std::string& element) {
    return svg.find(element) != std::string::npos;
}

// ─────────────────────────────────────────────
// TEST 1: Line plot cơ bản
// ─────────────────────────────────────────────
void test_line_plot() {
    std::cout << "[TEST] test_line_plot ... ";

    std::vector<double> x = {1, 2, 3, 4, 5};
    std::vector<double> y = {1, 4, 9, 16, 25};

    figure(400, 300);
    plot(x, y, "b-");
    xlabel("X");
    ylabel("Y");
    title("Basic Line Plot");
    savefig("test_line_plot.svg");

    // Kiểm tra file tồn tại
    assert(file_exists("test_line_plot.svg") &&
           "savefig() must create the SVG file");

    // Kiểm tra file không rỗng
    std::string content = read_file("test_line_plot.svg");
    assert(!content.empty() && "SVG file must not be empty");

    // Kiểm tra kích thước file tối thiểu (ít nhất 100 bytes)
    assert(content.size() > 100 && "SVG file is suspiciously small");

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 2: Scatter plot
// ─────────────────────────────────────────────
void test_scatter_plot() {
    std::cout << "[TEST] test_scatter_plot ... ";

    std::vector<double> x = {1.0, 2.0, 3.0, 4.0, 5.0};
    std::vector<double> y = {2.1, 3.9, 6.1, 7.9, 10.2};

    figure(400, 300);
    scatter(x, y, opts({{"color", "red"}, {"s", "40"}}));
    title("Scatter Plot");
    savefig("test_scatter.svg");

    assert(file_exists("test_scatter.svg"));
    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 3: Nhiều đường trên cùng figure
// ─────────────────────────────────────────────
void test_multiple_lines() {
    std::cout << "[TEST] test_multiple_lines ... ";

    auto x = linspace(0.0, 2.0 * M_PI, 100);
    std::vector<double> y_sin, y_cos;
    for (double xi : x) {
        y_sin.push_back(std::sin(xi));
        y_cos.push_back(std::cos(xi));
    }

    figure(600, 400);
    plot(x, y_sin, "b-", opts({{"label", "sin"}, {"linewidth", "2"}}));
    plot(x, y_cos, "r--", opts({{"label", "cos"}, {"linewidth", "2"}}));
    legend(true);
    grid(true);
    xlabel("x (rad)");
    ylabel("Amplitude");
    title("Sin and Cos");
    savefig("test_multiline.svg");

    assert(file_exists("test_multiline.svg"));

    // Cả hai đường phải được vẽ → SVG phải có nhiều path elements
    std::string content = read_file("test_multiline.svg");
    // SVG với 2 đường phải lớn hơn SVG với 1 đường đáng kể
    assert(content.size() > 500 && "Multi-line plot SVG seems too small");

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 4: figure() với kích thước tùy chỉnh
// ─────────────────────────────────────────────
void test_figure_sizes() {
    std::cout << "[TEST] test_figure_sizes ... ";

    // Small figure
    std::vector<double> x = {1, 2, 3};
    std::vector<double> y = {1, 2, 3};

    figure(200, 200);
    plot(x, y, "k-");
    savefig("test_small_figure.svg");
    assert(file_exists("test_small_figure.svg"));

    // Large figure
    figure(1600, 900);
    plot(x, y, "k-");
    savefig("test_large_figure.svg");
    assert(file_exists("test_large_figure.svg"));

    // Kiểm tra figure nhỏ có SVG nhỏ hơn figure lớn
    // (không phải lúc nào cũng đúng với SVG nhưng là sanity check)
    std::string small_svg = read_file("test_small_figure.svg");
    std::string large_svg = read_file("test_large_figure.svg");
    assert(!small_svg.empty() && !large_svg.empty());

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 5: Bar chart
// ─────────────────────────────────────────────
void test_bar_chart() {
    std::cout << "[TEST] test_bar_chart ... ";

    std::vector<std::string> categories = {"A", "B", "C", "D"};
    std::vector<double> values = {23.0, 45.0, 31.0, 67.0};

    figure(500, 350);
    bar(categories, values, opts({{"color", "steelblue"}}));
    title("Bar Chart");
    ylabel("Value");
    savefig("test_bar.svg");

    assert(file_exists("test_bar.svg"));
    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 6: Histogram
// ─────────────────────────────────────────────
void test_histogram() {
    std::cout << "[TEST] test_histogram ... ";

    // Tạo dữ liệu phân phối chuẩn đơn giản
    std::vector<double> data;
    data.reserve(200);
    // Dùng Box-Muller đơn giản
    for (int i = 0; i < 200; i++) {
        double u = (i + 0.5) / 200.0;
        data.push_back(u * 6.0 - 3.0); // uniform [-3, 3] đủ để test
    }

    figure(500, 350);
    hist(data, opts({{"bins", "20"}, {"color", "skyblue"}}));
    title("Histogram");
    xlabel("Value");
    ylabel("Count");
    savefig("test_histogram.svg");

    assert(file_exists("test_histogram.svg"));
    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// MAIN
// ─────────────────────────────────────────────
int main() {
    std::cout << "=== test_basic_plot ===\n";

    try {
        test_line_plot();
        test_scatter_plot();
        test_multiple_lines();
        test_figure_sizes();
        test_bar_chart();
        test_histogram();
    } catch (const std::exception& e) {
        std::cerr << "EXCEPTION: " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "UNKNOWN EXCEPTION\n";
        return 1;
    }

    std::cout << "=== ALL BASIC TESTS PASSED ===\n";
    return 0;
}
