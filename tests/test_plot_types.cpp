/**
 * test_plot_types.cpp
 * ─────────────────────────────────────────────
 * Kiểm tra tất cả plot types được liệt kê
 * trong README đều hoạt động không crash.
 */
#include <cppplot/cppplot.hpp>
#include <cassert>
#include <fstream>
#include <iostream>
#include <cmath>
#include <vector>
#include <string>

using namespace cppplot;

static bool file_exists(const std::string& p) {
    return std::ifstream(p).good();
}

// ─────────────────────────────────────────────
// Error bars
// ─────────────────────────────────────────────
void test_errorbar() {
    std::cout << "[TEST] test_errorbar ... ";

    std::vector<double> x    = {1, 2, 3, 4, 5};
    std::vector<double> y    = {2.1, 4.0, 5.9, 8.1, 10.0};
    std::vector<double> yerr = {0.5, 0.4, 0.6, 0.5, 0.7};

    figure(500, 350);
    errorbar(x, y, yerr, opts({{"color", "blue"}, {"capsize", "5"}}));
    title("Error Bars");
    savefig("test_errorbar.svg");

    assert(file_exists("test_errorbar.svg"));
    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// Log scale axes
// ─────────────────────────────────────────────
void test_log_scale() {
    std::cout << "[TEST] test_log_scale ... ";

    auto x = linspace(1.0, 100.0, 50);
    std::vector<double> y;
    for (double xi : x) y.push_back(std::pow(10.0, xi / 25.0));

    figure(600, 400);
    plot(x, y, "b-");
    yscale("log");
    grid(true);
    title("Log Scale Y");
    savefig("test_logscale.svg");

    assert(file_exists("test_logscale.svg"));
    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// Text annotations + axhline + axvline
// ─────────────────────────────────────────────
void test_annotations() {
    std::cout << "[TEST] test_annotations ... ";

    auto x = linspace(0.0, 2 * M_PI, 100);
    std::vector<double> y;
    for (double xi : x) y.push_back(std::sin(xi));

    figure(600, 400);
    plot(x, y, "b-");
    text(M_PI / 2.0, 1.0, "Peak",
         opts({{"ha", "center"}, {"fontsize", "12"}}));
    axhline(0.0,  opts({{"color", "gray"}, {"linestyle", "--"}}));
    axvline(M_PI, opts({{"color", "red"},  {"linestyle", ":"}}));
    title("Annotations");
    savefig("test_annotations.svg");

    assert(file_exists("test_annotations.svg"));

    // "Peak" text phải muncul dalam SVG
    std::ifstream f("test_annotations.svg");
    std::string content(std::istreambuf_iterator<char>(f),
                        std::istreambuf_iterator<char>());
    assert(content.find("Peak") != std::string::npos &&
           "text() annotation must appear in SVG");

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// Subplots 2x1
// ─────────────────────────────────────────────
void test_subplots_2x1() {
    std::cout << "[TEST] test_subplots_2x1 ... ";

    auto x = linspace(0.0, 2 * M_PI, 100);
    std::vector<double> y_sin, y_cos;
    for (double xi : x) {
        y_sin.push_back(std::sin(xi));
        y_cos.push_back(std::cos(xi));
    }

    figure(600, 600);
    subplot(2, 1, 1);
    plot(x, y_sin, "b-");
    ylabel("sin(x)"); grid(true);

    subplot(2, 1, 2);
    plot(x, y_cos, "r-");
    xlabel("x (rad)"); ylabel("cos(x)"); grid(true);

    savefig("test_subplots_2x1.svg");
    assert(file_exists("test_subplots_2x1.svg"));
    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// Subplots 1x3
// ─────────────────────────────────────────────
void test_subplots_1x3() {
    std::cout << "[TEST] test_subplots_1x3 ... ";

    std::vector<double> x = {1, 2, 3, 4, 5};
    std::vector<double> y = {1, 4, 9, 16, 25};

    figure(1200, 400);
    subplot(1, 3, 1);
    plot(x, y, "b-o"); title("Line");

    subplot(1, 3, 2);
    scatter(x, y, opts({{"color", "red"}})); title("Scatter");

    subplot(1, 3, 3);
    std::vector<std::string> cats = {"A","B","C","D","E"};
    bar(cats, y, opts({{"color", "green"}})); title("Bar");

    savefig("test_subplots_1x3.svg");
    assert(file_exists("test_subplots_1x3.svg"));
    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// fill_between dengan confidence band
// ─────────────────────────────────────────────
void test_fill_between() {
    std::cout << "[TEST] test_fill_between ... ";

    auto x = linspace(0.0, 10.0, 100);
    std::vector<double> mean, upper, lower;
    for (double xi : x) {
        double v = std::sin(xi) * std::exp(-0.1 * xi);
        mean.push_back(v);
        upper.push_back(v + 0.3);
        lower.push_back(v - 0.3);
    }

    figure(600, 400);
    fill_between(x, lower, upper,
                 opts({{"color", "royalblue"}, {"alpha", "0.25"}}));
    plot(x, mean, "b-", opts({{"linewidth", "2"}, {"label", "Mean"}}));
    legend(true);
    title("Signal with Confidence Band");
    savefig("test_fill_between.svg");

    assert(file_exists("test_fill_between.svg"));
    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// Format strings — semua kombinasi dasar
// ─────────────────────────────────────────────
void test_format_strings() {
    std::cout << "[TEST] test_format_strings ... ";

    std::vector<double> x = {1, 2, 3, 4, 5};
    std::vector<double> y = {1, 2, 3, 4, 5};

    // Dùng list initializer thay vì aggregate init
    std::vector<std::string> formats;
    formats.push_back("b-");
    formats.push_back("r--");
    formats.push_back("g:");
    formats.push_back("k-.");
    formats.push_back("m-o");
    formats.push_back("c-s");
    formats.push_back("y-^");

    for (const auto& fmt : formats) {
        std::string fname = "test_fmt_" + fmt[0] + ".svg";
        // Thay thế ký tự đặc biệt trong tên file
        for (char& c : fname) {
            if (c == '-' || c == '.') c = '_';
        }
        figure(300, 200);
        plot(x, y, fmt.c_str());
        savefig(fname);
        assert(file_exists(fname) &&
               ("Format string '" + fmt + "' must produce SVG").c_str());
    }

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// MAIN
// ─────────────────────────────────────────────
int main() {
    std::cout << "=== test_plot_types ===\n";

    try {
        test_errorbar();
        test_log_scale();
        test_annotations();
        test_subplots_2x1();
        test_subplots_1x3();
        test_fill_between();
        test_format_strings();
    } catch (const std::exception& e) {
        std::cerr << "EXCEPTION: " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "UNKNOWN EXCEPTION\n";
        return 1;
    }

    std::cout << "=== ALL PLOT TYPE TESTS PASSED ===\n";
    return 0;
}
