/**
 * test_numerical.cpp
 * ─────────────────────────────────────────────
 * Kiểm tra các utility functions (linspace,
 * logspace, v.v.) có đúng về mặt số học không.
 * Đây là foundation mà tất cả các test khác phụ thuộc.
 */
#include <cppplot/cppplot.hpp>
#include <cassert>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <numeric>

using namespace cppplot;

static const double EPS     = 1e-10;
static const double EPS_REL = 1e-9;

// ─────────────────────────────────────────────
// TEST 1: linspace — giống numpy.linspace
// ─────────────────────────────────────────────
void test_linspace() {
    std::cout << "[TEST] test_linspace ... ";

    // Basic
    auto v = linspace(0.0, 1.0, 5);
    assert(v.size() == 5 && "linspace must return exactly N points");
    assert(std::abs(v[0] - 0.0) < EPS && "linspace: first = start");
    assert(std::abs(v[4] - 1.0) < EPS && "linspace: last = stop");
    assert(std::abs(v[2] - 0.5) < EPS && "linspace: midpoint correct");

    // Step uniformity
    for (size_t i = 1; i < v.size(); i++) {
        double step_i = v[i] - v[i-1];
        assert(std::abs(step_i - 0.25) < EPS &&
               "linspace steps must be uniform");
    }

    // Larger range
    auto w = linspace(0.0, 2 * M_PI, 1000);
    assert(w.size() == 1000);
    assert(std::abs(w.back() - 2 * M_PI) < 1e-10);

    // Negative start
    auto neg = linspace(-5.0, 5.0, 11);
    assert(std::abs(neg[0] - (-5.0)) < EPS);
    assert(std::abs(neg[10] - 5.0) < EPS);
    assert(std::abs(neg[5] - 0.0) < EPS && "midpoint of [-5,5] must be 0");

    // Single point
    auto one = linspace(3.14, 3.14, 1);
    assert(one.size() == 1 && std::abs(one[0] - 3.14) < EPS);

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 2: logspace — giống numpy.logspace
// logspace(a, b, N) → 10^a to 10^b, N points
// ─────────────────────────────────────────────
void test_logspace() {
    std::cout << "[TEST] test_logspace ... ";

    // logspace(-1, 2, 4) → {0.1, 1.0, 10.0, 100.0}
    auto v = logspace(-1.0, 2.0, 4);
    assert(v.size() == 4 && "logspace must return N points");
    assert(std::abs(v[0] - 0.1)   < 1e-9 && "logspace: first point");
    assert(std::abs(v[1] - 1.0)   < 1e-9 && "logspace: second point");
    assert(std::abs(v[2] - 10.0)  < 1e-8 && "logspace: third point");
    assert(std::abs(v[3] - 100.0) < 1e-7 && "logspace: fourth point");

    // All values phải dương
    auto w = logspace(-3.0, 3.0, 100);
    for (double x : w) {
        assert(x > 0.0 && "logspace must produce strictly positive values");
    }

    // Tỷ lệ giữa các điểm liên tiếp phải equal (log-uniform spacing)
    auto ratio_first  = w[1] / w[0];
    auto ratio_last   = w.back() / w[w.size()-2];
    assert(std::abs(ratio_first - ratio_last) / ratio_first < 1e-6 &&
           "logspace must have uniform ratio between consecutive points");

    // Monotonically increasing
    for (size_t i = 1; i < w.size(); i++) {
        assert(w[i] > w[i-1] && "logspace must be monotonically increasing");
    }

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 3: Data range handling
// (library không crash với edge cases)
// ─────────────────────────────────────────────
void test_edge_cases() {
    std::cout << "[TEST] test_edge_cases ... ";

    // 1. Tất cả giá trị giống nhau (flat line)
    {
        std::vector<double> x = {1, 2, 3, 4, 5};
        std::vector<double> y = {3, 3, 3, 3, 3};
        figure(400, 300);
        plot(x, y, "b-");
        savefig("test_flat_line.svg");
        assert(std::ifstream("test_flat_line.svg").good());
    }

    // 2. Chỉ 2 điểm
    {
        std::vector<double> x = {0.0, 1.0};
        std::vector<double> y = {0.0, 1.0};
        figure(400, 300);
        plot(x, y, "r-");
        savefig("test_two_points.svg");
        assert(std::ifstream("test_two_points.svg").good());
    }

    // 3. Giá trị âm
    {
        std::vector<double> x = {-5, -3, -1, 0, 1, 3, 5};
        std::vector<double> y = {25,  9,  1, 0, 1, 9, 25};
        figure(400, 300);
        plot(x, y, "g-");
        ylabel("x^2"); xlabel("x");
        savefig("test_negative_values.svg");
        assert(std::ifstream("test_negative_values.svg").good());
    }

    // 4. Giá trị rất lớn
    {
        std::vector<double> x = {1e6, 2e6, 3e6};
        std::vector<double> y = {1e9, 4e9, 9e9};
        figure(400, 300);
        plot(x, y, "b-");
        savefig("test_large_values.svg");
        assert(std::ifstream("test_large_values.svg").good());
    }

    // 5. Giá trị rất nhỏ
    {
        std::vector<double> x = {1e-6, 2e-6, 3e-6};
        std::vector<double> y = {1e-9, 4e-9, 9e-9};
        figure(400, 300);
        plot(x, y, "b-");
        savefig("test_small_values.svg");
        assert(std::ifstream("test_small_values.svg").good());
    }

    // 6. Dữ liệu lớn (1000 điểm)
    {
        auto x = linspace(0.0, 100.0, 1000);
        std::vector<double> y;
        for (double xi : x)
            y.push_back(std::sin(xi) * xi / 100.0);
        figure(800, 400);
        plot(x, y, "b-");
        savefig("test_large_dataset.svg");
        assert(std::ifstream("test_large_dataset.svg").good());
    }

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 4: Reproducibility — cùng input → cùng output
// ─────────────────────────────────────────────
void test_reproducibility() {
    std::cout << "[TEST] test_reproducibility ... ";

    auto x = linspace(0.0, 2 * M_PI, 100);
    std::vector<double> y;
    for (double xi : x) y.push_back(std::sin(xi));

    // Lần 1
    figure(600, 400);
    plot(x, y, "b-", opts({{"linewidth","2"}}));
    title("Reproducibility Test");
    grid(true);
    savefig("test_repro_1.svg");

    // Lần 2 — hoàn toàn giống lần 1
    figure(600, 400);
    plot(x, y, "b-", opts({{"linewidth","2"}}));
    title("Reproducibility Test");
    grid(true);
    savefig("test_repro_2.svg");

    // So sánh nội dung byte-by-byte
    auto read_file = [](const std::string& p) {
        std::ifstream f(p);
        return std::string(std::istreambuf_iterator<char>(f),
                           std::istreambuf_iterator<char>());
    };

    std::string svg1 = read_file("test_repro_1.svg");
    std::string svg2 = read_file("test_repro_2.svg");

    assert(!svg1.empty() && !svg2.empty());
    assert(svg1 == svg2 &&
           "Same input must always produce identical SVG output (reproducible)");

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 5: opts() API — key-value map hoạt động
// ─────────────────────────────────────────────
void test_opts_api() {
    std::cout << "[TEST] test_opts_api ... ";

    std::vector<double> x = {1, 2, 3};
    std::vector<double> y = {1, 4, 9};

    // opts với nhiều key-value pairs
    figure(400, 300);
    plot(x, y, "b-", opts({
        {"linewidth", "3"},
        {"label",     "data"},
        {"alpha",     "0.8"}
    }));
    legend(true);
    savefig("test_opts.svg");
    assert(std::ifstream("test_opts.svg").good());

    // scatter với opts
    figure(400, 300);
    scatter(x, y, opts({
        {"color",  "green"},
        {"s",      "100"},
        {"marker", "o"},
        {"alpha",  "0.7"}
    }));
    savefig("test_opts_scatter.svg");
    assert(std::ifstream("test_opts_scatter.svg").good());

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// MAIN
// ─────────────────────────────────────────────
int main() {
    std::cout << "=== test_numerical ===\n";

    try {
        test_linspace();
        test_logspace();
        test_edge_cases();
        test_reproducibility();
        test_opts_api();
    } catch (const std::exception& e) {
        std::cerr << "EXCEPTION: " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "UNKNOWN EXCEPTION\n";
        return 1;
    }

    std::cout << "=== ALL NUMERICAL TESTS PASSED ===\n";
    return 0;
}
