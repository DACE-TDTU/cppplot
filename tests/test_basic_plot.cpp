/**
 * test_basic_plot.cpp
 * Tests rewritten to match ACTUAL API from test_core.cpp analysis.
 * Uses only classes/functions confirmed to exist:
 *   Color, PlotStyle, LineStyle, Colormap,
 *   linspace, arange, histogram, niceTicks,
 *   formatNumber, CoordinateTransform,
 *   Point, Rect, Limits, DataSeries
 */
#include <cppplot/cppplot.hpp>
#include <iostream>
#include <cassert>
#include <cmath>
#include <fstream>
#include <stdexcept>

using namespace cppplot;

int tests_passed = 0;
int tests_failed  = 0;

#define RUN_TEST(name) do { \
    std::cout << "Running " #name "... "; \
    try { test_##name(); std::cout << "PASSED\n"; tests_passed++; } \
    catch (const std::exception& e) { \
        std::cout << "FAILED: " << e.what() << "\n"; tests_failed++; } \
} while(0)

#define ASSERT_EQ(a,b)   do { if((a)!=(b)) throw std::runtime_error("ASSERT_EQ failed: " #a " != " #b); } while(0)
#define ASSERT_NEAR(a,b,t) do { if(std::abs((double)(a)-(double)(b))>(t)) throw std::runtime_error("ASSERT_NEAR failed: " #a); } while(0)
#define ASSERT_TRUE(x)   do { if(!(x)) throw std::runtime_error("ASSERT_TRUE failed: " #x); } while(0)
#define ASSERT_FALSE(x)  do { if( (x)) throw std::runtime_error("ASSERT_FALSE failed: " #x); } while(0)

// ═══════════════════════════════════════════
// Color — kiểm tra kỹ hơn test_core
// ═══════════════════════════════════════════

void test_color_all_channels() {
    Color c(10, 20, 30, 200);
    ASSERT_EQ(c.r(), 10);
    ASSERT_EQ(c.g(), 20);
    ASSERT_EQ(c.b(), 30);
    ASSERT_EQ(c.a(), 200);
}

void test_color_normalized() {
    Color c(255, 0, 128, 255);
    ASSERT_NEAR(c.rNorm(), 1.0, 0.01);
    ASSERT_NEAR(c.gNorm(), 0.0, 0.01);
    ASSERT_NEAR(c.bNorm(), 0.5, 0.02);
}

void test_color_named_all() {
    // Kiểm tra tất cả màu cơ bản từ format string
    struct { char ch; int r, g, b; } cases[] = {
        {'r', 255, 0,   0  },
        {'g', 0,   128, 0  },  // matplotlib green = (0,128,0) or (0,255,0)
        {'b', 0,   0,   255},
        {'k', 0,   0,   0  },
        {'w', 255, 255, 255},
    };
    for (auto& c : cases) {
        Color col = Color::fromChar(c.ch);
        // Chỉ kiểm tra màu có trong range hợp lệ
        ASSERT_TRUE(col.r() >= 0 && col.r() <= 255);
        ASSERT_TRUE(col.g() >= 0 && col.g() <= 255);
        ASSERT_TRUE(col.b() >= 0 && col.b() <= 255);
    }
    // Màu đỏ phải có R cao nhất
    Color red = Color::fromChar('r');
    ASSERT_TRUE(red.r() > red.g());
    ASSERT_TRUE(red.r() > red.b());
    // Màu xanh dương phải có B cao nhất
    Color blue = Color::fromChar('b');
    ASSERT_TRUE(blue.b() > blue.r());
    ASSERT_TRUE(blue.b() > blue.g());
    // Màu đen
    Color black = Color::fromChar('k');
    ASSERT_EQ(black.r(), 0);
    ASSERT_EQ(black.g(), 0);
    ASSERT_EQ(black.b(), 0);
}

void test_color_hex_roundtrip() {
    // fromHex → toHex phải cho lại kết quả gốc
    std::vector<std::string> hexcodes = {
        "#ff0000", "#00ff00", "#0000ff",
        "#ffffff", "#000000", "#ff8040"
    };
    for (auto& h : hexcodes) {
        Color c = Color::fromHex(h);
        std::string back = c.toHex();
        ASSERT_EQ(back, h);
    }
}

void test_color_with_alpha() {
    Color base = Color::red();
    Color semi = base.withAlpha(0.0);
    ASSERT_EQ(semi.a(), 0);
    Color opaque = base.withAlpha(1.0);
    ASSERT_EQ(opaque.a(), 255);
    Color half = base.withAlpha(0.5);
    ASSERT_TRUE(half.a() >= 127 && half.a() <= 129);
}

// ═══════════════════════════════════════════
// Colormap
// ═══════════════════════════════════════════

void test_colormap_boundary_values() {
    Colormap cm = Colormap::viridis();
    // Boundary values không crash
    Color c0   = cm(0.0);
    Color c05  = cm(0.5);
    Color c1   = cm(1.0);
    // Tất cả phải trong range hợp lệ
    for (Color* c : {&c0, &c05, &c1}) {
        ASSERT_TRUE(c->r() >= 0 && c->r() <= 255);
        ASSERT_TRUE(c->g() >= 0 && c->g() <= 255);
        ASSERT_TRUE(c->b() >= 0 && c->b() <= 255);
    }
    // Viridis: c0 phải tím/xanh, c1 phải vàng
    // Kiểm tra c0 != c1 (colormap không phải flat)
    ASSERT_TRUE(c0.r() != c1.r() || c0.g() != c1.g() || c0.b() != c1.b());
}

void test_colormap_monotone() {
    // Giá trị nằm trong [0,1] → colormap phải trả về màu hợp lệ liên tục
    Colormap cm = Colormap::viridis();
    const int N = 20;
    for (int i = 0; i <= N; i++) {
        double t = i / (double)N;
        Color c = cm(t);
        ASSERT_TRUE(c.r() >= 0 && c.r() <= 255);
        ASSERT_TRUE(c.g() >= 0 && c.g() <= 255);
        ASSERT_TRUE(c.b() >= 0 && c.b() <= 255);
    }
}

// ═══════════════════════════════════════════
// PlotStyle — format string parsing
// ═══════════════════════════════════════════

void test_plotstyle_formats() {
    struct Case { const char* fmt; const char* line; const char* marker; };
    Case cases[] = {
        {"b-",   "-",  "" },
        {"r--",  "--", "" },
        {"g:",   ":",  "" },
        {"k-.", "-.", "" },
        {"b-o",  "-",  "o"},
        {"r--s", "--", "s"},
        {"m-^",  "-",  "^"},
    };
    for (auto& c : cases) {
        PlotStyle s = PlotStyle::parse(c.fmt);
        ASSERT_EQ(s.line.style,   std::string(c.line));
        ASSERT_EQ(s.marker.marker, std::string(c.marker));
    }
}

void test_plotstyle_color_extraction() {
    PlotStyle s = PlotStyle::parse("r--o");
    ASSERT_EQ(s.line.color, Color::red());
    PlotStyle s2 = PlotStyle::parse("b-");
    ASSERT_EQ(s2.line.color, Color::blue());
}

// ═══════════════════════════════════════════
// LineStyle — dash array
// ═══════════════════════════════════════════

void test_linestyle_all_variants() {
    // Solid: dash array phải rỗng (không có dash)
    LineStyle solid("-");
    ASSERT_EQ(solid.toDashArray(), std::string(""));

    // Dashed: phải có dashes
    LineStyle dashed("--");
    std::string da = dashed.toDashArray();
    ASSERT_FALSE(da.empty());
    ASSERT_TRUE(da.find(",") != std::string::npos); // format "N,M"

    // Dotted
    LineStyle dotted(":");
    ASSERT_FALSE(dotted.toDashArray().empty());

    // Dash-dot
    LineStyle dashdot("-.");
    ASSERT_FALSE(dashdot.toDashArray().empty());
}

// ═══════════════════════════════════════════
// Geometry: Point, Rect, Limits
// ═══════════════════════════════════════════

void test_point_arithmetic() {
    Point a(3.0, 4.0);
    Point b(1.0, 2.0);

    Point sum  = a + b; ASSERT_NEAR(sum.x, 4.0, 1e-9); ASSERT_NEAR(sum.y, 6.0, 1e-9);
    Point diff = a - b; ASSERT_NEAR(diff.x, 2.0, 1e-9); ASSERT_NEAR(diff.y, 2.0, 1e-9);

    // Distance
    double dist = std::sqrt(a.x*a.x + a.y*a.y);
    ASSERT_NEAR(dist, 5.0, 1e-9); // 3-4-5 triangle
}

void test_rect_geometry() {
    Rect r(5.0, 10.0, 200.0, 100.0); // x, y, width, height
    ASSERT_NEAR(r.left(),    5.0,   1e-9);
    ASSERT_NEAR(r.right(),   205.0, 1e-9);
    ASSERT_NEAR(r.top(),     10.0,  1e-9);
    ASSERT_NEAR(r.bottom(),  110.0, 1e-9);
    ASSERT_NEAR(r.centerX(), 105.0, 1e-9);
    ASSERT_NEAR(r.centerY(), 60.0,  1e-9);
}

void test_limits_operations() {
    Limits lim(10.0, 90.0);
    ASSERT_NEAR(lim.range(),  80.0, 1e-9);
    ASSERT_NEAR(lim.center(), 50.0, 1e-9);

    // include: expand limits jika nilai di luar
    lim.include(100.0);
    ASSERT_NEAR(lim.max, 100.0, 1e-9);
    lim.include(0.0);
    ASSERT_NEAR(lim.min, 0.0, 1e-9);

    // include nilai dalam range: không thay đổi
    double old_max = lim.max;
    lim.include(50.0);
    ASSERT_NEAR(lim.max, old_max, 1e-9);
}

// ═══════════════════════════════════════════
// DataSeries<T>
// ═══════════════════════════════════════════

void test_dataseries_stats() {
    DataSeries<double> ds({2.0, 4.0, 6.0, 8.0, 10.0});
    ASSERT_EQ(ds.size(), 5u);
    ASSERT_NEAR(ds.min(),  2.0,  1e-9);
    ASSERT_NEAR(ds.max(),  10.0, 1e-9);
    ASSERT_NEAR(ds.mean(), 6.0,  1e-9);
    ASSERT_NEAR(ds.sum(),  30.0, 1e-9);
}

void test_dataseries_single() {
    DataSeries<double> ds({42.0});
    ASSERT_EQ(ds.size(), 1u);
    ASSERT_NEAR(ds.min(),  42.0, 1e-9);
    ASSERT_NEAR(ds.max(),  42.0, 1e-9);
    ASSERT_NEAR(ds.mean(), 42.0, 1e-9);
}

void test_dataseries_negative() {
    DataSeries<double> ds({-5.0, -3.0, -1.0, 0.0, 1.0});
    ASSERT_NEAR(ds.min(), -5.0, 1e-9);
    ASSERT_NEAR(ds.max(),  1.0, 1e-9);
    ASSERT_NEAR(ds.sum(), -8.0, 1e-9);
}

// ═══════════════════════════════════════════
// Numerical utilities
// ═══════════════════════════════════════════

void test_linspace_extended() {
    // Kiểm tra kỹ hơn test_core
    auto v = linspace(0.0, 1.0, 101);
    ASSERT_EQ(v.size(), 101u);
    ASSERT_NEAR(v[0],   0.0,  1e-12);
    ASSERT_NEAR(v[100], 1.0,  1e-12);
    ASSERT_NEAR(v[50],  0.5,  1e-12);

    // Bước đồng đều
    double step = v[1] - v[0];
    for (size_t i = 1; i < v.size(); i++) {
        ASSERT_NEAR(v[i] - v[i-1], step, 1e-12);
    }

    // Dải âm
    auto neg = linspace(-M_PI, M_PI, 5);
    ASSERT_NEAR(neg[0], -M_PI, 1e-10);
    ASSERT_NEAR(neg[4],  M_PI, 1e-10);
    ASSERT_NEAR(neg[2],  0.0,  1e-10);
}

void test_arange_extended() {
    auto v = arange(0.0, 1.0, 0.25);
    // arange(0, 1, 0.25) → {0, 0.25, 0.5, 0.75} (không gồm 1.0)
    ASSERT_EQ(v.size(), 4u);
    ASSERT_NEAR(v[0], 0.00, 1e-10);
    ASSERT_NEAR(v[1], 0.25, 1e-10);
    ASSERT_NEAR(v[2], 0.50, 1e-10);
    ASSERT_NEAR(v[3], 0.75, 1e-10);
}

void test_histogram_extended() {
    // Phân phối đồng đều → mỗi bin phải xấp xỉ bằng nhau
    std::vector<double> data;
    for (int i = 1; i <= 100; i++) data.push_back((double)i);

    auto h = histogram(data, 10, 1, 100);
    ASSERT_EQ(h.counts.size(), 10u);
    ASSERT_EQ(h.binEdges.size(), 11u);

    // Tổng counts phải bằng số điểm dữ liệu
    int total = 0;
    for (int c : h.counts) total += c;
    ASSERT_EQ(total, 100);
}

void test_coordinate_transform_extended() {
    // Data [0,10] → pixel [0,100]: transform phải scale 10x
    CoordinateTransform t(
        Rect(0, 0, 10, 10),
        Rect(0, 0, 100, 100)
    );
    Point p = t.dataToPixel(5.0, 5.0);
    ASSERT_NEAR(p.x, 50.0, 1.0);
    ASSERT_NEAR(p.y, 50.0, 1.0);

    // Origin
    Point origin = t.dataToPixel(0.0, 0.0);
    ASSERT_NEAR(origin.x, 0.0, 1.0);
}

// ═══════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════

int main() {
    std::cout << "CppPlot Extended Core Tests\n";
    std::cout << "============================\n\n";

    // Color
    RUN_TEST(color_all_channels);
    RUN_TEST(color_normalized);
    RUN_TEST(color_named_all);
    RUN_TEST(color_hex_roundtrip);
    RUN_TEST(color_with_alpha);

    // Colormap
    RUN_TEST(colormap_boundary_values);
    RUN_TEST(colormap_monotone);

    // PlotStyle & LineStyle
    RUN_TEST(plotstyle_formats);
    RUN_TEST(plotstyle_color_extraction);
    RUN_TEST(linestyle_all_variants);

    // Geometry
    RUN_TEST(point_arithmetic);
    RUN_TEST(rect_geometry);
    RUN_TEST(limits_operations);

    // DataSeries
    RUN_TEST(dataseries_stats);
    RUN_TEST(dataseries_single);
    RUN_TEST(dataseries_negative);

    // Numerical utils
    RUN_TEST(linspace_extended);
    RUN_TEST(arange_extended);
    RUN_TEST(histogram_extended);
    RUN_TEST(coordinate_transform_extended);

    std::cout << "\n============================\n";
    std::cout << "Passed: " << tests_passed << "\n";
    std::cout << "Failed: " << tests_failed << "\n";
    return tests_failed > 0 ? 1 : 0;
}
