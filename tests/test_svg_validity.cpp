/**
 * test_svg_validity.cpp
 * Tests SVGBackend low-level API + Figure/Axes high-level API.
 * API confirmed from test_svg.cpp:
 *   SVGBackend, Figure, Axes, TextStyle
 */
#include <cppplot/cppplot.hpp>
#include <iostream>
#include <cassert>
#include <fstream>
#include <cmath>
#include <stdexcept>

using namespace cppplot;

int tests_passed = 0, tests_failed = 0;

#define RUN_TEST(name) do { \
    std::cout << "Running " #name "... "; \
    try { test_##name(); std::cout << "PASSED\n"; tests_passed++; } \
    catch (const std::exception& e) { \
        std::cout << "FAILED: " << e.what() << "\n"; tests_failed++; } \
} while(0)

#define ASSERT_TRUE(x)  do { if(!(x))  throw std::runtime_error("TRUE failed: "  #x); } while(0)
#define ASSERT_FALSE(x) do { if( (x))  throw std::runtime_error("FALSE failed: " #x); } while(0)
#define ASSERT_EQ(a,b)  do { if((a)!=(b)) throw std::runtime_error("EQ failed: " #a); } while(0)
#define ASSERT_CONTAINS(s,sub) do { \
    if((s).find(sub)==std::string::npos) \
        throw std::runtime_error(std::string("CONTAINS failed: '") + (sub) + "'"); \
} while(0)

static std::string read_file(const std::string& p) {
    std::ifstream f(p);
    ASSERT_TRUE(f.good());
    return std::string(std::istreambuf_iterator<char>(f),
                       std::istreambuf_iterator<char>());
}

// ═══════════════════════════════════════════
// SVGBackend — low-level rendering
// ═══════════════════════════════════════════

void test_svgbackend_header() {
    SVGBackend svg(400, 300);
    svg.clear(Color::white());
    std::string out = svg.render();

    ASSERT_CONTAINS(out, "<?xml");
    ASSERT_CONTAINS(out, "<svg");
    ASSERT_CONTAINS(out, "width=\"400\"");
    ASSERT_CONTAINS(out, "height=\"300\"");
    ASSERT_CONTAINS(out, "</svg>");
}

void test_svgbackend_line() {
    SVGBackend svg(400, 300);
    svg.clear(Color::white());
    svg.drawLine(10, 20, 100, 200, LineStyle("-", 2, Color::red()));
    std::string out = svg.render();

    ASSERT_CONTAINS(out, "<line");
    ASSERT_CONTAINS(out, "x1=\"10\"");
    ASSERT_CONTAINS(out, "y1=\"20\"");
    ASSERT_CONTAINS(out, "x2=\"100\"");
    ASSERT_CONTAINS(out, "y2=\"200\"");
}

void test_svgbackend_rect() {
    SVGBackend svg(400, 300);
    svg.clear(Color::white());
    svg.drawRect(50, 50, 100, 80,
                 Color::blue(),
                 LineStyle("-", 1, Color::black()));
    std::string out = svg.render();

    ASSERT_CONTAINS(out, "<rect");
    ASSERT_CONTAINS(out, "width=\"100\"");
    ASSERT_CONTAINS(out, "height=\"80\"");
}

void test_svgbackend_circle() {
    SVGBackend svg(400, 300);
    svg.clear(Color::white());
    svg.drawCircle(200, 150, 50, Color::green(),
                   LineStyle("-", 1, Color::black()));
    std::string out = svg.render();

    ASSERT_CONTAINS(out, "<circle");
    ASSERT_CONTAINS(out, "cx=\"200\"");
    ASSERT_CONTAINS(out, "cy=\"150\"");
    ASSERT_CONTAINS(out, "r=\"50\"");
}

void test_svgbackend_polyline() {
    SVGBackend svg(400, 300);
    svg.clear(Color::white());
    std::vector<Point> pts = {{10,10},{100,50},{200,30},{300,100}};
    svg.drawPolyline(pts, LineStyle("-", 2, Color::blue()));
    std::string out = svg.render();

    ASSERT_CONTAINS(out, "<polyline");
    ASSERT_CONTAINS(out, "points=");
}

void test_svgbackend_text() {
    SVGBackend svg(400, 300);
    svg.clear(Color::white());
    TextStyle ts;
    ts.fontSize = 14;
    ts.color = Color::black();
    svg.drawText(100, 100, "Hello World", ts);
    std::string out = svg.render();

    ASSERT_CONTAINS(out, "<text");
    ASSERT_CONTAINS(out, "Hello World");
}

void test_svgbackend_polygon() {
    SVGBackend svg(400, 300);
    svg.clear(Color::white());
    std::vector<Point> pts = {{100,50},{150,150},{50,150}};
    svg.drawPolygon(pts, Color::yellow(),
                    LineStyle("-", 1, Color::black()));
    std::string out = svg.render();
    ASSERT_CONTAINS(out, "<polygon");
}

void test_svgbackend_clip() {
    SVGBackend svg(400, 300);
    svg.clear(Color::white());
    svg.setClipRect(Rect(50, 50, 200, 150));
    svg.drawCircle(100, 100, 100, Color::blue());
    svg.clearClip();
    std::string out = svg.render();

    ASSERT_CONTAINS(out, "<clipPath");
    ASSERT_CONTAINS(out, "clip-path=");
}

void test_svgbackend_sizes() {
    // SVG kích thước khác nhau → width/height attribute khác nhau
    struct TC { int w, h; };
    for (auto tc : std::vector<TC>{{200,150},{800,600},{1200,400}}) {
        SVGBackend svg(tc.w, tc.h);
        svg.clear(Color::white());
        std::string out = svg.render();
        ASSERT_CONTAINS(out, "width=\"" + std::to_string(tc.w) + "\"");
        ASSERT_CONTAINS(out, "height=\"" + std::to_string(tc.h) + "\"");
    }
}

void test_svgbackend_multiple_elements() {
    SVGBackend svg(400, 300);
    svg.clear(Color::white());
    // Nhiều element → SVG lớn hơn SVG trống
    std::string empty_out = svg.render();

    svg.drawLine(0,0,100,100, LineStyle("-",1,Color::red()));
    svg.drawLine(100,0,0,100, LineStyle("-",1,Color::blue()));
    svg.drawCircle(200,150,30, Color::green());
    std::string full_out = svg.render();

    ASSERT_TRUE(full_out.size() > empty_out.size());
}

// ═══════════════════════════════════════════
// Figure + Axes — high-level API
// ═══════════════════════════════════════════

void test_figure_creates_svg() {
    Figure fig(800, 600);
    auto& ax = fig.gca();

    std::vector<double> x = {1,2,3,4,5};
    std::vector<double> y = {1,4,9,16,25};
    ax.plot(x, y, "b-");

    std::string svg = fig.toSVG();
    ASSERT_CONTAINS(svg, "<svg");
    ASSERT_CONTAINS(svg, "width=\"800\"");
    ASSERT_FALSE(svg.empty());
}

void test_figure_save_and_load() {
    Figure fig(400, 300);
    auto& ax = fig.gca();
    ax.plot({1.0,2.0,3.0}, {1.0,4.0,9.0}, "r-");
    ax.set_title("Save Test");

    std::string fname = "test_svg_save.svg";
    fig.savefig(fname);

    std::string content = read_file(fname);
    ASSERT_CONTAINS(content, "<svg");
    ASSERT_CONTAINS(content, "Save Test");
    std::remove(fname.c_str());
}

void test_figure_labels_in_svg() {
    Figure fig(600, 400);
    auto& ax = fig.gca();
    ax.plot({0.0,1.0,2.0}, {0.0,1.0,0.0}, "b-");
    ax.set_xlabel("Time (s)");
    ax.set_ylabel("Amplitude");
    ax.set_title("Signal Plot");

    std::string svg = fig.toSVG();
    ASSERT_CONTAINS(svg, "Time (s)");
    ASSERT_CONTAINS(svg, "Amplitude");
    ASSERT_CONTAINS(svg, "Signal Plot");
}

void test_figure_subplot_count() {
    Figure fig(800, 600);
    fig.subplot(2, 2, 1);
    fig.subplot(2, 2, 2);
    fig.subplot(2, 2, 3);
    fig.subplot(2, 2, 4);

    ASSERT_TRUE(fig.getAxes().size() == 4);
}

void test_figure_has_polyline_for_line_plot() {
    Figure fig(600, 400);
    auto& ax = fig.gca();
    auto x = linspace(0.0, 2*M_PI, 50);
    std::vector<double> y;
    for (double xi : x) y.push_back(std::sin(xi));
    ax.plot(x, y, "b-");

    std::string svg = fig.toSVG();
    // Line plot → phải có polyline hoặc path
    bool has_line = (svg.find("<polyline") != std::string::npos) ||
                    (svg.find("<path")     != std::string::npos);
    ASSERT_TRUE(has_line);
}

void test_axes_all_plot_types() {
    Figure fig(800, 600);
    auto& ax = fig.gca();

    // plot
    ax.plot({1.0,2.0,3.0}, {1.0,4.0,9.0}, "b-o");
    // grid và legend
    ax.grid(true);
    ax.legend(true);
    ax.set_xlim(0, 4);
    ax.set_ylim(0, 10);

    std::string svg = fig.toSVG();
    ASSERT_FALSE(svg.empty());
}

// ═══════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════

int main() {
    std::cout << "CppPlot SVG Validity Tests\n";
    std::cout << "===========================\n\n";

    // Low-level SVGBackend
    RUN_TEST(svgbackend_header);
    RUN_TEST(svgbackend_line);
    RUN_TEST(svgbackend_rect);
    RUN_TEST(svgbackend_circle);
    RUN_TEST(svgbackend_polyline);
    RUN_TEST(svgbackend_text);
    RUN_TEST(svgbackend_polygon);
    RUN_TEST(svgbackend_clip);
    RUN_TEST(svgbackend_sizes);
    RUN_TEST(svgbackend_multiple_elements);

    // High-level Figure + Axes
    RUN_TEST(figure_creates_svg);
    RUN_TEST(figure_save_and_load);
    RUN_TEST(figure_labels_in_svg);
    RUN_TEST(figure_subplot_count);
    RUN_TEST(figure_has_polyline_for_line_plot);
    RUN_TEST(axes_all_plot_types);

    std::cout << "\n===========================\n";
    std::cout << "Passed: " << tests_passed << "\n";
    std::cout << "Failed: " << tests_failed << "\n";
    return tests_failed > 0 ? 1 : 0;
}
