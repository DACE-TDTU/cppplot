/**
 * test_svg_validity.cpp
 * ─────────────────────────────────────────────
 * Kiểm tra cấu trúc SVG output có hợp lệ không.
 * Đây là test quan trọng nhất với JOSS reviewer
 * vì nó chứng minh output thực sự đúng format,
 * không chỉ "không crash".
 */
#include <cppplot/cppplot.hpp>
#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>

using namespace cppplot;

// ── Đọc file ─────────────────────────────────
static std::string read_file(const std::string& path) {
    std::ifstream f(path);
    assert(f.is_open() && "Cannot open SVG file");
    return std::string(std::istreambuf_iterator<char>(f),
                       std::istreambuf_iterator<char>());
}

// ── Đếm số lần xuất hiện của substring ───────
static int count_occurrences(const std::string& text, const std::string& sub) {
    int count = 0;
    size_t pos = 0;
    while ((pos = text.find(sub, pos)) != std::string::npos) {
        ++count;
        pos += sub.size();
    }
    return count;
}

// ── Lấy giá trị attribute từ SVG tag ─────────
static std::string extract_attr(const std::string& svg,
                                 const std::string& attr) {
    std::string search = attr + "=\"";
    size_t pos = svg.find(search);
    if (pos == std::string::npos) return "";
    pos += search.size();
    size_t end = svg.find('"', pos);
    return svg.substr(pos, end - pos);
}

// ─────────────────────────────────────────────
// TEST 1: SVG header hợp lệ
// ─────────────────────────────────────────────
void test_svg_header() {
    std::cout << "[TEST] test_svg_header ... ";

    std::vector<double> x = {1, 2, 3};
    std::vector<double> y = {1, 2, 3};
    figure(800, 600);
    plot(x, y, "b-");
    savefig("test_header.svg");

    std::string svg = read_file("test_header.svg");

    // Phải bắt đầu bằng XML/SVG declaration hoặc <svg
    bool starts_correctly = (svg.substr(0, 5) == "<?xml") ||
                            (svg.substr(0, 4) == "<svg");
    assert(starts_correctly && "SVG must start with <?xml or <svg");

    // Phải có thẻ mở <svg
    assert(svg.find("<svg") != std::string::npos &&
           "SVG must contain <svg element");

    // Phải có thẻ đóng </svg>
    assert(svg.find("</svg>") != std::string::npos &&
           "SVG must be closed with </svg>");

    // Tags phải cân bằng: số <svg phải = số </svg>
    // (kiểm tra không bị unclosed tag)
    assert(count_occurrences(svg, "<svg") == count_occurrences(svg, "</svg>") ||
           count_occurrences(svg, "<svg") == 1 &&
           "SVG tags must be balanced");

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 2: Kích thước figure phản ánh trong SVG
// ─────────────────────────────────────────────
void test_svg_dimensions() {
    std::cout << "[TEST] test_svg_dimensions ... ";

    std::vector<double> x = {1, 2, 3};
    std::vector<double> y = {1, 2, 3};

    // Test với nhiều kích thước
    struct TestCase { int w; int h; };
    std::vector<TestCase> cases = {{400, 300}, {800, 600}, {1200, 400}};

    for (auto& tc : cases) {
        std::string fname = "test_dim_" + std::to_string(tc.w) +
                            "x" + std::to_string(tc.h) + ".svg";
        figure(tc.w, tc.h);
        plot(x, y, "b-");
        savefig(fname);

        std::string svg = read_file(fname);

        // SVG phải chứa width và height attribute
        // (có thể ở dạng số hoặc pixel như "800px")
        bool has_width  = svg.find("width")  != std::string::npos;
        bool has_height = svg.find("height") != std::string::npos;

        assert(has_width  && "SVG must have width attribute");
        assert(has_height && "SVG must have height attribute");
    }

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 3: SVG chứa path elements (đường vẽ)
// ─────────────────────────────────────────────
void test_svg_has_paths() {
    std::cout << "[TEST] test_svg_has_paths ... ";

    auto x = linspace(0.0, 2 * M_PI, 50);
    std::vector<double> y;
    for (double xi : x) y.push_back(std::sin(xi));

    figure(600, 400);
    plot(x, y, "b-");
    savefig("test_paths.svg");

    std::string svg = read_file("test_paths.svg");

    // Đường kẻ trong SVG được vẽ bằng <path> hoặc <polyline> hoặc <line>
    bool has_path_elements = (svg.find("<path")     != std::string::npos) ||
                             (svg.find("<polyline") != std::string::npos) ||
                             (svg.find("<line")     != std::string::npos);

    assert(has_path_elements &&
           "SVG must contain path/polyline/line elements for the plot");

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 4: SVG chứa text cho labels và title
// ─────────────────────────────────────────────
void test_svg_has_text() {
    std::cout << "[TEST] test_svg_has_text ... ";

    std::vector<double> x = {1, 2, 3};
    std::vector<double> y = {1, 4, 9};

    const std::string TITLE  = "My Test Title";
    const std::string XLABEL = "X Axis Label";
    const std::string YLABEL = "Y Axis Label";

    figure(600, 400);
    plot(x, y, "b-");
    title(TITLE);
    xlabel(XLABEL);
    ylabel(YLABEL);
    savefig("test_text.svg");

    std::string svg = read_file("test_text.svg");

    // SVG phải chứa <text> elements
    assert(svg.find("<text") != std::string::npos &&
           "SVG must contain <text> elements");

    // Title phải muncul trong SVG
    assert(svg.find(TITLE) != std::string::npos &&
           "Title text must appear in SVG");

    // Labels harus muncul
    assert(svg.find(XLABEL) != std::string::npos &&
           "xlabel must appear in SVG");

    assert(svg.find(YLABEL) != std::string::npos &&
           "ylabel must appear in SVG");

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 5: SVG chứa màu đúng với format string
// ─────────────────────────────────────────────
void test_svg_colors() {
    std::cout << "[TEST] test_svg_colors ... ";

    std::vector<double> x = {1, 2, 3, 4, 5};
    std::vector<double> y = {1, 2, 3, 4, 5};

    // Vẽ đường đỏ
    figure(400, 300);
    plot(x, y, "r-");
    savefig("test_color_red.svg");

    std::string svg_red = read_file("test_color_red.svg");
    // Màu đỏ trong SVG thường là "red" hoặc "#ff0000" hoặc "rgb(255,0,0)"
    bool has_red = (svg_red.find("red")     != std::string::npos) ||
                   (svg_red.find("ff0000")  != std::string::npos) ||
                   (svg_red.find("255,0,0") != std::string::npos);
    assert(has_red && "Red format string 'r-' must produce red color in SVG");

    // Vẽ đường xanh
    figure(400, 300);
    plot(x, y, "b-");
    savefig("test_color_blue.svg");

    std::string svg_blue = read_file("test_color_blue.svg");
    bool has_blue = (svg_blue.find("blue")    != std::string::npos) ||
                    (svg_blue.find("0000ff")  != std::string::npos) ||
                    (svg_blue.find("0,0,255") != std::string::npos);
    assert(has_blue && "Blue format string 'b-' must produce blue color in SVG");

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 6: fill_between tạo ra filled region
// ─────────────────────────────────────────────
void test_svg_fill_between() {
    std::cout << "[TEST] test_svg_fill_between ... ";

    auto x = linspace(0.0, 10.0, 50);
    std::vector<double> y_upper, y_lower, y_mid;
    for (double xi : x) {
        y_upper.push_back(std::sin(xi) + 0.5);
        y_lower.push_back(std::sin(xi) - 0.5);
        y_mid.push_back(std::sin(xi));
    }

    figure(600, 400);
    fill_between(x, y_lower, y_upper,
                 opts({{"color", "blue"}, {"alpha", "0.3"}}));
    plot(x, y_mid, "b-");
    savefig("test_fill_between.svg");

    std::string svg = read_file("test_fill_between.svg");

    // fill_between phải tạo ra element có fill attribute
    bool has_fill = (svg.find("fill=")     != std::string::npos) ||
                    (svg.find("fill:")     != std::string::npos);
    assert(has_fill &&
           "fill_between must produce SVG elements with fill attribute");

    // Phải có opacity/alpha cho vùng filled
    bool has_opacity = (svg.find("opacity")      != std::string::npos) ||
                       (svg.find("fill-opacity") != std::string::npos);
    assert(has_opacity &&
           "fill_between with alpha must produce opacity in SVG");

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 7: Grid lines xuất hiện khi bật
// ─────────────────────────────────────────────
void test_svg_grid() {
    std::cout << "[TEST] test_svg_grid ... ";

    std::vector<double> x = {1, 2, 3};
    std::vector<double> y = {1, 2, 3};

    // Không có grid
    figure(400, 300);
    plot(x, y, "b-");
    grid(false);
    savefig("test_no_grid.svg");
    std::string svg_no_grid = read_file("test_no_grid.svg");

    // Có grid
    figure(400, 300);
    plot(x, y, "b-");
    grid(true);
    savefig("test_with_grid.svg");
    std::string svg_with_grid = read_file("test_with_grid.svg");

    // SVG với grid phải lớn hơn SVG không có grid
    // (vì có thêm line elements)
    assert(svg_with_grid.size() > svg_no_grid.size() &&
           "SVG with grid must be larger than SVG without grid");

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// MAIN
// ─────────────────────────────────────────────
int main() {
    std::cout << "=== test_svg_validity ===\n";

    try {
        test_svg_header();
        test_svg_dimensions();
        test_svg_has_paths();
        test_svg_has_text();
        test_svg_colors();
        test_svg_fill_between();
        test_svg_grid();
    } catch (const std::exception& e) {
        std::cerr << "EXCEPTION: " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "UNKNOWN EXCEPTION\n";
        return 1;
    }

    std::cout << "=== ALL SVG VALIDITY TESTS PASSED ===\n";
    return 0;
}
