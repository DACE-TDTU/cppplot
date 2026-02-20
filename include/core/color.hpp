/**
 * @file color.hpp
 * @brief Color utilities for CppPlot
 */

#ifndef CPPPLOT_CORE_COLOR_HPP
#define CPPPLOT_CORE_COLOR_HPP

#include <cstdint>
#include <string>
#include <array>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <map>

namespace cppplot {

// C++14 compatible clamp function
template<typename T>
inline T clamp_value(T value, T minVal, T maxVal) {
    return value < minVal ? minVal : (value > maxVal ? maxVal : value);
}

/**
 * @brief RGBA Color class
 */
class Color {
private:
    uint8_t r_ = 0;
    uint8_t g_ = 0;
    uint8_t b_ = 0;
    uint8_t a_ = 255;

public:
    // Constructors
    Color() = default;
    
    Color(int r, int g, int b, int a = 255)
        : r_(static_cast<uint8_t>(clamp_value(r, 0, 255)))
        , g_(static_cast<uint8_t>(clamp_value(g, 0, 255)))
        , b_(static_cast<uint8_t>(clamp_value(b, 0, 255)))
        , a_(static_cast<uint8_t>(clamp_value(a, 0, 255))) {}
    
    // From normalized values (0.0 - 1.0)
    static Color fromNormalized(double r, double g, double b, double a = 1.0) {
        return Color(
            static_cast<uint8_t>(clamp_value(r, 0.0, 1.0) * 255),
            static_cast<uint8_t>(clamp_value(g, 0.0, 1.0) * 255),
            static_cast<uint8_t>(clamp_value(b, 0.0, 1.0) * 255),
            static_cast<uint8_t>(clamp_value(a, 0.0, 1.0) * 255)
        );
    }
    
    // From hex string (#RRGGBB or #RRGGBBAA)
    static Color fromHex(const std::string& hex) {
        std::string h = hex;
        if (h[0] == '#') h = h.substr(1);
        
        if (h.length() == 3) {
            // Short form #RGB
            h = std::string(2, h[0]) + std::string(2, h[1]) + std::string(2, h[2]);
        }
        
        int r = 0, g = 0, b = 0, a = 255;
        
        if (h.length() >= 6) {
            r = static_cast<int>(std::stoul(h.substr(0, 2), nullptr, 16));
            g = static_cast<int>(std::stoul(h.substr(2, 2), nullptr, 16));
            b = static_cast<int>(std::stoul(h.substr(4, 2), nullptr, 16));
        }
        if (h.length() == 8) {
            a = static_cast<int>(std::stoul(h.substr(6, 2), nullptr, 16));
        }
        
        return Color(r, g, b, a);
    }
    
    // From HSV (h: 0-360, s: 0-1, v: 0-1)
    static Color fromHSV(double h, double s, double v, double a = 1.0) {
        double c = v * s;
        double x = c * (1 - std::abs(std::fmod(h / 60.0, 2) - 1));
        double m = v - c;
        
        double r = 0, g = 0, b = 0;
        
        if (h < 60) { r = c; g = x; b = 0; }
        else if (h < 120) { r = x; g = c; b = 0; }
        else if (h < 180) { r = 0; g = c; b = x; }
        else if (h < 240) { r = 0; g = x; b = c; }
        else if (h < 300) { r = x; g = 0; b = c; }
        else { r = c; g = 0; b = x; }
        
        return fromNormalized(r + m, g + m, b + m, a);
    }
    
    // Named color
    static Color fromName(const std::string& name) {
        static const std::map<std::string, Color> namedColors = {
            {"red", Color(255, 0, 0)},
            {"green", Color(0, 128, 0)},
            {"blue", Color(0, 0, 255)},
            {"cyan", Color(0, 255, 255)},
            {"magenta", Color(255, 0, 255)},
            {"yellow", Color(255, 255, 0)},
            {"black", Color(0, 0, 0)},
            {"white", Color(255, 255, 255)},
            {"gray", Color(128, 128, 128)},
            {"grey", Color(128, 128, 128)},
            {"orange", Color(255, 165, 0)},
            {"purple", Color(128, 0, 128)},
            {"pink", Color(255, 192, 203)},
            {"brown", Color(165, 42, 42)},
            {"navy", Color(0, 0, 128)},
            {"teal", Color(0, 128, 128)},
            {"olive", Color(128, 128, 0)},
            {"maroon", Color(128, 0, 0)},
            {"lime", Color(0, 255, 0)},
            {"aqua", Color(0, 255, 255)},
            {"silver", Color(192, 192, 192)},
            {"steelblue", Color(70, 130, 180)},
            {"coral", Color(255, 127, 80)},
            {"salmon", Color(250, 128, 114)},
            {"gold", Color(255, 215, 0)},
            {"skyblue", Color(135, 206, 235)},
            {"violet", Color(238, 130, 238)},
            {"indigo", Color(75, 0, 130)},
            {"crimson", Color(220, 20, 60)},
            {"darkgreen", Color(0, 100, 0)},
            {"darkblue", Color(0, 0, 139)},
            {"darkred", Color(139, 0, 0)},
            {"lightgray", Color(211, 211, 211)},
            {"lightgrey", Color(211, 211, 211)},
            {"darkgray", Color(169, 169, 169)},
            {"darkgrey", Color(169, 169, 169)}
        };
        
        auto it = namedColors.find(name);
        if (it != namedColors.end()) {
            return it->second;
        }
        
        // Try parsing as hex if starts with #
        if (!name.empty() && name[0] == '#') {
            return fromHex(name);
        }
        
        return black(); // Default fallback
    }
    
    // Single character matplotlib color
    static Color fromChar(char c) {
        switch (c) {
            case 'b': return blue();
            case 'g': return green();
            case 'r': return red();
            case 'c': return cyan();
            case 'm': return magenta();
            case 'y': return yellow();
            case 'k': return black();
            case 'w': return white();
            default: return black();
        }
    }
    
    // Predefined colors
    static Color red() { return Color(255, 0, 0); }
    static Color green() { return Color(0, 128, 0); }
    static Color blue() { return Color(0, 0, 255); }
    static Color cyan() { return Color(0, 255, 255); }
    static Color magenta() { return Color(255, 0, 255); }
    static Color yellow() { return Color(255, 255, 0); }
    static Color black() { return Color(0, 0, 0); }
    static Color white() { return Color(255, 255, 255); }
    static Color gray(uint8_t level = 128) { return Color(level, level, level); }
    static Color transparent() { return Color(0, 0, 0, 0); }
    
    // Tab10 colormap (default matplotlib cycle)
    static Color tab10(int index) {
        static const std::array<Color, 10> colors = {{
            Color(31, 119, 180),   // tab:blue
            Color(255, 127, 14),   // tab:orange
            Color(44, 160, 44),    // tab:green
            Color(214, 39, 40),    // tab:red
            Color(148, 103, 189),  // tab:purple
            Color(140, 86, 75),    // tab:brown
            Color(227, 119, 194),  // tab:pink
            Color(127, 127, 127),  // tab:gray
            Color(188, 189, 34),   // tab:olive
            Color(23, 190, 207)    // tab:cyan
        }};
        return colors[index % 10];
    }
    
    // Accessors
    uint8_t r() const { return r_; }
    uint8_t g() const { return g_; }
    uint8_t b() const { return b_; }
    uint8_t a() const { return a_; }
    
    double rNorm() const { return r_ / 255.0; }
    double gNorm() const { return g_ / 255.0; }
    double bNorm() const { return b_ / 255.0; }
    double aNorm() const { return a_ / 255.0; }
    
    // Setters
    Color& setR(uint8_t r) { r_ = r; return *this; }
    Color& setG(uint8_t g) { g_ = g; return *this; }
    Color& setB(uint8_t b) { b_ = b; return *this; }
    Color& setA(uint8_t a) { a_ = a; return *this; }
    Color& setAlpha(double alpha) { 
        a_ = static_cast<uint8_t>(clamp_value(alpha, 0.0, 1.0) * 255);
        return *this;
    }
    
    // Output formats
    std::string toHex(bool includeAlpha = false) const {
        std::stringstream ss;
        ss << "#" << std::hex << std::setfill('0')
           << std::setw(2) << static_cast<int>(r_)
           << std::setw(2) << static_cast<int>(g_)
           << std::setw(2) << static_cast<int>(b_);
        if (includeAlpha) {
            ss << std::setw(2) << static_cast<int>(a_);
        }
        return ss.str();
    }
    
    std::string toRGB() const {
        std::stringstream ss;
        ss << "rgb(" << static_cast<int>(r_) << "," 
           << static_cast<int>(g_) << "," 
           << static_cast<int>(b_) << ")";
        return ss.str();
    }
    
    std::string toRGBA() const {
        std::stringstream ss;
        ss << "rgba(" << static_cast<int>(r_) << "," 
           << static_cast<int>(g_) << "," 
           << static_cast<int>(b_) << ","
           << std::fixed << std::setprecision(2) << aNorm() << ")";
        return ss.str();
    }
    
    // Color operations
    Color withAlpha(double alpha) const {
        Color c = *this;
        c.setAlpha(alpha);
        return c;
    }
    
    Color lighter(double factor = 0.2) const {
        return Color(
            std::min(255, static_cast<int>(r_ + (255 - r_) * factor)),
            std::min(255, static_cast<int>(g_ + (255 - g_) * factor)),
            std::min(255, static_cast<int>(b_ + (255 - b_) * factor)),
            a_
        );
    }
    
    Color darker(double factor = 0.2) const {
        return Color(
            static_cast<int>(r_ * (1 - factor)),
            static_cast<int>(g_ * (1 - factor)),
            static_cast<int>(b_ * (1 - factor)),
            a_
        );
    }
    
    // Comparison
    bool operator==(const Color& other) const {
        return r_ == other.r_ && g_ == other.g_ && 
               b_ == other.b_ && a_ == other.a_;
    }
    
    bool operator!=(const Color& other) const {
        return !(*this == other);
    }
};

/**
 * @brief Colormap for mapping values to colors
 */
class Colormap {
private:
    std::string name_;
    std::vector<Color> colors_;
    
public:
    Colormap() = default;
    
    Colormap(const std::string& name, std::vector<Color> colors)
        : name_(name), colors_(std::move(colors)) {}
    
    // Get color for normalized value (0.0 - 1.0)
    Color operator()(double value) const {
        if (colors_.empty()) return Color::black();
        
        value = clamp_value(value, 0.0, 1.0);
        double pos = value * (colors_.size() - 1);
        size_t idx = static_cast<size_t>(pos);
        double t = pos - idx;
        
        if (idx >= colors_.size() - 1) {
            return colors_.back();
        }
        
        // Linear interpolation between colors
        const Color& c1 = colors_[idx];
        const Color& c2 = colors_[idx + 1];
        
        return Color(
            static_cast<int>(c1.r() + t * (c2.r() - c1.r())),
            static_cast<int>(c1.g() + t * (c2.g() - c1.g())),
            static_cast<int>(c1.b() + t * (c2.b() - c1.b())),
            static_cast<int>(c1.a() + t * (c2.a() - c1.a()))
        );
    }
    
    const std::string& name() const { return name_; }
    
    // Predefined colormaps
    static Colormap viridis() {
        return Colormap("viridis", {
            Color(68, 1, 84),
            Color(72, 40, 120),
            Color(62, 73, 137),
            Color(49, 104, 142),
            Color(38, 130, 142),
            Color(31, 158, 137),
            Color(53, 183, 121),
            Color(109, 205, 89),
            Color(180, 222, 44),
            Color(253, 231, 37)
        });
    }
    
    static Colormap plasma() {
        return Colormap("plasma", {
            Color(13, 8, 135),
            Color(75, 3, 161),
            Color(125, 3, 168),
            Color(168, 34, 150),
            Color(203, 70, 121),
            Color(229, 107, 93),
            Color(248, 148, 65),
            Color(253, 195, 40),
            Color(240, 249, 33)
        });
    }
    
    static Colormap hot() {
        return Colormap("hot", {
            Color(0, 0, 0),
            Color(128, 0, 0),
            Color(255, 0, 0),
            Color(255, 128, 0),
            Color(255, 255, 0),
            Color(255, 255, 255)
        });
    }
    
    static Colormap cool() {
        return Colormap("cool", {
            Color(0, 255, 255),
            Color(255, 0, 255)
        });
    }
    
    static Colormap jet() {
        return Colormap("jet", {
            Color(0, 0, 128),
            Color(0, 0, 255),
            Color(0, 255, 255),
            Color(255, 255, 0),
            Color(255, 0, 0),
            Color(128, 0, 0)
        });
    }
    
    static Colormap grayscale() {
        return Colormap("gray", {
            Color(0, 0, 0),
            Color(255, 255, 255)
        });
    }
    
    static Colormap rainbow() {
        std::vector<Color> colors;
        for (int i = 0; i <= 360; i += 36) {
            colors.push_back(Color::fromHSV(i, 1.0, 1.0));
        }
        return Colormap("rainbow", colors);
    }
    
    // Get colormap by name
    static Colormap get(const std::string& name) {
        if (name == "viridis") return viridis();
        if (name == "plasma") return plasma();
        if (name == "hot") return hot();
        if (name == "cool") return cool();
        if (name == "jet") return jet();
        if (name == "gray" || name == "grayscale") return grayscale();
        if (name == "rainbow") return rainbow();
        return viridis(); // default
    }
};

} // namespace cppplot

#endif // CPPPLOT_CORE_COLOR_HPP
