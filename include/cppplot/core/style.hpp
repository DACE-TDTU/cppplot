/**
 * @file style.hpp
 * @brief Line, marker, and text styles for CppPlot
 */

#ifndef CPPPLOT_CORE_STYLE_HPP
#define CPPPLOT_CORE_STYLE_HPP

#include "color.hpp"
#include "types.hpp"
#include <string>
#include <regex>

namespace cppplot {

/**
 * @brief Line style definition
 */
struct LineStyle {
    std::string style = "-";    // "-", "--", "-.", ":", "none"
    double width = 1.5;
    Color color = Color::tab10(0);
    double alpha = 1.0;
    
    LineStyle() = default;
    
    LineStyle(const std::string& s, double w = 1.5, const Color& c = Color::tab10(0))
        : style(s), width(w), color(c) {}
    
    bool isVisible() const {
        return style != "none" && style != "" && width > 0 && alpha > 0;
    }
    
    // Convert style to SVG dash array
    std::string toDashArray() const {
        if (style == "-" || style == "solid") return "";
        if (style == "--" || style == "dashed") return "6,4";
        if (style == "-." || style == "dashdot") return "6,2,2,2";
        if (style == ":" || style == "dotted") return "2,2";
        return "";
    }
};

/**
 * @brief Marker style definition
 */
struct MarkerStyle {
    std::string marker = "none";  // "o", "s", "^", "v", "<", ">", "x", "+", "*", ".", "none"
    double size = 6.0;
    Color faceColor = Color::tab10(0);
    Color edgeColor = Color::tab10(0);
    double edgeWidth = 1.0;
    double alpha = 1.0;
    
    MarkerStyle() = default;
    
    MarkerStyle(const std::string& m, double s = 6.0, const Color& fc = Color::tab10(0))
        : marker(m), size(s), faceColor(fc), edgeColor(fc) {}
    
    bool isVisible() const {
        return marker != "none" && marker != "" && size > 0 && alpha > 0;
    }
    
    // Check if marker is filled
    bool isFilled() const {
        return marker == "o" || marker == "s" || marker == "^" || 
               marker == "v" || marker == "<" || marker == ">" ||
               marker == "d" || marker == "D" || marker == "h" ||
               marker == "H" || marker == "p" || marker == "8";
    }
};

/**
 * @brief Text style definition
 */
struct TextStyle {
    std::string fontFamily = "Arial, sans-serif";
    double fontSize = 12.0;
    std::string fontWeight = "normal";  // "normal", "bold"
    std::string fontStyle = "normal";   // "normal", "italic"
    Color color = Color::black();
    TextAnchor anchor = TextAnchor::Start;
    TextBaseline baseline = TextBaseline::Alphabetic;
    double rotation = 0.0;
    
    TextStyle() = default;
    
    TextStyle& setSize(double size) { fontSize = size; return *this; }
    TextStyle& setBold(bool bold = true) { 
        fontWeight = bold ? "bold" : "normal"; 
        return *this; 
    }
    TextStyle& setItalic(bool italic = true) { 
        fontStyle = italic ? "italic" : "normal"; 
        return *this; 
    }
    TextStyle& setColor(const Color& c) { color = c; return *this; }
    TextStyle& setAnchor(TextAnchor a) { anchor = a; return *this; }
    TextStyle& setRotation(double r) { rotation = r; return *this; }
    
    std::string anchorString() const {
        switch (anchor) {
            case TextAnchor::Start: return "start";
            case TextAnchor::Middle: return "middle";
            case TextAnchor::End: return "end";
            default: return "start";
        }
    }
    
    std::string baselineString() const {
        switch (baseline) {
            case TextBaseline::Top: return "text-before-edge";
            case TextBaseline::Middle: return "middle";
            case TextBaseline::Bottom: return "text-after-edge";
            case TextBaseline::Alphabetic: return "alphabetic";
            default: return "alphabetic";
        }
    }
};

/**
 * @brief Combined plot style (line + marker)
 */
struct PlotStyle {
    LineStyle line;
    MarkerStyle marker;
    std::string label;
    int zorder = 0;
    
    PlotStyle() = default;
    
    /**
     * @brief Parse matplotlib-style format string
     * 
     * Examples: "b-o", "r--", "g^", "k:", "m-."
     */
    static PlotStyle parse(const std::string& fmt, int colorIndex = 0) {
        PlotStyle style;
        
        // Default color from cycle
        Color defaultColor = Color::tab10(colorIndex);
        style.line.color = defaultColor;
        style.marker.faceColor = defaultColor;
        style.marker.edgeColor = defaultColor;
        
        if (fmt.empty()) {
            return style;
        }
        
        std::string remaining = fmt;
        
        // Parse color (single char at start)
        if (!remaining.empty()) {
            char c = remaining[0];
            if (c == 'b' || c == 'g' || c == 'r' || c == 'c' || 
                c == 'm' || c == 'y' || c == 'k' || c == 'w') {
                Color color = Color::fromChar(c);
                style.line.color = color;
                style.marker.faceColor = color;
                style.marker.edgeColor = color;
                remaining = remaining.substr(1);
            }
        }
        
        // Parse line style
        if (remaining.find("-.") != std::string::npos) {
            style.line.style = "-.";
            remaining = std::regex_replace(remaining, std::regex("-\\."), "");
        } else if (remaining.find("--") != std::string::npos) {
            style.line.style = "--";
            remaining = std::regex_replace(remaining, std::regex("--"), "");
        } else if (remaining.find(":") != std::string::npos) {
            style.line.style = ":";
            remaining = std::regex_replace(remaining, std::regex(":"), "");
        } else if (remaining.find("-") != std::string::npos) {
            style.line.style = "-";
            remaining = std::regex_replace(remaining, std::regex("-"), "");
        } else {
            style.line.style = "none"; // No line by default if marker specified
        }
        
        // Parse marker
        if (!remaining.empty()) {
            style.marker.marker = remaining;
            if (style.line.style == "none" && remaining.length() > 0) {
                // If only marker specified, we might want a line too
                // Check original format
                if (fmt.find('-') != std::string::npos || 
                    fmt.find(':') != std::string::npos) {
                    // Line was explicitly specified
                } else {
                    // Marker only
                    style.line.style = "none";
                }
            } else if (style.line.style != "none") {
                // Both line and marker
            }
        }
        
        // If line style was found but marker style not explicitly none
        if (style.line.style != "none" && style.marker.marker == "none") {
            // Just line, no marker - this is correct
        }
        
        return style;
    }
    
    PlotStyle& setColor(const Color& c) {
        line.color = c;
        marker.faceColor = c;
        marker.edgeColor = c;
        return *this;
    }
    
    PlotStyle& setLineWidth(double w) {
        line.width = w;
        return *this;
    }
    
    PlotStyle& setMarkerSize(double s) {
        marker.size = s;
        return *this;
    }
    
    PlotStyle& setLabel(const std::string& l) {
        label = l;
        return *this;
    }
    
    PlotStyle& setAlpha(double a) {
        line.alpha = a;
        marker.alpha = a;
        return *this;
    }
};

/**
 * @brief Grid style
 */
struct GridStyle {
    bool show = false;
    bool showMajor = true;
    bool showMinor = false;
    LineStyle majorStyle = LineStyle("-", 0.5, Color::gray(200));
    LineStyle minorStyle = LineStyle(":", 0.3, Color::gray(220));
    
    GridStyle() = default;
    
    GridStyle& setVisible(bool v) { show = v; return *this; }
    GridStyle& setColor(const Color& c) { 
        majorStyle.color = c; 
        minorStyle.color = c.lighter(); 
        return *this; 
    }
};

/**
 * @brief Axis style
 */
struct AxisStyle {
    bool visible = true;
    LineStyle lineStyle = LineStyle("-", 1.0, Color::black());
    bool showTicks = true;
    bool showTickLabels = true;
    double tickLength = 5.0;
    double tickWidth = 1.0;
    TextStyle labelStyle;
    TextStyle tickLabelStyle;
    
    AxisStyle() {
        labelStyle.fontSize = 12;
        tickLabelStyle.fontSize = 10;
    }
};

/**
 * @brief Legend style
 */
struct LegendStyle {
    bool visible = false;
    LegendPosition position = LegendPosition::Best;
    Color backgroundColor = Color::white().withAlpha(0.8);
    Color borderColor = Color::gray(200);
    double borderWidth = 1.0;
    TextStyle textStyle;
    double padding = 8.0;
    double spacing = 4.0;
    
    LegendStyle() {
        textStyle.fontSize = 10;
    }
};

/**
 * @brief Complete figure/axes style theme
 */
struct Theme {
    std::string name = "default";
    Color backgroundColor = Color::white();
    Color plotAreaColor = Color::white();
    AxisStyle xAxisStyle;
    AxisStyle yAxisStyle;
    GridStyle gridStyle;
    LegendStyle legendStyle;
    TextStyle titleStyle;
    std::vector<Color> colorCycle;
    
    Theme() {
        titleStyle.fontSize = 14;
        titleStyle.setBold();
        titleStyle.anchor = TextAnchor::Middle;
        
        // Default color cycle (tab10)
        for (int i = 0; i < 10; ++i) {
            colorCycle.push_back(Color::tab10(i));
        }
    }
    
    // Predefined themes
    static Theme defaultTheme() {
        return Theme();
    }
    
    static Theme dark() {
        Theme t;
        t.name = "dark";
        t.backgroundColor = Color(30, 30, 30);
        t.plotAreaColor = Color(40, 40, 40);
        t.xAxisStyle.lineStyle.color = Color::gray(150);
        t.yAxisStyle.lineStyle.color = Color::gray(150);
        t.xAxisStyle.labelStyle.color = Color::gray(200);
        t.yAxisStyle.labelStyle.color = Color::gray(200);
        t.xAxisStyle.tickLabelStyle.color = Color::gray(180);
        t.yAxisStyle.tickLabelStyle.color = Color::gray(180);
        t.gridStyle.majorStyle.color = Color::gray(60);
        t.titleStyle.color = Color::white();
        t.legendStyle.backgroundColor = Color(40, 40, 40, 230);
        t.legendStyle.borderColor = Color::gray(80);
        t.legendStyle.textStyle.color = Color::gray(200);
        return t;
    }
    
    static Theme seaborn() {
        Theme t;
        t.name = "seaborn";
        t.plotAreaColor = Color(234, 234, 242);
        t.gridStyle.show = true;
        t.gridStyle.majorStyle = LineStyle("-", 1.0, Color::white());
        t.xAxisStyle.lineStyle.style = "none";
        t.yAxisStyle.lineStyle.style = "none";
        return t;
    }
    
    static Theme ggplot() {
        Theme t;
        t.name = "ggplot";
        t.plotAreaColor = Color(235, 235, 235);
        t.gridStyle.show = true;
        t.gridStyle.majorStyle = LineStyle("-", 1.0, Color::white());
        return t;
    }
    
    // Get theme by name
    static Theme get(const std::string& name) {
        if (name == "dark") return dark();
        if (name == "seaborn") return seaborn();
        if (name == "ggplot") return ggplot();
        return defaultTheme();
    }
};

} // namespace cppplot

#endif // CPPPLOT_CORE_STYLE_HPP
