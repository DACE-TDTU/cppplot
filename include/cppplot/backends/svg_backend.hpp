/**
 * @file svg_backend.hpp
 * @brief SVG rendering backend
 */

#ifndef CPPPLOT_BACKENDS_SVG_BACKEND_HPP
#define CPPPLOT_BACKENDS_SVG_BACKEND_HPP

#include "backend.hpp"
#include "../core/utils.hpp"
#include <sstream>
#include <fstream>
#include <iomanip>
#include <stack>

namespace cppplot {

/**
 * @brief SVG rendering backend
 */
class SVGBackend : public Backend {
private:
    int width_ = 800;
    int height_ = 600;
    std::ostringstream content_;
    std::ostringstream defs_;
    int defCounter_ = 0;
    std::stack<std::string> clipIds_;
    
    std::string formatDouble(double v) const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2) << v;
        std::string s = ss.str();
        // Remove trailing zeros and unnecessary decimal point
        size_t dot = s.find('.');
        if (dot != std::string::npos) {
            size_t last = s.find_last_not_of('0');
            if (last == dot) {
                // All digits after decimal are zeros, remove decimal point too
                s = s.substr(0, dot);
            } else {
                s = s.substr(0, last + 1);
            }
        }
        return s;
    }
    
    std::string colorToSVG(const Color& c, double alpha = 1.0) const {
        if (c.a() == 0 || alpha == 0) return "none";
        return c.toHex();
    }
    
    std::string opacityAttr(const Color& c, double alpha = 1.0) const {
        double finalAlpha = c.aNorm() * alpha;
        if (finalAlpha >= 0.999) return "";
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2) << finalAlpha;
        return ss.str();
    }
    
    std::string lineStyleToSVG(const LineStyle& style) const {
        std::ostringstream ss;
        
        if (!style.isVisible()) {
            ss << "stroke=\"none\"";
            return ss.str();
        }
        
        ss << "stroke=\"" << colorToSVG(style.color) << "\"";
        ss << " stroke-width=\"" << formatDouble(style.width) << "\"";
        
        std::string opacity = opacityAttr(style.color, style.alpha);
        if (!opacity.empty()) {
            ss << " stroke-opacity=\"" << opacity << "\"";
        }
        
        std::string dash = style.toDashArray();
        if (!dash.empty()) {
            ss << " stroke-dasharray=\"" << dash << "\"";
        }
        
        ss << " stroke-linecap=\"round\" stroke-linejoin=\"round\"";
        
        return ss.str();
    }
    
    std::string fillToSVG(const Color& fill, double alpha = 1.0) const {
        std::ostringstream ss;
        
        if (fill.a() == 0) {
            ss << "fill=\"none\"";
        } else {
            ss << "fill=\"" << colorToSVG(fill) << "\"";
            std::string opacity = opacityAttr(fill, alpha);
            if (!opacity.empty()) {
                ss << " fill-opacity=\"" << opacity << "\"";
            }
        }
        
        return ss.str();
    }

public:
    SVGBackend() = default;
    
    SVGBackend(int width, int height) : width_(width), height_(height) {}
    
    void setSize(int width, int height) override {
        width_ = width;
        height_ = height;
    }
    
    int width() const override { return width_; }
    int height() const override { return height_; }
    
    void clear(const Color& color = Color::white()) override {
        content_.str("");
        content_.clear();
        defs_.str("");
        defs_.clear();
        defCounter_ = 0;
        
        if (color.a() > 0) {
            content_ << "<rect width=\"100%\" height=\"100%\" "
                     << fillToSVG(color) << "/>\n";
        }
    }
    
    void setClipRect(const Rect& rect) override {
        std::string id = "clip" + std::to_string(++defCounter_);
        
        defs_ << "<clipPath id=\"" << id << "\">\n";
        defs_ << "  <rect x=\"" << formatDouble(rect.x) << "\" "
              << "y=\"" << formatDouble(rect.y) << "\" "
              << "width=\"" << formatDouble(rect.width) << "\" "
              << "height=\"" << formatDouble(rect.height) << "\"/>\n";
        defs_ << "</clipPath>\n";
        
        content_ << "<g clip-path=\"url(#" << id << ")\">\n";
        clipIds_.push(id);
    }
    
    void clearClip() override {
        if (!clipIds_.empty()) {
            content_ << "</g>\n";
            clipIds_.pop();
        }
    }
    
    void drawLine(double x1, double y1, double x2, double y2,
                  const LineStyle& style) override {
        if (!style.isVisible()) return;
        
        content_ << "<line "
                 << "x1=\"" << formatDouble(x1) << "\" "
                 << "y1=\"" << formatDouble(y1) << "\" "
                 << "x2=\"" << formatDouble(x2) << "\" "
                 << "y2=\"" << formatDouble(y2) << "\" "
                 << lineStyleToSVG(style) << "/>\n";
    }
    
    void drawPolyline(const std::vector<Point>& points,
                      const LineStyle& style) override {
        if (points.size() < 2 || !style.isVisible()) return;
        
        content_ << "<polyline points=\"";
        for (size_t i = 0; i < points.size(); ++i) {
            if (i > 0) content_ << " ";
            content_ << formatDouble(points[i].x) << "," << formatDouble(points[i].y);
        }
        content_ << "\" fill=\"none\" " << lineStyleToSVG(style) << "/>\n";
    }
    
    void drawRect(double x, double y, double w, double h,
                  const Color& fill = Color::transparent(),
                  const LineStyle& stroke = LineStyle()) override {
        content_ << "<rect "
                 << "x=\"" << formatDouble(x) << "\" "
                 << "y=\"" << formatDouble(y) << "\" "
                 << "width=\"" << formatDouble(w) << "\" "
                 << "height=\"" << formatDouble(h) << "\" "
                 << fillToSVG(fill) << " "
                 << lineStyleToSVG(stroke) << "/>\n";
    }
    
    void drawCircle(double cx, double cy, double radius,
                    const Color& fill = Color::transparent(),
                    const LineStyle& stroke = LineStyle()) override {
        content_ << "<circle "
                 << "cx=\"" << formatDouble(cx) << "\" "
                 << "cy=\"" << formatDouble(cy) << "\" "
                 << "r=\"" << formatDouble(radius) << "\" "
                 << fillToSVG(fill) << " "
                 << lineStyleToSVG(stroke) << "/>\n";
    }
    
    void drawEllipse(double cx, double cy, double rx, double ry,
                     const Color& fill = Color::transparent(),
                     const LineStyle& stroke = LineStyle()) override {
        content_ << "<ellipse "
                 << "cx=\"" << formatDouble(cx) << "\" "
                 << "cy=\"" << formatDouble(cy) << "\" "
                 << "rx=\"" << formatDouble(rx) << "\" "
                 << "ry=\"" << formatDouble(ry) << "\" "
                 << fillToSVG(fill) << " "
                 << lineStyleToSVG(stroke) << "/>\n";
    }
    
    void drawPolygon(const std::vector<Point>& points,
                     const Color& fill = Color::transparent(),
                     const LineStyle& stroke = LineStyle()) override {
        if (points.empty()) return;
        
        content_ << "<polygon points=\"";
        for (size_t i = 0; i < points.size(); ++i) {
            if (i > 0) content_ << " ";
            content_ << formatDouble(points[i].x) << "," << formatDouble(points[i].y);
        }
        content_ << "\" " << fillToSVG(fill) << " " << lineStyleToSVG(stroke) << "/>\n";
    }
    
    void drawPath(const std::string& pathData,
                  const Color& fill = Color::transparent(),
                  const LineStyle& stroke = LineStyle()) override {
        content_ << "<path d=\"" << pathData << "\" "
                 << fillToSVG(fill) << " "
                 << lineStyleToSVG(stroke) << "/>\n";
    }
    
    void drawText(double x, double y, const std::string& text,
                  const TextStyle& style) override {
        content_ << "<text "
                 << "x=\"" << formatDouble(x) << "\" "
                 << "y=\"" << formatDouble(y) << "\" "
                 << "font-family=\"" << style.fontFamily << "\" "
                 << "font-size=\"" << formatDouble(style.fontSize) << "\" "
                 << "font-weight=\"" << style.fontWeight << "\" "
                 << "font-style=\"" << style.fontStyle << "\" "
                 << "fill=\"" << colorToSVG(style.color) << "\" "
                 << "text-anchor=\"" << style.anchorString() << "\" "
                 << "dominant-baseline=\"" << style.baselineString() << "\"";
        
        if (style.rotation != 0) {
            content_ << " transform=\"rotate(" << formatDouble(style.rotation) 
                     << " " << formatDouble(x) << " " << formatDouble(y) << ")\"";
        }
        
        content_ << ">" << escapeXML(text) << "</text>\n";
    }
    
    std::string render() const override {
        std::ostringstream svg;
        
        svg << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
            << "width=\"" << width_ << "\" "
            << "height=\"" << height_ << "\" "
            << "viewBox=\"0 0 " << width_ << " " << height_ << "\">\n";
        
        // Defs section
        std::string defsStr = defs_.str();
        if (!defsStr.empty()) {
            svg << "<defs>\n" << defsStr << "</defs>\n";
        }
        
        // Content
        svg << content_.str();
        
        svg << "</svg>";
        
        return svg.str();
    }
    
    void save(const std::string& filename) const override {
        std::ofstream file(filename);
        if (!file) {
            throw std::runtime_error("Cannot open file for writing: " + filename);
        }
        file << render();
    }
    
    // Additional SVG-specific methods
    void addRawSVG(const std::string& svg) {
        content_ << svg << "\n";
    }
    
    void startGroup(const std::string& attrs = "") {
        content_ << "<g";
        if (!attrs.empty()) content_ << " " << attrs;
        content_ << ">\n";
    }
    
    void endGroup() {
        content_ << "</g>\n";
    }
    
    // Add gradient definition
    std::string addLinearGradient(const Color& startColor, const Color& endColor,
                                   double x1 = 0, double y1 = 0,
                                   double x2 = 1, double y2 = 0) {
        std::string id = "grad" + std::to_string(++defCounter_);
        
        defs_ << "<linearGradient id=\"" << id << "\" "
              << "x1=\"" << formatDouble(x1 * 100) << "%\" "
              << "y1=\"" << formatDouble(y1 * 100) << "%\" "
              << "x2=\"" << formatDouble(x2 * 100) << "%\" "
              << "y2=\"" << formatDouble(y2 * 100) << "%\">\n";
        defs_ << "  <stop offset=\"0%\" stop-color=\"" << startColor.toHex() << "\"/>\n";
        defs_ << "  <stop offset=\"100%\" stop-color=\"" << endColor.toHex() << "\"/>\n";
        defs_ << "</linearGradient>\n";
        
        return "url(#" + id + ")";
    }
};

} // namespace cppplot

#endif // CPPPLOT_BACKENDS_SVG_BACKEND_HPP
