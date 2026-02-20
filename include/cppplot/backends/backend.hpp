/**
 * @file backend.hpp
 * @brief Abstract backend interface for rendering
 */

#ifndef CPPPLOT_BACKENDS_BACKEND_HPP
#define CPPPLOT_BACKENDS_BACKEND_HPP

// Define M_PI for Windows compatibility
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "../core/types.hpp"
#include "../core/color.hpp"
#include "../core/style.hpp"
#include <string>
#include <vector>
#include <cmath>

namespace cppplot {

/**
 * @brief Abstract rendering backend
 */
class Backend {
public:
    virtual ~Backend() = default;
    
    // Canvas setup
    virtual void setSize(int width, int height) = 0;
    virtual int width() const = 0;
    virtual int height() const = 0;
    
    // Background
    virtual void clear(const Color& color = Color::white()) = 0;
    
    // Clipping
    virtual void setClipRect(const Rect& rect) = 0;
    virtual void clearClip() = 0;
    
    // Primitives
    virtual void drawLine(double x1, double y1, double x2, double y2,
                          const LineStyle& style) = 0;
    
    virtual void drawPolyline(const std::vector<Point>& points,
                              const LineStyle& style) = 0;
    
    virtual void drawRect(double x, double y, double width, double height,
                          const Color& fill = Color::transparent(),
                          const LineStyle& stroke = LineStyle()) = 0;
    
    virtual void drawCircle(double cx, double cy, double radius,
                            const Color& fill = Color::transparent(),
                            const LineStyle& stroke = LineStyle()) = 0;
    
    virtual void drawEllipse(double cx, double cy, double rx, double ry,
                             const Color& fill = Color::transparent(),
                             const LineStyle& stroke = LineStyle()) = 0;
    
    virtual void drawPolygon(const std::vector<Point>& points,
                             const Color& fill = Color::transparent(),
                             const LineStyle& stroke = LineStyle()) = 0;
    
    virtual void drawPath(const std::string& pathData,
                          const Color& fill = Color::transparent(),
                          const LineStyle& stroke = LineStyle()) = 0;
    
    virtual void drawText(double x, double y, const std::string& text,
                          const TextStyle& style) = 0;
    
    // Markers
    virtual void drawMarker(double x, double y, const MarkerStyle& style);
    
    // Output
    virtual std::string render() const = 0;
    virtual void save(const std::string& filename) const = 0;
};

// Default implementation for drawMarker
inline void Backend::drawMarker(double x, double y, const MarkerStyle& style) {
    if (!style.isVisible()) return;
    
    double s = style.size / 2;
    Color fill = style.faceColor.withAlpha(style.alpha * style.faceColor.aNorm());
    LineStyle stroke;
    stroke.color = style.edgeColor;
    stroke.width = style.edgeWidth;
    stroke.alpha = style.alpha;
    
    const std::string& m = style.marker;
    
    if (m == "o" || m == "circle") {
        drawCircle(x, y, s, fill, stroke);
    }
    else if (m == "s" || m == "square") {
        drawRect(x - s, y - s, s * 2, s * 2, fill, stroke);
    }
    else if (m == "^" || m == "triangle_up") {
        std::vector<Point> pts = {
            {x, y - s},
            {x - s, y + s},
            {x + s, y + s}
        };
        drawPolygon(pts, fill, stroke);
    }
    else if (m == "v" || m == "triangle_down") {
        std::vector<Point> pts = {
            {x, y + s},
            {x - s, y - s},
            {x + s, y - s}
        };
        drawPolygon(pts, fill, stroke);
    }
    else if (m == "<" || m == "triangle_left") {
        std::vector<Point> pts = {
            {x - s, y},
            {x + s, y - s},
            {x + s, y + s}
        };
        drawPolygon(pts, fill, stroke);
    }
    else if (m == ">" || m == "triangle_right") {
        std::vector<Point> pts = {
            {x + s, y},
            {x - s, y - s},
            {x - s, y + s}
        };
        drawPolygon(pts, fill, stroke);
    }
    else if (m == "d" || m == "diamond") {
        std::vector<Point> pts = {
            {x, y - s},
            {x + s, y},
            {x, y + s},
            {x - s, y}
        };
        drawPolygon(pts, fill, stroke);
    }
    else if (m == "+" || m == "plus") {
        drawLine(x - s, y, x + s, y, stroke);
        drawLine(x, y - s, x, y + s, stroke);
    }
    else if (m == "x" || m == "cross") {
        double d = s * 0.707; // sqrt(2)/2
        drawLine(x - d, y - d, x + d, y + d, stroke);
        drawLine(x - d, y + d, x + d, y - d, stroke);
    }
    else if (m == "*" || m == "star") {
        // 5-pointed star
        for (int i = 0; i < 5; ++i) {
            double angle = i * 72 * M_PI / 180 - M_PI / 2;
            double x2 = x + s * std::cos(angle);
            double y2 = y + s * std::sin(angle);
            drawLine(x, y, x2, y2, stroke);
        }
    }
    else if (m == "." || m == "point") {
        drawCircle(x, y, style.size / 6, fill, stroke);
    }
    else if (m == "," || m == "pixel") {
        drawRect(x - 0.5, y - 0.5, 1, 1, fill, stroke);
    }
    else if (m == "h" || m == "hexagon") {
        std::vector<Point> pts;
        for (int i = 0; i < 6; ++i) {
            double angle = i * 60 * M_PI / 180;
            pts.push_back({x + s * std::cos(angle), y + s * std::sin(angle)});
        }
        drawPolygon(pts, fill, stroke);
    }
    else if (m == "p" || m == "pentagon") {
        std::vector<Point> pts;
        for (int i = 0; i < 5; ++i) {
            double angle = i * 72 * M_PI / 180 - M_PI / 2;
            pts.push_back({x + s * std::cos(angle), y + s * std::sin(angle)});
        }
        drawPolygon(pts, fill, stroke);
    }
}

} // namespace cppplot

#endif // CPPPLOT_BACKENDS_BACKEND_HPP
