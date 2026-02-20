/**
 * @file axes.hpp
 * @brief Axes class for plotting
 */

#ifndef CPPPLOT_AXES_HPP
#define CPPPLOT_AXES_HPP

#include "core/types.hpp"
#include "core/color.hpp"
#include "core/style.hpp"
#include "core/utils.hpp"
#include "core/latex.hpp"
#include "backends/backend.hpp"
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <limits>

namespace cppplot {

// Forward declaration
class Figure;

/**
 * @brief Plot element base class
 */
struct PlotElement {
    PlotType type;
    PlotStyle style;
    std::vector<double> xData;
    std::vector<double> yData;
    std::vector<std::string> labels;  // For bar charts
    std::vector<double> sizes;        // For scatter
    std::vector<Color> colors;        // For per-point colors
    PlotOptions options;
    
    virtual ~PlotElement() = default;
};

/**
 * @brief Text annotation element
 */
struct TextAnnotation {
    double x, y;
    std::string text;
    TextStyle style;
    bool dataCoords = true;    // true = data coordinates, false = axes fraction
    bool figureCoords = false; // true = figure coordinates (0-1), overrides dataCoords
    bool useLaTeX = false;     // true = render as LaTeX
    bool useMathML = false;    // true = use MathML (foreignObject), false = Unicode
};

/**
 * @brief Fill area element
 */
struct FillElement {
    std::vector<double> x;
    std::vector<double> y1;
    std::vector<double> y2;
    Color color;
    double alpha = 0.3;
    std::string label;
};

/**
 * @brief Error bar element
 */
struct ErrorBarElement {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> xerr;  // Optional x error
    std::vector<double> yerr;  // Optional y error
    PlotStyle style;
    double capsize = 3.0;
};

/**
 * @brief Axes class - represents a single plot area
 */
class Axes {
private:
    Rect position_;          // Position in figure (0-1 normalized)
    Limits xlim_;
    Limits ylim_;
    bool xlimSet_ = false;
    bool ylimSet_ = false;
    std::string xlabel_;
    std::string ylabel_;
    std::string title_;
    Theme theme_;
    GridStyle grid_;
    LegendStyle legend_;
    std::vector<std::shared_ptr<PlotElement>> elements_;
    std::vector<TextAnnotation> annotations_;
    std::vector<FillElement> fills_;
    std::vector<ErrorBarElement> errorBars_;
    int colorIndex_ = 0;
    
    // Scale type for axes
    std::string xscale_ = "linear";  // "linear" or "log"
    std::string yscale_ = "linear";
    
    // Margins for axis labels (in pixels)
    double marginLeft_ = 60;
    double marginRight_ = 20;
    double marginTop_ = 40;
    double marginBottom_ = 50;
    
    // Auto-calculate limits from data
    void autoLimits() {
        if (elements_.empty()) {
            xlim_ = Limits(0, 1);
            ylim_ = Limits(0, 1);
            return;
        }
        
        double xmin = std::numeric_limits<double>::max();
        double xmax = std::numeric_limits<double>::lowest();
        double ymin = std::numeric_limits<double>::max();
        double ymax = std::numeric_limits<double>::lowest();
        
        for (const auto& elem : elements_) {
            for (double x : elem->xData) {
                if (std::isfinite(x)) {
                    xmin = std::min(xmin, x);
                    xmax = std::max(xmax, x);
                }
            }
            for (double y : elem->yData) {
                if (std::isfinite(y)) {
                    ymin = std::min(ymin, y);
                    ymax = std::max(ymax, y);
                }
            }
        }
        
        // Add some padding
        double xpad = (xmax - xmin) * 0.05;
        double ypad = (ymax - ymin) * 0.05;
        
        if (xpad == 0) xpad = 0.5;
        if (ypad == 0) ypad = 0.5;
        
        if (!xlimSet_) {
            xlim_ = Limits(xmin - xpad, xmax + xpad);
        }
        if (!ylimSet_) {
            ylim_ = Limits(ymin - ypad, ymax + ypad);
        }
    }
    
    // Draw axis and labels
    void drawAxis(Backend& backend, const Rect& plotArea) {
        CoordinateTransform transform(
            Rect(xlim_.min, ylim_.min, xlim_.range(), ylim_.range()),
            plotArea
        );
        
        // Plot area background
        if (theme_.plotAreaColor != theme_.backgroundColor) {
            backend.drawRect(plotArea.x, plotArea.y, plotArea.width, plotArea.height,
                            theme_.plotAreaColor);
        }
        
        // Grid
        if (grid_.show) {
            auto xTicks = niceTicks(xlim_.min, xlim_.max);
            auto yTicks = niceTicks(ylim_.min, ylim_.max);
            
            for (double x : xTicks) {
                Point p1 = transform.dataToPixel(x, ylim_.min);
                Point p2 = transform.dataToPixel(x, ylim_.max);
                backend.drawLine(p1.x, p1.y, p2.x, p2.y, grid_.majorStyle);
            }
            
            for (double y : yTicks) {
                Point p1 = transform.dataToPixel(xlim_.min, y);
                Point p2 = transform.dataToPixel(xlim_.max, y);
                backend.drawLine(p1.x, p1.y, p2.x, p2.y, grid_.majorStyle);
            }
        }
        
        // Axis lines
        if (theme_.xAxisStyle.visible) {
            backend.drawLine(plotArea.left(), plotArea.bottom(),
                            plotArea.right(), plotArea.bottom(),
                            theme_.xAxisStyle.lineStyle);
        }
        if (theme_.yAxisStyle.visible) {
            backend.drawLine(plotArea.left(), plotArea.top(),
                            plotArea.left(), plotArea.bottom(),
                            theme_.yAxisStyle.lineStyle);
        }
        
        // X-axis ticks and labels
        if (theme_.xAxisStyle.showTicks) {
            auto xTicks = niceTicks(xlim_.min, xlim_.max);
            TextStyle tickStyle = theme_.xAxisStyle.tickLabelStyle;
            tickStyle.anchor = TextAnchor::Middle;
            tickStyle.baseline = TextBaseline::Top;
            
            for (double x : xTicks) {
                Point p = transform.dataToPixel(x, ylim_.min);
                
                // Tick mark
                backend.drawLine(p.x, plotArea.bottom(),
                                p.x, plotArea.bottom() + theme_.xAxisStyle.tickLength,
                                theme_.xAxisStyle.lineStyle);
                
                // Label
                if (theme_.xAxisStyle.showTickLabels) {
                    backend.drawText(p.x, plotArea.bottom() + theme_.xAxisStyle.tickLength + 4,
                                    formatNumber(x), tickStyle);
                }
            }
        }
        
        // Y-axis ticks and labels
        if (theme_.yAxisStyle.showTicks) {
            auto yTicks = niceTicks(ylim_.min, ylim_.max);
            TextStyle tickStyle = theme_.yAxisStyle.tickLabelStyle;
            tickStyle.anchor = TextAnchor::End;
            tickStyle.baseline = TextBaseline::Middle;
            
            for (double y : yTicks) {
                Point p = transform.dataToPixel(xlim_.min, y);
                
                // Tick mark
                backend.drawLine(plotArea.left() - theme_.yAxisStyle.tickLength,
                                p.y, plotArea.left(), p.y,
                                theme_.yAxisStyle.lineStyle);
                
                // Label
                if (theme_.yAxisStyle.showTickLabels) {
                    backend.drawText(plotArea.left() - theme_.yAxisStyle.tickLength - 4,
                                    p.y, formatNumber(y), tickStyle);
                }
            }
        }
        
        // X-axis label
        if (!xlabel_.empty()) {
            TextStyle style = theme_.xAxisStyle.labelStyle;
            style.anchor = TextAnchor::Middle;
            backend.drawText(plotArea.centerX(), plotArea.bottom() + 35, xlabel_, style);
        }
        
        // Y-axis label (rotated)
        if (!ylabel_.empty()) {
            TextStyle style = theme_.yAxisStyle.labelStyle;
            style.anchor = TextAnchor::Middle;
            style.rotation = -90;
            backend.drawText(plotArea.left() - 45, plotArea.centerY(), ylabel_, style);
        }
        
        // Title
        if (!title_.empty()) {
            TextStyle style = theme_.titleStyle;
            style.baseline = TextBaseline::Bottom;
            backend.drawText(plotArea.centerX(), plotArea.top() - 10, title_, style);
        }
    }
    
    // Draw plot elements
    void drawElements(Backend& backend, const Rect& plotArea) {
        CoordinateTransform transform(
            Rect(xlim_.min, ylim_.min, xlim_.range(), ylim_.range()),
            plotArea
        );
        
        // Set clip region
        backend.setClipRect(plotArea);
        
        for (const auto& elem : elements_) {
            switch (elem->type) {
                case PlotType::Line:
                    drawLinePlot(backend, transform, *elem);
                    break;
                case PlotType::Scatter:
                    drawScatterPlot(backend, transform, *elem);
                    break;
                case PlotType::Bar:
                    drawBarPlot(backend, transform, *elem);
                    break;
                case PlotType::Histogram:
                    drawHistogram(backend, transform, *elem);
                    break;
                default:
                    break;
            }
        }
        
        backend.clearClip();
    }
    
    void drawLinePlot(Backend& backend, const CoordinateTransform& transform,
                      const PlotElement& elem) {
        if (elem.xData.size() != elem.yData.size() || elem.xData.empty()) return;
        
        // Convert to pixel coordinates
        std::vector<Point> points;
        points.reserve(elem.xData.size());
        
        for (size_t i = 0; i < elem.xData.size(); ++i) {
            Point p = transform.dataToPixel(elem.xData[i], elem.yData[i]);
            points.push_back(p);
        }
        
        // Draw line
        if (elem.style.line.isVisible()) {
            backend.drawPolyline(points, elem.style.line);
        }
        
        // Draw markers
        if (elem.style.marker.isVisible()) {
            for (const auto& p : points) {
                backend.drawMarker(p.x, p.y, elem.style.marker);
            }
        }
    }
    
    void drawScatterPlot(Backend& backend, const CoordinateTransform& transform,
                         const PlotElement& elem) {
        if (elem.xData.size() != elem.yData.size() || elem.xData.empty()) return;
        
        for (size_t i = 0; i < elem.xData.size(); ++i) {
            Point p = transform.dataToPixel(elem.xData[i], elem.yData[i]);
            
            MarkerStyle style = elem.style.marker;
            
            // Per-point size
            if (i < elem.sizes.size()) {
                style.size = elem.sizes[i];
            }
            
            // Per-point color
            if (i < elem.colors.size()) {
                style.faceColor = elem.colors[i];
                style.edgeColor = elem.colors[i];
            }
            
            backend.drawMarker(p.x, p.y, style);
        }
    }
    
    void drawBarPlot(Backend& backend, const CoordinateTransform& transform,
                     const PlotElement& elem) {
        if (elem.xData.size() != elem.yData.size() || elem.xData.empty()) return;
        
        double barWidth = getOptionDouble(elem.options, "width", 0.8);
        
        // Calculate bar width in data coordinates
        double dataBarWidth = barWidth;
        if (elem.xData.size() > 1) {
            double minGap = std::abs(elem.xData[1] - elem.xData[0]);
            for (size_t i = 2; i < elem.xData.size(); ++i) {
                minGap = std::min(minGap, std::abs(elem.xData[i] - elem.xData[i-1]));
            }
            dataBarWidth = minGap * barWidth;
        }
        
        for (size_t i = 0; i < elem.xData.size(); ++i) {
            double x = elem.xData[i];
            double y = elem.yData[i];
            
            Point p1 = transform.dataToPixel(x - dataBarWidth/2, 0);
            Point p2 = transform.dataToPixel(x + dataBarWidth/2, y);
            
            double rectX = std::min(p1.x, p2.x);
            double rectY = std::min(p1.y, p2.y);
            double rectW = std::abs(p2.x - p1.x);
            double rectH = std::abs(p2.y - p1.y);
            
            Color fillColor = elem.style.line.color;
            if (i < elem.colors.size()) {
                fillColor = elem.colors[i];
            }
            
            backend.drawRect(rectX, rectY, rectW, rectH, fillColor, elem.style.line);
        }
    }
    
    void drawHistogram(Backend& backend, const CoordinateTransform& transform,
                       const PlotElement& elem) {
        // For histogram, xData contains bin edges, yData contains counts
        if (elem.xData.size() < 2 || elem.yData.empty()) return;
        
        for (size_t i = 0; i < elem.yData.size(); ++i) {
            double x1 = elem.xData[i];
            double x2 = elem.xData[i + 1];
            double y = elem.yData[i];
            
            Point p1 = transform.dataToPixel(x1, 0);
            Point p2 = transform.dataToPixel(x2, y);
            
            double rectX = std::min(p1.x, p2.x);
            double rectY = std::min(p1.y, p2.y);
            double rectW = std::abs(p2.x - p1.x);
            double rectH = std::abs(p2.y - p1.y);
            
            backend.drawRect(rectX, rectY, rectW, rectH, 
                           elem.style.line.color.withAlpha(0.7),
                           elem.style.line);
        }
    }
    
    // Draw legend
    void drawLegend(Backend& backend, const Rect& plotArea) {
        if (!legend_.visible) return;
        
        // Collect labeled elements
        std::vector<std::pair<std::string, PlotStyle>> legendItems;
        for (const auto& elem : elements_) {
            if (!elem->style.label.empty()) {
                legendItems.push_back(std::make_pair(elem->style.label, elem->style));
            }
        }
        
        if (legendItems.empty()) return;
        
        // Calculate legend size
        double itemHeight = legend_.textStyle.fontSize + legend_.spacing;
        double legendHeight = legendItems.size() * itemHeight + legend_.padding * 2;
        double legendWidth = 100; // TODO: calculate based on text
        
        // Position
        double lx = plotArea.right() - legendWidth - 10;
        double ly = plotArea.top() + 10;
        
        // Background
        backend.drawRect(lx, ly, legendWidth, legendHeight,
                        legend_.backgroundColor,
                        LineStyle("-", legend_.borderWidth, legend_.borderColor));
        
        // Items
        double y = ly + legend_.padding + legend_.textStyle.fontSize / 2;
        for (size_t i = 0; i < legendItems.size(); ++i) {
            const std::string& label = legendItems[i].first;
            const PlotStyle& style = legendItems[i].second;
            double x = lx + legend_.padding;
            
            // Line/marker sample
            if (style.line.isVisible()) {
                backend.drawLine(x, y, x + 20, y, style.line);
            }
            if (style.marker.isVisible()) {
                backend.drawMarker(x + 10, y, style.marker);
            }
            
            // Label
            TextStyle textStyle = legend_.textStyle;
            textStyle.anchor = TextAnchor::Start;
            textStyle.baseline = TextBaseline::Middle;
            backend.drawText(x + 25, y, label, textStyle);
            
            y += itemHeight;
        }
    }

public:
    Axes() : position_(0, 0, 1, 1) {}
    
    explicit Axes(const Rect& position) : position_(position) {}
    
    // Position
    void setPosition(const Rect& pos) { position_ = pos; }
    const Rect& position() const { return position_; }
    
    // Theme
    Axes& setTheme(const Theme& t) { theme_ = t; return *this; }
    Axes& setTheme(const std::string& name) { theme_ = Theme::get(name); return *this; }
    
    // Limits
    Axes& set_xlim(double min, double max) {
        xlim_ = Limits(min, max);
        xlimSet_ = true;
        return *this;
    }
    
    Axes& set_ylim(double min, double max) {
        ylim_ = Limits(min, max);
        ylimSet_ = true;
        return *this;
    }
    
    // Labels
    Axes& set_xlabel(const std::string& label) { xlabel_ = label; return *this; }
    Axes& set_ylabel(const std::string& label) { ylabel_ = label; return *this; }
    Axes& set_title(const std::string& t) { title_ = t; return *this; }
    
    // ============ Font Customization ============
    
    // Title font settings
    Axes& set_title_fontsize(double size) { theme_.titleStyle.fontSize = size; return *this; }
    Axes& set_title_fontfamily(const std::string& family) { theme_.titleStyle.fontFamily = family; return *this; }
    Axes& set_title_fontweight(const std::string& weight) { theme_.titleStyle.fontWeight = weight; return *this; }
    Axes& set_title_color(const Color& c) { theme_.titleStyle.color = c; return *this; }
    Axes& set_title_color(const std::string& c) { theme_.titleStyle.color = Color::fromName(c); return *this; }
    
    // X-axis label font settings
    Axes& set_xlabel_fontsize(double size) { theme_.xAxisStyle.labelStyle.fontSize = size; return *this; }
    Axes& set_xlabel_fontfamily(const std::string& family) { theme_.xAxisStyle.labelStyle.fontFamily = family; return *this; }
    Axes& set_xlabel_fontweight(const std::string& weight) { theme_.xAxisStyle.labelStyle.fontWeight = weight; return *this; }
    Axes& set_xlabel_color(const Color& c) { theme_.xAxisStyle.labelStyle.color = c; return *this; }
    Axes& set_xlabel_color(const std::string& c) { theme_.xAxisStyle.labelStyle.color = Color::fromName(c); return *this; }
    
    // Y-axis label font settings  
    Axes& set_ylabel_fontsize(double size) { theme_.yAxisStyle.labelStyle.fontSize = size; return *this; }
    Axes& set_ylabel_fontfamily(const std::string& family) { theme_.yAxisStyle.labelStyle.fontFamily = family; return *this; }
    Axes& set_ylabel_fontweight(const std::string& weight) { theme_.yAxisStyle.labelStyle.fontWeight = weight; return *this; }
    Axes& set_ylabel_color(const Color& c) { theme_.yAxisStyle.labelStyle.color = c; return *this; }
    Axes& set_ylabel_color(const std::string& c) { theme_.yAxisStyle.labelStyle.color = Color::fromName(c); return *this; }
    
    // X-axis tick label font settings
    Axes& set_xtick_fontsize(double size) { theme_.xAxisStyle.tickLabelStyle.fontSize = size; return *this; }
    Axes& set_xtick_fontfamily(const std::string& family) { theme_.xAxisStyle.tickLabelStyle.fontFamily = family; return *this; }
    Axes& set_xtick_fontweight(const std::string& weight) { theme_.xAxisStyle.tickLabelStyle.fontWeight = weight; return *this; }
    Axes& set_xtick_color(const Color& c) { theme_.xAxisStyle.tickLabelStyle.color = c; return *this; }
    Axes& set_xtick_color(const std::string& c) { theme_.xAxisStyle.tickLabelStyle.color = Color::fromName(c); return *this; }
    
    // Y-axis tick label font settings
    Axes& set_ytick_fontsize(double size) { theme_.yAxisStyle.tickLabelStyle.fontSize = size; return *this; }
    Axes& set_ytick_fontfamily(const std::string& family) { theme_.yAxisStyle.tickLabelStyle.fontFamily = family; return *this; }
    Axes& set_ytick_fontweight(const std::string& weight) { theme_.yAxisStyle.tickLabelStyle.fontWeight = weight; return *this; }
    Axes& set_ytick_color(const Color& c) { theme_.yAxisStyle.tickLabelStyle.color = c; return *this; }
    Axes& set_ytick_color(const std::string& c) { theme_.yAxisStyle.tickLabelStyle.color = Color::fromName(c); return *this; }
    
    // Both tick labels at once
    Axes& set_tick_fontsize(double size) { 
        set_xtick_fontsize(size); 
        set_ytick_fontsize(size); 
        return *this; 
    }
    Axes& set_tick_fontfamily(const std::string& family) { 
        set_xtick_fontfamily(family); 
        set_ytick_fontfamily(family); 
        return *this; 
    }
    
    // Both axis labels at once
    Axes& set_label_fontsize(double size) { 
        set_xlabel_fontsize(size); 
        set_ylabel_fontsize(size); 
        return *this; 
    }
    Axes& set_label_fontfamily(const std::string& family) { 
        set_xlabel_fontfamily(family); 
        set_ylabel_fontfamily(family); 
        return *this; 
    }
    
    // Legend font settings
    Axes& set_legend_fontsize(double size) { legend_.textStyle.fontSize = size; return *this; }
    Axes& set_legend_fontfamily(const std::string& family) { legend_.textStyle.fontFamily = family; return *this; }
    Axes& set_legend_fontweight(const std::string& weight) { legend_.textStyle.fontWeight = weight; return *this; }
    Axes& set_legend_color(const Color& c) { legend_.textStyle.color = c; return *this; }
    Axes& set_legend_color(const std::string& c) { legend_.textStyle.color = Color::fromName(c); return *this; }
    
    // Set all fonts at once
    Axes& set_fontfamily(const std::string& family) {
        set_title_fontfamily(family);
        set_label_fontfamily(family);
        set_tick_fontfamily(family);
        set_legend_fontfamily(family);
        return *this;
    }
    
    // Grid
    Axes& grid(bool show = true) { grid_.show = show; return *this; }
    Axes& setGrid(const GridStyle& g) { grid_ = g; return *this; }
    
    // Legend
    Axes& legend(bool show = true) { legend_.visible = show; return *this; }
    Axes& setLegend(const LegendStyle& l) { legend_ = l; return *this; }
    
    // ============ Plotting Methods ============
    
    /**
     * @brief Line plot
     */
    Axes& plot(const std::vector<double>& x, const std::vector<double>& y,
               const std::string& fmt = "-", const PlotOptions& opts = PlotOptions()) {
        auto elem = std::make_shared<PlotElement>();
        elem->type = PlotType::Line;
        elem->xData = x;
        elem->yData = y;
        elem->style = PlotStyle::parse(fmt, colorIndex_++);
        elem->options = opts;
        
        // Apply color option
        std::string colorOpt = getOptionStr(opts, "color");
        if (colorOpt.empty()) colorOpt = getOptionStr(opts, "c");
        if (!colorOpt.empty()) {
            elem->style.setColor(Color::fromName(colorOpt));
        }
        
        // Apply linestyle option
        std::string lsOpt = getOptionStr(opts, "linestyle");
        if (lsOpt.empty()) lsOpt = getOptionStr(opts, "ls");
        if (!lsOpt.empty()) {
            elem->style.line.style = lsOpt;
        }
        
        // Apply linewidth option
        double lw = getOptionDouble(opts, "linewidth", 0);
        if (lw == 0) lw = getOptionDouble(opts, "lw", 0);
        if (lw > 0) {
            elem->style.setLineWidth(lw);
        }
        
        // Apply marker option
        std::string markerOpt = getOptionStr(opts, "marker");
        if (!markerOpt.empty()) {
            elem->style.marker.marker = markerOpt;
        }
        
        // Apply markersize option
        double ms = getOptionDouble(opts, "markersize", 0);
        if (ms == 0) ms = getOptionDouble(opts, "ms", 0);
        if (ms > 0) {
            elem->style.setMarkerSize(ms);
        }
        
        // Apply markerfacecolor option
        std::string mfcOpt = getOptionStr(opts, "markerfacecolor");
        if (mfcOpt.empty()) mfcOpt = getOptionStr(opts, "mfc");
        if (!mfcOpt.empty()) {
            elem->style.marker.faceColor = Color::fromName(mfcOpt);
        }
        
        // Apply markeredgecolor option
        std::string mecOpt = getOptionStr(opts, "markeredgecolor");
        if (mecOpt.empty()) mecOpt = getOptionStr(opts, "mec");
        if (!mecOpt.empty()) {
            elem->style.marker.edgeColor = Color::fromName(mecOpt);
        }
        
        // Apply markeredgewidth option
        double mew = getOptionDouble(opts, "markeredgewidth", 0);
        if (mew == 0) mew = getOptionDouble(opts, "mew", 0);
        if (mew > 0) {
            elem->style.marker.edgeWidth = mew;
        }
        
        // Apply label option
        std::string label = getOptionStr(opts, "label");
        if (!label.empty()) {
            elem->style.setLabel(label);
        }
        
        // Apply alpha option
        double alpha = getOptionDouble(opts, "alpha", -1);
        if (alpha >= 0) {
            elem->style.setAlpha(alpha);
        }
        
        elements_.push_back(elem);
        return *this;
    }
    
    // Plot with y only (x = 0, 1, 2, ...)
    Axes& plot(const std::vector<double>& y, const std::string& fmt = "-",
               const PlotOptions& opts = PlotOptions()) {
        std::vector<double> x(y.size());
        std::iota(x.begin(), x.end(), 0.0);
        return plot(x, y, fmt, opts);
    }
    
    /**
     * @brief Scatter plot
     */
    Axes& scatter(const std::vector<double>& x, const std::vector<double>& y,
                  const PlotOptions& opts = PlotOptions()) {
        auto elem = std::make_shared<PlotElement>();
        elem->type = PlotType::Scatter;
        elem->xData = x;
        elem->yData = y;
        elem->style.line.style = "none";
        elem->style.marker.marker = "o";
        elem->style.marker.size = getOptionDouble(opts, "s", 36);
        elem->style.marker.faceColor = Color::tab10(colorIndex_);
        elem->style.marker.edgeColor = Color::tab10(colorIndex_);
        elem->options = opts;
        
        // Apply color option
        std::string colorOpt = getOptionStr(opts, "c");
        if (colorOpt.empty()) colorOpt = getOptionStr(opts, "color");
        if (!colorOpt.empty()) {
            Color c = Color::fromName(colorOpt);
            elem->style.marker.faceColor = c;
            elem->style.marker.edgeColor = c;
        }
        
        // Apply facecolor option (overrides color)
        std::string fcOpt = getOptionStr(opts, "facecolor");
        if (fcOpt.empty()) fcOpt = getOptionStr(opts, "fc");
        if (!fcOpt.empty()) {
            elem->style.marker.faceColor = Color::fromName(fcOpt);
        }
        
        // Apply edgecolor option
        std::string ecOpt = getOptionStr(opts, "edgecolor");
        if (ecOpt.empty()) ecOpt = getOptionStr(opts, "ec");
        if (!ecOpt.empty()) {
            elem->style.marker.edgeColor = Color::fromName(ecOpt);
        }
        
        // Apply linewidth for edge
        double lw = getOptionDouble(opts, "linewidth", 0);
        if (lw == 0) lw = getOptionDouble(opts, "lw", 0);
        if (lw > 0) {
            elem->style.marker.edgeWidth = lw;
        }
        
        // Apply marker option
        std::string marker = getOptionStr(opts, "marker");
        if (!marker.empty()) {
            elem->style.marker.marker = marker;
        }
        
        // Apply alpha option
        double alpha = getOptionDouble(opts, "alpha", -1);
        if (alpha >= 0) {
            elem->style.marker.alpha = alpha;
        }
        
        // Apply label option
        std::string label = getOptionStr(opts, "label");
        if (!label.empty()) {
            elem->style.setLabel(label);
        }
        
        colorIndex_++;
        elements_.push_back(elem);
        return *this;
    }
    
    /**
     * @brief Bar chart
     */
    Axes& bar(const std::vector<double>& x, const std::vector<double>& heights,
              const PlotOptions& opts = PlotOptions()) {
        auto elem = std::make_shared<PlotElement>();
        elem->type = PlotType::Bar;
        elem->xData = x;
        elem->yData = heights;
        elem->style.line.color = Color::tab10(colorIndex_);
        elem->style.line.width = 1;
        elem->options = opts;
        
        // Apply color/facecolor option
        std::string colorOpt = getOptionStr(opts, "color");
        if (colorOpt.empty()) colorOpt = getOptionStr(opts, "facecolor");
        if (colorOpt.empty()) colorOpt = getOptionStr(opts, "fc");
        if (!colorOpt.empty()) {
            elem->style.line.color = Color::fromName(colorOpt);
        }
        
        // Apply edgecolor option
        std::string edgeColor = getOptionStr(opts, "edgecolor");
        if (edgeColor.empty()) edgeColor = getOptionStr(opts, "ec");
        if (!edgeColor.empty()) {
            elem->style.marker.edgeColor = Color::fromName(edgeColor);
        }
        
        // Apply linewidth for edge
        double lw = getOptionDouble(opts, "linewidth", 0);
        if (lw == 0) lw = getOptionDouble(opts, "lw", 0);
        if (lw > 0) {
            elem->style.line.width = lw;
        }
        
        // Apply alpha option
        double alpha = getOptionDouble(opts, "alpha", -1);
        if (alpha >= 0) {
            elem->style.line.alpha = alpha;
        }
        
        // Apply label option
        std::string label = getOptionStr(opts, "label");
        if (!label.empty()) {
            elem->style.setLabel(label);
        }
        
        colorIndex_++;
        elements_.push_back(elem);
        return *this;
    }
    
    // Bar with string categories
    Axes& bar(const std::vector<std::string>& categories, const std::vector<double>& heights,
              const PlotOptions& opts = PlotOptions()) {
        std::vector<double> x(categories.size());
        std::iota(x.begin(), x.end(), 0.0);
        return bar(x, heights, opts);
    }
    
    /**
     * @brief Histogram
     */
    Axes& hist(const std::vector<double>& data, int bins = 10,
               const PlotOptions& opts = PlotOptions()) {
        auto result = histogram(data, bins);
        
        auto elem = std::make_shared<PlotElement>();
        elem->type = PlotType::Histogram;
        elem->xData = result.binEdges;
        elem->yData = result.counts;
        elem->style.line.color = Color::tab10(colorIndex_);
        elem->style.line.width = 1;
        elem->options = opts;
        
        std::string colorOpt = getOptionStr(opts, "color");
        if (!colorOpt.empty()) {
            elem->style.line.color = Color::fromName(colorOpt);
        }
        std::string label = getOptionStr(opts, "label");
        if (!label.empty()) {
            elem->style.setLabel(label);
        }
        
        colorIndex_++;
        elements_.push_back(elem);
        return *this;
    }
    
    // ============ NEW: Error Bars ============
    
    /**
     * @brief Plot with error bars
     * @param x X values
     * @param y Y values
     * @param yerr Y error values (symmetric)
     * @param opts Plot options
     */
    Axes& errorbar(const std::vector<double>& x, const std::vector<double>& y,
                   const std::vector<double>& yerr,
                   const PlotOptions& opts = PlotOptions()) {
        ErrorBarElement eb;
        eb.x = x;
        eb.y = y;
        eb.yerr = yerr;
        
        eb.style = PlotStyle::parse("-o", colorIndex_++);
        eb.capsize = getOptionDouble(opts, "capsize", 3.0);
        
        std::string colorOpt = getOptionStr(opts, "color");
        if (colorOpt.empty()) colorOpt = getOptionStr(opts, "c");
        if (!colorOpt.empty()) {
            eb.style.setColor(Color::fromName(colorOpt));
        }
        
        std::string label = getOptionStr(opts, "label");
        if (!label.empty()) {
            eb.style.setLabel(label);
        }
        
        errorBars_.push_back(eb);
        return *this;
    }
    
    /**
     * @brief Plot with asymmetric error bars
     */
    Axes& errorbar(const std::vector<double>& x, const std::vector<double>& y,
                   const std::vector<double>& xerr, const std::vector<double>& yerr,
                   const PlotOptions& opts = PlotOptions()) {
        ErrorBarElement eb;
        eb.x = x;
        eb.y = y;
        eb.xerr = xerr;
        eb.yerr = yerr;
        
        eb.style = PlotStyle::parse("-o", colorIndex_++);
        eb.capsize = getOptionDouble(opts, "capsize", 3.0);
        
        std::string colorOpt = getOptionStr(opts, "color");
        if (!colorOpt.empty()) {
            eb.style.setColor(Color::fromName(colorOpt));
        }
        
        errorBars_.push_back(eb);
        return *this;
    }
    
    // ============ NEW: Fill Between ============
    
    /**
     * @brief Fill between two y curves
     * @param x X values
     * @param y1 First Y curve
     * @param y2 Second Y curve
     * @param opts Plot options (color, alpha, label)
     */
    Axes& fill_between(const std::vector<double>& x, 
                       const std::vector<double>& y1,
                       const std::vector<double>& y2,
                       const PlotOptions& opts = PlotOptions()) {
        FillElement fill;
        fill.x = x;
        fill.y1 = y1;
        fill.y2 = y2;
        
        fill.color = Color::tab10(colorIndex_++);
        fill.alpha = getOptionDouble(opts, "alpha", 0.3);
        
        std::string colorOpt = getOptionStr(opts, "color");
        if (colorOpt.empty()) colorOpt = getOptionStr(opts, "facecolor");
        if (!colorOpt.empty()) {
            fill.color = Color::fromName(colorOpt);
        }
        
        fill.label = getOptionStr(opts, "label");
        
        fills_.push_back(fill);
        return *this;
    }
    
    /**
     * @brief Fill between y curve and constant value
     */
    Axes& fill_between(const std::vector<double>& x, 
                       const std::vector<double>& y,
                       double baseline = 0.0,
                       const PlotOptions& opts = PlotOptions()) {
        std::vector<double> y2(y.size(), baseline);
        return fill_between(x, y, y2, opts);
    }
    
    // ============ NEW: Text Annotations ============
    
    /**
     * @brief Add text annotation
     * @param x X position (data coordinates)
     * @param y Y position (data coordinates)
     * @param text Text content (supports LaTeX with $ delimiters)
     * @param opts Text style options
     */
    Axes& text(double x, double y, const std::string& text,
               const PlotOptions& opts = PlotOptions()) {
        TextAnnotation ann;
        ann.x = x;
        ann.y = y;
        ann.text = text;
        ann.dataCoords = true;
        ann.figureCoords = false;
        
        // Check if LaTeX rendering is requested
        ann.useLaTeX = (getOptionStr(opts, "usetex") == "true" || 
                        LaTeXRenderer::hasLaTeX(text));
        ann.useMathML = (getOptionStr(opts, "mathml") == "true");
        
        ann.style.fontSize = getOptionDouble(opts, "fontsize", 10);
        if (ann.style.fontSize == 0) ann.style.fontSize = 10;
        
        std::string fontfamily = getOptionStr(opts, "fontfamily");
        if (!fontfamily.empty()) ann.style.fontFamily = fontfamily;
        else if (ann.useLaTeX) ann.style.fontFamily = "STIX Two Math, Cambria Math, serif";
        
        std::string fontweight = getOptionStr(opts, "fontweight");
        if (!fontweight.empty()) ann.style.fontWeight = fontweight;
        
        std::string color = getOptionStr(opts, "color");
        if (!color.empty()) ann.style.color = Color::fromName(color);
        
        std::string ha = getOptionStr(opts, "ha");
        if (ha.empty()) ha = getOptionStr(opts, "horizontalalignment");
        if (ha == "center") ann.style.anchor = TextAnchor::Middle;
        else if (ha == "right") ann.style.anchor = TextAnchor::End;
        else ann.style.anchor = TextAnchor::Start;
        
        std::string va = getOptionStr(opts, "va");
        if (va.empty()) va = getOptionStr(opts, "verticalalignment");
        if (va == "center") ann.style.baseline = TextBaseline::Middle;
        else if (va == "top") ann.style.baseline = TextBaseline::Top;
        else if (va == "bottom") ann.style.baseline = TextBaseline::Bottom;
        
        double rotation = getOptionDouble(opts, "rotation", 0);
        ann.style.rotation = rotation;
        
        annotations_.push_back(ann);
        return *this;
    }
    
    /**
     * @brief Add text at figure coordinates (0-1 normalized)
     * @param x X position in figure (0=left, 1=right)
     * @param y Y position in figure (0=bottom, 1=top)
     * @param text Text content (supports LaTeX with $ delimiters)
     * @param opts Text style options
     */
    Axes& figtext(double x, double y, const std::string& text,
                  const PlotOptions& opts = PlotOptions()) {
        TextAnnotation ann;
        ann.x = x;
        ann.y = y;
        ann.text = text;
        ann.dataCoords = false;
        ann.figureCoords = true;
        
        // Check if LaTeX rendering is requested
        ann.useLaTeX = (getOptionStr(opts, "usetex") == "true" || 
                        LaTeXRenderer::hasLaTeX(text));
        ann.useMathML = (getOptionStr(opts, "mathml") == "true");
        
        ann.style.fontSize = getOptionDouble(opts, "fontsize", 12);
        if (ann.style.fontSize == 0) ann.style.fontSize = 12;
        
        std::string fontfamily = getOptionStr(opts, "fontfamily");
        if (!fontfamily.empty()) ann.style.fontFamily = fontfamily;
        else if (ann.useLaTeX) ann.style.fontFamily = "STIX Two Math, Cambria Math, serif";
        
        std::string fontweight = getOptionStr(opts, "fontweight");
        if (!fontweight.empty()) ann.style.fontWeight = fontweight;
        
        std::string color = getOptionStr(opts, "color");
        if (!color.empty()) ann.style.color = Color::fromName(color);
        
        std::string ha = getOptionStr(opts, "ha");
        if (ha.empty()) ha = getOptionStr(opts, "horizontalalignment");
        if (ha == "center") ann.style.anchor = TextAnchor::Middle;
        else if (ha == "right") ann.style.anchor = TextAnchor::End;
        else ann.style.anchor = TextAnchor::Start;
        
        std::string va = getOptionStr(opts, "va");
        if (va.empty()) va = getOptionStr(opts, "verticalalignment");
        if (va == "center") ann.style.baseline = TextBaseline::Middle;
        else if (va == "top") ann.style.baseline = TextBaseline::Top;
        else if (va == "bottom") ann.style.baseline = TextBaseline::Bottom;
        
        double rotation = getOptionDouble(opts, "rotation", 0);
        ann.style.rotation = rotation;
        
        annotations_.push_back(ann);
        return *this;
    }
    
    /**
     * @brief Add LaTeX formula at data coordinates
     * @param x X position (data coordinates)
     * @param y Y position (data coordinates)
     * @param latex LaTeX expression
     * @param opts Text style options
     */
    Axes& latex(double x, double y, const std::string& latexExpr,
                const PlotOptions& opts = PlotOptions()) {
        PlotOptions newOpts = opts;
        newOpts["usetex"] = "true";
        return text(x, y, latexExpr, newOpts);
    }
    
    /**
     * @brief Add annotation with arrow pointing to data point
     */
    Axes& annotate(const std::string& text, double x, double y,
                   double textX, double textY,
                   const PlotOptions& opts = PlotOptions()) {
        // Add text at textX, textY
        this->text(textX, textY, text, opts);
        
        // Arrow would go from (textX, textY) to (x, y)
        // For now, just add the text - arrow support TODO
        return *this;
    }
    
    // ============ NEW: Scale Methods ============
    
    /**
     * @brief Set x-axis scale
     * @param scale "linear" or "log"
     */
    Axes& set_xscale(const std::string& scale) {
        if (scale == "log" || scale == "linear") {
            xscale_ = scale;
        }
        return *this;
    }
    
    /**
     * @brief Set y-axis scale
     * @param scale "linear" or "log"
     */
    Axes& set_yscale(const std::string& scale) {
        if (scale == "log" || scale == "linear") {
            yscale_ = scale;
        }
        return *this;
    }
    
    // ============ NEW: Horizontal/Vertical Lines ============
    
    /**
     * @brief Add horizontal line across plot
     */
    Axes& axhline(double y, const PlotOptions& opts = PlotOptions()) {
        // Create a special element that spans full x range
        // We'll handle this in render by using current xlim
        auto elem = std::make_shared<PlotElement>();
        elem->type = PlotType::Line;
        elem->yData = {y, y};
        // xData will be set during render to span full width
        elem->style = PlotStyle::parse("--", colorIndex_++);
        elem->options = opts;
        elem->options["_axhline"] = "true";
        
        std::string colorOpt = getOptionStr(opts, "color");
        if (!colorOpt.empty()) elem->style.setColor(Color::fromName(colorOpt));
        
        double lw = getOptionDouble(opts, "linewidth", 1.0);
        if (lw == 0) lw = getOptionDouble(opts, "lw", 1.0);
        elem->style.setLineWidth(lw);
        
        elements_.push_back(elem);
        return *this;
    }
    
    /**
     * @brief Add vertical line across plot
     */
    Axes& axvline(double x, const PlotOptions& opts = PlotOptions()) {
        auto elem = std::make_shared<PlotElement>();
        elem->type = PlotType::Line;
        elem->xData = {x, x};
        elem->style = PlotStyle::parse("--", colorIndex_++);
        elem->options = opts;
        elem->options["_axvline"] = "true";
        
        std::string colorOpt = getOptionStr(opts, "color");
        if (!colorOpt.empty()) elem->style.setColor(Color::fromName(colorOpt));
        
        double lw = getOptionDouble(opts, "linewidth", 1.0);
        if (lw == 0) lw = getOptionDouble(opts, "lw", 1.0);
        elem->style.setLineWidth(lw);
        
        elements_.push_back(elem);
        return *this;
    }
    
    // Render to backend
    void render(Backend& backend, const Rect& figureArea) {
        // Calculate plot area in pixels
        double x = figureArea.x + position_.x * figureArea.width;
        double y = figureArea.y + position_.y * figureArea.height;
        double w = position_.width * figureArea.width;
        double h = position_.height * figureArea.height;
        
        Rect axesArea(x, y, w, h);
        Rect plotArea(
            x + marginLeft_,
            y + marginTop_,
            w - marginLeft_ - marginRight_,
            h - marginTop_ - marginBottom_
        );
        
        // Auto-calculate limits
        autoLimits();
        
        // Fix axhline/axvline elements with full range
        for (auto& elem : elements_) {
            if (getOptionBool(elem->options, "_axhline", false)) {
                elem->xData = {xlim_.min, xlim_.max};
            }
            if (getOptionBool(elem->options, "_axvline", false)) {
                elem->yData = {ylim_.min, ylim_.max};
            }
        }
        
        // Draw background
        backend.drawRect(axesArea.x, axesArea.y, axesArea.width, axesArea.height,
                        theme_.backgroundColor);
        
        // Draw axis, grid, labels
        drawAxis(backend, plotArea);
        
        // Draw fill elements first (behind other elements)
        drawFills(backend, plotArea);
        
        // Draw plot elements
        drawElements(backend, plotArea);
        
        // Draw error bars
        drawErrorBars(backend, plotArea);
        
        // Draw annotations (pass figureArea for figure coordinates)
        drawAnnotations(backend, plotArea, figureArea);
        
        // Draw legend
        drawLegend(backend, plotArea);
    }
    
    // Clear all elements
    void clear() {
        elements_.clear();
        annotations_.clear();
        fills_.clear();
        errorBars_.clear();
        colorIndex_ = 0;
    }

private:
    // Draw fill elements
    void drawFills(Backend& backend, const Rect& plotArea) {
        CoordinateTransform transform(
            Rect(xlim_.min, ylim_.min, xlim_.range(), ylim_.range()),
            plotArea
        );
        
        for (const auto& fill : fills_) {
            if (fill.x.empty()) continue;
            
            std::vector<Point> points;
            
            // First curve (forward)
            for (size_t i = 0; i < fill.x.size(); ++i) {
                double yVal = i < fill.y1.size() ? fill.y1[i] : 0;
                points.push_back(transform.dataToPixel(fill.x[i], yVal));
            }
            
            // Second curve (backward)
            for (size_t i = fill.x.size(); i > 0; --i) {
                size_t idx = i - 1;
                double yVal = idx < fill.y2.size() ? fill.y2[idx] : 0;
                points.push_back(transform.dataToPixel(fill.x[idx], yVal));
            }
            
            Color fillColor = fill.color.withAlpha(static_cast<uint8_t>(fill.alpha * 255));
            backend.drawPolygon(points, fillColor, LineStyle("none"));
        }
    }
    
    // Draw error bars
    void drawErrorBars(Backend& backend, const Rect& plotArea) {
        CoordinateTransform transform(
            Rect(xlim_.min, ylim_.min, xlim_.range(), ylim_.range()),
            plotArea
        );
        
        for (const auto& eb : errorBars_) {
            // Draw the main line/points
            if (!eb.x.empty() && !eb.y.empty()) {
                std::vector<Point> points;
                for (size_t i = 0; i < eb.x.size() && i < eb.y.size(); ++i) {
                    points.push_back(transform.dataToPixel(eb.x[i], eb.y[i]));
                }
                
                if (eb.style.line.isVisible()) {
                    backend.drawPolyline(points, eb.style.line);
                }
                
                if (eb.style.marker.isVisible()) {
                    for (const auto& p : points) {
                        backend.drawMarker(p.x, p.y, eb.style.marker);
                    }
                }
            }
            
            // Draw error bars
            LineStyle ebStyle = eb.style.line;
            ebStyle.style = "-";
            ebStyle.width = 1.0;
            
            for (size_t i = 0; i < eb.x.size(); ++i) {
                Point center = transform.dataToPixel(eb.x[i], eb.y[i]);
                
                // Y error bars
                if (i < eb.yerr.size() && eb.yerr[i] > 0) {
                    Point top = transform.dataToPixel(eb.x[i], eb.y[i] + eb.yerr[i]);
                    Point bot = transform.dataToPixel(eb.x[i], eb.y[i] - eb.yerr[i]);
                    
                    // Vertical line
                    backend.drawLine(center.x, top.y, center.x, bot.y, ebStyle);
                    
                    // Caps
                    if (eb.capsize > 0) {
                        backend.drawLine(center.x - eb.capsize, top.y, 
                                        center.x + eb.capsize, top.y, ebStyle);
                        backend.drawLine(center.x - eb.capsize, bot.y, 
                                        center.x + eb.capsize, bot.y, ebStyle);
                    }
                }
                
                // X error bars
                if (i < eb.xerr.size() && eb.xerr[i] > 0) {
                    Point left = transform.dataToPixel(eb.x[i] - eb.xerr[i], eb.y[i]);
                    Point right = transform.dataToPixel(eb.x[i] + eb.xerr[i], eb.y[i]);
                    
                    // Horizontal line
                    backend.drawLine(left.x, center.y, right.x, center.y, ebStyle);
                    
                    // Caps
                    if (eb.capsize > 0) {
                        backend.drawLine(left.x, center.y - eb.capsize,
                                        left.x, center.y + eb.capsize, ebStyle);
                        backend.drawLine(right.x, center.y - eb.capsize,
                                        right.x, center.y + eb.capsize, ebStyle);
                    }
                }
            }
        }
    }
    
    // Draw text annotations
    void drawAnnotations(Backend& backend, const Rect& plotArea, const Rect& figureArea) {
        CoordinateTransform transform(
            Rect(xlim_.min, ylim_.min, xlim_.range(), ylim_.range()),
            plotArea
        );
        
        for (const auto& ann : annotations_) {
            Point p;
            if (ann.figureCoords) {
                // Figure coordinates (0-1 normalized across whole figure)
                p.x = figureArea.x + ann.x * figureArea.width;
                p.y = figureArea.y + (1 - ann.y) * figureArea.height;
            } else if (ann.dataCoords) {
                // Data coordinates
                p = transform.dataToPixel(ann.x, ann.y);
            } else {
                // Axes fraction coordinates
                p.x = plotArea.x + ann.x * plotArea.width;
                p.y = plotArea.y + (1 - ann.y) * plotArea.height;
            }
            
            // Render text (with LaTeX support if needed)
            if (ann.useLaTeX) {
                // Convert LaTeX to Unicode
                std::string renderedText = LaTeXRenderer::toUnicode(ann.text);
                TextStyle style = ann.style;
                // Use math font for LaTeX
                if (style.fontFamily.empty() || style.fontFamily == "Arial, sans-serif") {
                    style.fontFamily = "STIX Two Math, Cambria Math, Latin Modern Math, serif";
                }
                backend.drawText(p.x, p.y, renderedText, style);
            } else {
                backend.drawText(p.x, p.y, ann.text, ann.style);
            }
        }
    }
};

} // namespace cppplot

#endif // CPPPLOT_AXES_HPP
