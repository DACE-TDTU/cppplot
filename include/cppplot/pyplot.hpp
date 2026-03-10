/**
 * @file pyplot.hpp
 * @brief Matplotlib pyplot-style interface
 *
 * This provides a global state-based interface similar to matplotlib.pyplot,
 * allowing simple scripts like:
 *
 *     plot(x, y, "b-o");
 *     xlabel("X");
 *     ylabel("Y");
 *     savefig("plot.svg");
 */

#ifndef CPPPLOT_PYPLOT_HPP
#define CPPPLOT_PYPLOT_HPP

#include "axes.hpp"
#include "core/utils.hpp"
#include "figure.hpp"

namespace cppplot {

// ============ Global State ============

namespace detail {
inline Figure &getCurrentFigure() {
  static Figure fig;
  return fig;
}

inline int &figureCounter() {
  static int counter = 0;
  return counter;
}
} // namespace detail

// ============ Figure Functions ============

/**
 * @brief Create a new figure
 */
inline Figure &figure(int width = 800, int height = 600) {
  detail::getCurrentFigure() = Figure(width, height);
  detail::figureCounter()++;
  return detail::getCurrentFigure();
}

/**
 * @brief Get current figure
 */
inline Figure &gcf() { return detail::getCurrentFigure(); }

/**
 * @brief Get current axes
 */
inline Axes &gca() { return gcf().gca(); }

/**
 * @brief Create subplot
 */
inline Axes &subplot(int nrows, int ncols, int index) {
  return gcf().subplot(nrows, ncols, index);
}

/**
 * @brief Create subplot spanning multiple cells (Julia-style)
 * Usage: subplot(2, 3, {1, 2}) - spans cells 1 and 2
 */
inline Axes &subplot(int nrows, int ncols, std::initializer_list<int> indices) {
  return gcf().subplot(nrows, ncols, indices);
}

/**
 * @brief Create subplot at specific position (0-based)
 */
inline Axes &subplot_at(int row, int col) { return gcf().subplot_at(row, col); }

/**
 * @brief Create subplot spanning multiple cells
 * @param row1, col1 - Top-left (0-based)
 * @param row2, col2 - Bottom-right (0-based, inclusive)
 */
inline Axes &subplot_span(int row1, int col1, int row2, int col2) {
  return gcf().subplot_span(row1, col1, row2, col2);
}

/**
 * @brief Set figure layout
 */
inline Figure &layout(int nrows, int ncols) {
  return gcf().setLayout(nrows, ncols);
}

/**
 * @brief Set figure layout with GridSpec
 */
inline Figure &layout(const GridSpec &gs) { return gcf().setLayout(gs); }

/**
 * @brief Set figure layout with custom column widths
 */
inline Figure &layout(int nrows, int ncols,
                      std::initializer_list<double> widths) {
  return gcf().setLayout(nrows, ncols, widths);
}

/**
 * @brief Set figure layout with custom widths and heights
 */
inline Figure &layout(int nrows, int ncols,
                      std::initializer_list<double> widths,
                      std::initializer_list<double> heights) {
  return gcf().setLayout(nrows, ncols, widths, heights);
}

/**
 * @brief Add axes at specific position (0-1 normalized coordinates)
 */
inline Axes &add_axes(double x, double y, double width, double height) {
  return gcf().add_axes(x, y, width, height);
}

/**
 * @brief Add inset axes within current axes
 */
inline Axes &inset_axes(double x, double y, double w, double h) {
  return gcf().inset_axes(x, y, w, h);
}

/**
 * @brief Add a secondary Y-axis to current axes
 */
inline Axes &twinx() { return gca().twinx(); }

// ============ Plotting Functions ============

/**
 * @brief Line plot
 */
inline Axes &plot(const std::vector<double> &x, const std::vector<double> &y,
                  const std::string &fmt = "-",
                  const PlotOptions &opts = PlotOptions()) {
  return gca().plot(x, y, fmt, opts);
}

inline Axes &plot(const std::vector<double> &y, const std::string &fmt = "-",
                  const PlotOptions &opts = PlotOptions()) {
  return gca().plot(y, fmt, opts);
}

/**
 * @brief Scatter plot
 */
inline Axes &scatter(const std::vector<double> &x, const std::vector<double> &y,
                     const PlotOptions &opts = PlotOptions()) {
  return gca().scatter(x, y, opts);
}

/**
 * @brief Bar chart
 */
inline Axes &bar(const std::vector<double> &x,
                 const std::vector<double> &heights,
                 const PlotOptions &opts = PlotOptions()) {
  return gca().bar(x, heights, opts);
}

inline Axes &bar(const std::vector<std::string> &categories,
                 const std::vector<double> &heights,
                 const PlotOptions &opts = PlotOptions()) {
  return gca().bar(categories, heights, opts);
}

/**
 * @brief Histogram
 */
inline Axes &hist(const std::vector<double> &data, int bins = 10,
                  const PlotOptions &opts = PlotOptions()) {
  return gca().hist(data, bins, opts);
}

/**
 * @brief Box plot
 */
inline Axes &boxplot(const std::vector<std::vector<double>> &data,
                     const std::vector<double> &positions = {},
                     const PlotOptions &opts = PlotOptions()) {
  return gca().boxplot(data, positions, opts);
}

// ============ NEW: Error Bars ============

/**
 * @brief Plot with error bars
 */
inline Axes &errorbar(const std::vector<double> &x,
                      const std::vector<double> &y,
                      const std::vector<double> &yerr,
                      const PlotOptions &opts = PlotOptions()) {
  return gca().errorbar(x, y, yerr, opts);
}

/**
 * @brief Plot with both x and y error bars
 */
inline Axes &errorbar(const std::vector<double> &x,
                      const std::vector<double> &y,
                      const std::vector<double> &xerr,
                      const std::vector<double> &yerr,
                      const PlotOptions &opts = PlotOptions()) {
  return gca().errorbar(x, y, xerr, yerr, opts);
}

// ============ NEW: Fill Between ============

/**
 * @brief Fill between two y curves
 */
inline Axes &fill_between(const std::vector<double> &x,
                          const std::vector<double> &y1,
                          const std::vector<double> &y2,
                          const PlotOptions &opts = PlotOptions()) {
  return gca().fill_between(x, y1, y2, opts);
}

/**
 * @brief Fill between curve and baseline
 */
inline Axes &fill_between(const std::vector<double> &x,
                          const std::vector<double> &y, double baseline = 0.0,
                          const PlotOptions &opts = PlotOptions()) {
  return gca().fill_between(x, y, baseline, opts);
}

// ============ NEW: Polar ============

/**
 * @brief Enable polar axes
 */
inline Axes &polar(bool p = true) { return gca().polar(p); }

// ============ NEW: Contour ============

/**
 * @brief Contour plot
 * @param x X grid coordinates
 * @param y Y grid coordinates
 * @param z 2D scalar field
 * @param levels Contour levels
 * @param opts Plot options
 */
inline Axes &contour(const std::vector<double> &x, const std::vector<double> &y,
                     const std::vector<std::vector<double>> &z,
                     const std::vector<double> &levels,
                     const PlotOptions &opts = PlotOptions()) {
  return gca().contour(x, y, z, levels, opts);
}

// ============ NEW: Text Annotations ============

/**
 * @brief Add text annotation at data coordinates
 * @param x X position (data coordinates)
 * @param y Y position (data coordinates)
 * @param text Text content (supports LaTeX with $ or backslash commands)
 * @param opts Options: fontsize, color, ha, va, rotation, usetex
 */
inline Axes &text(double x, double y, const std::string &text,
                  const PlotOptions &opts = PlotOptions()) {
  return gca().text(x, y, text, opts);
}

/**
 * @brief Add text at figure coordinates (0-1 normalized)
 * @param x X position (0=left, 1=right)
 * @param y Y position (0=bottom, 1=top)
 * @param text Text content (supports LaTeX)
 * @param opts Options: fontsize, color, ha, va, rotation, usetex
 */
inline Axes &figtext(double x, double y, const std::string &text,
                     const PlotOptions &opts = PlotOptions()) {
  return gca().figtext(x, y, text, opts);
}

/**
 * @brief Add LaTeX formula at data coordinates
 * @param x X position (data coordinates)
 * @param y Y position (data coordinates)
 * @param latexExpr LaTeX expression (e.g., "$\alpha^2 + \beta^2$")
 * @param opts Options: fontsize, color, ha, va
 */
inline Axes &latex(double x, double y, const std::string &latexExpr,
                   const PlotOptions &opts = PlotOptions()) {
  return gca().latex(x, y, latexExpr, opts);
}

/**
 * @brief Add annotation with optional arrow
 */
inline Axes &annotate(const std::string &text, double x, double y, double textX,
                      double textY, const PlotOptions &opts = PlotOptions()) {
  return gca().annotate(text, x, y, textX, textY, opts);
}

// ============ NEW: Horizontal/Vertical Lines ============

/**
 * @brief Add horizontal line across plot
 */
inline Axes &axhline(double y, const PlotOptions &opts = PlotOptions()) {
  return gca().axhline(y, opts);
}

/**
 * @brief Add vertical line across plot
 */
inline Axes &axvline(double x, const PlotOptions &opts = PlotOptions()) {
  return gca().axvline(x, opts);
}

// ============ NEW: Scale Functions ============

/**
 * @brief Set x-axis scale ("linear" or "log")
 */
inline void xscale(const std::string &scale) { gca().set_xscale(scale); }

/**
 * @brief Set y-axis scale ("linear" or "log")
 */
inline void yscale(const std::string &scale) { gca().set_yscale(scale); }

// ============ Labeling Functions ============

/**
 * @brief Set x-axis label
 */
inline void xlabel(const std::string &label) { gca().set_xlabel(label); }

/**
 * @brief Set x-axis label with font options
 */
inline void xlabel(const std::string &label, const PlotOptions &opts) {
  gca().set_xlabel(label);
  double fontsize = getOptionDouble(opts, "fontsize", 0);
  if (fontsize > 0)
    gca().set_xlabel_fontsize(fontsize);
  std::string fontfamily = getOptionStr(opts, "fontfamily");
  if (fontfamily.empty())
    fontfamily = getOptionStr(opts, "family");
  if (!fontfamily.empty())
    gca().set_xlabel_fontfamily(fontfamily);
  std::string fontweight = getOptionStr(opts, "fontweight");
  if (fontweight.empty())
    fontweight = getOptionStr(opts, "weight");
  if (!fontweight.empty())
    gca().set_xlabel_fontweight(fontweight);
  std::string color = getOptionStr(opts, "color");
  if (!color.empty())
    gca().set_xlabel_color(color);
}

/**
 * @brief Set y-axis label
 */
inline void ylabel(const std::string &label) { gca().set_ylabel(label); }

/**
 * @brief Set secondary y-axis label
 */
inline void ylabel2(const std::string &label) { gca().set_ylabel2(label); }

/**
 * @brief Set y-axis label with font options
 */
inline void ylabel(const std::string &label, const PlotOptions &opts) {
  gca().set_ylabel(label);
  double fontsize = getOptionDouble(opts, "fontsize", 0);
  if (fontsize > 0)
    gca().set_ylabel_fontsize(fontsize);
  std::string fontfamily = getOptionStr(opts, "fontfamily");
  if (fontfamily.empty())
    fontfamily = getOptionStr(opts, "family");
  if (!fontfamily.empty())
    gca().set_ylabel_fontfamily(fontfamily);
  std::string fontweight = getOptionStr(opts, "fontweight");
  if (fontweight.empty())
    fontweight = getOptionStr(opts, "weight");
  if (!fontweight.empty())
    gca().set_ylabel_fontweight(fontweight);
  std::string color = getOptionStr(opts, "color");
  if (!color.empty())
    gca().set_ylabel_color(color);
}

/**
 * @brief Set plot title
 */
inline void title(const std::string &t) { gca().set_title(t); }

/**
 * @brief Set plot title with font options
 */
inline void title(const std::string &t, const PlotOptions &opts) {
  gca().set_title(t);
  double fontsize = getOptionDouble(opts, "fontsize", 0);
  if (fontsize > 0)
    gca().set_title_fontsize(fontsize);
  std::string fontfamily = getOptionStr(opts, "fontfamily");
  if (fontfamily.empty())
    fontfamily = getOptionStr(opts, "family");
  if (!fontfamily.empty())
    gca().set_title_fontfamily(fontfamily);
  std::string fontweight = getOptionStr(opts, "fontweight");
  if (fontweight.empty())
    fontweight = getOptionStr(opts, "weight");
  if (!fontweight.empty())
    gca().set_title_fontweight(fontweight);
  std::string color = getOptionStr(opts, "color");
  if (!color.empty())
    gca().set_title_color(color);
}

/**
 * @brief Set figure title (suptitle)
 */
inline void suptitle(const std::string &t) { gcf().suptitle(t); }

// ============ Font Customization Functions ============

/**
 * @brief Set tick label font size (both axes)
 */
inline void tick_params(const PlotOptions &opts) {
  double labelsize = getOptionDouble(opts, "labelsize", 0);
  if (labelsize > 0)
    gca().set_tick_fontsize(labelsize);
  std::string labelfamily = getOptionStr(opts, "labelfamily");
  if (!labelfamily.empty())
    gca().set_tick_fontfamily(labelfamily);
  std::string labelcolor = getOptionStr(opts, "labelcolor");
  if (!labelcolor.empty()) {
    gca().set_xtick_color(labelcolor);
    gca().set_ytick_color(labelcolor);
  }
}

/**
 * @brief Set legend font options
 */
inline void legend(bool show, const PlotOptions &opts) {
  gca().legend(show);
  if (!show)
    return;
  double fontsize = getOptionDouble(opts, "fontsize", 0);
  if (fontsize > 0)
    gca().set_legend_fontsize(fontsize);
  std::string fontfamily = getOptionStr(opts, "fontfamily");
  if (fontfamily.empty())
    fontfamily = getOptionStr(opts, "family");
  if (!fontfamily.empty())
    gca().set_legend_fontfamily(fontfamily);
  std::string fontweight = getOptionStr(opts, "fontweight");
  if (!fontweight.empty())
    gca().set_legend_fontweight(fontweight);
  std::string color = getOptionStr(opts, "color");
  if (color.empty())
    color = getOptionStr(opts, "labelcolor");
  if (!color.empty())
    gca().set_legend_color(color);
}

/**
 * @brief Set font family for all text elements
 */
inline void fontfamily(const std::string &family) {
  gca().set_fontfamily(family);
}

/**
 * @brief Configure font settings using rcParams-like options
 */
inline void rc(const std::string &group, const PlotOptions &opts) {
  if (group == "font") {
    std::string family = getOptionStr(opts, "family");
    if (!family.empty())
      gca().set_fontfamily(family);
    double size = getOptionDouble(opts, "size", 0);
    if (size > 0) {
      gca().set_title_fontsize(size);
      gca().set_label_fontsize(size);
      gca().set_tick_fontsize(size * 0.9);
      gca().set_legend_fontsize(size * 0.9);
    }
    std::string weight = getOptionStr(opts, "weight");
    if (!weight.empty()) {
      gca().set_title_fontweight(weight);
      gca().set_xlabel_fontweight(weight);
      gca().set_ylabel_fontweight(weight);
    }
  } else if (group == "axes") {
    double titlesize = getOptionDouble(opts, "titlesize", 0);
    if (titlesize > 0)
      gca().set_title_fontsize(titlesize);
    double labelsize = getOptionDouble(opts, "labelsize", 0);
    if (labelsize > 0)
      gca().set_label_fontsize(labelsize);
    std::string titleweight = getOptionStr(opts, "titleweight");
    if (!titleweight.empty())
      gca().set_title_fontweight(titleweight);
    std::string labelweight = getOptionStr(opts, "labelweight");
    if (!labelweight.empty()) {
      gca().set_xlabel_fontweight(labelweight);
      gca().set_ylabel_fontweight(labelweight);
    }
  } else if (group == "xtick") {
    double labelsize = getOptionDouble(opts, "labelsize", 0);
    if (labelsize > 0)
      gca().set_xtick_fontsize(labelsize);
    std::string color = getOptionStr(opts, "color");
    if (!color.empty())
      gca().set_xtick_color(color);
  } else if (group == "ytick") {
    double labelsize = getOptionDouble(opts, "labelsize", 0);
    if (labelsize > 0)
      gca().set_ytick_fontsize(labelsize);
    std::string color = getOptionStr(opts, "color");
    if (!color.empty())
      gca().set_ytick_color(color);
  } else if (group == "legend") {
    double fontsize = getOptionDouble(opts, "fontsize", 0);
    if (fontsize > 0)
      gca().set_legend_fontsize(fontsize);
  }
}

// ============ Axis Functions ============

/**
 * @brief Set x-axis limits
 */
inline void xlim(double min, double max) { gca().set_xlim(min, max); }

/**
 * @brief Set y-axis limits
 */
inline void ylim(double min, double max) { gca().set_ylim(min, max); }

/**
 * @brief Show/hide grid
 */
inline void grid(bool show = true) { gca().grid(show); }

/**
 * @brief Show legend
 */
inline void legend(bool show = true) { gca().legend(show); }

// ============ Silent Mode for Notebook Integration ============

namespace detail {
inline bool &silent_mode() {
  static bool mode = false;
  return mode;
}
} // namespace detail

/**
 * @brief Enable/disable silent mode (suppresses plot output)
 *
 * Used by notebook environments to prevent intermediate cells
 * from generating visible plot files.
 */
inline void set_silent(bool silent) { detail::silent_mode() = silent; }

/**
 * @brief Check if silent mode is enabled
 */
inline bool is_silent() { return detail::silent_mode(); }

// ============ Output Functions ============

/**
 * @brief Save figure to file
 *
 * Respects silent mode - if silent, skips file output.
 */
inline void savefig(const std::string &filename) {
  if (detail::silent_mode())
    return; // Skip in silent mode
  gcf().savefig(filename);
}

/**
 * @brief Show figure
 *
 * Respects silent mode - if silent, skips showing.
 */
inline void show() {
  if (detail::silent_mode())
    return; // Skip in silent mode
  gcf().show();
}

/**
 * @brief Close/clear figure
 */
inline void clf() { gcf().clear(); }

inline void close() { clf(); }

// ============ Style Functions ============

/**
 * @brief Set style/theme
 */
inline void style(const std::string &name) { gca().setTheme(name); }

// ============ Convenience Data Generation ============

// These are already defined in utils.hpp but we expose them here
// for the pyplot-style interface
using cppplot::arange;
using cppplot::linspace;
using cppplot::logspace;
using cppplot::randn;
using cppplot::random;

} // namespace cppplot

#endif // CPPPLOT_PYPLOT_HPP
