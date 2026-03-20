/**
 * @file figure.hpp
 * @brief Figure class - container for multiple axes/subplots
 */

#ifndef CPPPLOT_FIGURE_HPP
#define CPPPLOT_FIGURE_HPP

#include "axes.hpp"
#include "backends/svg_backend.hpp"
#include <fstream>
#include <iomanip>
#include <initializer_list>
#include <memory>
#include <sstream>
#include <vector>

namespace cppplot {

/**
 * @brief GridSpec - Flexible grid layout specification (like matplotlib
 * GridSpec / Julia layout)
 *
 * Allows creating complex subplot layouts with:
 * - Custom row/column sizes (relative or absolute)
 * - Subplots spanning multiple rows/columns
 * - Variable spacing between subplots
 */
class GridSpec {
public:
  int nrows;
  int ncols;
  std::vector<double> widthRatios;  // Relative widths of columns
  std::vector<double> heightRatios; // Relative heights of rows
  double wspace =
      0.2; // Horizontal spacing between subplots (fraction of avg width)
  double hspace =
      0.2; // Vertical spacing between subplots (fraction of avg height)
  double left = 0.1;   // Left margin
  double right = 0.9;  // Right margin
  double bottom = 0.1; // Bottom margin
  double top = 0.9;    // Top margin

  GridSpec(int rows = 1, int cols = 1) : nrows(rows), ncols(cols) {
    widthRatios.resize(cols, 1.0);
    heightRatios.resize(rows, 1.0);
  }

  // Set width ratios for columns
  GridSpec &setWidthRatios(std::initializer_list<double> ratios) {
    widthRatios.assign(ratios.begin(), ratios.end());
    if (widthRatios.size() < static_cast<size_t>(ncols)) {
      widthRatios.resize(ncols, 1.0);
    }
    return *this;
  }

  GridSpec &setWidthRatios(const std::vector<double> &ratios) {
    widthRatios = ratios;
    if (widthRatios.size() < static_cast<size_t>(ncols)) {
      widthRatios.resize(ncols, 1.0);
    }
    return *this;
  }

  // Set height ratios for rows
  GridSpec &setHeightRatios(std::initializer_list<double> ratios) {
    heightRatios.assign(ratios.begin(), ratios.end());
    if (heightRatios.size() < static_cast<size_t>(nrows)) {
      heightRatios.resize(nrows, 1.0);
    }
    return *this;
  }

  GridSpec &setHeightRatios(const std::vector<double> &ratios) {
    heightRatios = ratios;
    if (heightRatios.size() < static_cast<size_t>(nrows)) {
      heightRatios.resize(nrows, 1.0);
    }
    return *this;
  }

  // Set spacing
  GridSpec &setWspace(double w) {
    wspace = w;
    return *this;
  }
  GridSpec &setHspace(double h) {
    hspace = h;
    return *this;
  }
  GridSpec &setSpacing(double w, double h) {
    wspace = w;
    hspace = h;
    return *this;
  }

  // Set margins
  GridSpec &setMargins(double l, double r, double b, double t) {
    left = l;
    right = r;
    bottom = b;
    top = t;
    return *this;
  }

  /**
   * @brief Get position for a subplot that spans multiple cells
   * @param row1, col1 - Top-left cell (0-based)
   * @param row2, col2 - Bottom-right cell (0-based, inclusive)
   */
  Rect getSubplotPosition(int row1, int col1, int row2, int col2) const {
    // Normalize ratios
    double totalWidthRatio = 0, totalHeightRatio = 0;
    for (double w : widthRatios)
      totalWidthRatio += w;
    for (double h : heightRatios)
      totalHeightRatio += h;

    // Available area
    double availWidth = right - left;
    double availHeight = top - bottom;

    // Total spacing
    double totalWspace = wspace * availWidth * (ncols - 1) / ncols;
    double totalHspace = hspace * availHeight * (nrows - 1) / nrows;

    // Usable area for plots
    double plotWidth = availWidth - totalWspace;
    double plotHeight = availHeight - totalHspace;

    // Calculate cell boundaries
    std::vector<double> colStarts(ncols + 1), rowStarts(nrows + 1);

    double x = left;
    double cellWspace = (ncols > 1) ? totalWspace / (ncols - 1) : 0;
    for (int c = 0; c <= ncols; ++c) {
      colStarts[c] = x;
      if (c < ncols) {
        double w = plotWidth * (widthRatios[c] / totalWidthRatio);
        x += w;
        if (c < ncols - 1)
          x += cellWspace;
      }
    }

    double y = top; // Start from top
    double cellHspace = (nrows > 1) ? totalHspace / (nrows - 1) : 0;
    for (int r = 0; r <= nrows; ++r) {
      rowStarts[r] = y;
      if (r < nrows) {
        double h = plotHeight * (heightRatios[r] / totalHeightRatio);
        y -= h; // Move down
        if (r < nrows - 1)
          y -= cellHspace;
      }
    }

    // Get position for the span
    double x1 = colStarts[col1];
    double x2 = colStarts[col2 + 1];
    double y1 = rowStarts[row2 + 1]; // Note: y is inverted
    double y2 = rowStarts[row1];

    return Rect(x1, y1, x2 - x1, y2 - y1);
  }

  // Get position for single cell
  Rect getSubplotPosition(int row, int col) const {
    return getSubplotPosition(row, col, row, col);
  }

  // Get position using 1-based index
  Rect getSubplotPositionByIndex(int index) const {
    int idx = index - 1;
    int row = idx / ncols;
    int col = idx % ncols;
    return getSubplotPosition(row, col);
  }
};

/**
 * @brief Layout helper functions (Julia-style)
 */

// Create a grid layout
inline GridSpec grid(int rows, int cols) { return GridSpec(rows, cols); }

// Create a grid with custom widths
inline GridSpec grid(int rows, int cols, std::initializer_list<double> widths) {
  return GridSpec(rows, cols).setWidthRatios(widths);
}

// Create a grid with custom widths and heights
inline GridSpec grid(int rows, int cols, std::initializer_list<double> widths,
                     std::initializer_list<double> heights) {
  return GridSpec(rows, cols).setWidthRatios(widths).setHeightRatios(heights);
}

/**
 * @brief SubplotSpec - Specifies position of a subplot within a GridSpec
 */
struct SubplotSpec {
  int row1, col1; // Top-left (0-based)
  int row2, col2; // Bottom-right (0-based, inclusive)

  SubplotSpec(int r, int c) : row1(r), col1(c), row2(r), col2(c) {}
  SubplotSpec(int r1, int c1, int r2, int c2)
      : row1(r1), col1(c1), row2(r2), col2(c2) {}
};

/**
 * @brief Figure class - represents a complete plot with one or more subplots
 */
class Figure {
private:
  int width_ = 800;
  int height_ = 600;
  int dpi_ = 100;
  Color backgroundColor_ = Color::white();
  std::vector<std::shared_ptr<Axes>> axes_;
  std::shared_ptr<Axes> currentAxes_;
  std::unique_ptr<Backend> backend_;

  static std::string escapeJson(const std::string &s) {
    std::ostringstream out;
    for (char c : s) {
      switch (c) {
      case '\"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\b': out << "\\b"; break;
      case '\f': out << "\\f"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          out << "\\u" << std::hex << std::uppercase << std::setw(4)
              << std::setfill('0') << (int)static_cast<unsigned char>(c);
        } else {
          out << c;
        }
      }
    }
    return out.str();
  }

  static const char *plotTypeName(PlotType t) {
    switch (t) {
    case PlotType::Line: return "line";
    case PlotType::Scatter: return "scatter";
    case PlotType::Bar: return "bar";
    case PlotType::Histogram: return "histogram";
    case PlotType::Pie: return "pie";
    case PlotType::Heatmap: return "heatmap";
    case PlotType::Area: return "area";
    case PlotType::Step: return "step";
    case PlotType::Surface: return "surface";
    case PlotType::Quiver: return "quiver";
    case PlotType::Box: return "box";
    case PlotType::Contour: return "contour";
    default: return "unknown";
    }
  }

  static const char *legendPositionName(LegendPosition p) {
    switch (p) {
    case LegendPosition::Best: return "best";
    case LegendPosition::UpperRight: return "upper_right";
    case LegendPosition::UpperLeft: return "upper_left";
    case LegendPosition::LowerRight: return "lower_right";
    case LegendPosition::LowerLeft: return "lower_left";
    case LegendPosition::Right: return "right";
    case LegendPosition::CenterRight: return "center_right";
    case LegendPosition::CenterLeft: return "center_left";
    case LegendPosition::LowerCenter: return "lower_center";
    case LegendPosition::UpperCenter: return "upper_center";
    case LegendPosition::Center: return "center";
    default: return "best";
    }
  }
  std::string title_;
  GridSpec gridSpec_;
  bool useGridSpec_ = false;

public:
  Figure(int width = 800, int height = 600) : width_(width), height_(height) {
    backend_ = std::make_unique<SVGBackend>(width, height);
  }

  // Size
  Figure &setSize(int width, int height) {
    width_ = width;
    height_ = height;
    backend_->setSize(width, height);
    return *this;
  }

  int width() const { return width_; }
  int height() const { return height_; }

  // DPI
  Figure &setDpi(int dpi) {
    dpi_ = dpi;
    return *this;
  }
  int dpi() const { return dpi_; }

  // Background
  Figure &setBackgroundColor(const Color &c) {
    backgroundColor_ = c;
    return *this;
  }

  // Title (suptitle)
  Figure &suptitle(const std::string &t) {
    title_ = t;
    return *this;
  }

  // ============ Layout Methods ============

  /**
   * @brief Set layout using GridSpec
   * Usage: fig.setLayout(GridSpec(2, 3).setWidthRatios({1, 2, 1}))
   */
  Figure &setLayout(const GridSpec &gs) {
    gridSpec_ = gs;
    useGridSpec_ = true;
    return *this;
  }

  /**
   * @brief Set simple grid layout
   * Usage: fig.setLayout(2, 3) for 2 rows x 3 columns
   */
  Figure &setLayout(int nrows, int ncols) {
    gridSpec_ = GridSpec(nrows, ncols);
    useGridSpec_ = true;
    return *this;
  }

  /**
   * @brief Set layout with custom column widths
   */
  Figure &setLayout(int nrows, int ncols,
                    std::initializer_list<double> widths) {
    gridSpec_ = GridSpec(nrows, ncols).setWidthRatios(widths);
    useGridSpec_ = true;
    return *this;
  }

  /**
   * @brief Set layout with custom column widths and row heights
   */
  Figure &setLayout(int nrows, int ncols, std::initializer_list<double> widths,
                    std::initializer_list<double> heights) {
    gridSpec_ =
        GridSpec(nrows, ncols).setWidthRatios(widths).setHeightRatios(heights);
    useGridSpec_ = true;
    return *this;
  }

  /**
   * @brief Get GridSpec for modification
   */
  GridSpec &layout() {
    useGridSpec_ = true;
    return gridSpec_;
  }

  /**
   * @brief Add subplot using matplotlib-style indexing
   * @param nrows Number of rows
   * @param ncols Number of columns
   * @param index 1-based index (row-major order)
   * @return Reference to created Axes
   */
  Axes &add_subplot(int nrows, int ncols, int index) {
    // If no GridSpec set, create one
    if (!useGridSpec_ || gridSpec_.nrows != nrows || gridSpec_.ncols != ncols) {
      gridSpec_ = GridSpec(nrows, ncols);
      useGridSpec_ = true;
    }

    Rect pos = gridSpec_.getSubplotPositionByIndex(index);

    auto ax = std::make_shared<Axes>(pos);
    axes_.push_back(ax);
    currentAxes_ = ax;

    return *ax;
  }

  // Shorthand for subplot
  Axes &subplot(int nrows, int ncols, int index) {
    return add_subplot(nrows, ncols, index);
  }

  /**
   * @brief Add subplot spanning multiple cells (Julia-style)
   * @param nrows, ncols - Grid dimensions
   * @param indices - List of cell indices to span (1-based)
   *
   * Example: subplot(2, 3, {1, 2}) spans cells 1 and 2 (top-left two cells)
   */
  Axes &subplot(int nrows, int ncols, std::initializer_list<int> indices) {
    if (!useGridSpec_ || gridSpec_.nrows != nrows || gridSpec_.ncols != ncols) {
      gridSpec_ = GridSpec(nrows, ncols);
      useGridSpec_ = true;
    }

    // Find bounding box of all indices
    int minRow = nrows, maxRow = 0;
    int minCol = ncols, maxCol = 0;

    for (int idx : indices) {
      int i = idx - 1;
      int row = i / ncols;
      int col = i % ncols;
      minRow = std::min(minRow, row);
      maxRow = std::max(maxRow, row);
      minCol = std::min(minCol, col);
      maxCol = std::max(maxCol, col);
    }

    Rect pos = gridSpec_.getSubplotPosition(minRow, minCol, maxRow, maxCol);

    auto ax = std::make_shared<Axes>(pos);
    axes_.push_back(ax);
    currentAxes_ = ax;

    return *ax;
  }

  /**
   * @brief Add subplot at specific grid position (0-based)
   * @param row, col - Grid position (0-based)
   */
  Axes &subplot_at(int row, int col) {
    if (!useGridSpec_) {
      gridSpec_ = GridSpec(row + 1, col + 1);
      useGridSpec_ = true;
    }

    Rect pos = gridSpec_.getSubplotPosition(row, col);

    auto ax = std::make_shared<Axes>(pos);
    axes_.push_back(ax);
    currentAxes_ = ax;

    return *ax;
  }

  /**
   * @brief Add subplot spanning multiple cells using row/col ranges (0-based)
   * @param row1, col1 - Top-left cell (0-based)
   * @param row2, col2 - Bottom-right cell (0-based, inclusive)
   */
  Axes &subplot_span(int row1, int col1, int row2, int col2) {
    if (!useGridSpec_) {
      gridSpec_ = GridSpec(std::max(row1, row2) + 1, std::max(col1, col2) + 1);
      useGridSpec_ = true;
    }

    Rect pos = gridSpec_.getSubplotPosition(row1, col1, row2, col2);

    auto ax = std::make_shared<Axes>(pos);
    axes_.push_back(ax);
    currentAxes_ = ax;

    return *ax;
  }

  /**
   * @brief Add subplot using SubplotSpec
   */
  Axes &add_subplot(const SubplotSpec &spec) {
    return subplot_span(spec.row1, spec.col1, spec.row2, spec.col2);
  }

  /**
   * @brief Add axes at specified position
   * @param x Left position (0-1)
   * @param y Bottom position (0-1)
   * @param width Width (0-1)
   * @param height Height (0-1)
   */
  Axes &add_axes(double x, double y, double width, double height) {
    auto ax = std::make_shared<Axes>(Rect(x, y, width, height));
    axes_.push_back(ax);
    currentAxes_ = ax;
    return *ax;
  }

  /**
   * @brief Add inset axes within current axes
   * @param x, y - Position relative to data coordinates or axes (0-1)
   * @param w, h - Size as fraction of axes
   */
  Axes &inset_axes(double x, double y, double w, double h) {
    // Position relative to current axes
    if (currentAxes_) {
      const Rect &parent = currentAxes_->position();
      Rect pos(parent.x + x * parent.width, parent.y + y * parent.height,
               w * parent.width, h * parent.height);
      auto ax = std::make_shared<Axes>(pos);
      axes_.push_back(ax);
      currentAxes_ = ax;
      return *ax;
    }
    return add_axes(x, y, w, h);
  }

  /**
   * @brief Get current axes or create one if none exists
   */
  Axes &gca() {
    if (!currentAxes_) {
      // Create default axes
      add_subplot(1, 1, 1);
    }
    return *currentAxes_;
  }

  /**
   * @brief Set current axes
   */
  void sca(Axes &ax) {
    for (auto &a : axes_) {
      if (a.get() == &ax) {
        currentAxes_ = a;
        return;
      }
    }
  }

  /**
   * @brief Get all axes
   */
  std::vector<std::shared_ptr<Axes>> &getAxes() { return axes_; }
  const std::vector<std::shared_ptr<Axes>> &getAxes() const { return axes_; }

  /**
   * @brief Render figure to backend
   */
  void render() {
    backend_->setSize(width_, height_);
    backend_->clear(backgroundColor_);

    Rect figureArea(0, 0, width_, height_);

    // Adjust for figure title
    if (!title_.empty()) {
      figureArea.y += 30;
      figureArea.height -= 30;

      TextStyle titleStyle;
      titleStyle.fontSize = 16;
      titleStyle.setBold();
      titleStyle.anchor = TextAnchor::Middle;
      titleStyle.baseline = TextBaseline::Middle;

      backend_->drawText(width_ / 2.0, 20, title_, titleStyle);
    }

    // Render all axes
    for (auto &ax : axes_) {
      ax->render(*backend_, figureArea);
    }
  }

  /**
   * @brief Save figure to file
   */
  void savefig(const std::string &filename) {
    render();
    backend_->save(trim(filename));

    // Save metadata alongside SVG
    std::string metaFile = trim(filename) + ".meta.json";
    std::ofstream metaOut(metaFile, std::ios::out | std::ios::trunc);
    if (metaOut) {
      metaOut << metadataJson();
    }
  }

  /**
   * @brief Get rendered content as string (SVG)
   */
  std::string toSVG() {
    render();
    return backend_->render();
  }

  /**
   * @brief Export figure metadata as JSON
   */
  std::string metadataJson() const {
    std::ostringstream ss;
    ss << "{";
    ss << "\"schema_version\":\"cppplot-meta-1.0\",";
    ss << "\"figure\":{";
    ss << "\"width\":" << width_ << ",";
    ss << "\"height\":" << height_ << ",";
    ss << "\"title\":\"" << escapeJson(title_) << "\"";
    ss << "},";

    ss << "\"axes\":[";
    for (size_t ai = 0; ai < axes_.size(); ++ai) {
      const auto &ax = axes_[ai];
      if (ai > 0) ss << ",";
      ss << "{";
      ss << "\"xlabel\":\"" << escapeJson(ax->xlabel()) << "\",";
      ss << "\"ylabel\":\"" << escapeJson(ax->ylabel()) << "\",";
      ss << "\"title\":\"" << escapeJson(ax->title()) << "\",";
      ss << "\"xlim\":[" << ax->xlim().min << "," << ax->xlim().max << "],";
      ss << "\"ylim\":[" << ax->ylim().min << "," << ax->ylim().max << "],";
      ss << "\"xlimSet\":" << (ax->xlimSet() ? "true" : "false") << ",";
      ss << "\"ylimSet\":" << (ax->ylimSet() ? "true" : "false") << ",";
      ss << "\"grid\":{";
      ss << "\"show\":" << (ax->gridShow() ? "true" : "false");
      ss << "},";
      ss << "\"legend\":{";
      ss << "\"show\":" << (ax->legendStyle().visible ? "true" : "false") << ",";
      ss << "\"position\":\"" << legendPositionName(ax->legendStyle().position) << "\"";
      ss << "},";

      ss << "\"series\":[";
      const auto &elems = ax->elements();
      for (size_t ei = 0; ei < elems.size(); ++ei) {
        const auto &el = elems[ei];
        if (ei > 0) ss << ",";
        ss << "{";
        ss << "\"type\":\"" << plotTypeName(el->type) << "\",";
        ss << "\"visible\":true,";
        ss << "\"label\":\"" << escapeJson(el->style.label) << "\",";
        ss << "\"color\":\"" << el->style.line.color.toHex() << "\",";
        ss << "\"linewidth\":" << el->style.line.width << ",";
        ss << "\"linestyle\":\"" << escapeJson(el->style.line.style) << "\",";
        ss << "\"marker\":\"" << escapeJson(el->style.marker.marker) << "\",";
        ss << "\"markersize\":" << el->style.marker.size;
        ss << "}";
      }
      ss << "]";
      ss << "}";
    }
    ss << "]";
    ss << "}";
    return ss.str();
  }

  /**
   * @brief Show figure (platform-dependent)
   * On Windows: Opens SVG in default browser
   * This is a simple implementation - a full implementation would
   * create a temporary file and open it
   */
  void show() {
    // Save to temp file and open
    std::string tempFile = "temp_plot.svg";
    savefig(tempFile);

#ifdef _WIN32
    std::string cmd = "start " + tempFile;
#elif __APPLE__
    std::string cmd = "open " + tempFile;
#else
    std::string cmd = "xdg-open " + tempFile;
#endif

    std::system(cmd.c_str());
  }

  /**
   * @brief Clear figure
   */
  void clear() {
    axes_.clear();
    currentAxes_.reset();
    title_.clear();
  }

  /**
   * @brief Apply metadata (Phase 1: limited style/label updates)
   */
  void applyMetadata(const std::string &json) {
    if (json.empty()) return;
    auto findString = [&](const std::string &key) -> std::string {
      std::string needle = "\"" + key + "\":\"";
      auto pos = json.find(needle);
      if (pos == std::string::npos) return "";
      pos += needle.size();
      auto end = json.find("\"", pos);
      if (end == std::string::npos) return "";
      return json.substr(pos, end - pos);
    };
    auto findNumber = [&](const std::string &key, double fallback) -> double {
      std::string needle = "\"" + key + "\":";
      auto pos = json.find(needle);
      if (pos == std::string::npos) return fallback;
      pos += needle.size();
      size_t end = pos;
      while (end < json.size() && (std::isdigit(json[end]) || json[end] == '.' || json[end] == '-' || json[end] == '+')) {
        end++;
      }
      try {
        return std::stod(json.substr(pos, end - pos));
      } catch (...) {
        return fallback;
      }
    };

    // Figure-level title (optional)
    std::string figTitle = findString("title");
    if (!figTitle.empty()) {
      title_ = figTitle;
    }

    // Apply only to first axes for Phase 1
    if (axes_.empty()) return;
    auto &ax = axes_[0];

    std::string xlabel = findString("xlabel");
    std::string ylabel = findString("ylabel");
    std::string atitle = findString("title");
    if (!xlabel.empty()) ax->set_xlabel(xlabel);
    if (!ylabel.empty()) ax->set_ylabel(ylabel);
    if (!atitle.empty()) ax->set_title(atitle);

    // Line style updates (best-effort, first line series only)
    std::string color = findString("color");
    if (!color.empty()) {
      const auto &elems = ax->elements();
      if (!elems.empty()) {
        elems[0]->style.line.color = Color::fromHex(color);
      }
    }

    double lw = findNumber("linewidth", -1);
    if (lw > 0) {
      const auto &elems = ax->elements();
      if (!elems.empty()) {
        elems[0]->style.line.width = lw;
      }
    }
  }

  /**
   * @brief Tight layout - adjust subplot parameters
   */
  void tight_layout() {
    // TODO: Implement automatic spacing adjustment
  }
};

} // namespace cppplot

#endif // CPPPLOT_FIGURE_HPP
