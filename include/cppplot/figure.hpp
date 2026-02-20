/**
 * @file figure.hpp
 * @brief Figure class - container for multiple axes/subplots
 */

#ifndef CPPPLOT_FIGURE_HPP
#define CPPPLOT_FIGURE_HPP

#include "axes.hpp"
#include "backends/svg_backend.hpp"
#include <memory>
#include <vector>
#include <initializer_list>

namespace cppplot {

/**
 * @brief GridSpec - Flexible grid layout specification (like matplotlib GridSpec / Julia layout)
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
    std::vector<double> widthRatios;   // Relative widths of columns
    std::vector<double> heightRatios;  // Relative heights of rows
    double wspace = 0.2;   // Horizontal spacing between subplots (fraction of avg width)
    double hspace = 0.2;   // Vertical spacing between subplots (fraction of avg height)
    double left = 0.1;     // Left margin
    double right = 0.9;    // Right margin
    double bottom = 0.1;   // Bottom margin
    double top = 0.9;      // Top margin
    
    GridSpec(int rows = 1, int cols = 1) : nrows(rows), ncols(cols) {
        widthRatios.resize(cols, 1.0);
        heightRatios.resize(rows, 1.0);
    }
    
    // Set width ratios for columns
    GridSpec& setWidthRatios(std::initializer_list<double> ratios) {
        widthRatios.assign(ratios.begin(), ratios.end());
        if (widthRatios.size() < static_cast<size_t>(ncols)) {
            widthRatios.resize(ncols, 1.0);
        }
        return *this;
    }
    
    GridSpec& setWidthRatios(const std::vector<double>& ratios) {
        widthRatios = ratios;
        if (widthRatios.size() < static_cast<size_t>(ncols)) {
            widthRatios.resize(ncols, 1.0);
        }
        return *this;
    }
    
    // Set height ratios for rows
    GridSpec& setHeightRatios(std::initializer_list<double> ratios) {
        heightRatios.assign(ratios.begin(), ratios.end());
        if (heightRatios.size() < static_cast<size_t>(nrows)) {
            heightRatios.resize(nrows, 1.0);
        }
        return *this;
    }
    
    GridSpec& setHeightRatios(const std::vector<double>& ratios) {
        heightRatios = ratios;
        if (heightRatios.size() < static_cast<size_t>(nrows)) {
            heightRatios.resize(nrows, 1.0);
        }
        return *this;
    }
    
    // Set spacing
    GridSpec& setWspace(double w) { wspace = w; return *this; }
    GridSpec& setHspace(double h) { hspace = h; return *this; }
    GridSpec& setSpacing(double w, double h) { wspace = w; hspace = h; return *this; }
    
    // Set margins
    GridSpec& setMargins(double l, double r, double b, double t) {
        left = l; right = r; bottom = b; top = t;
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
        for (double w : widthRatios) totalWidthRatio += w;
        for (double h : heightRatios) totalHeightRatio += h;
        
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
                if (c < ncols - 1) x += cellWspace;
            }
        }
        
        double y = top;  // Start from top
        double cellHspace = (nrows > 1) ? totalHspace / (nrows - 1) : 0;
        for (int r = 0; r <= nrows; ++r) {
            rowStarts[r] = y;
            if (r < nrows) {
                double h = plotHeight * (heightRatios[r] / totalHeightRatio);
                y -= h;  // Move down
                if (r < nrows - 1) y -= cellHspace;
            }
        }
        
        // Get position for the span
        double x1 = colStarts[col1];
        double x2 = colStarts[col2 + 1];
        double y1 = rowStarts[row2 + 1];  // Note: y is inverted
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
inline GridSpec grid(int rows, int cols) {
    return GridSpec(rows, cols);
}

// Create a grid with custom widths
inline GridSpec grid(int rows, int cols, std::initializer_list<double> widths) {
    return GridSpec(rows, cols).setWidthRatios(widths);
}

// Create a grid with custom widths and heights
inline GridSpec grid(int rows, int cols, 
                     std::initializer_list<double> widths,
                     std::initializer_list<double> heights) {
    return GridSpec(rows, cols).setWidthRatios(widths).setHeightRatios(heights);
}

/**
 * @brief SubplotSpec - Specifies position of a subplot within a GridSpec
 */
struct SubplotSpec {
    int row1, col1;  // Top-left (0-based)
    int row2, col2;  // Bottom-right (0-based, inclusive)
    
    SubplotSpec(int r, int c) : row1(r), col1(c), row2(r), col2(c) {}
    SubplotSpec(int r1, int c1, int r2, int c2) : row1(r1), col1(c1), row2(r2), col2(c2) {}
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
    std::string title_;
    GridSpec gridSpec_;
    bool useGridSpec_ = false;
    
public:
    Figure(int width = 800, int height = 600)
        : width_(width), height_(height) {
        backend_ = std::make_unique<SVGBackend>(width, height);
    }
    
    // Size
    Figure& setSize(int width, int height) {
        width_ = width;
        height_ = height;
        backend_->setSize(width, height);
        return *this;
    }
    
    int width() const { return width_; }
    int height() const { return height_; }
    
    // DPI
    Figure& setDpi(int dpi) { dpi_ = dpi; return *this; }
    int dpi() const { return dpi_; }
    
    // Background
    Figure& setBackgroundColor(const Color& c) { backgroundColor_ = c; return *this; }
    
    // Title (suptitle)
    Figure& suptitle(const std::string& t) { title_ = t; return *this; }
    
    // ============ Layout Methods ============
    
    /**
     * @brief Set layout using GridSpec
     * Usage: fig.setLayout(GridSpec(2, 3).setWidthRatios({1, 2, 1}))
     */
    Figure& setLayout(const GridSpec& gs) {
        gridSpec_ = gs;
        useGridSpec_ = true;
        return *this;
    }
    
    /**
     * @brief Set simple grid layout
     * Usage: fig.setLayout(2, 3) for 2 rows x 3 columns
     */
    Figure& setLayout(int nrows, int ncols) {
        gridSpec_ = GridSpec(nrows, ncols);
        useGridSpec_ = true;
        return *this;
    }
    
    /**
     * @brief Set layout with custom column widths
     */
    Figure& setLayout(int nrows, int ncols, std::initializer_list<double> widths) {
        gridSpec_ = GridSpec(nrows, ncols).setWidthRatios(widths);
        useGridSpec_ = true;
        return *this;
    }
    
    /**
     * @brief Set layout with custom column widths and row heights
     */
    Figure& setLayout(int nrows, int ncols, 
                      std::initializer_list<double> widths,
                      std::initializer_list<double> heights) {
        gridSpec_ = GridSpec(nrows, ncols).setWidthRatios(widths).setHeightRatios(heights);
        useGridSpec_ = true;
        return *this;
    }
    
    /**
     * @brief Get GridSpec for modification
     */
    GridSpec& layout() { 
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
    Axes& add_subplot(int nrows, int ncols, int index) {
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
    Axes& subplot(int nrows, int ncols, int index) {
        return add_subplot(nrows, ncols, index);
    }
    
    /**
     * @brief Add subplot spanning multiple cells (Julia-style)
     * @param nrows, ncols - Grid dimensions
     * @param indices - List of cell indices to span (1-based)
     * 
     * Example: subplot(2, 3, {1, 2}) spans cells 1 and 2 (top-left two cells)
     */
    Axes& subplot(int nrows, int ncols, std::initializer_list<int> indices) {
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
    Axes& subplot_at(int row, int col) {
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
    Axes& subplot_span(int row1, int col1, int row2, int col2) {
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
    Axes& add_subplot(const SubplotSpec& spec) {
        return subplot_span(spec.row1, spec.col1, spec.row2, spec.col2);
    }
    
    /**
     * @brief Add axes at specified position
     * @param x Left position (0-1)
     * @param y Bottom position (0-1)
     * @param width Width (0-1)
     * @param height Height (0-1)
     */
    Axes& add_axes(double x, double y, double width, double height) {
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
    Axes& inset_axes(double x, double y, double w, double h) {
        // Position relative to current axes
        if (currentAxes_) {
            const Rect& parent = currentAxes_->position();
            Rect pos(
                parent.x + x * parent.width,
                parent.y + y * parent.height,
                w * parent.width,
                h * parent.height
            );
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
    Axes& gca() {
        if (!currentAxes_) {
            // Create default axes
            add_subplot(1, 1, 1);
        }
        return *currentAxes_;
    }
    
    /**
     * @brief Set current axes
     */
    void sca(Axes& ax) {
        for (auto& a : axes_) {
            if (a.get() == &ax) {
                currentAxes_ = a;
                return;
            }
        }
    }
    
    /**
     * @brief Get all axes
     */
    std::vector<std::shared_ptr<Axes>>& getAxes() { return axes_; }
    const std::vector<std::shared_ptr<Axes>>& getAxes() const { return axes_; }
    
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
        for (auto& ax : axes_) {
            ax->render(*backend_, figureArea);
        }
    }
    
    /**
     * @brief Save figure to file
     */
    void savefig(const std::string& filename) {
        render();
        backend_->save(filename);
    }
    
    /**
     * @brief Get rendered content as string (SVG)
     */
    std::string toSVG() {
        render();
        return backend_->render();
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
     * @brief Tight layout - adjust subplot parameters
     */
    void tight_layout() {
        // TODO: Implement automatic spacing adjustment
    }
};

} // namespace cppplot

#endif // CPPPLOT_FIGURE_HPP
