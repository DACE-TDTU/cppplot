/**
 * @file types.hpp
 * @brief Core type definitions for CppPlot
 */

#ifndef CPPPLOT_CORE_TYPES_HPP
#define CPPPLOT_CORE_TYPES_HPP

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// C++17 optional support - fallback for older compilers
#if __cplusplus >= 201703L
#include <optional>
#include <variant>
#else
// Simple optional implementation for C++14
namespace cppplot {
template <typename T> class optional {
  bool has_value_ = false;
  T value_;

public:
  optional() : has_value_(false) {}
  optional(const T &v) : has_value_(true), value_(v) {}
  bool has_value() const { return has_value_; }
  T &value() { return value_; }
  const T &value() const { return value_; }
  T value_or(const T &def) const { return has_value_ ? value_ : def; }
  explicit operator bool() const { return has_value_; }
};
} // namespace cppplot
#endif

namespace cppplot {

/**
 * @brief 2D Point structure
 */
struct Point {
  double x = 0.0;
  double y = 0.0;

  Point() = default;
  Point(double x_, double y_) : x(x_), y(y_) {}

  Point operator+(const Point &other) const {
    return Point(x + other.x, y + other.y);
  }

  Point operator-(const Point &other) const {
    return Point(x - other.x, y - other.y);
  }

  Point operator*(double scalar) const { return Point(x * scalar, y * scalar); }

  double distance(const Point &other) const {
    double dx = x - other.x;
    double dy = y - other.y;
    return std::sqrt(dx * dx + dy * dy);
  }
};

/**
 * @brief Rectangle structure for bounds
 */
struct Rect {
  double x = 0.0;
  double y = 0.0;
  double width = 0.0;
  double height = 0.0;

  Rect() = default;
  Rect(double x_, double y_, double w_, double h_)
      : x(x_), y(y_), width(w_), height(h_) {}

  double left() const { return x; }
  double right() const { return x + width; }
  double top() const { return y; }
  double bottom() const { return y + height; }
  double centerX() const { return x + width / 2; }
  double centerY() const { return y + height / 2; }
  Point center() const { return Point(centerX(), centerY()); }

  bool contains(const Point &p) const {
    return p.x >= x && p.x <= x + width && p.y >= y && p.y <= y + height;
  }
};

/**
 * @brief Axis limits structure
 */
struct Limits {
  double min = 0.0;
  double max = 1.0;

  Limits() = default;
  Limits(double min_, double max_) : min(min_), max(max_) {}

  double range() const { return max - min; }
  double center() const { return (min + max) / 2; }

  void expand(double factor) {
    double c = center();
    double r = range() * factor / 2;
    min = c - r;
    max = c + r;
  }

  void include(double value) {
    min = std::min(min, value);
    max = std::max(max, value);
  }
};

/**
 * @brief Data series container
 */
template <typename T = double> class DataSeries {
private:
  std::vector<T> data_;
  std::string name_;

public:
  DataSeries() = default;

  explicit DataSeries(const std::vector<T> &data, const std::string &name = "")
      : data_(data), name_(name) {}

  DataSeries(std::initializer_list<T> init, const std::string &name = "")
      : data_(init), name_(name) {}

  // Access
  T &operator[](size_t i) { return data_[i]; }
  const T &operator[](size_t i) const { return data_[i]; }

  size_t size() const { return data_.size(); }
  bool empty() const { return data_.empty(); }

  const std::vector<T> &data() const { return data_; }
  std::vector<T> &data() { return data_; }

  const std::string &name() const { return name_; }
  void setName(const std::string &n) { name_ = n; }

  // Iterators
  auto begin() { return data_.begin(); }
  auto end() { return data_.end(); }
  auto begin() const { return data_.begin(); }
  auto end() const { return data_.end(); }

  // Statistics
  T min() const {
    if (data_.empty())
      return T{};
    return *std::min_element(data_.begin(), data_.end());
  }

  T max() const {
    if (data_.empty())
      return T{};
    return *std::max_element(data_.begin(), data_.end());
  }

  double mean() const {
    if (data_.empty())
      return 0.0;
    return std::accumulate(data_.begin(), data_.end(), 0.0) / data_.size();
  }

  double sum() const {
    return std::accumulate(data_.begin(), data_.end(), 0.0);
  }

  Limits limits() const {
    return Limits(static_cast<double>(min()), static_cast<double>(max()));
  }

  // Operations
  void push_back(const T &value) { data_.push_back(value); }
  void clear() { data_.clear(); }
  void reserve(size_t n) { data_.reserve(n); }
};

/**
 * @brief Plot options using map<string, string> for C++14 compatibility
 * Values are stored as strings and converted as needed
 */
using PlotOptions = std::map<std::string, std::string>;

/**
 * @brief Helper to create PlotOptions from initializer_list
 * Usage: opts({{"color", "red"}, {"linewidth", "2"}})
 */
inline PlotOptions
opts(std::initializer_list<std::pair<std::string, std::string>> init) {
  return PlotOptions(init.begin(), init.end());
}

/**
 * @brief Convert double to string for PlotOptions
 */
inline std::string str(double value) {
  std::ostringstream oss;
  oss << value;
  return oss.str();
}

/**
 * @brief Convert int to string for PlotOptions
 */
inline std::string str(int value) { return std::to_string(value); }

/**
 * @brief Helper to get option value with default (string version)
 */
inline std::string getOptionStr(const PlotOptions &opts, const std::string &key,
                                const std::string &defaultValue = "") {
  auto it = opts.find(key);
  if (it == opts.end())
    return defaultValue;
  return it->second;
}

/**
 * @brief Helper to get double option value
 */
inline double getOptionDouble(const PlotOptions &opts, const std::string &key,
                              double defaultValue = 0.0) {
  auto it = opts.find(key);
  if (it == opts.end())
    return defaultValue;
  try {
    return std::stod(it->second);
  } catch (...) {
    return defaultValue;
  }
}

/**
 * @brief Helper to get int option value
 */
inline int getOptionInt(const PlotOptions &opts, const std::string &key,
                        int defaultValue = 0) {
  auto it = opts.find(key);
  if (it == opts.end())
    return defaultValue;
  try {
    return std::stoi(it->second);
  } catch (...) {
    return defaultValue;
  }
}

/**
 * @brief Helper to get bool option value
 */
inline bool getOptionBool(const PlotOptions &opts, const std::string &key,
                          bool defaultValue = false) {
  auto it = opts.find(key);
  if (it == opts.end())
    return defaultValue;
  return it->second == "true" || it->second == "1" || it->second == "yes";
}

/**
 * @brief Text anchor positions
 */
enum class TextAnchor {
  Start,  // Left-aligned
  Middle, // Center-aligned
  End     // Right-aligned
};

/**
 * @brief Text baseline positions
 */
enum class TextBaseline { Top, Middle, Bottom, Alphabetic };

/**
 * @brief Legend position
 */
enum class LegendPosition {
  Best,
  UpperRight,
  UpperLeft,
  LowerRight,
  LowerLeft,
  Right,
  CenterRight,
  CenterLeft,
  LowerCenter,
  UpperCenter,
  Center
};

/**
 * @brief Plot element type
 */
enum class PlotType {
  Line,
  Scatter,
  Bar,
  Histogram,
  Pie,
  Heatmap,
  Area,
  Step,
  Surface,
  Quiver,
  Box,
  Contour
};

} // namespace cppplot

#endif // CPPPLOT_CORE_TYPES_HPP
