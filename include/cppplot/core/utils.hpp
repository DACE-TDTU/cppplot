/**
 * @file utils.hpp
 * @brief Utility functions for CppPlot
 */

#ifndef CPPPLOT_CORE_UTILS_HPP
#define CPPPLOT_CORE_UTILS_HPP

#include "types.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace cppplot {

/**
 * @brief Generate linearly spaced values
 */
inline std::vector<double> linspace(double start, double stop,
                                    size_t num = 50) {
  std::vector<double> result;
  if (num == 0)
    return result;
  if (num == 1) {
    result.push_back(start);
    return result;
  }

  result.reserve(num);
  double step = (stop - start) / (num - 1);
  for (size_t i = 0; i < num; ++i) {
    result.push_back(start + i * step);
  }
  return result;
}

/**
 * @brief Generate logarithmically spaced values
 */
inline std::vector<double> logspace(double start, double stop, size_t num = 50,
                                    double base = 10.0) {
  auto lin = linspace(start, stop, num);
  std::vector<double> result;
  result.reserve(num);
  for (double v : lin) {
    result.push_back(std::pow(base, v));
  }
  return result;
}

/**
 * @brief Generate values with a specific step
 */
inline std::vector<double> arange(double start, double stop,
                                  double step = 1.0) {
  std::vector<double> result;
  if (step > 0) {
    for (double v = start; v < stop; v += step) {
      result.push_back(v);
    }
  } else if (step < 0) {
    for (double v = start; v > stop; v += step) {
      result.push_back(v);
    }
  }
  return result;
}

/**
 * @brief Generate random uniform values
 */
inline std::vector<double> random(size_t n, double min = 0.0,
                                  double max = 1.0) {
  static std::mt19937 gen(std::random_device{}());
  std::uniform_real_distribution<double> dist(min, max);

  std::vector<double> result;
  result.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    result.push_back(dist(gen));
  }
  return result;
}

/**
 * @brief Generate random normal values
 */
inline std::vector<double> randn(size_t n, double mean = 0.0,
                                 double stddev = 1.0) {
  static std::mt19937 gen(std::random_device{}());
  std::normal_distribution<double> dist(mean, stddev);

  std::vector<double> result;
  result.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    result.push_back(dist(gen));
  }
  return result;
}

/**
 * @brief Compute histogram bins
 */
struct HistogramResult {
  std::vector<double> counts;
  std::vector<double> binEdges;
  std::vector<double> binCenters;
};

inline HistogramResult histogram(const std::vector<double> &data, int bins = 10,
                                 double minVal = NAN, double maxVal = NAN) {
  HistogramResult result;
  if (data.empty() || bins <= 0)
    return result;

  double dataMin =
      std::isnan(minVal) ? *std::min_element(data.begin(), data.end()) : minVal;
  double dataMax =
      std::isnan(maxVal) ? *std::max_element(data.begin(), data.end()) : maxVal;

  if (dataMin == dataMax) {
    dataMin -= 0.5;
    dataMax += 0.5;
  }

  double binWidth = (dataMax - dataMin) / bins;

  result.counts.resize(bins, 0);
  result.binEdges.resize(bins + 1);
  result.binCenters.resize(bins);

  for (int i = 0; i <= bins; ++i) {
    result.binEdges[i] = dataMin + i * binWidth;
  }

  for (int i = 0; i < bins; ++i) {
    result.binCenters[i] = dataMin + (i + 0.5) * binWidth;
  }

  for (double v : data) {
    if (v >= dataMin && v <= dataMax) {
      int idx = static_cast<int>((v - dataMin) / binWidth);
      if (idx >= bins)
        idx = bins - 1;
      if (idx < 0)
        idx = 0;
      result.counts[idx]++;
    }
  }

  return result;
}

/**
 * @brief Calculate nice axis tick values
 */
inline std::vector<double> niceTicks(double min, double max,
                                     int maxTicks = 10) {
  std::vector<double> ticks;

  double range = max - min;
  if (range <= 0) {
    ticks.push_back(min);
    return ticks;
  }

  double roughStep = range / (maxTicks - 1);
  double magnitude = std::pow(10, std::floor(std::log10(roughStep)));
  double normalized = roughStep / magnitude;

  double niceStep;
  if (normalized < 1.5)
    niceStep = 1;
  else if (normalized < 3)
    niceStep = 2;
  else if (normalized < 7)
    niceStep = 5;
  else
    niceStep = 10;

  niceStep *= magnitude;

  double niceMin = std::floor(min / niceStep) * niceStep;
  double niceMax = std::ceil(max / niceStep) * niceStep;

  for (double v = niceMin; v <= niceMax + niceStep * 0.5; v += niceStep) {
    if (v >= min - niceStep * 0.001 && v <= max + niceStep * 0.001) {
      ticks.push_back(v);
    }
  }

  return ticks;
}

/**
 * @brief Calculate nice log scale tick values
 */
inline std::vector<double> niceLogTicks(double min, double max,
                                        int maxTicks = 10) {
  std::vector<double> ticks;

  if (min <= 0 || max <= 0 || min >= max) {
    return ticks;
  }

  double logMin = std::floor(std::log10(min));
  double logMax = std::ceil(std::log10(max));

  // Generate major ticks at powers of 10
  for (double exp = logMin; exp <= logMax; exp += 1.0) {
    double tick = std::pow(10, exp);
    if (tick >= min && tick <= max) {
      ticks.push_back(tick);
    }
  }

  // If too few ticks, add intermediate values (2, 5)
  if (ticks.size() < 3) {
    std::vector<double> additionalTicks;
    for (double exp = logMin; exp <= logMax; exp += 1.0) {
      double base = std::pow(10, exp);
      for (double mult : {2.0, 5.0}) {
        double tick = base * mult;
        if (tick >= min && tick <= max) {
          additionalTicks.push_back(tick);
        }
      }
    }
    ticks.insert(ticks.end(), additionalTicks.begin(), additionalTicks.end());
    std::sort(ticks.begin(), ticks.end());
  }

  return ticks;
}

/**
 * @brief Format number for axis label
 */
inline std::string formatNumber(double value, int precision = -1) {
  if (std::abs(value) < 1e-14)
    value = 0.0; // Clean microscopic floating point noise
  std::ostringstream ss;

  if (precision < 0) {
    // Auto precision
    double absVal = std::abs(value);
    if (absVal == 0) {
      ss << "0";
    } else if (absVal >= 1e6 || absVal < 1e-3) {
      ss << std::scientific << std::setprecision(1) << value;
    } else if (absVal >= 100) {
      ss << std::fixed << std::setprecision(0) << value;
    } else if (absVal >= 1) {
      ss << std::fixed << std::setprecision(1) << value;
    } else {
      ss << std::fixed << std::setprecision(3) << value;
    }
  } else {
    ss << std::fixed << std::setprecision(precision) << value;
  }

  std::string result = ss.str();

  // Remove trailing zeros after decimal point
  if (result.find('.') != std::string::npos) {
    size_t lastNonZero = result.find_last_not_of('0');
    if (lastNonZero != std::string::npos && result[lastNonZero] == '.') {
      lastNonZero--;
    }
    if (lastNonZero != std::string::npos) {
      result = result.substr(0, lastNonZero + 1);
    }
  }

  return result;
}

/**
 * @brief Escape XML special characters
 */
inline std::string escapeXML(const std::string &str) {
  std::string result;
  result.reserve(str.size());

  for (char c : str) {
    switch (c) {
    case '&':
      result += "&amp;";
      break;
    case '<':
      result += "&lt;";
      break;
    case '>':
      result += "&gt;";
      break;
    case '"':
      result += "&quot;";
      break;
    case '\'':
      result += "&apos;";
      break;
    default:
      result += c;
    }
  }

  return result;
}

/**
 * @brief Transform data coordinates to pixel coordinates
 */
class CoordinateTransform {
private:
  Rect dataRect_;
  Rect pixelRect_;
  bool flipY_;
  bool logX_ = false;
  bool logY_ = false;

public:
  CoordinateTransform() : flipY_(true), logX_(false), logY_(false) {}

  CoordinateTransform(const Rect &dataRect, const Rect &pixelRect,
                      bool flipY = true, bool logX = false, bool logY = false)
      : dataRect_(dataRect), pixelRect_(pixelRect), flipY_(flipY), logX_(logX),
        logY_(logY) {}

  void setDataRect(const Rect &r) { dataRect_ = r; }
  void setPixelRect(const Rect &r) { pixelRect_ = r; }

  const Rect &dataRect() const { return dataRect_; }
  const Rect &pixelRect() const { return pixelRect_; }

  Point dataToPixel(double x, double y) const {
    // Apply log10 if requested and positive, else clamp/ignore
    double tx = x;
    if (logX_) {
      tx = (x > 0) ? std::log10(x) : dataRect_.x; // clamped to min on error
    }

    double px =
        pixelRect_.x + (tx - dataRect_.x) / dataRect_.width * pixelRect_.width;

    double ty = y;
    if (logY_) {
      ty = (y > 0) ? std::log10(y) : dataRect_.y;
    }

    double py;
    if (flipY_) {
      py = pixelRect_.y + pixelRect_.height -
           (ty - dataRect_.y) / dataRect_.height * pixelRect_.height;
    } else {
      py = pixelRect_.y +
           (ty - dataRect_.y) / dataRect_.height * pixelRect_.height;
    }
    return Point(px, py);
  }

  Point dataToPixel(const Point &p) const { return dataToPixel(p.x, p.y); }

  Point pixelToData(double px, double py) const {
    double x, y;

    // Inverse X transform
    if (logX_) {
      double logX = dataRect_.x +
                    (px - pixelRect_.x) / pixelRect_.width * dataRect_.width;
      x = std::pow(10, logX);
    } else {
      x = dataRect_.x +
          (px - pixelRect_.x) / pixelRect_.width * dataRect_.width;
    }

    // Inverse Y transform
    if (flipY_) {
      y = dataRect_.y + (pixelRect_.y + pixelRect_.height - py) /
                            pixelRect_.height * dataRect_.height;
    } else {
      y = dataRect_.y +
          (py - pixelRect_.y) / pixelRect_.height * dataRect_.height;
    }

    if (logY_) {
      y = std::pow(10, y);
    }

    return Point(x, y);
  }

  Point pixelToData(const Point &p) const { return pixelToData(p.x, p.y); }

  double scaleX(double dataWidth) const {
    return dataWidth / dataRect_.width * pixelRect_.width;
  }

  double scaleY(double dataHeight) const {
    return dataHeight / dataRect_.height * pixelRect_.height;
  }
};

} // namespace cppplot

#endif // CPPPLOT_CORE_UTILS_HPP
