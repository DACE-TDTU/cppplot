/**
 * @file polynomial.hpp
 * @brief Polynomial class for transfer function representation
 * 
 * Supports polynomial arithmetic: +, -, *, evaluation, roots
 */

#ifndef CPPPLOT_CONTROL_POLYNOMIAL_HPP
#define CPPPLOT_CONTROL_POLYNOMIAL_HPP

// Define M_PI for Windows compatibility
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <vector>
#include <complex>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <sstream>
#include <iomanip>

namespace cppplot {
namespace control {

/**
 * @class Polynomial
 * @brief Represents a polynomial: a_n*s^n + a_{n-1}*s^{n-1} + ... + a_1*s + a_0
 * 
 * Coefficients are stored in descending order: [a_n, a_{n-1}, ..., a_1, a_0]
 * This matches MATLAB/NumPy convention.
 */
class Polynomial {
public:
    std::vector<double> coeffs;  // Descending order: [a_n, ..., a_0]
    
    // ============ Constructors ============
    
    /// Default: constant 0
    Polynomial() : coeffs({0}) {}
    
    /// From coefficient vector (descending order)
    explicit Polynomial(const std::vector<double>& c) : coeffs(c) {
        if (coeffs.empty()) coeffs = {0};
        normalize();
    }
    
    /// From initializer list: Polynomial p({1, 2, 1}) = s^2 + 2s + 1
    Polynomial(std::initializer_list<double> c) : coeffs(c) {
        if (coeffs.empty()) coeffs = {0};
        normalize();
    }
    
    /// Constant polynomial
    explicit Polynomial(double c) : coeffs({c}) {}
    
    // ============ Basic Properties ============
    
    /// Degree of polynomial
    int degree() const {
        return static_cast<int>(coeffs.size()) - 1;
    }
    
    /// Leading coefficient
    double leading() const {
        return coeffs.empty() ? 0 : coeffs[0];
    }
    
    /// Trailing coefficient (constant term)
    double constant() const {
        return coeffs.empty() ? 0 : coeffs.back();
    }
    
    /// Check if zero polynomial
    bool isZero() const {
        return coeffs.size() == 1 && std::abs(coeffs[0]) < 1e-15;
    }
    
    /// Check if constant polynomial
    bool isConstant() const {
        return coeffs.size() == 1;
    }
    
    // ============ Evaluation ============
    
    /// Evaluate at real value using Horner's method
    double operator()(double s) const {
        double result = 0;
        for (double c : coeffs) {
            result = result * s + c;
        }
        return result;
    }
    
    /// Evaluate at complex value
    std::complex<double> operator()(std::complex<double> s) const {
        std::complex<double> result(0, 0);
        for (double c : coeffs) {
            result = result * s + c;
        }
        return result;
    }
    
    // ============ Arithmetic Operations ============
    
    /// Addition
    Polynomial operator+(const Polynomial& other) const {
        size_t n1 = coeffs.size();
        size_t n2 = other.coeffs.size();
        size_t n = std::max(n1, n2);
        
        std::vector<double> result(n, 0);
        
        // Add from right (low order to high)
        for (size_t i = 0; i < n1; ++i) {
            result[n - n1 + i] += coeffs[i];
        }
        for (size_t i = 0; i < n2; ++i) {
            result[n - n2 + i] += other.coeffs[i];
        }
        
        return Polynomial(result);
    }
    
    Polynomial operator+(double c) const {
        Polynomial result = *this;
        result.coeffs.back() += c;
        return result;
    }
    
    /// Subtraction
    Polynomial operator-(const Polynomial& other) const {
        return *this + (other * (-1.0));
    }
    
    Polynomial operator-() const {
        return *this * (-1.0);
    }
    
    /// Multiplication
    Polynomial operator*(const Polynomial& other) const {
        if (isZero() || other.isZero()) return Polynomial({0});
        
        size_t n1 = coeffs.size();
        size_t n2 = other.coeffs.size();
        std::vector<double> result(n1 + n2 - 1, 0);
        
        for (size_t i = 0; i < n1; ++i) {
            for (size_t j = 0; j < n2; ++j) {
                result[i + j] += coeffs[i] * other.coeffs[j];
            }
        }
        
        return Polynomial(result);
    }
    
    Polynomial operator*(double c) const {
        std::vector<double> result = coeffs;
        for (double& coef : result) coef *= c;
        return Polynomial(result);
    }
    
    /// Division (returns quotient, ignores remainder)
    Polynomial operator/(const Polynomial& divisor) const {
        if (divisor.isZero()) {
            throw std::runtime_error("Polynomial division by zero");
        }
        
        if (degree() < divisor.degree()) {
            return Polynomial({0});
        }
        
        std::vector<double> dividend = coeffs;
        std::vector<double> quotient;
        
        while (dividend.size() >= divisor.coeffs.size()) {
            double factor = dividend[0] / divisor.coeffs[0];
            quotient.push_back(factor);
            
            for (size_t i = 0; i < divisor.coeffs.size(); ++i) {
                dividend[i] -= factor * divisor.coeffs[i];
            }
            dividend.erase(dividend.begin());
        }
        
        return Polynomial(quotient);
    }
    
    Polynomial operator/(double c) const {
        return *this * (1.0 / c);
    }
    
    // ============ Roots Finding ============
    
    /**
     * @brief Find roots of polynomial using companion matrix method
     * @return Vector of complex roots
     */
    std::vector<std::complex<double>> roots() const {
        std::vector<std::complex<double>> result;
        
        if (degree() <= 0) return result;
        
        // Normalize
        std::vector<double> c = coeffs;
        double lead = c[0];
        for (double& x : c) x /= lead;
        
        if (degree() == 1) {
            // Linear: s + a = 0 => s = -a
            result.push_back(std::complex<double>(-c[1], 0));
            return result;
        }
        
        if (degree() == 2) {
            // Quadratic formula
            double a = c[0], b = c[1], cc = c[2];
            double disc = b * b - 4 * a * cc;
            if (disc >= 0) {
                result.push_back(std::complex<double>((-b + std::sqrt(disc)) / (2 * a), 0));
                result.push_back(std::complex<double>((-b - std::sqrt(disc)) / (2 * a), 0));
            } else {
                double re = -b / (2 * a);
                double im = std::sqrt(-disc) / (2 * a);
                result.push_back(std::complex<double>(re, im));
                result.push_back(std::complex<double>(re, -im));
            }
            return result;
        }
        
        // For higher degrees, use eigenvalue method (companion matrix)
        // Simplified: use Durand-Kerner method for now
        return durandKerner();
    }
    
    // ============ String Representation ============
    
    std::string toString(const std::string& var = "s") const {
        if (isZero()) return "0";
        
        std::ostringstream oss;
        bool first = true;
        
        int deg = degree();
        for (int i = 0; i <= deg; ++i) {
            double c = coeffs[i];
            int power = deg - i;
            
            if (std::abs(c) < 1e-15) continue;
            
            // Sign
            if (!first) {
                oss << (c >= 0 ? " + " : " - ");
                c = std::abs(c);
            } else if (c < 0) {
                oss << "-";
                c = -c;
            }
            first = false;
            
            // Coefficient
            if (power == 0 || std::abs(c - 1.0) > 1e-10) {
                oss << std::setprecision(4) << c;
            }
            
            // Variable
            if (power > 0) {
                oss << var;
                if (power > 1) oss << "^" << power;
            }
        }
        
        return first ? "0" : oss.str();
    }
    
private:
    /// Remove leading zeros
    void normalize() {
        while (coeffs.size() > 1 && std::abs(coeffs[0]) < 1e-15) {
            coeffs.erase(coeffs.begin());
        }
    }
    
    /// Durand-Kerner root finding method
    std::vector<std::complex<double>> durandKerner(int maxIter = 1000, double tol = 1e-10) const {
        int n = degree();
        if (n <= 0) return {};
        
        // Normalize polynomial
        std::vector<double> c = coeffs;
        double lead = c[0];
        for (double& x : c) x /= lead;
        
        // Initial guesses on unit circle
        std::vector<std::complex<double>> z(n);
        for (int i = 0; i < n; ++i) {
            double angle = 2.0 * M_PI * i / n + 0.1;
            double r = 1.0 + 0.1 * i;
            z[i] = std::complex<double>(r * std::cos(angle), r * std::sin(angle));
        }
        
        // Iteration
        for (int iter = 0; iter < maxIter; ++iter) {
            double maxChange = 0;
            
            for (int i = 0; i < n; ++i) {
                // Evaluate polynomial at z[i]
                std::complex<double> p = (*this)(z[i]) / lead;
                
                // Product of (z[i] - z[j]) for j != i
                std::complex<double> denom(1, 0);
                for (int j = 0; j < n; ++j) {
                    if (i != j) {
                        denom *= (z[i] - z[j]);
                    }
                }
                
                std::complex<double> delta = p / denom;
                z[i] -= delta;
                maxChange = std::max(maxChange, std::abs(delta));
            }
            
            if (maxChange < tol) break;
        }
        
        return z;
    }
};

// ============ Non-member operators ============

inline Polynomial operator+(double c, const Polynomial& p) {
    return p + c;
}

inline Polynomial operator*(double c, const Polynomial& p) {
    return p * c;
}

/**
 * @brief Create polynomial from roots: (s - r1)(s - r2)...
 */
inline Polynomial poly(const std::vector<std::complex<double>>& roots) {
    Polynomial result({1});
    for (const auto& r : roots) {
        if (std::abs(r.imag()) < 1e-10) {
            // Real root: (s - r)
            result = result * Polynomial({1, -r.real()});
        } else if (r.imag() > 0) {
            // Complex conjugate pair: (s - r)(s - r*) = s^2 - 2*Re(r)*s + |r|^2
            double re = r.real();
            double mag2 = std::norm(r);
            result = result * Polynomial({1, -2*re, mag2});
        }
        // Skip conjugates (imag < 0) as they're already handled
    }
    return result;
}

inline Polynomial poly(const std::vector<double>& realRoots) {
    Polynomial result({1});
    for (double r : realRoots) {
        result = result * Polynomial({1, -r});
    }
    return result;
}

} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_POLYNOMIAL_HPP
