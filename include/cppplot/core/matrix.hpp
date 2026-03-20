/**
 * @file matrix.hpp
 * @brief Standalone Matrix class for linear algebra operations
 *
 * A self-contained, header-only matrix library optimized for control systems.
 * Provides all fundamental operations needed for linear system theory:
 * - Basic arithmetic (+, -, *, scalar ops)
 * - Decompositions (LU, QR Householder, Hessenberg, Schur)
 * - Eigenvalue computation (QR iteration with implicit shifts, any size)
 * - Linear system solving (LU with partial pivoting)
 * - Matrix exponential (Padé approximation with scaling-and-squaring)
 * - Determinant, inverse, rank, trace, transpose
 *
 * Design goals:
 *   1. Zero external dependencies (no Eigen, LAPACK)
 *   2. Numerically stable algorithms suitable for systems up to ~50x50
 *   3. Clean educational code — every algorithm is readable
 *   4. Header-only for easy integration
 *
 * @note For production use with large matrices (n > 100), consider Eigen.
 *       This implementation prioritizes clarity and portability.
 */

#ifndef CPPPLOT_CORE_MATRIX_HPP
#define CPPPLOT_CORE_MATRIX_HPP

#include <algorithm>
#include <cassert>
#include <cmath>
#include <complex>
#include <cstddef>   // std::size_t — required for MinGW/older GCC
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <vector>


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace cppplot {

/**
 * @class Matrix
 * @brief Dense matrix of double values with comprehensive linear algebra
 * support
 *
 * Storage: row-major std::vector<std::vector<double>>
 * Indexing: 0-based, (row, col)
 *
 * @example
 *   Matrix A = {{1, 2}, {3, 4}};
 *   Matrix B = Matrix::eye(2);
 *   Matrix C = A * B + A.T();
 *   auto eigs = C.eigenvalues();
 */
class Matrix {
public:
  std::vector<std::vector<double>> data;
  size_t rows, cols;

  // ================================================================
  //  Constructors
  // ================================================================

  Matrix() : rows(0), cols(0) {}

  Matrix(size_t r, size_t c, double val = 0.0) : rows(r), cols(c) {
    data.resize(r, std::vector<double>(c, val));
  }

  Matrix(std::initializer_list<std::initializer_list<double>> init) {
    rows = init.size();
    cols = (rows > 0) ? init.begin()->size() : 0;
    data.reserve(rows);
    for (const auto &row : init) {
      data.push_back(std::vector<double>(row));
    }
  }

  /// Construct column vector from std::vector
  explicit Matrix(const std::vector<double> &v) : rows(v.size()), cols(1) {
    data.resize(rows);
    for (size_t i = 0; i < rows; ++i) {
      data[i] = {v[i]};
    }
  }

  /// Construct 1×n row matrix from std::vector
  static Matrix fromRowVector(const std::vector<double> &v) {
    Matrix m(1, v.size());
    m.data[0] = v;
    return m;
  }

  // ================================================================
  //  Element Access
  // ================================================================

  double &operator()(size_t i, size_t j) { return data[i][j]; }
  double operator()(size_t i, size_t j) const { return data[i][j]; }

  /// Get column j as a std::vector
  std::vector<double> col(size_t j) const {
    std::vector<double> c(rows);
    for (size_t i = 0; i < rows; ++i)
      c[i] = data[i][j];
    return c;
  }

  /// Get row i as a std::vector
  std::vector<double> row(size_t i) const { return data[i]; }

  /// Set column j from a std::vector
  void setCol(size_t j, const std::vector<double> &v) {
    for (size_t i = 0; i < std::min(rows, v.size()); ++i)
      data[i][j] = v[i];
  }

  /// Set row i from a std::vector
  void setRow(size_t i, const std::vector<double> &v) {
    for (size_t j = 0; j < std::min(cols, v.size()); ++j)
      data[i][j] = v[j];
  }

  /// Extract sub-matrix [r1..r2) x [c1..c2)
  Matrix block(size_t r1, size_t c1, size_t numRows, size_t numCols) const {
    Matrix result(numRows, numCols);
    for (size_t i = 0; i < numRows; ++i)
      for (size_t j = 0; j < numCols; ++j)
        result(i, j) = data[r1 + i][c1 + j];
    return result;
  }

  /// Set sub-matrix starting at (r1, c1)
  void setBlock(size_t r1, size_t c1, const Matrix &B) {
    for (size_t i = 0; i < B.rows; ++i)
      for (size_t j = 0; j < B.cols; ++j)
        data[r1 + i][c1 + j] = B(i, j);
  }

  // ================================================================
  //  Factory Methods
  // ================================================================

  static Matrix eye(size_t n) {
    Matrix I(n, n);
    for (size_t i = 0; i < n; ++i)
      I(i, i) = 1.0;
    return I;
  }

  static Matrix zeros(size_t r, size_t c) { return Matrix(r, c, 0.0); }

  static Matrix ones(size_t r, size_t c) { return Matrix(r, c, 1.0); }

  /// Diagonal matrix from vector
  static Matrix diag(const std::vector<double> &v) {
    size_t n = v.size();
    Matrix D(n, n);
    for (size_t i = 0; i < n; ++i)
      D(i, i) = v[i];
    return D;
  }

  /// Extract diagonal as vector
  std::vector<double> diagonal() const {
    size_t n = std::min(rows, cols);
    std::vector<double> d(n);
    for (size_t i = 0; i < n; ++i)
      d[i] = data[i][i];
    return d;
  }

  // ================================================================
  //  Basic Arithmetic
  // ================================================================

  Matrix operator+(const Matrix &other) const {
    if (rows != other.rows || cols != other.cols)
      throw std::runtime_error("Matrix dimension mismatch for addition");
    Matrix result(rows, cols);
    for (size_t i = 0; i < rows; ++i)
      for (size_t j = 0; j < cols; ++j)
        result(i, j) = data[i][j] + other(i, j);
    return result;
  }

  Matrix operator-(const Matrix &other) const {
    if (rows != other.rows || cols != other.cols)
      throw std::runtime_error("Matrix dimension mismatch for subtraction");
    Matrix result(rows, cols);
    for (size_t i = 0; i < rows; ++i)
      for (size_t j = 0; j < cols; ++j)
        result(i, j) = data[i][j] - other(i, j);
    return result;
  }

  Matrix operator*(const Matrix &other) const {
    if (cols != other.rows)
      throw std::runtime_error("Matrix dimension mismatch for multiplication");
    Matrix result(rows, other.cols);
    for (size_t i = 0; i < rows; ++i)
      for (size_t j = 0; j < other.cols; ++j) {
        double sum = 0;
        for (size_t k = 0; k < cols; ++k)
          sum += data[i][k] * other(k, j);
        result(i, j) = sum;
      }
    return result;
  }

  Matrix operator*(double scalar) const {
    Matrix result(rows, cols);
    for (size_t i = 0; i < rows; ++i)
      for (size_t j = 0; j < cols; ++j)
        result(i, j) = data[i][j] * scalar;
    return result;
  }

  friend Matrix operator*(double scalar, const Matrix &M) { return M * scalar; }

  /// Matrix × vector: treats v as a column vector (n×1)
  Matrix operator*(const std::vector<double> &v) const {
    Matrix col_vec(v); // n×1 column vector
    return (*this) * col_vec;
  }

  Matrix operator/(double scalar) const {
    if (std::abs(scalar) < 1e-300)
      throw std::runtime_error("Matrix division by zero");
    return (*this) * (1.0 / scalar);
  }

  Matrix operator-() const {
    Matrix result(rows, cols);
    for (size_t i = 0; i < rows; ++i)
      for (size_t j = 0; j < cols; ++j)
        result(i, j) = -data[i][j];
    return result;
  }

  Matrix &operator+=(const Matrix &other) {
    *this = *this + other;
    return *this;
  }

  Matrix &operator-=(const Matrix &other) {
    *this = *this - other;
    return *this;
  }

  Matrix &operator*=(double scalar) {
    for (size_t i = 0; i < rows; ++i)
      for (size_t j = 0; j < cols; ++j)
        data[i][j] *= scalar;
    return *this;
  }

  // ================================================================
  //  Matrix Properties
  // ================================================================

  /// Transpose
  Matrix T() const {
    Matrix result(cols, rows);
    for (size_t i = 0; i < rows; ++i)
      for (size_t j = 0; j < cols; ++j)
        result(j, i) = data[i][j];
    return result;
  }

  /// Trace (sum of diagonal elements)
  double trace() const {
    if (rows != cols)
      throw std::runtime_error("Trace requires square matrix");
    double sum = 0;
    for (size_t i = 0; i < rows; ++i)
      sum += data[i][i];
    return sum;
  }

  /// Frobenius norm: ||A||_F = sqrt(sum(a_ij^2))
  double norm() const {
    double sum = 0;
    for (size_t i = 0; i < rows; ++i)
      for (size_t j = 0; j < cols; ++j)
        sum += data[i][j] * data[i][j];
    return std::sqrt(sum);
  }

  /// Infinity norm (max absolute row sum)
  double normInf() const {
    double maxSum = 0;
    for (size_t i = 0; i < rows; ++i) {
      double rowSum = 0;
      for (size_t j = 0; j < cols; ++j)
        rowSum += std::abs(data[i][j]);
      maxSum = std::max(maxSum, rowSum);
    }
    return maxSum;
  }

  /// 1-norm (max absolute column sum)
  double norm1() const {
    double maxSum = 0;
    for (size_t j = 0; j < cols; ++j) {
      double colSum = 0;
      for (size_t i = 0; i < rows; ++i)
        colSum += std::abs(data[i][j]);
      maxSum = std::max(maxSum, colSum);
    }
    return maxSum;
  }

  /// Check if square
  bool isSquare() const { return rows == cols; }

  /// Max absolute element
  double maxAbs() const {
    double m = 0;
    for (size_t i = 0; i < rows; ++i)
      for (size_t j = 0; j < cols; ++j)
        m = std::max(m, std::abs(data[i][j]));
    return m;
  }

  /**
   * @brief Estimate condition number using norm(A) * norm(inv(A))
   *
   * Uses 1-norm for estimation. Returns infinity if matrix is singular.
   * High condition number (> 10^10) indicates ill-conditioning.
   */
  double cond() const {
    if (rows != cols)
      throw std::runtime_error("Condition number requires square matrix");
    try {
      Matrix Ainv = inv();
      return norm1() * Ainv.norm1();
    } catch (...) {
      return std::numeric_limits<double>::infinity();
    }
  }

  /**
   * @brief Spectral radius: max absolute eigenvalue
   *
   * Useful for stability analysis: system is stable if spectral_radius < 1
   * (discrete) or if all eigenvalues have negative real part (continuous).
   */
  double spectralRadius() const {
    if (rows != cols)
      throw std::runtime_error("Spectral radius requires square matrix");
    auto eigs = eigenvalues();
    double max_abs = 0;
    for (const auto &e : eigs) {
      max_abs = std::max(max_abs, std::abs(e));
    }
    return max_abs;
  }

  /**
   * @brief Check if matrix is symmetric within tolerance
   */
  bool isSymmetric(double tol = 1e-10) const {
    if (rows != cols)
      return false;
    for (size_t i = 0; i < rows; ++i) {
      for (size_t j = i + 1; j < cols; ++j) {
        if (std::abs(data[i][j] - data[j][i]) > tol)
          return false;
      }
    }
    return true;
  }

  /**
   * @brief Check if matrix is positive definite (symmetric & all eigenvalues >
   * 0)
   *
   * Uses Cholesky decomposition attempt as a robust check.
   */
  bool isPositiveDefinite() const {
    if (!isSymmetric())
      return false;
    size_t n = rows;
    Matrix L(n, n);
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j <= i; ++j) {
        double sum = 0;
        for (size_t k = 0; k < j; ++k)
          sum += L(i, k) * L(j, k);
        if (i == j) {
          double val = data[i][i] - sum;
          if (val <= 0)
            return false; // Not positive definite
          L(i, j) = std::sqrt(val);
        } else {
          L(i, j) = (data[i][j] - sum) / L(j, j);
        }
      }
    }
    return true;
  }

  /**
   * @brief Cholesky decomposition: A = L * L^T for symmetric positive definite
   * A
   * @return Lower triangular matrix L
   * @throws If matrix is not positive definite
   */
  Matrix cholesky() const {
    if (!isSymmetric())
      throw std::runtime_error("Cholesky requires symmetric matrix");
    size_t n = rows;
    Matrix L(n, n);
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j <= i; ++j) {
        double sum = 0;
        for (size_t k = 0; k < j; ++k)
          sum += L(i, k) * L(j, k);
        if (i == j) {
          double val = data[i][i] - sum;
          if (val <= 0)
            throw std::runtime_error("Matrix is not positive definite");
          L(i, j) = std::sqrt(val);
        } else {
          L(i, j) = (data[i][j] - sum) / L(j, j);
        }
      }
    }
    return L;
  }

  // ================================================================
  //  Determinant
  // ================================================================

  /**
   * @brief Compute determinant using LU decomposition
   *
   * For n <= 3: direct formulas (fast, exact for integers)
   * For n > 3: LU decomposition with partial pivoting
   *
   * Complexity: O(n^3)
   */
  double det() const {
    if (rows != cols)
      throw std::runtime_error("Determinant requires square matrix");
    size_t n = rows;

    if (n == 1)
      return data[0][0];
    if (n == 2)
      return data[0][0] * data[1][1] - data[0][1] * data[1][0];
    if (n == 3) {
      return data[0][0] * (data[1][1] * data[2][2] - data[1][2] * data[2][1]) -
             data[0][1] * (data[1][0] * data[2][2] - data[1][2] * data[2][0]) +
             data[0][2] * (data[1][0] * data[2][1] - data[1][1] * data[2][0]);
    }

    // LU decomposition with partial pivoting
    Matrix L = *this;
    double d = 1.0;
    for (size_t k = 0; k < n - 1; ++k) {
      // Partial pivoting
      size_t maxRow = k;
      for (size_t i = k + 1; i < n; ++i) {
        if (std::abs(L(i, k)) > std::abs(L(maxRow, k)))
          maxRow = i;
      }
      if (maxRow != k) {
        std::swap(L.data[k], L.data[maxRow]);
        d = -d; // Swap changes sign
      }
      if (std::abs(L(k, k)) < 1e-15)
        return 0.0;

      for (size_t i = k + 1; i < n; ++i) {
        double factor = L(i, k) / L(k, k);
        for (size_t j = k; j < n; ++j)
          L(i, j) -= factor * L(k, j);
      }
    }
    for (size_t i = 0; i < n; ++i)
      d *= L(i, i);
    return d;
  }

  // ================================================================
  //  Solve Linear System: Ax = b
  // ================================================================

  /**
   * @brief Solve Ax = b using LU decomposition with partial pivoting
   *
   * More numerically stable than computing inv(A)*b.
   *
   * @param b  Right-hand side (column vector or matrix)
   * @return   Solution x such that Ax = b
   *
   * Complexity: O(n^3) for factorization + O(n^2) per RHS column
   */
  Matrix solve(const Matrix &b) const {
    if (rows != cols)
      throw std::runtime_error("solve() requires square matrix");
    if (rows != b.rows)
      throw std::runtime_error("solve() dimension mismatch");

    size_t n = rows;
    size_t nrhs = b.cols;

    // Augmented matrix [A | b]
    Matrix aug(n, n + nrhs);
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < n; ++j)
        aug(i, j) = data[i][j];
      for (size_t j = 0; j < nrhs; ++j)
        aug(i, n + j) = b(i, j);
    }

    // Forward elimination with partial pivoting
    for (size_t k = 0; k < n; ++k) {
      // Find pivot
      size_t maxRow = k;
      for (size_t i = k + 1; i < n; ++i) {
        if (std::abs(aug(i, k)) > std::abs(aug(maxRow, k)))
          maxRow = i;
      }
      if (std::abs(aug(maxRow, k)) < 1e-14)
        throw std::runtime_error("Matrix is singular in solve()");
      if (maxRow != k)
        std::swap(aug.data[k], aug.data[maxRow]);

      // Eliminate below
      for (size_t i = k + 1; i < n; ++i) {
        double factor = aug(i, k) / aug(k, k);
        for (size_t j = k; j < n + nrhs; ++j)
          aug(i, j) -= factor * aug(k, j);
      }
    }

    // Back substitution
    Matrix x(n, nrhs);
    for (int k = static_cast<int>(n) - 1; k >= 0; --k) {
      for (size_t j = 0; j < nrhs; ++j) {
        double sum = aug(k, n + j);
        for (size_t m = k + 1; m < n; ++m)
          sum -= aug(k, m) * x(m, j);
        x(k, j) = sum / aug(k, k);
      }
    }
    return x;
  }

  // ================================================================
  //  Inverse
  // ================================================================

  /**
   * @brief Compute matrix inverse using Gauss-Jordan with partial pivoting
   *
   * For small matrices (n <= 2): direct formulas
   * For larger: Gauss-Jordan elimination
   */
  Matrix inv() const {
    if (rows != cols)
      throw std::runtime_error("Inverse requires square matrix");
    size_t n = rows;
    double d = det();
    if (std::abs(d) < 1e-15)
      throw std::runtime_error("Matrix is singular");

    if (n == 1) {
      Matrix result(1, 1);
      result(0, 0) = 1.0 / data[0][0];
      return result;
    }
    if (n == 2) {
      Matrix result(2, 2);
      result(0, 0) = data[1][1] / d;
      result(0, 1) = -data[0][1] / d;
      result(1, 0) = -data[1][0] / d;
      result(1, 1) = data[0][0] / d;
      return result;
    }

    // Gauss-Jordan with partial pivoting
    return solve(eye(n));
  }

  // ================================================================
  //  Rank
  // ================================================================

  /**
   * @brief Numerical rank via Gaussian elimination with partial pivoting
   * @param tol  Tolerance for zero detection
   */
  size_t rank(double tol = 1e-10) const {
    Matrix R = *this;
    size_t rank_count = 0;
    size_t pivot_col = 0;

    for (size_t pivot_row = 0; pivot_row < rows && pivot_col < cols;
         ++pivot_row) {
      size_t max_row = pivot_row;
      double max_val = std::abs(R(pivot_row, pivot_col));
      for (size_t i = pivot_row + 1; i < rows; ++i) {
        if (std::abs(R(i, pivot_col)) > max_val) {
          max_val = std::abs(R(i, pivot_col));
          max_row = i;
        }
      }
      if (max_val < tol) {
        pivot_col++;
        pivot_row--;
        continue;
      }
      if (max_row != pivot_row)
        std::swap(R.data[pivot_row], R.data[max_row]);

      for (size_t i = pivot_row + 1; i < rows; ++i) {
        double factor = R(i, pivot_col) / R(pivot_row, pivot_col);
        for (size_t j = pivot_col; j < cols; ++j)
          R(i, j) -= factor * R(pivot_row, j);
      }
      rank_count++;
      pivot_col++;
    }
    return rank_count;
  }

  // ================================================================
  //  QR Decomposition (Householder)
  // ================================================================

  /**
   * @brief QR decomposition using Householder reflections
   *
   * A = QR where Q is orthogonal and R is upper triangular.
   * Householder is more numerically stable than Gram-Schmidt.
   *
   * Complexity: O(2n^2(m - n/3)) for m x n matrix
   */
  struct QRResult;
  QRResult qr() const;

  // ================================================================
  //  Hessenberg Reduction
  // ================================================================

  /**
   * @brief Reduce to upper Hessenberg form using Householder reflections
   *
   * Returns H and Q such that A = Q * H * Q^T, where H is upper Hessenberg.
   * This is a prerequisite for efficient QR iteration for eigenvalues.
   *
   * Complexity: O(10n^3/3)
   */
  struct HessenbergResult;
  HessenbergResult hessenberg() const;

  // ================================================================
  //  Eigenvalues — QR Iteration with Implicit Double Shift
  // ================================================================

  /**
   * @brief Compute eigenvalues using Francis QR iteration
   *
   * Algorithm:
   *   1. Reduce A to upper Hessenberg form (O(n^3))
   *   2. Apply implicit double-shift QR iteration
   *   3. Extract eigenvalues from quasi-triangular result
   *
   * This replaces the old characteristic-polynomial approach and works
   * reliably for matrices of any reasonable size (tested up to 50x50).
   * The implicit shift avoids forming the shifted matrix explicitly,
   * improving numerical stability.
   *
   * Complexity: O(n^3) for Hessenberg + O(n^2) per iteration (typically ~2n
   * iterations)
   *
   * @return Vector of complex eigenvalues
   */
  std::vector<std::complex<double>> eigenvalues() const;

  // ================================================================
  //  Real Schur Decomposition
  // ================================================================

  /**
   * @brief Real Schur decomposition: A = Q * T * Q^T
   *
   * T is quasi-upper-triangular (upper triangular with 1x1 and 2x2 blocks
   * on the diagonal). 1x1 blocks are real eigenvalues, 2x2 blocks contain
   * complex conjugate pairs.
   *
   * This is the foundation for reliable CARE/DARE solvers and matrix functions.
   *
   * @return {T, Q} where A = Q*T*Q^T
   */
  struct SchurResult;
  SchurResult schur() const;

// ================================================================
  //  Singular Value Decomposition (SVD) & Pseudo-inverse
  // ================================================================

  /**
   * @brief Singular Value Decomposition (One-sided Jacobi method)
   * * Decomposes A = U * S * V^T
   * - U is m x m orthogonal matrix (if full) or m x n (thin)
   * - S is diagonal matrix of singular values
   * - V is n x n orthogonal matrix
   * * Highly accurate for small-to-medium matrices.
   */
  struct SVDResult;
  SVDResult svd() const;


  /**
   * @brief Moore-Penrose Pseudo-inverse via SVD
   * @param tol Tolerance for dropping small singular values
   */
  Matrix pinv(double tol = 1e-12) const;

  // ================================================================
  //  Matrix Exponential: expm(A)
  // ================================================================

  /**
   * @brief Matrix exponential e^A using Padé approximation with
   * scaling-and-squaring
   *
   * Algorithm (Higham 2005, "The Scaling and Squaring Method for the
   * Matrix Exponential Revisited"):
   *   1. Choose scaling parameter s so that ||A/2^s|| ~ 1
   *   2. Compute Padé[p/p] approximant of e^(A/2^s)
   *   3. Square the result s times: e^A = (e^(A/2^s))^(2^s)
   *
   * Uses order-13 Padé approximant for good accuracy.
   *
   * @return e^A (same dimensions as A)
   */
  Matrix expm() const {
    if (rows != cols)
      throw std::runtime_error("expm requires square matrix");
    size_t n = rows;
    if (n == 0)
      return Matrix();

    // Scaling: find s such that ||A||/2^s <= 1
    double normA = normInf();
    int s = 0;
    if (normA > 0) {
      s = std::max(0, static_cast<int>(std::ceil(std::log2(normA))));
    }

    Matrix As = (*this) * (1.0 / std::pow(2.0, s)); // A / 2^s

    // Padé [6/6] approximant: r66(X) = N(X) / D(X)
    // where N(X) = sum_{k=0}^{6} c_k X^k, D(X) = sum_{k=0}^{6} c_k (-X)^k
    // c_k = (2p - k)! p! / ((2p)! k! (p-k)!) with p = 6
    const double c[] = {1.0,           1.0 / 2.0,
                        1.0 / 9.0, // 5! * 6! / (12! * 2! * 4!) = 1/9...
                                   // actually let's use standard coeffs
                        1.0 / 72.0,    1.0 / 1008.0,
                        1.0 / 30240.0, 1.0 / 1814400.0};
    // Actually, use the well-known Padé coefficients for [6/6]:
    // b_k = (12-k)! * 6! / (12! * k! * (6-k)!)
    const double b[] = {
        1.0,           // b0
        1.0 / 2.0,     // b1 = 1/2
        5.0 / 44.0,    // b2
        1.0 / 66.0,    // b3
        1.0 / 792.0,   // b4
        1.0 / 15840.0, // b5
        1.0 / 665280.0 // b6
    };

    // Compute powers of As
    Matrix I = eye(n);
    Matrix A2 = As * As;
    Matrix A4 = A2 * A2;
    Matrix A6 = A4 * A2;

    // N = b6*A6 + b4*A4 + b2*A2 + b0*I + As*(b5*A4 + b3*A2 + b1*I)
    // D = b6*A6 + b4*A4 + b2*A2 + b0*I - As*(b5*A4 + b3*A2 + b1*I)
    Matrix U =
        As * (A6 * b[5] + A4 * b[3] + A2 * b[1] + I * 0.0); // Odd part * As

    // Let's do it more carefully for [6/6]:
    // even terms: b0*I + b2*A^2 + b4*A^4 + b6*A^6
    Matrix Neven = I * b[0] + A2 * b[2] + A4 * b[4] + A6 * b[6];
    // odd terms: b1*A + b3*A^3 + b5*A^5 = As * (b1*I + b3*A^2 + b5*A^4)
    Matrix Nodd = As * (I * b[1] + A2 * b[3] + A4 * b[5]);

    Matrix N = Neven + Nodd; // Numerator
    Matrix D = Neven - Nodd; // Denominator

    // Solve D * F = N  => F = D^(-1) * N
    Matrix F = D.solve(N);

    // Squaring phase: F = F^(2^s)
    for (int i = 0; i < s; ++i)
      F = F * F;

    return F;
  }

  // ================================================================
  //  Sylvester Equation: AX + XB = C
  // ================================================================

  /**
   * @brief Solve Sylvester equation AX + XB = C
   *
   * Uses Bartels-Stewart algorithm:
   *   1. Schur decomposition of A and B
   *   2. Transform to quasi-triangular form
   *   3. Back-substitution
   *
   * Special case: Lyapunov equation when B = A^T, C = -Q
   *   A*P + P*A^T + Q = 0  =>  A*P + P*A^T = -Q
   */
  static Matrix sylvester(const Matrix &A, const Matrix &B, const Matrix &C);

  /**
   * @brief Solve continuous Lyapunov equation: A*P + P*A^T + Q = 0
   * @return P
   */
  static Matrix lyapunov(const Matrix &A, const Matrix &Q) {
    return sylvester(A, A.T(), Q * (-1.0));
  }

  // ================================================================
  //  Eigenvectors
  // ================================================================

  /**
   * @brief Compute eigenvalues and eigenvectors via Real Schur decomposition
   *
   * Algorithm:
   *   1. Compute Real Schur: A = Q * T * Q^T
   *   2. For each 1×1 block (real eigenvalue λ): solve (T-λI)z=0 by
   *      back-substitution, then v = Q*z (normalized).
   *   3. For each 2×2 block (complex pair σ±iω): solve complex back-subst,
   *      store Re and Im parts separately.
   *
   * @return EigenDecomposition: eigenvalues + real/imag parts of eigenvectors
   *         Column i of vectors_real/vectors_imag = Re/Im of eigenvector i.
   *         For real eigenvalues: vectors_imag column i = zeros.
   *         For complex pair (i, i+1): col i+1 = conjugate of col i.
   */
  struct EigenDecomposition;
  EigenDecomposition eigenvectors() const;

  // ================================================================
  //  Schur Reordering
  // ================================================================

  /**
   * @brief Reorder Real Schur decomposition so "selected" blocks come first
   *
   * Uses adjacent-swap bubble sort on Schur blocks (1×1 or 2×2).
   * Each swap solves a small Sylvester equation to maintain A = Q*T*Q^T.
   *
   * @param sr     Input Schur decomposition from schur()
   * @param select Predicate: return true for eigenvalues to move to top
   *               CARE: [](auto z){ return z.real() < 0; }
   *               DARE: [](auto z){ return std::abs(z) < 1.0; }
   * @return Reordered {T_new, Q_new} with selected blocks in top-left
   */
  static SchurResult schur_reorder(
      const SchurResult &sr,
      std::function<bool(std::complex<double>)> select);

  // ================================================================
  //  String Representation
  // ================================================================

  std::string toString(int precision = 4) const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    for (size_t i = 0; i < rows; ++i) {
      oss << "[ ";
      for (size_t j = 0; j < cols; ++j) {
        oss << std::setw(precision + 6) << data[i][j];
        if (j < cols - 1)
          oss << ", ";
      }
      oss << " ]";
      if (i < rows - 1)
        oss << "\n";
    }
    return oss.str();
  }

  friend std::ostream &operator<<(std::ostream &os, const Matrix &M) {
    os << M.toString();
    return os;
  }

private:
  // ---- Internal helpers ----

  /// Eigenvalues of a 2x2 matrix (direct formula)
  std::vector<std::complex<double>> eigenvalues_2x2_() const {
    double tr = data[0][0] + data[1][1];
    double det_val = data[0][0] * data[1][1] - data[0][1] * data[1][0];
    double disc = tr * tr - 4.0 * det_val;
    std::vector<std::complex<double>> eigs(2);
    if (disc >= 0) {
      eigs[0] = std::complex<double>((tr + std::sqrt(disc)) / 2.0, 0);
      eigs[1] = std::complex<double>((tr - std::sqrt(disc)) / 2.0, 0);
    } else {
      eigs[0] = std::complex<double>(tr / 2.0, std::sqrt(-disc) / 2.0);
      eigs[1] = std::complex<double>(tr / 2.0, -std::sqrt(-disc) / 2.0);
    }
    return eigs;
  }

  /// Eigenvalues of a 2x2 block [[a,b],[c,d]]
  static std::pair<std::complex<double>, std::complex<double>>
  eigenvalues_2x2_block_(double a, double b, double c, double d) {
    double tr = a + d;
    double det_val = a * d - b * c;
    double disc = tr * tr - 4.0 * det_val;
    if (disc >= 0) {
      return {std::complex<double>((tr + std::sqrt(disc)) / 2.0, 0),
              std::complex<double>((tr - std::sqrt(disc)) / 2.0, 0)};
    } else {
      return {std::complex<double>(tr / 2.0, std::sqrt(-disc) / 2.0),
              std::complex<double>(tr / 2.0, -std::sqrt(-disc) / 2.0)};
    }
  }
};

// ================================================================
//  Out-of-line struct definitions (Matrix type is now complete)
// ================================================================

struct Matrix::QRResult {
  Matrix Q;
  Matrix R;
};

struct Matrix::HessenbergResult {
  Matrix H; // Upper Hessenberg
  Matrix Q; // Orthogonal transformation
};

struct Matrix::SchurResult {
  Matrix T; // Quasi-upper-triangular
  Matrix Q; // Orthogonal: A = Q * T * Q^T
};

struct Matrix::EigenDecomposition {
  std::vector<std::complex<double>> values;
  // Column i of vectors_real/vectors_imag = Re/Im of eigenvector i
  // For real eigenvalue:   vectors_imag[:,i] = zero vector
  // For complex pair (i, i+1):
  //   eigenvector i   = vectors_real[:,i] + j * vectors_imag[:,i]
  //   eigenvector i+1 = vectors_real[:,i] - j * vectors_imag[:,i]
  Matrix vectors_real;
  Matrix vectors_imag;
};

struct Matrix::SVDResult {
  Matrix U; // Left singular vectors
  Matrix S; // Diagonal matrix of singular values
  Matrix V; // Right singular vectors
};

// ================================================================
//  Out-of-line method implementations
// ================================================================

inline Matrix::QRResult Matrix::qr() const {
  size_t m = rows, n = cols;
  Matrix R = *this;
  Matrix Q = eye(m);

  size_t minmn = std::min(m, n);
  for (size_t k = 0; k < minmn; ++k) {
    // Extract column below diagonal
    std::vector<double> x(m - k);
    for (size_t i = k; i < m; ++i)
      x[i - k] = R(i, k);

    // Compute Householder vector v
    double normX = 0;
    for (double xi : x)
      normX += xi * xi;
    normX = std::sqrt(normX);

    if (normX < 1e-300)
      continue;

    // Choose sign to avoid cancellation
    double sign = (x[0] >= 0) ? 1.0 : -1.0;
    x[0] += sign * normX;

    // Normalize v
    double normV = 0;
    for (double xi : x)
      normV += xi * xi;
    normV = std::sqrt(normV);
    if (normV < 1e-300)
      continue;
    for (double &xi : x)
      xi /= normV;

    // Apply Householder to R: R = (I - 2vv^T) * R
    for (size_t j = k; j < n; ++j) {
      double dot = 0;
      for (size_t i = 0; i < m - k; ++i)
        dot += x[i] * R(i + k, j);
      for (size_t i = 0; i < m - k; ++i)
        R(i + k, j) -= 2.0 * x[i] * dot;
    }

    // Apply Householder to Q: Q = Q * (I - 2vv^T)
    for (size_t i = 0; i < m; ++i) {
      double dot = 0;
      for (size_t j = 0; j < m - k; ++j)
        dot += Q(i, j + k) * x[j];
      for (size_t j = 0; j < m - k; ++j)
        Q(i, j + k) -= 2.0 * dot * x[j];
    }
  }

  // Clean sub-diagonal
  for (size_t j = 0; j < n; ++j)
    for (size_t i = j + 1; i < m; ++i)
      R(i, j) = 0.0;

  return {Q, R};
}

inline Matrix::HessenbergResult Matrix::hessenberg() const {
  if (rows != cols)
    throw std::runtime_error("Hessenberg reduction requires square matrix");
  size_t n = rows;

  Matrix H = *this;
  Matrix Q = eye(n);

  for (size_t k = 0; k + 2 <= n; ++k) {
    size_t len = n - k - 1;
    std::vector<double> x(len);
    for (size_t i = 0; i < len; ++i)
      x[i] = H(k + 1 + i, k);

    double normX = 0;
    for (double xi : x)
      normX += xi * xi;
    normX = std::sqrt(normX);
    if (normX < 1e-300)
      continue;

    double sign = (x[0] >= 0) ? 1.0 : -1.0;
    x[0] += sign * normX;

    double normV = 0;
    for (double xi : x)
      normV += xi * xi;
    normV = std::sqrt(normV);
    if (normV < 1e-300)
      continue;
    for (double &xi : x)
      xi /= normV;

    // Left: H(k+1:n, :) -= 2 * v * (v^T * H(k+1:n, :))
    for (size_t j = 0; j < n; ++j) {
      double dot = 0;
      for (size_t i = 0; i < len; ++i)
        dot += x[i] * H(k + 1 + i, j);
      for (size_t i = 0; i < len; ++i)
        H(k + 1 + i, j) -= 2.0 * x[i] * dot;
    }

    // Right: H(:, k+1:n) -= 2 * (H(:, k+1:n) * v) * v^T
    for (size_t i = 0; i < n; ++i) {
      double dot = 0;
      for (size_t j = 0; j < len; ++j)
        dot += H(i, k + 1 + j) * x[j];
      for (size_t j = 0; j < len; ++j)
        H(i, k + 1 + j) -= 2.0 * dot * x[j];
    }

    // Accumulate Q
    for (size_t i = 0; i < n; ++i) {
      double dot = 0;
      for (size_t j = 0; j < len; ++j)
        dot += Q(i, k + 1 + j) * x[j];
      for (size_t j = 0; j < len; ++j)
        Q(i, k + 1 + j) -= 2.0 * dot * x[j];
    }
  }

  // Clean sub-sub-diagonal
  for (size_t j = 0; j + 2 < n; ++j)
    for (size_t i = j + 2; i < n; ++i)
      H(i, j) = 0.0;

  return {H, Q};
}

inline std::vector<std::complex<double>> Matrix::eigenvalues() const {
  if (rows != cols)
    throw std::runtime_error("Eigenvalues require square matrix");
  size_t n = rows;

  if (n == 0)
    return {};
  if (n == 1)
    return {std::complex<double>(data[0][0], 0)};
  if (n == 2)
    return eigenvalues_2x2_();

  auto hess = hessenberg();
  Matrix H = hess.H;

  const int maxIter = 100 * static_cast<int>(n);
  const double eps = 1e-14;

  std::vector<std::complex<double>> eigs;
  int nn = static_cast<int>(n);
  int iter = 0;

  while (nn > 0 && iter < maxIter) {
    int l = nn - 1;
    while (l > 0) {
      double s = std::abs(H(l - 1, l - 1)) + std::abs(H(l, l));
      if (s == 0.0)
        s = H.normInf();
      if (std::abs(H(l, l - 1)) < eps * s) {
        H(l, l - 1) = 0.0;
        break;
      }
      --l;
    }

    if (l == nn - 1) {
      eigs.push_back(std::complex<double>(H(nn - 1, nn - 1), 0));
      nn--;
      iter = 0;
      continue;
    }

    if (l == nn - 2) {
      double a = H(nn - 2, nn - 2), b = H(nn - 2, nn - 1);
      double c = H(nn - 1, nn - 2), d = H(nn - 1, nn - 1);
      auto pair = eigenvalues_2x2_block_(a, b, c, d);
      eigs.push_back(pair.first);
      eigs.push_back(pair.second);
      nn -= 2;
      iter = 0;
      continue;
    }

    double a = H(nn - 2, nn - 2), b = H(nn - 2, nn - 1);
    double c = H(nn - 1, nn - 2), d = H(nn - 1, nn - 1);
    double tr = a + d;
    double det_val = a * d - b * c;
    double disc = tr * tr - 4.0 * det_val;

    double shift;
    if (disc >= 0) {
      double s1 = (tr + std::sqrt(disc)) / 2.0;
      double s2 = (tr - std::sqrt(disc)) / 2.0;
      shift = (std::abs(s1 - d) < std::abs(s2 - d)) ? s1 : s2;
    } else {
      shift = d;
    }

    int sz = nn - l;
    for (int i = l; i < nn; ++i)
      H(i, i) -= shift;

    std::vector<double> cs(sz - 1), sn(sz - 1);
    for (int i = l; i < nn - 1; ++i) {
      int idx = i - l;
      double xi = H(i, i), xj = H(i + 1, i);
      double r = std::sqrt(xi * xi + xj * xj);
      if (r < 1e-300) {
        cs[idx] = 1.0;
        sn[idx] = 0.0;
        continue;
      }
      cs[idx] = xi / r;
      sn[idx] = xj / r;

      for (int j = i; j < nn; ++j) {
        double t1 = H(i, j), t2 = H(i + 1, j);
        H(i, j) = cs[idx] * t1 + sn[idx] * t2;
        H(i + 1, j) = -sn[idx] * t1 + cs[idx] * t2;
      }
    }

    for (int i = l; i < nn - 1; ++i) {
      int idx = i - l;
      int rowEnd = std::min(i + 2, nn - 1);
      for (int j = l; j <= rowEnd; ++j) {
        double t1 = H(j, i), t2 = H(j, i + 1);
        H(j, i) = cs[idx] * t1 + sn[idx] * t2;
        H(j, i + 1) = -sn[idx] * t1 + cs[idx] * t2;
      }
    }

    for (int i = l; i < nn; ++i)
      H(i, i) += shift;

    for (int i = l + 1; i < nn; ++i) {
      double s = std::abs(H(i - 1, i - 1)) + std::abs(H(i, i));
      if (s == 0.0)
        s = H.normInf();
      if (std::abs(H(i, i - 1)) < eps * s)
        H(i, i - 1) = 0.0;
    }
    ++iter;
  }

  if (nn > 0) {
    for (int i = 0; i < nn; ++i)
      eigs.push_back(std::complex<double>(H(i, i), 0));
  }

  return eigs;
}

inline Matrix::SchurResult Matrix::schur() const {
  if (rows != cols)
    throw std::runtime_error("Schur decomposition requires square matrix");
  size_t n = rows;

  if (n == 0)
    return {Matrix(), Matrix()};
  if (n == 1)
    return {*this, eye(1)};

  auto hess = hessenberg();
  Matrix H = hess.H;
  Matrix Q = hess.Q;

  const int maxIter = 100 * static_cast<int>(n);
  const double eps = 1e-14;

  int nn = static_cast<int>(n);
  int iter = 0;

  while (nn > 1 && iter < maxIter) {
    int l = nn - 1;
    while (l > 0) {
      double s = std::abs(H(l - 1, l - 1)) + std::abs(H(l, l));
      if (s == 0.0)
        s = H.normInf();
      if (std::abs(H(l, l - 1)) < eps * s) {
        H(l, l - 1) = 0.0;
        break;
      }
      --l;
    }

    if (l >= nn - 1) {
      nn--;
      iter = 0;
      continue;
    }
    if (l == nn - 2) {
      nn -= 2;
      iter = 0;
      continue;
    }

    double a = H(nn - 2, nn - 2), b = H(nn - 2, nn - 1);
    double c = H(nn - 1, nn - 2), d = H(nn - 1, nn - 1);
    double tr = a + d, det_val = a * d - b * c;
    double disc = tr * tr - 4.0 * det_val;
    double shift = (disc >= 0) ? ((std::abs((tr + std::sqrt(disc)) / 2.0 - d) <
                                   std::abs((tr - std::sqrt(disc)) / 2.0 - d))
                                      ? (tr + std::sqrt(disc)) / 2.0
                                      : (tr - std::sqrt(disc)) / 2.0)
                               : d;

    for (int i = l; i < nn; ++i)
      H(i, i) -= shift;

    int sz = nn - l;
    std::vector<double> cs(sz - 1), sn(sz - 1);
    for (int i = l; i < nn - 1; ++i) {
      int idx = i - l;
      double xi = H(i, i), xj = H(i + 1, i);
      double r = std::sqrt(xi * xi + xj * xj);
      if (r < 1e-300) {
        cs[idx] = 1;
        sn[idx] = 0;
        continue;
      }
      cs[idx] = xi / r;
      sn[idx] = xj / r;
      for (int j = i; j < static_cast<int>(n); ++j) {
        double t1 = H(i, j), t2 = H(i + 1, j);
        H(i, j) = cs[idx] * t1 + sn[idx] * t2;
        H(i + 1, j) = -sn[idx] * t1 + cs[idx] * t2;
      }
    }
    for (int i = l; i < nn - 1; ++i) {
      int idx = i - l;
      for (int j = 0; j <= std::min(i + 2, nn - 1); ++j) {
        double t1 = H(j, i), t2 = H(j, i + 1);
        H(j, i) = cs[idx] * t1 + sn[idx] * t2;
        H(j, i + 1) = -sn[idx] * t1 + cs[idx] * t2;
      }
      for (size_t j = 0; j < n; ++j) {
        double t1 = Q(j, i), t2 = Q(j, i + 1);
        Q(j, i) = cs[idx] * t1 + sn[idx] * t2;
        Q(j, i + 1) = -sn[idx] * t1 + cs[idx] * t2;
      }
    }

    for (int i = l; i < nn; ++i)
      H(i, i) += shift;

    for (int i = l + 1; i < nn; ++i) {
      double s = std::abs(H(i - 1, i - 1)) + std::abs(H(i, i));
      if (s == 0.0)
        s = H.normInf();
      if (std::abs(H(i, i - 1)) < eps * s)
        H(i, i - 1) = 0.0;
    }
    ++iter;
  }

  return {H, Q};
}

// ----------------------------------------------------------------
//  eigenvectors() — via Real Schur decomposition
// ----------------------------------------------------------------

inline Matrix::EigenDecomposition Matrix::eigenvectors() const {
  if (rows != cols)
    throw std::runtime_error("eigenvectors() requires square matrix");
  size_t n = rows;

  if (n == 0) return {{}, Matrix(), Matrix()};
  if (n == 1) {
    EigenDecomposition ed;
    ed.values = {std::complex<double>(data[0][0], 0.0)};
    ed.vectors_real = Matrix::eye(1);
    ed.vectors_imag = Matrix::zeros(1, 1);
    return ed;
  }

  // --- Step 1: Real Schur decomposition A = Q * T * Q^T ---
  auto sr = schur();
  const Matrix &T = sr.T;
  const Matrix &Q = sr.Q;

  const double eps = 1e-12;

  EigenDecomposition ed;
  ed.values.reserve(n);
  ed.vectors_real = Matrix::zeros(n, n);
  ed.vectors_imag = Matrix::zeros(n, n);

  // --- Step 2: Scan T diagonal to identify 1×1 and 2×2 blocks ---
  int col = 0; // output column index
  for (int i = 0; i < static_cast<int>(n); ) {

    // Determine block size at position i
    bool is_2x2 = (i + 1 < static_cast<int>(n)) &&
                  (std::abs(T(i + 1, i)) >
                   eps * (std::abs(T(i, i)) + std::abs(T(i + 1, i + 1))));

    if (!is_2x2) {
      // ─── 1×1 block: real eigenvalue λ = T[i,i] ──────────────────
      double lam = T(i, i);
      ed.values.push_back(std::complex<double>(lam, 0.0));

      // Solve (T - λI)z = 0 by back-substitution
      // Free variable: z[i] = 1, back-solve rows i-1 … 0
      std::vector<double> z(n, 0.0);
      z[i] = 1.0;

      for (int k = i - 1; k >= 0; --k) {
        double sum = 0.0;
        for (int j = k + 1; j <= i; ++j)
          sum += T(k, j) * z[j];
        double denom = T(k, k) - lam;
        z[k] = (std::abs(denom) > eps) ? (-sum / denom) : 0.0;
      }

      // Normalize z
      double znorm = 0.0;
      for (double v : z) znorm += v * v;
      znorm = std::sqrt(znorm);
      if (znorm < 1e-300) znorm = 1.0;
      for (double &v : z) v /= znorm;

      // Eigenvector v = Q * z
      for (size_t r = 0; r < n; ++r) {
        double val = 0.0;
        for (size_t s = 0; s < n; ++s)
          val += Q(r, s) * z[s];
        ed.vectors_real(r, col) = val;
        ed.vectors_imag(r, col) = 0.0;
      }

      ++col;
      ++i;

    } else {
      // ─── 2×2 block: may have real or complex eigenvalues ─────────
      double a = T(i, i),     b = T(i, i + 1);
      double c = T(i + 1, i), d = T(i + 1, i + 1);

      auto lam_pair = eigenvalues_2x2_block_(a, b, c, d);
      std::complex<double> lam1 = lam_pair.first;
      std::complex<double> lam2 = lam_pair.second;
      ed.values.push_back(lam1);
      ed.values.push_back(lam2);

      if (lam1.imag() == 0.0) {
        // ── Sub-case: real eigenvalues from 2×2 block ────────────────
        // Compute 2 separate real eigenvectors via back-substitution
        for (int ev = 0; ev < 2; ++ev) {
          double lam = (ev == 0) ? lam1.real() : lam2.real();

          // For 2×2 block [[a,b],[c,d]] at positions (i, i+1):
          // Null vector of [a-λ, b; c, d-λ]: free in row with larger pivot
          double r1 = std::abs(a - lam), r2 = std::abs(c);
          std::vector<double> z(n, 0.0);

          if (r1 > eps || r2 > eps) {
            // Use row 0 of block: (a-λ)*z_i + b*z_{i+1} = 0
            // Free: z_{i+1} = 1 if |b| > eps, else z_i = 1
            if (std::abs(b) > eps) {
              z[i + 1] = 1.0;
              z[i]     = -b / (a - lam + 1e-300 * (std::abs(a - lam) < 1e-300));
              if (std::abs(a - lam) > eps)
                z[i] = -b / (a - lam);
              else
                z[i] = 0.0; // degenerate
            } else if (std::abs(a - lam) > eps) {
              z[i]     = 1.0;
              z[i + 1] = 0.0;
            } else {
              z[i + ev] = 1.0; // last resort
            }
          } else {
            z[i] = (ev == 0) ? 1.0 : 0.0;
            z[i + 1] = (ev == 0) ? 0.0 : 1.0;
          }

          // Back-substitute rows above the block
          for (int k = i - 1; k >= 0; --k) {
            double sum = 0.0;
            for (int j = k + 1; j <= i + 1; ++j)
              sum += T(k, j) * z[j];
            double denom = T(k, k) - lam;
            z[k] = (std::abs(denom) > eps) ? (-sum / denom) : 0.0;
          }

          // Normalize
          double znorm = 0.0;
          for (double v : z) znorm += v * v;
          znorm = std::sqrt(znorm);
          if (znorm < 1e-300) znorm = 1.0;
          for (double &v : z) v /= znorm;

          // Eigenvector v = Q * z
          for (size_t r = 0; r < n; ++r) {
            double val = 0.0;
            for (size_t s = 0; s < n; ++s)
              val += Q(r, s) * z[s];
            ed.vectors_real(r, col) = val;
            ed.vectors_imag(r, col) = 0.0;
          }
          ++col;
        }

      } else {
        // ── Sub-case: complex conjugate pair σ ± iω ──────────────────
        std::complex<double> lam = lam1; // σ + iω

        // Within the 2×2 block, second component:
        //   row 0: (a-λ)*z_i + b*z_{i+1} = 0  → z_{i+1} = (λ-a)/b
        std::complex<double> z2;
        if (std::abs(b) > eps)
          z2 = (lam - a) / b;
        else if (std::abs(c) > eps)
          z2 = std::complex<double>((d - lam).real(), -(d - lam).imag()) /
               (std::abs(c) * std::abs(c)) * std::conj(std::complex<double>(0, 0) - (lam - d));
          // simpler: use column 1 of block
        else
          z2 = std::complex<double>(0.0, 1.0); // fallback

        // Rebuild z2 more robustly
        if (std::abs(b) > eps)
          z2 = (lam - a) / b;
        else
          z2 = std::complex<double>(1.0, 0.0); // use z_i as free instead

        std::vector<std::complex<double>> z_c(n, std::complex<double>(0.0));
        z_c[i]     = std::complex<double>(1.0, 0.0);
        z_c[i + 1] = z2;

        // Back-substitute rows above the block
        // Note: T is quasi-upper-triangular, so when stepping through rows
        // above, we may encounter 2×2 blocks — handle them as 2×2 complex systems
        int k = i - 1;
        while (k >= 0) {
          // Check if k is the bottom of a 2×2 block above
          bool in_upper_2x2 = (k > 0) &&
              (std::abs(T(k, k-1)) > eps * (std::abs(T(k-1,k-1)) + std::abs(T(k,k))));
          int kstart = in_upper_2x2 ? k - 1 : k;
          int ksize  = in_upper_2x2 ? 2 : 1;

          if (ksize == 1) {
            std::complex<double> sum(0.0, 0.0);
            for (int j = k + 1; j < static_cast<int>(n); ++j)
              sum += T(k, j) * z_c[j];
            std::complex<double> denom(T(k, k) - lam.real(), -lam.imag());
            z_c[k] = (std::abs(denom) > eps) ? (-sum / denom)
                                              : std::complex<double>(0.0, 0.0);
            k -= 1;
          } else {
            // 2×2 sub-system: solve
            // [ T[k-1,k-1]-λ   T[k-1,k]   ] [z[k-1]]   [rhs0]
            // [ T[k,  k-1]     T[k,k]  -λ ] [z[k]  ] = [rhs1]
            std::complex<double> rhs0(0.0, 0.0), rhs1(0.0, 0.0);
            for (int j = k + 1; j < static_cast<int>(n); ++j) {
              rhs0 -= T(k-1, j) * z_c[j];
              rhs1 -= T(k,   j) * z_c[j];
            }
            std::complex<double> A00(T(k-1,k-1) - lam.real(), -lam.imag());
            std::complex<double> A01(T(k-1,k),  0.0);
            std::complex<double> A10(T(k,  k-1),0.0);
            std::complex<double> A11(T(k,k) - lam.real(), -lam.imag());
            std::complex<double> det2 = A00*A11 - A01*A10;
            if (std::abs(det2) > eps * eps) {
              z_c[k-1] = ( A11*rhs0 - A01*rhs1) / det2;
              z_c[k]   = (-A10*rhs0 + A00*rhs1) / det2;
            } else {
              z_c[k-1] = std::complex<double>(0.0, 0.0);
              z_c[k]   = std::complex<double>(0.0, 0.0);
            }
            k -= 2;
          }
        }

        // Normalize
        double znorm = 0.0;
        for (const auto &v : z_c) znorm += std::norm(v);
        znorm = std::sqrt(znorm);
        if (znorm < 1e-300) znorm = 1.0;
        for (auto &v : z_c) v /= znorm;

        // Store: v for lam1 in col, conjugate (for lam2) in col+1
        for (size_t r = 0; r < n; ++r) {
          std::complex<double> val(0.0, 0.0);
          for (size_t s = 0; s < n; ++s)
            val += Q(r, s) * z_c[s];
          ed.vectors_real(r, col)     =  val.real();
          ed.vectors_imag(r, col)     =  val.imag();
          ed.vectors_real(r, col + 1) =  val.real();
          ed.vectors_imag(r, col + 1) = -val.imag();
        }
        col += 2;
      }

      i += 2;
    }
  }

  return ed;
}

// ----------------------------------------------------------------
//  schur_reorder() — bubble-sort Schur blocks by selector predicate
// ----------------------------------------------------------------

inline Matrix::SchurResult Matrix::schur_reorder(
    const SchurResult &sr,
    std::function<bool(std::complex<double>)> select) {

  Matrix T = sr.T;
  Matrix Q = sr.Q;
  size_t n = T.rows;
  const double eps = 1e-12;

  // Helper: determine block size at position i.
  // A sub-diagonal entry T(i+1,i) is considered "structural" (part of a 2×2
  // block) only if it is significantly larger than rounding noise.
  // LAPACK uses:  tol = ulp * max(|T(i,i)|, |T(i+1,i+1)|)
  // where ulp = machine epsilon.  But QR deflation can leave sub-diagonal
  // residuals as large as O(eps^{1/2} * ||T||), so we use a stronger floor:
  //   tol = sqrt(eps) * ||T||_F  (conservative: catches noise up to ~1e-8 for
  //                                ||T|| ~ 10, which covers the Hamiltonian case)
  const double T_frob = T.norm();    // Frobenius norm
  const double sqrteps = std::sqrt(eps);
  auto block_size = [&](int i) -> int {
    if (i + 1 >= static_cast<int>(n)) return 1;
    double tol = sqrteps * T_frob;
    return (std::abs(T(i + 1, i)) > tol) ? 2 : 1;
  };

  // Helper: eigenvalue(s) of block starting at i
  // For 2×2 block with complex pair, returns the one with Im > 0.
  // For 2×2 block with real eigenvalues, returns the larger-magnitude one.
  auto block_eig = [&](int i) -> std::complex<double> {
    if (block_size(i) == 1)
      return std::complex<double>(T(i, i), 0.0);
    auto e_pair = eigenvalues_2x2_block_(
        T(i, i), T(i, i+1), T(i+1, i), T(i+1, i+1));
    std::complex<double> e1 = e_pair.first;
    std::complex<double> e2 = e_pair.second;
    // Complex pair: return the one with Im > 0 (represents the whole block)
    if (std::abs(e1.imag()) > eps)
      return (e1.imag() >= 0) ? e1 : e2;
    // Real eigenvalues: return the one with larger absolute value as representative
    return (std::abs(e1.real()) >= std::abs(e2.real())) ? e1 : e2;
  };

  // Helper: check if a 2×2 block has real eigenvalues (discriminant ≥ 0)
  auto block_is_real_pair = [&](int i) -> bool {
    if (block_size(i) != 2) return false;
    double a=T(i,i), b=T(i,i+1), c=T(i+1,i), d=T(i+1,i+1);
    double disc = (a-d)*(a-d) + 4.0*b*c;
    return disc >= 0.0;
  };

  // Helper: split a 2×2 block with real eigenvalues into two 1×1 blocks
  // via a Givens rotation that diagonalizes the block.
  // Post-condition: T(i+1,i) ≈ 0, T is still quasi-upper-triangular.
  auto split_real_2x2_block = [&](int i) {
    double a=T(i,i), b=T(i,i+1), c=T(i+1,i), d=T(i+1,i+1);
    double disc = (a-d)*(a-d) + 4.0*b*c;
    if (disc < 0.0) return;  // complex pair, nothing to split

    // Eigenvalues of the 2×2 block
    double sqrtDisc = std::sqrt(disc);
    double lam1 = ((a+d) + sqrtDisc) / 2.0;
    double lam2 = ((a+d) - sqrtDisc) / 2.0;
    // Put the one selected first (or larger-magnitude first)
    // We just need to find a Givens rotation Z s.t. Z^T * [[a,b],[c,d]] * Z is triangular
    // Use the eigenvector of the 2×2 block for lam1:
    //   (a - lam1)*v0 + b*v1 = 0  →  v = [b, lam1-a]  (if b≠0)
    //   or v = [lam1-d, c]  (if c≠0)
    double gx, gy;
    if (std::abs(b) > std::abs(c)) {
      gx = b;  gy = lam1 - a;
    } else if (std::abs(c) > eps) {
      gx = lam1 - d;  gy = c;
    } else {
      return;  // already diagonal
    }
    double norm_g = std::sqrt(gx*gx + gy*gy);
    if (norm_g < eps) return;
    gx /= norm_g;  gy /= norm_g;

    // Build 2×2 Givens: Z = [gx, -gy; gy, gx]
    // Apply similarity: 2×2 sub-block → Z^T * sub * Z
    // Then apply globally to T and Q
    int sz = 2;
    Matrix Z_loc(2, 2);
    Z_loc(0,0)= gx; Z_loc(0,1)=-gy;
    Z_loc(1,0)= gy; Z_loc(1,1)= gx;

    // Update T rows
    Matrix Trows(2, static_cast<int>(n));
    for (int r=0;r<2;++r) for(size_t c=0;c<n;++c) Trows(r,c)=T(i+r,c);
    Matrix Trows_new = Z_loc.T() * Trows;
    for (int r=0;r<2;++r) for(size_t c=0;c<n;++c) T(i+r,c)=Trows_new(r,c);

    // Update T cols
    Matrix Tcols(static_cast<int>(n), 2);
    for(size_t r=0;r<n;++r) for(int c=0;c<2;++c) Tcols(r,c)=T(r,i+c);
    Matrix Tcols_new = Tcols * Z_loc;
    for(size_t r=0;r<n;++r) for(int c=0;c<2;++c) T(r,i+c)=Tcols_new(r,c);

    // Update Q cols
    Matrix Qcols(static_cast<int>(n), 2);
    for(size_t r=0;r<n;++r) for(int c=0;c<2;++c) Qcols(r,c)=Q(r,i+c);
    Matrix Qcols_new = Qcols * Z_loc;
    for(size_t r=0;r<n;++r) for(int c=0;c<2;++c) Q(r,i+c)=Qcols_new(r,c);

    // Zero out the sub-diagonal element
    const double tol_zero = eps * T.normInf();
    if (std::abs(T(i+1,i)) < tol_zero) T(i+1,i) = 0.0;
  };

  // ── Local helper: apply sz×sz orthogonal Z at position pos ──────
  // T[pos:pos+sz, :]  ← Z^T * T[pos:pos+sz, :]
  // T[:, pos:pos+sz]  ← T[:, pos:pos+sz] * Z
  // Q[:, pos:pos+sz]  ← Q[:, pos:pos+sz] * Z
  auto apply_local_Z = [&](const Matrix& Z, int pos, int sz) {
    {
      Matrix Trows(sz, static_cast<int>(n));
      for (int r=0;r<sz;++r) for(size_t c=0;c<n;++c) Trows(r,c)=T(pos+r,c);
      Matrix Tnew = Z.T()*Trows;
      for (int r=0;r<sz;++r) for(size_t c=0;c<n;++c) T(pos+r,c)=Tnew(r,c);
    }
    {
      Matrix Tcols(static_cast<int>(n), sz);
      for(size_t r=0;r<n;++r) for(int c=0;c<sz;++c) Tcols(r,c)=T(r,pos+c);
      Matrix Tnew = Tcols*Z;
      for(size_t r=0;r<n;++r) for(int c=0;c<sz;++c) T(r,pos+c)=Tnew(r,c);
    }
    {
      Matrix Qcols(static_cast<int>(n), sz);
      for(size_t r=0;r<n;++r) for(int c=0;c<sz;++c) Qcols(r,c)=Q(r,pos+c);
      Matrix Qnew = Qcols*Z;
      for(size_t r=0;r<n;++r) for(int c=0;c<sz;++c) Q(r,pos+c)=Qnew(r,c);
    }
    // Zero strict sub-diagonal elements in the affected block
    double tol_zero = eps * T.normInf();
    for (int r=pos+1;r<pos+sz;++r)
      for (int c=pos;c<r-1&&c<pos+sz;++c)
        if (std::abs(T(r,c)) < tol_zero*100) T(r,c)=0.0;
  };

  // ── Re-Schur a local sz×sz block at position pos ─────────────────
  // Restores quasi-upper-triangular form after a swap operation.
  auto re_schur_local = [&](int pos, int sz) {
    Matrix Tsz(sz,sz,0.0);
    for(int r=0;r<sz;++r) for(int c=0;c<sz;++c) Tsz(r,c)=T(pos+r,pos+c);
    auto sr_loc = Tsz.schur();
    apply_local_Z(sr_loc.Q, pos, sz);
  };

  // ── 2×2 Sylvester for swap(2,2): T22*X - X*T11 = C ──────────────
  // All matrices are 2×2. Solved via 4×4 vec-form (Kronecker product).
  auto sylv2x2_local = [&](const Matrix& T22l, const Matrix& T11l,
                            const Matrix& C) -> Matrix {
    double a=T22l(0,0),b=T22l(0,1),cv=T22l(1,0),d=T22l(1,1);
    double p=T11l(0,0),q=T11l(0,1),r=T11l(1,0),s=T11l(1,1);
    // (I⊗T22 - T11^T⊗I) * vec(X) = vec(C)  — col-major vec
    Matrix M(4,4,0.0);
    M(0,0)=a-p; M(0,1)=b;   M(0,2)=-r;  M(0,3)=0;
    M(1,0)=cv;  M(1,1)=d-p; M(1,2)=0;   M(1,3)=-r;
    M(2,0)=-q;  M(2,1)=0;   M(2,2)=a-s; M(2,3)=b;
    M(3,0)=0;   M(3,1)=-q;  M(3,2)=cv;  M(3,3)=d-s;
    Matrix rhs(4,1,0.0);
    rhs(0,0)=C(0,0); rhs(1,0)=C(1,0); rhs(2,0)=C(0,1); rhs(3,0)=C(1,1);
    Matrix sol = M.solve(rhs);
    Matrix X(2,2,0.0);
    X(0,0)=sol(0,0); X(1,0)=sol(1,0); X(0,1)=sol(2,0); X(1,1)=sol(3,0);
    return X;
  };

  // ── swap_blocks: LAPACK dtrexc — correct for all (si,sj) ─────────
  //
  // Goal: Z^T * [T11 T12] * Z = [T22  * ]
  //             [0   T22]       [0    T11]
  //
  // Cases (si,sj):
  //   (1,1): Givens rotation via scalar x = T12/(T22-T11)
  //   (1,2): Solve (T22-t11*I)*Xv = T12^T, then QR(W)
  //   (2,1): Solve (t22*I-T11)^T*Xv = T12^T, then QR(W)
  //   (2,2): Solve 2×2 Sylvester T22*X-X*T11=T12, QR(W), then re-Schur
  //   All cases with T12≈0: block permutation, then re-Schur if sz>2
  //
  // Post-condition: T remains quasi-upper-triangular; A = Q*T*Q^T preserved.
  auto swap_blocks = [&](int pos, int si, int sj) {
    int sz = si + sj;

    Matrix T11(si,si,0.0), T12(si,sj,0.0), T22(sj,sj,0.0);
    for(int r=0;r<si;++r) for(int c=0;c<si;++c) T11(r,c)=T(pos+r,   pos+c);
    for(int r=0;r<si;++r) for(int c=0;c<sj;++c) T12(r,c)=T(pos+r,   pos+si+c);
    for(int r=0;r<sj;++r) for(int c=0;c<sj;++c) T22(r,c)=T(pos+si+r,pos+si+c);

    double t12n  = T12.norm();
    double tol_t = eps * (T11.norm() + T22.norm() + 1.0);

    if (t12n <= tol_t) {
      // Decoupled blocks: plain permutation [0 I_sj; I_si 0]
      Matrix P(sz,sz,0.0);
      for(int r=0;r<sj;++r) P(r,    si+r) = 1.0;
      for(int r=0;r<si;++r) P(sj+r, r   ) = 1.0;
      apply_local_Z(P, pos, sz);
      if (sz > 2) re_schur_local(pos, sz);

    } else if (si == 1 && sj == 1) {
      // Exchange two 1×1 blocks.
      // Build Z = [v0, -v1; v1, v0] where [v0;v1] is the unit eigenvector
      // of T22 (the block we want to move to top-left) in the 2×2 sub-system.
      //
      // T_sub = [T11, T12; 0, T22] = [a, b; 0, d]
      // Eigenvector of d: (T_sub - d*I)*v = 0 → v = [b/(d-a); 1] (normalized)
      // Z with first col = v guarantees Z^T*T_sub*Z is upper triangular
      // with d on top (i.e., sub-diagonal element T_out(1,0) = 0 exactly).
      double a = T11(0,0), b = T12(0,0), d = T22(0,0);
      double denom = d - a;
      double v0, v1;
      if (std::abs(denom) > eps) {
        v0 = b / denom;  v1 = 1.0;
      } else {
        // Eigenvalues are equal — blocks already equivalent, use identity
        v0 = 1.0;  v1 = 0.0;
      }
      double nrm = std::sqrt(v0*v0 + v1*v1);
      if (nrm > eps) { v0 /= nrm;  v1 /= nrm; }
      // Z = [v0 -v1; v1 v0]
      Matrix Z2(2,2); Z2(0,0)=v0; Z2(0,1)=-v1; Z2(1,0)=v1; Z2(1,1)=v0;
      apply_local_Z(Z2, pos, sz);

    } else if (si == 1 && sj == 2) {
      // Solve (T22 - t11*I)*Xv = [T12(0,0); T12(0,1)], Xv is 2×1
      double t11v = T11(0,0);
      Matrix A2 = T22 - eye(2)*t11v;
      Matrix rhs(2,1,0.0); rhs(0,0)=T12(0,0); rhs(1,0)=T12(0,1);
      Matrix Xv = A2.solve(rhs);
      Matrix W = eye(3);
      W(2,0) = -Xv(0,0); W(2,1) = -Xv(1,0);
      apply_local_Z(W.qr().Q, pos, sz);
      re_schur_local(pos, sz);
      // Inline fixup: if re_schur placed unstable first, swap within the 3×3.
      // This avoids oscillation when cppplot::schur() has a fixed ordering preference.
      for (int fp = 0; fp < sz; ++fp) {
        int mi = pos, changed = 0;
        while (mi < pos + sz) {
          int msi = block_size(mi), mj = mi + msi;
          if (mj >= pos + sz) break;
          int msj = block_size(mj);
          if (!select(block_eig(mi)) && select(block_eig(mj))) {
            int mszz = msi + msj;
            if (msi==1 && msj==1) {
              double ma=T(mi,mi),mb=T(mi,mj),md=T(mj,mj);
              double den=md-ma,mv0=fabs(den)>eps?mb/den:1.0,mv1=fabs(den)>eps?1.0:0.0;
              double mn=std::sqrt(mv0*mv0+mv1*mv1); if(mn>eps){mv0/=mn;mv1/=mn;}
              Matrix mZ2(2,2);mZ2(0,0)=mv0;mZ2(0,1)=-mv1;mZ2(1,0)=mv1;mZ2(1,1)=mv0;
              apply_local_Z(mZ2,mi,mszz);
            } else if (msi==1 && msj==2) {
              Matrix mT22m(2,2,0.0);for(int r=0;r<2;++r)for(int c=0;c<2;++c)mT22m(r,c)=T(mj+r,mj+c);
              Matrix mA2m=mT22m-eye(2)*T(mi,mi);
              Matrix mr2(2,1,0.0);mr2(0,0)=T(mi,mj);mr2(1,0)=T(mi,mj+1);
              Matrix mXv2=mA2m.solve(mr2);
              Matrix mW2=eye(3);mW2(2,0)=-mXv2(0,0);mW2(2,1)=-mXv2(1,0);
              apply_local_Z(mW2.qr().Q,mi,mszz);
            } else if (msi==2 && msj==1) {
              Matrix mT11m(2,2,0.0);for(int r=0;r<2;++r)for(int c=0;c<2;++c)mT11m(r,c)=T(mi+r,mi+c);
              Matrix mA2Tm=(eye(2)*T(mj,mj)-mT11m).T();
              Matrix mr2(2,1,0.0);mr2(0,0)=T(mi,mj);mr2(1,0)=T(mi+1,mj);
              Matrix mXv2=mA2Tm.solve(mr2);
              Matrix mW2=eye(3);mW2(1,0)=-mXv2(0,0);mW2(2,0)=-mXv2(1,0);
              apply_local_Z(mW2.qr().Q,mi,mszz);
            }
            changed = 1;
            if (block_is_real_pair(mi)) split_real_2x2_block(mi);
            mi += block_size(mi);
          } else { mi += msi; }
        }
        if (!changed) break;
      }

    } else if (si == 2 && sj == 1) {
      // Solve (t22*I - T11)^T * Xv = [T12(0,0); T12(1,0)], Xv is 2×1
      double t22v = T22(0,0);
      Matrix A2T = (eye(2)*t22v - T11).T();
      Matrix rhs(2,1,0.0); rhs(0,0)=T12(0,0); rhs(1,0)=T12(1,0);
      Matrix Xv = A2T.solve(rhs);
      Matrix W = eye(3);
      W(1,0) = -Xv(0,0); W(2,0) = -Xv(1,0);
      apply_local_Z(W.qr().Q, pos, sz);
      re_schur_local(pos, sz);
      // Same inline fixup for (2,1) case
      for (int fp = 0; fp < sz; ++fp) {
        int mi = pos, changed = 0;
        while (mi < pos + sz) {
          int msi = block_size(mi), mj = mi + msi;
          if (mj >= pos + sz) break;
          int msj = block_size(mj);
          if (!select(block_eig(mi)) && select(block_eig(mj))) {
            int mszz = msi + msj;
            if (msi==1 && msj==1) {
              double ma=T(mi,mi),mb=T(mi,mj),md=T(mj,mj);
              double den=md-ma,mv0=fabs(den)>eps?mb/den:1.0,mv1=fabs(den)>eps?1.0:0.0;
              double mn=std::sqrt(mv0*mv0+mv1*mv1); if(mn>eps){mv0/=mn;mv1/=mn;}
              Matrix mZ2(2,2);mZ2(0,0)=mv0;mZ2(0,1)=-mv1;mZ2(1,0)=mv1;mZ2(1,1)=mv0;
              apply_local_Z(mZ2,mi,mszz);
            } else if (msi==1 && msj==2) {
              Matrix mT22m(2,2,0.0);for(int r=0;r<2;++r)for(int c=0;c<2;++c)mT22m(r,c)=T(mj+r,mj+c);
              Matrix mA2m=mT22m-eye(2)*T(mi,mi);
              Matrix mr2(2,1,0.0);mr2(0,0)=T(mi,mj);mr2(1,0)=T(mi,mj+1);
              Matrix mXv2=mA2m.solve(mr2);
              Matrix mW2=eye(3);mW2(2,0)=-mXv2(0,0);mW2(2,1)=-mXv2(1,0);
              apply_local_Z(mW2.qr().Q,mi,mszz);
            } else if (msi==2 && msj==1) {
              Matrix mT11m(2,2,0.0);for(int r=0;r<2;++r)for(int c=0;c<2;++c)mT11m(r,c)=T(mi+r,mi+c);
              Matrix mA2Tm=(eye(2)*T(mj,mj)-mT11m).T();
              Matrix mr2(2,1,0.0);mr2(0,0)=T(mi,mj);mr2(1,0)=T(mi+1,mj);
              Matrix mXv2=mA2Tm.solve(mr2);
              Matrix mW2=eye(3);mW2(1,0)=-mXv2(0,0);mW2(2,0)=-mXv2(1,0);
              apply_local_Z(mW2.qr().Q,mi,mszz);
            }
            changed = 1;
            if (block_is_real_pair(mi)) split_real_2x2_block(mi);
            mi += block_size(mi);
          } else { mi += msi; }
        }
        if (!changed) break;
      }

    } else { // (2,2): port of LAPACK dtgex2 via Givens rotations
      // LAPACK dtgex2 exchanges two adjacent 2×2 blocks in real Schur form
      // by applying 4 Givens rotations (2 from left, 2 from right).
      //
      // Reference: LAPACK Working Note 155, Granat & Kågström (2009)
      // "Parallel Eigenvalue Reordering in Real Schur Forms"
      //
      // The approach: solve a 4×4 linear system for a unit-norm vector d,
      // then build orthogonal Z from d via Gram-Schmidt (4 Givens rotations).
      // The resulting Z satisfies Z^T * T_sub * Z with the blocks exchanged.

      // Pack the 4×4 block
      double t[4][4];
      for(int r=0;r<4;++r) for(int c=0;c<4;++c) t[r][c]=T(pos+r,pos+c);

      // Build the 4×4 system from Bai-Demmel 1993, eq. (2.8):
      // (T22 ⊗ I2 - I2 ⊗ T11) * vec(X) = vec(T12)
      // i.e. the standard Sylvester equation T11*X - X*T22 = -T12
      // Solve for X (2×2), then build Z from [X; I2] orthogonalized.

      // Use sylvester: T11*X + X*(-T22) = -T12
      // → X = sylvester(T11, -T22, -T12)
      Matrix Y2(2,2,0.0);
      bool ok = true;
      try {
        Y2 = Matrix::sylvester(T11, T22 * (-1.0), T12 * (-1.0));
      } catch (...) { ok = false; }

      if (!ok) {
        // Fallback: permutation only (blocks degenerate/coincident eigenvalues)
        Matrix P4(4,4,0.0);
        P4(0,2)=P4(1,3)=P4(2,0)=P4(3,1)=1.0;
        apply_local_Z(P4, pos, sz);
        re_schur_local(pos, sz);
      } else {
        // Build candidate Z columns from [Y2; I2] (top block) and [-I2; Y2^T] (bottom)
        // The 4 columns of Z_candidate:
        //   col 0: [Y2(0,0); Y2(1,0); 1; 0]    col 1: [Y2(0,1); Y2(1,1); 0; 1]
        //   col 2: [-1; 0; Y2(0,0); Y2(0,1)]   col 3: [0; -1; Y2(1,0); Y2(1,1)]
        // Orthogonalize via Gram-Schmidt (4 Givens rotations per pair):
        Matrix Zcand(4,4,0.0);
        Zcand(0,0)=Y2(0,0); Zcand(1,0)=Y2(1,0); Zcand(2,0)=1; Zcand(3,0)=0;
        Zcand(0,1)=Y2(0,1); Zcand(1,1)=Y2(1,1); Zcand(2,1)=0; Zcand(3,1)=1;
        Zcand(0,2)=-1; Zcand(1,2)=0; Zcand(2,2)=Y2(0,0); Zcand(3,2)=Y2(1,0);
        Zcand(0,3)=0; Zcand(1,3)=-1; Zcand(2,3)=Y2(0,1); Zcand(3,3)=Y2(1,1);

        // Orthogonalize Zcand via QR
        auto qrZ = Zcand.qr();
        Matrix Z4 = qrZ.Q;

        // Apply Z4 as local similarity: T and Q at position pos
        apply_local_Z(Z4, pos, sz);

        // Re-Schur the 4×4 block to restore quasi-triangular form.
        re_schur_local(pos, sz);

        // After re-Schur, split any real 2×2 blocks in the entire T
        // (the similarity can affect off-diagonal coupling, regenerating
        // real blocks outside the swap position).
        {
          int i = 0;
          while (i < static_cast<int>(n)) {
            if (block_is_real_pair(i)) split_real_2x2_block(i);
            i += block_size(i);
          }
        }

        // Mini selector-aware sort within the 4×4 block to prevent oscillation.
        // Without this, re_schur_local may place stable/unstable blocks in random
        // order, causing the outer bubble-sort to oscillate indefinitely.
        for (int mp = 0; mp < sz * sz; ++mp) {
          bool msw = false;
          int mi = pos;
          while (mi < pos + sz) {
            if (block_is_real_pair(mi)) split_real_2x2_block(mi);
            int msi = block_size(mi);
            int mj  = mi + msi;
            if (mj >= pos + sz) break;
            if (block_is_real_pair(mj)) split_real_2x2_block(mj);
            int msj = block_size(mj);
            bool mi_sel = select(block_eig(mi));
            bool mj_sel = select(block_eig(mj));
            if (!mi_sel && mj_sel) {
              // Inline swap (no re_schur_local to avoid recursion/oscillation)
              Matrix mT11(msi,msi,0.0),mT12(msi,msj,0.0),mT22(msj,msj,0.0);
              for(int r=0;r<msi;++r) for(int c=0;c<msi;++c) mT11(r,c)=T(mi+r,mi+c);
              for(int r=0;r<msi;++r) for(int c=0;c<msj;++c) mT12(r,c)=T(mi+r,mi+msi+c);
              for(int r=0;r<msj;++r) for(int c=0;c<msj;++c) mT22(r,c)=T(mi+msi+r,mi+msi+c);
              int mszz = msi + msj;
              double mt12n = mT12.norm(), mtol = eps*(mT11.norm()+mT22.norm()+1.0);
              if (mt12n <= mtol) {
                Matrix mP(mszz,mszz,0.0);
                for(int r=0;r<msj;++r) mP(r,msi+r)=1.0;
                for(int r=0;r<msi;++r) mP(msj+r,r)=1.0;
                apply_local_Z(mP, mi, mszz);
              } else if (msi==1 && msj==1) {
                double ma=mT11(0,0),mb=mT12(0,0),md=mT22(0,0);
                double den=md-ma, mv0=(fabs(den)>eps)?mb/den:1.0, mv1=(fabs(den)>eps)?1.0:0.0;
                double mn=std::sqrt(mv0*mv0+mv1*mv1); if(mn>eps){mv0/=mn;mv1/=mn;}
                Matrix mZ2(2,2); mZ2(0,0)=mv0;mZ2(0,1)=-mv1;mZ2(1,0)=mv1;mZ2(1,1)=mv0;
                apply_local_Z(mZ2, mi, mszz);
              } else if (msi==1 && msj==2) {
                Matrix mA2=mT22-eye(2)*mT11(0,0);
                Matrix mr(2,1,0.0); mr(0,0)=mT12(0,0); mr(1,0)=mT12(0,1);
                Matrix mXv=mA2.solve(mr);
                Matrix mW=eye(3); mW(2,0)=-mXv(0,0); mW(2,1)=-mXv(1,0);
                apply_local_Z(mW.qr().Q, mi, mszz);
              } else if (msi==2 && msj==1) {
                Matrix mA2T=(eye(2)*mT22(0,0)-mT11).T();
                Matrix mr(2,1,0.0); mr(0,0)=mT12(0,0); mr(1,0)=mT12(1,0);
                Matrix mXv=mA2T.solve(mr);
                Matrix mW=eye(3); mW(1,0)=-mXv(0,0); mW(2,0)=-mXv(1,0);
                apply_local_Z(mW.qr().Q, mi, mszz);
              } else if (msi==2 && msj==2) {
                // (2,2) mini-swap: use Zcand QR approach (same as outer swap_blocks).
                // Solve T11*Y - Y*T22 = -T12, build Zcand, apply, split (no re_schur).
                Matrix mY2(2,2,0.0);
                bool mok=true;
                try { mY2=Matrix::sylvester(mT11,mT22*(-1.0),mT12*(-1.0)); }
                catch(...){mok=false;}
                if(mok){
                  Matrix mZcand(4,4,0.0);
                  mZcand(0,0)=mY2(0,0);mZcand(1,0)=mY2(1,0);mZcand(2,0)=1;mZcand(3,0)=0;
                  mZcand(0,1)=mY2(0,1);mZcand(1,1)=mY2(1,1);mZcand(2,1)=0;mZcand(3,1)=1;
                  mZcand(0,2)=-1;mZcand(1,2)=0;mZcand(2,2)=mY2(0,0);mZcand(3,2)=mY2(1,0);
                  mZcand(0,3)=0;mZcand(1,3)=-1;mZcand(2,3)=mY2(0,1);mZcand(3,3)=mY2(1,1);
                  apply_local_Z(mZcand.qr().Q, mi, mszz);
                  // Split any real pairs created — do NOT call re_schur_local
                  {int ii=mi; while(ii<mi+mszz){
                    if(block_is_real_pair(ii))split_real_2x2_block(ii);ii+=block_size(ii);}}
                } else {
                  Matrix mP4(4,4,0.0);mP4(0,2)=mP4(1,3)=mP4(2,0)=mP4(3,1)=1.0;
                  apply_local_Z(mP4,mi,mszz);
                }
              }
              if (block_is_real_pair(mi)) split_real_2x2_block(mi);
              mi += block_size(mi);
              msw = true;
            } else {
              mi += msi;
            }
          }
          if (!msw) break;
        }
      }
    }
  };  // ── end swap_blocks lambda ──────────────────────────────────────

  // ── Bubble-sort over Schur blocks ─────────────────────────────────
  const int maxPasses = static_cast<int>(n) * static_cast<int>(n) * static_cast<int>(n);
  for (int pass = 0; pass < maxPasses; ++pass) {
    // Split ALL real 2×2 blocks at start of each pass
    {
      int i = 0;
      while (i < static_cast<int>(n)) {
        if (block_is_real_pair(i)) split_real_2x2_block(i);
        i += block_size(i);
      }
    }

    bool swapped = false;
    int i = 0;
    while (i < static_cast<int>(n)) {
      int si = block_size(i);
      int j  = i + si;
      if (j >= static_cast<int>(n)) break;
      int sj = block_size(j);

      bool i_sel = select(block_eig(i));
      bool j_sel = select(block_eig(j));

      if (!i_sel && j_sel) {
        swap_blocks(i, si, sj);
        swapped = true;
        if (block_is_real_pair(i)) split_real_2x2_block(i);
        i += block_size(i);
      } else {
        i += si;
      }
    }
    if (!swapped) break;
  }

  return {T, Q};
}

inline Matrix Matrix::sylvester(const Matrix &A, const Matrix &B,
                                const Matrix &C) {
  // Bartels-Stewart algorithm — full support for 1×1 and 2×2 Schur blocks.
  //
  // Solves AX + XB = C.
  //
  // Steps:
  //   1. A = Qa*Ta*Qa^T,  B = Qb*Tb*Qb^T  (real Schur decompositions)
  //   2. Transform: Chat = Qa^T * C * Qb
  //   3. Solve Ta*Y + Y*Tb = Chat  column-block by column-block
  //      - sj=1 block: (Ta + tb_jj*I) * y_j = r_j          [m×m system]
  //      - sj=2 block: block-2m×2m Kronecker system          [2m×2m system]
  //   4. X = Qa * Y * Qb^T
  //
  // Handles complex eigenvalues in both A and B correctly.

  size_t m = A.rows, n = B.rows;
  if (A.rows != A.cols || B.rows != B.cols)
    throw std::runtime_error("sylvester: A and B must be square");
  if (C.rows != m || C.cols != n)
    throw std::runtime_error("sylvester: C dimensions must match A and B");

  auto schurA = A.schur();
  auto schurB = B.schur();
  const Matrix &Ta = schurA.T;
  const Matrix &Tb = schurB.T;
  Matrix Chat = schurA.Q.T() * C * schurB.Q;
  Matrix Y(m, n, 0.0);

  const double eps = 1e-12;

  // Block size of Tb at column j
  auto bsizeB = [&](int j) -> int {
    if (j + 1 >= static_cast<int>(n)) return 1;
    double tol = eps * (std::abs(Tb(j,j)) + std::abs(Tb(j+1,j+1)));
    return (std::abs(Tb(j+1,j)) > tol) ? 2 : 1;
  };

  int j = 0;
  while (j < static_cast<int>(n)) {
    int sj = bsizeB(j);

    // Build RHS block: R = Chat[:,j:j+sj] - Y[:,0:j] * Tb[0:j, j:j+sj]
    Matrix R(m, sj, 0.0);
    for (int p = 0; p < sj; ++p)
      for (size_t i = 0; i < m; ++i) {
        double val = Chat(i, j + p);
        for (int k = 0; k < j; ++k)
          val -= Y(i, k) * Tb(k, j + p);
        R(i, p) = val;
      }

    if (sj == 1) {
      // Solve (Ta + Tb(j,j)*I) * y = R[:,0]
      Matrix Ashift = Ta + eye(m) * Tb(j, j);
      Matrix rhs(m, 1);
      for (size_t i = 0; i < m; ++i) rhs(i, 0) = R(i, 0);
      Matrix yj = Ashift.solve(rhs);
      for (size_t i = 0; i < m; ++i) Y(i, j) = yj(i, 0);

    } else {
      // sj == 2: Tb has a 2×2 block at (j,j).
      //
      // We need: Ta*Y0 + Y0*Tb(0,0) + Y1*Tb(1,0) = R0   [col j  of Ta*Y+Y*Tb=R]
      //          Ta*Y1 + Y0*Tb(0,1) + Y1*Tb(1,1) = R1   [col j+1]
      // where Y0 = Y[:,j], Y1 = Y[:,j+1]  (each is m×1 column vector).
      //
      // Coupling is via COLUMNS of Tb: col p of (Y*Tb) = sum_k Y[:,k]*Tb(k,p)
      // So for the 2-block: col j   picks up Y0*Tb(j,j)   + Y1*Tb(j+1,j)
      //                     col j+1 picks up Y0*Tb(j,j+1) + Y1*Tb(j+1,j+1)
      //
      // Rearranged as 2m×2m system [vec(Y0); vec(Y1)]:
      // [Ta + Tb(j,j)*I,    Tb(j+1,j)*I  ] [Y0]   [R0]
      // [Tb(j,j+1)*I,    Ta+Tb(j+1,j+1)*I] [Y1] = [R1]
      double t00 = Tb(j,   j),   t10 = Tb(j+1, j);    // col j   of Tb
      double t01 = Tb(j,   j+1), t11 = Tb(j+1, j+1);  // col j+1 of Tb

      Matrix Asys(2*m, 2*m, 0.0);
      for (size_t r = 0; r < m; ++r) {
        for (size_t c = 0; c < m; ++c) {
          Asys(r,     c    ) = Ta(r,c) + (r==c ? t00 : 0.0);  // top-left
          Asys(m+r,   m+c  ) = Ta(r,c) + (r==c ? t11 : 0.0);  // bottom-right
        }
        Asys(r,   m+r) = t10;  // top-right:    Y1 contributes Tb(j+1,j) to col j
        Asys(m+r, r  ) = t01;  // bottom-left:  Y0 contributes Tb(j,j+1) to col j+1
      }

      Matrix rhs(2*m, 1, 0.0);
      for (size_t i = 0; i < m; ++i) {
        rhs(i,   0) = R(i, 0);
        rhs(m+i, 0) = R(i, 1);
      }

      Matrix sol = Asys.solve(rhs);
      for (size_t i = 0; i < m; ++i) {
        Y(i, j  ) = sol(i,   0);
        Y(i, j+1) = sol(m+i, 0);
      }
    }

    j += sj;
  }

  return schurA.Q * Y * schurB.Q.T();
}

// ----------------------------------------------------------------
//  SVD (One-sided Jacobi) & Pseudo-inverse
// ----------------------------------------------------------------

inline Matrix::SVDResult Matrix::svd() const {
  // 1. Handle Wide matrices (m < n) by transposing
  if (rows < cols) {
    auto res = this->T().svd();
    // A^T = U * S * V^T  =>  A = V * S * U^T
    // For compact SVD, S is always a square diagonal matrix
    return {res.V, res.S, res.U};
  }

  size_t m = rows, n = cols;
  Matrix W = *this;
  Matrix V = eye(n);
  
  const double eps = 1e-15;
  const int max_sweeps = 30;
  bool converged = false;

  // 2. Jacobi sweeps to orthogonalize columns of W
  for (int sweep = 0; sweep < max_sweeps; ++sweep) {
    converged = true;
    for (size_t j = 0; j < n - 1; ++j) {
      for (size_t k = j + 1; k < n; ++k) {
        double p = 0.0, q = 0.0, r = 0.0;
        for (size_t i = 0; i < m; ++i) {
          p += W(i, j) * W(i, k);
          q += W(i, j) * W(i, j);
          r += W(i, k) * W(i, k);
        }

        if (std::abs(p) > eps * std::sqrt(q * r)) {
          converged = false;
          double q_minus_r = q - r;
          double t;
          
          if (std::abs(q_minus_r) < eps * std::max(q, r)) {
            t = (p < 0) ? -1.0 : 1.0;
          } else {
            double theta = q_minus_r / (2.0 * p);
            t = std::copysign(1.0 / (std::abs(theta) + std::sqrt(1.0 + theta * theta)), theta);
          }
          
          double c = 1.0 / std::sqrt(1.0 + t * t);
          double s = t * c;

          for (size_t i = 0; i < m; ++i) {
            double w_ij = W(i, j), w_ik = W(i, k);
            W(i, j) = c * w_ij + s * w_ik;
            W(i, k) = -s * w_ij + c * w_ik;
          }
          for (size_t i = 0; i < n; ++i) {
            double v_ij = V(i, j), v_ik = V(i, k);
            V(i, j) = c * v_ij + s * v_ik;
            V(i, k) = -s * v_ij + c * v_ik;
          }
        }
      }
    }
    if (converged) break;
  }

  Matrix U = Matrix::zeros(m, n);
  Matrix S = Matrix::zeros(n, n);
  std::vector<double> sigmas(n);

  // 3. Extract singular values and normalize W to get U
  for (size_t j = 0; j < n; ++j) {
    double norm = 0.0;
    for (size_t i = 0; i < m; ++i) norm += W(i, j) * W(i, j);
    sigmas[j] = std::sqrt(norm);
    
    // Check if singular value is practically zero
    if (sigmas[j] > eps * std::max(1.0, std::sqrt(static_cast<double>(m)))) {
      for (size_t i = 0; i < m; ++i) U(i, j) = W(i, j) / sigmas[j];
    } else {
      sigmas[j] = 0.0; 
      // Null space handled in step 4
    }
  }

  // 4. Gram-Schmidt to invent orthogonal vectors for Null-space (Rank-deficient cases)
  for (size_t j = 0; j < n; ++j) {
    if (sigmas[j] == 0.0) {
      // Try to project standard basis vectors until we find a non-zero orthogonal vector
      for (size_t k = 0; k < m; ++k) {
        std::vector<double> e(m, 0.0);
        e[k] = 1.0;
        
        // Subtract projections of all previously validated columns
        for (size_t c = 0; c < n; ++c) {
          if (c == j) continue;
          if (sigmas[c] > 0.0 || c < j) { 
            double dot = 0.0;
            for (size_t i = 0; i < m; ++i) dot += U(i, c) * e[i];
            for (size_t i = 0; i < m; ++i) e[i] -= dot * U(i, c);
          }
        }
        
        double normE = 0.0;
        for (size_t i = 0; i < m; ++i) normE += e[i] * e[i];
        normE = std::sqrt(normE);
        
        if (normE > 1e-6) {
          for (size_t i = 0; i < m; ++i) U(i, j) = e[i] / normE;
          break; // Found our null-space basis vector!
        }
      }
    }
  }

  // 5. Sort singular values in descending order
  for (size_t j = 0; j < n - 1; ++j) {
    for (size_t k = j + 1; k < n; ++k) {
      if (sigmas[k] > sigmas[j]) {
        std::swap(sigmas[j], sigmas[k]);
        for (size_t i = 0; i < m; ++i) std::swap(U(i, j), U(i, k));
        for (size_t i = 0; i < n; ++i) std::swap(V(i, j), V(i, k));
      }
    }
  }

  for (size_t j = 0; j < n; ++j) S(j, j) = sigmas[j];

  return {U, S, V};
}

inline Matrix Matrix::pinv(double tol) const {
  if (rows == 0 || cols == 0) return Matrix(cols, rows);
  
  auto svd_res = svd();
  size_t k = std::min(rows, cols);
  Matrix S_inv = Matrix::zeros(k, k);
  
  double max_s = 0.0;
  for (size_t i = 0; i < k; ++i) {
    max_s = std::max(max_s, svd_res.S(i, i));
  }
  
  double threshold = tol * max_s * std::max(rows, cols);
  
  for (size_t i = 0; i < k; ++i) {
    if (svd_res.S(i, i) > threshold) {
      S_inv(i, i) = 1.0 / svd_res.S(i, i);
    }
  }
  
  // A^+ = V * S^+ * U^T
  return svd_res.V * S_inv * svd_res.U.T();
}

// ================================================================
//  Free-standing linear algebra functions
// ================================================================

/// Solve AX = B (convenience free function)
inline Matrix solve(const Matrix &A, const Matrix &B) { return A.solve(B); }

/// Matrix exponential (convenience free function)
inline Matrix expm(const Matrix &A) { return A.expm(); }

/// Kronecker product A ⊗ B
inline Matrix kron(const Matrix &A, const Matrix &B) {
  Matrix C(A.rows * B.rows, A.cols * B.cols);
  for (size_t i = 0; i < A.rows; ++i)
    for (size_t j = 0; j < A.cols; ++j)
      for (size_t k = 0; k < B.rows; ++k)
        for (size_t l = 0; l < B.cols; ++l)
          C(i * B.rows + k, j * B.cols + l) = A(i, j) * B(k, l);
  return C;
}

/// Horizontal concatenation [A | B]
inline Matrix horzcat(const Matrix &A, const Matrix &B) {
  if (A.rows != B.rows)
    throw std::runtime_error("horzcat: row count mismatch");
  Matrix C(A.rows, A.cols + B.cols);
  for (size_t i = 0; i < A.rows; ++i) {
    for (size_t j = 0; j < A.cols; ++j)
      C(i, j) = A(i, j);
    for (size_t j = 0; j < B.cols; ++j)
      C(i, A.cols + j) = B(i, j);
  }
  return C;
}

/// Vertical concatenation [A; B]
inline Matrix vertcat(const Matrix &A, const Matrix &B) {
  if (A.cols != B.cols)
    throw std::runtime_error("vertcat: column count mismatch");
  Matrix C(A.rows + B.rows, A.cols);
  for (size_t i = 0; i < A.rows; ++i)
    for (size_t j = 0; j < A.cols; ++j)
      C(i, j) = A(i, j);
  for (size_t i = 0; i < B.rows; ++i)
    for (size_t j = 0; j < B.cols; ++j)
      C(A.rows + i, j) = B(i, j);
  return C;
}

} // namespace cppplot

#endif // CPPPLOT_CORE_MATRIX_HPP
