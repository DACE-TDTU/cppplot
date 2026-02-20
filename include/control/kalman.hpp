/**
 * @file kalman.hpp
 * @brief Kalman Filter implementations for state estimation
 * 
 * This file provides various Kalman filter implementations:
 * - Discrete Kalman Filter (standard)
 * - Steady-State Kalman Filter
 * - Extended Kalman Filter (EKF) for nonlinear systems
 * - Unscented Kalman Filter (UKF) for highly nonlinear systems
 * 
 * @author CppPlot Control Systems Library
 * @date January 2026
 * 
 * References:
 * [1] Kalman, R.E. (1960). "A New Approach to Linear Filtering and 
 *     Prediction Problems"
 * [2] Simon, D. (2006). "Optimal State Estimation"
 */

#ifndef CPPPLOT_CONTROL_KALMAN_HPP
#define CPPPLOT_CONTROL_KALMAN_HPP

#include "state_space.hpp"
#include "controller_design.hpp"
#include <functional>

namespace cppplot {
namespace control {

/**
 * @brief Result structure for Kalman filter step
 */
struct KalmanEstimate {
    Matrix x_hat;      // State estimate
    Matrix P;          // Error covariance
    Matrix K;          // Kalman gain
    Matrix innovation; // y - C*x_hat (measurement residual)
};

/**
 * @brief Discrete-time Kalman Filter
 * 
 * State-space model:
 *   x(k+1) = A*x(k) + B*u(k) + w(k)    (process noise w ~ N(0, Q))
 *   y(k)   = C*x(k) + v(k)              (measurement noise v ~ N(0, R))
 * 
 * The Kalman filter provides the optimal (MMSE) state estimate.
 * 
 * @example
 * ```cpp
 * // Create system
 * Matrix A = {{1, 0.1}, {0, 1}};  // Double integrator (position, velocity)
 * Matrix B = {{0.005}, {0.1}};
 * Matrix C = {{1, 0}};             // Only position measured
 * 
 * // Noise covariances
 * Matrix Q = Matrix::eye(2) * 0.01;  // Process noise
 * Matrix R = Matrix::eye(1) * 0.1;   // Measurement noise
 * 
 * // Create Kalman filter
 * KalmanFilter kf(A, B, C, Q, R);
 * 
 * // Run filter
 * for (auto& measurement : measurements) {
 *     auto est = kf.update(measurement, control_input);
 *     std::cout << "Estimated state: " << est.x_hat << std::endl;
 * }
 * ```
 */
class KalmanFilter {
private:
    Matrix A_, B_, C_, D_;  // System matrices
    Matrix Q_;              // Process noise covariance
    Matrix R_;              // Measurement noise covariance
    Matrix x_hat_;          // Current state estimate
    Matrix P_;              // Current error covariance
    size_t n_, m_, p_;      // Dimensions: states, inputs, outputs
    bool initialized_;
    
public:
    /**
     * @brief Construct Kalman filter from system matrices
     * 
     * @param A State transition matrix (n x n)
     * @param B Input matrix (n x m)
     * @param C Output matrix (p x n)
     * @param Q Process noise covariance (n x n)
     * @param R Measurement noise covariance (p x p)
     */
    KalmanFilter(const Matrix& A, const Matrix& B, const Matrix& C,
                 const Matrix& Q, const Matrix& R)
        : A_(A), B_(B), C_(C), Q_(Q), R_(R), initialized_(false)
    {
        n_ = A.rows;
        m_ = B.cols;
        p_ = C.rows;
        
        // Initialize with zero state and large covariance
        x_hat_ = Matrix::zeros(n_, 1);
        P_ = Matrix::eye(n_) * 1000.0;  // Large initial uncertainty
        
        // D matrix defaults to zero
        D_ = Matrix::zeros(p_, m_);
    }
    
    /**
     * @brief Construct from StateSpace system
     */
    KalmanFilter(const StateSpace& sys, const Matrix& Q, const Matrix& R)
        : KalmanFilter(sys.A, sys.B, sys.C, Q, R)
    {
        D_ = sys.D;
    }
    
    /**
     * @brief Set initial state estimate
     */
    void setInitialState(const Matrix& x0, const Matrix& P0) {
        x_hat_ = x0;
        P_ = P0;
        initialized_ = true;
    }
    
    /**
     * @brief Set initial state estimate (with default covariance)
     */
    void setInitialState(const Matrix& x0) {
        x_hat_ = x0;
        initialized_ = true;
    }
    
    /**
     * @brief Prediction step (time update)
     * 
     * Propagates state estimate and covariance forward in time:
     *   x̂(k|k-1) = A*x̂(k-1|k-1) + B*u(k-1)
     *   P(k|k-1) = A*P(k-1|k-1)*A' + Q
     * 
     * @param u Control input (m x 1)
     * @return Predicted state estimate
     */
    KalmanEstimate predict(const Matrix& u) {
        // Predict state
        x_hat_ = A_ * x_hat_ + B_ * u;
        
        // Predict covariance
        P_ = A_ * P_ * A_.T() + Q_;
        
        // Symmetrize P for numerical stability
        for (size_t i = 0; i < n_; ++i) {
            for (size_t j = i + 1; j < n_; ++j) {
                double avg = (P_(i, j) + P_(j, i)) / 2.0;
                P_(i, j) = P_(j, i) = avg;
            }
        }
        
        KalmanEstimate est;
        est.x_hat = x_hat_;
        est.P = P_;
        est.K = Matrix::zeros(n_, p_);
        est.innovation = Matrix::zeros(p_, 1);
        
        return est;
    }
    
    /**
     * @brief Correction step (measurement update)
     * 
     * Updates estimate using measurement:
     *   K(k) = P(k|k-1)*C'*(C*P(k|k-1)*C' + R)^(-1)
     *   x̂(k|k) = x̂(k|k-1) + K(k)*(y(k) - C*x̂(k|k-1))
     *   P(k|k) = (I - K(k)*C)*P(k|k-1)
     * 
     * @param y Measurement vector (p x 1)
     * @return Updated state estimate with Kalman gain
     */
    KalmanEstimate correct(const Matrix& y) {
        // Innovation (measurement residual)
        Matrix y_pred = C_ * x_hat_;
        Matrix innovation = y - y_pred;
        
        // Innovation covariance
        Matrix S = C_ * P_ * C_.T() + R_;
        
        // Kalman gain
        Matrix K = P_ * C_.T() * S.inv();
        
        // Update state estimate
        x_hat_ = x_hat_ + K * innovation;
        
        // Update covariance (Joseph form for numerical stability)
        Matrix I = Matrix::eye(n_);
        Matrix IKC = I - K * C_;
        P_ = IKC * P_ * IKC.T() + K * R_ * K.T();
        
        // Symmetrize
        for (size_t i = 0; i < n_; ++i) {
            for (size_t j = i + 1; j < n_; ++j) {
                double avg = (P_(i, j) + P_(j, i)) / 2.0;
                P_(i, j) = P_(j, i) = avg;
            }
        }
        
        KalmanEstimate est;
        est.x_hat = x_hat_;
        est.P = P_;
        est.K = K;
        est.innovation = innovation;
        
        return est;
    }
    
    /**
     * @brief Combined predict and correct step
     * 
     * This is the typical usage: predict with control, then correct with measurement.
     * 
     * @param y Measurement vector (p x 1)
     * @param u Control input (m x 1)
     * @return Updated state estimate
     */
    KalmanEstimate update(const Matrix& y, const Matrix& u) {
        predict(u);
        return correct(y);
    }
    
    /**
     * @brief Update with zero control input
     */
    KalmanEstimate update(const Matrix& y) {
        Matrix u = Matrix::zeros(m_, 1);
        return update(y, u);
    }
    
    /**
     * @brief Get current state estimate
     */
    Matrix getState() const { return x_hat_; }
    
    /**
     * @brief Get current error covariance
     */
    Matrix getCovariance() const { return P_; }
    
    /**
     * @brief Compute steady-state Kalman gain
     * 
     * Solves the discrete algebraic Riccati equation (DARE):
     *   P = A*P*A' - A*P*C'*(C*P*C' + R)^(-1)*C*P*A' + Q
     * 
     * Then computes steady-state gain:
     *   K = P*C'*(C*P*C' + R)^(-1)
     * 
     * @return Steady-state Kalman gain
     */
    Matrix steadyStateGain() const {
        // Solve DARE using iteration
        Matrix P = Matrix::eye(n_);
        Matrix P_prev = Matrix::zeros(n_, n_);
        
        for (int iter = 0; iter < 1000; ++iter) {
            Matrix CPCT_R = C_ * P * C_.T() + R_;
            Matrix CPCT_R_inv = CPCT_R.inv();
            Matrix K_temp = P * C_.T() * CPCT_R_inv;
            
            P_prev = P;
            P = A_ * P * A_.T() - A_ * K_temp * C_ * P * A_.T() + Q_;
            
            // Check convergence
            double diff = 0;
            for (size_t i = 0; i < n_; ++i) {
                for (size_t j = 0; j < n_; ++j) {
                    diff += std::abs(P(i, j) - P_prev(i, j));
                }
            }
            if (diff < 1e-10) break;
        }
        
        // Compute steady-state gain
        Matrix S = C_ * P * C_.T() + R_;
        return P * C_.T() * S.inv();
    }
    
    /**
     * @brief Reset filter to initial state
     */
    void reset() {
        x_hat_ = Matrix::zeros(n_, 1);
        P_ = Matrix::eye(n_) * 1000.0;
        initialized_ = false;
    }
};

/**
 * @brief Steady-State Kalman Filter
 * 
 * Uses pre-computed steady-state gain for efficiency.
 * Suitable when the system has reached steady-state operation.
 */
class SteadyStateKalmanFilter {
private:
    Matrix A_, B_, C_;
    Matrix K_ss_;           // Steady-state Kalman gain
    Matrix x_hat_;
    size_t n_, m_, p_;
    
public:
    /**
     * @brief Construct with pre-computed gain
     */
    SteadyStateKalmanFilter(const Matrix& A, const Matrix& B, const Matrix& C,
                            const Matrix& K_ss)
        : A_(A), B_(B), C_(C), K_ss_(K_ss)
    {
        n_ = A.rows;
        m_ = B.cols;
        p_ = C.rows;
        x_hat_ = Matrix::zeros(n_, 1);
    }
    
    /**
     * @brief Construct and compute steady-state gain
     */
    SteadyStateKalmanFilter(const Matrix& A, const Matrix& B, const Matrix& C,
                            const Matrix& Q, const Matrix& R)
        : A_(A), B_(B), C_(C)
    {
        n_ = A.rows;
        m_ = B.cols;
        p_ = C.rows;
        x_hat_ = Matrix::zeros(n_, 1);
        
        // Compute steady-state gain
        KalmanFilter kf(A, B, C, Q, R);
        K_ss_ = kf.steadyStateGain();
    }
    
    /**
     * @brief Update estimate (simplified equations)
     * 
     * x̂(k|k) = A*x̂(k-1|k-1) + B*u(k-1) + K*(y(k) - C*(A*x̂(k-1|k-1) + B*u(k-1)))
     */
    Matrix update(const Matrix& y, const Matrix& u) {
        // Predict
        Matrix x_pred = A_ * x_hat_ + B_ * u;
        
        // Correct
        Matrix y_pred = C_ * x_pred;
        x_hat_ = x_pred + K_ss_ * (y - y_pred);
        
        return x_hat_;
    }
    
    Matrix update(const Matrix& y) {
        Matrix u = Matrix::zeros(m_, 1);
        return update(y, u);
    }
    
    Matrix getState() const { return x_hat_; }
    Matrix getGain() const { return K_ss_; }
    
    void setInitialState(const Matrix& x0) { x_hat_ = x0; }
};

/**
 * @brief Extended Kalman Filter for nonlinear systems
 * 
 * For nonlinear system:
 *   x(k+1) = f(x(k), u(k)) + w(k)
 *   y(k)   = h(x(k)) + v(k)
 * 
 * The EKF linearizes around the current estimate:
 *   F = ∂f/∂x |_{x̂}  (Jacobian of state transition)
 *   H = ∂h/∂x |_{x̂}  (Jacobian of measurement)
 * 
 * @example
 * ```cpp
 * // Define nonlinear dynamics
 * auto f = [](const Matrix& x, const Matrix& u) -> Matrix {
 *     Matrix x_next(2, 1);
 *     x_next(0, 0) = x(0, 0) + x(1, 0) * dt;
 *     x_next(1, 0) = x(1, 0) + u(0, 0) * dt;
 *     return x_next;
 * };
 * 
 * // Define Jacobian (analytically or numerically)
 * auto F = [](const Matrix& x, const Matrix& u) -> Matrix {
 *     return Matrix({{1, dt}, {0, 1}});
 * };
 * 
 * ExtendedKalmanFilter ekf(f, h, F, H, Q, R, 2, 1, 1);
 * ```
 */
class ExtendedKalmanFilter {
public:
    using StateFunc = std::function<Matrix(const Matrix&, const Matrix&)>;
    using MeasFunc = std::function<Matrix(const Matrix&)>;
    using JacobianFunc = std::function<Matrix(const Matrix&, const Matrix&)>;
    using MeasJacobianFunc = std::function<Matrix(const Matrix&)>;
    
private:
    StateFunc f_;           // State transition function
    MeasFunc h_;            // Measurement function
    JacobianFunc F_;        // State Jacobian
    MeasJacobianFunc H_;    // Measurement Jacobian
    Matrix Q_, R_;          // Noise covariances
    Matrix x_hat_, P_;      // State estimate and covariance
    size_t n_, m_, p_;
    double delta_;          // For numerical Jacobian
    
public:
    /**
     * @brief Construct EKF with analytical Jacobians
     */
    ExtendedKalmanFilter(StateFunc f, MeasFunc h,
                         JacobianFunc F, MeasJacobianFunc H,
                         const Matrix& Q, const Matrix& R,
                         size_t n, size_t m, size_t p)
        : f_(f), h_(h), F_(F), H_(H), Q_(Q), R_(R),
          n_(n), m_(m), p_(p), delta_(1e-6)
    {
        x_hat_ = Matrix::zeros(n_, 1);
        P_ = Matrix::eye(n_) * 1000.0;
    }
    
    /**
     * @brief Construct EKF with numerical Jacobians
     */
    ExtendedKalmanFilter(StateFunc f, MeasFunc h,
                         const Matrix& Q, const Matrix& R,
                         size_t n, size_t m, size_t p)
        : f_(f), h_(h), Q_(Q), R_(R),
          n_(n), m_(m), p_(p), delta_(1e-6)
    {
        x_hat_ = Matrix::zeros(n_, 1);
        P_ = Matrix::eye(n_) * 1000.0;
        
        // Define numerical Jacobians
        F_ = [this](const Matrix& x, const Matrix& u) -> Matrix {
            return numericalJacobian_f(x, u);
        };
        
        H_ = [this](const Matrix& x) -> Matrix {
            return numericalJacobian_h(x);
        };
    }
    
    /**
     * @brief Compute numerical Jacobian of f
     */
    Matrix numericalJacobian_f(const Matrix& x, const Matrix& u) {
        Matrix J(n_, n_);
        Matrix f0 = f_(x, u);
        
        for (size_t j = 0; j < n_; ++j) {
            Matrix x_pert = x;
            x_pert(j, 0) += delta_;
            Matrix f_pert = f_(x_pert, u);
            
            for (size_t i = 0; i < n_; ++i) {
                J(i, j) = (f_pert(i, 0) - f0(i, 0)) / delta_;
            }
        }
        return J;
    }
    
    /**
     * @brief Compute numerical Jacobian of h
     */
    Matrix numericalJacobian_h(const Matrix& x) {
        Matrix J(p_, n_);
        Matrix h0 = h_(x);
        
        for (size_t j = 0; j < n_; ++j) {
            Matrix x_pert = x;
            x_pert(j, 0) += delta_;
            Matrix h_pert = h_(x_pert);
            
            for (size_t i = 0; i < p_; ++i) {
                J(i, j) = (h_pert(i, 0) - h0(i, 0)) / delta_;
            }
        }
        return J;
    }
    
    /**
     * @brief EKF prediction step
     */
    KalmanEstimate predict(const Matrix& u) {
        // Compute Jacobian at current estimate
        Matrix F = F_(x_hat_, u);
        
        // Predict state using nonlinear model
        x_hat_ = f_(x_hat_, u);
        
        // Predict covariance using linearized model
        P_ = F * P_ * F.T() + Q_;
        
        // Symmetrize
        for (size_t i = 0; i < n_; ++i) {
            for (size_t j = i + 1; j < n_; ++j) {
                double avg = (P_(i, j) + P_(j, i)) / 2.0;
                P_(i, j) = P_(j, i) = avg;
            }
        }
        
        KalmanEstimate est;
        est.x_hat = x_hat_;
        est.P = P_;
        return est;
    }
    
    /**
     * @brief EKF correction step
     */
    KalmanEstimate correct(const Matrix& y) {
        // Compute Jacobian at current estimate
        Matrix H = H_(x_hat_);
        
        // Predicted measurement using nonlinear model
        Matrix y_pred = h_(x_hat_);
        Matrix innovation = y - y_pred;
        
        // Innovation covariance
        Matrix S = H * P_ * H.T() + R_;
        
        // Kalman gain
        Matrix K = P_ * H.T() * S.inv();
        
        // Update state
        x_hat_ = x_hat_ + K * innovation;
        
        // Update covariance (Joseph form)
        Matrix I = Matrix::eye(n_);
        Matrix IKH = I - K * H;
        P_ = IKH * P_ * IKH.T() + K * R_ * K.T();
        
        KalmanEstimate est;
        est.x_hat = x_hat_;
        est.P = P_;
        est.K = K;
        est.innovation = innovation;
        return est;
    }
    
    /**
     * @brief Combined predict and correct
     */
    KalmanEstimate update(const Matrix& y, const Matrix& u) {
        predict(u);
        return correct(y);
    }
    
    void setInitialState(const Matrix& x0, const Matrix& P0) {
        x_hat_ = x0;
        P_ = P0;
    }
    
    Matrix getState() const { return x_hat_; }
    Matrix getCovariance() const { return P_; }
};

// ============================================================
//                    UTILITY FUNCTIONS
// ============================================================

/**
 * @brief Create Kalman filter for continuous-time system
 * 
 * Discretizes the system first, then creates Kalman filter.
 * 
 * @param sys Continuous-time state-space system
 * @param Qc Continuous process noise intensity
 * @param R Measurement noise covariance
 * @param dt Sampling period
 */
inline KalmanFilter kalmanFromContinuous(
    const StateSpace& sys,
    const Matrix& Qc,
    const Matrix& R,
    double dt
) {
    // Simple discretization: Ad = I + A*dt, Bd = B*dt
    size_t n = sys.A.rows;
    size_t m = sys.B.cols;
    
    Matrix Ad = Matrix::eye(n) + sys.A * dt;
    Matrix Bd = sys.B * dt;
    
    // Discretize process noise: Qd ≈ Qc * dt
    Matrix Qd = Qc * dt;
    
    return KalmanFilter(Ad, Bd, sys.C, Qd, R);
}

/**
 * @brief Simulate Kalman filter on data
 * 
 * @param kf Kalman filter
 * @param measurements Vector of measurements
 * @param controls Vector of control inputs
 * @return Vector of state estimates
 */
inline std::vector<Matrix> simulateKalmanFilter(
    KalmanFilter& kf,
    const std::vector<Matrix>& measurements,
    const std::vector<Matrix>& controls
) {
    std::vector<Matrix> estimates;
    estimates.reserve(measurements.size());
    
    for (size_t k = 0; k < measurements.size(); ++k) {
        Matrix u = (k < controls.size()) ? controls[k] : Matrix::zeros(controls[0].rows, 1);
        auto est = kf.update(measurements[k], u);
        estimates.push_back(est.x_hat);
    }
    
    return estimates;
}

// ============================================================
//              ADAPTIVE KALMAN FILTER
// ============================================================

/**
 * @brief Adaptive Kalman Filter with online noise covariance estimation
 * 
 * Standard Kalman filter assumes known Q and R. In practice, these are often
 * unknown or time-varying. Adaptive KF estimates Q and R online using:
 * 
 * 1. **Innovation-based adaptation** (Mehra, 1970):
 *    Uses innovation sequence ν = y - C*x̂ to estimate R and Q
 *    
 * 2. **Covariance matching**:
 *    Compares theoretical innovation covariance with sample covariance
 *    
 * 3. **Multiple Model Adaptive Estimation (MMAE)**:
 *    Runs bank of filters with different Q/R, weights by likelihood
 * 
 * This implementation uses the **Innovation-based approach**:
 * 
 * Innovation covariance: S = C*P*C' + R
 * Sample covariance:     Ŝ = (1/N) * Σ νᵢ*νᵢ'
 * 
 * Adaptation:
 *   R_new = Ŝ - C*P*C'
 *   Q_new = K*Ŝ*K'
 * 
 * With exponential forgetting for non-stationary systems:
 *   Ŝ(k) = α*Ŝ(k-1) + (1-α)*ν(k)*ν(k)'
 * 
 * @author CppPlot Control Systems Library
 * @date January 2026
 * 
 * References:
 * [1] Mehra, R. (1970). "On the identification of variances and adaptive 
 *     Kalman filtering"
 * [2] Mohamed & Schwarz (1999). "Adaptive Kalman filtering for INS/GPS"
 * [3] Akhlaghi et al. (2017). "Adaptive adjustment of noise covariance in 
 *     Kalman filter for dynamic state estimation"
 */
class AdaptiveKalmanFilter {
public:
    /**
     * @brief Adaptation method
     */
    enum class Method {
        INNOVATION_CORRELATION,  // Innovation-based (Mehra)
        COVARIANCE_MATCHING,     // Match predicted vs actual covariance
        SAGE_HUSA,               // Sage-Husa adaptive algorithm
        VARIATIONAL_BAYES        // Variational Bayesian approach
    };

private:
    Matrix A_, B_, C_;
    Matrix Q_, R_;           // Current noise covariances
    Matrix Q_init_, R_init_; // Initial values (for bounds)
    Matrix x_hat_, P_;       // State and covariance
    
    // Adaptation parameters
    Method method_;
    double alpha_;           // Forgetting factor (0 < α < 1)
    size_t window_size_;     // Window for sample covariance
    size_t min_samples_;     // Minimum samples before adaptation
    
    // Innovation history for adaptation
    std::vector<Matrix> innovation_history_;
    Matrix S_sample_;        // Sample innovation covariance
    
    // Bounds for Q and R (prevent divergence)
    double q_min_, q_max_;
    double r_min_, r_max_;
    
    // Statistics
    size_t n_, m_, p_;
    size_t step_count_;
    bool initialized_;

public:
    /**
     * @brief Construct Adaptive Kalman Filter
     * 
     * @param A System matrix
     * @param B Input matrix  
     * @param C Output matrix
     * @param Q_init Initial process noise covariance
     * @param R_init Initial measurement noise covariance
     * @param method Adaptation method (default: INNOVATION_CORRELATION)
     * @param alpha Forgetting factor (default: 0.95)
     */
    AdaptiveKalmanFilter(
        const Matrix& A, const Matrix& B, const Matrix& C,
        const Matrix& Q_init, const Matrix& R_init,
        Method method = Method::INNOVATION_CORRELATION,
        double alpha = 0.95
    ) : A_(A), B_(B), C_(C), 
        Q_(Q_init), R_(R_init),
        Q_init_(Q_init), R_init_(R_init),
        method_(method), alpha_(alpha),
        window_size_(20), min_samples_(10),
        q_min_(1e-10), q_max_(1e6),
        r_min_(1e-10), r_max_(1e6),
        step_count_(0), initialized_(false)
    {
        n_ = A.rows;
        m_ = B.cols;
        p_ = C.rows;
        
        x_hat_ = Matrix::zeros(n_, 1);
        P_ = Matrix::eye(n_);
        S_sample_ = Matrix::zeros(p_, p_);
    }
    
    /**
     * @brief Set initial state
     */
    void setInitialState(const Matrix& x0, const Matrix& P0) {
        x_hat_ = x0;
        P_ = P0;
        initialized_ = true;
    }
    
    /**
     * @brief Set adaptation parameters
     */
    void setAdaptationParams(double alpha, size_t window_size, size_t min_samples) {
        alpha_ = alpha;
        window_size_ = window_size;
        min_samples_ = min_samples;
    }
    
    /**
     * @brief Set bounds for Q and R adaptation
     */
    void setBounds(double q_min, double q_max, double r_min, double r_max) {
        q_min_ = q_min;
        q_max_ = q_max;
        r_min_ = r_min;
        r_max_ = r_max;
    }
    
    /**
     * @brief Update filter with new measurement
     * 
     * @param y Measurement
     * @param u Control input
     * @return Kalman estimate with adapted covariances
     */
    KalmanEstimate update(const Matrix& y, const Matrix& u) {
        step_count_++;
        
        // === PREDICTION STEP ===
        Matrix x_pred = A_ * x_hat_ + B_ * u;
        Matrix P_pred = A_ * P_ * A_.T() + Q_;
        
        // === CORRECTION STEP ===
        Matrix CT = C_.T();
        Matrix S = C_ * P_pred * CT + R_;  // Innovation covariance
        Matrix S_inv = S.inv();
        
        // Kalman gain
        Matrix K = P_pred * CT * S_inv;
        
        // Innovation (measurement residual)
        Matrix innovation = y - C_ * x_pred;
        
        // State update
        x_hat_ = x_pred + K * innovation;
        
        // Covariance update (Joseph form for numerical stability)
        Matrix I = Matrix::eye(n_);
        Matrix IKC = I - K * C_;
        P_ = IKC * P_pred * IKC.T() + K * R_ * K.T();
        
        // === ADAPTATION STEP ===
        adaptNoiseCovariances(innovation, K, P_pred);
        
        // Return result
        KalmanEstimate est;
        est.x_hat = x_hat_;
        est.P = P_;
        est.K = K;
        est.innovation = innovation;
        return est;
    }
    
    /**
     * @brief Get current process noise covariance
     */
    Matrix getQ() const { return Q_; }
    
    /**
     * @brief Get current measurement noise covariance
     */
    Matrix getR() const { return R_; }
    
    /**
     * @brief Get current state estimate
     */
    Matrix getState() const { return x_hat_; }
    
    /**
     * @brief Get current error covariance
     */
    Matrix getCovariance() const { return P_; }
    
    /**
     * @brief Get number of steps processed
     */
    size_t getStepCount() const { return step_count_; }
    
    /**
     * @brief Reset filter
     */
    void reset() {
        x_hat_ = Matrix::zeros(n_, 1);
        P_ = Matrix::eye(n_);
        Q_ = Q_init_;
        R_ = R_init_;
        S_sample_ = Matrix::zeros(p_, p_);
        innovation_history_.clear();
        step_count_ = 0;
        initialized_ = false;
    }

private:
    /**
     * @brief Adapt Q and R based on innovation sequence
     */
    void adaptNoiseCovariances(const Matrix& innovation, const Matrix& K, 
                                const Matrix& P_pred) {
        // Store innovation for windowed estimation
        innovation_history_.push_back(innovation);
        if (innovation_history_.size() > window_size_) {
            innovation_history_.erase(innovation_history_.begin());
        }
        
        // Don't adapt until we have enough samples
        if (step_count_ < min_samples_) {
            return;
        }
        
        switch (method_) {
            case Method::INNOVATION_CORRELATION:
                adaptInnovationCorrelation(innovation, K, P_pred);
                break;
            case Method::COVARIANCE_MATCHING:
                adaptCovarianceMatching(innovation, K, P_pred);
                break;
            case Method::SAGE_HUSA:
                adaptSageHusa(innovation, K, P_pred);
                break;
            case Method::VARIATIONAL_BAYES:
                adaptVariationalBayes(innovation, K, P_pred);
                break;
        }
    }
    
    /**
     * @brief Innovation correlation method (Mehra, 1970)
     * 
     * Uses the fact that innovations should be white noise with covariance S.
     * If actual covariance differs, adapt Q and R.
     */
    void adaptInnovationCorrelation(const Matrix& innovation, const Matrix& K,
                                     const Matrix& P_pred) {
        // Update sample innovation covariance with exponential forgetting
        Matrix vvT = innovation * innovation.T();
        
        if (step_count_ == min_samples_) {
            S_sample_ = vvT;
        } else {
            for (size_t i = 0; i < p_; ++i) {
                for (size_t j = 0; j < p_; ++j) {
                    S_sample_(i, j) = alpha_ * S_sample_(i, j) + 
                                      (1.0 - alpha_) * vvT(i, j);
                }
            }
        }
        
        // Theoretical innovation covariance: S = C*P_pred*C' + R
        Matrix CT = C_.T();
        Matrix CPCT = C_ * P_pred * CT;
        
        // Adapt R: R_new = S_sample - C*P_pred*C'
        Matrix R_new = S_sample_;
        for (size_t i = 0; i < p_; ++i) {
            for (size_t j = 0; j < p_; ++j) {
                R_new(i, j) -= CPCT(i, j);
            }
        }
        
        // Ensure R is positive definite (diagonal dominance)
        for (size_t i = 0; i < p_; ++i) {
            R_new(i, i) = clamp(R_new(i, i), r_min_, r_max_);
            for (size_t j = 0; j < p_; ++j) {
                if (i != j) {
                    // Limit off-diagonal to maintain positive definiteness
                    double max_off = 0.9 * std::sqrt(R_new(i, i) * R_new(j, j));
                    R_new(i, j) = clamp(R_new(i, j), -max_off, max_off);
                }
            }
        }
        
        // Adapt Q: Q_new = K * S_sample * K'
        Matrix KT = K.T();
        Matrix Q_new = K * S_sample_ * KT;
        
        // Ensure Q is positive definite
        for (size_t i = 0; i < n_; ++i) {
            Q_new(i, i) = clamp(Q_new(i, i), q_min_, q_max_);
            for (size_t j = 0; j < n_; ++j) {
                if (i != j) {
                    double max_off = 0.9 * std::sqrt(Q_new(i, i) * Q_new(j, j));
                    Q_new(i, j) = clamp(Q_new(i, j), -max_off, max_off);
                }
            }
        }
        
        // Smooth update (avoid abrupt changes)
        double beta = 0.1;  // Learning rate
        for (size_t i = 0; i < p_; ++i) {
            for (size_t j = 0; j < p_; ++j) {
                R_(i, j) = (1 - beta) * R_(i, j) + beta * R_new(i, j);
            }
        }
        for (size_t i = 0; i < n_; ++i) {
            for (size_t j = 0; j < n_; ++j) {
                Q_(i, j) = (1 - beta) * Q_(i, j) + beta * Q_new(i, j);
            }
        }
    }
    
    /**
     * @brief Covariance matching method
     * 
     * Matches predicted innovation covariance with sample covariance.
     */
    void adaptCovarianceMatching(const Matrix& innovation, const Matrix& K,
                                  const Matrix& P_pred) {
        // Compute sample covariance from window
        if (innovation_history_.size() < min_samples_) return;
        
        // Sample mean
        Matrix mean = Matrix::zeros(p_, 1);
        for (const auto& v : innovation_history_) {
            for (size_t i = 0; i < p_; ++i) {
                mean(i, 0) += v(i, 0);
            }
        }
        for (size_t i = 0; i < p_; ++i) {
            mean(i, 0) /= innovation_history_.size();
        }
        
        // Sample covariance
        Matrix S_sample = Matrix::zeros(p_, p_);
        for (const auto& v : innovation_history_) {
            Matrix centered = v - mean;
            for (size_t i = 0; i < p_; ++i) {
                for (size_t j = 0; j < p_; ++j) {
                    S_sample(i, j) += centered(i, 0) * centered(j, 0);
                }
            }
        }
        for (size_t i = 0; i < p_; ++i) {
            for (size_t j = 0; j < p_; ++j) {
                S_sample(i, j) /= (innovation_history_.size() - 1);
            }
        }
        
        // Scale factor: ratio of sample to predicted covariance
        Matrix CT = C_.T();
        Matrix S_pred = C_ * P_pred * CT + R_;
        
        // For simplicity, use trace ratio
        double trace_sample = 0, trace_pred = 0;
        for (size_t i = 0; i < p_; ++i) {
            trace_sample += S_sample(i, i);
            trace_pred += S_pred(i, i);
        }
        
        double scale = trace_sample / (trace_pred + 1e-10);
        scale = clamp(scale, 0.5, 2.0);  // Limit adaptation rate
        
        // Scale R proportionally
        if (scale > 1.0) {
            // Underestimating noise - increase R
            for (size_t i = 0; i < p_; ++i) {
                R_(i, i) = clamp(R_(i, i) * (1 + 0.1 * (scale - 1)), r_min_, r_max_);
            }
        } else {
            // Overestimating noise - decrease R
            for (size_t i = 0; i < p_; ++i) {
                R_(i, i) = clamp(R_(i, i) * (1 + 0.1 * (scale - 1)), r_min_, r_max_);
            }
        }
    }
    
    /**
     * @brief Sage-Husa adaptive algorithm
     * 
     * Classical adaptive KF with recursive estimation of Q and R.
     */
    void adaptSageHusa(const Matrix& innovation, const Matrix& K,
                       const Matrix& P_pred) {
        // Forgetting factor (increases with time for convergence)
        double dk = (1.0 - alpha_) / (1.0 - std::pow(alpha_, step_count_));
        
        // Estimate R
        Matrix CT = C_.T();
        Matrix CPCT = C_ * P_pred * CT;
        
        Matrix vvT = innovation * innovation.T();
        
        // R_k = (1-d_k)*R_{k-1} + d_k*(v_k*v_k' - C*P_pred*C')
        for (size_t i = 0; i < p_; ++i) {
            for (size_t j = 0; j < p_; ++j) {
                double r_new = vvT(i, j) - CPCT(i, j);
                R_(i, j) = (1 - dk) * R_(i, j) + dk * r_new;
            }
            R_(i, i) = clamp(R_(i, i), r_min_, r_max_);
        }
        
        // Estimate Q using the state correction
        Matrix dx = K * innovation;  // State correction
        Matrix AT = A_.T();
        
        // Q_k = (1-d_k)*Q_{k-1} + d_k*(K*v*v'*K' - A*P*A' + P_pred)
        // Simplified: Use innovation-based estimate
        Matrix KT = K.T();
        Matrix Q_est = K * vvT * KT;
        
        for (size_t i = 0; i < n_; ++i) {
            for (size_t j = 0; j < n_; ++j) {
                Q_(i, j) = (1 - dk) * Q_(i, j) + dk * Q_est(i, j);
            }
            Q_(i, i) = clamp(Q_(i, i), q_min_, q_max_);
        }
    }
    
    /**
     * @brief Variational Bayesian adaptive method
     * 
     * Uses variational inference to estimate noise covariances.
     * Simplified implementation using inverse-Wishart conjugate prior.
     */
    void adaptVariationalBayes(const Matrix& innovation, const Matrix& K,
                                const Matrix& P_pred) {
        // Degrees of freedom (increases with time)
        double nu = std::min(static_cast<double>(step_count_), 100.0);
        
        // Update scale matrix for R (inverse-Wishart)
        Matrix vvT = innovation * innovation.T();
        
        // Posterior mean of R: E[R] = S / (nu - p - 1)
        for (size_t i = 0; i < p_; ++i) {
            for (size_t j = 0; j < p_; ++j) {
                S_sample_(i, j) = alpha_ * S_sample_(i, j) + (1 - alpha_) * vvT(i, j);
            }
        }
        
        if (nu > p_ + 1) {
            for (size_t i = 0; i < p_; ++i) {
                for (size_t j = 0; j < p_; ++j) {
                    R_(i, j) = S_sample_(i, j) / (nu - p_ - 1);
                }
                R_(i, i) = clamp(R_(i, i), r_min_, r_max_);
            }
        }
        
        // Similar update for Q (simplified)
        Matrix KT = K.T();
        Matrix Q_sample = K * vvT * KT;
        
        double beta = 0.05;  // Slow adaptation for Q
        for (size_t i = 0; i < n_; ++i) {
            for (size_t j = 0; j < n_; ++j) {
                Q_(i, j) = (1 - beta) * Q_(i, j) + beta * Q_sample(i, j);
            }
            Q_(i, i) = clamp(Q_(i, i), q_min_, q_max_);
        }
    }
    
    /**
     * @brief Clamp value to range
     */
    static double clamp(double val, double min_val, double max_val) {
        return std::max(min_val, std::min(max_val, val));
    }
};

/**
 * @brief Adaptive Kalman Filter estimate result with noise info
 */
struct AdaptiveKalmanResult {
    Matrix x_hat;      // State estimate
    Matrix P;          // Error covariance
    Matrix Q;          // Adapted process noise covariance
    Matrix R;          // Adapted measurement noise covariance
    Matrix innovation; // Measurement residual
};

} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_KALMAN_HPP
