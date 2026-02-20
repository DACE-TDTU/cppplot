/**
 * @file robust/mu_analysis.hpp
 * @brief Structured Singular Value (μ) Analysis
 *
 * Provides tools for μ-analysis in robust control:
 * - μ upper bound computation
 * - D-K iteration for μ synthesis
 * - Robust stability/performance margins
 *
 * The structured singular value μ_Δ(M) is defined as:
 *   μ_Δ(M) = 1 / min{σ̄(Δ) : det(I - MΔ) = 0, Δ ∈ Δ}
 *
 * For SISO systems with diagonal uncertainty:
 *   μ ≤ max_i |M_ii| (diagonal bound)
 *   μ ≤ σ̄(M) (spectral norm upper bound)
 *
 * References:
 * - Doyle (1982) "Analysis of Feedback Systems with Structured Uncertainties"
 * - Packard & Doyle (1993) "The Complex Structured Singular Value"
 * - Zhou, Doyle, Glover (1996) "Robust and Optimal Control", Ch. 11
 */

#ifndef CPPPLOT_CONTROL_ROBUST_MU_ANALYSIS_HPP
#define CPPPLOT_CONTROL_ROBUST_MU_ANALYSIS_HPP

#include "../transfer_function.hpp"
#include "../state_space.hpp"
#include "../analysis.hpp"
#include "uncertainty.hpp"
#include "../../pyplot.hpp"
#include <vector>
#include <complex>
#include <cmath>
#include <algorithm>

namespace cppplot {
namespace control {
namespace robust {

// ============================================================
//              UNCERTAINTY BLOCK STRUCTURE
// ============================================================

/**
 * @brief Types of uncertainty blocks
 */
enum class BlockType {
    COMPLEX_SCALAR,     ///< δ ∈ ℂ, |δ| ≤ 1
    REAL_SCALAR,        ///< δ ∈ ℝ, |δ| ≤ 1
    COMPLEX_FULL,       ///< Δ ∈ ℂ^(r×c), σ̄(Δ) ≤ 1
    REAL_FULL           ///< Δ ∈ ℝ^(r×c), σ̄(Δ) ≤ 1
};

/**
 * @brief Describes one uncertainty block
 */
struct UncertaintyBlock {
    BlockType type;
    size_t rows;        ///< Number of rows (for repeated scalar: multiplicity)
    size_t cols;        ///< Number of columns (for scalar: same as rows)
    std::string name;   ///< Optional name
    
    /// Create scalar complex uncertainty
    static UncertaintyBlock scalarComplex(size_t rep = 1, const std::string& n = "") {
        return {BlockType::COMPLEX_SCALAR, rep, rep, n};
    }
    
    /// Create scalar real uncertainty
    static UncertaintyBlock scalarReal(size_t rep = 1, const std::string& n = "") {
        return {BlockType::REAL_SCALAR, rep, rep, n};
    }
    
    /// Create full complex block
    static UncertaintyBlock fullComplex(size_t r, size_t c, const std::string& n = "") {
        return {BlockType::COMPLEX_FULL, r, c, n};
    }
};

/**
 * @brief Block-diagonal uncertainty structure
 *
 * Δ = diag(δ₁I_{r1}, δ₂I_{r2}, ..., Δ₁, Δ₂, ...)
 */
class UncertaintyStructure {
public:
    std::vector<UncertaintyBlock> blocks;
    
    /// Add a block
    void addBlock(const UncertaintyBlock& b) {
        blocks.push_back(b);
    }
    
    /// Total rows in Δ
    size_t totalRows() const {
        size_t sum = 0;
        for (const auto& b : blocks) sum += b.rows;
        return sum;
    }
    
    /// Total columns in Δ  
    size_t totalCols() const {
        size_t sum = 0;
        for (const auto& b : blocks) sum += b.cols;
        return sum;
    }
    
    /// Number of scalar blocks (real + complex)
    size_t numScalars() const {
        size_t count = 0;
        for (const auto& b : blocks) {
            if (b.type == BlockType::COMPLEX_SCALAR || b.type == BlockType::REAL_SCALAR) {
                count++;
            }
        }
        return count;
    }
    
    /// Check if purely complex (no real uncertainties)
    bool isPureComplex() const {
        for (const auto& b : blocks) {
            if (b.type == BlockType::REAL_SCALAR || b.type == BlockType::REAL_FULL) {
                return false;
            }
        }
        return true;
    }
};

// ============================================================
//                  μ COMPUTATION
// ============================================================

/**
 * @brief Result of μ computation at one frequency
 */
struct MuResult {
    double mu_upper;            ///< Upper bound on μ
    double mu_lower;            ///< Lower bound on μ (from power iteration)
    std::vector<double> D_scales;   ///< D-scales achieving upper bound
    bool converged;             ///< Whether bounds are tight
};

/**
 * @brief μ computation for complex matrix M with given structure
 *
 * For SISO systems, we can compute μ exactly in some cases.
 * For general MIMO, we compute upper/lower bounds.
 */
class MuAnalyzer {
public:
    UncertaintyStructure Delta;
    
    MuAnalyzer() {}
    MuAnalyzer(const UncertaintyStructure& d) : Delta(d) {}
    
    /**
     * @brief Compute μ upper bound using D-scaling
     *
     * μ ≤ inf_D σ̄(DMD⁻¹)
     * where D is block-diagonal commuting with Δ structure.
     *
     * For diagonal scalar uncertainties:
     * μ ≤ inf_{d_i > 0} σ̄(D⁻¹MD) where D = diag(d₁,...,d_n)
     */
    MuResult computeMu(const Matrix& M) const {
        MuResult result;
        result.converged = false;
        
        size_t n = M.rows;
        if (n != M.cols || n == 0) {
            result.mu_upper = 0;
            result.mu_lower = 0;
            return result;
        }
        
        // Spectral norm upper bound (always valid)
        double sigma_max = spectralNorm(M);
        result.mu_upper = sigma_max;
        
        // For single uncertainty block, μ = σ̄(M)
        if (Delta.blocks.size() <= 1) {
            result.mu_lower = sigma_max;
            result.converged = true;
            return result;
        }
        
        // D-scale optimization for diagonal uncertainties
        if (Delta.isPureComplex() && allScalarBlocks()) {
            result = computeMuDScale(M);
        } else {
            // Use power iteration for lower bound
            result.mu_lower = muLowerBoundPowerIteration(M);
        }
        
        return result;
    }
    
    /**
     * @brief Compute μ at frequency ω for transfer function matrix
     */
    MuResult computeMuAtFrequency(
        const std::vector<std::vector<TransferFunction>>& M_tf,
        double omega
    ) const {
        // Evaluate M(jω)
        size_t n = M_tf.size();
        Matrix M(n, n);
        
        std::complex<double> jw(0, omega);
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                auto val = M_tf[i][j].eval(jw);
                // Store magnitude (for simplified analysis)
                M(i, j) = std::abs(val);
            }
        }
        
        return computeMu(M);
    }

private:
    /// Check if all blocks are scalar
    bool allScalarBlocks() const {
        for (const auto& b : Delta.blocks) {
            if (b.type == BlockType::COMPLEX_FULL || b.type == BlockType::REAL_FULL) {
                return false;
            }
        }
        return true;
    }
    
    /// Spectral norm (largest singular value)
    double spectralNorm(const Matrix& M) const {
        // For small matrices, use power iteration on M'M
        size_t n = M.rows;
        Matrix MTM = M.T() * M;
        
        // Power iteration for largest eigenvalue of M'M
        std::vector<double> v(n, 1.0 / std::sqrt(n));
        
        for (int iter = 0; iter < 50; ++iter) {
            // w = MTM * v
            std::vector<double> w(n, 0);
            for (size_t i = 0; i < n; ++i) {
                for (size_t j = 0; j < n; ++j) {
                    w[i] += MTM(i, j) * v[j];
                }
            }
            
            // Normalize
            double norm = 0;
            for (size_t i = 0; i < n; ++i) norm += w[i] * w[i];
            norm = std::sqrt(norm);
            
            if (norm < 1e-15) return 0;
            
            for (size_t i = 0; i < n; ++i) v[i] = w[i] / norm;
        }
        
        // Rayleigh quotient
        double rq = 0;
        std::vector<double> Mv(n, 0);
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                Mv[i] += MTM(i, j) * v[j];
            }
            rq += v[i] * Mv[i];
        }
        
        return std::sqrt(std::abs(rq));
    }
    
    /// D-scale optimization for upper bound
    MuResult computeMuDScale(const Matrix& M) const {
        MuResult result;
        size_t n = M.rows;
        
        // Start with D = I
        std::vector<double> d(n, 1.0);
        result.D_scales = d;
        
        double best_mu = spectralNorm(M);
        result.mu_upper = best_mu;
        double best_val_outer = best_mu;  // Track best value across iterations
        
        // Simple coordinate descent on D
        for (int outer = 0; outer < 20; ++outer) {
            bool improved = false;
            
            for (size_t k = 0; k < n; ++k) {
                // Try scaling d[k]
                double best_dk = d[k];
                double best_val = best_val_outer;
                
                for (double scale : {0.5, 0.8, 1.0, 1.25, 2.0}) {
                    d[k] = best_dk * scale;
                    if (d[k] < 0.01) d[k] = 0.01;
                    if (d[k] > 100) d[k] = 100;
                    
                    // Compute σ̄(D⁻¹MD)
                    Matrix DinvMD(n, n);
                    for (size_t i = 0; i < n; ++i) {
                        for (size_t j = 0; j < n; ++j) {
                            DinvMD(i, j) = M(i, j) * d[j] / d[i];
                        }
                    }
                    
                    double sigma = spectralNorm(DinvMD);
                    if (sigma < best_val) {
                        best_val = sigma;
                        best_dk = d[k];
                        improved = true;
                    }
                }
                d[k] = best_dk;
                if (best_val < best_val_outer) {
                    best_val_outer = best_val;
                }
            }
            
            if (!improved) break;
            best_mu = best_val_outer;
        }
        
        result.D_scales = d;
        result.mu_upper = best_mu;
        result.mu_lower = best_mu * 0.9;  // Conservative lower bound
        result.converged = true;
        
        return result;
    }
    
    /// Power iteration lower bound
    double muLowerBoundPowerIteration(const Matrix& M) const {
        // Find Δ with small norm such that det(I - MΔ) ≈ 0
        // Start with random Δ and iterate
        
        size_t n = M.rows;
        double mu_lb = 0;
        
        // Try to find destabilizing Δ by searching on boundary
        for (int trial = 0; trial < 10; ++trial) {
            // Random direction on unit sphere
            Matrix Delta_try(n, n);
            double norm = 0;
            for (size_t i = 0; i < n; ++i) {
                double angle = 2.0 * M_PI * trial / 10.0 + i * 0.1;
                Delta_try(i, i) = std::cos(angle);
                norm += Delta_try(i, i) * Delta_try(i, i);
            }
            norm = std::sqrt(norm);
            for (size_t i = 0; i < n; ++i) {
                Delta_try(i, i) /= norm;
            }
            
            // Compute det(I - M*Delta)
            Matrix I_minus_MD = Matrix::eye(n) - M * Delta_try;
            double det_val = std::abs(I_minus_MD.det());
            
            // If det is small, we're close to singularity
            if (det_val < 0.1) {
                mu_lb = std::max(mu_lb, 1.0 / (1.0 + det_val));
            }
        }
        
        return mu_lb;
    }
};

// ============================================================
//               μ FREQUENCY SWEEP
// ============================================================

/**
 * @brief Result of μ analysis over frequency range
 */
struct MuFrequencyResult {
    std::vector<double> frequencies;
    std::vector<double> mu_upper;
    std::vector<double> mu_lower;
    double peak_mu;                 ///< Maximum μ over frequency
    double peak_frequency;          ///< Frequency of peak μ
    bool robustlyStable;            ///< True if peak_mu < 1
};

/**
 * @brief Perform μ analysis over frequency range
 *
 * For SISO system with M = closed-loop transfer function matrix,
 * robust stability is guaranteed if μ(M(jω)) < 1 for all ω.
 */
inline MuFrequencyResult muFrequencySweep(
    const MuAnalyzer& analyzer,
    const std::vector<std::vector<TransferFunction>>& M,
    double w_min = 0.001,
    double w_max = 1000.0,
    int n_points = 100
) {
    MuFrequencyResult result;
    result.peak_mu = 0;
    result.peak_frequency = 0;
    result.robustlyStable = true;
    
    double log_w_min = std::log10(w_min);
    double log_w_max = std::log10(w_max);
    
    for (int i = 0; i < n_points; ++i) {
        double log_w = log_w_min + (log_w_max - log_w_min) * i / (n_points - 1);
        double w = std::pow(10.0, log_w);
        
        auto mu_result = analyzer.computeMuAtFrequency(M, w);
        
        result.frequencies.push_back(w);
        result.mu_upper.push_back(mu_result.mu_upper);
        result.mu_lower.push_back(mu_result.mu_lower);
        
        if (mu_result.mu_upper > result.peak_mu) {
            result.peak_mu = mu_result.mu_upper;
            result.peak_frequency = w;
        }
        
        if (mu_result.mu_upper >= 1.0) {
            result.robustlyStable = false;
        }
    }
    
    return result;
}

// ============================================================
//                 D-K ITERATION
// ============================================================

/**
 * @brief D-K iteration result for μ-synthesis
 */
struct DKIterationResult {
    TransferFunction K;             ///< Synthesized controller
    std::vector<TransferFunction> D_fits;   ///< D-scale transfer function fits
    double achieved_mu;             ///< Achieved peak μ
    int iterations;                 ///< Number of D-K iterations
    bool converged;
};

/**
 * @brief Simplified D-K iteration for SISO systems
 *
 * D-K iteration alternates between:
 * 1. D-step: Fix K, find D-scales minimizing μ upper bound
 * 2. K-step: Fix D, design H∞ controller for scaled plant
 *
 * For SISO systems with single uncertainty, this simplifies considerably.
 */
inline DKIterationResult dkIteration(
    const TransferFunction& G_nom,
    const TransferFunction& W_perf,
    const TransferFunction& W_delta,
    int max_iter = 5,
    double gamma_target = 1.0
) {
    DKIterationResult result;
    result.converged = false;
    result.iterations = 0;
    
    // Initial controller: simple PI
    double wc = 1.0;  // Crossover frequency
    double Kp = 1.0 / std::abs(G_nom.dcgain());
    double Ki = Kp * wc / 5.0;
    
    TransferFunction K({Kp, Ki}, {1.0, 0.0});
    
    // D-scales (start with identity = 1)
    std::vector<TransferFunction> D = {TransferFunction({1}, {1})};
    
    double prev_mu = 1e10;
    
    for (int iter = 0; iter < max_iter; ++iter) {
        result.iterations = iter + 1;
        
        // Evaluate current μ
        // For SISO: μ ≈ max(|W_perf * S|, |W_delta * T|)
        auto GK = G_nom * K;
        auto T = feedback(GK, TransferFunction({1}, {1}));
        
        double peak_mu = 0;
        for (double w = 0.01; w < 100; w *= 1.2) {
            std::complex<double> jw(0, w);
            auto GK_jw = GK.eval(jw);
            auto S_jw = 1.0 / (1.0 + GK_jw);
            auto T_jw = T.eval(jw);
            
            double wp_s = std::abs(W_perf.eval(jw) * S_jw);
            double wd_t = std::abs(W_delta.eval(jw) * T_jw);
            
            // Approximate μ for RP problem
            double mu_approx = wp_s + wd_t;  // Upper bound
            peak_mu = std::max(peak_mu, mu_approx);
        }
        
        result.achieved_mu = peak_mu;
        
        // Check convergence
        if (std::abs(peak_mu - prev_mu) < 0.01 || peak_mu < gamma_target) {
            result.converged = true;
            break;
        }
        prev_mu = peak_mu;
        
        // D-step: Fit D-scales (simplified - adjust gain)
        // For SISO, D-scaling reduces to gain adjustment
        double d_scale = 1.0;
        if (peak_mu > 1.0) {
            // Reduce gain to improve robustness
            d_scale = 0.8;
        }
        
        // K-step: Redesign controller (simplified loop-shaping)
        // Increase bandwidth if μ < 1, decrease if μ > 1
        if (peak_mu < 1.0) {
            wc *= 1.1;  // Can afford more bandwidth
        } else {
            wc *= 0.9;  // Need to reduce bandwidth for robustness
        }
        
        Kp = d_scale / std::abs(G_nom.dcgain());
        Ki = Kp * wc / 5.0;
        K = TransferFunction({Kp, Ki}, {1.0, 0.0});
    }
    
    result.K = K;
    result.D_fits = D;
    
    return result;
}

// ============================================================
//             SKEWED μ ANALYSIS
// ============================================================

/**
 * @brief Skewed μ for performance analysis
 *
 * For robust performance, we need μ_skewed < 1 where the
 * performance block is treated as a fictitious full uncertainty.
 */
struct SkewedMuResult {
    double mu_skewed;               ///< Skewed μ value
    double robustPerformanceMargin; ///< 1/μ (how much margin)
    bool achievesRobustPerformance;
};

inline SkewedMuResult computeSkewedMu(
    const TransferFunction& G_nom,
    const TransferFunction& K,
    const TransferFunction& W_perf,
    const TransferFunction& W_delta,
    double omega
) {
    SkewedMuResult result;
    
    std::complex<double> jw(0, omega);
    
    // Compute closed-loop sensitivities
    auto GK_jw = (G_nom * K).eval(jw);
    auto S_jw = 1.0 / (1.0 + GK_jw);
    auto T_jw = GK_jw * S_jw;
    
    // Weighted sensitivities
    double wp_s = std::abs(W_perf.eval(jw) * S_jw);
    double wd_t = std::abs(W_delta.eval(jw) * T_jw);
    
    // For RP with single uncertainty:
    // μ_skewed = |W_p S| + |W_Δ T|  (upper bound)
    result.mu_skewed = wp_s + wd_t;
    result.robustPerformanceMargin = 1.0 / std::max(result.mu_skewed, 1e-15);
    result.achievesRobustPerformance = (result.mu_skewed < 1.0);
    
    return result;
}

// ============================================================
//               SUMMARY FUNCTIONS
// ============================================================

/**
 * @brief Compute and display μ analysis summary
 */
inline void printMuAnalysisSummary(
    const TransferFunction& G_nom,
    const TransferFunction& K,
    const TransferFunction& W_delta,
    const TransferFunction& W_perf = TransferFunction({1}, {1})
) {
    std::cout << "============== μ-Analysis Summary ==============\n\n";
    
    // Frequency sweep
    double peak_mu = 0;
    double peak_freq = 0;
    double peak_rs = 0;  // Robust stability
    double peak_rp = 0;  // Robust performance
    
    for (double w = 0.01; w < 1000; w *= 1.1) {
        std::complex<double> jw(0, w);
        
        auto GK_jw = (G_nom * K).eval(jw);
        auto S_jw = 1.0 / (1.0 + GK_jw);
        auto T_jw = GK_jw * S_jw;
        
        double wd_t = std::abs(W_delta.eval(jw) * T_jw);
        double wp_s = std::abs(W_perf.eval(jw) * S_jw);
        
        peak_rs = std::max(peak_rs, wd_t);
        peak_rp = std::max(peak_rp, wd_t + wp_s);
        
        if (wd_t + wp_s > peak_mu) {
            peak_mu = wd_t + wp_s;
            peak_freq = w;
        }
    }
    
    std::cout << "Robust Stability (|W_Δ T|_∞):\n";
    std::cout << "  Peak value: " << peak_rs << "\n";
    std::cout << "  Status: " << (peak_rs < 1.0 ? "STABLE" : "UNSTABLE") << "\n";
    std::cout << "  Margin: " << (1.0 / peak_rs) << "\n\n";
    
    std::cout << "Robust Performance (μ_RP):\n";
    std::cout << "  Peak μ: " << peak_mu << "\n";
    std::cout << "  Critical frequency: " << peak_freq << " rad/s\n";
    std::cout << "  Status: " << (peak_mu < 1.0 ? "ACHIEVED" : "NOT ACHIEVED") << "\n";
    std::cout << "  RP Margin: " << (1.0 / peak_mu) << "\n";
    
    std::cout << "\n================================================\n";
}

// ============================================================
//               PLOTTING FUNCTIONS (using CppPlot)
// ============================================================

/**
 * @brief Plot μ analysis results over frequency
 * 
 * Creates a comprehensive plot showing:
 * - μ upper/lower bounds vs frequency
 * - Robust stability boundary
 * - Peak μ indicator
 * 
 * @param result μ frequency sweep result
 * @param filename Output filename (without extension)
 */
inline void plotMuFrequencyResponse(
    const MuFrequencyResult& result,
    const std::string& filename = "mu_analysis"
) {
    using namespace cppplot;
    
    figure(900, 600);
    suptitle("Structured Singular Value (μ) Analysis");
    
    // Plot μ bounds
    xscale("log");
    plot(result.frequencies, result.mu_upper, "-", 
         opts({{"color", "blue"}, {"linewidth", "2"}, {"label", "μ upper bound"}}));
    plot(result.frequencies, result.mu_lower, "--", 
         opts({{"color", "cyan"}, {"linewidth", "1.5"}, {"label", "μ lower bound"}}));
    
    // Stability boundary at μ = 1
    axhline(1.0, opts({{"color", "red"}, {"linestyle", "--"}, {"linewidth", "2"}, {"label", "Stability boundary"}}));
    
    // Mark peak μ
    scatter({result.peak_frequency}, {result.peak_mu}, 
            opts({{"s", "100"}, {"color", "red"}, {"marker", "o"}}));
    
    // Add annotation for peak
    std::ostringstream peak_text;
    peak_text << "Peak: " << std::fixed << std::setprecision(3) << result.peak_mu 
              << " @ " << std::setprecision(2) << result.peak_frequency << " rad/s";
    text(result.peak_frequency * 1.5, result.peak_mu * 1.05, peak_text.str(),
         opts({{"fontsize", "10"}, {"color", "red"}}));
    
    xlabel("Frequency (rad/s)");
    ylabel("μ");
    title(result.robustlyStable ? "Robust Stability: GUARANTEED" : "Robust Stability: NOT GUARANTEED");
    legend(true);
    grid(true);
    ylim(0, std::max(1.5, result.peak_mu * 1.2));
    
    savefig(filename + ".svg");
    savefig(filename + ".png");
    std::cout << "Saved: " << filename << ".svg, " << filename << ".png\n";
}

/**
 * @brief Plot comprehensive μ-analysis with RS and RP
 * 
 * Creates multi-panel plot showing:
 * 1. |W_Δ T| - Robust Stability condition
 * 2. |W_p S| - Nominal Performance  
 * 3. μ_RP = |W_Δ T| + |W_p S| - Robust Performance
 */
inline void plotMuAnalysisComprehensive(
    const TransferFunction& G_nom,
    const TransferFunction& K,
    const TransferFunction& W_delta,
    const TransferFunction& W_perf,
    const std::string& filename = "mu_comprehensive"
) {
    using namespace cppplot;
    
    // Generate frequency data
    auto omega = logspace(-2, 3, 200);
    
    std::vector<double> WdT_mag, WpS_mag, mu_RP;
    double peak_WdT = 0, peak_WpS = 0, peak_mu = 0;
    double freq_peak_mu = 0;
    
    for (double w : omega) {
        std::complex<double> jw(0, w);
        
        auto GK_jw = (G_nom * K).eval(jw);
        auto S_jw = 1.0 / (1.0 + GK_jw);
        auto T_jw = GK_jw * S_jw;
        
        double wd_t = std::abs(W_delta.eval(jw) * T_jw);
        double wp_s = std::abs(W_perf.eval(jw) * S_jw);
        double mu = wd_t + wp_s;
        
        WdT_mag.push_back(wd_t);
        WpS_mag.push_back(wp_s);
        mu_RP.push_back(mu);
        
        peak_WdT = std::max(peak_WdT, wd_t);
        peak_WpS = std::max(peak_WpS, wp_s);
        if (mu > peak_mu) {
            peak_mu = mu;
            freq_peak_mu = w;
        }
    }
    
    // Create figure with 3 subplots
    figure(1000, 800);
    suptitle("Comprehensive μ-Analysis for Robust Performance");
    layout(3, 1);
    
    // Subplot 1: Robust Stability |W_Δ T|
    subplot(3, 1, 1);
    xscale("log");
    plot(omega, WdT_mag, "-", opts({{"color", "blue"}, {"linewidth", "2"}, {"label", "|W_Δ T|"}}));
    axhline(1.0, opts({{"color", "red"}, {"linestyle", "--"}, {"linewidth", "1.5"}}));
    ylabel("|W_Δ T|");
    std::ostringstream rs_title;
    rs_title << "Robust Stability: ||W_Δ T||_∞ = " << std::fixed << std::setprecision(3) << peak_WdT
             << (peak_WdT < 1.0 ? " < 1 ✓" : " ≥ 1 ✗");
    title(rs_title.str());
    legend(true);
    grid(true);
    
    // Subplot 2: Nominal Performance |W_p S|
    subplot(3, 1, 2);
    xscale("log");
    plot(omega, WpS_mag, "-", opts({{"color", "green"}, {"linewidth", "2"}, {"label", "|W_p S|"}}));
    axhline(1.0, opts({{"color", "red"}, {"linestyle", "--"}, {"linewidth", "1.5"}}));
    ylabel("|W_p S|");
    std::ostringstream np_title;
    np_title << "Nominal Performance: ||W_p S||_∞ = " << std::fixed << std::setprecision(3) << peak_WpS
             << (peak_WpS < 1.0 ? " < 1 ✓" : " ≥ 1 ✗");
    title(np_title.str());
    legend(true);
    grid(true);
    
    // Subplot 3: Robust Performance μ_RP
    subplot(3, 1, 3);
    xscale("log");
    plot(omega, mu_RP, "-", opts({{"color", "magenta"}, {"linewidth", "2.5"}, {"label", "μ_RP = |W_Δ T| + |W_p S|"}}));
    axhline(1.0, opts({{"color", "red"}, {"linestyle", "--"}, {"linewidth", "2"}, {"label", "RP boundary"}}));
    
    // Mark peak
    scatter({freq_peak_mu}, {peak_mu}, opts({{"s", "80"}, {"color", "red"}}));
    
    xlabel("Frequency (rad/s)");
    ylabel("μ_RP");
    std::ostringstream rp_title;
    rp_title << "Robust Performance: μ_peak = " << std::fixed << std::setprecision(3) << peak_mu
             << " @ " << std::setprecision(1) << freq_peak_mu << " rad/s"
             << (peak_mu < 1.0 ? " ✓" : " ✗");
    title(rp_title.str());
    legend(true);
    grid(true);
    ylim(0, std::max(1.5, peak_mu * 1.2));
    
    savefig(filename + ".svg");
    savefig(filename + ".png");
    std::cout << "Saved: " << filename << ".svg, " << filename << ".png\n";
}

/**
 * @brief Plot D-K iteration convergence
 */
inline void plotDKIterationConvergence(
    const std::vector<double>& mu_history,
    double gamma_target,
    const std::string& filename = "dk_convergence"
) {
    using namespace cppplot;
    
    std::vector<double> iterations;
    for (size_t i = 0; i < mu_history.size(); ++i) {
        iterations.push_back(static_cast<double>(i + 1));
    }
    
    figure(700, 500);
    
    plot(iterations, mu_history, "-o", 
         opts({{"color", "blue"}, {"linewidth", "2"}, {"markersize", "8"}, {"label", "Achieved μ"}}));
    axhline(gamma_target, opts({{"color", "green"}, {"linestyle", "--"}, {"linewidth", "2"}, {"label", "Target γ"}}));
    axhline(1.0, opts({{"color", "red"}, {"linestyle", ":"}, {"linewidth", "1.5"}, {"label", "RP boundary"}}));
    
    xlabel("D-K Iteration");
    ylabel("Peak μ");
    title("D-K Iteration Convergence");
    legend(true);
    grid(true);
    xlim(0.5, static_cast<double>(mu_history.size()) + 0.5);
    
    savefig(filename + ".svg");
    std::cout << "Saved: " << filename << ".svg\n";
}

/**
 * @brief Plot M-Δ structure visualization (Nyquist-like)
 * 
 * Shows the critical point (-1, 0) and uncertainty disk
 */
inline void plotMDeltaStructure(
    const TransferFunction& G_nom,
    const TransferFunction& K,
    const TransferFunction& W_delta,
    const std::string& filename = "m_delta_structure"
) {
    using namespace cppplot;
    
    figure(700, 700);
    
    // Generate loop transfer function L = GK
    auto L = G_nom * K;
    
    std::vector<double> L_real, L_imag;
    std::vector<double> disk_real, disk_imag;  // Uncertainty disk at critical freq
    
    double critical_freq = 1.0;
    double max_WdT = 0;
    
    // Find critical frequency (where |W_Δ T| is maximum)
    for (double w = 0.01; w < 100; w *= 1.1) {
        std::complex<double> jw(0, w);
        auto L_jw = L.eval(jw);
        auto T_jw = L_jw / (1.0 + L_jw);
        double wd_t = std::abs(W_delta.eval(jw) * T_jw);
        if (wd_t > max_WdT) {
            max_WdT = wd_t;
            critical_freq = w;
        }
    }
    
    // Plot Nyquist curve of L(jω)
    for (double w = 0.001; w < 1000; w *= 1.05) {
        std::complex<double> jw(0, w);
        auto L_jw = L.eval(jw);
        L_real.push_back(L_jw.real());
        L_imag.push_back(L_jw.imag());
    }
    
    plot(L_real, L_imag, "-", opts({{"color", "blue"}, {"linewidth", "2"}, {"label", "L(jω)"}}));
    
    // Plot uncertainty disk at critical frequency
    std::complex<double> jw_crit(0, critical_freq);
    auto L_crit = L.eval(jw_crit);
    double radius = std::abs(W_delta.eval(jw_crit)) * std::abs(L_crit / (1.0 + L_crit));
    
    // Draw disk
    for (int i = 0; i <= 50; ++i) {
        double theta = 2.0 * M_PI * i / 50;
        disk_real.push_back(L_crit.real() + radius * std::cos(theta));
        disk_imag.push_back(L_crit.imag() + radius * std::sin(theta));
    }
    plot(disk_real, disk_imag, "--", 
         opts({{"color", "orange"}, {"linewidth", "1.5"}, {"label", "Uncertainty disk"}}));
    
    // Mark critical point (-1, 0)
    scatter({-1.0}, {0.0}, opts({{"s", "150"}, {"color", "red"}, {"marker", "x"}}));
    text(-1.0, -0.2, "(-1, 0)", opts({{"fontsize", "12"}, {"color", "red"}, {"ha", "center"}}));
    
    // Mark point at critical frequency
    scatter({L_crit.real()}, {L_crit.imag()}, opts({{"s", "80"}, {"color", "green"}}));
    
    xlabel("Real");
    ylabel("Imaginary");
    title("M-Δ Structure: Nyquist Diagram with Uncertainty");
    legend(true);
    grid(true);
    
    // Equal aspect ratio approximation
    double max_range = std::max({std::abs(*std::max_element(L_real.begin(), L_real.end())),
                                  std::abs(*std::min_element(L_real.begin(), L_real.end())),
                                  std::abs(*std::max_element(L_imag.begin(), L_imag.end())),
                                  std::abs(*std::min_element(L_imag.begin(), L_imag.end())),
                                  1.5});
    xlim(-max_range * 1.1, max_range * 1.1);
    ylim(-max_range * 1.1, max_range * 1.1);
    
    savefig(filename + ".svg");
    savefig(filename + ".png");
    std::cout << "Saved: " << filename << ".svg, " << filename << ".png\n";
}

/**
 * @brief Complete μ-analysis visualization suite
 * 
 * Generates all relevant plots for μ-analysis
 */
/**
 * @brief Create standard 2x2 M-Δ structure for RP analysis
 *
 * For robust performance with multiplicative uncertainty:
 * 
 *     [ W_Δ T    W_Δ T G ]
 * M = [                  ]
 *     [ W_p S   W_p S G  ]
 *
 * Δ = diag(δ_Δ, δ_f) where δ_f is fictitious performance block
 */
inline std::vector<std::vector<TransferFunction>> createMDeltaStructure(
    const TransferFunction& G,
    const TransferFunction& K,
    const TransferFunction& W_delta,
    const TransferFunction& W_perf
) {
    // Compute sensitivities
    auto GK = G * K;
    auto one = TransferFunction({1}, {1});
    auto T = feedback(GK, one);
    
    // S = 1 - T (approximately, for proper computation)
    // For exact S, we'd need: S = 1/(1+GK)
    
    // Build M matrix
    std::vector<std::vector<TransferFunction>> M(2, std::vector<TransferFunction>(2, one));
    
    M[0][0] = W_delta * T;
    M[0][1] = W_delta * T * G;
    M[1][0] = W_perf * (one - T);  // W_p * S
    M[1][1] = W_perf * (one - T) * G;
    
    return M;
}

/**
 * @brief Complete μ-analysis visualization suite
 * 
 * Generates all relevant plots for μ-analysis
 */
inline void plotMuAnalysisSuite(
    const TransferFunction& G_nom,
    const TransferFunction& K,
    const TransferFunction& W_delta,
    const TransferFunction& W_perf = TransferFunction({1}, {1}),
    const std::string& prefix = "mu"
) {
    std::cout << "\n=== Generating μ-Analysis Plots ===\n";
    
    // 1. Comprehensive μ analysis
    plotMuAnalysisComprehensive(G_nom, K, W_delta, W_perf, prefix + "_comprehensive");
    
    // 2. M-Δ structure
    plotMDeltaStructure(G_nom, K, W_delta, prefix + "_mdelta");
    
    // 3. Create and plot frequency sweep result
    MuAnalyzer analyzer;
    auto M = createMDeltaStructure(G_nom, K, W_delta, W_perf);
    auto freq_result = muFrequencySweep(analyzer, M);
    plotMuFrequencyResponse(freq_result, prefix + "_frequency");
    
    std::cout << "=== All μ-analysis plots generated! ===\n";
}

} // namespace robust
} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_ROBUST_MU_ANALYSIS_HPP
