/**
 * @file robust/uncertainty.hpp
 * @brief Structured Uncertainty Modeling for Robust Control
 *
 * Provides tools for modeling and analyzing uncertainties in control systems:
 * - Parametric uncertainty (interval, percentage bounds)
 * - Unstructured uncertainty (multiplicative, additive, inverse multiplicative)
 * - Uncertainty weighting functions
 * - Robust stability analysis
 *
 * References:
 * - Skogestad & Postlethwaite (2005) "Multivariable Feedback Control", Ch. 7-8
 * - Zhou, Doyle, Glover (1996) "Robust and Optimal Control", Ch. 8
 * - Doyle (1982) "Analysis of Feedback Systems with Structured Uncertainties"
 */

#ifndef CPPPLOT_CONTROL_ROBUST_UNCERTAINTY_HPP
#define CPPPLOT_CONTROL_ROBUST_UNCERTAINTY_HPP

#include "../transfer_function.hpp"
#include "../state_space.hpp"
#include "../analysis.hpp"
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>

namespace cppplot {
namespace control {
namespace robust {

// ============================================================
//                 UNCERTAINTY TYPES
// ============================================================

/**
 * @brief Types of uncertainty models
 */
enum class UncertaintyType {
    PARAMETRIC,           ///< Parameter varies within bounds
    MULTIPLICATIVE,       ///< G_true = G_nom * (1 + W*Δ), |Δ| ≤ 1
    ADDITIVE,             ///< G_true = G_nom + W*Δ, |Δ| ≤ 1
    INV_MULTIPLICATIVE,   ///< G_true = G_nom / (1 + W*Δ), |Δ| ≤ 1
    FEEDBACK,             ///< G_true in feedback with Δ
    COPRIME               ///< Left/right coprime factor uncertainty
};

// ============================================================
//               PARAMETRIC UNCERTAINTY
// ============================================================

/**
 * @brief Represents a parameter with uncertainty
 */
struct UncertainParameter {
    std::string name;       ///< Parameter name
    double nominal;         ///< Nominal value
    double min_val;         ///< Minimum value
    double max_val;         ///< Maximum value
    double percentage;      ///< Percentage uncertainty (if defined by %)
    
    /// Create from absolute bounds
    UncertainParameter(const std::string& n, double nom, double min_v, double max_v)
        : name(n), nominal(nom), min_val(min_v), max_val(max_v) {
        percentage = 100.0 * std::max(std::abs(max_v - nom), std::abs(nom - min_v)) / std::abs(nom);
    }
    
    /// Create from percentage
    static UncertainParameter fromPercentage(const std::string& n, double nom, double pct) {
        double delta = std::abs(nom) * pct / 100.0;
        return UncertainParameter(n, nom, nom - delta, nom + delta);
    }
    
    /// Sample a value (0 = nominal, -1 = min, +1 = max)
    double sample(double normalized) const {
        if (normalized == 0) return nominal;
        if (normalized < 0) return nominal + normalized * (nominal - min_val);
        return nominal + normalized * (max_val - nominal);
    }
    
    /// Get the uncertainty radius
    double radius() const {
        return std::max(max_val - nominal, nominal - min_val);
    }
};

/**
 * @brief Collection of uncertain parameters
 */
class ParametricUncertaintySet {
public:
    std::vector<UncertainParameter> params;
    
    /// Add a parameter
    void add(const UncertainParameter& p) {
        params.push_back(p);
    }
    
    /// Add parameter from bounds
    void add(const std::string& name, double nom, double min_v, double max_v) {
        params.emplace_back(name, nom, min_v, max_v);
    }
    
    /// Add parameter from percentage
    void addPercent(const std::string& name, double nom, double pct) {
        params.push_back(UncertainParameter::fromPercentage(name, nom, pct));
    }
    
    /// Get number of parameters
    size_t size() const { return params.size(); }
    
    /// Get nominal values
    std::vector<double> getNominal() const {
        std::vector<double> v(params.size());
        for (size_t i = 0; i < params.size(); ++i) {
            v[i] = params[i].nominal;
        }
        return v;
    }
    
    /// Sample at a vertex (each entry is -1 or +1)
    std::vector<double> sampleVertex(const std::vector<int>& vertex) const {
        std::vector<double> v(params.size());
        for (size_t i = 0; i < params.size(); ++i) {
            v[i] = params[i].sample(vertex[i]);
        }
        return v;
    }
    
    /// Generate all 2^n vertices of the uncertainty hypercube
    std::vector<std::vector<double>> getAllVertices() const {
        size_t n = params.size();
        size_t num_vertices = 1 << n;  // 2^n
        std::vector<std::vector<double>> vertices;
        
        for (size_t v = 0; v < num_vertices; ++v) {
            std::vector<int> signs(n);
            for (size_t i = 0; i < n; ++i) {
                signs[i] = (v & (1 << i)) ? 1 : -1;
            }
            vertices.push_back(sampleVertex(signs));
        }
        return vertices;
    }
};

// ============================================================
//             UNSTRUCTURED UNCERTAINTY
// ============================================================

/**
 * @brief Unstructured (dynamic) uncertainty model
 *
 * Models frequency-dependent uncertainty using weighting functions.
 * The true plant is G_true = G_nom * (1 + W_Δ * Δ) for multiplicative,
 * where |Δ(jω)| ≤ 1 for all ω.
 */
class UnstructuredUncertainty {
public:
    UncertaintyType type;
    TransferFunction weight;        ///< Uncertainty weighting function W
    TransferFunction nominal;       ///< Nominal plant G_nom
    
    UnstructuredUncertainty() : type(UncertaintyType::MULTIPLICATIVE) {}
    
    /**
     * @brief Create multiplicative uncertainty: G = G_nom * (1 + W*Δ)
     * 
     * At frequency ω, the relative uncertainty is |W(jω)|.
     * Common choice: W(s) = (τs + r0) / ((τ/r_inf)s + 1)
     *   - r0: relative uncertainty at low frequency
     *   - r_inf: relative uncertainty at high frequency
     *   - τ: transition time constant
     */
    static UnstructuredUncertainty multiplicative(
        const TransferFunction& G_nom,
        double r0 = 0.2,      // 20% at DC
        double r_inf = 2.0,   // 200% at high freq
        double tau = 1.0      // bandwidth of transition
    ) {
        UnstructuredUncertainty u;
        u.type = UncertaintyType::MULTIPLICATIVE;
        u.nominal = G_nom;
        // W(s) = (τs + r0) / ((τ/r_inf)s + 1)
        u.weight = TransferFunction({tau, r0}, {tau / r_inf, 1.0});
        return u;
    }
    
    /**
     * @brief Create additive uncertainty: G = G_nom + W*Δ
     */
    static UnstructuredUncertainty additive(
        const TransferFunction& G_nom,
        const TransferFunction& W
    ) {
        UnstructuredUncertainty u;
        u.type = UncertaintyType::ADDITIVE;
        u.nominal = G_nom;
        u.weight = W;
        return u;
    }
    
    /**
     * @brief Create inverse multiplicative: G = G_nom / (1 + W*Δ)
     *
     * Useful when uncertainty affects plant input.
     */
    static UnstructuredUncertainty inverseMultiplicative(
        const TransferFunction& G_nom,
        const TransferFunction& W
    ) {
        UnstructuredUncertainty u;
        u.type = UncertaintyType::INV_MULTIPLICATIVE;
        u.nominal = G_nom;
        u.weight = W;
        return u;
    }
    
    /**
     * @brief Evaluate the uncertainty bound at frequency ω
     */
    double bound(double omega) const {
        auto W_jw = weight.evalS(std::complex<double>(0, omega));
        return std::abs(W_jw);
    }
    
    /**
     * @brief Get worst-case perturbed plant at frequency ω
     *
     * Returns plants at the boundary of uncertainty disk.
     */
    std::vector<TransferFunction> worstCasePlants(
        double omega, 
        int num_samples = 8
    ) const {
        std::vector<TransferFunction> plants;
        
        // Sample Δ on the unit circle
        for (int i = 0; i < num_samples; ++i) {
            double theta = 2.0 * M_PI * i / num_samples;
            std::complex<double> Delta(std::cos(theta), std::sin(theta));
            
            // This is a simplification - exact perturbation would need
            // frequency-domain manipulation
            double pert = std::real(Delta) * bound(omega);
            
            // Create perturbed plant (approximate)
            // Get coefficient vectors from Polynomial members
            std::vector<double> num = nominal.getNum().coeffs;
            std::vector<double> den = nominal.getDen().coeffs;
            
            if (type == UncertaintyType::MULTIPLICATIVE) {
                // G_pert ≈ G_nom * (1 + pert)
                for (auto& c : num) c *= (1.0 + pert);
            } else if (type == UncertaintyType::ADDITIVE) {
                // G_pert = G_nom + pert (DC approximation)
                num.back() += pert;
            }
            
            plants.push_back(TransferFunction(num, den));
        }
        
        return plants;
    }
};

// ============================================================
//            COMMON UNCERTAINTY WEIGHTS
// ============================================================

/**
 * @brief Standard uncertainty weight templates
 */
namespace weights {

/**
 * @brief First-order uncertainty weight
 * W(s) = (τs + r0) / ((τ/r_inf)s + 1)
 *
 * @param r0 Low-frequency uncertainty (e.g., 0.2 = 20%)
 * @param r_inf High-frequency uncertainty (e.g., 2.0 = 200%)
 * @param omega_b Uncertainty bandwidth (rad/s)
 */
inline TransferFunction firstOrder(double r0, double r_inf, double omega_b = 1.0) {
    double tau = 1.0 / omega_b;
    return TransferFunction({tau, r0}, {tau / r_inf, 1.0});
}

/**
 * @brief Second-order uncertainty weight for resonant systems
 * W(s) = (s² + 2ζ₁ω₁s + ω₁²) / (s² + 2ζ₂ω₂s + ω₂²) * K
 */
inline TransferFunction secondOrder(
    double wn1, double zeta1,
    double wn2, double zeta2,
    double K = 1.0
) {
    return TransferFunction(
        {K, K * 2 * zeta1 * wn1, K * wn1 * wn1},
        {1.0, 2 * zeta2 * wn2, wn2 * wn2}
    );
}

/**
 * @brief High-pass weight (uncertainty dominates at high frequency)
 * W(s) = K * s / (s + a)
 */
inline TransferFunction highPass(double K = 1.0, double a = 0.1) {
    return TransferFunction({K, 0}, {1.0, a});
}

/**
 * @brief Low-pass weight (uncertainty dominates at low frequency)
 * W(s) = K / (τs + 1)
 */
inline TransferFunction lowPass(double K = 1.0, double tau = 1.0) {
    return TransferFunction({K}, {tau, 1.0});
}

/**
 * @brief Band-pass weight
 */
inline TransferFunction bandPass(double K, double wc, double bw) {
    double w_lo = wc - bw/2;
    double w_hi = wc + bw/2;
    // Approximate band-pass
    return TransferFunction({K, 0}, {1.0, (w_lo + w_hi), w_lo * w_hi});
}

/**
 * @brief Parametric-to-unstructured weight
 *
 * For parameter p ∈ [p_nom - Δp, p_nom + Δp], if p appears in G(s)
 * as G = G0 + p*G1, then |W(s)| ≥ |Δp * G1(s)/G(s)|
 */
inline TransferFunction fromParametric(
    const TransferFunction& G_nom,
    const TransferFunction& dG_dp,  // ∂G/∂p
    double delta_p
) {
    // W ≈ |Δp| * |dG/dp| / |G|
    // Approximation: use DC gains
    double G_dc = std::abs(G_nom.dcgain());
    double dG_dc = std::abs(dG_dp.dcgain());
    
    if (G_dc < 1e-10) G_dc = 1.0;
    
    double r0 = delta_p * dG_dc / G_dc;
    
    // High-frequency behavior (use leading coefficients from Polynomial)
    const auto& num_G = G_nom.getNum().coeffs;
    const auto& num_dG = dG_dp.getNum().coeffs;
    double r_inf = delta_p * (num_dG.empty() ? 1.0 : num_dG.front()) / 
                   (num_G.empty() ? 1.0 : num_G.front());
    r_inf = std::abs(r_inf);
    if (r_inf < r0) r_inf = r0 * 2;
    
    return firstOrder(r0, r_inf);
}

} // namespace weights

// ============================================================
//            ROBUST STABILITY ANALYSIS
// ============================================================

/**
 * @brief Result of robust stability analysis
 */
struct RobustStabilityResult {
    bool isRobustlyStable;      ///< True if stable for all Δ with |Δ|≤1
    double stabilityMargin;     ///< How much uncertainty can be tolerated (>1 = robust)
    double criticalFrequency;   ///< Frequency where margin is smallest
    std::vector<double> frequencies;    ///< Analyzed frequencies
    std::vector<double> margins;        ///< Margin at each frequency
};

/**
 * @brief Check robust stability for multiplicative uncertainty
 *
 * For multiplicative uncertainty G = G_nom(1 + W_Δ Δ), closed-loop with
 * controller K is robustly stable iff:
 *   |W_Δ(jω) * T(jω)| < 1 for all ω
 * where T = GK/(1+GK) is the complementary sensitivity.
 *
 * Stability margin = 1 / max_ω |W_Δ T|
 */
inline RobustStabilityResult checkRobustStabilityMultiplicative(
    const TransferFunction& G_nom,
    const TransferFunction& K,
    const TransferFunction& W_delta,
    double w_min = 0.001,
    double w_max = 1000.0,
    int n_points = 200
) {
    RobustStabilityResult result;
    result.isRobustlyStable = true;
    result.stabilityMargin = std::numeric_limits<double>::infinity();
    result.criticalFrequency = 0;
    
    // Compute complementary sensitivity T = GK/(1+GK)
    auto GK = G_nom * K;
    auto T = feedback(GK, TransferFunction({1}, {1}));
    
    // Frequency sweep
    double log_w_min = std::log10(w_min);
    double log_w_max = std::log10(w_max);
    
    for (int i = 0; i < n_points; ++i) {
        double log_w = log_w_min + (log_w_max - log_w_min) * i / (n_points - 1);
        double w = std::pow(10.0, log_w);
        
        std::complex<double> jw(0, w);
        auto W_jw = W_delta.evalS(jw);
        auto T_jw = T.evalS(jw);
        
        double WT = std::abs(W_jw * T_jw);
        double margin = 1.0 / std::max(WT, 1e-15);
        
        result.frequencies.push_back(w);
        result.margins.push_back(margin);
        
        if (margin < result.stabilityMargin) {
            result.stabilityMargin = margin;
            result.criticalFrequency = w;
        }
        
        if (WT >= 1.0) {
            result.isRobustlyStable = false;
        }
    }
    
    return result;
}

/**
 * @brief Check robust stability for additive uncertainty
 *
 * For additive uncertainty G = G_nom + W_a Δ, closed-loop with K is
 * robustly stable iff:
 *   |W_a(jω) * K * S(jω)| < 1 for all ω
 * where S = 1/(1+GK) is the sensitivity.
 */
inline RobustStabilityResult checkRobustStabilityAdditive(
    const TransferFunction& G_nom,
    const TransferFunction& K,
    const TransferFunction& W_a,
    double w_min = 0.001,
    double w_max = 1000.0,
    int n_points = 200
) {
    RobustStabilityResult result;
    result.isRobustlyStable = true;
    result.stabilityMargin = std::numeric_limits<double>::infinity();
    result.criticalFrequency = 0;
    
    // Compute KS = K/(1+GK)
    auto GK = G_nom * K;
    auto one_plus_GK = TransferFunction({1}, {1}) + GK;
    
    // Frequency sweep
    double log_w_min = std::log10(w_min);
    double log_w_max = std::log10(w_max);
    
    for (int i = 0; i < n_points; ++i) {
        double log_w = log_w_min + (log_w_max - log_w_min) * i / (n_points - 1);
        double w = std::pow(10.0, log_w);
        
        std::complex<double> jw(0, w);
        auto W_jw = W_a.evalS(jw);
        auto K_jw = K.evalS(jw);
        auto L_jw = one_plus_GK.evalS(jw);
        
        std::complex<double> KS_jw = K_jw / L_jw;
        double WKS = std::abs(W_jw * KS_jw);
        double margin = 1.0 / std::max(WKS, 1e-15);
        
        result.frequencies.push_back(w);
        result.margins.push_back(margin);
        
        if (margin < result.stabilityMargin) {
            result.stabilityMargin = margin;
            result.criticalFrequency = w;
        }
        
        if (WKS >= 1.0) {
            result.isRobustlyStable = false;
        }
    }
    
    return result;
}

/**
 * @brief Check robust stability for parametric uncertainty
 *
 * Uses vertex analysis (Kharitonov-like) for interval polynomial uncertainty.
 * Tests stability at all 2^n vertices of the parameter space.
 *
 * @param G_factory Function that creates G given parameter values
 * @param K Controller
 * @param params Uncertain parameters
 */
template<typename GFactory>
inline RobustStabilityResult checkRobustStabilityParametric(
    GFactory G_factory,
    const TransferFunction& K,
    const ParametricUncertaintySet& params
) {
    RobustStabilityResult result;
    result.isRobustlyStable = true;
    result.stabilityMargin = std::numeric_limits<double>::infinity();
    
    auto vertices = params.getAllVertices();
    
    for (const auto& vertex : vertices) {
        auto G = G_factory(vertex);
        auto closed_loop = feedback(G * K, TransferFunction({1}, {1}));
        
        // Check stability
        bool stable = closed_loop.isStable();
        
        if (!stable) {
            result.isRobustlyStable = false;
            result.stabilityMargin = 0;
            return result;
        }
        
        // Compute stability margin for this vertex
        auto m = margin(G * K);
        double gm = std::pow(10.0, m.Gm_dB / 20.0);  // Convert dB to linear
        double pm_margin = m.Pm / 60.0;  // Normalize to ~1 at 60° PM
        
        double vertex_margin = std::min(gm, pm_margin);
        result.stabilityMargin = std::min(result.stabilityMargin, vertex_margin);
    }
    
    return result;
}

// ============================================================
//          ROBUST PERFORMANCE ANALYSIS
// ============================================================

/**
 * @brief Result of robust performance analysis
 */
struct RobustPerformanceResult {
    bool achievesRP;                ///< True if robust performance achieved
    double performanceMargin;       ///< Performance margin (>1 = achieved)
    double criticalFrequency;       ///< Frequency where margin is smallest
    double peakSensitivity;         ///< Max |S| over uncertainty set
    double peakComplementary;       ///< Max |T| over uncertainty set
};

/**
 * @brief Check robust performance
 *
 * Robust performance is achieved if:
 *   |W_p S| + |W_Δ T| < 1 for all ω
 *
 * This is equivalent to checking the structured singular value μ < 1.
 *
 * @param G_nom Nominal plant
 * @param K Controller  
 * @param W_perf Performance weight (typically on S)
 * @param W_delta Uncertainty weight
 */
inline RobustPerformanceResult checkRobustPerformance(
    const TransferFunction& G_nom,
    const TransferFunction& K,
    const TransferFunction& W_perf,
    const TransferFunction& W_delta,
    double w_min = 0.001,
    double w_max = 1000.0,
    int n_points = 200
) {
    RobustPerformanceResult result;
    result.achievesRP = true;
    result.performanceMargin = std::numeric_limits<double>::infinity();
    result.criticalFrequency = 0;
    result.peakSensitivity = 0;
    result.peakComplementary = 0;
    
    // Compute S and T
    auto GK = G_nom * K;
    auto L = TransferFunction({1}, {1}) + GK;
    auto T = feedback(GK, TransferFunction({1}, {1}));
    
    // Frequency sweep
    double log_w_min = std::log10(w_min);
    double log_w_max = std::log10(w_max);
    
    for (int i = 0; i < n_points; ++i) {
        double log_w = log_w_min + (log_w_max - log_w_min) * i / (n_points - 1);
        double w = std::pow(10.0, log_w);
        
        std::complex<double> jw(0, w);
        
        auto L_jw = L.evalS(jw);
        auto S_jw = 1.0 / L_jw;
        auto T_jw = T.evalS(jw);
        auto Wp_jw = W_perf.evalS(jw);
        auto Wd_jw = W_delta.evalS(jw);
        
        double WpS = std::abs(Wp_jw * S_jw);
        double WdT = std::abs(Wd_jw * T_jw);
        
        // RP condition: |W_p S| + |W_Δ T| < 1
        double rp_value = WpS + WdT;
        double margin = 1.0 / std::max(rp_value, 1e-15);
        
        if (margin < result.performanceMargin) {
            result.performanceMargin = margin;
            result.criticalFrequency = w;
        }
        
        result.peakSensitivity = std::max(result.peakSensitivity, std::abs(S_jw));
        result.peakComplementary = std::max(result.peakComplementary, std::abs(T_jw));
        
        if (rp_value >= 1.0) {
            result.achievesRP = false;
        }
    }
    
    return result;
}

// ============================================================
//              UNCERTAINTY DISK PLOTTING
// ============================================================

/**
 * @brief Data for uncertainty disk visualization
 */
struct UncertaintyDiskData {
    std::vector<double> frequencies;
    std::vector<double> nominal_real;
    std::vector<double> nominal_imag;
    std::vector<double> radii;          // Disk radius at each frequency
};

/**
 * @brief Compute uncertainty disk data for Nyquist-like visualization
 */
inline UncertaintyDiskData computeUncertaintyDisks(
    const TransferFunction& G_nom,
    const TransferFunction& W_delta,
    UncertaintyType type = UncertaintyType::MULTIPLICATIVE,
    double w_min = 0.001,
    double w_max = 100.0,
    int n_points = 50
) {
    UncertaintyDiskData data;
    
    double log_w_min = std::log10(w_min);
    double log_w_max = std::log10(w_max);
    
    for (int i = 0; i < n_points; ++i) {
        double log_w = log_w_min + (log_w_max - log_w_min) * i / (n_points - 1);
        double w = std::pow(10.0, log_w);
        
        std::complex<double> jw(0, w);
        auto G_jw = G_nom.evalS(jw);
        auto W_jw = W_delta.evalS(jw);
        
        data.frequencies.push_back(w);
        data.nominal_real.push_back(G_jw.real());
        data.nominal_imag.push_back(G_jw.imag());
        
        // Radius depends on uncertainty type
        double radius;
        if (type == UncertaintyType::MULTIPLICATIVE) {
            // Disk radius = |G| * |W|
            radius = std::abs(G_jw) * std::abs(W_jw);
        } else {
            // Additive: radius = |W|
            radius = std::abs(W_jw);
        }
        data.radii.push_back(radius);
    }
    
    return data;
}

// ============================================================
//               INTERVAL POLYNOMIAL
// ============================================================

/**
 * @brief Interval polynomial for Kharitonov analysis
 *
 * Each coefficient c_i ∈ [c_i^-, c_i^+]
 */
class IntervalPolynomial {
public:
    std::vector<double> coef_min;   ///< Minimum coefficients
    std::vector<double> coef_max;   ///< Maximum coefficients
    
    IntervalPolynomial() {}
    
    IntervalPolynomial(
        const std::vector<double>& c_min,
        const std::vector<double>& c_max
    ) : coef_min(c_min), coef_max(c_max) {}
    
    /// Degree of the polynomial
    size_t degree() const { 
        return std::max(coef_min.size(), coef_max.size()) - 1; 
    }
    
    /// Get nominal polynomial (midpoints)
    Polynomial getNominal() const {
        std::vector<double> nom(degree() + 1);
        for (size_t i = 0; i <= degree(); ++i) {
            double c_lo = (i < coef_min.size()) ? coef_min[i] : 0;
            double c_hi = (i < coef_max.size()) ? coef_max[i] : 0;
            nom[i] = (c_lo + c_hi) / 2.0;
        }
        return Polynomial(nom);
    }
    
    /**
     * @brief Generate four Kharitonov polynomials
     *
     * For stability analysis, it suffices to check these four:
     * K1: min, max, min, max, ... (even-max, odd-min → max, min, max, min from c0)
     * K2: max, min, max, min, ...
     * K3: min, min, max, max, ...
     * K4: max, max, min, min, ...
     */
    std::vector<Polynomial> getKharitonovPolynomials() const {
        size_t n = degree() + 1;
        std::vector<Polynomial> kp(4);
        
        for (int k = 0; k < 4; ++k) {
            std::vector<double> coeffs(n);
            for (size_t i = 0; i < n; ++i) {
                double c_lo = (i < coef_min.size()) ? coef_min[i] : 0;
                double c_hi = (i < coef_max.size()) ? coef_max[i] : 0;
                
                // Selection pattern for Kharitonov
                bool use_max;
                switch(k) {
                    case 0: use_max = (i % 4 == 1) || (i % 4 == 2); break;  // K1
                    case 1: use_max = (i % 4 == 0) || (i % 4 == 3); break;  // K2
                    case 2: use_max = (i % 4 == 2) || (i % 4 == 3); break;  // K3
                    case 3: use_max = (i % 4 == 0) || (i % 4 == 1); break;  // K4
                    default: use_max = false;
                }
                coeffs[i] = use_max ? c_hi : c_lo;
            }
            kp[k] = Polynomial(coeffs);
        }
        
        return kp;
    }
    
    /**
     * @brief Check robust stability using Kharitonov's theorem
     *
     * For a polynomial with interval coefficients, it is Hurwitz stable
     * for all coefficient combinations iff all 4 Kharitonov polynomials
     * are Hurwitz stable.
     */
    bool isRobustlyStable() const {
        auto kp = getKharitonovPolynomials();
        
        for (const auto& p : kp) {
            auto roots = p.roots();
            for (const auto& r : roots) {
                if (r.real() >= 0) return false;
            }
        }
        return true;
    }
};

// ============================================================
//            COPRIME FACTOR UNCERTAINTY
// ============================================================

/**
 * @brief Coprime factorization for robust control
 *
 * Represents G = N/M where N, M are coprime.
 * Uncertainty: G_pert = (N + Δ_N) / (M + Δ_M)
 */
struct CoprimeFactors {
    TransferFunction N;     ///< Numerator factor
    TransferFunction M;     ///< Denominator factor (stable, minimum phase)
    TransferFunction W_N;   ///< Uncertainty bound on N
    TransferFunction W_M;   ///< Uncertainty bound on M
    
    /// Reconstruct plant from coprime factors
    TransferFunction getPlant() const {
        return N / M;
    }
    
    /**
     * @brief Normalized coprime factorization
     *
     * For G(s) with poles p_i, choose:
     *   M(s) = Π(s + p_i) / Π(s - p_i*)  for unstable poles
     *   N(s) = G(s) * M(s)
     */
    static CoprimeFactors fromPlant(const TransferFunction& G) {
        CoprimeFactors cf;
        
        // For a stable plant, simple factorization
        auto poles = G.poles();
        
        // Build M to stabilize any RHP poles
        std::vector<double> m_num = {1};
        std::vector<double> m_den = {1};
        
        for (const auto& p : poles) {
            if (p.real() > 0) {
                // Reflect RHP pole
                // (s - p)(s - p*) / (s + |p|)²
                // Simplified: just use stable pole
                double stable_pole = -std::abs(p);
                // m_den = conv(m_den, [1, -stable_pole])
                std::vector<double> new_den(m_den.size() + 1);
                for (size_t i = 0; i < m_den.size(); ++i) {
                    new_den[i] += m_den[i];
                    new_den[i + 1] -= stable_pole * m_den[i];
                }
                m_den = new_den;
            }
        }
        
        cf.M = TransferFunction(m_num, m_den);
        cf.N = G * cf.M;
        
        // Default uncertainty weights
        cf.W_N = TransferFunction({0.1}, {1});
        cf.W_M = TransferFunction({0.1}, {1});
        
        return cf;
    }
};

// ============================================================
//               SUMMARY FUNCTION
// ============================================================

/**
 * @brief Print uncertainty analysis summary
 */
inline void printUncertaintySummary(
    const TransferFunction& G_nom,
    const UnstructuredUncertainty& unc,
    const TransferFunction& K
) {
    std::cout << "=============== Uncertainty Analysis ===============\n";
    std::cout << "\nNominal Plant:\n" << G_nom.toString() << "\n";
    std::cout << "\nUncertainty Type: ";
    switch (unc.type) {
        case UncertaintyType::MULTIPLICATIVE: std::cout << "Multiplicative"; break;
        case UncertaintyType::ADDITIVE: std::cout << "Additive"; break;
        case UncertaintyType::INV_MULTIPLICATIVE: std::cout << "Inverse Multiplicative"; break;
        default: std::cout << "Unknown";
    }
    std::cout << "\n\nUncertainty Weight:\n" << unc.weight.toString() << "\n";
    
    // Robust stability check
    auto rs = (unc.type == UncertaintyType::MULTIPLICATIVE) ?
              checkRobustStabilityMultiplicative(G_nom, K, unc.weight) :
              checkRobustStabilityAdditive(G_nom, K, unc.weight);
    
    std::cout << "\n--- Robust Stability ---\n";
    std::cout << "Robustly Stable: " << (rs.isRobustlyStable ? "YES" : "NO") << "\n";
    std::cout << "Stability Margin: " << rs.stabilityMargin << "\n";
    std::cout << "Critical Frequency: " << rs.criticalFrequency << " rad/s\n";
    
    std::cout << "===================================================\n";
}

} // namespace robust
} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_ROBUST_UNCERTAINTY_HPP
