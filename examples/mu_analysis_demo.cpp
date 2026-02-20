/**
 * @file mu_analysis_demo.cpp
 * @brief Demonstration of μ-analysis (structured singular value) for robust control
 * 
 * Shows D-scaling upper bound, μ frequency sweep, and robust performance analysis.
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <complex>
#include <algorithm>

namespace mu_demo {

//-----------------------------------------------------------------------------
// Complex number helpers
//-----------------------------------------------------------------------------
using Complex = std::complex<double>;

Complex operator*(double a, const Complex& b) { return Complex(a) * b; }

//-----------------------------------------------------------------------------
// 2x2 Complex Matrix for M-Δ framework
//-----------------------------------------------------------------------------
class CMatrix2x2 {
public:
    Complex m[2][2];
    
    CMatrix2x2() {
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                m[i][j] = 0.0;
    }
    
    CMatrix2x2(Complex m00, Complex m01, Complex m10, Complex m11) {
        m[0][0] = m00; m[0][1] = m01;
        m[1][0] = m10; m[1][1] = m11;
    }
    
    // Singular values (for 2x2)
    double maxSingularValue() const {
        // σ_max(M) = sqrt(max eigenvalue of M*M^H)
        // For 2x2, we can compute directly
        
        // M*M^H
        Complex a = m[0][0]*std::conj(m[0][0]) + m[0][1]*std::conj(m[0][1]);
        Complex b = m[0][0]*std::conj(m[1][0]) + m[0][1]*std::conj(m[1][1]);
        Complex c = m[1][0]*std::conj(m[0][0]) + m[1][1]*std::conj(m[0][1]);
        Complex d = m[1][0]*std::conj(m[1][0]) + m[1][1]*std::conj(m[1][1]);
        
        // Eigenvalues of [a b; c d]
        Complex trace = a + d;
        Complex det = a*d - b*c;
        
        Complex disc = trace*trace - 4.0*det;
        double disc_real = std::real(disc);
        
        double lambda_max;
        if (disc_real >= 0) {
            lambda_max = std::real(trace + std::sqrt(disc)) / 2.0;
        } else {
            lambda_max = std::real(trace) / 2.0;
        }
        
        return std::sqrt(std::max(0.0, lambda_max));
    }
    
    double minSingularValue() const {
        Complex a = m[0][0]*std::conj(m[0][0]) + m[0][1]*std::conj(m[0][1]);
        Complex b = m[0][0]*std::conj(m[1][0]) + m[0][1]*std::conj(m[1][1]);
        Complex c = m[1][0]*std::conj(m[0][0]) + m[1][1]*std::conj(m[0][1]);
        Complex d = m[1][0]*std::conj(m[1][0]) + m[1][1]*std::conj(m[1][1]);
        
        Complex trace = a + d;
        Complex det = a*d - b*c;
        Complex disc = trace*trace - 4.0*det;
        
        double lambda_min = std::real(trace - std::sqrt(disc)) / 2.0;
        return std::sqrt(std::max(0.0, lambda_min));
    }
    
    void print(const std::string& name) const {
        std::cout << name << " =\n";
        for (int i = 0; i < 2; i++) {
            std::cout << "  [";
            for (int j = 0; j < 2; j++) {
                std::cout << " (" << std::setw(6) << std::setprecision(3) 
                          << std::real(m[i][j]) << " + " 
                          << std::setw(6) << std::imag(m[i][j]) << "j)";
            }
            std::cout << " ]\n";
        }
    }
};

//-----------------------------------------------------------------------------
// Transfer function for frequency response
//-----------------------------------------------------------------------------
struct TransferFunction {
    std::vector<double> num, den;
    
    TransferFunction(std::vector<double> n, std::vector<double> d) : num(n), den(d) {}
    
    Complex eval(double omega) const {
        Complex s(0, omega);
        Complex n_val(0), d_val(0);
        
        for (size_t i = 0; i < num.size(); i++)
            n_val += num[i] * std::pow(s, double(num.size() - 1 - i));
        for (size_t i = 0; i < den.size(); i++)
            d_val += den[i] * std::pow(s, double(den.size() - 1 - i));
        
        return n_val / d_val;
    }
};

//-----------------------------------------------------------------------------
// μ Upper Bound via D-scaling
//-----------------------------------------------------------------------------

/**
 * @brief Compute μ upper bound for 2x2 matrix with diagonal Δ structure
 * 
 * For Δ = diag(δ₁, δ₂), the upper bound is:
 * μ(M) ≤ min_d σ̄(diag(d,1) M diag(1/d,1))
 */
double muUpperBound2x2_diagonal(const CMatrix2x2& M, int nSearch = 100) {
    double mu_min = 1e10;
    
    // Search over d > 0
    for (int i = 0; i < nSearch; i++) {
        double d = 0.01 * std::exp(std::log(100.0) * i / (nSearch - 1));
        
        // D M D^{-1} where D = diag(d, 1)
        CMatrix2x2 DMDinv;
        DMDinv.m[0][0] = M.m[0][0];              // d * m00 * (1/d) = m00
        DMDinv.m[0][1] = M.m[0][1] * d;          // d * m01 * 1 = d*m01
        DMDinv.m[1][0] = M.m[1][0] / d;          // 1 * m10 * (1/d) = m10/d
        DMDinv.m[1][1] = M.m[1][1];              // 1 * m11 * 1 = m11
        
        double sigma = DMDinv.maxSingularValue();
        mu_min = std::min(mu_min, sigma);
    }
    
    return mu_min;
}

/**
 * @brief Compute μ upper bound for full complex Δ (unstructured)
 * For unstructured uncertainty: μ(M) = σ̄(M)
 */
double muUpperBound_full(const CMatrix2x2& M) {
    return M.maxSingularValue();
}

/**
 * @brief Compute μ lower bound using power iteration
 * Lower bound: find Δ such that det(I - MΔ) = 0
 */
double muLowerBound2x2(const CMatrix2x2& M, int nIter = 50) {
    // For 2x2 diagonal Δ, we search for δ₁, δ₂ with |δᵢ| ≤ 1
    // such that det(I - M·diag(δ₁,δ₂)) = 0
    
    // Simplified: use real scalars
    double mu_lb = 0.0;
    
    for (int i = 0; i < 100; i++) {
        double theta1 = 2 * M_PI * i / 100;
        Complex delta1 = std::exp(Complex(0, theta1));
        
        for (int j = 0; j < 100; j++) {
            double theta2 = 2 * M_PI * j / 100;
            Complex delta2 = std::exp(Complex(0, theta2));
            
            // I - M·Δ = [1-m00·δ₁, -m01·δ₂; -m10·δ₁, 1-m11·δ₂]
            Complex det = (1.0 - M.m[0][0]*delta1) * (1.0 - M.m[1][1]*delta2)
                        - M.m[0][1]*delta2 * M.m[1][0]*delta1;
            
            if (std::abs(det) < 0.1) {
                mu_lb = std::max(mu_lb, 1.0);  // Destabilizing Δ found with |Δ|=1
            }
        }
    }
    
    return mu_lb;
}

//-----------------------------------------------------------------------------
// Build M matrix for robust performance analysis
//-----------------------------------------------------------------------------

/**
 * @brief Construct M matrix for mixed uncertainty and performance
 * 
 * Standard M-Δ structure for robust performance:
 * 
 *     ┌─────────────────┐
 *     │       M         │
 *     │  [M11  M12]     │
 *     │  [M21  M22]     │
 *     └─────────────────┘
 *           │
 *     ┌─────┴─────┐
 *     │ Δ = [Δp 0]│   Δp: performance block
 *     │     [0 Δu]│   Δu: uncertainty block
 *     └───────────┘
 * 
 * Where:
 *   M11 = W_p·S     (performance channel)
 *   M12 = W_p·T     
 *   M21 = -W_u·S    
 *   M22 = W_u·T     (uncertainty channel)
 */
CMatrix2x2 buildMMatrix(const TransferFunction& G, 
                         const TransferFunction& K,
                         const TransferFunction& Wp,
                         const TransferFunction& Wu,
                         double omega) {
    Complex G_val = G.eval(omega);
    Complex K_val = K.eval(omega);
    Complex Wp_val = Wp.eval(omega);
    Complex Wu_val = Wu.eval(omega);
    
    Complex L = G_val * K_val;           // Loop transfer function
    Complex S = 1.0 / (1.0 + L);         // Sensitivity
    Complex T = L / (1.0 + L);           // Complementary sensitivity
    
    CMatrix2x2 M;
    M.m[0][0] = Wp_val * S;              // Performance: W_p·S
    M.m[0][1] = Wp_val * T;              // Cross term
    M.m[1][0] = Wu_val * K_val * S;      // W_u·KS
    M.m[1][1] = Wu_val * T;              // Uncertainty: W_u·T
    
    return M;
}

//-----------------------------------------------------------------------------
// Demonstrations
//-----------------------------------------------------------------------------

void demo1_BasicMu() {
    std::cout << R"(
╔══════════════════════════════════════════════════════════════════════╗
║   DEMO 1: BASIC μ COMPUTATION                                        ║
╚══════════════════════════════════════════════════════════════════════╝
)";
    
    std::cout << "\n----- Example M Matrix -----\n";
    
    // Create a simple M matrix
    CMatrix2x2 M(Complex(0.5, 0.1), Complex(0.2, -0.1),
                 Complex(0.3, 0.2), Complex(0.4, 0.0));
    M.print("M");
    
    std::cout << "\n----- Singular Values -----\n";
    std::cout << "  σ_max(M) = " << M.maxSingularValue() << "\n";
    std::cout << "  σ_min(M) = " << M.minSingularValue() << "\n";
    
    std::cout << "\n----- μ Bounds for Diagonal Δ = diag(δ₁, δ₂) -----\n";
    double mu_ub = muUpperBound2x2_diagonal(M);
    double mu_full = muUpperBound_full(M);
    
    std::cout << "  μ_ub (D-scaling):  " << mu_ub << "\n";
    std::cout << "  σ_max (full Δ):    " << mu_full << "\n";
    
    std::cout << "\n  Note: μ_diagonal ≤ σ_max always\n";
    std::cout << "        Structure in Δ reduces conservatism\n";
}

void demo2_FrequencySweep() {
    std::cout << R"(
╔══════════════════════════════════════════════════════════════════════╗
║   DEMO 2: μ FREQUENCY SWEEP                                          ║
╚══════════════════════════════════════════════════════════════════════╝
)";
    
    // Motor plant: G(s) = K/(τs+1)
    double K_motor = 2.0;
    double tau = 0.1;
    TransferFunction G({K_motor}, {tau, 1.0});
    
    // PI Controller: C(s) = Kp + Ki/s = (Kp·s + Ki)/s
    double Kp = 5.0, Ki = 20.0;
    TransferFunction K({Kp, Ki}, {1.0, 0.0});
    
    // Performance weight (tracking): high at low freq
    TransferFunction Wp({0.5, 5.0}, {1.0, 0.25});  // |Wp(0)|=20, |Wp(∞)|=0.5
    
    // Uncertainty weight: high at high freq
    TransferFunction Wu({0.05, 0.2}, {0.025, 1.0});  // |Wu(0)|=0.2, |Wu(∞)|=2
    
    std::cout << "\n----- System Setup -----\n";
    std::cout << "  Plant:       G(s) = " << K_motor << "/(" << tau << "s + 1)\n";
    std::cout << "  Controller:  K(s) = " << Kp << " + " << Ki << "/s\n";
    std::cout << "  Perf weight: |Wp(0)| = " << Wp.eval(0.001).real() << "\n";
    std::cout << "  Unc weight:  |Wu(∞)| = " << Wu.eval(1000).real() << "\n";
    
    std::cout << "\n----- μ Sweep over Frequency -----\n";
    std::cout << "  ω(rad/s)   |S|      |T|      |Wp·S|   |Wu·T|    μ_ub\n";
    std::cout << "  --------   ----     ----     ------   ------    -----\n";
    
    double mu_peak = 0;
    double omega_peak = 0;
    
    std::vector<double> freqs = {0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0};
    
    for (double omega : freqs) {
        Complex G_val = G.eval(omega);
        Complex K_val = K.eval(omega);
        Complex L = G_val * K_val;
        Complex S = 1.0 / (1.0 + L);
        Complex T = L / (1.0 + L);
        
        double S_mag = std::abs(S);
        double T_mag = std::abs(T);
        double WpS = std::abs(Wp.eval(omega) * S);
        double WuT = std::abs(Wu.eval(omega) * T);
        
        CMatrix2x2 M = buildMMatrix(G, K, Wp, Wu, omega);
        double mu = muUpperBound2x2_diagonal(M);
        
        if (mu > mu_peak) {
            mu_peak = mu;
            omega_peak = omega;
        }
        
        std::cout << "  " << std::setw(8) << omega
                  << "   " << std::setw(5) << std::fixed << std::setprecision(3) << S_mag
                  << "    " << std::setw(5) << T_mag
                  << "    " << std::setw(6) << WpS
                  << "   " << std::setw(6) << WuT
                  << "    " << std::setw(5) << mu << "\n";
    }
    
    std::cout << "\n----- Robust Performance Result -----\n";
    std::cout << "  Peak μ = " << mu_peak << " at ω = " << omega_peak << " rad/s\n";
    
    if (mu_peak < 1.0) {
        std::cout << "  ✓ ROBUST PERFORMANCE ACHIEVED\n";
        std::cout << "  RP margin = " << 1.0/mu_peak << "\n";
    } else {
        std::cout << "  ✗ ROBUST PERFORMANCE NOT ACHIEVED\n";
        std::cout << "  Need to redesign or reduce requirements\n";
    }
}

void demo3_DKIteration() {
    std::cout << R"(
╔══════════════════════════════════════════════════════════════════════╗
║   DEMO 3: D-K ITERATION CONCEPT                                      ║
╚══════════════════════════════════════════════════════════════════════╝
)";
    
    std::cout << R"(
----- D-K Iteration Algorithm -----

The D-K iteration alternates between:

1. K-STEP: Fix D(s), design K to minimize ||D·M(K)·D⁻¹||_∞
   - This is a standard H∞ problem
   - Use hinf_state_feedback() or hinf_output_feedback()

2. D-STEP: Fix K, optimize D(jω) at each frequency
   - min_D σ̄(D·M·D⁻¹)
   - Fit rational D(s) to point-wise optimal D(jω)

Repeat until μ converges.

----- Simplified Example -----
)";
    
    // Motor and initial controller
    TransferFunction G({2.0}, {0.1, 1.0});
    TransferFunction K_init({5.0, 20.0}, {1.0, 0.0});
    
    TransferFunction Wp({0.5, 5.0}, {1.0, 0.25});
    TransferFunction Wu({0.05, 0.2}, {0.025, 1.0});
    
    std::cout << "\n  Initial Controller: K(s) = 5 + 20/s (PI)\n";
    
    // Compute μ at several frequencies
    std::vector<double> freqs = {0.1, 1.0, 10.0, 100.0};
    std::vector<double> d_opt(freqs.size());
    
    std::cout << "\n  D-STEP: Finding optimal D at each frequency\n";
    std::cout << "  ω        μ_ub     d_opt\n";
    std::cout << "  ----     -----    -----\n";
    
    for (size_t i = 0; i < freqs.size(); i++) {
        double omega = freqs[i];
        CMatrix2x2 M = buildMMatrix(G, K_init, Wp, Wu, omega);
        
        // Find optimal d
        double mu_min = 1e10;
        double d_best = 1.0;
        
        for (int j = 0; j < 100; j++) {
            double d = 0.01 * std::exp(std::log(100.0) * j / 99.0);
            
            CMatrix2x2 DMDinv;
            DMDinv.m[0][0] = M.m[0][0];
            DMDinv.m[0][1] = M.m[0][1] * d;
            DMDinv.m[1][0] = M.m[1][0] / d;
            DMDinv.m[1][1] = M.m[1][1];
            
            double sigma = DMDinv.maxSingularValue();
            if (sigma < mu_min) {
                mu_min = sigma;
                d_best = d;
            }
        }
        
        d_opt[i] = d_best;
        std::cout << "  " << std::setw(5) << omega
                  << "    " << std::setw(5) << std::setprecision(3) << mu_min
                  << "    " << std::setw(5) << d_best << "\n";
    }
    
    std::cout << "\n  K-STEP: Would redesign K using H∞ with D-scaled plant\n";
    std::cout << "  (Full implementation requires iterating until convergence)\n";
    
    std::cout << R"(
----- Convergence Properties -----

• D-K iteration is NOT guaranteed to converge to global minimum
• May converge to local minimum
• In practice, often achieves near-optimal results in 3-5 iterations
• Computational tools: MATLAB's dksyn, μ-Tools
)";
}

void demo4_RobustnessInterpretation() {
    std::cout << R"(
╔══════════════════════════════════════════════════════════════════════╗
║   DEMO 4: INTERPRETING μ FOR MOTOR CONTROL                           ║
╚══════════════════════════════════════════════════════════════════════╝
)";
    
    std::cout << R"(
----- Structured Singular Value Meaning -----

For the M-Δ loop with Δ = diag(Δ_perf, Δ_unc):

  μ < 1  →  Robust performance achieved
            Both performance specs AND stability guaranteed
            for ALL plants in uncertainty set
            
  μ = 1  →  Boundary case
            RP achieved marginally
            
  μ > 1  →  Robust performance NOT achieved
            Either stability OR performance violated
            for some plant in uncertainty set

----- Margins -----

  RP Margin = 1/μ_peak
  
  • RP Margin > 1: System is robust
  • RP Margin = 2: Can tolerate 2× the modeled uncertainty
  • RP Margin < 1: Violation at some frequency

----- Design Guidelines for Motor Control -----

  1. Start with conservative uncertainty model
     - Include ±50% inertia variation
     - Include ±100% friction variation
     - Include high-frequency unmodeled dynamics
     
  2. Set achievable performance specs
     - Bandwidth ≤ 10× of slowest uncertain pole
     - Allow for worst-case parameter combinations
     
  3. Use μ-synthesis (D-K) when:
     - Simple designs fail RP test
     - Multiple uncertainty sources interact
     - Need guaranteed performance
     
  4. Validate with Monte Carlo simulation
     - Sample 1000+ random plants
     - Verify stability AND performance
)";
    
    // Example calculation for motor
    TransferFunction G({2.0}, {0.1, 1.0});
    TransferFunction K({5.0, 20.0}, {1.0, 0.0});
    TransferFunction Wp({0.5, 5.0}, {1.0, 0.25});
    TransferFunction Wu({0.05, 0.2}, {0.025, 1.0});
    
    double mu_max = 0;
    std::vector<double> freqs;
    for (double w = 0.1; w <= 100; w *= 1.2) freqs.push_back(w);
    
    for (double omega : freqs) {
        CMatrix2x2 M = buildMMatrix(G, K, Wp, Wu, omega);
        double mu = muUpperBound2x2_diagonal(M);
        mu_max = std::max(mu_max, mu);
    }
    
    std::cout << "\n----- Example Motor System -----\n";
    std::cout << "  Peak μ = " << std::setprecision(3) << mu_max << "\n";
    std::cout << "  RP Margin = " << 1.0/mu_max << "\n";
    
    if (mu_max < 1.0) {
        std::cout << "\n  This motor control design achieves robust performance!\n";
        std::cout << "  Can tolerate " << 100.0*(1.0/mu_max - 1.0) 
                  << "% more uncertainty than modeled.\n";
    } else {
        std::cout << "\n  Design needs improvement.\n";
        std::cout << "  Consider: reducing bandwidth, increasing controller order,\n";
        std::cout << "  or relaxing performance specs.\n";
    }
}

} // namespace mu_demo

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
int main() {
    std::cout << R"(
╔══════════════════════════════════════════════════════════════════════════════╗
║                                                                              ║
║   μ-ANALYSIS DEMONSTRATION                                                   ║
║   Structured Singular Value for Robust Control                               ║
║                                                                              ║
╚══════════════════════════════════════════════════════════════════════════════╝
)";

    mu_demo::demo1_BasicMu();
    mu_demo::demo2_FrequencySweep();
    mu_demo::demo3_DKIteration();
    mu_demo::demo4_RobustnessInterpretation();
    
    std::cout << "\n════════════════════════════════════════════════════════════════════════════════\n";
    std::cout << "                          END OF DEMONSTRATION                                  \n";
    std::cout << "════════════════════════════════════════════════════════════════════════════════\n";
    
    return 0;
}
