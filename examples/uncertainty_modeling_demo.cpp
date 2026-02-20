/**
 * @file uncertainty_modeling_demo.cpp
 * @brief Demonstration of uncertainty modeling for motor control
 * 
 * Shows parametric uncertainty, multiplicative uncertainty,
 * and Kharitonov's theorem for interval polynomials.
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <complex>
#include <algorithm>
#include <random>

namespace uncertainty_demo {

//-----------------------------------------------------------------------------
// Transfer Function
//-----------------------------------------------------------------------------
class TransferFunction {
public:
    std::vector<double> num, den;
    
    TransferFunction(std::vector<double> n, std::vector<double> d) 
        : num(n), den(d) {}
    
    std::complex<double> eval(double omega) const {
        std::complex<double> s(0, omega);
        std::complex<double> n_val(0), d_val(0);
        
        for (size_t i = 0; i < num.size(); i++)
            n_val += num[i] * std::pow(s, num.size() - 1 - i);
        for (size_t i = 0; i < den.size(); i++)
            d_val += den[i] * std::pow(s, den.size() - 1 - i);
        
        return n_val / d_val;
    }
    
    double magnitude(double omega) const { return std::abs(eval(omega)); }
    double phase_deg(double omega) const { return std::arg(eval(omega)) * 180.0 / M_PI; }
    
    void print(const std::string& name) const {
        std::cout << name << "(s) = ";
        // Numerator
        std::cout << "(";
        for (size_t i = 0; i < num.size(); i++) {
            if (i > 0) std::cout << (num[i] >= 0 ? " + " : " - ");
            std::cout << std::abs(num[i]);
            int power = num.size() - 1 - i;
            if (power > 0) std::cout << "s" << (power > 1 ? "^" + std::to_string(power) : "");
        }
        std::cout << ") / (";
        // Denominator
        for (size_t i = 0; i < den.size(); i++) {
            if (i > 0) std::cout << (den[i] >= 0 ? " + " : " - ");
            std::cout << std::abs(den[i]);
            int power = den.size() - 1 - i;
            if (power > 0) std::cout << "s" << (power > 1 ? "^" + std::to_string(power) : "");
        }
        std::cout << ")\n";
    }
};

//-----------------------------------------------------------------------------
// Motor with uncertainty
//-----------------------------------------------------------------------------
struct MotorParams {
    double R, L, Km, Ke, J, B;
    
    MotorParams(double r=2.5, double l=0.005, double km=0.05, 
                double ke=0.05, double j=0.01, double b=0.002)
        : R(r), L(l), Km(km), Ke(ke), J(j), B(b) {}
    
    TransferFunction getTF() const {
        double K = Km / (B*R + Km*Ke);
        double tau = J*R / (B*R + Km*Ke);
        return TransferFunction({K}, {tau, 1.0});
    }
    
    double getDCGain() const { return Km / (B*R + Km*Ke); }
    double getTimeConst() const { return J*R / (B*R + Km*Ke); }
};

struct MotorUncertainty {
    double R_min, R_max;
    double J_min, J_max;
    double B_min, B_max;
    double Km_min, Km_max;
    
    MotorUncertainty() {
        R_min = 1.5;  R_max = 3.5;
        J_min = 0.005; J_max = 0.015;
        B_min = 0.001; B_max = 0.004;
        Km_min = 0.045; Km_max = 0.055;
    }
    
    // Generate random plant within uncertainty bounds
    MotorParams sample(std::mt19937& gen) const {
        std::uniform_real_distribution<> R_dist(R_min, R_max);
        std::uniform_real_distribution<> J_dist(J_min, J_max);
        std::uniform_real_distribution<> B_dist(B_min, B_max);
        std::uniform_real_distribution<> Km_dist(Km_min, Km_max);
        
        return MotorParams(R_dist(gen), 0.005, Km_dist(gen), 0.05, 
                          J_dist(gen), B_dist(gen));
    }
    
    // Get corner cases
    std::vector<MotorParams> getCornerPlants() const {
        std::vector<MotorParams> plants;
        
        // All 16 corners (2^4 combinations)
        for (int r = 0; r < 2; r++)
        for (int j = 0; j < 2; j++)
        for (int b = 0; b < 2; b++)
        for (int k = 0; k < 2; k++) {
            plants.push_back(MotorParams(
                r ? R_max : R_min,
                0.005,
                k ? Km_max : Km_min,
                0.05,
                j ? J_max : J_min,
                b ? B_max : B_min
            ));
        }
        
        return plants;
    }
};

//-----------------------------------------------------------------------------
// Multiplicative Uncertainty
//-----------------------------------------------------------------------------
class MultiplicativeUncertainty {
public:
    TransferFunction G_nom;
    TransferFunction W_delta;  // Uncertainty weight
    
    MultiplicativeUncertainty(const TransferFunction& g_nom,
                               double r0, double r_inf, double tau)
        : G_nom(g_nom),
          W_delta({tau, r0}, {tau/r_inf, 1.0}) {}
    
    // Check if a perturbed plant is within the uncertainty set
    bool contains(const TransferFunction& G_pert, int nPoints = 100) const {
        for (int i = 0; i < nPoints; i++) {
            double omega = 0.01 * std::pow(10.0, 4.0 * i / nPoints);
            
            auto G_n = G_nom.eval(omega);
            auto G_p = G_pert.eval(omega);
            auto W = W_delta.eval(omega);
            
            // Delta = (G_pert - G_nom) / G_nom
            auto delta = (G_p - G_n) / G_n;
            
            // Check |delta| <= |W|
            if (std::abs(delta) > std::abs(W) * 1.01) {  // 1% tolerance
                return false;
            }
        }
        return true;
    }
    
    // Get relative uncertainty at a frequency
    double relativeUncertainty(double omega) const {
        return W_delta.magnitude(omega);
    }
};

//-----------------------------------------------------------------------------
// Kharitonov's Theorem
//-----------------------------------------------------------------------------
class IntervalPolynomial {
public:
    std::vector<double> coef_min, coef_max;
    
    IntervalPolynomial(std::vector<double> c_min, std::vector<double> c_max)
        : coef_min(c_min), coef_max(c_max) {}
    
    // Get the four Kharitonov polynomials
    std::vector<std::vector<double>> getKharitonovPolynomials() const {
        int n = coef_min.size();
        std::vector<std::vector<double>> K(4, std::vector<double>(n));
        
        // Pattern: +- -- ++ -+ +- -- ...
        // K1: a0-, a1-, a2+, a3+, a4-, ...
        // K2: a0+, a1+, a2-, a3-, a4+, ...
        // K3: a0+, a1-, a2-, a3+, a4+, ...
        // K4: a0-, a1+, a2+, a3-, a4-, ...
        
        for (int i = 0; i < n; i++) {
            int pattern = i % 4;
            
            K[0][i] = (pattern == 0 || pattern == 1) ? coef_min[i] : coef_max[i];
            K[1][i] = (pattern == 0 || pattern == 1) ? coef_max[i] : coef_min[i];
            K[2][i] = (pattern == 0 || pattern == 3) ? coef_max[i] : coef_min[i];
            K[3][i] = (pattern == 0 || pattern == 3) ? coef_min[i] : coef_max[i];
        }
        
        return K;
    }
    
    // Check if polynomial is Hurwitz stable using Routh criterion
    static bool isHurwitz(const std::vector<double>& coef) {
        // Simplified check: all coefficients must be positive and same sign
        // (necessary but not sufficient for general case)
        
        if (coef.empty()) return false;
        
        double sign = coef[0] > 0 ? 1.0 : -1.0;
        for (double c : coef) {
            if (c * sign <= 0) return false;
        }
        
        // For 2nd order: a2*s² + a1*s + a0 is stable if all ai > 0
        if (coef.size() == 3) {
            return coef[0] > 0 && coef[1] > 0 && coef[2] > 0;
        }
        
        return true;  // Simplified
    }
    
    // Check robust stability via Kharitonov
    bool isRobustlyStable() const {
        auto K = getKharitonovPolynomials();
        
        for (int i = 0; i < 4; i++) {
            if (!isHurwitz(K[i])) {
                return false;
            }
        }
        return true;
    }
};

//-----------------------------------------------------------------------------
// Demonstrations
//-----------------------------------------------------------------------------

void demo1_ParametricUncertainty() {
    std::cout << R"(
╔══════════════════════════════════════════════════════════════════════╗
║   DEMO 1: PARAMETRIC UNCERTAINTY                                     ║
╚══════════════════════════════════════════════════════════════════════╝
)";
    
    MotorParams nominal;
    MotorUncertainty unc;
    
    std::cout << "\n----- Nominal Motor Parameters -----\n";
    std::cout << "  R  = " << nominal.R << " Ω\n";
    std::cout << "  J  = " << nominal.J*1000 << " g·m²\n";
    std::cout << "  B  = " << nominal.B << " Nm·s/rad\n";
    std::cout << "  Km = " << nominal.Km << " Nm/A\n";
    
    std::cout << "\n----- Parameter Uncertainty -----\n";
    std::cout << "  R  ∈ [" << unc.R_min << ", " << unc.R_max << "] Ω (±40%)\n";
    std::cout << "  J  ∈ [" << unc.J_min*1000 << ", " << unc.J_max*1000 << "] g·m² (±50%)\n";
    std::cout << "  B  ∈ [" << unc.B_min << ", " << unc.B_max << "] (±100%)\n";
    std::cout << "  Km ∈ [" << unc.Km_min << ", " << unc.Km_max << "] Nm/A (±10%)\n";
    
    // Analyze plant variations
    auto corners = unc.getCornerPlants();
    
    std::cout << "\n----- Plant Family Characteristics -----\n";
    std::cout << "  Case     DC Gain K    Time Const τ(ms)    ωn (rad/s)\n";
    std::cout << "  ------   ---------    ----------------    ----------\n";
    
    double K_min = 1e10, K_max = 0;
    double tau_min = 1e10, tau_max = 0;
    
    for (size_t i = 0; i < corners.size(); i++) {
        double K = corners[i].getDCGain();
        double tau = corners[i].getTimeConst();
        double wn = 1.0 / tau;
        
        K_min = std::min(K_min, K);
        K_max = std::max(K_max, K);
        tau_min = std::min(tau_min, tau);
        tau_max = std::max(tau_max, tau);
        
        if (i < 5 || i == corners.size()-1) {
            std::cout << "  " << std::setw(6) << i+1
                      << "   " << std::setw(9) << std::fixed << std::setprecision(3) << K
                      << "    " << std::setw(16) << tau*1000
                      << "    " << std::setw(10) << wn << "\n";
        } else if (i == 5) {
            std::cout << "  ...\n";
        }
    }
    
    std::cout << "\n----- Summary -----\n";
    std::cout << "  DC Gain range: [" << K_min << ", " << K_max << "]\n";
    std::cout << "  Time constant range: [" << tau_min*1000 << ", " << tau_max*1000 << "] ms\n";
    std::cout << "  Bandwidth range: [" << 1/tau_max << ", " << 1/tau_min << "] rad/s\n";
}

void demo2_MultiplicativeUncertainty() {
    std::cout << R"(
╔══════════════════════════════════════════════════════════════════════╗
║   DEMO 2: MULTIPLICATIVE UNCERTAINTY                                 ║
╚══════════════════════════════════════════════════════════════════════╝
)";
    
    MotorParams nominal;
    auto G_nom = nominal.getTF();
    
    std::cout << "\n----- Nominal Plant -----\n";
    G_nom.print("G_nom");
    std::cout << "  DC Gain: " << nominal.getDCGain() << "\n";
    std::cout << "  Time Const: " << nominal.getTimeConst()*1000 << " ms\n";
    
    // Create multiplicative uncertainty model
    // |W_Δ(jω)| should bound relative model error
    double r0 = 0.2;      // 20% DC uncertainty
    double r_inf = 1.5;   // 150% high-freq uncertainty
    double tau_w = 0.05;  // Crossover at 20 rad/s
    
    MultiplicativeUncertainty mult_unc(G_nom, r0, r_inf, tau_w);
    
    std::cout << "\n----- Multiplicative Uncertainty Model -----\n";
    std::cout << "  G_pert(s) = G_nom(s) × (1 + W_Δ(s)·Δ), |Δ| ≤ 1\n\n";
    mult_unc.W_delta.print("W_Δ");
    
    std::cout << "\n----- Uncertainty Envelope -----\n";
    std::cout << "  ω (rad/s)   |G_nom|   |W_Δ|    G_min      G_max\n";
    std::cout << "  ---------   ------    -----    ------      ------\n";
    
    std::vector<double> freqs = {0.1, 1.0, 5.0, 10.0, 20.0, 50.0, 100.0};
    for (double w : freqs) {
        double G_mag = G_nom.magnitude(w);
        double W_mag = mult_unc.W_delta.magnitude(w);
        double G_min = G_mag * (1 - W_mag);
        double G_max = G_mag * (1 + W_mag);
        
        std::cout << "  " << std::setw(9) << w
                  << "   " << std::setw(6) << std::fixed << std::setprecision(3) << G_mag
                  << "    " << std::setw(5) << W_mag
                  << "    " << std::setw(6) << G_min
                  << "      " << std::setw(6) << G_max << "\n";
    }
    
    // Check if corner plants are within uncertainty set
    MotorUncertainty param_unc;
    auto corners = param_unc.getCornerPlants();
    
    std::cout << "\n----- Validation: Corner Plants Coverage -----\n";
    int covered = 0;
    for (size_t i = 0; i < corners.size(); i++) {
        auto G_corner = corners[i].getTF();
        bool in_set = mult_unc.contains(G_corner);
        if (in_set) covered++;
    }
    std::cout << "  " << covered << " of " << corners.size() 
              << " corner plants covered by multiplicative model\n";
    
    if (covered < corners.size()) {
        std::cout << "  Warning: Increase W_Δ bounds to cover all uncertainty!\n";
    }
}

void demo3_KharitonovTheorem() {
    std::cout << R"(
╔══════════════════════════════════════════════════════════════════════╗
║   DEMO 3: KHARITONOV'S THEOREM                                       ║
╚══════════════════════════════════════════════════════════════════════╝
)";
    
    std::cout << "\n----- Interval Polynomial Family -----\n";
    std::cout << "  p(s) = a₂s² + a₁s + a₀\n";
    std::cout << "  where each aᵢ ∈ [aᵢ⁻, aᵢ⁺]\n\n";
    
    // For closed-loop characteristic polynomial
    // Assuming plant G = K/(τs+1) and PI controller C = Kp + Ki/s
    // Closed-loop char. poly: τs² + (1 + K·Kp)s + K·Ki
    
    // With uncertainty: K ∈ [K_min, K_max], τ ∈ [τ_min, τ_max]
    double K_min = 1.0, K_max = 5.0;
    double tau_min = 0.05, tau_max = 0.15;
    double Kp = 2.0, Ki = 10.0;  // Controller gains
    
    // Coefficient bounds
    double a2_min = tau_min, a2_max = tau_max;
    double a1_min = 1 + K_min * Kp, a1_max = 1 + K_max * Kp;
    double a0_min = K_min * Ki, a0_max = K_max * Ki;
    
    std::cout << "  Controller: Kp = " << Kp << ", Ki = " << Ki << "\n";
    std::cout << "  Plant uncertainty: K ∈ [" << K_min << ", " << K_max << "]\n";
    std::cout << "                    τ ∈ [" << tau_min << ", " << tau_max << "]\n\n";
    
    std::cout << "  Coefficient bounds:\n";
    std::cout << "    a₂ (s² coef) ∈ [" << a2_min << ", " << a2_max << "]\n";
    std::cout << "    a₁ (s coef)  ∈ [" << a1_min << ", " << a1_max << "]\n";
    std::cout << "    a₀ (const)   ∈ [" << a0_min << ", " << a0_max << "]\n";
    
    // Create interval polynomial
    IntervalPolynomial ip({a2_min, a1_min, a0_min}, {a2_max, a1_max, a0_max});
    
    std::cout << "\n----- Kharitonov Polynomials -----\n";
    auto K_polys = ip.getKharitonovPolynomials();
    
    std::vector<std::string> names = {"K₁", "K₂", "K₃", "K₄"};
    
    for (int i = 0; i < 4; i++) {
        std::cout << "  " << names[i] << "(s) = "
                  << K_polys[i][0] << "s² + "
                  << K_polys[i][1] << "s + "
                  << K_polys[i][2] << "\n";
        
        // Check stability for 2nd order: all coefs positive
        bool stable = K_polys[i][0] > 0 && K_polys[i][1] > 0 && K_polys[i][2] > 0;
        std::cout << "       Hurwitz stable: " << (stable ? "YES" : "NO") << "\n";
        
        if (stable) {
            // Compute poles
            double a = K_polys[i][0];
            double b = K_polys[i][1];
            double c = K_polys[i][2];
            double disc = b*b - 4*a*c;
            
            std::cout << "       Poles: ";
            if (disc < 0) {
                double re = -b / (2*a);
                double im = std::sqrt(-disc) / (2*a);
                std::cout << re << " ± j" << im << "\n";
            } else {
                double p1 = (-b + std::sqrt(disc)) / (2*a);
                double p2 = (-b - std::sqrt(disc)) / (2*a);
                std::cout << p1 << ", " << p2 << "\n";
            }
        }
        std::cout << "\n";
    }
    
    bool robust = ip.isRobustlyStable();
    std::cout << "----- Kharitonov Theorem Result -----\n";
    std::cout << "  The interval polynomial family is: ";
    std::cout << (robust ? "ROBUSTLY STABLE ✓" : "NOT ROBUSTLY STABLE ✗") << "\n";
}

void demo4_MonteCarlo() {
    std::cout << R"(
╔══════════════════════════════════════════════════════════════════════╗
║   DEMO 4: MONTE CARLO ROBUSTNESS ANALYSIS                            ║
╚══════════════════════════════════════════════════════════════════════╝
)";
    
    MotorUncertainty unc;
    std::mt19937 gen(42);  // Fixed seed for reproducibility
    
    // Controller: PI with Kp=5, Ki=20
    double Kp = 5.0, Ki = 20.0;
    
    std::cout << "\n----- Monte Carlo Settings -----\n";
    std::cout << "  Number of samples: 1000\n";
    std::cout << "  Controller: PI with Kp=" << Kp << ", Ki=" << Ki << "\n";
    
    // Analyze closed-loop poles for many plants
    int n_samples = 1000;
    int stable_count = 0;
    
    double pole_re_min = 1e10, pole_re_max = -1e10;
    double bandwidth_min = 1e10, bandwidth_max = 0;
    
    for (int i = 0; i < n_samples; i++) {
        MotorParams plant = unc.sample(gen);
        
        double K = plant.getDCGain();
        double tau = plant.getTimeConst();
        
        // Closed-loop: τs² + (1 + K·Kp)s + K·Ki = 0
        double a = tau;
        double b = 1 + K * Kp;
        double c = K * Ki;
        
        double disc = b*b - 4*a*c;
        double re1, re2;
        
        if (disc < 0) {
            re1 = re2 = -b / (2*a);
        } else {
            re1 = (-b + std::sqrt(disc)) / (2*a);
            re2 = (-b - std::sqrt(disc)) / (2*a);
        }
        
        // Check stability (all poles in LHP)
        if (re1 < 0 && re2 < 0) {
            stable_count++;
        }
        
        pole_re_min = std::min({pole_re_min, re1, re2});
        pole_re_max = std::max({pole_re_max, re1, re2});
        
        // Bandwidth ≈ |dominant pole|
        double bw = std::sqrt(c/a);  // ωn
        bandwidth_min = std::min(bandwidth_min, bw);
        bandwidth_max = std::max(bandwidth_max, bw);
    }
    
    std::cout << "\n----- Results -----\n";
    std::cout << "  Stable plants: " << stable_count << "/" << n_samples 
              << " (" << 100.0*stable_count/n_samples << "%)\n";
    std::cout << "  Pole real part range: [" << pole_re_min << ", " << pole_re_max << "]\n";
    std::cout << "  Bandwidth range: [" << bandwidth_min << ", " << bandwidth_max << "] rad/s\n";
    
    if (stable_count == n_samples) {
        std::cout << "\n  ✓ Controller provides robust stability!\n";
    } else {
        std::cout << "\n  ✗ Some plants result in instability. Redesign needed.\n";
    }
}

} // namespace uncertainty_demo

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
int main() {
    std::cout << R"(
╔══════════════════════════════════════════════════════════════════════════════╗
║                                                                              ║
║   UNCERTAINTY MODELING DEMONSTRATION                                         ║
║   Application: DC Motor for Differential Drive Robot                         ║
║                                                                              ║
╚══════════════════════════════════════════════════════════════════════════════╝
)";

    uncertainty_demo::demo1_ParametricUncertainty();
    uncertainty_demo::demo2_MultiplicativeUncertainty();
    uncertainty_demo::demo3_KharitonovTheorem();
    uncertainty_demo::demo4_MonteCarlo();
    
    std::cout << "\n════════════════════════════════════════════════════════════════════════════════\n";
    std::cout << "                          END OF DEMONSTRATION                                  \n";
    std::cout << "════════════════════════════════════════════════════════════════════════════════\n";
    
    return 0;
}
