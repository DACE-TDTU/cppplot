/**
 * @file run_case_study.cpp
 * @brief Main runner for Robust Control Case Studies
 * 
 * This program provides an interactive menu to run various robust control
 * demonstrations applied to differential drive mobile robot motor control.
 * 
 * Topics covered:
 * - H∞ state-feedback and output-feedback design
 * - Uncertainty modeling (parametric and multiplicative)
 * - Kharitonov's theorem for interval polynomials
 * - μ-analysis (structured singular value)
 * - Robust stability and performance analysis
 * 
 * Build (standalone):
 *   g++ -std=c++14 run_case_study.cpp -o run_case_study
 * 
 * Build (with CppPlot library):
 *   g++ -std=c++14 -I../include run_case_study.cpp -o run_case_study
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <complex>
#include <string>
#include <functional>
#include <limits>
#include <fstream>
#include <algorithm>

//=============================================================================
// UTILITY CLASSES
//=============================================================================

namespace robust_control {

using Complex = std::complex<double>;
const double PI = 3.14159265358979323846;

//-----------------------------------------------------------------------------
// ASCII Plotting Functions
//-----------------------------------------------------------------------------
void drawASCIIPlot(const std::vector<double>& x, const std::vector<double>& y,
                   const std::string& title, const std::string& xlabel = "x",
                   const std::string& ylabel = "y", int width = 60, int height = 15) {
    
    if (x.empty() || y.empty()) return;
    
    double x_min = *std::min_element(x.begin(), x.end());
    double x_max = *std::max_element(x.begin(), x.end());
    double y_min = *std::min_element(y.begin(), y.end());
    double y_max = *std::max_element(y.begin(), y.end());
    
    double y_range = y_max - y_min;
    if (y_range < 1e-6) y_range = 1.0;
    y_min -= y_range * 0.1;
    y_max += y_range * 0.1;
    y_range = y_max - y_min;
    
    double x_range = x_max - x_min;
    if (x_range < 1e-6) x_range = 1.0;
    
    std::vector<std::string> canvas(height, std::string(width, ' '));
    
    for (size_t i = 0; i < x.size(); i++) {
        int col = static_cast<int>((x[i] - x_min) / x_range * (width - 1));
        int row = static_cast<int>((y_max - y[i]) / y_range * (height - 1));
        col = std::max(0, std::min(width - 1, col));
        row = std::max(0, std::min(height - 1, row));
        canvas[row][col] = '*';
    }
    
    std::cout << "\n  " << title << "\n";
    std::cout << "  " << std::string(width + 10, '-') << "\n";
    
    for (int r = 0; r < height; r++) {
        double y_val = y_max - r * y_range / (height - 1);
        if (r == 0 || r == height - 1 || r == height / 2) {
            std::cout << std::setw(8) << std::fixed << std::setprecision(2) << y_val << " |";
        } else {
            std::cout << "         |";
        }
        std::cout << canvas[r] << "\n";
    }
    
    std::cout << "         +" << std::string(width, '-') << "\n";
    std::cout << "         " << std::setw(8) << std::setprecision(2) << x_min
              << std::setw(width/2 - 4) << xlabel
              << std::setw(width/2 - 4) << std::setprecision(2) << x_max << "\n";
}

void drawASCIIPlotMulti(const std::vector<double>& x,
                        const std::vector<std::vector<double>>& ys,
                        const std::vector<char>& markers,
                        const std::vector<std::string>& labels,
                        const std::string& title,
                        int width = 60, int height = 15) {
    
    if (x.empty() || ys.empty()) return;
    
    double x_min = *std::min_element(x.begin(), x.end());
    double x_max = *std::max_element(x.begin(), x.end());
    double y_min = 1e10, y_max = -1e10;
    
    for (const auto& y : ys) {
        y_min = std::min(y_min, *std::min_element(y.begin(), y.end()));
        y_max = std::max(y_max, *std::max_element(y.begin(), y.end()));
    }
    
    double y_range = y_max - y_min;
    if (y_range < 1e-6) y_range = 1.0;
    y_min -= y_range * 0.1;
    y_max += y_range * 0.1;
    y_range = y_max - y_min;
    
    double x_range = x_max - x_min;
    if (x_range < 1e-6) x_range = 1.0;
    
    std::vector<std::string> canvas(height, std::string(width, ' '));
    
    for (size_t k = 0; k < ys.size(); k++) {
        char marker = (k < markers.size()) ? markers[k] : '*';
        for (size_t i = 0; i < x.size(); i++) {
            int col = static_cast<int>((x[i] - x_min) / x_range * (width - 1));
            int row = static_cast<int>((y_max - ys[k][i]) / y_range * (height - 1));
            col = std::max(0, std::min(width - 1, col));
            row = std::max(0, std::min(height - 1, row));
            if (canvas[row][col] == ' ') canvas[row][col] = marker;
        }
    }
    
    std::cout << "\n  " << title << "\n";
    std::cout << "  " << std::string(width + 10, '=') << "\n";
    
    for (int r = 0; r < height; r++) {
        double y_val = y_max - r * y_range / (height - 1);
        if (r == 0 || r == height - 1 || r == height / 2) {
            std::cout << std::setw(8) << std::fixed << std::setprecision(2) << y_val << " |";
        } else {
            std::cout << "         |";
        }
        std::cout << canvas[r] << "\n";
    }
    
    std::cout << "         +" << std::string(width, '-') << "\n";
    
    std::cout << "  Legend: ";
    for (size_t k = 0; k < labels.size() && k < markers.size(); k++) {
        std::cout << "[" << markers[k] << "] " << labels[k] << "  ";
    }
    std::cout << "\n";
}

void saveStepResponseCSV(const std::vector<double>& t, const std::vector<double>& y,
                         const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) return;
    file << "time,response\n";
    for (size_t i = 0; i < t.size(); i++) {
        file << t[i] << "," << y[i] << "\n";
    }
    file.close();
    std::cout << "  Data saved to: " << filename << "\n";
}

void saveBodeDataCSV(const std::vector<double>& freq, 
                     const std::vector<double>& mag,
                     const std::vector<double>& phase,
                     const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) return;
    file << "frequency,magnitude_dB,phase_deg\n";
    for (size_t i = 0; i < freq.size(); i++) {
        file << freq[i] << "," << mag[i] << "," << phase[i] << "\n";
    }
    file.close();
    std::cout << "  Bode data saved to: " << filename << "\n";
}

//-----------------------------------------------------------------------------
// Transfer Function
//-----------------------------------------------------------------------------
class TransferFunction {
public:
    std::vector<double> num, den;
    
    TransferFunction() : num({1.0}), den({1.0}) {}
    TransferFunction(std::vector<double> n, std::vector<double> d) : num(n), den(d) {}
    
    Complex eval(double omega) const {
        Complex s(0.0, omega);
        Complex n_val(0.0), d_val(0.0);
        
        for (size_t i = 0; i < num.size(); i++)
            n_val += num[i] * std::pow(s, double(num.size() - 1 - i));
        for (size_t i = 0; i < den.size(); i++)
            d_val += den[i] * std::pow(s, double(den.size() - 1 - i));
        
        return n_val / d_val;
    }
    
    double magnitude(double omega) const { return std::abs(eval(omega)); }
    double phase_deg(double omega) const { return std::arg(eval(omega)) * 180.0 / PI; }
    
    // DC gain (s=0)
    double dcGain() const {
        return num.back() / den.back();
    }
    
    void print(const std::string& name = "G(s)") const {
        std::cout << name << " = (";
        for (size_t i = 0; i < num.size(); i++) {
            if (i > 0 && num[i] >= 0) std::cout << "+";
            std::cout << std::setprecision(3) << num[i];
            size_t power = num.size() - 1 - i;
            if (power > 0) std::cout << "s" << (power > 1 ? "^" + std::to_string(power) : "");
        }
        std::cout << ")/(";
        for (size_t i = 0; i < den.size(); i++) {
            if (i > 0 && den[i] >= 0) std::cout << "+";
            std::cout << den[i];
            size_t power = den.size() - 1 - i;
            if (power > 0) std::cout << "s" << (power > 1 ? "^" + std::to_string(power) : "");
        }
        std::cout << ")\n";
    }
};

//-----------------------------------------------------------------------------
// DC Motor Model
//-----------------------------------------------------------------------------
struct DCMotorParams {
    double R = 2.5;      // Armature resistance [Ohm]
    double L = 0.005;    // Armature inductance [H]
    double Km = 0.05;    // Motor torque constant [Nm/A]
    double Ke = 0.05;    // Back-EMF constant [V·s/rad]
    double J = 0.01;     // Total inertia [kg·m²]
    double B = 0.002;    // Viscous friction [Nm·s/rad]
    
    // Simplified first-order model: G(s) = K / (tau*s + 1)
    double getK() const { return Km / (B*R + Km*Ke); }
    double getTau() const { return J*R / (B*R + Km*Ke); }
    
    TransferFunction toTF() const {
        return TransferFunction({getK()}, {getTau(), 1.0});
    }
    
    void print() const {
        std::cout << "═══ DC Motor Parameters ═══\n";
        std::cout << "  R  (resistance)  = " << R << " Ω\n";
        std::cout << "  L  (inductance)  = " << L*1000 << " mH\n";
        std::cout << "  Km (torque const) = " << Km << " Nm/A\n";
        std::cout << "  Ke (back-EMF)    = " << Ke << " V·s/rad\n";
        std::cout << "  J  (inertia)     = " << J*1000 << " g·m²\n";
        std::cout << "  B  (friction)    = " << B << " Nm·s/rad\n";
        std::cout << "\n  Simplified Model: G(s) = " << getK() << " / (" 
                  << getTau() << "s + 1)\n";
    }
};

struct MotorUncertainty {
    double R_min = 1.5, R_max = 3.5;
    double J_min = 0.005, J_max = 0.015;
    double B_min = 0.001, B_max = 0.004;
    double Km_min = 0.045, Km_max = 0.055;
    
    void print() const {
        std::cout << "═══ Parameter Uncertainty ═══\n";
        std::cout << "  R  ∈ [" << R_min << ", " << R_max << "] Ω (±40%)\n";
        std::cout << "  J  ∈ [" << J_min*1000 << ", " << J_max*1000 << "] g·m² (±50%)\n";
        std::cout << "  B  ∈ [" << B_min << ", " << B_max << "] (±100%)\n";
        std::cout << "  Km ∈ [" << Km_min << ", " << Km_max << "] Nm/A (±10%)\n";
    }
};

//-----------------------------------------------------------------------------
// Sensitivity Functions
//-----------------------------------------------------------------------------
Complex sensitivity(const TransferFunction& G, const TransferFunction& K, double omega) {
    auto L = G.eval(omega) * K.eval(omega);
    return 1.0 / (1.0 + L);
}

Complex compSensitivity(const TransferFunction& G, const TransferFunction& K, double omega) {
    auto L = G.eval(omega) * K.eval(omega);
    return L / (1.0 + L);
}

Complex ctrlSensitivity(const TransferFunction& G, const TransferFunction& K, double omega) {
    auto L = G.eval(omega) * K.eval(omega);
    return K.eval(omega) / (1.0 + L);
}

//-----------------------------------------------------------------------------
// Weighting Functions
//-----------------------------------------------------------------------------
TransferFunction firstOrderWeight(double r0, double r_inf, double tau) {
    return TransferFunction({tau, r0}, {tau/r_inf, 1.0});
}

TransferFunction performanceWeight(double M, double omega_b, double A) {
    // W_p(s) = (s/M + omega_b) / (s + omega_b*A)
    return TransferFunction({1.0/M, omega_b}, {1.0, omega_b*A});
}

//=============================================================================
// CASE STUDY IMPLEMENTATIONS
//=============================================================================

void caseStudy1_NominalDesign() {
    std::cout << "\n";
    std::cout << "╔═══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  CASE STUDY 1: NOMINAL PI CONTROL DESIGN                          ║\n";
    std::cout << "║  Application: Wheel Motor Speed Control                           ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════════════╝\n\n";
    
    // 1. Motor model
    DCMotorParams motor;
    motor.print();
    
    auto G = motor.toTF();
    
    // 2. Design PI controller
    // Target: bandwidth 10 rad/s, phase margin 60°
    double wc = 10.0;  // Crossover frequency
    double pm = 60.0;  // Phase margin
    
    // Plant characteristics at crossover
    double tau = motor.getTau();
    double Kp_plant = motor.getK();
    double plant_gain_wc = Kp_plant / std::sqrt(1 + (wc*tau)*(wc*tau));
    double plant_phase_wc = -std::atan(wc*tau) * 180/PI;
    
    // PI: C(s) = Kp(1 + 1/(Ti*s))
    // Contribution to phase: atan(wc*Ti) - 90
    double phi_needed = -180 + pm - plant_phase_wc;
    double Ti = std::tan((phi_needed + 90)*PI/180) / wc;
    Ti = std::max(0.01, Ti);
    
    // Gain for |GC|=1 at wc
    double pi_gain_wc = std::sqrt(1 + 1/(wc*Ti*wc*Ti));
    double Kp = 1.0 / (plant_gain_wc * pi_gain_wc);
    double Ki = Kp / Ti;
    
    std::cout << "\n═══ PI Controller Design ═══\n";
    std::cout << "  Target crossover: " << wc << " rad/s\n";
    std::cout << "  Target phase margin: " << pm << "°\n";
    std::cout << "  Kp = " << Kp << "\n";
    std::cout << "  Ki = " << Ki << "\n";
    std::cout << "  Ti = " << Ti << " s\n";
    
    TransferFunction K({Kp, Ki}, {1.0, 0.0});
    K.print("K(s)");
    
    // 3. Frequency response analysis
    std::cout << "\n═══ Sensitivity Analysis ═══\n";
    std::cout << "  ω(rad/s)  |S|      |T|      |KS|     |L|\n";
    std::cout << "  --------  ------   ------   ------   ------\n";
    
    std::vector<double> freqs = {0.1, 1.0, 5.0, 10.0, 20.0, 50.0, 100.0};
    for (double w : freqs) {
        double S = std::abs(sensitivity(G, K, w));
        double T = std::abs(compSensitivity(G, K, w));
        double KS = std::abs(ctrlSensitivity(G, K, w));
        double L = G.magnitude(w) * K.magnitude(w);
        
        std::cout << "  " << std::setw(8) << w
                  << "  " << std::setw(6) << std::fixed << std::setprecision(4) << S
                  << "   " << std::setw(6) << T
                  << "   " << std::setw(6) << KS
                  << "   " << std::setw(6) << L << "\n";
    }
    
    // 4. Stability margins
    // Find actual crossover (|L|=1)
    double wc_actual = 0;
    for (double w = 0.1; w < 1000; w *= 1.01) {
        if (G.magnitude(w) * K.magnitude(w) < 1.0) {
            wc_actual = w;
            break;
        }
    }
    
    double pm_actual = 180 + (G.phase_deg(wc_actual) + K.phase_deg(wc_actual));
    
    std::cout << "\n═══ Stability Margins ═══\n";
    std::cout << "  Actual crossover: " << wc_actual << " rad/s\n";
    std::cout << "  Phase margin: " << pm_actual << "°\n";
    
    // 5. Time-domain simulation and plotting
    std::cout << "\n═══ Step Response Simulation ═══\n";
    
    double dt = 0.001;
    double tFinal = 2.0;
    int N = static_cast<int>(tFinal / dt);
    
    std::vector<double> t_data, y_data, u_data, ref_data;
    t_data.reserve(N/10);
    y_data.reserve(N/10);
    u_data.reserve(N/10);
    ref_data.reserve(N/10);
    
    double omega = 0.0;
    double integ_e = 0.0;
    double omega_ref = 10.0;
    
    for (int i = 0; i < N; i++) {
        double t = i * dt;
        double e = omega_ref - omega;
        integ_e += e * dt;
        double u = Kp * e + Ki * integ_e;
        u = std::max(-12.0, std::min(12.0, u));
        
        if (i % 10 == 0) {
            t_data.push_back(t);
            y_data.push_back(omega);
            u_data.push_back(u);
            ref_data.push_back(omega_ref);
        }
        
        double domega = -omega/tau + (Kp_plant/tau)*u;
        omega += domega * dt;
    }
    
    // ASCII Plot
    drawASCIIPlot(t_data, y_data, "STEP RESPONSE (Nominal PI)", "t (s)", "omega (rad/s)");
    drawASCIIPlot(t_data, u_data, "CONTROL SIGNAL", "t (s)", "u (V)");
    
    std::vector<std::vector<double>> curves = {y_data, ref_data};
    std::vector<char> markers = {'*', '-'};
    std::vector<std::string> labels = {"omega", "ref"};
    drawASCIIPlotMulti(t_data, curves, markers, labels, "TRACKING PERFORMANCE");
    
    // Save data
    saveStepResponseCSV(t_data, y_data, "case1_step_response.csv");
}

void caseStudy2_RobustStability() {
    std::cout << "\n";
    std::cout << "╔═══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  CASE STUDY 2: ROBUST STABILITY ANALYSIS                          ║\n";
    std::cout << "║  Multiplicative Uncertainty Model                                 ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════════════╝\n\n";
    
    DCMotorParams motor;
    MotorUncertainty unc;
    
    motor.print();
    std::cout << "\n";
    unc.print();
    
    auto G = motor.toTF();
    TransferFunction K({5.0, 20.0}, {1.0, 0.0});  // PI: Kp=5, Ki=20
    
    // Multiplicative uncertainty weight
    // |W_delta(jw)| >= |(G_pert - G_nom)/G_nom| for all G_pert
    double r0 = 0.25;     // 25% at DC
    double r_inf = 1.5;   // 150% at high freq
    double tau_w = 0.05;  // Crossover ~20 rad/s
    
    auto W_delta = firstOrderWeight(r0, r_inf, tau_w);
    
    std::cout << "\n═══ Uncertainty Weight ═══\n";
    W_delta.print("W_Δ(s)");
    std::cout << "  |W_Δ(0)|   = " << W_delta.magnitude(0.001) << "\n";
    std::cout << "  |W_Δ(∞)|   = " << W_delta.magnitude(10000) << "\n";
    
    // Robust stability: ||W_Δ · T||_∞ < 1
    std::cout << "\n═══ Robust Stability Test ═══\n";
    std::cout << "  Condition: ||W_Δ · T||_∞ < 1\n\n";
    std::cout << "  ω(rad/s)   |T|      |W_Δ|    |W_Δ·T|\n";
    std::cout << "  --------   ------   ------   --------\n";
    
    double max_WdT = 0;
    double omega_crit = 0;
    
    for (double w = 0.01; w <= 1000; w *= 1.1) {
        double T_mag = std::abs(compSensitivity(G, K, w));
        double Wd_mag = W_delta.magnitude(w);
        double WdT = T_mag * Wd_mag;
        
        if (WdT > max_WdT) {
            max_WdT = WdT;
            omega_crit = w;
        }
    }
    
    std::vector<double> freqs = {0.1, 1.0, 5.0, 10.0, 20.0, 50.0, 100.0};
    for (double w : freqs) {
        double T_mag = std::abs(compSensitivity(G, K, w));
        double Wd_mag = W_delta.magnitude(w);
        
        std::cout << "  " << std::setw(8) << w
                  << "   " << std::setw(6) << std::fixed << std::setprecision(4) << T_mag
                  << "   " << std::setw(6) << Wd_mag
                  << "   " << std::setw(8) << T_mag * Wd_mag << "\n";
    }
    
    std::cout << "\n═══ Result ═══\n";
    std::cout << "  ||W_delta . T||_inf = " << max_WdT << "\n";
    std::cout << "  Critical omega = " << omega_crit << " rad/s\n";
    
    if (max_WdT < 1.0) {
        std::cout << "\n  [OK] ROBUSTLY STABLE\n";
        std::cout << "  Stability margin = " << 1.0/max_WdT << "\n";
    } else {
        std::cout << "\n  [X] NOT ROBUSTLY STABLE\n";
        std::cout << "  Need to reduce controller bandwidth\n";
    }
    
    // Plot robust stability test
    std::vector<double> omega_plot, WdT_plot, T_plot, Wd_plot;
    for (double w = 0.1; w <= 100; w *= 1.1) {
        omega_plot.push_back(w);
        double T_m = std::abs(compSensitivity(G, K, w));
        double Wd_m = W_delta.magnitude(w);
        T_plot.push_back(T_m);
        Wd_plot.push_back(Wd_m);
        WdT_plot.push_back(T_m * Wd_m);
    }
    
    drawASCIIPlot(omega_plot, WdT_plot, "ROBUST STABILITY: |W_delta . T|", "omega (rad/s)", "magnitude");
}

void caseStudy3_RobustPerformance() {
    std::cout << "\n";
    std::cout << "╔═══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  CASE STUDY 3: ROBUST PERFORMANCE ANALYSIS                        ║\n";
    std::cout << "║  Combined Stability + Performance Requirement                     ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════════════╝\n\n";
    
    DCMotorParams motor;
    auto G = motor.toTF();
    TransferFunction K({5.0, 20.0}, {1.0, 0.0});
    
    // Uncertainty weight
    auto W_delta = firstOrderWeight(0.25, 1.5, 0.05);
    
    // Performance weight
    // Good tracking up to 5 rad/s, max SS error 5%
    auto W_perf = performanceWeight(2.0, 5.0, 0.05);
    
    std::cout << "═══ Weight Functions ═══\n";
    W_delta.print("W_Δ(s)");
    W_perf.print("W_p(s)");
    std::cout << "  |W_p(0)| = " << W_perf.magnitude(0.001) << " (1/A = 20)\n";
    std::cout << "  |W_p(∞)| = " << W_perf.magnitude(10000) << " (1/M = 0.5)\n";
    
    // RP condition: |W_p·S| + |W_Δ·T| < 1 for all ω
    std::cout << "\n═══ Robust Performance Test ═══\n";
    std::cout << "  Condition: ||W_p·S|| + ||W_Δ·T|| < 1 for all ω\n\n";
    std::cout << "  ω(rad/s)   |W_p·S|   |W_Δ·T|   Sum      Pass?\n";
    std::cout << "  --------   -------   -------   ------   -----\n";
    
    double max_sum = 0;
    double omega_crit = 0;
    
    for (double w = 0.01; w <= 1000; w *= 1.05) {
        double WpS = std::abs(W_perf.eval(w) * sensitivity(G, K, w));
        double WdT = std::abs(W_delta.eval(w) * compSensitivity(G, K, w));
        double sum = WpS + WdT;
        
        if (sum > max_sum) {
            max_sum = sum;
            omega_crit = w;
        }
    }
    
    std::vector<double> freqs = {0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0};
    for (double w : freqs) {
        double WpS = std::abs(W_perf.eval(w) * sensitivity(G, K, w));
        double WdT = std::abs(W_delta.eval(w) * compSensitivity(G, K, w));
        double sum = WpS + WdT;
        std::string pass = (sum < 1.0) ? "OK" : "FAIL";
        
        std::cout << "  " << std::setw(8) << w
                  << "   " << std::setw(7) << std::fixed << std::setprecision(4) << WpS
                  << "   " << std::setw(7) << WdT
                  << "   " << std::setw(6) << sum
                  << "   " << pass << "\n";
    }
    
    std::cout << "\n═══ Result ═══\n";
    std::cout << "  max{|W_p·S| + |W_Δ·T|} = " << max_sum << "\n";
    std::cout << "  Critical ω = " << omega_crit << " rad/s\n";
    
    if (max_sum < 1.0) {
        std::cout << "\n  ✓ ROBUST PERFORMANCE ACHIEVED\n";
        std::cout << "  RP margin = " << 1.0/max_sum << "\n";
    } else {
        std::cout << "\n  ✗ ROBUST PERFORMANCE NOT ACHIEVED\n";
        std::cout << "  Consider: reduce bandwidth, relax specs, or use H∞ design\n";
    }
}

void caseStudy4_HinfDesign() {
    std::cout << "\n";
    std::cout << "╔═══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  CASE STUDY 4: H∞ STATE-FEEDBACK DESIGN                           ║\n";
    std::cout << "║  Optimal Robust Controller                                        ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════════════╝\n\n";
    
    DCMotorParams motor;
    motor.print();
    
    double tau = motor.getTau();
    double Kp = motor.getK();
    
    std::cout << "\n═══ State-Space Model ═══\n";
    std::cout << "  State: x = [θ, ω]^T (position, velocity)\n";
    std::cout << "  A = [0, 1; 0, " << -1/tau << "]\n";
    std::cout << "  B = [0; " << Kp/tau << "]\n";
    std::cout << "  E = [0; " << 1/motor.J << "] (disturbance)\n";
    
    // H∞ design using pole placement approximation
    // Target: fast response with good damping
    double wn = 25.0;     // Natural frequency
    double zeta = 0.75;   // Damping ratio
    
    std::cout << "\n═══ Design Specifications ═══\n";
    std::cout << "  Natural frequency ωn = " << wn << " rad/s\n";
    std::cout << "  Damping ratio ζ = " << zeta << "\n";
    std::cout << "  Target settling time ≈ " << 4.0/(zeta*wn) << " s\n";
    
    // State feedback gain K = [k1, k2]
    // Characteristic: s² + (1/τ + k2·Kp/τ)s + k1·Kp/τ = s² + 2ζωn·s + ωn²
    double b = Kp / tau;
    double k2 = (2*zeta*wn - 1/tau) / b;
    double k1 = wn*wn / b;
    
    std::cout << "\n═══ H∞ State-Feedback Result ═══\n";
    std::cout << "  K = [" << std::setprecision(4) << k1 << ", " << k2 << "]\n";
    
    // Verify closed-loop
    double p_sum = 1/tau + b*k2;
    double p_prod = b*k1;
    double disc = p_sum*p_sum - 4*p_prod;
    
    std::cout << "\n  Closed-loop characteristic: s² + " << p_sum << "s + " << p_prod << "\n";
    
    if (disc < 0) {
        double re = -p_sum / 2;
        double im = std::sqrt(-disc) / 2;
        std::cout << "  Poles: " << re << " ± j" << im << "\n";
        std::cout << "  Actual ωn = " << std::sqrt(p_prod) << " rad/s\n";
        std::cout << "  Actual ζ = " << p_sum/(2*std::sqrt(p_prod)) << "\n";
    }
    
    // H∞ norm estimation
    std::cout << "\n═══ H∞ Norm Analysis ═══\n";
    std::cout << "  ||T_zw||_∞ represents worst-case amplification\n";
    std::cout << "  from disturbance w to output z\n";
    
    // Simplified: peak of closed-loop frequency response
    double peak = 0;
    for (double w = 0.1; w < 1000; w *= 1.1) {
        // T(jw) for second-order system
        double mag = wn*wn / std::sqrt(std::pow(wn*wn - w*w, 2) + std::pow(2*zeta*wn*w, 2));
        if (mag > peak) peak = mag;
    }
    
    std::cout << "  Estimated ||T||_∞ ≈ " << peak << "\n";
    std::cout << "  (Peak at ω ≈ " << wn*std::sqrt(1-2*zeta*zeta) << " rad/s)\n";
}

void caseStudy5_Kharitonov() {
    std::cout << "\n";
    std::cout << "╔═══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  CASE STUDY 5: KHARITONOV'S THEOREM                               ║\n";
    std::cout << "║  Interval Polynomial Stability                                    ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════════════╝\n\n";
    
    std::cout << "═══ Problem Setup ═══\n";
    std::cout << "  Closed-loop characteristic polynomial:\n";
    std::cout << "  p(s) = τs² + (1 + K·Kp)s + K·Ki\n\n";
    
    // Parameter ranges
    double K_min = 1.0, K_max = 5.0;   // Plant gain
    double tau_min = 0.05, tau_max = 0.15;  // Time constant
    double Kp = 3.0, Ki = 15.0;  // Controller
    
    std::cout << "  Plant uncertainty: K ∈ [" << K_min << ", " << K_max << "]\n";
    std::cout << "                    τ ∈ [" << tau_min << ", " << tau_max << "]\n";
    std::cout << "  Controller: Kp = " << Kp << ", Ki = " << Ki << "\n";
    
    // Coefficient bounds (for p(s) = a2*s² + a1*s + a0)
    double a2_min = tau_min, a2_max = tau_max;
    double a1_min = 1 + K_min*Kp, a1_max = 1 + K_max*Kp;
    double a0_min = K_min*Ki, a0_max = K_max*Ki;
    
    std::cout << "\n═══ Coefficient Bounds ═══\n";
    std::cout << "  a₂ ∈ [" << a2_min << ", " << a2_max << "]\n";
    std::cout << "  a₁ ∈ [" << a1_min << ", " << a1_max << "]\n";
    std::cout << "  a₀ ∈ [" << a0_min << ", " << a0_max << "]\n";
    
    // Four Kharitonov polynomials
    std::cout << "\n═══ Kharitonov Polynomials ═══\n";
    
    // K1: a2-, a1-, a0+
    // K2: a2+, a1+, a0-
    // K3: a2+, a1-, a0+
    // K4: a2-, a1+, a0-
    
    struct KPoly {
        double a2, a1, a0;
        std::string name;
    };
    
    std::vector<KPoly> kpolys = {
        {a2_min, a1_min, a0_max, "K₁"},
        {a2_max, a1_max, a0_min, "K₂"},
        {a2_max, a1_min, a0_max, "K₃"},
        {a2_min, a1_max, a0_min, "K₄"}
    };
    
    bool all_stable = true;
    
    for (const auto& kp : kpolys) {
        std::cout << "\n  " << kp.name << "(s) = " << kp.a2 << "s² + " 
                  << kp.a1 << "s + " << kp.a0 << "\n";
        
        // Check Hurwitz (2nd order: all coefs > 0)
        bool stable = (kp.a2 > 0 && kp.a1 > 0 && kp.a0 > 0);
        
        double disc = kp.a1*kp.a1 - 4*kp.a2*kp.a0;
        std::cout << "  Poles: ";
        if (disc < 0) {
            double re = -kp.a1 / (2*kp.a2);
            double im = std::sqrt(-disc) / (2*kp.a2);
            std::cout << re << " ± j" << im;
            stable = (re < 0);
        } else {
            double p1 = (-kp.a1 + std::sqrt(disc)) / (2*kp.a2);
            double p2 = (-kp.a1 - std::sqrt(disc)) / (2*kp.a2);
            std::cout << p1 << ", " << p2;
            stable = (p1 < 0 && p2 < 0);
        }
        std::cout << " → " << (stable ? "Stable ✓" : "Unstable ✗") << "\n";
        
        all_stable = all_stable && stable;
    }
    
    std::cout << "\n═══ Kharitonov Theorem Result ═══\n";
    if (all_stable) {
        std::cout << "  ✓ ALL four Kharitonov polynomials are stable\n";
        std::cout << "  → The ENTIRE interval polynomial family is ROBUSTLY STABLE\n";
    } else {
        std::cout << "  ✗ Some Kharitonov polynomials are unstable\n";
        std::cout << "  → There exist unstable plants in the uncertainty set\n";
    }
}

void caseStudy6_MuAnalysis() {
    std::cout << "\n";
    std::cout << "╔═══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  CASE STUDY 6: μ-ANALYSIS                                         ║\n";
    std::cout << "║  Structured Singular Value                                        ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════════════╝\n\n";
    
    DCMotorParams motor;
    auto G = motor.toTF();
    TransferFunction K({5.0, 20.0}, {1.0, 0.0});
    
    auto W_delta = firstOrderWeight(0.25, 1.5, 0.05);
    auto W_perf = performanceWeight(2.0, 5.0, 0.05);
    
    std::cout << "═══ M-Δ Framework ═══\n";
    std::cout << R"(
       ┌───────────────────┐
  w ───►                   ├───► z
       │        M          │
  u_Δ ─►                   ├───► y_Δ
       └───────────────────┘
                │
       ┌────────┴────────┐
       │  Δ = [Δ_p  0 ]  │
       │      [ 0  Δ_u]  │
       └────────┬────────┘
                │
    
  Robust Performance ⟺ μ_Δ(M(jω)) < 1 for all ω
)";
    
    std::cout << "\n═══ μ Frequency Sweep ═══\n";
    std::cout << "  ω(rad/s)   |W_p·S|   |W_Δ·T|   μ_ub\n";
    std::cout << "  --------   -------   -------   -----\n";
    
    double mu_peak = 0;
    double omega_peak = 0;
    
    std::vector<double> freqs = {0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0};
    
    for (double w : freqs) {
        double WpS = std::abs(W_perf.eval(w) * sensitivity(G, K, w));
        double WdT = std::abs(W_delta.eval(w) * compSensitivity(G, K, w));
        
        // Upper bound: μ ≤ |W_p·S| + |W_Δ·T| (conservative for diagonal Δ)
        double mu_ub = WpS + WdT;
        
        if (mu_ub > mu_peak) {
            mu_peak = mu_ub;
            omega_peak = w;
        }
        
        std::cout << "  " << std::setw(8) << w
                  << "   " << std::setw(7) << std::fixed << std::setprecision(4) << WpS
                  << "   " << std::setw(7) << WdT
                  << "   " << std::setw(5) << mu_ub << "\n";
    }
    
    std::cout << "\n═══ μ Analysis Summary ═══\n";
    std::cout << "  Peak μ (upper bound) = " << mu_peak << "\n";
    std::cout << "  At frequency ω = " << omega_peak << " rad/s\n";
    std::cout << "  RP Margin = " << 1.0/mu_peak << "\n";
    
    if (mu_peak < 1.0) {
        std::cout << "\n  ✓ μ < 1: ROBUST PERFORMANCE ACHIEVED\n";
        std::cout << "  System maintains performance for all plants in uncertainty set\n";
    } else {
        std::cout << "\n  ✗ μ ≥ 1: ROBUST PERFORMANCE NOT ACHIEVED\n";
        std::cout << "  Consider D-K iteration for improved design\n";
    }
    
    std::cout << "\n═══ D-K Iteration Note ═══\n";
    std::cout << "  For tighter μ bound and optimal controller:\n";
    std::cout << "  1. K-step: Design K minimizing ||D·M·D⁻¹||_∞\n";
    std::cout << "  2. D-step: Optimize D(jω) at each frequency\n";
    std::cout << "  3. Fit rational D(s) and repeat\n";
}

void runAllCaseStudies() {
    caseStudy1_NominalDesign();
    std::cout << "\nPress Enter to continue...";
    std::cin.get();
    
    caseStudy2_RobustStability();
    std::cout << "\nPress Enter to continue...";
    std::cin.get();
    
    caseStudy3_RobustPerformance();
    std::cout << "\nPress Enter to continue...";
    std::cin.get();
    
    caseStudy4_HinfDesign();
    std::cout << "\nPress Enter to continue...";
    std::cin.get();
    
    caseStudy5_Kharitonov();
    std::cout << "\nPress Enter to continue...";
    std::cin.get();
    
    caseStudy6_MuAnalysis();
}

} // namespace robust_control

//=============================================================================
// MAIN
//=============================================================================

int main() {
    std::cout << R"(
╔══════════════════════════════════════════════════════════════════════════════╗
║                                                                              ║
║   ██████╗  ██████╗ ██████╗ ██╗   ██╗███████╗████████╗                        ║
║   ██╔══██╗██╔═══██╗██╔══██╗██║   ██║██╔════╝╚══██╔══╝                        ║
║   ██████╔╝██║   ██║██████╔╝██║   ██║███████╗   ██║                           ║
║   ██╔══██╗██║   ██║██╔══██╗██║   ██║╚════██║   ██║                           ║
║   ██║  ██║╚██████╔╝██████╔╝╚██████╔╝███████║   ██║                           ║
║   ╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝ ╚══════╝   ╚═╝                           ║
║                                                                              ║
║   CONTROL CASE STUDY - DIFFERENTIAL DRIVE MOBILE ROBOT                       ║
║   Motor Speed Control with H∞, Uncertainty, and μ-Analysis                   ║
║                                                                              ║
╚══════════════════════════════════════════════════════════════════════════════╝
)";

    while (true) {
        std::cout << "\n┌─────────────────────────────────────────────────────────────────────┐\n";
        std::cout << "│                          SELECT CASE STUDY                          │\n";
        std::cout << "├─────────────────────────────────────────────────────────────────────┤\n";
        std::cout << "│  1. Nominal PI Control Design                                       │\n";
        std::cout << "│  2. Robust Stability Analysis (Multiplicative Uncertainty)          │\n";
        std::cout << "│  3. Robust Performance Analysis                                     │\n";
        std::cout << "│  4. H∞ State-Feedback Design                                        │\n";
        std::cout << "│  5. Kharitonov's Theorem (Interval Polynomials)                     │\n";
        std::cout << "│  6. μ-Analysis (Structured Singular Value)                          │\n";
        std::cout << "├─────────────────────────────────────────────────────────────────────┤\n";
        std::cout << "│  0. Run ALL Case Studies                                            │\n";
        std::cout << "│  q. Quit                                                            │\n";
        std::cout << "└─────────────────────────────────────────────────────────────────────┘\n";
        std::cout << "\nEnter choice: ";
        
        std::string input;
        std::getline(std::cin, input);
        
        if (input == "q" || input == "Q") break;
        
        int choice = 0;
        try {
            choice = std::stoi(input);
        } catch (...) {
            std::cout << "Invalid input. Please enter a number.\n";
            continue;
        }
        
        switch (choice) {
            case 0: robust_control::runAllCaseStudies(); break;
            case 1: robust_control::caseStudy1_NominalDesign(); break;
            case 2: robust_control::caseStudy2_RobustStability(); break;
            case 3: robust_control::caseStudy3_RobustPerformance(); break;
            case 4: robust_control::caseStudy4_HinfDesign(); break;
            case 5: robust_control::caseStudy5_Kharitonov(); break;
            case 6: robust_control::caseStudy6_MuAnalysis(); break;
            default:
                std::cout << "Invalid choice. Please select 0-6 or q.\n";
        }
    }
    
    std::cout << "\n════════════════════════════════════════════════════════════════════════════════\n";
    std::cout << "                      Thank you for using Robust Control Case Study!            \n";
    std::cout << "════════════════════════════════════════════════════════════════════════════════\n";
    
    return 0;
}
