/**
 * test_control_systems.cpp
 * Control systems plots using confirmed API:
 *   Figure fig(w,h); auto& ax = fig.gca();
 *   ax.plot(), ax.scatter(), ax.set_title()...
 * Numerically validates Bode, step response, SMC.
 */
#include <cppplot/cppplot.hpp>
#include <iostream>
#include <cassert>
#include <complex>
#include <cmath>
#include <vector>
#include <fstream>
#include <stdexcept>
#include <algorithm>

using namespace cppplot;

int tests_passed = 0, tests_failed = 0;

#define RUN_TEST(name) do { \
    std::cout << "Running " #name "... "; \
    try { test_##name(); std::cout << "PASSED\n"; tests_passed++; } \
    catch (const std::exception& e) { \
        std::cout << "FAILED: " << e.what() << "\n"; tests_failed++; } \
} while(0)

#define ASSERT_TRUE(x)     do { if(!(x))  throw std::runtime_error("TRUE: "  #x); } while(0)
#define ASSERT_NEAR(a,b,t) do { if(std::abs((double)(a)-(double)(b))>(t)) \
    throw std::runtime_error(std::string("NEAR: ") + #a + " = " + std::to_string((double)(a)) \
                             + ", expected " + std::to_string((double)(b))); } while(0)
#define ASSERT_CONTAINS(s,sub) do { \
    if((s).find(sub)==std::string::npos) \
        throw std::runtime_error(std::string("missing: '") + (sub) + "'"); \
} while(0)

// ── Hệ thống bậc 2: G(s) = wn²/(s²+2ζwns+wn²) ──
struct SecondOrder {
    double wn, zeta;

    std::complex<double> eval(double omega) const {
        std::complex<double> s(0, omega);
        return (wn*wn) / (s*s + 2*zeta*wn*s + wn*wn);
    }
    double mag_dB(double omega) const {
        return 20.0 * std::log10(std::abs(eval(omega)));
    }
    double phase_deg(double omega) const {
        return std::arg(eval(omega)) * 180.0 / M_PI;
    }
    double step(double t) const {
        double sigma = zeta*wn;
        double wd = wn*std::sqrt(1-zeta*zeta);
        return 1 - std::exp(-sigma*t)*(std::cos(wd*t) + sigma/wd*std::sin(wd*t));
    }
};

// ═══════════════════════════════════════════
// Bode Plot — numerical + visual
// ═══════════════════════════════════════════

void test_bode_dc_gain() {
    SecondOrder sys{10.0, 0.707};
    // DC gain (ω→0) phải ≈ 0 dB
    ASSERT_NEAR(sys.mag_dB(0.001), 0.0, 1.0);
}

void test_bode_bandwidth() {
    SecondOrder sys{10.0, 0.707};
    // ω=ωn, ζ=0.707 → -3 dB bandwidth
    ASSERT_NEAR(sys.mag_dB(sys.wn), -3.01, 0.15);
}

void test_bode_phase_at_wn() {
    SecondOrder sys{10.0, 0.3};
    // Phase tại ω=ωn luôn = -90° với mọi ζ
    ASSERT_NEAR(sys.phase_deg(sys.wn), -90.0, 2.0);
}

void test_bode_high_freq_phase() {
    SecondOrder sys{10.0, 0.5};
    // Phase ω>>ωn → -180°
    ASSERT_NEAR(sys.phase_deg(100000.0), -180.0, 3.0);
}

void test_bode_plot_renders() {
    SecondOrder sys{10.0, 0.3};
    auto w = logspace(-1, 3, 200);
    std::vector<double> mag_db, phase_d;
    for (double wi : w) {
        mag_db.push_back(sys.mag_dB(wi));
        phase_d.push_back(sys.phase_deg(wi));
    }

    Figure fig(800, 600);

    auto& ax1 = fig.subplot(2, 1, 1);
    ax1.plot(w, mag_db, "b-", {{"label", std::string("Magnitude")}});
    ax1.set_ylabel("Magnitude (dB)");
    ax1.set_title("Bode Plot - wn=10, zeta=0.3");
    ax1.grid(true);

    auto& ax2 = fig.subplot(2, 1, 2);
    ax2.plot(w, phase_d, "b-");
    ax2.set_xlabel("Frequency (rad/s)");
    ax2.set_ylabel("Phase (deg)");
    ax2.grid(true);

    std::string svg = fig.toSVG();
    ASSERT_CONTAINS(svg, "Bode Plot");
    ASSERT_CONTAINS(svg, "<polyline");
    fig.savefig("test_bode.svg");
    std::remove("test_bode.svg");
}

// ═══════════════════════════════════════════
// Step Response — numerical correctness
// ═══════════════════════════════════════════

void test_step_initial_value() {
    SecondOrder sys{10.0, 0.3};
    ASSERT_NEAR(sys.step(0.0), 0.0, 1e-10);
}

void test_step_final_value() {
    SecondOrder sys{10.0, 0.3};
    ASSERT_NEAR(sys.step(10.0), 1.0, 0.01);
}

void test_step_overshoot() {
    SecondOrder sys{10.0, 0.3};
    auto t = linspace(0.0, 3.0, 500);
    std::vector<double> y;
    for (double ti : t) y.push_back(sys.step(ti));

    double max_y = *std::max_element(y.begin(), y.end());
    ASSERT_TRUE(max_y > 1.0); // underdamped phải overshoot

    // Lý thuyết: OS = exp(-π·ζ/√(1-ζ²))
    double ζ = sys.zeta;
    double OS_theory = std::exp(-M_PI*ζ / std::sqrt(1-ζ*ζ));
    double OS_meas   = max_y - 1.0;
    ASSERT_NEAR(OS_meas, OS_theory, 0.05);
}

void test_step_overdamped_no_overshoot() {
    SecondOrder sys{10.0, 1.5}; // overdamped
    auto t = linspace(0.0, 5.0, 500);
    double max_y = 0;
    for (double ti : t) max_y = std::max(max_y, sys.step(ti));
    ASSERT_TRUE(max_y <= 1.01); // không overshoot
}

void test_step_response_plot() {
    SecondOrder sys{10.0, 0.3};
    auto t = linspace(0.0, 2.0, 300);
    std::vector<double> y;
    for (double ti : t) y.push_back(sys.step(ti));

    Figure fig(700, 400);
    auto& ax = fig.gca();
    ax.plot(t, y, "b-", {{"label", std::string("y(t)")}});
    ax.set_xlabel("Time (s)");
    ax.set_ylabel("y(t)");
    ax.set_title("Step Response - wn=10, zeta=0.3");
    ax.set_xlim(0, 2);
    ax.set_ylim(0, 1.6);
    ax.grid(true);
    ax.legend(true);

    std::string svg = fig.toSVG();
    ASSERT_CONTAINS(svg, "Step Response");
    fig.savefig("test_step.svg");
    std::remove("test_step.svg");
}

// ═══════════════════════════════════════════
// Nyquist Plot
// ═══════════════════════════════════════════

void test_nyquist_high_freq_magnitude() {
    // G(s)=1/(s(s+1)(s+2)) — mag→0 khi ω→∞
    auto w = logspace(-2, 3, 300);
    std::vector<double> re, im;
    for (double wi : w) {
        std::complex<double> s(0, wi);
        auto H = 1.0 / (s*(s+1.0)*(s+2.0));
        re.push_back(H.real());
        im.push_back(H.imag());
    }
    double mag_high = std::sqrt(re.back()*re.back() + im.back()*im.back());
    ASSERT_TRUE(mag_high < 0.01);
}

void test_nyquist_renders() {
    auto w = logspace(-2, 2, 200);
    std::vector<double> re, im;
    for (double wi : w) {
        std::complex<double> s(0, wi);
        auto H = 1.0 / (s*(s+1.0)*(s+2.0));
        re.push_back(H.real());
        im.push_back(H.imag());
    }

    Figure fig(600, 600);
    auto& ax = fig.gca();
    ax.plot(re, im, "b-", {{"label", std::string("G(jw)")}});
    ax.scatter({-1.0}, {0.0}, {{"c", std::string("red")}});
    ax.set_xlabel("Real");
    ax.set_ylabel("Imaginary");
    ax.set_title("Nyquist Plot");
    ax.grid(true);

    std::string svg = fig.toSVG();
    ASSERT_CONTAINS(svg, "Nyquist Plot");
}

// ═══════════════════════════════════════════
// Pole-Zero Map
// ═══════════════════════════════════════════

void test_pzmap_poles_in_lhp() {
    SecondOrder sys{5.0, 0.5};
    double sigma = sys.zeta * sys.wn;
    // Poles ở -sigma ± j*wd → Real part âm = stable
    ASSERT_TRUE(-sigma < 0.0);
}

void test_pzmap_renders() {
    SecondOrder sys{5.0, 0.5};
    double sigma = sys.zeta * sys.wn;
    double wd    = sys.wn * std::sqrt(1 - sys.zeta*sys.zeta);

    Figure fig(600, 500);
    auto& ax = fig.gca();
    ax.scatter({-sigma, -sigma}, {wd, -wd},
               {{"c", std::string("red")}});
    ax.scatter({-1.0}, {0.0},
               {{"c", std::string("blue")}});
    ax.set_xlabel("Real Axis");
    ax.set_ylabel("Imaginary Axis");
    ax.set_title("Pole-Zero Map");
    ax.grid(true);

    ASSERT_CONTAINS(fig.toSVG(), "Pole-Zero Map");
}

// ═══════════════════════════════════════════
// SMC Simulation + visualization
// ═══════════════════════════════════════════

void test_smc_convergence() {
    // Super-Twisting SMC: x1'=x2, x2'=u+d
    const double k1=1.5, k2=1.1, lambda=2.0, dt=1e-4;
    double x1=0.5, x2=0.0, v=0.0;

    for (int i = 0; i < 30000; i++) {
        double s = x2 + lambda*x1;
        double u = -k1*std::copysign(std::sqrt(std::abs(s)),s) + v;
        v  += -k2*std::copysign(1.0,s)*dt;
        double d = 0.1*std::sin(5.0*i*dt);
        x1 += x2*dt;
        x2 += (u+d)*dt;
    }
    // Phải hội tụ về 0
    ASSERT_NEAR(x1, 0.0, 0.05);
}

void test_smc_plot_renders() {
    const double k1=1.5, k2=1.1, lambda=2.0, dt=1e-4;
    double x1=0.5, x2=0.0, v=0.0;
    std::vector<double> t_vec, x1_vec, s_vec;

    for (int i = 0; i < 20000; i++) {
        double s = x2 + lambda*x1;
        double u = -k1*std::copysign(std::sqrt(std::abs(s)),s) + v;
        v  += -k2*std::copysign(1.0,s)*dt;
        x1 += x2*dt;
        x2 += (u + 0.1*std::sin(5.0*i*dt))*dt;
        if (i%10==0) {
            t_vec.push_back(i*dt);
            x1_vec.push_back(x1);
            s_vec.push_back(s);
        }
    }

    Figure fig(900, 600);

    auto& ax1 = fig.subplot(2, 1, 1);
    ax1.plot(t_vec, x1_vec, "b-", {{"label", std::string("x1")}});
    ax1.set_ylabel("State x1");
    ax1.set_title("Super-Twisting SMC");
    ax1.grid(true);
    ax1.legend(true);

    auto& ax2 = fig.subplot(2, 1, 2);
    ax2.plot(t_vec, s_vec, "r-", {{"label", std::string("s")}});
    ax2.set_xlabel("Time (s)");
    ax2.set_ylabel("Sliding var s");
    ax2.grid(true);
    ax2.legend(true);

    std::string svg = fig.toSVG();
    ASSERT_CONTAINS(svg, "Super-Twisting SMC");
    fig.savefig("test_smc.svg");
    std::remove("test_smc.svg");
}

// ═══════════════════════════════════════════
// Kalman Filter
// ═══════════════════════════════════════════

void test_kalman_tracks_state() {
    const double Q=0.01, R=1.0, dt=0.01;
    double x_true=5.0, x_hat=0.0, P=1.0;
    auto noise=[](int i,double a){return a*std::sin(i*1.234)*std::cos(i*0.567);};

    for (int i = 0; i < 300; i++) {
        double x_pred = 0.995*x_hat;
        double P_pred = 0.995*0.995*P + Q;
        double z = x_true + noise(i, std::sqrt(R));
        double K = P_pred/(P_pred+R);
        x_hat = x_pred + K*(z-x_pred);
        P = (1-K)*P_pred;
        x_true = 0.995*x_true + noise(i+100, 0.01);
    }
    ASSERT_NEAR(x_hat, x_true, 2.0);
}

// ═══════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════

int main() {
    std::cout << "CppPlot Control Systems Tests\n";
    std::cout << "==============================\n\n";

    RUN_TEST(bode_dc_gain);
    RUN_TEST(bode_bandwidth);
    RUN_TEST(bode_phase_at_wn);
    RUN_TEST(bode_high_freq_phase);
    RUN_TEST(bode_plot_renders);

    RUN_TEST(step_initial_value);
    RUN_TEST(step_final_value);
    RUN_TEST(step_overshoot);
    RUN_TEST(step_overdamped_no_overshoot);
    RUN_TEST(step_response_plot);

    RUN_TEST(nyquist_high_freq_magnitude);
    RUN_TEST(nyquist_renders);

    RUN_TEST(pzmap_poles_in_lhp);
    RUN_TEST(pzmap_renders);

    RUN_TEST(smc_convergence);
    RUN_TEST(smc_plot_renders);

    RUN_TEST(kalman_tracks_state);

    std::cout << "\n==============================\n";
    std::cout << "Passed: " << tests_passed << "\n";
    std::cout << "Failed: " << tests_failed << "\n";
    return tests_failed > 0 ? 1 : 0;
}
