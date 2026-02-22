/**
 * test_control_systems.cpp
 * ─────────────────────────────────────────────
 * Kiểm tra các plot chuyên dụng cho control systems.
 * Đặc biệt: kiểm tra NUMERICAL CORRECTNESS của
 * dữ liệu đầu vào, chứng minh library tương thích
 * với tính toán control systems chuẩn.
 */
#include <cppplot/cppplot.hpp>
#include <cassert>
#include <fstream>
#include <iostream>
#include <complex>
#include <cmath>
#include <vector>
#include <string>
#include <limits>

using namespace cppplot;

static bool file_exists(const std::string& p) {
    return std::ifstream(p).good();
}

// Độ chính xác số học
static const double EPS = 1e-9;

// ─────────────────────────────────────────────
// Utility: tính frequency response của hệ thống
// G(s) = wn² / (s² + 2ζωₙs + ωₙ²)
// ─────────────────────────────────────────────
struct SecondOrderSystem {
    double wn;   // natural frequency (rad/s)
    double zeta; // damping ratio

    std::complex<double> eval(double omega) const {
        std::complex<double> s(0.0, omega);
        return (wn * wn) / (s * s + 2.0 * zeta * wn * s + wn * wn);
    }

    double magnitude_dB(double omega) const {
        return 20.0 * std::log10(std::abs(eval(omega)));
    }

    double phase_deg(double omega) const {
        return std::arg(eval(omega)) * 180.0 / M_PI;
    }

    // Step response y(t) — analytical formula
    double step_response(double t) const {
        if (zeta >= 1.0) {
            // Overdamped / critically damped (simplified)
            double r = wn * std::sqrt(zeta * zeta - 1.0);
            double s1 = -zeta * wn + r;
            double s2 = -zeta * wn - r;
            double A = s2 / (s2 - s1);
            double B = -s1 / (s2 - s1);
            return 1.0 - A * std::exp(s1 * t) - B * std::exp(s2 * t);
        } else {
            // Underdamped
            double sigma = zeta * wn;
            double wd    = wn * std::sqrt(1.0 - zeta * zeta);
            return 1.0 - std::exp(-sigma * t) *
                         (std::cos(wd * t) + (sigma / wd) * std::sin(wd * t));
        }
    }
};

// ─────────────────────────────────────────────
// TEST 1: Bode plot — kiểm tra numerical correctness
// ─────────────────────────────────────────────
void test_bode_numerical_correctness() {
    std::cout << "[TEST] test_bode_numerical_correctness ... ";

    SecondOrderSystem sys{10.0, 0.707}; // wn=10, zeta=0.707 (critically damped)

    auto w = logspace(-1, 3, 300); // 0.1 to 1000 rad/s

    std::vector<double> mag_dB, phase_deg_vec;
    for (double wi : w) {
        mag_dB.push_back(sys.magnitude_dB(wi));
        phase_deg_vec.push_back(sys.phase_deg(wi));
    }

    // ── Numerical checks ──────────────────────
    // 1. Tại ω << ωₙ (DC): magnitude ≈ 0 dB
    double dc_gain_dB = sys.magnitude_dB(0.001);
    assert(std::abs(dc_gain_dB) < 1.0 &&
           "DC gain must be ~0 dB for unity DC gain system");

    // 2. Tại ω = ωₙ với zeta=0.707: magnitude ≈ -3 dB (định nghĩa bandwidth)
    double gain_at_wn = sys.magnitude_dB(sys.wn);
    assert(std::abs(gain_at_wn - (-3.01)) < 0.1 &&
           "Gain at wn with zeta=0.707 must be ~-3 dB");

    // 3. Tại ω >> ωₙ: phase → -180°
    double phase_high = sys.phase_deg(10000.0);
    assert(std::abs(phase_high - (-180.0)) < 5.0 &&
           "High-frequency phase must approach -180 deg");

    // 4. Tại ω = ωₙ: phase = -90° (đúng cho mọi zeta)
    double phase_at_wn = sys.phase_deg(sys.wn);
    assert(std::abs(phase_at_wn - (-90.0)) < 2.0 &&
           "Phase at wn must be ~-90 deg");

    // ── Vẽ Bode plot ──────────────────────────
    figure(800, 600);
    subplot(2, 1, 1);
    plot(w, mag_dB, "b-", opts({{"linewidth", "2"}}));
    xscale("log");
    axhline(-3.0, opts({{"color", "gray"}, {"linestyle", "--"}}));
    ylabel("Magnitude (dB)");
    grid(true);
    title("Bode Plot - 2nd Order (wn=10, zeta=0.707)");

    subplot(2, 1, 2);
    plot(w, phase_deg_vec, "b-", opts({{"linewidth", "2"}}));
    xscale("log");
    axhline(-90.0, opts({{"color", "gray"}, {"linestyle", "--"}}));
    xlabel("Frequency (rad/s)");
    ylabel("Phase (deg)");
    grid(true);

    savefig("test_bode.svg");
    assert(file_exists("test_bode.svg"));

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 2: Step response — kiểm tra correctness
// ─────────────────────────────────────────────
void test_step_response_correctness() {
    std::cout << "[TEST] test_step_response_correctness ... ";

    SecondOrderSystem sys{10.0, 0.3}; // underdamped

    auto t = linspace(0.0, 3.0, 500);
    std::vector<double> y;
    for (double ti : t) y.push_back(sys.step_response(ti));

    // ── Numerical checks ──────────────────────
    // 1. y(0) = 0 (starts at zero)
    assert(std::abs(y[0]) < EPS &&
           "Step response must start at y(0) = 0");

    // 2. y(∞) → 1.0 (steady state = 1 for unity DC gain)
    double y_final = y.back();
    assert(std::abs(y_final - 1.0) < 0.01 &&
           "Step response must converge to 1.0 for t → ∞");

    // 3. Underdamped system (zeta=0.3) phải có overshoot
    double max_val = *std::max_element(y.begin(), y.end());
    assert(max_val > 1.0 &&
           "Underdamped system (zeta=0.3) must exhibit overshoot");

    // 4. Overshoot phải đúng với formula: OS = exp(-pi*zeta/sqrt(1-zeta²))
    double zeta = sys.zeta;
    double theoretical_OS = std::exp(-M_PI * zeta /
                                     std::sqrt(1.0 - zeta * zeta));
    double measured_OS = max_val - 1.0;
    assert(std::abs(measured_OS - theoretical_OS) < 0.05 &&
           "Overshoot must match theoretical formula");

    // ── Vẽ step response ──────────────────────
    figure(800, 450);
    plot(t, y, "b-", opts({{"linewidth", "2"}, {"label", "y(t)"}}));
    axhline(1.0,          opts({{"color","gray"}, {"linestyle","--"}}));
    axhline(1.0 + theoretical_OS,
                          opts({{"color","red"},  {"linestyle",":"}}));
    text(2.5, 1.0 + theoretical_OS + 0.02, "OS",
         opts({{"color","red"}, {"fontsize","10"}}));
    xlabel("Time (s)");
    ylabel("y(t)");
    legend(true);
    grid(true);
    title("Step Response - wn=10, zeta=0.3");
    savefig("test_step_response.svg");
    assert(file_exists("test_step_response.svg"));

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 3: Nyquist plot
// ─────────────────────────────────────────────
void test_nyquist_plot() {
    std::cout << "[TEST] test_nyquist_plot ... ";

    // G(s) = 1 / (s(s+1)(s+2)) — open-loop unstable
    auto w = logspace(-2, 2, 400);
    std::vector<double> re, im;

    for (double wi : w) {
        std::complex<double> s(0.0, wi);
        std::complex<double> H = 1.0 / (s * (s + 1.0) * (s + 2.0));
        re.push_back(H.real());
        im.push_back(H.imag());
    }

    // Kiểm tra: tại ω→0, imaginary part → -∞ (pure integrator)
    // tại ω→∞, magnitude → 0
    double mag_high = std::sqrt(re.back() * re.back() + im.back() * im.back());
    assert(mag_high < 0.01 &&
           "High-frequency magnitude must approach 0 for strictly proper system");

    figure(700, 700);
    plot(re, im, "b-", opts({{"linewidth", "2"}, {"label", "omega > 0"}}));
    scatter({-1.0}, {0.0}, opts({{"s","50"}, {"color","red"}, {"marker","+"}}));
    text(-0.85, 0.05, "(-1, j0)", opts({{"color","red"}, {"fontsize","10"}}));
    axhline(0.0, opts({{"color","black"}, {"linewidth","0.5"}}));
    axvline(0.0, opts({{"color","black"}, {"linewidth","0.5"}}));
    xlabel("Real"); ylabel("Imaginary");
    title("Nyquist Plot - G(s) = 1/(s(s+1)(s+2))");
    grid(true); legend(true);
    savefig("test_nyquist.svg");
    assert(file_exists("test_nyquist.svg"));

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 4: Pole-Zero map
// ─────────────────────────────────────────────
void test_pole_zero_map() {
    std::cout << "[TEST] test_pole_zero_map ... ";

    SecondOrderSystem sys{5.0, 0.5};

    // Poles: -zeta*wn ± j*wd
    double sigma = sys.zeta * sys.wn;
    double wd    = sys.wn * std::sqrt(1.0 - sys.zeta * sys.zeta);

    std::vector<double> poles_re = {-sigma, -sigma};
    std::vector<double> poles_im = { wd,    -wd};

    // Zero tại s = -1
    std::vector<double> zeros_re = {-1.0};
    std::vector<double> zeros_im = { 0.0};

    // Verifikasi: poles ada di LHP (real part negatif)
    for (double pr : poles_re) {
        assert(pr < 0.0 && "All poles must be in LHP for stable system");
    }

    figure(700, 700);
    scatter(poles_re, poles_im,
            opts({{"s","80"}, {"color","red"}, {"marker","x"}}));
    scatter(zeros_re, zeros_im,
            opts({{"s","80"}, {"color","blue"}, {"marker","o"}}));
    axhline(0.0, opts({{"color","black"}, {"linewidth","0.5"}}));
    axvline(0.0, opts({{"color","black"}, {"linewidth","0.5"}}));
    xlabel("Real Axis"); ylabel("Imaginary Axis");
    title("Pole-Zero Map");
    grid(true);
    savefig("test_pzmap.svg");
    assert(file_exists("test_pzmap.svg"));

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 5: SMC simulation + visualization
// Sliding Mode Control — Super-Twisting
// ─────────────────────────────────────────────
void test_smc_simulation() {
    std::cout << "[TEST] test_smc_simulation ... ";

    // Plant: x1' = x2, x2' = u + d(t)
    // Super-Twisting SMC gains
    const double k1 = 1.5, k2 = 1.1, lambda = 2.0;
    const double dt = 1e-4;
    const int N = 30000; // 3 seconds

    double x1 = 0.5, x2 = 0.0, v = 0.0;
    std::vector<double> t_vec, x1_vec, s_vec, u_vec;

    for (int i = 0; i < N; i++) {
        double t = i * dt;
        double s = x2 + lambda * x1; // sliding variable

        // Super-Twisting control law
        double u = -k1 * std::copysign(std::pow(std::abs(s), 0.5), s) + v;
        v += -k2 * std::copysign(1.0, s) * dt;

        // Simple disturbance
        double d = 0.1 * std::sin(5.0 * t);

        // Euler integration
        x1 += x2 * dt;
        x2 += (u + d) * dt;

        if (i % 10 == 0) { // Lưu mỗi 10 bước để giảm memory
            t_vec.push_back(t);
            x1_vec.push_back(x1);
            s_vec.push_back(s);
            u_vec.push_back(u);
        }
    }

    // ── Numerical checks ──────────────────────
    // 1. Hệ thống phải hội tụ về 0
    double final_x1 = std::abs(x1_vec.back());
    assert(final_x1 < 0.05 &&
           "Super-Twisting SMC must stabilize x1 to near 0");

    // 2. Sliding variable phải về gần 0
    double final_s = std::abs(s_vec.back());
    assert(final_s < 0.1 &&
           "Sliding variable must converge to sliding surface (s≈0)");

    // ── Vẽ kết quả ───────────────────────────
    figure(900, 700);

    subplot(3, 1, 1);
    plot(t_vec, x1_vec, "b-", opts({{"linewidth","1.5"},{"label","x1"}}));
    axhline(0.0, opts({{"color","gray"},{"linestyle","--"}}));
    ylabel("State x1"); grid(true); legend(true);
    title("Super-Twisting SMC - Disturbance Rejection");

    subplot(3, 1, 2);
    plot(t_vec, s_vec, "r-", opts({{"linewidth","1.5"},{"label","s"}}));
    axhline(0.0, opts({{"color","gray"},{"linestyle","--"}}));
    ylabel("Sliding var s"); grid(true); legend(true);

    subplot(3, 1, 3);
    plot(t_vec, u_vec, "k-", opts({{"linewidth","1.0"},{"label","u"}}));
    xlabel("Time (s)"); ylabel("Control u");
    grid(true); legend(true);

    savefig("test_smc_supertwisting.svg");
    assert(file_exists("test_smc_supertwisting.svg"));

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// TEST 6: Kalman Filter simulation
// ─────────────────────────────────────────────
void test_kalman_filter() {
    std::cout << "[TEST] test_kalman_filter ... ";

    // Hệ thống: x' = -0.5x + u, y = x + noise
    // Discrete, dt = 0.01s
    const double dt = 0.01;
    const int N = 300;
    const double Q = 0.01; // process noise
    const double R = 1.0;  // measurement noise

    // State
    double x_true = 5.0;
    double x_hat  = 0.0; // initial estimate
    double P = 1.0;      // error covariance

    // Pseudo-random noise (deterministic để test reproducible)
    auto noise = [](int i, double amp) {
        return amp * std::sin(i * 1.234) * std::cos(i * 0.567);
    };

    std::vector<double> t_vec, x_true_vec, x_hat_vec, meas_vec;

    for (int i = 0; i < N; i++) {
        double t = i * dt;

        // Kalman predict
        double x_hat_pred = 0.995 * x_hat; // A = 0.995
        double P_pred = 0.995 * 0.995 * P + Q;

        // Measurement
        double z = x_true + noise(i, std::sqrt(R));

        // Kalman update
        double K = P_pred / (P_pred + R);
        x_hat = x_hat_pred + K * (z - x_hat_pred);
        P = (1.0 - K) * P_pred;

        // True state evolution
        x_true = 0.995 * x_true + noise(i + 100, 0.01);

        t_vec.push_back(t);
        x_true_vec.push_back(x_true);
        x_hat_vec.push_back(x_hat);
        meas_vec.push_back(z);
    }

    // ── Check: estimate phải gần true state ──
    double final_error = std::abs(x_hat_vec.back() - x_true_vec.back());
    assert(final_error < 2.0 &&
           "Kalman filter estimate must track true state");

    // ── Vẽ ───────────────────────────────────
    figure(800, 450);
    scatter(t_vec, meas_vec,
            opts({{"color","lightgray"}, {"s","5"}, {"label","Measurements"}}));
    plot(t_vec, x_true_vec, "b-",
         opts({{"linewidth","2"}, {"label","True state"}}));
    plot(t_vec, x_hat_vec,  "r-",
         opts({{"linewidth","2"}, {"label","KF estimate"}}));
    xlabel("Time (s)"); ylabel("x");
    legend(true); grid(true);
    title("Kalman Filter State Estimation");
    savefig("test_kalman.svg");
    assert(file_exists("test_kalman.svg"));

    std::cout << "PASS\n";
}

// ─────────────────────────────────────────────
// MAIN
// ─────────────────────────────────────────────
int main() {
    std::cout << "=== test_control_systems ===\n";

    try {
        test_bode_numerical_correctness();
        test_step_response_correctness();
        test_nyquist_plot();
        test_pole_zero_map();
        test_smc_simulation();
        test_kalman_filter();
    } catch (const std::exception& e) {
        std::cerr << "EXCEPTION: " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "UNKNOWN EXCEPTION\n";
        return 1;
    }

    std::cout << "=== ALL CONTROL SYSTEMS TESTS PASSED ===\n";
    return 0;
}
