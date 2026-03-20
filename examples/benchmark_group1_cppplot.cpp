/**
 * =============================================================================
 * CPPPLOT BENCHMARK — NHÓM 1: Polynomial & Transfer Function
 * Runner C++ — Chạy các test case và ghi kết quả ra CSV
 * =============================================================================
 *
 * Build (từ thư mục benchmark_group1/cpp/):
 *
 *   Linux / macOS:
* g++ -std=c++17 -O2 -I../cppplot/include benchmark_group1_cppplot.cpp -o bench_group1

 *     g++ -std=c++14 -O2 -I../../../cppplot/include \
 *         benchmark_group1_cppplot.cpp -o bench_group1
 *     ./bench_group1
 *
 *   Windows (MinGW):
 *     g++ -std=c++14 -O2 -I..\..\..\cppplot\include ^
 *         benchmark_group1_cppplot.cpp -o bench_group1.exe
 *     bench_group1.exe
 *
 * Output: ../results/cppplot_results.csv
 * =============================================================================
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <complex>
#include <iomanip>

using namespace cppplot;
using namespace cppplot::control;

// ─────────────────────────────────────────────────────────────────────────────
// Result recording
// ─────────────────────────────────────────────────────────────────────────────

struct BenchResult {
    std::string group;
    std::string case_id;
    std::string subcase;
    std::string quantity;
    double      expected;
    double      cppplot_val;
    double      tolerance;
    std::string unit;
    std::string note;
};

std::vector<BenchResult> results;

void record(const std::string& group,
            const std::string& case_id,
            const std::string& subcase,
            const std::string& quantity,
            double expected,
            double cppplot_val,
            double tolerance,
            const std::string& unit,
            const std::string& note = "")
{
    BenchResult r{group, case_id, subcase, quantity,
                  expected, cppplot_val, tolerance, unit, note};
    results.push_back(r);

    double err = std::abs(cppplot_val - expected);
    std::string status = (err <= tolerance) ? "PASS" : "FAIL";

    std::cout << "  [" << status << "] "
              << case_id << "/" << std::left << std::setw(20) << subcase
              << " | " << std::setw(30) << quantity
              << " | expected=" << std::setw(12) << expected
              << "  got=" << std::setw(12) << cppplot_val
              << "  err=" << std::scientific << std::setprecision(2) << err
              << "  tol=" << tolerance
              << "  [" << unit << "]\n"
              << std::defaultfloat;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: sort complex roots by real part then imag
// ─────────────────────────────────────────────────────────────────────────────
using Cplx = std::complex<double>;

std::vector<double> sortedRealRoots(std::vector<Cplx> roots) {
    std::vector<double> real_roots;
    for (auto& r : roots) real_roots.push_back(r.real());
    std::sort(real_roots.begin(), real_roots.end());
    return real_roots;
}

std::vector<Cplx> sortedComplexRoots(std::vector<Cplx> roots) {
    std::sort(roots.begin(), roots.end(),
              [](const Cplx& a, const Cplx& b){ return a.imag() < b.imag(); });
    return roots;
}

// ─────────────────────────────────────────────────────────────────────────────
// CASE 1.1 — ROOT FINDING
// ─────────────────────────────────────────────────────────────────────────────
void run_1_1() {
    std::cout << "\n" << std::string(70,'=') << "\n";
    std::cout << "CASE 1.1 — ROOT FINDING\n";
    std::cout << std::string(70,'=') << "\n";

    // ── 1.1.A  Real roots: p(s) = s³ + 6s² + 11s + 6 ─────────────────────
    std::cout << "\n[1.1.A] Real roots — p(s) = s^3 + 6s^2 + 11s + 6\n";
    {
        Polynomial p({1, 6, 11, 6});
        auto roots = sortedRealRoots(p.roots());
        double expected[] = {-3.0, -2.0, -1.0};
        for (int i = 0; i < 3; ++i)
            record("1.1", "1.1.A", "root_" + std::to_string(i+1),
                   "root_" + std::to_string(i+1),
                   expected[i], roots[i], 1e-10, "—", "analytical");
    }

    // ── 1.1.B  Complex roots: p(s) = s² + 2s + 5 ─────────────────────────
    std::cout << "\n[1.1.B] Complex roots — p(s) = s^2 + 2s + 5\n";
    {
        Polynomial p({1, 2, 5});
        auto roots = sortedComplexRoots(p.roots());
        // expected: -1-2j, -1+2j
        double exp_real[] = {-1.0, -1.0};
        double exp_imag[] = {-2.0,  2.0};
        for (int i = 0; i < 2; ++i) {
            record("1.1", "1.1.B", "root_" + std::to_string(i+1) + "_real",
                   "root_" + std::to_string(i+1) + " real",
                   exp_real[i], roots[i].real(), 1e-10, "—");
            record("1.1", "1.1.B", "root_" + std::to_string(i+1) + "_imag",
                   "root_" + std::to_string(i+1) + " imag",
                   exp_imag[i], roots[i].imag(), 1e-10, "—");
        }
    }

    // ── 1.1.C  Repeated roots: p(s) = (s+2)³ ─────────────────────────────
    std::cout << "\n[1.1.C] Repeated roots — p(s) = (s+2)^3\n";
    {
        Polynomial p({1, 6, 12, 8});
        auto roots = sortedRealRoots(p.roots());
        for (int i = 0; i < 3; ++i)
            record("1.1", "1.1.C", "root_" + std::to_string(i+1),
                   "root_" + std::to_string(i+1) + " (triple)",
                   -2.0, roots[i], 1e-4, "—",
                   "Repeated root — numerically ill-conditioned, tol=1e-4");
    }

    // ── 1.1.D  High-degree: roots at -1 to -8 ─────────────────────────────
    std::cout << "\n[1.1.D] High-degree — roots at -1 to -8 (degree 8)\n";
    {
        // Coefficients of (s+1)(s+2)...(s+8)
        // Pre-computed: poly([-1,-2,-3,-4,-5,-6,-7,-8])
        Polynomial p({1, 36, 546, 4536, 22449, 67284, 118124, 109584, 40320});
        auto roots = sortedRealRoots(p.roots());
        double expected[] = {-8,-7,-6,-5,-4,-3,-2,-1};
        for (int i = 0; i < 8; ++i)
            record("1.1", "1.1.D", "root_" + std::to_string(i+1),
                   "root_" + std::to_string(i+1),
                   expected[i], roots[i], 1e-5, "—", "High-degree: tol=1e-5");
    }

    // ── 1.1.E  Wilkinson-like: roots at -1 to -10 ─────────────────────────
    std::cout << "\n[1.1.E] Wilkinson-like — roots at -1 to -10 (degree 10)\n";
    {
        // Coefficients of (s+1)(s+2)...(s+10)
        Polynomial p({1, 55, 1320, 18150, 157773, 902055, 3416930,
                      8409500, 12753576, 10628640, 3628800});
        auto roots = sortedRealRoots(p.roots());
        double expected[] = {-10,-9,-8,-7,-6,-5,-4,-3,-2,-1};
        for (int i = 0; i < 10; ++i)
            record("1.1", "1.1.E", "root_" + std::to_string(i+1),
                   "root_" + std::to_string(i+1),
                   expected[i], roots[i], 1e-3, "—", "Wilkinson-like: tol=1e-3");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// CASE 1.2 — FREQUENCY RESPONSE
// ─────────────────────────────────────────────────────────────────────────────
void run_1_2() {
    std::cout << "\n" << std::string(70,'=') << "\n";
    std::cout << "CASE 1.2 — FREQUENCY RESPONSE\n";
    std::cout << std::string(70,'=') << "\n";

    // ── 1.2.A  2nd-order: |G(jωn)| = 1/(2ζ) ─────────────────────────────
    std::cout << "\n[1.2.A] 2nd-order: |G(jwn)| = 1/(2*zeta) [analytical]\n";
    {
        struct TC { double wn, zeta; };
        TC cases[] = {{1,0.1},{1,0.3},{1,0.5},{5,0.3},{10,0.5},{10,0.707}};
        for (auto& c : cases) {
            TransferFunction G({c.wn*c.wn}, {1, 2*c.zeta*c.wn, c.wn*c.wn});
            double mag_dB_got    = G.mag_dB(c.wn);
            double phase_deg_got = G.phase_deg(c.wn);
            double mag_dB_exp    = -20.0 * std::log10(2.0 * c.zeta);

            std::string sub = "wn=" + std::to_string(c.wn)
                            + "_z=" + std::to_string(c.zeta);
            record("1.2", "1.2.A", sub, "mag_dB_at_wn",
                   mag_dB_exp, mag_dB_got, 0.01, "dB", "analytical: 1/(2z)");
            record("1.2", "1.2.A", sub, "phase_deg_at_wn",
                   -90.0, phase_deg_got, 0.1, "deg", "analytical: -90 at wn");
        }
    }

    // ── 1.2.B  DC gain via frequency response (ω → 0) ─────────────────────
    std::cout << "\n[1.2.B] DC gain via freq response (omega=1e-6)\n";
    {
        struct TF_case { std::string name; TransferFunction G; double dc_exp; };
        std::vector<TF_case> cases = {
            {"first_order_K5",  TransferFunction({5},    {2, 1}),       5.0        },
            {"2nd_order_std",   TransferFunction({25},   {1, 4, 25}),   1.0        },
            {"3rd_order",       TransferFunction({100},  {1, 6, 11, 6}),100.0/6.0  },
            {"near_integrator", TransferFunction({10, 5},{1, 0.001}),   5.0/0.001  },
        };
        for (auto& c : cases) {
            double mag_got = c.G.mag(1e-6);   // linear magnitude
            record("1.2", "1.2.B", c.name, "DC_gain_linear",
                   c.dc_exp, mag_got, std::abs(c.dc_exp) * 0.001, "—",
                   "omega=1e-6 approx DC");
        }
    }

    // ── 1.2.C  Phase of integrator/differentiator ─────────────────────────
    std::cout << "\n[1.2.C] Phase of integrator/differentiator\n";
    {
        TransferFunction G_int({1},    {1, 0});   // 1/s
        TransferFunction G_diff({1, 0},{1});       // s
        double omegas[] = {0.01, 0.1, 1.0, 10.0, 100.0};
        for (double w : omegas) {
            record("1.2", "1.2.C",
                   "integrator_w=" + std::to_string(w), "phase_deg",
                   -90.0, G_int.phase_deg(w),  0.01, "deg");
            record("1.2", "1.2.C",
                   "differentiator_w=" + std::to_string(w), "phase_deg",
                    90.0, G_diff.phase_deg(w), 0.01, "deg");
        }
    }

    // ── 1.2.D  Series: |G1*G2|_dB = |G1|_dB + |G2|_dB ───────────────────
    std::cout << "\n[1.2.D] Series: |G1*G2|_dB = |G1|_dB + |G2|_dB\n";
    {
        TransferFunction G1({10}, {1, 2});
        TransferFunction G2({1},  {1, 5});
        TransferFunction G = G1 * G2;
        double omegas[] = {0.1, 1.0, 5.0, 20.0};
        for (double w : omegas) {
            double dB1 = G1.mag_dB(w);
            double dB2 = G2.mag_dB(w);
            double dBg = G.mag_dB(w);
            record("1.2", "1.2.D", "w=" + std::to_string(w), "mag_dB_series",
                   dB1 + dB2, dBg, 1e-8, "dB", "should be exact");
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// CASE 1.3 — DC GAIN & SPECIAL CASES
// ─────────────────────────────────────────────────────────────────────────────
void run_1_3() {
    std::cout << "\n" << std::string(70,'=') << "\n";
    std::cout << "CASE 1.3 — DC GAIN & SPECIAL CASES\n";
    std::cout << std::string(70,'=') << "\n";

    // ── 1.3.A  Type-0 DC gain ──────────────────────────────────────────────
    std::cout << "\n[1.3.A] Type-0 DC gain\n";
    {
        struct TC { std::string name; TransferFunction G; double exp; };
        std::vector<TC> cases = {
            {"K5_tau2",   TransferFunction({5},    {2,1}),      5.0       },
            {"2nd_std",   TransferFunction({25},   {1,4,25}),   1.0       },
            {"3rd_K100",  TransferFunction({100},  {1,6,11,6}), 100.0/6.0 },
            {"gain_K7",   TransferFunction({7},    {1}),        7.0       },
            {"zpk_K3",    TransferFunction({3, 6}, {1,3,2}),    3.0       },
        };
        for (auto& c : cases)
            record("1.3", "1.3.A", c.name, "dcgain",
                   c.exp, c.G.dcgain(), 1e-10, "—");
    }

    // ── 1.3.B  Type-1: DC gain = ∞ ───────────────────────────────────────
    std::cout << "\n[1.3.B] Type-1 system: DC gain = inf\n";
    {
        TransferFunction G({1}, {1, 1, 0});
        double dc = G.dcgain();
        double is_inf = (std::isinf(dc) || std::abs(dc) > 1e12) ? 1.0 : 0.0;
        std::cout << "  [INFO] Type-1 dcgain = " << dc
                  << "  -> is_inf = " << is_inf << "\n";
        record("1.3", "1.3.B", "type1", "dcgain_is_inf",
               1.0, is_inf, 0.5, "bool", "1=True 0=False");
    }

    // ── 1.3.C  Stability ──────────────────────────────────────────────────
    std::cout << "\n[1.3.C] Stability: all poles in LHP\n";
    {
        struct TC { std::string name; TransferFunction G; double exp_stable; };
        std::vector<TC> cases = {
            {"stable_2nd",   TransferFunction({1},{1,2,1}),    1.0},
            {"unstable_rHP", TransferFunction({1},{1,-1}),     0.0},
            {"marginal",     TransferFunction({1},{1,0,1}),    0.0},
            {"stable_3rd",   TransferFunction({1},{1,6,11,6}), 1.0},
        };
        for (auto& c : cases)
            record("1.3", "1.3.C", c.name, "is_stable",
                   c.exp_stable, c.G.isStable() ? 1.0 : 0.0, 0.5, "bool");
    }

    // ── 1.3.D  Proper / strictly proper ──────────────────────────────────
    std::cout << "\n[1.3.D] Proper / strictly proper\n";
    {
        struct TC {
            std::string name;
            TransferFunction G;
            double exp_proper, exp_strict;
        };
        std::vector<TC> cases = {
            {"strictly_proper", TransferFunction({1},   {1,2,1}), 1, 1},
            {"proper_biproper", TransferFunction({1,0}, {1,1}),   1, 0},
            {"improper",        TransferFunction({1,0}, {1}),     0, 0},
        };
        for (auto& c : cases) {
            record("1.3", "1.3.D", c.name, "is_proper",
                   c.exp_proper, c.G.isProper() ? 1.0 : 0.0, 0.5, "bool");
            record("1.3", "1.3.D", c.name, "is_strictly_proper",
                   c.exp_strict, c.G.isStrictlyProper() ? 1.0 : 0.0, 0.5, "bool");
        }
    }

    // ── 1.3.E  Poles & zeros consistency ─────────────────────────────────
    std::cout << "\n[1.3.E] Poles & zeros: G = 5(s+1)/((s+2)(s+3))\n";
    {
        TransferFunction G({5, 5}, {1, 5, 6});
        auto z = G.zeros();
        auto p = G.poles();

        // Sort by real part
        std::sort(z.begin(), z.end(), [](Cplx a, Cplx b){ return a.real() < b.real(); });
        std::sort(p.begin(), p.end(), [](Cplx a, Cplx b){ return a.real() < b.real(); });

        record("1.3","1.3.E","zero_1","zero",  -1.0, z[0].real(), 1e-10, "—");
        record("1.3","1.3.E","pole_1","pole",  -3.0, p[0].real(), 1e-10, "—");
        record("1.3","1.3.E","pole_2","pole",  -2.0, p[1].real(), 1e-10, "—");

        // Leading coefficient ratio
        double num_lead = G.num.leading();
        double den_lead = G.den.leading();
        record("1.3","1.3.E","leading_coeff_ratio","num/den leading",
               5.0, num_lead / den_lead, 1e-10, "—");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// MAIN
// ─────────────────────────────────────────────────────────────────────────────
int main() {
    std::cout << std::string(70,'=') << "\n";
    std::cout << "CPPPLOT BENCHMARK — NHÓM 1: Polynomial & Transfer Function\n";
    std::cout << std::string(70,'=') << "\n";

    run_1_1();
    run_1_2();
    run_1_3();

    // ── Summary ─────────────────────────────────────────────────────────────
    std::cout << "\n" << std::string(70,'=') << "\n";
    std::cout << "SUMMARY\n" << std::string(70,'=') << "\n";

    int n_pass = 0, n_fail = 0;
    std::vector<BenchResult*> failed;
    for (auto& r : results) {
        double err = std::abs(r.cppplot_val - r.expected);
        if (err <= r.tolerance) ++n_pass;
        else { ++n_fail; failed.push_back(&r); }
    }
    std::cout << "\n  Total : " << results.size()
              << "\n  PASS  : " << n_pass
              << "\n  FAIL  : " << n_fail << "\n";
    if (!failed.empty()) {
        std::cout << "\n  Failed cases:\n";
        for (auto* r : failed) {
            double err = std::abs(r->cppplot_val - r->expected);
            std::cout << "    x " << r->case_id << "/" << r->subcase
                      << " | " << r->quantity
                      << " | expected=" << r->expected
                      << " got=" << r->cppplot_val
                      << " err=" << std::scientific << std::setprecision(2) << err
                      << "\n" << std::defaultfloat;
        }
    }

    // ── Export CSV ───────────────────────────────────────────────────────────
    std::string csv_path = "../cppplot_results.csv";
    std::ofstream f(csv_path);
    if (!f.is_open()) {
        std::cerr << "ERROR: cannot open " << csv_path << "\n";
        return 1;
    }
    f << "group,case_id,subcase,quantity,expected_analytical,"
      << "cppplot_value,tolerance,unit,note\n";
    f << std::setprecision(15);
    for (auto& r : results) {
        f << r.group     << ","
          << r.case_id   << ","
          << r.subcase   << ","
          << r.quantity  << ","
          << r.expected  << ","
          << r.cppplot_val << ","
          << r.tolerance << ","
          << r.unit      << ","
          << r.note      << "\n";
    }
    f.close();
    std::cout << "\n  Results saved: " << csv_path << "\n\n";
    return 0;
}
