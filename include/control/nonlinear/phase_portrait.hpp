/**
 * @file nonlinear/phase_portrait.hpp
 * @brief Phase Portrait Analysis for 2D Nonlinear Systems
 *
 * General-purpose tools for visualizing and analyzing 2D autonomous systems:
 *   ẋ₁ = f₁(x₁, x₂)
 *   ẋ₂ = f₂(x₁, x₂)
 *
 * Features:
 *   - Vector field (quiver) plots
 *   - Trajectory integration from multiple initial conditions
 *   - Nullcline computation and plotting
 *   - Equilibrium point finding (Newton-Raphson)
 *   - Equilibrium classification via Jacobian eigenvalues
 *   - Limit cycle detection (Poincaré section method)
 *
 * References:
 *   - Strogatz (2015) "Nonlinear Dynamics and Chaos"
 *   - Khalil (2002) "Nonlinear Systems", Ch. 2
 *   - Jordan & Smith (2007) "Nonlinear Ordinary Differential Equations"
 *
 * Usage:
 * @code
 * #include <cppplot/control/nonlinear/phase_portrait.hpp>
 * using namespace cppplot::control::nonlinear;
 *
 * // Van der Pol oscillator: ẋ₁ = x₂, ẋ₂ = μ(1-x₁²)x₂ - x₁
 * auto vdp = [](Vec x) -> Vec {
 *     return {x[1], 1.0*(1-x[0]*x[0])*x[1] - x[0]};
 * };
 *
 * PhasePortrait pp({-3,3}, {-3,3}, 20, 20);
 * pp.quiver(vdp);
 * pp.trajectory(vdp, {2.0, 0.0}, 20.0);
 * pp.nullclines(vdp);
 * auto eqs = pp.equilibria(vdp);
 * pp.show();
 * @endcode
 */

#ifndef CPPPLOT_CONTROL_NONLINEAR_PHASE_PORTRAIT_HPP
#define CPPPLOT_CONTROL_NONLINEAR_PHASE_PORTRAIT_HPP

#include "../../pyplot.hpp"
#include <vector>
#include <functional>
#include <cmath>
#include <string>
#include <algorithm>
#include <stdexcept>
#include <sstream>

namespace cppplot {
namespace control {
namespace nonlinear {

/// Shorthand for a 2D state vector
using Vec = std::vector<double>;
/// 2D vector field: maps state → state derivative
using VectorField2D = std::function<Vec(Vec)>;

// ============================================================
//                   UTILITY FUNCTIONS
// ============================================================

/**
 * @brief RK4 integration step for 2D system
 */
inline Vec rk4_step(const VectorField2D& f, const Vec& x, double dt) {
    Vec k1 = f(x);
    Vec xk2 = {x[0] + 0.5*dt*k1[0], x[1] + 0.5*dt*k1[1]};
    Vec k2 = f(xk2);
    Vec xk3 = {x[0] + 0.5*dt*k2[0], x[1] + 0.5*dt*k2[1]};
    Vec k3 = f(xk3);
    Vec xk4 = {x[0] + dt*k3[0], x[1] + dt*k3[1]};
    Vec k4 = f(xk4);
    return {
        x[0] + dt/6.0*(k1[0]+2*k2[0]+2*k3[0]+k4[0]),
        x[1] + dt/6.0*(k1[1]+2*k2[1]+2*k3[1]+k4[1])
    };
}

/**
 * @brief Numeric Jacobian of 2D vector field at point x
 * @return 2x2 Jacobian as flat array [J00, J01, J10, J11]
 */
inline std::vector<double> numeric_jacobian_2d(const VectorField2D& f, const Vec& x,
                                               double h = 1e-5) {
    Vec fx = f(x);
    Vec xp0 = {x[0]+h, x[1]}, xm0 = {x[0]-h, x[1]};
    Vec xp1 = {x[0], x[1]+h}, xm1 = {x[0], x[1]-h};
    Vec fp0 = f(xp0), fm0 = f(xm0);
    Vec fp1 = f(xp1), fm1 = f(xm1);
    return {
        (fp0[0]-fm0[0])/(2*h), (fp1[0]-fm1[0])/(2*h),  // row 0
        (fp0[1]-fm0[1])/(2*h), (fp1[1]-fm1[1])/(2*h)   // row 1
    };
}

// ============================================================
//              EQUILIBRIUM CLASSIFICATION
// ============================================================

/**
 * @brief Classification of 2D equilibrium point
 */
enum class EquilibriumType {
    STABLE_NODE,
    UNSTABLE_NODE,
    STABLE_SPIRAL,
    UNSTABLE_SPIRAL,
    CENTER,
    SADDLE,
    UNKNOWN
};

/**
 * @brief Result of equilibrium analysis
 */
struct Equilibrium {
    double x1, x2;              ///< Location
    EquilibriumType type;
    double eig1_re, eig1_im;    ///< Eigenvalue 1 (real + imag)
    double eig2_re, eig2_im;    ///< Eigenvalue 2 (real + imag)
    double trace_J, det_J;      ///< Trace and determinant of Jacobian

    std::string typeString() const {
        switch (type) {
            case EquilibriumType::STABLE_NODE:    return "Stable Node";
            case EquilibriumType::UNSTABLE_NODE:  return "Unstable Node";
            case EquilibriumType::STABLE_SPIRAL:  return "Stable Spiral";
            case EquilibriumType::UNSTABLE_SPIRAL:return "Unstable Spiral";
            case EquilibriumType::CENTER:         return "Center";
            case EquilibriumType::SADDLE:         return "Saddle";
            default:                              return "Unknown";
        }
    }

    bool isStable() const {
        return type == EquilibriumType::STABLE_NODE ||
               type == EquilibriumType::STABLE_SPIRAL ||
               type == EquilibriumType::CENTER;
    }
};

/**
 * @brief Classify equilibrium from 2x2 Jacobian (flat: [J00,J01,J10,J11])
 */
inline Equilibrium classify_equilibrium(double x1, double x2,
                                        const std::vector<double>& J) {
    Equilibrium eq;
    eq.x1 = x1; eq.x2 = x2;

    double tr = J[0] + J[3];          // trace = J00 + J11
    double det = J[0]*J[3] - J[1]*J[2]; // det = J00*J11 - J01*J10
    double disc = tr*tr - 4*det;

    eq.trace_J = tr;
    eq.det_J = det;

    if (det < 0) {
        // Saddle: one positive, one negative real eigenvalue
        eq.type = EquilibriumType::SADDLE;
        double sq = std::sqrt(-disc);  // disc > 0 here since det < 0
        eq.eig1_re = (tr + std::sqrt(tr*tr - 4*det)) / 2;
        eq.eig1_im = 0;
        eq.eig2_re = (tr - std::sqrt(tr*tr - 4*det)) / 2;
        eq.eig2_im = 0;
    } else if (disc >= 0) {
        // Real eigenvalues: nodes
        double sq = std::sqrt(disc);
        eq.eig1_re = (tr + sq) / 2; eq.eig1_im = 0;
        eq.eig2_re = (tr - sq) / 2; eq.eig2_im = 0;
        if (tr < -1e-9)       eq.type = EquilibriumType::STABLE_NODE;
        else if (tr > 1e-9)   eq.type = EquilibriumType::UNSTABLE_NODE;
        else                  eq.type = EquilibriumType::UNKNOWN;
    } else {
        // Complex eigenvalues: spirals or center
        double sq = std::sqrt(-disc);
        eq.eig1_re = tr/2; eq.eig1_im =  sq/2;
        eq.eig2_re = tr/2; eq.eig2_im = -sq/2;
        if (std::abs(tr) < 1e-9) eq.type = EquilibriumType::CENTER;
        else if (tr < 0)         eq.type = EquilibriumType::STABLE_SPIRAL;
        else                     eq.type = EquilibriumType::UNSTABLE_SPIRAL;
    }

    return eq;
}

// ============================================================
//                  PHASE PORTRAIT CLASS
// ============================================================

/**
 * @brief Phase portrait analysis and visualization for 2D nonlinear systems
 *
 * Provides a clean API to build up a phase portrait incrementally:
 * quiver() → trajectory() → nullclines() → equilibria() → show()
 */
class PhasePortrait {
public:
    double x1_min, x1_max;  ///< x₁ axis range
    double x2_min, x2_max;  ///< x₂ axis range
    int nx, ny;              ///< Grid resolution for quiver

    std::string x1_label = "x₁";
    std::string x2_label = "x₂";
    std::string title_str = "Phase Portrait";

    /**
     * @param x1_range  Pair {min, max} for x₁ axis
     * @param x2_range  Pair {min, max} for x₂ axis
     * @param nx_       Grid points in x₁ direction for quiver
     * @param ny_       Grid points in x₂ direction for quiver
     */
    PhasePortrait(std::pair<double,double> x1_range,
                  std::pair<double,double> x2_range,
                  int nx_ = 20, int ny_ = 20)
        : x1_min(x1_range.first), x1_max(x1_range.second),
          x2_min(x2_range.first), x2_max(x2_range.second),
          nx(nx_), ny(ny_) {}

    // ---- Labels & Title ----
    PhasePortrait& xlabel(const std::string& s) { x1_label = s; return *this; }
    PhasePortrait& ylabel(const std::string& s) { x2_label = s; return *this; }
    PhasePortrait& title(const std::string& s)  { title_str = s; return *this; }

    // ============================================================
    //                    VECTOR FIELD (QUIVER)
    // ============================================================

    /**
     * @brief Plot normalized vector field arrows
     *
     * Arrows are normalized to uniform length so direction is visible
     * regardless of magnitude. Magnitude is encoded in color (optional).
     *
     * @param f       Vector field f(x) → [ẋ₁, ẋ₂]
     * @param normalize  If true, normalize arrow lengths (recommended)
     */
    void quiver(const VectorField2D& f, bool normalize = true,
                const std::string& color = "#888888", double scale = 0.4) {
        double dx = (x1_max - x1_min) / (nx - 1);
        double dy = (x2_max - x2_min) / (ny - 1);

        double arrow_scale = scale * std::min(dx, dy);

        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < ny; ++j) {
                double x1 = x1_min + i * dx;
                double x2 = x2_min + j * dy;
                Vec fv = f({x1, x2});
                double mag = std::sqrt(fv[0]*fv[0] + fv[1]*fv[1]);
                if (mag < 1e-12) continue;

                double ux = fv[0], uy = fv[1];
                if (normalize && mag > 0) { ux /= mag; uy /= mag; }

                double x1e = x1 + arrow_scale * ux;
                double x2e = x2 + arrow_scale * uy;

                // Draw arrow line
                cppplot::plot({x1, x1e}, {x2, x2e},
                              color + "-",
                              {{"linewidth","0.7"}, {"alpha","0.6"}});

                // Arrowhead (small marker at tip)
                cppplot::plot({x1e}, {x2e}, color + ".",
                              {{"markersize","2"}});
            }
        }
    }

    // ============================================================
    //                    TRAJECTORY INTEGRATION
    // ============================================================

    /**
     * @brief Integrate and plot a single trajectory from initial condition
     *
     * Uses 4th-order Runge-Kutta. Trajectory plotted in phase space (x₁, x₂).
     *
     * @param f        Vector field
     * @param x0       Initial condition {x₁₀, x₂₀}
     * @param T        Total integration time
     * @param dt       Time step (default 0.01)
     * @param color    Line color (e.g. "b-", "r-")
     * @param mark_ic  If true, marks initial condition with a dot
     * @return         Pair of trajectory vectors {x1_traj, x2_traj}
     */
    std::pair<std::vector<double>, std::vector<double>>
    trajectory(const VectorField2D& f, Vec x0, double T,
               double dt = 0.01,
               const std::string& line_spec = "b-",
               bool mark_ic = true) {
        int N = static_cast<int>(T / dt) + 1;
        std::vector<double> x1v(N), x2v(N);
        Vec x = x0;

        for (int i = 0; i < N; ++i) {
            // Clamp to display range for plotting
            x1v[i] = x[0];
            x2v[i] = x[1];
            x = rk4_step(f, x, dt);

            // Stop if trajectory diverges significantly beyond range
            double margin = 5.0;
            if (std::abs(x[0]) > margin*(x1_max-x1_min) ||
                std::abs(x[1]) > margin*(x2_max-x2_min)) break;
        }

        cppplot::plot(x1v, x2v, line_spec, {{"linewidth","1.5"}});
        if (mark_ic) {
            cppplot::plot({x0[0]}, {x0[1]}, "ko",
                          {{"markersize","5"}, {"label","IC"}});
        }

        return {x1v, x2v};
    }

    /**
     * @brief Integrate multiple trajectories from a grid of initial conditions
     *
     * @param f      Vector field
     * @param ics    List of initial conditions
     * @param T      Integration time per trajectory
     * @param dt     Time step
     */
    void trajectories(const VectorField2D& f,
                      const std::vector<Vec>& ics,
                      double T, double dt = 0.01) {
        // Cycle through a set of colors
        std::vector<std::string> colors = {"b-","r-","g-","m-","c-","y-","k-"};
        for (size_t i = 0; i < ics.size(); ++i) {
            trajectory(f, ics[i], T, dt, colors[i % colors.size()], true);
        }
    }

    /**
     * @brief Auto-generate initial conditions on a regular grid and integrate
     *
     * @param f        Vector field
     * @param T        Integration time
     * @param nx_ic    Grid points in x₁ for ICs
     * @param ny_ic    Grid points in x₂ for ICs
     */
    void trajectory_grid(const VectorField2D& f, double T,
                         int nx_ic = 5, int ny_ic = 5,
                         double dt = 0.01) {
        double dx = (x1_max - x1_min) / (nx_ic + 1);
        double dy = (x2_max - x2_min) / (ny_ic + 1);
        std::vector<Vec> ics;
        for (int i = 1; i <= nx_ic; ++i)
            for (int j = 1; j <= ny_ic; ++j)
                ics.push_back({x1_min + i*dx, x2_min + j*dy});
        trajectories(f, ics, T, dt);
    }

    // ============================================================
    //                       NULLCLINES
    // ============================================================

    /**
     * @brief Compute and plot nullclines by scanning the grid
     *
     * Nullclines are curves where ẋ₁ = 0 (f₁=0) or ẋ₂ = 0 (f₂=0).
     * Detected by sign changes along grid lines.
     *
     * @param f          Vector field
     * @param resolution Grid resolution for nullcline scan
     */
    void nullclines(const VectorField2D& f, int resolution = 200,
                    const std::string& color1 = "r",
                    const std::string& color2 = "b") {
        double dx = (x1_max - x1_min) / resolution;
        double dy = (x2_max - x2_min) / resolution;

        std::vector<double> nc1_x1, nc1_x2; // f₁ = 0
        std::vector<double> nc2_x1, nc2_x2; // f₂ = 0

        // Scan along x₁ for fixed x₂
        for (int j = 0; j <= resolution; ++j) {
            double x2 = x2_min + j * dy;
            for (int i = 0; i < resolution; ++i) {
                double x1a = x1_min + i * dx;
                double x1b = x1a + dx;
                Vec fa = f({x1a, x2});
                Vec fb = f({x1b, x2});

                // f₁ sign change
                if (fa[0] * fb[0] <= 0) {
                    nc1_x1.push_back(0.5*(x1a+x1b));
                    nc1_x2.push_back(x2);
                }
                // f₂ sign change
                if (fa[1] * fb[1] <= 0) {
                    nc2_x1.push_back(0.5*(x1a+x1b));
                    nc2_x2.push_back(x2);
                }
            }
        }

        if (!nc1_x1.empty())
            cppplot::plot(nc1_x1, nc1_x2, color1 + ".",
                          {{"markersize","1"}, {"label","ẋ₁=0 nullcline"}});
        if (!nc2_x1.empty())
            cppplot::plot(nc2_x1, nc2_x2, color2 + ".",
                          {{"markersize","1"}, {"label","ẋ₂=0 nullcline"}});
    }

    // ============================================================
    //                    EQUILIBRIA FINDING
    // ============================================================

    /**
     * @brief Find all equilibria in the domain via Newton-Raphson from grid seeds
     *
     * Seeds Newton iterations from a coarse grid of starting points.
     * Deduplicates nearby equilibria.
     *
     * @param f        Vector field
     * @param grid_n   Grid size for seeds (grid_n × grid_n)
     * @param tol      Newton convergence tolerance
     * @return         Vector of classified equilibria
     */
    std::vector<Equilibrium> equilibria(const VectorField2D& f,
                                        int grid_n = 10,
                                        double tol = 1e-8) {
        std::vector<Equilibrium> found;
        double dx = (x1_max - x1_min) / (grid_n - 1);
        double dy = (x2_max - x2_min) / (grid_n - 1);

        for (int i = 0; i < grid_n; ++i) {
            for (int j = 0; j < grid_n; ++j) {
                double x1 = x1_min + i * dx;
                double x2 = x2_min + j * dy;
                Vec x = {x1, x2};

                // Newton-Raphson: x ← x - J⁻¹f(x)
                bool converged = false;
                for (int iter = 0; iter < 50; ++iter) {
                    Vec fv = f(x);
                    double res = std::sqrt(fv[0]*fv[0] + fv[1]*fv[1]);
                    if (res < tol) { converged = true; break; }

                    // 2×2 Jacobian
                    auto J = numeric_jacobian_2d(f, x);
                    double det = J[0]*J[3] - J[1]*J[2];
                    if (std::abs(det) < 1e-15) break;

                    // J⁻¹ * fv (2×2 inverse)
                    double inv_det = 1.0 / det;
                    double dx1 = inv_det*(J[3]*fv[0] - J[1]*fv[1]);
                    double dx2 = inv_det*(-J[2]*fv[0] + J[0]*fv[1]);
                    x[0] -= dx1;
                    x[1] -= dx2;

                    // Keep within extended domain
                    double margin = 2.0 * std::max(x1_max-x1_min, x2_max-x2_min);
                    if (std::abs(x[0]) > margin || std::abs(x[1]) > margin) break;
                }

                if (!converged) continue;

                // Check within display domain
                if (x[0] < x1_min || x[0] > x1_max ||
                    x[1] < x2_min || x[1] > x2_max) continue;

                // Deduplicate (merge equilibria closer than tol*100)
                double dedup_tol = 1e-4;
                bool duplicate = false;
                for (const auto& eq : found) {
                    if (std::abs(eq.x1 - x[0]) < dedup_tol &&
                        std::abs(eq.x2 - x[1]) < dedup_tol) {
                        duplicate = true; break;
                    }
                }
                if (duplicate) continue;

                // Classify
                auto J = numeric_jacobian_2d(f, x);
                auto eq = classify_equilibrium(x[0], x[1], J);
                found.push_back(eq);
            }
        }

        // Draw equilibria on the current plot
        for (const auto& eq : found) {
            std::string marker, color;
            switch (eq.type) {
                case EquilibriumType::STABLE_NODE:
                case EquilibriumType::STABLE_SPIRAL:
                    marker = "bs"; color = "b"; break; // blue solid square
                case EquilibriumType::UNSTABLE_NODE:
                case EquilibriumType::UNSTABLE_SPIRAL:
                    marker = "r^"; color = "r"; break; // red triangle
                case EquilibriumType::SADDLE:
                    marker = "kD"; color = "k"; break; // black diamond
                case EquilibriumType::CENTER:
                    marker = "go"; color = "g"; break; // green circle
                default:
                    marker = "m*"; color = "m"; break;
            }
            cppplot::plot({eq.x1}, {eq.x2}, marker,
                          {{"markersize","10"}, {"label", eq.typeString()}});
        }

        return found;
    }

    // ============================================================
    //                    LIMIT CYCLE DETECTION
    // ============================================================

    /**
     * @brief Detect limit cycles via Poincaré section crossings
     *
     * Integrates a trajectory and records crossings of the x₁ = 0 section.
     * A periodic orbit is detected if consecutive crossings converge.
     *
     * @param f          Vector field
     * @param x0         Starting initial condition (should be near limit cycle)
     * @param T          Total integration time
     * @param dt         Time step
     * @param section_x1 Poincaré section location (x₁ = section_x1)
     * @return           Detected crossings {x₂ values at section}
     */
    struct LimitCycleResult {
        bool detected = false;
        double period = 0.0;
        double amplitude = 0.0;        ///< Amplitude of x₁ oscillation
        std::vector<double> crossings; ///< x₂ values at Poincaré section
    };

    LimitCycleResult detect_limit_cycle(const VectorField2D& f, Vec x0,
                                         double T = 100.0, double dt = 0.005,
                                         double section_x1 = 0.0) {
        LimitCycleResult result;
        Vec x = x0;
        double t = 0;
        double prev_x1 = x[0];
        double prev_t = 0;
        std::vector<double> crossing_times;

        double x1_max_val = x[0], x1_min_val = x[0];

        while (t < T) {
            x = rk4_step(f, x, dt);
            t += dt;

            x1_max_val = std::max(x1_max_val, x[0]);
            x1_min_val = std::min(x1_min_val, x[0]);

            // Detect upward crossing of x₁ = section_x1
            if (prev_x1 < section_x1 && x[0] >= section_x1) {
                // Linear interpolation to find exact crossing time
                double frac = (section_x1 - prev_x1) / (x[0] - prev_x1);
                double t_cross = t - dt + frac * dt;

                result.crossings.push_back(x[1]);
                crossing_times.push_back(t_cross);
            }
            prev_x1 = x[0];

            // After warmup (T/5), check if crossings are converging
            if (t > T/5 && result.crossings.size() > 3) {
                size_t n = result.crossings.size();
                double spread = std::abs(result.crossings[n-1] - result.crossings[n-2]);
                if (spread < 1e-3) {
                    result.detected = true;
                    if (crossing_times.size() >= 2) {
                        result.period = crossing_times[n-1] - crossing_times[n-2];
                    }
                }
            }
        }

        result.amplitude = (x1_max_val - x1_min_val) / 2.0;
        return result;
    }

    // ============================================================
    //              DISPLAY & FINALIZATION
    // ============================================================

    /**
     * @brief Set axis labels, title, legend, and gridlines
     *
     * Call this after all quiver/trajectory/nullcline calls.
     */
    void show(bool show_legend = true, bool show_grid = true) {
        cppplot::xlim(x1_min, x1_max);
        cppplot::ylim(x2_min, x2_max);
        cppplot::xlabel(x1_label);
        cppplot::ylabel(x2_label);
        cppplot::title(title_str);
        if (show_grid) cppplot::grid(true);
        if (show_legend) cppplot::legend(true);
    }
};

// ============================================================
//          FREE FUNCTIONS (CONVENIENCE API)
// ============================================================

/**
 * @brief Quick one-liner phase portrait
 *
 * Creates a PhasePortrait, adds quiver + trajectory grid, and calls show().
 *
 * @param f         Vector field
 * @param x1_range  {min, max}
 * @param x2_range  {min, max}
 * @param T         Integration time per trajectory
 */
inline std::vector<Equilibrium>
phase_portrait(const VectorField2D& f,
               std::pair<double,double> x1_range,
               std::pair<double,double> x2_range,
               double T = 10.0,
               int quiver_n = 18, int traj_grid = 4) {
    PhasePortrait pp(x1_range, x2_range, quiver_n, quiver_n);
    pp.quiver(f);
    pp.trajectory_grid(f, T, traj_grid, traj_grid);
    pp.nullclines(f);
    auto eqs = pp.equilibria(f);
    pp.show();
    return eqs;
}

/**
 * @brief Print equilibrium information to console
 */
inline void print_equilibria(const std::vector<Equilibrium>& eqs) {
    std::cout << "========== Equilibria Found: " << eqs.size() << " ==========\n";
    for (size_t i = 0; i < eqs.size(); ++i) {
        const auto& eq = eqs[i];
        std::cout << "\n[" << i+1 << "] (" << eq.x1 << ", " << eq.x2 << ")\n";
        std::cout << "    Type:  " << eq.typeString() << "\n";
        std::cout << "    Trace: " << eq.trace_J << "  Det: " << eq.det_J << "\n";
        std::cout << "    λ₁ = " << eq.eig1_re;
        if (std::abs(eq.eig1_im) > 1e-10)
            std::cout << " ± " << std::abs(eq.eig1_im) << "j";
        std::cout << "\n";
        std::cout << "    λ₂ = " << eq.eig2_re;
        if (std::abs(eq.eig2_im) > 1e-10)
            std::cout << " ± " << std::abs(eq.eig2_im) << "j";
        std::cout << "\n";
    }
    std::cout << "=================================================\n";
}

} // namespace nonlinear
} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_NONLINEAR_PHASE_PORTRAIT_HPP
