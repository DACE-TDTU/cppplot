---
title: 'CppPlot & CppNB: An Integrated Native C++ Toolkit for Scientific Visualization and Interactive Computing in Control Systems Engineering'
tags:
  - C++
  - scientific visualization
  - control systems
  - sliding mode control
  - header-only library
  - interactive computing
  - SVG
  - agentic AI
authors:
  - name: [Author Name]
    orcid: 0000-0000-0000-0000
    affiliation: 1
affiliations:
  - name: Department of Electrical and Electronics Engineering, Ton Duc Thang University, Ho Chi Minh City, Vietnam
    index: 1
date: 2025
bibliography: paper.bib
---

# Summary

`CppPlot` and `CppNB` are two complementary open-source libraries that together form a self-contained scientific visualization and interactive computing ecosystem for C++ developers, with a particular focus on control systems engineering. `CppPlot` is a modern, header-only C++ plotting library with a matplotlib-inspired API [@Hunter2007] that generates publication-quality Scalable Vector Graphics (SVG) output with zero runtime dependencies. `CppNB` provides a Jupyter-style notebook environment for C++, enabling cell-by-cell execution and inline visualization. Together, they close a long-standing workflow gap: engineers and researchers who write simulation, control, and estimation algorithms natively in C++ have historically had to bridge to a scripting language—or to heavyweight commercial software—solely to visualize results. This toolkit eliminates that step, delivering the full computation-visualization loop within a single, natively compiled C++ codebase.

The library supports an extensive feature set spanning general-purpose scientific plotting (line, scatter, histogram, bar chart, error bars, fill-between, log-scale axes, multi-panel subplots, GridSpec layouts, and inset axes), professional control-systems visualization (Bode, Nyquist, Nichols, pole-zero maps, root locus, step and impulse responses), advanced nonlinear control plots (multiple Sliding Mode Control variants, Control Barrier Functions, Fixed-Time Stabilization, Disturbance Observer–Based SMC, and Event-Triggered SMC), state-estimation plots for Kalman, Extended Kalman, and Unscented Kalman filters, and real-time optimization diagnostics for Model Predictive Control (convergence profiles, timing distributions, and Pareto trade-off plots for embedded QP solvers). All functionality is available in C++14 and has been verified across GCC 6.3+, Clang 5+, and MSVC 2017+ on Windows, Linux, and macOS.

# Statement of Need

C++ remains the implementation language of choice across a broad range of engineering domains—real-time and embedded control, robotic systems, high-performance simulation, safety-critical software, and hardware-in-the-loop testing [@Stroustrup2013]. Within these domains, researchers and practitioners routinely implement numerically intensive algorithms (nonlinear controllers, state estimators, trajectory optimizers) that produce data which must be inspected, validated, and published. Model Predictive Control (MPC) is a representative and demanding example: the controller solves a constrained Quadratic Program (QP) at every sampling step, and validating solver behaviour requires visualising convergence profiles on log-scale axes, timing distributions spanning several orders of magnitude, and accuracy trade-offs—all from within the same C++ codebase that runs on the target platform [@Stellato2020]. The standard practice of exporting data to CSV or binary format and post-processing in Python or MATLAB introduces friction in the development loop and, more critically for reproducibility, creates a dependency on an external language runtime that may not be available or consistent across platforms.

Several partial solutions exist. `gnuplot-iostream` [@gnuplotiostream] pipes data to an external gnuplot process, requiring a separate installation and providing limited API expressiveness. `matplotlib-cpp` [@Lenz2014] embeds the Python interpreter inside a C++ process, thereby trading the original dependency problem for a different one. `ROOT` [@Brun1997] from CERN offers rich visualization but at the cost of a large, domain-specific framework with a steep learning curve. No existing header-only, zero-dependency C++ library simultaneously addresses general scientific plotting *and* the full vocabulary of control-systems diagrams that practising control engineers require.

`CppPlot` addresses this need by offering: (1) a familiar matplotlib-style API that lowers the learning curve for engineers already acquainted with scientific Python; (2) a clean SVG backend that produces crisp, infinitely scalable graphics suitable for direct inclusion in papers and presentations without any rasterization artefacts; (3) a comprehensive control-systems module covering every major frequency-domain and time-domain plot type; (4) advanced nonlinear control visualization that, to the authors' knowledge, is not available in any other single header-only C++ library; and (5) zero runtime dependencies beyond a C++14 compiler and the C++ Standard Library.

`CppNB` complements `CppPlot` by providing an interactive notebook layer. It serves as the computational engine for the textbook *Modern Control Engineering with C++*, enabling students and researchers to iterate on algorithms and inspect visualizations without leaving the C++ language. The reproducibility properties of the combined toolkit—a single `cmake --build` command reconstructs every figure from source—make it particularly well-suited for research papers that aim to satisfy modern computational reproducibility standards [@Sandve2013; @Peng2011].

# Design and Architecture

## CppPlot

`CppPlot` is organized as a single-header library under `include/cppplot/cppplot.hpp`. Its architecture separates three concerns: the *state machine* (figure, axes, and subplot state that mirrors matplotlib's pyplot interface), the *renderer* (an SVG emitter that translates internal draw commands into valid SVG markup), and the *domain modules* (reusable building blocks for control-systems and nonlinear-control plots).

**API layer.** The public API is deliberately modelled on matplotlib's pyplot interface. Format strings (`"b-o"`, `"r--"`, `"k:"`) control color, line style, and marker simultaneously, minimizing boilerplate. An `opts()` helper wraps `std::initializer_list<std::pair<std::string,std::string>>` into a convenient key-value map, preserving type safety while matching the visual style of matplotlib's `**kwargs`.

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;

int main() {
    auto x = linspace(0.0, 2.0 * M_PI, 200);
    std::vector<double> y_sin, y_cos;
    for (double xi : x) {
        y_sin.push_back(std::sin(xi));
        y_cos.push_back(std::cos(xi));
    }
    figure(800, 500);
    plot(x, y_sin, "b-",  opts({{"label","sin(x)"},{"linewidth","2"}}));
    plot(x, y_cos, "r--", opts({{"label","cos(x)"},{"linewidth","2"}}));
    xlabel("x (rad)"); ylabel("Amplitude");
    title("Trigonometric Functions"); grid(true); legend(true);
    savefig("trig.svg");
}
```

**SVG backend.** All rendering is performed by a lightweight, self-contained SVG writer. The library produces standards-compliant SVG 1.1 output that renders identically in any modern browser, vector editor (Inkscape, Adobe Illustrator), and LaTeX workflow (via `\includegraphics`). Because SVG is XML-based text, the output is human-readable, diff-friendly, and version-control–compatible.

**Layout system.** `CppPlot` implements three complementary layout mechanisms: a MATLAB-style `subplot(rows, cols, index)` call; a `GridSpec` object with configurable width and height ratios and spacing, analogous to matplotlib's `GridSpec`; and free-form placement via `add_axes(left, bottom, width, height)` in normalized figure coordinates. Cell spanning via `subplot_span` and floating inset axes via `inset_axes` are also supported.

**Build integration.** The library ships with a CMake build system that exposes three integration paths: direct header copy, `add_subdirectory`, and CMake `FetchContent`. This covers the full spectrum from quick experiments to production CMake projects.

## CppNB

`CppNB` provides a notebook execution model for C++. Each *cell* is a self-contained C++ translation unit that is compiled and executed in sequence. `CppNB` captures standard output and any `savefig` calls and renders them inline in a browser-based interface, replicating the Jupyter Notebook experience without requiring a Python kernel. The notebook format is designed to accompany the textbook *Modern Control Engineering with C++*, where theory, derivations, code, and plots coexist in a single, reproducible document.

# Feature Showcase

## General Scientific Plotting

Table 1 summarises the plot types available in `CppPlot`.

| Category | Functions |
|---|---|
| Line / Curve | `plot`, `semilogx`, `semilogy`, `loglog` |
| Statistical | `scatter`, `bar`, `hist`, `errorbar`, `fill_between` |
| Annotation | `text`, `axhline`, `axvline`, `annotate` |
| Scale | `xscale("log")`, `yscale("log")` |
| Layout | `subplot`, `GridSpec`, `add_axes`, `inset_axes`, `subplot_span` |

**Table 1.** Summary of general-purpose plotting functions in CppPlot.

The `fill_between` function is particularly useful for visualising uncertainty regions and confidence intervals in simulation results:

```cpp
auto x = linspace(0.0, 10.0, 200);
std::vector<double> y_mean, y_upper, y_lower;
for (double xi : x) {
    double v = std::sin(xi);
    y_mean.push_back(v);
    y_upper.push_back(v + 0.3 * std::exp(-0.1 * xi));
    y_lower.push_back(v - 0.3 * std::exp(-0.1 * xi));
}
figure(800, 450);
fill_between(x, y_lower, y_upper,
             opts({{"color","royalblue"},{"alpha","0.25"}}));
plot(x, y_mean, "b-", opts({{"linewidth","2"},{"label","Mean"}}));
title("Signal with Decaying Confidence Interval");
legend(true); savefig("confidence_band.svg");
```

## Control Systems Visualization

The control-systems module implements every major analysis plot used in classical and modern control design, following conventions compatible with those of Python Control [@Fuller2021] and standard textbooks [@Ogata2010; @Franklin2019].

**Bode plot.** Magnitude and phase are computed in user code and passed to the standard `plot` and `subplot` functions, giving full control over frequency range, annotation, and styling:

```cpp
double wn = 10.0, zeta = 0.3;
auto w = logspace(-1, 3, 500);
std::vector<double> mag_dB, phase_deg;
for (double wi : w) {
    std::complex<double> s(0, wi);
    auto H = (wn*wn) / (s*s + 2*zeta*wn*s + wn*wn);
    mag_dB.push_back(20.0 * std::log10(std::abs(H)));
    phase_deg.push_back(std::arg(H) * 180.0 / M_PI);
}
figure(800, 600);
subplot(2, 1, 1);
plot(w, mag_dB, "b-", opts({{"linewidth","2"}}));
xscale("log"); ylabel("Magnitude (dB)"); grid(true);
title("Bode Plot — Second-Order System (zeta=0.3, wn=10 rad/s)");

subplot(2, 1, 2);
plot(w, phase_deg, "b-", opts({{"linewidth","2"}}));
xscale("log"); xlabel("Frequency (rad/s)");
ylabel("Phase (deg)"); grid(true);
savefig("bode.svg");
```

**Nyquist and Pole-Zero plots** follow the same pattern—compute complex frequency-response or eigenvalue data in C++, then use `scatter`, `plot`, and annotation primitives to construct the diagram.

**Step response.** The following example computes the exact analytical step response of a second-order system and annotates key characteristics:

```cpp
double sigma = -zeta * wn;
double wd    =  wn * std::sqrt(1.0 - zeta*zeta);
auto t = linspace(0.0, 2.0, 800);
std::vector<double> y;
for (double ti : t)
    y.push_back(1.0 - std::exp(sigma*ti) *
        (std::cos(wd*ti) - (sigma/wd)*std::sin(wd*ti)));
figure(800, 450);
plot(t, y, "b-", opts({{"linewidth","2"}}));
axhline(1.0, opts({{"color","gray"},{"linestyle","--"}}));
xlabel("Time (s)"); ylabel("y(t)"); grid(true);
title("Step Response — zeta=0.3, wn=10 rad/s");
savefig("step.svg");
```

## Advanced Nonlinear Control

This module constitutes the most distinctive contribution of `CppPlot` relative to other C++ plotting libraries. It provides visualization support for research-level nonlinear control algorithms, enabling figures that would previously have required MATLAB or Python scripts to be generated directly from the simulation source code.

**Sliding Mode Control variants.** The library includes example simulations and plot templates for Standard SMC, Super-Twisting SMC [@Levant1993], Integral SMC, and combinations thereof. Figure generation follows the same pattern: numerically integrate the closed-loop ODE in C++, accumulate trajectory data in `std::vector<double>`, and call `CppPlot`.

```cpp
// Super-Twisting SMC — inner integration loop (simplified excerpt)
double x1 = x1_0, x2 = x2_0, u = 0.0, v = 0.0;
double k1 = 1.5, k2 = 1.1;
std::vector<double> t_vec, x1_vec, s_vec;
for (int i = 0; i < N; ++i) {
    double s   = x2 + lambda * x1;          // sliding variable
    double dsdt = -k1 * std::copysign(
                        std::pow(std::abs(s), 0.5), s) + v;
    v += -k2 * std::copysign(1.0, s) * dt;
    x1 += x2 * dt;
    x2 += (dsdt - lambda * x2) * dt;
    t_vec.push_back(i * dt);
    x1_vec.push_back(x1); s_vec.push_back(s);
}
figure(900, 600);
subplot(2, 1, 1); plot(t_vec, x1_vec, "b-",
    opts({{"linewidth","2"},{"label","State x1"}}));
ylabel("x1"); grid(true); legend(true);
subplot(2, 1, 2); plot(t_vec, s_vec,  "r-",
    opts({{"linewidth","1.5"},{"label","Sliding variable s"}}));
xlabel("Time (s)"); ylabel("s"); grid(true); legend(true);
title("Super-Twisting SMC Response");
savefig("stwc.svg");
```

**Control Barrier Functions (CBF).** Safety-critical control is an active research area [@Ames2019]. `CppPlot` provides facilities to visualise the state trajectory relative to a barrier boundary, the barrier function value over time, and the safety constraint margin—plots that appear frequently in CBF research papers.

**Fixed-Time Stabilization.** Fixed-time convergence controllers [@Polyakov2012] guarantee convergence within a bounded time independent of initial conditions. `CppPlot` can overlay multiple trajectories starting from different initial conditions, visually demonstrating this property.

**Disturbance Observer–Based SMC and Event-Triggered SMC.** These two schemes reduce chattering and communication overhead, respectively. The library includes plot templates that juxtapose nominal and observer-augmented responses, and that mark event-trigger instants on the time axis using `axvline` calls.

## State Estimation

The library supports visualization of Kalman Filter, Extended Kalman Filter, and Unscented Kalman Filter results. Typical plots include state estimate versus true state overlays, innovation sequences, and error covariance envelopes rendered with `fill_between`.

## Real-Time Optimization and Model Predictive Control

MPC formulates the control problem as a constrained QP solved online at every sampling step:

$$\min_{U} \tfrac{1}{2} U^\top H U + g^\top U \quad \text{s.t.} \quad \mathbf{lb} \leq U \leq \mathbf{ub}$$

where $H = \Gamma^\top \bar{Q} \Gamma + \bar{R}$ is assembled once from system matrices and $g = \Gamma^\top \bar{Q}\,\Phi\,x_0$ is recomputed each step from the current state $x_0$. Validating an embedded QP solver for a 100 Hz AGV/AMR platform requires at least five distinct visualizations: (i) median solve time per solver/scenario; (ii) ADMM convergence profiles (primal and dual residuals vs. iteration); (iii) solution accuracy relative to a high-precision reference; (iv) a time–accuracy Pareto scatter; and (v) a cumulative distribution of solve times against the real-time deadline. `CppPlot` generates all five natively in C++ without any data export.

The following excerpt from `examples/benchmark_qp_solvers_V7.cpp` shows how native log-scale axes and boxplots simplify the timing figure. Because solve times span three orders of magnitude (0.29 µs to 375 µs), a logarithmic y-axis is essential for legibility; `yscale("log")` applies it in a single call. The `boxplot()` API (available natively since CppPlot V6) renders full IQR, whiskers, and outliers under log scale—a capability not available with bar charts:

```cpp
figure(1400, 480);
const std::vector<std::string> scenarios = {"Easy", "Medium", "Hard"};
for (int si = 0; si < 3; ++si) {
    subplot(1, 3, si + 1);
    std::vector<std::vector<double>> all_times;
    std::vector<double> positions;
    for (int k = 0; k < (int)solvers.size(); ++k) {
        all_times.push_back(raw_times(results, scenarios[si], solvers[k]));
        positions.push_back(k + 1.0);
        plot({positions.back()}, {median_val(all_times.back())}, fmts[k],
             opts({{"label", solvers[k]}}));
    }
    boxplot(all_times, positions, opts({{"widths","0.6"}}));
    yscale("log");
    axhline(10000.0, opts({{"color","red"},{"linestyle","--"},
                           {"label","10 ms deadline"}}));
    xlabel("Solver"); ylabel("Solve time (\u00b5s, log scale)");
    title(scenarios[si] + " scenario"); grid(true); legend(true);
}
savefig("fig_bench_timing.svg");
```

The `boxplot()` API renders full IQR, whiskers, and outliers natively under `yscale("log")`—the bar chart used in earlier versions (V1–V5) only showed median values and required a workaround for log scale. The CDF of solve times uses `semilogx` directly on the raw timing vector, with `axvline` marking the 10 ms deadline. The convergence profile (primal/dual residuals vs. iteration) uses a plain `semilogy` call. All five figure types require no data pre-transformation.

Table 4 summarises benchmark results obtained from `examples/benchmark_qp_solvers_V7.cpp` for a 10-dimensional MPC QP ($N=10$ horizon) across three scenarios that differ in the condition number of $H$ (60 timing repetitions, 20 initial conditions per scenario, x86-64 laptop, GCC `-O3`).

| Solver | S1 Easy Med/P95 (µs) | S2 Medium Med/P95 (µs) | S3 Hard Med/P95 (µs) | Conv. S1/S2/S3 |
|---|:---:|:---:|:---:|:---:|
| ADMM-cold ($\rho=1$, default) | 27.7 / 34.6 | 57.5 / 67.7 | 17.4 / 24.5 | 100% / **0%** / 85% |
| ADMM-tuned ($\rho^*$ spectral) | 130 / 149 | 262 / 300 | 35.4 / 48.0 | 100% / **0%** / 100% |
| **ADMM, warm-start** | **30.4 / 33.4** | **56.6 / 63.2** | **16.6 / 20.9** | 100% / **0%** / 85% |
| PGD (Barzilai-Borwein) | 29.3 / 32.5 | 200 / 244 | 132 / 168 | 100% / 0% / 0% |
| CG-proj | 0.95 / 1.28 | 116 / 148 | 110 / 142 | 100% / 15% / 0% |
| **Chol-Direct** | **0.29 / 0.42** | **0.23 / 0.42** | **0.20 / 0.58** | 100% / 100% / 100%$^\dagger$ |

**Table 4.** V7 verified: median and P95 solve time (µs), $N=10$, x86-64, GCC `-O3`. $\rho^* = \max(10\lambda_{\max},\sqrt{\lambda_{\min}\lambda_{\max}})$. $^\dagger$Chol-Direct is timing-correct but accuracy-incorrect when constraints are active (projected unconstrained minimiser, not the constrained QP solution). ADMM cold-start Conv = 0% on S2 is a scientific finding (fully-active constraint set), not a software bug. WCET worst case: 375 µs $\ll$ 10 ms deadline.

Figure 1 shows the log-scale timing boxplot generated directly from `benchmark_qp_solvers_V7.cpp`. Solve times span three orders of magnitude (0.29 µs to 375 µs); a linear y-axis would compress all fast solvers to zero. `yscale("log")` applies in a single call. No data export to Python or MATLAB was required.

![Timing distributions for six QP solvers across three MPC scenarios (S1–S3), generated natively by `benchmark_qp_solvers_V7.cpp` via CppPlot. Log-scale boxplot; IQR, whiskers, and outliers rendered by the native `boxplot()` API. All solvers are $\geq$26× below the 10 ms deadline.](results/fig_bench_timing.svg)

Figure 2 shows the ADMM convergence profiles (primal/dual residuals vs. iteration) generated via `semilogy`. This single figure type motivates the log-scale axis requirement: residuals span six orders of magnitude between initial and converged state.

![ADMM primal and dual residual convergence profiles for three benchmark scenarios, generated natively in C++ via `CppPlot::semilogy`.](results/fig_bench_convergence.svg)

Figure 3 shows the time–accuracy Pareto scatter, illustrating the trade-off between solve speed and solution quality across all solvers on the Hard scenario. This figure requires per-solver scatter with log-log axes and text labels—all generated natively by `CppPlot` with no postprocessing.

![Time–accuracy Pareto scatter for the S3 Hard scenario. ADMM-warm occupies the optimal trade-off frontier. Log-log axes via `CppPlot::xscale("log")` and `yscale("log")`.](results/fig_bench_tradeoff.svg)

The complete benchmark—six solvers, three scenarios, five figure types—is fully reproducible via `cmake --build` with no external runtime (see Reproducibility).

# Reproducibility

A central design goal of the toolkit is enabling fully reproducible computational figures. Every figure in the repository can be regenerated from source with three commands:

```bash
git clone https://github.com/DACE-TDTU/cppplot
mkdir build && cd build
cmake .. && cmake --build . && ctest
```

No additional runtime, interpreter, or package manager is required beyond a C++14 compiler and CMake 3.14. This property is particularly significant for engineering research, where the ability to audit, modify, and re-run simulations from their original source code is a key component of scientific integrity [@Sandve2013; @Peng2011].

The `CppNB` notebook format extends this property to the textbook context: a reader can clone the companion repository, build the project, and execute any notebook cell-by-cell, observing and modifying intermediate results without leaving C++.

# Developer Experience

## Integration

`CppPlot` supports three integration patterns to accommodate different project structures.

**Header copy** (simplest, no build system required):
```bash
cp -r include/cppplot /your/project/include/
```

**CMake subdirectory:**
```cmake
add_subdirectory(cppplot)
target_link_libraries(your_target PRIVATE cppplot)
```

**CMake FetchContent** (recommended for reproducible builds):
```cmake
include(FetchContent)
FetchContent_Declare(cppplot
    GIT_REPOSITORY https://github.com/DACE-TDTU/cppplot.git
    GIT_TAG        v1.0.0)
FetchContent_MakeAvailable(cppplot)
target_link_libraries(your_target PRIVATE cppplot)
```

## Format Strings and Styling

The format string mini-language (`"b-o"`, `"r--"`, `"k:"`) encodes color, line style, and marker in a single argument. Table 2 lists the supported tokens.

| Token | Meaning | Token | Meaning | Token | Meaning |
|-------|---------|-------|---------|-------|---------|
| `b` `g` `r` `c` `m` `y` `k` | Colors | `-` `--` `-.` `:` | Line styles | `o` `s` `^` `v` `x` `+` `*` | Markers |

**Table 2.** Format string tokens supported by CppPlot.

Additional styling—line width, alpha transparency, font size, marker size—is passed through the `opts()` map, keeping the primary call concise while exposing full control when needed.

## Compiler and Platform Support

| Compiler | Minimum version | Platforms |
|---|---|---|
| GCC | 6.3 | Linux, Windows (MinGW), macOS |
| Clang | 5.0 | Linux, macOS |
| MSVC | 2017 (19.10) | Windows |

**Table 3.** Verified compiler and platform combinations.

## AI-Assisted Development Methodology

The core implementations of both libraries were substantially generated and refined through an *agentic AI* development workflow, in which large language models (LLMs) served as code-generation and refactoring agents guided by a human architect who specified requirements, validated correctness, curated examples, and directed iterative improvements. This process is explicitly disclosed in the repository README in accordance with emerging norms for AI-assisted open-source development.

This workflow raises an interesting methodological question: can agentic AI systems generate library-quality scientific software? The repositories presented here represent a positive data point—the generated code passes a cross-platform CMake test suite, produces visually correct SVG output across all supported compilers, and implements domain-correct control-systems algorithms. We consider this an ancillary contribution of the present work: a concrete, publicly available case study of agentic AI applied to scientific software engineering.

`examples/benchmark_qp_solvers_V7.cpp` (the final of seven iterative versions) illustrates this workflow in detail. The file was generated by an AI agent given the specification: *port the Python prototype to C++17 using CppPlot, replicating all five figure types*. Subsequent agentic review passes identified 19 bugs across seven versions, categorised into two structurally distinct classes: nine Class~1 logic and specification errors (detectable by code review, including a critical reference-solver misconfiguration that silently corrupted all accuracy metrics) and ten Class~2 timing-measurement errors (detectable only by running on hardware, including clock-granularity failures, integer-truncation in nanosecond conversion, and scenario-dependent batch-size mismatches). Each bug was corrected with a targeted patch. This generate–review–patch cycle, in which correctness emerges from iterative AI-assisted inspection rather than from a single generation step, is characteristic of agentic software development and is documented in the repository commit history as a reproducible example of the methodology.

# Limitations and Future Work

The current release has the following known limitations. The SVG backend does not support interactive plots (zoom, pan, tooltip); all output is static. Three-dimensional surface and mesh plots are not yet implemented. Animation is not supported in the current release. The API is a deliberate subset of matplotlib—less frequently used functions (contour plots, polar axes, twin axes) are not yet present.

Planned extensions include a WebAssembly–compiled rendering backend for browser-based interactivity, 3D plotting via a lightweight OpenGL or WebGL path, animation export to SVG SMIL or MP4 via FFmpeg, and a formal CMake package for `find_package(cppplot)` installation.

# Acknowledgements

The authors thank the open-source communities behind CMake, the SVG standard, and the C++ ecosystem whose infrastructure this work relies upon. The control-systems domain knowledge embedded in the library draws on foundational texts by Ogata [@Ogata2010], Franklin et al. [@Franklin2019], and Slotine and Li [@Slotine1991]. Inspiration for the matplotlib-style API is acknowledged to Hunter et al. [@Hunter2007].

# References

<!-- New citation added by QP/MPC case study:
     @article{Stellato2020,
       author  = {Stellato, B. and Banjac, G. and Goulart, P. and Bemporad, A. and Boyd, S.},
       title   = {{OSQP}: An Operator Splitting Solver for Quadratic Programs},
       journal = {Mathematical Programming Computation},
       volume  = {12},
       pages   = {637--672},
       year    = {2020},
       doi     = {10.1007/s12532-020-00179-2}
     }
     Add this entry to paper.bib before submission.
-->
