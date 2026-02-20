# Changelog

All notable changes to CppPlot library will be documented in this file.

## [1.6.0] - 2026-01-26

### Added - Control System Design Tools

#### State-Space Systems (`state_space.hpp`)
- **`Matrix`** class for linear algebra operations
  - Addition, subtraction, multiplication
  - Transpose, determinant, inverse
  - Eye, zeros factory methods
  
- **`StateSpace`** class for state-space representation (ẋ = Ax + Bu, y = Cx + Du)
  - Poles calculation via characteristic polynomial
  - Stability check
  - Controllability matrix and test
  - Observability matrix and test

- **Conversion Functions:**
  - `tf2ss(G)` - Transfer function to controllable canonical form
  - `ss2tf(sys)` - State-space to transfer function (SISO)

- **Simulation:**
  - `lsim(sys, u, t, x0)` - Arbitrary input simulation
  - `ss_step(sys)` - Step response of state-space system
  - `initial(sys, x0)` - Initial condition response

#### Controller Design (`controller_design.hpp`)
- **Pole Placement:**
  - `acker(A, B, poles)` - Ackermann's formula
  - `place(A, B, real_poles)` - Simplified real pole placement
  - `place_complex(A, B, poles)` - Complex conjugate poles

- **LQR Controller:**
  - `care(A, B, Q, R)` - Continuous algebraic Riccati equation solver
  - `lqr(A, B, Q, R)` - Linear Quadratic Regulator design
  - `lqr_simple(A, B, q_diag, r)` - Simplified diagonal weights interface

- **Observer Design:**
  - `observer_gain(A, C, poles)` - Luenberger observer via duality
  - `place_observer(A, C, poles)` - Simplified observer placement

- **Sensitivity Analysis:**
  - `SensitivityFunctions` struct (S, T, CS, PS)
  - `sensitivity(G, K)` - Calculate all sensitivity functions
  - `sensitivity_plot(G, K)` - Plot sensitivity Bode diagram

- **PID Tuning:**
  - `ziegler_nichols(Ku, Tu, type)` - Z-N tuning (P, PI, PID, no-overshoot)
  - `cohen_coon(K, T, L, type)` - Cohen-Coon for FOPDT plants
  - `pid_tf(gains)` - Create PID transfer function from gains

- **Compensator Design:**
  - `design_lead(phi_max, wc)` - Lead compensator for phase margin
  - `design_lag(gain_increase, wc)` - Lag compensator for DC gain

#### Block Diagram Algebra (`block_diagram.hpp`)
- **Standard Control Loops:**
  - `closed_loop(G, C, H)` - Standard feedback loop
  - `pid_loop(G, Kp, Ki, Kd)` - PID control loop
  - `two_dof_loop(G, Cr, Cy)` - Two degree-of-freedom structure

- **Advanced Structures:**
  - `cascade_control(G1, G2, C1, C2)` - Nested/cascade control
  - `feedforward_loop(G, C, Cff)` - Feedforward compensation
  - `nfeedback(G, H)` / `pfeedback(G, H)` - Explicit feedback sign

- **Disturbance Analysis:**
  - `disturbance_to_output(G, C)` - d→y transfer function
  - `noise_to_output(G, C)` - n→y transfer function

- **IMC:**
  - `imc_to_classical(G, Cimc)` - Convert IMC to classical form
  - `imc_first_order(K, tau, lambda)` - IMC design for first-order

- **Loop Shaping:**
  - `gain_for_crossover(G, wc)` - Find gain for desired crossover
  - `required_phase_lead(G, wc, PM)` - Calculate required phase boost

### Demo Files
- Added `control_design_demo.cpp` with comprehensive examples:
  - State-space modeling of DC motor
  - TF ↔ SS conversions
  - Pole placement design
  - LQR controller design
  - Observer design
  - PID tuning methods
  - Lead compensator design
  - Sensitivity function analysis

---

## [1.5.1] - 2026-01-26

### Added

#### Nichols Chart (`nichols.hpp`)
- **`nichols(G, options)`** - Plot Nichols chart for single system
- **`nichols(systems, labels, options)`** - Compare multiple systems
- **`nichols_m_circle(M_dB)`** - Generate M-circle (constant CL magnitude)
- **`nichols_n_circle(N_deg)`** - Generate N-circle (constant CL phase)
- **`NicholsOptions`** struct with show_m_circles, show_n_circles, show_margins

#### Discrete-Time Systems (`discrete.hpp`)
- **`DiscreteTransferFunction`** class for z-domain transfer functions
  - Sample time handling (Ts)
  - Poles/zeros in z-plane
  - Stability check (unit circle criterion)
  - DC gain, Nyquist frequency
  
- **Discretization Methods:**
  - `c2d_tustin(Gc, Ts)` - Bilinear (Tustin) transform
  - `c2d_zoh(Gc, Ts)` - Zero-Order Hold approximation
  
- **Discrete Factory Functions:**
  - `dtf_integrator(Ts)` - Forward Euler integrator
  - `dtf_integrator_tustin(Ts)` - Tustin integrator  
  - `dtf_lowpass(tau, Ts)` - First-order lowpass filter
  
- **Discrete Plotting:**
  - `dbode(H)` - Discrete Bode diagram (0 to Nyquist)
  - `dzpmap(H)` - Pole-zero map in z-plane with unit circle
  - `dstep(H, n)` - Discrete step response (stem plot)
  - `dsysinfo(H)` - Print discrete system information

### Demo Files
- Added `control_advanced_demo.cpp` with 8 comprehensive examples:
  - Nichols chart single/comparison
  - Discrete Bode, pole-zero, step response
  - Continuous to discrete conversion comparison
  - Second-order discretization
  - Digital lowpass filter design

---

## [1.5.0] - 2026-01-26

### Added

#### Control Systems Module (`include/cppplot/control/`)
Complete control systems analysis and plotting library following Python Control / MATLAB standards.

**New Files:**
- `control.hpp` - Main control module header
- `polynomial.hpp` - Polynomial arithmetic class
- `transfer_function.hpp` - Transfer function class with full arithmetic
- `analysis.hpp` - Margin, stepinfo, bandwidth, stability analysis
- `bode.hpp` - Automatic Bode plot generation
- `nyquist.hpp` - Nyquist diagram with critical point
- `pzmap.hpp` - Pole-zero map with damping lines
- `root_locus.hpp` - Root locus plotting
- `time_response.hpp` - Step, impulse response

**Transfer Function Features:**
- Create from coefficients: `TransferFunction G({1}, {1, 2, 1});`
- Create from zeros/poles/gain: `zpk({-1}, {-2, -3}, 5.0)`
- Factory functions: `tf_second_order()`, `tf_first_order()`, `tf_integrator()`
- Arithmetic: Series (`*`), Parallel (`+`), Feedback (`feedback()`)
- Properties: `poles()`, `zeros()`, `dcgain()`, `isStable()`

**Analysis Functions:**
- `margin(G)` - Gain and phase margins
- `stepinfo(G)` - Rise time, settling time, overshoot
- `bandwidth(G)` - -3dB bandwidth
- `damp(G)` - Damping ratio and natural frequency of poles

**One-liner Plotting (Python Control style):**
```cpp
bode(G);           // Automatic Bode plot
nyquist(G);        // Nyquist diagram  
pzmap(G);          // Pole-zero map
rlocus(G);         // Root locus
step(G);           // Step response
impulse(G);        // Impulse response
```

**Standard Test Systems:**
```cpp
systems::first_order(K, tau);
systems::second_order(wn, zeta, K);
systems::integrator(K);
systems::type1(K, a);
systems::pid(Kp, Ki, Kd);
systems::lead(z, p, K);
systems::lag(z, p, K);
```

### Demo Files
- Added `control_module_demo.cpp` with comprehensive examples
- Generated SVG outputs: step, bode, nyquist, pzmap, rlocus

---

## [1.2.0] - 2026-01-25

### Added

#### LaTeX Support for Mathematical Expressions
- **`LaTeXRenderer`** class in `core/latex.hpp` for converting LaTeX to Unicode
- Support for Greek letters: `\alpha`, `\beta`, `\gamma`, `\Delta`, `\Theta`, etc.
- Support for superscripts: `x^2`, `x^{10}`, `e^{i\pi}` → x², x¹⁰, eⁱπ
- Support for subscripts: `x_1`, `x_{12}` → x₁, x₁₂
- Support for math operators: `\sum`, `\int`, `\prod`, `\partial`, `\nabla`, `\infty`
- Support for relations: `\leq`, `\geq`, `\neq`, `\approx`, `\equiv`
- Support for arrows: `\to`, `\leftarrow`, `\Rightarrow`, `\leftrightarrow`
- Support for logic symbols: `\forall`, `\exists`, `\wedge`, `\vee`
- Support for fractions: `\frac{a}{b}` → (a/b)
- Support for square root: `\sqrt{x}` → √(x)

#### New Text Functions
- **`figtext(x, y, text, opts)`** - Add text at figure coordinates (0-1 normalized)
  - x=0 is left edge, x=1 is right edge
  - y=0 is bottom, y=1 is top
  - Perfect for adding formulas, titles, or annotations anywhere on the figure
- **`latex(x, y, expr, opts)`** - Explicitly render LaTeX expression

#### Enhanced Text Annotation
- Auto-detection of LaTeX in text (backslash commands, `^`, `_`, `$`)
- `usetex` option to force LaTeX rendering
- Math font family default for LaTeX text (STIX Two Math, Cambria Math)

### Changed
- `TextAnnotation` struct now includes `figureCoords`, `useLaTeX`, `useMathML` flags
- `text()` function now automatically parses LaTeX commands
- `drawAnnotations()` now supports figure coordinates and LaTeX rendering

### Demo Files
- Added `latex_demo.cpp` with 8 comprehensive examples:
  - Greek letters in labels
  - Superscripts and subscripts
  - Gaussian distribution with formula
  - Physics equations (projectile motion)
  - Calculus notation (derivatives, integrals)
  - Math symbols reference chart
  - Euler's formula visualization
  - Statistical formulas

---

## [1.1.0] - 2026-01-25

### Added

#### New Plot Functions
- **`errorbar(x, y, yerr, opts)`** - Plot with vertical error bars
- **`errorbar(x, y, xerr, yerr, opts)`** - Plot with both X and Y error bars
  - Supports `capsize` option for error bar cap width
  - Supports standard styling options (color, linewidth, label)

#### Fill Between
- **`fill_between(x, y1, y2, opts)`** - Fill area between two curves
- **`fill_between(x, y, baseline, opts)`** - Fill area to a constant baseline
  - Supports `alpha` for transparency
  - Supports `color` and `label` options
  - Useful for confidence intervals, stacked areas

#### Text Annotations
- **`text(x, y, "label", opts)`** - Add text annotation at data coordinates
- **`annotate("text", x, y, textX, textY, opts)`** - Add annotation with arrow
  - Supports `fontsize`, `fontweight`, `fontstyle`
  - Supports `color` (or `textcolor`)
  - Supports alignment: `ha` (left/center/right), `va` (top/center/bottom)

#### Axis Scaling
- **`set_xscale("log")`** / **`xscale("log")`** - Set X axis to logarithmic scale
- **`set_yscale("log")`** / **`yscale("log")`** - Set Y axis to logarithmic scale
- Scale can be "linear" (default) or "log"

#### Reference Lines
- **`axhline(y, opts)`** - Add horizontal line across the plot
- **`axvline(x, opts)`** - Add vertical line across the plot
  - Supports `color`, `linewidth`, `linestyle`
  - Useful for marking thresholds, means, or key values

### Changed
- Updated `Axes` class with new private draw methods
- Added `niceLogTicks()` utility function for log scale tick generation

### Documentation
- Updated README with new feature examples
- Updated AUDIT_REPORT.md with implementation status
- Added new demo files:
  - `advanced_demo.cpp` - Comprehensive demo of all new features
  - `log_scale_demo.cpp` - Log scale specific examples

---

## [1.0.0] - 2026-01-20

### Initial Release
- Line plots with format strings (matplotlib style)
- Scatter plots with size and color mapping
- Bar charts (vertical)
- Histograms with bin control
- Subplot system with flexible layouts
- GridSpec for advanced layouts
- SVG backend for vector output
- Rich styling system (colors, line styles, markers)
- Font customization (size, weight, style, family)
- Legend support
- Grid lines
- Axis labels and titles

### Features
- Header-only library
- C++14 compatible
- Cross-platform (Windows, Linux, macOS)
- Matplotlib-like API
- Julia Plots-inspired layout system
