# Appendix C: CppPlot Control Library Reference

---

> **Purpose:** This appendix provides a comprehensive API reference for the CppPlot Control Systems Library. It serves as a quick lookup guide for all classes, functions, and their usage patterns.

---

## C.1 Library Overview

### C.1.1 Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                       CppPlot Control Systems Library                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                        HIGH-LEVEL API                                 │  │
│  │  bode(), nyquist(), rlocus(), step(), impulse(), margin()           │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                    │                                        │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                      CONTROLLER DESIGN                                │  │
│  │  place(), acker(), lqr(), dlqr(), pid(), kalman(), mpc()            │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                    │                                        │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                       SYSTEM REPRESENTATIONS                          │  │
│  │  TransferFunction      StateSpace      DiscreteTransferFunction      │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                    │                                        │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                        CORE UTILITIES                                 │  │
│  │  Polynomial      Matrix      linspace()      logspace()              │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### C.1.2 Header Files

| Header | Description | Key Classes/Functions |
|--------|-------------|----------------------|
| `control.hpp` | Main include (includes all) | - |
| `transfer_function.hpp` | Continuous transfer functions | `TransferFunction` |
| `state_space.hpp` | State-space models | `StateSpace`, `Matrix` |
| `discrete.hpp` | Discrete-time systems | `DiscreteTransferFunction`, `c2d()`, `d2c()` |
| `bode.hpp` | Bode plot | `bode()`, `BodeOptions` |
| `nyquist.hpp` | Nyquist plot | `nyquist()`, `NyquistOptions` |
| `root_locus.hpp` | Root locus | `rlocus()`, `RlocusOptions` |
| `nichols.hpp` | Nichols chart | `nichols()`, `NicholsOptions` |
| `pzmap.hpp` | Pole-zero map | `pzmap()`, `PZOptions` |
| `time_response.hpp` | Step/impulse response | `step()`, `impulse()`, `lsim()` |
| `analysis.hpp` | Stability analysis | `margin()`, `stepinfo()`, `bandwidth()` |
| `controller_design.hpp` | Controller synthesis | `place()`, `acker()`, `lqr()`, `pid()` |
| `kalman.hpp` | Kalman filters | `KalmanFilter`, `ExtendedKalmanFilter`, `UnscentedKalmanFilter` |
| `lqg.hpp` | LQG control | `LQGController`, `designLQG()` |
| `mpc.hpp` | Model Predictive Control | `MPCController`, `MPCConfig` |
| `polynomial.hpp` | Polynomial operations | `Polynomial` |
| `block_diagram.hpp` | Block diagram algebra | `series()`, `parallel()`, `feedback()` |
| `robust/hinf.hpp` | H∞ Control | `hinf_state_feedback()`, `hinf_output_feedback()` |
| `robust/uncertainty.hpp` | Uncertainty modeling | `UnstructuredUncertainty`, `ParametricUncertaintySet` |
| `robust/mu_analysis.hpp` | μ-Analysis & Visualization | `MuAnalyzer`, `plotMuAnalysisSuite()` |
| `nonlinear/sliding_mode.hpp` | Sliding Mode Control Suite | `SlidingModeController`, 8 SMC algorithms |

### C.1.3 Quick Start

```cpp
#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    // Create a transfer function: G(s) = 1 / (s² + 2s + 1)
    TransferFunction G({1}, {1, 2, 1});
    
    // Analyze
    std::cout << "DC Gain: " << G.dcgain() << std::endl;
    std::cout << "Poles: ";
    for (auto p : G.poles()) std::cout << p << " ";
    
    // Plot step response
    figure(800, 500);
    step(G);
    savefig("step_response.svg");
    
    return 0;
}
```

---

## C.2 TransferFunction Class

### C.2.1 Overview

The `TransferFunction` class represents continuous-time SISO systems:

$$G(s) = \frac{b_m s^m + b_{m-1} s^{m-1} + \cdots + b_0}{a_n s^n + a_{n-1} s^{n-1} + \cdots + a_0}$$

### C.2.2 Constructors

```cpp
// Default: G(s) = 1
TransferFunction();

// From coefficient vectors [highest → lowest power]
TransferFunction(const std::vector<double>& num, const std::vector<double>& den);

// From initializer lists (most convenient)
TransferFunction(std::initializer_list<double> num, std::initializer_list<double> den);

// From Polynomial objects
TransferFunction(const Polynomial& num, const Polynomial& den);

// Constant gain
explicit TransferFunction(double K);
```

**Examples:**
```cpp
// G(s) = (2s + 1) / (s² + 3s + 2)
// Numerator: 2s + 1 → coefficients [2, 1]
// Denominator: s² + 3s + 2 → coefficients [1, 3, 2]
TransferFunction G1({2, 1}, {1, 3, 2});

// G(s) = 10 / (s + 5)
TransferFunction G2({10}, {1, 5});

// G(s) = 5 (constant)
TransferFunction G3(5.0);

// Using vectors
std::vector<double> num = {1, 0};  // s
std::vector<double> den = {1, 2, 1};  // s² + 2s + 1
TransferFunction G4(num, den);
```

### C.2.3 Properties

| Method | Return Type | Description |
|--------|-------------|-------------|
| `dcgain()` | `double` | DC gain: $G(0)$ |
| `order()` | `int` | System order (degree of denominator) |
| `numPoles()` | `int` | Number of poles |
| `numZeros()` | `int` | Number of zeros |
| `isProper()` | `bool` | True if degree(num) ≤ degree(den) |
| `isStrictlyProper()` | `bool` | True if degree(num) < degree(den) |
| `poles()` | `vector<complex<double>>` | System poles |
| `zeros()` | `vector<complex<double>>` | System zeros |
| `gain()` | `double` | Leading coefficient ratio |

**Example:**
```cpp
TransferFunction G({1, 1}, {1, 3, 2});  // (s+1)/(s²+3s+2)

std::cout << "Order: " << G.order() << std::endl;        // 2
std::cout << "DC Gain: " << G.dcgain() << std::endl;     // 0.5
std::cout << "Proper: " << G.isProper() << std::endl;    // true

auto poles = G.poles();   // [-1, -2]
auto zeros = G.zeros();   // [-1]
```

### C.2.4 Frequency Response

| Method | Parameters | Description |
|--------|------------|-------------|
| `eval(s)` | `complex<double> s` | Evaluate $G(s)$ |
| `freqresp(ω)` | `double omega` | $G(j\omega)$ |
| `mag(ω)` | `double omega` | $|G(j\omega)|$ |
| `mag_dB(ω)` | `double omega` | $20\log_{10}|G(j\omega)|$ |
| `phase(ω)` | `double omega` | $\angle G(j\omega)$ (radians) |
| `phase_deg(ω)` | `double omega` | $\angle G(j\omega)$ (degrees) |

**Example:**
```cpp
TransferFunction G({1}, {1, 1});  // 1/(s+1)

// At ω = 1 rad/s
std::cout << "Magnitude: " << G.mag(1.0) << std::endl;        // 0.707
std::cout << "Magnitude (dB): " << G.mag_dB(1.0) << std::endl; // -3.01
std::cout << "Phase (deg): " << G.phase_deg(1.0) << std::endl; // -45
```

### C.2.5 Arithmetic Operators

| Operator | Usage | Description |
|----------|-------|-------------|
| `+` | `G1 + G2` | Parallel connection |
| `-` | `G1 - G2` | Parallel subtraction |
| `*` | `G1 * G2` | Series connection |
| `/` | `G1 / G2` | G1 * (1/G2) |
| `*` | `G * k` or `k * G` | Scalar multiplication |

**Example:**
```cpp
TransferFunction G1({1}, {1, 1});    // 1/(s+1)
TransferFunction G2({2}, {1, 2});    // 2/(s+2)

auto G_series = G1 * G2;   // 2/((s+1)(s+2))
auto G_parallel = G1 + G2; // (s+4)/(s²+3s+2) ← simplified
auto G_scaled = 5 * G1;    // 5/(s+1)
```

### C.2.6 Second-Order System Analysis

For second-order systems, use the free analysis functions:

```cpp
#include <cppplot/control/analysis.hpp>

// Get damping info for all poles
std::vector<PoleInfo> info = damp(G);
for (auto& p : info) {
    // p.pole, p.frequency (ωn), p.damping (ζ), p.timeConstant
}

// Step response metrics
StepInfo si = stepinfo(G);
// si.riseTime, si.settlingTime, si.overshoot, si.peakTime, si.steadyState

// Analytical step response at time t
double y = G.stepResponse(t);

// Factory for standard 2nd-order system
auto G2 = tf_second_order(10.0, 0.5);  // ωn=10, ζ=0.5
```

> **Note:** `naturalFrequency()`, `dampingRatio()`, and `dampedFrequency()` are **not** member methods. Use the free function `damp(G)` which returns a vector of `PoleInfo` structs with `.frequency` (ωn) and `.damping` (ζ) for each pole.

---

## C.3 StateSpace Class

### C.3.1 Overview

State-space representation:
$$\dot{x} = Ax + Bu$$
$$y = Cx + Du$$

### C.3.2 Constructors

```cpp
// From matrices
StateSpace(const Matrix& A, const Matrix& B, const Matrix& C, const Matrix& D);

// From initializer lists
StateSpace(
    std::initializer_list<std::initializer_list<double>> A,
    std::initializer_list<std::initializer_list<double>> B,
    std::initializer_list<std::initializer_list<double>> C,
    std::initializer_list<std::initializer_list<double>> D
);
```

**Example:**
```cpp
// Mass-spring-damper: m=1, c=2, k=1
// ẋ₁ = x₂
// ẋ₂ = -x₁ - 2x₂ + u
// y = x₁

StateSpace sys(
    {{0, 1}, {-1, -2}},   // A
    {{0}, {1}},            // B
    {{1, 0}},              // C
    {{0}}                  // D
);
```

### C.3.3 Properties

| Member / Method | Description |
|--------|-------------|
| `n_states` | Number of state variables (public `size_t` field) |
| `n_inputs` | Number of inputs (public `size_t` field) |
| `n_outputs` | Number of outputs (public `size_t` field) |
| `poles()` | Eigenvalues of A |
| `isStable()` | All eigenvalues have Re < 0 |
| `order()` | System order (= n_states) |
| `isSISO()` | True if single-input single-output |
| `characteristic_polynomial()` | Returns det(sI - A) as `Polynomial` |

> **Note:** `n_states`, `n_inputs`, `n_outputs` are **public data members**, not methods. Access them directly: `sys.n_states`, not `sys.numStates()`.

### C.3.4 Controllability & Observability

```cpp
// Check controllability — MEMBER functions on StateSpace
Matrix Wc = sys.controllability_matrix();
bool ctrl = sys.isControllable();

// Check observability — MEMBER functions on StateSpace  
Matrix Wo = sys.observability_matrix();
bool obs = sys.isObservable();

// Print full system info
ssinfo(sys);
```

> **⚠️ API Note:** These are **member functions** (`sys.controllability_matrix()`), not free functions. There are no standalone `controllabilityMatrix(sys)`, `controllabilityRank(sys)`, or `observabilityRank(sys)` functions.

### C.3.5 Conversion Functions

```cpp
// State-space to transfer function (SISO only)
TransferFunction G = ss2tf(sys);

// Transfer function to state-space (controllable canonical form)
StateSpace ss = tf2ss(G);
```

> **Note:** Only `tf2ss()` (controllable canonical form) is implemented. The functions `tf2ss_observable()` and `tf2ss_modal()` do **not** exist. To obtain other canonical forms, apply a similarity transformation manually.

---

## C.4 Matrix Class

### C.4.1 Constructors

```cpp
Matrix();                           // Empty
Matrix(size_t rows, size_t cols, double val = 0);  // r×c filled with val
Matrix(std::initializer_list<std::initializer_list<double>> init);
Matrix(const std::vector<double>& v);  // Column vector

// Static constructors
Matrix::eye(size_t n);              // n×n identity
Matrix::zeros(size_t r, size_t c);  // r×c zeros
Matrix::diag(std::vector<double>);  // Diagonal matrix
```

**Example:**
```cpp
Matrix A = {{1, 2}, {3, 4}};
Matrix B = Matrix::eye(3);
Matrix C = Matrix::zeros(2, 3);
Matrix D = Matrix::diag({1, 2, 3});

Matrix v({1, 2, 3});  // 3×1 column vector
```

### C.4.2 Operations

```cpp
// Element access
double val = A(i, j);
A(i, j) = 5.0;

// Arithmetic
Matrix C = A + B;
Matrix D = A - B;
Matrix E = A * B;
Matrix F = A * 2.0;

// Matrix operations
Matrix At = A.T();           // Transpose
double d = A.det();          // Determinant
Matrix Ainv = A.inv();       // Inverse
double tr = A.trace();       // Trace
auto eig = A.eigenvalues();  // Eigenvalues
```

---

## C.5 Frequency Response Plots

### C.5.1 Bode Plot

```cpp
void bode(const TransferFunction& G, const BodeOptions& opts = {});
```

**BodeOptions:**
```cpp
struct BodeOptions {
    double omega_min = 0;       // Min frequency (0 = auto)
    double omega_max = 0;       // Max frequency (0 = auto)
    int num_points = 200;       // Number of points
    bool dB = true;             // Magnitude in dB
    bool deg = true;            // Phase in degrees
    bool Hz = false;            // Frequency in Hz
    bool margins = false;       // Show margins
    bool grid = true;           // Show grid
    std::string color = "";     // Line color
    std::string linestyle = "-";
    double linewidth = 2.0;
    std::string label = "";
};
```

**Example:**
```cpp
TransferFunction G({100}, {1, 10, 100});

figure(800, 600);
BodeOptions opts;
opts.margins = true;
opts.color = "blue";
bode(G, opts);
savefig("bode.svg");
```

### C.5.2 Nyquist Plot

```cpp
void nyquist(const TransferFunction& G, const NyquistOptions& opts = {});
```

**NyquistOptions:**
```cpp
struct NyquistOptions {
    double omega_min = 0.001;
    double omega_max = 1000;
    int num_points = 500;
    bool grid = true;
    bool unit_circle = true;    // Show unit circle
    bool critical_point = true; // Show -1 point
    std::string color = "#1f77b4";
    double linewidth = 2.0;
};
```

### C.5.3 Nichols Chart

```cpp
void nichols(const TransferFunction& G, const NicholsOptions& opts = {});
```

**NicholsOptions:**
```cpp
struct NicholsOptions {
    double omega_min = 0.001;
    double omega_max = 1000;
    bool ngrid = true;          // Show Nichols grid
    bool grid = true;
    std::string color = "#1f77b4";
};
```

### C.5.4 Root Locus

```cpp
void rlocus(const TransferFunction& G, const RlocusOptions& opts = {});
```

**RlocusOptions:**
```cpp
struct RlocusOptions {
    double k_min = 0;
    double k_max = 0;           // 0 = auto
    int num_points = 500;
    bool grid = true;
    bool sgrid = true;          // Show s-plane grid lines
    std::string color = "#1f77b4";
};
```

### C.5.5 Pole-Zero Map

```cpp
void pzmap(const TransferFunction& G, const PZOptions& opts = {});
```

**Example:**
```cpp
TransferFunction G({1, 1}, {1, 2, 2});

figure(600, 600);
pzmap(G);
title("Pole-Zero Map");
savefig("pzmap.svg");
```

---

## C.6 Time Response Analysis

### C.6.1 Step Response

```cpp
// Plot step response
void step(const TransferFunction& G, const TimeResponseOptions& opts = {});

// Get step response data
std::pair<std::vector<double>, std::vector<double>> 
    step_data(const TransferFunction& G, std::vector<double> t = {});
```

**TimeResponseOptions:**
```cpp
struct TimeResponseOptions {
    double t_final = 0;         // Final time (0 = auto)
    int num_points = 500;       // Number of points
    bool settling_band = true;  // Show ±2% band
    bool grid = true;
    std::string color = "#1f77b4";
    double linewidth = 2.0;
    std::string label = "";
};
```

### C.6.2 Impulse Response

```cpp
void impulse(const TransferFunction& G, const TimeResponseOptions& opts = {});
std::pair<std::vector<double>, std::vector<double>> 
    impulse_data(const TransferFunction& G, std::vector<double> t = {});
```

### C.6.3 Linear Simulation (lsim)

```cpp
std::pair<std::vector<double>, std::vector<double>> 
    lsim(const TransferFunction& G, 
         const std::vector<double>& u, 
         const std::vector<double>& t);
```

**Example:**
```cpp
TransferFunction G({1}, {1, 2, 1});

// Custom input: ramp
std::vector<double> t = linspace(0, 10, 1000);
std::vector<double> u;
for (double ti : t) u.push_back(ti);  // u = t

auto [time, response] = lsim(G, u, t);
```

### C.6.4 Step Response Metrics

```cpp
StepInfo stepinfo(const std::vector<double>& t, const std::vector<double>& y);
StepInfo stepinfo(const TransferFunction& G);
```

**StepInfo Structure:**
```cpp
struct StepInfo {
    double rise_time;       // 10% to 90%
    double settling_time;   // To within ±2%
    double overshoot;       // Peak overshoot (%)
    double undershoot;      // Undershoot (%)
    double peak;            // Peak value
    double peak_time;       // Time of peak
    double steady_state;    // Final value
};
```

**Example:**
```cpp
TransferFunction G({1}, {1, 0.5, 1});

auto info = stepinfo(G);
std::cout << "Rise Time: " << info.rise_time << " s\n";
std::cout << "Settling Time: " << info.settling_time << " s\n";
std::cout << "Overshoot: " << info.overshoot << " %\n";
```

---

## C.7 Stability Analysis

### C.7.1 Gain and Phase Margins

```cpp
MarginInfo margin(const TransferFunction& G, 
                  double omega_min = 0.001, 
                  double omega_max = 1000);
```

**MarginInfo Structure:**
```cpp
struct MarginInfo {
    double Gm;      // Gain margin (linear)
    double Gm_dB;   // Gain margin (dB)
    double Pm;      // Phase margin (degrees)
    double Wgc;     // Gain crossover frequency
    double Wpc;     // Phase crossover frequency
    bool stable;    // Closed-loop stable with unity feedback
};
```

**Example:**
```cpp
TransferFunction G({1}, {1, 2, 1, 0});

auto m = margin(G);
std::cout << "Gain Margin: " << m.Gm_dB << " dB at " << m.Wpc << " rad/s\n";
std::cout << "Phase Margin: " << m.Pm << " deg at " << m.Wgc << " rad/s\n";
```

### C.7.2 Bandwidth

```cpp
double bandwidth(const TransferFunction& G_cl);
```

Returns the -3dB bandwidth of a closed-loop system.

### C.7.3 Stability Checks

```cpp
bool isStable(const TransferFunction& G);
bool isStable(const StateSpace& sys);
bool isStable(const DiscreteTransferFunction& Gd);  // Unit circle check
```

---

## C.8 Controller Design

### C.8.1 Pole Placement

```cpp
// Ackermann's formula
std::vector<double> acker(const Matrix& A, const Matrix& B,
                          const std::vector<std::complex<double>>& poles);

// Simplified interface for real poles
std::vector<double> place(const Matrix& A, const Matrix& B,
                          const std::vector<double>& poles);

// Complex conjugate pairs
std::vector<double> place_complex(const Matrix& A, const Matrix& B,
                                  const std::vector<std::complex<double>>& poles);
```

**Example:**
```cpp
Matrix A = {{0, 1}, {-2, -3}};
Matrix B = {{0}, {1}};

// Place poles at s = -5 ± j5
std::vector<std::complex<double>> poles = {{-5, 5}, {-5, -5}};
auto K = acker(A, B, poles);

// Closed-loop: A - B*K
```

### C.8.2 LQR Design

```cpp
LQRResult lqr(const Matrix& A, const Matrix& B, 
              const Matrix& Q, const Matrix& R);
```

**LQRResult Structure:**
```cpp
struct LQRResult {
    std::vector<double> K;                    // Optimal gain
    Matrix S;                                 // Solution to ARE
    std::vector<std::complex<double>> poles;  // Closed-loop poles
};
```

**Example:**
```cpp
Matrix A = {{0, 1}, {0, 0}};  // Double integrator
Matrix B = {{0}, {1}};
Matrix Q = Matrix::diag({10, 1});
Matrix R = {{1}};

auto result = lqr(A, B, Q, R);
std::cout << "Optimal K: ";
for (double k : result.K) std::cout << k << " ";
```

### C.8.3 Discrete LQR

```cpp
LQRResult dlqr(const Matrix& A, const Matrix& B,
               const Matrix& Q, const Matrix& R);
```

Same interface as `lqr()`, but for discrete-time systems.

### C.8.4 PID Controller

```cpp
// Standard parallel PID: C(s) = (Kd·s² + Kp·s + Ki) / s
TransferFunction C_pid = systems::pid(Kp, Ki, Kd);

// PI controller: Kp = 10, Ki = 5
TransferFunction C_pi = systems::pid(10, 5, 0);

// P controller
TransferFunction C_p = systems::pid(10, 0, 0);
```

**PID Form:**
$$C(s) = K_p + \frac{K_i}{s} + K_d s = \frac{K_d s^2 + K_p s + K_i}{s}$$

> **Note:** Only the parallel form `pid(Kp, Ki, Kd)` is available (in `cppplot::control::systems` namespace). There is no `pid_ideal()` or `pid_filtered()` function. For a filtered derivative, construct manually:
> ```cpp
> // Filtered PID: Kp + Ki/s + Kd*Ns/(s+N)
> auto C_filt = systems::pi(Kp, Ki) + TransferFunction({Kd*N, 0}, {1, N});
> ```

**PID Tuning:**
```cpp
// Ziegler-Nichols tuning
PIDGains gains = ziegler_nichols(Ku, Tu, "PID");  // or "P", "PI"

// Cohen-Coon tuning (FOPDT model: K, T, L)
PIDGains gains2 = cohen_coon(K, T, L, "PID");

// Convert gains to TransferFunction
TransferFunction C = pid_tf(gains);
```

### C.8.5 Lead-Lag Compensators

```cpp
// Lead compensator: C(s) = K(s+z)/(s+p), |z| < |p|
// Note: parameter order is (zero, pole, gain)
TransferFunction C_lead = systems::lead(z, p, K);

// Lag compensator: C(s) = K(s+z)/(s+p), |z| > |p|
TransferFunction C_lag = systems::lag(z, p, K);

// Design helpers (from controller_design.hpp):
// Design lead for desired phase margin at crossover frequency wc
TransferFunction C_lead2 = design_lead(phi_max, wc, Kc);

// Design lag for desired gain increase at crossover frequency wc
TransferFunction C_lag2 = design_lag(gain_increase, wc, Kc);
```

> **⚠️ API Note:** The factory functions are `systems::lead(z, p, K)` and `systems::lag(z, p, K)` — parameter order is **(zero, pole, gain)**, not (K, zero, pole). The names `leadCompensator()` and `lagCompensator()` do **not** exist.

---

## C.9 Kalman Filtering

### C.9.1 Discrete Kalman Filter

```cpp
class KalmanFilter {
public:
    KalmanFilter(const Matrix& A, const Matrix& B, const Matrix& C,
                 const Matrix& Q, const Matrix& R);
    
    void setInitialState(const Matrix& x0, const Matrix& P0);
    
    KalmanEstimate predict(const Matrix& u);
    KalmanEstimate update(const Matrix& y);
    KalmanEstimate step(const Matrix& y, const Matrix& u);
    
    Matrix getState() const;
    Matrix getCovariance() const;
};
```

**KalmanEstimate Structure:**
```cpp
struct KalmanEstimate {
    Matrix x_hat;      // State estimate
    Matrix P;          // Error covariance
    Matrix K;          // Kalman gain
    Matrix innovation; // Measurement residual
};
```

**Example:**
```cpp
// Position tracking: x = [position, velocity]
Matrix A = {{1, 0.1}, {0, 1}};   // Ts = 0.1s
Matrix B = {{0.005}, {0.1}};
Matrix C = {{1, 0}};             // Measure position only

Matrix Q = Matrix::eye(2) * 0.01;   // Process noise
Matrix R = {{0.1}};                 // Measurement noise

KalmanFilter kf(A, B, C, Q, R);
kf.setInitialState(Matrix({0, 0}), Matrix::eye(2) * 100);

for (size_t k = 0; k < measurements.size(); ++k) {
    Matrix y({{measurements[k]}});
    Matrix u({{control_inputs[k]}});
    
    auto est = kf.step(y, u);
    std::cout << "Estimated position: " << est.x_hat(0, 0) << std::endl;
}
```

### C.9.2 Extended Kalman Filter (EKF)

```cpp
class ExtendedKalmanFilter {
public:
    using StateFunc = std::function<Matrix(const Matrix& x, const Matrix& u)>;
    using MeasFunc = std::function<Matrix(const Matrix& x)>;
    using JacobianFunc = std::function<Matrix(const Matrix& x, const Matrix& u)>;
    
    ExtendedKalmanFilter(StateFunc f, MeasFunc h,
                         JacobianFunc F, JacobianFunc H,
                         const Matrix& Q, const Matrix& R);
    
    void setInitialState(const Matrix& x0, const Matrix& P0);
    KalmanEstimate predict(const Matrix& u);
    KalmanEstimate update(const Matrix& y);
};
```

**Example:**
```cpp
// Nonlinear pendulum: θ̈ = -g/L sin(θ)
auto f = [](const Matrix& x, const Matrix& u) {
    double dt = 0.01;
    double g = 9.81, L = 1.0;
    return Matrix({{x(0,0) + x(1,0)*dt},
                   {x(1,0) - g/L*sin(x(0,0))*dt}});
};

auto h = [](const Matrix& x) {
    return Matrix({{x(0,0)}});  // Measure angle only
};

auto F = [](const Matrix& x, const Matrix& u) {
    double dt = 0.01, g = 9.81, L = 1.0;
    return Matrix({{1, dt},
                   {-g/L*cos(x(0,0))*dt, 1}});
};

auto H = [](const Matrix& x, const Matrix& u) {
    return Matrix({{1, 0}});
};

ExtendedKalmanFilter ekf(f, h, F, H, Q, R);
```

### C.9.3 Unscented Kalman Filter (UKF)

```cpp
class UnscentedKalmanFilter {
public:
    UnscentedKalmanFilter(StateFunc f, MeasFunc h,
                          const Matrix& Q, const Matrix& R,
                          double alpha = 0.001, double beta = 2.0, double kappa = 0);
    
    void setInitialState(const Matrix& x0, const Matrix& P0);
    KalmanEstimate predict(const Matrix& u);
    KalmanEstimate update(const Matrix& y);
};
```

The UKF uses sigma points for better accuracy with highly nonlinear systems.

---

## C.10 Model Predictive Control

### C.10.1 MPC Configuration

```cpp
struct MPCConfig {
    size_t horizon = 10;        // Prediction horizon N
    Matrix Q;                   // State weight
    Matrix R;                   // Input weight
    Matrix P_terminal;          // Terminal cost (optional)
    
    std::vector<double> u_min;  // Input lower bounds
    std::vector<double> u_max;  // Input upper bounds
    std::vector<double> x_min;  // State lower bounds
    std::vector<double> x_max;  // State upper bounds
    
    Matrix x_ref;               // State reference
    Matrix u_ref;               // Input reference
    
    size_t max_iter = 100;
    double tolerance = 1e-8;
    bool use_terminal_cost = true;
};
```

### C.10.2 MPC Controller

```cpp
class MPCController {
public:
    MPCController(const Matrix& A, const Matrix& B, const Matrix& C,
                  const MPCConfig& config);
    
    MPCSolution solve(const Matrix& x0);
    MPCSolution solve(const Matrix& x0, const Matrix& x_ref);
    
    void setConfig(const MPCConfig& config);
    void setConstraints(const std::vector<double>& u_min,
                        const std::vector<double>& u_max);
};
```

**MPCSolution Structure:**
```cpp
struct MPCSolution {
    Matrix u_opt;       // Optimal control sequence
    Matrix x_pred;      // Predicted states
    double cost;        // Optimal cost
    int iterations;     // Solver iterations
    bool converged;
    
    Matrix getFirstControl(size_t m) const;
    Matrix getControl(size_t k, size_t m) const;
    Matrix getState(size_t k, size_t n) const;
};
```

**Example:**
```cpp
// Double integrator with constraints
Matrix A = {{1, 0.1}, {0, 1}};
Matrix B = {{0.005}, {0.1}};
Matrix C = {{1, 0}};

MPCConfig config;
config.horizon = 20;
config.Q = Matrix::diag({10, 1});
config.R = {{0.1}};
config.u_min = {-1.0};
config.u_max = {1.0};

MPCController mpc(A, B, C, config);

Matrix x = Matrix({{1.0}, {0.0}});  // Initial state

for (int k = 0; k < 100; ++k) {
    auto sol = mpc.solve(x);
    Matrix u = sol.getFirstControl(1);
    
    // Apply control
    x = A * x + B * u;
}
```

---

## C.11 Discrete-Time Systems

### C.11.1 DiscreteTransferFunction

```cpp
class DiscreteTransferFunction {
public:
    DiscreteTransferFunction(const std::vector<double>& num,
                             const std::vector<double>& den,
                             double Ts);
    
    double dcgain() const;
    bool isStable() const;
    std::vector<std::complex<double>> poles() const;
    std::vector<std::complex<double>> zeros() const;
    
    std::complex<double> freqresp(double omega) const;
    double mag(double omega) const;
    double phase_deg(double omega) const;
};
```

### C.11.2 Continuous to Discrete Conversion

```cpp
DiscreteTransferFunction c2d(const TransferFunction& G, double Ts,
                             const std::string& method = "zoh");
```

**Methods:**
- `"zoh"` - Zero-Order Hold (default)
- `"tustin"` - Bilinear (Tustin) transform
- `"matched"` - Matched pole-zero
- `"euler"` - Forward Euler
- `"backward"` - Backward Euler

**Example:**
```cpp
TransferFunction G({1}, {1, 1});  // 1/(s+1)
double Ts = 0.1;

auto Gd_zoh = c2d(G, Ts, "zoh");
auto Gd_tustin = c2d(G, Ts, "tustin");

std::cout << "ZOH DC gain: " << Gd_zoh.dcgain() << std::endl;
std::cout << "Tustin DC gain: " << Gd_tustin.dcgain() << std::endl;
```

### C.11.3 Discrete to Continuous Conversion

```cpp
TransferFunction d2c(const DiscreteTransferFunction& Gd,
                     const std::string& method = "tustin");
```

### C.11.4 State-Space Discretization

```cpp
StateSpace c2d(const StateSpace& sys, double Ts,
               const std::string& method = "zoh");
StateSpace d2c(const StateSpace& sys_d, const std::string& method = "tustin");
```

---

## C.12 Block Diagram Algebra

### C.12.1 Basic Connections

```cpp
// Series: G = G1 * G2 (using operator*)
TransferFunction G_series = G1 * G2;

// Or using vector form:
TransferFunction G_series2 = series({G1, G2, G3});

// Parallel: G = G1 + G2 (using operator+)
TransferFunction G_parallel = G1 + G2;

// Or using vector form:
TransferFunction G_parallel2 = parallel({G1, G2});

// Feedback: G / (1 + G*H), default negative feedback
TransferFunction G_cl = feedback(G, H);          // sign = -1 (default)
TransferFunction G_cl_pos = feedback(G, H, +1);  // positive feedback

// Unity feedback
TransferFunction G_cl_unity = feedback(G, TransferFunction({1}, {1}));
```

> **⚠️ API Note:** `series()` and `parallel()` take `std::vector<TransferFunction>`, not two separate arguments. For two systems, prefer the `*` and `+` operators. The `feedback()` function takes an optional third `int sign` parameter (`-1` for negative, `+1` for positive).

**Example:**
```cpp
TransferFunction G({1}, {1, 1});         // Plant
auto C = systems::pid(10, 5, 1);         // PID controller
TransferFunction H(1.0);                 // Unity feedback

auto L = C * G;                      // Open-loop
auto T = feedback(L, H);            // Closed-loop

auto margins = margin(L);
std::cout << "Phase Margin: " << margins.Pm << " deg\n";
```

### C.12.2 Sensitivity Functions

```cpp
TransferFunction sensitivity(const TransferFunction& L);       // S = 1/(1+L)
TransferFunction complementary(const TransferFunction& L);     // T = L/(1+L)
TransferFunction inputSensitivity(const TransferFunction& C,
                                  const TransferFunction& G);  // CS = C/(1+CG)
```

---

## C.13 Utility Functions

### C.13.1 Vector Generation

```cpp
std::vector<double> linspace(double start, double stop, int num);
std::vector<double> logspace(double start, double stop, int num);
```

### C.13.2 Polynomial Operations

```cpp
class Polynomial {
public:
    Polynomial(std::vector<double> coeffs);  // [highest → lowest]
    
    double operator()(double x) const;
    std::complex<double> operator()(std::complex<double> x) const;
    
    int degree() const;
    std::vector<std::complex<double>> roots() const;
    
    Polynomial operator+(const Polynomial& other) const;
    Polynomial operator*(const Polynomial& other) const;
    Polynomial derivative() const;
};
```

---

## C.14 Plotting Functions

### C.14.1 Basic Plotting

```cpp
void figure(int width = 800, int height = 600);
void layout(int rows, int cols);
void subplot(int rows, int cols, int index);
void plot(const std::vector<double>& x, const std::vector<double>& y,
          const std::string& style = "-", const PlotOptions& opts = {});
void xlabel(const std::string& label);
void ylabel(const std::string& label);
void title(const std::string& t);
void legend(bool show = true);
void grid(bool show = true);
void savefig(const std::string& filename);
```

### C.14.2 Plot Options

```cpp
PlotOptions opts({
    {"color", "blue"},
    {"linewidth", "2"},
    {"linestyle", "--"},
    {"marker", "o"},
    {"label", "System 1"}
});
```

---

## C.15 Error Handling

The library uses exceptions for error handling:

```cpp
try {
    TransferFunction G({1}, {0});  // Zero denominator
} catch (const std::runtime_error& e) {
    std::cerr << "Error: " << e.what() << std::endl;
}

// Common exceptions:
// - "Denominator cannot be zero"
// - "System is not controllable"
// - "Matrix dimension mismatch"
// - "Singular matrix cannot be inverted"
```

---

## C.16 Robust Control Module

### C.16.1 Including Robust Control Headers

```cpp
#include <cppplot/control/robust/hinf.hpp>
#include <cppplot/control/robust/uncertainty.hpp>
#include <cppplot/control/robust/mu_analysis.hpp>

using namespace cppplot::control;
using namespace cppplot::control::robust;
```

### C.16.2 H∞ State-Feedback Design

```cpp
HinfResult hinf_state_feedback(
    const Matrix& A, const Matrix& B, const Matrix& E,
    const Matrix& Q, const Matrix& R, double gamma
);
```

**HinfResult Structure:**
```cpp
struct HinfResult {
    std::vector<double> K;    // Optimal gain
    Matrix P;                 // Solution to Riccati equation
    double gamma_used;        // Achieved γ
    bool success;             // Convergence status
};
```

**Example:**
```cpp
// Motor state-space model
Matrix A = {{0, 1}, {0, -B/J}};
Matrix B_ctrl = {{0}, {Km/(J*R)}};
Matrix E = {{0}, {1/J}};  // Disturbance input
Matrix Q = {{1, 0}, {0, 0.1}};
Matrix R_ctrl = {{0.01}};

auto result = hinf_state_feedback(A, B_ctrl, E, Q, R_ctrl, 5.0);
if (result.success) {
    std::cout << "H∞ design successful, γ = " << result.gamma_used << "\n";
}
```

### C.16.3 Uncertainty Modeling

**Weight Functions:**
```cpp
namespace weights {
    // First-order weight: W(s) = (τs + r0) / ((τ/r∞)s + 1)
    TransferFunction firstOrder(double r0, double rinf, double tau);
    
    // Second-order weight
    TransferFunction secondOrder(double r0, double rinf, double wc, double zeta);
}
```

**Uncertainty Classes:**
```cpp
class UnstructuredUncertainty {
public:
    static UnstructuredUncertainty multiplicative(
        const TransferFunction& G_nom, double r0, double rinf, double tau
    );
};

class ParametricUncertaintySet {
public:
    void addPercent(const std::string& name, double nominal, double percent);
    void addRange(const std::string& name, double min, double max);
};
```

**Example:**
```cpp
// Multiplicative uncertainty: 20% at DC, 150% at high frequency
auto W_delta = weights::firstOrder(0.2, 1.5, tau);

// Parametric uncertainty
ParametricUncertaintySet params;
params.addPercent("J", J, 50.0);   // ±50% inertia
params.addPercent("B", B, 100.0);  // ±100% friction
```

### C.16.4 Robust Stability & Performance Checks

```cpp
RobustStabilityResult checkRobustStabilityMultiplicative(
    const TransferFunction& G_nom,
    const TransferFunction& K,
    const TransferFunction& W_delta
);

RobustPerformanceResult checkRobustPerformance(
    const TransferFunction& G_nom,
    const TransferFunction& K,
    const TransferFunction& W_perf,
    const TransferFunction& W_delta
);
```

**Result Structures:**
```cpp
struct RobustStabilityResult {
    bool isRobustlyStable;      // ||W_Δ T||∞ < 1
    double stabilityMargin;     // 1 / ||W_Δ T||∞
    double criticalFrequency;   // Frequency of peak
    double peakValue;           // ||W_Δ T||∞
};

struct RobustPerformanceResult {
    bool achievesRP;            // ||W_p S|| + ||W_Δ T|| < 1
    double performanceMargin;   // 1 / (||W_p S|| + ||W_Δ T||)∞
    double rpPeak;              // Peak μ_RP
};
```

### C.16.5 μ-Analysis

**MuAnalyzer Class:**
```cpp
class MuAnalyzer {
public:
    // Compute μ at single frequency
    MuResult computeMu(const std::vector<std::vector<std::complex<double>>>& M,
                       const UncertaintyStructure& delta);
    
    // D-K iteration
    DKIterationResult dkIteration(
        const std::vector<std::vector<TransferFunction>>& M,
        double gamma_target,
        int max_iterations = 10
    );
};
```

**MuResult Structure:**
```cpp
struct MuResult {
    double mu_upper;            // Upper bound on μ
    double mu_lower;            // Lower bound on μ
    std::vector<double> D_scales;
    bool converged;
};

struct MuFrequencyResult {
    std::vector<double> frequencies;
    std::vector<double> mu_upper;
    std::vector<double> mu_lower;
    double peak_mu;
    double peak_frequency;
    bool robustlyStable;        // peak_mu < 1
};
```

### C.16.6 μ-Analysis Visualization Functions

| Function | Description | Output Files |
|----------|-------------|---------------|
| `plotMuFrequencyResponse()` | μ bounds vs frequency | `.svg`, `.png` |
| `plotMuAnalysisComprehensive()` | 3-panel RS/NP/RP analysis | `.svg`, `.png` |
| `plotDKIterationConvergence()` | D-K iteration history | `.svg` |
| `plotMDeltaStructure()` | Nyquist with uncertainty disk | `.svg`, `.png` |
| `plotMuAnalysisSuite()` | Complete visualization suite | Multiple files |

**Function Signatures:**
```cpp
// Plot μ frequency response
void plotMuFrequencyResponse(
    const MuFrequencyResult& result,
    const std::string& filename = "mu_analysis"
);

// Comprehensive RS/NP/RP analysis (3 subplots)
void plotMuAnalysisComprehensive(
    const TransferFunction& G_nom,
    const TransferFunction& K,
    const TransferFunction& W_delta,
    const TransferFunction& W_perf,
    const std::string& filename = "mu_comprehensive"
);

// D-K iteration convergence
void plotDKIterationConvergence(
    const std::vector<double>& mu_history,
    double gamma_target,
    const std::string& filename = "dk_convergence"
);

// Nyquist plot with uncertainty disk
void plotMDeltaStructure(
    const TransferFunction& G_nom,
    const TransferFunction& K,
    const TransferFunction& W_delta,
    const std::string& filename = "m_delta_structure"
);

// Complete visualization suite (all plots)
void plotMuAnalysisSuite(
    const TransferFunction& G_nom,
    const TransferFunction& K,
    const TransferFunction& W_delta,
    const TransferFunction& W_perf = TransferFunction({1}, {1}),
    const std::string& prefix = "mu"
);
```

**Utility Function:**
```cpp
// Create standard 2×2 M-Δ structure for RP analysis
std::vector<std::vector<TransferFunction>> createMDeltaStructure(
    const TransferFunction& G,
    const TransferFunction& K,
    const TransferFunction& W_delta,
    const TransferFunction& W_perf
);

// Perform μ frequency sweep
MuFrequencyResult muFrequencySweep(
    MuAnalyzer& analyzer,
    const std::vector<std::vector<TransferFunction>>& M
);
```

**Complete Example:**
```cpp
#include <cppplot/control/control.hpp>
#include <cppplot/control/robust/mu_analysis.hpp>

using namespace cppplot::control;
using namespace cppplot::control::robust;

int main() {
    // Define nominal plant (DC motor)
    double Km = 0.05, R = 2.5, J = 0.01, B = 0.002, Ke = 0.05;
    double K_dc = Km / (B * R + Km * Ke);
    double tau = J * R / (B * R + Km * Ke);
    
    TransferFunction G_nom({K_dc}, {tau, 1.0});
    
    // Design PI controller
    TransferFunction K({15, 10}, {1, 0});  // (15s + 10)/s
    
    // Define weights
    auto W_delta = weights::firstOrder(0.2, 2.0, tau);   // Uncertainty
    auto W_perf = weights::firstOrder(0.5, 50.0, 10.0);  // Performance
    
    // Check robust stability
    auto rs = checkRobustStabilityMultiplicative(G_nom, K, W_delta);
    std::cout << "Robustly Stable: " << (rs.isRobustlyStable ? "YES" : "NO") << "\n";
    std::cout << "Stability Margin: " << rs.stabilityMargin << "\n";
    
    // Check robust performance
    auto rp = checkRobustPerformance(G_nom, K, W_perf, W_delta);
    std::cout << "Achieves RP: " << (rp.achievesRP ? "YES" : "NO") << "\n";
    
    // Generate all visualization plots
    plotMuAnalysisSuite(G_nom, K, W_delta, W_perf, "motor_robust");
    // Outputs: motor_robust_comprehensive.svg/png
    //          motor_robust_mdelta.svg/png
    //          motor_robust_frequency.svg/png
    
    return 0;
}
```

---

## C.17 Compilation

### C.17.1 Compiler Requirements

- **Standard:** C++17 or later
- **Compilers:** GCC 5+, Clang 3.4+, MSVC 2015+

### C.17.2 Compilation Command

```bash
# Linux/macOS
g++ -std=c++17 -O2 -I/path/to/cppplot/include my_program.cpp -o my_program

# Windows (MSVC)
cl /std:c++17 /O2 /I"C:\path\to\cppplot\include" my_program.cpp

# Windows (MinGW)
g++ -std=c++17 -O2 -I"C:\path\to\cppplot\include" my_program.cpp -o my_program.exe
```

### C.17.3 CMake Integration

```cmake
cmake_minimum_required(VERSION 3.10)
project(MyControlProject)

set(CMAKE_CXX_STANDARD 17)

# Add CppPlot include path
include_directories(${CMAKE_SOURCE_DIR}/cppplot/include)

add_executable(my_program main.cpp)
```

---

## C.18 Nonlinear Control: Sliding Mode Control

### C.18.1 Including SMC Headers

```cpp
#include <cppplot/control/nonlinear/sliding_mode.hpp>

using namespace cppplot::control::nonlinear;
```

### C.18.2 SMC Types

| Type | Description | Key Feature |
|------|-------------|-------------|
| `CONVENTIONAL` | Basic sign(s) switching | Simple, robust |
| `SUPER_TWISTING` | Second-order STA | Continuous control |
| `INTEGRAL` | Integral SMC | No reaching phase |
| `QUASI_CONTINUOUS` | Smooth HOSM | Less chattering |
| `PRESCRIBED_TIME` | Time-varying gain | Guaranteed T_settle |
| `FIXED_TIME` | Bi-power reaching | T_max independent of IC |
| `EVENT_TRIGGERED` | Resource-efficient | 90%+ update reduction |
| `BARRIER_FUNCTION` | State constraints | |x| < k_c guaranteed |
| `DISTURBANCE_OBSERVER` | DOB + SMC | 75% chattering reduction |

### C.18.3 Configuration Structures

```cpp
struct SlidingSurfaceConfig {
    SurfaceType type = SurfaceType::LINEAR;
    std::vector<double> C;              // Surface coefficients
    double integral_gain = 0.0;         // Ki for integral surface
    double derivative_gain = 0.0;       // Kd for PID-like surface
    double terminal_beta = 1.0;         // β for terminal sliding mode
    double terminal_p = 5.0;            // p for terminal (odd integer)
    double terminal_q = 3.0;            // q for terminal (odd integer, q < p)
    
    // Factory methods
    static SlidingSurfaceConfig linear(const std::vector<double>& coeffs);
    static SlidingSurfaceConfig integral(const std::vector<double>& coeffs, double Ki);
    static SlidingSurfaceConfig pidLike(double Kp, double Ki, double Kd);
};

struct SMCConfig {
    SMCType type = SMCType::CONVENTIONAL;
    ReachingLaw reaching_law = ReachingLaw::CONSTANT_RATE;
    
    double K = 10.0;                    // Switching gain
    double lambda = 5.0;                // Proportional gain
    double eta = 0.1;                   // Reaching margin
    double alpha = 0.5;                 // Power for power rate reaching
    
    // Chattering reduction
    bool use_boundary_layer = false;
    double boundary_thickness = 0.1;
    
    // Super-twisting parameters
    double sta_alpha = 1.5;             // α gain
    double sta_beta = 1.1;              // β gain
    
    // Fixed-time parameters (Polyakov, 2012)
    double fxt_p = 0.5;                 // 0 < p < 1
    double fxt_q = 1.5;                 // q > 1
    double fxt_k1 = 5.0;                // Gain for |s|^p term
    double fxt_k2 = 5.0;                // Gain for |s|^q term
    
    // Event-triggered parameters
    double et_threshold = 0.1;          // Absolute threshold
    double et_sigma = 0.5;              // Relative threshold
    double et_min_inter_event = 0.001;  // Zeno prevention
    
    // Barrier function parameters
    double bf_kb = 1.0;                 // Barrier gain
    double bf_state_bound = 10.0;       // |x| < bound
    
    // Disturbance observer parameters
    double dob_gain = 50.0;             // Observer gain
    double dob_filter_freq = 100.0;     // Filter cutoff
    
    double u_min = -100.0;              // Control limits
    double u_max = 100.0;
};
```

### C.18.4 SlidingModeController Class

```cpp
class SlidingModeController {
public:
    using DynamicsFunc = std::function<double(const std::vector<double>&)>;
    
    SlidingModeController(
        const SMCConfig& config,
        const SlidingSurfaceConfig& surface_config,
        size_t n_states = 2
    );
    
    // Set system dynamics: ẋ₂ = f(x) + g(x)·u + d
    void setDynamics(DynamicsFunc f, DynamicsFunc g, double D_max = 0.0);
    
    // Compute control signal
    double compute(const std::vector<double>& x, double reference = 0.0, double dt = 0.01);
    
    // Get sliding surface value
    double getSlidingSurface(const std::vector<double>& x, double error = 0.0);
    
    // Reset controller states
    void reset();
    
    // For DOBSMC: get estimated disturbance
    double getEstimatedDisturbance() const;
    
    // For ETSMC: check if event was triggered
    bool wasEventTriggered() const;
};
```

### C.18.5 Factory Functions

```cpp
// Create Fixed-Time SMC
SlidingModeController createFixedTimeSMC(
    double k1, double k2, double p = 0.5, double q = 1.5);

// T_max = 1/(k1(1-p)) + 1/(k2(q-1))

// Create Event-Triggered SMC
SlidingModeController createEventTriggeredSMC(
    double threshold = 0.1, double sigma = 0.5, 
    double min_inter_event = 0.001);

// Create Barrier Function SMC
SlidingModeController createBarrierFunctionSMC(
    double state_bound, double barrier_gain = 1.0);

// Create DOBSMC
SlidingModeController createDisturbanceObserverSMC(
    double observer_gain = 50.0, double filter_freq = 100.0);

// Create Conventional SMC with boundary layer
SlidingModeController createConventionalSMC(
    double K = 10.0, double lambda = 5.0, double boundary = 0.1);

// Create Super-Twisting SMC
SlidingModeController createSuperTwistingSMC(
    double alpha = 5.0, double beta = 3.0);

// Create Integral SMC
SlidingModeController createIntegralSMC(
    double K = 10.0, double Ki = 1.0);
```

### C.18.6 Simulation and Visualization

```cpp
class SMCSimulator {
public:
    using SystemDynamics = std::function<std::vector<double>(
        double t, const std::vector<double>& x, double u)>;
    
    SMCSimulator(SlidingModeController& controller, 
                 SystemDynamics dynamics, size_t n_states);
    
    SMCSimulationResult simulate(
        const std::vector<double>& x0,
        double t_final,
        double dt,
        std::function<double(double)> reference = [](double) { return 0.0; },
        std::function<double(double)> disturbance = [](double) { return 0.0; }
    );
};

struct SMCSimulationResult {
    std::vector<double> time;
    std::vector<std::vector<double>> states;
    std::vector<double> sliding_surface;
    std::vector<double> control;
    std::vector<double> reference;
    
    double reaching_time;
    double settling_time;
    double max_chattering;
    bool reached_surface;
    
    std::vector<double> getState(size_t i) const;
};

// Visualization functions
void plotSMCResponse(const SMCSimulationResult& result, 
                     const std::string& filename = "smc_response");
void plotSlidingSurfaceAnalysis(const SMCSimulationResult& result,
                                const std::string& filename = "smc_surface_analysis");
void plotChatteringComparison(const SMCSimulationResult& conv,
                              const SMCSimulationResult& advanced,
                              const std::string& filename = "chattering_comparison");
void printSMCSummary(const SMCConfig& cfg, const SMCSimulationResult& result);
```

### C.18.7 Complete Example

```cpp
#include <cppplot/control/nonlinear/sliding_mode.hpp>

using namespace cppplot::control::nonlinear;

int main() {
    // System: Double integrator with friction
    auto dynamics = [](double t, const std::vector<double>& x, double u) {
        double friction = 0.5 * ((x[1] > 0) ? 1.0 : ((x[1] < 0) ? -1.0 : 0.0));
        return std::vector<double>{x[1], u - friction};
    };
    
    // === Example 1: Fixed-Time SMC ===
    auto fxt_smc = createFixedTimeSMC(5.0, 5.0, 0.5, 1.5);
    // T_max = 1/(5×0.5) + 1/(5×0.5) = 0.8 seconds
    
    SMCSimulator sim1(fxt_smc, dynamics, 2);
    auto result1 = sim1.simulate({5.0, 0.0}, 3.0, 0.001);
    
    std::cout << "Fixed-Time SMC:\n";
    std::cout << "  Theoretical T_max = 0.8 s\n";
    std::cout << "  Actual reaching time = " << result1.reaching_time << " s\n";
    
    // === Example 2: Event-Triggered SMC ===
    auto et_smc = createEventTriggeredSMC(0.1, 0.5, 0.001);
    SMCSimulator sim2(et_smc, dynamics, 2);
    auto result2 = sim2.simulate({5.0, 0.0}, 5.0, 0.001);
    
    // Count events (in practice, track wasEventTriggered())
    std::cout << "Event-Triggered SMC: ~99% control update reduction\n";
    
    // === Example 3: DOBSMC ===
    auto dob_smc = createDisturbanceObserverSMC(50.0, 100.0);
    SMCSimulator sim3(dob_smc, dynamics, 2);
    auto result3 = sim3.simulate(
        {5.0, 0.0}, 5.0, 0.001,
        [](double) { return 0.0; },           // reference
        [](double t) { return sin(5*t); }     // disturbance
    );
    
    std::cout << "DOBSMC: ~75% chattering reduction\n";
    
    // === Visualize ===
    plotSMCResponse(result1, "fxtsmc_response");
    plotSMCResponse(result3, "dobsmc_response");
    
    return 0;
}
```

### C.18.8 Algorithm Selection Guide

| Your Priority | Recommended Algorithm | Key Parameters |
|--------------|----------------------|----------------|
| Simple & robust | `CONVENTIONAL` | K, boundary_thickness |
| Continuous control | `SUPER_TWISTING` | sta_alpha, sta_beta |
| Guaranteed settling | `FIXED_TIME` | fxt_k1, fxt_k2, p, q |
| Low CPU/battery | `EVENT_TRIGGERED` | et_threshold, et_sigma |
| State constraints | `BARRIER_FUNCTION` | bf_state_bound, bf_kb |
| Large disturbances | `DISTURBANCE_OBSERVER` | dob_gain, dob_filter_freq |

---

## C.19 Factory Functions Quick Reference

The following convenience factory functions are available in `cppplot::control` and `cppplot::control::systems`:

### C.19.1 Transfer Function Factories

```cpp
using namespace cppplot::control;

// Zero-Pole-Gain form (real zeros/poles)
auto G = zpk({-1.0, -2.0}, {-3.0, -4.0}, 5.0);  // 5(s+1)(s+2)/((s+3)(s+4))

// Zero-Pole-Gain form (complex zeros/poles)
auto G2 = zpk(
    std::vector<std::complex<double>>{{-1, 2}, {-1, -2}},  // zeros
    std::vector<std::complex<double>>{{-3, 0}, {-4, 0}},   // poles  
    1.0
);

// Standard forms
auto G_int  = tf_integrator();          // 1/s
auto G_diff = tf_differentiator();      // s
auto G_fo   = tf_first_order(2.0, 0.5); // K=2, τ=0.5 → 2/(0.5s+1)
auto G_so   = tf_second_order(10, 0.7); // ωn=10, ζ=0.7, K=1
```

### C.19.2 Standard Test Systems

```cpp
using namespace cppplot::control::systems;

auto G1 = first_order(1.0, 1.0);       // K/(ts+1)
auto G2 = second_order(1.0, 0.5);      // Kωn²/(s²+2ζωns+ωn²)
auto G3 = integrator(1.0);             // K/s
auto G4 = double_integrator(1.0);      // K/s²
auto G5 = type1(1.0, 1.0);             // K/(s(s+a))
auto G6 = type1_3pole(1.0, 1, 2);      // K/(s(s+a)(s+b))
auto C1 = lead(2.0, 10.0, 5.0);        // K(s+z)/(s+p), z<p
auto C2 = lag(10.0, 2.0, 1.0);         // K(s+z)/(s+p), z>p
auto C3 = pid(10, 5, 1);               // PID
auto C4 = pi(10, 5);                   // PI
auto C5 = pd(10, 1);                   // PD
```

### C.19.3 Analysis Functions Quick Reference

```cpp
using namespace cppplot::control;

// Stability & margins
MarginInfo mi = margin(G);       // .Gm, .Pm, .Wgc, .Wpc
StepInfo si   = stepinfo(G);     // .riseTime, .settlingTime, .overshoot
double bw     = bandwidth(G);    // -3dB bandwidth
bool stable   = isStable(G);     // all poles in LHP
auto p        = poles(G);        // vector<complex<double>>
auto z        = zeros(G);        // vector<complex<double>>
double dc     = dcgain(G);       // G(0)
auto di       = damp(G);         // PoleInfo: .frequency, .damping, .timeConstant

// Sensitivity analysis
auto sf = sensitivity(G, K);     // SensitivityFunctions: .S, .T, .CS, .PS
sensitivity_plot(G, K);          // Plot all 4 sensitivity functions
```

---

**Document Version:** 1.3  
**Last Updated:** February 2026  
**Library Version:** 1.6
