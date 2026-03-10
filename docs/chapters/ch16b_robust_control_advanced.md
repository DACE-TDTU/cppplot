# Chapter 16B: Advanced Robust Control - H∞ Synthesis and Uncertainty Modeling

---

## 🎯 Learning Objectives

This chapter provides advanced robust control theory and implementation using the CppPlot library:
- H∞ controller synthesis (state-feedback and output-feedback)
- Structured and unstructured uncertainty modeling
- μ-analysis for robust stability and performance
- Application to differential drive mobile robot motor control

---

## 16b.1 Introduction to Advanced Robust Control

### 16b.1.1 Beyond Classical Design

While classical gain/phase margins provide basic robustness, modern robust control offers:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     ROBUST CONTROL FRAMEWORK                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌──────────────┐                                                         │
│   │ Performance  │──► W_p ──►┐                                             │
│   │   Specs      │           │                                             │
│   └──────────────┘           │    ┌────────────────────┐                   │
│                              ├───►│                    │                   │
│   ┌──────────────┐           │    │   H∞ / μ          │───► Controller K  │
│   │ Uncertainty  │──► W_Δ ──►├───►│   Synthesis       │                   │
│   │   Model      │           │    │                    │                   │
│   └──────────────┘           │    └────────────────────┘                   │
│                              │                                             │
│   ┌──────────────┐           │                                             │
│   │ Plant Model  │──► G_nom─►┘                                             │
│   │   G(s)       │                                                         │
│   └──────────────┘                                                         │
│                                                                             │
│   KEY BENEFITS:                                                            │
│   • Systematic handling of multiple uncertainty sources                    │
│   • Guaranteed stability and performance bounds                            │
│   • Frequency-dependent uncertainty specification                          │
│   • Optimal trade-off between performance and robustness                   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 16b.1.2 Differential Drive Mobile Robot Application

We apply robust control to wheel motor speed control in a differential drive robot:

```
                    DIFFERENTIAL DRIVE ROBOT
                    
            ┌─────────────────────────────────┐
            │                                 │
      ┌─────┤         Robot Body              ├─────┐
      │     │                                 │     │
      │     │       ω_L          ω_R          │     │
    ┌─┴─┐   │     ┌───┐        ┌───┐         │   ┌─┴─┐
    │ L │◄──┼─────│ M │        │ M │─────────┼──►│ R │
    │   │   │     └───┘        └───┘         │   │   │
    └───┘   │      Left         Right        │   └───┘
    Left    │     Motor         Motor        │   Right
    Wheel   │                                │   Wheel
            └─────────────────────────────────┘
    
    Kinematics:
    • Linear velocity:  v = (v_R + v_L) / 2
    • Angular velocity: ω = (v_R - v_L) / L
    
    Motor Model (each wheel):
    • DC Motor with gear reduction
    • Load inertia varies with terrain and payload
    • Friction varies with speed and temperature
```

**Challenges requiring robust control:**
1. Payload variation (±50% mass uncertainty)
2. Terrain changes (friction coefficient varies 2x-5x)
3. Battery voltage drop (affects motor torque constant)
4. Temperature effects on motor resistance
5. Wheel slip on different surfaces

---

## 16b.2 H∞ Control Theory

### 16b.2.1 The H∞ Norm

The H∞ norm measures the maximum gain of a system across all frequencies:

$$\|G\|_\infty = \sup_{\omega} \bar{\sigma}(G(j\omega)) = \sup_{\omega} |G(j\omega)| \text{ (SISO)}$$

**Physical interpretation:**
- Maximum amplification from input to output
- Worst-case gain for any sinusoidal input
- Energy-based: $\|G\|_\infty = \sup_{u \neq 0} \frac{\|y\|_2}{\|u\|_2}$

### 16b.2.2 Standard H∞ Problem

**Generalized Plant Formulation:**

```
                    ┌─────────────────────┐
          w ────────►                     ├──────► z
                    │         P           │
          u ────────►                     ├──────► y
                    └─────────────────────┘
                              │
                              │y
                    ┌─────────┴──────────┐
                    │         K          │
                    └─────────┬──────────┘
                              │u
```

State-space representation of P:
$$\begin{align}
\dot{x} &= Ax + B_1w + B_2u \\
z &= C_1x + D_{11}w + D_{12}u \\
y &= C_2x + D_{21}w + D_{22}u
\end{align}$$

**H∞ Problem:** Find K(s) such that:
1. The closed-loop is internally stable
2. $\|T_{zw}\|_\infty < \gamma$ for specified $\gamma > 0$

### 16b.2.3 H∞ State-Feedback Synthesis

For full-state feedback (y = x), the H∞ problem reduces to solving the **H∞ Algebraic Riccati Equation (ARE)**:

$$A^TP + PA - P\left(B_2R^{-1}B_2^T - \gamma^{-2}B_1B_1^T\right)P + C_1^TC_1 = 0$$

**Conditions for existence:**
1. $(A, B_2)$ stabilizable
2. $(C_1, A)$ detectable
3. $\gamma > \gamma_{opt}$ (minimum achievable H∞ norm)

**State-feedback gain:**
$$K = R^{-1}B_2^TP$$

### 16b.2.4 γ-Iteration

The optimal $\gamma_{opt}$ is found via bisection:

```
Algorithm: γ-Iteration
─────────────────────────────────────────
Input: System matrices A, B, C, Q, R
       Bounds γ_lb, γ_ub
       Tolerance ε

γ_lo ← γ_lb
γ_hi ← γ_ub

while (γ_hi - γ_lo > ε):
    γ_mid ← (γ_lo + γ_hi) / 2
    
    if H∞_ARE_solvable(γ_mid):
        γ_hi ← γ_mid
        K_best ← solve_ARE(γ_mid)
    else:
        γ_lo ← γ_mid
    
return γ_opt = γ_hi, K = K_best
```

### 16b.2.5 H∞ Output-Feedback

When only output y (not full state x) is available, we need a dynamic controller:

$$\begin{align}
\dot{x}_K &= A_Kx_K + B_Ky \\
u &= C_Kx_K + D_Ky
\end{align}$$

This requires solving **two coupled Riccati equations**:

**Control ARE (for state-feedback):**
$$A^TX + XA + C_1^TC_1 - X\left(B_2R_1^{-1}B_2^T - \gamma^{-2}B_1B_1^T\right)X = 0$$

**Filtering ARE (for observer):**
$$AY + YA^T + B_1B_1^T - Y\left(C_2^TR_2^{-1}C_2 - \gamma^{-2}C_1^TC_1\right)Y = 0$$

**Spectral radius condition:**
$$\rho(XY) < \gamma^2$$

### 16b.2.6 Mixed-Sensitivity Design

For SISO systems, practical H∞ design uses mixed-sensitivity:

$$\min_K \left\| \begin{bmatrix} W_1S \\ W_2KS \\ W_3T \end{bmatrix} \right\|_\infty$$

where:
- $S = \frac{1}{1+GK}$ - Sensitivity (tracking)
- $KS = \frac{K}{1+GK}$ - Control sensitivity
- $T = \frac{GK}{1+GK}$ - Complementary sensitivity (robustness)

**Weight selection:**

| Weight | Purpose | Typical Form |
|--------|---------|--------------|
| $W_1(s)$ | Low-freq tracking | $\frac{s/M + \omega_b}{s + \omega_bA}$ |
| $W_2(s)$ | Control effort limit | Constant or low-pass |
| $W_3(s)$ | High-freq rolloff | $\frac{s + \omega_b/M}{As + \omega_b}$ |

---

## 16b.3 Uncertainty Modeling

### 16b.3.1 Parametric Uncertainty

When parameters are known to lie within bounds:

$$p \in [p_{nom}(1-\delta_p), p_{nom}(1+\delta_p)]$$

**For DC Motor:**
```
Parameter          Nominal    Uncertainty    Physical Cause
─────────────────────────────────────────────────────────────
Resistance R       2.5 Ω      ±40%          Temperature variation
Inductance L       5 mH       ±30%          Magnetic saturation
Torque constant    0.5 Nm/A   ±10%          Manufacturing tolerance
Inertia J          0.01 kg·m² ±50%          Payload variation
Friction B         0.001      ±100%         Wear, lubrication
Back-EMF Ke        0.5 V·s/rad ±10%         Temperature, field weakening
```

### 16b.3.2 Multiplicative Uncertainty

The most common unstructured uncertainty model:

$$G_p(s) = G_{nom}(s)\left(1 + W_\Delta(s)\Delta(s)\right), \quad |\Delta(j\omega)| \leq 1$$

**Weight selection:**
$$|W_\Delta(j\omega)| \geq \max_{G_p \in \mathcal{G}} \left|\frac{G_p(j\omega) - G_{nom}(j\omega)}{G_{nom}(j\omega)}\right|$$

**Standard first-order weight:**
$$W_\Delta(s) = \frac{\tau s + r_0}{(\tau/r_\infty)s + 1}$$

where:
- $r_0$ = relative uncertainty at DC (e.g., 0.2 = 20%)
- $r_\infty$ = relative uncertainty at high frequency (e.g., 2.0 = 200%)
- $\tau$ = transition time constant

### 16b.3.3 Additive Uncertainty

Alternative representation:
$$G_p(s) = G_{nom}(s) + W_A(s)\Delta(s)$$

Useful when absolute (not relative) uncertainty is known.

### 16b.3.4 Uncertainty Visualization

```
                     MULTIPLICATIVE UNCERTAINTY
                     
     |G|                                      |W_Δ|
      ↑                                         ↑
      │    ┌──── Uncertainty envelope           │  r_∞ ─────────────
      │   /│\                                   │       /
      │  / │ \   G_p = G(1 + W_Δ·Δ)            │      /
      │ /  │  \                                 │     /
      │/   │   \                                │    /
     G────────────                              │   /
      │\   │   /                               │  /
      │ \  │  /                                │ /
      │  \ │ /                                 │/
      │   \│/                              r_0 ├─────
      │                                        │
      └───────────────► ω                      └───────────────► ω
               ω_b                                      1/τ
```

---

## 16b.4 Robust Stability Analysis

### 16b.4.1 Small Gain Theorem

**Theorem:** For the feedback interconnection of M and Δ:

```
       ┌───────────────────┐
  ──►(+)───►│       M       │───┬──►
     - ▲    └───────────────┘   │
       │                        │
       │    ┌───────────────┐   │
       └────┤       Δ       │◄──┘
            └───────────────┘
```

The system is stable for all $\|\Delta\|_\infty \leq 1$ if and only if:
$$\|M\|_\infty < 1$$

### 16b.4.2 Robust Stability Conditions

**For multiplicative uncertainty:**
$$\|W_\Delta T\|_\infty < 1$$

where $T = \frac{GK}{1+GK}$ is the complementary sensitivity.

**For additive uncertainty:**
$$\|W_A KS\|_\infty < 1$$

where $KS = \frac{K}{1+GK}$ is the control sensitivity.

### 16b.4.3 Stability Margin

The **robustness margin** indicates how much additional uncertainty can be tolerated:

$$b = \frac{1}{\|W_\Delta T\|_\infty}$$

- $b > 1$: System robustly stable (can tolerate $b$ times the modeled uncertainty)
- $b < 1$: System NOT robustly stable

---

## 16b.5 Robust Performance

### 16b.5.1 Definition

**Robust Performance (RP):** The system achieves performance specifications for ALL plants in the uncertainty set.

**Performance specification:**
$$\|W_pS\|_\infty < 1$$

**Combined RS + NP (Conservative):**
$$\|W_\Delta T\|_\infty + \|W_pS\|_\infty < 1$$

### 16b.5.2 Exact RP Condition

For multiplicative uncertainty:
$$\|W_pS\| + |W_\Delta T| < 1 \quad \forall \omega$$

This is equivalent to:
$$\mu_\Delta(M) < 1$$

where μ is the structured singular value.

---

## 16b.6 Structured Singular Value (μ)

### 16b.6.1 Definition

For a matrix M and uncertainty structure Δ:

$$\mu_\Delta(M) = \frac{1}{\min\{\bar{\sigma}(\Delta) : \det(I - M\Delta) = 0, \Delta \in \boldsymbol{\Delta}\}}$$

**Interpretation:**
- $\mu < 1$ ⟹ Robust stability/performance achieved
- $\mu$ = smallest uncertainty magnitude that can destabilize

### 16b.6.2 Upper Bound via D-Scaling

$$\mu(M) \leq \inf_{D \in \mathcal{D}} \bar{\sigma}(DMD^{-1})$$

where D commutes with the uncertainty structure.

### 16b.6.3 D-K Iteration

Algorithm for μ-synthesis:

```
D-K Iteration Algorithm
───────────────────────────────────────────
1. Initialize D(jω) = I

2. K-step: Fix D, design K to minimize
   ||D·M(K)·D⁻¹||∞
   (This is an H∞ problem)

3. D-step: Fix K, minimize over D at each frequency
   min_D σ̄(D·M·D⁻¹)
   
4. Fit rational D(s) to frequency-domain D(jω)

5. If converged or max_iter reached, STOP
   Else, go to step 2

Output: Controller K, achieved μ bound
```

### 16b.6.4 μ-Analysis Visualization with CppPlot

The `mu_analysis.hpp` module provides comprehensive plotting functions for μ-analysis visualization:

#### plotMuFrequencyResponse()

Plots μ upper/lower bounds versus frequency with stability boundary indicator:

```cpp
#include <cppplot/control/robust/mu_analysis.hpp>

// Perform μ frequency sweep
MuAnalyzer analyzer;
auto M = createMDeltaStructure(G_nom, K, W_delta, W_perf);
auto freq_result = muFrequencySweep(analyzer, M);

// Generate frequency response plot
plotMuFrequencyResponse(freq_result, "mu_frequency_plot");
// Outputs: mu_frequency_plot.svg, mu_frequency_plot.png
```

**Output includes:**
- μ upper bound (solid blue line)
- μ lower bound (dashed cyan line)
- Stability boundary at μ = 1 (red dashed line)
- Peak μ marker with annotation

#### plotMuAnalysisComprehensive()

Creates a 3-subplot comprehensive analysis showing RS, NP, and RP conditions:

```cpp
// Plot comprehensive μ-analysis
plotMuAnalysisComprehensive(
    G_nom,          // Nominal plant
    K,              // Controller
    W_delta,        // Uncertainty weight
    W_perf,         // Performance weight
    "comprehensive_analysis"
);
```

**Subplots:**
1. **|W_Δ T|** - Robust Stability condition with peak value
2. **|W_p S|** - Nominal Performance condition with peak value
3. **μ_RP = |W_Δ T| + |W_p S|** - Robust Performance with peak marker

Each subplot shows:
- Frequency response magnitude
- Critical boundary at 1
- Pass/Fail indicator (✓/✗)

#### plotDKIterationConvergence()

Visualizes D-K iteration convergence history:

```cpp
// After running D-K iteration
auto dk_result = analyzer.dkIteration(M, gamma_target, max_iterations);

// Plot convergence
plotDKIterationConvergence(
    dk_result.mu_history,   // μ values at each iteration
    gamma_target,           // Target γ
    "dk_convergence"
);
```

**Shows:**
- Achieved μ at each iteration (blue line with markers)
- Target γ level (green dashed line)
- RP boundary at μ = 1 (red dotted line)

#### plotMDeltaStructure()

Creates Nyquist-like visualization with uncertainty disk:

```cpp
// Plot M-Δ structure
plotMDeltaStructure(
    G_nom,       // Nominal plant
    K,           // Controller
    W_delta,     // Uncertainty weight
    "m_delta_plot"
);
```

**Visualization includes:**
- Nyquist curve of L(jω) = G(jω)K(jω)
- Uncertainty disk at critical frequency
- Critical point (-1, 0) marked with red X
- Equal aspect ratio for proper geometry

#### plotMuAnalysisSuite()

One-call function that generates all μ-analysis visualizations:

```cpp
// Generate complete visualization suite
plotMuAnalysisSuite(
    G_nom,          // Nominal plant  
    K,              // Controller
    W_delta,        // Uncertainty weight
    W_perf,         // Performance weight (optional)
    "motor_mu"      // Output filename prefix
);

// Generates:
// - motor_mu_comprehensive.svg/png
// - motor_mu_mdelta.svg/png  
// - motor_mu_frequency.svg/png
```

#### Complete Example

```cpp
#include <cppplot/control/control.hpp>
#include <cppplot/control/robust/mu_analysis.hpp>

using namespace cppplot::control;
using namespace cppplot::control::robust;

int main() {
    // Define nominal plant (DC motor)
    double Km = 0.05, R = 2.5, J = 0.01, B_fric = 0.002, Ke = 0.05;
    double K_dc = Km / (B_fric * R + Km * Ke);
    double tau = J * R / (B_fric * R + Km * Ke);
    
    TransferFunction G_nom({K_dc}, {tau, 1.0});
    
    // Design PI controller
    double Kp = 15.0, Ki = 10.0;
    TransferFunction K({Kp, Ki}, {1.0, 0.0});
    
    // Uncertainty weight: 20% at DC, 200% at high frequency
    auto W_delta = weights::firstOrder(0.2, 2.0, 1.0/tau);
    
    // Performance weight: good tracking up to 10 rad/s
    auto W_perf = weights::firstOrder(0.5, 50.0, 10.0);
    
    // Generate all μ-analysis visualizations
    plotMuAnalysisSuite(G_nom, K, W_delta, W_perf, "dc_motor_robust");
    
    return 0;
}
```

### 16b.6.5 M-Δ Structure Utility Function

The `createMDeltaStructure()` helper builds the standard 2×2 M matrix for robust performance analysis:

$$M = \begin{bmatrix} W_\Delta T & W_\Delta TG \\ W_pS & W_pSG \end{bmatrix}$$

```cpp
// Create M-Δ structure for RP analysis
auto M = createMDeltaStructure(G_nom, K, W_delta, W_perf);

// M is a 2x2 matrix of TransferFunctions:
// M[0][0] = W_Δ * T
// M[0][1] = W_Δ * T * G  
// M[1][0] = W_p * S
// M[1][1] = W_p * S * G
```

This structure is used with `muFrequencySweep()` for computing the structured singular value across frequencies.

---

## 16b.7 Kharitonov's Theorem

### 16b.7.1 Interval Polynomials

For polynomial with interval coefficients:
$$p(s) = a_ns^n + a_{n-1}s^{n-1} + \cdots + a_1s + a_0$$
$$a_i \in [a_i^-, a_i^+]$$

### 16b.7.2 Kharitonov's Theorem

**Theorem:** The interval polynomial family is Hurwitz stable if and only if the following four Kharitonov polynomials are Hurwitz stable:

$$K_1(s) = a_0^- + a_1^-s + a_2^+s^2 + a_3^+s^3 + a_4^-s^4 + \cdots$$
$$K_2(s) = a_0^+ + a_1^+s + a_2^-s^2 + a_3^-s^3 + a_4^+s^4 + \cdots$$
$$K_3(s) = a_0^+ + a_1^-s + a_2^-s^2 + a_3^+s^3 + a_4^+s^4 + \cdots$$
$$K_4(s) = a_0^- + a_1^+s + a_2^+s^2 + a_3^-s^3 + a_4^-s^4 + \cdots$$

**Pattern:** Alternating min/max in groups of 2.

---

## 16b.8 Application: Differential Drive Motor Control

### 16b.8.1 DC Motor Model

The transfer function from voltage to angular velocity:

$$G(s) = \frac{\omega(s)}{V(s)} = \frac{K_m}{(Js+B)(Ls+R) + K_mK_e}$$

**Simplified (ignoring electrical dynamics L ≈ 0):**
$$G(s) = \frac{K_m/R}{Js + B + K_mK_e/R} = \frac{K}{τs + 1}$$

where:
- $K = \frac{K_m}{BR + K_mK_e}$ — DC gain
- $τ = \frac{JR}{BR + K_mK_e}$ — Time constant

### 16b.8.2 Uncertainty Sources in Mobile Robot

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                 UNCERTAINTY IN DIFFERENTIAL DRIVE ROBOT                     │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  1. PAYLOAD VARIATION                                                       │
│     ─────────────────                                                       │
│     • Empty robot: J_min = 0.005 kg·m²                                     │
│     • Full load:   J_max = 0.015 kg·m²                                     │
│     • Uncertainty: ±50% from nominal                                       │
│                                                                             │
│  2. TERRAIN FRICTION                                                        │
│     ────────────────                                                        │
│     • Smooth floor:  B = 0.001 N·m·s/rad                                   │
│     • Carpet:        B = 0.003 N·m·s/rad                                   │
│     • Gravel:        B = 0.01 N·m·s/rad                                    │
│     • Uncertainty:   Factor of 2-10x                                       │
│                                                                             │
│  3. BATTERY STATE                                                           │
│     ─────────────                                                           │
│     • Full:   V = 12.6V, K_m = 0.05 Nm/A                                   │
│     • Low:    V = 10.5V, K_m reduces ~15%                                  │
│     • Effect: DC gain varies ±20%                                          │
│                                                                             │
│  4. TEMPERATURE                                                             │
│     ───────────                                                             │
│     • Cold (0°C):  R increases 20%                                         │
│     • Hot (50°C):  R decreases 10%                                         │
│     • Effect: Time constant varies ±15%                                    │
│                                                                             │
│  5. WHEEL SLIP                                                              │
│     ──────────                                                              │
│     • Dry surface:  No slip                                                │
│     • Wet surface:  Up to 30% slip                                         │
│     • Effect: Effective gain reduction                                     │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 16b.8.3 Nominal Model Parameters

For a typical small differential drive robot (e.g., TurtleBot-style):

| Parameter | Symbol | Nominal Value | Unit |
|-----------|--------|---------------|------|
| Motor resistance | R | 2.5 | Ω |
| Motor inductance | L | 5 | mH |
| Torque constant | $K_m$ | 0.05 | Nm/A |
| Back-EMF constant | $K_e$ | 0.05 | V·s/rad |
| Rotor inertia | $J_m$ | 0.001 | kg·m² |
| Wheel+gear inertia | $J_w$ | 0.005 | kg·m² |
| Gear ratio | n | 30 | - |
| Viscous friction | B | 0.002 | Nm·s/rad |
| Wheel radius | r | 0.05 | m |
| Robot mass | m | 5 | kg |

**Effective reflected inertia:**
$$J_{eff} = J_m + \frac{J_w + mr^2/2}{n^2} \approx 0.01 \text{ kg·m}^2$$

### 16b.8.4 Control Objectives

| Specification | Requirement | Rationale |
|---------------|-------------|-----------|
| Rise time | $t_r < 0.2$ s | Fast response for agile navigation |
| Overshoot | $M_p < 10\%$ | Smooth motion, prevent wheel slip |
| Settling time | $t_s < 0.5$ s | Quick stabilization |
| Steady-state error | $e_{ss} < 2\%$ | Accurate trajectory following |
| Disturbance rejection | $< 5\%$ deviation | Handle bumps, slopes |
| Robustness | Stable for all uncertainty | Reliable operation |

---

## 16b.9 CppPlot Implementation

### 16b.9.1 Including Headers

```cpp
#include <cppplot/control/control.hpp>
#include <cppplot/control/robust/hinf.hpp>
#include <cppplot/control/robust/uncertainty.hpp>
#include <cppplot/control/robust/mu_analysis.hpp>

using namespace cppplot::control;
using namespace cppplot::control::robust;
```

### 16b.9.2 H∞ State-Feedback Design

```cpp
// Motor state-space model (2nd order: position and velocity)
// State: x = [θ, ω]^T
// ẋ = Ax + Bu + Ew
// y = Cx

Matrix A = {{0, 1},           // ω̇ appears in second row
            {0, -B/J}};       // Simplified: -B/J*ω + Km/(J*R)*V

Matrix B = {{0},
            {Km/(J*R)}};      // Control input (voltage)

Matrix E = {{0},
            {1/J}};           // Disturbance (load torque)

Matrix Q = {{1, 0},           // Position weight
            {0, 0.1}};        // Velocity weight

Matrix R = {{0.01}};          // Control effort weight

double gamma = 5.0;           // Target H∞ bound

auto result = hinf_state_feedback(A, B, E, Q, R, gamma);

if (result.success) {
    std::cout << "H∞ State-Feedback Design Successful\n";
    std::cout << "Gain K = [" << result.K(0,0) << ", " << result.K(0,1) << "]\n";
    std::cout << "Achieved γ = " << result.gamma_used << "\n";
}
```

### 16b.9.3 Uncertainty Modeling

```cpp
// Define nominal plant
double Km = 0.05, R = 2.5, J = 0.01, B = 0.002, Ke = 0.05;
double K_dc = Km / (B*R + Km*Ke);
double tau = J*R / (B*R + Km*Ke);

TransferFunction G_nom({K_dc}, {tau, 1.0});

// Create multiplicative uncertainty model
// 20% at DC, growing to 150% at high frequency
auto unc = UnstructuredUncertainty::multiplicative(G_nom, 0.2, 1.5, tau);

// Or use parametric uncertainty
ParametricUncertaintySet params;
params.addPercent("J", J, 50.0);      // ±50% inertia
params.addPercent("B", B, 100.0);     // ±100% friction
params.addPercent("R", R, 40.0);      // ±40% resistance
```

### 16b.9.4 Robust Stability Check

```cpp
// Design PI controller
double Kp = 10.0, Ki = 5.0;
TransferFunction K({Kp, Ki}, {1.0, 0.0});

// Uncertainty weight
auto W_delta = weights::firstOrder(0.2, 1.5, 1.0/tau);

// Check robust stability
auto rs = checkRobustStabilityMultiplicative(G_nom, K, W_delta);

std::cout << "Robustly Stable: " << (rs.isRobustlyStable ? "YES" : "NO") << "\n";
std::cout << "Stability Margin: " << rs.stabilityMargin << "\n";
std::cout << "Critical Frequency: " << rs.criticalFrequency << " rad/s\n";
```

### 16b.9.5 Mixed-Sensitivity Design

```cpp
// Performance weight: good tracking up to 10 rad/s
auto W_perf = weights::firstOrder(0.5, 50.0, 10.0);

// Uncertainty weight
auto W_delta = weights::firstOrder(0.2, 1.5, 5.0);

// Check robust performance
auto rp = checkRobustPerformance(G_nom, K, W_perf, W_delta);

if (rp.achievesRP) {
    std::cout << "Robust Performance Achieved!\n";
    std::cout << "Performance Margin: " << rp.performanceMargin << "\n";
} else {
    std::cout << "Need to redesign controller\n";
}
```

### 16b.9.6 μ-Analysis Visualization API

The `mu_analysis.hpp` module provides visualization functions using CppPlot:

```cpp
#include <cppplot/control/robust/mu_analysis.hpp>

// --- Individual Plot Functions ---

// 1. Plot μ frequency response with bounds
plotMuFrequencyResponse(
    freq_result,         // MuFrequencyResult from muFrequencySweep()
    "output_filename"    // Output file prefix
);

// 2. Comprehensive 3-panel μ-analysis (RS, NP, RP)
plotMuAnalysisComprehensive(
    G_nom,       // Nominal plant TF
    K,           // Controller TF
    W_delta,     // Uncertainty weight TF
    W_perf,      // Performance weight TF
    "filename"   // Output file prefix
);

// 3. D-K iteration convergence plot
plotDKIterationConvergence(
    mu_history,      // std::vector<double> of μ at each iteration
    gamma_target,    // Target γ value
    "filename"       // Output file prefix
);

// 4. Nyquist plot with uncertainty disk
plotMDeltaStructure(
    G_nom,       // Nominal plant TF
    K,           // Controller TF
    W_delta,     // Uncertainty weight TF
    "filename"   // Output file prefix
);

// --- Complete Visualization Suite ---

// Generate all μ-analysis plots with one call
plotMuAnalysisSuite(
    G_nom,       // Nominal plant
    K,           // Controller  
    W_delta,     // Uncertainty weight
    W_perf,      // Performance weight (optional, defaults to 1)
    "prefix"     // Output filename prefix
);

// --- Utility Functions ---

// Create M-Δ structure for RP analysis
auto M = createMDeltaStructure(G_nom, K, W_delta, W_perf);
// Returns: 2x2 matrix of TransferFunctions

// Perform μ frequency sweep
MuAnalyzer analyzer;
auto freq_result = muFrequencySweep(analyzer, M);
// Returns: MuFrequencyResult with frequencies, mu_upper, mu_lower, peak info
```

**Function Signatures:**

| Function | Output Files | Description |
|----------|--------------|-------------|
| `plotMuFrequencyResponse()` | `.svg`, `.png` | μ bounds vs frequency |
| `plotMuAnalysisComprehensive()` | `.svg`, `.png` | 3-panel RS/NP/RP analysis |
| `plotDKIterationConvergence()` | `.svg` | D-K iteration history |
| `plotMDeltaStructure()` | `.svg`, `.png` | Nyquist with uncertainty disk |
| `plotMuAnalysisSuite()` | Multiple files | All of the above |

---

## 16b.10 Design Guidelines

### 16b.10.1 Systematic Design Procedure

```
ROBUST CONTROLLER DESIGN WORKFLOW
═══════════════════════════════════════════════════════════════

Step 1: MODEL THE PLANT
        ├── Identify nominal model G_nom(s)
        ├── Characterize parametric uncertainties
        └── Define frequency-dependent uncertainty W_Δ(s)

Step 2: SPECIFY PERFORMANCE
        ├── Desired bandwidth ω_b
        ├── Tracking accuracy (→ W_p low-freq gain)
        ├── Disturbance rejection requirements
        └── Control effort limits (→ W_u)

Step 3: INITIAL DESIGN
        ├── Start with classical design (PID, lead-lag)
        ├── Check nominal performance
        └── Verify basic stability margins

Step 4: ROBUST ANALYSIS
        ├── Check: ||W_Δ T||_∞ < 1 (RS)
        ├── Check: ||W_p S||_∞ < 1 (NP)
        ├── Check: ||W_p S|| + ||W_Δ T|| < 1 (RP)
        └── If any fails, iterate design

Step 5: H∞/μ SYNTHESIS (if needed)
        ├── Formulate generalized plant P
        ├── Run H∞ or D-K iteration
        ├── Verify achieved γ
        └── Simplify controller if possible

Step 6: VALIDATION
        ├── Time-domain simulation with uncertainty samples
        ├── Monte Carlo analysis
        ├── Experimental validation
        └── Final robustness verification
```

### 16b.10.2 Trade-offs

| Increase | Effect on RS | Effect on NP | Trade-off |
|----------|--------------|--------------|-----------|
| Bandwidth ω_b | Worse (higher T at high freq) | Better tracking | RS ↔ NP |
| Controller gain | Worse (amplifies uncertainty) | Better | RS ↔ NP |
| Uncertainty bound | Harder to achieve RS | Unchanged | Conservative |

### 16b.10.3 Practical Tips

1. **Start conservative:** Use generous uncertainty bounds initially
2. **Iterate:** Refine uncertainty model based on experiments
3. **Validate:** Test on real hardware with parameter variations
4. **Simplify:** Reduce controller order for implementation
5. **Monitor:** Include health monitoring for out-of-spec conditions

---

### 📋 Signal Dictionary — H∞ / μ-Synthesis / MRAC

| Signal | Symbol | Meaning | Typical unit |
|--------|--------|---------|--------------|
| Generalized plant | $P(s)$ | Augmented plant with weights, uncertainty channels | — |
| Exogenous input | $\mathbf{w}$ | Disturbances, references, noise entering the system | mixed |
| Regulated output | $\mathbf{z}$ | Performance variables to be minimized ($\|z\|$ = cost) | mixed |
| H∞ norm | $\|T_{zw}\|_\infty$ | Worst-case energy gain from $w$ to $z$ — the number H∞ minimizes | — |
| Performance level | $\gamma$ | Achieved $\|T_{zw}\|_\infty < \gamma$; smaller = better | — |
| Structured uncertainty | $\boldsymbol{\Delta} = \text{diag}(\delta_1 I, \ldots, \Delta_k)$ | Block-diagonal uncertainty with known structure | — |
| Structured singular value | $\mu(M)$ | $\mu < 1 \Rightarrow$ robust performance — tightest stability test for structured $\Delta$ | — |
| D-scaling matrix | $D(s)$ | Frequency-dependent scaling in D-K iteration | — |
| Reference model (MRAC) | $x_m, \dot{x}_m = A_m x_m + B_m r$ | Desired closed-loop behavior — the "target" | state units |
| Adaptive parameters | $\hat{\theta}(t)$ | Time-varying controller gains adjusted online | varies |
| Tracking error (MRAC) | $e = x - x_m$ | Gap between plant and reference model | state units |
| Lyapunov function | $V(e, \tilde{\theta})$ | Energy-like function that MUST decrease — proves adaptation works | — |
| Adaptation rate | $\Gamma$ | How fast parameters adjust; too large = oscillation, too small = slow | — |

> **Key insight:** H∞ and μ-synthesis answer the question "what is the worst case?" — they optimize for the adversary. MRAC answers "what if I adapt?" — it adjusts to the plant online. These are complementary philosophies: robust control hedges against the worst, adaptive control learns from the actual.

---

## 16b.11 Exercises

**E15b.1 (MRAC Adaptation Law)**
Consider a first-order plant $\dot{x} = a_p x + b_p u$ where $a_p$ and $b_p$ are unknown but $\text{sign}(b_p)$ is known. The reference model is $\dot{x}_m = a_m x_m + b_m r$ with $a_m < 0$.

(a) Propose a Model Reference Adaptive Control (MRAC) law of the form $u = \hat{\theta}_1 x + \hat{\theta}_2 r$.

(b) Derive the MIT-rule adaptation laws for $\hat{\theta}_1$ and $\hat{\theta}_2$ using the error $e = x - x_m$.

(c) What are the ideal values of $\theta_1^*$ and $\theta_2^*$ in terms of $a_p, b_p, a_m, b_m$?

---

**E15b.2 (Parameter Convergence)**
For the MRAC system in E15b.1:

(a) Show that the tracking error $e(t) \to 0$ as $t \to \infty$ using a Lyapunov argument.

(b) Does parameter convergence ($\hat{\theta} \to \theta^*$) necessarily follow from tracking error convergence? Explain.

(c) Under what additional condition does parameter convergence occur?

---

**E15b.3 (Persistent Excitation)**
(a) Define persistent excitation (PE) for a signal $r(t)$.

(b) Give an example of a reference signal $r(t)$ that is PE and one that is not.

(c) Explain why PE is required for parameter convergence in MRAC but not for tracking error convergence.

(d) For a second-order plant, what is the minimum number of distinct frequencies required in $r(t)$ to ensure PE?

---

**E15b.4 (Lyapunov-Based Adaptation)**
Consign the system $\dot{x} = -x + \theta^* u$ where $\theta^*$ is unknown, with control law $u = \hat{\theta} x_{ref}$ and adaptation law $\dot{\hat{\theta}} = \gamma e x_{ref}$.

(a) Choose a Lyapunov function $V(e, \tilde{\theta}) = \frac{1}{2}e^2 + \frac{1}{2\gamma}\tilde{\theta}^2$ where $\tilde{\theta} = \hat{\theta} - \theta^*$.

(b) Show that $\dot{V} \leq 0$ and conclude stability.

(c) What does the choice of $\gamma$ (adaptation gain) affect? What are the trade-offs between fast and slow adaptation?

---

**E15b.5 (Adaptive vs. Robust Control Comparison)**
For the uncertain plant $G(s) = \frac{K}{s + a}$ with $K \in [0.5, 2]$ and $a \in [0.5, 1.5]$:

(a) Design a robust controller (e.g., using the methods from Chapter 16) that stabilizes all plants in the uncertainty set.

(b) Design an MRAC controller that adapts to the unknown parameters.

(c) Compare the two approaches in terms of: transient performance, steady-state performance, computational complexity, and behavior when uncertainty bounds are exceeded.

---

**E15b.6 (Implementation with cppplot)**
Implement a simple MRAC system using cppplot for the plant $\dot{x} = a_p x + b_p u$ with:
- True parameters: $a_p = -1$, $b_p = 2$
- Reference model: $a_m = -3$, $b_m = 3$
- Adaptation gain: $\gamma = 5$
- Reference input: $r(t) = \sin(t) + 0.5\sin(3t)$ (PE condition)

(a) Simulate for 30 seconds and plot: tracking error $e(t)$, parameter estimates $\hat{\theta}_1(t), \hat{\theta}_2(t)$.

(b) Repeat with $r(t) = 1$ (constant, not PE). Compare parameter convergence.

(c) At $t = 15$ s, change $a_p$ from $-1$ to $-2$. How does the adaptive controller respond vs. a fixed robust controller?

---

**E15b.7 🔴 (Level 3 — D-K Iteration and Its Limitations)**
You run D-K iteration for a system with 3 uncertain parameters and achieve $\mu < 1$ after 5 iterations.

(a) The D-scale fitting order increases from 2 to 8 across iterations. What does this mean for the final controller order? Is this practical for a microcontroller?

(b) After iteration 3, $\mu$ starts *increasing* instead of decreasing. What went wrong? (Hint: D-fitting accuracy.)

(c) A colleague suggests using $\mu$ upper bound from iteration 3 and stopping. Under what conditions is this conservative but safe?

**E15b.8 🔴 (Level 3 — MRAC vs. Robust: The Right Tool)**
A plant has $a_p \in [-3, -1]$ (slow variation over hours due to temperature).

(a) Design an H∞ controller for the worst-case $a_p = -1$. Compute the performance at nominal $a_p = -2$.

(b) Design an MRAC that adapts to the actual $a_p$. Simulate with $a_p$ ramping from $-1$ to $-3$ over 60 seconds.

(c) Compare: H∞ gives guaranteed performance in a *band*; MRAC gives asymptotically perfect tracking but has *transients* during adaptation. For a chemical reactor (slow, safety-critical), which do you choose? For a communication channel (fast, non-safety-critical)?

**E15b.9 ⚫ (Level 4 — The Certainty Equivalence Lie)**

(a) MRAC replaces unknown parameters with estimates and acts as if the estimates are correct. This is called "certainty equivalence." Under what conditions does this work? When does it fail catastrophically?

(b) In the Kalman filter (Ch. 15), certainty equivalence is *optimal* for linear-Gaussian systems. In MRAC, it is only justified by Lyapunov analysis. What is the difference? Why does Gaussianity matter?

(c) Write a one-paragraph argument for or against the following claim: "Adaptive control is always better than robust control because it uses more information." Use counterexamples.

---

## 16b.12 Summary

| Concept | Key Formula/Condition |
|---------|----------------------|
| H∞ norm | $\|G\|_\infty = \max_\omega \|G(j\omega)\|$ |
| H∞ ARE | $A^TP + PA - P(BR^{-1}B^T - \gamma^{-2}EE^T)P + Q = 0$ |
| Multiplicative uncertainty | $G_p = G(1 + W_\Delta\Delta)$ |
| Robust stability | $\|W_\Delta T\|_\infty < 1$ |
| Robust performance | $\|W_pS\| + \|W_\Delta T\| < 1$ |
| μ-synthesis | $\mu_\Delta(M) < 1$ |

### CppPlot μ-Analysis Visualization API

| Function | Purpose | Output |
|----------|---------|--------|
| `plotMuFrequencyResponse()` | μ bounds vs frequency | `.svg`, `.png` |
| `plotMuAnalysisComprehensive()` | RS/NP/RP 3-panel analysis | `.svg`, `.png` |
| `plotDKIterationConvergence()` | D-K iteration history | `.svg` |
| `plotMDeltaStructure()` | Nyquist + uncertainty disk | `.svg`, `.png` |
| `plotMuAnalysisSuite()` | Complete visualization | Multiple files |
| `createMDeltaStructure()` | Build M-Δ structure | 2×2 TF matrix |
| `muFrequencySweep()` | Compute μ over frequency | `MuFrequencyResult` |

---

## References

1. Zhou, K., Doyle, J.C., & Glover, K. (1996). *Robust and Optimal Control*. Prentice Hall.
2. Skogestad, S., & Postlethwaite, I. (2005). *Multivariable Feedback Control: Analysis and Design*. Wiley.
3. Doyle, J.C., Francis, B.A., & Tannenbaum, A.R. (1992). *Feedback Control Theory*. Macmillan.
4. Green, M., & Limebeer, D.J.N. (1995). *Linear Robust Control*. Prentice Hall.
5. Packard, A., & Doyle, J.C. (1993). "The Complex Structured Singular Value". *Automatica*, 29(1), 71-109.

---

*See the accompanying example files:*
- `robust_motor_control_case_study.cpp` - Complete robust control implementation
- `hinf_motor_cppplot_demo.cpp` - H∞ design with CppPlot visualization
- `mu_analysis_demo.cpp` - μ-analysis visualization examples
