# Chapter 17b: Sliding Mode Control - Comprehensive Guide

---

## 🎯 Learning Objectives and Outcomes

### Chapter Objectives

This chapter provides an in-depth treatment of Sliding Mode Control (SMC), covering mathematical foundations, design methodology, advanced algorithms, and practical implementation. Students will learn to design, analyze, and implement SMC controllers for robust control of uncertain nonlinear systems.

### Prerequisites
- Chapter 17: Introduction to Nonlinear Control
- Lyapunov stability theory
- State-space representation
- Basic differential equations

---

## Why This Chapter Matters: The Quest for Robustness

> **The Real Engineering Problem:** How can we design a controller that maintains performance despite model uncertainties, parameter variations, and external disturbances? Sliding Mode Control provides a mathematically elegant solution with **guaranteed robustness** to a class of uncertainties.

### The Power of Variable Structure Control

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    WHY SLIDING MODE CONTROL?                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   CONVENTIONAL CONTROL LIMITATIONS:                                         │
│   ─────────────────────────────────                                        │
│   • PID: Requires accurate model for tuning                                │
│   • LQR: Optimal only for the nominal model                                │
│   • H∞: Conservative, may be slow                                          │
│                                                                             │
│   SLIDING MODE ADVANTAGES:                                                  │
│   ─────────────────────────                                                │
│   ✓ Invariant to matched uncertainties                                     │
│   ✓ Finite-time convergence                                                │
│   ✓ Simple structure, easy implementation                                  │
│   ✓ Order reduction on sliding surface                                     │
│   ✓ Decoupling of dynamics                                                 │
│                                                                             │
│   APPLICATIONS:                                                             │
│   ─────────────                                                            │
│   • Robotics & manipulators                                                │
│   • Electric drives & power converters                                     │
│   • Aerospace & flight control                                             │
│   • Automotive systems                                                      │
│   • Process control                                                         │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Learning Outcomes

After completing this chapter, students will be able to:

| Bloom's Level | Outcome |
|---------------|---------|
| **Remember** | Define sliding surface, reaching condition, equivalent control |
| **Understand** | Explain the two phases of SMC: reaching and sliding |
| **Apply** | Design sliding surfaces and switching control laws |
| **Analyze** | Analyze stability using Lyapunov methods for SMC |
| **Evaluate** | Compare different SMC algorithms for specific applications |
| **Create** | Implement advanced SMC algorithms including Fixed-Time and DOBSMC |

---

## 17b.1 Mathematical Foundations of Sliding Mode Control

### 17b.1.1 System Model and Problem Formulation

Consider a nonlinear uncertain system in canonical form:

$$\dot{\mathbf{x}} = \mathbf{f}(\mathbf{x}, t) + \mathbf{B}(\mathbf{x}, t)\mathbf{u} + \mathbf{d}(\mathbf{x}, t)$$

where:
- $\mathbf{x} \in \mathbb{R}^n$ is the state vector
- $\mathbf{u} \in \mathbb{R}^m$ is the control input
- $\mathbf{f}(\mathbf{x}, t)$ is the nominal dynamics
- $\mathbf{B}(\mathbf{x}, t)$ is the input matrix
- $\mathbf{d}(\mathbf{x}, t)$ represents matched uncertainties and disturbances

**Assumption (Matched Uncertainty):** The disturbance $\mathbf{d}$ lies in the column space of $\mathbf{B}$, i.e., $\mathbf{d} = \mathbf{B}\mathbf{\delta}$ for some bounded $\mathbf{\delta}$.

For a SISO second-order system (most common in applications):

$$\ddot{x} = f(x, \dot{x}) + g(x)\cdot u + d(t)$$

**State-space form:**
$$\dot{x}_1 = x_2$$
$$\dot{x}_2 = f(\mathbf{x}) + g(\mathbf{x})u + d(t)$$

where $x_1$ is position, $x_2$ is velocity.

### 17b.1.2 Sliding Surface Design

The **sliding surface** (or switching manifold) is defined as:

$$s(\mathbf{x}) = 0$$

#### Linear Sliding Surface

For a second-order system tracking error $e = x - x_d$:

$$s = \dot{e} + \lambda e = \dot{x} - \dot{x}_d + \lambda(x - x_d)$$

where $\lambda > 0$ is the slope of the sliding surface.

**Physical Interpretation:**
- $s = 0$ defines a line in the phase plane $(e, \dot{e})$
- On this line: $\dot{e} = -\lambda e$, which implies exponential decay
- The sliding surface parameter $\lambda$ determines the convergence rate

```
                           ė
                           │
                           │    s > 0  (above surface)
                           │  ╱
                           │╱    s = 0 (sliding surface)
          ─────────────────┼─────────────────────── e
                          ╱│
                        ╱  │
              s < 0   ╱    │
           (below surface)
                           │
                           
    Sliding surface: s = ė + λe = 0  ⟹  ė = -λe
```

#### Higher-Order Sliding Surfaces

For an $n$-th order system:

$$s = e^{(n-1)} + c_{n-2}e^{(n-2)} + \cdots + c_1\dot{e} + c_0 e$$

The coefficients $c_i$ are chosen so that the polynomial $p^{n-1} + c_{n-2}p^{n-2} + \cdots + c_0$ is Hurwitz (all roots in LHP).

#### Integral Sliding Surface

To eliminate the reaching phase and ensure robustness from $t = 0$:

$$s = \dot{e} + \lambda e + K_i \int_0^t e(\tau)d\tau - s(0)$$

where $s(0)$ is chosen so that $s(\mathbf{x}(0)) = 0$.

#### Terminal Sliding Surface

For finite-time convergence to the equilibrium:

$$s = \dot{x} + \beta x^{q/p}$$

where $p, q$ are positive odd integers with $q < p$, and $\beta > 0$.

### 17b.1.3 Reaching Condition and Stability Analysis

**Theorem (Sliding Condition):** The system trajectory reaches and stays on the sliding surface $s = 0$ if:

$$s \cdot \dot{s} < 0 \quad \text{(sign condition)}$$

or equivalently:

$$s \cdot \dot{s} \leq -\eta |s| \quad \text{for some } \eta > 0$$

**Proof using Lyapunov's Direct Method:**

Define Lyapunov function candidate:
$$V = \frac{1}{2}s^2 > 0$$

Time derivative along trajectories:
$$\dot{V} = s\dot{s}$$

If $s\dot{s} \leq -\eta|s|$, then:
$$\dot{V} \leq -\eta|s| = -\eta\sqrt{2V} = -\sqrt{2}\eta V^{1/2}$$

This is a **finite-time stability** condition! Integrating:
$$\sqrt{V(t)} \leq \sqrt{V(0)} - \frac{\eta}{\sqrt{2}}t$$

The surface is reached in finite time:
$$t_{reach} \leq \frac{\sqrt{2V(0)}}{\eta} = \frac{|s(0)|}{\eta}$$

---

## 17b.2 Control Law Design

### 17b.2.1 Equivalent Control

The **equivalent control** $u_{eq}$ keeps the system on the sliding surface once reached.

Setting $\dot{s} = 0$ (staying on surface) and solving for $u$:

$$\dot{s} = \frac{\partial s}{\partial \mathbf{x}}\dot{\mathbf{x}} = \frac{\partial s}{\partial \mathbf{x}}[f(\mathbf{x}) + g(\mathbf{x})u_{eq}] = 0$$

For second-order system with $s = \dot{e} + \lambda e$:

$$\dot{s} = \ddot{e} + \lambda\dot{e} = (f + gu - \ddot{x}_d) + \lambda(\dot{x} - \dot{x}_d) = 0$$

Solving:
$$u_{eq} = \frac{1}{g}[-f + \ddot{x}_d - \lambda\dot{e}]$$

### 17b.2.2 Switching Control

The **switching control** $u_{sw}$ drives the system toward the sliding surface:

$$u_{sw} = -K \cdot \text{sign}(s)$$

where $K > 0$ is the switching gain, and:

$$\text{sign}(s) = \begin{cases} +1 & \text{if } s > 0 \\ -1 & \text{if } s < 0 \\ 0 & \text{if } s = 0 \end{cases}$$

**Total Control Law:**
$$u = u_{eq} + u_{sw} = \frac{1}{g}\left[-f + \ddot{x}_d - \lambda\dot{e} - (K + D_{max})\text{sign}(s)\right]$$

where $D_{max} = \sup|d(t)|$ is the bound on disturbance.

### 17b.2.3 Reaching Laws

Different reaching laws provide different convergence characteristics:

| Reaching Law | Equation | Properties |
|--------------|----------|------------|
| **Constant Rate** | $\dot{s} = -K\text{sign}(s)$ | Finite time, constant rate |
| **Constant + Proportional** | $\dot{s} = -K\text{sign}(s) - \lambda s$ | Faster near surface |
| **Power Rate** | $\dot{s} = -K|s|^\alpha\text{sign}(s)$ | Adjustable profile, $\alpha \in (0,1)$ |
| **Exponential** | $\dot{s} = -Ke^{-\beta|s|}\text{sign}(s)$ | Reduced chattering far from surface |

### 17b.2.4 Chattering Reduction: Boundary Layer Approach

The discontinuous control causes **chattering** - high-frequency oscillations around the sliding surface.

**Solution:** Replace $\text{sign}(s)$ with a continuous approximation:

$$\text{sat}(s/\phi) = \begin{cases} s/\phi & \text{if } |s| \leq \phi \\ \text{sign}(s) & \text{if } |s| > \phi \end{cases}$$

where $\phi$ is the boundary layer thickness.

**Trade-off:** Larger $\phi$ → less chattering but reduced robustness

```
                Control Signal
                     │
                  K  ├────────────────●
                     │              ╱
                     │            ╱   sign(s)
                     │          ╱
                     │        ╱
          ───────────┼──────●─────────────── s
                     │    ╱ ↑
                     │  ╱   φ (boundary layer)
                     │╱
                 -K  ●────────────────
                     │
                     
    Inside boundary layer: u ∝ s (linear)
    Outside: u = K·sign(s) (switching)
```

---

## 17b.3 Advanced SMC Algorithms

### 17b.3.1 Super-Twisting Algorithm (STA)

The Super-Twisting Algorithm provides **continuous control** with **finite-time convergence**:

$$u = -\alpha |s|^{1/2}\text{sign}(s) + v$$
$$\dot{v} = -\beta \text{sign}(s)$$

where $\alpha, \beta > 0$ are gains.

**Key Properties:**
- Control signal is continuous (integral of discontinuous term)
- Finite-time convergence to $s = \dot{s} = 0$
- Robust to Lipschitz disturbances
- No chattering in sliding phase

**Gain Selection (Moreno & Osorio, 2012):**
$$\alpha > \sqrt{4L}, \quad \beta > L$$

where $L$ is the Lipschitz constant of the disturbance derivative.

**Lyapunov Analysis:**

Define $\boldsymbol{\xi} = [|s|^{1/2}\text{sign}(s), v]^T$ and Lyapunov function:

$$V(\boldsymbol{\xi}) = \boldsymbol{\xi}^T \mathbf{P} \boldsymbol{\xi}$$

With proper choice of $\mathbf{P}$, one can show $\dot{V} \leq -\gamma V^{1/2}$ (finite-time stability).

### 17b.3.2 Fixed-Time SMC (FxTSMC)

**Reference:** Polyakov (2012) "Nonlinear feedback design for fixed-time stabilization"

Fixed-time convergence means the settling time is **bounded independent of initial conditions**:

$$T_{settle} \leq T_{max} \quad \forall x(0)$$

**Control Law:**
$$u = u_{eq} - \frac{1}{g}\left[k_1|s|^p\text{sign}(s) + k_2|s|^q\text{sign}(s) + D_{max}\text{sign}(s)\right]$$

where:
- $0 < p < 1$ (fast convergence far from surface)
- $q > 1$ (fast convergence near surface)
- $k_1, k_2 > 0$ are gains

**Reaching Law:**
$$\dot{s} = -k_1|s|^p\text{sign}(s) - k_2|s|^q\text{sign}(s)$$

**Maximum Settling Time:**
$$T_{max} = \frac{1}{k_1(1-p)} + \frac{1}{k_2(q-1)}$$

**Proof Sketch:**

Using Lyapunov function $V = |s|$:
$$\dot{V} = \text{sign}(s)\dot{s} = -k_1|s|^p - k_2|s|^q = -k_1 V^p - k_2 V^q$$

This bi-homogeneous system has a fixed settling time given by the formula above.

**Design Guidelines:**
- Typical choices: $p = 0.5$, $q = 1.5$
- Equal contribution: $k_1 = k_2$ for balanced convergence
- For faster settling: increase both gains proportionally

```cpp
// CppPlot Implementation
auto controller = createFixedTimeSMC(5.0, 5.0, 0.5, 1.5);  // k1, k2, p, q
// T_max = 1/(5×0.5) + 1/(5×0.5) = 0.8 seconds
```

### 17b.3.3 Event-Triggered SMC (ETSMC)

**Reference:** Heemels & Donkers (2012), adapted for SMC

Traditional SMC requires continuous control updates. Event-triggered SMC updates control **only when necessary**:

**Event Trigger Condition:**
$$\|s(t) - s(t_k)\| > \sigma\|s(t_k)\| + \epsilon$$

where:
- $t_k$ is the last event time
- $\sigma \in (0, 1)$ is relative threshold
- $\epsilon > 0$ is absolute threshold

**Control Law (Zero-Order Hold):**
$$u(t) = u(t_k) \quad \text{for } t \in [t_k, t_{k+1})$$

**Benefits:**
- 90-99% reduction in control updates
- Reduced actuator wear
- Lower computational load
- Energy saving in battery-powered systems

**Zeno Prevention:**
Minimum inter-event time $\tau_{min}$ ensures finite number of events:
$$t_{k+1} - t_k \geq \tau_{min}$$

**Stability Analysis:**

Using ISS (Input-to-State Stability) framework, the event-triggered system remains stable if the trigger threshold is sufficiently small relative to the switching gain.

### 17b.3.4 Barrier Function-based SMC

**Reference:** Tee et al. (2009) "Barrier Lyapunov Functions"

For systems with **state constraints** $|x| < k_c$:

**Barrier Lyapunov Function:**
$$V = \frac{1}{2}\log\left(\frac{k_c^2}{k_c^2 - x^2}\right) + \frac{1}{2}s^2$$

**Properties:**
- $V \to \infty$ as $|x| \to k_c$ (prevents constraint violation)
- If $V(0)$ is finite and $\dot{V} \leq 0$, then $|x(t)| < k_c$ for all $t \geq 0$

**Control Law:**
$$u = u_{eq} + u_{sw} - \frac{k_b}{g}\cdot\frac{x}{k_c^2 - x^2}$$

where $k_b > 0$ is the barrier gain.

**Barrier Term Derivation:**
$$V_{barrier} = \frac{1}{2}\ln\!\left(\frac{k_c^2}{k_c^2 - x^2}\right) = \frac{1}{2}\bigl[\ln k_c^2 - \ln(k_c^2 - x^2)\bigr]$$

$$\frac{\partial V_{barrier}}{\partial x} = \frac{1}{2} \cdot \frac{2x}{k_c^2 - x^2} = \frac{x}{k_c^2 - x^2}$$

> **Note:** The barrier gradient $\frac{x}{k_c^2 - x^2} \to \infty$ as $|x| \to k_c$, providing increasingly strong repulsion from the constraint boundary. This ensures $|x(t)| < k_c$ for all $t \geq 0$ when starting within the feasible region.

### 17b.3.5 Disturbance Observer-based SMC (DOBSMC)

**Concept:** Combine SMC with a disturbance observer to:
1. Estimate the disturbance online
2. Compensate it in the equivalent control
3. Reduce switching gain (less chattering)

**Disturbance Observer:**
$$\hat{d} = z + Lx_2$$
$$\dot{z} = -L(f + gu + \hat{d})$$

where $L > 0$ is the observer gain.

**Enhanced Control Law:**
$$u = \frac{1}{g}\left[-f - \hat{d} + \ddot{x}_d - \lambda\dot{e} - K_{reduced}\text{sign}(s)\right]$$

**Benefits:**
- Smaller switching gain $K_{reduced} \ll K$
- 60-80% reduction in chattering
- Better tracking accuracy
- Active disturbance rejection

**Observer Gain Selection:**
Higher $L$ → faster estimation but more noise sensitivity
Typical: $L = 20\omega_n$ to $50\omega_n$ where $\omega_n$ is system bandwidth

---

## 17b.4 Design Procedure

### Step-by-Step SMC Design

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    SMC DESIGN FLOWCHART                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   Step 1: SYSTEM MODELING                                                   │
│   ───────────────────────                                                   │
│   • Write equations in canonical form: ẍ = f(x) + g(x)u + d(t)            │
│   • Identify nominal dynamics f(x), g(x)                                   │
│   • Bound disturbance: |d(t)| ≤ D_max                                      │
│                                                                             │
│   Step 2: SLIDING SURFACE DESIGN                                           │
│   ────────────────────────────────                                          │
│   • Choose surface type (linear, integral, terminal)                       │
│   • Select convergence rate λ based on desired dynamics                    │
│   • For n-th order: ensure Hurwitz polynomial                              │
│                                                                             │
│   Step 3: CONTROL LAW DESIGN                                               │
│   ──────────────────────────                                                │
│   • Compute equivalent control u_eq                                         │
│   • Design switching control u_sw                                           │
│   • Select gains: K > D_max + η for robustness                             │
│                                                                             │
│   Step 4: CHATTERING MITIGATION                                            │
│   ─────────────────────────────                                             │
│   • Choose method: boundary layer, STA, or observer-based                  │
│   • Tune parameters: φ (boundary), α,β (STA), L (observer)                │
│                                                                             │
│   Step 5: STABILITY VERIFICATION                                           │
│   ──────────────────────────────                                            │
│   • Verify reaching condition: s·ṡ < -η|s|                                 │
│   • Check Lyapunov conditions                                              │
│   • Estimate reaching time                                                  │
│                                                                             │
│   Step 6: SIMULATION AND TUNING                                            │
│   ─────────────────────────────                                             │
│   • Test with nominal model                                                 │
│   • Add disturbances and uncertainties                                     │
│   • Fine-tune gains for performance/chattering trade-off                   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Design Example: DC Motor Position Control

**System Model:**
$$J\ddot{\theta} + B\dot{\theta} = K_t i_a + \tau_d$$

In canonical form with $x_1 = \theta$, $x_2 = \dot{\theta}$:
$$\dot{x}_2 = -\frac{B}{J}x_2 + \frac{K_t}{J}u + \frac{\tau_d}{J}$$

where:
- $f(x) = -\frac{B}{J}x_2$
- $g(x) = \frac{K_t}{J}$
- $d(t) = \frac{\tau_d}{J}$

**Step 1: Parameters**
- $J = 0.01$ kg·m², $B = 0.1$ N·m·s, $K_t = 0.1$ N·m/A
- Disturbance bound: $|\tau_d| \leq 0.5$ N·m → $D_{max} = 50$

**Step 2: Sliding Surface**
$$s = \dot{e} + \lambda e$$

Choose $\lambda = 10$ for 0.1s time constant on surface.

**Step 3: Control Law**
$$u_{eq} = \frac{J}{K_t}\left[\frac{B}{J}x_2 + \ddot{\theta}_d - \lambda\dot{e}\right]$$
$$u_{sw} = -\frac{J}{K_t}(K + D_{max})\text{sign}(s)$$

Choose $K = 60 > D_{max} + \eta = 51$.

**Step 4: Chattering Reduction**
Use boundary layer $\phi = 0.1$ or Super-Twisting with $\alpha = 1.5\sqrt{K}$, $\beta = 1.1K$.

---

## 17b.5 Implementation in CppPlot

### 17b.5.1 Basic Conventional SMC

```cpp
#include <cppplot/control/nonlinear/sliding_mode.hpp>

using namespace cppplot::control::nonlinear;

// Configure sliding surface: s = ẋ + λx
auto surface_cfg = SlidingSurfaceConfig::linear({10.0, 1.0});  // λ = 10

// Configure SMC controller
SMCConfig smc_cfg;
smc_cfg.type = SMCType::CONVENTIONAL;
smc_cfg.K = 20.0;
smc_cfg.use_boundary_layer = true;
smc_cfg.boundary_thickness = 0.1;

// Create controller
SlidingModeController smc(smc_cfg, surface_cfg, 2);

// Set system dynamics: ẍ = -0.5ẋ + u
smc.setDynamics(
    [](const auto& x) { return -0.5 * x[1]; },  // f(x)
    [](const auto& x) { return 1.0; },           // g(x)
    2.0                                           // D_max
);

// Simulation loop
std::vector<double> x = {5.0, 0.0};  // Initial state
double dt = 0.001;

for (double t = 0; t <= 5.0; t += dt) {
    double u = smc.compute(x, 0.0, dt);  // Track reference = 0
    
    // Integrate dynamics with disturbance
    double d = 1.5 * sin(5*t);
    x[1] += (-0.5*x[1] + u + d) * dt;
    x[0] += x[1] * dt;
}
```

### 17b.5.2 Fixed-Time SMC

```cpp
// Create Fixed-Time SMC
auto fxt_smc = createFixedTimeSMC(
    5.0,    // k1: gain for |s|^p term
    5.0,    // k2: gain for |s|^q term
    0.5,    // p: exponent (0 < p < 1)
    1.5     // q: exponent (q > 1)
);

// Theoretical maximum settling time
double T_max = 1.0/(5.0*(1-0.5)) + 1.0/(5.0*(1.5-1));  // = 0.8 s

std::cout << "Guaranteed settling time T_max = " << T_max << " s\n";
```

### 17b.5.3 Event-Triggered SMC

```cpp
// Create Event-Triggered SMC
auto et_smc = createEventTriggeredSMC(
    0.1,    // threshold: absolute trigger threshold
    0.5,    // sigma: relative threshold (0 < σ < 1)
    0.001   // min_inter_event: Zeno prevention
);

// In simulation loop:
int control_updates = 0;
for (double t = 0; t <= 5.0; t += dt) {
    double u = et_smc.compute(x, ref, dt);
    
    if (et_smc.wasEventTriggered()) {
        control_updates++;
    }
    // ... integrate ...
}

std::cout << "Control updates: " << control_updates 
          << " (vs " << int(5.0/dt) << " conventional)\n";
```

### 17b.5.4 Disturbance Observer SMC

```cpp
// Create DOBSMC
auto dob_smc = createDisturbanceObserverSMC(
    50.0,   // observer_gain
    100.0   // filter_frequency
);

// Monitor disturbance estimation
for (double t = 0; t <= 5.0; t += dt) {
    double u = dob_smc.compute(x, ref, dt);
    double d_hat = dob_smc.getEstimatedDisturbance();
    
    std::cout << "t=" << t << " d_actual=" << disturbance(t) 
              << " d_estimated=" << d_hat << "\n";
}
```

### 17b.5.5 Complete Simulation Example

```cpp
#include <cppplot/control/nonlinear/sliding_mode.hpp>
#include <cppplot/cppplot.hpp>

int main() {
    using namespace cppplot;
    using namespace cppplot::control::nonlinear;
    
    // System: Double integrator with friction
    //   ẋ₁ = x₂
    //   ẋ₂ = u + d(t) - 0.5*sign(x₂)  (Coulomb friction)
    
    auto dynamics = [](double t, const std::vector<double>& x, double u) {
        double friction = 0.5 * ((x[1] > 0) ? 1.0 : ((x[1] < 0) ? -1.0 : 0.0));
        return std::vector<double>{x[1], u - friction};
    };
    
    // Configure Super-Twisting SMC
    SMCConfig cfg;
    cfg.type = SMCType::SUPER_TWISTING;
    cfg.sta_alpha = 5.0;
    cfg.sta_beta = 3.0;
    
    auto surface_cfg = SlidingSurfaceConfig::linear({5.0, 1.0});
    SlidingModeController controller(cfg, surface_cfg, 2);
    
    // Simulate
    SMCSimulator sim(controller, dynamics, 2);
    
    auto result = sim.simulate(
        {2.0, 0.0},           // x0
        5.0,                   // t_final
        0.001,                 // dt
        [](double t) { return sin(t); },  // reference
        [](double t) { return 0.3*cos(3*t); }  // disturbance
    );
    
    // Plot results
    plotSMCResponse(result, "smc_super_twisting_demo");
    
    std::cout << "Reaching time: " << result.reaching_time << " s\n";
    std::cout << "Max chattering: " << result.max_chattering << "\n";
    
    return 0;
}
```

---

## 17b.6 Comparison of SMC Algorithms

### Performance Metrics

| Algorithm | Convergence Time | Chattering | Computational Load | Robustness |
|-----------|------------------|------------|-------------------|------------|
| Conventional | Finite (depends on IC) | High | Low | High |
| Super-Twisting | Finite | Low | Medium | High |
| Fixed-Time | Bounded T_max | Medium | Low | High |
| Event-Triggered | Finite | Low | Very Low | Medium-High |
| Barrier Function | Finite | Medium | Medium | High + Constraints |
| DOBSMC | Finite | Very Low | Medium-High | Very High |

### Selection Guidelines

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    ALGORITHM SELECTION GUIDE                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   QUESTION: What is your primary concern?                                   │
│                                                                             │
│   ▶ CHATTERING                                                             │
│     → Super-Twisting (continuous control)                                  │
│     → DOBSMC (reduced switching gain)                                      │
│                                                                             │
│   ▶ GUARANTEED SETTLING TIME                                               │
│     → Fixed-Time SMC (T_max independent of IC)                             │
│                                                                             │
│   ▶ RESOURCE EFFICIENCY (embedded systems)                                 │
│     → Event-Triggered SMC (90%+ update reduction)                          │
│                                                                             │
│   ▶ STATE CONSTRAINTS                                                      │
│     → Barrier Function SMC (guaranteed constraint satisfaction)            │
│                                                                             │
│   ▶ LARGE DISTURBANCES                                                     │
│     → DOBSMC (active estimation & compensation)                            │
│                                                                             │
│   ▶ SIMPLICITY                                                             │
│     → Conventional SMC with boundary layer                                 │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 17b.7 Practical Considerations

### 17b.7.1 Gain Tuning Guidelines

**Switching Gain K:**
- Must satisfy $K > D_{max} + \eta$
- Larger K → faster reaching, more chattering
- Start with $K = 2D_{max}$ and adjust

**Surface Parameter λ:**
- Determines convergence rate on surface: $\tau = 1/\lambda$
- Larger λ → faster response, more control effort
- Rule of thumb: $\lambda = 3-10 \times$ desired bandwidth

**Boundary Layer φ:**
- Trade-off: chattering vs tracking accuracy
- Start with $\phi = 0.1$ and observe steady-state error
- For precision: $\phi < 0.01$ but accept some chattering

### 17b.7.2 Implementation Issues

**Sampling Rate:**
- SMC requires fast sampling (100-1000× system bandwidth)
- Too slow → excessive chattering, missed surface crossings

**Noise Sensitivity:**
- Derivative estimation is noise-amplifying
- Use low-pass filters or observers
- Super-Twisting provides inherent filtering

**Actuator Dynamics:**
- Fast actuator assumed; if slow, include in model
- Bandwidth should be > 5× SMC switching frequency

### 17b.7.3 Common Pitfalls

1. **Underestimating D_max:** Leads to loss of sliding mode
2. **Overestimating D_max:** Excessive chattering
3. **Forgetting g(x) sign:** Control reversal if g(x) changes sign
4. **Ignoring unmatched uncertainties:** SMC only handles matched uncertainties

---

## 17b.8 Simulation Results and Analysis

### 17b.8.1 Fixed-Time SMC Demonstration

```
┌─────────────────────────────────────────────────────────────────────────────┐
│   FIXED-TIME SMC: Settling Time vs Initial Condition                        │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   Parameters: k1 = 5, k2 = 5, p = 0.5, q = 1.5                             │
│   Theoretical T_max = 0.8 s                                                 │
│                                                                             │
│   Initial Condition    Actual Settling Time    Within T_max?               │
│   ─────────────────    ────────────────────    ─────────────               │
│   x(0) = 1             0.152 s                 ✓                           │
│   x(0) = 5             0.531 s                 ✓                           │
│   x(0) = 10            0.687 s                 ✓                           │
│   x(0) = 20            0.752 s                 ✓                           │
│   x(0) = 50            0.798 s                 ✓                           │
│                                                                             │
│   Key observation: All settling times bounded by T_max regardless of IC!   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 17b.8.2 Event-Triggered SMC Efficiency

```
┌─────────────────────────────────────────────────────────────────────────────┐
│   EVENT-TRIGGERED SMC: Resource Efficiency Analysis                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   Simulation: 5 seconds, dt = 0.001 s                                       │
│   Conventional: 5000 control updates                                        │
│   Event-Triggered: 37 control updates                                       │
│                                                                             │
│   Reduction: 99.3%                                                          │
│                                                                             │
│   Control Updates Over Time:                                                │
│                                                                             │
│   |●●●●                                    Conventional (dense)            │
│   |●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●                  │
│   |                                                                         │
│   |●  ●  ●   ●    ●      ●        ●          ●                Event-Trig   │
│   └────────────────────────────────────────────────────────► time          │
│   0s                    2.5s                                 5s            │
│                                                                             │
│   Note: More events during transient, sparse in steady state               │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 17b.8.3 DOBSMC Chattering Reduction

```
┌─────────────────────────────────────────────────────────────────────────────┐
│   DOBSMC: Chattering Comparison                                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   Conventional SMC Chattering Metric: 14887.8                               │
│   DOBSMC Chattering Metric: 3713.4                                          │
│   Reduction: 75.1%                                                          │
│                                                                             │
│   Control Signal Comparison:                                                │
│                                                                             │
│   Conventional SMC:                                                         │
│   u │    ╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲╱╲    High-frequency switching                   │
│     │───╱──╲╱──╲╱──╲╱──╲───                                                │
│     └────────────────────────► t                                           │
│                                                                             │
│   DOBSMC:                                                                   │
│   u │    ╭──────────────╮      Smooth control                              │
│     │───╯              ╰───                                                 │
│     └────────────────────────► t                                           │
│                                                                             │
│   Disturbance Estimation:                                                   │
│   d │  ─── actual   ╴╴╴ estimated                                          │
│     │  ≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈≈    Accurate tracking                            │
│     └────────────────────────► t                                           │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 17b.9 Summary

| Topic | Key Takeaway |
|-------|--------------|
| **Sliding Surface** | $s = 0$ defines desired dynamics; reaching + sliding phases |
| **Reaching Condition** | $s\dot{s} < -\eta|s|$ guarantees finite-time reaching |
| **Equivalent Control** | Keeps system on surface; computed from $\dot{s} = 0$ |
| **Switching Control** | Drives system to surface; provides robustness |
| **Chattering** | High-frequency oscillation; mitigate with boundary layer or STA |
| **Fixed-Time SMC** | Bounded settling time independent of initial conditions |
| **Event-Triggered** | Resource-efficient; 90%+ reduction in control updates |
| **Barrier Function** | Guarantees state constraint satisfaction |
| **DOBSMC** | Active disturbance estimation; 75%+ chattering reduction |

---

### 📋 Signal Dictionary — Sliding Mode Control of a Position System

| Signal | Symbol | Meaning | Typical unit |
|--------|--------|---------|--------------|
| State | $\mathbf{x} = [x, \dot{x}]^T$ | Position and velocity | [m, m/s] |
| Sliding surface | $s = \dot{x} + \lambda x$ (or $s = \dot{e} + \lambda e$) | Linear combination of states — defines the "target manifold" | m/s |
| Equivalent control | $u_{eq}$ | Control that keeps system ON the surface ($\dot{s} = 0$) — smooth | N |
| Switching control | $u_{sw} = -K\,\text{sign}(s)$ | Drives system TO the surface — discontinuous | N |
| Total control | $u = u_{eq} + u_{sw}$ | Equivalent + switching | N |
| Reaching phase | — | Trajectory moving toward $s = 0$ ; $s\dot{s} < 0$ guaranteed | — |
| Sliding phase | — | Trajectory constrained to $s = 0$; dynamics reduced by 1 order | — |
| Chattering | — | High-frequency oscillation around $s = 0$ due to $\text{sign}(s)$ | — |
| Boundary layer | $\phi$ | Replaces $\text{sign}(s)$ with $\text{sat}(s/\phi)$ — smooths chattering | m/s |
| Disturbance | $d(t)$ | Unknown bounded perturbation: $|d| \leq D$ | N |
| Super-twisting gain | $\alpha, \beta$ | STA parameters: $u = -\alpha|s|^{1/2}\text{sign}(s) + v$, $\dot{v} = -\beta\,\text{sign}(s)$ | varies |
| Integral sliding surface | $s_I = s + k_I \int e\,d\tau$ | Adds integral action for zero steady-state error | m/s |

> **Key insight:** On the sliding surface, the system behaves as if it were *lower-order* and *governed entirely by surface coefficients* ($\lambda$, $k_I$). The matched disturbance $d(t)$ is **completely rejected** — this is the mechanism behind SMC's legendary robustness. Chattering is the price paid for this invariance.

---

## 17b.10 Exercises

**E16b.1 (Conventional SMC Design)**
For the system $\ddot{x} = u + d(t)$ with $|d(t)| \leq 1$, design a sliding mode controller using the sliding surface $s = \dot{x} + 2x$.

(a) Compute the equivalent control $u_{eq}$ by setting $\dot{s} = 0$ (with $d = 0$).

(b) Design the switching control $u_{sw} = -K\,\text{sign}(s)$ with $K$ chosen to satisfy the reaching condition $s\dot{s} < -\eta|s|$ for $\eta = 0.5$.

(c) Write the total control law $u = u_{eq} + u_{sw}$.

(d) Simulate from initial condition $(x_0, \dot{x}_0) = (2, 1)$ with $d(t) = 0.8\sin(5t)$. Plot the state trajectory in the phase plane and the control signal.

---

**E16b.2 (Equivalent Control Computation)**
For the system in E16b.1, when motion is on the sliding surface $s = 0$:

(a) Show that on the surface, $\dot{x} = -2x$, so the system reduces to a first-order stable system.

(b) Derive the equivalent control by substituting $\dot{s} = 0$ into the system dynamics *including* the disturbance: $u_{eq} = -2\dot{x} - d(t)$.

(c) Explain why the equivalent control automatically compensates for the matched disturbance $d(t)$ during the sliding phase.

---

**E16b.3 (Chattering Comparison)**
For the system in E16b.1, compare the chattering behavior of three control laws:

| Variant | Switching term |
|---------|---------------|
| Ideal | $-K\,\text{sign}(s)$ |
| Boundary layer ($\phi = 0.01$) | $-K\,\text{sat}(s/0.01)$ |
| Boundary layer ($\phi = 0.1$) | $-K\,\text{sat}(s/0.1)$ |
| Boundary layer ($\phi = 1.0$) | $-K\,\text{sat}(s/1.0)$ |

(a) Simulate each variant for 5 seconds and plot the control signal $u(t)$.

(b) Define a chattering metric (e.g., $\sum |u(k) - u(k-1)|$) and compute it for each case.

(c) Plot the tracking error for each case. Discuss the trade-off between chattering reduction and tracking accuracy as $\phi$ increases.

---

**E16b.4 (Super-Twisting Algorithm)**
Design a super-twisting sliding mode controller for the same system $\ddot{x} = u + d(t)$, $|d| \leq 1$, with $s = \dot{x} + 2x$:

$$u = u_{eq} - \alpha|s|^{1/2}\text{sign}(s) + v, \quad \dot{v} = -\beta\,\text{sign}(s)$$

(a) Choose gains $\alpha$ and $\beta$ satisfying the sufficient conditions ($\alpha > \sqrt{4L}$, $\beta > L$ where $L$ is the Lipschitz constant of $\dot{d}$).

(b) Simulate and compare with the conventional SMC from E16b.1.

(c) Show that the control signal $u(t)$ is continuous (no chattering) while the convergence is still finite-time.

---

**E16b.5 (Implementation with cppplot)**
Implement the conventional SMC from E16b.1 using cppplot.

(a) Simulate the closed-loop system for $t \in [0, 5]$ s with Euler integration ($dt = 0.001$).

(b) Plot on separate subplots: (i) state trajectory $(x, \dot{x})$ in the phase plane showing the sliding surface, (ii) time histories of $x(t)$ and $\dot{x}(t)$, (iii) control signal $u(t)$, and (iv) sliding variable $s(t)$.

(c) Add a step disturbance $d(t) = 0.5$ for $t \geq 2$ s and show that the system remains on the sliding surface.

---

**E16b.6 (Reaching Condition Proof)**
Prove that the reaching condition $s \cdot \dot{s} < -\eta|s|$ guarantees finite-time reaching of the sliding surface $s = 0$.

(a) Define the Lyapunov function $V = \frac{1}{2}s^2$ and compute $\dot{V}$.

(b) Show that $\dot{V} \leq -\eta\sqrt{2V}$, which implies $\dot{V} \leq -\sqrt{2}\,\eta\,V^{1/2}$.

(c) Integrate the differential inequality to obtain the reaching time bound:
$$t_{reach} \leq \frac{|s(0)|}{\eta}$$

(d) For $s(0) = 5$ and $\eta = 0.5$, compute the maximum reaching time.

---

**E16b.7 (Integral Sliding Mode for Disturbance Rejection)**
Design an integral sliding mode controller to reject constant disturbances with zero steady-state error for the system $\ddot{x} = u + d$, $d = \text{const}$.

(a) Define the integral sliding surface:
$$s = \dot{x} + 2x + k_I \int_0^t x(\tau)\,d\tau$$
Choose $k_I$ so that the characteristic polynomial $p^2 + 2p + k_I$ is Hurwitz.

(b) Derive the control law (equivalent + switching).

(c) Show that on the sliding surface, the closed-loop system has an integrating action that ensures $x \to 0$ for any constant disturbance $d$.

(d) Simulate with $d = 0.5$ (constant) and compare the steady-state error against the non-integral SMC from E16b.1.

---

**E16b.8 🔴 (Level 3 — Chattering: The Practical Killer)**
You implement conventional SMC ($u = u_{eq} - K\,\text{sign}(s)$) on a real DC motor.

(a) In simulation with a perfect model, chattering has infinite frequency and zero amplitude (ideal sliding). In reality, control updates happen at 10 kHz. What frequency does the chattering actually have? What is its amplitude?

(b) The motor driver has a maximum switching frequency of 20 kHz. At control rate 10 kHz with $\text{sign}(s)$, how many switching transitions per second does the driver see? Is this within its capability?

(c) Replace $\text{sign}(s)$ with $\text{sat}(s/\phi)$ using $\phi = 0.1$. The chattering disappears. But the steady-state error is now bounded by $|e_{ss}| \leq \phi / \lambda$. For $\lambda = 5$, what is this error? Is it acceptable for a position control application with 0.01 mm tolerance?

(d) Design a super-twisting algorithm for the same system. Show that it achieves $e_{ss} = 0$ without chattering. What is the trade-off compared to the boundary layer approach?

**E16b.9 🔴 (Level 3 — Matched vs. Unmatched Disturbances)**
For $\ddot{x} = u + d_1(t)$ with $|d_1| \leq 1$, SMC completely rejects $d_1$ (matched disturbance).

(a) Now consider $\dot{x}_1 = x_2 + d_2(t)$, $\dot{x}_2 = u + d_1(t)$ where $d_2$ enters the first equation (unmatched). Show that standard SMC on $s = x_2 + \lambda x_1$ does NOT reject $d_2$.

(b) What is the physical difference between matched (enters through control channel) and unmatched (enters elsewhere) disturbances?

(c) Design a hierarchical SMC with two sliding surfaces to handle both disturbances. Verify via simulation.

**E16b.10 ⚫ (Level 4 — The Robustness Paradox)**

(a) SMC claims "total insensitivity to matched disturbances." But the equivalent control $u_{eq}$ depends on the model ($A, B$ matrices). If the model is wrong, $u_{eq}$ is wrong, and sliding quality degrades. How can SMC be simultaneously "robust" and "model-dependent"? Resolve this paradox.

(b) Compare SMC's robustness mechanism (rejection via high-frequency switching) with H∞'s mechanism (frequency-domain worst-case optimization). Which is more appropriate for: (i) a hydraulic actuator with large, fast disturbances? (ii) a communication channel with broadband noise?

(c) A philosopher of engineering says: "Robustness is not a property of the controller; it is a property of the controller-plant-uncertainty triple." Discuss with reference to SMC's requirement that disturbances be matched and bounded.

---

## References

### Core References

1. Utkin, V.I. (1977). "Variable Structure Systems with Sliding Modes," *IEEE Trans. Automatic Control*, 22(2), 212-222.

2. Slotine, J.J.E. & Li, W. (1991). *Applied Nonlinear Control*. Prentice Hall. Chapter 7.

3. Edwards, C. & Spurgeon, S.K. (1998). *Sliding Mode Control: Theory and Applications*. Taylor & Francis.

### Advanced Algorithms

4. Levant, A. (1993). "Sliding order and sliding accuracy in sliding mode control," *Int. J. Control*, 58(6), 1247-1263.

5. Polyakov, A. (2012). "Nonlinear feedback design for fixed-time stabilization of linear control systems," *IEEE Trans. Automatic Control*, 57(8), 2106-2110.

6. Basin, M.V., et al. (2017). "Continuous fixed-time controller design for mechatronic systems with incomplete measurements," *IEEE/ASME Trans. Mechatronics*, 23(1), 57-67.

7. Heemels, W.P.M.H. & Donkers, M.C.F. (2012). "Model-based periodic event-triggered control for linear systems," *Automatica*, 49(3), 698-711.

8. Tee, K.P., Ge, S.S. & Tay, E.H. (2009). "Barrier Lyapunov functions for the control of output-constrained nonlinear systems," *Automatica*, 45(4), 918-927.

### Implementation & Applications

9. Shtessel, Y., Edwards, C., Fridman, L. & Levant, A. (2014). *Sliding Mode Control and Observation*. Birkhäuser.

10. Utkin, V., Guldner, J. & Shi, J. (2009). *Sliding Mode Control in Electro-Mechanical Systems*, 2nd ed. CRC Press.

---

## Appendix: Mathematical Proofs

### A.1 Proof of Finite-Time Reaching

**Theorem:** For the reaching law $\dot{s} = -K\text{sign}(s)$, the surface $s = 0$ is reached in finite time $t_{reach} = |s(0)|/K$.

**Proof:**

For $s > 0$: $\dot{s} = -K < 0$, so $s(t) = s(0) - Kt$

Setting $s(t_{reach}) = 0$:
$$0 = s(0) - K t_{reach}$$
$$t_{reach} = \frac{s(0)}{K}$$

Similarly for $s < 0$: $\dot{s} = K > 0$, so $s(t) = s(0) + Kt$

Setting $s(t_{reach}) = 0$:
$$t_{reach} = \frac{|s(0)|}{K}$$

Combined: $t_{reach} = \frac{|s(0)|}{K}$ □

### A.2 Proof of Fixed-Time Convergence

**Theorem:** For the bi-homogeneous system $\dot{V} = -k_1 V^p - k_2 V^q$ with $0 < p < 1$ and $q > 1$, the settling time is bounded by:
$$T_{max} = \frac{1}{k_1(1-p)} + \frac{1}{k_2(q-1)}$$

**Proof:**

The differential inequality can be written as:
$$\frac{dV}{-k_1 V^p - k_2 V^q} = dt$$

For large $V$, the $V^q$ term dominates, giving fast decay.
For small $V$, the $V^p$ term dominates, giving fast approach to zero.

Rigorous analysis using comparison lemma shows:
- Time from $V(0)$ to $V = 1$: $\leq \frac{1}{k_2(q-1)}$
- Time from $V = 1$ to $V = 0$: $\leq \frac{1}{k_1(1-p)}$

Total: $T_{max} = \frac{1}{k_1(1-p)} + \frac{1}{k_2(q-1)}$ □

---

*See the CppPlot examples directory for complete runnable code demonstrating all algorithms.*
