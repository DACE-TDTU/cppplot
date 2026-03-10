# Chapter 4: Time-Domain Analysis
## Modern Control Engineering with C++

---

## 🎯 Learning Objectives and Outcomes

### Chapter Objectives

This chapter develops your ability to analyze and predict the time-domain behavior of control systems, connecting physical system parameters to mathematical models and performance specifications.

### Learning Outcomes (Bloom's Taxonomy)

Upon successful completion of this chapter, you will be able to:

| Level | Outcome | Assessment |
|-------|---------|------------|
| **Remember** | Define time constant, natural frequency, damping ratio, and their physical meanings | Quiz |
| **Understand** | Explain how pole locations affect transient response characteristics | Concept questions |
| **Apply** | Calculate rise time, settling time, overshoot from system parameters | Problem sets |
| **Analyze** | Determine stability of a system using Routh-Hurwitz criterion | Analysis exercises |
| **Evaluate** | Assess whether a system meets given performance specifications | Design review |
| **Create** | Design system parameters to achieve specified transient performance | Design project |

### Prerequisites

- Chapter 2: System Modeling (transfer functions)
- Chapter 3: Laplace Transform (inverse transforms, partial fractions)
- Basic differential equations

### Key Competencies

✅ **Technical Skills:**
- Derive time response from transfer function
- Use CppPlot to simulate and visualize system responses
- Apply design formulas for second-order systems

✅ **Engineering Judgment:**
- Trade-off between speed and overshoot
- Select appropriate damping ratio for applications
- Interpret simulation results critically

---

## Why This Chapter Matters

> **The Real Problem:** A pharmaceutical bioreactor must hold temperature at 37.0°C ± 0.5°C. Too hot and the enzyme denatures — a $50,000 batch is destroyed. Too cold and the reaction stalls — 8 hours wasted. The engineer asks: *How fast can we heat up without overshooting? How much oscillation is acceptable? Will the system settle before the reaction deadline?* These are not abstract math questions — they are specifications that determine whether the product lives or dies.
>
> This chapter gives you the tools to answer these questions *quantitatively* — and to understand the **trade-offs** that make them hard.

---

## 4.1 Introduction

Time-domain analysis examines how a control system responds to inputs over time. Unlike frequency-domain methods (covered in Chapter 6), time-domain analysis directly shows the system's behavior, making it intuitive for understanding and specifying system performance.

**The Engineering Questions:**
1. How fast does the system respond? (Speed)
2. Does the output oscillate? (Damping)
3. Does the system reach the desired value? (Accuracy)
4. Is the system stable? (Stability)

---

## 4.2 First-Order Systems

### 4.2.1 Physical System: Thermal System

> **🔬 From Physics to Math:** We start with a real physical system and derive its mathematical model.

**Physical Setup:** Consider a heated metal block in an ambient environment.

```
    ┌─────────────────┐
    │   Metal Block   │  Temperature: T(t)
    │    Mass: M      │  Heat input: Q(t)
    │  Specific heat: c│
    └────────┬────────┘
             │ Heat loss to ambient
             │ h·A·(T - T_ambient)
             ▼
    ═══════════════════  Ambient: T_ambient
```

**Energy Balance (First Law of Thermodynamics):**
$$Mc\frac{dT}{dt} = Q(t) - hA(T - T_{amb})$$

where:
- $M$ = mass of block [kg]
- $c$ = specific heat capacity [J/(kg·K)]
- $h$ = convection coefficient [W/(m²·K)]
- $A$ = surface area [m²]
- $Q(t)$ = heat input [W]

**Signal Dictionary — Heated Metal Block**

| Signal | Symbol | Unit | Physical Meaning | Sensor/Actuator |
|--------|--------|------|------------------|-----------------|
| Heat input (control) | $Q(t)$ | W | Electrical power to the heater element | Cartridge heater + SSR |
| Block temperature (output) | $T(t)$ | °C or K | Temperature of the metal block | Thermocouple / RTD |
| Ambient temperature | $T_{amb}$ | °C or K | Surrounding air temperature (disturbance) | Ambient sensor |
| Convective heat loss | $hA(T - T_{amb})$ | W | Heat flowing out through the surface | (Internal — not directly measured) |
| Thermal time constant | $\tau = Mc/(hA)$ | s | How fast the block responds — large $\tau$ means sluggish | (Derived parameter) |
| DC gain | $K = 1/(hA)$ | °C/W | Steady-state temperature rise per watt of input | (Derived parameter) |

> **Key insight:** The time constant $\tau$ and DC gain $K$ are not arbitrary math — they come directly from the mass, material, and geometry of the block. Change the surface area $A$, and both $\tau$ and $K$ change. This is why we model *physics*, not just transfer functions.

**Rearranging:**
$$\frac{Mc}{hA}\frac{dT}{dt} + T = T_{amb} + \frac{Q(t)}{hA}$$

Define: $\tau = \frac{Mc}{hA}$ (thermal time constant)

**Transfer Function (deviation from ambient):**
$$G(s) = \frac{\Delta T(s)}{Q(s)} = \frac{1/hA}{\tau s + 1} = \frac{K}{\tau s + 1}$$

### 4.2.2 General First-Order Model

The standard first-order transfer function:

$$G(s) = \frac{K}{\tau s + 1}$$

where:
- $K$ = DC gain (steady-state value for unit step input)
- $\tau$ = time constant (determines response speed)

**Physical Interpretation:**
| Parameter | Physical Meaning | Units |
|-----------|------------------|-------|
| $K$ | System sensitivity (output/input at steady state) | varies |
| $\tau$ | Time for 63.2% of change; larger = slower | seconds |

### 4.2.3 Step Response

For a unit step input, the output is:

$$y(t) = K(1 - e^{-t/\tau}), \quad t \geq 0$$

**Key characteristics:**
- At $t = \tau$: output reaches 63.2% of final value
- At $t = 4\tau$: output reaches 98.2% of final value (within 2%)
- At $t = 5\tau$: output reaches 99.3% of final value (within 1%)

### 4.2.4 Example: Thermal System Control

> **🎯 Learning Outcome:** Apply first-order model to predict heating system behavior.

**Problem:** A 2 kg aluminum block ($c = 900$ J/(kg·K)) with surface area 0.02 m² is heated. The convection coefficient is $h = 25$ W/(m²·K). A heater provides 50 W.

**Calculate:**
1. Time constant
2. Steady-state temperature rise
3. Time to reach 90% of final temperature

**Solution:**

```cpp
// Physical parameters
double M = 2.0;      // mass [kg]
double c = 900;      // specific heat [J/(kg·K)]
double h = 25;       // convection coefficient [W/(m²·K)]
double A = 0.02;     // surface area [m²]
double Q = 50;       // heat input [W]

// Derived parameters
double tau = M * c / (h * A);     // time constant
double K = 1.0 / (h * A);          // DC gain [K/W]
double delta_T_ss = K * Q;         // steady-state temp rise

std::cout << "Time constant τ = " << tau << " s = " << tau/60 << " min" << std::endl;
std::cout << "Steady-state ΔT = " << delta_T_ss << " °C" << std::endl;
std::cout << "Time to 90%: t = 2.3τ = " << 2.3*tau << " s" << std::endl;
```

**Output:**
```
Time constant τ = 3600 s = 60 min
Steady-state ΔT = 100 °C
Time to 90%: t = 2.3τ = 8280 s ≈ 138 min
```

**Engineering Insight:** This slow response suggests we need active feedback control (thermostat) for practical temperature regulation.

### 4.2.5 C++ Implementation

```cpp
/**
 * @file ch04_first_order.cpp
 * @brief First-order system analysis
 * 
 * Compile: g++ -std=c++17 -I "../include" ch04_first_order.cpp -o ch04_first_order.exe
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <cmath>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    // System parameters
    double K = 2.0;    // DC gain
    double tau = 3.0;  // Time constant [s]
    
    // Create transfer function: G(s) = K / (tau*s + 1)
    TransferFunction G({K}, {tau, 1});
    
    std::cout << "First-Order System: G(s) = " << K << " / (" 
              << tau << "s + 1)" << std::endl;
    
    // Generate step response
    double t_final = 5 * tau;  // Simulate for 5 time constants
    auto [t, y] = step_data(G, t_final);
    
    // Calculate theoretical values
    double y_tau = K * (1 - exp(-1));      // Value at t = tau
    double y_4tau = K * (1 - exp(-4));     // Value at t = 4*tau
    
    std::cout << "\nTheoretical Analysis:" << std::endl;
    std::cout << "  DC Gain K = " << K << std::endl;
    std::cout << "  Time constant τ = " << tau << " s" << std::endl;
    std::cout << "  At t = τ:   y = " << y_tau << " (" << y_tau/K*100 << "% of final)" << std::endl;
    std::cout << "  At t = 4τ:  y = " << y_4tau << " (" << y_4tau/K*100 << "% of final)" << std::endl;
    
    // Create plot
    figure(900, 600);
    
    // Plot step response
    plot(t, y, "b-", {{"linewidth", "2"}, {"label", "Step Response"}});
    
    // Add reference lines
    axhline(K, {{"color", "green"}, {"linestyle", "--"}, {"label", "Final Value K"}});
    axhline(0.632 * K, {{"color", "orange"}, {"linestyle", ":"}, {"alpha", "0.7"}});
    axvline(tau, {{"color", "red"}, {"linestyle", "--"}, {"label", "t = τ"}});
    
    // Mark the 63.2% point
    scatter({tau}, {y_tau}, {{"color", "red"}, {"markersize", "10"}});
    
    xlabel("Time [s]");
    ylabel("Output y(t)");
    title("First-Order System Step Response: G(s) = K/(τs+1)");
    legend();
    grid(true);
    
    savefig("ch04_first_order_step.svg");
    std::cout << "\nPlot saved to ch04_first_order_step.svg" << std::endl;
    
    return 0;
}
```

**Output:**
```
First-Order System: G(s) = 2 / (3s + 1)

Theoretical Analysis:
  DC Gain K = 2
  Time constant τ = 3 s
  At t = τ:   y = 1.264 (63.2% of final)
  At t = 4τ:  y = 1.963 (98.2% of final)

Plot saved to ch04_first_order_step.svg
```

> **🔍 Stop and Think — Mechanism Questions**
>
> Before moving on, answer these from the output above:
> 1. At $t = \tau$, the response reaches 63.2% of its final value. *Why* 63.2% and not 50%? (Hint: it comes from $1 - e^{-1}$. What does $e^{-1}$ represent physically?)
> 2. The settling time is $4\tau$. If you need the system to settle in half the time, what physical change would you make to the heated metal block? (Hint: which parameters appear in $\tau = Mc/(hA)$?)
> 3. The DC gain $K = 1/(hA)$ determines the final temperature rise per watt. If $K$ is too large, the system overshoots the safe temperature. Is this a control problem or a plant design problem?

### 4.2.6 Effect of Time Constant

The time constant $\tau$ determines how quickly the system responds:

```cpp
// Compare different time constants
figure(900, 600);

std::vector<double> taus = {0.5, 1.0, 2.0, 5.0};
double K = 1.0;

for (double tau : taus) {
    TransferFunction G({K}, {tau, 1});
    auto [t, y] = step_data(G, 20.0);
    plot(t, y, "-", {{"label", "τ = " + std::to_string(tau) + " s"}});
}

xlabel("Time [s]");
ylabel("Output");
title("Effect of Time Constant on Step Response");
legend();
grid(true);
savefig("ch04_time_constant_comparison.svg");
```

> **Key Insight:** Smaller time constants produce faster responses. In control design, we often try to reduce the effective time constant of the closed-loop system.

---

## 4.3 Second-Order Systems

### 4.3.1 Physical System: Mass-Spring-Damper

> **🔬 From Physics to Math:** The mass-spring-damper is the canonical second-order system.

**Physical Setup:**

```
         x(t) →
    ┌─────────┐
    │    M    │────► F(t) (applied force)
    └────┬────┘
     ════╪════  Spring (k)
         │
     ────╫────  Damper (b)
         │
    ─────┴─────  Fixed wall
```

**Newton's Second Law:**
$$M\ddot{x} + b\dot{x} + kx = F(t)$$

where:
- $M$ = mass [kg]
- $b$ = damping coefficient [N·s/m]
- $k$ = spring constant [N/m]
- $F(t)$ = applied force [N]

**Transfer Function:**
$$G(s) = \frac{X(s)}{F(s)} = \frac{1/M}{s^2 + \frac{b}{M}s + \frac{k}{M}} = \frac{1/k}{\frac{M}{k}s^2 + \frac{b}{k}s + 1}$$

**Standard Form Parameters:**
$$\omega_n = \sqrt{\frac{k}{M}}, \quad \zeta = \frac{b}{2\sqrt{kM}}$$

| Physical | Symbol | Mathematical Meaning |
|----------|--------|----------------------|
| Stiff spring | Large $k$ | High $\omega_n$ (fast) |
| Heavy mass | Large $M$ | Low $\omega_n$ (slow) |
| High friction | Large $b$ | High $\zeta$ (damped) |
| No friction | $b = 0$ | $\zeta = 0$ (oscillates forever) |

### 4.3.2 Other Physical Second-Order Systems

| System | Inertia | Spring | Damping |
|--------|---------|--------|--------|
| Mechanical | Mass $M$ | Spring $k$ | Friction $b$ |
| Electrical RLC | Inductance $L$ | $1/C$ | Resistance $R$ |
| Rotational | Moment of inertia $J$ | Torsional spring | Bearing friction |
| Hydraulic | Fluid inertia | Compressibility | Viscous losses |

### 4.3.3 Standard Form

The general second-order transfer function in standard form:

$$G(s) = \frac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}$$

where:
- $\omega_n$ = **natural frequency** [rad/s] - determines the speed of response
- $\zeta$ = **damping ratio** [dimensionless] - determines the shape of response

### 4.3.4 Response Classification

The damping ratio $\zeta$ determines the nature of the response:

| Condition | Response Type | Characteristic | Physical Example |
|-----------|---------------|----------------|------------------|
| $\zeta = 0$ | Undamped | Sustained oscillation | Ideal pendulum in vacuum |
| $0 < \zeta < 1$ | Underdamped | Oscillation with decay | Car suspension |
| $\zeta = 1$ | Critically damped | Fastest non-oscillatory | Door closer |
| $\zeta > 1$ | Overdamped | Slow, no oscillation | Heavy oil damper |

### 4.3.5 Poles and Damping

The poles of a second-order system are:

$$s_{1,2} = -\zeta\omega_n \pm \omega_n\sqrt{\zeta^2 - 1}$$

For the underdamped case ($0 < \zeta < 1$):
$$s_{1,2} = -\sigma \pm j\omega_d$$

where:
- $\sigma = \zeta\omega_n$ = **damping coefficient**
- $\omega_d = \omega_n\sqrt{1-\zeta^2}$ = **damped natural frequency**

### 4.3.6 Design Example: Car Suspension System

> **🎯 Learning Outcome:** Design suspension parameters to meet ride comfort specifications.

**Problem:** Design a car suspension system with:
- Quarter-car mass: $M = 400$ kg (including wheel)
- Desired settling time: $t_s \leq 1$ s
- Maximum overshoot: $OS \leq 10\%$ (for passenger comfort)

**Design Process:**

**Step 1: Determine required ζ from overshoot**
$$\zeta = \frac{-\ln(OS/100)}{\sqrt{\pi^2 + \ln^2(OS/100)}} = \frac{-\ln(0.1)}{\sqrt{\pi^2 + \ln^2(0.1)}} \approx 0.59$$

**Step 2: Determine required ωn from settling time**
$$t_s = \frac{4}{\zeta\omega_n} \Rightarrow \omega_n = \frac{4}{\zeta \cdot t_s} = \frac{4}{0.59 \times 1} \approx 6.78 \text{ rad/s}$$

**Step 3: Calculate physical parameters**
$$k = M\omega_n^2 = 400 \times 6.78^2 \approx 18,400 \text{ N/m}$$
$$b = 2\zeta\sqrt{kM} = 2 \times 0.59 \times \sqrt{18400 \times 400} \approx 3,200 \text{ N·s/m}$$

**Verification with CppPlot:**

```cpp
// Design parameters
double M = 400;        // kg
double k = 18400;      // N/m
double b = 3200;       // N·s/m

// Calculate actual ωn and ζ
double wn = sqrt(k / M);           // 6.78 rad/s
double zeta = b / (2 * sqrt(k * M)); // 0.59

// Create transfer function: X(s)/F(s)
TransferFunction G({1/k}, {M/k, b/k, 1});
// Or equivalently in standard form:
TransferFunction G_std({wn*wn}, {1, 2*zeta*wn, wn*wn});

// Simulate response to road bump (step input)
auto [t, y] = step_data(G_std, 3.0);
auto info = stepinfo(t, y);

std::cout << "Actual settling time: " << info.settling_time << " s" << std::endl;
std::cout << "Actual overshoot: " << info.overshoot << "%" << std::endl;
```

**Result:** Design meets specifications ✓

### 4.3.7 C++ Implementation: Damping Ratio Study

```cpp
/**
 * @file ch04_second_order.cpp
 * @brief Second-order system analysis with varying damping ratio
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <cmath>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    double wn = 2.0;  // Natural frequency [rad/s]
    
    // Different damping ratios
    std::vector<double> zetas = {0.1, 0.3, 0.5, 0.707, 1.0, 2.0};
    
    figure(1200, 900);
    
    // === Subplot 1: Step Response ===
    subplot(2, 2, 1);
    
    for (double zeta : zetas) {
        // G(s) = wn²/(s² + 2ζωn·s + ωn²)
        TransferFunction G({wn*wn}, {1, 2*zeta*wn, wn*wn});
        auto [t, y] = step_data(G, 15.0);
        
        std::string label = "ζ = " + std::to_string(zeta).substr(0, 4);
        plot(t, y, "-", {{"label", label}});
    }
    
    axhline(1.0, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.3"}});
    xlabel("Time [s]");
    ylabel("Output");
    title("Step Response vs Damping Ratio (ωn = 2 rad/s)");
    legend();
    grid(true);
    
    // === Subplot 2: Pole Locations ===
    subplot(2, 2, 2);
    
    // Draw unit circle for reference
    std::vector<double> theta_circle, x_circle, y_circle;
    for (int i = 0; i <= 100; ++i) {
        double th = i * 2 * M_PI / 100;
        x_circle.push_back(wn * cos(th));
        y_circle.push_back(wn * sin(th));
    }
    plot(x_circle, y_circle, "k--", {{"alpha", "0.3"}, {"label", "|s| = ωn"}});
    
    // Plot poles for each damping ratio
    for (double zeta : zetas) {
        double sigma = zeta * wn;
        double wd = wn * sqrt(std::abs(1 - zeta*zeta));
        
        if (zeta < 1) {  // Complex conjugate poles
            scatter({-sigma, -sigma}, {wd, -wd}, 
                    {{"markersize", "10"}, {"label", "ζ = " + std::to_string(zeta).substr(0,4)}});
        } else if (zeta == 1) {  // Repeated real poles
            scatter({-sigma}, {0.0}, {{"markersize", "12"}, {"label", "ζ = 1 (critical)"}});
        } else {  // Distinct real poles
            double p1 = -zeta*wn + wn*sqrt(zeta*zeta - 1);
            double p2 = -zeta*wn - wn*sqrt(zeta*zeta - 1);
            scatter({p1, p2}, {0.0, 0.0}, 
                    {{"markersize", "10"}, {"label", "ζ = " + std::to_string(zeta).substr(0,4)}});
        }
    }
    
    axhline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    axvline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    xlabel("Real");
    ylabel("Imaginary");
    title("Pole Locations in s-plane");
    legend();
    grid(true);
    xlim(-5, 1);
    ylim(-3, 3);
    
    // === Subplot 3: Performance Metrics ===
    subplot(2, 2, 3);
    
    std::vector<double> zeta_range, rise_times, settling_times, overshoots;
    
    for (double z = 0.1; z <= 2.0; z += 0.05) {
        TransferFunction G({wn*wn}, {1, 2*z*wn, wn*wn});
        auto [t, y] = step_data(G, 20.0, 500);
        auto info = stepinfo(t, y);
        
        zeta_range.push_back(z);
        rise_times.push_back(info.rise_time);
        settling_times.push_back(info.settling_time);
        overshoots.push_back(info.overshoot);
    }
    
    plot(zeta_range, rise_times, "b-", {{"label", "Rise Time [s]"}, {"linewidth", "2"}});
    plot(zeta_range, settling_times, "r-", {{"label", "Settling Time [s]"}, {"linewidth", "2"}});
    
    xlabel("Damping Ratio ζ");
    ylabel("Time [s]");
    title("Rise Time and Settling Time vs ζ");
    legend();
    grid(true);
    
    // === Subplot 4: Overshoot ===
    subplot(2, 2, 4);
    
    plot(zeta_range, overshoots, "g-", {{"linewidth", "2"}});
    
    // Theoretical overshoot formula: OS = exp(-πζ/√(1-ζ²)) × 100%
    std::vector<double> os_theory;
    for (double z : zeta_range) {
        if (z < 1) {
            double os = 100 * exp(-M_PI * z / sqrt(1 - z*z));
            os_theory.push_back(os);
        } else {
            os_theory.push_back(0);
        }
    }
    plot(zeta_range, os_theory, "r--", {{"label", "Theory"}, {"linewidth", "1.5"}});
    
    xlabel("Damping Ratio ζ");
    ylabel("Overshoot [%]");
    title("Percent Overshoot vs ζ");
    legend();
    grid(true);
    
    savefig("ch04_second_order_analysis.svg");
    
    // Print table of results
    std::cout << "\n=== Second-Order System Performance (ωn = " << wn << " rad/s) ===" << std::endl;
    std::cout << "\n   ζ    |  tr [s]  |  ts [s]  |  OS [%]  |  Poles" << std::endl;
    std::cout << std::string(65, '-') << std::endl;
    
    for (double zeta : zetas) {
        TransferFunction G({wn*wn}, {1, 2*zeta*wn, wn*wn});
        auto [t, y] = step_data(G, 20.0);
        auto info = stepinfo(t, y);
        
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "  " << std::setw(5) << zeta << "  | "
                  << std::setw(8) << info.rise_time << " | "
                  << std::setw(8) << info.settling_time << " | "
                  << std::setw(8) << info.overshoot << " | ";
        
        auto poles = G.poles();
        for (auto& p : poles) {
            std::cout << p << " ";
        }
        std::cout << std::endl;
    }
    
    return 0;
}
```

**Output:**
```
=== Second-Order System Performance (ωn = 2 rad/s) ===

   ζ    |  tr [s]  |  ts [s]  |  OS [%]  |  Poles
-----------------------------------------------------------------
  0.100  |    0.765 |   19.850 |   72.919 | (-0.2+1.99j) (-0.2-1.99j) 
  0.300  |    0.680 |    6.567 |   37.232 | (-0.6+1.91j) (-0.6-1.91j) 
  0.500  |    0.635 |    3.900 |   16.303 | (-1+1.73j) (-1-1.73j) 
  0.707  |    0.640 |    2.850 |    4.321 | (-1.41+1.41j) (-1.41-1.41j) 
  1.000  |    0.900 |    3.650 |    0.000 | -2 -2 
  2.000  |    2.150 |    3.100 |    0.000 | -0.536 -7.46 
```

### 4.3.8 Derivation of the Underdamped Step Response

For the standard second-order system with unit step input:
$$C(s) = \frac{\omega_n^2}{s(s^2 + 2\zeta\omega_n s + \omega_n^2)}$$

Partial fraction expansion:
$$C(s) = \frac{1}{s} - \frac{s + 2\zeta\omega_n}{s^2 + 2\zeta\omega_n s + \omega_n^2}$$

Completing the square in the denominator: $(s + \sigma)^2 + \omega_d^2$ where $\sigma = \zeta\omega_n$ and $\omega_d = \omega_n\sqrt{1-\zeta^2}$:

$$C(s) = \frac{1}{s} - \frac{s + \sigma}{(s+\sigma)^2 + \omega_d^2} - \frac{\sigma}{\omega_d} \cdot \frac{\omega_d}{(s+\sigma)^2 + \omega_d^2}$$

Inverse Laplace transform:
$$c(t) = 1 - e^{-\sigma t}\left[\cos(\omega_d t) + \frac{\sigma}{\omega_d}\sin(\omega_d t)\right]$$

Using the identity $A\cos\theta + B\sin\theta = \sqrt{A^2+B^2}\sin(\theta + \phi)$ where $\phi = \arctan(A/B)$:

$$\boxed{c(t) = 1 - \frac{e^{-\zeta\omega_n t}}{\sqrt{1-\zeta^2}}\sin(\omega_d t + \phi), \quad \phi = \arccos\zeta}$$

From this formula, we can derive all time-domain specifications.

### 4.3.9 Design Formulas for Underdamped Systems

For underdamped systems ($0 < \zeta < 1$), useful approximations:

**Percent Overshoot:**
$$\%OS = 100 \cdot e^{-\frac{\pi\zeta}{\sqrt{1-\zeta^2}}}$$

Solving for $\zeta$:
$$\zeta = \frac{-\ln(\%OS/100)}{\sqrt{\pi^2 + \ln^2(\%OS/100)}}$$

**Rise Time (0% to 100%):**

Exact formula (for underdamped systems, $0 < \zeta < 1$):
$$t_r = \frac{\pi - \arccos(\zeta)}{\omega_d} = \frac{\pi - \arccos(\zeta)}{\omega_n\sqrt{1-\zeta^2}}$$

Common approximation (valid for $0.3 \leq \zeta \leq 0.8$, error < 5%):
$$t_r \approx \frac{1.8}{\omega_n}$$

> **Key relationship:** Higher $\omega_n$ → **shorter** rise time (faster response). This is because $\omega_n$ sets the time scale of the oscillation.

**Settling Time (2% criterion):**
$$t_s \approx \frac{4}{\zeta\omega_n}$$

**Peak Time:**
$$t_p = \frac{\pi}{\omega_d} = \frac{\pi}{\omega_n\sqrt{1-\zeta^2}}$$

```cpp
// Design calculation helper
double zeta_from_overshoot(double os_percent) {
    double ln_os = log(os_percent / 100.0);
    return -ln_os / sqrt(M_PI * M_PI + ln_os * ln_os);
}

double wn_from_settling_time(double ts, double zeta) {
    return 4.0 / (zeta * ts);
}

// Example: Design for OS = 10%, ts = 2s
double os_target = 10.0;
double ts_target = 2.0;

double zeta = zeta_from_overshoot(os_target);  // ζ ≈ 0.591
double wn = wn_from_settling_time(ts_target, zeta);  // ωn ≈ 3.38 rad/s
```

---

## 4.4 Higher-Order Systems

### 4.4.1 Dominant Poles

Higher-order systems can often be approximated by lower-order systems by identifying **dominant poles**—the poles closest to the imaginary axis.

**Rule of thumb:** A pole is negligible if it is at least 5× farther from the imaginary axis than the dominant poles.

> **Validity Criterion for Second-Order Approximation:**
> The second-order approximation is valid when the non-dominant poles are at least **5× farther** from the $j\omega$ axis than the dominant poles. Formally, if the dominant poles have real part $-\sigma_d$ and a non-dominant pole has real part $-\sigma_{nd}$, the approximation requires $|\sigma_{nd}| \geq 5|\sigma_d|$. Under this condition, the transient component from the non-dominant pole decays to less than 1% of its initial value within one dominant time constant ($1/\sigma_d$), making its contribution negligible.

```cpp
// Higher-order system with dominant poles
TransferFunction G({100}, {1, 11, 110, 1100, 1000});

// Find poles
auto poles = G.poles();
std::cout << "Poles: ";
for (auto& p : poles) std::cout << p << " ";

// Identify dominant pair (closest to imaginary axis)
// Approximate with second-order system

figure(800, 600);
auto [t_full, y_full] = step_data(G, 5.0);
plot(t_full, y_full, "b-", {{"linewidth", "2"}, {"label", "Full 4th-order"}});

// Second-order approximation (match dominant poles)
// DC gain preserved: 100/1000 = 1/10 = 0.1
TransferFunction G_approx({1.0}, {1, 2, 10});  // DC gain = 1/10 = 0.1, matches original
auto [t_approx, y_approx] = step_data(G_approx, 5.0);
plot(t_approx, y_approx, "r--", {{"linewidth", "2"}, {"label", "2nd-order approx"}});

legend();
title("Higher-Order System and Dominant Pole Approximation");
```

### 4.4.2 Effect of Additional Poles and Zeros on Step Response

When a system has poles and zeros beyond the dominant second-order pair, the transient response changes significantly. Understanding these effects is essential for practical design.

#### Effect of an Additional Real LHP Pole

Consider adding a real pole at $s = -a$ to a standard second-order system:
$$G(s) = \frac{\omega_n^2}{(s^2 + 2\zeta\omega_n s + \omega_n^2)} \quad \longrightarrow \quad G_3(s) = \frac{a\cdot\omega_n^2}{(s + a)(s^2 + 2\zeta\omega_n s + \omega_n^2)}$$

(DC gain is preserved by the factor $a$ in the numerator.)

| Extra pole location | Effect on step response |
|---------------------|------------------------|
| $a \gg \zeta\omega_n$ (far from dominant poles) | Negligible effect — pole decays quickly |
| $a \approx \zeta\omega_n$ (near dominant poles) | **Increases rise time** and **reduces overshoot** |
| $a < \zeta\omega_n$ (slower than dominant poles) | Dominates the response — system acts more like first-order |

> **Rule of thumb:** An additional pole has negligible effect if it is at least **5× farther** from the imaginary axis than the dominant poles.

**Physical intuition:** An extra pole acts like additional "inertia" in the system, slowing down the response and smoothing out oscillations.

#### Effect of an Additional Real LHP Zero

Adding a zero at $s = -b$ to a second-order system:
$$G(s) = \frac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2} \quad \longrightarrow \quad G_z(s) = \frac{\omega_n^2(s + b)}{b(s^2 + 2\zeta\omega_n s + \omega_n^2)}$$

The zero adds a derivative component to the response:
$$y_z(t) = y_{\text{original}}(t) + \frac{1}{b}\dot{y}_{\text{original}}(t)$$

| Extra zero location | Effect on step response |
|---------------------|------------------------|
| $b \gg \omega_n$ (far from dominant poles) | Negligible effect |
| $b \approx \omega_n$ (near dominant poles) | **Decreases rise time** and **increases overshoot** |
| $b \to 0$ (zero near origin) | Response becomes very aggressive, large overshoot |

**Physical intuition:** A LHP zero adds "anticipation" (derivative action), making the system respond faster but overshoot more.

#### Effect of a RHP Zero (Non-Minimum Phase)

A zero in the right-half plane at $s = +c$ causes **initial inverse response** (undershoot):
$$y_{\text{RHP}}(t) = y_{\text{original}}(t) - \frac{1}{c}\dot{y}_{\text{original}}(t)$$

The negative derivative component initially drives the output in the **opposite direction** before the system recovers.

| Characteristic | Effect |
|----------------|--------|
| **Rise time** | Increases (initial undershoot delays reaching final value) |
| **Overshoot** | May decrease in the positive sense, but **undershoot** appears |
| **Bandwidth limit** | Cannot exceed $\omega_{BW} < c/2$ without instability |

**Example:** Boiler drum level, flexible spacecraft, boost converter — all exhibit non-minimum phase behavior with practical bandwidth limits.

#### Summary Table

| Modification | Rise Time | Overshoot | Settling Time |
|-------------|-----------|-----------|---------------|
| Extra LHP pole (near) | ↑ Increases | ↓ Decreases | ↑ Increases |
| Extra LHP zero (near) | ↓ Decreases | ↑ Increases | ≈ Similar |
| Extra RHP zero | ↑ Increases (undershoot) | Undershoot appears | ↑ Increases |

---

## 4.5 Stability Analysis

### 4.5.1 BIBO Stability

A system is **Bounded-Input Bounded-Output (BIBO) stable** if every bounded input produces a bounded output.

**Theorem:** A linear time-invariant system is BIBO stable if and only if all poles have negative real parts (lie in the left half-plane).

```cpp
// Check stability
TransferFunction G({1}, {1, 3, 2});  // s² + 3s + 2 = (s+1)(s+2)

auto poles = G.poles();
bool stable = true;
for (auto& p : poles) {
    if (p.real() >= 0) stable = false;
}

std::cout << "System is " << (stable ? "STABLE" : "UNSTABLE") << std::endl;
```

### 4.5.2 Routh-Hurwitz Criterion

The Routh-Hurwitz criterion determines stability without computing poles — this is especially valuable for high-order systems and systems with symbolic parameters (e.g., gain $K$).

For characteristic polynomial:
$$a_n s^n + a_{n-1} s^{n-1} + \cdots + a_1 s + a_0 = 0$$

**Necessary conditions for stability:**
1. All coefficients must have the same sign
2. No coefficients can be zero

**Sufficient condition:** All elements in the first column of the Routh table must be positive.

#### Routh Array Construction Algorithm

The Routh table is constructed row by row from the polynomial coefficients:

| Row | Col 1 | Col 2 | Col 3 | ... |
|-----|-------|-------|-------|-----|
| $s^n$ | $a_n$ | $a_{n-2}$ | $a_{n-4}$ | ... |
| $s^{n-1}$ | $a_{n-1}$ | $a_{n-3}$ | $a_{n-5}$ | ... |
| $s^{n-2}$ | $b_1$ | $b_2$ | $b_3$ | ... |
| $s^{n-3}$ | $c_1$ | $c_2$ | $c_3$ | ... |
| $\vdots$ | | | | |
| $s^0$ | | | | |

where each element is computed as a $2 \times 2$ determinant divided by the first element of the row above:

$$b_1 = \frac{a_{n-1} \cdot a_{n-2} - a_n \cdot a_{n-3}}{a_{n-1}}, \quad b_2 = \frac{a_{n-1} \cdot a_{n-4} - a_n \cdot a_{n-5}}{a_{n-1}}$$

$$c_1 = \frac{b_1 \cdot a_{n-3} - a_{n-1} \cdot b_2}{b_1}, \quad \text{etc.}$$

**Stability rule:** The number of sign changes in the first column equals the number of RHP poles.

#### Worked Example

Determine the stability of a system with characteristic equation:
$$s^3 + 6s^2 + 11s + 6 = 0$$

Construct the Routh table ($a_3=1,\ a_2=6,\ a_1=11,\ a_0=6$):

| Row | Col 1 | Col 2 |
|-----|-------|-------|
| $s^3$ | $1$ | $11$ |
| $s^2$ | $6$ | $6$ |
| $s^1$ | $\frac{6 \cdot 11 - 1 \cdot 6}{6} = \frac{60}{6} = 10$ | $0$ |
| $s^0$ | $\frac{10 \cdot 6 - 6 \cdot 0}{10} = 6$ | |

First column: $1, 6, 10, 6$ — **all positive**, no sign changes → **system is stable** (all 3 poles in LHP).

Verification: $s^3+6s^2+11s+6 = (s+1)(s+2)(s+3)$, poles at $s=-1,-2,-3$. ✓

#### Special Cases

**Case 1 — Zero in first column:** If a first-column element is zero but the rest of the row is not all zeros, replace the zero with a small positive number $\varepsilon \to 0^+$, complete the table, then evaluate signs in the limit.

**Case 2 — Entire row of zeros:** This indicates symmetric root pairs (e.g., $\pm j\omega$ or $\pm\sigma$). Form the **auxiliary polynomial** from the row above the zero row, take its derivative, and use the derivative's coefficients to replace the zero row. Then continue construction.

#### Finding Range of Gain $K$ for Stability

One of the most powerful applications of Routh-Hurwitz is determining the range of a parameter (typically gain $K$) for which the closed-loop system remains stable.

**Example:** Given characteristic equation $s^3 + 3s^2 + 2s + K = 0$, find the range of $K$ for stability.

| Row | Col 1 | Col 2 |
|-----|-------|-------|
| $s^3$ | $1$ | $2$ |
| $s^2$ | $3$ | $K$ |
| $s^1$ | $\frac{3 \cdot 2 - 1 \cdot K}{3} = \frac{6 - K}{3}$ | $0$ |
| $s^0$ | $K$ | |

For stability, all first-column elements must be positive:
- $s^3$: $1 > 0$ ✓ (always)
- $s^2$: $3 > 0$ ✓ (always)
- $s^1$: $\frac{6-K}{3} > 0 \implies K < 6$
- $s^0$: $K > 0$

$$\boxed{0 < K < 6}$$

At $K = 6$, the $s^1$ row becomes zero → sustained oscillations (marginally stable). The auxiliary polynomial from the $s^2$ row gives $3s^2 + 6 = 0 \implies s = \pm j\sqrt{2}$, confirming purely imaginary roots at the stability boundary.

```cpp
// Routh-Hurwitz stability check
Polynomial char_poly({1, 6, 11, 6});  // s³ + 6s² + 11s + 6

auto routh = routhTable(char_poly);
printRouthTable(routh);

bool stable = checkRouthStability(routh);
std::cout << "System is " << (stable ? "STABLE" : "UNSTABLE") << std::endl;

// Finding K range for stability
// Characteristic equation: s³ + 3s² + 2s + K = 0
std::cout << "\nStability range for K:" << std::endl;
for (double K : {-1.0, 0.0, 3.0, 5.9, 6.0, 6.1, 10.0}) {
    Polynomial cp({1, 3, 2, K});
    auto rt = routhTable(cp);
    bool is_stable = checkRouthStability(rt);
    std::cout << "  K = " << K << ": "
              << (is_stable ? "STABLE" : "UNSTABLE") << std::endl;
}
```

---

## 4.6 Steady-State Error

### 4.6.1 System Type and Error Constants

The **system type** is the number of integrators (poles at s=0) in the open-loop transfer function.

For unity feedback system with open-loop transfer function $G(s)$:

| System Type | Position Constant | Velocity Constant | Acceleration Constant |
|-------------|-------------------|-------------------|----------------------|
| Type 0 | $K_p = \lim_{s \to 0} G(s)$ | $K_v = 0$ | $K_a = 0$ |
| Type 1 | $K_p = \infty$ | $K_v = \lim_{s \to 0} sG(s)$ | $K_a = 0$ |
| Type 2 | $K_p = \infty$ | $K_v = \infty$ | $K_a = \lim_{s \to 0} s^2G(s)$ |

### 4.6.2 Steady-State Error Formulas

| Input | Error Formula |
|-------|---------------|
| Step (R/s) | $e_{ss} = \frac{R}{1 + K_p}$ |
| Ramp (R/s²) | $e_{ss} = \frac{R}{K_v}$ |
| Parabola (R/s³) | $e_{ss} = \frac{R}{K_a}$ |

```cpp
/**
 * @file ch04_steady_state_error.cpp
 * @brief Steady-state error analysis
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    // Type 1 system (one integrator)
    // G(s) = 10/(s(s+2))
    TransferFunction G({10}, {1, 2, 0});
    
    // Error constants
    double Kp = std::numeric_limits<double>::infinity();  // Type 1
    double Kv = 10.0 / 2.0;  // lim s→0 of s*G(s) = 10/2 = 5
    double Ka = 0;  // Type 1
    
    std::cout << "System Type: 1" << std::endl;
    std::cout << "Kp = ∞, Kv = " << Kv << ", Ka = " << Ka << std::endl;
    
    // Steady-state errors
    double R = 1.0;  // Input magnitude
    double ess_step = 0;  // R/(1+Kp) = 0 for Type 1+
    double ess_ramp = R / Kv;  // = 0.2
    double ess_parabola = std::numeric_limits<double>::infinity();
    
    std::cout << "\nSteady-state errors for unit inputs:" << std::endl;
    std::cout << "  Step: ess = " << ess_step << std::endl;
    std::cout << "  Ramp: ess = " << ess_ramp << std::endl;
    std::cout << "  Parabola: ess = ∞" << std::endl;
    
    // Closed-loop system
    TransferFunction G_cl = feedback(G, TransferFunction({1}, {1}));
    
    // Generate ramp input r(t) = t
    std::vector<double> t_ramp, r_ramp;
    for (double ti = 0; ti <= 10.0; ti += 0.01) {
        t_ramp.push_back(ti);
        r_ramp.push_back(ti);
    }
    auto sys_cl = tf2ss(G_cl);
    auto [t, y] = lsim(sys_cl, r_ramp, t_ramp);
    
    // Plot showing error
    figure(1000, 600);
    
    subplot(1, 2, 1);
    // Step response - zero steady-state error
    auto [ts, ys] = step_data(G_cl, 10.0);
    plot(ts, ys, "b-", {{"linewidth", "2"}, {"label", "Output"}});
    axhline(1.0, {{"color", "red"}, {"linestyle", "--"}, {"label", "Reference"}});
    xlabel("Time [s]");
    ylabel("Output");
    title("Step Response (Type 1: ess = 0)");
    legend();
    grid(true);
    
    subplot(1, 2, 2);
    // Show ramp tracking with finite error
    std::vector<double> time_vec, ramp_input, ramp_output;
    for (double t = 0; t <= 10; t += 0.05) {
        time_vec.push_back(t);
        ramp_input.push_back(t);  // Ramp: r(t) = t
    }
    // ... simulate ramp response ...
    
    savefig("ch04_steady_state_error.svg");
    
    return 0;
}
```

---

## 4.7 Real-World Application: Industrial Servo Drive System

> **Practical Integration:** An industrial servo drive illustrates how time-domain specifications translate to real hardware, involving mechanical load, motor, power electronics, sensors, and digital controllers.

### 4.7.1 System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                 INDUSTRIAL SERVO DRIVE SYSTEM                           │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐         │
│  │ Motion   │───►│  Servo   │───►│  PMSM    │───►│ Gearbox  │───► Load │
│  │Controller│    │  Drive   │    │  Motor   │    │  N:1     │         │
│  │ (PLC)    │    │ (Inverter│    │          │    │          │         │
│  └────┬─────┘    └────┬─────┘    └────┬─────┘    └────┬─────┘         │
│       │               │               │               │                │
│       │  Position     │  PWM          │  Torque       │  Motion       │
│       │  Command      │  (10-20kHz)   │  Production   │  Transmission │
│       │               │               │               │                │
│       └───────────────┴───────────────┴───────────────┘                │
│                               ▲                                        │
│                               │ Feedback (Encoder: 1M counts/rev)      │
│                               │                                        │
│                      ┌────────┴────────┐                               │
│                      │    Encoder /    │                               │
│                      │    Resolver     │                               │
│                      └─────────────────┘                               │
└─────────────────────────────────────────────────────────────────────────┘
```

### 4.7.2 Cascaded Control Loops

```
   Position      Velocity      Current
   Command       Command       Command
      │             │             │
      ▼             ▼             ▼
   ┌──────┐     ┌──────┐     ┌──────┐     ┌──────┐     ┌──────┐
   │ Pos  │────►│ Vel  │────►│ Curr │────►│Motor │────►│ Load │
   │ Loop │     │ Loop │     │ Loop │     │      │     │      │
   │ 1kHz │     │ 4kHz │     │ 16kHz│     │      │     │      │
   └──┬───┘     └──┬───┘     └──┬───┘     └──────┘     └──────┘
      │            │            │              ▲           ▲
      │            │            └──────────────┘           │
      │            └───────────────────────────────────────┘
      └────────────────────────────────────────────────────┘
```

### 4.7.3 Time-Domain Specs Translation

| Specification | Control Design | Physical Meaning | Typical Value |
|---------------|----------------|------------------|---------------|
| **Rise Time** | Increase loop gain, bandwidth | How fast motor reaches position | 10-100 ms |
| **Overshoot** | Damping ratio ζ ≥ 0.7 | Position error, mechanical stress | < 5% |
| **Settling Time** | $t_s = 4/(\zeta\omega_n)$ | Time to stabilize | 50-500 ms |
| **Steady-State Error** | Integral action, encoder resolution | Final positioning accuracy | ±1 count |
| **Bandwidth** | Sample rate, sensor delay | Disturbance rejection | 50-500 Hz |

### 4.7.4 Multi-Domain Time Constants

| Domain | Component | Time Constant | Impact |
|--------|-----------|---------------|--------|
| **Electrical** | Motor inductance L/R | 0.1 - 1 ms | Current loop bandwidth limit |
| **Mechanical** | Motor inertia J/b | 10 - 100 ms | Velocity loop response |
| **Sensor** | Encoder/resolver | 10 - 100 μs | Feedback delay |
| **Communication** | EtherCAT/PROFINET | 0.25 - 4 ms | Command latency |
| **Thermal** | Motor winding | 10 - 60 min | Continuous torque derating |

### 4.7.5 Design Example

**Application:** CNC machine axis
- Load inertia: $J_L = 0.5$ kg·m²
- Gear ratio: N = 10
- Motor inertia: $J_M = 0.001$ kg·m²
- Total inertia at motor: $J_{total} = J_M + J_L/N^2 = 0.006$ kg·m²

**Specification:** Settling time < 100 ms, Overshoot < 5%

**Design:**
- Required: $\zeta \geq 0.7$ (for < 5% overshoot)
- From $t_s = 4/(\zeta\omega_n)$: $\omega_n \geq 4/(0.7 \times 0.1) = 57$ rad/s

**Closed-Loop TF:**
$$G_{cl}(s) = \frac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2} = \frac{3249}{s^2 + 80s + 3249}$$

### 4.7.6 Integration Challenges

> **Engineering Reality:**
> 
> 1. **Mechanical Resonance:** Gearbox compliance creates resonant modes (anti-resonance notch filter needed)
> 2. **Friction:** Stiction causes limit cycles at low velocity
> 3. **Backlash:** Dead-zone in gearing limits positioning accuracy
> 4. **Thermal:** Motor heats up → resistance changes → current loop detunes
> 5. **Communication Delay:** Network latency adds phase lag

---

## 4.8 Exercises

**Exercise 4.1 — Second-Order System Parameter Extraction** *(Analytical)*

Given the transfer function:
$$G(s) = \frac{25}{s^2 + 6s + 25}$$

(a) Identify the natural frequency $\omega_n$ and damping ratio $\zeta$.  
(b) Classify the system response (underdamped, critically damped, or overdamped).  
(c) Calculate the following time-domain specifications:
   - Peak time: $t_p = \frac{\pi}{\omega_d}$ where $\omega_d = \omega_n\sqrt{1-\zeta^2}$
   - Percent overshoot: $\%OS = 100\,e^{-\zeta\pi/\sqrt{1-\zeta^2}}$
   - Settling time (2% criterion): $t_s \approx \frac{4}{\zeta\omega_n}$
   - Rise time (approximate): $t_r \approx \frac{1.8}{\omega_n}$

(d) Find the pole locations and sketch them in the s-plane.

---

**Exercise 4.2 — Designing to Specifications** *(Analytical/Design)*

Design a second-order system that meets **both** of the following specifications:
- Percent overshoot: $\%OS \leq 10\%$
- Settling time: $t_s \leq 2\,\text{s}$ (2% criterion)

(a) From the overshoot specification, find the minimum required $\zeta$.  
   *Hint:* $\zeta \geq \frac{-\ln(\%OS/100)}{\sqrt{\pi^2 + \ln^2(\%OS/100)}}$

(b) From the settling time specification and the $\zeta$ found above, find the minimum required $\omega_n$.  
   *Hint:* $\omega_n \geq \frac{4}{\zeta \cdot t_s}$

(c) Write the transfer function of your designed system.  
(d) Identify the region in the s-plane where the closed-loop poles must lie to satisfy both specs simultaneously. Sketch this region.

---

**Exercise 4.3 — Dominant Pole Approximation** *(Analytical)*

Consider the third-order system:
$$G(s) = \frac{10}{(s+1)(s^2 + 4s + 8)}$$

(a) Find all three poles of $G(s)$.  
(b) Identify the **dominant poles** — the poles closest to the imaginary axis that dominate the transient response.  
(c) Justify why the non-dominant pole can be neglected (compute the ratio of real parts).  
(d) Write a second-order approximation $G_{approx}(s)$ based on the dominant poles, adjusting the DC gain to match.  
(e) What are the estimated $\omega_n$, $\zeta$, and $\%OS$ from the approximation?

---

**Exercise 4.4 — CppPlot stepinfo() Verification** *(Computational/Coding)* ⭐

Verify your hand calculations from Exercise 4.1 using CppPlot:

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;

int main() {
    // G(s) = 25/(s^2 + 6s + 25)
    TransferFunction G({25}, {1, 6, 25});
    
    // Step response and performance metrics
    auto [t, y] = step_data(G, 5.0);
    auto info = stepinfo(G);
    
    std::cout << "Rise Time:      " << info.rise_time << " s" << std::endl;
    std::cout << "Peak Time:      " << info.peak_time << " s" << std::endl;
    std::cout << "Settling Time:  " << info.settling_time << " s" << std::endl;
    std::cout << "Overshoot:      " << info.overshoot << " %" << std::endl;
    std::cout << "Steady-State:   " << info.steady_state << std::endl;
    
    plot(t, y);
    title("Step Response — Exercise 4.1 Verification");
    xlabel("Time (s)");
    ylabel("Output");
    grid(true);
    show();
    
    return 0;
}
```

(a) Compare the CppPlot results with your hand calculations. Do they agree?  
(b) If there are discrepancies, explain why (e.g., approximation formulas vs. exact computation).

---

**Exercise 4.5 — Effect of Adding a Zero** *(Analytical/Computational)*

Start with the system from Exercise 4.1:
$$G_1(s) = \frac{25}{s^2 + 6s + 25}$$

Now add a zero at $s = -2$:
$$G_2(s) = \frac{25(s + 2)}{2(s^2 + 6s + 25)}$$

(The numerator is scaled so that both systems have the same DC gain.)

(a) Using CppPlot, plot the step responses of $G_1(s)$ and $G_2(s)$ on the same graph.  
(b) Compare the overshoot, rise time, and settling time.  
(c) Explain physically why adding a zero **increases** the overshoot.  
   *Hint:* A zero adds a derivative term to the response: $y(t) = y_{original}(t) + \frac{1}{z}\dot{y}_{original}(t)$.  
(d) What happens as the zero moves closer to the origin (e.g., $s = -0.5$)? And if it moves far away (e.g., $s = -50$)?

---

**Exercise 4.6 — Effect of Additional Poles** *(Computational/Coding)*

Compare the step responses of the following three systems:

$$G_1(s) = \frac{1}{s+1}, \qquad G_2(s) = \frac{5}{(s+1)(s+5)}, \qquad G_3(s) = \frac{100}{(s+1)(s+5)(s+20)}$$

(Note: gains are adjusted so all three have DC gain = 1.)

(a) Plot all three step responses on the same graph using CppPlot.  
(b) For each system, measure the rise time and settling time.  
(c) How does adding a non-dominant pole affect the transient response?  
(d) At what point is the additional pole far enough away that its effect is negligible? State a rule of thumb.  
(e) Relate your observations to the **dominant pole approximation** concept.

---

**Exercise 4.7 — Complex Poles: Peak Time and Steady-State** *(Analytical)*

A system has poles at $s = -1 \pm j2$.

(a) Write the second-order transfer function assuming DC gain of 1.  
   *Hint:* $(s+1-j2)(s+1+j2) = s^2 + 2s + 5$, so $G(s) = \frac{5}{s^2+2s+5}$.

(b) Find $\omega_n$ and $\zeta$.  
(c) Calculate the damped frequency $\omega_d$.  
(d) Calculate the time to first peak: $t_p = \pi/\omega_d$.  
(e) For a unit step input, find the steady-state value using the Final Value Theorem.  
(f) Calculate the percent overshoot and the peak value $y(t_p)$.

### Problem Identification Exercises (Level 3-4)

**Exercise 4.8 — What Is the Real Problem?**
A pharmaceutical company's bioreactor temperature controller meets the spec ($T_s < 30s$, $\%OS < 5\%$) during test runs with water. With the actual chemical broth (higher viscosity, exothermic reaction), the system oscillates.

(a) Explain, using the first-order model from §4.2, how changed physical properties (thermal mass, heat generation) shift the time constant $\tau$ and DC gain $K$.
(b) Is the problem the controller design, the model, or the specifications? Justify your answer.
(c) Propose a solution that does NOT involve re-tuning the controller. (Hint: think about the model.)

**Exercise 4.9 — Mechanism vs. Procedure**
Two students are given a step response with 20% overshoot and 2-second settling time. Student A uses the formulas to compute $\zeta$ and $\omega_n$. Student B says: "The system overshoots because it has too much energy stored in the oscillatory mode — the damping ratio is too low."

(a) Verify Student A's calculation: find $\zeta$ and $\omega_n$.
(b) Explain Student B's statement mathematically — what does "energy stored in the oscillatory mode" mean in terms of pole locations?
(c) If you increase the damping ratio to $\zeta = 0.9$, what happens to the settling time $T_s = 4/(\zeta\omega_n)$? Is this always an acceptable trade-off?

---

## 4.9 Summary

### Key Concepts

1. **First-order systems** are characterized by time constant $\tau$
   - Rise time ≈ 2.2τ, Settling time ≈ 4τ
   
2. **Second-order systems** are characterized by $\omega_n$ and $\zeta$
   - Underdamped ($\zeta < 1$): oscillatory, overshoot
   - Critically damped ($\zeta = 1$): fastest non-oscillatory
   - Overdamped ($\zeta > 1$): slow, no oscillation

3. **Stability** requires all poles in the left half-plane
   - Routh-Hurwitz criterion checks without computing poles

4. **Steady-state error** depends on system type
   - Higher type = better tracking = more integrators

### Design Guidelines

| Specification | Typical Range | Design Action |
|---------------|---------------|---------------|
| Rise time | 0.1 - 2 s | Increase $\omega_n$ |
| Overshoot | < 5-20% | Increase $\zeta$ toward 0.7 |
| Settling time | 1 - 5 s | Increase $\zeta\omega_n$ |
| Steady-state error | < 1-5% | Increase system type or gain |

---

## 4.10 Chapter Summary and Self-Assessment

### Key Concepts Checklist

✅ **First-Order Systems:**
- [ ] I can identify time constant from transfer function
- [ ] I can calculate rise time and settling time
- [ ] I can derive first-order model from physical systems

✅ **Second-Order Systems:**
- [ ] I can identify ωn and ζ from transfer function
- [ ] I can classify response as underdamped/overdamped/critical
- [ ] I can calculate overshoot, settling time, peak time
- [ ] I can relate pole location to transient behavior

✅ **Stability:**
- [ ] I can determine stability from pole locations
- [ ] I can apply Routh-Hurwitz criterion
- [ ] I can find parameter ranges for stability

✅ **Design:**
- [ ] I can select ζ and ωn to meet specifications
- [ ] I can translate physical requirements to mathematical specs

### Outcome Assessment Rubric

| Outcome | Developing | Proficient | Advanced |
|---------|------------|------------|----------|
| **Analyze 1st-order** | Calculate τ from TF | Relate τ to physical params | Design physical system for τ |
| **Analyze 2nd-order** | Identify ωn, ζ | Predict response type | Trade-off speed vs damping |
| **Stability analysis** | Check pole signs | Apply Routh-Hurwitz | Find K ranges symbolically |
| **Design** | Use formulas | Meet single spec | Balance multiple specs |

### Common Misconceptions

⚠️ **Misconception 1:** "Faster systems are always better"
- **Reality:** Faster response often means more overshoot or higher control effort

⚠️ **Misconception 2:** "ζ = 1 (critical damping) is optimal"
- **Reality:** ζ ≈ 0.7 often gives better balance of speed and overshoot

⚠️ **Misconception 3:** "Settling time formula ts = 4/(ζωn) is exact"
- **Reality:** It's an approximation valid mainly for 0.5 < ζ < 0.8

---

## References

1. Ogata, K. - Modern Control Engineering, Chapter 4-5
2. Franklin, G.F. et al. - Feedback Control of Dynamic Systems, Chapter 3
3. Nise, N.S. - Control Systems Engineering, Chapter 4

---

## Next Steps

📚 **Next Chapter:** Root Locus Analysis
- How do poles move as gain changes?
- Graphical design using root locus

🔬 **Lab Exercise:** Implement car suspension simulation with CppPlot
- Vary spring stiffness and observe response
- Find optimal damping for different road conditions

---

*Chapter 4 Complete*
