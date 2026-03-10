# Chapter 3: Laplace Transform and Transfer Functions
## Modern Control Engineering with C++

---

## 🎯 Learning Objectives and Outcomes

### Chapter Objectives

This chapter provides the mathematical foundation for analyzing linear time-invariant (LTI) systems using Laplace transforms and transfer functions.

### Learning Outcomes (Bloom's Taxonomy)

| Level | Outcome | Assessment |
|-------|---------|------------|
| **Remember** | State the definition and properties of the Laplace transform | Quiz |
| **Understand** | Explain the relationship between time domain and s-domain | Concept questions |
| **Apply** | Compute Laplace transforms and inverse transforms | Problem sets |
| **Analyze** | Determine system behavior from transfer function poles and zeros | Analysis exercises |
| **Evaluate** | Assess system stability from transfer function properties | Stability problems |
| **Create** | Derive transfer functions for interconnected systems | Modeling project |

### Prerequisites
- Chapter 2: Mathematical Modeling
- Calculus (differentiation, integration)
- Complex numbers

---

## Why This Chapter Matters: The Engineer's Perspective

> **The Real Problem:** You have a DC motor circuit with $L\frac{di}{dt} + Ri = v(t)$ and need to find current $i(t)$ for any input voltage $v(t)$. Solving this differential equation by hand for every new input is tedious and error-prone.

**The Laplace Transform Solution:**

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    FROM PHYSICS TO ALGEBRA                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   PHYSICAL SYSTEM              LAPLACE TRANSFORM              SOLUTION     │
│   ──────────────               ─────────────────              ────────     │
│                                                                             │
│   Motor circuit:               In s-domain:                   Current:     │
│   L(di/dt) + Ri = v(t)   →    (Ls + R)I(s) = V(s)    →    i(t) = ...     │
│                                                                             │
│   ┌─────────────┐              ┌─────────────┐              ┌──────────┐  │
│   │ Differential│   Laplace    │  Algebraic  │   Inverse   │  Time    │  │
│   │  Equation   │ ──────────►  │  Equation   │ ──────────► │ Solution │  │
│   │  (hard)     │              │  (easy)     │   Laplace   │          │  │
│   └─────────────┘              └─────────────┘              └──────────┘  │
│                                                                             │
│   KEY INSIGHT: Transfer function G(s) = I(s)/V(s) = 1/(Ls+R)              │
│   This ENCAPSULATES the system - use it for ANY input!                    │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Why Control Engineers Love Transfer Functions:**

| Advantage | Explanation |
|-----------|-------------|
| **Reusability** | Once you have $G(s)$, use it for step, ramp, sine, any input |
| **Series connection** | Two systems in series: $G_{total}(s) = G_1(s) \cdot G_2(s)$ (multiplication!) |
| **Feedback analysis** | Closed-loop: $\frac{G}{1+GH}$ - just algebra, not integro-differential equations |
| **Frequency insight** | Substitute $s = j\omega$ to see how system responds to sinusoids |
| **Stability check** | Poles of $G(s)$ directly tell you if system is stable |

---

## 3.1 The Laplace Transform

### Definition

The Laplace transform converts a time-domain function $f(t)$ into a complex frequency-domain function $F(s)$:

$$\mathcal{L}\{f(t)\} = F(s) = \int_0^\infty f(t)e^{-st} dt$$

where $s = \sigma + j\omega$ is the complex frequency variable.

### Why Use Laplace Transform?

```
┌─────────────────────────────────────────────────────────────────┐
│                                                                 │
│  TIME DOMAIN                        S-DOMAIN                    │
│  ───────────                        ────────                    │
│                                                                 │
│  Differential equations    ━━━▶    Algebraic equations         │
│  (hard to solve)                   (easy to solve)              │
│                                                                 │
│  Convolution integral     ━━━▶     Multiplication              │
│  y(t) = h(t) * u(t)                Y(s) = H(s)·U(s)            │
│                                                                 │
│  Initial conditions       ━━━▶     Included automatically       │
│  (separate handling)               in transformed equations     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Common Laplace Transform Pairs

| Time Function $f(t)$ | Laplace Transform $F(s)$ | Notes |
|---------------------|-------------------------|-------|
| $\delta(t)$ (impulse) | $1$ | Unit impulse |
| $u(t)$ (step) | $\frac{1}{s}$ | Unit step |
| $t$ (ramp) | $\frac{1}{s^2}$ | Ramp function |
| $t^n$ | $\frac{n!}{s^{n+1}}$ | Power function |
| $e^{-at}$ | $\frac{1}{s+a}$ | Exponential decay |
| $\sin(\omega t)$ | $\frac{\omega}{s^2+\omega^2}$ | Sine wave |
| $\cos(\omega t)$ | $\frac{s}{s^2+\omega^2}$ | Cosine wave |
| $e^{-at}\sin(\omega t)$ | $\frac{\omega}{(s+a)^2+\omega^2}$ | Damped sine |
| $e^{-at}\cos(\omega t)$ | $\frac{s+a}{(s+a)^2+\omega^2}$ | Damped cosine |

### Key Properties

| Property | Time Domain | S-Domain |
|----------|-------------|----------|
| Linearity | $af(t) + bg(t)$ | $aF(s) + bG(s)$ |
| Differentiation | $\frac{df}{dt}$ | $sF(s) - f(0^-)$ |
| 2nd Derivative | $\frac{d^2f}{dt^2}$ | $s^2F(s) - sf(0^-) - f'(0^-)$ |
| Integration | $\int_0^t f(\tau)d\tau$ | $\frac{F(s)}{s}$ |
| Time shift | $f(t-T)u(t-T)$ | $e^{-sT}F(s)$ |
| Frequency shift | $e^{-at}f(t)$ | $F(s+a)$ |
| Final value | $\lim_{t\to\infty}f(t)$ | $\lim_{s\to 0}sF(s)$ |
| Initial value | $\lim_{t\to 0^+}f(t)$ | $\lim_{s\to\infty}sF(s)$ |
| Convolution | $\int_0^t f_1(\tau)f_2(t-\tau)d\tau$ | $F_1(s) \cdot F_2(s)$ |

> **Convolution Theorem:** If $\mathcal{L}\{f_1(t)\} = F_1(s)$ and $\mathcal{L}\{f_2(t)\} = F_2(s)$, then $\mathcal{L}\{f_1 * f_2\} = F_1(s) \cdot F_2(s)$. This is why multiplication in the s-domain corresponds to cascading systems: the output of G₁(s)G₂(s) is the convolution of the individual impulse responses.

### Value Theorems

Two important theorems allow us to find the initial and final values of $f(t)$ directly from $F(s)$ without computing the inverse transform.

#### Final Value Theorem (FVT)

$$\lim_{t \to \infty} f(t) = \lim_{s \to 0} sF(s)$$

**Condition:** All poles of $sF(s)$ must lie in the **open left half-plane** (i.e., the limit must exist and be finite). If any pole of $sF(s)$ is on the imaginary axis or in the RHP, the FVT does not apply.

**Application:** Finding steady-state values without solving the full inverse transform — essential for computing steady-state error in control systems.

#### Initial Value Theorem (IVT)

$$\lim_{t \to 0^+} f(t) = \lim_{s \to \infty} sF(s)$$

**Condition:** $f(t)$ must have a Laplace transform (i.e., $F(s)$ exists) and $f(t)$ must not contain impulses or higher-order singularities at $t = 0$.

**Application:** Determining the initial value of a system's response directly from its transfer function, useful for verifying initial conditions and detecting improper system behavior.

**Example:** For $F(s) = \frac{5s + 3}{s^2 + 3s + 5}$:
- IVT: $\lim_{s \to \infty} s \cdot \frac{5s+3}{s^2+3s+5} = \lim_{s \to \infty} \frac{5s^2+3s}{s^2+3s+5} = 5$, so $f(0^+) = 5$
- FVT: $\lim_{s \to 0} s \cdot \frac{5s+3}{s^2+3s+5} = \frac{0}{5} = 0$, so $f(\infty) = 0$

---

## 3.2 Transfer Functions

### Definition

The transfer function $G(s)$ is the ratio of the Laplace transform of the output to the input, assuming zero initial conditions:

$$G(s) = \frac{Y(s)}{U(s)} = \frac{\mathcal{L}\{y(t)\}}{\mathcal{L}\{u(t)\}}$$

### What Is a Transfer Function *Really* For?

> **The question many students cannot answer after finishing a control course:** "You have $G(s) = \frac{1}{s+2}$. What is this object? What does it *do*? Why do we need it?"

A transfer function is not just a formula. It is a **prediction machine**.

**Without** a transfer function, every time someone asks "what happens if I apply a step input? a ramp? a 10 Hz sine wave?" — you must solve a differential equation from scratch. Three inputs, three complete solutions.

**With** a transfer function, you solve *once*:

$$Y(s) = G(s) \cdot U(s)$$

Plug in any $U(s)$, get $Y(s)$. The transfer function **encapsulates** the entire input-output behavior of the system in a single algebraic object. It is the system's *fingerprint*.

But the deeper insight is this:

| What $G(s)$ reveals | How | Why it matters |
|---------------------|-----|----------------|
| The system's **intrinsic dynamics** | The poles of $G(s)$ | These are the natural frequencies and decay rates — they exist *regardless* of what input you apply. A motor's time constant is $\tau = J/B$ whether you apply a step, a ramp, or nothing at all. |
| How the system **filters inputs** | Substitute $s = j\omega$ to get $G(j\omega)$ | The system amplifies some frequencies and attenuates others. $G(j\omega)$ tells you exactly which. This is the basis of all frequency-domain design (Chapters 6–8). |
| Whether the system is **stable** | Check if all poles have $\text{Re}(s) < 0$ | If any pole is in the right half-plane, the system's response grows without bound — the motor accelerates until something breaks. (See §3.3 below for *why*.) |
| How to **connect subsystems** | $G_1(s) \cdot G_2(s)$ for series; $\frac{G}{1+GH}$ for feedback | In time domain, connecting two systems requires *convolution* (an integral). In the s-domain, it is *multiplication*. This is why transfer functions revolutionized control engineering. |

> **An analogy:** A transfer function is to a dynamic system what a recipe is to a dish. The recipe (transfer function) does not depend on who is cooking (what input is applied). It captures the *essence* of the system. The same recipe with different ingredients (inputs) produces different dishes (outputs), but the recipe itself is invariant.
>
> Unlike a recipe, however, a transfer function also reveals *why* some dishes fail: the poles tell you about inherent instabilities that no amount of careful cooking (input shaping) can fix. You need to change the recipe (redesign the system).

**The bottom line:** If you have $G(s)$, you can predict the system's response to *any* input, determine its stability, design controllers, and analyze robustness — all using algebra instead of differential equations. That is why every physical system in this book gets converted to a transfer function.

> **\u2194 Connection to Chapter 2:** A transfer function is always a *design model* (Level 2 in the model hierarchy of §2.8.7). It deliberately omits effects like nonlinearity, noise, and parameter variation. Understanding *what* was omitted is critical for knowing when the transfer function's predictions will fail. See §2.8.7 "The Three Levels of Models" for the full framework.

### General Form

$$G(s) = \frac{b_m s^m + b_{m-1}s^{m-1} + \cdots + b_1 s + b_0}{s^n + a_{n-1}s^{n-1} + \cdots + a_1 s + a_0}$$

or in factored form:

$$G(s) = K\frac{(s-z_1)(s-z_2)\cdots(s-z_m)}{(s-p_1)(s-p_2)\cdots(s-p_n)}$$

where:
- $z_i$ are **zeros** (roots of numerator)
- $p_i$ are **poles** (roots of denominator)
- $K$ is the **gain**

### System Order and Type

**Order:** Highest power of $s$ in the denominator = number of energy storage elements

**Type:** Number of poles at $s = 0$ (number of integrators)

```
Type 0:  G(s) = K/(τs + 1)           - Position control
Type 1:  G(s) = K/[s(τs + 1)]        - Velocity control
Type 2:  G(s) = K/[s²(τs + 1)]       - Acceleration control
```

---

## 3.3 Poles, Zeros, and System Response

### Why Must All Poles Be in the Left Half-Plane? — The Physical Explanation

> **The question many students memorize but cannot explain:** "Poles must be in the LHP for stability." But *why*? What is so special about the left side?

The answer is not a convention. It is a direct consequence of **how exponentials behave in time**.

Every pole $s_i$ of the transfer function contributes a term $e^{s_i t}$ to the system's natural response. Since $s_i = \sigma_i + j\omega_i$ is a complex number:

$$e^{s_i t} = e^{(\sigma_i + j\omega_i)t} = \underbrace{e^{\sigma_i t}}_{\text{amplitude envelope}} \cdot \underbrace{e^{j\omega_i t}}_{\text{oscillation}}$$

The imaginary part $\omega_i$ determines the oscillation frequency. But the **real part** $\sigma_i$ determines whether the amplitude grows or dies:

| Real part $\sigma_i$ | Amplitude $e^{\sigma_i t}$ as $t \to \infty$ | Physical meaning |
|---------------------|-----------------------------------------------|------------------|
| $\sigma_i < 0$ (LHP) | $\to 0$ | Energy dissipates — friction absorbs it, resistance converts it to heat. Signal **dies out**. Safe. |
| $\sigma_i = 0$ (imaginary axis) | $= 1$ forever | Energy neither grows nor decays — a perfect, frictionless oscillation. **Marginally stable** — unrealizable in practice. |
| $\sigma_i > 0$ (RHP) | $\to \infty$ | Energy grows — the system produces more energy than it dissipates. Signal **explodes**. Physical consequence: motor spins faster and faster, voltage rails, structure vibrates until failure. |

**The left half-plane is not special because of mathematics. It is special because of physics: only systems that dissipate energy are stable, and dissipation maps to $\sigma < 0$.**

### A Physical Example: Why the Mass-Spring-Damper is Stable

Consider $m\ddot{x} + b\dot{x} + kx = 0$. The characteristic equation is $ms^2 + bs + k = 0$, giving:

$$s = \frac{-b \pm \sqrt{b^2 - 4mk}}{2m}$$

- The spring ($k$) stores energy ↔ creates oscillation ($j\omega$ part)
- The damper ($b$) dissipates energy ↔ pushes poles left ($\sigma < 0$)
- If $b = 0$ (no friction): poles are on the imaginary axis → perpetual oscillation (no energy loss)
- If $b > 0$: poles move into the LHP → oscillation decays → stable
- If $b < 0$ (physically impossible for a passive system, but possible with feedback): poles in RHP → unstable

**This is the mechanism:** Damping = energy dissipation = negative real part = LHP = stable. It is one chain of causation, not five separate facts to memorize.

> **🔑 The One-Sentence Summary:** A pole in the right half-plane means the system generates energy faster than it dissipates it — and nothing in the physical world can grow forever without breaking.
>
> **→ Forward connections:** Chapter 5 (Root Locus) shows how feedback *moves* poles — the entire goal is keeping them in the LHP. Chapter 7 (Bode) provides frequency-domain tools to check *how close* poles are to crossing into the RHP. Chapter 8 (Nyquist) gives the definitive stability test for feedback systems.

### Relationship to Time Response

**Poles determine the natural response modes:**

| Pole Location | Time Response | Stability |
|---------------|---------------|-----------|
| Real negative ($s = -a$) | $e^{-at}$ (decaying) | Stable |
| Real positive ($s = +a$) | $e^{+at}$ (growing) | Unstable |
| Imaginary ($s = \pm j\omega$) | $\sin(\omega t)$, $\cos(\omega t)$ | Marginally stable |
| Complex with negative real ($s = -\sigma \pm j\omega$) | $e^{-\sigma t}\sin(\omega t)$ | Stable |
| Complex with positive real ($s = +\sigma \pm j\omega$) | $e^{+\sigma t}\sin(\omega t)$ | Unstable |

### S-Plane Map

```
                jω
                 │
        Unstable │ Stable
           ×     │     ×
                 │   (decaying
                 │    oscillation)
    ─────────────┼─────────────── σ
                 │
           ×     │     ×
                 │
                 │
        Growing  │  Decaying
        exp.     │  exp.
```

**Stability Rule:** A system is stable if and only if ALL poles have negative real parts (lie in the left half of the s-plane).

> **\u2191 Why?** Because each pole contributes $e^{\sigma t}$ to the response. Negative $\sigma$ = decaying = stable. See the full physical explanation in §3.3 "Why Must All Poles Be in the Left Half-Plane?" above. The consequences of this rule for controller design are explored in Chapter 5 (Root Locus) and Chapter 8 (Nyquist Stability).

### Effect of Zeros

Zeros affect the **shape** of the response (not stability):
- Zeros near poles can cancel their effect
- Zeros in the right half-plane cause **non-minimum phase** behavior (initial response opposite to final direction)

### CppPlot Implementation: Pole-Zero Analysis

```cpp
/**
 * @file ch03_pole_zero.cpp
 * @brief Visualize relationship between pole locations and time response
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <cmath>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║   Chapter 3: Pole-Zero Analysis and Time Response            ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    figure(1400, 1000);
    
    // ════════════════════════════════════════════════════════════════════════
    // Case 1: Real Poles at Different Locations
    // ════════════════════════════════════════════════════════════════════════
    subplot(2, 3, 1);
    
    std::vector<double> real_poles = {-0.5, -1, -2, -5};
    std::vector<std::string> colors = {"#E74C3C", "#E67E22", "#2ECC71", "#3498DB"};
    
    double t_final = 8.0;
    for (size_t i = 0; i < real_poles.size(); ++i) {
        double p = real_poles[i];
        TransferFunction G({-p}, {1, -p});  // G(s) = |p|/(s+|p|), DC gain = 1
        auto [t, y] = step_data(G, t_final);
        
        std::ostringstream label;
        label << "p = " << p;
        plot(t, y, "-", {{"color", colors[i]}, {"linewidth", "2"}, {"label", label.str()}});
    }
    
    xlabel("Time [s]");
    ylabel("Response");
    title("Effect of Real Pole Location");
    legend();
    grid(true);
    
    // ════════════════════════════════════════════════════════════════════════
    // Case 2: Complex Poles (varying damping)
    // ════════════════════════════════════════════════════════════════════════
    subplot(2, 3, 2);
    
    double wn = 2.0;
    std::vector<double> zeta_values = {0.1, 0.3, 0.5, 0.707, 1.0};
    
    for (size_t i = 0; i < zeta_values.size(); ++i) {
        double z = zeta_values[i];
        TransferFunction G({wn*wn}, {1, 2*z*wn, wn*wn});
        auto [t, y] = step_data(G, t_final);
        
        std::ostringstream label;
        label << "ζ = " << std::fixed << std::setprecision(3) << z;
        plot(t, y, "-", {{"linewidth", "2"}, {"label", label.str()}});
    }
    
    axhline(1.0, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.3"}});
    xlabel("Time [s]");
    ylabel("Response");
    title("Effect of Damping Ratio (ωn = 2)");
    legend();
    grid(true);
    
    // ════════════════════════════════════════════════════════════════════════
    // Case 3: S-Plane with Pole Locations
    // ════════════════════════════════════════════════════════════════════════
    subplot(2, 3, 3);
    
    // Draw stability boundary
    axvline(0, {{"color", "red"}, {"linestyle", "--"}, {"linewidth", "2"}, {"label", "Stability boundary"}});
    axhline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    
    // Draw constant damping lines
    for (double z : {0.3, 0.5, 0.707}) {
        double angle = std::acos(z);
        std::vector<double> line_re, line_im_pos, line_im_neg;
        for (double r = 0; r <= 5; r += 0.1) {
            line_re.push_back(-r * std::cos(angle));
            line_im_pos.push_back(r * std::sin(angle));
            line_im_neg.push_back(-r * std::sin(angle));
        }
        plot(line_re, line_im_pos, ":", {{"color", "gray"}, {"alpha", "0.5"}});
        plot(line_re, line_im_neg, ":", {{"color", "gray"}, {"alpha", "0.5"}});
    }
    
    // Draw constant wn circles
    for (double wn_circle : {1, 2, 3}) {
        std::vector<double> circle_re, circle_im;
        for (int j = 0; j <= 50; ++j) {
            double angle = M_PI/2 + j * M_PI / 50;
            circle_re.push_back(wn_circle * std::cos(angle));
            circle_im.push_back(wn_circle * std::sin(angle));
        }
        plot(circle_re, circle_im, ":", {{"color", "blue"}, {"alpha", "0.3"}});
    }
    
    // Plot example poles
    scatter({-1}, {0}, {{"color", "blue"}, {"s", "100"}, {"marker", "x"}});
    scatter({-1, -1}, {1, -1}, {{"color", "green"}, {"s", "100"}, {"marker", "x"}});
    scatter({0.5}, {0}, {{"color", "red"}, {"s", "100"}, {"marker", "x"}});
    
    xlabel("Real (σ)");
    ylabel("Imaginary (jω)");
    title("S-Plane: Pole Locations");
    xlim(-4, 2);
    ylim(-3, 3);
    grid(true);
    
    // ════════════════════════════════════════════════════════════════════════
    // Case 4: Effect of Zero Location
    // ════════════════════════════════════════════════════════════════════════
    subplot(2, 3, 4);
    
    // Fixed poles at -1 ± j1, varying zero
    TransferFunction G_no_zero({2}, {1, 2, 2});  // No zero
    auto [t1, y1] = step_data(G_no_zero, t_final);
    plot(t1, y1, "k-", {{"linewidth", "2"}, {"label", "No zero"}});
    
    // Zero in LHP (faster response)
    TransferFunction G_lhp_zero({2, 4}, {1, 2, 2});  // Zero at s = -2
    auto [t2, y2] = step_data(G_lhp_zero, t_final);
    plot(t2, y2, "b-", {{"linewidth", "2"}, {"label", "Zero at s = -2 (LHP)"}});
    
    // Zero near origin (overshoot)
    TransferFunction G_slow_zero({2, 1}, {1, 2, 2});  // Zero at s = -0.5
    auto [t3, y3] = step_data(G_slow_zero, t_final);
    plot(t3, y3, "g-", {{"linewidth", "2"}, {"label", "Zero at s = -0.5"}});
    
    // Zero in RHP (non-minimum phase)
    TransferFunction G_rhp_zero({2, -2}, {1, 2, 2});  // Zero at s = +1
    auto [t4, y4] = step_data(G_rhp_zero, t_final);
    plot(t4, y4, "r-", {{"linewidth", "2"}, {"label", "Zero at s = +1 (RHP)"}});
    
    axhline(1.0, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.3"}});
    xlabel("Time [s]");
    ylabel("Response");
    title("Effect of Zero Location");
    legend({{"fontsize", "8"}});
    grid(true);
    
    // ════════════════════════════════════════════════════════════════════════
    // Case 5: Unstable Systems
    // ════════════════════════════════════════════════════════════════════════
    subplot(2, 3, 5);
    
    // Stable pole
    TransferFunction G_stable({1}, {1, 1});  // Pole at -1
    auto [ts, ys] = step_data(G_stable, 5.0);
    plot(ts, ys, "g-", {{"linewidth", "2"}, {"label", "Stable: p = -1"}});
    
    // Marginally stable (pole at origin - integrator)
    std::vector<double> t_ramp, y_ramp;
    for (double ti = 0; ti <= 5; ti += 0.05) {
        t_ramp.push_back(ti);
        y_ramp.push_back(ti);  // Ramp response to step
    }
    plot(t_ramp, y_ramp, "orange", {{"linewidth", "2"}, {"label", "Integrator: p = 0"}});
    
    // Unstable
    TransferFunction G_unstable({1}, {1, -0.5});  // Pole at +0.5
    auto [tu, yu] = step_data(G_unstable, 5.0);
    // Clip for visualization
    std::vector<double> yu_clip;
    for (double val : yu) {
        yu_clip.push_back(std::min(val, 20.0));
    }
    plot(tu, yu_clip, "r-", {{"linewidth", "2"}, {"label", "Unstable: p = +0.5"}});
    
    xlabel("Time [s]");
    ylabel("Response");
    title("Stability Comparison");
    legend();
    grid(true);
    ylim(-1, 10);
    
    // ════════════════════════════════════════════════════════════════════════
    // Case 6: Higher Order Systems
    // ════════════════════════════════════════════════════════════════════════
    subplot(2, 3, 6);
    
    // First order
    TransferFunction G1({1}, {1, 1});
    auto [t_1, y_1] = step_data(G1, t_final);
    plot(t_1, y_1, "-", {{"linewidth", "2"}, {"label", "1st order"}});
    
    // Second order
    TransferFunction G2({1}, {1, 1.4, 1});  // ζ = 0.7
    auto [t_2, y_2] = step_data(G2, t_final);
    plot(t_2, y_2, "-", {{"linewidth", "2"}, {"label", "2nd order"}});
    
    // Third order
    TransferFunction G3({1}, {1, 2, 2, 1});
    auto [t_3, y_3] = step_data(G3, t_final);
    plot(t_3, y_3, "-", {{"linewidth", "2"}, {"label", "3rd order"}});
    
    // Fourth order
    TransferFunction G4({1}, {1, 2.6, 3.4, 2.6, 1});
    auto [t_4, y_4] = step_data(G4, t_final);
    plot(t_4, y_4, "-", {{"linewidth", "2"}, {"label", "4th order"}});
    
    axhline(1.0, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.3"}});
    xlabel("Time [s]");
    ylabel("Response");
    title("System Order Comparison");
    legend();
    grid(true);
    
    savefig("ch03_pole_zero_analysis.svg");
    std::cout << "\n✓ Saved ch03_pole_zero_analysis.svg" << std::endl;
    
    return 0;
}
```

---

## 3.4 Block Diagram Algebra

### Basic Connections

**Series Connection:**
```
U(s) ──▶[G₁(s)]──▶[G₂(s)]──▶ Y(s)

Equivalent: G(s) = G₁(s) · G₂(s)
```

**Parallel Connection:**
```
        ┌──▶[G₁(s)]──┐
U(s) ──┤            ├──(+)──▶ Y(s)
        └──▶[G₂(s)]──┘

Equivalent: G(s) = G₁(s) + G₂(s)
```

**Feedback Connection:**
```
            ┌───────────────────────────┐
            │                           │
R(s) ──(+)──┴──▶[G(s)]──┬──▶ Y(s)      │
        ↑               │              │
        │ -             │              │
        └───[H(s)]◀─────┘              │
                                       │
Equivalent: Y(s)/R(s) = G(s)/(1 + G(s)H(s))
```

### CppPlot Implementation: Block Diagram Operations

```cpp
// In CppPlot control library:

// Series connection
TransferFunction G1({1}, {1, 1});
TransferFunction G2({2}, {1, 2});
TransferFunction G_series = G1 * G2;  // G1(s) * G2(s)

// Parallel connection
TransferFunction G_parallel = G1 + G2;

// Feedback connection
TransferFunction G_closed = feedback(G1, G2);  // G1/(1 + G1*G2)
TransferFunction G_closed_unity = feedback(G1, TransferFunction({1}, {1}));  // G1/(1 + G1)
```

---

## 3.5 Partial Fraction Expansion

### Purpose

Convert complex transfer functions into simpler terms for inverse Laplace transform.

### For Distinct Poles

$$\frac{N(s)}{(s-p_1)(s-p_2)\cdots(s-p_n)} = \frac{A_1}{s-p_1} + \frac{A_2}{s-p_2} + \cdots + \frac{A_n}{s-p_n}$$

**Residue Formula:**
$$A_i = \lim_{s \to p_i}(s-p_i)\frac{N(s)}{D(s)}$$

### For Repeated Poles

If $p_1$ has multiplicity $r$:

$$\frac{N(s)}{(s-p_1)^r \cdots} = \frac{A_{1r}}{(s-p_1)^r} + \frac{A_{1,r-1}}{(s-p_1)^{r-1}} + \cdots + \frac{A_{11}}{s-p_1} + \cdots$$

**Coefficient Formula for Repeated Roots:**

The coefficients $A_{1k}$ (for $k = 1, 2, \ldots, r$) are computed by:

$$A_{1k} = \frac{1}{(r-k)!} \frac{d^{r-k}}{ds^{r-k}}\left[(s-p_1)^r \frac{N(s)}{D(s)}\right]\Bigg|_{s=p_1}$$

where $D(s)$ is the full denominator polynomial.

> **Derivation:** Multiply both sides by $(s - p_1)^r$:
> $$G(s) = (s-p_1)^r \frac{N(s)}{D(s)} = A_{1r} + A_{1,r-1}(s-p_1) + \cdots + A_{11}(s-p_1)^{r-1} + \cdots$$
>
> Setting $s = p_1$ gives $A_{1r}$ directly. Differentiating once and setting $s = p_1$ gives $A_{1,r-1}$, and so on. After $r-k$ differentiations:
> $$\frac{d^{r-k}}{ds^{r-k}} G(s) \bigg|_{s=p_1} = (r-k)! \cdot A_{1k}$$

**Worked Example:** Find the partial fractions of $F(s) = \frac{2}{s(s+1)^3}$.

Expansion form:
$$F(s) = \frac{A_0}{s} + \frac{A_3}{(s+1)^3} + \frac{A_2}{(s+1)^2} + \frac{A_1}{(s+1)}$$

- $A_0 = \left[s \cdot F(s)\right]_{s=0} = \frac{2}{(0+1)^3} = 2$
- $A_3 = \left[(s+1)^3 F(s)\right]_{s=-1} = \frac{2}{s}\bigg|_{s=-1} = -2$
- $A_2 = \frac{d}{ds}\left[\frac{2}{s}\right]_{s=-1} = \frac{-2}{s^2}\bigg|_{s=-1} = -2$
- $A_1 = \frac{1}{2!}\frac{d^2}{ds^2}\left[\frac{2}{s}\right]_{s=-1} = \frac{1}{2} \cdot \frac{4}{s^3}\bigg|_{s=-1} = \frac{1}{2}\cdot(-4) = -2$

Thus: $F(s) = \frac{2}{s} - \frac{2}{(s+1)^3} - \frac{2}{(s+1)^2} - \frac{2}{(s+1)}$

Inverse Laplace: $f(t) = 2 - 2\frac{t^2}{2}e^{-t} - 2te^{-t} - 2e^{-t} = 2 - (t^2 + 2t + 2)e^{-t}$

### For Complex Conjugate Poles

When a transfer function has complex conjugate poles $s = -\alpha \pm j\beta$, completing the square and using the Laplace transform pair for damped sinusoids avoids the need for complex-valued residues.

**Worked Example:** Find the inverse Laplace transform of:
$$F(s) = \frac{1}{s^2 + 2s + 5}$$

**Step 1 — Complete the square in the denominator:**
$$s^2 + 2s + 5 = (s+1)^2 + 4 = (s+1)^2 + 2^2$$

The poles are at $s = -1 \pm j2$ (i.e., $\alpha = 1$, $\beta = 2$).

**Step 2 — Match to the standard Laplace pair:**
$$\mathcal{L}^{-1}\left\{\frac{\beta}{(s+\alpha)^2 + \beta^2}\right\} = e^{-\alpha t}\sin(\beta t)$$

Rewrite $F(s)$ with $\beta = 2$ in the numerator:
$$F(s) = \frac{1}{(s+1)^2 + 2^2} = \frac{1}{2} \cdot \frac{2}{(s+1)^2 + 2^2}$$

**Step 3 — Inverse Laplace transform:**
$$\boxed{f(t) = \frac{1}{2}e^{-t}\sin(2t), \quad t \geq 0}$$

> **Key insight:** For any $F(s)$ with complex conjugate poles $s = -\alpha \pm j\beta$ and no finite zeros, the inverse Laplace transform will contain the term $e^{-\alpha t}\sin(\beta t)$ — a damped sinusoid. The decay rate is set by $\alpha$ (the real part) and the oscillation frequency by $\beta$ (the imaginary part). This pattern appears throughout control engineering: it is the natural response of any underdamped second-order system.

### Example: Partial Fraction Expansion

```cpp
/**
 * @file ch03_partial_fraction.cpp
 * @brief Demonstrate partial fraction expansion and inverse Laplace
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <cmath>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║   Chapter 3: Partial Fraction Expansion                      ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // Example: G(s) = 2(s+3) / [(s+1)(s+2)]
    // Step input: Y(s) = G(s) * (1/s) = 2(s+3) / [s(s+1)(s+2)]
    
    // Partial fractions:
    // Y(s) = A/s + B/(s+1) + C/(s+2)
    // A = 2*3/(1*2) = 3
    // B = 2*2/(-1*1) = -4
    // C = 2*1/(-2*-1) = 1
    // y(t) = 3 - 4*exp(-t) + exp(-2t)
    
    std::cout << "\nExample: G(s) = 2(s+3) / [(s+1)(s+2)]" << std::endl;
    std::cout << "Step response: Y(s) = 2(s+3) / [s(s+1)(s+2)]" << std::endl;
    std::cout << "\nPartial Fractions:" << std::endl;
    std::cout << "  Y(s) = 3/s - 4/(s+1) + 1/(s+2)" << std::endl;
    std::cout << "\nInverse Laplace:" << std::endl;
    std::cout << "  y(t) = 3 - 4e^(-t) + e^(-2t)" << std::endl;
    
    TransferFunction G({2, 6}, {1, 3, 2});
    
    auto poles = G.poles();
    auto zeros = G.zeros();
    
    std::cout << "\nPoles: ";
    for (auto& p : poles) std::cout << p << " ";
    std::cout << "\nZeros: ";
    for (auto& z : zeros) std::cout << z << " ";
    std::cout << std::endl;
    
    // Simulate step response
    double t_final = 6.0;
    auto [t, y_sim] = step_data(G, t_final);
    
    // Analytical solution
    std::vector<double> y_analytical;
    for (double ti : t) {
        double y = 3 - 4*std::exp(-ti) + std::exp(-2*ti);
        y_analytical.push_back(y);
    }
    
    // Individual components
    std::vector<double> y_dc, y_mode1, y_mode2;
    for (double ti : t) {
        y_dc.push_back(3);
        y_mode1.push_back(-4*std::exp(-ti));
        y_mode2.push_back(std::exp(-2*ti));
    }
    
    figure(1000, 600);
    
    subplot(1, 2, 1);
    plot(t, y_sim, "b-", {{"linewidth", "3"}, {"label", "Simulation"}});
    plot(t, y_analytical, "r--", {{"linewidth", "2"}, {"label", "Analytical"}});
    xlabel("Time [s]");
    ylabel("Response y(t)");
    title("Step Response Comparison");
    legend();
    grid(true);
    
    subplot(1, 2, 2);
    plot(t, y_dc, "g--", {{"linewidth", "1.5"}, {"label", "DC: 3"}});
    plot(t, y_mode1, "r--", {{"linewidth", "1.5"}, {"label", "Mode 1: -4e^(-t)"}});
    plot(t, y_mode2, "b--", {{"linewidth", "1.5"}, {"label", "Mode 2: e^(-2t)"}});
    plot(t, y_sim, "k-", {{"linewidth", "2"}, {"label", "Sum"}});
    axhline(0, {{"color", "gray"}, {"linestyle", ":"}});
    xlabel("Time [s]");
    ylabel("Component");
    title("Partial Fraction Components");
    legend();
    grid(true);
    
    savefig("ch03_partial_fraction.svg");
    std::cout << "\n✓ Saved ch03_partial_fraction.svg" << std::endl;
    
    return 0;
}
```

---

## 3.6 CppPlot Transfer Function Class

### Creating Transfer Functions

```cpp
#include <cppplot/control/control.hpp>
using namespace cppplot::control;

// Polynomial form: G(s) = (2s + 1)/(s² + 3s + 2)
TransferFunction G1({2, 1}, {1, 3, 2});

// From poles and zeros with gain
TransferFunction G2 = zpk({-0.5}, {-1, -2}, 2.0);

// From numerator and denominator polynomials
std::vector<double> num = {1};
std::vector<double> den = {1, 2, 1};
TransferFunction G3(num, den);
```

### Transfer Function Operations

```cpp
// Get poles and zeros
auto poles = G.poles();      // Returns vector<complex<double>>
auto zeros = G.zeros();

// Arithmetic operations
auto G_series = G1 * G2;     // Cascade
auto G_parallel = G1 + G2;   // Parallel sum
auto G_scaled = 2.0 * G1;    // Gain scaling

// Feedback connections
auto G_cl = feedback(G, H);     // G/(1 + GH)
auto G_unity = feedback(G, TransferFunction({1}, {1}));     // G/(1 + G)

// Time responses
auto [t, y] = step_data(G, 10.0);       // Step response
auto [ti, yi] = impulse_data(G, 10.0);  // Impulse response
auto [tr, yr] = lsim(G, u, t);     // Arbitrary input

// Frequency response
bode(G);  // Plots Bode diagram directly (bode() returns void)
```

---

## 3.7 Physical Interpretation: What Poles and Zeros Really Mean

> **Q4: How to Implement?** The mathematics of Laplace transforms connects directly to physical quantities you can measure and design for.

### 3.7.1 Poles = Natural Behaviors of the System

Every pole corresponds to a natural mode of the physical system:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    POLES AND THEIR PHYSICAL MEANING                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   POLE LOCATION                PHYSICAL MEANING                            │
│   ─────────────                ────────────────                            │
│                                                                             │
│   Real pole at s = -1/τ        Exponential decay with time constant τ      │
│                                Example: RC circuit, thermal system          │
│                                τ = RC for electrical, τ = mc/hA for thermal │
│                                                                             │
│   Complex poles                Oscillation + decay                          │
│   s = -σ ± jωd                 σ = damping rate (how fast oscillation dies) │
│                                ωd = damped frequency (Hz of oscillation)    │
│                                Example: Mass-spring-damper, LC circuit      │
│                                                                             │
│   Pole at origin s = 0         Integration (accumulator)                    │
│                                Example: Tank level, motor position          │
│                                                                             │
│   RHP pole s = +a              Exponential growth (UNSTABLE)               │
│                                Example: Inverted pendulum, unstable aircraft│
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Practical Example - DC Motor:**

```
Motor transfer function: G(s) = Km / [(Js + B)(Ls + R) + KmKe]

Physical parameters you can MEASURE:
- J = rotor inertia (kg·m²) ← weigh the rotor, measure geometry
- B = friction coefficient (N·m·s/rad) ← spin motor, measure deceleration
- L = armature inductance (H) ← LCR meter
- R = armature resistance (Ω) ← multimeter
- Km = torque constant (N·m/A) ← apply current, measure torque
- Ke = back-EMF constant (V·s/rad) ← spin motor, measure voltage

From these, you can PREDICT the poles and time response!
```

**Signal Dictionary — DC Motor (Armature Circuit)**

| Signal | Symbol | Unit | Physical Meaning | Sensor/Actuator |
|--------|--------|------|------------------|-----------------|
| Armature voltage (input) | $v(t)$ | V | Voltage applied to motor terminals | Power amplifier / H-Bridge |
| Armature current | $i(t)$ | A | Current flowing through the winding — produces torque | Current sensor (shunt resistor / Hall effect) |
| Angular velocity (output) | $\omega(t)$ | rad/s | Rotor shaft speed | Tachometer / encoder derivative |
| Back-EMF voltage | $e_b = K_e\omega$ | V | Voltage generated by the spinning rotor — opposes applied voltage | (Internal — not directly measured) |
| Electromagnetic torque | $T_e = K_m i$ | N·m | Torque produced by the motor | (Internal — inferred from current) |
| Friction torque | $T_f = B\omega$ | N·m | Resistive torque from bearings and windage | (Internal — measured by spin-down test) |
| Rotor inertia | $J$ | kg·m² | Resistance to angular acceleration | (Parameter — weigh rotor, measure geometry) |

> **Where do the poles come from?** The electrical pole $s = -R/L$ comes from the armature RL circuit. The mechanical pole $s = -B/J$ comes from friction vs. inertia. The Laplace transform converts these physical time constants into pole locations on the s-plane. Change $J$ → pole moves. That is the connection between hardware and mathematics.

### 3.7.2 Zeros = How Input Affects Output Path

Zeros determine how the input signal flows through the system:

| Zero Type | Physical Meaning | Example |
|-----------|-----------------|---------|
| **LHP zero** | Derivative action, speeds up response | Tachometer feedback adds zero |
| **RHP zero** | Non-minimum phase, initial wrong-way response | Flexible spacecraft, boiler drum level |
| **Zero at origin** | Blocks DC (differentiator) | High-pass filter, rate sensor |

**Non-Minimum Phase (RHP Zero) - Real Example:**

```
BOILER DRUM LEVEL CONTROL:
- Increase feed water → Level initially DROPS (shrink effect from cold water)
- Then level rises to new value

G(s) = K(τ₁s - 1)/[(τ₂s + 1)(τ₃s + 1)]
       ↑
    RHP zero at s = 1/τ₁ causes "wrong way" initial response

Implication: Must limit control speed, or you'll overcorrect!
```

#### Mathematical Explanation of RHP Zero Wrong-Way Behavior

A right-half-plane (RHP) zero — a zero with positive real part — causes an **initial inverse response** (undershoot) before the output settles to its final value.

**Canonical example:** Consider
$$G(s) = \frac{1 - s}{(1 + s)^2}$$

This has a zero at $s = +1$ (RHP) and poles at $s = -1$ (stable, repeated).

Partial fraction of the unit step response $Y(s) = G(s)/s$:
$$Y(s) = \frac{1-s}{s(1+s)^2}$$

The inverse Laplace transform yields:
$$y(t) = 1 - 2te^{-t} - e^{-t} = 1 - (1 + 2t)e^{-t}$$

At $t = 0$: $y(0) = 1 - 1 = 0$, as expected (the output starts at zero). However, the **initial slope** is $y'(0) = -1$ (negative!), meaning the output initially moves in the **opposite direction** before recovering.

**Physical intuition:** Write $G(s) = \frac{1}{(1+s)^2} - \frac{s}{(1+s)^2}$. The first term is the expected response; the second term $-sG_0(s)$ is a **negative derivative** component that dominates initially, pulling the response in the wrong direction.

**Control implications:**
- RHP zeros impose a fundamental **upper limit on achievable bandwidth**: approximately $\omega_{BW} < z/2$ where $z$ is the RHP zero location.
- Attempting fast control (high bandwidth) with RHP zeros leads to large undershoot or instability.
- This is a **plant limitation**, not fixable by any controller.

### 3.7.3 From Transfer Function to System Identification

> **Key Insight:** You can work BACKWARDS - measure the response, find the transfer function, then extract physical parameters.
>
> **→ Preview of Chapter 9:** This section gives a first taste of system identification — extracting model parameters from step response data. Chapter 9 covers this topic comprehensively: time-domain and frequency-domain identification, parametric methods (least squares, ARX models), input signal design (PRBS, chirp), and rigorous model validation.

**Step Response Identification for First-Order System:**

```cpp
/**
 * @brief Extract time constant from measured step response
 * 
 * Physical meaning: τ is when response reaches 63.2% of final value
 * For RC circuit: τ = RC
 * For thermal system: τ = mc/(hA)
 */

// From measured step response data
double y_final = measured_data.back();
double y_63percent = 0.632 * y_final;

// Find time when output reaches 63.2%
double tau_measured = 0;
for (size_t i = 0; i < time.size(); ++i) {
    if (measured_data[i] >= y_63percent) {
        tau_measured = time[i];
        break;
    }
}

// Now you know: G(s) = K/(τs + 1) where K = y_final, τ = tau_measured
std::cout << "Identified parameters:" << std::endl;
std::cout << "  DC Gain K = " << y_final << std::endl;
std::cout << "  Time constant τ = " << tau_measured << " seconds" << std::endl;

// For thermal system, if you know mass m and specific heat c:
double hA = m * c / tau_measured;  // Heat transfer coefficient × Area
std::cout << "  Heat transfer hA = " << hA << " W/K" << std::endl;
```

### 3.7.4 Practical Limitations: When Theory Meets Reality

```
┌─────────────────────────────────────────────────────────────────────────────┐
│              TRANSFER FUNCTION ASSUMPTIONS vs REALITY                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ASSUMPTION                    REALITY                      IMPACT        │
│   ──────────                    ───────                      ──────        │
│   Linear system                 Saturation, dead zones       Model valid   │
│                                                              only in range │
│                                                                             │
│   Time-invariant                Parameters drift with        Robust design │
│                                 temperature, wear            needed        │
│                                                                             │
│   Lumped parameters             Distributed systems at       High-freq     │
│                                 high frequency               errors        │
│                                                                             │
│   Continuous signals            Digital implementation       Sampling      │
│                                 is discrete                  effects       │
│                                                                             │
│   Perfect measurements          Sensor noise, delays         Filter design │
│                                                              needed        │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 3.8 Electrical and Telecommunications System Examples

### 3.8.1 Electrical Engineering: Power Converter Dynamics

**Example: Boost Converter Transfer Functions**

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    BOOST CONVERTER SMALL-SIGNAL MODEL                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   V_in ──┬──[L]──┬──[D]──┬────[R]──── V_out                               │
│          │       │       │                                                  │
│         [S]      │      ═╪═ C                                              │
│          │       │       │                                                  │
│   GND ───┴───────┴───────┴───────────                                      │
│                                                                             │
│   TRANSFER FUNCTIONS (averaged model):                                     │
│                                                                             │
│   Control-to-Output:                                                       │
│   Gvd(s) = Vg/(1-D) · (1 - s·L/((1-D)²R)) / [LC·s²/(1-D)² + L·s/((1-D)²R) + 1] │
│                          ↑ RHP zero at s = (1-D)²R/L                       │
│                        RHP ZERO! (non-minimum phase)                       │
│                                                                             │
│   This RHP zero limits control bandwidth!                                  │
│   Design implication: Cannot use aggressive high-gain control              │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

**CppPlot Analysis:**

```cpp
/**
 * @file ch03_boost_converter.cpp
 * @brief Boost converter transfer function analysis
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
using namespace cppplot;
using namespace cppplot::control;

int main() {
    // Boost converter parameters
    double V_in = 12.0, V_out = 24.0;
    double D = 1 - V_in/V_out;  // Duty cycle = 0.5
    double L = 100e-6, C = 100e-6, R = 10.0;
    
    // Control-to-output transfer function
    // Gvd(s) has form: K(1 - s/ωz) / (s²/ωn² + 2ζs/ωn + 1)
    double Dp = 1 - D;  // D' = 1-D
    
    // RHP zero frequency
    double w_rhpz = Dp*Dp * R / L;
    std::cout << "RHP Zero at: s = +" << w_rhpz << " rad/s" << std::endl;
    std::cout << "Maximum control bandwidth ≈ " << w_rhpz/5 << " rad/s" << std::endl;
    
    // Natural frequency and damping
    double wn = Dp / std::sqrt(L*C);
    double Q = Dp * R * std::sqrt(C/L);
    
    // Transfer function coefficients
    double K = V_in / (Dp*Dp);
    TransferFunction Gvd({-K/w_rhpz, K}, {L*C/(Dp*Dp), L/(Dp*Dp*R), 1});
    
    // Step response shows RHP zero effect
    auto [t, v] = step_data(Gvd, 0.01);
    
    figure(800, 400);
    plot(t, v, "b-", {{"linewidth", "2"}});
    xlabel("Time [s]");
    ylabel("Δv_out/Δd");
    title("Boost Converter: RHP Zero Causes Initial Wrong-Way Response");
    grid(true);
    savefig("ch03_boost_rhpz.svg");
    
    return 0;
}
```

### 3.8.2 Telecommunications: PLL Transfer Functions

**Phase-Locked Loop Linear Model:**

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    PLL TRANSFER FUNCTION ANALYSIS                           │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   Open-Loop Transfer Function (Type II PLL with PI loop filter):           │
│                                                                             │
│   G(s) = Kpd × F(s) × Kvco/s × 1/N                                        │
│                                                                             │
│   where F(s) = (1 + s/ωz)/(s/ωp) for PI filter                            │
│                                                                             │
│   G(s) = Kpd·Kvco·(s + ωz) / (N·s²)                                       │
│                                                                             │
│   Two integrators → Type II system → Zero static phase error              │
│                                                                             │
│   ─────────────────────────────────────────────────────────────────────    │
│                                                                             │
│   Closed-Loop Phase Transfer:                                              │
│                                                                             │
│   H(s) = θ_out/θ_ref = N·G/(1+G)                                          │
│                                                                             │
│        = N·Kpd·Kvco·(s + ωz) / [N·s² + Kpd·Kvco·(s + ωz)]                │
│                                                                             │
│   Standard form: H(s) = ωn²(2ζs/ωn + 1) / (s² + 2ζωn·s + ωn²)            │
│                                                                             │
│   where: ωn = √(Kpd·Kvco·ωz/N)  (natural frequency)                       │
│          ζ = (1/2)·√(Kpd·Kvco/(N·ωz))  (damping ratio)                    │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Design Equations:**

Given desired $\omega_n$ and $\zeta$:

$$K_{pd} \cdot K_{vco} = \frac{N \cdot \omega_n^2}{\omega_z}$$

$$\omega_z = \frac{\omega_n}{2\zeta}$$

**CppPlot Implementation:**

```cpp
/**
 * @file ch03_pll_transfer.cpp
 * @brief PLL closed-loop transfer function design
 */

int main() {
    // Design specs
    double wn = 2*M_PI*1000;  // 1 kHz natural frequency
    double zeta = 0.707;       // Critical damping
    double N = 100;            // Divider ratio
    double Kvco = 2*M_PI*1e6;  // VCO gain: 1 MHz/V
    
    // Derived parameters
    double wz = wn / (2*zeta);                    // Loop filter zero
    double Kpd_Kvco = N * wn * wn / wz;          // Product
    double Kpd = Kpd_Kvco / Kvco;                // Phase detector gain
    
    // Closed-loop transfer function
    // H(s) = ωn²(2ζ/ωn·s + 1) / (s² + 2ζωn·s + ωn²)
    TransferFunction H({2*zeta*wn, wn*wn}, {1, 2*zeta*wn, wn*wn});
    
    // Phase step response (reference phase jump)
    auto [t, theta] = step_data(H * N, 0.005);  // 5 ms simulation
    
    // Lock time (settling to 1% of final)
    double ts = 4.6 / (zeta * wn);
    std::cout << "Theoretical settling time: " << ts*1000 << " ms" << std::endl;
    
    // Bandwidth (3 dB frequency)
    double w3dB = wn * std::sqrt(1 + 2*zeta*zeta + 
                  std::sqrt((1 + 2*zeta*zeta)*(1 + 2*zeta*zeta) + 1));
    std::cout << "3 dB bandwidth: " << w3dB/(2*M_PI) << " Hz" << std::endl;
    
    return 0;
}
```

### 3.8.3 Filter Design Using Transfer Functions

**Active Low-Pass Filter (Butterworth):**

```cpp
/**
 * @brief 2nd-order Butterworth low-pass filter design
 * 
 * H(s) = ωc² / (s² + √2·ωc·s + ωc²)
 * 
 * Poles at: s = -ωc/√2 ± j·ωc/√2  (45° angle, maximally flat)
 */

double fc = 1000;  // Cutoff frequency 1 kHz
double wc = 2*M_PI*fc;

// Butterworth: ζ = 1/√2 = 0.707
TransferFunction H_butter({wc*wc}, {1, std::sqrt(2)*wc, wc*wc});

// Op-amp circuit realization (Sallen-Key topology):
// Given: R1 = R2 = R, C1 = C2 = C
// fc = 1/(2π·R·C), and gain = 1 for unity-gain Butterworth

double R = 10e3;  // Choose 10 kΩ
double C = 1.0/(2*M_PI*fc*R);  // Solve for C
std::cout << "Component values: R = " << R/1000 << " kΩ, C = " << C*1e9 << " nF" << std::endl;
```

### 3.8.4 Transmission Line Model

For long cables in communication systems:

$$H(s) = e^{-s\tau} \cdot e^{-\alpha \sqrt{s}}$$

**Approximation (Padé delay):**

$$e^{-s\tau} \approx \frac{1 - s\tau/2}{1 + s\tau/2} \quad \text{(first-order Padé)}$$

```cpp
// Cable delay approximation
double tau = 10e-9;  // 10 ns delay
double alpha = 0.1;  // Attenuation factor

// First-order Padé approximation for delay
TransferFunction H_delay({1, -tau/2}, {1, tau/2});

// Combined with loss (first-order model)
TransferFunction H_cable = H_delay * TransferFunction({1}, {alpha/2, 1});

// This is useful for stability analysis of long communication links
```

---

## 3.9 Exercises

**Exercise 3.1 — Laplace Transform Using Properties** *(Analytical)*

Find the Laplace transform of:
$$f(t) = t\,e^{-2t}\sin(3t)$$

*Hint:* Use the following properties systematically:
1. $\mathcal{L}\{\sin(3t)\} = \frac{3}{s^2 + 9}$
2. Frequency shifting: $\mathcal{L}\{e^{-at}f(t)\} = F(s+a)$
3. Multiplication by $t$: $\mathcal{L}\{t\,f(t)\} = -\frac{d}{ds}F(s)$

Show all intermediate steps and simplify to a single rational function.

---

**Exercise 3.2 — Inverse Laplace via Partial Fractions** *(Analytical)*

Find the inverse Laplace transform of:
$$F(s) = \frac{3s + 5}{(s+1)(s+2)(s+3)}$$

(a) Decompose $F(s)$ into partial fractions:  
$$F(s) = \frac{A}{s+1} + \frac{B}{s+2} + \frac{C}{s+3}$$

(b) Solve for $A$, $B$, and $C$ using the cover-up method or simultaneous equations.  
(c) Write the time-domain function $f(t)$ for $t \geq 0$.  
(d) Verify that $f(0^+) = F(s)\cdot s\big|_{s\to\infty}$ matches your result.

---

**Exercise 3.3 — Solving an ODE Using Laplace** *(Analytical)*

Solve the following ODE using the Laplace transform method:
$$y'' + 3y' + 2y = e^{-t}, \qquad y(0) = 1,\; y'(0) = 0$$

(a) Take the Laplace transform of both sides, incorporating the initial conditions.  
(b) Solve for $Y(s)$ algebraically.  
(c) Perform partial fraction decomposition of $Y(s)$.  
(d) Find $y(t)$ by inverse Laplace transform.  
(e) Verify: does $y(0) = 1$? Does $y'(0) = 0$?

---

**Exercise 3.4 — Transfer Function from ODE** *(Analytical)*

Using the ODE from Exercise 3.3:
$$y'' + 3y' + 2y = e^{-t}$$

(a) Assuming **zero initial conditions**, derive the transfer function $G(s) = Y(s)/U(s)$ where $u(t) = e^{-t}$.  
(b) Find the poles and zeros of $G(s)$.  
(c) Is the system stable? Justify using pole locations.  
(d) What is the system order? Does the input $u(t) = e^{-t}$ introduce any pole-zero cancellation? Discuss the implications.

---

**Exercise 3.5 — Final Value Theorem** *(Analytical)*

Consider the transfer function:
$$G(s) = \frac{5}{s^2 + 3s + 5}$$

(a) For a unit step input $U(s) = 1/s$, write $Y(s) = G(s) \cdot U(s)$.  
(b) Verify that the Final Value Theorem (FVT) conditions are satisfied: all poles of $sY(s)$ must be in the left half-plane.  
(c) Apply the FVT to find the steady-state output:
$$y_{ss} = \lim_{s \to 0} s\,Y(s)$$

(d) What is the steady-state error for a unit step input?  
(e) How would adding an integrator (changing the plant to $G(s) = \frac{5}{s(s^2+3s+5)}$) affect the steady-state error for a step input?

---

**Exercise 3.6 — FVT Verification with CppPlot** *(Computational/Coding)* ⭐

Verify your Final Value Theorem result from Exercise 3.5 using CppPlot:

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;

int main() {
    // G(s) = 5/(s^2 + 3s + 5)
    TransferFunction G({5}, {1, 3, 5});
    
    // Unit step response
    auto [t, y] = step_data(G, 20.0);
    plot(t, y);
    title("Step Response — FVT Verification");
    xlabel("Time (s)");
    ylabel("Output y(t)");
    grid(true);
    show();
    
    // Print final value from simulation
    std::cout << "Simulated steady-state value: " << y.back() << std::endl;
    std::cout << "FVT prediction: " << 5.0/5.0 << std::endl;  // s->0: 5/5 = 1
    
    return 0;
}
```

(a) Run the program and confirm the steady-state value matches your FVT calculation.  
(b) How long does it take the system to reach within 2% of the final value?  
(c) Is the response oscillatory? Relate this to the poles of $G(s)$.

---

**Exercise 3.7 — Time-Delay Property Proof** *(Analytical — Challenge)*

Prove the time-delay property of the Laplace transform:
$$\mathcal{L}\{f(t-a)\,u(t-a)\} = e^{-as}F(s) \qquad \text{for } a > 0$$

where $u(t-a)$ is the unit step function shifted by $a$, and $F(s) = \mathcal{L}\{f(t)\}$.

(a) Start from the definition: $\int_0^\infty f(t-a)\,u(t-a)\,e^{-st}\,dt$.  
(b) Use the substitution $\tau = t - a$ to transform the integral.  
(c) Show that the result simplifies to $e^{-as}F(s)$.  
(d) **Application:** If $f(t) = e^{-t}u(t)$, write the Laplace transform of $f(t)$ delayed by 2 seconds. Verify by computing the integral directly.

---

## 3.10 Chapter Summary

### Key Concepts

✅ **Laplace Transform:** Converts differential equations to algebraic equations

✅ **Transfer Function:** $G(s) = Y(s)/U(s)$ characterizes system dynamics

✅ **Poles:** Roots of denominator, determine stability and natural modes

✅ **Zeros:** Roots of numerator, affect shape of response

✅ **Stability:** All poles must be in left half-plane

✅ **Block Diagram Algebra:** Series, parallel, and feedback connections

✅ **Partial Fractions:** Decompose for inverse transform

✅ **Physical Interpretation:** Poles relate to time constants and oscillation frequencies

### What's Next

Chapter 4 analyzes time-domain performance specifications (rise time, settling time, overshoot) using the first and second-order system models.

---

## 3.11 Self-Assessment

### Checklist

- [ ] I can compute Laplace transforms of common functions
- [ ] I can derive transfer functions from differential equations
- [ ] I can identify poles and zeros from a transfer function
- [ ] I can determine stability from pole locations
- [ ] I can perform block diagram reduction
- [ ] I can expand a transfer function using partial fractions

### Practice Problems

**3.1** Find the Laplace transform of $f(t) = t^2 e^{-2t}$.

**3.2** Given $G(s) = \frac{s+2}{s^2+5s+6}$, find the poles, zeros, and determine stability.

**3.3** For the feedback system with $G(s) = \frac{10}{s+1}$ and $H(s) = \frac{1}{s+2}$, find the closed-loop transfer function.

**3.4** Expand $\frac{3s+1}{(s+1)(s+2)^2}$ using partial fractions.

### Problem Identification Exercises (Level 3-4)

**3.5 — What Is the Real Problem?**
A motor speed controller works perfectly in the lab but oscillates in the field. The field motor has the same $K_m$, $R$, $L$, and $J$, but a much longer cable (adding 2Ω resistance and 5mH inductance).

(a) How does the extra cable impedance change the pole locations of the motor transfer function?
(b) Could this shift move a stable pole into the RHP? Explain using the relationship between physical parameters and pole locations from §3.7.
(c) Propose two solutions — one that modifies the *plant* (hardware) and one that modifies the *controller* (software). Which is cheaper? Which is more robust?

**3.6 — Mechanism vs. Procedure**
A student performs a partial fraction expansion of $G(s)$ and gets three terms. When asked "what does each term mean physically?", the student cannot answer.

(a) For $G(s) = \frac{10}{(s+1)(s+10)}$, perform the partial fraction expansion.
(b) Explain what each term represents in the *time domain*. Which term dominates at $t = 0$? Which dominates at $t \to \infty$?
(c) If you could eliminate one pole, which would you remove to make the system respond faster? What physical change would achieve this?

---

## References

1. Ogata, K. (2010). Modern Control Engineering (5th ed.). Prentice Hall.
2. Franklin, G. F., Powell, J. D., & Emami-Naeini, A. (2019). Feedback Control of Dynamic Systems (8th ed.). Pearson.
3. Åström, K. J., & Murray, R. M. (2021). Feedback Systems (2nd ed.). Princeton University Press.


*Next Chapter: Time-Domain Analysis →*
