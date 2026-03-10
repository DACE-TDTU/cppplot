# Chapter 5: Root Locus Analysis and Design
## Modern Control Engineering with C++

---

## 🎯 Learning Objectives and Outcomes

### Chapter Objectives

This chapter develops your ability to analyze and design feedback control systems using root locus techniques, providing a graphical method to understand how closed-loop pole locations change with controller gain.

### Learning Outcomes (Bloom's Taxonomy)

| Level | Outcome | Assessment |
|-------|---------|------------|
| **Remember** | State the rules for sketching root locus | Quiz |
| **Understand** | Explain how closed-loop poles migrate as gain varies | Concept questions |
| **Apply** | Sketch root locus for given systems and select gain for specs | Problem sets |
| **Analyze** | Determine stability margins from root locus | Analysis exercises |
| **Evaluate** | Assess controller designs using root locus criteria | Design review |
| **Create** | Design compensators to reshape root locus for desired performance | Design project |

### Prerequisites
- Chapter 3: Transfer Functions and Poles
- Chapter 4: Time-Domain Performance Specifications
- Complex numbers and polynomials

---

## Why This Chapter Matters: The Gain Selection Problem

> **The Real Engineering Problem:** You have a motor, a sensor, and an amplifier. The math says "choose K for desired performance." But what IS K physically? What limits it? And why can't you just make it infinitely large?

### The Honest Truth About Gain K

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    WHAT IS K IN THE REAL WORLD?                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   MATHEMATICS says:         REALITY says:                                  │
│   "Choose K = 10"           "K is limited by physical hardware"            │
│                                                                             │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │                       PHYSICAL MEANING OF K                         │  │
│   ├─────────────────────────────────────────────────────────────────────┤  │
│   │                                                                     │  │
│   │   K = K_sensor × K_amplifier × K_actuator × K_code                 │  │
│   │                                                                     │  │
│   │   where:                                                            │  │
│   │   • K_sensor = sensor sensitivity (V/rad, V/°C, etc.)              │  │
│   │   • K_amplifier = op-amp or power amplifier gain (V/V)             │  │
│   │   • K_actuator = motor torque constant (N·m/A)                     │  │
│   │   • K_code = digital gain in controller software                    │  │
│   │                                                                     │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│                                                                             │
│   LIMITS ON K:                                                             │
│   • Power amplifier saturates at ±V_supply                                │
│   • DAC has finite resolution (12-bit = 4096 levels)                      │
│   • Op-amp has gain-bandwidth product limit                                │
│   • High K amplifies sensor noise → actuator chattering                   │
│   • Very high K causes instability (root locus shows this!)               │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Root Locus Answers Critical Design Questions

| Question | Root Locus Answer |
|----------|-------------------|
| **What K makes system unstable?** | Where root locus crosses imaginary axis |
| **What K gives desired damping?** | Where root locus intersects ζ-line |
| **Why can't I make it faster?** | Increasing K may push poles toward instability |
| **What if plant parameters change?** | Root locus shows sensitivity to parameter variations |

---

## 5.1 Introduction to Root Locus

### The Feedback Control Problem

```
                    ┌──────────────────────────────────────┐
                    │                                      │
    R(s) ──(+)──────┴──▶ K ──▶ G(s) ──┬──▶ Y(s)          │
            ↑                          │                   │
            │ -                        │                   │
            └──────────────────────────┘                   │
                                                          │
    Question: As K varies from 0 to ∞, where do the       │
    closed-loop poles move in the s-plane?                │
    └──────────────────────────────────────────────────────┘
```

### Closed-Loop Transfer Function

For unity feedback system:
$$T(s) = \frac{KG(s)}{1 + KG(s)}$$

More generally, for non-unity feedback $H(s)$: $T(s) = \frac{KG(s)}{1 + KG(s)H(s)}$

The **characteristic equation** is:
$$1 + KG(s)H(s) = 0$$

The roots of this equation are the **closed-loop poles**.

### Root Locus Definition

The root locus is the path traced by the closed-loop poles in the s-plane as the parameter K varies from 0 to ∞.

**Key Insight:** The root locus shows ALL possible closed-loop pole locations for any value of gain K.

---

## 5.2 Root Locus Construction Rules

### Magnitude and Angle Conditions

For a point s to be on the root locus:
$$KG(s)H(s) = -1$$

This gives us:
- **Magnitude condition:** $|KG(s)H(s)| = 1$
- **Angle condition:** $\angle G(s)H(s) = (2k+1) \times 180°$, $k = 0, \pm 1, \pm 2, ...$

### The Ten Rules

**Rule 1: Starting Points (K = 0)**
Root locus branches start at the open-loop poles.

**Rule 2: Ending Points (K → ∞)**
Root locus branches end at the open-loop zeros or at infinity.

**Rule 3: Number of Branches**
Number of branches = Number of open-loop poles (n)

**Rule 4: Symmetry**
Root locus is symmetric about the real axis.

**Rule 5: Real Axis Segments**
A point on the real axis is on the root locus if the number of open-loop poles and zeros to its right is odd.

**Rule 6: Asymptotes (for branches going to infinity)**
- Number of asymptotes = n - m (poles minus zeros)
- Angles: $\theta_a = \frac{(2k+1) \times 180°}{n-m}$, $k = 0, 1, ..., n-m-1$
- Centroid: $\sigma_a = \frac{\sum \text{poles} - \sum \text{zeros}}{n-m}$

**Rule 7: Breakaway/Break-in Points**
Found where $\frac{dK}{ds} = 0$ on real axis segments.

**Rule 8: Angle of Departure from Complex Poles**

The angle of departure $\theta_d$ from a complex pole $p_i$ determines the initial direction a root locus branch takes as it leaves that pole. It is derived from the angle condition.

> **Derivation:** Consider a test point $s$ infinitesimally close to complex pole $p_i$. The angle condition requires:
> $$\sum_{j=1}^{m} \angle(s - z_j) - \sum_{\substack{k=1 \\ k \neq i}}^{n} \angle(s - p_k) - \angle(s - p_i) = (2q+1) \times 180°$$
>
> Since $s \to p_i$, the angle $\angle(s - p_i) = \theta_d$ (the departure angle), while all other angles are evaluated at $s = p_i$:

$$\boxed{\theta_d = 180° - \sum_{\substack{k=1 \\ k \neq i}}^{n} \angle(p_i - p_k) + \sum_{j=1}^{m} \angle(p_i - z_j)}$$

**Procedure:** Compute the angle from every *other* pole and every zero to the pole $p_i$ in question. The departure angle equals $180°$ minus the sum of angles from other poles, plus the sum of angles from zeros.

**Example:** For $G(s) = \frac{K}{s(s^2 + 2s + 2)}$ with poles at $0, -1 \pm j1$:

Departure from $p = -1 + j1$:
- Angle from pole at $0$: $\angle(-1+j1 - 0) = 135°$
- Angle from pole at $-1-j1$: $\angle(-1+j1-(-1-j1)) = \angle(j2) = 90°$
- No zeros.
- $\theta_d = 180° - (135° + 90°) + 0° = -45°$

**Rule 9: Angle of Arrival at Complex Zeros**

The angle of arrival $\theta_a$ at a complex zero $z_i$ is the direction from which a root locus branch approaches that zero as $K \to \infty$.

> **Derivation:** Using the same approach as Rule 8, place a test point $s$ infinitesimally close to zero $z_i$:
> $$\angle(s - z_i) + \sum_{\substack{j=1 \\ j \neq i}}^{m} \angle(s - z_j) - \sum_{k=1}^{n} \angle(s - p_k) = (2q+1) \times 180°$$
>
> Since $\angle(s - z_i) = \theta_a$:

$$\boxed{\theta_a = 180° + \sum_{k=1}^{n} \angle(z_i - p_k) - \sum_{\substack{j=1 \\ j \neq i}}^{m} \angle(z_i - z_j)}$$

**Procedure:** Compute the angle from every pole and every *other* zero to the zero $z_i$. The arrival angle equals $180°$ plus the sum of angles from poles, minus the sum of angles from other zeros.

**Rule 10: Intersection with Imaginary Axis (jω-axis Crossings)**

The points where the root locus crosses the imaginary axis determine the gain $K$ at which the system becomes marginally stable.

> **Method 1 — Direct substitution:** Substitute $s = j\omega$ into the characteristic equation $1 + KG(s)H(s) = 0$. Separate real and imaginary parts, set both to zero, and solve for $\omega$ and $K$.
>
> **Method 2 — Routh-Hurwitz criterion:** Form the Routh array for the characteristic polynomial (which contains $K$ as a parameter). Find the value of $K$ that makes an entire row zero — this is the marginal stability gain. The crossing frequency $\omega$ is obtained from the auxiliary polynomial formed by the row above the zero row.

**Example (Method 1):** For $1 + \frac{K}{s(s+1)(s+2)} = 0$, the characteristic equation is $s^3 + 3s^2 + 2s + K = 0$.

Substitute $s = j\omega$:
$$(j\omega)^3 + 3(j\omega)^2 + 2(j\omega) + K = 0$$
$$(-j\omega^3 - 3\omega^2 + j2\omega + K) = 0$$

Real part: $K - 3\omega^2 = 0 \implies K = 3\omega^2$

Imaginary part: $2\omega - \omega^3 = 0 \implies \omega(2 - \omega^2) = 0 \implies \omega = \sqrt{2}$

Thus $K_{\text{marginal}} = 3 \times 2 = 6$, and the locus crosses the $j\omega$-axis at $s = \pm j\sqrt{2}$.

---

## 5.3 Physical System Example: Position Control

### DC Motor Position Control

```
    ┌──────────────────────────────────────────────────────────────┐
    │  POSITION CONTROL SYSTEM                                      │
    │                                                               │
    │   θ_ref(s) ──(+)──▶ K ──▶ DC Motor ──▶ θ(s)                 │
    │               ↑            G(s)           │                   │
    │               │ -                         │                   │
    │               └───────────────────────────┘                   │
    │                                                               │
    │   Open-loop TF: KG(s) = K/(s(s+a))                           │
    │   (Type 1 system: one integrator)                            │
    └──────────────────────────────────────────────────────────────┘
```

**Signal Dictionary — DC Motor Position Control**

| Signal | Symbol | Unit | Physical Meaning | Sensor/Actuator |
|--------|--------|------|------------------|-----------------|
| Reference position (input) | $\theta_{ref}$ | rad | Desired angular position of the shaft | Commanded by operator / trajectory planner |
| Motor position (output) | $\theta(t)$ | rad | Actual angular position of the shaft | Optical encoder / resolver |
| Error signal | $e = \theta_{ref} - \theta$ | rad | How far the motor is from the desired position | Computed (not measured directly) |
| Control voltage | $u(t) = Ke$ | V | Amplified error signal applied to motor | Power amplifier |
| Motor angular velocity | $\dot{\theta}(t)$ | rad/s | Rate of rotation — the integrator output | Tachometer / encoder derivative |
| Gain | $K$ | V/rad | Proportional gain — amplifies error into voltage | Controller parameter |
| Mechanical time constant | $\tau_m = 1/a$ | s | How fast the motor accelerates — depends on inertia and friction | (System parameter) |

> **Reading this table:** The plant $G(s) = 1/[s(s+a)]$ has two poles: $s=0$ (the integrator from velocity to position) and $s=-a$ (mechanical dynamics). Every pole has a physical origin.

**Motor Model:**
$$G(s) = \frac{1}{s(s+a)}$$

where $a = 1/\tau_m$ is the inverse mechanical time constant.

**Open-Loop Poles:** $s = 0$ and $s = -a$

**Closed-Loop Characteristic Equation:**
$$1 + \frac{K}{s(s+a)} = 0 \implies s^2 + as + K = 0$$

**Closed-Loop Poles:**
$$s = \frac{-a \pm \sqrt{a^2 - 4K}}{2}$$

### CppPlot Implementation: Position Control Root Locus

```cpp
/**
 * @file ch05_position_control_rl.cpp
 * @brief Root locus analysis of DC motor position control
 * 
 * Physical System: DC motor with position feedback
 * G(s) = 1/[s(s+a)]
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <cmath>
#include <complex>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║   Chapter 5: Root Locus - DC Motor Position Control          ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // Motor parameter
    double a = 2.0;  // 1/τm
    
    std::cout << "\n▶ Open-Loop Transfer Function" << std::endl;
    std::cout << "  G(s) = 1/[s(s+" << a << ")]" << std::endl;
    std::cout << "  Poles: s = 0, s = -" << a << std::endl;
    std::cout << "  Zeros: none (both branches go to infinity)" << std::endl;
    
    // Open-loop system
    TransferFunction G({1}, {1, a, 0});  // 1/(s² + as)
    
    // Root locus analysis
    std::cout << "\n▶ Root Locus Rules" << std::endl;
    std::cout << "  • n = 2 poles, m = 0 zeros → 2 asymptotes" << std::endl;
    std::cout << "  • Asymptote angles: 90°, 270° (vertical)" << std::endl;
    std::cout << "  • Centroid: (0 + (-" << a << "))/2 = -" << a/2 << std::endl;
    std::cout << "  • Real axis: segment from s=0 to s=-" << a << std::endl;
    std::cout << "  • Breakaway: at s = -" << a/2 << std::endl;
    
    // Critical gain (when poles reach jω axis)
    // s² + as + K = 0, for s = jω: -ω² + K = 0, jaω = 0
    // Since a > 0, the root locus never crosses jω axis
    // System is always stable for K > 0
    
    figure(1400, 1000);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 1: Root Locus
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 2, 1);
    
    // Generate root locus data
    std::vector<double> K_values;
    std::vector<std::vector<double>> re_poles, im_poles;
    
    // Initialize vectors for each pole
    re_poles.resize(2);
    im_poles.resize(2);
    
    for (double K = 0.001; K <= 50; K += 0.1) {
        K_values.push_back(K);
        
        // Closed-loop poles: s = (-a ± √(a²-4K))/2
        double discriminant = a*a - 4*K;
        
        if (discriminant >= 0) {
            // Two real poles
            double sqrt_disc = std::sqrt(discriminant);
            re_poles[0].push_back((-a + sqrt_disc) / 2);
            im_poles[0].push_back(0);
            re_poles[1].push_back((-a - sqrt_disc) / 2);
            im_poles[1].push_back(0);
        } else {
            // Complex conjugate poles
            double sqrt_disc = std::sqrt(-discriminant);
            re_poles[0].push_back(-a / 2);
            im_poles[0].push_back(sqrt_disc / 2);
            re_poles[1].push_back(-a / 2);
            im_poles[1].push_back(-sqrt_disc / 2);
        }
    }
    
    // Plot root locus
    plot(re_poles[0], im_poles[0], "b-", {{"linewidth", "2"}, {"label", "Root Locus"}});
    plot(re_poles[1], im_poles[1], "b-", {{"linewidth", "2"}});
    
    // Mark open-loop poles (K=0)
    scatter({0, -a}, {0, 0}, {{"color", "red"}, {"s", "150"}, {"marker", "x"}, {"label", "OL Poles"}});
    
    // Mark centroid
    scatter({-a/2}, {0}, {{"color", "green"}, {"s", "80"}, {"marker", "o"}, {"label", "Centroid"}});
    
    // Draw asymptotes
    std::vector<double> asym_re = {-a/2, -a/2};
    std::vector<double> asym_im = {-8, 8};
    plot(asym_re, asym_im, "--", {{"color", "gray"}, {"alpha", "0.5"}, {"label", "Asymptotes"}});
    
    // Stability boundary (jω axis)
    axvline(0, {{"color", "red"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    axhline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    
    xlabel("Real (σ)");
    ylabel("Imaginary (jω)");
    title("Root Locus: G(s) = 1/[s(s+2)]");
    legend({{"fontsize", "8"}});
    grid(true);
    xlim(-5, 1);
    ylim(-5, 5);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 2: Step Response for Different K
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 2, 2);
    
    std::vector<double> K_demo = {0.5, 1.0, 2.0, 4.0, 8.0};
    std::vector<std::string> colors = {"#E74C3C", "#E67E22", "#2ECC71", "#3498DB", "#9B59B6"};
    
    double t_final = 10.0;
    
    for (size_t i = 0; i < K_demo.size(); ++i) {
        double K = K_demo[i];
        TransferFunction G_cl = feedback(K * G, TransferFunction({1}, {1}));  // KG/(1+KG)
        auto [t, y] = step_data(G_cl, t_final);
        
        std::ostringstream label;
        label << "K = " << K;
        plot(t, y, "-", {{"color", colors[i]}, {"linewidth", "2"}, {"label", label.str()}});
    }
    
    axhline(1.0, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.3"}});
    xlabel("Time [s]");
    ylabel("Position θ/θ_ref");
    title("Closed-Loop Step Response");
    legend({{"fontsize", "9"}});
    grid(true);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 3: Damping Ratio vs Gain
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 2, 3);
    
    std::vector<double> K_sweep, zeta_sweep;
    double K_critical = a*a / 4;  // When discriminant = 0
    
    for (double K = 0.01; K <= 20; K += 0.1) {
        K_sweep.push_back(K);
        
        // For s² + as + K = 0: ωn = √K, ζ = a/(2ωn) = a/(2√K)
        double wn = std::sqrt(K);
        double zeta = a / (2 * wn);
        zeta_sweep.push_back(zeta);
    }
    
    plot(K_sweep, zeta_sweep, "b-", {{"linewidth", "2"}});
    
    // Mark critical damping (ζ = 1)
    axhline(1.0, {{"color", "red"}, {"linestyle", "--"}, {"label", "Critical (ζ=1)"}});
    axhline(0.707, {{"color", "green"}, {"linestyle", "--"}, {"label", "Optimal (ζ=0.707)"}});
    axvline(K_critical, {{"color", "orange"}, {"linestyle", ":"}, {"label", "K_crit=" + std::to_string(K_critical).substr(0,3)}});
    
    xlabel("Gain K");
    ylabel("Damping Ratio ζ");
    title("Damping Ratio vs Controller Gain");
    legend({{"fontsize", "8"}});
    grid(true);
    xlim(0, 20);
    ylim(0, 3);
    
    // ─────────────────────────────────────────────────────────────────
    // Plot 4: Performance Metrics vs Gain
    // ─────────────────────────────────────────────────────────────────
    subplot(2, 2, 4);
    
    std::vector<double> K_perf, ts_values, Mp_values;
    
    for (double K = 0.5; K <= 20; K += 0.2) {
        K_perf.push_back(K);
        
        double wn = std::sqrt(K);
        double zeta = a / (2 * wn);
        
        // Settling time (2%)
        double ts = 4.0 / (zeta * wn);
        ts_values.push_back(std::min(ts, 15.0));  // Clip for visualization
        
        // Overshoot
        double Mp;
        if (zeta < 1) {
            Mp = std::exp(-M_PI * zeta / std::sqrt(1 - zeta*zeta)) * 100;
        } else {
            Mp = 0;
        }
        Mp_values.push_back(Mp);
    }
    
    plot(K_perf, ts_values, "b-", {{"linewidth", "2"}, {"label", "Settling time [s]"}});
    plot(K_perf, Mp_values, "r-", {{"linewidth", "2"}, {"label", "Overshoot [%]"}});
    
    xlabel("Gain K");
    ylabel("Performance Metric");
    title("Performance vs Controller Gain");
    legend();
    grid(true);
    
    savefig("ch05_position_control_rl.svg");
    std::cout << "\n✓ Saved ch05_position_control_rl.svg" << std::endl;
    
    // Design example
    std::cout << "\n▶ Design Example: Select K for ζ = 0.707" << std::endl;
    std::cout << "  ───────────────────────────────────────────" << std::endl;
    
    // For ζ = 0.707: a/(2√K) = 0.707 → K = (a/(2×0.707))² = (a/1.414)²
    double zeta_target = 0.707;
    double K_design = std::pow(a / (2 * zeta_target), 2);
    double wn_design = std::sqrt(K_design);
    
    std::cout << "  Target: ζ = 0.707" << std::endl;
    std::cout << "  From ζ = a/(2√K): K = (a/(2ζ))² = " << K_design << std::endl;
    std::cout << "  Natural frequency ωn = √K = " << wn_design << " rad/s" << std::endl;
    std::cout << "  Expected overshoot: 4.3%" << std::endl;
    std::cout << "  Expected settling time: " << 4/(zeta_target*wn_design) << " s" << std::endl;
    
    return 0;
}
```

> **🔍 Stop and Think — Mechanism Questions**
>
> Before looking at the CppPlot verification below, answer these:
> 1. The root locus shows poles moving from $s = 0$ and $s = -2$ toward each other as $K$ increases. *Physically*, what does it mean when two poles meet on the real axis and become complex? (Hint: the motor starts oscillating. Why?)
> 2. For very large $K$, the poles have large imaginary parts — the system oscillates fast. But does increasing $K$ also make the system *settle faster*? Look at the real part of the poles.
> 3. The breakaway point is where the root locus leaves the real axis. At this value of $K$, the system transitions from overdamped to underdamped. Why is this $K$ value often the *best* design choice?

### CppPlot Verification: Using `rlocus()`

The manual computation above is instructive, but CppPlot provides `rlocus()` to generate root locus plots directly from a transfer function — the recommended approach for practical work:

```cpp
// CppPlot verification — replaces the manual for-loop above
#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    // DC motor position control: G(s) = 1/[s(s+2)]
    TransferFunction G({1}, {1, 2, 0});

    figure(800, 600);
    rlocus(G);  // Automatically plots root locus with gain variation
    title("Root Locus: G(s) = 1/[s(s+2)]  — via rlocus()");
    grid(true);
    savefig("ch05_position_control_rl_api.svg");

    // The rlocus() function:
    //  • Sweeps K from 0 to an auto-determined upper bound
    //  • Computes closed-loop poles at each K
    //  • Marks open-loop poles (×) and zeros (○)
    //  • Draws asymptotes and the jω axis
    // Compare this with the 100+ lines of manual code above!

    return 0;
}
```

> **Teaching note:** Understanding the manual computation (§5.3 code) is essential for exams and deep understanding. Use `rlocus()` for verification and in engineering practice.

---

## 5.4 Higher-Order System Example

### Third-Order System

Consider a motor with additional dynamics:
$$G(s) = \frac{1}{s(s+1)(s+4)}$$

**Open-Loop Poles:** $s = 0, -1, -4$
**Zeros:** None (3 asymptotes)

**Root Locus Characteristics:**
- 3 branches, starting at poles
- Asymptote angles: 60°, 180°, 300°
- Centroid: $(0 - 1 - 4)/3 = -5/3$
- Real axis segments: $(-\infty, -4]$ and $[-1, 0]$

### System Becomes Unstable!

For this third-order system, the root locus CROSSES the jω axis, meaning the system becomes unstable for large enough K.

### CppPlot Implementation: Third-Order Root Locus

```cpp
#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    // Third-order system: G(s) = 1/[s(s+1)(s+4)]
    // Denominator: s³ + 5s² + 4s
    TransferFunction G({1}, {1, 5, 4, 0});

    // Plot root locus
    figure(1000, 800);
    subplot(1, 2, 1);
    rlocus(G);
    title("Root Locus: 1/[s(s+1)(s+4)]");
    grid(true);

    // Find critical gain using Routh criterion:
    // Char. eq.: s³ + 5s² + 4s + K = 0
    // Routh array:
    //   s³ |  1    4
    //   s² |  5    K
    //   s¹ | (20-K)/5
    //   s⁰ |  K
    // Stability requires: (20-K)/5 > 0 AND K > 0  →  0 < K < 20
    // At K = 20: auxiliary poly from s² row → 5s² + 20 = 0 → s = ±j2
    std::cout << "Critical gain K = 20" << std::endl;
    std::cout << "Crossing frequency: ω = 2 rad/s (s = ±j2)" << std::endl;

    // Verify: closed-loop step response at K = 15 (stable, near limit)
    subplot(1, 2, 2);
    TransferFunction G_cl = feedback(G * 15.0, TransferFunction({1}, {1}));
    auto [t, y] = step_data(G_cl, 10.0);
    plot(t, y, "b-", {{"linewidth", "2"}, {"label", "K = 15 (stable)"}});

    // Compare with K = 5 (well-damped)
    TransferFunction G_cl2 = feedback(G * 5.0, TransferFunction({1}, {1}));
    auto [t2, y2] = step_data(G_cl2, 10.0);
    plot(t2, y2, "g-", {{"linewidth", "2"}, {"label", "K = 5 (well-damped)"}});

    axhline(1.0, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.3"}});
    xlabel("Time [s]");
    ylabel("Response");
    title("Closed-Loop Step Response");
    legend();
    grid(true);

    savefig("ch05_third_order_rl.svg");
    return 0;
}
```

---

## 5.5 Lead and Lag Compensation

### The Design Challenge

Sometimes we cannot achieve desired performance just by adjusting gain K. We need to **reshape** the root locus using compensators.

### Lead Compensator

$$G_c(s) = K_c \frac{s + z_c}{s + p_c}, \quad |p_c| > |z_c|$$

**Effect:** Adds a zero (left of pole) that "pulls" root locus left, improving stability and speed.

### Lag Compensator

$$G_c(s) = K_c \frac{s + z_c}{s + p_c}, \quad |z_c| > |p_c|$$

**Effect:** Adds a pole-zero pair near origin to improve steady-state accuracy without significantly changing transient response.

### CppPlot Implementation: Compensator Design

```cpp
/**
 * @file ch05_lead_compensation.cpp
 * @brief Lead compensator design using root locus
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <cmath>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║   Chapter 5: Lead Compensator Design                         ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // Original plant
    // G(s) = 1/[s(s+2)]
    TransferFunction G({1}, {1, 2, 0});
    
    std::cout << "\n▶ Original System" << std::endl;
    std::cout << "  G(s) = 1/[s(s+2)]" << std::endl;
    std::cout << "  Problem: Limited bandwidth, slow response" << std::endl;
    
    // Lead compensator design
    // Gc(s) = Kc(s+z)/(s+p), where p > z
    double z_c = 2.0;   // Compensator zero
    double p_c = 10.0;  // Compensator pole
    double Kc = 20.0;   // Compensator gain
    
    std::cout << "\n▶ Lead Compensator" << std::endl;
    std::cout << "  Gc(s) = " << Kc << "(s+" << z_c << ")/(s+" << p_c << ")" << std::endl;
    std::cout << "  Zero at s = -" << z_c << " (cancels plant pole)" << std::endl;
    std::cout << "  Pole at s = -" << p_c << " (provides phase lead)" << std::endl;
    
    TransferFunction Gc({Kc, Kc*z_c}, {1, p_c});
    
    // Compensated open-loop
    TransferFunction GGc = G * Gc;
    
    // Closed-loop systems
    TransferFunction T_orig = feedback(10 * G, TransferFunction({1}, {1}));  // K = 10
    TransferFunction T_comp = feedback(GGc, TransferFunction({1}, {1}));
    
    figure(1200, 800);
    
    // Step response comparison
    subplot(2, 2, 1);
    
    double t_final = 5.0;
    auto [t1, y1] = step_data(T_orig, t_final);
    auto [t2, y2] = step_data(T_comp, t_final);
    
    plot(t1, y1, "b-", {{"linewidth", "2"}, {"label", "Original (K=10)"}});
    plot(t2, y2, "r-", {{"linewidth", "2"}, {"label", "With Lead Comp."}});
    
    axhline(1.0, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.3"}});
    xlabel("Time [s]");
    ylabel("Response");
    title("Step Response Comparison");
    legend();
    grid(true);
    
    // Root locus comparison (simplified visualization)
    subplot(2, 2, 2);
    
    // Original poles movement
    std::vector<double> orig_re_1, orig_im_1, orig_re_2, orig_im_2;
    for (double K = 0.01; K <= 50; K += 0.5) {
        double disc = 4 - 4*K;
        if (disc >= 0) {
            double sq = std::sqrt(disc);
            orig_re_1.push_back((-2 + sq) / 2);
            orig_im_1.push_back(0);
            orig_re_2.push_back((-2 - sq) / 2);
            orig_im_2.push_back(0);
        } else {
            double sq = std::sqrt(-disc);
            orig_re_1.push_back(-1);
            orig_im_1.push_back(sq / 2);
            orig_re_2.push_back(-1);
            orig_im_2.push_back(-sq / 2);
        }
    }
    
    plot(orig_re_1, orig_im_1, "b-", {{"linewidth", "2"}, {"label", "Original"}});
    plot(orig_re_2, orig_im_2, "b-", {{"linewidth", "2"}});
    
    // Mark original open-loop poles
    scatter({0, -2}, {0, 0}, {{"color", "blue"}, {"s", "100"}, {"marker", "x"}});
    
    // Compensated has different root locus (pole-zero cancellation)
    scatter({-10}, {0}, {{"color", "red"}, {"s", "100"}, {"marker", "x"}, {"label", "Comp. pole"}});
    scatter({-2}, {0}, {{"color", "red"}, {"s", "80"}, {"marker", "o"}, {"label", "Comp. zero"}});
    
    axvline(0, {{"color", "red"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    axhline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    
    xlabel("Real");
    ylabel("Imaginary");
    title("Root Locus Comparison");
    legend({{"fontsize", "8"}});
    grid(true);
    xlim(-12, 2);
    ylim(-6, 6);
    
    // Bode comparison
    subplot(2, 2, 3);
    
    // Simplified magnitude response
    std::vector<double> freq, mag_orig, mag_comp;
    for (double f = 0.01; f <= 100; f *= 1.1) {
        double w = 2 * M_PI * f;
        std::complex<double> s(0, w);
        
        // Original: 10/[s(s+2)]
        std::complex<double> H_orig = 10.0 / (s * (s + 2.0));
        mag_orig.push_back(20 * std::log10(std::abs(H_orig)));
        
        // Compensated: 20(s+2)/[s(s+2)(s+10)] = 20/[s(s+10)]
        std::complex<double> H_comp = 20.0 * (s + 2.0) / (s * (s + 2.0) * (s + 10.0));
        mag_comp.push_back(20 * std::log10(std::abs(H_comp)));
        
        freq.push_back(f);
    }
    
    plot(freq, mag_orig, "b-", {{"linewidth", "2"}, {"label", "Original"}});
    plot(freq, mag_comp, "r-", {{"linewidth", "2"}, {"label", "Compensated"}});
    
    axhline(0, {{"color", "gray"}, {"linestyle", ":"}, {"alpha", "0.5"}});
    xlabel("Frequency [Hz]");
    ylabel("Magnitude [dB]");
    title("Open-Loop Frequency Response");
    legend();
    grid(true);
    xscale("log");
    
    // Phase margin illustration
    subplot(2, 2, 4);
    
    std::vector<double> phase_orig, phase_comp;
    for (double f = 0.01; f <= 100; f *= 1.1) {
        double w = 2 * M_PI * f;
        std::complex<double> s(0, w);
        
        std::complex<double> H_orig = 10.0 / (s * (s + 2.0));
        phase_orig.push_back(std::arg(H_orig) * 180 / M_PI);
        
        std::complex<double> H_comp = 20.0 * (s + 2.0) / (s * (s + 2.0) * (s + 10.0));
        phase_comp.push_back(std::arg(H_comp) * 180 / M_PI);
    }
    
    plot(freq, phase_orig, "b-", {{"linewidth", "2"}, {"label", "Original"}});
    plot(freq, phase_comp, "r-", {{"linewidth", "2"}, {"label", "Compensated"}});
    
    axhline(-180, {{"color", "red"}, {"linestyle", "--"}, {"alpha", "0.5"}, {"label", "-180°"}});
    xlabel("Frequency [Hz]");
    ylabel("Phase [deg]");
    title("Open-Loop Phase Response");
    legend();
    grid(true);
    xscale("log");
    
    savefig("ch05_lead_compensation.svg");
    std::cout << "\n✓ Saved ch05_lead_compensation.svg" << std::endl;
    
    return 0;
}
```

---

## 5.6 Practical Constraints: K in the Real World

> **Q4: How to Implement?** Root locus gives you ideal K values. But real systems have limits.

### 5.6.1 Physical Hardware Limits on Gain

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    GAIN IMPLEMENTATION CHAIN                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   Error e(t) → [Software K] → [DAC] → [Op-Amp] → [Power Amp] → [Motor]    │
│                     ↓            ↓         ↓           ↓           ↓        │
│                  K_code     Resolution   Bandwidth  Saturation  Torque     │
│                                                                             │
│   EXAMPLE: Servo motor control with 12-bit DAC, ±24V supply               │
│                                                                             │
│   • DAC resolution: 24V / 4096 = 5.9mV per bit                            │
│   • If K_code = 1000, minimum voltage change = 5.9V (too coarse!)         │
│   • Power amp saturates at 24V → max current limited → max torque limited │
│   • Op-amp bandwidth: high-freq components lost if gain × BW exceeded      │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 5.6.2 From Root Locus K to Hardware Specification

**Step-by-step process:**

```cpp
/**
 * @brief Translate root locus K to hardware requirements
 * 
 * Example: Position servo with motor Kt = 0.5 N·m/A, Rsensor = 0.1 V/rad
 */

// From root locus analysis: K_total = 25 for desired poles
double K_total = 25.0;  // units: V/rad (voltage per radian error)

// Physical parameters
double K_sensor = 0.1;       // V/rad (encoder + signal conditioning)
double K_motor = 0.5;        // N·m/A (motor torque constant)
double R_motor = 2.0;        // Ω (motor resistance)

// Calculate required amplifier gain
// K_total = K_sensor × K_amplifier × (K_motor/R_motor)
// K_amplifier = K_total × R_motor / (K_sensor × K_motor)
double K_amplifier = K_total * R_motor / (K_sensor * K_motor);

std::cout << "Hardware Requirements:" << std::endl;
std::cout << "  Required amplifier gain: " << K_amplifier << " V/V" << std::endl;

// Check saturation
double max_error = 30.0 * M_PI / 180.0;  // 30 degrees max error
double max_voltage = K_amplifier * K_sensor * max_error;
double supply_voltage = 24.0;

if (max_voltage > supply_voltage) {
    std::cout << "  WARNING: Amplifier will saturate at " << max_error * 180/M_PI 
              << " degree error!" << std::endl;
    std::cout << "  Actual max output limited to " << supply_voltage << "V" << std::endl;
}
```

### 5.6.3 Noise Amplification and High-Gain Issues

**Why you can't just increase K indefinitely:**

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    HIGH GAIN PROBLEMS                                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   PROBLEM 1: Sensor Noise Amplification                                    │
│   ─────────────────────────────────────                                    │
│   • Sensor noise: typically 1-10 mV RMS                                    │
│   • With K = 1000, noise becomes 1-10V at actuator input                   │
│   • Motor current fluctuates → audible noise, heating, vibration          │
│                                                                             │
│   PROBLEM 2: Unmodeled Dynamics                                            │
│   ─────────────────────────────────                                        │
│   • Real motors have resonance at high frequency                           │
│   • High K excites these modes → instability at frequencies               │
│     not shown in simplified model                                          │
│                                                                             │
│   PROBLEM 3: Sample Rate Limits (Digital Control)                          │
│   ──────────────────────────────────────────────                           │
│   • Digital controller runs at finite sample rate fs                       │
│   • High K with slow sample rate → effective delay → instability          │
│   • Rule of thumb: bandwidth < fs/10                                       │
│                                                                             │
│   PROBLEM 4: Actuator Wear                                                 │
│   ──────────────────────────                                               │
│   • High gain → rapid switching of actuator                               │
│   • Relay contacts wear, motor brushes degrade faster                      │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 5.6.4 Integrated Example: Industrial Robot Joint Control

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    ROBOT JOINT ROOT LOCUS DESIGN                            │
│                    Mechanical + Electrical + Software                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   PHYSICAL SYSTEM:                                                         │
│   ┌────────────────┐    ┌─────────────┐    ┌────────────────┐             │
│   │  17-bit Encoder│───▶│  Controller │───▶│ Brushless Motor│─┐           │
│   │  (131072 counts)│   │  (DSP@10kHz)│    │ (Kv=0.2 Nm/A) │ │           │
│   └────────────────┘    └─────────────┘    └────────────────┘ │           │
│          │                     │                    │          │           │
│          │                     ▼                    ▼          │           │
│          │              ┌─────────────┐     ┌─────────────┐   │           │
│          │              │ 16-bit DAC  │────▶│ 50A Motor   │   │           │
│          │              │  (±10V out) │     │ Driver      │   │ Gear      │
│          │              └─────────────┘     └─────────────┘   │ 100:1     │
│          │                                                     │           │
│          └──────────────────── Load Arm ◄─────────────────────┘           │
│                                                                             │
│   DESIGN CONSTRAINTS:                                                      │
│   • Encoder resolution: 131072 / (2π) = 20861 counts/rad                  │
│   • DAC resolution: 20V / 65536 = 0.3mV per bit                          │
│   • Max motor current: 50A → Max torque = 10 N·m                          │
│   • After gearing: Max joint torque = 1000 N·m                            │
│   • Sample rate: 10 kHz → bandwidth limit ~1 kHz                          │
│                                                                             │
│   ROOT LOCUS ANALYSIS:                                                     │
│   • Open-loop: G(s) = Kt/(Js² + Bs) with J=5 kg·m², B=2 N·m·s/rad        │
│   • Design K for ωn = 50 rad/s, ζ = 0.7                                   │
│   • Check: Required torque at max tracking error                          │
│   • Verify: Current limit not exceeded                                    │
│                                                                             │
│   RESULT: K_theory = 12500, but implement with current limiter            │
│   to protect motor during large errors (startup, collision)               │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 5.7 Exercises

**Exercise 5.1 — Root Locus Sketching** *(Analytical)*

Sketch the root locus for the system with open-loop transfer function:
$$G(s) = \frac{K}{s(s+2)(s+4)}$$

(a) Identify the open-loop poles (starting points, $K=0$) and the number of branches.  
(b) Determine the segments of the real axis that belong to the root locus.  
(c) Calculate the **asymptote angles** and **centroid**:
   - Angles: $\theta_k = \frac{(2k+1) \times 180°}{n - m}$, $k = 0, 1, \ldots$
   - Centroid: $\sigma_a = \frac{\sum \text{poles} - \sum \text{zeros}}{n - m}$

(d) Find the **break-away point(s)** on the real axis.  
   *Hint:* Solve $\frac{dK}{ds} = 0$ where $K = -\frac{1}{G(s)} = -s(s+2)(s+4)$.  
(e) Determine the **imaginary axis crossing** using the Routh criterion (or by substituting $s = j\omega$). What is the value of $K$ at marginal stability?  
(f) Sketch the complete root locus.

---

**Exercise 5.2 — Stability Analysis from Root Locus** *(Analytical)*

For the system in Exercise 5.1, $G(s) = \frac{K}{s(s+2)(s+4)}$:

(a) Using the root locus from E5.1, identify the value of $K$ where the locus crosses the imaginary axis (marginal stability).  
(b) Verify this value using the **Routh-Hurwitz criterion** applied to the closed-loop characteristic equation:
$$s^3 + 6s^2 + 8s + K = 0$$

(c) For what range of $K > 0$ is the closed-loop system stable?  
(d) What type of instability occurs when $K$ exceeds the critical value (sustained oscillation, divergence, etc.)? What is the oscillation frequency?

---

**Exercise 5.3 — Effect of Adding a Zero** *(Analytical)*

Compare the root locus of the original system from E5.1 with the modified system that includes a zero at $s = -1$:
$$G_2(s) = \frac{K(s+1)}{s(s+2)(s+4)}$$

(a) Sketch the root locus of $G_2(s)$.  
(b) How do the asymptotes change (number, angles, centroid)?  
(c) Does the system remain stable for all $K > 0$? Why or why not?  
(d) Compare the break-away/break-in points with the original system.  
(e) Explain physically why adding a zero near the open-loop poles improves stability.

---

**Exercise 5.4 — CppPlot Root Locus Verification** *(Computational/Coding)* ⭐

Use CppPlot to verify your hand sketch from Exercise 5.1:

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;

int main() {
    // G(s) = K / [s(s+2)(s+4)]
    // Numerator: 1, Denominator: s^3 + 6s^2 + 8s = (s)(s+2)(s+4)
    TransferFunction G({1}, {1, 6, 8, 0});
    
    // Plot root locus
    rlocus(G);
    title("Root Locus: K / [s(s+2)(s+4)]");
    grid(true);
    show();
    
    // Also plot with the zero added
    TransferFunction G2({1, 1}, {1, 6, 8, 0});  // K(s+1)/[s(s+2)(s+4)]
    figure();
    rlocus(G2);
    title("Root Locus: K(s+1) / [s(s+2)(s+4)]");
    grid(true);
    show();
    
    return 0;
}
```

(a) Compare the CppPlot root locus with your hand sketch. Do the key features match (asymptotes, break-away, crossings)?  
(b) Use the CppPlot plot to read off the gain $K$ at the imaginary axis crossing. Does it match your Routh calculation from E5.2?

---

**Exercise 5.5 — Lead Compensator Design** *(Design)*

For the plant:
$$G(s) = \frac{1}{s(s+4)}$$

Design a lead compensator $G_c(s) = K_c \frac{s + z}{s + p}$ (with $p > z$) so that the closed-loop system satisfies:
- Damping ratio: $\zeta \geq 0.5$
- Natural frequency: $\omega_n \geq 3\,\text{rad/s}$

(a) Determine the desired closed-loop pole locations from the specifications.  
   *Hint:* For $\zeta = 0.5$, $\omega_n = 3$: poles at $s = -1.5 \pm j2.598$.  
(b) Calculate the angle deficiency: the angle contribution needed from the compensator so the desired point lies on the root locus.  
(c) Choose the compensator zero $z$ and pole $p$ to provide the required angle.  
(d) Determine $K_c$ so the desired point satisfies the magnitude condition.  
(e) Verify your design using CppPlot's `rlocus()` — does the locus pass through the desired pole location?  
(f) Plot the closed-loop step response and confirm $\%OS$ and $t_s$ meet the specs.

---

**Exercise 5.6 — Departure Angles and Stability Range** *(Analytical)*

For the system:
$$G(s) = \frac{K}{(s+1)(s+2)(s+3)}$$

(a) Identify the open-loop poles and the real-axis segments of the root locus.  
(b) Calculate the asymptote angles and centroid.  
(c) Find the break-away point(s).  
(d) Determine the imaginary axis crossings and the corresponding value of $K$.  
(e) State the range of $K > 0$ for which the system is stable.  
(f) At $K = K_{marginal}$, what is the frequency of sustained oscillation?

---

**Exercise 5.7 — Challenging Root Locus with Complex Zeros** *(Analytical — Challenge)* ⭐⭐

Sketch the root locus for:
$$G(s) = \frac{K(s^2 + 4)}{(s^2 + 1)(s + 2)}$$

(a) Identify all poles and zeros (note: some are complex).  
   - Poles: $s = -2$, $s = \pm j1$
   - Zeros: $s = \pm j2$

(b) Determine the real-axis segments (if any) that belong to the root locus.  
(c) Since there are complex poles on the imaginary axis, calculate the **departure angles** from the poles at $s = \pm j1$.  
(d) Calculate the **arrival angles** at the zeros $s = \pm j2$.  
(e) How many imaginary-axis crossings are there? At what values of $K$?  
(f) Sketch the complete root locus. Note any unusual features.  
(g) Is the system stable for small $K > 0$? For large $K$? Describe the stability behavior as $K$ increases from 0.

---

## 5.8 Chapter Summary

### Key Concepts

✅ **Root Locus:** Path of closed-loop poles as K varies from 0 to ∞

✅ **Starting Points:** Open-loop poles (K = 0)

✅ **Ending Points:** Open-loop zeros or infinity (K → ∞)

✅ **Asymptotes:** For branches going to infinity

✅ **Real Axis Rule:** Odd number of poles+zeros to the right

✅ **Stability:** System unstable when poles cross jω axis

✅ **Compensators:** Lead (speed up), Lag (steady-state accuracy)

✅ **Physical K:** Must translate mathematical K to hardware specifications

### Design Procedure

1. Sketch root locus of uncompensated system
2. Identify specifications (ζ, ωn, settling time)
3. Determine if achievable with gain adjustment alone
4. If not, design compensator to reshape root locus
5. Select K to place poles at desired locations
6. **NEW:** Translate K to hardware requirements, check saturation/noise limits
7. Verify with simulation

---

## 5.9 Self-Assessment

### Checklist

- [ ] I can sketch root locus using the ten rules
- [ ] I can determine stability margin from root locus
- [ ] I can select gain K to meet performance specifications
- [ ] I understand when compensators are needed
- [ ] I can design a lead compensator for improved speed
- [ ] I can translate mathematical K to hardware requirements
- [ ] I understand why high gain causes practical problems

### Practice Problems

**5.1** Sketch the root locus for $G(s) = K/[s(s+3)(s+5)]$ and find the range of K for stability.

**5.2** For $G(s) = K/(s+1)(s+2)$, find K such that ζ = 0.5.

**5.3** Design a lead compensator for $G(s) = 1/[s(s+2)]$ to achieve settling time < 1s with ζ ≥ 0.7.

**5.4** (NEW) For problem 5.3, if your motor has torque constant 0.3 N·m/A and power supply is 24V with 5Ω winding resistance, calculate the required amplifier gain and check if the system will saturate at 10° error.

### Problem Identification Exercises (Level 3-4)

**5.5 — What Is the Real Problem?**
A student designs a controller for $G(s) = K/[s(s+2)]$ using root locus. The design achieves $\zeta = 0.7$ on the root locus, but when implemented on a real motor, the system oscillates.

(a) List three real-world effects that are NOT in the idealized model $G(s) = K/[s(s+2)]$ (e.g., actuator saturation, sensor noise, unmodeled dynamics).
(b) For each effect, explain how it could cause the root locus prediction to fail.
(c) The student’s instinct is to re-draw the root locus with a more accurate model. A senior engineer says: "First, check the gain margin." Why is this advice better?

**5.6 — Mechanism vs. Procedure**
A root locus has three branches that all enter the right half-plane for $K > K_{max}$.

(a) Explain *physically* (not just mathematically) why increasing gain eventually destabilizes a system with three or more poles. (Hint: think about signal delay around the loop.)
(b) For $G(s) = 1/[s(s+1)(s+10)]$, the asymptote centroid is at $\sigma_a = -(0+1+10)/3 = -3.67$. What does this number mean in terms of the "average speed" of the system modes?
(c) If you add a zero at $s = -5$, the centroid shifts. Compute the new centroid and explain how the zero stabilizes the high-gain behavior.

---

## References

1. Ogata, K. (2010). Modern Control Engineering (5th ed.). Prentice Hall.
2. Franklin, G. F., Powell, J. D., & Emami-Naeini, A. (2019). Feedback Control of Dynamic Systems (8th ed.). Pearson.


---

*Next Chapter: Frequency Response Methods →*
