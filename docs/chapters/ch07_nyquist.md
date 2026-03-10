# Chapter 7: Nyquist Stability Criterion

---

## 🎯 Learning Objectives and Outcomes

### Chapter Objectives

This chapter develops the Nyquist stability criterion, a powerful graphical method for determining closed-loop stability from open-loop frequency response, especially valuable for systems with time delays or unstable open-loop dynamics.

### Prerequisites
- Chapter 6: Frequency Response Analysis (Bode plots, stability margins)
- Complex analysis basics (contour mapping)

---

## Why This Chapter Matters: Systems That Fool Bode Plots

> **The Real Engineering Problem:** You check gain margin and phase margin from Bode plots - both look fine! But when you close the loop, the system oscillates. What went wrong?

### When Bode Plots Are Not Enough

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    CASES WHERE BODE PLOTS MISLEAD                           │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   CASE 1: Time Delay (Transport Lag)                                       │
│   ─────────────────────────────────                                        │
│   • Conveyor belt, pipeline, network latency                               │
│   • G(s) = G₀(s) × e^(-sT)                                                 │
│   • Phase keeps decreasing with frequency: -ωT rad/s                       │
│   • Eventually crosses -180° multiple times!                                │
│   • Bode margin can look OK but system is unstable                         │
│                                                                             │
│   CASE 2: Unstable Open-Loop Plant                                         │
│   ───────────────────────────────                                          │
│   • Inverted pendulum, Segway, unstable aircraft                           │
│   • Open-loop already has RHP poles                                        │
│   • Must "catch" the instability with feedback                             │
│   • Bode plots don't show you need specific encirclements                  │
│                                                                             │
│   CASE 3: Non-Minimum Phase                                                │
│   ───────────────────────────                                              │
│   • Boiler drum, flexible structures, reversed-direction zeros             │
│   • RHP zeros cause extra phase lag                                        │
│   • Bode plots miss the physical constraints these impose                  │
│                                                                             │
│   CASE 4: Multiple Crossover Frequencies                                   │
│   ──────────────────────────────────                                       │
│   • System with notch filters or anti-resonance                            │
│   • Magnitude crosses 0 dB multiple times                                  │
│   • Which crossover determines stability?                                  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Physical Intuition for Nyquist

The Nyquist criterion answers a fundamental question:

> **"If I inject a sinusoid at frequency ω into the feedback loop, what comes back to the summing junction?"**

```
                         What comes back?
                              │
              ◄───────────────┘
              │
    r ───(+)──┴──▶ G(s) ──▶ H(s) ──┐
          ↑                         │
          │ -                       │
          └─────────────────────────┘

At each frequency ω, the signal returns with:
• Magnitude: |G(jω)H(jω)|
• Phase: ∠G(jω)H(jω)

CRITICAL POINT: If magnitude = 1 AND phase = -180°...
                The returning signal EXACTLY CANCELS the input!
                The system can self-oscillate!

This is the point (-1, 0) on the Nyquist plot.
```

**Signal Dictionary — Feedback Loop Signals**

| Signal | Symbol | Unit | Physical Meaning | Sensor/Actuator |
|--------|--------|------|------------------|-----------------|
| Reference input | $R(s)$ | varies | The desired setpoint | Operator / trajectory generator |
| Error signal | $E(s) = R - HY$ | varies | Difference between reference and feedback | Computed (summing junction) |
| Controller output | $U(s) = C(s)E(s)$ | varies | Corrective action from the controller | Controller (PID, lead-lag, etc.) |
| Plant output | $Y(s) = G(s)U(s)$ | varies | Physical response of the system | Process variable |
| Feedback signal | $H(s)Y(s)$ | varies | Measured/filtered output returned to summing junction | Sensor + signal conditioning |
| Loop gain signal | $L(j\omega) = G(j\omega)H(j\omega)$ | — | Signal that travels around the entire loop at frequency $\omega$ | (Analysis quantity, not a physical wire) |
| Critical point | $-1 + 0j$ | — | If loop gain equals this, the returning signal exactly cancels the input → sustained oscillation | (Mathematical threshold) |

> **Key insight:** Nyquist plots the *loop gain* $L(j\omega)$ as $\omega$ sweeps from $0$ to $\infty$. The question is always: does the returning signal grow, shrink, or exactly cancel? That is what the $(-1, 0)$ point tests.

### What Nyquist Shows That Bode Doesn't

| Situation | Bode Analysis | Nyquist Analysis |
|-----------|---------------|------------------|
| Time delay of 0.1s | GM=6dB, PM=45° (looks OK) | Nyquist shows encirclements if ω_gc > 15 rad/s |
| Unstable plant (P=2) | Cannot determine stability | Needs 2 CCW encirclements of (-1,0) |
| Multiple crossovers | Which one matters? | Complete picture from full plot |
| Conditional stability | Increasing K can destabilize AND restabilize | Shows full stability regions |

---

## Learning Outcomes

After completing this chapter, students will be able to:

| Bloom's Level | Outcome |
|---------------|---------|
| **Remember** | State the Nyquist stability criterion |
| **Understand** | Explain the argument principle and contour mapping |
| **Apply** | Draw Nyquist plots for given transfer functions |
| **Analyze** | Count encirclements of the critical point (-1, 0) |
| **Evaluate** | Assess closed-loop stability from open-loop frequency response |
| **Create** | Design controllers using Nyquist criterion |

---

## 7.1 Introduction

### 7.1.1 Motivation

**Problem:** Given open-loop transfer function $G(s)H(s)$, determine closed-loop stability.

**Bode plot limitation:** Cannot easily handle unstable open-loop systems.

**Nyquist advantage:** Works for all system types, including:
- Unstable open-loop systems
- Systems with time delay
- Non-minimum phase systems

### 7.1.2 Characteristic Equation

Closed-loop system:
$$T(s) = \frac{G(s)}{1 + G(s)H(s)}$$

**Stability condition:** All roots of $1 + G(s)H(s) = 0$ must have negative real parts.

---

## 7.2 Mathematical Foundation

### 7.2.1 The Argument Principle

> **Theorem (Cauchy's Argument Principle):**
> 
> If $F(s)$ is a meromorphic function (has poles and zeros), and contour $\Gamma$ encloses Z zeros and P poles of $F(s)$, then:
> $$N = Z - P$$
> 
> where N is the number of clockwise encirclements of the origin by $F(s)$ as s traverses $\Gamma$.

### 7.2.2 Application to Control Systems

Let $F(s) = 1 + G(s)H(s)$

- **Zeros of F(s)** = Closed-loop poles (what we want to count)
- **Poles of F(s)** = Open-loop poles (usually known)

**Key relationship:**
$$Z = N + P$$

Where:
- Z = Number of unstable closed-loop poles
- N = Number of clockwise encirclements of (-1, 0)
- P = Number of unstable open-loop poles

### 7.2.3 Why the Critical Point Is (-1, 0)

The entire Nyquist criterion hinges on the point $-1 + j0$ in the complex plane. Understanding *why* this specific point matters requires tracing the mathematics from the characteristic equation through the Cauchy argument principle.

#### From Characteristic Equation to Contour Mapping

The closed-loop transfer function is:
$$T(s) = \frac{G(s)}{1 + G(s)H(s)}$$

Closed-loop poles are the roots of the **characteristic equation**:
$$1 + G(s)H(s) = 0$$

Define the **return ratio** function:
$$F(s) = 1 + G(s)H(s)$$

The closed-loop system is **unstable** if $F(s)$ has any zeros in the right-half plane (RHP), because those zeros are the closed-loop poles.

#### Applying the Cauchy Argument Principle

The argument principle states: if we map a closed contour $\Gamma_s$ in the $s$-plane through a function $F(s)$, the resulting contour $\Gamma_F$ in the $F$-plane encircles the **origin** a net number of times:
$$N_{\text{origin}} = Z - P$$

where $Z$ = number of zeros of $F(s)$ inside $\Gamma_s$, and $P$ = number of poles of $F(s)$ inside $\Gamma_s$.

Now, $F(s) = 1 + G(s)H(s)$, so mapping through $F(s)$ is the same as mapping through $G(s)H(s)$ and then **shifting by +1**:
$$F(s) = 1 + G(s)H(s) \quad \Longleftrightarrow \quad G(s)H(s) = F(s) - 1$$

Therefore, if we plot $G(s)H(s)$ instead of $F(s)$, the entire image shifts left by 1. The origin of the $F$-plane becomes the point $(-1, 0)$ in the $G(s)H(s)$-plane:

$$\boxed{\text{Encirclements of } (-1,0) \text{ by } G(s)H(s) = \text{Encirclements of origin by } F(s) = Z - P}$$

#### The Complete Derivation

Choose $\Gamma_s$ = the Nyquist contour (encloses the entire RHP). Then:

| Quantity | In terms of $F(s) = 1 + G(s)H(s)$ |
|----------|--------------------------------------|
| $Z$ | Zeros of $F(s)$ in RHP = **closed-loop RHP poles** (unstable CL poles) |
| $P$ | Poles of $F(s)$ in RHP = **open-loop RHP poles** (known from $G(s)H(s)$) |
| $N$ | Clockwise encirclements of $(-1,0)$ by $G(j\omega)H(j\omega)$ |

The argument principle gives:
$$\boxed{N = Z - P}$$

Rearranging for the number of unstable closed-loop poles:
$$\boxed{Z = N + P}$$

**For closed-loop stability**, we need $Z = 0$, which requires:
$$N = -P$$

This means we need exactly $P$ **counter-clockwise** encirclements of $(-1, 0)$.

#### Physical Interpretation

When $G(j\omega)H(j\omega) = -1 + j0$, the open-loop transfer function has:
- **Magnitude** $|G(j\omega)H(j\omega)| = 1$ — the signal returns with unchanged amplitude
- **Phase** $\angle G(j\omega)H(j\omega) = -180°$ — the signal returns perfectly inverted

Combined with the $-1$ at the summing junction, the total loop produces $(-1) \times (-1) = +1$: the signal reinforces itself perfectly. This is the **threshold of oscillation**.

```
    Im{GH}
      ↑
      │         ω increases
      │        ╱
      │       ╱   Nyquist plot of G(jω)H(jω)
      │      ╱
  ────●─────●──────────────▶ Re{GH}
   (-1,0)  origin
      │
      │  If the plot encircles (-1,0):
      │    • CW encirclement → more unstable CL poles
      │    • CCW encirclement → fewer unstable CL poles
      │
```

#### Summary Table

| $P$ (OL RHP poles) | Required $N$ for stability | Meaning |
|---------------------|----------------------------|----------|
| 0 | $N = 0$ | No encirclements of $(-1,0)$ |
| 1 | $N = -1$ | 1 CCW encirclement of $(-1,0)$ |
| 2 | $N = -2$ | 2 CCW encirclements of $(-1,0)$ |
| $k$ | $N = -k$ | $k$ CCW encirclements of $(-1,0)$ |

---

## 7.3 Nyquist Contour

### 7.3.1 Definition

The **Nyquist contour** encloses the entire right-half plane:

1. **Positive imaginary axis**: $s = j\omega$, $0^+ \leq \omega < \infty$
2. **Large semicircle**: $s = Re^{j\theta}$, $R \to \infty$, $-90° \leq \theta \leq 90°$
3. **Negative imaginary axis**: $s = j\omega$, $-\infty < \omega \leq 0^-$

### 7.3.2 Practical Simplification

For **proper** transfer functions:
- Large semicircle maps to origin
- Plot only $G(j\omega)H(j\omega)$ for $0 < \omega < \infty$
- Use symmetry: $G(-j\omega) = G^*(j\omega)$ (complex conjugate)

### 7.3.3 Indentation Around jω-Axis Poles

When $G(s)H(s)$ has poles **on the imaginary axis** (e.g., integrators at the origin, or undamped oscillatory poles at $s = \pm j\omega_0$), the Nyquist contour passes directly through a singularity. The mapping $G(s)H(s)$ is undefined at these points, so we must **indent** the contour around them.

#### The Indentation Rule

We modify the Nyquist contour by detouring around each jω-axis pole with a **small semicircle of radius $\varepsilon \to 0$** into the right-half plane:

```
    Im(s)
      ↑
      │
      │     Nyquist contour (modified)
      │         │
      │         │  ω increasing
      │         │
      ●─────────┤  pole at s = 0
     ╱ ╲ε       │
    ╱   ╲       │
   ╱  ε→0╲     │
  (indent)     │
      │         │
      │         ↓  ω decreasing (negative freq)
```

The indentation takes the form:
$$s = \varepsilon e^{j\theta}, \quad \theta: \frac{\pi}{2} \to -\frac{\pi}{2}$$

(going clockwise from just above the pole to just below it, staying in the RHP).

#### Mapping of the Indentation

For a transfer function with a **pole of multiplicity $k$** at the origin:
$$G(s)H(s) = \frac{N(s)}{s^k \cdot D'(s)}$$

where $D'(0) \neq 0$. On the indentation $s = \varepsilon e^{j\theta}$:

$$G(\varepsilon e^{j\theta})H(\varepsilon e^{j\theta}) = \frac{N(\varepsilon e^{j\theta})}{(\varepsilon e^{j\theta})^k \cdot D'(\varepsilon e^{j\theta})}$$

As $\varepsilon \to 0$:
$$G \approx \frac{N(0)}{D'(0)} \cdot \frac{1}{\varepsilon^k e^{jk\theta}} = \frac{N(0)}{D'(0) \cdot \varepsilon^k} \, e^{-jk\theta}$$

This means:
- **Radius**: $\propto \frac{1}{\varepsilon^k} \to \infty$ — the indentation maps to an **infinitely large arc**
- **Angle**: $-k\theta$, spanning from $-k(+\pi/2)$ to $-k(-\pi/2)$, i.e., an arc of $k\pi$ radians **clockwise**

#### Summary by Pole Multiplicity

| Pole multiplicity $k$ | Arc radius | Arc span (clockwise) | Appearance in Nyquist plot |
|------------------------|------------|----------------------|----------------------------|
| $k = 1$ (single integrator) | $\to \infty$ | $\pi$ (180°) | Half-circle at infinity, CW |
| $k = 2$ (double integrator) | $\to \infty$ | $2\pi$ (360°) | Full circle at infinity, CW |
| $k = 3$ (triple integrator) | $\to \infty$ | $3\pi$ (540°) | 1.5 circles at infinity, CW |

#### Example: Single Integrator

For $G(s) = \dfrac{K}{s(s+a)}$, the pole at $s = 0$ has multiplicity $k = 1$.

On the indentation $s = \varepsilon e^{j\theta}$, $\theta: +90° \to -90°$:
$$G \approx \frac{K}{a\varepsilon} e^{-j\theta}$$

As $\theta$ goes from $+90°$ to $-90°$, the angle $-\theta$ goes from $-90°$ to $+90°$: a **clockwise semicircle of infinite radius** sweeping from $-90°$ to $+90°$ in the $G$-plane.

```
    Im{GH}
      ↑
      │              Indentation maps to
      │              this infinite arc (CW)
      │
  ────┼──────────────────────● ──▶ Re{GH}
      │                  ╱
      │               ╱    R → ∞
      │            ╱
      │         ╱
      │      ●  (from ω = 0⁺)
      │
```

#### Poles Not at the Origin

For poles at $s = \pm j\omega_0$ (undamped oscillatory modes), the same principle applies. Indent to the right around each pole:
$$s = j\omega_0 + \varepsilon e^{j\theta}, \quad \theta: +\frac{\pi}{2} \to -\frac{\pi}{2}$$

The mapped arc has radius $\to \infty$ and spans $k\pi$ radians clockwise, where $k$ is the pole multiplicity at that location.

---

## 7.4 Nyquist Stability Criterion

### 7.4.1 Main Theorem

> **Nyquist Criterion:**
> 
> A feedback system with open-loop transfer function $G(s)H(s)$ is closed-loop stable if and only if the Nyquist plot of $G(j\omega)H(j\omega)$ encircles the point (-1, 0) exactly P times **counter-clockwise**, where P is the number of right-half plane poles of $G(s)H(s)$.

### 7.4.2 Special Cases

| Case | Condition | Interpretation |
|------|-----------|----------------|
| P = 0, N = 0 | No encirclements | Stable |
| P = 0, N > 0 | CW encirclements | Unstable |
| P > 0, N = -P | CCW encirclements | Stable |

### 7.4.3 Encirclement Counting Rules

1. Trace the Nyquist plot for ω: 0 → ∞
2. Count crossings of the **negative real axis** left of (-1, 0)
3. Upward crossing = +1, Downward crossing = -1
4. Total = N (number of CW encirclements)

---

## 7.5 Stability Margins from Nyquist Plot

### 7.5.1 Gain Margin

The **Gain Margin** is determined from the Nyquist plot at the **phase crossover frequency** $\omega_{pc}$, where the plot crosses the negative real axis (i.e., $\angle G(j\omega) = -180°$).

At this point, $G(j\omega_{pc})H(j\omega_{pc})$ is a negative real number. The gain margin is:
$$GM = \frac{1}{|G(j\omega_{pc})H(j\omega_{pc})|}$$

In decibels:
$$GM_{dB} = -20\log_{10}|G(j\omega_{pc})H(j\omega_{pc})| = 20\log_{10}\left(\frac{1}{|G(j\omega_{pc})H(j\omega_{pc})|}\right)$$

**Graphical interpretation on the Nyquist plot:**

```
    Im{GH}
      ↑
      │
      │
  ────●───────●────────────────────▶ Re{GH}
   (-1,0)    (-a,0)
      │
      │   a = |G(jω_pc)H(jω_pc)|    at phase = -180°
      │
      │   GM = 1/a
      │   If a < 1: GM > 1 (stable, plot doesn't reach -1)
      │   If a > 1: GM < 1 (unstable, plot passes beyond -1)
      │   If a = 1: GM = 1 (marginally stable, plot hits -1)
```

The gain margin tells you: **by what factor can you multiply the gain before the Nyquist plot passes through (-1, 0)?**

### 7.5.2 Phase Margin

The **Phase Margin** is determined from the Nyquist plot at the **gain crossover frequency** $\omega_{gc}$, where the plot crosses the **unit circle** (i.e., $|G(j\omega)H(j\omega)| = 1$).

The phase margin is the angular distance from the negative real axis to this crossing point:
$$PM = 180° + \angle G(j\omega_{gc})H(j\omega_{gc})$$

**Graphical interpretation on the Nyquist plot:**

```
    Im{GH}
      ↑
              Unit circle
           ╱──────╲
         ╱    ●     ╲     ← G(jω_gc)H(jω_gc) on unit circle
        │   ╱  PM    │
  ──────●──╱─────────●─────▶ Re{GH}
     (-1,0)       (1,0)
        │            │
         ╲          ╱
           ╲──────╱
      │
      │   PM = angle from (-1,0) direction to the crossing point
      │   PM > 0: stable (crossing is above the negative real axis)
      │   PM < 0: unstable (crossing is below the negative real axis)
```

The phase margin tells you: **how much additional phase lag can the system tolerate before the Nyquist plot rotates to hit (-1, 0)?**

### 7.5.3 Relationship Between Margins

For a well-designed control system, typical specifications are:
- **Gain Margin**: $GM \geq 6$ dB (factor of 2)
- **Phase Margin**: $30° \leq PM \leq 60°$

Both margins should be positive simultaneously for robust stability. A system with good PM but poor GM (or vice versa) may still be fragile.

### 7.5.4 Reading Margins with cppplot

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;

// Define the open-loop transfer function
TransferFunction G({10}, {1, 3, 2, 0});  // 10 / (s^3 + 3s^2 + 2s)

// Plot Nyquist diagram — margins are annotated automatically
auto [fig, info] = nyquist(G);

std::cout << "Gain Margin: " << info.gain_margin << " dB\n";
std::cout << "Phase Margin: " << info.phase_margin << " deg\n";
std::cout << "Phase Crossover Freq: " << info.omega_pc << " rad/s\n";
std::cout << "Gain Crossover Freq: " << info.omega_gc << " rad/s\n";
```

---

## 7.6 Examples

### Example 7.1: First-Order System

$$G(s) = \frac{K}{s+1}$$

- P = 0 (no RHP poles)
- Plot starts at K (ω = 0) and approaches 0 (ω → ∞)
- No encirclement of (-1, 0) for any K > 0
- **Conclusion:** Always stable!

**cppplot code:**

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;

TransferFunction G({5}, {1, 1});  // G(s) = 5/(s+1)
nyquist(G);                       // Plots semi-circle in lower-right quadrant
```

### Example 7.2: Third-Order System

$$G(s) = \frac{K}{s(s+1)(s+2)}$$

- P = 0 (no RHP poles)
- For small K: No encirclement → Stable
- As K increases: Plot expands and may encircle (-1, 0)

**Finding Critical Gain:**

At $\omega = \omega_{pc}$ where phase = -180°:
$$\angle G(j\omega) = -90° - \arctan(\omega) - \arctan(\omega/2) = -180°$$

Solving: $\arctan(\omega) + \arctan(\omega/2) = 90°$

Using $\tan^{-1}(a) + \tan^{-1}(b) = 90°$ when $ab = 1$:
$$\omega \cdot (\omega/2) = 1 \Rightarrow \omega_{pc} = \sqrt{2}$$

Magnitude at $\omega_{pc}$:
$$|G(j\sqrt{2})| = \frac{K}{\sqrt{2} \cdot \sqrt{3} \cdot \sqrt{6}} = \frac{K}{6}$$

**Critical gain:** $K_{cr} = 6$ (when $|G| = 1$ at phase crossover)

**Conclusion:** System stable for $0 < K < 6$

**cppplot code:**

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;

// Compare stable vs unstable gain
TransferFunction G_stable({4}, {1, 3, 2, 0});    // K=4: stable (K < 6)
TransferFunction G_unstable({10}, {1, 3, 2, 0});  // K=10: unstable (K > 6)

figure();
nyquist(G_stable, NyquistOptions({{"label", "K=4 (stable)"}}));
nyquist(G_unstable, NyquistOptions({{"label", "K=10 (unstable)"}}));
title("Example 7.2: Effect of Gain on Nyquist Plot");
legend();
```

### Example 7.3: Unstable Open-Loop System

Consider a plant with an **unstable open-loop pole**:
$$G(s) = \frac{2(s+1)}{(s-1)(s+3)}$$

#### Step 1: Identify Open-Loop RHP Poles

The denominator $(s-1)(s+3) = 0$ gives poles at $s = +1$ and $s = -3$.

**One RHP pole:** $P = 1$

#### Step 2: Stability Requirement

For closed-loop stability we need $Z = 0$:
$$Z = N + P = 0 \implies N = -P = -1$$

We need **one counter-clockwise (CCW) encirclement** of $(-1, 0)$.

#### Step 3: Frequency Response

Substitute $s = j\omega$:
$$G(j\omega) = \frac{2(j\omega + 1)}{(j\omega - 1)(j\omega + 3)}$$

**Key points:**
- At $\omega = 0$: $G(0) = \frac{2 \cdot 1}{(-1)(3)} = -\frac{2}{3}$
- As $\omega \to \infty$: $|G| \to 0$
- Phase at $\omega = 0$: $\angle G(0) = 0° - 180° - 0° = -180°$ (starts on negative real axis)

The phase starts at $-180°$ and increases (becomes less negative) because the RHP pole contributes positive phase:
$$\angle G(j\omega) = \arctan(\omega) - (180° - \arctan(\omega)) - \arctan(\omega/3)$$

#### Step 4: Sketch and Encirclement Count

```
    Im{GH}
      ↑
      │         ω increases
      │        ╱
      │       ╱
      │      ╱  Plot goes CCW
  ────●───●─╱──────────────▶ Re{GH}
   (-1,0)(-2/3)             origin
      │      ╲
      │       ╲
      │        ╲  (conjugate mirror)
      │
      │  The plot makes 0 net encirclements of (-1,0)
      │  N = 0, P = 1  ⟹  Z = N + P = 1  ✗ UNSTABLE!
      │
      │  Note: For K=6 (above the Routh boundary K>3), the plot
      │  makes 1 CCW encirclement: N = -1, P = 1, Z = 0  ✓ Stable
```

#### Step 5: Verify

Closed-loop characteristic equation: $1 + G(s) = 0$
$$(s-1)(s+3) + 2(s+1) = 0$$
$$s^2 + 2s - 3 + 2s + 2 = 0$$
$$s^2 + 4s - 1 = 0$$
$$s = \frac{-4 \pm \sqrt{16 + 4}}{2} = \frac{-4 \pm \sqrt{20}}{2} = -2 \pm \sqrt{5}$$

$s_1 = -2 + 2.236 = 0.236$ (RHP!) — Wait, let's recheck with unity feedback $H(s) = 1$:

Actually, the characteristic polynomial is $(s-1)(s+3) + 2(s+1) = s^2 + 4s - 1 = 0$. The roots are $s = -2 \pm \sqrt{5}$, giving $s \approx 0.236$ and $s \approx -4.236$. With this gain, the system is **not** stable.

Let's increase the gain. With $G(s) = \frac{K(s+1)}{(s-1)(s+3)}$, the characteristic equation becomes:
$$(s-1)(s+3) + K(s+1) = 0 \implies s^2 + (2+K)s + (K-3) = 0$$

By Routh's criterion, we need $2 + K > 0$ and $K - 3 > 0$, so **$K > 3$** for stability.

With $K = 6$:
$$s^2 + 8s + 3 = 0 \implies s = -4 \pm \sqrt{13} \approx -0.39, -7.61$$

Both poles in LHP — stable! The Nyquist plot for $K = 6$ makes exactly 1 CCW encirclement of $(-1, 0)$.

**cppplot code:**

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;

// Unstable OL system that needs CCW encirclement
TransferFunction G({6, 6}, {1, 2, -3});  // 6(s+1)/((s-1)(s+3))
nyquist(G);
```

### Example 7.4: Conditionally Stable System

Some systems are stable only for a **range** of gain values — increasing gain too much OR too little causes instability. This is called **conditional stability**.

Consider:
$$G(s) = \frac{K}{(s+1)(s+2)(s+3)}$$

#### Open-Loop Analysis

- **Poles:** $s = -1, -2, -3$ (all in LHP)
- $P = 0$ — no open-loop RHP poles
- For stability we need $N = 0$ (no encirclements of $-1$)

#### Frequency Response

$$G(j\omega) = \frac{K}{(j\omega+1)(j\omega+2)(j\omega+3)}$$

Magnitude:
$$|G(j\omega)| = \frac{K}{\sqrt{1+\omega^2}\sqrt{4+\omega^2}\sqrt{9+\omega^2}}$$

Phase:
$$\angle G(j\omega) = -\arctan(\omega) - \arctan(\omega/2) - \arctan(\omega/3)$$

#### Phase Crossover

Set phase $= -180°$:
$$\arctan(\omega) + \arctan(\omega/2) + \arctan(\omega/3) = 180°$$

This equation has a single solution at $\omega_{pc} \approx \sqrt{11} \approx 3.317$ rad/s.

(Using $\tan(\arctan a + \arctan b) = \frac{a+b}{1-ab}$ iteratively.)

At $\omega_{pc}$:
$$|G(j\omega_{pc})| = \frac{K}{\sqrt{12}\sqrt{15}\sqrt{20}} = \frac{K}{60}$$

Critical gain: $K_{cr} = 60$.

For this simple 3-pole system, it is **stable for $0 < K < 60$** and unstable for $K > 60$.

#### Making It Conditionally Stable

For true conditional stability, consider a more complex system:
$$G(s) = \frac{K(s+5)^2}{s(s+1)(s+2)(s+10)^2}$$

This type of system has **two phase crossover frequencies** $\omega_1 < \omega_2$, meaning the phase dips below $-180°$ and comes back above $-180°$ before eventually dropping below again.

The Nyquist plot crosses the negative real axis **twice** (besides the origin). This creates **three gain regions**:

| Gain Range | Encirclements of $(-1,0)$ | Stability |
|------------|---------------------------|-----------|
| $0 < K < K_1$ | $N = 0$ | Stable |
| $K_1 < K < K_2$ | $N = 2$ (CW) | **Unstable** |
| $K_2 < K < K_3$ | $N = 0$ | **Stable** |
| $K > K_3$ | $N = 2$ (CW) | **Unstable** |

```
    Im{GH}
      ↑              Conditionally Stable System
      │
      │     K increasing → plot expands
      │
  ────●────x─────x────────────▶ Re{GH}
   (-1,0)  A     B    origin
      │    ↑     ↑
      │    │     │
      │  (-1/K₃) (-1/K₁)     Negative real axis crossings
      │
      │  When (-1,0) is between A and B: N=2 → unstable
      │  When (-1,0) is left of A or right of B: N=0 → stable
```

> **Engineering Warning:** Conditionally stable systems are dangerous because a momentary gain reduction (e.g., sensor saturation) can push the system into the unstable region. Avoid conditional stability in safety-critical designs.

**cppplot code:**

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;

// Conditionally stable system
// G(s) = K(s+5)^2 / (s(s+1)(s+2)(s+10)^2)
Poly num = poly({-5, -5});         // (s+5)^2
Poly den = poly({0, -1, -2, -10, -10});  // s(s+1)(s+2)(s+10)^2

figure();
for (double K : {10.0, 50.0, 200.0, 500.0}) {
    TransferFunction G(K * num, den);
    nyquist(G, NyquistOptions({{"label", "K=" + std::to_string((int)K)}}));
}
title("Conditional Stability: Effect of K");
legend();
```

### Example 7.5: Complete Worked Nyquist Analysis

Perform a complete Nyquist analysis for:
$$G(s) = \frac{10}{s(s+1)(s+2)}$$

This is a **Type 1** system (one integrator) with no explicit $H(s)$ (unity feedback).

#### Step 1: Identify P (Open-Loop RHP Poles)

Open-loop poles: $s = 0, -1, -2$

All poles are in the LHP or on the jω-axis. The pole at $s = 0$ is **on** the jω-axis, not in the RHP.

$$\boxed{P = 0}$$

For stability, we need $N = 0$ (no encirclements of $-1$).

#### Step 2: Handle the jω-Axis Pole (Indentation)

Since $G(s)$ has a pole at $s = 0$ with multiplicity $k = 1$, we indent the Nyquist contour with a small semicircle:
$$s = \varepsilon e^{j\theta}, \quad \theta: +90° \to -90°$$

On this indentation:
$$G(\varepsilon e^{j\theta}) = \frac{10}{\varepsilon e^{j\theta}(\varepsilon e^{j\theta}+1)(\varepsilon e^{j\theta}+2)} \approx \frac{10}{2\varepsilon e^{j\theta}} = \frac{5}{\varepsilon}e^{-j\theta}$$

As $\varepsilon \to 0$, this traces a **clockwise semicircle of infinite radius**:
- From $\theta = +90°$: $G \to \frac{5}{\varepsilon}e^{-j90°}$ (pointing downward, $-j\infty$)
- Through $\theta = 0°$: $G \to \frac{5}{\varepsilon}$ (positive real axis, $+\infty$)
- To $\theta = -90°$: $G \to \frac{5}{\varepsilon}e^{+j90°}$ (pointing upward, $+j\infty$)

The indentation maps to a **CW semicircle from $-j\infty$ through $+\infty$ to $+j\infty$**.

#### Step 3: Frequency Response for $\omega: 0^+ \to \infty$

$$G(j\omega) = \frac{10}{j\omega(j\omega+1)(j\omega+2)}$$

Magnitude:
$$|G(j\omega)| = \frac{10}{\omega\sqrt{1+\omega^2}\sqrt{4+\omega^2}}$$

Phase:
$$\angle G(j\omega) = -90° - \arctan(\omega) - \arctan(\omega/2)$$

**Key frequency points:**

| $\omega$ (rad/s) | $|G(j\omega)|$ | $\angle G(j\omega)$ | Point in GH-plane |
|-------------------|-----------------|---------------------|--------------------|
| $0^+$ | $\to \infty$ | $-90°$ | $-j\infty$ |
| $0.5$ | $\approx 8.68$ | $-90° - 26.6° - 14.0° = -130.6°$ | Quadrant III |
| $1.0$ | $\approx 3.162$ | $-90° - 45° - 26.6° = -161.6°$ | Near negative real axis |
| $\sqrt{2}$ | $\approx 1.667$ | $-90° - 54.7° - 35.3° = -180°$ | Negative real axis |
| $2.0$ | $\approx 0.791$ | $-90° - 63.4° - 45° = -198.4°$ | Quadrant II |
| $5.0$ | $\approx 0.0728$ | $-90° - 78.7° - 68.2° = -236.9°$ | Near origin |
| $\infty$ | $0$ | $-270°$ | Origin |

#### Step 4: Phase Crossover — Find $\omega_{pc}$

Set $\angle G = -180°$:
$$-90° - \arctan(\omega) - \arctan(\omega/2) = -180°$$
$$\arctan(\omega) + \arctan(\omega/2) = 90°$$

Using $\arctan(a) + \arctan(b) = 90°$ when $ab = 1$:
$$\omega \cdot \frac{\omega}{2} = 1 \implies \omega^2 = 2 \implies \omega_{pc} = \sqrt{2} \approx 1.414 \text{ rad/s}$$

#### Step 5: Gain Margin

$$|G(j\sqrt{2})| = \frac{10}{\sqrt{2}\cdot\sqrt{3}\cdot\sqrt{6}} = \frac{10}{\sqrt{36}} = \frac{10}{6} = 1.667$$

Since $|G| > 1$ at phase crossover, the Nyquist plot crosses the negative real axis **beyond** $(-1, 0)$.

$$GM = \frac{1}{1.667} = 0.6 \quad (\approx -4.4 \text{ dB})$$

**Gain margin is negative!** The plot encircles $(-1, 0)$.

#### Step 6: Count Encirclements

The Nyquist plot for $\omega: 0^+ \to \infty$:
- Starts at $-j\infty$ (from the indentation)
- Sweeps through the third and second quadrants
- Crosses the negative real axis at $(-1.667, 0)$ — to the LEFT of $(-1, 0)$
- Spirals toward the origin

With the conjugate mirror (negative frequencies), the complete Nyquist plot makes **2 clockwise encirclements** of $(-1, 0)$.

$$N = 2$$

#### Step 7: Apply Nyquist Criterion

$$Z = N + P = 2 + 0 = 2$$

**Two unstable closed-loop poles!** The closed-loop system is **unstable** with $K = 10$.

#### Step 8: Find the Gain for Marginal Stability

For marginal stability, the Nyquist plot must pass through $(-1, 0)$:
$$\frac{K}{6} = 1 \implies K_{cr} = 6$$

For $K < 6$: $|G(j\omega_{pc})| < 1$, no encirclement, $N = 0$, $Z = 0$ → **Stable**

For $K > 6$: $|G(j\omega_{pc})| > 1$, two CW encirclements, $N = 2$, $Z = 2$ → **Unstable**

#### Step 9: Phase Margin (at $K = 4$, a stable case)

With $K = 4$, find $\omega_{gc}$ where $|G(j\omega)| = 1$:
$$\frac{4}{\omega\sqrt{1+\omega^2}\sqrt{4+\omega^2}} = 1$$

Solving numerically: $\omega_{gc} \approx 1.08$ rad/s.

Phase at $\omega_{gc}$:
$$\angle G(j \cdot 1.08) = -90° - \arctan(1.08) - \arctan(0.54) \approx -90° - 47.2° - 28.4° = -165.6°$$

$$PM = 180° + (-165.6°) = 14.4°$$

> This is a rather poor phase margin (< 30°), indicating the system will have significant overshoot.

#### Complete cppplot Code

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;

// G(s) = K / (s(s+1)(s+2))
// Test three gain values
figure();

TransferFunction G1({4}, {1, 3, 2, 0});    // K=4: stable
TransferFunction G2({6}, {1, 3, 2, 0});    // K=6: marginally stable
TransferFunction G3({10}, {1, 3, 2, 0});   // K=10: unstable

nyquist(G1, NyquistOptions({{"label", "K=4 (stable)"},  {"color", "green"}}));
nyquist(G2, NyquistOptions({{"label", "K=6 (marginal)"}, {"color", "orange"}}));
nyquist(G3, NyquistOptions({{"label", "K=10 (unstable)"},{"color", "red"}}));

title("Example 7.5: G(s) = K/[s(s+1)(s+2)] — Nyquist Analysis");
legend();
savefig("nyquist_example_7_5.svg");
```

---

## 7.7 Systems with Time Delay

### 7.7.1 Time Delay Transfer Function

A time delay of $L$ seconds in the loop (also called **transport lag** or **dead time**) has the transfer function:
$$G_d(s) = e^{-Ls}$$

In frequency domain ($s = j\omega$):
$$G_d(j\omega) = e^{-jL\omega}$$

- **Magnitude:** $|G_d(j\omega)| = |e^{-jL\omega}| = 1$ for all $\omega$ — no amplitude change
- **Phase:** $\angle G_d(j\omega) = -L\omega$ radians $= -\frac{180L\omega}{\pi}$ degrees — **linearly increasing phase lag**

This is the key insight: **time delay adds phase lag without changing magnitude**. The phase lag grows linearly with frequency, so at high enough frequencies, any amount of delay will push the phase past $-180°$.

### 7.7.2 Effect on Nyquist Plot

The delay factor $e^{-jL\omega}$ **rotates** each point of the Nyquist plot clockwise by an angle $L\omega$ (proportional to frequency). Since higher-frequency points get rotated more, the Nyquist plot **spirals inward** toward the origin:

```
    Im{GH}
      ↑
      │         Without delay: smooth curve
      │        ╱
      │       ╱
  ────●──────╱─────────────────▶ Re{GH}
   (-1,0)
      │
      │
    Im{GH}
      ↑
      │         With delay: spiral
      │        ╱
      │      ╱ ╲  ╱
  ────●────╱────╲╱─────────────▶ Re{GH}
   (-1,0) ╱
      │  ╱  The spiral may cross
      │      through (-1,0)!
```

### 7.7.3 Maximum Allowable Time Delay

Given a system $G_0(s)$ without delay that has:
- **Gain crossover frequency:** $\omega_{gc}$
- **Phase margin without delay:** $PM_0$

Adding a delay $L$ reduces the phase margin by $L\omega_{gc}$ radians:
$$PM = PM_0 - L\omega_{gc}$$

The system becomes unstable when $PM = 0$:
$$L_{\max} = \frac{PM_0}{\omega_{gc}}$$

where $PM_0$ is in **radians**. In degrees:
$$L_{\max} = \frac{PM_0 \,(\text{degrees})}{\omega_{gc}} \cdot \frac{\pi}{180}$$

#### Example: Delay in a First-Order System

$$G(s) = \frac{2}{s+1} \cdot e^{-Ls}$$

Without delay ($L = 0$):
- $|G(j\omega_{gc})| = 1 \implies \omega_{gc} = \sqrt{3}$ rad/s
- $PM_0 = 180° + (-\arctan\sqrt{3}) = 180° - 60° = 120°$

Maximum delay:
$$L_{\max} = \frac{120° \cdot \pi/180}{\sqrt{3}} = \frac{2\pi/3}{\sqrt{3}} = \frac{2\pi}{3\sqrt{3}} \approx 1.21 \text{ s}$$

### 7.7.4 Padé Approximation

For numerical simulation and controller design, delays are often approximated using the **Padé approximation**:

**First-order Padé:**
$$e^{-Ls} \approx \frac{1 - Ls/2}{1 + Ls/2} = \frac{2 - Ls}{2 + Ls}$$

**Second-order Padé:**
$$e^{-Ls} \approx \frac{1 - Ls/2 + (Ls)^2/12}{1 + Ls/2 + (Ls)^2/12}$$

Note: Padé approximations introduce **RHP zeros**, creating a non-minimum phase system. The Nyquist plot with Padé approximation closely matches the true delay for frequencies below approximately $\omega < 5/L$.

### 7.7.5 cppplot Delay Example

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;

// System: G(s) = 2/(s+1) with various delays
TransferFunction G0({2}, {1, 1});  // No delay

figure();
nyquist(G0, NyquistOptions({{"label", "L=0"}}));

for (double L : {0.3, 0.6, 1.0, 1.5}) {
    auto G_delayed = G0 * delay(L);  // Add time delay
    nyquist(G_delayed, NyquistOptions({{"label", "L=" + std::to_string(L)}}));
}
title("Effect of Time Delay on Nyquist Plot");
legend();
savefig("nyquist_time_delay.svg");
```

---

## 📝 Exercises

### Exercise 7.1 — Nyquist Criterion Fundamentals

**(a)** State the Nyquist stability criterion precisely. Define all variables: $N$, $P$, $Z$, and the direction convention for encirclements.

**(b)** A system $G(s)$ has $P = 2$ open-loop RHP poles. If the Nyquist plot of $G(s)$ encircles the point $(-1, 0)$ **twice clockwise**, how many closed-loop RHP poles ($Z$) does the system have? Is the closed-loop system stable?

**(c)** For the same system, how many **counterclockwise** encirclements of $(-1, 0)$ would be required for closed-loop stability?

**(d)** Explain why Bode-based stability analysis (using gain/phase margins) can fail for systems with $P > 0$, while the Nyquist criterion remains valid.

---

### Exercise 7.2 — Critical Gain via Nyquist Analysis

For the system:

$$G(s) = \frac{K}{(s+1)^3}$$

**(a)** Find the frequency $\omega_{180}$ where $\angle G(j\omega) = -180°$.

> *Hint:* Each $(j\omega + 1)$ factor contributes $-\arctan(\omega)$. Set $3\arctan(\omega_{180}) = 180°$.

**(b)** Compute $|G(j\omega_{180})|$ in terms of $K$.

**(c)** Find the critical gain $K_{\text{critical}}$ such that the Nyquist plot passes through $(-1, 0)$.

**(d)** For $K = K_{\text{critical}}/2$, what is the gain margin in dB?

---

### Exercise 7.3 — Nyquist Plot Sketch with jω-Axis Pole

For the system:

$$G(s) = \frac{1}{s(s+1)}$$

**(a)** Identify the poles of $G(s)$. How many are on the $j\omega$-axis? How many are in the RHP ($P = ?$)?

**(b)** Describe the Nyquist contour, including the small semicircular indentation around the pole at the origin.

**(c)** Compute $G(j\omega)$ for several frequencies ($\omega = 0.5, 1, 2, 5$) and plot the points in the $G$-plane.

**(d)** Sketch the complete Nyquist plot, including:
- The contribution from the large semicircle ($|s| \to \infty$).
- The indentation around $s = 0$.
- The mirror image for $\omega < 0$.

**(e)** Count the encirclements of $(-1, 0)$ and determine closed-loop stability.

---

### Exercise 7.4 — Nyquist Verification with CppPlot

Verify your hand sketch from Exercise 7.3 using CppPlot:

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;

int main() {
    // G(s) = 1 / (s(s+1))
    TransferFunction G({1}, {1, 1, 0});
    
    figure();
    nyquist(G);
    title("Exercise 7.4: Nyquist Plot of 1/(s(s+1))");
    savefig("nyquist_exercise_7_4.svg");
    
    // Student task: Mark the critical point (-1, 0)
    // Student task: Annotate gain and phase margins
}
```

**(a)** Compare the CppPlot output with your hand sketch. Are the encirclements consistent?

**(b)** Modify the code to add a gain $K$ in the loop. Find the maximum $K$ before the Nyquist plot encircles $(-1, 0)$.

---

### Exercise 7.5 — Gain and Phase Margins from Nyquist Plot

For the system:

$$G(s) = \frac{10}{s(s+1)(s+2)}$$

**(a)** Find the phase crossover frequency $\omega_{pc}$ where $\angle G(j\omega) = -180°$.

**(b)** Compute $|G(j\omega_{pc})|$ and determine the gain margin $GM = 1/|G(j\omega_{pc})|$ (also express in dB).

**(c)** Find the gain crossover frequency $\omega_{gc}$ where $|G(j\omega)| = 1$.

**(d)** Compute the phase margin $PM = 180° + \angle G(j\omega_{gc})$.

**(e)** On the Nyquist plot, identify the points corresponding to GM and PM geometrically.

---

### Exercise 7.6 — Time Delay and Stability

Consider a first-order system with time delay:

$$G(s) = \frac{e^{-0.5s}}{s+1}$$

**(a)** Show that the time delay $e^{-j\omega T}$ adds phase $-\omega T$ (in radians) without changing the magnitude.

**(b)** For $T = 0.5$ s, find the frequency where the total phase reaches $-180°$.

**(c)** Compute $|G(j\omega)|$ at that frequency. What is the gain margin?

**(d)** Find the **maximum additional delay** $\Delta T$ (beyond the existing $T = 0.5$ s) that the system can tolerate before becoming unstable.

> *Hint:* The system becomes unstable when the Nyquist plot crosses $(-1, 0)$, i.e., when $|G| = 1$ and $\angle G = -180°$ simultaneously.

---

### Exercise 7.7 — Nyquist for Unstable Open-Loop Plant ⭐

For the system with an unstable (RHP) pole:

$$G(s) = \frac{K(s+2)}{(s-1)(s+5)}$$

**(a)** Identify the open-loop poles and determine $P$ (number of RHP poles).

**(b)** For $K = 1$, sketch the Nyquist plot. Pay attention to:
- The starting point ($\omega = 0$): compute $G(0)$.
- The ending point ($\omega \to \infty$): compute $\lim_{\omega \to \infty} G(j\omega)$.
- The real-axis crossing: find $\omega$ where $\text{Im}\{G(j\omega)\} = 0$.

**(c)** For the closed-loop to be stable, how many **counterclockwise** encirclements of $(-1, 0)$ are needed?

**(d)** Determine the range of $K > 0$ for which the closed-loop system is stable. Verify with the Routh criterion.

**(e)** Does a negative range of $K$ also yield stability? If so, find it.

### Problem Identification Exercises (Level 3-4)

**Exercise 7.8 — What Is the Real Problem?**
A Bode analysis of a control loop shows GM = 6dB and PM = 30° — marginal but acceptable. However, the plant has a 50ms transport delay that was not included in the Bode analysis.

(a) Explain why Bode analysis alone cannot reliably assess stability when delays are present. (Hint: does delay change the magnitude plot?)
(b) Using the Nyquist criterion, explain how the delay causes the Nyquist plot to spiral, potentially encircling $(-1, 0)$.
(c) The real problem is not "the system is unstable" but rather "the engineer used the wrong analysis tool." Propose a systematic decision framework: when should an engineer use Bode vs. Nyquist?

**Exercise 7.9 — Mechanism vs. Procedure**
A student counts encirclements of $(-1, 0)$ and determines $N = -2$. With $P = 0$, the Nyquist criterion gives $Z = 0 + (-2)$… which is negative. The student is confused.

(a) Explain the sign convention: what does $N < 0$ mean in terms of CW vs. CCW encirclements?
(b) For a stable open-loop system ($P = 0$), what is the *physical* meaning of "no encirclements"? (Hint: the loop gain signal never reaches the critical magnitude-and-phase combination.)
(c) Draw a simple example where $P = 2$ and the system IS stable. What must $N$ be, and what does this mean physically?

---

## What Comes Next

**Chapter 8: Frequency-Domain Controller Design** — Now that you can determine stability from both Bode plots (Chapter 6) and the Nyquist diagram (this chapter), the next step is to *design* controllers that achieve desired gain and phase margins. Lead compensators add phase; lag compensators boost low-frequency gain; PID controllers combine both effects. Chapter 8 provides systematic design procedures grounded in the frequency-domain understanding you've built.

---

## References
