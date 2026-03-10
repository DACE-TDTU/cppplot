# Chapter 16: Introduction to Robust Control

---

## 🎯 Learning Objectives and Outcomes

### Chapter Objectives

This chapter introduces robust control concepts, addressing the critical practical issue that mathematical models never perfectly represent real systems. Students learn to analyze and design controllers that maintain stability and performance despite model uncertainty.

### Prerequisites
- Chapter 6-8: Frequency domain analysis and design
- Chapter 7: Nyquist criterion

---

## Why This Chapter Matters: Models Lie (But We Use Them Anyway)

> **The Real Engineering Problem:** Your controller works perfectly in simulation. You deploy it on the real system. It oscillates, overshoots, or worst case - goes unstable. What happened?

### The Uncomfortable Truth About Models

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    WHERE DOES UNCERTAINTY COME FROM?                        │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   SOURCE 1: Parameter Uncertainty (You can't measure exactly)              │
│   ─────────────────────────────────────────────────────────                │
│   • Motor resistance: Datasheet says 2.5Ω, actual could be 2.3-2.8Ω       │
│   • Inertia: Varies with payload, manufacturing tolerance                  │
│   • Friction: Changes with temperature, wear, lubrication                  │
│   • Spring constant: ±5-10% from nominal even in precision springs         │
│                                                                             │
│   SOURCE 2: Unmodeled Dynamics (You didn't include everything)             │
│   ────────────────────────────────────────────────────────────             │
│   • Motor: You modeled L, R, J, B but ignored armature inductance time lag │
│   • Structure: Rigid body model ignores flexible modes at high frequency   │
│   • Hydraulics: Simplified model ignores oil compressibility               │
│   • Thermal: Lumped model ignores distributed temperature gradients        │
│                                                                             │
│   SOURCE 3: Nonlinearity (Linear model is only approximation)              │
│   ──────────────────────────────────────────────────────────               │
│   • Saturation: Amplifier clips, motor torque limits                      │
│   • Dead zone: Gear backlash, valve overlap                               │
│   • Stiction: Breakaway friction different from running friction          │
│   • Hysteresis: Magnetic materials, pneumatic systems                      │
│                                                                             │
│   SOURCE 4: Time-Varying Parameters (System changes over time)             │
│   ────────────────────────────────────────────────────────────             │
│   • Temperature: Electronics drift, motor winding resistance changes       │
│   • Wear: Bearings degrade, brushes wear, seals leak                      │
│   • Load: Robot picks up different objects, vehicle weight varies          │
│   • Aging: Capacitors dry out, springs fatigue                            │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Real-World Uncertainty Example: Industrial Servo Motor

```
NOMINAL MODEL (from datasheet):
  G(s) = Km/[(Js+B)(Ls+R) + KmKe]
  
  Km = 0.5 Nm/A,  J = 0.001 kg·m²,  B = 0.01 Nm·s/rad
  L = 0.005 H,    R = 2.5 Ω,        Ke = 0.5 V·s/rad

ACTUAL VALUES (measured over 100 motors + operating conditions):
  Km = 0.45 - 0.55 Nm/A          (±10% manufacturing variation)
  J  = 0.0008 - 0.0015 kg·m²     (±25% with different loads)
  B  = 0.005 - 0.02 Nm·s/rad     (100% variation with temperature!)
  R  = 2.0 - 3.5 Ω               (40% variation hot vs cold!)
  L  = 0.003 - 0.008 H           (±40% due to saturation)

YOUR CONTROLLER MUST WORK FOR ALL COMBINATIONS!
```

### The Robustness Question

| Design Approach | Question Answered |
|-----------------|-------------------|
| **Nominal design** | "What controller works for my model?" |
| **Robust stability** | "Will my controller be stable for ALL possible plants?" |
| **Robust performance** | "Will my controller meet specs for ALL possible plants?" |

---

## Learning Outcomes

After completing this chapter, students will be able to:

| Bloom's Level | Outcome |
|---------------|---------|
| **Remember** | Define sensitivity functions, uncertainty, and robustness |
| **Understand** | Explain the limitations of nominal design |
| **Apply** | Compute sensitivity functions for feedback systems |
| **Analyze** | Analyze robustness using sensitivity function bounds |
| **Evaluate** | Evaluate designs for robust stability and performance |
| **Create** | Design controllers with robustness considerations |

---

## 16.1 Why Robust Control?

### 16.1.1 Model Uncertainty

All models are approximate. Real systems have:
- **Parametric uncertainty:** Unknown exact values (mass, friction)
- **Unmodeled dynamics:** Neglected high-frequency modes
- **Nonlinearity:** Linearization errors
- **Time-varying parameters:** Wear, temperature effects

### 16.1.2 The Robustness Problem

A controller designed for the **nominal model** may:
- Lose stability for the real plant
- Degrade in performance
- Behave unexpectedly

**Goal:** Design controllers that work well despite uncertainty.

---

## 16.2 Feedback System Analysis

### 16.2.1 Standard Feedback Configuration

```
         r(t)     e(t)    ┌────┐   u(t)   ┌────┐   y(t)
    ──────►(+)───────────►│ K  │─────────►│ G  │───┬──►
           -▲             └────┘          └────┘   │
            │                                      │
            └──────────────────────────────────────┘
```

**Open-loop transfer function:** $L(s) = G(s)K(s)$

### 16.2.2 Closed-Loop Transfer Functions

| Function | Formula | Meaning |
|----------|---------|---------|
| **Sensitivity** S | $\frac{1}{1+L}$ | Error/Reference |
| **Complementary Sensitivity** T | $\frac{L}{1+L}$ | Output/Reference |
| **Control Sensitivity** KS | $\frac{K}{1+L}$ | Control/Reference |
| **Load Sensitivity** GS | $\frac{G}{1+L}$ | Output/Disturbance |

### 16.2.3 Fundamental Relationship

$$S(s) + T(s) = 1$$

**Implication:** Cannot make both S and T small at the same frequency.

---

## 16.3 Sensitivity Functions

### 16.3.1 Sensitivity Function S(s)

**Physical meaning:**
- Effect of disturbances at output
- Tracking error for reference inputs
- Effect of plant variations on closed-loop

**Desired:** $|S(j\omega)|$ small at low frequencies (good tracking, disturbance rejection)

### 16.3.2 Complementary Sensitivity T(s)

**Physical meaning:**
- Closed-loop transfer function from r to y
- Effect of measurement noise on output
- Determines robust stability margin

**Desired:** $|T(j\omega)|$ small at high frequencies (noise rejection)

### 16.3.3 Typical Specifications

| Frequency | S | T |
|-----------|---|---|
| Low | Small (< 0.1) | ≈ 1 |
| Crossover | ≈ 0.5-0.7 | ≈ 0.5-0.7 |
| High | ≈ 1 | Small (< 0.1) |

---

## 16.4 Robustness Margins

### 16.4.1 Classical Margins Revisited

**Gain Margin (GM):**
$$GM = \frac{1}{|L(j\omega_{pc})|}$$ 
where $\angle L(j\omega_{pc}) = -180°$

**Phase Margin (PM):**
$$PM = 180° + \angle L(j\omega_{gc})$$
where $|L(j\omega_{gc})| = 1$

### 16.4.2 Disk Margin

More comprehensive than GM/PM alone:
- Considers simultaneous gain and phase variations
- Related to distance from Nyquist plot to (-1, 0)

### 16.4.3 Maximum Sensitivity

$$M_S = \max_\omega |S(j\omega)| = \max_\omega \frac{1}{|1 + L(j\omega)|}$$

**Geometric interpretation:** $1/M_S$ = minimum distance from Nyquist plot to (-1, 0)

**Guidelines:**
| $M_S$ | Robustness |
|-------|------------|
| < 1.5 | Good |
| 1.5 - 2.0 | Acceptable |
| > 2.0 | Poor |

---

## 16.5 Uncertainty Models

### 16.5.1 Parametric Uncertainty

Parameters known within bounds:
$$p \in [p_{min}, p_{max}]$$

**Example:** Mass $m \in [0.9m_0, 1.1m_0]$ (±10% uncertainty)

### 16.5.2 Unstructured Uncertainty

**Additive uncertainty:**
$$G_p(s) = G(s) + \Delta_A(s)W_A(s)$$

**Multiplicative uncertainty:**
$$G_p(s) = G(s)(1 + \Delta_M(s)W_M(s))$$

where:
- $G(s)$: Nominal model
- $G_p(s)$: Actual plant
- $\Delta(s)$: Unknown stable transfer function with $|\Delta(j\omega)| \leq 1$
- $W(s)$: Weighting function capturing frequency-dependent uncertainty

### 16.5.3 Uncertainty Weighting

$W_M(j\omega)$ represents relative model error:
$$|W_M(j\omega)| = \frac{|G_p(j\omega) - G(j\omega)|}{|G(j\omega)|}$$

Typically increases with frequency (poor model at high frequencies).

---

## 16.6 Robust Stability

### 16.6.1 Multiplicative Uncertainty

For system with multiplicative uncertainty:
$$G_p = G(1 + W_M\Delta_M), \quad |\Delta_M| \leq 1$$

> **Theorem (Robust Stability):**
> The closed-loop system is stable for all $|\Delta_M| \leq 1$ if and only if:
> $$|T(j\omega)W_M(j\omega)| < 1, \quad \forall \omega$$
> 
> or equivalently:
> $$\|W_M T\|_\infty < 1$$

### 16.6.2 Additive Uncertainty

For additive uncertainty:
$$G_p = G + W_A\Delta_A, \quad |\Delta_A| \leq 1$$

Robust stability condition:
$$|KS(j\omega)W_A(j\omega)| < 1, \quad \forall \omega$$

### 16.6.3 The Small Gain Theorem

The robust stability conditions above are special cases of the **Small Gain Theorem**, the fundamental result underlying all robustness analysis.

> **Theorem (Small Gain Theorem):**
> Consider the feedback interconnection of two stable systems $\Delta$ and $M$. The closed-loop system is stable for all $\|\Delta\|_\infty \leq 1/\gamma$ if and only if:
> $$\|M\|_\infty < \gamma$$

**Standard M-Δ Structure (Linear Fractional Transformation):**

The proper M-Δ framework uses a 2×2 block partitioning of $M$:
$$M = \begin{bmatrix} M_{11} & M_{12} \\ M_{21} & M_{22} \end{bmatrix}$$

The **upper LFT** (uncertainty interconnection) is:
$$F_u(M, \Delta) = M_{22} + M_{21}\Delta(I - M_{11}\Delta)^{-1}M_{12}$$

The nominal closed-loop transfer function is $M_{22}$, and the structured uncertainty $\Delta$ enters through $M_{11}$. The system is well-posed if $(I - M_{11}\Delta)$ is invertible.

```
                    ┌─────────────────────┐
       ┌────────────┤      Δ (uncertainty) ├◄───────────┐
       │            └─────────────────────┘            │
       │ q                                          p │
       ▼                                              │
  ┌────────────────────────────────────────────────────┐
  │           ┌─────────┬─────────┐                    │
  │     q ──► │  M₁₁    │  M₁₂   │ ──► p              │
  │           ├─────────┼─────────┤                    │
  │     w ──► │  M₂₁    │  M₂₂   │ ──► z              │
  │           └─────────┴─────────┘                    │
  │                    M                               │
  └────────────────────────────────────────────────────┘
       ▲                                              │
       │ w (exogenous inputs)            z (outputs)  │
       │                                              ▼
```

Here $p, q$ are the signals connecting $M$ to $\Delta$, while $w, z$ are the external input/output channels.

**Proof sketch:** The closed-loop transfer function from $e \to y$ is $(I - M\Delta)^{-1}M$. By the **Neumann series**, the inverse $(I - M\Delta)^{-1}$ exists whenever $\|M\Delta\|_\infty < 1$. Since

$$\|M\Delta\|_\infty \leq \|M\|_\infty \cdot \|\Delta\|_\infty < \gamma \cdot \frac{1}{\gamma} = 1$$

the closed-loop is well-posed and internally stable. $\blacksquare$

**Connection to robust stability conditions:**
- For multiplicative uncertainty: $M = W_M T$ and $\Delta = \Delta_M$, so $\|W_M T\|_\infty < 1$
- For additive uncertainty: $M = W_A KS$ and $\Delta = \Delta_A$, so $\|W_A KS\|_\infty < 1$

---

## 16.7 Robust Performance

### 16.7.1 Definition

**Robust Performance:** Achieving performance specifications for all plants in the uncertainty set.

### 16.7.2 Performance Specifications

Typical requirements:
- Tracking: $|S(j\omega)| < 1/|W_P(j\omega)|$ for low ω
- Noise rejection: $|T(j\omega)|$ small for high ω
- Disturbance rejection: $|GS|$ bounded

### 16.7.3 Mixed Sensitivity

**Nominal performance:**
$$\|W_P S\|_\infty < 1$$

**Robust stability:**
$$\|W_M T\|_\infty < 1$$

**Combined (conservative):**
$$\|W_P S\|_\infty + \|W_M T\|_\infty < 1$$

### 16.7.4 Structured Singular Value (μ-Analysis)

The combined condition above is conservative because it treats all uncertainty as a single unstructured block. When uncertainty has known **structure** (e.g., separate parameter variations), the **structured singular value** $\mu$ provides a tighter, less conservative analysis.

**Definition:** For a complex matrix $M$ and a structured uncertainty set $\boldsymbol{\Delta}$, the structured singular value is:

$$\mu_{\boldsymbol{\Delta}}(M) = \frac{1}{\min\{\|\Delta\| : \Delta \in \boldsymbol{\Delta},\; \det(I - M\Delta) = 0\}}$$

with $\mu = 0$ if no $\Delta \in \boldsymbol{\Delta}$ makes $\det(I - M\Delta) = 0$.

**Key properties:**

| Case | Result | Interpretation |
|------|--------|----------------|
| **Full complex uncertainty** (single block) | $\mu(M) = \bar{\sigma}(M) = \|M\|_\infty$ | Reduces to the small gain theorem |
| **Structured uncertainty** (multiple blocks) | $\mu(M) \leq \bar{\sigma}(M)$ | Less conservative than small gain |

**Upper bound (D-scaling):**
$$\mu_{\boldsymbol{\Delta}}(M) \leq \inf_{D \in \mathcal{D}} \bar{\sigma}(DMD^{-1})$$

where $\mathcal{D}$ is the set of block-diagonal scaling matrices that commute with the uncertainty structure $\boldsymbol{\Delta}$. This upper bound is computable via convex optimization and is tight for up to three uncertainty blocks.

**Robust stability with structured uncertainty:**
$$\mu_{\boldsymbol{\Delta}}(M(j\omega)) < 1 \quad \forall \omega \implies \text{robust stability}$$

**Robust performance:** By augmenting the uncertainty block with a fictitious "performance block" $\Delta_P$, robust performance can also be cast as a $\mu$ condition, unifying both analyses.

### 16.7.5 D-K Iteration for μ-Synthesis

Since $\mu$ is generally difficult to compute exactly, **μ-synthesis** seeks to design a controller $K$ that minimizes $\mu$ across frequency. The practical algorithm is the **D-K iteration**, which alternates between two convex sub-problems:

1. **K-step:** Fix the D-scales, synthesize an $H_\infty$ controller $K$ to minimize:
   $$\|\hat{D}M\hat{D}^{-1}\|_\infty$$
   This is a standard $H_\infty$ problem and can be solved via Riccati equations or LMIs.

2. **D-step:** Fix $K$, and at each frequency $\omega$ find diagonal scaling $D(\omega)$ to minimize:
   $$\bar{\sigma}(D(\omega)M(\omega)D^{-1}(\omega))$$
   Then fit rational transfer functions to $D(\omega)$ to obtain a realizable $\hat{D}(s)$.

Repeat until convergence (or a maximum number of iterations).

**Convergence:** D-K iteration is **not guaranteed to converge** to the global optimum because:
- The overall problem is **non-convex** (bilinear in $K$ and $D$)
- Each sub-problem is convex individually, but alternation may find only **local minima**
- D-scale order (rational fitting) introduces approximation error

**Practical guidelines:**
- Start with **3–5 iterations**; monitor for decreasing $\mu$ upper bound
- Typical D-scale order: **2–4** (higher order rarely improves results and may cause numerical issues)
- Compare results across different initializations to increase confidence
- If the $\mu$ upper bound stops decreasing, increasing D-scale order by 1–2 may help

---

## 16.8 Fundamental Limitations

### 16.8.1 Waterbed Effect

**Bode's Integral Theorem:**
$$\int_0^{\infty} \ln|S(j\omega)| d\omega = \pi \sum_i \text{Re}(p_i)$$

where $p_i$ are the open-loop **RHP poles** (unstable poles).

**Conditions:**
- The open-loop transfer function $L(s)$ must have **relative degree ≥ 2** (at least 2 more poles than zeros)

> **Note on time delay:** Time delay $e^{-\tau s}$ does not alter this integral (since $|e^{-j\omega\tau}|=1$), but imposes additional point-wise constraints via the Poisson integral formula, making high-bandwidth control harder.

**Interpretation:** If the plant has RHP poles, making $|S|$ small at some frequencies necessarily makes it large at others — the **"waterbed effect"**. The total area under $\ln|S|$ is **fixed** by the RHP poles. You cannot escape this fundamental limitation; pushing sensitivity down in one frequency range forces it up elsewhere.

### 16.8.2 Right-Half-Plane Zeros

RHP zeros limit achievable bandwidth.

### 16.8.3 Time Delay

Delays limit achievable phase margin and bandwidth.

---

## 16.9 H-infinity Control

### 16.9.1 Basic Concept

**H∞ control** minimizes the infinity norm:
$$\|T_{zw}\|_\infty = \max_\omega \bar{\sigma}(T_{zw}(j\omega))$$

where $T_{zw}$ is the closed-loop transfer function from exogenous inputs $w$ to regulated outputs $z$.

### 16.9.2 Generalized Plant Formulation

The $H_\infty$ framework uses a **generalized plant** $P(s)$ that encapsulates the plant, weighting functions, and signal interconnections:

$$\begin{bmatrix} z \\ y \end{bmatrix} = \begin{bmatrix} P_{11} & P_{12} \\ P_{21} & P_{22} \end{bmatrix} \begin{bmatrix} w \\ u \end{bmatrix}$$

where:
- $w$ = exogenous inputs (references, disturbances, noise)
- $u$ = control inputs
- $z$ = regulated outputs (weighted errors, control effort)
- $y$ = measured outputs (available for feedback)

The **closed-loop transfer function** from $w$ to $z$ under controller $K$ is the **lower LFT**:
$$T_{zw} = F_l(P, K) = P_{11} + P_{12}K(I - P_{22}K)^{-1}P_{21}$$

### 16.9.3 State-Space Formulation

The generalized plant is described in state-space as:
$$P(s) = \begin{bmatrix} A & B_1 & B_2 \\ C_1 & D_{11} & D_{12} \\ C_2 & D_{21} & D_{22} \end{bmatrix}$$

corresponding to the state equations:
$$\dot{x} = Ax + B_1 w + B_2 u$$
$$z = C_1 x + D_{11} w + D_{12} u$$
$$y = C_2 x + D_{21} w + D_{22} u$$

### 16.9.4 Conditions for Solution Existence

The $H_\infty$ sub-optimal controller (achieving $\|T_{zw}\|_\infty < \gamma$) exists if the following conditions hold:

1. $(A, B_2)$ is **stabilizable** and $(C_2, A)$ is **detectable**
2. $D_{12}$ has **full column rank**, and $\begin{bmatrix} A - j\omega I & B_2 \\ C_1 & D_{12} \end{bmatrix}$ has **full column rank** for all $\omega$ (no invariant zeros on the imaginary axis)
3. $D_{21}$ has **full row rank**, and $\begin{bmatrix} A - j\omega I & B_1 \\ C_2 & D_{21} \end{bmatrix}$ has **full row rank** for all $\omega$ (no invariant zeros on the imaginary axis)

Additional standard assumptions (simplifying the solution): $D_{11} = 0$, $D_{22} = 0$.

### 16.9.5 Two-Riccati Solution (Doyle–Glover–Khargonekar–Francis)

The sub-optimal $\gamma$-level $H_\infty$ controller exists if and only if two **algebraic Riccati equations** have stabilizing solutions $X_\infty \geq 0$ and $Y_\infty \geq 0$ satisfying:

$$A^T X_\infty + X_\infty A + C_1^T C_1 + X_\infty (\gamma^{-2} B_1 B_1^T - B_2 B_2^T) X_\infty = 0$$

$$A Y_\infty + Y_\infty A^T + B_1 B_1^T + Y_\infty (\gamma^{-2} C_1^T C_1 - C_2^T C_2) Y_\infty = 0$$

with the **coupling condition:**
$$\rho(X_\infty Y_\infty) < \gamma^2$$

where $\rho(\cdot)$ denotes the spectral radius. The resulting controller has the same order as the generalized plant.

### 16.9.6 Mixed Sensitivity H∞

A common practical formulation: find $K$ to minimize:
$$\left\|\begin{bmatrix} W_P S \\ W_T T \\ W_U KS \end{bmatrix}\right\|_\infty$$

where $W_P, W_T, W_U$ are weighting functions encoding performance, robustness, and control effort requirements respectively.

### 16.9.7 Solution Methods

- **Two-Riccati approach** (DGKF, 1989): Closed-form state-space solution
- **LMI-based methods:** Convex optimization, handles additional constraints
- **Numerical optimization:** γ-iteration (bisection on $\gamma$)
- **Software:** MATLAB Robust Control Toolbox (`hinfsyn`), Python (`python-control`)

---

## 16.10 Example: Robust Analysis

### 16.10.1 System

Motor position control with ±20% parameter uncertainty.

### 16.10.2 Analysis Steps

1. Compute nominal sensitivity functions
2. Define uncertainty weight $W_M(s)$
3. Check robust stability: $\|W_M T\|_\infty < 1$?
4. Evaluate robust performance

See **ch16_robust_control.cpp** for complete implementation.

---

### 📋 Signal Dictionary — Robust Control of an Uncertain Plant

| Signal | Symbol | Meaning | Typical unit |
|--------|--------|---------|--------------|
| Nominal plant | $G_0(s)$ | The model you designed the controller for | — |
| True plant | $G_p(s)$ | What nature actually gave you (unknown exactly) | — |
| Multiplicative uncertainty | $\Delta(s)$, $\|\Delta\|_\infty \leq 1$ | Normalized "how wrong is the model?" — bounded but unknown | — |
| Uncertainty weight | $W_\Delta(s)$ | Frequency-dependent size of uncertainty: $G_p = G_0(1 + W_\Delta \Delta)$ | — |
| Sensitivity function | $S(s) = \frac{1}{1 + L(s)}$ | Closed-loop transfer from $d \to e$ — measures disturbance rejection | — |
| Complementary sensitivity | $T(s) = \frac{L(s)}{1 + L(s)}$ | Closed-loop transfer from $r \to y$ — measures tracking and noise amplification | — |
| Loop transfer function | $L(s) = G(s)C(s)$ | Open-loop gain — the "shape" we design | — |
| Performance weight | $W_P(s)$ | Desired bound on $|S(j\omega)|$ — encodes tracking/rejection specs | — |
| Peak sensitivity | $M_S = \|S\|_\infty$ | Worst-case amplification of disturbance — proxy for robustness | — |
| Peak complementary | $M_T = \|T\|_\infty$ | Worst-case noise amplification | — |
| Gain margin | GM | Factor by which gain can increase before instability | dB |
| Phase margin | PM | Phase rotation tolerable before instability | ° |

> **Key insight:** $S + T = 1$ always. You cannot make both small at the same frequency. This is the **fundamental limitation** of feedback: good tracking (small $S$) at low frequencies forces noise amplification (large $T$) at those frequencies, and vice versa. Robust control is the art of shaping *where* you win and *where* you accept losing.

---

## 16.11 Exercises

**E15.1 (Multiplicative Uncertainty Modeling)**
Consider a plant $G(s) = \frac{1}{s+a}$ where the parameter $a \in [1, 3]$ (nominal $a_0 = 2$).

(a) Express the uncertain plant in multiplicative uncertainty form:
$$G_p(s) = G_0(s)(1 + W(s)\Delta), \quad |\Delta| \leq 1$$
where $G_0(s) = \frac{1}{s+2}$.

(b) Compute $\frac{G_p(j\omega) - G_0(j\omega)}{G_0(j\omega)}$ for $a = 1$ and $a = 3$ at several frequencies.

(c) Find a rational weighting function $W(s)$ that overbounds the relative uncertainty at all frequencies.

---

**E15.2 (Sensitivity and Complementary Sensitivity)**
For a unity feedback system with loop transfer function
$$L(s) = G(s)C(s) = \frac{10}{s+1}$$

(a) Compute the sensitivity $S(s) = \frac{1}{1+L(s)}$ and complementary sensitivity $T(s) = \frac{L(s)}{1+L(s)}$.

(b) Plot $|S(j\omega)|$ and $|T(j\omega)|$ on the same Bode magnitude plot.

(c) Find the crossover frequency where $|S| = |T|$.

(d) Compute the peak sensitivity $M_S = \max_\omega |S(j\omega)|$. Is the design robust?

---

**E15.3 (Fundamental Constraint: S + T = 1)**
(a) Prove algebraically that $S(s) + T(s) = 1$ for any unity feedback system.

(b) Explain the *waterbed effect*: why does making $|S(j\omega)|$ small at low frequencies necessarily make it large near the crossover frequency?

(c) For a plant with an unstable pole at $s = p > 0$, use Bode's integral theorem
$$\int_0^\infty \ln|S(j\omega)|\,d\omega = \pi p$$
to explain why unstable plants are harder to control.

---

**E15.4 (Sensitivity Analysis with cppplot)**
Using cppplot, define the plant $G(s) = \frac{5}{s(s+1)(s+5)}$ and a PI controller $C(s) = 2 + \frac{5}{s}$.

(a) Compute and plot $S(s)$ and $T(s)$ using `cppplot::control::sensitivity()`.

(b) Find $M_S$ and $M_T$ (peak values).

(c) Determine the gain margin and phase margin from the loop transfer function.

(d) Is the system robustly stable for 20% multiplicative uncertainty at all frequencies?

---

**E15.5 (Weighting Function Design)**
(a) What is the typical shape of the performance weight $W_S(s)$ and why? Sketch $|W_S(j\omega)|$.

(b) What is the typical shape of the robustness weight $W_T(s)$ and why? Sketch $|W_T(j\omega)|$.

(c) For a system with desired bandwidth $\omega_b = 10$ rad/s, steady-state error $< 1\%$, and 10% model uncertainty at DC growing to 100% at $\omega = 100$ rad/s, propose specific first-order weighting functions $W_S(s)$ and $W_T(s)$.

---

**E15.6 (Maximum Bandwidth with Uncertainty)**
For the plant $G(s) = \frac{1}{s(s+1)}$ with 20% multiplicative uncertainty at high frequencies ($|W_M(j\omega)| \to 0.2$ as $\omega \to \infty$):

(a) State the robust stability condition in terms of $T(s)$ and $W_M(s)$.

(b) What is the maximum achievable closed-loop bandwidth? (*Hint:* $|T(j\omega)| < 1/|W_M(j\omega)| = 5$ at all frequencies, but $T(j\omega) \to 1$ at low frequencies.)

(c) Design a lead compensator that achieves the maximum bandwidth while satisfying $\|W_M T\|_\infty < 1$.

---

**E15.7 (Loop Shaping Design)**
Design a controller for $G(s) = \frac{1}{s(s+1)(s+10)}$ that simultaneously satisfies:
- $|S(j\omega)| < -20$ dB for $\omega < 0.1$ rad/s (good tracking)
- $|T(j\omega)| < -20$ dB for $\omega > 100$ rad/s (noise rejection and robust stability)

(a) Translate these specifications into constraints on the loop transfer function $L(j\omega) = G(j\omega)C(j\omega)$.

(b) Use loop shaping (lead-lag compensation) to design $C(s)$.

(c) Plot $|S|$, $|T|$, and verify both specifications are met.

(d) Compute $M_S$ and comment on the overall robustness.

---

**E15.8 🔴 (Level 3 — The Waterbed Effect in Practice)**
You design a controller for a plant with bandwidth 10 rad/s, achieving $|S(j\omega)| < 0.1$ for $\omega < 1$ rad/s (good low-frequency disturbance rejection).

(a) The Bode sensitivity integral says $\int_0^\infty \ln|S(j\omega)|\,d\omega = \pi \sum p_k$ (sum over RHP poles). For a stable plant, the integral = 0. What does this mean for $|S|$ at frequencies above 10 rad/s?

(b) Sketch $|S(j\omega)|$ showing the "waterbed" — pushing $S$ down at low frequencies forces it up elsewhere.

(c) A sensor with noise bandwidth 100 rad/s is added. The noise enters through $T = 1 - S$. If $|S|$ peaks at 2.5 near 50 rad/s, what is $|T|$ there? Is the noise amplified or attenuated?

(d) Can you simultaneously achieve $|S| < 0.01$ for $\omega < 1$ and $|T| < 0.01$ for $\omega > 100$? Prove or disprove using $S + T = 1$.

**E15.9 🔴 (Level 3 — Gain and Phase Margin Are Not Enough)**
A system has GM = 12 dB and PM = 50°, which look excellent by classical standards.

(a) Compute $M_S = \|S\|_\infty$ for this system. (Use the geometric relationship between margins and peak sensitivity.)

(b) Now add an *unstructured* multiplicative uncertainty of 30% at frequency $\omega = 5$ rad/s (i.e., $|W_\Delta(5j)| = 0.3$). Check the robust stability condition $|T(j\omega)| < 1/|W_\Delta(j\omega)|$ at $\omega = 5$. Is the system robustly stable?

(c) This shows a system can have good margins but poor robustness. Explain why — what does $M_S$ capture that GM/PM miss?

**E15.10 ⚫ (Level 4 — The Fundamental Trade-off)**

(a) A student says: "If I could make the loop gain $|L(j\omega)|$ infinitely large at all frequencies, $S \to 0$ everywhere, and all problems would be solved." What two physical constraints prevent this?

(b) Another student says: "Since $S + T = 1$, perfect tracking ($S = 0$) requires $T = 1$, which means the system passes sensor noise directly to the output. So feedback is fundamentally flawed." Is this argument correct? What is the student missing?

(c) Write a paragraph explaining why the $S/T$ trade-off is not a deficiency of control theory but a *fundamental law of information* — you cannot reject disturbances without being sensitive to noise in the same frequency band.

---

## 16.12 Summary

| Concept | Key Point |
|---------|-----------|
| Model uncertainty | Parametric, unstructured |
| Sensitivity S | Tracking, disturbance rejection |
| Complementary T | Noise rejection, stability |
| S + T = 1 | Fundamental trade-off |
| Robust stability | $\|W_M T\|_\infty < 1$ |
| $M_S$ | Peak sensitivity, keep < 2 |

---

## References


1. Skogestad, S. & Postlethwaite, I. (2005). *Multivariable Feedback Control*
2. Zhou, K. & Doyle, J.C. (1998). *Essentials of Robust Control*
