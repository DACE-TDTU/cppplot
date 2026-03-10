# Chapter 14: State Observers

---

## 🎯 Learning Objectives and Outcomes

### Chapter Objectives

This chapter addresses the practical limitation that not all states are measurable by introducing state observers (estimators) that reconstruct unmeasured states from available outputs.

### Prerequisites
- Chapter 11: State-Space Analysis (observability)
- Chapter 12: State Feedback Control

---

## Learning Outcomes

After completing this chapter, students will be able to:

| Bloom's Level | Outcome |
|---------------|---------|
| **Remember** | Define state observer and estimation error |
| **Understand** | Explain the concept of observer poles and their selection |
| **Apply** | Design Luenberger observers for given systems |
| **Analyze** | Analyze observer error dynamics |
| **Evaluate** | Evaluate observer performance and trade-offs |
| **Create** | Design observer-based controllers using separation principle |

---

## Why This Chapter Matters

> **The Real Problem:** A $50,000 torque sensor on each joint of a 6-DOF industrial robot arm costs $300,000 — more than the robot itself. Chapter 12 showed that state feedback can stabilize any controllable system, but it requires measuring *all* states. What if some states are too expensive, too noisy, or physically impossible to measure? Can we *estimate* what we cannot afford to *measure*?
>
> This chapter introduces **state observers** — dynamic systems that reconstruct unmeasured states from available outputs. The key insight: if the system is observable (Chapter 11), we can build a virtual copy of the plant that converges to the true state — and use these estimates for feedback as if we had measured them.

---

## 13.1 Motivation

### 13.1.1 The Problem

State feedback requires **all states to be measured**.

In practice:
- Some states are difficult/expensive to measure
- Sensors add noise
- Some states are physically inaccessible

### 13.1.2 The Solution

**State observer:** A dynamical system that estimates states from:
- Input signal u(t)
- Output measurement y(t)
- Plant model (A, B, C, D)

---

## 13.2 Full-Order Luenberger Observer

### 13.2.1 Structure

The observer is a copy of the plant with correction term:

$$\dot{\hat{\mathbf{x}}} = \mathbf{A}\hat{\mathbf{x}} + \mathbf{B}\mathbf{u} + \mathbf{L}(\mathbf{y} - \hat{\mathbf{y}})$$
$$\hat{\mathbf{y}} = \mathbf{C}\hat{\mathbf{x}}$$

Substituting $\hat{\mathbf{y}}$:
$$\dot{\hat{\mathbf{x}}} = (\mathbf{A} - \mathbf{L}\mathbf{C})\hat{\mathbf{x}} + \mathbf{B}\mathbf{u} + \mathbf{L}\mathbf{y}$$

### 13.2.2 Observer Gain Matrix

$\mathbf{L}$ is the **observer gain** (n × p matrix for p outputs)

### 13.2.3 Block Diagram

```
                                    ┌─────────┐
    u(t) ───────────────────────────►│  Plant  │──────┬──► y(t)
                │                   └─────────┘      │
                │                                    │
                │         ┌─────────────────┐        │
                ├────────►│                 │◄───────┤
                │         │    Observer     │        │
                │         │ẋ̂ = Ax̂+Bu+L(y-ŷ)│        │
                │         │    ŷ = Cx̂      │        │
                │         └────────┬────────┘        │
                │                  │                 │
                │                  ▼ x̂(t)            │
                │                                    │
                └────────────────────────────────────┘
```

---

## 13.3 Error Dynamics

### 13.3.1 Estimation Error

Define estimation error:
$$\mathbf{e} = \mathbf{x} - \hat{\mathbf{x}}$$

### 13.3.2 Error Dynamics Derivation

Starting from the observer equation in its original form:
$$\dot{\hat{\mathbf{x}}} = \mathbf{A}\hat{\mathbf{x}} + \mathbf{B}\mathbf{u} + \mathbf{L}(\mathbf{y} - \mathbf{C}\hat{\mathbf{x}})$$

Subtract from the plant equation $\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}$:

$$\dot{\mathbf{e}} = \dot{\mathbf{x}} - \dot{\hat{\mathbf{x}}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u} - \mathbf{A}\hat{\mathbf{x}} - \mathbf{B}\mathbf{u} - \mathbf{L}(\mathbf{C}\mathbf{x} - \mathbf{C}\hat{\mathbf{x}})$$
$$= \mathbf{A}(\mathbf{x} - \hat{\mathbf{x}}) - \mathbf{L}\mathbf{C}(\mathbf{x} - \hat{\mathbf{x}})$$

**Error dynamics:**
$$\boxed{\dot{\mathbf{e}} = (\mathbf{A} - \mathbf{L}\mathbf{C})\mathbf{e}}$$

> **Key insight:** The error dynamics are **autonomous** — independent of $\mathbf{u}(t)$ and $\mathbf{r}(t)$. The error converges to zero at a rate determined solely by the eigenvalues of $(\mathbf{A} - \mathbf{L}\mathbf{C})$.

### 13.3.3 Stability Condition

For error to converge to zero:
$$\text{All eigenvalues of } (\mathbf{A} - \mathbf{L}\mathbf{C}) \text{ must be in LHP}$$

---

## 13.4 Observer Design

### 13.4.1 Observability Requirement

> **Theorem:**
> If (A, C) is observable, then for any desired set of observer poles, there exists L such that eigenvalues of (A - LC) are those desired poles.

### 13.4.2 Duality with Pole Placement

| State Feedback | Observer |
|----------------|----------|
| A - BK | A - LC |
| Controllability | Observability |
| K design | L design |

**Key insight:** Observer design is the **dual problem** of pole placement.

### 13.4.3 Ackermann's Formula for Observers

Using the **duality** between state feedback and observer design, Ackermann's formula for the observer gain $\mathbf{L}$ is:

$$\mathbf{L} = \phi_o(\mathbf{A}) \cdot \mathcal{O}^{-1} \begin{bmatrix} 0 \\ 0 \\ \vdots \\ 0 \\ 1 \end{bmatrix}$$

or equivalently, via the transpose duality (designing $\mathbf{K}^T$ for the dual system $(\mathbf{A}^T, \mathbf{C}^T)$):

$$\mathbf{L}^T = \begin{bmatrix} 0 & 0 & \cdots & 0 & 1 \end{bmatrix} (\mathcal{O}^T)^{-1} \phi_o(\mathbf{A}^T)$$

where:
- $\mathcal{O} = \begin{bmatrix} \mathbf{C} \\ \mathbf{C}\mathbf{A} \\ \vdots \\ \mathbf{C}\mathbf{A}^{n-1} \end{bmatrix}$ is the observability matrix ($n \times n$ for SISO)
- $\phi_o(s) = s^n + \bar{a}_{n-1}s^{n-1} + \cdots + \bar{a}_0$ is the desired observer characteristic polynomial
- $\phi_o(\mathbf{A}) = \mathbf{A}^n + \bar{a}_{n-1}\mathbf{A}^{n-1} + \cdots + \bar{a}_0\mathbf{I}$

> **⚠️ Important:** The two forms above are equivalent only for SISO systems. For MIMO systems, use direct pole placement methods. Also note that Ackermann's formula involves $\mathcal{O}^{-1}$, which becomes ill-conditioned for $n \geq 5$ — prefer numerical `place()` algorithms for higher-order systems.

> **Example (2nd order):** For $\mathbf{A} = \begin{bmatrix} 0 & 1 \\ -2 & -3 \end{bmatrix}$, $\mathbf{C} = \begin{bmatrix} 1 & 0 \end{bmatrix}$, desired observer poles at $s = -10, -10$:
>
> $\phi_o(s) = (s+10)^2 = s^2 + 20s + 100$
>
> $\phi_o(\mathbf{A}) = \mathbf{A}^2 + 20\mathbf{A} + 100\mathbf{I} = \begin{bmatrix} -2 & -3 \\ 6 & 7 \end{bmatrix} + \begin{bmatrix} 0 & 20 \\ -40 & -60 \end{bmatrix} + \begin{bmatrix} 100 & 0 \\ 0 & 100 \end{bmatrix} = \begin{bmatrix} 98 & 17 \\ -34 & 47 \end{bmatrix}$
>
> $\mathcal{O} = \begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}$ → $\mathcal{O}^{-1} = \mathbf{I}$ → $\mathbf{L} = \begin{bmatrix} 98 & 17 \\ -34 & 47 \end{bmatrix}\begin{bmatrix} 0 \\ 1 \end{bmatrix} = \begin{bmatrix} 17 \\ 47 \end{bmatrix}$
>
> **Verification:** $\text{eig}(\mathbf{A} - \mathbf{L}\mathbf{C}) = \text{eig}\begin{bmatrix} -17 & 1 \\ -49 & -3 \end{bmatrix} = \{-10, -10\}$ ✓

### 13.4.4 Pole Selection Guidelines

**Rule of thumb:** Observer poles should be 2-5× faster than controller poles.

| Consideration | Guideline |
|---------------|-----------|
| Fast convergence | Place poles further left |
| Noise sensitivity | Don't place too far left |
| Model uncertainty | Moderate pole locations |
| Typical ratio | Observer poles 2-5× faster than controller |

---

## 13.5 Separation Principle

### 13.5.1 Observer-Based Controller

Complete control law:
$$\mathbf{u} = -\mathbf{K}\hat{\mathbf{x}}$$

Combined system:
$$\begin{bmatrix} \dot{\mathbf{x}} \\ \dot{\mathbf{e}} \end{bmatrix} = \begin{bmatrix} \mathbf{A} - \mathbf{B}\mathbf{K} & \mathbf{B}\mathbf{K} \\ \mathbf{0} & \mathbf{A} - \mathbf{L}\mathbf{C} \end{bmatrix} \begin{bmatrix} \mathbf{x} \\ \mathbf{e} \end{bmatrix}$$

### 13.5.2 The Separation Principle

> **Theorem (Separation Principle):**
> The closed-loop eigenvalues of the observer-based controller are:
> - Eigenvalues of (A - BK) — controller poles
> - Eigenvalues of (A - LC) — observer poles
> 
> **Implication:** Controller and observer can be designed **independently**.

### 13.5.3 Design Procedure

1. **Design state feedback K** assuming all states available
2. **Design observer L** for desired error dynamics
3. **Combine:** Use $u = -K\hat{x}$

---

## 13.6 Reduced-Order Observer

### 13.6.1 Motivation

If p states are directly measured, we only need to estimate (n - p) states.

### 13.6.2 Structure

Partition states:
$$\mathbf{x} = \begin{bmatrix} \mathbf{x}_a \\ \mathbf{x}_b \end{bmatrix}$$

where $\mathbf{x}_a = \mathbf{y}$ (measured states)

### 13.6.3 Reduced-Order Observer Derivation

Partition the state-space matrices conformally with $\mathbf{x} = [\mathbf{x}_a; \mathbf{x}_b]$:

$$\begin{bmatrix} \dot{\mathbf{x}}_a \\ \dot{\mathbf{x}}_b \end{bmatrix} = \begin{bmatrix} \mathbf{A}_{aa} & \mathbf{A}_{ab} \\ \mathbf{A}_{ba} & \mathbf{A}_{bb} \end{bmatrix} \begin{bmatrix} \mathbf{x}_a \\ \mathbf{x}_b \end{bmatrix} + \begin{bmatrix} \mathbf{B}_a \\ \mathbf{B}_b \end{bmatrix} \mathbf{u}$$

where $\mathbf{x}_a = \mathbf{y}$ (measured directly) and $\mathbf{x}_b$ must be estimated.

From the second row:
$$\dot{\mathbf{x}}_b = \mathbf{A}_{ba}\mathbf{y} + \mathbf{A}_{bb}\mathbf{x}_b + \mathbf{B}_b\mathbf{u}$$

This looks like a system with "output" obtained from the first row:
$$\dot{\mathbf{y}} = \mathbf{A}_{aa}\mathbf{y} + \mathbf{A}_{ab}\mathbf{x}_b + \mathbf{B}_a\mathbf{u}$$

rearranged as a "measurement" of $\mathbf{x}_b$:
$$\underbrace{\dot{\mathbf{y}} - \mathbf{A}_{aa}\mathbf{y} - \mathbf{B}_a\mathbf{u}}_{\text{known (but requires } \dot{\mathbf{y}}\text{)}} = \mathbf{A}_{ab}\mathbf{x}_b$$

A full observer for $\mathbf{x}_b$ would be:
$$\dot{\hat{\mathbf{x}}}_b = \mathbf{A}_{bb}\hat{\mathbf{x}}_b + \mathbf{A}_{ba}\mathbf{y} + \mathbf{B}_b\mathbf{u} + \mathbf{L}_r(\dot{\mathbf{y}} - \mathbf{A}_{ab}\hat{\mathbf{x}}_b - \mathbf{A}_{aa}\mathbf{y} - \mathbf{B}_a\mathbf{u})$$

**Avoiding differentiation of y:** Define the auxiliary variable:
$$\mathbf{z} = \hat{\mathbf{x}}_b - \mathbf{L}_r\mathbf{y}$$

Then the observer equations become:
$$\boxed{\dot{\mathbf{z}} = \mathbf{F}\mathbf{z} + \mathbf{G}\mathbf{y} + \mathbf{H}\mathbf{u}}$$
$$\boxed{\hat{\mathbf{x}}_b = \mathbf{z} + \mathbf{L}_r\mathbf{y}}$$

where:
- $\mathbf{F} = \mathbf{A}_{bb} - \mathbf{L}_r\mathbf{A}_{ab}$ — determines observer error dynamics (eigenvalues of $\mathbf{F}$ must be stable)
- $\mathbf{G} = \mathbf{A}_{ba} - \mathbf{L}_r\mathbf{A}_{aa} + \mathbf{F}\mathbf{L}_r$
- $\mathbf{H} = \mathbf{B}_b - \mathbf{L}_r\mathbf{B}_a$

The estimation error $\mathbf{e}_b = \mathbf{x}_b - \hat{\mathbf{x}}_b$ satisfies:
$$\dot{\mathbf{e}}_b = (\mathbf{A}_{bb} - \mathbf{L}_r\mathbf{A}_{ab})\mathbf{e}_b = \mathbf{F}\mathbf{e}_b$$

so the observer gain $\mathbf{L}_r$ is chosen to place the eigenvalues of $\mathbf{F}$ at desired locations.

### 13.6.4 Advantages/Disadvantages

| Aspect | Full-Order | Reduced-Order |
|--------|------------|---------------|
| Dimension | n states | n - p states |
| Complexity | Simpler design | More complex |
| Noise | Better filtering | More sensitive |
| Initial error | Larger | Smaller |

---

## 13.7 Observer for Tracking Systems

### 13.7.1 With Integral Action

When combining observer with integral control:

1. Augment plant with integrator
2. Design observer for augmented system
3. Or: Integrate output error directly (simpler)

---

## 13.8 Example: DC Motor Observer

### 13.8.1 System Model

**States:**
- $x_1 = \theta$ (angular position)
- $x_2 = \omega$ (angular velocity)

**Measurable output:** Only position θ

**Signal Dictionary — DC Motor Observer**

| Signal | Symbol | Unit | Physical Meaning | Sensor/Actuator |
|--------|--------|------|------------------|-----------------|
| Angular position (state, measured) | $x_1 = \theta$ | rad | Shaft angle — this is what the encoder measures | Optical encoder |
| Angular velocity (state, estimated) | $x_2 = \omega$ | rad/s | Shaft speed — **not directly measured**, must be estimated by observer | (No sensor — observer reconstructs this) |
| Armature voltage (input) | $u$ | V | Voltage applied to motor — the known input | Power amplifier |
| Motor torque constant | $K_t$ | N·m/A | Torque per ampere — appears in **B** matrix as $K_t/J$ | (Motor parameter) |
| Friction coefficient | $b$ | N·m·s/rad | Viscous friction — appears in **A** matrix as $-b/J$ | (Motor parameter) |
| Observer output | $\hat{\omega}$ | rad/s | Estimated velocity from observer — replaces the \$300K sensor | (Software computation) |
| Observer error | $e = \theta - \hat{\theta}$ | rad | Difference between measured and estimated position — drives the correction term $\mathbf{L}e$ | (Computed) |

> **The engineering question:** We can measure $\theta$ (cheap encoder) but not $\omega$ (expensive tachometer). The observer uses the known input $u$ and the measured output $\theta$ to reconstruct $\omega$. The gain **L** determines how aggressively the observer corrects its estimate.

### 13.8.2 State-Space Model

$$\mathbf{A} = \begin{bmatrix} 0 & 1 \\ 0 & -\frac{b}{J} \end{bmatrix}, \quad \mathbf{B} = \begin{bmatrix} 0 \\ \frac{K_t}{J} \end{bmatrix}$$

$$\mathbf{C} = \begin{bmatrix} 1 & 0 \end{bmatrix}$$

### 13.8.3 Observer Design

1. Check observability
2. Choose observer poles (e.g., 3× faster than controller)
3. Calculate L using Ackermann's formula
4. Implement observer

See **ch13_observer.cpp** for complete implementation.

---

## 13.9 Practical Considerations

### 13.9.1 Initialization

Observer initial condition $\hat{\mathbf{x}}(0)$:
- If known, set $\hat{\mathbf{x}}(0) = \mathbf{x}(0)$
- If unknown, use zero or best guess
- Error will converge if observer is stable

### 13.9.2 Sensor Noise

- Fast observer poles → high gain → amplifies noise
- Trade-off: Speed vs. noise rejection
- The Kalman filter resolves this trade-off *optimally* — see §13.9A for the full comparison

### 13.9.3 Model Mismatch

- Observer assumes perfect model
- Real systems have uncertainty
- Robust observer design may be needed

---

## ⚠️ 12.9A Observer vs. Estimator — A Critical Distinction

> **A common confusion:** Students often see "observer" and "estimator" used interchangeably in textbooks. They are closely related, but the distinction reveals a **fundamental philosophical divide** in control engineering.

### Same Structure, Different Philosophy

Both the Luenberger observer (this chapter) and the Kalman filter (Chapter 15) share *exactly the same equation*:

$$\dot{\hat{\mathbf{x}}} = A\hat{\mathbf{x}} + Bu + L(y - C\hat{\mathbf{x}})$$

Both use the error $e = y - C\hat{x}$ to correct the estimate. Both require the system to be **observable**. The difference is entirely in **how $L$ is chosen** — and this difference reflects two fundamentally different assumptions about the world.

### The Two Worldviews

```
┌─────────────────────────────────────────────────────────────────────────┐
│                 OBSERVER vs. ESTIMATOR: TWO WORLDVIEWS                  │
├─────────────────────────────────────────┬───────────────────────────────┤
│         OBSERVER (Luenberger)           │      ESTIMATOR (Kalman)       │
├─────────────────────────────────────────┼───────────────────────────────┤
│                                         │                               │
│  Worldview: DETERMINISTIC               │  Worldview: STOCHASTIC        │
│  "The model is exact.                   │  "The model has noise.        │
│   I just can't see all the states."     │   The sensor has noise too."  │
│                                         │                               │
│  Plant model:                           │  Plant model:                 │
│   ẋ = Ax + Bu                           │   ẋ = Ax + Bu + w(t)          │
│   y = Cx                                │   y = Cx + v(t)               │
│                                         │                               │
│  w(t) = 0 (no process noise)            │  w ~ N(0, Q) process noise    │
│  v(t) = 0 (no sensor noise)             │  v ~ N(0, R) sensor noise     │
│                                         │                               │
│  Choose L by: POLE PLACEMENT            │  Choose L by: OPTIMIZATION    │
│  "I want error to decay in 0.1s"        │  "Minimize E[eᵀe] given Q, R" │
│                                         │                               │
│  Error dynamics:                        │  Error dynamics:              │
│   ė = (A − LC)e → 0 exactly            │   E[||e||²] → P_ss > 0       │
│   (converges to zero)                   │   (converges to minimum       │
│                                         │    variance, NOT zero)        │
│                                         │                               │
│  Gain L: you pick eigenvalues           │  Gain L: Riccati equation     │
│  of (A − LC) — heuristic               │  solves for L — optimal       │
│                                         │                               │
│  When to use:                           │  When to use:                 │
│  • Model is accurate                    │  • Significant noise          │
│  • Noise is small                       │  • Noise statistics known     │
│  • Simple implementation needed         │  • Optimal performance needed │
│                                         │                               │
└─────────────────────────────────────────┴───────────────────────────────┘
```

### The Mechanism: Why Kalman Gain Is Different

The Luenberger observer designer asks: *"How fast should the error decay?"* and picks poles accordingly. Faster poles → larger $L$ → faster convergence, but also more noise amplification (as discussed in §13.9.2).

The Kalman filter solves this trade-off *optimally*. Given:
- $Q$ = process noise covariance ("how much I distrust my model")
- $R$ = measurement noise covariance ("how much I distrust my sensor")

The Kalman gain $L_K$ is computed from the algebraic Riccati equation:

$$AP + PA^T - PC^TR^{-1}CP + Q = 0 \quad \Rightarrow \quad L_K = PC^TR^{-1}$$

| If... | Then Kalman gain... | Physical meaning |
|-------|---------------------|------------------|
| $Q \gg R$ (bad model, good sensor) | $L_K$ is large | "Trust the measurement more — correct aggressively" |
| $Q \ll R$ (good model, bad sensor) | $L_K$ is small | "Trust the model more — correct gently" |
| $Q = R$ | Balanced | "Split the difference" |

**This is the exact same speed-vs-noise trade-off** from §13.9.2, but resolved by mathematics instead of engineering judgment.

### Relationship Between the Two

```
┌────────────────────────────────────────────────────────────┐
│                                                            │
│  Every Kalman filter IS an observer                        │
│  (same equation, same structure, same block diagram)       │
│                                                            │
│  Every Luenberger observer IS an estimator                 │
│  (it estimates states — just not optimally)                │
│                                                            │
│  The Kalman filter is the UNIQUE observer that minimizes   │
│  E[||x − x̂||²] when noise is Gaussian                    │
│                                                            │
│  Luenberger + perfect model + no noise = exact recovery    │
│  Kalman + noise + imperfect model = minimum variance       │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

### 🛑 Stop and Think

> You design a Luenberger observer with poles 10× faster than the plant. The simulation works perfectly. On the real hardware, the estimated states are unusably noisy.
>
> 1. *Why* does this happen? (What assumption of §13.2 is violated in reality?)
> 2. Would making the observer *slower* help? Why?
> 3. A Kalman filter would automatically find the right balance — but what information does it need that the Luenberger approach does not? (Hint: $Q$ and $R$)
> 4. If you don't know $Q$ and $R$, is the Kalman filter still better than Luenberger? Why or why not?

> **→ Forward connection:** Chapter 15 derives the Kalman filter in full and combines it with LQR to form **LQG** (Linear Quadratic Gaussian) — the stochastic counterpart of the observer-based controller from §13.5. The separation principle (§13.5.2) holds for LQG as well: the Kalman filter and LQR can be designed independently.

---

## 13.10 Real-World Application: Sensorless Motor Control

> **Practical Integration:** Sensorless control of electric motors is one of the most successful industrial applications of state observers, eliminating expensive position/speed sensors while maintaining performance.

### 13.10.1 System Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│             SENSORLESS PMSM DRIVE (Variable Frequency Drive)            │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐                 │
│  │    Grid     │───►│  Rectifier  │───►│  Inverter   │───► PMSM        │
│  │  3-phase    │    │  (Diode/    │    │  (IGBT/SiC) │     Motor       │
│  │  400V AC    │    │   Active)   │    │  3-phase    │                 │
│  └─────────────┘    └─────────────┘    └──────┬──────┘                 │
│                                               │                         │
│                                          PWM Signals                    │
│                                               │                         │
│                     ┌─────────────────────────┴─────────────────────┐  │
│                     │              DSP Controller                    │  │
│                     │  ┌─────────────────────────────────────────┐  │  │
│                     │  │  Field-Oriented Control (FOC)           │  │  │
│                     │  │                                         │  │  │
│                     │  │  ┌─────────┐   ┌─────────┐  ┌────────┐ │  │  │
│                     │  │  │ Current │──►│ Speed   │──►│Position│ │  │  │
│                     │  │  │ Control │   │ Control │  │Control │ │  │  │
│                     │  │  │  (PI)   │   │  (PI)   │  │  (P)   │ │  │  │
│                     │  │  └────▲────┘   └────▲────┘  └────▲───┘ │  │  │
│                     │  │       │             │            │      │  │  │
│                     │  │  ┌────┴─────────────┴────────────┴───┐ │  │  │
│                     │  │  │        STATE OBSERVER             │ │  │  │
│                     │  │  │  • Estimates: θ (angle), ω (speed)│ │  │  │
│                     │  │  │  • Inputs: i_a, i_b, v_a, v_b     │ │  │  │
│                     │  │  │  • Methods: SMO, EKF, MRAS, PLL   │ │  │  │
│                     │  │  └──────────────▲────────────────────┘ │  │  │
│                     │  │                 │                       │  │  │
│                     │  └─────────────────┼───────────────────────┘  │  │
│                     │                    │                          │  │
│                     └────────────────────┼──────────────────────────┘  │
│                                          │                              │
│                    ┌─────────────────────┴─────────────────────┐       │
│                    │     Current Sensors (ia, ib)              │       │
│                    │     (Hall effect or shunt resistors)      │       │
│                    └───────────────────────────────────────────┘       │
│                                                                         │
│   NO ENCODER/RESOLVER NEEDED - Cost savings $50-200 per drive          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 13.10.2 PMSM Model for Observer Design

**State Variables:** $\mathbf{x} = [\omega, \theta, i_d, i_q]^T$

**Extended Back-EMF Model (αβ frame):**

$$\begin{bmatrix} \dot{i}_\alpha \\ \dot{i}_\beta \end{bmatrix} = 
\begin{bmatrix} -R/L & 0 \\ 0 & -R/L \end{bmatrix}
\begin{bmatrix} i_\alpha \\ i_\beta \end{bmatrix} +
\frac{1}{L}\begin{bmatrix} v_\alpha \\ v_\beta \end{bmatrix} +
\frac{1}{L}\begin{bmatrix} e_\alpha \\ e_\beta \end{bmatrix}$$

Where back-EMF: $e_\alpha = -\lambda_m \omega \sin\theta$, $e_\beta = \lambda_m \omega \cos\theta$

**Observable:** The back-EMF contains position information!

### 13.10.3 Sliding Mode Observer (SMO)

**Structure:**

$$\dot{\hat{i}}_\alpha = -\frac{R}{L}\hat{i}_\alpha + \frac{1}{L}v_\alpha + \frac{1}{L}z_\alpha$$
$$\dot{\hat{i}}_\beta = -\frac{R}{L}\hat{i}_\beta + \frac{1}{L}v_\beta + \frac{1}{L}z_\beta$$

**Switching signals:**
$$z_\alpha = k \cdot \text{sign}(\hat{i}_\alpha - i_\alpha)$$
$$z_\beta = k \cdot \text{sign}(\hat{i}_\beta - i_\beta)$$

**Angle extraction:**
$$\hat{\theta} = -\arctan\left(\frac{\bar{z}_\alpha}{\bar{z}_\beta}\right)$$

where $\bar{z}$ is low-pass filtered switching signal.

### 13.10.4 Multi-Domain Integration

| Domain | Component | Observer Role |
|--------|-----------|---------------|
| **Electrical** | Motor L, R, λm | Model parameters (may need adaptation) |
| **Magnetic** | Back-EMF | Carries position information |
| **Mechanical** | Inertia, friction | Speed dynamics |
| **Electronic** | ADC, current sensors | Measurement inputs (noise consideration) |
| **Software** | DSP algorithm | Observer implementation @ 10-20 kHz |
| **Thermal** | Winding temperature | R changes ±50% (parameter adaptation) |

### 13.10.5 Observer Types Comparison

| Observer Type | Complexity | Low-Speed Performance | Robustness |
|---------------|------------|----------------------|------------|
| **Open-Loop Estimator** | Low | Poor (no correction) | Low |
| **Sliding Mode (SMO)** | Medium | Moderate | High |
| **Extended Kalman (EKF)** | High | Good | Very High |
| **Model Reference (MRAS)** | Medium | Moderate | Medium |
| **High-Frequency Injection** | Medium | Excellent | Medium |

### 13.10.6 Implementation Challenges

> **Practical Considerations:**
> 
> 1. **Zero/Low Speed:** Back-EMF vanishes → inject high-frequency signal
> 2. **Parameter Sensitivity:** R varies with temperature, L varies with saturation
> 3. **Inverter Nonlinearity:** Dead-time causes voltage distortion
> 4. **Noise:** Current sensor noise corrupted by PWM switching
> 5. **Initial Position:** Must detect initial rotor angle before starting

**Solution Architecture:**

```
┌─────────────────────────────────────────────────────────────────┐
│                  HYBRID SENSORLESS STRATEGY                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   0 ────────5%────────20%────────100%  Speed (% rated)         │
│   │         │          │          │                             │
│   ▼         ▼          ▼          ▼                             │
│  ┌───┐   ┌─────┐    ┌─────┐    ┌─────┐                        │
│  │HFI│──►│Blend│───►│Back │───►│Back │                        │
│  │   │   │     │    │EMF  │    │EMF  │                        │
│  │   │   │     │    │SMO  │    │Only │                        │
│  └───┘   └─────┘    └─────┘    └─────┘                        │
│                                                                 │
│  HFI: High-Frequency Injection (at standstill/low speed)       │
│  SMO: Sliding Mode Observer (medium to high speed)             │
└─────────────────────────────────────────────────────────────────┘
```

---

## 13.11 Additional Electrical and Telecommunications Observer Examples

### 13.11.1 Grid Voltage Observer for Single-Phase Systems

**Problem:** Estimate grid voltage magnitude and phase without direct voltage sensor.

**Why Needed:**
- Reduce sensor cost
- Improve reliability (no voltage sensor failure)
- Enable grid-forming capability

**Observer Model:**

Model grid voltage as rotating phasor:
$$v_g(t) = V_m \sin(\omega t + \phi)$$

State-space in αβ coordinates:
$$\mathbf{x} = \begin{bmatrix} v_\alpha \\ v_\beta \end{bmatrix}, \quad
\dot{\mathbf{x}} = \begin{bmatrix} 0 & \omega \\ -\omega & 0 \end{bmatrix}\mathbf{x}$$

**Luenberger Observer:**
$$\dot{\hat{\mathbf{x}}} = \mathbf{A}\hat{\mathbf{x}} + \mathbf{L}(v_{meas} - \hat{v}_\alpha)$$

Where $v_{meas}$ can be derived from current measurement and known inverter voltage.

```cpp
#include <cppplot/cppplot.hpp>
#include <cmath>

class GridVoltageObserver {
private:
    double v_alpha_hat, v_beta_hat;
    double w0;  // Grid frequency (rad/s)
    double L1, L2;  // Observer gains
    double Ts;
    
public:
    GridVoltageObserver(double grid_freq, double bandwidth, double sample_time) {
        w0 = 2 * M_PI * grid_freq;
        Ts = sample_time;
        
        // Observer gains for desired bandwidth
        // Poles at -bandwidth ± j*w0
        double wb = 2 * M_PI * bandwidth;
        L1 = 2 * wb;
        L2 = wb * wb / w0;
        
        v_alpha_hat = v_beta_hat = 0;
    }
    
    void update(double v_alpha_meas) {
        // Error
        double e = v_alpha_meas - v_alpha_hat;
        
        // Observer dynamics (Euler integration)
        double dv_alpha = w0 * v_beta_hat + L1 * e;
        double dv_beta = -w0 * v_alpha_hat + L2 * e;
        
        v_alpha_hat += dv_alpha * Ts;
        v_beta_hat += dv_beta * Ts;
    }
    
    double getMagnitude() const {
        return std::sqrt(v_alpha_hat*v_alpha_hat + v_beta_hat*v_beta_hat);
    }
    
    double getPhase() const {
        return std::atan2(v_beta_hat, v_alpha_hat);
    }
    
    double getFrequency() const {
        return w0 / (2 * M_PI);
    }
};
```

> **🔍 Stop and Think — Mechanism Questions**
>
> Before moving on:
> 1. The observer uses the *measured* position $\theta$ and the *estimated* position $\hat{\theta}$ to compute an error $e = \theta - \hat{\theta}$. This error then corrects the velocity estimate. *Why does measuring position help estimate velocity?* (Hint: if the estimated position drifts from the real position, what does that reveal about the velocity estimate?)
> 2. The observer gain $\mathbf{L}$ is analogous to the controller gain $\mathbf{K}$ but in "dual" space. Large $\mathbf{L}$ means aggressive correction. Give one benefit and one cost of making $\mathbf{L}$ large.
> 3. If the motor model is slightly wrong (e.g., friction $b$ is 20% higher than assumed), the observer will have a steady-state estimation error. *Why?* Which parameter in the observer dynamics causes this?


**Channel Model:**

For a frequency-selective fading channel:
$$y[n] = \sum_{k=0}^{L-1} h_k[n] \cdot x[n-k] + w[n]$$

Model channel taps as random walk:
$$h_k[n+1] = h_k[n] + v_k[n]$$

Where $v_k$ is process noise representing channel variation.

**State-Space Formulation:**

$$\mathbf{h}[n+1] = \mathbf{I} \cdot \mathbf{h}[n] + \mathbf{v}[n]$$
$$y[n] = \mathbf{x}^T[n] \cdot \mathbf{h}[n] + w[n]$$

Where $\mathbf{x}[n] = [x[n], x[n-1], ..., x[n-L+1]]^T$.

**Kalman Filter Implementation:**

```cpp
#include <cppplot/cppplot.hpp>
#include <cppplot/matrix.hpp>
#include <cmath>
#include <complex>
#include <vector>
#include <iostream>

using namespace cppplot;

class ChannelKalmanFilter {
private:
    Matrix h_hat;    // Channel estimate (Lx1 column vector)
    Matrix P;        // Error covariance (LxL matrix)
    double q_var;    // Process noise variance (scalar)
    double R;        // Measurement noise variance
    int L;           // Number of taps
    
public:
    ChannelKalmanFilter(int num_taps, double process_var, double meas_var)
        : L(num_taps), q_var(process_var), R(meas_var) {
        h_hat = Matrix::zeros(L, 1);
        P = Matrix::eye(L) * 0.1;
    }
    
    std::complex<double> update(const std::vector<std::complex<double>>& x,
                                 std::complex<double> y) {
        // Prediction: P_pred = P + Q*I (random walk model)
        P = P + Matrix::eye(L) * q_var;
        
        // Extract real part of input into column vector
        Matrix x_real(L, 1);
        for (int i = 0; i < L; ++i) x_real(i, 0) = x[i].real();
        
        // Kalman gain: K = P * x_real / (x_real^T * P * x_real + R)
        Matrix Px = P * x_real;
        double S = (x_real.T() * Px)(0,0) + R;
        Matrix Kk = Px * (1.0 / S);
        
        // Innovation: y_hat = x_real^T * h_hat
        double y_hat = (x_real.T() * h_hat)(0,0);
        double innovation = y.real() - y_hat;
        
        // Update estimate: h_hat += K * innovation
        h_hat = h_hat + Kk * innovation;
        
        // Update covariance: P = (I - K * x_real^T) * P
        P = (Matrix::eye(L) - Kk * x_real.T()) * P;
        
        // Return equalized output
        double eq = (x_real.T() * h_hat)(0,0);
        return y - std::complex<double>(eq, 0);
    }
    
    Matrix getChannelEstimate() const {
        return h_hat;
    }
};
```

### 13.11.3 DC-Link Voltage Observer for Motor Drives

**Problem:** Estimate DC-link voltage without dedicated sensor for cost reduction.

**Why Important:**
- DC voltage affects motor control accuracy
- Battery voltage varies during operation
- Adding sensor increases cost and failure points

**Observer Design:**

Using motor current dynamics:
$$\frac{di_q}{dt} = \frac{1}{L_q}(v_q - R_s i_q - \omega_e \lambda_d)$$

The applied voltage $v_q = V_{dc} \cdot d_q$ where $d_q$ is duty cycle.

**Extended State Observer:**

Augment system with $V_{dc}$ as slowly-varying state:
$$\frac{dV_{dc}}{dt} \approx 0$$

$$\mathbf{x} = \begin{bmatrix} i_q \\ V_{dc} \end{bmatrix}$$

$$\dot{\mathbf{x}} = \begin{bmatrix} -R_s/L_q & d_q/L_q \\ 0 & 0 \end{bmatrix}\mathbf{x} + \begin{bmatrix} -\omega_e\lambda_d/L_q \\ 0 \end{bmatrix}$$

Observer:
$$\dot{\hat{\mathbf{x}}} = \mathbf{A}\hat{\mathbf{x}} + \mathbf{f} + \mathbf{L}(i_{q,meas} - \hat{i}_q)$$

### 13.11.4 PLL as Observer Interpretation

**Insight:** A Phase-Locked Loop is essentially a state observer for phase and frequency!

**Standard PLL Structure:**

```
┌────────────────────────────────────────────────────────────────────────┐
│           PLL AS STATE OBSERVER                                        │
├────────────────────────────────────────────────────────────────────────┤
│                                                                        │
│   Input phase θ_in                                                    │
│        │                                                               │
│        ▼     ┌───────────────┐                                        │
│       (─)───►│ Phase Detector│───► Error = θ_in - θ_hat              │
│        ▲     └───────────────┘          │                             │
│        │                                ▼                              │
│        │                         ┌─────────────┐                      │
│        │                         │ Loop Filter │                      │
│        │                         │  (Observer  │                      │
│        │                         │    Gains)   │                      │
│        │                         └──────┬──────┘                      │
│        │                                │                              │
│        │     ┌───────────────┐          ▼                             │
│        └─────┤     VCO       │◄─── ω_hat (frequency estimate)        │
│              │ ∫ω_hat dt     │                                        │
│              └───────────────┘                                        │
│                    │                                                   │
│                    └──────────────────────► θ_hat (phase estimate)   │
│                                                                        │
│   STATE-SPACE VIEW:                                                   │
│   x = [θ, ω]ᵀ,  A = [0 1; 0 0],  C = [1 0]                          │
│   This is a Luenberger observer for phase and frequency!             │
│                                                                        │
└────────────────────────────────────────────────────────────────────────┘
```

**Mathematical Equivalence:**

PLL with PI loop filter:
$$\dot{\hat{\omega}} = K_i \cdot e_\phi$$
$$\dot{\hat{\theta}} = \hat{\omega} + K_p \cdot e_\phi$$

Where $e_\phi = \theta_{in} - \hat{\theta}$.

This is exactly Luenberger observer form with:
$$\mathbf{L} = \begin{bmatrix} K_p \\ K_i \end{bmatrix}$$

**Design Implication:**
- PLL bandwidth = observer bandwidth
- Damping ratio = observer damping
- All observer design rules apply to PLL!

### 13.11.5 Summary: Observers in EE and Telecom

| Application | Observed States | Measurement | Key Challenge |
|-------------|-----------------|-------------|---------------|
| **Sensorless motor** | Position, speed | Currents | Low-speed operation |
| **Grid voltage** | Magnitude, phase | Current + known V | Harmonic rejection |
| **Channel estimation** | Tap coefficients | Received signal | Time-varying channel |
| **DC-link voltage** | Bus voltage | Motor currents | Nonlinear dependence |
| **PLL** | Phase, frequency | Input phase | Noise vs. tracking speed |

**Key Insights:**

1. **Observer = Sensor Replacement:** Observers enable cost reduction and reliability improvement by eliminating sensors.

2. **PLL ≡ Observer:** Understanding PLL as observer clarifies design tradeoffs (bandwidth vs. noise).

3. **Kalman for Time-Varying:** When parameters change (fading channel, varying load), Kalman filter automatically adapts estimation.

4. **Separation Still Holds:** In power electronics/telecom, separation principle allows independent design of current controller and state observer.

---

## 13.12 Exercises

**E12.1 (Full-Order Observer Design)**
For the system
$$\mathbf{A} = \begin{bmatrix} 0 & 1 \\ -2 & -3 \end{bmatrix}, \quad \mathbf{C} = \begin{bmatrix} 1 & 0 \end{bmatrix}$$
design a full-order Luenberger observer with observer poles at $s = -10$ and $s = -12$.

(a) Verify that the system is observable.

(b) Compute the observer gain $\mathbf{L}$ using Ackermann's formula.

(c) Verify that $\text{eig}(\mathbf{A} - \mathbf{L}\mathbf{C}) = \{-10, -12\}$.

---

**E12.2 (Observer Simulation with cppplot)**
Implement the observer from E12.1 using `cppplot::control::observer_gain()`. Simulate the system with:
- True initial state $\mathbf{x}(0) = [1, 0]^T$
- Observer initial estimate $\hat{\mathbf{x}}(0) = [0, 0]^T$
- Input $u(t) = 0$ (autonomous system)

Plot the true states, estimated states, and estimation error $\mathbf{e}(t) = \mathbf{x}(t) - \hat{\mathbf{x}}(t)$ over time. How quickly does the error converge to zero?

---

**E12.3 (Observer-Based Controller — Separation Principle)**
Combine the state feedback from E11.1 ($\mathbf{K}$ for poles at $-5 \pm j5$) with the observer from E12.1 ($\mathbf{L}$ for poles at $-10, -12$) using the control law $u = -\mathbf{K}\hat{\mathbf{x}}$.

With $\mathbf{B} = [0; 1]$:

(a) Write the combined closed-loop system equations.

(b) Simulate the closed-loop response from $\mathbf{x}(0) = [1, 0]^T$, $\hat{\mathbf{x}}(0) = [0, 0]^T$.

(c) How does the transient differ from the case where all states are measured directly?

---

**E12.4 (Verification of Separation Principle)**
For the combined system in E12.3, compute the $4 \times 4$ closed-loop system matrix:
$$\begin{bmatrix} \mathbf{A} - \mathbf{B}\mathbf{K} & \mathbf{B}\mathbf{K} \\ \mathbf{0} & \mathbf{A} - \mathbf{L}\mathbf{C} \end{bmatrix}$$

Verify that the eigenvalues are the union of $\{-5 \pm j5\}$ (controller poles) and $\{-10, -12\}$ (observer poles). Explain why the block-triangular structure guarantees this.

---

**E12.5 (Reduced-Order Observer)**
For the system
$$\mathbf{A} = \begin{bmatrix} 0 & 1 \\ -6 & -5 \end{bmatrix}, \quad \mathbf{C} = \begin{bmatrix} 1 & 0 \end{bmatrix}$$
where $x_1 = y$ is directly measured:

(a) Design a reduced-order observer to estimate $x_2$ only, with observer pole at $s = -15$.

(b) Write the reduced-order observer equations.

(c) Simulate and compare the estimation of $x_2$ with the full-order observer approach.

---

**E12.6 (Full-Order vs. Reduced-Order Comparison)**
For the system in E12.5, design both a full-order observer (poles at $-15, -20$) and a reduced-order observer (pole at $-15$).

(a) Compare the observer dimensions and computational cost.

(b) Simulate both observers with the same initial estimation error and compare convergence.

(c) Add measurement noise $v(t) \sim \mathcal{N}(0, 0.01)$ and compare which observer is more sensitive to noise.

---

**E12.7 (Challenge: Observability and Observer Convergence)**
Show that observability of $(\mathbf{A}, \mathbf{C})$ is a **necessary** condition for asymptotic convergence of the observer estimation error to zero for arbitrary initial errors.

*Hint:* Consider the error dynamics $\dot{\mathbf{e}} = (\mathbf{A} - \mathbf{L}\mathbf{C})\mathbf{e}$. If $(\mathbf{A}, \mathbf{C})$ is not observable, show that there exists a direction in state space where the error cannot be corrected regardless of the choice of $\mathbf{L}$. (Use the PBH observability test.)

### Problem Identification Exercises (Level 3-4)

**E12.8 — What Is the Real Problem?**
An engineer designs a Luenberger observer for a DC motor, placing observer poles at 5× the controller poles. In simulation, the observer converges in 0.2s. On the real motor, the estimated velocity $\hat{\omega}$ is noisy and unusable.

(a) Explain the observer trade-off: faster poles mean faster convergence but also more noise amplification. Where does this trade-off appear mathematically? (Hint: observer gain $\mathbf{L}$.)
(b) The student’s instinct: "Make the observer even faster to track the real state." Why is this exactly the wrong direction?
(c) The *real* problem is not observer design — it is defining *what performance is acceptable*. Propose a design criterion that balances convergence speed against noise sensitivity.

**E12.9 — Mechanism vs. Procedure**
The separation principle says: "Design the controller and observer independently." A student interprets this as: "The observer does not affect the controller, and vice versa."

(a) This is mathematically correct but practically misleading. Explain one way the observer *does* affect closed-loop performance in practice (even though the eigenvalues are independent).
(b) If the observer is slow relative to the controller, what happens to the transient response? Draw a qualitative sketch.
(c) The Kalman filter (Chapter 15) is the *optimal* observer. In what sense is it optimal? What does it optimize that the Luenberger observer does not?

---

## 13.13 Summary

| Concept | Key Point |
|---------|-----------|
| Observer purpose | Estimate unmeasured states |
| Error dynamics | $\dot{e} = (A - LC)e$ |
| Observability | Required for arbitrary pole placement |
| Separation principle | Design K and L independently |
| Pole selection | Observer 2-5× faster than controller |

---

## What Comes Next

**Chapter 14: Digital Control** — Every observer and controller in Chapters 9–12 assumed continuous time. Real implementations run on microcontrollers at discrete sample rates. Chapter 14 bridges this gap.

**Chapter 15: Optimal Control (LQR/LQG)** — The Luenberger observer in this chapter uses hand-picked pole locations. The **Kalman filter** (Chapter 15) is the *optimal* observer — it minimizes estimation error in the presence of process and measurement noise. Combining the Kalman filter with LQR gives **LQG** (Linear Quadratic Gaussian) — the optimal version of the separation principle introduced in this chapter.

---

## References
