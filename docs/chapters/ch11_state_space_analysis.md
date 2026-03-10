# Chapter 12: State-Space Analysis

---

## 🎯 Learning Objectives and Outcomes

### Chapter Objectives

This chapter develops essential analysis tools for state-space systems: computing system response, testing controllability and observability, and understanding the relationship between internal structure and input-output behavior.

### Prerequisites
- Chapter 10: State-Space Introduction
- Matrix exponentials and eigenvalue decomposition

---

## Why This Chapter Matters: Can You Actually Control Your System?

> **The Real Engineering Problem:** You've built a drone with 4 motors and want to control position, altitude, and attitude. You have accelerometers and gyroscopes. But can you actually achieve stable flight? Are there "hidden modes" your sensors can't see?

### Controllability and Observability: Not Just Math!

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    WHAT THESE CONCEPTS REALLY MEAN                          │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   CONTROLLABILITY: "Can my actuators affect ALL the system's modes?"       │
│   ─────────────────────────────────────────────────────────────────        │
│                                                                             │
│   Example 1: DC Motor Position Control                                     │
│   • States: x₁ = position, x₂ = velocity                                   │
│   • Input: armature voltage                                                │
│   • Voltage → Current → Torque → Acceleration → Velocity → Position       │
│   • ✓ CONTROLLABLE: Voltage eventually affects both states                 │
│                                                                             │
│   Example 2: Two Masses with Common Spring (Symmetric)                     │
│   ┌───┐    ╱╲╱╲╱╲    ┌───┐                                                 │
│   │ M ├───╲╱╲╱╲╱────┤ M │  Push only the left mass                        │
│   └───┘              └───┘                                                  │
│   • If you push left mass, center-of-mass mode is controllable            │
│   • But relative mode (both masses moving opposite) is NOT controllable!  │
│   • ✗ NOT CONTROLLABLE: Can't affect all modes from single input          │
│                                                                             │
│   ─────────────────────────────────────────────────────────────────────    │
│                                                                             │
│   OBSERVABILITY: "Can my sensors SEE all the system's modes?"              │
│   ──────────────────────────────────────────────────────────               │
│                                                                             │
│   Example 1: Motor with Encoder on Output Shaft                            │
│   • States: x₁ = motor position, x₂ = motor velocity                       │
│   • Sensor: encoder measures position directly                             │
│   • Position is measured, velocity can be computed (differentiate)         │
│   • ✓ OBSERVABLE: All states can be determined from measurements          │
│                                                                             │
│   Example 2: Two Tanks in Series, Sensor on Second Tank Only               │
│   ┌─────┐   ┌─────┐                                                        │
│   │ h₁  │──▶│ h₂  │◀── sensor here only                                    │
│   └─────┘   └─────┘                                                        │
│   • If flow from tank 1 to tank 2 depends only on h₂-h₁...               │
│   • Can we figure out h₁ from measuring h₂?                               │
│   • Depends on dynamics! May have unobservable modes                      │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Why These Concepts Are Crucial

| Situation | Physical Consequence | What to Do |
|-----------|---------------------|------------|
| **Uncontrollable mode, stable** | Mode will decay on its own | No problem, but can't speed it up |
| **Uncontrollable mode, unstable** | System WILL go unstable, nothing you can do | Redesign hardware! |
| **Unobservable mode, stable** | Can't estimate state, but it decays | May need more sensors for diagnostics |
| **Unobservable mode, unstable** | Growing mode you CAN'T SEE | DANGEROUS! System fails unexpectedly |

**Real Example: Aircraft Spin Mode**
```
Early aircraft had spin modes that were:
• Uncontrollable with ailerons alone (needed rudder)
• Sometimes unobservable until too late

Modern aircraft use:
• Redundant control surfaces (multiple ways to affect each mode)
• Redundant sensors (multiple ways to observe each mode)
• This is controllability and observability in practice!
```

---

## Learning Outcomes

After completing this chapter, students will be able to:

| Bloom's Level | Outcome |
|---------------|---------|
| **Remember** | Define state transition matrix, controllability, and observability |
| **Understand** | Explain the significance of eigenvalues in system behavior |
| **Apply** | Calculate state transition matrix and system response |
| **Analyze** | Test controllability and observability of systems |
| **Evaluate** | Assess system properties from state-space representation |
| **Create** | Design controllable and observable state-space models |

---

## 11.1 Solution of State Equations

### 11.1.1 Homogeneous Solution

For $\dot{\mathbf{x}} = \mathbf{A}\mathbf{x}$ with initial condition $\mathbf{x}(0) = \mathbf{x}_0$:

$$\mathbf{x}(t) = e^{\mathbf{A}t}\mathbf{x}_0$$

### 11.1.2 State Transition Matrix

The **state transition matrix** (matrix exponential):

$$\boldsymbol{\Phi}(t) = e^{\mathbf{A}t} = \mathbf{I} + \mathbf{A}t + \frac{(\mathbf{A}t)^2}{2!} + \frac{(\mathbf{A}t)^3}{3!} + ...$$

**Properties:**
1. $\boldsymbol{\Phi}(0) = \mathbf{I}$
2. $\boldsymbol{\Phi}^{-1}(t) = \boldsymbol{\Phi}(-t)$
3. $\boldsymbol{\Phi}(t_1 + t_2) = \boldsymbol{\Phi}(t_1)\boldsymbol{\Phi}(t_2)$
4. $\frac{d}{dt}\boldsymbol{\Phi}(t) = \mathbf{A}\boldsymbol{\Phi}(t)$

### 11.1.3 Calculation Methods

**Method 1: Laplace Transform**
$$\boldsymbol{\Phi}(t) = \mathcal{L}^{-1}\{(s\mathbf{I} - \mathbf{A})^{-1}\}$$

**Method 2: Diagonalization** (when A is diagonalizable)
$$e^{\mathbf{A}t} = \mathbf{P}e^{\mathbf{\Lambda}t}\mathbf{P}^{-1}$$

where $\mathbf{\Lambda}$ is diagonal matrix of eigenvalues.

**Method 3: Cayley-Hamilton Theorem**

By Cayley-Hamilton, every matrix satisfies its own characteristic equation. For an $n \times n$ matrix $\mathbf{A}$ with characteristic polynomial $\Delta(s) = s^n + a_{n-1}s^{n-1} + \cdots + a_0$:

$$e^{\mathbf{A}t} = \sum_{k=0}^{n-1} \alpha_k(t) \mathbf{A}^k$$

where the scalar coefficients $\alpha_k(t)$ are found by solving:
$$e^{\lambda_i t} = \sum_{k=0}^{n-1} \alpha_k(t) \lambda_i^k, \quad i = 1, 2, \ldots, n$$

For repeated eigenvalue $\lambda$ with multiplicity $m$, differentiate $j = 0, 1, \ldots, m-1$ times:
$$\frac{d^j}{d\lambda^j} e^{\lambda t} = \sum_{k=0}^{n-1} \alpha_k(t) \frac{d^j}{d\lambda^j} \lambda^k$$

> **Example (2×2):** For $\mathbf{A} = \begin{bmatrix} 0 & 1 \\ -2 & -3 \end{bmatrix}$ with eigenvalues $\lambda_1 = -1, \lambda_2 = -2$:
>
> $$e^{-t} = \alpha_0(t) + \alpha_1(t)(-1), \quad e^{-2t} = \alpha_0(t) + \alpha_1(t)(-2)$$
>
> Solving: $\alpha_0(t) = 2e^{-t} - e^{-2t}$, $\alpha_1(t) = e^{-t} - e^{-2t}$
>
> $$e^{\mathbf{A}t} = (2e^{-t} - e^{-2t})\mathbf{I} + (e^{-t} - e^{-2t})\mathbf{A} = \begin{bmatrix} 2e^{-t} - e^{-2t} & e^{-t} - e^{-2t} \\ -2e^{-t} + 2e^{-2t} & -e^{-t} + 2e^{-2t} \end{bmatrix}$$

### 11.1.4 Complete Solution

For $\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}$:

$$\mathbf{x}(t) = e^{\mathbf{A}t}\mathbf{x}(0) + \int_0^t e^{\mathbf{A}(t-\tau)}\mathbf{B}\mathbf{u}(\tau)d\tau$$

---

## 11.2 Eigenvalues and System Modes

### 11.2.1 Characteristic Equation

$$\det(\lambda\mathbf{I} - \mathbf{A}) = 0$$

Solutions $\lambda_1, \lambda_2, ..., \lambda_n$ are **eigenvalues** of $\mathbf{A}$.

### 11.2.2 System Modes

Each eigenvalue corresponds to a **mode** of the system:

| Eigenvalue Type | Mode Response | Physical Example |
|-----------------|---------------|------------------|
| Real negative | Decaying exponential | Damped mass, RC discharge |
| Real positive | Growing exponential | Unstable pendulum, positive feedback |
| Complex conjugate | Oscillation | Spring-mass, LC circuit |
| Zero | Constant (integrator) | Tank level, motor position |

### 11.2.3 Stability from Eigenvalues

> **Theorem (Stability):**
> A linear system is asymptotically stable if and only if all eigenvalues of $\mathbf{A}$ have negative real parts.

$$\text{Stable} \Leftrightarrow \text{Re}(\lambda_i) < 0, \quad \forall i$$

---

## 11.3 Controllability

### 11.3.1 Definition

> **Definition (Controllability):**
> A system is **controllable** if for any initial state $\mathbf{x}_0$ and any final state $\mathbf{x}_f$, there exists a finite time $t_f$ and input $\mathbf{u}(t)$ that transfers the state from $\mathbf{x}_0$ to $\mathbf{x}_f$.

**Physical Meaning:** Can the actuator(s) influence ALL states, either directly or indirectly through the system dynamics?

### 11.3.2 Controllability Matrix

$$\mathcal{C} = \begin{bmatrix} \mathbf{B} & \mathbf{AB} & \mathbf{A}^2\mathbf{B} & \cdots & \mathbf{A}^{n-1}\mathbf{B} \end{bmatrix}$$

**Physical interpretation of columns:**
- $\mathbf{B}$: Direct effect of input
- $\mathbf{AB}$: Effect one time step later (through dynamics)
- $\mathbf{A}^2\mathbf{B}$: Effect two time steps later
- ...and so on

### 11.3.3 Controllability Test

> **Theorem:**
> System (A, B) is controllable if and only if:
> $$\text{rank}(\mathcal{C}) = n$$

### 11.3.4 Example

For system:
$$\mathbf{A} = \begin{bmatrix} 0 & 1 \\ -2 & -3 \end{bmatrix}, \quad \mathbf{B} = \begin{bmatrix} 0 \\ 1 \end{bmatrix}$$

**Controllability matrix:**
$$\mathcal{C} = \begin{bmatrix} 0 & 1 \\ 1 & -3 \end{bmatrix}$$

$$\det(\mathcal{C}) = -1 \neq 0 \Rightarrow \text{rank} = 2$$

**Conclusion:** System is controllable.

### 11.3.5 Physical Interpretation

- **Controllable:** Input can influence all system modes
- **Uncontrollable:** Some modes cannot be affected by input
- Uncontrollable modes are "hidden" from input

---

## 11.4 Observability

### 11.4.1 Definition

> **Definition (Observability):**
> A system is **observable** if the initial state $\mathbf{x}(0)$ can be uniquely determined from the output $\mathbf{y}(t)$ and input $\mathbf{u}(t)$ over a finite time interval.

### 11.4.2 Observability Matrix

$$\mathcal{O} = \begin{bmatrix} \mathbf{C} \\ \mathbf{CA} \\ \mathbf{CA}^2 \\ \vdots \\ \mathbf{CA}^{n-1} \end{bmatrix}$$

### 11.4.3 Observability Test

> **Theorem:**
> System (A, C) is observable if and only if:
> $$\text{rank}(\mathcal{O}) = n$$

### 11.4.4 Duality

There is a **duality** between controllability and observability:

| Property | (A, B) | Dual Property | (A', C') |
|----------|--------|---------------|----------|
| Controllable | | Observable | |
| Observable | | Controllable | |

**Mathematical relationship:**
- (A, B) controllable ⟺ (A', B') observable
- (A, C) observable ⟺ (A', C') controllable

### 11.4.5 Physical Interpretation

- **Observable:** Output contains information about all states
- **Unobservable:** Some states cannot be determined from output
- Unobservable states are "hidden" from output

### 11.4.6 Popov–Belevitch–Hautus (PBH) Test

The **PBH test** provides a powerful eigenvalue-based algebraic test for controllability and observability that is often easier to apply than computing the rank of the full controllability or observability matrix.

> **Theorem (PBH Test for Controllability):**
> The pair $(\mathbf{A}, \mathbf{B})$ is controllable if and only if:
> $$\text{rank}\begin{bmatrix} s\mathbf{I} - \mathbf{A} & \mathbf{B} \end{bmatrix} = n \quad \text{for all } s \in \mathbb{C}$$
>
> Equivalently, it suffices to check only the eigenvalues of $\mathbf{A}$:
> $$\text{rank}\begin{bmatrix} \lambda_i\mathbf{I} - \mathbf{A} & \mathbf{B} \end{bmatrix} = n \quad \text{for every eigenvalue } \lambda_i \text{ of } \mathbf{A}$$
>
> If the rank drops below $n$ at some eigenvalue $\lambda_i$, then the mode associated with $\lambda_i$ is **uncontrollable**.

> **Theorem (PBH Test for Observability):**
> The pair $(\mathbf{A}, \mathbf{C})$ is observable if and only if:
> $$\text{rank}\begin{bmatrix} s\mathbf{I} - \mathbf{A} \\ \mathbf{C} \end{bmatrix} = n \quad \text{for all } s \in \mathbb{C}$$
>
> Equivalently:
> $$\text{rank}\begin{bmatrix} \lambda_i\mathbf{I} - \mathbf{A} \\ \mathbf{C} \end{bmatrix} = n \quad \text{for every eigenvalue } \lambda_i \text{ of } \mathbf{A}$$
>
> If the rank drops below $n$ at some eigenvalue $\lambda_i$, then the mode associated with $\lambda_i$ is **unobservable**.

**Example:** For $(\mathbf{A}, \mathbf{B})$ with $\mathbf{A} = \begin{bmatrix} -1 & 0 \\ 0 & -2 \end{bmatrix}$, $\mathbf{B} = \begin{bmatrix} 1 \\ 0 \end{bmatrix}$.

Eigenvalues: $\lambda_1 = -1$, $\lambda_2 = -2$.

Check $\lambda_1 = -1$:
$$\begin{bmatrix} 0 & 0 \\ 0 & 1 \end{bmatrix} \begin{bmatrix} 1 \\ 0 \end{bmatrix} = \begin{bmatrix} 0 & 0 & 1 \\ 0 & 1 & 0 \end{bmatrix} \implies \text{rank} = 2 \; \checkmark$$

Check $\lambda_2 = -2$:
$$\begin{bmatrix} -1 & 0 & 1 \\ 0 & 0 & 0 \end{bmatrix} \implies \text{rank} = 1 \neq 2$$

The mode at $\lambda_2 = -2$ is **uncontrollable** — the input cannot influence the second state.

**Why PBH is useful:**
- Identifies *which specific modes* are uncontrollable or unobservable.
- Numerically more reliable than checking $\text{rank}(\mathcal{C})$ for large systems.
- Connects controllability/observability directly to the system's eigenstructure.

---

## 11.5 Kalman Decomposition

### 11.5.1 Four Subspaces

Any system can be decomposed into four parts:

1. **Controllable and Observable** (C,O)
2. **Controllable but Unobservable** (C,Ō)
3. **Uncontrollable but Observable** (C̄,O)
4. **Uncontrollable and Unobservable** (C̄,Ō)

### 11.5.2 Decomposed Form

$$\mathbf{A} = \begin{bmatrix} A_{co} & 0 & A_{13} & 0 \\ A_{21} & A_{c\bar{o}} & A_{23} & A_{24} \\ 0 & 0 & A_{\bar{c}o} & 0 \\ 0 & 0 & A_{43} & A_{\bar{c}\bar{o}} \end{bmatrix}$$

### 11.5.3 Transfer Function Insight

**Key result:** Transfer function only depends on (C,O) subspace.

- Uncontrollable modes: **pole-zero cancellation** from input
- Unobservable modes: **pole-zero cancellation** to output

---

## 11.6 Minimal Realization

### 11.6.1 Definition

A state-space realization is **minimal** if:
1. It is both controllable and observable
2. It has the smallest possible dimension

### 11.6.2 Characterization

> **Theorem:**
> A realization is minimal if and only if it is both controllable and observable.

### 11.6.3 Finding Minimal Realization

1. Start with any realization from transfer function
2. Identify uncontrollable/unobservable subspaces
3. Remove those states
4. Result is minimal realization

---

## 11.7 Stability Analysis

### 11.7.1 Types of Stability

| Type | Condition |
|------|-----------|
| Asymptotically stable | All eigenvalues strictly in LHP (Re(λ) < 0) |
| Marginally stable | No eigenvalues in open RHP, and all eigenvalues on the imaginary axis are **simple** (algebraic multiplicity = 1) |
| Unstable | At least one eigenvalue in RHP, **or** repeated eigenvalue on imaginary axis |

> **⚠️ Common Pitfall:** A repeated eigenvalue on the $j\omega$-axis with a Jordan block produces unbounded $te^{j\omega t}$ terms, making the system **unstable** — not marginally stable. Only *simple* (non-repeated) imaginary-axis eigenvalues yield bounded oscillations.

### 11.7.2 BIBO Stability

> **Theorem:**
> A system is BIBO stable if and only if all poles of the transfer function have negative real parts.

**Note:** Hidden modes (uncontrollable/unobservable) don't affect BIBO stability but can cause internal instability.

### 11.7.3 Internal Stability

For complete stability, we need:
1. All eigenvalues of A in LHP (internal stability)
2. Not just transfer function poles

---

## 11.8 Gramians

### 11.8.1 Controllability Gramian

$$\mathbf{W}_c(t) = \int_0^t e^{\mathbf{A}\tau}\mathbf{B}\mathbf{B}'e^{\mathbf{A}'\tau}d\tau$$

System controllable ⟺ $\mathbf{W}_c(t)$ is positive definite for some t.

### 11.8.2 Observability Gramian

$$\mathbf{W}_o(t) = \int_0^t e^{\mathbf{A}'\tau}\mathbf{C}'\mathbf{C}e^{\mathbf{A}\tau}d\tau$$

System observable ⟺ $\mathbf{W}_o(t)$ is positive definite for some t.

---

## 11.9 Electrical and Telecommunications Analysis Examples

### 11.9.1 Controllability Analysis: Three-Phase Inverter

**Problem:** Analyze controllability of a grid-connected inverter in dq-frame.

**System States:**
- $i_d$: Direct-axis current (active power component)
- $i_q$: Quadrature-axis current (reactive power component)

**Signal Dictionary — Three-Phase Inverter (dq-frame)**

| Signal | Symbol | Unit | Physical Meaning | Sensor/Actuator |
|--------|--------|------|------------------|-----------------|
| d-axis current (state) | $i_d$ | A | Active power component — controls real power flow to grid | Current sensor + Park transform |
| q-axis current (state) | $i_q$ | A | Reactive power component — controls voltage support / power factor | Current sensor + Park transform |
| d-axis voltage (control) | $v_d$ | V | Voltage command for active power channel | Inverter PWM (d-axis) |
| q-axis voltage (control) | $v_q$ | V | Voltage command for reactive power channel | Inverter PWM (q-axis) |
| Grid voltage (disturbance) | $v_{gd}, v_{gq}$ | V | Grid voltage in dq-frame — acts as a disturbance | PLL + voltage measurement |
| Resistance | $R$ | Ω | Filter winding resistance — the $-R/L$ diagonal in **A** | (System parameter) |
| Inductance | $L$ | H | Filter inductance — limits current rate of change | (System parameter) |
| Cross-coupling | $\omega$ | rad/s | Grid angular frequency — creates off-diagonal terms in **A** | PLL |

> **Key insight:** The off-diagonal $\pm\omega$ terms in the **A** matrix are not mathematical artifacts — they represent the physical coupling between d and q axes due to the rotating reference frame. Decoupling control must cancel these terms.

**State-Space Model:**

$$\begin{bmatrix} \dot{i}_d \\ \dot{i}_q \end{bmatrix} = 
\begin{bmatrix} -R/L & \omega \\ -\omega & -R/L \end{bmatrix}
\begin{bmatrix} i_d \\ i_q \end{bmatrix} +
\frac{1}{L}\begin{bmatrix} v_d - v_{gd} \\ v_q - v_{gq} \end{bmatrix}$$

With two control inputs ($v_d$ and $v_q$):

$$\mathbf{B} = \frac{1}{L}\begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}$$

**Controllability Matrix:**
$$\mathcal{C} = \begin{bmatrix} \mathbf{B} & \mathbf{AB} \end{bmatrix} = \frac{1}{L}\begin{bmatrix} 1 & 0 & -R/L & \omega \\ 0 & 1 & -\omega & -R/L \end{bmatrix}$$

**Analysis:**
$$\text{rank}(\mathcal{C}) = 2 = n \quad \Rightarrow \quad \text{Fully Controllable}$$

**Physical Interpretation:**
- With both $v_d$ and $v_q$, we can independently control both current components
- This enables independent control of active power (via $i_d$) and reactive power (via $i_q$)

**What if single-phase?**
With only one voltage input, system becomes:

$$\mathbf{B} = \frac{1}{L}\begin{bmatrix} 1 \\ 0 \end{bmatrix}$$

$$\mathcal{C} = \frac{1}{L}\begin{bmatrix} 1 & -R/L \\ 0 & -\omega \end{bmatrix}$$

rank(𝒞) = 2 → Still controllable! (Cross-coupling from ω provides path to $i_q$)

### 11.9.2 Observability Analysis: Sensorless Motor Control

**Problem:** Determine if motor speed can be estimated from voltage and current measurements only (no encoder).

**PMSM Model in dq-frame:**

States: $\mathbf{x} = [i_d, i_q, \omega]^T$

$$\mathbf{A} = \begin{bmatrix}
-R_s/L_d & \omega L_q/L_d & 0 \\
-\omega L_d/L_q & -R_s/L_q & \lambda_{pm}/L_q \\
0 & K_t/J & -B/J
\end{bmatrix}$$

Measurements: Both currents (typical) 

$$\mathbf{C} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \end{bmatrix}$$

**Observability Matrix:**
$$\mathcal{O} = \begin{bmatrix} \mathbf{C} \\ \mathbf{CA} \\ \mathbf{CA}^2 \end{bmatrix}$$

**Analysis Result:**

For typical PMSM parameters, $\text{rank}(\mathcal{O}) = 3$ when $\omega \neq 0$.

**Critical Finding:** At zero speed ($\omega = 0$), observability may be lost because the back-EMF (which carries speed information) disappears!

```
┌────────────────────────────────────────────────────────────────────────┐
│        SENSORLESS MOTOR OBSERVABILITY                                  │
├────────────────────────────────────────────────────────────────────────┤
│                                                                        │
│   Speed (ω)      Observability    Practical Implication               │
│   ─────────      ─────────────    ─────────────────────               │
│                                                                        │
│   ω >> 0         OBSERVABLE       • Back-EMF provides speed info      │
│   (Running)                       • Standard observers work well      │
│                                                                        │
│   ω ≈ 0          UNOBSERVABLE     • No back-EMF at zero speed        │
│   (Standstill)                    • Need HIGH-FREQUENCY INJECTION     │
│                                   • Or use saliency-based methods     │
│                                                                        │
│   SOLUTION: Hybrid observer                                           │
│   • Low speed: HF injection (uses motor saliency)                     │
│   • High speed: Back-EMF based observer                               │
│   • Smooth transition around 5-10% rated speed                        │
│                                                                        │
└────────────────────────────────────────────────────────────────────────┘
```

### 11.9.3 PLL Stability via Eigenvalue Analysis

**Problem:** Analyze stability of a digital PLL using state-space eigenvalues.

**State-Space Model:**

$$\mathbf{x} = \begin{bmatrix} \theta_e \\ \omega_e \end{bmatrix}, \quad
\mathbf{A} = \begin{bmatrix} 0 & 1 \\ -K_i & -K_p \end{bmatrix}$$

Where $K_p$ and $K_i$ are proportional and integral gains.

**Characteristic Equation:**
$$\det(s\mathbf{I} - \mathbf{A}) = s^2 + K_p s + K_i = 0$$

**Eigenvalues (Closed-Loop Poles):**
$$\lambda_{1,2} = \frac{-K_p \pm \sqrt{K_p^2 - 4K_i}}{2}$$

**Stability Regions:**

| Condition | $K_p > 0$, $K_i > 0$ | $K_p > 2\sqrt{K_i}$ |
|-----------|----------------------|---------------------|
| Stable | Yes | Yes (overdamped) |
| Stable | Yes | No (underdamped) |
| Unstable | No | - |

**CppPlot Implementation:**

```cpp
#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <cmath>
#include <complex>
#include <iostream>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    // Analyze PLL stability for different gain combinations
    std::vector<double> Kp_vals, Ki_vals;
    std::vector<double> real_part_vec;
    std::vector<std::string> stability_vec;
    
    for (double Kp = 0.1; Kp <= 10; Kp += 0.5) {
        for (double Ki = 0.1; Ki <= 50; Ki += 2) {
            // Build 2x2 PLL system matrix and compute eigenvalues
            Matrix A_pll = {{0, 1}, {-Ki, -Kp}};
            auto eigs = A_pll.eigenvalues();
            double max_real = std::max(eigs[0].real(), eigs[1].real());
            
            Kp_vals.push_back(Kp);
            Ki_vals.push_back(Ki);
            real_part_vec.push_back(max_real);
            stability_vec.push_back(max_real < 0 ? "Stable" : "Unstable");
        }
    }
    
    std::cout << "PLL Stability Analysis Complete\n";
    std::cout << "All combinations with Kp>0, Ki>0 are stable.\n";
    
    // Specific design: Find gains for desired natural frequency and damping
    double wn = 2*M_PI*1000;   // 1 kHz natural frequency
    double zeta = 0.707;       // Damping ratio
    
    double Kp_design = 2 * zeta * wn;
    double Ki_design = wn * wn;
    
    std::cout << "\nFor wn = 1 kHz, zeta = 0.707:\n";
    std::cout << "Kp = " << Kp_design << "\n";
    std::cout << "Ki = " << Ki_design << "\n";
    
    return 0;
}
```

> **🔍 Stop and Think — Mechanism Questions**
>
> Before moving on to the next application:
> 1. The controllability matrix $\mathcal{C}$ had full rank for the three-phase inverter because it has two independent inputs ($v_d$ and $v_q$). *Physically*, what would make the system uncontrollable? (Hint: what if both inputs affected the same state in the same way?)
> 2. The eigenvalues of $\mathbf{A}$ determine stability. For the inverter, the diagonal entries $-R/L$ are negative (stable), but the off-diagonal entries $\pm\omega$ create coupling. Can the coupling terms alone cause instability? Why or why not?
> 3. If $R \to 0$ (superconducting filter), the eigenvalues become purely imaginary ($\pm j\omega$). What does this mean physically for the current waveforms?

### 11.9.4 Power System Mode Analysis

**Problem:** Analyze oscillatory modes in a two-generator power system.

**Simplified Two-Machine Model:**

States: $\mathbf{x} = [\delta_1, \omega_1, \delta_2, \omega_2]^T$

Where $\delta$ is rotor angle and $\omega$ is rotor speed deviation.

$$\mathbf{A} = \begin{bmatrix}
0 & 1 & 0 & 0 \\
-K_{11}/M_1 & -D_1/M_1 & -K_{12}/M_1 & 0 \\
0 & 0 & 0 & 1 \\
-K_{21}/M_2 & 0 & -K_{22}/M_2 & -D_2/M_2
\end{bmatrix}$$

**Eigenvalue Analysis reveals:**

1. **Inter-area mode:** Low frequency (0.1-1 Hz), generators swing against each other
2. **Local modes:** Higher frequency (1-2 Hz), individual generator oscillations

**Observability from PMU measurements:**

If Phasor Measurement Units (PMUs) measure only at Generator 1:

$$\mathbf{C} = \begin{bmatrix} 1 & 0 & 0 & 0 \end{bmatrix}$$

Testing observability reveals whether inter-area oscillations can be detected from a single measurement location - crucial for Wide Area Monitoring Systems (WAMS).

### 11.9.5 Summary: Analysis Concepts in EE and Telecom

| Concept | Power Electronics | Telecommunications |
|---------|-------------------|-------------------|
| **Controllability** | Can we control both P and Q? | Can we track phase and frequency? |
| **Observability** | Sensorless control possible? | Can we estimate channel state? |
| **Stability** | Converter stability margins | PLL lock-in range |
| **Modal analysis** | Resonances in filters | Oscillation modes in loops |

**Key Insights:**

1. **Cross-coupling aids controllability:** In dq-frame, rotation coupling means single input can eventually affect both states.

2. **Observability depends on operating point:** Sensorless motor control loses observability at zero speed - a fundamental limitation, not a design flaw.

3. **Eigenvalue location = performance:** For PLL, eigenvalues directly give natural frequency and damping of tracking response.

---

## 📝 Exercises

### Exercise 11.1 — Matrix Exponential via Cayley-Hamilton

For the matrix:

$$A = \begin{bmatrix} 0 & 1 \\ -2 & -3 \end{bmatrix}$$

**(a)** Find the eigenvalues $\lambda_1, \lambda_2$ of $A$.

**(b)** By the Cayley-Hamilton theorem, $e^{At}$ for a $2 \times 2$ matrix can be written as:

$$e^{At} = \alpha_0(t)I + \alpha_1(t)A$$

Set up the system of equations using $e^{\lambda_1 t} = \alpha_0 + \alpha_1 \lambda_1$ and $e^{\lambda_2 t} = \alpha_0 + \alpha_1 \lambda_2$. Solve for $\alpha_0(t)$ and $\alpha_1(t)$.

**(c)** Substitute back to obtain the four entries of $e^{At}$ as explicit functions of time.

**(d)** **Verify** using eigenvalue decomposition: $e^{At} = P \, \text{diag}(e^{\lambda_1 t}, e^{\lambda_2 t}) \, P^{-1}$, where $P$ is the matrix of eigenvectors. Confirm both methods give the same result.

---

### Exercise 11.2 — Controllability and Observability Test

Given:

$$A = \begin{bmatrix} 0 & 1 \\ -6 & -5 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ 1 \end{bmatrix}, \quad C = \begin{bmatrix} 1 & 0 \end{bmatrix}$$

**(a)** Compute the **controllability matrix** $\mathcal{C} = [B \;\; AB]$ and check its rank. Is the system controllable?

**(b)** Compute the **observability matrix** $\mathcal{O} = \begin{bmatrix} C \\ CA \end{bmatrix}$ and check its rank. Is the system observable?

**(c)** If either test fails, find the eigenvalues of $A$ and identify which mode (eigenvalue) is uncontrollable or unobservable.

**(d)** Give a physical interpretation: what does it mean for a mode to be uncontrollable? Unobservable?

---

### Exercise 11.3 — CppPlot Verification of Controllability/Observability

Verify your results from Exercise 11.2 using CppPlot:

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;

int main() {
    Matrix A = {{0, 1}, {-6, -5}};
    Matrix B = {{0}, {1}};
    Matrix C = {{1, 0}};
    
    auto Cm = controllability_matrix(A, B);
    std::cout << "Controllability matrix:\n" << Cm << std::endl;
    std::cout << "Controllable: " << isControllable(A, B) << std::endl;
    
    auto Om = observability_matrix(A, C);
    std::cout << "Observability matrix:\n" << Om << std::endl;
    std::cout << "Observable: " << isObservable(A, C) << std::endl;
    
    // Eigenvalues
    auto eigs = eigenvalues(A);
    std::cout << "Eigenvalues: " << eigs << std::endl;
}
```

**(a)** Run the code and compare with your hand calculations.

**(b)** Modify $B$ to $[1; 0]$ and re-test controllability. Does it change? Explain.

**(c)** Modify $C$ to $[0 \; 1]$ and re-test observability. Does it change? Explain.

---

### Exercise 11.4 — Reachable Subspace

For the diagonal system:

$$A = \begin{bmatrix} -1 & 0 \\ 0 & -2 \end{bmatrix}, \quad B = \begin{bmatrix} 1 \\ 1 \end{bmatrix}$$

**(a)** Compute the controllability matrix $\mathcal{C} = [B \;\; AB]$.

**(b)** Determine $\text{rank}(\mathcal{C})$. Is the system controllable?

**(c)** The **reachable subspace** is $\text{Im}(\mathcal{C})$ (the column space of $\mathcal{C}$). Find a basis for this subspace.

**(d)** Interpret geometrically: can the input $u(t)$ drive the state to any point in $\mathbb{R}^2$, or only to a subspace?

**(e)** Now change $B = \begin{bmatrix} 1 \\ 0 \end{bmatrix}$. Is the system still controllable? What is the reachable subspace?

---

### Exercise 11.5 — State Transition Matrix via Laplace Transform

For the matrix:

$$A = \begin{bmatrix} -1 & 1 \\ 0 & -2 \end{bmatrix}$$

**(a)** Compute $(sI - A)$.

**(b)** Find $(sI - A)^{-1}$ by computing the adjugate and determinant.

**(c)** Take the inverse Laplace transform of each entry to obtain $\Phi(t) = e^{At} = \mathcal{L}^{-1}\{(sI - A)^{-1}\}$.

**(d)** Verify the properties:
- $\Phi(0) = I$
- $\dot{\Phi}(t) = A\Phi(t)$
- $\Phi(t_1 + t_2) = \Phi(t_1)\Phi(t_2)$

**(e)** For initial condition $x(0) = \begin{bmatrix} 1 \\ 0 \end{bmatrix}$, compute the free response $x(t) = \Phi(t) x(0)$ and sketch both components.

---

### Exercise 11.6 — Cayley-Hamilton for Matrix Powers

A system has the characteristic polynomial:

$$\lambda^2 + 5\lambda + 6 = 0$$

**(a)** Factor the polynomial and find the eigenvalues.

**(b)** By the Cayley-Hamilton theorem, $A^2 + 5A + 6I = 0$. Express $A^2$ in terms of $A$ and $I$.

**(c)** Using the recurrence $A^{k+2} = -5A^{k+1} - 6A^k$, express $A^3$, $A^4$, and $A^5$ in terms of $I$ and $A$.

**(d)** Verify with a specific matrix. Let $A = \begin{bmatrix} 0 & 1 \\ -6 & -5 \end{bmatrix}$ (companion form). Compute $A^5$ directly and compare with your expression $\alpha_0 I + \alpha_1 A$.

**(e)** Why is this technique useful in computing $e^{At}$ (via the series definition)?

---

### Exercise 11.7 — Invariance of Controllability Under State Transformation ⭐

Prove that controllability is invariant under a non-singular state transformation.

**(a)** Let $(A, B)$ be a controllable pair with controllability matrix $\mathcal{C} = [B \; AB \; A^2B \; \cdots \; A^{n-1}B]$.

**(b)** Under the transformation $\bar{x} = T^{-1}x$, the new system matrices are $\bar{A} = T^{-1}AT$ and $\bar{B} = T^{-1}B$.

**(c)** Compute the new controllability matrix:

$$\bar{\mathcal{C}} = [\bar{B} \;\; \bar{A}\bar{B} \;\; \bar{A}^2\bar{B} \;\; \cdots \;\; \bar{A}^{n-1}\bar{B}]$$

Show that $\bar{A}^k \bar{B} = T^{-1}A^k B$ for all $k$.

**(d)** Conclude that $\bar{\mathcal{C}} = T^{-1}\mathcal{C}$, and since $T$ is non-singular, $\text{rank}(\bar{\mathcal{C}}) = \text{rank}(\mathcal{C})$.

**(e)** Does the same argument hold for observability? State and prove the analogous result for $(A, C)$.

### Problem Identification Exercises (Level 3-4)

**Exercise 11.8 — What Is the Real Problem?**
An engineer runs `rank(ctrb(A,B))` in MATLAB and gets $n$ (full rank). The engineer declares: "The system is controllable. We can place poles anywhere." Three months into prototyping, the controller requires unrealistically large voltages.

(a) Explain why mathematical controllability does NOT guarantee practical controllability. (Hint: controllability Gramian, condition number.)
(b) For the three-phase inverter example (§11.9.1), the system is controllable with two inputs ($v_d$, $v_q$). What happens if the inverter current limit is 10A but the controller demands 50A? Is the system still "controllable"?
(c) Propose a more useful definition of "practical controllability" that accounts for actuator limits.

**Exercise 11.9 — Mechanism vs. Procedure**
A student computes the observability matrix $\mathcal{O}$ for a 4th-order system and finds $\text{rank}(\mathcal{O}) = 3$. The student says: "The system is unobservable. We cannot build an observer."

(a) This conclusion is too strong. What can you *actually* say? (Hint: can you observe 3 of the 4 states?)
(b) Identify the unobservable subspace. What is its physical meaning? Give a concrete example (e.g., a mode that produces no output).
(c) If the unobservable mode is stable, does it matter? Under what conditions can you safely ignore it?

---

## What Comes Next

**Chapter 12: State Feedback Control** — Controllability tells you *whether* you can steer the system to any state. Chapter 12 shows you *how*: state feedback $\mathbf{u} = -\mathbf{K}\mathbf{x}$ places the closed-loop eigenvalues at any desired location — provided the system is controllable.

**Chapter 13: State Observers** — Observability tells you *whether* you can reconstruct unmeasured states from the output. Chapter 13 builds observers that do exactly this, enabling state feedback even when not all states are sensed.

Together, Chapters 10–12 form a unified framework: analyze (controllability + observability) → design (state feedback) → implement (observer-based control).

---

## References
