# Chapter 15: Optimal Control - LQR

---

## 🎯 Learning Objectives and Outcomes

### Chapter Objectives

This chapter introduces optimal control theory, specifically the Linear Quadratic Regulator (LQR), providing a systematic method for balancing performance and control effort through cost function optimization.

### Prerequisites
- Chapter 12-12: State-space representation and feedback
- Matrix algebra (Riccati equation)

---

## Learning Outcomes

After completing this chapter, students will be able to:

| Bloom's Level | Outcome |
|---------------|---------|
| **Remember** | Define cost function, LQR, and Riccati equation |
| **Understand** | Explain the optimality principle and trade-offs in LQR |
| **Apply** | Design LQR controllers for state-space systems |
| **Analyze** | Analyze effect of Q and R weighting matrices |
| **Evaluate** | Evaluate LQR designs against specifications |
| **Create** | Design optimal controllers for practical applications |

---

## Why This Chapter Matters

> **The Real Problem:** At 120 km/h, a crosswind gust pushes your autonomous car 30 cm toward the lane edge. The steering controller must react — but steer too aggressively and the ride is harsh, tires wear, and passengers feel unsafe. Steer too gently and the car drifts out of the lane. How do you *optimally* balance safety against comfort, performance against actuator effort?
>
> Chapter 12 showed that pole placement can achieve any desired closed-loop dynamics. But it left a critical question unanswered: *where exactly should the poles be?* Optimal control answers this by replacing intuition with mathematics — defining a **cost function** that quantifies the trade-off, then finding the control law that minimizes it.

---

## 15.1 Introduction to Optimal Control

### 15.1.1 Motivation

**Problem with pole placement:**
- Where exactly should poles be placed?
- How to balance performance vs. control effort?
- No systematic way to handle these trade-offs

**Optimal control solution:**
- Define a **cost function** (performance index)
- Find control that **minimizes** this cost
- Provides systematic design methodology

### 15.1.2 General Optimal Control Problem

**Given:** System dynamics $\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u}, t)$

**Find:** Control u(t) that minimizes:
$$J = \phi(\mathbf{x}(t_f), t_f) + \int_{t_0}^{t_f} L(\mathbf{x}, \mathbf{u}, t) dt$$

Subject to constraints on states and controls.

### 15.1.3 Pontryagin's Minimum/Maximum Principle

The most fundamental result in optimal control theory is Pontryagin's Minimum Principle. It provides **necessary conditions** for optimality using the concept of a Hamiltonian function.

**Hamiltonian Definition:**

For the system $\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})$ with cost:
$$J = \phi(\mathbf{x}(t_f), t_f) + \int_{t_0}^{t_f} L(\mathbf{x}, \mathbf{u}, t) \, dt$$

Define the Hamiltonian:
$$H(\mathbf{x}, \mathbf{u}, \boldsymbol{\lambda}, t) = L(\mathbf{x}, \mathbf{u}, t) + \boldsymbol{\lambda}^T f(\mathbf{x}, \mathbf{u})$$

where $\boldsymbol{\lambda}(t)$ is the **co-state** (adjoint) vector.

> **Theorem (Pontryagin's Minimum Principle):**
> If $\mathbf{u}^*(t)$ is optimal, then there exists a co-state trajectory $\boldsymbol{\lambda}^*(t)$ such that the following **necessary conditions** hold:

| Condition | Equation | Meaning |
|-----------|----------|---------|
| State equation | $\dot{\mathbf{x}} = \frac{\partial H}{\partial \boldsymbol{\lambda}} = f(\mathbf{x}, \mathbf{u})$ | System dynamics |
| Co-state equation | $\dot{\boldsymbol{\lambda}} = -\frac{\partial H}{\partial \mathbf{x}}$ | Adjoint dynamics |
| Stationarity | $\frac{\partial H}{\partial \mathbf{u}} = 0$ (unconstrained $\mathbf{u}$) | Optimality of control |
| Boundary condition | $\boldsymbol{\lambda}(t_f) = \frac{\partial \phi}{\partial \mathbf{x}}\Big|_{t_f}$ | Terminal co-state |

**Connection to LQR:**

For the linear-quadratic case $\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}$, $L = \mathbf{x}^T\mathbf{Q}\mathbf{x} + \mathbf{u}^T\mathbf{R}\mathbf{u}$:

1. **Hamiltonian:**
$$H = \mathbf{x}^T\mathbf{Q}\mathbf{x} + \mathbf{u}^T\mathbf{R}\mathbf{u} + \boldsymbol{\lambda}^T(\mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u})$$

2. **Stationarity** ($\partial H / \partial \mathbf{u} = 0$):
$$2\mathbf{R}\mathbf{u} + \mathbf{B}^T\boldsymbol{\lambda} = 0 \quad \Rightarrow \quad \mathbf{u}^* = -\tfrac{1}{2}\mathbf{R}^{-1}\mathbf{B}^T\boldsymbol{\lambda}$$

3. **Co-state equation:**
$$\dot{\boldsymbol{\lambda}} = -2\mathbf{Q}\mathbf{x} - \mathbf{A}^T\boldsymbol{\lambda}$$

4. **Key substitution:** Let $\boldsymbol{\lambda} = 2\mathbf{P}\mathbf{x}$, then:
$$\dot{\mathbf{P}} = -(\mathbf{P}\mathbf{A} + \mathbf{A}^T\mathbf{P} + \mathbf{Q} - \mathbf{P}\mathbf{B}\mathbf{R}^{-1}\mathbf{B}^T\mathbf{P})$$

5. **At steady state** ($\dot{\mathbf{P}} = 0$): this yields the **Algebraic Riccati Equation!**

Thus, Pontryagin's Minimum Principle provides the theoretical foundation from which the LQR solution is derived.

---

## 15.2 Linear Quadratic Regulator (LQR)

### 15.2.1 Problem Formulation

**System:** Linear time-invariant
$$\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}$$

**Cost function:** Quadratic
$$J = \int_0^{\infty} (\mathbf{x}'\mathbf{Q}\mathbf{x} + \mathbf{u}'\mathbf{R}\mathbf{u}) dt$$

where:
- $\mathbf{Q}$: State weighting matrix (n × n, positive semi-definite)
- $\mathbf{R}$: Control weighting matrix (m × m, positive definite)

### 15.2.2 Physical Interpretation

| Term | Meaning |
|------|---------|
| $\mathbf{x}'\mathbf{Q}\mathbf{x}$ | Penalty on state deviation |
| $\mathbf{u}'\mathbf{R}\mathbf{u}$ | Penalty on control effort |

**Trade-off:** 
- Large Q → Small states, possibly large control
- Large R → Small control, possibly slow response

### 15.2.3 Optimal Solution

> **Theorem (LQR Solution):**
> The optimal control is linear state feedback:
> $$\mathbf{u}^* = -\mathbf{K}\mathbf{x}$$
> where
> $$\mathbf{K} = \mathbf{R}^{-1}\mathbf{B}'\mathbf{P}$$
> and $\mathbf{P}$ is the unique positive definite solution of the **Algebraic Riccati Equation (ARE)**.

---

## 15.3 Algebraic Riccati Equation

### 15.3.1 The ARE

$$\mathbf{A}'\mathbf{P} + \mathbf{P}\mathbf{A} - \mathbf{P}\mathbf{B}\mathbf{R}^{-1}\mathbf{B}'\mathbf{P} + \mathbf{Q} = \mathbf{0}$$

### 15.3.2 Existence and Uniqueness

**Conditions for unique positive definite solution:**
1. (A, B) is stabilizable
2. (A, Q^{1/2}) is detectable

### 15.3.3 Solution Methods

1. **Eigenvalue decomposition** of Hamiltonian matrix
2. **Iterative methods** (Newton, Schur)
3. **MATLAB/numerical libraries**

### 15.3.4 Minimum Cost

The minimum cost from initial state $\mathbf{x}_0$:
$$J^* = \mathbf{x}_0'\mathbf{P}\mathbf{x}_0$$

### 15.3.5 ARE Derivation (Completing the Square)

The algebraic approach provides an elegant derivation of the ARE without requiring calculus of variations.

**Step 1: Assume optimal cost form.**

Assume $J^* = \mathbf{x}^T(0)\mathbf{P}\mathbf{x}(0)$ for some $\mathbf{P} \geq 0$. Since $\mathbf{x}(\infty) = 0$ for a stable system:
$$J = -\int_0^\infty \frac{d}{dt}(\mathbf{x}^T\mathbf{P}\mathbf{x}) \, dt = \int_0^\infty (\mathbf{x}^T\mathbf{Q}\mathbf{x} + \mathbf{u}^T\mathbf{R}\mathbf{u}) \, dt$$

Therefore:
$$0 = \frac{d}{dt}(\mathbf{x}^T\mathbf{P}\mathbf{x}) + \mathbf{x}^T\mathbf{Q}\mathbf{x} + \mathbf{u}^T\mathbf{R}\mathbf{u}$$
$$= \mathbf{x}^T(\mathbf{P}\mathbf{A} + \mathbf{A}^T\mathbf{P})\mathbf{x} + 2\mathbf{x}^T\mathbf{P}\mathbf{B}\mathbf{u} + \mathbf{x}^T\mathbf{Q}\mathbf{x} + \mathbf{u}^T\mathbf{R}\mathbf{u}$$

**Step 2: Complete the square in u.**

Rearranging and completing the square:
$$= \mathbf{x}^T(\mathbf{P}\mathbf{A} + \mathbf{A}^T\mathbf{P} + \mathbf{Q} - \mathbf{P}\mathbf{B}\mathbf{R}^{-1}\mathbf{B}^T\mathbf{P})\mathbf{x} + (\mathbf{u} + \mathbf{R}^{-1}\mathbf{B}^T\mathbf{P}\mathbf{x})^T\mathbf{R}(\mathbf{u} + \mathbf{R}^{-1}\mathbf{B}^T\mathbf{P}\mathbf{x})$$

**Step 3: Minimize.**

The second term is a quadratic form with $\mathbf{R} > 0$, so it is always $\geq 0$. For minimum cost, it must equal zero:
$$\mathbf{u}^* = -\mathbf{R}^{-1}\mathbf{B}^T\mathbf{P}\mathbf{x} = -\mathbf{K}\mathbf{x}$$

**Step 4: Obtain the ARE.**

The first term must also equal zero for all $\mathbf{x}$:
$$\boxed{\mathbf{P}\mathbf{A} + \mathbf{A}^T\mathbf{P} + \mathbf{Q} - \mathbf{P}\mathbf{B}\mathbf{R}^{-1}\mathbf{B}^T\mathbf{P} = \mathbf{0} \quad \text{(ARE)}}$$

This completing-the-square approach is more intuitive than the calculus of variations derivation and directly reveals the structure of the optimal control law.

---

## 15.4 Choosing Q and R Matrices

### 15.4.1 Diagonal Weighting

Simplest approach: Use diagonal matrices

$$\mathbf{Q} = \begin{bmatrix} q_1 & & 0 \\ & \ddots & \\ 0 & & q_n \end{bmatrix}, \quad
\mathbf{R} = \begin{bmatrix} r_1 & & 0 \\ & \ddots & \\ 0 & & r_m \end{bmatrix}$$

### 15.4.2 Bryson's Rule

Normalize by maximum acceptable values:
$$q_i = \frac{1}{x_{i,max}^2}, \quad r_j = \frac{1}{u_{j,max}^2}$$

**Example:**
- If position should stay within ±0.1 m: $q_1 = 100$
- If control should not exceed ±10 V: $r_1 = 0.01$

### 15.4.3 Tuning Guidelines

| Goal | Adjustment |
|------|------------|
| Faster response | Increase Q relative to R |
| Less control effort | Increase R relative to Q |
| Prioritize state $x_i$ | Increase $q_i$ |
| Limit control $u_j$ | Increase $r_j$ |

### 15.4.4 Cross-Coupling Terms

Non-diagonal Q can:
- Penalize combinations of states
- Achieve specific closed-loop characteristics
- Implement output weighting: $\mathbf{Q} = \mathbf{C}'\mathbf{C}$

---

## 15.5 Properties of LQR

### 15.5.1 Guaranteed Stability

If (A, B) controllable and (A, Q^{1/2}) observable:
- Closed-loop system is **asymptotically stable**
- All eigenvalues of (A - BK) are in LHP

### 15.5.2 Robustness Properties

**Gain margin:** At least 6 dB (factor of 2)
**Phase margin:** At least 60°

These are **guaranteed** properties, unlike arbitrary pole placement.

### 15.5.3 Return Difference Inequality

$$|1 + K(j\omega I - A)^{-1}B| \geq 1, \quad \forall \omega$$

This ensures the robustness guarantees.

---

## 15.6 LQR with Reference Tracking

### 15.6.1 Problem

Standard LQR drives states to zero. What if we want to track a reference?

### 15.6.2 Solution: Augmented System

Add integral of tracking error as a state:
$$\dot{x}_I = r - y = r - \mathbf{C}\mathbf{x}$$

Augmented system:
$$\begin{bmatrix} \dot{\mathbf{x}} \\ \dot{x}_I \end{bmatrix} = \begin{bmatrix} \mathbf{A} & \mathbf{0} \\ -\mathbf{C} & 0 \end{bmatrix} \begin{bmatrix} \mathbf{x} \\ x_I \end{bmatrix} + \begin{bmatrix} \mathbf{B} \\ 0 \end{bmatrix} u + \begin{bmatrix} \mathbf{0} \\ 1 \end{bmatrix} r$$

Apply LQR to augmented system.

---

## 15.7 Linear Quadratic Gaussian (LQG)

### 15.7.1 Combining LQR with Kalman Filter

When states are not directly measurable:
- **LQR** provides optimal state feedback
- **Kalman filter** provides optimal state estimation
- Combined: **LQG controller**

### 15.7.2 Separation Principle (LQG)

Design LQR and Kalman filter independently:
- Optimal for linear systems with Gaussian noise
- Closed-loop stability guaranteed

### 15.7.3 Limitation

LQG does **not** inherit LQR's robustness guarantees.

### 15.7.4 Kalman-Bucy Filter Equations (Duality with LQR)

The continuous-time Kalman filter is the **dual** of the LQR problem. For the stochastic system:
$$\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u} + \mathbf{w}, \quad \mathbf{y} = \mathbf{C}\mathbf{x} + \mathbf{v}$$

where $\mathbf{w} \sim \mathcal{N}(0, \mathbf{R}_w)$ is process noise and $\mathbf{v} \sim \mathcal{N}(0, \mathbf{R}_v)$ is measurement noise.

**Kalman-Bucy filter:**
$$\dot{\hat{\mathbf{x}}} = \mathbf{A}\hat{\mathbf{x}} + \mathbf{B}\mathbf{u} + \mathbf{L}(\mathbf{y} - \mathbf{C}\hat{\mathbf{x}})$$

**Observer gain:**
$$\mathbf{L} = \mathbf{P}\mathbf{C}^T\mathbf{R}_v^{-1}$$

**Filter Riccati equation:**
$$\mathbf{A}\mathbf{P} + \mathbf{P}\mathbf{A}^T + \mathbf{B}\mathbf{R}_w\mathbf{B}^T - \mathbf{P}\mathbf{C}^T\mathbf{R}_v^{-1}\mathbf{C}\mathbf{P} = \mathbf{0}$$

**LQR–Kalman Duality:**

The filter Riccati equation has the **identical structure** as the LQR ARE with the following substitutions:

| LQR | Kalman Filter |
|-----|---------------|
| $\mathbf{A}$ | $\mathbf{A}^T$ |
| $\mathbf{B}$ | $\mathbf{C}^T$ |
| $\mathbf{Q}$ | $\mathbf{B}\mathbf{R}_w\mathbf{B}^T$ |
| $\mathbf{R}$ | $\mathbf{R}_v$ |
| $\mathbf{K}$ | $\mathbf{L}^T$ |

This duality means that any algorithm for solving the LQR problem can also solve the Kalman filter problem, and vice versa.

### 15.7.5 LQG/LTR (Loop Transfer Recovery)

LQG = LQR + Kalman filter. The **separation principle** guarantees that the combined LQG controller is stabilizing.

**The robustness problem:**

Despite LQR having excellent robustness margins (GM $\geq$ 6 dB, PM $\geq$ 60°), the combined LQG controller does **NOT** inherit these margins. This was famously shown by Doyle (1978): LQG can have arbitrarily poor robustness.

**LTR Procedure:**

Loop Transfer Recovery restores the robustness of LQR by modifying the Kalman filter design:

1. Design the LQR gain $\mathbf{K}$ normally
2. For the Kalman filter, increase process noise: $\mathbf{R}_w \to \rho \mathbf{R}_w$ with large $\rho$
3. As $\rho \to \infty$, the loop transfer function at the plant input approaches the LQR loop transfer:
$$\mathbf{K}(s\mathbf{I} - \mathbf{A} + \mathbf{B}\mathbf{K} + \mathbf{L}\mathbf{C})^{-1}\mathbf{L}\mathbf{C}(s\mathbf{I} - \mathbf{A})^{-1}\mathbf{B} \xrightarrow{\rho \to \infty} \mathbf{K}(s\mathbf{I} - \mathbf{A})^{-1}\mathbf{B}$$

**Effect:** The LTR procedure recovers LQR's guaranteed robustness margins at the cost of increased observer bandwidth (more sensitivity to sensor noise).

| Parameter | Effect on LQG/LTR |
|-----------|-------------------|
| Small $\rho$ | Standard LQG, poor robustness |
| Large $\rho$ | Approaches LQR robustness |
| $\rho \to \infty$ | Full recovery of LQR margins |

> **Design trade-off:** LTR improves robustness at the expense of noise sensitivity. The designer must choose $\rho$ to balance these competing objectives.

---

## 15.8 Discrete-Time LQR

### 15.8.1 Discrete Cost Function

$$J = \sum_{k=0}^{\infty} [\mathbf{x}'(k)\mathbf{Q}\mathbf{x}(k) + \mathbf{u}'(k)\mathbf{R}\mathbf{u}(k)]$$

### 15.8.2 Discrete ARE

$$\mathbf{P} = \mathbf{A}'\mathbf{P}\mathbf{A} - \mathbf{A}'\mathbf{P}\mathbf{B}(\mathbf{B}'\mathbf{P}\mathbf{B} + \mathbf{R})^{-1}\mathbf{B}'\mathbf{P}\mathbf{A} + \mathbf{Q}$$

### 15.8.3 Discrete Gain

$$\mathbf{K} = (\mathbf{B}'\mathbf{P}\mathbf{B} + \mathbf{R})^{-1}\mathbf{B}'\mathbf{P}\mathbf{A}$$

---

## 15.9 Example: Mass-Spring-Damper with LQR

### 15.9.1 System

$$\mathbf{A} = \begin{bmatrix} 0 & 1 \\ -\omega_n^2 & -2\zeta\omega_n \end{bmatrix}, \quad \mathbf{B} = \begin{bmatrix} 0 \\ 1/m \end{bmatrix}$$

### 15.9.2 Design Comparison

Compare different Q/R ratios:

| Design | Q | R | Result |
|--------|---|---|--------|
| A | $\begin{bmatrix} 100 & 0 \\ 0 & 1 \end{bmatrix}$ | 1 | Fast, high effort |
| B | $\begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}$ | 1 | Balanced |
| C | $\begin{bmatrix} 1 & 0 \\ 0 & 1 \end{bmatrix}$ | 100 | Slow, low effort |

See **ch15_optimal_control.cpp** for complete implementation.

### 15.9.3 Worked Numerical Example: Double Integrator with cppplot

The double integrator is the simplest nontrivial LQR example:
$$\dot{\mathbf{x}} = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix} \mathbf{x} + \begin{bmatrix} 0 \\ 1 \end{bmatrix} u, \quad y = \begin{bmatrix} 1 & 0 \end{bmatrix} \mathbf{x}$$

**LQR design:** Penalize position more than velocity ($q_1 = 10, q_2 = 1, r = 1$).

**Hand calculation:**

The ARE for this system:
$$\begin{bmatrix} 0 & 0 \\ 1 & 0 \end{bmatrix}\begin{bmatrix} p_{11} & p_{12} \\ p_{12} & p_{22} \end{bmatrix} + \begin{bmatrix} p_{11} & p_{12} \\ p_{12} & p_{22} \end{bmatrix}\begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix} + \begin{bmatrix} 10 & 0 \\ 0 & 1 \end{bmatrix} - \begin{bmatrix} p_{12}^2 & p_{12}p_{22} \\ p_{12}p_{22} & p_{22}^2 \end{bmatrix} = \mathbf{0}$$

Solving the three independent equations yields: $p_{11} = \sqrt{10+20\sqrt{10}}$, $p_{12} = \sqrt{10}$, $p_{22} = \sqrt{1+2\sqrt{10}}$.

Optimal gain: $\mathbf{K} = \mathbf{R}^{-1}\mathbf{B}^T\mathbf{P} = \begin{bmatrix} p_{12} & p_{22} \end{bmatrix} \approx \begin{bmatrix} 3.162 & 2.514 \end{bmatrix}$

**cppplot implementation:**

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;
using namespace cppplot::control;

int main() {
    // Double integrator plant
    Matrix A = {{0, 1}, {0, 0}};
    Matrix B = {{0}, {1}};
    Matrix C = {{1, 0}};
    Matrix D = {{0}};

    // LQR design: penalize position more than velocity
    Matrix Q = {{10, 0}, {0, 1}};
    Matrix R = {{1}};

    // lqr() returns a 1×n row Matrix (the gain matrix)
    Matrix K = lqr(A, B, Q, R);

    // Riccati solution (if needed separately)
    Matrix P = care(A, B, Q, R);

    // Closed-loop system: A_cl = A - B*K
    Matrix Acl = A - B * K;        // (2×1)*(1×2) = (2×2)
    StateSpace sys_cl(Acl, B, C, D);

    // Closed-loop eigenvalues
    auto eigs = Acl.eigenvalues();
    std::cout << "Closed-loop poles:" << std::endl;
    for (const auto& e : eigs)
        std::cout << "  " << e.real() << " + " << e.imag() << "j" << std::endl;

    // Print gain
    std::cout << "K = [" << K(0,0) << ", " << K(0,1) << "]" << std::endl;

    // Observer design using pole placement (duality)
    auto L = observer_gain(A, C, {std::complex<double>(-5,0), 
                                   std::complex<double>(-6,0)});
    // Or using care for Kalman filter:
    // Matrix Rw = {{1}};   // process noise covariance
    // Matrix Rv = {{0.1}}; // measurement noise covariance
    // Matrix P_kf = care(A.T(), C.T(), B * Rw * B.T(), Rv);
    // std::vector<double> L_kalman = ...;  // from P_kf
}
```

**Expected results:**
- Closed-loop poles: $\approx -1.353 \pm j1.154$ (stable, well-damped)
- Step response: no overshoot, settling time $\approx 3$ s
- The higher $q_1/r$ ratio prioritizes position tracking over control effort

---

## 15.10 Real-World Application: Autonomous Vehicle Lane Keeping

> **Practical Integration:** Lane-keeping assist (LKA) in autonomous vehicles is an excellent application of LQR, integrating vehicle dynamics, steering actuator, camera/LIDAR sensors, and embedded processors.

### 15.10.1 System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│               AUTONOMOUS VEHICLE LANE KEEPING SYSTEM                    │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐   ┌───────────┐  │
│  │   Camera    │──►│   Lane      │──►│    LQR      │──►│  Steering │  │
│  │   System    │   │   Detection │   │  Controller │   │  Actuator │  │
│  │ (30-60 fps) │   │   (CNN/CV)  │   │  (50 Hz)    │   │  (EPS)    │  │
│  └─────────────┘   └─────────────┘   └──────┬──────┘   └─────┬─────┘  │
│                                             │                 │         │
│  ┌─────────────┐                           │                 │         │
│  │   LIDAR     │──► Localization ──────────┤                 │         │
│  │             │    (HD Map fusion)        │                 │         │
│  └─────────────┘                           │                 ▼         │
│                                            │          ┌─────────────┐  │
│  ┌─────────────┐                           │          │   Vehicle   │  │
│  │    IMU      │──► State Estimation ──────┘          │  Dynamics   │  │
│  │  GPS/RTK    │    (EKF)                             │  (Bicycle   │  │
│  └─────────────┘                                      │   Model)    │  │
│                                                        └─────────────┘  │
│                                                               │         │
│  ┌─────────────────────────────────────────────────────────────┐       │
│  │                    CAN Bus / FlexRay                         │       │
│  │        (Interconnects all ECUs: 500 kbps - 10 Mbps)         │       │
│  └─────────────────────────────────────────────────────────────┘       │
└─────────────────────────────────────────────────────────────────────────┘
```

### 15.10.2 Vehicle Lateral Dynamics (Bicycle Model)

**State Variables:**

| State | Symbol | Description |
|-------|--------|-------------|
| Lateral position | $e_y$ | Distance from lane center |
| Lateral velocity | $\dot{e}_y$ | Rate of lateral displacement |
| Heading error | $e_\psi$ | Angle between vehicle and lane |
| Heading rate | $\dot{e}_\psi$ | Yaw rate error |

**Linearized State-Space Model:**

$$\begin{bmatrix} \dot{e}_y \\ \ddot{e}_y \\ \dot{e}_\psi \\ \ddot{e}_\psi \end{bmatrix} = 
\begin{bmatrix} 
0 & 1 & 0 & 0 \\
0 & -\frac{2(C_f+C_r)}{mV_x} & \frac{2(C_f+C_r)}{m} & -\frac{2(C_f l_f - C_r l_r)}{mV_x} \\
0 & 0 & 0 & 1 \\
0 & -\frac{2(C_f l_f - C_r l_r)}{I_z V_x} & \frac{2(C_f l_f - C_r l_r)}{I_z} & -\frac{2(C_f l_f^2 + C_r l_r^2)}{I_z V_x}
\end{bmatrix}
\begin{bmatrix} e_y \\ \dot{e}_y \\ e_\psi \\ \dot{e}_\psi \end{bmatrix} +
\begin{bmatrix} 0 \\ \frac{2C_f}{m} \\ 0 \\ \frac{2C_f l_f}{I_z} \end{bmatrix} \delta$$

Where:
- $C_f, C_r$: Front/rear cornering stiffness (N/rad)
- $l_f, l_r$: Distance from CG to front/rear axle
- $V_x$: Longitudinal velocity
- $I_z$: Yaw moment of inertia
- $\delta$: Front wheel steering angle (control input)

### 15.10.3 LQR Design for Lane Keeping

**Cost Function Weighting:**

$$J = \int_0^\infty \left( q_1 e_y^2 + q_2 \dot{e}_y^2 + q_3 e_\psi^2 + q_4 \dot{e}_\psi^2 + r \delta^2 \right) dt$$

**Q and R Selection:**

$$\mathbf{Q} = \begin{bmatrix} 
q_1 & 0 & 0 & 0 \\
0 & q_2 & 0 & 0 \\
0 & 0 & q_3 & 0 \\
0 & 0 & 0 & q_4
\end{bmatrix}, \quad R = r$$

**Typical Values:**

| Weight | Physical Meaning | Value Range |
|--------|------------------|-------------|
| $q_1$ | Lateral position penalty | 1 - 100 |
| $q_2$ | Lateral velocity penalty | 0.1 - 10 |
| $q_3$ | Heading error penalty | 10 - 1000 |
| $q_4$ | Yaw rate penalty | 0.1 - 10 |
| $r$ | Steering effort penalty | 1 - 100 |

### 15.10.4 Multi-Domain Integration

| Domain | Component | LQR Design Impact |
|--------|-----------|-------------------|
| **Mechanical** | Tire dynamics, suspension | Cornering stiffness varies with load |
| **Electrical** | EPS motor, torque sensor | Steering actuator bandwidth (~10 Hz) |
| **Electronic** | ECU, CAN bus | Computation + communication delay |
| **Sensors** | Camera, IMU, GPS | Measurement noise covariance |
| **Software** | Lane detection CNN | Latency 30-100 ms |
| **Thermal** | Tire temperature | Affects grip → cornering stiffness |
| **Safety** | ISO 26262 ASIL-D | Redundant control paths required |

### 15.10.5 Speed-Scheduled LQR (Gain Scheduling)

Since vehicle dynamics depend on speed $V_x$, use gain scheduling:

$$\mathbf{K}(V_x) = \text{Interpolate}[\mathbf{K}_{low}, \mathbf{K}_{mid}, \mathbf{K}_{high}]$$

**Lookup Table:**

| Speed (km/h) | $K_1$ | $K_2$ | $K_3$ | $K_4$ |
|--------------|-------|-------|-------|-------|
| 30 | 0.15 | 0.05 | 2.5 | 0.8 |
| 60 | 0.12 | 0.08 | 3.2 | 1.0 |
| 90 | 0.08 | 0.10 | 4.0 | 1.2 |
| 120 | 0.05 | 0.12 | 5.0 | 1.5 |

### 15.10.6 Implementation Considerations

> **Engineering Challenges:**
> 
> 1. **Latency:** Camera → CNN → LQR → Actuator chain has 50-150 ms delay
> 2. **Model Uncertainty:** Tire characteristics vary ±20% with conditions
> 3. **Actuator Limits:** Steering rate limited to ~400 deg/s
> 4. **Disturbances:** Road camber, crosswind, uneven pavement
> 5. **Safety:** Must detect and handle lane detection failures

**Robust Design:**

```
┌─────────────────────────────────────────────────────────────────┐
│            LANE KEEPING CONTROL HIERARCHY                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Level 3:  Path Planning (reference trajectory)     [10 Hz]     │
│                          │                                      │
│                          ▼                                      │
│  Level 2:  LQR Controller + Feedforward             [50 Hz]     │
│            u = -Kx + u_ff(curvature)                            │
│                          │                                      │
│                          ▼                                      │
│  Level 1:  Steering Angle Controller (low-level)    [500 Hz]   │
│            EPS motor current control                            │
│                          │                                      │
│                          ▼                                      │
│  Level 0:  EPS Hardware (torque production)         [Hardware] │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

### 📋 Signal Dictionary — LQR Speed Control of a DC Motor

| Signal | Symbol | Meaning | Typical unit |
|--------|--------|---------|--------------|
| State vector | $\mathbf{x} = [\omega, i_a]^T$ | Motor speed and armature current | [rad/s, A] |
| Control input | $u$ | Armature voltage (the ONE thing we choose) | V |
| Reference state | $\mathbf{x}_r$ | Desired operating point | [rad/s, A] |
| State error | $\mathbf{e} = \mathbf{x} - \mathbf{x}_r$ | Deviation from desired state | [rad/s, A] |
| State weighting | $\mathbf{Q}$ | Penalty on state error — "how much do I care about each state?" | — |
| Control weighting | $R$ | Penalty on control effort — "how expensive is voltage?" | — |
| Cost functional | $J = \int_0^\infty (\mathbf{e}^T Q \mathbf{e} + u^T R u)\,dt$ | Total "regret" — the number LQR minimizes | — |
| Riccati solution | $\mathbf{P}$ | Positive-definite matrix solving the ARE | — |
| Optimal gain | $\mathbf{K} = R^{-1}B^T P$ | Feedback gain — maps state to control action | — |
| Kalman gain (LQG) | $\mathbf{L}$ | Observer gain from dual Riccati equation | — |
| Process noise | $\mathbf{w}(t) \sim N(0, Q_w)$ | Unmodeled disturbances (load torque, friction) | N·m |
| Measurement noise | $v(t) \sim N(0, R_v)$ | Sensor noise (encoder jitter, ADC noise) | rad/s |
| Estimated state | $\hat{\mathbf{x}}$ | Kalman filter output — best guess of true state | [rad/s, A] |

> **Key insight:** $Q$ and $R$ are not arbitrary tuning knobs. $Q_{ii}$ should be $\approx 1/(\text{max acceptable } e_i)^2$ — this normalizes the cost so each state's penalty has comparable magnitude. Similarly, $R \approx 1/(\text{max acceptable } u)^2$. LQR is not "optimal" in some absolute sense; it is optimal *for the cost you specified*. Bad $Q, R$ → optimal solution to the wrong problem.

---

## 15.11 Exercises

**E14.1 (LQR for Double Integrator)**
For the double integrator
$$\mathbf{A} = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix}, \quad \mathbf{B} = \begin{bmatrix} 0 \\ 1 \end{bmatrix}$$
solve the LQR problem with $\mathbf{Q} = \begin{bmatrix} 1 & 0 \\ 0 & 0 \end{bmatrix}$ and $R = 1$.

(a) Write out the three independent equations from the ARE.

(b) Solve for $\mathbf{P}$ by hand.

(c) Compute $\mathbf{K} = R^{-1}\mathbf{B}^T\mathbf{P}$ and the closed-loop poles.

---

**E14.2 (Verification with cppplot)**
Verify your hand calculation from E14.1 using `cppplot::control::lqr(A, B, Q, R)`.

(a) Compare the $\mathbf{K}$, $\mathbf{P}$, and closed-loop eigenvalues with your analytical solution.

(b) Simulate the closed-loop step response and plot position, velocity, and control effort.

---

**E14.3 (Cheap Control Limit)**
For the double integrator with $\mathbf{Q} = \mathbf{I}$:

(a) Compute the LQR gain $\mathbf{K}$ for $R = 10, 1, 0.1, 0.01, 0.001$.

(b) Plot the closed-loop poles as $R \to 0$. What pattern do you observe?

(c) Show analytically that as $R \to 0$, the closed-loop poles approach the mirror images (reflected about the imaginary axis) of the open-loop transmission zeros of the system.

---

**E14.4 (Pontryagin's Minimum Principle)**
Apply Pontryagin's Minimum Principle to minimize
$$J = \int_0^\infty (x^2 + u^2) \, dt$$
for the scalar system $\dot{x} = -x + u$, $x(0) = 1$.

(a) Form the Hamiltonian $H = x^2 + u^2 + \lambda(-x + u)$.

(b) Apply the stationarity condition $\partial H/\partial u = 0$ to find $u^*(\lambda)$.

(c) Write and solve the co-state equation $\dot{\lambda} = -\partial H/\partial x$.

(d) Find the optimal control $u^*(t)$ and the optimal trajectory $x^*(t)$.

---

**E14.5 (Solving the ARE by Hand)**
Solve the Algebraic Riccati Equation for the scalar system $A = -2$, $B = 1$, $Q = 4$, $R = 1$:
$$A^T P + PA - PBR^{-1}B^T P + Q = 0$$

(a) Substitute the scalar values and solve the resulting quadratic for $P$.

(b) Select the positive solution. Compute $K$ and the closed-loop pole.

(c) Verify using `cppplot::control::care(A, B, Q, R)`.

---

**E14.6 (Kalman Filter Design)**
Design a Kalman filter for the system
$$\mathbf{A} = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix}, \quad \mathbf{C} = \begin{bmatrix} 1 & 0 \end{bmatrix}$$
with process noise covariance $R_w = 0.1$ (entering through $\mathbf{B} = [0; 1]$) and measurement noise covariance $R_v = 1$.

(a) Using the duality between LQR and Kalman filtering, formulate the filter Riccati equation.

(b) Solve for the Kalman gain $\mathbf{L} = \mathbf{P}\mathbf{C}^T R_v^{-1}$.

(c) Simulate the filter with a noisy measurement and plot the true state vs. the estimate.

---

**E14.7 (Challenge: LQR Robustness Margins)**
Show that the LQR controller guarantees the following robustness margins at the plant input:
- Gain margin: $[\frac{1}{2}, \infty)$ (i.e., at least 6 dB)
- Phase margin: at least $60°$

*Hint:* Start from the return difference inequality $|1 + \mathbf{K}(j\omega\mathbf{I} - \mathbf{A})^{-1}\mathbf{B}| \geq 1$ for all $\omega$, which follows from the ARE. Show that the Nyquist plot of the loop transfer function $L(j\omega) = \mathbf{K}(j\omega\mathbf{I} - \mathbf{A})^{-1}\mathbf{B}$ never enters the disk of radius 1 centered at $(-1, 0)$.

---

**E14.8 🔴 (Level 3 — When "Optimal" Fails)**
You design an LQR controller for a DC motor with $Q = \text{diag}(100, 1)$ and $R = 0.01$, giving aggressive response.

(a) Compute the control gain $K$ and simulate with the nominal model. Report $M_p$, $t_s$, and peak control voltage.

(b) The real motor's inertia $J$ is 50% larger than the model value. Simulate with the perturbed plant. Does the LQR still stabilize? What happens to the transient?

(c) Now switch to an LQG design where the Kalman filter assumes $Q_w = 0.01 I$ (small process noise) but the real system has $Q_w = 1.0 I$. Simulate. What fails first — the controller or the observer?

(d) Explain why LQR has guaranteed robustness margins (E14.7) but LQG does *not*. What is it about the Kalman filter that destroys the margins?

**E14.9 🔴 (Level 3 — Q and R as Engineering Decisions)**
A lane-keeping system has states $\mathbf{x} = [e_y, \dot{e}_y, e_\psi, \dot{e}_\psi]^T$ (lateral error, heading error, and their rates).

(a) The highway has 3.7 m wide lanes. Set $Q_{11}$ so that a 0.5 m lateral error contributes cost = 1. What is $Q_{11}$?

(b) The maximum comfortable steering rate is 15°/s. Set $R$ accordingly.

(c) A passenger complains the car "jerks too much." Which matrix element do you change, and in which direction? Explain the physical reasoning.

(d) An engineer proposes $Q = 10000 \cdot I$ and $R = 0.001$ for "maximum performance." What is mathematically optimal about this? What is practically dangerous?

**E14.10 ⚫ (Level 4 — Optimal for Whom?)**

(a) LQR minimizes $J = \int (\mathbf{x}^T Q \mathbf{x} + u^T R u)\,dt$. Is the physical system aware of this cost function? Does nature minimize $J$?

(b) Two engineers design LQR controllers for the same motor with different $Q, R$. Both are "optimal." How can two different controllers both be optimal? What does this say about the word "optimal" in engineering?

(c) Compare LQR's cost function with H∞'s worst-case criterion (Ch. 17). In what sense is LQR optimistic and H∞ pessimistic? Which is more appropriate for safety-critical systems? Justify.

---

## 15.12 Summary

| Concept | Key Point |
|---------|-----------|
| Pontryagin's Principle | Necessary conditions via Hamiltonian: $\dot{\boldsymbol{\lambda}} = -\partial H/\partial \mathbf{x}$, $\partial H/\partial \mathbf{u} = 0$ |
| LQR objective | Minimize $\int (x'Qx + u'Ru)dt$ |
| Solution | $u = -Kx$, $K = R^{-1}B'P$ |
| ARE | Algebraic Riccati Equation for P |
| ARE derivation | Completing the square in u yields ARE and optimal gain |
| Q/R tuning | Balance state penalty vs. control effort |
| Guaranteed margins | GM ≥ 6 dB, PM ≥ 60° |
| Kalman-Bucy filter | Dual of LQR: $L = PC^TR_v^{-1}$, same ARE structure |
| LQG/LTR | LQG = LQR + Kalman; LTR recovers LQR robustness via $\rho \to \infty$ |
| Reference tracking | Augment with integral state |

---

## References


1. Anderson, B.D.O. & Moore, J.B. (2007). *Optimal Control: Linear Quadratic Methods*
2. Athans, M. & Falb, P.L. (1966). *Optimal Control*
3. Lewis, F.L. et al. (2012). *Optimal Control*, 3rd ed.
4. Rajamani, R. (2012). *Vehicle Dynamics and Control*, 2nd ed.
