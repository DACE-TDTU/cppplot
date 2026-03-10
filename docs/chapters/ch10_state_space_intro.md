# Chapter 10: Introduction to State-Space Representation

---

## 🎯 Learning Objectives and Outcomes

### Chapter Objectives

This chapter introduces state-space representation, the foundation of modern control theory, enabling analysis and design of multi-input multi-output (MIMO) systems and advanced control techniques.

### Prerequisites
- Chapter 2: Mathematical Modeling
- Chapter 3: Laplace Transform
- Linear algebra (matrices, eigenvalues)

---

## Why This Chapter Matters: Beyond Input-Output

> **The Real Engineering Problem:** Your robot arm has 6 joints, 6 motors, and 6 encoders. Transfer functions work great for one motor at a time. But the joints interact - moving joint 3 affects joint 4 through inertia coupling. How do you design a controller for the whole arm?

### The Limitation You've Hit

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    TRANSFER FUNCTION LIMITATIONS                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   SISO System (Transfer Function Works Great):                             │
│   ──────────────────────────────────────────                               │
│                                                                             │
│   Reference ──▶ Controller ──▶ Motor ──▶ Position                         │
│       1 input               1 output                                        │
│       G(s) = Y(s)/U(s) completely describes the system                     │
│                                                                             │
│   ─────────────────────────────────────────────────────────────────────    │
│                                                                             │
│   MIMO System (Transfer Function Not Enough):                              │
│   ───────────────────────────────────────────                              │
│                                                                             │
│   References ──▶ Controller ──▶ Robot ──▶ Positions                        │
│   ┌─────────┐                   ┌────────────────┐   ┌─────────┐           │
│   │ θ1_ref  │                   │ Joint 1 ◄─────┐│   │ θ1      │           │
│   │ θ2_ref  │                   │ Joint 2 ◄──┐  ││   │ θ2      │           │
│   │ θ3_ref  │   ???             │ Joint 3 ◄──┼──┼┼   │ θ3      │           │
│   │ θ4_ref  │                   │    ▲       │  ││   │ θ4      │           │
│   │ θ5_ref  │                   │    │ Inertia coupling                    │
│   │ θ6_ref  │                   │    │       │  ││   │ θ6      │           │
│   └─────────┘                   └────┼───────┴──┼┘   └─────────┘           │
│       6 inputs                  COUPLED!        6 outputs                   │
│                                                                             │
│   You would need a 6×6 matrix of transfer functions!                       │
│   And you couldn't see the internal dynamics                               │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### State-Space: Seeing Inside the Black Box

The state-space approach answers: **"What's happening INSIDE the system?"**

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    STATE = COMPLETE INTERNAL INFORMATION                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   PHYSICAL SYSTEM              STATE VARIABLES                             │
│   ───────────────              ───────────────                             │
│                                                                             │
│   Mass-spring-damper           x₁ = position     (stored potential energy) │
│                                x₂ = velocity     (stored kinetic energy)   │
│                                                                             │
│   RC circuit                   x = capacitor voltage (stored charge energy)│
│                                                                             │
│   DC motor                     x₁ = position     (mechanical energy)       │
│                                x₂ = velocity     (kinetic energy)          │
│                                x₃ = current      (magnetic energy)         │
│                                                                             │
│   Thermal system               x = temperature   (stored thermal energy)   │
│                                                                             │
│   Robot arm (6-DOF)            x₁...x₆ = joint angles                     │
│                                x₇...x₁₂ = joint velocities                │
│                                (12 states for 6 joints!)                   │
│                                                                             │
│   KEY INSIGHT: States = Variables describing STORED ENERGY                 │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Why Control Engineers Need State-Space

| Capability | Transfer Function | State-Space |
|------------|-------------------|-------------|
| **MIMO systems** | Awkward (matrix of TFs) | Natural |
| **Internal dynamics** | Hidden | Explicit |
| **Initial conditions** | Assumed zero | Included |
| **Nonlinear systems** | Not applicable | Straightforward extension |
| **Digital implementation** | Discretize each TF | Direct matrix equations |
| **Optimal control** | Not applicable | LQR, MPC, etc. |
| **State estimation** | Not applicable | Kalman filter |

---

## Learning Outcomes

After completing this chapter, students will be able to:

| Bloom's Level | Outcome |
|---------------|---------|
| **Remember** | Define state, state variables, and state space |
| **Understand** | Explain the concept of system order and state dimension |
| **Apply** | Write state-space models for physical systems |
| **Analyze** | Convert between transfer function and state-space forms |
| **Evaluate** | Compare different state-space representations |
| **Create** | Construct state-space models from system descriptions |

---

## 10.1 Motivation for State-Space Approach

### 10.1.1 Limitations of Transfer Functions

Transfer functions:
- Limited to **linear, time-invariant (LTI)** systems
- Require **zero initial conditions**
- Cannot easily represent **multi-input, multi-output (MIMO)** systems
- Hide internal system dynamics

### 10.1.2 Advantages of State-Space

State-space representation:
- Natural extension to **MIMO** systems
- Handles **nonlinear** systems
- Includes **initial conditions**
- Reveals **internal structure** (controllability, observability)
- Foundation for **modern control** (optimal, robust)

### 10.1.3 ⚠️ The Embedded Systems Reality: Why Trial-and-Error Fails for MIMO

> **An uncomfortable truth:** The vast majority of embedded control projects in university labs — capstone projects, competitions, thesis work — use exactly one design method: **PID with manual tuning**. Perhaps with a Kalman filter for sensor fusion. The mathematical model, if it exists at all, is written in the report *after* the system already works.
>
> For SISO systems, this approach can succeed. For MIMO systems, it **always** fails.

#### Why PID Tuning Works for SISO

A single-input, single-output system has **one** loop to close. The engineer adjusts three gains ($K_p, K_i, K_d$) and observes one output. The search space is 3-dimensional. Human intuition can navigate this:

```
SISO: One motor, one sensor
┌──────────────────────────────────────────────┐
│                                              │
│  r ──►(+)──► PID ──► Motor ──► Position      │
│        ▲                        │            │
│        └────── encoder ◄────────┘            │
│                                              │
│  Tuning: Increase Kp → faster but oscillates │
│          Add Kd → damps oscillation          │
│          Add Ki → removes steady-state error │
│                                              │
│  3 knobs, 1 output → human can "feel" it    │
└──────────────────────────────────────────────┘
```

#### Why PID Tuning FAILS for MIMO

A Wheeled Mobile Robot (WMR) has 2 inputs (left/right wheel voltages) and 3 outputs ($x, y, \theta$). A quadrotor has 4 inputs (rotor speeds) and 6 outputs (position + orientation). The states are **coupled** — changing one input affects ALL outputs simultaneously:

```
MIMO: Wheeled Mobile Robot (WMR) — 2 inputs, 3 outputs
┌──────────────────────────────────────────────────────────────────────────┐
│                                                                          │
│  v_left  ────┐    ┌─────────────────┐    ┌── x(t)                       │
│              ├───►│  Nonlinear      │────┤                              │
│  v_right ────┘    │  Kinematics     │    ├── y(t)                       │
│                   │                 │    │                              │
│                   │  ẋ = v·cos(θ)   │    └── θ(t)                       │
│                   │  ẏ = v·sin(θ)   │                                   │
│                   │  θ̇ = ω          │                                   │
│                   └─────────────────┘                                   │
│                                                                          │
│  PROBLEM: v and ω affect x, y, AND θ simultaneously                    │
│           through NONLINEAR coupling (sin θ, cos θ)                     │
│                                                                          │
│  Trial-and-error: you tune PID for x-tracking.                          │
│  But changing v affects y through sin(θ).                                │
│  So you re-tune y. But that changes θ.                                  │
│  So you re-tune θ. But that changes x again.                            │
│                                                                          │
│  This is NOT convergent. You are chasing your own tail.                 │
└──────────────────────────────────────────────────────────────────────────┘
```

```
MIMO: Quadrotor — 4 inputs, 6 outputs
┌──────────────────────────────────────────────────────────────────────────┐
│                                                                          │
│  ω₁² ──┐     ┌──────────────────┐     ┌── x         ┐                  │
│  ω₂² ──┼────►│  Rigid body      │─────┤── y         │ position         │
│  ω₃² ──┤     │  dynamics        │     ├── z         ┘                  │
│  ω₄² ──┘     │  + aerodynamics  │     ├── ϕ (roll)  ┐                  │
│              │                  │     ├── θ (pitch) │ orientation      │
│              └──────────────────┘     └── ψ (yaw)   ┘                  │
│                                                                          │
│  COUPLING: increasing ω₁ and ω₃ together → pitch → forward motion     │
│            but also changes altitude (total thrust changes)              │
│            and changes yaw (reaction torques don't cancel)              │
│                                                                          │
│  12 states, 4 inputs → tuning by hand requires adjusting               │
│  a 4×12 gain matrix K = 48 parameters simultaneously                   │
│                                                                          │
│  Trial-and-error in 48-dimensional space?                               │
│  This. Does. Not. Work.                                                 │
└──────────────────────────────────────────────────────────────────────────┘
```

#### The Dimensionality Argument

| System | Inputs | States | Gain parameters | Can human tune? |
|--------|--------|--------|-----------------|----------------|
| DC motor (SISO) | 1 | 2 | 3 ($K_p, K_i, K_d$) | ✅ Yes — 3D search |
| Inverted pendulum (SISO output) | 1 | 4 | 3 (PID on angle) | ⚠️ Sometimes — ignores 2 states |
| WMR (MIMO) | 2 | 3 | $2 \times 3 = 6$ minimum | ❌ Rarely — cross-coupling |
| Quadrotor (MIMO) | 4 | 12 | $4 \times 12 = 48$ | ❌ Never — impossible by hand |
| 6-DOF robot arm | 6 | 12 | $6 \times 12 = 72$ | ❌ Never |

#### What the Mathematical Model Actually Does

The student who asks "Why do I need a model when I can just tune?" is really asking: **"Why can't I search a 48-dimensional space by intuition?"**

The answer: **the model replaces search with computation.**

```
┌───────────────────────────────────────────────────────────────────────────┐
│  WITHOUT MODEL (trial-and-error):                                        │
│  ─────────────────────────────────                                       │
│  1. Guess K₁₁ = 5. Test. Output oscillates.                            │
│  2. Reduce K₁₁ to 3. Test. Oscillation stops but y drifts.             │
│  3. Increase K₂₃ to 2. Test. y improves but θ diverges.                │
│  4. Reduce K₃₁... but this affects x again.                             │
│  5. 487 iterations later: "It sort of works on this flat floor."        │
│  6. Move to a different floor: everything breaks.                        │
│                                                                           │
│  WITH MODEL (systematic design):                                         │
│  ─────────────────────────────────                                       │
│  1. Write ẋ = Ax + Bu from physics (20 minutes)                         │
│  2. Check controllability: rank[B, AB, A²B,...] = n? YES                │
│  3. Choose desired poles (from performance specs)                        │
│  4. Compute K = place(A, B, desired_poles) → ONE command, ONE answer    │
│  5. Or: K = lqr(A, B, Q, R) → optimal, with guaranteed margins          │
│  6. Deploy. Works on any floor because the PHYSICS is encoded.           │
│                                                                           │
│  Time: 20 minutes + 1 computation vs. 2 weeks of trial and error        │
│  Robustness: physics-based vs. overfitted to one test condition          │
└───────────────────────────────────────────────────────────────────────────┘
```

#### 🛑 Stop and Think

> 1. You have a WMR that works on a smooth lab floor (tuned by trial-and-error). You take it outside to rough terrain. The friction coefficients change. Does the controller still work? **Why the answer depends on whether you used a model.**
>
> 2. A quadrotor team spends 3 weeks hand-tuning PID gains. They achieve stable hover. Then they add a 50g camera payload. The drone crashes. A different team uses LQR with an identified model — they update $J$ (inertia) and $m$ (mass) in the model, recompute $K$ in 10 seconds. Why does the model-based approach generalize while the tuning-based approach doesn't?
>
> 3. The inverted pendulum can *sometimes* be stabilized with a single PID loop on the angle. But this ignores cart position — the cart drifts to the rail limit and crashes. The state-space approach stabilizes *all four states* simultaneously. What property of PID prevents it from stabilizing the cart position and pendulum angle at the same time? (Hint: how many outputs does a single PID controller have?)

> **→ Connection to Chapter 2 (§2.8.7):** The mathematical model is the *design model* (Level 2). When students skip the model and tune by hand, they are trying to design for the *real system* (Level 3) directly — without the map. For SISO, the road is short enough to walk without a map. For MIMO, you're lost in a 48-dimensional forest.

---

## 10.2 Fundamental Concepts

### 10.2.1 State Definition

> **Definition (State):**
> The state of a system at time $t_0$ is the minimum set of information needed at $t_0$ such that, together with input $u(t)$ for $t \geq t_0$, the output $y(t)$ for $t \geq t_0$ can be uniquely determined.

**Physical Intuition:** The state tells you everything about the system's "memory" - past inputs have left their mark on the state variables.

### 10.2.2 State Variables

**State variables** $x_1(t), x_2(t), ..., x_n(t)$ are chosen to describe the system state.

**State vector:**
$$\mathbf{x}(t) = \begin{bmatrix} x_1(t) \\ x_2(t) \\ \vdots \\ x_n(t) \end{bmatrix}$$

### 10.2.3 State Space

The **state space** is the n-dimensional space whose axes are the state variables.

---

## 10.3 State-Space Equations

### 10.3.1 Standard Form

**State equation:**
$$\dot{\mathbf{x}}(t) = \mathbf{A}\mathbf{x}(t) + \mathbf{B}\mathbf{u}(t)$$

**Output equation:**
$$\mathbf{y}(t) = \mathbf{C}\mathbf{x}(t) + \mathbf{D}\mathbf{u}(t)$$

### 10.3.2 System Matrices

| Matrix | Dimension | Name | Physical Meaning |
|--------|-----------|------|------------------|
| $\mathbf{A}$ | n × n | System matrix | How states affect each other's rates |
| $\mathbf{B}$ | n × m | Input matrix | How inputs drive state changes |
| $\mathbf{C}$ | p × n | Output matrix | Which states we can measure |
| $\mathbf{D}$ | p × m | Feedthrough matrix | Direct input-to-output path |

Where:
- n = number of states
- m = number of inputs
- p = number of outputs

### 10.3.3 Block Diagram

```
                    ┌───────────┐
u(t) ──►[B]──┬──►(+)──►│ Integrator │──►┬──►[C]──►(+)──► y(t)
             │        │   1/s     │   │         │
             │        └───────────┘   │         │
             │              ▲         │         │
             │              │         │        [D]
             └──────────────┴────[A]──┘         │
                                                │
             u(t) ──────────────────────────────┘
```

---

## 10.4 Deriving State-Space Models

### 10.4.1 From Physical Systems

**General approach:**
1. Identify **energy storage elements** (capacitors, inductors, masses, springs)
2. Choose states as variables describing stored energy
3. Write differential equations from physics laws
4. Arrange in matrix form

### 10.4.2 Example: Mass-Spring-Damper

**System:** Mass m, spring k, damper b

**Physical equation:**
$$m\ddot{y} + b\dot{y} + ky = F$$

**State variables:**
- $x_1 = y$ (position)
- $x_2 = \dot{y}$ (velocity)

**Signal Dictionary — Mass-Spring-Damper (State-Space View)**

| Signal | Symbol | Unit | Physical Meaning | Sensor/Actuator |
|--------|--------|------|------------------|-----------------|
| Applied force (input) | $F$ | N | External force on the mass | Electromagnetic actuator |
| Position (state & output) | $x_1 = y$ | m | Displacement from equilibrium — relates to stored spring energy $\frac{1}{2}ky^2$ | LVDT / laser sensor |
| Velocity (state) | $x_2 = \dot{y}$ | m/s | Rate of motion — relates to stored kinetic energy $\frac{1}{2}m\dot{y}^2$ | Differentiated position / accelerometer |
| Spring force | $kx_1$ | N | Restoring force — the $-k/m$ entry in **A** matrix | (Internal force) |
| Damping force | $bx_2$ | N | Dissipative force — the $-b/m$ entry in **A** matrix | (Internal force) |

> **Why state-space?** In transfer-function form, $G(s) = 1/(ms^2 + bs + k)$ hides the internal variables. State-space exposes them: $x_1$ carries the spring energy, $x_2$ carries the kinetic energy. The **A** matrix encodes *how energy flows between states*.

**State-space form:**
$$\begin{bmatrix} \dot{x}_1 \\ \dot{x}_2 \end{bmatrix} = \begin{bmatrix} 0 & 1 \\ -k/m & -b/m \end{bmatrix} \begin{bmatrix} x_1 \\ x_2 \end{bmatrix} + \begin{bmatrix} 0 \\ 1/m \end{bmatrix} F$$

$$y = \begin{bmatrix} 1 & 0 \end{bmatrix} \begin{bmatrix} x_1 \\ x_2 \end{bmatrix}$$

### 10.4.3 Example: RLC Circuit

**Components:** R (resistor), L (inductor), C (capacitor) in series with input voltage $v_{in}$

**State variables:**
- $x_1 = v_C$ (capacitor voltage) - relates to stored electric energy $\frac{1}{2}Cv_C^2$
- $x_2 = i_L$ (inductor current) - relates to stored magnetic energy $\frac{1}{2}Li_L^2$

**Circuit equations:**
- Capacitor: $C\dot{v}_C = i_L$ → $\dot{x}_1 = \frac{1}{C}x_2$
- KVL: $v_{in} = Ri_L + L\dot{i}_L + v_C$ → $\dot{x}_2 = -\frac{1}{L}x_1 - \frac{R}{L}x_2 + \frac{1}{L}v_{in}$

**State-space form:**
$$\begin{bmatrix} \dot{x}_1 \\ \dot{x}_2 \end{bmatrix} = \begin{bmatrix} 0 & \frac{1}{C} \\ -\frac{1}{L} & -\frac{R}{L} \end{bmatrix} \begin{bmatrix} x_1 \\ x_2 \end{bmatrix} + \begin{bmatrix} 0 \\ \frac{1}{L} \end{bmatrix} v_{in}$$

**Output** (capacitor voltage): $y = \begin{bmatrix} 1 & 0 \end{bmatrix} \begin{bmatrix} x_1 \\ x_2 \end{bmatrix}$

---

## 10.5 Canonical Forms

### 10.5.1 Controllable Canonical Form

For transfer function:
$$G(s) = \frac{b_{n-1}s^{n-1} + ... + b_1s + b_0}{s^n + a_{n-1}s^{n-1} + ... + a_1s + a_0}$$

**State-space:**
$$\mathbf{A} = \begin{bmatrix} 0 & 1 & 0 & \cdots & 0 \\ 0 & 0 & 1 & \cdots & 0 \\ \vdots & & & \ddots & \vdots \\ 0 & 0 & 0 & \cdots & 1 \\ -a_0 & -a_1 & -a_2 & \cdots & -a_{n-1} \end{bmatrix}$$

$$\mathbf{B} = \begin{bmatrix} 0 \\ 0 \\ \vdots \\ 0 \\ 1 \end{bmatrix}, \quad \mathbf{C} = \begin{bmatrix} b_0 & b_1 & \cdots & b_{n-1} \end{bmatrix}$$

### 10.5.2 Observable Canonical Form

$$\mathbf{A} = \begin{bmatrix} 0 & 0 & \cdots & 0 & -a_0 \\ 1 & 0 & \cdots & 0 & -a_1 \\ 0 & 1 & \cdots & 0 & -a_2 \\ \vdots & & \ddots & & \vdots \\ 0 & 0 & \cdots & 1 & -a_{n-1} \end{bmatrix}$$

$$\mathbf{B} = \begin{bmatrix} b_0 \\ b_1 \\ \vdots \\ b_{n-1} \end{bmatrix}, \quad \mathbf{C} = \begin{bmatrix} 0 & 0 & \cdots & 0 & 1 \end{bmatrix}$$

> **Important:** These canonical forms assume the transfer function is **strictly proper** (numerator degree < denominator degree), giving $\mathbf{D} = \mathbf{0}$. For a proper transfer function where the degrees are equal ($G(s) = b_n s^n + \cdots$ ÷ $a_n s^n + \cdots$), we first perform polynomial long division to extract the direct feedthrough term $D = b_n/a_n$, then apply these forms to the strictly proper remainder.

### 10.5.3 Diagonal (Modal) Canonical Form

When the system matrix $\mathbf{A}$ has **$n$ distinct eigenvalues** $\lambda_1, \lambda_2, \ldots, \lambda_n$, it can be diagonalised into the modal form:

$$\bar{\mathbf{A}} = \mathbf{T}^{-1}\mathbf{A}\mathbf{T} = \begin{bmatrix} \lambda_1 & 0 & \cdots & 0 \\ 0 & \lambda_2 & \cdots & 0 \\ \vdots & & \ddots & \vdots \\ 0 & 0 & \cdots & \lambda_n \end{bmatrix} = \text{diag}(\lambda_1, \lambda_2, \ldots, \lambda_n)$$

**Transformation matrix:** The similarity transformation is formed from the eigenvectors of $\mathbf{A}$:

$$\mathbf{T} = \begin{bmatrix} \mathbf{v}_1 & \mathbf{v}_2 & \cdots & \mathbf{v}_n \end{bmatrix}$$

where $\mathbf{v}_i$ is the eigenvector corresponding to eigenvalue $\lambda_i$ (i.e., $\mathbf{A}\mathbf{v}_i = \lambda_i \mathbf{v}_i$). This transformation exists if and only if $\mathbf{A}$ has $n$ linearly independent eigenvectors, which is guaranteed when all eigenvalues are distinct.

The transformed input and output matrices become:
$$\bar{\mathbf{B}} = \mathbf{T}^{-1}\mathbf{B}, \quad \bar{\mathbf{C}} = \mathbf{C}\mathbf{T}$$

**Advantages:**
- **Decoupled dynamics:** Each state equation $\dot{z}_i = \lambda_i z_i + \bar{b}_i u$ evolves independently, so each mode is visible and isolated.
- **Easy stability analysis:** The system is stable if and only if $\text{Re}(\lambda_i) < 0$ for all $i$.
- **Controllability/observability check:** The system is controllable iff no row of $\bar{\mathbf{B}}$ is zero, and observable iff no column of $\bar{\mathbf{C}}$ is zero.

> **Note:** When $\mathbf{A}$ has repeated eigenvalues, the diagonal form may not exist. In that case, the closest achievable form is the **Jordan canonical form**, where repeated eigenvalues appear in Jordan blocks with ones on the super-diagonal.

---

## 10.6 Transfer Function from State-Space

### 10.6.1 Derivation

Taking Laplace transform (zero initial conditions):
$$s\mathbf{X}(s) = \mathbf{A}\mathbf{X}(s) + \mathbf{B}\mathbf{U}(s)$$

Solving:
$$\mathbf{X}(s) = (s\mathbf{I} - \mathbf{A})^{-1}\mathbf{B}\mathbf{U}(s)$$

**Transfer function:**
$$\mathbf{G}(s) = \mathbf{C}(s\mathbf{I} - \mathbf{A})^{-1}\mathbf{B} + \mathbf{D}$$

### 10.6.2 Characteristic Equation

$$\det(s\mathbf{I} - \mathbf{A}) = 0$$

**Eigenvalues of A** = Poles of transfer function *(for minimal realizations)*

> **Caveat:** This equality holds only when the state-space realization is **minimal** (both controllable and observable). If the system has uncontrollable or unobservable modes, some eigenvalues of A undergo pole-zero cancellation in $G(s) = C(sI-A)^{-1}B + D$ and do not appear as transfer function poles. Always check controllability and observability before equating eigenvalues with transfer function poles.

---

## 10.7 State Transformation

### 10.7.1 Similarity Transformation

Given state vector $\mathbf{x}$, define new state vector:
$$\mathbf{z} = \mathbf{T}^{-1}\mathbf{x}$$

New state-space matrices:
$$\bar{\mathbf{A}} = \mathbf{T}^{-1}\mathbf{A}\mathbf{T}, \quad \bar{\mathbf{B}} = \mathbf{T}^{-1}\mathbf{B}$$
$$\bar{\mathbf{C}} = \mathbf{C}\mathbf{T}, \quad \bar{\mathbf{D}} = \mathbf{D}$$

### 10.7.2 Invariant Properties

Under similarity transformation:
- **Eigenvalues remain the same**
- **Transfer function unchanged**
- **Controllability/observability properties preserved**

---

## 10.8 Electrical and Telecommunications State-Space Examples

### 10.8.1 Three-Phase Inverter in dq-Frame

**Problem:** Model a three-phase voltage source inverter with LCL filter for grid connection using state-space representation.

**System Description:**

```
┌─────────────────────────────────────────────────────────────────────────┐
│           THREE-PHASE INVERTER STATE-SPACE MODEL                        │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   DC Bus    Inverter      L1 (Inverter-side)   Cf    L2 (Grid-side)    │
│             ┌───┐                                                       │
│   Vdc ──────┤PWM├──┬──/\/\/\──┬──────┬──────/\/\/\──┬── Grid          │
│             └───┘  │    L1    │      │        L2    │   Vg             │
│                    │          │     ═╧═             │                   │
│                  i_inv      v_c     Cf            i_g                  │
│                                                                         │
│   State Variables in dq-frame:                                         │
│   x = [i_d1, i_q1, v_cd, v_cq, i_d2, i_q2]ᵀ                           │
│                                                                         │
│   - i_d1, i_q1: Inverter-side currents (d,q)                          │
│   - v_cd, v_cq: Capacitor voltages (d,q)                              │
│   - i_d2, i_q2: Grid-side currents (d,q)                              │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

**State-Space Equations in dq-Frame:**

The dq-transformation converts AC quantities to DC at steady state, with cross-coupling terms due to the rotating frame.

$$\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}$$

$$\mathbf{A} = \begin{bmatrix}
-R_1/L_1 & \omega & -1/L_1 & 0 & 0 & 0 \\
-\omega & -R_1/L_1 & 0 & -1/L_1 & 0 & 0 \\
1/C_f & 0 & 0 & \omega & -1/C_f & 0 \\
0 & 1/C_f & -\omega & 0 & 0 & -1/C_f \\
0 & 0 & 1/L_2 & 0 & -R_2/L_2 & \omega \\
0 & 0 & 0 & 1/L_2 & -\omega & -R_2/L_2
\end{bmatrix}$$

$$\mathbf{B} = \begin{bmatrix}
1/L_1 & 0 & 0 & 0 \\
0 & 1/L_1 & 0 & 0 \\
0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 \\
0 & 0 & -1/L_2 & 0 \\
0 & 0 & 0 & -1/L_2
\end{bmatrix}, \quad \mathbf{u} = \begin{bmatrix} v_d \\ v_q \\ v_{gd} \\ v_{gq} \end{bmatrix}$$

**CppPlot Implementation:**

```cpp
#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <cmath>
#include <iostream>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    // LCL filter parameters
    double L1 = 3e-3;    // 3 mH inverter inductance
    double L2 = 1e-3;    // 1 mH grid inductance
    double Cf = 10e-6;   // 10 uF filter capacitor
    double R1 = 0.1;     // Parasitic resistance
    double R2 = 0.05;
    double w = 2*M_PI*50; // 50 Hz grid frequency
    
    // State matrix A (6x6) - using cppplot::Matrix
    Matrix A(6, 6);
    A(0,0) = -R1/L1; A(0,1) = w;      A(0,2) = -1/L1; A(0,3) = 0;     A(0,4) = 0;      A(0,5) = 0;
    A(1,0) = -w;      A(1,1) = -R1/L1; A(1,2) = 0;     A(1,3) = -1/L1; A(1,4) = 0;      A(1,5) = 0;
    A(2,0) = 1/Cf;    A(2,1) = 0;      A(2,2) = 0;     A(2,3) = w;     A(2,4) = -1/Cf;  A(2,5) = 0;
    A(3,0) = 0;        A(3,1) = 1/Cf;   A(3,2) = -w;    A(3,3) = 0;     A(3,4) = 0;      A(3,5) = -1/Cf;
    A(4,0) = 0;        A(4,1) = 0;      A(4,2) = 1/L2;  A(4,3) = 0;     A(4,4) = -R2/L2; A(4,5) = w;
    A(5,0) = 0;        A(5,1) = 0;      A(5,2) = 0;     A(5,3) = 1/L2;  A(5,4) = -w;     A(5,5) = -R2/L2;
    
    // Analyze eigenvalues (poles)
    auto poles = A.eigenvalues();
    std::cout << "System Poles:\n";
    for (const auto& p : poles) {
        std::cout << "  " << p.real() << " + " << p.imag() << "j\n";
    }
    
    // Time-domain simulation using Euler method
    double dt = 1e-5;  // 10 us step
    int N = 5000;      // 50 ms simulation
    
    // State vector (6 elements) and input vector (4 elements)
    std::vector<double> x(6, 0.0);
    std::vector<double> u = {0.0, 0.0, 325.0, 0.0};  // Grid voltage only
    
    std::vector<double> time_vec, id2, iq2;
    
    for (int k = 0; k < N; ++k) {
        double t = k * dt;
        
        // Step input: apply inverter voltage at t = 10 ms
        if (t >= 0.01) {
            u[0] = 350;  // vd (slightly above grid for power injection)
            u[1] = 0;    // vq = 0 for unity power factor
        }
        
        // Euler integration: dx = A*x + B*u
        std::vector<double> dx(6, 0.0);
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) dx[i] += A(i,j) * x[j];
        }
        // Add B*u contributions (sparse)
        dx[0] += (1/L1) * u[0];
        dx[1] += (1/L1) * u[1];
        dx[4] += (-1/L2) * u[2];
        dx[5] += (-1/L2) * u[3];
        
        for (int i = 0; i < 6; ++i) x[i] += dx[i] * dt;
        
        // Store results every 10 steps
        if (k % 10 == 0) {
            time_vec.push_back(t * 1000);  // ms
            id2.push_back(x[4]);           // Grid d-current
            iq2.push_back(x[5]);           // Grid q-current
        }
    }
    
    figure(800, 500);
    plot(time_vec, id2, "-", opts({{"color", "blue"}, {"label", "i_d2 (Active current)"}}));
    plot(time_vec, iq2, "--", opts({{"color", "red"}, {"label", "i_q2 (Reactive current)"}}));
    xlabel("Time (ms)");
    ylabel("Current (A)");
    title("Three-Phase Inverter: Grid Current Response");
    legend(true);
    grid(true);
    savefig("ch10_inverter_dq_response.svg");
    
    return 0;
}
```

### 10.8.2 DC Motor with Field Weakening

**Problem:** Model a separately-excited DC motor including field circuit dynamics for field-weakening operation.

**Complete State-Space Model:**

States: $\mathbf{x} = [i_a, \omega, i_f]^T$ (armature current, speed, field current)

$$\frac{di_a}{dt} = \frac{1}{L_a}(V_a - R_a i_a - K_e \phi(\psi_f) \omega)$$
$$\frac{d\omega}{dt} = \frac{1}{J}(K_t \phi(\psi_f) i_a - B\omega - T_L)$$
$$\frac{di_f}{dt} = \frac{1}{L_f}(V_f - R_f i_f)$$

Where field flux $\phi = k_f i_f$ (linear region) or saturated.

**State-Space (Linearized about operating point):**

$$\mathbf{A} = \begin{bmatrix}
-R_a/L_a & -K_e\phi_0/L_a & -K_e\omega_0 k_f/L_a \\
K_t\phi_0/J & -B/J & K_t i_{a0} k_f/J \\
0 & 0 & -R_f/L_f
\end{bmatrix}$$

$$\mathbf{B} = \begin{bmatrix}
1/L_a & 0 & 0 \\
0 & -1/J & 0 \\
0 & 0 & 1/L_f
\end{bmatrix}, \quad \mathbf{u} = \begin{bmatrix} V_a \\ T_L \\ V_f \end{bmatrix}$$

**Why State-Space for Field Weakening?**

- Transfer function approach: Would need separate models for armature and field
- State-space: Captures cross-coupling naturally
- Enables: MIMO control design for coordinated torque/flux control

### 10.8.3 PLL as State-Space System

**Problem:** Represent a second-order PLL in state-space form for Kalman filter-based demodulation.

**PLL State Variables:**

$$\mathbf{x} = \begin{bmatrix} \theta_e \\ \omega_e \end{bmatrix}$$

Where $\theta_e$ is phase error and $\omega_e$ is frequency error.

**Continuous-Time State-Space:**

$$\dot{\mathbf{x}} = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix}\mathbf{x} + \begin{bmatrix} 0 \\ 1 \end{bmatrix}w$$

$$y = \begin{bmatrix} 1 & 0 \end{bmatrix}\mathbf{x} + v$$

Where $w$ is frequency drift (process noise) and $v$ is phase measurement noise.

**Kalman Filter for PLL:**

This state-space formulation enables optimal phase tracking:

$$\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k(y_k - \mathbf{C}\hat{\mathbf{x}}_{k|k-1})$$

The Kalman gain $\mathbf{K}_k$ automatically adapts to noise conditions, providing better jitter performance than fixed-parameter PLLs.

```cpp
#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <random>
#include <cmath>
#include <iostream>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    // PLL Kalman Filter parameters
    double Ts = 1e-6;     // 1 MHz sample rate
    double sigma_w = 10;   // Frequency drift std (rad/s)
    double sigma_v = 0.01; // Phase measurement noise std (rad)
    
    // Discrete state-space matrices (2x2 system)
    // F = [1 Ts; 0 1], G = [0.5*Ts^2; Ts], H = [1 0]
    Matrix F = {{1, Ts}, {0, 1}};
    Matrix G = {{0.5*Ts*Ts}, {Ts}};  // 2x1 column vector
    Matrix H = {{1, 0}};             // 1x2 row vector
    
    // Process and measurement noise covariances
    // Q = G * G^T * sigma_w^2
    Matrix Q = G * G.T() * (sigma_w * sigma_w);
    double R = sigma_v * sigma_v;
    
    // Initial state and covariance
    Matrix x_hat = Matrix::zeros(2, 1);
    Matrix P = Matrix::eye(2);
    
    // Simulate noisy input signal
    std::default_random_engine gen;
    std::normal_distribution<double> noise_w(0, sigma_w);
    std::normal_distribution<double> noise_v(0, sigma_v);
    
    // True state trajectory (frequency step at t=0.5ms)
    std::vector<double> time_us, theta_true_vec, theta_est_vec;
    std::vector<double> omega_true_vec, omega_est_vec;
    
    Matrix x_true = {{0}, {2*M_PI*1000}};  // Initial: 1 kHz offset
    
    for (int k = 0; k < 2000; ++k) {
        double t = k * Ts;
        
        // Frequency step at t = 0.5 ms
        if (k == 500) {
            x_true(1,0) += 2*M_PI*500;  // +500 Hz step
        }
        
        // True state evolution: x_true = F * x_true
        x_true = F * x_true;
        x_true(1,0) += noise_w(gen) * Ts;  // Frequency drift
        
        // Measurement: y = H * x_true + noise
        double y = (H * x_true)(0,0) + noise_v(gen);
        
        // Kalman predict: x_pred = F * x_hat
        Matrix xp = F * x_hat;
        
        // P_pred = F * P * F^T + Q
        Matrix Pp = F * P * F.T() + Q;
        
        // Kalman update: S = H * P_pred * H^T + R
        double S = (H * Pp * H.T())(0,0) + R;
        
        // Kalman gain: K = P_pred * H^T / S
        Matrix Kk = Pp * H.T() * (1.0 / S);
        
        // Innovation: y - H * x_pred
        double innov = y - (H * xp)(0,0);
        x_hat = xp + Kk * innov;
        
        // P = (I - K*H) * P_pred
        P = (Matrix::eye(2) - Kk * H) * Pp;
        
        // Store every 10 samples
        if (k % 10 == 0) {
            time_us.push_back(t * 1e6);
            theta_true_vec.push_back(x_true(0,0));
            theta_est_vec.push_back(x_hat(0,0));
            omega_true_vec.push_back(x_true(1,0) / (2*M_PI));  // Hz
            omega_est_vec.push_back(x_hat(1,0) / (2*M_PI));
        }
    }
    
    figure(800, 600);
    subplot(2, 1, 1);
    plot(time_us, theta_true_vec, "-", opts({{"color", "blue"}, {"label", "True Phase"}}));
    plot(time_us, theta_est_vec, "--", opts({{"color", "red"}, {"label", "Estimated Phase"}}));
    ylabel("Phase (rad)");
    title("Kalman Filter PLL: Phase Tracking");
    legend(true);
    grid(true);
    
    subplot(2, 1, 2);
    plot(time_us, omega_true_vec, "-", opts({{"color", "blue"}, {"label", "True Frequency"}}));
    plot(time_us, omega_est_vec, "--", opts({{"color", "red"}, {"label", "Estimated Frequency"}}));
    xlabel("Time (us)");
    ylabel("Frequency (Hz)");
    legend(true);
    grid(true);
    
    savefig("ch10_kalman_pll.svg");
    return 0;
}
```

### 10.8.4 Summary: State-Space in EE and Telecom

| System | States | Why State-Space? |
|--------|--------|-----------------|
| **3-phase inverter** | Currents, voltages in dq | Captures cross-coupling, enables multivariable control |
| **DC motor w/ field** | ia, ω, if | Coordinated torque/flux control |
| **PLL** | Phase, frequency error | Enables Kalman filtering for optimal tracking |
| **Power converter** | Inductor currents, capacitor voltages | State feedback, observers |

**Key Benefits of State-Space for EE/Telecom:**

1. **MIMO systems:** Natural representation for multi-phase or multi-stage systems
2. **Modern control:** Enables LQR, Kalman filter, MPC
3. **Simulation:** Direct numerical integration
4. **Implementation:** Maps directly to digital controller code

---

## 📝 Exercises

### Exercise 10.1 — Transfer Function to Controllable Canonical Form

Convert the following transfer function to controllable canonical form by hand:

$$G(s) = \frac{2s + 3}{s^3 + 4s^2 + 5s + 2}$$

**(a)** Write the denominator polynomial and read off the controllable canonical form matrices:

$$A_c = \begin{bmatrix} 0 & 1 & 0 \\ 0 & 0 & 1 \\ -a_0 & -a_1 & -a_2 \end{bmatrix}, \quad B_c = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$$

Identify $a_0, a_1, a_2$.

**(b)** From the numerator polynomial, determine $C_c$ and $D$.

**(c)** Verify by computing $C_c(sI - A_c)^{-1}B_c + D$ and simplifying back to $G(s)$.

**(d)** What are the eigenvalues of $A_c$? How do they relate to the poles of $G(s)$?

---

### Exercise 10.2 — State-Space to Transfer Function

Given the state-space model:

$$A = \begin{bmatrix} 0 & 1 \\ -3 & -4 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ 1 \end{bmatrix}, \quad C = \begin{bmatrix} 1 & 2 \end{bmatrix}, \quad D = \begin{bmatrix} 0 \end{bmatrix}$$

**(a)** Compute $(sI - A)$.

**(b)** Find $\det(sI - A)$ — this is the characteristic polynomial.

**(c)** Compute $(sI - A)^{-1}$ using the adjugate formula.

**(d)** Evaluate $G(s) = C(sI - A)^{-1}B + D$ and simplify.

**(e)** Identify the poles and zeros of the resulting transfer function.

---

### Exercise 10.3 — Verification with CppPlot

Use CppPlot to verify your hand calculations from Exercises 9.1 and 9.2:

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;

int main() {
    // Exercise 10.1: TF to SS
    TransferFunction G({2, 3}, {1, 4, 5, 2});
    auto [A1, B1, C1, D1] = tf2ss(G);
    std::cout << "A = " << A1 << std::endl;
    std::cout << "B = " << B1 << std::endl;
    std::cout << "C = " << C1 << std::endl;
    
    // Exercise 10.2: SS to TF
    Matrix A2 = {{0, 1}, {-3, -4}};
    Matrix B2 = {{0}, {1}};
    Matrix C2 = {{1, 2}};
    Matrix D2 = {{0}};
    auto G2 = ss2tf(A2, B2, C2, D2);
    std::cout << "G(s) = " << G2 << std::endl;
}
```

**(a)** Compare the CppPlot output with your hand calculations.

**(b)** Are the state-space matrices from `tf2ss()` in controllable canonical form? If not, what form does CppPlot use?

---

### Exercise 10.4 — Similarity Transformations

Two state-space realizations $(A_1, B_1, C_1, D)$ and $(A_2, B_2, C_2, D)$ represent the same transfer function if they are related by a similarity transformation $T$:

$$A_2 = T^{-1}A_1 T, \quad B_2 = T^{-1}B_1, \quad C_2 = C_1 T$$

**(a)** Using your results from Exercise 10.1, let $(A_1, B_1, C_1)$ be the controllable canonical form. Choose:

$$T = \begin{bmatrix} 1 & 0 & 0 \\ 1 & 1 & 0 \\ 0 & 1 & 1 \end{bmatrix}$$

Compute $(A_2, B_2, C_2)$.

**(b)** Verify that $G(s) = C_2(sI - A_2)^{-1}B_2 + D$ gives the same transfer function.

**(c)** Show that $\det(sI - A_1) = \det(sI - A_2)$, i.e., eigenvalues are invariant under similarity transformation.

---

### Exercise 10.5 — Physical System: Mass-Spring-Damper

A mass-spring-damper system has parameters: $m = 1$ kg, $c = 2$ N·s/m, $k = 5$ N/m.

The equation of motion is:

$$m\ddot{x} + c\dot{x} + kx = F(t)$$

**(a)** Define state variables $x_1 = x$ (position) and $x_2 = \dot{x}$ (velocity). Write the state equations:

$$\dot{\mathbf{x}} = A\mathbf{x} + Bu, \quad y = C\mathbf{x} + Du$$

where $u = F(t)$ and $y = x$ (position output).

**(b)** Write out $A$, $B$, $C$, $D$ explicitly with the given numerical values.

**(c)** Find the eigenvalues of $A$. Are they consistent with the system's natural frequency and damping ratio?

**(d)** Compute the transfer function $G(s) = C(sI - A)^{-1}B$ and verify it matches $G(s) = 1/(ms^2 + cs + k)$.

---

### Exercise 10.6 — State Simulation with CppPlot

Simulate the mass-spring-damper system from Exercise 10.5 using CppPlot:

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;

int main() {
    // Mass-spring-damper: m=1, c=2, k=5
    Matrix A = {{0, 1}, {-5, -2}};
    Matrix B = {{0}, {1}};
    Matrix C = {{1, 0}, {0, 1}};  // Output both states
    Matrix D = {{0}, {0}};
    
    StateSpace sys(A, B, C, D);
    
    // Step input simulation
    figure();
    auto [t, y] = lsim(sys, step_input, 10.0);
    plot(t, y[0], {{"label", "x₁ (position)"}});
    plot(t, y[1], {{"label", "x₂ (velocity)"}});
    xlabel("Time (s)");
    legend();
    title("Mass-Spring-Damper Step Response");
    savefig("mass_spring_damper_step.svg");
}
```

**(a)** Run the simulation and observe both state trajectories.

**(b)** From the plot, estimate the natural frequency $\omega_n$ and damping ratio $\zeta$ from the oscillation period and decay rate.

**(c)** Modify the output matrix $C$ to observe only position. Compare with the transfer function step response.

---

### Exercise 10.7 — Observable Canonical Form

For the transfer function:

$$G(s) = \frac{1}{s^2 + 3s + 2}$$

**(a)** Derive the **controllable canonical form** $(A_c, B_c, C_c, D)$.

**(b)** Derive the **observable canonical form** $(A_o, B_o, C_o, D)$:

$$A_o = \begin{bmatrix} 0 & -a_0 \\ 1 & -a_1 \end{bmatrix}, \quad B_o = \begin{bmatrix} b_0 \\ b_1 \end{bmatrix}, \quad C_o = \begin{bmatrix} 0 & 1 \end{bmatrix}$$

**(c)** Show that $A_o = A_c^T$. Is this always true? Under what conditions?

**(d)** Verify that both forms yield the same transfer function $G(s)$.

**(e)** Find the similarity transformation $T$ such that $A_o = T^{-1}A_c T$.

### Problem Identification Exercises (Level 3-4)

**Exercise 10.8 — What Is the Real Problem?**
An engineer has a transfer function model $G(s) = 1/(s^2 + 3s + 2)$ that works well for controller design. A colleague insists: "You must convert to state-space." The engineer asks: "Why?"

(a) Give two specific situations where a transfer function model is *insufficient* and state-space is required. (Hint: think MIMO, internal stability.)
(b) For the mass-spring-damper system, the transfer function $G(s)$ hides the velocity state $x_2 = \dot{y}$. Explain a practical scenario where knowing $x_2$ matters (e.g., for safety, for control).
(c) A transfer function has a pole-zero cancellation. In state-space, this manifests as a mode that is uncontrollable or unobservable. Why is this dangerous?

**Exercise 10.9 — Mechanism vs. Procedure**
A student converts a 3rd-order transfer function to state-space using controllable canonical form and gets matrices $A$, $B$, $C$, $D$. When asked "what do the states $x_1$, $x_2$, $x_3$ represent physically?", the student cannot answer.

(a) Explain why controllable canonical form states have no direct physical meaning. Compare with the mass-spring-damper example where $x_1 = y$ (position) and $x_2 = \dot{y}$ (velocity).
(b) Is physical meaning of states *required* for controller design? What is lost if the states have no physical interpretation?
(c) Propose a systematic approach: given a canonical form, how can you recover physical states using a similarity transformation?

---

## References


1. Ogata, K. (2010). *Modern Control Engineering*, 5th ed.
2. Chen, C.T. (2013). *Linear System Theory and Design*
3. Teodorescu, R. et al. (2011). *Grid Converters for Photovoltaic and Wind Power Systems*
4. Holmes, D.G. & Lipo, T.A. (2003). *Pulse Width Modulation for Power Converters*
