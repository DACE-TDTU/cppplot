# Chapter 17: Introduction to Nonlinear Control

---

## 🎯 Learning Objectives and Outcomes

### Chapter Objectives

This chapter introduces nonlinear systems analysis, recognizing that all real systems are inherently nonlinear. Students learn when linear approximations fail and gain tools for analyzing stability and behavior of nonlinear systems.

### Prerequisites
- Chapters 1-11: Linear control theory foundation
- Differential equations
- Basic understanding of phase portraits

---

## Why This Chapter Matters: The Linear Lie

> **The Real Engineering Problem:** You've learned 15 chapters of linear control theory. But here's the truth: **NO real system is actually linear.** So when does the linear approximation work, and when does it fail spectacularly?

### Where Nonlinearities Hide in Every System

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    NONLINEARITIES IN REAL SYSTEMS                           │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   YOUR LINEAR MODEL:  G(s) = K/(τs + 1)                                    │
│   REALITY:            Everything is nonlinear!                              │
│                                                                             │
│   ACTUATOR NONLINEARITIES:                                                 │
│   ─────────────────────────                                                │
│   • Saturation: Motor voltage limited to ±24V                              │
│     └─ Linear region only for small control signals                        │
│   • Dead zone: PWM below 5% duty cycle produces no motion                  │
│   • Slew rate: Hydraulic valve can't move faster than 10 mm/s             │
│   • Backlash: Gears have 0.5° of play                                     │
│                                                                             │
│   PLANT NONLINEARITIES:                                                    │
│   ──────────────────────                                                   │
│   • Pendulum: τ = mgl sin(θ), not mgl·θ for large angles                  │
│   • Aerodynamics: Drag ∝ v², not v                                        │
│   • Thermal: Radiation ∝ T⁴, not T                                        │
│   • Magnetic: Inductance varies with current (saturation)                  │
│   • Chemical: Reaction rate = k·e^(-Ea/RT), highly nonlinear!             │
│                                                                             │
│   SENSOR NONLINEARITIES:                                                   │
│   ───────────────────────                                                  │
│   • Quantization: 12-bit ADC has 4096 levels                              │
│   • Deadband: Encoder has minimum detectable motion                        │
│   • Saturation: Sensor output clips at ±10V                               │
│   • Hysteresis: Magnetic sensors have B-H curve                           │
│                                                                             │
│   FRICTION (The Nemesis of Precision):                                     │
│   ─────────────────────────────────────                                    │
│   • Static friction (stiction) > kinetic friction                         │
│   • Velocity-dependent: Coulomb, viscous, Stribeck                        │
│   • Direction-dependent: Different μ for + vs - velocity                  │
│   • Temperature-dependent: Cold start vs warm operation                    │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### When Linear Control Fails: Real Examples

| Situation | Linear Expectation | Nonlinear Reality |
|-----------|-------------------|-------------------|
| **High gain PID** | Faster response | Actuator saturates, integrator windup |
| **Tracking large steps** | Smooth transition | Slew limiting, overshoot |
| **Low-speed motion** | Precise positioning | Stick-slip from friction |
| **Pendulum swing-up** | Works near vertical | Needs nonlinear strategy for full swing |
| **Aggressive maneuvers** | Scaled version of small motion | Aerodynamic nonlinearity dominates |

### A Concrete Example: Position Control with Friction

```cpp
// What you THINK happens (linear model):
// τ = J·α + B·ω
// Smooth motion proportional to control signal

// What ACTUALLY happens (with friction):
double compute_friction(double omega, double omega_prev) {
    const double Fc = 0.5;    // Coulomb friction [Nm]
    const double Fs = 0.8;    // Static friction [Nm]
    const double Fv = 0.01;   // Viscous friction [Nm·s/rad]
    const double vs = 0.1;    // Stribeck velocity [rad/s]
    
    if (std::abs(omega) < 1e-6 && std::abs(omega_prev) < 1e-6) {
        // Stiction regime - friction matches applied torque up to Fs
        return 0;  // Return 0, but actual friction = applied torque
    } else {
        // Moving - Stribeck friction model
        double sign_omega = (omega >= 0) ? 1.0 : -1.0;
        return sign_omega * (Fc + (Fs - Fc) * exp(-pow(omega/vs, 2))) 
               + Fv * omega;
    }
}

// Consequence: Your linear controller will have:
// • Limit cycles around setpoint (constant oscillation)
// • Position-dependent steady-state error
// • Different behavior for + vs - direction
```

### The Chapter's Purpose

This chapter teaches you to:
1. **Recognize** when linear control will fail
2. **Analyze** nonlinear systems using phase portraits and Lyapunov theory
3. **Design** controllers that handle nonlinearities (gain scheduling, feedback linearization)
4. **Implement** practical fixes (anti-windup, friction compensation)

---

## Learning Outcomes

After completing this chapter, students will be able to:

| Bloom's Level | Outcome |
|---------------|---------|
| **Remember** | Define nonlinear system, equilibrium point, limit cycle |
| **Understand** | Explain why nonlinear systems require special analysis |
| **Apply** | Linearize nonlinear systems around equilibrium points |
| **Analyze** | Analyze stability using phase plane and Lyapunov methods |
| **Evaluate** | Evaluate applicability of linear control to nonlinear plants |
| **Create** | Apply basic nonlinear control techniques |

---

## 17.1 Introduction to Nonlinear Systems

### 17.1.1 What Makes a System Nonlinear?

**Linear system:** $\dot{x} = ax + bu$ (constant coefficients)

**Nonlinear system:** Any system that is not linear
- $\dot{x} = x^2 + u$ (polynomial)
- $\dot{x} = \sin(x) + u$ (trigonometric)
- $\dot{x} = x + |u|$ (absolute value)
- $\dot{x} = x + \text{sat}(u)$ (saturation)

### 17.1.2 Why Nonlinear Control?

**All real systems are nonlinear!**

Linear control works when:
- Operating near equilibrium
- Deviations are small
- Linearization is valid

Nonlinear control needed when:
- Large operating range
- Strong nonlinearities
- Performance demands exceed linear capabilities

### 17.1.3 Common Nonlinearities

| Type | Mathematical Form | Physical Example | Why It Matters |
|------|-------------------|------------------|----------------|
| Saturation | $\text{sat}(u)$ | Actuator limits | Integrator windup |
| Dead zone | $\text{dz}(u)$ | Gear backlash | Limit cycles |
| Friction | $F(\dot{x})$ | Stiction, Coulomb | Position error |
| Product | $x_1 \cdot x_2$ | Power = voltage × current | Bilinear systems |
| Trigonometric | $\sin(\theta)$ | Pendulum | Multiple equilibria |
| Square | $x^2$ | Aerodynamic drag | Velocity-dependent behavior |

---

## 17.2 Nonlinear Phenomena

### 17.2.1 Multiple Equilibria

Linear systems: One equilibrium (or infinite if marginally stable)

Nonlinear systems: May have multiple equilibria

**Example:** Pendulum $\ddot{\theta} + \sin\theta = 0$
- Equilibria at $\theta = 0, \pm\pi, \pm 2\pi, ...$
- $\theta = 0$ is stable, $\theta = \pm\pi$ is unstable

### 17.2.2 Limit Cycles

**Limit cycle:** Isolated periodic orbit in state space

- Not possible in linear systems
- Can be stable (attractive) or unstable
- **Example:** Van der Pol oscillator

### 17.2.3 Finite Escape Time

Solutions can go to infinity in finite time.

**Example:** $\dot{x} = x^2$, $x(0) = 1$
- Solution: $x(t) = \frac{1}{1-t}$
- Escapes to infinity at $t = 1$

### 17.2.4 Chaos

Deterministic systems with sensitive dependence on initial conditions.

- Bounded but non-periodic
- **Example:** Lorenz system

---

## 17.3 Equilibrium Points

### 17.3.1 Definition

For system $\dot{\mathbf{x}} = \mathbf{f}(\mathbf{x}, \mathbf{u})$:

> **Equilibrium point:** State $\mathbf{x}_e$ where $\mathbf{f}(\mathbf{x}_e, \mathbf{u}_e) = \mathbf{0}$

System remains at $\mathbf{x}_e$ if started there (with constant input $\mathbf{u}_e$).

### 17.3.2 Finding Equilibrium Points

1. Set $\dot{\mathbf{x}} = \mathbf{0}$
2. Solve resulting algebraic equations
3. May have 0, 1, or multiple solutions

### 17.3.3 Example: Pendulum

$$\dot{\theta} = \omega$$
$$\dot{\omega} = -\frac{g}{l}\sin\theta - \frac{b}{m}\omega + \frac{1}{ml^2}\tau$$

Equilibria (with $\tau = 0$):
- $\omega = 0$, $\sin\theta = 0$ → $\theta = n\pi$

---

## 17.4 Linearization

### 17.4.1 Jacobian Linearization

For nonlinear system $\dot{\mathbf{x}} = \mathbf{f}(\mathbf{x}, \mathbf{u})$ around equilibrium $(\mathbf{x}_e, \mathbf{u}_e)$:

$$\mathbf{A} = \frac{\partial \mathbf{f}}{\partial \mathbf{x}}\bigg|_{(\mathbf{x}_e, \mathbf{u}_e)}, \quad \mathbf{B} = \frac{\partial \mathbf{f}}{\partial \mathbf{u}}\bigg|_{(\mathbf{x}_e, \mathbf{u}_e)}$$

Linear approximation:
$$\delta\dot{\mathbf{x}} = \mathbf{A}\delta\mathbf{x} + \mathbf{B}\delta\mathbf{u}$$

where $\delta\mathbf{x} = \mathbf{x} - \mathbf{x}_e$, $\delta\mathbf{u} = \mathbf{u} - \mathbf{u}_e$

### 17.4.2 Validity of Linearization

**Hartman-Grobman Theorem:**
Near a **hyperbolic** equilibrium (no eigenvalues on imaginary axis), the nonlinear system behaves qualitatively like its linearization.

**Limitation:** Only valid for small perturbations.

### 17.4.3 Example: Pendulum Linearization

Around $\theta = 0$ (down position):
- $\sin\theta \approx \theta$

Around $\theta = \pi$ (up position):
- Let $\phi = \theta - \pi$
- $\sin\theta = -\sin\phi \approx -\phi$

---

## 17.5 Phase Plane Analysis

### 17.5.1 Phase Plane

For second-order system:
$$\dot{x}_1 = f_1(x_1, x_2)$$
$$\dot{x}_2 = f_2(x_1, x_2)$$

**Phase plane:** Plot of $x_2$ vs $x_1$ showing trajectories.

### 17.5.2 Phase Portrait

Collection of representative trajectories showing system behavior.

### 17.5.3 Equilibrium Classification

Based on linearization eigenvalues:

| Eigenvalues | Equilibrium Type |
|-------------|------------------|
| Both real, negative | Stable node |
| Both real, positive | Unstable node |
| Real, opposite signs | Saddle point |
| Complex, negative real | Stable focus (spiral) |
| Complex, positive real | Unstable focus |
| Pure imaginary | Center (linear only) |

### 17.5.4 Isoclines

**Isocline:** Curve where $\frac{dx_2}{dx_1}$ has constant slope.

Useful for sketching phase portraits.

---

## 17.6 Lyapunov Stability

### 17.6.1 Stability Definitions

**Stable:** Trajectories starting near equilibrium stay near it.

**Asymptotically stable:** Stable AND trajectories converge to equilibrium.

**Globally asymptotically stable:** Asymptotically stable from any initial condition.

**Unstable:** Not stable.

### 17.6.2 Lyapunov's Direct Method

> **Theorem (Lyapunov Stability):**
> Given system $\dot{\mathbf{x}} = \mathbf{f}(\mathbf{x})$ with equilibrium at the origin.
> 
> If there exists a continuously differentiable scalar function $V(\mathbf{x})$ such that:
> 1. $V(\mathbf{0}) = 0$
> 2. $V(\mathbf{x}) > 0$ for $\mathbf{x} \neq \mathbf{0}$ (positive definite)
> 3. $\dot{V}(\mathbf{x}) \leq 0$ (negative semi-definite)
> 
> Then the equilibrium is **stable** (in the sense of Lyapunov).
> 
> If additionally $\dot{V}(\mathbf{x}) < 0$ for $\mathbf{x} \neq \mathbf{0}$ (negative definite), then **locally asymptotically stable**.
>
> If additionally $V(\mathbf{x}) \to \infty$ as $\|\mathbf{x}\| \to \infty$ (**radially unbounded**), then **globally asymptotically stable**.

> **⚠️ Radial Unboundedness:** Without the condition $V(\mathbf{x}) \to \infty$ as $\|\mathbf{x}\| \to \infty$, asymptotic stability is only **local**. For example, $V(x) = \frac{x^2}{1+x^2}$ is positive definite but bounded — trajectories could escape to infinity without $V$ increasing. Always verify radial unboundedness when claiming **global** asymptotic stability.

### 17.6.3 LaSalle's Invariance Principle

When $\dot{V} \leq 0$ but **not** strictly negative definite, Lyapunov's theorem only guarantees stability, not asymptotic stability. **LaSalle's invariance principle** bridges this gap:

> **Theorem (LaSalle):**
> Let $\Omega$ be a compact, positively invariant set for $\dot{\mathbf{x}} = \mathbf{f}(\mathbf{x})$. Let $V(\mathbf{x})$ be continuously differentiable with $\dot{V}(\mathbf{x}) \leq 0$ in $\Omega$. Define:
> $$E = \{\mathbf{x} \in \Omega : \dot{V}(\mathbf{x}) = 0\}$$
> Let $M$ be the largest invariant set contained in $E$. Then every trajectory starting in $\Omega$ converges to $M$.

**Application Example — Damped Pendulum:**

$$V(\theta, \dot{\theta}) = \frac{1}{2}ml^2\dot{\theta}^2 + mgl(1 - \cos\theta)$$

$$\dot{V} = -b\dot{\theta}^2 \leq 0$$

$\dot{V} = 0$ only when $\dot{\theta} = 0$. In $E = \{\dot{\theta} = 0\}$, the only invariant set requires $\ddot{\theta} = 0$ → $\sin\theta = 0$ → $\theta = 0$. By LaSalle, the origin is **asymptotically stable** even though $\dot{V}$ is only negative *semi*-definite.

> **Practical Importance:** LaSalle's principle is the standard tool for proving asymptotic stability of mechanical systems with friction/damping, where the Lyapunov function (total energy) has $\dot{V} \leq 0$ but not $\dot{V} < 0$ everywhere.

### 17.6.4 Lyapunov Function

$V(\mathbf{x})$ is called a **Lyapunov function**.

**Physical interpretation:** Energy-like function that decreases along trajectories.

### 17.6.5 Common Lyapunov Functions

**Quadratic:**
$$V(\mathbf{x}) = \mathbf{x}'\mathbf{P}\mathbf{x}$$

For linear systems, P from Lyapunov equation:
$$\mathbf{A}'\mathbf{P} + \mathbf{P}\mathbf{A} = -\mathbf{Q}$$

---

## 17.7 Describing Functions

### 17.7.1 Describing Function Method

For a nonlinear element $f(\cdot)$ preceded by a linear block, if the linear part acts as a low-pass filter, the output is approximately sinusoidal. The **describing function** is the fundamental harmonic gain:

$$N(A) = \frac{b_1}{A}, \quad b_1 = \frac{1}{\pi}\int_0^{2\pi} f(A\sin\theta)\sin\theta\,d\theta$$

(for memoryless symmetric nonlinearities where $a_1 = 0$)

### 17.7.2 Common Describing Functions

| Nonlinearity | $N(A)$ |
|-------------|--------|
| Saturation ($M$) | $\frac{2}{\pi}\left[\arcsin\frac{M}{A} + \frac{M}{A}\sqrt{1-(M/A)^2}\right]$, $A > M$ |
| Dead zone ($\delta$) | $\frac{2}{\pi}\left[\arccos\frac{\delta}{A} - \frac{\delta}{A}\sqrt{1-(\delta/A)^2}\right]$ |
| Ideal relay | $\frac{4M}{\pi A}$ |
| Relay with hysteresis ($h$) | $\frac{4M}{\pi A}\sqrt{1-(h/A)^2} - j\frac{4Mh}{\pi A^2}$ |

### 17.7.3 Limit Cycle Prediction

A limit cycle exists where the Nyquist plot of $G(j\omega)$ intersects $-1/N(A)$:
$$G(j\omega) = -\frac{1}{N(A)}$$
This gives two equations (real and imaginary parts) for two unknowns ($\omega$ and $A$).

**Stability of limit cycles:** If $-1/N(A)$ moves toward the Nyquist plot as A increases → unstable limit cycle. If away → stable limit cycle.

### 17.7.4 Worked Example

**Relay ($M=1$) with $G(s) = 1/[s(s+1)(s+2)]$:**
- $N(A) = 4/(\pi A)$
- $-1/N(A) = -\pi A/4$ (negative real line)
- At $\omega = \sqrt{2}$: $G(j\omega)$ crosses the negative real axis
- $|G(j\sqrt{2})| = 1/6$, so $A = 4/(6\pi) \approx 0.212$

---

## 17.8 Basic Nonlinear Control Techniques

### 17.8.1 Gain Scheduling

Use different linear controllers for different operating regions.

**Implementation:**
1. Linearize at multiple operating points
2. Design controller for each
3. Interpolate or switch based on operating condition

### 17.8.2 Feedback Linearization

Transform nonlinear system to linear through state feedback.

**Example:** For $\dot{x} = f(x) + g(x)u$, if $g(x) \neq 0$:

$$u = \frac{1}{g(x)}(v - f(x))$$

Results in $\dot{x} = v$ (linear!)

**Limitation:** Requires accurate model, may need full state.

### 17.8.3 Relative Degree and Normal Form

For a SISO system $\dot{x} = f(x) + g(x)u$, $y = h(x)$, the **relative degree** $r$ is the number of times you differentiate $y$ before $u$ appears explicitly:
$$y^{(r)} = L_f^r h(x) + L_g L_f^{r-1} h(x) \cdot u$$

where $L_f h = \nabla h \cdot f$ is the Lie derivative.

If $r < n$ (state dimension), the system can be transformed into **normal form**:
$$\dot{\eta} = q(\eta, \xi) \quad \text{(zero dynamics — } n - r \text{ internal states)}$$
$$\dot{\xi}_1 = \xi_2, \quad \dot{\xi}_2 = \xi_3, \quad \ldots, \quad \dot{\xi}_r = v$$

**Zero dynamics** are the internal dynamics when the output is kept identically zero ($y \equiv 0$).

**Critical result:** Feedback linearization yields a stable closed-loop **only if the zero dynamics are stable** (minimum phase). For non-minimum phase systems, attempting feedback linearization creates internal instability.

### 17.8.4 Sliding Mode Control (Overview)

- Design a "sliding surface" in state space
- High-speed switching to reach and stay on surface
- Robust to matched uncertainties
- Causes chattering (high-frequency switching)

> **📘 See Chapter 17b** for comprehensive treatment of Sliding Mode Control including:
> - Mathematical foundations and Lyapunov stability proofs
> - Super-Twisting Algorithm (STA)
> - Fixed-Time SMC with guaranteed settling time bounds
> - Event-Triggered SMC for resource efficiency
> - Barrier Function SMC for state constraints
> - Disturbance Observer-based SMC (DOBSMC)
> - Complete design procedures and implementation examples

---

## 17.9 Passivity and Energy-Based Methods

### 17.9.1 Definition

A system with input $u$ and output $y$ is **passive** if there exists a storage function $V(x) \geq 0$ (energy) such that:
$$\dot{V} \leq u^T y$$

This is the **dissipation inequality** — the system can only store energy that flows in through the power port $u^T y$.

### 17.9.2 Special Cases

- **Lossless:** $\dot{V} = u^T y$ (no dissipation)
- **Strictly output passive:** $\dot{V} \leq u^T y - \epsilon y^T y$ for some $\epsilon > 0$
- **Input strictly passive:** $\dot{V} \leq u^T y - \delta u^T u$

### 17.9.3 Passivity Theorem

The negative feedback interconnection of two passive systems is stable. If at least one is strictly passive → asymptotically stable.

**Connection to Lyapunov:** The storage function serves as a Lyapunov function for the interconnected system.

### 17.9.4 Example — RLC Circuit

An RLC network with input voltage $u = V_{in}$ and output current $y = i$ is passive with storage function $V = \frac{1}{2}Li^2 + \frac{1}{2}Cv_C^2$ (sum of magnetic and electric energies). The dissipation is $Ri^2 \geq 0$.

**Application — Power electronics:** Passivity-based control of DC-DC converters: design the controller to inject damping while preserving the passive structure, ensuring global stability without linearization.

---

## 17.10 Applications in Electrical and Communication Systems

### 17.10.1 PLL Nonlinear Analysis

The phase detector characteristic is inherently nonlinear (sin or sawtooth). Analyzing pull-in range and lock-in range requires phase-plane analysis of the nonlinear PLL model:
$$\ddot{\theta} + 2\zeta\omega_n\dot{\theta} + \omega_n^2\sin(\theta) = 0$$

### 17.10.2 Power Amplifier Predistortion

AM/AM and AM/PM nonlinearities in RF power amplifiers. Describing function analysis to predict spectral regrowth and intermodulation. Digital predistortion as feedback linearization.

### 17.10.3 Switching Converter Modeling

Large-signal averaged models of DC-DC converters are bilinear ($\dot{x} = (A_1 d + A_2(1-d))x + Bu$). State-dependent switching creates limit cycles. Sliding mode control (Ch17b) provides robust regulation.

---

## 17.11 Example: Inverted Pendulum Phase Plane

### 17.11.1 System

$$\ddot{\theta} = \frac{g}{l}\sin\theta - \frac{b}{ml^2}\dot{\theta}$$

### 17.11.2 Analysis

**State variables:** $x_1 = \theta$, $x_2 = \dot{\theta}$

**Equilibrium points** (with $\ddot{\theta} = +\frac{g}{l}\sin\theta$, so $\theta = 0$ is the **upright** position):
- $(0, 0)$: Pendulum **up** (inverted) — **saddle point** (unstable)
- $(\pm\pi, 0)$: Pendulum **down** (hanging) — **stable** node/focus

**Linearization at $(0, 0)$** (upright equilibrium)**:**
$$\mathbf{A} = \begin{bmatrix} 0 & 1 \\ \frac{g}{l} & -\frac{b}{ml^2} \end{bmatrix}$$

Eigenvalues have opposite signs → **saddle point** (unstable)

**Phase Portrait Features:**
1. **Separatrices:** Trajectories that divide the phase plane into regions
2. **Homoclinic orbits:** Trajectories connecting saddle point to itself
3. **Periodic orbits:** Closed curves around the stable equilibrium (for low damping)

See **ch17_nonlinear_control.cpp** for complete implementation.

---

## 17.12 Computational Example: Phase Portrait with cppplot

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;

int main() {
    // Phase portrait: Van der Pol oscillator
    // ẍ - μ(1-x²)ẋ + x = 0, μ = 1
    double mu = 1.0;
    
    figure(800, 800);
    // Plot trajectories from multiple initial conditions
    for (double x0 = -4.0; x0 <= 4.0; x0 += 1.0) {
        for (double v0 = -4.0; v0 <= 4.0; v0 += 1.0) {
            std::vector<double> xs, vs;
            double x = x0, v = v0, dt = 0.01;
            for (int i = 0; i < 5000; ++i) {
                xs.push_back(x);
                vs.push_back(v);
                double dx = v;
                double dv = mu * (1 - x*x) * v - x;
                x += dx * dt;
                v += dv * dt;
            }
            plot(xs, vs, "b-");
        }
    }
    xlabel("x");
    ylabel("dx/dt");
    title("Van der Pol Oscillator Phase Portrait (μ = 1)");
    grid(true);
    savefig("van_der_pol.svg");
}
```

This example integrates the Van der Pol oscillator from a grid of initial conditions using forward Euler, plotting all trajectories to reveal the stable limit cycle that attracts all non-equilibrium solutions.

---

### 📋 Signal Dictionary — Nonlinear Pendulum with Feedback Linearization

| Signal | Symbol | Meaning | Typical unit |
|--------|--------|---------|--------------|
| State vector | $\mathbf{x} = [\theta, \dot{\theta}]^T$ | Angle and angular velocity | [rad, rad/s] |
| Equilibrium point | $\mathbf{x}_e$ | Where $f(\mathbf{x}_e, u_e) = 0$ — may be multiple! | [rad, rad/s] |
| Nonlinear dynamics | $\dot{\mathbf{x}} = f(\mathbf{x}, u)$ | True system — no approximation | — |
| Jacobian | $A = \partial f/\partial x \big|_{x_e}$ | Local linear approximation at equilibrium | — |
| Region of attraction | $\mathcal{R}$ | Set of initial conditions that converge to $\mathbf{x}_e$ | state space |
| Lyapunov function | $V(\mathbf{x})$ | "Energy-like" scalar: $V > 0$, $\dot{V} < 0$ ⇒ stable | J (or —) |
| Lyapunov derivative | $\dot{V} = \nabla V \cdot f(\mathbf{x})$ | Rate of energy dissipation along trajectories | J/s |
| Feedback linearizing input | $u = \alpha(\mathbf{x}) + \beta(\mathbf{x}) v$ | Cancels nonlinearity; $v$ = new linear input | N·m |
| Virtual input | $v$ | Design variable in the linearized coordinates | N·m |
| Limit cycle | — | Isolated periodic orbit — trajectories spiral toward (stable) or away (unstable) | — |
| Describing function | $N(A)$ | Amplitude-dependent "gain" of a nonlinearity; predicts limit cycles | — |
| Phase portrait | — | Plot of trajectories in $(x_1, x_2)$ plane — reveals global behavior | — |

> **Key insight:** Linearization ($A = \partial f/\partial x$) is valid only *near* the equilibrium. The Lyapunov approach gives *global* or *regional* stability guarantees without linearization — but requires finding a suitable $V(\mathbf{x})$, which is an art rather than an algorithm.

---

## 17.13 Exercises

**E16.1 (Equilibrium Points and Linearization)**
Consider the nonlinear system:
$$\dot{x}_1 = x_2$$
$$\dot{x}_2 = -\sin(x_1) - 0.5 x_2$$

(a) Find all equilibrium points by setting $\dot{x}_1 = \dot{x}_2 = 0$.

(b) Compute the Jacobian $\mathbf{A} = \frac{\partial \mathbf{f}}{\partial \mathbf{x}}$ evaluated at each equilibrium.

(c) Classify each equilibrium (stable node, saddle, stable/unstable focus, etc.) based on the eigenvalues of the linearized system.

(d) Sketch the phase portrait near each equilibrium.

---

**E16.2 (Lyapunov Stability Analysis)**
For the system in E16.1, consider the equilibrium at the origin $(0, 0)$.

(a) Propose the energy-like Lyapunov function:
$$V(x_1, x_2) = (1 - \cos x_1) + \frac{1}{2}x_2^2$$

(b) Show that $V > 0$ for $(x_1, x_2) \neq (0, 0)$ in a neighborhood of the origin.

(c) Compute $\dot{V}$ and show that $\dot{V} \leq 0$.

(d) Use LaSalle's invariance principle to conclude asymptotic stability.

(e) What is the region of attraction? Is the origin globally asymptotically stable?

---

**E16.3 (Describing Function Analysis)**
A relay nonlinearity (output $= +M$ for positive input, $-M$ for negative input, with $M = 1$) is in feedback with the linear system:
$$G(s) = \frac{1}{s(s+1)(s+2)}$$

(a) Compute the describing function $N(A) = \frac{4M}{\pi A}$ for the ideal relay.

(b) Find the intersection of $-1/N(A)$ with the Nyquist plot of $G(j\omega)$ to predict the limit cycle amplitude and frequency.

(c) Is the predicted limit cycle stable or unstable? Explain using the graphical criterion.

---

**E16.4 (Phase Portrait Analysis)**
Analyze the system $\ddot{x} + x^3 = 0$ (Duffing equation without damping or forcing).

(a) Write in state-space form: $\dot{x}_1 = x_2$, $\dot{x}_2 = -x_1^3$.

(b) Find the equilibrium and linearize. What type of equilibrium does the linearized system predict?

(c) Show that $V = \frac{1}{2}x_2^2 + \frac{1}{4}x_1^4$ is a Lyapunov function with $\dot{V} = 0$. What does this say about stability?

(d) Sketch the phase portrait. Are the trajectories closed curves? Is the origin stable, asymptotically stable, or unstable?

---

**E16.5 (LaSalle's Invariance Principle)**
Apply LaSalle's invariance principle to the system:
$$\ddot{x} + \dot{x}^3 + x = 0$$

with Lyapunov function $V = \frac{1}{2}(\dot{x}^2 + x^2)$.

(a) Compute $\dot{V}$ and show it is $\leq 0$.

(b) Identify the set $E = \{(x, \dot{x}) : \dot{V} = 0\}$.

(c) Find the largest invariant set $M \subseteq E$.

(d) Conclude about the asymptotic stability of the origin.

---

**E16.6 (Region of Attraction)**
For the scalar system $\dot{x} = -x + x^3$:

(a) Find the equilibrium points and determine their stability by linearization.

(b) Using the Lyapunov function $V = \frac{1}{2}x^2$, compute $\dot{V}$.

(c) Find the largest level set $\{x : V(x) \leq c\}$ on which $\dot{V} < 0$. This estimates the region of attraction.

(d) Compare your estimate with the exact region of attraction $|x| < 1$.

---

**E16.7 (Challenge: Feedback Linearization and the Inverted Pendulum)**
Consider the inverted pendulum:
$$\ddot{\theta} = \frac{g}{l}\sin\theta + \frac{1}{ml^2}u$$

(a) Apply input-output feedback linearization: choose $u$ so that $\ddot{\theta} = v$ for a new input $v$.

(b) Show that the linearizing control law requires knowledge of the full state $(\theta, \dot{\theta})$.

(c) Design a linear controller for the resulting linear system $\ddot{\theta} = v$ to stabilize the upright position.

(d) What happens if there is a model error in $g/l$ or $1/(ml^2)$? Is feedback linearization robust? Discuss.

---

**E16.8 🔴 (Level 3 — Region of Attraction Matters)**
An inverted pendulum has two equilibria: $\theta = 0$ (upright, unstable) and $\theta = \pi$ (hanging, stable).

(a) Linearize at $\theta = 0$ and design a stabilizing linear controller (e.g., LQR). What are the closed-loop poles?

(b) Simulate the nonlinear system with initial conditions $\theta_0 = 10°, 30°, 60°, 90°$. For which $\theta_0$ does the linear controller fail? This defines the boundary of the region of attraction.

(c) Find a Lyapunov function $V(\theta, \dot{\theta})$ (Hint: use total energy) and estimate the region of attraction analytically. Compare with your simulation.

(d) A student says "the poles are in the LHP, so the system is stable for all initial conditions." Explain precisely what is wrong with this statement.

**E16.9 🔴 (Level 3 — Limit Cycles and Describing Functions)**
A relay-controlled system has plant $G(s) = \frac{1}{s(s+1)(s+2)}$ with a relay nonlinearity of amplitude $\pm 1$.

(a) Compute the describing function $N(A) = \frac{4}{\pi A}$ for an ideal relay.

(b) Find the intersection of $-1/N(A)$ with $G(j\omega)$ to predict the limit cycle amplitude and frequency.

(c) Simulate the full nonlinear system and compare the actual limit cycle with the describing function prediction. How accurate is it?

(d) If you replace the relay with a saturation nonlinearity (smooth), does the limit cycle persist, change, or disappear? Explain the mechanism.

**E16.10 ⚫ (Level 4 — The Linearization Trap)**

(a) Linearization says: if the Jacobian eigenvalues have negative real parts, the equilibrium is locally asymptotically stable. What happens if one eigenvalue has zero real part? Can linearization decide stability in this case? What theorem applies?

(b) A system has a stable equilibrium at the origin AND a stable limit cycle. A Lyapunov function $V(x)$ proves the origin is stable. How is this consistent with the limit cycle? (Hint: what is the domain of $\dot{V} < 0$?)

(c) "All real systems are nonlinear; linear control is just an approximation." If this is true, why does linear control work so well in practice for most systems? Write a reasoned argument connecting this to the model hierarchy of Ch. 2 (§2.8.7).

---

## 17.14 Summary

| Concept | Key Point |
|---------|----------|
| Nonlinear phenomena | Multiple equilibria, limit cycles, chaos |
| Equilibrium | $f(x_e, u_e) = 0$ |
| Linearization | Jacobian at equilibrium |
| Phase plane | Trajectory visualization for 2D systems |
| Lyapunov | Stability via energy-like functions |
| Control techniques | Gain scheduling, feedback linearization |

---

## References


1. Khalil, H.K. (2002). *Nonlinear Systems*, 3rd ed.
2. Slotine, J.J.E. & Li, W. (1991). *Applied Nonlinear Control*
