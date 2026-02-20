# Physical Systems Catalog for Control Engineering
## From Real Systems to Mathematical Models

This document catalogs the physical systems used throughout the textbook, providing a consistent reference from physical principles to mathematical models.

---

## 📋 Systems Overview

| System | Chapter | Order | Key Learning |
|--------|---------|-------|--------------|
| Thermal (Heated Block) | 4 | 1st | Time constant |
| Mass-Spring-Damper | 4, 5 | 2nd | Damping, oscillation |
| RLC Circuit | 4 | 2nd | Analog to mechanical |
| DC Motor Position | 2, 6, 9 | 3rd | Electromechanical |
| Car Suspension | 4, 7, 14 | 2nd | Design trade-offs |
| Inverted Pendulum | 9, 10, 16 | 4th | Unstable systems |
| Ball and Beam | 10, 16 | 4th | Nonlinear, cascade |
| Quadrotor Altitude | 14, 15 | 2nd | Modern control |
| Tank Level Control | 8, 13 | 1st | Industrial, digital |

---

## 🌡️ System 1: Thermal System (Heated Block)

### Physical Description
A metal block with heating element, losing heat to ambient air.

```
         Q(t) (Heater power)
           │
           ▼
    ┌──────────────────┐
    │   Metal Block    │  Temperature: T(t)
    │   Mass: M        │  Ambient: T_amb
    │   Specific Heat: c│
    │   Surface Area: A │
    └────────┬─────────┘
             │ Heat loss: h·A·(T - T_amb)
             ▼
    ═══════════════════  Ambient Environment
```

### Physical Laws
**First Law of Thermodynamics (Energy Balance):**
$$\text{Rate of energy storage} = \text{Heat in} - \text{Heat out}$$
$$Mc\frac{dT}{dt} = Q(t) - hA(T - T_{amb})$$

### Mathematical Model

**Differential Equation:**
$$\tau\frac{d\theta}{dt} + \theta = K \cdot Q(t)$$

where $\theta = T - T_{amb}$ (deviation from ambient)

**Transfer Function:**
$$G(s) = \frac{\Theta(s)}{Q(s)} = \frac{K}{\tau s + 1}$$

**Parameters:**
| Symbol | Expression | Physical Meaning |
|--------|------------|------------------|
| $\tau$ | $\frac{Mc}{hA}$ | Thermal time constant |
| $K$ | $\frac{1}{hA}$ | Thermal resistance |

### CppPlot Implementation

```cpp
// Physical parameters
double M = 2.0;      // mass [kg]
double c = 900;      // specific heat [J/(kg·K)] - aluminum
double h = 25;       // convection coefficient [W/(m²·K)]
double A = 0.02;     // surface area [m²]

// Derived parameters
double tau = M * c / (h * A);     // time constant [s]
double K = 1.0 / (h * A);          // gain [K/W]

// Transfer function
TransferFunction G_thermal({K}, {tau, 1});

// Example: 50W heater step response
auto [t, temp] = step(G_thermal * 50.0, 5*tau);
```

### Typical Values
| Parameter | Aluminum Block | Water Tank | Building |
|-----------|---------------|------------|----------|
| τ | 60 min | 2-4 hours | 10-50 hours |
| K | 2°C/W | 0.1°C/W | 0.05°C/W |

### Chapter Usage
- **Chapter 4:** First-order response, time constant concept
- **Chapter 8:** PID temperature control design
- **Chapter 13:** Digital temperature controller

---

## ⚙️ System 2: Mass-Spring-Damper

### Physical Description
A mass attached to a spring and damper, subjected to external force.

```
     F(t) applied force
       │
       ▼
    ┌──────┐
    │  M   │────▶ x(t) displacement
    └──┬───┘
       │
    ═══╪═══  Spring (k) - stores energy
       │
    ───╫───  Damper (b) - dissipates energy
       │
    ───┴───  Fixed reference
```

### Physical Laws
**Newton's Second Law:**
$$\sum F = Ma$$
$$F(t) - kx - b\dot{x} = M\ddot{x}$$

### Mathematical Model

**Differential Equation:**
$$M\ddot{x} + b\dot{x} + kx = F(t)$$

**Transfer Function:**
$$G(s) = \frac{X(s)}{F(s)} = \frac{1}{Ms^2 + bs + k} = \frac{1/k}{\frac{M}{k}s^2 + \frac{b}{k}s + 1}$$

**Standard Form:**
$$G(s) = \frac{\omega_n^2/k}{s^2 + 2\zeta\omega_n s + \omega_n^2}$$

**Parameters:**
| Symbol | Expression | Physical Meaning |
|--------|------------|------------------|
| $\omega_n$ | $\sqrt{k/M}$ | Natural frequency |
| $\zeta$ | $\frac{b}{2\sqrt{kM}}$ | Damping ratio |

### State-Space Form

$$\dot{\mathbf{x}} = \begin{bmatrix} 0 & 1 \\ -k/M & -b/M \end{bmatrix} \mathbf{x} + \begin{bmatrix} 0 \\ 1/M \end{bmatrix} F$$

$$y = \begin{bmatrix} 1 & 0 \end{bmatrix} \mathbf{x}$$

### CppPlot Implementation

```cpp
// Physical parameters
double M = 1.0;    // mass [kg]
double b = 0.5;    // damping [N·s/m]
double k = 4.0;    // spring constant [N/m]

// Derived parameters
double wn = sqrt(k / M);           // 2 rad/s
double zeta = b / (2 * sqrt(k*M)); // 0.125 (underdamped)

// Transfer function
TransferFunction G_msd({1}, {M, b, k});

// State-space form
Matrix A = {{0, 1}, {-k/M, -b/M}};
Matrix B = {{0}, {1/M}};
Matrix C = {{1, 0}};
Matrix D = {{0}};
StateSpace sys_msd(A, B, C, D);
```

### Design Guidelines
| Damping Ratio | Response | Application |
|---------------|----------|-------------|
| ζ < 0.3 | Too oscillatory | Avoid |
| ζ = 0.5 | Some overshoot | Fast response needed |
| ζ = 0.707 | ~5% overshoot | Optimal balance |
| ζ = 1.0 | No overshoot | Critical damping |
| ζ > 1.5 | Sluggish | Overdamped |

### Chapter Usage
- **Chapter 4:** Second-order response, damping effects
- **Chapter 5:** Root locus with varying parameters
- **Chapter 9:** State-space modeling
- **Chapter 10:** State feedback design

---

## 🔌 System 3: RLC Circuit

### Physical Description
Series RLC circuit with voltage source input and capacitor voltage output.

```
    Vin ──┬──[R]──[L]──┬── Vout
          │            │
          │           ═╧═ C
          │            │
          └────────────┘
```

### Physical Laws
**Kirchhoff's Voltage Law:**
$$V_{in} = V_R + V_L + V_C$$
$$V_{in} = Ri + L\frac{di}{dt} + \frac{1}{C}\int i \, dt$$

### Mathematical Model

**Transfer Function (Capacitor Voltage):**
$$G(s) = \frac{V_{out}(s)}{V_{in}(s)} = \frac{1/LC}{s^2 + \frac{R}{L}s + \frac{1}{LC}}$$

**Analogy to Mechanical System:**
| Electrical | Mechanical | Role |
|------------|------------|------|
| L (inductance) | M (mass) | Inertia |
| R (resistance) | b (damping) | Energy dissipation |
| 1/C (inverse capacitance) | k (spring) | Energy storage |
| i (current) | v (velocity) | Flow variable |
| V (voltage) | F (force) | Effort variable |

### CppPlot Implementation

```cpp
// Physical parameters
double R = 10;      // resistance [Ω]
double L = 0.1;     // inductance [H]
double C = 100e-6;  // capacitance [F]

// Derived parameters
double wn = 1 / sqrt(L*C);          // 316.2 rad/s
double zeta = R / (2 * sqrt(L/C));  // 0.158

// Transfer function
TransferFunction G_RLC({1/(L*C)}, {1, R/L, 1/(L*C)});

// Verify resonance frequency
std::cout << "Resonance: " << wn/(2*M_PI) << " Hz" << std::endl;
```

### Chapter Usage
- **Chapter 2:** Electrical system modeling
- **Chapter 4:** Second-order systems (parallel to mechanical)
- **Chapter 6:** Frequency response, resonance

---

## 🔧 System 4: DC Motor (Position Control)

### Physical Description
Armature-controlled DC motor with load.

```
                 ┌─────────────────┐
     V(t) ──────▶│    DC Motor     │──────▶ θ(t) position
   (Voltage)     │   Ra, La, J, b  │      (Rotation)
                 │   Kt, Kb        │
                 └─────────────────┘
```

### Physical Laws

**Electrical Subsystem (Kirchhoff):**
$$V = L_a\frac{di_a}{dt} + R_a i_a + e_b$$
$$e_b = K_b \omega \quad \text{(back-EMF)}$$

**Mechanical Subsystem (Newton for rotation):**
$$J\frac{d\omega}{dt} + b\omega = \tau_m = K_t i_a$$
$$\theta = \int \omega \, dt$$

### Mathematical Model

**Transfer Function (θ/V):**
$$G(s) = \frac{\Theta(s)}{V(s)} = \frac{K_t}{s[(L_a s + R_a)(Js + b) + K_t K_b]}$$

**Simplified (La ≈ 0):**
$$G(s) = \frac{K_m}{s(\tau_m s + 1)}$$

where:
- $K_m = \frac{K_t}{R_a b + K_t K_b}$ (motor gain)
- $\tau_m = \frac{J R_a}{R_a b + K_t K_b}$ (mechanical time constant)

### CppPlot Implementation

```cpp
// Physical parameters
double Ra = 2.0;   // armature resistance [Ω]
double La = 0.01;  // armature inductance [H] (often negligible)
double J = 0.01;   // moment of inertia [kg·m²]
double b = 0.001;  // friction coefficient [N·m·s]
double Kt = 0.1;   // torque constant [N·m/A]
double Kb = 0.1;   // back-EMF constant [V·s/rad]

// Simplified model (La ≈ 0)
double Km = Kt / (Ra*b + Kt*Kb);
double tau_m = J*Ra / (Ra*b + Kt*Kb);

// Transfer function: θ(s)/V(s)
TransferFunction G_motor({Km}, {tau_m, 1, 0});  // Km/(s(τm·s + 1))

// Full third-order model
TransferFunction G_motor_full({Kt}, 
    {La*J, La*b + Ra*J, Ra*b + Kt*Kb, 0});
```

### Chapter Usage
- **Chapter 2:** Electromechanical modeling
- **Chapter 5, 7:** Compensator design
- **Chapter 9:** State-space model (3 states: θ, ω, i)
- **Chapter 10:** State feedback position control

---

## 🚗 System 5: Car Suspension (Quarter-Car Model)

### Physical Description
Simplified model of one wheel with suspension.

```
         z_s (sprung mass position)
           │
    ┌──────┴──────┐
    │  Body Mass  │  M_s (sprung mass)
    │     M_s     │
    └──────┬──────┘
           │
        ═══╪═══  Suspension spring (k_s)
           │
        ───╫───  Shock absorber (b_s)
           │
    ┌──────┴──────┐
    │  Wheel Mass │  M_u (unsprung mass)
    │     M_u     │
    └──────┬──────┘
           │
        ═══╪═══  Tire stiffness (k_t)
           │
    ───────┴───────  Road input z_r(t)
```

### Physical Laws

**Newton's Law for each mass:**

Sprung mass: $M_s \ddot{z}_s = -k_s(z_s - z_u) - b_s(\dot{z}_s - \dot{z}_u)$

Unsprung mass: $M_u \ddot{z}_u = k_s(z_s - z_u) + b_s(\dot{z}_s - \dot{z}_u) - k_t(z_u - z_r)$

### CppPlot Implementation

```cpp
// Physical parameters (typical sedan)
double Ms = 400;    // sprung mass (1/4 car body) [kg]
double Mu = 40;     // unsprung mass (wheel assembly) [kg]
double ks = 18000;  // suspension spring [N/m]
double bs = 1500;   // shock absorber [N·s/m]
double kt = 180000; // tire stiffness [N/m]

// State: [z_s, dz_s, z_u, dz_u]
Matrix A = {
    {0, 1, 0, 0},
    {-ks/Ms, -bs/Ms, ks/Ms, bs/Ms},
    {0, 0, 0, 1},
    {ks/Mu, bs/Mu, -(ks+kt)/Mu, -bs/Mu}
};
Matrix B = {{0}, {0}, {0}, {kt/Mu}};
Matrix C = {{1, 0, -1, 0}};  // suspension deflection output
Matrix D = {{0}};

StateSpace suspension(A, B, C, D);
```

### Design Trade-offs

| Parameter | ↑ Increase | Effect on Ride | Effect on Handling |
|-----------|------------|----------------|-------------------|
| ks | Stiffer spring | Harsher | Better |
| bs | More damping | Less oscillation | Better |

### Chapter Usage
- **Chapter 4:** Second-order approximation design
- **Chapter 7:** Frequency response, vibration isolation
- **Chapter 11:** Observer design (estimate road profile)
- **Chapter 14:** Active suspension (LQR design)

---

## 🎢 System 6: Inverted Pendulum on Cart

### Physical Description
A classic nonlinear control problem - balancing a pole on a moving cart.

```
                    θ (angle from vertical)
                   ╱
                  ╱ L (pendulum length)
                 ╱
    ┌───────────●───────────┐
    │         pivot         │  M (cart mass)
    │        (m at tip)     │  m (pendulum mass)
    └───────────────────────┘
              │
              ▼ F(t) (applied force)
    ════════════════════════════  Track
              x (cart position)
```

### Physical Laws (Lagrangian Mechanics)

**Lagrangian:** $\mathcal{L} = T - V$ (kinetic - potential energy)

$$T = \frac{1}{2}M\dot{x}^2 + \frac{1}{2}m(\dot{x}_p^2 + \dot{y}_p^2)$$
$$V = mgy_p = mgL\cos\theta$$

where $x_p = x + L\sin\theta$, $y_p = L\cos\theta$

### Linearized Model (around θ = 0)

**Transfer Function (θ/F):**
$$G(s) = \frac{\Theta(s)}{F(s)} = \frac{-1/L}{(M+m)s^2 - \frac{(M+m)g}{L}}$$

This has a pole in the RHP → **inherently unstable!**

### State-Space Model

State: $\mathbf{x} = [x, \dot{x}, \theta, \dot{\theta}]^T$

$$A = \begin{bmatrix} 
0 & 1 & 0 & 0 \\
0 & 0 & -mg/M & 0 \\
0 & 0 & 0 & 1 \\
0 & 0 & (M+m)g/(ML) & 0
\end{bmatrix}$$

### CppPlot Implementation

```cpp
// Physical parameters
double M = 1.0;    // cart mass [kg]
double m = 0.1;    // pendulum mass [kg]
double L = 0.5;    // pendulum length [m]
double g = 9.81;   // gravity [m/s²]

// Linearized state-space model
Matrix A = {
    {0, 1, 0, 0},
    {0, 0, -m*g/M, 0},
    {0, 0, 0, 1},
    {0, 0, (M+m)*g/(M*L), 0}
};
Matrix B = {{0}, {1/M}, {0}, {-1/(M*L)}};
Matrix C = {{1, 0, 0, 0}, {0, 0, 1, 0}};  // x and θ outputs
Matrix D = {{0}, {0}};

StateSpace invpend(A, B, C, D);

// Check stability (should have unstable pole)
auto poles = invpend.poles();
// Expect: two imaginary, two real (one positive)
```

### Chapter Usage
- **Chapter 9:** State-space modeling of unstable systems
- **Chapter 10:** Pole placement for stabilization
- **Chapter 11:** Observer for angle estimation
- **Chapter 14:** LQR for swing-up and balance
- **Chapter 16:** Nonlinear control, feedback linearization

---

## 🚁 System 7: Quadrotor Altitude Control

### Physical Description
Simplified 1-DOF model for vertical (altitude) control.

```
         ┌─────┬─────┐
    ═════╪═════╪═════╪═════  Propellers (total thrust T)
         │     │     │
         └─────┼─────┘
               │
               │  z (altitude)
               │
               ● m (mass)
               │
               ▼ mg (gravity)
```

### Physical Laws
**Newton's Second Law (vertical):**
$$m\ddot{z} = T - mg - D$$

where $D = k_d \dot{z}$ is aerodynamic drag.

**Linearized around hover:** At hover, $T_0 = mg$

Let $u = T - T_0$ (thrust deviation), then:
$$m\ddot{z} = u - k_d\dot{z}$$

### Transfer Function
$$G(s) = \frac{Z(s)}{U(s)} = \frac{1/m}{s^2 + \frac{k_d}{m}s}$$

### CppPlot Implementation

```cpp
// Physical parameters
double m = 1.5;     // mass [kg]
double g = 9.81;    // gravity [m/s²]
double kd = 0.1;    // drag coefficient [N·s/m]

// Transfer function
TransferFunction G_quad({1/m}, {1, kd/m, 0});

// State-space: state = [z, dz]
Matrix A = {{0, 1}, {0, -kd/m}};
Matrix B = {{0}, {1/m}};
Matrix C = {{1, 0}};
Matrix D = {{0}};

StateSpace quad_alt(A, B, C, D);
```

### Chapter Usage
- **Chapter 12:** Observer-based control (estimate altitude from noisy GPS)
- **Chapter 14:** LQR altitude control
- **Chapter 15:** Kalman filter for sensor fusion

---

## 🏭 System 8: Tank Level Control

### Physical Description
Industrial liquid level control system.

```
    Qin(t) ──▶ ┌─────────────┐
   (Inflow)   │             │
              │    ▓▓▓▓▓    │ ← h(t) (liquid level)
              │    ▓▓▓▓▓    │
              │    ▓▓▓▓▓    │   A = cross-sectional area
              │    ▓▓▓▓▓    │
              └──────┬──────┘
                     │
                     ▼ Qout = h/R (outflow through valve)
```

### Physical Laws
**Mass Balance:**
$$A\frac{dh}{dt} = Q_{in} - Q_{out}$$

With linear valve: $Q_{out} = h/R$ (where R is valve resistance)

### Mathematical Model

**Differential Equation:**
$$AR\frac{dh}{dt} + h = R \cdot Q_{in}$$

**Transfer Function:**
$$G(s) = \frac{H(s)}{Q_{in}(s)} = \frac{R}{\tau s + 1}$$

where $\tau = AR$ is the hydraulic time constant.

### CppPlot Implementation

```cpp
// Physical parameters
double A = 2.0;     // tank area [m²]
double R = 100;     // valve resistance [s/m²]

// Derived parameters
double tau = A * R;  // time constant [s]
double K = R;        // DC gain [m/(m³/s)]

// Transfer function
TransferFunction G_tank({K}, {tau, 1});
```

### Chapter Usage
- **Chapter 4:** First-order response
- **Chapter 8:** PI level control
- **Chapter 13:** Digital level controller, sampling effects

---

## Summary: Physical System Selection Guide

| Teaching Goal | Recommended System | Why |
|---------------|-------------------|-----|
| First-order basics | Thermal, Tank | Simple, intuitive |
| Second-order, damping | Mass-Spring-Damper | Clear physical meaning |
| Frequency response | RLC Circuit | Resonance visible |
| Electromechanical | DC Motor | Industry-relevant |
| Design trade-offs | Car Suspension | Practical decisions |
| Stabilization | Inverted Pendulum | Unstable, challenging |
| Modern control | Quadrotor | Current technology |
| Industrial control | Tank Level | Process control |

---

## Code Library: Physical Systems

All systems are available in the textbook code repository:

```cpp
#include <cppplot/control/physical_systems.hpp>

// Ready-to-use system models
auto thermal = PhysicalSystems::heatedBlock(M, c, h, A);
auto msd = PhysicalSystems::massSpringDamper(M, b, k);
auto motor = PhysicalSystems::dcMotor(Ra, La, J, b, Kt, Kb);
auto suspension = PhysicalSystems::quarterCar(Ms, Mu, ks, bs, kt);
auto pendulum = PhysicalSystems::invertedPendulum(M, m, L);
auto quadrotor = PhysicalSystems::quadrotorAltitude(m, kd);
auto tank = PhysicalSystems::tankLevel(A, R);
```
