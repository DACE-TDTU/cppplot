# Chapter 13: State Feedback Control

---

## 🎯 Learning Objectives and Outcomes

### Chapter Objectives

This chapter introduces state feedback control design, teaching how to arbitrarily place closed-loop poles through full-state feedback when the system is controllable.

### Prerequisites
- Chapter 11: State-Space Analysis (controllability)
- Chapter 4: Time-Domain specifications

---

## Learning Outcomes

After completing this chapter, students will be able to:

| Bloom's Level | Outcome |
|---------------|---------|
| **Remember** | Define state feedback and pole placement |
| **Understand** | Explain how state feedback changes system dynamics |
| **Apply** | Apply pole placement and Ackermann's formula |
| **Analyze** | Analyze closed-loop stability and performance |
| **Evaluate** | Evaluate trade-offs in pole placement design |
| **Create** | Design state feedback controllers for given specifications |

---

## Why This Chapter Matters

> **The Real Problem:** Your self-balancing robot falls over in 0.3 seconds. You have four measurements: cart position, cart velocity, pendulum angle, and angular velocity. A single PID loop on the angle isn't enough — the system has *four* states that interact. Can you compute a *single* control force that simultaneously stabilizes all four states? And if so — where should the closed-loop poles be?
>
> This chapter answers both questions. State feedback gives you direct control over *every* eigenvalue of the closed-loop system — something impossible in the transfer function framework. The price: you must measure (or estimate) the full state vector.

> **From Chapter 10 (§10.1.3):** If you tried to stabilize a quadrotor or a wheeled mobile robot by tuning PID gains one-at-a-time, you discovered the coupling problem — adjusting one axis disturbs another, and 48-parameter manual search never converges. This chapter provides the *systematic* answer: state feedback with pole placement computes the entire gain matrix $K$ in one step, and LQR (Chapter 15) does so optimally. **The mathematical model is not optional decoration — it is the computational engine that replaces weeks of futile trial-and-error with a single matrix equation.**

---

## 12.1 Introduction to State Feedback

### 12.1.1 Concept

**State feedback** uses all state variables to compute the control input:

$$\mathbf{u} = -\mathbf{K}\mathbf{x} + \mathbf{r}$$

where:
- $\mathbf{K}$: State feedback gain matrix (m × n)
- $\mathbf{r}$: Reference input

### 12.1.2 Closed-Loop System

Substituting into state equation:
$$\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}(-\mathbf{K}\mathbf{x} + \mathbf{r})$$
$$\dot{\mathbf{x}} = (\mathbf{A} - \mathbf{B}\mathbf{K})\mathbf{x} + \mathbf{B}\mathbf{r}$$

**Closed-loop system matrix:**
$$\mathbf{A}_{cl} = \mathbf{A} - \mathbf{B}\mathbf{K}$$

### 12.1.3 Block Diagram

```
         r(t)          u(t)           ┌─────────────┐
    ──────►(+)────────────────────────►│   Plant     │───┬──► y(t)
            -▲                        │ ẋ = Ax + Bu │   │
             │                        │  y = Cx     │   │
             │                        └─────────────┘   │
             │                                          │
             │              x(t)                        │
             └───────[K]◄───────────────────────────────┘
```

---

## 12.2 Pole Placement

### 12.2.1 Fundamental Theorem

> **Theorem (Pole Placement):**
> If the system (A, B) is controllable, then for any desired set of closed-loop poles, there exists a state feedback gain K such that the eigenvalues of (A - BK) are exactly those desired poles.

### 12.2.2 Design Approach

1. **Specify desired poles** based on performance requirements
2. **Check controllability** of (A, B)
3. **Compute K** using pole placement algorithm
4. **Verify** closed-loop eigenvalues

### 12.2.3 Choosing Desired Poles

| Requirement | Pole Selection |
|-------------|----------------|
| Settling time $t_s$ | $\text{Re}(\lambda) \approx -4/t_s$ |
| Overshoot | $\zeta \geq 0.5$ (complex poles) |
| Bandwidth | Related to natural frequency |
| Actuator limits | Not too far left (high gains) |

**Rule of thumb:** Place dominant poles for desired response; place others 3-5× faster.

---

## 12.3 Ackermann's Formula

### 12.3.1 For SISO Systems

For single-input systems, the feedback gain is:

$$\mathbf{K} = \begin{bmatrix} 0 & 0 & \cdots & 0 & 1 \end{bmatrix} \mathcal{C}^{-1} \phi_d(\mathbf{A})$$

where:
- $\mathcal{C}$ is the controllability matrix
- $\phi_d(s) = (s - p_1)(s - p_2)...(s - p_n)$ is desired characteristic polynomial
- $\phi_d(\mathbf{A})$ is computed by substituting matrix A

### 12.3.2 Example

**System:**
$$\mathbf{A} = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix}, \quad \mathbf{B} = \begin{bmatrix} 0 \\ 1 \end{bmatrix}$$

**Desired poles:** $p_1 = -2, p_2 = -3$

**Step 1:** Desired characteristic polynomial
$$\phi_d(s) = (s+2)(s+3) = s^2 + 5s + 6$$

**Step 2:** Calculate $\phi_d(\mathbf{A})$
$$\phi_d(\mathbf{A}) = \mathbf{A}^2 + 5\mathbf{A} + 6\mathbf{I} = \begin{bmatrix} 6 & 5 \\ 0 & 6 \end{bmatrix}$$

**Step 3:** Controllability matrix
$$\mathcal{C} = \begin{bmatrix} 0 & 1 \\ 1 & 0 \end{bmatrix}$$

**Step 4:** Apply Ackermann's formula
$$\mathbf{K} = \begin{bmatrix} 0 & 1 \end{bmatrix} \begin{bmatrix} 0 & 1 \\ 1 & 0 \end{bmatrix}^{-1} \begin{bmatrix} 6 & 5 \\ 0 & 6 \end{bmatrix}$$
$$\mathbf{K} = \begin{bmatrix} 6 & 5 \end{bmatrix}$$

---

## 12.4 Reference Tracking

### 12.4.1 Problem Statement

With state feedback alone, the steady-state output may not equal the reference.

### 12.4.2 Feedforward Gain

Add scaling factor $\bar{N}$:
$$u = -\mathbf{K}\mathbf{x} + \bar{N}r$$

**Computation:**
$$\bar{N} = \frac{1}{\mathbf{C}(\mathbf{B}\mathbf{K} - \mathbf{A})^{-1}\mathbf{B}}$$

Or equivalently:
$$\begin{bmatrix} \mathbf{A} & \mathbf{B} \\ \mathbf{C} & 0 \end{bmatrix} \begin{bmatrix} N_x \\ N_u \end{bmatrix} = \begin{bmatrix} \mathbf{0} \\ 1 \end{bmatrix}$$

$$\bar{N} = N_u + \mathbf{K}N_x$$

### 12.4.3 Integral Action

For robust tracking and disturbance rejection, add **integral state**:

$$\dot{x}_I = r - y$$

Augmented system:
$$\begin{bmatrix} \dot{\mathbf{x}} \\ \dot{x}_I \end{bmatrix} = \begin{bmatrix} \mathbf{A} & \mathbf{0} \\ -\mathbf{C} & 0 \end{bmatrix} \begin{bmatrix} \mathbf{x} \\ x_I \end{bmatrix} + \begin{bmatrix} \mathbf{B} \\ 0 \end{bmatrix} u + \begin{bmatrix} \mathbf{0} \\ 1 \end{bmatrix} r$$

Design feedback for augmented system.

---

## 12.5 Design Considerations

### 12.5.1 Control Effort

Moving poles far left requires large gains → large control effort

**Trade-off:** Fast response vs. actuator saturation

### 12.5.2 Robustness

State feedback can be sensitive to:
- Model uncertainty
- Parameter variations
- Unmodeled dynamics

### 12.5.3 State Availability

**Key assumption:** All states are measurable.

In practice, often need **state observer** (Chapter 13).

---

## 12.6 Multi-Input Systems

### 12.6.1 Non-Uniqueness

For MIMO systems, many K matrices give same poles.

### 12.6.2 Design Methods

- **Sequential design:** Place poles one input at a time
- **Eigenstructure assignment:** Specify eigenvectors too
- **Optimization:** LQR approach (Chapter 15)

---

## 12.7 Example: Inverted Pendulum

### 12.7.1 System Model

For the inverted pendulum on a cart (pendulum up = $\theta = 0$), the linearized model is:

$$\mathbf{A} = \begin{bmatrix} 0 & 1 & 0 & 0 \\ 0 & 0 & -\frac{mg}{M} & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 0 & \frac{(M+m)g}{Ml} & 0 \end{bmatrix}$$

$$\mathbf{B} = \begin{bmatrix} 0 \\ \frac{1}{M} \\ 0 \\ -\frac{1}{Ml} \end{bmatrix}$$

**States:** $x_1$ = cart position, $x_2$ = cart velocity, $x_3$ = pendulum angle (from vertical, positive counterclockwise), $x_4$ = angular velocity

**Derivation:** From the Lagrangian with small-angle linearization ($\sin\theta \approx \theta$):
- Cart: $M\ddot{x} = F - mg\theta$ → $A_{23} = -mg/M$ (cart acceleration *opposes* tilt)
- Pendulum: $Ml\ddot{\theta} = (M+m)g\theta - F$ → $A_{43} = +(M+m)g/(Ml)$ (gravity destabilizes)
- Force on pendulum: $B_4 = -1/(Ml)$ (pushing cart right rotates pendulum left)

**Signal Dictionary — Inverted Pendulum on Cart**

| Signal | Symbol | Unit | Physical Meaning | Sensor/Actuator |
|--------|--------|------|------------------|-----------------|
| Cart position (state) | $x_1 = x$ | m | Horizontal position of the cart | Linear encoder / potentiometer |
| Cart velocity (state) | $x_2 = \dot{x}$ | m/s | Rate of cart travel — determines kinetic energy of cart | Differentiated encoder |
| Pendulum angle (state) | $x_3 = \theta$ | rad | Tilt from vertical — small $\theta$ means near-upright | Rotary encoder / IMU |
| Angular velocity (state) | $x_4 = \dot{\theta}$ | rad/s | Rate of tilt — determines how fast the pendulum is falling | Differentiated encoder / gyroscope |
| Horizontal force (control) | $F$ | N | Force applied to the cart by a motor/belt | DC motor + belt drive |
| Gravity torque | $(M+m)g\theta$ | N·m | Destabilizing torque — source of the RHP pole ($A_{43} > 0$) | (Physics — cannot be actuated) |

> **Reading the A matrix through physics:** $A_{43} > 0$ is the *only* positive entry — it produces the unstable eigenvalue (RHP pole). This single number encodes the fact that gravity pulls the pendulum away from vertical. The controller must generate enough force $F$ to counteract this before the pendulum falls.

> **Note:** Only $A_{43} > 0$ produces the unstable eigenvalue (RHP pole). The negative $A_{23}$ means the cart is pulled toward the tilt direction, not pushed away.

### 12.7.2 Design Goals

- Stabilize unstable equilibrium (pendulum up)
- Reasonable settling time
- Limited cart travel

### 12.7.3 Implementation

See **ch12_state_feedback.cpp** for complete implementation with CppPlot visualization.

---

## 12.8 Real-World Application: Quadcopter Attitude Control

> **Practical Integration:** A quadcopter is an excellent example of state feedback in an integrated mechatronic system, combining mechanical dynamics, brushless motors, electronic speed controllers, IMU sensors, and embedded software.

### 12.8.1 System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                   QUADCOPTER CONTROL SYSTEM                             │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐              │
│  │   Pilot /   │     │   State     │     │    PWM      │              │
│  │   GPS Cmd   │────►│  Feedback   │────►│   Signals   │              │
│  │   (r)       │     │  Controller │     │   (to ESC)  │              │
│  └─────────────┘     │  u = -Kx+Nr │     └──────┬──────┘              │
│                      └──────▲──────┘            │                      │
│                             │                   ▼                      │
│                      ┌──────┴──────┐     ┌─────────────┐              │
│                      │   State     │     │  4 × BLDC   │              │
│                      │  Estimator  │     │   Motors    │              │
│                      │   (EKF)     │     │   + ESCs    │              │
│                      └──────▲──────┘     └──────┬──────┘              │
│                             │                   │                      │
│                      ┌──────┴──────┐     ┌──────▼──────┐              │
│                      │    IMU      │     │  Airframe   │              │
│                      │ Accel+Gyro  │◄────│  Dynamics   │              │
│                      │ + Mag + Baro│     │  (6 DOF)    │              │
│                      └─────────────┘     └─────────────┘              │
└─────────────────────────────────────────────────────────────────────────┘
```

### 12.8.2 Attitude Dynamics (Simplified)

For small angles, roll ($\phi$), pitch ($\theta$), yaw ($\psi$) dynamics decouple:

**Roll Channel:**
$$\ddot{\phi} = \frac{\tau_\phi}{I_{xx}} = \frac{l \cdot k_F}{I_{xx}}(\omega_2^2 - \omega_4^2)$$

**State-Space Model (Roll):**

$$\mathbf{x} = \begin{bmatrix} \phi \\ \dot{\phi} \end{bmatrix}, \quad
\mathbf{A} = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix}, \quad
\mathbf{B} = \begin{bmatrix} 0 \\ \frac{l \cdot k_F}{I_{xx}} \end{bmatrix}$$

Input: $u = \omega_2^2 - \omega_4^2$ (differential motor speed squared)

### 12.8.3 State Feedback Design

**Design Specifications:**
- Settling time: < 0.5 s (for agile maneuvering)
- Overshoot: < 5% (for stability)
- Maximum motor command: ±2000 RPM² differential

**Desired Poles:** $s_{1,2} = -8 \pm j8$ (ζ = 0.707, ω_n = 11.3 rad/s)

**Pole Placement:**
With $I_{xx} = 0.015$ kg·m², $l = 0.23$ m, $k_F = 6.11 \times 10^{-8}$ N/(rad/s)²:

The effective input gain is $b = l \cdot k_F / I_{xx} = 0.23 \times 6.11 \times 10^{-8} / 0.015 = 9.37 \times 10^{-7}$.

Desired characteristic polynomial: $s^2 + 16s + 128 = 0$ (from poles $-8 \pm j8$).

$$k_1 = \frac{128}{b} = \frac{128}{9.37 \times 10^{-7}} \approx 1.37 \times 10^8, \quad k_2 = \frac{16}{b} = \frac{16}{9.37 \times 10^{-7}} \approx 1.71 \times 10^7$$

$$\mathbf{K} = \begin{bmatrix} 1.37 \times 10^8 & 1.71 \times 10^7 \end{bmatrix}$$

> **⚠️ Practical Note:** These gains are extremely large because $k_F$ is very small. In practice, the thrust-to-torque relationship should use the *linearized* force coefficient around an operating point, or the model should be normalized.

### 12.8.4 Multi-Domain Integration

| Domain | Component | Role in Control |
|--------|-----------|-----------------|
| **Mechanical** | Airframe, propellers | Plant dynamics (I_xx, I_yy, I_zz) |
| **Electrical** | BLDC motors | Torque generation (~1ms response) |
| **Electronic** | ESC (32-bit MCU) | Motor speed control (8-32 kHz PWM) |
| **Sensors** | IMU (MPU6050/BMI088) | State measurement (1000 Hz) |
| **Software** | EKF + State FB | Sensor fusion + control law |
| **Communication** | SBUS/PPM/MAVLink | Pilot commands + telemetry |

### 12.8.5 Implementation Considerations

> **Practical Challenges:**
> 
> 1. **State Estimation:** Cannot directly measure angles — must fuse accelerometer (noisy, no drift) with gyroscope (smooth, drifts)
> 2. **Motor Dynamics:** ESC + motor has ~10ms lag → include in model
> 3. **Coupling:** Roll/pitch/yaw couple at high angles → full nonlinear model
> 4. **Saturation:** Motor speed has limits → anti-windup needed
> 5. **Battery Voltage:** Affects motor gain → feedforward compensation

**Control Loop Rates:**

| Loop | Rate | Implemented On |
|------|------|----------------|
| Motor current | 32 kHz | ESC (hardware) |
| Attitude | 500 Hz | Flight controller |
| Position | 50 Hz | Flight controller |
| Mission | 10 Hz | Companion computer |

---

## 12.9 Electrical and Telecommunications State Feedback Examples

### 12.9.1 Grid-Connected Inverter Current Control

**Problem:** Design state feedback controller for LCL-filtered inverter current control.

**System Model (dq-frame, single-phase equivalent):**

States: $\mathbf{x} = [i_1, v_c, i_2]^T$ (inverter current, capacitor voltage, grid current)

$$\mathbf{A} = \begin{bmatrix}
-R_1/L_1 & -1/L_1 & 0 \\
1/C_f & 0 & -1/C_f \\
0 & 1/L_2 & -R_2/L_2
\end{bmatrix}, \quad
\mathbf{B} = \begin{bmatrix} 1/L_1 \\ 0 \\ 0 \end{bmatrix}$$

Output (grid current): $\mathbf{C} = [0, 0, 1]$

**Parameters:** $L_1 = 3mH$, $L_2 = 1mH$, $C_f = 10\mu F$, $R_1 = R_2 = 0.1\Omega$

**Design Challenge:** LCL filter has resonance at $f_{res} = \frac{1}{2\pi}\sqrt{\frac{L_1+L_2}{L_1 L_2 C_f}} \approx 2.9$ kHz

**Pole Placement Design:**

Place poles to:
1. Provide adequate damping of resonance
2. Achieve fast current response
3. Keep control effort reasonable

Desired poles: $s_1 = -3000$, $s_{2,3} = -2000 \pm j2000$

```cpp
#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <cmath>
#include <complex>
#include <iostream>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    // LCL filter parameters
    double L1 = 3e-3, L2 = 1e-3, Cf = 10e-6;
    double R1 = 0.1, R2 = 0.1;
    
    // State matrices (3x3 single-phase LCL) using cppplot::Matrix
    // x = [i_L1, v_C, i_L2]^T
    Matrix A = {
        {-R1/L1, -1/L1, 0},
        {1/Cf,    0,    -1/Cf},
        {0,       1/L2, -R2/L2}
    };
    Matrix B = {{1/L1}, {0}, {0}};    // 3x1 column vector
    Matrix C_out = {{0, 0, 1}};       // 1x3 row vector (grid current output)
    Matrix D = Matrix::zeros(1, 1);
    
    // Create StateSpace system and analyze open-loop properties
    StateSpace sys(A, B, C_out, D);
    std::cout << "System order: " << sys.n_states << "\n";
    std::cout << "Controllable: " << (sys.isControllable() ? "Yes" : "No") << "\n\n";
    
    // Display open-loop poles
    auto ol_poles = sys.poles();
    std::cout << "Open-loop poles:\n";
    for (const auto& p : ol_poles)
        std::cout << "  " << p.real() << " + " << p.imag() << "j\n";
    
    // Desired closed-loop poles
    std::vector<std::complex<double>> desired_poles = {
        {-3000, 0}, {-2000, 2000}, {-2000, -2000}
    };
    
    // Compute state feedback gain using Ackermann's formula
    Matrix K = acker(A, B, desired_poles);
    std::cout << "\nState feedback gain K = " << K << "\n\n";
    
    // Simulate step response
    double dt = 1e-6;
    int N = 5000;
    
    Matrix x = Matrix::zeros(3, 1);
    double i_ref = 10.0;  // 10A reference
    
    // Compute DC gain compensation: Kr = -1 / (C * (A - B*K)^-1 * B)
    Matrix Acl = A - B * K;  // Closed-loop system matrix
    double Kr = -1.0 / (C_out * Acl.inv() * B)(0,0);
    
    std::vector<double> time_ms, i_grid, v_inv;
    
    for (int k = 0; k < N; ++k) {
        double t = k * dt;
        
        // Control law: u = -K*x + Kr*r
        double u = -(K * x)(0,0) + Kr * i_ref;
        
        // Clamp inverter voltage
        if (u > 400) u = 400;
        if (u < -400) u = -400;
        
        // Euler integration: dx = A*x + B*u
        Matrix dx = A * x + B * u;
        x = x + dx * dt;
        
        if (k % 50 == 0) {
            time_ms.push_back(t * 1000);
            i_grid.push_back(x(2,0));
            v_inv.push_back(u);
        }
    }
    
    figure(800, 600);
    subplot(2, 1, 1);
    plot(time_ms, i_grid, "-", opts({{"color", "blue"}, {"label", "Grid Current"}}));
    axhline(i_ref, opts({{"color", "red"}, {"linestyle", "--"}, {"label", "Reference"}}));
    ylabel("Current (A)");
    title("LCL Inverter State Feedback Control");
    legend(true);
    grid(true);
    
    subplot(2, 1, 2);
    plot(time_ms, v_inv, "-", opts({{"color", "green"}, {"label", "Inverter Voltage"}}));
    xlabel("Time (ms)");
    ylabel("Voltage (V)");
    grid(true);
    
    savefig("ch12_lcl_state_feedback.svg");
    return 0;
}
```

> **🔍 Stop and Think — Mechanism Questions**
>
> Before moving on:
> 1. The state feedback gain $\mathbf{K}$ was computed to place eigenvalues at specific locations. *Physically*, what does each column of $\mathbf{K}$ do? (The first column multiplies position error; the second multiplies velocity error. Which matters more for stability? For settling time?)
> 2. The inverted pendulum has 4 states but only 1 input. With pole placement, you can assign all 4 eigenvalues. But you have *no freedom left* — the gain $\mathbf{K}$ is uniquely determined. What design trade-off can you NOT make with a single input?
> 3. If one state (e.g., cart velocity $\dot{x}$) has sensor noise, the feedback term $K_2 \dot{x}$ injects noise directly into the control signal. How would you mitigate this without changing the controller?

### 12.9.2 BLDC Motor Speed Control with State Feedback

**Problem:** Design state feedback for BLDC motor speed control using current and speed measurements.

**Motor Model:**

States: $\mathbf{x} = [i_q, \omega]^T$ (q-axis current, mechanical speed)

$$\mathbf{A} = \begin{bmatrix}
-R_s/L_q & -\lambda_{pm}/L_q \\
K_t/J & -B/J
\end{bmatrix}, \quad
\mathbf{B} = \begin{bmatrix} 1/L_q \\ 0 \end{bmatrix}$$

Where:
- $R_s = 0.5\Omega$, $L_q = 2mH$
- $\lambda_{pm} = 0.1$ Wb (permanent magnet flux)
- $K_t = 0.1$ Nm/A, $J = 0.001$ kg·m², $B = 0.001$ Nm·s

**Design for Fast Response:**

Desired settling time: 50 ms → poles at $s_{1,2} = -80 \pm j80$

**Integral Action for Zero Steady-State Error:**

Augment system with integrator:

$$\mathbf{x}_{aug} = \begin{bmatrix} i_q \\ \omega \\ \int(\omega_{ref} - \omega)dt \end{bmatrix}$$

$$\mathbf{A}_{aug} = \begin{bmatrix}
-R_s/L_q & -\lambda_{pm}/L_q & 0 \\
K_t/J & -B/J & 0 \\
0 & -1 & 0
\end{bmatrix}, \quad
\mathbf{B}_{aug} = \begin{bmatrix} 1/L_q \\ 0 \\ 0 \end{bmatrix}$$

### 12.9.3 Digital PLL State Feedback Design

**Problem:** Design state feedback for DPLL with fast lock-time.

> **⚠️ Clarification:** The gains below are standard Type-2 PLL loop filter gains derived from a second-order model with damping ratio $\zeta$ and natural frequency $\omega_n$. This is **not** an LQR design (which would require solving a discrete algebraic Riccati equation). The state-space formulation below provides a convenient framework for the conventional PLL design.

**Discrete-Time State-Space:**

$$\mathbf{x}[k+1] = \mathbf{F}\mathbf{x}[k] + \mathbf{G}u[k]$$

$$\mathbf{F} = \begin{bmatrix} 1 & T_s \\ 0 & 1 \end{bmatrix}, \quad
\mathbf{G} = \begin{bmatrix} T_s^2/2 \\ T_s \end{bmatrix}$$

States: Phase error $\theta_e$ and frequency error $\omega_e$

**LQR Design for Minimum Lock Time:**

Cost function: $J = \sum_{k=0}^{\infty} (\mathbf{x}^T\mathbf{Q}\mathbf{x} + u^2 R)$

With $\mathbf{Q} = \text{diag}(100, 1)$ (prioritize phase error) and $R = 0.01$:

$$\mathbf{K} = (R + \mathbf{G}^T\mathbf{P}\mathbf{G})^{-1}\mathbf{G}^T\mathbf{P}\mathbf{F}$$

Where $\mathbf{P}$ solves the discrete algebraic Riccati equation.

```cpp
// Digital PLL with state feedback (proportional-integral structure)
class DPLL_StateFeedback {
private:
    double K1, K2;  // State feedback gains
    double theta_hat, omega_hat;  // State estimates
    double Ts;
    
public:
    DPLL_StateFeedback(double sample_period, double bandwidth) {
        Ts = sample_period;
        
        // Standard Type-2 PLL gains (2nd-order, ζ = 0.707)
        // These are NOT LQR gains — they are derived from
        // the desired closed-loop natural frequency and damping
        double wn = 2 * M_PI * bandwidth;
        K1 = 2 * 0.707 * wn * Ts;  // Proportional (phase) gain
        K2 = wn * wn * Ts * Ts;     // Integral (frequency) gain
        
        theta_hat = omega_hat = 0;
    }
    
    double update(double phase_error) {
        // State feedback control law
        double u = K1 * phase_error + K2 * omega_hat;
        
        // State prediction
        theta_hat += Ts * omega_hat + u * Ts * Ts / 2;
        omega_hat += u * Ts;
        
        // Wrap phase
        while (theta_hat > M_PI) theta_hat -= 2*M_PI;
        while (theta_hat < -M_PI) theta_hat += 2*M_PI;
        
        return theta_hat;
    }
    
    double getFrequency() const { return omega_hat / (2*M_PI); }
};
```

### 12.9.4 Summary: State Feedback in EE and Telecom

| Application | States | Control Objective |
|-------------|--------|------------------|
| **Grid inverter** | Currents, capacitor voltage | Active damping of resonance |
| **Motor drive** | Current, speed (+integral) | Fast speed response, zero error |
| **PLL** | Phase error, frequency error | Minimum lock time |
| **Power converter** | Inductor current, output voltage | Tight voltage regulation |

**Design Guidelines:**

1. **Active damping:** State feedback can add damping to resonant modes without physical dampers (power loss)

2. **Integral augmentation:** Always add integrator state for zero steady-state error to step references

3. **LQR for optimality:** Use LQR when control effort must be minimized (power efficiency) or multiple states must be balanced

4. **Discrete implementation:** For digital controllers, design directly in discrete time to avoid discretization errors

---

## 12.10 Exercises

**E11.1 (Pole Placement by Hand)**
For the system with
$$\mathbf{A} = \begin{bmatrix} 0 & 1 \\ -2 & -3 \end{bmatrix}, \quad \mathbf{B} = \begin{bmatrix} 0 \\ 1 \end{bmatrix}$$
design a state feedback gain $\mathbf{K} = [k_1 \; k_2]$ to place the closed-loop poles at $s = -5 \pm j5$.

*Hint:* The desired characteristic polynomial is $s^2 + 10s + 50 = 0$. Use the relation $\det(s\mathbf{I} - (\mathbf{A} - \mathbf{B}\mathbf{K})) = s^2 + (3+k_2)s + (2+k_1) = s^2 + 10s + 50$ and solve for $k_1, k_2$.

---

**E11.2 (Verification with cppplot)**
Verify your design from E11.1 using `cppplot::control::acker()`. Also compute the gain using `place()` and compare the two results. Are they identical? Why or why not?

```cpp
// Starter code
Matrix A = {{0, 1}, {-2, -3}};
Matrix B = {{0}, {1}};
std::vector<std::complex<double>> poles = {{-5, 5}, {-5, -5}};
Matrix K_acker = acker(A, B, poles);
// Student task: also try place() and compare
```

---

**E11.3 (LQR Design and Comparison)**
Design an LQR controller for the same system in E11.1 with $\mathbf{Q} = \text{diag}(10, 1)$ and $R = 1$.

(a) Compute the optimal gain $\mathbf{K}$ and the resulting closed-loop pole locations.

(b) Compare the LQR gain and pole locations with the pole-placement design from E11.1. Which design results in lower control effort for a unit step initial condition?

---

**E11.4 (Feedforward Gain for Reference Tracking)**
For the system and state feedback gain from E11.1, with output $\mathbf{C} = [1 \; 0]$:

(a) Compute the feedforward gain $\bar{N}$ so that the steady-state output equals the step reference.

(b) Simulate the step response with and without $\bar{N}$ and compare the steady-state error.

*Recall:* $\bar{N} = -[\mathbf{C}(\mathbf{A} - \mathbf{B}\mathbf{K})^{-1}\mathbf{B}]^{-1}$

---

**E11.5 (Inverted Pendulum Stabilization)**
Using the inverted pendulum model from Section 11.7 with parameters $M = 1$ kg, $m = 0.3$ kg, $l = 0.5$ m, $g = 9.81$ m/s²:

(a) Verify that the open-loop system is unstable by computing the eigenvalues of $\mathbf{A}$.

(b) Check controllability of $(\mathbf{A}, \mathbf{B})$.

(c) Design a state feedback controller to place all closed-loop poles in the LHP with settling time $\leq 2$ s.

(d) Simulate the closed-loop response from a small initial angle perturbation.

---

**E11.6 (Stability Proof)**
Show that the closed-loop system $\dot{\mathbf{x}} = (\mathbf{A} - \mathbf{B}\mathbf{K})\mathbf{x}$ is asymptotically stable if and only if all eigenvalues of $\mathbf{A} - \mathbf{B}\mathbf{K}$ have strictly negative real parts.

*Hint:* Use the Lyapunov equation $\mathbf{A}_{cl}^T\mathbf{P} + \mathbf{P}\mathbf{A}_{cl} = -\mathbf{Q}$ with $\mathbf{Q} > 0$ and show $V(\mathbf{x}) = \mathbf{x}^T\mathbf{P}\mathbf{x}$ is a valid Lyapunov function.

---

**E11.7 (Design Trade-offs for Double Integrator)**
For the double integrator $\mathbf{A} = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix}$, $\mathbf{B} = \begin{bmatrix} 0 \\ 1 \end{bmatrix}$, design three LQR controllers:

| Case | Q | R |
|------|---|---|
| (a) Conservative | $\mathbf{I}$ | 1 |
| (b) Aggressive state penalty | $100\mathbf{I}$ | 1 |
| (c) Cheap control | $\mathbf{I}$ | 0.01 |

For each case, compute $\mathbf{K}$, closed-loop poles, and simulate the step response from $\mathbf{x}_0 = [1, 0]^T$. Plot all three responses on the same figure and discuss the trade-offs between settling time, overshoot, and peak control effort.

### Problem Identification Exercises (Level 3-4)

**E11.8 — What Is the Real Problem?**
An inverted pendulum controller is designed using pole placement. In simulation, the pendulum stabilizes perfectly. On the real hardware, the cart hits its physical travel limit (0.5m rail) before the pendulum reaches vertical.

(a) Why does pole placement not account for this constraint? What assumption in the design is violated?
(b) The student’s approach: "Move the desired poles further left to make the system faster." Explain why this makes the problem *worse*, not better.
(c) Reformulate: what is the *real* design problem? (Hint: it is not just stability — it is stability *within physical constraints*.)

**E11.9 — Mechanism vs. Procedure**
Two controllers are designed for the same plant: (i) pole placement with $\mathbf{K}$ computed via Ackermann’s formula, (ii) LQR with $\mathbf{Q}$ and $R$ chosen by trial and error. Both give similar closed-loop poles.

(a) What is the fundamental difference in *philosophy* between the two approaches? (One specifies *where* the poles should be; the other specifies *what matters*.)
(b) If the plant model has 20% uncertainty in the $\mathbf{B}$ matrix, which design is likely more robust? Why? (Hint: LQR has guaranteed gain margins.)
(c) A practicing engineer says: "I always use LQR because it gives guaranteed margins." Under what conditions is this statement FALSE?

---

## 12.11 Summary

| Concept | Key Point |
|---------|-----------|
| State feedback | $u = -Kx$ modifies closed-loop poles |
| Controllability | Required for arbitrary pole placement |
| Ackermann's formula | Direct computation of K |
| Reference tracking | Requires feedforward or integral action |
| Trade-offs | Speed vs. control effort, robustness |

---

## References


1. Franklin, G.F. et al. (2019). *Feedback Control of Dynamic Systems*
2. Ogata, K. (2010). *Modern Control Engineering*
3. Teodorescu, R. et al. (2011). *Grid Converters for Photovoltaic and Wind Power Systems*
4. Krishnan, R. (2017). *Permanent Magnet Synchronous and Brushless DC Motor Drives*
