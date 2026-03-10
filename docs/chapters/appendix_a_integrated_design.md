# Appendix A: Integrated Control System Design - From Problem to Implementation

---

> **Philosophy:** This appendix demonstrates the **complete journey** of control system development, bridging the gap between isolated academic disciplines. We follow a real-world problem through all stages: understanding the need, physics-based modeling, multi-domain analysis, controller design, and embedded implementation.

---

## A.1 The Problem: Why Do We Need Control?

### A.1.1 Starting Point: A Real Human Need

**Scenario:** An electric vehicle (EV) manufacturer wants to improve driving experience. Customers complain about:
- Jerky acceleration from standstill
- Poor traction on wet roads
- Range anxiety (inefficient energy use)
- Inconsistent braking feel

**The Question:** How do we make the car respond smoothly, safely, and efficiently to driver inputs?

### A.1.2 Decomposing the Problem

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    FROM HUMAN NEED TO TECHNICAL REQUIREMENTS                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   HUMAN EXPERIENCE          TECHNICAL TRANSLATION        CONTROL OBJECTIVE │
│   ────────────────          ──────────────────────        ──────────────── │
│                                                                             │
│   "Smooth acceleration" ──► Low jerk (da/dt)        ──► Bandwidth limit    │
│                                                                             │
│   "Good traction"       ──► Wheel slip < 10%        ──► Slip ratio control │
│                                                                             │
│   "Efficient"           ──► Minimize I²R losses     ──► Optimal current    │
│                                                                             │
│   "Responsive"          ──► < 100ms torque response ──► Closed-loop BW     │
│                                                                             │
│   "Safe"                ──► Bounded behavior always ──► Robust stability   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### A.1.3 The Interdisciplinary Nature

This "simple" problem requires knowledge from:

| Domain | Relevant Knowledge | Role in Solution |
|--------|-------------------|------------------|
| **Mechanical Engineering** | Vehicle dynamics, tire-road friction | Plant model |
| **Electrical Engineering** | Motor characteristics, power electronics | Actuator model |
| **Electronics** | Sensors, ADC/DAC, EMC | Signal conditioning |
| **Computer Science** | Real-time systems, algorithms | Implementation |
| **Control Theory** | Stability, performance, robustness | Controller design |
| **Physics** | Newton's laws, electromagnetism | First principles |
| **Mathematics** | Differential equations, linear algebra | Analysis tools |

---

## A.2 Modeling: Capturing Reality in Equations

### A.2.1 The Art of Modeling

> **Key Insight:** A model is not reality—it's a *useful approximation* that captures the essential dynamics for our control objective.

**Modeling Philosophy:**

```
    Reality              Model                  Purpose
    ───────              ─────                  ───────
    
    Infinitely     →    Finite-dimensional  →  Tractable analysis
    complex              state-space
    
    Continuous     →    Discrete samples    →  Digital implementation
    physics
    
    Nonlinear      →    Linearized around   →  Linear control tools
    behavior             operating point
    
    Uncertain      →    Nominal + bounds    →  Robust design
    parameters
```

### A.2.2 Building the Vehicle Model

**Step 1: Identify the Physical Subsystems**

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         EV TRACTION SYSTEM                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐ │
│   │ Battery │───►│Inverter │───►│ Motor   │───►│Gearbox  │───►│ Wheel/  │ │
│   │  Pack   │    │(Power)  │    │ (PMSM)  │    │         │    │  Tire   │ │
│   └─────────┘    └─────────┘    └─────────┘    └─────────┘    └────┬────┘ │
│        │              │              │              │               │      │
│        ▼              ▼              ▼              ▼               ▼      │
│   Electrochemical  Power        Electro-       Mechanical      Tire-Road  │
│   Dynamics        Electronics   magnetic       Coupling        Friction   │
│                                                                             │
│   V_batt(SOC,T)   Switching    Torque         Gear ratio      μ(λ,road)  │
│   R_int(T)        dynamics     production     Inertia         Slip ratio │
│                   Dead-time    Back-EMF       Efficiency      Normal load│
└─────────────────────────────────────────────────────────────────────────────┘
```

**Step 2: Write Physics Equations for Each Subsystem**

#### Battery (Electrochemical)
$$V_{batt} = V_{oc}(SOC) - I_{batt} \cdot R_{int}(T)$$

*Where this comes from:* Thévenin equivalent circuit of battery
*Why it matters:* Available voltage limits maximum motor power

#### Inverter (Power Electronics)
$$V_{motor} = m \cdot V_{DC} \cdot \sin(\omega_e t + \phi)$$

*Where this comes from:* Sinusoidal PWM modulation theory (see note below on SVPWM vs sinusoidal PWM)
*Why it matters:* Bandwidth ~10-20 kHz, essentially instantaneous for control

#### Motor (Electromagnetism + Mechanics)

**Electrical (in dq frame):**
$$L_d \frac{di_d}{dt} = v_d - R_s i_d + \omega_e L_q i_q$$
$$L_q \frac{di_q}{dt} = v_q - R_s i_q - \omega_e L_d i_d - \omega_e \lambda_m$$

**Mechanical:**
$$J_m \frac{d\omega_m}{dt} = \tau_e - \tau_{load}$$

**Torque production:**
$$\tau_e = \frac{3}{2} p (\lambda_m i_q + (L_d - L_q) i_d i_q)$$

*Where this comes from:* Park transformation of 3-phase equations
*Why it matters:* Decoupled $i_d$ (flux) and $i_q$ (torque) control

#### Tire-Road Interface (Tribology + Vehicle Dynamics)

**Pacejka Magic Formula (empirical):**
$$F_x = D \sin(C \arctan(B\lambda - E(B\lambda - \arctan(B\lambda))))$$

**Simplified for control:**
$$F_x \approx C_\lambda \cdot \lambda \quad \text{for small slip}$$

**Slip ratio definition:**
$$\lambda = \frac{\omega_w r_w - v_x}{\max(\omega_w r_w, v_x)}$$

*Where this comes from:* Empirical tire testing + Coulomb friction theory
*Why it matters:* Nonlinear, varies with road condition—key uncertainty!

### A.2.3 Assembling the Complete Model

**State Variables (what we track):**
$$\mathbf{x} = \begin{bmatrix} i_d \\ i_q \\ \omega_m \\ \omega_w \\ v_x \end{bmatrix}$$

**Inputs (what we command):**
$$\mathbf{u} = \begin{bmatrix} v_d \\ v_q \end{bmatrix}$$

**Outputs (what we measure):**
$$\mathbf{y} = \begin{bmatrix} i_d \\ i_q \\ \omega_w \end{bmatrix}$$

**Note:** Motor speed $\omega_m$ is measured via encoder, wheel speed $\omega_w$ via wheel speed sensor, but vehicle velocity $v_x$ may need estimation!

### A.2.4 Model Hierarchy for Control Design

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        MODEL HIERARCHY                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   Level 0: TRUTH MODEL (Simulation)                                        │
│   ─────────────────────────────────                                        │
│   • Full nonlinear dynamics                                                │
│   • Temperature-dependent parameters                                       │
│   • Switching-level inverter model                                         │
│   • 3D tire model with load transfer                                       │
│   • Purpose: Validate controller in simulation                             │
│                                                                             │
│   Level 1: DESIGN MODEL (Controller Synthesis)                             │
│   ────────────────────────────────────────────                             │
│   • Linearized around operating point                                      │
│   • Averaged inverter model                                                │
│   • Linear tire model with uncertainty                                     │
│   • Purpose: Apply linear control theory                                   │
│                                                                             │
│   Level 2: REDUCED MODEL (Real-time Implementation)                        │
│   ─────────────────────────────────────────────────                        │
│   • Neglect fast dynamics (current loops)                                  │
│   • Cascaded structure                                                     │
│   • Purpose: Fit in embedded processor                                     │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## A.3 Control Architecture: Structuring the Solution

### A.3.1 Why Cascaded Control?

**Problem:** The system has dynamics spanning 4 orders of magnitude in bandwidth:
- Current dynamics: ~1 kHz
- Speed dynamics: ~10-100 Hz  
- Slip dynamics: ~1-10 Hz

**Solution:** Cascaded (nested) loops, each handling its own time scale:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     CASCADED CONTROL ARCHITECTURE                           │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   Driver      Traction        Torque         Current        Inverter       │
│   Intent      Control         Control        Control        Switching      │
│                                                                             │
│   ┌─────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐     │
│   │Accel│───►│  Slip   │───►│ Torque  │───►│  FOC    │───►│  PWM    │     │
│   │Pedal│    │ Control │    │ Limit   │    │ PI+PI   │    │(Sinusoi)│     │
│   └─────┘    └────┬────┘    └────┬────┘    └────┬────┘    └────┬────┘     │
│                   │              │              │              │           │
│    10 Hz        50 Hz         200 Hz        4 kHz          20 kHz         │
│   (Human)     (Vehicle)      (Mech)       (Elec)          (Switch)        │
│                   │              │              │              │           │
│                   ▼              ▼              ▼              ▼           │
│              ┌─────────────────────────────────────────────────────┐      │
│              │                    MOTOR + VEHICLE                   │      │
│              └─────────────────────────────────────────────────────┘      │
│                                       │                                    │
│                                       ▼                                    │
│                              [Sensors: Encoders, Current, Wheel Speed]    │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### A.3.2 Inner Loop: Field-Oriented Control (FOC)

**Purpose:** Make motor torque respond instantly to torque command

**Design Model (Current Loop):**
$$L_q \frac{di_q}{dt} = v_q - R_s i_q - e_{back}$$

This is first-order! Transfer function:
$$\frac{I_q(s)}{V_q(s)} = \frac{1/R_s}{\tau_e s + 1}, \quad \tau_e = L_q/R_s$$

**PI Controller Design:**

Using IMC (Internal Model Control) tuning:
$$K_p = \frac{L_q}{\tau_c}, \quad K_i = \frac{R_s}{\tau_c}$$

where $\tau_c$ = desired closed-loop time constant (typ. 0.5-2 ms)

**Result:** Current tracks command with bandwidth ~500 Hz - 2 kHz

### A.3.3 Middle Loop: Torque/Speed Control

With fast current loop, assume $i_q \approx i_{q,ref}$ instantly.

**Design Model:**
$$J \frac{d\omega}{dt} = K_t i_q - T_{load}$$

**PI Speed Controller:**
$$K_p = \frac{J}{\tau_s}, \quad K_i = \frac{K_p}{\tau_i}$$

where $\tau_s$ = desired speed loop bandwidth (typ. 10-50 ms)

### A.3.4 Outer Loop: Traction Control

**Purpose:** Prevent wheel slip, maximize traction force

**The Challenge:** 
- Tire-road friction is highly nonlinear
- Road condition (μ) is unknown and varies
- Slip ratio must stay in stable region

**Slip Dynamics Model:**
$$\dot{\lambda} = \frac{1}{v_x}[\dot{\omega}_w r_w - \dot{v}_x] = f(\tau_{motor}, F_x, v_x)$$

**Control Approaches:**

| Method | Pros | Cons |
|--------|------|------|
| Slip ratio control | Directly controls slip | Needs wheel speed measurement |
| Torque limiting | Simple | Conservative |
| Model Reference Adaptive | Adapts to road | Complex |
| Sliding Mode | Robust | Chattering |

---

## A.4 Controller Design: Applying Theory

### A.4.1 Current Controller (FOC) - Classical PI

**Specifications:**
- Bandwidth: 2 kHz (to support 200 Hz torque bandwidth)
- Phase margin: >60°
- Zero steady-state error to step

**Design using Bode:**

Plant: $G_i(s) = \frac{1/R_s}{(L_q/R_s)s + 1} = \frac{100}{0.001s + 1}$ (example values)

PI Controller: $C_i(s) = K_p(1 + \frac{1}{T_i s})$

**Tuning (Symmetric Optimum):**
$$T_i = \tau_e = L_q/R_s = 1\text{ ms}$$
$$K_p = \frac{\tau_e}{2\tau_c} \cdot R_s$$

For $\tau_c = 0.25$ ms: $K_p = 2$

> **Reconciliation:** The IMC tuning in §A.3.2 and the Symmetric Optimum in §A.4.1 both design the d/q-axis current controller but yield different gains because they optimize different criteria:
> - **IMC** targets a first-order closed-loop $G_{cl} = 1/(\tau_c s + 1)$ via direct model inversion, giving $K_p = L_q/\tau_c$
> - **Symmetric Optimum** maximizes phase margin of the current loop (accounting for PWM delay and sensor filtering), giving $K_p = L_q/(2\tau_c)$ where $\tau_c$ now represents the *sum* of parasitic time constants
> 
> In practice, Symmetric Optimum ($K_p = L_q/(2\tau_c)$) is preferred for current loops because it accounts for the PWM delay. The apparent factor-of-2 discrepancy disappears when $\tau_c$ in each method is interpreted correctly.

### A.4.2 Speed Controller - PI with Anti-Windup

**Specifications:**
- Bandwidth: 50 Hz
- Torque limit: ±200 Nm
- Smooth response (no overshoot)

**Anti-Windup Implementation:**

```cpp
class PIController {
private:
    double Kp, Ki;
    double integral;
    double output_min, output_max;
    
public:
    double compute(double error, double dt) {
        // Proportional term
        double P = Kp * error;
        
        // Integral term with anti-windup
        double I_tentative = integral + Ki * error * dt;
        double output_tentative = P + I_tentative;
        
        // Apply saturation
        double output = std::clamp(output_tentative, output_min, output_max);
        
        // Back-calculate integral (anti-windup)
        if (output != output_tentative) {
            integral = output - P;  // Adjust integral to match saturated output
        } else {
            integral = I_tentative;
        }
        
        return output;
    }
};
```

### A.4.3 Traction Controller - Sliding Mode

**Why Sliding Mode?**
- Tire friction is uncertain (dry vs. wet vs. ice)
- We need robustness to parameter variation
- Fast response to incipient slip

**Design:**

Define sliding surface:
$$s = \lambda - \lambda_{ref}$$

where $\lambda_{ref}$ = optimal slip ratio (typ. 0.1-0.2)

Control law:
$$\tau_{motor} = \tau_{eq} - k \cdot \text{sign}(s)$$

where:
- $\tau_{eq}$ = equivalent control (from nominal model)
- $k$ = switching gain (must exceed uncertainty bound)

**Chattering Reduction:**
Replace $\text{sign}(s)$ with $\tanh(s/\phi)$ or boundary layer approach.

### A.4.4 Stability Analysis

**For the cascaded system:**

1. **Inner loop stable?** Check eigenvalues of $A_{cl,current}$
2. **Separation of time scales?** Current BW >> Speed BW >> Slip BW
3. **Robust stability?** Check against parameter variations

**Nyquist Criterion Application:**

For speed loop with uncertain load inertia $J \in [J_{min}, J_{max}]$:
- Plot Nyquist for both extremes
- Ensure no encirclements for entire range

---

## A.5 Implementation: From Math to Metal

### A.5.1 The Implementation Challenge

```
┌─────────────────────────────────────────────────────────────────────────────┐
│              FROM CONTINUOUS THEORY TO DISCRETE REALITY                     │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   CONTINUOUS DESIGN              DISCRETE IMPLEMENTATION                   │
│   ─────────────────              ───────────────────────                   │
│                                                                             │
│   $u(t) = K_p e(t) + ...$   →   u[k] = Kp * e[k] + ...                    │
│   (infinite precision)          (32-bit float or fixed-point)              │
│                                                                             │
│   Continuous time               Sample period T_s = 50 μs                  │
│                                                                             │
│   Ideal sensors                 ADC: 12-bit, ±2 LSB noise                  │
│                                                                             │
│   Instant actuation             PWM: 20 kHz, dead-time 500 ns              │
│                                                                             │
│   No delay                      Computation: 10-20 μs                       │
│                                                                             │
│   Perfect model                 ±20% parameter uncertainty                  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### A.5.2 Discretization

**Current Controller (Tustin/Bilinear):**

Continuous: $C(s) = K_p + \frac{K_i}{s}$

Discrete (Tustin with $s = \frac{2}{T_s}\frac{z-1}{z+1}$):
$$C(z) = K_p + K_i \frac{T_s}{2} \frac{z+1}{z-1}$$

Implementation form:
$$u[k] = u[k-1] + K_p(e[k] - e[k-1]) + K_i \frac{T_s}{2}(e[k] + e[k-1])$$

### A.5.3 Fixed-Point vs. Floating-Point

**Comparison:**

| Aspect | Fixed-Point (Q15) | Floating-Point (32-bit) |
|--------|-------------------|-------------------------|
| Precision | 15 bits fractional | 23 bits mantissa |
| Range | -1 to +0.99997 | ±3.4 × 10³⁸ |
| Overflow | Must scale carefully | Automatic |
| Speed | Faster on DSP | Similar on modern MCU |
| Cost | Cheaper processors | Standard now |

**Modern Recommendation:** Use 32-bit float unless extreme cost pressure.

### A.5.4 Complete Embedded Code Structure

```cpp
/**
 * @file motor_control.cpp
 * @brief Complete FOC + Traction Control Implementation
 * 
 * Target: ARM Cortex-M4F @ 168 MHz (e.g., STM32F4)
 * ADC: 12-bit, 3 channels simultaneous sampling
 * PWM: Center-aligned, 20 kHz, complementary outputs with dead-time
 */

#include <cmath>
#include <algorithm>

// ============================================================================
// HARDWARE ABSTRACTION
// ============================================================================

struct ADCReadings {
    float i_a, i_b, i_c;     // Phase currents [A]
    float v_dc;               // DC bus voltage [V]
    float theta_e;            // Electrical angle [rad]
    float omega_m;            // Mechanical speed [rad/s]
    float omega_w;            // Wheel speed [rad/s]
};

struct PWMCommands {
    float duty_a, duty_b, duty_c;  // 0.0 to 1.0
};

// ============================================================================
// COORDINATE TRANSFORMS
// ============================================================================

struct DQFrame {
    float d, q;
};

struct AlphaBetaFrame {
    float alpha, beta;
};

// Clarke Transform: abc → αβ
AlphaBetaFrame clarkeTransform(float i_a, float i_b, float i_c) {
    return {
        .alpha = i_a,
        .beta = (i_a + 2.0f * i_b) / sqrtf(3.0f)
    };
}

// Park Transform: αβ → dq
DQFrame parkTransform(AlphaBetaFrame ab, float theta) {
    float cos_t = cosf(theta);
    float sin_t = sinf(theta);
    return {
        .d = ab.alpha * cos_t + ab.beta * sin_t,
        .q = -ab.alpha * sin_t + ab.beta * cos_t
    };
}

// Inverse Park: dq → αβ
AlphaBetaFrame inversePark(DQFrame dq, float theta) {
    float cos_t = cosf(theta);
    float sin_t = sinf(theta);
    return {
        .alpha = dq.d * cos_t - dq.q * sin_t,
        .beta = dq.d * sin_t + dq.q * cos_t
    };
}

// Sinusoidal PWM (inverse Clarke modulation)
// NOTE: This is NOT Space Vector PWM (SVPWM). True SVPWM uses sector
// determination and active/zero vector timing to achieve ~15% better DC bus
// utilization. This implementation uses sinusoidal (inverse Clarke) modulation,
// which directly maps αβ voltages to per-phase duty cycles.
PWMCommands sinusoidal_pwm(AlphaBetaFrame v_ab, float v_dc) {
    // Normalize to DC bus
    float v_alpha = v_ab.alpha / v_dc;
    float v_beta = v_ab.beta / v_dc;
    
    // Inverse Clarke transform to 3-phase duty cycles
    // d_a = 0.5 + v_alpha
    // d_b = 0.5 - 0.5*v_alpha + (√3/2)*v_beta
    // d_c = 0.5 - 0.5*v_alpha - (√3/2)*v_beta
    return {
        .duty_a = std::clamp(0.5f + v_alpha, 0.0f, 1.0f),
        .duty_b = std::clamp(0.5f - 0.5f*v_alpha + 0.866f*v_beta, 0.0f, 1.0f),
        .duty_c = std::clamp(0.5f - 0.5f*v_alpha - 0.866f*v_beta, 0.0f, 1.0f)
    };
}

// ============================================================================
// PI CONTROLLERS WITH ANTI-WINDUP
// ============================================================================

class PIController {
public:
    float Kp, Ki;
    float integral = 0.0f;
    float out_min, out_max;
    
    PIController(float kp, float ki, float min, float max)
        : Kp(kp), Ki(ki), out_min(min), out_max(max) {}
    
    float compute(float error, float dt) {
        float P = Kp * error;
        float I_new = integral + Ki * error * dt;
        float output = P + I_new;
        
        // Clamp and anti-windup
        if (output > out_max) {
            output = out_max;
            I_new = out_max - P;
        } else if (output < out_min) {
            output = out_min;
            I_new = out_min - P;
        }
        integral = I_new;
        
        return output;
    }
    
    void reset() { integral = 0.0f; }
};

// ============================================================================
// TRACTION CONTROLLER (Sliding Mode)
// ============================================================================

class TractionController {
public:
    float lambda_ref = 0.15f;    // Target slip ratio
    float k_switch = 50.0f;      // Switching gain [Nm]
    float phi = 0.02f;           // Boundary layer thickness
    
    float compute(float omega_wheel, float v_vehicle, float r_wheel) {
        // Compute actual slip ratio
        float v_wheel = omega_wheel * r_wheel;
        float v_max = std::max(std::abs(v_wheel), std::abs(v_vehicle));
        
        float lambda = 0.0f;
        if (v_max > 0.5f) {  // Avoid division by near-zero
            lambda = (v_wheel - v_vehicle) / v_max;
        }
        
        // Sliding surface
        float s = lambda - lambda_ref;
        
        // Smooth switching function (tanh for chattering reduction)
        float u_switch = tanhf(s / phi);
        
        // Torque reduction command
        float torque_correction = -k_switch * u_switch;
        
        return torque_correction;
    }
};

// ============================================================================
// MAIN CONTROL LOOP (Called from PWM interrupt @ 20 kHz)
// ============================================================================

// Controller instances (initialized at startup)
PIController pi_id(2.0f, 500.0f, -400.0f, 400.0f);   // d-axis current
PIController pi_iq(2.0f, 500.0f, -400.0f, 400.0f);   // q-axis current
PIController pi_speed(0.5f, 5.0f, -200.0f, 200.0f);  // Speed (torque output)
TractionController traction;

// Reference inputs (from higher-level control / driver)
volatile float torque_request = 0.0f;    // From accelerator pedal [Nm]
volatile float speed_limit = 1000.0f;    // Maximum speed [rad/s]

// State variables
float v_vehicle_estimated = 0.0f;

void controlLoopISR(const ADCReadings& adc, PWMCommands& pwm) {
    static const float Ts = 50e-6f;  // 50 μs sample period
    static const float r_wheel = 0.3f;  // Wheel radius [m]
    static const float gear_ratio = 8.0f;
    
    // ────────────────────────────────────────────────────────────────
    // 1. COORDINATE TRANSFORM: abc → dq
    // ────────────────────────────────────────────────────────────────
    AlphaBetaFrame i_ab = clarkeTransform(adc.i_a, adc.i_b, adc.i_c);
    DQFrame i_dq = parkTransform(i_ab, adc.theta_e);
    
    // ────────────────────────────────────────────────────────────────
    // 2. TRACTION CONTROL (Outer loop, runs every 10th cycle = 2 kHz)
    // ────────────────────────────────────────────────────────────────
    static int traction_counter = 0;
    static float torque_limit = 200.0f;
    
    if (++traction_counter >= 10) {
        traction_counter = 0;
        
        // Estimate vehicle velocity (simple: assume no slip on average)
        v_vehicle_estimated = 0.95f * v_vehicle_estimated + 
                              0.05f * (adc.omega_w * r_wheel);
        
        // Compute traction correction
        float traction_correction = traction.compute(
            adc.omega_w, v_vehicle_estimated, r_wheel);
        
        // Apply correction to torque limit
        torque_limit = std::max(10.0f, 200.0f + traction_correction);
    }
    
    // ────────────────────────────────────────────────────────────────
    // 3. SPEED CONTROL (Middle loop, runs every 4th cycle = 5 kHz)
    // ────────────────────────────────────────────────────────────────
    static int speed_counter = 0;
    static float iq_ref = 0.0f;
    
    if (++speed_counter >= 4) {
        speed_counter = 0;
        
        // Speed control only if in speed mode (otherwise direct torque)
        float speed_error = speed_limit - adc.omega_m;
        float torque_from_speed = pi_speed.compute(speed_error, 4*Ts);
        
        // Combine with driver torque request
        float torque_cmd = std::min(torque_request, torque_from_speed);
        
        // Apply traction limit
        torque_cmd = std::clamp(torque_cmd, -torque_limit, torque_limit);
        
        // Convert torque to q-axis current reference
        // τ = (3/2) * p * λm * iq  →  iq = τ / ((3/2) * p * λm)
        static const float Kt = 0.5f;  // Torque constant [Nm/A]
        iq_ref = torque_cmd / Kt;
    }
    
    // ────────────────────────────────────────────────────────────────
    // 4. CURRENT CONTROL (Inner loop, every cycle = 20 kHz)
    // ────────────────────────────────────────────────────────────────
    float id_ref = 0.0f;  // Zero for max torque/amp (MTPA would be different)
    
    float vd_cmd = pi_id.compute(id_ref - i_dq.d, Ts);
    float vq_cmd = pi_iq.compute(iq_ref - i_dq.q, Ts);
    
    // Feedforward decoupling (optional, improves dynamic response)
    static const float Lq = 0.001f, Ld = 0.001f, lambda_m = 0.1f;
    vd_cmd += -adc.omega_m * Lq * i_dq.q;           // d-axis decoupling
    vq_cmd +=  adc.omega_m * (Ld * i_dq.d + lambda_m);  // q-axis + back-EMF
    
    // ────────────────────────────────────────────────────────────────
    // 5. INVERSE TRANSFORM: dq → abc (PWM)
    // ────────────────────────────────────────────────────────────────
    DQFrame v_dq = {vd_cmd, vq_cmd};
    AlphaBetaFrame v_ab = inversePark(v_dq, adc.theta_e);
    pwm = sinusoidal_pwm(v_ab, adc.v_dc);
}
```

### A.5.5 Testing and Validation

**Test Hierarchy:**

| Level | Test Type | What's Checked |
|-------|-----------|----------------|
| 1 | Unit Test | Individual functions (transforms, PI) |
| 2 | Software-in-Loop (SIL) | Algorithm on PC with plant model |
| 3 | Hardware-in-Loop (HIL) | Real controller + simulated plant |
| 4 | Dynamometer | Real controller + real motor, no vehicle |
| 5 | Vehicle Test | Complete system on test track |

---

## A.6 Debugging: When Things Go Wrong

### A.6.1 Common Problems and Solutions

| Symptom | Likely Cause | Diagnostic | Solution |
|---------|--------------|------------|----------|
| Motor vibrates at standstill | Encoder offset wrong | Check angle at known position | Calibrate offset |
| Current spikes | Wrong PWM output | Log PWM vs. angle | Fix PWM logic |
| Torque oscillation | PI gains too high | Reduce and observe | Retune with margins |
| Loss of control at high speed | Back-EMF > V_dc | Check voltage headroom | Field weakening |
| Slip during launch | Traction gains too low | Log slip ratio | Increase k_switch |

### A.6.2 The Debugging Mindset

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                       SYSTEMATIC DEBUGGING                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   1. OBSERVE: What exactly is the symptom?                                 │
│      • Time domain: oscillation, drift, saturation?                        │
│      • Frequency: at what frequency does problem occur?                    │
│      • Conditions: always, or only at certain speed/load?                  │
│                                                                             │
│   2. HYPOTHESIZE: What could cause this?                                   │
│      • Sensor issue? (noise, offset, scaling)                              │
│      • Algorithm bug? (sign error, overflow, race condition)               │
│      • Model mismatch? (parameters wrong)                                  │
│      • Hardware fault? (connection, interference)                          │
│                                                                             │
│   3. TEST: Design experiment to confirm/reject hypothesis                  │
│      • One variable at a time                                              │
│      • Compare expected vs. actual                                         │
│      • Use scope, logic analyzer, data logging                             │
│                                                                             │
│   4. FIX: Implement solution                                               │
│      • Document the fix and why it works                                   │
│      • Verify fix doesn't break something else                             │
│      • Add test to prevent regression                                      │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## A.7 Iteration: The Real Engineering Process

### A.7.1 It's Never Right the First Time

```
    Requirements → Design → Implement → Test → Debug → Requirements...
                                                 │
                                                 └── "Aha! We need to add..."
```

**Evolution of our traction control:**

| Version | What we learned | What we changed |
|---------|-----------------|-----------------|
| v1.0 | Slip ratio noisy at low speed | Added velocity threshold |
| v1.1 | Too conservative on dry road | Made k_switch adaptive |
| v1.2 | Jerk when traction kicks in | Added rate limiter |
| v1.3 | Doesn't work in reverse | Fixed slip ratio sign |
| v2.0 | Need integration with ABS | Redesigned interface |

### A.7.2 Documentation is Part of the Product

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                   WHAT TO DOCUMENT                                          │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   1. REQUIREMENTS                                                           │
│      • What problem are we solving?                                        │
│      • What are the performance specs?                                     │
│      • What are the constraints?                                           │
│                                                                             │
│   2. DESIGN RATIONALE                                                       │
│      • Why this architecture?                                              │
│      • Why these controller types?                                         │
│      • What alternatives were considered?                                  │
│                                                                             │
│   3. MODEL DOCUMENTATION                                                    │
│      • Equations and assumptions                                           │
│      • Parameter values and sources                                        │
│      • Validation results                                                  │
│                                                                             │
│   4. CODE DOCUMENTATION                                                     │
│      • Function descriptions                                               │
│      • Data flow                                                           │
│      • Timing requirements                                                 │
│                                                                             │
│   5. TEST RESULTS                                                           │
│      • Test procedures                                                     │
│      • Pass/fail criteria                                                  │
│      • Known limitations                                                   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## A.8 Key Takeaways: The Integrated Approach

### A.8.1 What We Learned

| Stage | Traditional (Isolated) | Integrated Approach |
|-------|------------------------|---------------------|
| **Problem Definition** | "Design a motor controller" | "Make vehicle smooth, safe, efficient" |
| **Modeling** | Motor equations only | Motor + inverter + tire + driver |
| **Design** | Optimize one loop | Co-design cascaded loops |
| **Implementation** | "Software engineer handles it" | Controller engineer specifies constraints |
| **Debugging** | "It's a hardware problem" | Systematic multi-domain diagnosis |

### A.8.2 The Interdisciplinary Engineer

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    THE T-SHAPED ENGINEER                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│                    BREADTH (understand many fields)                        │
│    ─────────────────────────────────────────────────────────────────────   │
│    Mech │ Elec │ Electronics │ Software │ Control │ Physics │ Math        │
│         │      │             │          │         │         │              │
│         │      │             │          │    │    │         │              │
│         │      │             │          │    │    │         │              │
│         │      │             │          │    │    │         │              │
│         │      │             │          │    │    │         │              │
│         │      │             │          │    ▼    │         │              │
│                                         DEPTH                              │
│                                   (master one field)                       │
│                                                                             │
│   The most effective engineers have DEPTH in their specialty               │
│   and BREADTH to communicate across disciplines.                           │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### A.8.3 Final Thought

> **Engineering is not about finding the "optimal" solution in isolation. It's about finding a solution that works across all the constraints—physical, economic, regulatory, and human—that define the real problem.**

The control system we designed is not just mathematics. It's the intersection of:
- **Physics** (what the motor and tire can do)
- **Electronics** (how we measure and actuate)
- **Computing** (how fast we can calculate)
- **Economics** (what sensors/processors we can afford)
- **Safety** (what happens when things fail)
- **Human factors** (what the driver expects)

This is why control engineering remains challenging and rewarding—and why the ability to integrate across disciplines is the most valuable skill you can develop.

---

### 📋 Signal Dictionary — Electric Power Steering (EPS) Integrated System

| Signal | Symbol | Meaning | Typical unit |
|--------|--------|---------|--------------|
| Driver torque | $T_d$ | Torque applied by driver at steering wheel | N·m |
| Assist torque | $T_a$ | Torque produced by EPS motor to help the driver | N·m |
| Steering angle | $\theta_s$ | Steering wheel angle | rad |
| Motor current | $i_m$ | EPS motor armature current (proportional to $T_a$) | A |
| Motor voltage | $v_m$ | PWM-modulated voltage applied to EPS motor | V |
| Rack force | $F_r$ | Force at the steering rack — translates to tire angles | N |
| Vehicle speed | $v_{veh}$ | Used for speed-dependent assist: less assist at high speed | m/s |
| Lateral acceleration | $a_y$ | Feedback for stability — excessive = oversteer risk | m/s² |
| Torque sensor output | $T_{sens}$ | Measured torsion bar deflection = driver torque estimate | N·m |
| Reference current | $i_{ref}$ | Assist map output: $i_{ref} = f(T_{sens}, v_{veh})$ | A |
| Current error | $e_i = i_{ref} - i_m$ | Input to the inner PI current controller | A |
| PWM duty cycle | $D$ | Controller output to H-bridge inverter | % |

> **Key insight:** This system has THREE nested loops: (1) inner current loop (bandwidth ~1 kHz), (2) middle torque-assist loop (~100 Hz), (3) outer vehicle stability loop (~10 Hz). Each loop uses a different model fidelity and design method. The integrated design challenge is ensuring they don't interfere with each other.

---

### Exercises

**EA.1 🟢 (Design Trade-off Analysis)**
In the EPS system above, increasing the current loop bandwidth improves torque response but amplifies sensor noise.

(a) If the current sensor has noise with spectral density $N_0 = 10^{-6}$ A²/Hz, estimate the RMS noise in the assist torque for bandwidths of 500 Hz, 1 kHz, and 2 kHz.

(b) What is the maximum bandwidth if the acceptable torque ripple is < 0.1 N·m RMS?

(c) How would a Kalman filter (Ch. 15) change this trade-off? Sketch the design.

**EA.2 🟢 (Multi-Loop Interaction)**
Consider the three nested loops described above. If the outer stability loop has a bandwidth of 15 Hz (instead of 10 Hz), and the middle torque loop has 100 Hz:

(a) What is the frequency separation ratio? Is it sufficient?

(b) Describe a scenario where insufficient separation causes instability.

(c) Use Bode plots to show where the loops would interact.

**EA.3 🔴 (Level 3 — Cross-Domain Debugging)**
The EPS system works in simulation but oscillates at 200 Hz on the real vehicle.

(a) The 200 Hz is not near any designed closed-loop pole. What could cause this?

(b) You discover the torsion bar has a resonance at 210 Hz. How would you modify the controller to avoid exciting it?

(c) Would the H∞ framework from Ch. 16 have predicted this problem? Why or why not?

**EA.4 ⚫ (Level 4 — Philosophy)**
The EPS must feel "natural" to the driver — this is a *subjective* requirement that cannot be expressed as a transfer function specification.

(a) How do automotive engineers translate subjective feel into control specifications? Research and describe at least two approaches.

(b) Is this a control problem, a human factors problem, or both? What does this say about the limits of mathematical optimization?

---

## References

1. Pacejka, H.B. (2012). *Tire and Vehicle Dynamics*, 3rd ed. Butterworth-Heinemann.
2. Krishnan, R. (2010). *Permanent Magnet Synchronous and Brushless DC Motor Drives*. CRC Press.
3. Bose, B.K. (2002). *Modern Power Electronics and AC Drives*. Prentice Hall.
4. Franklin, G.F. et al. (2015). *Feedback Control of Dynamic Systems*, 7th ed. Pearson.
5. Åström, K.J. & Murray, R.M. (2021). *Feedback Systems*, 2nd ed. Princeton University Press.
6. Rajamani, R. (2012). *Vehicle Dynamics and Control*, 2nd ed. Springer.
