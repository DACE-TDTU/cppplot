# Chapter 1: Introduction to Control Systems
## Modern Control Engineering with C++

---

## 🎯 Learning Objectives and Outcomes

### Chapter Objectives

This chapter introduces the fundamental concepts of control systems and establishes the computational framework using CppPlot that will be used throughout the textbook.

### Learning Outcomes (Bloom's Taxonomy)

Upon successful completion of this chapter, you will be able to:

| Level | Outcome | Assessment |
|-------|---------|------------|
| **Remember** | Define open-loop, closed-loop, feedback, plant, controller, sensor | Quiz |
| **Understand** | Explain why feedback control is necessary and its advantages | Concept questions |
| **Apply** | Set up and compile CppPlot programs for control system analysis | Lab exercise |
| **Analyze** | Identify components in real-world control systems | System identification |
| **Evaluate** | Compare open-loop vs closed-loop performance for given scenarios | Case study |
| **Create** | Develop a simple feedback control simulation | Mini-project |

### Prerequisites

- Basic calculus (derivatives, integrals)
- Elementary differential equations
- C++ programming fundamentals
- Familiarity with a code editor (VS Code recommended)

---

## Why This Chapter Matters

> **The Real Problem:** A factory furnace overshoots by 50°C on every startup, ruining product batches worth $10,000 each. The operator adjusts the gas valve manually — sometimes too much, sometimes too late. An engineer installs a thermocouple and a PID controller. The overshoot drops to 2°C. **Why did feedback work?** The controller never measured the furnace's thermal mass, the gas flow rate, or the ambient temperature. It only observed the *temperature error* — and that single signal was enough. How is that possible?
>
> This chapter answers that question. By the end, you will understand the **mechanism** of feedback — not just the recipe.

---

## A Note on This Textbook's Approach

> **The Interdisciplinary Challenge:** Traditional control textbooks often present theory in isolation—equations floating in a vacuum without connection to physical implementation. This creates a gap between what students learn and what engineers actually do.

**This textbook takes a different approach:**

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    THE INTEGRATED ENGINEERING MINDSET                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   Traditional Approach           This Textbook's Approach                  │
│   ────────────────────           ─────────────────────────                  │
│                                                                             │
│   Control theory ─────┐          ┌─── Real Problem ◄─── Human Need         │
│                       │          │                                          │
│   Isolated from       │          ▼                                          │
│   physical reality    │     Physics-Based Modeling                         │
│                       │          │                                          │
│   "u(t) is the        │          ▼                                          │
│    control signal"    │     Controller Design                              │
│   (What IS u(t)??)    │     (with physical meaning)                        │
│                       │          │                                          │
│                       │          ▼                                          │
│                       │     Implementation                                  │
│                       │     (code, hardware, real constraints)             │
│                       │          │                                          │
│                       │          ▼                                          │
│                       └──── Working System                                  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Four Questions We Always Ask:**

| Question | What it Addresses |
|----------|-------------------|
| **Where does this problem come from?** | The real-world need driving the control design |
| **How do we describe it mathematically?** | Physics → Equations → Transfer functions |
| **How do we design a solution?** | Controller structure, tuning, trade-offs |
| **How do we make it work in practice?** | Sensors, actuators, power electronics, software |

**Example of the Difference:**

| Concept | Isolated Approach | Integrated Approach |
|---------|-------------------|---------------------|
| Transfer function $G(s)$ | "Given: $G(s) = \frac{10}{s+2}$" | "A DC motor has $G(s) = \frac{K_m}{\tau s + 1}$ where $K_m$ comes from torque constant and back-EMF..." |
| Controller output $u(t)$ | "The control signal" | "$u(t)$ is the **desired voltage** which becomes PWM duty cycle → H-bridge switching → actual motor voltage" |
| Stability margin | "GM > 6 dB, PM > 45°" | "We need GM > 6 dB because motor inductance can vary ±30% with temperature" |

> **See Appendix A** for a complete case study demonstrating this integrated approach: designing an EV traction control system from human needs to embedded code.

---

## 1.1 What is a Control System?

### Definition

A **control system** is an interconnection of components that provides a desired response by controlling one or more outputs based on one or more inputs.

### The Control Problem

Consider these everyday challenges:

1. **Room Temperature Control**: You want to maintain room temperature at 22°C despite outside weather changes
2. **Cruise Control**: You want to maintain car speed at 100 km/h despite hills and wind
3. **Robot Arm**: You want to move a robot arm to a precise position quickly without oscillation

All these problems share a common structure:
- A **desired output** (setpoint/reference)
- A **system to control** (plant)
- **Disturbances** that affect the output
- Need for **corrective action** to maintain desired behavior

### Visual Representation

```
                    CONTROL SYSTEM
    ┌────────────────────────────────────────────────┐
    │                                                │
    │   Reference ───▶ Controller ───▶ Plant ───▶ Output
    │   (Desired)         │            │             │
    │                     │            │             │
    │                     ▼            ▼             │
    │               Control        Disturbance       │
    │               Signal                           │
    └────────────────────────────────────────────────┘
```

---

## 1.2 Open-Loop vs Closed-Loop Control

### 1.2.1 Open-Loop Control

In **open-loop control**, the controller does not use feedback from the output to adjust its action.

```
    Reference ───▶ Controller ───▶ Plant ───▶ Output
                                      │
                                      ▼
                                 Disturbance
```

**Examples:**
- Toaster (fixed timer, doesn't check bread color)
- Washing machine (fixed cycle, doesn't check cleanliness)
- Traffic light (fixed timing, doesn't check traffic)

**Characteristics:**
- ✅ Simple and inexpensive
- ✅ No stability problems
- ❌ Cannot compensate for disturbances
- ❌ Requires accurate plant model

### 1.2.2 Closed-Loop (Feedback) Control

In **closed-loop control**, the controller uses measurement of the actual output to compute the control action.

```
    Reference ──┬──▶ Controller ───▶ Plant ───▶ Output
                │         ▲            │          │
                │         │            ▼          │
                │    Error│       Disturbance     │
                │         │                       │
                └────  - ─┴───── Sensor ◀─────────┘
                        (comparison)
```

**Examples:**
- Thermostat (measures temperature, adjusts heating)
- Cruise control (measures speed, adjusts throttle)
- Human balancing (senses position, adjusts muscles)

**Characteristics:**
- ✅ Compensates for disturbances
- ✅ Reduces sensitivity to plant variations
- ✅ Can stabilize unstable systems
- ❌ More complex
- ❌ Can become unstable if poorly designed

### 1.2.3 Comparison Example: Water Heater

**Scenario:** Heat water to 60°C

**Open-Loop Approach:**
- Calculate: "100W heater for 10 minutes should raise temperature by 40°C"
- Problem: Actual result depends on initial temperature, ambient conditions, container insulation

**Closed-Loop Approach:**
- Measure actual water temperature
- If temperature < 60°C → turn heater ON
- If temperature ≥ 60°C → turn heater OFF
- Result: Maintains 60°C regardless of disturbances

---

## 1.3 Components of a Control System

### 1.3.1 Plant (Process)

The **plant** is the system to be controlled. It's the physical process whose output we want to regulate.

| Application | Plant |
|-------------|-------|
| Temperature control | Room, oven, furnace |
| Speed control | Motor, vehicle |
| Position control | Robot arm, antenna |
| Level control | Tank, reservoir |
| Chemical process | Reactor, distillation column |

### 1.3.2 Sensor (Measurement)

The **sensor** measures the actual output and converts it to a signal the controller can use.

| Measured Variable | Sensor Type |
|-------------------|-------------|
| Temperature | Thermocouple, RTD, thermistor |
| Position | Encoder, potentiometer, LVDT |
| Speed | Tachometer, encoder |
| Pressure | Strain gauge, piezoelectric |
| Flow | Turbine meter, ultrasonic |

**Sensor Characteristics:**
- Accuracy and precision
- Range and resolution
- Dynamic response (bandwidth)
- Noise and interference

### 1.3.3 Actuator

The **actuator** converts the control signal into physical action on the plant.

| Application | Actuator |
|-------------|----------|
| Thermal | Heater, cooler, valve |
| Mechanical | Motor, hydraulic cylinder |
| Electrical | Power amplifier, relay |
| Chemical | Valve, pump |

### 1.3.4 Power Electronics: The Missing Link

> **Critical Insight:** Traditional control textbooks often show a direct connection from "Controller" to "Plant," obscuring a crucial component: the **power electronics interface**.

**Why this matters:**

```
┌───────────────────────────────────────────────────────────────────────────┐
│                   WHAT TEXTBOOKS OFTEN SHOW                               │
├───────────────────────────────────────────────────────────────────────────┤
│                                                                           │
│   Reference ──►(+)──► Controller ────────────────────►  Plant ──► Output  │
│                -▲           │                             │               │
│                 │           u(t)                          │               │
│                 │      "Control Signal"                   │               │
│                 └─────────────────────────────────────────┘               │
│                                                                           │
│   Problem: What IS u(t) physically? A voltage? A current? A force?       │
└───────────────────────────────────────────────────────────────────────────┘
```

```
┌───────────────────────────────────────────────────────────────────────────┐
│                     ACTUAL PHYSICAL IMPLEMENTATION                        │
├───────────────────────────────────────────────────────────────────────────┤
│                                                                           │
│              u_desired     ┌──────────┐   u_actual                        │
│   Controller ────────────► │  Power   │ ──────────►  Plant                │
│   (Digital/     (voltage   │Electronics  (actual                          │
│    Analog)       command)  │          │   voltage/                        │
│                            │ • H-Bridge   current/                        │
│                            │ • Inverter   torque)                         │
│                            │ • Chopper │                                  │
│                            │ • Amplifier                                  │
│                            └──────────┘                                   │
│                                                                           │
│   KEY: u_desired ≠ u_actual (limited by power supply, PWM, dynamics)     │
└───────────────────────────────────────────────────────────────────────────┘
```

**Example: DC Motor Speed Control**

| Stage | Signal | Physical Meaning |
|-------|--------|------------------|
| Controller output | $u_{desired}$ = 7.5 V | **Desired** armature voltage |
| PWM modulator | Duty cycle D = 62.5% | $D = u_{desired}/V_{supply}$ |
| H-Bridge output | $u_{actual}$ ≈ 7.5 V (average) | **Actual** voltage to motor |
| Motor response | $\omega$ increases | Mechanical output |

**What can go wrong:**
- $V_{supply}$ = 12V but drops to 11V under load → $u_{actual}$ ≠ $u_{desired}$
- PWM switching causes current ripple → torque ripple
- H-Bridge has dead-time → nonlinearity at zero-crossing
- Power transistors have saturation voltage → voltage loss

**Power Electronics Interfaces by Application:**

| Application | Power Converter | Input Signal | Output |
|-------------|-----------------|--------------|--------|
| DC motor speed | H-Bridge (4 MOSFET) | PWM duty cycle | Bipolar voltage |
| DC motor position | H-Bridge | PWM duty cycle | Voltage |
| BLDC/PMSM | 3-phase Inverter | 6 PWM signals | 3-phase AC |
| Induction motor | VFD (Inverter) | V/f command | Variable freq AC |
| Heater | SSR/Triac | Phase angle / On-off | AC power |
| Pneumatic valve | Solenoid driver | On/off or PWM | Valve position |
| Hydraulic | Servo valve | ±10V analog | Flow rate |

> **Textbook Convention:** For simplicity, control theory often **absorbs** the power electronics into the plant model, assuming $u_{actual} = u_{desired}$. This is valid when the power converter bandwidth is much higher than the control bandwidth (typically 10× or more).

### 1.3.5 Controller

The **controller** computes the control signal based on the error between reference and measured output.

**Controller types:**
- On-Off (bang-bang)
- Proportional (P)
- Proportional-Integral (PI)
- Proportional-Integral-Derivative (PID)
- State-space controllers
- Optimal controllers (LQR, MPC)

---

## 1.4 Control Applications Across Engineering Disciplines

### 1.4.1 Electrical Engineering Applications

Control systems are fundamental to power systems and electrical machines:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    ELECTRICAL ENGINEERING CONTROL APPLICATIONS              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   POWER SYSTEMS                                                            │
│   ─────────────                                                            │
│   • Voltage regulation (AVR - Automatic Voltage Regulator)                 │
│   • Frequency control (governor, load frequency control)                   │
│   • Power factor correction (VAR compensation)                             │
│   • HVDC transmission control                                              │
│   • Smart grid and microgrid control                                       │
│                                                                             │
│   ELECTRICAL MACHINES                                                      │
│   ───────────────────                                                      │
│   • DC motor speed/position control                                        │
│   • Induction motor V/f control, vector control, DTC                      │
│   • Synchronous machine excitation control                                 │
│   • PMSM/BLDC FOC (Field-Oriented Control)                                │
│   • Switched reluctance motor control                                      │
│                                                                             │
│   POWER ELECTRONICS                                                        │
│   ─────────────────                                                        │
│   • Buck/Boost/Buck-boost converter voltage regulation                     │
│   • Inverter grid synchronization (PLL)                                   │
│   • Active power filter control                                           │
│   • UPS output voltage regulation                                          │
│   • Battery charger CC-CV control                                          │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Example: Automatic Voltage Regulator (AVR)**

```
    V_ref ──►(+)──► AVR ──► Exciter ──► Generator ──► V_terminal
              -▲    Controller          Field         (Output)
               │                        Current
               │
               └──────────── PT ◄────────────────────────┘
                         (Potential Transformer)

    Control Objective: Maintain terminal voltage = V_ref despite load changes
```

### 1.4.2 Telecommunications Engineering Applications

Communication systems rely heavily on control theory:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    TELECOMMUNICATIONS CONTROL APPLICATIONS                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ANALOG COMMUNICATION                                                     │
│   ────────────────────                                                     │
│   • AGC (Automatic Gain Control) in receivers                             │
│   • AFC (Automatic Frequency Control) in oscillators                      │
│   • PLL (Phase-Locked Loop) for demodulation                              │
│   • ALC (Automatic Level Control) in transmitters                         │
│                                                                             │
│   DIGITAL COMMUNICATION                                                    │
│   ─────────────────────                                                    │
│   • Clock recovery (timing synchronization)                               │
│   • Carrier synchronization (COSTAS loop, decision-directed)              │
│   • Adaptive equalization (LMS, RLS algorithms)                           │
│   • Power control in cellular systems (CDMA, LTE)                         │
│   • Rate control in video streaming (TCP congestion control)              │
│                                                                             │
│   RF SYSTEMS                                                               │
│   ──────────                                                               │
│   • Antenna tracking systems (satellite, radar)                           │
│   • Beam steering (phased arrays)                                         │
│   • Transmitter power control                                             │
│   • Temperature compensation in oscillators (TCXO, OCXO)                  │
│                                                                             │
│   OPTICAL COMMUNICATION                                                    │
│   ─────────────────────                                                    │
│   • Laser diode current/temperature control                               │
│   • APD bias voltage control                                              │
│   • Fiber alignment systems                                               │
│   • EDFA gain control                                                      │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Example: Phase-Locked Loop (PLL)**

```
    f_ref ──►Phase ──► Loop ──► VCO ──┬──► f_out
             Detector   Filter        │
               ▲                      │
               │                      │
               └────── ÷N ◄───────────┘
                    (Divider)

    Control Objective: Lock f_out = N × f_ref with zero phase error

    PLL is a FEEDBACK SYSTEM!
    • Phase detector = error detector (comparing phases)
    • Loop filter = controller (often PI or lead-lag)
    • VCO = plant (voltage → frequency conversion)
    • Divider = feedback path (frequency scaling)
```

### 1.4.3 Cross-Domain Examples

| Control Concept | Electrical Example | Telecom Example |
|-----------------|-------------------|------------------|
| **PID Control** | Motor speed control | Power control in LTE |
| **State Feedback** | Field-oriented control | Adaptive equalizer |
| **Observer** | Sensorless motor drive | Channel estimation |
| **Optimal Control** | MPPT for solar | Waterfilling power allocation |
| **Robust Control** | Grid-connected inverter | Carrier recovery with noise |
| **Nonlinear Control** | Hysteresis current control | Limiter in AGC |

---

## 1.5 Historical Development of Control Systems

### Timeline

| Era | Development | Key Figures |
|------|------|------|
| **Ancient** | Water clocks, float valves | Ktesibios (270 BC) |
| **1788** | Steam engine governor | James Watt |
| **1868** | Mathematical analysis of governors | James Clerk Maxwell |
| **1932** | Feedback amplifiers | Harold Black |
| **1938** | Frequency response methods | Hendrik Bode |
| **1932** | Nyquist stability criterion | Harry Nyquist |
| **1948** | Root locus method | Walter Evans |
| **1960s** | State-space, optimal control | Rudolf Kalman |
| **1970s** | Robust control | John Doyle |
| **1980s+** | Digital control, adaptive control | Various |
| **2000s+** | MPC, machine learning | Various |

### Watt's Governor: The First Feedback Controller

```
                    ┌─────┐
                    │     │
              ┌─────┤     ├─────┐
             ╱      │     │      ╲
            ○       │shaft│       ○  ← Rotating masses
             ╲      │     │      ╱     (fly outward when
              └─────┤     ├─────┘      speed increases)
                    │     │
                    │  ║  │ ← Linkage to steam valve
                    │  ║  │
            ════════╪══╬══╪════════
                       ║
                   Steam Engine
```

**How it works:**
1. Engine speed increases → balls fly outward (centrifugal force)
2. Balls rising → linkage closes steam valve
3. Less steam → engine slows down
4. Result: Speed self-regulates

---

## 1.6 Introduction to CppPlot

### 1.6.1 Why C++ for Control Systems?

| Advantage | Description |
|-----------|-------------|
| **Performance** | Compiled code runs fast; critical for real-time control |
| **Embedded systems** | C++ runs on microcontrollers and embedded platforms |
| **Industry standard** | Widely used in automotive, aerospace, robotics |
| **Mathematical rigor** | Strong typing catches errors at compile time |
| **Learning value** | Understanding implementation deepens conceptual knowledge |

### 1.6.2 CppPlot Library Overview

CppPlot is a header-only C++ library providing:

```
CppPlot Library
├── Core Plotting
│   ├── figure(), plot(), scatter()
│   ├── subplot(), xlabel(), ylabel(), title()
│   └── savefig() → SVG output
│
├── Control Systems (cppplot/control/)
│   ├── TransferFunction
│   ├── StateSpace
│   ├── step(), impulse(), lsim()
│   ├── bode(), nyquist(), rlocus()
│   └── margin(), stepinfo()
│
├── Controller Design
│   ├── pid(), place(), acker()
│   ├── lqr(), lqe(), kalman()
│   └── mpc()
│
└── Linear Algebra
    ├── Matrix operations
    └── Polynomial operations
```

### 1.6.3 Setting Up Your Environment

**Requirements:**
- C++17 compatible compiler (g++ 7+, clang 5+, MSVC 2017+)
- CppPlot library (header-only, no installation needed)

**Project Structure:**
```
my_project/
├── include/
│   └── cppplot/          ← CppPlot headers
│       ├── cppplot.hpp
│       └── control/
│           └── control.hpp
├── src/
│   └── main.cpp          ← Your code
└── output/               ← Generated plots
```

**Compilation:**
```bash
g++ -std=c++17 -I include src/main.cpp -o my_program
```

### 1.6.4 Your First CppPlot Program

```cpp
/**
 * @file ch01_hello_control.cpp
 * @brief First CppPlot program - plotting a sine wave
 * 
 * Compile: g++ -std=c++17 -I "../include" ch01_hello_control.cpp -o ch01_hello
 */

#include <cppplot/cppplot.hpp>
#include <vector>
#include <cmath>

using namespace cppplot;

constexpr double PI = 3.14159265358979323846;

int main() {
    // Generate data: sine wave
    std::vector<double> t, y;
    for (double ti = 0; ti <= 4 * PI; ti += 0.05) {
        t.push_back(ti);
        y.push_back(std::sin(ti));
    }
    
    // Create figure
    figure(800, 500);
    
    // Plot the sine wave
    plot(t, y, "b-", {{"linewidth", "2"}, {"label", "sin(t)"}});
    
    // Add labels and formatting
    xlabel("Time t [rad]");
    ylabel("Amplitude");
    title("My First CppPlot: Sine Wave");
    legend();
    grid(true);
    
    // Save to file
    savefig("ch01_hello_control.svg");
    
    std::cout << "Plot saved to ch01_hello_control.svg" << std::endl;
    
    return 0;
}
```

---

## 1.7 First Control System Simulation

### 1.7.1 Physical System: Temperature Control

Let's simulate a simple room temperature control system.

**Physical Setup:**

```
    ┌──────────────────────────────────────┐
    │            ROOM                       │
    │                                       │
    │    ┌───────┐      T_room             │ ←── T_outside
    │    │HEATER │      (to control)       │     (disturbance)
    │    │ Q(t)  │                         │
    │    └───────┘                         │
    │                    [Sensor]          │
    └──────────────────────────────────────┘
                            │
                            ▼
                      T_measured
```

**Simplified Model:**
$$C\frac{dT}{dt} = Q(t) - \frac{T - T_{outside}}{R}$$

where:
- $C$ = thermal capacitance of room
- $R$ = thermal resistance of walls
- $Q(t)$ = heater power

**Signal Dictionary — Room Temperature Control**

| Signal | Symbol | Unit | Physical Meaning | Sensor/Actuator |
|--------|--------|------|------------------|-----------------|
| Reference temperature | $T_{ref}$ | °C | Desired room temperature (setpoint) | Thermostat dial / digital setting |
| Room temperature (output) | $T$ | °C | Actual air temperature inside the room | Thermistor / RTD sensor |
| Outside temperature (disturbance) | $T_{outside}$ | °C | Ambient temperature — the disturbance we cannot control | Weather station / outdoor sensor |
| Error signal | $e = T_{ref} - T$ | °C | How far the room is from the desired temperature | Computed (not measured directly) |
| Heater power (control input) | $Q(t)$ | W | Electrical power delivered to the heater element | Relay / SSR / TRIAC actuator |
| Thermal capacitance | $C$ | J/°C | How much energy the room stores per degree — large $C$ means slow response | (System parameter, not a signal) |
| Thermal resistance | $R$ | °C/W | How well the walls insulate — large $R$ means less heat loss | (System parameter, not a signal) |

> **Reading this table:** Every symbol in the equation above has a physical meaning, a unit, and a piece of hardware that produces or measures it. If you cannot fill in a row, you do not yet understand the system.

### 1.7.2 Open-Loop vs Closed-Loop Simulation

```cpp
/**
 * @file ch01_temperature_control.cpp
 * @brief Comparison of open-loop vs closed-loop temperature control
 * 
 * Learning Outcome: Understand the advantage of feedback control
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║   Chapter 1: Open-Loop vs Closed-Loop Temperature Control    ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // ========================================================================
    // Physical Parameters
    // ========================================================================
    double C = 1000;    // Thermal capacitance [J/°C]
    double R = 0.5;     // Thermal resistance [°C/W]
    double tau = C * R; // Time constant = 500 s
    
    // Reference temperature
    double T_ref = 22.0;      // Desired temperature [°C]
    double T_outside = 5.0;   // Outside temperature [°C]
    double T_initial = 15.0;  // Starting room temperature [°C]
    
    // Calculate required steady-state heater power
    double Q_ss = (T_ref - T_outside) / R;  // = 34 W
    
    std::cout << "\nSystem Parameters:" << std::endl;
    std::cout << "  Time constant τ = " << tau << " s" << std::endl;
    std::cout << "  Reference T_ref = " << T_ref << " °C" << std::endl;
    std::cout << "  Outside T_out = " << T_outside << " °C" << std::endl;
    std::cout << "  Required heater power Q = " << Q_ss << " W" << std::endl;
    
    // ========================================================================
    // Room thermal model: T(s)/Q(s) = R/(τs + 1)
    // ========================================================================
    TransferFunction G_room({R}, {tau, 1});
    
    // ========================================================================
    // Simulation Setup
    // ========================================================================
    double t_final = 3000;  // seconds
    int n_points = 500;
    double dt = t_final / n_points;
    
    std::vector<double> time;
    for (int i = 0; i <= n_points; ++i) {
        time.push_back(i * dt);
    }
    
    // ========================================================================
    // Scenario 1: Open-Loop Control (constant heater power)
    // ========================================================================
    std::cout << "\n▶ OPEN-LOOP CONTROL" << std::endl;
    
    // Heater set to calculated steady-state power
    std::vector<double> Q_openloop(time.size(), Q_ss);
    
    // Simulate temperature response
    auto sys_room = tf2ss(G_room);
    auto [t_ol, T_ol_response] = lsim(sys_room, Q_openloop, time);
    
    // Add bias from outside temperature contribution
    std::vector<double> T_openloop;
    for (size_t i = 0; i < T_ol_response.size(); ++i) {
        // T = T_outside + ΔT (where ΔT is response to heater)
        double T = T_outside + T_ol_response[i] * (1 - std::exp(-time[i]/tau)) 
                   + (T_initial - T_outside) * std::exp(-time[i]/tau);
        T_openloop.push_back(T);
    }
    
    // Simplified: direct numerical simulation
    T_openloop.clear();
    double T_current = T_initial;
    for (size_t i = 0; i < time.size(); ++i) {
        T_openloop.push_back(T_current);
        double dTdt = (Q_ss - (T_current - T_outside)/R) / C;
        T_current += dTdt * dt;
    }
    
    // ========================================================================
    // Scenario 2: Closed-Loop Control (on-off thermostat)
    // ========================================================================
    std::cout << "▶ CLOSED-LOOP CONTROL (On-Off Thermostat)" << std::endl;
    
    std::vector<double> T_closedloop;
    std::vector<double> Q_closedloop;
    double Q_max = 50;  // Maximum heater power [W]
    double hysteresis = 0.5;  // °C
    
    T_current = T_initial;
    bool heater_on = true;
    
    for (size_t i = 0; i < time.size(); ++i) {
        T_closedloop.push_back(T_current);
        
        // On-off controller with hysteresis
        if (T_current < T_ref - hysteresis) {
            heater_on = true;
        } else if (T_current > T_ref + hysteresis) {
            heater_on = false;
        }
        
        double Q = heater_on ? Q_max : 0;
        Q_closedloop.push_back(Q);
        
        // Update temperature
        double dTdt = (Q - (T_current - T_outside)/R) / C;
        T_current += dTdt * dt;
    }
    
    // ========================================================================
    // Scenario 3: Open-Loop with Disturbance (outside temp drops)
    // ========================================================================
    std::cout << "▶ OPEN-LOOP WITH DISTURBANCE" << std::endl;
    
    std::vector<double> T_openloop_dist;
    T_current = T_initial;
    
    for (size_t i = 0; i < time.size(); ++i) {
        T_openloop_dist.push_back(T_current);
        
        // Disturbance: outside temperature drops at t = 1000s
        double T_out = (time[i] < 1000) ? T_outside : (T_outside - 10);
        
        // Open-loop: heater power stays constant
        double dTdt = (Q_ss - (T_current - T_out)/R) / C;
        T_current += dTdt * dt;
    }
    
    // ========================================================================
    // Scenario 4: Closed-Loop with Same Disturbance
    // ========================================================================
    std::cout << "▶ CLOSED-LOOP WITH DISTURBANCE" << std::endl;
    
    std::vector<double> T_closedloop_dist;
    std::vector<double> Q_closedloop_dist;
    T_current = T_initial;
    heater_on = true;
    
    for (size_t i = 0; i < time.size(); ++i) {
        T_closedloop_dist.push_back(T_current);
        
        // Same disturbance
        double T_out = (time[i] < 1000) ? T_outside : (T_outside - 10);
        
        // Closed-loop controller
        if (T_current < T_ref - hysteresis) {
            heater_on = true;
        } else if (T_current > T_ref + hysteresis) {
            heater_on = false;
        }
        
        double Q = heater_on ? Q_max : 0;
        Q_closedloop_dist.push_back(Q);
        
        double dTdt = (Q - (T_current - T_out)/R) / C;
        T_current += dTdt * dt;
    }
    
    // ========================================================================
    // Plotting Results
    // ========================================================================
    std::cout << "\n▶ GENERATING PLOTS..." << std::endl;
    
    // Figure 1: Basic comparison
    figure(1000, 600);
    
    plot(time, T_openloop, "b-", {{"linewidth", "2"}, {"label", "Open-Loop"}});
    plot(time, T_closedloop, "r-", {{"linewidth", "2"}, {"label", "Closed-Loop (On-Off)"}});
    axhline(T_ref, {{"color", "green"}, {"linestyle", "--"}, {"label", "Reference (22°C)"}});
    
    xlabel("Time [s]");
    ylabel("Temperature [°C]");
    title("Open-Loop vs Closed-Loop Temperature Control");
    legend();
    grid(true);
    ylim(10, 30);
    
    savefig("ch01_ol_vs_cl_basic.svg");
    std::cout << "  ✓ Saved ch01_ol_vs_cl_basic.svg" << std::endl;
    
    // Figure 2: With disturbance
    figure(1000, 700);
    layout(2, 1);
    
    subplot(2, 1, 1);
    plot(time, T_openloop_dist, "b-", {{"linewidth", "2"}, {"label", "Open-Loop"}});
    plot(time, T_closedloop_dist, "r-", {{"linewidth", "2"}, {"label", "Closed-Loop"}});
    axhline(T_ref, {{"color", "green"}, {"linestyle", "--"}, {"label", "Reference"}});
    axvline(1000, {{"color", "orange"}, {"linestyle", ":"}, {"alpha", "0.7"}});
    text(1050, 25, "Disturbance: T_outside drops by 10°C", {{"fontsize", "10"}});
    
    xlabel("Time [s]");
    ylabel("Temperature [°C]");
    title("Disturbance Rejection: Open-Loop vs Closed-Loop");
    legend();
    grid(true);
    
    subplot(2, 1, 2);
    std::vector<double> T_out_plot;
    for (double t : time) {
        T_out_plot.push_back((t < 1000) ? T_outside : (T_outside - 10));
    }
    plot(time, T_out_plot, "purple", {{"linewidth", "2"}, {"label", "Outside Temperature"}});
    xlabel("Time [s]");
    ylabel("T_outside [°C]");
    title("Disturbance: Outside Temperature");
    legend();
    grid(true);
    
    savefig("ch01_disturbance_rejection.svg");
    std::cout << "  ✓ Saved ch01_disturbance_rejection.svg" << std::endl;
    
    // Figure 3: Control effort comparison
    figure(1000, 500);
    
    plot(time, Q_closedloop_dist, "r-", {{"linewidth", "1.5"}, {"label", "Closed-Loop Control Signal"}});
    axhline(Q_ss, {{"color", "blue"}, {"linestyle", "--"}, {"label", "Open-Loop (constant)"}});
    axvline(1000, {{"color", "orange"}, {"linestyle", ":"}, {"alpha", "0.7"}});
    
    xlabel("Time [s]");
    ylabel("Heater Power [W]");
    title("Control Effort: On-Off Controller Adapts to Disturbance");
    legend();
    grid(true);
    ylim(-5, 60);
    
    savefig("ch01_control_effort.svg");
    std::cout << "  ✓ Saved ch01_control_effort.svg" << std::endl;
    
    // ========================================================================
    // Summary Statistics
    // ========================================================================
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                       RESULTS SUMMARY                        ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════╣" << std::endl;
    
    // Calculate steady-state errors
    double ss_error_ol = std::abs(T_ref - T_openloop.back());
    double ss_error_cl = std::abs(T_ref - T_closedloop.back());
    double ss_error_ol_dist = std::abs(T_ref - T_openloop_dist.back());
    double ss_error_cl_dist = std::abs(T_ref - T_closedloop_dist.back());
    
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Scenario                    │ Open-Loop │ Closed-Loop      ║" << std::endl;
    std::cout << "║  ───────────────────────────────────────────────────────    ║" << std::endl;
    std::cout << "║  Normal operation (SS error) │   " << std::fixed << std::setprecision(1) 
              << std::setw(5) << ss_error_ol << "°C  │     " 
              << std::setw(5) << ss_error_cl << "°C         ║" << std::endl;
    std::cout << "║  With disturbance (SS error) │   " << std::setw(5) << ss_error_ol_dist 
              << "°C  │     " << std::setw(5) << ss_error_cl_dist << "°C         ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    std::cout << "\n✓ Chapter 1 simulation complete!" << std::endl;
    
    return 0;
}
```

> **🔍 Stop and Think — Mechanism Questions**
>
> Before moving on, answer these without re-reading:
> 1. In the simulation above, the closed-loop system corrects for disturbances but the open-loop system does not. *Why?* What specific signal carries the disturbance information back to the controller?
> 2. If the sensor has a 2-minute delay, the controller acts on *old* information. Will the system still converge to the setpoint? Could it oscillate? Explain the mechanism.
> 3. The error signal $e = T_{ref} - T$ is zero at steady state. Does this mean the heater power $Q(t)$ is zero? If not, where is the heat going?

---

## 1.8 Block Diagram Representation

### 1.8.1 Basic Elements

Control systems are represented using **block diagrams** with standard elements:

**Block (Transfer Function):**
```
         ┌─────────┐
  U(s) ──┤  G(s)   ├──▶ Y(s) = G(s)·U(s)
         └─────────┘
```

**Summing Junction:**
```
  R(s) ───┬───▶ ⊕ ───▶ E(s) = R(s) - Y(s)
          │    │
          │  - │
          │    ▼
  Y(s) ◀──┴────┘
```

**Pickoff Point:**
```
         ───────┬────────▶ (same signal)
                │
                └────────▶ (same signal)
```

### 1.8.2 Standard Feedback Configuration

```
                     ┌─────────────────────────────────────┐
                     │                                     │
   R(s) ───▶ ⊕ ─────▶│   G_c(s)    │   G_p(s)   │─────▶ Y(s)
             │  E(s) │  Controller │    Plant   │         │
             │       └─────────────────────────────────────┘
             │                                             │
             │             H(s)                            │
             └──────────┤ Sensor ├◀────────────────────────┘
                    -
```

**Key Signals:**
- $R(s)$ = Reference input (setpoint)
- $E(s)$ = Error = $R(s) - H(s)Y(s)$
- $Y(s)$ = Output (controlled variable)
- $G_c(s)$ = Controller transfer function
- $G_p(s)$ = Plant transfer function
- $H(s)$ = Sensor transfer function

### 1.8.3 Closed-Loop Transfer Function

For unity feedback ($H(s) = 1$):

$$\frac{Y(s)}{R(s)} = \frac{G_c(s)G_p(s)}{1 + G_c(s)G_p(s)}$$

Let $G(s) = G_c(s)G_p(s)$ (open-loop transfer function):

$$T(s) = \frac{G(s)}{1 + G(s)}$$

```cpp
// Compute closed-loop transfer function
TransferFunction G_open = G_controller * G_plant;
TransferFunction G_closed = feedback(G_open, TransferFunction(1.0));  // Unity feedback
```

---

## 1.9 Control System Design Process

### The Design Cycle

```
┌──────────────────────────────────────────────────────────────────┐
│                                                                  │
│    ┌─────────────┐     ┌─────────────┐     ┌─────────────┐      │
│    │  1. DEFINE  │────▶│  2. MODEL   │────▶│ 3. ANALYZE  │      │
│    │  Problem &  │     │  Physical   │     │  Open-Loop  │      │
│    │  Specs      │     │  System     │     │  System     │      │
│    └─────────────┘     └─────────────┘     └──────┬──────┘      │
│                                                    │             │
│                                                    ▼             │
│    ┌─────────────┐     ┌─────────────┐     ┌─────────────┐      │
│    │  6. DEPLOY  │◀────│ 5. VERIFY   │◀────│ 4. DESIGN   │      │
│    │  & Monitor  │     │  Simulate   │     │  Controller │      │
│    │             │     │  & Test     │     │             │      │
│    └─────────────┘     └─────────────┘     └─────────────┘      │
│         │                                        │               │
│         │              If not satisfied          │               │
│         │         ◀──────────────────────────────┘               │
│         │                                                        │
└─────────┴────────────────────────────────────────────────────────┘
```

### Design Specifications

| Category | Specification | Typical Value |
|----------|---------------|---------------|
| **Stability** | System must be stable | Required |
| **Steady-state** | Steady-state error | < 2% |
| **Transient** | Rise time | < 1 s |
| | Settling time | < 4 s |
| | Overshoot | < 10% |
| **Robustness** | Gain margin | > 6 dB |
| | Phase margin | > 45° |

---

## 1.10 Exercises

**Exercise 1.1 — Open-Loop vs Closed-Loop Identification** *(Conceptual)*

Choose a household appliance (e.g., microwave oven, washing machine, air conditioner, or refrigerator). For your chosen appliance:

(a) Identify whether its primary operation uses open-loop or closed-loop control. Justify your answer.  
(b) If it uses open-loop control, propose a modification that would make it closed-loop. What sensor would you add?  
(c) If it uses closed-loop control, describe a scenario where it effectively operates open-loop (e.g., sensor failure).

---

**Exercise 1.2 — Cruise Control Block Diagram** *(Conceptual/Analytical)*

Consider an automobile cruise control system that maintains a set vehicle speed.

(a) Draw a complete block diagram identifying:
   - The **plant** (what is being controlled)
   - The **sensor** (what measures the output)
   - The **controller** (what computes the control action)
   - The **actuator** (what physically acts on the plant)
   - The **reference input** and **output**

(b) Label each signal in the block diagram (e.g., desired speed, actual speed, error, throttle command).  
(c) Identify at least two disturbances that would affect this system.

---

**Exercise 1.3 — Temperature Control and Feedback** *(Conceptual)*

Consider a temperature-controlled laboratory where the goal is to maintain the room at exactly 22°C.

(a) List at least three disturbances that could cause the temperature to deviate from the setpoint.  
(b) Explain why open-loop control (e.g., running the heater at a fixed power level) would be inadequate.  
(c) Describe how feedback improves performance. What happens to steady-state error when feedback is applied?  
(d) What potential problem does feedback introduce that open-loop does not have? *(Hint: think about stability.)*

---

**Exercise 1.4 — First CppPlot Program** *(Computational/Coding)* ⭐

Write a CppPlot program that:

(a) Creates the first-order transfer function $G(s) = \frac{5}{s + 2}$.  
(b) Plots its unit step response from $t = 0$ to $t = 5$ seconds.  
(c) From the plot, estimate:
   - The DC gain (final value)
   - The time constant $\tau$
   - The rise time (10%–90%)

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;

int main() {
    // Create transfer function G(s) = 5/(s+2)
    TransferFunction G({5}, {1, 2});
    
    // Plot step response
    auto [t, y] = step_data(G, 5.0);
    plot(t, y);
    title("Step Response of G(s) = 5/(s+2)");
    xlabel("Time (s)");
    ylabel("Output");
    grid(true);
    show();
    
    return 0;
}
```

Verify your graphical estimates against the theoretical values: DC gain $= K = 5/2 = 2.5$, $\tau = 1/2 = 0.5$ s.

---

**Exercise 1.5 — Comparing First-Order Systems** *(Computational/Coding)*

Using CppPlot, compare the unit step responses of:

$$G_1(s) = \frac{1}{s+1} \qquad \text{and} \qquad G_2(s) = \frac{1}{s+5}$$

(a) Plot both responses on the same graph.  
(b) For each system, determine:
   - The time constant $\tau$
   - The steady-state value
   - The time to reach 98% of the final value ($\approx 4\tau$)

(c) Which system responds faster? Explain why in terms of the pole location.  
(d) Both systems have the same numerator. Why are the steady-state values different?

---

**Exercise 1.6 — Industrial Control Application Research** *(Open-Ended)*

Research one real-world industrial control application (e.g., chemical process control, autonomous vehicle steering, drone altitude control, or power grid frequency regulation).

Write a short report (300–500 words) that includes:

(a) A description of the physical system and the control objective.  
(b) Identification of the plant, sensor(s), actuator(s), and controller type.  
(c) A simplified block diagram of the control structure.  
(d) Key disturbances and how the control system handles them.  
(e) Why feedback control is essential for this application (what would happen without it?).

---

## 1.11 Chapter Summary

### Key Concepts

✅ **Control System:** Interconnection of components to achieve desired output behavior

✅ **Open-Loop:** No feedback; simple but cannot handle disturbances

✅ **Closed-Loop:** Uses feedback; compensates for disturbances and uncertainties

✅ **Components:** Plant, Sensor, Actuator, Controller

✅ **Block Diagrams:** Visual representation of signal flow

✅ **CppPlot:** C++ library for control system simulation and visualization

### What's Next

Chapter 2 will develop the mathematical tools for modeling physical systems:
- Differential equations → Transfer functions
- Mechanical, electrical, thermal systems
- State-space representation

---

## 1.12 Self-Assessment

### Checklist

- [ ] I can define open-loop and closed-loop control
- [ ] I can explain why feedback is beneficial
- [ ] I can identify components in a control system
- [ ] I can compile and run a CppPlot program
- [ ] I can interpret a block diagram

### Practice Problems

**1.1** Identify whether each system is open-loop or closed-loop:
   (a) Electric toaster with timer
   (b) Toilet tank fill mechanism
   (c) Automatic door opener (motion sensor)
   (d) Vending machine

**1.2** For a home heating system:
   (a) Identify the plant, sensor, actuator, and controller
   (b) Draw a block diagram
   (c) What are potential disturbances?

**1.3** Modify the temperature control program to:
   (a) Use a different reference temperature
   (b) Add a second disturbance (door opens at t = 2000s)
   (c) Compare results

**1.4** A car cruise control system:
   (a) What is the reference input?
   (b) What is the controlled variable?
   (c) What disturbances affect it?
   (d) Why is closed-loop control necessary?

### Problem Identification Exercises (Level 3-4)

**1.5 — What Is the Real Problem?**
A bakery oven must maintain 180°C ± 2°C. The baker complains that bread quality is inconsistent even though the thermostat "works fine."

(a) List at least three possible *root causes* that are NOT the thermostat itself.
(b) For each root cause, explain whether feedback control can solve it or whether the problem requires a different intervention (better insulation, different sensor placement, etc.).
(c) Write one sentence that identifies the *real* problem — using the format: "The system fails because [mechanism], not because [surface symptom]."

**1.6 — Mechanism vs. Procedure**
A student is given the block diagram of a temperature control system and asked: "Is this system stable?" The student says: "I'll just simulate it and see."

(a) What is wrong with this approach? (Hint: what does the student learn?)
(b) Explain, using only the concepts from this chapter (feedback, error, disturbance), *why* the system should or should not be stable — without running any simulation.
(c) Under what conditions would the feedback loop make things *worse* instead of better?

---

## References

1. Ogata, K. - Modern Control Engineering, Chapter 1
2. Franklin, G.F. et al. - Feedback Control of Dynamic Systems, Chapter 1
3. Dorf, R.C. & Bishop, R.H. - Modern Control Systems, Chapter 1

---

*Next Chapter: Mathematical Modeling of Dynamic Systems →*
