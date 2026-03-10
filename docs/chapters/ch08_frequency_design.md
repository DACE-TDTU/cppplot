# Chapter 8: Frequency Domain Controller Design

---

## 🎯 Learning Objectives and Outcomes

### Chapter Objectives

This chapter applies frequency-domain concepts to controller design, teaching systematic methods for designing lead, lag, and PID compensators to meet stability and performance specifications.

### Prerequisites
- Chapter 6: Frequency Response (Bode plots, margins)
- Chapter 7: Nyquist Criterion

---

## Learning Outcomes

After completing this chapter, students will be able to:

| Bloom's Level | Outcome |
|---------------|---------|
| **Remember** | List types of compensators and their characteristics |
| **Understand** | Explain how compensators affect frequency response |
| **Apply** | Design lead and lag compensators using Bode methods |
| **Analyze** | Analyze the effect of compensators on stability margins |
| **Evaluate** | Compare different compensator designs |
| **Create** | Design lead-lag compensators for complex systems |

---

## Why This Chapter Matters

> **The Real Problem:** Your motor controller tracks position accurately, but overshoots by 40% and takes 3 seconds to settle. The customer demands <10% overshoot and <0.5s settling time. You cannot change the motor, the sensor, or the power supply — you can only change the *controller*. How do you systematically reshape the controller's frequency response to meet these specifications?
>
> Chapters 6 and 7 gave you the language to *describe* frequency-domain behavior. This chapter gives you the tools to *change* it — to design compensators that add phase where you need it, boost gain where it matters, and leave the system stable.

---

## 8.1 Introduction to Compensator Design

### 8.1.1 Design Goals

Controller design in frequency domain aims to:

1. **Stability:** Ensure adequate gain and phase margins
2. **Steady-state accuracy:** Achieve required error constants
3. **Transient response:** Meet bandwidth requirements
4. **Noise rejection:** Limit high-frequency gain

### 8.1.2 Types of Compensators

| Type | Transfer Function | Primary Effect |
|------|-------------------|----------------|
| Lead | $\frac{s+z}{s+p}$, $z < p$ | Adds phase at crossover |
| Lag | $\frac{s+z}{s+p}$, $z > p$ | Increases low-freq gain |
| Lead-Lag | Combination | Both effects |
| PID | $K_p + \frac{K_i}{s} + K_d s$ | Versatile control |

---

## 8.2 Lead Compensator Design

### 8.2.1 Lead Compensator Structure

$$G_c(s) = K_c \frac{s + z}{s + p} = K_c \frac{s + \frac{1}{T}}{s + \frac{1}{\alpha T}}$$

where $\alpha < 1$ (typically 0.05 to 0.5)

### 8.2.2 Frequency Response Properties

**Maximum phase lead:**
$$\phi_{max} = \sin^{-1}\frac{1-\alpha}{1+\alpha}$$

**Frequency of maximum phase:**
$$\omega_m = \frac{1}{T\sqrt{\alpha}}$$

### 8.2.3 Design Procedure

1. **Determine gain K** for steady-state error requirement
2. **Calculate required phase margin** (add 5°-12° safety margin)
3. **Find α** from $\phi_{max}$ equation
4. **Set** $\omega_m$ at new gain crossover frequency
5. **Calculate T** from $\omega_m$ equation
6. **Verify** design meets specifications

### 8.2.4 Design Example

**Requirement:** For $G(s) = \frac{14}{s(s+2)}$, design lead compensator for PM ≥ 50°.

**Solution:**
1. Current PM ≈ 30° (from Bode plot, $\omega_{gc} \approx 3.5$ rad/s)
2. Need additional 50° - 30° + 10° = 30° phase
3. $\alpha = \frac{1 - \sin(30°)}{1 + \sin(30°)} = 0.33$
4. Choose $\omega_m$ at new crossover
5. Calculate compensator parameters

---

## 8.3 Lag Compensator Design

### 8.3.1 Lag Compensator Structure

$$G_c(s) = K_c \frac{s + z}{s + p} = K_c \frac{s + \frac{1}{T}}{s + \frac{1}{\beta T}}$$

where $\beta > 1$ (typically 3 to 10)

### 8.3.2 Design Philosophy

**Key insight:** Lag compensator adds gain at low frequencies without significantly changing phase margin.

- Pole and zero placed at **low frequencies**
- Phase contribution at crossover frequency is negligible

### 8.3.3 Design Procedure

1. **Set gain K** for desired error constant
2. **Find frequency** where phase margin is satisfactory
3. **Calculate β** to reduce gain to 0 dB at that frequency
4. **Place zero** 1 decade below new crossover
5. **Place pole** at $p = z/\beta$
6. **Verify** specifications

---

## 8.4 Lead-Lag Compensator

### 8.4.1 When to Use

Use lead-lag when:
- Lead alone cannot provide enough phase margin
- Lag alone doesn't meet bandwidth requirements
- Both error constant and transient response are important

### 8.4.2 Structure

$$G_c(s) = K \cdot \underbrace{\frac{s + z_1}{s + p_1}}_{\text{Lead}} \cdot \underbrace{\frac{s + z_2}{s + p_2}}_{\text{Lag}}$$

### 8.4.3 Design Strategy

1. **Design lag section** first for steady-state error
2. **Design lead section** for phase margin
3. **Iterate** if necessary

---

## 8.5 PID Controller in Frequency Domain

### 8.5.1 Ideal PID Transfer Function

$$G_{PID}(s) = K_p + \frac{K_i}{s} + K_d s = K_p\left(1 + \frac{1}{T_i s} + T_d s\right)$$

### 8.5.2 Practical PID (with Filter)

$$G_{PID}(s) = K_p\left(1 + \frac{1}{T_i s} + \frac{T_d s}{1 + T_d s/N}\right)$$

N typically 8-20 to limit high-frequency gain.

### 8.5.3 Frequency Domain Interpretation

| Term | Effect | Frequency Range |
|------|--------|-----------------|
| $K_p$ | Proportional gain | All frequencies |
| $K_i/s$ | Phase lag, error elimination | Low frequency |
| $K_d s$ | Phase lead, damping | Mid frequency |

### 8.5.4 Ziegler-Nichols Frequency Method

1. Find **ultimate gain** $K_u$ (gain at marginal stability)
2. Find **ultimate period** $T_u$
3. Use table for PID parameters:

| Controller | $K_p$ | $T_i$ | $T_d$ |
|------------|-------|-------|-------|
| P | 0.5$K_u$ | - | - |
| PI | 0.45$K_u$ | $T_u$/1.2 | - |
| PID | 0.6$K_u$ | $T_u$/2 | $T_u$/8 |

### 8.5.5 From PID Output to Physical Actuation

> **Critical Reality Check:** The PID controller computes a **desired** control signal. This must be converted to **actual** physical actuation through power electronics.

**Complete Signal Flow:**

```
┌─────────────────────────────────────────────────────────────────────────┐
│            FROM PID OUTPUT TO MOTOR SHAFT                               │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   ┌─────┐     ┌──────────┐     ┌──────────┐     ┌──────┐     ┌──────┐ │
│   │ PID │────►│   PWM    │────►│ H-Bridge │────►│Motor │────►│ Load │ │
│   │     │     │Modulator │     │(Power)   │     │      │     │      │ │
│   └─────┘     └──────────┘     └──────────┘     └──────┘     └──────┘ │
│      │              │               │               │                  │
│      ▼              ▼               ▼               ▼                  │
│   u_desired      D (duty)      V_actual         ω (speed)             │
│   (0-10V or     (0-100%)       (pulsed)                               │
│   -10 to +10V)                                                         │
│                                                                         │
│   EXAMPLE:                                                              │
│   u_des = 7.5V → D = 62.5% → V_avg = 7.5V → ω = 750 rpm               │
│   (if V_supply = 12V)                                                  │
└─────────────────────────────────────────────────────────────────────────┘
```
**Signal Dictionary — Motor PID Control with H-Bridge**

| Signal | Symbol | Unit | Physical Meaning | Sensor/Actuator |
|--------|--------|------|------------------|-----------------|
| Speed reference (setpoint) | $r(t)$ | rpm or rad/s | Desired motor speed | Operator / supervisory controller |
| Measured speed (feedback) | $y(t)$ | rpm or rad/s | Actual motor speed | Encoder / tachometer |
| Error signal | $e = r - y$ | rpm or rad/s | How far the motor is from the desired speed | Computed (summing junction) |
| PID output (desired voltage) | $u_{des}$ | V (0–10 V) | Controller’s corrective command | PID algorithm (MCU / DSP) |
| Duty cycle | $D = u_{des}/V_{supply}$ | — (0–1) | PWM fraction — converts voltage command to switching pattern | PWM timer peripheral |
| Actual motor voltage | $V_{actual}$ | V (pulsed) | Pulsed voltage from H-Bridge transistors | H-Bridge (4 MOSFETs) |
| Average motor voltage | $V_{avg} = D \times V_{supply}$ | V | Effective DC voltage the motor “sees” | (Time-averaged) |
| Supply voltage | $V_{supply}$ | V | Power rail (e.g., 12 V battery) | Power supply / battery |

> **The signal chain:** $r \to e \to u_{des} \to D \to V_{actual} \to \omega \to y$. Each arrow is a physical transformation with a specific gain. The compensator design (lead, lag, PID) shapes *how* $e$ becomes $u_{des}$. The H-Bridge and motor are the *plant* — they convert voltage to speed.
**DC Motor with H-Bridge: Detailed View**

```
       V_supply (12V)
           │
     ┌─────┴─────┐
     │           │
   ┌─┴─┐       ┌─┴─┐
   │Q1 │       │Q3 │     Q1,Q4 ON: Forward current
   │   │       │   │     Q2,Q3 ON: Reverse current
   └─┬─┘       └─┬─┘
     │   ┌───┐   │
     ├───┤ M ├───┤       PWM on Q1 (Q4 always ON):
     │   └───┘   │       V_motor = D × V_supply
   ┌─┴─┐       ┌─┴─┐
   │Q2 │       │Q4 │
   │   │       │   │
   └─┬─┘       └─┬─┘
     │           │
     └─────┬─────┘
           │
          GND
```

**Why $u_{desired} \neq u_{actual}$:**

| Effect | Cause | Impact |
|--------|-------|--------|
| Voltage droop | Supply impedance, transistor R_on | $V_{actual} < V_{desired}$ under load |
| Dead-time | Prevent shoot-through | Voltage distortion near zero |
| PWM ripple | Discrete switching | Torque ripple, acoustic noise |
| Saturation | $u_{desired} > V_{supply}$ | Control signal clipping |
| Temperature | MOSFET R_on increases | Efficiency drops |

**Practical Implementation Example (Arduino-style):**

```cpp
// PID output: u_desired in volts (-12V to +12V range)
// PWM: 8-bit (0-255), Frequency: 20 kHz
// Supply: 12V

void applyControlSignal(double u_desired) {
    // Convert desired voltage to duty cycle
    double V_supply = 12.0;
    
    // Saturate to available voltage range
    u_desired = constrain(u_desired, -V_supply, V_supply);
    
    // Determine direction
    if (u_desired >= 0) {
        digitalWrite(DIR_PIN, HIGH);  // Forward
    } else {
        digitalWrite(DIR_PIN, LOW);   // Reverse
        u_desired = -u_desired;       // Make positive for PWM
    }
    
    // Convert to PWM duty cycle (0-255)
    // Note: This assumes V_actual ≈ D × V_supply (ideal case)
    int pwm_value = (int)(255.0 * u_desired / V_supply);
    
    analogWrite(PWM_PIN, pwm_value);
}
```

**Compensating for Power Stage:**

| Method | Description | When to Use |
|--------|-------------|-------------|
| Feedforward | Add $V_{supply}$ measurement, adjust D | Battery-powered systems |
| Inner current loop | Control current, not voltage | High-performance drives |
| Dead-time compensation | Inject correction signal | Precision applications |
| Voltage feedback | Measure actual motor voltage | When accuracy critical |

---

## 8.6 Electrical and Telecommunications Design Examples

### 8.6.1 Voltage-Mode Buck Converter Compensation

**Problem:** Design a Type III compensator for a voltage-mode controlled Buck converter.

**Plant Transfer Function (Control-to-Output):**

$$G_{vd}(s) = V_{in} \cdot \frac{1 + sR_C C}{s^2 LC + s(L/R + R_C C) + 1}$$

For typical values: $V_{in}=12V$, $L=100\mu H$, $C=100\mu F$, $R=5\Omega$, $R_C=20m\Omega$:

- LC resonance: $f_0 = \frac{1}{2\pi\sqrt{LC}} \approx 1.6 \text{ kHz}$
- ESR zero: $f_z = \frac{1}{2\pi R_C C} \approx 80 \text{ kHz}$

**Type III Compensator Design:**

$$G_c(s) = K_c \cdot \frac{(1 + s/\omega_{z1})(1 + s/\omega_{z2})}{s(1 + s/\omega_{p1})(1 + s/\omega_{p2})}$$

**Design Procedure:**

1. **Set crossover frequency:** $f_c = f_{sw}/10 = 10 \text{ kHz}$ (for $f_{sw}=100$ kHz)
2. **Place compensator zeros** at or below LC resonance to cancel phase lag
3. **Place one compensator pole** at ESR zero location
4. **Place second pole** at half switching frequency

```
┌────────────────────────────────────────────────────────────────────────┐
│        BUCK CONVERTER LOOP GAIN DESIGN                                 │
├────────────────────────────────────────────────────────────────────────┤
│                                                                        │
│  Magnitude (dB)                                                        │
│      │                                                                 │
│   60 │──┐                                                             │
│      │   \  Compensator                                               │
│   40 │    \──────────────────────────────────────────                 │
│      │     \                          /                               │
│   20 │      \   Plant resonance →   /                                │
│      │       \    ┌──────┐         /                                  │
│    0 │────────\──│ -40dB │────────●───────────────── Loop T(jω)      │
│      │         \ │/decade│       / ← f_c = 10 kHz                     │
│  -20 │          \└───────┘      /                                     │
│      │           \             /  Plant                               │
│  -40 │            \───────────/                                       │
│      │                                                                │
│      └──────┬──────┬──────┬──────┬──────┬──────┬──► f (Hz)           │
│          100     1k    10k   100k    1M                               │
│                         ↑                                             │
│                    Crossover                                          │
│                    (PM ≥ 45°)                                         │
│                                                                        │
│   DESIGN EQUATIONS:                                                   │
│   ω_z1, ω_z2 ≈ ω_0/2    (compensate LC resonance)                    │
│   ω_p1 ≈ ω_esr          (cancel ESR zero)                            │
│   ω_p2 ≈ π·f_sw         (attenuate switching noise)                  │
└────────────────────────────────────────────────────────────────────────┘
```

**CppPlot Implementation:**

```cpp
#include <cppplot/cppplot.hpp>
#include <cmath>

using namespace cppplot;

int main() {
    // Buck converter parameters
    double Vin = 12.0, L = 100e-6, C = 100e-6;
    double R = 5.0, Rc = 20e-3;  // ESR
    double fsw = 100e3, Vm = 1.0;
    
    // Derived parameters
    double w0 = 1.0 / std::sqrt(L*C);  // LC resonance
    double Q = R * std::sqrt(C/L);
    double wz_esr = 1.0 / (Rc*C);       // ESR zero
    
    // Type III compensator design for 10 kHz crossover
    double fc = 10e3;
    double wc = 2 * M_PI * fc;
    
    // Place zeros at LC resonance / 2
    double wz1 = w0 / 2;
    double wz2 = w0 / 2;
    // Place poles at ESR zero and half switching frequency
    double wp1 = wz_esr;
    double wp2 = M_PI * fsw;
    
    double Kc = 0.5;  // Adjust empirically
    
    std::vector<double> f_hz, mag_loop, phase_loop;
    
    for (double f = 10; f <= 1e6; f *= 1.1) {
        double w = 2 * M_PI * f;
        std::complex<double> jw(0, w);
        f_hz.push_back(f);
        
        // Plant: Gvd(s)
        std::complex<double> num_p = Vin * (1.0 + jw*Rc*C);
        std::complex<double> den_p = jw*jw*L*C + jw*(L/R + Rc*C) + 1.0;
        std::complex<double> Gvd = num_p / den_p;
        
        // Compensator: Type III
        std::complex<double> num_c = Kc * (1.0 + jw/wz1) * (1.0 + jw/wz2);
        std::complex<double> den_c = jw * (1.0 + jw/wp1) * (1.0 + jw/wp2);
        std::complex<double> Gc = num_c / den_c;
        
        // Loop gain
        std::complex<double> T = Gc * Gvd / Vm;
        
        mag_loop.push_back(20*std::log10(std::abs(T)));
        phase_loop.push_back(std::arg(T) * 180/M_PI);
    }
    
    figure(800, 600);
    
    subplot(2, 1, 1);
    plot(f_hz, mag_loop, "-", opts({{"color", "blue"}, {"label", "Loop Gain T(jw)"}}));
    axhline(0, opts({{"color", "black"}, {"linestyle", "--"}, {"linewidth", "0.5"}}));
    xscale("log");
    ylabel("Magnitude (dB)");
    title("Buck Converter Type III Compensation");
    grid(true);
    legend(true);
    
    subplot(2, 1, 2);
    plot(f_hz, phase_loop, "-", opts({{"color", "red"}, {"label", "Phase"}}));
    axhline(-180, opts({{"color", "black"}, {"linestyle", "--"}}));
    xscale("log");
    xlabel("Frequency (Hz)");
    ylabel("Phase (deg)");
    grid(true);
    
    savefig("ch08_buck_type3_compensation.svg");
    return 0;
}
```

### 8.6.2 PLL Loop Filter Design

**Problem:** Design a loop filter for a charge-pump PLL with specified bandwidth and phase margin.

**Charge-Pump PLL Open-Loop:**

$$G_{OL}(s) = \frac{I_{cp} \cdot K_{vco}}{N \cdot s} \cdot F(s)$$

Where $I_{cp}$ is charge pump current, $K_{vco}$ is VCO gain (rad/s/V), and $N$ is the feedback divider ratio.

**Second-Order Loop Filter (Type II):**

$$F(s) = \frac{1 + sR_2C_2}{s(C_1 + C_2)\left(1 + \frac{sR_2 C_1 C_2}{C_1 + C_2}\right)}$$

For simplified analysis with $C_1 >> C_2$:
$$F(s) \approx \frac{1 + s\tau_2}{s\tau_1}$$

where $\tau_1 = R_2 C_1$, $\tau_2 = R_2 C_2$.

**Design Equations for Specified Bandwidth and Damping:**

Given natural frequency $\omega_n$ and damping ratio $\zeta$:

$$\tau_2 = \frac{2\zeta}{\omega_n}$$

$$\omega_n^2 = \frac{I_{cp} K_{vco}}{N \tau_1}$$

**Phase Margin Approximation:**

$$PM \approx \arctan(2\zeta) + \arctan\left(\frac{\omega_c \tau_2}{1 - \omega_c^2/\omega_z^2}\right)$$

For $\zeta = 0.707$: PM ≈ 65°

```
┌────────────────────────────────────────────────────────────────────────┐
│           CHARGE-PUMP PLL LOOP FILTER DESIGN                          │
├────────────────────────────────────────────────────────────────────────┤
│                                                                        │
│   Circuit:            Bode Plot:                                      │
│                                                                        │
│   From PFD      │     Magnitude                                       │
│        │        │         │                                           │
│        ▼        │      40 │                                           │
│   ┌────┬────┐   │         │   -20 dB/dec                             │
│   │    │    │   │      20 │───────\                                  │
│   │   ═╧═   │   │         │        \    -40 dB/dec                   │
│   │   C1    │   │       0 │─────────●──────\──────────               │
│   │    │    │   │         │         ↑       \                        │
│   │   ─┼─   │   │     -20 │         ωc       \                       │
│   │  R2│C2  │   │         │                   \                      │
│   │   ═╧═   │   │     -40 │────────────────────\──────               │
│   │    │    │   │         │                                          │
│   └────┴────┘   │         └───────┬────┬────┬────┬──► ω             │
│   To VCO        │               ωz   ωc   ωp                         │
│                 │                                                     │
│   Design:       │     ωz = 1/τ2 (zero from R2-C2)                   │
│   C1 >> C2      │     ωp = (C1+C2)/(R2·C1·C2) ≈ 1/(R2·C2·C1/C1)    │
│   (10-20×)      │                                                     │
└────────────────────────────────────────────────────────────────────────┘
```

**CppPlot Implementation:**

```cpp
#include <cppplot/cppplot.hpp>
#include <cmath>
#include <iostream>

using namespace cppplot;

int main() {
    // PLL specifications
    double f_ref = 10e6;          // 10 MHz reference
    double f_vco = 1e9;           // 1 GHz VCO center frequency
    int N = 100;                  // Divider ratio
    double Icp = 1e-3;            // 1 mA charge pump
    double Kvco = 2*M_PI*100e6;   // 100 MHz/V VCO gain
    
    // Loop bandwidth and damping specifications
    double fc = 100e3;            // 100 kHz loop bandwidth
    double zeta = 0.707;          // Damping ratio
    
    // Calculate loop filter components
    double wc = 2 * M_PI * fc;
    double wn = wc / std::sqrt(2*zeta*zeta + 1 + 
                std::sqrt(std::pow(2*zeta*zeta+1,2) + 1));
    
    double tau2 = 2 * zeta / wn;
    double tau1 = Icp * Kvco / (N * wn * wn);
    
    // Component values (choose C1, calculate R2 and C2)
    double C1 = 10e-9;   // 10 nF
    double R2 = tau1 / C1;
    double C2 = tau2 / R2;
    
    std::cout << "Loop Filter Components:\n";
    std::cout << "C1 = " << C1*1e9 << " nF\n";
    std::cout << "R2 = " << R2 << " Ohm\n";
    std::cout << "C2 = " << C2*1e12 << " pF\n";
    
    // Generate Bode plot
    std::vector<double> f_hz, mag_dB, phase_deg;
    
    for (double f = 1e3; f <= 10e6; f *= 1.1) {
        double w = 2 * M_PI * f;
        std::complex<double> jw(0, w);
        f_hz.push_back(f);
        
        // Loop filter F(s)
        std::complex<double> F = (1.0 + jw*tau2) / (jw*tau1);
        
        // Open-loop gain
        std::complex<double> G = (Icp * Kvco / (double)N) / jw * F;
        
        mag_dB.push_back(20*std::log10(std::abs(G)));
        phase_deg.push_back(std::arg(G) * 180/M_PI);
    }
    
    figure(800, 600);
    
    subplot(2, 1, 1);
    plot(f_hz, mag_dB, "-", opts({{"color", "blue"}, {"label", "Open-Loop |G(jw)|"}}));
    axhline(0, opts({{"color", "black"}, {"linestyle", "--"}, {"linewidth", "0.5"}}));
    xscale("log");
    ylabel("Magnitude (dB)");
    title("Charge-Pump PLL Loop Gain (BW = 100 kHz, zeta = 0.707)");
    grid(true);
    legend(true);
    
    subplot(2, 1, 2);
    plot(f_hz, phase_deg, "-", opts({{"color", "red"}, {"label", "Phase"}}));
    axhline(-180, opts({{"color", "black"}, {"linestyle", "--"}}));
    xscale("log");
    xlabel("Frequency (Hz)");
    ylabel("Phase (deg)");
    grid(true);
    
    savefig("ch08_pll_loop_filter.svg");
    return 0;
}
```

### 8.6.3 Active Filter Frequency Compensation

**Problem:** Design a Butterworth low-pass filter with op-amp implementation and analyze the effect of finite GBW.

**Ideal 2nd-Order Butterworth:**

$$H(s) = \frac{\omega_c^2}{s^2 + \sqrt{2}\omega_c s + \omega_c^2}$$

**Sallen-Key Implementation:**

```
┌─────────────────────────────────────────────────────────────────────────┐
│           SALLEN-KEY LOW-PASS FILTER                                    │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│         R1            R2                                               │
│   Vin ─/\/\/\─┬──────/\/\/\─┬────────┐                                │
│               │              │        │                                │
│              ═╧═ C1          │     ┌──┴──┐                             │
│               │              │     │  +  │                             │
│               ▼              │  ┌──┤     ├──┬── Vout                   │
│              GND             │  │  │  -  │  │                          │
│                              │  │  └─────┘  │                          │
│                             ═╧═ C2    ▲     │                          │
│                              │        │     │                          │
│                              ▼       ─┴─────┘                          │
│                             GND     (Unity gain)                       │
│                                                                         │
│   Transfer Function (Unity Gain):                                      │
│                                                                         │
│              1/(R1·R2·C1·C2)                                           │
│   H(s) = ─────────────────────────────                                 │
│          s² + s(1/R1·C1 + 1/R2·C1) + 1/(R1·R2·C1·C2)                  │
│                                                                         │
│   For Butterworth (Q = 1/√2): Choose R1=R2=R, C1=2C, C2=C             │
└─────────────────────────────────────────────────────────────────────────┘
```

**Effect of Finite Op-Amp GBW:**

The actual transfer function becomes:

$$H_{actual}(s) = H_{ideal}(s) \cdot \frac{1}{1 + \frac{H_{ideal}(s)}{A_{OL}(s)}}$$

For high-Q filters near cutoff, this causes:
- Reduced Q (more damping)
- Slight shift in cutoff frequency
- Additional high-frequency rolloff

**Design Rule:** $GBW > 100 \cdot f_c \cdot Q$ for accurate response.

### 8.6.4 Communication System Equalizer Design

**Problem:** Design a frequency-domain equalizer for a dispersive channel.

**Channel Model (Two-Tap):**

$$H_{ch}(j\omega) = 1 + \alpha e^{-j\omega T_d}$$

Where $\alpha = 0.5$ is the multipath coefficient and $T_d = 1\mu s$ is the delay.

**Zero-Forcing (ZF) Equalizer:**

$$H_{eq}(j\omega) = \frac{1}{H_{ch}(j\omega)}$$

**Problem:** Noise enhancement at frequency nulls.

**MMSE Equalizer:**

$$H_{eq,MMSE}(j\omega) = \frac{H_{ch}^*(j\omega)}{|H_{ch}(j\omega)|^2 + \frac{1}{SNR}}$$

This balances ISI reduction against noise enhancement.

```cpp
#include <cppplot/cppplot.hpp>
#include <cmath>

using namespace cppplot;

int main() {
    // Channel parameters
    double alpha = 0.5;     // Multipath coefficient
    double Td = 1e-6;       // 1 us delay
    double SNR_dB = 20;     // 20 dB SNR
    double SNR_lin = std::pow(10, SNR_dB/10);
    
    std::vector<double> f_MHz, mag_ch, mag_zf, mag_mmse;
    
    for (double f = 0.1e6; f <= 5e6; f += 0.1e6) {
        double w = 2 * M_PI * f;
        f_MHz.push_back(f / 1e6);
        
        // Channel: H_ch = 1 + alpha*exp(-j*w*Td)
        std::complex<double> H_ch = 1.0 + alpha * std::exp(
            std::complex<double>(0, -w*Td));
        
        // ZF equalizer
        std::complex<double> H_zf = 1.0 / H_ch;
        
        // MMSE equalizer
        std::complex<double> H_mmse = std::conj(H_ch) / 
            (std::norm(H_ch) + 1.0/SNR_lin);
        
        mag_ch.push_back(20*std::log10(std::abs(H_ch)));
        mag_zf.push_back(20*std::log10(std::abs(H_zf)));
        mag_mmse.push_back(20*std::log10(std::abs(H_mmse)));
    }
    
    figure(800, 500);
    plot(f_MHz, mag_ch, "-", opts({{"color", "blue"}, {"label", "Channel"}}));
    plot(f_MHz, mag_zf, "--", opts({{"color", "red"}, {"label", "ZF Equalizer"}}));
    plot(f_MHz, mag_mmse, "-", opts({{"color", "green"}, {"label", "MMSE Equalizer"}}));
    xlabel("Frequency (MHz)");
    ylabel("Magnitude (dB)");
    title("Channel Equalization: ZF vs MMSE");
    legend(true);
    grid(true);
    savefig("ch08_channel_equalizer.svg");
    
    return 0;
}
```

### 8.6.5 Summary: Frequency Design in EE and Telecom

| Application | Compensator Type | Key Design Criterion |
|-------------|-----------------|---------------------|
| **Buck converter** | Type III | Crossover < fsw/10, PM > 45° |
| **PLL** | 2nd-order lag-lead | BW vs. jitter tradeoff |
| **Active filters** | Sallen-Key, etc. | GBW > 100·fc·Q |
| **Channel equalizer** | MMSE | SNR-dependent balance |

**Cross-Domain Insights:**

1. **Loop shaping is universal:** Whether designing power converter compensation or PLL filters, the goal is shaping |T(jω)| for stability and performance.

2. **Fundamental tradeoffs:** 
   - Fast response ↔ noise sensitivity
   - Disturbance rejection ↔ stability margin
   - Equalization depth ↔ noise enhancement

3. **Non-minimum phase systems:** RHP zeros (e.g., boost converter) fundamentally limit achievable bandwidth - this applies to both power electronics and communication systems.

---

## 8.7 Loop Shaping

### 8.7.1 Concept

**Loop shaping** designs controller by directly shaping the open-loop frequency response.

### 8.7.2 Desired Loop Shape

| Frequency Range | Desired Behavior | Reason |
|-----------------|------------------|--------|
| Low | High gain | Track references, reject disturbances |
| Crossover | -20 dB/decade slope | Ensure stability |
| High | Low gain | Attenuate noise |

### 8.7.3 Sensitivity Functions

**Sensitivity function:**
$$S(s) = \frac{1}{1 + G(s)K(s)}$$

**Complementary sensitivity:**
$$T(s) = \frac{G(s)K(s)}{1 + G(s)K(s)}$$

**Relationship:** $S(s) + T(s) = 1$

---

## 8.8 Design Trade-offs

### 8.8.1 Fundamental Limitations

**Bode's Sensitivity Integral:**
$$\int_0^{\infty} \ln|S(j\omega)|d\omega = \pi\sum_k \text{Re}(p_k)$$

where $p_k$ are RHP poles.

**Implication:** Reducing sensitivity at some frequencies increases it at others.

### 8.8.2 Waterbed Effect

- If we push down sensitivity at low frequencies
- It must rise at high frequencies
- More severe for systems with RHP poles

### 8.8.3 Practical Guidelines

| Requirement | Constraint |
|-------------|------------|
| Fast response | Requires high bandwidth, more noise sensitivity |
| Robustness | Limits achievable bandwidth |
| Low overshoot | Requires PM > 45° |
| Good disturbance rejection | Requires high low-frequency gain |

**Design Rules of Thumb:**

1. **Crossover frequency:** Set $\omega_{gc}$ to achieve desired bandwidth
2. **Phase margin:** Target PM = 45°-60° for good transient response
3. **Gain margin:** Target GM ≥ 6 dB for robustness
4. **Slope at crossover:** -20 dB/decade for single-crossover stability
5. **Low-frequency gain:** Maximize for disturbance rejection
6. **High-frequency rolloff:** -40 dB/decade or steeper for noise rejection

---

## 📝 Exercises

### Exercise 8.1 — Lead Compensator Design

For the plant:

$$G(s) = \frac{1}{s(s+1)}$$

Design a lead compensator $C(s) = K_c \dfrac{s + z}{s + p}$ (with $p > z > 0$) to achieve:
- **Phase margin:** PM ≥ 50°
- **Velocity error constant:** $K_v \geq 10$

**(a)** From the $K_v$ requirement, determine the required DC loop gain $K_c$.

**(b)** Plot (or compute) the Bode diagram of $K_c \cdot G(s)$ and find the uncompensated PM.

**(c)** Determine the additional phase lead $\phi_{\max}$ needed (add a safety margin of ~5°–10°).

**(d)** Compute $\alpha = (1 - \sin\phi_{\max}) / (1 + \sin\phi_{\max})$ and place the compensator's maximum phase at the new gain crossover frequency.

**(e)** Compute the compensator zero $z$ and pole $p$, and write the final $C(s)$.

**(f)** Verify that the compensated system meets both specifications.

---

### Exercise 8.2 — Lag Compensator Design

For the plant:

$$G(s) = \frac{10}{s(s+5)}$$

Design a lag compensator $C(s) = K_c \dfrac{s + z}{s + p}$ (with $z > p > 0$) to achieve:
- **Phase margin:** PM ≥ 45°
- The gain crossover frequency $\omega_{gc}$ should not change significantly.

**(a)** Plot the Bode diagram of $G(s)$ and identify the current $\omega_{gc}$ and PM.

**(b)** Find the frequency where the phase of $G(j\omega)$ equals $-180° + 45° + 5°$ (the desired PM plus a safety margin). This will be the new $\omega_{gc}$.

**(c)** Determine how much gain reduction (in dB) is needed at the new $\omega_{gc}$. This sets the ratio $z/p$.

**(d)** Place the lag compensator zero $z$ at $\omega_{gc}/10$ (one decade below crossover) and compute $p = z \cdot \alpha$.

**(e)** Verify the compensated Bode plot meets PM ≥ 45°.

---

### Exercise 8.3 — Lead vs. Lag Comparison

For the plant:

$$G(s) = \frac{5}{s(s+2)}$$

**(a)** Design a **lead compensator** to achieve PM ≥ 45° (following the procedure from Exercise 8.1).

**(b)** Design a **lag compensator** to achieve PM ≥ 45° (following the procedure from Exercise 8.2).

**(c)** For each design, determine:
- Closed-loop bandwidth $\omega_{BW}$
- Gain margin $GM$
- Step response rise time $t_r$ and overshoot $M_p$

**(d)** Summarize the trade-offs: which compensator gives better bandwidth? Better low-frequency performance? Better noise rejection?

---

### Exercise 8.4 — Lead-Lag Compensator Design

For the plant:

$$G(s) = \frac{1}{s(s+1)(s+5)}$$

Design a lead-lag compensator to achieve:
- **Velocity error constant:** $K_v \geq 20$
- **Phase margin:** PM ≥ 45°

**(a)** Determine the gain $K$ needed for $K_v = 20$. Plot the Bode of $K \cdot G(s)$.

**(b)** Design the **lag section** first: reduce gain at intermediate frequencies so that the lead section can provide adequate PM.

**(c)** Design the **lead section** to boost the phase at the new gain crossover frequency.

**(d)** Combine: $C(s) = C_{\text{lead}}(s) \cdot C_{\text{lag}}(s)$. Verify both $K_v$ and PM specifications.

**(e)** Plot the closed-loop step response and verify satisfactory transient performance.

---

### Exercise 8.5 — CppPlot Design Verification

Implement and verify your lead compensator from Exercise 8.1 using CppPlot:

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;

int main() {
    // Plant: G(s) = 1/(s(s+1))
    TransferFunction G({1}, {1, 1, 0});
    
    // Lead compensator: C(s) = Kc*(s+z)/(s+p)
    // Student task: Fill in your Kc, z, p values
    double Kc = ???, z = ???, p = ???;
    TransferFunction C({Kc, Kc*z}, {1, p});
    
    auto open_loop = C * G;
    auto closed_loop = feedback(open_loop, TransferFunction({1}, {1}));
    
    // Bode plot
    figure();
    bode(open_loop);
    title("Compensated Open-Loop Bode Plot");
    savefig("compensated_bode.svg");
    
    // Step response
    figure();
    step(closed_loop);
    title("Closed-Loop Step Response");
    savefig("closed_loop_step.svg");
}
```

**(a)** Fill in your compensator values and run the code.

**(b)** Read off PM and $\omega_{gc}$ from the Bode plot. Do they meet specs?

**(c)** From the step response, measure $t_r$, $M_p$, and $t_s$.

---

### Exercise 8.6 — PID Controller in Frequency Domain

A PID controller has the parallel form:

$$C(s) = K_p + \frac{K_i}{s} + K_d s$$

**(a)** Combine into a single transfer function: $C(s) = \dfrac{K_d s^2 + K_p s + K_i}{s}$.

**(b)** For $K_p = 10$, $K_i = 2$, $K_d = 3$, express $C(s)$ in factored form $C(s) = K_d \dfrac{(s + z_1)(s + z_2)}{s}$ and find $z_1, z_2$.

**(c)** Sketch the Bode plot of $C(s)$. Identify the integrator slope, the two break frequencies, and the high-frequency behavior.

**(d)** Implement with CppPlot:

```cpp
TransferFunction C_pid({Kd, Kp, Ki}, {1, 0});
bode(C_pid);
```

**(e)** Discuss: Why is a practical PID usually implemented with a derivative filter $C_d(s) = K_d s / (\tau_f s + 1)$ instead of pure $K_d s$?

---

### Exercise 8.7 — PLL Loop Filter Design ⭐

A Phase-Locked Loop (PLL) has an open-loop transfer function:

$$G_{OL}(s) = \frac{K_{VCO} \cdot K_{PD} \cdot F(s)}{s}$$

where $K_{VCO} \cdot K_{PD} = 2\pi \times 10^6$ rad/s and $F(s)$ is the loop filter.

**(a)** For a target bandwidth of $\omega_{BW} = 2\pi \times 100 \times 10^3$ rad/s and PM = 60°, design a lead-type loop filter:

$$F(s) = \frac{1 + s\tau_2}{1 + s\tau_1}$$

**(b)** Determine $\tau_1$ and $\tau_2$ such that the gain crossover occurs at $\omega_{BW}$ with the desired PM.

**(c)** Plot the open-loop Bode diagram and verify the design.

**(d)** Simulate the PLL step response (phase step input) and measure the lock-in time.

### Problem Identification Exercises (Level 3-4)

**Exercise 8.8 — What Is the Real Problem?**
A motor speed controller is designed with a lead compensator that provides PM = 55° and $\omega_{gc} = 50$ rad/s. The system works perfectly with no load. Under full load (increased inertia), the motor oscillates.

(a) Explain how increased load inertia changes the plant transfer function $G(s)$. Which parameter(s) change?
(b) How does this change affect the Bode plot (gain crossover shifts, phase margin changes)?
(c) The student’s instinct is "redesign the compensator for the loaded case." A better approach is to design for the *worst case*. Which loading condition should the compensator be designed for, and why?

**Exercise 8.9 — Mechanism vs. Procedure**
A student designs a lag compensator and a lead compensator for the same plant. Both achieve PM = 45°. The student asks: "Which one is better?"

(a) Explain the *mechanism* difference: what does each compensator do to the Bode plot, and therefore to the *transient response*?
(b) Lead compensation increases bandwidth. When is this desirable? When is it harmful? (Hint: sensor noise.)
(c) Lag compensation achieves steady-state accuracy without increasing bandwidth. In what physical application (slow process, noisy sensor, etc.) would you prefer lag over lead? Give a specific example.

---

## What Comes Next

**Chapter 9: System Identification** — You now have a complete toolkit for designing controllers in both the time domain (Chapters 3–5) and the frequency domain (Chapters 6–8). But all of these tools require a transfer function $G(s)$ — and so far, we've obtained $G(s)$ only from first-principles modeling (Chapter 2). In practice, most real systems cannot be modeled purely from physics: parameters are uncertain, parasitic dynamics are unmodeled, and datasheet values don't match your specific hardware. Chapter 9 teaches you how to obtain $G(s)$ from measured data — closing the gap between textbook theory and real-world controller design.

**Chapter 10: State-Space Representation** — After identification, we shift from input-output descriptions (transfer functions) to internal descriptions (state-space). State-space methods are essential for MIMO systems and open the door to modern techniques — pole placement, optimal control, and observer design.

---

## References


1. Åström, K.J. & Murray, R.M. (2021). *Feedback Systems*
2. Skogestad, S. & Postlethwaite, I. (2005). *Multivariable Feedback Control*
