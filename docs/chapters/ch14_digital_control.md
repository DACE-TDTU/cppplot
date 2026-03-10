# Chapter 14: Digital Control Systems

---

## 🎯 Learning Objectives and Outcomes

### Chapter Objectives

This chapter bridges continuous-time theory with digital implementation, covering sampling, the Z-transform, discretization methods, and digital controller design essential for modern microcontroller-based control.

### Prerequisites
- Chapters 1-8: Classical control theory
- Chapter 3: Laplace Transform
- Basic signal processing concepts

---

## Learning Outcomes

After completing this chapter, students will be able to:

| Bloom's Level | Outcome |
|---------------|---------|
| **Remember** | Define sampling, Z-transform, and hold circuits |
| **Understand** | Explain discretization methods and their effects |
| **Apply** | Design digital controllers using various methods |
| **Analyze** | Analyze stability and performance of digital systems |
| **Evaluate** | Compare continuous and digital controller performance |
| **Create** | Implement digital controllers for practical applications |

---

## Why This Chapter Matters

> **The Real Problem:** Your 3D printer's firmware update changes the control loop rate from 1 kHz to 500 Hz, and print quality degrades catastrophically — visible ringing on every curved surface. The controller gains haven't changed. The motor hasn't changed. Only the *sampling rate* changed. Why does this break everything?
>
> Every controller you've designed in Chapters 1–12 assumed continuous time: signals exist at every instant, and the controller acts without delay. Real embedded systems sample at discrete intervals, compute in finite time, and output through digital-to-analog converters. This chapter bridges that gap — teaching you *how* and *why* sampling fundamentally changes the control problem.

---

## 14.1 Introduction to Digital Control

### 14.1.1 Why Digital Control?

**Advantages:**
- Flexibility: Easy to modify control laws
- Accuracy: No component drift
- Complexity: Can implement sophisticated algorithms
- Cost: Microcontrollers are inexpensive
- Communication: Easy networking/logging

**Challenges:**
- Sampling introduces delay
- Quantization errors
- Requires discrete-time analysis

### 14.1.2 Digital Control System Structure

```
    ┌─────┐   ┌─────┐   ┌─────────┐   ┌─────┐   ┌───────┐
r ──►│ A/D │──►│ D/C │──►│ ZOH+    │──►│Plant│──►│Sensor │──┐
    └─────┘   └─────┘   │Actuator │   └─────┘   └───────┘  │
       ▲                └─────────┘                        │
       │                                                   │
       └───────────────────────────────────────────────────┘
    
    A/D: Analog-to-Digital Converter (Sampler)
    D/C: Digital Controller
    ZOH: Zero-Order Hold
```

### 14.1.3 Controller Output: Desired vs. Actual Values

> **Critical Insight:** The digital controller computes a **desired** control value (a number). This must be converted to an **actual** physical signal through D/A conversion and power electronics.

**Complete Signal Path:**

```
┌─────────────────────────────────────────────────────────────────────────┐
│              DIGITAL CONTROL SIGNAL PATH                                │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  Digital Controller                Power Stage            Physical      │
│  (MCU/DSP)                                                World        │
│                                                                         │
│  ┌──────────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐     │
│  │ Control Law  │───►│   DAC    │───►│  Power   │───►│  Plant   │     │
│  │  u[k] = ...  │    │ or PWM   │    │Converter │    │ (Motor)  │     │
│  └──────────────┘    └──────────┘    └──────────┘    └──────────┘     │
│        │                  │               │               │            │
│        ▼                  ▼               ▼               ▼            │
│    u_desired[k]      u_analog(t)     u_power(t)      y(t)             │
│    (integer or     (0-3.3V or       (0-48V or       (speed,          │
│     float)          ±10V)            AC voltage)     position,        │
│                                                       temp)           │
│                                                                         │
│  EXAMPLE VALUES:                                                       │
│  u_desired = 2048   →   1.65V   →   PWM 50%   →   6V to motor        │
│  (12-bit DAC)          (mid-scale)   (12V supply)   (average)         │
└─────────────────────────────────────────────────────────────────────────┘
```

**Output Methods Comparison:**

| Method | Resolution | Speed | Application |
|--------|------------|-------|-------------|
| **DAC (Digital-to-Analog)** | 8-16 bit | Fast | Audio, analog servo |
| **PWM (Pulse Width Modulation)** | Timer-dependent | Very fast | Motor drives, LED |
| **PWM + Low-pass Filter** | Depends on filter | Slow | Create analog signal |
| **Sigma-Delta DAC** | 16-24 bit | Slow | Precision measurement |

**PWM as "Poor Man's DAC":**

```
   PWM Signal (10 kHz)              Average Value
   
   D = 25%:  ┌┐    ┌┐    ┌┐         ──────── 3V (if V_supply = 12V)
             ┘└────┘└────┘└────
   
   D = 50%:  ┌──┐  ┌──┐  ┌──┐       ──────── 6V
             ┘  └──┘  └──┘  └──
   
   D = 75%:  ┌────┐┌────┐┌────┐     ──────── 9V
             ┘    ┘┘    ┘┘    ┘
```

**Code Example: Digital Control Output**

```cpp
// Digital PID controller output to physical actuation
void controlLoop() {
    // Read sensor (ADC)
    int16_t adc_value = readADC(SENSOR_CHANNEL);
    double measurement = adc_value * (3.3 / 4096.0);  // 12-bit ADC, 3.3V ref
    
    // PID computation
    double error = setpoint - measurement;
    double u_desired = Kp*error + Ki*integral + Kd*derivative;
    
    // u_desired is just a NUMBER at this point!
    // We must convert it to physical actuation:
    
    // Option 1: DAC output (if using analog power stage)
    // DAC expects 0-4095 for 0-10V output
    int16_t dac_value = (int16_t)(u_desired * 409.5);  // Scale to DAC range
    dac_value = constrain(dac_value, 0, 4095);         // Saturate
    writeDAC(dac_value);
    
    // Option 2: PWM output (if using H-bridge)
    // PWM expects 0-255 for 0-100% duty cycle
    double V_supply = 24.0;
    double duty = u_desired / V_supply;                // Convert to duty cycle
    duty = constrain(duty, -1.0, 1.0);                // Saturate
    
    if (duty >= 0) {
        setDirection(FORWARD);
        setPWM((uint8_t)(duty * 255));
    } else {
        setDirection(REVERSE);
        setPWM((uint8_t)(-duty * 255));
    }
}
```

**Key Point:** The transfer function $G_c(z)$ of a digital controller relates **desired** input to **desired** output. The actual physical output depends on the power stage characteristics.

---

## 14.2 Sampling and Reconstruction

### 14.2.1 Ideal Sampling

**Sampled signal:**
$$x^*(t) = x(t) \cdot \delta_T(t) = \sum_{k=0}^{\infty} x(kT)\delta(t-kT)$$

where T is the **sampling period**.

### 14.2.2 Shannon's Sampling Theorem

> **Theorem:**
> To perfectly reconstruct a signal from samples, the sampling frequency must be at least twice the highest frequency component:
> $$f_s \geq 2f_{max}$$

**Nyquist frequency:** $f_N = f_s/2$

### 14.2.3 Aliasing

When sampling theorem is violated:
- High frequencies appear as low frequencies
- Cannot be corrected after sampling
- Solution: Anti-aliasing filter before A/D

### 14.2.4 Zero-Order Hold (ZOH)

Holds sampled value constant between samples:
$$u(t) = u(kT), \quad kT \leq t < (k+1)T$$

**Transfer function:**
$$G_{ZOH}(s) = \frac{1 - e^{-Ts}}{s}$$

### 14.2.5 PWM as Output Reconstruction

> **Important:** In motor control, PWM serves a similar purpose to ZOH but operates differently:

| Aspect | ZOH (DAC) | PWM |
|--------|-----------|-----|
| Output | Held constant value | Pulsed signal |
| Averaging | No averaging needed | Motor inductance averages |
| Frequency | = Sample rate | Typically >> Sample rate |
| Suitable for | Analog systems | Inductive loads (motors) |

**PWM Frequency Selection:**

$$f_{PWM} \geq 10 \times \text{max}(f_{control}, f_{audible})$$

Example: For 1 kHz control loop and audible range 20 kHz → use $f_{PWM}$ ≥ 20 kHz

---

## 14.3 The Z-Transform

### 14.3.1 Definition

For sequence $\{x(k)\}$:
$$X(z) = \mathcal{Z}\{x(k)\} = \sum_{k=0}^{\infty} x(k)z^{-k}$$

### 14.3.2 Important Z-Transforms

#### Basic Z-Transform Pairs

| # | Time Signal $x(k)$ or $x(kT)$ | Z-Transform $X(z)$ | ROC |
|---|-------------------------------|---------------------|-----|
| 1 | $\delta(k)$ (unit impulse) | $1$ | All $z$ |
| 2 | $u(k)$ (unit step) | $\dfrac{z}{z-1}$ | $|z|>1$ |
| 3 | $kT$ (ramp) | $\dfrac{Tz}{(z-1)^2}$ | $|z|>1$ |
| 4 | $(kT)^2$ (parabolic) | $\dfrac{T^2 z(z+1)}{(z-1)^3}$ | $|z|>1$ |
| 5 | $a^k$ | $\dfrac{z}{z-a}$ | $|z|>|a|$ |
| 6 | $e^{-akT}$ | $\dfrac{z}{z-e^{-aT}}$ | $|z|>e^{-aT}$ |
| 7 | $k a^{k}$ | $\dfrac{az}{(z-a)^2}$ | $|z|>|a|$ |
| 8 | $k e^{-akT}$ | $\dfrac{e^{-aT}z}{(z-e^{-aT})^2}$ | $|z|>e^{-aT}$ |
| 9 | $\sin(\omega k T)$ | $\dfrac{z\sin(\omega T)}{z^2 - 2z\cos(\omega T) + 1}$ | $|z|>1$ |
| 10 | $\cos(\omega k T)$ | $\dfrac{z(z - \cos(\omega T))}{z^2 - 2z\cos(\omega T) + 1}$ | $|z|>1$ |
| 11 | $e^{-akT}\sin(\omega kT)$ | $\dfrac{e^{-aT}z\sin(\omega T)}{z^2 - 2e^{-aT}z\cos(\omega T) + e^{-2aT}}$ | $|z|>e^{-aT}$ |
| 12 | $e^{-akT}\cos(\omega kT)$ | $\dfrac{z(z - e^{-aT}\cos(\omega T))}{z^2 - 2e^{-aT}z\cos(\omega T) + e^{-2aT}}$ | $|z|>e^{-aT}$ |
| 13 | $1 - e^{-akT}$ | $\dfrac{(1-e^{-aT})z}{(z-1)(z-e^{-aT})}$ | $|z|>1$ |

#### ZOH Equivalent Discretizations

These are the Z-transforms of $\mathcal{Z}\left\{\frac{1-e^{-sT}}{s} \cdot G(s)\right\}$, i.e., the ZOH-discretized plant transfer functions used most commonly in digital control:

| # | Laplace Transform $G(s)$ | ZOH Discrete Equivalent $G(z)$ |
|---|--------------------------|----------------------------------|
| 1 | $\dfrac{1}{s}$ (integrator) | $\dfrac{Tz^{-1}}{1 - z^{-1}} = \dfrac{T}{z-1}$ |
| 2 | $\dfrac{1}{s^2}$ (double integrator) | $\dfrac{T^2 z^{-1}(1 + z^{-1})}{2(1-z^{-1})^2} = \dfrac{T^2(z+1)}{2(z-1)^2}$ |
| 3 | $\dfrac{1}{s+a}$ (first order) | $\dfrac{(1 - e^{-aT})z^{-1}}{1 - e^{-aT}z^{-1}} = \dfrac{1-e^{-aT}}{z - e^{-aT}}$ |
| 4 | $\dfrac{a}{s(s+a)}$ | $\dfrac{(1-e^{-aT})z^{-1}(1 + \frac{1-e^{-aT}-aT e^{-aT}}{a(1-e^{-aT})}z^{-1})}{(1-z^{-1})(1-e^{-aT}z^{-1})}$ |
| 5 | $\dfrac{1}{(s+a)(s+b)}$ | $\dfrac{\frac{1}{b-a}\left[\frac{(1-e^{-aT})}{z-e^{-aT}} - \frac{(1-e^{-bT})}{z-e^{-bT}}\right]}{1}$ (partial fractions) |
| 6 | $\dfrac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}$ | Use partial fractions or state-space ZOH method |

> **Tip:** For higher-order or complex plants, use the state-space ZOH discretization method ($\mathbf{A}_d = e^{\mathbf{A}T}$) which handles any plant exactly.

#### Common Z-Transform Properties Quick Reference

| Property | Time Domain | Z-Domain |
|----------|-------------|----------|
| Linearity | $\alpha x_1(k) + \beta x_2(k)$ | $\alpha X_1(z) + \beta X_2(z)$ |
| Right shift (delay) | $x(k-m)$ | $z^{-m}X(z)$ |
| Left shift (advance) | $x(k+1)$ | $zX(z) - zx(0)$ |
| Convolution | $x_1(k) * x_2(k)$ | $X_1(z) \cdot X_2(z)$ |
| Multiplication by $k$ | $kx(k)$ | $-z\dfrac{dX(z)}{dz}$ |
| Multiplication by $a^k$ | $a^k x(k)$ | $X(z/a)$ |
| Initial value | $x(0)$ | $\lim_{z\to\infty} X(z)$ |
| Final value | $\lim_{k\to\infty} x(k)$ | $\lim_{z\to 1}(z-1)X(z)$ |

### 14.3.3 Properties

| Property | Time Domain | Z-Domain |
|----------|-------------|----------|
| Linearity | $ax(k) + by(k)$ | $aX(z) + bY(z)$ |
| Time shift | $x(k-n)$ | $z^{-n}X(z)$ |
| Final value | $\lim_{k\to\infty} x(k)$ | $\lim_{z\to 1}(z-1)X(z)$ |

### 14.3.4 Relationship to s-domain

$$z = e^{sT}$$

| s-plane | z-plane |
|---------|---------|
| Left half-plane | Inside unit circle |
| Imaginary axis | Unit circle |
| Right half-plane | Outside unit circle |

---

## 14.4 Discrete-Time Systems

### 14.4.1 Pulse Transfer Function

For system with ZOH and continuous plant G(s):
$$G(z) = \mathcal{Z}\{G_{ZOH}(s)G(s)\} = (1-z^{-1})\mathcal{Z}\left\{\frac{G(s)}{s}\right\}$$

### 14.4.2 Difference Equations

**General form:**
$$y(k) = -\sum_{i=1}^{n} a_i y(k-i) + \sum_{j=0}^{m} b_j u(k-j)$$

**Example:** First-order system
$$y(k) = ay(k-1) + bu(k-1)$$

### 14.4.3 Discrete State-Space

$$\mathbf{x}(k+1) = \mathbf{A}_d\mathbf{x}(k) + \mathbf{B}_d\mathbf{u}(k)$$
$$\mathbf{y}(k) = \mathbf{C}_d\mathbf{x}(k) + \mathbf{D}_d\mathbf{u}(k)$$

---

## 14.5 Discretization Methods

### 14.5.1 Exact Discretization (ZOH)

For continuous system with ZOH:
$$\mathbf{A}_d = e^{\mathbf{A}T}$$
$$\mathbf{B}_d = \left(\int_0^T e^{\mathbf{A}\tau}d\tau\right)\mathbf{B} = \mathbf{A}^{-1}(e^{\mathbf{A}T} - \mathbf{I})\mathbf{B}$$

> **Note:** The closed-form $\mathbf{A}^{-1}(e^{\mathbf{A}T} - \mathbf{I})$ requires $\mathbf{A}$ to be invertible. For systems with singular $\mathbf{A}$ (e.g., pure integrators where $\det(\mathbf{A}) = 0$), compute the integral directly via the matrix exponential series:
> $$\int_0^T e^{\mathbf{A}\tau}d\tau = \mathbf{I}T + \frac{\mathbf{A}T^2}{2!} + \frac{\mathbf{A}^2 T^3}{3!} + \cdots$$
> or use the block-matrix exponential: $\exp\begin{pmatrix}\mathbf{A} & \mathbf{B} \\ \mathbf{0} & \mathbf{0}\end{pmatrix}T = \begin{pmatrix}e^{\mathbf{A}T} & \int_0^T e^{\mathbf{A}\tau}d\tau\,\mathbf{B} \\ \mathbf{0} & \mathbf{I}\end{pmatrix}$.

### 14.5.2 Forward Euler (Forward Difference)

Approximation: $s \approx \frac{z-1}{T}$

**Properties:**
- Simple
- Can make stable systems unstable
- First-order accurate

### 14.5.3 Backward Euler (Backward Difference)

Approximation: $s \approx \frac{z-1}{Tz}$

**Properties:**
- Stable systems remain stable
- May add excessive damping
- First-order accurate

### 14.5.4 Tustin's Method (Bilinear Transform)

Approximation: $s = \frac{2}{T}\frac{z-1}{z+1}$

**Properties:**
- Stable systems remain stable
- Preserves frequency response shape
- Frequency warping occurs: $\omega_d = \frac{2}{T}\tan\frac{\omega T}{2}$

### 14.5.5 Comparison

| Method | Stability | Accuracy | Complexity |
|--------|-----------|----------|------------|
| ZOH | Exact | Exact | High |
| Forward Euler | May fail | Low | Low |
| Backward Euler | Preserved | Low | Low |
| Tustin | Preserved | Medium | Medium |

---

## 14.6 Stability in Z-Domain

### 14.6.1 Stability Criterion

> **Theorem:**
> A discrete-time system is stable if and only if all poles of G(z) lie **inside the unit circle**.

$$|z_i| < 1, \quad \forall i$$

### 14.6.2 Jury Stability Test

The Jury stability test is the z-domain analog of the Routh-Hurwitz test for continuous systems. Given the characteristic polynomial of a discrete-time system:

$$P(z) = a_n z^n + a_{n-1} z^{n-1} + \cdots + a_1 z + a_0$$

#### Necessary Conditions (Quick Checks)

Before constructing the full Jury array, three necessary conditions must be satisfied:

1. $P(1) > 0$
2. $(-1)^n P(-1) > 0$
3. $|a_0| < a_n$

If **any** of these conditions fail, the system is **unstable** and there is no need to proceed further.

#### Jury Array Construction

The Jury array is formed by alternating rows — each odd row is the coefficients in order, and each even row is the same coefficients in reverse. Each subsequent pair of rows is computed from the determinant operation on the previous pair:

| Row | $z^0$ | $z^1$ | $z^2$ | ... | $z^{n-k}$ |
|-----|--------|--------|--------|-----|------------|
| 1 | $a_0$ | $a_1$ | $a_2$ | ... | $a_n$ |
| 2 | $a_n$ | $a_{n-1}$ | $a_{n-2}$ | ... | $a_0$ |
| 3 | $b_0$ | $b_1$ | $b_2$ | ... | $b_{n-1}$ |
| 4 | $b_{n-1}$ | $b_{n-2}$ | $b_{n-3}$ | ... | $b_0$ |
| 5 | $c_0$ | $c_1$ | $c_2$ | ... | $c_{n-2}$ |
| ... | ... | ... | ... | ... | ... |

Where the elements of each new row are computed as:

$$b_k = \det\begin{pmatrix} a_0 & a_{n-k} \\ a_n & a_k \end{pmatrix} = a_0 a_k - a_n a_{n-k}$$

$$c_k = \det\begin{pmatrix} b_0 & b_{n-1-k} \\ b_{n-1} & b_k \end{pmatrix} = b_0 b_k - b_{n-1} b_{n-1-k}$$

The array is continued until a row of length 1 is reached (total of $2n - 3$ rows).

#### Stability Criterion

> **Jury Stability Criterion:** The system is stable if and only if **all** the first elements of the odd rows are positive:
> $$|a_0| < a_n, \quad |b_0| > |b_{n-1}|, \quad |c_0| > |c_{n-2}|, \quad \ldots$$

#### Worked Example

**Problem:** Determine the stability of the system with characteristic polynomial:
$$P(z) = z^3 - 1.5z^2 + 0.7z - 0.1$$

Here $n = 3$, $a_3 = 1$, $a_2 = -1.5$, $a_1 = 0.7$, $a_0 = -0.1$.

**Step 1: Check necessary conditions**

1. $P(1) = 1 - 1.5 + 0.7 - 0.1 = 0.1 > 0$ ✓
2. $(-1)^3 P(-1) = (-1)(-1 - 1.5 - 0.7 - 0.1) = (-1)(-3.3) = 3.3 > 0$ ✓
3. $|a_0| = 0.1 < 1 = a_3$ ✓

All necessary conditions pass.

**Step 2: Construct Jury array**

| Row | $z^0$ | $z^1$ | $z^2$ | $z^3$ |
|-----|--------|--------|--------|--------|
| 1 | $-0.1$ | $0.7$ | $-1.5$ | $1$ |
| 2 | $1$ | $-1.5$ | $0.7$ | $-0.1$ |

Compute $b_k$:
- $b_0 = a_0 \cdot a_0 - a_3 \cdot a_3 = (-0.1)(-0.1) - (1)(1) = 0.01 - 1 = -0.99$
- $b_1 = a_0 \cdot a_1 - a_3 \cdot a_2 = (-0.1)(0.7) - (1)(-1.5) = -0.07 + 1.5 = 1.43$
- $b_2 = a_0 \cdot a_2 - a_3 \cdot a_1 = (-0.1)(-1.5) - (1)(0.7) = 0.15 - 0.7 = -0.55$

| Row | $z^0$ | $z^1$ | $z^2$ |
|-----|--------|--------|--------|
| 3 | $-0.99$ | $1.43$ | $-0.55$ |
| 4 | $-0.55$ | $1.43$ | $-0.99$ |

Compute $c_k$:
- $c_0 = b_0 \cdot b_0 - b_2 \cdot b_2 = (-0.99)^2 - (-0.55)^2 = 0.9801 - 0.3025 = 0.6776$
- $c_1 = b_0 \cdot b_1 - b_2 \cdot b_1 = (-0.99)(1.43) - (-0.55)(1.43) = -1.4157 + 0.7865 = -0.6292$

| Row | $z^0$ | $z^1$ |
|-----|--------|--------|
| 5 | $0.6776$ | $-0.6292$ |

**Step 3: Check stability**

Odd-row first elements:
- Row 1: $|a_0| = 0.1 < 1 = a_3$ ✓
- Row 3: $|b_0| = 0.99 > 0.55 = |b_2|$ ✓
- Row 5: $c_0 = 0.6776 > 0$ ✓

**Conclusion:** All conditions are satisfied. The system is **stable** — all roots of $P(z)$ lie inside the unit circle.

#### cppplot Implementation

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;

// Verify stability by computing roots
TransferFunction Pz({1, -1.5, 0.7, -0.1}, {1});
auto poles = roots({1, -1.5, 0.7, -0.1});

// Check all poles inside unit circle
for (auto& p : poles) {
    std::cout << "Pole: " << p << ", |z| = " << std::abs(p) << std::endl;
}
// All |z| < 1 confirms Jury test result
```

### 14.6.3 Bilinear Transformation for Stability

Transform z-plane to w-plane:
$$w = \frac{z-1}{z+1}$$

Then apply Routh-Hurwitz in w-plane.

### 14.6.4 Discrete Root Locus

The root locus technique applies in the z-plane with rules identical to the s-plane, but with a different stability boundary.

#### Key Differences from s-plane Root Locus

| Feature | s-plane | z-plane |
|---------|---------|----------|
| **Stability boundary** | Imaginary axis ($j\omega$) | Unit circle $|z| = 1$ |
| **Stable region** | Left half-plane ($\text{Re}(s) < 0$) | Inside unit circle ($|z| < 1$) |
| **Unstable region** | Right half-plane | Outside unit circle |
| **Constant $\sigma$ lines** | Vertical lines | Circles centered at origin |
| **Constant $\omega_n$ lines** | Circles centered at origin | Radial lines from origin |
| **Constant $\zeta$ lines** | Radial lines from origin | Logarithmic spirals |

#### s-to-z Mapping Relationships

The fundamental mapping $z = e^{sT}$ relates the s-plane and z-plane:

$$z = e^{sT} = e^{(\sigma + j\omega)T} = e^{\sigma T} e^{j\omega T}$$

From this:
- **Magnitude:** $|z| = e^{\sigma T}$
  - Constant decay rate $\sigma$ maps to circles of radius $e^{\sigma T}$ in the z-plane
  - $\sigma = 0$ (imaginary axis) maps to $|z| = 1$ (unit circle)
  - $\sigma < 0$ (stable) maps to $|z| < 1$ (inside unit circle)

- **Angle:** $\angle z = \omega T$
  - Constant frequency $\omega$ maps to radial lines at angle $\omega T$
  - The entire left half-plane strip $-\pi/T < \omega < \pi/T$ maps onto the full unit circle

#### Design Regions in the z-plane

**Constant damping ratio $\zeta$:**
In the s-plane, lines of constant $\zeta$ are straight rays from the origin. Under the mapping $z = e^{sT}$, these become logarithmic spirals in the z-plane:

$$|z| = e^{-\zeta \omega_n T}, \quad \angle z = \omega_n T \sqrt{1 - \zeta^2}$$

**Constant settling time $t_s$:**
Settling time is governed by $\sigma = -\zeta\omega_n$, so constant $t_s \approx 4/|\sigma|$ maps to circles:
$$|z| = e^{\sigma T} = e^{-4T/t_s}$$

**Constant natural frequency $\omega_n$:**
Maps to lines at angle $\theta = \omega_d T = \omega_n \sqrt{1-\zeta^2} \cdot T$ from the positive real axis.

#### Applying Root Locus Rules

All classical root locus rules apply directly:
1. **Start at open-loop poles** of $G(z)H(z)$, **end at open-loop zeros** (or infinity)
2. **Number of branches** = number of open-loop poles
3. **Real-axis segments**: to the left of an odd number of real poles + zeros
4. **Asymptotes**: angles $(2q+1)\pi / (n-m)$, centroid $\sigma_a = (\sum p_i - \sum z_j)/(n-m)$
5. **Breakaway/break-in points**: solve $dK/dz = 0$
6. **Angle of departure/arrival**: from angle condition

> **Important:** When sketching the root locus in the z-plane, always draw the **unit circle** as the stability boundary, not the imaginary axis.

#### cppplot Implementation

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;

// Discrete transfer function: G(z) = (z + 0.5) / (z^2 - 1.5z + 0.7)
TransferFunction Gz({1, 0.5}, {1, -1.5, 0.7});  // Discrete TF

// Plot root locus in z-plane (unit circle shown automatically)
rlocus(Gz);
title("Discrete Root Locus in z-plane");

// Add unit circle for stability reference
auto theta = linspace(0, 2*M_PI, 200);
std::vector<double> uc_re, uc_im;
for (auto t : theta) {
    uc_re.push_back(std::cos(t));
    uc_im.push_back(std::sin(t));
}
plot(uc_re, uc_im, "--", opts({{"color", "gray"}, {"label", "Unit Circle"}}));
grid(true);
savefig("ch14_discrete_rlocus.svg");
```

#### Example: Root Locus for Digital Controller Design

Consider the discrete plant (ZOH discretization of $G(s) = 1/(s(s+1))$ with $T = 0.1$s):

$$G(z) = \frac{0.004837(z + 0.9672)}{(z-1)(z-0.9048)}$$

Using the root locus, we can find the gain $K$ that places the closed-loop poles at a desired location inside the unit circle, satisfying both stability and performance requirements.

```cpp
// Plant G(z) from ZOH discretization
TransferFunction Gp({0.004837, 0.004837*0.9672}, {1, -(1+0.9048), 1*0.9048});
rlocus(Gp);

// Find gain for desired damping ratio
auto [K, poles] = rlocfind(Gp);  // Interactive gain selection
std::cout << "Selected gain K = " << K << std::endl;
for (auto& p : poles) {
    std::cout << "CL pole: " << p << ", |z| = " << std::abs(p) << std::endl;
}
```

---

## 14.7 Digital PID Controller

### 14.7.1 Continuous PID

$$u(t) = K_p e(t) + K_i \int_0^t e(\tau)d\tau + K_d \frac{de(t)}{dt}$$

### 14.7.2 Discrete Approximation

**Position form:**
$$u(k) = K_p e(k) + K_i T \sum_{j=0}^{k} e(j) + K_d \frac{e(k) - e(k-1)}{T}$$

**Velocity form (incremental):**
$$\Delta u(k) = u(k) - u(k-1) = K_p[e(k)-e(k-1)] + K_i T e(k) + K_d \frac{e(k)-2e(k-1)+e(k-2)}{T}$$

### 14.7.3 Derivative Filtering

To reduce noise sensitivity:
$$D(z) = K_d \frac{N(z-1)}{z - (1-NT)}$$

where N = 8-20 typically.

> **Stability condition:** The filter pole is at $z = 1-NT$. For stability, $|1-NT| < 1$, requiring $0 < NT < 2$, i.e., $N < 2/T$. For example, with $T = 0.1$ s, $N$ must be less than 20; setting $N = 20$ places the pole at $z = -1$ (marginal instability).

### 14.7.4 Anti-Windup

When the actuator saturates, the integrator continues accumulating error ("windup"), causing large overshoot when the error changes sign.

**Method 1: Conditional integration (clamping)**
Stop updating the integrator when the output is saturated AND the integrator and error have the same sign:
$$\text{If } |u| > u_{max} \text{ and } \text{sign}(e) = \text{sign}(I): \quad I(k+1) = I(k)$$

**Method 2: Back-calculation**
Add a correction term proportional to the saturation error:
$$I(k+1) = I(k) + K_i T_s \, e(k) + \frac{T_s}{T_t}\left(u_{sat}(k) - u(k)\right)$$
where $T_t$ is the tracking time constant (typically $T_t = \sqrt{T_i T_d}$ for PID), $u(k)$ is the unsaturated controller output, and $u_{sat}(k) = \text{clip}(u(k), -u_{max}, u_{max})$.

### 14.7.5 Deadbeat Control Design

A **deadbeat controller** is a uniquely digital design strategy that achieves zero steady-state error in the **minimum number of sampling periods**. This is impossible in continuous-time control and is one of the key advantages of digital systems.

#### Principle

The deadbeat design places **all closed-loop poles at $z = 0$**. Since poles at $z = 0$ correspond to finite impulse response (FIR) behavior, the closed-loop system reaches steady state in at most $n$ sampling periods (where $n$ is the system order).

#### General Formulation

For a plant $G(z) = B(z)/A(z)$ preceded by a ZOH, the deadbeat controller is designed so that the closed-loop transfer function has the form:

$$H(z) = \frac{z^{-d} B^{+}(z)}{\tilde{B}^{+}(z)}$$

where:
- $d$ is the plant relative degree (number of excess poles over zeros)
- $B^{+}(z)$ contains the cancelable (minimum phase) zeros of $G(z)$
- Non-minimum phase zeros (outside the unit circle) must **not** be canceled

The controller is then:
$$C(z) = \frac{H(z)}{G(z)(1 - H(z))}$$

#### First-Order Plant Derivation

For a first-order plant with ZOH:
$$G(z) = \frac{b}{z - a}$$

where $b = 1 - e^{-T/\tau}$ and $a = e^{-T/\tau}$ for a continuous plant $G(s) = 1/(\tau s + 1)$.

We want the closed-loop transfer function to be a one-sample delay (minimum achievable for a strictly proper plant):
$$H(z) = z^{-1}$$

The required controller is:
$$C(z) = \frac{H(z)}{G(z)(1 - H(z))} = \frac{z^{-1}}{\frac{b}{z-a} \cdot (1 - z^{-1})}$$

Simplifying:
$$C(z) = \frac{z^{-1}(z - a)}{b \cdot \frac{z-1}{z}} = \frac{z^{-1} \cdot z \cdot (z-a)}{b(z-1)} = \frac{z - a}{b(z - 1)}$$

Or equivalently in negative powers of z:
$$C(z) = \frac{1 - az^{-1}}{b(1 - z^{-1})}$$

#### Worked Numerical Example

**Problem:** Design a deadbeat controller for the plant $G(s) = \frac{1}{s+1}$ with sampling period $T = 0.5$ s.

**Step 1: ZOH Discretization**

For $G(s) = \frac{1}{s+1}$, we have $\tau = 1$ s, so:
- $a = e^{-T/\tau} = e^{-0.5} = 0.6065$
- $b = 1 - e^{-T/\tau} = 1 - 0.6065 = 0.3935$

$$G(z) = \frac{0.3935}{z - 0.6065}$$

**Step 2: Deadbeat Controller**

$$C(z) = \frac{z - a}{b(z - 1)} = \frac{z - 0.6065}{0.3935(z - 1)}$$

**Step 3: Verification**

Open-loop transfer function:
$$G(z)C(z) = \frac{0.3935}{z - 0.6065} \cdot \frac{z - 0.6065}{0.3935(z - 1)} = \frac{1}{z - 1}$$

Closed-loop transfer function:
$$H(z) = \frac{G(z)C(z)}{1 + G(z)C(z)} = \frac{\frac{1}{z-1}}{1 + \frac{1}{z-1}} = \frac{\frac{1}{z-1}}{\frac{z}{z-1}} = \frac{1}{z} = z^{-1} \checkmark$$

The closed-loop pole is at $z = 0$ (deadbeat), and the step response reaches steady state in exactly **one** sampling period.

**Step response verification:**
- $k = 0$: $y(0) = 0$ (due to one-sample delay)
- $k = 1$: $y(1) = 1$ (reaches setpoint)
- $k \geq 1$: $y(k) = 1$ (stays at setpoint)

#### cppplot Implementation

```cpp
#include <cppplot/cppplot.hpp>
#include <cmath>
using namespace cppplot;

int main() {
    double T = 0.5;  // Sampling period
    double a = std::exp(-T);   // 0.6065
    double b = 1 - std::exp(-T); // 0.3935
    
    // Plant: G(z) = b / (z - a)
    TransferFunction Gz({b}, {1, -a});
    
    // Deadbeat controller: C(z) = (z - a) / (b*(z - 1))
    TransferFunction Cz({1, -a}, {b, -b});
    
    // Closed-loop: H(z) = G*C / (1 + G*C)
    auto L = Gz * Cz;              // Loop transfer function
    auto H = feedback(L, TransferFunction({1}, {1}));  // Closed-loop
    
    // Verify closed-loop poles
    auto cl_poles = pole(H);
    std::cout << "Closed-loop poles:" << std::endl;
    for (auto& p : cl_poles) {
        std::cout << "  z = " << p << " (|z| = " << std::abs(p) << ")" << std::endl;
    }
    
    // Step response
    int N = 10;  // 10 samples
    auto [t, y] = step_data(H);
    
    figure(700, 400);
    stem(t, y, opts({{"color", "blue"}, {"label", "Deadbeat Response"}}));
    axhline(1.0, opts({{"color", "red"}, {"linestyle", "--"}, {"label", "Setpoint"}}));
    xlabel("Sample k");
    ylabel("Output y[k]");
    title("Deadbeat Controller Step Response (T = 0.5s)");
    legend(true);
    grid(true);
    savefig("ch14_deadbeat_step.svg");
    
    return 0;
}
```

#### Practical Considerations

| Advantage | Disadvantage |
|-----------|-------------|
| Fastest possible response (finite settling) | Large control effort at first sample |
| Zero steady-state error | Sensitive to model uncertainty |
| Simple design procedure | May cancel plant zeros (fragile) |
| Unique to digital control | Requires accurate plant model |

> **Warning — Inter-sample behavior:** The deadbeat response is optimal only at the sampling instants. Between samples, the output may exhibit significant **ripple** or **overshoot**. Always check the continuous-time output, not just the sampled response.

> **Warning — Non-minimum phase plants:** If $G(z)$ has zeros outside the unit circle, canceling them leads to internal instability. Modified deadbeat designs must be used that do not cancel non-minimum phase zeros.

---

## 14.8 Sample Rate Selection

### 14.8.1 Guidelines

| Criterion | Sampling Period |
|-----------|-----------------|
| Shannon | $T < \frac{\pi}{\omega_{max}}$ |
| Practical | $T \leq \frac{1}{10\omega_{BW}}$ |
| Very good | $T \leq \frac{1}{20\omega_{BW}}$ |

where $\omega_{BW}$ is closed-loop bandwidth.

### 14.8.2 Trade-offs

| Fast Sampling | Slow Sampling |
|---------------|---------------|
| Better performance | Lower computational load |
| More noise sensitive | More aliasing risk |
| Higher cost | Larger phase lag |

---

## 14.9 Real-World Application: Automotive Engine Control Unit (ECU)

> **Practical Integration:** Modern automotive ECU demonstrates all aspects of digital control: multiple sensors, real-time constraints, communication protocols, and safety-critical implementation.

### 14.9.1 System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    AUTOMOTIVE ECU SYSTEM                                │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   SENSORS                    ECU                       ACTUATORS        │
│   ───────                 ──────────                   ─────────        │
│                     ┌─────────────────────┐                            │
│  ┌──────────┐       │  ┌───────────────┐  │       ┌──────────┐         │
│  │ Crank    │──────►│  │   Signal      │  │       │ Fuel     │         │
│  │ Position │  ────►│  │   Conditioning│  │──────►│ Injectors│         │
│  │ (Hall)   │       │  │   + ADC       │  │       └──────────┘         │
│  └──────────┘       │  └───────┬───────┘  │                            │
│                     │          │          │       ┌──────────┐         │
│  ┌──────────┐       │  ┌───────▼───────┐  │       │ Ignition │         │
│  │ MAF/MAP  │──────►│  │  32-bit MCU   │  │──────►│ Coils    │         │
│  │ Sensors  │       │  │  (Tricore/    │  │       └──────────┘         │
│  └──────────┘       │  │   ARM Cortex) │  │                            │
│                     │  │               │  │       ┌──────────┐         │
│  ┌──────────┐       │  │  ┌─────────┐  │  │       │ Throttle │         │
│  │ O2/Lambda│──────►│  │  │ Control │  │  │──────►│ Body     │         │
│  │ Sensors  │       │  │  │  Tasks  │  │  │       │ (ETC)    │         │
│  └──────────┘       │  │  └─────────┘  │  │       └──────────┘         │
│                     │  │               │  │                            │
│  ┌──────────┐       │  │  ┌─────────┐  │  │       ┌──────────┐         │
│  │ Temp/    │──────►│  │  │  Diag   │  │  │──────►│ EGR      │         │
│  │ Pressure │       │  │  │  + CAN  │  │  │       │ Valve    │         │
│  └──────────┘       │  │  └─────────┘  │  │       └──────────┘         │
│                     │  └───────────────┘  │                            │
│                     └─────────┬───────────┘                            │
│                               │                                        │
│                               ▼                                        │
│                     ┌─────────────────────┐                            │
│                     │   CAN Bus Network   │                            │
│                     │ (500 kbps / 1 Mbps) │                            │
│                     └─────────────────────┘                            │
│                               │                                        │
│              ┌────────────────┼────────────────┐                       │
│              ▼                ▼                ▼                       │
│        ┌──────────┐    ┌──────────┐    ┌──────────┐                   │
│        │   TCU    │    │   ABS    │    │Instrument│                   │
│        │(Transmis)│    │ /ESP ECU │    │ Cluster  │                   │
│        └──────────┘    └──────────┘    └──────────┘                   │
└─────────────────────────────────────────────────────────────────────────┘
```

### 14.9.2 Digital Control Loops in ECU

| Control Loop | Sample Rate | Plant | Algorithm |
|--------------|-------------|-------|-----------|
| **Idle Speed** | 10 ms | Engine + load | PID with feedforward |
| **Air-Fuel Ratio** | Per-cylinder | Combustion | Stoichiometric feedback |
| **Knock Control** | Per-cycle | Combustion | Event-based retard |
| **EGR Position** | 20 ms | EGR valve | Position PID |
| **Boost Pressure** | 10 ms | Turbocharger | Model-predictive |
| **Throttle Position** | 5 ms | DC motor + spring | Cascaded PID |

### 14.9.3 Example: Electronic Throttle Control (ETC)

**Plant Model (DC Motor + Return Spring):**

$$G(s) = \frac{\theta(s)}{V(s)} = \frac{K_m}{(Ls + R)(Js + b) + K_m K_e + K_{spring} \cdot (Ls+R)/s}$$

**Simplified (dominant dynamics):**
$$G(s) \approx \frac{K}{(s+a)(s+b)}$$

With typical values: $K = 50$, $a = 10$, $b = 100$ rad/s

**Discretization (ZOH, T = 5 ms):**

$$G(z) = \mathcal{Z}\left\{\frac{1-e^{-sT}}{s} \cdot G(s)\right\}$$

**Digital PID Controller:**
$$u[k] = u[k-1] + K_p(e[k] - e[k-1]) + K_i T e[k] + \frac{K_d}{T}(e[k] - 2e[k-1] + e[k-2])$$

### 14.9.4 Real-Time Constraints

```
┌─────────────────────────────────────────────────────────────────────────┐
│             ECU TASK SCHEDULING (Typical at 6000 RPM)                   │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  Time (ms)  0    5    10   15   20   25   30   35   40                 │
│             │    │    │    │    │    │    │    │    │                  │
│  Crank Sync ┼────┼────┼────┼────┼────┼────┼────┼────┼  (Event-based)  │
│             █         █         █         █                            │
│                                                                         │
│  Fast Loop  ┼────┼────┼────┼────┼────┼────┼────┼────┼  (5 ms)         │
│             █    █    █    █    █    █    █    █    █                  │
│                                                                         │
│  Medium     ┼────┼────┼────┼────┼────┼────┼────┼────┼  (10 ms)        │
│             █         █         █         █                            │
│                                                                         │
│  Slow Loop  ┼────┼────┼────┼────┼────┼────┼────┼────┼  (100 ms)       │
│             █                                                          │
│                                                                         │
│  Background │░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░│  (Idle time)      │
│                                                                         │
│  Legend: █ = Task execution   ░ = Available for diagnostics           │
└─────────────────────────────────────────────────────────────────────────┘
```

### 14.9.5 Multi-Domain Considerations

| Domain | ECU Aspect | Digital Control Impact |
|--------|------------|------------------------|
| **Mechanical** | Throttle plate inertia, friction | Plant dynamics in discrete model |
| **Electrical** | H-bridge driver, PWM | Quantized output (8-bit to 12-bit) |
| **Thermal** | ECU temperature -40°C to +125°C | Parameter scheduling |
| **Communication** | CAN latency 1-10 ms | Additional delay in feedback |
| **Software** | AUTOSAR stack overhead | Computation time budget |
| **Safety** | ASIL-C/D requirements | Redundant sensors, watchdog |

### 14.9.6 Implementation Code Pattern

```cpp
// Automotive-style digital PID (velocity form)
class ThrottlePID {
private:
    float Kp, Ki, Kd;
    float T;  // Sample time (5 ms typical)
    float e_prev, e_prev2;
    float u;
    float u_min, u_max;  // Actuator limits
    
public:
    float compute(float setpoint, float measurement) {
        float e = setpoint - measurement;
        
        // Velocity form PID
        float du = Kp * (e - e_prev) 
                 + Ki * T * e 
                 + Kd / T * (e - 2*e_prev + e_prev2);
        
        u += du;
        
        // Anti-windup with clamping
        if (u > u_max) u = u_max;
        if (u < u_min) u = u_min;
        
        // Update history
        e_prev2 = e_prev;
        e_prev = e;
        
        return u;
    }
};
```

---

## 14.10 Example: Digital Tank Level Control

### 14.10.1 System Description

**Plant:** Tank with inflow control
- Continuous model: $G(s) = \frac{K}{s(Ts+1)}$

### 14.10.2 Design Process

1. Choose sampling period (10× bandwidth)
2. Discretize plant with ZOH
3. Design digital PID controller
4. Implement anti-windup

See **ch14_digital_control.cpp** for complete implementation.

---

## 14.11 Electrical and Telecommunications Digital Control Examples

### 14.11.1 Digital Power Factor Correction (PFC)

**Problem:** Implement digital control for a boost PFC converter to achieve unity power factor and regulated DC bus voltage.

**System Architecture:**

```
┌─────────────────────────────────────────────────────────────────────────┐
│              DIGITAL PFC CONTROL SYSTEM                                 │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   AC Mains         Rectifier      Boost PFC        DC Bus              │
│                                                                         │
│   ∿ 220V ──►┌──┐──►┌────────┐──►┌────────┐──► 400V DC                │
│     50Hz    │DB│   │Inductor │   │  Cap   │      to Load              │
│             └──┘   │   L     │   │   C    │                            │
│                    └────┬───┘   └────────┘                             │
│                         │                                               │
│                     ┌───▼───┐                                          │
│                     │ MOSFET│  ← PWM from DSP                          │
│                     └───────┘                                          │
│                                                                         │
│   DSP/MCU:                                                             │
│   ┌─────────────────────────────────────────────────┐                  │
│   │  ┌──────────┐    ┌──────────┐    ┌──────────┐  │                  │
│   │  │ Voltage  │───►│ Current  │───►│   PWM    │  │                  │
│   │  │  Loop    │    │  Loop    │    │Generator │  │                  │
│   │  │ (slow)   │    │ (fast)   │    │          │  │                  │
│   │  └──────────┘    └──────────┘    └──────────┘  │                  │
│   │       ▲              ▲                         │                   │
│   │   V_bus ADC      I_L ADC                       │                  │
│   └─────────────────────────────────────────────────┘                  │
│                                                                         │
│   Sample Rates: Voltage loop = 1 kHz, Current loop = 50 kHz           │
└─────────────────────────────────────────────────────────────────────────┘
```

**Digital Control Implementation:**

```cpp
#include <cppplot/cppplot.hpp>
#include <cmath>

// Digital PFC Controller
class DigitalPFC {
private:
    // Voltage loop (PI, Ts_v = 1 ms)
    double Kp_v = 0.5;
    double Ki_v = 50.0;
    double Ts_v = 1e-3;
    double v_integral = 0;
    
    // Current loop (P + Resonant, Ts_i = 20 us)
    double Kp_i = 0.1;
    double Kr_i = 10.0;  // Resonant gain at 100 Hz (2*line freq)
    double Ts_i = 20e-6;
    double w_res = 2 * M_PI * 100;  // 100 Hz for 50 Hz mains
    
    // Resonant controller states (discretized)
    double x1_res = 0, x2_res = 0;
    
    // Limits
    double i_ref_max = 20.0;  // Peak current limit
    double duty_max = 0.95;
    
public:
    // Voltage loop - generates current reference
    double voltageLoop(double v_bus, double v_ref, double v_ac_rms) {
        double error = v_ref - v_bus;
        
        // PI controller
        v_integral += Ki_v * Ts_v * error;
        
        // Anti-windup
        if (v_integral > i_ref_max) v_integral = i_ref_max;
        if (v_integral < 0) v_integral = 0;
        
        double i_amplitude = Kp_v * error + v_integral;
        
        // Limit current amplitude
        if (i_amplitude > i_ref_max) i_amplitude = i_ref_max;
        if (i_amplitude < 0) i_amplitude = 0;
        
        return i_amplitude;
    }
    
    // Current loop - generates duty cycle
    double currentLoop(double i_L, double i_ref, double v_ac, double v_bus) {
        double error = i_ref - i_L;
        
        // P controller + Resonant compensator for harmonic rejection
        // Resonant: H(s) = Kr * s / (s^2 + w0^2)
        // Discretized using Tustin with pre-warping
        
        double w_d = 2/Ts_i * std::tan(w_res * Ts_i / 2);
        double a = 4 + Ts_i*Ts_i*w_d*w_d;
        double b = 2*Ts_i*w_d*w_d;
        
        // Resonant filter state update
        double x1_new = (4*x1_res - b*x2_res + 2*Kr_i*Ts_i*error) / a;
        double x2_new = x2_res + Ts_i*(x1_res + x1_new)/2;
        
        double u_res = x1_new;
        x1_res = x1_new;
        x2_res = x2_new;
        
        // Total control output
        double u = Kp_i * error + u_res;
        
        // Feedforward for faster response
        double d_ff = 1.0 - std::abs(v_ac) / v_bus;
        double duty = d_ff + u;
        
        // Saturate
        if (duty > duty_max) duty = duty_max;
        if (duty < 0) duty = 0;
        
        return duty;
    }
};
```

**Key Digital Control Considerations:**

| Aspect | Voltage Loop | Current Loop |
|--------|-------------|--------------|
| Sample rate | 1 kHz | 50-100 kHz |
| Bandwidth | ~10 Hz | ~5 kHz |
| Controller | PI | P + Resonant |
| ADC bits | 12 bit | 12 bit |
| Computation | ~10 μs | ~1 μs |

### 14.11.2 Digital Phase-Locked Loop (DPLL)

**Problem:** Implement an all-digital PLL for clock recovery in a communication receiver.

**DPLL Architecture:**

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    ALL-DIGITAL PLL (ADPLL)                              │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   Input    ┌─────────┐    ┌───────────┐    ┌─────────┐    Output      │
│   Clock ──►│  TDC    │───►│Digital    │───►│  DCO    │───► Clock      │
│            │(Phase   │    │Loop Filter│    │(Digitally│               │
│            │Detector)│    │           │    │Controlled│               │
│            └─────────┘    └───────────┘    │Oscillator│               │
│                 ▲                          └────┬────┘                │
│                 │                               │                      │
│                 └───────────────────────────────┘                      │
│                        Feedback                                        │
│                                                                         │
│   All signals are DIGITAL - no analog VCO!                            │
│                                                                         │
│   TDC: Time-to-Digital Converter (phase quantization)                 │
│   DCO: Uses fractional-N divider or delta-sigma modulator             │
└─────────────────────────────────────────────────────────────────────────┘
```

**Mathematical Model:**

Phase detector output (linearized):
$$e[n] = \phi_{ref}[n] - \phi_{out}[n]$$

Digital loop filter (Type II):
$$D[n] = D[n-1] + K_1 e[n] + (K_2 - K_1) e[n-1]$$

Where:
- $K_1 = \frac{2\zeta\omega_n T_s + (\omega_n T_s)^2}{1 + 2\zeta\omega_n T_s + (\omega_n T_s)^2}$
- $K_2 = \frac{(\omega_n T_s)^2}{1 + 2\zeta\omega_n T_s + (\omega_n T_s)^2}$

**Implementation:**

```cpp
#include <cppplot/cppplot.hpp>
#include <cmath>

using namespace cppplot;

class DigitalPLL {
private:
    // Loop filter coefficients
    double K1, K2;
    
    // State variables
    double phase_accum;      // DCO phase accumulator
    double freq_word;        // DCO frequency control word
    double e_prev;           // Previous phase error
    
    // Parameters
    double fs;               // Sample frequency
    double f_center;         // Center frequency
    double bits_phase;       // Phase accumulator bits
    
public:
    DigitalPLL(double sample_freq, double center_freq, 
               double bandwidth, double damping) {
        fs = sample_freq;
        f_center = center_freq;
        bits_phase = 32;
        
        // Calculate loop filter coefficients
        double Ts = 1.0 / fs;
        double wn = 2 * M_PI * bandwidth / 4;  // Natural frequency
        double zeta = damping;
        
        double denom = 1 + 2*zeta*wn*Ts + std::pow(wn*Ts, 2);
        K1 = (2*zeta*wn*Ts + std::pow(wn*Ts, 2)) / denom;
        K2 = std::pow(wn*Ts, 2) / denom;
        
        // Initialize
        phase_accum = 0;
        freq_word = f_center / fs;  // Normalized frequency
        e_prev = 0;
    }
    
    // Process one sample
    double process(double input_phase) {
        // Phase detector
        double e = input_phase - phase_accum;
        
        // Wrap to [-0.5, 0.5]
        while (e > 0.5) e -= 1.0;
        while (e < -0.5) e += 1.0;
        
        // Digital loop filter (Type II, proportional-integral)
        freq_word += K1 * e + (K2 - K1) * e_prev;
        
        // Update DCO phase
        phase_accum += freq_word;
        
        // Wrap phase accumulator
        while (phase_accum >= 1.0) phase_accum -= 1.0;
        while (phase_accum < 0) phase_accum += 1.0;
        
        e_prev = e;
        
        return phase_accum;
    }
    
    double getFrequency() const {
        return freq_word * fs;
    }
};

int main() {
    // Simulate DPLL locking to input with frequency offset
    double fs = 100e6;        // 100 MHz sample rate
    double f_input = 10.001e6; // Input frequency (10.001 MHz)
    double f_center = 10e6;    // Initial DCO frequency
    
    DigitalPLL pll(fs, f_center, 50e3, 0.707);  // 50 kHz BW, zeta=0.707
    
    std::vector<double> time_vec, freq_est, phase_err;
    
    int N = 10000;
    for (int n = 0; n < N; ++n) {
        double t = n / fs;
        double input_phase = std::fmod(f_input * t, 1.0);
        
        double output_phase = pll.process(input_phase);
        
        time_vec.push_back(t * 1e6);  // Convert to microseconds
        freq_est.push_back(pll.getFrequency() / 1e6);  // MHz
        phase_err.push_back((input_phase - output_phase) * 360);  // degrees
    }
    
    figure(800, 600);
    subplot(2, 1, 1);
    plot(time_vec, freq_est, "-", opts({{"color", "blue"}, {"label", "Estimated Frequency"}}));
    axhline(f_input/1e6, opts({{"color", "red"}, {"linestyle", "--"}}));
    ylabel("Frequency (MHz)");
    title("Digital PLL Frequency Acquisition");
    grid(true);
    
    subplot(2, 1, 2);
    plot(time_vec, phase_err, "-", opts({{"color", "green"}, {"label", "Phase Error"}}));
    xlabel("Time (us)");
    ylabel("Phase Error (deg)");
    grid(true);
    
    savefig("ch14_dpll_acquisition.svg");
    return 0;
}
```

### 14.11.3 Digital Automatic Gain Control (AGC)

**Problem:** Implement digital AGC for a software-defined radio receiver.

**System Architecture:**

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    DIGITAL AGC SYSTEM                                   │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   RF In    ┌─────┐   ┌─────┐   ┌─────┐   ┌─────┐                      │
│   ──────►  │ LNA │──►│Mixer│──►│ ADC │──►│ DDC │──► I/Q samples       │
│            └──┬──┘   └─────┘   └─────┘   └──┬──┘                       │
│               │                             │                          │
│               │    Analog gain              │   Digital gain           │
│               │    (coarse)                 │   (fine)                 │
│               ▼                             ▼                          │
│            ┌─────────────────────────────────────┐                     │
│            │        DIGITAL AGC PROCESSOR        │                     │
│            │                                     │                     │
│            │  ┌─────────┐    ┌─────────────┐   │                      │
│            │  │ Power   │───►│ Loop Filter │   │                      │
│            │  │Estimator│    │ (PI or log) │   │                      │
│            │  └─────────┘    └──────┬──────┘   │                      │
│            │                        │          │                       │
│            │              ┌─────────▼────────┐ │                      │
│            │              │  Gain Lookup     │ │                      │
│            │              │  Table (dB→lin)  │ │                      │
│            │              └──────────────────┘ │                      │
│            └─────────────────────────────────────┘                     │
│                                                                         │
│   Attack time: Fast (detect strong signal quickly)                    │
│   Release time: Slow (avoid pumping on fading)                        │
└─────────────────────────────────────────────────────────────────────────┘
```

**Digital AGC Implementation:**

```cpp
#include <cppplot/cppplot.hpp>
#include <cmath>
#include <algorithm>

using namespace cppplot;

class DigitalAGC {
private:
    // AGC parameters
    double target_level_dB;
    double alpha_attack;     // Fast attack coefficient
    double alpha_release;    // Slow release coefficient
    double gain_dB;          // Current gain in dB
    double gain_min_dB;
    double gain_max_dB;
    
    // Power estimator state
    double power_avg;
    
public:
    DigitalAGC(double target_dB, double attack_time_ms, 
               double release_time_ms, double fs) {
        target_level_dB = target_dB;
        
        // Convert time constants to filter coefficients
        // alpha = 1 - exp(-Ts/tau)
        double Ts = 1.0 / fs;
        alpha_attack = 1.0 - std::exp(-Ts / (attack_time_ms * 1e-3));
        alpha_release = 1.0 - std::exp(-Ts / (release_time_ms * 1e-3));
        
        gain_dB = 0;
        gain_min_dB = -40;
        gain_max_dB = 60;
        power_avg = 1e-10;  // Small initial value
    }
    
    // Process I/Q sample pair
    std::pair<double, double> process(double I, double Q) {
        // Instantaneous power
        double power_inst = I*I + Q*Q;
        
        // Adaptive smoothing (fast attack, slow release)
        double alpha = (power_inst > power_avg) ? alpha_attack : alpha_release;
        power_avg = alpha * power_inst + (1 - alpha) * power_avg;
        
        // Power in dB
        double power_dB = 10 * std::log10(power_avg + 1e-20);
        
        // Error signal
        double error_dB = target_level_dB - power_dB;
        
        // Update gain (with limits)
        gain_dB += 0.1 * error_dB;  // Slow integration
        gain_dB = std::clamp(gain_dB, gain_min_dB, gain_max_dB);
        
        // Apply gain
        double gain_linear = std::pow(10, gain_dB / 20);
        
        return {I * gain_linear, Q * gain_linear};
    }
    
    double getGain_dB() const { return gain_dB; }
};

int main() {
    // Simulate AGC with varying input signal strength
    double fs = 1e6;  // 1 MHz sample rate
    DigitalAGC agc(-10, 0.1, 10, fs);  // Target -10 dB, 0.1ms attack, 10ms release
    
    std::vector<double> time_ms, input_power_dB, output_power_dB, gain_dB_vec;
    
    int N = 50000;
    double power_out_avg = 1e-10;
    
    for (int n = 0; n < N; ++n) {
        double t = n / fs;
        
        // Input signal with varying amplitude
        double amplitude = 1.0;
        if (n > 10000 && n < 20000) amplitude = 10.0;   // Strong signal
        if (n > 30000 && n < 40000) amplitude = 0.1;    // Weak signal
        
        // Generate I/Q (simple sine wave)
        double I = amplitude * std::cos(2 * M_PI * 100e3 * t);
        double Q = amplitude * std::sin(2 * M_PI * 100e3 * t);
        
        // Process through AGC
        auto [I_out, Q_out] = agc.process(I, Q);
        
        // Track output power
        double p_out = I_out*I_out + Q_out*Q_out;
        power_out_avg = 0.001 * p_out + 0.999 * power_out_avg;
        
        // Log every 100 samples
        if (n % 100 == 0) {
            time_ms.push_back(t * 1000);
            input_power_dB.push_back(10 * std::log10(amplitude * amplitude));
            output_power_dB.push_back(10 * std::log10(power_out_avg));
            gain_dB_vec.push_back(agc.getGain_dB());
        }
    }
    
    figure(800, 600);
    subplot(2, 1, 1);
    plot(time_ms, input_power_dB, "-", opts({{"color", "blue"}, {"label", "Input Power"}}));
    plot(time_ms, output_power_dB, "-", opts({{"color", "red"}, {"label", "Output Power"}}));
    axhline(-10, opts({{"color", "black"}, {"linestyle", "--"}}));
    ylabel("Power (dB)");
    title("Digital AGC: Automatic Gain Control");
    legend(true);
    grid(true);
    
    subplot(2, 1, 2);
    plot(time_ms, gain_dB_vec, "-", opts({{"color", "green"}, {"label", "AGC Gain"}}));
    xlabel("Time (ms)");
    ylabel("Gain (dB)");
    grid(true);
    
    savefig("ch14_digital_agc.svg");
    return 0;
}
```

### 14.11.4 Digital Control for Grid-Tied Inverter

**Problem:** Implement digital control for a single-phase grid-tied inverter with current regulation.

**Control Structure:**

```
┌─────────────────────────────────────────────────────────────────────────┐
│           GRID-TIED INVERTER DIGITAL CONTROL                           │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   DC Bus    ┌─────────┐    ┌─────┐                                    │
│   ──────────┤H-Bridge │────┤ LCL │──────── Grid                        │
│   400V DC   │Inverter │    │Filter│         230V AC                    │
│             └────┬────┘    └─────┘                                     │
│                  │                                                      │
│             PWM (20 kHz)                                               │
│                  │                                                      │
│   DSP Control:   │                                                      │
│   ┌──────────────▼──────────────────────────────────────────┐          │
│   │                                                          │         │
│   │  P_ref    ┌─────┐   i_ref   ┌────────┐   PWM            │         │
│   │  ────────►│P→I  │──────────►│Current │──────►            │         │
│   │  Q_ref    │Calc │           │Control │                   │         │
│   │  ────────►│     │           │(PR+HC) │                   │         │
│   │           └─────┘           └────────┘                   │         │
│   │               ▲                  ▲                        │         │
│   │               │                  │                        │         │
│   │           ┌───┴───┐         ┌───┴───┐                    │         │
│   │           │  PLL  │         │i_grid │                    │         │
│   │           │(Grid  │         │  ADC  │                    │         │
│   │           │ Sync) │         │       │                    │         │
│   │           └───────┘         └───────┘                    │         │
│   │               ▲                                          │         │
│   │               │                                          │         │
│   │           v_grid ADC                                     │         │
│   └──────────────────────────────────────────────────────────┘         │
│                                                                         │
│   Current Control: Proportional-Resonant (PR) + Harmonic Compensators │
│   H(s) = Kp + Kr*s/(s² + ω₀²) + Σ Kh*s/(s² + (hω₀)²)                  │
│                                 h=3,5,7                                │
└─────────────────────────────────────────────────────────────────────────┘
```

**Discrete PR Controller:**

For fundamental frequency $\omega_0 = 2\pi \cdot 50$ rad/s:

$$H_{PR}(s) = K_p + \frac{K_r s}{s^2 + \omega_0^2}$$

Discretization using Tustin transform:

$$H_{PR}(z) = K_p + K_r \frac{2T_s(z^2 - 1)}{(4 + T_s^2\omega_0^2)z^2 + (-8 + 2T_s^2\omega_0^2)z + (4 + T_s^2\omega_0^2)}$$

**Implementation:**

```cpp
#include <cppplot/cppplot.hpp>
#include <cmath>

// Discrete Proportional-Resonant Controller
class DiscretePR {
private:
    double Kp, Kr;
    double w0;          // Resonant frequency
    double Ts;          // Sample time
    
    // State variables (second-order IIR)
    double x1, x2;      // Internal states
    double y_prev;
    
    // Discretization coefficients
    double a0, a1, a2;  // Denominator
    double b0, b1, b2;  // Numerator
    
public:
    DiscretePR(double Kp_, double Kr_, double f0, double sample_freq) {
        Kp = Kp_;
        Kr = Kr_;
        w0 = 2 * M_PI * f0;
        Ts = 1.0 / sample_freq;
        
        // Tustin discretization with pre-warping
        double wd = 2/Ts * std::tan(w0 * Ts / 2);
        
        // For resonant part: H(s) = Kr*s/(s^2 + w0^2)
        // Bilinear: s = 2/Ts * (z-1)/(z+1)
        double c = 2 / Ts;
        double d = wd * wd;
        
        // Coefficients for resonant part
        double norm = c*c + d;
        a0 = 1.0;
        a1 = 2 * (d - c*c) / norm;
        a2 = 1.0;  // Symmetric denominator
        
        b0 = Kr * c / norm;
        b1 = 0;
        b2 = -Kr * c / norm;
        
        x1 = x2 = y_prev = 0;
    }
    
    double process(double error) {
        // Proportional term
        double y_p = Kp * error;
        
        // Resonant term (Direct Form II)
        double w = error - a1 * x1 - a2 * x2;
        double y_r = b0 * w + b1 * x1 + b2 * x2;
        
        // Update states
        x2 = x1;
        x1 = w;
        
        return y_p + y_r;
    }
    
    void reset() {
        x1 = x2 = y_prev = 0;
    }
};

// Complete Grid-Tied Inverter Controller
class GridTiedInverterControl {
private:
    DiscretePR pr_fund;      // Fundamental (50 Hz)
    DiscretePR pr_3rd;       // 3rd harmonic (150 Hz)
    DiscretePR pr_5th;       // 5th harmonic (250 Hz)
    
    // PLL for grid synchronization
    double theta_pll;
    double w_pll;
    double Kp_pll, Ki_pll;
    double pll_integral;
    double Ts;
    
public:
    GridTiedInverterControl(double sample_freq) 
        : pr_fund(5.0, 500.0, 50, sample_freq),
          pr_3rd(0, 200.0, 150, sample_freq),
          pr_5th(0, 200.0, 250, sample_freq),
          Ts(1.0/sample_freq) {
        
        theta_pll = 0;
        w_pll = 2 * M_PI * 50;
        Kp_pll = 100;
        Ki_pll = 5000;
        pll_integral = 0;
    }
    
    // PLL update - returns grid angle
    double updatePLL(double v_grid) {
        // Quadrature signal generator (simplified)
        double v_q = v_grid * std::cos(theta_pll);  // Should be ~0 when locked
        
        // PI controller
        double error = -v_q;
        pll_integral += Ki_pll * Ts * error;
        double w_adjust = Kp_pll * error + pll_integral;
        
        w_pll = 2 * M_PI * 50 + w_adjust;
        theta_pll += w_pll * Ts;
        
        // Wrap angle
        while (theta_pll > 2 * M_PI) theta_pll -= 2 * M_PI;
        while (theta_pll < 0) theta_pll += 2 * M_PI;
        
        return theta_pll;
    }
    
    // Current control - returns PWM duty cycle
    double currentControl(double i_ref, double i_meas) {
        double error = i_ref - i_meas;
        
        // PR controller for fundamental + harmonic compensators
        double u = pr_fund.process(error) 
                 + pr_3rd.process(error) 
                 + pr_5th.process(error);
        
        // Saturate to valid PWM range
        if (u > 0.95) u = 0.95;
        if (u < -0.95) u = -0.95;
        
        return u;
    }
    
    double getTheta() const { return theta_pll; }
    double getFreq() const { return w_pll / (2 * M_PI); }
};
```

### 14.11.5 Summary: Digital Control in EE and Telecom

| Application | Sample Rate | Controller Type | Key Challenge |
|-------------|-------------|-----------------|---------------|
| **PFC** | 50-100 kHz | PI + Resonant | Two-loop coordination |
| **DPLL** | MHz range | Type II digital | Phase noise, jitter |
| **AGC** | Signal BW | Log-domain | Attack/release balance |
| **Grid inverter** | 10-50 kHz | PR + Harmonics | Grid synchronization |

**Common Digital Control Patterns:**

1. **Cascaded loops**: Fast inner loop (current), slow outer loop (voltage/power)
2. **Resonant controllers**: For AC quantities at known frequencies
3. **Feedforward**: Reduce control effort, improve transients
4. **Anti-windup**: Prevent integrator saturation during limits

**Discretization Guidelines:**

| Method | Best For | EE/Telecom Example |
|--------|----------|-------------------|
| **Tustin** | General, preserves frequency | PR controllers |
| **ZOH** | Accurate step response | Power converters |
| **Pre-warping** | Resonant controllers | Harmonic compensators |
| **Impulse invariant** | Filter design | Anti-alias filters |

---

### 📋 Signal Dictionary — Digital Control of a DC Motor (Sampled-Data System)

| Signal | Symbol | Domain | Physical meaning | Typical unit |
|--------|--------|--------|------------------|--------------|
| Reference speed | $r(k)$ | Discrete | Desired motor speed at sample $k$ | rad/s |
| Measured speed | $y(k)$ | Discrete | Encoder reading latched at $t = kT$ | rad/s |
| Continuous speed | $y(t)$ | Continuous | Actual shaft speed between samples | rad/s |
| Tracking error | $e(k) = r(k) - y(k)$ | Discrete | Speed deviation at sample instant | rad/s |
| Control voltage | $u(k)$ | Discrete | DAC output, held for one period $T$ | V |
| Held voltage | $u_{ZOH}(t)$ | Continuous | Staircase from ZOH: constant on $[kT, (k+1)T)$ | V |
| Motor current | $i(t)$ | Continuous | Armature current (between samples, unmeasured) | A |
| Sampling period | $T$ | — | Time between consecutive samples | s |
| Sampling frequency | $f_s = 1/T$ | — | How fast the controller runs | Hz |
| Anti-alias filter output | $y_f(t)$ | Continuous | Low-pass filtered measurement before ADC | rad/s |
| Quantization error | $e_q(k)$ | Discrete | ADC resolution limit: $y(k) - y_{true}(kT)$ | rad/s |
| z-domain plant | $G(z)$ | z-domain | $\mathcal{Z}\{\text{ZOH} \cdot G(s)\}$ — includes hold effect | — |

> **Key insight:** In digital control, the *continuous* signal $y(t)$ and the *sampled* signal $y(k)$ are **different objects**. The plant operates in continuous time; the controller sees only samples. The ZOH creates a staircase $u_{ZOH}(t)$ that the motor must follow between updates. Every signal in the loop has a domain — confusing them is the #1 source of digital control errors.

---

## 14.12 Exercises

**E13.1 (Discretization Comparison)**
Discretize the continuous-time transfer function
$$G(s) = \frac{1}{s+2}$$
using Zero-Order Hold (ZOH) for three sampling periods: $T = 0.1$ s, $T = 0.5$ s, and $T = 1.0$ s.

(a) Compute $G(z)$ analytically for each $T$.

(b) Plot the step response of the continuous system and all three discrete systems on the same figure.

(c) At what sampling period does the discrete response begin to deviate significantly from the continuous one? Relate this to the system bandwidth.

---

**E13.2 (Jury Stability Test)**
Apply the Jury stability test to the characteristic polynomial:
$$P(z) = z^3 - 1.2z^2 + 0.5z - 0.06$$

(a) Check the three necessary conditions: $P(1) > 0$, $(-1)^3 P(-1) > 0$, and $|a_0| < a_n$.

(b) Construct the full Jury array.

(c) Determine whether the system is stable.

(d) Verify your answer by computing the roots of $P(z)$ numerically and checking $|z_i| < 1$.

---

**E13.3 (Deadbeat Controller Design)**
For the discrete-time plant
$$G(z) = \frac{0.5}{z - 0.8}$$

(a) Design a deadbeat controller $D(z)$ such that the closed-loop response to a unit step reaches steady state in exactly one sample.

(b) Verify that the closed-loop transfer function is $H(z) = z^{-1}$ (one-step delay).

(c) Compute the control signal $u(k)$ for a unit step input. Comment on the magnitude of the initial control effort.

---

**E13.4 (Z-Transform Derivation)**
Find the Z-transform of
$$f(kT) = e^{-2kT}\sin(3kT)$$

(a) Derive $F(z)$ using the Z-transform definition or the standard table entry for $e^{-akT}\sin(\omega kT)$.

(b) Verify your result for $T = 0.1$ s by computing the first 5 values of $f(kT)$ and checking against the inverse Z-transform.

---

**E13.5 (s-Plane to z-Plane Mapping)**
Given continuous-time specifications $\zeta = 0.7$ and $\omega_n = 5$ rad/s with sampling period $T = 0.1$ s:

(a) Compute the desired continuous-time poles $s_{1,2}$.

(b) Map these poles to the z-plane using $z = e^{sT}$.

(c) Determine the corresponding z-plane magnitude and angle. Sketch the z-plane region corresponding to $\zeta \geq 0.7$ and $\omega_n \leq 5$ rad/s.

---

**E13.6 (Discretization with cppplot)**
Using cppplot, discretize the continuous-time system
$$G(s) = \frac{10}{s^2 + 3s + 10}$$
using ZOH, Tustin, and Forward Euler methods with $T = 0.1$ s.

(a) Plot the step responses of all three discretizations and the continuous system.

(b) Compare the steady-state values and transient characteristics.

(c) Which method best preserves the continuous system behavior? Why?

---

**E13.7 (Digital PID Controller Design)**
For the continuous-time plant $G(s) = \frac{1}{s(s+1)}$, design a digital PID controller with $T = 0.05$ s to achieve a settling time $t_s \leq 2$ s.

(a) Discretize the plant using ZOH.

(b) Design a digital PID controller (use the velocity or position form).

(c) Simulate the closed-loop step response and verify $t_s \leq 2$ s.

(d) Check the control signal $u(k)$ — is it within reasonable bounds?

(e) What happens if you change the sampling period to $T = 0.5$ s? Explain.

---

**E13.8 🔴 (Level 3 — Aliasing Disaster)**
A vibration sensor on a motor reads a 900 Hz mechanical resonance. The digital controller samples at $f_s = 1000$ Hz with no anti-alias filter.

(a) What frequency does the 900 Hz signal appear as after sampling? (Apply the aliasing formula.)

(b) The aliased frequency is near the controller's bandwidth. Describe the physical consequence.

(c) You add a 2nd-order Butterworth anti-alias filter with cutoff 400 Hz. How much is the 900 Hz component attenuated? Is it enough?

(d) A colleague suggests "just sample faster." To push the aliased 900 Hz above 400 Hz (where the filter kills it), what minimum $f_s$ is needed? Is this practical for a microcontroller running a PID at each sample?

**E13.9 🔴 (Level 3 — ZOH Delay and Stability)**
The ZOH introduces an effective delay of $T/2$ seconds into the loop.

(a) For a continuous-time system with phase margin PM = 45° at crossover frequency $\omega_c = 50$ rad/s, what is the maximum sampling period $T$ before the ZOH delay erodes all phase margin?

(b) Design the continuous controller first, then discretize with ZOH at $T$ = 1 ms and $T$ = 10 ms. Compare the Bode plots. At what $T$ does the digital system become unstable?

(c) Your plant has an additional 0.5 ms computational delay (sensor read → compute → actuator write). How does this compound with the ZOH delay? Modify your stability analysis.

**E13.10 ⚫ (Level 4 — The Sampling Theorem is Not Enough)**
Shannon says: sample at 2× the highest frequency. Control engineers say: sample at 10–20× the closed-loop bandwidth.

(a) Why is the Shannon criterion *insufficient* for control? (Hint: Shannon guarantees signal reconstruction, not control performance.)

(b) A student designs a continuous controller with perfect phase margin, then discretizes at exactly 2× bandwidth. The system oscillates. Explain the mechanism.

(c) Write a one-paragraph explanation of why "fast enough sampling" is a necessary but not sufficient condition for good digital control. What other factors matter?

---

## 14.13 Summary

| Concept | Key Point |
|---------|-----------|
| Sampling | Must satisfy Nyquist criterion |
| Z-transform | Discrete analog of Laplace |
| Stability | Poles inside unit circle |
| Discretization | Choose method based on requirements |
| Digital PID | Position or velocity form |
| Sample rate | 10-20× closed-loop bandwidth |

---

## References


1. Franklin, G.F. et al. (1998). *Digital Control of Dynamic Systems*
2. Åström, K.J. & Wittenmark, B. (2011). *Computer-Controlled Systems*
3. Erickson, R.W. & Maksimovic, D. (2020). *Fundamentals of Power Electronics*
4. Best, R.E. (2007). *Phase-Locked Loops: Design, Simulation, and Applications*
5. Teodorescu, R. et al. (2011). *Grid Converters for Photovoltaic and Wind Power Systems*
