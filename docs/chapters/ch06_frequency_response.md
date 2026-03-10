# Chapter 6: Frequency Response Analysis

---

## 🎯 Learning Objectives and Outcomes

### Chapter Objectives

This chapter introduces frequency-domain analysis methods, providing powerful tools for understanding system behavior and designing controllers based on sinusoidal response characteristics.

### Prerequisites
- Chapter 3: Laplace Transform and Transfer Functions
- Chapter 4: Time-Domain Analysis
- Complex number arithmetic

---

## Why This Chapter Matters: Real Systems Have Bandwidth Limits

> **The Real Engineering Problem:** Your controller design looks perfect in simulation. But when you test it, the motor makes a high-pitched noise and heats up. What went wrong?

### The Hidden Truth: Every Physical Component Has Bandwidth

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    FREQUENCY LIMITS IN REAL SYSTEMS                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   COMPONENT               TYPICAL BANDWIDTH           WHAT HAPPENS BEYOND   │
│   ─────────               ─────────────────           ────────────────────  │
│   DC motor (velocity)     10-100 Hz                   Phase lag, reduced    │
│                                                       response              │
│                                                                             │
│   Hydraulic actuator      5-50 Hz                     Oil compressibility   │
│                                                       causes lag            │
│                                                                             │
│   Temperature sensor      0.01-1 Hz (thermal mass)    Cannot track fast     │
│                                                       changes               │
│                                                                             │
│   Op-amp                  1 MHz (but GBW limited)     High gain reduces BW  │
│                                                                             │
│   Power amplifier         1-10 kHz                    Slew rate limits      │
│                                                                             │
│   Digital controller      fs/10 (sample rate limit)  Delay causes phase lag│
│                                                                             │
│   Mechanical structure    10-1000 Hz (resonance)     AMPLIFICATION at       │
│                                                       resonant frequency!   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### What Frequency Response Tells You

| Bode Plot Feature | Physical Meaning | Design Impact |
|-------------------|------------------|---------------|
| **Low-freq gain** | Steady-state accuracy | Higher is better for tracking |
| **Bandwidth (-3dB)** | Speed of response | Limited by actuator/sensor |
| **Resonant peak** | Oscillation tendency | Must be damped or avoided |
| **High-freq rolloff** | Noise rejection | Steeper rolloff = less noise |
| **Phase at crossover** | Stability margin | <-180° means instability |

**Example: Why Your Motor Makes Noise**

```
Your design: Kp = 100 for fast response
Bode plot shows: Crossover at 500 Hz

But motor driver has: 10 kHz PWM
And motor winding: L/R = 1ms time constant → 160 Hz bandwidth

At 500 Hz:
- Motor driver is fine (10 kHz >> 500 Hz)
- Motor winding is NOT fine (160 Hz << 500 Hz)
- Controller commands 500 Hz signal
- Motor cannot follow → current lags → generates heat
- PWM switching at 10 kHz causes audible noise

SOLUTION: Frequency response analysis shows you must 
          limit bandwidth to 50-100 Hz for this motor
```

---

## Learning Outcomes

After completing this chapter, students will be able to:

| Bloom's Level | Outcome |
|---------------|---------|
| **Remember** | Define frequency response, magnitude, and phase |
| **Understand** | Explain the meaning of Bode plots and their parameters |
| **Apply** | Sketch Bode plots from transfer functions |
| **Analyze** | Determine stability margins (gain margin, phase margin) |
| **Evaluate** | Assess system performance from frequency response |
| **Create** | Design controllers to achieve required stability margins |

---

## 6.1 Introduction to Frequency Response

### 6.1.1 Motivation: Why Frequency Analysis?

In many practical applications, input signals are not step or impulse functions, but **periodic signals** or **random noise**:

| Application | Signal | Frequency Range |
|-------------|--------|-----------------|
| Aircraft control | Wind disturbance | 0.1-10 Hz |
| Audio systems | Music, voice | 20 Hz - 20 kHz |
| Industrial robots | Mechanical vibration | 10-100 Hz |
| Power grid | AC voltage | 50/60 Hz |

### 6.1.2 Definition of Frequency Response

**Frequency response** describes how a system responds to **sinusoidal** input at different frequencies.

> **Fundamental Theorem:**
> For a linear time-invariant (LTI) system, if the input is a sine wave with frequency ω:
> $$u(t) = A\sin(\omega t)$$
> 
> Then at steady state, the output is also a sine wave at the same frequency:
> $$y_{ss}(t) = A|G(j\omega)|\sin(\omega t + \angle G(j\omega))$$

Where:
- $|G(j\omega)|$: **Magnitude** - gain factor at frequency ω
- $\angle G(j\omega)$: **Phase** - angular delay at frequency ω

### 6.1.3 Frequency Transfer Function

Given transfer function $G(s)$, the **frequency transfer function** is:

$$G(j\omega) = G(s)|_{s=j\omega}$$

**Example:** Given $G(s) = \frac{1}{s+1}$

$$G(j\omega) = \frac{1}{j\omega + 1}$$

$$|G(j\omega)| = \frac{1}{\sqrt{\omega^2 + 1}}$$

$$\angle G(j\omega) = -\arctan(\omega)$$

---

## 6.2 Bode Plots

### 6.2.1 Structure of Bode Plot

Bode plot consists of **two graphs**:

1. **Magnitude plot**: Magnitude (dB) vs. frequency (log scale)
   $$M_{dB} = 20\log_{10}|G(j\omega)|$$

2. **Phase plot**: Phase (degrees) vs. frequency (log scale)

**Advantages of Bode plots:**
- Multiplication becomes addition (dB scale)
- Easy asymptotic approximation
- Intuitive frequency range understanding

### 6.2.2 Bode Plots of Basic Elements

#### 1. Constant K
$$G(s) = K$$
- Magnitude: $20\log_{10}|K|$ dB (horizontal line)
- Phase: 0° if K > 0, ±180° if K < 0

#### 2. Integrator $\frac{1}{s}$
- Magnitude: -20 dB/decade slope
- Phase: -90° (constant)

#### 3. First-Order Pole $\frac{1}{s/\omega_p + 1}$
- Corner frequency: $\omega_p$
- Low frequency: 0 dB, 0°
- High frequency: -20 dB/decade, -90°

#### 4. Second-Order System
$$G(s) = \frac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}$$

- Resonance peak depends on ζ
- Peak magnitude: $M_r = \frac{1}{2\zeta\sqrt{1-\zeta^2}}$

---

## 6.3 Composite Bode Plot Construction

The central skill of frequency response analysis is constructing **composite Bode plots** by hand — decomposing any transfer function $G(s)$ into basic building blocks (from §6.2.2 above), drawing each one's asymptotic contribution, and summing them graphically. This section provides the systematic method.

### 6.3.1 Step-by-Step Method

**Step 1 — Put $G(s)$ in Bode form (time-constant form):**

Factor out constants so that every first-order term looks like $(1 + s/\omega_i)$ and every second-order term looks like the standard form:

$$G(s) = K_0 \;\frac{\displaystyle\prod_i(1+s/z_i)}{\displaystyle\prod_j(1+s/p_j)} \cdot \frac{1}{\left(\dfrac{s}{\omega_n}\right)^{\!2} + 2\zeta\dfrac{s}{\omega_n} + 1}$$

where $K_0$ is the **Bode gain** (the DC value when all time-constant factors equal 1) and any pure integrators/differentiators $s^{\pm k}$ are kept separate.

**Step 2 — Identify the building blocks and their individual Bode plots:**

| Building Block | Magnitude Asymptote | Phase |
|---|---|---|
| **Constant gain $K_0$** | Flat line at $20\log_{10}|K_0|$ dB | $0°$ if $K_0>0$; $\pm180°$ if $K_0<0$ |
| **Integrator $1/s$** | $-20$ dB/dec through 0 dB at $\omega=1$ | $-90°$ (constant) |
| **Differentiator $s$** | $+20$ dB/dec through 0 dB at $\omega=1$ | $+90°$ (constant) |
| **First-order zero $(1+s/z)$** | 0 dB for $\omega\ll z$; $+20$ dB/dec for $\omega\gg z$; break at $\omega=z$ | $0°\to+90°$, centered at $\omega=z$ |
| **First-order pole $1/(1+s/p)$** | 0 dB for $\omega\ll p$; $-20$ dB/dec for $\omega\gg p$; break at $\omega=p$ | $0°\to-90°$, centered at $\omega=p$ |
| **Underdamped complex pair** | Resonant peak at $\omega_n$ of height $\approx 1/(2\zeta)$ ($+20\log_{10}(1/2\zeta)$ dB); $-40$ dB/dec for $\omega\gg\omega_n$ | $0°\to-180°$ transition centered at $\omega_n$ |

**Step 3 — Magnitude: Sum the individual dB contributions.**

Because $20\log_{10}|G| = 20\log_{10}|K_0| + \sum 20\log_{10}|\text{block}_i|$, multiplying transfer functions in the $s$-domain becomes **addition on the dB scale**:

$$M_{\text{total}}(\omega)\;[\text{dB}] = \sum_{k} M_k(\omega)\;[\text{dB}]$$

Draw each building block's asymptote on the **same** semilog axes, then add them graphically.

**Step 4 — Phase: Sum the individual phase contributions.**

$$\angle G(j\omega) = \angle K_0 + \sum_i \angle(\text{block}_i)$$

Each first-order term transitions over roughly two decades (one decade below to one decade above its break frequency).

**Step 5 — Corner corrections (smooth the asymptote).**

At every break frequency the exact curve differs from the straight-line asymptote:

| Feature | Correction at break frequency |
|---|---|
| First-order pole/zero | $\pm3$ dB magnitude; $\pm45°$ phase |
| One decade below/above break | $\pm1$ dB; $\pm5.7°$ |
| Underdamped pair (ζ < 0.707) | Peak exceeds asymptote by $20\log_{10}(1/2\zeta)$ dB |

Apply these corrections to convert the piecewise-linear sketch into a smooth curve.

### 6.3.2 Worked Example

**Transfer function:**
$$G(s) = \frac{100(s+1)}{s(s+10)(s+50)}$$

**Step 1 — Bode form.** Factor each bracket into time-constant form:

$$G(s) = \frac{100 \cdot 1 \cdot (1+s/1)}{s \cdot 10 \cdot (1+s/10) \cdot 50 \cdot (1+s/50)}
       = \frac{100}{10 \cdot 50} \cdot \frac{(1+s)}{s\,(1+s/10)(1+s/50)}
       = 0.2\,\frac{(1+s)}{s\,(1+s/10)(1+s/50)}$$

So $K_0 = 0.2$.

**Step 2 — Identify building blocks:**

| Block | Break freq (rad/s) | Slope change | Phase contribution |
|---|---|---|---|
| $K_0 = 0.2$ | — | $20\log_{10}(0.2) = -14$ dB flat | $0°$ |
| $1/s$ (integrator) | — | $-20$ dB/dec | $-90°$ |
| $(1+s)$ (zero) | $\omega = 1$ | $+20$ dB/dec above 1 | $0°\to+90°$ |
| $1/(1+s/10)$ (pole) | $\omega = 10$ | $-20$ dB/dec above 10 | $0°\to-90°$ |
| $1/(1+s/50)$ (pole) | $\omega = 50$ | $-20$ dB/dec above 50 | $0°\to-90°$ |

**Step 3 — Asymptotic magnitude construction:**

```
Magnitude (dB)
 │
 20│         slope = -20 dB/dec
   │\          (integrator + K₀)
  0│ \                ← zero at ω=1 adds +20
   │  \               ───────────── net slope = 0 dB/dec
-20│   \──────────────────\            ← pole at ω=10 subtracts 20
   │                       \              net slope = -20 dB/dec
-40│                        \      ← pole at ω=50 subtracts 20
   │                         \\       net slope = -40 dB/dec
-60│                           \\
   │                             \
   └──┬───┬───┬───┬───┬───┬───┬───► ω (log)
    0.1   1   5  10  20  50  100
           ↑       ↑       ↑
          zero   pole₁   pole₂
```

**Step 4 — Asymptotic phase construction:**

| Frequency range | Cumulative phase |
|---|---|
| $\omega \ll 1$ | $-90°$ (integrator only) |
| $\omega \approx 1$ | $-90° + 45° = -45°$ (zero contributes $+45°$ at its break) |
| $\omega \approx 10$ | $-45° - 45° = -90°$ (first pole contributes $-45°$) |
| $\omega \approx 50$ | $-90° - 45° = -135°$ (second pole contributes $-45°$) |
| $\omega \gg 50$ | $-90° + 90° - 90° - 90° = -180°$ (final value) |

**Step 5 — Corner corrections:** At each break frequency shift the curve by $\pm3$ dB from the asymptote.

### 6.3.3 Verification with CppPlot

Use CppPlot to overlay the exact Bode plot on the hand-sketched asymptotes:

```cpp
#include <cppplot/cppplot.hpp>
#include <cmath>
#include <vector>

using namespace cppplot;

int main() {
    // G(s) = 100(s+1) / [s(s+10)(s+50)]
    // Numerator coefficients (descending powers): s + 1  → 100s + 100
    // Denominator: s(s+10)(s+50) = s^3 + 60s^2 + 500s
    TransferFunction G({100, 100}, {1, 60, 500, 0});

    // --- Exact Bode plot ---
    bode(G);                       // magnitude and phase vs frequency
    title("Composite Bode Plot — G(s) = 100(s+1)/[s(s+10)(s+50)]");

    // --- Asymptotic magnitude overlay ---
    std::vector<double> w_vec, mag_asymp;
    for (double w = 0.01; w <= 1000; w *= 1.05) {
        w_vec.push_back(w);

        // Start with K0 and integrator: 20*log10(0.2) - 20*log10(w)
        double m = 20 * std::log10(0.2) - 20 * std::log10(w);

        // Zero at 1: +20 dB/dec above w = 1
        if (w > 1)   m += 20 * std::log10(w / 1.0);
        // Pole at 10: -20 dB/dec above w = 10
        if (w > 10)  m -= 20 * std::log10(w / 10.0);
        // Pole at 50: -20 dB/dec above w = 50
        if (w > 50)  m -= 20 * std::log10(w / 50.0);

        mag_asymp.push_back(m);
    }

    figure();
    plot(w_vec, mag_asymp, "--",
         opts({{"color", "red"}, {"label", "Asymptotic (hand-sketch)"}}));
    xscale("log");
    xlabel("Frequency (rad/s)");
    ylabel("Magnitude (dB)");
    title("Exact vs Asymptotic Bode Magnitude");
    legend(true);
    grid(true);
    savefig("ch06_composite_bode_example.svg");

    return 0;
}
```

The red dashed asymptotic curve should closely track the exact blue curve, differing by at most 3 dB at each break frequency. This confirms that the hand-sketch method provides an excellent first approximation.

> **Key Takeaway:** Composite Bode construction is the **single most important hand-analysis skill** in frequency response. It lets you rapidly estimate gain crossover, phase margin, and bandwidth without computation — essential in exams and in early design iterations.

> **🔍 Stop and Think — Mechanism Questions**
>
> Before moving on to stability margins:
> 1. The Bode magnitude plot rolls off at -20 dB/decade per pole. *Physically*, why does a system attenuate high-frequency inputs? (Hint: think about mass/inertia — heavy objects cannot follow fast commands.)
> 2. Each pole adds up to -90° of phase lag. What does phase lag *mean* for the physical system? (The output is delayed relative to the input. But why?)
> 3. If a system has two poles at the same frequency, the roll-off is -40 dB/decade. Give a physical example where this happens and explain why the doubled rate makes sense.

---

## 6.4 Stability Margins

### 6.4.1 Gain Margin (GM)

**Definition:** Factor by which gain can increase before instability.

At **phase crossover frequency** $\omega_{pc}$ (phase = -180°):
$$GM = \frac{1}{|G(j\omega_{pc})|}$$
$$GM_{dB} = -20\log_{10}|G(j\omega_{pc})|$$

### 6.4.2 Phase Margin (PM)

**Definition:** Additional phase lag allowed before instability.

At **gain crossover frequency** $\omega_{gc}$ ($|G(j\omega)| = 1$):
$$PM = 180° + \angle G(j\omega_{gc})$$

### 6.4.3 Typical Requirements

| Parameter | Requirement |
|-----------|-------------|
| GM | ≥ 6 dB |
| PM | ≥ 30° - 60° |

### 6.4.4 Computing Stability Margins with CppPlot

CppPlot provides `margin()` to compute gain and phase margins directly from a transfer function:

```cpp
#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    // Open-loop TF: G(s) = 10/[s(s+1)(s+2)]
    TransferFunction G({10}, {1, 3, 2, 0});

    // Compute stability margins
    auto margins = margin(G);
    std::cout << "Gain Margin: " << margins.gm << " dB at "
              << margins.wpc << " rad/s\n";
    std::cout << "Phase Margin: " << margins.pm << "° at "
              << margins.wgc << " rad/s\n";

    // Visual verification: Bode plot with margins annotated
    figure(900, 700);
    bode(G);  // bode() automatically marks GM and PM when called after margin()
    title("Bode Plot with Stability Margins — G(s) = 10/[s(s+1)(s+2)]");
    savefig("ch06_margin_example.svg");

    // Analytical check:
    // Phase crossover: ∠G(jω) = -180°
    //   -90° - atan(ω) - atan(ω/2) = -180°  →  ω_pc = √2 rad/s
    //   |G(j√2)| = 10/[√2 · √3 · √6] = 10/√36 = 10/6 ≈ 1.667
    //   GM = -20·log10(1.667) ≈ -4.4 dB  → UNSTABLE (GM < 0)
    // This system is actually unstable at K=10! Reduce K for stability.

    return 0;
}
```

> **Key insight:** A negative gain margin (in dB) means the system is already unstable — the gain must be *reduced* to achieve stability. Use `margin()` early in the design process to avoid surprises.

### 6.4.5 Minimum Phase and Non-Minimum Phase Systems

**Minimum phase systems** have all zeros in the left half-plane (LHP). For these systems, the Bode magnitude plot *uniquely determines* the phase plot through **Bode's gain-phase relationship**:

$$\angle G(j\omega_0) \approx n \times 90°$$

where $n$ is the slope of the magnitude plot in decades per decade at $\omega_0$. This means you can infer stability margins from the magnitude plot alone.

**Non-minimum phase systems** have one or more right half-plane (RHP) zeros. They exhibit "extra" phase lag beyond what the magnitude curve implies:

1. **Excess phase lag:** The Bode phase drops below what a minimum-phase system with identical magnitude response would produce — by up to $180°$ per RHP zero.
2. **Bandwidth limitation:** A useful rule of thumb is $\omega_{BW} < |z_{RHP}|/2$. Attempting higher bandwidth pushes the phase margin negative and leads to instability.
3. **Inverse response:** In the time domain, the step response initially moves in the *wrong* direction before correcting — a signature of RHP zeros.

**Example — same magnitude, different phase:**

| System | Transfer Function | Type | DC gain |
|---|---|---|---|
| $G_1(s)$ | $\dfrac{s+2}{s+1}$ | Minimum phase | 2 |
| $G_2(s)$ | $\dfrac{-s+2}{s+1}$ | Non-minimum phase | 2 |

Both have $|G_1(j\omega)| = |G_2(j\omega)|$ for all $\omega$ (verify: numerator magnitudes $\sqrt{\omega^2+4}$ are identical). However:
- $G_1$: phase goes from $0°$ toward $+90°$ — phase *lead*
- $G_2$: phase goes from $0°$ toward $-180°$ — phase *lag*

The phase difference reaches $180°$ at high frequency. For feedback design, $G_2$ requires a much lower crossover frequency to maintain adequate phase margin.

> **Practical impact:** Non-minimum phase behavior arises in flexible structures (collocated vs. non-collocated sensors), some chemical processes, and systems with intentional time delay. Always check for RHP zeros before setting bandwidth targets.

---

## 6.5 Bandwidth and Performance

### 6.5.1 Bandwidth Definition

The frequency where $|G(j\omega)| = \frac{1}{\sqrt{2}}$ (-3 dB)

### 6.5.2 Relationship to Time Domain

- Larger bandwidth → faster response
- For second-order systems with damping ratio ζ:
  $$\omega_{BW} \approx \omega_n\sqrt{1 - 2\zeta^2 + \sqrt{4\zeta^4 - 4\zeta^2 + 2}}$$
- Simplified approximation (ζ ≈ 0.7): $\omega_{BW} \approx \frac{4}{t_s}$
- General rule: $t_r \approx \frac{0.35}{f_{BW}}$ where $f_{BW} = \omega_{BW}/(2\pi)$

---

## 6.6 Electrical and Telecommunications System Examples

### 6.6.1 Power Electronics: Buck Converter Loop Gain Analysis

**Problem:** Design a voltage-mode controlled Buck converter with adequate stability margins.

**System Description:**
- Input voltage: $V_{in} = 12V$
- Output voltage: $V_{out} = 5V$
- Switching frequency: $f_{sw} = 100 kHz$
- Load: $R = 5\Omega$
- Filter: $L = 100\mu H$, $C = 100\mu F$

**Signal Dictionary — Buck Converter (Voltage-Mode Control)**

| Signal | Symbol | Unit | Physical Meaning | Sensor/Actuator |
|--------|--------|------|------------------|-----------------|
| Input voltage | $V_{in}$ | V | DC supply to the converter (12 V) | Power supply |
| Output voltage (controlled) | $v_{out}(t)$ | V | Regulated DC output (5 V nominal) | Resistive divider + ADC |
| Duty cycle (control) | $d(t)$ | — (0–1) | Fraction of switching period the transistor is ON | PWM modulator |
| Inductor current | $i_L(t)$ | A | Current through the LC filter inductor | Current-sense resistor / Hall sensor |
| Error voltage | $v_e = V_{ref} - v_{out}$ | V | Difference between reference and measured output | Error amplifier (op-amp) |
| Compensator output | $v_c(t)$ | V | Output of the error amplifier → compared with ramp | Compensation network (RC) |
| PWM ramp | $V_m$ | V | Sawtooth waveform amplitude — converts $v_c$ to duty cycle $d$ | Oscillator IC |
| Loop gain | $T(s)$ | — | Signal traveling around the entire feedback loop | (Analysis quantity) |

> **Engineering insight:** The LC filter creates a resonant peak in $G_{vd}(s)$. The compensator $G_c(s)$ must add enough phase margin at the crossover frequency to prevent oscillation. Every signal in this table maps to a physical wire or IC pin on the PCB.

**Loop Gain Transfer Function:**

The control-to-output transfer function for voltage-mode control:

$$G_{vd}(s) = \frac{\hat{v}_{out}(s)}{\hat{d}(s)} = V_{in} \cdot \frac{1 + sRC}{s^2LC + s\frac{L}{R} + 1}$$

With an error amplifier compensation network, the loop gain becomes:

$$T(s) = G_c(s) \cdot \frac{1}{V_m} \cdot G_{vd}(s) \cdot H(s)$$

Where:
- $G_c(s)$: Compensator transfer function
- $V_m$: PWM ramp amplitude
- $H(s)$: Feedback divider (often = 1)

**Type III Compensation Design:**

To achieve high crossover frequency and adequate phase margin:

$$G_c(s) = K_c \cdot \frac{(1 + s/\omega_{z1})(1 + s/\omega_{z2})}{s(1 + s/\omega_{p1})(1 + s/\omega_{p2})}$$

**Bode Plot Analysis with CppPlot:**

```cpp
#include <cppplot/cppplot.hpp>
#include <cmath>

using namespace cppplot;

int main() {
    // Buck converter parameters
    double Vin = 12.0;
    double Vout = 5.0;
    double L = 100e-6;
    double C = 100e-6;
    double R = 5.0;
    double Vm = 1.0;  // PWM ramp
    
    // Plant zeros and poles
    double R_ESR = 0.05;                  // Capacitor ESR (50 mΩ typical for electrolytic)
    double w_esr = 1.0 / (R_ESR * C);    // ESR zero (~200 krad/s)
    double w0 = 1.0 / std::sqrt(L * C);  // LC resonance
    double Q = R * std::sqrt(C / L);  // Quality factor
    
    // Frequency range
    std::vector<double> f_hz, mag_plant, phase_plant;
    std::vector<double> mag_comp, phase_comp;
    std::vector<double> mag_loop, phase_loop;
    
    // Type III compensator parameters (designed for 10kHz crossover)
    double fc = 10e3;  // Crossover frequency
    double wc = 2 * M_PI * fc;
    double Kc = 20.0;
    double wz1 = w0 / 2;      // Zeros below LC resonance
    double wz2 = w0 / 2;
    double wp1 = 2 * M_PI * 50e3;  // Poles at half switching freq
    double wp2 = 2 * M_PI * 50e3;
    
    for (double f = 10; f <= 100e3; f *= 1.1) {
        double w = 2 * M_PI * f;
        f_hz.push_back(f);
        
        // Plant: Gvd(s) = Vin * (1 + s/w_esr) / (s^2*LC + s*L/R + 1)
        std::complex<double> jw(0, w);
        std::complex<double> plant_num = Vin * (1.0 + jw / w_esr);  // ESR zero
        std::complex<double> plant_den = jw*jw*L*C + jw*L/R + 1.0;
        std::complex<double> Gvd = plant_num / plant_den;
        
        // Type III compensator
        std::complex<double> comp_num = Kc * (1.0 + jw/wz1) * (1.0 + jw/wz2);
        std::complex<double> comp_den = jw * (1.0 + jw/wp1) * (1.0 + jw/wp2);
        std::complex<double> Gc = comp_num / comp_den;
        
        // Loop gain
        std::complex<double> T = Gc * Gvd / Vm;
        
        mag_loop.push_back(20 * std::log10(std::abs(T)));
        phase_loop.push_back(std::arg(T) * 180 / M_PI);
    }
    
    figure(800, 600);
    
    subplot(2, 1, 1);
    plot(f_hz, mag_loop, "-", opts({{"color", "blue"}, {"label", "Loop Gain"}}));
    axhline(0, opts({{"color", "black"}, {"linestyle", "--"}, {"linewidth", "0.5"}}));
    xscale("log");
    ylabel("Magnitude (dB)");
    title("Buck Converter Loop Gain Analysis");
    grid(true);
    legend(true);
    
    subplot(2, 1, 2);
    plot(f_hz, phase_loop, "-", opts({{"color", "red"}, {"label", "Phase"}}));
    axhline(-180, opts({{"color", "black"}, {"linestyle", "--"}, {"linewidth", "0.5"}}));
    xscale("log");
    xlabel("Frequency (Hz)");
    ylabel("Phase (deg)");
    grid(true);
    
    savefig("ch06_buck_loop_gain.svg");
    return 0;
}
```

**Design Guidelines:**
- Crossover frequency: $f_c < f_{sw}/10$ (typically 5-20 kHz for 100 kHz switching)
- Phase margin: PM ≥ 45° for good transient response
- Gain margin: GM ≥ 10 dB to account for component variations

### 6.6.2 Op-Amp Gain-Bandwidth Product Limitation

**Problem:** Design an inverting amplifier with gain = -100 and analyze bandwidth limitation.

**Op-Amp Model with Finite GBW:**

$$A_{OL}(s) = \frac{A_0}{1 + s/\omega_a} \approx \frac{GBW}{s}$$

For high frequencies, where GBW = $A_0 \cdot \omega_a$ (Gain-Bandwidth Product).

**Closed-Loop Transfer Function:**

For an inverting amplifier with gain $K = -R_f/R_{in}$:

$$\frac{V_{out}}{V_{in}} = \frac{-R_f/R_{in}}{1 + (1 + R_f/R_{in})/A_{OL}(s)}$$

**Bandwidth Limitation:**

$$f_{3dB} \approx \frac{GBW}{1 + R_f/R_{in}} = \frac{GBW}{|K| + 1}$$

For K = -100 with GBW = 10 MHz:
$$f_{3dB} \approx \frac{10 \text{ MHz}}{101} \approx 99 \text{ kHz}$$

```
┌────────────────────────────────────────────────────────────────────────┐
│           OP-AMP GAIN-BANDWIDTH TRADEOFF                               │
├────────────────────────────────────────────────────────────────────────┤
│                                                                        │
│   Magnitude (dB)                                                       │
│        │                                                               │
│    100 │─────────── Open-loop A_OL                                    │
│        │           \                                                   │
│     80 │            \                                                  │
│        │             \                                                 │
│     60 │──────────────\──── Closed-loop |K|=100 (40 dB)              │
│        │               \   ← f_3dB ≈ 99 kHz                           │
│     40 │────────────────\─────────── |K|=10 (20 dB)                  │
│        │                 \  ← f_3dB ≈ 910 kHz                         │
│     20 │──────────────────\─────────────── |K|=1 (0 dB)              │
│        │                   \ ← f_3dB ≈ 5 MHz                          │
│      0 │────────────────────\────────────────────────                 │
│        │                     \                                         │
│        └─────┬──────┬──────┬──\───┬──────┬──────┬───► f (Hz)         │
│            100   1k    10k  100k   1M    10M                          │
│                                 ↑                                      │
│                               GBW = 10 MHz                             │
│                                                                        │
│   KEY INSIGHT: Higher gain → Lower bandwidth!                         │
│   Design must consider BOTH requirements.                             │
└────────────────────────────────────────────────────────────────────────┘
```

### 6.6.3 Phase-Locked Loop Bandwidth Design

**Problem:** Design a Type II PLL for clock recovery with specified loop bandwidth and phase margin.

**PLL Open-Loop Transfer Function:**

$$G_{OL}(s) = \frac{K_{pd} \cdot K_{vco}}{s} \cdot F(s)$$

Where:
- $K_{pd}$: Phase detector gain (V/rad)
- $K_{vco}$: VCO gain (rad/s/V)
- $F(s)$: Loop filter transfer function

**Second-Order Type II Loop Filter:**

$$F(s) = \frac{1 + s\tau_2}{s\tau_1}$$

This gives the characteristic open-loop:

$$G_{OL}(s) = \frac{K_{pd} K_{vco} (1 + s\tau_2)}{s^2 \tau_1} = \frac{K(1 + s\tau_2)}{s^2}$$

**Design Parameters:**

Natural frequency and damping ratio:
$$\omega_n = \sqrt{\frac{K_{pd} K_{vco}}{\tau_1}}, \quad \zeta = \frac{\omega_n \tau_2}{2}$$

**Bandwidth and Phase Margin Relationship:**

Loop bandwidth (0 dB crossover):
$$\omega_c = \omega_n \sqrt{2\zeta^2 + 1 + \sqrt{(2\zeta^2+1)^2 + 1}}$$

Phase margin:
$$PM = \arctan(\omega_c \tau_2) - \arctan\left(\frac{\omega_c}{\omega_z}\right) + 90°$$

For a simple approximation: $PM \approx \arctan(2\zeta)$

**Bode Plot Analysis:**

```cpp
#include <cppplot/cppplot.hpp>
#include <cmath>

using namespace cppplot;

int main() {
    // PLL parameters for 10 kHz loop bandwidth
    double wc = 2 * M_PI * 10e3;  // 10 kHz loop BW
    double zeta = 0.707;          // Damping ratio for 65° PM
    
    double wn = wc / std::sqrt(2*zeta*zeta + 1 + 
                std::sqrt(std::pow(2*zeta*zeta+1,2) + 1));
    double tau2 = 2 * zeta / wn;
    double K = wn * wn;  // Kpd * Kvco / tau1
    
    std::vector<double> f_hz, mag_db, phase_deg;
    
    for (double f = 1e2; f <= 1e6; f *= 1.1) {
        double w = 2 * M_PI * f;
        f_hz.push_back(f);
        
        // G_OL(jw) = K(1 + jw*tau2) / (jw)^2
        std::complex<double> jw(0, w);
        std::complex<double> num = K * (1.0 + jw * tau2);
        std::complex<double> den = jw * jw;
        std::complex<double> G = num / den;
        
        mag_db.push_back(20 * std::log10(std::abs(G)));
        phase_deg.push_back(std::arg(G) * 180 / M_PI);
    }
    
    figure(800, 600);
    
    subplot(2, 1, 1);
    plot(f_hz, mag_db, "-", opts({{"color", "blue"}, {"label", "PLL Open-Loop"}}));
    axhline(0, opts({{"color", "black"}, {"linestyle", "--"}, {"linewidth", "0.5"}}));
    xscale("log");
    ylabel("Magnitude (dB)");
    title("Type II PLL Loop Gain (BW = 10 kHz, zeta = 0.707)");
    grid(true);
    legend(true);
    
    subplot(2, 1, 2);
    plot(f_hz, phase_deg, "-", opts({{"color", "red"}, {"label", "Phase"}}));
    axhline(-180, opts({{"color", "black"}, {"linestyle", "--"}}));
    xscale("log");
    xlabel("Frequency (Hz)");
    ylabel("Phase (deg)");
    grid(true);
    
    savefig("ch06_pll_bode.svg");
    return 0;
}
```

### 6.6.4 Communication Channel Equalization

**Problem:** Analyze frequency-domain characteristics of a channel equalizer.

**Channel Model (Baseband):**

A dispersive channel introduces intersymbol interference (ISI):
$$H_{ch}(s) = \sum_{k} a_k e^{-s\tau_k}$$

For a simple two-ray model:
$$H_{ch}(s) = 1 + \alpha e^{-sT_d}$$

Where $\alpha$ is the relative amplitude and $T_d$ is the delay.

**Frequency Response:**

$$H_{ch}(j\omega) = 1 + \alpha e^{-j\omega T_d}$$

$$|H_{ch}(j\omega)| = \sqrt{(1 + \alpha\cos(\omega T_d))^2 + (\alpha\sin(\omega T_d))^2}$$
$$= \sqrt{1 + \alpha^2 + 2\alpha\cos(\omega T_d)}$$

This creates a **frequency-selective fading** pattern with nulls at:
$$\omega_{null} = \frac{(2k+1)\pi}{T_d}$$

**Zero-Forcing Equalizer:**

$$H_{eq}(j\omega) = \frac{1}{H_{ch}(j\omega)}$$

**Noise Enhancement Problem:**

At frequencies where $|H_{ch}|$ is small, the equalizer gain becomes very large, amplifying noise:

$$\text{Noise Enhancement} = \frac{1}{|H_{ch}(j\omega)|^2}$$

```
┌────────────────────────────────────────────────────────────────────────┐
│         CHANNEL AND EQUALIZER FREQUENCY RESPONSE                       │
├────────────────────────────────────────────────────────────────────────┤
│                                                                        │
│   |H| (dB)                                                             │
│        │                                                               │
│    +20 │                    ┌─┐        Equalizer H_eq                 │
│        │                   │  │        (noise enhancement!)            │
│    +10 │        ┌─┐       │  │                                        │
│        │       │  │      │    │                                       │
│      0 │──────●──●──────●────●──────●────────── Channel H_ch         │
│        │     │    │    │      │    │                                  │
│    -10 │    │      │  │        │  │                                   │
│        │   │        ││          ││                                    │
│    -20 │  │         │            │    ← Deep nulls                    │
│        │                                                               │
│        └────┬────┬────┬────┬────┬────┬────► Frequency                 │
│            0   f1   f2   f3   f4   f5                                 │
│                                                                        │
│   f_null = (2k+1)/(2·T_d) where T_d = multipath delay                 │
│                                                                        │
│   SOLUTION: Use MMSE equalizer to balance ISI reduction and          │
│             noise enhancement, or use OFDM to avoid deep nulls.       │
└────────────────────────────────────────────────────────────────────────┘
```

**MMSE Equalizer Alternative:**

$$H_{eq,MMSE}(j\omega) = \frac{H_{ch}^*(j\omega)}{|H_{ch}(j\omega)|^2 + N_0/S}$$

Where $N_0/S$ is the noise-to-signal ratio, which prevents excessive gain at nulls.

### 6.6.5 Summary: Frequency Domain in EE and Telecom

| Application | Key Frequency Metric | Design Consideration |
|-------------|---------------------|----------------------|
| **Buck converter** | Crossover freq, PM | fc < fsw/10, PM > 45° |
| **Op-amp circuits** | GBW, closed-loop BW | BW = GBW/(1+|K|) |
| **PLL** | Loop bandwidth, PM | BW vs. jitter tradeoff |
| **Channel equalizer** | Null frequencies | Noise enhancement |
| **Active filters** | Cutoff frequency | Component sensitivity |

**Cross-Domain Insight:**

The same Bode plot techniques apply across all domains:
- **Power electronics**: Stability of switching converters
- **Analog design**: Op-amp bandwidth limitations
- **Communications**: Channel equalization and PLL design
- **RF systems**: Filter design and matching networks

The universal principle: **Bandwidth × Gain = Constant** appears in:
- Op-amp GBW product
- PLL noise-bandwidth tradeoff  
- Shannon capacity limit (bandwidth vs. SNR)

---

## 📝 Exercises

### Exercise 6.1 — Asymptotic Bode Plot and Exact Gain

Consider the transfer function:

$$G(s) = \frac{10(s+1)}{s(s+10)}$$

**(a)** Identify all corner (break) frequencies and the initial low-frequency asymptote slope.

**(b)** Sketch the asymptotic (straight-line) Bode magnitude and phase plots. Clearly label slopes in each frequency region.

**(c)** Compute the **exact** gain (in dB) at $\omega = 1$ rad/s and $\omega = 10$ rad/s.

**(d)** Compare the exact values with the asymptotic approximation. At which frequency is the asymptotic error largest, and why?

---

### Exercise 6.2 — Transfer Function Identification from Bode Data

A system's Bode plot yields the following measurements:

| $\omega$ (rad/s) | Magnitude (dB) | Phase (°) |
|---|---|---|
| 0.1 | +40 | −90 |
| 1 | +20 | −135 |
| 10 | −20 | −180 |
| 100 | −60 | −270 |

**(a)** From the low-frequency slope and magnitude, determine the system type (number of pure integrators) and the DC/Bode gain.

**(b)** Identify the corner frequencies from the slope transitions.

**(c)** Propose a transfer function $G(s)$ consistent with the data.

**(d)** Verify your answer by computing $|G(j\omega)|$ at the four given frequencies.

---

### Exercise 6.3 — Resonant Peak and Frequency

For the second-order system:

$$G(s) = \frac{100}{s^2 + 10s + 100}$$

**(a)** Identify $\omega_n$ and $\zeta$.

**(b)** Using the formulas for underdamped second-order systems, compute the resonant peak $M_r$ and resonant frequency $\omega_r$.

**(c)** Under what condition on $\zeta$ does a resonant peak exist? Does this system satisfy it?

**(d)** Estimate the bandwidth $\omega_{BW}$ (the frequency where $|G(j\omega)| = -3$ dB from the DC value).

---

### Exercise 6.4 — Bode Plot Verification with CppPlot

Using the CppPlot library, verify your asymptotic sketch from Exercise 6.1:

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;

int main() {
    // Define G(s) = 10(s+1) / (s(s+10))
    TransferFunction G({10, 10}, {1, 10, 0});
    
    figure();
    bode(G);
    title("Exercise 6.4: Bode Plot Verification");
    show();
    
    // Student task: overlay your asymptotic approximation on this plot
    // Student task: mark the exact gain values at ω = 1 and ω = 10
}
```

**(a)** Run the code and compare the CppPlot output with your hand sketch.

**(b)** Add markers at $\omega = 1$ and $\omega = 10$ showing the exact gain values.

**(c)** Annotate the gain margin and phase margin on the plot.

---

### Exercise 6.5 — Composite Bode Plot Construction

Construct the composite Bode plot for:

$$G(s) = \frac{50(s+2)}{(s+0.5)(s+20)}$$

**(a)** Rewrite $G(s)$ in Bode (time-constant) form and determine the Bode gain $K_B$.

**(b)** List all corner frequencies in ascending order.

**(c)** Build the asymptotic magnitude plot step by step:
- Start with the constant $20\log_{10}|K_B|$.
- At each corner frequency, add or subtract the appropriate slope change.

**(d)** Sketch the phase plot using the $\pm 45°/\text{decade}$ rule for each term.

**(e)** Determine $\omega_{gc}$ (gain crossover) and read off the phase margin from your sketch.

---

### Exercise 6.6 — Bandwidth and Time-Domain Estimates

A system has:
- Bandwidth: $\omega_{BW} = 10$ rad/s
- DC gain: 20 dB

**(a)** Convert the DC gain from dB to absolute value.

**(b)** Using the approximate relationship $t_r \approx 1.8 / \omega_{BW}$, estimate the rise time.

**(c)** What is the steady-state gain (the ratio of output to input amplitude for a step input)?

**(d)** If the system is second-order with $\zeta = 0.5$, refine your rise time and peak overshoot estimates using the exact second-order formulas. Compare with part (b).

---

### Exercise 6.7 — Gain Crossover Design ⭐

For the open-loop transfer function:

$$G(s) = \frac{K}{s(s+1)(s+5)}$$

**(a)** Compute $|G(j\omega)|$ as a function of $\omega$ and $K$.

**(b)** Set $|G(j2)| = 1$ (i.e., 0 dB at $\omega_{gc} = 2$ rad/s) and solve for $K$.

**(c)** With your value of $K$, compute the phase margin.

**(d)** Is the resulting closed-loop system stable? Would you consider the phase margin adequate for a practical design?

**(e)** Use CppPlot to plot the Bode diagram with your computed $K$ and verify $\omega_{gc}$ and PM.

### Problem Identification Exercises (Level 3-4)

**Exercise 6.8 — What Is the Real Problem?**
A power supply designer measures the Bode plot of a Buck converter and sees PM = 50°. The product ships. Six months later, field failures occur: output oscillation under heavy load.

(a) Explain how component aging (capacitor ESR increase, inductor saturation) could reduce the phase margin from the original measurement.
(b) The engineer’s first instinct is to add more phase margin. A more experienced colleague says: "First, identify *which* component drifted." Why is root-cause identification more important than just increasing margin?
(c) Propose a design guideline that accounts for component drift. What minimum PM would you specify?

**Exercise 6.9 — Mechanism vs. Procedure**
A student plots the Bode diagram of $G(s) = 10/[(s+1)(s+10)]$ and measures GM = 20dB and PM = 65°. When asked "why is this system stable?", the student says: "Because GM > 0 and PM > 0."

(a) This answer is *correct* but shows no understanding. Restate the answer in terms of *what the signals are doing* at the gain crossover frequency.
(b) At $\omega_{gc}$, what is the physical meaning of PM = 65°? (Hint: how much additional phase lag can the system tolerate before the feedback signal aligns destructively with the input?)
(c) If you add a pure time delay of $T$ seconds, how much delay can the system tolerate before instability? Express your answer in terms of PM and $\omega_{gc}$.

---

## What Comes Next

**Chapter 7: Nyquist Stability Criterion** — The Bode plot reveals gain and phase margins, but it cannot handle open-loop unstable plants or long time delays. The Nyquist criterion provides a complete, rigorous stability test that works where Bode analysis fails. It answers the question: *given the full frequency response, exactly how many closed-loop poles are in the right half-plane?*

**Chapter 8: Frequency-Domain Controller Design** — Once you can read the Bode plot, the next step is to *reshape* it. Lead, lag, and PID compensators systematically modify the frequency response to meet stability and performance specifications.

---

## References
