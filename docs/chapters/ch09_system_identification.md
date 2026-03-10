# Chapter 9: System Identification — From Measured Data to Mathematical Models

---

## 🎯 Learning Objectives and Outcomes

### Chapter Objectives

This chapter teaches how to obtain mathematical models from measured input-output data — the essential bridge between first-principles modeling (Chapter 2) and controller design (Chapters 3–8, 10–16). In practice, most real systems cannot be modeled purely from physics; system identification provides the tools to build, validate, and refine models from experiments.

### Prerequisites
- Chapter 2: Mathematical Modeling (first-principles approach)
- Chapter 3: Transfer Functions and Laplace Transform
- Chapter 6: Frequency Response (Bode plots)
- Chapter 8: Frequency-Domain Design (gain/phase margins)
- Linear algebra basics (matrix multiplication, least squares concept)

---

## Learning Outcomes

After completing this chapter, students will be able to:

| Bloom's Level | Outcome |
|---------------|---------|
| **Remember** | List types of identification methods (time-domain, frequency-domain, parametric) and their assumptions |
| **Understand** | Explain why first-principles models are often insufficient and when identification is needed |
| **Apply** | Extract transfer function parameters from step response and Bode data |
| **Analyze** | Evaluate model quality using residual analysis, cross-validation, and information criteria |
| **Evaluate** | Choose appropriate identification method and input signal for a given system |
| **Create** | Design a complete identification experiment and derive a validated model suitable for controller design |

---

## Why This Chapter Matters

> **The Real Engineering Problem:** You've studied modeling (Chapter 2) and learned to derive transfer functions from physics. You've spent Chapters 3–8 learning to design controllers for those transfer functions. But now you face a real motor on your lab bench. You know it's *approximately* a second-order system — but what are the actual values of $J$, $B$, $K_t$, $K_e$, and $L_a$? The datasheet gives nominal values, but your specific motor, with its specific load and specific friction, is different. You could spend days measuring each parameter individually... or you could apply a known input, record the output, and let the data tell you the transfer function directly.
>
> **This is system identification** — the science of building mathematical models from measured data. It is the skill that transforms textbook control theory into working controllers on real hardware.

### The Gap This Chapter Fills

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                  THE MODEL GAP IN CONTROL ENGINEERING                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   Chapter 2 taught:                                                        │
│   ────────────────                                                         │
│   Physics → Equations → Transfer Function                                  │
│   F = ma, V = IR, Q = CΔT → G(s) = K/(τs+1)                             │
│                                                                             │
│   But in practice:                                                         │
│   ───────────────                                                          │
│   • Parameters are unknown or uncertain (friction, thermal resistance)     │
│   • Some dynamics are too complex to model from first principles           │
│   • The real system has nonlinearities, delays, and parasitic effects      │
│   • The datasheet values are nominal, not YOUR specific hardware           │
│                                                                             │
│   Chapters 3–8 taught:                                                     │
│   ────────────────────                                                     │
│   Given G(s), design C(s) for stability and performance                    │
│                                                                             │
│   THE GAP: How do you GET G(s) from a real system?                        │
│                                                                             │
│   ┌─────────────────┐      ┌────────────────────┐     ┌────────────────┐  │
│   │ First-principles │      │  SYSTEM             │     │ Controller     │  │
│   │ model (Ch 2)     │─ ─ ─▶│  IDENTIFICATION    │────▶│ Design         │  │
│   │ (approximate)    │      │  (THIS CHAPTER)     │     │ (Ch 3–8, 10+) │  │
│   └─────────────────┘      └────────────────────┘     └────────────────┘  │
│           │                         ▲                                       │
│           │    Experiment design     │                                      │
│           └─────────── + ──────────┘                                      │
│                  Measured data                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### ⚠️ Stop and Think

Before reading further, consider: Why can't you just use the first-principles model from Chapter 2 and skip identification entirely?

*Think about this for 30 seconds before continuing...*

**Answer:** First-principles models give the *structure* (e.g., "this is a second-order system with one zero"), but identification gives the *parameters* (the actual numerical values of $K$, $\tau$, $\zeta$). In practice, you often need both: physics tells you what to look for; data tells you what you've got.

---

## Signal Dictionary — DC Motor Identification Experiment

This chapter's running example is identifying the transfer function of a DC motor from experimental data.

```
══════════════════════════════════════════════════════════════════════════════
 SIGNAL DICTIONARY — DC Motor System Identification
══════════════════════════════════════════════════════════════════════════════

 Signal  │ Name              │ Unit   │ Physical Meaning                  │ Hardware
 ────────┼───────────────────┼────────┼───────────────────────────────────┼──────────────────────
 u(t)    │ Excitation input  │ V      │ Voltage applied to motor          │ DAC → power amplifier
         │                   │        │ armature for identification       │ → motor terminals
 ────────┼───────────────────┼────────┼───────────────────────────────────┼──────────────────────
 y(t)    │ Measured output   │ rad/s  │ Motor shaft angular velocity      │ Encoder → pulse counter
         │                   │        │                                   │ → µC timer capture
 ────────┼───────────────────┼────────┼───────────────────────────────────┼──────────────────────
 n(t)    │ Measurement noise │ rad/s  │ Encoder quantization + vibration  │ Inherent in sensor
 ────────┼───────────────────┼────────┼───────────────────────────────────┼──────────────────────
 ŷ(t)    │ Model prediction  │ rad/s  │ What our identified model         │ Computed in software
         │                   │        │ predicts the output should be     │
 ────────┼───────────────────┼────────┼───────────────────────────────────┼──────────────────────
 ε(t)    │ Residual          │ rad/s  │ Prediction error: y(t) − ŷ(t)    │ Computed in software
         │                   │        │ Should be white noise if model    │
         │                   │        │ captures all dynamics             │
══════════════════════════════════════════════════════════════════════════════
```

---

## 9.1 The Two Paths to a Model

### 9.1.1 First-Principles vs. Data-Driven

In Chapter 2, we derived models from physical laws:

$$\text{Newton: } J\dot{\omega} = K_t i - B\omega \quad \Rightarrow \quad G(s) = \frac{K_t/J}{s + B/J}$$

This is the **first-principles** (or **white-box**) approach. It requires:
- Deep knowledge of the physics
- Values for every parameter ($J$, $B$, $K_t$, ...)
- Assumptions about linearity, lumped parameters, etc.

The **data-driven** (or **black-box**) approach is fundamentally different:

$$u(t) \xrightarrow{\text{apply to system}} y(t) \xrightarrow{\text{fit model}} \hat{G}(s)$$

| Approach | Needs | Gives | Weakness |
|----------|-------|-------|----------|
| First-principles (Ch 2) | Physics knowledge, parameter values | Model with physical insight | Parameters may be wrong |
| Data-driven (this chapter) | Measured input-output data | Numerically accurate model | May lack physical insight |
| **Grey-box** (best practice) | Physics structure + measured data | Accurate model WITH physical insight | Requires both skills |

### 9.1.2 When Is Identification Necessary?

> **Rule of thumb:** Use first-principles when you *can*, identification when you *must*, and grey-box when you're *wise*.

| Situation | Approach | Why |
|-----------|----------|-----|
| Simple well-understood physics (RC circuit, mass-spring) | First-principles | Parameters from component values |
| Complex physics (turbulent flow, biological system) | Black-box identification | Physics too complex to derive |
| Known structure, unknown parameters (motor with unknown friction) | Grey-box identification | Physics gives structure; data gives numbers |
| Safety-critical (aerospace, medical) | First-principles + validation with data | Regulatory requirements demand physical traceability |
| Rapid prototyping (startup, competition) | Black-box identification | Speed matters more than insight |

### 9.1.3 The Three-Level Model Revisited

Recall from §2.8.7 the three levels of models:

| Level | Model Type | Role |
|-------|-----------|------|
| Level 1 | Mental/conceptual | Engineer's intuition about system behavior |
| Level 2 | Design model | The transfer function used for controller design |
| Level 3 | Simulation model | High-fidelity model for verification |

**System identification primarily targets Level 2** — giving us the design model (transfer function) that we will use in Chapters 10–16 for controller synthesis. But the identified model also serves as a reality check on Level 1 (does the system behave as we expected?) and feeds into Level 3 (providing numerical parameters for simulation).

---

## 9.2 Time-Domain Identification

### 9.2.1 Step Response Method — First-Order Systems

The simplest identification technique: apply a step input and measure the output.

**The mechanism:** A first-order system $G(s) = \frac{K}{\tau s + 1}$ has a step response:

$$y(t) = K\left(1 - e^{-t/\tau}\right)$$

From the measured step response, we extract:
- **DC gain $K$:** The final (steady-state) value: $K = y(\infty) / u_{\text{step}}$
- **Time constant $\tau$:** The time at which $y(t)$ reaches 63.2% of its final value

```
┌──────────────────────────────────────────────────────────────────────┐
│  Step Response of First-Order System                                 │
│                                                                      │
│  y(t)                                                                │
│   K ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ────────────               │
│   │                            ╱───────────                          │
│   │                        ╱──╱                                      │
│   │                     ╱─╱                                          │
│  0.632K ─ ─ ─ ─ ─ ─╱─ ─ ─ ─ ─ ─ ─ ─                               │
│   │              ╱╱                                                  │
│   │           ╱╱                                                     │
│   │        ╱╱                                                        │
│   │     ╱╱                                                           │
│   │  ╱╱                                                              │
│   ╱╱─────┼──────────────────────────────────────▶ t                  │
│   0     τ                                                            │
│                                                                      │
│  Read: K = final value, τ = time to reach 0.632K                    │
└──────────────────────────────────────────────────────────────────────┘
```

**CppPlot implementation:**

```cpp
#include <cppplot/control/control.hpp>
using namespace cppplot;
using namespace cppplot::control;
using namespace cppplot::control::sysid;

int main() {
    // Measured step response data (from experiment)
    std::vector<double> t_meas = {0, 0.1, 0.2, 0.5, 1.0, 2.0, 3.0, 5.0, 8.0, 10.0};
    std::vector<double> y_meas = {0, 0.18, 0.33, 0.63, 0.86, 0.98, 1.0, 1.0, 1.0, 1.0};
    
    // One-line identification: extracts K, τ and refines with least squares
    auto result = id_step_first_order(t_meas, y_meas, 1.0);
    print_step_id(result);  // K ≈ 1.0, τ ≈ 0.5, FIT ≈ 99%
    
    // Compare model prediction with measured data
    figure();
    plot(t_meas, y_meas, "ro", "Measured data");
    step(result.G, 10.0);   // Identified model prediction
    title("Step Response ID: K=" + std::to_string(result.K)
          + ", τ=" + std::to_string(result.tau)
          + " (FIT=" + std::to_string((int)result.fit_percent) + "%)");
    xlabel("Time [s]");
    ylabel("Output [rad/s]");
    legend();
    savefig("ch09_step_id_first_order.svg");
}
```

### 9.2.2 Step Response Method — Second-Order Systems

For underdamped second-order systems $G(s) = \frac{K\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}$, the step response contains richer information:

**Extractable parameters:**

| Measurement | Formula | Physical meaning |
|-------------|---------|------------------|
| DC gain $K$ | $y_{ss} / u_{\text{step}}$ | Steady-state amplification |
| Overshoot $M_p$ | $\frac{y_{peak} - y_{ss}}{y_{ss}}$ | Excess above final value |
| Damping ratio $\zeta$ | $\zeta = \frac{-\ln(M_p)}{\sqrt{\pi^2 + \ln^2(M_p)}}$ | How quickly oscillations decay |
| Peak time $t_p$ | Measured from response | Time of first overshoot |
| Natural frequency $\omega_n$ | $\omega_n = \frac{\pi}{t_p\sqrt{1-\zeta^2}}$ | Oscillation frequency |

### ⚠️ Stop and Think

You measure a step response with 25% overshoot and peak time of 0.2 s. What are $\zeta$ and $\omega_n$?

*Work it out before checking...*

**Solution:**
- $M_p = 0.25 \Rightarrow \zeta = \frac{-\ln(0.25)}{\sqrt{\pi^2 + \ln^2(0.25)}} = \frac{1.386}{\sqrt{9.87 + 1.92}} = \frac{1.386}{3.44} \approx 0.40$
- $\omega_n = \frac{\pi}{0.2\sqrt{1 - 0.16}} = \frac{\pi}{0.183} \approx 17.1$ rad/s

### 9.2.3 Limitations of Step Response Identification

| Limitation | Explanation | Mitigation |
|-----------|-------------|------------|
| Noise sensitivity | Real step responses are noisy; reading $t_p$ and $M_p$ is imprecise | Average multiple runs; use least-squares fit |
| Assumes known order | You must decide: first-order? Second-order? | Use physical insight (Ch 2) to choose structure |
| Only DC and dominant dynamics | Cannot identify high-frequency poles/zeros | Use frequency-domain methods (§9.3) for full bandwidth |
| Requires step input feasible | Some systems cannot tolerate a step (e.g., chemical reactor) | Use PRBS or chirp inputs (§9.5) |

---

## 9.3 Frequency-Domain Identification

### 9.3.1 The Idea: Bode Plot from Measured Data

In Chapter 6, you plotted Bode diagrams from a *known* transfer function. Now we reverse the process: **measure** the Bode diagram and **extract** the transfer function.

**The mechanism:** Apply a sinusoidal input $u(t) = A\sin(\omega t)$ at various frequencies. At each frequency, measure:
- The output amplitude $B$ → magnitude: $|G(j\omega)| = B/A$
- The phase shift $\phi$ → phase: $\angle G(j\omega) = \phi$

Repeat for 20–50 frequencies spanning the bandwidth of interest.

```
┌──────────────────────────────────────────────────────────────────────┐
│  Frequency-Domain Identification Principle                           │
│                                                                      │
│               ω₁                                                     │
│  u(t) = A sin(ω₁t) ──▶ [SYSTEM] ──▶ y(t) = B₁ sin(ω₁t + φ₁)     │
│                                                                      │
│               ω₂                                                     │
│  u(t) = A sin(ω₂t) ──▶ [SYSTEM] ──▶ y(t) = B₂ sin(ω₂t + φ₂)     │
│                                                                      │
│               ω₃                                                     │
│  u(t) = A sin(ω₃t) ──▶ [SYSTEM] ──▶ y(t) = B₃ sin(ω₃t + φ₃)     │
│                                                                      │
│  Then: |G(jωₖ)| = Bₖ/A  and  ∠G(jωₖ) = φₖ                        │
│                                                                      │
│  Plot these points → you have the empirical Bode diagram            │
└──────────────────────────────────────────────────────────────────────┘
```

### 9.3.2 From Bode Data to Transfer Function

Once you have the empirical Bode plot (magnitude and phase vs. frequency), you extract the transfer function by **asymptote fitting**:

**Step 1: Determine the system order** from the high-frequency roll-off slope:
- $-20$ dB/dec → one more pole than zeros
- $-40$ dB/dec → two more poles than zeros
- $-60$ dB/dec → three more poles than zeros

**Step 2: Locate corner frequencies** from where the slope changes.

**Step 3: Identify resonant peaks** — a sharp peak indicates an underdamped complex pole pair.

**Step 4: Read DC gain** from the low-frequency magnitude.

**Example:** Measured Bode data shows:
- DC gain = 20 dB (= 10 in linear)
- Slope changes from 0 to −20 dB/dec at $\omega = 5$ rad/s
- Slope changes from −20 to −40 dB/dec at $\omega = 50$ rad/s

This suggests: $G(s) = \frac{10}{(s/5 + 1)(s/50 + 1)} = \frac{2500}{(s+5)(s+50)}$

### 9.3.3 Spectral Analysis and Coherence

For noisy systems, single-frequency sinusoidal testing is slow. **Spectral analysis** uses broadband excitation and Fourier transforms:

$$\hat{G}(j\omega) = \frac{S_{yu}(\omega)}{S_{uu}(\omega)}$$

where $S_{yu}(\omega)$ is the cross-spectral density between output and input, and $S_{uu}(\omega)$ is the input auto-spectral density.

**The coherence function** tells you how trustworthy the estimate is at each frequency:

$$\gamma^2(\omega) = \frac{|S_{yu}(\omega)|^2}{S_{uu}(\omega) \cdot S_{yy}(\omega)}, \quad 0 \leq \gamma^2 \leq 1$$

| Coherence value | Interpretation |
|-----------------|---------------|
| $\gamma^2 \approx 1$ | Output is well-explained by input at this frequency (reliable estimate) |
| $\gamma^2 \ll 1$ | Output contains energy NOT from the input (noise, nonlinearity, external disturbance) |

> **Practical rule:** Trust $\hat{G}(j\omega)$ only where $\gamma^2(\omega) > 0.8$. Low coherence at specific frequencies often reveals *nonlinearities* or *unmeasured disturbances* — a diagnostic signal, not just noise.

### 9.3.4 CppPlot Frequency-Domain Identification Example

```cpp
#include <cppplot/control/control.hpp>
using namespace cppplot;
using namespace cppplot::control;
using namespace cppplot::control::sysid;

int main() {
    // Measured Bode data from experiment
    std::vector<double> freq = {0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200};
    std::vector<double> mag_dB = {20, 20, 19.8, 19, 17, 14, 7, -2, -20, -34, -48};
    std::vector<double> phase_deg = {-1, -2, -6, -11, -22, -45, -72, -105, -152, -169, -176};

    // Automatic frequency-domain identification
    auto result = id_bode(freq, mag_dB, phase_deg, 2);
    
    std::cout << "Identified TF: " << result.G.toString() << std::endl;
    std::cout << "Mag RMS error:  " << result.mag_rms_error << " dB" << std::endl;
    std::cout << "Phase RMS error: " << result.phase_rms_error << " deg" << std::endl;
    
    // Compare measured vs. identified Bode
    figure();
    subplot(2, 1, 1);
    semilogx(freq, mag_dB, "ro", "Measured");
    semilogx(freq, result.mag_dB_model, "b-", "Identified model");
    ylabel("Magnitude [dB]");
    legend();
    title("Frequency-Domain System Identification");
    
    subplot(2, 1, 2);
    semilogx(freq, phase_deg, "ro", "Measured");
    semilogx(freq, result.phase_deg_model, "b-", "Identified model");
    ylabel("Phase [deg]");
    xlabel("Frequency [rad/s]");
    legend();
    savefig("ch09_bode_identification.svg");
}
```

---

## 9.4 Parametric Identification: Least Squares

### 9.4.1 The Core Idea

Step response and Bode fitting are graphical methods — useful but imprecise. **Parametric identification** uses *all* the measured data simultaneously to find the best-fit model in a mathematically optimal sense.

**The mechanism:** We assume a model structure (e.g., "the plant is second-order") and find the parameters that minimize the prediction error.

### 9.4.2 The ARX Model

The most common linear model structure in discrete time:

$$y[k] = a_1 y[k-1] + a_2 y[k-2] + \cdots + a_{n_a} y[k-n_a] + b_1 u[k-1] + b_2 u[k-2] + \cdots + b_{n_b} u[k-n_b] + e[k]$$

This says: the current output depends on past outputs, past inputs, and white noise $e[k]$.

**In matrix form:** $y[k] = \boldsymbol{\varphi}^T[k]\,\boldsymbol{\theta} + e[k]$

where:
- $\boldsymbol{\varphi}[k] = \begin{bmatrix} -y[k-1] & \cdots & -y[k-n_a] & u[k-1] & \cdots & u[k-n_b] \end{bmatrix}^T$ is the **regression vector**
- $\boldsymbol{\theta} = \begin{bmatrix} a_1 & \cdots & a_{n_a} & b_1 & \cdots & b_{n_b} \end{bmatrix}^T$ is the **parameter vector**

### 9.4.3 Batch Least Squares Solution

Stacking $N$ measurements:

$$\mathbf{Y} = \boldsymbol{\Phi}\,\boldsymbol{\theta} + \mathbf{E}$$

where $\mathbf{Y} = \begin{bmatrix} y[n_a+1] \\ y[n_a+2] \\ \vdots \\ y[N] \end{bmatrix}$, $\boldsymbol{\Phi} = \begin{bmatrix} \boldsymbol{\varphi}^T[n_a+1] \\ \boldsymbol{\varphi}^T[n_a+2] \\ \vdots \\ \boldsymbol{\varphi}^T[N] \end{bmatrix}$

The least-squares estimate minimizes $J(\boldsymbol{\theta}) = \sum_{k} \left(y[k] - \boldsymbol{\varphi}^T[k]\boldsymbol{\theta}\right)^2$:

$$\boxed{\hat{\boldsymbol{\theta}} = \left(\boldsymbol{\Phi}^T \boldsymbol{\Phi}\right)^{-1} \boldsymbol{\Phi}^T \mathbf{Y}}$$

> **Physical interpretation:** Least squares finds the parameter values that make the model's prediction error have the smallest total energy. It is the maximum-likelihood estimate when the noise is Gaussian.

### 9.4.4 From ARX Parameters to Transfer Function

Once you have $\hat{\boldsymbol{\theta}} = [a_1, a_2, b_1, b_2]$ (for a second-order ARX model), the discrete-time transfer function is:

$$\hat{G}(z) = \frac{b_1 z^{-1} + b_2 z^{-2}}{1 - a_1 z^{-1} - a_2 z^{-2}}$$

To obtain the continuous-time transfer function for controller design, use the **Tustin (bilinear) transformation** with sampling period $T_s$:

$$s = \frac{2}{T_s} \cdot \frac{z - 1}{z + 1}$$

**CppPlot ARX identification example:**

```cpp
#include <cppplot/control/control.hpp>
using namespace cppplot;
using namespace cppplot::control;
using namespace cppplot::control::sysid;

int main() {
    // True plant: G(s) = 1/(s+1), discretized at Ts = 0.1 s
    TransferFunction G_true({1.0}, {1.0, 1.0});
    double Ts = 0.1;
    auto Hd = c2d_tustin(G_true, Ts);
    
    // Excite with PRBS input
    auto [t, u] = generate_prbs(7, 1.0, Ts, 3);  // 3 periods of PRBS-7
    auto y = dsim(Hd, u);                          // Simulate plant
    
    // Identify ARX(2,2) model
    auto model = id_arx(u, y, 2, 2, 1, Ts);
    print_arx(model);  // Shows difference equation, parameters, AIC/BIC
    
    // Convert to continuous-time transfer function
    auto G_id = arx_to_tf(model);
    std::cout << "Identified: " << G_id.toString() << std::endl;
    std::cout << "True:       " << G_true.toString() << std::endl;
    
    // Automatic order selection using BIC
    auto best = id_arx_auto(u, y, 4, 1, Ts);
    std::cout << "Best order by BIC: ARX(" 
              << best.na << "," << best.nb << ")" << std::endl;
}
```

### 9.4.5 When Least Squares Fails

| Failure mode | Cause | Symptom | Fix |
|-------------|-------|---------|-----|
| Biased estimates | Colored noise (noise is NOT white) | Residuals are correlated | Use IV, ARMAX, or OE methods |
| Ill-conditioned $\Phi^T\Phi$ | Insufficient excitation | Parameters have huge variance | Redesign input signal (§9.5) |
| Wrong model order | $n_a$ or $n_b$ too low/high | Residuals show structure (underfitting) or parameters are insignificant (overfitting) | Use AIC/BIC (§9.6) |
| Nonlinear system | System violates linearity assumption | Model changes with operating point | Linearize around operating point, or use nonlinear ID (Ch 17) |

### 9.4.6 Beyond ARX: A Brief Map

ARX is the starting point. When its assumptions fail, there are more sophisticated structures:

```
┌──────────────────────────────────────────────────────────────────────────┐
│              PARAMETRIC MODEL STRUCTURES — THE LANDSCAPE                 │
├──────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│   Model     │ Equation                          │ When to Use            │
│   ──────────┼─────────────────────────────────  ┼────────────────────── │
│   ARX       │ A(q)y = B(q)u + e                 │ White noise, simplest  │
│   ARMAX     │ A(q)y = B(q)u + C(q)e             │ Colored noise          │
│   OE        │ y = B(q)/F(q) u + e               │ Focus on plant, not    │
│             │                                    │ noise model            │
│   BJ        │ y = B(q)/F(q) u + C(q)/D(q) e    │ Separate plant and     │
│             │                                    │ noise models           │
│   State-    │ x[k+1] = Ax[k] + Bu[k] + w[k]   │ MIMO systems           │
│   Space     │ y[k] = Cx[k] + v[k]              │ (after Ch 10)          │
│                                                                          │
│   Rule of thumb: Start with ARX. If residuals are correlated,           │
│   try ARMAX or OE. Use BJ only if you need accuracy at all frequencies. │
└──────────────────────────────────────────────────────────────────────────┘
```

> **For this course:** ARX with least squares is sufficient for most single-loop control applications. The advanced methods (ARMAX, OE, BJ) are covered in dedicated system identification courses — e.g., Ljung's *System Identification: Theory for the User*.

---

## 9.5 Input Signal Design

### 9.5.1 Why the Input Matters

> **Fundamental principle:** You can only identify dynamics that the input signal *excites*. A constant input teaches you nothing about dynamics. A slow sinusoid teaches you nothing about high-frequency behavior. The quality of your identified model is fundamentally limited by the quality of your experiment.

This is the concept of **persistence of excitation**: the input must have sufficient spectral content to "energize" all the modes you want to identify.

### 9.5.2 Common Excitation Signals

| Signal | Description | Spectrum | Best for |
|--------|-------------|----------|----------|
| **Step** | Sudden jump $u(t) = A \cdot \mathbb{1}(t)$ | All frequencies (but decaying) | Quick rough estimate; first-order systems |
| **PRBS** | Pseudo-Random Binary Sequence: switches between $\pm A$ at random intervals | Approximately flat up to clock frequency | Parametric ID; safe amplitude bounds |
| **Chirp** | Sinusoid with linearly increasing frequency $u(t) = A\sin(\omega_0 t + \frac{\beta}{2}t^2)$ | Sweeps from $\omega_0$ to $\omega_0 + \beta T$ | Frequency-domain ID; controlled bandwidth |
| **Multisine** | Sum of sinusoids at selected frequencies $u(t) = \sum_k A_k \sin(\omega_k t + \phi_k)$ | Energy at chosen frequencies only | Precise Bode measurement; nonlinearity detection |
| **White noise** | Random signal with flat spectrum | Flat everywhere | Maximum information (but unbounded amplitude) |

### 9.5.3 PRBS — The Workhorse of Industrial Identification

A PRBS is a deterministic binary signal that *looks* random. It switches between two levels ($+A$ and $-A$) according to a shift register sequence.

```
┌──────────────────────────────────────────────────────────────────────┐
│  PRBS Signal Example (7-bit, period = 127 clock cycles)              │
│                                                                      │
│  u(t)                                                                │
│  +A ─┐  ┌──┐  ┌─┐  ┌────┐    ┌─┐  ┌──┐                            │
│      │  │  │  │ │  │    │    │ │  │  │                              │
│      │  │  │  │ │  │    │    │ │  │  │                              │
│  -A  └──┘  └──┘ └──┘    └────┘ └──┘  └──                           │
│                                                                      │
│  Properties:                                                         │
│  • Bounded amplitude: ±A (safe for hardware)                        │
│  • Approximately white spectrum up to f_clock / 2                    │
│  • Deterministic: repeatable experiments                             │
│  • Persistence of excitation: guaranteed for order ≤ (2^n − 1)/2    │
└──────────────────────────────────────────────────────────────────────┘
```

**Design choices:**
- **Amplitude $A$:** Large enough for good signal-to-noise ratio (SNR), small enough to stay in the linear regime
- **Clock frequency $f_{clk}$:** At least 5–10× the expected system bandwidth
- **Sequence length $2^n - 1$:** Longer = better frequency resolution. $n = 7$ (127 samples) to $n = 10$ (1023 samples) covers most cases

### 9.5.4 Chirp Signal — Frequency Sweep

The chirp (or swept sine) is ideal when you want a Bode plot:

$$u(t) = A\sin\left(2\pi\left(f_0 t + \frac{f_1 - f_0}{2T}t^2\right)\right)$$

sweeps from frequency $f_0$ to $f_1$ over duration $T$.

```cpp
// CppPlot chirp signal generation
#include <cppplot/control/control.hpp>
using namespace cppplot;
using namespace cppplot::control::sysid;

int main() {
    double f0 = 0.1;    // Start frequency [Hz]
    double f1 = 100.0;  // End frequency [Hz]
    double T = 60.0;     // Sweep duration [s]
    double A = 1.0;      // Amplitude [V]
    double Ts = 0.001;   // Sampling period [s]
    
    // One-liner: generates time vector and chirp signal
    auto [t, u] = generate_chirp(f0, f1, T, Ts, A);
    
    figure();
    plot(t, u);
    title("Chirp Signal: " + std::to_string(f0) + " → " + std::to_string(f1) + " Hz");
    xlabel("Time [s]");
    ylabel("Amplitude [V]");
    savefig("ch09_chirp_signal.svg");
    
    // Other input signals available:
    // auto [t_p, u_p] = generate_prbs(7, 1.0, 0.01);      // PRBS-7
    // auto [t_m, u_m] = generate_multisine({1,5,10,50}, 10.0); // Multisine
    // auto [t_n, u_n] = generate_white_noise(1000, 0.01);  // White noise
}
```

### 9.5.5 Choosing the Right Input

```
┌──────────────────────────────────────────────────────────────────────────┐
│                  INPUT SIGNAL SELECTION GUIDE                             │
├──────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│   What do you know about the system?                                     │
│                                                                          │
│   "Almost nothing"                                                       │
│       └──▶ Start with a step response (§9.2)                            │
│           └──▶ Get rough K, τ or ζ, ωn                                   │
│               └──▶ Use these to design PRBS or chirp bandwidth           │
│                                                                          │
│   "It's approximately first/second-order"                                │
│       └──▶ PRBS with f_clk = 10 × estimated bandwidth                  │
│           └──▶ ARX identification (§9.4)                                 │
│                                                                          │
│   "I need a precise Bode plot"                                           │
│       └──▶ Chirp sweep covering 0.1ω_BW to 10ω_BW                      │
│           └──▶ Or multisine at specific frequencies                      │
│               └──▶ Spectral analysis (§9.3.3)                           │
│                                                                          │
│   "System is fragile / can't tolerate large inputs"                      │
│       └──▶ Low-amplitude PRBS or slow chirp                             │
│           └──▶ Accept lower SNR → need longer experiment                 │
│                                                                          │
│   "System operates in closed-loop (can't open the loop)"                │
│       └──▶ Add PRBS to setpoint (indirect identification)               │
│           └──▶ Use closed-loop identification methods                    │
│               └──▶ (Advanced: joint input-output method)                 │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## 9.6 Model Validation

### 9.6.1 The Cardinal Rule

> **An identified model is worthless until validated.** Fitting data is easy — any model with enough parameters can fit ANY data set perfectly (overfitting). The question is: does the model *predict* data it has never seen?

### 9.6.2 The Validation Workflow

```
┌──────────────────────────────────────────────────────────────────────────┐
│                     MODEL VALIDATION WORKFLOW                             │
├──────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│   Measured data                                                          │
│       │                                                                  │
│       ├──▶ Training set (60–80%)  ──▶  Fit model parameters             │
│       │                                    │                             │
│       └──▶ Validation set (20–40%) ──▶  Test model predictions          │
│                                            │                             │
│                                     ┌──────┴──────┐                      │
│                                     │  Validation  │                     │
│                                     │   Tests      │                     │
│                                     └──────┬──────┘                      │
│                                            │                             │
│            ┌───────────────┬───────────────┼───────────────┐             │
│            ▼               ▼               ▼               ▼             │
│       Residual        Cross-         Information     Physical           │
│       analysis       validation      criteria       sanity check        │
│       (§9.6.3)       (§9.6.4)       (§9.6.5)       (§9.6.6)           │
└──────────────────────────────────────────────────────────────────────────┘
```

### 9.6.3 Residual Analysis

The residual (prediction error) is: $\varepsilon[k] = y[k] - \hat{y}[k]$

**If the model is good**, the residuals should be:
1. **White** (uncorrelated): no remaining dynamics to capture
2. **Zero-mean**: no systematic bias
3. **Uncorrelated with the input**: the model has captured all the input-output relationship

**Tests:**
- **Autocorrelation of residuals** $R_{\varepsilon\varepsilon}[\tau]$: should be a delta function (impulse at $\tau = 0$, zero elsewhere)
- **Cross-correlation of residuals with input** $R_{\varepsilon u}[\tau]$: should be zero for all $\tau$

```
┌──────────────────────────────────────────────────────────────────────┐
│  Residual Analysis — Good vs. Bad Model                              │
│                                                                      │
│  GOOD MODEL (residuals are white noise):                             │
│                                                                      │
│  R_εε[τ]                                                             │
│    │     ╋                                                           │
│    │   ┄┄╋┄┄ 95% confidence bounds                                  │
│    │─ ─ ─╋─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─                               │
│    │     ╋                                                           │
│    └─────╋────────────────────────────▶ τ                            │
│       Impulse at τ=0, zero elsewhere ✓                               │
│                                                                      │
│  BAD MODEL (residuals have structure):                               │
│                                                                      │
│  R_εε[τ]                                                             │
│    │     ╋                                                           │
│    │    ╱╋╲                                                          │
│    │  ╱  ╋  ╲    ╱╲         Decaying oscillation ✗                  │
│    │─╱───╋────╲╱────╲───────────────                                │
│    │     ╋                                                           │
│    └─────╋────────────────────────────▶ τ                            │
│       Residuals are correlated → model is missing dynamics           │
└──────────────────────────────────────────────────────────────────────┘
```

### 9.6.4 Cross-Validation

The most intuitive validation test:

1. Split data into **training set** and **validation set**
2. Identify model parameters using ONLY the training set
3. Simulate the model with the validation set's input
4. Compare model prediction $\hat{y}$ with actual validation output $y$

**Fit metric:**

$$\text{FIT} = 100\% \times \left(1 - \frac{\|\mathbf{y} - \hat{\mathbf{y}}\|}{\|\mathbf{y} - \bar{y}\|}\right)$$

| FIT value | Interpretation |
|-----------|---------------|
| > 90% | Excellent — model captures almost all dynamics |
| 70–90% | Good — suitable for control design with robustness margins |
| 50–70% | Mediocre — consider higher model order or different structure |
| < 50% | Poor — fundamental problem (wrong structure, nonlinearity, etc.) |

### 9.6.5 Information Criteria: AIC and BIC

How do you choose the model order ($n_a$, $n_b$)? **Information criteria** balance fit quality against model complexity:

**Akaike Information Criterion (AIC):**
$$\text{AIC} = N \ln(\hat{\sigma}^2_\varepsilon) + 2p$$

**Bayesian Information Criterion (BIC):**
$$\text{BIC} = N \ln(\hat{\sigma}^2_\varepsilon) + p \ln(N)$$

where $N$ = number of data points, $p$ = number of parameters, $\hat{\sigma}^2_\varepsilon$ = residual variance.

> **Mechanism:** Both criteria penalize model complexity. Adding a parameter reduces $\hat{\sigma}^2_\varepsilon$ (better fit) but increases the penalty term. The minimum AIC/BIC identifies the model order that best balances fit and parsimony.

**Practical procedure:**
1. Identify models for orders $n = 1, 2, 3, \ldots, n_{max}$
2. Compute AIC and BIC for each
3. Choose the order where AIC/BIC is minimized

```
┌──────────────────────────────────────────────────────────────────────┐
│  AIC/BIC vs. Model Order                                             │
│                                                                      │
│  AIC/BIC                                                             │
│    │  ╲                                                              │
│    │    ╲                                                            │
│    │      ╲        **                                                │
│    │        ╲    **  *                                               │
│    │         ╲ **      * *                                           │
│    │          ✱             * * *    ← overfitting region            │
│    │        **  ╲                                                    │
│    │      **      ╲                                                  │
│    │                 ╲                                                │
│    │   underfitting    optimal    overfitting                        │
│    └──────────┼─────────┼────────┼──────────▶ Model order            │
│               1    n_opt = 2     5                                    │
│                                                                      │
│  Choose the order at the minimum (n_opt)                             │
└──────────────────────────────────────────────────────────────────────┘
```

**CppPlot model validation example:**

```cpp
#include <cppplot/control/control.hpp>
using namespace cppplot;
using namespace cppplot::control;
using namespace cppplot::control::sysid;

int main() {
    // Simulate a known plant with PRBS excitation
    TransferFunction G_true({1.0}, {1.0, 1.0});
    double Ts = 0.1;
    auto Hd = c2d_tustin(G_true, Ts);
    auto [t, u] = generate_prbs(7, 1.0, Ts, 3);
    auto y = dsim(Hd, u);
    
    // Split 70/30 into training and validation sets
    int N = static_cast<int>(u.size());
    int N_tr = static_cast<int>(0.7 * N);
    std::vector<double> u_tr(u.begin(), u.begin()+N_tr);
    std::vector<double> y_tr(y.begin(), y.begin()+N_tr);
    std::vector<double> u_val(u.begin()+N_tr, u.end());
    std::vector<double> y_val(y.begin()+N_tr, y.end());
    
    // Identify and validate
    auto model = id_arx(u_tr, y_tr, 2, 2, 1, Ts);
    auto val = validate_model(model, u_val, y_val);
    print_validation(val);
    // Output: FIT ≈ 99%, residuals white: YES ✓
    
    // AIC/BIC comparison for order selection
    for (int na = 1; na <= 4; ++na)
      for (int nb = 1; nb <= 4; ++nb) {
        auto m = id_arx(u_tr, y_tr, na, nb, 1, Ts);
        auto ic = compute_aic_bic(m.sigma2, m.num_params(), m.N);
        std::cout << "ARX(" << na << "," << nb << ") "
                  << "AIC=" << ic.AIC << " BIC=" << ic.BIC << std::endl;
      }
}
```

### 9.6.6 Physical Sanity Check

The most important validation is often the simplest: **does the model make physical sense?**

| Check | What to verify | Red flag |
|-------|---------------|----------|
| DC gain sign | If increasing input should increase output, is $K > 0$? | Negative gain for a motor |
| Stability | Are all poles in LHP (continuous) or inside unit circle (discrete)? | Unstable pole for a clearly stable system |
| Time constants | Do they match your physical intuition? | τ = 0.001 s for a thermal system (should be minutes) |
| Gain magnitude | Is the gain physically reasonable? | K = 10⁶ for a system that should have unity gain |
| Bandwidth | Does it match the observed speed of response? | 100 Hz bandwidth for a system that visibly takes seconds to respond |

> **Golden rule:** If your identified model says the motor should spin at 10⁶ rad/s or your thermal system should have a 1-ms time constant, your model is wrong — no matter how good the AIC score is.

---

## 9.7 From Identification to Control Design

### 9.7.1 The Identified Model in the Design Workflow

System identification produces a **design model** (Level 2 from §2.8.7). This model now enters the control design workflow from Chapters 3–8:

```
┌──────────────────────────────────────────────────────────────────────────┐
│              FROM IDENTIFICATION TO CONTROLLER DESIGN                    │
├──────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│   Experiment                                                             │
│      │                                                                   │
│      ├──▶ u(t), y(t) data                                               │
│      │                                                                   │
│   Identification (§9.2–§9.4)                                             │
│      │                                                                   │
│      ├──▶ Ĝ(s) = identified transfer function                           │
│      │                                                                   │
│   Validation (§9.6)                                                      │
│      │                                                                   │
│      ├──▶ "Model FIT = 85%, residuals white, physics check OK"          │
│      │                                                                   │
│   Controller Design (Ch 3–8)                                             │
│      │                                                                   │
│      ├──▶ Design C(s) using Ĝ(s) as the plant model                    │
│      │     • Root locus (Ch 5) for pole placement                       │
│      │     • Bode design (Ch 8) for gain/phase margins                  │
│      │                                                                   │
│   Implementation & Testing                                               │
│      │                                                                   │
│      └──▶ Deploy controller → test on real system → iterate              │
│                                                                          │
│   If performance is inadequate:                                          │
│      └──▶ Re-identify with better input signal, or                      │
│           Increase model order, or                                       │
│           Add robustness margins (Ch 16)                                 │
└──────────────────────────────────────────────────────────────────────────┘
```

### 9.7.2 Model Uncertainty and Robust Control

**Critical insight:** Every identified model has uncertainty. The question is not "Is my model perfect?" (it isn't) but "How wrong might it be, and can my controller tolerate that uncertainty?"

The identification process naturally provides uncertainty information:

| Information from identification | How it feeds into robust design (Ch 16) |
|------|------|
| Parameter confidence intervals ($\hat{\theta} \pm \Delta\theta$) | Defines parametric uncertainty set |
| Coherence function $\gamma^2(\omega)$ | Low coherence frequencies → high model uncertainty |
| Cross-validation FIT % | Overall model reliability metric |
| Frequency range of reliable data | Trust model only within identified bandwidth |

$$\text{True plant: } G(s) = \hat{G}(s)\big(1 + W_2(s)\Delta(s)\big), \quad \|\Delta\|_\infty \leq 1$$

where $W_2(s)$ is the **multiplicative uncertainty weight** — large at frequencies where identification is unreliable (low coherence) and small where it's reliable.

> **The bridge to Chapter 16:** System identification doesn't just give you a model — it tells you how much to *distrust* the model. This distrust is precisely what the robust control framework needs.

### 9.7.3 Iterative Identification and Control

In practice, identification and control design form an iterative loop:

1. **Identify** an initial model from open-loop data
2. **Design** a conservative controller (large gain/phase margins)
3. **Deploy** the controller on the real system
4. **Re-identify** — now with the system in closed-loop, you can often get better data (the controller stabilizes the system, allowing larger excitation)
5. **Refine** the controller based on the improved model
6. **Repeat** until performance meets specifications

> **Why iterative?** The first identification may be rough (noisy data, limited excitation). But once a stabilizing controller is in place, you can push the system harder, collect better data, build a better model, and design a better controller. Each iteration improves both the model and the controller.

---

## 9.8 When Identification Fails: Anti-Examples

### 9.8.1 The Overfitting Trap

**Scenario:** A student collects 200 data points from a DC motor. They fit an ARX model with $n_a = 10$, $n_b = 10$ (20 free parameters). The model fits the training data with 99.5% FIT.

**What goes wrong:** When tested on new data, the FIT drops to 35%. The model memorized noise patterns, not system dynamics.

**The lesson:** More parameters ≠ better model. AIC/BIC would have shown the optimum at $n_a = 2$, $n_b = 2$.

> **Physical mechanism:** A DC motor is a second-order system (electrical + mechanical time constants). An ARX(10,10) model has 20 free parameters for a system with 4 physical parameters. The extra 16 parameters fit noise, not dynamics.

### 9.8.2 Insufficient Excitation

**Scenario:** An engineer tries to identify a servomotor by commanding a constant velocity. They collect beautiful steady-state data: flat velocity, constant current.

**What goes wrong:** Least squares returns meaningful $b_0$ (DC gain) but wildly uncertain $a_1$, $a_2$ (dynamics). The parameters change unpredictably between experiments.

**The lesson:** A constant input has zero spectral content at all frequencies except DC. You can only identify dynamics if you excite them.

> **Persistence of excitation violated:** The input must be "sufficiently rich" — containing at least $n_a + n_b$ distinct frequencies for an ARX($n_a$, $n_b$) model.

### 9.8.3 Feedback Corruption

**Scenario:** A student tries to identify a motor while the PID controller is running. They use the motor command $u[k]$ as the "input" and the motor speed $y[k]$ as the "output."

**What goes wrong:** The identified model doesn't match the open-loop plant at all. The estimated damping is much higher than it should be (the controller is adding artificial damping that the identification attributes to the plant).

**The lesson:** In closed-loop, $u[k]$ depends on past $y[k]$ through the controller. This violates the assumption that input and noise are uncorrelated, causing biased least-squares estimates.

**Fix:** Either identify in open-loop, or use specialized closed-loop identification methods (e.g., two-stage method, joint input-output method).

### 9.8.4 Operating Point Dependence

**Scenario:** An engineer identifies a DC motor at 10% load and designs a controller. The controller works perfectly at 10% load but oscillates violently at 90% load.

**What goes wrong:** The motor's dynamics change with operating point (friction is nonlinear, back-EMF changes with speed). The model identified at one operating point is invalid at another.

**The lesson:** Always identify at or near the **intended operating point**. If the system operates across a wide range, identify at multiple operating points and use gain scheduling or robust control.

---

## 9.9 Exercises

### Exercise 9.1 — Step Response Parameter Extraction (Level 1)

A first-order system is subjected to a unit step input. The measured step response data is:

| $t$ [s] | 0 | 0.5 | 1.0 | 1.5 | 2.0 | 3.0 | 5.0 | 10.0 |
|---------|---|-----|-----|-----|-----|-----|-----|------|
| $y(t)$  | 0 | 1.57 | 2.53 | 3.10 | 3.42 | 3.80 | 3.97 | 4.00 |

**(a)** Determine the DC gain $K$.

**(b)** Determine the time constant $\tau$ (interpolate if necessary).

**(c)** Write the identified transfer function $\hat{G}(s)$.

**(d)** Verify with CppPlot: compare the model step response with the measured data.

---

### Exercise 9.2 — Second-Order Step Response Identification (Level 1)

A measured step response shows:
- Final value: $y_{ss} = 2.0$
- First peak: $y_{peak} = 2.6$ at $t_p = 0.05$ s
- Input step magnitude: $u_0 = 1.0$

**(a)** Calculate $K$, $M_p$, $\zeta$, and $\omega_n$.

**(b)** Write the identified transfer function $\hat{G}(s) = \frac{K\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}$.

**(c)** From the identified $\hat{G}(s)$, predict the settling time $t_s \approx 4/(\zeta\omega_n)$. Does this match the measured response?

---

### Exercise 9.3 — Bode Plot Identification (Level 2)

The following Bode magnitude data is measured from a system:

| $\omega$ [rad/s] | 0.1 | 0.5 | 1 | 5 | 10 | 20 | 50 | 100 | 500 |
|---------|-----|-----|---|---|----|----|----|----|-----|
| $|G|$ [dB] | 40 | 40 | 39.5 | 34 | 26 | 14 | -6 | -26 | -66 |

**(a)** Determine the system order from the high-frequency slope.

**(b)** Identify the DC gain and corner frequencies.

**(c)** Propose a transfer function $\hat{G}(s)$ and verify by computing $20\log_{10}|\hat{G}(j\omega)|$ at the measured frequencies.

**(d)** If the phase at $\omega = 10$ rad/s is measured as $-135°$, is this consistent with your proposed $\hat{G}(s)$? Why or why not?

---

### Exercise 9.4 — Least Squares Identification (Level 2)

A discrete-time system with sampling period $T_s = 0.1$ s is excited with a PRBS input. The following input-output data is collected:

| $k$ | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
|-----|---|---|---|---|---|---|---|---|
| $u[k]$ | 1 | -1 | 1 | 1 | -1 | -1 | 1 | -1 |
| $y[k]$ | 0 | 0.8 | -0.2 | 1.1 | 0.9 | -0.5 | -0.8 | 0.9 |

**(a)** Set up the regression matrices $\boldsymbol{\Phi}$ and $\mathbf{Y}$ for an ARX(1,1) model: $y[k] = a_1 y[k-1] + b_1 u[k-1] + e[k]$.

**(b)** Compute $\hat{\boldsymbol{\theta}} = (\boldsymbol{\Phi}^T\boldsymbol{\Phi})^{-1}\boldsymbol{\Phi}^T\mathbf{Y}$.

**(c)** Write the discrete-time transfer function $\hat{G}(z) = \frac{b_1 z^{-1}}{1 - a_1 z^{-1}}$.

**(d)** Compute the one-step-ahead predictions $\hat{y}[k]$ and the residuals $\varepsilon[k] = y[k] - \hat{y}[k]$.

---

### Exercise 9.5 — Input Signal Design (Level 2)

You need to identify a thermal system (expected bandwidth ~0.01 Hz, i.e., time constant ~100 s).

**(a)** Design a PRBS signal: choose the clock frequency and sequence length. Justify your choices.

**(b)** Design a chirp signal: choose start frequency, end frequency, and sweep duration. Justify.

**(c)** Which would you prefer for this application and why? (Consider experiment duration, equipment limitations, and noise.)

**(d)** The system's heater has a maximum power of 100 W. What is the maximum safe PRBS amplitude? What happens if you exceed the linear range?

---

### Exercise 9.6 — Model Order Selection with AIC (Level 2)

Models of orders 1 through 5 are fit to 500 data points. The residual variances are:

| Order $p$ | $\hat{\sigma}^2_\varepsilon$ | Number of parameters |
|----|------|------|
| 1 | 2.50 | 2 |
| 2 | 0.85 | 4 |
| 3 | 0.80 | 6 |
| 4 | 0.79 | 8 |
| 5 | 0.78 | 10 |

**(a)** Compute AIC for each model order.

**(b)** Which order minimizes AIC?

**(c)** Compute BIC. Does it agree with AIC?

**(d)** The physical system is known to be a DC motor (second-order). Does your AIC/BIC result agree with physics?

---

### Exercise 9.7 — Complete Identification Workflow (Level 3) ⭐

You are given access to a DC motor on a lab bench with:
- A DAC that can output ±5V
- An encoder that measures speed with resolution 0.01 rad/s
- A sampling rate of 1 kHz

Design a complete identification experiment:

**(a)** What input signal do you choose? Specify all parameters (amplitude, frequency range, duration).

**(b)** What model structure and order do you choose? Why?

**(c)** How will you split the data for training and validation?

**(d)** What validation tests will you perform? What thresholds will you use (FIT %, residual whiteness)?

**(e)** If the residuals show a peak in their autocorrelation at lag 5, what does this mean physically? What would you change?

**(f)** The resulting model is $\hat{G}(s) = \frac{42.5}{s^2 + 15.3s + 210}$. Is this physically reasonable for a small DC motor? Check the implied time constants and DC gain.

---

### Exercise 9.8 — What Is the Real Problem? (Level 3–4) ⭐

An engineer identifies a plant model $\hat{G}(s) = \frac{10}{s+5}$ and designs a PI controller that gives PM = 50°. The system works perfectly in the lab. When deployed in the field, the controller oscillates.

**(a)** List at least four possible reasons for the field failure.

**(b)** For each reason, describe what additional data or experiment you would need to diagnose it.

**(c)** One hypothesis is "the plant parameters changed due to temperature." How would you use system identification to test this hypothesis?

**(d)** Another hypothesis is "there's an unmodeled delay in the field installation's communication bus." How would you detect a delay from identification data? (Hint: what does a delay do to the phase in the Bode plot?)

**(e)** The deeper question: was the real problem the PI controller, the model, or the identification experiment? Discuss.

---

### Exercise 9.9 — Closed-Loop Identification Pitfall (Level 4) ⭐⭐

A student identifies a motor in closed-loop (with PID running) using the motor command as "input" and speed as "output."

**(a)** Draw the block diagram showing the feedback loop. Label the signals: reference $r$, error $e$, controller output $u$, plant output $y$, and noise $n$.

**(b)** In closed-loop, is $u[k]$ independent of $n[k-1]$? Why does this matter for least squares?

**(c)** The student computes the ARX model and finds $\hat{G}(s) = \frac{8}{s + 20}$. The open-loop plant actually has $G(s) = \frac{10}{s + 5}$. Explain why the identified damping ($p = 20$) is so much higher than the true damping ($p = 5$). Where did the extra damping come from?

**(d)** Propose a method to identify the true plant from closed-loop data without opening the loop. (Hint: inject a known signal at a specific point in the loop.)

---

## 9.10 Self-Assessment Checklist

Before moving to Chapter 10, verify that you can:

| # | Skill | Check |
|---|-------|-------|
| 1 | Explain why first-principles models alone are usually insufficient for controller design | ☐ |
| 2 | Extract $K$ and $\tau$ from a first-order step response | ☐ |
| 3 | Extract $K$, $\zeta$, and $\omega_n$ from a second-order step response | ☐ |
| 4 | Determine system order and corner frequencies from Bode data | ☐ |
| 5 | Set up and solve the least squares problem for an ARX model | ☐ |
| 6 | Choose an appropriate input signal (step, PRBS, chirp) for an identification experiment | ☐ |
| 7 | Perform residual analysis and interpret autocorrelation plots | ☐ |
| 8 | Use AIC/BIC to select model order | ☐ |
| 9 | Explain what coherence tells you about model reliability at each frequency | ☐ |
| 10 | Perform cross-validation and interpret the FIT metric | ☐ |
| 11 | Explain why closed-loop identification with standard LS gives biased estimates | ☐ |
| 12 | Connect identification uncertainty to robust control design (Ch 16) | ☐ |

---

## What Comes Next

**Chapter 10: State-Space Representation** — With a validated transfer function model in hand, you're ready for the next paradigm shift. Transfer functions describe *input-output* behavior. State-space models describe *internal* behavior — the system's memory, its energy storage elements, its hidden dynamics. This is essential for MIMO systems (where transfer function matrices become unwieldy) and for modern control techniques like pole placement, LQR, and Kalman filtering. State-space also connects naturally to system identification: the state-space subspace methods (N4SID, MOESP) identify MIMO state-space models directly from data — a topic we'll revisit in Chapter 16 when we need uncertainty models for robust design.

---

## References

1. Ljung, L. (1999). *System Identification: Theory for the User*, 2nd ed. Prentice Hall. — The definitive textbook on system identification.
2. Söderström, T. & Stoica, P. (1989). *System Identification*. Prentice Hall. — Rigorous mathematical treatment.
3. Ljung, L. & Glad, T. (1994). *Modeling of Dynamic Systems*. Prentice Hall. — Bridges modeling and identification.
4. Pintelon, R. & Schoukens, J. (2012). *System Identification: A Frequency Domain Approach*, 2nd ed. Wiley. — Comprehensive frequency-domain methods.
5. Åström, K.J. & Murray, R.M. (2021). *Feedback Systems*, 2nd ed. — Chapter 3 covers experimental modeling.
6. Keesman, K.J. (2011). *System Identification: An Introduction*. Springer. — Accessible introduction with practical examples.
