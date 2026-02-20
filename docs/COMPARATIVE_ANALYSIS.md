# Comparative Analysis: CppPlot vs Classic Control Textbooks

## Overview

This document analyzes the content structure of classic control systems textbooks and maps them to CppPlot library capabilities.

---

## 📖 OGATA: Modern Control Engineering (6th Edition)

### Chapter-by-Chapter Analysis

| Chapter | Topic | CppPlot Coverage | Status |
|---------|-------|------------------|--------|
| 1 | Introduction to Control Systems | Basics covered | ✅ |
| 2 | Mathematical Modeling (Laplace) | TransferFunction, Polynomial | ✅ |
| 3 | Mathematical Modeling (Mechanical/Electrical) | StateSpace | ✅ |
| 4 | Transient and Steady-State Response | step(), impulse(), lsim() | ✅ |
| 5 | Root-Locus Analysis | rlocus() | ✅ |
| 6 | Frequency-Response Analysis | bode(), nyquist(), nichols() | ✅ |
| 7 | Control Design by Root-Locus | rlocus(), place() | ✅ |
| 8 | Control Design by Frequency-Response | margin(), bode() | ✅ |
| 9 | PID Controllers | pid(), tunePID() | ✅ |
| 10 | State-Space Analysis | controllability, observability | ✅ |
| 11 | State-Space Design | lqr(), place(), observer | ✅ |
| 12 | Modeling in MATLAB | **CppPlot equivalent** | ✅ |

### Key Ogata Concepts in CppPlot

```cpp
// ===== Chapter 4: Time Response =====
// Ogata focuses heavily on second-order system analysis

// Standard second-order system
double wn = 2.0;   // Natural frequency
double zeta = 0.5; // Damping ratio

TransferFunction G({wn*wn}, {1, 2*zeta*wn, wn*wn});

// Time-domain specifications
auto [t, y] = step(G, 10.0);
auto info = stepinfo(t, y);
// Rise time, settling time, overshoot - as in Ogata Table 4-1

// ===== Chapter 5: Root Locus =====
// Ogata's 7 rules for root locus construction

TransferFunction G_ol({1}, {1, 3, 2, 0});  // Open-loop
figure();
rlocus(G_ol);
// Shows poles moving as K varies from 0 to ∞

// ===== Chapter 6: Frequency Response =====
// Ogata emphasizes Bode plot asymptotic approximations

bode(G);
auto margins = margin(G);
// Gain margin, phase margin as defined in Ogata

// ===== Chapter 9: PID Control =====
// Ogata covers Ziegler-Nichols methods extensively

TransferFunction C = pid(Kp, Ki, Kd);
// Or use tuning rules:
// auto [Kp, Ki, Kd] = zieglerNicholsOpenLoop(K, L, T);
```

---

## 📖 FRANKLIN: Feedback Control of Dynamic Systems (8th Edition)

### Chapter-by-Chapter Analysis

| Chapter | Topic | CppPlot Coverage | Status |
|---------|-------|------------------|--------|
| 1 | Overview and History | Documentation | ✅ |
| 2 | Dynamic Models | TransferFunction, StateSpace | ✅ |
| 3 | Dynamic Response | step(), impulse(), freqresp() | ✅ |
| 4 | First Analysis of Feedback | feedback(), series() | ✅ |
| 5 | Root-Locus Design | rlocus(), place() | ✅ |
| 6 | Frequency-Response Design | bode(), margin(), lead/lag | ✅ |
| 7 | State-Space Design | lqr(), place(), observer | ✅ |
| 8 | Digital Control | c2d(), d2c(), dlqr() | ✅ |
| 9 | Nonlinear Systems | linearize() | ⚠️ Partial |
| 10 | Design Case Studies | Examples available | ✅ |

### Key Franklin Concepts in CppPlot

```cpp
// ===== Chapter 3: Dynamic Response =====
// Franklin emphasizes impulse response and convolution

TransferFunction G({1}, {1, 2, 1});
auto [t_imp, y_imp] = impulse(G, 10.0);
auto [t_step, y_step] = step(G, 10.0);

// Relationship: step response = integral of impulse response

// ===== Chapter 4: Feedback Analysis =====
// Franklin's sensitivity analysis

TransferFunction L = series(C, G);  // Loop gain
TransferFunction S = 1 / (1 + L);   // Sensitivity
TransferFunction T = L / (1 + L);   // Complementary sensitivity

// Key identity: S + T = 1

// ===== Chapter 7: State-Space Design =====
// Franklin emphasizes estimator-based control

StateSpace sys(A, B, C, D);

// Controller design
auto K = place(A, B, desired_controller_poles);

// Estimator (observer) design  
auto L = place(A.T(), C.T(), desired_estimator_poles).T();

// Separation principle: design independently, combine

// ===== Chapter 8: Digital Control =====
// Franklin covers various discretization methods

TransferFunction G_c({1}, {1, 1});  // Continuous

// Zero-Order Hold (most common in practice)
auto G_zoh = c2d(G_c, Ts, "zoh");

// Tustin (bilinear) - preserves frequency characteristics
auto G_tustin = c2d(G_c, Ts, "tustin");

// Matched pole-zero - preserves DC gain and dominant dynamics
auto G_matched = c2d(G_c, Ts, "matched");
```

---

## 🔄 SYNTHESIS: Combined Approach for CppPlot Textbook

### Topic Organization (Best of Both)

| Topic Area | Ogata Strength | Franklin Strength | CppPlot Approach |
|------------|---------------|-------------------|------------------|
| Modeling | Detailed physical examples | Transfer function focus | Both with C++ models |
| Time Analysis | Second-order emphasis | Performance specs | Visual + metrics |
| Root Locus | Construction rules | Design examples | Interactive plots |
| Frequency | Asymptotic Bode | Loop shaping | Complete toolset |
| State-Space | Mathematical rigor | Design-oriented | LQR/LQG focus |
| Digital | Brief coverage | Comprehensive | Modern MPC inclusion |

### Unique CppPlot Contributions

1. **Kalman Filtering** (beyond both textbooks)
   - Standard Kalman Filter
   - Extended Kalman Filter (EKF)
   - Adaptive Kalman Filter
   - Not covered in depth in either Ogata or Franklin

2. **Model Predictive Control** (beyond both textbooks)
   - Constrained optimization
   - Receding horizon
   - Modern industrial practice

3. **LQG/LTR** (extended coverage)
   - Loop Transfer Recovery
   - Robustness recovery

4. **Implementation Focus**
   - Neither Ogata nor Franklin focus on real implementation
   - CppPlot provides actual working code

---

## 📊 Feature Comparison Matrix

### Transfer Function Analysis

| Feature | Ogata | Franklin | MATLAB | CppPlot |
|---------|-------|----------|--------|---------|
| Create TF | ✓ | ✓ | tf() | TransferFunction() |
| Poles/Zeros | ✓ | ✓ | pole(), zero() | poles(), zeros() |
| DC Gain | ✓ | ✓ | dcgain() | dcgain() |
| Stability | ✓ | ✓ | isstable() | isStable() |
| Series | ✓ | ✓ | series() | series() |
| Parallel | ✓ | ✓ | parallel() | parallel() |
| Feedback | ✓ | ✓ | feedback() | feedback() |

### State-Space Analysis

| Feature | Ogata | Franklin | MATLAB | CppPlot |
|---------|-------|----------|--------|---------|
| Create SS | ✓ | ✓ | ss() | StateSpace() |
| Controllability | ✓ | ✓ | ctrb() | controllabilityMatrix() |
| Observability | ✓ | ✓ | obsv() | observabilityMatrix() |
| Pole Placement | ✓ | ✓ | place() | place() |
| LQR | ✓ | ✓ | lqr() | lqr() |
| Observer | ✓ | ✓ | Manual | buildObserver() |

### Frequency Analysis

| Feature | Ogata | Franklin | MATLAB | CppPlot |
|---------|-------|----------|--------|---------|
| Bode Plot | ✓ | ✓ | bode() | bode() |
| Nyquist | ✓ | ✓ | nyquist() | nyquist() |
| Nichols | ✓ | ✓ | nichols() | nichols() |
| Margins | ✓ | ✓ | margin() | margin() |
| Bandwidth | Brief | ✓ | bandwidth() | bandwidth() |

### Digital Control

| Feature | Ogata | Franklin | MATLAB | CppPlot |
|---------|-------|----------|--------|---------|
| c2d | Brief | ✓ | c2d() | c2d() |
| d2c | Brief | ✓ | d2c() | d2c() |
| DLQR | ✗ | ✓ | dlqr() | dlqr() |
| DARE | ✗ | ✓ | dare() | dare() |
| MPC | ✗ | ✗ | mpc() | MPCController |

### Advanced Topics

| Feature | Ogata | Franklin | MATLAB | CppPlot |
|---------|-------|----------|--------|---------|
| Kalman Filter | Brief | Brief | kalman() | KalmanFilter |
| EKF | ✗ | ✗ | extendedKalmanFilter() | ExtendedKalmanFilter |
| Adaptive KF | ✗ | ✗ | Custom | AdaptiveKalmanFilter |
| LQG | Brief | ✓ | lqg() | designLQG() |
| LQG/LTR | ✗ | Brief | Custom | designLQG_LTR() |
| H∞ | Intro | Intro | hinfsyn() | ⏳ Future |

---

## 📝 Teaching Recommendations

### From Ogata (Recommended to Adopt)
1. **Second-order system analysis** - Excellent pedagogical value
2. **Root locus construction rules** - Step-by-step approach
3. **PID tuning examples** - Practical and industry-relevant
4. **Detailed mathematical derivations** - Build understanding

### From Franklin (Recommended to Adopt)
1. **Historical context** - Motivates learning
2. **Design-oriented approach** - Focus on practical outcomes
3. **Comprehensive digital control** - Modern relevance
4. **Case studies** - Real-world applications

### CppPlot Additions (Novel Contributions)
1. **Hands-on coding** - Active learning
2. **Visualization** - Immediate feedback
3. **Modern algorithms** - MPC, Adaptive KF
4. **Real-time capable** - Industry preparation

---

## 🎓 Course Structure Recommendations

### Undergraduate (1 semester)
- Chapters 1-6: Classical control
- Chapter 7-8: Intro to state-space
- Chapter 11: Intro to digital
- Selected labs

### Undergraduate (2 semesters)
**Semester 1:** Chapters 1-6 (Classical)
**Semester 2:** Chapters 7-12 (Modern + Digital)

### Graduate
- Chapters 7-10: State-space focus
- Chapter 12: MPC
- Chapters 13-15: Advanced topics
- Research projects

---

## 📚 Problem Set Mapping

### Ogata Problems → CppPlot Exercises

| Ogata Problem Type | CppPlot Exercise |
|-------------------|------------------|
| Sketch root locus by hand | Verify with rlocus() |
| Calculate gain/phase margins | Use margin(), compare |
| Design PID by Z-N | Implement, simulate, tune |
| State-space transformations | Code transformations |

### Franklin Problems → CppPlot Exercises

| Franklin Problem Type | CppPlot Exercise |
|----------------------|------------------|
| Model physical system | Create StateSpace |
| Design by specifications | Use place() or lqr() |
| Digital implementation | c2d(), simulate |
| Sensitivity analysis | Plot S and T |

---

**Document Version:** 1.0  
**Last Updated:** January 26, 2026
