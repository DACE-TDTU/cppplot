# Modern Control Engineering with C++
## Textbook Development Plan

**Inspired by:**
- Katsuhiko Ogata - *Modern Control Engineering* (6th Edition)
- Gene F. Franklin, J. David Powell, Abbas Emami-Naeini - *Feedback Control of Dynamic Systems* (8th Edition)

**Implementation Platform:** CppPlot Control Systems Library

---

## 🎯 PEDAGOGICAL PHILOSOPHY: OUTCOME-BASED LEARNING (OBL)

### Core Principles

1. **Clear Learning Outcomes:** Each chapter begins with measurable objectives using Bloom's Taxonomy
2. **Physical-First Approach:** Every example starts from a real physical system, then derives the mathematical model
3. **Design Thinking:** Connect theory to practical engineering design decisions
4. **Self-Assessment:** Students can evaluate their own progress against clear rubrics

### Chapter Template Structure

```
┌──────────────────────────────────────────────────────────────┐
│ CHAPTER N: Topic Title                                        │
├──────────────────────────────────────────────────────────────┤
│ 🎯 LEARNING OUTCOMES (Bloom's Levels)                         │
│    □ Remember: Define key terms...                            │
│    □ Understand: Explain relationships...                     │
│    □ Apply: Calculate/simulate...                             │
│    □ Analyze: Determine/compare...                            │
│    □ Evaluate: Assess/justify...                              │
│    □ Create: Design/synthesize...                             │
├──────────────────────────────────────────────────────────────┤
│ 📐 PHYSICAL SYSTEM → MATHEMATICAL MODEL                       │
│    Physical Setup → Energy/Force Balance → Transfer Function  │
├──────────────────────────────────────────────────────────────┤
│ 💻 IMPLEMENTATION WITH CppPlot                                │
│    Code examples with engineering context                     │
├──────────────────────────────────────────────────────────────┤
│ 🔧 DESIGN EXAMPLES                                            │
│    Specifications → Analysis → Design → Verification          │
├──────────────────────────────────────────────────────────────┤
│ ✅ SELF-ASSESSMENT CHECKLIST & RUBRIC                         │
│    Outcome verification, common misconceptions                │
└──────────────────────────────────────────────────────────────┘
```

### Learning Flow: Physical System to Controller Design

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│ PHYSICAL SYSTEM │────▶│ MATHEMATICAL     │────▶│ TRANSFER        │
│ (Real world)    │     │ MODEL (ODE)      │     │ FUNCTION G(s)   │
└─────────────────┘     └──────────────────┘     └────────┬────────┘
        │                                                  │
        │                                                  ▼
        │                                        ┌─────────────────┐
        │                                        │ ANALYSIS        │
        │                                        │ - Stability     │
        │                                        │ - Performance   │
        │                                        └────────┬────────┘
        │                                                  │
        ▼                                                  ▼
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│ IMPLEMENTATION  │◀────│ CONTROLLER       │◀────│ DESIGN          │
│ & TESTING       │     │ SYNTHESIS        │     │ SPECIFICATIONS  │
└─────────────────┘     └──────────────────┘     └─────────────────┘
```

### Bloom's Taxonomy Alignment

| Level | Verb Examples | Assessment Type |
|-------|---------------|-----------------|
| **Remember** | Define, List, State | Quiz, Term matching |
| **Understand** | Explain, Interpret, Compare | Concept questions |
| **Apply** | Calculate, Simulate, Use | Problem sets |
| **Analyze** | Determine, Classify, Differentiate | Analysis exercises |
| **Evaluate** | Assess, Justify, Critique | Design review |
| **Create** | Design, Construct, Develop | Projects |

---

## 📚 Part I: TEXTBOOK STRUCTURE ANALYSIS

### Ogata's "Modern Control Engineering" Structure:

### Ogata's "Modern Control Engineering" Structure:
```
1. Introduction to Control Systems
2. Mathematical Modeling of Dynamic Systems
3. Mathematical Modeling of Mechanical/Electrical Systems
4. Transient and Steady-State Response Analysis
5. Root-Locus Analysis
6. Frequency-Response Analysis
7. Control Systems Design by Root-Locus Method
8. Control Systems Design by Frequency-Response Method
9. PID Controllers and Modified PID Controllers
10. State-Space Analysis of Control Systems
11. State-Space Design of Control Systems
12. Modeling and Analysis in MATLAB
```

### Franklin's "Feedback Control of Dynamic Systems" Structure:
```
1. An Overview and Brief History of Feedback Control
2. Dynamic Models
3. Dynamic Response
4. A First Analysis of Feedback
5. The Root-Locus Design Method
6. The Frequency-Response Design Method
7. State-Space Design
8. Digital Control
9. Nonlinear Systems
10. Control System Design: Principles and Case Studies
```

---

## 📖 Part II: PROPOSED TEXTBOOK OUTLINE

# **"Modern Control Engineering: Theory and C++ Implementation"**

## **Preface**
- Why C++ for Control Systems?
- About the CppPlot Library
- How to Use This Book
- Prerequisites (Linear Algebra, Differential Equations, C++ basics)

---

## **PART I: FOUNDATIONS OF CONTROL SYSTEMS**

### **Chapter 1: Introduction to Control Systems**
#### 1.1 What is a Control System?
- Open-loop vs Closed-loop Systems
- Examples: Thermostat, Cruise Control, Industrial Processes

#### 1.2 Historical Development
- James Watt's Governor
- Feedback Amplifiers (Black, Bode, Nyquist)
- Modern Era: Digital Control, Optimal Control

#### 1.3 Control System Components
- Plant, Sensor, Actuator, Controller
- Block Diagram Representation

#### 1.4 Introduction to CppPlot
```cpp
// First program: Plot a step response
#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>

int main() {
    using namespace cppplot;
    using namespace cppplot::control;
    
    // Create a first-order system: G(s) = 1/(s+1)
    TransferFunction G({1}, {1, 1});
    
    // Step response
    auto [t, y] = step(G, 10.0);
    
    // Plot
    figure(800, 600);
    plot(t, y, "b-", {{"linewidth", "2"}});
    xlabel("Time [s]");
    ylabel("Response");
    title("First-Order Step Response");
    grid(true);
    savefig("ch1_first_order.svg");
    
    return 0;
}
```

#### 1.5 C++ Lab: Building Your First Control System Simulation
- Setting up the development environment
- Compiling and running examples

---

### **Chapter 2: Mathematical Modeling of Dynamic Systems**

> **🎯 Learning Outcomes:**
> - **Remember:** Define transfer function, poles, zeros, state-space representation
> - **Understand:** Explain how physical laws lead to differential equations
> - **Apply:** Derive mathematical models from mechanical, electrical, thermal systems
> - **Analyze:** Convert between transfer function and state-space representations
> - **Create:** Build models of new physical systems from first principles

#### 2.1 Modeling Philosophy: From Physics to Math

**The Systematic Modeling Process:**
```
┌──────────────┐    ┌───────────────┐    ┌────────────────┐    ┌──────────────┐
│   Physical   │───▶│ Conservation  │───▶│  Differential  │───▶│   Transfer   │
│    System    │    │     Laws      │    │   Equations    │    │   Function   │
└──────────────┘    └───────────────┘    └────────────────┘    └──────────────┘
```

#### 2.2 Mechanical Systems: Mass-Spring-Damper

**Physical System:**
```
    ┌───────┐
    │   M   │──────▶ F(t)    Free Body Diagram:
    └───┬───┘                  F - kx - bẋ = Mẍ
    ════╪════  (Spring k)
        │                    Newton's Law leads to:
    ────╫────  (Damper b)      Mẍ + bẋ + kx = F(t)
        │
    ────┴────  (Fixed)
```

#### 2.3 Electrical Systems: RLC Circuit

**From Kirchhoff's Laws to Transfer Function:**
```cpp
// Physical parameters
double R = 100, L = 0.1, C = 1e-6;

// Natural frequency and damping
double wn = 1/sqrt(L*C);       // 3162 rad/s
double zeta = R/(2*sqrt(L/C)); // 0.158

// Transfer function (voltage across capacitor)
TransferFunction G_RLC({1/(L*C)}, {1, R/L, 1/(L*C)});
```

#### 2.4 Transfer Functions
- Laplace Transform Review
- Transfer Function Definition
- Poles and Zeros

```cpp
// Example: Create and analyze transfer function
TransferFunction G({2, 1}, {1, 3, 2});  // G(s) = (2s+1)/(s²+3s+2)

std::cout << "Poles: ";
auto poles = G.poles();
for (auto p : poles) std::cout << p << " ";

std::cout << "\nZeros: ";
auto zeros = G.zeros();
for (auto z : zeros) std::cout << z << " ";

std::cout << "\nDC Gain: " << G.dcgain() << std::endl;
```

#### 2.3 State-Space Representation
- State Variables
- State Equations
- Converting Between Representations

```cpp
// State-space system
Matrix A = {{0, 1}, {-2, -3}};
Matrix B = {{0}, {1}};
Matrix C = {{1, 0}};
Matrix D = {{0}};

StateSpace sys(A, B, C, D);

// Convert to transfer function
TransferFunction G = sys.toTransferFunction();
```

#### 2.4 Block Diagram Algebra
- Series, Parallel, Feedback Connections
- Mason's Gain Formula

```cpp
// Block diagram algebra
TransferFunction G1({1}, {1, 1});
TransferFunction G2({2}, {1, 2});
TransferFunction H({1}, {1, 0.5});

auto G_series = series(G1, G2);
auto G_parallel = parallel(G1, G2);
auto G_feedback = feedback(series(G1, G2), H);
```

#### 2.5 Mechanical System Modeling
- Mass-Spring-Damper Systems
- Rotational Systems
- Analogies with Electrical Systems

#### 2.6 Electrical System Modeling
- RLC Circuits
- Op-Amp Circuits
- Motor Dynamics

#### 2.7 C++ Lab: Modeling a DC Motor
```cpp
// DC Motor model
// State: [θ, ω, i]
// θ = angle, ω = angular velocity, i = current
double J = 0.01;   // Inertia
double b = 0.1;    // Friction
double K = 0.01;   // Motor constant
double R = 1.0;    // Resistance
double L = 0.5;    // Inductance

Matrix A = {{0, 1, 0},
            {0, -b/J, K/J},
            {0, -K/L, -R/L}};
Matrix B = {{0}, {0}, {1/L}};
Matrix C = {{1, 0, 0}};  // Measure angle
Matrix D = {{0}};

StateSpace motor(A, B, C, D);
```

---

### **Chapter 3: Time-Domain Analysis**

#### 3.1 First-Order Systems
- Time Constant
- Rise Time, Settling Time

```cpp
// First-order system analysis
double tau = 2.0;  // Time constant
TransferFunction G({1}, {tau, 1});

auto [t, y] = step(G, 10.0);
auto info = stepinfo(t, y);

std::cout << "Rise Time: " << info.rise_time << " s\n";
std::cout << "Settling Time: " << info.settling_time << " s\n";
```

#### 3.2 Second-Order Systems
- Natural Frequency (ωn)
- Damping Ratio (ζ)
- Overdamped, Critically Damped, Underdamped

```cpp
// Second-order system parameter study
figure(1000, 800);
layout(2, 2);

std::vector<double> zetas = {0.1, 0.3, 0.5, 0.7, 1.0, 2.0};
double wn = 1.0;

subplot(2, 2, 1);
for (double zeta : zetas) {
    TransferFunction G({wn*wn}, {1, 2*zeta*wn, wn*wn});
    auto [t, y] = step(G, 15.0);
    plot(t, y, "-", {{"label", "ζ=" + std::to_string(zeta)}});
}
title("Step Response vs Damping Ratio");
legend();
grid(true);
```

#### 3.3 Higher-Order Systems
- Dominant Poles
- Approximation by Second-Order Systems

#### 3.4 Stability Analysis
- BIBO Stability
- Routh-Hurwitz Criterion

```cpp
// Routh-Hurwitz stability
Polynomial den({1, 6, 11, 6});  // s³ + 6s² + 11s + 6
auto routh = routhTable(den);
bool stable = isStable(routh);
```

#### 3.5 Steady-State Error
- System Type
- Error Constants (Kp, Kv, Ka)
- Steady-State Error Formulas

```cpp
// Error analysis
TransferFunction G({10}, {1, 2, 0});  // Type 1 system

double Kp = dcgain(G);  // Position error constant
double Kv = velocityErrorConstant(G);
double Ka = accelerationErrorConstant(G);

std::cout << "Step input error: " << 1/(1+Kp) << std::endl;
std::cout << "Ramp input error: " << 1/Kv << std::endl;
```

#### 3.6 C++ Lab: Complete Time-Domain Analysis Tool
- Interactive system analysis
- Automatic performance metrics extraction

---

### **Chapter 4: Root Locus Analysis**

#### 4.1 Basic Root Locus Concepts
- Definition and Motivation
- Root Locus Rules

#### 4.2 Constructing Root Locus
- Starting and Ending Points
- Asymptotes
- Breakaway and Break-in Points
- Angle of Departure/Arrival

```cpp
// Root locus plot
TransferFunction G({1}, {1, 3, 2, 0});  // G(s) = 1/(s(s+1)(s+2))

figure(800, 600);
rlocus(G);
title("Root Locus: G(s) = 1/(s(s+1)(s+2))");
grid(true);
savefig("ch4_rlocus.svg");
```

#### 4.3 Design Using Root Locus
- Gain Selection
- Adding Poles and Zeros
- Lead and Lag Compensators

```cpp
// Lead compensator design
TransferFunction G({1}, {1, 1, 0});

// Design specs: ζ = 0.5, ωn = 4 rad/s
// Desired poles: -2 ± j3.46

// Lead compensator: Gc(s) = Kc(s+z)/(s+p)
TransferFunction Gc({1, 2}, {1, 10});  // Zero at -2, pole at -10

auto G_comp = series(Gc, G);

figure(1000, 500);
layout(1, 2);

subplot(1, 2, 1);
rlocus(G);
title("Original System");

subplot(1, 2, 2);
rlocus(G_comp);
title("With Lead Compensator");

savefig("ch4_lead_design.svg");
```

#### 4.4 C++ Lab: Interactive Root Locus Design Tool

---

### **Chapter 5: Frequency Response Analysis**

#### 5.1 Frequency Response Fundamentals
- Sinusoidal Input/Output
- Magnitude and Phase

#### 5.2 Bode Plots
- Magnitude Plot (dB)
- Phase Plot (degrees)
- Asymptotic Approximations

```cpp
// Bode plot
TransferFunction G({100}, {1, 10, 100});

figure(800, 600);
bode(G);
title("Bode Plot: Second-Order System");
savefig("ch5_bode.svg");
```

#### 5.3 Nyquist Plots
- Nyquist Stability Criterion
- Gain and Phase Margins

```cpp
// Nyquist plot and stability margins
TransferFunction G({1}, {1, 2, 1, 0});

figure(1000, 500);
layout(1, 2);

subplot(1, 2, 1);
nyquist(G);
title("Nyquist Plot");

subplot(1, 2, 2);
bode(G);
auto margins = margin(G);
std::cout << "Gain Margin: " << margins.gain_margin << " dB\n";
std::cout << "Phase Margin: " << margins.phase_margin << " deg\n";
```

#### 5.4 Nichols Charts

#### 5.5 Frequency Domain Specifications
- Bandwidth
- Resonance Peak
- Relation to Time-Domain Specs

```cpp
// Bandwidth and resonance analysis
TransferFunction G_cl = feedback(G, 1);
auto bw = bandwidth(G_cl);
auto [Mr, wr] = resonancePeak(G_cl);
```

#### 5.6 C++ Lab: Frequency Response Design Tool

---

### **Chapter 6: Classical Controller Design**

#### 6.1 PID Control
- Proportional Control
- Integral Control
- Derivative Control
- PID Tuning Methods

```cpp
// PID controller
double Kp = 10, Ki = 5, Kd = 2;
TransferFunction C_pid = pid(Kp, Ki, Kd);

// Plant
TransferFunction G({1}, {1, 2, 1});

// Closed-loop
auto G_cl = feedback(series(C_pid, G), 1);

// Compare with and without PID
figure(800, 600);
auto [t1, y1] = step(feedback(G, 1), 10);
auto [t2, y2] = step(G_cl, 10);
plot(t1, y1, "b--", {{"label", "Without PID"}});
plot(t2, y2, "r-", {{"label", "With PID"}});
legend();
title("PID Control Effect");
```

#### 6.2 Ziegler-Nichols Tuning
- Open-Loop Method
- Closed-Loop Method

```cpp
// Ziegler-Nichols tuning
auto [Kp, Ki, Kd] = zieglerNicholsClosedLoop(Ku, Tu);
// Ku = ultimate gain, Tu = ultimate period
```

#### 6.3 Lead-Lag Compensation
- Lead Compensator Design
- Lag Compensator Design
- Lead-Lag Combination

```cpp
// Lead compensator design by phase margin
TransferFunction G({1}, {1, 1, 0});

// Design for PM = 50°, crossover = 2 rad/s
double phi_max = 50 * M_PI / 180;
double alpha = (1 - sin(phi_max)) / (1 + sin(phi_max));
double wc = 2.0;
double T = 1 / (wc * sqrt(alpha));

TransferFunction C_lead({T, 1}, {alpha*T, 1});
double K = 1 / abs(evalTF(series(C_lead, G), wc));
C_lead = C_lead * K;
```

#### 6.4 C++ Lab: Complete Controller Design Project

---

## **PART II: STATE-SPACE METHODS**

### **Chapter 7: State-Space Analysis**

#### 7.1 State-Space Representation Review
- Canonical Forms
- Similarity Transformations

#### 7.2 Solution of State Equations
- Matrix Exponential
- State Transition Matrix

```cpp
// State transition matrix
Matrix A = {{0, 1}, {-2, -3}};
Matrix eAt = matrixExponential(A, t);
```

#### 7.3 Controllability and Observability
- Controllability Matrix
- Observability Matrix
- Kalman Decomposition

```cpp
// Check controllability and observability
StateSpace sys(A, B, C, D);

auto Wc = controllabilityMatrix(sys);
auto Wo = observabilityMatrix(sys);

bool controllable = isControllable(sys);
bool observable = isObservable(sys);

std::cout << "Controllable: " << (controllable ? "Yes" : "No") << std::endl;
std::cout << "Observable: " << (observable ? "Yes" : "No") << std::endl;
```

#### 7.4 Stability in State-Space
- Eigenvalues and Stability
- Lyapunov Stability

```cpp
// Lyapunov stability analysis
Matrix Q = Matrix::eye(2);
Matrix P = lyapunov(A, Q);  // Solve A'P + PA + Q = 0

// Check if P is positive definite
auto eig_P = eigenvalues(P);
bool lyap_stable = all_positive(eig_P);
```

#### 7.5 C++ Lab: State-Space Analysis Toolkit

---

### **Chapter 8: State-Space Controller Design**

#### 8.1 State Feedback Control
- Pole Placement
- Ackermann's Formula

```cpp
// Pole placement
StateSpace sys(A, B, C, D);
std::vector<std::complex<double>> desired_poles = {{-2, 1}, {-2, -1}};

auto K = place(A, B, desired_poles);

std::cout << "State feedback gain K = " << K << std::endl;

// Closed-loop system
Matrix A_cl = A - B * K;
StateSpace sys_cl(A_cl, B, C, D);
```

#### 8.2 Observer Design
- Full-Order Observer
- Reduced-Order Observer

```cpp
// Observer design
auto L = place(A.transpose(), C.transpose(), observer_poles).transpose();

// Observer state-space
Matrix Ao = A - L * C;
```

#### 8.3 Combined Controller-Observer (Compensator)
- Separation Principle
- Compensator Transfer Function

```cpp
// Build compensator
auto comp = buildCompensator(A, B, C, K, L);
```

#### 8.4 Integral Control in State-Space
- Tracking and Disturbance Rejection
- Augmented System

#### 8.5 C++ Lab: Inverted Pendulum Control

```cpp
// Inverted pendulum state-space model
double m = 0.1, M = 1.0, l = 0.5, g = 9.81;

Matrix A = {{0, 1, 0, 0},
            {0, 0, -m*g/M, 0},
            {0, 0, 0, 1},
            {0, 0, (M+m)*g/(M*l), 0}};
Matrix B = {{0}, {1/M}, {0}, {-1/(M*l)}};
Matrix C = {{1, 0, 0, 0}, {0, 0, 1, 0}};  // Measure position and angle
Matrix D = Matrix::zeros(2, 1);

StateSpace pendulum(A, B, C, D);

// LQR design
Matrix Q = Matrix::diag({10, 1, 100, 10});  // Penalize angle heavily
Matrix R = {{1}};

auto [K, S, E] = lqr(A, B, Q, R);
```

---

### **Chapter 9: Optimal Control**

#### 9.1 Introduction to Optimal Control
- Performance Indices
- Calculus of Variations

#### 9.2 Linear Quadratic Regulator (LQR)
- Finite-Horizon LQR
- Infinite-Horizon LQR
- Algebraic Riccati Equation

```cpp
// LQR design
Matrix Q = {{10, 0}, {0, 1}};
Matrix R = {{1}};

auto [K, S, poles] = lqr(A, B, Q, R);

std::cout << "Optimal gain K = " << K << std::endl;
std::cout << "Cost matrix S = " << S << std::endl;
std::cout << "Closed-loop poles: ";
for (auto p : poles) std::cout << p << " ";
```

#### 9.3 Linear Quadratic Gaussian (LQG) Control
- Kalman Filter
- LQG Compensator
- Loop Transfer Recovery (LTR)

```cpp
// LQG design
Matrix W = Matrix::eye(2) * 0.1;  // Process noise
Matrix V = Matrix::eye(1) * 0.01; // Measurement noise

auto lqg = designLQG(A, B, C, Q, R, W, V);

std::cout << "State feedback K = " << lqg.K << std::endl;
std::cout << "Kalman gain L = " << lqg.L << std::endl;

// LQG with Loop Transfer Recovery
auto lqg_ltr = designLQG_LTR(A, B, C, Q, R, rho);
```

#### 9.4 H∞ Control (Introduction)

#### 9.5 C++ Lab: Optimal Cruise Control System

---

### **Chapter 10: Kalman Filtering and State Estimation**

#### 10.1 The Estimation Problem
- Stochastic Systems
- Optimal Estimation

#### 10.2 Discrete Kalman Filter
- Prediction Step
- Correction Step
- Kalman Gain

```cpp
// Kalman filter implementation
KalmanFilter kf(A, B, C, Q, R);
kf.setInitialState(x0, P0);

for (auto& measurement : measurements) {
    auto estimate = kf.update(measurement, control_input);
    std::cout << "Estimated state: " << estimate.x_hat << std::endl;
    std::cout << "Covariance: " << estimate.P << std::endl;
}
```

#### 10.3 Extended Kalman Filter (EKF)
- Nonlinear Systems
- Linearization

```cpp
// EKF for nonlinear system
auto f = [](const Matrix& x, const Matrix& u) {
    // Nonlinear state transition
    return Matrix({{x(0,0) + x(1,0)*dt},
                   {x(1,0) + sin(x(0,0))*dt + u(0,0)*dt}});
};

auto h = [](const Matrix& x) {
    // Nonlinear measurement
    return Matrix({{x(0,0)*x(0,0)}});
};

ExtendedKalmanFilter ekf(f, h, Q, R);
```

#### 10.4 Adaptive Kalman Filter
- Innovation-Based Adaptation
- Covariance Matching
- Sage-Husa Algorithm

```cpp
// Adaptive Kalman filter
AdaptiveKalmanFilter akf(A, B, C, Q_init, R_init,
                          AdaptiveKalmanFilter::Method::COVARIANCE_MATCHING);

// Filter adapts Q and R online
for (auto& z : measurements) {
    auto est = akf.update(z, u);
    
    // Check adapted covariances
    Matrix Q_adapted = akf.getQ();
    Matrix R_adapted = akf.getR();
}
```

#### 10.5 Sensor Fusion Applications
- GPS/INS Integration
- Multi-Sensor Fusion

#### 10.6 C++ Lab: Vehicle Position Tracking with Kalman Filter

---

## **PART III: DIGITAL CONTROL SYSTEMS**

### **Chapter 11: Discrete-Time Systems**

#### 11.1 Sampling and Reconstruction
- Ideal Sampler
- Zero-Order Hold
- Shannon's Sampling Theorem

#### 11.2 Z-Transform
- Definition and Properties
- Inverse Z-Transform

#### 11.3 Discrete Transfer Functions
- Pulse Transfer Function
- Discretization Methods

```cpp
// Continuous to discrete conversion
TransferFunction G_c({1}, {1, 1});  // Continuous: 1/(s+1)

double Ts = 0.1;  // Sampling time

// Different methods
auto G_d_zoh = c2d(G_c, Ts, "zoh");    // Zero-order hold
auto G_d_tustin = c2d(G_c, Ts, "tustin");  // Bilinear transform
auto G_d_matched = c2d(G_c, Ts, "matched"); // Matched pole-zero
```

#### 11.4 Discrete State-Space
- Discretization of State Equations
- Discrete Controllability/Observability

```cpp
// Discrete state-space
StateSpace sys_c(A, B, C, D);
auto sys_d = c2d(sys_c, Ts, "zoh");
```

#### 11.5 Stability in Discrete-Time
- Unit Circle Criterion
- Jury Stability Test

#### 11.6 C++ Lab: Digital Filter Design

---

### **Chapter 12: Digital Controller Design**

#### 12.1 Digital PID Control
- Discretization of PID
- Anti-Windup

```cpp
// Discrete PID
class DigitalPID {
    double Kp, Ki, Kd, Ts;
    double integral = 0;
    double prev_error = 0;
    double u_max, u_min;  // Saturation limits
    
public:
    double compute(double setpoint, double measurement) {
        double error = setpoint - measurement;
        
        // Anti-windup: only integrate if not saturated
        if (output > u_min && output < u_max) {
            integral += error * Ts;
        }
        
        double derivative = (error - prev_error) / Ts;
        prev_error = error;
        
        double output = Kp * error + Ki * integral + Kd * derivative;
        return std::clamp(output, u_min, u_max);
    }
};
```

#### 12.2 Direct Digital Design
- Root Locus in Z-plane
- Frequency Response in Discrete-Time

#### 12.3 Discrete LQR and LQG
- Discrete Riccati Equation
- DLQR, DLQE

```cpp
// Discrete LQR
auto K = dlqr(A_d, B_d, Q, R);

// Discrete Kalman filter
auto L = dlqe(A_d, C_d, W, V);
```

#### 12.4 Model Predictive Control (MPC)
- Basic MPC Formulation
- Constraints Handling
- Receding Horizon

```cpp
// MPC controller
MPCConfig config(20, Q, R);  // Horizon = 20
config.u_min = {-10.0};
config.u_max = {10.0};
config.x_min = {-5.0, -2.0};
config.x_max = {5.0, 2.0};

MPCController mpc(A, B, C, config);

// Control loop
Matrix x = x0;
for (int k = 0; k < N; ++k) {
    auto solution = mpc.solve(x);
    Matrix u = solution.getFirstControl(1);
    
    // Apply control and update state
    x = A * x + B * u;
}
```

#### 12.5 C++ Lab: Real-Time MPC Implementation

---

## **PART IV: ADVANCED TOPICS**

### **Chapter 13: Nonlinear Control Systems**

#### 13.1 Nonlinear System Analysis
- Phase Plane Analysis
- Describing Functions

#### 13.2 Linearization
- Jacobian Linearization
- Small-Signal Analysis

```cpp
// Linearization around equilibrium
auto [A_lin, B_lin] = linearize(f, g, x_eq, u_eq);
```

#### 13.3 Lyapunov Stability Theory
- Lyapunov Functions
- LaSalle's Invariance Principle

#### 13.4 Feedback Linearization

#### 13.5 Sliding Mode Control

#### 13.6 C++ Lab: Nonlinear Pendulum Control

---

### **Chapter 14: Robust Control**

#### 14.1 Uncertainty and Robustness
- Parametric Uncertainty
- Unstructured Uncertainty

#### 14.2 Sensitivity Functions
- Sensitivity S(s)
- Complementary Sensitivity T(s)

```cpp
// Sensitivity analysis
TransferFunction L = series(C, G);  // Loop transfer function
TransferFunction S = sensitivity(L);     // S = 1/(1+L)
TransferFunction T = complementary(L);   // T = L/(1+L)

figure(800, 600);
bode(S, "b-", {{"label", "S"}});
bode(T, "r-", {{"label", "T"}});
legend();
title("Sensitivity Functions");
```

#### 14.3 Loop Shaping

#### 14.4 μ-Synthesis (Introduction)

#### 14.5 C++ Lab: Robust Controller Design

---

### **Chapter 15: MIMO Systems**

#### 15.1 MIMO Transfer Functions
- Transfer Function Matrix
- Poles and Zeros of MIMO Systems

```cpp
// MIMO system
Matrix num = {{Polynomial({1}), Polynomial({0})},
              {Polynomial({2}), Polynomial({1})}};
Matrix den = {{Polynomial({1, 1}), Polynomial({1})},
              {Polynomial({1, 2}), Polynomial({1, 1})}};
              
TransferFunctionMatrix G(num, den);
```

#### 15.2 Decoupling Control
- Relative Gain Array (RGA)
- Decoupling Networks

#### 15.3 MIMO Frequency Response
- Singular Value Plots

```cpp
// Singular value plot
figure(800, 600);
sigma(G);  // Singular value Bode plot
title("MIMO Singular Values");
```

#### 15.4 C++ Lab: Multivariable Process Control

---

### **Chapter 16: Case Studies and Projects**

#### 16.1 Automotive Cruise Control
- Complete design from modeling to implementation

#### 16.2 Quadrotor Altitude Control
- Nonlinear model, linearization, LQR design

#### 16.3 Industrial Temperature Control
- PID tuning, disturbance rejection

#### 16.4 Flexible Robot Arm
- Vibration suppression, observer design

#### 16.5 Autonomous Vehicle Lane Keeping
- MPC with constraints

---

## **APPENDICES**

### **Appendix A: Linear Algebra Review**
- Matrix operations
- Eigenvalues and eigenvectors
- Matrix decompositions

### **Appendix B: Laplace and Z-Transform Tables**

### **Appendix C: CppPlot Control Library Reference**
- Complete API documentation
- Installation guide
- Compilation options

### **Appendix D: C++ Programming for Control Engineers**
- Modern C++ features (C++17)
- Numerical computing best practices
- Code optimization techniques

### **Appendix E: Solutions to Selected Problems**

---

## 📋 Part III: IMPLEMENTATION ROADMAP

### Phase 1: Foundation (Chapters 1-3) ✅ COMPLETE
- [x] Complete time-domain analysis tools
- [x] Step response, impulse response
- [x] Performance metrics (stepinfo)
- [x] Stability analysis (Routh-Hurwitz)

### Phase 2: Classical Methods (Chapters 4-6) ✅ COMPLETE
- [x] Root locus plotting and analysis
- [x] Bode, Nyquist, Nichols plots
- [x] Margin calculations
- [x] PID controller design tools

### Phase 3: State-Space (Chapters 7-10) ✅ COMPLETE
- [x] Controllability/Observability
- [x] Pole placement (place, acker)
- [x] LQR/LQG design
- [x] Kalman filtering (KF, EKF, UKF)

### Phase 4: Digital Control (Chapters 11-12) ✅ COMPLETE
- [x] c2d/d2c conversions (ZOH, Tustin, matched)
- [x] Discrete analysis tools
- [x] DLQR
- [x] MPC with constraints

### Phase 5: Advanced Topics (Chapters 13-15) ✅ COMPLETE
- [x] Linearization tools
- [x] MIMO analysis (basic)
- [x] Robust control concepts

### Phase 6: Case Studies (Chapter 16) ✅ COMPLETE
- [x] Complete examples with code
- [x] Documentation (16 chapters + 2 appendices)

### Phase 7: EE/Telecom Extensions ✅ COMPLETE (January 2026)
- [x] Power electronics examples (Buck, Boost, PFC)
- [x] PLL/DPLL control examples
- [x] Motor drive applications (DC, BLDC, PMSM)
- [x] Grid-tied inverter control
- [x] Telecommunications examples (AGC, equalization)

### Phase 8: Quality Assurance ✅ COMPLETE (January 2026)
- [x] Library audit against textbook content
- [x] Missing ch10 example added
- [x] All 24 textbook examples verified
- [x] Audit report created

---

## 📊 Part IV: COMPARISON WITH EXISTING TOOLS

| Feature | MATLAB | Python-Control | CppPlot |
|---------|--------|----------------|---------|
| Transfer Functions | ✓ | ✓ | ✓ |
| State-Space | ✓ | ✓ | ✓ |
| Root Locus | ✓ | ✓ | ✓ |
| Bode/Nyquist/Nichols | ✓ | ✓ | ✓ |
| LQR/LQG | ✓ | ✓ | ✓ |
| Kalman Filter (KF/EKF/UKF) | ✓ | ✓ | ✓ |
| MPC with Constraints | ✓ (Toolbox) | ✓ | ✓ |
| Adaptive KF | Custom | Custom | ✓ |
| Discrete Control (ZOH/Tustin) | ✓ | ✓ | ✓ |
| DARE/CARE Solvers | ✓ | ✓ | ✓ |
| Pole Placement (Ackermann) | ✓ | ✓ | ✓ |
| Real-time capable | Limited | Limited | ✓ |
| Header-only | N/A | N/A | ✓ |
| No dependencies | ✗ | ✗ | ✓ |
| Cross-platform | ✓ | ✓ | ✓ |
| Free/Open-source | ✗ | ✓ | ✓ |
| Educational Textbook | Multiple | Limited | ✓ (16 ch) |
| EE/Telecom Examples | ✓ | Limited | ✓ |

---

## 📝 Part V: WRITING GUIDELINES

### Style
- Clear, concise explanations
- Mathematical rigor with intuitive explanations
- Every concept illustrated with C++ code
- Figures generated from actual CppPlot code

### Pedagogy
- Learning objectives at chapter start
- Summary at chapter end
- Exercises: theory + programming
- Lab sections for hands-on practice

### Code Examples
- Complete, compilable programs
- Well-documented with comments
- Follow modern C++ best practices
- Available in accompanying repository

---

## 🎯 Part VI: UNIQUE SELLING POINTS

1. **First Control Systems textbook using pure C++**
   - No MATLAB license required
   - Industry-relevant programming skills

2. **Real-time implementation ready**
   - Code can run on embedded systems
   - No interpreter overhead

3. **Modern C++ practices**
   - Teaching good software engineering
   - Preparing students for industry

4. **Open-source tools**
   - Free for education
   - Students can modify and extend

5. **Visual learning**
   - High-quality SVG plots
   - Interactive examples possible

---

## 🔎 Attribution & Citation Policy

- This textbook is written in original language by the authors of the CppPlot Control Systems Project. It does not reproduce verbatim text from third-party books, articles, or websites.
- Standard mathematical facts, formulas, and algorithms (e.g., Laplace transforms, Routh-Hurwitz, LQR, Kalman filter) are presented in our own wording. Facts and methods themselves are public domain; however, when the presentation follows well-known treatments, we acknowledge sources in the bibliography.
- Any direct quotations, tables, or figures adapted from external sources will be explicitly marked as quotations or adaptations and accompanied by full citations.
- All code examples are original and implemented specifically for this textbook using the CppPlot library; plots are generated programmatically from the provided code.
- If readers identify wording that resembles a published source too closely, please report it. We will revise the text to ensure paraphrasing quality and add citations where appropriate.

---

## 📚 BIBLIOGRAPHY (Key References)

1. Ogata, K. (2010). *Modern Control Engineering* (5th ed.). Prentice Hall.
2. Franklin, G. F., Powell, J. D., & Emami-Naeini, A. (2019). *Feedback Control of Dynamic Systems* (8th ed.). Pearson.
3. Dorf, R. C., & Bishop, R. H. (2016). *Modern Control Systems* (13th ed.). Pearson.
4. Åström, K. J., & Murray, R. M. (2021). *Feedback Systems: An Introduction for Scientists and Engineers* (2nd ed.). Princeton University Press.
5. Skogestad, S., & Postlethwaite, I. (2005). *Multivariable Feedback Control* (2nd ed.). Wiley.
6. Anderson, B. D. O., & Moore, J. B. (2007). *Optimal Control: Linear Quadratic Methods*. Dover.
7. Simon, D. (2006). *Optimal State Estimation: Kalman, H∞, and Nonlinear Approaches*. Wiley.
8. Rawlings, J. B., Mayne, D. Q., & Diehl, M. (2017). *Model Predictive Control: Theory, Computation, and Design* (2nd ed.). Nob Hill Publishing.

---

## 📡 Part VII: ELECTRICAL ENGINEERING & TELECOMMUNICATIONS APPLICATIONS

### Added Content Summary (January 2026)

The following EE/Telecom-specific sections have been integrated across chapters:

| Chapter | New Section | Topics Covered |
|---------|-------------|----------------|
| Ch01 | §1.4 | Control in EE: power converters, PLL overview |
| Ch02 | §2.8 | Buck/Boost converter modeling, RLC circuits |
| Ch03 | §3.8 | Op-amp transfer functions, filter analysis |
| Ch06 | §6.5 | Buck converter Bode, Op-amp GBW, PLL bandwidth, channel equalization |
| Ch08 | §8.6 | Type III compensator, PLL loop filter, active filter design |
| Ch09 | §9.8 | Three-phase inverter dq-frame, DC motor field weakening, PLL state-space |
| Ch10 | §10.9 | Controllability/Observability for inverters, sensorless motors, PLL |
| Ch11 | §11.9 | Grid inverter state feedback, BLDC motor control, DPLL LQR |
| Ch12 | §12.11 | Grid voltage observer, Kalman channel estimation, DC-link observer |
| Ch13 | §13.12 | Digital PFC, DPLL, Digital AGC, Grid-tied inverter control |

### Key EE/Telecom Applications Covered

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    EE/TELECOM APPLICATIONS IN TEXTBOOK                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  POWER ELECTRONICS                    TELECOMMUNICATIONS                    │
│  ─────────────────                    ──────────────────                    │
│  • Buck converter voltage control     • Phase-Locked Loop (PLL)            │
│  • Boost PFC (Power Factor Correction)• Digital PLL (DPLL/ADPLL)           │
│  • Grid-tied inverter control         • Automatic Gain Control (AGC)       │
│  • DC-link voltage regulation         • Channel equalization               │
│  • MPPT for solar inverters           • Carrier recovery                   │
│                                                                             │
│  MOTOR DRIVES                         SIGNAL PROCESSING                    │
│  ────────────                         ─────────────────                    │
│  • DC motor speed/position control    • Active filters (Butterworth)       │
│  • BLDC/PMSM field-oriented control   • State-variable filters             │
│  • Sensorless motor control           • Kalman-based estimation            │
│  • Field weakening operation          • Adaptive filtering                 │
│  • dq-frame current control                                                │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Example: PLL as Control System

```cpp
// Type II PLL - Linearized model
// Open-loop: G_OL(s) = K_PD * K_VCO * F(s) / s
// where F(s) = (1 + sτ₂)/(sτ₁) is the loop filter

double Kpd = 1.0;           // Phase detector gain (V/rad)
double Kvco = 2*M_PI*1000;  // VCO gain (rad/s/V)
double tau1 = 1e-3;         // Loop filter pole
double tau2 = 0.1e-3;       // Loop filter zero

// Transfer function: K_PD * K_VCO * (1 + sτ₂) / (s² * τ₁)
TransferFunction G_OL({Kpd*Kvco*tau2, Kpd*Kvco}, {tau1, 0, 0});

// Analyze stability
auto margins = margin(G_OL);
std::cout << "PLL Phase Margin: " << margins.Pm << " deg" << std::endl;

// Closed-loop bandwidth
TransferFunction G_CL = feedback(G_OL, 1.0);
auto bw = bandwidth(G_CL);
```

### Example: Buck Converter Voltage Control

```cpp
// Buck converter small-signal model
// Control-to-output: G_vd(s) = Vin * (1 - s*L/(R*D²)) / (s²LC + sL/R + 1)

double Vin = 12.0, Vout = 5.0;  // Input/output voltage
double L = 100e-6, C = 100e-6;  // Filter components
double R = 5.0;                  // Load resistance
double D = Vout/Vin;             // Duty cycle

double wLC = 1.0/sqrt(L*C);      // LC resonance
double Q = R*sqrt(C/L);          // Quality factor

TransferFunction G_vd({Vin, -Vin*L/(R*D*D)}, 
                      {1.0/(wLC*wLC), 1.0/(Q*wLC), 1.0});

// Type III compensator design for voltage mode control
// ...
```

---

## ✅ Part VIII: LIBRARY AUDIT RESULTS (January 2026)

### Audit Summary

| Category | Status | Notes |
|----------|--------|-------|
| Core Control Module | ✅ Complete | 17 header files |
| Chapter Coverage | ✅ Full | Chapters 1-14 fully supported |
| Textbook Examples | ✅ Complete | 24 example files (ch10 added) |
| EE/Telecom Support | ✅ Supported | Via TransferFunction/StateSpace |
| Documentation | ✅ Complete | API docs, chapters, appendices |

### Library Capabilities Matrix

| Feature | Module | Status |
|---------|--------|--------|
| Transfer Functions | `transfer_function.hpp` | ✅ |
| State-Space | `state_space.hpp` | ✅ |
| Bode/Nyquist/Nichols | `bode.hpp`, `nyquist.hpp`, `nichols.hpp` | ✅ |
| Root Locus | `root_locus.hpp` | ✅ |
| Pole-Zero Map | `pzmap.hpp` | ✅ |
| Discretization | `discrete.hpp` | ✅ |
| Pole Placement | `controller_design.hpp` | ✅ |
| LQR/LQG | `controller_design.hpp`, `lqg.hpp` | ✅ |
| Kalman Filter | `kalman.hpp` | ✅ |
| MPC | `mpc.hpp` | ✅ |
| Stability Margins | `analysis.hpp` | ✅ |

### Recommendations from Audit

1. **High Priority:** None - library is ready for use
2. **Medium Priority:** 
   - Standardize utility code in ch11/ch14 examples
   - Consider dedicated PLL module (optional)
3. **Low Priority:**
   - Add dq-frame transforms module
   - Add resonant (PR) controllers

---

**Document Version:** 2.0  
**Last Updated:** January 27, 2026  
**Author:** CppPlot Control Systems Project
