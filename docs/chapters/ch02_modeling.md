# Chapter 2: Mathematical Modeling of Dynamic Systems
## Modern Control Engineering with C++

---

## 🎯 Learning Objectives and Outcomes

### Chapter Objectives

This chapter develops the essential skill of translating physical systems into mathematical models suitable for control system analysis and design.

### Learning Outcomes (Bloom's Taxonomy)

| Level | Outcome | Assessment |
|-------|---------|------------|
| **Remember** | State Newton's laws, Kirchhoff's laws, energy balance equations | Quiz |
| **Understand** | Explain how physical laws lead to differential equations | Concept questions |
| **Apply** | Derive transfer functions from mechanical, electrical, and thermal systems | Problem sets |
| **Analyze** | Determine the order and type of a system from its physical description | Analysis exercises |
| **Evaluate** | Assess model accuracy by comparing simulation with physical intuition | Verification tasks |
| **Create** | Develop mathematical models for new physical systems | Modeling project |

### Prerequisites
- Chapter 1: Introduction to Control Systems
- Differential equations (first and second order)
- Basic physics (mechanics, circuits)

---

## Why This Chapter Matters

> **The Real Problem:** Your self-balancing scooter oscillates violently — riders can barely stay on. The manufacturer's simulation says it shouldn't. You measure the real system and discover a 200ms sensor delay that the model ignored. The model is *wrong*. Not because the physics is wrong, but because the model left out a physical effect that matters.
>
> Every controller in this book is designed for a *model*, not for the real system. The gap between model and reality determines whether your controller works brilliantly or fails catastrophically. This chapter teaches you to build models that are **good enough** — and, more importantly, to understand *which simplifications are dangerous*.

---

## 2.1 The Modeling Process

### From Physics to Mathematics

```
┌─────────────────┐
│ PHYSICAL SYSTEM │
│ (Real World)    │
└────────┬────────┘
         │
         ▼ Identify energy storage elements
┌─────────────────┐
│ PHYSICAL LAWS   │
│ • Newton's laws │
│ • Kirchhoff's   │
│ • Conservation  │
└────────┬────────┘
         │
         ▼ Apply constitutive relations
┌─────────────────┐
│ DIFFERENTIAL    │
│ EQUATIONS       │
└────────┬────────┘
         │
         ▼ Laplace transform
┌─────────────────┐
│ TRANSFER        │
│ FUNCTION G(s)   │
└────────┬────────┘
         │
         ▼ Or convert to
┌─────────────────┐
│ STATE-SPACE     │
│ ẋ = Ax + Bu     │
│ y = Cx + Du     │
└─────────────────┘
```

### System Order

The **order** of a system equals the number of independent energy storage elements:
- Mechanical: mass (kinetic), spring (potential)
- Electrical: inductor (magnetic), capacitor (electric)
- Thermal: thermal capacitance
- Hydraulic: fluid inertia, compressibility

---

## 2.2 The Art of Abstraction: How Real Objects Become Model Elements

> **The question no textbook answers:** You see a tire. The textbook shows a mass-spring-damper. *How did someone decide that a tire is a spring and a damper?* You see a car turning on a road. The textbook shows a point mass, or a disk. *What reasoning process transforms a 1,500 kg automobile into a circle with mass $m$ and radius $R$?*
>
> This section teaches **abstraction** — the most important and least taught skill in mathematical modeling.

### 2.2.1 What Is Abstraction?

Abstraction is the deliberate act of **replacing a real object with a simpler object that preserves the behaviors you care about and discards the ones you don't.**

This is NOT simplification by accident. It is simplification by design — and the choice of *what to keep* and *what to discard* is the core engineering decision.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                     THE ABSTRACTION PROCESS                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  STEP 1: Identify the QUESTION you want the model to answer                │
│  ─────────────────────────────────────────────────────────                  │
│  "How does the car respond to bumps?" (vertical dynamics)                  │
│  "How does the car turn?" (lateral/yaw dynamics)                           │
│  "How fast does the car accelerate?" (longitudinal dynamics)               │
│                                                                             │
│  STEP 2: Identify the ENERGY MECHANISMS relevant to that question          │
│  ──────────────────────────────────────────────────────────────             │
│  Vertical: tire stores elastic energy + dissipates via damping            │
│  Lateral:  tire generates side force via slip angle                        │
│  Longitudinal: engine torque → tire traction force                        │
│                                                                             │
│  STEP 3: Map each mechanism to the SIMPLEST element that reproduces it     │
│  ─────────────────────────────────────────────────────────────────          │
│  Elastic energy storage → spring (k)                                       │
│  Energy dissipation → damper (b)                                           │
│  Inertia → mass (m)                                                        │
│                                                                             │
│  STEP 4: Assemble the elements and VERIFY the abstraction                  │
│  ────────────────────────────────────────────────────────                   │
│  Does the simplified model's behavior match reality for YOUR question?     │
│  If yes → abstraction is valid for this purpose.                           │
│  If no → you discarded something you shouldn't have. Go to Step 2.        │
│                                                                             │
│  HOW TO VERIFY (concrete criteria):                                        │
│  ┌───────────────────────────────────────────────────────────────────┐     │
│  │ a. Compare STEP RESPONSE: Does the model's rise time, overshoot,│     │
│  │    and settling time match measured data within ±10–20%?         │     │
│  │ b. Compare FREQUENCY RESPONSE (Bode plot): Does the model's     │     │
│  │    gain and phase match measurements in the frequency range      │     │
│  │    where the controller will operate?                            │     │
│  │ c. Check STEADY-STATE GAIN: Does DC gain match within ±5%?      │     │
│  │ d. Check DOMINANT DYNAMICS: Are the number of oscillation modes  │     │
│  │    and their approximate frequencies correct?                    │     │
│  │ e. If no measured data exists: compare with a HIGHER-ORDER model │     │
│  │    (Level 1) — does reducing order lose the dominant behavior?   │     │
│  └───────────────────────────────────────────────────────────────────┘     │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2.2 Example 1: How a Tire Becomes a Spring-Damper

A tire is a complex composite object: rubber, cord, air, tread pattern, steel belts. But the **question** determines the model.

#### Question: "How does the tire transmit road bumps to the car body?"

This is a **vertical dynamics** question. What matters is:
1. The tire **deforms** when it hits a bump → it stores elastic energy → **spring**
2. The rubber **dissipates** energy during deformation → **damper**
3. The tire wall has **mass** → but it's small compared to the car → **often neglected**

```
    REAL TIRE                          ABSTRACTED MODEL
    ─────────                          ─────────────────
                                       
    ┌───────────────┐                        │
    │ Steel belts   │                        │  Car body (m_b)
    │ Rubber layers │                        │
    │ Air pressure  │                     ┌──┴──┐
    │ Tread pattern │     ──────►         │ k_s │ Spring (suspension)
    │ Cord structure│                     │ b_s │ Damper (shock absorber)
    │ Sidewall flex │                     └──┬──┘
    └───────────────┘                        │  Wheel mass (m_w)
         on road                          ┌──┴──┐
                                          │ k_t │ Spring (tire stiffness)
                                          │ b_t │ Damper (tire damping)
                                          └──┬──┘
                                             │
                                          ═══════  Road surface
```

**The reasoning chain:**

| Real property | Physically means... | Maps to... | Parameter |
|---------------|---------------------|------------|-----------|
| Air pressure + rubber elasticity | Tire resists deformation and returns to shape | Spring | $k_t \approx 150{,}000$ – $250{,}000$ N/m |
| Internal friction in rubber | Energy lost during each deformation cycle | Damper | $b_t \approx 100$ – $500$ N·s/m |
| Tread pattern | Grip on wet/dry surfaces | *Not relevant* for vertical dynamics → **discarded** |
| Steel belts | Distribute force, resist puncture | *Not relevant* → **discarded** |
| Tire mass | Inertia of the unsprung mass | Mass | $m_w \approx 30$ – $50$ kg (wheel + tire) |

> **This is the key insight:** The tread pattern, steel belts, and cord structure are *real* — they exist physically. But they don't participate in the *vertical energy exchange* that answers our question. Abstraction means having the **judgment** to know what to discard.

#### What if the question were different?

| Question | Relevant physics | Tire model |
|----------|------------------|------------|
| Vertical ride comfort | Stiffness + damping | Spring-damper ($k_t, b_t$) |
| Braking/acceleration | Longitudinal friction | Pacejka "Magic Formula": $F_x = f(\text{slip ratio})$ |
| Cornering | Lateral force vs. slip angle | Linear tire: $F_y = C_\alpha \cdot \alpha$ |
| Tire noise (NVH) | Tread vibration modes | FEA model with 10,000+ elements |
| Tire wear | Rubber chemistry + contact pressure | Material science model — NOT a control model |

**The same physical object (tire) becomes five completely different mathematical models depending on the question.** This is abstraction.

### 2.2.3 Example 2: How a Car Becomes a Rotating Disk

In yaw dynamics and lateral control, a car is often modeled as a **rigid disk** or **bicycle model**. This seems outrageous — a car has doors, seats, an engine, passengers. How can we replace it with a disk?

#### Question: "How does the car rotate when the driver turns the steering wheel?"

What matters for yaw rotation:
1. The car has **mass** $m$ → resists translational acceleration → **point mass**
2. The car has a **moment of inertia** $I_z$ about the vertical axis → resists rotation → **disk**
3. The tires generate **lateral forces** at front and rear → create a **torque** about the center of gravity
4. The shape of the car (doors, mirrors, trunk) does NOT affect yaw dynamics at moderate speeds → **discarded**

```
         REAL CAR                              ABSTRACTED MODEL
         ────────                              ────────────────

    ┌─────────────────┐                     Front axle
    │    ┌───┐        │                         │
    │    │   │ Engine  │                         │  l_f
    │    └───┘        │                         │
    │  ┌─────────┐   │                    ──────●────── CG (mass m, inertia I_z)
    │  │ Cabin   │   │    ──────►               │
    │  │ Seats   │   │                          │  l_r
    │  │ Humans  │   │                          │
    │  └─────────┘   │                     Rear axle
    │    ┌───┐        │
    │    │   │ Trunk   │
    │    └───┘        │
    └─────────────────┘
```

**The reasoning chain:**

| Real component | Relevant to yaw dynamics? | Maps to... |
|----------------|---------------------------|------------|
| Total vehicle mass | ✅ Yes — resists lateral acceleration | Point mass $m$ |
| Mass distribution (front/rear) | ✅ Yes — determines $l_f$, $l_r$ and $I_z$ | Distances from CG to axles |
| Moment of inertia about z-axis | ✅ Yes — resists yaw rotation | $I_z$ of a disk (or measured directly) |
| Tire lateral forces | ✅ Yes — these CREATE the yaw torque | $F_{yf} = C_f \alpha_f$, $F_{yr} = C_r \alpha_r$ |
| Engine, seats, passengers | ❌ No — contribute only to $m$ and $I_z$ total | **Lumped** into $m$ and $I_z$ |
| Body shape (aerodynamics) | ❌ At moderate speeds | **Discarded** (add back at >100 km/h) |
| Suspension compliance | ❌ For basic lateral model | **Discarded** (add back for precision) |

The result is the **bicycle model** — one of the most used abstractions in vehicle dynamics:

$$m(\dot{v}_y + v_x \dot{\psi}) = F_{yf} + F_{yr}$$
$$I_z \ddot{\psi} = l_f F_{yf} - l_r F_{yr}$$

where $v_y$ is lateral velocity, $\psi$ is yaw angle, and $F_{yf}, F_{yr}$ are front/rear lateral tire forces.

> Two equations. The entire car — engine, seats, passengers, trunk — reduced to $m$, $I_z$, $l_f$, $l_r$. Because for the question "how does it turn?", nothing else matters.

### 2.2.4 Example 3: How a Building Becomes a Single Thermal Capacitance

A building has walls (concrete, insulation, brick), windows (glass, air gaps), furniture (wood, fabric), air (convective currents), a roof, a foundation. Yet for HVAC control, it often becomes:

$$C \frac{dT}{dt} = Q_{in} - \frac{T - T_{out}}{R}$$

One capacitance $C$, one resistance $R$, one temperature $T$.

| Real component | Abstraction | Why |
|----------------|-------------|-----|
| Air volume | Part of $C$ | Stores thermal energy ($\rho c_p V$) |
| Furniture + walls | Part of $C$ | Also store thermal energy (large thermal mass) |
| Wall thickness + insulation | $R$ | Resists heat flow (Fourier's law: $R = L/kA$) |
| Windows | Different $R$ (lower) | Parallel thermal resistance |
| Convective air currents | *Lumped* into $R$ | Effective heat transfer coefficient |
| Individual room temperatures | *Averaged* into single $T$ | Single-zone assumption |
| Furniture arrangement | **Discarded** | Affects air flow patterns, not bulk energy balance |

> **The abstraction is valid when** the time scale of internal mixing (minutes) is much shorter than the time scale of heat loss through walls (hours). If rooms have very different temperatures (e.g., sunlit vs. shaded), the single-zone model fails and you need multi-zone ($T_1, T_2, \ldots$).

### 2.2.5 Example 4: How an Op-Amp Circuit Becomes a First-Order Transfer Function

An operational amplifier IC contains ~20 transistors, bias networks, a compensation capacitor, current mirrors, and output stage protection. Yet in control systems, the entire circuit is modeled as:

$$G(s) = \frac{A_0}{\tau s + 1}$$

One gain $A_0$, one time constant $\tau$. How?

#### Question: "How does the amplifier respond to signals below 1 MHz?"

| Real component | Relevant below 1 MHz? | Maps to... |
|----------------|------------------------|------------|
| Open-loop DC gain | ✅ Yes — determines accuracy | $A_0 \approx 10^5$ – $10^6$ |
| Dominant compensation pole | ✅ Yes — sets bandwidth | Time constant $\tau = 1/(2\pi f_p)$, $f_p \approx 5$ – $10$ Hz |
| Input bias current | ❌ negligible with feedback | **Discarded** |
| Output current limit | ❌ unless saturated | **Discarded** (add back for large signals) |
| Higher-order poles (>10 MHz) | ❌ far above operating range | **Discarded** |
| Transistor-level parasitics | ❌ absorbed into $A_0$ and $\tau$ | **Lumped** |
| Slew rate | ❌ for small signals | **Discarded** (add back for large-signal analysis) |

```
    REAL OP-AMP (LM741 die)                    ABSTRACTED MODEL
    ───────────────────────                    ─────────────────

    ┌─────────────────────┐                         V_in
    │ Q1-Q4: Diff pair    │                          │
    │ Q5-Q8: Current mirr. │                     ┌───┴───┐
    │ Q9-Q12: Gain stage   │    ──────►          │  A₀   │
    │ C_comp: 30 pF        │                     │ ───── │
    │ Q13-Q20: Output      │                     │ τs+1  │
    │ R_bias network       │                     └───┬───┘
    └─────────────────────┘                          │
                                                    V_out
```

> **The key insight:** 20 transistors, a capacitor, and multiple bias resistors → ONE pole. This works because the 30 pF compensation capacitor was *deliberately designed* by the IC engineer to make one pole dominate all others below 1 MHz. The abstraction $A_0/(\tau s + 1)$ is valid **because the IC was designed to make it valid** — a beautiful example of design enabling abstraction.

> **When it fails:** At frequencies above ~10 MHz, the higher-order poles (from transistor parasitics) become significant. The Bode plot deviates from the single-pole –20 dB/decade slope. If your control loop has high bandwidth, you need a multi-pole model — the single-pole abstraction was too aggressive.

### 2.2.6 The Abstraction Decision Framework

Every abstraction follows the same logic:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    THE ABSTRACTION DECISION FRAMEWORK                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  For each real physical phenomenon, ask:                                   │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │ 1. Does this phenomenon STORE energy relevant to my question?      │   │
│  │    YES → model as MASS (kinetic), SPRING (potential),              │   │
│  │          CAPACITOR (electric), THERMAL MASS (thermal)              │   │
│  │    NO  → go to 2                                                   │   │
│  ├─────────────────────────────────────────────────────────────────────┤   │
│  │ 2. Does this phenomenon DISSIPATE energy relevant to my question?  │   │
│  │    YES → model as DAMPER, RESISTOR, THERMAL RESISTANCE             │   │
│  │    NO  → go to 3                                                   │   │
│  ├─────────────────────────────────────────────────────────────────────┤   │
│  │ 3. Does this phenomenon TRANSFORM energy between domains?          │   │
│  │    YES → model as TRANSDUCER (motor, generator, piezo, thermocouple│   │
│  │    NO  → go to 4                                                   │   │
│  ├─────────────────────────────────────────────────────────────────────┤   │
│  │ 4. Does this phenomenon affect the GEOMETRY or CONSTRAINTS?        │   │
│  │    YES → model as CONSTRAINT (lever ratio, gear ratio, boundary)   │   │
│  │    NO  → DISCARD IT. It does not participate in the dynamics.      │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  The number of energy storage elements you keep = the ORDER of your model. │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2.7 Abstraction and the Universal Element Table

The power of abstraction lies in the fact that **every physical domain uses the same three building blocks**: something that stores potential energy, something that stores kinetic energy, and something that dissipates.

| Role | Mechanical (trans.) | Mechanical (rot.) | Electrical | Thermal | Hydraulic |
|------|--------------------|--------------------|------------|---------|-----------|
| **Stores potential energy** | Spring $k$ | Torsional spring $k_t$ | Capacitor $C$ | — | Fluid compressibility |
| **Stores kinetic energy** | Mass $m$ | Inertia $J$ | Inductor $L$ | — | Fluid inertia |
| **Dissipates energy** | Damper $b$ | Rotational damper $b_r$ | Resistor $R$ | Thermal resistance $R_{th}$ | Fluid resistance |
| **Stores thermal energy** | — | — | — | Thermal capacitance $C_{th}$ | — |

> **This is why the mechanical-electrical analogy works** (§2.4.4): a tire (spring-damper) and an RC circuit (resistor-capacitor) obey the *same* differential equation because both involve one energy storage element and one dissipation element. The abstraction to $m$-$k$-$b$ or $R$-$L$-$C$ is not a mathematical trick — it is a reflection of the **universality of energy physics**.

### ⚠️ 2.2.8 When Abstraction FAILS: A Cautionary Tale

> **The lesson abstraction teaches last — and hardest — is what happens when you discard the wrong thing.**

#### Case Study: The Millennium Bridge, London (2000)

When the Millennium Bridge opened on June 10, 2000, thousands of pedestrians crossed it. Within minutes, the bridge began **swaying laterally** with alarming amplitude. It was closed after two days.

**What the engineers modeled:** The bridge was designed to withstand vertical loads (pedestrian weight → static and vertical dynamic forces). The abstraction treated pedestrians as **vertical point loads** — masses that push DOWN on the deck.

**What they discarded:** Pedestrians also push **laterally** as they walk — a small side-to-side force (~25 N per step). This was deemed negligible compared to the vertical load (~700 N per person).

**Why the abstraction failed:** The lateral wobble, once started, caused pedestrians to unconsciously **synchronize their steps** to maintain balance. This created a positive feedback loop:

```
┌────────────────────────────────────────────────────────────────────────┐
│                                                                        │
│   Small lateral       Pedestrians sync      Larger lateral             │
│   bridge vibration ──► steps to bridge  ──► force on bridge ──┐       │
│         ▲                                                      │       │
│         │                                                      │       │
│         └──────────────────────────────────────────────────────┘       │
│                     POSITIVE FEEDBACK LOOP                             │
│                                                                        │
│   Abstraction error: pedestrians modeled as PASSIVE loads              │
│   Reality: pedestrians are ACTIVE agents that adapt to motion          │
│                                                                        │
└────────────────────────────────────────────────────────────────────────┘
```

**Using the Decision Framework (§2.2.6):**

| Phenomenon | Framework question | Engineer's answer | Correct answer |
|------------|-------------------|-------------------|----------------|
| Pedestrian lateral force | Stores/dissipates energy? | "No — too small" | **YES** — creates positive feedback that *adds* energy |
| Step synchronization | Affects dynamics? | *Not considered* | **YES** — it's a nonlinear coupling: $F_{lateral} = f(\dot{x}_{bridge}, N_{pedestrians})$ |
| Pedestrian adaptation | Passive or active? | "Passive load" | **ACTIVE** — humans change behavior in response to bridge motion |

**The fix:** Viscous dampers were installed (adding energy dissipation $b\dot{x}$ to prevent the lateral resonance from growing). Cost: £5 million. The bridge was closed for nearly two years.

> **The abstraction lesson:** The engineers had the energy framework right — they just applied it to the wrong direction. Lateral energy input was small *per person* but grew via synchronization. **An abstraction is only as good as its worst omission.** Before finalizing a model, always ask: *"What have I discarded, and under what conditions could it become dominant?"*

> **→ Connection to Step 4 (Verification):** This is exactly why Step 4 exists. If the bridge engineers had measured the lateral frequency response with pedestrians on a shaker test, they would have seen the anomalous gain growth. Verification is not optional — it is the final defense against abstraction failure.

### 🛑 Stop and Think

> 1. A washing machine vibrates during the spin cycle. You want to model the vibration. What is the "question"? Which physical properties of the machine matter (drum mass, motor torque, unbalanced load)? Which do NOT matter (color, brand, door handle shape)?
>
> 2. A smartphone drops 1.5 m onto concrete. You want to model the impact to predict screen breakage. Is the phone a point mass? A spring-mass? A rigid body? An elastic body? The answer depends on **what duration of impact** you care about. Explain why.
>
> 3. For the bicycle model of a car: at what speed does the "discard aerodynamics" assumption break down? How would you know? What would you add back to the model?
>
> 4. **The meta-question:** Two engineers model the same electric motor. One uses a 1st-order model $G(s) = K_m/(\tau s + 1)$; the other uses a 5th-order model with thermal dynamics, magnetic saturation, and friction nonlinearity. Neither model is "wrong." What determines which is *appropriate*? (Hint: what question is each trying to answer? See §2.8.7 "Three Levels of Models.")
>
> 5. **The Millennium Bridge revisited (§2.2.8):** If you were hired to model the bridge *before* opening day, what experiment would you propose to validate the pedestrian load abstraction? How would you design a test that reveals whether pedestrians should be modeled as passive loads or active agents?

> **→ Connection to §2.8.7 (Three Levels of Models):** Abstraction is *how* you move between levels. The truth model (Level 1) includes every phenomenon. The design model (Level 2) is the result of deliberate abstraction — keeping only what the controller needs to see. The art of abstraction is choosing Level 2 wisely: keep too much and the model is intractable; discard too much and the controller fails on the real system (Level 3).

---

## 2.3 Mechanical Systems

### 2.3.1 Translational Systems

**Fundamental Elements:**

| Element | Constitutive Relation | Energy Storage |
|---------|----------------------|----------------|
| Mass | $F = m\ddot{x}$ | Kinetic: $\frac{1}{2}mv^2$ |
| Spring | $F = kx$ | Potential: $\frac{1}{2}kx^2$ |
| Damper | $F = b\dot{x}$ | Dissipates energy |

### 2.3.2 Example: Mass-Spring-Damper System

**Physical System:**

```
         x(t) displacement
           │
    ┌──────┴──────┐
    │     Mass    │◀─────── F(t) applied force
    │      M      │
    └──────┬──────┘
           │
       ════╪════  Spring (stiffness k)
           │      F_spring = k·x
       ────╫────  Damper (coefficient b)
           │      F_damper = b·ẋ
       ────┴────  Fixed wall
```

**Free Body Diagram:**
```
        F(t)
          │
          ▼
    ┌───────────┐
    │     M     │───▶ ẍ (acceleration)
    └───────────┘
          ▲
          │
     F_spring + F_damper
        (opposing)
```

**Signal Dictionary — Mass-Spring-Damper System**

| Signal | Symbol | Unit | Physical Meaning | Sensor/Actuator |
|--------|--------|------|------------------|-----------------|
| Applied force (input) | $F(t)$ | N | External force pushing the mass | Electromagnetic shaker / pneumatic actuator |
| Displacement (output) | $x(t)$ | m | Position of mass relative to equilibrium | LVDT / laser displacement sensor |
| Velocity | $\dot{x}(t)$ | m/s | Rate of change of position | Accelerometer (integrated) / laser vibrometer |
| Spring force | $kx$ | N | Restoring force — proportional to displacement | (Internal system force) |
| Damping force | $b\dot{x}$ | N | Resistive force — proportional to velocity | (Internal system force) |
| Mass | $m$ | kg | Inertia of the moving body | (System parameter) |
| Spring stiffness | $k$ | N/m | How much force per unit displacement | (System parameter) |
| Damping coefficient | $b$ | N·s/m | How much force per unit velocity | (System parameter) |

> **Reading this table:** Notice the three physical parameters ($m$, $k$, $b$) map directly to the three terms in the ODE. Each parameter has a clear unit and meaning — this is what makes the model *physical*, not just mathematical.

**Newton's Second Law:** $\sum F = ma$

$$F(t) - kx - b\dot{x} = m\ddot{x}$$

**Standard Form:**
$$m\ddot{x} + b\dot{x} + kx = F(t)$$

**Laplace Transform (zero initial conditions):**
$$ms^2X(s) + bsX(s) + kX(s) = F(s)$$

**Transfer Function:**
$$G(s) = \frac{X(s)}{F(s)} = \frac{1}{ms^2 + bs + k}$$

### 2.3.3 CppPlot Implementation: Mass-Spring-Damper

```cpp
/**
 * @file ch02_mass_spring_damper.cpp
 * @brief Modeling and simulation of mass-spring-damper system
 * 
 * Physical System → Mathematical Model → Simulation
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <cmath>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║     Chapter 2: Mass-Spring-Damper System Modeling            ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // ========================================================================
    // Physical Parameters
    // ========================================================================
    double m = 1.0;     // Mass [kg]
    double b = 0.5;     // Damping coefficient [N·s/m]
    double k = 4.0;     // Spring constant [N/m]
    
    std::cout << "\nPhysical Parameters:" << std::endl;
    std::cout << "  Mass m = " << m << " kg" << std::endl;
    std::cout << "  Damping b = " << b << " N·s/m" << std::endl;
    std::cout << "  Spring k = " << k << " N/m" << std::endl;
    
    // ========================================================================
    // Derived Parameters
    // ========================================================================
    double wn = std::sqrt(k / m);               // Natural frequency [rad/s]
    double zeta = b / (2 * std::sqrt(k * m));   // Damping ratio
    double wd = wn * std::sqrt(1 - zeta*zeta);  // Damped frequency
    
    std::cout << "\nDerived Parameters:" << std::endl;
    std::cout << "  Natural frequency ωn = " << wn << " rad/s" << std::endl;
    std::cout << "  Damping ratio ζ = " << zeta << std::endl;
    std::cout << "  Damped frequency ωd = " << wd << " rad/s" << std::endl;
    
    // Classify response
    std::string response_type;
    if (zeta < 1) response_type = "Underdamped";
    else if (zeta == 1) response_type = "Critically damped";
    else response_type = "Overdamped";
    std::cout << "  Response type: " << response_type << std::endl;
    
    // ========================================================================
    // Transfer Function: G(s) = 1/(ms² + bs + k)
    // ========================================================================
    TransferFunction G({1}, {m, b, k});
    
    std::cout << "\nTransfer Function: G(s) = 1/(" << m << "s² + " << b << "s + " << k << ")" << std::endl;
    
    // Poles
    auto poles = G.poles();
    std::cout << "\nPoles:" << std::endl;
    for (auto& p : poles) {
        std::cout << "  s = " << p.real();
        if (std::abs(p.imag()) > 1e-6) {
            std::cout << " + j" << p.imag();
        }
        std::cout << std::endl;
    }
    
    // ========================================================================
    // Simulation: Step Response (F = 1 N step force)
    // ========================================================================
    double t_final = 15.0;
    auto [t, x] = step_data(G, t_final);
    
    // ========================================================================
    // Different Damping Scenarios
    // ========================================================================
    std::vector<double> damping_values = {0.1, 0.3, 0.5, 0.707, 1.0, 2.0};
    
    figure(1200, 800);
    
    // Subplot 1: Step responses for different damping
    subplot(2, 2, 1);
    
    std::vector<std::string> colors = {"red", "orange", "gold", "green", "blue", "purple"};
    
    for (size_t i = 0; i < damping_values.size(); ++i) {
        double b_i = 2 * damping_values[i] * std::sqrt(k * m);
        TransferFunction G_i({1}, {m, b_i, k});
        auto [t_i, x_i] = step_data(G_i, t_final);
        
        std::string label = "ζ = " + std::to_string(damping_values[i]).substr(0, 5);
        plot(t_i, x_i, "-", {{"color", colors[i]}, {"linewidth", "2"}, {"label", label}});
    }
    
    axhline(1.0/k, {{"color", "black"}, {"linestyle", "--"}, {"alpha", "0.5"}});
    xlabel("Time [s]");
    ylabel("Displacement x [m]");
    title("Step Response: Effect of Damping Ratio");
    legend();
    grid(true);
    
    // Subplot 2: Pole locations
    subplot(2, 2, 2);
    
    // Draw unit circle scaled by wn
    std::vector<double> circle_x, circle_y;
    for (int i = 0; i <= 100; ++i) {
        double angle = M_PI/2 + i * M_PI / 100;
        circle_x.push_back(wn * std::cos(angle));
        circle_y.push_back(wn * std::sin(angle));
    }
    plot(circle_x, circle_y, "k--", {{"alpha", "0.3"}});
    
    for (size_t i = 0; i < damping_values.size(); ++i) {
        double b_i = 2 * damping_values[i] * std::sqrt(k * m);
        TransferFunction G_i({1}, {m, b_i, k});
        auto poles_i = G_i.poles();
        
        std::vector<double> re, im;
        for (auto& p : poles_i) {
            re.push_back(p.real());
            im.push_back(p.imag());
        }
        
        scatter(re, im, {{"color", colors[i]}, {"markersize", "10"}});
    }
    
    axhline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    axvline(0, {{"color", "black"}, {"linewidth", "0.5"}});
    xlabel("Real");
    ylabel("Imaginary");
    title("Pole Locations in s-plane");
    grid(true);
    xlim(-5, 1);
    ylim(-3, 3);
    
    // Subplot 3: Impulse response
    subplot(2, 2, 3);
    
    auto [t_imp, x_imp] = impulse_data(G, t_final);
    plot(t_imp, x_imp, "b-", {{"linewidth", "2"}});
    xlabel("Time [s]");
    ylabel("Displacement [m]");
    title("Impulse Response (ζ = 0.125)");
    grid(true);
    
    // Subplot 4: Free vibration (initial displacement)
    subplot(2, 2, 4);
    
    // Analytical solution for underdamped free vibration
    // x(t) = x0 * e^(-ζωn*t) * [cos(ωd*t) + (ζωn/ωd)*sin(ωd*t)]
    double x0 = 1.0;  // Initial displacement
    std::vector<double> t_free, x_free, envelope_pos, envelope_neg;
    
    for (double ti = 0; ti <= t_final; ti += 0.05) {
        t_free.push_back(ti);
        double decay = std::exp(-zeta * wn * ti);
        x_free.push_back(x0 * decay * (std::cos(wd * ti) + (zeta * wn / wd) * std::sin(wd * ti)));
        envelope_pos.push_back(x0 * decay);
        envelope_neg.push_back(-x0 * decay);
    }
    
    plot(t_free, x_free, "b-", {{"linewidth", "2"}, {"label", "Free vibration"}});
    plot(t_free, envelope_pos, "r--", {{"linewidth", "1"}, {"label", "Envelope"}});
    plot(t_free, envelope_neg, "r--", {{"linewidth", "1"}});
    
    xlabel("Time [s]");
    ylabel("Displacement x [m]");
    title("Free Vibration from x(0) = 1 m");
    legend();
    grid(true);
    
    savefig("ch02_mass_spring_damper.svg");
    std::cout << "\n✓ Saved ch02_mass_spring_damper.svg" << std::endl;
    
    // ========================================================================
    // Physical Interpretation Summary
    // ========================================================================
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║               PHYSICAL INTERPRETATION                        ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Parameter Effect:                                           ║" << std::endl;
    std::cout << "║  • ↑ Mass m    → ↓ ωn (slower response)                      ║" << std::endl;
    std::cout << "║  • ↑ Spring k  → ↑ ωn (faster oscillation)                   ║" << std::endl;
    std::cout << "║  • ↑ Damping b → ↑ ζ  (less overshoot)                       ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Response Types:                                             ║" << std::endl;
    std::cout << "║  • ζ < 1: Underdamped - oscillates, decays                   ║" << std::endl;
    std::cout << "║  • ζ = 1: Critical - fastest non-oscillatory                 ║" << std::endl;
    std::cout << "║  • ζ > 1: Overdamped - slow, no oscillation                  ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    return 0;
}
```

> **🔍 Stop and Think — Mechanism Questions**
>
> Before moving on, answer these without looking up formulas:
> 1. The simulation shows an underdamped oscillatory response. *Which physical parameter* ($m$, $b$, or $k$) is primarily responsible for the oscillation, and which one causes it to eventually die out?
> 2. If you double the mass $m$, what happens to the natural frequency $\omega_n = \sqrt{k/m}$? Does the system oscillate faster or slower? Why does this make physical sense?
> 3. The transfer function $G(s) = 1/(ms^2 + bs + k)$ has units of m/N. Verify this from the physical parameters. What would it mean if the units did not match?

---

## 2.4 Electrical Systems

### 2.4.1 Fundamental Elements

| Element | Constitutive Relation | Energy Storage |
|---------|----------------------|----------------|
| Resistor | $v = Ri$ | Dissipates |
| Capacitor | $i = C\frac{dv}{dt}$ | Electric: $\frac{1}{2}Cv^2$ |
| Inductor | $v = L\frac{di}{dt}$ | Magnetic: $\frac{1}{2}Li^2$ |

### 2.4.2 Kirchhoff's Laws

**KVL (Voltage Law):** Sum of voltages around any closed loop = 0
**KCL (Current Law):** Sum of currents at any node = 0

### 2.4.3 Example: Series RLC Circuit

**Physical System:**

```
         V_in(t)
           │
           ▼
    ┌──────┴──────┐
    │             │
   ═╪═            │
    R             │  i(t)
   ═╪═            │   →
    │             │
    └──[L]────────┤
                  │
                 ═╪═
                  C   V_out(t)
                 ═╪═
                  │
    ──────────────┴──────────── GND
```

**KVL around the loop:**
$$V_{in} = V_R + V_L + V_C$$
$$V_{in} = Ri + L\frac{di}{dt} + \frac{1}{C}\int i \, dt$$

**In terms of capacitor voltage $V_C$ (output):**

Since $i = C\frac{dV_C}{dt}$:

$$V_{in} = RC\frac{dV_C}{dt} + LC\frac{d^2V_C}{dt^2} + V_C$$

**Transfer Function:**
$$G(s) = \frac{V_{out}(s)}{V_{in}(s)} = \frac{1/LC}{s^2 + \frac{R}{L}s + \frac{1}{LC}}$$

### 2.4.4 Mechanical-Electrical Analogy

> **→ Connection to §2.2 (The Art of Abstraction):** The analogy below works precisely *because* the abstraction process in §2.2 reduces both mechanical and electrical systems to the same three roles — energy storage (potential), energy storage (kinetic), and dissipation. A tire abstracted to $k$-$b$ and an RC circuit abstracted to $R$-$C$ obey the same equation for the same reason.

| Mechanical | Electrical | Role |
|------------|------------|------|
| Force $F$ | Voltage $V$ | Effort |
| Velocity $v$ | Current $i$ | Flow |
| Mass $m$ | Inductance $L$ | Inertia |
| Damping $b$ | Resistance $R$ | Dissipation |
| Spring $k$ | $1/C$ | Compliance |
| Displacement $x$ | Charge $q$ | Stored |

Both systems: $ms^2 + bs + k$ ↔ $Ls^2 + Rs + 1/C$

### 2.4.5 CppPlot Implementation: RLC Circuit

```cpp
/**
 * @file ch02_rlc_circuit.cpp
 * @brief RLC circuit modeling and electrical-mechanical analogy
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <cmath>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║         Chapter 2: RLC Circuit Modeling                      ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // ========================================================================
    // RLC Circuit Parameters
    // ========================================================================
    double R = 10;       // Resistance [Ω]
    double L = 0.1;      // Inductance [H]
    double C = 100e-6;   // Capacitance [F] = 100 μF
    
    std::cout << "\nRLC Circuit Parameters:" << std::endl;
    std::cout << "  R = " << R << " Ω" << std::endl;
    std::cout << "  L = " << L*1000 << " mH" << std::endl;
    std::cout << "  C = " << C*1e6 << " μF" << std::endl;
    
    // Derived parameters
    double wn = 1.0 / std::sqrt(L * C);      // Natural frequency
    double zeta = R / (2 * std::sqrt(L / C)); // Damping ratio
    double f_resonance = wn / (2 * M_PI);     // Resonance frequency [Hz]
    
    std::cout << "\nDerived Parameters:" << std::endl;
    std::cout << "  ωn = " << wn << " rad/s" << std::endl;
    std::cout << "  ζ = " << zeta << std::endl;
    std::cout << "  Resonance frequency = " << f_resonance << " Hz" << std::endl;
    
    // ========================================================================
    // Transfer Function: Vout/Vin = (1/LC)/(s² + (R/L)s + 1/LC)
    // ========================================================================
    TransferFunction G_rlc({1/(L*C)}, {1, R/L, 1/(L*C)});
    
    // ========================================================================
    // Equivalent Mechanical System
    // ========================================================================
    // Using analogy: m↔L, b↔R, k↔1/C
    double m_eq = L;
    double b_eq = R;
    double k_eq = 1/C;
    
    TransferFunction G_mech({1}, {m_eq, b_eq, k_eq});
    
    std::cout << "\nEquivalent Mechanical System:" << std::endl;
    std::cout << "  m_eq = " << m_eq << " kg" << std::endl;
    std::cout << "  b_eq = " << b_eq << " N·s/m" << std::endl;
    std::cout << "  k_eq = " << k_eq << " N/m" << std::endl;
    
    // ========================================================================
    // Simulation
    // ========================================================================
    double t_final = 0.05;  // 50 ms
    auto [t, v_out] = step_data(G_rlc, t_final);
    
    figure(1200, 600);
    
    // Step response
    subplot(1, 2, 1);
    plot(t, v_out, "b-", {{"linewidth", "2"}, {"label", "Capacitor Voltage"}});
    axhline(1.0, {{"color", "green"}, {"linestyle", "--"}, {"label", "Input (1V step)"}});
    xlabel("Time [s]");
    ylabel("Voltage [V]");
    title("RLC Circuit Step Response");
    legend();
    grid(true);
    
    // Bode magnitude
    subplot(1, 2, 2);
    
    // Generate frequency response data
    std::vector<double> freq, mag_db;
    for (double f = 10; f <= 10000; f *= 1.1) {
        double w = 2 * M_PI * f;
        std::complex<double> s(0, w);
        std::complex<double> H = (1.0/(L*C)) / (s*s + (R/L)*s + 1.0/(L*C));
        double mag = 20 * std::log10(std::abs(H));
        freq.push_back(f);
        mag_db.push_back(mag);
    }
    
    plot(freq, mag_db, "b-", {{"linewidth", "2"}});
    axvline(f_resonance, {{"color", "red"}, {"linestyle", "--"}, {"label", "Resonance"}});
    xlabel("Frequency [Hz]");
    ylabel("Magnitude [dB]");
    title("Frequency Response (Bode Magnitude)");
    legend();
    grid(true);
    xscale("log");
    
    savefig("ch02_rlc_circuit.svg");
    std::cout << "\n✓ Saved ch02_rlc_circuit.svg" << std::endl;
    
    return 0;
}
```

---

## 2.5 Electromechanical Systems: DC Motor

### 2.5.1 Physical Description

A DC motor converts electrical energy to mechanical energy through electromagnetic interaction.

```
    ┌────────────────────────────────────────────────────────┐
    │                    DC MOTOR                            │
    │                                                        │
    │   V(t) ──┬──[Ra]──[La]──┬──▶  ┌─────┐                 │
    │   input  │              │     │Motor│  ω(t) speed     │
    │          │             e_b    │     │──────▶          │
    │          │            (back   └──┬──┘  θ(t) position  │
    │          │             EMF)      │                     │
    │          └───────────────────────┘                     │
    │                                                        │
    │   Electrical:  V = La(di/dt) + Ra·i + Kb·ω            │
    │   Mechanical:  J(dω/dt) + b·ω = Kt·i                  │
    └────────────────────────────────────────────────────────┘
```

### 2.5.2 Governing Equations

**Electrical Subsystem (Kirchhoff's Voltage Law):**
$$V(t) = L_a\frac{di_a}{dt} + R_a i_a + e_b$$

where back-EMF: $e_b = K_b \omega$

**Mechanical Subsystem (Newton's Law for Rotation):**
$$J\frac{d\omega}{dt} + b\omega = \tau_m = K_t i_a$$

**Parameters:**
| Symbol | Description | Typical Unit |
|--------|-------------|--------------|
| $R_a$ | Armature resistance | Ω |
| $L_a$ | Armature inductance | H |
| $J$ | Rotor inertia | kg·m² |
| $b$ | Viscous friction | N·m·s |
| $K_t$ | Torque constant | N·m/A |
| $K_b$ | Back-EMF constant | V·s/rad |

Note: For ideal motor, $K_t = K_b$ (same physical constant in SI units)

### 2.5.3 Transfer Function Derivation

**Laplace Transform of equations:**
$$V(s) = (L_a s + R_a)I_a(s) + K_b \Omega(s)$$
$$\Omega(s)(Js + b) = K_t I_a(s)$$

**Solving for speed/voltage transfer function:**
$$\frac{\Omega(s)}{V(s)} = \frac{K_t}{(L_a s + R_a)(Js + b) + K_t K_b}$$

**Simplified (neglecting $L_a$):**
$$\frac{\Omega(s)}{V(s)} = \frac{K_t/R_a}{Js + b + K_t K_b/R_a} = \frac{K_m}{\tau_m s + 1}$$

where:
- Motor gain: $K_m = \frac{K_t}{R_a b + K_t K_b}$
- Mechanical time constant: $\tau_m = \frac{JR_a}{R_a b + K_t K_b}$

**Position transfer function (integrate speed):**
$$\frac{\Theta(s)}{V(s)} = \frac{K_m}{s(\tau_m s + 1)}$$

### 2.5.4 Important: What is V(t) in Practice?

> **Critical Clarification:** In the above model, $V(t)$ is the **actual voltage** applied to the motor terminals. In a real control system, this comes from a **power converter**, not directly from the controller.

**Complete System View:**

```
┌─────────────────────────────────────────────────────────────────────────┐
│           COMPLETE DC MOTOR CONTROL SYSTEM                              │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  Reference    Error    ┌──────────┐   V_desired   ┌──────────┐         │
│    ω_ref ──►(+)──────► │   PID    │ ────────────► │   PWM    │         │
│              -▲        │Controller│   (0-12V)     │Modulator │         │
│               │        └──────────┘               └────┬─────┘         │
│               │                                        │ D (duty)      │
│               │                                        ▼               │
│               │   ┌─────────────────────────────────────────────┐      │
│               │   │              H-BRIDGE                        │      │
│               │   │                                              │      │
│               │   │    V_supply ────┬────────┬                  │      │
│               │   │                 │        │                  │      │
│               │   │               [Q1]     [Q3]                 │      │
│               │   │                 │        │                  │      │
│               │   │                 ├──[M]───┤  ──► V_actual    │      │
│               │   │                 │        │      (to motor)  │      │
│               │   │               [Q2]     [Q4]                 │      │
│               │   │                 │        │                  │      │
│               │   │                GND ──────┴                  │      │
│               │   └─────────────────────────────────────────────┘      │
│               │                          │                             │
│               │                          ▼                             │
│               │              ┌────────────────────┐                    │
│               │              │     DC MOTOR       │                    │
│               │              │  V = L(di/dt)+Ri+e │───► ω (speed)      │
│               │              │  J(dω/dt)+bω = τ   │                    │
│               │              └────────────────────┘                    │
│               │                          │                             │
│               │                          ▼                             │
│               │              ┌────────────────────┐                    │
│               └──────────────│   ENCODER          │                    │
│                  ω_measured  │   (feedback)       │                    │
│                              └────────────────────┘                    │
└─────────────────────────────────────────────────────────────────────────┘
```

**Key Relationships:**

| Symbol | Meaning | Relationship |
|--------|---------|--------------|
| $V_{desired}$ | Controller output | PID computation result |
| $D$ | PWM duty cycle | $D = V_{desired} / V_{supply}$ (saturated to 0-1) |
| $V_{actual}$ | Actual motor voltage | $V_{actual} = D \times V_{supply}$ (average) |
| $V_{supply}$ | Power supply voltage | Fixed (e.g., 12V, 24V, 48V) |

**Why Textbooks Use $V(t)$ Directly:**

For control system analysis, we often assume:
$$V_{actual}(t) \approx V_{desired}(t)$$

This assumption is valid when:
1. PWM frequency >> system bandwidth (e.g., 20 kHz PWM vs. 100 Hz bandwidth)
2. Power supply is stiff (low impedance)
3. Controller output is within saturation limits

**When This Assumption Fails:**

| Situation | Effect | Solution |
|-----------|--------|----------|
| $V_{desired} > V_{supply}$ | Saturation, windup | Anti-windup in PID |
| Battery voltage drops | Gain changes | Feedforward compensation |
| High PWM ripple | Torque oscillation | Increase PWM frequency |
| Dead-time in H-bridge | Nonlinearity | Dead-time compensation |

### 2.5.5 CppPlot Implementation: DC Motor

```cpp
/**
 * @file ch02_dc_motor.cpp
 * @brief DC motor modeling from physical parameters to transfer function
 * 
 * Learning Outcome: Model an electromechanical system
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <cmath>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║         Chapter 2: DC Motor Modeling                         ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // ========================================================================
    // Physical Parameters (typical small DC motor)
    // ========================================================================
    double Ra = 2.0;     // Armature resistance [Ω]
    double La = 0.01;    // Armature inductance [H]
    double J = 0.01;     // Rotor inertia [kg·m²]
    double b = 0.001;    // Viscous friction [N·m·s]
    double Kt = 0.1;     // Torque constant [N·m/A]
    double Kb = 0.1;     // Back-EMF constant [V·s/rad]
    
    std::cout << "\nDC Motor Physical Parameters:" << std::endl;
    std::cout << "  Armature resistance Ra = " << Ra << " Ω" << std::endl;
    std::cout << "  Armature inductance La = " << La*1000 << " mH" << std::endl;
    std::cout << "  Rotor inertia J = " << J << " kg·m²" << std::endl;
    std::cout << "  Friction b = " << b << " N·m·s" << std::endl;
    std::cout << "  Torque constant Kt = " << Kt << " N·m/A" << std::endl;
    std::cout << "  Back-EMF constant Kb = " << Kb << " V·s/rad" << std::endl;
    
    // ========================================================================
    // Transfer Function: Full 2nd-order model (speed)
    // Ω(s)/V(s) = Kt / [(La*s + Ra)(J*s + b) + Kt*Kb]
    //           = Kt / [La*J*s² + (La*b + Ra*J)*s + (Ra*b + Kt*Kb)]
    // ========================================================================
    double a2 = La * J;
    double a1 = La * b + Ra * J;
    double a0 = Ra * b + Kt * Kb;
    
    // Speed transfer function: Ω(s)/V(s)
    TransferFunction G_speed({Kt}, {a2, a1, a0});
    
    // Position transfer function: θ(s)/V(s) = Ω(s)/(s·V(s))
    TransferFunction G_position({Kt}, {a2, a1, a0, 0});
    
    std::cout << "\nTransfer Functions:" << std::endl;
    std::cout << "  Speed:    Ω(s)/V(s) = " << Kt << " / (" 
              << a2 << "s² + " << a1 << "s + " << a0 << ")" << std::endl;
    std::cout << "  Position: θ(s)/V(s) = " << Kt << " / s(" 
              << a2 << "s² + " << a1 << "s + " << a0 << ")" << std::endl;
    
    // ========================================================================
    // Simplified Model (La ≈ 0)
    // ========================================================================
    double Km = Kt / (Ra * b + Kt * Kb);
    double tau_m = J * Ra / (Ra * b + Kt * Kb);
    
    std::cout << "\nSimplified Model Parameters (La ≈ 0):" << std::endl;
    std::cout << "  Motor gain Km = " << Km << " rad/(V·s)" << std::endl;
    std::cout << "  Time constant τm = " << tau_m << " s" << std::endl;
    
    TransferFunction G_speed_simple({Km}, {tau_m, 1});
    TransferFunction G_position_simple({Km}, {tau_m, 1, 0});
    
    // ========================================================================
    // Poles Analysis
    // ========================================================================
    std::cout << "\nPole Analysis:" << std::endl;
    auto poles_full = G_speed.poles();
    std::cout << "  Full model poles:" << std::endl;
    for (auto& p : poles_full) {
        std::cout << "    s = " << p << std::endl;
    }
    
    std::cout << "  Simplified model pole: s = " << -1/tau_m << std::endl;
    
    // ========================================================================
    // Simulation
    // ========================================================================
    double t_final = 0.5;
    
    // Speed response to 12V step
    double V_step = 12.0;
    auto [t, speed_full] = step_data(G_speed * V_step, t_final);
    auto [t2, speed_simple] = step_data(G_speed_simple * V_step, t_final);
    
    // Position response
    auto [t3, pos_full] = step_data(G_position * V_step, t_final);
    auto [t4, pos_simple] = step_data(G_position_simple * V_step, t_final);
    
    // ========================================================================
    // Plotting
    // ========================================================================
    figure(1200, 800);
    
    // Speed response comparison
    subplot(2, 2, 1);
    plot(t, speed_full, "b-", {{"linewidth", "2"}, {"label", "Full model"}});
    plot(t2, speed_simple, "r--", {{"linewidth", "2"}, {"label", "Simplified"}});
    xlabel("Time [s]");
    ylabel("Speed ω [rad/s]");
    title("Speed Response to 12V Step");
    legend();
    grid(true);
    
    // Position response
    subplot(2, 2, 2);
    plot(t3, pos_full, "b-", {{"linewidth", "2"}, {"label", "Full model"}});
    plot(t4, pos_simple, "r--", {{"linewidth", "2"}, {"label", "Simplified"}});
    xlabel("Time [s]");
    ylabel("Position θ [rad]");
    title("Position Response to 12V Step");
    legend();
    grid(true);
    
    // Current response (derived from speed)
    subplot(2, 2, 3);
    
    // Current can be calculated from: i = (V - Kb*ω)/Ra
    std::vector<double> current;
    for (double w : speed_full) {
        double i = (V_step - Kb * w) / Ra;
        current.push_back(i);
    }
    plot(t, current, "g-", {{"linewidth", "2"}});
    xlabel("Time [s]");
    ylabel("Armature Current i [A]");
    title("Current Response");
    grid(true);
    
    // Torque response
    subplot(2, 2, 4);
    std::vector<double> torque;
    for (double i : current) {
        torque.push_back(Kt * i);
    }
    plot(t, torque, "m-", {{"linewidth", "2"}});
    xlabel("Time [s]");
    ylabel("Torque τ [N·m]");
    title("Motor Torque");
    grid(true);
    
    savefig("ch02_dc_motor.svg");
    std::cout << "\n✓ Saved ch02_dc_motor.svg" << std::endl;
    
    // ========================================================================
    // Physical Interpretation
    // ========================================================================
    double steady_state_speed = Km * V_step;
    double stall_current = V_step / Ra;
    double stall_torque = Kt * stall_current;
    
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║               MOTOR OPERATING POINTS                         ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  For V = 12V input:                                          ║" << std::endl;
    std::cout << "║  • Steady-state speed (no load): " << std::fixed << std::setprecision(1) 
              << steady_state_speed << " rad/s         ║" << std::endl;
    std::cout << "║  • Stall current (ω=0): " << stall_current << " A                        ║" << std::endl;
    std::cout << "║  • Stall torque: " << stall_torque << " N·m                           ║" << std::endl;
    std::cout << "║                                                              ║" << std::endl;
    std::cout << "║  Time constant τm = " << tau_m*1000 << " ms                             ║" << std::endl;
    std::cout << "║  63% of final speed reached in " << tau_m*1000 << " ms                  ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    return 0;
}
```

---

## 2.6 Thermal Systems

### 2.6.1 Thermal Elements

| Element | Relation | Analogy |
|---------|----------|---------|
| Thermal capacitance | $Q = C_{th}\frac{dT}{dt}$ | Capacitor |
| Thermal resistance | $Q = \frac{\Delta T}{R_{th}}$ | Resistor |

### 2.6.2 Energy Balance

**First Law of Thermodynamics:**
$$\text{Rate of energy storage} = \text{Heat in} - \text{Heat out}$$

$$C_{th}\frac{dT}{dt} = Q_{in}(t) - \frac{T - T_{ambient}}{R_{th}}$$

### 2.6.3 Example: Heated Chamber

See Chapter 1 for detailed thermal system modeling.

---

## 2.7 State-Space Representation

### 2.7.1 General Form

For systems with multiple energy storage elements, state-space provides a more general representation:

$$\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}$$
$$\mathbf{y} = C\mathbf{x} + D\mathbf{u}$$

where:
- $\mathbf{x}$ = state vector (energy storage variables)
- $\mathbf{u}$ = input vector
- $\mathbf{y}$ = output vector
- $A$ = system matrix
- $B$ = input matrix
- $C$ = output matrix
- $D$ = feedthrough matrix

### 2.7.2 Mass-Spring-Damper in State-Space

Define states: $x_1 = x$ (position), $x_2 = \dot{x}$ (velocity)

$$\begin{bmatrix} \dot{x}_1 \\ \dot{x}_2 \end{bmatrix} = \begin{bmatrix} 0 & 1 \\ -k/m & -b/m \end{bmatrix} \begin{bmatrix} x_1 \\ x_2 \end{bmatrix} + \begin{bmatrix} 0 \\ 1/m \end{bmatrix} F$$

$$y = \begin{bmatrix} 1 & 0 \end{bmatrix} \begin{bmatrix} x_1 \\ x_2 \end{bmatrix}$$

### 2.7.3 Conversion: Transfer Function ↔ State-Space

```cpp
// Transfer function to state-space
TransferFunction G({1}, {1, 3, 2});
StateSpace sys = tf2ss(G);

// State-space to transfer function
Matrix A = {{0, 1}, {-2, -3}};
Matrix B = {{0}, {1}};
Matrix C = {{1, 0}};
Matrix D = {{0}};
StateSpace ss(A, B, C, D);
TransferFunction G_back = ss2tf(ss);
```

### 2.7.4 Linearization of Nonlinear Systems

Most physical systems are inherently nonlinear. **Linearization** approximates a nonlinear system by a linear one valid near an operating (equilibrium) point, enabling the use of all linear analysis and design tools.

#### General Nonlinear State-Space Model

Consider a nonlinear system:
$$\dot{\mathbf{x}} = \mathbf{f}(\mathbf{x}, \mathbf{u}), \qquad \mathbf{y} = \mathbf{g}(\mathbf{x}, \mathbf{u})$$

where $\mathbf{x} \in \mathbb{R}^n$ is the state vector, $\mathbf{u} \in \mathbb{R}^m$ is the input, and $\mathbf{y} \in \mathbb{R}^p$ is the output.

#### Step 1: Find the Equilibrium Point

An **equilibrium point** $(\mathbf{x}_0, \mathbf{u}_0)$ satisfies:
$$\mathbf{f}(\mathbf{x}_0, \mathbf{u}_0) = \mathbf{0}$$

Physically, this is the steady-state operating condition (e.g., hover for a drone, cruising speed for a vehicle).

#### Step 2: Define Deviation Variables

$$\delta\mathbf{x} = \mathbf{x} - \mathbf{x}_0, \quad \delta\mathbf{u} = \mathbf{u} - \mathbf{u}_0, \quad \delta\mathbf{y} = \mathbf{y} - \mathbf{y}_0$$

#### Step 3: Taylor Series Expansion

Expand $\mathbf{f}$ around $(\mathbf{x}_0, \mathbf{u}_0)$ and keep only first-order terms:

$$\dot{\mathbf{x}} = \mathbf{f}(\mathbf{x}_0, \mathbf{u}_0) + \frac{\partial \mathbf{f}}{\partial \mathbf{x}}\bigg|_0 (\mathbf{x} - \mathbf{x}_0) + \frac{\partial \mathbf{f}}{\partial \mathbf{u}}\bigg|_0 (\mathbf{u} - \mathbf{u}_0) + \text{H.O.T.}$$

Since $\mathbf{f}(\mathbf{x}_0, \mathbf{u}_0) = \mathbf{0}$ and neglecting higher-order terms:
$$\delta\dot{\mathbf{x}} \approx \frac{\partial \mathbf{f}}{\partial \mathbf{x}}\bigg|_0 \delta\mathbf{x} + \frac{\partial \mathbf{f}}{\partial \mathbf{u}}\bigg|_0 \delta\mathbf{u}$$

#### Step 4: Identify the Jacobian Matrices

The linearized system is $\delta\dot{\mathbf{x}} = \mathbf{A}\,\delta\mathbf{x} + \mathbf{B}\,\delta\mathbf{u}$, $\delta\mathbf{y} = \mathbf{C}\,\delta\mathbf{x} + \mathbf{D}\,\delta\mathbf{u}$, where:

$$\mathbf{A} = \frac{\partial \mathbf{f}}{\partial \mathbf{x}}\bigg|_{(\mathbf{x}_0,\mathbf{u}_0)} = \begin{bmatrix} \frac{\partial f_1}{\partial x_1} & \cdots & \frac{\partial f_1}{\partial x_n} \\ \vdots & \ddots & \vdots \\ \frac{\partial f_n}{\partial x_1} & \cdots & \frac{\partial f_n}{\partial x_n} \end{bmatrix}_0$$

$$\mathbf{B} = \frac{\partial \mathbf{f}}{\partial \mathbf{u}}\bigg|_{(\mathbf{x}_0,\mathbf{u}_0)}, \quad \mathbf{C} = \frac{\partial \mathbf{g}}{\partial \mathbf{x}}\bigg|_{(\mathbf{x}_0,\mathbf{u}_0)}, \quad \mathbf{D} = \frac{\partial \mathbf{g}}{\partial \mathbf{u}}\bigg|_{(\mathbf{x}_0,\mathbf{u}_0)}$$

> **Important:** The linearized model is only valid for **small deviations** from the equilibrium point. Large perturbations require re-linearization or nonlinear control methods.

---

## 2.8 Integrated Mechatronic Systems: Real-World Examples

> **Key Insight:** Traditional textbooks often present isolated mechanical or electrical systems. However, **real control systems are multi-domain** integrations of mechanical, electrical, electronic, and information technology components.

### 2.8.1 The Mechatronic Perspective

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    REAL-WORLD CONTROL SYSTEM                            │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐        │
│   │MECHANICAL│◄──►│ELECTRICAL│◄──►│ELECTRONIC│◄──►│ SOFTWARE │        │
│   │ • Mass   │    │ • Motor  │    │ • Sensors│    │ • Control│        │
│   │ • Inertia│    │ • Coils  │    │ • ADC/DAC│    │   Algorithm       │
│   │ • Friction    │ • Back-EMF    │ • PWM    │    │ • Comm   │        │
│   │ • Gears  │    │ • Battery│    │ • MCU    │    │ • HMI    │        │
│   └──────────┘    └──────────┘    └──────────┘    └──────────┘        │
│         ▲               ▲               ▲               ▲              │
│         └───────────────┴───────────────┴───────────────┘              │
│                    ENERGY & INFORMATION FLOW                            │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2.8.2 Example 1: Electric Vehicle (EV) Powertrain

**System Architecture:**

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     ELECTRIC VEHICLE SYSTEM                             │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐             │
│  │ Battery │───►│Inverter │───►│ PMSM    │───►│Gearbox &│───► Wheels  │
│  │ Pack    │    │(3-phase)│    │ Motor   │    │ Diff    │             │
│  │ 400V DC │    │ IGBT/   │    │ AC      │    │ N:1     │             │
│  └────┬────┘    │ SiC     │    └────┬────┘    └─────────┘             │
│       │         └────┬────┘         │                                  │
│       │              │              │                                  │
│       ▼              ▼              ▼                                  │
│  ┌─────────┐    ┌─────────┐    ┌─────────┐                            │
│  │   BMS   │    │  Motor  │    │Resolver/│                            │
│  │ Battery │    │Controller    │ Encoder │                            │
│  │Managemt │    │  (FOC)  │    │ θ, ω    │                            │
│  └────┬────┘    └────┬────┘    └────┬────┘                            │
│       │              │              │                                  │
│       └──────────────┼──────────────┘                                  │
│                      ▼                                                  │
│              ┌──────────────┐         ┌──────────────┐                 │
│              │  Vehicle ECU │◄───────►│  CAN Bus     │                 │
│              │  (Central)   │         │  Network     │                 │
│              └──────────────┘         └──────────────┘                 │
│                      ▲                                                  │
│                      │                                                  │
│              ┌───────┴───────┐                                         │
│              │ Driver Input  │  Accelerator, Brake, Steering           │
│              └───────────────┘                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

**Multi-Domain Model:**

| Domain | Components | State Variables | Equations |
|--------|------------|-----------------|-----------|
| **Mechanical** | Vehicle mass M, wheel radius r, drag | Velocity v, position x | $M\dot{v} = F_{motor} - F_{drag} - F_{rolling}$ |
| **Electrical** | Motor inductance L, resistance R | Currents $i_d$, $i_q$ | $L\frac{di_q}{dt} = v_q - Ri_q - \omega_e \lambda_d$ |
| **Magnetic** | Permanent magnets, flux linkage λ | Flux ψ | $\tau = \frac{3}{2}p\lambda_m i_q$ |
| **Thermal** | Motor temperature, battery temp | T_motor, T_batt | $C\dot{T} = I^2R - h(T - T_{amb})$ |
| **Electrochemical** | Battery SOC, internal resistance | SOC, V_oc | $V_{batt} = V_{oc}(SOC) - I \cdot R_{int}(T)$ |
| **Information** | CAN messages, control loops | Setpoints, states | Digital control at 10-20 kHz |

**Integrated Transfer Function (Simplified Speed Control):**

$$G_{vehicle}(s) = \underbrace{\frac{K_{inv}}{1 + \tau_{inv}s}}_{\text{Inverter}} \cdot \underbrace{\frac{K_m}{(L_q s + R)(Js + b) + K_e K_t}}_{\text{Motor + Load}} \cdot \underbrace{\frac{1}{r \cdot N}}_{\text{Gearbox}}$$

### 2.8.3 Example 2: Industrial Robot Arm (6-DOF)

**Complete System View:**

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    INDUSTRIAL ROBOT SYSTEM                              │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   MECHANICAL          ELECTRICAL         ELECTRONIC        SOFTWARE    │
│   ──────────          ──────────         ──────────        ────────    │
│   • 6 Links           • 6 Servo motors   • 6 Encoders      • Trajectory│
│   • 6 Joints          • Harmonic drives  • 6 Current       • Inverse   │
│   • Inertia matrix    • Brake system       sensors           Kinematics│
│   • Coriolis/         • 48V/560V bus     • EtherCAT        • PID/PD+   │
│     Centrifugal       • Regenerative     • Safety PLC      • Collision │
│   • Gravity             braking          • Force/Torque      Detection │
│   • End-effector      • Cable routing      sensors         • Vision    │
│     (gripper/tool)                       • I/O modules       Integration│
│                                                            • MES/ERP   │
│                                                              Interface │
└─────────────────────────────────────────────────────────────────────────┘
```

**Coupled Dynamics (Lagrangian Formulation):**

$$\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{G}(\mathbf{q}) = \boldsymbol{\tau}_{motor} - \boldsymbol{\tau}_{friction}$$

Where:
- $\mathbf{q} \in \mathbb{R}^6$: Joint angles
- $\mathbf{M}(\mathbf{q})$: Configuration-dependent inertia matrix
- $\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})$: Coriolis and centrifugal terms
- $\mathbf{G}(\mathbf{q})$: Gravity vector
- $\boldsymbol{\tau}_{motor}$: Motor torques (from servo drives)

**Control Hierarchy:**

| Level | Update Rate | Function | Technology |
|-------|-------------|----------|------------|
| Current loop | 20 kHz | Motor current regulation | DSP in servo drive |
| Velocity loop | 4 kHz | Speed control | DSP in servo drive |
| Position loop | 1 kHz | Joint position control | Motion controller |
| Trajectory | 100 Hz | Path planning | Industrial PC |
| Task level | 10 Hz | Pick-and-place logic | PLC / Robot controller |
| MES interface | 1 Hz | Production scheduling | Enterprise software |

### 2.8.4 Example 3: Quadcopter Drone (UAV)

**System Integration:**

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      QUADCOPTER SYSTEM                                  │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│         Motor 1                                   Motor 2               │
│           ◯ CW ──────────────────────────────────── ◯ CCW              │
│             \                                       /                   │
│              \           ┌──────────┐              /                    │
│               \          │ Flight   │             /                     │
│                \─────────│Controller│────────────/                      │
│                /         │ (FCU)    │            \                      │
│               /          └────┬─────┘             \                     │
│              /                │                    \                    │
│             /                 ▼                     \                   │
│           ◯ CCW ─────────────────────────────────── ◯ CW               │
│         Motor 3                                   Motor 4               │
│                                                                         │
│   SENSORS (IMU)          ACTUATORS              COMMUNICATION          │
│   ─────────────          ─────────              ─────────────          │
│   • Accelerometer 3-axis • BLDC Motors ×4       • RC Receiver          │
│   • Gyroscope 3-axis     • ESC ×4 (PWM)        • Telemetry (MAVLink)  │
│   • Magnetometer 3-axis  • Servo (gimbal)       • GPS (u-blox)        │
│   • Barometer            • LED indicators       • WiFi/4G (optional)  │
│   • GPS receiver                                                       │
└─────────────────────────────────────────────────────────────────────────┘
```

**Multi-Physics Model:**

| Domain | Model | Key Variables |
|--------|-------|---------------|
| **Aerodynamics** | $F_i = k_F \omega_i^2$, $\tau_i = k_\tau \omega_i^2$ | Thrust, drag |
| **Rigid Body** | Newton-Euler: $m\dot{\mathbf{v}} = \mathbf{R}(\sum F_i)\mathbf{e}_3 - mg\mathbf{e}_3$ | Position, velocity |
| **Attitude** | $\mathbf{J}\dot{\boldsymbol{\omega}} = -\boldsymbol{\omega} \times \mathbf{J}\boldsymbol{\omega} + \boldsymbol{\tau}$ | Roll, pitch, yaw |
| **Motor** | First-order: $\tau_m \dot{\omega}_i = K_{ESC} u_i - \omega_i$ | Motor speeds |
| **Battery** | $V = V_{oc}(SOC) - IR_{int}$ | Voltage, current |
| **Sensor Fusion** | Extended Kalman Filter | Estimated states |

**State-Space Representation (Linearized Hover):**

States: $\mathbf{x} = [x, y, z, \phi, \theta, \psi, \dot{x}, \dot{y}, \dot{z}, p, q, r]^T$

Inputs: $\mathbf{u} = [T, \tau_\phi, \tau_\theta, \tau_\psi]^T$ (total thrust + 3 torques)

$$\dot{\mathbf{x}} = \begin{bmatrix} 
\mathbf{0}_{6\times6} & \mathbf{I}_{6\times6} \\
\mathbf{A}_{21} & \mathbf{A}_{22}
\end{bmatrix} \mathbf{x} + \begin{bmatrix} \mathbf{0}_{6\times4} \\ \mathbf{B}_2 \end{bmatrix} \mathbf{u}$$

### 2.8.5 Example 4: Smart Building HVAC System

**IoT-Enabled Climate Control:**

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    SMART HVAC SYSTEM                                    │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  CLOUD LAYER                                                            │
│  ───────────                                                            │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │  • Weather API    • Energy pricing    • Occupancy prediction    │   │
│  │  • ML models      • Digital twin      • Remote monitoring       │   │
│  └────────────────────────────┬────────────────────────────────────┘   │
│                               │ MQTT/REST API                          │
│  EDGE LAYER                   ▼                                        │
│  ──────────      ┌────────────────────────┐                           │
│                  │    Building Gateway     │                           │
│                  │    (Edge Computer)      │                           │
│                  └───────────┬────────────┘                           │
│                              │ Modbus/BACnet                          │
│  CONTROL LAYER               ▼                                        │
│  ─────────────   ┌────────────────────────┐                           │
│                  │   Building Automation   │                           │
│                  │   System (BAS/DDC)      │                           │
│                  └───────────┬────────────┘                           │
│                              │                                         │
│  FIELD LAYER    ┌────────────┼────────────┐                           │
│  ───────────    ▼            ▼            ▼                           │
│            ┌────────┐   ┌────────┐   ┌────────┐                       │
│            │ AHU    │   │ Chiller│   │ Boiler │                       │
│            │Control │   │Control │   │Control │                       │
│            └───┬────┘   └───┬────┘   └───┬────┘                       │
│                │            │            │                             │
│            ┌───┴────┐   ┌───┴────┐   ┌───┴────┐                       │
│  SENSORS   │• Temp  │   │• Flow  │   │• Temp  │   ACTUATORS           │
│            │• Humid │   │• Press │   │• Press │   • VFD (fans)        │
│            │• CO2   │   │• Temp  │   │• Flame │   • Valves            │
│            │• VOC   │   │        │   │        │   • Dampers           │
│            └────────┘   └────────┘   └────────┘                       │
└─────────────────────────────────────────────────────────────────────────┘
```

**Thermal Zone Model:**

$$C_{zone}\frac{dT_{zone}}{dt} = \dot{Q}_{HVAC} + \dot{Q}_{internal} + \dot{Q}_{solar} - UA(T_{zone} - T_{ambient})$$

**Integrated State-Space (Single Zone):**

$$\begin{bmatrix} \dot{T}_{zone} \\ \dot{T}_{wall} \\ \dot{x}_{humid} \end{bmatrix} = 
\begin{bmatrix} a_{11} & a_{12} & 0 \\ a_{21} & a_{22} & 0 \\ 0 & 0 & a_{33} \end{bmatrix}
\begin{bmatrix} T_{zone} \\ T_{wall} \\ x_{humid} \end{bmatrix} +
\begin{bmatrix} b_1 & 0 \\ 0 & 0 \\ 0 & b_3 \end{bmatrix}
\begin{bmatrix} \dot{m}_{air} \\ \dot{m}_{water} \end{bmatrix} +
\begin{bmatrix} d_1 \\ d_2 \\ d_3 \end{bmatrix}$$

### 2.8.6 Modeling Philosophy for Integrated Systems

> **Engineering Reality Check:**
> 
> When modeling real systems, always consider:
> 
> 1. **Interface Dynamics:** Actuator bandwidth, sensor delays, communication latency
> 2. **Multi-Rate Sampling:** Different subsystems update at different rates
> 3. **Saturation & Limits:** Physical constraints on all actuators
> 4. **Noise & Disturbances:** Sensor noise, electromagnetic interference
> 5. **Thermal Coupling:** Power electronics generate heat affecting other components
> 6. **Software Delays:** Computation time, bus arbitration, interrupt latency

**Practical Modeling Steps:**

| Step | Action | Tools |
|------|--------|-------|
| 1 | Identify all energy domains | Block diagram |
| 2 | List interface signals | Signal flow diagram |
| 3 | Model each subsystem | Transfer functions |
| 4 | Add interface dynamics | Time delays, filters |
| 5 | Include nonlinearities | Saturation, dead-zone |
| 6 | Validate against data | System identification |
| 7 | Simplify for control design | Model reduction |

---

## ⚠️ 2.8.7 The Three Levels of Models: A Critical Distinction

> **→ Connection to §2.2 (The Art of Abstraction):** This section defines the three model levels. Section §2.2 teaches the *technique* for moving between them — specifically, how to reduce Level 1 (truth model with all physics) to Level 2 (design model) by systematically identifying which energy mechanisms matter for the control question and discarding the rest.

> **Common misconception:** Many students — and even some practicing engineers — treat "the model" as a single entity. In reality, every control engineering project involves at least **three fundamentally different models**, and confusing them is the source of most real-world control failures.

### The Three Models

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    THREE MODELS IN CONTROL ENGINEERING                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  LEVEL 1: THEORETICAL MODEL ("Truth Model")                                 │
│  ─────────────────────────────────────────                                  │
│  • Captures ALL known physics                                               │
│  • Nonlinear, high-order, distributed parameters                           │
│  • Used for: simulation, validation, "what if" analysis                    │
│  • Example: Full Navier-Stokes for a thermal system                        │
│  • Typical order: 10th–1000th (or infinite for PDEs)                       │
│                                                                             │
│  LEVEL 2: DESIGN MODEL ("Control Model")                                    │
│  ─────────────────────────────────────────                                  │
│  • Simplified, linearized, low-order                                        │
│  • Used for: controller design (root locus, Bode, LQR)                     │
│  • Example: G(s) = K/(τs + 1) for the same thermal system                 │
│  • Typical order: 1st–5th                                                   │
│  • THIS IS WHAT THE CONTROLLER "SEES"                                      │
│                                                                             │
│  LEVEL 3: REAL SYSTEM ("Reality")                                           │
│  ─────────────────────────────────────────                                  │
│  • The actual physical hardware                                             │
│  • Contains effects NO model captures: aging, manufacturing                │
│    tolerances, electromagnetic interference, thermal drift...               │
│  • THIS IS WHAT THE CONTROLLER MUST ACTUALLY CONTROL                       │
│                                                                             │
│  THE GAP: Reality ≠ Design Model ≠ Theoretical Model                       │
│  ────────                                                                   │
│  The controller is designed for Level 2.                                     │
│  It must work on Level 3.                                                   │
│  Level 1 tells us HOW MUCH the two differ.                                 │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Why This Matters: The DC Motor Example

| Aspect | Theoretical Model | Design Model | Real Motor |
|--------|-------------------|--------------|------------|
| **Equations** | $L_a \frac{di}{dt} + R_a i + K_e \omega = v(t)$ | $G(s) = \frac{K_m}{\tau_m s + 1}$ | — |
| | $J\frac{d\omega}{dt} + B\omega = K_t i - T_L$ | (1st order, $L_a$ neglected) | |
| **Friction** | Coulomb + viscous + Stribeck | Viscous only ($B\omega$) | All types + hysteresis |
| **Inductance** | Included ($L_a \neq 0$) | Neglected ($L_a \approx 0$) | Varies with rotor position |
| **Temperature** | Resistance varies: $R(T)$ | $R$ = constant | $R$ increases 40% at full load |
| **Backlash** | Modeled if known | Ignored | 0.5° in gearbox |
| **Order** | 2nd or higher | 1st | $\infty$ (continuous medium) |

### The Engineering Question

The question is **never** "Is the model correct?" — no model is correct.

The question is: **"Is the gap between the design model and reality small enough that the controller still works?"**

This is called **robustness**, and it is the central concern of Chapters 6–8 (gain/phase margins) and Chapter 16 (robust control).

> **The Diagnostic Hierarchy:**
> When a controller works in simulation but fails on hardware, ask:
> 1. Is the design model missing a critical physical effect? (model structure error)
> 2. Are the model parameters wrong? (identification error)
> 3. Is the controller not robust enough to tolerate the gap? (design error)
>
> Most students jump to (3). Most real problems are (1).

### Concrete Example: Why Controllers Fail

```
┌──────────────────────────────────────────────────────────────────────────┐
│  SCENARIO: Motor speed controller designed using G(s) = Km/(τs+1)       │
│                                                                          │
│  Design model says:  PM = 60°, settling time = 0.2s  ✓ Looks great     │
│                                                                          │
│  Real motor has:                                                         │
│  • 2ms sensor delay (not in model) → adds −ωT radians phase lag        │
│  • Gear backlash (not in model) → limit cycle oscillation              │
│  • R increases 40% at operating temperature → τ changes, gain changes  │
│                                                                          │
│  Result: PM drops from 60° to 15° → near-oscillatory behavior          │
│                                                                          │
│  FIX: Not "re-tune the controller" but "improve the model"             │
│       Add delay: G(s)·e^(−0.002s)                                       │
│       Add backlash: dead-zone nonlinearity in simulation                │
│       Use worst-case R: design for R_hot, not R_cold                    │
└──────────────────────────────────────────────────────────────────────────┘
```

> **Key takeaway for students:** Every chapter in this book uses *design models* (Level 2). When you graduate and face *real systems* (Level 3), the first skill you need is recognizing which simplifications in the design model are safe and which are dangerous. This is the difference between a textbook exercise and engineering practice.
>
> **→ Bridge to Chapter 9 (System Identification):** This chapter teaches you how to derive models from physics. But physics gives you the *structure* (e.g., "this is a second-order system"), not always the *parameters* (the actual values of $K$, $\tau$, $\zeta$). Chapter 9 teaches the complementary skill: extracting model parameters from measured input-output data. The combination — physics for structure, data for parameters — is the grey-box approach used by practicing engineers.

---

## 2.9 Power Systems and Telecommunications Examples

### 2.9.1 Power System Examples (Electrical Engineering)

#### Example: Buck Converter Voltage Regulation

A Buck (step-down) DC-DC converter is fundamental in power electronics:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    BUCK CONVERTER CIRCUIT                               │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│       V_in ────┬──────[S]──────┬────[L]────┬────────┬──── V_out        │
│       (Input)  │    (Switch)   │           │        │    (Output)      │
│                │               │          ═╪═       │                  │
│               ═╪═             [D]          C    [R_load]               │
│                │            (Diode)       ═╪═       │                  │
│                │               │           │        │                  │
│       GND ─────┴───────────────┴───────────┴────────┴──── GND          │
│                                                                         │
│   Switching: S closes for time D·Ts, opens for (1-D)·Ts                │
│   Output: V_out = D × V_in (ideal, continuous conduction mode)         │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

**State-Space Averaging Model:**

States: $x_1 = i_L$ (inductor current), $x_2 = v_C$ (capacitor voltage)

**During switch ON ($0 < t < DT_s$):**
$$L\frac{di_L}{dt} = V_{in} - v_C, \quad C\frac{dv_C}{dt} = i_L - \frac{v_C}{R}$$

**During switch OFF ($DT_s < t < T_s$):**
$$L\frac{di_L}{dt} = -v_C, \quad C\frac{dv_C}{dt} = i_L - \frac{v_C}{R}$$

**Averaged Model (small-signal around operating point):**
$$\begin{bmatrix} \dot{\tilde{i}}_L \\ \dot{\tilde{v}}_C \end{bmatrix} = 
\begin{bmatrix} 0 & -1/L \\ 1/C & -1/RC \end{bmatrix}
\begin{bmatrix} \tilde{i}_L \\ \tilde{v}_C \end{bmatrix} +
\begin{bmatrix} V_{in}/L \\ 0 \end{bmatrix} \tilde{d}$$

**Control-to-Output Transfer Function:**
$$G_{vd}(s) = \frac{\tilde{v}_C(s)}{\tilde{d}(s)} = \frac{V_{in}}{LC} \cdot \frac{1}{s^2 + \frac{1}{RC}s + \frac{1}{LC}}$$

**CppPlot Implementation:**

```cpp
/**
 * @file ch02_buck_converter.cpp
 * @brief Buck converter small-signal modeling for voltage regulation
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <cmath>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║   Buck Converter Small-Signal Model (Power Electronics)      ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // Converter parameters
    double V_in = 12.0;      // Input voltage [V]
    double V_out = 5.0;      // Desired output [V]
    double D = V_out/V_in;   // Duty cycle (0.417)
    double L = 100e-6;       // Inductance [H]
    double C = 100e-6;       // Capacitance [F]
    double R = 10.0;         // Load resistance [Ω]
    double f_sw = 100e3;     // Switching frequency [Hz]
    
    std::cout << "Converter Parameters:" << std::endl;
    std::cout << "  V_in = " << V_in << " V" << std::endl;
    std::cout << "  V_out = " << V_out << " V (target)" << std::endl;
    std::cout << "  Duty cycle D = " << D << std::endl;
    std::cout << "  L = " << L*1e6 << " µH, C = " << C*1e6 << " µF" << std::endl;
    
    // Small-signal transfer function: Gvd(s) = Vout(s)/d(s)
    // Gvd(s) = (V_in/LC) / (s² + s/RC + 1/LC)
    double wn = 1.0/std::sqrt(L*C);        // Natural frequency
    double zeta = 1.0/(2*R)*std::sqrt(L/C); // Damping ratio
    double K_dc = V_in;                     // DC gain
    
    std::cout << "\nSmall-Signal Parameters:" << std::endl;
    std::cout << "  Natural frequency ωn = " << wn << " rad/s (" << wn/(2*M_PI) << " Hz)" << std::endl;
    std::cout << "  Damping ratio ζ = " << zeta << std::endl;
    
    TransferFunction Gvd({V_in/(L*C)}, {1, 1/(R*C), 1/(L*C)});
    
    // Voltage mode control: PI compensator
    double Kp = 0.1, Ki = 1000;
    TransferFunction Gc({Kp, Ki}, {1, 0});  // PI controller
    
    // Open-loop and closed-loop
    TransferFunction L_open = Gc * Gvd;
    TransferFunction T_closed = feedback(L_open, TransferFunction(1.0));
    
    // Step response
    double t_final = 0.01;  // 10 ms
    auto [t, v] = step_data(T_closed * V_out, t_final);
    
    figure(1200, 600);
    
    subplot(1, 2, 1);
    plot(t, v, "b-", {{"linewidth", "2"}});
    axhline(V_out, {{"color", "green"}, {"linestyle", "--"}, {"label", "Reference"}});
    xlabel("Time [s]");
    ylabel("Output Voltage [V]");
    title("Buck Converter Voltage Regulation");
    legend();
    grid(true);
    
    // Bode plot of plant
    subplot(1, 2, 2);
    bode(Gvd);
    title("Control-to-Output Bode Plot");
    
    savefig("ch02_buck_converter.svg");
    
    return 0;
}
```

#### Example: Synchronous Generator AVR (Automatic Voltage Regulator)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    GENERATOR AVR SYSTEM                                 │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   V_ref ──►(+)─── AVR ─── Exciter ─── Generator ─── V_terminal         │
│             -▲  Controller  (DC)      (3-phase)                        │
│              │                                                          │
│              └──────────── PT (feedback) ◄─────────────┘               │
│                                                                         │
│   TRANSFER FUNCTIONS:                                                   │
│                                                                         │
│   AVR:       G_AVR(s) = K_A / (1 + sT_A)                               │
│                         K_A = 200, T_A = 0.05s                         │
│                                                                         │
│   Exciter:   G_E(s) = K_E / (1 + sT_E)                                 │
│                       K_E = 1.0, T_E = 0.5s                            │
│                                                                         │
│   Generator: G_G(s) = K_G / (1 + sT_G)                                 │
│                       K_G = 1.0, T_G = 1.0s                            │
│                                                                         │
│   Sensor:    H(s) = K_R / (1 + sT_R)                                   │
│                     K_R = 1.0, T_R = 0.05s                             │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

**Open-Loop Transfer Function:**
$$G_{open}(s) = \frac{K_A K_E K_G}{(1+sT_A)(1+sT_E)(1+sT_G)}$$

### 2.9.2 Telecommunications System Examples

#### Example: Phase-Locked Loop (PLL)

The PLL is a fundamental building block in communications:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    PHASE-LOCKED LOOP MODEL                              │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   θ_ref(s) ──►(+)─── K_pd ─── F(s) ─── K_vco/s ───┬──► θ_out(s)       │
│               -▲    Phase   Loop      VCO         │                    │
│                │    Detector Filter   (Integrator)│                    │
│                │                                   │                    │
│                └──────── 1/N (Divider) ◄──────────┘                    │
│                                                                         │
│   COMPONENT TRANSFER FUNCTIONS:                                         │
│                                                                         │
│   Phase Detector: K_pd [V/rad]                                         │
│   Loop Filter (Type II): F(s) = (1 + s/ω_z)/(s/ω_p)  (Lead-lag)       │
│   VCO: K_vco/s [rad/V·s]  (Frequency to phase = integrator)           │
│   Divider: 1/N                                                          │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

**Linearized PLL Transfer Function:**

Open-loop:
$$G_{open}(s) = K_{pd} \cdot F(s) \cdot \frac{K_{vco}}{s} \cdot \frac{1}{N}$$

For Type II PLL with active PI loop filter $F(s) = K_p + K_i/s$:

$$G_{open}(s) = \frac{K_{pd} K_{vco}}{Ns} \left(K_p + \frac{K_i}{s}\right) = \frac{K_{pd} K_{vco}(K_p s + K_i)}{Ns^2}$$

**Closed-Loop (Phase Transfer):**

Since $G_{open}(s)$ is the *loop gain* (which already includes the $1/N$ divider), the forward-path gain from phase error to output phase is $G_{fwd}(s) = N \cdot G_{open}(s)$. Applying the standard feedback formula $G_{fwd}/(1 + G_{fwd} \cdot (1/N))$:

$$H(s) = \frac{\theta_{out}(s)}{\theta_{ref}(s)} = \frac{N \cdot G_{open}(s)}{1 + G_{open}(s)}$$

**CppPlot Implementation:**

```cpp
/**
 * @file ch02_pll.cpp
 * @brief Phase-Locked Loop modeling and analysis
 */

#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
#include <iostream>
#include <cmath>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║   Phase-Locked Loop Model (Telecommunications)               ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    // PLL parameters
    double K_pd = 0.5;       // Phase detector gain [V/rad]
    double K_vco = 2*M_PI*1e6; // VCO gain [rad/s/V] (1 MHz/V)
    double N = 100;          // Frequency divider ratio
    
    // Loop filter (Type II - PI)
    double Kp = 0.01;        // Proportional gain
    double Ki = 100;         // Integral gain
    
    // Natural frequency and damping (for design)
    double wn = std::sqrt(K_pd * K_vco * Ki / N);  // Natural frequency
    double zeta = 0.5 * Kp * std::sqrt(K_pd * K_vco / (Ki * N));  // Damping
    
    std::cout << "PLL Parameters:" << std::endl;
    std::cout << "  K_pd = " << K_pd << " V/rad" << std::endl;
    std::cout << "  K_vco = " << K_vco/(2*M_PI) << " Hz/V" << std::endl;
    std::cout << "  Divider N = " << N << std::endl;
    std::cout << "\nLoop Dynamics:" << std::endl;
    std::cout << "  Natural frequency ωn = " << wn << " rad/s" << std::endl;
    std::cout << "  Damping ratio ζ = " << zeta << std::endl;
    std::cout << "  Lock bandwidth ≈ " << 2*zeta*wn/(2*M_PI) << " Hz" << std::endl;
    
    // Loop filter: F(s) = Kp + Ki/s = (Kp*s + Ki)/s
    TransferFunction F({Kp, Ki}, {1, 0});
    
    // VCO: K_vco/s
    TransferFunction VCO({K_vco}, {1, 0});
    
    // Open-loop: K_pd * F(s) * K_vco/s * (1/N)
    TransferFunction G_open = K_pd * F * VCO * (1.0/N);
    
    // Closed-loop phase transfer
    TransferFunction H = feedback(G_open * N, TransferFunction(1.0));
    
    // Phase step response (e.g., reference phase jump)
    double t_final = 1e-3;  // 1 ms
    auto [t, theta_out] = step_data(H, t_final);
    
    // Scale to degrees
    std::vector<double> theta_deg;
    for (double th : theta_out) theta_deg.push_back(th * 180/M_PI);
    
    figure(1200, 600);
    
    subplot(1, 2, 1);
    plot(t, theta_deg, "b-", {{"linewidth", "2"}});
    axhline(180/M_PI, {{"color", "green"}, {"linestyle", "--"}, {"label", "Reference"}});
    xlabel("Time [s]");
    ylabel("Phase [degrees]");
    title("PLL Phase Step Response");
    legend();
    grid(true);
    
    // Bode plot of open-loop
    subplot(1, 2, 2);
    bode(G_open);
    title("PLL Open-Loop Bode Plot");
    
    savefig("ch02_pll.svg");
    
    return 0;
}
```

#### Example: Automatic Gain Control (AGC) in Receivers

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    AGC SYSTEM IN RECEIVER                               │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   RF Input ──► VGA ──► Demodulator ──►┬──► Output (constant level)     │
│               (Variable              │                                  │
│                Gain Amp)             │                                  │
│                  ▲                    │                                  │
│                  │ V_control          │                                  │
│                  │                    │                                  │
│              ┌───┴────┐               │                                  │
│              │  Loop  │◄── Detector ◄─┘                                 │
│              │ Filter │    (Envelope/                                   │
│              └───┬────┘     Power)                                      │
│                  │                                                      │
│                  └──► Compare with V_ref                                │
│                                                                         │
│   NONLINEAR MODEL:                                                      │
│   V_out = G(V_c) × V_in  where G(V_c) = G_0 × exp(-k×V_c)             │
│                                                                         │
│   LINEARIZED (small-signal):                                           │
│   ΔV_out/ΔV_c = -k × G × V_in ≈ -k × V_out_ref                        │
│                                                                         │
│   LOOP FILTER: Typically integrator or PI for zero steady-state error  │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

**AGC Design Considerations:**

| Parameter | Typical Value | Purpose |
|-----------|---------------|---------|
| Attack time | 1-10 ms | Fast response to sudden increase |
| Release time | 100-500 ms | Slow decay to avoid pumping |
| Dynamic range | 60-100 dB | Range of input levels handled |
| Loop bandwidth | 10-100 Hz | Trade-off: speed vs noise immunity |

#### Example: Adaptive Equalizer (LMS Algorithm)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    ADAPTIVE EQUALIZER                                   │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   x[n] (received) ──► FIR Filter ──► y[n] ──►┬──► Output               │
│                       w[0]...w[N-1]          │    (equalized)          │
│                            ▲                  │                         │
│                            │ w_update         │                         │
│                            │                  │                         │
│                       ┌────┴────┐             │                         │
│                       │   LMS   │◄── e[n] ◄──(+)──► d[n]               │
│                       │Algorithm│           -      (desired/            │
│                       └─────────┘                   training)           │
│                                                                         │
│   LMS UPDATE EQUATION:                                                  │
│   w[k+1] = w[k] + μ × e[n] × x[n]                                      │
│                                                                         │
│   where: e[n] = d[n] - y[n]   (error signal)                           │
│          μ = step size (learning rate)                                  │
│                                                                         │
│   STABILITY: 0 < μ < 2/(N × E[x²])                                     │
│                                                                         │
│   THIS IS A FEEDBACK CONTROL SYSTEM!                                    │
│   • Error e[n] drives adaptation (like controller error)               │
│   • Weights w[k] are the "state" being controlled                      │
│   • Desired output d[n] is the "reference"                             │
│   • Step size μ is like a gain parameter                               │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2.9.3 Cross-Domain Modeling Insights

| Concept | Power Systems | Telecommunications |
|---------|---------------|-------------------|
| **Feedback** | AVR, governor, FACTS | PLL, AGC, APC |
| **State variables** | Voltage, current, speed | Phase, amplitude, coefficients |
| **Time constants** | ms to seconds | μs to ms |
| **Stability concern** | Rotor angle, voltage collapse | Lock range, convergence |
| **Nonlinearity** | Saturation, dead-band | Limiting, quantization |
| **Disturbance** | Load changes, faults | Fading, interference |

---

## 2.10 Exercises

**Exercise 2.1 — RLC Series Circuit Transfer Function** *(Analytical)*

Consider a series RLC circuit with resistance $R$, inductance $L$, and capacitance $C$, driven by an input voltage $V_{in}(s)$. The output is the voltage across the capacitor $V_C(s)$.

(a) Using Kirchhoff's voltage law, write the differential equation relating $V_{in}(t)$ to $V_C(t)$.  
(b) Take the Laplace transform (assuming zero initial conditions) and derive the transfer function:

$$G(s) = \frac{V_C(s)}{V_{in}(s)} = \frac{1/LC}{s^2 + (R/L)s + 1/LC}$$

(c) For $R = 2\,\Omega$, $L = 1\,\text{H}$, $C = 0.5\,\text{F}$, find the natural frequency $\omega_n$ and damping ratio $\zeta$.  
(d) Is this system underdamped, critically damped, or overdamped?

---

**Exercise 2.2 — DC Motor Transfer Function** *(Analytical)*

A DC motor has the following parameters:
- Armature resistance: $R_a = 2\,\Omega$
- Armature inductance: $L_a = 0.5\,\text{H}$
- Torque constant: $K_t = 0.1\,\text{N·m/A}$
- Back-EMF constant: $K_b = 0.1\,\text{V·s/rad}$
- Rotor inertia: $J = 0.01\,\text{kg·m}^2$
- Viscous friction: $b = 0.001\,\text{N·m·s/rad}$

(a) Write the electrical and mechanical equations of the motor.  
(b) Derive the transfer function from armature voltage $V_a(s)$ to angular speed $\Omega(s)$.  
(c) What is the order of this system?  
(d) If $L_a$ is neglected (common approximation), what does the transfer function simplify to? What is the effective time constant?

---

**Exercise 2.3 — Linearization** *(Analytical)*

Consider the nonlinear system:
$$\dot{x} = x^2 - 2x + u$$

(a) Find the equilibrium point for $u_{eq} = 1$. (Set $\dot{x} = 0$ and solve for $x_{eq}$.)  
(b) Define deviation variables $\delta x = x - x_{eq}$ and $\delta u = u - u_{eq}$.  
(c) Linearize the system around the equilibrium point $(x_{eq} = 1, u_{eq} = 1)$ using a Taylor series expansion.  
(d) Write the linearized equation in the form $\delta\dot{x} = a\,\delta x + b\,\delta u$ and identify $a$ and $b$.  
(e) Is the linearized system stable? Justify your answer.

---

**Exercise 2.4 — Mass-Spring-Damper Simulation** *(Computational/Coding)* ⭐

Using CppPlot, model and simulate a mass-spring-damper system with:
- Mass: $m = 2\,\text{kg}$
- Damping coefficient: $c = 3\,\text{N·s/m}$
- Spring constant: $k = 5\,\text{N/m}$

(a) Derive the transfer function from applied force $F(s)$ to displacement $X(s)$:
$$G(s) = \frac{1}{ms^2 + cs + k} = \frac{1}{2s^2 + 3s + 5}$$

(b) Write a CppPlot program to plot the unit step response.

```cpp
#include <cppplot/cppplot.hpp>
using namespace cppplot;

int main() {
    // Mass-spring-damper: G(s) = 1/(2s^2 + 3s + 5)
    TransferFunction G({1}, {2, 3, 5});
    
    auto [t, y] = step_data(G, 10.0);
    plot(t, y);
    title("Mass-Spring-Damper Step Response (m=2, c=3, k=5)");
    xlabel("Time (s)");
    ylabel("Displacement (m)");
    grid(true);
    show();
    
    return 0;
}
```

(c) From the plot, estimate the natural frequency, damping ratio, and steady-state displacement.  
(d) Verify your estimates: $\omega_n = \sqrt{k/m}$, $\zeta = c/(2\sqrt{mk})$.

---

**Exercise 2.5 — Two-Tank Liquid Level System** *(Analytical)*

Consider two tanks connected in series. Tank 1 has cross-sectional area $A_1$ and drains into Tank 2 (area $A_2$) through a valve with resistance $R_1$. Tank 2 drains through a valve with resistance $R_2$. The inflow to Tank 1 is $q_{in}(t)$.

(a) Using mass balance ($A\dot{h} = q_{in} - q_{out}$, where $q = h/R$), write the differential equations for the levels $h_1(t)$ and $h_2(t)$.  
(b) Derive the state-space model in the form:
$$\begin{bmatrix} \dot{h}_1 \\ \dot{h}_2 \end{bmatrix} = A \begin{bmatrix} h_1 \\ h_2 \end{bmatrix} + B\, q_{in}$$

Identify matrices $A$, $B$, $C$, $D$ where the output is $h_2$.  
(c) What is the order of this system? How many energy storage elements are there?

---

**Exercise 2.6 — Transfer Function to State-Space Conversion** *(Analytical/Computational)*

Given the transfer function:
$$G(s) = \frac{1}{s^2 + 2s + 1} = \frac{1}{(s+1)^2}$$

(a) By hand, convert this to controllable canonical state-space form:
$$\dot{\mathbf{x}} = \begin{bmatrix} 0 & 1 \\ -1 & -2 \end{bmatrix} \mathbf{x} + \begin{bmatrix} 0 \\ 1 \end{bmatrix} u, \quad y = \begin{bmatrix} 1 & 0 \end{bmatrix} \mathbf{x}$$

(b) Verify using CppPlot's `tf2ss()` function:  
```cpp
TransferFunction G({1}, {1, 2, 1});
auto [A, B, C, D] = tf2ss(G);
std::cout << "A = " << A << std::endl;
std::cout << "B = " << B << std::endl;
std::cout << "C = " << C << std::endl;
std::cout << "D = " << D << std::endl;
```

(c) Simulate the step response using both the transfer function and state-space representations. Do they match?

---

**Exercise 2.7 — Buck Converter Duty Cycle Variation** *(Computational/Coding)*

Refer to the buck converter small-signal model presented in this chapter (§2.9.1 or relevant section).

**(a)** Using the linearized small-signal model, vary the steady-state duty cycle $D$ from 0.2 to 0.8 in steps of 0.1.  

**(b)** For each value of $D$, compute the steady-state output voltage $V_{out,ss} = D \times V_{in}$ (ideal non-switching loss case).  

**(c)** Plot the step response of the small-signal transfer function for $D = 0.3$, $D = 0.5$, and $D = 0.7$ on the same graph using CppPlot (or compare all values).  

**(d)** How do the transient characteristics (overshoot, settling time, oscillation frequency) change with different operating points $D$?  

**(e)** At what duty cycle (if any) does the system exhibit the most oscillatory behavior? Relate this to the damping ratio and/or other parameters.

**Hint — Small-Signal Parameters and Model Insights**  
In the basic averaged small-signal model of a Buck converter operating in continuous conduction mode (CCM) with ideal components (no inductor DCR, no capacitor ESR), the **dynamics** (pole locations, natural frequency $\omega_n = 1/\sqrt{LC}$, and damping ratio $\zeta = \frac{1}{2} \sqrt{\frac{L}{R_{load}^2 C}}$) are **independent of the steady-state duty cycle $D$**.  

The control-to-output transfer function (from duty cycle disturbance $\hat{d}$ to output voltage $\hat{v}_o$) is:  
$$G_{vd}(s) = V_{in} \cdot \frac{1}{1 + s \frac{L}{R_{load}} + s^2 LC}$$  

Use the following typical parameters (adjust if the chapter specifies different values):  
- Input voltage: $V_{in} = 12\,\text{V}$  
- Inductor: $L = 100\,\mu\text{H}$  
- Capacitor: $C = 470\,\mu\text{F}$  
- Load resistance: $R_{load} = 8\,\Omega$ (chosen to give reasonable damping)  

The DC gain of $G_{vd}(s)$ is $V_{in}$ (fixed), so a fixed-amplitude step in $\hat{d}$ (e.g., 0.01) produces the same transient shape regardless of $D$. Only the steady-state operating voltage changes with $D$.  

If the chapter includes parasitic effects (capacitor ESR $R_{esr}$ or inductor DCR $r_L$), add them to the model — these can introduce slight $D$-dependence in more advanced approximations. For simulation in CppPlot, define the transfer function with the denominator $\{LC, L/R_{load}, 1\}$ and numerator $\{V_{in}\}$ (or scaled appropriately).  

---
**Exercise 2.8 — Abstraction in Quadrotor Drone Modeling** *(Conceptual/Analytical)*

A quadrotor drone (four rotors in a symmetric cross configuration) is a popular platform for control systems studies. It has complex physics: rotor aerodynamics, body inertia, gyroscopic effects, motor dynamics, battery voltage sag, airframe flexibility, sensor noise, and external disturbances like wind.

Use the abstraction process from §2.2 to build appropriate models for two different control questions.

**(a) Altitude control in calm air**  
The control objective is to maintain or change the drone's vertical position (altitude) smoothly and precisely, with no significant lateral motion or rotation. There is no wind or other external lateral disturbance.

- List at least five physical effects present in a real quadrotor.
- For each effect, decide whether to **keep** it (include in the model) or **discard** it (neglect). Justify your decision based on whether the effect significantly influences the vertical energy exchange and dynamics relevant to this question.
- Write the simplified differential equation(s) or transfer function that result from your abstraction. What is the order of the resulting model?
- Expected dominant model: often a simple double integrator (thrust → acceleration → velocity → altitude) or with added motor/thrust dynamics.

**(b) Attitude control in wind**  
The control objective is to maintain stable roll, pitch, and yaw angles despite gusty wind that applies unpredictable torques and forces.

- Repeat the same analysis as in (a), but now for this question.
- How does your list of kept/discarded effects differ from part (a)? Why?
- What additional physical mechanisms become dominant (e.g., aerodynamic drag, gyroscopic precession, differential thrust for torque)?
- Write the simplified equations or state-space form that capture the essential dynamics. What is the order of this model? Why is it higher than the altitude model?

**(c) Reflection**  
Explain why the same physical object (the quadrotor) leads to two very different mathematical models depending on the control question. How would you verify that each abstraction is valid for its intended purpose (suggest at least two verification methods from §2.2)?

---

## 2.11 Chapter Summary

### Key Concepts

✅ **Modeling Process:** Physical system → Laws → Differential equations → Transfer function

✅ **Mechanical Systems:** Newton's laws, mass-spring-damper

✅ **Electrical Systems:** Kirchhoff's laws, RLC circuits

✅ **Electromechanical:** DC motor, coupled electrical-mechanical dynamics

✅ **Thermal Systems:** Energy balance, first-order model

✅ **Analogies:** Mechanical ↔ Electrical systems share mathematical structure

✅ **State-Space:** Multi-variable representation

### What's Next

Chapter 3 develops the Laplace transform techniques needed for solving differential equations and analyzing transfer functions.

---

## 2.12 Self-Assessment

### Checklist

- [ ] I can apply Newton's laws to derive equations of motion
- [ ] I can apply Kirchhoff's laws to analyze circuits
- [ ] I can convert differential equations to transfer functions
- [ ] I can identify the order of a system from its physical description
- [ ] I understand the mechanical-electrical analogy
- [ ] I can abstract a real physical object into canonical model elements by identifying which energy mechanisms dominate (§2.2)

### Practice Problems

**2.1** Derive the transfer function for a two-mass system connected by a spring.

**2.2** Model an RC low-pass filter and compare with a thermal system.

**2.3** Extend the DC motor model to include a gearbox with ratio N.

**2.4** Model a simple pendulum (small angle approximation) and identify its natural frequency.

### Problem Identification Exercises (Level 3-4)

**2.5 — What Is the Real Problem?**
An engineer models a car suspension as a simple mass-spring-damper. The simulation predicts a smooth ride, but the real car bounces excessively on rough roads.

(a) List at least three physical effects that the simple model *ignores* (e.g., tire compliance, nonlinear damping, multi-body dynamics).
(b) For each missing effect, explain whether it would make the real response *better* or *worse* than the model predicts, and why.
(c) At what point does adding more detail to the model stop being useful? State your criterion.

**2.6 — Mechanism vs. Procedure**
Two students both derive the transfer function $G(s) = 1/(ms^2 + bs + k)$. Student A memorizes the formula. Student B draws the free body diagram, applies Newton's law, and derives it from scratch.

(a) If the system changes (e.g., add a second spring in series), which student can adapt? Why?
(b) Explain *physically* what each term ($ms^2$, $bs$, $k$) represents in the denominator. What happens to the step response if $b = 0$?
(c) A colleague says: "The transfer function is wrong — the real system has friction that increases with velocity squared." Can the linear model still be useful? Under what conditions?

---

## References

1. Ogata, K. (2010). Modern Control Engineering (5th ed.). Prentice Hall.
2. Franklin, G. F., Powell, J. D., & Emami-Naeini, A. (2019). Feedback Control of Dynamic Systems (8th ed.). Pearson.
3. Dorf, R. C., & Bishop, R. H. (2016). Modern Control Systems (13th ed.). Pearson.

---

*Next Chapter: Laplace Transform Methods →*
