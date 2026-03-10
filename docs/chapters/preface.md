# Preface

---

## Why This Book Exists

Every semester, I face the same scene: a classroom of bright engineering students, fluent in Python, comfortable with MATLAB, yet struggling to connect the elegant mathematics of Laplace transforms and Bode plots to the embedded C++ code running on the microcontrollers they will program after graduation. The gap between textbook theory and industrial practice is not merely pedagogical — it is linguistic. The language of control theory is mathematics; the language of implementation is code. Most textbooks speak only the first language. This book speaks both.

**Modern Control Engineering: Theory and C++ Implementation** was born from two convictions:

1. **You don't truly understand a transfer function until you've built one in code, plotted its response, and watched it go unstable when you change a coefficient.** Mathematical derivations are necessary but insufficient. A student who can prove Lyapunov stability but cannot implement a Kalman filter in a language that runs on real hardware has learned only half of control engineering.

2. **C++ is the language of real-time control.** While MATLAB and Python are indispensable for design and analysis, the vast majority of deployed controllers — from automotive ECUs to industrial PLCs, from drone flight computers to 5G base station DSPs — run compiled C or C++ code. Students who learn control theory in C++ from the start develop an intuition for computational cost, numerical precision, and real-time constraints that transfers directly to their professional careers.

---

## What Makes This Book Different

### "How", Not "How To"

This book teaches **mechanisms**, not **recipes**.

The difference is fundamental. A "how to" approach says: *"Step 1: compute the error. Step 2: multiply by the gain. Step 3: apply to the plant."* A student can follow these instructions and get a correct answer without understanding anything. They have learned a procedure. They cannot adapt it.

A "how" approach says: *"Feedback works because the error signal carries information about the disturbance's effect on the output. The controller cannot see the disturbance directly — it can only observe the disturbance's consequence through the error. By acting on this information, the controller compensates for what it cannot measure."* A student who understands this mechanism can design systems they have never seen before.

This distinction shapes every chapter:

| "How to" (we avoid this)                          | "How" (we teach this)                                    |
|---------------------------------------------------|----------------------------------------------------------|
| "Draw the Bode plot using these steps"            | "The Bode plot reveals HOW the system filters different frequencies — and WHY gain margin predicts instability" |
| "Apply the Routh criterion"                       | "The Routh array counts sign changes because each sign change corresponds to a root crossing the imaginary axis" |
| "Compute the LQR gain from the Riccati equation"  | "The Riccati equation balances two competing costs: deviating from the desired state vs. expending control effort" |

### Physical Meaning and Implementation of Every Signal

Every signal in every block diagram throughout this book has four attributes:

1. **A name** — what we call it ($e(t)$, $u(t)$, $y(t)$, $d(t)$)
2. **A unit** — its physical dimension (°C, Watts, Volts, rad/s)
3. **A physical meaning** — what it represents in the real world
4. **A hardware realization** — how it actually exists in a physical system

For example, in a room temperature controller:

| Signal | Name | Unit | Physical Meaning | Hardware |
|--------|------|------|------------------|----------|
| $r(t)$ | Reference | °C | The user's desired temperature — their *wish* | Thermostat dial → potentiometer → ADC → µC |
| $e(t)$ | Error | °C | Temperature deficit: how far from comfort. **The controller's only window into the world.** | Computed in firmware: `e = r - y` |
| $u(t)$ | Control | W | Heater electrical power — the physical action that changes temperature | Relay/MOSFET switching a 50W ceramic element |
| $y(t)$ | Output | °C | Actual room temperature — the result of all heat flows | NTC thermistor → voltage divider → ADC |
| $d(t)$ | Disturbance | °C | Outside temperature change — the environment acting on us. **Not under our control.** | Weather, open windows — no hardware |

This is not mere labeling. When a student understands that $e(t)$ is "the temperature deficit in degrees Celsius, computed by the microcontroller as a firmware subtraction, and it is the *only* channel through which the controller learns about disturbances," they understand *why* feedback works. The error signal is not an abstract mathematical quantity — it is a physical information carrier. Every signal in every block diagram in this book is grounded this way.

### Theory First, Always

This is not a programming book that happens to mention control theory. It is a rigorous control systems textbook — covering everything from Routh-Hurwitz stability to H∞ robust control to describing functions — that uses C++ as its computational language instead of MATLAB. Every theorem is stated precisely. Every derivation is shown. Every design procedure is justified mathematically before it is implemented in code.

### The CppPlot Control Systems Library

All code examples use **CppPlot**, a modern, header-only C++ library designed specifically for this textbook. CppPlot provides a MATLAB-like API for control system analysis:

```cpp
#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
using namespace cppplot;
using namespace cppplot::control;

int main() {
    // Define a plant
    TransferFunction G({10}, {1, 3, 2});
    
    // Design with frequency-domain tools
    auto m = margin(G);
    std::cout << "GM = " << 20*std::log10(m.Gm) << " dB, "
              << "PM = " << m.Pm << "°" << std::endl;
    
    // Visualize
    figure(800, 500);
    bode(G);
    savefig("bode_plot.svg");
    
    return 0;
}
```

The library handles `TransferFunction`, `StateSpace`, `Matrix`, Bode/Nyquist/root locus plots, step/impulse responses, controller design (`lqr`, `place`, `acker`), Kalman filters, MPC, and H∞ synthesis — everything needed for a complete control systems course. Appendix C provides the full API reference.

### Physical-First, EE/Telecom-Rich

Every chapter begins with a real physical system — DC motors, inverted pendulums, Buck converters, Phase-Locked Loops, grid-tied inverters — and derives the mathematical model from first principles before proceeding to analysis and design. The Electrical Engineering and Telecommunications applications are not relegated to a single chapter; they are woven throughout the text:

- **Chapter 1:** PLL as a feedback system
- **Chapter 2:** RLC circuits, DC motor modeling
- **Chapter 8:** PLL loop filter design via frequency-domain methods
- **Chapter 9:** System identification from measured data (bridging models to reality)
- **Chapter 10:** Three-phase inverter dq-frame modeling
- **Chapter 12:** DPLL as state-feedback design  
- **Chapter 14:** Digital PID, anti-windup, discrete PR resonant controllers
- **Appendix A:** Complete EV traction drive design (FOC + current loops + speed loop)

### Problem Identification, Not Just Problem Solving

With the explosive growth of artificial intelligence, a fundamental shift is underway in what it means to be an engineer. AI can already write code, tune controllers, and solve well-posed optimization problems with superhuman speed. The skills that remain irreducibly human are not about solving faster — they are about **seeing deeper**: identifying the right problem, questioning assumptions, and distinguishing between the surface-level symptom and the root cause.

To understand this shift, it helps to distinguish three concepts that are often conflated:

| | Definition | Relationship to Change |
|---|---|---|
| **Science** | Understanding *why* nature behaves as it does | Unchanging — physics does not expire |
| **Engineering** | Identifying the *right problem* and formulating a solution within constraints | Evolves slowly — principles endure |
| **Technology** | The *tools and methods* used to implement a solution | Changes rapidly — today's tool is tomorrow's artifact |

A student who masters only technology (a specific software tool, a specific algorithm, a specific hardware platform) will find their skills obsolete within a decade. A student who masters only science (theorems, proofs, derivations) may lack the practical judgment to apply them. **The engineer who understands the science deeply enough to identify the right problem will always find — or create — the right technology.** This is the skill that no AI can replace.

#### The Postal Mail Parable

Consider a simple question: *how do we improve the postal service?*

The answer depends entirely on how deeply we identify the real problem:

```
  Level 0 — The surface problem:
  "We need to deliver letters faster."
  → Solution: faster horses, trains, airplanes, overnight delivery.
     Technology: transportation.

  Level 1 — One layer deeper:
  "Wait — the goal is not to move the PAPER. It is to move the CONTENT."
  → Solution: telegraph, fax, email.
     Technology: electrical signaling.
     The entire transportation infrastructure becomes irrelevant.

  Level 2 — Deeper still:
  "The content itself is a vehicle. The real goal is to transmit
   the THOUGHTS and EMOTIONS of the sender to the receiver."
  → Solution: telephone, video call. Now we transmit not just words
     but tone, facial expression, gesture — the full bandwidth
     of human communication.
     Technology: audio/video encoding and transmission.

  Level 3 — The deepest reframing:
  "Voice and video are still indirect. The sender's brain encodes
   thoughts into facial muscles, vocal cords, and gestures.
   The receiver's brain decodes them back. Both conversions
   are lossy. What if we transmit brain signals directly?"
  → Solution: brain-computer interfaces, neural signal transmission.
     Technology: neuroscience + signal processing.
     This is arguably the future of communication.
```

Notice what happened. **Each level of deeper problem identification didn't improve the previous solution — it made it obsolete.** Faster horses did not lead to email. Better fax machines did not lead to video calls. The breakthrough at each level came not from solving the current problem better, but from *redefining what the problem actually was*.

This is exactly analogous to control engineering:

| Postal Analogy | Control Engineering Analog |
|---|---|
| "Deliver the letter faster" | "Tune the PID gains for faster response" |
| "Transmit the content, not the paper" | "The real problem isn't speed — it's disturbance rejection. Use a different architecture." |
| "Transmit thoughts and emotions" | "The real problem isn't even at the control level — it's the sensor model. Fix observability first." |
| "Direct brain-to-brain" | "Redesign the entire system — replace feedback control with feedforward prediction using a physics-informed model" |

Every chapter in this book asks **"What is the real problem?"** before asking "How do we solve it?" We train students to peel back layers — to distinguish symptoms from causes, to question whether the standard formulation captures the true engineering need, and to recognize when a fundamentally different approach is required. **When you understand the essence of a problem, the solution — and often an entirely new solution nobody anticipated — follows naturally.**

### Outcome-Based Learning

Each chapter opens with a **Learning Outcomes** table aligned to Bloom's Taxonomy — from *Remember* (define key terminology) through *Create* (design a complete controller). These outcomes are not decorative; the exercises at the end of each chapter are explicitly mapped to them. An instructor can assess whether students have achieved each level, and students can self-evaluate their mastery.

---

## How This Book Is Organized

The book is structured in five parts, progressing from foundations to frontiers:

```
PART I — FOUNDATIONS (Chapters 1–3)
  Ch 1: Introduction to Control Systems
  Ch 2: Mathematical Modeling of Dynamic Systems
  Ch 3: Laplace Transform and Transfer Functions

PART II — CLASSICAL CONTROL (Chapters 4–8)
  Ch 4: Time-Domain Analysis (transient response, stability, Routh-Hurwitz)
  Ch 5: Root Locus Analysis and Design
  Ch 6: Frequency Response (Bode plots, gain and phase margins)
  Ch 7: Nyquist Stability Criterion
  Ch 8: Frequency-Domain Controller Design (lead, lag, PID)

PART III — FROM DATA TO MODELS (Chapter 9)
  Ch 9:  System Identification — From Measured Data to Mathematical Models

PART IV — MODERN CONTROL (Chapters 10–13)
  Ch 10: State-Space Representation
  Ch 11: State-Space Analysis (controllability, observability, stability)
  Ch 12: State Feedback and Pole Placement
  Ch 13: Observers and Kalman Filtering

PART V — ADVANCED TOPICS (Chapters 14–17)
  Ch 14: Digital Control Systems
  Ch 15: Optimal Control (LQR, LQG, Kalman-Bucy)
  Ch 16: Robust Control (H∞, μ-synthesis, structured uncertainty)
  Ch 17: Nonlinear Control (Lyapunov, describing functions, sliding mode)

PART VI — FRONTIERS AND APPLICATIONS
  Ch 18: Control Engineering in the Agentic AI Era
  App A: Integrated Design Case Study — EV Traction Drive
  App B: Robotics Systems
  App C: CppPlot Library Reference
  App D: C++ Programming for Control Engineers
```

**Parts I–II** (Chapters 1–8) cover a standard one-semester undergraduate course in classical control. **Parts I–IV** (Chapters 1–13) cover a two-semester sequence, with Chapter 9 (System Identification) bridging classical and modern control by teaching students how to obtain models from measured data. **Parts V–VI** are suitable for a graduate course or for self-study by practicing engineers.

### Dependency Map

```
Ch1 ──▶ Ch2 ──▶ Ch3 ──▶ Ch4 ──▶ Ch5 ──▶ Ch8
                  │       │              ▲
                  │       └──▶ Ch6 ──▶ Ch7 ─┘
                  │
                  └──▶ Ch8 ──▶ Ch9 (System Identification)
                                  │
                  ┌──────────────┘
                  ▼
                 Ch10 ──▶ Ch11 ──▶ Ch12 ──▶ Ch13
                               │          │
                               ▼          ▼
                             Ch14       Ch15 ──▶ Ch16
                                                   │
                             Ch17 ◀──────────────┘
                               │
                             Ch18 (requires Ch1-Ch17)
```

---

## Prerequisites

This textbook assumes:

- **Mathematics:** Calculus (derivatives, integrals, Taylor series), ordinary differential equations, basic linear algebra (matrices, eigenvalues, determinants). A review of Laplace transforms is provided in Chapter 3.
- **Programming:** Working knowledge of C++ fundamentals — variables, functions, loops, classes, and the standard library. No template metaprogramming or advanced C++ is required. Appendix D provides a refresher tailored to the control systems context.
- **Engineering:** Basic physics (Newton's laws, Kirchhoff's laws, energy conservation). Prior exposure to circuit analysis or dynamics is helpful but not strictly necessary — all models are derived from scratch.

---

## How to Use This Book

### For Instructors

- **One-semester undergraduate course:** Chapters 1–8 (classical control). Assign 4–5 exercises per chapter. Use Appendix A as a capstone project.
- **Two-semester sequence:** Add Chapters 9–15. The second semester begins with system identification (Ch9), proceeds to state-space (Ch10–13), digital control (Ch14), and optimal control (Ch15).
- **Graduate course:** Chapters 15–18 (optimal, robust, nonlinear, AI-era control), with Chapters 10–13 as prerequisites. Chapter 18 is ideal for seminar-style discussions.
- **Lab component:** Every chapter includes compilable code examples. Students can reproduce every plot in the book by typing the code and running it.

### For Students

- **Read actively:** Don't skip the derivations. The mathematical reasoning is what makes control engineering predictive rather than empirical.
- **Type the code:** Don't just read the code examples — type them, compile them, run them, and then *change something*. What happens when you double the damping ratio? What if the gain margin goes negative?
- **Do the exercises:** They are ordered by Bloom's level. If you can complete the ⭐⭐⭐ exercises, you have mastered the chapter.
- **Refer to Appendix C frequently:** It is the API reference for every function used in the book. When you see `margin(G)` in a code example, look it up in Appendix C to understand all available options.

### For Practicing Engineers

- **Focus on Chapters 14–18** if you already know classical and modern control. Digital control (Ch14), robust control (Ch16), and the AI chapter (Ch18) address the most relevant industrial topics.
- **Use Appendix A** as a template for your own integrated design projects.
- **Use the code** as starting points for your own implementations. The CppPlot library is header-only and permissively licensed.

---

## On the Choice of C++

The choice of C++ over MATLAB or Python deserves explicit justification, because it is the most consequential pedagogical decision in this book.

**MATLAB** is the standard tool in control education, and for good reason: its Control System Toolbox provides a mature, well-tested API. But MATLAB is proprietary, expensive for institutions in many countries, and — most importantly — not the language that runs on embedded systems. A student who learns `tf`, `bode`, and `step` in MATLAB must later translate that knowledge to C/C++ for deployment, often encountering subtle issues with numerical precision, memory management, and real-time constraints for the first time.

**Python** (with `python-control`) is an excellent free alternative and is increasingly used in control education. However, Python's dynamic typing and garbage collection make it ill-suited for real-time embedded control, and the `python-control` library, while growing, lacks the completeness of MATLAB's toolbox.

**C++** offers a unique combination:
- **Performance:** Deterministic execution time, no garbage collector, direct hardware access.
- **Industry relevance:** The de facto language for embedded control (automotive, aerospace, industrial automation, robotics).
- **Modern features:** C++17 structured bindings (`auto [t, y] = step_data(G)`) make the code almost as readable as MATLAB.
- **Self-contained:** The CppPlot library has zero external dependencies. Students need only a C++17 compiler and a text editor.

The trade-off is a steeper initial learning curve. We mitigate this with Appendix D (C++ refresher), carefully scaffolded code examples that build in complexity, and the CppPlot library's deliberate similarity to MATLAB's API (`TransferFunction`, `bode()`, `step()`, `lqr()`, `margin()`).

---

## Acknowledgments

This textbook stands on the shoulders of giants. The mathematical treatments owe much to:
- **Katsuhiko Ogata**, whose *Modern Control Engineering* has educated generations of control engineers with its clarity and rigor.
- **Gene Franklin, J. David Powell, and Abbas Emami-Naeini**, whose *Feedback Control of Dynamic Systems* set the standard for integrating theory with design.
- **Karl Johan Åström and Richard Murray**, whose *Feedback Systems: An Introduction for Scientists and Engineers* demonstrated that control theory can be made accessible without sacrificing depth.
- **Lennart Ljung**, whose *System Identification: Theory for the User* is the definitive treatment and inspired Chapter 9.
- **Kemin Zhou, John Doyle, and Keith Glover**, whose *Robust and Optimal Control* provided the mathematical foundations for Chapters 15–16.
- **Hassan Khalil**, whose *Nonlinear Systems* is the definitive reference for Chapter 17.

The EE/Telecom applications draw from the author's experience in power electronics and motor drive control, and from the excellent treatments in:
- **Ned Mohan**, *Power Electronics* — for converter modeling and control.
- **Peter Vas**, *Sensorless Vector and Direct Torque Control* — for motor drive applications.

The CppPlot library was developed as a teaching tool and reflects the author's belief that students learn best when they can see the correspondence between mathematics and code on the same page.

I am grateful to colleagues and students who tested early drafts of these chapters and identified errors, ambiguities, and missing explanations. Their feedback transformed a set of lecture notes into a textbook. Special thanks to the reviewers who performed four rounds of comprehensive auditing, catching mathematical errors, API inconsistencies, and pedagogical gaps across all 22 chapters and appendices.

---

## A Note on AI and the Future

Chapter 18 — *Control Engineering in the Agentic AI Era* — addresses the elephant in the room: if AI can design controllers, why learn control theory?

The surface-level answer is that AI does not replace control theory any more than calculators replaced mathematics. But the deeper answer requires us to revisit the distinction between **science**, **engineering**, and **technology** introduced earlier:

- **Technology** (tools, code, algorithms) changes rapidly — and AI is the most powerful new technology in a generation.
- **Science** (stability theory, Lyapunov analysis, Shannon's theorem) does not change. These are truths about nature.
- **Engineering** (identifying the right problem, formulating it correctly, judging solutions) endures across every shift in technology.

AI excels at the lower levels of the engineering skill hierarchy:

| Level | Skill | AI's Current Reach |
|-------|-------|--------------------|
| 1 | **Execution** — translating a design into working code | AI does this well, and improving rapidly |
| 2 | **Problem-solving** — given a well-posed problem, finding the optimal solution | AI is competitive and advancing |
| 3 | **Problem identification** — recognizing *which* problem to solve and *why* | Irreducibly human |
| 4 | **Original thinking** — seeing connections, structures, and possibilities that nobody has articulated | Irreducibly human |

The era when coding speed defined a competent engineer is passing. What remains irreplaceable is the ability to *think clearly* about what the machine should compute and why — to identify the real problem before anyone writes (or generates) a single line of code. Recall the Postal Mail Parable: the engineer who sees that the real problem is "transmit the content" rather than "deliver the letter faster" does not need a faster horse. They invent the telegraph.

This has profound implications for control engineering education. A student who learns this book should emerge not just as someone who *can* design a PID controller — AI can do that. They should emerge as someone who:

- **Identifies** that the real problem in a power grid is not frequency regulation but cascading failure prevention under renewable intermittency — and recognizes that this requires robust $H_\infty$ design, not just proportional control.
- **Questions** whether the standard formulation is appropriate: "Is this system truly linear? Is the disturbance really Gaussian? Does our model capture the coupling between thermal and electrical dynamics?"
- **Connects** across disciplines: seeing that a PLL is a feedback system (Chapter 1), that a Kalman filter is Bayesian inference (Chapter 13), that LQR and reinforcement learning minimize the same cost function (Chapter 15/18).
- **Judges** AI-generated designs: verifying stability margins on a Bode plot, checking Lyapunov certificates, ensuring robustness bounds — because an AI that proposes a controller without a stability guarantee has produced something beautiful but potentially lethal.

The engineer of 2030 will not choose between classical control and AI. They will wield both — using Bode plots to certify robustness, LQR to guarantee optimality, $H_\infty$ to bound worst-case performance, and reinforcement learning to adapt to conditions no designer anticipated. But the *irreplaceable* contribution of the human engineer is the act of original thinking: framing the problem, choosing the formulation, judging the solution, and bearing responsibility for the result.

**Technology will change. Science will not.** This book equips you with both — the science that endures and the engineering judgment that no machine can replicate. AI can compute a gain margin. Only you can decide whether that margin is *sufficient* for a system where human lives are at stake.

---

*The best control system is invisible: it performs so well you forget it exists. The best textbook does the same — it removes every barrier to understanding until the ideas feel inevitable. That is the aspiration of this book. Whether it succeeds is for you to judge.*

\vspace{1cm}

\hfill *Tri-Vien Vu (PhD)*

\hfill *February 2026*
