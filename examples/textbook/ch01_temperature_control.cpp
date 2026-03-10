/**
 * @file ch01_temperature_control.cpp
 * @brief Chapter 1: HOW Feedback Works — A Room Temperature Control System
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  PEDAGOGICAL NOTE — "How", not "How to"
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * This example does NOT teach "how to implement temperature control."
 * It teaches HOW feedback control works — the physics, the mechanism,
 * the reason a closed-loop system rejects disturbances while an
 * open-loop system cannot.
 *
 * The difference is fundamental:
 *   "How to" = a recipe.   ("Step 1: measure temperature. Step 2: ...")
 *   "How"    = a mechanism. ("Feedback works BECAUSE the error signal
 *                            carries information about the disturbance's
 *                            effect, and the controller acts on that
 *                            information to compensate.")
 *
 * A student who knows "how to" can follow instructions.
 * A student who knows "how" can design new systems from scratch.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  THE BLOCK DIAGRAM — with physical meaning of EVERY signal
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *              d(t) [°C]
 *              "outside temperature change"
 *              (someone opens a window, weather shifts)
 *              Hardware: the environment itself — NOT under our control
 *                ↓
 *   r(t) ──►( ⊕ )──► Controller ──► u(t) ──► Plant (Room) ──► y(t)
 *   T_ref    (−)↑    (thermostat)    Q(t)    C·dT/dt = Q      T_room
 *   [°C]     │  │     on/off         [W]    −(T−T_out)/R      [°C]
 *   "the     │  │     decision               ↑                "the
 *    user's  │  │                     Heat flows OUT            actual
 *    wish"   │  │                     through walls             room
 *            │  │                                               temp"
 *            │  └──── e(t) = r(t) − y(t)
 *            │        [°C]
 *            │        "the temperature DEFICIT"
 *            │        HOW MANY DEGREES too cold (or hot) the room is.
 *            │        This is the INFORMATION CHANNEL — the controller
 *            │        cannot see the disturbance directly; it can only
 *            │        see the disturbance's EFFECT through e(t).
 *            │
 *            └──── Sensor (thermistor → voltage → ADC → µC) ◄── y(t)
 *                  H(s) ≈ 1 (ideal sensor, fast compared to room τ)
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  SIGNAL DICTIONARY — every signal named, unitized, physically grounded
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  Signal │ Name        │ Unit │ Physical Meaning            │ Hardware
 *  ───────┼─────────────┼──────┼─────────────────────────────┼───────────────
 *  r(t)   │ Reference   │ °C   │ Desired room temperature.   │ Thermostat dial
 *         │ (setpoint)  │      │ Encodes the USER'S INTENT.  │ → potentiometer
 *         │             │      │ "I want 22°C."              │ → ADC → µC
 *  ───────┼─────────────┼──────┼─────────────────────────────┼───────────────
 *  e(t)   │ Error       │ °C   │ Temperature deficit:        │ Computed in µC:
 *         │             │      │ "how far from comfort."     │ e = r − y
 *         │             │      │ THIS IS THE KEY SIGNAL.     │ (firmware
 *         │             │      │ It carries disturbance info │ subtraction)
 *         │             │      │ → the controller's only     │
 *         │             │      │   window into the world.    │
 *  ───────┼─────────────┼──────┼─────────────────────────────┼───────────────
 *  u(t)   │ Control     │ W    │ Heater electrical power.    │ Relay or MOSFET
 *         │ (= Q(t))    │      │ = rate of heat energy       │ switching a 50W
 *         │             │      │   flowing INTO the room.    │ ceramic element.
 *         │             │      │ This is what physically     │ ON = 50W,
 *         │             │      │ CHANGES the temperature.    │ OFF = 0W.
 *  ───────┼─────────────┼──────┼─────────────────────────────┼───────────────
 *  y(t)   │ Output      │ °C   │ Actual room temperature.    │ NTC thermistor
 *         │ (= T_room)  │      │ The physical RESULT of all  │ → voltage
 * divider │             │      │ heat flows: heater in,      │ → ADC → µC │ │
 * │ walls out, disturbance.     │ (e.g. 10-bit, │             │      │ │
 * sampled at 1Hz)
 *  ───────┼─────────────┼──────┼─────────────────────────────┼───────────────
 *  d(t)   │ Disturbance │ °C   │ Change in outside temp.     │ NOT a designed
 *         │             │      │ We do NOT control this.     │ signal — it is
 *         │             │      │ It enters through walls     │ the environment
 *         │             │      │ as unwanted heat loss.      │ acting on us.
 *  ═══════════════════════════════════════════════════════════════════════════
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  THE PHYSICS — WHY this equation, not just WHAT equation
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * The room obeys Newton's Law of Cooling (energy conservation):
 *
 *     C · dT/dt = Q_in − Q_out
 *
 * where:
 *   C · dT/dt = rate of change of stored thermal energy [J/s = W]
 *               (the room's thermal mass absorbs or releases heat)
 *   Q_in = Q(t) = heater power [W]
 *               (electrical energy → heat via resistive element)
 *   Q_out = (T_room − T_outside) / R   [W]
 *               (heat flows OUT through walls; the bigger the ΔT,
 *                the faster the heat escapes — Fourier's law)
 *
 * Rearranging:
 *     C · dT/dt = Q(t) − (T − T_outside) / R
 *
 * This is a FIRST-ORDER ODE with time constant τ = R·C.
 * τ = 500s means the room "remembers" its temperature for ~8 minutes.
 * After ~3τ = 1500s, the room is within 5% of its new steady state.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  THE MECHANISM OF DISTURBANCE REJECTION — the "How"
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * WHY does feedback reject disturbances? Not "it just does." Here's HOW:
 *
 * 1. Disturbance hits: T_outside drops from 5°C to −5°C (cold front).
 * 2. Physics: Heat loss Q_out = (T − T_out)/R INCREASES (larger ΔT).
 *    More heat escapes through the walls.
 * 3. Consequence: T_room drops. The room gets colder.
 * 4. Sensor detects: y(t) = T_room decreases.
 * 5. Error GROWS: e(t) = T_ref − y(t) = 22 − (now colder) = BIGGER.
 *    The error signal now CARRIES INFORMATION about the disturbance.
 *    Nobody told the controller "it got cold outside."
 *    But the controller sees the EFFECT: "the room is too cold."
 * 6. Controller acts: e(t) > threshold → heater stays ON longer.
 *    u(t) = Q(t) increases (more duty cycle for on-off, or higher
 *    power for proportional control).
 * 7. Compensation: Extra heat input offsets the extra heat loss.
 *    Temperature recovers toward T_ref.
 *
 * This is the MECHANISM: the error signal is the information channel
 * through which the controller LEARNS about disturbances — not by
 * measuring the disturbance directly, but by observing its EFFECT
 * on the output.
 *
 * In open-loop: Steps 4–7 DON'T EXIST. The heater is blind.
 * It outputs a fixed Q_ss = 34W regardless. When the disturbance
 * hits, the room cools, and nobody notices. Nobody compensates.
 * The temperature drifts permanently.
 *
 * Compile: g++ -std=c++17 -I "../../include" ch01_temperature_control.cpp -o
 * ch01_temp
 */

#include <cmath>
#include <cppplot/cppplot.hpp>
#include <iomanip>
#include <iostream>
#include <vector>


using namespace cppplot;

int main() {
  std::cout
      << "╔══════════════════════════════════════════════════════════════╗\n";
  std::cout
      << "║   Chapter 1: HOW Feedback Works — Temperature Control        ║\n";
  std::cout
      << "║   Teaching the MECHANISM, not a recipe                        ║\n";
  std::cout
      << "╚══════════════════════════════════════════════════════════════╝\n";

  // ════════════════════════════════════════════════════════════════════════
  // THE PLANT: A room with walls, air, and a heater
  // ════════════════════════════════════════════════════════════════════════
  //
  // Every physical parameter has a name, a unit, and a PHYSICAL ORIGIN:
  //
  //   C [J/°C] — Thermal capacitance of the room.
  //              WHERE IT COMES FROM: the mass of air, furniture, walls.
  //              A large room with heavy furniture → large C → slow response.
  //              A small server closet → small C → fast response.
  //              Physically: how many Joules of heat energy it takes to
  //              raise the room temperature by 1°C.
  //
  //   R [°C/W] — Thermal resistance of the walls.
  //              WHERE IT COMES FROM: wall thickness, insulation material,
  //              window area, door seals.
  //              Well-insulated room → large R → less heat escapes per °C.
  //              Single-pane windows → small R → heat pours out.
  //              Physically: the temperature difference in °C needed to
  //              drive 1 Watt of heat flow through the walls.
  //
  //   τ = R·C [s] — Time constant. NOT an independent parameter —
  //              it EMERGES from the physics (capacitance × resistance).
  //              MEANING: how long the room "remembers" a temperature change.
  //              After one τ, the room has moved 63% toward steady state.
  //              After 3τ, it's 95% there. After 5τ, essentially done.

  const double C = 1000.0;  // Thermal capacitance [J/°C]
  const double R = 0.5;     // Thermal resistance  [°C/W]
  const double tau = C * R; // Time constant       [s]   (= 500s ≈ 8.3 min)

  // ════════════════════════════════════════════════════════════════════════
  // THE SIGNALS — every one physically grounded
  // ════════════════════════════════════════════════════════════════════════
  //
  //   r(t) = T_ref — the reference (setpoint).
  //     PHYSICAL MEANING: the temperature the occupant WANTS.
  //     HARDWARE: user turns a thermostat dial. Inside: a potentiometer
  //     produces a voltage proportional to desired temperature.
  //     An ADC digitizes it. The µC stores T_ref = 22.
  //
  //   y(t) = T_room — the output (process variable).
  //     PHYSICAL MEANING: the actual temperature of the air in the room.
  //     HARDWARE: an NTC thermistor (resistance varies with temperature)
  //     in a voltage divider. The ADC reads the voltage. The µC converts
  //     it to °C using a calibration table. Sampled at ~1 Hz (fast enough
  //     because the room's τ = 500s — no aliasing risk).
  //
  //   e(t) = r(t) − y(t) — the error signal.
  //     PHYSICAL MEANING: the temperature DEFICIT. If e > 0, the room is
  //     colder than desired. If e < 0, it's warmer.
  //     CRITICAL INSIGHT: This is the controller's ONLY window into the world.
  //     The controller does not know WHY the room is cold. It only knows
  //     HOW MUCH too cold it is. Disturbances, model errors, sensor drift —
  //     all are visible to the controller ONLY through their effect on e(t).
  //     HARDWARE: computed as a subtraction in firmware: e = T_ref - T_meas.
  //
  //   u(t) = Q(t) — the control signal (actuator command).
  //     PHYSICAL MEANING: heater electrical power [Watts].
  //     This is the physical ACTION that changes the temperature.
  //     ENERGY FLOW: electrical power → resistive heating element → heat
  //     → transferred to room air by convection.
  //     HARDWARE (on-off): a relay or MOSFET switches the heater.
  //       ON  → Q = Q_max = 50W (full power to the ceramic element)
  //       OFF → Q = 0W
  //     HARDWARE (proportional): a PWM signal with variable duty cycle.
  //       duty = 60% → Q = 0.6 × 50 = 30W average power.

  const double T_ref = 22.0;     // r(t): desired temperature [°C]
  const double T_outside = 5.0;  // nominal outside temperature [°C]
  const double T_initial = 15.0; // y(0): initial room temperature [°C]

  // ════════════════════════════════════════════════════════════════════════
  // OPEN-LOOP: compute the EXACT heater power needed (no feedback)
  // ════════════════════════════════════════════════════════════════════════
  //
  // At steady state, dT/dt = 0, so:
  //     0 = Q_ss − (T_ref − T_outside) / R
  //     Q_ss = (22 − 5) / 0.5 = 34 W
  //
  // This is the ONLY heater power that produces exactly T = 22°C
  // when T_outside = 5°C. It is calculated FROM THE MODEL, not measured.
  //
  // THE FUNDAMENTAL PROBLEM OF OPEN-LOOP:
  // This calculation assumes we know R, T_outside, and T_ref perfectly.
  // If ANY of these assumptions break (insulation degrades, weather changes,
  // someone opens a door), the heater still outputs 34W — it CAN'T adapt
  // because it has NO SENSOR, hence NO INFORMATION about reality.

  const double Q_ss = (T_ref - T_outside) / R; // = 34 W (exactly)
  const double Q_max = 50.0;                   // heater maximum [W]

  std::cout << "\n┌──────────────────────────────────────────────────────┐\n";
  std::cout << "│              SYSTEM PHYSICAL PARAMETERS               │\n";
  std::cout << "├──────────────────────────────────────────────────────┤\n";
  std::cout << std::fixed << std::setprecision(1);
  std::cout << "│ C (thermal mass of room)     = " << std::setw(7) << C
            << " J/°C  │\n";
  std::cout << "│ R (wall insulation)          = " << std::setw(7) << R
            << " °C/W  │\n";
  std::cout << "│ τ = R·C (room time constant) = " << std::setw(7) << tau
            << " s     │\n";
  std::cout << "│                               ≈ " << std::setw(7)
            << tau / 60.0 << " min   │\n";
  std::cout << "├──────────────────────────────────────────────────────┤\n";
  std::cout << "│            BLOCK DIAGRAM SIGNALS                     │\n";
  std::cout << "├──────────────────────────────────────────────────────┤\n";
  std::cout << "│ r(t) = T_ref    = " << std::setw(5) << T_ref
            << " °C  (user's wish)      │\n";
  std::cout << "│ y(0) = T_init   = " << std::setw(5) << T_initial
            << " °C  (starting temp)     │\n";
  std::cout << "│ e(0) = r − y    = " << std::setw(5) << T_ref - T_initial
            << " °C  (initial deficit)   │\n";
  std::cout << "│ d    = T_out    = " << std::setw(5) << T_outside
            << " °C  (environment)       │\n";
  std::cout << "│ u_ss = Q_ss     = " << std::setw(5) << Q_ss
            << " W   (open-loop power)   │\n";
  std::cout << "│ u_max = Q_max   = " << std::setw(5) << Q_max
            << " W   (heater capacity)   │\n";
  std::cout << "└──────────────────────────────────────────────────────┘\n";

  // ════════════════════════════════════════════════════════════════════════
  // SIMULATION SETUP
  // ════════════════════════════════════════════════════════════════════════
  const double t_final =
      3000.0; // simulate for 3000s ≈ 6τ (well past steady state)
  const int n_steps = 500;
  const double dt = t_final / n_steps; // 6s per step

  std::vector<double> time(n_steps + 1);
  for (int i = 0; i <= n_steps; ++i)
    time[i] = i * dt;

  // We'll record EVERY signal at each time step — not just temperature,
  // but error, control effort, and heat loss — so we can trace the
  // MECHANISM of feedback through the data.

  // ════════════════════════════════════════════════════════════════════════
  // SCENARIO 1: OPEN-LOOP — the heater is BLIND
  // ════════════════════════════════════════════════════════════════════════
  //
  // HOW IT WORKS (or rather, how it FAILS to work):
  //   The heater outputs a FIXED Q_ss = 34W, calculated from the model.
  //   There is NO sensor. NO error signal. NO feedback path.
  //   The heater does not know what the room temperature is.
  //   If the model is perfect and nothing changes, this works.
  //   If anything changes... it can't adapt.

  std::cout << "\n▶ Scenario 1: OPEN-LOOP (heater is blind — fixed " << Q_ss
            << "W)\n";

  std::vector<double> T_openloop(time.size());
  std::vector<double> e_openloop(time.size()); // error (for analysis)
  std::vector<double> Q_openloop(time.size()); // control effort
  double T = T_initial;

  for (size_t i = 0; i < time.size(); ++i) {
    T_openloop[i] = T;
    e_openloop[i] = T_ref - T; // error exists physically, but nobody reads it!
    Q_openloop[i] = Q_ss;      // fixed: the heater doesn't know e(t)

    // Plant physics: energy balance
    //   Q_in  = Q_ss = 34W (constant — the heater is blind)
    //   Q_out = (T − T_outside)/R   (heat lost through walls)
    //   Net   = Q_in − Q_out        (if positive, room warms; if negative,
    //   cools)
    double Q_out = (T - T_outside) / R;
    double dTdt = (Q_ss - Q_out) / C;
    T += dTdt * dt;
  }

  // ════════════════════════════════════════════════════════════════════════
  // SCENARIO 2: CLOSED-LOOP (on-off thermostat) — the heater can SEE
  // ════════════════════════════════════════════════════════════════════════
  //
  // HOW IT WORKS (the mechanism):
  //   Every time step, the sensor measures y(t) = T_room.
  //   The µC computes e(t) = T_ref − T_room.
  //   The controller makes a DECISION based on e(t):
  //     If e(t) > +0.5°C (room too cold by half a degree) → heater ON
  //     If e(t) < −0.5°C (room too warm by half a degree) → heater OFF
  //     Otherwise → keep previous state (hysteresis prevents chattering)
  //
  //   WHY HYSTERESIS? Without it, at exactly T = T_ref, the heater would
  //   switch on-off-on-off every time step — relay chatter that destroys
  //   the relay contacts and wastes energy. The ±0.5°C deadband is a
  //   DESIGN TRADE-OFF: accept ±0.5°C steady-state ripple in exchange
  //   for reasonable switching frequency.

  std::cout
      << "▶ Scenario 2: CLOSED-LOOP (on-off thermostat with ±0.5°C deadband)\n";

  const double hysteresis = 0.5; // deadband [°C] — prevents relay chatter

  std::vector<double> T_closedloop(time.size());
  std::vector<double> e_closedloop(time.size());
  std::vector<double> Q_closedloop(time.size());
  T = T_initial;
  bool heater_on = true; // start ON because room is cold (e(0) = +7°C)

  for (size_t i = 0; i < time.size(); ++i) {
    T_closedloop[i] = T;

    // ── Step 1: SENSE — the sensor measures reality ──
    double y = T; // thermistor reads room temperature [°C]

    // ── Step 2: COMPARE — compute the error (the information signal) ──
    double e = T_ref - y; // e > 0 means "too cold"
    e_closedloop[i] = e;

    // ── Step 3: DECIDE — controller acts on the error ──
    // The controller does NOT know T_outside, does NOT know the model.
    // It knows ONLY the error e(t). That is enough.
    if (e > +hysteresis)
      heater_on = true; // room too cold → heat
    if (e < -hysteresis)
      heater_on = false; // room too warm → stop

    double Q = heater_on ? Q_max : 0.0;
    Q_closedloop[i] = Q;

    // ── Step 4: ACTUATE — the heater changes the physics ──
    double Q_out = (T - T_outside) / R;
    double dTdt = (Q - Q_out) / C;
    T += dTdt * dt;

    // ── The loop closes: new T → new y → new e → new decision ──
  }

  // ════════════════════════════════════════════════════════════════════════
  // SCENARIO 3: OPEN-LOOP + DISTURBANCE — the heater stays blind
  // ════════════════════════════════════════════════════════════════════════
  //
  // At t = 1000s, a cold front arrives: T_outside drops from 5°C to −5°C.
  //
  // WHAT HAPPENS (the physics):
  //   Heat loss Q_out = (T − T_outside)/R DOUBLES because ΔT doubles.
  //   The heater still outputs 34W (it doesn't know anything changed).
  //   Energy balance: Q_in < Q_out → net heat flow is OUTWARD → room cools.
  //   The heater was sized for T_outside = 5°C. At T_outside = −5°C,
  //   it would need Q_ss = (22 − (−5)) / 0.5 = 54W — but it only has 34W.
  //   The room will settle at a new equilibrium where Q_in = Q_out:
  //     34 = (T_new − (−5)) / 0.5  →  T_new = 34 × 0.5 + (−5) = 12°C.
  //   That's 10°C below the setpoint. Information-less control fails.

  std::cout
      << "▶ Scenario 3: OPEN-LOOP + disturbance (T_outside drops at t=1000s)\n";

  std::vector<double> T_ol_dist(time.size());
  std::vector<double> T_outside_profile(time.size());
  std::vector<double> Q_loss(time.size()); // heat lost through walls
  T = T_initial;

  for (size_t i = 0; i < time.size(); ++i) {
    T_ol_dist[i] = T;

    // Disturbance: cold front arrives at t = 1000s
    double T_out_now = (time[i] < 1000.0) ? T_outside : (T_outside - 10.0);
    T_outside_profile[i] = T_out_now;

    // Physics: heat escapes faster when ΔT is larger
    double Q_out = (T - T_out_now) / R;
    Q_loss[i] = Q_out;

    double dTdt = (Q_ss - Q_out) / C; // Q_ss is still 34W — blind!
    T += dTdt * dt;
  }

  // ════════════════════════════════════════════════════════════════════════
  // SCENARIO 4: CLOSED-LOOP + DISTURBANCE — watch the mechanism in action
  // ════════════════════════════════════════════════════════════════════════
  //
  // Same disturbance, but now the feedback loop is active.
  //
  // THE MECHANISM IN DETAIL (this is the core "How" of the entire chapter):
  //
  //   t < 1000s: Steady state. T ≈ 22°C, e ≈ 0, heater cycles on/off
  //              at moderate duty cycle (~68% = 34W/50W average).
  //
  //   t = 1000s: Cold front hits. T_outside drops to −5°C.
  //              Q_out increases (more heat escapes through walls).
  //              Nothing immediate happens to Q_in — the heater is
  //              in whatever state it was.
  //
  //   t ≈ 1010s: Room temperature starts to drop. Slowly at first
  //              (C = 1000 J/°C is large; thermal mass resists change).
  //
  //   t ≈ 1050s: y(t) drops enough that e(t) = T_ref − y(t) exceeds
  //              the hysteresis band. The error signal has now ABSORBED
  //              information about the disturbance.
  //
  //   t ≈ 1050s+: Controller responds: heater stays ON more often.
  //              Duty cycle increases from ~68% toward ~100%.
  //              Q_in increases toward 50W.
  //
  //   t → ∞:    New steady state. The heater runs at higher duty cycle
  //              to compensate for the increased heat loss. Temperature
  //              recovers to within ±0.5°C of T_ref.
  //              (If Q_max ≥ Q_needed = 54W, recovery is complete.
  //               With Q_max = 50W, there's a small residual error —
  //               the heater is slightly undersized for this disturbance.)
  //
  //   CRITICAL OBSERVATION: The controller never measured T_outside.
  //   It never knew a cold front arrived. It only saw its EFFECT:
  //   "the room is getting cold." And that was ENOUGH to compensate.
  //   This is the power — and the beauty — of feedback.

  std::cout << "▶ Scenario 4: CLOSED-LOOP + disturbance (same cold front, with "
               "feedback)\n";

  std::vector<double> T_cl_dist(time.size());
  std::vector<double> e_cl_dist(time.size());
  std::vector<double> Q_cl_dist(time.size());
  T = T_initial;
  heater_on = true;

  for (size_t i = 0; i < time.size(); ++i) {
    T_cl_dist[i] = T;

    double T_out_now = T_outside_profile[i];

    // SENSE → COMPARE → DECIDE → ACTUATE  (the feedback loop)
    double y = T;
    double e = T_ref - y;
    e_cl_dist[i] = e;

    if (e > +hysteresis)
      heater_on = true;
    if (e < -hysteresis)
      heater_on = false;

    double Q = heater_on ? Q_max : 0.0;
    Q_cl_dist[i] = Q;

    double Q_out = (T - T_out_now) / R;
    double dTdt = (Q - Q_out) / C;
    T += dTdt * dt;
  }

  // ════════════════════════════════════════════════════════════════════════
  // VISUALIZATION — not just plots, but PHYSICAL STORIES
  // ════════════════════════════════════════════════════════════════════════
  std::cout << "\n▶ Generating plots...\n";

  // ── Plot 1: Open-Loop vs Closed-Loop (no disturbance) ──
  // WHAT TO SEE: Both reach ~22°C, but the closed-loop oscillates ±0.5°C
  // (the price of on-off control). The open-loop is smooth but fragile.
  figure(1000, 600);

  plot(time, T_openloop, "b-",
       {{"linewidth", "2"}, {"label", "Open-Loop (blind, Q=34W fixed)"}});
  plot(time, T_closedloop, "r-",
       {{"linewidth", "2"}, {"label", "Closed-Loop (on-off, sees e(t))"}});
  axhline(T_ref, {{"color", "green"},
                  {"linestyle", "--"},
                  {"linewidth", "1.5"},
                  {"label", "r(t) = T_ref = 22°C"}});

  xlabel("Time [s]");
  ylabel("y(t) = Room Temperature [°C]");
  title("Why Feedback? Both work here — the difference is ROBUSTNESS");
  legend();
  grid(true);
  ylim(10, 28);

  savefig("ch01_ol_vs_cl_basic.svg");
  std::cout << "  ✓ Saved ch01_ol_vs_cl_basic.svg\n";

  // ── Plot 2: The error signal — the information channel ──
  // WHAT TO SEE: In open-loop, e(t) decays to 0 (lucky — the model is perfect).
  // In closed-loop, e(t) oscillates within the hysteresis band (±0.5°C).
  // The on-off controller CANNOT drive e(t) to exactly zero — it lacks the
  // resolution. This motivates proportional/PID control (Chapter 8).
  figure(1000, 500);

  plot(time, e_openloop, "b-",
       {{"linewidth", "2"}, {"label", "e(t) open-loop (nobody reads this!)"}});
  plot(time, e_closedloop, "r-",
       {{"linewidth", "2"},
        {"label", "e(t) closed-loop (controller acts on this)"}});
  axhline(0, {{"color", "gray"}, {"linestyle", "--"}, {"linewidth", "1"}});

  xlabel("Time [s]");
  ylabel("e(t) = T_ref − T_room [°C]");
  title("The Error Signal: The Controller's Window Into the World");
  legend();
  grid(true);

  savefig("ch01_error_signal.svg");
  std::cout << "  ✓ Saved ch01_error_signal.svg\n";

  // ── Plot 3: Disturbance rejection — the core "How" demonstration ──
  // WHAT TO SEE: At t = 1000s, both systems are hit by the same disturbance.
  // Open-loop: temperature drifts to ~12°C (10°C error!). Heater doesn't react.
  // Closed-loop: temperature dips briefly, then recovers. The error signal
  // carried the disturbance information → the controller compensated.
  figure(1200, 900);

  subplot(3, 1, 1);
  plot(time, T_ol_dist, "b-",
       {{"linewidth", "2"}, {"label", "Open-Loop (drifts — heater is blind)"}});
  plot(time, T_cl_dist, "r-",
       {{"linewidth", "2"}, {"label", "Closed-Loop (recovers — sees e(t))"}});
  axhline(T_ref,
          {{"color", "green"}, {"linestyle", "--"}, {"label", "r(t) = 22°C"}});
  axvline(1000, {{"color", "orange"}, {"linestyle", ":"}, {"alpha", "0.7"}});
  ylabel("y(t) [°C]");
  title("HOW Feedback Rejects Disturbances");
  legend();
  grid(true);
  ylim(5, 28);

  // Error signals under disturbance — watch e(t) reveal the disturbance
  subplot(3, 1, 2);
  std::vector<double> e_ol_dist(time.size());
  for (size_t i = 0; i < time.size(); ++i)
    e_ol_dist[i] = T_ref - T_ol_dist[i];
  plot(time, e_ol_dist, "b-",
       {{"linewidth", "2"}, {"label", "e(t) open-loop (grows unchecked)"}});
  plot(time, e_cl_dist, "r-",
       {{"linewidth", "2"}, {"label", "e(t) closed-loop (compensated)"}});
  axvline(1000, {{"color", "orange"}, {"linestyle", ":"}, {"alpha", "0.7"}});
  axhline(0, {{"color", "gray"}, {"linestyle", "--"}, {"linewidth", "1"}});
  ylabel("e(t) [°C]");
  title("Error Signal: Carries Disturbance Information");
  legend();
  grid(true);

  // Disturbance profile (the cause)
  subplot(3, 1, 3);
  plot(time, T_outside_profile, "purple",
       {{"linewidth", "2"}, {"label", "d(t) = T_outside"}});
  axvline(1000, {{"color", "orange"}, {"linestyle", ":"}, {"alpha", "0.7"}});
  xlabel("Time [s]");
  ylabel("d(t) [°C]");
  title("Disturbance: Cold Front Arrives (T_outside drops 10°C)");
  legend();
  grid(true);
  ylim(-10, 10);

  savefig("ch01_disturbance_mechanism.svg");
  std::cout << "  ✓ Saved ch01_disturbance_mechanism.svg\n";

  // ── Plot 4: Control effort — what the actuator DOES ──
  // WHAT TO SEE: Open-loop: fixed 34W regardless. Closed-loop: duty cycle
  // increases after the disturbance — the controller works HARDER to
  // compensate. This is the physical cost of disturbance rejection.
  figure(1000, 600);

  subplot(2, 1, 1);
  std::vector<double> Q_ol_const(time.size(), Q_ss);
  plot(time, Q_ol_const, "b--",
       {{"linewidth", "2"}, {"label", "u(t) open-loop (fixed 34W — blind)"}});
  plot(time, Q_cl_dist, "r-",
       {{"linewidth", "1.5"},
        {"label", "u(t) closed-loop (adapts to disturbance)"}});
  axvline(1000, {{"color", "orange"}, {"linestyle", ":"}, {"alpha", "0.7"}});
  ylabel("u(t) = Q [W]");
  title("Control Effort: The Actuator's Response to Disturbance");
  legend();
  grid(true);
  ylim(-5, 60);

  // Heat loss through walls — the hidden physics
  subplot(2, 1, 2);
  std::vector<double> Q_loss_cl(time.size());
  for (size_t i = 0; i < time.size(); ++i) {
    Q_loss_cl[i] = (T_cl_dist[i] - T_outside_profile[i]) / R;
  }
  plot(time, Q_loss, "b-",
       {{"linewidth", "2"}, {"label", "Q_out open-loop [W]"}});
  plot(time, Q_loss_cl, "r-",
       {{"linewidth", "2"}, {"label", "Q_out closed-loop [W]"}});
  axvline(1000, {{"color", "orange"}, {"linestyle", ":"}, {"alpha", "0.7"}});
  xlabel("Time [s]");
  ylabel("Heat loss through walls [W]");
  title("Hidden Signal: Heat Escaping Through Walls (Fourier's Law)");
  legend();
  grid(true);

  savefig("ch01_control_effort.svg");
  std::cout << "  ✓ Saved ch01_control_effort.svg\n";

  // ════════════════════════════════════════════════════════════════════════
  // RESULTS — not just numbers, but PHYSICAL INTERPRETATION
  // ════════════════════════════════════════════════════════════════════════
  double ss_err_ol = std::abs(T_ref - T_openloop.back());
  double ss_err_cl = std::abs(T_ref - T_closedloop.back());
  double ss_err_ol_dist = std::abs(T_ref - T_ol_dist.back());
  double ss_err_cl_dist = std::abs(T_ref - T_cl_dist.back());

  std::cout << std::fixed << std::setprecision(2);
  std::cout << "\n╔════════════════════════════════════════════════════════════"
               "══════════╗\n";
  std::cout << "║                       RESULTS AND INTERPRETATION             "
               "         ║\n";
  std::cout << "╠══════════════════════════════════════════════════════════════"
               "════════╣\n";
  std::cout << "║                              │  Open-Loop    │  Closed-Loop  "
               "        ║\n";
  std::cout << "║  "
               "────────────────────────────┼───────────────┼──────────────────"
               "─────║\n";
  std::cout << "║  Normal: y(∞)                │  " << std::setw(6)
            << T_openloop.back() << " °C    │  " << std::setw(6)
            << T_closedloop.back() << " °C (±0.5°C)       ║\n";
  std::cout << "║  Normal: |e(∞)|              │  " << std::setw(6) << ss_err_ol
            << " °C    │  " << std::setw(6) << ss_err_cl
            << " °C                ║\n";
  std::cout << "║  "
               "────────────────────────────┼───────────────┼──────────────────"
               "─────║\n";
  std::cout << "║  Disturbed: y(∞)             │  " << std::setw(6)
            << T_ol_dist.back() << " °C    │  " << std::setw(6)
            << T_cl_dist.back() << " °C                ║\n";
  std::cout << "║  Disturbed: |e(∞)|           │  " << std::setw(6)
            << ss_err_ol_dist << " °C    │  " << std::setw(6) << ss_err_cl_dist
            << " °C                ║\n";
  std::cout << "╠══════════════════════════════════════════════════════════════"
               "════════╣\n";
  std::cout << "║                                                              "
               "        ║\n";
  std::cout << "║  HOW THE MECHANISM WORKS — in one sentence:                  "
               "        ║\n";
  std::cout << "║                                                              "
               "        ║\n";
  std::cout << "║  The error signal e(t) carries INFORMATION about the "
               "disturbance's   ║\n";
  std::cout << "║  effect on the output. The controller acts on this "
               "information to    ║\n";
  std::cout << "║  adjust u(t), compensating for what it cannot directly "
               "observe.      ║\n";
  std::cout << "║                                                              "
               "        ║\n";
  std::cout << "║  Open-loop has no error signal → no information → no "
               "compensation.   ║\n";
  std::cout << "║                                                              "
               "        ║\n";
  std::cout << "╠══════════════════════════════════════════════════════════════"
               "════════╣\n";
  std::cout << "║  QUESTIONS TO PONDER (not  how to — but how and why) ║\n";
  std::cout << "║                                                      ║\n";
  std::cout << "║  1. If Q_max were only 45W instead of 50W, could the "
               "closed-loop    ║\n";
  std::cout << "║     still maintain 22°C after the cold front? WHY or WHY "
               "NOT?        ║\n";
  std::cout << "║     (Hint: what is Q_needed = (22−(−5))/0.5 ?)               "
               "       ║\n";
  std::cout << "║                                                              "
               "        ║\n";
  std::cout << "║  2. If the sensor had a 60-second delay (slow thermistor), "
               "how       ║\n";
  std::cout << "║     would the error signal e(t) change? Would feedback still "
               "work?   ║\n";
  std::cout << "║     (Hint: e(t) would carry DELAYED information...)          "
               "        ║\n";
  std::cout << "║                                                              "
               "        ║\n";
  std::cout << "║  3. The on-off controller has ±0.5°C ripple. What would a    "
               "        ║\n";
  std::cout << "║     PROPORTIONAL controller (Q = Kp · e(t)) do differently?  "
               "        ║\n";
  std::cout << "║     (This leads to Chapter 8.)                               "
               "        ║\n";
  std::cout << "╚══════════════════════════════════════════════════════════════"
               "════════╝\n";

  std::cout << "\n✅ Chapter 1 demonstration complete.\n";
  std::cout << "   Generated 4 SVG plots, each telling a PHYSICAL story.\n";

  return 0;
}
