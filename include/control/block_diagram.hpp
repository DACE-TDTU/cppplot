/**
 * @file block_diagram.hpp
 * @brief Block diagram algebra and system interconnections
 * 
 * Provides tools for building complex control systems from basic blocks
 */

#ifndef CPPPLOT_CONTROL_BLOCK_DIAGRAM_HPP
#define CPPPLOT_CONTROL_BLOCK_DIAGRAM_HPP

#include "transfer_function.hpp"
#include "state_space.hpp"
#include <vector>
#include <string>
#include <memory>
#include <functional>

namespace cppplot {
namespace control {

// ============================================================
//                    BASIC CONNECTIONS
// ============================================================

// Note: series() and parallel() are defined in transfer_function.hpp

/**
 * @brief Negative feedback connection
 * 
 *     r -->(+)---[G]--->y
 *           ^         |
 *           |---[H]<--|
 * 
 * Closed-loop: T = G / (1 + GH)
 */
inline TransferFunction nfeedback(const TransferFunction& G, const TransferFunction& H) {
    return feedback(G, H, -1);
}

/**
 * @brief Positive feedback connection
 * Closed-loop: T = G / (1 - GH)
 */
inline TransferFunction pfeedback(const TransferFunction& G, const TransferFunction& H) {
    return feedback(G, H, +1);
}

// ============================================================
//                    STANDARD CONTROL LOOPS
// ============================================================

/**
 * @brief Standard feedback control loop
 * 
 *     r -->(+)---[C]---[G]--->y
 *           ^              |
 *           |------[H]<----|
 * 
 * @param G Plant transfer function
 * @param C Controller transfer function
 * @param H Feedback transfer function (default: unity)
 * @return Closed-loop transfer function from r to y
 */
inline TransferFunction closed_loop(
    const TransferFunction& G,
    const TransferFunction& C,
    const TransferFunction& H = TransferFunction({1}, {1})
) {
    TransferFunction L = C * G;  // Loop transfer function
    return feedback(L, H);
}

/**
 * @brief Standard PID control loop
 * 
 * @param G Plant
 * @param Kp Proportional gain
 * @param Ki Integral gain  
 * @param Kd Derivative gain
 * @return Closed-loop transfer function
 */
inline TransferFunction pid_loop(
    const TransferFunction& G,
    double Kp, double Ki, double Kd
) {
    // PID: G(s) = Kp + Ki/s + Kd*s = (Kd*s^2 + Kp*s + Ki) / s
    TransferFunction C({Kd, Kp, Ki}, {1, 0});
    return closed_loop(G, C);
}

/**
 * @brief Two degree-of-freedom controller structure
 * 
 *     r -->[Cr]-->(+)---[Cy]---[G]--->y
 *                  ^               |
 *                  |-------[H]<----|
 * 
 * @param G Plant
 * @param Cr Reference prefilter
 * @param Cy Feedback controller
 * @param H Feedback (default: unity)
 * @return Closed-loop from r to y
 */
inline TransferFunction two_dof_loop(
    const TransferFunction& G,
    const TransferFunction& Cr,
    const TransferFunction& Cy,
    const TransferFunction& H = TransferFunction({1}, {1})
) {
    TransferFunction inner_loop = feedback(Cy * G, H);
    return Cr * inner_loop;
}

// ============================================================
//                    CASCADE CONTROL
// ============================================================

/**
 * @brief Cascade (nested) control structure
 * 
 *     r -->[C1]-->(+)-->[C2]-->[G2]-->[G1]--->y
 *                  ^              |      |
 *                  |------[H2]<---|      |
 *                  |                     |
 *                  |-------[H1]<---------|
 * 
 * @return Closed-loop transfer function
 */
inline TransferFunction cascade_control(
    const TransferFunction& G1,  // Outer plant
    const TransferFunction& G2,  // Inner plant
    const TransferFunction& C1,  // Outer controller
    const TransferFunction& C2,  // Inner controller
    const TransferFunction& H1 = TransferFunction({1}, {1}),
    const TransferFunction& H2 = TransferFunction({1}, {1})
) {
    // Inner loop first
    TransferFunction inner = feedback(C2 * G2, H2);
    
    // Outer loop
    return feedback(C1 * inner * G1, H1);
}

// ============================================================
//                    FEEDFORWARD CONTROL
// ============================================================

/**
 * @brief Feedback with feedforward structure
 * 
 *     d ----->[Gd]---.
 *                    v
 *     r -->[Gff]->(+)---[C]-->(+)---[G]--->y
 *           |      ^                  |
 *           |      |-------[H]<-------|
 *           |                         ^
 *           `--------->[Cff]----------'
 * 
 * Simplified: r -> y with feedforward compensation
 */
inline TransferFunction feedforward_loop(
    const TransferFunction& G,
    const TransferFunction& C,
    const TransferFunction& Cff,  // Feedforward controller
    const TransferFunction& H = TransferFunction({1}, {1})
) {
    TransferFunction fb_loop = feedback(C * G, H);
    TransferFunction ff_path = Cff * G;
    
    // Total: ff_path + fb_loop (assuming they add at output)
    return ff_path + fb_loop;
}

// ============================================================
//                    DISTURBANCE REJECTION
// ============================================================

/**
 * @brief Calculate disturbance-to-output transfer function
 * 
 *              d
 *              |
 *              v
 *     r -->[C]-->(+)---[G]--->y
 *           ^              |
 *           |------[H]<----|
 * 
 * G_yd = G / (1 + GCH) = G * S  where S is sensitivity
 */
inline TransferFunction disturbance_to_output(
    const TransferFunction& G,
    const TransferFunction& C,
    const TransferFunction& H = TransferFunction({1}, {1})
) {
    TransferFunction one({1}, {1});
    TransferFunction S = feedback(one, C * G * H);  // Sensitivity
    return G * S;
}

/**
 * @brief Calculate noise-to-output transfer function
 * 
 *     r -->[C]---[G]-->(+)--->y
 *           ^          ^
 *           |          n (noise)
 *           |--[H]<----|
 * 
 * G_yn = -GCH / (1 + GCH) = -T (for noise after output)
 */
inline TransferFunction noise_to_output(
    const TransferFunction& G,
    const TransferFunction& C,
    const TransferFunction& H = TransferFunction({1}, {1})
) {
    TransferFunction T = feedback(G * C * H, TransferFunction({1}, {1}));
    return T * (-1);
}

// ============================================================
//                    STABILITY ANALYSIS OF LOOPS
// ============================================================

/**
 * @brief Check closed-loop stability
 */
inline bool is_closed_loop_stable(
    const TransferFunction& G,
    const TransferFunction& C,
    const TransferFunction& H = TransferFunction({1}, {1})
) {
    TransferFunction T = closed_loop(G, C, H);
    return T.isStable();
}

/**
 * @brief Get loop transfer function
 */
inline TransferFunction loop_transfer(
    const TransferFunction& G,
    const TransferFunction& C,
    const TransferFunction& H = TransferFunction({1}, {1})
) {
    return G * C * H;
}

// ============================================================
//                    SPECIAL CONFIGURATIONS
// ============================================================

/**
 * @brief Smith predictor for time-delay compensation
 * 
 * For plant G(s) = G0(s) * e^(-Ls)
 * Smith predictor uses model G0m and delay estimate Lm
 * 
 * @param G0 Plant without delay (model)
 * @param C Controller designed for G0
 * @param L Delay estimate (used conceptually - actual delay not implemented)
 * @return Equivalent closed-loop without delay effect
 */
inline TransferFunction smith_predictor_ideal(
    const TransferFunction& G0,
    const TransferFunction& C
) {
    // Ideal case: closed-loop behaves as if no delay
    // T = CG0 / (1 + CG0)
    return feedback(C * G0, TransferFunction({1}, {1}));
}

/**
 * @brief Internal Model Control (IMC) structure
 * 
 * IMC controller: Cimc such that closed-loop is stable
 * For perfect model: T = G * Cimc
 * 
 * Convert IMC to classical: C = Cimc / (1 - G*Cimc)
 */
inline TransferFunction imc_to_classical(
    const TransferFunction& G_model,
    const TransferFunction& Cimc
) {
    TransferFunction one({1}, {1});
    TransferFunction denom = one - G_model * Cimc;
    
    // C = Cimc / (1 - G*Cimc)
    return TransferFunction(Cimc.num * denom.den, Cimc.den * denom.num);
}

/**
 * @brief Design IMC controller for first-order plant
 * G(s) = K / (tau*s + 1)
 * Cimc(s) = (tau*s + 1) / (K * (lambda*s + 1))
 * 
 * @param K Plant gain
 * @param tau Plant time constant
 * @param lambda Desired closed-loop time constant
 */
inline TransferFunction imc_first_order(double K, double tau, double lambda) {
    // Cimc = (tau*s + 1) / (K * (lambda*s + 1))
    return TransferFunction({tau, 1}, {K * lambda, K});
}

// ============================================================
//                    LOOP SHAPING HELPERS
// ============================================================

/**
 * @brief Calculate required controller gain for desired crossover
 * 
 * Find K such that |K * G(jwc)| = 1
 */
inline double gain_for_crossover(
    const TransferFunction& G,
    double wc
) {
    double Gwc = G.mag(wc);
    if (Gwc < 1e-15) return std::numeric_limits<double>::infinity();
    return 1.0 / Gwc;
}

/**
 * @brief Calculate required phase lead for desired phase margin
 */
inline double required_phase_lead(
    const TransferFunction& G,
    double wc,
    double desired_PM
) {
    double current_phase = G.phase_deg(wc);
    double current_PM = 180 + current_phase;
    return desired_PM - current_PM;
}

} // namespace control
} // namespace cppplot

#endif // CPPPLOT_CONTROL_BLOCK_DIAGRAM_HPP
