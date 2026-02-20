# 🔬 CppPlot Control Systems - Research Roadmap

## Tầm nhìn
Xây dựng một thư viện C++ header-only phục vụ:
1. **Giáo dục**: Giúp sinh viên hiểu sâu thuật toán điều khiển
2. **Nghiên cứu**: Công cụ hỗ trợ nghiên cứu học thuật cá nhân
3. **Thực hành**: Áp dụng được cho embedded systems

---

## � Tổng quan Phiên bản

| Version | Date | Highlights |
|---------|------|------------|
| v1.0 | 2025 | Core modules: TF, SS, Analysis, Plotting |
| v1.1 | Q1 2026 | Kalman Filter (KF, EKF, UKF), LQG |
| v1.2 | Q1 2026 | MPC with constraints |
| v1.3 | Q1 2026 | Phase portraits, Lyapunov tools |
| v1.4 | Q1 2026 | EE/Telecom applications |
| v1.5 | Q1 2026 | H∞, Uncertainty modeling, μ-analysis |
| **v1.6** | **Jan 28, 2026** | **Advanced SMC (8 algorithms) + Bug fixes** |

---

## 📚 Module Hiện có (v1.6 - January 28, 2026)

| Module | Chức năng | Trạng thái |
|--------|-----------|------------|
| `polynomial.hpp` | Đa thức, tìm nghiệm | ✅ Hoàn thành |
| `transfer_function.hpp` | Hàm truyền SISO | ✅ Hoàn thành |
| `state_space.hpp` | Biểu diễn không gian trạng thái | ✅ Hoàn thành |
| `controller_design.hpp` | LQR, Pole placement, PID, CARE/DARE | ✅ Hoàn thành |
| `analysis.hpp` | Margin, stability, stepinfo | ✅ Hoàn thành |
| `discrete.hpp` | Hệ rời rạc, ZOH, Tustin, matched | ✅ Hoàn thành |
| `bode.hpp`, `nyquist.hpp`, `nichols.hpp` | Biểu đồ tần số | ✅ Hoàn thành |
| `root_locus.hpp` | Quỹ đạo nghiệm số | ✅ Hoàn thành |
| `pzmap.hpp` | Biểu đồ cực-không | ✅ Hoàn thành |
| `time_response.hpp` | Đáp ứng thời gian | ✅ Hoàn thành |
| `block_diagram.hpp` | Đại số khối | ✅ Hoàn thành |
| `kalman.hpp` | KF, EKF, UKF, Adaptive KF | ✅ Hoàn thành |
| `lqg.hpp` | Linear Quadratic Gaussian | ✅ Hoàn thành |
| `mpc.hpp` | MPC với ràng buộc | ✅ Hoàn thành |
| `robust/hinf.hpp` | H∞ synthesis | ✅ Hoàn thành |
| `robust/uncertainty.hpp` | Uncertainty modeling | ✅ Hoàn thành |
| `robust/mu_analysis.hpp` | μ-analysis & visualization | ✅ Hoàn thành |
| `nonlinear/sliding_mode.hpp` | **Advanced SMC Suite (8 algorithms)** | ✅ Hoàn thành |

### 🎯 Advanced SMC Algorithms (v1.6 - January 28, 2026)

| Algorithm | Description | Performance | Status |
|-----------|-------------|-------------|--------|
| Conventional SMC | Basic sign(s) switching control | Baseline | ✅ Complete |
| Super-Twisting (STA) | Continuous control, finite-time | Continuous u(t) | ✅ Complete |
| Integral SMC (ISMC) | Eliminates reaching phase | No reaching phase | ✅ Complete |
| Quasi-Continuous HOSM | Smoother control signal | Less chattering | ✅ Complete |
| Prescribed-Time SMC | Guaranteed settling time T_s | T = T_prescribed | ✅ Complete |
| **Fixed-Time SMC** | T_max independent of IC (Polyakov 2012) | T ≤ T_max ∀ x(0) | ✅ **Tested** |
| **Event-Triggered SMC** | Control only when needed | **99.3% update reduction** | ✅ **Tested** |
| **Barrier Function SMC** | Hard state constraints | |x| < bound guaranteed | ✅ **Tested** |
| **DOBSMC** | Disturbance observer + SMC | **75.1% chattering reduction** | ✅ **Tested** |

### 📝 Documentation (v1.6)

| Document | Content | Status |
|----------|---------|--------|
| `ch16b_sliding_mode_control.md` | SMC mathematical foundations, Lyapunov proofs, design methodology | ✅ NEW |
| `appendix_c_library_reference.md` | C.18 SMC API reference | ✅ Updated |
| `appendix_d_cpp_programming.md` | D.11 SMC implementation patterns, D.13 Algorithm selection | ✅ Updated |

### 🐛 Bug Fixes (v1.6 - January 28, 2026)

| Issue | Severity | Fix | File |
|-------|----------|-----|------|
| Static variable in DOBSMC | HIGH | Converted to member variable `dob_filtered_disturbance_` | `sliding_mode.hpp` |
| Duplicate C.16 Compilation section | MEDIUM | Removed duplicate, renumbered sections | `appendix_c_library_reference.md` |
| Missing factory functions in docs | MEDIUM | Added `createConventionalSMC()`, etc. | `appendix_c_library_reference.md` |

---

## 🚀 Roadmap Phát triển

### Phase 1: Optimal Control (Ưu tiên cao)
```
📁 include/cppplot/control/
├── optimal/
│   ├── lqg.hpp          - Linear Quadratic Gaussian
│   ├── kalman.hpp       - Kalman Filter (steady-state & time-varying)
│   ├── mpc.hpp          - Model Predictive Control
│   └── dynamic_prog.hpp - Dynamic Programming
```

**Chi tiết:**
- **LQG**: Kết hợp LQR + Kalman Filter cho hệ có nhiễu
- **Kalman Filter**: Ước lượng trạng thái tối ưu
- **MPC**: Điều khiển dự báo mô hình (quan trọng trong công nghiệp)
- **DP**: Giải bài toán điều khiển tối ưu tổng quát

### Phase 2: Robust Control (Ưu tiên cao)
```
📁 include/cppplot/control/
├── robust/
│   ├── hinf.hpp         - H∞ control
│   ├── h2.hpp           - H2 control
│   ├── uncertainty.hpp  - Uncertainty modeling
│   ├── mu_analysis.hpp  - Structured singular value
│   └── loop_shaping.hpp - Loop shaping design
```

**Chi tiết:**
- **H∞**: Thiết kế controller tối thiểu hóa worst-case gain
- **Uncertainty**: Mô hình hóa sai số tham số, không mô hình được
- **μ-analysis**: Phân tích robust stability/performance

### Phase 3: Nonlinear Control (Ưu tiên trung bình)
```
📁 include/cppplot/control/
├── nonlinear/
│   ├── sliding_mode.hpp - Advanced Sliding Mode Control  ✅ Complete (8 algorithms)
│   │   ├── Conventional SMC with reaching laws           ✅ Complete
│   │   ├── Super-Twisting Algorithm (STA)                ✅ Complete
│   │   ├── Integral SMC (ISMC)                           ✅ Complete
│   │   ├── Fixed-Time SMC (FxTSMC)                       ✅ NEW
│   │   ├── Event-Triggered SMC (ETSMC)                   ✅ NEW
│   │   ├── Barrier Function SMC                          ✅ NEW
│   │   └── Disturbance Observer SMC (DOBSMC)             ✅ NEW
│   ├── lyapunov.hpp     - Lyapunov stability analysis
│   ├── phase_portrait.hpp - Phase plane analysis
│   ├── backstepping.hpp - Backstepping design
│   ├── feedback_lin.hpp - Feedback linearization
│   └── describing_func.hpp - Describing function analysis
```

**Chi tiết:**
- **SMC (Complete)**: 8 thuật toán SMC từ cơ bản đến nâng cao
  - Fixed-Time: Thời gian hội tụ không phụ thuộc điều kiện đầu
  - Event-Triggered: Giảm 90%+ cập nhật điều khiển
  - Barrier Function: Đảm bảo ràng buộc trạng thái
  - DOBSMC: Giảm 75% chattering
- **Lyapunov**: Phân tích ổn định phi tuyến
- **Phase Portrait**: Vẽ quỹ đạo pha cho hệ 2D
- **Backstepping**: Thiết kế recursive cho hệ strict-feedback

### Phase 4: System Identification (Ưu tiên trung bình)
```
📁 include/cppplot/control/
├── identification/
│   ├── arx.hpp          - ARX model estimation
│   ├── armax.hpp        - ARMAX model estimation
│   ├── subspace.hpp     - Subspace identification
│   ├── rls.hpp          - Recursive Least Squares
│   └── pem.hpp          - Prediction Error Method
```

**Chi tiết:**
- **ARX/ARMAX**: Nhận dạng mô hình từ dữ liệu vào-ra
- **Subspace**: N4SID, MOESP algorithms
- **RLS**: Online identification

### Phase 5: Adaptive Control (Ưu tiên thấp)
```
📁 include/cppplot/control/
├── adaptive/
│   ├── mrac.hpp         - Model Reference Adaptive Control
│   ├── str.hpp          - Self-Tuning Regulator
│   └── l1_adaptive.hpp  - L1 Adaptive Control
```

### Phase 6: Advanced Topics (Tương lai)
```
📁 include/cppplot/control/
├── advanced/
│   ├── networked.hpp    - Networked Control Systems
│   ├── event_trigger.hpp - Event-triggered Control
│   ├── data_driven.hpp  - Data-driven Control
│   └── reinforcement.hpp - RL-based Control
```

---

## 📊 Chi tiết Thuật toán Cần Triển khai

### 1. Kalman Filter
```
Prediction:
  x̂(k|k-1) = A·x̂(k-1|k-1) + B·u(k-1)
  P(k|k-1) = A·P(k-1|k-1)·A' + Q

Update:
  K(k) = P(k|k-1)·C'·(C·P(k|k-1)·C' + R)^(-1)
  x̂(k|k) = x̂(k|k-1) + K(k)·(y(k) - C·x̂(k|k-1))
  P(k|k) = (I - K(k)·C)·P(k|k-1)
```

### 2. Model Predictive Control (MPC)
```
Minimize: J = Σ(x'Qx + u'Ru) over horizon N
Subject to:
  x(k+1) = Ax(k) + Bu(k)
  u_min ≤ u ≤ u_max
  x_min ≤ x ≤ x_max
  
Formulate as QP:
  min  (1/2)U'HU + f'U
  s.t. A_ineq·U ≤ b_ineq
```

### 3. H∞ Control
```
Find K minimizing ||T_zw||_∞

Hamiltonian:
H = [ A    -γ^(-2)B1B1' + B2B2' ]
    [ -Q              -A'       ]

Solve coupled Riccati equations:
  A'X + XA - X(B1B1'/γ² - B2B2'R^(-1))X + C1'C1 = 0
  AY + YA' - Y(C1'C1/γ² - C2'C2Q^(-1))Y + B1B1' = 0
  
With spectral radius condition: ρ(XY) < γ²
```

### 4. Sliding Mode Control ✅ IMPLEMENTED (v1.6)
```
=== Basic SMC ===
Sliding surface: s = Cx + ∫(r - y)dt = 0

Control law:
  u = u_eq + u_sw
  u_eq = equivalent control (keeps system on surface)
  u_sw = -K·sign(s) (switching control)
  
Reaching condition: s·ṡ < 0

=== Fixed-Time SMC (Polyakov 2012) ===
Control: u = u_eq - (k1|s|^p·sign(s) + k2|s|^q·sign(s)) / g(x)
where 0 < p < 1, q > 1

Settling time bound: T_max = 1/(k1(1-p)) + 1/(k2(q-1))
→ Independent of initial conditions!

=== Event-Triggered SMC ===
Trigger condition: ||s(t) - s(tk)|| > σ||s(tk)|| + ε
Zeno prevention: t_{k+1} - t_k ≥ τ_min
→ 90-99% reduction in control updates

=== Barrier Function SMC ===
Barrier Lyapunov: V = (1/2)log(kc²/(kc² - x²)) + (1/2)s²
→ Guarantees |x| < kc at all times

=== Disturbance Observer SMC ===
Observer: d̂ = z + L·x,  ż = -L·(f + gu + d̂)
Reduced gain: K_reduced = 0.3K
→ 60-80% chattering reduction
```

### 5. Lyapunov Analysis
```
For ẋ = f(x), find V(x) such that:
  1. V(0) = 0
  2. V(x) > 0 for x ≠ 0
  3. V̇(x) = ∂V/∂x · f(x) < 0

Common choices:
  - Quadratic: V(x) = x'Px
  - Sum of squares (SOS)
```

---

## 🎓 Ứng dụng Giáo dục

### Bài tập mẫu có thể tạo:
1. **Control Theory 101**: Step response, Bode, Nyquist
2. **State-Space Analysis**: Controllability, Observability
3. **Classical Control**: PID tuning, Root locus
4. **Modern Control**: LQR, Pole placement
5. **Optimal Control**: LQG, Kalman filter
6. **Robust Control**: Uncertainty, H∞
7. **Nonlinear Control**: Phase portraits, Lyapunov

### Interactive Learning:
```cpp
// Example: Sinh viên có thể thay đổi tham số và thấy kết quả ngay
auto sys = TransferFunction({1}, {1, 2, 1});
sys.stepResponse();  // Vẽ đáp ứng

// Thay đổi damping ratio và so sánh
for (double zeta : {0.2, 0.5, 0.7, 1.0, 2.0}) {
    auto sys = secondOrder(1.0, zeta, 2.0);
    sys.stepResponse();
}
```

---

## 📈 Metrics & Milestones

| Milestone | Target Date | Deliverables | Status |
|-----------|-------------|--------------|--------|
| v1.1 | Q1 2026 | Kalman Filter, LQG | ✅ Complete |
| v1.2 | Q1 2026 | MPC (constrained) | ✅ Complete |
| v1.3 | Q1 2026 | Phase portraits, Lyapunov tools | ✅ Complete |
| v1.4 | Q1 2026 | EE/Telecom Examples | ✅ Complete |
| v1.5 | Q1 2026 | H∞, Robust analysis | ✅ Complete |
| **v1.6** | **Jan 28, 2026** | **Advanced SMC (8 algorithms), Audit & Fixes** | ✅ **Complete** |
| v1.7 | Q2 2026 | Lyapunov analysis tools, Phase portraits | 🔄 Next |
| v2.0 | Q2 2026 | System identification (ARX/ARMAX) | ⬜ Planned |
| v2.1 | Q3 2026 | Backstepping, Feedback linearization | ⬜ Planned |
| v3.0 | Q4 2026 | Adaptive Control (MRAC, L1) | ⬜ Planned |

---

## 🎯 Kế hoạch Tiếp theo (v1.7)

### Priority 1: Hoàn thiện Nonlinear Control Module
```
📁 include/cppplot/control/nonlinear/
├── sliding_mode.hpp     ✅ Complete (8 algorithms)
├── lyapunov.hpp         ⬜ Phase portraits, LaSalle, SOS
├── backstepping.hpp     ⬜ Recursive design
├── feedback_lin.hpp     ⬜ Input-output, input-state
└── describing_func.hpp  ⬜ Limit cycle prediction
```

**Ưu tiên:**
1. `lyapunov.hpp` - Vẽ phase portrait, phân tích LaSalle invariance
2. `phase_portrait.hpp` - Vector field visualization
3. `backstepping.hpp` - Strict-feedback systems

### Priority 2: System Identification
```
📁 include/cppplot/control/identification/
├── arx.hpp              ⬜ ARX model estimation
├── armax.hpp            ⬜ ARMAX model estimation
├── subspace.hpp         ⬜ N4SID, MOESP
└── rls.hpp              ⬜ Recursive Least Squares
```

### Priority 3: Adaptive Control
```
📁 include/cppplot/control/adaptive/
├── mrac.hpp             ⬜ Model Reference Adaptive Control
├── str.hpp              ⬜ Self-Tuning Regulator
└── l1_adaptive.hpp      ⬜ L1 Adaptive Control
```

---

## 📖 References

### Textbooks:
1. Åström & Murray - "Feedback Systems" (Free online)
2. Skogestad & Postlethwaite - "Multivariable Feedback Control"
3. Khalil - "Nonlinear Systems"
4. Boyd et al. - "Linear Matrix Inequalities in System and Control Theory"
5. Rawlings & Mayne - "Model Predictive Control"

### Papers:
1. Kalman (1960) - "A New Approach to Linear Filtering"
2. Doyle (1978) - "Guaranteed Margins for LQG Regulators"
3. Zames (1981) - "Feedback and Optimal Sensitivity"
4. Utkin (1977) - "Variable Structure Systems with Sliding Modes"

---

## 🔧 Implementation Status

### ✅ Completed (v1.0 - v1.5):
1. ✅ CARE/DARE solvers
2. ✅ Kalman Filter (KF, EKF, UKF)
3. ✅ LQG design with separation principle
4. ✅ MPC with constraints (QP formulation)
5. ✅ Phase portrait plotting
6. ✅ Lyapunov equation solver
7. ✅ 16-chapter textbook with examples
8. ✅ EE/Telecom applications (PLL, converters, motors)
9. ✅ Appendix A: EV Traction Control
10. ✅ Appendix B: Robotics Systems
11. ✅ H∞ synthesis (state/output-feedback, γ-iteration, mixed-sensitivity)
12. ✅ Structured uncertainty modeling (parametric, multiplicative, additive, Kharitonov)
13. ✅ μ-analysis (structured singular value, D-K iteration basics, visualization)

### ✅ Completed (v1.6 - January 28, 2026):
**Advanced Sliding Mode Control:**
- ✅ Conventional SMC with reaching laws
- ✅ Super-Twisting Algorithm (STA)
- ✅ Integral SMC (ISMC)
- ✅ Quasi-Continuous HOSM
- ✅ Prescribed-Time SMC
- ✅ Fixed-Time SMC (FxTSMC) - Polyakov 2012
- ✅ Event-Triggered SMC (ETSMC) - 99.3% update reduction verified
- ✅ Barrier Function SMC - state constraints guaranteed
- ✅ Disturbance Observer SMC (DOBSMC) - 75.1% chattering reduction verified

**Documentation & Quality:**
- ✅ Chapter 16b: SMC Mathematical Foundations
- ✅ Appendix C.18: SMC Library Reference
- ✅ Appendix D.11: SMC Implementation Patterns
- ✅ Appendix D.13: Algorithm Selection Guide
- ✅ Code audit completed
- ✅ Bug fixes: static variable, duplicate sections, missing docs

### 🔄 Next Up (v1.7):
- Lyapunov analysis tools (`lyapunov.hpp`)
- Enhanced phase portrait visualization
- Backstepping design for strict-feedback systems

### ⬜ Planned (v2.0+):
- ARX/ARMAX identification
- MRAC adaptive control
- L1 adaptive control
- Feedback linearization
- Data-driven control

---

## 📊 Test Results Summary (January 28, 2026)

### Advanced SMC Demo (`advanced_smc_demo.exe`):
```
Fixed-Time SMC:
  - T_max theoretical = 0.800s
  - IC=(1,0.5): 0.152s ✓
  - IC=(3,1): 0.163s ✓
  
Event-Triggered SMC:
  - Conventional updates: 5000
  - ET updates: 37
  - Reduction: 99.3% ✓
  
Barrier Function SMC:
  - State bound: 4.0
  - Max |x| achieved: 3.509
  - Constraint satisfied: ✓

DOBSMC:
  - High-gain chattering: 14887.8
  - DOBSMC chattering: 3713.4
  - Reduction: 75.1% ✓
```

---

*Last updated: January 28, 2026*
