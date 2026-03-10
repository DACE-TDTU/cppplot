# CppPlot Development Roadmap

## 🎯 Vision

**CppPlot** - Thư viện C++ plotting chuyên nghiệp, header-only, không phụ thuộc, tối ưu cho hệ thống embedded, real-time, và scientific computing.

---

## 📊 Architecture Overview

```
cppplot/
├── include/cppplot/
│   ├── cppplot.hpp           # Main header (includes all)
│   ├── pyplot.hpp            # Matplotlib-style API
│   ├── figure.hpp            # Figure management
│   ├── axes.hpp              # Axes and plotting
│   │
│   ├── core/                 # Core utilities
│   │   ├── types.hpp
│   │   ├── color.hpp
│   │   ├── style.hpp
│   │   ├── utils.hpp
│   │   └── latex.hpp
│   │
│   ├── backends/             # Output backends
│   │   ├── svg_backend.hpp   # ✅ Done
│   │   ├── pdf_backend.hpp   # 🔜 v2.5
│   │   └── png_backend.hpp   # 🔜 v3.0
│   │
│   ├── control/              # 🔜 v1.5 - Control Systems
│   │   ├── control.hpp       # Main control header
│   │   ├── transfer_function.hpp
│   │   ├── state_space.hpp
│   │   ├── bode.hpp
│   │   ├── nyquist.hpp
│   │   ├── root_locus.hpp
│   │   └── time_response.hpp
│   │
│   ├── signal/               # 🔜 v2.0 - Signal Processing
│   │   ├── signal.hpp
│   │   ├── fft.hpp
│   │   ├── spectrogram.hpp
│   │   ├── filter.hpp
│   │   └── window.hpp
│   │
│   ├── stats/                # 🔜 v2.0 - Statistics
│   │   ├── stats.hpp
│   │   ├── histogram.hpp
│   │   ├── boxplot.hpp
│   │   ├── violin.hpp
│   │   └── heatmap.hpp
│   │
│   └── plot3d/               # 🔜 v3.0 - 3D Plotting
│       ├── plot3d.hpp
│       ├── surface.hpp
│       ├── contour.hpp
│       └── wireframe.hpp
│
├── examples/
│   ├── basic_demo.cpp
│   ├── subplot_demo.cpp
│   ├── control_system_demo.cpp    # ✅ Done
│   ├── signal_processing_demo.cpp
│   └── statistics_demo.cpp
│
└── tests/
```

---

## 🗓️ Release Schedule

### ✅ v1.0 - Core (RELEASED)
- [x] Line plots, scatter, bar, histogram
- [x] SVG backend
- [x] Format strings (matplotlib-style)
- [x] Subplots and layouts
- [x] GridSpec
- [x] Error bars, fill_between
- [x] Log scale
- [x] Annotations (text, axhline, axvline)
- [x] Basic LaTeX rendering

### ✅ v1.5 - Control Systems (COMPLETE)
**Status: All planned features implemented**

#### Phase 1: Transfer Function Library ✅
- [x] `TransferFunction` class (polynomials num/den)
- [x] `StateSpace` class (A, B, C, D matrices)
- [x] Conversion: TF ↔ SS ↔ ZPK
- [x] Series, parallel, feedback connections
- [x] Stability analysis (poles, zeros)

#### Phase 2: Frequency Domain ✅
- [x] `bode()` - Automatic Bode plot function
- [x] `nyquist()` - Automatic Nyquist plot
- [x] `nichols()` - Nichols chart
- [x] Gain margin, phase margin calculation
- [x] Bandwidth calculation

#### Phase 3: Time Domain ✅
- [x] `step()` - Step response
- [x] `impulse()` - Impulse response
- [x] `initial()` - Initial condition response
- [x] `lsim()` - Arbitrary input simulation
- [x] Rise time, settling time, overshoot (`stepinfo()`)

#### Phase 4: Root Locus & Design ✅
- [x] `rlocus()` - Automatic root locus
- [x] `pzmap()` - Pole-zero map
- [ ] `sisotool()`-style interactive (basic)
- [x] Lead/Lag compensator design helpers

#### Phase 5: Advanced Control (BONUS - COMPLETE) ✅
- [x] Discrete-time systems (`DiscreteTransferFunction`, `c2d_tustin`, `c2d_zoh`)
- [x] LQR/DLQR controller design
- [x] Kalman filter and LQG
- [x] MPC (Model Predictive Control)
- [x] H∞ synthesis (robust control)
- [x] Sliding mode control (nonlinear)
- [x] μ-analysis (uncertainty modeling)

#### Phase 6: Matrix Library (COMPLETE) ✅
- [x] Standalone Matrix class (no Eigen dependency)
- [x] LU, QR, Hessenberg, Schur decompositions
- [x] Eigenvalue computation (any size)
- [x] Matrix exponential (Padé with scaling)
- [x] Condition number, spectral radius
- [x] Cholesky decomposition, positive definiteness check
- [x] Lyapunov/Sylvester equation solvers

### 📅 v2.0 - Signal & Statistics (Month 2-3)
- [ ] FFT (using kiss_fft or custom)
- [ ] Spectrogram, PSD
- [ ] Filter design (Butterworth, Chebyshev)
- [ ] Window functions
- [ ] Advanced histograms
- [ ] Boxplot, violin plot
- [ ] Heatmap, correlation matrix
- [ ] Scatter matrix

### 📅 v2.5 - Enhanced Output (Month 4-5)
- [ ] PDF backend (using libharu or custom)
- [ ] Better font handling
- [ ] Full LaTeX support (equations)
- [ ] Color maps (viridis, plasma, etc.)
- [ ] Custom themes

### 📅 v3.0 - 3D & Advanced (Month 6+)
- [ ] 3D surface plots
- [ ] Contour plots
- [ ] Wireframe
- [ ] 3D scatter
- [ ] PNG backend (stb_image_write)

---

## 📋 v1.5 Detailed Checklist

### Transfer Function Module

```cpp
// Target API
#include <cppplot/control/control.hpp>
using namespace cppplot::control;

// Create transfer function: G(s) = (s+1) / (s^2 + 2s + 1)
TransferFunction G({1, 1}, {1, 2, 1});

// Or from zeros, poles, gain
auto G2 = zpk({-1}, {-1, -1}, 1.0);

// State-space
StateSpace sys(A, B, C, D);

// Conversions
auto ss = tf2ss(G);
auto tf = ss2tf(sys);

// Connections
auto G_series = G1 * G2;           // Series
auto G_parallel = G1 + G2;         // Parallel
auto G_feedback = feedback(G, H);  // Closed-loop
```

### Plotting API

```cpp
// One-liner plots (Python Control style)
bode(G);                    // Auto Bode plot
bode({G1, G2, G3});         // Multiple systems
bode(G, omega);             // Custom frequency range

nyquist(G);                 // Auto Nyquist
nyquist(G, opts({{"arrows", "true"}}));

step(G);                    // Step response
step(G, t);                 // Custom time vector
step({G1, G2});             // Compare systems

rlocus(G);                  // Root locus
rlocus(G, kvect);           // Custom K range

pzmap(G);                   // Pole-zero map
pzmap({G1, G2});            // Compare
```

### Analysis Functions

```cpp
// Margins
auto [gm, pm, wgc, wpc] = margin(G);

// Time-domain specs  
auto info = stepinfo(G);
// info.rise_time, info.settling_time, info.overshoot, info.peak

// Stability
bool stable = isstable(G);
auto p = poles(G);
auto z = zeros(G);

// Bandwidth
double wb = bandwidth(G);
```

---

## 🔧 Implementation Priority

### HIGH Priority (Core functionality) ✅ ALL COMPLETE
1. ✅ `TransferFunction` class
2. ✅ `bode()` function
3. ✅ `nyquist()` function  
4. ✅ `step()` function
5. ✅ `pzmap()` function
6. ✅ `rlocus()` function
7. ✅ `margin()` function
8. ✅ `stepinfo()` function

### MEDIUM Priority (Convenience) ✅ ALL COMPLETE
- ✅ `StateSpace` class
- ✅ `impulse()` function
- ✅ `nichols()` chart
- ✅ `bandwidth()` function

### LOW Priority (Advanced) ✅ ALL COMPLETE
- ✅ `lsim()` arbitrary input
- ✅ Discrete-time systems
- [ ] `sisotool()` interactive (not planned)

---

## 📁 File Structure for v1.5

```
include/cppplot/control/
├── control.hpp              # Main header - includes all
├── transfer_function.hpp    # TF class
├── state_space.hpp          # SS class (optional for v1.5)
├── analysis.hpp             # margin, stepinfo, poles, zeros
├── plots/
│   ├── bode.hpp            # bode() function
│   ├── nyquist.hpp         # nyquist() function
│   ├── root_locus.hpp      # rlocus() function
│   ├── pzmap.hpp           # pzmap() function
│   └── time_response.hpp   # step(), impulse(), lsim()
└── utils/
    ├── polynomial.hpp       # Polynomial arithmetic
    └── frequency.hpp        # logspace, unwrap, etc.
```

---

## 🎯 Success Criteria for v1.5

1. **API Simplicity**: One-liner for common plots
   ```cpp
   bode(TransferFunction({1}, {1, 2, 1}));
   ```

2. **Compatibility**: Follow Python Control naming conventions

3. **Performance**: Fast enough for real-time visualization

4. **Documentation**: Every function documented with examples

5. **Testing**: Unit tests for all transfer function operations

---

## 📚 References

- [Python Control Library](https://python-control.readthedocs.io/)
- [MATLAB Control System Toolbox](https://www.mathworks.com/help/control/)
- [GNU Octave Control Package](https://gnu-octave.github.io/packages/control/)

---

*Last updated: January 2026*
