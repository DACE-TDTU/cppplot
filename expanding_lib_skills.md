# Level 1
You are extending two C++ header-only libraries:

## CppPlot — Visualization
- Entry point: #include <cppplot/cppplot.hpp>
- Namespace: cppplot
- Core API (CONFIRMED, do not invent alternatives):
  Figure fig(w, h);           // NOT auto fig = figure(w,h)
  auto& ax = fig.gca();
  auto& ax = fig.subplot(r,c,i);  // 1-indexed
  ax.plot(x, y, "b-", {{"label", std::string("name")}});
  ax.scatter(x, y, {{"c", std::string("red")}});
  ax.bar(x, heights);
  ax.hist(data, bins);
  ax.set_title/xlabel/ylabel(str);
  ax.set_xlim/ylim(min, max);
  ax.grid(bool); ax.legend(bool);
  fig.toSVG() → std::string;
  fig.savefig("file.svg");
  fig.getAxes().size();
- Utility: linspace, arange, logspace (all confirmed)
- Types: Color, PlotStyle, LineStyle, Colormap,
         Point, Rect, Limits, DataSeries<T>,
         CoordinateTransform, SVGBackend, TextStyle
- Standard: C++17, header-only, zero dependencies
- CRITICAL: Figure has deleted copy constructor (unique_ptr backend)
- CRITICAL: scatter/hist use opts map, NOT format strings

## CppNB — Notebook
- Jupyter-style for C++
- Components: Notebook, Cell (CodeCell/MarkdownCell),
  CppKernel (compile+run), CellOutput, HTMLRenderer
- Cell execution: compile → run → capture SVG/stdout → 
  embed in output
- CppPlot SVG auto-detected when cell calls fig.savefig()

## Repository
- CppPlot: github.com/DACE-TDTU/cppplot
- CppNB:   github.com/DACE-TDTU/cppnb
- Tests confirmed passing: test_core (Color/Style/Utils),
  test_svg (SVGBackend/Figure/Axes)
- Known issue: examples/ use wrong API (pyplot-style),
  CI disables examples build (CPPPLOT_BUILD_EXAMPLES=OFF)

# Level 2: Task Contract — QP Solver cho Embedded MPC

## Task: Build cppqp — QP Solver for Embedded MPC (AGV/AMR)

### Problem statement
Implement a header-only C++17 QP solver library at:
  include/cppqp/cppqp.hpp

Target: Model Predictive Control on embedded systems
  (ESP32-S3, STM32H7, Jetson Nano — 256KB–8MB RAM)

### Mathematical formulation to solve
  minimize    (1/2) x'Px + q'x
  subject to  lb ≤ Ax ≤ ub   (inequality)
              Cx  = d         (equality, optional)

Where for AGV/AMR MPC:
  x = [U0, U1, ..., U_{N-1}]  (control sequence)
  P = block-diagonal from stage costs
  q = linear term from reference tracking
  A = constraint matrix (velocity, acceleration limits)

### Algorithm priority (implement in this order)
1. ADMM (Alternating Direction Method of Multipliers)
   — best for embedded: fixed iteration count,
     warm-starting, parallelizable
2. Active Set (fallback for small problems < 20 vars)
3. Interior Point (optional, for larger systems)

### Integration requirements
1. Standalone: cppqp works without cppplot
2. With cppplot: convergence curves, constraint 
   visualization via CppPlot API confirmed above
3. With cppnb: solver can be called from notebook cells

### Output API contract (design this interface):
  QPSolver solver;
  solver.setObjective(P, q);        // dense or sparse
  solver.setInequality(A, lb, ub);
  solver.setEquality(C, d);         // optional
  solver.setMaxIter(100);
  solver.setWarmStart(x_prev);
  
  QPResult result = solver.solve();
  result.x          // solution vector
  result.cost       // objective value
  result.iters      // iterations used
  result.converged  // bool
  result.solve_time // microseconds

  # Level 3: Constraints — Embedded-specific
  ## Embedded Constraints (MUST follow)

### Memory
- No heap allocation after init (use fixed-size arrays)
- Max stack: configurable via template param
  template<int NX, int NC> class QPSolver { ... }
  // NX = num variables, NC = num constraints
- All matrices: Eigen-free, custom fixed-size matrix class
  OR accept raw double arrays with size params

### Numerics  
- Use double for solver internals
- Provide float specialization for MCU without FPU
- Handle ill-conditioned P via Tikhonov regularization
  (add epsilon * I automatically)
- ADMM rho parameter: auto-tune or document manual tuning

### Real-time
- solve() must have bounded execution time
- Iteration limit = hard stop (not just convergence check)
- Document worst-case flops for given problem size

### Testing requirements
Generate tests in tests/test_qp_solver.cpp:
1. Unconstrained QP: closed-form solution verification
2. Box constraints: AGV velocity limits
3. MPC warm-starting: verify cost decreases iteration-to-iter
4. Numerical: ill-conditioned P still returns valid result
5. Timing: solve() completes within 1ms for N=10 horizon

### Visualization with CppPlot (use confirmed API)
Generate examples/mpc_agv_demo.cpp:
- Show convergence: ax.plot(iters, residuals, "b-")
- Show trajectory: ax.plot(x_traj, y_traj, "r-")  
- Show constraint satisfaction: ax.scatter for violations
- Use Figure fig(w,h) pattern, NOT figure() function
