# Vehicle-Adaptive Minimum-Time Racing Line Tool

**MathWorks Excellence in Innovation, Project 208: Path Planning for Autonomous Race Cars**

> One track, any car. Enter your vehicle's characteristics and your track, and get the minimum-time racing line and speed profile **for that specific car**.

<!-- Replace with a real image/GIF once available -->
![Hero: three cars, three racing lines, one track](docs/media/hero.gif)

- Demo video: `[TODO: YouTube link]`
- Team: `[TODO: team name]`
- Institution: `[TODO]`
- MATLAB version: `[TODO: e.g. R2025b]`

---

## 1. Motivation

An optimal racing line is not a property of the track alone. It is a property of the **track and the car together**.

The effect of a vehicle on the optimal line depends on the **balance between its longitudinal performance and its cornering capability**, as well as on how aerodynamic forces change with speed. The following are hypotheses for our experiments, not universal rules:

- A car with relatively low power and high grip is less able to gain time through acceleration. It is therefore expected to prioritize carrying speed through corners, with a line closer to a **minimum-curvature** solution.
- A car with high power relative to its available grip, and little downforce, can benefit from **squaring off** a corner: accepting a lower minimum speed when useful, completing the turn earlier, and prioritizing acceleration on corner exit (often described as a late-apex or point-and-squirt approach).
- A car with substantial downforce gains cornering grip as speed increases. Its preferred line may therefore change with speed: high-speed corners can reward carrying more speed and using a larger-radius line, while slow corners may still favour a late apex because aerodynamic grip is limited there.

These behaviours are governed by ratios evaluated at the relevant speed, not by labels such as “high-power car” or “high-grip car” alone. In particular, the friction ellipse couples lateral cornering force and longitudinal acceleration: using more tyre force to turn leaves less available for acceleration or braking.

Most published racing-line methods optimize a **geometric proxy** (minimum curvature or shortest path), which does not directly account for the balance of vehicle capabilities. Our tool instead optimizes **lap time directly**, using the vehicle's own parameters, so the resulting line can adapt to the car.

## 2. Project Goal

Build a MATLAB tool, based on the Optimization Toolbox, that:

1. Accepts **user-defined vehicle parameters** and a **user-supplied track**.
2. Computes a **minimum-time racing line and velocity profile** for that vehicle.
3. Visualizes the result in 2D, with real-time animation.
4. Demonstrates that the result is **vehicle-specific** and **robust across tracks**.
5. Validates the reference path in a higher-fidelity model (Vehicle Dynamics Blockset).

## 3. How This Maps to the Project 208 Requirements

| # | Project 208 suggested step | Our implementation | Owner |
|---|---|---|---|
| 1 | Research optimization-based path planning | Literature review in `docs/literature.md` (minimum curvature, minimum time, convex speed optimization) | Keiza |
| 2 | Define a basic bicycle vehicle model | Parametric vehicle model (`+vehicle/`), with bicycle/point-mass formulation and friction ellipse | Kael |
| 3 | Create or import a racetrack | CSV importer (`x, y, w_right, w_left`), plus tracks built or imported via Driving Scenario Designer / RoadRunner | Jeni |
| 4 | Generate a minimum-time velocity profile | Convex fixed-path speed optimization (`b = v²`), cross-checked with a forward-backward pass | Keiza |
| 5 | Formulate an optimization problem (min-time or min-curvature) | Min-time formulation, initialized by min-curvature QP, solved with `quadprog`, `fmincon`, and `coneprog`/`linprog` | Keiza |
| 6 | 2D visualization of the trajectory in real time | App Designer UI with speed-colored line and animated vehicle | Jeni |
| 7 | Test robustness on different racetracks | Benchmark suite of `[TODO: N]` tracks with different characters | Jeni, Keiza |
| A1 | Advanced: obstacles, then Vehicle Dynamics Blockset | Reference path replayed in Vehicle Dynamics Blockset with a path-tracking controller. Obstacles: `[TODO: in scope / future work]` | Kael |
| A2 | Advanced: Simscape / battery | `[TODO: in scope / future work]` | Kael |

## 4. Gap Analysis: Where We Differ from Prior Project 208 Submissions

We reviewed the public READMEs of three earlier submissions. The comparison below is based on what those READMEs describe, not on a line-by-line review of their code.

| | Prior work A (MSc, Southampton) | Prior work B (patternsearch) | Prior work C (QP + Simulink) | **This work** |
|---|---|---|---|---|
| Path objective | Manual compromise between shortest path and min curvature, offline and online | Min curvature via per-point `patternsearch` | Min curvature and shortest path via `quadprog` | **Lap time, with the vehicle in the loop** |
| Speed profile | Friction-circle target speed | Point-mass, forward-backward | Critical-velocity method, forward-backward, aero drag | **Convex optimal speed profile with friction ellipse, drag, power, and downforce** |
| Vehicle model | Four-wheel, Dugoff tyre | Point-mass | Point-mass, then Simulink 3DOF | **User-parametrized, plus Vehicle Dynamics Blockset validation** |
| Adapts to vehicle characteristics | Tuned per car by hand | No | No | **Yes, automatically** |
| Reusable tool with UI | No | No | No | **Yes (App Designer)** |
| Quantified lap-time comparison across baselines | Tracking accuracy | Visual | Lap time vs. record | **Lap time vs. centerline, shortest path, min curvature, and cross-vehicle matrix** |

### Specific gaps we address

1. **Proxy objectives.** Prior solutions use minimum curvature or a hand-tuned weighting of curvature and length as a stand-in for lap time. These do not respond to the car's physical limits. We optimize lap time itself.
2. **Known simplifications left open.** One prior submission documents that it neglects the `2x'y'` term in its curvature definition and suggests an iterative QP as an improvement. Our iterative formulation (Section 5) re-linearizes around the previous solution, which is the direction that README points to.
3. **No vehicle-adaptive behaviour.** None of the prior submissions show that the line changes in a physically meaningful way when the car changes. We make this the central experiment (Section 7).
4. **Limited use of the Project 208 advanced toolchain.** The advanced items (Vehicle Dynamics Blockset, Simscape) are only partly addressed in earlier work. We use Vehicle Dynamics Blockset as an independent check on our reference line.
5. **Not usable as a tool.** Earlier work consists of scripts for fixed cases. Ours is parametrized by the user's car and track.

We do not compete on high-fidelity tyre modelling or online/dynamic obstacle avoidance, where earlier work is already strong. Our contribution is the **vehicle-adaptive, lap-time-driven formulation** and the tool built around it.

## 5. Approach

### 5.1 Vehicle model

A parametrized point-mass model with a friction ellipse, extended with aerodynamics:

| Parameter | Symbol |
|---|---|
| Mass | `m` |
| Tyre friction coefficient | `mu` |
| Max drive force / engine power | `F_drive_max`, `P_max` |
| Max braking force | `F_brake_max` |
| Drag area | `CdA` |
| Downforce area | `ClA` |
| Vehicle width | `w` |
| Air density | `rho` |

Available grip grows with speed through downforce: `F_grip(v) = mu * (m*g + 0.5*rho*ClA*v^2)`, where `ClA` is the downforce coefficient-area product under the model's convention.

For vehicle comparisons, we will report more than raw parameter values. Useful comparison quantities include:

- **Power-to-weight:** `P_max / m`, describing the vehicle's potential longitudinal performance.
- **Downforce-to-weight at a reference speed:** `0.5*rho*ClA*v_ref^2 / (m*g)`, describing how significant aerodynamic load is at that speed.
- **Drag-to-weight at a reference speed:** `0.5*rho*CdA*v_ref^2 / (m*g)`, describing the relative aerodynamic resistance.
- **Longitudinal-to-lateral capability:** the available longitudinal acceleration relative to lateral acceleration at a stated speed and operating condition. This captures the friction-ellipse trade-off more directly than a vehicle label.

The reference speed and assumptions used for these comparisons must be stated alongside the results. These ratios help explain the observed line; they do not predetermine it.

### 5.2 Track representation

- Input: `x, y, w_right, w_left` (the format used by the open TUMFTM Race Track Database), or tracks created with Driving Scenario Designer / RoadRunner.
- Smoothing and spline resampling to `N` points, with arc length `s` and normal vectors.
- Decision variable: lateral offset `alpha_i` at each point, bounded by track width minus half the vehicle width. The first and last points are tied so the line closes.

### 5.3 Speed profile on a fixed path

With `b = v^2`, longitudinal acceleration becomes `b'/2` and lateral acceleration becomes `kappa * b`. The friction-ellipse, drag, and power constraints are convex in `b`, so minimum-time speed optimization on a **fixed path is a convex problem** and is solved globally. A forward-backward pass runs alongside as a fast baseline and sanity check.

### 5.4 Path optimization

Two layers, built in this order:

- **Baseline (Plan B): vehicle-tuned blend.** A QP (`quadprog`) minimizes `(1 - lambda) * curvature + lambda * length`. An outer search over `lambda` (`fminbnd`) minimizes the lap time from 5.3. This already makes the line vehicle-specific.
- **Full method (Plan A): alternating optimization.** Fix the speed profile and update the path (SQP via `fmincon`, or iterated QP linearized around the previous path). Then fix the path and recompute the speed profile. Repeat until lap time converges, keeping the best iterate.

The final reported method is `[TODO: Plan A / Plan B, decide by 31 Jan 2027]`.

### 5.5 Validation

- **Sanity check** against published lap times and the TUMFTM reference raceline on shared tracks.
- **Higher-fidelity replay:** the optimized path and speed profile are used as the reference for a Vehicle Dynamics Blockset model with a path-tracking controller, to check that the line is drivable and to quantify the gap to the point-mass assumption.

## 6. Tool and User Interface

An App Designer application with:

- Vehicle parameter panel and presets
- Track upload (CSV) and built-in tracks
- One-click computation of the racing line and speed profile
- Speed-colored line plot, `v(s)` plot, and lap-time comparison table
- Real-time animation of the vehicle around the lap

Deployment as a browser-accessible app via MATLAB Web App Server is `[TODO: included / future work, depends on licence]`.

## 7. Key Experiment: One Track, Three Cars

We run the same track with three vehicles defined by **relative capability**, not by isolated labels. Exact parameter values will be recorded in the experiment configuration.

| Vehicle | Parameter relationship | Expected line behaviour (hypothesis to test) |
|---|---|---|
| **Car A** | Low power relative to available grip; modest aerodynamic effects | Expected to favour carrying corner speed, with a line closer to minimum curvature |
| **Car B** | High power relative to available grip; little downforce | Expected to benefit more from squaring off corners and prioritizing exit acceleration |
| **Car C** | High downforce relative to vehicle weight, especially at higher speeds | Expected to show a speed-dependent line: more momentum-preserving behaviour in fast corners, while slow corners may still favour a late apex |

For a fair comparison, report the key ratios (including power-to-weight and downforce-to-weight at a stated reference speed), as well as the underlying vehicle parameters. “High” and “low” must be defined relative to the other cars in this experiment.

**Important control experiment:** if grip, drive-force, and braking-force limits are all multiplied by the same factor while the rest of the model is held fixed, the predicted optimal line may remain unchanged under the corresponding idealized scaling assumptions. Therefore, a “grippier” car alone is not guaranteed to produce a visibly different line. We will test this behaviour rather than assume that increasing grip necessarily shortens or changes the line.

**Cross-evaluation matrix.** Each car is driven on each car's optimized line. If the method is truly vehicle-adaptive, the diagonal (own line) is expected to be fastest for each vehicle, subject to numerical convergence and the limitations of the model.

| Lap time (s) | Line of Car 1 | Line of Car 2 | Line of Car 3 |
|---|---|---|---|
| **Car 1** | `[TBD]` | `[TBD]` | `[TBD]` |
| **Car 2** | `[TBD]` | `[TBD]` | `[TBD]` |
| **Car 3** | `[TBD]` | `[TBD]` | `[TBD]` |

**Baseline comparison** (per car): centerline, shortest path, minimum curvature, and our method.

| Method | Lap time (s) | Improvement vs. centerline |
|---|---|---|
| Centerline | `[TBD]` | n/a |
| Shortest path | `[TBD]` | `[TBD]` |
| Minimum curvature | `[TBD]` | `[TBD]` |
| **Ours** | `[TBD]` | `[TBD]` |

## 8. Robustness Across Tracks

| Track | Character | Length (m) | Converged | Lap time (s) | Compute time (s) |
|---|---|---|---|---|---|
| `[TODO]` | Oval / fast | | | | |
| `[TODO]` | Hairpin-heavy | | | | |
| `[TODO]` | Technical / mixed | | | | |
| `[TODO]` | Real circuit | | | | |

## 9. Results

> Fill this section with final figures, GIFs, and tables. Every claim of improvement should have a number next to it.

- Racing lines for the three cars (figure), with the vehicle ratios and reference speed stated
- Speed profiles `v(s)` (figure)
- Cross-evaluation matrix (Section 7)
- Vehicle Dynamics Blockset tracking result (figure, tracking error)
- A comparison showing whether the observed line changes agree with or contradict the hypotheses in Sections 1 and 7

## 10. Repository Structure

```
.
├── README.md
├── main.m                      % runs the full pipeline from scratch
├── app/                        % App Designer application
├── +track/                     % loading, smoothing, resampling, normals
├── +vehicle/                   % parameter structs, presets, limits
├── +optim/                     % min-curvature QP, speed optimization, alternating loop
├── +viz/                       % plots and animation
├── tracks/                     % example tracks (CSV)
├── validation/                 % Vehicle Dynamics Blockset model and scripts
├── tests/                      % unit tests and regression checks
└── docs/                       % literature notes, figures, media
```

### Shared data contracts

| From → To | Structure |
|---|---|
| Track → Optimizer | `x_c, y_c, w_left, w_right, normal, s` |
| Vehicle → Optimizer | `m, mu, g, F_drive_max, P_max, F_brake_max, CdA, ClA, width` |
| Optimizer → Visualization / Validation | `x, y, s, kappa, v, t, lapTime` |

## 11. Getting Started

**Requirements:** MATLAB `[TODO: version]`, Optimization Toolbox. Optional: Vehicle Dynamics Blockset (validation), App Designer (UI), Driving Scenario Designer / RoadRunner (track creation).

```matlab
% 1. Clone the repository and open the project folder in MATLAB
% 2. Run the full demo (one track, three cars)
main

% 3. Or launch the interactive app
app.RacingLineApp
```

To use your own data, provide a vehicle struct (Section 5.1) and a track CSV with columns `x, y, w_right, w_left` (meters).

## 12. Limitations and Future Work

- The core optimizer uses a point-mass model: no load transfer, non-linear tyre behaviour, or yaw dynamics. We quantify the resulting gap with the Vehicle Dynamics Blockset replay.
- Results assume a perfectly known track and constant friction. Sensitivity to friction error and tracking error is `[TODO: studied / future work]`.
- Possible extensions: static obstacles as path constraints, battery/energy-aware optimization with Simscape, and cone-defined tracks for Formula Student Driverless.

## 13. Team

| Member | Role | Responsibilities |
|---|---|---|
| **Keiza** | Path Planning & Optimization Lead | Literature review, min-curvature QP, convex speed profile, lap-time optimization, convergence and benchmarks |
| **Jeni** | Track, Simulation & Visualization Lead | Track import and processing, multi-track test suite, App Designer UI, 2D visualization and animation |
| **Kael** | Vehicle Model & Control Lead | Vehicle model and parameter presets, Vehicle Dynamics Blockset model, path-tracking controller, validation |

## 14. References

1. A. Heilmeier, A. Wischnewski, L. Hermansdorfer, J. Betz, M. Lienkamp, and B. Lohmann, "Minimum curvature trajectory planning and control for an autonomous race car," *Vehicle System Dynamics*, vol. 58, no. 10, 2020. (First published online 2019.) DOI: 10.1080/00423114.2019.1631455
   *Used for: min-curvature QP formulation, iterative QP, forward-backward speed profile (our initialization and baseline).*
2. T. Lipp and S. Boyd, "Minimum-time speed optimisation over a fixed path," *International Journal of Control*, vol. 87, no. 6, pp. 1297-1311, 2014. DOI: 10.1080/00207179.2013.875224
   *Used for: convex reformulation of fixed-path speed optimization (b = v²).*
3. N. R. Kapania, J. Subosits, and J. C. Gerdes, "A sequential two-step algorithm for fast generation of vehicle racing trajectories," *ASME Dynamic Systems and Control Conference*, 2015. arXiv: 1902.00606
   *Used for: the idea of alternating between a speed-profile step and a path step. Note: in that paper the path step minimizes curvature under the fixed speed profile, whereas our Plan A targets lap time directly.*
4. F. Braghin, F. Cheli, S. Melzi, and E. Sabbioni, "Race driver model," *Computers & Structures*, vol. 86, no. 13-14, pp. 1503-1516, 2008.
   *Used for: the compromise between shortest path and minimum curvature that motivates our vehicle-tuned blend (Plan B).*
5. TUM Institute of Automotive Technology (TUMFTM), *Race Track Database*. https://github.com/TUMFTM/racetrack-database
6. MathWorks Excellence in Innovation, *Project 208: Path Planning for Autonomous Race Cars*. https://github.com/mathworks/MathWorks-Excellence-in-Innovation/tree/main/projects/Path%20Planning%20for%20Autonomous%20Race%20Cars

## 15. Acknowledgements

Built for the MathWorks Excellence in Innovation program, Project 208. Thanks to the project mentors and the earlier Project 208 contributors whose public work informed our gap analysis.

## 16. Licence

`[TODO: e.g. MIT]`
