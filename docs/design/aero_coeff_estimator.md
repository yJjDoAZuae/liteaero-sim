# AeroCoeffEstimator — Design Document

This document is the design authority for `AeroCoeffEstimator`: its current
implementation, planned extension to full body-axis stability derivatives, the estimation
methods available (empirical, analytical, external tools), and the architecture governing
how those methods are integrated.

**Scope:** subsonic ($M < 0.3$), rigid-airframe, fixed-wing configurations. The
estimator operates on geometry data alone; no flight-test data or wind-tunnel data are
required, though both may be used to validate or override estimated values.

**Relationship to other documents:**

- Algorithm detail for the current empirical formulas:
  [`docs/algorithms/aerodynamics.md`](../algorithms/aerodynamics.md)
- Definition of the coefficient format and sign conventions that estimator output must
  satisfy: [`docs/architecture/aero_coefficient_model.md`](aero_coefficient_model.md)
- Parallel document for propulsion parameter estimation and propulsion-aero coupling:
  [`docs/architecture/propulsion_coeff_estimator.md`](propulsion_coeff_estimator.md)
- Roadmap item that drives the extension:
  [`docs/roadmap/aircraft.md`](../roadmap/aircraft.md) — item 1 (Aerodynamic and
  Propulsion Coefficient Design Study)

---

## Current Implementation

`AeroCoeffEstimator` is a stateless utility class with one public entry point:

```cpp
static std::pair<AeroPerformance, LiftCurveParams>
    estimate(const AircraftGeometry& geom);
```

It applies DATCOM / Raymer empirical formulas (documented in
[`aerodynamics.md`](../algorithms/aerodynamics.md) Parts 1–8) to derive the **trim aero
model** coefficients:

| Output | Coefficients produced |
| --- | --- |
| `AeroPerformance` | $S_\text{ref}$, $A\!\!R$, $e$, $C_{D_0}$, $C_{Y_\beta}$, $C_{L_q}$, $\bar{c}$, $C_{Y_r}$, $l_{VT}$ |
| `LiftCurveParams` | $C_{L_\alpha}$, $C_{L_\text{max}}$, $C_{L_\text{min}}$, $\Delta\alpha_\text{stall}$, $C_{L_\text{sep}}$ (and negative-side counterparts) |

These are wind-frame scalars; pitching and yawing moments are not modeled — they are
implicit in the load-factor abstraction of `Aircraft` (existing model).

### What the Current Estimator Does Not Produce

`BodyAxisCoeffModel` (roadmap item 1 / 7) requires the full 6DOF body-axis stability
derivative set. The following derivatives are **not yet produced** by the current
implementation:

| Group | Missing derivatives |
| --- | --- |
| Axial force | $C_{X_0}$, $C_{X_\alpha}$, $C_{X_{\delta_e}}$ |
| Normal force | $C_{Z_0}$, $C_{Z_\alpha}$, $C_{Z_q}$, $C_{Z_{\delta_e}}$ |
| Pitch moment | $C_{m_0}$, $C_{m_\alpha}$, $C_{m_q}$, $C_{m_{\delta_e}}$ |
| Lateral force | $C_{Y_p}$ |
| Roll moment | $C_{l_\beta}$, $C_{l_p}$, $C_{l_r}$, $C_{l_{\delta_a}}$, $C_{l_{\delta_r}}$ |
| Yaw moment | $C_{n_\beta}$, $C_{n_p}$, $C_{n_r}$, $C_{n_{\delta_a}}$, $C_{n_{\delta_r}}$ |
| Control effectiveness | All $\delta_e$, $\delta_a$, $\delta_r$ derivatives |

Producing these derivatives requires either extending the empirical pipeline, adopting a
more capable analytical method, or integrating with an external tool. The options are
described below.

---

## Estimation Methods

### Method 1 — Empirical (DATCOM / Raymer) — Current Approach

**Description.** Closed-form analytical formulas derived from curve-fits to experimental
data spanning thousands of configurations. Implemented in `AeroCoeffEstimator.cpp`.

**Inputs.** `AircraftGeometry` (wing, tail, fuselage geometry; see
[`aerodynamics.md`](../algorithms/aerodynamics.md) §Input Parameters).

**Accuracy.** ±15–25% on most derivatives for conventional configurations within the
validated range ($0 \le A\!\!R \le 12$, $\Lambda_{QC} \le 35°$, $M < 0.3$). Accuracy
degrades significantly for unconventional planforms (flying wings, highly swept delta
wings, biplanes).

**Extension to body-axis derivatives.** Most missing derivatives can be obtained from
DATCOM via additional formulas that use the same geometry inputs already in
`AircraftGeometry`. Key extensions required:

| Derivative | DATCOM section | Additional inputs needed |
| --- | --- | --- |
| $C_{Z_\alpha}$, $C_{Z_q}$ | §4.1, §5.2 | From existing $C_{L_\alpha}$, $l_{HT}$ |
| $C_{m_\alpha}$, $C_{m_0}$ | §4.3 | $x_\text{NP}$, $x_\text{CG}$ (already in `AircraftGeometry`) |
| $C_{m_q}$ | §5.3 | $l_{HT}$, $S_{HT}$, $\bar{c}$ |
| $C_{l_\beta}$ | §4.2.1 | Wing dihedral $\Gamma$, sweep $\Lambda_{QC}$ |
| $C_{l_p}$, $C_{l_r}$ | §4.2.4 | Wing geometry, $C_{L_\alpha}$ |
| $C_{n_\beta}$ | §4.4 | $C_{Y_\beta}$, $l_{VT}$, $b$ |
| $C_{n_p}$, $C_{n_r}$ | §5.4 | Vertical tail, wing geometry |

**Control effectiveness** ($C_{Z_{\delta_e}}$, $C_{m_{\delta_e}}$, $C_{l_{\delta_a}}$,
$C_{n_{\delta_r}}$, etc.) requires control surface geometry not currently in
`AircraftGeometry`. A `ControlSurfaceGeometry` struct is needed (see
[Architecture](#architecture) section).

**Limitations.** Empirical formulas are unsuitable for:

- Flying wings with no conventional tail (tail-volume correlations undefined)
- Canard configurations (downwash direction reversed)
- Tandem-wing or blended-wing-body configurations
- Configurations outside the DATCOM validated range

For these cases a physics-based method (Methods 2–4 below) is required.

---

### Method 2 — Classical Lifting-Line Theory (LLT)

**Description.** Prandtl's lifting-line theory models the wing as a bound vortex filament
with a trailing horseshoe wake. The Biot-Savart law gives the induced velocity
distribution, and Glauert's Fourier series solution provides the span loading.

**Scope.** Straight or lightly swept unswept wings ($\Lambda_{QC} < 15°$). For moderate
sweep, the Weissinger method (an extended LLT using a single-horseshoe representation per
strip) improves accuracy.

**Outputs.** $C_{L_\alpha}$, $C_{L_\text{max}}$ distribution, $e$ (Oswald efficiency
from actual span loading), $C_{l_p}$ (rolling resistance from Prandtl LLT strip integral),
$C_{l_r}$, $C_{l_\beta}$ (from dihedral — applied as a geometric incidence correction to
each strip).

**Advantage over empirical.** LLT captures non-elliptic span loading explicitly and gives
accurate $e$ and roll damping without the curve-fit assumptions. For a simple, moderate-AR
straight wing it is more reliable than DATCOM for $C_{l_p}$ and $C_{l_r}$.

**Implementation options.**

- Internal C++ implementation using Glauert Fourier expansion (straightforward, ~200 LOC)
- Third-party library (e.g. OpenVSP's built-in LLT solver, callable as a subprocess)

---

### Method 3 — Vortex Lattice Method (VLM)

**Description.** The lifting surfaces are discretized into a lattice of vortex panels
(horseshoe vortices). Boundary conditions (no normal flow through panel control points)
yield a linear system for the vortex strengths. Integration of the pressure distribution
gives forces and moments; finite-difference of the vortex strength distribution with
respect to each state and control perturbation gives the full stability derivative matrix.

**Scope.** All subsonic lifting surface geometries, including swept, tapered, and
low-aspect-ratio wings. Fuselage and thick-body interference are not captured (VLM is a
thin-lifting-surface method).

**Outputs.** All 6DOF body-axis force and moment coefficients, all stability derivatives,
and all control effectiveness derivatives — in a single run. This is the most complete
output of any analytical method.

**Accuracy.** ±10–20% on most derivatives for configurations within the thin-wing
assumption. The main limitation is that VLM cannot predict viscous effects (stall,
separation), boundary layer drag, or thick-wing pressure distribution. $C_{D_0}$ still
requires empirical estimation.

**External implementations.** Two production-grade VLM codes are directly relevant
(Methods 4 and 5 below). An internal C++ VLM implementation is feasible but approximately
1000–2000 LOC and requires careful validation.

---

### Method 4 — AVL (Athena Vortex Lattice)

**Description.** AVL is an extended VLM and strip-theory code written by Mark Drela
(MIT) and Harold Youngren. It is the most widely used open-source VLM in the UAV
community and produces a complete eigenmode, trim, and stability derivative analysis.

**License.** Free to use for non-commercial research; source available; no formal OSS
license. Distribution restrictions apply — AVL cannot be included in liteaero-sim
directly. Integration must use subprocess invocation.

**Inputs.** AVL geometry file (`.avl`): declares lifting surfaces as a sequence of
sections with chord, twist, control surface definitions, and lattice density parameters.
The format is straightforward to generate programmatically from `AircraftGeometry` +
`ControlSurfaceGeometry`.

**Outputs.** After a `STABILITY` run, AVL writes a stability derivative table covering
all 6DOF force/moment coefficients and their derivatives with respect to $\alpha$, $\beta$,
$\hat{p}$, $\hat{q}$, $\hat{r}$, and each control deflection. These map directly to the
`BodyAxisCoeffModel` fields.

**Integration pattern.**

```
AircraftGeometry  ──►  AvlGeometryWriter  ──►  avl_input.avl
avl_input.avl     ──►  AVL process (subprocess)  ──►  stability_output.txt
stability_output.txt  ──►  AvlOutputParser  ──►  BodyAxisCoefficients
```

**Advantages.**

- Mature, validated code with extensive published comparisons to wind-tunnel data
- Handles arbitrary sweep, taper, dihedral, twist, and control surfaces natively
- Output format is well-documented and stable across versions

**Limitations.**

- Thin-surface VLM: no viscous drag, no thick-body effects, no stall
- Subprocess dependency: build system must locate the `avl` binary at runtime; not
  suitable for a mandatory CI path (binary may not be present on all build machines)
- No direct C++ API; all integration is file-based I/O

---

### Method 5 — XFLR5

**Description.** XFLR5 is a GUI analysis tool for low-Reynolds-number airfoils and
aircraft. It implements 3D lifting-line (LLT), horseshoe VLM, and ring-vortex VLM methods,
combined with XFOIL-based 2D section data. It exports stability analysis results as XML.

**License.** GNU GPL v3. Direct linking to liteaero-sim is not permitted under the
project license policy. Integration must be file-based (parse exported XML).

**Inputs.** XFLR5 project files (`.xfl` format); geometry and analysis settings are
specified through the GUI. Batch/scripted operation is supported only in limited form via
command-line export.

**Outputs.** Stability derivative XML export: full $6 \times 6$ derivative matrix, trim
states, and eigenvalue analysis.

**Integration pattern.** XFLR5 is most suitable as a validation tool rather than an
automated pipeline component. A designer sets up a case in the GUI, exports the derivative
XML, and the project provides a parser (`XflrOutputParser`) that converts the export to
`BodyAxisCoefficients`. This allows hand-validation of `AeroCoeffEstimator` output for the
three design study cases (Cases A, B, C).

**Advantages.**

- Combines 2D viscous section data (XFOIL) with 3D VLM — improves $C_{L_\text{max}}$ and
  stall angle prediction
- Interactive geometry visualization useful for configuration validation
- Widely available, well-documented for educational and preliminary-design use

**Limitations.**

- GUI-only for geometry input; scripted automation is limited
- GPL license precludes tight integration
- Results differ between LLT / horseshoe VLM / ring-VLM modes; method selection requires
  user judgment

---

### Method 6 — OpenVSP + VSPAERO *(High Interest)*

**Description.** OpenVSP (Vehicle Sketch Pad) is an open-source parametric aircraft
geometry tool developed by NASA. It provides a complete programmatic geometry API,
a rich set of parametric component types (Wing, Fuselage, BodyOfRevolution, PropGeom,
Pod, Stack, and others), and the VSPAERO solver for aerodynamic analysis. Together they
form a geometry-to-derivatives pipeline that can be driven entirely from Python scripts
with no GUI interaction.

**License.** NASA Open Source Agreement (NOSA 1.3) — permissive, compatible with the
project license policy. Source available; binary distributions for Windows, macOS, and
Linux are provided by NASA.

**Why this is high interest.** OpenVSP is the only external tool in this list that
offers all of the following simultaneously:

- Full programmatic geometry authoring via a Python API (no GUI required)
- Parametric component topology — geometry is defined by design parameters, not raw
  coordinates; changing span, chord, sweep, or dihedral re-meshes automatically
- A VLM solver (VSPAERO) and a higher-fidelity panel method in the same package
- Viscous drag correction (skin-friction buildup integrated into VSPAERO)
- Direct output of stability derivative CSV files compatible with `BodyAxisCoeffModel`
- An active development community and NASA backing

#### OpenVSP Python API

The `openvsp` Python package (distributed alongside OpenVSP) exposes the full geometry
kernel:

```python
import openvsp as vsp

vsp.ClearVSPModel()

# Wing component
wing_id = vsp.AddGeom("WING")
vsp.SetParmVal(wing_id, "TotalSpan", "WingGeom", 2.4)
vsp.SetParmVal(wing_id, "Root_Chord", "XSec_1", 0.30)
vsp.SetParmVal(wing_id, "Tip_Chord", "XSec_1", 0.18)
vsp.SetParmVal(wing_id, "Sweep", "XSec_1", 5.0)   # degrees

# Horizontal tail
ht_id = vsp.AddGeom("WING")
# ... (similar parameters, positioned aft)

# Fuselage
fuse_id = vsp.AddGeom("FUSELAGE")
vsp.SetParmVal(fuse_id, "Length", "Design", 1.6)

vsp.WriteVSPFile("aircraft.vsp3")
```

This API is stable across OpenVSP versions and is used in production by aerospace
companies and universities. The parameter names are documented in the OpenVSP API
reference.

#### VSPAERO Run Modes

VSPAERO supports three solver modes, all accessible from the Python API:

| Mode | Method | Outputs | Best for |
| --- | --- | --- | --- |
| VLM | Vortex lattice | Full stability derivatives, forces, moments | Routine coefficient estimation |
| Panel | 3D panel method | Same + thickness effects | Configurations where thickness matters |
| Viscous correction | VLM + skin friction | $C_{D_0}$ included | Drag estimation for polar generation |

Stability derivative computation is a built-in VSPAERO run mode:

```python
# After geometry is defined:
analysis_name = "VSPAEROComputeGeometry"
vsp.ExecAnalysis(analysis_name)

stab_analysis = "VSPAEROSweep"
vsp.SetAnalysisInputDefaults(stab_analysis)
vsp.SetIntAnalysisInput(stab_analysis, "AnalysisMethod", [0])  # VLM
vsp.SetIntAnalysisInput(stab_analysis, "Stability", [1])       # compute derivatives
results_id = vsp.ExecAnalysis(stab_analysis)
```

Results are written to CSV files that map directly to `BodyAxisCoeffModel` fields.

#### Integration Architecture with LiteAero

The proposed integration has three layers:

```
ParametricAircraftConfig (Python dataclass)
    │
    ▼
VspGeometryBuilder (Python)        ← drives openvsp API to build .vsp3
    │
    ▼
VSPAERO (subprocess or Python API) ← computes stability derivatives
    │
    ▼
VspaeroOutputParser (Python/C++)   ← parses CSV → BodyAxisCoefficients JSON
    │
    ▼
BodyAxisCoeffModel (C++)           ← loads JSON config
```

The `VspGeometryBuilder` is a Python module that owns the mapping from
`ParametricAircraftConfig` fields to OpenVSP parameter names. This is the natural
boundary: Python for geometry authoring and solver orchestration; C++ for the simulation
loop.

See [Parametric Geometry Specification](#parametric-geometry-specification) below for the
`ParametricAircraftConfig` design.

**Limitations.**

- OpenVSP binary must be installed separately (not a Conan/pip dependency); VSPAERO is
  distributed as part of OpenVSP
- VSPAERO VLM accuracy is comparable to AVL; panel method is somewhat higher fidelity
  but slower
- The Python API parameter names are not always intuitive; an abstraction layer
  (`VspGeometryBuilder`) is needed to avoid scattering OpenVSP-specific names throughout
  calling code
- Like all VLM tools, VSPAERO does not predict stall or post-stall behavior

---

### Method 7 — High-Fidelity CFD *(High Interest for Validation and Surrogate Training)*

**Description.** Computational fluid dynamics (CFD) solves the Reynolds-Averaged
Navier-Stokes (RANS) equations on a volume mesh surrounding the aircraft. Unlike VLM, RANS
captures viscous effects, flow separation, thick-body pressure distribution, and nonlinear
lift behavior at high $\alpha$ — the dominant contributors to errors in VLM-based
derivatives.

#### Local RANS Solvers

| Tool | License | Notes |
| --- | --- | --- |
| OpenFOAM | GPL v3 | General-purpose; rhoCentralFoam for external flow; widely used for aircraft aerodynamics |
| SU2 | LGPL v2.1 | Designed for aerodynamic shape optimization; built-in stability derivative computation via automatic differentiation |
| FUN3D | NASA proprietary | High accuracy; available to US entities via data user agreement; not freely redistributable |
| OVERFLOW | NASA proprietary | Overset structured grids; preferred for complex configurations |

The practical barrier to local CFD is computational cost: a single RANS run at one flight
condition typically requires 2–8 hours on a workstation, and a stability derivative sweep
($\pm\alpha$, $\pm\beta$, $\pm$rate perturbations) requires 10–20 runs. Mesh generation
adds additional setup time.

#### Cloud-Based RANS Solvers *(High Interest)*

Cloud execution eliminates the local hardware barrier and enables parametric sweeps that
would be impractical on a single workstation:

| Platform | Solver | Notes |
| --- | --- | --- |
| SimScale | OpenFOAM (cloud) | Browser-based interface; automated meshing (SnappyHexMesh); Python API for batch job submission; results via REST API |
| Rescale | OpenFOAM, SU2, others | HPC-as-a-service; full solver choice; scriptable via Rescale Python SDK; cost per core-hour |
| AWS + OpenFOAM | OpenFOAM on EC2 | Self-managed; HPC cluster via AWS ParallelCluster; spot instances reduce cost; full automation via boto3 |
| Azure + OpenFOAM | OpenFOAM on CycleCloud | Similar to AWS path; tighter HPC integration |
| Cadence Fidelity CFD | Commercial cloud | Formerly Numeca; higher automation capability; per-seat or cloud licensing |

Cloud RANS is most valuable in combination with the [Parametric Geometry Specification](#parametric-geometry-specification)
and the surrogate model (Method 8): a Design of Experiments sweep across aircraft
configuration parameters generates the training dataset without requiring local HPC
infrastructure. Each cloud run produces the full set of body-axis stability derivatives
at a specific design point; the surrogate then generalizes across the parameter space.

**SimScale Python API (illustrative pattern):**

```python
from simscale_sdk import ApiClient, SimulationsApi, RunsApi
# Build geometry (from OpenVSP .vsp3 export), upload, configure mesh and solver,
# submit run, poll for completion, download results CSV.
# Full pipeline scriptable; no GUI interaction required.
```

**Role in this project.**

- Primary: final validation of $C_{D_0}$ and nonlinear $C_L(\alpha)$ at high $\alpha$
  for the three design study cases (Cases A, B, C)
- Secondary: generating ground-truth training data for surrogate model (Method 8) via a
  parametric sweep driven by `ParametricAircraftConfig`
- Not suitable for the real-time estimation pipeline (too slow for interactive use)

---

### Method 8 — Machine Learning Surrogates (Future)

**Description.** A surrogate model trained on a parametric dataset of
geometry-to-coefficient pairs can predict stability derivatives for arbitrary new
geometries in milliseconds, combining near-RANS accuracy with near-empirical speed.

**Candidate model types:**

| Model | Strengths | Weaknesses |
| --- | --- | --- |
| Gaussian Process Regression | Uncertainty quantification; well-calibrated extrapolation warnings | Scales poorly beyond ~10k training points |
| Neural network (MLP) | High capacity; handles nonlinear interactions | Requires large dataset; black-box |
| Gradient Boosted Trees (XGBoost, LightGBM) | Fast training; robust to outliers; interpretable feature importance | Weaker on smooth interpolation |
| Physics-Informed Neural Network (PINN) | Enforces known physical relationships (e.g. $C_{l_\beta}$ sign for positive dihedral) | Research-level; complex training setup |

**Training data strategy — Design of Experiments (DOE).**

A surrogate is only as good as the coverage of its training set. For a geometry space
with $n$ parameters, a random sample is inefficient — DOE methods are used to achieve
maximum information per run:

| DOE method | Description | Use case |
| --- | --- | --- |
| Latin Hypercube Sampling (LHS) | Stratified random sampling; fills the parameter space uniformly | General-purpose; good for 5–20 parameters |
| Full factorial | All combinations of discrete levels | Low-dimensional studies ($\le 5$ parameters) |
| Central composite design (CCD) | Factorial + axial + center points; fits quadratic response surface | Response surface models for specific derivative trends |
| Sobol sequence | Low-discrepancy quasi-random sequence; better space-filling than LHS | Large parametric sweeps ($\ge 15$ parameters) |

#### DOE and Surrogate Libraries

**Python (primary path):**

| Library | Purpose | License | Notes |
| --- | --- | --- | --- |
| `scipy.stats.qmc` | LHS, Sobol, Halton sequences | BSD-3 | Part of SciPy; preferred for zero added dependency; available since SciPy 1.7 |
| `pyDOE3` | Full factorial, fractional factorial, CCD, Box-Behnken, LHS | BSD-3 | Lightweight; purpose-built for classical DOE designs; active fork of `pyDOE2` |
| `SALib` | Sobol sensitivity analysis, Morris screening, FAST | MIT | Useful for identifying which geometry parameters have the highest derivative influence before committing to a large DOE |
| `SMT` (Surrogate Modeling Toolbox) | LHS sampling + Kriging, RBF, polynomial, KPLS surrogates in one package | BSD-3 | Best single-library choice if both DOE and surrogate modeling are needed together; well-documented for aerospace applications |
| `scikit-learn` | GP regression, gradient boosted trees (XGBoost-compatible), neural networks | BSD-3 | Already in common use; good for ensemble surrogates; `GaussianProcessRegressor` is production-ready |
| `GPflow` | Gaussian processes on GPU via TensorFlow | Apache 2.0 | Preferred over scikit-learn GP when training sets exceed ~5000 points |
| `GPyTorch` | Gaussian processes on GPU via PyTorch | MIT | Alternative to GPflow; supports batch GP for multi-output models (all derivatives at once) |

**C++ (for tightly integrated or embedded workflows):**

| Library | Purpose | License | Notes |
| --- | --- | --- | --- |
| Dakota (Sandia NL) | Comprehensive DOE, sensitivity analysis, surrogate modeling, optimization | LGPL v2.1 | The standard in aerospace UQ/design optimization; Python bindings available; heavyweight but battle-tested; suitable if liteaero-sim ever needs to drive optimization from C++ |
| `nlopt` | Nonlinear optimization (derivative-free and gradient-based) | MIT/LGPL | Useful downstream for surrogate-based design optimization; lightweight and embeddable |

**Recommendation for this project.** Use `scipy.stats.qmc` for Sobol/LHS sampling (no
extra dependency) and `SMT` for surrogate training (Kriging + LHS in one package, with
aerospace-oriented documentation). Use `scikit-learn` if gradient-boosted tree surrogates
are compared alongside Kriging. Dakota is noted for future interest if design optimization
is added to the roadmap.

The [Parametric Geometry Specification](#parametric-geometry-specification) defines the
design parameters; each DOE sample point maps to a `ParametricAircraftConfig` instance,
which drives the OpenVSP Python API to generate a geometry and then VSPAERO or a cloud
RANS solver to compute the derivatives. This pipeline can be fully automated:

```text
DOE sample points (N × k matrix)
    │
    ▼
ParametricAircraftConfig × N    ← one per sample point
    │
    ▼
VspGeometryBuilder              ← generates .vsp3 for each
    │
    ▼
VSPAERO or cloud RANS           ← computes BodyAxisCoefficients for each
    │
    ▼
Training dataset (N × d)        ← d = number of derivative outputs
    │
    ▼
Surrogate model training        ← scikit-learn, PyTorch, GPflow, etc.
    │
    ▼
SurrogateCoeffEstimator         ← Python callable, replaces AeroCoeffEstimator
                                   for configurations outside DATCOM range
```

**Accuracy expectation.** With 1000–5000 VSPAERO samples across the design space, a
well-tuned GPR or neural network can achieve ±3–8% on most derivatives — significantly
better than empirical formulas and at VSPAERO speed once trained.

**Status.** Not in scope for the current design study. The `ParametricAircraftConfig`
and OpenVSP pipeline are prerequisites; the surrogate training step follows naturally
once sufficient data exists.

---

## Parametric Geometry Specification

The current `AircraftGeometry` and `SurfaceGeometry` structs capture the minimum
parameters needed for the empirical DATCOM pipeline. They are not designed for:

- Generating OpenVSP `.vsp3` files with the correct component topology
- Representing non-conventional configurations (canards, blended-wing-body, multi-segment
  wings, multiple propulsors)
- Systematic DOE sweeps across design variables
- Round-tripping between the estimator, the VLM solver, and a human designer

A richer `ParametricAircraftConfig` is proposed as a future capability. It is the
geometry representation that feeds all downstream tools (OpenVSP, VSPAERO, cloud RANS,
and the surrogate DOE pipeline). It is not required for the current design study but
should be designed before any work on Methods 6–8 begins.

### Design Goals

1. **Parametric.** All dimensions are named design variables; changing one variable
   automatically re-derives dependent geometry (e.g. MAC, aerodynamic center location,
   tail moment arm).
2. **Topology-aware.** The configuration declares which components are present (wing,
   H-tail, V-tail, canard, fuselage, nacelles, etc.) and how they are attached. The
   OpenVSP component tree is derived from this topology.
3. **OpenVSP Python API compatible.** Each field maps to a specific OpenVSP parameter
   name; `VspGeometryBuilder` is a thin translation layer, not a geometry engine in its
   own right.
4. **DOE-friendly.** The config can be constructed from a flat vector of design variable
   values, enabling systematic sampling without manual field-by-field setup.
5. **Backward compatible with `AircraftGeometry`.** `ParametricAircraftConfig` can
   produce an `AircraftGeometry` instance for use with the existing empirical pipeline,
   so both estimation paths remain available for the same configuration.

### Proposed Structure (Sketch)

```python
# Proposed — not yet implemented
from dataclasses import dataclass, field

@dataclass
class WingConfig:
    span_m:             float        # tip-to-tip
    root_chord_m:       float
    taper_ratio_nd:     float        # c_tip / c_root
    le_sweep_deg:       float        # leading-edge sweep
    dihedral_deg:       float = 0.0
    incidence_deg:      float = 0.0
    airfoil_name:       str   = "NACA 2412"
    # Control surface
    aileron_inboard_eta_nd:  float = 0.6   # fraction of semi-span
    aileron_outboard_eta_nd: float = 0.95
    aileron_chord_frac_nd:   float = 0.25

@dataclass
class TailConfig:
    span_m:          float
    root_chord_m:    float
    taper_ratio_nd:  float
    le_sweep_deg:    float
    x_le_root_m:     float   # body x of root leading edge
    z_offset_m:      float = 0.0   # T-tail: positive up (body z down)
    control_chord_frac_nd: float = 0.35  # elevator or rudder fraction

@dataclass
class FuselageConfig:
    length_m:    float
    max_diam_m:  float
    x_cg_frac:   float = 0.30   # CG as fraction of fuselage length

@dataclass
class ParametricAircraftConfig:
    name:       str
    wing:       WingConfig
    fuselage:   FuselageConfig
    h_tail:     TailConfig | None = None   # None for flying wing
    v_tail:     TailConfig | None = None
    # Derived fields (computed on construction)
    # x_cg_m, mac_m, tail_arm_m, etc.

    def to_aircraft_geometry(self) -> AircraftGeometry:
        """Project to flat AircraftGeometry for empirical estimator."""
        ...

    def to_vsp3(self, output_path: str) -> None:
        """Write OpenVSP file via openvsp Python API."""
        ...

    def design_vector(self) -> list[float]:
        """Flat parameter vector for DOE sampling."""
        ...

    @staticmethod
    def from_design_vector(v: list[float], template: "ParametricAircraftConfig") \
            -> "ParametricAircraftConfig":
        """Reconstruct config from DOE sample point."""
        ...
```

### Design Variables for DOE

When used with a DOE sampler, the following parameters form a representative design
space for conventional fixed-wing UAV configurations:

| Variable | Symbol | Typical range | Notes |
| --- | --- | --- | --- |
| Wing span | $b$ | 0.6–6 m | Primary sizing variable |
| Wing aspect ratio | $A\!\!R$ | 4–18 | Derived from $b$ and $S$ |
| Wing taper ratio | $\lambda$ | 0.3–1.0 | 1.0 = untapered |
| Wing leading-edge sweep | $\Lambda_0$ | 0–35° | |
| Wing dihedral | $\Gamma$ | −5–10° | |
| Tail volume coefficient | $V_{HT}$ | 0.3–0.7 | Encodes $l_{HT} S_{HT} / (S \bar{c})$ |
| Fin volume coefficient | $V_{VT}$ | 0.02–0.06 | |
| CG position | $x_{CG}/l_f$ | 0.25–0.40 | |
| Fuselage fineness ratio | $l_f/d_f$ | 4–12 | |
| Airfoil thickness ratio | $(t/c)$ | 0.08–0.18 | |

With $k = 10$ design variables and Latin Hypercube Sampling at $N = 500$–$2000$ sample
points, a surrogate model can be trained that covers the full conventional fixed-wing
UAV design space with good accuracy.

---

## Architecture

### Current Interface

```cpp
// AeroCoeffEstimator.hpp (current)
class AeroCoeffEstimator {
public:
    [[nodiscard]] static std::pair<AeroPerformance, LiftCurveParams>
        estimate(const AircraftGeometry& geom);
};
```

### Target Interface (Extension for BodyAxisCoeffModel)

The estimator must be extended to produce `BodyAxisCoefficients` (all 25 derivatives plus
reference geometry). The trim-model path must remain unchanged — `Aircraft` (load-factor
model) must continue to work without modification.

```cpp
// Proposed extension (not yet implemented)
struct BodyAxisCoefficients {
    // Reference geometry
    float s_ref_m2;
    float mac_m;
    float span_m;
    // All 25 stability derivative fields (see aero_coefficient_model.md)
    // ...
};

class AeroCoeffEstimator {
public:
    // Existing path — unchanged
    [[nodiscard]] static std::pair<AeroPerformance, LiftCurveParams>
        estimate(const AircraftGeometry& geom);

    // New path — body-axis derivatives
    [[nodiscard]] static BodyAxisCoefficients
        estimateBodyAxis(const AircraftGeometry& geom,
                         const ControlSurfaceGeometry& controls);
};
```

### Required Addition: `ControlSurfaceGeometry`

Control effectiveness derivatives ($C_{Z_{\delta_e}}$, $C_{m_{\delta_e}}$,
$C_{l_{\delta_a}}$, $C_{n_{\delta_r}}$, etc.) cannot be estimated from `AircraftGeometry`
alone — control surface dimensions are not currently in that struct. A new struct is
needed:

```cpp
// Proposed (not yet implemented)
struct ControlSurfaceGeometry {
    // Elevator
    float elevator_span_fraction_nd; // fraction of HT span covered by elevator
    float elevator_chord_fraction_nd; // e/c ratio (control chord / local chord)

    // Aileron
    float aileron_inboard_eta_nd;    // inboard span station (y / (b/2))
    float aileron_outboard_eta_nd;   // outboard span station
    float aileron_chord_fraction_nd; // a/c ratio

    // Rudder
    float rudder_span_fraction_nd;   // fraction of VT span covered by rudder
    float rudder_chord_fraction_nd;  // r/c ratio
};
```

This struct extends the interface without modifying `AircraftGeometry`, preserving
backward compatibility with the existing trim model path.

### External Tool Integration Pattern

External tools (AVL, XFLR5, OpenVSP/VSPAERO) are integrated through a file-based adapter
pattern. Each adapter is a separate class that does not depend on the external tool being
present at compile time:

```
AircraftGeometry
ControlSurfaceGeometry     ──►  AvlGeometryWriter  ──►  .avl file
                           ──►  XflrGeometryWriter  ──►  .xfl file
                           ──►  VspGeometryWriter   ──►  .vsp3 file

avl process output         ──►  AvlOutputParser     ──►  BodyAxisCoefficients
XFLR5 XML export           ──►  XflrOutputParser    ──►  BodyAxisCoefficients
VSPAERO CSV output         ──►  VspaeroOutputParser ──►  BodyAxisCoefficients
```

Adapters are used during the design study (offline) and optionally in the test suite when
the external binary is available. They must never be mandatory for a CI build to pass.

---

## Method Comparison

| Method | All 6DOF derivatives | Control effectiveness | Flying wing | Accuracy | Speed | Automation | DOE / surrogate training |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Empirical (DATCOM) | Partial (extendable) | With `ControlSurfaceGeometry` | Poor | ±15–25% | Instant | Full | Not recommended (accuracy) |
| Lifting-line (LLT) | No (lift/drag/roll only) | No | Poor (sweep limit) | ±10–20% | Fast | Full | Limited |
| VLM (internal) | Yes | Yes | Yes | ±10–20% | Fast | Full | Suitable |
| AVL (subprocess) | Yes | Yes | Yes | ±10–20% | Fast | Conditional | Suitable |
| XFLR5 (file import) | Yes | Yes | Yes | ±5–15% (with XFOIL) | Medium | Manual | Not suitable |
| **OpenVSP + VSPAERO** | Yes | Yes | Yes | ±10–20% | Fast | **Full (Python API)** | **Preferred — scriptable DOE** |
| CFD / cloud RANS | Yes | Yes | Yes | ±2–5% | Hours–days | Full (cloud API) | Preferred for ground truth |
| Surrogate (trained) | Yes | Yes | Yes | ±3–8% (after training) | Instant | Full | N/A (is the output) |

**Recommended path for design study (item 1):**

1. Extend the empirical pipeline (DATCOM) with the missing damping and moment derivatives
   — lowest-risk, no new dependencies, unblocks `BodyAxisCoeffModel` format decision.
2. Use AVL subprocess for independent cross-validation of Cases A, B, and C.
3. Record discrepancies as a calibration baseline.

**Recommended path for post-study tooling (future roadmap items):**

1. Design and implement `ParametricAircraftConfig` with `VspGeometryBuilder` (OpenVSP
   Python API). This is the enabling capability for all downstream automated pipelines.
2. Integrate VSPAERO as the primary automated derivative estimator for configurations
   outside the DATCOM validated range.
3. Set up a DOE sweep (Latin Hypercube, 500–2000 samples) using `ParametricAircraftConfig`
   and VSPAERO to generate a training dataset for the surrogate model.
4. Evaluate cloud RANS (SimScale or AWS + OpenFOAM) for a subset of the DOE points to
   provide high-fidelity ground truth for surrogate validation.
5. Train and validate the surrogate model; integrate as `SurrogateCoeffEstimator`.

---

## Open Questions

| ID | Summary | Blocking | Recommendation |
| --- | --- | --- | --- |
| OQ-E1 | **`ControlSurfaceGeometry` interface.** Where should this struct live — in `include/aerodynamics/AircraftGeometry.hpp` alongside `AircraftGeometry`, or in a separate header? Should it be an optional member of `AircraftGeometry` (with defaults representing no control surfaces)? | `AeroCoeffEstimator` extension | Decide before writing any control-effectiveness estimation code. Preferred: separate struct, passed explicitly to `estimateBodyAxis()`. |
| OQ-E2 | **Flying-wing $C_{m_\alpha}$ without a horizontal tail.** The DATCOM tail-volume formula for $C_{m_\alpha}$ is undefined for a tailless aircraft. What formula is used for Case A? | Design study Case A | Apply the reflexed-trailing-edge or center-of-pressure method from Raymer §16.3; document explicitly. |
| OQ-E3 | **AVL binary availability in CI.** If AVL is used for cross-validation in the design study, should AVL be present in the MSYS2 ucrt64 environment? Or should the validation tests be marked as optional (skipped when binary absent)? | CI build | Decision: make AVL-dependent tests `DISABLED_` by default (Google Test convention); enable manually with a CMake option. |
| OQ-E4 | **Dihedral as a geometry input.** `AircraftGeometry.SurfaceGeometry` has no dihedral field. $C_{l_\beta}$ estimation from DATCOM §4.2.1 requires wing dihedral $\Gamma$. Should `SurfaceGeometry` be extended, or should dihedral be supplied as a separate parameter? | $C_{l_\beta}$ estimation | Extend `SurfaceGeometry` with `dihedral_rad`; default zero. |
| OQ-E5 | **Relationship between `aerodynamics.md` and this document.** `aerodynamics.md` currently documents the algorithm detail for the trim path. Should it be extended in-place with the body-axis formulas, or should a new `aerodynamics_body_axis.md` be created? | Documentation structure | Recommendation: extend `aerodynamics.md` with Parts 10–N for new derivatives; keep this document as the design authority for architecture decisions. |
