# Defect Report — KinematicState Attitude Model: Trim Aero vs. 6DOF Confusion

**Status:** Open  
**Affects:** `KinematicState` (liteaero-sim), `equations_of_motion.md` (liteaero-sim)  
**Discovered during:** Ground contact dynamics investigation, 2026-05-09

---

## Background: Two Distinct Attitude Models

The project uses a **trim aerodynamic (trim aero) point-mass model**, not a 6DOF rigid-body model.
These two models share some translational kinematics (position and velocity integration via RK4 on
the WGS84 state) but differ fundamentally in how attitude is defined and propagated.

### 6DOF Attitude Model

In a 6DOF model, attitude is the PRIMARY integrated state. The body quaternion `q_nb` is evolved
via the quaternion ODE driven by body angular rates `[p, q, r]`:

$$
\dot{q}_{nb} = \tfrac{1}{2}\,q_{nb} \otimes [0,\,p,\,q,\,r]^T
$$

Body rates are either measured (sensor path) or commanded (inner-loop control path). Attitude is
initialized from known conditions and propagates forward in time independently of the force field.
Aerodynamic angles α and β are then DERIVED from the resulting body attitude and the velocity
vector.

### Trim Aero Attitude Model

In a trim aero model, the force balance is the PRIMARY constraint. The aerodynamic angles α and β
are INPUTS (solved by the load factor allocator each step) and are not independently integrated.
Attitude is therefore DERIVED FROM the velocity vector and the aerodynamic angles — it is not an
independent integrated state.

The attitude chain for the trim aero model is:

1. **Velocity vector direction → VW frame (algebraic).**  
   The Velocity-Wind frame (VW) is defined algebraically from the airmass-relative velocity:

   $$C_{VW,N} = C_y(-\gamma_a)\,C_z(\chi_a)$$

   where $\chi_a$ is the aerodynamic track angle and $\gamma_a$ is the aerodynamic flight path angle.
   This is a deterministic function of the current velocity vector; it contains no accumulated error.

2. **Roll about the velocity axis → Wind frame (accumulated).**  
   The Wind frame (W) is the VW frame rolled by the wind-axis bank angle $\mu$:

   $$C_{VW,W} = C_x(\mu) \quad\Longrightarrow\quad q_{nw} = q_{nVW} \otimes R_x(\mu)$$

   $\mu$ is the only quantity that must be integrated over time; it is driven by the commanded
   wind-frame roll rate $p_W$.

3. **Body frame → derived from Wind frame and α, β (algebraic).**

   $$q_{nb} = q_{nw} \otimes q_{wb}, \qquad C_{BW} = C_y(\alpha)\,C_z(-\beta)$$

   Body rates `[p, q, r]` are derived as a function of path-curvature rates, roll rate, and
   α̇, β̇ — they are NOT integrated.

---

## Current Implementation

### Primary State

`KinematicStateSnapshot` (liteaero-flight, `include/liteaero/nav/KinematicStateSnapshot.hpp`,
line 33) correctly stores `q_nw` as the primary orientation quaternion.  `q_nb` is documented as a
derived quantity:

> "q_nb is not stored because it is exactly derivable from q_nw, alpha_rad, and beta_rad."

The snapshot stores `roll_rate_wind_rad_s` as the instantaneous wind-frame roll rate; it does NOT
store the accumulated wind-axis bank angle `μ` as a separate scalar.

### `stepQnw` — Differential Path Tracking

`KinematicState::stepQnw` (`src/KinematicState.cpp`, lines 235–257):

```cpp
Eigen::Quaternionf diff_rot_n;
diff_rot_n.setFromTwoVectors(velocity_prev_NED_mps, velocity_NED_mps);
q_nw = (diff_rot_n * q_nw * roll_delta).normalized();
```

**What it does:**
- Computes the minimal rotation (in NED) that maps the previous velocity vector to the new one.
- Pre-multiplies `q_nw` by this rotation in NED — i.e., tracks velocity direction changes by
  accumulating differential rotations.
- Right-multiplies by `roll_delta = Rx(p_W · dt)` to accumulate roll about the Wind x-axis.

**What the trim aero model requires:**
- The velocity-direction component of `q_nw` should be computed **algebraically** from the current
  velocity vector (equivalent to `stepQnv` applied to the airmass velocity), not accumulated from
  differential rotations.
- The roll component `μ` should be accumulated separately and layered on top.

The differential approach is mathematically equivalent to the correct approach only in the limit of
infinitely small timesteps with smooth velocity changes. For large velocity direction changes — such
as those produced by landing gear contact forces — the accumulated error is unbounded and permanent.

### `stepQnv` — Algebraic Velocity Frame (Not Called)

`KinematicState::stepQnv` (`src/KinematicState.cpp`, lines 259–281) constructs `q_nv`
algebraically from the velocity vector each step. This is the correct approach for the velocity-
direction component of a trim aero frame.

However, it is **never called** from within `KinematicState::step()`. The public accessor
`q_nv()` (`include/KinematicState.hpp`, line 82) returns `Eigen::Quaternionf::Identity()`
unconditionally:

```cpp
Eigen::Quaternionf q_nv() const { return Eigen::Quaternionf::Identity(); } // not implemented
```

`stepQnv` is dead code.

---

## Defects

### D-1 — `stepQnw` Uses Differential (6DOF-Style) Integration Instead of Algebraic Reconstruction

**File:** `src/KinematicState.cpp`, lines 235–257  
**Severity:** High — root cause of the ground contact dynamics corruption bug

The velocity-direction component of `q_nw` is updated by accumulating differential rotations via
`setFromTwoVectors(v_prev, v_new)`. This is the 6DOF approach of integrating angular velocity. In
the trim aero model, the velocity-direction component is algebraically defined by the current
velocity vector and must be RECONSTRUCTED each step, not accumulated.

**Consequence:** Any non-aerodynamic force that causes a large velocity direction change in a single
timestep (landing gear spring, body collider, terrain hard constraint) permanently corrupts `q_nw`.
Once `q_nw` is corrupted, the gravity vector `g_wind = R_nw^T · {0,0,g}` computed in
`Aircraft::step()` (line 270) points in the wrong direction in Wind frame, producing a phantom
acceleration of up to `2g` directed against the true gravity vector. This is the mechanism behind
the observed "floating upward" symptom after ground contact.

### D-2 — Wind-Axis Bank Angle `μ` Is Implicitly Encoded in `q_nw`

**File:** `include/liteaero/nav/KinematicStateSnapshot.hpp`  
**Severity:** Low — no state change required; resolved by the quaternion roll residual approach

The wind-axis bank angle `μ` (the roll of the Wind frame about the velocity vector) is implicitly
encoded in `q_nw` as the angular difference between `q_nw` and `q_nVW`. It is not stored as a
separate scalar state.

**Resolution (OQ-2, OQ-4):** A scalar encoding of `μ` must NOT be introduced. At vertical flight
path angles the roll axis (the velocity vector) becomes degenerate and the scalar wraps at ±π,
producing orientation discontinuities. The quaternion roll residual approach adopted for fixing D-1
carries the roll component as a full quaternion residual `q_roll = q_nVW_prev⁻¹ ⊗ q_nw_prev`,
which is globally smooth through any trajectory. No scalar `μ` state is ever extracted or stored.

**Consequence for D-1 fix implementation:** When rebuilding `q_nw` each step, the roll component
must be preserved as the quaternion residual, not lost. The D-1 algorithm (see OQ-2 decision)
handles this correctly: `q_roll` is computed before `q_nVW_new` replaces `q_nVW_prev`, then
re-applied. No change to `KinematicStateSnapshot` is required.

**Initialization:** At sim start, `q_nw` is constructed from `q_nb` (via Constructor 2 using
`alpha`, `beta`, and the `roll_rad` config parameter). The roll component is therefore directly
set by configuration. At snapshot restore, `q_nw` is stored directly in `KinematicStateSnapshot`
and is immediately available as both the full quaternion and the source of `q_roll` for the
first step. No separate `μ` initialization path is needed.

### D-3 — `stepQnv` Is Defined but Never Called; `q_nv()` Returns Identity

**File:** `include/KinematicState.hpp` line 82; `src/KinematicState.cpp` lines 259–281  
**Severity:** Low (dead code) — but masks the correct building block for fixing D-1

`stepQnv` contains the algebraic VW-frame construction that IS needed for the trim aero model fix.
It is unreachable via the public interface and its public accessor always returns identity, making
any caller silently incorrect.

### D-4 — `equations_of_motion.md` Integration Scheme Summary Describes 6DOF Attitude, Not Trim Aero

**File:** `docs/algorithms/equations_of_motion.md`, lines 1112–1126 (Integration Scheme Summary)  
**Severity:** Medium — documentation defect that obscures the correct trim aero algorithm

The Integration Scheme Summary shows:

```
ATT["Propagate quaternion
     q_{nb,k+1} = q_{nb,k} + Δt/2 · q ⊗ ω
     then renormalize"]
```

This is the 6DOF quaternion ODE. The trim aero model does not propagate `q_nb` via a quaternion
ODE. `q_nb` is a derived quantity in this model; it is never directly integrated.

The same section's "Implementation Notes" (line 1129) states `_q_nb` is the primary attitude
quaternion. This is incorrect — the primary stored quaternion is `q_nw` (see `KinematicStateSnapshot`,
line 33).

### D-5 — `equations_of_motion.md` Does Not Clearly Separate Trim Aero and 6DOF Models

**File:** `docs/algorithms/equations_of_motion.md`  
**Severity:** Medium — caused D-4 and contributed to D-1 being introduced

The document contains correct derivations for both models interleaved without explicit labeling of
which applies to this simulator:

| Section | Model described | Applicable? |
| --- | --- | --- |
| Attitude Kinematics — Quaternion ODE | 6DOF (q_nb integrated) | No |
| Body Rates from Wind Frame Rates | Trim aero | Yes |
| Integration Scheme Summary | 6DOF (q_nb propagated) | No |
| Implementation Notes | Incorrectly claims q_nb primary | No |
| Velocity and Velocity-Wind Frames | Trim aero | Yes |
| Path Angle Rates | Trim aero | Yes |

A reader cannot tell from the document which integration scheme is actually implemented.

---

## Observable Symptom Linking to D-1

The ground contact bug observed in the Godot simulation is a direct consequence of D-1:

1. Aircraft makes second gear contact with sufficient vertical velocity.
2. The gear spring produces a large vertical velocity reversal within one timestep.
3. `setFromTwoVectors(v_prev, v_new)` returns a near-180° rotation.
4. `q_nw` is corrupted by this large rotation.
5. `g_wind = R_nw^T · {0, 0, g}` in `Aircraft::step()` now points approximately opposite to its
   correct direction.
6. `az = F.z_n/m + g_wind.z ≈ -g + (-g) = -2g` — net upward acceleration of 2g regardless of
   commanded load factor.
7. The aircraft accelerates upward with nose attitude displayed as ~-30° because `q_nb`, derived
   from the corrupted `q_nw`, no longer represents the actual aerodynamic trim attitude.

---

## Open Questions

**OQ-1 — Resolved: no separate initialization path required.**  
This question was wrongly framed around scalar μ. There is no μ state to initialize.

At simulation start, `q_nw` is initialized through the existing KinematicState Constructor 2
path: `Aircraft::initialize()` reads optional `heading_rad`, `pitch_rad`, and `roll_rad` fields
from `initial_state` (all defaulting to 0), constructs `q_nb`, and Constructor 2 derives `q_nw`
from `q_nb`, `alpha`, and `beta`. Roll attitude at sim start is therefore fully covered by the
existing `roll_rad` config parameter.

At snapshot restore, `q_nw` is stored directly in `KinematicStateSnapshot` and is immediately
available. `q_nVW_prev` for the first step after restore is computed algebraically from
`snapshot_.velocity_ned_mps`, which is also stored. The quaternion roll residual
`q_roll = q_nVW_prev⁻¹ ⊗ q_nw` is available without any additional state.

**OQ-2 — Resolved: use quaternion roll residual, not scalar μ.**  
A scalar encoding of μ has the same gimbal singularity as Euler angles: the roll axis (the velocity
vector) becomes degenerate at γ = ±90°, and the scalar wraps at ±π, producing orientation
discontinuities on any trajectory that passes through vertical flight path angles. Scalar μ must NOT
be used.

**Decision:** Preserve the roll component as a **quaternion residual** and never extract a scalar
angle. The correction algorithm at each step is:

1. Compute `q_nVW_new` algebraically from `v_new` (via `stepQnv` semantics).
2. Compute `q_nVW_prev` algebraically from `v_prev`.
3. Extract roll residual as a full quaternion: `q_roll = q_nVW_prev⁻¹ ⊗ q_nw_prev`.
4. Apply velocity correction: `q_nw_temp = q_nVW_new ⊗ q_roll`.
5. Accumulate new roll: `q_nw = (q_nw_temp ⊗ Rx(p_W · dt)).normalized()`.

This is entirely quaternion arithmetic — no scalar angle extraction, no atan2, no wrap-around. The
roll residual changes continuously through any velocity trajectory including vertical, because
quaternion multiplication is globally smooth. The VW-frame singularity (azimuth preservation
fallback in `stepQnv`) affects both `q_nVW_prev` and `q_nVW_new` consistently, so `q_roll` remains
continuous.

**OQ-3 — Resolved: velocity correction first, then roll increment about the post-increment velocity vector.**  
Apply the velocity-direction correction (steps 1–4) before accumulating the roll increment (step 5).
After step 4, `q_nw_temp = q_nVW_new ⊗ q_roll` has its Wind x-axis aligned with `v_new`. The
right-multiplication `⊗ Rx(p_W · dt)` in step 5 is therefore automatically a rotation about
`v_new` — the instantaneous (post-increment) velocity vector — with no additional projection or
re-expression required. The ordering is not merely equivalent to some other choice; it is the
physically correct one.

**OQ-4 — Resolved: do not add scalar μ to `KinematicStateSnapshot`.**  
The primary stored state remains `q_nw` (a full quaternion). The quaternion roll residual approach
(OQ-2 decision) requires no scalar state. The only change to `KinematicStateSnapshot` that might
be needed is caching `q_nVW_prev` between steps so it does not need to be recomputed from
`v_prev`; that is an implementation detail rather than a state design question.

**OQ-5 — Resolved: both force sources correctly contribute to path curvature during ground contact.**  
Path curvature `omega_wn_n` is derived from the total NED acceleration `snapshot_.acceleration_ned_mps2`,
which in `Aircraft::step()` is the Wind-frame sum of aerodynamic forces, thrust, gear contact
forces, and gravity — all rotated to NED. Both gear forces and aerodynamic forces drive path
curvature, and the body rate computation in `KinematicState::step()` correctly reflects this. The
body rates are physically meaningful during ground contact.

The residual concern is that `snapshot_.acceleration_ned_mps2` is set from the Wind-frame
acceleration passed to `KinematicState::step()`, which depends on `q_nw`. Until D-1 is fixed,
a corrupted `q_nw` produces a corrupted `acceleration_ned_mps2`, which in turn produces corrupted
path curvature and body rates. This is a symptom of D-1, not a separate defect.

---

---

## Implementation Plan

All open questions are resolved. Implementation proceeds in TDD order: failing tests first,
production code second. Items are sequenced so each builds on verified predecessors.

### IP-1 — Tests for algebraic VW-frame construction (D-3 prerequisite)

**Target:** `test/KinematicState_test.cpp`

Write tests verifying that `stepQnv` (or the refactored successor) produces the correct `q_nVW`
from known velocity vectors:

- **Level flight, north (γ = 0, χ = 0):** `q_nVW` = identity.
- **Level flight, east (γ = 0, χ = 90°):** correct yaw-only rotation; no pitch component.
- **Climbing flight (γ = 45°, χ = 0):** correct pitch-only rotation; no yaw component.
- **Zero-velocity degeneracy:** returns a defined (identity or last-valid) quaternion without NaN.
- **Orientation continuity near vertical — ascending approach:** Sweep γ through a sequence of
  angles approaching 90° from below (e.g., 80°, 85°, 88°, 89°, 89.9°, 90°) with a fixed
  horizontal component (χ = 45°). At each step compute `q_nVW` and verify that the angular
  distance `2 * acos(|q_i · q_{i+1}|)` between consecutive outputs is bounded by a small
  multiple of the input angle increment. The bound must hold all the way to γ = 90° — the
  singularity fallback must not introduce a discontinuous jump.
- **Orientation continuity near vertical — descending approach:** Same sweep from γ = −80°
  toward −90°, again with a fixed nonzero χ. Same continuity bound must hold.
- **Orientation continuity across vertical:** Sweep γ from 89° through 90° to 91° (velocity
  passes from steep climb through straight up to slightly past vertical). `q_nVW` must change
  by no more than the angular displacement corresponding to the 2° total sweep — i.e., it must
  not jump discontinuously as γ crosses 90°. Because χ is degenerate at exactly γ = 90°, the
  singularity fallback must preserve the previous azimuth (or a defined convention) rather than
  snapping to zero or producing a large rotation.
- **Azimuth independence at vertical:** At γ = 90° exactly, two velocity vectors with different
  horizontal components but the same vertical component (e.g., v = [ε, 0, −V] and
  v = [0, ε, −V] for small ε → 0) must produce `q_nVW` values that are close to each other
  and to the γ → 90° limit from the sweep above. This validates that the singularity fallback
  is consistent with the surrounding neighborhood.

These tests fully specify the contract for the VW-frame builder — correctness in the non-singular
regime, and orientation continuity (no jump discontinuities) through the vertical singularity —
before any production code changes.

### IP-2 — Activate `stepQnv`; fix `q_nv()` accessor (D-3)

**Targets:** `src/KinematicState.cpp`, `include/KinematicState.hpp`

Make `stepQnv` callable and return a real value from `q_nv()`. No behavior change to `stepQnw`
yet — this item just removes the dead-code status and verifies IP-1 tests pass.

Steps:

1. Call `stepQnv` from within `step()` to keep `q_nv_` up to date each tick.
2. Replace the stub `q_nv()` accessor body with `return q_nv_;` (or equivalent internal member).
3. IP-1 tests must pass. No other tests may regress.

### IP-3 — Tests for quaternion roll residual step algorithm (D-1 prerequisite)

**Target:** `test/KinematicState_test.cpp`

Write tests that will fail against the current `stepQnw` and pass only after IP-4:

- **Large velocity direction change, zero roll rate:** After a near-180° velocity reversal (as
  produced by landing gear contact), `q_nw` must align with the new velocity direction with zero
  residual roll. The current `stepQnw` will produce a corrupted quaternion; the new algorithm must
  not.
- **Roll accumulation under constant roll rate, constant heading:** `q_nw` must accumulate the
  correct wind-axis bank angle over multiple steps. Verify that `q_nb` (derived from `q_nw`,
  `alpha`, `beta`) reflects the expected bank.
- **Identity case:** Zero velocity change, zero roll rate, zero α and β → `q_nw` and `q_nb` both
  remain at their initial values.
- **Snapshot round-trip through velocity change:** Serialize `KinematicState`, restore, apply a
  large velocity direction change — `q_nw` must remain coherent (roll component preserved via
  `q_roll` extracted from the snapshot's stored `q_nw`).
- **Roll residual continuity through vertical — no roll input:** Starting from a steep climb
  (γ = 80°, χ = 45°, nonzero wind-axis bank angle, zero roll rate), drive γ through 90° and
  beyond (e.g., 80° → 85° → 90° → 95°) by applying a sequence of small velocity increments,
  each within a single `step()` call. After each step, extract the roll residual
  `q_roll = q_nVW⁻¹ ⊗ q_nw` and verify that its angular magnitude changes by no more than
  a small tolerance driven by floating-point precision — the roll residual must not jump as
  γ crosses 90°. This test will fail against the current `stepQnw` because `setFromTwoVectors`
  produces a large spurious rotation whenever the velocity direction changes sharply, whether
  due to a gear bounce or an arithmetically near-degenerate azimuth.
- **Roll residual continuity through vertical — with constant roll input:** Same trajectory as
  above but with a nonzero constant `p_W`. Verify that the accumulated roll residual grows
  monotonically at the commanded rate throughout the vertical crossing — it must not stall,
  reverse, or jump as γ passes 90°.

### IP-4 — Rewrite `stepQnw` with quaternion roll residual algorithm (D-1)

**Target:** `src/KinematicState.cpp`, lines 235–257

Replace the `setFromTwoVectors` differential accumulation with the five-step quaternion residual
algorithm:

```text
1. q_nVW_prev  = stepQnv(v_prev)                          // algebraic, from stored v_prev
2. q_nVW_new   = stepQnv(v_new)                           // algebraic, from current velocity
3. q_roll      = q_nVW_prev.inverse() * q_nw_prev         // quaternion residual; no scalar angle
4. q_nw_temp   = q_nVW_new * q_roll                       // velocity correction
5. q_nw        = (q_nw_temp * Rx(p_W * dt)).normalized()  // roll increment about v_new axis
```

`v_prev` is the velocity vector from the previous step; it must be cached in `KinematicState`
between calls to `step()` (or is already available as `snapshot_.velocity_ned_mps` at step entry
before the new velocity is written).

IP-3 tests must pass. No change to `KinematicStateSnapshot` fields is required.

### IP-5 — Reorganize and correct `equations_of_motion.md` (D-4, D-5)

**Target:** `docs/algorithms/equations_of_motion.md`

The document currently interleaves trim aero and 6DOF content without labeling either. The
required changes are a reorganization of the document's top-level structure combined with
targeted corrections to the attitude sections. The section inventory below identifies what
belongs where.

#### Section inventory

| Section (current) | Model | Action |
| --- | --- | --- |
| Overview | Common | Update the flowchart to include the `q_nw` attitude output; otherwise keep |
| Reference Frames | Common | Keep as-is |
| Velocity and Velocity-Wind Frames | **Trim aero** | Keep; move into trim aero part |
| Aerodynamic Angles | **Trim aero** | Keep; move into trim aero part |
| Attitude Kinematics — Quaternion ODE | **6DOF only** | Move into a clearly labeled 6DOF reference section; add a note that this model is NOT implemented |
| Velocity Kinematics — Wind Frame Input | **Trim aero** | Keep; move into trim aero part |
| Position Kinematics — WGS84 Integration | Common | Keep; move into trim aero part (RK4 applies to both but the implementation is shared) |
| Euler Angles (3-2-1 Convention) | Derived output | Keep; label as derived quantities for display, applicable to both models |
| Body Rate–Euler Rate Kinematics | 6DOF reference | Move into 6DOF reference section; the trim aero model derives body rates via the Wind-frame decomposition, not this Euler-rate inversion |
| Body Rates from Wind Frame Rates | **Trim aero** | Keep; move into trim aero part; this is the trim aero body rate equation |
| Plane of Motion Frame | **Trim aero** | Keep; move into trim aero part |
| Aerodynamic Angle Rates | **Trim aero** | Keep; move into trim aero part |
| Aerodynamic Force Vectors | **Trim aero** | Keep; move into trim aero part |
| Lift Curve Model | **Trim aero** | Keep; move into trim aero part |
| Thrust Decomposition and Load Factor Allocation | **Trim aero** | Keep; move into trim aero part |
| Derived Quantities | Common | Keep |
| Integration Scheme Summary | **Wrong** | Rewrite — see below |
| Implementation Notes | **Wrong** | Rewrite — see below |

#### Top-level document structure after reorganization

```text
# Equations of Motion

## Overview
## Reference Frames

## Part 1 — Trim Aero Model (Implemented)

  ### Trim Aero Overview
  ### Velocity and Velocity-Wind Frames
  ### Aerodynamic Angles
  ### Body Rates from Wind Frame Rates
  ### Plane of Motion Frame
  ### Aerodynamic Angle Rates
  ### Velocity Kinematics — Wind Frame Input
  ### Position Kinematics — WGS84 Integration
  ### Aerodynamic Force Vectors
  ### Lift Curve Model
  ### Thrust Decomposition and Load Factor Allocation
  ### Integration Scheme Summary — Trim Aero
  ### Implementation Notes

## Part 2 — 6DOF Reference (Not Implemented)

  ### 6DOF Overview
  ### Attitude Kinematics — Quaternion ODE
  ### Body Rate–Euler Rate Kinematics
  ### Euler Angles (3-2-1 Convention)

## Derived Quantities
```

A short preamble under "Trim Aero Overview" must state:

- This simulator implements the trim aero model. Part 1 describes this model.
- Part 2 is retained as reference material only. None of Part 2 is implemented.
- The primary attitude quaternion is `q_nw` (Wind-to-NED). `q_nb` is a derived quantity.

#### Corrections to Integration Scheme Summary

Replace the current mermaid flowchart node:

```text
ATT["Propagate quaternion
     q_{nb,k+1} = q_{nb,k} + Δt/2 · q ⊗ ω
     then renormalize"]
```

with the correct trim aero attitude update, which is a five-step algebraic reconstruction,
not a quaternion ODE integration:

```text
ATT["Update q_nw (trim aero)
     1. q_nVW_prev = algebraic(v_prev)
     2. q_nVW_new  = algebraic(v_new)
     3. q_roll     = q_nVW_prev⁻¹ ⊗ q_nw_prev   (quaternion residual)
     4. q_nw_temp  = q_nVW_new ⊗ q_roll          (velocity correction)
     5. q_nw       = (q_nw_temp ⊗ Rx(p_W·Δt)).normalized()"]
```

Also update the prose below the flowchart: attitude is NOT updated with forward Euler on a
quaternion ODE; `q_nw` is reconstructed algebraically each step with accumulated roll residual.

#### Corrections to Implementation Notes

Replace the stale member-variable list with one that reflects the current
`KinematicStateSnapshot` fields:

- Primary attitude quaternion: `q_nw` (Wind-to-NED, `Eigen::Quaternionf`) — stored in
  `KinematicStateSnapshot::q_nw`.
- `q_nb` is NOT stored; it is derived on demand from `q_nw`, `alpha_rad`, and `beta_rad`.
- Body rates `[p, q, r]` are derived each step from Wind frame rates, α, β, α̇, β̇ — they are
  not integrated.
- NED velocity: `snapshot_.velocity_ned_mps`
- NED acceleration: `snapshot_.acceleration_ned_mps2`
- Position: `snapshot_.position_wgs84` (WGS84 lat/lon/alt)
- Aerodynamic angles α, β are inputs to `step()` — computed by the aerodynamics subsystem.

### IP-6 — Cross-reference in `aircraft.md`

**Target:** `docs/architecture/aircraft.md`

Add a note in the Physics Integration Loop section referencing this defect document and the
corrected `stepQnw` algorithm, so future readers understand why the attitude update is not a
simple quaternion ODE.

---

## Files Requiring Changes

| File | Change required | Plan item |
| --- | --- | --- |
| `src/KinematicState.cpp` | Rewrite `stepQnw` with quaternion roll residual algorithm; activate `stepQnv` call | IP-2, IP-4 |
| `include/KinematicState.hpp` | Update `q_nv()` to return real value | IP-2 |
| `test/KinematicState_test.cpp` | Add algebraic VW-frame tests; add roll residual algorithm tests | IP-1, IP-3 |
| `docs/algorithms/equations_of_motion.md` | Label trim aero vs. 6DOF; correct Integration Scheme Summary and Implementation Notes | IP-5 |
| `docs/architecture/aircraft.md` | Cross-reference this defect in the Physics Integration Loop section | IP-6 |
