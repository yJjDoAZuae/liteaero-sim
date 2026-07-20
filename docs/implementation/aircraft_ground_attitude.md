# Aircraft Ground / At-Rest Attitude (OQ-AC-9) — Implementation Plan

**Scope:** Implements the resolved OQ-AC-9 fix for the ground-start attitude divergence (the velocity-slaved
`q_nw` runs to ~90° nose-up from a rest-on-gear start). Two parts: **Alt 2 — reference re-anchoring** (make
the `q_nw` propagation self-correcting so `q_nw.x = v̂_ref` is enforced every step), and **Alt 1 —
ground-trim attitude reference** (initialize `q_nw` to the static gear-trim attitude and converge to it as
`V → 0`, static state `α = Δθ = 0`). Derived from the design authority below. OQ-AC-9 and **OQ-AC-10** are
both resolved: OQ-AC-10 → the **relaxation** convergence (null the per-step `M_gear`), with the relaxation
time constant and the `Φ(V)` fade speed scale as **configuration parameters — no hardcoded magic numbers**.
Both parts are a *measured no-op above very low speed*, so every item is gated by strict TDD with no
regression of the existing above-low-speed ground-roll and flight behavior.

**Design authority:** [aircraft.md §Velocity-Slaved Attitude and Low-Speed Slew Saturation](../design/aircraft.md),
[aircraft.md §Ground-Trim Attitude](../design/aircraft.md), [aircraft.md §OQ-AC-9 / §OQ-AC-10](../design/aircraft.md),
[landing_gear.md §OQ-LG-21 / §OQ-LG-23](../design/landing_gear.md).

**Last updated:** 2026-07-20

---

## Work Items

| ID | Status | Title | Depends on | Design refs |
| --- | --- | --- | --- | --- |
| IP-AGA-1 | done | Permanent verbose Δθ / attitude debug accessors — `Aircraft::StepDiag` + `stepDiag()` → `AircraftStepDiag` (reference-chain elevations + `qnw_ref_desync_rad`) bound in `src/python/bind_aircraft.cpp`; `KinematicState::attitudeRefPrev_ned_mps()` | — | [aircraft.md §Diagnostics Accessors](../design/aircraft.md) |
| IP-AGA-2 | todo | Ground-rest pitch-up repro / init-trim acceptance test (`test_ground_rest_start_does_not_pitch_up_oq_ac9`, already added, `xfail`): initialize **at rest on the ground**, throttle up, assert bounded pitch and `qnw_ref_desync_rad ≈ 0`. This is the acceptance for the init-trim (IP-AGA-5) — it stays `xfail` until then. It currently forces a manual 1 m settle-drop only as a **stopgap** because init-trim does not yet place the aircraft; when IP-AGA-5 lands, rewrite it to rely on init-trim placement (remove the manual drop and the `xfail`) | IP-AGA-1 | [aircraft.md §OQ-AC-9](../design/aircraft.md), [§Ground-Trim Attitude](../design/aircraft.md) |
| IP-AGA-3 | todo | Re-anchor `KinematicState::stepQnw` — build `diff_rot_n = setFromTwoVectors(q_nw.col(0), v_ref_sat)` from `q_nw`'s own forward axis instead of `velocity_prev` (`att_ref_prev`); make IP-AGA-2 pass; verify no regression in OQ-AC-2 slew, OQ-AC-4 crab, and gear-only crabbed-touchdown scenarios, and that the wind-axis `roll_delta` bank is unchanged | IP-AGA-2 | [aircraft.md §Velocity-Slaved Attitude (reference re-anchoring)](../design/aircraft.md) |
| IP-AGA-4 | todo | Implement the static gear-trim equilibrium solve (body attitude + strut compressions in force + moment balance on a given ground plane; flat-ground rake as warm start) with unit tests | — | [aircraft.md §Ground-Trim Attitude](../design/aircraft.md) |
| IP-AGA-5 | todo | Init-trim: on an at-rest ground start (weight-on-wheels and `V ≈ 0`), initialize `q_nw` / `att_ref_prev` (and the ride height) to the gear-trim state so the aircraft starts settled — static rest state has `α = 0`, `Δθ = 0`, no residual tilt, no settle-drop. Acceptance = IP-AGA-2 rewritten to rely on init-trim placement (manual drop and `xfail` removed): no divergence on throttle-up | IP-AGA-3, IP-AGA-4 | [aircraft.md §Ground-Trim Attitude](../design/aircraft.md) |
| IP-AGA-6 | todo | Add the ground-trim relaxation **config parameters** (no magic numbers): `ground_trim_relax_tau_s` (relaxation time constant) and the `Φ(V)` fade speed scale (reuse `dtheta_vref_ratio` or add `ground_trim_fade_vref_ratio`) — to the config schema, the config JSONs/fixtures, `Aircraft::initialize` (read + range validation), and the aircraft proto with a JSON/proto round-trip test | — | [aircraft_config_v1.md](../schemas/aircraft_config_v1.md), [aircraft.md §Ground-Trim Attitude / §OQ-AC-10](../design/aircraft.md) |
| IP-AGA-7 | todo | Low-speed relaxation convergence: converge `q_nw` to the gear-trim attitude as `V → 0` by nulling `M_gear` (weight-on-wheels/`Φ(V)`-gated), realized as the attitude `slerp` blend, releasing into the OQ-LG-23 takeoff rotation; uses the IP-AGA-6 config parameters; test rollout-to-stop settles to the trim attitude and takeoff rotation begins from it | IP-AGA-4, IP-AGA-5, IP-AGA-6 | [aircraft.md §Ground-Trim Attitude / §OQ-AC-10](../design/aircraft.md), [landing_gear.md §OQ-LG-21 / §OQ-LG-23](../design/landing_gear.md) |

---

## Notes

**Sequencing rationale.** The re-anchor (IP-AGA-2/3) lands first: it is a small, localized change to
`stepQnw`, is self-correcting, and is a measured no-op wherever the velocity-slaving invariant already holds
(desync < 0.05° in high-speed roll, in-flight maneuvering, and realistic crab — see
[aircraft.md §Velocity-Slaved Attitude](../design/aircraft.md)). It removes the *accumulation* mechanism and
is fully guarded by the existing OQ-AC-2 / OQ-AC-4 / crabbed-touchdown scenario suites plus the new IP-AGA-2
repro, so it can be verified in isolation before any ground-trim work. The ground-trim reference (IP-AGA-4/5)
then removes the *seed* (the inconsistent at-rest IC) at its source; the config parameters (IP-AGA-6) land
before the relaxation convergence (IP-AGA-7) that consumes them (OQ-AC-10 resolved → relaxation).

**Configuration, not magic numbers.** The relaxation time constant and the `Φ(V)` fade speed scale are
config fields (IP-AGA-6), following the existing `dtheta_*` / `att_filt_tau_s` convention; no tuning constant
for the ground-trim convergence is hardcoded in `Aircraft` / `KinematicState`.

**No-regression requirement (project standing constraint for this plan).** Every item must leave the
above-very-low-speed ground-roll and flight behavior unchanged. The regression gate for IP-AGA-3, IP-AGA-5,
and IP-AGA-7 is the existing attitude/gear scenario tests (OQ-AC-2 slew, OQ-AC-4 crab, gear-only crabbed touchdown,
full-stop landing, powered touch-and-go) plus the `qnw_ref_desync_rad` no-op check across regimes.

**Algorithm document.** The static-solve mathematics for IP-AGA-4 (equilibrium equations, method,
existence/uniqueness on a slope, convergence) should be formalized in a dedicated `docs/algorithms/`
document when IP-AGA-4 is begun, per [aircraft.md §Ground-Trim Attitude](../design/aircraft.md).
