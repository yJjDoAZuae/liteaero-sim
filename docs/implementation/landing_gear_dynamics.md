# Landing Gear — Consolidated Implementation Plan

**Scope:** The single implementation plan for the landing-gear subsystem, covering the wheel
model (`WheelUnit`, `LandingGear`) and the gear→aircraft coupling path in `Aircraft` (§2a/§2b). It
consolidates the three former landing-gear plans (`landing_gear_dynamics.md` wheel dynamics,
`aircraft_gear_ground_forces.md` ground-force coupling, and `landing_gear_debt_closure.md`) into one
dependency-ordered, status-correct list. Four phases:

- **Phase A — wheel-internal dynamics** (`done`): kappa fix, Tustin spin ODE (OQ-LG-5), airborne
  bearing drag (OQ-LG-6), rolling-condition clamp added then removed.
- **Phase B — initial ground-force coupling** (mostly `superseded`): the `applyGearGroundConstraint`
  revert and damping fix (`done`), and the `_n_contact_z_filt` vertical-load suppression plus the
  moment-perturbation high-pass paths (OQ-LG-9/13) — **superseded** by Phase C.
- **Phase C — gear force-&-moment integration** (`done`): the current coupling design — force channel
  $G(s)$ (OQ-LG-18/20), destanced load + $C^2$ smootherstep authority fade $\Phi(V)$ (OQ-LG-19),
  moment channel $H_2$ (OQ-LG-16/17), filtered-velocity attitude reference (OQ-LG-21), apportionment
  relaxation $H_1$ (OQ-LG-22), additive settle/rotation term (OQ-LG-23), full non-dimensional
  parameterization (OQ-LG-17), and serialization. These items are recorded retroactively (implemented
  before this consolidated plan; verifiable in source).
- **Phase D — pending debt** (`todo`): the OQ-LG-24 effectiveness weight (resolved Alternative 1) and
  the OQ-LG-15 diagnostic-instrumentation cleanup.

**Resolved OQs reflected here:** OQ-LG-5, 6, 7, 9–24, and **OQ-LG-26** (the OQ-LG-24 weight `w_a` is a
smooth `smootherstep` on dynamic pressure over a band below stall `q` — §2b (b-iii); IP-LGD-26/27/28
implement it and the interim single-`V_ref` form was replaced). **Open OQs:** OQ-LG-1, 2, 3, 4, 8
(unrelated; no work item depends on them); and **OQ-LG-25** (whether/how to reconcile the
flight-physics smoothness-audit findings — gates no item in this plan).

**Design authority:** [`landing_gear.md`](../design/landing_gear.md),
[`aircraft.md`](../design/aircraft.md),
[`aircraft_config_v1.md`](../schemas/aircraft_config_v1.md)

**Last updated:** 2026-06-25 (OQ-LG-26 resolved → `w_a(q)` smootherstep band; IP-LGD-26/27/28 implemented and tested; IP-LGD-29 blocked pending a scope decision)

---

## Work Items

| ID | Status | Title | Depends on | Design refs |
| --- | --- | --- | --- | --- |
| IP-LGD-1 | done | Fix kappa slip-ratio denominator in `WheelUnit::step()`: `V_ref = max(\|V_cx\|, \|ω·r\|) + ε` | — | [landing_gear.md §3b](../design/landing_gear.md) |
| IP-LGD-2 | done | Add `spindown_time_s` + `spindown_reference_speed_mps` to `WheelUnitParams` and `LandingGear::initialize()` JSON loading | — | [landing_gear.md §OQ-LG-6](../design/landing_gear.md) |
| IP-LGD-3 | done | Compute bearing-drag coefficients (closed-form Bernoulli solution) in `WheelUnit::initialize()` | IP-LGD-2 | [landing_gear.md §OQ-LG-6](../design/landing_gear.md) |
| IP-LGD-4 | done | Set `substeps` in aircraft config to satisfy the 3× Nyquist bound (OQ-LG-5) | — | [landing_gear.md §OQ-LG-5](../design/landing_gear.md) |
| IP-LGD-5 | done | Replace explicit Euler wheel-spin ODE with Tustin discretization in `WheelUnit::step()` contact branch | IP-LGD-4 | [landing_gear.md §OQ-LG-5, §4a](../design/landing_gear.md) |
| IP-LGD-6 | done | Apply linear+quadratic bearing drag with Tustin integration in `WheelUnit::step()` airborne branch | IP-LGD-3, IP-LGD-5 | [landing_gear.md §OQ-LG-6](../design/landing_gear.md) |
| IP-LGD-7 | superseded | Add rolling-condition clamp (snap `ω` to `ω_roll` on sign change) in `WheelUnit::step()` | — | [landing_gear.md §4b](../design/landing_gear.md) |
| IP-LGD-8 | done | Remove the rolling-condition clamp once the Tustin integrator is verified stable (current state: no clamp) | IP-LGD-5, IP-LGD-6 | [landing_gear.md §OQ-LG-5, §4b](../design/landing_gear.md) |
| IP-LGD-9 | done | Update §4b in `landing_gear.md` for the OQ-LG-11 quasi-static free-roll resolution | — | [landing_gear.md §4b, §OQ-LG-11](../design/landing_gear.md) |
| IP-LGD-10 | done | Remove `applyGearGroundConstraint()` from `KinematicState` and its call site in `Aircraft` (no design authority; remedy is gear damping) | — | [landing_gear.md §Integration Contract, §OQ-LG-10](../design/landing_gear.md) |
| IP-LGD-11 | done | Fix strut damping in the `addLandingGear()` test fixture to realistic ζ (OQ-LG-10) | IP-LGD-10 | [landing_gear.md §OQ-LG-10](../design/landing_gear.md) |
| IP-LGD-12 | superseded | Add `_n_contact_z_filt` gear/body vertical-load suppression filter + `AircraftState` proto field | — | [landing_gear.md §Integration Contract](../design/landing_gear.md) |
| IP-LGD-13 | superseded | Step-5b symmetric lag filter on the gear/body vertical-load fraction in `Aircraft::step()` | IP-LGD-12 | [landing_gear.md §Integration Contract](../design/landing_gear.md) |
| IP-LGD-14 | superseded | Step-11 body-collider `_n_contact_z_filt` refresh to sustain suppression during body contact | IP-LGD-12 | [landing_gear.md §Integration Contract](../design/landing_gear.md) |
| IP-LGD-15 | superseded | JSON + proto round-trip for `_n_contact_z_filt` | IP-LGD-12 | [landing_gear.md §Serialization](../design/landing_gear.md) |
| IP-LGD-16 | superseded | Move contact-nz filter τ to config; add high-pass moment filters (`_nz_moment_filt`, `_ay_moment_filt`, `_roll_rate_moment_filt`) + serialization (OQ-LG-9/13) | IP-LGD-11 | [landing_gear.md §OQ-LG-9, §OQ-LG-13](../design/landing_gear.md) |
| IP-LGD-17 | superseded | Implement moment-to-perturbation paths (n_z / a_y / roll-rate high-pass) in `Aircraft::step()` | IP-LGD-16 | [landing_gear.md §OQ-LG-9](../design/landing_gear.md) |
| IP-LGD-18 | done | Realize the force-channel transfer `G(s)` inline via the control library's `tustin_2_tf`+`tf2ss` (2-state `_force_x`, no free integrator) (OQ-LG-18/20) | IP-LGD-10 | [landing_gear.md §2a, §OQ-LG-18, §OQ-LG-20](../design/landing_gear.md) |
| IP-LGD-19 | done | Destance the gear vertical load (`_fz_stance_filter`) and gate the force-channel input by the C² smootherstep authority fade `Φ(V)` = `phiAuthority` (OQ-LG-19) | IP-LGD-18 | [landing_gear.md §2a, §OQ-LG-19](../design/landing_gear.md) |
| IP-LGD-20 | done | Realize the Δθ moment channel as a 2nd-order low-pass `H₂` on `M/I` (`_dtheta_pitch/roll/yaw_filter`), no free integrator (OQ-LG-16/17) | IP-LGD-10 | [landing_gear.md §2a, §OQ-LG-16, §OQ-LG-17](../design/landing_gear.md) |
| IP-LGD-21 | done | Filtered-velocity attitude reference blended by `Φ(V)`; body rates from the committed attitude (OQ-LG-21) | IP-LGD-19 | [landing_gear.md §2a, §OQ-LG-21](../design/landing_gear.md) |
| IP-LGD-22 | done | Apportionment relaxation `H₁` on the actual gear reaction (`_nz_relax_filter`) inside the `max(0,·)` floor (OQ-LG-22) | IP-LGD-10 | [landing_gear.md §2b-i, §OQ-LG-22](../design/landing_gear.md) |
| IP-LGD-23 | done | Additive axial-acceleration settle/rotation term (`_settle_*`) with ground fade `Φ_g` and clip (OQ-LG-23) | IP-LGD-22 | [landing_gear.md §2b-ii, §OQ-LG-23](../design/landing_gear.md) |
| IP-LGD-24 | done | Non-dimensionalize all §2a/§2b gear knobs as required config ratios × physical scales; `initialize()` throws if any is missing (OQ-LG-17) | IP-LGD-18, IP-LGD-19, IP-LGD-20, IP-LGD-21, IP-LGD-22, IP-LGD-23 | [landing_gear.md §2 Parameterization, §OQ-LG-17](../design/landing_gear.md) |
| IP-LGD-25 | done | JSON + proto round-trip for the §2a/§2b state (`_force_x`, `_fz_stance_filter`, `_dtheta_*_filter`, `_nz_relax_filter`, `_settle_axbar`, attitude filter, CL-recovery flags) | IP-LGD-18, IP-LGD-19, IP-LGD-20, IP-LGD-21, IP-LGD-22, IP-LGD-23 | [landing_gear.md §Serialization](../design/landing_gear.md), [aircraft.md §Serialization](../design/aircraft.md) |
| IP-LGD-26 | done | Add required non-dimensional `aero_authority_v_lower_ratio` and `aero_authority_v_upper_ratio` (airspeed-ratio band edges) to every aircraft config (`configs/*.json`, `test/data/aircraft/*.json`), `makeConfig()`, the binding-test fixtures, the validator, and the config schema doc; enforce `0 < lower < upper ≤ 1` in `Aircraft::initialize()` (throws) and `validate_aircraft_config.py`, with C++ and pytest coverage | — | [landing_gear.md §2b (b-iii)](../design/landing_gear.md), [aircraft_config_v1.md](../schemas/aircraft_config_v1.md) |
| IP-LGD-27 | done | In `Aircraft`: read the two band ratios in `initialize()` (derive `_wa_q_lo_pa`, `_wa_q_hi_pa` from `q_stall`); in `step()` §2b apply `w_a(q) = smootherstepEdges(q, q_lo, q_hi)` to the gear-relative demand inside the existing `max(0,·)` floor | IP-LGD-26, IP-LGD-22 | [landing_gear.md §2b (b-iii)](../design/landing_gear.md) |
| IP-LGD-28 | done | Scenario acceptance tests: C++ `LandingGear_LowSpeedRollout_Converges_OQ_LG24` (sub-stall roll-out converges, α bounded, no launch) and pytest `test_full_stop_landing_converges_no_lowspeed_divergence_oq_lg24` (flown landing through the transition band) | IP-LGD-27 | [landing_gear.md §2b (b-iii)](../design/landing_gear.md), [landing_gear.md §Test Strategy](../design/landing_gear.md#test-strategy) |
| IP-LGD-29 | done | **Retain** `ContactDiag`/`lastContactDiag()` as a permanent model-diagnosis interface (de-labeled from "TEMPORARY"); the per-wheel force breakdown is not recoverable from the aggregate `step()` force and is used by the tire-force regression tests. No removal — the proposed deletion is cancelled. | — | [landing_gear.md §OQ-LG-15 resolution](../design/landing_gear.md) |

> **IP-LGD-29 resolution:** The original item (remove the "TEMPORARY" OQ-LG-15 diagnostic) was a stale
> directive. The `ContactDiag` accessor exposes the per-wheel `F_x`/`F_y`/`F_rr`/slip breakdown that is
> not recoverable from the public aggregate force vector, a permanent regression
> (`LandingGear_TireNeverPropels_FullScenario`) and the tire data-emitter depend on it, and it remains
> useful for diagnosing model behavior. Decision: **keep it** as a permanent diagnostic interface (code
> comments de-labeled); the OQ-LG-15 diagnostic test and its CSV are likewise retained as diagnosis
> tooling. Nothing is removed.

---

## Notes

**Consolidation provenance.** This plan supersedes and absorbs the former
`aircraft_gear_ground_forces.md` (its IP-AGF-1/2/3 → IP-LGD-9/10/11 `done`; IP-AGF-4/5 → IP-LGD-16/17
`superseded`) and `landing_gear_debt_closure.md` (its OQ-LG-24 + cleanup items → IP-LGD-26..29). Both
files are removed; this is the only landing-gear implementation plan. Items were renumbered into one
topologically-ordered sequence.

**Why the `superseded` items are kept.** IP-LGD-7 (rolling-condition clamp) was removed by IP-LGD-8.
IP-LGD-12..15 (the `_n_contact_z_filt` vertical-load suppression) and IP-LGD-16..17 (the
moment-perturbation high-pass paths, OQ-LG-9/13) were the *initial* coupling approach; they were
replaced by the Phase C gear force-&-moment integration (OQ-LG-15/16/18/19/20/21/22/23), and their
members (`_n_contact_z_filt`, `_nz_moment_filt`, `_ay_moment_filt`, `_roll_rate_moment_filt`,
`_wow0_elapsed_s`) are no longer in `Aircraft`. They are retained as `superseded` for traceability,
not as pending work. The root-cause analysis that drove the supersession is the standalone report
[`oq_lg15_gear_attitude_feedback_investigation.md`](../defects/oq_lg15_gear_attitude_feedback_investigation.md).

**Phase C recorded retroactively.** IP-LGD-18..25 implement the current coupling design and were built
before this consolidated plan existed; they are marked `done` and are verifiable in `Aircraft.cpp` /
`Aircraft.hpp` (`_force_*`, `_fz_stance_filter`, `_dtheta_*_filter`, `_nz_relax_filter`, `_settle_*`,
`phiAuthority`) and the resolved OQ-LG-16/17/18/19/20/21/22/23 sections of `landing_gear.md`.

**OQ-LG-24 (Phase D) scope.** Only the effectiveness weight is in scope (IP-LGD-26/27); the existing
`max(0,·)` apportionment floor and all other §2b operators are left unchanged. Smoothing those
operators is the **undecided** SD-1 reconciliation tracked by
[OQ-LG-25](../design/landing_gear.md#oq-lg-25--whether-and-how-to-reconcile-the-flight-physics-smoothness-audit-findings)
and is intentionally **not** in this plan.

**IP-LGD-28 is a standalone scenario item** per the TDD style rule (unit tests belong to each item; a
scenario-level acceptance gate is its own item).
