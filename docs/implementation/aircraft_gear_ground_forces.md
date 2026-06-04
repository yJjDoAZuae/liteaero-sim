# Aircraft Gear Ground Forces — Implementation Plan

**Scope:** This plan covers the corrections needed to make the landing gear ground force
integration in `Aircraft` physically correct and the associated test and documentation
fixes. Specifically: (1) revert the physically unjustified `applyGearGroundConstraint`
velocity clamp added to `KinematicState` (no design authority); (2) fix unrealistically
low strut damping parameters in the `addLandingGear` test fixture, which cause the
`LandingGear_FullStop_SpeedNearZero` scenario test to fail; (3) update §4b of
`landing_gear.md` to accurately describe the quasi-static free-roll clamp per the
OQ-LG-11 resolution; (4) implement the OQ-LG-9 moment-to-perturbation paths in
`Aircraft::step()` once the three blocking open questions (OQ-LG-12, OQ-LG-13, OQ-LG-14)
are resolved.

OQ-LG-10, OQ-LG-11 are resolved and do not block any items. OQ-LG-12, OQ-LG-13, and
OQ-LG-14 are open and block IP-AGF-4 and IP-AGF-5. OQ-LG-1, OQ-LG-2, OQ-LG-3, OQ-LG-4,
and OQ-LG-8 remain open and do not block any item in this plan.

**Design authority:** [`landing_gear.md`](../design/landing_gear.md),
[`aircraft.md`](../design/aircraft.md)

**Last updated:** 2026-05-26 (IP-AGF-2 through IP-AGF-4 complete; IP-AGF-5 partial — see implementation notes)

---

## Work Items

| ID | Status | Title | Depends on | Design refs |
| --- | --- | --- | --- | --- |
| IP-AGF-1 | done | Update §4b in `landing_gear.md`: replace open-question text with OQ-LG-11 resolution; update canonical events table "Free-roll convergence" row | — | [landing_gear.md §4b, §OQ-LG-11](../design/landing_gear.md) |
| IP-AGF-2 | done | Remove `applyGearGroundConstraint()` from `KinematicState.hpp` (declaration), `KinematicState.cpp` (implementation), and `Aircraft.cpp` (call + `gear_in_contact` variable) | — | [landing_gear.md §Integration Contract](../design/landing_gear.md) |
| IP-AGF-3 | done | Fix strut damping in `addLandingGear()` test fixture: set `damping_compression_nspm` = 2600 and `damping_extension_nspm` = 520 for all three wheel units (ζ_c ≈ 0.40, ζ_e ≈ 0.08, 5:1 ratio) | IP-AGF-2 | [landing_gear.md §OQ-LG-10 resolution](../design/landing_gear.md) |
| IP-AGF-4 | done | Move `kContactFiltTau_s` to named config parameter `contact_nz_filter_tau_s`; add three `FilterSS2Clip` high-pass filter members (`_nz_moment_filt`, `_ay_moment_filt`, `_roll_rate_moment_filt`) to `Aircraft`; wire ωn/ζ from per-axis FBW config fields; add JSON and proto serialization | IP-AGF-3 | [landing_gear.md §OQ-LG-9, §OQ-LG-13](../design/landing_gear.md) |
| IP-AGF-5 | partial | Implement moment-to-perturbation paths in `Aircraft::step()`: compute $\mathbf{M}^W = R_{WN}R_{NB}\mathbf{M}^B$, derive $\Delta\Omega_x$ / $n_{z,\text{moment}}$ / $\Delta a_y$, pass each through its second-order HP filter, inject into roll-rate, n_z, and ay channels | IP-AGF-4 | [landing_gear.md §OQ-LG-9 resolution](../design/landing_gear.md) |

**IP-AGF-3 ordering note:** IP-AGF-3 depends on IP-AGF-2 so that the corrected gear
physics are in place before the test damping fix is validated. The test was written
against clean physics; validating it while `applyGearGroundConstraint` is present would
give a misleading result.

**IP-AGF-4 blocked reason:** OQ-LG-13 must be resolved to determine $\tau_{hp}$ for
the high-pass moment filter and the config parameter name under which it is stored.
Both decisions affect what members are added to `Aircraft`, what JSON keys are introduced,
and what proto fields are required. OQ-LG-12 is now resolved (wind-y → n_z, wind-z →
$\Delta a_y$). OQ-LG-14 is resolved (no velocity floor needed).

**IP-AGF-5 blocked reason:** Depends on IP-AGF-4 for the filter state and config
parameter; additionally blocked by OQ-LG-13 ($\tau_{hp}$ value).

---

## Notes

**`applyGearGroundConstraint` removed (IP-AGF-2 — done).** The function was removed from
`KinematicState` and the call site in `Aircraft::step()`. OQ-LG-10 concluded that the
$F_z\sin\gamma$ coupling is physically correct; the correct remedy is adequate gear damping,
not a velocity clamp.

**Test fixture aircraft model.** The `addLandingGear` fixture is built on `makeConfig()`,
a synthetic C172-class aircraft: mass = 1045 kg, S_ref = 16.2 m², AR = 7.47, CL_max = 1.80,
V_ne = 82.3 m/s. These parameters match published Cessna 172S data closely (the wing area
is exact). The fixture is not formally declared as a C172; it is a synthetic model calibrated
to GA light-single data.

**Damping target derivation (IP-AGF-3 — done).** For the `addLandingGear` fixture: mass
= 1045 kg, k = 20000 N/m per wheel (3 wheels → k_total = 60000 N/m). ζ_e = 520 /
(2 × sqrt(20000 × 1045/3)) = 520 / 5281 ≈ 0.099. This is the per-wheel damping ratio.
See OQ-LG-15 for the observed consequence.

**Actual test outcome (IP-AGF-2 + IP-AGF-3).** The `LandingGear_FullStop_SpeedNearZero`
test still fails at ~5 m/s after 90 s — a slowly-decaying limit cycle rather than a stop.
The root cause is now confirmed (gear–attitude feedback artifact) and the fix designed; see
the note below.

**OQ-LG-15 root cause confirmed; fix designed (as of 2026-06).** The diagnostic test
`LandingGear_FullStop_OQ_LG15_Diagnostic` plus `WheelUnit::step()` force-breakdown
instrumentation identified the complete chain: a zero-inertia, velocity-slaved attitude
sweeps the long-lever-arm nose wheel at ~9 m/s → deep one-step penetration (34 kN normal
force) → near-peak forward slip force (+24 kN F_x) → 17.3 Hz bounce + periodic +22.7 kN
forward spike. The **full defect investigation** (defect chain, instrumented force breakdown,
quantified data, 9 figures) is the standalone report
[`oq_lg15_gear_attitude_feedback_investigation.md`](../defects/oq_lg15_gear_attitude_feedback_investigation.md).
The **fix** (gear-F&M integration: a gear-load-driven body rotation-deviation state feeding
α/CL and the gear geometry, plus a lagged n_z-command relaxation) is design content in
[`landing_gear.md` — Integration Contract — `Aircraft` §2](../design/landing_gear.md);
OQ-LG-15 is resolved, and filter parameterization is resolved (OQ-LG-17: rotation-deviation
filter from the inertia tensor, n_z-relaxation filter from FBW ωₙ/ζ, independently). Design
complete; not yet implemented, no code change made.

*Diagnostic instrumentation still in tree (TEMPORARY, remove when OQ-LG-15 closes):*
`WheelUnit::ContactDiag` struct + `_diag` member + `lastContactDiag()` accessor in
`WheelUnit.hpp/.cpp`, and the `nose_*` columns in the diagnostic test CSV.

**IP-AGF-4 changes delivered.** `_contact_nz_filter_tau_s` moved from a compile-time
constant to a JSON config field. The three HP moment filters (`_nz_moment_filt`,
`_ay_moment_filt`, `_roll_rate_moment_filt`) were added to `Aircraft`. JSON and proto
serialization were extended for all new state. A `_wow0_elapsed_s` member was also added
as part of the n_z suppression redesign (see design deviation note below).

**IP-AGF-5 partial (nz_moment path disabled).** The ay and roll-rate moment perturbation
paths are active in `Aircraft::step()`. The nz_moment subtraction from n_z_shaped is
intentionally commented out: the contact-nz suppression filter drives `n_z_shaped → 0`
during ground contact, so the `max(0, n_z_shaped − n_z_moment_filt_val)` subtraction is
annihilated by the floor exactly when gear contact (the only moment source) is active. The
comment reads `// n_z_shaped = std::max(0.f, n_z_shaped - n_z_moment_filt_val);`. No test
covers the nz_moment path; IP-AGF-5 is therefore not complete. The design question — how the
gear pitch moment should reach the load-factor model — is now **resolved**
([OQ-LG-16](../design/landing_gear.md)): it is subsumed by the OQ-LG-15 **gear-F&M
integration**, where the gear pitch moment is one input to the body rotation-deviation
$\Delta\theta$ (a stable 2nd-order low-pass, finite DC, *not* a rate into `q_nw`) → α → CL →
realized Nz. The disabled n_z-suppression-channel scheme is abandoned; the dead `n_z_moment`
code should be removed/replaced when the OQ-LG-15 fix is implemented (the pitch path is part of
that work, not re-enabled separately).

**Design deviation — n_z suppression filter redesigned without consultation.**
The design document (`landing_gear.md` §Integration Contract §2) specifies a simple
first-order lag (τ = 0.10 s, now the config parameter `contact_nz_filter_tau_s`). During
implementation, the simple lag was found insufficient: brief WoW=0 bounce episodes
(≈0.06 s) allowed nzfilt to decay from 1.0 to 0.942, causing the LFA to target
alpha ≈ 16° at V ≈ 6 m/s (sub-stall), injecting aerodynamic energy. The filter was
redesigned to a two-speed hold-time scheme:

- **WoW=1 (or _body_in_hard_contact):** nzfilt = 1.0 instantly (or driven by raw = 2 − n_z_shaped).
- **WoW=0, elapsed < τ_hold:** hold nzfilt at its current value (no decay). τ_hold = 10 × τ_engage = 1.0 s.
- **WoW=0, elapsed ≥ τ_hold:** slow exponential decay at τ_decay = τ_hold = 1.0 s.

Two new members were added to `Aircraft` without design-document authority: `_wow0_elapsed_s`
(elapsed seconds since WoW last went to zero) and the `_body_in_hard_contact` handling path
in the filter block. JSON field `wow0_elapsed_s` and proto field 33 (`wow0_elapsed_s` in
`AircraftState`) were added accordingly. The `landing_gear.md` §Integration Contract §2 has
been updated to reflect this algorithm.

**Debug printf in production code.** `Aircraft::step()` contains a diagnostic print block
(guarded by `_has_landing_gear && (time_sec < 5.0 || time_sec > 87.0)`) and the
`LandingGear_FullStop_SpeedNearZero` test prints to stdout every 25 steps. Both must be
removed once the FullStop test resolution is agreed and OQ-LG-15 is closed.

**OQ-LG-12, OQ-LG-13, OQ-LG-14 resolved.** These are all marked resolved in
`landing_gear.md`. OQ-LG-13 resolution determined that the HP filters reuse existing FBW
ωn/ζ parameters (no new config field needed beyond `contact_nz_filter_tau_s`).
