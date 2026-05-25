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

**Last updated:** 2026-05-25 (OQ-LG-12, OQ-LG-13, OQ-LG-14 resolved; IP-AGF-4 and IP-AGF-5 unblocked)

---

## Work Items

| ID | Status | Title | Depends on | Design refs |
| --- | --- | --- | --- | --- |
| IP-AGF-1 | done | Update §4b in `landing_gear.md`: replace open-question text with OQ-LG-11 resolution; update canonical events table "Free-roll convergence" row | — | [landing_gear.md §4b, §OQ-LG-11](../design/landing_gear.md) |
| IP-AGF-2 | todo | Remove `applyGearGroundConstraint()` from `KinematicState.hpp` (declaration), `KinematicState.cpp` (implementation), and `Aircraft.cpp` (call + `gear_in_contact` variable) | — | [landing_gear.md §Integration Contract](../design/landing_gear.md) |
| IP-AGF-3 | todo | Fix strut damping in `addLandingGear()` test fixture: set `damping_compression_nspm` = 2600 and `damping_extension_nspm` = 520 for all three wheel units (ζ_c ≈ 0.40, ζ_e ≈ 0.08, 5:1 ratio) | IP-AGF-2 | [landing_gear.md §OQ-LG-10 resolution](../design/landing_gear.md) |
| IP-AGF-4 | todo | Move `kContactFiltTau_s` to named config parameter `contact_nz_filter_tau_s`; add three `FilterSS2Clip` high-pass filter members (`_nz_moment_filt`, `_ay_moment_filt`, `_roll_rate_moment_filt`) to `Aircraft`; wire ωn/ζ from per-axis FBW config fields; add JSON and proto serialization | IP-AGF-3 | [landing_gear.md §OQ-LG-9, §OQ-LG-13](../design/landing_gear.md) |
| IP-AGF-5 | todo | Implement moment-to-perturbation paths in `Aircraft::step()`: compute $\mathbf{M}^W = R_{WN}R_{NB}\mathbf{M}^B$, derive $\Delta\Omega_x$ / $n_{z,\text{moment}}$ / $\Delta a_y$, pass each through its second-order HP filter, inject into roll-rate, n_z, and ay channels | IP-AGF-4 | [landing_gear.md §OQ-LG-9 resolution](../design/landing_gear.md) |

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

**`applyGearGroundConstraint` has no design authority (IP-AGF-2).** The function was
added in a prior session to suppress the $F_z\sin\gamma$ coupling during gear bounce.
OQ-LG-10 concluded that the coupling is physically correct and the correct remedy is
adequate gear damping, not a velocity clamp. The `gear_in_contact` local variable in
`Aircraft::step()` has no remaining use after the call is removed and must be deleted
together with it.

**Damping target derivation (IP-AGF-3).** For the `addLandingGear` fixture: mass = 1045
kg (from `makeConfig()`), k = 20000 N/m (unchanged). Formula: b = ζ × √(2km).
Target ζ_c = 0.40 → b_c = 0.40 × √(2 × 20000 × 1045) = 0.40 × 6465 ≈ 2586, rounded
to 2600. Target ζ_e = b_c / 5 / 6465 ≈ 0.080 → b_e = 2600 / 5 = 520. All three wheel
units in the fixture use identical parameters and must all be updated.

**Expected test outcome after IP-AGF-2 + IP-AGF-3.** Rolling resistance force:
F_rr = 0.02 × m × g ≈ 205 N. Deceleration: 205 / 1045 ≈ 0.196 m/s². Time from 15 m/s
to 0.5 m/s: ≈ 74 s — within the 90 s test window.

**OQ-LG-12 axis-swap concern.** The OQ-LG-9 resolution text assigns wind-z → n_z and
wind-y → ay. In the `Aircraft` wind frame (x forward, y lateral, z lift), a moment about
y (lateral) is pitch and drives n_z; a moment about z (lift) is yaw and drives ay. The
OQ-LG-9 y/z assignment may be swapped. OQ-LG-12 must be resolved by reading
`Aircraft::step()` before IP-AGF-4 is begun.

**Proto field for `_n_z_moment_filt` (IP-AGF-4).** Follows the same pattern as
`_n_contact_z_filt` (added in IP-LGD-3 through IP-LGD-6 in
[`landing_gear_dynamics.md`](landing_gear_dynamics.md)). The field is filter state (not
config), so it belongs in `AircraftState` and is serialized in both JSON and proto round-trips.
