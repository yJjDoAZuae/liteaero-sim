# Landing Gear Dynamics — Implementation Plan

**Scope:** This plan covers all changes to the landing gear wheel dynamics model derived from
the design decisions in [`landing_gear.md`](../design/landing_gear.md) and the associated
[`aircraft.md`](../design/aircraft.md) integration. Specifically it tracks: (1) the kappa
slip ratio fix and rolling-condition clamp (§3b, §4b — implemented); (2) the gear/body-collider
vertical load filter `_n_contact_z_filt` in `Aircraft` (implemented; design not yet documented
in `aircraft.md` — see Notes); (3) airborne bearing drag parametrized by spindown time (OQ-LG-6 —
resolved, pending implementation); (4) Tustin discretization of the wheel spin ODE with substep
count set by the 3× Nyquist rule (OQ-LG-5 — resolved, pending implementation). OQ-LG-7 is
resolved with no code change required. OQ-LG-1, OQ-LG-2, OQ-LG-3, and OQ-LG-4 remain open
and do not block any work items in this plan.

**Design authority:** [`landing_gear.md`](../design/landing_gear.md),
[`aircraft.md`](../design/aircraft.md)

**Last updated:** 2026-05-20

---

## Work Items

| ID | Status | Title | Depends on | Design refs |
| --- | --- | --- | --- | --- |
| IP-LGD-1 | done | Fix kappa slip ratio denominator in `WheelUnit::step()`: use `V_ref = max(\|V_cx\|, \|ω_w·r_w\|) + ε` | — | [landing_gear.md §3b](../design/landing_gear.md) |
| IP-LGD-2 | done | Add rolling-condition clamp in `WheelUnit::step()`: snap `ω_w` to `ω_roll` when sign of `(ω_w − ω_roll)` changes mid-substep | — | [landing_gear.md §4b](../design/landing_gear.md) |
| IP-LGD-3 | done | Add `_n_contact_z_filt` member to `Aircraft` and `n_contact_z_filt` field to `AircraftState` proto message | — | [aircraft.md §UC-1](../design/aircraft.md) |
| IP-LGD-4 | done | Implement step 5b symmetric lag filter (τ = 0.10 s) on gear/body-collider vertical load fraction in `Aircraft::step()` | IP-LGD-3 | [aircraft.md §UC-1](../design/aircraft.md) |
| IP-LGD-5 | done | Implement step 11 body-collider `_n_contact_z_filt` refresh in `Aircraft::step()` to sustain aero suppression during body contact | IP-LGD-3 | [aircraft.md §UC-1](../design/aircraft.md) |
| IP-LGD-6 | done | Add JSON + proto round-trip serialization for `_n_contact_z_filt` in `Aircraft::serializeJson()`, `deserializeJson()`, `serializeProto()`, `deserializeProto()` | IP-LGD-3 | [aircraft.md §UC-1](../design/aircraft.md) |
| IP-LGD-7 | todo | Add `spindown_time_s` and `spindown_reference_speed_mps` fields to `WheelUnitParams` struct and to `WheelUnitParams` JSON loading in `LandingGear::initialize()` | — | [landing_gear.md §OQ-LG-6 resolution](../design/landing_gear.md) |
| IP-LGD-8 | todo | Compute `c1` and `c2` bearing drag coefficients in `WheelUnit::initialize()` from `spindown_time_s` and `spindown_reference_speed_mps` per the closed-form Bernoulli solution | IP-LGD-7 | [landing_gear.md §OQ-LG-6 resolution](../design/landing_gear.md) |
| IP-LGD-9 | todo | Set `substeps` in the aircraft JSON config file(s) to satisfy the 3× Nyquist substep count bound from the OQ-LG-5 formula for the specific aircraft geometry | — | [landing_gear.md §OQ-LG-5 resolution](../design/landing_gear.md) |
| IP-LGD-10 | todo | Replace explicit Euler wheel ODE with Tustin discretization in `WheelUnit::step()` contact branch; update computational resource table in landing_gear.md | IP-LGD-9 | [landing_gear.md §OQ-LG-5 resolution, §4a](../design/landing_gear.md) |
| IP-LGD-11 | todo | Apply linear + quadratic bearing drag with Tustin integration in `WheelUnit::step()` airborne branch | IP-LGD-8, IP-LGD-10 | [landing_gear.md §OQ-LG-6 resolution](../design/landing_gear.md) |
| IP-LGD-12 | todo | Remove rolling-condition clamp from `WheelUnit::step()` after Tustin integrator is verified stable; update §4a and §4b in landing_gear.md to reflect the new implementation | IP-LGD-10, IP-LGD-11 | [landing_gear.md §OQ-LG-5 resolution, §4b](../design/landing_gear.md) |

---

## Notes

**n_contact_z_filt design gap (IP-LGD-3 through IP-LGD-6):** The `_n_contact_z_filt` gear/body-collider
vertical load filter is implemented in `Aircraft` but its design (what problem it solves, the choice
of τ = 0.10 s, and the step 11 body-collider refresh) is not documented in `aircraft.md`. The design
refs for IP-LGD-3 through IP-LGD-6 cite `aircraft.md §UC-1` as the closest applicable section, but
that section does not yet describe the filter. A documentation work item should be raised in the
roadmap to add a design section to `aircraft.md` before these items are audited.

**spindown_time_s and spindown_reference_speed_mps are config, not state (IP-LGD-7):**
These two new fields belong in `WheelUnitParams` alongside the existing geometry and tyre
parameters. Per the serialization contract in `landing_gear.md §Serialization`, configuration
parameters are not serialized — they are reloaded from the JSON config on `initialize()`. No
proto field additions are needed for the spindown parameters.

**Tustin airborne branch depends on Tustin contact branch (IP-LGD-11):** The bearing drag
ODE is non-stiff and does not require the substep count bound from OQ-LG-5, but it uses the
same Tustin discretization pattern. IP-LGD-11 is ordered after IP-LGD-10 so the Tustin method
is established in the contact branch before being applied to the airborne branch, keeping the two
branches consistent and the Tustin helper tested before it is reused.

**Rolling-condition clamp removal (IP-LGD-12):** The clamp must remain in place until both
the Tustin contact ODE (IP-LGD-10) and the airborne bearing drag (IP-LGD-11) are in place.
IP-LGD-11 is a prerequisite because, once the clamp is removed, the correctness of
first-contact behavior depends on the wheel arriving at contact with a physically realistic
airborne spin-down history; without IP-LGD-11 the wheel retains its pre-liftoff speed
indefinitely, which would produce a large initial slip at contact that the Tustin integrator
must resolve without the clamp's protection.

**OQ-LG-7 — no work items:** OQ-LG-7 (first-contact wheel speed initial condition) is
resolved with the decision that no special handling is required; the Tustin integrator at
the correct substep count handles spin-up naturally. No additional work items arise from
this resolution beyond those already listed for IP-LGD-10 and IP-LGD-11.
