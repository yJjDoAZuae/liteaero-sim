# Body Collider Dynamics — Implementation Plan

**Scope:** Implements the four decided body-collider improvements in
[body_collider.md §5](../design/body_collider.md): §5a inelastic velocity-arrest normal contact,
§5b restitution-consistent hard constraint, §5c dedicated gear-style rotational-reaction $\Delta\theta$,
and §5d tangential scrape friction (Coulomb + viscous). §5a/§5b/§5d are **implemented and tested**
(OQ-BC-1/2/4/5 resolved). §5c (the dedicated rotational channel) is **fully designed and unblocked**
(OQ-BC-3/6/7 resolved): separate, attributed gear/collider moment **and** force channels via a parallel
filter set in `Aircraft`, behavior-preserving by the $H_2$/`LP`/$G(s)$ linearity invariant (the
equivalence test is the guard). OQ-BC-5 fixed the §5a parameterization: the only
user-facing contact knob is the non-dimensional coefficient of restitution `restitution_nd` ($e$,
§5b); the §5a arrest damping is derived internally ($b_\text{total}=m/(N_\text{arr}\,dt)$, fixed
$N_\text{arr}\approx3$) from airframe properties that `Aircraft` now supplies to
`BodyCollider::initialize`, and `CollisionVolumeParams` reduces to geometry.

**Design authority:** [body_collider.md](../design/body_collider.md) (§5a–§5d, Integration,
Serialization), [landing_gear.md §2a, §7](../design/landing_gear.md) (the $\Delta\theta$ pattern and
its stability), [aircraft.md](../design/aircraft.md) (step 5a / step 12 call sites).

**Last updated:** 2026-06-26

---

## Work Items

| ID | Status | Title | Depends on | Design refs |
| --- | --- | --- | --- | --- |
| IP-BC-1 | done | Add optional `restitution_nd` (default 0) to `BodyColliderParams` JSON + proto + `parse`/`serialize`; existing configs unaffected by the default | — | [body_collider.md §5b](../design/body_collider.md), Serialization |
| IP-BC-2 | done | Generalize `KinematicState::applyTerrainHardConstraint(pen, restitution_nd)` to remove $(1+e)\,v_z$ instead of hard-zeroing; pass the configured restitution from `Aircraft::step` step 12 | IP-BC-1 | [body_collider.md §5b, §3](../design/body_collider.md) |
| IP-BC-3 | done | Reduce `CollisionVolumeParams` to geometry — remove `stiffness_npm`/`damping_nspm` (struct + proto + parse/serialize) — and update the C++ test fixtures. (JSON config dead-key strip pending — see Notes.) | — | [body_collider.md §5a, OQ-BC-5](../design/body_collider.md) |
| IP-BC-4 | done | Add airframe mass + outer `dt` to `BodyCollider::initialize`; derive the per-corner arrest damping `_b_corner_nspm = m / (n_corners_total * N_arr * dt)` (fixed internal `kArrestSteps = 3`), serialized so deserialize reproduces behavior; `Aircraft` passes `_inertia.mass_kg`, `_outer_dt_s` | — | [body_collider.md §5a, OQ-BC-5](../design/body_collider.md) |
| IP-BC-5 | done | Replace the Kelvin–Voigt penalty in `BodyCollider::step` with the velocity-arrest force $F=\max(0,\,c\,\delta\dot\delta)$, $c=$ `_b_corner_nspm`$/h_z$ (no spring); BodyCollider tests migrated to the velocity-arrest semantics (no static force, scales with $\delta\dot\delta$, no suction, mass-scaling) | IP-BC-2, IP-BC-3, IP-BC-4 | [body_collider.md §5a](../design/body_collider.md) |
| IP-BC-6 | done | Add collider-level `friction_coulomb_nd` ($\mu$) and `friction_viscous_nd` ($k_\text{visc}$) to `BodyColliderParams` JSON + proto + parse/serialize (default 0 → frictionless). Viscous coefficient derived as $k_\text{visc}\cdot$`b_corner` (mass-scaled, OQ-BC-5) | — | [body_collider.md §5d](../design/body_collider.md) |
| IP-BC-7 | done | Apply the tangential force $-\mu F_\text{pen}\hat{\mathbf v}_{t,\text{reg}} - k_\text{visc} b_\text{corner}\mathbf v_t$ at penetrating corners in `BodyCollider::step`, Coulomb direction regularized by `kSlipRegMps` | IP-BC-5, IP-BC-6 | [body_collider.md §5d](../design/body_collider.md) |
| IP-BC-8 | done | Added the parallel body-collider rotation-filter set in `Aircraft` (OQ-BC-6 → Alt 2): `_bc_dtheta_{pitch,roll,yaw}_filter` + collider force-channel `_bc_force_x`/`_bc_fz_stance_filter`, configured identically to the gear set, reset-cleared, and serialized (JSON + 10 new proto fields) | — | [body_collider.md §5c, OQ-BC-6](../design/body_collider.md), [landing_gear.md §2a](../design/landing_gear.md) |
| IP-BC-9 | done | Separated gear/collider force+moment at step 5a; the §5c Δθ block now sums per-source channels (force via a `forceChannel` lambda, moment per axis), behavior-preserving by linearity. Regression net: all existing gear-only and bc-only tests pass unchanged; `BodyColliderImpact_RotationState_RoundTrips` verifies the new collider rotation state round-trips through JSON + proto | IP-BC-8 | [body_collider.md §5c](../design/body_collider.md), [aircraft.md](../design/aircraft.md) |

---

## Notes

**Sequencing.** §5a/§5b/§5d are done. §5c is unblocked (OQ-BC-3/6/7 resolved): IP-BC-8 (parallel
`_bc_dtheta_*` filters + collider force-channel state in `Aircraft`) then IP-BC-9 (separate the gear and
collider channels + the equivalence/golden-trajectory regression). Earlier order was: §5b (IP-BC-1 →
IP-BC-2) and the §5a foundation (IP-BC-3 geometry-only struct, IP-BC-4 `initialize` plumbing + derived damping), then
IP-BC-5 (the velocity-arrest force, which the inelastic drop test verifies together with the §5b
constraint). §5d (IP-BC-6 → IP-BC-7) and §5c (IP-BC-8 → IP-BC-9) follow once the §5a force/moment
exists.

**§5b is nearly behavior-preserving at $e=0$.** The current `applyTerrainHardConstraint` already zeros
the downward velocity component, which is exactly $(1+e)v_z$ at $e=0$; IP-BC-2 generalizes it to a
configured restitution and makes the constraint the primary, parameterized inelastic mechanism per the
resolved OQ-BC-2.

**Cross-cutting plumbing.** Both IP-BC-4 (mass + `dt`) and IP-BC-8 (inertia tensor) require
`Aircraft` to pass airframe properties into `BodyCollider::initialize`, which is presently
config-only. Coordinate the signature change so it is made once.

**TDD / build.** Every item writes a failing test first. Build and test with the project toolchain:
`PATH="/c/msys64/ucrt64/bin:$PATH" mingw32-make -C build liteaerosim_test liteaero_sim_py` then
`ctest --test-dir build -R "BodyCollider|Aircraft|KinematicState"`. (Rebuild `liteaero_sim_py`
whenever a header/source the binding depends on changes, or the staleness guard fails the CrossLang
tests.)

**Remaining after §5a/§5b code (not blocking, follow-ups):**

- **JSON config dead-key strip — done.** `stiffness_npm`/`damping_nspm` removed from all 9
  `configs/*.json` body-collider volumes (validated). `test/data/aircraft/*.json` and
  `python/assets/` carry no such keys; `gen_test_assets.py` writes no body-collider section.
- **Pre-existing gap discovered (not §5c).** `Aircraft::_body_in_hard_contact` (the body-contact
  weight-on-wheels latch, set in `Aircraft::step`) is **not serialized** (JSON or proto), so a
  belly-settled aircraft loses the latch on round-trip and re-saturates α on the next step. Surfaced by
  the §5c serialization test (which was refocused to test only the collider rotation state). A 2-line
  fix (serialize the bool in JSON + a proto bool field); separate task, needs explicit instruction.
- **Design-doc as-built reconciliation — done.** [body_collider.md](../design/body_collider.md) updated:
  §2 now describes the velocity-arrest contact (was Kelvin–Voigt), §3 the restitution-consistent
  constraint, the Class Hierarchy / Interface / Serialization / proto blocks match the current code, the
  §5 section and OQ resolutions are relabeled "implemented", and the Test Strategy cites the actual tests.

**Pre-existing unrelated failures:** `LandingGear.BearingDragCoeffs_NonzeroWhenSpindownConfigured` and
`LandingGear.DifferentialBrake_ProducesYawMoment` fail independently of this plan (brake/spindown logic).
