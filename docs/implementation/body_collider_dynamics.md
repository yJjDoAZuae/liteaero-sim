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
| IP-BC-10 | done | **Bug fix (Aircraft-level serialization cluster):** fixed `_has_landing_gear`/`_has_body_collider` not restored (JSON + added proto subsystem fields 71/72), the **w_a band edges** `_wa_q_lo_pa`/`_wa_q_hi_pa` not serialized (the actual α-saturation cause), `_outer_dt_s` (reconstructed), and `_body_in_hard_contact`. Regression: `BodyColliderRoundTrip_PreservesSubsystem_ContinuedStepMatches` (deserialize-then-step matches the original through JSON + proto) | — | [body_collider.md Serialization](../design/body_collider.md) |
| IP-BC-11 | done | Made the **reporting** WoW a hysteretic geometric latch on `minCornerClearance_m`: engage at clearance `≤ 0` (on contact), release at clearance `> δ_off`, derived band `δ_off = _stall_speed_mps · _outer_dt_s` (replacing the magic 0.05 m); reporting-only after IP-BC-12's decoupling (committed 73a9468). (`V_stall` reference provisional — revisit vs post-impact speed) | IP-BC-12 | [body_collider.md OQ-BC-8](../design/body_collider.md) |

| IP-BC-12 | done | Resolve the envelope-wide post-impact limit cycle (OQ-BC-9 → Alt 1): split the **gear-only** contact reaction from the combined reaction and gate the step-5b n_z apportionment (`nz_relax`) and the OQ-LG-23 settle term on **gear** WoW alone, so body-collider contact no longer feeds the FBW lift-shaping loop (committed 73a9468). Guard: the four-case impact envelope suite + gear regressions. Shallow now passes; **steep/inverted retain a residual** (the lift-into-ground fight + roll-energy injection) addressed by IP-BC-13 + the lift-path decision | — | [body_collider.md OQ-BC-9](../design/body_collider.md) |

| IP-BC-13 | superseded (OQ-BC-12) | Inelastic §5c rotational velocity-arrest (OQ-BC-10 → Alt 1). **Prototyped and shown ineffective** for the reported gear-landing catapult: it cut the launch but left the 177° roll-over, because the driver is the §5a CM-arresting force + the missing rotational DOF, not the §5c moment channel (OQ-BC-12). Test scaffold retained (roll metrics + `WingtipStrike` + wide-wing config + the `GearLanding_WingtipGraze` reproduction). | — | [body_collider.md OQ-BC-10](../design/body_collider.md) |
| IP-BC-14 | superseded (OQ-BC-12) | Scoped aero-lift suppression for the steep/inverted pitch limit cycle. Subsumed: the lift-fight is a facet of the missing rotational DOF (OQ-BC-12); the suppression is at most a velocity-slaved patch. | — | [body_collider.md OQ-BC-11](../design/body_collider.md) |
| IP-BC-15 | blocked (OQ-BC-12) | Implement the OQ-BC-12 resolution (Alt A rigid-body contact rotation, or Alt B velocity-slaved pivot-rate approximation). Guard: the `GearLanding_WingtipGraze_DoesNotCatapultOrLaunch` reproduction + the impact-envelope + the full landing-gear regressions | — | [body_collider.md OQ-BC-12](../design/body_collider.md) |

> **IP-BC-11 reason:** OQ-BC-8 is resolved (Alt 1 — hysteretic geometric latch, derived band). Ordered
> after IP-BC-12, which removes the control-loop WoW gate, so the latch lands as reporting-only.
>
> **IP-BC-14 blocked reason:** OQ-BC-11 (whether to adopt the scoped aero-lift suppression) is
> unresolved. The steep/inverted pitch limit cycle is the lift-into-ground fight, not the collider
> moment channel — an IP-BC-13 rotational-arrest prototype shifted only the reversal count and left the
> pitch p2p unchanged (10.5° steep / 6.9° inverted), confirming the oscillator is the lift path.
> Resolve OQ-BC-11 before implementing.
>
> **IP-BC-12 (the real defect, OQ-BC-9 → Alt 1):** the body-collider post-impact behavior is a non-physical
> limit cycle across most of the impact envelope — shallow (581 pitch reversals / 584 steps, 5.8° p2p),
> steep −53° (56 reversals, 44° p2p), and inverted (480 reversals, 41° p2p); only the Vne-vertical case is
> clean (its near-normal velocity is fully arrested by the inelastic constraint, leaving negligible
> **tangential** airspeed → post-impact q below the aero band → lift loop dormant). Root cause: the FBW
> load-factor lift-shaping loop (OQ-LG-23 settle term + n_z apportionment) is driven by body-collider
> contact and fights the backstop. Resolution: decouple body-collider contact from that loop (gear-only
> gating). The four envelope regressions are already in the suite (3 fail today: shallow/steep/inverted;
> vertical passes) and are the acceptance guard.

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
- **Aircraft serialization cluster — fixed (IP-BC-10).** The deserialize-then-step regression exposed
  several pre-existing `Aircraft` serialization gaps (none body-collider-specific): the **w_a band edges**
  (`_wa_q_lo_pa`/`_wa_q_hi_pa`, the actual α-saturation cause), `_has_landing_gear`/`_has_body_collider`
  not restored (gear/collider silently dropped), the **proto omitting both subsystems** entirely,
  `_outer_dt_s` (latent), and `_body_in_hard_contact`. All fixed; `_body_in_hard_contact` (reporting-only)
  was a red herring. (Design write-up: [body_collider.md Serialization note](../design/body_collider.md).)
- **WoW detection design — OQ-BC-8 (open) → IP-BC-11.** `_body_in_hard_contact` is a reporting-only
  latch (it does **not** feed the control loop — that was the OQ-LG-22 misuse, already removed for the
  gear). Recommended replacement: a stateless geometric WoW (`minCornerClearance_m < margin`) that
  removes the latch and its serialization concern. Blocked on the OQ-BC-8 decision; characterization
  test required if it also replaces the control-loop gate.
- **Design-doc as-built reconciliation — done.** [body_collider.md](../design/body_collider.md) updated:
  §2 now describes the velocity-arrest contact (was Kelvin–Voigt), §3 the restitution-consistent
  constraint, the Class Hierarchy / Interface / Serialization / proto blocks match the current code, the
  §5 section and OQ resolutions are relabeled "implemented", and the Test Strategy cites the actual tests.

**Pre-existing unrelated failures:** `LandingGear.BearingDragCoeffs_NonzeroWhenSpindownConfigured` and
`LandingGear.DifferentialBrake_ProducesYawMoment` fail independently of this plan (brake/spindown logic).
