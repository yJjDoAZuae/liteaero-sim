# Momentum-Consistent Filtered Contact Reaction (OQ-BC-12 Alt B) — Implementation Plan

**Scope:** Implements the OQ-BC-12 Alt B design — a unified velocity-slaved contact-reaction model
covering **both** the landing gear and the body collider, built on the existing filtered-deviation
pattern in `Aircraft`. Three mechanisms: (1) a momentum-partitioned translational reaction that replaces
the collider's §5a CM-arresting penalty force with the contact impulse `J = -(1+e) u_n / K_c`,
`K_c = 1/m + (r×n̂)ᵀ I⁻¹ (r×n̂)` (gear strut/tyre force physics unchanged); (2) a contact-excluded attitude
reference that removes the contact-induced velocity change from the velocity-slaving path-curvature (reusing
the OQ-LG-21 `v_att_ref` hook); (3) a per-axis rotational reaction — pitch and yaw stay compliant/returning
(anchored by `n_z`/α and β=0), while roll is corrected to a filtered **wind-axis roll rate applied directly
to `q_nw`**, driven by the gear torque `M_x/Ixx` plus the collider impulse kick, replacing
`delta_rr = d(Δθ_roll)/dt` so bank persists instead of springing back. Requires plumbing the inertia tensor
into `BodyCollider` (OQ-BC-5 resolved this; not yet implemented).

**Blocking open question:** **None — OQ-BC-12 was resolved to Alternative B on 2026-07-10** (design promoted
from proposal to decided design). The gate is lifted: IP-CRB-1 and IP-CRB-4 are `todo`; the remaining items
are `blocked` only on their `IP-*` predecessors and unblock as those reach `done`. This plan **supersedes**
IP-BC-13 / IP-BC-14 / IP-BC-15 in [body_collider_dynamics.md](body_collider_dynamics.md), which were the
per-symptom patches OQ-BC-12 subsumes.

**Design authority:** [body_collider.md (OQ-BC-12 → Alt B, Decided design)](../design/body_collider.md),
[landing_gear.md §2a (filtered-deviation pattern), §OQ-LG-21 (`v_att_ref`)](../design/landing_gear.md)

**Last updated:** 2026-07-12

---

## Work Items

| ID | Status | Title | Depends on | Design refs |
| --- | --- | --- | --- | --- |
| IP-CRB-1 | done | Add `_mass_kg` and body-frame **inertia diagonal** `_inertia_diag_kgm2` (Ixx/Iyy/Izz) members to `BodyCollider`; matching `BodyColliderParams` proto fields (7,8) + JSON serialize/deserialize + round-trip tests (inertia is diagonal in this model, not a full 3×3) | — | [body_collider.md OQ-BC-12 Alt B mech.1](../design/body_collider.md), [OQ-BC-5 resolution](../design/body_collider.md) |
| IP-CRB-2 | done | Extend `BodyCollider::initialize` to accept the body inertia diagonal and populate `_inertia_diag_kgm2`; `Aircraft::initialize` passes `_inertia` (Ixx/Iyy/Izz). `1/m` and `1/I` are formed at use in `step` | IP-CRB-1 | [body_collider.md OQ-BC-12 Alt B mech.1](../design/body_collider.md), [OQ-BC-5 resolution](../design/body_collider.md) |
| IP-CRB-3 | done | Replace the §5a penalty normal force in `BodyCollider::step` with the momentum impulse force `F = u_n/(K_c·N_arr·dt)`, `K_c = 1/m + (r×n̂)ᵀI⁻¹(r×n̂)` per penetrating corner (small `Δv_cm`; moment `r×F` carries the correct angular impulse). `1/(N_arr·dt)` recovered from `b_corner` (no dt stored); onset factor for C0/deep-embed. Collider suite + P-B2 long-lever test green | IP-CRB-2 | [body_collider.md OQ-BC-12 Alt B mech.1](../design/body_collider.md), [§5a](../design/body_collider.md) |
| IP-CRB-4 | done | Contact-excluded attitude reference in `Aircraft::step`: subtract `(R_nb·F_contact/m)·dt` from the `v_att_ref` passed to `commitAttitude`, extending the OQ-LG-21 blend (free-flight identity: `F_contact=0`). Combined with IP-CRB-3 this **cures the crosswind gear-landing NaN divergence** (`2.6e8 m`→finite) and reduces the wingtip launch (7.9→3.0 m); roll catapult remains pending IP-CRB-5 | — | [body_collider.md OQ-BC-12 Alt B mech.2](../design/body_collider.md), [landing_gear.md §OQ-LG-21](../design/landing_gear.md) |
| IP-CRB-5 | done | §5c roll = persistent wind-axis roll-rate state on `(M_x,gear + M_x,collider)/Ixx`, damped by the FBW `roll_rate` closed loop, **advanced with Tustin (stiff-stable) integration + a roll-rate clamp (OQ-BC-13)**. Banked wingtip touchdown settles (roll gain 173°→8.2°, no launch); no NaN; round-trips (JSON+proto). `rollRateState_rps()` accessor | IP-CRB-3, IP-CRB-4 | [body_collider.md OQ-BC-12 Alt B mech.3 (roll)](../design/body_collider.md), [OQ-BC-13 resolution](../design/body_collider.md) |
| IP-CRB-5b | done | Castering nosewheel (`is_castering` → `F_y=0`) in `WheelUnit`/`LandingGear` + config + test; removes the crosswind nose-cornering yaw disturbance. FBW-loop roll damping in place | IP-CRB-4 | [landing_gear.md §3](../design/landing_gear.md), [body_collider.md OQ-BC-12 Alt B mech.3 (yaw)](../design/body_collider.md) |
| IP-CRB-5c | todo | Retarget the §5c **yaw** channel onto the FBW `n_y` loop (`ny_wn`/`ny_zeta`), keep `Izz`, hold authority effective to zero ground speed while WoW. For the crosswind crab: the roll is now stable, but a sustained `n_y` crab command still swings the velocity vector (path-curvature) → residual roll; assess whether the yaw retarget + a fuller contact-exclusion is needed | IP-CRB-5 | [body_collider.md OQ-BC-12 Alt B mech.3 (yaw)](../design/body_collider.md) |
| IP-CRB-6 | blocked (IP-CRB-5) | Acceptance harness: add the roll-persistence time-history regression (banked touchdown rolls to wings-level and *stays*, P-B4) and verify P-B1…P-B6 across `GearLanding_WithRollAndYaw_GearOnly_Settles`, `GearLanding_WingtipGraze_DoesNotCatapultOrLaunch`, the impact-envelope suite, and the existing gear regressions | IP-CRB-5 | [body_collider.md OQ-BC-12 Alt B invariants](../design/body_collider.md) |
| IP-CRB-7 | blocked (IP-CRB-5, IP-CRB-6) | Remove the superseded §5c roll deviation-derivative path and mark IP-BC-13/14/15 superseded in [body_collider_dynamics.md](body_collider_dynamics.md) | IP-CRB-5, IP-CRB-6 | [body_collider.md OQ-BC-12 Alt B mech.3 (roll)](../design/body_collider.md) |
| IP-CRB-8 | todo | Speed-proportional saturation of the attitude-reference velocity slew (OQ-AC-2 Alt 4): in `commitAttitude` limit the per-step direction change of `v_att_ref` to `θ_max = (V/R_min)·dt` (rotate the stored previous reference toward the new one by at most `θ_max`), store the **saturated** reference as the previous, and pass it to `stepQnw`, which tracks it **strictly** (`setFromTwoVectors`, NO rotation cap — remove the direct-cap code; keep only a tiny epsilon guard, replacing the `kMinSpeed` cliff). Plumb `qnw_min_turn_radius_m` through `Aircraft` config (**fail init if unset — no default**); JSON+proto serialize + round-trip; TDD (reference slew bounded ∝ V; strict `q_nw.x = v̂` preserved; converges for steady direction). **Done + verified:** unit tests green; the 12 m/s gear crab settles (no launch, `max_agl` 2.5→0.29 m). An **interim** helix-angle bound on the contact-induced roll DOF (`p_max = kRollRateLimit_rps·V/V_stall`) was also added, but it only **caps the limit-cycle amplitude ∝ V, without dissipating** — the real fix is IP-CRB-9, which **removes** this clamp. **Still open (separate):** the gentle-approach near-stall crab launch (OQ-BC-11 aero-lift family). | IP-CRB-5 | [aircraft.md OQ-AC-2](../design/aircraft.md) |
| IP-CRB-9 | done | **Substep co-integration of the roll DOF with the strut forces (OQ-AC-3).** `LandingGear::step` split into an **outer** `beginContact()` (terrain tangent plane: height + surface normal) and a per-substep `substepContact(snap, plane, …, inner_dt)` evaluator that recomputes per-strut penetration + `contact_vel` from the passed pose against the stored plane; `step()` is a behavior-preserving wrapper. `Aircraft` §5a owns the substep loop: for each of `substeps` inner steps at `inner_dt`, advance the provisional bank, recompute `M_x`, **Tustin**-integrate `roll_rate_state`; `commitAttitude` applies the **substep-swept bank** (mean rate, phase-consistent); the aggregate force/moment fed to the EOM/§5b/§5c is the **substep MEAN** (the last-substep force injected energy — the mean was the decisive fix). Interim helix clamp removed; `roll_rate_state` no longer in §5c. Roll stays in `Aircraft`/`KinematicState`; no rigid-body EOM (trimaero). **Result:** the 90°/s limit-cycle onset is eliminated at every dt; the mode is now *stable and converging* (residual → 0 as `inner_dt` → 0, vs *growing* before). Full settle to 0°/s at rest needs `inner_dt·ωₙ` small enough to resolve the gear righting mode — for the small-UAS fixture `substeps=16` at dt=0.01 (`inner_dt=6.25e-4`) settles completely; `substeps=8` leaves a ~10°/s converging residual. `configs/small_uas_ksba.json` bumped 8→16. C++ suite 609 pass (same 6 pre-existing) | IP-CRB-8 | [aircraft.md OQ-AC-3](../design/aircraft.md), [landing_gear.md §Integration Contract](../design/landing_gear.md) |
| IP-CRB-10 | done | **Aero-velocity wind frame (OQ-AC-4, Alt 2).** Drive the `v_att_ref` (velocity-slaving reference) from the **aero** velocity `v_a = velocity_ned − wind` — done (`v_att_base = v_g − wind`), preserving the D-1 incremental `setFromTwoVectors` conditioning and the OQ-AC-2 slew cap; position keeps integrating `v_g`. **Bind wind:** the Python `step()` previously hardcoded wind = 0 — added a `wind_ned_mps` arg (done). **Initialization (key):** because the increment tracks only the *change* in the velocity direction, a steady wind's constant offset (the crab) is invisible to it — so the **initial `q_nw` must be built from the aero velocity** `v_g,init − wind,init` (not the ground velocity, which absorbs a crabbed heading into β and drives it to 0). Add derived accessors for `χ_a` (from `q_nw`), `χ_g` (from `v_g`), and **crab = χ_a − χ_g**. Verify: still air unchanged (suite green); crabbed IC (runway azimuth + crabbed heading + crosswind, no controller) holds **β = 0 with a steady, nonzero crab** to touchdown; crabbed touchdown scrubs. **Done + verified (airborne):** wind reaches the dynamics; a 1.4 m/s crosswind crabbed IC holds **β = 0, crab = −9.5° steady** through the approach. **Accessors:** `crab_rad`, `air_track_azimuth_rad`, `ground_track_azimuth_rad` on `KinematicState` + bound (χ_a − χ_g == crab exactly). **Euler-branch fix:** the flip was a real bug in liteaero-flight `KinematicStateUtil::roll_pitch_heading` — `Eigen::eulerAngles(2,1,0)` forces the first angle into [0, π] and returns the flipped (pitch magnitude > π/2) branch for headings in [−π, 0); replaced with a direct 3-2-1 `asin`/`atan2` extraction (correct-solution selection). This also **fixed `GearLanding_WithRollAndYaw`** (was tripping the Euler `peak_roll` metric) → liteaero-sim **610 pass**. Euler test coverage added (all-quadrant round-trip both directions incl. random quaternions, gimbal lock, negative-heading regression; liteaero-flight nav suite **40 pass**) | IP-CRB-9 | [aircraft.md OQ-AC-4](../design/aircraft.md) |
| IP-CRB-11 | todo | **On-ground yaw / weathervane: gear-aero moment balance (OQ-AC-5, Alt 3).** Emulate the crosswind aero yaw moment via the FBW `A_y`-hold loop; estimate the nose-gear steering **moment capability** (normal load × geometry, no modeled deflection); react the aero moment through the gear (tyre side forces + steering capability) up to that limit, yawing on the excess; **scale by contact/normal force** for a smooth flight↔ground transition. Reconcile the OQ-AC-2 azimuth gate (ground- vs aero-speed) so a parked crosswind holds a **static β = wind angle** (gear-reacted, no free weathervane). Verify: light wind → gear holds (static β, no weathervane); strong wind → slips; smooth transition as contact force fades; airborne weathervane recovered. Design fully resolved and math developed in [ground_directional_dynamics.md](../algorithms/ground_directional_dynamics.md): the gear conditions **which velocity the heading slaves to** via the hold fraction `w_hold = min(k_g/(k_a+k_g), N_gear,max/(k_a·abs(c)))` (c = crab), realized as the attitude reference `v_att,ref = v_g − (1−w_hold)·wind` (the sole change from the OQ-AC-4 aero reference). `k_a = abs(Cy_β)·qS·x_acy` (OQ-AC-7), `k_g = Σ C_α,i·x_i`, `N_gear,max = N_main + μ_y·F_z,nose·x_nose` (OQ-AC-8); new config `x_acy`. Gives static `β = w_hold·c` on the ground, reduces to coordinated flight (`β=0`) at liftoff. The commanded `Ny` separately steers the ground track `χ_g` (OQ-AC-6). **NB:** the earlier `a_y`-side-force-gating sketch was wrong (`β≈0` for `Ny=0`, so it did nothing) — the balance conditions the velocity-slaving, not `a_y`. Authority: [aircraft.md §On-Ground Gear-Aero Yaw Balance](../design/aircraft.md). | IP-CRB-10 | [aircraft.md §On-Ground Gear-Aero Yaw Balance](../design/aircraft.md), [OQ-AC-5](../design/aircraft.md), [OQ-AC-7](../design/aircraft.md), [OQ-AC-8](../design/aircraft.md) |
| IP-CRB-12 | done | **`Ny` command curvature authority limit (OQ-AC-6).** Clamp the FBW `Ny` command to a maximum path curvature `Ny·g ≤ V²/R` instead of a constant g-envelope, blending a flight regime (airspeed, `R_flight = qnw_min_turn_radius_m`) and a ground-steering regime (ground speed, new `ground_steering_min_turn_radius_m`) by weight `w = f_WoW · (1 − smoothstep(V_ground; V_lo,V_hi))`, where `f_WoW` = previous-step gear vertical-load fraction and the smoothstep band is right below stall (new `ground_steering_vblend_lower_ratio`,`_upper_ratio × V_stall`). Airspeed in the flight term keeps full authority in a headwind; the WoW gate keeps a headwind from engaging the ground regime airborne; the below-stall smoothstep eases a fast touchdown into `R_ground`. New required config (fail-init if unset) + JSON/proto serialize + `_prev_wow_load_fraction` state; TDD (pure `lateralLoadAuthority` unit tests across regimes + airborne β speed-independence at bind). **No** channel remap, β/curvature input, steerable-gear use, or control loop. | IP-CRB-10 | [aircraft.md OQ-AC-6](../design/aircraft.md), [§Lateral Authority Limit](../design/aircraft.md) |

> **IP-CRB-5 blocked reason:** the persistent-rate roll channel is coded, but its **explicit** integration
> of the (correctly-righting) gear roll moment against the tiny airframe roll inertia is a stiff mode that
> diverges at the outer step — **OQ-BC-13** must be resolved (stiff-stable integration vs characterized
> bank-restoring mode) before the roll channel is stable and the banked/crosswind regressions can pass.
>
> **IP-CRB-5c blocked reason:** the yaw retarget onto the `n_y` loop is a refinement that only matters once
> the roll channel is stable (IP-CRB-5); until then the roll divergence dominates the crosswind result.
>
> **IP-CRB-6 blocked reason:** the acceptance invariants (P-B1…P-B6, including roll persistence) can only be
> evaluated once the roll fix (IP-CRB-5) is stable.
>
> **IP-CRB-7 blocked reason:** the superseded roll deviation-derivative path may be removed only after its
> replacement (IP-CRB-5) is verified by the full acceptance run (IP-CRB-6).

---

## Notes

**Ordering rationale.** Two independent roots feed the roll fix and can proceed in parallel once the OQ
closes: the **inertia plumbing chain** (IP-CRB-1 → IP-CRB-2 → IP-CRB-3) that makes the collider's contact
force momentum-correct, and the **attitude-reference fix** (IP-CRB-4) that removes the path-curvature whip
(the diagnostic-confirmed dominant driver of the gear-only catapult). IP-CRB-5 (the roll-currency
correction) depends on both: it needs the physical collider moment (IP-CRB-3, else the huge penalty moment
still kicks the roll integrator) and the contact-excluded reference (IP-CRB-4, else roll is driven twice —
once through the integrator and once through the velocity swing).

**Pitch and yaw are deliberately untouched.** They are anchored (pitch to `n_z`/α, yaw to β=0), so their
existing compliant/returning channels are correct; the collider's now-smaller impulse moment simply flows
through them unchanged. No work items are needed for those axes, which is what keeps this a bounded change.

**Gear is touched in exactly two bounded ways** — the shared contact-excluded attitude reference (IP-CRB-4)
and the roll-rate currency (IP-CRB-5). The strut/tyre **force** physics and the gear pitch/force channels
are unchanged.

**Serialization ordering.** IP-CRB-1 adds proto fields and JSON keys before any consumer uses them
(IP-CRB-2), per the standing rule that proto/schema additions precede the C++ that depends on them, so the
serialization round-trip tests never see a half-migrated message.

**Residual scope limit (not a work item).** Alt B does not conserve angular momentum across successive
contacts (the collider pivot bleeds on `τ` rather than propagating as `Iω`); this is the accepted fidelity
boundary recorded in the design and is out of scope for this plan. Crossing it is the Alt A decision, not a
CRB item.

**Discovered gap during implementation (needs a decision — rule 14).** After IP-CRB-3/4, the steep-dive
(−53°) and inverted body-collider impact tests (`BodyColliderOnly_SteepDiveImpact`,
`BodyColliderOnly_InvertedImpact`) still limit-cycle in **pitch** (~49 / ~616 reversals) — these are the
*pre-existing* OQ-BC-11 "lift-into-ground fight" failures. Alt B mechanisms 1–3 do **not** address them:
the driver is the FBW commanding aero lift (`n_z=+1`) *toward* the terrain in a pitched/inverted attitude,
an aero-lift loop that neither the momentum impulse nor the contact-excluded reference touches. The
OQ-BC-12 closure over-claimed that Alt B addresses OQ-BC-11. Additionally the shallow-belly
`BodyColliderOnly_Landing_DoesNotOscillateAfterContact` regressed slightly (pitch p2p 1.0→1.72°) from the
smaller impulse force. **Resolution options:** (a) add the scoped aero-lift suppression (OQ-BC-9 Alt 3 /
OQ-BC-11 Alt 1 — clamp aero lift toward zero while body-collider contact is the active contact) as a 4th
Alt B mechanism; (b) reopen OQ-BC-11 as a separate follow-up and re-scope these envelope tests out of the
CRB acceptance. Roll (IP-CRB-5) is orthogonal to this pitch gap and proceeds regardless.
