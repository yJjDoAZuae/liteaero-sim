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

**Last updated:** 2026-07-10

---

## Work Items

| ID | Status | Title | Depends on | Design refs |
| --- | --- | --- | --- | --- |
| IP-CRB-1 | todo | Add `mass_kg` and inverse-inertia (`I⁻¹`, 3×3) members to `BodyCollider`; add matching `BodyColliderParams` proto fields and JSON serialize/deserialize with a round-trip test | — | [body_collider.md OQ-BC-12 Alt B mech.1](../design/body_collider.md), [OQ-BC-5 resolution](../design/body_collider.md) |
| IP-CRB-2 | blocked (IP-CRB-1) | Extend `BodyCollider::initialize` to accept the body inertia tensor and store `1/m`, `I⁻¹`; `Aircraft::initialize` passes `_inertia` (mass + Ixx/Iyy/Izz) | IP-CRB-1 | [body_collider.md OQ-BC-12 Alt B mech.1](../design/body_collider.md), [OQ-BC-5 resolution](../design/body_collider.md) |
| IP-CRB-3 | blocked (IP-CRB-2) | Replace the §5a penalty normal force in `BodyCollider::step` with the momentum impulse force `F = u_n/(K_c·N_arr·dt)`, `K_c = 1/m + (r×n̂)ᵀI⁻¹(r×n̂)` per penetrating corner (small `Δv_cm`; moment `r×F` carries the correct angular impulse) | IP-CRB-2 | [body_collider.md OQ-BC-12 Alt B mech.1](../design/body_collider.md), [§5a](../design/body_collider.md) |
| IP-CRB-4 | todo | Contact-excluded attitude reference in `Aircraft::step`: subtract `(F_contact_ned/m)·dt` from the `v_att_ref` passed to `commitAttitude`, extending the OQ-LG-21 blend | — | [body_collider.md OQ-BC-12 Alt B mech.2](../design/body_collider.md), [landing_gear.md §OQ-LG-21](../design/landing_gear.md) |
| IP-CRB-5 | blocked (IP-CRB-3, IP-CRB-4) | Replace §5c roll `delta_rr = d(Δθ_roll)/dt` with a filtered wind-axis roll-rate state driven by `(M_x,gear + M_x,collider)/Ixx` with FBW/strut roll-rate damping `τ` (new `dtheta_roll_damp_tau_s` config); apply directly to the `commitAttitude` roll-rate input; leave pitch (α) and yaw (δ_ay) compliant channels unchanged | IP-CRB-3, IP-CRB-4 | [body_collider.md OQ-BC-12 Alt B mech.3 (roll)](../design/body_collider.md) |
| IP-CRB-6 | blocked (IP-CRB-5) | Acceptance harness: add the roll-persistence time-history regression (banked touchdown rolls to wings-level and *stays*, P-B4) and verify P-B1…P-B6 across `GearLanding_WithRollAndYaw_GearOnly_Settles`, `GearLanding_WingtipGraze_DoesNotCatapultOrLaunch`, the impact-envelope suite, and the existing gear regressions | IP-CRB-5 | [body_collider.md OQ-BC-12 Alt B invariants](../design/body_collider.md) |
| IP-CRB-7 | blocked (IP-CRB-5, IP-CRB-6) | Remove the superseded §5c roll deviation-derivative path and mark IP-BC-13/14/15 superseded in [body_collider_dynamics.md](body_collider_dynamics.md) | IP-CRB-5, IP-CRB-6 | [body_collider.md OQ-BC-12 Alt B mech.3 (roll)](../design/body_collider.md) |

> **IP-CRB-2 / IP-CRB-3 blocked reason:** the inertia-plumbing chain — IP-CRB-2 needs the `BodyCollider`
> inertia members and their serialization (IP-CRB-1) in place; IP-CRB-3 then needs `initialize` to have
> stored `1/m` and `I⁻¹` (IP-CRB-2) before it can form `K_c` in `step`.
>
> **IP-CRB-5 blocked reason:** the roll-currency correction needs *both* the physical collider moment
> (IP-CRB-3 — otherwise the oversized penalty moment still kicks the roll integrator) and the
> contact-excluded attitude reference (IP-CRB-4 — otherwise roll is driven twice, once through the
> integrator and once through the velocity-vector swing).
>
> **IP-CRB-6 blocked reason:** the acceptance invariants (P-B1…P-B6, including roll persistence) can only be
> evaluated once the roll fix (IP-CRB-5) is in place.
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
