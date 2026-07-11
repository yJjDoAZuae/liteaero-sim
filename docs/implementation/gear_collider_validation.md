# Gear & Collider Validation — Bindings + Notebook Scenario Coverage — Implementation Plan

**Scope:** Deliver "much better landing gear and ground collider validation capability" (user directive):
(1) Python bindings that expose the **verbose per-wheel and collider model internals** already computed in
C++ but not currently reachable from Python, and (2) **full scenario coverage** in
`gear_terrain_validation.ipynb` — the lateral-asymmetry landing cases (banked, crabbed, banked+crabbed,
single-main-first, wingtip graze) plus hard touchdown, on top of the existing straight-in and touch-and-go.
This is roadmap item 7 (LandingGear Python bindings + scenario tests + notebooks), expanded per the user's
requirements. The lateral-contact dynamics are now stable in the model (OQ-BC-12 Alt B + OQ-BC-13 Tustin),
so these cases can finally be exercised without divergence.

**Plotting requirements (user):** do **not** overlay collider-only cases on gear-present cases (separate
figures per scenario), but **do** surface whether the body collider engages *within* a gear landing (a
collider-involvement trace — collider WoW / collider force — in the gear-present figures).

**Design authority:** [landing_gear.md §3 (ContactDiag), §Test Strategy / §Scenario Tests / §Notebooks](../design/landing_gear.md),
[body_collider.md (OQ-BC-12 Alt B)](../design/body_collider.md),
[python_bindings.md](../design/python_bindings.md)

**Last updated:** 2026-07-11

**Note on C++ accessors:** most already exist — `Aircraft.landingGear().wheelUnits()[i].lastContactDiag()`
returns the full per-wheel breakdown; `Aircraft.rollRateState_rps()` exists (IP-CRB-5). The gaps are
(a) *binding* these, and (b) a small accessor to separate the **collider-only** contribution from the
summed `contactForces()` so gear cases can show collider involvement.

---

## Work Items

| ID | Status | Title | Depends on | Design refs |
| --- | --- | --- | --- | --- |
| IP-GCV-1 | done | Bound `WheelUnit::ContactDiag` as `WheelContactDiag` (F_z/F_x/F_y/F_rr/kappa/alpha_t/V_cx/V_cy/omega/delta_dot/force_body_n) + `Aircraft.wheel_contact_diags()` + `Aircraft.roll_rate_state_rps` in `bind_aircraft.cpp`; pyd rebuilt, smoke-tested (all exposed, 3 wheels, castering nose F_y=0). Also fixed a **stale `site-packages` pyd** shadowing the fresh build | — | [landing_gear.md §3](../design/landing_gear.md) |
| IP-GCV-2 | todo | Bind `Aircraft.roll_rate_state_rps` (`rollRateState_rps()`) and the collider hard-contact latch for Alt B validation | — | [body_collider.md OQ-BC-12](../design/body_collider.md) |
| IP-GCV-3 | blocked (IP-GCV-2) | Add + bind a **collider-only** contact accessor (last collider `ContactForces` + WoW, separate from the summed gear+collider `contactForces()`) so gear-present figures can show collider engagement | IP-GCV-2 | [body_collider.md §Integration](../design/body_collider.md), [aircraft.md](../design/aircraft.md) |
| IP-GCV-4 | blocked (IP-GCV-1, IP-GCV-3) | Notebook: parameterize the sim runner for lateral commands (roll-rate schedule, `n_y`, initial roll/heading) and collect the new diagnostics (per-wheel ContactDiag, roll-rate state, collider-only force/WoW) | IP-GCV-1, IP-GCV-3 | [landing_gear.md §Notebooks](../design/landing_gear.md) |
| IP-GCV-5 | blocked (IP-GCV-4) | Notebook: lateral-asymmetry scenario cells — **banked**, **crabbed (crosswind)**, **banked+crabbed**, **single-main-first**, **wingtip graze** — each in its **own figure** (not overlaid with the gear-vs-no-gear comparison) | IP-GCV-4 | [landing_gear.md §Scenario Tests, `crab_landing_dynamics.ipynb`](../design/landing_gear.md), [body_collider.md OQ-BC-12](../design/body_collider.md) |
| IP-GCV-6 | blocked (IP-GCV-4, IP-GCV-5) | Notebook: per-wheel **friction-utilization** and tyre-force (F_x/F_y/slip) plots + **collider-involvement trace** within the gear cases; ground-track (top-down) colored by friction | IP-GCV-4, IP-GCV-5 | [landing_gear.md §`crab_landing_dynamics.ipynb`](../design/landing_gear.md) |
| IP-GCV-7 | blocked (IP-GCV-4) | Notebook: hard-touchdown (high sink) case; optional body-collider envelope cases (steep/inverted) in a **separate** collider-only figure | IP-GCV-4 | [body_collider.md §Test Strategy](../design/body_collider.md) |
| IP-GCV-8 | blocked (IP-GCV-1, IP-GCV-3) | `python/test/` pytest scenario tests mirroring the C++ `GearLanding_*` lateral cases (banked settles, crabbed settles, wingtip does not catapult), asserting roll/launch bounds via the bindings | IP-GCV-1, IP-GCV-3 | [landing_gear.md §Scenario Tests](../design/landing_gear.md), [body_collider.md OQ-BC-12 invariants](../design/body_collider.md) |

> **IP-GCV-3 blocked reason:** the collider-only accessor is grouped with the other Alt B state bindings
> (IP-GCV-2) and shares the `bind_aircraft.cpp` edit; it also needs a small C++ accessor on `Aircraft` for
> the last collider contribution, which is cleanest added alongside IP-GCV-2.
>
> **IP-GCV-4 blocked reason:** the parameterized runner collects the new diagnostics, so it needs the
> per-wheel ContactDiag binding (IP-GCV-1) and the collider-only accessor (IP-GCV-3) available in Python.
>
> **IP-GCV-5 / IP-GCV-6 / IP-GCV-7 blocked reason:** the scenario/plot cells consume the runner and
> diagnostics from IP-GCV-4 (and IP-GCV-5 for the friction/involvement overlays in IP-GCV-6).
>
> **IP-GCV-8 blocked reason:** the pytest scenario asserts on the bound diagnostics (IP-GCV-1/3).

---

## Notes

**Ordering rationale.** The two binding roots (IP-GCV-1 per-wheel diags; IP-GCV-2→3 Alt B / collider-only
state) are independent and unblock the notebook runner (IP-GCV-4), which everything else consumes. Bindings
first because they require rebuilding `liteaero_sim_py` (the `.pyd`) with the uv-managed MSVC Python
(`-DPython3_EXECUTABLE=...`), a heavier build than the C++ test target.

**Config prerequisite (done).** `configs/small_uas_ksba.json` (the notebook's config) already carries the
low roll inertia (`Ixx=0.08`) and now the castering nose wheel (`is_castering: true`), so the notebook
reproduces the same lateral dynamics as the C++ `GearLanding_*` tests.

**Scenario matrix (target full coverage).** Straight-in (exists), touch-and-go (exists), banked, crabbed,
banked+crabbed, single-main-first, wingtip graze, hard touchdown; body-collider envelope (steep/inverted)
kept to a separate collider-only figure.

**Corrected finding (IP-GCV-4/5, notebook config with a *proper* approach).** An initial crude smoke test
(fixed `n_z=1`, 12 m/s, no approach) showed a banked catapult; that was a **test-setup artifact**. Run
through the notebook's stabilized **FPA-hold approach at Vref (8.4 m/s)**, the notebook config
(`small_uas_ksba.json`) behaves correctly:

- **Banked ~4°:** peak roll **4.6°**, final **0.0°**, no launch (0.23 m) — settles to wings-level and stays.
  The OQ-BC-12 Alt B roll fix generalizes to the notebook config.
- **Castering nose confirmed:** nose `F_y` peak = **0.000** in all cases; the mains carry the cornering.
- **Crabbed / banked+crabbed:** still catapult (~175°) — the **known crosswind transient** (sustained `n_y`
  crab → mains' lateral force → path-curvature whip), i.e. the deferred cumulative-contact-exclusion item
  ([contact_reaction_alt_b.md](contact_reaction_alt_b.md)), not a new issue.

**Build-hygiene note.** The uv venv's `site-packages/liteaero_sim_py.cp312-win_amd64.pyd` was a **stale
copy** that shadowed the freshly-built pyd (the build copies to `python/`, and the notebook prepends
`python/` to `sys.path`, so the notebook is fine — but any plain `import` from the venv gets the stale one,
with an old config schema). Refreshed manually; the install/copy step should target `site-packages` too.
