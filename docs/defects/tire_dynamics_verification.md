# Tire Dynamics Verification — Free-Rolling Tire Cannot Propel

**Status:** Defect found and fixed; verified by isolated tests and plots (2026-06).

## Invariant

A free-rolling (unbraked, undriven) tire can only produce **rolling resistance** and **slip
forces that oppose the contact-patch motion**. It must never produce a net longitudinal
**traction** that propels the vehicle. Equivalently, the Pacejka longitudinal force `F_x` must be
≈ 0 whenever the wheel is free (no brake torque), because a free wheel spins to match the contact
speed (slip ratio κ = 0).

## Defect

In [`WheelUnit::step`](../../src/landing_gear/WheelUnit.cpp) the free-roll quasi-static wheel speed
was clamped non-negative:

```cpp
_wheel_speed_rps = std::max(0.0f, V_cx / r_w);   // BUG
```

When the contact-patch longitudinal velocity `V_cx < 0` (the patch moving aft — which happens
transiently under **any** pitch rotation of a lever-arm wheel, e.g. the nose wheel during an
attitude swing), the `max(0, …)` forced ω = 0. The wheel was then modeled as **locked** against a
rearward-moving patch, giving

```
κ = (r_w·ω − V_cx) / V_ref = −V_cx / V_ref ≈ +1
```

and `pacejka(+1) ≈ +μ·F_z` — a large **forward** traction. A free wheel that simply can't spin
backward was modeled as a *driven* wheel gripping the runway and **propelling** the aircraft. This
was the source of the periodic forward impulses in `LandingGear_FullStop_SpeedNearZero`.

## Fix

Allow ω to match the contact patch in either direction, so κ = 0 for both signs of `V_cx`:

```cpp
_wheel_speed_rps = V_cx / r_w;   // free roll: match contact speed (incl. reverse) → κ = 0
```

A free wheel then produces `F_x = 0` regardless of `V_cx` sign; only rolling resistance (which
opposes the contact motion) and lateral slip remain.

## Verification (isolated tests)

All in [`test/TireDynamics_test.cpp`](../../test/TireDynamics_test.cpp) (single tire, contact
motion prescribed directly into `WheelUnit::step`, so strut/attitude/aero/collider are excluded)
and one aircraft-level test in [`test/Aircraft_test.cpp`](../../test/Aircraft_test.cpp):

| Test | Scenario | Assertion |
| --- | --- | --- |
| `FreeRoll_LongitudinalForceIsRollingResistanceOnly` | prescribed `V_cx` sweep, both signs | `|F_long| ≤ rr·F_z` and `F_long·V_cx ≤ 0` (opposes slip) |
| `FreeRoll_SettlingDoesNotPropel` | forward speed + vertical descent | longitudinal force retarding |
| `SprungMass_ConstantForwardSpeed_NetRetarding` | 1-DOF mass on strut, constant forward speed, 40 s | net longitudinal impulse < 0 |
| `LandingGear_TireNeverPropels_FullScenario` | full 90 s aircraft ground roll (aero present, no body collider, with bouncing) | per-wheel `max|F_x| < 5 N` |

Before the fix, `FreeRoll_LongitudinalForceIsRollingResistanceOnly` reported **+1503 N** forward
traction at every `V_cx < 0`. After the fix, all pass; the aircraft-run nose-tire traction peaks
at **0.04 N**.

## Plots

**Single free-rolling tire — longitudinal force vs contact speed.** The Pacejka traction `F_x` is
identically zero for both signs of `V_cx`; the net longitudinal force is pure rolling resistance
(a ±40 N step that always opposes the contact motion).

![Free-roll sweep](img/tire_freeroll_sweep.png)

**Aircraft ground roll — tire traction vs total contact force.** The nose-tire `F_x` (top, and red
in the lower panel) stays ≈ 0 for the entire run. The large oscillation in the **total** wind-frame
contact force (lower panel, gray) is the **normal-force projection** `F_z·sinγ` during bouncing —
a separate strut/flight-path effect, *not* tire traction.

![Aircraft tire F_x](img/tire_aircraft_fx.png)

Regenerate: run the diagnostic and sweep tests to refresh the CSVs under `build/test/`, then
`python docs/defects/img/plot_tire_dynamics.py build/test/tire_freeroll_sweep.csv build/test/oq_lg15_diagnostic.csv`.

## Scope note

This document covers the **tire** longitudinal force only. The residual ±5–25 kN oscillation in the
*total* contact force during the FullStop scenario (and the fact that the scenario still plateaus
near 6.4 m/s) is the normal-force projection plus the wing-lift / Δθ behavior — tracked separately,
not a tire-traction defect.
