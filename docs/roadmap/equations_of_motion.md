# Equations of Motion — Recommended Next Steps

Items are listed roughly in dependency order.  Each item should follow TDD: write a
failing test before writing production code.

---

## ~~1. Higher-Order Integration~~ ✅ Complete

`KinematicState::step()` integrates position and velocity with classic RK4 (4 evaluations
per step, 4th-order accuracy).  The previous trapezoidal/Euler scheme has been replaced.

**Design document:** [`docs/algorithms/integration.md`](../algorithms/integration.md) —
covers algorithm options, the `IVehicleDynamics` / `rk4Step<>` architecture for
multi-vehicle support, and the migration path to 6DOF.

**Next step (proposed):** Introduce the `IVehicleDynamics` interface and `rk4Step<>`
template to decouple the integrator from the trim-aero vehicle model, enabling 6DOF
extension without changing the integrator code.  See §3–§5 of the design document.

---

## ~~2. JSON Parameter Schema~~ ✅ Complete

A JSON schema for simulation model initialization has been defined and implemented.

**Schema reference:** [`docs/schemas/aircraft_config_v1.md`](../schemas/aircraft_config_v1.md) —
field definitions, SI units, allowed ranges, and cross-field invariants.

**Example fixture files** under `test/data/aircraft/`:

| File | Category | Representative type |
|------|----------|---------------------|
| `general_aviation.json` | General aviation | Single-engine piston (Cessna 172 analog) |
| `jet_trainer.json` | Jet trainer | Subsonic two-seat trainer (T-38 analog) |
| `small_uas.json` | Small UAS (< 25 kg) | Fixed-wing electric UAV |

**Validation script:** `python/tools/validate_aircraft_config.py` — checks all required
fields, field types, numeric ranges, and cross-field invariants.  Exit code 0 on success.

Usage: `python python/tools/validate_aircraft_config.py <file.json>`

**Tests:** `python/test/test_validate_aircraft_config.py` — 40 pytest cases covering valid
input, each missing field, each range violation, type errors, and bool rejection.
