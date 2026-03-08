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

## 2. JSON Parameter Schema

Define a JSON schema for initialization of the simulation model from a configuration file.
The schema covers only parameters that are currently known to be used by the model; extend
it as new functionality is designed and implemented.

### Currently Known Parameters

**Lift curve (`"lift_curve"`)** — maps directly to `LiftCurveParams`:

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `cl_alpha` | float | rad⁻¹ | Pre-stall lift-curve slope |
| `cl_max` | float | — | Peak lift coefficient (positive stall vertex) |
| `cl_min` | float | — | Minimum lift coefficient (negative stall vertex, < 0) |
| `delta_alpha_stall` | float | rad | Positive linear-join to stall-peak angular distance |
| `delta_alpha_stall_neg` | float | rad | Negative linear-join to stall-trough angular distance |
| `cl_sep` | float | — | Positive post-stall plateau (≤ cl_max) |
| `cl_sep_neg` | float | — | Negative post-stall plateau (≥ cl_min) |

**Aircraft (`"aircraft"`)** — mass and geometry used by `LoadFactorAllocator`:

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `S_ref_m2` | float | m² | Reference wing area |
| `mass_kg` | float | kg | Aircraft mass |
| `cl_y_beta` | float | rad⁻¹ | Lateral force slope C_Yβ (< 0) |

**Initial state (`"initial_state"`)** — passed to the `KinematicState` constructor:

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `latitude_rad` | double | rad | Geodetic latitude |
| `longitude_rad` | double | rad | Longitude |
| `altitude_m` | float | m | WGS84 ellipsoidal altitude |
| `velocity_north_mps` | float | m/s | Initial northward velocity component |
| `velocity_east_mps` | float | m/s | Initial eastward velocity component |
| `velocity_down_mps` | float | m/s | Initial downward velocity component |
| `wind_north_mps` | float | m/s | NED-north wind component (positive = northward) |
| `wind_east_mps` | float | m/s | NED-east wind component (positive = eastward) |
| `wind_down_mps` | float | m/s | NED-down wind component (positive = downward; negative = updraft) |

### Top-Level Schema Structure

```json
{
    "schema_version": 1,
    "aircraft": { ... },
    "lift_curve": { ... },
    "initial_state": { ... }
}
```

Every object includes `"schema_version"` (integer) consistent with the project
serialization convention.

### Example Files

Create example JSON files under `test/data/aircraft/` for at least three aircraft
categories.  Each file provides realistic parameter values and serves as a test fixture.

| File | Category | Representative type |
|------|----------|---------------------|
| `test/data/aircraft/general_aviation.json` | General aviation | Single-engine piston (Cessna 172 analog) |
| `test/data/aircraft/jet_trainer.json` | Jet trainer | Subsonic two-seat trainer (T-38 analog) |
| `test/data/aircraft/small_uas.json` | Small UAS (< 25 kg) | Fixed-wing electric UAV |

### Deliverables

1. Schema reference at `docs/schemas/aircraft_config_v1.md` — field definitions, SI units,
   allowed ranges, and invariants (e.g., `cl_sep ≤ cl_max`, `delta_alpha_stall > 0`).
2. Example JSON files under `test/data/aircraft/` for each category above.
3. A schema validation script (`python/tools/validate_aircraft_config.py`) that can be run
   against any JSON file to verify compliance without building the C++ project.  The script
   checks:
   - All required fields are present in each section.
   - Field types match the schema (float vs. double vs. int).
   - Numeric ranges are satisfied (e.g., `delta_alpha_stall > 0`, `mass_kg > 0`,
     `S_ref_m2 > 0`).
   - Cross-field invariants hold (e.g., `cl_sep ≤ cl_max`, `cl_sep_neg ≥ cl_min`,
     `cl_min < 0`).
   - `schema_version` is present and equals the expected value.

   Exit code 0 on success; non-zero with a human-readable error message on failure.
   Usage: `python python/tools/validate_aircraft_config.py <file.json>`

   Tests for the validator live in `python/test/test_validate_aircraft_config.py` and
   cover at least: a valid file passes, each required field missing triggers failure, each
   invariant violation triggers failure.
