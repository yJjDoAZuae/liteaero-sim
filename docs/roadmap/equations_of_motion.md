# Equations of Motion — Recommended Next Steps

Items are listed roughly in dependency order.  Each item should follow TDD: write a
failing test before writing production code.

---

## 1. 3D Wind Support

The current wind model is 2D (horizontal only): `windSpeed_mps` + `windDirFrom_rad`
always sets `_wind_NED_mps[2] = 0`.  Updrafts, downdrafts, and turbulence require a
full 3D wind vector.

**Change:** Replace the `windSpeed_mps` / `windDirFrom_rad` pair in both the constructor
and `step()` with a single `Eigen::Vector3f wind_NED_mps` parameter.  Update all call
sites.  The 2D convenience conversion can live in a utility function at the interface
layer if needed.

---

## 2. LoadFactorAllocator — AeroPerformance Integration

`LoadFactorAllocator` solves for α and β given commanded load factors, but is not yet
wired to any upstream controller or downstream force model.  The intended data flow is:

```
FlightController
    → load factors (n, n_y)
LoadFactorAllocator::solve()
    → (α, β, stall)
AeroPerformance (forces from α, β, q_inf)
    → acceleration_Wind_mps
KinematicState::step()
    → updated position / velocity / attitude
```

The `AeroPerformance` class needs to be designed or extended to consume α and β from
`LoadFactorAllocator` rather than computing them internally.

---

## 3. Higher-Order Integration

`KinematicState::step()` uses forward Euler for velocity and first-order position
integration.  For scenarios requiring long-horizon accuracy (e.g., trajectory planning
over minutes), consider replacing with RK4 or a symplectic integrator.  This is low
priority for the current point-mass model.

---

## 4. JSON Parameter Schema

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
| `wind_speed_mps` | float | m/s | Horizontal wind speed |
| `wind_dir_from_rad` | float | rad | Wind direction (meteorological from-direction) |

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
