# Aircraft Configuration Schema — Version 1

This document is the authoritative reference for the `aircraft_config_v1` JSON schema used
to initialize the simulation model.  The schema is validated by
`python/tools/validate_aircraft_config.py` before any C++ component reads the file.

---

## Top-Level Structure

```json
{
    "schema_version": 1,
    "aircraft":      { ... },
    "airframe":      { ... },
    "inertia":       { ... },
    "lift_curve":    { ... },
    "initial_state": { ... }
}
```

| Field | Type | Description |
|-------|------|-------------|
| `schema_version` | integer | Must equal `1`.  Increment when backward-incompatible changes are made. |
| `aircraft` | object | Aerodynamic geometry and drag polar parameters. |
| `airframe` | object | Structural and operational performance limits (envelope). |
| `inertia` | object | Mass and moment-of-inertia properties. |
| `lift_curve` | object | Lift-curve model parameters. |
| `initial_state` | object | Initial kinematic state. |

---

## `aircraft` Section

Maps to `LoadFactorAllocator` and `AeroPerformance` constructor parameters.

| Field | Type | Unit | Constraint | Description |
|-------|------|------|------------|-------------|
| `S_ref_m2` | float | m² | > 0 | Reference wing area. |
| `cl_y_beta` | float | rad⁻¹ | < 0 | Lateral force coefficient slope C_Yβ. |
| `ar` | float | — | > 0 | Wing aspect ratio b²/S. |
| `e` | float | — | 0 < e ≤ 1 | Oswald span efficiency factor. |
| `cd0` | float | — | > 0 | Zero-lift drag coefficient. |

`S_ref_m2` and `cl_y_beta` are shared inputs to both `LoadFactorAllocator` and
`AeroPerformance`. `ar`, `e`, and `cd0` are used exclusively by `AeroPerformance` to
compute the drag polar: CD = cd0 + CL² / (π · e · ar).

---

## `airframe` Section

Maps to `AirframePerformance`. Used by `Aircraft::step()` to clamp commanded load factors
before passing them to `LoadFactorAllocator`.

| Field | Type | Unit | Constraint | Description |
|-------|------|------|------------|-------------|
| `g_max_nd` | float | g | > 0 | Maximum positive load factor. |
| `g_min_nd` | float | g | < 0 | Maximum negative load factor (structural limit). |
| `tas_max_mps` | float | m/s | > 0 | Never-exceed true airspeed. |
| `mach_max_nd` | float | — | > 0 | Never-exceed Mach number. |

---

## `inertia` Section

Maps to `Inertia`. `mass_kg` is used every step for force-to-acceleration conversion.
Moment-of-inertia fields are stored and reserved for the 6-DOF moment equations (not yet
active in the point-mass model).

| Field | Type | Unit | Constraint | Description |
|-------|------|------|------------|-------------|
| `mass_kg` | float | kg | > 0 | Total aircraft mass. |
| `Ixx_kgm2` | float | kg·m² | > 0 | Roll moment of inertia. |
| `Iyy_kgm2` | float | kg·m² | > 0 | Pitch moment of inertia. |
| `Izz_kgm2` | float | kg·m² | > 0 | Yaw moment of inertia. |

---

## `lift_curve` Section

Maps directly to `LiftCurveParams` used by `LiftCurveModel`.

| Field | Type | Unit | Constraint | Description |
|-------|------|------|------------|-------------|
| `cl_alpha` | float | rad⁻¹ | — | Pre-stall lift-curve slope. |
| `cl_max` | float | — | — | Peak lift coefficient (positive stall vertex). |
| `cl_min` | float | — | < 0 | Minimum lift coefficient (negative stall vertex). |
| `delta_alpha_stall` | float | rad | > 0 | Angular distance from the linear-region join to the positive stall peak. |
| `delta_alpha_stall_neg` | float | rad | > 0 | Angular distance from the linear-region join to the negative stall trough. |
| `cl_sep` | float | — | ≤ `cl_max` | Positive post-stall separated-flow plateau. |
| `cl_sep_neg` | float | — | ≥ `cl_min` | Negative post-stall separated-flow plateau. |

### Cross-Field Invariants

| Invariant | Rationale |
|-----------|-----------|
| `cl_sep ≤ cl_max` | The post-stall plateau cannot exceed the stall peak. |
| `cl_sep_neg ≥ cl_min` | The negative plateau cannot exceed (in magnitude) the negative stall trough. |
| `cl_min < 0` | The negative stall vertex must be below zero. |

---

## `initial_state` Section

Passed to the `KinematicState` constructor.  All velocities are ground-referenced
(Earth-fixed NED frame).

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `latitude_rad` | float | rad | Geodetic latitude (WGS84). |
| `longitude_rad` | float | rad | Longitude (WGS84). |
| `altitude_m` | float | m | WGS84 ellipsoidal altitude (positive upward). |
| `velocity_north_mps` | float | m/s | Initial northward velocity component. |
| `velocity_east_mps` | float | m/s | Initial eastward velocity component. |
| `velocity_down_mps` | float | m/s | Initial downward velocity component (positive = descending). |
| `wind_north_mps` | float | m/s | NED-north wind component (positive = northward). |
| `wind_east_mps` | float | m/s | NED-east wind component (positive = eastward). |
| `wind_down_mps` | float | m/s | NED-down wind component (positive = downward; negative = updraft). |

---

## Numeric Type Convention

All numeric fields accept JSON numbers with or without a decimal point.  The C++ layer
stores them as `float` (single precision) for all fields except `latitude_rad` and
`longitude_rad`, which are stored as `double`.  The Python validator accepts both
`int` and `float` JSON values for all fields; `bool` values are rejected.

---

## Validation Script

```
python python/tools/validate_aircraft_config.py <file.json>
```

Exit code 0 on success; non-zero with a human-readable error message on failure.

Tests for the validator live in `python/test/test_validate_aircraft_config.py`.

---

## Example Files

Three canonical fixture files are provided under `test/data/aircraft/`:

| File | Category | Representative type |
|------|----------|---------------------|
| [`general_aviation.json`](../../test/data/aircraft/general_aviation.json) | General aviation | Single-engine piston (Cessna 172 analog) |
| [`jet_trainer.json`](../../test/data/aircraft/jet_trainer.json) | Jet trainer | Subsonic two-seat trainer (T-38 analog) |
| [`small_uas.json`](../../test/data/aircraft/small_uas.json) | Small UAS (< 25 kg) | Fixed-wing electric UAV |
