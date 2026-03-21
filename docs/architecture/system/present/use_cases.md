# Use Cases — Present State

---

## UC-1 — Configure and Initialize a Simulation

**Actor:** Simulation framework (external caller, not yet implemented)

**Preconditions:** JSON configuration files are available.

**Steps:**

1. Caller loads component JSON configurations.
2. Caller constructs each component and calls `initialize(config)`.
3. Caller establishes any inter-component data references (e.g., terrain pointer injected into sensors).
4. Caller calls `reset()` to set initial conditions.

**Postconditions:** All components are in a consistent, ready-to-step state.

---

## UC-2 — Run a Simulation Step

**Actor:** Simulation framework

**Preconditions:** All components initialized and reset.

**Steps:**

1. Step environment: query `Atmosphere`, `Wind`, `Turbulence`, `Gust`; assemble `EnvironmentState`.
2. Step `Aircraft` with `AircraftCommand` and `EnvironmentState`; receive updated `KinematicState`.
3. Step each `Sensor` with truth state and `AtmosphericState`; receive measurement structs.
4. Write logged quantities to `Logger`.

**Postconditions:** `KinematicState` advanced by one timestep; all sensor outputs current; log updated.

**Note:** No simulation runner is currently implemented. This use case describes the intended
orchestration pattern; the caller currently performs this sequence manually.

---

## UC-3 — Reset Between Runs

**Actor:** Simulation framework

**Steps:**

1. Call `reset()` on all components in dependency order.
2. Optionally supply new initial conditions (e.g., a new starting `KinematicState`).
3. Resume at UC-2.

---

## UC-4 — Checkpoint and Restore State

**Actor:** Simulation framework

**Steps:**

1. Call `serializeJson()` (or `serializeProto()`) on all components; store snapshots.
2. Later, call `deserializeJson()` (or `deserializeProto()`) on each component to restore state.
3. Next `step()` call on each component produces output consistent with the checkpoint.

---

## UC-5 — Ingest Terrain Data

**Actor:** Python terrain pipeline (offline)

**Steps:**

1. Download digital elevation model (DEM) from Copernicus or NASA EarthData.
2. Mosaic and reproject to ENU reference frame.
3. Apply geoid undulation correction.
4. Triangulate DEM into a triangle mesh (Delaunay TIN).
5. Colorize facets from imagery raster.
6. Simplify mesh using quadric error metric.
7. Verify mesh quality; export to `.las_terrain` binary format.
8. (Optional) Export to `.glb` (glTF) for visualization.

**Postconditions:** `.las_terrain` file ready for `TerrainMesh` initialization.

---

## UC-6 — Post-Flight Analysis

**Actor:** Engineer (offline)

**Steps:**

1. Load `.csv` or MCAP log file produced by `Logger`.
2. Use Python post-processing tools to produce time-series plots.
3. Inspect physical quantities, sensor outputs, and trajectory.
