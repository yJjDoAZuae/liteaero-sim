# Terrain Build Tool — Architecture and Design

Design authority for the automated terrain build pipeline that produces a ready-to-use
terrain dataset from an aircraft configuration JSON file.

Implemented by roadmap item TB-1.  Depends on the existing terrain ingestion tools in
[`python/tools/terrain/`](../../python/tools/terrain/) (design authority:
[`terrain.md`](terrain.md)) and on the aircraft configuration schema
([`aircraft.md`](../roadmap/aircraft.md)).

---

## Purpose

The live simulation requires a terrain dataset — a `.las_terrain` file and a companion
`.glb` export — covering a geographic area appropriate for one simulated flight.  Producing
that dataset currently requires calling each pipeline stage (download, mosaic, triangulate,
colorize, simplify, verify, export) individually, supplying bounding boxes, resolution
parameters, and LOD counts by hand.

The terrain build tool removes all manual steps.  Given only an aircraft configuration
JSON file, it derives every pipeline parameter automatically and produces a complete
terrain dataset with a single function call.

---

## Scope

This document specifies:

- The `build_terrain` function in `python/tools/terrain/build_terrain.py` — the single
  public entry point for terrain generation.
- The parameter derivation rules that convert an aircraft configuration into all
  pipeline parameters.
- The required extension to `python/tools/terrain/download.py`: a `resolution_deg`
  keyword argument on `download_dem` and `download_imagery`.
- The output layout produced by a successful build.

Out of scope: loading terrain at runtime (neither `SimRunner` nor Godot currently consume
a `TerrainMesh` for display — that is a separate future roadmap item), and modifications
to the C++ `TerrainMesh` class.

---

## Open Questions

| ID | Question | Status |
| --- | --- | --- |
| OQ-TB-1 | Is it feasible to download at native (L0) resolution for all aircraft classes, and how should large coverage areas be tiled? | **Resolved — see §OQ-TB-1 Analysis** |
| OQ-TB-2 | How should the terrain world origin and GLB path be communicated to the Godot scene without manual GUI interaction? | **Resolved — see §OQ-TB-2 Analysis** |
| OQ-TB-3 | How should LOD levels be exported to the Godot GLB so that the correct LOD renders at each camera distance without exceeding GPU VRAM? | **Resolved — see §OQ-TB-3 Analysis** |
| OQ-TB-4 | The current `triangulate.py` produces sliver triangles (small interior angles, high aspect ratios) in flat and coastal regions.  How should the triangulation algorithm be redesigned to guarantee mesh quality? | **Open — see §OQ-TB-4 Analysis** |
| OQ-TB-5 | How should surface imagery be bound to terrain geometry to achieve runway-visible detail without per-facet color aliasing? | **Resolved — see §OQ-TB-5 Addendum** |
| OQ-TB-6 | Continuous features (roads, runways) shift discontinuously across tile boundaries — the per-cell `arange` grid does not land on the cell edges, so adjacent tiles do not share boundary vertices. How should the tile grid be sampled so neighbours meet seamlessly? | **Resolved — see §OQ-TB-6 Analysis** |
| OQ-TB-7 | Continuous features still tear at tile seams after OQ-TB-6 — each tile's texture is rotated by ~the grid convergence angle because the projected (UTM) source is mapped onto the WGS84 tile grid without derotating grid-north to true-north. How should projected imagery be resampled onto the geographic tile grid so features stay registered across seams? | **Resolved — see §OQ-TB-7 Analysis** |

---

## OQ-TB-1 Analysis — Download Resolution Strategy

### Background

`download.py:download_dem` currently hardcodes `resx = resy = 0.000090` degrees/pixel
(≈ 10 m at mid-latitudes) in the Sentinel Hub API request.  The Sentinel Hub Process API
limits each request to 2,500 × 2,500 pixels, imposing a maximum chunk side of
`2500 × 0.000090 = 0.225 deg ≈ 25 km`.  For large coverage areas, the bounding box must
therefore be tiled into multiple chunks.  The question is whether this approach is
feasible for all three aircraft classes.

### All Aircraft Require Native (L0) Resolution

All aircraft classes operate at ground level during takeoff, landing, and taxi.  The
terrain LOD system selects L0 (10 m vertex spacing) for terrain within approximately
345 m slant range of the aircraft.  During takeoff roll and final approach, every
aircraft class is below 50 m AGL, placing the runway surface well within L0 slant range.

**The aircraft dependence is on coverage area (bbox extent), not on resolution.**  A jet
trainer needs to see more terrain at a given moment than a small UAS — so its terrain
dataset covers a larger geographic area.  But both aircraft need 10 m resolution terrain
beneath their wheels.  Scaling download resolution with cruise altitude would produce
terrain with visibly coarse geometry during ground operations for any aircraft class that
cruises above 500 m.

All terrain datasets are therefore downloaded at native L0 resolution (0.000090 deg/px).
All seven LOD levels (L0–L6) are built for every aircraft class by successive
simplification from the L0 source.

### Feasibility at Native Resolution

| Aircraft | Bbox extent | Chunks (DEM) | Chunks (imagery) | Total requests | Est. DEM volume |
| --- | --- | --- | --- | --- | --- |
| Small UAS | 24 km × 24 km | 1 × 1 | 1 × 1 | 2 | ~7 MB |
| General aviation | 66 km × 66 km | 3 × 3 | 3 × 3 | 18 | ~65 MB |
| Jet trainer | 180 km × 180 km | 8 × 8 | 8 × 8 | 128 | ~1.2 GB |

The jet trainer case — 128 requests and ~1.2 GB DEM — requires approximately 20–40
minutes at typical API throughput.  This is an acceptable one-time offline cost.  Each
downloaded chunk is cached at `python/cache/terrain/dem/<source>/<bbox_tag>.tif`; a
subsequent `build_terrain` call for the same location skips all downloads that are
already present.  Once a location dataset is built, it is reused for all simulations at
that location regardless of where within the coverage area the aircraft starts.

The 1.2 GB DEM cache is a build artefact on disk.  It is not loaded into RAM or GPU
memory at simulation runtime; only the processed `.las_terrain` and the Godot GLB are
used in-run.  In-run memory implications are analyzed in §OQ-TB-3.

### Recommendation — OQ-TB-1

**Always download at native (L0, 0.000090 deg/px) resolution.  Tile the bbox into
2,500 × 2,500-pixel chunks and cache each chunk individually.**

No change to `download.py` is required for the resolution parameter — the existing
default is correct.  The build tool tiles the bbox and calls `download_dem` once per
chunk.  OQ-TB-1 is resolved.

---

## OQ-TB-2 Analysis — World Origin Communication to Godot

### Context

`SimulationReceiver.gd` converts each received `SimulationFrame` geodetic position
(lat/lon/alt) to a Godot-space ENU offset.  This offset is computed relative to a
**world origin** — a fixed geodetic reference point that must correspond to the position
of the terrain mesh's geographic center in Godot space.

The terrain GLB exports vertices as ENU offsets from the tile centroid (see
[`terrain.md §Local Grid Vertex Encoding`](terrain.md#local-grid-vertex-encoding)).  When
the terrain mesh is placed at Godot world origin (0, 0, 0), its geographic center maps to
Godot position (0, 0, 0).  The aircraft must therefore also compute its position relative
to that same geographic center for the two to visually coincide.

The existing `SimulationReceiver.gd` already contains a mechanism for this:

```gdscript
# Set world origin to first received frame if not yet established.
if not world_origin_set:
    world_origin_lat_rad = lat_rad
    world_origin_lon_rad = lon_rad
    world_origin_h_m     = h_m
    world_origin_set     = true
```

The aircraft's first packet position is used as the world origin.

### Option A — Auto-origin from first packet (current behavior, no build tool change)

The first `SimulationFrame` packet establishes `world_origin`.  Terrain alignment
follows automatically provided three conditions hold:

1. The terrain GLB is placed at Godot position (0, 0, 0).
2. The terrain was built centered on the aircraft's initial position
   (`initial_state.latitude_rad` / `longitude_rad`).
3. The simulation starts from approximately the same position each session.

Condition 2 is guaranteed by `build_terrain`: it derives the terrain center from
`initial_state`, so `terrain_center ≡ aircraft_start_position`.  On the first frame,
`world_origin ← aircraft_start_position ≡ terrain_center`, and the aircraft appears at
Godot (0, 0, 0) — exactly coincident with the terrain geographic center.

**Implications:**

- Zero additional work: no sidecar files, no `download.py` changes, no GDScript changes.
- Works correctly for the primary use case.
- Fragile in edge cases:
  - If a developer places the terrain mesh at a non-zero Godot position, alignment breaks.
  - If the simulation is resumed from a checkpoint mid-flight, `world_origin` resets to
    that position rather than the terrain center, and the terrain appears offset.
  - If two terrain datasets are loaded in one scene (e.g., overlapping coverage), neither
    dataset's center is guaranteed to match world origin.
- The auto-origin mechanism is implicit; it is not expressed as a verified invariant
  anywhere in the system.

### Option B — Sidecar `terrain_config.json` at the dataset root

`build_terrain` writes `terrain_config.json` into the dataset root
(`data/terrain/<name>/terrain_config.json`).  The GLB is **not** copied;
`glb_path` is an absolute filesystem path pointing to the GLB already present in
the dataset:

```text
data/terrain/<name>/
    terrain_config.json          build metadata read at scene start
    derived/
        gltf/
            terrain.glb          GLB in-place — not copied to godot/
        las_terrain/
            terrain.las_terrain  physics collision data
```

```json
{
    "schema_version": 1,
    "dataset_name": "general_aviation",
    "glb_path": "C:/path/to/data/terrain/general_aviation/derived/gltf/terrain.glb",
    "las_terrain_path": "C:/path/to/data/terrain/general_aviation/derived/las_terrain/terrain.las_terrain",
    "world_origin_lat_rad": 1.0490,
    "world_origin_lon_rad": -2.0909,
    "world_origin_height_m": 610.0,
    "aircraft_mesh_path": "res://assets/aircraft_lp.glb",
    "aircraft_wingspan_m": 9.14
}
```

`live_sim.exe` receives the terrain config path via a `--terrain` flag and passes
it to Godot as a user argument (after `--`).  A `TerrainLoader.gd` autoload or
scene node runs `_ready()`:

1. Reads the terrain config path from Godot command-line user args
   (`OS.get_cmdline_user_args()` — args following `--` in the Godot launch command).
2. Opens `terrain_config.json` from that path.
3. Calls `ResourceLoader.load(glb_path)` to load the GLB as a `PackedScene`.
4. Instantiates it and adds it as a child of the `World` node — no drag-and-drop.
5. Iterates all `MeshInstance3D` children; parses the node name to extract LOD level;
   sets `visibility_range_begin` / `visibility_range_end` on each node (OQ-TB-3).

The developer's workflow after a terrain build is:

1. Run `build_terrain`.
2. Run `live_sim.exe --terrain data/terrain/<name>/terrain_config.json`.

No Godot editor interaction is required after the initial scene setup.

**Implications:**

- Terrain alignment is explicit and correct even if the simulation starts from a
  position other than `initial_state`.
- GLB is loaded at runtime via absolute path — no import step, no scene tree
  editing, no `.import` sidecar files to manage.
- Visibility ranges are set programmatically, not via the Inspector.
- Requires: `TerrainLoader.gd` (new node or autoload), a `--terrain` flag on
  `live_sim.exe`, and `build_terrain` writing the JSON file to the dataset root.
- `godot/terrain/` is no longer used and does not need to exist.
- `push_error()` on missing terrain config path argument or missing config file —
  no silent fallback.

### Option C — World origin embedded in the GLB, read via Godot GLTF extras

`export_gltf.py` already writes scene-level extras into the GLB:

```json
{
    "liteaerosim_terrain": true,
    "world_origin_lat_rad": ...,
    "world_origin_lon_rad": ...,
    "world_origin_height_m": ...
}
```

`SimulationReceiver.gd` could read these extras after loading the terrain GLB.

**Implications:**

- No sidecar file; world origin travels with the GLB.
- Godot 4's built-in GLB importer does not surface scene-level extras as node properties.
  Accessing them requires either: (a) a custom Godot import plugin registered via
  `EditorImportPlugin`, or (b) runtime loading via `GLTFDocument`/`GLTFState` (bypasses
  the asset importer entirely).  Both are significantly more complex than a file read.
- The `SimulationReceiver` node does not hold a reference to the terrain mesh node and
  would need to locate it by scanning the scene tree — fragile.
- Not recommended at this stage.

### Option D — `@export var` on `SimulationReceiver`, set once in the Inspector

`world_origin_lat_rad`, `world_origin_lon_rad`, `world_origin_h_m` become exported
variables.  The developer sets them in the Godot Inspector once after each terrain build.
Values are serialized into `World.tscn`.

**Implications:**

- No build tool change.  No GDScript file I/O.
- Requires a manual Inspector edit after every terrain build — breaks the automation goal.
- The values in `World.tscn` are committed to version control, which creates a coupling
  between the scene file and the terrain dataset on a specific developer's machine.

### Recommendation — OQ-TB-2

**Adopt Option B (revised).**

Option A is fragile (auto-origin from first UDP packet can silently offset terrain on
checkpoint resume) and requires manual drag-and-drop in the Godot editor every time the
terrain dataset changes.  Option D also requires manual Inspector edits.  Option C
requires a custom Godot import plugin and is significantly more complex.

Option B makes the terrain dataset self-describing: the sidecar JSON at the dataset root
contains the GLB path, las_terrain path, world origin, and dataset name.
`TerrainLoader.gd` reads the config path from Godot command-line user args at startup,
loads the GLB programmatically via absolute path, and applies visibility ranges — all
before the first frame.  `live_sim.exe` passes the terrain config path to Godot via
`--terrain`.  The developer workflow reduces to: build terrain, run `live_sim.exe
--terrain <path>`.

The original Option B copied the GLB to `godot/terrain/` and used a `res://terrain/`
Godot-relative path.  This is superseded: `glb_path` is now an absolute filesystem path
and no copy step is needed.  `godot/terrain/` is no longer used.  OQ-TB-2 is resolved.

---

## OQ-TB-3 Analysis — LOD Export and In-Run GPU Performance

### The Rendering Problem

`export_gltf.py` exports whichever `TerrainTileData` list it is given.  Without LOD
selection logic in Godot, every exported `MeshInstance3D` node renders every frame.
Different LOD levels cover the same geographic footprint at different vertex spacings,
so rendering them simultaneously produces z-fighting (per-pixel flickering where
coarser and finer surfaces intersect) and redundant vertex processing.

The correct solution is to export all LOD levels and configure Godot to render only the
appropriate LOD for each tile at each camera distance, using
`GeometryInstance3D.visibility_range_begin` / `visibility_range_end`.

The blocking constraint is GPU VRAM: Godot 4 uploads all scene geometry to the GPU at
load time, regardless of which nodes are currently visible.  Total VRAM consumption must
fit within the GPU budget.

### GPU VRAM Budget

The reference GPU is an **NVIDIA GTX 1050 with 4 GB VRAM**.  Reserving approximately
1.5 GB for render targets, shadow maps, textures, and Godot overhead, the practical
terrain geometry budget is **~2 GB**.

Each vertex entry in the uploaded buffer costs:

| Attribute | Size |
| --- | --- |
| Position (xyz, float32) | 12 B |
| Normal (xyz, float32) | 12 B |
| Per-facet color (RGB, padded) | 12 B |
| **Total per entry** | **36 B** |

Per-facet color requires three unique vertex entries per triangle (no sharing across
faces).  A tile with ~10,000 vertices in the `.las_terrain` representation expands to
approximately 30,000 GPU vertex entries, occupying **~1.1 MB of VRAM** per tile.

### VRAM If All LODs Exported for Full Coverage Area

From [`terrain.md §Level-of-Detail System`](terrain.md#level-of-detail-lod-system),
each LOD level tiles the full coverage area with cells of the sizes shown below.

| LOD | Cell side | Jet trainer cells | Jet trainer VRAM | GA cells | GA VRAM | UAS cells | UAS VRAM |
| --- | --- | --- | --- | --- | --- | --- | --- |
| L0 | 1 km | 32,400 | **35.0 GB** | 4,356 | 4.7 GB | 576 | 623 MB |
| L1 | 3 km | 3,600 | 3.9 GB | 484 | 523 MB | 64 | 69 MB |
| L2 | 10 km | 324 | 350 MB | 49 | 53 MB | 9 | 10 MB |
| L3 | 30 km | 36 | 39 MB | 5 | 5 MB | 1 | 1 MB |
| L4–L6 | 100 km | ~4 each | ~4 MB each | ~4 each | ~4 MB each | ~4 each | ~4 MB each |
| **Total** | | | **~39 GB** | | **~5.3 GB** | | **~715 MB** |

The jet trainer (39 GB) and general aviation (5.3 GB) both exceed the 4 GB VRAM budget
by a wide margin.  The jet trainer GLB itself would be on the order of 10–15 GB —
too large for `ResourceLoader.load()` to process at all.  The GPU driver would begin
evicting other resources to system RAM, causing severe frame stuttering.

Only the small UAS (715 MB) fits without modification.

### Geographic Partitioning

The solution is to limit which tiles are exported at each LOD level to those within the
maximum useful slant range for that level.  Tiles beyond this radius are never selected
by the visibility range system when the camera is anywhere near the dataset center, so
uploading them wastes VRAM with zero rendering benefit.

Maximum slant ranges are from
[`terrain.md §Slant-Range Selection Rule`](terrain.md#slant-range-selection-rule):

| LOD | `visibility_range_end` | Export radius | Max tiles at that LOD |
| --- | --- | --- | --- |
| L0 | 345 m | 345 m | ~1 tile (1 km cell side) |
| L1 | 1,035 m | 1,035 m | ~1 tile (3 km cell side) |
| L2 | 3,105 m | 3,105 m | ~1 tile (10 km cell side) |
| L3 | 9,315 m | 9,315 m | ~1 tile (30 km cell side) |
| L4 | 27,945 m | 27,945 m | ~1 tile (100 km cell side) |
| L5 | 83,835 m | 83,835 m | ~1–4 tiles |
| L6 | ∞ | full bbox | varies by aircraft class |

With geographic partitioning the VRAM cost is dominated by L5 and L6 (widest coverage)
and is essentially the same across all three aircraft classes:

| LOD | Max tiles exported | VRAM |
| --- | --- | --- |
| L0 | 1 | ~1 MB |
| L1 | 1 | ~1 MB |
| L2 | 1 | ~1 MB |
| L3 | 1 | ~1 MB |
| L4 | 1 | ~1 MB |
| L5 | ~4 | ~4 MB |
| L6 | ~4 (UAS/GA), ~4 (jet) | ~4 MB |
| **Total** | | **~13 MB** |

**13 MB is 0.3% of the 4 GB budget.**  All three aircraft classes fit comfortably on the
reference GPU with headroom for multiple simultaneous terrain datasets if needed.

**Limitation:** when the aircraft flies low over terrain that is far from the dataset
center, L0 and L1 tiles are absent for that distant location — those areas will render
at the next available LOD (L2 or coarser), showing visibly less detail up close.  For
the defined scenarios — jet trainer at Edwards AFB cruising at altitude, general aviation
patterns over Santa Barbara, small UAS low-and-slow over the coast — low-level operations away
from the home base are not primary use cases.  The C++ `.las_terrain` file retains full
L0 data everywhere for simulation fidelity regardless of what is exported to Godot.

### Option X3 — Godot-side LOD switching via `GeometryInstance3D.visibility_range_*`

Export all LOD levels with geographic partitioning applied.  Each tile node's
`MeshInstance3D` receives visibility range bounds matching its LOD slant-range band:

| LOD | `visibility_range_begin` | `visibility_range_end` |
| --- | --- | --- |
| L0 | 0 m | 345 m |
| L1 | 255 m | 1,035 m |
| L2 | 765 m | 3,105 m |
| L3 | 2,295 m | 9,315 m |
| L4 | 6,885 m | 27,945 m |
| L5 | 20,655 m | 83,835 m |
| L6 | 61,965 m | ∞ |

At any camera distance, Godot activates exactly the one tile whose range spans that
distance.  No z-fighting; per-frame vertex processing is limited to the single active
LOD per geographic cell; total VRAM ~13 MB.

Setting visibility ranges requires that tile node names encode LOD level (e.g.
`tile_L2_row3_col7`) — a required change to `export_gltf.py`.  The visibility range
values are then assigned at runtime by the terrain loader `_ready()` pass described
in OQ-TB-2 §Option B, requiring no Godot editor interaction.

### Recommendation — OQ-TB-3

**Adopt Option X3 with geographic partitioning.**

Geographic partitioning reduces VRAM from an infeasible 39 GB to 13 MB for the jet
trainer, bringing all aircraft classes well within the 4 GB GTX 1050 budget.  Godot's
`visibility_range_*` attributes eliminate z-fighting and drive per-frame vertex load to
the minimum appropriate set at every camera altitude — fine detail at low level, coarse
coverage at cruise altitude.

This uses the game engine's built-in LOD capability correctly, mirrors what the C++
`LodSelector` does for simulation queries, and requires no frame-rate-coupled LOD
management code.

Required changes:

1. **`build_terrain`**: filter exported tiles to the geographic partitioning radius per
   LOD level before passing to `export_gltf`.
2. **`export_gltf.py`**: embed LOD level and grid position in node names
   (e.g. `tile_L0_row0_col0`).
3. **Terrain loader `_ready()` pass**: read node names and set `visibility_range_begin`
   / `visibility_range_end` on each `MeshInstance3D` (see OQ-TB-2 §Option B).

OQ-TB-3 is resolved pending implementation of these three changes.

---

## Design Decisions

| Decision | Rationale |
| --- | --- |
| Single entry point, one required argument | The aircraft config encodes position and cruise speed — all quantities needed to derive the full set of pipeline parameters. No other required argument is needed. |
| Radius derived from initial velocity, not supplied explicitly | `initial_state.velocity_*_mps` fields define cruise speed; 10-minute radius follows deterministically. Users who need a different radius pass `radius_m` explicitly. |
| Native L0 resolution always used | All aircraft operate at ground level during takeoff and landing, requiring 10 m terrain resolution. Coverage area, not resolution, varies by aircraft class. |
| Chunked download for large areas | The Sentinel Hub Process API limits each request to 2,500 × 2,500 pixels (≈ 25 km at L0 resolution). Tiling large coverage areas into chunks ensures every request stays within this limit. |
| All LOD levels always built | Every LOD serves a different slant-range band in the C++ LOD selector. Building all levels from the L0 source costs only CPU time. Geographic partitioning limits which tiles are exported to Godot (OQ-TB-3). |
| Incremental rebuild via per-chunk cache | Each downloaded chunk is stored in `python/cache/terrain/` keyed by source and bbox. Re-running `build_terrain` on the same location skips all downloads already cached. Pass `force=True` to invalidate. |
| Raster sampling: whole-read DEM, windowed-read imagery | The build samples the DEM at every grid vertex and the imagery at every facet centroid, once per tile. The DEM is small, so it is read once into memory ([`RasterSampler`](../../python/tools/terrain/raster_sample.py)) and sampled by vectorized fancy-index — eliminating the per-point `rasterio.sample()` I/O that dominated the build. The imagery, however, can be multi-gigabyte (NAIP quarter-quads ≈ 0.5 GB each), so a whole-read would hold that array per process — with one copy per parallel worker it exhausts RAM (observed: `--workers 12` on a 34 GB machine → Windows commit-limit failure). Imagery is therefore sampled with [`WindowedRasterSampler`](../../python/tools/terrain/raster_sample.py): the dataset handle stays open and each `sample()` reads only the bounding pixel window of the tile's points, decimation-capped so a region-spanning coarse tile still reads a bounded block. Memory is bounded by the window, not the raster, so the build scales to large imagery and full worker parallelism, and to the several-hundred-km fast-jet coverage areas. |
| Analytic regular-grid triangulation, not general Delaunay | Every tile is a regular lat/lon grid, whose triangulation is the fixed two-triangles-per-cell pattern — computed analytically in pure numpy ([`triangulate._grid_triangulation`](../../python/tools/terrain/triangulate.py)), with consistent upward-facing winding. A general triangulator is unnecessary for a regular grid (it yields the same pattern), and, decisively, it kept `scipy`/OpenBLAS out of the build's parallel workers: `scipy.spatial.Delaunay`'s bundled OpenBLAS reserves per-thread scratch buffers sized to the CPU count, and one copy per worker exhausted the Windows commit limit at worker startup, before any tile data was touched. `scipy.spatial.Delaunay` is now imported lazily and only on the optional `boundary_points` (irregular-set stitching) path, which the build does not use. |
| Build parallelism: triangulation and texture render both over a process pool | The two per-tile stages — triangulate+colorize+check, and per-tile mosaic render — each run over a `ProcessPoolExecutor` (`--workers`, default all cores). Only small path/bbox tuples cross the process boundary; rasters are sampled from a process-local cache (DEM) or windowed reads (imagery). Both stages emit periodic progress (count / rate / ETA). Native math thread pools are capped to 1 (`OPENBLAS_NUM_THREADS` etc., set before any numeric import) so per-worker memory does not scale with core count. |
| Per-LOD texture resolution from the screen-space-error budget | Each tile's texture side is sized so a texel projects to ≈ 1 px at the closest range its LOD renders ([`lod_policy.lod_texture_max_px`](../../python/tools/terrain/lod_policy.py); [terrain_lod_rendering.md §4](terrain_lod_rendering.md)) — ≈ 200 px per tile, roughly constant across LODs, with the finest LOD using source-native capped at `near_field_cap`. This replaces the fixed per-LOD ceiling (`_LOD_MAX_PIXEL_DIM`, 2048–8192 px), which under the per-LOD footprints was ~20× too large for the coarse LODs (an L3 tile viewed from 16 km was rendered at 8192 px / ~0.8 m/px when ~400 px / ~17 m/px suffices), so 16 resident L3 tiles alone were ≈ 3 GB of VRAM. The SSE budget shrinks coarse-LOD textures by ~400× in area and cuts render time. |
| Dataset named from aircraft config basename | `general_aviation.json` → dataset name `general_aviation`. Terrain datasets represent locations, not aircraft states — moving the aircraft to the other end of the runway reuses the same dataset. An explicit `name` override is available. |
| Terrain dataset selected at launch via `--terrain` flag | `live_sim.exe --terrain <path>` passes the terrain config to Godot via command-line user args; `TerrainLoader.gd` reads the path from `OS.get_cmdline_user_args()` and loads the GLB via `ResourceLoader.load()`. No editor interaction required; switching datasets requires no rebuild. |

---

## Entry Point

**File:** `python/tools/terrain/build_terrain.py`

```python
def build_terrain(
    aircraft_config_path: Path | str,
    *,
    name: str | None = None,
    radius_m: float | None = None,
    dem_source: str = "copernicus_dem_glo30",
    imagery_source: str = "naip",
    force: bool = False,
) -> Path:
    """Build a complete terrain dataset for a live-sim aircraft configuration.

    Parameters
    ----------
    aircraft_config_path:
        Path to an aircraft configuration JSON file (see §Aircraft Config Integration).
        All spatial parameters are derived from this file.
    name:
        Dataset name used as the directory key under ``data/terrain/``.
        Defaults to the config file's stem (e.g. ``general_aviation`` from
        ``general_aviation.json``).
    radius_m:
        Half-side of the square bounding box centred on the aircraft's initial position,
        in metres.  Defaults to 10 minutes of flight at the aircraft's initial speed
        (``600 * |v_initial|``).
    dem_source:
        DEM data source.  One of ``"copernicus_dem_glo30"``, ``"nasadem"``, ``"srtm"``.
    imagery_source:
        Imagery source for colorisation.  One of ``"naip"`` (default),
        ``"sentinel2"``, ``"landsat9"``, ``"modis"``.
    force:
        If True, delete cached downloads and regenerate all derived files from scratch.

    Returns
    -------
    Path
        Absolute path to the exported GLB file
        (``data/terrain/<name>/derived/gltf/terrain.glb``).
    """
```

---

## Parameter Derivation

All parameters are derived deterministically from the aircraft configuration JSON.
No user interaction or intermediate decision is required.

### Center Position

Read directly from `initial_state`:

```python
center_lat_deg = math.degrees(config["initial_state"]["latitude_rad"])
center_lon_deg = math.degrees(config["initial_state"]["longitude_rad"])
```

### Cruise Speed and Default Radius

```python
vn = config["initial_state"]["velocity_north_mps"]
ve = config["initial_state"]["velocity_east_mps"]
vd = config["initial_state"]["velocity_down_mps"]
cruise_speed_mps = math.sqrt(vn**2 + ve**2 + vd**2)
radius_m = radius_m or (600.0 * cruise_speed_mps)   # 10 minutes
```

The half-side of the bounding box equals `radius_m`.  The full bbox is therefore
`2 × radius_m` in each horizontal dimension.

### LOD Levels Built

All seven LOD levels (L0–L6) are always built, regardless of aircraft type.  Every
aircraft class operates at ground level during takeoff and landing, so L0 (10 m vertex
spacing) is always required.  Coarser levels (L1–L6) are produced by successive
simplification from the L0 source at no additional download cost.

The `finest_lod` constant in `build_terrain` is therefore always 0.  The variable is
retained in the implementation so that the simplification loop `range(1, 7)` remains
parameterized and testable.

### Bounding Box

The bounding box is a square centred on the initial position.  The degree extents vary
with latitude because longitude degrees are shorter at higher latitudes:

```python
lat_deg_per_m = 1.0 / 111_132.0                          # approx, WGS84 mid-range
lon_deg_per_m = 1.0 / (111_132.0 * math.cos(math.radians(center_lat_deg)))

delta_lat = radius_m * lat_deg_per_m
delta_lon = radius_m * lon_deg_per_m

bbox_deg = (
    center_lon_deg - delta_lon,   # lon_min
    center_lat_deg - delta_lat,   # lat_min
    center_lon_deg + delta_lon,   # lon_max
    center_lat_deg + delta_lat,   # lat_max
)
```

---

## Download Strategy

### DEM — Chunk Size

The Sentinel Hub Process API limits each request to approximately 2,500 × 2,500 pixels.
At the native L0 resolution of 0.000090 deg/px, the maximum chunk side is
`2500 × 0.000090 = 0.225 deg ≈ 25 km`.

`build_terrain` computes the number of chunks needed in each dimension:

```python
RESOLUTION_DEG = 0.000090          # native L0 resolution, fixed
MAX_CHUNK_DEG  = 2500 * RESOLUTION_DEG   # = 0.225 deg ≈ 25 km
n_lon = math.ceil((bbox_deg[2] - bbox_deg[0]) / MAX_CHUNK_DEG)
n_lat = math.ceil((bbox_deg[3] - bbox_deg[1]) / MAX_CHUNK_DEG)
```

Each chunk bbox is passed to `download_dem(chunk_bbox)`.  All chunk paths are merged by
a single `mosaic_dem` call.

| Aircraft class | Bbox extent | Chunks (per axis) | Total DEM requests | Est. DEM volume |
| --- | --- | --- | --- | --- |
| Small UAS | 24 km × 24 km | 1 × 1 | 1 | ~7 MB |
| General aviation | 66 km × 66 km | 3 × 3 | 9 | ~65 MB |
| Jet trainer | 180 km × 180 km | 8 × 8 | 64 | ~1.2 GB |

### NAIP Imagery — Planetary Computer STAC API

NAIP (National Agriculture Imagery Program) provides ~0.6 m/pixel native-resolution
aerial orthophotos for the continental US.  It is the default `imagery_source` because
it resolves features the runway width (~45 m) that are invisible in Sentinel-2 (10 m/px).

**Data source:** Microsoft Planetary Computer STAC API.

```
https://planetarycomputer.microsoft.com/api/stac/v1/collections/naip/items
```

The TNM API previously used for NAIP is decommissioned and returns no results.  The
Planetary Computer API returns NAIP quarter-quad COG tiles; imagery assets are hosted
on Azure Blob Storage and are publicly accessible without SAS tokens.

**Tile discovery:** `download.py:_download_naip` queries the API with the terrain bbox,
follows pagination links (`rel='next'`), and sorts results by `-datetime` (newest first).
Feature IDs use the format `<state>_m_<quad>_<quadrant>_<zone>_<gsd>_<YYYYMMDD>`.

**Geographic deduplication:** The first four underscore-separated components form the
geographic quad key (`state_m_quad_quadrant`), which uniquely identifies the ground
footprint independent of GSD variant (060cm vs 100cm) and acquisition date.  Only the
most recent tile for each quad key is retained.  This prevents downloading both 0.6 m
and 1.0 m captures of the same area.

**Download method — full-tile streaming:** NAIP quarter-quads are COGs (~13 km × 11 km,
~300 MB per tile).  COG windowed reads (HTTP range requests) issue thousands of
round-trips for any terrain-scale window, taking 25 minutes or more per tile.  The
implementation streams each tile in its entirety as a single HTTP response
(`requests.get(stream=True)`) into a temp file, then renames it to the cache path.
This reduces per-tile download time to under 10 minutes on typical consumer broadband.

**Temp file cleanup:** The temp file is deleted on any exception, including
`KeyboardInterrupt` (caught as `BaseException`), so a cancelled build does not
accumulate orphaned files in the cache directory.

**Cache key — feature ID only:** Each NAIP tile is cached as
`python/cache/terrain/imagery/naip/<feature_id>.tif`.  The cache key contains no bbox
component so the same downloaded tile is reused for any terrain configuration whose
bbox intersects that tile's footprint.

**NAIP → `render_mosaic` bypass of `mosaic_imagery`:** `mosaic_imagery` computes pixel
resolution from the source raster's `transform.a` field (UTM pixel size in metres, e.g.
0.6) and passes it as `resolution` to `calculate_default_transform` targeting EPSG:4326.
This misinterprets metres as degrees, producing a 1×1-pixel output.  For NAIP tiles,
`build_terrain` therefore skips `mosaic_imagery` entirely and passes the per-tile paths
directly to `render_mosaic` as `(path, "naip")` pairs.

**Projected-source resampling — full reprojection, not a linear window stretch:** NAIP is
delivered in UTM (e.g. EPSG:26911).  When resampling a UTM source onto a tile's geographic
(EPSG:4326) grid, `render_mosaic._read_source_rgb` performs a true reprojection
(`rasterio.warp.reproject` on a bounded source window) — it does **not** read an
axis-aligned UTM window and linearly stretch it onto the tile.  A linear stretch treats
UTM grid-north as if it were true north, so every tile is rotated by the local grid
convergence angle γ (≈ (λ − λ₀)·sin φ, ≈ 1.6° at KSBA) about its own centre; neighbouring
tiles rotate about different centres and continuous features tear at the seam.  The
reprojection derotates grid-north to true north per pixel, keeping features registered
across seams.  The read is **decimated on read** (`out_shape`-capped to ~2× the tile's
output grid), so a region-spanning coarse tile reads a bounded block rather than the full
native window (tens of thousands of px per side ≈ multiple GB) — preserving the per-tile
memory bound from the sampling design (§Design Decisions).  EPSG:4326 sources (Sentinel-2,
Landsat) need no reprojection — their window maps linearly to the tile grid — and take the
direct windowed-read path.  See §OQ-TB-7.

### Sentinel-2 and Landsat Imagery

Sentinel-2 and Landsat 9 continue to use the existing chunked download from Sentinel Hub
with the same chunk dimensions as the DEM:

| Aircraft class | Bbox extent | Chunks (per axis) | Total imagery requests |
| --- | --- | --- | --- |
| Small UAS | 24 km × 24 km | 1 × 1 | 1 |
| General aviation | 66 km × 66 km | 3 × 3 | 9 |
| Jet trainer | 180 km × 180 km | 8 × 8 | 64 |

Each chunk is merged by `mosaic_imagery` before being passed to `render_mosaic`.

### Cache Layout

DEM chunks and non-NAIP imagery chunks are stored at:

```
python/cache/terrain/dem/<source>/<bbox_tag>.tif
python/cache/terrain/imagery/<source>/<bbox_tag>.tif
```

NAIP tiles are stored at:

```
python/cache/terrain/imagery/naip/<feature_id>.tif
```

If the cache file already exists the download is skipped.  `force=True` deletes all
cached files for the dataset before beginning.

---

## Pipeline Orchestration

`build_terrain` drives the existing pipeline tools in sequence.  All intermediate files
are written under `data/terrain/<name>/`.

```text
aircraft_config_path
    │
    ├─ derive: center_lat, center_lon, cruise_speed
    ├─ compute: radius_m, bbox_deg, n_chunks
    │
    ▼
[1] download_dem(chunk_bbox)                ×n_chunks   (native L0 resolution)
    download_naip(bbox_deg)                 (NAIP: full tile per quarter-quad)
       — or —
    download_imagery(chunk_bbox)            ×n_chunks   (Sentinel-2, Landsat)
    │
    ▼
[2] mosaic_dem(dem_chunks → source/dem_mosaic.tif)
    [NAIP: skip mosaic_imagery; pass tile paths directly to render_mosaic]
    [Sentinel-2/Landsat: mosaic_imagery(img_chunks → source/img_mosaic_<src>.tif)]
    │
    ▼
[3] triangulate(dem_mosaic, bbox_deg, lod=0)
    → L0 tile (TerrainTileData)
    │
    ▼
[4] colorize(tile, img_mosaic, source=imagery_source)   [advisory; .las_terrain colors]
    → L0 tile with per-facet RGB
    │
    ▼
[5] for target_lod in range(1, 7):
        tile = simplify(prev_tile, target_lod)
        tile = colorize(tile, img_mosaic, source=imagery_source)
        check(tile)
    → tiles[0..6]
    │
    ▼
[6] write_las_terrain(las_terrain_dir / "terrain.las_terrain", all_tiles)
    │
    ▼
[7] display_tiles = geographic_partition(all_tiles)      (see OQ-TB-3)
    for tile in display_tiles:
        tile_mosaic = render_mosaic(
            tile_bbox_deg, img_mosaic_paths,
            max_pixel_dim=_LOD_MAX_PIXEL_DIM[tile.lod],
        )
        tile_mosaics.append((tile, tile_mosaic))
    export_gltf(tile_mosaics, gltf_path)                 (see §OQ-TB-5 Addendum)
    │
    ▼
return gltf_path
```

Steps [1]–[7] correspond to the stages defined in
[terrain-implementation-plan.md](../roadmap/terrain-implementation-plan.md) steps 15–21.

### Progress Reporting

`build_terrain` logs each step to `logging.getLogger("build_terrain")` at `INFO` level:

```
[build_terrain] general_aviation: center 34.426°N 119.840°W  radius 33.0 km  9 chunks
[build_terrain] general_aviation: downloading DEM  1/9 chunks ...
...
[build_terrain] general_aviation: downloading imagery  1/9 chunks ...
...
[build_terrain] general_aviation: mosaicking ...
[build_terrain] general_aviation: triangulating LOD 0 ...
[build_terrain] general_aviation: colorizing LOD 0 ...
[build_terrain] general_aviation: simplifying LOD 1 ...
...
[build_terrain] general_aviation: exporting GLB → data/terrain/general_aviation/derived/gltf/terrain.glb
```

---

## Output Layout

A completed build produces the following directory tree under `data/terrain/<name>/`:

```text
data/terrain/<name>/
    terrain_config.json         world origin, GLB and las_terrain paths, aircraft mesh
    source/
        dem_mosaic.tif          seamless DEM GeoTIFF (native L0 resolution)
        img_mosaic.tif          seamless imagery GeoTIFF (same extent and resolution)
    derived/
        las_terrain/
            terrain.las_terrain all LODs in a single .las_terrain binary
        gltf/
            terrain.glb         all LODs as a single GLB for Godot import
        metadata.json           build parameters for provenance and reproducibility
```

### `metadata.json` Schema

```json
{
    "schema_version": 1,
    "dataset_name": "general_aviation",
    "aircraft_config": "python/assets/aircraft_configs/general_aviation.json",
    "center_lat_deg": 34.426,
    "center_lon_deg": -119.840,
    "radius_m": 33000.0,
    "lod_range": [0, 6],
    "dem_source": "copernicus_dem_glo30",
    "imagery_source": "sentinel2",
    "dem_resolution_deg": 0.000090,
    "bbox_deg": [-120.200, 34.129, -119.480, 34.723],
    "build_timestamp_utc": "2026-04-07T12:00:00Z"
}
```

This file is written last, after all derived files are verified, so its presence indicates
a complete and successful build.

---

## Error Handling

| Condition | Behaviour |
| --- | --- |
| Aircraft config not found or invalid JSON | Raise `FileNotFoundError` / `json.JSONDecodeError` immediately, before any network I/O |
| `terrain.radius_km` absent and `--radius-km` not supplied | Raise `ValueError` immediately, before any network I/O |
| Download authentication failure | `DownloadError` (from `download.py`) propagates; no partial output is written |
| Degenerate facets (zero-area triangles) | `MeshQualityError` raised; the `.las_terrain` file is not written |
| Sliver triangles (small angle or high aspect ratio) | `WARNING` logged (see OQ-TB-4); build continues; `.las_terrain` is written |
| Any step raises after mosaicking | Intermediate rasters in `source/` are retained so a subsequent run can skip the download step |
| `BLOCKXSIZE can only be used with TILED=YES` (GDAL) | Benign.  The Sentinel-2 source files carry a `BLOCKXSIZE` creation option in their rasterio profile.  `mosaic.py` strips this key before writing the untiled mosaic output; the warning is suppressed. |
| `TIFFReadDirectory: Sum of Photometric type-related color channels…` (libtiff) | Benign, expected on every run with Sentinel-2 imagery.  The warning originates inside the downloaded provider files — a tag inconsistency in the provider's TIFF encoding.  It does not affect the data read and cannot be suppressed from our code. |

---

## Integration with Live Simulation

After `build_terrain` completes, the developer runs `live_sim.exe --terrain
<path-to-terrain_config.json>`, which launches Godot passing the terrain config
path as a user argument.  No editor interaction is required.

`build_terrain` is responsible for:

- Writing `data/terrain/<name>/terrain_config.json` with world origin, GLB path,
  las_terrain path, and aircraft mesh path.  The GLB is **not** copied; `glb_path`
  is an absolute filesystem path pointing to
  `data/terrain/<name>/derived/gltf/terrain.glb`.

### `terrain_config.json` schema

```json
{
    "schema_version": 1,
    "dataset_name":          "<string>",
    "glb_path":              "<absolute path to data/terrain/<name>/derived/gltf/terrain.glb>",
    "las_terrain_path":      "<absolute path to data/terrain/<name>/derived/las_terrain/terrain.las_terrain>",
    "world_origin_lat_rad":  "<double>",
    "world_origin_lon_rad":  "<double>",
    "world_origin_height_m": "<double>",
    "aircraft_mesh_path":    "res://assets/aircraft_lp.glb",
    "aircraft_wingspan_m":   "<double>"
}
```

| Field | Source | Notes |
| --- | --- | --- |
| `schema_version` | Constant `1` | |
| `dataset_name` | Aircraft config basename or `--name` override | |
| `glb_path` | Absolute path to `data/terrain/<name>/derived/gltf/terrain.glb` | Written by `build_terrain`; must not be edited by hand |
| `las_terrain_path` | Absolute path to `data/terrain/<name>/derived/las_terrain/terrain.las_terrain` | Written by `build_terrain`; must not be edited by hand |
| `world_origin_lat_rad` | Derived from aircraft config `initial_state.latitude_rad` or `--center-lat` | |
| `world_origin_lon_rad` | Derived from aircraft config `initial_state.longitude_rad` or `--center-lon` | |
| `world_origin_height_m` | Derived from aircraft config `initial_state.altitude_m` | WGS84 ellipsoidal height in metres |
| `aircraft_mesh_path` | Aircraft config `visualization.mesh_res_path`; default `"res://assets/aircraft_lp.glb"` | May be edited by hand to swap mesh without terrain rebuild |
| `aircraft_wingspan_m` | `sqrt(S_ref_m2 * ar)` from aircraft config | Used by TerrainLoader for mesh scale |

`aircraft_mesh_path` is the only field safe to edit by hand after a build.  All
other fields must match the terrain dataset and must not be changed without a
terrain rebuild.

`live_sim.exe` passes the terrain config path to Godot:

```text
live_sim.exe --terrain data/terrain/<name>/terrain_config.json
```

Internally, `live_sim.exe` launches Godot with:

```text
godot4 -- --terrain <absolute-path-to-terrain_config.json>
```

`TerrainLoader.gd` runs at scene start and is responsible for:

- Reading the terrain config path from Godot command-line user args
  (`OS.get_cmdline_user_args()` — args after `--` in the Godot launch command).
- Loading the terrain GLB from `glb_path` via `ResourceLoader.load()`.
- Instantiating it as a child of the `World` node.
- Iterating `MeshInstance3D` children; parsing node names to extract LOD; setting
  `visibility_range_begin` / `visibility_range_end` on each node.
- Loading the aircraft mesh from `aircraft_mesh_path`; instantiating it as a child
  of the `Vehicle` node with `rotation_degrees = Vector3(0, 180, 0)` applied.

`godot/terrain/` is not used by this pipeline and does not need to exist.

The `.las_terrain` file will be consumed by `TerrainMesh::deserializeLasTerrain()` once
the `Aircraft` / `SimRunner` terrain integration is implemented (a separate future item).

---

## OQ-TB-4 Analysis — Triangulation Mesh Quality

### Observed Problem

During initial operation of the terrain build pipeline, `verify.py` reports sliver
triangles in flat and coastal terrain cells — minimum interior angles below 10° and
aspect ratios above 15.  These violations are currently downgraded to warnings so the
build continues, but the underlying cause must be corrected.

### Root Cause

`triangulate.py` performs **naive 2.5D triangulation**: it samples the DEM raster on a
regular grid, then connects adjacent sample points using a fixed connectivity pattern
(quad-split into two triangles per grid cell).  The triangle connectivity is therefore
inherited directly from the raster grid topology.

In flat terrain — where adjacent sample points have nearly identical elevation — three
grid-adjacent vertices can be nearly collinear in 3D space even though they are
well-separated in the horizontal plane.  The resulting triangles are geometrically valid
(non-zero area) but degenerate in shape.  Coastal tiles are a common trigger because the
ocean surface is perfectly flat, but the same pathology can appear anywhere the terrain
is level over a sufficiently large area.

### Why This Is a Design Issue

A proper surface triangulation should optimise triangle quality independently of the
source data topology.  The correct approach is:

1. **Sample** the DEM to obtain a set of 3D point positions (this step is unchanged).
2. **Triangulate** the point set as a surface problem using 2D Delaunay on the XY
   projection — producing connectivity that maximizes minimum interior angle regardless
   of the original raster grid layout.
3. **Lift** the 2D Delaunay simplices back to 3D by restoring each vertex's Z (elevation)
   value sampled from the DEM.
4. Optionally apply a **quality-enforcement pass** (vertex insertion via Ruppert's
   algorithm, or constrained Delaunay with a minimum-angle argument) if the baseline
   Delaunay result still contains triangles below the `verify.py` thresholds.

The fix belongs entirely inside `triangulate.py`.  No changes to `verify.py`, the
pipeline orchestration, or the LOD scheme are required.

> **Current-implementation note (2026-07).** The mechanism has since changed: `triangulate.py`
> now emits the quad-split **analytically** ([`_grid_triangulation`](../../python/tools/terrain/triangulate.py),
> pure numpy, consistent upward winding) rather than via `scipy.spatial.Delaunay` — see the
> §Design Decisions row on analytic triangulation. This does **not** resolve OQ-TB-4: for a *regular*
> grid, 2D Delaunay produces the same two-triangles-per-cell connectivity, so the "use 2D Delaunay"
> step above does not by itself improve triangle quality. A genuine quality fix requires either a
> minimum-angle refinement (Triangle.c `"pq25"`) on an intentionally *irregular* / adaptively-sampled
> point set, or a proper 3D surface remesh (the deeper 2.5D-heightmap limitation). OQ-TB-4 remains
> **open** on that basis. The per-LOD footprint tiling now uses roughly square cells, which keeps the
> quad-split triangles near 45°-45°-90° and makes slivers rare in practice.

### Available Library Support — trimesh 4.11.3

The project already depends on trimesh (v4.11.3).  The following capabilities are
relevant to the fix.

#### What trimesh provides

| Function | Module | Notes |
| --- | --- | --- |
| `triangulate_polygon(polygon, triangle_args, engine)` | `trimesh.creation` | Triangulates a 2D Shapely polygon using a selectable engine.  With `engine="triangle"` (Shewchuk's Triangle.c), the `triangle_args` string is passed directly to Triangle — e.g. `"pq25"` enforces a 25° minimum interior angle via Ruppert refinement. |
| `mesh.face_angles` | `trimesh.Trimesh` | Returns `(F, 3)` array of interior angles in radians — useful for post-triangulation verification. |
| `mesh.nondegenerate_faces(height)` | `trimesh.Trimesh` | Boolean mask of non-degenerate faces; can be used to remove any residual zero-area triangles after lifting. |
| `trimesh.smoothing.filter_laplacian(mesh, lamb, iterations)` | `trimesh.smoothing` | Laplacian vertex relocation.  Not a quality triangulator on its own, but can be applied after Delaunay lifting to improve regularity of the vertex distribution in near-flat regions. |
| `trimesh.triangles.angles(triangles)` | `trimesh.triangles` | Low-level angle computation on `(N, 3, 3)` triangle arrays; equivalent to `mesh.face_angles` but usable without constructing a full `Trimesh` object. |

#### What trimesh does NOT provide

- **Native Delaunay triangulation from a point cloud** — trimesh has no direct API for
  this.  The recommended path is to use `scipy.spatial.Delaunay(points[:, :2])` to
  obtain 2D simplices, then lift to 3D by indexing the Z values.
- **Edge flipping or edge collapse** — trimesh's `remesh` module provides only
  subdivision variants (`subdivide`, `subdivide_to_size`, `subdivide_loop`) which
  split triangles without re-optimizing angles.
- **Aspect-ratio-targeted refinement** — the Triangle.c engine (via `triangulate_polygon`)
  accepts a maximum-area argument (`"pa{area}"`) but not a direct aspect-ratio bound.
  The minimum-angle argument (`"pq{deg}"`) is the correct lever; enforcing ≥ 25° is
  sufficient to satisfy the `verify.py` thresholds.

#### Recommended implementation path

```python
# Step 1 — sample DEM on LOD grid (unchanged from current code)
xy = sample_dem_grid(dem_path, cell_bbox, lod)        # (N, 2) float, degrees
z  = query_dem_elevation(dem_path, xy)                 # (N,)   float, metres

# Step 2 — 2D Delaunay on horizontal projection
import scipy.spatial
tri = scipy.spatial.Delaunay(xy)                       # simplices: (F, 3) int

# Step 3 — lift to 3D ENU and construct TerrainTileData
vertices_3d = np.column_stack([xy_to_enu(xy), z])     # (N, 3) float32 ENU
indices     = tri.simplices.astype(np.uint32)          # (F, 3)

# Step 4 — optional: enforce minimum angle via Triangle.c
# If scipy Delaunay alone does not meet quality thresholds:
#   import trimesh.creation
#   verts_2d, faces = trimesh.creation.triangulate_polygon(
#       boundary_polygon, triangle_args="pq25", engine="triangle")
#   # then lift verts_2d back to 3D via DEM query
```

The Triangle.c engine (`triangle_args="pq25"`) inserts additional Steiner points where
needed to enforce the angle bound — this produces more vertices than the input DEM grid
but guarantees quality.  The current grid-sampling step must be replaced with a
boundary-polygon construction step when this path is taken.

### Current Workaround

`verify.py:check()` logs a `WARNING` for angle and aspect-ratio violations and returns
without raising `MeshQualityError`.  The mesh is usable for display purposes; the
quality deficiency does not affect simulation correctness because terrain is currently
display-only.

### Resolution Criterion

OQ-TB-4 is resolved when `triangulate.py` is redesigned so that `verify.py:check()`
produces no warnings on any real DEM tile across the KSBA test datasets.

---

## OQ-TB-5 Analysis — Imagery Mapping: Per-Facet Color → Texture-Based Surface

### Imagery Resolution Loss in the Per-Facet Pipeline

Surface imagery is currently transferred to the rendered terrain as a single uint8
RGB triplet per triangle, computed by [colorize.py](../../python/tools/terrain/colorize.py)
sampling the imagery raster at exactly one point — the facet centroid — and stored
in the GLB as duplicated `COLOR_0` vertex attributes.

Three failure modes follow from this:

1. **Resolution loss.** Imagery (Sentinel-2 at 10 m/pixel; Landsat 9 at 30 m/pixel) is
   sampled once per facet.  At LOD 0 a typical facet is 50–100 m² of irregular shape;
   at LOD 4 it is hundreds of thousands of m².  All imagery pixels not at the facet
   centroid are discarded.
2. **Feature aliasing.** Linear high-contrast features (runways, roads, shorelines)
   narrower than ~2× the local mesh edge length reduce to a speckle pattern of
   facets that happen to fall on the feature versus those that don't.  The KSBA
   runway (45 m wide) is invisible at LOD 0 in practice.
3. **Flat shading per face.** No interpolation across the triangle, so even a feature
   well-resolved by the mesh tessellation appears as a hard polygon edge rather than a
   smooth surface.

### Mesh-Imagery Coupling — The Underlying Cause

The pipeline conflates two independent quantities — geometric mesh resolution and
imagery sample resolution — and locks them to a single per-face value.  Standard
real-time terrain rendering decouples the two: **mesh geometry** controls silhouette,
self-shadowing, and AGL collision (LOD-driven tessellation), while **surface imagery**
is delivered as a 2D texture and sampled by the GPU's fragment shader at full imagery
resolution regardless of triangle size.

This decoupling is the standard glTF 2.0 pattern: `MeshPrimitive` carries `POSITION`,
`NORMAL`, and `TEXCOORD_0`; the `material.pbrMetallicRoughness.baseColorTexture`
references a JPEG / PNG payload embedded in the GLB.  No per-vertex color is used.

### Use Case Constraints

The texturing approach must serve three distinct simulation profiles, each with
different coverage and resolution requirements:

| Profile | Typical aircraft | Coverage radius | Min imagery resolution |
| --- | --- | --- | --- |
| **GA / training** | Cessna, small UAS | 5–15 km | ~5 m/pixel (runway markings) |
| **Jet trainer** | T-38, light jet | 30–50 km | ~10 m/pixel |
| **Commercial / high-altitude** | Airliner, business jet | 100–300 km | ~30 m/pixel |

The first profile (where this question originated — KSBA UAS) is the most
demanding per-pixel: low altitude, short distances to surface features, runway
detail must be visible.  Higher-altitude profiles tolerate coarser imagery because
viewing distance compresses pixel detail anyway.

### Hardware Constraints

The viewer is Godot 4.6 + Vulkan running on the operator's desktop GPU.  Concrete
limits that bound the design:

| Constraint | Value | Source |
| --- | --- | --- |
| GPU max 2D texture size | 16 384 × 16 384 (~268 MP) | Vulkan minimum spec; most desktop GPUs since 2014 |
| GPU max 2D texture size (low-end / mobile / iGPU) | 4096 × 4096 | Older Intel iGPUs, some integrated AMD |
| Target VRAM budget | 4–8 GB total scene | Per OQ-TB-3; aircraft mesh + terrain mesh + textures + Godot scene state |
| `Texture2DArray` max layers | 2048 | Vulkan minimum spec |
| GLB maximum file size | Practically ~2 GB | glTF 2.0 spec uses uint32 byte offsets |
| Native imagery resolution | 10 m (Sentinel-2), 30 m (Landsat 9), 500 m (MODIS) | Source-determined; cannot be exceeded without super-resolution |

Two of these dominate: GPU max texture size limits how large a single mosaic can be,
and native imagery resolution limits how much detail any approach can deliver.  At
Sentinel-2's 10 m/pixel, a 16 384² texture covers `163 km × 163 km` — comfortably more
than any single-airport build needs.

### Imagery-Source Limits

Sentinel-2 returns 10 m/pixel native.  No matter which texture-binding strategy we
choose, that's the ceiling on usable detail.  Storing higher-resolution textures
(say, 0.5 m/pixel via bilinear up-sampling) only makes sense when the GPU's bilinear
fragment shader has detail to interpolate; for a flat 10 m source pixel, an oversize
texture wastes VRAM without adding sharpness.  The only reason to oversample is to
preserve edge sharpness through mipmap chains and anisotropic filtering at oblique
viewing angles — a second-order benefit.

### Alternatives Considered

Nine candidates, ordered roughly from least to most ambitious.

#### Option A — Per-facet vertex color (status quo)

Each triangle carries one RGB triplet via duplicated `COLOR_0` vertex attributes.
Already in production; analyzed at length above.

- **VRAM:** ~0 dedicated texture; duplicated vertices add ~30% to geometry.
- **Disk:** Smallest GLB.
- **Quality:** Speckle aliasing on linear features; flat-shaded faces; runway
  invisible.
- **Verdict:** Rejected.

#### Option B — Per-vertex color, GPU-interpolated

Sample imagery at each vertex (not facet centroid); store per-vertex colors; GPU's
rasterizer interpolates RGB linearly across the triangle.

- **VRAM:** Same as A.
- **Quality:** No flat-face artifacts; smoother surface.  Imagery sampling still
  bounded by mesh resolution — runway still under-resolved at LOD 0 (10 m vertex
  spacing) since the runway feature is only 45 m wide and centered between vertex
  positions much of the time.
- **Verdict:** Marginal improvement, doesn't solve the fundamental problem.
  Rejected.

#### Option C — Per-tile JPEG textures

One texture per terrain tile, embedded in the GLB.  Tile UVs derived from each
vertex's geodetic position normalized to the tile bounds.

- **VRAM (KSBA 12 km, ~144 L0 tiles at 2048²):** ~2.3 GB.  Much of this is
  bilinear up-sampling of native 10 m/pixel imagery to 0.5 m/pixel — minimal
  display benefit at significant VRAM cost.
- **Disk (GLB):** ~200 MB of JPEGs (1–2 MB × 144).  Loading triggers ~144 GPU
  texture uploads at scene start — measurable startup stutter.
- **Quality issues:**
  - **Tile seams.** Bilinear filtering cannot sample across a tile boundary.
    Without 1–2 px of imagery padding, seams are visible.  Padding adds 5–10%
    to texture data and complicates the build (each tile must be rendered with
    overlap into neighbor tiles).
  - **Independent mipmap chains.** Each tile generates mipmaps in isolation.
    Adjacent tiles' coarse mip levels do not match — a far-distance LOD switch
    can show a visible discontinuity.
- **Draw calls / state:** ~144 texture binds per frame; not a bottleneck on
  modern GPUs but real overhead.
- **Implementation cost:** High — per-tile texture render, per-tile glTF
  material binding, padding logic, separate Godot material per tile.
- **Verdict:** Originally proposed; rejected after performance analysis.

#### Option D — Single mosaic texture per coverage area (recommended for typical builds)

One texture covers the entire dataset's bounding box at imagery-native pixel
density.  Every tile shares the same texture; UVs are computed per vertex from the
vertex's geodetic position normalized to the dataset bounds.

- **VRAM (KSBA 12 km, 24 km bounding box at 10 m/pixel):** 2400 × 2400 ≈ 6 MP.
  Padded to 4096² = 16 MP.  RGBA8 = 64 MB; with mipmap chain (33% extra) ≈ 85 MB.
  An order of magnitude less than Option C.
- **Disk (GLB):** Single JPEG q92 ≈ 2–3 MB.  Loading: one texture upload, ~10 ms.
- **Quality:**
  - **No seams.** All filtering is internal to a single texture.  Bilinear,
    anisotropic, and the full mipmap chain work as designed everywhere.
  - **Native resolution preserved.** No per-tile resolution mismatch — every
    tile sees the same texture under the same mipmap chain, so distant tiles
    automatically use coarser mip levels via Godot's screen-space derivative
    sampling.
  - **No PoT-padding waste.** Texture sized to actual imagery extent (rounded
    up to the nearest power of two for mipmap support).
- **Draw calls / state:** One texture bound for all terrain rendering.  Per-tile
  draw calls remain (geometry is still per-tile for LOD culling) but no per-call
  texture state change.
- **LOD interaction:** Mesh LOD switching (per OQ-TB-3) operates on geometry
  alone.  All mesh LODs reference the same mosaic texture; the GPU's mipmap
  selection chooses the appropriate detail level per fragment.
- **Implementation cost:** Lower than Option C (no padding, one material, simpler
  glTF) and lower than the status quo's vertex-duplication path.
- **Coverage limit:** Fits within GPU max texture size for areas up to ~163 km
  at 10 m/pixel (Sentinel-2) or ~491 km at 30 m/pixel (Landsat 9).  Comfortably
  covers all GA and most jet-trainer profiles.
- **Verdict:** **Recommended baseline for all builds within the coverage limit.**

#### Option E — Multi-mosaic regional textures (extension for very large coverage)

For coverage exceeding the single-mosaic limit (> ~50 km radius at Sentinel-2 native
resolution, with safety margin for low-end GPUs at 4096² limit), partition the
dataset into geographic regions, one mosaic per region, sized within the lowest
common denominator GPU limit.

- **VRAM:** Sum of regional mosaic VRAMs.  E.g., 200 km × 200 km coverage at
  Sentinel-2 = 4 mosaics of 80 MB each = 320 MB.  Linear scaling, no per-tile
  multiplier.
- **Disk:** Linear in coverage area.
- **Quality:** Seams at region boundaries (~ few times per dataset, not per
  tile).  Manageable with the same overlap-padding technique that would be
  required for Option C, applied at regional rather than tile scale.
- **Implementation cost:** Slightly above Option D — region partitioning logic
  in the build pipeline, region-to-tile assignment, multiple materials in GLB.
- **Verdict:** **Recommended extension when coverage exceeds the single-mosaic
  limit.**  Defer until the use case appears (no current jet-trainer / airliner
  build).

#### Option F — Texture array (`Texture2DArray`)

All tile textures packed as slices in a single `Texture2DArray` resource; shader
indexes via per-vertex slice ID attribute.

- **VRAM:** Same as Option C (one slice per tile).
- **Quality:** Same per-tile seam problem as Option C.
- **State:** Single binding for all terrain.
- **Layer limit:** Vulkan minimum is 2048; KSBA's 144 tiles are fine, but a
  large jet-trainer build can exceed 2048 tiles at fine LODs.
- **glTF support:** Texture arrays are an extension (`MSFT_texture_dds` /
  custom), not standard glTF 2.0.  Godot 4 supports them at the engine level
  but the import path requires a custom converter.
- **Verdict:** Same quality issues as Option C with comparable VRAM.  The only
  win is texture state batching, which Option D already achieves natively.
  Rejected.

#### Option G — Atlas (single texture, sub-region per tile)

All tiles packed into a single texture as sub-rectangles (UV mapping per tile
references the corresponding sub-region).  Effectively a manual texture array.

- **VRAM:** Slightly less than Option F (no slice padding to power of two).
- **Quality:** Mipmap bleeding between adjacent sub-regions — a coarse mip
  level samples across atlas boundaries, mixing one tile's pixels into another.
  Mitigated by inter-region padding (typical: 8 px guard band per side per LOD
  level).  Padding overhead grows with mipmap depth.
- **Verdict:** Equivalent to Option D from a quality standpoint but with
  significant atlas-management complexity.  No advantage.  Rejected.

#### Option H — Triplanar / procedural with imagery as parameters

Sample imagery as sparse data points; reconstruct a continuous coloration via a
shader (e.g., triplanar projection from three orthogonal planes, or noise-based
ground texture modulated by imagery).

- **Verdict:** Appropriate for procedurally-generated planetary terrain where
  no high-quality source raster exists.  We have a high-quality raster.
  Discarded.

#### Option I — Virtual / sparse texturing (page-based streaming)

A single very large logical texture, with only the visible portion (pages of
typically 128² each) resident in VRAM at any time; pages are loaded on demand
based on the camera frustum.

- **Quality:** Equivalent to Option D, no seams, single shader binding.
- **VRAM:** Bounded constant regardless of dataset size — only resident pages
  count.
- **Disk:** Same as Option E or larger (full source imagery preserved).
- **Implementation cost:** Substantial.  Godot 4 does not provide a
  general-purpose virtual texturing system; it would require a custom
  RenderingDevice integration (compute shader feedback + page table +
  on-demand IO).  Industry-standard for AAA simulators (X-Plane, MSFS) but
  far beyond this project's current scope.
- **Verdict:** **Future option** if datasets grow into continent-scale coverage
  with sub-meter native imagery.  Not now.

### Performance Comparison Summary

For a 12 km KSBA build (24 km × 24 km bounding box, 144 L0 tiles, Sentinel-2
imagery at 10 m/pixel):

| Option | Tex VRAM | GLB size | Tile seams | Tex state changes / frame | Build complexity |
| --- | --- | --- | --- | --- | --- |
| **A — Per-facet color** | 0 | small | n/a (vertex colors) | 0 | done |
| **B — Per-vertex color** | 0 | small | n/a | 0 | low |
| **C — Per-tile JPEG** | ~2.3 GB | +200 MB | yes (need padding) | ~144 | high |
| **D — Single mosaic** | ~85 MB | +3 MB | none | 1 | low |
| **E — Multi-mosaic regional** | scales linearly | scales linearly | few | small | medium |
| **F — Texture2DArray** | ~2.3 GB | +200 MB | yes | 1 | high (custom glTF) |
| **G — Atlas** | ~85 MB | +3 MB | yes (need padding) | 1 | high |
| **H — Triplanar / procedural** | low | small | n/a | 1 | very high; wrong fit |
| **I — Virtual texturing** | bounded | very large | none | 1 | very high |

For larger datasets (jet trainer / airliner, > 50 km radius), Option D's mosaic
exceeds the GPU max texture size at native imagery resolution; Option E (multi-
mosaic) is the appropriate choice.

### Recommendation — OQ-TB-5

**Option D — Single mosaic texture per coverage area** as the baseline.  All
current build profiles (GA, jet trainer ≤ 50 km) fit within the single-mosaic
limit at Sentinel-2 native resolution.

**Option E — Multi-mosaic regional textures** as a documented extension; not
implemented now.  Add when a build profile requires coverage exceeding ~50 km
radius at native imagery resolution.  The data model accommodates this from the
start: each tile records its assigned mosaic ID; current builds always use
mosaic ID 0, future builds may have several.

#### Mosaic specification

For a build covering geographic bounds `[lat_min, lat_max] × [lon_min, lon_max]`
with imagery raster at native pixel size `Δ_imagery`:

- **Pixel resolution:**

  ```text
  W_pixels = ceil((lon_max - lon_min) * cos(center_lat) * R_earth / Δ_imagery)
  H_pixels = ceil((lat_max - lat_min) * R_earth / Δ_imagery)
  ```

  Round up to the next power of two on each axis (with the larger axis at most
  4096 for low-end-GPU compatibility, 8192 for desktop, 16384 for high-end).
  If both axes would exceed 4096 the build emits a warning and the operator
  should switch to Option E (not implemented yet — flagged as a runtime error
  at build time until E is delivered).
- **Padding:** None required — single texture has no internal seams.  The
  mosaic extent is the union of all tile extents, slightly larger than the
  configured radius bounds, so vertices at the dataset's outer edge are
  comfortably inside the texture.
- **Format:** JPEG quality 92 inside the GLB; sRGB gamma encoding (Sentinel-2
  imagery is already display-gamma encoded in our `colorize.py`).
- **Mipmap chain:** Generated on import by Godot 4 — automatic.

#### UV mapping

Each vertex's UVs are computed from its geodetic position relative to the
mosaic's geographic bounds, identical to the formula in the per-tile design but
referencing the mosaic bounds (not tile bounds):

```text
u = (lon - mosaic_lon_min) / (mosaic_lon_max - mosaic_lon_min)
v = (mosaic_lat_max - lat) / (mosaic_lat_max - mosaic_lat_min)   # row 0 = north
```

Since all tiles share the same mosaic, vertices at tile boundaries produce
identical UVs in the two adjacent tiles' meshes — no boundary stitching logic
is required beyond the existing `boundary_points` mechanism for geometric
continuity.

#### Material and shading

Single `BaseMaterial3D` (or `StandardMaterial3D`) per terrain GLB:

- `albedo_texture = <mosaic ImageTexture>`
- `shading_mode = SHADING_MODE_PER_PIXEL`
- `cull_mode = CULL_DISABLED`
- `texture_filter = TEXTURE_FILTER_LINEAR_WITH_MIPMAPS_ANISOTROPIC`

The `vertex_color_use_as_albedo` flag is removed; `COLOR_0` is no longer emitted.

[TerrainLoader.gd](../../godot/addons/liteaero_sim/TerrainLoader.gd) drops the
`material_override` walk for terrain — the GLB already carries a textured
material on every `MeshInstance3D`.  The brightness / saturation / contrast
sliders (currently overriding `_terrain_material`) should instead drive material
parameters on the imported material in place, or be implemented as a post-
process screen shader if global control is desired.

#### Pipeline impact

| Step | Current | After OQ-TB-5 (Option D) |
| --- | --- | --- |
| `download.py` | downloads imagery chunks | unchanged |
| `mosaic.py` | mosaics chunks → single GeoTIFF | unchanged |
| `geoid_correct.py` (LS-T3) | DEM → ellipsoidal | unchanged |
| `triangulate.py` | DEM → `TerrainTileData` with ENU vertices | unchanged geometry; adds geodetic position cache for UV computation |
| `colorize.py` | per-facet RGB from imagery centroid sample | superseded by `mosaic_render.py` (new) |
| `mosaic_render.py` (NEW) | crops imagery mosaic → JPEG bytes at PoT resolution | new |
| `las_terrain.py` | per-facet `colors` field | unchanged — `colors` advisory only |
| `export_gltf.py` | per-facet `COLOR_0` via duplicated vertices | mosaic JPEG + single material; per-vertex `TEXCOORD_0`; shared vertices |
| `TerrainLoader.gd` | builds custom `_terrain_material` and overrides per-`MeshInstance3D` | leaves GLB material in place; removes the `_terrain_material` construction |

#### Vertex deduplication side effect

Removing `COLOR_0` allows each shared vertex to appear once instead of three
times in the GLB.  KSBA L0 vertex count drops from ~432 K (3 × 144 K facets) to
~144 K — the geometry portion of the GLB shrinks accordingly.  GPU vertex shader
work also drops 3×.  Free benefit of the texturing change.

### Implementation Tasks — TB-T1 through TB-T5

| Task | Scope |
| --- | --- |
| TB-T1 | New `python/tools/terrain/mosaic_render.py` — given `(mosaic_bbox_deg, imagery_path, max_pixel_dim)` produces a JPEG byte buffer covering the bbox at imagery-native resolution rounded up to the nearest PoT, capped at `max_pixel_dim`.  Returns also the actual geographic bounds of the rendered raster (may exceed `mosaic_bbox_deg` due to PoT rounding). |
| TB-T2 | Extend `TerrainTileData` (or its parent dataset metadata) with the per-build mosaic descriptor: `mosaic_jpeg: bytes`, `mosaic_lat_min/max`, `mosaic_lon_min/max`, `mosaic_w_pixels`, `mosaic_h_pixels`.  Populate from `mosaic_render.py` output during `build_terrain.py`. |
| TB-T3 | Rewrite `export_gltf.py`: build vertex deduplication (one POSITION per logical vertex); compute per-vertex `TEXCOORD_0` from each vertex's geodetic position vs the mosaic bounds; emit a single glTF `Image` (JPEG) + `Texture` + `Material` referenced by every tile's `MeshPrimitive`; remove `COLOR_0`. |
| TB-T4 | Update `TerrainLoader.gd`: remove `_create_materials()`'s terrain-material construction and `_apply_material_to_tree()`'s terrain branch; rely on the GLB's imported textured material.  Add `texture_filter` override per-`MeshInstance3D` if the GLB import default doesn't already select `LINEAR_WITH_MIPMAPS_ANISOTROPIC`.  Re-wire saturation / brightness / contrast sliders against the imported material's `albedo_color` modulation, or remove them and document a follow-up. |
| TB-T5 | Detection + warning: `mosaic_render.py` raises `MosaicTooLargeError` when computed `(W_pixels, H_pixels)` exceeds the configured GPU ceiling.  `build_terrain.py` catches and prints a clear message instructing the operator to wait for Option E (multi-mosaic regional) — or, optionally, downsample imagery resolution to fit (with the trade-off documented). |

Total: ~400 lines of new/changed Python, ~80 lines of GDScript change, no C++
change, no simulation-side change.  The `.las_terrain` schema is unaffected.

### Resolution Criterion — OQ-TB-5

OQ-TB-5 is resolved when:

1. A KSBA terrain build produces a GLB carrying a single embedded JPEG mosaic
   texture and per-vertex `TEXCOORD_0` attributes; `COLOR_0` is absent.
2. The runway is visible as a clean, continuous strip in the Godot view at every
   altitude where the surrounding tiles are within the active LOD's display
   radius.
3. Mesh LOD switching produces no visible texture discontinuity (mipmap chain
   handles the resolution transition).
4. VRAM consumption for terrain textures is within an order of magnitude of the
   imagery-native-resolution lower bound (i.e., not bilateral up-sampling waste).
5. Build pipeline detects coverage areas exceeding the single-mosaic limit and
   emits a clear error referencing Option E.

---

## OQ-TB-5 Addendum — Per-Tile Textures with NAIP Implemented

### Why Option C Became Viable

The original OQ-TB-5 analysis rejected Option C (per-tile textures) because it assumed
the full set of L0 tiles across the terrain coverage area would be exported — 144 tiles
for a 12 km KSBA build.  At 2048² per tile that is 2.3 GB of VRAM, which exceeds the
4 GB GTX 1050 Ti budget when combined with other scene resources.

OQ-TB-3 analysis revealed that **geographic partitioning** limits the number of exported
tiles to roughly one per LOD level within the visibility-range cutoff radius.  For a
KSBA build with seven LOD levels (L0–L6), the total exported tile count is
approximately 7–10 tiles.  Per-tile texture VRAM is then:

| LOD | Tile count | Max pixel dim | Approx VRAM per tile | Subtotal |
| --- | --- | --- | --- | --- |
| L0 | 1 | 2048 | ~12 MB | ~12 MB |
| L1 | 1 | 4096 | ~48 MB | ~48 MB |
| L2 | 1 | 8192 | ~192 MB | ~192 MB |
| L3 | 1 | 8192 | ~192 MB | ~192 MB |
| L4–L6 | 1–4 | 4096 | ~48 MB each | ~100 MB |
| **Total** | **~7** | | | **~550 MB** |

550 MB is 14% of the 4 GB budget — well within headroom alongside the aircraft mesh,
render targets, and Godot overhead.

### Why NAIP Makes Per-Tile Worth It

Sentinel-2's 10 m/pixel native resolution means that a single-mosaic texture covering
all LOD levels has no more inherent detail than 10 m/px regardless of the number of
pixels allocated.  At LOD 0 (~1 km × 1 km tile footprint), a 2048² texture at
Sentinel-2 native scales to ~0.5 m/px per tile — but this only resamples existing 10 m
pixels; it does not recover sub-10 m features.

NAIP provides **~0.6 m/pixel native resolution** — genuine sub-meter photographic
detail.  An L0 tile (~1 km) at NAIP native fits in a 1667 × 1667 pixel texture, which
is within the 2048 cap without clamping.  The KSBA runway (45 m wide) maps to
approximately 75 pixels across in the L0 texture — sharp, unaliased, and visible at
close range.

The combination of geographic partitioning (few tiles) and NAIP native resolution
(genuine sub-meter imagery) makes per-tile textures both feasible and the approach that
delivers the most imagery detail for close-in LODs.

### Per-LOD Texture Resolution Caps

```python
_LOD_MAX_PIXEL_DIM: list[int] = [
    2048,   # L0  ~1 km footprint   → NAIP native ~1667 px, fits without clamping
    4096,   # L1  ~3 km footprint   → NAIP native ~5000 px → 4096 cap (~0.73 m/px)
    8192,   # L2  ~10 km footprint  → NAIP native ~16 k px → 8192 cap (~1.2 m/px)
    8192,   # L3  ~30 km footprint  → 8192 cap (~3.7 m/px)
    4096,   # L4  ~100 km footprint → Sentinel-2 native ~10 k px → 4096 cap
    4096,   # L5  same as L4
    4096,   # L6  same as L4
]
```

L4–L6 tiles cover 100 km+ footprints where NAIP's quarter-quad tiles would require
assembling hundreds of source tiles; Sentinel-2 is appropriate at those scales.

### Implementation

`build_terrain.py` renders one mosaic per exported tile and passes the list of
`(TerrainTileData, MosaicDescriptor)` pairs to `export_gltf`:

```python
tile_mosaics: list[tuple[TerrainTileData, MosaicDescriptor]] = []
for tile in display_tiles:
    tile_bbox_deg = (
        math.degrees(tile.lon_min_rad),
        math.degrees(tile.lat_min_rad),
        math.degrees(tile.lon_max_rad),
        math.degrees(tile.lat_max_rad),
    )
    tile_mosaic = render_mosaic(
        tile_bbox_deg, img_mosaic_paths,
        max_pixel_dim=_LOD_MAX_PIXEL_DIM[tile.lod],
    )
    tile_mosaics.append((tile, tile_mosaic))
export_gltf(tile_mosaics, gltf_path, world_origin=world_origin)
```

`export_gltf` creates one `Image`, `Texture`, and `Material` per tile inside the tile
loop.  All primitives share a single `Sampler` (linear filtering, clamp-to-edge).
Each tile's `MeshPrimitive` references its own material index.  Per-vertex
`TEXCOORD_0` maps each vertex's geodetic position to the tile's own mosaic bounds
(`u=0` at tile west edge, `u=1` at tile east edge; `v=0` at tile north edge,
`v=1` at tile south edge).

### Updated Performance Table

For a KSBA build (~7 exported tiles):

| Aspect | Old baseline (Option A — per-facet color) | Implemented (per-tile NAIP texture) |
| --- | --- | --- |
| Imagery resolution | 10 m/px (Sentinel-2) | 0.6 m/px (NAIP) |
| Runway visible in Godot | No (45 m feature, speckle alias) | Yes (~75 px wide at L0) |
| Tex VRAM | 0 | ~550 MB |
| GPU geometry | 3× duplicated vertices (COLOR_0) | 1× deduplicated vertices |
| GLB texture data | 0 | ~10–30 MB JPEG |
| Godot material count | 1 (shared terrain material) | ~7 (one per exported tile) |
| Seams | None (vertex color) | None (one texture per tile; UV clamped) |

### Resolution Criterion — OQ-TB-5 (Revised)

OQ-TB-5 is resolved when:

1. A KSBA terrain build produces a GLB with one JPEG texture per exported tile;
   `COLOR_0` is absent from all primitives.
2. `TEXCOORD_0` maps each vertex's geodetic position correctly to that tile's own
   mosaic bounds.
3. The KSBA runway is visible as a continuous strip in the Godot view when the
   camera is within the L0 tile's visibility range (≤ 345 m slant).
4. Total terrain texture VRAM is under 1 GB on the reference GTX 1050 Ti (4 GB).
5. The pipeline detects `imagery_source="naip"` and bypasses `mosaic_imagery`,
   passing NAIP tile paths directly to `render_mosaic`.

---

## OQ-TB-6 Analysis — Tile-Boundary Seams (Shared Boundary Vertices)

### Observed Problem

In the Godot viewer, continuous ground features — a straight road or the KSBA runway —
**shift discontinuously across tile boundaries**: the feature is drawn up to a few metres
out of place on one side of the seam, breaking the "runway visible as a continuous strip"
acceptance criterion (§OQ-TB-5 Addendum). The offset is largest at coarse LODs and shrinks
at fine LODs, which rules out a source-imagery artifact (the DEM and imagery mosaics are
continuous) and rules out the projection (each tile converts to ENU from its own centroid
consistently, so a *shared* boundary vertex would project identically on both tiles).

### Root Cause

The tiling and the meshing use two different, unsynchronised grids.
[`_compute_cell_grid`](../../python/tools/terrain/build_terrain.py) cuts the coverage bbox
into cells that abut exactly (each cell's east edge is the next cell's west edge). But each
cell is then meshed **independently**, starting from its own `lon_min`:

```python
lons = np.arange(lon_min, lon_max + spacing * 0.5, spacing)   # triangulate.py
```

Unless the cell width is an exact integer multiple of the LOD grid `spacing`, the last
sampled vertex does **not** land on the cell's east edge — it undershoots (leaving a missing
strip) or overshoots (leaving an overlap) by up to **half a grid step** (≈ 5 m at LOD 0,
`spacing ≈ 10 m`; larger at coarser LODs). Adjacent tiles therefore do not share their
boundary vertices: the terrain gaps/overlaps at the seam, and because each tile's texture
`TEXCOORD_0` is mapped to its own mosaic bounds
([`export_gltf.py`](../../python/tools/terrain/export_gltf.py)), a feature crossing the seam
is drawn out of place on the mis-terminated side. The size tracking the grid spacing is the
signature of this half-step error.

### Alternatives

1. **Endpoint-locked grid (`linspace`).** Replace the per-cell `arange` with
   `np.linspace(lon_min, lon_max, nx)`, `nx = max(2, round((lon_max − lon_min)/spacing) + 1)`
   (same for latitude). Every tile then spans its cell **exactly**, with the west/east and
   south/north edges as vertices; a shared cell edge is a shared vertex on both neighbours.
   - *Benefits:* one-line change, fully inside `triangulate.py`; guarantees shared boundary
     vertices for all abutting same-LOD tiles; the interior spacing shifts by a sub-percent
     amount (immaterial to the DEM/texture sampling); no change to `verify.py`, the cell
     grid, LOD scheme, projection, or export.
   - *Drawbacks:* the realized vertex spacing is the cell width divided by an integer rather
     than exactly `spacing` — a negligible deviation, and the LOD footprint policy already
     uses roughly square cells.
2. **Global-grid snap of the cell boundaries.** Make `_compute_cell_grid` emit cells whose
   edges are integer multiples of the LOD `spacing`, so the existing `arange` lands on them.
   - *Benefits:* keeps `arange` and the exact nominal spacing.
   - *Drawbacks:* the cells are shared across LODs but `spacing` is per-LOD, so no single set
     of cell edges is a multiple of every LOD's spacing — a cell grid aligned for one LOD is
     misaligned for the others, so this does not generalize.
3. **Delaunay with locked boundary points.** Use the existing `triangulate()`
   `boundary_points` path (Delaunay over the grid plus injected, locked shared-edge vertices).
   - *Benefits:* already partly implemented; supports irregular adaptive sampling later.
   - *Drawbacks:* pulls in `scipy.spatial.Delaunay` on the common build path (the regular-grid
     path was deliberately kept numpy-only for memory/perf); heavier than needed to solve a
     pure boundary-alignment problem.

### Resolution

**Alternative 1 — endpoint-locked `linspace` grid.** `triangulate.py` samples each cell on
`np.linspace(lon_min, lon_max, nx) × np.linspace(lat_min, lat_max, ny)`, so every tile spans
its cell exactly and abutting same-LOD tiles share boundary vertices. This closes the
**shared-vertex geometry seam** (the meshing/tiling grid mismatch). It does **not** address
the dominant residual seam — a per-tile texture azimuth rotation from projected-imagery
resampling — which was diagnosed separately and resolved in §OQ-TB-7. It does not resolve
OQ-TB-4 (triangle quality) — the connectivity is unchanged — and requires a terrain
**rebuild** to regenerate the GLB tiles. A regression test asserts that two
horizontally-adjacent cells produce identical lon/lat (and ENU-consistent) vertices along
their shared edge.

---

## OQ-TB-7 Analysis — Per-Tile Texture Azimuth Rotation (Projected-Source Resampling)

### Observed Problem

After OQ-TB-6 closed the shared-vertex geometry seam, continuous features — the KSBA runway
edge and centerline, straight roads — **still tear at every tile seam** in the Godot viewer:
the marked edge jogs laterally by a metre or two across the boundary. The mesh is continuous
(adjacent tiles' shared vertices coincide exactly and there is no gap in the surface), so it is
purely a **texture** displacement. It is present at every LOD and **directly under the aircraft**
(the finest LOD), which rules out an LOD-transition artifact, and the jog reads as a slight
**azimuth rotation of each tile's imagery about its own centre**.

### Root Cause

NAIP is delivered in a projected CRS (UTM, e.g. EPSG:26911).
[`render_mosaic._read_source_rgb`](../../python/tools/terrain/mosaic_render.py) resampled it onto
each tile's geographic (EPSG:4326) grid by taking the UTM **bounding box** of the tile
(`transform_bounds`) and doing a windowed read with `out_shape` — i.e. it read an axis-aligned
UTM window and **linearly stretched** it onto the tile's lat/lon grid:

```python
bounds = transform_bounds("EPSG:4326", src.crs, lon_min, lat_min, lon_max, lat_max)
window = _window_from_bounds(*bounds, src.transform)
src.read(..., window=window, out_shape=(3, h_pixels, w_pixels), resampling=bilinear)
```

A linear stretch maps UTM **grid-north** onto the tile's **true north**. The two differ by the
**grid convergence** angle

$$\gamma \approx (\lambda - \lambda_0)\,\sin\varphi \approx 1.6^\circ \text{ at KSBA (zone 11, } \lambda_0 = -117^\circ).$$

So every tile's imagery comes out rotated by ≈ γ about its own centre. Neighbouring tiles rotate
about **different** centres, so at a shared edge the two contents are displaced in opposite
directions and a continuous feature tears by up to ≈ γ · (tile size) — a few metres, largest at
coarse LODs, non-zero even at the finest LOD. Verified: the raw NAIP reprojected properly with
`rasterio.warp.reproject` is continuous, while the shipped per-tile texture is sheared relative to
it (the our-vs-proper horizontal shift trends monotonically across the tile — a rotation
signature — median ≈ 2.3 m, ≈ 5 m swing across a 280 m tile). The source is clean (both covering
NAIP quads share a capture date), confirming the tear is introduced by our resampling, not the
imagery.

### Alternatives

1. **Windowed reprojection (`rasterio.warp.reproject` on a bounded source window).** For a
   projected source, read only the source window covering the tile bbox (padded a few pixels so
   the rotation does not clip the corners), then `reproject` that array onto the tile's EPSG:4326
   grid (`dst_transform = from_bounds(lon_min, lat_min, lon_max, lat_max, w, h)`).
   - *Benefits:* correct per-pixel derotation of grid-north → true north; change is localized to
     `_read_source_rgb`; the read stays windowed so the per-tile memory bound (§Design Decisions)
     is preserved; EPSG:4326 sources (Sentinel-2, Landsat) are already north-aligned and keep the
     existing direct windowed-read path unchanged.
   - *Drawbacks:* one `reproject` per tile per projected source (modest CPU); needs a small source
     window pad so the rotated tile corners are covered.
2. **`WarpedVRT` wrapper.** Wrap the source in a `WarpedVRT(crs="EPSG:4326")` and windowed-read
   from the virtual reprojected raster.
   - *Benefits:* same correctness; rasterio manages source block reads.
   - *Drawbacks:* per-call VRT construction; boundless/edge-fill behaviour is more opaque than an
     explicit windowed `reproject`; no advantage over Alternative 1 for a single bbox read.
3. **Pre-reproject each NAIP quad to EPSG:4326 once, then window-read (as `mosaic_imagery` does
   for Sentinel-2).**
   - *Benefits:* reprojection amortized across tiles; uniform EPSG:4326 read path downstream.
   - *Drawbacks:* materializes full-resolution reprojected ~0.5 GB quads; `mosaic_imagery` already
     mishandles UTM metre pixel size (the reason NAIP bypasses it), so this reintroduces that path;
     heavier than the per-tile windowed reproject for no registration benefit.

### Resolution

**Alternative 1 — windowed reprojection.** `_read_source_rgb` reprojects any non-EPSG:4326 source
onto the tile's geographic grid with `rasterio.warp.reproject` over a padded source window read
**decimated** (`out_shape`-capped to ~2× the tile's output grid, so a region-spanning coarse tile
reads a bounded block rather than the full native window); EPSG:4326 sources keep the direct
windowed read. This derotates grid-north to true north per pixel, so per-tile azimuth rotation is
removed and features stay registered across seams, while memory stays window-bounded. It requires a terrain **rebuild** to regenerate the tile
textures (the shear is baked into the shipped JPEGs). A regression test builds a UTM source off
its central meridian (non-zero γ) carrying a true-north-aligned edge and asserts `render_mosaic`
keeps the edge vertical to within ~1 px / matches a reference `reproject` — the old linear-stretch
tilts it by γ and fails.

---

## Roadmap Item

This design is associated with roadmap item **TB-1** in
[`docs/roadmap/aircraft.md`](../roadmap/aircraft.md).

**TB-1** depends on:

- OQ-TB-1, OQ-TB-2, OQ-TB-3, and OQ-TB-5 are resolved by this document.
  Implementation is complete for all four.
- OQ-TB-4 (triangulation mesh quality) is open.  It does not block the current
  implementation but must be resolved before the terrain pipeline is considered
  production-ready.
- All existing terrain ingestion tools in `python/tools/terrain/` (Steps 14–21 of
  [terrain-implementation-plan.md](../roadmap/terrain-implementation-plan.md)) — all
  complete.
