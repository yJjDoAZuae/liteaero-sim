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

All three open questions are resolved.  Implementation of TB-1 may begin.

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

### Option B — Sidecar `terrain_config.json` with terrain loader node

`build_terrain` writes a JSON file into the Godot project directory alongside a
terrain GLB that has been copied there:

```
godot/terrain/
    terrain.glb             copy of the exported GLB (symlink or file copy)
    terrain_config.json     build metadata read at scene start
```

```json
{
    "schema_version": 1,
    "dataset_name": "general_aviation",
    "glb_path": "res://terrain/terrain.glb",
    "world_origin_lat_rad": 1.0490,
    "world_origin_lon_rad": -2.0909,
    "world_origin_height_m": 610.0
}
```

A `TerrainLoader.gd` autoload or scene node runs `_ready()`:

1. Opens `terrain_config.json`.
2. Calls `ResourceLoader.load(glb_path)` to load the GLB as a `PackedScene`.
3. Instantiates it and adds it as a child of the `World` node — no drag-and-drop.
4. Iterates all `MeshInstance3D` children; parses the node name to extract LOD level;
   sets `visibility_range_begin` / `visibility_range_end` on each node (OQ-TB-3).
5. Sets `SimulationReceiver.world_origin_*` directly before any UDP packet arrives.

The developer's workflow after a terrain build is:

1. Run `build_terrain`.
2. Press Play in Godot.

No Godot editor interaction is required after the initial scene setup.

**Implications:**

- Terrain alignment is explicit and correct even if the simulation starts from a
  position other than `initial_state`.
- GLB is loaded at runtime — no import step, no scene tree editing, no `.import`
  sidecar files to manage.
- Visibility ranges are set programmatically, not via the Inspector.
- Requires: `TerrainLoader.gd` (new node or autoload), a GDScript change to
  `SimulationReceiver.gd`, and `build_terrain` writing both the JSON file and
  copying/symlinking the GLB to `godot/terrain/`.
- `godot/terrain/` should be listed in `.gitignore` (generated file; varies per
  developer machine).
- `push_error()` on missing `terrain_config.json` or missing GLB — no silent fallback.

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

**Adopt Option B.**

Option A is fragile (auto-origin from first UDP packet can silently offset terrain on
checkpoint resume) and requires manual drag-and-drop in the Godot editor every time the
terrain dataset changes.  Option D also requires manual Inspector edits.  Option C
requires a custom Godot import plugin and is significantly more complex.

Option B makes the terrain dataset self-describing: the sidecar JSON contains the GLB
path, world origin, and dataset name.  `TerrainLoader.gd` reads it at startup, loads the
GLB programmatically, sets world origin on `SimulationReceiver`, and applies visibility
ranges — all before the first frame.  The developer workflow reduces to: build terrain,
press Play.

`godot/terrain/` must be added to `.gitignore`.  OQ-TB-2 is resolved pending
implementation of `TerrainLoader.gd`.

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
| Dataset named from aircraft config basename | `general_aviation.json` → dataset name `general_aviation`. Terrain datasets represent locations, not aircraft states — moving the aircraft to the other end of the runway reuses the same dataset. An explicit `name` override is available. |
| Terrain loaded programmatically at runtime | `TerrainLoader.gd` reads `terrain_config.json` and loads the GLB via `ResourceLoader.load()`. No Godot editor drag-and-drop is required after the initial scene setup. |

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
    imagery_source: str = "sentinel2",
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
        Imagery source for facet colorisation.  One of ``"sentinel2"``,
        ``"landsat9"``, ``"modis"``.
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

### Chunk Size

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

Imagery download uses the same tiling with the same chunk dimensions.  Total API
requests (DEM + imagery) = 2 × the DEM request count above.

### Cache Layout

Each downloaded chunk is stored at:

```
python/cache/terrain/dem/<source>/<bbox_tag>.tif
python/cache/terrain/imagery/<source>/<bbox_tag>.tif
```

where `bbox_tag` is generated by `download.py:_bbox_tag`.  If the file already exists
the download is skipped.  `force=True` deletes all cached files for the dataset before
beginning.

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
[1] download_dem(chunk_bbox)          ×n_chunks   (native L0 resolution)
    download_imagery(chunk_bbox)       ×n_chunks
    │
    ▼
[2] mosaic_dem(dem_chunks → source/dem_mosaic.tif)
    mosaic_dem(img_chunks → source/img_mosaic.tif)
    │
    ▼
[3] triangulate(dem_mosaic, bbox_deg, lod=0)
    → L0 tile (TerrainTileData)
    │
    ▼
[4] colorize(tile, img_mosaic, source=imagery_source)
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
[7] export_gltf(display_tiles, gltf_path)   (see OQ-TB-3 §Geographic Partitioning)
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

```
data/terrain/<name>/
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
| All velocity components zero | Raise `ValueError("cruise speed is zero — supply radius_m explicitly")` |
| Download authentication failure | `DownloadError` (from `download.py`) propagates; no partial output is written |
| Mesh quality check failure | `MeshQualityError` (from `verify.py`) propagates; the `.las_terrain` file is not written |
| Any step raises after mosaicking | Intermediate rasters in `source/` are retained so a subsequent run can skip the download step |

---

## Integration with Live Simulation

After `build_terrain` completes, the developer presses Play in the Godot project.
No editor interaction is required.

`build_terrain` is responsible for:

- Writing `godot/terrain/terrain.glb` (copy or symlink of
  `data/terrain/<name>/derived/gltf/terrain.glb`).
- Writing `godot/terrain/terrain_config.json` with world origin and GLB path.

`TerrainLoader.gd` runs at scene start and is responsible for:

- Loading the GLB from the path in `terrain_config.json` via `ResourceLoader.load()`.
- Instantiating it as a child of the `World` node.
- Iterating `MeshInstance3D` children; parsing node names to extract LOD; setting
  `visibility_range_begin` / `visibility_range_end` on each node.
- Setting `SimulationReceiver.world_origin_*` from `terrain_config.json` before the
  first UDP packet arrives.

`godot/terrain/` is in `.gitignore`.  A fresh checkout requires one `build_terrain`
call per location before the scene can be played.

The `.las_terrain` file will be consumed by `TerrainMesh::deserializeLasTerrain()` once
the `Aircraft` / `SimRunner` terrain integration is implemented (a separate future item).

---

## Roadmap Item

This design is associated with roadmap item **TB-1** in
[`docs/roadmap/aircraft.md`](../roadmap/aircraft.md).

**TB-1** depends on:

- OQ-TB-1, OQ-TB-2, and OQ-TB-3 are resolved by this document.  Implementation may begin.
- All existing terrain ingestion tools in `python/tools/terrain/` (Steps 14–21 of
  [terrain-implementation-plan.md](../roadmap/terrain-implementation-plan.md)) — all
  complete.
