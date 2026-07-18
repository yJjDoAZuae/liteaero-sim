# Conformal Terrain Grid Overlay — Implementation Plan

**Design authority:** [`docs/design/godot_plugin.md` §Conformal Terrain Grid](../design/godot_plugin.md)
(and §Appearance / §Viewer Appearance Config).

A runtime reference grid drawn on the terrain surface: north/east-aligned lines at equal metric
spacing from the region centroid, with configurable spacing, color, opacity, and physical line
width. Rendered as a term in the per-tile terrain shader (not baked into the mosaic), so it is
conformal at every LOD and never lies above/below the terrain.

## Current State

- Terrain tiles are wrapped at load in a `ShaderMaterial` running
  [`terrain_grade.gdshader`](../../godot/addons/liteaero_sim/terrain_grade.gdshader), driven by the
  grade uniforms from `configs/viewer.json` via `TerrainLoader._apply_viewer_config()` →
  `_set_terrain_grade_params()` → `_update_terrain_grade()`.
- No grid exists. This plan adds a grid term to that shader plus `grid_*` config/uniforms.
- No design open questions remain (line width = physical metres; grid color independent of the
  terrain grade; single uniform grid — major/minor deferred).

## Files to Create / Modify

| File | Change |
| --- | --- |
| `python/tools/terrain/grid_overlay.py` | **New.** Executable spec/reference of the grid-coverage math (numpy), mirrored by the shader. |
| `python/test/test_grid_overlay.py` | **New.** pytest for the coverage math (TDD, written first). |
| `godot/addons/liteaero_sim/terrain_grade.gdshader` | Add `world_pos` varying + grid term + `grid_*` uniforms. |
| `godot/addons/liteaero_sim/TerrainLoader.gd` | `grid_*` `@export`s, config load, uniform push. |
| `configs/viewer.json` | Add `grid` block (default `enabled: false`). |

## 1. Grid-coverage math (executable spec) — `grid_overlay.py`

Pure function mirrored verbatim by the shader. Per axis coordinate `c` (metres from centroid):

```text
g   = c / spacing
f   = abs(fract(g - 0.5) - 0.5)     # grid units to nearest line, in [0, 0.5]
d   = f * spacing                    # metres to nearest line
# analytic 1-D pixel coverage (aa = metres-per-pixel = clamped fwidth): overlap of the pixel
# span [d - aa/2, d + aa/2] with the line span [-width_m/2, width_m/2], as a fraction of aa.
cov = clamp((min(width_m/2, d + aa/2) - max(-width_m/2, d - aa/2)) / aa, 0, 1)
```

The analytic form (not a derivative-scaled smoothstep) is robust at grazing angles where `aa`
(fwidth) is huge; the shader clamps `aa = clamp(fwidth(c), 1e-4, spacing)`.

`line_coverage(c, spacing, width_m, aa)` returns `cov`; `grid_coverage(east, north, ...)` returns
`max(line_coverage(east,...), line_coverage(north,...))`. Final blend (documented, not in the
reference): `albedo = mix(albedo, grid_color, grid_coverage * grid_opacity)`.

## 2. Shader — `terrain_grade.gdshader`

- `varying vec3 v_world;` set in `vertex()` as `(MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz`.
- Uniforms: `grid_enabled` (float 0/1), `grid_spacing_m`, `grid_color` (vec3), `grid_opacity`,
  `grid_width_m`.
- In `fragment()`, after the matte grade and **before** the final clamp:
  `east = v_world.x; north = -v_world.z;` compute `cover` (§1 math, `aa = fwidth(coord)`), then
  `c = mix(c, grid_color, grid_enabled * cover * grid_opacity);`. Guard `grid_spacing_m > 0`.
- `grid_color`/`grid_opacity` are applied at full strength (independent of the grade).

## 3. `TerrainLoader.gd`

- `@export_group("Terrain Grid")`: `grid_enabled: bool`, `grid_spacing_m: float`,
  `grid_color: Color`, `grid_opacity: float (0–1)`, `grid_line_width_m: float`; each setter →
  `_update_terrain_grade()`.
- `_set_terrain_grade_params(sm)` also sets the five `grid_*` uniforms (`grid_enabled` as
  `1.0`/`0.0`).
- `_apply_viewer_config()` reads `cfg.grid`: `enabled`, `spacing_m`, `color` (via
  `_color_from_array`), `opacity`, `line_width_m`.

## 4. `configs/viewer.json`

Add: `"grid": { "enabled": false, "spacing_m": 1000.0, "color": [0,1,0], "opacity": 0.5, "line_width_m": 5.0 }`.

## 5. Test Specification (TDD order)

**Failing pytest first** — `python/test/test_grid_overlay.py` (against `grid_overlay.py`):

| Test | Assertion |
| --- | --- |
| `test_on_line_full_coverage` | `c = k*spacing` (several k, incl. 0 and negative) → coverage ≈ 1 (width ≫ aa). |
| `test_midway_zero_coverage` | `c = (k+0.5)*spacing` → coverage ≈ 0. |
| `test_physical_width_edges` | coverage crosses 0.5 at `abs(d) = width_m/2` (within aa); half-width scales with `width_m`. |
| `test_subpixel_line_dims` | `width_m < aa` → on-line coverage < 1 (partial), monotonic in `width_m/aa`. |
| `test_grid_is_axis_max` | `grid_coverage` = `max` of the two axis coverages; a point on an east line but far from any north line still reads full. |
| `test_spacing_scales` | line positions move with `spacing` (lines at multiples only). |

**Integration / acceptance** (no GDScript unit harness — per project convention):

- **Headless**: load the viewer with a `grid.enabled=true` config; assert the wrapped terrain
  `ShaderMaterial` carries `grid_enabled=1`, `grid_spacing_m`, `grid_color`, `grid_opacity`,
  `grid_width_m` (temporary-instrumented uniform dump, as used for the grade wiring).
- **GPU render (manual/local)**: enable the grid; screenshot; confirm (a) lines appear on the
  terrain surface, (b) north/east aligned and continuous across tiles, (c) spacing matches
  `spacing_m`, (d) lines lie on the surface at fine and coarse LODs (no float/sink), (e) color is
  independent of the terrain grade, (f) `enabled=false` removes it entirely.

## 6. Implementation Order

1. `test_grid_overlay.py` (failing) → `grid_overlay.py` (green).
2. Shader grid term + uniforms.
3. `TerrainLoader.gd` exports + config load + uniform push.
4. `configs/viewer.json` grid block.
5. Headless compile + uniform-propagation check; GPU screenshot acceptance.
