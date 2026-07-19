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

- `varying highp vec3 v_world;` set in `vertex()` as `(MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz`
  (`highp` for km-scale world coordinates).
- Uniforms: `grid_enabled` (float 0/1); major `grid_spacing_m`, `grid_color` (vec3),
  `grid_opacity`, `grid_width_m`; minor `grid_minor_divisions` (float; minor spacing =
  `grid_spacing_m / grid_minor_divisions`), `grid_minor_color`, `grid_minor_opacity`,
  `grid_minor_width_m`.
- `grid_line(coord, spacing, width_m, aa)` = §1 analytic coverage; `aa = clamp(fwidth(coord),
  1e-4, spacing)` (the clamp keeps grazing angles from filling the screen).
- `minor_line(coord, minor_spacing, major_spacing, width_m, aa)` = `grid_line` **zeroed** where the
  nearest minor line coincides with a major line (§1 `minor_line_coverage`).
- In `fragment()`, after the matte grade and **before** the final clamp, with `east = v_world.x;
  north = -v_world.z;` — if `grid_enabled`, composite the **minor** grid first (guard
  `grid_spacing_m > 0 && grid_minor_divisions >= 2`, using `minor_line`) then the **major** grid on
  top (guard `grid_spacing_m > 0`, using `grid_line`), each `cover = max(east, north)`,
  `c = mix(c, <level>_color, cover · <level>_opacity)`. Each level's color/opacity is full strength
  (independent of the grade).

## 3. `TerrainLoader.gd`

- `@export_group("Terrain Grid")`: `grid_enabled: bool`; major `grid_spacing_m`, `grid_color`,
  `grid_opacity`, `grid_line_width_m`; minor `grid_minor_divisions: int`, `grid_minor_color`,
  `grid_minor_opacity`, `grid_minor_line_width_m`; each setter → `_update_terrain_grade()`.
- `_set_terrain_grade_params(sm)` sets all `grid_*` / `grid_minor_*` uniforms (`grid_enabled` and
  `grid_minor_divisions` as floats).
- `_apply_viewer_config()` reads `cfg.grid`: `enabled`, `spacing_m`, `color`, `opacity`,
  `line_width_m`, and `minor_divisions`, `minor_color`, `minor_opacity`, `minor_line_width_m`.

## 4. `configs/viewer.json`

Add a `grid` block with major keys plus `minor_divisions` (integer; minor spacing = major /
divisions; `< 2` = minor grid off), `minor_color`, `minor_opacity`, `minor_line_width_m`.

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
| `test_minor_nests_major` | when the minor spacing divides the major, every major line is also a minor line (major lines are a subset) — the property the two-level grid relies on. |
| `test_minor_skips_major` | `minor_line_coverage` is 0 where the minor line coincides with a major line, unchanged elsewhere, and unsuppressed when `major_spacing = 0`. |

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
