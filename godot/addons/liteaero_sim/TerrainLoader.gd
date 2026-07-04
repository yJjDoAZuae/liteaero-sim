## TerrainLoader.gd — Programmatic terrain dataset loader.
##
## Design authority: docs/architecture/terrain_build.md §OQ-TB-2, §OQ-TB-3, §OQ-TB-5
##                   docs/architecture/godot_plugin.md §TerrainLoader Integration
##
## Reads terrain_config.json from the path supplied via the Godot command-line user
## arg --terrain at scene start, loads the terrain, applies per-node LOD visibility
## ranges, loads the aircraft mesh, and positions the camera — all before the first
## UDP packet arrives.
##
## Terrain input (two shapes, dispatched in _ready):
##   * Live streaming terrain — config "terrain_descriptor_path" points at a chunked
##     dataset's descriptor.json (OQ-LS-20): per-(LOD, chunk) GLB files addressed by
##     integer chunk coordinate.  IP-LV-8 loads every chunk up front; IP-LV-7 adds
##     proximity streaming via load_chunk() / free_chunk() / chunk_coord_for_position().
##   * Static test scene — config "glb_path" points at a single monolithic GLB
##     (gen_test_assets axis-prism terrain).
##
## Terrain texture: each GLB carries embedded JPEG mosaic textures and a PBR material
## per MeshInstance3D.  No material override is applied by this loader.
##
## Appearance controls (adjustable in the Inspector while the scene runs):
##   aircraft_saturation / aircraft_brightness / aircraft_contrast / aircraft_transparency
##
## Workflow after a terrain build:
##   1. Run build_terrain (Python).
##   2. Launch Godot with:
##        godot4 -- --terrain <absolute-path-to-terrain_config.json>
##      Or in the Godot editor: Project → Project Settings → Run → Launch Flags,
##      set to:  -- --terrain <absolute-path-to-terrain_config.json>

extends Node


## Rendering-LOD visibility bands are NOT fixed constants: they are recomputed at runtime from
## the live viewport height and field of view using the screen-space-error policy recorded in the
## terrain descriptor (terrain_lod_rendering.md, OQ-LR-1 Alternative 2 / OQ-LR-2).  The former
## fixed convention table (r_0 = 300 m) is superseded — see _recompute_lod_bands().

# ---------------------------------------------------------------------------
# Appearance controls — aircraft
# ---------------------------------------------------------------------------

@export_group("Aircraft Appearance")

## Saturation: 0.0 = greyscale, 1.0 = source color, 2.0 = double saturation.
@export_range(0.0, 2.0) var aircraft_saturation: float = 1.0:
	set(v):
		aircraft_saturation = v
		_update_aircraft_color()

## Brightness multiplier applied to the base color.  0.0 = black, 1.0 = source.
@export_range(0.0, 2.0) var aircraft_brightness: float = 1.0:
	set(v):
		aircraft_brightness = v
		_update_aircraft_color()

## Contrast: 0.0 = flat grey, 1.0 = source, 2.0 = double contrast.
@export_range(0.0, 2.0) var aircraft_contrast: float = 1.0:
	set(v):
		aircraft_contrast = v
		_update_aircraft_color()

## Transparency: 0.0 = invisible, 1.0 = fully opaque.
@export_range(0.0, 1.0) var aircraft_transparency: float = 1.0:
	set(v):
		aircraft_transparency = v
		_update_aircraft_color()

# ---------------------------------------------------------------------------
# Camera controls
# ---------------------------------------------------------------------------

@export_group("Camera")

## Distance from camera to aircraft origin (metres).
## Default: wingspan (~16 m) projects to 1/3 of horizontal view at FOV 90°.
@export_range(5.0, 2000.0) var camera_distance: float = 24.0:
	set(v):
		camera_distance = v
		_update_camera()

## Far clip distance (metres).  Increase to show more terrain at the cost of
## depth-buffer precision on nearby geometry.
@export_range(1000.0, 100000.0) var camera_far_m: float = 20000.0:
	set(v):
		camera_far_m = v
		_update_camera()

## Near clip distance (metres).  Keep >= 0.5 when far >= 10000 to preserve
## depth-buffer precision.
@export_range(0.05, 10.0) var camera_near_m: float = 0.5:
	set(v):
		camera_near_m = v
		_update_camera()

# ---------------------------------------------------------------------------
# Aircraft CG offset controls
# ---------------------------------------------------------------------------

@export_group("Aircraft CG")

## Longitudinal CG position as a fraction of the mesh X extent.
## 0.0 = nose tip, 1.0 = tail tip.  Tune visually against the mesh.
@export_range(0.0, 1.0, 0.01) var aircraft_cg_x_fraction: float = 0.5:
	set(v):
		aircraft_cg_x_fraction = v
		_update_aircraft_cg_offset()

## Vertical CG position as a fraction of the mesh Z extent.
## 0.0 = belly, 1.0 = top of vertical stabilizer.
@export_range(0.0, 1.0, 0.01) var aircraft_cg_z_fraction: float = 0.2:
	set(v):
		aircraft_cg_z_fraction = v
		_update_aircraft_cg_offset()

# ---------------------------------------------------------------------------
# Terrain streaming (OQ-LS-21 / IP-LV-7)
# ---------------------------------------------------------------------------

@export_group("Terrain Streaming")

## Page fine-LOD chunks in/out by aircraft proximity (chunked datasets only).
## When false, every chunk is loaded up front (the IP-LV-8 behavior).
@export var enable_streaming: bool = true

## Highest LOD index that streams by proximity.  LODs 0..stream_lod_max are the
## fine, texture-heavy tiles paged in/out; LODs above this load once and remain
## resident as the distant backdrop.
@export_range(0, 6) var stream_lod_max: int = 2

## Fetch margin (metres) added to a LOD's visibility-band outer radius to get its
## load radius R_fetch — a chunk is loaded slightly before its tiles become visible.
@export_range(0.0, 20000.0) var fetch_margin_m: float = 800.0

## Unload margin (metres) added to a LOD's visibility-band outer radius to get its
## free radius R_unload.  Must exceed fetch_margin_m: the R_unload > R_fetch gap is
## the hysteresis that prevents load/free thrash at the boundary (OQ-LS-21).
@export_range(0.0, 40000.0) var unload_margin_m: float = 2400.0

# ---------------------------------------------------------------------------
# Private state
# ---------------------------------------------------------------------------

var _aircraft_material:     StandardMaterial3D = null
var _camera:                Camera3D           = null
var _aircraft_center_world_y: float            = 0.0
var _aircraft_mesh_node:    Node3D             = null
var _aircraft_cg_prepos:    Vector3            = Vector3.ZERO  # CG in pre-position world space

# ---------------------------------------------------------------------------
# Streamable-chunk terrain state (OQ-LS-20 / IP-LV-8)
# ---------------------------------------------------------------------------

## Parent node that all loaded terrain chunk nodes are added under.
var _terrain_root:          Node3D             = null
## Absolute directory holding the chunk GLB files (descriptor.json's directory).
var _tiles_root:            String             = ""
## Per-LOD chunk side length in metres (tile_footprints_m[lod] * chunk_footprints), from the
## descriptor.  Per-LOD footprints (OQ-LS-22 Alt 3) give per-LOD chunk grids.
var _chunk_sizes_m:         Array              = []   # per-LOD chunk side (m); index = LOD
## Loaded chunk nodes keyed by "lod_cx_cy" → Node3D (for load/free by the streaming manager).
var _loaded_chunks:         Dictionary         = {}
## Descriptor chunk records keyed by "lod_cx_cy" → { "lod", "cx", "cy", "file", "tile_count" }.
var _chunk_index:           Dictionary         = {}
## Aircraft chunk coordinate at the last streaming evaluation (re-evaluate on change only).
var _last_stream_chunk:     Vector2i           = Vector2i(2147483647, 2147483647)

# --- Runtime screen-space-error LOD bands (terrain_lod_rendering.md / IP-LV-10) ---
## Rendering-LOD policy parameters from the descriptor (transition_error_m, tau_px, delta, ...).
var _lod_policy:            Dictionary         = {}
## LOD band edges [0, R1, ..., R6, R7] (m) at the current viewport height / FOV; band ℓ = [ℓ, ℓ+1].
var _lod_edges_m:           Array              = []
## Per-LOD visibility begin/end (m) before per-node tile-radius padding; end 0 = infinite (coarsest).
var _lod_begin_base:        Array              = []
var _lod_end_base:          Array              = []
## Viewport height / vertical FOV at the last band recompute (recompute only on change).
var _last_view_h:           float              = -1.0
var _last_view_fov:         float              = -1.0

# ---------------------------------------------------------------------------

var _diag_frame_counter: int = 0

func _process(_delta: float) -> void:
	_update_camera()
	_maybe_recompute_lod_bands()
	_update_terrain_streaming()
	_diag_frame_counter += 1
	if _diag_frame_counter % 300 == 0:
		var vehicle := _find_vehicle(get_tree().root)
		if vehicle != null:
			var p := vehicle.global_position
			if is_nan(p.x) or is_nan(p.y) or is_nan(p.z) or is_inf(p.x) or is_inf(p.y) or is_inf(p.z):
				print("TerrainLoader DIAG: vehicle position is NaN/Inf: %s" % str(p))
			else:
				print("TerrainLoader DIAG: vehicle global_pos Y=%.3f m (%.1f ft above GLB origin)" % [p.y, p.y * 3.28084])


func _ready() -> void:
	# Run after SimulationReceiver has updated Vehicle position this frame.
	# SimulationReceiver (child of Vehicle) processes at priority 0; setting a
	# higher value here guarantees _update_camera() sees the current-frame position.
	process_priority = 1
	_create_materials()

	var config := _load_config()
	if config.is_empty():
		print("TerrainLoader: config load failed — aborting")
		return

	# Two terrain input shapes are supported for two distinct scenes:
	#   * live streaming terrain  → "terrain_descriptor_path" (chunked, OQ-LS-20)
	#   * static axis-test scene  → "glb_path" (single monolithic GLB, gen_test_assets)
	var terrain_ok := false
	if config.has("terrain_descriptor_path"):
		terrain_ok = _load_chunked_terrain(config["terrain_descriptor_path"])
	elif config.has("glb_path"):
		terrain_ok = _load_single_glb_terrain(config["glb_path"])
	else:
		print("TerrainLoader: config has neither terrain_descriptor_path nor glb_path — aborting")
		return

	if not terrain_ok:
		print("TerrainLoader: terrain load failed — aborting")
		return

	_load_aircraft_mesh(config)
	# LS-T8: SimulationReceiver no longer needs the world origin.  The
	# simulation-side GodotEnuProjector consumes terrain_config.json directly
	# (live_sim.cpp wiring) and broadcasts viewer-projected position over UDP.
	_update_camera()

## Print the global Y position of the first few MeshInstance3D tiles to verify
## terrain is positioned at the expected Godot world height.
func _diag_print_terrain_positions(node: Node) -> void:
	var count := [0]  # array so recursive calls share the counter
	_diag_visit(node, count)

func _diag_visit(node: Node, count: Array) -> void:
	if count[0] >= 5:
		return
	if node is MeshInstance3D and node.name.begins_with("tile_"):
		var mi := node as MeshInstance3D
		var gp := mi.global_position
		print("TerrainLoader DIAG: tile '%s'  global_pos Y=%.3f m (%.1f ft)" % [
			mi.name, gp.y, gp.y * 3.28084])
		count[0] += 1
	for child: Node in node.get_children():
		_diag_visit(child, count)

# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------

## Position the scene Camera3D behind and above the Vehicle origin along the
## horizontal velocity direction (nose projected onto horizontal plane), so the
## camera remains level through pitch maneuvers.
## Called each frame via _process() and whenever camera_distance changes.
func _update_camera() -> void:
	if not is_inside_tree():
		return
	if _camera == null:
		_camera = _find_camera(get_tree().root)
	if _camera == null:
		return
	_camera.fov = 90.0
	_camera.keep_aspect = Camera3D.KEEP_WIDTH
	_camera.near = camera_near_m
	_camera.far  = camera_far_m
	var vehicle := _find_vehicle(get_tree().root)
	if vehicle == null:
		return
	var look_target := vehicle.global_position
	# Chase camera: offset aft and up along the horizontal velocity direction.
	# global_basis.x is the body nose direction in world space.  Project it onto
	# the horizontal plane so the camera stays level regardless of aircraft pitch.
	var nose := vehicle.global_basis.x
	var world_up := Vector3.UP
	var horizontal_fwd := Vector3(nose.x, 0.0, nose.z).normalized()
	if horizontal_fwd.length_squared() < 0.001:
		horizontal_fwd = Vector3(0.0, 0.0, -1.0)  # fallback: north
	var cam_offset := -horizontal_fwd * camera_distance + world_up * camera_distance * 0.15
	_camera.global_position = look_target + cam_offset
	_camera.look_at(look_target, world_up)


## Recompute mesh_node.position when CG fractions change in the Inspector.
func _update_aircraft_cg_offset() -> void:
	if _aircraft_mesh_node == null:
		return
	# Recompute CG prepos from stored value scaled by new fractions requires
	# re-reading the AABB, which means re-running the full placement logic.
	# Since the mesh is already instantiated, find the MeshInstance3D and recompute.
	var mi := _find_first_mesh_instance(_aircraft_mesh_node)
	if mi == null:
		return
	var aabb := mi.get_aabb()
	var cg_local := Vector3(
		aabb.position.x + aircraft_cg_x_fraction * aabb.size.x,
		aabb.position.y + 0.5 * aabb.size.y,
		aabb.position.z + aircraft_cg_z_fraction * aabb.size.z)
	_aircraft_cg_prepos = _aircraft_mesh_node.basis * (mi.transform * cg_local)
	_aircraft_mesh_node.position = -_aircraft_cg_prepos


## Depth-first search for the first Camera3D in the scene tree.
func _find_camera(node: Node) -> Camera3D:
	if node is Camera3D:
		return node as Camera3D
	for child: Node in node.get_children():
		var result := _find_camera(child)
		if result != null:
			return result
	return null

# ---------------------------------------------------------------------------
# Material creation
# ---------------------------------------------------------------------------

func _create_materials() -> void:
	_aircraft_material = StandardMaterial3D.new()
	_update_aircraft_color()


## Recompute the aircraft StandardMaterial3D albedo color from the base color
## (#4A7FC1 per PP-F25) modulated by brightness, contrast, and saturation.
## Transparency is applied via the material's alpha channel.
func _update_aircraft_color() -> void:
	if _aircraft_material == null:
		return
	var base := Color(0.290, 0.498, 0.757)  # #4A7FC1 per PP-F25
	var col := base
	col = Color(col.r * aircraft_brightness,
				col.g * aircraft_brightness,
				col.b * aircraft_brightness)
	col = Color((col.r - 0.5) * aircraft_contrast + 0.5,
				(col.g - 0.5) * aircraft_contrast + 0.5,
				(col.b - 0.5) * aircraft_contrast + 0.5)
	var lum: float = col.r * 0.2126 + col.g * 0.7152 + col.b * 0.0722
	col = Color(lerpf(lum, col.r, aircraft_saturation),
				lerpf(lum, col.g, aircraft_saturation),
				lerpf(lum, col.b, aircraft_saturation))
	col = col.clamp()
	col.a = aircraft_transparency
	_aircraft_material.albedo_color = col
	_aircraft_material.flags_transparent = aircraft_transparency < 1.0


## Recursively apply mat and enable shadow casting on every MeshInstance3D under node.
## Used for the aircraft mesh only; terrain material is provided by the GLB.
func _apply_material_to_tree(node: Node, mat: Material) -> void:
	if node is MeshInstance3D:
		var mi := node as MeshInstance3D
		mi.material_override = mat
		mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_ON
	for child: Node in node.get_children():
		_apply_material_to_tree(child, mat)

# ---------------------------------------------------------------------------

func _get_terrain_config_path() -> String:
	var user_args := OS.get_cmdline_user_args()
	var i := 0
	while i < user_args.size():
		if user_args[i] == "--terrain" and i + 1 < user_args.size():
			return user_args[i + 1]
		i += 1
	return ""


func _load_config() -> Dictionary:
	var config_path := _get_terrain_config_path()
	if config_path.is_empty():
		push_error(
			"TerrainLoader: --terrain argument not provided. "
			+ "Launch Godot with: godot4 -- --terrain <path-to-terrain_config.json>"
		)
		return {}

	if not FileAccess.file_exists(config_path):
		push_error(
			"TerrainLoader: terrain_config.json not found at '%s' — run build_terrain first"
			% config_path
		)
		return {}

	var f := FileAccess.open(config_path, FileAccess.READ)
	if f == null:
		push_error("TerrainLoader: cannot open '%s'" % config_path)
		return {}

	var parsed: Variant = JSON.parse_string(f.get_as_text())
	if parsed == null or not (parsed is Dictionary):
		push_error("TerrainLoader: terrain_config.json is not a valid JSON object at '%s'" % config_path)
		return {}

	return parsed as Dictionary


## Load the single-GLB static test scene (gen_test_assets axis-prism terrain).
func _load_single_glb_terrain(glb_path: String) -> bool:
	print("TerrainLoader: loading single-GLB terrain, glb_path=%s" % glb_path)
	var terrain_node := _load_terrain_scene(glb_path)
	if terrain_node == null:
		return false
	print("TerrainLoader: terrain scene loaded OK, child count=%d" % terrain_node.get_child_count())
	add_child(terrain_node)
	_diag_print_terrain_positions(terrain_node)
	_apply_visibility_ranges(terrain_node)
	return true


## Load a chunked streamable-terrain dataset from its descriptor.json (OQ-LS-20).
##
## Reads the descriptor, records the chunk index (for coordinate-addressable load/free),
## and — for IP-LV-8 — loads every chunk up front so behavior matches the former monolithic
## GLB.  IP-LV-7 will switch the fine LODs to on-demand streaming by aircraft proximity via
## load_chunk() / free_chunk() and chunk_coord_for_position().
func _load_chunked_terrain(descriptor_path: String) -> bool:
	var abs_desc := _resolve_terrain_path(descriptor_path)
	print("TerrainLoader: loading chunked terrain from %s" % abs_desc)
	var f := FileAccess.open(abs_desc, FileAccess.READ)
	if f == null:
		push_error("TerrainLoader: cannot open descriptor %s" % abs_desc)
		return false
	var parsed: Variant = JSON.parse_string(f.get_as_text())
	if typeof(parsed) != TYPE_DICTIONARY:
		push_error("TerrainLoader: descriptor %s is not a JSON object" % abs_desc)
		return false
	var descriptor: Dictionary = parsed

	_tiles_root = abs_desc.get_base_dir()
	_chunk_sizes_m = descriptor.get("chunk_sizes_m", [])
	if _chunk_sizes_m.is_empty() or float(_chunk_sizes_m[0]) <= 0.0:
		push_error("TerrainLoader: descriptor chunk_sizes_m must be a non-empty per-LOD list of positive sizes")
		return false

	# Screen-space-error LOD policy (terrain_lod_rendering.md / IP-LV-10): the visibility bands are
	# computed from these parameters at the live viewport height / FOV, not from a fixed table.
	_lod_policy = descriptor.get("lod_policy", {})
	_recompute_lod_bands()

	_terrain_root = Node3D.new()
	_terrain_root.name = "TerrainChunks"
	add_child(_terrain_root)

	var chunks: Array = descriptor.get("chunks", [])
	for rec in chunks:
		_chunk_index[_chunk_key(int(rec["lod"]), int(rec["cx"]), int(rec["cy"]))] = rec

	# Load the coarse (backdrop) LODs up front; when streaming is enabled the fine
	# LODs (0..stream_lod_max) are paged in by proximity in _update_terrain_streaming().
	# With streaming disabled, every chunk is loaded here (the IP-LV-8 behavior).
	for rec in chunks:
		var lod := int(rec["lod"])
		if enable_streaming and lod <= stream_lod_max:
			continue
		_load_chunk(lod, int(rec["cx"]), int(rec["cy"]))

	print("TerrainLoader: indexed %d chunk(s), %d resident up front, chunk_sizes=%s m" % [
		chunks.size(), _loaded_chunks.size(), str(_chunk_sizes_m)])
	return chunks.size() > 0


## Key a chunk by "(lod)_(cx)_(cy)" for the loaded-chunk / index dictionaries.
func _chunk_key(lod: int, cx: int, cy: int) -> String:
	return "%d_%d_%d" % [lod, cx, cy]


## Load one chunk GLB into the terrain root and apply LOD visibility ranges.
## Idempotent: a chunk already resident is left as-is.  Returns true if resident afterward.
func load_chunk(lod: int, cx: int, cy: int) -> bool:
	return _load_chunk(lod, cx, cy)


func _load_chunk(lod: int, cx: int, cy: int) -> bool:
	var key := _chunk_key(lod, cx, cy)
	if _loaded_chunks.has(key):
		return true
	if not _chunk_index.has(key):
		return false  # no such chunk in this dataset (sparse coverage)
	var rec: Dictionary = _chunk_index[key]
	var abs_path := _tiles_root.path_join(str(rec["file"]))
	var node := _load_terrain_scene(abs_path)
	if node == null:
		push_error("TerrainLoader: failed to load chunk %s" % abs_path)
		return false
	node.name = "chunk_%s" % key
	_terrain_root.add_child(node)
	_apply_visibility_ranges(node)
	_loaded_chunks[key] = node
	return true


## Free a resident chunk (used by the streaming manager to unload distant fine tiles).
func free_chunk(lod: int, cx: int, cy: int) -> void:
	var key := _chunk_key(lod, cx, cy)
	if not _loaded_chunks.has(key):
		return
	var node: Node = _loaded_chunks[key]
	_loaded_chunks.erase(key)
	if is_instance_valid(node):
		node.queue_free()


## Per-LOD chunk side length (m).  Clamps the LOD index to the available range.
func _chunk_size_for_lod(lod: int) -> float:
	if _chunk_sizes_m.is_empty():
		return 0.0
	return float(_chunk_sizes_m[clampi(lod, 0, _chunk_sizes_m.size() - 1)])


## Return the (cx, cy) chunk coordinate containing a Godot-space position at a given LOD.
## Godot axes: X = ENU east, Z = −ENU north; each LOD indexes its own chunk grid.
func chunk_coord_for_position(godot_pos: Vector3, lod: int) -> Vector2i:
	var size := _chunk_size_for_lod(lod)
	var east := godot_pos.x
	var north := -godot_pos.z
	return Vector2i(int(floor(east / size)), int(floor(north / size)))


## Page fine-LOD chunks in/out by aircraft proximity (OQ-LS-21, Alternative 1 with
## asymmetric hysteresis).  Re-evaluated only when the aircraft crosses into a new finest-LOD
## chunk (the smallest grid — the most sensitive trigger).
func _update_terrain_streaming() -> void:
	if not enable_streaming or _terrain_root == null or _chunk_sizes_m.is_empty():
		return
	var vehicle := _find_vehicle(get_tree().root)
	if vehicle == null:
		return
	var pos := vehicle.global_position
	if is_nan(pos.x) or is_inf(pos.x) or is_nan(pos.z) or is_inf(pos.z):
		return
	var here := chunk_coord_for_position(pos, 0)
	if here == _last_stream_chunk:
		return
	_last_stream_chunk = here
	_stream_evaluate(pos)


## Load streamed chunks within each fine LOD's fetch radius and free those beyond its
## (larger) unload radius.  R_unload > R_fetch is the hysteresis gap that prevents thrash.
## Each LOD uses its own chunk grid (per-LOD chunk size).
func _stream_evaluate(aircraft_pos: Vector3) -> void:
	var top_lod := mini(stream_lod_max, 6)
	# Fetch: for each streamed LOD, load every existing chunk within R_fetch.
	for lod in range(top_lod + 1):
		var size := _chunk_size_for_lod(lod)
		if size <= 0.0:
			continue
		var r_fetch := _stream_radius_m(lod, fetch_margin_m)
		var here := chunk_coord_for_position(aircraft_pos, lod)
		var reach := int(ceil(r_fetch / size)) + 1
		for dcx in range(-reach, reach + 1):
			for dcy in range(-reach, reach + 1):
				var cx := here.x + dcx
				var cy := here.y + dcy
				if not _chunk_index.has(_chunk_key(lod, cx, cy)):
					continue
				if aircraft_pos.distance_to(_chunk_center_godot(lod, cx, cy)) <= r_fetch:
					_load_chunk(lod, cx, cy)
	# Unload: free resident streamed chunks that have fallen outside R_unload.
	var to_free: Array = []
	for key: String in _loaded_chunks.keys():
		var parts := key.split("_")
		var lod := int(parts[0])
		if lod > top_lod:
			continue  # coarse LODs stay resident as the backdrop
		var cx := int(parts[1])
		var cy := int(parts[2])
		if aircraft_pos.distance_to(_chunk_center_godot(lod, cx, cy)) > _stream_radius_m(lod, unload_margin_m):
			to_free.append([lod, cx, cy])
	for t: Array in to_free:
		free_chunk(t[0], t[1], t[2])


## Recompute the per-LOD visibility band edges from the screen-space-error policy at the current
## viewport height and vertical FOV (terrain_lod_rendering.md §1; OQ-LR-1 Alternative 2).  Sets
## _lod_edges_m (band edges), _lod_begin_base / _lod_end_base (δ-adjusted, pre tile-radius padding).
func _recompute_lod_bands() -> void:
	if _lod_policy.is_empty():
		return
	var errs: Array = _lod_policy.get("transition_error_m", [])
	if errs.is_empty():
		return
	var tau: float = float(_lod_policy.get("tau_px", 1.0))
	var delta: float = float(_lod_policy.get("delta", 0.15))
	var h_px := _live_viewport_height()
	var vfov := _live_vertical_fov_rad()
	# R = eps * H / (2 * tau * tan(vfov/2));  k is the metres-per-metre-of-error scale.
	var k := h_px / (2.0 * tau * tan(vfov * 0.5))

	# Band edges: [0, R1, ..., R6, R7]; the coarsest outer edge is extrapolated geometrically.
	_lod_edges_m = [0.0]
	for e in errs:
		_lod_edges_m.append(float(e) * k)
	var n := _lod_edges_m.size()
	if n >= 2 and _lod_edges_m[n - 2] > 0.0:
		var ratio: float = _lod_edges_m[n - 1] / _lod_edges_m[n - 2]
		_lod_edges_m.append(_lod_edges_m[n - 1] + (_lod_edges_m[n - 1] - _lod_edges_m[n - 2]) * ratio)
	else:
		_lod_edges_m.append(_lod_edges_m[n - 1] * 4.0)

	# Per-LOD δ-adjusted band [edges[l]*(1-δ), edges[l+1]*(1+δ)]; coarsest end = 0 (infinite).
	_lod_begin_base = []
	_lod_end_base = []
	var lod_count := _lod_edges_m.size() - 1
	for lod in range(lod_count):
		_lod_begin_base.append(float(_lod_edges_m[lod]) * (1.0 - delta))
		if lod == lod_count - 1:
			_lod_end_base.append(0.0)  # coarsest LOD renders to the horizon
		else:
			_lod_end_base.append(float(_lod_edges_m[lod + 1]) * (1.0 + delta))


## Recompute the bands and re-apply them to every resident chunk when the viewport height or
## vertical FOV has changed (window resize / FOV change) — OQ-LR-1 Alternative 2.
func _maybe_recompute_lod_bands() -> void:
	if _lod_policy.is_empty() or _terrain_root == null:
		return
	var h := _live_viewport_height()
	var vfov := _live_vertical_fov_rad()
	if is_equal_approx(h, _last_view_h) and is_equal_approx(vfov, _last_view_fov):
		return
	_last_view_h = h
	_last_view_fov = vfov
	_recompute_lod_bands()
	for node in _loaded_chunks.values():
		if is_instance_valid(node):
			_apply_visibility_ranges(node)


## Live viewport height in pixels (falls back to the descriptor's reference height).
func _live_viewport_height() -> float:
	var vp := get_viewport()
	if vp == null:
		return float(_lod_policy.get("h_ref_px", 1080.0))
	return maxf(1.0, vp.get_visible_rect().size.y)


## Live vertical field of view (radians).  The camera uses KEEP_WIDTH (its `fov` is horizontal),
## so the vertical FOV is derived from the viewport aspect ratio.
func _live_vertical_fov_rad() -> float:
	if _camera == null:
		return float(_lod_policy.get("fov_ref_rad", deg_to_rad(90.0)))
	var fov_rad := deg_to_rad(_camera.fov)
	if _camera.keep_aspect == Camera3D.KEEP_HEIGHT:
		return fov_rad
	# KEEP_WIDTH: _camera.fov is the horizontal FOV; convert to vertical via the aspect ratio.
	var aspect := 1.0
	var vp := get_viewport()
	if vp != null:
		var sz := vp.get_visible_rect().size
		if sz.y > 0.0:
			aspect = sz.x / sz.y
	return 2.0 * atan(tan(fov_rad * 0.5) / maxf(0.01, aspect))


## Streaming radius for a LOD = its screen-space-error band outer edge (R_{lod+1}) + margin.
## The coarsest LOD has no finite outer edge; fall back to the camera far clip.
func _stream_radius_m(lod: int, margin_m: float) -> float:
	var outer := 0.0
	if lod + 1 < _lod_edges_m.size():
		outer = float(_lod_edges_m[lod + 1])
	if outer <= 0.0:
		outer = camera_far_m
	return outer + margin_m


## Center of chunk (lod, cx, cy) in Godot space (Y = 0, the world-origin plane).
func _chunk_center_godot(lod: int, cx: int, cy: int) -> Vector3:
	var size := _chunk_size_for_lod(lod)
	var east := (float(cx) + 0.5) * size
	var north := (float(cy) + 0.5) * size
	return Vector3(east, 0.0, -north)


## Resolve a project-root-relative terrain path to an absolute/res:// path usable by the
## file and glTF loaders (shared by the descriptor and single-GLB paths).
func _resolve_terrain_path(p: String) -> String:
	var resolved := p
	if not p.begins_with("res://") and not p.begins_with("/") and not (p.length() > 1 and p[1] == ":"):
		resolved = ProjectSettings.globalize_path("res://").path_join(p)
	return resolved.replace("\\", "/")


func _load_terrain_scene(glb_path: String) -> Node3D:
	# Resolve project-root-relative paths (not starting with res:// or an absolute path)
	# and normalize separators for Godot's loaders.
	var resolved := _resolve_terrain_path(glb_path)

	# res:// paths go through the pre-imported cache via ResourceLoader.
	# Absolute filesystem paths use GLTFDocument for runtime loading, because
	# ResourceLoader silently returns null for external GLB files when no
	# .import sidecar exists in the project.
	if resolved.begins_with("res://"):
		var packed: Resource = ResourceLoader.load(
			resolved, "", ResourceLoader.CACHE_MODE_IGNORE
		)
		if packed == null:
			push_error("TerrainLoader: failed to load GLB from %s" % resolved)
			return null
		if not (packed is PackedScene):
			push_error("TerrainLoader: resource at %s is not a PackedScene" % resolved)
			return null
		return (packed as PackedScene).instantiate() as Node3D
	else:
		print("TerrainLoader: using GLTFDocument for external path: %s" % resolved)
		var doc   := GLTFDocument.new()
		var state := GLTFState.new()
		var err: int = doc.append_from_file(resolved, state)
		if err != OK:
			print("TerrainLoader: GLTFDocument.append_from_file failed, error=%d" % err)
			push_error("TerrainLoader: GLTFDocument failed on %s (error %d)" % [resolved, err])
			return null
		print("TerrainLoader: GLTFDocument parsed OK, generating scene...")
		var node: Node = doc.generate_scene(state)
		if node == null:
			print("TerrainLoader: generate_scene returned null")
			return null
		print("TerrainLoader: generate_scene OK")
		return node as Node3D


## Load the aircraft mesh from aircraft_mesh_path in config and attach it to
## the Vehicle node as a child.  Applies body-frame correction (nose=+X ->
## Godot -Z forward) per OQ-LS-9 Option B, then scales to real wingspan.
func _load_aircraft_mesh(config: Dictionary) -> void:
	var mesh_path: String = config.get("aircraft_mesh_path", "")
	if mesh_path.is_empty():
		push_warning("TerrainLoader: no aircraft_mesh_path in terrain_config.json — skipping mesh load")
		return

	var packed: Resource = ResourceLoader.load(mesh_path, "", ResourceLoader.CACHE_MODE_IGNORE)
	if packed == null or not (packed is PackedScene):
		push_error("TerrainLoader: failed to load aircraft mesh from %s" % mesh_path)
		return

	var mesh_node: Node3D = (packed as PackedScene).instantiate() as Node3D
	mesh_node.name = "AircraftMesh"

	# Layout-frame → body-frame correction.
	# Aircraft mesh source files use the layout frame:
	#   +X = aft (increasing fuselage station), +Y = right buttline, +Z = up waterline.
	# SimulationReceiver applies r_ned_to_godot * q_b2n where q_b2n is the
	# body-to-NED quaternion.  r_ned_to_godot expects body frame (+X=fwd, +Y=right,
	# +Z=down).  Ry(180°) converts layout → body: +X→-X(fwd), +Y→+Y, +Z→-Z(down).
	# Result: nose (-X layout face) points Godot -Z (into scene), right (+Y) → +X (east),
	# up (+Z layout) → +Y (world up).  Chase camera at +Z sees the aft face.
	mesh_node.rotation_degrees = Vector3(0, 180, 0)

	var mi := _find_first_mesh_instance(mesh_node)
	if mi != null:
		var aabb := mi.get_aabb()

		# Scale mesh so its wingspan matches the real aircraft wingspan.
		# The wingspan spans local Y (right buttline); aabb.size.y is the full span.
		var mesh_wingspan_m: float = aabb.size.y
		if config.has("aircraft_wingspan_m") and mesh_wingspan_m > 0.001:
			var target_wingspan_m: float = float(config["aircraft_wingspan_m"])
			mesh_node.scale = Vector3.ONE * (target_wingspan_m / mesh_wingspan_m)

		if config.has("aircraft_cg_x_fraction"):
			aircraft_cg_x_fraction = float(config["aircraft_cg_x_fraction"])
		if config.has("aircraft_cg_z_fraction"):
			aircraft_cg_z_fraction = float(config["aircraft_cg_z_fraction"])

		# CG in mesh local space; transform through mi and mesh_node basis to world.
		var cg_local := Vector3(
			aabb.position.x + aircraft_cg_x_fraction * aabb.size.x,
			aabb.position.y + 0.5 * aabb.size.y,
			aabb.position.z + aircraft_cg_z_fraction * aabb.size.z)
		_aircraft_cg_prepos = mesh_node.basis * (mi.transform * cg_local)

		# Place mesh so its body CG coincides with the Vehicle node origin.
		# The physics simulation positions Vehicle at the aircraft CG; no
		# terrain-relative offset is applied here.
		_aircraft_center_world_y = 0.0  # CG is at Vehicle origin by definition
		mesh_node.position = -_aircraft_cg_prepos
		_aircraft_mesh_node = mesh_node

	_apply_material_to_tree(mesh_node, _aircraft_material)

	var vehicle := _find_vehicle(get_tree().root)
	if vehicle == null:
		push_error("TerrainLoader: Vehicle node not found — cannot attach aircraft mesh")
		return

	vehicle.add_child(mesh_node)


## Recursively walk node and set visibility_range_* on every MeshInstance3D
## whose name matches the tile_L{N}_* convention written by export_gltf.py.
##
## Godot culls by distance from camera to mesh node origin (centroid).  For large
## tiles the centroid may be far from the camera even when the camera is inside
## the tile bbox.  We compensate by adding the tile's AABB half-diagonal to both
## begin and end thresholds, converting centroid-distance culling to approximate
## nearest-edge-distance culling.
func _apply_visibility_ranges(node: Node) -> void:
	if node is MeshInstance3D:
		var lod := _parse_lod_from_name(node.name)
		if lod >= 0 and lod < _lod_begin_base.size():
			var mi := node as MeshInstance3D
			# Pad the per-LOD band by the tile's AABB half-diagonal so centroid-distance culling
			# approximates nearest-edge-distance culling (lod_culling_geometry.md).
			var radius := mi.get_aabb().size.length() * 0.5
			var end_base: float = _lod_end_base[lod]
			mi.visibility_range_begin = max(0.0, float(_lod_begin_base[lod]) - radius)
			mi.visibility_range_end   = (end_base + radius) if end_base > 0.0 else 0.0
			# Cross-fade self out (not in) at both edges so only one LOD level is
			# opaque at a time.  Without this, the hysteresis overlap band shows two
			# terrain meshes simultaneously — the coarser LOD visible as a second
			# ground layer below the finer one.
			mi.visibility_range_fade_mode = GeometryInstance3D.VISIBILITY_RANGE_FADE_SELF

	for child: Node in node.get_children():
		_apply_visibility_ranges(child)


## Parse LOD level from a node name of the form "tile_L{N}_...".
## Returns the integer LOD (0–6) or -1 if the name does not match.
func _parse_lod_from_name(node_name: String) -> int:
	if not node_name.begins_with("tile_L") or node_name == "tile_L0_flat":
		return -1
	var rest := node_name.substr(6)  # characters after "tile_L"
	var underscore := rest.find("_")
	var lod_str: String
	if underscore < 0:
		lod_str = rest
	else:
		lod_str = rest.substr(0, underscore)
	if not lod_str.is_valid_int():
		return -1
	var lod := lod_str.to_int()
	if lod < 0 or lod > 6:
		return -1
	return lod


## Depth-first search for the first MeshInstance3D under node.
func _find_first_mesh_instance(node: Node) -> MeshInstance3D:
	if node is MeshInstance3D:
		return node as MeshInstance3D
	for child: Node in node.get_children():
		var result := _find_first_mesh_instance(child)
		if result != null:
			return result
	return null


## Depth-first search for the Vehicle node by name.
func _find_vehicle(node: Node) -> Node3D:
	if node.name == "Vehicle" and node is Node3D:
		return node as Node3D
	for child: Node in node.get_children():
		var result := _find_vehicle(child)
		if result != null:
			return result
	return null


# LS-T8: _set_world_origin and _find_simulation_receiver removed.  The
# simulation-side GodotEnuProjector (in live_sim.cpp) reads the world origin
# directly from terrain_config.json; the receiver no longer needs it.
