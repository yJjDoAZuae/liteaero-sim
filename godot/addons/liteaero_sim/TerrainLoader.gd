## TerrainLoader.gd — Programmatic terrain dataset loader.
##
## Design authority: docs/architecture/terrain_build.md §OQ-TB-2 and §OQ-TB-3
##                   docs/architecture/godot_plugin.md §TerrainLoader Integration
##
## Reads godot/terrain/terrain_config.json at scene start, loads the terrain GLB
## programmatically via ResourceLoader, instantiates it into the scene tree,
## applies per-node LOD visibility ranges, loads the aircraft mesh, and sets
## the world origin on SimulationReceiver — all before the first UDP packet arrives.
##
## Workflow after a terrain build:
##   1. Run build_terrain (Python).
##   2. Press Play in Godot.
##   No editor drag-and-drop or Inspector edits required.

extends Node

const _CONFIG_PATH := "res://terrain/terrain_config.json"

## LOD visibility range lower bounds (metres) — switch-to-finer hysteresis thresholds.
## Index = LOD level (0–6).  From terrain.md §LOD Hysteresis Band.
const _LOD_VIS_BEGIN_M: Array[float] = [
	0.0,      # L0
	255.0,    # L1
	765.0,    # L2
	2295.0,   # L3
	6885.0,   # L4
	20655.0,  # L5
	61965.0,  # L6
]

## LOD visibility range upper bounds (metres) — switch-to-coarser hysteresis thresholds.
## 0.0 means no upper limit (infinity in Godot's convention for visibility_range_end = 0).
const _LOD_VIS_END_M: Array[float] = [
	345.0,    # L0
	1035.0,   # L1
	3105.0,   # L2
	9315.0,   # L3
	27945.0,  # L4
	83835.0,  # L5
	0.0,      # L6  (infinity)
]

# ---------------------------------------------------------------------------

func _ready() -> void:
	var config := _load_config()
	if config.is_empty():
		return

	var terrain_node := _load_terrain_scene(config["glb_path"])
	if terrain_node == null:
		return

	_apply_visibility_ranges(terrain_node)
	add_child(terrain_node)
	_load_aircraft_mesh(config)
	_set_world_origin(config)

# ---------------------------------------------------------------------------

func _load_config() -> Dictionary:
	if not FileAccess.file_exists(_CONFIG_PATH):
		push_error(
			"TerrainLoader: terrain_config.json not found at %s — run build_terrain first"
			% _CONFIG_PATH
		)
		return {}

	var f := FileAccess.open(_CONFIG_PATH, FileAccess.READ)
	if f == null:
		push_error("TerrainLoader: cannot open %s" % _CONFIG_PATH)
		return {}

	var parsed: Variant = JSON.parse_string(f.get_as_text())
	if parsed == null or not (parsed is Dictionary):
		push_error("TerrainLoader: terrain_config.json is not a valid JSON object")
		return {}

	return parsed as Dictionary


func _load_terrain_scene(glb_path: String) -> Node3D:
	var packed: Resource = ResourceLoader.load(
		glb_path, "", ResourceLoader.CACHE_MODE_IGNORE
	)
	if packed == null:
		push_error("TerrainLoader: failed to load GLB from %s" % glb_path)
		return null

	if not (packed is PackedScene):
		push_error("TerrainLoader: resource at %s is not a PackedScene" % glb_path)
		return null

	return (packed as PackedScene).instantiate() as Node3D


## Load the aircraft mesh from aircraft_mesh_path in config and attach it to
## the Vehicle node as a child.  Applies body-frame correction (nose=+X ->
## Godot -Z forward) per OQ-LS-9 Option B.
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
	# Body-frame correction: nose=+X body frame -> Godot -Z (forward). See OQ-LS-9.
	mesh_node.rotation_degrees = Vector3(0, 90, 0)

	var vehicle := _find_vehicle(get_tree().root)
	if vehicle == null:
		push_error("TerrainLoader: Vehicle node not found — cannot attach aircraft mesh")
		return

	vehicle.add_child(mesh_node)


## Recursively walk node and set visibility_range_* on every MeshInstance3D
## whose name matches the tile_L{N}_* convention written by export_gltf.py.
func _apply_visibility_ranges(node: Node) -> void:
	if node is MeshInstance3D:
		var lod := _parse_lod_from_name(node.name)
		if lod >= 0:
			(node as MeshInstance3D).visibility_range_begin = _LOD_VIS_BEGIN_M[lod]
			(node as MeshInstance3D).visibility_range_end   = _LOD_VIS_END_M[lod]

	for child: Node in node.get_children():
		_apply_visibility_ranges(child)


## Parse LOD level from a node name of the form "tile_L{N}_...".
## Returns the integer LOD (0–6) or -1 if the name does not match.
func _parse_lod_from_name(node_name: String) -> int:
	if not node_name.begins_with("tile_L"):
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


## Depth-first search for the Vehicle node by name.
func _find_vehicle(node: Node) -> Node3D:
	if node.name == "Vehicle" and node is Node3D:
		return node as Node3D
	for child: Node in node.get_children():
		var result := _find_vehicle(child)
		if result != null:
			return result
	return null


## Set world origin on the SimulationReceiver node found in the scene tree.
## Supports both the native GDExtension type and the GDScript placeholder.
func _set_world_origin(config: Dictionary) -> void:
	var receiver := _find_simulation_receiver(get_tree().root)
	if receiver == null:
		push_warning("TerrainLoader: SimulationReceiver not found in scene tree")
		return

	var lat_rad := float(config.get("world_origin_lat_rad", 0.0))
	var lon_rad := float(config.get("world_origin_lon_rad", 0.0))
	var h_m     := float(config.get("world_origin_height_m", 0.0))

	if receiver.get_class() == "SimulationReceiver":
		# GDExtension native type — use the bound method.
		receiver.set_world_origin(lat_rad, lon_rad, h_m)
	else:
		# GDScript placeholder — direct property assignment.
		receiver.world_origin_lat_rad = lat_rad
		receiver.world_origin_lon_rad = lon_rad
		receiver.world_origin_h_m     = h_m
		receiver.world_origin_set     = true


## Depth-first search for the SimulationReceiver node.
## Detects both the native GDExtension type (by class name) and the GDScript
## placeholder (by script resource path) so TerrainLoader works in either state.
func _find_simulation_receiver(node: Node) -> Node:
	# Native GDExtension type — get_class() returns the registered C++ class name.
	if node.get_class() == "SimulationReceiver":
		return node
	# GDScript placeholder — check script path.
	var script: Script = node.get_script() as Script
	if script != null and script.resource_path.ends_with("SimulationReceiver.gd"):
		return node
	for child: Node in node.get_children():
		var result := _find_simulation_receiver(child)
		if result != null:
			return result
	return null
