## SimulationReceiver — GDScript UDP listener for LiteAero Sim telemetry.
##
## Design authority: docs/architecture/live_sim_view.md §Godot Plugin Design
##
## Polls a UDP socket each _process() frame, deserializes SimulationFrameProto
## datagrams, converts geodetic position to ENU offset from the world origin,
## converts body-to-NED quaternion to body-to-Godot quaternion, and drives the
## parent Vehicle node transform.
##
## NOTE: This GDScript implementation is a functional placeholder.  A GDExtension
## C++ implementation may replace it for production use (access to protobuf C++
## runtime).  For now, datagrams are raw-binary decoded using the known fixed
## field layout of SimulationFrameProto (fields 1-11, all double/float, no
## repeated or nested fields).
##
## Coordinate mapping (ENU -> Godot):
##   Godot +X = East
##   Godot +Y = Up
##   Godot +Z = -North
##
## To use: attach this script to a Node3D inside the Vehicle node.  Set
## broadcast_port to match the port used by live_sim.py or live_sim.exe.

extends Node3D

## UDP port to listen on (must match the broadcaster port).
@export var broadcast_port: int = 14560

## Maximum datagrams to drain per _process() frame (bounds latency spike).
@export var max_datagrams_per_frame: int = 10

# LS-T8: viewer-projected position arrives in proto fields 14-16 (viewer_x_m,
# viewer_y_m, viewer_z_m), computed simulation-side by GodotEnuProjector.
# This receiver no longer needs the world origin or any geodetic math.

var _socket: PacketPeerUDP = null
var _hud_label: Label = null

# ---------------------------------------------------------------------------

func _ready() -> void:
	_socket = PacketPeerUDP.new()
	var err := _socket.bind(broadcast_port, "127.0.0.1")
	if err != OK:
		push_error("SimulationReceiver: failed to bind UDP port %d (error %d)" % [broadcast_port, err])
		_socket = null
	else:
		print("SimulationReceiver: listening on port %d" % broadcast_port)
	_create_hud()


func _create_hud() -> void:
	var canvas := CanvasLayer.new()
	canvas.name = "HudOverlay"
	get_tree().root.add_child(canvas)

	var label := Label.new()
	label.name = "HudLabel"
	label.add_theme_font_size_override("font_size", 20)
	label.add_theme_color_override("font_color", Color(0.0, 1.0, 0.2, 1.0))
	label.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	label.set_anchor(SIDE_LEFT,   1.0)
	label.set_anchor(SIDE_RIGHT,  1.0)
	label.set_anchor(SIDE_TOP,    0.0)
	label.set_anchor(SIDE_BOTTOM, 0.0)
	label.set_offset(SIDE_LEFT,  -230.0)
	label.set_offset(SIDE_RIGHT, -10.0)
	label.set_offset(SIDE_TOP,    20.0)
	label.set_offset(SIDE_BOTTOM, 140.0)
	canvas.add_child(label)
	_hud_label = label

# ---------------------------------------------------------------------------

func _process(_delta: float) -> void:
	if _socket == null:
		return

	var frames_processed := 0
	while _socket.get_available_packet_count() > 0 and frames_processed < max_datagrams_per_frame:
		var data: PackedByteArray = _socket.get_packet()
		if data.size() == 0:
			break
		_apply_frame(data)
		frames_processed += 1

# ---------------------------------------------------------------------------

## Parse a SimulationFrameProto binary datagram and apply to Vehicle transform.
##
## SimulationFrameProto wire format (proto3, fields 1-16, all scalar).  LS-T8
## reads the viewer-projected position from fields 14-16 directly; the
## simulation-side GodotEnuProjector does the curvature-aware ECEF -> ENU
## projection so this receiver does not need to know the world origin.
##   field 1  (double): timestamp_s
##   field 4  (float):  height_wgs84_m       (HUD only)
##   field 5..8 (float): q_w/q_x/q_y/q_z     (body-to-NED quaternion)
##   field 11 (float):  velocity_down_mps    (HUD only; V/S = -down)
##   field 12 (float):  airspeed_mps         (HUD only)
##   field 13 (float):  agl_m                (HUD only)
##   field 14 (float):  viewer_x_m
##   field 15 (float):  viewer_y_m
##   field 16 (float):  viewer_z_m
func _apply_frame(data: PackedByteArray) -> void:
	var parsed := _parse_proto(data)
	if parsed.is_empty():
		return

	var h_m: float          = parsed.get(4, 0.0)
	var q_w: float          = parsed.get(5, 1.0)
	var q_x: float          = parsed.get(6, 0.0)
	var q_y: float          = parsed.get(7, 0.0)
	var q_z: float          = parsed.get(8, 0.0)
	var vs_down_mps: float  = parsed.get(11, 0.0)
	var airspeed_mps: float = parsed.get(12, 0.0)
	var agl_m: float        = parsed.get(13, -1.0)
	var viewer_x: float     = parsed.get(14, 0.0)
	var viewer_y: float     = parsed.get(15, 0.0)
	var viewer_z: float     = parsed.get(16, 0.0)
	var h_msl_m: float      = parsed.get(17, h_m)  # MSL via EGM2008; falls back to h_WGS84

	get_parent().position = Vector3(viewer_x, viewer_y, viewer_z)

	# Body-to-NED quaternion -> body-to-Godot quaternion.
	# NED->Godot rotation: North->-Z, East->+X, Down->-Y.
	# R_NED_to_Godot applied as: q_Godot = R * q_b2n
	# R in quaternion form (90-deg rotation about X followed by 180-deg about Y, approx):
	# Exact matrix from design doc:
	#   [  0   1   0 ]       =>  quaternion: w=0.5, x=0.5, y=0.5, z=-0.5
	#   [  0   0  -1 ]
	#   [ -1   0   0 ]
	var r := Quaternion(0.5, 0.5, 0.5, -0.5)
	var q_b2n := Quaternion(q_w, q_x, q_y, q_z).normalized()
	get_parent().quaternion = (r * q_b2n).normalized()

	if _hud_label != null:
		var spd_kt := int(airspeed_mps * 1.94384)
		# ALT is MSL (orthometric) via EGM2008.
		var alt_ft := int(h_msl_m * 3.28084)
		var agl_line: String
		if agl_m >= 0.0:
			agl_line = "AGL  %d ft" % int(agl_m * 3.28084)
		else:
			agl_line = "AGL  ---"
		# Vertical speed: telemetry down-velocity (field 11); climb positive.
		var vs_fps := int(round(-vs_down_mps * 3.28084))
		_hud_label.text = "SPD  %d kt\nALT  %d ft\n%s\nV/S  %+d fps" % [spd_kt, alt_ft, agl_line, vs_fps]

# ---------------------------------------------------------------------------
## Minimal proto3 varint + wire-type decoder.
## Returns a Dictionary mapping field_number -> float value.
## Only handles wire types 1 (64-bit double) and 5 (32-bit float).
func _parse_proto(data: PackedByteArray) -> Dictionary:
	var result := {}
	var pos := 0
	while pos < data.size():
		# Read varint tag.
		var tag := 0
		var shift := 0
		while pos < data.size():
			var b: int = data[pos]
			pos += 1
			tag |= (b & 0x7F) << shift
			shift += 7
			if (b & 0x80) == 0:
				break
		var field_number := tag >> 3
		var wire_type    := tag & 0x07
		if wire_type == 1:  # 64-bit (double)
			if pos + 8 > data.size():
				break
			var bytes64 := data.slice(pos, pos + 8)
			result[field_number] = bytes64.decode_double(0)
			pos += 8
		elif wire_type == 5:  # 32-bit (float)
			if pos + 4 > data.size():
				break
			var bytes32 := data.slice(pos, pos + 4)
			result[field_number] = bytes32.decode_float(0)
			pos += 4
		else:
			# Unknown wire type — stop parsing.
			break
	return result
