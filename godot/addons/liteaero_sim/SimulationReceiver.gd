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

## World origin geodetic coordinates.  Set from terrain metadata or on first frame.
var world_origin_lat_rad: float = 0.0
var world_origin_lon_rad: float = 0.0
var world_origin_h_m: float     = 0.0
var world_origin_set: bool      = false

const EARTH_RADIUS_M := 6371000.0

var _socket: PacketPeerUDP = null

# ---------------------------------------------------------------------------

func _ready() -> void:
	_socket = PacketPeerUDP.new()
	var err := _socket.bind(broadcast_port, "127.0.0.1")
	if err != OK:
		push_error("SimulationReceiver: failed to bind UDP port %d (error %d)" % [broadcast_port, err])
		_socket = null
	else:
		print("SimulationReceiver: listening on port %d" % broadcast_port)

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
## SimulationFrameProto wire format (proto3, fields 1-11, all scalar):
##   field 1  (double): timestamp_s
##   field 2  (double): latitude_rad
##   field 3  (double): longitude_rad
##   field 4  (float):  height_wgs84_m
##   field 5  (float):  q_w
##   field 6  (float):  q_x
##   field 7  (float):  q_y
##   field 8  (float):  q_z
##   field 9  (float):  velocity_north_mps
##   field 10 (float):  velocity_east_mps
##   field 11 (float):  velocity_down_mps
func _apply_frame(data: PackedByteArray) -> void:
	var parsed := _parse_proto(data)
	if parsed.is_empty():
		return

	var lat_rad: float  = parsed.get(2, 0.0)
	var lon_rad: float  = parsed.get(3, 0.0)
	var h_m: float      = parsed.get(4, 0.0)
	var q_w: float      = parsed.get(5, 1.0)
	var q_x: float      = parsed.get(6, 0.0)
	var q_y: float      = parsed.get(7, 0.0)
	var q_z: float      = parsed.get(8, 0.0)

	# Set world origin to first received frame if not yet established.
	if not world_origin_set:
		world_origin_lat_rad = lat_rad
		world_origin_lon_rad = lon_rad
		world_origin_h_m     = h_m
		world_origin_set     = true

	# Geodetic -> ENU offset from world origin.
	var dlat := lat_rad - world_origin_lat_rad
	var dlon := lon_rad - world_origin_lon_rad
	var north_m := dlat * EARTH_RADIUS_M
	var east_m  := dlon * EARTH_RADIUS_M * cos(world_origin_lat_rad)
	var up_m    := h_m - world_origin_h_m

	# ENU -> Godot position: X=East, Y=Up, Z=-North
	get_parent().position = Vector3(east_m, up_m, -north_m)

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
