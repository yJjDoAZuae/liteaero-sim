"""gen_test_assets.py — Generate test assets for coordinate-system verification.

Creates (real godot/terrain/ is NEVER modified):
  python/assets/test_prism.obj        — 3 x 2 x 1 m prism, centroid at origin,
  python/assets/test_prism.glb          axis labels + arrows embossed on each face.
  godot/assets/test_prism.glb         — copy for res:// access.
  godot/terrain_test/terrain/terrain.glb      — 1000 x 1000 m flat tile, Y = 0 in ENU space.
  godot/terrain_test/terrain/terrain.las_terrain — flat terrain for live_sim physics.
  godot/terrain_test/terrain/terrain_config.json — test config (world origin lat=0,lon=0,h=10 m).
  configs/test_axis_prism.json        — aircraft config with 1 m landing gear height.

Coordinate conventions
----------------------
Prism mesh (aircraft body frame, matches gen_aircraft_mesh.py):
  +X  nose / length  (3 m extent, spans [-1.5, +1.5])
  +Y  right wing     (2 m extent, spans [-1.0, +1.0])
  +Z  up             (1 m extent, spans [-0.5, +0.5])
  centroid at (0, 0, 0)

Flat terrain GLB (matches export_gltf.py convention):
  glTF X = ENU East,  glTF Y = ENU Up,  glTF Z = -ENU North

Expected result in Godot
------------------------
World origin: lat=0, lon=0, ellipsoid height=10 m.
Terrain surface: world Y = 0  (= ellipsoid height 10 m).
Aircraft gear height: 1.0 m.
Sim initial altitude: 11.0 m (= 10 + 1) → Vehicle.y = 1.0 m.
Prism centroid (CG) = Vehicle origin → world Y = 1.0 m.
Prism bottom face: world Y = 0.5 m  (ellipsoid 10.5 m).
Prism top face:    world Y = 1.5 m  (ellipsoid 11.5 m).  ← verify this in Godot.
"""

from __future__ import annotations

import json
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

# Allow importing from sibling terrain/ package.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from terrain.las_terrain import TerrainTileData, write_las_terrain  # noqa: E402

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

_ROOT = Path(__file__).resolve().parents[2]
_OPENSCAD_EXE = Path("C:/Program Files/OpenSCAD/openscad.exe")

_ASSETS_DIR       = _ROOT / "python" / "assets"
_GODOT_TERRAIN    = _ROOT / "godot" / "terrain"               # real terrain — never written
_TEST_GODOT_DIR   = _ROOT / "godot" / "terrain_test"          # mirrors godot/ root for LITEAERO_GODOT_DIR
_TEST_TERRAIN_DIR = _TEST_GODOT_DIR / "terrain"               # terrain_test/terrain/ mirrors godot/terrain/
_CONFIGS_DIR      = _ROOT / "configs"


# ---------------------------------------------------------------------------
# 1. Test prism — via OpenSCAD
# ---------------------------------------------------------------------------

_SCAD_SOURCE = """\
// Test prism: 3 x 2 x 1 m, centroid at origin.
// Body-frame axes: +X = nose (length), +Y = right wing (width), +Z = up (height).
// Embossed labels on each face show axis associations and positive directions.

depth  = 0.04;   // emboss relief depth
tsize  = 0.18;   // text size (m)
arrow  = 0.10;   // arrowhead size

// Main box: centred at origin.
difference() {
    cube([3, 2, 1], center=true);

    // ---- +Z face (top, Z=+0.5): shows +X and +Y axes ----
    translate([0, 0, 0.5 - depth])
        linear_extrude(depth + 0.01) {
            // label
            translate([-0.6, 0.1, 0]) text("+Z face", size=tsize, halign="center");
            translate([-0.6, -0.15, 0]) text("(up)", size=tsize*0.8, halign="center");
            // +X arrow (pointing in local X on this face)
            translate([0.0, -0.5, 0]) {
                square([0.5, 0.03], center=true);
                translate([0.25, 0, 0]) polygon([[0,0],[-arrow,arrow/2],[-arrow,-arrow/2]]);
            }
            translate([0.28, -0.5+0.06, 0]) text("+X", size=tsize*0.8);
            // +Y arrow (pointing in local Y on this face)
            translate([-0.6, 0.0, 0]) {
                square([0.03, 0.5], center=true);
                translate([0, 0.25, 0]) polygon([[0,0],[arrow/2,-arrow],[-arrow/2,-arrow]]);
            }
            translate([-0.6+0.06, 0.28, 0]) text("+Y", size=tsize*0.8);
        }

    // ---- -Z face (bottom, Z=-0.5): shows +X and +Y axes ----
    translate([0, 0, -0.5])
        linear_extrude(depth + 0.01) {
            translate([0.6, 0.1, 0]) mirror([1,0,0]) text("-Z face", size=tsize, halign="center");
            translate([0.6, -0.15, 0]) mirror([1,0,0]) text("(down)", size=tsize*0.8, halign="center");
        }

    // ---- +X face (nose, X=+1.5): shows +Y and +Z axes ----
    translate([1.5 - depth, 0, 0])
        rotate([0, -90, 0])
        linear_extrude(depth + 0.01) {
            translate([0, 0.2, 0]) text("+X nose", size=tsize, halign="center");
            // +Y arrow
            translate([0.0, -0.3, 0]) {
                square([0.5, 0.03], center=true);
                translate([0.25, 0, 0]) polygon([[0,0],[-arrow,arrow/2],[-arrow,-arrow/2]]);
            }
            translate([0.28, -0.3+0.06, 0]) text("+Y", size=tsize*0.8);
            // +Z arrow
            translate([-0.5, 0.0, 0]) {
                square([0.03, 0.4], center=true);
                translate([0, 0.2, 0]) polygon([[0,0],[arrow/2,-arrow],[-arrow/2,-arrow]]);
            }
            translate([-0.5+0.06, 0.22, 0]) text("+Z", size=tsize*0.8);
        }

    // ---- -X face (tail, X=-1.5): shows +Y and +Z axes ----
    translate([-1.5, 0, 0])
        rotate([0, 90, 0])
        linear_extrude(depth + 0.01) {
            translate([0, 0.2, 0]) text("-X tail", size=tsize, halign="center");
        }

    // ---- +Y face (right wing, Y=+1.0): shows +X and +Z axes ----
    translate([0, 1.0 - depth, 0])
        rotate([90, 0, 0])
        linear_extrude(depth + 0.01) {
            translate([0, 0.2, 0]) text("+Y R.wing", size=tsize, halign="center");
            // +X arrow
            translate([0.0, -0.3, 0]) {
                square([0.5, 0.03], center=true);
                translate([0.25, 0, 0]) polygon([[0,0],[-arrow,arrow/2],[-arrow,-arrow/2]]);
            }
            translate([0.28, -0.3+0.06, 0]) text("+X", size=tsize*0.8);
            // +Z arrow
            translate([-0.6, 0.0, 0]) {
                square([0.03, 0.4], center=true);
                translate([0, 0.2, 0]) polygon([[0,0],[arrow/2,-arrow],[-arrow/2,-arrow]]);
            }
            translate([-0.6+0.06, 0.22, 0]) text("+Z", size=tsize*0.8);
        }

    // ---- -Y face (left wing, Y=-1.0): shows +X and +Z axes ----
    translate([0, -1.0, 0])
        rotate([-90, 0, 0])
        linear_extrude(depth + 0.01) {
            translate([0, 0.2, 0]) text("-Y L.wing", size=tsize, halign="center");
        }
}
"""

_SCAD_SOURCE_SIMPLE = """\
// Test prism: 3 x 2 x 1 m, centroid at origin.
// Layout frame: +X=aft(increasing FS, 3m), +Y=right buttline(2m), +Z=up waterline(1m).
// Nose is at the -X end; tail at +X end.
//
// Each face is placed with multmatrix(M) where the columns of M are:
//   col0 = world direction of local 2D +X  ("right"  when viewed from outside)
//   col1 = world direction of local 2D +Y  ("up"     when viewed from outside)
//   col2 = world direction of extrusion     (outward normal = away from solid)
//   col3 = translation to face surface minus depth
//
// face_content() is the single annotation module used for all six faces.
// h_axis / v_axis name the body axes that point right / up from outside.
//
// Axis assignments (verified against body frame):
//   +Z top    : right=+X  up=+Y   normal=+Z
//   -Z bottom : right=+X  up=+Y   normal=-Z  (same as top — X/Y axes don't flip)
//   +X nose   : right=+Y  up=+Z   normal=+X
//   -X tail   : right=+Y  up=+Z   normal=-X  (same in-plane axes, opposite normal)
//   +Y R.wing : right=+X  up=+Z   normal=+Y
//   -Y L.wing : right=+X  up=+Z   normal=-Y

depth = 0.35;
ts    = 0.22;
ta    = 0.16;

module arrow_up(len=0.24) {
    hw = 0.04;
    square([0.016, len]);
    translate([0, len]) polygon([[-hw/2,0],[hw/2,0],[0,hw*0.9]]);
}

module arrow_right(len=0.24) {
    hw = 0.04;
    square([len, 0.016]);
    translate([len, 0]) polygon([[0,-hw/2],[0,hw/2],[hw*0.9,0]]);
}

module face_content(face_id, sub, h_axis, v_axis) {
    linear_extrude(depth + 0.01) {
        translate([0,  0.22]) text(face_id, size=ts, halign="center", valign="center");
        translate([0,  0.07]) text(sub,     size=ta, halign="center", valign="center");
        // h_axis arrow — horizontal, pointing right, centred on lower-right area
        translate([-0.10, -0.28]) {
            arrow_right(0.24);
            translate([0.26, 0.02]) text(h_axis, size=ta, valign="center");
        }
        // v_axis arrow — vertical, pointing up, left side
        translate([-0.36, -0.28]) {
            arrow_up(0.24);
            translate([0.03, 0.26]) text(v_axis, size=ta);
        }
    }
}

// M(right, up, normal, pos) builds a 4x4 column-major matrix for multmatrix.
// right/up/normal are [x,y,z] world vectors; pos is the translation.
function M(r,u,n,p) = [
    [r[0], u[0], n[0], p[0]],
    [r[1], u[1], n[1], p[1]],
    [r[2], u[2], n[2], p[2]],
    [0,    0,    0,    1   ]
];

difference() {
    cube([3, 2, 1], center=true);

    // +Z top:    right=+X=[1,0,0]  up=+Y=[0,1,0]  normal=+Z=[0,0,1]
    multmatrix(M([1,0,0],[0,1,0],[0,0,1],[0,0,0.5-depth]))
        face_content("+Z","top","+X","+Y");

    // All (right, up, normal) chosen so det(r,u,n)=+1 — verified with numpy.
    // h_axis/v_axis name the positive body axis each arrow points toward.

    // -Z bottom: right=+Y up=+X normal=-Z  det=+1
    multmatrix(M([0,1,0],[1,0,0],[0,0,-1],[0,0,-0.5+depth]))
        face_content("-Z","bottom","+Y","+X");

    // +X aft(tail):  right=+Y up=+Z normal=+X  det=+1
    multmatrix(M([0,1,0],[0,0,1],[1,0,0],[1.5-depth,0,0]))
        face_content("+X","aft","+Y","+Z");

    // -X fwd(nose):  right=+Z up=+Y normal=-X  det=+1
    multmatrix(M([0,0,1],[0,1,0],[-1,0,0],[-1.5+depth,0,0]))
        face_content("-X","fwd","+Z","+Y");

    // +Y R.wing: right=+Z up=+X normal=+Y  det=+1
    multmatrix(M([0,0,1],[1,0,0],[0,1,0],[0,1.0-depth,0]))
        face_content("+Y","R.wing","+Z","+X");

    // -Y L.wing: right=+X up=+Z normal=-Y  det=+1
    multmatrix(M([1,0,0],[0,0,1],[0,-1,0],[0,-1.0+depth,0]))
        face_content("-Y","L.wing","+X","+Z");
}
"""


def _generate_prism_glb(dst_glb: Path) -> None:
    """Generate a 3x2x1 m prism via OpenSCAD with axis labels embossed on each face.

    Body frame: +X=nose(length), +Y=right wing(width), +Z=up(height).
    Centroid at origin.  Labels are geometry — visible independent of any shader.
    """
    print("Generating test prism via OpenSCAD ...")
    import trimesh  # type: ignore

    scad_file = Path(tempfile.mktemp(suffix=".scad"))
    stl_file  = Path(tempfile.mktemp(suffix=".stl"))
    try:
        scad_file.write_text(_SCAD_SOURCE_SIMPLE, encoding="utf-8")
        result = subprocess.run(
            [str(_OPENSCAD_EXE), "-o", str(stl_file), str(scad_file)],
            capture_output=True, text=True,
        )
        if result.returncode != 0:
            raise RuntimeError(f"OpenSCAD failed:\n{result.stderr}")

        mesh = trimesh.load(str(stl_file), force="mesh", process=False)
        # Force flat shading: unshare vertices and embed explicit face normals.
        # Without explicit normals, Godot's importer smooths across edges.
        flat_verts = mesh.vertices[mesh.faces].reshape(-1, 3)
        flat_faces = np.arange(len(flat_verts), dtype=np.int32).reshape(-1, 3)
        face_normals = np.repeat(mesh.face_normals, 3, axis=0)  # one per vertex
        mesh = trimesh.Trimesh(
            vertices=flat_verts,
            faces=flat_faces,
            vertex_normals=face_normals,
            process=False,
        )
        print(f"  Prism bounds:"
              f"\n    X [{mesh.bounds[0,0]:.3f}, {mesh.bounds[1,0]:.3f}]"
              f"\n    Y [{mesh.bounds[0,1]:.3f}, {mesh.bounds[1,1]:.3f}]"
              f"\n    Z [{mesh.bounds[0,2]:.3f}, {mesh.bounds[1,2]:.3f}]")
        print(f"  Centroid: {mesh.centroid}")

        dst_glb.parent.mkdir(parents=True, exist_ok=True)

        # OBJ for external inspection.
        dst_obj = dst_glb.with_suffix(".obj")
        mesh.export(str(dst_obj))
        print(f"  OBJ: {dst_obj}")

        scene = trimesh.scene.scene.Scene(geometry={"prism": mesh})
        scene.export(str(dst_glb))
        print(f"  GLB: {dst_glb}")
    finally:
        scad_file.unlink(missing_ok=True)
        stl_file.unlink(missing_ok=True)


# ---------------------------------------------------------------------------
# 2. Flat terrain GLB
# ---------------------------------------------------------------------------

def _generate_flat_terrain_glb(dst: Path, size_m: float = 1000.0) -> None:
    """Single flat quad at Y=0 in ENU/glTF space (X=East, Y=Up, Z=-North)."""
    print("Generating flat terrain GLB ...")
    import trimesh  # type: ignore

    half = size_m / 2.0
    # Two triangles forming a flat quad in the XZ plane (Y=0).
    # ENU: X=East, Y=Up, Z=-North → flat ground at Y=0.
    verts = np.array([
        [-half, 0.0, -half],
        [ half, 0.0, -half],
        [ half, 0.0,  half],
        [-half, 0.0,  half],
    ], dtype=np.float32)
    # CCW winding viewed from above (+Y) so normal points +Y (upward).
    faces = np.array([[0, 2, 1], [0, 3, 2]], dtype=np.int32)

    # Flat green-ish vertex color.
    colors = np.array([[80, 120, 60, 255]] * 4, dtype=np.uint8)

    mesh = trimesh.Trimesh(
        vertices=verts,
        faces=faces,
        vertex_colors=colors,
    )
    dst.parent.mkdir(parents=True, exist_ok=True)

    # Wrap in a scene so export produces a multi-node GLB that TerrainLoader can load.
    scene = trimesh.scene.scene.Scene(geometry={"tile_L0_flat": mesh})
    scene.export(str(dst))
    print(f"  Flat terrain GLB: {dst}")


# ---------------------------------------------------------------------------
# 3. Flat .las_terrain (for live_sim physics)
# ---------------------------------------------------------------------------

def _generate_flat_las_terrain(dst: Path, size_m: float = 1000.0, height_m: float = 10.0) -> None:
    """Write a flat 1000×1000 m terrain tile at ellipsoid height height_m.

    Vertices are ENU offsets from the centroid (centroid at lat=0,lon=0,h=height_m),
    so up-offset = 0.0 for all vertices.
    """
    print("Generating flat .las_terrain ...")
    half = size_m / 2.0
    # Four corners, flat (up-offset = 0).
    vertices = np.array([
        [-half, -half, 0.0],
        [ half, -half, 0.0],
        [ half,  half, 0.0],
        [-half,  half, 0.0],
    ], dtype=np.float32)
    indices = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32)
    colors  = np.array([[80, 120, 60], [80, 120, 60]], dtype=np.uint8)

    # 1 degree in radians spans ~111 km; half-tile in radians.
    half_rad = (size_m / 2.0) / 6_378_137.0

    tile = TerrainTileData(
        lod=0,
        centroid_lat_rad=0.0,
        centroid_lon_rad=0.0,
        centroid_height_m=height_m,
        lat_min_rad=-half_rad,
        lat_max_rad= half_rad,
        lon_min_rad=-half_rad,
        lon_max_rad= half_rad,
        height_min_m=height_m,
        height_max_m=height_m,
        vertices=vertices,
        indices=indices,
        colors=colors,
    )

    dst.parent.mkdir(parents=True, exist_ok=True)
    write_las_terrain(dst, [tile])
    print(f"  Flat .las_terrain: {dst}")


# ---------------------------------------------------------------------------
# 4. terrain_config.json
# ---------------------------------------------------------------------------

def _write_terrain_config(dst: Path, prism_glb_res: str, las_terrain_path: Path) -> None:
    cfg = {
        "schema_version": 1,
        "dataset_name": "test_axis_prism",
        "glb_path": "res://terrain/terrain.glb",
        "world_origin_lat_rad": 0.0,
        "world_origin_lon_rad": 0.0,
        "world_origin_height_m": 10.0,
        "aircraft_mesh_path": prism_glb_res,
        "aircraft_wingspan_m": 2.0,
        "aircraft_gear_contact_height_m": 1.0,
        "aircraft_cg_x_fraction": 0.5,
        "aircraft_cg_z_fraction": 0.5,
        "las_terrain_path": str(las_terrain_path),
    }
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(json.dumps(cfg, indent=4))
    print(f"  terrain_config.json: {dst}")


# ---------------------------------------------------------------------------
# 4. Aircraft config (live_sim)
# ---------------------------------------------------------------------------

_TEST_AIRCRAFT_CONFIG = {
    "schema_version": 1,
    "aircraft": {
        "S_ref_m2": 2.0,
        "cl_y_beta": 0.0,
        "ar": 1.0,
        "e": 1.0,
        "cd0": 0.02,
        "cmd_filter_substeps": 1,
        "nz_wn_rad_s": 10.0,
        "nz_zeta_nd": 0.7,
        "ny_wn_rad_s": 10.0,
        "ny_zeta_nd": 0.7,
        "roll_rate_wn_rad_s": 10.0,
        "roll_rate_zeta_nd": 0.7,
        "qnw_min_turn_radius_m": 10.0,
        "ground_steering_min_turn_radius_m": 3.0,
        "ground_steering_vblend_lower_ratio": 0.6,
        "ground_steering_vblend_upper_ratio": 1.0,
    },
    "airframe": {
        "g_max_nd": 4.0,
        "g_min_nd": -2.0,
        "tas_max_mps": 50.0,
        "mach_max_nd": 0.15,
    },
    "inertia": {
        "mass_kg": 10.0,
        "Ixx_kgm2": 1.0,
        "Iyy_kgm2": 2.0,
        "Izz_kgm2": 3.0,
    },
    "lift_curve": {
        "cl_alpha": 5.7,
        "cl_max": 1.5,
        "cl_min": -0.8,
        "delta_alpha_stall": 0.175,
        "delta_alpha_stall_neg": 0.175,
        "cl_sep": 0.9,
        "cl_sep_neg": -0.5,
    },
    "landing_gear": {
        "substeps": 4,
        "wheel_units": [
            {
                "_comment": "Nose gear",
                "attach_point_body_m": [0.5, 0.0, 0.7],
                "travel_axis_body": [0.0, 0.0, 1.0],
                "spring_stiffness_npm": 5000.0,
                "damper_coeff_nspm": 200.0,
                "preload_n": 0.0,
                "travel_max_m": 0.15,
                "tyre_radius_m": 0.3,
                "tyre_cornering_stiffness_npm": 5000.0,
                "tyre_longitudinal_stiffness_npm": 8000.0,
                "rolling_resistance_nd": 0.02,
                "is_steerable": True,
                "has_brake": False,
            },
            {
                "_comment": "Left main gear",
                "attach_point_body_m": [-0.2, -0.8, 0.7],
                "travel_axis_body": [0.0, 0.0, 1.0],
                "spring_stiffness_npm": 8000.0,
                "damper_coeff_nspm": 400.0,
                "preload_n": 0.0,
                "travel_max_m": 0.15,
                "tyre_radius_m": 0.3,
                "tyre_cornering_stiffness_npm": 8000.0,
                "tyre_longitudinal_stiffness_npm": 10000.0,
                "rolling_resistance_nd": 0.02,
                "is_steerable": False,
                "has_brake": True,
            },
            {
                "_comment": "Right main gear",
                "attach_point_body_m": [-0.2, 0.8, 0.7],
                "travel_axis_body": [0.0, 0.0, 1.0],
                "spring_stiffness_npm": 8000.0,
                "damper_coeff_nspm": 400.0,
                "preload_n": 0.0,
                "travel_max_m": 0.15,
                "tyre_radius_m": 0.3,
                "tyre_cornering_stiffness_npm": 8000.0,
                "tyre_longitudinal_stiffness_npm": 10000.0,
                "rolling_resistance_nd": 0.02,
                "is_steerable": False,
                "has_brake": True,
            },
        ],
    },
    "visualization": {
        "mesh_res_path": "res://assets/test_prism.glb",
    },
    "initial_state": {
        "_comment": "At rest on ground: terrain=10 m, gear=1 m, CG altitude=11 m",
        "latitude_rad": 0.0,
        "longitude_rad": 0.0,
        "altitude_m": 11.0,
        "velocity_north_mps": 0.0,
        "velocity_east_mps": 0.0,
        "velocity_down_mps": 0.0,
        "wind_north_mps": 0.0,
        "wind_east_mps": 0.0,
        "wind_down_mps": 0.0,
    },
}


def _write_aircraft_config(dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(json.dumps(_TEST_AIRCRAFT_CONFIG, indent=4))
    print(f"  Aircraft config: {dst}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    import shutil

    print("=" * 60)
    print("gen_test_assets.py — coordinate system verification rig")
    print("=" * 60)
    print("Outputs go to godot/terrain_test/ — real terrain is NOT touched.")

    prism_glb = _ASSETS_DIR / "test_prism.glb"
    godot_prism_glb = _ROOT / "godot" / "assets" / "test_prism.glb"

    print("\n[1/5] Prism mesh")
    _generate_prism_glb(prism_glb)
    godot_prism_glb.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(prism_glb, godot_prism_glb)
    print(f"  Copied to: {godot_prism_glb}")

    print("\n[2/5] Flat terrain GLB")
    terrain_glb = _TEST_TERRAIN_DIR / "terrain.glb"
    _generate_flat_terrain_glb(terrain_glb, size_m=100.0)

    print("\n[3/5] Flat .las_terrain")
    las_path = _TEST_TERRAIN_DIR / "terrain.las_terrain"
    _generate_flat_las_terrain(las_path, size_m=100.0, height_m=10.0)

    print("\n[4/5] terrain_config.json")
    terrain_cfg = _TEST_TERRAIN_DIR / "terrain_config.json"
    _write_terrain_config(terrain_cfg, "res://assets/test_prism.glb", las_path)

    print("\n[5/5] Aircraft config")
    aircraft_cfg = _CONFIGS_DIR / "test_axis_prism.json"
    _write_aircraft_config(aircraft_cfg)

    print("\n" + "=" * 60)
    print("Done.  Expected result in Godot:")
    print("  Terrain surface:   world Y = 0.0 m  (ellipsoid 10.0 m)")
    print("  Aircraft CG:       world Y = 1.0 m  (ellipsoid 11.0 m)")
    print("  Prism bottom face: world Y = 0.5 m  (ellipsoid 10.5 m)")
    print("  Prism top face:    world Y = 1.5 m  (ellipsoid 11.5 m)  <- verify")
    print("")
    print("Run the sim with:")
    print('  $env:LITEAERO_GODOT_DIR = "godot/terrain_test"')
    print("  build\\tools\\live_sim.exe --config configs\\test_axis_prism.json")
    print('  $env:LITEAERO_GODOT_DIR = ""')
    print("=" * 60)


if __name__ == "__main__":
    main()
