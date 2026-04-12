"""build_terrain.py — Automated terrain dataset builder.

Produces a complete terrain dataset (all LODs, .las_terrain, GLB, Godot sidecar)
from a single aircraft configuration JSON file.

Design authority: docs/architecture/terrain_build.md
"""

from __future__ import annotations

import json
import logging
import math
import os
import shutil
from datetime import datetime, timezone
from pathlib import Path

from colorize import colorize
from download import DownloadError, default_cache_dir, download_dem, download_imagery
from export_gltf import export_gltf
from las_terrain import TerrainTileData, write_las_terrain
from mosaic import mosaic_dem, mosaic_imagery
from terrain_paths import derived_dir, gltf_path, las_terrain_dir, metadata_path, source_dir
from triangulate import lod_grid_spacing_deg, triangulate
from verify import MeshQualityError, check

_log = logging.getLogger("build_terrain")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Download chunk size: Sentinel Hub limit ≈ 2,500 × 2,500 px at L0 resolution.
_L0_RESOLUTION_DEG: float = 0.000090  # native Copernicus GLO-30 resolution
_MAX_CHUNK_SIDE_DEG: float = 2500 * _L0_RESOLUTION_DEG  # = 0.225 deg ≈ 25 km

# LOD cell side in degrees: 100 grid points × lod_grid_spacing_deg(N) per terrain.md.
# L5 and L6 use the same side as L4 (clamped to the coverage area in practice).
_LOD_CELL_SIDE_DEG: list[float] = [
    100 * lod_grid_spacing_deg(lod) for lod in range(5)
] + [
    100 * lod_grid_spacing_deg(4),  # L5: same as L4, ≥ 1 cell per region
    100 * lod_grid_spacing_deg(4),  # L6: same as L4, ≥ 1 cell per region
]

# Geographic partitioning: export radius per LOD (m).
# Equals the switch-to-coarser hysteresis threshold from terrain.md §LOD Hysteresis Band.
_LOD_EXPORT_RADIUS_M: list[float] = [
    345.0,
    1_035.0,
    3_105.0,
    9_315.0,
    27_945.0,
    83_835.0,
    math.inf,  # L6 always exported
]

# Godot visibility range bounds per LOD (m).
# From terrain.md §LOD Hysteresis Band.
_VIS_BEGIN_M: list[float] = [0.0, 255.0, 765.0, 2_295.0, 6_885.0, 20_655.0, 61_965.0]
_VIS_END_M: list[float | None] = [345.0, 1_035.0, 3_105.0, 9_315.0, 27_945.0, 83_835.0, None]

# Project root: parents[3] of python/tools/terrain/build_terrain.py = liteaero-sim/
_PROJECT_ROOT: Path = Path(__file__).resolve().parents[3]


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------

def build_terrain(
    aircraft_config_path: Path | str,
    *,
    name: str | None = None,
    center_lat_deg: float | None = None,
    center_lon_deg: float | None = None,
    radius_m: float | None = None,
    dem_source: str = "copernicus_dem_glo30",
    imagery_source: str = "sentinel2",
    force: bool = False,
) -> Path:
    """Build a complete terrain dataset for a live-sim aircraft configuration.

    Parameters
    ----------
    aircraft_config_path:
        Path to an aircraft configuration JSON file.
    name:
        Dataset name used as the directory key under ``data/terrain/``.
        Defaults to the config file's stem (e.g. ``general_aviation_ksba``).
    center_lat_deg:
        Geodetic latitude of the terrain tile centroid in decimal degrees.
        Overrides ``initial_state.latitude_rad`` from the config file.
    center_lon_deg:
        Longitude of the terrain tile centroid in decimal degrees.
        Overrides ``initial_state.longitude_rad`` from the config file.
    radius_m:
        Half-side of the square bounding box in metres.
        Overrides ``terrain.radius_km`` from the config file.
    dem_source:
        DEM data source.  One of ``"copernicus_dem_glo30"``, ``"nasadem"``, ``"srtm"``.
    imagery_source:
        Imagery source for facet colorization.  One of ``"sentinel2"``,
        ``"landsat9"``, ``"modis"``.
    force:
        If True, delete all derived outputs and rebuild from scratch.  Cached
        DEM/imagery files in ``python/cache/terrain/`` are retained.

    Returns
    -------
    Path
        Absolute path to the exported GLB file
        (``data/terrain/<name>/derived/gltf/terrain.glb``).

    Raises
    ------
    FileNotFoundError
        If ``aircraft_config_path`` does not exist.
    json.JSONDecodeError
        If the config file contains invalid JSON.
    ValueError
        If terrain radius cannot be determined (neither ``radius_m`` nor
        ``terrain.radius_km`` in config is supplied).
    DownloadError
        If a DEM or imagery download fails.
    MeshQualityError
        If a triangulated tile fails mesh quality thresholds.
    """
    aircraft_config_path = Path(aircraft_config_path)

    # Raises FileNotFoundError if absent, JSONDecodeError if malformed.
    config = json.loads(aircraft_config_path.read_text())

    dataset_name = name or aircraft_config_path.stem
    _log.info("%s: loading config from %s", dataset_name, aircraft_config_path)

    # -------------------------------------------------------------------
    # Parameter derivation
    # -------------------------------------------------------------------
    init = config["initial_state"]

    # Terrain centroid: CLI/caller override takes precedence over initial_state.
    if center_lat_deg is None:
        center_lat_deg = math.degrees(float(init["latitude_rad"]))
    if center_lon_deg is None:
        center_lon_deg = math.degrees(float(init["longitude_rad"]))
    center_lat_rad: float = math.radians(center_lat_deg)
    center_lon_rad: float = math.radians(center_lon_deg)
    center_h_m: float = float(init.get("altitude_m", 0.0))

    # Terrain radius: CLI/caller override takes precedence over config terrain section.
    if radius_m is None:
        radius_m = _radius_from_config(config)

    bbox_deg = _bbox_from_center(center_lat_deg, center_lon_deg, radius_m)

    _log.info(
        "%s: center %.3f°N %.3f°  radius %.1f km  bbox %s",
        dataset_name, center_lat_deg, center_lon_deg, radius_m / 1000.0,
        "%.3f,%.3f → %.3f,%.3f" % bbox_deg,
    )

    # -------------------------------------------------------------------
    # Directory setup
    # -------------------------------------------------------------------
    s_dir = source_dir(dataset_name)
    d_dir = derived_dir(dataset_name)
    lt_dir = las_terrain_dir(dataset_name)
    g_path = gltf_path(dataset_name)
    m_path = metadata_path(dataset_name)

    s_dir.mkdir(parents=True, exist_ok=True)
    d_dir.mkdir(parents=True, exist_ok=True)
    lt_dir.mkdir(parents=True, exist_ok=True)
    g_path.parent.mkdir(parents=True, exist_ok=True)

    if force:
        if lt_dir.exists():
            shutil.rmtree(lt_dir)
            lt_dir.mkdir(parents=True, exist_ok=True)
        if g_path.exists():
            g_path.unlink()
        if m_path.exists():
            m_path.unlink()

    # -------------------------------------------------------------------
    # Step 1: Download DEM and imagery in chunks
    # -------------------------------------------------------------------
    chunks = _compute_chunk_grid(bbox_deg)
    cache_dir = default_cache_dir()

    _log.info("%s: downloading DEM  %d chunk(s) ...", dataset_name, len(chunks))
    dem_chunk_paths: list[Path] = []
    for i, chunk in enumerate(chunks, 1):
        _log.info("%s: DEM chunk %d/%d ...", dataset_name, i, len(chunks))
        dem_chunk_paths.extend(download_dem(chunk, output_dir=cache_dir / "dem", source=dem_source))

    _log.info("%s: downloading imagery  %d chunk(s) ...", dataset_name, len(chunks))
    img_chunk_paths: list[Path] = []
    for i, chunk in enumerate(chunks, 1):
        _log.info("%s: imagery chunk %d/%d ...", dataset_name, i, len(chunks))
        img_chunk_paths.extend(
            download_imagery(chunk, output_dir=cache_dir / "imagery", source=imagery_source)
        )

    # -------------------------------------------------------------------
    # Step 2: Mosaic all chunks into seamless source rasters
    # -------------------------------------------------------------------
    _log.info("%s: mosaicking ...", dataset_name)
    dem_mosaic_path = s_dir / "dem_mosaic.tif"
    img_mosaic_path = s_dir / "img_mosaic.tif"
    mosaic_dem(dem_chunk_paths, dem_mosaic_path)
    mosaic_imagery(img_chunk_paths, img_mosaic_path)

    # -------------------------------------------------------------------
    # Step 3: Triangulate and colorize all LOD levels
    # -------------------------------------------------------------------
    all_tiles: list[TerrainTileData] = []

    for lod in range(7):
        cells = _compute_cell_grid(bbox_deg, lod=lod)
        _log.info(
            "%s: triangulating LOD %d  (%d cell(s)) ...", dataset_name, lod, len(cells)
        )
        for row, col, cell_bbox in cells:
            _log.debug("%s: LOD %d  row=%d col=%d ...", dataset_name, lod, row, col)
            tile = triangulate(dem_mosaic_path, cell_bbox, lod=lod)
            tile = colorize(tile, img_mosaic_path, source=imagery_source)
            check(tile)
            all_tiles.append(tile)

    # -------------------------------------------------------------------
    # Step 4: Write .las_terrain (all LODs)
    # -------------------------------------------------------------------
    lt_path = lt_dir / "terrain.las_terrain"
    _log.info("%s: writing .las_terrain  %d tiles ...", dataset_name, len(all_tiles))
    write_las_terrain(lt_path, all_tiles)

    # -------------------------------------------------------------------
    # Step 5: Select display tiles (geographic partitioning for Godot)
    # -------------------------------------------------------------------
    display_tiles = _select_display_tiles(all_tiles, center_lat_rad, center_lon_rad)
    if not display_tiles:
        raise RuntimeError(
            f"{dataset_name}: geographic partitioning produced no display tiles; "
            "check that the coverage area contains the center position"
        )
    _log.info(
        "%s: exporting GLB  %d display tile(s) ...", dataset_name, len(display_tiles)
    )

    # -------------------------------------------------------------------
    # Step 6: Export GLB
    # -------------------------------------------------------------------
    export_gltf(
        display_tiles,
        g_path,
        world_origin=(center_lat_rad, center_lon_rad, center_h_m),
    )

    # -------------------------------------------------------------------
    # Step 7: Write Godot sidecar files
    # -------------------------------------------------------------------
    godot_terrain_dir = _godot_terrain_dir()
    godot_terrain_dir.mkdir(parents=True, exist_ok=True)

    shutil.copy2(g_path, godot_terrain_dir / "terrain.glb")

    terrain_config = _build_terrain_config(
        config=config,
        dataset_name=dataset_name,
        center_lat_rad=center_lat_rad,
        center_lon_rad=center_lon_rad,
        center_h_m=center_h_m,
    )
    (godot_terrain_dir / "terrain_config.json").write_text(
        json.dumps(terrain_config, indent=4)
    )

    # -------------------------------------------------------------------
    # Step 8: Write metadata.json (written last; its presence signals success)
    # -------------------------------------------------------------------
    metadata = {
        "schema_version": 1,
        "dataset_name": dataset_name,
        "aircraft_config": str(aircraft_config_path.resolve()),
        "center_lat_deg": center_lat_deg,
        "center_lon_deg": center_lon_deg,
        "radius_m": radius_m,
        "lod_range": [0, 6],
        "dem_source": dem_source,
        "imagery_source": imagery_source,
        "dem_resolution_deg": _L0_RESOLUTION_DEG,
        "bbox_deg": list(bbox_deg),
        "build_timestamp_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
    }
    m_path.write_text(json.dumps(metadata, indent=4))

    _log.info("%s: build complete → %s", dataset_name, g_path)
    return g_path


# ---------------------------------------------------------------------------
# Internal helpers (also exposed for unit-testing parameter derivation)
# ---------------------------------------------------------------------------

def _radius_from_config(config: dict) -> float:
    """Return the terrain radius (m) from the config ``terrain.radius_km`` field.

    Raises ValueError if the field is absent.
    """
    terrain_section = config.get("terrain")
    if terrain_section is None or "radius_km" not in terrain_section:
        raise ValueError(
            "terrain radius not specified — add 'terrain.radius_km' to the "
            "aircraft config or pass --radius-km on the command line"
        )
    return float(terrain_section["radius_km"]) * 1000.0


def _bbox_from_center(
    center_lat_deg: float,
    center_lon_deg: float,
    radius_m: float,
) -> tuple[float, float, float, float]:
    """Return (lon_min, lat_min, lon_max, lat_max) in degrees.

    The bounding box is a square of half-side ``radius_m`` centred on
    (``center_lat_deg``, ``center_lon_deg``).
    """
    lat_deg_per_m = 1.0 / 111_132.0
    lon_deg_per_m = 1.0 / (111_132.0 * math.cos(math.radians(center_lat_deg)))
    delta_lat = radius_m * lat_deg_per_m
    delta_lon = radius_m * lon_deg_per_m
    return (
        center_lon_deg - delta_lon,  # lon_min
        center_lat_deg - delta_lat,  # lat_min
        center_lon_deg + delta_lon,  # lon_max
        center_lat_deg + delta_lat,  # lat_max
    )


def _compute_chunk_grid(
    bbox_deg: tuple[float, float, float, float],
) -> list[tuple[float, float, float, float]]:
    """Tile bbox_deg into download chunks of at most _MAX_CHUNK_SIDE_DEG per side.

    Returns a flat list of (lon_min, lat_min, lon_max, lat_max) chunk bboxes.
    """
    lon_min, lat_min, lon_max, lat_max = bbox_deg
    n_lon = max(1, math.ceil((lon_max - lon_min) / _MAX_CHUNK_SIDE_DEG))
    n_lat = max(1, math.ceil((lat_max - lat_min) / _MAX_CHUNK_SIDE_DEG))
    chunk_w = (lon_max - lon_min) / n_lon
    chunk_h = (lat_max - lat_min) / n_lat
    chunks: list[tuple[float, float, float, float]] = []
    for row in range(n_lat):
        for col in range(n_lon):
            clon_min = lon_min + col * chunk_w
            clon_max = lon_min + (col + 1) * chunk_w
            clat_min = lat_min + row * chunk_h
            clat_max = lat_min + (row + 1) * chunk_h
            chunks.append((clon_min, clat_min, clon_max, clat_max))
    return chunks


def _compute_cell_grid(
    bbox_deg: tuple[float, float, float, float],
    lod: int,
) -> list[tuple[int, int, tuple[float, float, float, float]]]:
    """Tile bbox_deg into LOD-appropriate cells.

    Returns a list of (row, col, cell_bbox_deg).  Cell size is
    _LOD_CELL_SIDE_DEG[lod]; all coverage is always included (≥ 1 cell).
    """
    lon_min, lat_min, lon_max, lat_max = bbox_deg
    cell_side = _LOD_CELL_SIDE_DEG[lod]
    n_lon = max(1, math.ceil((lon_max - lon_min) / cell_side))
    n_lat = max(1, math.ceil((lat_max - lat_min) / cell_side))
    cell_w = (lon_max - lon_min) / n_lon
    cell_h = (lat_max - lat_min) / n_lat
    cells: list[tuple[int, int, tuple[float, float, float, float]]] = []
    for row in range(n_lat):
        for col in range(n_lon):
            clon_min = lon_min + col * cell_w
            clon_max = lon_min + (col + 1) * cell_w
            clat_min = lat_min + row * cell_h
            clat_max = lat_min + (row + 1) * cell_h
            cells.append((row, col, (clon_min, clat_min, clon_max, clat_max)))
    return cells


def _geodetic_distance_m(
    lat1_rad: float, lon1_rad: float,
    lat2_rad: float, lon2_rad: float,
) -> float:
    """Equirectangular great-circle distance in metres (sufficient for LOD partitioning)."""
    dlat = lat2_rad - lat1_rad
    dlon = (lon2_rad - lon1_rad) * math.cos((lat1_rad + lat2_rad) * 0.5)
    return math.sqrt(dlat**2 + dlon**2) * 6_371_000.0


def _select_display_tiles(
    tiles: list[TerrainTileData],
    center_lat_rad: float,
    center_lon_rad: float,
) -> list[TerrainTileData]:
    """Return the subset of tiles within the geographic export radius for each LOD.

    Tiles within _LOD_EXPORT_RADIUS_M[tile.lod] of (center_lat_rad, center_lon_rad)
    are included.  L6 tiles are always included (export radius = infinity).
    """
    display: list[TerrainTileData] = []
    for tile in tiles:
        radius = _LOD_EXPORT_RADIUS_M[tile.lod]
        if math.isinf(radius):
            display.append(tile)
            continue
        dist_m = _geodetic_distance_m(
            center_lat_rad, center_lon_rad,
            tile.centroid_lat_rad, tile.centroid_lon_rad,
        )
        if dist_m <= radius:
            display.append(tile)
    return display


_DEFAULT_MESH_RES_PATH = "res://assets/aircraft_lp.glb"


def _build_terrain_config(
    config: dict,
    dataset_name: str,
    center_lat_rad: float,
    center_lon_rad: float,
    center_h_m: float,
) -> dict:
    """Return the terrain_config.json dict for the Godot sidecar file.

    Reads ``visualization.mesh_res_path`` from *config* to populate
    ``aircraft_mesh_path``.  Falls back to ``_DEFAULT_MESH_RES_PATH`` when
    the field is absent.
    """
    mesh_res_path: str = (
        config.get("visualization", {}).get("mesh_res_path", _DEFAULT_MESH_RES_PATH)
        or _DEFAULT_MESH_RES_PATH
    )
    return {
        "schema_version": 1,
        "dataset_name": dataset_name,
        "glb_path": "res://terrain/terrain.glb",
        "world_origin_lat_rad": center_lat_rad,
        "world_origin_lon_rad": center_lon_rad,
        "world_origin_height_m": center_h_m,
        "aircraft_mesh_path": mesh_res_path,
    }


def _godot_terrain_dir() -> Path:
    """Return the path to godot/terrain/, using LITEAERO_GODOT_DIR env var if set."""
    env = os.environ.get("LITEAERO_GODOT_DIR")
    if env:
        return Path(env) / "terrain"
    return _PROJECT_ROOT / "godot" / "terrain"


# ---------------------------------------------------------------------------
# Command-line entry point
# ---------------------------------------------------------------------------

def _main() -> None:
    import argparse

    parser = argparse.ArgumentParser(
        prog="build_terrain",
        description=(
            "Build a terrain dataset for a LiteAero simulation location.\n\n"
            "Reads an aircraft configuration JSON file, downloads DEM and\n"
            "satellite imagery for the surrounding area, triangulates all\n"
            "LOD levels, and writes a GLB + sidecar files ready for Godot.\n\n"
            "After running this tool, press Play in Godot — no further\n"
            "editor interaction is required."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "examples:\n"
            "  # Build terrain for Santa Barbara airport (centroid from config):\n"
            "  python build_terrain.py ../configs/general_aviation_ksba.json\n\n"
            "  # Override centroid to a different runway threshold:\n"
            "  python build_terrain.py ../configs/general_aviation_ksba.json \\\n"
            "      --center-lat 34.4258 --center-lon -119.8400\n\n"
            "  # Override coverage radius:\n"
            "  python build_terrain.py ../configs/jet_trainer_ksba.json --radius-km 120\n\n"
            "  # Force a full rebuild (keeps cached downloads):\n"
            "  python build_terrain.py ../configs/general_aviation_ksba.json --force\n\n"
            "output locations:\n"
            "  Terrain data:   data/terrain/<name>/\n"
            "  Godot GLB:      godot/terrain/terrain.glb\n"
            "  Godot config:   godot/terrain/terrain_config.json"
        ),
    )
    parser.add_argument(
        "config",
        metavar="AIRCRAFT_CONFIG",
        type=Path,
        help="path to the aircraft configuration JSON file",
    )
    parser.add_argument(
        "--name",
        metavar="NAME",
        default=None,
        help=(
            "dataset name used as the directory key under data/terrain/. "
            "Defaults to the config file stem (e.g. 'ksba_ga')."
        ),
    )
    parser.add_argument(
        "--center-lat",
        metavar="DEG",
        type=float,
        default=None,
        help=(
            "geodetic latitude of the terrain tile centroid in decimal degrees. "
            "Overrides initial_state.latitude_rad from the config file."
        ),
    )
    parser.add_argument(
        "--center-lon",
        metavar="DEG",
        type=float,
        default=None,
        help=(
            "longitude of the terrain tile centroid in decimal degrees. "
            "Overrides initial_state.longitude_rad from the config file."
        ),
    )
    parser.add_argument(
        "--radius-km",
        metavar="KM",
        type=float,
        default=None,
        help=(
            "half-side of the terrain coverage square in kilometres. "
            "Overrides terrain.radius_km from the config file."
        ),
    )
    parser.add_argument(
        "--dem-source",
        metavar="SOURCE",
        default="copernicus_dem_glo30",
        choices=["copernicus_dem_glo30", "nasadem", "srtm"],
        help="DEM data source (default: copernicus_dem_glo30)",
    )
    parser.add_argument(
        "--imagery-source",
        metavar="SOURCE",
        default="sentinel2",
        choices=["sentinel2", "landsat9", "modis"],
        help="satellite imagery source for terrain colorization (default: sentinel2)",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="delete all derived outputs and rebuild from scratch (cached downloads are kept)",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="show detailed per-step progress",
    )

    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s  %(levelname)-7s  %(message)s",
        datefmt="%H:%M:%S",
    )

    radius_m = args.radius_km * 1000.0 if args.radius_km is not None else None

    try:
        glb_path = build_terrain(
            args.config,
            name=args.name,
            center_lat_deg=args.center_lat,
            center_lon_deg=args.center_lon,
            radius_m=radius_m,
            dem_source=args.dem_source,
            imagery_source=args.imagery_source,
            force=args.force,
        )
    except (FileNotFoundError, ValueError) as exc:
        parser.error(str(exc))

    print(f"terrain ready → {glb_path}")


if __name__ == "__main__":
    _main()
