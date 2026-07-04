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
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime, timezone
from pathlib import Path

import _bootstrap  # noqa: F401  # inserts tools/ on sys.path; must precede shared imports
from colorize import colorize
from download import DownloadError, default_cache_dir, download_dem, download_imagery
from geoid_correct import apply_geoid_correction
from las_terrain import TerrainTileData, write_las_terrain
from lod_policy import lod_footprints_m, lod_policy_dict
from mosaic import mosaic_dem, mosaic_imagery
from mosaic_render import render_mosaic
from raster_sample import RasterSampler
from terrain_chunks import export_chunked_terrain
from terrain_paths import (
    derived_dir, las_terrain_dir, metadata_path, source_dir,
    terrain_config_path, terrain_descriptor_path, terrain_tiles_dir,
)
from triangulate import lod_grid_spacing_deg, triangulate
from verify import MeshQualityError, check

# DEM source -> native geoid model.  Each source publishes orthometric heights
# referenced to a specific geoid; geoid_correct.apply_geoid_correction() must
# be invoked with the matching geoid to convert to WGS84 ellipsoidal.
# Resolution per OQ-LS-13.
_DEM_SOURCE_GEOID: dict[str, str] = {
    "copernicus_dem_glo30": "egm2008",
    "nasadem":              "egm96",
    "srtm":                 "egm96",
}

_log = logging.getLogger("build_terrain")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Download chunk size: Sentinel Hub limit ≈ 2,500 × 2,500 px at L0 resolution.
_L0_RESOLUTION_DEG: float = 0.000090  # native Copernicus GLO-30 resolution
_MAX_CHUNK_SIDE_DEG: float = 2500 * _L0_RESOLUTION_DEG  # = 0.225 deg ≈ 25 km

# Per-LOD GPU texture ceiling (pixels per axis).
# L0 tiles are ~1 km wide; at NAIP 0.6 m/pixel the native count is ~1667 px, which fits
# in 2048 with no clamping.  Coarser LODs cover larger footprints — caps are chosen so
# that the effective ground resolution degrades gracefully as the viewer moves farther away.
_LOD_MAX_PIXEL_DIM: list[int] = [
    2048,   # L0  ~1 km  → NAIP native ~1667 px, fits without clamping
    4096,   # L1  ~3 km  → NAIP native ~5000 px → 4096 cap (~0.73 m/px)
    8192,   # L2  ~10 km → NAIP native ~16 k px → 8192 cap (~1.2 m/px)
    8192,   # L3  ~30 km → 8192 cap (~3.7 m/px); Sentinel-2 at 10 m is adequate here
    4096,   # L4  ~100 km → Sentinel-2 native ~10 k px → 4096 cap
    4096,   # L5  same as L4
    4096,   # L6  same as L4
]

# Per-LOD footprint tiling (OQ-LS-22 Alt 3 → IP-LV-5).  Each LOD tiles the whole region on its
# own grid at a screen-space-error-scaled footprint f_l (lod_policy): small tiles for the fine
# near-ground LODs, large tiles for the coarse backdrop LODs, so every tile is smaller than its
# own LOD's culling band and per-node centroid-distance culling stays crisp everywhere.
#
# Degree-per-metre factors for the metre→degree footprint conversion (spherical approximation;
# longitude shrinks with latitude).  The chunk assignment that drives culling uses exact ENU.
_METERS_PER_DEG_LAT: float = 110_540.0
_EQUATOR_METERS_PER_DEG_LON: float = 111_320.0

# Minimum grid points per axis a cell is triangulated with.  A small fixed footprint cannot hold
# a coarse LOD's native spacing (e.g. a 400 m cell at L6's 10 km spacing < 2 points), so the
# effective spacing is clamped so each cell yields at least this many points — a small tile
# becomes a coarse mesh rather than degenerating.
_MIN_CELL_POINTS: int = 3

# Godot visibility range bounds per LOD (m).  From terrain.md §LOD Hysteresis Band.
_VIS_BEGIN_M: list[float] = [0.0, 255.0, 765.0, 2_295.0, 6_885.0, 20_655.0, 61_965.0]
_VIS_END_M: list[float | None] = [345.0, 1_035.0, 3_105.0, 9_315.0, 27_945.0, 83_835.0, None]

# Chunk side, in tile footprints (OQ-LS-20).  The per-LOD tile footprints come from the
# screen-space-error policy (lod_policy); this only sets how many footprints wide a chunk file is.
# Tunable via the config ``terrain.chunk_footprints`` or the ``--chunk-footprints`` CLI flag.
_DEFAULT_CHUNK_FOOTPRINTS: int = 4

# Project root: parents[3] of python/tools/terrain/build_terrain.py = liteaero-sim/
_PROJECT_ROOT: Path = Path(__file__).resolve().parents[3]

# ---------------------------------------------------------------------------
# Per-cell triangulation worker (used by the parallel build path)
# ---------------------------------------------------------------------------

# Process-local cache of in-memory raster samplers, keyed by (path, multiband).
# Each ProcessPoolExecutor worker reads the DEM and imagery once, then reuses the
# in-memory arrays for every cell it processes — so the raster read cost is paid
# once per worker, not once per cell.
_SAMPLER_CACHE: dict[tuple[str, bool], RasterSampler] = {}


def _get_sampler(path: str, multiband: bool) -> RasterSampler:
    """Return a cached (per-process) RasterSampler for path, reading it once."""
    key = (path, multiband)
    sampler = _SAMPLER_CACHE.get(key)
    if sampler is None:
        sampler = RasterSampler.from_path(Path(path), multiband=multiband)
        _SAMPLER_CACHE[key] = sampler
    return sampler


def _build_cell(
    task: tuple[str, str, str, int, tuple[float, float, float, float], float],
) -> TerrainTileData:
    """Triangulate + colorize + quality-check a single cell (ProcessPool worker).

    ``task`` = (dem_path, imagery_path, imagery_source, lod, cell_bbox_deg, spacing_deg).
    Rasters are sampled from the process-local in-memory cache (see _get_sampler).
    """
    dem_path, img_path, source, lod, cell_bbox, spacing = task
    dem = _get_sampler(dem_path, multiband=False)
    img = _get_sampler(img_path, multiband=True)
    tile = triangulate(dem, cell_bbox, lod=lod, spacing_deg=spacing)
    tile = colorize(tile, img, source=source)
    check(tile)
    return tile

# Approximate bounding box of the contiguous United States (CONUS).
# Used to decide whether NAIP imagery is available as a high-resolution source.
_CONUS_LON_MIN: float = -124.85
_CONUS_LON_MAX: float = -66.88
_CONUS_LAT_MIN: float = 24.39
_CONUS_LAT_MAX: float = 49.38


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
    imagery_source: str = "auto",
    force: bool = False,
    workers: int = 1,
    chunk_footprints: int | None = None,
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
        Imagery source selection.  ``"auto"`` (default) picks the best available
        source(s) by location: NAIP (~1 m/pixel) where available (CONUS), with
        Sentinel-2 (~10 m/pixel) as a global fallback.  Explicit values
        ``"sentinel2"``, ``"landsat9"``, ``"modis"``, or ``"naip"`` force a single
        source regardless of location.
    force:
        If True, delete all derived outputs and rebuild from scratch.  Cached
        DEM/imagery files in ``python/cache/terrain/`` are retained.
    workers:
        Number of processes used to triangulate/colorize tiles.  ``1`` (default)
        runs the per-cell loop in-process; a value ``> 1`` distributes cells over
        a ``ProcessPoolExecutor`` (each worker reads the DEM and imagery once).
    chunk_footprints:
        Chunk side as an integer number of tile footprints (OQ-LS-20).  ``None``
        (default) reads ``terrain.chunk_footprints`` from the config, falling back to
        the module default.  The per-LOD tile footprints themselves are fixed by the
        screen-space-error policy (``lod_policy``), not this argument.

    Returns
    -------
    Path
        Absolute path to the streamable-terrain descriptor
        (``data/terrain/<name>/derived/terrain_tiles/descriptor.json``).

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
    # center_h_m is derived from the geoid-corrected DEM after Step 2a so that
    # the world-origin WGS84 height matches the actual terrain surface, not the
    # aircraft's initial state altitude_m (which is independent and also WGS84).
    center_h_m: float = 0.0  # overwritten after geoid correction in Step 2a

    # Terrain radius: CLI/caller override takes precedence over config terrain section.
    if radius_m is None:
        radius_m = _radius_from_config(config)

    # Chunk side (in tile footprints): caller argument overrides config, config overrides default.
    # The per-LOD tile footprints themselves come from the screen-space-error policy (lod_policy).
    resolved_chunk_footprints = (
        chunk_footprints if chunk_footprints is not None
        else _chunk_footprints_from_config(config)
    )

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
    tiles_dir = terrain_tiles_dir(dataset_name)
    m_path = metadata_path(dataset_name)

    s_dir.mkdir(parents=True, exist_ok=True)
    d_dir.mkdir(parents=True, exist_ok=True)
    lt_dir.mkdir(parents=True, exist_ok=True)
    tiles_dir.mkdir(parents=True, exist_ok=True)

    if force:
        if lt_dir.exists():
            shutil.rmtree(lt_dir)
            lt_dir.mkdir(parents=True, exist_ok=True)
        if tiles_dir.exists():
            shutil.rmtree(tiles_dir)
            tiles_dir.mkdir(parents=True, exist_ok=True)
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

    # Select imagery sources based on bbox location and the caller's override.
    selected_sources = _select_imagery_sources(bbox_deg, imagery_source)
    _log.info("%s: imagery sources: %s", dataset_name, selected_sources)

    _log.info(
        "%s: downloading imagery  %d chunk(s) × %d source(s) ...",
        dataset_name, len(chunks), len(selected_sources),
    )
    img_mosaic_paths: list[tuple[Path, str]] = []
    for src_name in selected_sources:
        src_chunk_paths: list[Path] = []
        for i, chunk in enumerate(chunks, 1):
            _log.info(
                "%s: imagery chunk %d/%d [%s] ...", dataset_name, i, len(chunks), src_name
            )
            src_chunk_paths.extend(
                download_imagery(
                    chunk,
                    output_dir=cache_dir / "imagery" / src_name,
                    source=src_name,
                )
            )
        if src_name == "naip":
            # NAIP tiles are full quarter-quads in projected UTM CRS.  Mosaicking
            # them would require reading all tiles into memory (~17 GB) and the
            # UTM pixel size (0.6 m) is incompatible with mosaic_imagery's
            # assumption that pixel sizes are in EPSG:4326 degrees.
            # render_mosaic._read_source_rgb already handles UTM windowing via
            # transform_bounds, so pass tiles directly — one entry per unique tile.
            seen: set[Path] = set()
            for p in src_chunk_paths:
                if p not in seen:
                    seen.add(p)
                    img_mosaic_paths.append((p, src_name))
        else:
            mosaic_path = s_dir / f"img_mosaic_{src_name}.tif"
            mosaic_imagery(src_chunk_paths, mosaic_path)
            img_mosaic_paths.append((mosaic_path, src_name))

    # -------------------------------------------------------------------
    # Step 2: Mosaic DEM chunks into a seamless source raster
    # -------------------------------------------------------------------
    _log.info("%s: mosaicking DEM ...", dataset_name)
    dem_mosaic_path = s_dir / "dem_mosaic.tif"
    mosaic_dem(dem_chunk_paths, dem_mosaic_path)

    # -------------------------------------------------------------------
    # Step 2a: Apply geoid correction (orthometric -> WGS84 ellipsoidal).
    #
    # All currently-supported DEM sources publish orthometric heights against
    # one of two geoid models (EGM2008 for Copernicus; EGM96 for NASADEM/SRTM).
    # The downstream pipeline (triangulate, las_terrain) treats heights as
    # WGS84 ellipsoidal, so the conversion must occur here.  Resolution per
    # OQ-LS-13.  See live_sim_view.md Issue 6.
    # -------------------------------------------------------------------
    geoid = _DEM_SOURCE_GEOID.get(dem_source)
    if geoid is None:
        raise ValueError(
            f"unknown dem_source '{dem_source}': no geoid mapping configured"
        )
    dem_ellipsoidal_path = s_dir / "dem_ellipsoidal.tif"
    _log.info(
        "%s: geoid correction (%s -> WGS84 ellipsoidal) ...",
        dataset_name, geoid,
    )
    apply_geoid_correction(dem_mosaic_path, dem_ellipsoidal_path, geoid=geoid)

    center_h_m = _sample_dem_height(dem_ellipsoidal_path, center_lon_deg, center_lat_deg)
    _log.info(
        "%s: world origin WGS84 height %.3f m  (sampled from ellipsoidal DEM)",
        dataset_name, center_h_m,
    )

    # Highest-priority source (last in img_mosaic_paths) is used for per-facet
    # colorization — secondary data stored in las_terrain alongside the geometry.
    best_img_path, best_img_source = img_mosaic_paths[-1]

    # -------------------------------------------------------------------
    # Step 3: Triangulate and colorize all LOD levels
    # -------------------------------------------------------------------
    all_tiles: list[TerrainTileData] = []

    # Per-LOD footprint tiling (IP-LV-5, OQ-LS-22 Alt 3): each LOD is tiled on its OWN grid at
    # its screen-space-error-scaled footprint f_l (small fine tiles, large coarse tiles), so a
    # tile is smaller than its own LOD's culling band and per-node centroid-distance culling stays
    # crisp.  Footprints are baked at the reference resolution (terrain_lod_rendering.md OQ-LR-1);
    # the vertex spacing within a cell is the LOD's native spacing, clamped up if a (small) cell
    # would otherwise hold too few points.
    footprints_m = lod_footprints_m()  # per-LOD tile footprint (m), index = LOD

    cell_tasks: list[tuple[int, tuple[float, float, float, float], float]] = []
    for lod in range(7):
        lon_side, lat_side = _cell_side_deg(footprints_m[lod], center_lat_rad)
        cells_lod = _compute_cell_grid(bbox_deg, lon_side, lat_side)
        spacing = _effective_spacing_deg(lod, lon_side, lat_side)
        cell_tasks.extend((lod, cell_bbox, spacing) for _row, _col, cell_bbox in cells_lod)
        _log.info(
            "%s: LOD %d  footprint %.0f m  ->  %d cell(s)",
            dataset_name, lod, footprints_m[lod], len(cells_lod),
        )

    _log.info(
        "%s: %d tiles across 7 per-LOD grids  (workers=%d) ...",
        dataset_name, len(cell_tasks), workers,
    )

    if workers == 1:
        # Serial, in-process: read the DEM and imagery once, reuse for every cell.
        dem_sampler = RasterSampler.from_path(dem_ellipsoidal_path)
        img_sampler = RasterSampler.from_path(best_img_path, multiband=True)
        for lod, cell_bbox, spacing in cell_tasks:
            tile = triangulate(dem_sampler, cell_bbox, lod=lod, spacing_deg=spacing)
            tile = colorize(tile, img_sampler, source=best_img_source)
            check(tile)
            all_tiles.append(tile)
    else:
        # Parallel: each worker reads the rasters once (process-local cache) and
        # processes a chunk of cells.  Only small (path, lod, bbox, spacing) tuples cross the
        # process boundary — the large raster arrays are never pickled.
        worker_args = [
            (str(dem_ellipsoidal_path), str(best_img_path), best_img_source, lod, cell_bbox, spacing)
            for lod, cell_bbox, spacing in cell_tasks
        ]
        chunksize = max(1, len(worker_args) // (workers * 8))
        with ProcessPoolExecutor(max_workers=workers) as executor:
            all_tiles.extend(executor.map(_build_cell, worker_args, chunksize=chunksize))

    # -------------------------------------------------------------------
    # Step 4: Write .las_terrain (all LODs)
    # -------------------------------------------------------------------
    lt_path = lt_dir / "terrain.las_terrain"
    _log.info("%s: writing .las_terrain  %d tiles ...", dataset_name, len(all_tiles))
    write_las_terrain(lt_path, all_tiles)

    # -------------------------------------------------------------------
    # Step 5: Display tiles = every tile.  Uniform-footprint tiling exports all LODs
    # region-wide (no nested export-radius selection); chunking + streaming handle scale.
    # -------------------------------------------------------------------
    display_tiles = all_tiles
    if not display_tiles:
        raise RuntimeError(
            f"{dataset_name}: triangulation produced no tiles; "
            "check that the coverage area contains the center position"
        )

    # -------------------------------------------------------------------
    # Step 5b: Render per-tile JPEG mosaic textures
    #
    # Each display tile gets its own texture covering only its geographic
    # footprint, at the native resolution of the highest-priority imagery
    # source capped at the per-LOD GPU texture ceiling (_LOD_MAX_PIXEL_DIM).
    # This delivers NAIP at or near its native 0.6 m/pixel for close-in
    # LOD tiles (L0, L1) while staying within VRAM budget on the reference
    # GPU (GTX 1050 Ti, 4 GB).
    # -------------------------------------------------------------------
    _log.info(
        "%s: rendering per-tile textures  %d tile(s) ...",
        dataset_name, len(display_tiles),
    )
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
        _log.debug(
            "%s: LOD %d tile texture %d×%d px  %.1f kB JPEG",
            dataset_name, tile.lod,
            tile_mosaic.width_pixels, tile_mosaic.height_pixels,
            len(tile_mosaic.jpeg_bytes) / 1024.0,
        )
        tile_mosaics.append((tile, tile_mosaic))

    # -------------------------------------------------------------------
    # Step 6: Export streamable chunk files + descriptor (OQ-LS-20 Alt 2)
    #
    # Tiles are grouped into per-(LOD, chunk) GLB files addressed by integer chunk
    # coordinate, so the Godot streaming manager (OQ-LS-21) can page fine LODs by
    # aircraft proximity.  Per-LOD chunk side = tile_footprints_m[lod] × chunk_footprints.
    # -------------------------------------------------------------------
    _log.info(
        "%s: exporting %d display tile(s) as per-LOD chunks  (chunk = footprint × %d) ...",
        dataset_name, len(display_tiles), resolved_chunk_footprints,
    )
    descriptor = export_chunked_terrain(
        tile_mosaics,
        tiles_dir,
        world_origin=(center_lat_rad, center_lon_rad, center_h_m),
        tile_footprints_m=footprints_m,
        chunk_footprints=resolved_chunk_footprints,
        bounds_deg=bbox_deg,
        lod_policy=lod_policy_dict(),
    )
    _log.info(
        "%s: wrote %d chunk file(s)  →  %s",
        dataset_name, len(descriptor.chunks), tiles_dir,
    )

    # -------------------------------------------------------------------
    # Step 7: Write terrain_config.json to dataset root
    # -------------------------------------------------------------------
    terrain_config = _build_terrain_config(
        config=config,
        dataset_name=dataset_name,
        center_lat_rad=center_lat_rad,
        center_lon_rad=center_lon_rad,
        center_h_m=center_h_m,
    )
    tc_path_out = terrain_config_path(dataset_name)
    tc_path_out.write_text(json.dumps(terrain_config, indent=4))
    _log.info("%s: terrain_config.json → %s", dataset_name, tc_path_out)

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
        "dem_geoid": geoid,
        "imagery_sources": selected_sources,
        "dem_resolution_deg": _L0_RESOLUTION_DEG,
        "bbox_deg": list(bbox_deg),
        "build_timestamp_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
    }
    m_path.write_text(json.dumps(metadata, indent=4))

    descriptor_path = terrain_descriptor_path(dataset_name)
    _log.info("%s: build complete → %s", dataset_name, descriptor_path)
    return descriptor_path


# ---------------------------------------------------------------------------
# Internal helpers (also exposed for unit-testing parameter derivation)
# ---------------------------------------------------------------------------

def _sample_dem_height(dem_path: Path, lon_deg: float, lat_deg: float) -> float:
    """Return the WGS84 ellipsoidal height (m) from dem_path at (lon_deg, lat_deg).

    Uses rasterio's point-sampling via the affine transform.  The DEM must
    already carry ellipsoidal heights (i.e., geoid correction applied).
    """
    import rasterio

    with rasterio.open(dem_path) as src:
        vals = list(src.sample([(lon_deg, lat_deg)]))
    return float(vals[0][0])


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


def _chunk_footprints_from_config(config: dict) -> int:
    """Return the chunk side (in tile footprints) from ``terrain.chunk_footprints``, or the default."""
    terrain_section = config.get("terrain") or {}
    return int(terrain_section.get("chunk_footprints", _DEFAULT_CHUNK_FOOTPRINTS))


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


def _cell_side_deg(footprint_m: float, center_lat_rad: float) -> tuple[float, float]:
    """Return (lon_side_deg, lat_side_deg) for a square ``footprint_m`` cell at a latitude."""
    lat_side = footprint_m / _METERS_PER_DEG_LAT
    lon_side = footprint_m / (_EQUATOR_METERS_PER_DEG_LON * max(0.01, math.cos(center_lat_rad)))
    return lon_side, lat_side


def _effective_spacing_deg(
    lod: int, cell_side_lon_deg: float, cell_side_lat_deg: float
) -> float:
    """Grid spacing (deg) for a LOD within a fixed-footprint cell.

    The LOD's native spacing is used when it fits; for coarse LODs whose native spacing is
    larger than the small cell, it is clamped so the cell still yields ``_MIN_CELL_POINTS``
    points per axis (a small tile becomes a coarse mesh instead of degenerating).
    """
    native = lod_grid_spacing_deg(lod)
    cap = min(cell_side_lon_deg, cell_side_lat_deg) / (_MIN_CELL_POINTS - 1)
    return min(native, cap)


def _compute_cell_grid(
    bbox_deg: tuple[float, float, float, float],
    cell_side_lon_deg: float,
    cell_side_lat_deg: float,
) -> list[tuple[int, int, tuple[float, float, float, float]]]:
    """Tile bbox_deg into uniform cells of the given per-axis degree sizes.

    Returns a list of (row, col, cell_bbox_deg).  The same grid is used for every LOD
    (LOD varies vertex density within the fixed footprint); all coverage is included (≥ 1 cell).
    """
    lon_min, lat_min, lon_max, lat_max = bbox_deg
    n_lon = max(1, math.ceil((lon_max - lon_min) / cell_side_lon_deg))
    n_lat = max(1, math.ceil((lat_max - lat_min) / cell_side_lat_deg))
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


def _bbox_intersects_conus(bbox_deg: tuple[float, float, float, float]) -> bool:
    """Return True if any part of bbox_deg overlaps the CONUS bounding box."""
    lon_min, lat_min, lon_max, lat_max = bbox_deg
    return (
        lon_min < _CONUS_LON_MAX and lon_max > _CONUS_LON_MIN
        and lat_min < _CONUS_LAT_MAX and lat_max > _CONUS_LAT_MIN
    )


def _select_imagery_sources(
    bbox_deg: tuple[float, float, float, float],
    source_override: str,
) -> list[str]:
    """Return ordered imagery source list (lowest to highest priority).

    With ``source_override="auto"``, the selection is based on location:

    - CONUS bbox: ``["sentinel2", "naip"]`` — Sentinel-2 as global fallback,
      NAIP (~1 m/pixel) as highest-priority source for land areas.
    - Elsewhere:  ``["sentinel2"]`` — Sentinel-2 only.

    Any explicit source name (``"sentinel2"``, ``"landsat9"``, ``"modis"``,
    ``"naip"``) bypasses auto-selection and returns a single-element list.
    """
    if source_override != "auto":
        return [source_override]
    if _bbox_intersects_conus(bbox_deg):
        return ["sentinel2", "naip"]
    return ["sentinel2"]


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

    Computes ``aircraft_wingspan_m`` from the aerodynamic geometry parameters
    in the ``aircraft`` section: wingspan = sqrt(S_ref_m2 * ar).
    """
    mesh_res_path: str = (
        config.get("visualization", {}).get("mesh_res_path", _DEFAULT_MESH_RES_PATH)
        or _DEFAULT_MESH_RES_PATH
    )
    las_terrain_path: str = str(
        (las_terrain_dir(dataset_name) / "terrain.las_terrain").resolve()
    )

    # Wingspan derived from aerodynamic geometry: b = sqrt(S_ref * AR).
    aircraft_section = config.get("aircraft", {})
    s_ref_m2 = float(aircraft_section.get("S_ref_m2", 0.0))
    ar = float(aircraft_section.get("ar", 0.0))
    wingspan_m = math.sqrt(s_ref_m2 * ar) if s_ref_m2 > 0 and ar > 0 else 0.0

    # Gear contact height: max over all wheel units of (attach_z + tyre_radius).
    # attach_point_body_m[2] is body-frame Z (Z-down convention), so positive =
    # below CG.  This is the CG height above terrain when the gear contacts the ground.
    gear_contact_height_m = 0.0
    for wheel in config.get("landing_gear", {}).get("wheel_units", []):
        attach_z = float(wheel.get("attach_point_body_m", [0.0, 0.0, 0.0])[2])
        tyre_r = float(wheel.get("tyre_radius_m", 0.0))
        gear_contact_height_m = max(gear_contact_height_m, attach_z + tyre_r)

    descriptor_path_str = str(terrain_descriptor_path(dataset_name).resolve())

    return {
        "schema_version": 1,
        "dataset_name": dataset_name,
        "terrain_descriptor_path": descriptor_path_str,
        "world_origin_lat_rad": center_lat_rad,
        "world_origin_lon_rad": center_lon_rad,
        "world_origin_height_m": center_h_m,
        "aircraft_mesh_path": mesh_res_path,
        "aircraft_wingspan_m": round(wingspan_m, 4),
        "aircraft_gear_contact_height_m": round(gear_contact_height_m, 4),
        "las_terrain_path": las_terrain_path,
    }


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
            "  Terrain data:        data/terrain/<name>/\n"
            "  terrain_config.json: data/terrain/<name>/terrain_config.json\n\n"
            "to run the live sim after build:\n"
            "  live_sim.exe --config configs/<name>.json \\\n"
            "               --terrain data/terrain/<name>/terrain_config.json"
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
        default="auto",
        choices=["auto", "sentinel2", "landsat9", "modis", "naip"],
        help=(
            "imagery source (default: auto — selects NAIP for CONUS, "
            "Sentinel-2 elsewhere)"
        ),
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
    parser.add_argument(
        "--workers",
        metavar="N",
        type=int,
        default=os.cpu_count() or 1,
        help=(
            "number of processes for tile triangulation/colorization "
            "(default: all CPU cores; use 1 for serial in-process)"
        ),
    )
    parser.add_argument(
        "--chunk-footprints",
        metavar="N",
        type=int,
        default=None,
        help=(
            "chunk side as an integer number of tile footprints "
            "(default: terrain.chunk_footprints from config, else "
            f"{_DEFAULT_CHUNK_FOOTPRINTS})"
        ),
    )

    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s  %(levelname)-7s  %(message)s",
        datefmt="%H:%M:%S",
    )

    radius_m = args.radius_km * 1000.0 if args.radius_km is not None else None

    try:
        descriptor_path = build_terrain(
            args.config,
            name=args.name,
            center_lat_deg=args.center_lat,
            center_lon_deg=args.center_lon,
            radius_m=radius_m,
            dem_source=args.dem_source,
            imagery_source=args.imagery_source,
            force=args.force,
            workers=args.workers,
            chunk_footprints=args.chunk_footprints,
        )
    except (FileNotFoundError, ValueError, DownloadError) as exc:
        parser.error(str(exc))

    print(f"terrain ready -> {descriptor_path}")


if __name__ == "__main__":
    _main()
