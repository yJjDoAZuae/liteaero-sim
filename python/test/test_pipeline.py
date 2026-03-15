"""Full pipeline integration test (Step 21).

Runs a synthetic end-to-end pipeline using a flat 5×5 km DEM at the equator:
    1. triangulate()  → L0 tile
    2. simplify()     → L1 tile (face count < L0)
    3. colorize()     → all facets white (solid-color imagery)
    4. write_las_terrain() / read_las_terrain() round-trip
    5. export_gltf()  → GLB magic bytes + POSITION count == 3 × facets
"""

from __future__ import annotations

import struct
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("rasterio")
pytest.importorskip("scipy")
pytest.importorskip("pyfqmr")
pytest.importorskip("trimesh")


def test_synthetic_pipeline_end_to_end(tmp_path: Path) -> None:
    import rasterio
    from rasterio.crs import CRS
    from rasterio.transform import from_bounds

    from export_gltf import export_gltf
    from las_terrain import read_las_terrain, write_las_terrain
    from simplify import simplify
    from triangulate import lod_grid_spacing_deg, triangulate

    # -------------------------------------------------------------------
    # 1. Build a synthetic flat 15×15 DEM at the equator (enough for LOD 0).
    # -------------------------------------------------------------------
    spacing = lod_grid_spacing_deg(0)  # ~0.000090°
    n = 15
    lon_min, lat_min = 0.0, 0.0
    lon_max = lon_min + (n - 1) * spacing
    lat_max = lat_min + (n - 1) * spacing
    bbox = (lon_min, lat_min, lon_max, lat_max)

    dem_path = tmp_path / "flat.tif"
    half = spacing / 2.0
    data = np.full((n, n), 50.0, dtype=np.float32)
    transform = from_bounds(
        lon_min - half, lat_min - half, lon_max + half, lat_max + half, n, n
    )
    with rasterio.open(
        dem_path,
        "w",
        driver="GTiff",
        height=n,
        width=n,
        count=1,
        dtype="float32",
        crs=CRS.from_epsg(4326),
        transform=transform,
    ) as dst:
        dst.write(data, 1)

    # -------------------------------------------------------------------
    # 2. Triangulate → L0 tile.
    # -------------------------------------------------------------------
    tile_l0 = triangulate(dem_path, bbox, lod=0)
    assert tile_l0.lod == 0
    assert len(tile_l0.indices) >= 2

    # -------------------------------------------------------------------
    # 3. Simplify → L1 tile; face count must decrease.
    # -------------------------------------------------------------------
    tile_l1 = simplify(tile_l0, target_lod=1)
    assert tile_l1.lod == 1
    assert len(tile_l1.indices) < len(tile_l0.indices)

    # -------------------------------------------------------------------
    # 4. Colorize → all facets white (solid-color imagery).
    # -------------------------------------------------------------------
    imagery_path = tmp_path / "white.tif"
    # Sentinel-2 scale 1/10000: raw 10000 → reflectance 1.0 → 255 in 8-bit (clipped to 255).
    white_raw = 10000
    img_data = np.full((4, 5, 5), white_raw, dtype=np.uint16)
    img_transform = from_bounds(
        lon_min - 0.001, lat_min - 0.001, lon_max + 0.001, lat_max + 0.001, 5, 5
    )
    with rasterio.open(
        imagery_path,
        "w",
        driver="GTiff",
        height=5,
        width=5,
        count=4,
        dtype="uint16",
        crs=CRS.from_epsg(4326),
        transform=img_transform,
    ) as dst:
        dst.write(img_data)

    from colorize import colorize

    tile_colored = colorize(tile_l1, imagery_path, source="sentinel2")
    # All facets should be white (255, 255, 255).
    np.testing.assert_array_equal(tile_colored.colors, 255)

    # -------------------------------------------------------------------
    # 5. write_las_terrain() / read_las_terrain() round-trip.
    # -------------------------------------------------------------------
    las_path = tmp_path / "output.las_terrain"
    write_las_terrain(las_path, [tile_l0, tile_colored])
    assert las_path.stat().st_size > 0

    recovered = read_las_terrain(las_path)
    assert len(recovered) == 2
    assert recovered[0].lod == 0
    assert recovered[1].lod == 1
    np.testing.assert_array_almost_equal(recovered[0].vertices, tile_l0.vertices, decimal=4)

    # -------------------------------------------------------------------
    # 6. export_gltf() → GLB magic + POSITION count == 3 × facets.
    # -------------------------------------------------------------------
    glb_path = tmp_path / "terrain.glb"
    export_gltf([tile_l0, tile_colored], glb_path)

    glb_bytes = glb_path.read_bytes()
    assert glb_bytes[:4] == b"glTF", f"Bad GLB magic: {glb_bytes[:4]!r}"

    # Parse JSON chunk.
    import json

    chunk_length = struct.unpack("<I", glb_bytes[12:16])[0]
    gltf = json.loads(glb_bytes[20 : 20 + chunk_length].rstrip(b"\x20"))

    accessors = gltf.get("accessors", [])
    total_positions = 0
    for mesh in gltf.get("meshes", []):
        for primitive in mesh.get("primitives", []):
            pos_idx = primitive.get("attributes", {}).get("POSITION")
            if pos_idx is not None:
                total_positions += accessors[pos_idx]["count"]

    expected = 3 * (len(tile_l0.indices) + len(tile_colored.indices))
    assert total_positions == expected, (
        f"Expected {expected} POSITION vertices, got {total_positions}"
    )

    assert b"liteaerosim_terrain" in glb_bytes
