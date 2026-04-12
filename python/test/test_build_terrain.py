"""Tests for build_terrain.py (Step TB-1).

All tests that require real terrain pipeline tools (rasterio, scipy, pyfqmr, trimesh)
are gated with pytest.importorskip so the suite can run in environments where those
packages are absent.

Download calls are always mocked; no network I/O occurs.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest

pytest.importorskip("rasterio")
pytest.importorskip("scipy")
pytest.importorskip("pyfqmr")
pytest.importorskip("trimesh")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _write_flat_dem(path: Path, bbox_deg: tuple[float, float, float, float], n: int = 120) -> None:
    """Write a flat (50 m elevation) GeoTIFF DEM covering bbox_deg with n×n pixels."""
    import rasterio
    from rasterio.crs import CRS
    from rasterio.transform import from_bounds

    lon_min, lat_min, lon_max, lat_max = bbox_deg
    data = np.full((n, n), 50.0, dtype=np.float32)
    transform = from_bounds(lon_min, lat_min, lon_max, lat_max, n, n)
    path.parent.mkdir(parents=True, exist_ok=True)
    with rasterio.open(
        path, "w", driver="GTiff", height=n, width=n, count=1,
        dtype="float32", crs=CRS.from_epsg(4326), transform=transform,
    ) as dst:
        dst.write(data, 1)


def _write_white_imagery(path: Path, bbox_deg: tuple[float, float, float, float], n: int = 32) -> None:
    """Write a white 4-band uint16 imagery GeoTIFF covering bbox_deg."""
    import rasterio
    from rasterio.crs import CRS
    from rasterio.transform import from_bounds

    lon_min, lat_min, lon_max, lat_max = bbox_deg
    # Sentinel-2 scale 1/10000; value 10000 = reflectance 1.0 → rendered as 255.
    data = np.full((4, n, n), 10000, dtype=np.uint16)
    transform = from_bounds(lon_min, lat_min, lon_max, lat_max, n, n)
    path.parent.mkdir(parents=True, exist_ok=True)
    with rasterio.open(
        path, "w", driver="GTiff", height=n, width=n, count=4,
        dtype="uint16", crs=CRS.from_epsg(4326), transform=transform,
    ) as dst:
        dst.write(data)


def _minimal_config(
    lat_rad: float = 0.0,
    lon_rad: float = 0.0,
    alt_m: float = 100.0,
    radius_km: float = 1.0,
) -> dict:
    return {
        "schema_version": 1,
        "terrain": {"radius_km": radius_km},
        "initial_state": {
            "latitude_rad": lat_rad,
            "longitude_rad": lon_rad,
            "altitude_m": alt_m,
            "velocity_north_mps": 0.0,
            "velocity_east_mps": 0.0,
            "velocity_down_mps": 0.0,
            "wind_north_mps": 0.0,
            "wind_east_mps": 0.0,
            "wind_down_mps": 0.0,
        },
    }


# ---------------------------------------------------------------------------
# Unit tests — parameter derivation (no pipeline, no mocks needed)
# ---------------------------------------------------------------------------

class TestParameterDerivation:
    def test_raises_file_not_found_for_missing_config(self, tmp_path: Path) -> None:
        import build_terrain as bt

        with pytest.raises(FileNotFoundError):
            bt.build_terrain(tmp_path / "nonexistent.json")

    def test_raises_json_decode_error_for_invalid_json(self, tmp_path: Path) -> None:
        import build_terrain as bt

        bad = tmp_path / "bad.json"
        bad.write_text("not json {{{")
        with pytest.raises(json.JSONDecodeError):
            bt.build_terrain(bad)

    def test_raises_value_error_for_missing_radius(self, tmp_path: Path) -> None:
        import build_terrain as bt

        cfg = _minimal_config()
        del cfg["terrain"]  # remove terrain section entirely
        cfg_path = tmp_path / "no_terrain.json"
        cfg_path.write_text(json.dumps(cfg))
        with pytest.raises(ValueError, match="terrain radius not specified"):
            bt.build_terrain(cfg_path)

    def test_radius_from_terrain_section(self) -> None:
        import build_terrain as bt

        config = {"terrain": {"radius_km": 50.0}, "initial_state": {}}
        radius_m = bt._radius_from_config(config)
        assert math.isclose(radius_m, 50_000.0, rel_tol=1e-9)

    def test_bbox_from_center_and_radius(self) -> None:
        import build_terrain as bt

        lon_min, lat_min, lon_max, lat_max = bt._bbox_from_center(
            center_lat_deg=34.0,
            center_lon_deg=-120.0,
            radius_m=10_000.0,
        )
        # Half-side in lat ≈ 10000 / 111132 ≈ 0.09003°
        delta_lat_deg = 10_000.0 / 111_132.0
        # Half-side in lon ≈ 10000 / (111132 × cos(34°)) ≈ 0.10857°
        delta_lon_deg = 10_000.0 / (111_132.0 * math.cos(math.radians(34.0)))
        assert math.isclose(lat_min, 34.0 - delta_lat_deg, rel_tol=1e-6)
        assert math.isclose(lat_max, 34.0 + delta_lat_deg, rel_tol=1e-6)
        assert math.isclose(lon_min, -120.0 - delta_lon_deg, rel_tol=1e-6)
        assert math.isclose(lon_max, -120.0 + delta_lon_deg, rel_tol=1e-6)


class TestChunkGrid:
    def test_single_chunk_for_small_bbox(self) -> None:
        import build_terrain as bt

        # 10 km × 10 km bbox — well within the 25 km chunk limit.
        delta = 10_000.0 / 111_132.0  # ≈ 0.09 deg
        bbox = (-delta, -delta, delta, delta)
        chunks = bt._compute_chunk_grid(bbox)
        assert len(chunks) == 1
        assert chunks[0] == pytest.approx(bbox, rel=1e-6)

    def test_multiple_chunks_for_large_bbox(self) -> None:
        import build_terrain as bt

        # 80 km × 80 km bbox at equator — requires 4 × 4 = 16 chunks.
        delta = 80_000.0 / 111_132.0  # ≈ 0.72 deg; max chunk ≈ 0.225 deg → ceil(3.2) = 4
        bbox = (-delta, -delta, delta, delta)
        chunks = bt._compute_chunk_grid(bbox)
        n_expected = math.ceil((2 * delta) / bt._MAX_CHUNK_SIDE_DEG) ** 2
        assert len(chunks) == n_expected

    def test_chunks_cover_full_bbox(self) -> None:
        import build_terrain as bt

        delta = 60_000.0 / 111_132.0
        bbox = (-delta, -delta, delta, delta)
        chunks = bt._compute_chunk_grid(bbox)
        lon_min = min(c[0] for c in chunks)
        lat_min = min(c[1] for c in chunks)
        lon_max = max(c[2] for c in chunks)
        lat_max = max(c[3] for c in chunks)
        assert lon_min == pytest.approx(bbox[0], rel=1e-6)
        assert lat_min == pytest.approx(bbox[1], rel=1e-6)
        assert lon_max == pytest.approx(bbox[2], rel=1e-6)
        assert lat_max == pytest.approx(bbox[3], rel=1e-6)


class TestCellGrid:
    def test_l0_cell_count_for_1km_bbox(self) -> None:
        import build_terrain as bt

        # Exactly 1 km × 1 km bbox → 1×1 L0 cells.
        side_deg = bt._LOD_CELL_SIDE_DEG[0]  # 0.009
        bbox = (0.0, 0.0, side_deg, side_deg)
        cells = bt._compute_cell_grid(bbox, lod=0)
        assert len(cells) == 1

    def test_l0_cell_count_for_larger_bbox(self) -> None:
        import build_terrain as bt

        # 3 km × 3 km bbox → 3×3 = 9 L0 cells.
        side_deg = 3 * bt._LOD_CELL_SIDE_DEG[0]  # 0.027
        bbox = (0.0, 0.0, side_deg, side_deg)
        cells = bt._compute_cell_grid(bbox, lod=0)
        assert len(cells) == 9

    def test_each_cell_has_row_col_and_bbox(self) -> None:
        import build_terrain as bt

        side_deg = 2 * bt._LOD_CELL_SIDE_DEG[1]
        bbox = (0.0, 0.0, side_deg, side_deg)
        cells = bt._compute_cell_grid(bbox, lod=1)
        assert len(cells) == 4
        for row, col, cell_bbox in cells:
            assert isinstance(row, int)
            assert isinstance(col, int)
            assert len(cell_bbox) == 4


class TestGeographicPartitioning:
    def test_center_tile_always_included(self) -> None:
        import build_terrain as bt
        from las_terrain import TerrainTileData

        # Build a minimal tile at (0,0) with lod=0.
        tile = TerrainTileData(
            lod=0,
            centroid_lat_rad=0.0,
            centroid_lon_rad=0.0,
            centroid_height_m=0.0,
            lat_min_rad=0.0, lat_max_rad=0.001,
            lon_min_rad=0.0, lon_max_rad=0.001,
            height_min_m=0.0, height_max_m=0.0,
            vertices=np.zeros((3, 3), dtype=np.float32),
            indices=np.zeros((1, 3), dtype=np.uint32),
            colors=np.zeros((1, 3), dtype=np.uint8),
        )
        selected = bt._select_display_tiles([tile], center_lat_rad=0.0, center_lon_rad=0.0)
        assert len(selected) == 1

    def test_distant_l0_tile_excluded(self) -> None:
        import build_terrain as bt
        from las_terrain import TerrainTileData

        # L0 export radius = 345 m.  Place tile 1 km away.
        offset_rad = 1_000.0 / 6_371_000.0
        tile = TerrainTileData(
            lod=0,
            centroid_lat_rad=offset_rad,
            centroid_lon_rad=0.0,
            centroid_height_m=0.0,
            lat_min_rad=0.0, lat_max_rad=offset_rad * 2,
            lon_min_rad=0.0, lon_max_rad=0.001,
            height_min_m=0.0, height_max_m=0.0,
            vertices=np.zeros((3, 3), dtype=np.float32),
            indices=np.zeros((1, 3), dtype=np.uint32),
            colors=np.zeros((1, 3), dtype=np.uint8),
        )
        selected = bt._select_display_tiles([tile], center_lat_rad=0.0, center_lon_rad=0.0)
        assert len(selected) == 0

    def test_l6_tile_always_included(self) -> None:
        import build_terrain as bt
        from las_terrain import TerrainTileData

        # L6 export radius = infinity.
        tile = TerrainTileData(
            lod=6,
            centroid_lat_rad=0.5,
            centroid_lon_rad=0.5,
            centroid_height_m=0.0,
            lat_min_rad=0.0, lat_max_rad=1.0,
            lon_min_rad=0.0, lon_max_rad=1.0,
            height_min_m=0.0, height_max_m=0.0,
            vertices=np.zeros((3, 3), dtype=np.float32),
            indices=np.zeros((1, 3), dtype=np.uint32),
            colors=np.zeros((1, 3), dtype=np.uint8),
        )
        selected = bt._select_display_tiles([tile], center_lat_rad=0.0, center_lon_rad=0.0)
        assert len(selected) == 1


# ---------------------------------------------------------------------------
# Integration test — full build with mocked downloads
# ---------------------------------------------------------------------------

class TestEndToEndMocked:
    """Full build pipeline with all I/O replaced by stubs.

    Downloads, triangulation, colorisation, and mesh quality checks are all
    mocked so the suite validates pipeline orchestration only (file routing,
    config content, metadata) without performing heavy raster or mesh
    computation.  Correctness of the individual pipeline steps is covered by
    their own unit-test modules.
    """

    @pytest.fixture(autouse=True)
    def _setup_env(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.setenv("LITEAERO_TERRAIN_ROOT", str(tmp_path / "terrain_data"))
        monkeypatch.setenv("LITEAERO_GODOT_DIR", str(tmp_path / "godot"))

    @pytest.fixture
    def _aircraft_config(self, tmp_path: Path) -> Path:
        # Triangulation is mocked so radius size doesn't matter for performance.
        cfg = _minimal_config(radius_km=1.0)
        path = tmp_path / "test_aircraft.json"
        path.write_text(json.dumps(cfg))
        return path

    # ------------------------------------------------------------------
    # Stub factories
    # ------------------------------------------------------------------

    def _make_mock_download_dem(self, cache_dir: Path):
        def mock_download_dem(
            bbox_deg: tuple,
            output_dir: Path | None = None,
            source: str = "copernicus_dem_glo30",
        ) -> list[Path]:
            out = output_dir or cache_dir
            out.mkdir(parents=True, exist_ok=True)
            lon_min, lat_min = bbox_deg[0], bbox_deg[1]
            tag = f"{lon_min:.6f}_{lat_min:.6f}"
            path = out / f"dem_{tag}.tif"
            _write_flat_dem(path, bbox_deg)
            return [path]

        return mock_download_dem

    def _make_mock_download_imagery(self, cache_dir: Path):
        def mock_download_imagery(
            bbox_deg: tuple,
            output_dir: Path | None = None,
            source: str = "sentinel2",
            **kwargs,
        ) -> list[Path]:
            out = output_dir or cache_dir
            out.mkdir(parents=True, exist_ok=True)
            lon_min, lat_min = bbox_deg[0], bbox_deg[1]
            tag = f"{lon_min:.6f}_{lat_min:.6f}"
            path = out / f"img_{tag}.tif"
            _write_white_imagery(path, bbox_deg)
            return [path]

        return mock_download_imagery

    @staticmethod
    def _mock_triangulate(dem_path: Path, cell_bbox: tuple, *, lod: int = 0):
        """Return a minimal TerrainTileData without reading dem_path."""
        from las_terrain import TerrainTileData

        lon_min, lat_min, lon_max, lat_max = cell_bbox
        c_lat = math.radians((lat_min + lat_max) / 2.0)
        c_lon = math.radians((lon_min + lon_max) / 2.0)
        return TerrainTileData(
            lod=lod,
            centroid_lat_rad=c_lat,
            centroid_lon_rad=c_lon,
            centroid_height_m=0.0,
            lat_min_rad=math.radians(lat_min),
            lat_max_rad=math.radians(lat_max),
            lon_min_rad=math.radians(lon_min),
            lon_max_rad=math.radians(lon_max),
            height_min_m=0.0,
            height_max_m=0.0,
            vertices=np.zeros((3, 3), dtype=np.float32),
            indices=np.zeros((1, 3), dtype=np.uint32),
            colors=np.zeros((1, 3), dtype=np.uint8),
        )

    @staticmethod
    def _mock_colorize(tile, img_path: Path, *, source: str = "sentinel2"):
        """Return tile unchanged (colorization mocked)."""
        return tile

    @staticmethod
    def _mock_check(tile) -> None:
        """Accept any tile (mesh quality check mocked)."""

    def _all_mocks(self, bt, cache_dir: Path):
        """Return a context manager applying all pipeline stubs."""
        from contextlib import ExitStack

        stack = ExitStack()
        stack.enter_context(
            patch.object(bt, "download_dem", self._make_mock_download_dem(cache_dir))
        )
        stack.enter_context(
            patch.object(bt, "download_imagery", self._make_mock_download_imagery(cache_dir))
        )
        stack.enter_context(patch.object(bt, "triangulate", self._mock_triangulate))
        stack.enter_context(patch.object(bt, "colorize", self._mock_colorize))
        stack.enter_context(patch.object(bt, "check", self._mock_check))
        return stack

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------

    def test_output_files_created(
        self, tmp_path: Path, _aircraft_config: Path
    ) -> None:
        import build_terrain as bt

        with self._all_mocks(bt, tmp_path / "cache"):
            glb_path = bt.build_terrain(_aircraft_config)

        assert glb_path.exists(), "GLB file not created"
        assert glb_path.suffix == ".glb"

    def test_las_terrain_file_created(
        self, tmp_path: Path, _aircraft_config: Path
    ) -> None:
        import build_terrain as bt
        from terrain_paths import las_terrain_dir

        with self._all_mocks(bt, tmp_path / "cache"):
            bt.build_terrain(_aircraft_config)

        las_path = las_terrain_dir("test_aircraft") / "terrain.las_terrain"
        assert las_path.exists(), ".las_terrain file not created"

    def test_metadata_json_created(
        self, tmp_path: Path, _aircraft_config: Path
    ) -> None:
        import build_terrain as bt
        from terrain_paths import metadata_path

        with self._all_mocks(bt, tmp_path / "cache"):
            bt.build_terrain(_aircraft_config)

        meta_path = metadata_path("test_aircraft")
        assert meta_path.exists(), "metadata.json not created"
        meta = json.loads(meta_path.read_text())
        assert meta["schema_version"] == 1
        assert meta["dataset_name"] == "test_aircraft"
        assert meta["lod_range"] == [0, 6]

    def test_godot_terrain_config_created(
        self, tmp_path: Path, _aircraft_config: Path
    ) -> None:
        import build_terrain as bt

        with self._all_mocks(bt, tmp_path / "cache"):
            bt.build_terrain(_aircraft_config)

        config_path = tmp_path / "godot" / "terrain" / "terrain_config.json"
        assert config_path.exists(), "terrain_config.json not created in godot/terrain/"
        cfg = json.loads(config_path.read_text())
        assert cfg["schema_version"] == 1
        assert cfg["glb_path"] == "res://terrain/terrain.glb"
        assert "world_origin_lat_rad" in cfg

    def test_godot_glb_copied(
        self, tmp_path: Path, _aircraft_config: Path
    ) -> None:
        import build_terrain as bt

        with self._all_mocks(bt, tmp_path / "cache"):
            bt.build_terrain(_aircraft_config)

        godot_glb = tmp_path / "godot" / "terrain" / "terrain.glb"
        assert godot_glb.exists(), "terrain.glb not copied to godot/terrain/"

    def test_display_tiles_subset_of_all_tiles(
        self, tmp_path: Path, _aircraft_config: Path
    ) -> None:
        import build_terrain as bt
        from las_terrain import read_las_terrain
        from terrain_paths import gltf_path, las_terrain_dir

        with self._all_mocks(bt, tmp_path / "cache"):
            bt.build_terrain(_aircraft_config)

        # .las_terrain holds all LODs; GLB holds only display subset.
        las_path = las_terrain_dir("test_aircraft") / "terrain.las_terrain"
        all_tiles = read_las_terrain(las_path)

        glb_bytes = gltf_path("test_aircraft").read_bytes()
        assert glb_bytes[:4] == b"glTF"

        # All 7 LODs must be present in the .las_terrain.
        lods_present = {t.lod for t in all_tiles}
        assert lods_present == set(range(7)), f"Missing LODs: {set(range(7)) - lods_present}"

    def test_glb_node_names_contain_lod(
        self, tmp_path: Path, _aircraft_config: Path
    ) -> None:
        """Node names in the GLB must follow the tile_L{N}_... convention."""
        import json as _json
        import struct

        import build_terrain as bt
        from terrain_paths import gltf_path

        with self._all_mocks(bt, tmp_path / "cache"):
            bt.build_terrain(_aircraft_config)

        glb_bytes = gltf_path("test_aircraft").read_bytes()
        chunk_length = struct.unpack("<I", glb_bytes[12:16])[0]
        gltf = _json.loads(glb_bytes[20 : 20 + chunk_length].rstrip(b"\x20"))

        node_names = [n.get("name", "") for n in gltf.get("nodes", [])]
        assert any(n.startswith("tile_L") for n in node_names), (
            f"No tile_L* nodes found in GLB. Node names: {node_names}"
        )

    def test_terrain_config_includes_default_aircraft_mesh_path(
        self, tmp_path: Path, _aircraft_config: Path
    ) -> None:
        """terrain_config.json must include aircraft_mesh_path with the default
        value when visualization section is absent from the aircraft config."""
        import build_terrain as bt

        with self._all_mocks(bt, tmp_path / "cache"):
            bt.build_terrain(_aircraft_config)

        config_path = tmp_path / "godot" / "terrain" / "terrain_config.json"
        cfg = json.loads(config_path.read_text())
        assert "aircraft_mesh_path" in cfg, "aircraft_mesh_path missing from terrain_config.json"
        assert cfg["aircraft_mesh_path"] == "res://assets/aircraft_lp.glb"

    def test_terrain_config_aircraft_mesh_path_from_visualization_section(
        self, tmp_path: Path
    ) -> None:
        """terrain_config.json uses mesh_res_path from the visualization section
        when it is present in the aircraft config."""
        import build_terrain as bt

        cfg = _minimal_config(radius_km=1.0)
        cfg["visualization"] = {"mesh_res_path": "res://assets/jet_trainer.glb"}
        path = tmp_path / "jet.json"
        path.write_text(json.dumps(cfg))

        with self._all_mocks(bt, tmp_path / "cache"):
            bt.build_terrain(path)

        config_path = tmp_path / "godot" / "terrain" / "terrain_config.json"
        result = json.loads(config_path.read_text())
        assert result["aircraft_mesh_path"] == "res://assets/jet_trainer.glb"


# ---------------------------------------------------------------------------
# Unit tests — _build_terrain_config helper
# ---------------------------------------------------------------------------

class TestBuildTerrainConfig:
    def test_aircraft_mesh_path_from_visualization_section(self) -> None:
        import build_terrain as bt

        config = {"visualization": {"mesh_res_path": "res://assets/custom.glb"}}
        result = bt._build_terrain_config(
            config=config,
            dataset_name="ds",
            center_lat_rad=0.1,
            center_lon_rad=0.2,
            center_h_m=10.0,
        )
        assert result["aircraft_mesh_path"] == "res://assets/custom.glb"

    def test_aircraft_mesh_path_default_when_no_visualization(self) -> None:
        import build_terrain as bt

        result = bt._build_terrain_config(
            config={},
            dataset_name="ds",
            center_lat_rad=0.1,
            center_lon_rad=0.2,
            center_h_m=10.0,
        )
        assert result["aircraft_mesh_path"] == "res://assets/aircraft_lp.glb"

    def test_aircraft_mesh_path_default_when_mesh_res_path_absent(self) -> None:
        import build_terrain as bt

        result = bt._build_terrain_config(
            config={"visualization": {}},
            dataset_name="ds",
            center_lat_rad=0.1,
            center_lon_rad=0.2,
            center_h_m=10.0,
        )
        assert result["aircraft_mesh_path"] == "res://assets/aircraft_lp.glb"

    def test_required_fields_present(self) -> None:
        import build_terrain as bt

        result = bt._build_terrain_config(
            config={},
            dataset_name="ksba_ga",
            center_lat_rad=0.60073,
            center_lon_rad=-2.09139,
            center_h_m=3.0,
        )
        for key in ("schema_version", "dataset_name", "glb_path",
                    "world_origin_lat_rad", "world_origin_lon_rad",
                    "world_origin_height_m", "aircraft_mesh_path"):
            assert key in result, f"missing key: {key}"
        assert result["schema_version"] == 1
        assert result["glb_path"] == "res://terrain/terrain.glb"
