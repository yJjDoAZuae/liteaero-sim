"""Tests for terrain_paths — python/tools/terrain/terrain_paths.py."""
from __future__ import annotations

from pathlib import Path


class TestTerrainDataRoot:
    def test_default_root_is_under_data_terrain(self, monkeypatch) -> None:
        """Without LITEAERO_TERRAIN_ROOT, root should be project_root/data/terrain."""
        monkeypatch.delenv("LITEAERO_TERRAIN_ROOT", raising=False)
        from terrain_paths import get_terrain_data_root
        root = get_terrain_data_root()
        assert isinstance(root, Path)
        assert root.name == "terrain"
        assert root.parent.name == "data"

    def test_default_root_is_absolute(self, monkeypatch) -> None:
        monkeypatch.delenv("LITEAERO_TERRAIN_ROOT", raising=False)
        from terrain_paths import get_terrain_data_root
        assert get_terrain_data_root().is_absolute()

    def test_env_override(self, monkeypatch, tmp_path: Path) -> None:
        """LITEAERO_TERRAIN_ROOT env var overrides the default."""
        monkeypatch.setenv("LITEAERO_TERRAIN_ROOT", str(tmp_path))
        from terrain_paths import get_terrain_data_root
        assert get_terrain_data_root() == tmp_path

    def test_env_override_returns_path_object(self, monkeypatch, tmp_path: Path) -> None:
        monkeypatch.setenv("LITEAERO_TERRAIN_ROOT", str(tmp_path))
        from terrain_paths import get_terrain_data_root
        assert isinstance(get_terrain_data_root(), Path)

    def test_module_constant_is_path(self) -> None:
        """TERRAIN_DATA_ROOT module-level constant must be a Path."""
        import terrain_paths
        assert isinstance(terrain_paths.TERRAIN_DATA_ROOT, Path)


class TestDatasetHelpers:
    def test_dataset_dir_is_under_root(self, monkeypatch, tmp_path: Path) -> None:
        monkeypatch.setenv("LITEAERO_TERRAIN_ROOT", str(tmp_path))
        from terrain_paths import dataset_dir, get_terrain_data_root
        assert dataset_dir("test_dataset") == get_terrain_data_root() / "test_dataset"

    def test_source_dir_is_under_dataset(self, monkeypatch, tmp_path: Path) -> None:
        monkeypatch.setenv("LITEAERO_TERRAIN_ROOT", str(tmp_path))
        from terrain_paths import source_dir, dataset_dir
        assert source_dir("ds") == dataset_dir("ds") / "source"

    def test_derived_dir_is_under_dataset(self, monkeypatch, tmp_path: Path) -> None:
        monkeypatch.setenv("LITEAERO_TERRAIN_ROOT", str(tmp_path))
        from terrain_paths import derived_dir, dataset_dir
        assert derived_dir("ds") == dataset_dir("ds") / "derived"

    def test_las_terrain_dir_is_under_derived(self, monkeypatch, tmp_path: Path) -> None:
        monkeypatch.setenv("LITEAERO_TERRAIN_ROOT", str(tmp_path))
        from terrain_paths import las_terrain_dir, derived_dir
        assert las_terrain_dir("ds") == derived_dir("ds") / "las_terrain"

    def test_gltf_path_ends_with_terrain_glb(self, monkeypatch, tmp_path: Path) -> None:
        monkeypatch.setenv("LITEAERO_TERRAIN_ROOT", str(tmp_path))
        from terrain_paths import gltf_path, derived_dir
        assert gltf_path("ds") == derived_dir("ds") / "gltf" / "terrain.glb"

    def test_metadata_path_ends_with_json(self, monkeypatch, tmp_path: Path) -> None:
        monkeypatch.setenv("LITEAERO_TERRAIN_ROOT", str(tmp_path))
        from terrain_paths import metadata_path, derived_dir
        assert metadata_path("ds") == derived_dir("ds") / "metadata.json"
