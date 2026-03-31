"""terrain_paths — Central path constants for the terrain data repository.

``TERRAIN_DATA_ROOT`` defaults to ``data/terrain/`` under the liteaero-sim
project root (``Path(__file__).parents[3]``).  Override by setting
``LITEAERO_TERRAIN_ROOT`` to an absolute path.

Repository structure under ``TERRAIN_DATA_ROOT``::

    <dataset_name>/
        source/           raw downloaded files (DEM GeoTIFF, imagery GeoTIFF)
        derived/
            las_terrain/  LAS-format triangulated terrain
            gltf/
                terrain.glb
            metadata.json
"""
from __future__ import annotations

import os
from pathlib import Path


def get_terrain_data_root() -> Path:
    """Return the terrain data root directory.

    Resolution order:
    1. ``LITEAERO_TERRAIN_ROOT`` environment variable (if set).
    2. ``data/terrain/`` relative to the liteaero-sim project root
       (``Path(__file__).parents[3]``).
    """
    env = os.environ.get("LITEAERO_TERRAIN_ROOT")
    if env:
        return Path(env)
    return Path(__file__).parents[3] / "data" / "terrain"


TERRAIN_DATA_ROOT: Path = get_terrain_data_root()


def dataset_dir(dataset_name: str) -> Path:
    """Return the root directory for a named terrain dataset."""
    return get_terrain_data_root() / dataset_name


def source_dir(dataset_name: str) -> Path:
    """Return the source (raw download) directory for a dataset."""
    return dataset_dir(dataset_name) / "source"


def derived_dir(dataset_name: str) -> Path:
    """Return the derived-data directory for a dataset."""
    return dataset_dir(dataset_name) / "derived"


def las_terrain_dir(dataset_name: str) -> Path:
    """Return the LAS terrain directory for a dataset."""
    return derived_dir(dataset_name) / "las_terrain"


def gltf_path(dataset_name: str) -> Path:
    """Return the path to the exported terrain GLB for a dataset."""
    return derived_dir(dataset_name) / "gltf" / "terrain.glb"


def metadata_path(dataset_name: str) -> Path:
    """Return the path to the dataset metadata JSON file."""
    return derived_dir(dataset_name) / "metadata.json"
