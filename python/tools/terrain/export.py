"""export.py — Thin wrapper around las_terrain.write_las_terrain().

Provides a pipeline-uniform entry point for serializing tiles to .las_terrain files.
"""

from __future__ import annotations

from pathlib import Path

from las_terrain import TerrainTileData, write_las_terrain


def export_las_terrain(
    tiles: list[TerrainTileData],
    output_path: Path,
) -> None:
    """Write tiles to a .las_terrain file.

    Equivalent to calling write_las_terrain() directly; provided for pipeline
    uniformity so all pipeline stages have a named export entry point.

    Raises:
        ValueError: if tiles is empty.
        IOError:    on write failure.
    """
    write_las_terrain(output_path, tiles)
