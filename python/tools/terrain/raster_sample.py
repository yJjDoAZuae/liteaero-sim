"""raster_sample.py — In-memory raster with vectorized nearest-cell sampling.

The terrain build samples a DEM (heights) and an imagery mosaic (per-facet color)
at every grid point of every tile.  rasterio's ``DatasetReader.sample()`` performs
one windowed read *per point*, so tiling a region into many small cells re-pays that
per-point I/O thousands of times — the dominant cost of the terrain build.

``RasterSampler`` reads the whole raster into memory once, then samples arbitrary
lon/lat points by flooring to the pixel that contains each point (the same cell
``DatasetReader.sample()`` selects).  Sampling is a single vectorized fancy-index,
so the per-cell cost drops from hundreds of milliseconds to well under a millisecond.

The result is bit-identical to ``DatasetReader.sample()`` for points inside the
raster; points outside clamp to the nearest edge pixel.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from affine import Affine


@dataclass(frozen=True)
class RasterSampler:
    """A raster held in memory for fast vectorized point sampling.

    ``values`` is ``(height, width)`` for a single band or ``(bands, height, width)``
    for a multiband raster; ``sample()`` returns ``(N,)`` or ``(bands, N)`` to match.
    """

    values: np.ndarray
    inverse_transform: Affine  # maps (lon, lat) -> (col, row) in fractional pixels
    width: int
    height: int
    nodata: float | None
    bounds: tuple[float, float, float, float]  # (left, bottom, right, top)

    @classmethod
    def from_path(cls, path: Path, *, multiband: bool = False) -> "RasterSampler":
        """Read the full raster at ``path`` into memory.

        With ``multiband=True`` all bands are read as ``(bands, height, width)``;
        otherwise only band 1 is read as ``(height, width)``.
        """
        import rasterio

        with rasterio.open(path) as src:
            values = src.read() if multiband else src.read(1)
            b = src.bounds
            return cls(
                values=values,
                inverse_transform=~src.transform,
                width=src.width,
                height=src.height,
                nodata=src.nodata,
                bounds=(b.left, b.bottom, b.right, b.top),
            )

    def sample(self, lon_deg: np.ndarray, lat_deg: np.ndarray) -> np.ndarray:
        """Sample the raster at the given lon/lat arrays (nearest containing pixel).

        Returns ``(N,)`` for a single-band raster or ``(bands, N)`` for multiband.
        Points outside the raster clamp to the nearest edge pixel.
        """
        lon = np.asarray(lon_deg, dtype=np.float64)
        lat = np.asarray(lat_deg, dtype=np.float64)
        # Affine * (xs, ys) applies the transform element-wise to the arrays.
        cols, rows = self.inverse_transform * (lon, lat)
        col_idx = np.clip(np.floor(cols).astype(np.intp), 0, self.width - 1)
        row_idx = np.clip(np.floor(rows).astype(np.intp), 0, self.height - 1)
        if self.values.ndim == 2:
            return self.values[row_idx, col_idx]
        return self.values[:, row_idx, col_idx]

    def covers(self, bbox_deg: tuple[float, float, float, float], eps: float = 1e-8) -> bool:
        """Return True if ``bbox_deg`` (lon_min, lat_min, lon_max, lat_max) is within bounds."""
        lon_min, lat_min, lon_max, lat_max = bbox_deg
        left, bottom, right, top = self.bounds
        return (
            lon_min >= left - eps
            and lat_min >= bottom - eps
            and lon_max <= right + eps
            and lat_max <= top + eps
        )
