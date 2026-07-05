"""raster_sample.py — In-memory raster with vectorized nearest-cell sampling.

The terrain build samples a DEM (heights) and an imagery mosaic (per-facet color)
at every grid point of every tile.  rasterio's ``DatasetReader.sample()`` performs
one windowed read *per point*, so tiling a region into many small cells re-pays that
per-point I/O thousands of times — the dominant cost of the terrain build.

``RasterSampler`` reads the whole raster into memory once, then samples arbitrary
lon/lat points by flooring to the pixel that contains each point (the same cell
``DatasetReader.sample()`` selects).  Sampling is a single vectorized fancy-index,
so the per-cell cost drops from hundreds of milliseconds to well under a millisecond.
It is bit-identical to ``DatasetReader.sample()`` for points inside the raster; points
outside clamp to the nearest edge pixel.  Reading the whole raster is ideal for a small
raster (a DEM), but for a large one (multi-gigabyte NAIP imagery) it holds the entire
array in memory — and with one copy per parallel worker that exhausts RAM.

``WindowedRasterSampler`` addresses that: it keeps the dataset handle open and, per
``sample()`` call, reads only the bounding **window** of the query points (with a
decimation cap so a query spanning the whole raster still reads a bounded block).  Memory
is bounded by the window, not the raster, so it scales to large imagery and many workers.
Use ``RasterSampler`` for the DEM and ``WindowedRasterSampler`` for imagery.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from affine import Affine

# When a query window exceeds this many pixels per axis, the windowed read is decimated to
# this cap (rasterio ``out_shape``) so a region-spanning coarse tile reads a bounded block
# rather than the whole raster.  2048 px keeps the read ≤ ~16 MB at 4 bands.
_WINDOW_DECIMATE_CAP: int = 2048


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


class WindowedRasterSampler:
    """Samples a raster by reading only the window covering each query, holding the dataset open.

    Unlike :class:`RasterSampler`, the whole raster is never loaded into memory — each
    ``sample()`` reads the bounding pixel window of its query points (decimated to
    ``_WINDOW_DECIMATE_CAP`` per axis when the window is larger), so memory is bounded by the
    window rather than the raster.  This scales to multi-gigabyte imagery and many parallel
    workers, at the cost of a small per-call windowed read.  The ``sample`` / ``bounds`` /
    ``nodata`` interface matches :class:`RasterSampler` so it is a drop-in for colorization.

    Holds an open ``rasterio`` dataset handle; construct one per process (it is not picklable).
    """

    def __init__(self, path: Path, *, multiband: bool = False) -> None:
        import rasterio

        self._src = rasterio.open(path)
        self._multiband = multiband
        self._inverse_transform: Affine = ~self._src.transform
        self.width: int = self._src.width
        self.height: int = self._src.height
        self.nodata: float | None = self._src.nodata
        b = self._src.bounds
        self.bounds: tuple[float, float, float, float] = (b.left, b.bottom, b.right, b.top)

    def sample(self, lon_deg: np.ndarray, lat_deg: np.ndarray) -> np.ndarray:
        """Sample by reading only the bounding window of the query points.

        Returns ``(N,)`` for a single-band raster or ``(bands, N)`` for multiband — matching
        :meth:`RasterSampler.sample`.  Points outside the raster clamp to the nearest edge pixel.
        """
        from rasterio.windows import Window

        lon = np.asarray(lon_deg, dtype=np.float64)
        lat = np.asarray(lat_deg, dtype=np.float64)
        cols, rows = self._inverse_transform * (lon, lat)
        col_idx = np.clip(np.floor(cols).astype(np.intp), 0, self.width - 1)
        row_idx = np.clip(np.floor(rows).astype(np.intp), 0, self.height - 1)

        c0 = int(col_idx.min())
        r0 = int(row_idx.min())
        win_w = int(col_idx.max()) - c0 + 1
        win_h = int(row_idx.max()) - r0 + 1
        window = Window(c0, r0, win_w, win_h)

        # Cap the read size: decimate a large window to at most the cap per axis.
        out_w = min(win_w, _WINDOW_DECIMATE_CAP)
        out_h = min(win_h, _WINDOW_DECIMATE_CAP)
        local_c = col_idx - c0
        local_r = row_idx - r0
        if out_w != win_w or out_h != win_h:
            # Read the window decimated to (out_h, out_w); remap query indices to the smaller grid.
            local_c = np.clip((local_c * out_w) // win_w, 0, out_w - 1)
            local_r = np.clip((local_r * out_h) // win_h, 0, out_h - 1)
            out_shape_2d = (out_h, out_w)
        else:
            out_shape_2d = (win_h, win_w)

        if self._multiband:
            n_bands = self._src.count
            block = self._src.read(window=window, out_shape=(n_bands, *out_shape_2d))
            return block[:, local_r, local_c]
        block = self._src.read(1, window=window, out_shape=out_shape_2d)
        return block[local_r, local_c]

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

    def close(self) -> None:
        """Close the underlying dataset handle."""
        self._src.close()
