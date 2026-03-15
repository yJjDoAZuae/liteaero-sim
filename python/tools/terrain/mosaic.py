"""mosaic.py — Merge and reproject GeoTIFF tiles to a common CRS and resolution.

Uses rasterio.merge and rasterio.warp to produce a single seamless output raster from
multiple input GeoTIFFs.
"""

from __future__ import annotations

from pathlib import Path


def mosaic_dem(
    input_paths: list[Path],
    output_path: Path,
    target_epsg: int = 4326,
    target_resolution_deg: float | None = None,  # None → keep finest native resolution
    resampling: str = "bilinear",
) -> None:
    """Merge and reproject input GeoTIFFs into a single output GeoTIFF.

    Algorithm:
    1. Open all input GeoTIFFs with rasterio.
    2. Determine union bounding box and minimum native resolution.
    3. If target_resolution_deg is None, use the native resolution of the finest source.
    4. Call rasterio.merge.merge() to blend overlapping regions (first-source-wins).
    5. Reproject to target_epsg with rasterio.warp.reproject() using bilinear resampling.
    6. Write output as a single-band Cloud-Optimised GeoTIFF.

    Nodata values are propagated from source tiles.

    Raises:
        ValueError: if input_paths is empty.
    """
    if not input_paths:
        raise ValueError("input_paths must not be empty")

    import rasterio
    from rasterio.crs import CRS
    from rasterio.enums import Resampling
    from rasterio.merge import merge
    from rasterio.warp import calculate_default_transform, reproject

    _RESAMPLING = {
        "bilinear": Resampling.bilinear,
        "nearest": Resampling.nearest,
        "cubic": Resampling.cubic,
        "lanczos": Resampling.lanczos,
    }
    resamp = _RESAMPLING.get(resampling, Resampling.bilinear)
    target_crs = CRS.from_epsg(target_epsg)

    # Open all sources
    sources = [rasterio.open(p) for p in input_paths]
    try:
        mosaic_data, mosaic_transform = merge(sources, resampling=resamp)
        src_crs = sources[0].crs
        src_nodata = sources[0].nodata

        # Determine output resolution
        if target_resolution_deg is not None:
            res = target_resolution_deg
        else:
            # Use the finest native pixel size (minimum of absolute pixel sizes)
            res = min(abs(src.transform.a) for src in sources)

        # Calculate target transform
        dst_transform, dst_width, dst_height = calculate_default_transform(
            src_crs,
            target_crs,
            mosaic_data.shape[2],
            mosaic_data.shape[1],
            left=mosaic_transform.c,
            bottom=mosaic_transform.f + mosaic_transform.e * mosaic_data.shape[1],
            right=mosaic_transform.c + mosaic_transform.a * mosaic_data.shape[2],
            top=mosaic_transform.f,
            resolution=res,
        )

        output_path.parent.mkdir(parents=True, exist_ok=True)
        profile = sources[0].profile.copy()
        profile.update(
            crs=target_crs,
            transform=dst_transform,
            width=dst_width,
            height=dst_height,
            count=mosaic_data.shape[0],
            nodata=src_nodata,
            driver="GTiff",
        )

        with rasterio.open(output_path, "w", **profile) as dst:
            for band_idx in range(1, mosaic_data.shape[0] + 1):
                reproject(
                    source=mosaic_data[band_idx - 1],
                    destination=rasterio.band(dst, band_idx),
                    src_transform=mosaic_transform,
                    src_crs=src_crs,
                    dst_transform=dst_transform,
                    dst_crs=target_crs,
                    resampling=resamp,
                    src_nodata=src_nodata,
                )
    finally:
        for src in sources:
            src.close()


def mosaic_imagery(
    input_paths: list[Path],
    output_path: Path,
    target_epsg: int = 4326,
    target_resolution_deg: float | None = None,
    resampling: str = "nearest",
) -> None:
    """Same as mosaic_dem() but preserves all RGB/multispectral bands.

    Raises:
        ValueError: if input_paths is empty.
    """
    # Same algorithm; delegate with nearest resampling default for categorical data.
    mosaic_dem(input_paths, output_path, target_epsg, target_resolution_deg, resampling)
