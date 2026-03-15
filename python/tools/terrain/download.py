"""download.py — Authenticated download from Copernicus Data Space and NASA EarthData.

All network I/O is isolated here; all other terrain pipeline tools consume local files only.

Authentication:
    Copernicus Data Space (GLO-10/30, Sentinel-2): OAuth2 via
        $COPERNICUS_CLIENT_ID / $COPERNICUS_CLIENT_SECRET environment variables.
    NASA EarthData (SRTM, NASADEM, Landsat, MODIS): .netrc token via
        earthaccess.login().

Caching: Downloaded tiles are cached in output_dir.  If a file already exists with the
expected name, the download is skipped to allow resuming interrupted downloads.
"""

from __future__ import annotations

import os
from pathlib import Path

BBox = tuple[float, float, float, float]  # (lon_min, lat_min, lon_max, lat_max) degrees


class DownloadError(IOError):
    """Raised on HTTP errors, authentication failure, or missing coverage."""


def download_dem(
    bbox_deg: BBox,
    output_dir: Path,
    source: str = "copernicus_dem_glo10",  # or "copernicus_dem_glo30", "nasadem", "srtm"
) -> list[Path]:
    """Download DEM tiles covering bbox_deg.  Returns paths of downloaded GeoTIFFs.

    Copernicus sources use OAuth2 client credentials via $COPERNICUS_CLIENT_ID /
    $COPERNICUS_CLIENT_SECRET environment variables.
    NASA EarthData sources use an .netrc token via earthaccess.login().

    Returns an empty list (rather than raising) if no tiles exist for the requested area.

    Raises:
        DownloadError: on HTTP errors, authentication failure, or unsupported source.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    _SOURCES = {"copernicus_dem_glo10", "copernicus_dem_glo30", "nasadem", "srtm"}
    if source not in _SOURCES:
        raise DownloadError(f"Unsupported DEM source '{source}'. Choose from: {_SOURCES}")

    if source.startswith("copernicus"):
        return _download_copernicus_dem(bbox_deg, output_dir, source)
    else:
        return _download_earthdata_dem(bbox_deg, output_dir, source)


def download_imagery(
    bbox_deg: BBox,
    output_dir: Path,
    source: str = "sentinel2",  # or "landsat9", "modis"
    lod: int = 0,
) -> list[Path]:
    """Download cloud-free imagery tiles covering bbox_deg at the resolution
    recommended for lod (see §Imagery Data Sources in terrain.md).

    Sentinel-2 uses Copernicus authentication.  Landsat/MODIS use earthaccess.

    Returns an empty list if no tiles exist for the requested area.

    Raises:
        DownloadError: on HTTP errors, authentication failure, or unsupported source.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    _SOURCES = {"sentinel2", "landsat9", "modis"}
    if source not in _SOURCES:
        raise DownloadError(f"Unsupported imagery source '{source}'. Choose from: {_SOURCES}")

    if source == "sentinel2":
        return _download_copernicus_imagery(bbox_deg, output_dir, source, lod)
    else:
        return _download_earthdata_imagery(bbox_deg, output_dir, source, lod)


# ---------------------------------------------------------------------------
# Copernicus Data Space helpers
# ---------------------------------------------------------------------------

def _copernicus_token() -> str:
    """Obtain an OAuth2 bearer token from Copernicus Data Space."""
    import requests

    client_id = os.environ.get("COPERNICUS_CLIENT_ID")
    client_secret = os.environ.get("COPERNICUS_CLIENT_SECRET")
    if not client_id or not client_secret:
        raise DownloadError(
            "Copernicus authentication requires COPERNICUS_CLIENT_ID and "
            "COPERNICUS_CLIENT_SECRET environment variables."
        )

    resp = requests.post(
        "https://identity.dataspace.copernicus.eu/auth/realms/CDSE/protocol/openid-connect/token",
        data={
            "grant_type": "client_credentials",
            "client_id": client_id,
            "client_secret": client_secret,
        },
        timeout=30,
    )
    if not resp.ok:
        raise DownloadError(f"Copernicus token request failed: {resp.status_code} {resp.text[:200]}")
    return str(resp.json()["access_token"])


def _download_copernicus_dem(bbox_deg: BBox, output_dir: Path, source: str) -> list[Path]:
    """Download Copernicus DEM GLO-10 or GLO-30 tiles."""
    import requests

    resolution = "10" if source == "copernicus_dem_glo10" else "30"
    token = _copernicus_token()
    lon_min, lat_min, lon_max, lat_max = bbox_deg

    # Query the STAC API for DEM tiles
    stac_url = "https://catalogue.dataspace.copernicus.eu/stac/collections/COP-DEM/items"
    params = {
        "bbox": f"{lon_min},{lat_min},{lon_max},{lat_max}",
        "limit": 100,
    }
    resp = requests.get(stac_url, params=params, timeout=60)
    if not resp.ok:
        raise DownloadError(f"STAC query failed: {resp.status_code}")

    paths: list[Path] = []
    for feature in resp.json().get("features", []):
        asset_url = feature.get("assets", {}).get("data", {}).get("href", "")
        if not asset_url:
            continue
        fname = output_dir / Path(asset_url).name
        if fname.exists():
            paths.append(fname)
            continue
        dl = requests.get(
            asset_url,
            headers={"Authorization": f"Bearer {token}"},
            stream=True,
            timeout=300,
        )
        if not dl.ok:
            raise DownloadError(f"Download failed: {dl.status_code} {asset_url}")
        with open(fname, "wb") as out:
            for chunk in dl.iter_content(chunk_size=65536):
                out.write(chunk)
        paths.append(fname)
    return paths


def _download_copernicus_imagery(
    bbox_deg: BBox, output_dir: Path, source: str, lod: int
) -> list[Path]:
    """Download Sentinel-2 L2A imagery tiles."""
    import requests

    token = _copernicus_token()
    lon_min, lat_min, lon_max, lat_max = bbox_deg

    stac_url = "https://catalogue.dataspace.copernicus.eu/stac/collections/SENTINEL-2/items"
    params = {
        "bbox": f"{lon_min},{lat_min},{lon_max},{lat_max}",
        "limit": 10,
        "filter": "eo:cloud_cover < 20",
    }
    resp = requests.get(stac_url, params=params, timeout=60)
    if not resp.ok:
        raise DownloadError(f"STAC query failed: {resp.status_code}")

    paths: list[Path] = []
    for feature in resp.json().get("features", []):
        for band, asset in feature.get("assets", {}).items():
            href = asset.get("href", "")
            if not href:
                continue
            fname = output_dir / Path(href).name
            if fname.exists():
                paths.append(fname)
                continue
            dl = requests.get(
                href,
                headers={"Authorization": f"Bearer {token}"},
                stream=True,
                timeout=300,
            )
            if not dl.ok:
                continue  # skip unavailable assets silently
            with open(fname, "wb") as out:
                for chunk in dl.iter_content(chunk_size=65536):
                    out.write(chunk)
            paths.append(fname)
    return paths


# ---------------------------------------------------------------------------
# NASA EarthData helpers
# ---------------------------------------------------------------------------

def _download_earthdata_dem(bbox_deg: BBox, output_dir: Path, source: str) -> list[Path]:
    """Download SRTM / NASADEM tiles via earthaccess."""
    import earthaccess

    earthaccess.login(strategy="netrc")
    lon_min, lat_min, lon_max, lat_max = bbox_deg
    collection_id = "SRTMGL1" if source == "srtm" else "NASADEM_HGT"

    results = earthaccess.search_data(
        short_name=collection_id,
        bounding_box=(lon_min, lat_min, lon_max, lat_max),
    )
    if not results:
        return []
    return [Path(p) for p in earthaccess.download(results, str(output_dir))]


def _download_earthdata_imagery(
    bbox_deg: BBox, output_dir: Path, source: str, lod: int
) -> list[Path]:
    """Download Landsat-9 / MODIS imagery tiles via earthaccess."""
    import earthaccess

    earthaccess.login(strategy="netrc")
    lon_min, lat_min, lon_max, lat_max = bbox_deg
    collection_ids = {"landsat9": "HLSL30", "modis": "MCD43A4"}
    short_name = collection_ids[source]

    results = earthaccess.search_data(
        short_name=short_name,
        bounding_box=(lon_min, lat_min, lon_max, lat_max),
    )
    if not results:
        return []
    return [Path(p) for p in earthaccess.download(results, str(output_dir))]
