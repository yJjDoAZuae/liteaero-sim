# External Dependencies

## License Policy

Prefer permissive open-source licenses. See [guidelines/general.md](../guidelines/general.md#external-dependencies-and-licensing) for the full policy.

| License | Acceptability |
| --- | --- |
| MIT, BSD-2/3-Clause, Clear BSD, Apache 2.0, Boost, ISC | ✅ Preferred |
| LGPL (any) | ⚠️ Acceptable with dynamic linking only |
| GPL (any) | ❌ Avoid |
| Proprietary | ❌ Avoid |

## Current Dependencies

### C++ Dependencies

| Library | Version / SHA | License | Integration | Purpose |
| --- | --- | --- | --- | --- |
| [Eigen3](https://eigen.tuxfamily.org) | 3.4.0 | MPL-2 | Conan | Linear algebra — matrices, vectors, state-space |
| [nlohmann/json](https://github.com/nlohmann/json) | 3.12.0 | MIT | Conan | JSON serialization / deserialization |
| [googletest](https://github.com/google/googletest) | 1.14.0 | BSD-3-Clause | Conan | Unit testing (gtest + gmock) |
| [protobuf](https://github.com/protocolbuffers/protobuf) | 3.21.12 | BSD-3-Clause | Conan | Binary serialization (proto3 wire format) |
| [SDL2](https://www.libsdl.org) | 2.28.5 | zlib | Conan | Keyboard and joystick input |
| [pybind11](https://github.com/pybind/pybind11) | 2.11.1 | BSD-3-Clause | Conan | Python bindings (optional — `LITEAERO_SIM_BUILD_PYTHON_BINDINGS`) |
| [trochoids](https://github.com/castacks/trochoids) | `38d23eb` | Clear BSD | FetchContent (pattern 1b) | Dubins and trochoidal path planning |
| [tinygltf](https://github.com/syoyo/tinygltf) | v2.9.3 | MIT | FetchContent (pattern 1b) | glTF 2.0 / GLB export |

### Python Dependencies

Declared in `python/pyproject.toml`. Python version pinned in `python/.python-version`.

| Package | License | Purpose |
| --- | --- | --- |
| numpy | BSD-3-Clause | Numerical arrays |
| matplotlib | PSF | Plotting |
| rasterio | BSD-3-Clause | GeoTIFF I/O |
| scipy | BSD-3-Clause | Delaunay triangulation, KD-tree |
| pyproj | MIT | Coordinate reference system / geoid |
| pyfqmr | MIT | QEM mesh simplification |
| trimesh | MIT | GLB/glTF mesh export |
| requests | Apache-2.0 | Sentinel Hub Process API + Copernicus OAuth2 |
| earthaccess | MIT | NASA EarthData download |
| pyvista | MIT | 3D visualization |
| ipywidgets | BSD-3-Clause | Jupyter widget support |
| ipympl | BSD-3-Clause | `%matplotlib widget` backend — interactive matplotlib figures in Jupyter cells |
| pyside6 | LGPL-3.0 | Qt window, event loop, `QTimer` — standalone GUI apps only; dynamically linked |
| pytest | MIT | Unit testing |
| pytest-cov | MIT | Coverage reporting |
| mypy | MIT | Static type checking |
| black | MIT | Code formatting |
| ruff | MIT | Linting |
| jupyterlab | BSD-3-Clause | Notebook server |
| ipykernel | BSD-3-Clause | Jupyter Python kernel |

### Copernicus Data Access — Sentinel Hub Process API

DEM and Sentinel-2 imagery are downloaded via the **Sentinel Hub Process API**:

```http
POST https://sh.dataspace.copernicus.eu/api/v1/process
Authorization: Bearer <OAuth2 client_credentials token>
```

Authentication uses the same `COPERNICUS_CLIENT_ID` / `COPERNICUS_CLIENT_SECRET` credentials
and the same token endpoint as other CDSE services.  A single request returns a GeoTIFF
directly in the response body — no STAC query, no S3 access required.

The CDSE STAC API does provide DEM tile metadata (`cop-dem-glo-30-dged-cog` collection),
but the asset hrefs are S3-only (`s3://eodata/...`).  Converting them to HTTPS
(`https://eodata.dataspace.copernicus.eu/...`) returns 403 `InvalidAccessKeyId` because
that endpoint requires AWS S3 credentials, not Bearer tokens.  The OData Products API
does not index individual DEM COG tiles.  The Sentinel Hub Process API is the only
supported HTTPS download path for Copernicus DEM data.

Reference: [Sentinel Hub API Reference](https://documentation.dataspace.copernicus.eu/APIs/SentinelHub/ApiReference.html)

### ipykernel Version Constraint

ipykernel 7.1.0 through at least 7.2.0 have a concurrent-context bug that causes
`RuntimeError: cannot enter context: already entered` in the JupyterLab server console
whenever a pyvista/trame 3D widget cell runs.  The bug is patched at kernel startup by
`scripts/kernel_launcher.py` (see below).

**Root cause:** ipykernel PR #1462 (released in 7.1.0) introduced `_async_in_context()` in
`ipykernel/utils.py` to fix ContextVar persistence across cells (issue #1457).  The
implementation captures a single `Context` object at closure-creation time and reuses it
for every `asyncio.create_task()` call:

```python
context = copy_context()          # captured ONCE at closure creation

async def run_in_context(*args, **kwargs):
    coro = f(*args, **kwargs)
    return await asyncio.create_task(coro, context=context)  # same object every call
```

CPython's `Context.run()` enforces mutual exclusion on **every Python version**: a Context
object can only be entered by one execution frame at a time.  When pyvista activates a
trame widget, trame's tornado server schedules async callbacks from a background thread
concurrently with the asyncio event loop running `shell_main`.  Both frames try to enter
the same shared `context` object and the second `PyContext_Enter` fails:

```text
Exception in callback Task.__step()
RuntimeError: cannot enter context: <_contextvars.Context object> is already entered
```

**Fix — custom kernel launcher (`scripts/kernel_launcher.py`):**

The fix is to call `copy_context()` _inside_ `run_in_context` (once per task) rather than
capturing it once at closure-creation time.  This is applied via a monkey-patch at kernel
startup in `scripts/kernel_launcher.py`:

```python
@functools.wraps(original)
def _patched(f, context=None):
    @functools.wraps(f)
    async def run_in_context(*args, **kwargs):
        return await asyncio.create_task(
            f(*args, **kwargs),
            context=copy_context(),   # fresh copy — not the shared closure object
        )
    return run_in_context

ipykernel.utils._async_in_context = _patched
```

The launcher patches `ipykernel.utils._async_in_context` **before** `ipykernel.kernelbase`
is imported (kernelbase does `from .utils import _async_in_context` at import time, binding
the name once).  The patched name is what kernelbase binds.

The kernel spec (`kernel.json`) points at `kernel_launcher.py` instead of the default
`ipykernel_launcher`.  Install the spec once during initial project setup:

```bash
uv run python scripts/install_kernel.py
```

The spec is installed to the **user Jupyter data directory**
(`%APPDATA%\jupyter\kernels\` on Windows, `~/.local/share/jupyter/kernels/` on
Linux/macOS), which is outside the venv and survives all `uv sync` runs including full
venv rebuilds.  Re-run `install_kernel.py` only if the project directory is moved
(the kernel spec contains an absolute path to `kernel_launcher.py`).

**Version range known to have the bug:** 7.1.0 – 7.2.0.  `kernel_launcher.py` emits a
`warnings.warn` if the installed version is newer than 7.2.0, which is the trigger to
re-evaluate whether the fix has been merged upstream.

**To re-evaluate after a new ipykernel release:**

1. Check the ipykernel changelog / GitHub: look for a fix that calls `copy_context()` per
   invocation inside `run_in_context` rather than once at closure-creation time.
2. Update `_PATCH_IPYKERNEL_VERSION_RANGE` in `scripts/kernel_launcher.py` to note the
   new upper bound (or remove the patch call if fixed upstream).
3. Update the `ipykernel` lower bound in `python/pyproject.toml` if appropriate.
4. Run `uv sync --group dev` followed by `uv run python scripts/install_kernel.py`.
5. Run `uv run pytest test/test_ipykernel_compat.py -v` — T2 and T3 must pass; T1 skips
   when running via plain pytest (patch not active outside the kernel).
6. Start JupyterLab (`uv run jupyter lab`), open a terrain notebook, and run the 3D
   visualization cell.  Confirm no `RuntimeError: cannot enter context` appears in the
   server console.

**Test coverage:** `python/test/test_ipykernel_compat.py` contains three tests:

- T1 — verifies post-patch isolation (skip when patch not active; meaningful inside kernel)
- T2 — documents the CPython mutual-exclusion invariant that makes the bug possible
- T3 — demonstrates that independent `copy_context()` per task is safe for concurrent use

## Integration Methods

### Conan — Preferred (Eigen3, nlohmann_json, googletest, protobuf)

Used for all packages available in ConanCenter. Declare in `conanfile.txt`; locate with `find_package()` after the Conan toolchain is loaded.

```cmake
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(nlohmann_json REQUIRED)
find_package(GTest REQUIRED)
find_package(protobuf REQUIRED CONFIG)
```

### FetchContent Pattern 1b — Source with incompatible build system (trochoids, mcap, tinygltf)

Used for packages not in ConanCenter whose upstream build system is incompatible. Source is downloaded and compiled with a manually defined CMake target, bypassing the upstream `CMakeLists.txt`.

```cmake
FetchContent_Declare(
    trochoids
    GIT_REPOSITORY https://github.com/castacks/trochoids.git
    GIT_TAG        38d23eb3346737fe9d6e9ff57c742113e29dfe4f
)
FetchContent_GetProperties(trochoids)
if(NOT trochoids_POPULATED)
    FetchContent_Populate(trochoids)
    add_library(trochoids STATIC
        ${trochoids_SOURCE_DIR}/src/trochoids.cpp
        ${trochoids_SOURCE_DIR}/src/trochoid_utils.cpp
        ${trochoids_SOURCE_DIR}/src/DubinsStateSpace.cpp
    )
    target_include_directories(trochoids SYSTEM PUBLIC
        ${trochoids_SOURCE_DIR}/include
    )
endif()
```

> **Note:** `FetchContent_Populate(<name>)` is deprecated in CMake 3.30. If the project minimum is raised to 3.28+, migrate to `FetchContent_MakeAvailable` with a cmake override directory.

## Adding a New Dependency

1. Check the license against the policy above.
2. Choose the integration method — in ConanCenter? Use Conan. Source available but not in ConanCenter? Use FetchContent pattern 1b. Binary-only? Vendor in `libs/` as a last resort.
3. Add to `conanfile.txt` (if Conan) or `CMakeLists.txt` dependency registry comment block.
4. Record in this document.
