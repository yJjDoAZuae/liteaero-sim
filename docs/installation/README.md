# Installation and Build

## Prerequisites

| Tool | Minimum Version | Notes |
| --- | --- | --- |
| CMake | 3.16 | [cmake.org](https://cmake.org/download/) |
| C++ Compiler | C++17 | MSVC 2019+, GCC 9+, or Clang 10+ |
| Git | Any recent | Required for FetchContent dependency downloads |
| Eigen3 | 3.4 | Must be installed separately (see below) |
| Python | 3.10+ | Optional — required for analysis scripts and notebooks |

## 1. Clone the Repository

```bash
git clone https://github.com/<org>/liteaero-sim.git
cd liteaero-sim
```

## 2. Install Eigen3

Eigen3 must be available as a system or user installation. It is a header-only library.

**Windows (winget):**

```powershell
winget install eigen
```

**Or download directly** from [eigen.tuxfamily.org](https://eigen.tuxfamily.org) and extract to a known path. Then set the environment variable so CMake can find it:

```powershell
$env:EIGEN3_INCLUDE_DIR = "C:\path\to\eigen-3.4.0"
```

Or add it permanently in System Properties → Environment Variables.

## 3. Configure and Build

All other dependencies (googletest, nlohmann/json, trochoids) are fetched automatically by CMake on first configure.

```bash
# Configure
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build --config Release
```

> **First configure will download dependencies** from GitHub. Ensure internet access is available or pre-populate the CMake fetch cache.

### Debug build with sanitizers

```bash
cmake -B build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build-debug
```

The Debug configuration enables `-fsanitize=address,undefined` on GCC/Clang.

## 4. Run Tests

PATH must include ucrt64/bin when running ctest. Without it, Windows finds the wrong
`libstdc++` DLL and every test crashes with an "Entry Point Not Found" dialog box.

```bash
PATH="/c/msys64/ucrt64/bin:$PATH" ctest --test-dir build --output-on-failure
```

Expected output: all tests pass except the known pre-existing failures documented in [testing/strategy.md](../testing/strategy.md#known-failures).

## 5. VSCode Setup

Install the following extensions:

- **CMake Tools** (`ms-vscode.cmake-tools`) — configure, build, and run CTest from the IDE
- **C/C++** (`ms-vscode.cpptools`) — IntelliSense and debugging

On first open, CMake Tools will prompt to select a kit (compiler). Select your installed compiler.

Tests appear in the **Testing** panel (beaker icon) via CTest.

### Recommended `settings.json` additions

```json
{
    "cmake.buildDirectory": "${workspaceFolder}/build",
    "cmake.configureOnOpen": true,
    "terminal.integrated.defaultProfile.windows": "Git Bash"
}
```

## 6. Python Environment (Optional)

Python tooling uses [uv](https://docs.astral.sh/uv/). Install it if not already present:

```powershell
winget install astral-sh.uv
```

Then, from the `python/` directory:

```bash
cd python

# Install all dependencies (dev group includes JupyterLab and ipykernel)
uv sync --group dev

# One-time: install the custom Jupyter kernel spec
uv run python scripts/install_kernel.py
```

Run Python tests:

```bash
uv run pytest
```

Start JupyterLab:

```bash
uv run jupyter lab
```

### Re-running `install_kernel.py`

The kernel spec lives inside `.venv` and is recreated automatically on `uv sync`.
Re-run `install_kernel.py` only after a **full venv rebuild** — that is, after
deleting `.venv` by hand or changing the Python version in `.python-version`.
Routine `uv sync` (adding or updating packages) does not rebuild the venv and
does not require re-running the script.

## Troubleshooting

| Symptom | Likely Cause | Fix |
| --- | --- | --- |
| `Could not find Eigen3` | Eigen not installed or path not set | Set `EIGEN3_INCLUDE_DIR` environment variable |
| FetchContent download fails | No internet / proxy | Pre-populate fetch cache or use offline mirror |
| `gh` not found in terminal | PATH not updated | Restart VSCode after adding `C:\Program Files\GitHub CLI` to PATH |
| CTest shows "Build failed" | CMakeLists.txt changed, needs reconfigure | CMake Tools → Delete Cache and Reconfigure |
