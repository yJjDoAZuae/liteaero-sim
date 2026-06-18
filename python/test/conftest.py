"""Pytest session configuration for liteaero-sim Python tests."""
import importlib.util
import os
import sys
import sysconfig

import matplotlib

matplotlib.use("Agg")

# On Windows, Python 3.8+ no longer searches PATH for DLL dependencies of
# extension modules.  Register the MSYS2 ucrt64 bin directory so that
# liteaero_sim_py.pyd can find SDL2.dll and the MinGW runtime DLLs.
if sys.platform == "win32":
    _ucrt64_bin = r"C:\msys64\ucrt64\bin"
    if os.path.isdir(_ucrt64_bin):
        os.add_dll_directory(_ucrt64_bin)

# ---------------------------------------------------------------------------
# Never test against a stale binary.
#
# Two independent staleness traps, both of which have silently let the suite
# validate months-old code:
#   1. Wrong copy.  The CMake build copies the .pyd into python/ after every
#      build, but a separate copy may also be installed in the venv's
#      site-packages; a bare `import liteaero_sim_py` resolves the (possibly
#      ancient) site-packages copy.
#   2. Un-rebuilt copy.  Even the python/ build output goes stale the moment
#      C++/proto source is edited without rebuilding.
#
# Defense: load the build-output .pyd by explicit path (defeats trap 1), and
# refuse to run if any compiled source is newer than that .pyd (defeats trap 2).
# A stale binary is a hard error, never a silently-passing run.
# ---------------------------------------------------------------------------


def _newest_source_mtime(roots):
    """Most-recent mtime among C++/proto sources under the given roots."""
    newest_mtime = 0.0
    newest_path = None
    exts = (".cpp", ".cc", ".cxx", ".hpp", ".h", ".proto")
    for root in roots:
        if not os.path.isdir(root):
            continue
        for dirpath, _dirs, files in os.walk(root):
            for name in files:
                if name.endswith(exts):
                    path = os.path.join(dirpath, name)
                    mtime = os.path.getmtime(path)
                    if mtime > newest_mtime:
                        newest_mtime, newest_path = mtime, path
    return newest_mtime, newest_path


_python_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_repo_root = os.path.dirname(_python_dir)
_pyd = os.path.join(_python_dir, "liteaero_sim_py" + sysconfig.get_config_var("EXT_SUFFIX"))

if "liteaero_sim_py" not in sys.modules and os.path.isfile(_pyd):
    # Sources that compile into the extension: this repo plus the co-located
    # liteaero-flight sources linked in via add_subdirectory.
    _src_roots = [os.path.join(_repo_root, d) for d in ("src", "include", "proto")]
    _flight_root = os.path.join(os.path.dirname(_repo_root), "liteaero-flight")
    _src_roots += [os.path.join(_flight_root, d) for d in ("src", "include", "proto")]

    _newest_mtime, _newest_path = _newest_source_mtime(_src_roots)
    if _newest_mtime > os.path.getmtime(_pyd):
        raise RuntimeError(
            "liteaero_sim_py is STALE: source\n"
            f"    {_newest_path}\n"
            "is newer than the built extension\n"
            f"    {_pyd}\n"
            "Refusing to run tests against a stale binary. Rebuild first:\n"
            '    PATH="/c/msys64/ucrt64/bin:$PATH" mingw32-make -C build liteaero_sim_py'
        )

    _spec = importlib.util.spec_from_file_location("liteaero_sim_py", _pyd)
    _mod = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_mod)
    sys.modules["liteaero_sim_py"] = _mod
