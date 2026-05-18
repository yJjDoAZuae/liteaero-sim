# ---------------------------------------------------------------------------
# liteaero_sim_py — pybind11 Python extension module
#
# Design authority: docs/architecture/python_bindings.md
#
# Built by default; disable with -DLITEAERO_SIM_BUILD_PYTHON_BINDINGS=OFF.
# pybind11 must be available via Conan (pybind11/2.11.1 in conanfile.txt).
# ---------------------------------------------------------------------------

# Use CMake's built-in Python3 finder before pybind11.
# This is more reliable on Windows/MSYS2 than pybind11's own FindPythonLibsNew.cmake.
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)

pybind11_add_module(liteaero_sim_py
    bindings.cpp
    bind_terrain.cpp
    bind_aircraft.cpp
    bind_runner.cpp
    bind_ring_buffer.cpp
    bind_manual_input.cpp
    bind_broadcaster.cpp
)

target_link_libraries(liteaero_sim_py PRIVATE liteaerosim)

target_include_directories(liteaero_sim_py PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_compile_features(liteaero_sim_py PRIVATE cxx_std_17)

# After each build, copy the .pyd into python/ so that `uv run python` finds
# it automatically (python/ is the working directory for uv invocations and
# files placed there are importable without PYTHONPATH manipulation).
#
# The .pyd target name is derived from the Python3_EXECUTABLE passed at
# configure time.  Pass the uv-managed Python so the suffix matches what
# uv's .venv expects (cp312-win_amd64 on Windows):
#
#   cmake ... -DPython3_EXECUTABLE=<uv-python-path>
#
# libwinpthread-1.dll is a residual dynamic dependency brought in via
# -lmingw32 from the SDL Conan package.  The notebook import cell calls
# os.add_dll_directory('C:/msys64/ucrt64/bin') before importing the module
# to ensure Windows can find it at runtime.
add_custom_command(TARGET liteaero_sim_py POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:liteaero_sim_py>
        ${CMAKE_SOURCE_DIR}/python/$<TARGET_FILE_NAME:liteaero_sim_py>
    COMMENT "Copying liteaero_sim_py to python/"
)
