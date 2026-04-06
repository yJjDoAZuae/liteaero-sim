# ---------------------------------------------------------------------------
# liteaero_sim_py — pybind11 Python extension module
#
# Design authority: docs/architecture/python_bindings.md
#
# Built only when LITEAERO_SIM_BUILD_PYTHON_BINDINGS=ON.
# pybind11 must be available via Conan (pybind11/2.11.1 in conanfile.txt).
# ---------------------------------------------------------------------------

# Use CMake's built-in Python3 finder before pybind11.
# This is more reliable on Windows/MSYS2 than pybind11's own FindPythonLibsNew.cmake.
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)

pybind11_add_module(liteaero_sim_py
    bindings.cpp
    bind_aircraft.cpp
    bind_runner.cpp
    bind_ring_buffer.cpp
    bind_manual_input.cpp
)

target_link_libraries(liteaero_sim_py PRIVATE liteaerosim)

target_include_directories(liteaero_sim_py PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_compile_features(liteaero_sim_py PRIVATE cxx_std_17)
