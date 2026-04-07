// liteaero_sim_py — pybind11 extension module entry point.
//
// Design authority: docs/architecture/python_bindings.md
//
// Each subsystem's exported symbols are registered by a dedicated bind_*()
// function defined in a co-located source file.  Add new subsystems by
// declaring and calling a new bind_*() function here.

#include <pybind11/pybind11.h>

namespace py = pybind11;

void bind_aircraft(py::module_& m);
void bind_runner(py::module_& m);
void bind_ring_buffer(py::module_& m);
void bind_manual_input(py::module_& m);
void bind_broadcaster(py::module_& m);

PYBIND11_MODULE(liteaero_sim_py, m)
{
    m.doc() = "LiteAero Sim Python bindings";
    // Aircraft and KinematicState must be registered before runner and
    // manual input so that argument-type matching works correctly.
    // bind_ring_buffer adds channel_registry() to the already-registered
    // SimRunner type, so it must run after bind_runner.
    bind_aircraft(m);
    bind_runner(m);
    bind_ring_buffer(m);
    bind_manual_input(m);
    bind_broadcaster(m);
}
