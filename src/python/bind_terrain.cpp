// Terrain bindings — exposes FlatTerrain to Python.
//
// Design authority: docs/architecture/python_bindings.md
//
// FlatTerrain is the minimal Terrain implementation needed for offline validation
// tests (e.g. the landing-gear / terrain-collision validation notebook).  TerrainMesh
// is not bound here; it is loaded at runtime by live_sim.exe.

#include <pybind11/pybind11.h>
#include <liteaero/terrain/Terrain.hpp>

namespace py = pybind11;

void bind_terrain(py::module_& m)
{
    // Abstract base — registered so pybind11 can match subclass arguments.
    py::class_<liteaero::terrain::Terrain>(m, "Terrain");

    py::class_<liteaero::terrain::FlatTerrain, liteaero::terrain::Terrain>(m, "FlatTerrain")
        .def(py::init<float>(),
             py::arg("elevation_m") = 0.f,
             "Flat terrain at a constant WGS84 ellipsoidal elevation (m).")
        .def("elevation_m",
             [](const liteaero::terrain::FlatTerrain& t,
                double latitude_rad, double longitude_rad) {
                 return t.elevation_m(latitude_rad, longitude_rad);
             },
             py::arg("latitude_rad"), py::arg("longitude_rad"),
             "Return the terrain elevation (m) at the given geodetic position.  "
             "Always returns the constant elevation set at construction.");
}
