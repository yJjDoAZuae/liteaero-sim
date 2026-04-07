// UdpSimulationBroadcaster Python binding.
//
// Design authority: docs/architecture/live_sim_view.md §Python SimSession Design
//
// Exposes UdpSimulationBroadcaster to Python so SimSession can construct and
// wire a broadcaster into SimRunner.  ISimulationBroadcaster is not exposed
// directly — Python callers always use the concrete UDP implementation.

#include <pybind11/pybind11.h>

#include "broadcaster/UdpSimulationBroadcaster.hpp"

namespace py = pybind11;
using namespace liteaero::simulation;

void bind_broadcaster(py::module_& m)
{
    py::class_<UdpSimulationBroadcaster>(m, "UdpSimulationBroadcaster")
        .def(py::init<uint16_t>(),
             py::arg("port") = 14560,
             "Construct a UDP broadcaster that sends SimulationFrame datagrams "
             "to 127.0.0.1:<port> (default 14560).");
}
