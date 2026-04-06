// Ring buffer and channel registry bindings.
//
// Design authority: docs/architecture/ring_buffer.md
//
// Exposed classes:
//   Sample            — (time_s, value) namedtuple-like
//   ChannelSubscriber — drain() returns list[tuple[float,float]]
//   ChannelRegistry   — available_channels(), subscribe()
//
// SimRunner exposes its registry via runner.channel_registry(), added in
// bind_runner.cpp to avoid double-registration of the SimRunner type.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "runner/ChannelRegistry.hpp"

namespace py = pybind11;
using namespace liteaero::simulation;

void bind_ring_buffer(py::module_& m)
{
    // --- Sample ---
    py::class_<Sample>(m, "Sample")
        .def_readonly("time_s", &Sample::time_s,
                      "Simulation time at which this sample was produced (s)")
        .def_readonly("value",  &Sample::value,
                      "Channel value in SI units")
        .def("__repr__", [](const Sample& s) {
            return "Sample(time_s=" + std::to_string(s.time_s)
                 + ", value="       + std::to_string(s.value) + ")";
        })
        .def("__iter__", [](const Sample& s) {
            // Allow tuple unpacking: time_s, value = sample
            return py::iter(py::make_tuple(s.time_s, s.value));
        });

    // --- ChannelSubscriber ---
    // Held as shared_ptr; non-copyable.
    py::class_<ChannelSubscriber, std::shared_ptr<ChannelSubscriber>>(
            m, "ChannelSubscriber")
        .def_property_readonly("channel_name", &ChannelSubscriber::channel_name,
                               "Name of the subscribed channel")
        .def("drain",
             [](ChannelSubscriber& self) {
                 std::vector<Sample> samples = self.drain();
                 // Return as list of (time_s, value) tuples for easy numpy conversion:
                 //   np.array(sub.drain())  → shape (N, 2)
                 py::list result;
                 for (const auto& s : samples) {
                     result.append(py::make_tuple(s.time_s, static_cast<double>(s.value)));
                 }
                 return result;
             },
             "Return all buffered samples as a list of (time_s, value) tuples "
             "and reset the buffer.  Returns an empty list if no new data has arrived.");

    // --- ChannelRegistry ---
    py::class_<ChannelRegistry>(m, "ChannelRegistry")
        .def("available_channels",
             &ChannelRegistry::available_channels,
             "Return a list of all registered channel names.")
        .def("subscribe",
             &ChannelRegistry::subscribe,
             py::arg("name"),
             "Subscribe to a registered channel.  Returns a ChannelSubscriber "
             "whose buffer starts empty (no backfill).  "
             "Raises KeyError if the channel has not been registered.");
}
