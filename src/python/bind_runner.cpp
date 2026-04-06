// SimRunner and RunnerConfig bindings.
//
// Design authority: docs/architecture/python_bindings.md
//
// SimRunner is exposed directly.  The initialize() binding accepts a PyAircraft
// wrapper (the Python "Aircraft" class) and extracts the inner Aircraft& reference.
// py::keep_alive<1, 3> ensures the Python Aircraft object is not garbage-collected
// while the SimRunner instance is alive.

#include <pybind11/pybind11.h>

#include "py_aircraft_types.hpp"
#include "runner/SimRunner.hpp"

namespace py = pybind11;
using namespace liteaero::simulation;

void bind_runner(py::module_& m)
{
    // --- ExecutionMode ---
    py::enum_<ExecutionMode>(m, "ExecutionMode")
        .value("BATCH",            ExecutionMode::Batch)
        .value("REAL_TIME",        ExecutionMode::RealTime)
        .value("SCALED_REAL_TIME", ExecutionMode::ScaledRealTime)
        .export_values();

    // --- RunnerConfig ---
    // mode is accepted as a string so Python callers do not need to import the enum.
    py::class_<RunnerConfig>(m, "RunnerConfig")
        .def(py::init([](float dt_s, double duration_s, float time_scale,
                         const std::string& mode_str) {
                RunnerConfig cfg;
                cfg.dt_s       = dt_s;
                cfg.duration_s = duration_s;
                cfg.time_scale = time_scale;
                if (mode_str == "batch" || mode_str == "BATCH")
                    cfg.mode = ExecutionMode::Batch;
                else if (mode_str == "realtime" || mode_str == "REAL_TIME")
                    cfg.mode = ExecutionMode::RealTime;
                else if (mode_str == "scaled_realtime" || mode_str == "SCALED_REAL_TIME")
                    cfg.mode = ExecutionMode::ScaledRealTime;
                else
                    throw py::value_error(
                        "RunnerConfig: mode must be \"batch\", \"realtime\", or "
                        "\"scaled_realtime\"; got \"" + mode_str + "\"");
                return cfg;
             }),
             py::arg("dt_s")       = 0.02f,
             py::arg("duration_s") = 0.0,
             py::arg("time_scale") = 1.0f,
             py::arg("mode")       = "batch")
        .def_readwrite("dt_s",       &RunnerConfig::dt_s)
        .def_readwrite("duration_s", &RunnerConfig::duration_s)
        .def_readwrite("time_scale", &RunnerConfig::time_scale);

    // --- SimRunner ---
    py::class_<SimRunner>(m, "SimRunner")
        .def(py::init<>())
        .def("initialize",
             [](SimRunner& self, const RunnerConfig& config, PyAircraft& aircraft) {
                 self.initialize(config, *aircraft.aircraft);
             },
             py::arg("config"),
             py::arg("aircraft"),
             // Keep the Python Aircraft object alive for the lifetime of this SimRunner.
             // Nurse = self (index 1), Patient = aircraft (index 3).
             py::keep_alive<1, 3>(),
             "Wire the aircraft and runner config.  The aircraft object is kept alive "
             "for the lifetime of this SimRunner.")
        .def("start",
             &SimRunner::start,
             py::call_guard<py::gil_scoped_release>(),
             "Start the run loop.  Blocks in Batch mode; returns immediately in "
             "RealTime and ScaledRealTime.")
        .def("stop",
             &SimRunner::stop,
             py::call_guard<py::gil_scoped_release>(),
             "Request termination and block until the run loop exits.")
        .def("is_running",
             &SimRunner::is_running,
             "True while the run loop is active.")
        .def("elapsed_sim_time_s",
             &SimRunner::elapsed_sim_time_s,
             "Simulation time elapsed since start() (seconds).");
}
