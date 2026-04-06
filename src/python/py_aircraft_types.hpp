// py_aircraft_types.hpp — PyAircraft wrapper struct shared by bind_aircraft.cpp
// and bind_runner.cpp.
//
// Design authority: docs/architecture/python_bindings.md
#pragma once

#include "Aircraft.hpp"
#include <memory>

namespace liteaero::simulation {

// PyAircraft — Python-visible Aircraft wrapper.
//
// Owns the Propulsion and Aircraft objects together (the C++ Aircraft constructor
// requires a unique_ptr<Propulsion>, so they must be kept together).  Tracks
// simulation time so that Python callers can use the simplified step(cmd, dt_s) API
// without managing absolute simulation time themselves.
//
// Exposed to Python as the "Aircraft" class.
struct PyAircraft {
    std::unique_ptr<Aircraft> aircraft;
    double                    time_s     = 0.0;
    float                     outer_dt_s = 0.02f;
};

}  // namespace liteaero::simulation
