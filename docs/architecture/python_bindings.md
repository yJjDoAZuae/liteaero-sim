# Python Bindings — Architecture and Interface Design

This document is the design authority for the pybind11 extension module that exposes
LiteAero Sim C++ types to Python. It covers the module structure, CMake integration,
exported surface, and threading contract.

---

## Purpose

Python scenario scripts, Jupyter notebooks, and test fixtures require direct access to
selected C++ simulation types — primarily to drive simulation inputs from Python and to
enumerate hardware devices. All simulation logic remains in C++; the binding layer
provides access, not reimplementation.

---

## Module

| Property | Value |
| --- | --- |
| Python import name | `liteaero_sim_py` |
| Extension source entry point | `src/python/bindings.cpp` |
| CMake target | `liteaero_sim_py` (pybind11 extension module) |
| Build flag | `LITEAERO_SIM_BUILD_PYTHON_BINDINGS` (default `OFF`) |
| Conan dependency | `pybind11/2.11.1` (or latest stable in ConanCenter) |

The module is an optional build target. It is not built by default and is not required
for C++ unit tests or batch simulation. It must be enabled explicitly when building for
Python integration.

---

## CMake Integration

```cmake
# Root CMakeLists.txt (conditional on LITEAERO_SIM_BUILD_PYTHON_BINDINGS)
option(LITEAERO_SIM_BUILD_PYTHON_BINDINGS "Build pybind11 Python extension module" OFF)

if(LITEAERO_SIM_BUILD_PYTHON_BINDINGS)
    find_package(pybind11 REQUIRED)
    pybind11_add_module(liteaero_sim_py src/python/bindings.cpp)
    target_link_libraries(liteaero_sim_py PRIVATE liteaero-sim)
endif()
```

`pybind11` is declared in `conanfile.txt`. It is a project-wide dependency; it must
appear in `conanfile.txt` and the dependency registry
([`docs/dependencies/README.md`](../dependencies/README.md)) before any binding code
is built.

---

## Source Organization

`src/python/bindings.cpp` is the single entry point: it defines the `PYBIND11_MODULE`
block and calls one registration function per subsystem. Each subsystem's registration
function is declared in a header and defined in a dedicated source file co-located with
the C++ it wraps:

```text
src/python/
    bindings.cpp              — PYBIND11_MODULE entry point; calls bind_*() functions
    py_aircraft_types.hpp     — PyAircraft wrapper struct (shared by bind_aircraft and bind_runner)
    bind_manual_input.cpp     — AircraftCommand, ScriptedInput, JoystickInput
    bind_aircraft.cpp         — KinematicState, Aircraft (via PyAircraft wrapper)
    bind_runner.cpp           — ExecutionMode, RunnerConfig, SimRunner (including channel_registry())
    bind_ring_buffer.cpp      — Sample, ChannelSubscriber, ChannelRegistry
    bind_landing_gear.cpp     — LandingGear, WheelUnit, StrutState, ContactForces
```

Each `bind_*()` function signature follows the pattern:

```cpp
void bind_manual_input(py::module_& m);
void bind_landing_gear(py::module_& m);
```

New subsystems add a new `bind_*.cpp` file and a call in `bindings.cpp`. No existing
files are modified when adding a new subsystem binding.

---

## Exported Surface

### Aircraft and SimRunner

Design authority: [`aircraft.md`](aircraft.md), [`sim_runner.md`](sim_runner.md)

`Aircraft` is exposed through a Python-side `PyAircraft` wrapper (defined in
`py_aircraft_types.hpp`) that owns both the `Propulsion` and the `Aircraft` objects and
tracks simulation time for standalone `step()` calls. Python users never interact with
propulsion types directly — the wrapper selects the propulsion model from the
`"propulsion"` section of the JSON config.

#### `PyAircraft` — Python class name `Aircraft`

| C++ type / function | Python exposure | Purpose |
| --- | --- | --- |
| `KinematicState` | Read-only class with scalar attributes (see table below) | Inspect current simulation state from Python scripts and notebooks |
| `PyAircraft(config, dt_s=0.02)` | Constructor taking a JSON string or file path and an optional timestep | Construct, configure, and initialize the simulation plant. If the config has a `"propulsion"` section with a `"type"` key (`"jet"`, `"edf"`, `"prop"`, `"none"`), the corresponding propulsion model is created and initialized from that section; if the section is absent, a zero-thrust stub is used |
| `Aircraft::reset()` | Instance method | Reset to initial conditions; resets the internal simulation time to zero |
| `Aircraft::step(cmd, dt_s=0.02, rho_kgm3=1.225)` | Instance method | Advance simulation by one timestep; time is tracked internally; wind is assumed zero |
| `Aircraft::state()` | Instance method returning `KinematicState` | Read current kinematic state |

#### `KinematicState` attributes

| Attribute | C++ source | Units |
| --- | --- | --- |
| `time_s` | `time_sec()` | s |
| `latitude_rad` | `positionDatum().latitudeGeodetic_rad()` | rad |
| `longitude_rad` | `positionDatum().longitude_rad()` | rad |
| `altitude_m` | `positionDatum().height_WGS84_m()` | m |
| `velocity_north_mps` | `velocity_NED_mps()(0)` | m/s |
| `velocity_east_mps` | `velocity_NED_mps()(1)` | m/s |
| `velocity_down_mps` | `velocity_NED_mps()(2)` | m/s |
| `heading_rad` | `heading()` | rad |
| `pitch_rad` | `pitch()` | rad |
| `roll_rad` | `roll()` | rad |
| `alpha_rad` | `alpha()` | rad |
| `beta_rad` | `beta()` | rad |
| `airspeed_m_s` | `velocity_Wind_mps()(0)` — wind-axis forward speed | m/s |
| `roll_rate_rad_s` | `rollRate_Wind_rps()` | rad/s |

#### `RunnerConfig` and `SimRunner`

| C++ type / function | Python exposure | Purpose |
| --- | --- | --- |
| `ExecutionMode` | `ExecutionMode.BATCH`, `ExecutionMode.REAL_TIME`, `ExecutionMode.SCALED_REAL_TIME` | Execution mode selector |
| `RunnerConfig(dt_s, duration_s, time_scale, mode)` | Constructor; `mode` is a string: `"batch"`, `"realtime"`, `"scaled_realtime"` | Configure `SimRunner` from Python |
| `SimRunner::initialize(config, aircraft)` | Instance method; `aircraft` is a Python `Aircraft` object | Wire the aircraft and config; `aircraft` is kept alive by the runner |
| `SimRunner::start()` | Instance method; releases the GIL | Start the run loop; blocks in Batch mode; returns immediately in RealTime and ScaledRealTime |
| `SimRunner::stop()` | Instance method; releases the GIL | Request termination; blocks until the run loop exits |
| `SimRunner::is_running()` | Instance method | `True` while the run loop is active |
| `SimRunner::elapsed_sim_time_s()` | Instance method | Simulation time elapsed since `start()` |
| `SimRunner::channel_registry()` | Instance method returning `ChannelRegistry&` (`reference_internal`) | Access the channel registry; reference is valid for the lifetime of the `SimRunner` |

### Ring Buffer and Channel Registry

Design authority: [`ring_buffer.md`](ring_buffer.md)

The channel registry and subscriber API allow Python code to poll live simulation output
without coupling to the run loop. `SimRunner` registers 14 kinematic channels at
initialization and publishes to them after each `Aircraft::step()`.

#### `Sample`

| Attribute | Type | Description |
| --- | --- | --- |
| `time_s` | `float` | Simulation time at which this sample was produced (s) |
| `value` | `float` | Channel value in SI units |

`Sample` supports tuple unpacking (`time_s, value = sample`) and `repr()`.

#### `ChannelSubscriber`

| Method / property | Description |
| --- | --- |
| `channel_name` | Name of the subscribed channel |
| `drain()` | Return all buffered samples as `list[tuple[float, float]]` and reset the buffer; returns `[]` if no new data |

Subscribers are obtained via `ChannelRegistry.subscribe()`. Each subscriber maintains an
independent circular buffer (capacity = `ceil(sample_rate_hz × depth_s)`, minimum 2);
overflow silently drops the oldest sample. A late-joining subscriber starts with an empty
buffer — there is no backfill (PP-F37). Destroying a subscriber automatically unregisters
it from the channel.

#### `ChannelRegistry`

| Method | Description |
| --- | --- |
| `available_channels()` | Return a list of all registered channel names |
| `subscribe(name)` | Subscribe to a registered channel; raises `KeyError` if the channel has not been registered |

#### Kinematic channels registered by `SimRunner`

All channels are registered at `1/dt_s` Hz with a 60 s buffer depth.

| Channel name | Source field | Units |
| --- | --- | --- |
| `kinematic/time_s` | `time_sec()` | s |
| `kinematic/latitude_rad` | `positionDatum().latitudeGeodetic_rad()` | rad |
| `kinematic/longitude_rad` | `positionDatum().longitude_rad()` | rad |
| `kinematic/altitude_m` | `positionDatum().height_WGS84_m()` | m |
| `kinematic/velocity_north_mps` | `velocity_NED_mps()(0)` | m/s |
| `kinematic/velocity_east_mps` | `velocity_NED_mps()(1)` | m/s |
| `kinematic/velocity_down_mps` | `velocity_NED_mps()(2)` | m/s |
| `kinematic/heading_rad` | `heading()` | rad |
| `kinematic/pitch_rad` | `pitch()` | rad |
| `kinematic/roll_rad` | `roll()` | rad |
| `kinematic/alpha_rad` | `alpha()` | rad |
| `kinematic/beta_rad` | `beta()` | rad |
| `kinematic/airspeed_m_s` | `velocity_Wind_mps()(0)` | m/s |
| `kinematic/roll_rate_rad_s` | `rollRate_Wind_rps()` | rad/s |

### Manual Input

Design authority: [`manual_input.md`](manual_input.md)

| C++ type / function | Python exposure | Purpose |
| --- | --- | --- |
| `AircraftCommand` | Class with named constructor arguments (`n_z`, `n_y`, `roll_rate_wind_rps`, `throttle_nd`) and attribute access | Passed to `ScriptedInput.push()` from Python scenario scripts |
| `ScriptedInput::push(const AircraftCommand&)` | Instance method | Drive simulation inputs from Python |
| `JoystickInput::enumerateDevices()` | Static method returning a list of dicts (`device_index`, `name`, `num_axes`) | Enumerate connected joystick devices from Python setup code or notebooks |

### Landing Gear

Design authority: [`landing_gear.md`](landing_gear.md)

| C++ type / function | Python exposure | Purpose |
| --- | --- | --- |
| `LandingGear` | Class with `initialize()`, `reset()`, `step()` | Call contact physics from Python scenario and animation scripts |
| `WheelUnit` | Class | Per-wheel state access |
| `StrutState` | Class | Strut deflection state |
| `ContactForces` | Class | Ground contact force output |

---

## Threading Contract

pybind11 releases the GIL when calling into C++ functions that do not touch Python
objects. The following rules apply to all binding code in this module:

- `ScriptedInput::push()` acquires `mutex_` internally (C++ mutex, not the GIL). It
  may be called from the Python thread while `SimRunner` calls `read()` from the
  simulation thread. No additional locking is required in the binding.
- `JoystickInput::enumerateDevices()` calls SDL2 functions. SDL2 must have been
  initialized (`SDL_Init`) before this call. Callers are responsible for SDL lifecycle.
- `SimRunner::start()` and `SimRunner::stop()` release the GIL. `start()` in Batch mode
  blocks the caller; the GIL release allows other Python threads to run while the
  simulation executes. In RealTime mode `start()` returns immediately after spawning the
  worker thread.
- `Aircraft::step()` is not thread-safe with respect to a concurrent `SimRunner` run. Do
  not call `aircraft.step()` from Python while a `SimRunner` run is in progress with the
  same `Aircraft` instance.
- `LandingGear::step()` is not thread-safe with respect to concurrent Python access.
  Callers must not call it from Python while a `SimRunner` run is in progress with the
  same `LandingGear` instance.
- `ChannelSubscriber::drain()` acquires only the subscriber mutex (not the registry
  mutex). It is safe to call from any Python thread concurrently with `SimRunner`
  publishing from the simulation thread. Lock ordering is registry → subscriber; `drain()`
  only ever takes the subscriber lock, so no deadlock is possible.

---

## Open Questions

None.
