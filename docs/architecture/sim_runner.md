# SimRunner — Architecture and Interface Design

This document is the design authority for the simulation runner subsystem. It covers the
execution mode model, threading contract, timing mechanism, and the interface between
`SimRunner` and `Aircraft`.

---

## Use Case Decomposition

| ID | Use Case | Primary Actor | Mechanism |
| --- | --- | --- | --- |
| UC-1 | Run a fixed-duration batch simulation at full rate | Test harness, Monte Carlo, CI | `start()` blocks; returns once sim time exceeds `duration_s` |
| UC-2 | Run a real-time simulation paced to wall clock | External interface, SITL | `start()` spawns thread; `stop()` or `duration_s` terminates |
| UC-3 | Run a scaled-real-time simulation | Development / slow-motion replay | Same as UC-2 with `time_scale` applied to sleep target |
| UC-4 | Stop a running simulation from another thread | External interface, test | `stop()` sets atomic flag; loop exits within one timestep |
| UC-5 | Query elapsed simulation time | Logger, status display | `elapsed_sim_time_s()` returns `step_count * dt_s` |
| UC-6 | Query whether the runner is currently active | External interface, test | `is_running()` returns atomic flag |
| UC-7 | Inject a manual input adapter | Scenario setup | `setManualInput(ManualInput*)` before `start()` |
| UC-8 | Read the manual input frame from the most recent tick | Logger, HUD display | `lastManualInputFrame()` returns mutex-protected snapshot |

---

## Class Design

### `ExecutionMode`

```cpp
enum class ExecutionMode { RealTime, ScaledRealTime, Batch };
```

| Value | Behavior |
| --- | --- |
| `Batch` | Steps execute as fast as possible. `start()` blocks the caller until `duration_s` elapses or `stop()` is called. |
| `RealTime` | Each step is paced so that the wall time between step starts equals `dt_s`. `start()` spawns a background thread and returns immediately. |
| `ScaledRealTime` | Same as `RealTime` but the wall-time target per step is `dt_s / time_scale`. A `time_scale` of 2.0 runs twice as fast as real time; 0.5 runs at half speed. |

### `RunnerConfig`

```cpp
struct RunnerConfig {
    ExecutionMode mode        = ExecutionMode::Batch;
    float         dt_s        = 0.02f;  // output step, seconds; float is sufficient (values ≤ a few seconds)
    float         time_scale  = 1.0f;   // ScaledRealTime only; must be > 0
    double        duration_s  = 0.0;    // 0 = run until stop() is called; double needed because duration_s
                                        // may be large (hours) and is compared directly to accumulated
                                        // sim time — float precision would cause imprecise termination
};
```

`dt_s` is the **output step** — the interval at which SimRunner calls `Aircraft::step()` and
at which the simulation produces observable outputs (state snapshots, log entries, display
frames). It is not the physics integration timestep.

`Aircraft` owns the physics integration timestep independently. At `initialize()` time,
`Aircraft` reads its `max_integration_dt_s` config parameter and the runner's `dt_s`, then
selects an integration substep count N = ⌈runner_dt_s / max_integration_dt_s⌉ such that
`integration_dt_s = runner_dt_s / N ≤ max_integration_dt_s`. Each call to `Aircraft::step()`
executes N full physics substeps of duration `integration_dt_s`.

The runner is therefore responsible for the **wall-clock relationship** (pacing and threading);
`Aircraft` is responsible for **numerical integration accuracy**.

### `SimRunner`

```cpp
class SimRunner {
public:
    void             initialize(const RunnerConfig& config, Aircraft& aircraft);
    void             start();
    void             stop();
    bool             is_running() const;
    double           elapsed_sim_time_s() const;

    // Inject a manual input adapter. nullptr (default) uses a neutral AircraftCommand
    // on every tick.  Must not be called while a run is in progress.
    void             setManualInput(ManualInput* input);

    // Returns a mutex-protected snapshot of the ManualInputFrame produced on the most
    // recent tick.  Returns a neutral frame when no adapter is set or before the first
    // tick.  Safe to call from any thread.
    ManualInputFrame lastManualInputFrame() const;
};
```

`SimRunner` holds non-owning pointers to `Aircraft` and `ManualInput`. The caller is
responsible for keeping both alive for the duration of the run.

---

## Threading Contract

| Mode | `start()` behavior | Thread ownership |
| --- | --- | --- |
| `Batch` | Blocks. Returns when the run completes. | No thread spawned. |
| `RealTime` / `ScaledRealTime` | Returns immediately after spawning a `std::thread`. | `SimRunner` owns the thread; joined by `stop()` or destructor. |

`stop()` sets an `std::atomic<bool>` stop flag and, for threaded modes, joins the worker
thread before returning. Calling `stop()` from the run thread itself (e.g., from within a
callback) is undefined behavior.

`is_running()` and `elapsed_sim_time_s()` are safe to call from any thread. `is_running()`
reads the same atomic flag. `elapsed_sim_time_s()` is derived from
`step_count_.load() * config_.dt_s`, where `step_count_` is `std::atomic<uint64_t>`.

`initialize()` must not be called while a run is in progress.

---

## Duration and Termination Policy

When `duration_s > 0`, the run loop terminates when the simulation time at the start of
the current step would exceed the run window:

$$\text{terminate if} \quad t_{\text{sim}} > t_{\text{initial}} + \text{duration\_s}$$

where $t_{\text{sim}} = k \cdot dt\_s$ is the time argument that *would be* passed to
`Aircraft::step()` on this iteration, and $t_{\text{initial}}$ is the simulation time at
the first step (equal to `step_count * dt_s` at the moment `start()` is called, which is
0 after a fresh `initialize()`).

**Consequence for step count.** Steps execute for all $t_{\text{sim}} \leq t_{\text{initial}} + \text{duration\_s}$. If `duration_s` is an exact multiple of `dt_s`, the final step
executed is the one at $t_{\text{sim}} = t_{\text{initial}} + \text{duration\_s}$, and
`elapsed_sim_time_s()` on return equals $\text{duration\_s} + dt\_s$.

This avoids precomputing a step count — no rounding or integer conversion of
`duration_s / dt_s` is required, and the condition is well-defined for any positive
`dt_s` and `duration_s`.

When `duration_s = 0`, the loop runs indefinitely until `stop()` sets the stop flag.

---

## Timing Mechanism — RealTime and ScaledRealTime

The wall-time target for step $k$ is:

$$t_{\text{wall},k} = t_{\text{start}} + k \cdot \frac{dt\_s}{\text{time\_scale}}$$

where $t_{\text{start}}$ is the `std::chrono::steady_clock` value captured at the first
step.

After each step, the runner computes the remaining sleep duration:

$$\Delta t_{\text{sleep}} = t_{\text{wall},k} - t_{\text{now}}$$

If $\Delta t_{\text{sleep}} > 0$, the runner calls `std::this_thread::sleep_until(t_{\text{wall},k})`.

**Late-step policy.** If a step takes longer than the wall-time budget ($\Delta t_{\text{sleep}} \leq 0$), the runner does not attempt to compensate by skipping steps or shortening future sleep intervals. It simply proceeds immediately to the next step, accepting the accumulated lag. This keeps the simulation time monotonic and the step count exact.

The `Batch` mode has no timing mechanism; `sleep_until` is never called.

---

## Private Members

```cpp
private:
    RunnerConfig            config_;
    Aircraft*               aircraft_     = nullptr;
    ManualInput*            manual_input_ = nullptr;
    std::atomic<bool>       stop_flag_    {false};
    std::atomic<bool>       running_      {false};
    std::atomic<uint64_t>   step_count_   {0};
    std::thread             worker_;

    mutable std::mutex      frame_mutex_;
    ManualInputFrame        last_frame_;   // guarded by frame_mutex_

    void runLoop();
```

`runLoop()` contains the step-advance logic common to all modes and is called either
directly (Batch) or from the worker thread.

---

## Manual Input Integration

When a `ManualInput` adapter is injected via `setManualInput()`, the run loop calls it
each tick before `Aircraft::step()`:

```text
runLoop() each tick:
    if manual_input_ != nullptr:
        SDL_PumpEvents()                    // flush OS event queue; caller owns SDL_Init
        frame = manual_input_->read()
    else:
        frame = { neutral AircraftCommand, actions = 0 }
    { frame_mutex_ lock }
        last_frame_ = frame
    aircraft_->step(sim_time_s, frame.command, environment_state)
```

**SDL ownership.** `SimRunner` calls `SDL_PumpEvents()` but does not call `SDL_Init()`
or `SDL_Quit()`. The scenario code is responsible for the full SDL2 lifecycle before
calling `start()` and after calling `stop()`. See the SDL2 Initialization Contract in
[`manual_input.md`](manual_input.md).

**Echo.** `lastManualInputFrame()` returns the `last_frame_` snapshot under
`frame_mutex_`. This allows the scenario or application layer to read the ingested
`ManualInputFrame` after each step and log it alongside other step outputs, without
requiring `SimRunner` to hold a logger reference. See Open Questions.

---

## `initialize()` Contract

`initialize()` must be called exactly once before `start()`. It:

1. Validates `config_.dt_s > 0`.
2. Validates `config_.time_scale > 0`.
3. Validates `config_.duration_s >= 0`.
4. Stores `config_` and a pointer to `aircraft_`.
5. Resets `step_count_` to 0, `stop_flag_` to false.

Calling `initialize()` on a running instance throws `std::logic_error`.

---

## CMake Integration

```cmake
# src/CMakeLists.txt (addition)
add_library(liteaero_sim_runner STATIC
    runner/SimRunner.cpp
)
add_library(liteaerosim::runner ALIAS liteaero_sim_runner)
target_include_directories(liteaero_sim_runner
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(liteaero_sim_runner PUBLIC cxx_std_17)
target_link_libraries(liteaero_sim_runner
    PUBLIC
        liteaero_sim_aircraft   # provides Aircraft
)
```

`SimRunner` has no dependency on logging, serialization, or flight components.
When a `ManualInput` adapter is injected, `SimRunner` gains a conditional SDL2 dependency
(for `SDL_PumpEvents()`). The `liteaero_sim_runner` CMake target must link `SDL2::SDL2`
when the manual input feature is included.

---

## Test Strategy

File: `test/SimRunner_test.cpp`

| Test | Pass criterion |
| --- | --- |
| `BatchMode_StepCountExact` | `Batch` with `duration_s = 1.0`, `dt_s = 0.02`: `Aircraft::step()` called 51 times (steps at $t = 0, 0.02, \ldots, 1.0$ all execute; termination check fails at $t = 1.02$); `elapsed_sim_time_s()` returns `51 * 0.02` |
| `BatchMode_ZeroDuration_RunsUntilStop` | `duration_s = 0` with `stop()` called from a second thread: run terminates; `elapsed_sim_time_s() > 0` |
| `RealTime_WallTimeAccuracy` | `RealTime` with `dt_s = 0.02`, `duration_s = 10 * 0.02`: wall time elapsed is within [75%, 150%] of 0.2 s |
| `ScaledRealTime_HalfSpeed` | `ScaledRealTime` with `dt_s = 0.02`, `time_scale = 0.5`, `duration_s = 5 * 0.02`: wall time elapsed is within [75%, 150%] of 0.2 s |
| `Stop_TerminatesWithinOneStep` | `RealTime` runner stopped from external thread: `is_running()` returns false within `2 * dt_s` of `stop()` call |
| `ElapsedSimTime_AccumulatesCorrectly` | `Batch` with `duration_s = 7 * 0.02`, `dt_s = 0.02`: `elapsed_sim_time_s()` equals `8 * 0.02` exactly (step at $t = 7 \cdot dt\_s$ executes) |
| `Initialize_InvalidDt_Throws` | `dt_s <= 0` causes `initialize()` to throw |
| `Initialize_InvalidTimeScale_Throws` | `time_scale <= 0` causes `initialize()` to throw |
| `IsRunning_FalseBeforeStart` | `is_running()` is false before `start()` |
| `IsRunning_FalseAfterBatchCompletes` | `is_running()` is false after a `Batch` run completes |

**Manual input tests** (additions to `test/SimRunner_test.cpp`):

| Test | Pass criterion |
| --- | --- |
| `ManualInput_NullAdapter_UsesNeutralCommand` | `setManualInput(nullptr)` — `Aircraft::step()` receives neutral `AircraftCommand` every tick |
| `ManualInput_InjectedAdapter_CommandPropagated` | Injected mock adapter returning a fixed command — `Aircraft::step()` receives that command; `lastManualInputFrame()` reflects it |
| `ManualInput_LastFrame_ThreadSafe` | `lastManualInputFrame()` called from a second thread while `Batch` run executes — no data race (run under ThreadSanitizer) |
| `ManualInput_NotSet_LastFrameIsNeutral` | `lastManualInputFrame()` before `start()` returns neutral frame |

---

## Open Questions

### OQ-SR-1 — ManualInputFrame Echo Delivery

`manual_input.md` requires that the `ManualInputFrame` ingested each tick is available
to Python HUD display tools and post-processing. The current design exposes it via
`lastManualInputFrame()`, which the scenario or application layer can poll after each
step and log alongside other outputs. This keeps `SimRunner` free of a logger
dependency.

The question is whether a polling getter is sufficient or whether an active delivery
mechanism is needed.

| Option | Description | Tradeoff |
| --- | --- | --- |
| Getter (`lastManualInputFrame()`) | Caller reads after each tick. | Simple. Requires caller to poll every step. Suitable for batch scenarios where the caller drives the loop; less clean for real-time scenarios where the run loop is on a background thread. |
| Injected callback | `setFrameCallback(std::function<void(const ManualInputFrame&)>)` called each tick from the run loop. | Active delivery; no polling. Adds callback invocation overhead. Callback runs on the sim thread — caller must be thread-safe. |
| Logger dependency | `SimRunner` accepts a `Logger*` and logs `ManualInputFrame` directly. | Direct, no extra mechanism. Contradicts the design constraint that `SimRunner` has no logging dependency. |

**Recommendation:** Getter for now. The batch and scripted use cases (which are the
near-term use cases) work naturally with polling. If real-time HUD display proves to
need active delivery, the callback option can be added without breaking the getter API.

---

### OQ-SR-2 — SDL2 Initialization Ownership

When `SimRunner` calls `SDL_PumpEvents()` each tick, it assumes SDL has been
initialized by the caller. In scenarios that use neither keyboard nor joystick input,
SDL would never be initialized, and `SDL_PumpEvents()` would be a no-op or could
behave unexpectedly.

| Option | Description | Tradeoff |
| --- | --- | --- |
| Caller retains ownership (current design) | `SimRunner` calls `SDL_PumpEvents()` only when `manual_input_ != nullptr`. Caller initializes SDL before `start()`. | No SDL dependency in `SimRunner` when no adapter is set. Consistent with `manual_input.md` SDL contract. |
| `SimRunner` conditionally initializes SDL | When `setManualInput()` is called with a non-null adapter, `SimRunner` calls `SDL_Init(SDL_INIT_JOYSTICK \| SDL_INIT_EVENTS)` in `start()` and `SDL_Quit()` in `stop()`. | Simpler for the caller. `SDL_Quit()` would be unsafe if other SDL subsystems are in use. |

**Recommendation:** Caller retains ownership. Guard `SDL_PumpEvents()` behind
`if (manual_input_ != nullptr)` so the call is never made in batch/scripted scenarios.
This is consistent with `manual_input.md` and avoids `SimRunner` taking responsibility
for a global subsystem.
