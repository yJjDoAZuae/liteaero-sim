#pragma once

#include "Aircraft.hpp"
#include "broadcaster/ISimulationBroadcaster.hpp"
#include "input/ManualInput.hpp"
#include "runner/ChannelRegistry.hpp"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace liteaero::simulation {

enum class ExecutionMode { RealTime, ScaledRealTime, Batch };

struct RunnerConfig {
    ExecutionMode mode       = ExecutionMode::Batch;
    float         dt_s       = 0.02f;
    float         time_scale = 1.0f;
    double        duration_s = 0.0;
};

class SimRunner {
public:
    ~SimRunner();

    void   initialize(const RunnerConfig& config, Aircraft& aircraft);
    void   start();
    void   stop();
    bool   is_running() const;
    double elapsed_sim_time_s() const;

    // Inject a manual input adapter.  nullptr (default) uses a neutral
    // AircraftCommand on every tick.  Must not be called while a run is in
    // progress.  SimRunner does not take ownership; the caller must keep the
    // adapter alive for the duration of the run.
    void setManualInput(ManualInput* input);

    // Inject a simulation broadcaster.  nullptr (default) disables broadcasting.
    // Must not be called while a run is in progress.  SimRunner does not take
    // ownership; the broadcaster must outlive the run.
    void set_broadcaster(ISimulationBroadcaster* broadcaster);

    // Returns a mutex-protected snapshot of the ManualInputFrame produced on
    // the most recent tick.  Returns a neutral frame when no adapter is set or
    // before the first tick.  Safe to call from any thread.
    ManualInputFrame lastManualInputFrame() const;

    // Access the channel registry.  Subscribers attach here to receive real-time
    // simulation output.  The registry is valid for the lifetime of this SimRunner.
    ChannelRegistry&       channel_registry()       { return registry_; }
    const ChannelRegistry& channel_registry() const { return registry_; }

private:
    void runLoop();

    RunnerConfig               config_;
    Aircraft*                  aircraft_     = nullptr;
    ManualInput*               manual_input_ = nullptr;
    ISimulationBroadcaster*    broadcaster_  = nullptr;
    std::atomic<bool>     stop_flag_    {false};
    std::atomic<bool>     running_      {false};
    std::atomic<uint64_t> step_count_   {0};
    std::thread           worker_;

    mutable std::mutex    frame_mutex_;
    ManualInputFrame      last_frame_;   // guarded by frame_mutex_

    ChannelRegistry       registry_;
};

}  // namespace liteaero::simulation
