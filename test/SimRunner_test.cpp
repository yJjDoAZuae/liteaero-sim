// Tests for SimRunner — roadmap item 1.
// Design authority: docs/architecture/sim_runner.md

#include "runner/SimRunner.hpp"
#include "Aircraft.hpp"
#include "input/ManualInput.hpp"
#include "propulsion/Propulsion.hpp"
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

// ---------------------------------------------------------------------------
// StubPropulsion — constant-thrust, stateless test double
// ---------------------------------------------------------------------------

class StubPropulsion : public liteaero::simulation::Propulsion {
public:
    explicit StubPropulsion(float thrust_n = 989.0f) : thrust_(thrust_n) {}

    [[nodiscard]] float step(float, float, float) override { return thrust_; }
    [[nodiscard]] float thrust_n() const override { return thrust_; }

    [[nodiscard]] std::vector<uint8_t> serializeProto()                              const override { return {}; }
    void                               deserializeProto(const std::vector<uint8_t>&)        override {}

protected:
    void           onInitialize(const nlohmann::json&)              override {}
    void           onReset()                                        override {}
    nlohmann::json onSerializeJson()                          const override { return {}; }
    void           onDeserializeJson(const nlohmann::json&)         override {}
    int            schemaVersion()                            const override { return 1; }
    const char*    typeName()                                 const override { return "StubPropulsion"; }

private:
    float thrust_;
};

// ---------------------------------------------------------------------------
// Test helpers
// ---------------------------------------------------------------------------

static nlohmann::json makeAircraftConfig(float dt_s)
{
    auto cfg = nlohmann::json::parse(R"({
        "schema_version": 1,
        "aircraft": {
            "S_ref_m2": 16.2,
            "cl_y_beta": -0.60,
            "ar": 7.47,
            "e": 0.80,
            "cd0": 0.027,
            "cmd_filter_substeps": 1,
            "nz_wn_rad_s": 10.0,
            "nz_zeta_nd": 0.7,
            "ny_wn_rad_s": 10.0,
            "ny_zeta_nd": 0.7,
            "roll_rate_wn_rad_s": 20.0,
            "roll_rate_zeta_nd": 0.7,
            "dtheta_zeta_nd": 0.7,
            "dtheta_wn_pitch_ratio": 7.34,
            "dtheta_wn_roll_ratio": 9.79,
            "dtheta_wn_yaw_ratio": 4.89,
            "dtheta_vref_ratio": 1.0,
            "att_filt_tau_ratio": 1.22,
            "nz_relax_wn_ratio": 2.45,
            "nz_relax_zeta_nd": 0.8,
            "settle_gain_nd": 25.0,
            "settle_clip_nd": 1.0,
            "settle_tau_ratio": 0.52,
            "settle_wheel_rr_nd": 0.05,
            "settle_vland_ratio": 1.0,
            "settle_vtakeoff_ratio": 1.15,
            "settle_vwidth_ratio": 0.5
        },
        "airframe": {
            "g_max_nd":    3.8,
            "g_min_nd":   -1.52,
            "tas_max_mps": 82.3,
            "mach_max_nd": 0.25
        },
        "load_factor_allocator": {
            "alpha_dot_max_ratio": 0.0
        },
        "inertia": {
            "mass_kg":   1045.0,
            "Ixx_kgm2":  1285.0,
            "Iyy_kgm2":  1825.0,
            "Izz_kgm2":  2667.0
        },
        "lift_curve": {
            "cl_alpha":              5.1,
            "cl_max":                1.80,
            "cl_min":               -1.20,
            "delta_alpha_stall":     0.262,
            "delta_alpha_stall_neg": 0.262,
            "cl_sep":                1.05,
            "cl_sep_neg":           -0.80,
            "alpha_max_rad":         0.42,
            "alpha_min_rad":        -0.26
        },
        "initial_state": {
            "latitude_rad":        0.0,
            "longitude_rad":       0.0,
            "altitude_m":        300.0,
            "velocity_north_mps": 55.0,
            "velocity_east_mps":   0.0,
            "velocity_down_mps":   0.0,
            "wind_north_mps":      0.0,
            "wind_east_mps":       0.0,
            "wind_down_mps":       0.0
        }
    })");
    return cfg;
}

static std::unique_ptr<liteaero::simulation::Aircraft> makeAircraft(float dt_s = 0.02f)
{
    auto ac = std::make_unique<liteaero::simulation::Aircraft>(
        std::make_unique<StubPropulsion>());
    ac->initialize(makeAircraftConfig(dt_s), dt_s);
    return ac;
}

using liteaero::simulation::ExecutionMode;
using liteaero::simulation::RunnerConfig;
using liteaero::simulation::SimRunner;

// ---------------------------------------------------------------------------
// Batch mode — step count
// ---------------------------------------------------------------------------

TEST(SimRunnerTest, BatchMode_StepCountExact)
{
    auto ac = makeAircraft(0.02f);
    SimRunner runner;

    RunnerConfig cfg;
    cfg.mode       = ExecutionMode::Batch;
    cfg.dt_s       = 0.02f;
    cfg.duration_s = 1.0;
    runner.initialize(cfg, *ac);
    runner.start();  // blocks

    // 51 steps: condition sim_time_s > 1.0 first fails at t=1.0 (step executes), then
    // fires at t=1.02; EXPECT_NEAR because dt_s is float.
    EXPECT_NEAR(runner.elapsed_sim_time_s(), 51 * 0.02, 1e-4);
    EXPECT_FALSE(runner.is_running());
}

// ---------------------------------------------------------------------------
// Batch mode — zero duration, stop from external thread
// ---------------------------------------------------------------------------

TEST(SimRunnerTest, BatchMode_ZeroDuration_RunsUntilStop)
{
    auto ac = makeAircraft(0.02f);
    SimRunner runner;

    RunnerConfig cfg;
    cfg.mode       = ExecutionMode::Batch;
    cfg.dt_s       = 0.02f;
    cfg.duration_s = 0.0;
    runner.initialize(cfg, *ac);

    std::thread stopper([&runner] {
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
        runner.stop();
    });

    runner.start();  // blocks until stop_flag_ is set
    stopper.join();

    EXPECT_FALSE(runner.is_running());
    EXPECT_GT(runner.elapsed_sim_time_s(), 0.0);
}

// ---------------------------------------------------------------------------
// RealTime mode — wall-clock pacing
// ---------------------------------------------------------------------------

TEST(SimRunnerTest, RealTime_WallTimeAccuracy)
{
    auto ac = makeAircraft(0.02f);
    SimRunner runner;

    RunnerConfig cfg;
    cfg.mode       = ExecutionMode::RealTime;
    cfg.dt_s       = 0.02f;
    cfg.duration_s = 0.02 * 10;  // 10 steps
    runner.initialize(cfg, *ac);

    const auto t0 = std::chrono::steady_clock::now();
    runner.start();
    // wait for background thread to complete
    while (runner.is_running()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const double elapsed_wall_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    const double expected_s = 10 * 0.02;
    EXPECT_GT(elapsed_wall_s, expected_s * 0.75);
    EXPECT_LT(elapsed_wall_s, expected_s * 1.5);
}

// ---------------------------------------------------------------------------
// ScaledRealTime mode — half speed (time_scale = 0.5)
// ---------------------------------------------------------------------------

TEST(SimRunnerTest, ScaledRealTime_HalfSpeed)
{
    auto ac = makeAircraft(0.02f);
    SimRunner runner;

    RunnerConfig cfg;
    cfg.mode       = ExecutionMode::ScaledRealTime;
    cfg.dt_s       = 0.02f;
    cfg.time_scale = 0.5f;
    cfg.duration_s = 0.02 * 5;  // 5 steps
    runner.initialize(cfg, *ac);

    const auto t0 = std::chrono::steady_clock::now();
    runner.start();
    while (runner.is_running()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const double elapsed_wall_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    // 5 steps at 0.02s sim time, time_scale=0.5 → wall budget = 0.02/0.5 = 0.04s/step → 0.2s total
    const double expected_s = 5 * 0.02 / 0.5;
    EXPECT_GT(elapsed_wall_s, expected_s * 0.75);
    EXPECT_LT(elapsed_wall_s, expected_s * 1.5);
}

// ---------------------------------------------------------------------------
// Stop terminates the run loop
// ---------------------------------------------------------------------------

TEST(SimRunnerTest, Stop_TerminatesWithinOneStep)
{
    auto ac = makeAircraft(0.02f);
    SimRunner runner;

    RunnerConfig cfg;
    cfg.mode       = ExecutionMode::RealTime;
    cfg.dt_s       = 0.02f;
    cfg.duration_s = 0.0;
    runner.initialize(cfg, *ac);

    runner.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    runner.stop();  // joins worker; running_ is false on return

    EXPECT_FALSE(runner.is_running());
}

// ---------------------------------------------------------------------------
// elapsed_sim_time_s accumulates correctly
// ---------------------------------------------------------------------------

TEST(SimRunnerTest, ElapsedSimTime_AccumulatesCorrectly)
{
    auto ac = makeAircraft(0.02f);
    SimRunner runner;

    RunnerConfig cfg;
    cfg.mode       = ExecutionMode::Batch;
    cfg.dt_s       = 0.02f;
    cfg.duration_s = 0.02 * 7;  // step at t=7*dt_s executes → 8 steps total
    runner.initialize(cfg, *ac);
    runner.start();

    EXPECT_NEAR(runner.elapsed_sim_time_s(), 8 * 0.02, 1e-4);
}

// ---------------------------------------------------------------------------
// initialize() validation
// ---------------------------------------------------------------------------

TEST(SimRunnerTest, Initialize_InvalidDt_Throws)
{
    auto ac = makeAircraft(0.02f);
    SimRunner runner;

    RunnerConfig cfg;
    cfg.dt_s = 0.0f;
    EXPECT_THROW(runner.initialize(cfg, *ac), std::invalid_argument);

    cfg.dt_s = -0.01f;
    EXPECT_THROW(runner.initialize(cfg, *ac), std::invalid_argument);
}

TEST(SimRunnerTest, Initialize_InvalidTimeScale_Throws)
{
    auto ac = makeAircraft(0.02f);
    SimRunner runner;

    RunnerConfig cfg;
    cfg.dt_s       = 0.02f;
    cfg.time_scale = 0.0f;
    EXPECT_THROW(runner.initialize(cfg, *ac), std::invalid_argument);

    cfg.time_scale = -1.0f;
    EXPECT_THROW(runner.initialize(cfg, *ac), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// is_running() state transitions
// ---------------------------------------------------------------------------

TEST(SimRunnerTest, IsRunning_FalseBeforeStart)
{
    auto ac = makeAircraft(0.02f);
    SimRunner runner;

    RunnerConfig cfg;
    cfg.mode       = ExecutionMode::Batch;
    cfg.dt_s       = 0.02f;
    cfg.duration_s = 0.1;
    runner.initialize(cfg, *ac);

    EXPECT_FALSE(runner.is_running());
}

TEST(SimRunnerTest, IsRunning_FalseAfterBatchCompletes)
{
    auto ac = makeAircraft(0.02f);
    SimRunner runner;

    RunnerConfig cfg;
    cfg.mode       = ExecutionMode::Batch;
    cfg.dt_s       = 0.02f;
    cfg.duration_s = 0.1;
    runner.initialize(cfg, *ac);
    runner.start();

    EXPECT_FALSE(runner.is_running());
}

// ---------------------------------------------------------------------------
// Manual input tests
// ---------------------------------------------------------------------------

// Mock ManualInput: returns a fixed frame; counts how many times read() was called.
class MockManualInput : public liteaero::simulation::ManualInput {
public:
    explicit MockManualInput(liteaero::simulation::AircraftCommand cmd) : cmd_(cmd) {}

    void initialize(const nlohmann::json&) override {}
    void reset() override { read_count_.store(0); }

    liteaero::simulation::ManualInputFrame read() override {
        ++read_count_;
        liteaero::simulation::ManualInputFrame f;
        f.command = cmd_;
        f.actions = 0u;
        return f;
    }

    int readCount() const { return read_count_.load(); }

private:
    liteaero::simulation::AircraftCommand cmd_;
    std::atomic<int> read_count_{0};
};

TEST(SimRunnerTest, ManualInput_NotSet_LastFrameIsNeutral)
{
    auto ac = makeAircraft(0.02f);
    SimRunner runner;

    RunnerConfig cfg;
    cfg.mode       = ExecutionMode::Batch;
    cfg.dt_s       = 0.02f;
    cfg.duration_s = 0.1;
    runner.initialize(cfg, *ac);

    // Before start(): last frame must be neutral.
    const auto frame = runner.lastManualInputFrame();
    EXPECT_FLOAT_EQ(frame.command.n_z,               1.0f);
    EXPECT_FLOAT_EQ(frame.command.n_y,               0.0f);
    EXPECT_FLOAT_EQ(frame.command.rollRate_Wind_rps, 0.0f);
    EXPECT_FLOAT_EQ(frame.command.throttle_nd,       0.0f);
    EXPECT_EQ(frame.actions, 0u);
}

TEST(SimRunnerTest, ManualInput_NullAdapter_UsesNeutralCommand)
{
    auto ac = makeAircraft(0.02f);
    SimRunner runner;

    RunnerConfig cfg;
    cfg.mode       = ExecutionMode::Batch;
    cfg.dt_s       = 0.02f;
    cfg.duration_s = 3 * 0.02;
    runner.initialize(cfg, *ac);
    runner.setManualInput(nullptr);
    runner.start();

    // With null adapter, last_frame_ must contain the neutral AircraftCommand.
    const auto frame = runner.lastManualInputFrame();
    EXPECT_FLOAT_EQ(frame.command.n_z,               1.0f);
    EXPECT_FLOAT_EQ(frame.command.n_y,               0.0f);
    EXPECT_FLOAT_EQ(frame.command.rollRate_Wind_rps, 0.0f);
    EXPECT_FLOAT_EQ(frame.command.throttle_nd,       0.0f);
}

TEST(SimRunnerTest, ManualInput_InjectedAdapter_CommandPropagated)
{
    auto ac = makeAircraft(0.02f);
    SimRunner runner;

    liteaero::simulation::AircraftCommand injected;
    injected.n_z               = 2.0f;
    injected.n_y               = 0.5f;
    injected.rollRate_Wind_rps = 0.3f;
    injected.throttle_nd       = 0.7f;
    MockManualInput mock(injected);

    RunnerConfig cfg;
    cfg.mode       = ExecutionMode::Batch;
    cfg.dt_s       = 0.02f;
    cfg.duration_s = 5 * 0.02;
    runner.initialize(cfg, *ac);
    runner.setManualInput(&mock);
    runner.start();

    // Adapter read() must have been called once per step.
    EXPECT_EQ(mock.readCount(), 6);  // steps at t=0,0.02,...,0.10 (6 steps)

    // lastManualInputFrame() must reflect the injected command.
    const auto frame = runner.lastManualInputFrame();
    EXPECT_FLOAT_EQ(frame.command.n_z,               2.0f);
    EXPECT_FLOAT_EQ(frame.command.n_y,               0.5f);
    EXPECT_FLOAT_EQ(frame.command.rollRate_Wind_rps, 0.3f);
    EXPECT_FLOAT_EQ(frame.command.throttle_nd,       0.7f);
}

TEST(SimRunnerTest, ManualInput_LastFrame_ThreadSafe)
{
    // Run under ThreadSanitizer to verify absence of data races.
    // Correctness check: reader never observes a torn (partially updated) frame.
    auto ac = makeAircraft(0.02f);
    SimRunner runner;

    liteaero::simulation::AircraftCommand injected;
    injected.n_z = 1.5f;
    MockManualInput mock(injected);

    RunnerConfig cfg;
    cfg.mode       = ExecutionMode::Batch;
    cfg.dt_s       = 0.001f;   // fast steps to maximize contention
    cfg.duration_s = 0.5;
    runner.initialize(cfg, *ac);
    runner.setManualInput(&mock);

    std::atomic<bool> done{false};
    std::atomic<int>  errors{0};

    // Reader thread: call lastManualInputFrame() while the Batch run executes.
    std::thread reader([&]() {
        while (!done.load()) {
            const auto f = runner.lastManualInputFrame();
            // n_z must always equal the injected value or the initial neutral 1.0.
            if (f.command.n_z != 1.5f && f.command.n_z != 1.0f) {
                ++errors;
            }
        }
    });

    runner.start();   // blocks until Batch completes
    done.store(true);
    reader.join();

    EXPECT_EQ(errors.load(), 0);
}
