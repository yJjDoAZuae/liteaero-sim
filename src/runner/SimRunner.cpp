#include "runner/SimRunner.hpp"

#include <SDL2/SDL.h>
#include <chrono>
#include <stdexcept>

namespace liteaero::simulation {

// ---------------------------------------------------------------------------

SimRunner::~SimRunner()
{
    stop_flag_.store(true);
    if (worker_.joinable()) {
        worker_.join();
    }
}

// ---------------------------------------------------------------------------

void SimRunner::initialize(const RunnerConfig& config, Aircraft& aircraft)
{
    if (running_.load()) {
        throw std::logic_error("SimRunner::initialize called while a run is in progress");
    }
    if (config.dt_s <= 0.0f) {
        throw std::invalid_argument("RunnerConfig::dt_s must be > 0");
    }
    if (config.time_scale <= 0.0f) {
        throw std::invalid_argument("RunnerConfig::time_scale must be > 0");
    }
    if (config.duration_s < 0.0) {
        throw std::invalid_argument("RunnerConfig::duration_s must be >= 0");
    }

    config_    = config;
    aircraft_  = &aircraft;
    stop_flag_.store(false);
    step_count_.store(0);

    // Register all kinematic state channels.  Idempotent — safe to call on
    // re-initialize.  Depth: 60 s of history per subscriber.
    const float sim_hz  = 1.0f / config.dt_s;
    const float depth_s = 60.0f;
    registry_.register_channel("kinematic/time_s",             sim_hz, depth_s);
    registry_.register_channel("kinematic/latitude_rad",        sim_hz, depth_s);
    registry_.register_channel("kinematic/longitude_rad",       sim_hz, depth_s);
    registry_.register_channel("kinematic/altitude_m",          sim_hz, depth_s);
    registry_.register_channel("kinematic/velocity_north_mps",  sim_hz, depth_s);
    registry_.register_channel("kinematic/velocity_east_mps",   sim_hz, depth_s);
    registry_.register_channel("kinematic/velocity_down_mps",   sim_hz, depth_s);
    registry_.register_channel("kinematic/heading_rad",         sim_hz, depth_s);
    registry_.register_channel("kinematic/pitch_rad",           sim_hz, depth_s);
    registry_.register_channel("kinematic/roll_rad",            sim_hz, depth_s);
    registry_.register_channel("kinematic/alpha_rad",           sim_hz, depth_s);
    registry_.register_channel("kinematic/beta_rad",            sim_hz, depth_s);
    registry_.register_channel("kinematic/airspeed_m_s",        sim_hz, depth_s);
    registry_.register_channel("kinematic/roll_rate_rad_s",     sim_hz, depth_s);
}

// ---------------------------------------------------------------------------

void SimRunner::start()
{
    if (config_.mode == ExecutionMode::Batch) {
        running_.store(true);
        runLoop();
        running_.store(false);
    } else {
        running_.store(true);
        worker_ = std::thread([this] {
            runLoop();
            running_.store(false);
        });
    }
}

// ---------------------------------------------------------------------------

void SimRunner::stop()
{
    stop_flag_.store(true);
    if (worker_.joinable()) {
        worker_.join();
    }
}

// ---------------------------------------------------------------------------

bool SimRunner::is_running() const
{
    return running_.load();
}

// ---------------------------------------------------------------------------

double SimRunner::elapsed_sim_time_s() const
{
    return static_cast<double>(step_count_.load()) * static_cast<double>(config_.dt_s);
}

// ---------------------------------------------------------------------------

void SimRunner::setManualInput(ManualInput* input)
{
    manual_input_ = input;
}

// ---------------------------------------------------------------------------

ManualInputFrame SimRunner::lastManualInputFrame() const
{
    std::lock_guard<std::mutex> lock(frame_mutex_);
    return last_frame_;
}

// ---------------------------------------------------------------------------

void SimRunner::runLoop()
{
    const bool   has_duration   = (config_.duration_s > 0.0);
    const double dt_s_d         = static_cast<double>(config_.dt_s);
    const double time_initial_s = static_cast<double>(step_count_.load()) * dt_s_d;

    const bool timed = (config_.mode == ExecutionMode::RealTime ||
                        config_.mode == ExecutionMode::ScaledRealTime);

    using Clock    = std::chrono::steady_clock;
    using Duration = std::chrono::duration<double>;

    const double wall_step_s = dt_s_d / static_cast<double>(config_.time_scale);

    const auto t_start = Clock::now();

    const Eigen::Vector3f wind_NED_mps = Eigen::Vector3f::Zero();
    constexpr float rho_kgm3 = 1.225f;

    while (!stop_flag_.load()) {
        const uint64_t k          = step_count_.load();
        const double   sim_time_s = static_cast<double>(k) * dt_s_d;

        if (has_duration && sim_time_s > config_.duration_s + time_initial_s) {
            break;
        }

        ManualInputFrame frame;
        if (manual_input_ != nullptr) {
            // SDL_PumpEvents flushes the OS event queue so joystick-removed events
            // are visible to JoystickInput::checkDisconnectEvents().
            // Guard behind SDL_WasInit so tests that inject ScriptedInput without
            // initializing SDL do not crash.
            if (SDL_WasInit(0) != 0) {
                SDL_PumpEvents();
            }
            frame = manual_input_->read();
        }
        // frame is neutral (AircraftCommand default + actions=0) when no adapter is set.

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            last_frame_ = frame;
        }

        aircraft_->step(sim_time_s, frame.command, wind_NED_mps, rho_kgm3);
        step_count_.store(k + 1);

        // Publish kinematic state channels.  Each publish() is a no-op when no
        // subscriber is attached for that channel (PP-F34).
        {
            const KinematicState& s = aircraft_->state();
            const double t = s.time_sec();
            registry_.publish("kinematic/time_s",             t, static_cast<float>(t));
            registry_.publish("kinematic/latitude_rad",        t, static_cast<float>(s.positionDatum().latitudeGeodetic_rad()));
            registry_.publish("kinematic/longitude_rad",       t, static_cast<float>(s.positionDatum().longitude_rad()));
            registry_.publish("kinematic/altitude_m",          t, s.positionDatum().height_WGS84_m());
            registry_.publish("kinematic/velocity_north_mps",  t, s.velocity_NED_mps()(0));
            registry_.publish("kinematic/velocity_east_mps",   t, s.velocity_NED_mps()(1));
            registry_.publish("kinematic/velocity_down_mps",   t, s.velocity_NED_mps()(2));
            registry_.publish("kinematic/heading_rad",         t, s.heading());
            registry_.publish("kinematic/pitch_rad",           t, s.pitch());
            registry_.publish("kinematic/roll_rad",            t, s.roll());
            registry_.publish("kinematic/alpha_rad",           t, s.alpha());
            registry_.publish("kinematic/beta_rad",            t, s.beta());
            registry_.publish("kinematic/airspeed_m_s",        t, s.velocity_Wind_mps()(0));
            registry_.publish("kinematic/roll_rate_rad_s",     t, s.rollRate_Wind_rps());
        }

        if (timed) {
            const auto t_target = t_start + std::chrono::duration_cast<Clock::duration>(
                Duration(static_cast<double>(k + 1) * wall_step_s));
            const auto t_now = Clock::now();
            if (t_target > t_now) {
                std::this_thread::sleep_until(t_target);
            }
        }
    }
}

}  // namespace liteaero::simulation
