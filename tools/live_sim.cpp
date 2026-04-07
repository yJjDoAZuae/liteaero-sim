// live_sim — C++ live simulation launcher with joystick or scripted input.
//
// Design authority: docs/architecture/live_sim_view.md §Launch Modes
//
// Owns Aircraft, SimRunner, JoystickInput (or ScriptedInput), and
// UdpSimulationBroadcaster.  Wires them together and blocks on the run loop.
// The Godot scene is launched separately by the developer.
//
// Usage:
//   live_sim --config <aircraft-json> [--joystick <joystick-json>]
//            [--device <index>] [--dt <seconds>] [--port <udp-port>]
//
// Arguments:
//   --config   Aircraft JSON config file (required)
//   --joystick JoystickInput JSON config file; enables joystick mode
//   --device   SDL device index to use with joystick mode (default 0)
//   --dt       Simulation timestep in seconds (default 0.02)
//   --port     UDP broadcast port (default 14560)

#include "Aircraft.hpp"
#include "broadcaster/UdpSimulationBroadcaster.hpp"
#include "input/JoystickInput.hpp"
#include "input/ScriptedInput.hpp"
#include "propulsion/PropulsionEDF.hpp"
#include "propulsion/PropulsionJet.hpp"
#include "propulsion/PropulsionProp.hpp"
#include "propulsion/MotorElectric.hpp"
#include "propulsion/MotorPiston.hpp"
#include "propulsion/Propulsion.hpp"
#include "runner/SimRunner.hpp"

#include <SDL2/SDL.h>
#include <nlohmann/json.hpp>

#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace liteaero::simulation;

// ---------------------------------------------------------------------------
// Zero-thrust propulsion stub — used when config has no "propulsion" section.
// ---------------------------------------------------------------------------

namespace {

class ZeroThrustPropulsion final : public Propulsion {
public:
    float step(float, float, float) override { return 0.0f; }
    float thrust_n() const override          { return 0.0f; }

    std::vector<uint8_t> serializeProto() const override           { return {}; }
    void                 deserializeProto(const std::vector<uint8_t>&) override {}

protected:
    void           onInitialize(const nlohmann::json&) override {}
    void           onReset() override {}
    nlohmann::json onSerializeJson() const override  { return {}; }
    void           onDeserializeJson(const nlohmann::json&) override {}
    int            schemaVersion() const override    { return 1; }
    const char*    typeName() const override         { return "ZeroThrustPropulsion"; }
};

static std::unique_ptr<Propulsion> make_propulsion(const nlohmann::json& config)
{
    if (!config.contains("propulsion"))
        return std::make_unique<ZeroThrustPropulsion>();

    const auto&       prop_cfg = config.at("propulsion");
    const std::string type     = prop_cfg.at("type").get<std::string>();

    if (type == "jet") {
        auto p = std::make_unique<PropulsionJet>();
        p->initialize(prop_cfg);
        return p;
    } else if (type == "edf") {
        auto p = std::make_unique<PropulsionEDF>();
        p->initialize(prop_cfg);
        return p;
    } else if (type == "prop") {
        const auto&       motor_cfg  = prop_cfg.at("motor");
        const std::string motor_type = motor_cfg.at("type").get<std::string>();
        std::unique_ptr<Motor> motor;
        if (motor_type == "electric") {
            MotorElectricMotorParams mp;
            mp.kv_rps_per_v        = motor_cfg.at("kv_rps_per_v").get<float>();
            mp.r_terminal_ohm      = motor_cfg.at("r_terminal_ohm").get<float>();
            mp.inertia_motor_kg_m2 = motor_cfg.at("inertia_motor_kg_m2").get<float>();
            MotorElectricEscParams ep;
            ep.supply_voltage_v   = motor_cfg.at("supply_voltage_v").get<float>();
            ep.i_max_a            = motor_cfg.at("i_max_a").get<float>();
            ep.esc_efficiency_nd  = motor_cfg.at("esc_efficiency_nd").get<float>();
            motor = std::make_unique<MotorElectric>(mp, ep);
        } else if (motor_type == "piston") {
            motor = std::make_unique<MotorPiston>(
                motor_cfg.at("power_max_w").get<float>(),
                motor_cfg.at("peak_omega_rps").get<float>(),
                motor_cfg.at("altitude_exponent").get<float>(),
                motor_cfg.at("inertia_kg_m2").get<float>());
        } else {
            throw std::invalid_argument("live_sim: unknown motor type: " + motor_type);
        }
        auto p = std::make_unique<PropulsionProp>(std::move(motor));
        p->initialize(prop_cfg);
        return p;
    } else if (type == "none") {
        return std::make_unique<ZeroThrustPropulsion>();
    }
    return std::make_unique<ZeroThrustPropulsion>();
}

} // namespace

// ---------------------------------------------------------------------------
// Signal handling — SIGINT / SIGTERM → request stop
// ---------------------------------------------------------------------------

static volatile bool g_stop_requested = false;

static void handle_signal(int /*sig*/)
{
    g_stop_requested = true;
}

// ---------------------------------------------------------------------------
// Argument parsing
// ---------------------------------------------------------------------------

struct Args {
    std::string config_path;
    std::string joystick_config_path;
    int         device_index = 0;
    float       dt_s         = 0.02f;
    uint16_t    port         = 14560;
};

static void print_usage(const char* prog)
{
    std::cerr
        << "Usage: " << prog
        << " --config <aircraft-json>"
           " [--joystick <joystick-json>] [--device <index>]"
           " [--dt <seconds>] [--port <udp-port>]\n";
}

static Args parse_args(int argc, char** argv)
{
    Args args;
    for (int i = 1; i < argc; ++i) {
        const std::string flag(argv[i]);
        if ((flag == "--config" || flag == "-c") && i + 1 < argc) {
            args.config_path = argv[++i];
        } else if (flag == "--joystick" && i + 1 < argc) {
            args.joystick_config_path = argv[++i];
        } else if (flag == "--device" && i + 1 < argc) {
            args.device_index = std::stoi(argv[++i]);
        } else if (flag == "--dt" && i + 1 < argc) {
            args.dt_s = std::stof(argv[++i]);
        } else if (flag == "--port" && i + 1 < argc) {
            args.port = static_cast<uint16_t>(std::stoi(argv[++i]));
        } else {
            std::cerr << "Unknown argument: " << flag << "\n";
            print_usage(argv[0]);
            std::exit(1);
        }
    }
    if (args.config_path.empty()) {
        std::cerr << "Error: --config is required\n";
        print_usage(argv[0]);
        std::exit(1);
    }
    return args;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    std::signal(SIGINT,  handle_signal);
    std::signal(SIGTERM, handle_signal);

    const Args args = parse_args(argc, argv);

    // --- Load aircraft config ---
    std::ifstream config_file(args.config_path);
    if (!config_file.is_open()) {
        std::cerr << "Error: cannot open config file: " << args.config_path << "\n";
        return 1;
    }
    nlohmann::json config_json;
    try {
        config_file >> config_json;
    } catch (const std::exception& e) {
        std::cerr << "Error: JSON parse error in " << args.config_path
                  << ": " << e.what() << "\n";
        return 1;
    }

    // --- Construct Aircraft ---
    std::unique_ptr<Aircraft> aircraft;
    try {
        aircraft = std::make_unique<Aircraft>(make_propulsion(config_json));
        aircraft->initialize(config_json, args.dt_s);
    } catch (const std::exception& e) {
        std::cerr << "Error: Aircraft initialization failed: " << e.what() << "\n";
        return 1;
    }

    // --- Construct manual input ---
    ScriptedInput scripted_input;
    std::unique_ptr<JoystickInput> joystick_input;

    ManualInput* active_input = nullptr;

    if (!args.joystick_config_path.empty()) {
        // Initialize SDL for joystick support.
        if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
            std::cerr << "Error: SDL_Init failed: " << SDL_GetError() << "\n";
            return 1;
        }

        std::ifstream joy_file(args.joystick_config_path);
        if (!joy_file.is_open()) {
            std::cerr << "Error: cannot open joystick config: "
                      << args.joystick_config_path << "\n";
            SDL_Quit();
            return 1;
        }
        nlohmann::json joy_json;
        try {
            joy_file >> joy_json;
        } catch (const std::exception& e) {
            std::cerr << "Error: joystick JSON parse error: " << e.what() << "\n";
            SDL_Quit();
            return 1;
        }
        try {
            // device_index is passed to the constructor; initialize() only takes config.
            joystick_input = std::make_unique<JoystickInput>(args.device_index);
            joystick_input->initialize(joy_json);
        } catch (const std::exception& e) {
            std::cerr << "Error: JoystickInput initialization failed: "
                      << e.what() << "\n";
            SDL_Quit();
            return 1;
        }
        active_input = joystick_input.get();
        std::cout << "Joystick mode: device " << args.device_index
                  << " from " << args.joystick_config_path << "\n";
    } else {
        active_input = &scripted_input;
        std::cout << "Scripted mode: neutral AircraftCommand on every tick\n";
    }

    // --- Construct broadcaster ---
    UdpSimulationBroadcaster broadcaster(args.port);
    std::cout << "Broadcasting to 127.0.0.1:" << args.port << "\n";

    // --- Configure and start SimRunner ---
    RunnerConfig cfg;
    cfg.mode       = ExecutionMode::RealTime;
    cfg.dt_s       = args.dt_s;
    cfg.duration_s = 0.0;  // run until stopped

    SimRunner runner;
    runner.initialize(cfg, *aircraft);
    runner.setManualInput(active_input);
    runner.set_broadcaster(&broadcaster);

    std::cout << "Starting live simulation (dt=" << args.dt_s
              << " s).  Press Ctrl+C to stop.\n";

    runner.start();  // returns immediately in RealTime mode

    // Block main thread until SIGINT/SIGTERM or runner stops for another reason.
    while (!g_stop_requested && runner.is_running()) {
        SDL_Delay(50);
    }

    runner.stop();

    if (!args.joystick_config_path.empty()) {
        SDL_Quit();
    }

    std::cout << "Simulation stopped after "
              << runner.elapsed_sim_time_s() << " s simulated.\n";
    return 0;
}
