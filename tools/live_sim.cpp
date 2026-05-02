// live_sim — C++ live simulation launcher with joystick or scripted input.
//
// Design authority: docs/architecture/live_sim_view.md §Launch Modes
//
// Owns Aircraft, SimRunner, JoystickInput (or ScriptedInput), and
// UdpSimulationBroadcaster.  Wires them together and blocks on the run loop.
// The Godot scene is launched separately by the developer.
//
// Usage:
//   live_sim --config <aircraft-json> --terrain <terrain-config-json>
//            [--joystick <joystick-json>] [--device <index>]
//            [--dt <seconds>] [--port <udp-port>]
//
// Arguments:
//   --config   Aircraft JSON config file (required)
//   --terrain  terrain_config.json path written by build_terrain.py (required)
//   --joystick JoystickInput JSON config file; enables joystick mode
//   --device   SDL device index to use with joystick mode (default 0)
//   --dt       Simulation timestep in seconds (default 0.02)
//   --port     UDP broadcast port (default 14560)

#include "Aircraft.hpp"
#include "broadcaster/UdpSimulationBroadcaster.hpp"
#include "environment/TerrainMesh.hpp"
#include "geodesy/Egm2008Geoid.hpp"
#include "input/JoystickInput.hpp"
#include "input/ScriptedInput.hpp"
#include "projection/GodotEnuProjector.hpp"
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
#include <vector>

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
    std::string terrain_config_path;
    std::string joystick_config_path;
    int         device_index = 0;
    float       dt_s         = 0.02f;
    uint16_t    port         = 14560;
};

static void print_usage(const char* prog)
{
    std::cerr
        << "Usage: " << prog
        << " --config <aircraft-json> --terrain <terrain-config-json>"
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
        } else if (flag == "--terrain" && i + 1 < argc) {
            args.terrain_config_path = argv[++i];
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
    if (args.terrain_config_path.empty()) {
        std::cerr << "Error: --terrain is required\n";
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

    // --- Locate terrain_config.json ---
    // Path supplied via --terrain; written by build_terrain.py.
    const std::string tc_path = args.terrain_config_path;

    nlohmann::json terrain_config_json;
    {
        std::ifstream tc_file(tc_path);
        if (!tc_file.is_open()) {
            std::cerr << "Error: terrain_config.json not found at " << tc_path
                      << "\n       Run build_terrain.py first.\n";
            return 1;
        }
        try {
            tc_file >> terrain_config_json;
        } catch (const std::exception& e) {
            std::cerr << "Error: JSON parse error in " << tc_path
                      << ": " << e.what() << "\n";
            return 1;
        }
    }

    if (!terrain_config_json.contains("las_terrain_path")) {
        std::cerr << "Error: terrain_config.json missing 'las_terrain_path' field.\n"
                     "       Regenerate it by re-running build_terrain.py.\n";
        return 1;
    }
    const std::string las_terrain_path =
        terrain_config_json.at("las_terrain_path").get<std::string>();

    // --- Load TerrainMesh ---
    liteaero::simulation::TerrainMesh terrain_mesh;
    {
        std::ifstream las_file(las_terrain_path, std::ios::binary);
        if (!las_file.is_open()) {
            std::cerr << "Error: .las_terrain file not found: " << las_terrain_path
                      << "\n       Run build_terrain.py first.\n";
            return 1;
        }
        const std::vector<uint8_t> bytes(
            (std::istreambuf_iterator<char>(las_file)),
            std::istreambuf_iterator<char>());
        try {
            terrain_mesh.deserializeLasTerrain(bytes);
        } catch (const std::exception& e) {
            std::cerr << "Error: failed to load .las_terrain: " << e.what() << "\n";
            return 1;
        }
        std::cout << "Terrain loaded from " << las_terrain_path << "\n";
    }

    // --- Construct Aircraft ---
    std::unique_ptr<Aircraft> aircraft;
    try {
        aircraft = std::make_unique<Aircraft>(make_propulsion(config_json));
        aircraft->initialize(config_json, args.dt_s);
        aircraft->setTerrain(&terrain_mesh);
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

    // --- Construct viewer projector (LS-T7 / OQ-LS-15) ---
    // World origin from terrain_config.json; the same value the GLB tile
    // placement was computed against.  The projector applies curvature-aware
    // ECEF -> ENU and the glTF axis permutation; the Godot SimulationReceiver
    // simply reads viewer_x/y/z_m off the wire (LS-T8).
    std::unique_ptr<liteaero::projection::GodotEnuProjector> projector;
    if (terrain_config_json.contains("world_origin_lat_rad")
     && terrain_config_json.contains("world_origin_lon_rad")
     && terrain_config_json.contains("world_origin_height_m")) {
        liteaero::projection::ViewerOrigin origin{
            terrain_config_json.at("world_origin_lat_rad").get<double>(),
            terrain_config_json.at("world_origin_lon_rad").get<double>(),
            terrain_config_json.at("world_origin_height_m").get<double>(),
        };
        projector = std::make_unique<liteaero::projection::GodotEnuProjector>(origin);
        std::cout << "Viewer projector configured at origin lat="
                  << origin.lat_rad << " lon=" << origin.lon_rad
                  << " h=" << origin.height_wgs84_m << " m\n";
    } else {
        std::cerr << "Warning: terrain_config.json missing world_origin_* fields; "
                     "broadcaster will leave viewer fields zero (LS-T8 fallback).\n";
    }

    // --- Optional EGM2008 geoid for atmosphere h_WGS84 -> h_MSL conversion ---
    // The runtime grid file is generated by python/tools/terrain/generate_egm_grid.py.
    // Absence is non-fatal: SimRunner falls back to treating h_WGS84 as h_MSL
    // (~ N bias on density, well below other ISA approximations).
    std::unique_ptr<liteaero::geodesy::Egm2008Geoid> geoid;
    {
        const std::string geoid_path = "data/proj_grids/egm2008.bin";
        try {
            geoid = std::make_unique<liteaero::geodesy::Egm2008Geoid>(geoid_path);
            std::cout << "EGM2008 geoid loaded from " << geoid_path << "\n";
        } catch (const std::exception& e) {
            std::cerr << "Note: EGM2008 grid not loaded (" << e.what()
                      << ").  Atmospheric density bias of order N (~ 30 m at "
                         "mid-latitudes) accepted.  Run "
                         "python/tools/terrain/generate_egm_grid.py to enable.\n";
        }
    }

    // --- Construct broadcaster ---
    UdpSimulationBroadcaster broadcaster(args.port, projector.get());
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
    runner.setGeoid(geoid.get());

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
