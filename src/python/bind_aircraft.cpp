// Aircraft and KinematicState bindings.
//
// Design authority: docs/architecture/python_bindings.md
//
// Aircraft is exposed through PyAircraft (py_aircraft_types.hpp), which owns
// both the Propulsion and the Aircraft and tracks simulation time internally.
// The propulsion model is selected from the optional "propulsion" section of the
// JSON config; if that section is absent, a zero-thrust stub is used.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "py_aircraft_types.hpp"
#include "KinematicState.hpp"
#include "propulsion/PropulsionJet.hpp"
#include "propulsion/PropulsionEDF.hpp"
#include "propulsion/PropulsionProp.hpp"
#include "propulsion/MotorElectric.hpp"
#include "propulsion/MotorPiston.hpp"
#include <nlohmann/json.hpp>

#include <Eigen/Dense>
#include <fstream>
#include <stdexcept>
#include <string>

namespace py = pybind11;
using namespace liteaero::simulation;

// ---------------------------------------------------------------------------
// ZeroThrustPropulsion — used when config has no "propulsion" section
// ---------------------------------------------------------------------------

namespace {

class ZeroThrustPropulsion final : public Propulsion {
public:
    float step(float, float, float) override          { return 0.0f; }
    float thrust_n() const override                   { return 0.0f; }

    std::vector<uint8_t> serializeProto() const override           { return {}; }
    void                 deserializeProto(const std::vector<uint8_t>&) override {}

protected:
    void           onInitialize(const nlohmann::json&) override {}
    void           onReset() override                           {}
    nlohmann::json onSerializeJson() const override             { return {}; }
    void           onDeserializeJson(const nlohmann::json&) override {}
    int            schemaVersion() const override               { return 1; }
    const char*    typeName() const override                    { return "ZeroThrustPropulsion"; }
};

// Load JSON from an inline string or a file path.
// A string that does not start with '{' is treated as a file path.
static nlohmann::json load_json(const std::string& json_or_path)
{
    if (!json_or_path.empty() && json_or_path.front() != '{') {
        std::ifstream f(json_or_path);
        if (!f.is_open())
            throw std::runtime_error("Aircraft: cannot open config file: " + json_or_path);
        nlohmann::json j;
        f >> j;
        return j;
    }
    return nlohmann::json::parse(json_or_path);
}

// Create and initialize the appropriate Propulsion subclass from the JSON config.
// Reads config["propulsion"]["type"] if the "propulsion" key is present.
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
    }
    if (type == "edf") {
        auto p = std::make_unique<PropulsionEDF>();
        p->initialize(prop_cfg);
        return p;
    }
    if (type == "prop") {
        // PropulsionProp requires a Motor at construction.
        // The config "propulsion" section must contain a "motor" object with a "type"
        // key of "electric" or "piston" and the relevant parameters for that motor.
        const auto&       motor_cfg  = prop_cfg.at("motor");
        const std::string motor_type = motor_cfg.at("type").get<std::string>();

        std::unique_ptr<Motor> motor;
        if (motor_type == "electric") {
            MotorElectricMotorParams mp;
            mp.kv_rps_per_v       = motor_cfg.at("kv_rps_per_v").get<float>();
            mp.r_terminal_ohm     = motor_cfg.at("r_terminal_ohm").get<float>();
            mp.inertia_motor_kg_m2 = motor_cfg.at("inertia_motor_kg_m2").get<float>();
            MotorElectricEscParams ep;
            ep.supply_voltage_v  = motor_cfg.at("supply_voltage_v").get<float>();
            ep.i_max_a           = motor_cfg.at("i_max_a").get<float>();
            ep.esc_efficiency_nd = motor_cfg.at("esc_efficiency_nd").get<float>();
            motor = std::make_unique<MotorElectric>(mp, ep);
        } else if (motor_type == "piston") {
            motor = std::make_unique<MotorPiston>(
                motor_cfg.at("power_max_w").get<float>(),
                motor_cfg.at("peak_omega_rps").get<float>(),
                motor_cfg.at("altitude_exponent").get<float>(),
                motor_cfg.at("inertia_kg_m2").get<float>());
        } else {
            throw std::invalid_argument(
                "Aircraft: unknown motor type \"" + motor_type
                + "\". Expected: \"electric\" or \"piston\".");
        }

        auto p = std::make_unique<PropulsionProp>(std::move(motor));
        p->initialize(prop_cfg);
        return p;
    }
    if (type == "none")
        return std::make_unique<ZeroThrustPropulsion>();

    throw std::invalid_argument("Aircraft: unknown propulsion type \"" + type
                                + "\". Expected: \"jet\", \"edf\", \"prop\", \"none\".");
}

} // anonymous namespace

// ---------------------------------------------------------------------------

void bind_aircraft(py::module_& m)
{
    // --- KinematicState (read-only; returned by Aircraft.state()) ---
    py::class_<KinematicState>(m, "KinematicState")
        .def_property_readonly("time_s",
            &KinematicState::time_sec)
        .def_property_readonly("latitude_rad",
            [](const KinematicState& s) {
                return s.positionDatum().latitudeGeodetic_rad(); })
        .def_property_readonly("longitude_rad",
            [](const KinematicState& s) {
                return s.positionDatum().longitude_rad(); })
        .def_property_readonly("altitude_m",
            [](const KinematicState& s) {
                return static_cast<double>(s.positionDatum().height_WGS84_m()); })
        .def_property_readonly("velocity_north_mps",
            [](const KinematicState& s) {
                return static_cast<double>(s.velocity_NED_mps()(0)); })
        .def_property_readonly("velocity_east_mps",
            [](const KinematicState& s) {
                return static_cast<double>(s.velocity_NED_mps()(1)); })
        .def_property_readonly("velocity_down_mps",
            [](const KinematicState& s) {
                return static_cast<double>(s.velocity_NED_mps()(2)); })
        .def_property_readonly("heading_rad",
            [](const KinematicState& s) {
                return static_cast<double>(s.heading()); })
        .def_property_readonly("pitch_rad",
            [](const KinematicState& s) {
                return static_cast<double>(s.pitch()); })
        .def_property_readonly("roll_rad",
            [](const KinematicState& s) {
                return static_cast<double>(s.roll()); })
        .def_property_readonly("alpha_rad",
            [](const KinematicState& s) {
                return static_cast<double>(s.alpha()); })
        .def_property_readonly("beta_rad",
            [](const KinematicState& s) {
                return static_cast<double>(s.beta()); })
        .def_property_readonly("airspeed_m_s",
            // Wind-axis forward speed = TAS.
            [](const KinematicState& s) {
                return static_cast<double>(s.velocity_Wind_mps()(0)); })
        .def_property_readonly("roll_rate_rad_s",
            [](const KinematicState& s) {
                return static_cast<double>(s.rollRate_Wind_rps()); })
        .def("__repr__", [](const KinematicState& s) {
            return "KinematicState(altitude_m="
                 + std::to_string(s.positionDatum().height_WGS84_m())
                 + ", airspeed_m_s="
                 + std::to_string(s.velocity_Wind_mps()(0))
                 + ", heading_rad="
                 + std::to_string(s.heading()) + ")";
        });

    // --- Aircraft (wraps PyAircraft) ---
    py::class_<PyAircraft>(m, "Aircraft")
        .def(py::init([](const std::string& config_str_or_path, float outer_dt_s) {
                const nlohmann::json config = load_json(config_str_or_path);
                auto propulsion = make_propulsion(config);
                auto pa         = std::make_unique<PyAircraft>();
                pa->outer_dt_s  = outer_dt_s;
                pa->aircraft    = std::make_unique<Aircraft>(std::move(propulsion));
                pa->aircraft->initialize(config, outer_dt_s);
                return pa;
             }),
             py::arg("config"),
             py::arg("dt_s") = 0.02f,
             "Construct from a JSON config string or file path.\n\n"
             "Include a \"propulsion\" section with a \"type\" key "
             "(\"jet\", \"edf\", \"prop\", or \"none\") to select a propulsion model; "
             "omit the section to use a zero-thrust stub.")
        .def("reset",
             [](PyAircraft& self) {
                 self.time_s = 0.0;
                 self.aircraft->reset();
             },
             "Reset aircraft to initial conditions; resets internal simulation time to zero.")
        .def("step",
             [](PyAircraft& self,
                const AircraftCommand& cmd,
                float dt_s,
                float rho_kgm3) {
                 self.time_s += dt_s;
                 self.aircraft->step(self.time_s, cmd,
                                     Eigen::Vector3f::Zero(),
                                     rho_kgm3);
             },
             py::arg("cmd"),
             py::arg("dt_s")      = 0.02f,
             py::arg("rho_kgm3") = 1.225f,
             "Advance simulation by one timestep.\n\n"
             "Wind is assumed zero; use rho_kgm3 to override ISA sea-level density.")
        .def("state",
             [](const PyAircraft& self) -> const KinematicState& {
                 return self.aircraft->state();
             },
             py::return_value_policy::reference_internal,
             "Return the current kinematic state (read-only reference).");
}
