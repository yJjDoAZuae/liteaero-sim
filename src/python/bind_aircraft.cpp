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
#include "physics/ContactForces.hpp"
#include "landing_gear/StrutState.hpp"
#include "landing_gear/LandingGear.hpp"
#include "propulsion/PropulsionJet.hpp"
#include "propulsion/PropulsionEDF.hpp"
#include "propulsion/PropulsionProp.hpp"
#include "propulsion/MotorElectric.hpp"
#include "propulsion/MotorPiston.hpp"
#include <liteaero/terrain/Terrain.hpp>
#include <nlohmann/json.hpp>

#include <Eigen/Dense>
#include <array>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

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
        throw std::invalid_argument(
            "Aircraft: config is missing the required \"propulsion\" section. Add a "
            "\"propulsion\" object with a \"type\" key (\"jet\", \"edf\", \"prop\", or "
            "\"none\"); use \"none\" to model an unpowered airframe explicitly. An omitted "
            "section is rejected so a config that forgets its engine cannot silently become "
            "a zero-thrust aircraft that can never accelerate or take off.");

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
    // --- ContactForces ---
    py::class_<ContactForces>(m, "ContactForces")
        .def_property_readonly("force_body_n",
            [](const ContactForces& cf) -> std::array<float, 3> {
                return {cf.force_body_n.x(), cf.force_body_n.y(), cf.force_body_n.z()};
            },
            "Body-frame contact force [Fx, Fy, Fz] (N).  +X forward, +Y right, +Z down.")
        .def_property_readonly("moment_body_nm",
            [](const ContactForces& cf) -> std::array<float, 3> {
                return {cf.moment_body_nm.x(), cf.moment_body_nm.y(), cf.moment_body_nm.z()};
            },
            "Body-frame contact moment [Mx, My, Mz] (N·m).")
        .def_property_readonly("weight_on_wheels",
            [](const ContactForces& cf) { return cf.weight_on_wheels; },
            "True when at least one wheel is in contact with terrain.");

    // --- StrutState ---
    py::class_<StrutState>(m, "StrutState")
        .def_property_readonly("deflection_m",
            [](const StrutState& s) { return s.strut_deflection_m; },
            "Strut spring compression (m).  0 = fully extended; travel_max = fully compressed.")
        .def_property_readonly("deflection_rate_mps",
            [](const StrutState& s) { return s.strut_deflection_rate_mps; },
            "Rate of strut compression (m/s).  Positive = compressing.")
        .def_property_readonly("wheel_speed_rps",
            [](const StrutState& s) { return s.wheel_speed_rps; },
            "Wheel angular speed (rad/s).");

    // --- WheelUnit::ContactDiag (per-wheel contact-force breakdown; model-diagnosis interface) ---
    py::class_<WheelUnit::ContactDiag>(m, "WheelContactDiag")
        .def_readonly("F_z",       &WheelUnit::ContactDiag::F_z,       "Normal force magnitude (N).")
        .def_readonly("F_x",       &WheelUnit::ContactDiag::F_x,       "Pacejka longitudinal force (N, +fwd).")
        .def_readonly("F_y",       &WheelUnit::ContactDiag::F_y,       "Pacejka lateral force (N). Zero for a castering wheel.")
        .def_readonly("F_rr",      &WheelUnit::ContactDiag::F_rr,      "Rolling resistance (N, signed).")
        .def_readonly("kappa",     &WheelUnit::ContactDiag::kappa,     "Longitudinal slip ratio.")
        .def_readonly("alpha_t",   &WheelUnit::ContactDiag::alpha_t,   "Slip angle (rad).")
        .def_readonly("V_cx",      &WheelUnit::ContactDiag::V_cx,      "Contact-patch forward speed (m/s).")
        .def_readonly("V_cy",      &WheelUnit::ContactDiag::V_cy,      "Contact-patch lateral speed (m/s).")
        .def_readonly("omega",     &WheelUnit::ContactDiag::omega,     "Wheel spin (rad/s).")
        .def_readonly("delta_dot", &WheelUnit::ContactDiag::delta_dot, "Strut closing rate (m/s).")
        .def_property_readonly("force_body_n",
            [](const WheelUnit::ContactDiag& d) {
                return std::array<float, 3>{d.force_body.x(), d.force_body.y(), d.force_body.z()};
            },
            "Per-wheel contact force in the body frame (N) as [x, y, z].");

    // --- Aircraft::StepDiag (verbose Δθ / attitude debug accessors; model-diagnosis interface) ---
    py::class_<Aircraft::StepDiag>(m, "AircraftStepDiag")
        .def_readonly("dtheta_pitch", &Aircraft::StepDiag::dtheta_pitch,
            "Total pitch rotation deviation Δθ_pitch (rad) = force + moment channels; feeds α_body\n"
            "and the gear geometry.")
        .def_readonly("dtheta_force", &Aircraft::StepDiag::dtheta_force,
            "Force-channel Δθ (rad): G(s)·u on the destanced gear+body vertical load, faded by\n"
            "Φ(V). →0 in steady roll and at low airspeed.")
        .def_readonly("dtheta_moment_pitch", &Aircraft::StepDiag::dtheta_moment_pitch,
            "Moment-channel Δθ (rad): (1/ωₙ²)·H₂(M_pitch/I_yy) over gear + body collider. Finite\n"
            "DC = static stance; no velocity fade (live even at rest).")
        .def_readonly("dtheta_yaw", &Aircraft::StepDiag::dtheta_yaw,
            "Yaw rotation deviation Δθ_yaw (rad).")
        .def_readonly("alpha_cmd", &Aircraft::StepDiag::alpha_cmd,
            "LFA-commanded angle of attack (rad).")
        .def_readonly("alpha_body", &Aircraft::StepDiag::alpha_body,
            "Body angle of attack (rad) = alpha_cmd + dtheta_pitch (used downstream and for the\n"
            "gear geometry at the next step).")
        .def_readonly("phi_authority", &Aircraft::StepDiag::phi_authority,
            "Φ(V) dynamic-pressure authority fade (nd, [0,1]); 0 at rest, →1 with airspeed. Gates\n"
            "the force channel.")
        .def_readonly("gamma_fpa_rad", &Aircraft::StepDiag::gamma_fpa_rad,
            "Inertial flight-path angle γ (rad) used in the force channel.")
        .def_readonly("a_arrest_gear_mps2", &Aircraft::StepDiag::a_arrest_gear_mps2,
            "Gear destanced vertical acceleration (F_z − F_stance)/m (m/s²) — the force-channel input.")
        .def_readonly("fz_stance_gear_n", &Aircraft::StepDiag::fz_stance_gear_n,
            "Gear stance (destance reference) vertical load in NED (N).")
        .def_readonly("gear_moment_pitch_nm", &Aircraft::StepDiag::gear_moment_pitch_nm,
            "Gear pitching moment about the CG (body Y, N·m) — drives the moment channel.")
        .def_readonly("bc_moment_pitch_nm", &Aircraft::StepDiag::bc_moment_pitch_nm,
            "Body-collider pitching moment about the CG (body Y, N·m).")
        .def_readonly("n_z_shaped", &Aircraft::StepDiag::n_z_shaped,
            "FBW-shaped normal load-factor command feeding the LFA (g).")
        .def_readonly("weight_on_wheels", &Aircraft::StepDiag::weight_on_wheels,
            "True if any wheel or body-collider contact occurred on this step.");

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
        .def_property_readonly("pitch_rate_rad_s",
            [](const KinematicState& s) {
                return static_cast<double>(s.pitchRate_rps()); })
        .def_property_readonly("yaw_rate_rad_s",
            [](const KinematicState& s) {
                return static_cast<double>(s.headingRate_rps()); })
        .def_property_readonly("crab_rad",
            [](const KinematicState& s) {
                return static_cast<double>(s.crab_rad()); },
            "Crab angle (air-track azimuth − ground-track azimuth), rad (OQ-AC-4).")
        .def_property_readonly("air_track_azimuth_rad",
            [](const KinematicState& s) {
                return static_cast<double>(s.airTrackAzimuth_rad()); })
        .def_property_readonly("ground_track_azimuth_rad",
            [](const KinematicState& s) {
                return static_cast<double>(s.groundTrackAzimuth_rad()); })
        .def_property_readonly("acceleration_ned_mps2",
            [](const KinematicState& s) -> std::array<double, 3> {
                const auto a = s.acceleration_NED_mps();
                return {static_cast<double>(a.x()),
                        static_cast<double>(a.y()),
                        static_cast<double>(a.z())};
            },
            "NED-frame acceleration [aN, aE, aD] (m/s²).  +D = downward.")
        .def_property_readonly("velocity_body_mps",
            [](const KinematicState& s) -> std::array<double, 3> {
                const auto v = s.velocity_Body_mps();
                return {static_cast<double>(v.x()),
                        static_cast<double>(v.y()),
                        static_cast<double>(v.z())};
            },
            "Body-frame velocity [u, v, w] (m/s).  +X forward, +Y right, +Z down.")
        .def_property_readonly("rates_body_rad_s",
            [](const KinematicState& s) -> std::array<double, 3> {
                const auto r = s.rates_Body_rps();
                return {static_cast<double>(r.x()),
                        static_cast<double>(r.y()),
                        static_cast<double>(r.z())};
            },
            "Body-frame angular rates [p, q, r] (rad/s).")
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
             "A \"propulsion\" section with a \"type\" key (\"jet\", \"edf\", \"prop\", or "
             "\"none\") is required; \"none\" selects a zero-thrust stub for an unpowered "
             "airframe. Omitting the section is an error.")
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
                float rho_kgm3,
                const std::array<float, 3>& wind_ned_mps) {
                 self.time_s += dt_s;
                 self.aircraft->step(self.time_s, cmd,
                                     Eigen::Vector3f{wind_ned_mps[0], wind_ned_mps[1],
                                                     wind_ned_mps[2]},
                                     rho_kgm3);
             },
             py::arg("cmd"),
             py::arg("dt_s")      = 0.02f,
             py::arg("rho_kgm3") = 1.225f,
             py::arg("wind_ned_mps") = std::array<float, 3>{0.f, 0.f, 0.f},
             "Advance simulation by one timestep.\n\n"
             "wind_ned_mps: steady NED wind vector (m/s), default zero; rho_kgm3 overrides\n"
             "ISA sea-level density.")
        .def("state",
             [](const PyAircraft& self) -> const KinematicState& {
                 return self.aircraft->state();
             },
             py::return_value_policy::reference_internal,
             "Return the current kinematic state (read-only reference).")
        .def("set_terrain",
             [](PyAircraft& self, const liteaero::terrain::Terrain* terrain) {
                 self.aircraft->setTerrain(terrain);
             },
             py::arg("terrain"),
             py::keep_alive<1, 2>(),  // keep terrain alive as long as this Aircraft is alive
             "Set the terrain model used by LandingGear and BodyCollider each step.\n\n"
             "Must be called before the first step() if landing-gear contact forces are\n"
             "needed.  Pass None (or do not call) to disable gear contact.")
        .def("contact_forces",
             [](const PyAircraft& self) -> const ContactForces& {
                 return self.aircraft->contactForces();
             },
             py::return_value_policy::reference_internal,
             "Landing gear + body-collider contact forces from the most recent step().")
        .def_property_readonly("weight_on_wheels",
             [](const PyAircraft& self) {
                 return self.aircraft->weightOnWheels();
             },
             "True when at least one wheel was in contact on the most recent step().")
        .def("agl_m",
             [](const PyAircraft& self) {
                 return self.aircraft->agl_m();
             },
             "Height above ground level (m) at the current CG position.\n\n"
             "Returns -1 if no terrain has been set via set_terrain().")
        .def_property_readonly("cl_eff",
             [](const PyAircraft& self) { return self.aircraft->clEff(); },
             "Effective lift coefficient applied on the most recent step(): the nominal lift-curve\n"
             "value in attached flow, or the rate-limited value during stall recovery.")
        .def_property_readonly("is_stalled",
             [](const PyAircraft& self) { return self.aircraft->isStalled(); },
             "True if the wing was stalled (positive- or negative-side) on the most recent step().")
        .def_property_readonly("is_cl_recovering",
             [](const PyAircraft& self) { return self.aircraft->isClRecovering(); },
             "True while the post-stall CL recovery rate-limit is active.")
        .def("gear_strut_states",
             [](const PyAircraft& self) -> std::vector<StrutState> {
                 if (!self.aircraft->hasLandingGear()) return {};
                 const auto& units = self.aircraft->landingGear().wheelUnits();
                 std::vector<StrutState> out;
                 out.reserve(units.size());
                 for (const auto& wu : units)
                     out.push_back(wu.strutState());
                 return out;
             },
             "Per-wheel strut states from the most recent step().\n\n"
             "Returns a list of StrutState objects in the same order as\n"
             "the wheel_units array in the aircraft config.  Returns an empty\n"
             "list when no landing gear is configured.")
        .def("wheel_contact_diags",
             [](const PyAircraft& self) -> std::vector<WheelUnit::ContactDiag> {
                 if (!self.aircraft->hasLandingGear()) return {};
                 const auto& units = self.aircraft->landingGear().wheelUnits();
                 std::vector<WheelUnit::ContactDiag> out;
                 out.reserve(units.size());
                 for (const auto& wu : units)
                     out.push_back(wu.lastContactDiag());
                 return out;
             },
             "Per-wheel contact-force breakdown (WheelContactDiag) from the most recent\n"
             "step(): normal / longitudinal / lateral / rolling-resistance forces, slip\n"
             "ratio and angle, contact-patch velocities, and wheel spin.  Same order as\n"
             "the wheel_units config; empty when no landing gear is configured.")
        .def_property_readonly("roll_rate_state_rps",
             [](const PyAircraft& self) { return self.aircraft->rollRateState_rps(); },
             "OQ-BC-12 Alt B persistent wind-axis roll-rate state (rad/s) committed to the\n"
             "attitude each step (the induced contact roll rate, Tustin-integrated).")
        .def("step_diag",
             [](const PyAircraft& self) -> const Aircraft::StepDiag& {
                 return self.aircraft->stepDiag();
             },
             py::return_value_policy::reference_internal,
             "Verbose Δθ / attitude debug diagnostics (AircraftStepDiag) from the most recent\n"
             "step(): the pitch rotation-deviation decomposition (force vs moment channel),\n"
             "α_cmd/α_body, the Φ(V) authority fade, flight-path angle, gear destance/stance\n"
             "load, and the gear / body-collider pitching moments. Recomputed each step (not\n"
             "serialized). Use for model diagnosis of the OQ-LG-15 gear-F&M attitude behavior.");
}

