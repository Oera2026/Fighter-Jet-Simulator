/**
 * @file test_aircraft_entity.cpp
 * @brief Phase 0 acceptance: the per-aircraft pipeline extracted from the
 * viewer's main loop into sim::Aircraft flies bit-identically to the original.
 *
 * ReferenceLoop below is the viewer's physics step as it stood before the
 * extraction (IMU -> OFC -> FLCS -> speedbrake -> engine -> fuel -> gear ->
 * turbulence -> RK4 -> collision), kept verbatim apart from the input source
 * and the combat payload now carried in the mass.
 * Both are driven with the same scripted inputs for 60 s and every state
 * component must match exactly, not within a tolerance.
 */
#include "fastjet/sim/aircraft.hpp"
#include "fastjet/sim/world.hpp"
#include <cassert>
#include <cmath>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>

using namespace fastjet;
using aircraft::AircraftType;

namespace {

constexpr double DT = 0.005;

struct ReferenceLoop {
    AircraftType type;
    propulsion::MultiEngine engine;
    gear::LandingGear landing_gear;
    fdm::MassProperties mass;
    const fdm::RK4Integrator integrator{0.005};
    aero::AircraftAeroModel aero_model;
    flcs::AircraftFLCS flight_control_system;
    flcs::OnBoardFlightComputer ofc;
    input::SpeedbrakeController speedbrake{};
    environment::WindTurbulenceModel wind_model;
    fdm::FlightState state{};
    fdm::AircraftForces current_forces{};
    flcs::IMUData current_imu{};
    bool is_crashed{false};
    double sim_time{0.0};

    explicit ReferenceLoop(AircraftType t)
        : type(t), engine(t), landing_gear(t), aero_model(t), flight_control_system(t), ofc(t) {
        mass = fdm::FuelSystem::compute(engine.fuel_kg, t, aircraft::AircraftConfig::get(t).mass.combat_payload_kg);
        wind_model.base_wind_speed_mps = 5.0;
        current_imu.Nz = 1.0;
    }

    void step(const sim::AircraftControls& in) {
        if (is_crashed) return;
        const flcs::PilotCommands pilot_cmd = in.stick;
        const auto air = environment::Atmosphere1976::compute(state.altitude(), state.airspeed());
        current_imu = flcs::IMUData::read(state, current_forces, mass, air.dynamic_pressure);
        const flcs::PilotCommands effective_cmd =
            ofc.update(DT, state, current_imu, pilot_cmd, landing_gear.deployed);
        auto surfaces = flight_control_system.update(DT, state, effective_cmd, air.dynamic_pressure,
                                                     current_forces, mass);
        speedbrake.update(in.speedbrake_out ? input::SpeedbrakeController::SwitchPosition::EXTEND
                                            : input::SpeedbrakeController::SwitchPosition::RETRACT,
                          DT);
        surfaces.speedbrake = speedbrake.position;
        double active_throttle = in.throttle;
        if (ofc.has_throttle_override()) active_throttle = ofc.throttle_override_value();
        engine.tvc_pitch_cmd = flight_control_system.tvc_pitch_cmd;
        const double thrust_n = engine.update(active_throttle, air, DT);
        mass = fdm::FuelSystem::compute(engine.fuel_kg, type, aircraft::AircraftConfig::get(type).mass.combat_payload_kg);
        landing_gear.steer_cmd = pilot_cmd.rudder_pedal;
        landing_gear.brake_left = in.brake_left;
        landing_gear.brake_right = in.brake_right;
        const double current_agl = environment::GroundCollision::get_agl(state);
        wind_model.update(DT, current_agl, state.airspeed());
        integrator.step(state, sim_time, mass,
            [&](double, const fdm::FlightState& s) noexcept -> fdm::AircraftForces {
                const double s_agl = environment::GroundCollision::get_agl(s);
                const math::Vector3 v_rel = wind_model.compute_relative_velocity(s.vel_b, s_agl, s.q_att);
                const double v_rel_norm = (std::max)(1.0, v_rel.norm());
                const auto stage_air = environment::Atmosphere1976::compute(s.altitude(), v_rel_norm);
                fdm::FlightState s_aero = s;
                s_aero.vel_b = v_rel;
                fdm::AircraftForces f = aero_model.compute_forces_and_moments(
                    s_aero, surfaces, stage_air.dynamic_pressure, stage_air.mach_number);
                f.force_b.x += thrust_n;
                f.moment_b.y += engine.pitch_moment();
                const auto gear_f = landing_gear.compute(s, mass);
                f.force_b = f.force_b + gear_f.force_b;
                f.moment_b = f.moment_b + gear_f.moment_b;
                return f;
            });
        const double state_agl = environment::GroundCollision::get_agl(state);
        const math::Vector3 v_rel_curr = wind_model.compute_relative_velocity(state.vel_b, state_agl, state.q_att);
        fdm::FlightState state_aero = state;
        state_aero.vel_b = v_rel_curr;
        current_forces = aero_model.compute_forces_and_moments(state_aero, surfaces, air.dynamic_pressure,
                                                               air.mach_number);
        current_forces.force_b.x += thrust_n;
        current_forces.moment_b.y += engine.pitch_moment();
        const auto col = environment::GroundCollision::check_collision(state, type, landing_gear.deployed,
                                                                       landing_gear.collapsed);
        if (col.has_collided) is_crashed = true;
        if (is_crashed) {
            environment::GroundCollision::clamp_to_surface(state);
            current_imu.Nz = 0.0;
        }
    }
};

bool same(const math::Vector3& a, const math::Vector3& b) {
    return std::memcmp(&a, &b, sizeof(math::Vector3)) == 0;
}

bool same(const fdm::FlightState& a, const fdm::FlightState& b) {
    return same(a.pos_ned, b.pos_ned) && same(a.vel_b, b.vel_b) && same(a.omega_b, b.omega_b) &&
           a.q_att.w == b.q_att.w && a.q_att.x == b.q_att.x && a.q_att.y == b.q_att.y && a.q_att.z == b.q_att.z;
}

using Script = std::function<sim::AircraftControls(double t)>;

void compare(AircraftType type, const fdm::FlightState& start, bool gear_down, const Script& script,
             const char* what) {
    ReferenceLoop ref(type);
    ref.state = start;
    ref.landing_gear.deployed = gear_down;

    // Through the World, exactly as the viewer now runs it (index 0).
    auto world = std::make_unique<sim::World>();
    world->clear();
    world->add(type, start);
    world->aircraft[0].landing_gear.deployed = gear_down;
    world->dispersion = false;

    std::array<sim::AircraftControls, sim::World::kMaxAircraft> controls{};
    const int steps = static_cast<int>(60.0 / DT);
    for (int i = 0; i < steps; ++i) {
        const double t = i * DT;
        const auto in = script(t);
        ref.step(in);
        controls[0] = in;
        controls[0].trigger = false;
        world->step(DT, controls);
        const sim::Aircraft& a = world->aircraft[0];
        if (!same(ref.state, a.state) || ref.is_crashed != a.crashed || ref.engine.fuel_kg != a.engine.fuel_kg ||
            ref.current_imu.Nz != a.imu.Nz) {
            std::cerr << "  divergence in " << what << " at t=" << t << " s\n";
            assert(false && "sim::Aircraft must be bit-identical to the original loop");
        }
    }
    const auto& a = world->aircraft[0];
    std::cout << "    " << aircraft::to_short_string(type) << " " << what << ": identical after 60 s (alt "
              << a.state.altitude() << " m, " << a.state.airspeed() << " m/s" << (a.crashed ? ", crashed" : "")
              << ")\n";
}

void test_airborne_manoeuvring_is_bit_identical() {
    std::cout << "[ENTITY] Airborne manoeuvring, speedbrake and throttle through sim::Aircraft...\n";
    fdm::FlightState s{};
    s.pos_ned = math::Vector3(0.0, 0.0, -1500.0);
    s.vel_b = math::Vector3(220.0, 0.0, 0.0);
    const Script script = [](double t) {
        sim::AircraftControls c{};
        c.throttle = t < 20.0 ? 0.65 : (t < 40.0 ? 1.0 : 0.3);
        c.stick.pitch_stick = (t > 5.0 && t < 9.0) ? 0.6 : (t > 30.0 && t < 33.0 ? -0.2 : 0.0);
        c.stick.roll_stick = (t > 12.0 && t < 13.5) ? 0.8 : (t > 25.0 && t < 26.0 ? -0.5 : 0.0);
        c.stick.rudder_pedal = (t > 18.0 && t < 19.0) ? 0.3 : 0.0;
        c.speedbrake_out = t > 42.0 && t < 50.0;
        return c;
    };
    // The F-35B in CTOL mode (STOVL never selected) is on the same pipeline.
    for (AircraftType t : {AircraftType::F16_FIGHTING_FALCON, AircraftType::F22_RAPTOR,
                           AircraftType::A10_THUNDERBOLT, AircraftType::F35B_LIGHTNING_II}) {
        compare(t, s, /*gear_down=*/false, script, "air work");
    }
    std::cout << "  -> PASSED\n";
}

void test_takeoff_roll_is_bit_identical() {
    std::cout << "[ENTITY] Runway takeoff roll with brakes, steering and gear through sim::Aircraft...\n";
    const auto cfg = aircraft::AircraftConfig::get(AircraftType::F16_FIGHTING_FALCON);
    const double uncompressed_h = cfg.gear.main_l_pos_b.z + cfg.gear.rest_length;
    const double static_weight = (cfg.mass.empty_mass_kg + cfg.mass.internal_fuel_capacity_kg) * 9.80665;
    const double compression = (static_weight * 0.85) * 0.5 / cfg.gear.main_spring_k;
    fdm::FlightState s{};
    s.pos_ned = math::Vector3(150.0, 0.0, -(uncompressed_h - std::clamp(compression, 0.05, cfg.gear.max_stroke * 0.8)));
    const Script script = [](double t) {
        sim::AircraftControls c{};
        c.throttle = t < 3.0 ? 0.08 : 1.0;
        c.brake_left = c.brake_right = t < 3.0 ? 1.0 : 0.0;
        c.stick.rudder_pedal = (t > 5.0 && t < 6.0) ? 0.2 : 0.0;
        c.stick.pitch_stick = (t > 22.0 && t < 30.0) ? 0.4 : 0.0;
        return c;
    };
    compare(AircraftType::F16_FIGHTING_FALCON, s, /*gear_down=*/true, script, "takeoff");
    std::cout << "  -> PASSED\n";
}

void test_undamaged_jet_ignores_damage_model() {
    std::cout << "[ENTITY] Damage hooks are exact identities on an undamaged jet...\n";
    sim::DamageState d{AircraftType::F15EX_EAGLE_II};
    aero::ControlSurfaces s{-3.25, 1.5, 0.75, 2.0, 0.0};
    const aero::ControlSurfaces before = s;
    d.apply_to_surfaces(s);
    assert(s.delta_e == before.delta_e && s.delta_a == before.delta_a && s.delta_r == before.delta_r);
    assert(d.thrust_factor() == 1.0);
    std::cout << "  -> PASSED\n";
}

} // namespace

int main() {
    std::cout << "=========================================================\n";
    std::cout << "  Aircraft Entity Extraction (Phase 0)                   \n";
    std::cout << "=========================================================\n";
    test_undamaged_jet_ignores_damage_model();
    test_airborne_manoeuvring_is_bit_identical();
    test_takeoff_roll_is_bit_identical();
    std::cout << "\nAll aircraft entity tests passed successfully!\n";
    return 0;
}
