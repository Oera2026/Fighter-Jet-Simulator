#pragma once

#include "fastjet/aircraft/aircraft_type.hpp"
#include "fastjet/aircraft/aircraft_config.hpp"
#include "fastjet/aero/aircraft_aero_model.hpp"
#include "fastjet/environment/atmosphere1976.hpp"
#include "fastjet/environment/ground_collision.hpp"
#include "fastjet/environment/wind_turbulence.hpp"
#include "fastjet/fdm/fuel_system.hpp"
#include "fastjet/fdm/rk4_integrator.hpp"
#include "fastjet/flcs/aircraft_flcs.hpp"
#include "fastjet/flcs/imu_sensor.hpp"
#include "fastjet/flcs/onboard_flight_computer.hpp"
#include "fastjet/flcs/pilot_commands.hpp"
#include "fastjet/flcs/stovl_control_law.hpp"
#include "fastjet/gear/landing_gear.hpp"
#include "fastjet/input/avionics_controls.hpp"
#include "fastjet/propulsion/multi_engine.hpp"
#include "fastjet/propulsion/stovl_lift_system.hpp"
#include "fastjet/propulsion/stovl_status.hpp"
#include "fastjet/sim/damage_model.hpp"
#include "fastjet/sim/gun.hpp"
#include <algorithm>
#include <cstdint>

namespace fastjet::sim {

/// @brief Everything a pilot (human or AI) puts into the cockpit for one step.
///
/// A human pilot and the AI produce exactly this struct, and both go through
/// the same OFC, FLCS, engine and integrator. No pilot has a path around the
/// airframe's limits.
struct AircraftControls {
    flcs::PilotCommands stick{};   ///< Raw inceptors, before the OFC
    double throttle{0.0};          ///< Lever position [0, 1]
    bool speedbrake_out{false};    ///< Speedbrake switch
    double brake_left{0.0};        ///< Toe brakes [0, 1]
    double brake_right{0.0};
    bool trigger{false};           ///< Gun trigger held
    bool pickle{false};            ///< Missile release / pickle button pressed
    bool dispense{false};          ///< Countermeasure dispense switch (chaff: one program per press)
    bool dispense_flare{false};    ///< Countermeasure dispense switch (flares: one program per press)
    bool eject{false};             ///< Ejection handle pulled
    bool stovl_mode{false};        ///< STOVL/CTOL button state: true requests STOVL (F-35B only)
};

/**
 * @brief One jet: the complete per-aircraft pipeline that used to live in the
 * viewer's main loop.
 *
 * Each step runs IMU -> OFC -> FLCS -> speedbrake -> engine -> fuel/mass ->
 * gear -> turbulence -> RK4 -> ground collision, in that order and with the
 * same arithmetic as the original loop, so a single ownship flies
 * bit-identically through it (see tests/test_aircraft_entity.cpp).
 *
 * Every jet carries its own turbulence state: gusts are felt where the
 * aircraft is, not shared across the sky.
 *
 * The F-35B adds its lift system: while it is engaged (converting or in
 * STOVL) the unified STOVL control law flies the jet in place of the FLCS's
 * up-and-away law, and the lift system's forces and moments replace the plain
 * thrust along body X.
 *
 * Zero heap allocation; safe in the 200 Hz loop.
 */
class Aircraft {
public:
    static constexpr double DT = fdm::RK4Integrator::DEFAULT_DT;

    aircraft::AircraftType type{aircraft::AircraftType::F16_FIGHTING_FALCON};

    fdm::FlightState state{};
    fdm::FlightState prev_state{};        ///< State before the latest step (render interpolation, hit sweeps)
    fdm::MassProperties mass{};
    fdm::AircraftForces forces{};         ///< Forces at the current state (IMU / HUD)
    flcs::IMUData imu{};

    propulsion::MultiEngine engine;
    gear::LandingGear landing_gear;
    aero::AircraftAeroModel aero_model;
    flcs::AircraftFLCS flcs;
    flcs::OnBoardFlightComputer ofc;
    input::SpeedbrakeController speedbrake{};
    environment::WindTurbulenceModel wind{};
    GunSystem gun{};
    DamageState damage{};
    propulsion::StovlStatus stovl{};      ///< Lift-system snapshot after the latest step (read-only for consumers)
    propulsion::StovlLiftSystem lift;     ///< F-35B LiftFan, 3BSM and roll posts (unequipped on other jets)
    flcs::StovlControlLaw stovl_law;      ///< F-35B unified STOVL law

    bool crashed{false};
    double time{0.0};

    Aircraft() noexcept : Aircraft(aircraft::AircraftType::F16_FIGHTING_FALCON) {}

    explicit Aircraft(aircraft::AircraftType t) noexcept
        : type(t), engine(t), landing_gear(t), aero_model(t), flcs(t), ofc(t), lift(t), stovl_law(t) {
        refresh_mass();
        wind.base_wind_speed_mps = 5.0;
        gun.configure(t);
        damage = DamageState{t};
        imu.Nz = 1.0;
    }

    /// @brief Adopt a different airframe. Fuel is carried across as a fraction
    /// (an in-flight change must not refuel the jet).
    void configure(aircraft::AircraftType t) noexcept {
        type = t;
        engine.configure(t, /*preserve_fuel=*/true);
        aero_model.configure(t);
        flcs.configure(t);
        ofc.configure(t);
        landing_gear.configure(t);
        lift.configure(t);
        stovl_law.configure(t);
        stovl = propulsion::StovlStatus{};
        refresh_mass();
        gun.configure(t);
        damage = DamageState{t};
    }

    /// @brief Fresh jet at @p start: full fuel, systems reset, undamaged.
    void reset(const fdm::FlightState& start) noexcept {
        state = start;
        prev_state = start;
        crashed = false;
        engine.reset();
        landing_gear.reset(type);
        ofc.reset();
        flcs.reset();
        speedbrake.position = 0.0;
        refresh_mass();
        forces = fdm::AircraftForces{};
        imu = flcs::IMUData{};
        imu.Nz = 1.0;
        gun.configure(type);
        damage = DamageState{type};
        lift.reset();
        stovl_law.reset();
        stovl = propulsion::StovlStatus{};
        aero_force_b_ = math::Vector3::zero();
    }

    /// @brief Put an F-35B straight into the converted STOVL configuration
    /// (doors open, LiftFan clutched, nozzle down) for hover and approach starts.
    /// No effect on jets without a lift system. Call after reset().
    ///
    /// @param fuel_kg Fuel to start with, or negative to keep what is on board.
    ///        A jet-borne start needs a light jet: with the combat payload the
    ///        F-35B hovers out of ground effect up to ~2,300 kg of fuel and has
    ///        a useful control margin below ~1,600 kg.
    ///
    /// The engine starts at hover thrust for the jet's weight, so a hover start
    /// (zero velocity, ~3 deg nose up) holds where it is placed; the throttle
    /// lever should sit at kStovlThrottleDetent.
    void start_converted_stovl(double fuel_kg = -1.0) noexcept {
        if (!lift.config.equipped) return;
        if (fuel_kg >= 0.0) {
            engine.fuel_kg = std::min(fuel_kg, aircraft::AircraftConfig::get(type).mass.internal_fuel_capacity_kg);
            refresh_mass();
        }
        lift.start_converted();
        stovl_law.reset();
        engine.lift_fan_clutch = 1.0;
        engine.stovl_mode = true;
        engine.ab_thrust_n = 0.0;
        engine.core_thrust_n = std::min(mass.mass_kg * fdm::SixDoFFDM::GRAVITY_ACCEL, engine.max_dry_rating_n());
        engine.net_thrust_n = engine.core_thrust_n;
        refresh_stovl_status(engine.net_thrust_n, true,
                             environment::GroundCollision::get_agl(state) - lift.config.nozzle_pos_b.z,
                             environment::Atmosphere1976::compute(state.altitude(), state.airspeed()));
    }

    /// @brief Rebuild mass and inertia from the fuel on board and the combat payload.
    void refresh_mass() noexcept {
        mass = fdm::FuelSystem::compute(engine.fuel_kg, type,
                                        aircraft::AircraftConfig::get(type).mass.combat_payload_kg);
    }

    /// @brief Re-seed this jet's turbulence so jets in one sky do not share gusts.
    void seed_turbulence(uint32_t seed) noexcept { wind.seed(seed); }

    /// @brief Still in the fight: flying, intact and with a conscious pilot at the controls.
    [[nodiscard]] bool alive() const noexcept {
        return !crashed && !damage.destroyed && !damage.pilot_incapacitated && !damage.pilot_ejected;
    }

    /// @brief One fixed 200 Hz physics step.
    void step(double dt, const AircraftControls& in) noexcept {
        prev_state = state;
        if (crashed) return;

        // A destroyed jet has no pilot on the controls and no working engine;
        // it falls out of the sky on its own aerodynamics.
        damage.update(dt);
        AircraftControls cmd = in;
        if (damage.destroyed || damage.pilot_incapacitated || damage.pilot_ejected) {
            cmd.stick = {};
            cmd.trigger = false;
        }

        const auto air = environment::Atmosphere1976::compute(state.altitude(), state.airspeed());
        imu = flcs::IMUData::read(state, forces, mass, air.dynamic_pressure);

        // OFC intercept: overrides the pilot while GLOC or Auto-GCAS is active.
        const bool lift_on = lift.engaged();
        const flcs::PilotCommands effective =
            ofc.update(dt, state, imu, cmd.stick, landing_gear.deployed, lift_on);

        aero::ControlSurfaces surfaces{};
        double lever = cmd.throttle;
        propulsion::StovlEffectorCommands lift_cmd{};
        bool ctol_ok = true;
        if (lift_on) {
            const double agl_now = environment::GroundCollision::get_agl(state);
            const double wheel_depth = landing_gear.struts[1].pos_b.z + landing_gear.struts[1].rest_length;
            const auto law = stovl_law.update(
                dt, state, imu, effective, cmd.throttle, mass, aero_force_b_,
                wind.compute_relative_velocity(state.vel_b, agl_now, state.q_att), air,
                agl_now - wheel_depth, landing_gear.mains_on_ground(), lift, engine);
            surfaces = flcs.drive_surfaces(dt, law.de_deg, law.da_deg, law.dr_deg, imu.Nz);
            lever = law.lever;
            lift_cmd = law.lift;
            ctol_ok = law.ctol_ok;
        } else {
            surfaces = flcs.update(dt, state, effective, air.dynamic_pressure, forces, mass);
        }
        damage.apply_to_surfaces(surfaces);

        speedbrake.update(cmd.speedbrake_out ? input::SpeedbrakeController::SwitchPosition::EXTEND
                                             : input::SpeedbrakeController::SwitchPosition::RETRACT,
                          dt);
        surfaces.speedbrake = speedbrake.position;

        // Auto-GCAS throttle authority overrides the lever.
        if (ofc.has_throttle_override()) lever = ofc.throttle_override_value();
        engine.tvc_pitch_cmd = flcs.tvc_pitch_cmd;

        // Conversion sequencer and lift effectors; the engine follows the clutch.
        if (lift.config.equipped) {
            const double kcas = std::sqrt(2.0 * air.dynamic_pressure / environment::Atmosphere1976::RHO0) * 1.94384;
            lift.update(dt, cmd.stovl_mode, kcas, ctol_ok, lift_cmd);
            engine.lift_fan_clutch = lift.clutch;
            engine.stovl_mode = lift.engaged();
        }
        const double thrust_n = engine.update(lever, air, dt) * damage.thrust_factor();
        if (damage.fuel_leak_kgs > 0.0) {
            engine.fuel_kg = std::max(0.0, engine.fuel_kg - damage.fuel_leak_kgs * dt);
        }
        refresh_mass();

        landing_gear.steer_cmd = cmd.stick.rudder_pedal;
        landing_gear.brake_left = cmd.brake_left;
        landing_gear.brake_right = cmd.brake_right;

        const double agl = environment::GroundCollision::get_agl(state);
        wind.update(dt, agl, state.airspeed());

        // RK4 step with forces re-evaluated at each stage, in the air-relative frame.
        const double pitch_moment = engine.pitch_moment();
        const bool jets_on_lift = lift.engaged();
        const double dry_rating = engine.max_dry_rating_n();
        const double exit_depth = lift.config.nozzle_pos_b.z;
        integrator_.step(state, time, mass,
            [&](double, const fdm::FlightState& s) noexcept -> fdm::AircraftForces {
                const double s_agl = environment::GroundCollision::get_agl(s);
                const math::Vector3 v_rel = wind.compute_relative_velocity(s.vel_b, s_agl, s.q_att);
                const double v_rel_norm = (std::max)(1.0, v_rel.norm());
                const auto stage_air = environment::Atmosphere1976::compute(s.altitude(), v_rel_norm);

                fdm::FlightState s_aero = s;
                s_aero.vel_b = v_rel;
                fdm::AircraftForces f = aero_model.compute_forces_and_moments(
                    s_aero, surfaces, stage_air.dynamic_pressure, stage_air.mach_number);
                if (jets_on_lift) {
                    const math::Vector3 v_ned = s.velocity_ned();
                    const auto jet = lift.compute(thrust_n, dry_rating, v_rel, s.q_att, s_agl - exit_depth,
                                                  std::hypot(v_ned.x, v_ned.y));
                    f.force_b = f.force_b + jet.force_b;
                    f.moment_b = f.moment_b + jet.moment_b;
                } else {
                    f.force_b.x += thrust_n;
                    f.moment_b.y += pitch_moment;
                }

                const auto gear_f = landing_gear.compute(s, mass);
                f.force_b = f.force_b + gear_f.force_b;
                f.moment_b = f.moment_b + gear_f.moment_b;
                return f;
            });

        // Forces at the new state, for the next IMU read and the HUD.
        const double state_agl = environment::GroundCollision::get_agl(state);
        fdm::FlightState state_aero = state;
        state_aero.vel_b = wind.compute_relative_velocity(state.vel_b, state_agl, state.q_att);
        forces = aero_model.compute_forces_and_moments(state_aero, surfaces, air.dynamic_pressure, air.mach_number);
        aero_force_b_ = forces.force_b;
        if (jets_on_lift) {
            const math::Vector3 v_ned = state.velocity_ned();
            const double gs = std::hypot(v_ned.x, v_ned.y);
            const auto jet = lift.compute(thrust_n, dry_rating, state_aero.vel_b, state.q_att,
                                          state_agl - exit_depth, gs, &stovl);
            forces.force_b = forces.force_b + jet.force_b;
            forces.moment_b = forces.moment_b + jet.moment_b;
        } else {
            forces.force_b.x += thrust_n;
            forces.moment_b.y += engine.pitch_moment();
        }
        if (lift.config.equipped) refresh_stovl_status(thrust_n, jets_on_lift, state_agl - exit_depth, air);

        // Structural load from the air loads (the landing gear has its own limits).
        damage.apply_load(-forces.force_b.z / (mass.mass_kg * fdm::SixDoFFDM::GRAVITY_ACCEL), dt);

        const auto col = environment::GroundCollision::check_collision(
            state, type, landing_gear.deployed, landing_gear.collapsed);
        if (col.has_collided) {
            crashed = true;
            environment::GroundCollision::clamp_to_surface(state);
            imu.Nz = 0.0;
        }
    }

    // ---------------------------------------------------------------------
    // Geometry helpers used by weapons, sensors and the AI
    // ---------------------------------------------------------------------

    [[nodiscard]] math::Vector3 position() const noexcept { return state.pos_ned; }
    [[nodiscard]] math::Vector3 velocity() const noexcept { return state.velocity_ned(); }

    /// @brief Inertial acceleration [m/s^2, NED] from the latest forces (gravity included).
    [[nodiscard]] math::Vector3 acceleration() const noexcept {
        const math::Vector3 specific_b = forces.force_b / std::max(1.0, mass.mass_kg);
        return state.q_att.rotate_body_to_ned(specific_b) + math::Vector3(0.0, 0.0, fdm::SixDoFFDM::GRAVITY_ACCEL);
    }

private:
    fdm::RK4Integrator integrator_{DT};
    math::Vector3 aero_force_b_{};   ///< Aerodynamic force at the latest step (the STOVL law's estimate)

    void refresh_stovl_status(double thrust_n, bool jets_on_lift, double exit_height_m,
                              const environment::AirData& air) noexcept {
        lift.fill_status(stovl);
        if (!jets_on_lift) {
            stovl.lift_fan_thrust_n = stovl.roll_post_left_n = stovl.roll_post_right_n = stovl.jet_lift_n = 0.0;
            stovl.nozzle_thrust_n = thrust_n;
        }
        const math::Vector3 v_ned = state.velocity_ned();
        stovl.hover_thrust_margin = lift.max_jet_lift_n(engine.lapse_factor(air, false), exit_height_m,
                                                        std::hypot(v_ned.x, v_ned.y))
                                  / (mass.mass_kg * fdm::SixDoFFDM::GRAVITY_ACCEL);
        stovl.control_phase = jets_on_lift ? stovl_law.control_phase : 0;
        stovl.commanded_vs_mps = jets_on_lift ? stovl_law.commanded_vs_mps : 0.0;
        stovl.commanded_gs_mps = jets_on_lift ? stovl_law.commanded_gs_mps : 0.0;
    }
};

} // namespace fastjet::sim
