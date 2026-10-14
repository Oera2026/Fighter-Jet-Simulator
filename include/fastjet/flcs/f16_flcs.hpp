#pragma once

#include "actuator.hpp"
#include "pilot_commands.hpp"
#include "imu_sensor.hpp"
#include "pitch_controller.hpp"
#include "pitch_trim_model.hpp"
#include "lateral_directional_controller.hpp"
#include "../aero/control_surfaces.hpp"
#include "../fdm/flight_state.hpp"
#include "../fdm/aircraft_forces.hpp"
#include "../fdm/mass_properties.hpp"

namespace fastjet {
namespace flcs {

/**
 * @brief Integrated F-16 Digital Fly-By-Wire Flight Control System (FLCS).
 * Manages full 3-axis control laws, G & AoA limiters, and hydraulic actuators.
 * Zero dynamic heap allocation.
 */
class F16FLCS {
public:
    // Physical Actuators
    Actuator stabilator{Actuator::create_stabilator(0.0)};
    Actuator flaperon{Actuator::create_flaperon(0.0)};
    Actuator rudder{Actuator::create_rudder(0.0)};

    // Axis Controllers
    PitchController pitch_ctrl{};
    LateralDirectionalController lat_dir_ctrl{};
    PitchTrimModel trim_model{aircraft::AircraftType::F16_FIGHTING_FALCON};

    F16FLCS() noexcept = default;

    void reset(double init_de = 0.0, double init_da = 0.0, double init_dr = 0.0) noexcept {
        stabilator.reset(init_de);
        flaperon.reset(init_da);
        rudder.reset(init_dr);
        pitch_ctrl.reset();
        lat_dir_ctrl.reset();
        trim_model.reset();
    }

    /**
     * @brief Computes automatic Leading-Edge Flap (LEF) schedule vs alpha and dynamic pressure.
     * Standard F-16 schedule: delta_lef = 1.38 * alpha - 9.05 * (q_bar / P0) + 1.45, clamped [0, 25] deg.
     */
    [[nodiscard]] static constexpr double compute_lef_schedule(double alpha_deg, double q_bar) noexcept {
        constexpr double P0 = 101325.0; // Sea level pressure
        const double q_ratio = q_bar / P0;
        const double lef_deg = 1.38 * alpha_deg - 9.05 * q_ratio + 1.45;
        return std::clamp(lef_deg, 0.0, 25.0);
    }

    /**
     * @brief Main FLCS update loop called every simulation timestep (e.g. 200 Hz).
     *
     * @param dt Timestep in seconds
     * @param state Current 6-DoF flight state
     * @param pilot Cockpit inceptor commands
     * @param dynamic_pressure Ambient dynamic pressure [Pa]
     * @param aero_forces Current aerodynamic forces
     * @param mass Mass and inertia properties
     * @return aero::ControlSurfaces Physical surface deflections after actuator lag and limits
     */
    aero::ControlSurfaces update(
        double dt,
        const fdm::FlightState& state,
        const PilotCommands& pilot,
        double dynamic_pressure,
        const fdm::AircraftForces& aero_forces,
        const fdm::MassProperties& mass
    ) noexcept {
        // 1. Ingest IMU sensor measurements
        const IMUData imu = IMUData::read(state, aero_forces, mass, dynamic_pressure);

        // 2. Pitch Axis Control Law
        const bool is_sat = stabilator.is_pos_limited || stabilator.is_rate_limited;
        const double de_cmd = pitch_ctrl.update(dt, pilot, imu, is_sat, &trim_model, mass.mass_kg,
                                                &compute_lef_schedule, inertial_pitch_moment(imu, mass));

        // 3. Roll & Yaw Axes Control Laws
        double da_cmd{0.0};
        double dr_cmd{0.0};
        lat_dir_ctrl.update(dt, pilot, imu, da_cmd, dr_cmd, mass.Izz, mass.mass_kg, rudder.position);

        // 4. Step Hydraulic Actuators (Lag filter + rate limits + deflection limits)
        const double de_actual = stabilator.step(de_cmd, dt);
        const double da_actual = flaperon.step(da_cmd, dt);
        const double dr_actual = rudder.step(dr_cmd, dt);

        // 5. Automatic Leading-Edge Flap schedule
        const double dlef_actual = compute_lef_schedule(imu.alpha_deg, dynamic_pressure);

        return {de_actual, da_actual, dr_actual, dlef_actual};
    }
};

} // namespace flcs
} // namespace fastjet
