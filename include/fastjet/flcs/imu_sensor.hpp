#pragma once

#include "../fdm/flight_state.hpp"
#include "../fdm/aircraft_forces.hpp"
#include "../fdm/mass_properties.hpp"
#include "../fdm/six_dof_fdm.hpp"
#include <cmath>

namespace fastjet {
namespace flcs {

/**
 * @brief Synthetic IMU and Air Data sensor package for FLCS feedback.
 * Zero dynamic memory allocation.
 */
struct IMUData {
    double Nx{0.0};           // Longitudinal acceleration load factor [G]
    double Ny{0.0};           // Lateral acceleration load factor [G]
    double Nz{1.0};           // Normal acceleration load factor [G] (+1.0 in level flight)
    double p_deg{0.0};        // Roll rate [deg/s]
    double q_deg{0.0};        // Pitch rate [deg/s]
    double r_deg{0.0};        // Yaw rate [deg/s]
    double alpha_deg{0.0};    // Angle of attack [deg]
    double beta_deg{0.0};     // Sideslip angle [deg]
    double q_bar{0.0};        // Dynamic pressure [Pa]
    double airspeed{0.0};     // True airspeed [m/s]
    double altitude{0.0};     // Geometric altitude [m]
    double gz{1.0};           // Gravity along body z (down), cos(theta) cos(phi) [G]: 1 wings level, -1 inverted
    double gy{0.0};           // Gravity along body y (right), cos(theta) sin(phi) [G]

    static IMUData read(
        const fdm::FlightState& state,
        const fdm::AircraftForces& aero_forces,
        const fdm::MassProperties& mass,
        double dynamic_pressure
    ) noexcept {
        constexpr double RAD_TO_DEG = 180.0 / M_PI;
        constexpr double G0 = fdm::SixDoFFDM::GRAVITY_ACCEL;

        IMUData data;
        data.airspeed = state.airspeed();
        data.altitude = state.altitude();
        data.q_bar = dynamic_pressure;

        data.p_deg = state.omega_b.x * RAD_TO_DEG;
        data.q_deg = state.omega_b.y * RAD_TO_DEG;
        data.r_deg = state.omega_b.z * RAD_TO_DEG;

        const math::Vector3 g_b = state.q_att.rotate_ned_to_body(math::Vector3(0.0, 0.0, 1.0));
        data.gz = g_b.z;
        data.gy = g_b.y;

        data.alpha_deg = state.alpha() * RAD_TO_DEG;
        data.beta_deg  = state.beta()  * RAD_TO_DEG;

        // Accelerations (load factor in G):
        if (mass.mass_kg > 0.0) {
            data.Nx =  aero_forces.force_b.x / (mass.mass_kg * G0);
            data.Ny =  aero_forces.force_b.y / (mass.mass_kg * G0);
            data.Nz = -aero_forces.force_b.z / (mass.mass_kg * G0);
        } else {
            data.Nx = 0.0;
            data.Ny = 0.0;
            data.Nz = 1.0;
        }

        return data;
    }
};

} // namespace flcs
} // namespace fastjet
