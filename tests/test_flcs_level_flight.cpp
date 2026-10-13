#include "../include/fastjet/flcs/f16_flcs.hpp"
#include "../include/fastjet/aero/f16_aero_model.hpp"
#include "../include/fastjet/environment/atmosphere1976.hpp"
#include "../include/fastjet/fdm/rk4_integrator.hpp"
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace fastjet::math;
using namespace fastjet::fdm;
using namespace fastjet::environment;
using namespace fastjet::aero;
using namespace fastjet::flcs;

void test_hands_off_level_flight() {
    std::cout << "[Test] Hands-Off Stable Level Flight (60s / 12,000 steps with FLCS active)...\n";

    const MassProperties mass = MassProperties::create_clean_f16();
    const RK4Integrator integrator(0.005); // 200 Hz

    F16FLCS flcs;
    // Initial trim elevator ~ +0.67 deg to balance natural pitching moment
    flcs.reset(0.67, 0.0, 0.0);
    // The pitch law's trim feed-forward (PitchTrimModel) carries the 1 G trim,
    // so the integrator starts empty: pre-loading it with the trim as well
    // doubled it and knocked the jet 0.2 G off at the start.

    constexpr double ALTITUDE = 5000.0;
    constexpr double AIRSPEED = 200.0;
    constexpr double alpha_trim_deg = 2.5;
    constexpr double alpha_trim_rad = alpha_trim_deg * (M_PI / 180.0);

    // Initial state
    FlightState state;
    state.pos_ned = Vector3(0.0, 0.0, -ALTITUDE);
    state.vel_b = Vector3(AIRSPEED * std::cos(alpha_trim_rad), 0.0, AIRSPEED * std::sin(alpha_trim_rad));
    state.omega_b = Vector3::zero();
    state.q_att = Quaternion::from_euler(0.0, alpha_trim_rad, 0.0);

    const PilotCommands pilot_hands_off(0.0, 0.0, 0.0);

    double time = 0.0;
    constexpr double DURATION = 60.0; // 60 seconds
    const int total_steps = static_cast<int>(DURATION / integrator.dt);

    double max_nz_err = 0.0;
    double max_q_rate = 0.0;
    double min_alt = state.altitude();
    double max_alt = state.altitude();

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  Simulating 60 seconds of hands-off flight at 200 Hz...\n";
    std::cout << "  ----------------------------------------------------------------------\n";
    std::cout << "  Time [s] | Altitude [m] | Airspeed [m/s] | Alpha [deg] | Nz [G] | Elev [deg]\n";
    std::cout << "  ----------------------------------------------------------------------\n";

    // Engine thrust to balance cruise drag (~18.5 kN)
    constexpr double THRUST_N = 18500.0;

    for (int step = 0; step < total_steps; ++step) {
        const AirData ad = Atmosphere1976::compute(state.altitude(), state.airspeed());

        // 1. Evaluate current aerodynamic forces for sensor measurement
        ControlSurfaces current_surfaces(flcs.stabilator.position, flcs.flaperon.position, flcs.rudder.position);
        AircraftForces current_aero = F16AeroModel::compute_forces_and_moments(state, current_surfaces, ad.dynamic_pressure);

        // 2. Discrete 200 Hz FLCS update (actuator advance + control laws)
        const ControlSurfaces commanded_surfaces = flcs.update(
            integrator.dt, state, pilot_hands_off, ad.dynamic_pressure, current_aero, mass
        );

        // 3. Continuous 6-DoF RK4 integration holding actuator positions over dt
        integrator.step(state, time, mass, [&](double, const FlightState& s) noexcept -> AircraftForces {
            const AirData ad_stage = Atmosphere1976::compute(s.altitude(), s.airspeed());
            AircraftForces aero = F16AeroModel::compute_forces_and_moments(s, commanded_surfaces, ad_stage.dynamic_pressure);
            aero.force_b.x += THRUST_N;
            return aero;
        });

        // Track metrics
        const AirData ad_post = Atmosphere1976::compute(state.altitude(), state.airspeed());
        const AircraftForces aero_post = F16AeroModel::compute_forces_and_moments(state, commanded_surfaces, ad_post.dynamic_pressure);
        const IMUData imu = IMUData::read(state, aero_post, mass, ad_post.dynamic_pressure);

        const double nz_err = std::abs(imu.Nz - 1.0);
        if (nz_err > max_nz_err) max_nz_err = nz_err;
        if (std::abs(imu.q_deg) > max_q_rate) max_q_rate = std::abs(imu.q_deg);
        if (state.altitude() < min_alt) min_alt = state.altitude();
        if (state.altitude() > max_alt) max_alt = state.altitude();

        if ((step + 1) % 2000 == 0) { // Every 10s
            std::cout << "  " << std::setw(6) << time
                      << "  |  " << std::setw(10) << state.altitude()
                      << "  |  " << std::setw(12) << state.airspeed()
                      << "  |  " << std::setw(9) << imu.alpha_deg
                      << "  |  " << std::setw(6) << imu.Nz
                      << "  |  " << std::setw(8) << flcs.stabilator.position << "\n";
        }
    }
    std::cout << "  ----------------------------------------------------------------------\n";

    std::cout << "  Observed max_nz_err = " << max_nz_err << " G\n";
    assert(max_nz_err < 0.15);
    // 2. Pitch rate is well damped
    assert(max_q_rate < 0.5);
    // 3. Altitude drift is bounded (< 150 m over 60s / 13 km flight without outer-loop altitude-hold autopilot)
    assert((max_alt - min_alt) < 150.0);
    // 4. Roll and yaw angles remain zero
    assert(std::abs(state.euler_angles().roll) < 0.01);
    assert(std::abs(state.euler_angles().yaw) < 0.01);

    std::cout << "  HANDS-OFF STABILITY VERIFIED:\n"
              << "  - 60.0 seconds / 12,000 steps completed with zero divergence\n"
              << "  - Max Nz Error:  " << max_nz_err << " G\n"
              << "  - Max Pitch Rate:" << max_q_rate << " deg/s\n"
              << "  - Altitude Range: [" << min_alt << ", " << max_alt << "] m (Delta = "
              << (max_alt - min_alt) << " m)\n"
              << "  - FLCS successfully stabilizes the open-loop unstable F-16 airframe!\n\n";
}

int main() {
    std::cout << "=== FLCS Hands-Off Level Flight Stability Verification ===\n";
    test_hands_off_level_flight();
    std::cout << "All Level Flight tests passed successfully!\n\n";
    return 0;
}
