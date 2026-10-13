/**
 * @file test_multi_aircraft_performance.cpp
 * @brief Dynamic flight testing and performance envelope validation across
 * all 5 combat aircraft (F-16C, F-15EX, Typhoon, F-22A, A-10C).
 */
#include "fastjet/aircraft/aircraft_type.hpp"
#include "fastjet/aircraft/aircraft_config.hpp"
#include "fastjet/fdm/rk4_integrator.hpp"
#include "fastjet/fdm/fuel_system.hpp"
#include "fastjet/environment/atmosphere1976.hpp"
#include "fastjet/aero/aircraft_aero_model.hpp"
#include "fastjet/flcs/aircraft_flcs.hpp"
#include "fastjet/propulsion/multi_engine.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace fastjet;
using namespace fastjet::aircraft;

namespace {

constexpr double DT = 0.005;

struct FlightSimHarness {
    AircraftType type;
    fdm::FlightState state{};
    fdm::MassProperties mass{};
    fdm::RK4Integrator integrator{DT};
    aero::AircraftAeroModel aero{};
    flcs::AircraftFLCS flcs{};
    propulsion::MultiEngine engine{};
    fdm::AircraftForces forces{};
    double sim_time{0.0};

    explicit FlightSimHarness(AircraftType t) : type(t), aero(t), flcs(t), engine(t) {
        mass = fdm::FuelSystem::compute(engine.fuel_kg, t);
    }

    void trim_level(double alt_m, double airspeed_mps) {
        state.pos_ned = math::Vector3(0.0, 0.0, -alt_m);
        state.vel_b = math::Vector3(airspeed_mps, 0.0, 0.0);
        state.omega_b = math::Vector3::zero();
        state.q_att = math::Quaternion::identity();
    }

    /// @brief Run the engine up to @p throttle at the current flight condition
    /// (a jet flying at speed has its engine running; the core takes seconds
    /// to spool and the afterburner lights only once it has).
    void prespool(double throttle) {
        const auto air = environment::Atmosphere1976::compute(state.altitude(), state.airspeed());
        for (int i = 0; i < 3000; ++i) (void)engine.update(throttle, air, DT);
    }

    void step(double throttle, const flcs::PilotCommands& cmd) {
        const auto air = environment::Atmosphere1976::compute(state.altitude(), state.airspeed());
        const double thrust = engine.update(throttle, air, DT);
        engine.tvc_pitch_cmd = flcs.tvc_pitch_cmd;

        const auto surfaces = flcs.update(DT, state, cmd, air.dynamic_pressure, forces, mass);
        mass = fdm::FuelSystem::compute(engine.fuel_kg, type);

        forces = aero.compute_forces_and_moments(state, surfaces, air.dynamic_pressure, air.mach_number);
        forces.force_b.x += thrust;
        forces.moment_b.y += engine.pitch_moment();

        integrator.step(state, sim_time, mass,
            [&](double, const fdm::FlightState& s) noexcept -> fdm::AircraftForces {
                const auto stage_air = environment::Atmosphere1976::compute(s.altitude(), s.airspeed());
                auto f = aero.compute_forces_and_moments(s, surfaces, stage_air.dynamic_pressure, stage_air.mach_number);
                f.force_b.x += thrust;
                f.moment_b.y += engine.pitch_moment();
                return f;
            });
    }
};

void test_f15ex_high_speed_dash() {
    std::cout << "[PERF] Testing F-15EX Eagle II high-speed envelope...\n";
    FlightSimHarness sim(AircraftType::F15EX_EAGLE_II);
    // Trim at 12,000 m (tropopause) at Mach 1.5
    const auto air = environment::Atmosphere1976::compute(12000.0, 440.0);
    sim.trim_level(12000.0, 440.0);
    sim.prespool(1.0);

    // Full afterburner acceleration
    flcs::PilotCommands cmd{};
    for (int i = 0; i < 2000; ++i) { // 10 seconds of flight
        sim.step(1.0, cmd);
    }

    // F-15EX twin F110s deliver massive excess thrust: aircraft accelerates rapidly
    const double final_mach = sim.state.airspeed() / air.speed_of_sound;
    assert(sim.engine.net_thrust_n > 100000.0); // High net thrust at altitude
    assert(final_mach > 1.50);
    std::cout << "  -> F-15EX high-speed dash passed (Mach: " << final_mach << ").\n";
}

void test_typhoon_supercruise() {
    std::cout << "[PERF] Testing Eurofighter Typhoon dry supercruise...\n";
    FlightSimHarness sim(AircraftType::EUROFIGHTER_TYPHOON);
    // Trim at 11,000 m at Mach 1.40
    const auto air = environment::Atmosphere1976::compute(11000.0, 413.0);
    sim.trim_level(11000.0, 413.0);

    // Military (dry) power: lever at 0.85 (no afterburner)
    flcs::PilotCommands cmd{};
    for (int i = 0; i < 2000; ++i) {
        sim.step(0.85, cmd);
    }

    assert(sim.engine.ab_power == 0.0); // Confirm dry power
    const double mach = sim.state.airspeed() / air.speed_of_sound;
    // Typhoon sustains supersonic flight on dry power alone
    assert(mach > 1.30);
    std::cout << "  -> Typhoon dry supercruise passed (Mach: " << mach << ").\n";
}

void test_f22_thrust_vectoring() {
    std::cout << "[PERF] Testing F-22A Raptor 2D thrust vectoring pitch authority...\n";
    FlightSimHarness sim(AircraftType::F22_RAPTOR);
    // Low-speed high-altitude regime: 80 m/s at 5,000 m
    sim.trim_level(5000.0, 80.0);

    flcs::PilotCommands cmd{};
    cmd.pitch_stick = -1.0; // Command aggressive pitch up

    for (int i = 0; i < 200; ++i) { // 1.0 second
        sim.step(0.85, cmd);
    }

    // Verify TVC was commanded and generated substantial pitching moment
    assert(sim.engine.config.has_thrust_vectoring == true);
    assert(std::abs(sim.flcs.tvc_pitch_cmd) > 0.3);
    assert(std::abs(sim.engine.pitch_moment()) > 10000.0); // Powerful TVC pitch moment
    std::cout << "  -> F-22A TVC pitch authority verified (Moment: "
              << sim.engine.pitch_moment() << " N*m).\n";
}

void test_a10_nacelle_pitch_coupling() {
    std::cout << "[PERF] Testing A-10C high-mounted nacelle pitch coupling...\n";
    FlightSimHarness sim(AircraftType::A10_THUNDERBOLT);
    sim.trim_level(1500.0, 150.0); // 150 m/s (~300 knots)

    // At idle, thrust is low
    flcs::PilotCommands cmd{};
    for (int i = 0; i < 200; ++i) sim.step(0.08, cmd);
    const double idle_moment = sim.engine.pitch_moment();

    // Slam throttle to maximum dry power (1.0)
    for (int i = 0; i < 400; ++i) sim.step(1.0, cmd);
    const double full_moment = sim.engine.pitch_moment();

    // A-10's high engine placement creates a nose-down moment on throttle application
    // (negative My in FastJet body axes where pitch-up is positive)
    assert(sim.engine.config.thrust_z_offset_m < 0.0);
    assert(full_moment < idle_moment); // Substantial nose-down moment from high thrust
    std::cout << "  -> A-10C pitch-thrust coupling verified (Nacelle moment: "
              << full_moment << " N*m).\n";
}

void test_a10_subsonic_drag_barrier() {
    std::cout << "[PERF] Testing A-10C transonic wave drag barrier...\n";
    aero::AircraftAeroModel aero(AircraftType::A10_THUNDERBOLT);
    fdm::FlightState s{};
    aero::ControlSurfaces ctrl{};

    // Below critical Mach (M = 0.50): drag is normal
    const auto c_sub = aero.compute_coefficients(s, ctrl, 20000.0, 0.50);
    // Transonic (M = 0.80): drag climbs steeply on the thick straight wing
    const auto c_trans = aero.compute_coefficients(s, ctrl, 20000.0, 0.80);

    assert(c_trans.CD > c_sub.CD * 2.5);
    std::cout << "  -> A-10C subsonic barrier passed (CD at M0.5: "
              << c_sub.CD << ", CD at M0.8: " << c_trans.CD << ").\n";
}

void test_lift_continuous_through_transonic() {
    std::cout << "[PERF] Testing lift stays continuous from M0.70 to M1.60 on every airframe...\n";
    // The compressibility factor used to switch to a bounded Ackeret branch just
    // past M1.05, multiplying lift by up to 2.84 and then dropping it back to 1.0
    // at each airframe's mach_peak: 10-15 G spikes on anything flown near M1.1.
    fdm::FlightState s{};
    const double a = 3.0 * M_PI / 180.0;
    s.vel_b = math::Vector3(300.0 * std::cos(a), 0.0, 300.0 * std::sin(a));
    const aero::ControlSurfaces ctrl{};

    for (AircraftType t : {AircraftType::F16_FIGHTING_FALCON, AircraftType::F15EX_EAGLE_II,
                           AircraftType::EUROFIGHTER_TYPHOON, AircraftType::F22_RAPTOR,
                           AircraftType::A10_THUNDERBOLT, AircraftType::F35B_LIGHTNING_II}) {
        const aero::AircraftAeroModel aero(t);
        double prev_cl = aero.compute_coefficients(s, ctrl, 20000.0, 0.70).CL;
        double worst = 0.0;
        for (double m = 0.705; m <= 1.60; m += 0.005) {
            const double cl = aero.compute_coefficients(s, ctrl, 20000.0, m).CL;
            worst = std::max(worst, std::abs(cl / prev_cl - 1.0));
            prev_cl = cl;
        }
        std::cout << "  -> " << to_short_string(t) << " worst CL step per 0.005 Mach: "
                  << worst * 100.0 << " %\n";
        assert(worst < 0.03);
    }

    // The F-16 flies its own table-driven model, with its own factor.
    double prev_pg = aero::F16AeroModel::prandtl_glauert(0.70);
    double worst_pg = 0.0;
    for (double m = 0.705; m <= 1.60; m += 0.005) {
        const double pg = aero::F16AeroModel::prandtl_glauert(m);
        worst_pg = std::max(worst_pg, std::abs(pg / prev_pg - 1.0));
        prev_pg = pg;
    }
    std::cout << "  -> F-16C table model worst PG step per 0.005 Mach: " << worst_pg * 100.0 << " %\n";
    assert(worst_pg < 0.03);
}

void test_a10_thrust_falls_with_mach() {
    std::cout << "[PERF] Testing A-10C high-bypass TF34 thrust falls with Mach...\n";
    // A positive ram term let the A-10 gain thrust with speed and run well past
    // its 381 kt level-flight maximum.
    propulsion::MultiEngine engine(AircraftType::A10_THUNDERBOLT);
    const auto static_air = environment::Atmosphere1976::compute(0.0, 0.0);
    const auto cruise_air = environment::Atmosphere1976::compute(0.0, 190.0); // ~M0.56
    const double ratio = engine.lapse_factor(cruise_air, false) / engine.lapse_factor(static_air, false);
    assert(ratio < 0.80 && ratio > 0.60);
    std::cout << "  -> A-10C thrust at M0.56 is " << ratio * 100.0 << " % of static.\n";
}

void test_full_aft_stick_stays_in_alpha_envelope() {
    std::cout << "[PERF] Testing sustained full aft stick at low speed stays inside the AoA envelope...\n";
    struct Case { AircraftType type; double alt, speed, max_swing; };
    // F-16: at full fuel the old limiter reacted on raw alpha and grew into a
    // -8..34 deg pitch oscillation.  F-22: the thrust vectoring followed the raw
    // stick against the limiter and tumbled the jet end over end.  Held at 60 deg
    // post-stall the F-22 keeps a slow, bounded bob of several degrees.
    constexpr Case cases[] = {{AircraftType::F16_FIGHTING_FALCON, 5000.0, 150.0, 6.0},
                              {AircraftType::F16_FIGHTING_FALCON, 9000.0, 180.0, 6.0},
                              {AircraftType::F22_RAPTOR,          5000.0, 130.0, 10.0}};
    for (const Case& c : cases) {
        FlightSimHarness sim(c.type);
        sim.trim_level(c.alt, c.speed);
        const double limit = AircraftConfig::get(c.type).flcs.alpha_limit_deg;
        double max_alpha = -180.0, late_min = 180.0, late_max = -180.0;
        const flcs::PilotCommands full_aft{1.0, 0.0, 0.0};
        for (int i = 0; i < 2000; ++i) { // 10 s
            sim.step(1.0, full_aft);
            const double alpha = sim.state.alpha() * (180.0 / M_PI);
            max_alpha = std::max(max_alpha, alpha);
            if (i >= 1200) {
                late_min = std::min(late_min, alpha);
                late_max = std::max(late_max, alpha);
            }
        }
        std::cout << "  -> " << to_short_string(c.type) << " at " << c.speed << " m/s: peak alpha "
                  << max_alpha << " deg (limit " << limit << "), settled " << late_min << ".."
                  << late_max << " deg\n";
        assert(max_alpha < limit + 12.0);          // never tumbles through
        assert(late_max < limit + 5.0);            // settles on the limit
        assert(late_max - late_min < c.max_swing); // no growing oscillation
    }
}

void test_full_stick_roll_reaches_profile_rate() {
    std::cout << "[PERF] Testing full lateral stick reaches each FBW profile's roll rate...\n";
    // Proportional-only roll loops settled 20-35% short (the F-22 managed 184 of
    // its 280 deg/s); the integral trim closes the gap.
    for (AircraftType t : {AircraftType::F15EX_EAGLE_II, AircraftType::EUROFIGHTER_TYPHOON,
                           AircraftType::F22_RAPTOR}) {
        FlightSimHarness sim(t);
        sim.trim_level(5000.0, 230.0);
        double p_max = 0.0;
        for (int i = 0; i < 300; ++i) { // 1.5 s
            sim.step(0.8, flcs::PilotCommands{0.0, 1.0, 0.0});
            p_max = std::max(p_max, sim.state.omega_b.x * 180.0 / M_PI);
        }
        const double rated = AircraftConfig::get(t).flcs.max_roll_rate_dps;
        std::cout << "  -> " << to_short_string(t) << ": " << p_max << " deg/s (rated " << rated << ")\n";
        assert(p_max > 0.93 * rated && p_max < 1.10 * rated);
    }
}

void test_a10_full_stick_is_lift_and_feel_limited() {
    std::cout << "[PERF] Testing A-10C full aft stick: no deep stall at medium altitude, no 10+ G at Vne...\n";
    // 22 deg of elevator per unit stick trimmed to ~77 deg alpha: full stick pulled
    // 9-17 G at speed and deep-stalled everywhere else.
    struct Case { double alt, speed; };
    for (const Case c : {Case{1000.0, 220.0}, Case{5000.0, 150.0}}) {
        FlightSimHarness sim(AircraftType::A10_THUNDERBOLT);
        sim.trim_level(c.alt, c.speed);
        double max_alpha = 0.0, max_nz = 0.0;
        for (int i = 0; i < 1000; ++i) { // 5 s
            sim.step(1.0, flcs::PilotCommands{1.0, 0.0, 0.0});
            max_alpha = std::max(max_alpha, sim.state.alpha() * 180.0 / M_PI);
            max_nz = std::max(max_nz, -sim.forces.force_b.z / (sim.mass.mass_kg * 9.80665));
        }
        std::cout << "  -> " << c.speed << " m/s at " << c.alt << " m: peak " << max_nz
                  << " G, peak alpha " << max_alpha << " deg\n";
        assert(max_nz < 8.5);     // can bend it a little past 7.33 G, not break it
        assert(max_alpha < 22.0); // stall buffet, not a deep stall
    }
}

} // namespace

int main() {
    std::cout << "=========================================================\n";
    std::cout << "  Fast Jet Multi-Aircraft Flight Performance Test Suite  \n";
    std::cout << "=========================================================\n";

    test_f15ex_high_speed_dash();
    test_typhoon_supercruise();
    test_f22_thrust_vectoring();
    test_a10_nacelle_pitch_coupling();
    test_a10_subsonic_drag_barrier();
    test_lift_continuous_through_transonic();
    test_a10_thrust_falls_with_mach();
    test_full_aft_stick_stays_in_alpha_envelope();
    test_full_stick_roll_reaches_profile_rate();
    test_a10_full_stick_is_lift_and_feel_limited();

    std::cout << "\nAll aircraft flight performance tests successfully passed!\n";
    return 0;
}
