/**
 * @file test_ofc_closed_loop.cpp
 * @brief Closed-loop On-Board Flight Computer verification across all 5 airframes.
 *
 * test_ofc_handover checks the OFC one tick at a time against hand-placed states.
 * That never flew a recovery to the end, so it could not see that Auto-GCAS
 * engaged too late to pull most dives out, over-stressed the A-10, or stood in
 * the way of a normal approach.  Here each airframe is flown through the same
 * loop as f16_sim_viewer (IMU -> OFC -> FLCS -> throttle override -> engine ->
 * RK4) and judged on the outcome: did the jet hit the ground, did it exceed its
 * own G limit, and did the OFC stay out of manoeuvres that needed no help.
 */
#include "fastjet/aircraft/aircraft_type.hpp"
#include "fastjet/aircraft/aircraft_config.hpp"
#include "fastjet/fdm/rk4_integrator.hpp"
#include "fastjet/fdm/fuel_system.hpp"
#include "fastjet/environment/atmosphere1976.hpp"
#include "fastjet/environment/ground_collision.hpp"
#include "fastjet/aero/aircraft_aero_model.hpp"
#include "fastjet/flcs/aircraft_flcs.hpp"
#include "fastjet/flcs/onboard_flight_computer.hpp"
#include "fastjet/propulsion/multi_engine.hpp"
#include <cassert>
#include <cmath>
#include <functional>
#include <iostream>

using namespace fastjet;
using namespace fastjet::aircraft;

namespace {

constexpr double DT = 0.005;
constexpr double DEG = M_PI / 180.0;

constexpr AircraftType ALL_AIRCRAFT[] = {
    AircraftType::F16_FIGHTING_FALCON, AircraftType::F15EX_EAGLE_II,
    AircraftType::EUROFIGHTER_TYPHOON, AircraftType::F22_RAPTOR,
    AircraftType::A10_THUNDERBOLT, AircraftType::F35B_LIGHTNING_II};

struct Outcome {
    bool   crashed{false};
    double min_agl{1e9};
    double max_nz{-1e9};
    double gcas_s{0.0};
    double tumble_s{0.0};
};

struct ClosedLoopSim {
    AircraftType type;
    AircraftConfig cfg;
    fdm::FlightState state{};
    fdm::MassProperties mass{};
    fdm::RK4Integrator integrator{DT};
    aero::AircraftAeroModel aero;
    flcs::AircraftFLCS flcs;
    propulsion::MultiEngine engine;
    flcs::OnBoardFlightComputer ofc;
    fdm::AircraftForces forces{};
    double time{0.0};
    bool gear_down{false};

    explicit ClosedLoopSim(AircraftType t)
        : type(t), cfg(AircraftConfig::get(t)), aero(t), flcs(t), engine(t), ofc(t) {
        mass = fdm::FuelSystem::compute(engine.fuel_kg, t);
    }

    /// Wings-level-or-banked start on a straight flight path over the flat
    /// airfield basin, heading +X (toward the field centre).
    void start(double agl, double airspeed, double gamma_deg, double roll_deg) {
        state = fdm::FlightState{};
        const double x = -1500.0;
        const double h = environment::GroundCollision::get_terrain_height(x, 0.0) + agl;
        state.pos_ned = math::Vector3(x, 0.0, -h);
        state.vel_b   = math::Vector3(airspeed, 0.0, 0.0); // alpha 0: pitch == gamma
        state.q_att   = math::Quaternion::from_euler(roll_deg * DEG, gamma_deg * DEG, 0.0);
        const auto air = environment::Atmosphere1976::compute(state.altitude(), airspeed);
        forces = aero.compute_forces_and_moments(state, {}, air.dynamic_pressure, air.mach_number);
    }

    Outcome fly(double seconds, double throttle,
                const std::function<flcs::PilotCommands(const ClosedLoopSim&)>& pilot) {
        Outcome out;
        const int steps = static_cast<int>(seconds / DT);
        for (int i = 0; i < steps; ++i) {
            const auto air = environment::Atmosphere1976::compute(state.altitude(), state.airspeed());
            const auto imu = flcs::IMUData::read(state, forces, mass, air.dynamic_pressure);
            const auto effective = ofc.update(DT, state, imu, pilot(*this), gear_down);
            const auto surfaces = flcs.update(DT, state, effective, air.dynamic_pressure, forces, mass);

            const double lever = ofc.has_throttle_override() ? ofc.throttle_override_value() : throttle;
            engine.tvc_pitch_cmd = flcs.tvc_pitch_cmd;
            const double thrust = engine.update(lever, air, DT);
            mass = fdm::FuelSystem::compute(engine.fuel_kg, type);

            integrator.step(state, time, mass,
                [&](double, const fdm::FlightState& s) noexcept -> fdm::AircraftForces {
                    const auto sa = environment::Atmosphere1976::compute(s.altitude(), s.airspeed());
                    auto f = aero.compute_forces_and_moments(s, surfaces, sa.dynamic_pressure, sa.mach_number);
                    f.force_b.x += thrust;
                    f.moment_b.y += engine.pitch_moment();
                    return f;
                });
            forces = aero.compute_forces_and_moments(state, surfaces, air.dynamic_pressure, air.mach_number);
            forces.force_b.x += thrust;
            forces.moment_b.y += engine.pitch_moment();

            const double agl = environment::GroundCollision::get_agl(state);
            out.min_agl = std::min(out.min_agl, agl);
            out.max_nz  = std::max(out.max_nz, imu.Nz);
            if (ofc.is_gcas_active())   out.gcas_s   += DT;
            if (ofc.is_tumble_active()) out.tumble_s += DT;
            if (agl < 1.0) {
                out.crashed = true;
                break;
            }
        }
        return out;
    }
};

flcs::PilotCommands hands_off(const ClosedLoopSim&) { return {}; }

void expect_saved(AircraftType t, const Outcome& o, const char* what) {
    const double g_limit = AircraftConfig::get(t).flcs.max_g_positive;
    std::cout << "    " << to_short_string(t) << " " << what << ": min AGL " << o.min_agl
              << " m, peak " << o.max_nz << " G (limit " << g_limit << ")\n";
    assert(!o.crashed && "Auto-GCAS must pull a survivable dive out");
    assert(o.min_agl > 10.0);
    assert(o.max_nz <= g_limit + 0.3 && "the save must not over-stress the airframe");
}

void test_gcas_saves_hands_off_dives() {
    std::cout << "[OFC] Hands-off dives from 1,200 m: every airframe is pulled out, within its G limit...\n";
    struct Dive { double v, gamma, roll; };
    // Includes steep, fast entries that need more than the nominal 5 G and banked
    // entries that must roll wings-level first.
    constexpr Dive FAST_JET[] = {{120, -30, 0}, {120, -60, 120}, {200, -30, 120},
                                 {200, -60, 0}, {300, -30, 0},   {300, -60, 120}};
    constexpr Dive A10[]      = {{110, -30, 0}, {150, -60, 120}, {190, -30, 0}};

    for (AircraftType t : ALL_AIRCRAFT) {
        const bool a10 = (t == AircraftType::A10_THUNDERBOLT);
        const Dive* dives = a10 ? A10 : FAST_JET;
        const int n = a10 ? 3 : 6;
        for (int i = 0; i < n; ++i) {
            ClosedLoopSim sim(t);
            sim.start(1200.0, dives[i].v, dives[i].gamma, dives[i].roll);
            const Outcome o = sim.fly(25.0, 0.8, hands_off);
            expect_saved(t, o, "dive");
        }
    }
    std::cout << "  -> PASSED\n";
}

void test_gcas_saves_slow_dives() {
    std::cout << "[OFC] Slow 30 deg dive at 95 m/s: recovery planned on the G actually available...\n";
    // At 95 m/s none of these jets can pull the nominal 5 G.  Planning on it
    // engaged too late, and the F-15EX, Typhoon and F-22 all hit the ground.
    for (AircraftType t : ALL_AIRCRAFT) {
        ClosedLoopSim sim(t);
        sim.start(1200.0, 95.0, -30.0, 0.0);
        const Outcome o = sim.fly(25.0, 0.8, hands_off);
        expect_saved(t, o, "slow dive");
    }
    std::cout << "  -> PASSED\n";
}

void test_gcas_stays_out_of_a_sane_pullout() {
    std::cout << "[OFC] Pilot pulls out of a 30 deg dive with room to spare: no intervention...\n";
    for (AircraftType t : ALL_AIRCRAFT) {
        ClosedLoopSim sim(t);
        sim.start(900.0, t == AircraftType::A10_THUNDERBOLT ? 150.0 : 220.0, -30.0, 0.0);
        const Outcome o = sim.fly(12.0, 0.8, [](const ClosedLoopSim& s) {
            return flcs::PilotCommands{s.time > 1.0 ? 0.6 : 0.0, 0.0, 0.0};
        });
        assert(!o.crashed);
        assert(o.gcas_s == 0.0 && "GCAS must not take a recovery the pilot is already flying");
        assert(o.tumble_s == 0.0);
    }
    std::cout << "  -> PASSED\n";
}

void test_gcas_stays_out_of_low_level_cruise() {
    std::cout << "[OFC] Level cruise at 60 m AGL over flat ground: no intervention...\n";
    for (AircraftType t : ALL_AIRCRAFT) {
        ClosedLoopSim sim(t);
        sim.start(60.0, t == AircraftType::A10_THUNDERBOLT ? 150.0 : 230.0, 0.0, 0.0);
        // Flight-path hold with integral trim, so the same pilot flies both the
        // fly-by-wire jets and the A-10's direct-linkage stick.
        double trim = 0.0;
        const Outcome o = sim.fly(12.0, 0.7, [&trim](const ClosedLoopSim& s) {
            const auto v = s.state.velocity_ned();
            const double gamma = std::atan2(-v.z, std::hypot(v.x, v.y)) / DEG;
            trim = std::clamp(trim - 0.02 * gamma * DT, -0.5, 0.5);
            return flcs::PilotCommands{std::clamp(trim - 0.1 * gamma - 0.2 * s.state.omega_b.y, -1.0, 1.0),
                                       std::clamp(-2.0 * s.state.roll(), -1.0, 1.0), 0.0};
        });
        assert(!o.crashed);
        assert(o.gcas_s == 0.0);
    }
    std::cout << "  -> PASSED\n";
}

void test_tumble_law_respects_airframe_alpha_envelope() {
    std::cout << "[OFC] Full aft stick at low speed: tumble recovery leaves limiter-held jets alone...\n";
    // The Typhoon's carefree FBW holds up to 35 deg; the fixed 32 deg departure
    // threshold used to slam full nose-down on a perfectly controlled pull.
    for (AircraftType t : {AircraftType::F16_FIGHTING_FALCON, AircraftType::F15EX_EAGLE_II,
                           AircraftType::EUROFIGHTER_TYPHOON}) {
        ClosedLoopSim sim(t);
        sim.start(5000.0, 120.0, 0.0, 0.0);
        const Outcome o = sim.fly(6.0, 1.0, [](const ClosedLoopSim&) {
            return flcs::PilotCommands{1.0, 0.0, 0.0};
        });
        std::cout << "    " << to_short_string(t) << ": tumble law active " << o.tumble_s << " s\n";
        assert(o.tumble_s == 0.0);
    }
    std::cout << "  -> PASSED\n";
}

} // namespace

int main() {
    std::cout << "=========================================================\n";
    std::cout << "  OFC Closed-Loop Verification (all airframes)           \n";
    std::cout << "=========================================================\n";
    test_gcas_saves_hands_off_dives();
    test_gcas_saves_slow_dives();
    test_gcas_stays_out_of_a_sane_pullout();
    test_gcas_stays_out_of_low_level_cruise();
    test_tumble_law_respects_airframe_alpha_envelope();
    std::cout << "\nAll OFC closed-loop tests passed successfully!\n";
    return 0;
}
