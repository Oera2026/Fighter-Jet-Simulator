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
    int    gcas_activations{0}; ///< Recoveries started
    int    gcas_changes{0};     ///< GCAS state changes
    int    throttle_changes{0}; ///< Throttle override taken or given back
    double min_rearm_s{1e9};    ///< Shortest time from a hand-back to the next activation
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
    double nz{1.0};
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
        double released_at = -1.0;
        const int steps = static_cast<int>(seconds / DT);
        for (int i = 0; i < steps; ++i) {
            const auto air = environment::Atmosphere1976::compute(state.altitude(), state.airspeed());
            const auto imu = flcs::IMUData::read(state, forces, mass, air.dynamic_pressure);
            nz = imu.Nz;
            const auto gcas_before = ofc.gcas_state;
            const bool override_before = ofc.has_throttle_override();
            const auto effective = ofc.update(DT, state, imu, pilot(*this), gear_down);
            const auto surfaces = flcs.update(DT, state, effective, air.dynamic_pressure, forces, mass);

            const bool was_active = gcas_before == flcs::OnBoardFlightComputer::GcasState::WING_LEVEL ||
                                    gcas_before == flcs::OnBoardFlightComputer::GcasState::PULL_UP;
            if (ofc.is_gcas_active() && !was_active) {
                ++out.gcas_activations;
                if (released_at >= 0.0) out.min_rearm_s = std::min(out.min_rearm_s, time - released_at);
            }
            if (!ofc.is_gcas_active() && was_active) released_at = time;
            out.gcas_changes += ofc.gcas_state != gcas_before;
            out.throttle_changes += ofc.has_throttle_override() != override_before;
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
    // The recovery is planned to bottom out at the terrain buffer (30 ft); it
    // may run up to a quarter of it below.
    assert(o.min_agl > 0.75 * flcs::OnBoardFlightComputer::GCAS_DEFAULT_BUFFER_M);
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

/// Holds @p bank and the starting height at low level: G from the height and
/// climb errors, closed on the measured Nz with integral trim, so the same
/// pilot flies the G-command jets and the A-10's direct-linkage stick.
struct LowLevelTurn {
    double bank;
    double alt{-1.0};
    double trim{0.0};
    flcs::PilotCommands operator()(const ClosedLoopSim& s) {
        if (alt < 0.0) alt = s.state.altitude();
        const double g_max = AircraftConfig::get(s.type).flcs.max_g_positive;
        const double phi = s.state.roll();
        const double climb = -s.state.velocity_ned().z;
        const double n_cmd = std::clamp(1.0 / std::max(0.2, std::cos(phi)) + 0.05 * (alt - s.state.altitude())
                                            - 0.15 * climb, 0.0, g_max - 0.5);
        trim = std::clamp(trim + 0.6 * (n_cmd - s.nz) * DT, -0.5, 0.5);
        // Roll in over the first second, then hold the bank.
        const double bank_cmd = bank * std::min(1.0, s.time);
        return flcs::PilotCommands{std::clamp((n_cmd - 1.0) / (g_max - 1.0) + trim, -1.0, 1.0),
                                   std::clamp(2.0 * (bank_cmd - phi) - 0.3 * s.state.omega_b.x, -1.0, 1.0), 0.0};
    }
};

void test_gcas_stays_out_of_low_level_turns() {
    std::cout << "[OFC] Level turns at 150 ft AGL (70 deg bank, A-10 45 deg): no intervention...\n";
    // The pilot holds his height in the turn: the trajectory prediction must
    // count the turn and the G already on the jet, not fly a wings-level 1 G
    // path through the ground.
    for (AircraftType t : ALL_AIRCRAFT) {
        const bool a10 = (t == AircraftType::A10_THUNDERBOLT);
        ClosedLoopSim sim(t);
        sim.start(46.0, a10 ? 150.0 : 230.0, 0.0, 0.0);
        LowLevelTurn pilot{(a10 ? 45.0 : 70.0) * DEG};
        const Outcome o = sim.fly(15.0, a10 ? 1.0 : 0.85, [&pilot](const ClosedLoopSim& s) { return pilot(s); });
        std::cout << "    " << to_short_string(t) << ": min AGL " << o.min_agl << " m, peak " << o.max_nz
                  << " G, GCAS " << o.gcas_s << " s\n";
        assert(!o.crashed);
        assert(o.gcas_s == 0.0 && "a held low-level turn is not a collision course");
    }
    std::cout << "  -> PASSED\n";
}

void test_gcas_saves_inverted_dives() {
    std::cout << "[OFC] Hands-off inverted dives at speed: rolled upright first, then pulled out...\n";
    struct Dive { double agl, v, gamma, roll; };
    constexpr Dive FAST_JET[] = {{2500, 280, -45, 150}, {2500, 280, -60, 120}};
    constexpr Dive A10[]      = {{2000, 200, -45, 150}, {2000, 200, -60, 120}};
    for (AircraftType t : ALL_AIRCRAFT) {
        const Dive* dives = t == AircraftType::A10_THUNDERBOLT ? A10 : FAST_JET;
        for (int i = 0; i < 2; ++i) {
            ClosedLoopSim sim(t);
            sim.start(dives[i].agl, dives[i].v, dives[i].gamma, dives[i].roll);
            const Outcome o = sim.fly(30.0, 0.8, hands_off);
            expect_saved(t, o, "inverted dive");
        }
    }
    std::cout << "  -> PASSED\n";
}

void test_gcas_save_does_not_chatter() {
    std::cout << "[OFC] Every save runs roll -> pull -> hand-back once: no mode or throttle chatter...\n";
    // A recovery that flips back and forth fights the pilot and the throttle
    // every few ticks. A second, separate save is fine: the hands-off A-10
    // (no auto-trim, ~0.1 G with the stick let go) noses over again after the
    // hand-back and is saved a second time some 8 s later.
    for (AircraftType t : ALL_AIRCRAFT) {
        ClosedLoopSim sim(t);
        sim.start(1200.0, t == AircraftType::A10_THUNDERBOLT ? 150.0 : 250.0, -45.0, 90.0);
        const Outcome o = sim.fly(25.0, 0.8, hands_off);
        std::cout << "    " << to_short_string(t) << ": " << o.gcas_activations << " saves, " << o.gcas_changes
                  << " GCAS state changes, " << o.throttle_changes << " throttle hand-overs, min AGL " << o.min_agl
                  << " m\n";
        assert(!o.crashed);
        assert(o.gcas_activations >= 1);
        assert(o.gcas_changes <= 3 * o.gcas_activations);
        assert(o.throttle_changes <= 2 * o.gcas_activations);
        assert(o.min_rearm_s > 3.0 && "a hand-back must not be followed at once by another activation");
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
    test_gcas_stays_out_of_low_level_turns();
    test_gcas_saves_inverted_dives();
    test_gcas_save_does_not_chatter();
    test_tumble_law_respects_airframe_alpha_envelope();
    std::cout << "\nAll OFC closed-loop tests passed successfully!\n";
    return 0;
}
