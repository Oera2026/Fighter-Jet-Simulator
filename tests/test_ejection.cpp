/**
 * @file test_ejection.cpp
 * @brief Ejection seats: the zero-zero envelope, the sequencer's three modes,
 * descent under the canopy, the escapes that kill, and the world's handling
 * of an empty cockpit.
 */
#include "fastjet/sim/ejection_seat.hpp"
#include "fastjet/sim/world.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string_view>

using namespace fastjet;
using sim::EjectionSeat;

namespace {

constexpr double DT = 0.005;
constexpr double KT = 0.514444;

/// A jet at @p alt_m, @p kt true airspeed, level unless @p roll_deg, over the
/// flat airfield basin (so the ground is at 0 m).
fdm::FlightState jet_at(double alt_m, double kt, double roll_deg = 0.0, double pitch_deg = 0.0) {
    fdm::FlightState s{};
    s.pos_ned = math::Vector3(1500.0, 0.0, -alt_m);
    s.vel_b = math::Vector3(kt * KT, 0.0, 0.0);
    s.q_att = math::Quaternion::from_euler(roll_deg * M_PI / 180.0, pitch_deg * M_PI / 180.0, 0.0);
    return s;
}

struct Flight {
    double apex_agl = 0.0;
    double chute_open_alt = -1.0; ///< Altitude when the canopy started to open
    bool used_drogue = false;
    double time = 0.0;
};

/// Pull the handle and fly the seat to the ground. The jet carries on in a
/// straight line.
Flight eject(EjectionSeat& seat, aircraft::AircraftType type, fdm::FlightState jet) {
    Flight f;
    seat.initiate(type, jet);
    const math::Vector3 v = jet.velocity_ned();
    for (int i = 0; i < 200 * 600 && seat.phase != EjectionSeat::Phase::LANDED; ++i) {
        jet.pos_ned += v * DT;
        seat.step(DT, jet);
        f.apex_agl = std::max(f.apex_agl, -seat.pos.z);
        if (seat.drogue_out()) f.used_drogue = true;
        if (seat.phase == EjectionSeat::Phase::CHUTE && f.chute_open_alt < 0.0) f.chute_open_alt = -seat.pos.z;
    }
    f.time = seat.time;
    return f;
}

void test_zero_zero() {
    std::cout << "[Test] Ejection: Zero-Zero From The Runway... ";
    EjectionSeat seat;
    // On the runway, stopped: the seat's own rocket has to get the chute open.
    const Flight f = eject(seat, aircraft::AircraftType::F16_FIGHTING_FALCON, jet_at(1.6, 0.0));
    assert(seat.mode == 1);
    assert(f.apex_agl > 45.0 && "ACES II zero-zero apex is about 60 m");
    assert(seat.outcome == EjectionSeat::Outcome::SAFE);
    assert(seat.chute_fraction > 0.99);
    std::printf("PASSED (apex %.0f m, down in %.1f s at %.1f m/s)\n", f.apex_agl, f.time, seat.landing_speed_mps);
}

void test_mode_1_low_and_slow() {
    std::cout << "[Test] Ejection: Mode 1 (200 KEAS, 300 m) Opens The Canopy At Once... ";
    EjectionSeat seat;
    const Flight f = eject(seat, aircraft::AircraftType::A10_THUNDERBOLT, jet_at(300.0, 200.0));
    assert(seat.mode == 1);
    assert(!f.used_drogue);
    assert(seat.outcome == EjectionSeat::Outcome::SAFE);
    std::printf("PASSED (canopy opening at %.0f m)\n", f.chute_open_alt);
}

void test_f35b_hover_zero_zero() {
    std::cout << "[Test] Ejection: F-35B US16E From A 30 m Hover... ";
    EjectionSeat seat;
    const Flight f = eject(seat, aircraft::AircraftType::F35B_LIGHTNING_II, jet_at(30.0, 0.0));
    assert(std::string_view(seat.spec.name) == "US16E");
    assert(seat.mode == 1);
    assert(seat.outcome == EjectionSeat::Outcome::SAFE);
    std::printf("PASSED (apex %.0f m AGL)\n", f.apex_agl);
}

void test_mode_2_drogue_first() {
    std::cout << "[Test] Ejection: Mode 2 (450 KEAS) Drogue Before The Canopy... ";
    EjectionSeat seat;
    const Flight f = eject(seat, aircraft::AircraftType::F16_FIGHTING_FALCON, jet_at(1500.0, 450.0));
    assert(seat.mode == 2);
    assert(f.used_drogue);
    assert(seat.peak_decel_g < EjectionSeat::OPENING_INJURY_G && "The drogue must slow him before the canopy opens");
    assert(seat.outcome == EjectionSeat::Outcome::SAFE);
    std::printf("PASSED (opening shock %.1f G)\n", seat.peak_decel_g);
}

void test_mode_3_rides_the_drogue_down() {
    std::cout << "[Test] Ejection: Mode 3 (30,000 ft) Rides The Drogue Below 15,000 ft... ";
    EjectionSeat seat;
    const Flight f = eject(seat, aircraft::AircraftType::F22_RAPTOR, jet_at(9144.0, 420.0));
    assert(seat.mode == 3);
    assert(f.chute_open_alt <= seat.spec.mode3_alt_m + 1.0 && "No canopy in the thin air above 15,000 ft");
    assert(f.chute_open_alt > 4000.0);
    assert(seat.outcome == EjectionSeat::Outcome::SAFE);
    std::printf("PASSED (canopy at %.0f m, %.0f s to the ground)\n", f.chute_open_alt, f.time);
}

void test_typhoon_barostat() {
    std::cout << "[Test] Ejection: Mk16A Holds The Drogue To 10,000 ft... ";
    EjectionSeat seat;
    const Flight f = eject(seat, aircraft::AircraftType::EUROFIGHTER_TYPHOON, jet_at(6000.0, 350.0));
    assert(seat.mode == 3);
    assert(f.chute_open_alt <= 3049.0);
    assert(seat.outcome == EjectionSeat::Outcome::SAFE);
    std::printf("PASSED (canopy at %.0f m)\n", f.chute_open_alt);
}

void test_descent_rate_under_canopy() {
    std::cout << "[Test] Ejection: 7-9 m/s Under A Full Canopy... ";
    EjectionSeat seat;
    (void)eject(seat, aircraft::AircraftType::F15EX_EAGLE_II, jet_at(1200.0, 250.0));
    assert(seat.landing_speed_mps > 6.0 && seat.landing_speed_mps < EjectionSeat::LANDING_SAFE_MPS);
    std::printf("PASSED (%.1f m/s at touchdown)\n", seat.landing_speed_mps);
}

void test_inverted_and_low_is_fatal() {
    std::cout << "[Test] Ejection: Inverted At 40 m Is Fatal... ";
    EjectionSeat seat;
    (void)eject(seat, aircraft::AircraftType::F16_FIGHTING_FALCON, jet_at(40.0, 300.0, 180.0));
    assert(seat.outcome == EjectionSeat::Outcome::KILLED);
    std::printf("PASSED (%s)\n", seat.cause);
}

void test_upright_at_40m_survives() {
    std::cout << "[Test] Ejection: Upright At 40 m Survives... ";
    EjectionSeat seat;
    (void)eject(seat, aircraft::AircraftType::F16_FIGHTING_FALCON, jet_at(40.0, 150.0));
    assert(seat.outcome != EjectionSeat::Outcome::KILLED);
    std::printf("PASSED (%s)\n", sim::to_string(seat.outcome));
}

void test_windblast() {
    std::cout << "[Test] Ejection: Windblast Beyond The Seat's Limits... ";
    EjectionSeat injured, killed;
    (void)eject(injured, aircraft::AircraftType::F16_FIGHTING_FALCON, jet_at(1000.0, 640.0));
    (void)eject(killed, aircraft::AircraftType::F16_FIGHTING_FALCON, jet_at(300.0, 760.0));
    assert(injured.outcome == EjectionSeat::Outcome::INJURED);
    assert(killed.outcome == EjectionSeat::Outcome::KILLED);
    std::printf("PASSED (640 KEAS %s, 760 KEAS %s)\n", injured.cause, killed.cause);
}

void test_world_empties_the_cockpit() {
    std::cout << "[Test] Ejection: World Hands The Jet Over To Physics... ";
    sim::World w;
    w.clear(5);
    w.add(aircraft::AircraftType::F16_FIGHTING_FALCON, jet_at(3000.0, 400.0));
    std::array<sim::AircraftControls, sim::World::kMaxAircraft> c{};
    c[0].throttle = 0.8;
    w.step(DT, c);
    assert(w.aircraft[0].alive());
    c[0].eject = true;
    c[0].stick.pitch_stick = 1.0; // ignored: nobody is holding the stick any more
    w.step(DT, c);
    assert(!w.aircraft[0].alive());
    assert(w.seats[0].initiated());
    bool ejection_event = false, kill_event = false;
    for (int i = 0; i < w.event_count(); ++i) {
        ejection_event |= w.event(i).kind == sim::CombatEvent::Kind::EJECTION;
        kill_event |= w.event(i).kind == sim::CombatEvent::Kind::KILL;
    }
    assert(ejection_event && kill_event);
    for (int i = 0; i < 400; ++i) w.step(DT, c);
    assert(w.seats[0].clear_of_jet());
    // The seat has gone up and away from the jet.
    const double sep = (w.seats[0].pos - w.aircraft[0].position()).norm();
    assert(sep > 20.0);
    assert(w.aircraft[0].imu.Nz < 3.0 && "The abandoned jet must not follow the last stick input");
    std::printf("PASSED (seat %.0f m from the jet after 2 s)\n", sep);
}

} // namespace

int main() {
    std::cout << "=========================================================\n";
    std::cout << "  Ejection Seat Verification\n";
    std::cout << "=========================================================\n";
    test_zero_zero();
    test_mode_1_low_and_slow();
    test_f35b_hover_zero_zero();
    test_mode_2_drogue_first();
    test_mode_3_rides_the_drogue_down();
    test_typhoon_barostat();
    test_descent_rate_under_canopy();
    test_inverted_and_low_is_fatal();
    test_upright_at_40m_survives();
    test_windblast();
    test_world_empties_the_cockpit();
    std::cout << "All ejection seat tests passed.\n";
    return 0;
}
