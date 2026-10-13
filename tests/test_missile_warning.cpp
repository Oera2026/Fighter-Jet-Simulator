/**
 * @file test_missile_warning.cpp
 * @brief Missile approach warning systems (MAWS).
 *
 * The spec table: the F-16 has none; the UV/IR plume detectors see a burning
 * motor and lose a coasting missile; the Typhoon's pulse-doppler MAW sees a
 * coasting missile inside 8 km and nothing beyond; the F-35's DAS outranges
 * the rest. Threat geometry: a missile flying away or crossing well off is
 * not a threat, and the jet's own missiles never are. Cue: the clock bearing
 * of a missile from behind, in any heading, and a launch flag that fires
 * once per missile after the declaration time. In the world: the MAWS warns
 * of a heat seeker the RWR cannot hear.
 */
#include "fastjet/sim/world.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>

using namespace fastjet;
using aircraft::AircraftType;

namespace {

constexpr double DT = 0.005;
constexpr double V_OWN = 250.0;

fdm::FlightState level(const math::Vector3& pos, double heading, double v) {
    fdm::FlightState s{};
    s.pos_ned = pos;
    s.vel_b = math::Vector3(v, 0.0, 0.0);
    s.q_att = math::Quaternion::from_euler(0.0, 0.0, heading);
    return s;
}

/// The jet under test at the origin (5 km up, heading @p heading), and a
/// shooter at @p shooter_xy (level with it, facing it): by default 20 km
/// north, out of the way.
std::unique_ptr<sim::World> sky(AircraftType own, double heading = 0.0,
                                AircraftType shooter = AircraftType::F16_FIGHTING_FALCON,
                                const math::Vector3& shooter_xy = {20000.0, 0.0, 0.0}) {
    auto w = std::make_unique<sim::World>();
    w->clear(7);
    w->add(own, level({0.0, 0.0, -5000.0}, heading, V_OWN));
    w->add(shooter, level(shooter_xy + math::Vector3(0.0, 0.0, -5000.0), std::atan2(-shooter_xy.y, -shooter_xy.x), V_OWN));
    for (int i = 0; i < 2; ++i) w->aircraft[static_cast<size_t>(i)].landing_gear.deployed = false;
    return w;
}

/// A heat seeker at @p pos (relative to the jet under test) flying @p vel,
/// its motor burning or long since burnt out.
sim::Missile missile(const math::Vector3& pos, const math::Vector3& vel, bool burning, int shooter = 1) {
    sim::Missile m;
    m.spec = sim::missile_specs::aim9x();
    m.body.pos = pos + math::Vector3(0.0, 0.0, -5000.0);
    m.body.vel = vel;
    m.body.tof = burning ? 1.0 : m.spec.motor_time_s() + 2.0;
    m.shooter = shooter;
    m.target = 0;
    m.active = true;
    m.phase = sim::Missile::Phase::ACTIVE_TRACK;
    return m;
}

/// What jet 0's MAWS reports after watching @p m (held still) for @p seconds.
sim::MawsStatus watch(AircraftType own, const sim::Missile& m, double seconds = 1.0, double heading = 0.0) {
    auto w = sky(own, heading);
    w->missiles[3] = m;
    for (int i = 0; i < static_cast<int>(seconds / DT); ++i) w->maws[0].update(DT, 0, w->aircraft[0], w->missiles);
    return w->maws[0].status;
}

/// A missile dead astern at @p range, closing at Mach 3.
sim::Missile from_six(double range, bool burning) {
    return missile({-range, 0.0, 0.0}, {1000.0, 0.0, 0.0}, burning);
}

const AircraftType kAll[] = {AircraftType::F16_FIGHTING_FALCON, AircraftType::F15EX_EAGLE_II,
                             AircraftType::EUROFIGHTER_TYPHOON, AircraftType::F22_RAPTOR,
                             AircraftType::A10_THUNDERBOLT,     AircraftType::F35B_LIGHTNING_II};
const AircraftType kPlume[] = {AircraftType::F15EX_EAGLE_II, AircraftType::F22_RAPTOR, AircraftType::A10_THUNDERBOLT,
                               AircraftType::F35B_LIGHTNING_II};

void test_spec_table() {
    std::cout << "[MAWS] Spec table: who carries what, and how far it sees...\n";
    for (const AircraftType t : kAll) {
        const sim::MawsSpec s = sim::maws_spec(t);
        std::cout << "    " << aircraft::to_short_string(t) << ": " << sim::to_string(s.kind) << " " << s.name << " "
                  << s.detect_range_m / 1000.0 << " km, declares in " << s.declare_s << " s\n";
        assert(s.fitted() == (t != AircraftType::F16_FIGHTING_FALCON));
        if (s.fitted()) assert(s.detect_range_m > 1000.0 && s.declare_s > 0.0 && s.declare_s < 1.0);
    }
    assert(sim::maws_spec(AircraftType::A10_THUNDERBOLT).kind == sim::MawsKind::PASSIVE_UV);
    const sim::MawsSpec ef = sim::maws_spec(AircraftType::EUROFIGHTER_TYPHOON);
    assert(ef.kind == sim::MawsKind::PULSE_DOPPLER && !ef.needs_motor() && ef.measures_range());
    assert(std::abs(ef.detect_range_m - 8000.0) < 1.0);
    const sim::MawsSpec das = sim::maws_spec(AircraftType::F35B_LIGHTNING_II);
    for (const AircraftType t : kPlume) {
        const sim::MawsSpec s = sim::maws_spec(t);
        assert(s.needs_motor() && !s.measures_range());
        if (t != AircraftType::F35B_LIGHTNING_II) assert(das.detect_range_m > 2.0 * s.detect_range_m);
    }
    std::cout << "  -> PASSED\n";
}

void test_f16_never_warns() {
    std::cout << "[MAWS] F-16C: no MAWS, so even a burning motor at 2 km is silent...\n";
    for (const bool burning : {true, false}) {
        const sim::MawsStatus s = watch(AircraftType::F16_FIGHTING_FALCON, from_six(2000.0, burning), 2.0);
        assert(!s.threat && !s.launch && s.launch_count == 0 && s.threat_count == 0);
    }
    std::cout << "  -> PASSED\n";
}

void test_plume_detectors_need_the_motor() {
    std::cout << "[MAWS] UV/IR plume detectors: a burning motor yes, a coasting missile no...\n";
    for (const AircraftType t : kPlume) {
        const double inside = 0.6 * sim::maws_spec(t).detect_range_m;
        const sim::MawsStatus burning = watch(t, from_six(inside, true));
        const sim::MawsStatus coasting = watch(t, from_six(inside, false));
        std::cout << "    " << aircraft::to_short_string(t) << " at " << inside / 1000.0
                  << " km: burning -> " << burning.threat << ", coasting -> " << coasting.threat << "\n";
        assert(burning.threat && burning.launch_count == 1 && !burning.range_valid);
        assert(!coasting.threat && coasting.launch_count == 0);
        // Beyond the plume range, burning or not, nothing.
        assert(!watch(t, from_six(1.2 * sim::maws_spec(t).detect_range_m, true)).threat);
    }
    std::cout << "  -> PASSED\n";
}

void test_typhoon_doppler_sees_a_coasting_missile() {
    std::cout << "[MAWS] Typhoon pulse-doppler MAW: a coasting missile inside 8 km, not beyond...\n";
    const sim::MawsStatus near = watch(AircraftType::EUROFIGHTER_TYPHOON, from_six(6000.0, false));
    const sim::MawsStatus far = watch(AircraftType::EUROFIGHTER_TYPHOON, from_six(9000.0, false));
    std::cout << "    6 km: threat " << near.threat << ", range " << near.range_m << " m, TTG " << near.time_to_go_s
              << " s; 9 km: threat " << far.threat << "\n";
    assert(near.threat && near.range_valid && std::abs(near.range_m - 6000.0) < 1.0);
    assert(std::abs(near.time_to_go_s - 6000.0 / (1000.0 - V_OWN)) < 0.05);
    assert(!far.threat && far.launch_count == 0);
    // ...and a burning one beyond 8 km is still out of its reach.
    assert(!watch(AircraftType::EUROFIGHTER_TYPHOON, from_six(9000.0, true)).threat);
    std::cout << "  -> PASSED\n";
}

void test_das_outranges_the_rest() {
    std::cout << "[MAWS] F-35B DAS sees a launch at 20 km that the UV/IR sets cannot...\n";
    const sim::Missile launch = missile({20000.0, 0.0, 0.0}, {-900.0, 0.0, 0.0}, true); // head-on, 20 km
    assert(watch(AircraftType::F35B_LIGHTNING_II, launch).threat);
    for (const AircraftType t : {AircraftType::F15EX_EAGLE_II, AircraftType::F22_RAPTOR, AircraftType::A10_THUNDERBOLT,
                                 AircraftType::EUROFIGHTER_TYPHOON}) {
        assert(!watch(t, launch).threat);
    }
    std::cout << "  -> PASSED\n";
}

void test_threat_geometry() {
    std::cout << "[MAWS] Only a closing missile pointed at the jet is a threat...\n";
    for (const AircraftType t : {AircraftType::F35B_LIGHTNING_II, AircraftType::EUROFIGHTER_TYPHOON}) {
        // Behind and flying away: opening.
        assert(!watch(t, missile({-3000.0, 0.0, 0.0}, {-900.0, 0.0, 0.0}, true)).threat);
        // Ahead, flying the same way faster than the jet: opening.
        assert(!watch(t, missile({3000.0, 0.0, 0.0}, {900.0, 0.0, 0.0}, true)).threat);
        // 4 km off the right wing, crossing in front at Mach 2.5 toward
        // someone else: it passes kilometres ahead of the jet.
        const sim::Missile crossing = missile({0.0, 4000.0, 0.0}, {800.0, -400.0, 0.0}, true);
        const sim::MawsContact c =
            sim::maws_assess(sim::maws_spec(t), {0.0, 0.0, -5000.0}, {V_OWN, 0.0, 0.0}, crossing);
        std::cout << "    " << aircraft::to_short_string(t) << " crossing: closing " << c.closing_mps
                  << " m/s, miss " << c.miss_m << " m\n";
        assert(c.detected && !c.threat);
        assert(!watch(t, crossing).threat);
        // The same missile on a collision course from 3 o'clock is one.
        const sim::Missile beam = missile({0.0, 3000.0, 0.0}, {V_OWN, -800.0, 0.0}, true);
        assert(watch(t, beam).threat);
        // The jet's own missile, even one that turns back on it, never is.
        assert(!watch(t, missile({-3000.0, 0.0, 0.0}, {1000.0, 0.0, 0.0}, true, /*shooter=*/0)).threat);
    }
    std::cout << "  -> PASSED\n";
}

void test_clock_bearing() {
    std::cout << "[MAWS] Clock bearing: a missile from behind is 6 o'clock, whatever the heading...\n";
    for (const double heading : {0.0, M_PI / 2.0, -2.5}) {
        // Dead astern of a jet on this heading, 4 km back, 300 m low.
        const math::Vector3 fwd(std::cos(heading), std::sin(heading), 0.0);
        const sim::Missile m = missile(fwd * -4000.0 + math::Vector3(0.0, 0.0, 300.0), fwd * 1000.0, true);
        const sim::MawsStatus s = watch(AircraftType::F35B_LIGHTNING_II, m, 1.0, heading);
        std::cout << "    heading " << heading * 180.0 / M_PI << ": clock " << s.clock << ", bearing "
                  << s.bearing_rad * 180.0 / M_PI << ", elevation " << s.elevation_rad * 180.0 / M_PI << ", TTG "
                  << s.time_to_go_s << " s\n";
        assert(s.threat && s.clock == 6);
        assert(std::abs(std::abs(s.bearing_rad) - M_PI) < 0.01);
        assert(s.elevation_rad < -0.05 && s.elevation_rad > -0.1); // a little low
        assert(std::abs(s.time_to_go_s - std::hypot(4000.0, 300.0) / (1000.0 - V_OWN)) < 0.5);
        assert((s.los_ned + fwd).norm() < 0.1);
    }
    // 3 o'clock, low and high.
    const sim::MawsStatus right = watch(AircraftType::A10_THUNDERBOLT, missile({0.0, 3000.0, 0.0}, {V_OWN, -800.0, 0.0}, true));
    assert(right.threat && right.clock == 3);
    assert(sim::MawsStatus::clock_of(0.0) == 12 && sim::MawsStatus::clock_of(-M_PI / 2.0) == 9 &&
           sim::MawsStatus::clock_of(M_PI / 6.0) == 1 && sim::MawsStatus::clock_of(-0.2) == 12);
    std::cout << "  -> PASSED\n";
}

void test_launch_flag_once() {
    std::cout << "[MAWS] Launch flag: after the declaration time, once per missile...\n";
    auto w = sky(AircraftType::F22_RAPTOR);
    const double declare = sim::maws_spec(AircraftType::F22_RAPTOR).declare_s;
    w->missiles[5] = from_six(3000.0, true);
    int launches = 0;
    double declared_at = -1.0;
    for (int i = 0; i < static_cast<int>(2.0 / DT); ++i) {
        w->maws[0].update(DT, 0, w->aircraft[0], w->missiles);
        const sim::MawsStatus& s = w->maws[0].status;
        if (s.launch) {
            ++launches;
            declared_at = (i + 1) * DT;
        }
        assert(s.threat == (declared_at > 0.0)); // nothing shown before it is declared
    }
    std::cout << "    declared after " << declared_at << " s (spec " << declare << " s)\n";
    assert(launches == 1 && w->maws[0].status.launch_count == 1);
    assert(std::abs(declared_at - declare) < 2.0 * DT);

    // A second missile in another slot: declared, flagged once more.
    w->missiles[6] = missile({0.0, -2500.0, 0.0}, {V_OWN, 800.0, 0.0}, true);
    for (int i = 0; i < static_cast<int>(1.0 / DT); ++i) {
        w->maws[0].update(DT, 0, w->aircraft[0], w->missiles);
        launches += w->maws[0].status.launch ? 1 : 0;
    }
    assert(launches == 2 && w->maws[0].status.launch_count == 2 && w->maws[0].status.threat_count == 2);
    // The more urgent one (2.5 km closing at 800 m/s vs 3 km at 750) leads: 9 o'clock.
    assert(w->maws[0].status.clock == 9);

    // The first missile's slot is freed and reused by a new missile: a new launch.
    w->missiles[5].active = false;
    w->maws[0].update(DT, 0, w->aircraft[0], w->missiles);
    w->missiles[5] = from_six(3500.0, true);
    for (int i = 0; i < static_cast<int>(1.0 / DT); ++i) {
        w->maws[0].update(DT, 0, w->aircraft[0], w->missiles);
        launches += w->maws[0].status.launch ? 1 : 0;
    }
    assert(launches == 3 && w->maws[0].status.launch_count == 3);
    std::cout << "  -> PASSED\n";
}

void test_in_the_world() {
    std::cout << "[WORLD] A heat seeker the RWR cannot hear sets off the MAWS, once...\n";
    for (const AircraftType t : {AircraftType::F16_FIGHTING_FALCON, AircraftType::A10_THUNDERBOLT,
                                 AircraftType::F35B_LIGHTNING_II}) {
        // A Typhoon 2.5 km behind, its AIM-9X just off the rail and tracking.
        auto w = sky(t, 0.0, AircraftType::EUROFIGHTER_TYPHOON, {-2500.0, 0.0, 0.0});
        sim::Missile m = missile({-2480.0, 0.0, 10.0}, {V_OWN, 0.0, 0.0}, true);
        m.body.tof = 0.0;
        m.est_pos = w->aircraft[0].position();
        m.est_vel = w->aircraft[0].velocity();
        w->missiles[0] = m;
        std::array<sim::AircraftControls, sim::World::kMaxAircraft> c{};
        for (auto& k : c) k.throttle = 0.8;
        int launches = 0;
        bool warned = false, rwr_heard = false;
        double first_warning = -1.0, first_ttg = 0.0, naive_ttg = 0.0;
        for (int i = 0; i < static_cast<int>(8.0 / DT) && w->missiles[0].active; ++i) {
            w->step(DT, c);
            const sim::MawsStatus& s = w->maws[0].status;
            launches += s.launch ? 1 : 0;
            if (s.threat && !warned) {
                first_warning = w->time;
                first_ttg = s.time_to_go_s;
                const math::Vector3 r = w->missiles[0].body.pos - w->aircraft[0].position();
                naive_ttg = r.norm() / s.closing_mps;
            }
            warned = warned || s.threat;
            rwr_heard = rwr_heard || w->rwr(0).level != sim::RwrStatus::Level::CLEAR;
            // The shooter's MAWS (a doppler set, motor or not) never warns of its own missile.
            assert(!w->maws[1].status.threat);
        }
        double detonated = -1.0;
        for (int k = 0; k < w->event_count(); ++k) {
            if (w->event(k).kind == sim::CombatEvent::Kind::MISSILE_DETONATION) detonated = w->event(k).time;
        }
        std::cout << "    " << aircraft::to_short_string(t) << ": MAWS " << (warned ? "warned" : "silent");
        if (warned) {
            std::cout << " at " << first_warning << " s, time to go " << first_ttg << " s (range / closing alone "
                      << naive_ttg << " s); detonation at " << detonated << " s";
        }
        std::cout << ", RWR " << (rwr_heard ? "heard it" : "silent") << "\n";
        assert(!rwr_heard); // a heat seeker is silent on the RWR
        if (t == AircraftType::F16_FIGHTING_FALCON) {
            assert(!warned && launches == 0);
        } else {
            assert(warned && launches == 1 && w->maws[0].status.launch_count == 1);
            assert(first_warning < 1.0);
            // A boosting missile: allowing for its acceleration gets the time
            // to go about right, where range over closing speed is far out.
            assert(detonated > first_warning);
            const double actual = detonated - first_warning;
            assert(first_ttg > 0.6 * actual && first_ttg < 1.5 * actual);
            assert(naive_ttg > 2.0 * actual);
        }
    }
    std::cout << "  -> PASSED\n";
}

} // namespace

int main() {
    std::cout << "=== Missile approach warning (MAWS) ===\n";
    test_spec_table();
    test_f16_never_warns();
    test_plume_detectors_need_the_motor();
    test_typhoon_doppler_sees_a_coasting_missile();
    test_das_outranges_the_rest();
    test_threat_geometry();
    test_clock_bearing();
    test_launch_flag_once();
    test_in_the_world();
    std::cout << "All MAWS tests passed.\n";
    return 0;
}
