/**
 * @file test_missiles.cpp
 * @brief Radar missiles, fire-control radar, chaff and the RWR.
 *
 * Physics first: motor and drag give a Mach 3 burnout and a coast that
 * decays; lateral acceleration is capped by G and by available lift, so a slow
 * missile cannot turn. Guidance: PN intercepts a non-manoeuvring target, and
 * the launch-zone predictor flies exactly what the live missile flies.
 * Sensors: the doppler notch needs both beaming and look-down; chaff only
 * works inside the doppler gate. World: locks, launches, RWR levels,
 * warhead damage and bit-exact replay. Stores: separate heat-seeker and
 * radar-missile stores, weapon select, and the heat seeker's real lock:
 * IR signature by aspect, engine and airframe, gimbal and cueing. The launch
 * zone is a flyout of the missile itself.
 */
#include "fastjet/sim/world.hpp"
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>

using namespace fastjet;
using sim::MissileSpec;

namespace {

constexpr double DT = 0.005;
constexpr double G0 = fdm::SixDoFFDM::GRAVITY_ACCEL;

fdm::FlightState level(const math::Vector3& pos, double heading, double v) {
    fdm::FlightState s{};
    s.pos_ned = pos;
    s.vel_b = math::Vector3(v, 0.0, 0.0);
    s.q_att = math::Quaternion::from_euler(0.0, 0.0, heading);
    return s;
}

void test_motor_and_drag() {
    std::cout << "[MISSILE] Boost to Mach 3+, then a decaying coast...\n";
    const MissileSpec sp;
    sim::MissileBody m;
    m.pos = {0.0, 0.0, -5000.0};
    m.vel = {250.0, 0.0, 0.0};
    double v_burnout = 0.0;
    for (double t = 0.0; t < 40.0; t += DT) {
        m.step(sp, DT, sim::hold_weight());
        if (v_burnout == 0.0 && !m.motor_burning(sp)) v_burnout = m.speed();
    }
    const double a = environment::Atmosphere1976::compute(5000.0).speed_of_sound;
    std::cout << "    burnout Mach " << v_burnout / a << ", after 40 s " << m.speed() / a << ", mass "
              << m.mass(sp) << " kg\n";
    assert(v_burnout / a > 2.8 && v_burnout / a < 3.8);
    assert(m.speed() < 0.6 * v_burnout);          // drag bleeds the coast
    assert(std::abs(m.mass(sp) - sp.burnout_mass_kg) < 1e-9);
    // Weight held up: after the autopilot's first lag it flies level (under 1 deg).
    assert(std::abs(m.vel.z) < std::sin(M_PI / 180.0) * m.speed());
    std::cout << "  -> PASSED\n";
}

void test_lateral_limits() {
    std::cout << "[MISSILE] Turn capability: 40 G when fast, lift-limited when slow...\n";
    const MissileSpec sp;
    for (const double v : {1000.0, 350.0}) {
        sim::MissileBody m;
        m.pos = {0.0, 0.0, -5000.0};
        m.vel = {v, 0.0, 0.0};
        m.tof = sp.burn_time_s + 1.0; // coasting
        const double a_max = m.max_lateral_accel(sp);
        for (int i = 0; i < 200; ++i) m.step(sp, DT, {0.0, 1000.0 * G0, 0.0});
        const double a_lat = m.a_lat.norm();
        std::cout << "    " << v << " m/s: limit " << a_max / G0 << " G, achieved " << a_lat / G0 << " G\n";
        assert(a_lat <= m.max_lateral_accel(sp) + 1e-6 + 0.5 * G0); // limit falls as it slows
        if (v > 900.0) assert(std::abs(a_max - sp.g_limit * G0) < 1e-6);
        else assert(a_max < 0.5 * sp.g_limit * G0);
    }
    // Pulling G costs speed: induced drag.
    sim::MissileBody straight, turning;
    straight.pos = turning.pos = {0.0, 0.0, -5000.0};
    straight.vel = turning.vel = {700.0, 0.0, 0.0};
    straight.tof = turning.tof = 10.0;
    for (int i = 0; i < 400; ++i) {
        straight.step(sp, DT, sim::hold_weight());
        turning.step(sp, DT, {0.0, 25.0 * G0, -G0});
    }
    std::cout << "    2 s coasting: " << straight.speed() << " m/s straight, " << turning.speed() << " m/s at 25 G\n";
    assert(turning.speed() < straight.speed() - 30.0);
    std::cout << "  -> PASSED\n";
}

void test_pro_nav_intercept() {
    std::cout << "[MISSILE] PN intercepts a crossing, non-manoeuvring target...\n";
    const MissileSpec sp;
    sim::MissileBody m;
    m.pos = {0.0, 0.0, -5000.0};
    m.vel = {250.0, 0.0, 0.0};
    math::Vector3 tp{9000.0, -3000.0, -5200.0};
    const math::Vector3 tv{0.0, 250.0, 0.0};
    double best = 1e9;
    for (double t = 0.0; t < 40.0; t += DT) {
        const math::Vector3 m0 = m.pos, t0 = tp;
        m.step(sp, DT, m.tof < sim::RAIL_CLEAR_S ? sim::hold_weight() : sim::pro_nav(m, tp, tv, sp.nav_gain));
        tp += tv * DT;
        double s = 0.0;
        best = std::min(best, sim::closest_approach(m0, m.pos, t0, tp, s));
    }
    std::cout << "    miss distance " << best << " m\n";
    assert(best < 2.0);
    std::cout << "  -> PASSED\n";
}

void test_predictor_matches_live_missile() {
    std::cout << "[MISSILE] Launch-zone flyout = the live missile's own guidance and flight...\n";
    const MissileSpec sp;
    const math::Vector3 lp{0.0, 0.0, -6000.0}, lv{280.0, 0.0, 0.0};
    const math::Vector3 tp0{12000.0, 2000.0, -5500.0}, tv{-200.0, -120.0, 0.0};
    const sim::FlyoutResult f = sim::predict_flyout(sp, lp, lv, tp0, tv, sim::FlyoutTarget::STRAIGHT, DT);

    sim::Missile live;
    live.spec = sp;
    live.body.pos = lp;
    live.body.vel = lv;
    live.est_pos = tp0;
    live.est_vel = tv;
    live.active = true;
    math::Vector3 tp = tp0;
    double t_hit = -1.0, miss = 1e9;
    while (live.body.tof < sp.max_tof_s && t_hit < 0.0) {
        const math::Vector3 m0 = live.body.pos, t0 = tp;
        live.update(DT, /*uplink=*/true, tp, tv);
        tp += tv * DT;
        double s = 0.0;
        const double d = sim::closest_approach(m0, live.body.pos, t0, tp, s);
        if (live.body.tof > sp.arm_time_s && d < 0.5 * sp.fuze_radius_m && s < 1.0) {
            t_hit = live.body.tof;
            miss = d;
        }
    }
    std::cout << "    flyout: hit " << f.hit << " at " << f.time_s << " s, miss " << f.miss_m << " m; live: "
              << t_hit << " s, miss " << miss << " m (" << sim::to_string(live.phase) << ")\n";
    assert(f.hit);
    assert(std::abs(t_hit - f.time_s) < 1e-9);
    assert(std::abs(miss - f.miss_m) < 1e-9);
    std::cout << "  -> PASSED\n";
}

void test_launch_zones() {
    std::cout << "[MISSILE] Launch zones: head-on > beam > tail, thin air > thick, Rne < Rmax...\n";
    const MissileSpec sp;
    auto zone = [&](double alt, math::Vector3 tv, sim::FlyoutTarget model) {
        double r_best = 0.0;
        for (double r = 2000.0; r < 70000.0; r += 1000.0) {
            const auto f = sim::predict_flyout(sp, {0.0, 0.0, -alt}, {250.0, 0.0, 0.0}, {r, 0.0, -alt}, tv, model);
            if (f.hit) r_best = r;
        }
        return r_best;
    };
    const double head = zone(5000.0, {-250.0, 0.0, 0.0}, sim::FlyoutTarget::STRAIGHT);
    const double beam = zone(5000.0, {0.0, 250.0, 0.0}, sim::FlyoutTarget::STRAIGHT);
    const double tail = zone(5000.0, {250.0, 0.0, 0.0}, sim::FlyoutTarget::STRAIGHT);
    const double tail_low = zone(1000.0, {250.0, 0.0, 0.0}, sim::FlyoutTarget::STRAIGHT);
    const double beam_rne = zone(5000.0, {0.0, 250.0, 0.0}, sim::FlyoutTarget::TURN_COLD);
    std::cout << "    5 km: head-on " << head / 1000 << " km, beam " << beam / 1000 << " km, tail " << tail / 1000
              << " km (1 km altitude: " << tail_low / 1000 << " km); beam Rne " << beam_rne / 1000 << " km\n";
    assert(head > beam && beam > tail);
    assert(tail_low < tail);
    assert(beam_rne < 0.8 * beam);
    assert(head > 30000.0 && tail > 10000.0 && tail < 30000.0);
    std::cout << "  -> PASSED\n";
}

void test_doppler_notch() {
    std::cout << "[SENSOR] The notch needs beaming AND ground in the beam...\n";
    sim::SensorLimits lim;
    const math::Vector3 sensor{0.0, 0.0, -6000.0};
    const math::Vector3 fwd{1.0, 0.0, 0.0};
    const math::Vector3 below{10000.0, 0.0, -4000.0}; // look-down
    const math::Vector3 above{10000.0, 0.0, -8000.0}; // look-up, 11 deg
    const math::Vector3 beaming{0.0, 250.0, 0.0};
    const math::Vector3 cold{250.0, 0.0, 0.0};
    assert(lim.check(sensor, fwd, below, beaming) == sim::TrackCheck::NOTCH);
    assert(lim.check(sensor, fwd, below, cold) == sim::TrackCheck::OK);
    assert(lim.check(sensor, fwd, above, beaming) == sim::TrackCheck::OK);
    assert(lim.check(sensor, fwd, {0.0, 10000.0, -6000.0}, cold) == sim::TrackCheck::GIMBAL);
    assert(lim.check(sensor, fwd, {40000.0, 0.0, -6000.0}, cold) == sim::TrackCheck::RANGE);
    std::cout << "  -> PASSED\n";
}

void test_chaff_gates() {
    std::cout << "[SENSOR] Chaff is in the gates only when the target has no radial velocity of its own...\n";
    const math::Vector3 sensor{0.0, 0.0, -5000.0};
    sim::ChaffCloud c;
    c.pos = {8000.0, 0.0, -5000.0};
    c.vel = {0.0, 0.0, 2.0};
    c.active = true;
    const math::Vector3 tgt{8000.0, 60.0, -5000.0};
    const double beam = 1.75 * M_PI / 180.0;
    assert(sim::chaff_in_gates(sensor, tgt, {0.0, 280.0, 0.0}, c, beam));   // beaming: stolen
    assert(!sim::chaff_in_gates(sensor, tgt, {280.0, 0.0, 0.0}, c, beam));  // running: doppler rejects
    assert(!sim::chaff_in_gates(sensor, {8000.0, 900.0, -5000.0}, {0.0, 280.0, 0.0}, c, beam)); // outside the beam
    std::cout << "  -> PASSED\n";
}

std::unique_ptr<sim::World> two_jets(double range, double target_heading) {
    auto w = std::make_unique<sim::World>();
    w->clear(11);
    w->add(aircraft::AircraftType::F16_FIGHTING_FALCON, level({0.0, 0.0, -5000.0}, 0.0, 250.0));
    w->add(aircraft::AircraftType::F16_FIGHTING_FALCON, level({range, 0.0, -5000.0}, target_heading, 250.0));
    for (int i = 0; i < 2; ++i) w->aircraft[static_cast<size_t>(i)].landing_gear.deployed = false;
    return w;
}

/// Straight and level at a steady lever, for both jets.
std::array<sim::AircraftControls, sim::World::kMaxAircraft> cruise() {
    std::array<sim::AircraftControls, sim::World::kMaxAircraft> c{};
    for (auto& k : c) k.throttle = 0.8;
    return c;
}

void test_radar_lock_and_rwr() {
    std::cout << "[RADAR] Acquire -> lock -> launch, heard on the target's RWR...\n";
    auto w = two_jets(12000.0, 0.0); // target 12 km ahead, flying away
    assert(w->rwr(1).level == sim::RwrStatus::Level::CLEAR);
    w->radars[0].enable(1, 1.0);
    w->stores[0].mrm = {MissileSpec{}, 2};
    auto c = cruise();
    w->step(DT, c);
    assert(w->rwr(1).level == sim::RwrStatus::Level::SEARCH);
    for (int i = 0; i < static_cast<int>(1.1 / DT); ++i) w->step(DT, c);
    const auto locked = w->rwr(1);
    assert(w->radars[0].locked());
    assert(locked.level == sim::RwrStatus::Level::LOCK);
    // The lock is behind the target: bearing near 180 deg.
    std::cout << "    lock heard at bearing " << locked.emitter_bearing * 180.0 / M_PI << " deg, symbol "
              << locked.emitter_symbol << "\n";
    assert(std::abs(std::abs(locked.emitter_bearing) - M_PI) < 0.1);
    assert(std::strcmp(locked.emitter_symbol, "16") == 0);

    assert(w->launch_missile(0, 1, sim::WeaponSel::MRM) >= 0);
    assert(w->stores[0].mrm.count == 1);
    w->step(DT, c);
    const auto launch = w->rwr(1);
    assert(launch.level == sim::RwrStatus::Level::LAUNCH && launch.missile_valid);
    // No lock, no launch: a missile needs a track to start from.
    w->radars[0].break_lock();
    assert(w->launch_missile(0, 1, sim::WeaponSel::MRM) < 0);
    std::cout << "  -> PASSED\n";
}

void test_lock_breaks_outside_gimbal() {
    std::cout << "[RADAR] A lock coasts on memory, then drops when the target leaves the gimbal...\n";
    auto w = two_jets(8000.0, 0.0);
    w->radars[0].enable(1, 0.5);
    auto c = cruise();
    for (int i = 0; i < static_cast<int>(1.0 / DT); ++i) w->step(DT, c);
    assert(w->radars[0].locked());
    // Put the target 90 deg off the nose.
    w->aircraft[1].state.pos_ned = w->aircraft[0].state.pos_ned + math::Vector3(0.0, 8000.0, 0.0);
    w->step(DT, c);
    assert(w->radars[0].locked() && w->radars[0].coasting);
    for (int i = 0; i < static_cast<int>((w->radars[0].spec.memory_s + 0.1) / DT); ++i) w->step(DT, c);
    assert(!w->radars[0].locked());
    assert(w->rwr(1).level == sim::RwrStatus::Level::CLEAR); // out of the scan volume too
    std::cout << "  -> PASSED\n";
}

void test_missile_kill_and_replay() {
    std::cout << "[WORLD] A missile at a straight-flying jet fuzes and kills it, bit-exactly on replay...\n";
    auto run = [] {
        auto w = two_jets(9000.0, 0.0);
        w->radars[0].enable(1, 0.5);
        w->stores[0].mrm = {MissileSpec{}, 1};
        auto c = cruise();
        bool launched = false;
        for (int i = 0; i < static_cast<int>(40.0 / DT); ++i) {
            if (!launched && w->radars[0].locked()) launched = w->launch_missile(0, 1, sim::WeaponSel::MRM) >= 0;
            w->step(DT, c);
        }
        return w;
    };
    const auto a = run();
    const auto b = run();
    int det = -1;
    for (int k = 0; k < a->event_count(); ++k) {
        if (a->event(k).kind == sim::CombatEvent::Kind::MISSILE_DETONATION) det = k;
    }
    assert(det >= 0);
    std::cout << "    detonated " << a->event(det).miss_m << " m from the target at t=" << a->event(det).time
              << " s; target alive: " << a->aircraft[1].alive() << "\n";
    assert(a->event(det).miss_m < a->missiles[0].spec.fuze_radius_m);
    assert(!a->aircraft[1].alive());
    assert(a->missiles_in_flight() == 0);
    assert(a->event_count() == b->event_count());
    for (int i = 0; i < 2; ++i) {
        const auto& sa = a->aircraft[static_cast<size_t>(i)].state;
        const auto& sb = b->aircraft[static_cast<size_t>(i)].state;
        assert(std::memcmp(&sa.pos_ned, &sb.pos_ned, sizeof(sa.pos_ned)) == 0);
    }
    std::cout << "  -> PASSED\n";
}

void test_chaff_dispenser() {
    std::cout << "[WORLD] Chaff: two bundles per press, never more than the load...\n";
    auto w = two_jets(8000.0, 0.0);
    auto c = cruise();
    const int load = w->chaff_load[1];
    for (int press = 0; press < 3; ++press) {
        c[1].dispense = true;
        w->step(DT, c);
        c[1].dispense = false;
        for (int i = 0; i < static_cast<int>(0.5 / DT); ++i) w->step(DT, c);
    }
    std::cout << "    3 presses: " << load - w->chaff_load[1] << " bundles\n";
    assert(load - w->chaff_load[1] == 3 * sim::World::CHAFF_PER_PROGRAM);
    // Holding the switch is one press.
    c[1].dispense = true;
    for (int i = 0; i < static_cast<int>(1.0 / DT); ++i) w->step(DT, c);
    assert(load - w->chaff_load[1] == 4 * sim::World::CHAFF_PER_PROGRAM);
    w->chaff_load[1] = 1;
    c[1].dispense = false;
    w->step(DT, c);
    c[1].dispense = true;
    for (int i = 0; i < static_cast<int>(1.0 / DT); ++i) w->step(DT, c);
    assert(w->chaff_load[1] == 0);
    std::cout << "  -> PASSED\n";
}

/// Bring a jet's engine to the lever's steady state (15 s of spool), as a fight's pre-spool does.
void spool(sim::Aircraft& a, double lever) {
    const auto air = environment::Atmosphere1976::compute(a.state.altitude(), a.state.airspeed());
    for (int i = 0; i < 3000; ++i) (void)a.engine.update(lever, air, sim::Aircraft::DT);
}

/// A world with a shooter at the origin heading north and a target at
/// @p rel (NED offset) heading @p target_heading, both at 250 m/s.
std::unique_ptr<sim::World> shooter_and_target(aircraft::AircraftType shooter, aircraft::AircraftType target,
                                               const math::Vector3& rel, double target_heading) {
    auto w = std::make_unique<sim::World>();
    w->clear(5);
    w->add(shooter, level({0.0, 0.0, -5000.0}, 0.0, 250.0));
    w->add(target, level(math::Vector3(0.0, 0.0, -5000.0) + rel, target_heading, 250.0));
    for (int i = 0; i < 2; ++i) w->aircraft[static_cast<size_t>(i)].landing_gear.deployed = false;
    return w;
}

/// Lock range of an AIM-9X against @p target seen from @p from (same altitude, @p lever on its engine).
double lock_range_from(aircraft::AircraftType target, double lever, const math::Vector3& from, double heading) {
    auto w = shooter_and_target(aircraft::AircraftType::F16_FIGHTING_FALCON, target, {0.0, 0.0, 0.0}, heading);
    sim::Aircraft& t = w->aircraft[1];
    spool(t, lever);
    return sim::missile_specs::aim9x().ir_lock_range(sim::ir_signature(t, t.position() + from));
}

void test_stores_and_weapon_select() {
    std::cout << "[STORES] Separate SRM and MRM stores; weapon select fires the right missile...\n";
    auto w = shooter_and_target(aircraft::AircraftType::F16_FIGHTING_FALCON, aircraft::AircraftType::F16_FIGHTING_FALCON,
                                {3000.0, 0.0, 0.0}, 0.0);
    spool(w->aircraft[1], 0.8);
    sim::WeaponStores& st = w->stores[0];
    st.load(sim::airframe_missile_loadout(aircraft::AircraftType::F16_FIGHTING_FALCON), true);
    assert(st.selected == sim::WeaponSel::GUN);
    assert(st.srm.spec.infrared() && !st.mrm.spec.infrared());
    const int srm0 = st.srm.count, mrm0 = st.mrm.count;
    assert(srm0 > 0 && mrm0 > 0);
    auto c = cruise();
    auto pickle = [&] {
        c[0].pickle = true;
        w->step(DT, c);
        c[0].pickle = false;
        w->step(DT, c);
    };
    // Gun selected: the pickle fires nothing, but the press is answered (GUN SEL cue).
    pickle();
    assert(w->missiles_in_flight() == 0 && w->pickle_presses[0] == 1);
    assert(w->launch_cue[0] == sim::World::LaunchCue::GUN);

    // SRM: the seeker uncages and locks the target's tail; the pickle fires a heat seeker.
    assert(st.cycle() == sim::WeaponSel::SRM);
    w->step(DT, c);
    assert(w->seekers[0].active && w->seekers[0].locked && w->seekers[0].target == 1);
    pickle();
    assert(w->launch_cue[0] == sim::World::LaunchCue::LAUNCH);
    assert(w->missiles_in_flight(0) == 1 && w->missiles[0].spec.infrared());
    assert(std::strcmp(w->missiles[0].spec.name, "AIM-9X") == 0);
    assert(st.srm.count == srm0 - 1 && st.mrm.count == mrm0);

    // MRM without a radar lock: NO LOCK, nothing fired.
    assert(st.cycle() == sim::WeaponSel::MRM);
    w->step(DT, c);
    assert(!w->seekers[0].active); // a radar missile selected cages the heat seeker
    pickle();
    assert(w->launch_cue[0] == sim::World::LaunchCue::NO_LOCK);
    assert(w->missiles_in_flight(0) == 1 && st.mrm.count == mrm0);
    // With the radar in STT the same press fires an AMRAAM.
    w->radars[0].enable(1, 0.2);
    for (int i = 0; i < static_cast<int>(0.4 / DT); ++i) w->step(DT, c);
    assert(w->radars[0].locked());
    pickle();
    assert(w->launch_cue[0] == sim::World::LaunchCue::LAUNCH);
    assert(w->missiles_in_flight(0) == 2 && st.mrm.count == mrm0 - 1 && st.srm.count == srm0 - 1);
    bool radar_missile = false;
    for (const auto& m : w->missiles) radar_missile |= m.active && !m.spec.infrared() && m.shooter == 0;
    assert(radar_missile);
    // A heat seeker cannot be fired at a radar lock without its own seeker's lock.
    assert(w->launch_missile(0, 1, sim::WeaponSel::SRM) < 0);

    // Weapon select skips empty and unfitted stores.
    assert(st.cycle() == sim::WeaponSel::GUN);
    st.srm.count = 0;
    assert(st.cycle() == sim::WeaponSel::MRM);
    sim::WeaponStores a10;
    a10.load(sim::airframe_missile_loadout(aircraft::AircraftType::A10_THUNDERBOLT), true);
    assert(a10.mrm.count == 0 && a10.srm.count == 2);
    assert(a10.cycle() == sim::WeaponSel::SRM && a10.cycle() == sim::WeaponSel::GUN);
    // Firing the last round of the selected store steps to the next weapon.
    w->stores[0].srm.count = 1;
    w->stores[0].select(sim::WeaponSel::SRM);
    w->step(DT, c);
    assert(w->seekers[0].locked);
    assert(w->launch_missile(0, 1, sim::WeaponSel::SRM) >= 0);
    assert(w->stores[0].srm.count == 0 && w->stores[0].selected == sim::WeaponSel::MRM);
    std::cout << "  -> PASSED\n";
}

void test_every_loadout_is_named() {
    std::cout << "[STORES] Every jet's loadout: heat seekers for all, radar missiles for the radar jets...\n";
    using T = aircraft::AircraftType;
    for (T t : {T::F16_FIGHTING_FALCON, T::F15EX_EAGLE_II, T::EUROFIGHTER_TYPHOON, T::F22_RAPTOR, T::A10_THUNDERBOLT,
                T::F35B_LIGHTNING_II}) {
        const auto l = sim::airframe_missile_loadout(t);
        std::cout << "    " << aircraft::to_short_string(t) << ": " << l.mrm_count << " x " << (l.mrm_count ? l.mrm_spec.name : "-")
                  << ", " << l.srm_count << " x " << l.srm_spec.name << "\n";
        assert(l.srm_count > 0 && l.srm_spec.infrared() && l.srm_spec.ir_lock_range_m > 0.0);
        assert((l.mrm_count > 0) == sim::radar_spec(t).fitted);
        if (l.mrm_count > 0) assert(!l.mrm_spec.infrared());
    }
    assert(std::strcmp(sim::airframe_missile_loadout(T::EUROFIGHTER_TYPHOON).mrm_spec.name, "METEOR") == 0);
    assert(std::strcmp(sim::airframe_missile_loadout(T::EUROFIGHTER_TYPHOON).srm_spec.name, "IRIS-T") == 0);
    assert(std::strcmp(sim::airframe_missile_loadout(T::A10_THUNDERBOLT).srm_spec.name, "AIM-9M") == 0);
    // The Meteor's ramjet keeps its speed: it out-ranges an AMRAAM in a tail chase.
    auto tail = [](const MissileSpec& sp) {
        double r_best = 0.0;
        for (double r = 2000.0; r < 60000.0; r += 1000.0) {
            if (sim::predict_flyout(sp, {0.0, 0.0, -8000.0}, {280.0, 0.0, 0.0}, {r, 0.0, -8000.0}, {250.0, 0.0, 0.0},
                                    sim::FlyoutTarget::STRAIGHT).hit) {
                r_best = r;
            }
        }
        return r_best;
    };
    const double r120c = tail(sim::missile_specs::aim120c()), r120d = tail(sim::missile_specs::aim120d());
    const double rmtr = tail(sim::missile_specs::meteor());
    std::cout << "    tail-chase reach at 8 km: AIM-120C " << r120c / 1000 << " km, AIM-120D " << r120d / 1000
              << " km, Meteor " << rmtr / 1000 << " km\n";
    assert(r120d > r120c && rmtr > r120d);
    std::cout << "  -> PASSED\n";
}

void test_ir_signature_aspect() {
    std::cout << "[IR] A seeker locks a jet's tail from further out than its nose...\n";
    using T = aircraft::AircraftType;
    // Target heading north: its tail looks south.
    const double tail = lock_range_from(T::F16_FIGHTING_FALCON, 0.8, {-1000.0, 0.0, 0.0}, 0.0);
    const double beam = lock_range_from(T::F16_FIGHTING_FALCON, 0.8, {0.0, 1000.0, 0.0}, 0.0);
    const double head = lock_range_from(T::F16_FIGHTING_FALCON, 0.8, {1000.0, 0.0, 0.0}, 0.0);
    std::cout << "    F-16 near military power: tail " << tail / 1000 << " km, beam " << beam / 1000 << " km, head-on "
              << head / 1000 << " km\n";
    assert(tail > beam && beam > head);
    assert(tail > 2.5 * head);
    // In the world: 6 km behind its tail the seeker locks; 6 km off its nose it cannot.
    for (const bool from_tail : {true, false}) {
        auto w = shooter_and_target(T::F16_FIGHTING_FALCON, T::F16_FIGHTING_FALCON, {6000.0, 0.0, 0.0},
                                    from_tail ? 0.0 : M_PI);
        spool(w->aircraft[1], 0.8);
        w->stores[0].load(sim::airframe_missile_loadout(T::F16_FIGHTING_FALCON), false);
        w->stores[0].select(sim::WeaponSel::SRM);
        auto c = cruise();
        w->step(DT, c);
        std::cout << "    6 km " << (from_tail ? "astern" : "head-on") << ": lock range "
                  << sim::missile_specs::aim9x().ir_lock_range(sim::ir_signature(w->aircraft[1], w->aircraft[0].position())) / 1000
                  << " km, " << (w->seekers[0].locked ? "LOCKED" : "no lock") << "\n";
        assert(w->seekers[0].active && w->seekers[0].locked == from_tail);
    }
    std::cout << "  -> PASSED\n";
}

void test_ir_signature_engine_and_airframe() {
    std::cout << "[IR] Reheat lengthens the lock range; a Raptor is harder to lock than an Eagle...\n";
    using T = aircraft::AircraftType;
    // From the beam: from astern full reheat would reach the seeker's own range limit.
    const math::Vector3 beam{0.0, 1000.0, 0.0};
    const double idle = lock_range_from(T::F16_FIGHTING_FALCON, 0.08, beam, 0.0);
    const double mil = lock_range_from(T::F16_FIGHTING_FALCON, 0.85, beam, 0.0);
    const double ab = lock_range_from(T::F16_FIGHTING_FALCON, 1.0, beam, 0.0);
    std::cout << "    F-16 from the beam: idle " << idle / 1000 << " km, MIL " << mil / 1000 << " km, full reheat "
              << ab / 1000 << " km\n";
    assert(idle < mil && mil < ab);
    assert(ab > 1.4 * mil);
    const math::Vector3 quarter{-700.0, 700.0, 0.0};
    const double f22 = lock_range_from(T::F22_RAPTOR, 0.85, quarter, 0.0);
    const double f15 = lock_range_from(T::F15EX_EAGLE_II, 0.85, quarter, 0.0);
    const double a10 = lock_range_from(T::A10_THUNDERBOLT, 1.0, quarter, 0.0);
    const double f35 = lock_range_from(T::F35B_LIGHTNING_II, 0.85, quarter, 0.0);
    std::cout << "    45 deg off the tail at MIL: F-22 " << f22 / 1000 << " km, F-35B " << f35 / 1000 << " km, F-15EX "
              << f15 / 1000 << " km; A-10 at full power " << a10 / 1000 << " km\n";
    assert(f22 < f35 && f35 < f15);
    assert(f22 < 0.7 * f15);
    assert(a10 < lock_range_from(T::F16_FIGHTING_FALCON, 0.85, quarter, 0.0));
    // A hot target is harder to decoy than a cold one.
    const double p_hot = sim::flare_seduction_chance(0.2, 2.5, 3.0);
    const double p_cold = sim::flare_seduction_chance(0.2, 2.5, 0.05);
    assert(p_cold > 1.5 * p_hot);
    std::cout << "  -> PASSED\n";
}

void test_seeker_gimbal_and_cueing() {
    std::cout << "[IR] No lock outside the gimbal; a lock drops when the target leaves it...\n";
    using T = aircraft::AircraftType;
    auto c = cruise();
    // 100 deg off the nose, 2 km, from its tail: inside lock range, outside the AIM-9X's 90 deg gimbal.
    {
        auto w = shooter_and_target(T::F16_FIGHTING_FALCON, T::F16_FIGHTING_FALCON,
                                    {2000.0 * std::cos(100.0 * M_PI / 180.0), 2000.0 * std::sin(100.0 * M_PI / 180.0), 0.0},
                                    100.0 * M_PI / 180.0);
        spool(w->aircraft[1], 1.0);
        w->stores[0].load(sim::airframe_missile_loadout(T::F16_FIGHTING_FALCON), false);
        w->stores[0].select(sim::WeaponSel::SRM);
        w->seekers[0].search_half_deg = 180.0; // even a helmet sight cannot cue it there
        w->step(DT, c);
        assert(w->seekers[0].active && !w->seekers[0].locked);
        c[0].pickle = true;
        w->step(DT, c);
        c[0].pickle = false;
        assert(w->launch_cue[0] == sim::World::LaunchCue::NO_LOCK && w->missiles_in_flight() == 0);
    }
    // 40 deg off: outside the pilot's HUD search, so no lock until the
    // seeker is cued there (a helmet sight, or slaved to a radar lock).
    {
        auto w = shooter_and_target(T::F16_FIGHTING_FALCON, T::F16_FIGHTING_FALCON,
                                    {3000.0 * std::cos(40.0 * M_PI / 180.0), 3000.0 * std::sin(40.0 * M_PI / 180.0), 0.0},
                                    0.0);
        spool(w->aircraft[1], 1.0);
        w->stores[0].load(sim::airframe_missile_loadout(T::F16_FIGHTING_FALCON), false);
        w->stores[0].select(sim::WeaponSel::SRM);
        w->step(DT, c);
        assert(!w->seekers[0].locked);
        w->radars[0].enable(1, 0.2);
        for (int i = 0; i < static_cast<int>(0.3 / DT); ++i) w->step(DT, c);
        assert(w->radars[0].locked() && w->seekers[0].locked && w->seekers[0].slaved);
        // Now put the target behind the 3-9 line: the lock breaks.
        w->aircraft[1].state.pos_ned = w->aircraft[0].position() + math::Vector3(-1500.0, 1500.0, 0.0);
        w->step(DT, c);
        assert(!w->seekers[0].locked);
    }
    std::cout << "  -> PASSED\n";
}

void test_launch_zone_flyout() {
    std::cout << "[DLZ] The launch zone is the missile's own flyout: inside Rmax it hits, beyond it misses...\n";
    for (const MissileSpec& sp : {sim::missile_specs::aim9x(), sim::missile_specs::aim120c()}) {
        fdm::FlightState launcher = level({0.0, 0.0, -6000.0}, 0.0, 280.0);
        fdm::FlightState target = level({15000.0, 2000.0, -6000.0}, M_PI, 250.0);
        const auto dlz = sim::DynamicLaunchZone::compute(launcher, target, sp);
        std::cout << "    " << sp.name << " head-on at 6 km: Rmin " << dlz.rmin_m / 1000 << " km, Rne " << dlz.rne_m / 1000
                  << " km, Rmax " << dlz.rmax_m / 1000 << " km\n";
        assert(dlz.rmin_m < dlz.rne_m && dlz.rne_m <= dlz.rmax_m);
        const math::Vector3 u = (target.pos_ned - launcher.pos_ned).normalized();
        auto hit_at = [&](double r) {
            return sim::predict_flyout(sp, launcher.pos_ned, launcher.velocity_ned(), launcher.pos_ned + u * r,
                                       target.velocity_ned(), sim::FlyoutTarget::STRAIGHT).hit;
        };
        assert(hit_at(0.98 * dlz.rmax_m));
        assert(!hit_at(1.05 * dlz.rmax_m));
    }
    // The AMRAAM reaches further than the Sidewinder, and both reach further
    // head-on than in a tail chase.
    const fdm::FlightState launcher = level({0.0, 0.0, -6000.0}, 0.0, 280.0);
    const fdm::FlightState head_on = level({15000.0, 0.0, -6000.0}, M_PI, 250.0);
    const fdm::FlightState going_away = level({15000.0, 0.0, -6000.0}, 0.0, 250.0);
    const auto t0 = std::chrono::steady_clock::now();
    const auto amraam = sim::DynamicLaunchZone::compute(launcher, head_on, sim::missile_specs::aim120c());
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const auto sidewinder = sim::DynamicLaunchZone::compute(launcher, head_on, sim::missile_specs::aim9x());
    const auto tail = sim::DynamicLaunchZone::compute(launcher, going_away, sim::missile_specs::aim9x());
    // A display spreads the same search over frames, a few flyouts at a time.
    sim::LaunchZoneSolver solver;
    solver.begin(launcher, head_on, sim::missile_specs::aim120c());
    int frames = 0;
    double worst_frame_ms = 0.0;
    for (bool done = false; !done; ++frames) {
        const auto f0 = std::chrono::steady_clock::now();
        done = solver.advance(4);
        worst_frame_ms = std::max(
            worst_frame_ms, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - f0).count());
    }
    std::cout << "    AIM-9X tail chase: Rmax " << tail.rmax_m / 1000 << " km; one AIM-120C launch zone: " << ms
              << " ms at once, or " << frames << " frames of at most " << worst_frame_ms << " ms\n";
    assert(solver.zone().rmax_m == amraam.rmax_m && solver.zone().rne_m == amraam.rne_m &&
           solver.zone().rmin_m == amraam.rmin_m);
    assert(amraam.rmax_m > 1.4 * sidewinder.rmax_m);
    assert(tail.rmax_m < 0.6 * sidewinder.rmax_m && tail.in_envelope(3000.0));
    std::cout << "  -> PASSED\n";
}

void test_no_radar_on_a10() {
    std::cout << "[RADAR] The A-10 has no air-to-air radar: nothing to lock, nothing to hear...\n";
    auto w = std::make_unique<sim::World>();
    w->clear(3);
    w->add(aircraft::AircraftType::A10_THUNDERBOLT, level({0.0, 0.0, -3000.0}, 0.0, 150.0));
    w->add(aircraft::AircraftType::F16_FIGHTING_FALCON, level({5000.0, 0.0, -3000.0}, 0.0, 200.0));
    w->radars[0].enable(1, 0.5);
    auto c = cruise();
    for (int i = 0; i < static_cast<int>(2.0 / DT); ++i) w->step(DT, c);
    assert(!w->radars[0].locked());
    assert(w->rwr(1).level == sim::RwrStatus::Level::CLEAR);
    std::cout << "  -> PASSED\n";
}

} // namespace

int main() {
    std::cout << "=========================================================\n";
    std::cout << "  Radar Missiles, Radar, Chaff and RWR                   \n";
    std::cout << "=========================================================\n";
    test_motor_and_drag();
    test_lateral_limits();
    test_pro_nav_intercept();
    test_predictor_matches_live_missile();
    test_launch_zones();
    test_doppler_notch();
    test_chaff_gates();
    test_radar_lock_and_rwr();
    test_lock_breaks_outside_gimbal();
    test_missile_kill_and_replay();
    test_chaff_dispenser();
    test_no_radar_on_a10();
    test_stores_and_weapon_select();
    test_every_loadout_is_named();
    test_ir_signature_aspect();
    test_ir_signature_engine_and_airframe();
    test_seeker_gimbal_and_cueing();
    test_launch_zone_flyout();
    std::cout << "\nAll missile and sensor tests passed successfully!\n";
    return 0;
}
