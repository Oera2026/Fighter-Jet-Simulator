/**
 * @file test_radar_scope.cpp
 * @brief The pilot's fire-control radar (FCR page): RWS search and STT.
 *
 * RWS: the stabilised 4-bar raster takes 4 x 120 deg / 65 deg/s per frame;
 * it paints one brick per jet per frame, where the jet was; it shows only
 * what the radar can see (scan volume, detection range, doppler notch).
 * STT: designating a brick locks the jet through the world's radar, so the
 * target's RWR hears it; a broken lock drops back to RWS without relocking
 * by itself, and a stale brick cannot be locked.
 * Narrower RWS scans refresh faster. ACM finds and locks a jet in a close
 * fight that the 4-bar RWS block never sees.
 * The 360 deg sweep display paints a jet behind the ownship once per turn
 * of its beam, where it was, and scales itself to the nearest jet.
 */
#include "fastjet/sim/radar_scope.hpp"
#include "fastjet/sim/sweep_scope.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>

using namespace fastjet;
using sim::RadarScope;

namespace {

constexpr double DT = 0.005;
constexpr double DEG = M_PI / 180.0;

fdm::FlightState level(const math::Vector3& pos, double heading, double v) {
    fdm::FlightState s{};
    s.pos_ned = pos;
    s.vel_b = math::Vector3(v, 0.0, 0.0);
    s.q_att = math::Quaternion::from_euler(0.0, 0.0, heading);
    return s;
}

/// Ownship at 5 km heading north at 250 m/s; the other jet at @p offset from it.
std::unique_ptr<sim::World> two_jets(const math::Vector3& offset, double target_heading,
                                     aircraft::AircraftType own_type = aircraft::AircraftType::F16_FIGHTING_FALCON) {
    auto w = std::make_unique<sim::World>();
    w->clear(7);
    const math::Vector3 own_pos(0.0, 0.0, -5000.0);
    w->add(own_type, level(own_pos, 0.0, 250.0));
    w->add(aircraft::AircraftType::F16_FIGHTING_FALCON, level(own_pos + offset, target_heading, 250.0));
    for (int i = 0; i < 2; ++i) w->aircraft[static_cast<size_t>(i)].landing_gear.deployed = false;
    return w;
}

std::array<sim::AircraftControls, sim::World::kMaxAircraft> cruise() {
    std::array<sim::AircraftControls, sim::World::kMaxAircraft> c{};
    for (auto& k : c) k.throttle = 0.8;
    return c;
}

void run(sim::World& w, RadarScope& scope, double seconds) {
    const auto c = cruise();
    for (int i = 0; i < static_cast<int>(seconds / DT + 0.5); ++i) {
        w.step(DT, c);
        scope.step(DT, w, 0);
    }
}

constexpr double FRAME_S = RadarScope::BARS * 2.0 * RadarScope::AZ_LIMIT_RAD / RadarScope::SCAN_RATE_RAD_S;

/// Bricks painted after one full frame (plus a margin) of scanning.
int bricks_after_a_frame(const math::Vector3& offset, double target_heading) {
    auto w = two_jets(offset, target_heading);
    RadarScope scope;
    run(*w, scope, FRAME_S + 0.2);
    return scope.hit_count;
}

void test_rws_paints_where_the_jet_was() {
    std::cout << "[FCR] RWS: one brick per frame, where the beam crossed the jet...\n";
    auto w = two_jets({30000.0, 0.0, 0.0}, M_PI); // head-on, closing at 500 m/s
    RadarScope scope;
    scope.sync(*w, 0);
    assert(scope.fitted() && scope.mode == RadarScope::Mode::RWS);

    // Time the first frame.
    const auto c = cruise();
    double t = 0.0;
    while (scope.frame == 0) {
        w->step(DT, c);
        scope.step(DT, *w, 0);
        t += DT;
        assert(t < 20.0);
    }
    std::cout << "    frame time " << t << " s (expected " << FRAME_S << ")\n";
    assert(std::abs(t - FRAME_S) < 0.02);
    assert(scope.hit_count == 1);
    const sim::RadarHit& h = scope.hits[0];
    std::cout << "    brick at az " << h.az_rad / DEG << " deg, " << h.range_m << " m, frame " << h.frame << "\n";
    assert(h.target == 1 && h.frame == 0);
    // On the line of sight, measured from where the ownship points: in the
    // 5 m/s crosswind a hands-off jet crabs a degree or more into the wind,
    // so the target is not assumed dead ahead.
    const math::Vector3 los = w->aircraft[1].state.pos_ned - w->aircraft[0].state.pos_ned;
    const double bearing = std::remainder(std::atan2(los.y, los.x) - w->aircraft[0].state.yaw(), 2.0 * M_PI);
    assert(std::abs(h.az_rad - bearing) < 1.0 * DEG);
    assert(h.range_m < 30000.0 && h.range_m > 30000.0 - 500.0 * FRAME_S);
    assert(std::abs(h.alt_m - 5000.0) < 100.0);

    // In trail at 15 km (the same jet flying the same way, so it holds its
    // range): one brick per frame, history ageing out after three frames.
    w = two_jets({15000.0, 0.0, 0.0}, 0.0);
    run(*w, scope, 4.9 * FRAME_S);
    std::cout << "    in trail after 4.9 frames: frame " << scope.frame << ", " << scope.hit_count << " bricks\n";
    assert(scope.frame == 4);
    assert(scope.hit_count == RadarScope::HISTORY_FRAMES);
    const sim::RadarHit* cur = scope.cursor_hit();
    assert(cur && cur->frame == scope.frame);
    for (int i = 0; i < scope.hit_count; ++i) {
        const sim::RadarHit& o = scope.hits[static_cast<size_t>(i)];
        std::cout << "      f" << o.frame << " " << o.range_m << " m, az " << o.az_rad / DEG << " deg" << std::endl;
        assert(o.frame >= scope.frame - 2 && std::abs(o.range_m - 15000.0) < 100.0);
    }
    std::cout << "  -> PASSED\n";
}

void test_rws_sees_only_what_the_radar_can() {
    std::cout << "[FCR] RWS shows only the scan volume, detection range and out-of-notch jets...\n";
    const int ahead = bricks_after_a_frame({15000.0, 0.0, 0.0}, M_PI);
    const int behind = bricks_after_a_frame({-10000.0, 0.0, 0.0}, 0.0);
    const int far = bricks_after_a_frame({60000.0, 0.0, 0.0}, M_PI);
    // 300 m below at 10 km: look-down, clutter in the beam. The beaming jet
    // crosses the nose mid-frame, so its radial speed stays inside the notch.
    const double cross = -0.5 * 250.0 * FRAME_S;
    const int beaming_low = bricks_after_a_frame({10000.0, cross, 300.0}, M_PI / 2.0);
    const int hot_low = bricks_after_a_frame({10000.0, 0.0, 300.0}, M_PI);
    // Beaming but 700 m above: no ground behind it, no notch.
    const int beaming_high = bricks_after_a_frame({10000.0, cross, -700.0}, M_PI / 2.0);
    std::cout << "    ahead " << ahead << ", behind " << behind << ", 60 km " << far << ", beaming look-down "
              << beaming_low << ", hot look-down " << hot_low << ", beaming look-up " << beaming_high << std::endl;
    assert(ahead == 1 && behind == 0 && far == 0);
    assert(beaming_low == 0 && hot_low == 1 && beaming_high == 1);

    // 4 km above at 10 km, in trail, is 22 deg up: outside the 4-bar block until the
    // pilot raises the antenna.
    auto w = two_jets({10000.0, 0.0, -4000.0}, 0.0);
    RadarScope scope;
    run(*w, scope, FRAME_S + 0.2);
    assert(scope.hit_count == 0);
    std::cout << "    block at 10 km: " << (5000.0 + 10000.0 * std::sin(scope.coverage_bottom_rad())) / 0.3048
              << " - " << (5000.0 + 10000.0 * std::sin(scope.coverage_top_rad())) / 0.3048 << " ft\n";
    scope.slew_elevation(1.0, 22.0 * DEG / RadarScope::ELEV_SLEW_RAD_S);
    run(*w, scope, FRAME_S + 0.2);
    std::cout << "    antenna up " << scope.el_centre / DEG << " deg: " << scope.hit_count << " brick\n";
    assert(scope.hit_count == 1);
    std::cout << "  -> PASSED\n";
}

void test_stt_is_a_lock_the_target_hears() {
    std::cout << "[FCR] Designate -> STT through the jet's radar; the target's RWR hears the lock...\n";
    auto w = two_jets({15000.0, 0.0, 0.0}, M_PI);
    RadarScope scope;
    scope.sync(*w, 0);
    assert(!scope.designate(*w)); // nothing on the scope yet
    run(*w, scope, FRAME_S + 0.2);
    assert(w->rwr(1).level == sim::RwrStatus::Level::CLEAR);
    assert(scope.designate(*w));
    run(*w, scope, 0.15);
    assert(scope.mode == RadarScope::Mode::STT && w->radars[0].locked());
    assert(scope.locked_target(*w) == 1 && scope.hit_count == 0);
    assert(w->rwr(1).level == sim::RwrStatus::Level::LOCK);
    // The antenna follows the target.
    double az = 0.0, el = 0.0;
    RadarScope::stabilised_angles(w->aircraft[0].state, w->aircraft[1].position(), az, el);
    assert(std::abs(scope.ant_az - az) < 0.1 * DEG && std::abs(scope.ant_el - el) < 0.1 * DEG);

    scope.undesignate(*w);
    assert(scope.mode == RadarScope::Mode::RWS && !w->radars[0].locked());
    run(*w, scope, 0.05);
    assert(w->rwr(1).level == sim::RwrStatus::Level::CLEAR);
    std::cout << "  -> PASSED\n";
}

void test_broken_lock_needs_the_pilot() {
    std::cout << "[FCR] A broken lock drops to RWS and stays off until the pilot designates again...\n";
    auto w = two_jets({12000.0, 0.0, 0.0}, M_PI);
    RadarScope scope;
    run(*w, scope, FRAME_S + 0.2);
    assert(scope.designate(*w));
    run(*w, scope, 0.15);
    assert(scope.mode == RadarScope::Mode::STT);
    // 90 deg off the nose: memory, then the lock breaks.
    const math::Vector3 own = w->aircraft[0].position();
    w->aircraft[1].state.pos_ned = own + math::Vector3(0.0, 8000.0, 0.0);
    run(*w, scope, 0.15);
    assert(scope.mode == RadarScope::Mode::STT && w->radars[0].coasting);
    run(*w, scope, w->radars[0].spec.memory_s + 0.1);
    assert(scope.mode == RadarScope::Mode::RWS);
    assert(w->radars[0].mode == sim::FireControlRadar::Mode::OFF);
    // Back in front of the nose: no lock without a new designation.
    w->aircraft[1].state.pos_ned = w->aircraft[0].position() + math::Vector3(10000.0, 0.0, 0.0);
    run(*w, scope, 1.0);
    assert(!w->radars[0].locked() && w->rwr(1).level == sim::RwrStatus::Level::CLEAR);

    // A stale brick: the jet has gone behind since it was painted.
    run(*w, scope, FRAME_S + 0.2);
    assert(scope.hit_count >= 1);
    w->aircraft[1].state.pos_ned = w->aircraft[0].position() + math::Vector3(-10000.0, 0.0, 0.0);
    assert(scope.designate(*w));
    run(*w, scope, RadarScope::LOCK_TIMEOUT_S + 0.05);
    assert(scope.mode == RadarScope::Mode::RWS && scope.lock_failed_cue);
    assert(w->radars[0].mode == sim::FireControlRadar::Mode::OFF);
    std::cout << "  -> PASSED\n";
}

void test_range_scales() {
    std::cout << "[FCR] Range scale: off-scale bricks cannot be designated; STT auto-ranges...\n";
    auto w = two_jets({26000.0, 0.0, 0.0}, 0.0); // 14 nm, going away
    RadarScope scope;
    run(*w, scope, FRAME_S + 0.2);
    assert(scope.hit_count == 1);
    scope.range_down(); // 10 nm
    assert(scope.cursor_hit() == nullptr && !scope.designate(*w));
    scope.range_up();   // 20 nm
    assert(scope.designate(*w));
    run(*w, scope, 0.15);
    assert(scope.mode == RadarScope::Mode::STT);
    scope.range_down();
    run(*w, scope, DT);
    assert(scope.range_index == 2); // bumped back out to 20 nm
    for (int i = 0; i < 6; ++i) scope.range_up();
    assert(scope.range_index == 4);
    run(*w, scope, 2.0 * DT);
    assert(scope.range_index == 2); // 14 nm is under 40% of 80 and 40 nm: stepped down to 20
    for (int i = 0; i < 6; ++i) scope.range_down();
    assert(scope.range_index == 0 && scope.range_scale_m() == 5.0 * 1852.0);
    std::cout << "  -> PASSED\n";
}

void test_no_radar() {
    std::cout << "[FCR] The A-10 has no radar: no scope, nothing to lock...\n";
    auto w = two_jets({12000.0, 0.0, 0.0}, M_PI, aircraft::AircraftType::A10_THUNDERBOLT);
    RadarScope scope;
    run(*w, scope, FRAME_S + 0.2);
    assert(!scope.fitted() && scope.mode == RadarScope::Mode::OFF && scope.hit_count == 0);
    assert(!scope.designate(*w));
    std::cout << "  -> PASSED\n";
}

void test_new_world_resets_the_scope() {
    std::cout << "[FCR] A new fight or a reset gives a cold scope...\n";
    auto w = two_jets({12000.0, 0.0, 0.0}, M_PI);
    RadarScope scope;
    run(*w, scope, FRAME_S + 0.2);
    assert(scope.designate(*w));
    run(*w, scope, 0.15);
    assert(scope.mode == RadarScope::Mode::STT);
    scope.range_up();
    const int range = scope.range_index;
    w = two_jets({12000.0, 0.0, 0.0}, M_PI);
    scope.sync(*w, 0);
    assert(scope.mode == RadarScope::Mode::RWS && scope.hit_count == 0 && scope.frame == 0);
    assert(scope.range_index == range); // the pilot's settings stay
    std::cout << "  -> PASSED\n";
}

void test_narrow_rws_scan() {
    std::cout << "[FCR] RWS azimuth A6 / A3 / A1: narrower scans refresh faster...\n";
    auto w = two_jets({15000.0, 0.0, 0.0}, 0.0);
    RadarScope scope;
    scope.sync(*w, 0);
    const double frame_s[3] = {FRAME_S, FRAME_S / 2.0, FRAME_S / 6.0};
    for (int k = 0; k < 3; ++k) {
        assert(scope.rws_az_index == k);
        const auto c = cruise();
        double t = 0.0;
        while (scope.frame == 0) {
            w->step(DT, c);
            scope.step(DT, *w, 0);
            t += DT;
            assert(t < 20.0);
        }
        std::cout << "    A" << static_cast<int>(RadarScope::kRwsAzLimitsDeg[static_cast<size_t>(k)] / 10.0)
                  << ": frame " << t << " s, " << scope.hit_count << " brick\n";
        assert(std::abs(t - frame_s[k]) < 0.02);
        assert(scope.hit_count == 1); // dead ahead: in every scan
        scope.cycle_scan();
        assert(scope.frame == 0 && scope.hit_count == 0);
    }
    assert(scope.rws_az_index == 0);
    // 20 deg off the nose is outside A1.
    w = two_jets({15000.0 * std::cos(20.0 * DEG), 15000.0 * std::sin(20.0 * DEG), 0.0}, 0.0);
    scope.cycle_scan();
    scope.cycle_scan();
    run(*w, scope, 3.0 * FRAME_S / 6.0);
    assert(scope.rws_az_index == 2 && scope.frame >= 2 && scope.hit_count == 0);
    std::cout << "  -> PASSED\n";
}

/// Pitch the ownship up by @p pitch (level wings) and keep it there.
void set_pitch(sim::World& w, double pitch) {
    w.aircraft[0].state.q_att = math::Quaternion::from_euler(0.0, pitch, 0.0);
}

void test_acm_auto_acquires_in_a_close_fight() {
    std::cout << "[FCR] ACM: nose-referenced scans lock the bandit that RWS never sees...\n";
    // 1.5 km ahead and 600 m above: 22 deg up, well outside the 4-bar block.
    const math::Vector3 above(1500.0, 0.0, -600.0);
    {
        auto w = two_jets(above, 0.0);
        RadarScope scope;
        run(*w, scope, FRAME_S + 0.2);
        assert(scope.hit_count == 0 && !w->radars[0].locked());
    }
    // ACM 20x30 with the jet above the HUD: nothing. Pull the nose onto it: locked.
    auto w = two_jets(above, 0.0);
    RadarScope scope;
    scope.sync(*w, 0);
    scope.range_up(); // pilot's CRM scale, 40 nm
    scope.toggle_master_mode(*w);
    assert(scope.acm && scope.mode == RadarScope::Mode::ACM);
    assert(scope.range_index == RadarScope::ACM_RANGE_INDEX);
    assert(scope.acm_scan == RadarScope::AcmScan::HUD_20X30);
    const double up = std::atan2(600.0, 1500.0);
    run(*w, scope, 2.0);
    assert(!w->radars[0].locked() && scope.mode == RadarScope::Mode::ACM);

    // 10x60 reaches 50 deg up the lift vector: it finds the jet with the nose level.
    scope.cycle_scan();
    assert(scope.acm_scan == RadarScope::AcmScan::VERTICAL_10X60);
    double t = 0.0;
    while (scope.mode != RadarScope::Mode::STT) {
        w->aircraft[1].state.pos_ned = w->aircraft[0].position() + above; // hold the geometry
        run(*w, scope, DT);
        t += DT;
        assert(t < 3.0);
    }
    std::cout << "    10x60: locked " << up / DEG << " deg up after " << t << " s\n";
    assert(scope.locked_target(*w) == 1 && w->rwr(1).level == sim::RwrStatus::Level::LOCK);
    assert(scope.range_index == 0); // 1 nm: STT steps down to the 5 nm scale

    // TMS aft goes back to the ACM scan, not RWS.
    scope.undesignate(*w);
    assert(scope.mode == RadarScope::Mode::ACM && !w->radars[0].locked());

    // 20x30 with the nose pulled 15 deg up puts the jet in the HUD.
    scope.cycle_scan();
    scope.cycle_scan();
    assert(scope.acm_scan == RadarScope::AcmScan::HUD_20X30);
    t = 0.0;
    while (scope.mode != RadarScope::Mode::STT) {
        set_pitch(*w, up - 7.0 * DEG);
        w->aircraft[1].state.pos_ned = w->aircraft[0].position() + above;
        run(*w, scope, DT);
        t += DT;
        assert(t < 3.0);
    }
    std::cout << "    20x30: locked after " << t << " s\n";

    // BORE: only a jet on the nose.
    scope.undesignate(*w);
    scope.cycle_scan();
    scope.cycle_scan();
    assert(scope.acm_scan == RadarScope::AcmScan::BORE);
    for (int i = 0; i < 100; ++i) {
        set_pitch(*w, up - 5.0 * DEG);
        w->aircraft[1].state.pos_ned = w->aircraft[0].position() + above;
        run(*w, scope, DT);
    }
    assert(scope.mode == RadarScope::Mode::ACM);
    for (int i = 0; i < 100; ++i) {
        set_pitch(*w, up);
        w->aircraft[1].state.pos_ned = w->aircraft[0].position() + above;
        run(*w, scope, DT);
    }
    assert(scope.mode == RadarScope::Mode::STT);
    std::cout << "    BORE: locked on the nose only\n";

    // Back to CRM: the lock goes and the pilot's range scale returns.
    scope.toggle_master_mode(*w);
    assert(!scope.acm && scope.mode == RadarScope::Mode::RWS && !w->radars[0].locked());
    assert(scope.range_index == RadarScope::DEFAULT_RANGE_INDEX + 1);
    std::cout << "  -> PASSED\n";
}

void test_acm_range_gate() {
    std::cout << "[FCR] ACM ignores jets beyond 10 nm...\n";
    auto w = two_jets({25000.0, 0.0, 0.0}, 0.0); // 13.5 nm dead ahead
    RadarScope scope;
    scope.sync(*w, 0);
    scope.set_master_mode(*w, true); // the dogfight switch
    scope.set_master_mode(*w, true); // already there: no change
    assert(scope.acm && scope.mode == RadarScope::Mode::ACM);
    scope.cycle_scan();
    scope.cycle_scan(); // BORE, right on it
    run(*w, scope, 2.0);
    assert(scope.mode == RadarScope::Mode::ACM && !w->radars[0].locked());
    // Inside 10 nm on the boresight, wherever the nose has trimmed to.
    const math::Vector3 nose = w->aircraft[0].state.q_att.rotate_body_to_ned(math::Vector3(1.0, 0.0, 0.0));
    w->aircraft[1].state.pos_ned = w->aircraft[0].position() + nose * 15000.0;
    run(*w, scope, 0.3);
    assert(scope.mode == RadarScope::Mode::STT);
    std::cout << "  -> PASSED\n";
}

} // namespace

void test_sweep_paints_all_round_once_a_turn() {
    std::cout << "[SWEEP] The beam paints a jet in the six once a turn, where it was...\n";
    using sim::SweepScope;
    // Directly astern at 0.5 nm: out of the FCR's reach, 180 deg round the beam.
    auto w = two_jets(math::Vector3(-926.0, 0.0, 0.0), 0.0);
    SweepScope sweep;
    const auto c = cruise();
    auto fly = [&](double seconds) {
        for (int i = 0; i < static_cast<int>(seconds / DT + 0.5); ++i) {
            w->step(DT, c);
            sweep.update(*w, 0, DT);
        }
    };
    fly(0.45 * SweepScope::PERIOD_S);
    assert(sweep.blip_count == 0); // The beam has not reached 6 o'clock yet
    fly(0.1 * SweepScope::PERIOD_S);
    assert(sweep.blip_count == 1 && sweep.blips[0].id == 1);
    const double painted = sweep.blips[0].painted_s;
    const math::Vector3 at_paint = sweep.blips[0].pos;
    std::cout << "    painted at " << painted << " s, bearing "
              << SweepScope::relative_bearing(w->aircraft[0].state, at_paint) / DEG << " deg, scale "
              << SweepScope::SCALES_NM[static_cast<size_t>(sweep.scale)] << " nm\n";
    assert(std::abs(painted - 0.5 * SweepScope::PERIOD_S) < 0.02);
    assert(std::abs(SweepScope::relative_bearing(w->aircraft[0].state, at_paint) / DEG - 180.0) < 3.0);
    assert(SweepScope::SCALES_NM[static_cast<size_t>(sweep.scale)] == 1.0); // Fits a half-mile fight

    // Between paints the blip holds where the jet was; one turn later it moves on.
    fly(0.5 * SweepScope::PERIOD_S);
    assert(sweep.blips[0].painted_s == painted && sweep.blips[0].pos.x == at_paint.x);
    assert(sweep.age(sweep.blips[0]) > 0.4);
    fly(0.55 * SweepScope::PERIOD_S);
    assert(sweep.blip_count == 1 && std::abs(sweep.blips[0].painted_s - painted - SweepScope::PERIOD_S) < 0.05);
    assert(sweep.blips[0].pos.x > at_paint.x + 400.0); // 250 m/s for a turn

    // A jet that goes down fades out and is gone within two turns.
    w->aircraft[1].crashed = true;
    fly(2.0 * SweepScope::PERIOD_S);
    assert(sweep.blip_count == 0);
    std::cout << "  -> PASSED\n";
}

void test_sweep_scale_follows_the_nearest_jet() {
    std::cout << "[SWEEP] The range scale fits the nearest jet, with hysteresis...\n";
    using sim::SweepScope;
    auto scale_for = [](double nm) {
        auto w = two_jets(math::Vector3(0.0, nm * 1852.0, 0.0), 0.0);
        SweepScope sweep;
        sweep.update(*w, 0, DT);
        return SweepScope::SCALES_NM[static_cast<size_t>(sweep.scale)];
    };
    assert(scale_for(0.5) == 1.0);
    assert(scale_for(1.5) == 2.0);
    assert(scale_for(3.0) == 5.0);
    assert(scale_for(10.0) == 20.0); // Evade's start, with room to spare
    assert(scale_for(60.0) == 40.0); // Past the largest: on the rim

    // Once set, it changes only as the beam passes the nose and past the band.
    auto w = two_jets(math::Vector3(0.0, 0.5 * 1852.0, 0.0), 0.0);
    SweepScope sweep;
    sweep.update(*w, 0, DT);
    w->aircraft[1].state.pos_ned.y = 0.93 * 1852.0; // Inside the 1 nm hysteresis band
    for (int i = 0; i < static_cast<int>(1.5 * SweepScope::PERIOD_S / DT); ++i) sweep.update(*w, 0, DT);
    assert(SweepScope::SCALES_NM[static_cast<size_t>(sweep.scale)] == 1.0);
    w->aircraft[1].state.pos_ned.y = 1.2 * 1852.0;
    sweep.update(*w, 0, 0.1 * SweepScope::PERIOD_S);
    assert(SweepScope::SCALES_NM[static_cast<size_t>(sweep.scale)] == 1.0); // Not mid-turn
    for (int i = 0; i < static_cast<int>(SweepScope::PERIOD_S / DT); ++i) sweep.update(*w, 0, DT);
    assert(SweepScope::SCALES_NM[static_cast<size_t>(sweep.scale)] == 2.0);
    std::cout << "  -> PASSED\n";
}

int main() {
    std::cout << "=========================================================\n";
    std::cout << "  Fire-Control Radar Scope                               \n";
    std::cout << "=========================================================\n";
    test_rws_paints_where_the_jet_was();
    test_rws_sees_only_what_the_radar_can();
    test_stt_is_a_lock_the_target_hears();
    test_broken_lock_needs_the_pilot();
    test_range_scales();
    test_no_radar();
    test_new_world_resets_the_scope();
    test_narrow_rws_scan();
    test_acm_auto_acquires_in_a_close_fight();
    test_acm_range_gate();
    test_sweep_paints_all_round_once_a_turn();
    test_sweep_scale_follows_the_nearest_jet();
    std::cout << "\nAll radar scope tests passed successfully!\n";
    return 0;
}
