/**
 * @file test_guns.cpp
 * @brief Guns and damage: ballistics against analytic solutions, gun rate and
 * spin-up, swept hit detection at high closure, the gunsight's honesty (the
 * predicted round is the round that flies) and component damage effects.
 */
#include "fastjet/sim/world.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>

using namespace fastjet;
using aircraft::AircraftType;

namespace {

constexpr double DT = 0.005;

fdm::FlightState level(const math::Vector3& pos, double heading, double v) {
    fdm::FlightState s{};
    s.pos_ned = pos;
    s.vel_b = math::Vector3(v, 0.0, 0.0);
    s.q_att = math::Quaternion::from_euler(0.0, 0.0, heading);
    return s;
}

void test_vacuum_trajectory_is_a_parabola() {
    std::cout << "[GUNS] Drag-free round follows the analytic parabola...\n";
    sim::GunSpec g = sim::gun_spec(AircraftType::F16_FIGHTING_FALCON);
    g.drag_coeff = 0.0;
    const fdm::FlightState shooter = level({0.0, 0.0, -3000.0}, 0.3, 200.0);
    math::Vector3 p0, v0;
    sim::Ballistics::launch(shooter, g, sim::gun_boresight_b(g), p0, v0);
    for (double t : {0.5, 1.0, 2.0}) {
        const math::Vector3 p = sim::Ballistics::predict(shooter, g, sim::gun_boresight_b(g), t);
        const math::Vector3 exact = p0 + v0 * t + math::Vector3(0.0, 0.0, 0.5 * sim::Ballistics::G * t * t);
        const double err = (p - exact).norm();
        std::cout << "    t=" << t << " s: error " << err << " m\n";
        assert(err < 1e-6);
    }
    std::cout << "  -> PASSED\n";
}

void test_drag_decay_matches_analytic() {
    std::cout << "[GUNS] Velocity decay under quadratic drag matches v0 / (1 + k rho v0 t)...\n";
    for (AircraftType type : {AircraftType::F16_FIGHTING_FALCON, AircraftType::EUROFIGHTER_TYPHOON,
                              AircraftType::A10_THUNDERBOLT, AircraftType::F35B_LIGHTNING_II}) {
        const sim::GunSpec g = sim::gun_spec(type);
        const double k = sim::Ballistics::drag_factor(g);
        math::Vector3 pos{0.0, 0.0, 0.0};
        math::Vector3 vel{g.muzzle_velocity_mps, 0.0, 0.0};
        const double rho = environment::Atmosphere1976::compute(0.0).density;
        const double t_end = 1.0;
        for (int i = 0; i < static_cast<int>(t_end / DT); ++i) sim::Ballistics::step(pos, vel, k, DT);
        const double v_exact = g.muzzle_velocity_mps / (1.0 + k * rho * g.muzzle_velocity_mps * t_end);
        const double x_exact = std::log(1.0 + k * rho * g.muzzle_velocity_mps * t_end) / (k * rho);
        std::cout << "    " << g.name << ": v(1 s) " << vel.x << " m/s (analytic " << v_exact << "), range "
                  << pos.x << " m (analytic " << x_exact << ")\n";
        // Gravity bends the path slightly, so compare along-track speed loosely.
        assert(std::abs(vel.x - v_exact) / v_exact < 0.005);
        assert(std::abs(pos.x - x_exact) / x_exact < 0.005);
        // Heavier 30 mm rounds hold their speed better than 20 mm.
        if (type == AircraftType::A10_THUNDERBOLT) assert(vel.x / g.muzzle_velocity_mps > 0.65);
    }
    std::cout << "  -> PASSED\n";
}

void test_rate_of_fire_and_spin_up() {
    std::cout << "[GUNS] Cyclic rate after spin-up, ammunition limit...\n";
    for (AircraftType type : {AircraftType::F16_FIGHTING_FALCON, AircraftType::EUROFIGHTER_TYPHOON,
                              AircraftType::A10_THUNDERBOLT, AircraftType::F35B_LIGHTNING_II}) {
        sim::GunSystem gun;
        gun.configure(type);
        int fired = 0;
        const double burst = 1.0;
        for (int i = 0; i < static_cast<int>(burst / DT); ++i) fired += gun.update(DT, true);
        // Linear spin-up loses half the spin-up time at full rate.
        const double expected = gun.spec.rate_rpm / 60.0 * (burst - 0.5 * gun.spec.spin_up_s);
        std::cout << "    " << gun.spec.name << ": " << fired << " rounds in 1 s (expected ~" << expected << ")\n";
        assert(std::abs(fired - expected) <= 2.0);

        // Hold until empty: never more than the magazine.
        for (int i = 0; i < 60 * 200; ++i) fired += gun.update(DT, true);
        assert(gun.ammo == 0);
        assert(fired == gun.spec.rounds);
        assert(gun.update(DT, true) == 0);
    }
    std::cout << "  -> PASSED\n";
}

void test_swept_hit_catches_crossing_shot() {
    std::cout << "[GUNS] Crossing shot at 1,050 m/s cannot tunnel through a 1.8 m fuselage...\n";
    const auto spec = sim::airframe_combat_spec(AircraftType::F16_FIGHTING_FALCON);
    // Target heading north at 350 m/s; a round heading east at 1,050 m/s moves
    // 5.25 m per step, three fuselage widths. Both step samples are clear of
    // the airframe; only the swept path crosses it.
    const fdm::FlightState prev = level({0.0, 0.0, -3000.0}, 0.0, 350.0);
    fdm::FlightState now = prev;
    now.pos_ned.x += 350.0 * DT;
    const math::Vector3 p0 = prev.pos_ned + math::Vector3(3.0, -2.6, 0.0);
    const math::Vector3 p1 = p0 + math::Vector3(0.0, 1050.0 * DT, 0.0);
    const math::Vector3 r1 = p1 - now.pos_ned;
    assert(std::abs(p0.y - prev.pos_ned.y) > spec.fuselage_radius_m && std::abs(r1.y) > spec.fuselage_radius_m);
    math::Vector3 hit_b;
    const bool hit = sim::World::sweep_hit(p0, p1, prev, now, spec, hit_b);
    std::cout << "    round moved " << (p1 - p0).norm() << " m this step; hit at body (" << hit_b.x << ", "
              << hit_b.y << ") m\n";
    assert(hit);
    assert(sim::classify_hit(hit_b, spec) == sim::HitZone::FUSELAGE);

    // The same shot 3 m below the jet is a miss.
    const math::Vector3 q0 = p0 + math::Vector3(0.0, 0.0, 3.0);
    const math::Vector3 q1 = p1 + math::Vector3(0.0, 0.0, 3.0);
    assert(!sim::World::sweep_hit(q0, q1, prev, now, spec, hit_b));

    // Head-on, 1,400 m/s closure, 7 m per step: entering the nose is caught.
    const math::Vector3 h0 = prev.pos_ned + math::Vector3(12.0, 0.3, 0.0);
    const math::Vector3 h1 = h0 + math::Vector3(-1050.0 * DT, 0.0, 0.0);
    assert(sim::World::sweep_hit(h0, h1, prev, now, spec, hit_b));
    assert(sim::classify_hit(hit_b, spec) == sim::HitZone::COCKPIT);
    std::cout << "  -> PASSED\n";
}

void test_predictor_is_the_round_that_flies() {
    std::cout << "[GUNS] Gunsight prediction matches the live round exactly...\n";
    auto world = std::make_unique<sim::World>();
    world->clear(3);
    world->dispersion = false;
    world->add(AircraftType::EUROFIGHTER_TYPHOON, level({0.0, 0.0, -4000.0}, 0.0, 240.0));
    world->aircraft[0].landing_gear.deployed = false;
    std::array<sim::AircraftControls, sim::World::kMaxAircraft> c{};
    c[0].throttle = 0.9;
    c[0].stick.pitch_stick = 0.3; // manoeuvring shooter: rotation adds to the launch velocity

    fdm::FlightState at_fire{};
    int fired_step = -1;
    for (int i = 0; i < 400 && fired_step < 0; ++i) {
        c[0].trigger = true;
        world->step(DT, c);
        if (world->aircraft[0].gun.rounds_fired > 0) {
            at_fire = world->aircraft[0].state;
            fired_step = i;
        }
    }
    assert(fired_step >= 0 && world->aircraft[0].gun.rounds_fired == 1);
    c[0].trigger = false;
    const int extra = 200;
    for (int i = 0; i < extra; ++i) world->step(DT, c);

    const sim::Projectile* round = nullptr;
    for (const auto& r : world->rounds) {
        if (r.active) round = &r;
    }
    assert(round);
    const math::Vector3 predicted = sim::Ballistics::predict(at_fire, world->aircraft[0].gun.spec,
                                                             world->aircraft[0].gun.boresight_b(), (extra + 1) * DT);
    const double err = (predicted - round->pos).norm();
    std::cout << "    after " << (extra + 1) * DT << " s of flight: prediction error " << err << " m\n";
    assert(err < 1e-6);
    std::cout << "  -> PASSED\n";
}

/// Put the target where the pipper is: iterate the director solution until
/// the predicted miss is zero, as a pilot does by flying the pipper onto it.
void place_on_pipper(sim::World& w) {
    sim::Aircraft& shooter = w.aircraft[0];
    sim::Aircraft& target = w.aircraft[1];
    for (int it = 0; it < 6; ++it) {
        const auto sol = sim::GunSolution::compute(shooter.state, shooter.gun.spec, target.state.pos_ned,
                                                   target.velocity(), target.acceleration());
        assert(sol.valid);
        target.state.pos_ned = target.state.pos_ned - sol.miss_ned;
    }
    target.prev_state = target.state;
}

void test_pipper_on_target_scores_hits() {
    std::cout << "[GUNS] Non-manoeuvring target at 600 m: pipper on it -> hits...\n";
    for (AircraftType type : {AircraftType::F16_FIGHTING_FALCON, AircraftType::EUROFIGHTER_TYPHOON,
                              AircraftType::A10_THUNDERBOLT, AircraftType::F35B_LIGHTNING_II}) {
        auto w = std::make_unique<sim::World>();
        w->clear(11);
        const double v = (type == AircraftType::A10_THUNDERBOLT) ? 150.0 : 230.0;
        w->add(type, level({0.0, 0.0, -4000.0}, 0.0, v));
        w->add(type, level({600.0, 0.0, -4000.0}, 0.0, v)); // same type: holds formation exactly
        for (int i = 0; i < 2; ++i) w->aircraft[static_cast<size_t>(i)].landing_gear.deployed = false;
        std::array<sim::AircraftControls, sim::World::kMaxAircraft> c{};
        c[0].throttle = c[1].throttle = 0.7;
        // The A-10 pilots hold 1 G with pitch damping: it has no G law to do it
        // for them. A fly-by-wire jet holds 1 G hands-off, and a pilot loop on
        // top of its shaped G command only rocks it.
        const bool fbw = aircraft::AircraftConfig::get(type).flcs.law_type
                      != aircraft::FLCSConfig::LawType::HYDRO_SAS_AUGMENTED;
        std::array<double, 2> trim{0.0, 0.0};
        auto fly = [&] {
            for (size_t k = 0; k < 2 && !fbw; ++k) {
                const auto& a = w->aircraft[k];
                trim[k] = std::clamp(trim[k] + 0.4 * (1.0 - a.imu.Nz) * DT, -0.5, 0.5);
                c[k].stick.pitch_stick = std::clamp(trim[k] - 0.8 * a.state.omega_b.y, -1.0, 1.0);
            }
            w->step(DT, c);
        };
        // Let both jets trim out in formation first: from a zero-alpha start the
        // nose rises several degrees, which would throw the stream high.
        for (int i = 0; i < static_cast<int>(8.0 / DT); ++i) fly();
        place_on_pipper(*w);

        // Frozen-geometry check: a round fired now is predicted to pass within a metre.
        const auto sol = sim::GunSolution::compute(w->aircraft[0].state, w->aircraft[0].gun.spec,
                                                   w->aircraft[1].state.pos_ned, w->aircraft[1].velocity(),
                                                   w->aircraft[1].acceleration());
        std::cout << "    shooter pitch rate " << w->aircraft[0].state.omega_b.y * 57.3 << " deg/s, Nz "
                  << w->aircraft[0].imu.Nz << "\n";
        assert(sol.miss_distance < 1.0);

        c[0].trigger = true;
        int hits = 0;
        for (int i = 0; i < static_cast<int>(0.6 / DT); ++i) fly();
        c[0].trigger = false;
        for (int i = 0; i < static_cast<int>(1.5 / DT); ++i) fly();
        for (int e = 0; e < w->event_count(); ++e) {
            if (w->event(e).kind == sim::CombatEvent::Kind::HIT && w->event(e).shooter == 0) ++hits;
        }
        const int fired = w->aircraft[0].gun.rounds_fired;
        std::cout << "    " << w->aircraft[0].gun.spec.name << ": " << hits << " hits of " << fired
                  << " rounds (TOF " << sol.time_of_flight << " s)\n";
        assert(fired > 0);
        // Dispersion alone (2.5 mrad, 1.5 m at 600 m, against a ~1 m target)
        // puts the expected share near 20%; asserting exactly that flipped on
        // a single round whenever anything in the flight model moved.
        assert(hits >= std::max(1, fired / 6));
    }
    std::cout << "  -> PASSED\n";
}

void test_damage_effects() {
    std::cout << "[DAMAGE] Engine, wing and structural damage feed the flight model...\n";
    sim::DamageRng rng{};

    // Single-engine F-16: two engine hits and the thrust is mostly gone.
    sim::DamageState f16{AircraftType::F16_FIGHTING_FALCON};
    f16.apply_hit(sim::HitZone::ENGINE, 1.0, rng);
    f16.apply_hit(sim::HitZone::ENGINE, 1.0, rng);
    std::cout << "    F-16C after 2 engine hits: thrust x" << f16.thrust_factor() << "\n";
    assert(f16.thrust_factor() < 0.15);

    // Twin-engine F-15EX: knock one engine out entirely and half the thrust remains.
    sim::DamageState f15{AircraftType::F15EX_EAGLE_II};
    f15.engine_health[0] = 0.0;
    f15.hits_taken = 1;
    std::cout << "    F-15EX with one engine out: thrust x" << f15.thrust_factor() << "\n";
    assert(std::abs(f15.thrust_factor() - 0.5) < 1e-12);

    // A shot-up left wing rolls the jet toward it and cuts roll authority.
    sim::DamageState wing{AircraftType::F16_FIGHTING_FALCON};
    wing.apply_hit(sim::HitZone::WING_LEFT, 1.0, rng);
    aero::ControlSurfaces s{0.0, 0.0, 0.0};
    wing.apply_to_surfaces(s);
    assert(s.delta_a < 0.0 || s.delta_a > 0.0);
    assert(wing.fuel_leak_kgs > 0.0);

    // The A-10 absorbs more 20 mm hits than the F-16 before structural failure.
    auto hits_to_kill = [&](AircraftType t) {
        sim::DamageState d{t};
        sim::DamageRng r{};
        int n = 0;
        while (!d.destroyed && n < 100) {
            d.apply_hit(sim::HitZone::FUSELAGE, 1.0, r);
            ++n;
        }
        return n;
    };
    const int f16_hits = hits_to_kill(AircraftType::F16_FIGHTING_FALCON);
    const int a10_hits = hits_to_kill(AircraftType::A10_THUNDERBOLT);
    std::cout << "    fuselage hits to destroy: F-16C " << f16_hits << ", A-10C " << a10_hits << "\n";
    assert(a10_hits > f16_hits);
    std::cout << "  -> PASSED\n";
}

void test_damaged_jet_flies_damaged() {
    std::cout << "[DAMAGE] A jet with a dead engine decelerates; a destroyed jet is a kill...\n";
    auto w = std::make_unique<sim::World>();
    w->clear(5);
    w->add(AircraftType::F16_FIGHTING_FALCON, level({0.0, 0.0, -4000.0}, 0.0, 250.0));
    w->add(AircraftType::F16_FIGHTING_FALCON, level({0.0, 3000.0, -4000.0}, 0.0, 250.0));
    for (int i = 0; i < 2; ++i) w->aircraft[static_cast<size_t>(i)].landing_gear.deployed = false;
    w->aircraft[1].damage.engine_health[0] = 0.0;
    w->aircraft[1].damage.hits_taken = 1;
    std::array<sim::AircraftControls, sim::World::kMaxAircraft> c{};
    c[0].throttle = c[1].throttle = 0.9;
    for (int i = 0; i < static_cast<int>(20.0 / DT); ++i) w->step(DT, c);
    const double e0 = w->aircraft[0].state.altitude() + std::pow(w->aircraft[0].state.airspeed(), 2) / 19.6;
    const double e1 = w->aircraft[1].state.altitude() + std::pow(w->aircraft[1].state.airspeed(), 2) / 19.6;
    std::cout << "    energy height after 20 s: healthy " << e0 << " m, engine out " << e1 << " m\n";
    assert(e1 < e0 - 500.0);

    // Destroy jet 1 and look for the kill event.
    w->aircraft[1].damage.destroyed = true;
    w->step(DT, c);
    bool kill = false;
    for (int e = 0; e < w->event_count(); ++e) {
        kill |= w->event(e).kind == sim::CombatEvent::Kind::KILL && w->event(e).victim == 1;
    }
    assert(kill);
    assert(!w->aircraft[1].alive());
    std::cout << "  -> PASSED\n";
}

} // namespace

int main() {
    std::cout << "=========================================================\n";
    std::cout << "  Guns, Ballistics and Damage                            \n";
    std::cout << "=========================================================\n";
    test_vacuum_trajectory_is_a_parabola();
    test_drag_decay_matches_analytic();
    test_rate_of_fire_and_spin_up();
    test_swept_hit_catches_crossing_shot();
    test_predictor_is_the_round_that_flies();
    test_pipper_on_target_scores_hits();
    test_damage_effects();
    test_damaged_jet_flies_damaged();
    std::cout << "\nAll guns and damage tests passed successfully!\n";
    return 0;
}
