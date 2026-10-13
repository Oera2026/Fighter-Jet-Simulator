/**
 * @file test_flight_realism.cpp
 * @brief Physics-realism checks across all five airframes, from the findings
 * of a flight-model review:
 *
 *  - Compressibility raised lift but not drag, so every jet sustained 8-9 G at
 *    30,000 ft (real fighters hold 3.5-4.5 G there).
 *  - No jet could stall: lift was held at its maximum to 90 deg alpha and
 *    beyond, with far too little drag.
 *  - The F-16 FLCS went into a divergent pitch oscillation (~50 G) on a full
 *    pull at Mach 1.2 on the deck, and rolling pulls fired every FBW jet past
 *    11 G. Full aft stick, meanwhile, settled 0.5-1 G short of the limit.
 *  - Slamming the throttle from idle into reheat brought the whole thrust in
 *    on the afterburner light-off time, skipping the core spool-up.
 *  - Nothing happened to an airframe pulled past its limits.
 *
 * Energy-manoeuvrability numbers are worked from the aero and engine models
 * directly (a steady turn: thrust equals drag); control-law checks fly the
 * full sim::Aircraft pipeline.
 */
#include "fastjet/sim/engagement.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>

using namespace fastjet;
using aircraft::AircraftType;

namespace {

constexpr double DT = 0.005;
constexpr double G0 = 9.80665;
constexpr double DEG = M_PI / 180.0;

constexpr AircraftType ALL[] = {AircraftType::F16_FIGHTING_FALCON, AircraftType::F15EX_EAGLE_II,
                                AircraftType::EUROFIGHTER_TYPHOON, AircraftType::F22_RAPTOR,
                                AircraftType::A10_THUNDERBOLT, AircraftType::F35B_LIGHTNING_II};
constexpr AircraftType FBW[] = {AircraftType::F16_FIGHTING_FALCON, AircraftType::F15EX_EAGLE_II,
                                AircraftType::EUROFIGHTER_TYPHOON, AircraftType::F22_RAPTOR,
                                AircraftType::F35B_LIGHTNING_II};

fdm::FlightState at_alpha(double alpha_deg, double v = 150.0) {
    fdm::FlightState s{};
    s.vel_b = math::Vector3(v * std::cos(alpha_deg * DEG), 0.0, v * std::sin(alpha_deg * DEG));
    return s;
}

double speed_of_sound(double alt) { return environment::Atmosphere1976::compute(alt, 100.0).speed_of_sound; }

/// A jet in level flight at @p alt / @p mach, engine running at @p lever, half fuel.
std::unique_ptr<sim::Aircraft> airborne(AircraftType t, double alt, double mach, double lever = 1.0) {
    auto a = std::make_unique<sim::Aircraft>(t);
    fdm::FlightState s{};
    s.pos_ned = {0.0, 0.0, -alt};
    s.vel_b = {mach * speed_of_sound(alt), 0.0, 0.0};
    a->reset(s);
    a->landing_gear.deployed = false;
    const auto air = environment::Atmosphere1976::compute(alt, s.vel_b.x);
    for (int i = 0; i < 3000; ++i) (void)a->engine.update(lever, air, DT);
    a->engine.fuel_kg *= 0.5;
    a->refresh_mass();
    return a;
}

/// Highest load factor the jet sustains (thrust >= drag) in a steady turn,
/// capped at its G limit, and the matching turn rate [deg/s].
struct Turn { double g, rate_dps; };
Turn sustained_turn(AircraftType t, double alt, double mach) {
    const auto cfg = aircraft::AircraftConfig::get(t);
    const aero::AircraftAeroModel model(t);
    const propulsion::MultiEngine eng(t);
    const double V = mach * speed_of_sound(alt);
    const auto air = environment::Atmosphere1976::compute(alt, V);
    const double thrust = cfg.propulsion.static_ab_thrust_n * eng.lapse_factor(air, cfg.propulsion.has_afterburner);
    const double weight = fdm::FuelSystem::compute(0.5 * cfg.mass.internal_fuel_capacity_kg, t,
                                                   cfg.mass.combat_payload_kg).mass_kg * G0;
    const double qS = air.dynamic_pressure * cfg.aero.s_ref;
    double n_sus = 0.0;
    for (double a = 0.0; a <= cfg.flcs.alpha_limit_deg; a += 0.05) {
        aero::ControlSurfaces c{};
        if (t == AircraftType::F16_FIGHTING_FALCON) c.delta_lef = flcs::F16FLCS::compute_lef_schedule(a, air.dynamic_pressure);
        const auto k = model.compute_coefficients(at_alpha(a, V), c, air.dynamic_pressure, mach);
        const double lift = qS * k.CL + thrust * std::sin(a * DEG);
        const double drag = qS * k.CD - thrust * std::cos(a * DEG);
        if (drag <= 0.0) n_sus = std::max(n_sus, lift / weight);
    }
    n_sus = std::min(n_sus, cfg.flcs.max_g_positive);
    const double rate = n_sus > 1.0 ? std::sqrt(n_sus * n_sus - 1.0) * G0 / V / DEG : 0.0;
    return {n_sus, rate};
}

double max_cl(const aero::AircraftAeroModel& m, double mach) {
    double best = 0.0;
    for (double a = 0.0; a <= 50.0; a += 0.25) {
        best = std::max(best, m.compute_coefficients(at_alpha(a), {}, 6000.0, mach).CL);
    }
    return best;
}

void test_sustained_turn_at_altitude() {
    std::cout << "[REALISM] Sustained turns at 30,000 ft are 3.5-5 G, not 9...\n";
    for (AircraftType t : FBW) {
        const Turn hi = sustained_turn(t, 9144.0, 0.9);
        std::cout << "    " << aircraft::to_short_string(t) << " 30,000 ft M0.9: " << hi.g << " G, " << hi.rate_dps
                  << " deg/s\n";
        // The F-35B is the weakest turner up high: its transonic spec was cut to
        // 4.5 G at M0.8 / 15,000 ft (DOT&E FY2012), so it holds under 3 G here.
        assert(hi.g > (t == AircraftType::F35B_LIGHTNING_II ? 2.5 : 3.0) && hi.g < 5.5);
    }
    const Turn f35 = sustained_turn(AircraftType::F35B_LIGHTNING_II, 4572.0, 0.8);
    std::cout << "    F-35B 15,000 ft M0.8: " << f35.g << " G (spec 4.5)\n";
    assert(f35.g > 4.0 && f35.g < 5.3);
    // Published F-16C figures: ~15 deg/s sustained at 15,000 ft, ~20 at sea level.
    const Turn mid = sustained_turn(AircraftType::F16_FIGHTING_FALCON, 4572.0, 0.8);
    const Turn low = sustained_turn(AircraftType::F16_FIGHTING_FALCON, 0.0, 0.6);
    std::cout << "    F-16C 15,000 ft M0.8: " << mid.g << " G, " << mid.rate_dps << " deg/s; sea level M0.6: "
              << low.g << " G, " << low.rate_dps << " deg/s\n";
    assert(mid.rate_dps > 13.0 && mid.rate_dps < 18.5);
    assert(low.rate_dps > 18.0 && low.rate_dps < 25.0);
    std::cout << "  -> PASSED\n";
}

void test_lift_is_not_free_with_mach() {
    std::cout << "[REALISM] Mach lowers CLmax and costs the drag of the lift it adds...\n";
    for (AircraftType t : ALL) {
        const aero::AircraftAeroModel m(t);
        const auto slow = m.compute_coefficients(at_alpha(15.0), {}, 6000.0, 0.3);
        const auto fast = m.compute_coefficients(at_alpha(15.0), {}, 6000.0, 0.85);
        const double cl_slow = max_cl(m, 0.3), cl_fast = max_cl(m, 0.9);
        std::cout << "    " << aircraft::to_short_string(t) << ": L/D at 15 deg " << slow.CL / slow.CD << " (M0.3) -> "
                  << fast.CL / fast.CD << " (M0.85); CLmax " << cl_slow << " -> " << cl_fast << " (M0.9)\n";
        assert(fast.CL / fast.CD <= slow.CL / slow.CD);
        assert(cl_fast < 0.85 * cl_slow);
    }
    std::cout << "  -> PASSED\n";
}

void test_every_jet_stalls() {
    std::cout << "[REALISM] Past the stall lift falls away and drag climbs to a flat plate's...\n";
    for (AircraftType t : ALL) {
        const aero::AircraftAeroModel m(t);
        const double clmax = max_cl(m, 0.3);
        const auto a60 = m.compute_coefficients(at_alpha(60.0), {}, 6000.0, 0.3);
        const auto a90 = m.compute_coefficients(at_alpha(90.0), {}, 6000.0, 0.3);
        const auto b90 = m.compute_coefficients(at_alpha(90.5), {}, 6000.0, 0.3);
        const auto tail_first = m.compute_coefficients(at_alpha(179.0), {}, 6000.0, 0.3);
        std::cout << "    " << aircraft::to_short_string(t) << ": CLmax " << clmax << ", 60 deg CL " << a60.CL
                  << " CD " << a60.CD << ", 90 deg CL " << a90.CL << " CD " << a90.CD << ", tail first CD "
                  << tail_first.CD << "\n";
        assert(a60.CL < 0.8 * clmax);
        assert(std::abs(a90.CL) < 0.05 && a90.CD > 1.4);
        assert(std::abs(b90.CL - a90.CL) < 0.05 && std::abs(b90.CD - a90.CD) < 0.05); // no jump at 90 deg
        assert(tail_first.CD < 0.2);
    }
    std::cout << "  -> PASSED\n";
}

struct Pull { double peak, mean, low; };

/// Hold the stick for @p hold_s from level flight; mean over 2.0-3.5 s.
Pull pull(AircraftType t, double alt, double mach, double pitch, double roll = 0.0, double hold_s = 4.0) {
    auto a = airborne(t, alt, mach);
    sim::AircraftControls c{};
    c.throttle = 1.0;
    for (int i = 0; i < 300; ++i) a->step(DT, c);
    c.stick.pitch_stick = pitch;
    c.stick.roll_stick = roll;
    Pull p{-99.0, 0.0, 99.0};
    int n = 0;
    for (int i = 0; i < static_cast<int>(hold_s / DT); ++i) {
        if (i * DT > 3.0) c.stick.roll_stick = 0.0; // the roll stops, the pull goes on
        a->step(DT, c);
        p.peak = std::max(p.peak, a->imu.Nz);
        p.low = std::min(p.low, a->imu.Nz);
        if (i * DT >= 2.0 && i * DT < 3.5) { p.mean += a->imu.Nz; ++n; }
    }
    p.mean /= std::max(1, n);
    assert(!a->damage.yielded && "the FLCS must not bend its own jet");
    return p;
}

void test_full_stick_reaches_the_limit_not_past_it() {
    std::cout << "[REALISM] Full aft stick pulls the rated G where the wing can make it, and no more...\n";
    for (AircraftType t : FBW) {
        // M0.9 at 3,000 m: every airframe here has the lift for its limit G.
        const Pull full = pull(t, 3000.0, 0.9, 1.0);
        const Pull half = pull(t, 3000.0, 0.9, 0.5);
        const Pull rolling = pull(t, 100.0, 0.9, 1.0, 1.0);
        const Pull push = pull(t, 6000.0, 0.4, -1.0);
        std::cout << "    " << aircraft::to_short_string(t) << ": full " << full.mean << " G (peak " << full.peak
                  << "), half " << half.mean << " G, rolling pull peak " << rolling.peak << " G, low-speed push "
                  << push.low << " G\n";
        // Against each jet's own limit (9 G, the F-35B 7 G).
        const double lim = aircraft::AircraftConfig::get(t).flcs.max_g_positive;
        const double half_g = 1.0 + 0.5 * (lim - 1.0);
        assert(full.mean > lim - 0.4 && full.mean < lim + 0.3 && full.peak < lim + 0.6);
        assert(half.mean > half_g - 0.4 && half.mean < half_g + 0.4);
        assert(rolling.peak < lim + 0.8);
        assert(push.low > -3.8);
    }
    // The pull that used to diverge: full aft stick at Mach 1.2 on the deck.
    const Pull deck = pull(AircraftType::F16_FIGHTING_FALCON, 100.0, 1.2, 1.0);
    std::cout << "    F-16C Mach 1.2 on the deck: peak " << deck.peak << " G, low " << deck.low << " G\n";
    assert(deck.peak < 10.0 && deck.low > -1.0);
    std::cout << "  -> PASSED\n";
}

void test_afterburner_waits_for_the_core() {
    std::cout << "[REALISM] Idle to full reheat: the core spools first, then the afterburner lights...\n";
    for (AircraftType t : {AircraftType::F16_FIGHTING_FALCON, AircraftType::F22_RAPTOR}) {
        propulsion::MultiEngine e(t);
        const auto air = environment::Atmosphere1976::compute(3000.0, 200.0);
        for (int i = 0; i < 4000; ++i) (void)e.update(0.08, air, DT);
        double at_1s = 0.0, ab_at_1s = 0.0;
        for (int i = 1; i <= static_cast<int>(15.0 / DT); ++i) {
            (void)e.update(1.0, air, DT);
            if (i == static_cast<int>(1.0 / DT)) { at_1s = e.net_thrust_n; ab_at_1s = e.ab_thrust_n; }
        }
        const double mil = e.config.static_dry_thrust_n * e.lapse_factor(air, false);
        std::cout << "    " << aircraft::to_short_string(t) << ": " << at_1s / 1000.0 << " kN after 1 s (mil "
                  << mil / 1000.0 << " kN), " << e.net_thrust_n / 1000.0 << " kN after 15 s of "
                  << e.commanded_thrust_n / 1000.0 << "\n";
        assert(ab_at_1s == 0.0 && at_1s < 0.6 * mil);
        assert(std::abs(e.net_thrust_n - e.commanded_thrust_n) < 0.03 * e.commanded_thrust_n);
    }
    std::cout << "  -> PASSED\n";
}

void test_over_g_bends_then_breaks_the_airframe() {
    std::cout << "[REALISM] Over-G: overstress past the limit, a bent wing past 1.2x, failure at 1.5x...\n";
    auto hold = [](sim::DamageState& d, double nz, double seconds) {
        for (int i = 0; i < static_cast<int>(seconds / DT); ++i) d.apply_load(nz, DT);
    };
    sim::DamageState f16{AircraftType::F16_FIGHTING_FALCON};
    hold(f16, 9.0, 3.0);
    assert(!f16.overstressed && !f16.damaged());
    hold(f16, 10.0, 1.0);
    assert(f16.overstressed && !f16.yielded && !f16.damaged());
    hold(f16, 11.5, 1.0);
    assert(f16.yielded && f16.damaged() && f16.wing_left < 1.0 && !f16.destroyed);
    hold(f16, 13.6, 1.0);
    assert(f16.destroyed && f16.structural_failure);
    std::cout << "    F-16C: 10 G overstress, 11.5 G bent wings (" << f16.wing_left << " roll authority), 13.6 G failure\n";

    // A single integration step's spike is not a structural load.
    sim::DamageState spike{AircraftType::F16_FIGHTING_FALCON};
    spike.apply_load(20.0, DT);
    hold(spike, 1.0, 0.5);
    assert(!spike.yielded && !spike.destroyed);

    // The negative limit is lower: an A-10 rated for -3 G fails at -4.5.
    sim::DamageState a10{AircraftType::A10_THUNDERBOLT};
    hold(a10, -4.6, 1.0);
    assert(a10.structural_failure);

    // In the world, a broken airframe is out of the fight and says why.
    auto w = std::make_unique<sim::World>();
    w->clear(5);
    fdm::FlightState s{};
    s.pos_ned = {0.0, 0.0, -5000.0};
    s.vel_b = {250.0, 0.0, 0.0};
    w->add(AircraftType::F16_FIGHTING_FALCON, s);
    w->aircraft[0].damage.apply_load(14.0, 1.0);
    std::array<sim::AircraftControls, sim::World::kMaxAircraft> c{};
    w->step(DT, c);
    assert(!w->aircraft[0].alive());
    bool reported = false;
    for (int k = 0; k < w->event_count(); ++k) {
        reported = reported || w->event(k).kind == sim::CombatEvent::Kind::STRUCTURAL_FAILURE;
    }
    assert(reported);
    std::cout << "  -> PASSED\n";
}

void test_combat_payload_is_carried() {
    std::cout << "[REALISM] Every jet carries its pilot, ammunition and missiles...\n";
    for (AircraftType t : ALL) {
        const sim::Aircraft a(t);
        const auto m = aircraft::AircraftConfig::get(t).mass;
        assert(m.combat_payload_kg >= 700.0);
        assert(std::abs(a.mass.mass_kg - (m.empty_mass_kg + a.engine.fuel_kg + m.combat_payload_kg)) < 1e-6);
    }
    std::cout << "  -> PASSED\n";
}

void test_ai_fights_do_not_break_airframes() {
    std::cout << "[REALISM] AI dogfights stay inside the airframe limits...\n";
    for (uint32_t seed = 1; seed <= 3; ++seed) {
        sim::EngagementSetup s;
        s.geometry = sim::StartGeometry::HEAD_ON_MERGE;
        s.skill = sim::AiSkill::ACE;
        s.seed = seed;
        s.own_is_ai = true;
        s.time_limit_s = 90.0;
        auto e = std::make_unique<sim::Engagement>();
        e->setup(s);
        const sim::AircraftControls none{};
        double peak = 0.0;
        while (!e->finished()) {
            e->step(DT, none);
            for (int i = 0; i < 2; ++i) {
                const auto& a = e->world.aircraft[static_cast<size_t>(i)];
                if (a.alive()) peak = std::max(peak, a.damage.peak_load_ratio);
            }
        }
        std::cout << "    seed " << seed << ": " << sim::to_string(e->outcome) << ", peak load " << peak * 100.0
                  << "% of the limit\n";
        for (int i = 0; i < 2; ++i) {
            assert(!e->world.aircraft[static_cast<size_t>(i)].damage.yielded);
            assert(!e->world.aircraft[static_cast<size_t>(i)].damage.structural_failure);
        }
    }
    std::cout << "  -> PASSED\n";
}

} // namespace

int main() {
    std::cout << "=========================================================\n";
    std::cout << "  Flight Physics Realism                                 \n";
    std::cout << "=========================================================\n";
    test_sustained_turn_at_altitude();
    test_lift_is_not_free_with_mach();
    test_every_jet_stalls();
    test_full_stick_reaches_the_limit_not_past_it();
    test_afterburner_waits_for_the_core();
    test_over_g_bends_then_breaks_the_airframe();
    test_combat_payload_is_carried();
    test_ai_fights_do_not_break_airframes();
    std::cout << "\nAll flight realism tests passed successfully!\n";
    return 0;
}
