/**
 * @file test_air_combat.cpp
 * @brief The AI air-to-air fight as a whole: missiles, countermeasures and
 * the missile approach warner (MAWS) together.
 *
 *  - AI-vs-AI fights in every loadout end (a kill or the time limit), with
 *    no jet flown into the ground unshot and no mid-air.
 *  - A pilot whose MAWS sees a heat seeker the RWR can't hear treats it as a
 *    heat seeker: flares, no chaff, a defensive turn. An F-16 pilot (no MAWS)
 *    doesn't react to one he can't see until he sights it.
 *  - A novice never shoots a heat seeker from the target's front hemisphere;
 *    an ace takes the all-aspect shot from the same start.
 *  - The symmetric head-on start is fair: the two jets fly mirror images of
 *    each other in still air, and neither side wins two fights in three.
 *
 * Fights are chaotic, so behaviour that must hold on every seed (safety,
 * shot discipline) is checked on every seed, and balance in aggregate with a
 * wide tolerance.
 */
#include "fastjet/sim/engagement.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>

using namespace fastjet;
using aircraft::AircraftType;

namespace {

constexpr double DT = 0.005;
constexpr int OWN = sim::Engagement::OWN;
constexpr int BANDIT = sim::Engagement::BANDIT;

struct FightResult {
    sim::EngagementOutcome outcome{};
    double time{0.0};
    int unforced_ground_impacts{0};
    int midairs{0};
    int novice_srm_shots{0};
    int novice_front_shots{0}; ///< Heat seekers a novice fired from the target's front hemisphere
};

/// Off the tail of @p jet as seen from @p viewer, both before the latest step [rad].
double off_tail_before_step(const sim::Aircraft& jet, const sim::Aircraft& viewer) {
    const math::Vector3 tail = jet.prev_state.q_att.rotate_body_to_ned(math::Vector3(-1.0, 0.0, 0.0));
    return sim::AirCombatGeometry::angle_between(tail, viewer.prev_state.pos_ned - jet.prev_state.pos_ned);
}

/// Fly a whole engagement with the AI in both cockpits, @p own_skill in the
/// player's (the bandit's skill is s.skill). Every heat-seeker launch is
/// checked for its aspect: AI shots are taken before the step that flies
/// them, from the states the step leaves in prev_state.
FightResult fly(sim::EngagementSetup s, sim::AiSkill own_skill) {
    s.own_is_ai = true;
    auto e = std::make_unique<sim::Engagement>();
    e->setup(s);
    e->ai[OWN] = sim::BfmPilot(own_skill, s.seed * 31u + 5u);
    e->ai[OWN].hard_deck_agl_m = s.hard_deck_agl_m;
    const sim::AiMissileDiscipline d = sim::AiMissileDiscipline::of(own_skill);
    e->world.seekers[OWN].search_half_deg = d.off_boresight_deg;
    e->srm_shooters[OWN].off_boresight_deg = d.off_boresight_deg;
    e->srm_shooters[OWN].confirm_lock_s = d.confirm_lock_s;
    e->srm_shooters[OWN].spacing_s = d.spacing_s;
    e->srm_shooters[OWN].max_off_tail_deg = d.max_off_tail_deg;
    e->srm_shooters[OWN].strong_off_tail_deg = d.strong_off_tail_deg;
    const sim::AiSkill skills[2] = {own_skill, s.skill};

    FightResult r;
    const sim::AircraftControls none{};
    while (!e->finished()) {
        const int srm_before[2] = {e->world.stores[OWN].srm.count, e->world.stores[BANDIT].srm.count};
        e->step(DT, none);
        for (int i = 0; i < 2; ++i) {
            if (e->world.stores[static_cast<size_t>(i)].srm.count >= srm_before[i]) continue;
            if (skills[i] != sim::AiSkill::NOVICE) continue;
            ++r.novice_srm_shots;
            const double off = off_tail_before_step(e->world.aircraft[static_cast<size_t>(1 - i)],
                                                    e->world.aircraft[static_cast<size_t>(i)]);
            if (off > 0.5 * M_PI) ++r.novice_front_shots;
        }
    }
    r.outcome = e->outcome;
    r.time = e->world.time;
    for (int i = 0; i < 2; ++i) {
        const auto& a = e->world.aircraft[static_cast<size_t>(i)];
        if (a.crashed && a.damage.hits_taken == 0) ++r.unforced_ground_impacts;
    }
    for (int k = 0; k < e->world.event_count(); ++k) {
        if (e->world.event(k).kind == sim::CombatEvent::Kind::MIDAIR) ++r.midairs;
    }
    return r;
}

void test_every_loadout_ends_safely() {
    std::cout << "[AIR COMBAT] AI vs AI in every loadout: every fight ends, no unforced ground impacts...\n";
    constexpr double LIMIT = 40.0;
    for (auto load : {sim::WeaponsLoadout::GUNS_ONLY, sim::WeaponsLoadout::GUNS_AND_IR, sim::WeaponsLoadout::FULL_LOADOUT}) {
        for (auto geo : {sim::StartGeometry::HEAD_ON_MERGE, sim::StartGeometry::OFFENSIVE_PERCH}) {
            sim::EngagementSetup s;
            s.loadout = load;
            s.geometry = geo;
            s.own_type = AircraftType::F15EX_EAGLE_II;           // MAWS
            s.bandit_type = AircraftType::F16_FIGHTING_FALCON;   // no MAWS
            s.skill = sim::AiSkill::VETERAN;
            s.time_limit_s = LIMIT;
            s.seed = 5;
            const FightResult r = fly(s, sim::AiSkill::ACE);
            std::cout << "    " << sim::to_string(load) << ", " << sim::to_string(geo) << ": " << sim::to_string(r.outcome)
                      << " at " << r.time << " s\n";
            assert(r.outcome != sim::EngagementOutcome::IN_PROGRESS);
            assert(r.time <= LIMIT + DT);
            assert(r.unforced_ground_impacts == 0);
            assert(r.midairs == 0);
        }
    }
    std::cout << "  -> PASSED\n";
}

/// A fight set up and then pulled apart: the bandit 25 km off and running, so
/// the only thing happening near the player's jet is the missile we put there.
std::unique_ptr<sim::Engagement> quiet_sky(AircraftType own) {
    sim::EngagementSetup s;
    s.own_type = own;
    s.bandit_type = AircraftType::F16_FIGHTING_FALCON;
    s.loadout = sim::WeaponsLoadout::GUNS_ONLY;
    s.own_is_ai = true;
    s.seed = 3;
    auto e = std::make_unique<sim::Engagement>();
    e->setup(s);
    sim::Aircraft& b = e->world.aircraft[BANDIT];
    b.state.pos_ned = e->own().state.pos_ned + math::Vector3(25000.0, 0.0, 0.0);
    b.state.q_att = math::Quaternion::from_euler(0.0, 0.0, 0.0);
    b.prev_state = b.state;
    return e;
}

/// A heat seeker fired by the bandit at the player, @p behind metres dead
/// astern of him, closing at @p closing m/s; its motor burning or burnt out.
void inject_heat_seeker(sim::Engagement& e, double behind, double closing, bool burning) {
    const sim::Aircraft& own = e.own();
    const math::Vector3 fwd = own.velocity().normalized();
    sim::Missile& m = e.world.missiles[5];
    m = sim::Missile{};
    m.spec = sim::missile_specs::aim9x();
    m.body.pos = own.position() - fwd * behind;
    m.body.vel = own.velocity() + fwd * closing;
    m.body.tof = burning ? 0.3 : m.spec.motor_time_s() + 1.0;
    m.shooter = BANDIT;
    m.target = OWN;
    m.active = true;
    m.phase = sim::Missile::Phase::ACTIVE_TRACK;
    m.est_pos = own.position();
    m.est_vel = own.velocity();
}

void test_maws_heat_seeker_gets_flares() {
    std::cout << "[AIR COMBAT] A heat seeker on the MAWS, silent on the RWR: flares, no chaff, a defensive turn...\n";
    for (AircraftType t : {AircraftType::F15EX_EAGLE_II, AircraftType::EUROFIGHTER_TYPHOON, AircraftType::F35B_LIGHTNING_II}) {
        auto e = quiet_sky(t);
        const sim::AircraftControls none{};
        for (int k = 0; k < 200; ++k) e->step(DT, none); // settle: nothing in the air, no countermeasures
        const int flares0 = e->world.flare_load[OWN];
        const int chaff0 = e->world.chaff_load[OWN];
        // 7 km astern: beyond the 5 km at which its smoke trail can be seen.
        inject_heat_seeker(*e, 7000.0, 300.0, /*burning=*/true);
        bool warned = false, turned = false;
        double t_warn = -1.0;
        const double t0 = e->world.time;
        while (e->world.time - t0 < 1.5) {
            e->step(DT, none);
            const sim::MissileThreat th = e->detect_missile_threat(OWN);
            const double range = (e->world.missiles[5].body.pos - e->own().position()).norm();
            if (th.inbound && !warned) {
                warned = true;
                t_warn = e->world.time - t0;
                assert(th.maws && th.heat_seeker && !th.radar_guided);
                assert(range > sim::Engagement::VISUAL_MISSILE_SMOKE_M); // the MAWS, not the eye
            }
            turned = turned || e->ai[OWN].maneuver == sim::BfmPilot::Maneuver::DEFEND_MISSILE;
        }
        const int flares = flares0 - e->world.flare_load[OWN];
        const int chaff = chaff0 - e->world.chaff_load[OWN];
        std::cout << "    " << aircraft::to_short_string(t) << " (" << e->world.maws[OWN].spec.name << "): warned after "
                  << t_warn << " s, " << flares << " flares, " << chaff << " chaff in 1.5 s\n";
        assert(warned && t_warn < 0.6);
        assert(turned);
        assert(flares >= 2);
        assert(chaff == 0);
    }
    std::cout << "  -> PASSED\n";
}

void test_f16_reacts_only_once_it_sees_the_missile() {
    std::cout << "[AIR COMBAT] F-16 (no MAWS): no reaction to an unseen heat seeker until it is sighted...\n";
    auto e = quiet_sky(AircraftType::F16_FIGHTING_FALCON);
    assert(!e->world.maws[OWN].spec.fitted());
    const sim::AircraftControls none{};
    for (int k = 0; k < 200; ++k) e->step(DT, none);
    const int flares0 = e->world.flare_load[OWN];
    // Burnt out and 4 km astern: no smoke trail to see, and nothing on the RWR.
    inject_heat_seeker(*e, 4000.0, 450.0, /*burning=*/false);
    double sighted_range = -1.0;
    int flares_unseen = 0;
    while (e->world.missiles[5].active && sighted_range < 0.0) {
        e->step(DT, none);
        assert(!e->world.maws[OWN].status.threat);
        const double range = (e->world.missiles[5].body.pos - e->own().position()).norm();
        const sim::MissileThreat th = e->detect_missile_threat(OWN);
        assert(!th.maws);
        if (th.inbound) sighted_range = range;
        else flares_unseen = flares0 - e->world.flare_load[OWN];
    }
    // Sighted inside the range a coasting missile can be seen, and not before.
    assert(sighted_range > 0.0 && sighted_range <= sim::Engagement::VISUAL_MISSILE_M + 50.0);
    assert(flares_unseen == 0);
    const int flares_before = e->world.flare_load[OWN];
    for (int k = 0; k < 100 && e->own().alive(); ++k) e->step(DT, none);
    const int flares_after_sighting = flares_before - e->world.flare_load[OWN];
    std::cout << "    sighted at " << sighted_range << " m; flares while unseen " << flares_unseen
              << ", in the 0.5 s after the sighting " << flares_after_sighting << "\n";
    assert(flares_after_sighting > 0);
    std::cout << "  -> PASSED\n";
}

void test_novice_heat_seeker_discipline() {
    std::cout << "[AIR COMBAT] Heat-seeker shots: a novice only from the target's rear hemisphere...\n";
    // The rule itself.
    {
        sim::IrSeeker sk;
        sk.lock_range_m = 6000.0;
        auto shooter = [](sim::AiSkill skill) {
            const sim::AiMissileDiscipline d = sim::AiMissileDiscipline::of(skill);
            sim::HeatSeekerShooter h;
            h.max_off_tail_deg = d.max_off_tail_deg;
            h.strong_off_tail_deg = d.strong_off_tail_deg;
            return h;
        };
        constexpr double D2R = M_PI / 180.0;
        const sim::HeatSeekerShooter novice = shooter(sim::AiSkill::NOVICE);
        const sim::HeatSeekerShooter veteran = shooter(sim::AiSkill::VETERAN);
        const sim::HeatSeekerShooter ace = shooter(sim::AiSkill::ACE);
        assert(novice.aspect_ok(sk, 2000.0, 80.0 * D2R) && !novice.aspect_ok(sk, 2000.0, 100.0 * D2R));
        assert(!novice.aspect_ok(sk, 500.0, 170.0 * D2R)); // not even on a strong tone
        assert(veteran.aspect_ok(sk, 4000.0, 115.0 * D2R) && !veteran.aspect_ok(sk, 4000.0, 150.0 * D2R));
        assert(veteran.aspect_ok(sk, 2000.0, 170.0 * D2R)); // head-on, on a strong tone
        assert(ace.aspect_ok(sk, 5900.0, 179.0 * D2R));
    }
    // Head-on at 2 nm the seekers lock at once: the ace shoots, the novice holds.
    int novice_shots = 0, ace_shots = 0;
    for (sim::AiSkill skill : {sim::AiSkill::NOVICE, sim::AiSkill::ACE}) {
        sim::EngagementSetup s;
        s.loadout = sim::WeaponsLoadout::GUNS_AND_IR;
        s.skill = skill;
        s.own_is_ai = true;
        s.seed = 2;
        auto e = std::make_unique<sim::Engagement>();
        e->setup(s);
        const sim::AircraftControls none{};
        for (int k = 0; k < static_cast<int>(4.0 / DT) && !e->finished(); ++k) e->step(DT, none);
        const int shots = e->srm_shooters[OWN].launches + e->srm_shooters[BANDIT].launches;
        (skill == sim::AiSkill::NOVICE ? novice_shots : ace_shots) = shots;
    }
    std::cout << "    first 4 s of a head-on merge: novices fired " << novice_shots << ", aces " << ace_shots << "\n";
    assert(novice_shots == 0 && ace_shots > 0);
    // Whole fights: every shot a novice takes is from the rear hemisphere.
    int shots = 0, front = 0;
    for (auto geo : {sim::StartGeometry::OFFENSIVE_PERCH, sim::StartGeometry::DEFENSIVE_PERCH,
                     sim::StartGeometry::HEAD_ON_MERGE}) {
        for (uint32_t seed = 1; seed <= 2; ++seed) {
            sim::EngagementSetup s;
            s.loadout = sim::WeaponsLoadout::GUNS_AND_IR;
            s.geometry = geo;
            s.skill = sim::AiSkill::NOVICE;
            s.time_limit_s = 40.0;
            s.seed = seed;
            const FightResult r = fly(s, sim::AiSkill::NOVICE);
            shots += r.novice_srm_shots;
            front += r.novice_front_shots;
            assert(r.unforced_ground_impacts == 0);
        }
    }
    std::cout << "    novice-vs-novice fights: " << shots << " heat seekers fired, " << front
              << " from the front hemisphere\n";
    assert(shots > 0);
    assert(front == 0);
    std::cout << "  -> PASSED\n";
}

void test_symmetric_head_on_is_fair() {
    std::cout << "[AIR COMBAT] Symmetric head-on: mirror-image jets in still air, neither side favoured...\n";
    // The same jets, the same controls: in still air they fly mirror images.
    // (In the old 5 m/s surface wind, ~11 m/s at 5 km, jet 0 flew into a
    // headwind and jet 1 down a tailwind, ~15 m/s apart over the ground.)
    {
        sim::EngagementSetup s;
        s.seed = 4;
        auto e = std::make_unique<sim::Engagement>();
        e->setup(s);
        std::array<sim::AircraftControls, sim::World::kMaxAircraft> c{};
        c[OWN].throttle = c[BANDIT].throttle = 0.9;
        for (int k = 0; k < static_cast<int>(5.0 / DT); ++k) e->world.step(DT, c);
        const sim::Aircraft& a = e->world.aircraft[OWN];
        const sim::Aircraft& b = e->world.aircraft[BANDIT];
        const math::Vector3 mid = (a.position() + b.position()) * 0.5;
        const double v_diff = std::abs(a.velocity().norm() - b.velocity().norm());
        std::cout << "    after 5 s: ground speeds " << a.velocity().norm() << " / " << b.velocity().norm()
                  << " m/s, mirror point " << std::hypot(mid.x, mid.y) << " m off the start's centre\n";
        assert(v_diff < 1.0);
        assert(std::hypot(mid.x, mid.y) < 5.0);
        assert(e->world.aircraft[OWN].wind.compute_mean_wind_ned(5000.0).norm() == 0.0);
    }
    // And the fights: aces with heat seekers, all-aspect, from the merge.
    constexpr uint32_t SEEDS = 40;
    int w0 = 0, w1 = 0, mutual = 0;
    for (uint32_t seed = 1; seed <= SEEDS; ++seed) {
        sim::EngagementSetup s;
        s.loadout = sim::WeaponsLoadout::GUNS_AND_IR;
        s.skill = sim::AiSkill::ACE;
        s.time_limit_s = 30.0;
        s.seed = seed;
        const FightResult r = fly(s, sim::AiSkill::ACE);
        w0 += r.outcome == sim::EngagementOutcome::WIN;
        w1 += r.outcome == sim::EngagementOutcome::LOSS;
        mutual += r.outcome == sim::EngagementOutcome::MUTUAL_KILL;
        assert(r.unforced_ground_impacts == 0 && r.midairs == 0);
    }
    const int decided = w0 + w1;
    std::cout << "    " << SEEDS << " fights: jet 0 won " << w0 << ", jet 1 won " << w1 << ", " << mutual
              << " mutual kills\n";
    assert(decided >= 10);
    // Neither side wins two fights in three.
    assert(3 * std::min(w0, w1) > decided);
    std::cout << "  -> PASSED\n";
}

} // namespace

int main() {
    std::cout << "=========================================================\n";
    std::cout << "  Air Combat: MAWS, Countermeasures, Shot Discipline     \n";
    std::cout << "=========================================================\n";
    test_maws_heat_seeker_gets_flares();
    test_f16_reacts_only_once_it_sees_the_missile();
    test_novice_heat_seeker_discipline();
    test_symmetric_head_on_is_fair();
    test_every_loadout_ends_safely();
    std::cout << "\nAll air combat tests passed successfully!\n";
    return 0;
}
