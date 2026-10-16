/**
 * @file test_intercept_mode.cpp
 * @brief Intercept mode: evade mode the other way round. A bandit runs for
 * home; the player locks him up and shoots him down.
 *
 * Checks the set-up of every level (and the menu's description of it), the
 * rules (home, with your missiles in the air still counting; Winchester;
 * getting away; nothing counts after the result), what the bandit does at
 * each level (a novice ignores a lock, a veteran hears your search, notches a
 * lock and chaffs, only an expert shoots back, and only once you are close),
 * and that the levels get harder for an AI hunter who works the radar by the
 * player's rules (RWS scope, designate, find him again after chaff): he kills
 * every novice with one missile, needs more against a veteran and loses some
 * fights against the aces, in the A-10 too. Safety (no unforced ground
 * impacts) and determinism hold on every seed.
 */
#include "fastjet/sim/engagement.hpp"
#include "fastjet/sim/radar_scope.hpp"
#include "fastjet/ui/screens/evade_setup_screen.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>

using namespace fastjet;
using sim::InterceptDifficulty;
using aircraft::AircraftType;

namespace {

constexpr double DT = 0.005;
constexpr InterceptDifficulty ALL_LEVELS[] = {InterceptDifficulty::EASY, InterceptDifficulty::MEDIUM,
                                              InterceptDifficulty::HARD, InterceptDifficulty::EXPERT};

/// Wings level, holding the starting altitude, at a fixed lever.
struct LevelFlight {
    double throttle;
    double trim{0.0};
    double alt{-1.0};
    sim::AircraftControls update(const sim::Aircraft& me) {
        if (alt < 0.0) alt = me.state.altitude();
        sim::AircraftControls c{};
        c.throttle = throttle;
        const double climb = -me.state.velocity_ned().z;
        const double n = 1.0 + 0.003 * (alt - me.state.altitude()) - 0.03 * climb;
        trim = std::clamp(trim + 0.05 * (n - me.imu.Nz) * DT, -0.4, 0.4);
        c.stick.pitch_stick = std::clamp((n - 1.0) / 8.0 + trim, -0.5, 1.0);
        c.stick.roll_stick = std::clamp(-1.5 * me.state.roll() - 0.3 * me.state.omega_b.x, -1.0, 1.0);
        return c;
    }
};

sim::EngagementSetup setup_for(InterceptDifficulty d, AircraftType type, uint32_t seed, bool ai_hunter) {
    sim::EngagementSetup s;
    s.mode = sim::EngagementMode::INTERCEPT;
    s.intercept_difficulty = d;
    s.own_type = s.bandit_type = type;
    s.own_is_ai = ai_hunter;
    s.skill = sim::AiSkill::VETERAN;
    s.seed = seed;
    return s;
}

struct Result {
    sim::EngagementOutcome outcome{};
    double time{0.0};
    sim::EngagementStats stats{};
    int unforced_ground_impacts{0};
    int bandit_launches{0};
    double furthest_bandit_launch_m{0.0};
    bool hunter_designated{false}; ///< The AI hunter's scope reached STT (a designated lock)
};

/// One fight with the AI flying the hunter.
Result fly(InterceptDifficulty d, AircraftType type, uint32_t seed) {
    auto e = std::make_unique<sim::Engagement>();
    e->setup(setup_for(d, type, seed, true));
    Result r;
    int read = 0;
    while (!e->finished() && e->world.time < 400.0) {
        e->step(DT, {});
        r.hunter_designated = r.hunter_designated || e->hunter_scope().mode == sim::RadarScope::Mode::STT;
        for (; read < e->world.event_count(); ++read) {
            const sim::CombatEvent& ev = e->world.event(read);
            if (ev.kind != sim::CombatEvent::Kind::MISSILE_LAUNCH || ev.shooter != sim::Engagement::BANDIT) continue;
            ++r.bandit_launches;
            r.furthest_bandit_launch_m =
                std::max(r.furthest_bandit_launch_m, (e->bandit().position() - e->own().position()).norm());
        }
    }
    r.outcome = e->outcome;
    r.time = e->world.time;
    r.stats = e->stats;
    for (int k = 0; k < e->world.event_count(); ++k) {
        const sim::CombatEvent& ev = e->world.event(k);
        if (ev.kind == sim::CombatEvent::Kind::GROUND_IMPACT && ev.shooter < 0) ++r.unforced_ground_impacts;
    }
    return r;
}

void test_setup() {
    std::cout << "[Test] Set-up: bandit ahead and running, your missiles, his countermeasures... " << std::flush;
    for (InterceptDifficulty d : ALL_LEVELS) {
        const sim::InterceptProfile p = sim::InterceptProfile::of(d);
        for (uint32_t seed = 1; seed <= 8; ++seed) {
            auto e = std::make_unique<sim::Engagement>();
            const sim::EngagementSetup s = setup_for(d, AircraftType::F16_FIGHTING_FALCON, seed, false);
            e->setup(s);
            const sim::Aircraft& me = e->own();
            const sim::Aircraft& him = e->bandit();
            const math::Vector3 los = him.position() - me.position();
            const double range = std::hypot(los.x, los.y);
            constexpr double SPREAD = sim::Engagement::START_RANGE_SPREAD;
            assert(range <= (1.0 + SPREAD) * p.start_range_m + 1.0 && range >= (1.0 - SPREAD) * p.start_range_m - 1.0);
            // Ahead of you, within the level's angle off your nose.
            const double off = std::atan2(std::abs(los.y), los.x) * 180.0 / M_PI;
            assert(off <= p.off_nose_deg + 0.01);
            // Running: his heading within 60 deg of the line from you to him.
            const double away = std::abs(std::remainder(him.state.yaw() - std::atan2(los.y, los.x), 2.0 * M_PI));
            assert(away < 60.0 * M_PI / 180.0);
            assert(std::abs(him.state.altitude() - me.state.altitude() - p.altitude_offset_m) <
                   sim::Engagement::START_HEIGHT_SPREAD_M + 1.0);
            assert(std::abs(him.state.airspeed() - me.state.airspeed() - p.speed_offset_mps) <
                   sim::Engagement::START_SPEED_SPREAD_MPS + 1.0);
            // You carry your full load; he carries countermeasures by level.
            assert(e->world.stores[sim::Engagement::OWN].mrm.count > 0);
            assert(e->world.stores[sim::Engagement::OWN].srm.count > 0);
            assert((e->world.chaff_load[sim::Engagement::BANDIT] > 0) == p.chaff);
            assert((e->world.flare_load[sim::Engagement::BANDIT] > 0) == p.flares);
            // Only an expert carries missiles to fight back with.
            assert((e->world.stores[sim::Engagement::BANDIT].missiles() > 0) == p.fights_back);
            assert(e->world.stores[sim::Engagement::BANDIT].mrm.count == 0);
            assert(std::abs(e->setup_data.time_limit_s - p.home_s) < 1e-9);
        }
    }
    // No two fights at a level open alike.
    {
        auto a = std::make_unique<sim::Engagement>(), b = std::make_unique<sim::Engagement>();
        a->setup(setup_for(InterceptDifficulty::MEDIUM, AircraftType::F16_FIGHTING_FALCON, 1, false));
        b->setup(setup_for(InterceptDifficulty::MEDIUM, AircraftType::F16_FIGHTING_FALCON, 2, false));
        assert((a->bandit().position() - b->bandit().position()).norm() > 100.0);
    }
    // A hunter without a radar starts within heat-seeker reach of a slower
    // bandit, who flares where he would have chaffed.
    for (InterceptDifficulty d : ALL_LEVELS) {
        auto e = std::make_unique<sim::Engagement>();
        const sim::EngagementSetup s = setup_for(d, AircraftType::A10_THUNDERBOLT, 3, false);
        e->setup(s);
        const double range = (e->bandit().position() - e->own().position()).norm();
        assert(range <= (1.0 + sim::Engagement::START_RANGE_SPREAD) * sim::Engagement::NO_RADAR_START_M + 100.0);
        assert(e->bandit().state.airspeed() < e->own().state.airspeed());
        const auto p = sim::Engagement::intercept_profile(s);
        assert(p.flares == (p.chaff || sim::InterceptProfile::of(d).flares));
        assert((e->world.flare_load[sim::Engagement::BANDIT] > 0) == p.flares);
    }
    // The levels: harder bandits react faster, start further away and carry more.
    for (size_t i = 1; i < std::size(ALL_LEVELS); ++i) {
        const auto a = sim::InterceptProfile::of(ALL_LEVELS[i - 1]);
        const auto b = sim::InterceptProfile::of(ALL_LEVELS[i]);
        assert(b.start_range_m >= a.start_range_m);
        assert(a.reaction_s < 0.0 || (b.reaction_s >= 0.0 && b.reaction_s < a.reaction_s));
        assert(b.speed_offset_mps >= a.speed_offset_mps);
        assert(int(b.chaff) + int(b.flares) + int(b.fights_back) >= int(a.chaff) + int(a.flares) + int(a.fights_back));
    }
    std::cout << "PASSED\n";
}

void test_menu_matches_profiles() {
    std::cout << "[Test] The set-up screen describes the levels as flown... " << std::flush;
    static_assert(ui::kInterceptDifficulties.size() == 4);
    for (int i = 0; i < 4; ++i) {
        const auto d = static_cast<InterceptDifficulty>(i);
        const auto p = sim::InterceptProfile::of(d);
        const ui::EvadeDifficultyInfo& m = ui::kInterceptDifficulties[static_cast<size_t>(i)];
        assert(std::string(m.name) == sim::to_string(d));
        assert(m.survive_s == static_cast<int>(p.home_s));
        assert(std::abs(m.start_nm - p.start_range_m / 1852.0) < 0.05);
    }
    std::cout << "PASSED\n";
}

/// How far the bandit turns away from his starting heading in @p seconds
/// while the hunter flies level holding a radar lock on him.
double bandit_turn_under_lock(InterceptDifficulty d, double seconds, int* chaff_used) {
    auto e = std::make_unique<sim::Engagement>();
    e->setup(setup_for(d, AircraftType::F16_FIGHTING_FALCON, 5, false));
    e->world.radars[sim::Engagement::OWN].enable(sim::Engagement::BANDIT, 0.5);
    const double psi0 = e->bandit().state.yaw();
    const int chaff0 = e->world.chaff_load[sim::Engagement::BANDIT];
    LevelFlight pilot{0.9};
    double turned = 0.0;
    while (e->world.time < seconds && !e->finished()) {
        e->step(DT, pilot.update(e->own()));
        turned = std::max(turned, std::abs(std::remainder(e->bandit().state.yaw() - psi0, 2.0 * M_PI)));
    }
    if (chaff_used) *chaff_used = chaff0 - e->world.chaff_load[sim::Engagement::BANDIT];
    return turned * 180.0 / M_PI;
}

void test_bandit_reactions() {
    std::cout << "[Test] A novice flies on under a lock; a veteran notches it and chaffs... " << std::flush;
    int chaff = 0;
    const double easy = bandit_turn_under_lock(InterceptDifficulty::EASY, 20.0, &chaff);
    assert(easy < 15.0 && chaff == 0);
    const double medium = bandit_turn_under_lock(InterceptDifficulty::MEDIUM, 30.0, &chaff);
    std::cout << "(turns " << static_cast<int>(easy) << " vs " << static_cast<int>(medium) << " deg, chaff "
              << chaff << ") " << std::flush;
    assert(medium > 45.0);
    assert(chaff > 0);
    std::cout << "PASSED\n";
}

/// The bandit's RWR hears the player's RWS search before any lock: a veteran
/// turns cold on it, a novice never looks.
void test_search_is_heard() {
    std::cout << "[Test] Your RWS search shows on his RWR; a veteran turns cold on it... " << std::flush;
    for (InterceptDifficulty d : {InterceptDifficulty::EASY, InterceptDifficulty::MEDIUM}) {
        auto e = std::make_unique<sim::Engagement>();
        e->setup(setup_for(d, AircraftType::F16_FIGHTING_FALCON, 4, false));
        sim::RadarScope scope;
        scope.sync(e->world, sim::Engagement::OWN);
        scope.set_master_mode(e->world, false);
        LevelFlight pilot{0.9};
        bool heard = false;
        double cold = 1e9;
        while (e->world.time < 20.0 && !e->finished()) {
            e->step(DT, pilot.update(e->own()));
            scope.step(DT, e->world, sim::Engagement::OWN);
            assert(scope.mode == sim::RadarScope::Mode::RWS); // never designated: no lock
            const sim::RwrStatus rwr = e->world.rwr(sim::Engagement::BANDIT);
            if (rwr.level == sim::RwrStatus::Level::SEARCH) {
                heard = true;
                assert(std::string(rwr.emitter_symbol) == "16");
                // Off his tail: how far the emitter is from dead astern.
                cold = std::min(cold, M_PI - std::abs(rwr.emitter_bearing));
            }
        }
        assert(heard);
        if (d == InterceptDifficulty::MEDIUM) {
            // He has put you within 20 deg of his tail.
            assert(cold < 20.0 * M_PI / 180.0);
            assert(e->evader.mode == sim::EvasionPilot::Mode::CRUISE);
        }
    }
    std::cout << "PASSED\n";
}

void test_rules() {
    std::cout << "[Test] Rules: home, Winchester and getting away... " << std::flush;
    {
        // He reaches home.
        auto e = std::make_unique<sim::Engagement>();
        e->setup(setup_for(InterceptDifficulty::EASY, AircraftType::F16_FIGHTING_FALCON, 1, false));
        e->setup_data.time_limit_s = 5.0;
        LevelFlight pilot{0.7};
        while (!e->finished() && e->world.time < 10.0) e->step(DT, pilot.update(e->own()));
        assert(e->outcome == sim::EngagementOutcome::GOT_AWAY);
        assert(std::abs(e->world.time - 5.0) < 0.01);
        assert(std::string(sim::to_string(e->outcome)) == "BANDIT GOT AWAY");
    }
    {
        // Winchester: nothing left to shoot with and nothing in the air.
        auto e = std::make_unique<sim::Engagement>();
        e->setup(setup_for(InterceptDifficulty::EASY, AircraftType::F16_FIGHTING_FALCON, 1, false));
        e->world.stores[sim::Engagement::OWN].clear();
        e->own().gun.ammo = 0;
        LevelFlight pilot{0.7};
        e->step(DT, pilot.update(e->own()));
        assert(e->outcome == sim::EngagementOutcome::GOT_AWAY);
    }
    {
        // Beyond escape range with no lock on him: gone after the hold time.
        auto e = std::make_unique<sim::Engagement>();
        e->setup(setup_for(InterceptDifficulty::EASY, AircraftType::F16_FIGHTING_FALCON, 1, false));
        e->own().state.pos_ned = e->bandit().position() - math::Vector3(sim::Engagement::INTERCEPT_ESCAPE_RANGE_M + 5000.0, 0.0, 0.0);
        LevelFlight pilot{0.0};
        while (!e->finished() && e->world.time < 30.0) e->step(DT, pilot.update(e->own()));
        assert(e->outcome == sim::EngagementOutcome::GOT_AWAY);
        assert(std::abs(e->world.time - sim::Engagement::ESCAPE_HOLD_S) < 0.1);
    }
    {
        // A lock on him counts, and the first lock is timed.
        auto e = std::make_unique<sim::Engagement>();
        e->setup(setup_for(InterceptDifficulty::EASY, AircraftType::F16_FIGHTING_FALCON, 1, false));
        e->world.radars[sim::Engagement::OWN].enable(sim::Engagement::BANDIT, 1.0);
        LevelFlight pilot{0.9};
        while (e->world.time < 5.0) e->step(DT, pilot.update(e->own()));
        assert(e->own_locked_on_bandit());
        assert(e->stats.first_lock_s > 0.5 && e->stats.first_lock_s < 3.0);
        assert(e->stats.time_on_lock_s > 1.0);
    }
    {
        // A jet without radar holds him by heat-seeker tone.
        auto e = std::make_unique<sim::Engagement>();
        e->setup(setup_for(InterceptDifficulty::EASY, AircraftType::A10_THUNDERBOLT, 1, false));
        assert(!e->own_locked_on_bandit());
        e->world.seekers[sim::Engagement::OWN].locked = true;
        e->world.seekers[sim::Engagement::OWN].target = sim::Engagement::BANDIT;
        assert(e->own_locked_on_bandit());
    }
    {
        // A pickle press with the gun selected is answered (the HUD cues GUN SEL).
        auto e = std::make_unique<sim::Engagement>();
        e->setup(setup_for(InterceptDifficulty::EASY, AircraftType::F16_FIGHTING_FALCON, 1, false));
        e->world.stores[sim::Engagement::OWN].select(sim::WeaponSel::GUN);
        LevelFlight pilot{0.9};
        sim::AircraftControls c = pilot.update(e->own());
        c.pickle = true;
        e->step(DT, c);
        assert(e->world.pickle_presses[sim::Engagement::OWN] == 1);
        assert(e->world.launch_cue[sim::Engagement::OWN] == sim::World::LaunchCue::GUN);
        assert(e->world.missiles_in_flight() == 0);
    }
    {
        // Home with your missile in the air: it still gets its chance. No new
        // shots once he is home, and nothing counts after the result.
        auto e = std::make_unique<sim::Engagement>();
        e->setup(setup_for(InterceptDifficulty::EASY, AircraftType::F16_FIGHTING_FALCON, 1, false));
        e->world.radars[sim::Engagement::OWN].enable(sim::Engagement::BANDIT, 0.5);
        e->world.stores[sim::Engagement::OWN].select(sim::WeaponSel::MRM);
        LevelFlight pilot{0.9};
        bool fired = false, pressed = false;
        while (!e->finished() && e->world.time < 60.0) {
            sim::AircraftControls c = pilot.update(e->own());
            // One shot as soon as the lock has settled, then the time runs out.
            if (!fired && e->own_locked_on_bandit() && e->world.radars[sim::Engagement::OWN].lock_time > 0.5) {
                c.pickle = true;
                fired = true;
                e->setup_data.time_limit_s = e->world.time + 1.0;
            } else if (e->bandit_home()) {
                // Keep pressing at home: nothing more leaves the rails.
                c.pickle = pressed = !pressed;
                c.trigger = true;
            }
            e->step(DT, c);
            if (e->bandit_home() && !e->finished()) assert(e->world.missiles_in_flight(sim::Engagement::OWN) == 1);
        }
        assert(fired);
        assert(e->world.time > e->setup_data.time_limit_s + 1.0); // the missile flew on past home time
        assert(e->stats.missiles_fired == 1);
        assert(e->outcome == sim::EngagementOutcome::WIN || e->outcome == sim::EngagementOutcome::GOT_AWAY);
        const sim::EngagementStats at_end = e->stats;
        LevelFlight after{0.9};
        for (int k = 0; k < 2000; ++k) {
            sim::AircraftControls c = after.update(e->own());
            c.pickle = (k / 10) % 2 == 0;
            e->step(DT, c);
        }
        assert(e->stats.missiles_fired == at_end.missiles_fired && e->stats.missiles_missed == at_end.missiles_missed);
        assert(e->world.missiles_in_flight(sim::Engagement::OWN) == 0);
    }
    std::cout << "PASSED\n";
}

void test_levels_get_harder() {
    std::cout << "[Test] An AI hunter: one missile kills a novice, an ace takes more; only an expert shoots back... "
              << std::flush;
    constexpr int SEEDS = 8;
    int expert_launches = 0;
    for (AircraftType type : {AircraftType::F16_FIGHTING_FALCON, AircraftType::EUROFIGHTER_TYPHOON,
                              AircraftType::A10_THUNDERBOLT}) {
        const bool radar = sim::radar_spec(type).fitted;
        std::array<int, 4> wins{}, fired{}, missed{};
        for (size_t l = 0; l < 4; ++l) {
            for (int s = 1; s <= SEEDS; ++s) {
                const Result r = fly(ALL_LEVELS[l], type, static_cast<uint32_t>(s * 7919 + 13));
                assert(r.unforced_ground_impacts == 0);
                wins[l] += r.outcome == sim::EngagementOutcome::WIN;
                fired[l] += r.stats.missiles_fired;
                missed[l] += r.stats.missiles_missed;
                // A hunter with a radar finds him through the scope.
                if (radar) assert(r.hunter_designated);
                // He shoots only once he has turned to fight, inside visual range.
                if (!sim::InterceptProfile::of(ALL_LEVELS[l]).fights_back) assert(r.bandit_launches == 0);
                assert(r.furthest_bandit_launch_m < sim::Engagement::VISUAL_FIGHT_RANGE_M + 500.0);
                if (ALL_LEVELS[l] == InterceptDifficulty::EXPERT) expert_launches += r.bandit_launches;
            }
        }
        std::cout << "\n    " << aircraft::to_string(type) << ": wins " << wins[0] << "/" << wins[1] << "/" << wins[2] << "/"
                  << wins[3] << ", missiles " << fired[0] << "/" << fired[1] << "/" << fired[2] << "/" << fired[3]
                  << ", missed " << missed[0] << "/" << missed[1] << "/" << missed[2] << "/" << missed[3];
        // A novice flies on: every first missile kills him.
        assert(wins[0] == SEEDS && fired[0] == SEEDS && missed[0] == 0);
        // A veteran defeats some missiles; the aces get away from some fights.
        assert(missed[1] > 0);
        assert(wins[3] <= wins[1] && wins[3] < SEEDS);
        if (radar) {
            assert(missed[2] > missed[1] && missed[3] > missed[1]);
            assert(fired[2] > fired[1] && fired[3] > fired[1]);
            assert(wins[2] <= wins[1] && wins[2] < SEEDS);
        }
    }
    std::cout << "\n    expert heat-seeker shots: " << expert_launches;
    assert(expert_launches > 0);
    std::cout << "\n  PASSED\n";
}

void test_determinism() {
    std::cout << "[Test] Determinism: the same seed flies the same fight... " << std::flush;
    const Result a = fly(InterceptDifficulty::HARD, AircraftType::F16_FIGHTING_FALCON, 42);
    const Result b = fly(InterceptDifficulty::HARD, AircraftType::F16_FIGHTING_FALCON, 42);
    assert(a.outcome == b.outcome);
    assert(a.time == b.time);
    assert(a.stats.missiles_fired == b.stats.missiles_fired);
    assert(a.stats.locks_he_broke == b.stats.locks_he_broke);
    std::cout << "PASSED\n";
}

} // namespace

int main() {
    std::cout << "=== Intercept mode ===\n";
    test_setup();
    test_menu_matches_profiles();
    test_bandit_reactions();
    test_search_is_heard();
    test_rules();
    test_levels_get_harder();
    test_determinism();
    std::cout << "All intercept mode tests passed.\n";
    return 0;
}
