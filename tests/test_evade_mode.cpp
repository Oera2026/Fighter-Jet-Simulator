/**
 * @file test_evade_mode.cpp
 * @brief Evade mode: a bandit with radar missiles and a gun hunts the player.
 *
 * Checks the set-up of every difficulty, the lock warning, the rules (bingo,
 * Winchester, escape), and that the fight rewards what works in real life
 * and punishes what doesn't:
 *  - flying straight and level is fatal at every level;
 *  - running in reheat defeats max-range shots but not no-escape shots;
 *  - a competent missile defence (beam, chaff, last-ditch break) survives
 *    far more often against a veteran than against an expert.
 * Fights are chaotic, so behavioural assertions use several seeds and wide
 * margins. Safety (no unforced ground impacts or mid-airs) and
 * determinism must hold on every seed.
 */
#include "fastjet/sim/engagement.hpp"
#include "fastjet/ui/screens/evade_setup_screen.hpp"
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>

using namespace fastjet;
using sim::EvadeDifficulty;

namespace {

constexpr double DT = 0.005;
constexpr EvadeDifficulty ALL_LEVELS[] = {EvadeDifficulty::EASY, EvadeDifficulty::MEDIUM, EvadeDifficulty::HARD,
                                          EvadeDifficulty::EXPERT};

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

struct Result {
    sim::EngagementOutcome outcome{};
    double time{0.0};
    sim::EngagementStats stats{};
    int unforced_ground_impacts{0};
    int midairs{0};
};

Result tally(const sim::Engagement& e) {
    Result r;
    r.outcome = e.outcome;
    r.time = e.world.time;
    r.stats = e.stats;
    for (int i = 0; i < 2; ++i) {
        const auto& a = e.world.aircraft[static_cast<size_t>(i)];
        if (a.crashed && a.damage.hits_taken == 0) ++r.unforced_ground_impacts;
    }
    for (int k = 0; k < e.world.event_count(); ++k) {
        if (e.world.event(k).kind == sim::CombatEvent::Kind::MIDAIR) ++r.midairs;
    }
    return r;
}

sim::EngagementSetup evade(EvadeDifficulty d, uint32_t seed) {
    sim::EngagementSetup s;
    s.mode = sim::EngagementMode::EVADE;
    s.evade_difficulty = d;
    s.seed = seed;
    return s;
}

/// The player flies level at @p throttle for the whole fight.
Result fly_level(EvadeDifficulty d, uint32_t seed, double throttle) {
    auto e = std::make_unique<sim::Engagement>();
    e->setup(evade(d, seed));
    LevelFlight pilot{throttle};
    while (!e->finished()) e->step(DT, pilot.update(e->own()));
    return tally(*e);
}

/// The AI's missile-defence pilot flies the player's jet.
Result fly_defended(EvadeDifficulty d, uint32_t seed) {
    auto s = evade(d, seed);
    s.own_is_ai = true;
    auto e = std::make_unique<sim::Engagement>();
    e->setup(s);
    const sim::AircraftControls none{};
    while (!e->finished()) e->step(DT, none);
    return tally(*e);
}

bool lost(const Result& r) {
    return r.outcome == sim::EngagementOutcome::LOSS || r.outcome == sim::EngagementOutcome::MUTUAL_KILL;
}

void test_setup_every_level() {
    std::cout << "[EVADE] Every level: bandit in your six, armed as briefed, radar hunting you...\n";
    for (EvadeDifficulty d : ALL_LEVELS) {
        auto e = std::make_unique<sim::Engagement>();
        e->setup(evade(d, 3));
        const auto p = sim::EvadeProfile::of(d);
        const auto& own = e->own();
        const auto& bandit = e->bandit();
        const math::Vector3 to_bandit = bandit.position() - own.position();
        const double behind = sim::AirCombatGeometry::angle_between(own.velocity(), to_bandit) * 180.0 / M_PI;
        std::cout << "    " << sim::to_string(d) << ": " << p.summary << ", " << to_bandit.norm() / 1852.0
                  << " nm, " << behind << " deg off the nose, " << p.survive_s << " s to survive\n";
        assert(std::abs(std::hypot(to_bandit.x, to_bandit.y) - p.start_range_m) < 1.0);
        assert(std::abs(-to_bandit.z - p.altitude_advantage_m) < 1.0);
        // Off the tail by up to off_tail_deg in azimuth, plus his height advantage.
        const double elevation = std::atan2(p.altitude_advantage_m, p.start_range_m) * 180.0 / M_PI;
        assert(behind > 180.0 - p.off_tail_deg - elevation - 1.0);
        assert(e->world.missile_load[sim::Engagement::BANDIT] == p.missiles);
        assert(e->world.radars[sim::Engagement::BANDIT].mode == sim::FireControlRadar::Mode::SEARCH);
        assert(e->setup_data.time_limit_s == p.survive_s);
        assert(bandit.velocity().norm() > own.velocity().norm() - 1e-6);
        assert(e->ai_controlled[sim::Engagement::BANDIT] && !e->ai_controlled[sim::Engagement::OWN]);
    }
    // Harder levels: a better pilot, more missiles, a better start.
    for (int i = 1; i < 4; ++i) {
        const auto a = sim::EvadeProfile::of(ALL_LEVELS[i - 1]);
        const auto b = sim::EvadeProfile::of(ALL_LEVELS[i]);
        assert(static_cast<int>(b.pilot) >= static_cast<int>(a.pilot));
        assert(b.missiles >= a.missiles);
        assert(b.radar_acquire_s <= a.radar_acquire_s);
        assert(b.speed_advantage_mps >= a.speed_advantage_mps);
    }
    // No radar, no missiles: an A-10 bandit hunts you with its gun and its eyes.
    auto s = evade(EvadeDifficulty::HARD, 3);
    s.bandit_type = aircraft::AircraftType::A10_THUNDERBOLT;
    auto e = std::make_unique<sim::Engagement>();
    e->setup(s);
    assert(e->world.missile_load[sim::Engagement::BANDIT] == 0);
    assert(e->shooter.doctrine == sim::LaunchDoctrine::NONE);
    std::cout << "  -> PASSED\n";
}

void test_lock_warning() {
    std::cout << "[EVADE] The RWR hears the search, then the lock, then the launch...\n";
    auto e = std::make_unique<sim::Engagement>();
    e->setup(evade(EvadeDifficulty::HARD, 2));
    LevelFlight pilot{0.8};
    sim::RwrStatus::Level first = sim::RwrStatus::Level::CLEAR;
    double t_lock = -1.0, t_launch = -1.0;
    while (e->world.time < 10.0 && !e->finished()) {
        e->step(DT, pilot.update(e->own()));
        const auto rwr = e->own_rwr();
        if (first == sim::RwrStatus::Level::CLEAR) first = rwr.level;
        if (t_lock < 0.0 && rwr.level == sim::RwrStatus::Level::LOCK) t_lock = e->world.time;
        if (t_launch < 0.0 && rwr.level == sim::RwrStatus::Level::LAUNCH) t_launch = e->world.time;
    }
    std::cout << "    first " << sim::to_string(first) << ", lock at " << t_lock << " s, launch at " << t_launch
              << " s\n";
    assert(first == sim::RwrStatus::Level::SEARCH);
    const double acquire = sim::EvadeProfile::of(EvadeDifficulty::HARD).radar_acquire_s;
    assert(t_lock > acquire - 0.01 && t_lock < acquire + 0.1);
    assert(t_launch > t_lock);
    std::cout << "  -> PASSED\n";
}

void test_flying_straight_is_fatal() {
    std::cout << "[EVADE] Straight and level at cruise is fatal at every level...\n";
    for (EvadeDifficulty d : ALL_LEVELS) {
        for (uint32_t seed = 1; seed <= 2; ++seed) {
            const Result r = fly_level(d, seed, 0.75);
            std::cout << "    " << sim::to_string(d) << " seed " << seed << ": " << sim::to_string(r.outcome) << " at "
                      << r.time << " s (" << r.stats.missiles_at_you << " missiles, " << r.stats.hits_taken
                      << " hits)\n";
            assert(lost(r));
            assert(r.unforced_ground_impacts == 0 && r.midairs == 0);
            // Guns only at EASY; the others kill with missiles.
            if (d == EvadeDifficulty::EASY) assert(r.stats.missiles_at_you == 0);
            else assert(r.stats.missile_detonations > 0);
        }
    }
    std::cout << "  -> PASSED\n";
}

void test_running_beats_rmax_shots_only() {
    std::cout << "[EVADE] Running in reheat defeats max-range shots, not no-escape shots...\n";
    for (uint32_t seed = 1; seed <= 3; ++seed) {
        const Result medium = fly_level(EvadeDifficulty::MEDIUM, seed, 1.0);
        const Result hard = fly_level(EvadeDifficulty::HARD, seed, 1.0);
        std::cout << "    seed " << seed << ": MEDIUM " << sim::to_string(medium.outcome) << " ("
                  << medium.stats.missiles_defeated << "/" << medium.stats.missiles_at_you << " missiles defeated), HARD "
                  << sim::to_string(hard.outcome) << " at " << hard.time << " s\n";
        assert(medium.outcome == sim::EngagementOutcome::SURVIVED);
        assert(medium.stats.missiles_at_you > 0 && medium.stats.missiles_defeated == medium.stats.missiles_at_you);
        assert(lost(hard));
    }
    std::cout << "  -> PASSED\n";
}

void test_defence_and_difficulty_ordering() {
    std::cout << "[EVADE] A competent missile defence: survives a veteran far more often than an expert...\n";
    constexpr uint32_t SEEDS = 8;
    int survived[4] = {0, 0, 0, 0};
    int defeated[4] = {0, 0, 0, 0}, fired[4] = {0, 0, 0, 0};
    for (int li : {1, 3}) {
        for (uint32_t seed = 1; seed <= SEEDS; ++seed) {
            const Result r = fly_defended(ALL_LEVELS[li], seed);
            survived[li] += !lost(r);
            defeated[li] += r.stats.missiles_defeated;
            fired[li] += r.stats.missiles_at_you;
            assert(r.unforced_ground_impacts == 0 && r.midairs == 0);
        }
    }
    std::cout << "    survived MEDIUM " << survived[1] << "/" << SEEDS << ", EXPERT " << survived[3] << "/" << SEEDS
              << "; missiles defeated MEDIUM " << defeated[1] << "/" << fired[1] << ", EXPERT " << defeated[3] << "/"
              << fired[3] << "\n";
    assert(survived[1] >= 3);
    assert(survived[1] >= survived[3] + 3);
    // Beaming, chaff and the break defeat most max-range shots. The expert's
    // no-escape salvos are built not to be defeated that way, but the defence
    // still beats some of them.
    assert(defeated[1] * 2 >= fired[1]);
    std::cout << "  -> PASSED\n";
}

void test_end_rules() {
    std::cout << "[EVADE] Bingo, Winchester and escape end the fight...\n";
    {   // Bingo: EASY, run and outlast him.
        const Result r = fly_level(EvadeDifficulty::EASY, 1, 1.0);
        std::cout << "    EASY runner: " << sim::to_string(r.outcome) << " at " << r.time << " s\n";
        assert(r.outcome == sim::EngagementOutcome::SURVIVED);
        assert(std::abs(r.time - sim::EvadeProfile::of(EvadeDifficulty::EASY).survive_s) < 0.01);
    }
    {   // Winchester: nothing left to shoot with.
        auto e = std::make_unique<sim::Engagement>();
        e->setup(evade(EvadeDifficulty::MEDIUM, 1));
        e->world.missile_load[sim::Engagement::BANDIT] = 0;
        e->world.aircraft[sim::Engagement::BANDIT].gun.ammo = 0;
        LevelFlight pilot{0.8};
        e->step(DT, pilot.update(e->own()));
        assert(e->outcome == sim::EngagementOutcome::SURVIVED);
    }
    {   // Escape: an A-10 (no radar) that lost you 35 km back gives up after 10 s.
        auto s = evade(EvadeDifficulty::MEDIUM, 1);
        s.bandit_type = aircraft::AircraftType::A10_THUNDERBOLT;
        auto e = std::make_unique<sim::Engagement>();
        e->setup(s);
        e->world.aircraft[sim::Engagement::OWN].state.pos_ned.x += 35000.0;
        e->world.aircraft[sim::Engagement::OWN].prev_state = e->world.aircraft[sim::Engagement::OWN].state;
        LevelFlight pilot{1.0};
        while (!e->finished() && e->world.time < 30.0) e->step(DT, pilot.update(e->own()));
        std::cout << "    A-10 left 35 km behind: " << sim::to_string(e->outcome) << " at " << e->world.time << " s\n";
        assert(e->outcome == sim::EngagementOutcome::ESCAPED);
        assert(std::abs(e->world.time - sim::Engagement::ESCAPE_HOLD_S) < 0.05);
    }
    std::cout << "  -> PASSED\n";
}

void test_replay_is_bit_exact() {
    std::cout << "[EVADE] Same seed, same inputs -> identical fight (missiles and chaff included)...\n";
    auto run = [] {
        auto s = evade(EvadeDifficulty::EXPERT, 5);
        s.own_is_ai = true;
        auto e = std::make_unique<sim::Engagement>();
        e->setup(s);
        const sim::AircraftControls none{};
        for (int i = 0; i < static_cast<int>(45.0 / DT) && !e->finished(); ++i) e->step(DT, none);
        return e;
    };
    const auto a = run();
    const auto b = run();
    assert(a->world.event_count() == b->world.event_count());
    assert(a->stats.missiles_at_you > 0 && a->stats.chaff_used > 0);
    assert(a->stats.chaff_used == b->stats.chaff_used);
    for (int i = 0; i < 2; ++i) {
        const auto& sa = a->world.aircraft[static_cast<size_t>(i)].state;
        const auto& sb = b->world.aircraft[static_cast<size_t>(i)].state;
        assert(std::memcmp(&sa.pos_ned, &sb.pos_ned, sizeof(sa.pos_ned)) == 0);
        assert(std::memcmp(&sa.q_att, &sb.q_att, sizeof(sa.q_att)) == 0);
    }
    std::cout << "  -> PASSED\n";
}

} // namespace

void test_menu_describes_every_level() {
    std::cout << "[EVADE] The set-up menu's threat descriptions match the profiles...\n";
    // The menu keeps its own copy so the UI does not depend on the simulation.
    static_assert(ui::kEvadeDifficulties.size() == 4);
    for (int i = 0; i < 4; ++i) {
        const auto d = static_cast<sim::EvadeDifficulty>(i);
        const auto p = sim::EvadeProfile::of(d);
        const ui::EvadeDifficultyInfo& m = ui::kEvadeDifficulties[static_cast<size_t>(i)];
        assert(std::strcmp(m.name, sim::to_string(d)) == 0);
        assert(m.survive_s == static_cast<int>(p.survive_s));
        assert(std::abs(m.start_nm - p.start_range_m / 1852.0) < 0.05);
        assert((p.missiles == 0) == (std::strstr(m.weapons, "Guns only") != nullptr));
    }
    std::cout << "  -> PASSED\n";
}

int main() {
    std::cout << "=========================================================\n";
    std::cout << "  Evade Mode                                             \n";
    std::cout << "=========================================================\n";
    test_menu_describes_every_level();
    test_setup_every_level();
    test_lock_warning();
    test_end_rules();
    test_replay_is_bit_exact();
    test_flying_straight_is_fatal();
    test_running_beats_rmax_shots_only();
    test_defence_and_difficulty_ordering();
    std::cout << "\nAll evade mode tests passed successfully!\n";
    return 0;
}
