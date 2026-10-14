#pragma once

#include "fastjet/sim/bfm_pilot.hpp"
#include "fastjet/sim/evasion_pilot.hpp"
#include "fastjet/sim/missile_shooter.hpp"
#include "fastjet/sim/world.hpp"
#include <array>
#include <cmath>
#include <cstdint>

namespace fastjet::sim {

/// @brief How the two jets are set up when the fight starts.
enum class StartGeometry : uint8_t {
    HEAD_ON_MERGE,    ///< Neutral: 2 nm apart, nose to nose, offset for a left-to-left pass
    OFFENSIVE_PERCH,  ///< Player 3,000 ft behind the bandit, 30 deg off his tail, 1,000 ft high
    DEFENSIVE_PERCH,  ///< The same with the roles reversed
    GUNNERY_RANGE,    ///< Scripted, non-shooting target for gun and pursuit practice
};

/// @brief What a gunnery-range target flies.
enum class RangeProfile : uint8_t { STRAIGHT_AND_LEVEL, CONSTANT_TURN, WEAVE };

[[nodiscard]] constexpr const char* to_string(StartGeometry g) noexcept {
    switch (g) {
        case StartGeometry::HEAD_ON_MERGE:   return "HEAD-ON MERGE";
        case StartGeometry::OFFENSIVE_PERCH: return "OFFENSIVE PERCH";
        case StartGeometry::DEFENSIVE_PERCH: return "DEFENSIVE PERCH";
        case StartGeometry::GUNNERY_RANGE:   return "GUNNERY RANGE";
    }
    return "?";
}

/// @brief Which game is being played.
enum class EngagementMode : uint8_t {
    DOGFIGHT, ///< 1v1 guns BFM from a chosen set-up
    EVADE,    ///< Defensive: a bandit with radar missiles and a gun hunts you from your six
};

/// @brief Threat level for evade mode.
enum class EvadeDifficulty : uint8_t { EASY, MEDIUM, HARD, EXPERT };

[[nodiscard]] constexpr const char* to_string(EvadeDifficulty d) noexcept {
    switch (d) {
        case EvadeDifficulty::EASY:   return "EASY";
        case EvadeDifficulty::MEDIUM: return "MEDIUM";
        case EvadeDifficulty::HARD:   return "HARD";
        case EvadeDifficulty::EXPERT: return "EXPERT";
    }
    return "?";
}

/**
 * @brief What a difficulty level changes. The physics never changes: every
 * level flies the same jets, radar and missiles. What changes is the pilot
 * (perception, G tolerance, gunnery), his launch discipline and missile load,
 * and how much of an advantage he starts with.
 */
struct EvadeProfile {
    AiSkill pilot;
    int missiles;
    LaunchDoctrine doctrine;
    bool shoot_shoot;            ///< Ripples two missiles per shot
    double radar_acquire_s;      ///< Operator's time to lock you up in ACM
    double start_range_m;        ///< Directly astern at the start
    double off_tail_deg;         ///< Up to this far off your tail
    double altitude_advantage_m;
    double speed_advantage_mps;
    double survive_s;            ///< Bandit bingo fuel: hold out this long
    const char* summary;

    [[nodiscard]] static constexpr EvadeProfile of(EvadeDifficulty d) noexcept {
        switch (d) {
            case EvadeDifficulty::EASY:
                return {AiSkill::NOVICE, 0, LaunchDoctrine::NONE, false, 2.5, 5556.0, 25.0, 0.0, 0.0, 120.0,
                        "NOVICE, GUNS ONLY"};
            case EvadeDifficulty::MEDIUM:
                return {AiSkill::VETERAN, 2, LaunchDoctrine::MAX_RANGE, false, 1.5, 18520.0, 20.0, 300.0, 20.0, 150.0,
                        "VETERAN, 2 AIM-120, RMAX SHOTS"};
            case EvadeDifficulty::HARD:
                return {AiSkill::ACE, 4, LaunchDoctrine::NO_ESCAPE, false, 1.0, 12000.0, 15.0, 600.0, 40.0, 180.0,
                        "ACE, 4 AIM-120, NO-ESCAPE SHOTS"};
            case EvadeDifficulty::EXPERT:
                return {AiSkill::ACE, 4, LaunchDoctrine::NO_ESCAPE, true, 0.8, 11112.0, 10.0, 900.0, 60.0, 210.0,
                        "ACE, 4 AIM-120, SHOOT-SHOOT"};
        }
        return of(EvadeDifficulty::MEDIUM);
    }
};

enum class WeaponsLoadout : uint8_t {
    GUNS_ONLY,
    GUNS_AND_IR,
    FULL_LOADOUT,
};

/**
 * @brief How an AI pilot of each skill employs missiles in a dogfight.
 * Every skill flies the same missiles; what changes is how far off the nose
 * he will cue the heat seeker and trust its lock, how long he waits for a
 * steady tone, how he spaces his shots, his radar-missile doctrine and how
 * fast he locks you up with the radar.
 */
struct AiMissileDiscipline {
    double off_boresight_deg;   ///< Heat-seeker cueing and shot limit (an ace's helmet sight: the whole gimbal)
    double confirm_lock_s;      ///< Tone held before pickling
    double spacing_s;           ///< Least time between heat-seeker shots
    double max_off_tail_deg;    ///< Heat-seeker shots only this far off the target's tail...
    double strong_off_tail_deg; ///< ...or this far on a strong tone (see HeatSeekerShooter)
    LaunchDoctrine mrm_doctrine;
    double radar_acquire_s;     ///< Time to lock the bandit up with the radar

    /// A novice shoots heat seekers only from the target's rear hemisphere,
    /// where the tail pipe fills the seeker. A veteran shoots from the beam or
    /// behind it, and forward of the beam only on a strong tone. An ace trusts
    /// the all-aspect seeker from any angle.
    [[nodiscard]] static constexpr AiMissileDiscipline of(AiSkill s) noexcept {
        switch (s) {
            case AiSkill::NOVICE:  return {15.0, 1.0, 8.0, 90.0, 90.0, LaunchDoctrine::MAX_RANGE, 2.5};
            case AiSkill::VETERAN: return {30.0, 0.6, 6.0, 120.0, 180.0, LaunchDoctrine::MAX_RANGE, 1.5};
            case AiSkill::ACE:     return {180.0, 0.4, 4.0, 180.0, 180.0, LaunchDoctrine::NO_ESCAPE, 1.0};
        }
        return of(AiSkill::VETERAN);
    }
};

[[nodiscard]] constexpr const char* to_string(WeaponsLoadout w) noexcept {
    switch (w) {
        case WeaponsLoadout::GUNS_ONLY:    return "GUNS ONLY";
        case WeaponsLoadout::GUNS_AND_IR:  return "GUNS + HEAT SEEKERS";
        case WeaponsLoadout::FULL_LOADOUT: return "FULL COMBAT LOAD";
    }
    return "?";
}

struct EngagementSetup {
    EngagementMode mode{EngagementMode::DOGFIGHT};
    EvadeDifficulty evade_difficulty{EvadeDifficulty::MEDIUM};
    aircraft::AircraftType own_type{aircraft::AircraftType::F16_FIGHTING_FALCON};
    aircraft::AircraftType bandit_type{aircraft::AircraftType::F16_FIGHTING_FALCON};
    AiSkill skill{AiSkill::VETERAN};
    StartGeometry geometry{StartGeometry::HEAD_ON_MERGE};
    RangeProfile range_profile{RangeProfile::CONSTANT_TURN};
    WeaponsLoadout loadout{WeaponsLoadout::GUNS_ONLY};
    double altitude_m{5000.0};
    double speed_mps{250.0};
    double hard_deck_agl_m{1000.0};
    double time_limit_s{300.0};
    uint32_t seed{1};
    bool own_is_ai{false};    ///< AI flies the player's jet too (tests, attract mode)
};

enum class EngagementOutcome : uint8_t { IN_PROGRESS, WIN, LOSS, MUTUAL_KILL, TIMEOUT, SURVIVED, ESCAPED };

[[nodiscard]] constexpr const char* to_string(EngagementOutcome o) noexcept {
    switch (o) {
        case EngagementOutcome::IN_PROGRESS: return "FIGHT'S ON";
        case EngagementOutcome::WIN:         return "SPLASH ONE - VICTORY";
        case EngagementOutcome::LOSS:        return "YOU WERE SHOT DOWN";
        case EngagementOutcome::MUTUAL_KILL: return "MUTUAL KILL";
        case EngagementOutcome::TIMEOUT:     return "KNOCK IT OFF - TIME";
        case EngagementOutcome::SURVIVED:    return "SURVIVED - BANDIT RTB";
        case EngagementOutcome::ESCAPED:     return "ESCAPED - BANDIT LOST CONTACT";
    }
    return "?";
}

/// @brief Tallies for the debrief.
struct EngagementStats {
    int rounds_fired{0};
    int hits_scored{0};
    int hits_taken{0};
    double time_above_7g_s{0.0};
    double peak_g{0.0};
    double min_agl_m{1e9};
    double duration_s{0.0};
    // Evade mode
    int missiles_at_you{0};
    int missiles_defeated{0};   ///< Ran out of energy, time or track
    int missile_detonations{0}; ///< Fuzed near you (damaging or not)
    int chaff_used{0};
    int locks_broken_by_chaff{0};
    double time_locked_s{0.0};
};

/**
 * @brief Straight, turning or weaving target for the gunnery range.
 *
 * Holds altitude with a bank-angle schedule; flies through the same FLCS as
 * everyone else, so its turn is a real one.
 */
class RangeTargetPilot {
public:
    RangeProfile profile{RangeProfile::CONSTANT_TURN};
    double target_altitude_m{5000.0};
    double bank_deg{60.0};         ///< Constant-turn bank (2 G level turn)
    double weave_period_s{12.0};
    double target_speed_mps{230.0};

    [[nodiscard]] AircraftControls update(double dt, const Aircraft& self) noexcept {
        time_ += dt;
        AircraftControls out{};
        // Speed hold: a target that accelerates away through Mach 1 is no use for gunnery.
        const double v_err = target_speed_mps - self.state.airspeed();
        throttle_ = std::clamp(throttle_ + 0.01 * v_err * dt, 0.0, 1.0);
        out.throttle = std::clamp(throttle_ + 0.03 * v_err, 0.0, 1.0);
        const auto cfg = aircraft::AircraftConfig::get(self.type);
        double bank = 0.0;
        switch (profile) {
            case RangeProfile::STRAIGHT_AND_LEVEL: bank = 0.0; break;
            case RangeProfile::CONSTANT_TURN:      bank = bank_deg; break;
            case RangeProfile::WEAVE:
                bank = bank_deg * (std::sin(2.0 * M_PI * time_ / weave_period_s) >= 0.0 ? 1.0 : -1.0);
                break;
        }
        const double phi_des = bank * (M_PI / 180.0);
        const double phi = self.state.roll();
        out.stick.roll_stick = std::clamp(1.5 * (phi_des - phi) - 0.3 * self.state.omega_b.x, -1.0, 1.0);

        // Level-turn load factor plus an altitude / climb-rate hold.
        const double climb = -self.state.velocity_ned().z;
        const double alt_err = target_altitude_m - self.state.altitude();
        const double n_level = 1.0 / std::max(0.2, std::cos(phi));
        const double n_cmd = std::clamp(n_level + 0.002 * alt_err - 0.02 * climb, 0.0, 5.0);
        const bool fbw = cfg.flcs.law_type != aircraft::FLCSConfig::LawType::HYDRO_SAS_AUGMENTED;
        const double err = n_cmd - self.imu.Nz;
        trim_ = std::clamp(trim_ + (fbw ? 0.05 : 0.3) * err * dt, -0.4, 0.4);
        const double ff = fbw ? (n_cmd - 1.0) / std::max(1.0, cfg.flcs.max_g_positive - 1.0) : 0.05 * (n_cmd - 1.0);
        out.stick.pitch_stick = std::clamp(ff + trim_, -0.5, 1.0);
        return out;
    }

private:
    double time_{0.0};
    double trim_{0.0};
    double throttle_{0.7};
};

/**
 * @brief One dogfight: the world, the AI pilots, the rules and the score.
 *
 * Index 0 is always the player's jet. The engagement ends on a kill (or a
 * mutual kill), when anyone busts through the ground, or at the time limit.
 */
class Engagement {
public:
    static constexpr int OWN = 0;
    static constexpr int BANDIT = 1;

    World world{};
    EngagementSetup setup_data{};
    std::array<BfmPilot, World::kMaxAircraft> ai{};
    std::array<bool, World::kMaxAircraft> ai_controlled{};
    RangeTargetPilot range_pilot{};
    MissileShooter shooter{};    ///< The bandit's missile shot decisions (evade mode)
    std::array<HeatSeekerShooter, World::kMaxAircraft> srm_shooters{}; ///< AI heat-seeker shots (dogfight)
    std::array<MissileShooter, World::kMaxAircraft> mrm_shooters{};    ///< AI radar-missile shots (dogfight)
    EvasionPilot evader{};       ///< Flies the player's jet in an AI-flown evade (demo, tests)
    EngagementStats stats{};
    EngagementOutcome outcome{EngagementOutcome::IN_PROGRESS};
    double end_time{-1.0};

    /// @brief Start a new fight.
    void setup(const EngagementSetup& s) noexcept {
        setup_data = s;
        world.clear(s.seed);
        outcome = EngagementOutcome::IN_PROGRESS;
        stats = EngagementStats{};
        end_time = -1.0;
        events_read_ = 0;
        ai_controlled.fill(false);

        const double alt = s.altitude_m;
        const double v = s.speed_mps;
        const bool evade = s.mode == EngagementMode::EVADE;
        const EvadeProfile profile = EvadeProfile::of(s.evade_difficulty);
        const AiSkill bandit_skill = evade ? profile.pilot : s.skill;
        fdm::FlightState own{}, bandit{};
        if (evade) {
            evade_start(own, bandit, s, profile);
        } else {
            switch (s.geometry) {
                case StartGeometry::HEAD_ON_MERGE:
                    own = make_state({-1852.0, -150.0, -alt}, 0.0, v);
                    bandit = make_state({1852.0, 150.0, -alt}, M_PI, v);
                    break;
                case StartGeometry::OFFENSIVE_PERCH:
                case StartGeometry::GUNNERY_RANGE:
                    perch(own, bandit, alt, v);
                    break;
                case StartGeometry::DEFENSIVE_PERCH:
                    perch(bandit, own, alt, v);
                    break;
            }
        }

        world.add(s.own_type, own);
        world.add(s.bandit_type, bandit);
        for (int i = 0; i < world.count; ++i) {
            Aircraft& a = world.aircraft[static_cast<size_t>(i)];
            a.landing_gear.deployed = false;
            // Still air (the gusts stay): missiles and rounds fly in still
            // air, so the jets must too. In a wind, the jet with the tailwind
            // in a head-on flew faster over the ground into the other's
            // missile and was hit first: jet 0 won two fights in three.
            a.wind.mean_wind_enabled = false;
            prespool(a, 0.9);
        }
        maws_memory_.fill(MawsMemory{});

        ai_controlled[OWN] = s.own_is_ai;
        ai_controlled[BANDIT] = evade || s.geometry != StartGeometry::GUNNERY_RANGE;
        for (int i = 0; i < world.count; ++i) {
            ai[static_cast<size_t>(i)] =
                BfmPilot(i == BANDIT ? bandit_skill : s.skill, s.seed * 2654435761u + static_cast<uint32_t>(i) * 97u + 1u);
            ai[static_cast<size_t>(i)].hard_deck_agl_m = s.hard_deck_agl_m;
        }
        // A bandit held to transonic speed let anyone escape by running in
        // reheat. He chases supersonic, still held to the pilot's dynamic
        // pressure limits, where the FLCS behaves.
        ai[BANDIT].max_chase_mach = DOGFIGHT_CHASE_MACH;
        ai[BANDIT].far_chase_q_pa = EVADE_FAR_CHASE_Q_PA;

        shooter = MissileShooter{};
        shooter.reset();
        srm_shooters.fill(HeatSeekerShooter{});
        mrm_shooters.fill(MissileShooter{});
        evader = EvasionPilot{};
        evader.reset();
        const AirframeMissileLoadout own_loadout = airframe_missile_loadout(s.own_type);
        const AirframeMissileLoadout bandit_loadout = airframe_missile_loadout(s.bandit_type);

        if (evade) {
            // Hunting a runner: he follows you low and fast, within the airframe's safe envelope.
            ai[BANDIT].hard_deck_agl_m = std::min(s.hard_deck_agl_m, EVADE_HARD_DECK_AGL_M);
            ai[BANDIT].max_chase_mach = EVADE_CHASE_MACH;
            ai[BANDIT].far_chase_q_pa = EVADE_FAR_CHASE_Q_PA;
            world.radars[BANDIT].enable(OWN, profile.radar_acquire_s);
            const bool radar = world.radars[BANDIT].spec.fitted;
            world.stores[BANDIT].mrm = {missile_specs::aim120c(), radar ? profile.missiles : 0};
            shooter.doctrine = radar ? profile.doctrine : LaunchDoctrine::NONE;
            shooter.shoot_shoot = profile.shoot_shoot;
            setup_data.time_limit_s = profile.survive_s;
        } else if (s.loadout != WeaponsLoadout::GUNS_ONLY) {
            // Dogfight: each jet's own heat seekers, and its radar missiles in the full load.
            const bool with_mrm = s.loadout == WeaponsLoadout::FULL_LOADOUT;
            world.stores[OWN].load(own_loadout, with_mrm);
            world.stores[BANDIT].load(bandit_loadout, with_mrm);
            for (int i = 0; i < world.count; ++i) {
                const auto idx = static_cast<size_t>(i);
                if (!ai_controlled[idx]) continue;
                const AiMissileDiscipline d = AiMissileDiscipline::of(i == BANDIT ? bandit_skill : s.skill);
                world.seekers[idx].search_half_deg = d.off_boresight_deg;
                srm_shooters[idx].off_boresight_deg = d.off_boresight_deg;
                srm_shooters[idx].confirm_lock_s = d.confirm_lock_s;
                srm_shooters[idx].spacing_s = d.spacing_s;
                srm_shooters[idx].max_off_tail_deg = d.max_off_tail_deg;
                srm_shooters[idx].strong_off_tail_deg = d.strong_off_tail_deg;
                // Radar missiles need the radar hunting the bandit (an AI
                // pilot's radar is otherwise off in a dogfight).
                const bool rf = world.stores[idx].mrm.count > 0 && world.radars[idx].spec.fitted;
                mrm_shooters[idx].doctrine = rf ? d.mrm_doctrine : LaunchDoctrine::NONE;
                if (rf) world.radars[idx].enable(i == OWN ? BANDIT : OWN, d.radar_acquire_s);
            }
        }
        chaff_at_start_ = world.chaff_load[OWN];
        unlocked_far_s_ = 0.0;
        eject_timer_.fill(0.0);
        range_pilot = RangeTargetPilot{};
        range_pilot.profile = s.range_profile;
        range_pilot.target_altitude_m = alt;
        range_pilot.target_speed_mps = std::min(v, s.own_type == aircraft::AircraftType::A10_THUNDERBOLT ? 150.0 : 230.0);
    }

    [[nodiscard]] bool finished() const noexcept { return outcome != EngagementOutcome::IN_PROGRESS; }
    [[nodiscard]] Aircraft& own() noexcept { return world.aircraft[OWN]; }
    [[nodiscard]] const Aircraft& own() const noexcept { return world.aircraft[OWN]; }
    [[nodiscard]] const Aircraft& bandit() const noexcept { return world.aircraft[BANDIT]; }

    /// @brief One physics step. @p player drives the own jet unless it is AI-flown.
    void step(double dt, const AircraftControls& player) noexcept {
        std::array<AircraftControls, World::kMaxAircraft> controls{};
        controls[OWN] = player;
        for (int i = 0; i < world.count; ++i) {
            const auto idx = static_cast<size_t>(i);
            const Aircraft& self = world.aircraft[idx];
            const Aircraft& foe = world.aircraft[i == OWN ? BANDIT : OWN];
            if (ai_controlled[idx] && i == OWN && evade_mode()) {
                // Both run every step to keep their tracks and timers current.
                // Missiles are defeated off the RWR (and the MAWS, if fitted);
                // a bandit close enough to see is a guns threat and gets basic
                // fighter manoeuvres.
                const RwrStatus rwr = world.rwr(OWN);
                const MissileThreat threat = detect_missile_threat(i);
                remember_maws(i, threat, dt);
                const AircraftControls bfm = ai[idx].update(dt, self, foe);
                const AircraftControls defend = evader.update(dt, self, rwr, threat);
                const double range = (foe.position() - self.position()).norm();
                const bool missile_defence = evader.mode == EvasionPilot::Mode::NOTCH_MISSILE ||
                                             evader.mode == EvasionPilot::Mode::LAST_DITCH;
                const bool visual_fight = range < VISUAL_FIGHT_RANGE_M && !missile_defence;
                controls[idx] = visual_fight ? bfm : defend;
            } else if (ai_controlled[idx]) {
                MissileThreat threat{};
                if (!evade_mode()) {
                    threat = detect_missile_threat(i);
                    remember_maws(i, threat, dt);
                }
                controls[idx] = ai[idx].update(dt, self, foe, threat);
                if (evade_mode() && idx == BANDIT) {
                    controls[idx].pickle = false;
                    controls[idx].dispense = false;
                    controls[idx].dispense_flare = false;
                }
            } else if (i == BANDIT && setup_data.geometry == StartGeometry::GUNNERY_RANGE) {
                controls[idx] = range_pilot.update(dt, self);
            }
        }
        // AI pilots in a jet that is going down get out.
        for (int i = 0; i < world.count; ++i) {
            const auto idx = static_cast<size_t>(i);
            if (ai_controlled[idx] && should_eject(i, dt)) controls[idx].eject = true;
        }
        // Once the fight is decided the survivors hold fire.
        if (finished()) {
            for (auto& c : controls) c.trigger = false;
        } else if (evade_mode()) {
            (void)shooter.update(dt, world, BANDIT, OWN);
        } else {
            employ_missiles(dt);
        }

        world.step(dt, controls);
        tally(dt);
        judge();
    }

    [[nodiscard]] bool evade_mode() const noexcept { return setup_data.mode == EngagementMode::EVADE; }

    /// Range inside which a pilot sees a missile whose motor is burning (its smoke trail) [m].
    static constexpr double VISUAL_MISSILE_SMOKE_M = 5000.0;
    /// Range inside which a pilot sees a coasting missile [m].
    static constexpr double VISUAL_MISSILE_M = 1500.0;

    /**
     * @brief What AI pilot @p i knows about missiles coming at him: the one
     * place an AI's missile threat picture comes from. Only his own sensors:
     *
     *  - His RWR: a radar missile it hears (the launcher's datalink, or its
     *    own seeker once pointed at him).
     *  - His eyes: any missile whose motor is still burning (the smoke trail)
     *    inside VISUAL_MISSILE_SMOKE_M, or any inside VISUAL_MISSILE_M. A heat
     *    seeker that has burnt out further away arrives unseen.
     *  - His own jet's MAWS (world.maws[i], if one is fitted): a missile
     *    closing on him, its line of sight and time to go (a boosting
     *    missile's included), and the launch call. A passive set loses the
     *    missile when its motor burns out; he keeps defending against where
     *    it was until its last time to go, stretched for the coast, has run out.
     *
     * The nearest missile heard or seen is the threat, timed by the MAWS when
     * it has one too. One that only the MAWS has, silent on the RWR, is
     * treated as a heat seeker.
     */
    [[nodiscard]] MissileThreat detect_missile_threat(int i) const noexcept {
        MissileThreat threat;
        const auto idx = static_cast<size_t>(i);
        const Aircraft& self = world.aircraft[idx];
        for (const Missile& m : world.missiles) {
            if (!m.active || m.target != i || m.shooter == i) continue;
            const math::Vector3 r = m.body.pos - self.position();
            const double range = r.norm();
            const bool heard = world.missile_warning(m);
            const bool seen = range < VISUAL_MISSILE_M ||
                              (range < VISUAL_MISSILE_SMOKE_M && m.body.motor_burning(m.spec));
            if (!(heard || seen) || (threat.inbound && range >= threat.range_m)) continue;
            threat.inbound = true;
            threat.radar_guided = heard;
            threat.pos = m.body.pos;
            threat.range_m = range;
            threat.closure_mps = range > 1.0 ? (self.velocity() - m.body.vel).dot(r / range) : 0.0;
        }

        const MissileWarningSystem& mws = world.maws[idx];
        if (!mws.spec.fitted()) return threat;
        const MawsStatus& w = mws.status;
        const MawsMemory& mem = maws_memory_[idx];
        if (!w.threat && mem.hold_s <= 0.0) return threat;
        const math::Vector3 los = w.threat ? w.los_ned : mem.los_ned;
        const double closing = w.threat ? w.closing_mps : mem.closing_mps;
        const double tgo = w.threat ? w.time_to_go_s : mem.hold_s;
        threat.maws = true;
        threat.held = !w.threat;
        threat.launch = w.launch;
        // A live MAWS track times the missile best (a boosting one too); a
        // remembered one only when nothing better has it.
        if (w.threat || !threat.inbound) threat.tgo_s = tgo;
        if (!threat.inbound) {
            // Only the MAWS has it: on its line of sight, at the range it
            // measures (pulse-doppler) or the one its closure and time to go give.
            const double range = w.threat && w.range_valid ? w.range_m : closing * tgo;
            threat.inbound = true;
            threat.pos = self.position() + los * std::max(range, 1.0);
            threat.range_m = range;
            threat.closure_mps = closing;
        }
        threat.heat_seeker = !threat.radar_guided && !mem.rwr_heard;
        return threat;
    }

    /// @brief What the player's RWR shows.
    [[nodiscard]] RwrStatus own_rwr() const noexcept { return world.rwr(OWN); }

    /// @brief Seconds since the fight ended (0 while in progress).
    [[nodiscard]] double time_since_end() const noexcept {
        return end_time < 0.0 ? 0.0 : world.time - end_time;
    }

    /// Evade: range at which a bandit who has lost radar contact loses you for good.
    static constexpr double ESCAPE_RANGE_M = 30000.0;
    static constexpr double ESCAPE_HOLD_S = 10.0;
    static constexpr double EVADE_HARD_DECK_AGL_M = 500.0;
    static constexpr double EVADE_CHASE_MACH = 1.6;
    static constexpr double DOGFIGHT_CHASE_MACH = 1.4;
    static constexpr double EVADE_FAR_CHASE_Q_PA = 95000.0; ///< ~390 m/s at sea level, ~510 m/s at 5 km
    static constexpr double VISUAL_FIGHT_RANGE_M = 4000.0;

    /// Time an AI pilot takes to decide to eject from a doomed jet [s].
    static constexpr double EJECT_DECISION_S = 1.5;

private:
    /// @brief What an AI pilot remembers of his own MAWS's latest threat.
    struct MawsMemory {
        double hold_s{0.0};      ///< Defence still held after a passive set lost the missile [s]
        math::Vector3 los_ned{}; ///< Its last line of sight
        double closing_mps{0.0};
        bool rwr_heard{false};   ///< The RWR heard a missile while this one was tracked: a radar missile
    };
    /// A passive MAWS's last time to go, stretched for the coast after
    /// burnout (the missile slows), plus a margin, is how long the defence is held.
    static constexpr double MAWS_HOLD_STRETCH = 1.25;
    static constexpr double MAWS_HOLD_MARGIN_S = 0.5;

    std::array<MawsMemory, World::kMaxAircraft> maws_memory_{};
    int events_read_{0};
    int chaff_at_start_{0};
    double unlocked_far_s_{0.0};
    std::array<double, World::kMaxAircraft> eject_timer_{};

    /// @brief Keep AI pilot @p i's memory of his own MAWS's threat: a passive
    /// set's track is held for its time to go once the motor burns out.
    void remember_maws(int i, const MissileThreat& threat, double dt) noexcept {
        const auto idx = static_cast<size_t>(i);
        const MissileWarningSystem& mws = world.maws[idx];
        MawsMemory& mem = maws_memory_[idx];
        if (!mws.spec.fitted()) {
            mem = MawsMemory{};
            return;
        }
        const MawsStatus& w = mws.status;
        if (w.threat) {
            mem.hold_s = mws.spec.needs_motor() ? MAWS_HOLD_STRETCH * w.time_to_go_s + MAWS_HOLD_MARGIN_S : 0.0;
            mem.los_ned = w.los_ned;
            mem.closing_mps = w.closing_mps;
        } else {
            mem.hold_s = std::max(0.0, mem.hold_s - dt);
        }
        if (threat.radar_guided) mem.rwr_heard = true;
        else if (!w.threat && mem.hold_s <= 0.0) mem.rwr_heard = false;
    }

    /// @brief Dogfight: each AI pilot's missile shots, heat seekers first; one
    /// missile of his in the air at a time.
    void employ_missiles(double dt) noexcept {
        for (int i = 0; i < world.count; ++i) {
            const auto idx = static_cast<size_t>(i);
            if (!ai_controlled[idx]) continue;
            const int foe = i == OWN ? BANDIT : OWN;
            if (!srm_shooters[idx].update(dt, world, i, foe)) (void)mrm_shooters[idx].update(dt, world, i, foe);
        }
    }

    /// @brief A conscious pilot in a jet that is destroyed, or that has broken
    /// itself by over-G, pulls the handle once he has taken it in.
    bool should_eject(int i, double dt) noexcept {
        const auto idx = static_cast<size_t>(i);
        const Aircraft& a = world.aircraft[idx];
        const bool doomed = a.damage.destroyed || a.damage.structural_failure;
        if (!doomed || a.crashed || a.damage.pilot_incapacitated || world.seats[idx].initiated()) {
            eject_timer_[idx] = 0.0;
            return false;
        }
        eject_timer_[idx] += dt;
        return eject_timer_[idx] >= EJECT_DECISION_S;
    }

    /**
     * @brief Evade start: you heading north, the bandit behind you, nose on,
     * a little off your tail (either side), above you and faster.
     */
    static void evade_start(fdm::FlightState& own, fdm::FlightState& bandit, const EngagementSetup& s,
                            const EvadeProfile& p) noexcept {
        DamageRng rng{s.seed * 0x9E3779B1u + 0x7F4A7C15u};
        const double side = rng.uniform() < 0.5 ? -1.0 : 1.0;
        const double off = p.off_tail_deg * (0.5 + 0.5 * rng.uniform()) * (M_PI / 180.0);
        own = make_state({0.0, 0.0, -s.altitude_m}, 0.0, s.speed_mps);
        const math::Vector3 pos{-p.start_range_m * std::cos(off), side * p.start_range_m * std::sin(off),
                                -(s.altitude_m + p.altitude_advantage_m)};
        const double heading = std::atan2(-pos.y, -pos.x);
        bandit = make_state(pos, heading, s.speed_mps + p.speed_advantage_mps);
        // Nose on you in pitch too, so you start inside his radar.
        const double pitch = std::atan2(-p.altitude_advantage_m, p.start_range_m);
        bandit.q_att = math::Quaternion::from_euler(0.0, pitch, heading);
    }

    static fdm::FlightState make_state(const math::Vector3& pos, double heading, double v) noexcept {
        fdm::FlightState s{};
        s.pos_ned = pos;
        s.vel_b = math::Vector3(v, 0.0, 0.0);
        s.q_att = math::Quaternion::from_euler(0.0, 0.0, heading);
        return s;
    }

    /// @brief Attacker 3,000 ft (914 m) from the target, 30 deg off its tail,
    /// 1,000 ft (305 m) above; the target heading north, the attacker parallel.
    static void perch(fdm::FlightState& attacker, fdm::FlightState& target, double alt, double v) noexcept {
        constexpr double RANGE = 914.0;
        constexpr double OFF_TAIL = 30.0 * M_PI / 180.0;
        target = make_state({0.0, 0.0, -alt}, 0.0, v);
        const math::Vector3 offset{-RANGE * std::cos(OFF_TAIL), RANGE * std::sin(OFF_TAIL), -305.0};
        attacker = make_state(target.pos_ned + offset, 0.0, v);
    }

    /// @brief Bring the engine to its running setting before the fight, so
    /// neither jet starts with a spooled-down core. Fuel burnt is refilled.
    static void prespool(Aircraft& a, double lever) noexcept {
        const auto air = environment::Atmosphere1976::compute(a.state.altitude(), a.state.airspeed());
        // 15 s: long enough for the core to spool up and the afterburner to light.
        for (int i = 0; i < 3000; ++i) (void)a.engine.update(lever, air, Aircraft::DT);
        a.engine.fuel_kg = aircraft::AircraftConfig::get(a.type).mass.internal_fuel_capacity_kg;
        a.refresh_mass();
    }

    void tally(double dt) noexcept {
        const Aircraft& me = world.aircraft[OWN];
        if (!finished()) {
            stats.duration_s += dt;
            stats.peak_g = std::max(stats.peak_g, me.imu.Nz);
            if (me.imu.Nz > 7.0) stats.time_above_7g_s += dt;
            stats.min_agl_m = std::min(stats.min_agl_m, environment::GroundCollision::get_agl(me.state));
        }
        stats.rounds_fired = me.gun.rounds_fired;
        stats.chaff_used = chaff_at_start_ - world.chaff_load[OWN];
        if (!finished() && world.rwr(OWN).level >= RwrStatus::Level::LOCK) stats.time_locked_s += dt;
        for (; events_read_ < world.event_count(); ++events_read_) {
            const CombatEvent& e = world.event(events_read_);
            const bool at_me = e.victim == OWN;
            switch (e.kind) {
                case CombatEvent::Kind::HIT:
                    if (e.shooter == OWN) ++stats.hits_scored;
                    if (at_me) ++stats.hits_taken;
                    break;
                case CombatEvent::Kind::MISSILE_LAUNCH:     stats.missiles_at_you += at_me; break;
                case CombatEvent::Kind::MISSILE_DEFEATED:   stats.missiles_defeated += at_me; break;
                case CombatEvent::Kind::MISSILE_DETONATION: stats.missile_detonations += at_me; break;
                case CombatEvent::Kind::LOCK_BROKEN_CHAFF:  stats.locks_broken_by_chaff += at_me; break;
                default: break;
            }
        }
    }

    void judge() noexcept {
        if (finished()) return;
        const bool own_out = !world.aircraft[OWN].alive();
        const bool bandit_out = !world.aircraft[BANDIT].alive();
        if (own_out && bandit_out) outcome = EngagementOutcome::MUTUAL_KILL;
        else if (bandit_out) outcome = EngagementOutcome::WIN;
        else if (own_out) outcome = EngagementOutcome::LOSS;
        else if (evade_mode()) judge_evade();
        else if (world.time >= setup_data.time_limit_s) outcome = EngagementOutcome::TIMEOUT;
        if (finished()) end_time = world.time;
    }

    /// @brief Evade ends when you outlast his fuel, when he is out of weapons
    /// (Winchester) or when he has lost you: no lock, nothing in the air and you
    /// beyond his radar's reach for long enough that he gives up.
    void judge_evade() noexcept {
        constexpr double DT = Aircraft::DT;
        const auto b = static_cast<size_t>(BANDIT);
        const bool nothing_inbound = world.missiles_in_flight(BANDIT, OWN) == 0;
        const bool winchester = world.stores[b].missiles() == 0 && world.aircraft[b].gun.ammo == 0 && nothing_inbound;
        const double range = (world.aircraft[b].position() - world.aircraft[OWN].position()).norm();
        const bool lost = !world.radars[b].locked() && nothing_inbound && range > ESCAPE_RANGE_M;
        unlocked_far_s_ = lost ? unlocked_far_s_ + DT : 0.0;
        if (world.time >= setup_data.time_limit_s || winchester) outcome = EngagementOutcome::SURVIVED;
        else if (unlocked_far_s_ >= ESCAPE_HOLD_S) outcome = EngagementOutcome::ESCAPED;
    }
};

} // namespace fastjet::sim
