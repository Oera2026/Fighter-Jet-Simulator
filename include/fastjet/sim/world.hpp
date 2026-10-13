#pragma once

#include "fastjet/environment/ground_collision.hpp"
#include "fastjet/sim/air_combat_geometry.hpp"
#include "fastjet/sim/aircraft.hpp"
#include "fastjet/sim/combat_specs.hpp"
#include "fastjet/sim/damage_model.hpp"
#include "fastjet/sim/ejection_seat.hpp"
#include "fastjet/sim/gun.hpp"
#include "fastjet/sim/missile.hpp"
#include "fastjet/sim/missile_warning.hpp"
#include "fastjet/sim/radar.hpp"
#include <array>
#include <cmath>
#include <cstdint>

namespace fastjet::sim {

/// @brief Something that happened in the fight, for scoring, HUD cues and the debrief.
struct CombatEvent {
    enum class Kind : uint8_t {
        HIT, KILL, GROUND_IMPACT, MIDAIR,
        MISSILE_LAUNCH,     ///< shooter fired at victim
        MISSILE_DETONATION, ///< proximity fuze fired near victim
        MISSILE_DEFEATED,   ///< ran out of energy, time or track without fuzing
        LOCK_BROKEN_CHAFF,  ///< a chaff bundle stole a radar or seeker track on victim
        LOCK_BROKEN_FLARE,  ///< a flare stole an infrared heat-seeker track on victim
        STRUCTURAL_FAILURE, ///< victim pulled its own airframe apart (over-G)
        EJECTION,           ///< victim's pilot pulled the handle
    };
    Kind kind{Kind::HIT};
    int victim{-1};
    int shooter{-1};   ///< Credited aircraft (-1 if none)
    HitZone zone{HitZone::FUSELAGE};
    double time{0.0};
    double miss_m{0.0};  ///< Missile events: closest approach to the victim
};

/**
 * @brief Every jet, round and event in one sky, stepped deterministically.
 *
 * Fixed capacity and no heap use in step(). Order within a step is fixed:
 * aircraft (by index) -> guns -> rounds and hit tests -> countermeasures ->
 * radars -> missiles and fuzes -> mid-airs -> ground impacts, and all
 * randomness (dispersion, damage, chaff) comes from one seeded generator,
 * so an engagement replays exactly from its inputs.
 */
class World {
public:
    static constexpr int kMaxAircraft = 8;
    static constexpr int kMaxRounds = 4096;
    static constexpr int kMaxEvents = 256;
    static constexpr int kMaxMissiles = 16;
    static constexpr int kMaxChaff = 128;
    static constexpr int CHAFF_PER_PROGRAM = 2;
    static constexpr double CHAFF_INTERVAL_S = 0.15;
    static constexpr int kMaxFlares = 128;
    static constexpr int FLARES_PER_PROGRAM = 2;
    static constexpr double FLARE_INTERVAL_S = 0.20;
    static constexpr int DEFAULT_FLARES = 60;
    /// A jet that hits the ground this long after being shot is credited to the shooter.
    static constexpr double KILL_CREDIT_WINDOW_S = 30.0;

    std::array<Aircraft, kMaxAircraft> aircraft{};
    std::array<Projectile, kMaxRounds> rounds{};
    std::array<Missile, kMaxMissiles> missiles{};
    std::array<ChaffCloud, kMaxChaff> chaff{};
    std::array<FlareDecoy, kMaxFlares> flares{};
    std::array<FireControlRadar, kMaxAircraft> radars{};
    std::array<int, kMaxAircraft> missile_load{}; ///< Missiles left on the rails
    std::array<MissileSpec, kMaxAircraft> missile_spec{};
    std::array<int, kMaxAircraft> chaff_load{};   ///< Chaff cartridges left
    std::array<int, kMaxAircraft> flare_load{};   ///< Flare cartridges left
    std::array<EjectionSeat, kMaxAircraft> seats{}; ///< Each jet's escape system
    std::array<MissileWarningSystem, kMaxAircraft> maws{}; ///< Each jet's missile approach warner (read maws[i].status)
    int count{0};
    double time{0.0};
    bool dispersion{true};

    World() noexcept = default;

    void clear(uint32_t seed = 1) noexcept {
        count = 0;
        time = 0.0;
        for (auto& r : rounds) r.active = false;
        for (auto& m : missiles) m.active = false;
        for (auto& c : chaff) c.active = false;
        for (auto& f : flares) f.active = false;
        next_round_ = 0;
        next_chaff_ = 0;
        next_flare_ = 0;
        missile_load.fill(0);
        chaff_load.fill(0);
        flare_load.fill(0);
        program_left_.fill(0);
        program_timer_.fill(0.0);
        prev_dispense_.fill(false);
        flare_program_left_.fill(0);
        flare_timer_.fill(0.0);
        prev_flare_.fill(false);
        prev_pickle_.fill(false);
        for (auto& s : seats) s.reset();
        event_count_ = 0;
        rng_.state = seed ? seed : 1u;
        last_hit_by_.fill(-1);
        last_hit_time_.fill(-1e9);
        was_crashed_.fill(false);
        was_alive_.fill(true);
    }

    /// @brief Add a jet at @p start. Returns its index, or -1 when full.
    int add(aircraft::AircraftType type, const fdm::FlightState& start) noexcept {
        if (count >= kMaxAircraft) return -1;
        const int i = count++;
        Aircraft& a = aircraft[static_cast<size_t>(i)];
        if (a.type != type) a.configure(type);
        a.reset(start); // full tanks, systems reset, undamaged
        // The first jet keeps the default gust stream (identical to the
        // single-aircraft viewer); the others get their own.
        a.seed_turbulence(i == 0 ? 0u : 0x2545F491u * static_cast<uint32_t>(i + 1));
        was_crashed_[static_cast<size_t>(i)] = false;
        was_alive_[static_cast<size_t>(i)] = true;
        radars[static_cast<size_t>(i)].configure(type);
        missile_load[static_cast<size_t>(i)] = 0;
        chaff_load[static_cast<size_t>(i)] = DEFAULT_CHAFF;
        flare_load[static_cast<size_t>(i)] = DEFAULT_FLARES;
        seats[static_cast<size_t>(i)].reset();
        return i;
    }

    /// @brief One fixed step of the whole world.
    void step(double dt, const std::array<AircraftControls, kMaxAircraft>& controls) noexcept {
        for (int i = 0; i < count; ++i) {
            aircraft[static_cast<size_t>(i)].step(dt, controls[static_cast<size_t>(i)]);
        }
        time += dt;
        fire_guns(dt, controls);
        step_rounds(dt);
        step_electronic_combat(dt, controls);
        check_midairs();
        check_ground_impacts();
        check_mission_kills();
    }

    // ---------------------------------------------------------------------
    // Events
    // ---------------------------------------------------------------------

    [[nodiscard]] int event_count() const noexcept { return event_count_; }
    [[nodiscard]] const CombatEvent& event(int i) const noexcept { return events_[static_cast<size_t>(i)]; }
    /// @brief Forget consumed events (the consumer keeps its own read cursor otherwise).
    void clear_events() noexcept { event_count_ = 0; }

    [[nodiscard]] int active_rounds() const noexcept {
        int n = 0;
        for (const auto& r : rounds) n += r.active ? 1 : 0;
        return n;
    }

    // ---------------------------------------------------------------------
    // Missiles, countermeasures and sensors
    // ---------------------------------------------------------------------

    static constexpr int DEFAULT_CHAFF = 60;

    /// @brief Fire a missile from @p shooter at @p target. It needs a missile on
    /// the rails and a radar lock on the target to initialise its guidance.
    /// Returns the missile's slot, or -1.
    /// @brief Fire a missile from @p shooter at @p target. Active radar missiles
    /// need a radar lock; infrared heat-seekers need the target within their
    /// seeker gimbal FOV and range. Returns the missile's slot, or -1.
    int launch_missile(int shooter, int target) noexcept {
        if (shooter < 0 || shooter >= count || target < 0 || target >= count || shooter == target) return -1;
        const auto si = static_cast<size_t>(shooter);
        const auto ti = static_cast<size_t>(target);
        const Aircraft& a = aircraft[si];
        const Aircraft& t = aircraft[ti];
        const MissileSpec& spec = missile_spec[si];
        if (missile_load[si] <= 0 || !a.alive() || !t.alive()) return -1;

        const FireControlRadar& radar = radars[si];
        if (spec.infrared()) {
            const math::Vector3 nose = a.state.q_att.rotate_body_to_ned(math::Vector3(1.0, 0.0, 0.0));
            const math::Vector3 los = t.position() - a.position();
            const double range = los.norm();
            if (range > spec.seeker_range_m) return -1;
            const double ata_deg = AirCombatGeometry::angle_between(nose, los) * 180.0 / M_PI;
            if (ata_deg > spec.seeker_gimbal_deg) return -1;
        } else {
            if (!radar.locked() || radar.target != target) return -1;
        }

        int slot = -1;
        for (int k = 0; k < kMaxMissiles; ++k) {
            if (!missiles[static_cast<size_t>(k)].active) { slot = k; break; }
        }
        if (slot < 0) return -1;
        --missile_load[si];
        Missile& m = missiles[static_cast<size_t>(slot)];
        m = Missile{};
        m.spec = spec;
        m.shooter = shooter;
        m.target = target;
        m.active = true;
        // Off the rail or ejector below the fuselage, at the launcher's velocity.
        m.body.pos = a.position() + a.state.q_att.rotate_body_to_ned(math::Vector3(0.0, 0.0, 1.2));
        m.body.vel = a.velocity();
        if (spec.infrared()) {
            m.est_pos = t.position();
            m.est_vel = t.velocity();
            m.phase = Missile::Phase::ACTIVE_TRACK;
        } else {
            m.est_pos = radar.track_pos;
            m.est_vel = radar.track_vel;
            m.phase = radar.track_fresh() ? Missile::Phase::DATALINK : Missile::Phase::INERTIAL;
        }
        push({CombatEvent::Kind::MISSILE_LAUNCH, target, shooter, HitZone::FUSELAGE, time, 0.0});
        return slot;
    }

    [[nodiscard]] int missiles_in_flight(int shooter = -1, int target = -1) const noexcept {
        int n = 0;
        for (const auto& m : missiles) {
            if (!m.active) continue;
            if (shooter >= 0 && m.shooter != shooter) continue;
            if (target >= 0 && m.target != target) continue;
            ++n;
        }
        return n;
    }

    /// @brief What @p i's radar warning receiver shows.
    [[nodiscard]] RwrStatus rwr(int i) const noexcept {
        RwrStatus s;
        if (i < 0 || i >= count) return s;
        const Aircraft& own = aircraft[static_cast<size_t>(i)];
        auto raise = [&s](RwrStatus::Level l) { if (static_cast<int>(l) > static_cast<int>(s.level)) s.level = l; };
        for (int j = 0; j < count; ++j) {
            if (j == i) continue;
            const FireControlRadar& r = radars[static_cast<size_t>(j)];
            const Aircraft& e = aircraft[static_cast<size_t>(j)];
            if (r.mode == FireControlRadar::Mode::OFF || r.target != i || !e.alive()) continue;
            if (!r.locked() && !r.painting()) continue;
            raise(r.locked() ? RwrStatus::Level::LOCK : RwrStatus::Level::SEARCH);
            s.emitter_valid = true;
            s.emitter_bearing = RwrStatus::relative_bearing(own.state, e.position());
            s.emitter_symbol = r.spec.rwr_symbol;
        }
        double nearest = 1e18;
        for (const auto& m : missiles) {
            if (!m.active || m.target != i || !missile_warning(m)) continue;
            raise(RwrStatus::Level::LAUNCH);
            const double d = (m.body.pos - own.position()).norm();
            if (d < nearest) {
                nearest = d;
                s.missile_valid = true;
                s.missile_bearing = RwrStatus::relative_bearing(own.state, m.body.pos);
                s.missile_range_m = d;
                s.missile_seeker_active = m.seeker_active();
            }
        }
        return s;
    }

    // ---------------------------------------------------------------------
    // Hit geometry (public for tests)
    // ---------------------------------------------------------------------

    /// @brief Test a round's swept path against one airframe.
    ///
    /// The sweep is done in the target's frame (round displacement minus the
    /// target's), so a head-on pass at 1,400 m/s closure, 7 m per step, cannot
    /// tunnel through the jet. Returns true with the hit point in body axes.
    [[nodiscard]] static bool sweep_hit(const math::Vector3& round_p0, const math::Vector3& round_p1,
                                        const fdm::FlightState& target_prev, const fdm::FlightState& target,
                                        const AirframeCombatSpec& spec, math::Vector3& hit_b) noexcept {
        const math::Vector3 r0 = target.q_att.rotate_ned_to_body(round_p0 - target_prev.pos_ned);
        const math::Vector3 r1 = target.q_att.rotate_ned_to_body(round_p1 - target.pos_ned);

        // Broad phase: bounding sphere against the swept segment.
        const double bound = 0.5 * std::max(spec.length_m, spec.span_m) + spec.fuselage_radius_m;
        double s = 0.0, t = 0.0;
        const math::Vector3 o{0.0, 0.0, 0.0};
        if (segment_segment_distance(r0, r1, o, o, s, t) > bound) return false;

        const double half_l = 0.5 * spec.length_m - spec.fuselage_radius_m;
        const math::Vector3 fus_a{half_l, 0.0, 0.0};
        const math::Vector3 fus_b{-half_l, 0.0, 0.0};
        const double wing_x = -0.05 * spec.length_m;
        const double half_b = 0.5 * spec.span_m - spec.wing_radius_m;
        const math::Vector3 wing_a{wing_x, -half_b, 0.0};
        const math::Vector3 wing_b{wing_x, half_b, 0.0};

        double best_s = 2.0;
        double sf = 0.0, tf = 0.0;
        if (segment_segment_distance(r0, r1, fus_a, fus_b, sf, tf) <= spec.fuselage_radius_m) best_s = sf;
        double sw = 0.0, tw = 0.0;
        if (segment_segment_distance(r0, r1, wing_a, wing_b, sw, tw) <= spec.wing_radius_m && sw < best_s) {
            best_s = sw;
        }
        if (best_s > 1.0) return false;
        hit_b = r0 + (r1 - r0) * best_s;
        return true;
    }

private:
    std::array<CombatEvent, kMaxEvents> events_{};
    int event_count_{0};
    int next_round_{0};
    DamageRng rng_{};
    std::array<int, kMaxAircraft> last_hit_by_{};
    std::array<double, kMaxAircraft> last_hit_time_{};
    std::array<bool, kMaxAircraft> was_crashed_{};
    std::array<bool, kMaxAircraft> was_alive_{};
    int next_chaff_{0};
    std::array<int, kMaxAircraft> program_left_{};
    std::array<double, kMaxAircraft> program_timer_{};
    std::array<bool, kMaxAircraft> prev_dispense_{};
    int next_flare_{0};
    std::array<int, kMaxAircraft> flare_program_left_{};
    std::array<double, kMaxAircraft> flare_timer_{};
    std::array<bool, kMaxAircraft> prev_flare_{};
    std::array<bool, kMaxAircraft> prev_pickle_{};

    /// @brief Can the target's RWR hear this missile? The launcher's uplink
    /// while it holds lock, or the missile's own seeker once it is pointed at
    /// the jet. An inertial missile, or one searching somewhere else, is silent.
    [[nodiscard]] bool missile_warning(const Missile& m) const noexcept {
        if (m.spec.infrared()) return false; // Heat seekers are passive and silent on the RWR
        switch (m.phase) {
            case Missile::Phase::DATALINK:
                return m.shooter >= 0 && radars[static_cast<size_t>(m.shooter)].track_fresh();
            case Missile::Phase::ACTIVE_TRACK: return true;
            case Missile::Phase::ACTIVE_SEARCH: {
                if (m.decoy_timer > 0.0) return false;
                const Aircraft& t = aircraft[static_cast<size_t>(m.target)];
                return AirCombatGeometry::angle_between(m.est_pos - m.body.pos, t.position() - m.body.pos) <
                       2.0 * m.spec.seeker_basket_deg * M_PI / 180.0;
            }
            default: return false;
        }
    }

    /// @brief Countermeasures, radars and missiles. Kept out of line so that
    /// step() compiles the aircraft pipeline exactly as the single-jet
    /// reference loop does (same inlining, same FMA contraction), which
    /// test_aircraft_entity checks bit for bit.
    [[gnu::noinline]] void step_electronic_combat(double dt,
                                                  const std::array<AircraftControls, kMaxAircraft>& controls) noexcept {
        dispense_countermeasures(dt, controls);
        step_escape_systems(dt, controls);
        step_radars(dt);
        fire_missiles(controls);
        step_missiles(dt);
        step_chaff(dt);
        step_flares(dt);
        for (int i = 0; i < count; ++i) maws[static_cast<size_t>(i)].update(dt, i, aircraft[static_cast<size_t>(i)], missiles);
    }

    void fire_missiles(const std::array<AircraftControls, kMaxAircraft>& controls) noexcept {
        for (int i = 0; i < count; ++i) {
            const auto idx = static_cast<size_t>(i);
            const bool pressed = controls[idx].pickle && !prev_pickle_[idx];
            prev_pickle_[idx] = controls[idx].pickle;
            if (!pressed || missile_load[idx] <= 0 || !aircraft[idx].alive()) continue;

            int tgt = -1;
            const MissileSpec& spec = missile_spec[idx];
            if (spec.infrared()) {
                if (radars[idx].locked() && radars[idx].target >= 0 && radars[idx].target != i) {
                    tgt = radars[idx].target;
                } else {
                    double best_score = 1e9;
                    const math::Vector3 nose = aircraft[idx].state.q_att.rotate_body_to_ned(math::Vector3(1.0, 0.0, 0.0));
                    for (int j = 0; j < count; ++j) {
                        if (j == i || !aircraft[static_cast<size_t>(j)].alive()) continue;
                        const math::Vector3 los = aircraft[static_cast<size_t>(j)].position() - aircraft[idx].position();
                        const double r = los.norm();
                        if (r > spec.seeker_range_m) continue;
                        const double ata_deg = AirCombatGeometry::angle_between(nose, los) * 180.0 / M_PI;
                        if (ata_deg <= spec.seeker_gimbal_deg && r < best_score) {
                            best_score = r;
                            tgt = j;
                        }
                    }
                }
            } else {
                if (radars[idx].locked() && radars[idx].target >= 0 && radars[idx].target != i) {
                    tgt = radars[idx].target;
                }
            }
            if (tgt >= 0) {
                launch_missile(i, tgt);
            }
        }
    }

    void dispense_countermeasures(double dt, const std::array<AircraftControls, kMaxAircraft>& controls) noexcept {
        // Chaff dispensing
        for (int i = 0; i < count; ++i) {
            const auto idx = static_cast<size_t>(i);
            const Aircraft& a = aircraft[idx];
            const bool pressed = controls[idx].dispense && !prev_dispense_[idx];
            prev_dispense_[idx] = controls[idx].dispense;
            if (pressed && a.alive()) program_left_[idx] += CHAFF_PER_PROGRAM;
            if (program_left_[idx] <= 0) continue;
            program_timer_[idx] -= dt;
            if (program_timer_[idx] > 0.0) continue;
            if (chaff_load[idx] <= 0 || a.crashed) {
                program_left_[idx] = 0;
                continue;
            }
            --program_left_[idx];
            --chaff_load[idx];
            program_timer_[idx] = CHAFF_INTERVAL_S;
            ChaffCloud& c = chaff[static_cast<size_t>(next_chaff_)];
            next_chaff_ = (next_chaff_ + 1) % kMaxChaff;
            c = ChaffCloud{};
            c.pos = a.position() + a.state.q_att.rotate_body_to_ned(math::Vector3(-5.0, 0.0, 0.8));
            c.vel = a.velocity() + a.state.q_att.rotate_body_to_ned(math::Vector3(0.0, 0.0, 15.0));
            c.owner = i;
            c.active = true;
        }

        // Flare dispensing
        for (int i = 0; i < count; ++i) {
            const auto idx = static_cast<size_t>(i);
            const Aircraft& a = aircraft[idx];
            const bool pressed = controls[idx].dispense_flare && !prev_flare_[idx];
            prev_flare_[idx] = controls[idx].dispense_flare;
            if (pressed && a.alive()) flare_program_left_[idx] += FLARES_PER_PROGRAM;
            if (flare_program_left_[idx] <= 0) continue;
            flare_timer_[idx] -= dt;
            if (flare_timer_[idx] > 0.0) continue;
            if (flare_load[idx] <= 0 || a.crashed) {
                flare_program_left_[idx] = 0;
                continue;
            }
            --flare_program_left_[idx];
            --flare_load[idx];
            flare_timer_[idx] = FLARE_INTERVAL_S;
            FlareDecoy& f = flares[static_cast<size_t>(next_flare_)];
            next_flare_ = (next_flare_ + 1) % kMaxFlares;
            f = FlareDecoy{};
            f.pos = a.position() + a.state.q_att.rotate_body_to_ned(math::Vector3(-4.0, 0.0, 1.0));
            f.vel = a.velocity() + a.state.q_att.rotate_body_to_ned(math::Vector3(-10.0, 0.0, 25.0));
            f.owner = i;
            f.active = true;
        }
    }

    /// @brief Ejection handles and the seats in flight. A conscious pilot can
    /// eject from a jet that is still in the air, however badly damaged.
    void step_escape_systems(double dt, const std::array<AircraftControls, kMaxAircraft>& controls) noexcept {
        for (int i = 0; i < count; ++i) {
            const auto idx = static_cast<size_t>(i);
            Aircraft& a = aircraft[idx];
            EjectionSeat& seat = seats[idx];
            if (controls[idx].eject && !seat.initiated() && !a.crashed && !a.damage.pilot_incapacitated) {
                seat.initiate(a.type, a.state);
                a.damage.pilot_ejected = true;
                push({CombatEvent::Kind::EJECTION, i, credited_shooter(i), HitZone::COCKPIT, time, 0.0});
            }
            seat.step(dt, a.state);
        }
    }

    void step_radars(double dt) noexcept {
        for (int i = 0; i < count; ++i) {
            FireControlRadar& r = radars[static_cast<size_t>(i)];
            if (r.mode == FireControlRadar::Mode::OFF || r.target < 0 || r.target >= count) continue;
            r.update(dt, aircraft[static_cast<size_t>(i)], aircraft[static_cast<size_t>(r.target)]);
        }
    }

    /// @brief A bundle blooming in a tracker's gates can steal its track. Each
    /// bundle gets one roll per tracker when it blooms.
    void step_chaff(double dt) noexcept {
        for (auto& c : chaff) {
            if (!c.active) continue;
            c.step(dt);
            if (c.bloomed || c.age < ChaffCloud::BLOOM_S || c.owner < 0) continue;
            c.bloomed = true;
            const Aircraft& victim = aircraft[static_cast<size_t>(c.owner)];
            for (int j = 0; j < count; ++j) {
                FireControlRadar& r = radars[static_cast<size_t>(j)];
                if (j == c.owner || !r.locked() || r.target != c.owner) continue;
                const Aircraft& a = aircraft[static_cast<size_t>(j)];
                if (!chaff_in_gates(a.position(), victim.position(), victim.velocity(), c, r.limits().beam_half_rad)) {
                    continue;
                }
                if (rng_.uniform() < r.spec.chaff_seduction) {
                    r.break_lock(1.0);
                    push({CombatEvent::Kind::LOCK_BROKEN_CHAFF, c.owner, j, HitZone::FUSELAGE, time, 0.0});
                }
            }
            for (auto& m : missiles) {
                if (!m.active || m.target != c.owner || m.phase != Missile::Phase::ACTIVE_TRACK || m.spec.infrared()) continue;
                if (!chaff_in_gates(m.body.pos, victim.position(), victim.velocity(), c,
                                     m.seeker_limits().beam_half_rad)) {
                    continue;
                }
                if (rng_.uniform() < m.spec.chaff_seduction) {
                    m.seduce(c);
                    push({CombatEvent::Kind::LOCK_BROKEN_CHAFF, c.owner, m.shooter, HitZone::FUSELAGE, time, 0.0});
                }
            }
        }
    }

    void step_flares(double dt) noexcept {
        for (auto& f : flares) {
            if (!f.active) continue;
            f.step(dt);
            if (f.bloomed || f.age < 0.15 || f.owner < 0) continue;
            f.bloomed = true;
            const Aircraft& victim = aircraft[static_cast<size_t>(f.owner)];
            for (auto& m : missiles) {
                if (!m.active || m.target != f.owner || !m.spec.infrared()) continue;
                if (!flare_in_gates(m.body.pos, victim.position(), f, m.spec.ir_fov_deg * M_PI / 180.0)) {
                    continue;
                }
                const double p = m.spec.flare_susceptibility * (f.intensity() / 2.0);
                if (rng_.uniform() < p) {
                    m.seduce(f);
                    push({CombatEvent::Kind::LOCK_BROKEN_FLARE, f.owner, m.shooter, HitZone::FUSELAGE, time, 0.0});
                }
            }
        }
    }

    void step_missiles(double dt) noexcept {
        for (auto& m : missiles) {
            if (!m.active) continue;
            const Aircraft& tgt = aircraft[static_cast<size_t>(m.target)];
            const auto& radar = radars[static_cast<size_t>(m.shooter)];
            const bool uplink = aircraft[static_cast<size_t>(m.shooter)].alive() && radar.target == m.target &&
                                radar.track_fresh();
            const math::Vector3 p0 = m.body.pos;
            m.update(dt, uplink, tgt.position(), tgt.velocity());

            // Proximity fuze: closest approach this step, swept against every jet but the shooter.
            if (m.body.tof > m.spec.arm_time_s) {
                for (int j = 0; j < count; ++j) {
                    if (j == m.shooter) continue;
                    Aircraft& t = aircraft[static_cast<size_t>(j)];
                    if (t.crashed) continue;
                    double s = 0.0;
                    const double d = closest_approach(p0, m.body.pos, t.prev_state.pos_ned, t.state.pos_ned, s);
                    if (d < m.spec.fuze_radius_m && s < 1.0) {
                        const math::Vector3 burst = p0 + (m.body.pos - p0) * s;
                        detonate(m, j, burst, d);
                        break;
                    }
                }
            }
            if (m.active && m.expired()) {
                m.active = false;
                push({CombatEvent::Kind::MISSILE_DEFEATED, m.target, m.shooter, HitZone::FUSELAGE, time, 0.0});
            }
        }
    }

    /// @brief Blast-fragmentation warhead. A burst within a few metres destroys
    /// the jet. Further out, fragments strike components in proportion to the
    /// solid angle the jet fills, weighted to the side facing the burst.
    void detonate(Missile& m, int victim, const math::Vector3& burst, double miss) noexcept {
        m.active = false;
        Aircraft& t = aircraft[static_cast<size_t>(victim)];
        constexpr double DIRECT_HIT_M = 3.0;
        constexpr double MAX_FRAGMENT_HITS = 18.0;
        HitZone first = HitZone::FUSELAGE;
        const int hits_before = t.damage.hits_taken;
        if (miss <= DIRECT_HIT_M) {
            ++t.damage.hits_taken;
            t.damage.destroyed = true;
        } else {
            const double f = std::clamp(1.0 - miss / m.spec.lethal_radius_m, 0.0, 1.0);
            const int hits = static_cast<int>(std::lround(MAX_FRAGMENT_HITS * f * f));
            const math::Vector3 side_b = t.state.q_att.rotate_ned_to_body(burst - t.position());
            const bool left = side_b.y < 0.0;
            for (int k = 0; k < hits && !t.damage.destroyed; ++k) {
                const double u = rng_.uniform();
                HitZone z = HitZone::FUSELAGE;
                if (u < 0.08) z = HitZone::COCKPIT;
                else if (u < 0.36) z = HitZone::FUSELAGE;
                else if (u < 0.60) z = HitZone::ENGINE;
                else if (u < 0.82) z = left ? HitZone::WING_LEFT : HitZone::WING_RIGHT;
                else if (u < 0.90) z = left ? HitZone::WING_RIGHT : HitZone::WING_LEFT;
                else z = HitZone::TAIL;
                if (k == 0) first = z;
                t.damage.apply_hit(z, 1.0, rng_);
            }
        }
        last_hit_by_[static_cast<size_t>(victim)] = m.shooter;
        last_hit_time_[static_cast<size_t>(victim)] = time;
        push({CombatEvent::Kind::MISSILE_DETONATION, victim, m.shooter, first, time, miss});
        if (t.damage.hits_taken > hits_before) push({CombatEvent::Kind::HIT, victim, m.shooter, first, time, miss});
    }

    void push(CombatEvent e) noexcept {
        if (event_count_ < kMaxEvents) events_[static_cast<size_t>(event_count_++)] = e;
    }

    double gaussian() noexcept {
        const double u1 = std::max(1e-12, rng_.uniform());
        const double u2 = rng_.uniform();
        return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * M_PI * u2);
    }

    void fire_guns(double dt, const std::array<AircraftControls, kMaxAircraft>& controls) noexcept {
        for (int i = 0; i < count; ++i) {
            Aircraft& a = aircraft[static_cast<size_t>(i)];
            const bool trigger = controls[static_cast<size_t>(i)].trigger && a.alive() && !a.crashed;
            const int n = a.gun.update(dt, trigger);
            if (n <= 0) continue;
            const double k = Ballistics::drag_factor(a.gun.spec);
            const double sigma = dispersion ? a.gun.spec.dispersion_mrad * 1e-3 : 0.0;
            for (int r = 0; r < n; ++r) {
                math::Vector3 dir = a.gun.boresight_b();
                if (sigma > 0.0) {
                    dir = math::Vector3(dir.x, dir.y + sigma * gaussian(), dir.z + sigma * gaussian()).normalized();
                }
                Projectile& p = rounds[static_cast<size_t>(next_round_)];
                next_round_ = (next_round_ + 1) % kMaxRounds;
                Ballistics::launch(a.state, a.gun.spec, dir, p.pos, p.vel);
                // Rounds leave evenly through the step rather than in a clump.
                const double lead = dt * (static_cast<double>(r) + 0.5) / static_cast<double>(n);
                p.pos += p.vel * (dt * 0.5 - lead);
                p.k_drag = k;
                p.lethality = a.gun.spec.lethality;
                p.age = 0.0;
                p.owner = i;
                p.active = true;
            }
        }
    }

    void step_rounds(double dt) noexcept {
        for (auto& p : rounds) {
            if (!p.active) continue;
            const math::Vector3 p0 = p.pos;
            Ballistics::step(p.pos, p.vel, p.k_drag, dt);
            p.age += dt;
            if (p.age > Ballistics::MAX_TIME_OF_FLIGHT_S ||
                p.pos.z > environment::GroundCollision::get_ground_z(p.pos.x, p.pos.y)) {
                p.active = false;
                continue;
            }
            for (int j = 0; j < count; ++j) {
                if (j == p.owner) continue;
                Aircraft& t = aircraft[static_cast<size_t>(j)];
                if (t.crashed) continue;
                math::Vector3 hit_b;
                if (!sweep_hit(p0, p.pos, t.prev_state, t.state, airframe_combat_spec(t.type), hit_b)) continue;

                const HitZone zone = classify_hit(hit_b, airframe_combat_spec(t.type));
                t.damage.apply_hit(zone, p.lethality, rng_);
                last_hit_by_[static_cast<size_t>(j)] = p.owner;
                last_hit_time_[static_cast<size_t>(j)] = time;
                push({CombatEvent::Kind::HIT, j, p.owner, zone, time, 0.0});
                p.active = false;
                break;
            }
        }
    }

    void check_midairs() noexcept {
        for (int i = 0; i < count; ++i) {
            for (int j = i + 1; j < count; ++j) {
                Aircraft& a = aircraft[static_cast<size_t>(i)];
                Aircraft& b = aircraft[static_cast<size_t>(j)];
                if (a.crashed || b.crashed || a.damage.destroyed || b.damage.destroyed) continue;
                const auto sa = airframe_combat_spec(a.type);
                const auto sb = airframe_combat_spec(b.type);
                // Bounding capsules along each fuselage, swept over the step in a's frame of reference.
                const double reach = 0.5 * (std::max(sa.length_m, sa.span_m) + std::max(sb.length_m, sb.span_m));
                if ((a.state.pos_ned - b.state.pos_ned).norm() > reach + 20.0) continue;
                const math::Vector3 ax = a.state.q_att.rotate_body_to_ned(math::Vector3(0.45 * sa.length_m, 0, 0));
                const math::Vector3 bx = b.state.q_att.rotate_body_to_ned(math::Vector3(0.45 * sb.length_m, 0, 0));
                double s = 0.0, t = 0.0;
                const double d = segment_segment_distance(a.state.pos_ned - ax, a.state.pos_ned + ax,
                                                          b.state.pos_ned - bx, b.state.pos_ned + bx, s, t);
                // Fuselages plus a share of the wings: jets whose axes pass this
                // close are overlapping somewhere unless perfectly stacked.
                const double contact = sa.fuselage_radius_m + sb.fuselage_radius_m + 0.25 * (sa.span_m + sb.span_m);
                if (d < contact) {
                    a.damage.destroyed = true;
                    b.damage.destroyed = true;
                    push({CombatEvent::Kind::MIDAIR, i, j, HitZone::FUSELAGE, time, 0.0});
                    push({CombatEvent::Kind::MIDAIR, j, i, HitZone::FUSELAGE, time, 0.0});
                    was_alive_[static_cast<size_t>(i)] = false;
                    was_alive_[static_cast<size_t>(j)] = false;
                }
            }
        }
    }

    [[nodiscard]] int credited_shooter(int victim) const noexcept {
        const auto v = static_cast<size_t>(victim);
        return (time - last_hit_time_[v] <= KILL_CREDIT_WINDOW_S) ? last_hit_by_[v] : -1;
    }

    void check_ground_impacts() noexcept {
        for (int i = 0; i < count; ++i) {
            const auto idx = static_cast<size_t>(i);
            const Aircraft& a = aircraft[idx];
            if (a.crashed && !was_crashed_[idx]) {
                was_crashed_[idx] = true;
                push({CombatEvent::Kind::GROUND_IMPACT, i, credited_shooter(i), HitZone::FUSELAGE, time, 0.0});
                if (was_alive_[idx]) {
                    was_alive_[idx] = false;
                    push({CombatEvent::Kind::KILL, i, credited_shooter(i), HitZone::FUSELAGE, time, 0.0});
                }
            }
        }
    }

    /// @brief A jet shot out of the fight (destroyed or pilot down) is a kill the
    /// moment it happens, not when the wreck reaches the ground.
    void check_mission_kills() noexcept {
        for (int i = 0; i < count; ++i) {
            const auto idx = static_cast<size_t>(i);
            if (was_alive_[idx] && !aircraft[idx].alive()) {
                was_alive_[idx] = false;
                if (aircraft[idx].damage.structural_failure) {
                    push({CombatEvent::Kind::STRUCTURAL_FAILURE, i, -1, HitZone::FUSELAGE, time, 0.0});
                }
                if (!aircraft[idx].crashed) {
                    push({CombatEvent::Kind::KILL, i, credited_shooter(i), HitZone::FUSELAGE, time, 0.0});
                }
            }
        }
    }
};

} // namespace fastjet::sim
