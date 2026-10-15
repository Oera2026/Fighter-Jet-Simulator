#pragma once

#include "fastjet/aircraft/aircraft_config.hpp"
#include "fastjet/environment/atmosphere1976.hpp"
#include "fastjet/environment/ground_collision.hpp"
#include "fastjet/flcs/onboard_flight_computer.hpp"
#include "fastjet/sim/air_combat_geometry.hpp"
#include "fastjet/sim/aircraft.hpp"
#include "fastjet/sim/damage_model.hpp"
#include "fastjet/sim/gun.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace fastjet::sim {

enum class AiSkill : uint8_t { NOVICE, VETERAN, ACE };

[[nodiscard]] constexpr const char* to_string(AiSkill s) noexcept {
    switch (s) {
        case AiSkill::NOVICE:  return "NOVICE";
        case AiSkill::VETERAN: return "VETERAN";
        case AiSkill::ACE:     return "ACE";
    }
    return "?";
}

/**
 * @brief What separates a novice from an ace: perception and physiology, never
 * physics. Every tier flies the same jet through the same FLCS and OFC.
 */
struct PilotSkill {
    double reaction_s;      ///< Delay before a bandit's manoeuvre is noticed
    double g_fraction;      ///< Share of the airframe G limit the pilot will pull
    double aim_noise_mrad;  ///< Tracking error (slowly wandering)
    double fire_miss_m;     ///< Predicted miss below which the trigger is squeezed
    double max_fire_range_m;
    double assess_period_s; ///< How often the fight is re-assessed (checks six)
    bool uses_energy;       ///< Manages speed around corner velocity
    double greyout_limit;   ///< Vision loss at which the pilot eases the pull (GLOC awareness)

    [[nodiscard]] static constexpr PilotSkill of(AiSkill s) noexcept {
        switch (s) {
            case AiSkill::NOVICE:  return {0.45, 0.72, 5.0, 6.0, 700.0, 0.8, false, 0.45};
            case AiSkill::VETERAN: return {0.30, 0.85, 2.5, 4.5, 900.0, 0.4, true, 0.25};
            case AiSkill::ACE:     return {0.18, 0.95, 1.2, 4.0, 1000.0, 0.2, true, 0.15};
        }
        return {0.3, 0.85, 2.5, 4.5, 900.0, 0.4, true, 0.25};
    }
};

/**
 * @brief A missile the pilot knows is coming at him. Engagement::detect_missile_threat()
 * is the one place that decides what an AI pilot knows about missiles: his
 * RWR, what he can see, and his own jet's missile approach warner (MAWS).
 *
 * What it is decides the countermeasure: chaff against a radar missile
 * (heard on the RWR), flares against a heat seeker (seen by the MAWS, silent
 * on the RWR), both when the MAWS and the RWR both have it or when it was
 * only seen and its type is unknown.
 */
struct MissileThreat {
    bool inbound{false};
    bool radar_guided{false}; ///< Heard on the RWR: a radar missile.
    bool maws{false};         ///< The MAWS has it (or a passive set had it until its motor burnt out)
    bool held{false};         ///< ...had it: a passive set lost it at burnout, and its last time to go is counting down
    bool heat_seeker{false};  ///< The MAWS has it and the RWR is silent: treated as a heat seeker
    bool launch{false};       ///< The MAWS declared a new missile this step
    math::Vector3 pos{};      ///< Where the missile is [m, NED] (MAWS only: on its line of sight, at an estimated range)
    double range_m{0.0};
    double closure_mps{0.0};
    double tgo_s{-1.0};       ///< The MAWS's time to go (allows for a boosting missile); negative if none

    [[nodiscard]] double time_to_go() const noexcept {
        if (tgo_s >= 0.0) return tgo_s;
        return closure_mps > 50.0 ? range_m / closure_mps : 1e9;
    }
    /// @brief Chaff decoys a radar missile, or one whose type is unknown.
    [[nodiscard]] bool wants_chaff() const noexcept { return inbound && !heat_seeker; }
    /// @brief Flares decoy a heat seeker, or one whose type is unknown or that the MAWS sees too.
    [[nodiscard]] bool wants_flares() const noexcept { return inbound && (!radar_guided || maws); }
};

/// @brief What an AI pilot knows about a bandit (a delayed track, not the truth).
struct TrackSample {
    math::Vector3 pos{};
    math::Vector3 vel{};
    math::Vector3 acc{};
    double time{-1.0};
};

/**
 * @brief Basic fighter manoeuvring AI.
 *
 * Three layers:
 *  1. Tactical: every assess_period, classify the fight as offensive,
 *     defensive or neutral from range, angle-off and aspect.
 *  2. Manoeuvre: each situation becomes an aim point, a G ceiling and a
 *     throttle setting (pure / lead / lag pursuit, break turn, jink,
 *     extension, hard-deck recovery). Expressing every manoeuvre this way
 *     keeps the control layer single and testable.
 *  3. Control: a lift-vector controller banks to put the aim point in the lift
 *     plane (gravity included) and closes the loop on measured Nz, producing
 *     ordinary stick and throttle for the jet's own FLCS and OFC.
 *
 * Energy decisions use corner speed computed from the jet's own wing area,
 * CLmax, mass and G limit, not per-type tables.
 */
class BfmPilot {
public:
    enum class Situation : uint8_t { NEUTRAL, OFFENSIVE, DEFENSIVE };
    enum class Maneuver : uint8_t {
        PURE_PURSUIT, LEAD_PURSUIT, LAG_PURSUIT, BREAK, JINK, EXTEND, HARD_DECK, MERGE,
        DEFEND_MISSILE, ///< Turning cold on, or beaming, a missile he knows is inbound
    };

    PilotSkill skill{PilotSkill::of(AiSkill::VETERAN)};
    double hard_deck_agl_m{1000.0};
    /// Fastest the pilot will chase a runner. Guns fights gain nothing past the
    /// transonic region. A pilot hunting a runner with missiles will go supersonic,
    /// but never past MAX_CHASE_Q_PA, the high-q corner where the jet's FLCS is at its worst.
    double max_chase_mach{0.97};
    /// Dynamic pressure the pilot accepts while still far out (beyond
    /// MANOEUVRE_RANGE_M), where he is only making small pursuit corrections.
    /// Closer in he slows below MAX_CHASE_Q_PA before he has to manoeuvre.
    double far_chase_q_pa{MAX_CHASE_Q_PA};

    // Diagnostics
    Situation situation{Situation::NEUTRAL};
    Maneuver maneuver{Maneuver::PURE_PURSUIT};
    double g_cmd{1.0};
    double aim_error_rad{0.0};
    bool trigger{false};

    BfmPilot() noexcept = default;
    explicit BfmPilot(AiSkill s, uint32_t seed = 7) noexcept : skill(PilotSkill::of(s)) { rng_.state = seed ? seed : 7u; }

    void reset() noexcept {
        track_ = {};
        track_head_ = 0;
        assess_timer_ = 0.0;
        nz_trim_ = 0.0;
        burst_timer_ = 0.0;
        cooldown_timer_ = 0.0;
        jink_timer_ = 0.0;
        jink_sign_ = 1.0;
        noise_y_ = noise_z_ = 0.0;
        los_rate_ = {};
        has_prev_aim_ = false;
        solution_ = {};
        solution_timer_ = 0.0;
        g_easing_ = false;
        threat_cm_timer_ = 0.0;
        salvo_left_ = 0;
        salvo_armed_ = true;
        flare_out_ = false;
        beam_side_ = 0.0;
        roll_stick_ = 0.0;
        rudder_int_ = 0.0;
        g_cmd = 1.0;
        last_sample_time_ = -1.0;
        time_ = 0.0;
        situation = Situation::NEUTRAL;
        maneuver = Maneuver::PURE_PURSUIT;
    }

    /// @brief Fly one step against @p bandit, defending against @p threat if
    /// one is inbound. Call every physics step.
    [[nodiscard]] AircraftControls update(double dt, const Aircraft& self, const Aircraft& bandit,
                                          const MissileThreat& threat = {}) noexcept {
        time_ += dt;
        record_track(bandit);
        AircraftControls out{};
        out.throttle = 1.0;
        trigger = false;
        if (!self.alive()) return out;

        const TrackSample seen = perceived_track(self.state.pos_ned);
        // His ATA on us is judged from his (delayed) flight path.
        const AirCombatGeometry geo = geometry_to(self.state, seen);
        const double his_ata = AirCombatGeometry::angle_between(seen.vel, self.state.pos_ned - seen.pos);

        assess_timer_ -= dt;
        if (assess_timer_ <= 0.0) {
            assess_timer_ = skill.assess_period_s;
            situation = assess(geo, his_ata);
        }

        const aircraft::AircraftConfig cfg = aircraft::AircraftConfig::get(self.type);
        const double V = self.state.airspeed();
        const double corner = corner_speed(self, cfg);
        const double a_sound = environment::Atmosphere1976::compute(self.state.altitude()).speed_of_sound;
        const double g_max = cfg.flcs.max_g_positive * skill.g_fraction;

        // ---- Manoeuvre layer: aim point, G ceiling, throttle ----
        math::Vector3 aim = seen.pos;
        double g_ceiling = g_max;
        double closure_throttle = 1.0;
        bool speedbrake = false;
        GunSolution sol{};

        switch (situation) {
            case Situation::OFFENSIVE: {
                // The ballistic solution is a few hundred integration steps: refresh it at 20 Hz.
                solution_timer_ -= dt;
                if (solution_timer_ <= 0.0 || !solution_.valid) {
                    solution_timer_ = 0.05;
                    solution_ = GunSolution::compute(self.state, self.gun.spec, seen.pos, seen.vel, seen.acc);
                    // Between refreshes, hold the lead point as an offset from the
                    // bandit. Held in body axes it would turn with the nose, and the
                    // error the tracking loop sees would freeze while the jet moves.
                    lead_offset_ = self.state.q_att.rotate_body_to_ned(solution_.lead_dir_b) * geo.range -
                                   (seen.pos - self.state.pos_ned);
                }
                sol = solution_;
                if (geo.range < 1400.0 && sol.valid) {
                    maneuver = Maneuver::LEAD_PURSUIT;
                    aim = seen.pos + lead_offset_;
                } else if (geo.range < 2500.0 && geo.closure > 80.0) {
                    // Too fast inside: lag to hold turn circle position without overshooting.
                    maneuver = Maneuver::LAG_PURSUIT;
                    aim = seen.pos - seen.vel * 1.5;
                } else {
                    maneuver = Maneuver::PURE_PURSUIT;
                }
                // Closure control inside gun range: arrive at ~500 m with modest closure.
                if (geo.range < 2000.0) {
                    const double desired_vc = std::clamp((geo.range - 450.0) * 0.08, -20.0, 80.0);
                    closure_throttle = std::clamp(0.75 + 0.02 * (desired_vc - geo.closure), 0.0, 1.0);
                    speedbrake = geo.closure > desired_vc + 60.0;
                }
                break;
            }
            case Situation::DEFENSIVE: {
                // Lift vector on the bandit: the break turn denies his solution.
                aim = seen.pos;
                maneuver = Maneuver::BREAK;
                if (geo.range < 1200.0 && his_ata < 12.0 * DEG) {
                    // He is tracking: jink out of his plane of motion.
                    maneuver = Maneuver::JINK;
                    jink_timer_ -= dt;
                    if (jink_timer_ <= 0.0) {
                        jink_timer_ = 0.8 + 0.8 * rng_.uniform();
                        jink_sign_ = -jink_sign_;
                    }
                    const math::Vector3 v = self.state.velocity_ned();
                    const math::Vector3 side = v.cross(seen.pos - self.state.pos_ned).normalized();
                    aim = seen.pos + side * (jink_sign_ * 600.0);
                }
                break;
            }
            case Situation::NEUTRAL: {
                maneuver = Maneuver::PURE_PURSUIT;
                aim = seen.pos;
                if (geo.hca > 110.0 * DEG && geo.closure > 0.0 && geo.range < 5000.0) {
                    // Merge: the training rule is a left-to-left pass. Aim to
                    // the right of the bandit so he passes down the left side
                    // at a few hundred metres; lead-turn once abeam.
                    maneuver = Maneuver::MERGE;
                    const math::Vector3 los = (seen.pos - self.state.pos_ned) / std::max(1.0, geo.range);
                    // NED: line of sight x up points to our right (north x up = east).
                    math::Vector3 right = los.cross(math::Vector3(0.0, 0.0, -1.0));
                    right = (right.norm() < 1e-6) ? math::Vector3(0.0, 1.0, 0.0) : right.normalized();
                    aim = seen.pos + right * MERGE_PASS_M;
                } else if (geo.range > 6000.0) {
                    // Point at the bandit's future position while closing to the merge.
                    maneuver = Maneuver::LEAD_PURSUIT;
                    const double t = std::clamp(geo.range / std::max(100.0, geo.closure), 0.0, 20.0);
                    aim = seen.pos + seen.vel * (0.3 * t);
                }
                break;
            }
        }

        // Missile defence: a missile he knows about comes before the bandit.
        //
        // His MAWS calls the launch, and gives its line of sight and time to
        // go: a trained pilot with time in hand turns cold while the time to
        // go is long (missile at 6 o'clock: make it chase), then beams it
        // (missile at 3 or 9), where its closure is least and its line of
        // sight turns fastest. With only a few seconds left, or untrained, he
        // turns hard into it. A second or two before impact, a maximum-G break
        // perpendicular to its line of sight, so all of the pull turns the
        // line of sight: the missile has to follow with the G it has left.
        //
        // Without the MAWS (a missile heard on the RWR, or one he has seen) he
        // fights on and breaks hard into it a second or two before impact,
        // across its line of sight.
        const double tgo = threat.inbound ? threat.time_to_go() : 1e9;
        const bool endgame = threat.inbound && tgo < MISSILE_BREAK_TGO_S + skill.reaction_s;
        if (threat.inbound && !threat.maws) {
            if (endgame) {
                maneuver = Maneuver::BREAK;
                const math::Vector3 v_dir = self.state.velocity_ned().normalized();
                const math::Vector3 to_m = threat.pos - self.state.pos_ned;
                math::Vector3 across = to_m - v_dir * to_m.dot(v_dir);
                if (across.norm() < 1.0) across = self.state.q_att.rotate_body_to_ned(math::Vector3(0.0, 0.0, -1.0));
                aim = self.state.pos_ned + v_dir * 300.0 + across.normalized() * 1000.0;
                g_ceiling = g_max;
            }
            beam_side_ = 0.0;
        } else if (threat.inbound) {
            const math::Vector3 v_dir = self.state.velocity_ned().normalized();
            const math::Vector3 to_m = threat.pos - self.state.pos_ned;
            const math::Vector3 los = to_m.norm() > 1.0 ? to_m.normalized() : -v_dir;
            g_ceiling = g_max;
            if (endgame) {
                maneuver = Maneuver::BREAK;
                aim = self.state.pos_ned + v_dir * 300.0 + break_across(self, v_dir, los) * 1000.0;
            } else if (skill.uses_energy && tgo > MISSILE_BEAM_TGO_S) {
                maneuver = Maneuver::DEFEND_MISSILE;
                aim = self.state.pos_ned + defensive_heading(v_dir, los, tgo) * 1000.0;
            } else {
                // Defensive turn: lift vector on the missile.
                maneuver = Maneuver::DEFEND_MISSILE;
                math::Vector3 into = los - v_dir * los.dot(v_dir);
                if (into.norm() < 0.05) into = self.state.q_att.rotate_body_to_ned(math::Vector3(0.0, 0.0, -1.0));
                aim = self.state.pos_ned + v_dir * 600.0 + into.normalized() * 1000.0;
            }
        } else {
            beam_side_ = 0.0;
        }

        // Overshoot and collision avoidance: nobody presses inside the minimum
        // range or flies toward a collision. The closest point of approach is
        // predicted from the relative motion; if it is too close and coming
        // soon, each pilot turns his own flight path away from it and comes off
        // the power. If both do it they can only separate.
        {
            const math::Vector3 r = seen.pos - self.state.pos_ned;
            const math::Vector3 v_rel = seen.vel - self.state.velocity_ned();
            const double vv = std::max(1e-6, v_rel.dot(v_rel));
            const double t_cpa = -r.dot(v_rel) / vv;
            const math::Vector3 miss = r + v_rel * std::max(0.0, t_cpa);
            const bool collision_course = t_cpa > 0.0 && t_cpa < CPA_HORIZON_S && miss.norm() < CPA_MIN_MISS_M;
            if (geo.range < MIN_RANGE_M || collision_course) {
                maneuver = Maneuver::EXTEND;
                const math::Vector3 v_dir = self.state.velocity_ned().normalized();
                math::Vector3 away = -miss;
                away = away - v_dir * away.dot(v_dir);
                // Near the hard deck, separate sideways or up, never down.
                const double agl_now = environment::GroundCollision::get_agl(self.state);
                if (agl_now < hard_deck_agl_m + 1500.0 && away.z > 0.0) away.z = 0.0;
                if (away.norm() < 1.0) {
                    // Dead on: pull through the lift vector.
                    away = self.state.q_att.rotate_body_to_ned(math::Vector3(0.0, 0.0, -1.0));
                }
                aim = self.state.pos_ned + v_dir * 500.0 + away.normalized() * 800.0;
                closure_throttle = 0.0;
                speedbrake = geo.closure > 10.0 && t_cpa > 1.0;
                g_ceiling = g_max; // separation takes priority over energy
            }
        }

        // Energy: turning slower than corner speed bleeds energy fast. Ease
        // the pull to recover speed unless the shot (or survival) is now.
        if (skill.uses_energy && maneuver != Maneuver::BREAK && maneuver != Maneuver::JINK &&
            maneuver != Maneuver::LEAD_PURSUIT && maneuver != Maneuver::EXTEND) {
            const double ratio = V / std::max(1.0, corner);
            if (ratio < 0.85) {
                g_ceiling = std::max(2.0, g_max * std::clamp((ratio - 0.55) / 0.30, 0.0, 1.0));
            }
        }
        if (V < 0.6 * corner) g_ceiling = std::min(g_ceiling, 3.0);

        // G awareness: the same OFC physiology that can knock the player out
        // applies here. Once vision starts to grey, relax to the G the body
        // can sustain until it clears; better pilots notice earlier.
        const double greyout = self.ofc.blackout_fraction_value();
        if (greyout > skill.greyout_limit) g_easing_ = true;
        else if (greyout < 0.5 * skill.greyout_limit) g_easing_ = false;
        if (g_easing_) {
            g_ceiling = std::min(g_ceiling, flcs::OnBoardFlightComputer::GLOC_BASE_TOLERANCE_G - 0.5);
        }

        // Throttle: hold the fight near corner speed, where turn rate peaks.
        // Well above it every G buys less turn and the fight runs away
        // downhill; well below it the jet cannot hold the G it needs.
        double throttle = 1.0;
        const double rho = environment::Atmosphere1976::compute(self.state.altitude()).density;
        const double q_cap = geo.range > MANOEUVRE_RANGE_M ? std::max(far_chase_q_pa, MAX_CHASE_Q_PA) : MAX_CHASE_Q_PA;
        const double v_cap = std::min(max_chase_mach * a_sound, std::sqrt(2.0 * q_cap / rho));
        if (skill.uses_energy) {
            double v_target = corner * (situation == Situation::DEFENSIVE ? 1.0 : 1.08);
            if (situation == Situation::OFFENSIVE) {
                // A bandit running away has to be caught: match his speed plus
                // enough closure to reach gun range, but not past the transonic
                // region, where a guns fight has nothing left to gain.
                const double chase = seen.vel.norm() + std::clamp((geo.range - 500.0) * 0.06, 0.0, 90.0);
                v_target = std::max(v_target, std::min(chase, v_cap));
            }
            throttle = std::clamp(1.0 - (V - v_target) / 50.0, 0.05, 1.0);
            speedbrake = speedbrake || V > 1.35 * v_target;
        }
        // Defending a missile his MAWS has, energy is life: full power, no speedbrake.
        if (threat.inbound && threat.maws) {
            throttle = 1.0;
            speedbrake = false;
        }
        // Everyone, novice or not, comes off the power rather than ram the bandit.
        if (V > 0.9 * corner || geo.range < 600.0) throttle = std::min(throttle, closure_throttle);
        if (V > v_cap - 0.02 * a_sound) throttle = std::min(throttle, 0.8); // out of reheat
        if (V > v_cap + 0.08 * a_sound) speedbrake = true;
        // A missile that may be a heat seeker (the MAWS has it and the RWR is
        // silent, or it was only seen): a trained pilot comes out of reheat,
        // the brightest thing in its seeker's view.
        if (threat.inbound && !threat.radar_guided && skill.uses_energy) throttle = std::min(throttle, 0.8);

        // Hard deck: blend the aim point up and away from the ground, starting
        // with enough height left to pull out of the current dive. A pull-out
        // at n G from dive angle gamma loses R (1 - cos gamma), R = V^2 / ((n - 1) g).
        const double agl = environment::GroundCollision::get_agl(self.state);
        const math::Vector3 v_ned = self.state.velocity_ned();
        const double gamma = std::atan2(-v_ned.z, std::hypot(v_ned.x, v_ned.y));
        const double pull_n = std::max(2.0, g_max - 1.0);
        const double radius = V * V / (pull_n * fdm::SixDoFFDM::GRAVITY_ACCEL);
        const double pullout_loss = gamma < 0.0 ? radius * (1.0 - std::cos(gamma)) : 0.0;
        const double deck_margin = hard_deck_agl_m + 300.0 + 1.3 * pullout_loss;
        if (agl < deck_margin + 400.0) {
            const double w = std::clamp((deck_margin + 400.0 - agl) / 600.0, 0.0, 1.0);
            const math::Vector3 horiz = math::Vector3(v_ned.x, v_ned.y, 0.0).normalized();
            const math::Vector3 up_aim = (horiz + math::Vector3(0.0, 0.0, -0.6)).normalized();
            const math::Vector3 to_aim = (aim - self.state.pos_ned).normalized();
            // Only the downhill part of the aim is overridden: a climbing aim is left alone.
            if (to_aim.z > up_aim.z || w >= 1.0) {
                const math::Vector3 blended = to_aim * (1.0 - w) + up_aim * w;
                aim = self.state.pos_ned + blended.normalized() * 1000.0;
                if (w > 0.5) maneuver = Maneuver::HARD_DECK;
            }
        }

        // Tracking error: a slowly wandering aim offset.
        const double sigma = skill.aim_noise_mrad * 1e-3;
        const double theta = 1.0 - std::exp(-dt / 0.6);
        noise_y_ += (gaussian() * sigma - noise_y_) * theta;
        noise_z_ += (gaussian() * sigma - noise_z_) * theta;
        {
            const math::Vector3 d_b = self.state.q_att.rotate_ned_to_body(aim - self.state.pos_ned);
            const double r = d_b.norm();
            const math::Vector3 jitter_b{0.0, noise_y_ * r, noise_z_ * r};
            aim = aim + self.state.q_att.rotate_body_to_ned(jitter_b);
        }

        // ---- Control layer ----
        fly_towards(dt, self, cfg, aim, seen.pos, g_ceiling, maneuver == Maneuver::LEAD_PURSUIT, out);
        out.throttle = throttle;
        out.speedbrake_out = speedbrake;

        // ---- Gun employment ----
        if (situation == Situation::OFFENSIVE && sol.valid) {
            update_trigger(dt, self, geo, sol);
        } else {
            burst_timer_ = 0.0;
            cooldown_timer_ = std::max(0.0, cooldown_timer_ - dt);
        }
        out.trigger = trigger;

        // ---- Countermeasure employment ----
        // Guns-defensive: pre-emptive flares and chaff against a shot he can't
        // see coming (not while his MAWS has a missile: then it decides).
        if (situation == Situation::DEFENSIVE && !threat.maws) {
            flare_timer_ -= dt;
            if (flare_timer_ <= 0.0) {
                out.dispense_flare = true;
                out.dispense = true;
                flare_timer_ = (skill.uses_energy ? 1.0 : 2.2) + 0.4 * rng_.uniform();
            }
        }
        // A missile inbound, all the way in: chaff against a radar missile,
        // flares against a heat seeker, both when its type is unknown or the
        // MAWS and the RWR both have it. A MAWS launch call starts a program
        // at once.
        threat_cm_timer_ = std::max(0.0, threat_cm_timer_ - dt);
        if (threat.inbound && (threat.launch || threat_cm_timer_ <= 0.0)) {
            out.dispense = threat.wants_chaff();
            out.dispense_flare = out.dispense_flare || threat.wants_flares();
            threat_cm_timer_ = skill.uses_energy ? 0.6 : 1.2;
        }
        // The MAWS-timed endgame break goes with a flare salvo: several
        // programs back to back, so fresh flares burn in the seeker's view all
        // through the last second or two. Re-armed once no threat is close.
        if (endgame && salvo_armed_ && threat.maws && threat.wants_flares()) {
            salvo_left_ = FLARE_SALVO_PROGRAMS;
            salvo_armed_ = false;
        } else if (!threat.inbound || tgo > MISSILE_BREAK_TGO_S + skill.reaction_s + 1.0) {
            salvo_armed_ = true;
        }
        if (salvo_left_ > 0) {
            // Each program is one press of the switch: release it every other step.
            out.dispense_flare = !flare_out_;
            if (out.dispense_flare) --salvo_left_;
        }
        flare_out_ = out.dispense_flare;
        // Missile shots are not the BFM pilot's: see MissileShooter and HeatSeekerShooter.
        return out;
    }

    /// @brief Corner speed from the jet's own wing, CLmax, weight and G limit.
    [[nodiscard]] static double corner_speed(const Aircraft& self, const aircraft::AircraftConfig& cfg) noexcept {
        const auto atm = environment::Atmosphere1976::compute(self.state.altitude());
        const double w = self.mass.mass_kg * fdm::SixDoFFDM::GRAVITY_ACCEL;
        const double k = 2.0 * cfg.flcs.max_g_positive * w / (atm.density * cfg.aero.s_ref * cfg.aero.cl_max);
        double v = std::sqrt(k);
        for (int i = 0; i < 4; ++i) {
            const double f = aero::CompressibleLift::clmax_mach_factor(v / atm.speed_of_sound);
            v = std::sqrt(k / f);
        }
        return v;
    }

private:
    static constexpr double DEG = M_PI / 180.0;
    static constexpr double G_ONSET_RATE = 7.0;   ///< [G/s]
    static constexpr double MERGE_PASS_M = 450.0; ///< Lateral pass distance at the merge
    static constexpr double MIN_RANGE_M = 150.0;  ///< Training-rules minimum range (~500 ft)
    static constexpr double CPA_HORIZON_S = 4.0;  ///< Look-ahead for collision avoidance
    static constexpr double CPA_MIN_MISS_M = 200.0;
    static constexpr double MAX_SHOT_ASPECT = 120.0 * M_PI / 180.0; ///< No forward-quarter snapshots
    static constexpr double MISSILE_BREAK_TGO_S = 1.8; ///< Endgame break this long before impact (plus reaction)
    static constexpr double MISSILE_COLD_TGO_S = 10.0; ///< Turn cold on a missile further out than this; beam it nearer
    static constexpr double MISSILE_BEAM_TGO_S = 4.0;  ///< ...down to this; then a defensive turn into it
    static constexpr int FLARE_SALVO_PROGRAMS = 3;     ///< Flare programs fired with the MAWS-timed endgame break
    static constexpr double G_OFFSET_RATE = 12.0; ///< [G/s]
    static constexpr double MAX_CHASE_Q_PA = 75000.0; ///< ~350 m/s at sea level, ~450 m/s at 5 km
    static constexpr double MANOEUVRE_RANGE_M = 6000.0;
    static constexpr double ROLL_BANDWIDTH_RAD_S = 3.0;
    static constexpr double ROLL_RATE_DAMPING = 0.2;
    static constexpr double SEE_TURN_RANGE_M = 1500.0;
    static constexpr int kTrackLen = 48;
    static constexpr double kTrackPeriod = 0.02; // 50 Hz

    std::array<TrackSample, kTrackLen> track_{};
    int track_head_{0};
    double last_sample_time_{-1.0};
    double time_{0.0};
    double assess_timer_{0.0};
    double nz_trim_{0.0};
    double burst_timer_{0.0};
    double cooldown_timer_{0.0};
    double jink_timer_{0.0};
    double jink_sign_{1.0};
    double noise_y_{0.0};
    double noise_z_{0.0};
    GunSolution solution_{};
    math::Vector3 lead_offset_{};
    double solution_timer_{0.0};
    bool g_easing_{false};
    double roll_stick_{0.0};
    double rudder_int_{0.0};
    math::Vector3 prev_to_aim_{};
    math::Vector3 los_rate_{};
    bool has_prev_aim_{false};
    double flare_timer_{0.0};
    double threat_cm_timer_{0.0};
    int salvo_left_{0};        ///< Flare programs still to fire in the endgame salvo
    bool salvo_armed_{true};
    bool flare_out_{false};    ///< Flare switch as output last step
    double beam_side_{0.0};    ///< Beam chosen against the missile: +1 right of its line of sight, -1 left, 0 none
    DamageRng rng_{};

    double gaussian() noexcept {
        const double u1 = std::max(1e-12, rng_.uniform());
        const double u2 = rng_.uniform();
        return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * M_PI * u2);
    }

    void record_track(const Aircraft& bandit) noexcept {
        if (last_sample_time_ >= 0.0 && time_ - last_sample_time_ < kTrackPeriod - 1e-9) return;
        last_sample_time_ = time_;
        track_head_ = (track_head_ + 1) % kTrackLen;
        track_[static_cast<size_t>(track_head_)] = {bandit.position(), bandit.velocity(), bandit.acceleration(), time_};
    }

    /// @brief The bandit as seen reaction_s ago, extrapolated to now at constant
    /// velocity: exact for a straight flier, late on every manoeuvre.
    [[nodiscard]] TrackSample perceived_track(const math::Vector3& own_pos) const noexcept {
        const int lag = std::clamp(static_cast<int>(skill.reaction_s / kTrackPeriod + 0.5), 0, kTrackLen - 1);
        int idx = track_head_;
        for (int i = 0; i < lag; ++i) {
            const int prev = (idx + kTrackLen - 1) % kTrackLen;
            if (track_[static_cast<size_t>(prev)].time < 0.0) break;
            idx = prev;
        }
        TrackSample s = track_[static_cast<size_t>(idx)];
        const double age = time_ - s.time;
        s.pos += s.vel * age;
        if (skill.uses_energy && (s.pos - own_pos).norm() < SEE_TURN_RANGE_M) {
            s.pos += s.acc * (0.5 * age * age);
            s.vel += s.acc * age;
        }
        s.time = time_;
        return s;
    }

    [[nodiscard]] static AirCombatGeometry geometry_to(const fdm::FlightState& own, const TrackSample& t) noexcept {
        AirCombatGeometry g;
        g.los_ned = t.pos - own.pos_ned;
        g.range = g.los_ned.norm();
        g.los_b = own.q_att.rotate_ned_to_body(g.los_ned);
        if (g.range < 1e-6) return g;
        const math::Vector3 u = g.los_ned / g.range;
        const math::Vector3 v_own = own.velocity_ned();
        g.closure = -(t.vel - v_own).dot(u);
        g.ata = AirCombatGeometry::angle_between(own.velocity_ned(), u);
        g.aspect = AirCombatGeometry::angle_between(-t.vel, -u);
        g.hca = AirCombatGeometry::angle_between(v_own, t.vel);
        return g;
    }

    [[nodiscard]] static Situation assess(const AirCombatGeometry& geo, double his_ata) noexcept {
        const bool he_points = his_ata < 50.0 * DEG;
        const bool i_point = geo.ata < 60.0 * DEG;
        if (he_points && !i_point && geo.range < 4000.0) return Situation::DEFENSIVE;
        // Offensive: nose on and behind his 3-9 line (or closing on his tail quarter).
        if (i_point && geo.aspect < 110.0 * DEG) return Situation::OFFENSIVE;
        if (i_point && geo.range < 1500.0 && !he_points) return Situation::OFFENSIVE;
        return Situation::NEUTRAL;
    }

    /**
     * @brief Where to steer, level, against a missile on line of sight
     * @p los while its time to go @p tgo is still long: straight away from
     * it (cold) beyond MISSILE_COLD_TGO_S, otherwise onto the beam. The beam
     * nearer the current heading is chosen once and then held: with the
     * missile dead ahead or astern the two are equally far, and a pilot who
     * kept re-choosing would never turn.
     */
    [[nodiscard]] math::Vector3 defensive_heading(const math::Vector3& v_dir, const math::Vector3& los,
                                                  double tgo) noexcept {
        math::Vector3 h{los.x, los.y, 0.0};
        if (h.norm() < 1e-3) h = math::Vector3(v_dir.x, v_dir.y, 0.0); // straight above or below
        if (h.norm() < 1e-3) h = math::Vector3(1.0, 0.0, 0.0);
        h = h.normalized();
        if (tgo > MISSILE_COLD_TGO_S) return -h;
        const math::Vector3 right{-h.y, h.x, 0.0}; // NED: right of the line of sight, level
        const double d = v_dir.x * right.x + v_dir.y * right.y;
        constexpr double SWITCH = 0.5; // flips only once well established toward the other beam
        if (beam_side_ == 0.0) beam_side_ = d >= 0.0 ? 1.0 : -1.0;
        else if (beam_side_ * d < -SWITCH) beam_side_ = -beam_side_;
        return right * beam_side_;
    }

    /**
     * @brief Direction of the endgame break: perpendicular to both the flight
     * path and the missile's line of sight @p los, so that all of the pull
     * turns the line of sight and the missile must match it. Of the two
     * senses, the one nearer the lift vector (the least roll), but never
     * downhill near the hard deck. With the missile dead ahead or astern any
     * pull crosses its line of sight: the lift vector as it is.
     */
    [[nodiscard]] math::Vector3 break_across(const Aircraft& self, const math::Vector3& v_dir,
                                             const math::Vector3& los) const noexcept {
        const math::Vector3 lift = self.state.q_att.rotate_body_to_ned(math::Vector3(0.0, 0.0, -1.0));
        math::Vector3 n = v_dir.cross(los);
        if (n.norm() < 0.05) return lift;
        n = n.normalized();
        double sense = n.dot(lift) >= 0.0 ? 1.0 : -1.0;
        const bool low = environment::GroundCollision::get_agl(self.state) < hard_deck_agl_m + 1500.0;
        if (low && n.z * sense > 0.3) sense = -sense; // NED: +z is down
        return n * sense;
    }

    /**
     * @brief Lift-vector control: roll so the required specific force lies in
     * the lift plane, then pull the G that force needs, closed on measured Nz.
     *
     * The commanded rotation of the flight path is the line-of-sight rate
     * (feed-forward, so a turning target is tracked without a standing error)
     * plus a correction proportional to the aim error. With @p point_nose the
     * error is measured off the gun line rather than the velocity vector,
     * which is what a guns track needs: the nose leads the flight path by alpha.
     */
    void fly_towards(double dt, const Aircraft& self, const aircraft::AircraftConfig& cfg, const math::Vector3& aim,
                     const math::Vector3& bandit_pos, double g_ceiling, bool point_nose,
                     AircraftControls& out) noexcept {
        constexpr double G0 = fdm::SixDoFFDM::GRAVITY_ACCEL;
        const fdm::FlightState& s = self.state;
        const double V = std::max(30.0, s.airspeed());
        const math::Vector3 v = s.velocity_ned();
        const math::Vector3 u = v / std::max(1.0, v.norm());
        const math::Vector3 nose = s.q_att.rotate_body_to_ned(math::Vector3(1.0, 0.0, 0.0));
        const math::Vector3 u_ref = point_nose ? nose : u;
        const math::Vector3 to_aim = (aim - s.pos_ned).normalized();

        // Line-of-sight rate to the bandit himself, low-passed and bounded. The
        // aim point is no use here: it jumps whenever the manoeuvre or the gun
        // solution changes, and its derivative is mostly noise.
        const math::Vector3 los = (bandit_pos - s.pos_ned).normalized();
        if (has_prev_aim_) {
            math::Vector3 w = prev_to_aim_.cross(los) / dt;
            const double wn = w.norm();
            if (wn > 0.6) w = w * (0.6 / wn);
            los_rate_ += (w - los_rate_) * (1.0 - std::exp(-dt / 0.12));
        }
        prev_to_aim_ = los;
        has_prev_aim_ = true;

        aim_error_rad = AirCombatGeometry::angle_between(u_ref, to_aim);
        math::Vector3 axis = u_ref.cross(to_aim);
        const double axis_n = axis.norm();
        if (axis_n > 1e-9) {
            axis = axis / axis_n;
        } else {
            // Dead ahead or dead astern: pull through the lift vector.
            axis = s.q_att.rotate_body_to_ned(math::Vector3(0.0, 1.0, 0.0));
        }
        // A direct-linkage jet responds more slowly: ask for less.
        const bool fbw = cfg.flcs.law_type != aircraft::FLCSConfig::LawType::HYDRO_SAS_AUGMENTED;
        const double k_err = (point_nose ? 2.5 : 1.5) * (fbw ? 1.0 : 0.5);
        const math::Vector3 omega = los_rate_ + axis * std::min(k_err * aim_error_rad, 1.0);

        // Specific force the wing must supply: turn acceleration minus gravity,
        // normal to the flight path.
        math::Vector3 f = omega.cross(u) * V - math::Vector3(0.0, 0.0, G0);
        f = f - u * f.dot(u);
        const math::Vector3 f_b = s.q_att.rotate_ned_to_body(f);

        // Bank to put the lift vector (-Z body) on f.
        const double phi_err = std::atan2(f_b.y, -f_b.z);
        const double p = s.omega_b.x;
        const double p_full = std::max(0.5, cfg.flcs.max_roll_rate_dps * DEG);
        const double p_want = ROLL_BANDWIDTH_RAD_S * phi_err - ROLL_RATE_DAMPING * p;
        const double roll_raw = std::clamp(p_want / p_full, -1.0, 1.0);
        roll_stick_ += (roll_raw - roll_stick_) * (1.0 - std::exp(-dt / 0.06));
        out.stick.roll_stick = roll_stick_;

        // Pull only once the lift vector is roughly placed: an unloaded roll is faster.
        const double align = std::max(0.0, std::cos(phi_err));
        const double nz_req = f.norm() / G0;
        const double g_want = std::clamp(nz_req * align * align, 0.5, g_ceiling);
        // G onset: pilots load the jet at a few G per second, not in a step,
        // and let it off faster than they put it on.
        g_cmd = std::clamp(g_want, g_cmd - G_OFFSET_RATE * dt, g_cmd + G_ONSET_RATE * dt);
        out.stick.pitch_stick = pitch_for_g(dt, self, cfg, g_cmd);

        // Fine tracking: small lateral errors are taken out with rudder
        // ("pedal the pipper") rather than by re-banking the whole jet. Only
        // at low load: pedalling at high G just adds sideslip to the solution.
        out.stick.rudder_pedal = 0.0;
        if (point_nose && aim_error_rad < 3.0 * DEG && g_cmd < 3.0) {
            const math::Vector3 d_b = s.q_att.rotate_ned_to_body(to_aim);
            const double yaw_err = std::atan2(d_b.y, d_b.x);
            rudder_int_ = std::clamp(rudder_int_ + 4.0 * yaw_err * dt, -0.2, 0.2);
            out.stick.rudder_pedal = std::clamp(12.0 * yaw_err + rudder_int_, -0.5, 0.5);
        } else {
            rudder_int_ = 0.0;
        }
    }

    /// @brief Stick for a G command: feed-forward on the jet's stick law,
    /// integral trim on the measured Nz (the same scheme Auto-GCAS uses).
    [[nodiscard]] double pitch_for_g(double dt, const Aircraft& self, const aircraft::AircraftConfig& cfg,
                                     double g_target) noexcept {
        const double nz = self.imu.Nz;
        const double err = g_target - nz;
        const bool fbw = cfg.flcs.law_type != aircraft::FLCSConfig::LawType::HYDRO_SAS_AUGMENTED;
        if (fbw) {
            const double ff = (g_target - 1.0) / std::max(1.0, cfg.flcs.max_g_positive - 1.0);
            nz_trim_ = std::clamp(nz_trim_ + 0.08 * err * dt, -0.15, 0.15);
            return std::clamp(ff + nz_trim_, -0.3, 1.0);
        }
        const auto air = environment::Atmosphere1976::compute(self.state.altitude(), self.state.airspeed());
        const double qsw = air.dynamic_pressure * cfg.aero.s_ref /
                           (std::max(1.0, self.mass.mass_kg) * fdm::SixDoFFDM::GRAVITY_ACCEL);
        const double g_per_stick = std::max(1.0, flcs::OnBoardFlightComputer::GCAS_DIRECT_G_PER_QSW * qsw);
        const double ff = (g_target - 1.0) / g_per_stick;
        const double alpha_over = self.state.alpha() / DEG - (cfg.flcs.alpha_limit_deg - 3.0);
        if (alpha_over < 0.0 || err < 0.0) {
            nz_trim_ = std::clamp(nz_trim_ + (flcs::OnBoardFlightComputer::GCAS_DIRECT_NZ_KI / g_per_stick) * err * dt,
                                  -0.5, 0.5);
        }
        double cmd = ff + (flcs::OnBoardFlightComputer::GCAS_DIRECT_NZ_KP / g_per_stick) * err + nz_trim_;
        // No G law to damp the short period: the pilot does it, easing the
        // stick against pitch rate beyond what the commanded turn needs.
        const double q_turn = (g_target - 1.0) * fdm::SixDoFFDM::GRAVITY_ACCEL / std::max(50.0, self.state.airspeed());
        cmd -= 1.2 * (self.state.omega_b.y - q_turn);
        if (alpha_over > 0.0) cmd -= 0.08 * alpha_over;
        return std::clamp(cmd, -0.3, 1.0);
    }

    void update_trigger(double dt, const Aircraft& self, const AirCombatGeometry& geo, const GunSolution& sol) noexcept {
        (void)self;
        if (cooldown_timer_ > 0.0) {
            cooldown_timer_ -= dt;
            trigger = false;
            return;
        }
        // Training rules: no forward-quarter gun shots (closure makes them
        // near-useless and a collision likely), and a 150 m minimum range.
        const bool in_envelope = geo.range < skill.max_fire_range_m && geo.range > 150.0 &&
                                 geo.aspect < MAX_SHOT_ASPECT;
        const bool on_target = sol.miss_distance < skill.fire_miss_m;
        if (in_envelope && on_target) {
            trigger = true;
            burst_timer_ += dt;
            if (burst_timer_ > 1.0) { // burst discipline: ~1 s bursts
                burst_timer_ = 0.0;
                cooldown_timer_ = 0.4;
                trigger = false;
            }
        } else {
            // Keep the trigger through a short burst once started, if still close.
            trigger = burst_timer_ > 0.0 && burst_timer_ < 0.25 && sol.miss_distance < 2.0 * skill.fire_miss_m;
            if (!trigger) burst_timer_ = 0.0;
            else burst_timer_ += dt;
        }
    }
};

} // namespace fastjet::sim
