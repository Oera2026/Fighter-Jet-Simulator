#pragma once

#include "fastjet/aircraft/aircraft_type.hpp"
#include "fastjet/environment/atmosphere1976.hpp"
#include "fastjet/math/vector3.hpp"
#include "fastjet/sim/air_combat_geometry.hpp"
#include "fastjet/sim/aircraft.hpp"
#include "fastjet/sim/missile.hpp"
#include "fastjet/sim/radar.hpp"
#include <algorithm>
#include <cmath>

namespace fastjet::sim {

/**
 * @file infrared.hpp
 * @brief Infrared signatures, and the heat-seeker head a jet uncages before launch.
 *
 * A jet's IR signature is given relative to a single-engine fighter seen from
 * dead astern at military power (signature 1). It is the product of three
 * factors:
 *
 *  - Aspect: from the tail a seeker sees the hot nozzle, the turbine face
 *    behind it and the core of the plume; from the beam only the plume and
 *    the warm aft fuselage; head-on only the plume beyond the jet and the
 *    skin. Strongest from the tail, much weaker from the beam, weakest head-on.
 *  - Engine: hot-part temperature and plume size rise steeply with power
 *    (radiance goes as T^4), so idle is low, military power medium, and an
 *    afterburner's long, far hotter plume high.
 *  - Airframe: engine size and count and how well the nozzles are shielded
 *    and cooled.
 *
 * A seeker's lock range against it is MissileSpec::ir_lock_range():
 * ir_lock_range_m * sqrt(signature), since a point source's irradiance falls
 * as 1/R^2.
 *
 * All of the numbers here are estimates chosen to rank the cases sensibly.
 * Real signatures are classified and depend on waveband, altitude, speed
 * (skin heating) and the background.
 */

/// @brief Airframe factor: engine size and count, nozzle shielding (estimates).
[[nodiscard]] constexpr double ir_airframe_factor(aircraft::AircraftType type) noexcept {
    using T = aircraft::AircraftType;
    switch (type) {
        case T::F16_FIGHTING_FALCON: return 1.0;  // One F110: the reference
        case T::EUROFIGHTER_TYPHOON: return 1.0;  // Two smaller EJ200s: about the same
        case T::F15EX_EAGLE_II:      return 1.4;  // Two big F110-129s side by side
        case T::F22_RAPTOR:          return 0.4;  // 2-D nozzles, shielded and cooled
        case T::A10_THUNDERBOLT:     return 0.5;  // High-bypass TF34s, cool exhaust, masked by the tail
        case T::F35B_LIGHTNING_II:   return 0.7;  // One hot F135, but a low-observable nozzle
    }
    return 1.0;
}

/**
 * @brief Aspect factor from the angle between the target's tail (its -X body
 * axis) and the line from the target to the seeker: 0 dead astern, pi head-on.
 * Piecewise linear through estimated points.
 */
[[nodiscard]] inline double ir_aspect_factor(double off_tail_rad) noexcept {
    struct P { double deg, k; };
    static constexpr P table[] = {{0.0, 1.00}, {30.0, 0.85}, {60.0, 0.50}, {90.0, 0.22},
                                  {120.0, 0.12}, {150.0, 0.09}, {180.0, 0.07}};
    constexpr int n = static_cast<int>(sizeof(table) / sizeof(table[0]));
    const double deg = std::clamp(off_tail_rad * 180.0 / M_PI, 0.0, 180.0);
    for (int i = 1; i < n; ++i) {
        if (deg <= table[i].deg) {
            const double t = (deg - table[i - 1].deg) / (table[i].deg - table[i - 1].deg);
            return table[i - 1].k + t * (table[i].k - table[i - 1].k);
        }
    }
    return table[n - 1].k;
}

/// Engine factor at idle, military power and full reheat (estimates).
inline constexpr double IR_ENGINE_IDLE = 0.3;
inline constexpr double IR_ENGINE_MIL = 1.0;
inline constexpr double IR_ENGINE_MAX_AB = 3.0;

/**
 * @brief Engine factor from how hard the engine is working: @p core_fraction
 * is core thrust over installed military thrust, @p ab_fraction the share of
 * the afterburner's increment that is lit. Square law on the core (hot-part
 * temperature and plume both grow with power), linear in reheat.
 */
[[nodiscard]] inline double ir_engine_factor(double core_fraction, double ab_fraction) noexcept {
    const double c = std::clamp(core_fraction, 0.0, 1.0);
    const double a = std::clamp(ab_fraction, 0.0, 1.0);
    return IR_ENGINE_IDLE + (IR_ENGINE_MIL - IR_ENGINE_IDLE) * c * c + (IR_ENGINE_MAX_AB - IR_ENGINE_MIL) * a;
}

/// @brief The engine factor of @p jet now, read from its propulsion model
/// (spooled core and afterburner thrust against their installed ratings).
[[nodiscard]] inline double ir_engine_factor(const Aircraft& jet) noexcept {
    const auto& e = jet.engine;
    const auto air = environment::Atmosphere1976::compute(jet.state.altitude(), jet.state.airspeed());
    const double mil = e.config.static_dry_thrust_n * e.lapse_factor(air, false);
    const double core = e.core_thrust_n / std::max(1.0, mil);
    double ab = 0.0;
    if (e.config.has_afterburner) {
        const double ab_span = e.config.static_ab_thrust_n * e.lapse_factor(air, true) - mil;
        ab = e.ab_thrust_n / std::max(1.0, ab_span);
    }
    return ir_engine_factor(core, ab);
}

/// @brief Aspect off the tail of @p jet as seen from @p viewer_pos [rad].
[[nodiscard]] inline double ir_off_tail_angle(const Aircraft& jet, const math::Vector3& viewer_pos) noexcept {
    const math::Vector3 tail = jet.state.q_att.rotate_body_to_ned(math::Vector3(-1.0, 0.0, 0.0));
    return AirCombatGeometry::angle_between(tail, viewer_pos - jet.position());
}

/// @brief IR signature of @p jet seen from @p viewer_pos (1 = single-engine
/// fighter's tail at military power).
[[nodiscard]] inline double ir_signature(const Aircraft& jet, const math::Vector3& viewer_pos) noexcept {
    return ir_airframe_factor(jet.type) * ir_aspect_factor(ir_off_tail_angle(jet, viewer_pos)) *
           ir_engine_factor(jet);
}

/**
 * @brief A heat-seeking missile's seeker head on the rail, as the launching
 * jet's avionics see it: caged off, searching, or locked on a target.
 *
 * It is active while a heat-seeker with rounds left is the selected weapon.
 * Searching, it looks along the nose, or down the radar's line of sight when
 * the radar holds a single-target track inside the seeker's gimbal
 * (slaved). It locks the brightest jet inside its search cone, inside its
 * gimbal and inside its signature-scaled lock range. Locked, it tracks the
 * target anywhere in the gimbal, and drops the lock outside the gimbal or
 * beyond its track range (lock range x MissileSpec::IR_TRACK_MARGIN).
 *
 * The search cone is the pilot's cueing: a few degrees around the HUD for a
 * pilot without a helmet sight, wider for an AI pilot by skill (an ace with a
 * helmet sight can cue the seeker anywhere in its gimbal).
 */
struct IrSeeker {
    static constexpr double DEFAULT_SEARCH_DEG = 15.0; ///< Uncaged search around the HUD bore (estimate)
    static constexpr double SLAVED_SEARCH_DEG = 3.0;   ///< Slaved: a few degrees around the radar's LOS

    bool active{false};
    bool locked{false};
    bool slaved{false};          ///< Looking down the radar's line of sight
    int target{-1};
    math::Vector3 dir_ned{1.0, 0.0, 0.0}; ///< Where the seeker looks (unit, NED)
    double signature{0.0};       ///< Target signature seen now (locked)
    double lock_range_m{0.0};    ///< Lock range against that signature
    double lock_time_s{0.0};     ///< Continuous time locked on the current target
    double search_half_deg{DEFAULT_SEARCH_DEG};

    void cage() noexcept {
        active = locked = slaved = false;
        target = -1;
        signature = lock_range_m = lock_time_s = 0.0;
    }

    /**
     * @brief One step of the seeker of jet @p self (index into @p jets).
     * @param spec  The heat-seeker on the rail.
     * @param radar The jet's fire-control radar (for slaving).
     */
    void update(double dt, int self, const Aircraft* jets, int count, const MissileSpec& spec,
                const FireControlRadar& radar) noexcept {
        const Aircraft& own = jets[self];
        const math::Vector3 pos = own.position();
        const math::Vector3 nose = own.state.q_att.rotate_body_to_ned(math::Vector3(1.0, 0.0, 0.0));
        const double gimbal = spec.seeker_gimbal_deg * M_PI / 180.0;
        active = true;

        if (locked) {
            const bool valid = target >= 0 && target < count && target != self && !jets[target].crashed;
            if (valid) {
                const math::Vector3 los = jets[target].position() - pos;
                const double r = los.norm();
                signature = ir_signature(jets[target], pos);
                lock_range_m = spec.ir_lock_range(signature);
                if (r > 1.0 && AirCombatGeometry::angle_between(nose, los) <= gimbal &&
                    r <= MissileSpec::IR_TRACK_MARGIN * lock_range_m) {
                    dir_ned = los / r;
                    lock_time_s += dt;
                    return;
                }
            }
            locked = false;
            target = -1;
            lock_time_s = 0.0;
        }

        // Search: down the radar's line of sight when it holds a track the
        // seeker can see, otherwise along the nose.
        math::Vector3 look = nose;
        slaved = false;
        if (radar.locked() && radar.target >= 0 && radar.target < count && radar.target != self) {
            const math::Vector3 rl = radar.track_pos - pos;
            if (rl.norm() > 1.0 && AirCombatGeometry::angle_between(nose, rl) <= gimbal) {
                look = rl.normalized();
                slaved = true;
            }
        }
        const double cone = (slaved ? SLAVED_SEARCH_DEG : search_half_deg) * M_PI / 180.0;
        double best = 0.0;
        for (int j = 0; j < count; ++j) {
            if (j == self || !jets[j].alive()) continue;
            const math::Vector3 los = jets[j].position() - pos;
            const double r = los.norm();
            if (r < 1.0 || AirCombatGeometry::angle_between(look, los) > cone) continue;
            if (AirCombatGeometry::angle_between(nose, los) > gimbal) continue;
            const double sig = ir_signature(jets[j], pos);
            const double lock_r = spec.ir_lock_range(sig);
            if (r > lock_r) continue;
            const double irradiance = sig / (r * r); // The brightest source wins the seeker
            if (irradiance > best) {
                best = irradiance;
                target = j;
                signature = sig;
                lock_range_m = lock_r;
                dir_ned = los / r;
            }
        }
        locked = target >= 0;
        lock_time_s = 0.0;
        if (!locked) {
            dir_ned = look;
            signature = lock_range_m = 0.0;
        }
    }
};

} // namespace fastjet::sim
