#pragma once

#include "fastjet/aero/control_surfaces.hpp"
#include "fastjet/aircraft/aircraft_config.hpp"
#include "fastjet/aircraft/aircraft_type.hpp"
#include "fastjet/math/vector3.hpp"
#include "fastjet/sim/combat_specs.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace fastjet::sim {

/// @brief Where on the airframe a round struck.
enum class HitZone : uint8_t { COCKPIT, FUSELAGE, ENGINE, WING_LEFT, WING_RIGHT, TAIL };

[[nodiscard]] constexpr const char* to_string(HitZone z) noexcept {
    switch (z) {
        case HitZone::COCKPIT:    return "COCKPIT";
        case HitZone::FUSELAGE:   return "FUSELAGE";
        case HitZone::ENGINE:     return "ENGINE";
        case HitZone::WING_LEFT:  return "LEFT WING";
        case HitZone::WING_RIGHT: return "RIGHT WING";
        case HitZone::TAIL:       return "TAIL";
    }
    return "?";
}

/// @brief Zone from a hit point in body axes (X forward). The hull is split
/// along its length; anything well outboard of the fuselage is wing.
[[nodiscard]] inline HitZone classify_hit(const math::Vector3& p_b, const AirframeCombatSpec& a) noexcept {
    if (std::abs(p_b.y) > a.fuselage_radius_m * 1.3) {
        return p_b.y < 0.0 ? HitZone::WING_LEFT : HitZone::WING_RIGHT;
    }
    const double half = 0.5 * a.length_m;
    const double x = p_b.x / half; // -1 tail .. +1 nose
    if (x > 0.45) return HitZone::COCKPIT;
    if (x > -0.10) return HitZone::FUSELAGE;
    if (x > -0.65) return HitZone::ENGINE;
    return HitZone::TAIL;
}

/// @brief Small deterministic generator for damage rolls (per engagement seed).
struct DamageRng {
    uint32_t state{0x9E3779B9u};
    double uniform() noexcept {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return static_cast<double>(state) / 4294967296.0;
    }
};

/**
 * @brief Component damage. Hits do not drain a health bar: they break the
 * things the flight model already has, so a damaged jet flies damaged.
 *
 *  - Engines lose thrust and can catch fire; a fire that burns long enough
 *    destroys the aircraft. Twin-engine jets survive losing one.
 *  - Fuel hits open a leak that drains the tanks (and moves the mass).
 *  - Wing and tail hits cut control-surface authority; a wing or the
 *    structure as a whole can fail outright.
 *  - Cockpit hits can incapacitate the pilot.
 *  - Over-G: past its rated load factor the airframe is overstressed; past
 *    YIELD_FACTOR of it the wings take a permanent set (less roll authority,
 *    a weaker structure); at ULTIMATE_FACTOR the structure fails outright.
 *
 * With no hits taken and no yield every effect is an exact identity, so an
 * undamaged jet flies bit-identically to one without a damage model.
 */
struct DamageState {
    static constexpr double ENGINE_DAMAGE_PER_HIT = 0.45;
    static constexpr double ENGINE_FIRE_CHANCE    = 0.25;
    static constexpr double FIRE_BURN_THROUGH_S   = 12.0;
    static constexpr double LEAK_PER_HIT_KGS      = 1.5;
    static constexpr double WING_DAMAGE_PER_HIT   = 0.22;
    static constexpr double TAIL_DAMAGE_PER_HIT   = 0.25;
    static constexpr double PILOT_HIT_CHANCE      = 0.30;
    // Structural load limits, as multiples of the rated load factor. Design
    // practice: no permanent deformation at limit load, failure at 1.5x it.
    static constexpr double OVERSTRESS_FACTOR     = 1.03;  ///< Logged as an overstress (inspection)
    static constexpr double YIELD_FACTOR          = 1.20;  ///< Permanent set begins
    static constexpr double ULTIMATE_FACTOR       = 1.50;  ///< Structural failure
    /// The load is filtered this much, so a single integration step's spike
    /// does not count as a sustained structural load [s].
    static constexpr double LOAD_FILTER_TAU_S     = 0.05;

    std::array<double, 2> engine_health{1.0, 1.0};
    int engine_count{1};
    bool engine_fire{false};
    double fire_time{0.0};
    double fuel_leak_kgs{0.0};
    double wing_left{1.0};
    double wing_right{1.0};
    double tail{1.0};
    double structure_used{0.0};
    double structure_capacity{8.0};
    bool pilot_incapacitated{false};
    bool pilot_ejected{false};      ///< The cockpit is empty: the handle has been pulled
    bool destroyed{false};
    int hits_taken{0};
    // Over-G
    double g_limit_pos{9.0};
    double g_limit_neg{-3.0};
    double load_nz{1.0};            ///< Filtered structural load factor
    double peak_load_ratio{0.0};    ///< Highest load / rated limit seen
    double strength{1.0};           ///< Share of the ultimate strength left after yielding
    bool overstressed{false};       ///< The rated limit has been exceeded
    bool yielded{false};            ///< The wings have taken a permanent set
    bool structural_failure{false}; ///< Broken by load, not by the enemy

    DamageState() noexcept = default;

    explicit DamageState(aircraft::AircraftType type) noexcept {
        const auto cfg = aircraft::AircraftConfig::get(type);
        engine_count = cfg.propulsion.engine_count;
        structure_capacity = airframe_combat_spec(type).structure;
        g_limit_pos = cfg.flcs.max_g_positive;
        g_limit_neg = cfg.flcs.max_g_negative;
    }

    [[nodiscard]] bool damaged() const noexcept { return hits_taken > 0 || yielded; }

    /// @brief Load as a multiple of the rated limit in its own direction.
    [[nodiscard]] double load_ratio(double nz) const noexcept {
        return nz >= 0.0 ? nz / g_limit_pos : nz / g_limit_neg;
    }

    /// @brief Apply the airframe's structural load factor for one step.
    /// Returns true on the step the structure fails.
    bool apply_load(double nz, double dt) noexcept {
        if (destroyed || dt <= 0.0) return false;
        load_nz += (nz - load_nz) * (1.0 - std::exp(-dt / LOAD_FILTER_TAU_S));
        const double ratio = load_ratio(load_nz);
        peak_load_ratio = std::max(peak_load_ratio, ratio);
        if (ratio > OVERSTRESS_FACTOR) overstressed = true;
        if (ratio >= ULTIMATE_FACTOR * strength) {
            structural_failure = true;
            destroyed = true;
            return true;
        }
        if (ratio > YIELD_FACTOR) {
            // Permanent set grows with the load past yield. A bent wing rolls
            // less well, and a yielded structure breaks at a lower load.
            const double set = std::clamp((ratio - YIELD_FACTOR) / (ULTIMATE_FACTOR - YIELD_FACTOR), 0.0, 1.0);
            yielded = true;
            const double wing = 1.0 - 0.5 * set;
            wing_left = std::min(wing_left, wing);
            wing_right = std::min(wing_right, wing);
            strength = std::min(strength, 1.0 - 0.25 * set);
        }
        return false;
    }

    /// @brief Thrust multiplier across the installed engines.
    [[nodiscard]] double thrust_factor() const noexcept {
        if (destroyed) return 0.0;
        if (!damaged()) return 1.0;
        double sum = 0.0;
        for (int i = 0; i < engine_count; ++i) sum += engine_health[static_cast<size_t>(i)];
        return sum / static_cast<double>(std::max(1, engine_count));
    }

    /// @brief Scale surface deflections by what is left of each surface.
    /// A shot-up wing loses lift on one side, felt as a roll off toward it.
    void apply_to_surfaces(aero::ControlSurfaces& s) const noexcept {
        if (!damaged()) return;
        const double roll_auth = 0.5 * (wing_left + wing_right);
        s.delta_a *= roll_auth;
        s.delta_a += 6.0 * (wing_right - wing_left); // [deg] asymmetric lift trim
        s.delta_e *= 0.35 + 0.65 * tail;
        s.delta_r *= tail;
    }

    /// @brief Advance fires. Returns true on the step the aircraft is destroyed.
    bool update(double dt) noexcept {
        if (destroyed || !engine_fire) return false;
        fire_time += dt;
        for (int i = 0; i < engine_count; ++i) {
            auto& h = engine_health[static_cast<size_t>(i)];
            h = std::max(0.0, h - 0.04 * dt);
        }
        if (fire_time >= FIRE_BURN_THROUGH_S) {
            destroyed = true;
            return true;
        }
        return false;
    }

    /// @brief Apply one round's hit. Returns true if this hit destroyed the aircraft.
    bool apply_hit(HitZone zone, double lethality, DamageRng& rng) noexcept {
        if (destroyed) return false;
        ++hits_taken;
        structure_used += lethality * (zone == HitZone::WING_LEFT || zone == HitZone::WING_RIGHT ? 0.5 : 1.0);

        switch (zone) {
            case HitZone::COCKPIT:
                if (rng.uniform() < PILOT_HIT_CHANCE * lethality) pilot_incapacitated = true;
                break;
            case HitZone::FUSELAGE:
                fuel_leak_kgs += LEAK_PER_HIT_KGS * lethality;
                break;
            case HitZone::ENGINE: {
                // Twin installations: the round finds one engine or the other.
                const size_t e = (engine_count > 1 && rng.uniform() < 0.5) ? 1 : 0;
                engine_health[e] = std::max(0.0, engine_health[e] - ENGINE_DAMAGE_PER_HIT * lethality);
                if (rng.uniform() < ENGINE_FIRE_CHANCE * lethality) engine_fire = true;
                break;
            }
            case HitZone::WING_LEFT:
                wing_left = std::max(0.0, wing_left - WING_DAMAGE_PER_HIT * lethality);
                fuel_leak_kgs += 0.5 * LEAK_PER_HIT_KGS * lethality; // wet wing
                break;
            case HitZone::WING_RIGHT:
                wing_right = std::max(0.0, wing_right - WING_DAMAGE_PER_HIT * lethality);
                fuel_leak_kgs += 0.5 * LEAK_PER_HIT_KGS * lethality;
                break;
            case HitZone::TAIL:
                tail = std::max(0.0, tail - TAIL_DAMAGE_PER_HIT * lethality);
                break;
        }

        if (structure_used >= structure_capacity || wing_left <= 0.0 || wing_right <= 0.0) {
            destroyed = true;
            return true;
        }
        return false;
    }
};

} // namespace fastjet::sim
