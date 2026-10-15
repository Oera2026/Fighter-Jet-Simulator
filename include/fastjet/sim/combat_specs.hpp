#pragma once

#include "fastjet/aircraft/aircraft_type.hpp"
#include "fastjet/math/vector3.hpp"
#include "fastjet/sim/missile.hpp"

namespace fastjet::sim {

/// @brief Missile configuration carried by an airframe.
struct AirframeMissileLoadout {
    MissileSpec mrm_spec{};
    int mrm_count{0};
    MissileSpec srm_spec{};
    int srm_count{0};
};

/// @brief A typical air-to-air load for each jet (open-source configurations).
[[nodiscard]] inline AirframeMissileLoadout airframe_missile_loadout(aircraft::AircraftType type) noexcept {
    using T = aircraft::AircraftType;
    switch (type) {
        case T::F16_FIGHTING_FALCON: // Wingtips and stations 3/7: AMRAAM; 2/8: Sidewinder
            return {missile_specs::aim120c(), 4, missile_specs::aim9x(), 2};
        case T::F15EX_EAGLE_II:      // The Eagle's classic four and four
            return {missile_specs::aim120d(), 4, missile_specs::aim9x(), 4};
        case T::EUROFIGHTER_TYPHOON: // Four Meteor under the fuselage, IRIS-T outboard
            return {missile_specs::meteor(), 4, missile_specs::iris_t(), 2};
        case T::F22_RAPTOR:          // Main bay six, side bays two
            return {missile_specs::aim120d(), 6, missile_specs::aim9x(), 2};
        case T::A10_THUNDERBOLT:     // A self-defence pair on an LAU-105, no radar
            return {MissileSpec{}, 0, missile_specs::aim9m(), 2};
        case T::F35B_LIGHTNING_II:   // Four in the bays, two Sidewinders outboard
            return {missile_specs::aim120d(), 4, missile_specs::aim9x(), 2};
    }
    return {};
}

/// @brief What the pilot has selected to shoot with.
enum class WeaponSel : uint8_t {
    GUN, ///< The gun (the trigger fires it whatever is selected)
    SRM, ///< Short-range missiles: heat seekers
    MRM, ///< Medium-range missiles: active radar
};

[[nodiscard]] constexpr const char* to_string(WeaponSel w) noexcept {
    switch (w) {
        case WeaponSel::GUN: return "GUN";
        case WeaponSel::SRM: return "SRM";
        case WeaponSel::MRM: return "MRM";
    }
    return "?";
}

/// @brief One kind of missile on the rails.
struct MissileStore {
    MissileSpec spec{};
    int count{0};
};

/**
 * @brief A jet's missile stores: a short-range and a medium-range store (either
 * may be empty) and the selected weapon, which the pickle button fires.
 */
struct WeaponStores {
    MissileStore srm{};
    MissileStore mrm{};
    WeaponSel selected{WeaponSel::GUN};

    void clear() noexcept { *this = WeaponStores{}; }

    /// @brief Load what @p l carries: the heat seekers always, the radar
    /// missiles only if @p with_mrm. The gun is selected.
    void load(const AirframeMissileLoadout& l, bool with_mrm) noexcept {
        clear();
        srm = {l.srm_spec, l.srm_count};
        if (with_mrm) mrm = {l.mrm_spec, l.mrm_count};
    }

    [[nodiscard]] MissileStore* store(WeaponSel w) noexcept {
        return w == WeaponSel::SRM ? &srm : w == WeaponSel::MRM ? &mrm : nullptr;
    }
    [[nodiscard]] const MissileStore* store(WeaponSel w) const noexcept {
        return w == WeaponSel::SRM ? &srm : w == WeaponSel::MRM ? &mrm : nullptr;
    }
    /// @brief The selected missile store, or nullptr with the gun selected.
    [[nodiscard]] const MissileStore* selected_store() const noexcept { return store(selected); }
    [[nodiscard]] int missiles() const noexcept { return srm.count + mrm.count; }

    /// @brief Select @p w if it is the gun or a store with rounds left.
    bool select(WeaponSel w) noexcept {
        const MissileStore* s = store(w);
        if (s != nullptr && s->count <= 0) return false;
        selected = w;
        return true;
    }

    /// @brief Weapon select: GUN -> SRM -> MRM -> GUN, skipping empty or unfitted stores.
    WeaponSel cycle() noexcept {
        for (int k = 1; k <= 3; ++k) {
            if (select(static_cast<WeaponSel>((static_cast<int>(selected) + k) % 3))) break;
        }
        return selected;
    }
};

/// @brief Internal gun installation.
///
/// Figures are public-domain nominal values (manufacturer / service fact
/// sheets). Rate and muzzle velocity vary with ammunition lot and rate
/// selection; treat them as representative, not exact.
struct GunSpec {
    const char* name{"M61A1"};
    double rate_rpm{6000.0};          ///< Full-rate cyclic rate
    double muzzle_velocity_mps{1050.0};
    int rounds{511};
    double spin_up_s{0.35};           ///< Time from trigger to full rate (rotary guns)
    double dispersion_mrad{2.5};      ///< 1-sigma radial dispersion
    // Projectile
    double projectile_mass_kg{0.102};
    double calibre_m{0.020};
    double drag_coeff{0.30};          ///< Representative supersonic Cd
    double lethality{1.0};            ///< Damage per hit, 20 mm HEI = 1
    // Installation, body axes from the CG [m]
    math::Vector3 muzzle_b{3.0, -1.0, -0.6};
    double boresight_elev_deg{0.0};   ///< Gun line above the body X axis
};

/// @brief Airframe vulnerability and hit-test geometry.
struct AirframeCombatSpec {
    double length_m{15.06};
    double fuselage_radius_m{0.9};
    double span_m{9.45};
    double wing_radius_m{0.35};
    double structure{8.0};            ///< Accumulated lethality the structure absorbs before failing
};

[[nodiscard]] constexpr GunSpec gun_spec(aircraft::AircraftType type) noexcept {
    using T = aircraft::AircraftType;
    GunSpec g{};
    switch (type) {
        case T::F16_FIGHTING_FALCON:
            g = {"M61A1", 6000.0, 1050.0, 511, 0.35, 2.5, 0.102, 0.020, 0.30, 1.0, {3.0, -1.0, -0.6}, 0.0};
            break;
        case T::F15EX_EAGLE_II:
            g = {"M61A1", 6000.0, 1050.0, 510, 0.35, 2.5, 0.102, 0.020, 0.30, 1.0, {2.5, 1.9, -0.5}, 0.0};
            break;
        case T::F22_RAPTOR:
            g = {"M61A2", 6000.0, 1050.0, 480, 0.30, 2.5, 0.102, 0.020, 0.30, 1.0, {3.0, 1.2, -0.3}, 0.0};
            break;
        case T::EUROFIGHTER_TYPHOON:
            // Single-barrel revolver: no spin-up, full rate almost at once.
            g = {"BK-27", 1700.0, 1025.0, 150, 0.05, 1.5, 0.260, 0.027, 0.28, 1.9, {2.0, 1.0, 0.3}, 0.0};
            break;
        case T::A10_THUNDERBOLT:
            // Firing barrel on the centreline under the nose.
            g = {"GAU-8/A", 3900.0, 1010.0, 1174, 0.55, 2.5, 0.425, 0.030, 0.26, 2.4, {6.5, 0.0, 0.6}, 0.0};
            break;
        case T::F35B_LIGHTNING_II:
            // Four-barrel GAU-22/A in the centreline pod (GD-OTS: 3,300 rds/min,
            // 1,085 m/s), 220 rounds: four seconds of fire.
            g = {"GAU-22/A", 3300.0, 1085.0, 220, 0.25, 2.5, 0.180, 0.025, 0.29, 1.5, {2.5, 0.0, 1.1}, 0.0};
            break;
    }
    return g;
}

[[nodiscard]] constexpr AirframeCombatSpec airframe_combat_spec(aircraft::AircraftType type) noexcept {
    using T = aircraft::AircraftType;
    switch (type) {
        case T::F16_FIGHTING_FALCON: return {15.06, 0.90, 9.45, 0.35, 8.0};
        case T::F15EX_EAGLE_II:      return {19.43, 1.20, 13.05, 0.40, 10.0};
        case T::EUROFIGHTER_TYPHOON: return {15.96, 0.95, 10.95, 0.40, 8.5};
        case T::F22_RAPTOR:          return {18.92, 1.15, 13.56, 0.40, 9.5};
        // Titanium tub, redundant structure and manual reversion: built to absorb hits.
        case T::A10_THUNDERBOLT:     return {16.26, 1.00, 17.53, 0.45, 16.0};
        case T::F35B_LIGHTNING_II:   return {15.60, 1.05, 10.70, 0.40, 9.0};
    }
    return {};
}

} // namespace fastjet::sim
