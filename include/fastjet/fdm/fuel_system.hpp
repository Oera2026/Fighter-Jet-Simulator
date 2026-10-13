#pragma once

#include "mass_properties.hpp"
#include "../aircraft/aircraft_type.hpp"
#include "../aircraft/aircraft_config.hpp"
#include <algorithm>

namespace fastjet {
namespace fdm {

/**
 * @brief Variable-mass model: rebuilds MassProperties as internal fuel is burned.
 *
 * Supports all aircraft types, scaling inertia from the aircraft's empty tensor
 * by the gross-to-empty mass ratio.
 */
class FuelSystem {
public:
    /// F-16C empty weight [kg] (19,700 lb), the default airframe's.
    static constexpr double EMPTY_MASS_KG =
        aircraft::AircraftConfig::get(aircraft::AircraftType::F16_FIGHTING_FALCON).mass.empty_mass_kg;

    /**
     * @brief Builds mass properties for a given fuel load and aircraft type.
     *
     * @param fuel_kg Internal fuel remaining [kg].
     * @param type Aircraft model (defaults to F-16C).
     * @param payload_kg Crew, ammunition and stores carried [kg].
     * @return MassProperties with gross mass and scaled inertia tensor.
     */
    [[nodiscard]] static MassProperties compute(
        double fuel_kg,
        aircraft::AircraftType type = aircraft::AircraftType::F16_FIGHTING_FALCON,
        double payload_kg = 0.0
    ) noexcept {
        const auto cfg = aircraft::AircraftConfig::get(type);
        const double fuel = std::max(0.0, fuel_kg);
        const double empty_mass = cfg.mass.empty_mass_kg;
        const double gross_mass = empty_mass + fuel + std::max(0.0, payload_kg);

        // Empty-airframe inertia
        const MassProperties empty = MassProperties::create(type);

        // Scale the tensor by the mass ratio.
        const double ratio = (empty_mass > 0.0)
                           ? (gross_mass / empty_mass)
                           : 1.0;

        return MassProperties(
            gross_mass,
            empty.Ixx * ratio,
            empty.Iyy * ratio,
            empty.Izz * ratio,
            empty.Ixz * ratio
        );
    }
};

} // namespace fdm
} // namespace fastjet
