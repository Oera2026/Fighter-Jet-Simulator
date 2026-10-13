#pragma once

#include <string_view>
#include <string>

namespace fastjet::aircraft {

/**
 * @brief Supported aircraft models in the Fast Jet Flight Simulator.
 */
enum class AircraftType {
    F16_FIGHTING_FALCON,   ///< Baseline F-16C Block 50 multirole single-engine FBW fighter
    F15EX_EAGLE_II,        ///< Boeing F-15EX Eagle II twin F110-GE-129 digital FBW heavy fighter
    EUROFIGHTER_TYPHOON,   ///< Eurofighter Typhoon twin EJ200 delta-canard carefree FBW fighter
    F22_RAPTOR,            ///< Lockheed Martin F-22A Raptor 5th-gen 2D TVC air dominance fighter
    A10_THUNDERBOLT,       ///< Fairchild Republic A-10C Warthog twin TF34 high-bypass CAS attack jet
    F35B_LIGHTNING_II      ///< Lockheed Martin F-35B STOVL: F135-PW-600 + Rolls-Royce LiftSystem
};

[[nodiscard]] constexpr std::string_view to_string(AircraftType type) noexcept {
    switch (type) {
        case AircraftType::F16_FIGHTING_FALCON: return "F-16C Fighting Falcon";
        case AircraftType::F15EX_EAGLE_II:      return "F-15EX Eagle II";
        case AircraftType::EUROFIGHTER_TYPHOON: return "Eurofighter Typhoon";
        case AircraftType::F22_RAPTOR:          return "F-22A Raptor";
        case AircraftType::A10_THUNDERBOLT:     return "A-10C Thunderbolt II";
        case AircraftType::F35B_LIGHTNING_II:   return "F-35B Lightning II";
    }
    return "Unknown Aircraft";
}

/// @brief Short designation for cockpit displays, where panel space is tight.
/// The HUD and MFD have room for a type code, not a full marketing name.
[[nodiscard]] constexpr std::string_view to_short_string(AircraftType type) noexcept {
    switch (type) {
        case AircraftType::F16_FIGHTING_FALCON: return "F-16C";
        case AircraftType::F15EX_EAGLE_II:      return "F-15EX";
        case AircraftType::EUROFIGHTER_TYPHOON: return "TYPHOON";
        case AircraftType::F22_RAPTOR:          return "F-22A";
        case AircraftType::A10_THUNDERBOLT:     return "A-10C";
        case AircraftType::F35B_LIGHTNING_II:   return "F-35B";
    }
    return "UNKNOWN";
}

/// @brief Installed powerplant designation, for the MFD engine page header.
[[nodiscard]] constexpr std::string_view engine_designation(AircraftType type) noexcept {
    switch (type) {
        case AircraftType::F16_FIGHTING_FALCON: return "F110-GE-129";
        case AircraftType::F15EX_EAGLE_II:      return "2X F110-GE-129";
        case AircraftType::EUROFIGHTER_TYPHOON: return "2X EJ200";
        case AircraftType::F22_RAPTOR:          return "2X F119-PW-100";
        case AircraftType::A10_THUNDERBOLT:     return "2X TF34-GE-100A";
        case AircraftType::F35B_LIGHTNING_II:   return "F135-PW-600";
    }
    return "UNKNOWN";
}

/// @brief One line of hard numbers that differ between airframes.
///
/// Shown on the switch confirmation banner so the pilot can verify the change
/// reached the flight model, not merely the name plate.
[[nodiscard]] constexpr std::string_view signature_spec(AircraftType type) noexcept {
    switch (type) {
        case AircraftType::F16_FIGHTING_FALCON: return "1 ENG  9.0G  M2.05";
        case AircraftType::F15EX_EAGLE_II:      return "2 ENG  9.0G  M2.50";
        case AircraftType::EUROFIGHTER_TYPHOON: return "2 ENG  9.0G  SUPERCRUISE M1.50";
        case AircraftType::F22_RAPTOR:          return "2 ENG  9.0G  SUPERCRUISE M1.82  TVC";
        case AircraftType::A10_THUNDERBOLT:     return "2 ENG  7.33G  M0.56  NO AB";
        case AircraftType::F35B_LIGHTNING_II:   return "1 ENG  7.0G  M1.60  STOVL";
    }
    return "";
}

[[nodiscard]] inline AircraftType parse_aircraft_type(std::string_view str) noexcept {
    if (str == "f15" || str == "f15ex" || str == "f-15" || str == "f-15ex") {
        return AircraftType::F15EX_EAGLE_II;
    }
    if (str == "typhoon" || str == "eurofighter" || str == "ef2000") {
        return AircraftType::EUROFIGHTER_TYPHOON;
    }
    if (str == "f22" || str == "f-22" || str == "raptor") {
        return AircraftType::F22_RAPTOR;
    }
    if (str == "a10" || str == "a-10" || str == "warthog" || str == "thunderbolt") {
        return AircraftType::A10_THUNDERBOLT;
    }
    if (str == "f35" || str == "f35b" || str == "f-35" || str == "f-35b" || str == "lightning") {
        return AircraftType::F35B_LIGHTNING_II;
    }
    return AircraftType::F16_FIGHTING_FALCON;
}

} // namespace fastjet::aircraft
