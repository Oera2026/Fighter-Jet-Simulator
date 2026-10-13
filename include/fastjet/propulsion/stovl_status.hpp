#pragma once

#include <cstdint>

namespace fastjet::propulsion {

/// @brief F-35B propulsion configuration as selected by the STOVL/CTOL mode button.
enum class StovlMode : std::uint8_t {
    CTOL,                ///< Up-and-away: doors shut, LiftFan declutched, 3BSM aft
    CONVERTING_TO_STOVL, ///< Doors opening, clutch engaging, nozzle rotating down
    STOVL,               ///< Jet-borne / semi-jet-borne: lift system powered
    CONVERTING_TO_CTOL   ///< Nozzle aft, clutch released, doors closing
};

[[nodiscard]] constexpr const char* to_string(StovlMode m) noexcept {
    switch (m) {
        case StovlMode::CTOL:                return "CTOL";
        case StovlMode::CONVERTING_TO_STOVL: return "CONV STOVL";
        case StovlMode::STOVL:               return "STOVL";
        case StovlMode::CONVERTING_TO_CTOL:  return "CONV CTOL";
    }
    return "";
}

/// @brief Throttle-lever position of the STOVL inceptor's hold detent.
///
/// Once the unified control law is flying the jet on the lift system, the
/// lever no longer sets thrust: it commands longitudinal acceleration (semi-
/// jet-borne) or ground speed (hover) relative to this point. The viewer snaps
/// keyboard throttle here on conversion; the control law reads it.
inline constexpr double kStovlThrottleDetent = 0.5;

/// @brief Read-only snapshot of the STOVL lift system, for the cockpit,
/// airframe animation, exhaust effects and audio.
///
/// Produced by the flight model each step; every consumer only reads it.
/// Angles are degrees, thrusts newtons, door positions 0 shut .. 1 open.
struct StovlStatus {
    bool equipped = false;             ///< Airframe has a lift system (F-35B only)
    StovlMode mode = StovlMode::CTOL;
    double conversion = 0.0;           ///< 0 CTOL configuration .. 1 fully converted to STOVL

    double lift_fan_door = 0.0;        ///< Rear-hinged LiftFan door above the fan
    double aux_inlet_door = 0.0;       ///< Auxiliary engine inlet doors behind the fan
    double fan_nozzle_door = 0.0;      ///< Lower LiftFan doors (vane box exit)
    double roll_post_door = 0.0;       ///< Roll post doors under both wings
    double clutch = 0.0;               ///< LiftFan clutch engagement 0..1

    double nozzle_pitch_deg = 0.0;     ///< 3BSM deflection: 0 aft, 90 straight down, max ~95
    double nozzle_yaw_deg = 0.0;       ///< 3BSM lateral vectoring for yaw, + nozzle swung right
    double vane_box_deg = 0.0;         ///< LiftFan thrust vector aft of vertical

    double lift_fan_thrust_n = 0.0;
    double nozzle_thrust_n = 0.0;
    double roll_post_left_n = 0.0;
    double roll_post_right_n = 0.0;
    double jet_lift_n = 0.0;           ///< Sum of lift-system thrust along the body -Z axis
    double hover_thrust_margin = 0.0;  ///< Max jet lift / weight (> 1 can hover)

    /// Unified control law phase, for HUD/PCD cueing:
    /// 0 conventional (wing-borne), 1 semi-jet-borne, 2 jet-borne (hover).
    std::uint8_t control_phase = 0;
    double commanded_vs_mps = 0.0;     ///< Vertical speed the law is flying to (+ up)
    double commanded_gs_mps = 0.0;     ///< Ground speed (forward) the law is flying to
    bool conversion_inhibited = false; ///< Selection refused: above 250 KCAS, or too slow to go wing-borne
};

} // namespace fastjet::propulsion
