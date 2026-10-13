#pragma once

#include "fastjet/aircraft/aircraft_type.hpp"
#include "fastjet/propulsion/stovl_status.hpp"
#include <cstdint>

namespace fastjet::graphics {

/// @brief Dogfight symbology: target track, gunsight, weapons and the fight's state.
/// Directions are unit vectors in the ownship's body axes (X nose, Y right wing, Z belly).
struct CombatTelemetry {
    bool active = false;              ///< Dogfight mode running (draws the combat HUD)
    bool target_valid = false;        ///< Bandit tracked
    double target_dir_b[3] = {1.0, 0.0, 0.0};
    double target_range_m = 0.0;
    double closure_mps = 0.0;
    double aspect_deg = 0.0;
    double ata_deg = 0.0;
    bool pipper_valid = false;        ///< Director gunsight solution available
    double pipper_dir_b[3] = {1.0, 0.0, 0.0};
    double gun_max_range_m = 1200.0;  ///< Outer edge of the range ring
    bool in_gun_range = false;
    int ammo = 0;
    bool gun_firing = false;
    char gun_name[12] = "";
    double turn_rate_dps = 0.0;
    double ps_mps = 0.0;
    int hits_scored = 0;
    int hits_taken = 0;
    double hit_cue_s = 0.0;           ///< "HIT" flash time remaining
    bool engine_damage = false;
    bool fuel_leak = false;
    bool engine_fire = false;
    bool control_damage = false;
    char status[64] = "";             ///< Fight description (skill, set-up)
    char banner[40] = "";             ///< Outcome, empty while the fight is on
    char debrief[96] = "";            ///< One-line score under the banner
    char debrief2[96] = "";           ///< Second debrief line (evade: missiles and chaff)

    // Radar warning receiver and countermeasures. Bearings are relative
    // azimuths: 0 on the nose, positive clockwise, degrees.
    bool rwr_active = false;          ///< Draw the RWR azimuth display
    int rwr_level = 0;                ///< 0 clear, 1 search (painted), 2 lock, 3 missile launch
    bool rwr_emitter_valid = false;
    double rwr_emitter_bearing_deg = 0.0;
    char rwr_symbol[4] = "";          ///< Emitter type, e.g. "16"
    bool rwr_missile_valid = false;
    double rwr_missile_bearing_deg = 0.0;
    double rwr_missile_range_m = 0.0;
    // Missile approach warning system (MAWS): sees missiles the RWR cannot
    // hear (heat seekers), by the motor's plume or with its own doppler
    // radar. Bearing as the RWR's; elevation positive above the wing plane.
    bool maws_fitted = false;         ///< The jet carries a missile approach warner
    bool maws_threat = false;         ///< A declared missile is closing on the jet
    int maws_threat_count = 0;
    int maws_clock = 12;              ///< Most urgent threat's clock position (12 on the nose)
    double maws_bearing_deg = 0.0;
    double maws_elevation_deg = 0.0;
    double maws_time_to_go_s = 0.0;   ///< Estimated time to impact
    bool maws_range_valid = false;    ///< Range measured (pulse-doppler MAW only)
    double maws_range_m = 0.0;
    unsigned maws_launch_count = 0;   ///< Threats declared so far: a rise cues the launch tone once
    int chaff = -1;                   ///< Cartridges left (-1: no dispenser shown)
    int flares = -1;                  ///< Flares left (-1: no dispenser shown)
    bool blink = false;               ///< Warning flash phase (~3 Hz)

    // Weapons employment & DLZ
    int selected_weapon = 0;          ///< 0: GUN, 1: IR (AIM-9X), 2: RADAR (AIM-120C)
    char weapon_name[16] = "GUN";
    int missiles_remaining = 0;
    bool missile_locked = false;      ///< Weapon has valid lock
    bool shoot_cue = false;           ///< Target within DLZ / in parameters
    double dlz_rmax_m = 0.0;          ///< Maximum kinematic range
    double dlz_rne_m = 0.0;           ///< No-escape range against manoeuvring target
    double dlz_rmin_m = 0.0;          ///< Minimum arming/turn range
    bool ir_seeker_active = false;    ///< Uncaged heat-seeker looking
    double ir_seeker_dir_b[3] = {1.0, 0.0, 0.0}; ///< Seeker bore/track vector
    bool ir_seeker_locked = false;    ///< Heat lock acquired
};

/// @brief The fire-control radar (FCR) page: a B-scope of azimuth against range.
/// Azimuths are from the ownship's heading in the stabilised scan frame,
/// positive right, degrees.
struct RadarTelemetry {
    static constexpr int kMaxHits = 32;
    struct Hit {
        float az_deg = 0.0f;
        float range_nm = 0.0f;
        int age = 0;                  ///< Scan frames since the paint (0: this frame)
    };

    bool page_fcr = false;            ///< Left MFD shows FCR (else the FLCS page)
    bool fitted = false;              ///< The airframe carries an air-to-air radar
    int mode = 0;                     ///< 0 off, 1 RWS, 2 STT, 3 ACM search
    bool acm = false;                 ///< Master mode ACM (else CRM)
    int acm_scan = 0;                 ///< 0 20x30, 1 10x60, 2 BORE
    double range_scale_nm = 20.0;
    double az_limit_deg = 60.0;       ///< Scope half-width
    double scan_az_deg = 60.0;        ///< Half-width the search covers (RWS A6/A3/A1, ACM pattern)
    double scan_el_lo_deg = 0.0;      ///< Elevation the search covers (ACM: from the nose)
    double scan_el_hi_deg = 0.0;
    int bars = 4;
    double ant_az_deg = 0.0;          ///< Antenna position (the carets)
    double ant_el_deg = 0.0;
    double pitch_deg = 0.0;           ///< Ownship attitude for the artificial horizon
    double roll_deg = 0.0;
    int hit_count = 0;
    Hit hits[kMaxHits]{};
    double cursor_az_deg = 0.0;       ///< Acquisition cursor
    double cursor_range_nm = 10.0;
    int cov_top_kft = 0;              ///< Altitude block the scan covers at the cursor's range
    int cov_bottom_kft = 0;
    // STT
    bool track_memory = false;        ///< Coasting on an extrapolated track
    double tgt_az_deg = 0.0;
    double tgt_range_nm = 0.0;
    double tgt_alt_kft = 0.0;
    double tgt_rel_heading_deg = 0.0; ///< Target track relative to the ownship's heading
    double tgt_heading_deg = 0.0;     ///< True track, 0-360
    double tgt_gs_kt = 0.0;
    double closure_kt = 0.0;
    int aspect_tens = 0;              ///< Target aspect in tens of degrees (18: head-on)
    char aspect_side = 'R';           ///< Which side of the target the ownship is on
    bool blink = false;               ///< ~3 Hz flash phase
    // Datalink (Link 16) picture: every other jet, seen by the radar or not.
    static constexpr int kMaxTracks = 4;
    struct Track {
        float az_deg = 0.0f;          ///< Stabilised, from the ownship's heading
        float range_nm = 0.0f;
        float alt_kft = 0.0f;
        float rel_heading_deg = 0.0f; ///< Its track relative to the ownship's heading
        bool locked = false;          ///< The radar's STT target: the radar symbol carries the data
    };
    int track_count = 0;
    Track tracks[kMaxTracks]{};
};

/// @brief The 360 deg sweep display (right MFD in a fight): a turning beam
/// and a blip for each jet it has painted. Bearings are from the ownship's
/// nose, clockwise, degrees.
struct SweepTelemetry {
    static constexpr int kMaxBlips = 8;
    struct Blip {
        float bearing_deg = 0.0f;
        float range_nm = 0.0f;
        float alt_kft = 0.0f;
        float rel_heading_deg = 0.0f; ///< Its track relative to the ownship's heading
        float age = 0.0f;             ///< Beam turns since it was painted (0 fresh)
    };
    bool active = false;
    float sweep_deg = 0.0f;
    float range_scale_nm = 10.0f;
    float fcr_az_limit_deg = 60.0f;   ///< The FCR's scope, drawn as the cone it covers
    int blip_count = 0;
    Blip blips[kMaxBlips]{};
};

/// @brief Comprehensive real-time cockpit avionics and propulsion telemetry
/// All data required for in-cockpit displays and aural alerts.
struct AvionicsTelemetry {
    // Aircraft Type
    aircraft::AircraftType aircraft_type = aircraft::AircraftType::F16_FIGHTING_FALCON;

    /// The pilot has ejected: the engine is someone else's problem now.
    bool pilot_ejected = false;

    // Airframe Change Confirmation
    // Counts down from AIRCRAFT_SWITCH_BANNER_SEC after a type change, so the
    // HUD can confirm on-screen that the switch reached the flight model.
    // Zero means no confirmation is in progress.
    double aircraft_switch_timer = 0.0;

    // Engine & Propulsion
    double throttle_input    = 0.0;     // Commanded throttle [0.0 - 1.0]
    double net_thrust_n      = 0.0;     // Net engine thrust [N]
    double engine_rpm_pct    = 65.0;    // Core rotor speed [65% idle - 100% MIL - 104% AB]
    double engine_ftit_deg_c = 400.0;   // Fan Turbine Inlet Temperature [°C]
    double nozzle_pos_pct    = 10.0;    // Variable exhaust nozzle opening [0% - 100%]
    double oil_pressure_psi  = 50.0;    // Engine lube oil pressure [PSI]
    double hyd_press_a_psi   = 3000.0;  // Flight control hydraulic system A [PSI]
    double hyd_press_b_psi   = 3000.0;  // Flight control hydraulic system B [PSI]
    const char* detent_str   = "IDLE";  // "CUTOFF", "IDLE", "MIL", "AFTERBURNER"

    // Fuel System
    double fuel_remaining_kg = 3175.0;  // Total internal fuel remaining [kg]
    double fuel_fraction     = 1.0;     // Fuel remaining fraction [0.0 - 1.0]
    double fuel_flow_kg_hr   = 850.0;   // Current burn rate [kg/hr]

    // Landing Gear & Brakes
    bool gear_deployed       = true;    // True if landing gear handle is DOWN
    bool gear_collapsed      = false;   // True if structural collapse occurred on touchdown
    double gear_transit_pos  = 1.0;     // 0.0 = UP and locked, 1.0 = DOWN and locked
    double brake_left        = 0.0;     // Left toe brake [0.0 - 1.0]
    double brake_right       = 0.0;     // Right toe brake [0.0 - 1.0]
    double touchdown_sink_mps= 0.0;     // Touchdown sink rate [m/s]
    bool on_ground           = false;   // Any gear strut compressed

    // Aerodynamic Surfaces
    double speedbrake_pos    = 0.0;     // Speedbrake hydraulic extension [0.0 - 1.0]

    // System Warnings & Faults
    bool is_crashed          = false;   // Ground collision / airframe impact
    bool over_g_alert        = false;   // Nz within 3% of the airframe's G limits, or past them
    bool airframe_yielded    = false;   // Over-G has bent the wings (permanent)
    bool structure_failed    = false;   // Over-G has broken the airframe
    bool high_aoa_alert      = false;   // AoA > 20.0 deg
    bool bingo_fuel_alert    = false;   // Fuel < 800 kg
    bool low_altitude_alert  = false;   // Low altitude sink rate / pull up
    bool master_caution      = false;   // Master caution lamp illuminated

    // Hardware & Flight Control Status
    bool is_hardware_hotas   = false;
    const char* input_name   = "KEYBOARD";

    // On-Board Flight Computer (OFC) Status
    bool   ofc_gloc_active        = false;   ///< True while pilot is incapacitated (GLOC or recovery)
    bool   ofc_gcas_active        = false;   ///< True while Auto-GCAS manoeuvre is executing
    bool   ofc_tumble_active      = false;   ///< True while Auto-Tumble & Spin recovery is active
    double ofc_gloc_blackout_frac = 0.0;     ///< Screen-space fade [0=clear, 1=full black]
    double ofc_gcas_tti_sec       = 999.0;   ///< Seconds to terrain impact at current trajectory
    double ofc_g_exposure         = 0.0;     ///< G-exposure accumulator value (debug / HUD readout)

    // STOVL (F-35B): lift system state and the ground-referenced drift the
    // hover symbology needs. Ground velocity is in the heading frame.
    propulsion::StovlStatus stovl{};
    double ground_fwd_mps    = 0.0;     // Ground speed along the heading [m/s]
    double ground_right_mps  = 0.0;     // Lateral drift, + right [m/s]

    CombatTelemetry combat{};
    RadarTelemetry radar{};
    SweepTelemetry sweep{};
};

} // namespace fastjet::graphics
