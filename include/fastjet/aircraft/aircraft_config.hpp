#pragma once

#include "aircraft_type.hpp"
#include "../math/vector3.hpp"

namespace fastjet::aircraft {

/**
 * @brief Verified mass and inertia properties for an aircraft type.
 */
struct MassConfig {
    double empty_mass_kg{9298.64};
    double internal_fuel_capacity_kg{3175.1};
    double mtow_kg{19187.0};
    double Ixx{12875.0};
    double Iyy{75674.0};
    double Izz{85552.0};
    double Ixz{1331.0};
    /// Pilot(s), full gun ammunition and a typical air-to-air loadout with its
    /// launchers. Carried on top of the empty mass and fuel in every flight.
    double combat_payload_kg{0.0};
};

/**
 * @brief Engine ratings and propulsion installation parameters.
 */
struct PropulsionConfig {
    int engine_count{1};
    double static_idle_thrust_n{4450.0};
    double static_dry_thrust_n{75600.0};
    double static_ab_thrust_n{129000.0};
    bool has_afterburner{true};
    bool has_supercruise{false};
    double supercruise_mach{1.0};
    double tau_spool_up{3.2};
    double tau_spool_down{1.6};
    double tau_ab_light{0.6};
    double tsfc_dry{2.12e-5};
    double tsfc_ab{5.66e-5};
    double thrust_z_offset_m{0.0};      ///< Pitch-thrust arm (A-10 high nacelle = -1.2m)
    bool has_thrust_vectoring{false};   ///< 2D pitch thrust vectoring (F-22)
    double tvc_max_deflection_deg{0.0}; ///< Max TVC deflection (+/- 20 deg for F-22)
    double tvc_moment_arm_m{0.0};       ///< Distance from CG to TVC nozzle (~6.0m for F-22)
};

/**
 * @brief Aerodynamic geometric references and polar coefficients.
 */
struct AeroConfig {
    double s_ref{27.87};               ///< Wing reference area [m^2]
    double b_span{9.144};              ///< Wingspan [m]
    double c_bar{3.450};               ///< Mean aerodynamic chord [m]
    double aspect_ratio{3.0};          ///< Wing aspect ratio
    double sweep_le_deg{40.0};         ///< Leading edge sweep [deg]
    double cd0{0.020};                 ///< Parasitic zero-lift drag coefficient
    double k_induced{0.16};            ///< Induced drag factor: CD = CD0 + k * CL^2
    double cl_alpha{4.0};              ///< Lift-curve slope [1/rad]
    double cl_max{1.4};                ///< Clean max lift coefficient
    double mach_crit{0.86};            ///< Critical Mach drag-divergence onset
    double mach_peak{1.05};            ///< Transonic drag peak Mach
    double cd_wave_peak{0.026};        ///< Peak transonic wave drag
    double cd_wave_super{0.017};       ///< Supersonic wave drag plateau
    double cd_speedbrake{0.025};       ///< Full speedbrake drag increment
    double cm_speedbrake{0.006};       ///< Full speedbrake pitching moment
    double pitch_damping{ -10.0 };     ///< Cmq pitch damping derivative
    double roll_damping{ -0.45 };      ///< Clp roll damping derivative
    double yaw_damping{ -0.20 };       ///< Cnr yaw damping derivative
    // Stall. x is the linear (attached-flow) lift over cl_max: 1.0 is where a
    // straight lift line would reach CLmax.
    double stall_peak_x{1.30};         ///< Lift rounds off to CLmax by this x
    double stall_break_x{1.50};        ///< Post-stall lift loss starts here...
    double stall_width_x{1.00};        ///< ...and is complete this much further on
    double cn_90{1.60};                ///< Flat-plate normal-force coefficient at 90 deg alpha
    double cm_90{0.30};                ///< Nose-down pitching moment at 90 deg alpha
};

/**
 * @brief Flight control laws, envelope limits, and augmentation.
 */
struct FLCSConfig {
    enum class LawType {
        DIGITAL_FBW_G_ALPHA,  ///< F-16 & F-15EX: G-command / Alpha-limited digital FBW
        CAREFREE_DELTA_CANARD,///< Eurofighter: Relaxed pitch stability carefree FBW
        FBW_TVC_ALLOCATED,    ///< F-22: 5th-gen FBW with unified aero and 2D TVC
        HYDRO_SAS_AUGMENTED,  ///< A-10: Mechanical linkage with dual pitch/yaw SAS
        UNIFIED_STOVL         ///< F-35B: G/alpha FBW up-and-away; unified STOVL law once converted
    };

    /**
     * @brief Gains and handling of a G-command pitch law (AircraftFLCS::g_command_pitch).
     * The F-16 flies its own TP-1538-scheduled law (PitchController) and does not use these.
     */
    struct PitchLaw {
        double kp{1.0};                 ///< Proportional on the G error [deg / G]
        double ki{2.0};                 ///< Integral trim [deg / (G s)]
        double kd{0.6};                 ///< Pitch-rate damper [deg / (deg/s)]
        double k_alpha{0.5};            ///< Alpha feedback about the reference alpha [deg / deg]
        double integrator_limit{15.0};  ///< [G s]
        double aoa_k1{2.0};             ///< AoA limiter push per degree over [deg / deg]
        double aoa_k2{0.12};            ///< ...and per degree squared
        double de_limit{25.0};          ///< Stabilator / canard authority [deg]
        double q_cmd_max_dps{30.0};     ///< Pitch-rate command at full stick, very low q [deg/s]
        double ref_bandwidth{1.0};      ///< Scale on the G reference model's frequency: below 1 a heavier, slower feel
        double alpha_hands_off_deg{20.0}; ///< Highest alpha the law holds with the stick centred [deg]
    };

    /**
     * @brief Gains of a fly-by-wire roll/yaw law (AircraftFLCS::fbw_roll_law).
     */
    struct RollLaw {
        double kp{0.15};                ///< Roll-rate error to aileron [deg / (deg/s)]
        double ki{0.30};                ///< Roll-rate trim [deg / deg]
        double da_limit{25.0};          ///< Aileron authority [deg]
        double pedal_deg{25.0};         ///< Rudder at full pedal [deg]
        double k_yaw{0.40};             ///< Sideslip-rate damper [deg / (deg/s)]
        double k_beta{0.60};            ///< Sideslip feedback [deg / deg]
        double dr_limit{30.0};          ///< Rudder authority [deg]
        double atten_start_frac{0.6};   ///< Roll-rate attenuation starts at this share of the alpha limit...
        double atten_min{0.25};         ///< ...falling to this share of the rated rate at the limit
        double roll_cmd_tau_s{0.04};    ///< Roll command prefilter [s]: longer is a heavier feel
    };

    LawType law_type{LawType::DIGITAL_FBW_G_ALPHA};
    double max_g_positive{9.0};
    double max_g_negative{-3.0};
    double alpha_limit_deg{25.2};
    double max_roll_rate_dps{308.0};
    PitchLaw pitch{};
    RollLaw roll{};
};

/**
 * @brief Tricycle landing gear geometry and strut characteristics.
 */
struct LandingGearConfig {
    math::Vector3 nose_pos_b{3.60, 0.0, 0.30};
    double nose_spring_k{1.03e5};
    double nose_damping_c{1.20e4};
    math::Vector3 main_l_pos_b{-0.50, -1.13, 0.30};
    math::Vector3 main_r_pos_b{-0.50, 1.13, 0.30};
    double main_spring_k{3.70e5};
    double main_damping_c{4.30e4};
    double rest_length{1.40};
    double max_stroke{0.46};
    double max_steer_deg{32.0};
};

/**
 * @brief Shaft-driven lift system of a STOVL jet (F-35B: Rolls-Royce LiftSystem).
 *
 * Thrusts are the installed sea-level-static ratings with the system fully
 * converted at maximum STOVL power; they scale with the engine's power and
 * its density lapse. Positions are body axes from the CG [m] (x fwd, y right,
 * z down). Unequipped airframes keep the defaults and never use it.
 */
struct StovlConfig {
    bool equipped{false};
    double lift_fan_thrust_n{0.0};      ///< LiftFan through the vane box
    double nozzle_thrust_n{0.0};        ///< Core + bypass through the 3BSM with the fan clutched
    double roll_post_thrust_n{0.0};     ///< Both roll posts together (bypass air)
    math::Vector3 lift_fan_pos_b{};     ///< Vane-box exit
    math::Vector3 nozzle_pos_b{};       ///< 3BSM exit, nozzle down
    math::Vector3 roll_post_pos_b{};    ///< Right roll post (the left is mirrored)
    math::Vector3 fan_inlet_pos_b{};    ///< LiftFan inlet, where its intake momentum drag acts
    double fan_mass_flow_kgs{0.0};      ///< LiftFan airflow at full power (intake momentum drag)
    double fan_split_authority{0.0};    ///< LiftFan IGV thrust modulation for pitch, share of fan thrust
    double nozzle_max_deg{95.0};        ///< 3BSM: 0 aft .. 90 down .. max
    double nozzle_rate_dps{40.0};
    double nozzle_yaw_max_deg{0.0};     ///< 3BSM lateral vectoring for yaw
    double nozzle_yaw_rate_dps{30.0};
    double vane_min_deg{0.0};           ///< Vane box: fan thrust vector from vertical, + aft
    double vane_max_deg{0.0};
    double vane_rate_dps{30.0};
    double door_open_s{6.0};            ///< All STOVL doors open
    double door_close_s{5.0};
    double clutch_engage_s{7.0};        ///< LiftFan clutch engagement, spin-up and lock-up
    double clutch_release_s{2.5};
    double max_conversion_kcas{250.0};  ///< Conversion to STOVL inhibited above this
    double tsfc_stovl{1.8e-5};          ///< Fuel per newton of total lift-system thrust [kg/(N s)]
    double tau_spool_s{0.35};           ///< Small-signal thrust response in STOVL engine mode
};

/**
 * @brief Complete unified aircraft configuration specification.
 */
struct AircraftConfig {
    AircraftType type{AircraftType::F16_FIGHTING_FALCON};
    const char* display_name{"F-16C Fighting Falcon"};
    MassConfig mass{};
    PropulsionConfig propulsion{};
    AeroConfig aero{};
    FLCSConfig flcs{};
    LandingGearConfig gear{};
    StovlConfig stovl{};

    /**
     * @brief Returns the verified technical configuration for any supported aircraft.
     */
    [[nodiscard]] static constexpr AircraftConfig get(AircraftType type) noexcept {
        AircraftConfig cfg{};
        cfg.type = type;

        switch (type) {
            case AircraftType::F16_FIGHTING_FALCON: {
                cfg.display_name = "F-16C Fighting Falcon";
                // 19,700 lb empty (USAF fact sheet), 7,000 lb internal fuel, 42,300 lb MTOW.
                // Inertias are the Stevens & Lewis tensor (measured at 20,500 lb)
                // scaled to the empty weight; FuelSystem scales them back up with fuel.
                cfg.mass = MassConfig{
                    8935.8, 3175.1, 19187.0,
                    12373.0, 72723.0, 82216.0, 1279.0,
                    // Pilot 100, 510 x 20 mm 130, 2 AIM-120 + 2 AIM-9 480, rails and pylons 90
                    800.0
                };
                cfg.propulsion = PropulsionConfig{
                    1, 4450.0, 75600.0, 129000.0,
                    true, false, 1.0,
                    3.2, 1.6, 0.6,
                    2.12e-5, 5.66e-5, 0.0,
                    false, 0.0, 0.0
                };
                cfg.aero = AeroConfig{
                    27.87, 9.144, 3.450, 3.0, 40.0,
                    0.020, 0.16, 4.0, 1.4,
                    0.86, 1.05, 0.026, 0.017,
                    0.025, 0.006, -10.0, -0.45, -0.20,
                    1.30, 1.50, 1.00, 1.80, 0.36
                };
                // The F-16 flies its own TP-1538 laws (PitchController,
                // LateralDirectionalController); the generic gains are unused.
                cfg.flcs = FLCSConfig{
                    FLCSConfig::LawType::DIGITAL_FBW_G_ALPHA,
                    9.0, -3.0, 25.2, 308.0,
                    FLCSConfig::PitchLaw{}, FLCSConfig::RollLaw{}
                };
                cfg.gear = LandingGearConfig{
                    math::Vector3(3.60, 0.0, 0.30), 1.03e5, 1.20e4,
                    math::Vector3(-0.50, -1.13, 0.30),
                    math::Vector3(-0.50, 1.13, 0.30),
                    3.70e5, 4.30e4, 1.40, 0.46, 32.0
                };
                break;
            }

            case AircraftType::F15EX_EAGLE_II: {
                cfg.display_name = "F-15EX Eagle II";
                // 31,700 lb empty, 13,550 lb internal fuel, 81,000 lb MTOW
                cfg.mass = MassConfig{
                    14379.0, 6146.0, 36741.0,
                    42500.0, 255000.0, 290000.0, 3100.0,
                    // Crew of two 200, 510 x 20 mm 130, 4 AIM-120 + 2 AIM-9 780, pylons and launchers 190
                    1300.0
                };
                // 2x GE F110-GE-129 engines: 34,000 lbf dry (151.2 kN), 59,000 lbf AB (262.4 kN)
                cfg.propulsion = PropulsionConfig{
                    2, 8900.0, 151200.0, 262400.0,
                    true, false, 1.0,
                    2.8, 1.4, 0.5,
                    2.12e-5, 5.66e-5, 0.0,
                    false, 0.0, 0.0
                };
                // S_ref = 608 sq ft (56.49 m^2), b = 42.8 ft (13.05 m), c_bar = 15.94 ft (4.86 m)
                cfg.aero = AeroConfig{
                    56.49, 13.045, 4.858, 3.01, 45.0,
                    0.0175, 0.12, 4.4, 1.45,
                    0.88, 1.15, 0.028, 0.018,
                    0.045, 0.008, -12.5, -0.42, -0.22,
                    1.30, 1.50, 1.00, 1.70, 0.30
                };
                // Modern Digital Fly-By-Wire system: a conventional stable
                // airframe with a CAS, heavier in pitch and roll than the F-16
                // (slower G reference, longer roll prefilter).
                cfg.flcs = FLCSConfig{
                    FLCSConfig::LawType::DIGITAL_FBW_G_ALPHA,
                    9.0, -3.0, 30.0, 260.0,
                    // kp, ki, kd, k_alpha, int limit, AoA k1, k2, de limit, q cmd, ref bandwidth, hands-off alpha
                    FLCSConfig::PitchLaw{1.60, 3.00, 0.56, 0.40, 25.0, 2.2, 0.15, 25.0, 30.0, 0.85, 20.0},
                    // kp, ki, da limit, pedal, k yaw, k beta, dr limit, atten start, atten min, roll tau
                    FLCSConfig::RollLaw{0.20, 0.30, 25.0, 25.0, 0.40, 0.60, 30.0, 0.6, 0.25, 0.06}
                };
                // Wheelbase: 5.50m, Track: 2.76m
                cfg.gear = LandingGearConfig{
                    math::Vector3(4.80, 0.0, 0.40), 1.50e5, 1.80e4,
                    math::Vector3(-0.70, -1.38, 0.40),
                    math::Vector3(-0.70, 1.38, 0.40),
                    5.20e5, 6.00e4, 1.55, 0.50, 30.0
                };
                break;
            }

            case AircraftType::EUROFIGHTER_TYPHOON: {
                cfg.display_name = "Eurofighter Typhoon";
                // 11,000 kg empty, 4,998 kg fuel, 23,500 kg MTOW
                cfg.mass = MassConfig{
                    11000.0, 4998.0, 23500.0,
                    19500.0, 98000.0, 115000.0, 1500.0,
                    // Pilot 100, 150 x 27 mm 75, 4 BVR + 2 ASRAAM 800, launchers 125
                    1100.0
                };
                // 2x Eurojet EJ200: 120.0 kN dry, 180.0 kN AB. Dry supercruise at Mach 1.50!
                cfg.propulsion = PropulsionConfig{
                    2, 6000.0, 120000.0, 180000.0,
                    true, true, 1.50,
                    2.4, 1.2, 0.4,
                    2.10e-5, 4.80e-5, 0.0,
                    false, 0.0, 0.0
                };
                // S_ref = 51.2 m^2, b = 10.95 m, c_bar = 4.22 m, delta-canard
                cfg.aero = AeroConfig{
                    51.20, 10.950, 4.220, 2.34, 53.0,
                    0.0150, 0.14, 3.8, 1.60,
                    0.90, 1.10, 0.020, 0.014,
                    0.040, 0.005, -9.0, -0.38, -0.18,
                    1.30, 1.60, 1.20, 1.80, 0.30
                };
                // Quadruple carefree handling digital FBW: a quick G reference
                // and strong alpha feedback on the unstable delta-canard.
                cfg.flcs = FLCSConfig{
                    FLCSConfig::LawType::CAREFREE_DELTA_CANARD,
                    9.0, -3.0, 35.0, 270.0,
                    FLCSConfig::PitchLaw{1.04, 2.00, 0.70, 0.70, 15.0, 2.0, 0.12, 30.0, 30.0, 1.15, 22.0},
                    FLCSConfig::RollLaw{0.12, 0.30, 25.0, 28.0, 0.45, 0.60, 30.0, 0.6, 0.25, 0.04}
                };
                // Wheelbase: 4.40m, Track: 2.80m
                cfg.gear = LandingGearConfig{
                    math::Vector3(3.80, 0.0, 0.35), 1.25e5, 1.50e4,
                    math::Vector3(-0.60, -1.40, 0.35),
                    math::Vector3(-0.60, 1.40, 0.35),
                    4.40e5, 5.20e4, 1.45, 0.48, 35.0
                };
                break;
            }

            case AircraftType::F22_RAPTOR: {
                cfg.display_name = "F-22A Raptor";
                // 19,659 kg empty (43,340 lb), 8,165 kg fuel (18,000 lb), 37,875 kg MTOW
                cfg.mass = MassConfig{
                    19659.0, 8165.0, 37875.0,
                    61012.0, 379629.0, 427082.0, 4745.0,
                    // Pilot 100, 480 x 20 mm 125, 6 AIM-120 + 2 AIM-9 1080, bay launchers 195
                    1500.0
                };
                // 2x F119-PW-100: 232.0 kN dry, 312.0 kN AB. Dry supercruise at Mach 1.82!
                // 2D Pitch TVC nozzles (+/- 20 deg)
                cfg.propulsion = PropulsionConfig{
                    2, 12000.0, 232000.0, 312000.0,
                    true, true, 1.82,
                    2.2, 1.2, 0.5,
                    1.98e-5, 5.20e-5, 0.0,
                    true, 20.0, 6.0
                };
                // S_ref = 78.04 m^2 (840 sq ft), b = 13.56 m, c_bar = 6.40 m
                cfg.aero = AeroConfig{
                    78.04, 13.564, 6.401, 2.36, 48.0,
                    0.0165, 0.13, 3.9, 2.00,
                    0.92, 1.12, 0.022, 0.015,
                    0.035, 0.004, -14.0, -0.40, -0.25,
                    1.30, 1.60, 1.20, 1.90, 0.25
                };
                // Integrated FBW + 2D TVC Mixer
                cfg.flcs = FLCSConfig{
                    FLCSConfig::LawType::FBW_TVC_ALLOCATED,
                    9.0, -3.0, 65.0, 280.0,
                    FLCSConfig::PitchLaw{1.08, 2.00, 0.64, 0.70, 15.0, 1.8, 0.10, 25.0, 32.0, 1.0, 26.0},
                    FLCSConfig::RollLaw{0.10, 0.30, 22.0, 26.0, 0.40, 0.60, 28.0, 0.6, 0.25, 0.04}
                };
                // Wheelbase: 5.40m, Track: 3.40m
                cfg.gear = LandingGearConfig{
                    math::Vector3(4.60, 0.0, 0.45), 2.10e5, 2.50e4,
                    math::Vector3(-0.80, -1.70, 0.45),
                    math::Vector3(-0.80, 1.70, 0.45),
                    7.50e5, 8.50e4, 1.60, 0.52, 30.0
                };
                break;
            }

            case AircraftType::A10_THUNDERBOLT: {
                cfg.display_name = "A-10C Thunderbolt II";
                // 11,321 kg empty (24,959 lb), 4,990 kg fuel (11,000 lb), 22,680 kg MTOW
                cfg.mass = MassConfig{
                    11321.0, 4990.0, 22680.0,
                    47047.0, 69689.0, 106432.0, -2440.0,
                    // Pilot 100, 1,150 x 30 mm 790, 2 AIM-9 + ECM pod 270, pylons 340
                    1500.0
                };
                // 2x GE TF34-GE-100A: 80.6 kN dry, NO AFTERBURNER!
                // High nacelle mount pitch coupling: thrust line Z offset = -1.2 m
                cfg.propulsion = PropulsionConfig{
                    2, 4000.0, 80600.0, 80600.0,
                    false, false, 0.0,
                    4.5, 2.2, 0.0,
                    1.05e-5, 1.05e-5, -1.2,
                    false, 0.0, 0.0
                };
                // High-aspect-ratio straight wing: AR = 6.54, high lift, high drag
                // S_ref = 47.01 m^2 (506 sq ft), b = 17.53 m, c_bar = 3.03 m
                // CD0 is the installed figure with 11 pylons: at 0.032 the jet ran
                // ~450 kt level against a published 381 kt.  Thick-wing drag rise
                // is steep past the M0.65 critical Mach.
                cfg.aero = AeroConfig{
                    47.01, 17.526, 3.030, 6.54, 0.0,
                    0.043, 0.055, 5.1, 1.80,
                    0.65, 0.85, 0.055, 0.045,
                    0.060, 0.000, -11.0, -0.55, -0.28,
                    1.15, 1.25, 0.60, 1.50, 0.35
                };
                // Mechanical linkage with dual Pitch/Yaw Stability Augmentation System (SAS)
                // (The generic FBW gains are unused: no FBW.)
                cfg.flcs = FLCSConfig{
                    FLCSConfig::LawType::HYDRO_SAS_AUGMENTED,
                    7.33, -3.0, 18.0, 130.0,
                    FLCSConfig::PitchLaw{}, FLCSConfig::RollLaw{}
                };
                // Wide-stance gear. Nose gear offset 0.3m right for GAU-8 cannon!
                // Wheelbase: 4.60m, Track: 5.24m
                cfg.gear = LandingGearConfig{
                    math::Vector3(4.20, 0.30, 0.35), 1.30e5, 1.60e4,
                    math::Vector3(-0.40, -2.62, 0.35),
                    math::Vector3(-0.40, 2.62, 0.35),
                    4.80e5, 5.50e4, 1.50, 0.48, 35.0
                };
                break;
            }

            case AircraftType::F35B_LIGHTNING_II: {
                cfg.display_name = "F-35B Lightning II";
                // Figures from docs/f35b-reference.md (sources tagged there).
                // 32,472 lb empty, 13,100 lb internal fuel, 60,000 lb class MTOW. Inertias
                // are estimates (F-16 tensor scaled by m L^2), taken back to the empty weight.
                cfg.mass = MassConfig{
                    14729.0, 5942.0, 27200.0,
                    26850.0, 128850.0, 150300.0, 2140.0,
                    // Pilot 100, 4 AIM-120C internal 610, GAU-22/A pod with 220 x 25 mm 290
                    1000.0
                };
                // F135-PW-600: 27,000 lbf (120.1 kN) military, 41,000 lbf class (182.4 kN)
                // reheat in CTOL (P&W). Reheat is not available in STOVL.
                cfg.propulsion = PropulsionConfig{
                    1, 6000.0, 120100.0, 182400.0,
                    true, false, 1.0,
                    3.0, 1.5, 0.5,
                    2.20e-5, 5.40e-5, 0.0,
                    false, 0.0, 0.0
                };
                // S_ref = 460 sq ft (42.7 m^2), b = 35 ft (10.67 m), MAC 4.51 m, LE sweep 34.1.
                // CD0 0.021 subsonic, peaking at the blunt fuselage's 0.046 transonic; the
                // supersonic plateau is set so reheat runs out at the M1.6 limit (this
                // engine model's ram recovery is the F119's, more than a BPR 0.57 F135
                // gets). CLmax 1.5 near 35 deg with no hard stall out to the 50 deg limit.
                cfg.aero = AeroConfig{
                    42.70, 10.670, 4.510, 2.67, 34.1,
                    0.0210, 0.15, 3.3, 1.50,
                    0.88, 1.10, 0.025, 0.036,
                    0.030, 0.004, -11.0, -0.40, -0.22,
                    1.30, 1.60, 1.20, 1.80, 0.30
                };
                // F-35B: +7.0 / -3.0 G, 50 deg AoA limiter, ~200 deg/s roll.
                cfg.flcs = FLCSConfig{
                    FLCSConfig::LawType::UNIFIED_STOVL,
                    7.0, -3.0, 50.0, 200.0,
                    FLCSConfig::PitchLaw{1.30, 2.50, 0.60, 0.60, 20.0, 2.0, 0.12, 25.0, 30.0, 1.0, 25.0},
                    FLCSConfig::RollLaw{0.15, 0.30, 25.0, 25.0, 0.40, 0.60, 30.0, 0.6, 0.25, 0.04}
                };
                // Wheelbase 6.02 m, track 4.34 m, wheels 1.9 m below the CG.
                cfg.gear = LandingGearConfig{
                    math::Vector3(5.40, 0.0, 0.40), 1.40e5, 1.70e4,
                    math::Vector3(-0.60, -2.17, 0.40),
                    math::Vector3(-0.60, 2.17, 0.40),
                    5.00e5, 6.00e4, 1.50, 0.50, 30.0
                };
                // F135-PW-600 + Rolls-Royce LiftSystem, P&W hover split: 40,650 lbf =
                // LiftFan 18,680 + 3BSM 18,680 + roll posts 2 x 1,645 lbf. Post positions
                // are estimates: the fan sits just behind the cockpit (~5 m from the
                // nose) and the swivelled 3BSM exit under the aft fuselage (~13.3 m);
                // equal fan/nozzle thrust puts the CG midway, so the arms are equal.
                // 3BSM 0-95 deg in 2.5 s; vane box 54 deg in < 2 s; fan limit 250 KCAS;
                // conversion ~15 s to STOVL, ~10 s back.
                cfg.stovl = StovlConfig{
                    true,
                    83090.0, 83090.0, 14630.0,
                    math::Vector3(4.20, 0.0, 0.60),
                    math::Vector3(-4.20, 0.0, 0.70),
                    math::Vector3(-0.30, 2.40, 0.30),
                    math::Vector3(4.20, 0.0, -1.00),
                    230.0, 0.10,
                    95.0, 38.0, 12.0, 30.0,
                    -5.0, 49.0, 27.0,
                    6.0, 5.0, 7.0, 2.5, 250.0,
                    1.8e-5, 0.35
                };
                break;
            }
        }

        return cfg;
    }
};

} // namespace fastjet::aircraft
