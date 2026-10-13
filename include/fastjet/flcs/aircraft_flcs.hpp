#pragma once

#include "f16_flcs.hpp"
#include "pitch_trim_model.hpp"
#include "../aircraft/aircraft_type.hpp"
#include "../aircraft/aircraft_config.hpp"

namespace fastjet::flcs {

/**
 * @brief Unified Flight Control System supporting all six combat aircraft.
 * Adapts control laws, limiters, actuator dynamics, and control surface scheduling:
 * - F-16: Quad digital FBW with G/AoA limiter (25.2 deg)
 * - F-15EX: Advanced digital FBW with +9g limiter and 30 deg AoA protection
 * - Typhoon: Quad carefree handling FBW with delta-canard scheduling and 35 deg AoA protection
 * - F-22A: 5th-gen FBW with integrated 2D pitch thrust vectoring allocation
 * - A-10C: Hydromechanical flight controls with dual Pitch/Yaw Stability Augmentation System (SAS)
 * - F-35B: G/AoA FBW up-and-away; once converted the unified STOVL law
 *   (StovlControlLaw) flies the jet through these actuators (drive_surfaces)
 *
 * Zero dynamic heap allocation.
 */
class AircraftFLCS {
public:
    aircraft::AircraftType aircraft_type{aircraft::AircraftType::F16_FIGHTING_FALCON};
    aircraft::FLCSConfig config{};
    aircraft::AeroConfig aero_cfg{};  ///< Roll damping and span for the roll feed-forward

    F16FLCS f16_core{};

    // Actuators
    Actuator stabilator{Actuator::create_stabilator(0.0)};
    Actuator aileron{Actuator::create_flaperon(0.0)};
    Actuator rudder{Actuator::create_rudder(0.0)};

    // Thrust vectoring command [-1.0, +1.0] for F-22
    double tvc_pitch_cmd{0.0};

    // Internal integrator and SAS damper states
    double pitch_integrator{0.0};
    GReferenceModel nz_ref{};      ///< G trajectory the pitch law flies
    double sas_pitch_damper{0.0};
    double sas_yaw_damper{0.0};
    RollRateTrim roll_trim{};
    RollShaping roll_shaping{};
    PitchTrimModel trim_model{};

    AircraftFLCS() noexcept {
        configure(aircraft::AircraftType::F16_FIGHTING_FALCON);
    }

    explicit AircraftFLCS(aircraft::AircraftType type) noexcept {
        configure(type);
    }

    void configure(aircraft::AircraftType type) noexcept {
        aircraft_type = type;
        config = aircraft::AircraftConfig::get(type).flcs;
        aero_cfg = aircraft::AircraftConfig::get(type).aero;
        trim_model.configure(type);
        f16_core.reset();
        stabilator.reset();
        aileron.reset();
        rudder.reset();
        tvc_pitch_cmd = 0.0;
        pitch_integrator = 0.0;
        nz_ref.reset();
        sas_pitch_damper = 0.0;
        sas_yaw_damper = 0.0;
        roll_trim.reset();
        roll_shaping.reset();
        trim_model.reset();
    }

    void reset() noexcept {
        f16_core.reset();
        stabilator.reset();
        aileron.reset();
        rudder.reset();
        tvc_pitch_cmd = 0.0;
        pitch_integrator = 0.0;
        nz_ref.reset();
        sas_pitch_damper = 0.0;
        sas_yaw_damper = 0.0;
        roll_trim.reset();
        roll_shaping.reset();
        trim_model.reset();
    }

    /// Prediction horizon for the AoA limiters [s].
    static constexpr double ALPHA_LEAD_S = 0.45;

    /**
     * @brief Alpha predicted @p lead_s ahead from alpha's own rate [deg].
     *
     * Alpha rate is pitch rate less the flight-path turn rate the current Nz
     * accounts for.  Leading on raw pitch rate instead charged every sustained
     * pull as if alpha were climbing, which forced a choice between capping the
     * pull well short of the limit and overshooting it in transients (Typhoon to
     * ~40 deg of a 35 deg limit, the F-22 to ~80 of 65).
     */
    [[nodiscard]] static double predicted_alpha_deg(const IMUData& imu, double lead_s) noexcept {
        const double alpha_rate = imu.q_deg - steady_pull_rate_dps(imu.Nz, imu.airspeed, imu.gz);
        return imu.alpha_deg + lead_s * alpha_rate;
    }

    /**
     * @brief Pitch rate a steady pull at @p nz_cmd flies at [deg/s].
     *
     * @p gz is gravity's share along the body's z axis, cos(theta) cos(phi):
     * the aero G holds the flight path against gravity's 1 G wings level, but
     * works with it inverted (a jet holding 1 G on its back pitches at 2 g / V).
     *
     * The pitch-rate damper acts on the rate *beyond* this.  Damping the whole
     * rate fought every sustained pull: the damper opposed the very rate the G
     * command needs, the integrator had to wind up to overcome it, and the
     * F-15EX, Typhoon and F-22 settled a full G or more short of command (7-8 G
     * on a full-aft stick rated for 9 G).
     */
    [[nodiscard]] static double steady_pull_rate_dps(double nz_cmd, double airspeed, double gz = 1.0) noexcept {
        constexpr double G0 = 9.80665;
        return (180.0 / M_PI) * G0 * (nz_cmd - gz) / std::max(airspeed, 50.0);
    }

    /// @brief Gains of one G-command pitch law.
    struct GLawGains {
        double Kp;               ///< Proportional [deg / G]
        double Ki;               ///< Integral [deg / (G s)]
        double Kd;               ///< Pitch-rate damper [deg / (deg/s)]
        double integrator_limit; ///< [G s]
        double aoa_k1;           ///< AoA limiter push per degree over [deg / deg]
        double aoa_k2;           ///< ...and per degree squared
        double de_limit;         ///< Stabilator / canard authority [deg]
        double q_cmd_max_dps;    ///< Pitch-rate command at full stick, low q
    };

    /// The integrator sees the G error clipped to this band [G].
    static constexpr double INTEGRATE_BAND_G = 1.0;
    /// Nose-down stabilator per G over the limit [deg / G], gain-scheduled.
    static constexpr double OVER_G_PUSH = 4.0;
    /// Rolling-pull G relief: the G limit drops by this much at full roll
    /// rate, and the integrator holds while rolling faster than
    /// ROLL_HOLD_DPS. The pitch loop cannot hold the G through a full-rate
    /// roll; integrating the shortfall wound up trim that fired the jet past
    /// 11 G when the roll stopped.
    static constexpr double ROLL_G_RELIEF = 1.8;
    static constexpr double ROLL_HOLD_DPS = 45.0;
    /// Negative angle-of-attack limit [deg].
    static constexpr double NEG_ALPHA_LIMIT_DEG = -15.0;
    /// The trim feed-forward is not solved past this alpha [deg]: beyond the
    /// stall the lift no longer rises with alpha and the AoA limiter rules.
    static constexpr double TRIM_ALPHA_MAX_DEG = 30.0;
    /// The pitch trim integrator learns only while the reference G moves
    /// slower than this [G/s].
    static constexpr double REF_STEADY_G_PER_S = 1.0;
    /// ...and while the jet's pitch rate is within this of the reference's
    /// [deg/s]: a jet still catching the reference up is not mistrimmed.
    static constexpr double PITCH_STEADY_DPS = 4.0;

    /**
     * @brief G-command pitch law shared by the F-15EX, Typhoon and F-22.
     *
     * Full aft stick commands the airframe's limit G. The stick's G passes
     * through a reference model (GReferenceModel) and the law flies that
     * trajectory: a model-based feed-forward of the stabilator it needs
     * (PitchTrimModel), a pitch-rate damper acting only on the rate beyond
     * what following it needs, and PI on the G error, with the gains scheduled
     * down as dynamic pressure rises (control power grows with it). A
     * predictive AoA limiter takes over near the alpha limit, and any
     * excursion past the G limit is pushed back.
     */
    [[nodiscard]] double g_command_pitch(double dt, const PilotCommands& pilot, const IMUData& imu,
                                         double q_bar, double mass_kg, const GLawGains& k) noexcept {
        const double pitch_in = std::clamp(pilot.pitch_stick, -1.0, 1.0);
        const double nz_raw = (pitch_in >= 0.0) ? (1.0 + pitch_in * (config.max_g_positive - 1.0))
                                                : (1.0 + pitch_in * (1.0 - config.max_g_negative));
        // Rolling-pull G relief (see ROLL_G_RELIEF).
        const double roll_frac = std::min(1.0, std::abs(imu.p_deg) / std::max(1.0, config.max_roll_rate_dps));
        const double nz_max_rolling = config.max_g_positive - ROLL_G_RELIEF * roll_frac;
        // The G the law flies: the stick's G, shaped by the reference model.
        double Nz_cmd = nz_ref.step(dt, std::clamp(nz_raw, config.max_g_negative, nz_max_rolling),
                                    GReferenceModel::natural_frequency(q_bar));
        const double q_cmd = pitch_in * k.q_cmd_max_dps;

        // Pitch rate at low speed, G at high speed.
        const double w_Nz = std::clamp((q_bar - 2000.0) / 4000.0, 0.0, 1.0);
        // Control power grows with dynamic pressure; so must the loop gain fall.
        const double q_scale = std::clamp(std::sqrt(15000.0 / std::max(2500.0, q_bar)), 0.30, 1.40);

        // Predictive AoA limiter: anticipates stall entry and smoothly arrests pitch rate.
        const double alpha_pred = predicted_alpha_deg(imu, ALPHA_LEAD_S);
        double aoa_override_de = 0.0;
        if (alpha_pred > (config.alpha_limit_deg - 5.0)) {
            const double alpha_headroom = std::max(0.0, config.alpha_limit_deg - alpha_pred);
            Nz_cmd = std::min(Nz_cmd, 1.0 + (alpha_headroom / 5.0) * (config.max_g_positive - 1.0));
            if (alpha_pred > (config.alpha_limit_deg - 2.5)) {
                const double excess = alpha_pred - (config.alpha_limit_deg - 2.5);
                aoa_override_de = k.aoa_k1 * excess + k.aoa_k2 * excess * excess;
            }
        }

        // The same protection below NEG_ALPHA_LIMIT_DEG: a push at low speed
        // otherwise ran alpha out to -30 deg and the jet to -4 G as it gathered
        // speed nose-down.
        if (alpha_pred < NEG_ALPHA_LIMIT_DEG + 5.0) {
            const double headroom = std::max(0.0, alpha_pred - NEG_ALPHA_LIMIT_DEG);
            Nz_cmd = std::max(Nz_cmd, 1.0 - (headroom / 5.0) * (1.0 - config.max_g_negative));
            if (alpha_pred < NEG_ALPHA_LIMIT_DEG + 2.5) {
                const double excess = (NEG_ALPHA_LIMIT_DEG + 2.5) - alpha_pred;
                aoa_override_de -= k.aoa_k1 * excess + k.aoa_k2 * excess * excess;
            }
        }

        const double err_Nz = Nz_cmd - imu.Nz;
        const double err_q  = (q_cmd - imu.q_deg) * 0.04;
        const double error  = w_Nz * err_Nz + (1.0 - w_Nz) * err_q;

        // Feed-forward: the stabilator a steady pull at the reference G needs
        // (PitchTrimModel). At low q, where the law commands pitch rate, it
        // holds the 1 G trim.
        const double nz_ff = w_Nz * Nz_cmd + (1.0 - w_Nz);
        double q_ref = 0.0;
        const double de_ff = trim_model.feed_forward(dt, nz_ff, imu, mass_kg,
                                                     std::min(config.alpha_limit_deg, TRIM_ALPHA_MAX_DEG),
                                                     k.de_limit, q_ref);

        // Damp only the pitch rate beyond what following the reference needs.
        // Referenced to the measured G instead, the damper braked the onset
        // (G lags pitch rate) and let go just as the G overshot: 50% overshoot
        // on the Typhoon and a bounce through negative G on release.
        const double q_steady = w_Nz * q_ref;

        // Anti-windup: bleed near the alpha limits; hold while the surface is
        // pinned against its stop in the direction the error asks for; and
        // integrate a clipped error, so the large error while the G is still
        // coming on cannot wind up trim. Integrating it whole carried the
        // relaxed-stability Typhoon and F-22 half a G past the limit before
        // the excess bled away.
        const double stop = std::min(k.de_limit, stabilator.pos_max);
        const bool pinned = (stabilator.position <= -stop + 0.5 && error > 0.0)
                         || (stabilator.position >= stop - 0.5 && error < 0.0);
        // Near an alpha limit the trim that pulls toward it bleeds away and
        // only trim away from the limit may build. (Scaling the whole
        // integrator down instead left it pinned just above zero, so a jet at
        // the limit could never trim nose-down and hung there hands-off.)
        const bool near_pos_alpha = alpha_pred >= (config.alpha_limit_deg - 2.5);
        const bool near_neg_alpha = alpha_pred <= (NEG_ALPHA_LIMIT_DEG + 2.5);
        if (dt > 0.0) {
            if (near_pos_alpha && pitch_integrator > 0.0) {
                pitch_integrator *= std::max(0.0, 1.0 - 15.0 * dt);
            }
            if (near_neg_alpha && pitch_integrator < 0.0) {
                pitch_integrator *= std::max(0.0, 1.0 - 15.0 * dt);
            }
            double e_int = std::clamp(error, -INTEGRATE_BAND_G, INTEGRATE_BAND_G);
            // The integrator carries only the trim the feed-forward misses, so
            // it learns while the reference G is steady. Integrating the
            // transient error as the G came off wound up a degree of wrong
            // trim that left the jet sagging at 0.5 G for seconds.
            if (std::abs(nz_ref.nz_rate) > REF_STEADY_G_PER_S
                || std::abs(imu.q_deg - q_steady) > PITCH_STEADY_DPS) {
                e_int = 0.0;
            }
            if (near_pos_alpha) e_int = std::min(e_int, 0.0);
            if (near_neg_alpha) e_int = std::max(e_int, 0.0);
            if (!pinned && std::abs(imu.p_deg) <= ROLL_HOLD_DPS) {
                pitch_integrator = std::clamp(pitch_integrator + e_int * dt, -k.integrator_limit, k.integrator_limit);
            }
        }

        const double over_g = std::max(0.0, imu.Nz - config.max_g_positive)
                            - std::max(0.0, config.max_g_negative - imu.Nz);

        const double de = de_ff - q_scale * (k.Kp * error + k.Ki * pitch_integrator)
                        + k.Kd * (imu.q_deg - q_steady)
                        + q_scale * OVER_G_PUSH * over_g + aoa_override_de;
        return std::clamp(de, -k.de_limit, k.de_limit);
    }

    /// @brief Gains of one fly-by-wire roll law.
    struct RollLawGains {
        double kp;          ///< Roll-rate error to aileron [deg / (deg/s)]
        double ki;          ///< Roll-rate trim [deg / deg]
        double da_limit;    ///< Aileron authority [deg]
        double pedal_deg;   ///< Rudder at full pedal [deg]
        double k_yaw;       ///< Yaw damper [deg / (deg/s)]
        double k_beta;      ///< Sideslip feedback [deg / deg]
        double dr_limit;    ///< Rudder authority [deg]
    };

    /// @brief Aileron that holds a steady roll at @p p_dps [deg]: where the
    /// aileron's rolling moment balances roll damping. Independent of dynamic
    /// pressure, it scales only with 1 / airspeed.
    [[nodiscard]] double steady_roll_aileron_deg(double p_dps, double airspeed) const noexcept {
        constexpr double CL_DELTA_A = 0.22; // Aileron roll power [1/rad] (AircraftAeroModel)
        return aero_cfg.roll_damping * aero_cfg.b_span / (2.0 * std::max(airspeed, 50.0) * CL_DELTA_A) * p_dps;
    }

    /**
     * @brief Roll-rate command law shared by the F-15EX, Typhoon and F-22.
     *
     * The shaped command (RollShaping) drives a feed-forward of the aileron a
     * steady roll needs, so the rate arrives without waiting on the trim
     * integrator, plus proportional feedback and a small trim for what the
     * feed-forward misses. Roll rate is cut back toward the alpha limit, and
     * the yaw damper works on the stability axis so the jet rolls about its
     * flight path.
     */
    void fbw_roll_law(double dt, const PilotCommands& pilot, const IMUData& imu, const RollLawGains& k,
                      double& da_cmd, double& dr_cmd) noexcept {
        const double atten_start = 0.6 * config.alpha_limit_deg;
        const double atten = std::clamp(
            1.0 - 0.75 * (imu.alpha_deg - atten_start) / (config.alpha_limit_deg - atten_start), 0.25, 1.0);
        const double p_cmd = std::clamp(pilot.roll_stick, -1.0, 1.0) * config.max_roll_rate_dps * atten;
        const double p_ref = roll_shaping.shape(dt, p_cmd);

        const double da_lim = std::min(k.da_limit, aileron.pos_max);
        const double da_ff = std::clamp(steady_roll_aileron_deg(p_ref, imu.airspeed), -da_lim, da_lim);
        const double da_p = da_ff - k.kp * (p_ref - imu.p_deg);
        da_cmd = std::clamp(da_p + roll_trim.update(dt, p_ref, imu.p_deg, k.ki, da_p, da_lim), -da_lim, da_lim);

        const double r_damp = roll_shaping.damper_rate(dt, imu);
        dr_cmd = std::clamp(-pilot.rudder_pedal * k.pedal_deg + k.k_yaw * r_damp - k.k_beta * imu.beta_deg,
                            -k.dr_limit, k.dr_limit);
    }

    /**
     * @brief Steps the flight control laws and physical actuators.
     */
    aero::ControlSurfaces update(
        double dt,
        const fdm::FlightState& state,
        const PilotCommands& pilot,
        double dynamic_pressure,
        const fdm::AircraftForces& aero_forces,
        const fdm::MassProperties& mass
    ) noexcept {
        // Delegate F-16 directly to original tested implementation
        if (aircraft_type == aircraft::AircraftType::F16_FIGHTING_FALCON) {
            return f16_core.update(dt, state, pilot, dynamic_pressure, aero_forces, mass);
        }

        const IMUData imu = IMUData::read(state, aero_forces, mass, dynamic_pressure);

        double de_cmd = 0.0;
        double da_cmd = 0.0;
        double dr_cmd = 0.0;
        double dlef_cmd = 0.0;

        const double q_bar = dynamic_pressure;

        switch (config.law_type) {
            case aircraft::FLCSConfig::LawType::DIGITAL_FBW_G_ALPHA: {
                // F-15EX Digital FBW: G command blending with smooth predictive AoA protection
                de_cmd = g_command_pitch(dt, pilot, imu, q_bar, mass.mass_kg, {1.60, 3.00, 0.56, 25.0, 2.2, 0.15, 25.0, 30.0});

                // Roll & Yaw: roll rate command + yaw rate damper + turn coordination
                fbw_roll_law(dt, pilot, imu, {0.20, 0.30, 25.0, 25.0, 0.40, 0.20, 30.0}, da_cmd, dr_cmd);
                break;
            }

            case aircraft::FLCSConfig::LawType::UNIFIED_STOVL: {
                // F-35B up-and-away (CTOL mode): G command with AoA limiting and
                // roll-rate command. Once converted, StovlControlLaw flies the jet
                // and drives these actuators through drive_surfaces().
                de_cmd = g_command_pitch(dt, pilot, imu, q_bar, mass.mass_kg, {1.30, 2.50, 0.60, 20.0, 2.0, 0.12, 25.0, 30.0});
                fbw_roll_law(dt, pilot, imu, {0.15, 0.30, 25.0, 25.0, 0.40, 0.20, 30.0}, da_cmd, dr_cmd);
                break;
            }

            case aircraft::FLCSConfig::LawType::CAREFREE_DELTA_CANARD: {
                // Eurofighter Typhoon Carefree Handling
                de_cmd = g_command_pitch(dt, pilot, imu, q_bar, mass.mass_kg, {1.04, 2.00, 0.70, 15.0, 2.0, 0.12, 30.0, 30.0});

                fbw_roll_law(dt, pilot, imu, {0.12, 0.30, 25.0, 28.0, 0.45, 0.25, 30.0}, da_cmd, dr_cmd);
                break;
            }

            case aircraft::FLCSConfig::LawType::FBW_TVC_ALLOCATED: {
                // F-22A Raptor: Unified aerodynamic surfaces + 2D thrust vectoring
                de_cmd = g_command_pitch(dt, pilot, imu, q_bar, mass.mass_kg, {1.08, 2.00, 0.64, 15.0, 1.8, 0.10, 25.0, 32.0});
                const double alpha_pred = predicted_alpha_deg(imu, ALPHA_LEAD_S);

                // TVC Pitch Allocation: Pitch-up elevator (de < 0) corresponds to nozzle pitch-up.
                // The nose-up stick feed-forward fades out as predicted alpha closes on
                // the limit, leaving the nozzles to the control law's own command (which
                // carries the AoA limiter).  Unfaded, the raw stick kept the nozzles
                // pitching up against the limiter's push: full aft stick below ~180 m/s
                // drove alpha through the 65 deg limit and tumbled the jet end over end.
                const double q_factor = std::clamp(1.0 - dynamic_pressure / 15000.0, 0.0, 1.0);
                const double alpha_factor = std::clamp((imu.alpha_deg - 20.0) / 20.0, 0.0, 1.0);
                const double tvc_weight = (std::max)(q_factor, alpha_factor);
                const double stick_ff = (pilot.pitch_stick > 0.0)
                    ? pilot.pitch_stick * std::clamp((config.alpha_limit_deg - alpha_pred) / 10.0, 0.0, 1.0)
                    : pilot.pitch_stick;

                tvc_pitch_cmd = std::clamp(stick_ff * (0.35 + 0.65 * tvc_weight) - (de_cmd / 25.0) * 0.5 * tvc_weight, -1.0, 1.0);

                fbw_roll_law(dt, pilot, imu, {0.10, 0.30, 22.0, 26.0, 0.40, 0.20, 28.0}, da_cmd, dr_cmd);
                break;
            }

            case aircraft::FLCSConfig::LawType::HYDRO_SAS_AUGMENTED: {
                // A-10C Warthog: Direct hydromechanical linkage + dual Pitch/Yaw SAS dampers
                // Pulling stick back (pitch_stick > 0) commands elevator UP (de_cmd < 0).
                // Gearing puts full aft stick just past stall alpha, as on any
                // conventional jet: 22 deg per unit stick would have trimmed to ~77 deg
                // and pulled 9-17 G.  Above Q_FEEL the feel system's stick force per G
                // leaves a pilot short of full elevator, so a full pull near Vne bends
                // the jet a little past 7.33 G rather than breaking it.
                constexpr double ELEVATOR_PER_STICK_DEG = 10.0;
                constexpr double Q_FEEL_PA = 20000.0;
                const double feel_authority = std::clamp(Q_FEEL_PA / std::max(q_bar, 1.0), 0.4, 1.0);
                // Pushing, the feel system is stiffer still: the jet is rated
                // for -3 G against +7.33, and full forward stick at speed made
                // -6 to -8 G, well past what it is built for.
                constexpr double PUSH_GEARING = 0.45;
                const double gearing = pilot.pitch_stick < 0.0 ? PUSH_GEARING : 1.0;
                de_cmd = -pilot.pitch_stick * ELEVATOR_PER_STICK_DEG * feel_authority * gearing;

                // Pitch SAS damper (damps pitch rate)
                sas_pitch_damper += (imu.q_deg - sas_pitch_damper) * (1.0 - std::exp(-dt / 0.20));
                de_cmd += sas_pitch_damper * 0.25;
                de_cmd = std::clamp(de_cmd, -25.0, 15.0);

                // Direct ailerons: right stick (roll_stick > 0) commands right roll (da_cmd < 0)
                da_cmd = -pilot.roll_stick * 20.0;

                // Direct rudder: right pedal (rudder_pedal > 0) commands right yaw (dr_cmd < 0)
                sas_yaw_damper += (imu.r_deg - sas_yaw_damper) * (1.0 - std::exp(-dt / 0.15));
                dr_cmd = -pilot.rudder_pedal * 25.0 + sas_yaw_damper * 0.35;
                dr_cmd = std::clamp(dr_cmd, -25.0, 25.0);

                tvc_pitch_cmd = 0.0;
                break;
            }
        }

        // Step hydraulic actuators
        const double de_actual = stabilator.step(de_cmd, dt);
        const double da_actual = aileron.step(da_cmd, dt);
        const double dr_actual = rudder.step(dr_cmd, dt);

        return {de_actual, da_actual, dr_actual, dlef_cmd};
    }

    /**
     * @brief Steps the surface actuators on commands from another control law
     * (the F-35B's unified STOVL law), keeping them continuous across the
     * handover. The up-and-away law's states restart from the current flight
     * condition, so it picks up cleanly when it is given the jet back.
     */
    aero::ControlSurfaces drive_surfaces(double dt, double de_cmd, double da_cmd, double dr_cmd,
                                         double nz_now) noexcept {
        pitch_integrator = 0.0;
        nz_ref.reset(nz_now);
        roll_trim.reset();
        roll_shaping.reset();
        tvc_pitch_cmd = 0.0;
        return {stabilator.step(de_cmd, dt), aileron.step(da_cmd, dt), rudder.step(dr_cmd, dt), 0.0};
    }
};

} // namespace fastjet::flcs
