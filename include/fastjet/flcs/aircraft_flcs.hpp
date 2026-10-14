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
    /// Dynamic pressure band [Pa] over which the law goes from pitch-rate
    /// command (barely flying, where G means little) to G command. Every
    /// jet's approach (~3 kPa) is above it: blended up to 6 kPa, the G and
    /// pitch-rate commands disagreed, the jet settled 0.2-0.8 G short and a
    /// pilot closing the loop on attitude there could set up a PIO.
    static constexpr double Q_BLEND_LO_PA = 800.0;
    static constexpr double Q_BLEND_HI_PA = 2000.0;
    /// Loop-gain schedule: control power grows with dynamic pressure, so the
    /// loop gains go as sqrt(Q_SCHEDULE_PA / q) within these bounds.
    static constexpr double Q_SCHEDULE_PA = 15000.0;
    static constexpr double Q_SCALE_MIN = 0.30;
    static constexpr double Q_SCALE_MAX = 1.40;
    /// Nose-down pitch-rate command per degree of alpha beyond the stick's
    /// alpha, in the pitch-rate-command region [deg/s / deg].
    static constexpr double ALPHA_OVER_Q_DPS = 1.0;

    /**
     * @brief Highest alpha the stick allows [deg]: the hands-off alpha
     * (FLCSConfig::PitchLaw::alpha_hands_off_deg) with the stick centred or
     * forward, the airframe's alpha limit at full aft stick.
     *
     * At low speed, where the wing cannot make the G the stick asks for, this
     * makes the stick an alpha command. Asking for 1 G with nothing to say how
     * much alpha to spend on it, a centred stick held the Typhoon at 34 deg,
     * the F-22 at 60 and the F-35B at 50 for as long as the jet would mush.
     */
    [[nodiscard]] double alpha_cap_deg(double pitch_in) const noexcept {
        const double hands_off = std::min(config.pitch.alpha_hands_off_deg, config.alpha_limit_deg);
        return hands_off + std::clamp(pitch_in, 0.0, 1.0) * (config.alpha_limit_deg - hands_off);
    }

    /**
     * @brief G-command pitch law shared by the F-15EX, Typhoon, F-22 and F-35B
     * (gains in FLCSConfig::PitchLaw).
     *
     * Full aft stick commands the airframe's limit G. The stick's G passes
     * through a reference model (GReferenceModel) and the law flies that
     * trajectory: a model-based feed-forward of the stabilator it needs
     * (PitchTrimModel), alpha feedback about the alpha the model says that G
     * takes, a pitch-rate damper acting only on the rate beyond what following
     * it needs, and PI on the G error, with the gains scheduled down as dynamic
     * pressure rises (control power grows with it).
     *
     * The alpha feedback is the stability the relaxed-stability airframes
     * lack, in degrees per degree: both the instability and the stabilator's
     * power are coefficients, so one gain holds at any speed, where a G loop's
     * stiffness falls away with dynamic pressure (at 120 m/s the Typhoon's G
     * error alone was weaker than its instability, and a 1 G step took 2.3 s).
     *
     * A G the wing cannot make below the stick's alpha (alpha_cap_deg) is
     * flown as that alpha instead. A predictive AoA limiter guards the
     * airframe's alpha limit, and any excursion past the G limit is pushed
     * back.
     */
    [[nodiscard]] double g_command_pitch(double dt, const PilotCommands& pilot, const IMUData& imu,
                                         double q_bar, const fdm::MassProperties& mass) noexcept {
        const aircraft::FLCSConfig::PitchLaw& k = config.pitch;
        const double mass_kg = mass.mass_kg;
        const double pitch_in = std::clamp(pilot.pitch_stick, -1.0, 1.0);
        const double nz_raw = (pitch_in >= 0.0) ? (1.0 + pitch_in * (config.max_g_positive - 1.0))
                                                : (1.0 + pitch_in * (1.0 - config.max_g_negative));
        // Rolling-pull G relief (see ROLL_G_RELIEF).
        const double roll_frac = std::min(1.0, std::abs(imu.p_deg) / std::max(1.0, config.max_roll_rate_dps));
        const double nz_max_rolling = config.max_g_positive - ROLL_G_RELIEF * roll_frac;
        // The G the law flies: the stick's G, shaped by the reference model.
        double Nz_cmd = nz_ref.step(dt, std::clamp(nz_raw, config.max_g_negative, nz_max_rolling),
                                    GReferenceModel::natural_frequency(q_bar) * k.ref_bandwidth);
        // The alpha the stick allows. Barely flying, where the law commands
        // pitch rate, alpha beyond it asks for nose-down rate: a centred
        // stick holding pitch rate at zero otherwise held a stalled jet's
        // nose up at 40 deg alpha as it mushed.
        const double alpha_cap = alpha_cap_deg(pitch_in);
        const double q_cmd = pitch_in * k.q_cmd_max_dps - ALPHA_OVER_Q_DPS * std::max(0.0, imu.alpha_deg - alpha_cap);

        // Pitch rate when barely flying, G above (Q_BLEND_LO_PA).
        const double w_Nz = std::clamp((q_bar - Q_BLEND_LO_PA) / (Q_BLEND_HI_PA - Q_BLEND_LO_PA), 0.0, 1.0);
        // Control power grows with dynamic pressure; so must the loop gain fall.
        const double q_scale = std::clamp(std::sqrt(Q_SCHEDULE_PA / std::max(2500.0, q_bar)), Q_SCALE_MIN, Q_SCALE_MAX);

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

        // Feed-forward: the stabilator a steady pull at the reference G needs
        // (PitchTrimModel), solved no further than the alpha the stick allows.
        // At low q, where the law commands pitch rate, it holds the 1 G trim.
        const double nz_ff = w_Nz * Nz_cmd + (1.0 - w_Nz);
        double q_ref = 0.0;
        double de_ff = trim_model.feed_forward(dt, nz_ff, imu, mass_kg, std::min(alpha_cap, TRIM_ALPHA_MAX_DEG),
                                               k.de_limit, q_ref);
        // The reference alpha: what the model says the G takes, or, when the
        // wing cannot make that G below the stick's alpha, the stick's alpha
        // and the G the jet makes there.
        double alpha_ref = trim_model.solved_alpha_deg();
        const bool short_of_g = trim_model.short_of_g();
        if (short_of_g) {
            alpha_ref = alpha_cap;
            const double nz_avail = trim_model.nz_at_alpha(alpha_cap, imu, mass_kg);
            if (Nz_cmd > nz_avail) {
                // Trim for the pull the jet will actually fly: solved for the
                // stick's G, the feed-forward assumed that G's pitch rate and
                // short-changed the nose-down trim an unstable jet needs.
                Nz_cmd = nz_avail;
                de_ff = trim_model.stabilator_for(w_Nz * Nz_cmd + (1.0 - w_Nz), imu, mass_kg,
                                                  std::min(alpha_cap, TRIM_ALPHA_MAX_DEG), k.de_limit);
                q_ref = steady_pull_rate_dps(Nz_cmd, imu.airspeed, imu.gz);
            }
        }
        const double de_alpha = w_Nz * k.k_alpha * (imu.alpha_deg - alpha_ref);

        const double err_Nz = Nz_cmd - imu.Nz;
        const double err_q  = (q_cmd - imu.q_deg) * 0.04;
        const double error  = w_Nz * err_Nz + (1.0 - w_Nz) * err_q;

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
        // Near an alpha limit (or flying the stick's alpha) the trim that
        // pulls toward it bleeds away and only trim away from it may build:
        // the shortfall in G there is the limit at work, not mistrim. (Scaling
        // the whole integrator down instead left it pinned just above zero,
        // so a jet at the limit could never trim nose-down and hung there.)
        const bool near_pos_alpha = alpha_pred >= (config.alpha_limit_deg - 2.5) || short_of_g;
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

        // The inertial pitch-up of a roll at alpha, fed forward.
        const double de_coupling = trim_model.stabilator_for_moment(inertial_pitch_moment(imu, mass), q_bar);

        const double de = de_ff + de_coupling + de_alpha - q_scale * (k.kp * error + k.ki * pitch_integrator)
                        + k.kd * (imu.q_deg - q_steady)
                        + q_scale * OVER_G_PUSH * over_g + aoa_override_de;
        return std::clamp(de, -k.de_limit, k.de_limit);
    }

    /// Aileron roll power [1/rad] and rudder yaw power [1/rad] of the
    /// generic airframes (AircraftAeroModel).
    static constexpr double CL_DELTA_A = 0.22;
    static constexpr double CN_DELTA_R = 0.10;
    /// Side force with sideslip and rudder [1/rad] (AircraftAeroModel).
    static constexpr double CY_BETA = 0.75;
    static constexpr double CY_DELTA_R = 0.18;

    /// @brief Aileron that holds a steady roll at @p p_dps [deg]: where the
    /// aileron's rolling moment balances roll damping. Independent of dynamic
    /// pressure, it scales only with 1 / airspeed.
    [[nodiscard]] double steady_roll_aileron_deg(double p_dps, double airspeed) const noexcept {
        return aero_cfg.roll_damping * aero_cfg.b_span / (2.0 * std::max(airspeed, 50.0) * CL_DELTA_A) * p_dps;
    }

    /**
     * @brief Roll/yaw law shared by the F-15EX, Typhoon, F-22 and F-35B
     * (gains in FLCSConfig::RollLaw).
     *
     * The stick commands a roll rate about the flight path, cut back toward
     * the alpha limit. The shaped command (RollShaping) is flown as body roll
     * rate p = mu-dot cos(alpha) by the ailerons, with a feed-forward of the
     * aileron a steady roll needs, so the rate arrives without waiting on the
     * trim integrator, plus proportional feedback and a small trim for what
     * the feed-forward misses. The rudder supplies the yaw rate
     * r = mu-dot sin(alpha) the roll needs (aileron-rudder interconnect: the
     * yaw-damping moment it must overcome and the yaw acceleration to build
     * it), and closes on sideslip rate and sideslip.
     */
    void fbw_roll_law(double dt, const PilotCommands& pilot, const IMUData& imu, const fdm::MassProperties& mass,
                      double& da_cmd, double& dr_cmd) noexcept {
        const double Izz = mass.Izz;
        constexpr double D2R = M_PI / 180.0;
        const aircraft::FLCSConfig::RollLaw& k = config.roll;
        const double atten_start = k.atten_start_frac * config.alpha_limit_deg;
        const double atten = std::clamp(
            1.0 - (1.0 - k.atten_min) * (imu.alpha_deg - atten_start) / (config.alpha_limit_deg - atten_start),
            k.atten_min, 1.0);
        // Yaw authority at this dynamic pressure: how fast the rudder can
        // build (and stop) the yaw rate a flight-path roll needs.
        const double dr_lim = std::min(k.dr_limit, rudder.pos_max);
        const double qsb = imu.q_bar * aero_cfg.s_ref * aero_cfg.b_span;
        const double yaw_accel_max = Izz > 0.0 ? qsb * CN_DELTA_R * dr_lim * D2R / Izz : 1e9;
        const double mu_max = std::min(config.max_roll_rate_dps * atten,
                                       RollShaping::coordinated_roll_rate_max_dps(imu, yaw_accel_max));
        const double mu_cmd = std::clamp(pilot.roll_stick, -1.0, 1.0) * mu_max;
        const double mu_ref = roll_shaping.shape(dt, mu_cmd, k.roll_cmd_tau_s,
                                                 RollShaping::coordinated_roll_accel_max_dps2(imu, yaw_accel_max));
        const double a = RollShaping::kinematic_alpha(imu);
        const double p_ref = mu_ref * std::cos(a);
        const double r_ref = mu_ref * std::sin(a);

        const double da_lim = std::min(k.da_limit, aileron.pos_max);
        const double da_ff = std::clamp(steady_roll_aileron_deg(p_ref, imu.airspeed), -da_lim, da_lim);
        const double da_p = da_ff - k.kp * (p_ref - imu.p_deg);
        da_cmd = std::clamp(da_p + roll_trim.update(dt, p_ref, imu.p_deg, k.ki, da_p, da_lim), -da_lim, da_lim);

        // ARI: Cn_dr dr = Izz r-dot / (q S b) - Cn_r (b / 2V) r. Negative rudder
        // yaws the nose right.
        const double lat = aero_cfg.b_span / (2.0 * std::max(imu.airspeed, 50.0));
        const double cn_accel = (Izz > 0.0 && qsb > 1.0) ? Izz * roll_shaping.p_ref_rate * std::sin(a) * D2R / qsb : 0.0;
        const double cn_needed = cn_accel - aero_cfg.yaw_damping * lat * r_ref * D2R;
        const double dr_ari = std::clamp(-cn_needed / (CN_DELTA_R * D2R), -dr_lim, dr_lim);

        // Sideslip-rate damper (r beyond the yaw rate that holds sideslip) and
        // sideslip feedback.
        const double pg = trim_model.aero.prandtl_glauert(
            environment::Atmosphere1976::compute(imu.altitude, imu.airspeed).mach_number);
        const double beta = RollShaping::air_sideslip_deg(imu, mass.mass_kg, aero_cfg.s_ref, -pg * CY_BETA * D2R,
                                                          pg * CY_DELTA_R * rudder.position * D2R);
        const double dr_damp = k.k_yaw * (imu.r_deg - RollShaping::coordinated_yaw_rate_dps(imu)) - k.k_beta * beta;
        dr_cmd = std::clamp(-pilot.rudder_pedal * k.pedal_deg + dr_ari + dr_damp, -dr_lim, dr_lim);
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
            case aircraft::FLCSConfig::LawType::DIGITAL_FBW_G_ALPHA:     // F-15EX
            case aircraft::FLCSConfig::LawType::CAREFREE_DELTA_CANARD:   // Eurofighter Typhoon
            case aircraft::FLCSConfig::LawType::UNIFIED_STOVL: {         // F-35B up and away
                // G command with alpha limiting and flight-path roll-rate
                // command; per-jet gains and handling in FLCSConfig. Once the
                // F-35B converts, StovlControlLaw flies the jet and drives
                // these actuators through drive_surfaces().
                de_cmd = g_command_pitch(dt, pilot, imu, q_bar, mass);
                fbw_roll_law(dt, pilot, imu, mass, da_cmd, dr_cmd);
                break;
            }

            case aircraft::FLCSConfig::LawType::FBW_TVC_ALLOCATED: {
                // F-22A Raptor: Unified aerodynamic surfaces + 2D thrust vectoring
                de_cmd = g_command_pitch(dt, pilot, imu, q_bar, mass);
                const double alpha_pred = predicted_alpha_deg(imu, ALPHA_LEAD_S);

                // TVC Pitch Allocation: Pitch-up elevator (de < 0) corresponds to nozzle pitch-up.
                // The nose-up stick feed-forward fades out as predicted alpha closes on
                // the alpha the stick allows (alpha_cap_deg), leaving the nozzles to the
                // control law's own command (which carries the AoA limiter).  Unfaded, the
                // raw stick kept the nozzles pitching up against the limiter's push: full
                // aft stick below ~180 m/s drove alpha through the 65 deg limit and tumbled
                // the jet end over end.
                const double q_factor = std::clamp(1.0 - dynamic_pressure / 15000.0, 0.0, 1.0);
                const double alpha_factor = std::clamp((imu.alpha_deg - 20.0) / 20.0, 0.0, 1.0);
                const double tvc_weight = (std::max)(q_factor, alpha_factor);
                const double stick_ff = (pilot.pitch_stick > 0.0)
                    ? pilot.pitch_stick * std::clamp((alpha_cap_deg(pilot.pitch_stick) - alpha_pred) / 10.0, 0.0, 1.0)
                    : pilot.pitch_stick;

                tvc_pitch_cmd = std::clamp(stick_ff * (0.35 + 0.65 * tvc_weight) - (de_cmd / 25.0) * 0.5 * tvc_weight, -1.0, 1.0);

                fbw_roll_law(dt, pilot, imu, mass, da_cmd, dr_cmd);
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
