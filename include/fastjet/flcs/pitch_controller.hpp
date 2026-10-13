#pragma once

#include "pilot_commands.hpp"
#include "imu_sensor.hpp"
#include "pitch_trim_model.hpp"
#include <algorithm>
#include <cmath>

namespace fastjet {
namespace flcs {

/**
 * @brief F-16 Pitch Axis Control Laws.
 * Features:
 * - Dynamic pressure blended pitch rate (q_cmd) and normal acceleration (Nz_cmd).
 * - Hard limiters: strictly clamped to +9.0G / -3.0G and max 25.5 deg angle of attack.
 * - The stick's G is shaped by a reference model (GReferenceModel) and flown with a
 *   model-based stabilator feed-forward (PitchTrimModel), a pitch-rate damper on the
 *   rate beyond what following the reference needs, and PI on the G error.
 *
 * Sign convention (NASA TP-1538):
 * - delta_e > 0 (trailing edge DOWN) produces nose-DOWN moment (Cm < 0, reduces G/q).
 * - delta_e < 0 (trailing edge UP) produces nose-UP moment (Cm > 0, increases G/q).
 */
class PitchController {
public:
    // Limits
    static constexpr double NZ_MAX         = +9.0;   // Maximum normal acceleration [G]
    static constexpr double NZ_MIN         = -3.0;   // Minimum normal acceleration [G]
    static constexpr double ALPHA_MAX_DEG  = 25.5;   // Maximum allowable angle of attack [deg]
    static constexpr double ALPHA_MIN_DEG  = -12.0;  // Minimum allowable angle of attack [deg]
    static constexpr double Q_CMD_MAX_DPS  = 25.0;   // Maximum pitch rate command [deg/s]
    static constexpr double ALPHA_LEAD_S   = 0.10;   // AoA limiter prediction horizon [s]

    // Tuned gains for 200 Hz digital loop
    double Kp{0.80};            // Proportional gain [deg elevator / G error]
    double Ki{1.20};            // Integral gain [deg elevator / (G * s)]
    double Kd{0.50};            // Pitch rate damping gain [deg elevator / (deg/s)]
    double K_alpha{2.20};       // AoA limiter direct feedforward gain
    double K_alpha_rate{0.30};  // AoA rate lead gain
    double K_alpha_ref{0.60};   // Alpha error from the reference's alpha [deg stabilator / deg]

    // High-q gain schedule. Stabilator power grows with dynamic pressure; the
    // loop gains were tuned below Q_SCHEDULE_PA and are scaled down above it.
    // Unscheduled, a full pull at Mach 1.2 on the deck (q ~ 100 kPa) went into
    // a divergent pitch oscillation that peaked near 50 G.
    static constexpr double Q_SCHEDULE_PA   = 30000.0;
    static constexpr double Q_SCALE_MIN     = 0.30;
    /// Below Q_SCHEDULE_PA the gains rise as 1/sqrt(q) to this cap: at low
    /// dynamic pressure the unscheduled loop was too weak to unload the jet,
    /// which hung at high alpha for seconds after the stick came forward.
    static constexpr double Q_SCALE_MAX     = 1.7;
    /// The integrator sees the G error clipped to this band [G], so the large
    /// error while the G is still coming on cannot wind up trim.
    static constexpr double INTEGRATE_BAND_G = 1.0;
    /// Nose-down stabilator per G past the limit [deg / G], gain-scheduled
    /// like the rest of the loop (unscheduled it drove a pitch oscillation
    /// around the -3 G stop at Mach 1.2).
    static constexpr double OVER_G_PUSH     = 4.0;
    /// Rolling-pull G relief: the G limit drops by this much at full roll
    /// rate, and the integrator holds while the roll rate is above
    /// ROLL_HOLD_DPS. Rolling at 300 deg/s the pitch loop cannot hold the G;
    /// integrating the shortfall wound up trim that fired the jet past 11 G
    /// when the roll stopped.
    static constexpr double ROLL_G_RELIEF   = 1.8;
    static constexpr double ROLL_RATE_REF_DPS = 308.0;
    static constexpr double ROLL_HOLD_DPS   = 45.0;
    /// The trim integrator learns only while the reference G moves slower
    /// than this [G/s] and the pitch rate is within PITCH_STEADY_DPS of the
    /// reference's: with the feed-forward carrying the manoeuvre trim, what is
    /// left for it is small, and learning from the jet's lag behind the
    /// reference wound up wrong trim that held the G after release.
    static constexpr double REF_STEADY_G_PER_S = 1.0;
    static constexpr double PITCH_STEADY_DPS   = 4.0;
    /// The trim feed-forward is not solved past this alpha [deg].
    static constexpr double TRIM_ALPHA_MAX_DEG = 25.0;
    /// Predicted alpha at which the AoA limiter starts capping the G [deg].
    static constexpr double AOA_LIMITER_ONSET_DEG = 18.0;

    double integrator{0.0};     // Integrator state [G * s]
    double last_error{0.0};
    double last_de_cmd{0.0};
    GReferenceModel nz_ref{};    // G trajectory the law flies
    double last_de_ff{0.0};      // Feed-forward stabilator of the last update [deg]
    double last_q_ref{0.0};      // Pitch-rate reference of the last update [deg/s]

    constexpr PitchController() noexcept = default;

    void reset(double init_integrator = 0.0) noexcept {
        integrator = init_integrator;
        last_error = 0.0;
        last_de_cmd = 0.0;
        nz_ref.reset();
    }

    /// @brief Loop-gain multiplier for dynamic pressure @p q_bar.
    [[nodiscard]] static double q_gain_scale(double q_bar) noexcept {
        if (q_bar <= Q_SCHEDULE_PA) {
            return std::min(Q_SCALE_MAX, std::sqrt(Q_SCHEDULE_PA / std::max(q_bar, 1.0)));
        }
        return std::max(Q_SCALE_MIN, Q_SCHEDULE_PA / q_bar);
    }

    /**
     * @brief Executes pitch control law for a discrete time step.
     *
     * @param dt Time step [s]
     * @param pilot Pilot cockpit commands
     * @param imu Current IMU sensor feedback
     * @param is_actuator_saturated Boolean flag if stabilator is currently saturated
     * @param trim Onboard pitch model for the stabilator feed-forward (none: no feed-forward)
     * @param mass_kg Aircraft mass [kg], for the feed-forward
     * @param lef_schedule Leading-edge flap schedule the feed-forward assumes
     * @return double Commanded stabilator deflection [deg]
     */
    double update(
        double dt,
        const PilotCommands& pilot,
        const IMUData& imu,
        bool is_actuator_saturated = false,
        PitchTrimModel* trim = nullptr,
        double mass_kg = 0.0,
        double (*lef_schedule)(double, double) = nullptr
    ) noexcept {
        const double pitch_in = std::clamp(pilot.pitch_stick, -1.0, 1.0);

        // 1. Map Stick Input to Nz_cmd (+9.0G to -3.0G), shaped by the reference model
        const double nz_raw = (pitch_in >= 0.0) ? (1.0 + pitch_in * (NZ_MAX - 1.0))
                                                : (1.0 + pitch_in * (1.0 - NZ_MIN));
        const double roll_frac = std::min(1.0, std::abs(imu.p_deg) / ROLL_RATE_REF_DPS);
        const double nz_max_rolling = NZ_MAX - ROLL_G_RELIEF * roll_frac;
        double Nz_cmd = nz_ref.step(dt, std::clamp(nz_raw, NZ_MIN, nz_max_rolling),
                                    GReferenceModel::natural_frequency(imu.q_bar));

        // 2. Map Stick Input to q_cmd for low dynamic pressure
        double q_cmd = pitch_in * Q_CMD_MAX_DPS;

        // 3. Dynamic Pressure Blending Weight:
        // q_bar < 2000 Pa: pure q_cmd (w_Nz = 0.0)
        // q_bar > 8000 Pa: pure Nz_cmd (w_Nz = 1.0)
        const double w_Nz = std::clamp((imu.q_bar - 2000.0) / 6000.0, 0.0, 1.0);

        // 4. Hard Angle of Attack (AoA) Limiter Command Restriction
        // As alpha exceeds 18 deg and approaches 25.5 deg, override pilot command.
        // Every tier acts on alpha *predicted* ALPHA_LEAD_S ahead.  On raw alpha the
        // limiter only reacted once the nose was already through the limit, and its
        // hard push-down then threw alpha back to single figures while the pilot
        // still held full aft stick: at low q (heavy jet, 150 m/s at 5 km) that grew
        // into a divergent -8..34 deg pitch oscillation.  The rate used is alpha's,
        // i.e. pitch rate less the flight-path turn rate the current Nz accounts
        // for — a sustained pull's steady pitch rate is not alpha rising.
        constexpr double G0 = 9.80665;
        const double path_rate_dps = (180.0 / M_PI) * G0 * (imu.Nz - imu.gz)
                                   / std::max(imu.airspeed, 50.0);
        const double alpha_lim = imu.alpha_deg + ALPHA_LEAD_S * (imu.q_deg - path_rate_dps);
        double aoa_override_de = 0.0;
        if (alpha_lim > AOA_LIMITER_ONSET_DEG) {
            const double alpha_headroom = std::max(0.0, ALPHA_MAX_DEG - alpha_lim);
            // Alpha rate, not raw pitch rate: a sustained pull's steady pitch
            // rate is not alpha rising, and charging it capped every hard turn
            // near the AoA limit several G short.
            const double alpha_rate_dps = imu.q_deg - path_rate_dps;
            const double max_allowed_G = 1.0 + (alpha_headroom / 7.5) * (NZ_MAX - 1.0) - K_alpha_rate * std::max(0.0, alpha_rate_dps);
            Nz_cmd = std::min(Nz_cmd, max_allowed_G);
            q_cmd  = std::min(q_cmd,  (alpha_headroom / 7.5) * Q_CMD_MAX_DPS);

            if (alpha_lim > 22.0) {
                const double excess = alpha_lim - 22.0;
                aoa_override_de = K_alpha * excess + 0.15 * std::max(0.0, imu.q_deg);
            }
            if (alpha_lim >= 25.0) {
                // Hard ceiling push-down
                aoa_override_de += (alpha_lim - 25.0) * 15.0;
                Nz_cmd = std::min(Nz_cmd, 0.5);
                q_cmd = std::min(q_cmd, -2.0);
            }
        }

        // Negative AoA limiter: the same protection below ALPHA_MIN_DEG, so a
        // push at low speed cannot run alpha (and the G) past the negative limits.
        if (alpha_lim < ALPHA_MIN_DEG + 5.0) {
            const double headroom = std::max(0.0, alpha_lim - ALPHA_MIN_DEG);
            Nz_cmd = std::max(Nz_cmd, 1.0 - (headroom / 5.0) * (1.0 - NZ_MIN));
            q_cmd  = std::max(q_cmd, -(headroom / 5.0) * Q_CMD_MAX_DPS);
            if (alpha_lim < ALPHA_MIN_DEG + 2.5) {
                aoa_override_de -= K_alpha * ((ALPHA_MIN_DEG + 2.5) - alpha_lim);
            }
        }

        // 5. Commanded Pitch Error:
        constexpr double K_q_to_G = 0.04;
        const double err_Nz = Nz_cmd - imu.Nz;
        const double err_q  = (q_cmd - imu.q_deg) * K_q_to_G;
        const double error  = w_Nz * err_Nz + (1.0 - w_Nz) * err_q;

        // 6. Feed-forward: the stabilator a steady pull at the reference G
        // needs, and the pitch rate following the reference takes. At low q,
        // where the law commands pitch rate, it holds the 1 G trim.
        double de_ff = 0.0;
        double q_ref = 0.0;
        if (trim) {
            const double nz_ff = w_Nz * Nz_cmd + (1.0 - w_Nz);
            de_ff = trim->feed_forward(dt, nz_ff, imu, mass_kg, TRIM_ALPHA_MAX_DEG, 25.0, q_ref, lef_schedule);
            q_ref *= w_Nz;
        }
        // Alpha feedback about the alpha the reference G needs: the static
        // stability this airframe lacks. The G error alone is weak at low
        // dynamic pressure, and after an unload the jet hung at 10 deg alpha
        // and 2 G with the reference long since back at 1 G. Both the
        // instability and the stabilator's power are coefficients, so one
        // gain in degrees per degree holds at any speed. The reference alpha
        // stops short of the AoA limiter, which rules above it.
        double de_alpha = 0.0;
        if (trim) {
            const double alpha_ref = std::min(trim->solved_alpha_deg(), AOA_LIMITER_ONSET_DEG);
            de_alpha = w_Nz * K_alpha_ref * (imu.alpha_deg - alpha_ref);
        }
        last_de_ff = de_ff;
        last_q_ref = q_ref;

        // 7. Integrator Update with Anti-Windup
        bool freeze_integrator = false;
        if (is_actuator_saturated) {
            if ((last_de_cmd >= 25.0 && error < 0.0) || (last_de_cmd <= -25.0 && error > 0.0)) {
                freeze_integrator = true;
            }
        }
        if (imu.alpha_deg >= 24.5 && error > 0.0) {
            freeze_integrator = true; // Stop integrating pitch-up command near stall
        }
        // Once the AoA limiter is capping the G, the shortfall is the limit
        // at work, not mistrim: nose-up trim bleeds away and cannot build.
        // Built up through an alpha-limited pull, it held the jet at 2 G and
        // 15 deg alpha for seconds after the stick came forward.
        if (trim && alpha_lim > AOA_LIMITER_ONSET_DEG) {
            if (error > 0.0) freeze_integrator = true;
            if (integrator > 0.0 && dt > 0.0) integrator *= std::max(0.0, 1.0 - 10.0 * dt);
        }
        if (imu.alpha_deg <= ALPHA_MIN_DEG + 1.0 && error < 0.0) {
            freeze_integrator = true; // ...and pitch-down near the negative limit
        }
        if (std::abs(imu.p_deg) > ROLL_HOLD_DPS) {
            freeze_integrator = true; // Rolling pull: see ROLL_G_RELIEF
        }
        if (trim && (std::abs(nz_ref.nz_rate) > REF_STEADY_G_PER_S
                     || std::abs(imu.q_deg - q_ref) > PITCH_STEADY_DPS)) {
            freeze_integrator = true; // Transient: see REF_STEADY_G_PER_S
        }

        // The clip widens as the gains are scheduled down, so the trim the
        // integrator can build per second (in stabilator degrees) holds up at
        // high dynamic pressure.
        const double k_q = q_gain_scale(imu.q_bar);
        if (!freeze_integrator && dt > 0.0) {
            const double band = INTEGRATE_BAND_G / k_q;
            integrator += std::clamp(error, -band, band) * dt;
            integrator = std::clamp(integrator, -15.0, 15.0);
        }

        // 8. Closed-loop PI Controller with Pitch Rate Damping, gain-scheduled
        // on dynamic pressure. The damper acts on the pitch rate beyond what
        // following the reference needs: on this statically unstable airframe
        // it is what holds the G steady. Damping the whole rate opposed every
        // sustained pull, the integrator had to wind up to overcome it, and it
        // then held the G for seconds after the stick came forward.
        const double over_g = std::max(0.0, imu.Nz - NZ_MAX) - std::max(0.0, NZ_MIN - imu.Nz);
        double de_cmd = de_ff + de_alpha - k_q * (Kp * error + Ki * integrator) + k_q * Kd * (imu.q_deg - q_ref)
                      + k_q * OVER_G_PUSH * over_g + aoa_override_de;

        // Clamp to stabilator physical limits [-25, +25] deg
        de_cmd = std::clamp(de_cmd, -25.0, 25.0);

        last_error = error;
        last_de_cmd = de_cmd;

        return de_cmd;
    }
};

} // namespace flcs
} // namespace fastjet
