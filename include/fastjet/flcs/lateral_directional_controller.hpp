#pragma once

#include "pilot_commands.hpp"
#include "imu_sensor.hpp"
#include "../aero/tp1538_tables.hpp"
#include "../aero/interpolator.hpp"
#include <algorithm>
#include <cmath>

namespace fastjet {
namespace flcs {

/**
 * @brief F-16 Lateral-Directional (Roll & Yaw) Control Laws.
 * Features:
 * - Roll-rate command (up to 300 deg/s) with high-AoA roll-rate attenuation,
 *   shaped by RollShaping and flown with a feed-forward of the flaperon a steady
 *   roll needs (from the TP-1538 roll damping and flaperon power).
 * - Aileron-Rudder Interconnect (ARI) for automatic coordinated turns.
 * - Stability-axis yaw rate damper and sideslip suppression for Dutch roll damping.
 *
 * Zero dynamic heap allocation.
 */
/**
 * @brief Integral trim on a roll-rate command loop.
 *
 * Every roll law here was proportional-only, and roll damping then settled the
 * rate 20-35% short of what full stick commands (the F-22 managed 184 of its
 * 280 deg/s).  The integral closes that gap.  It bleeds away while the stick is
 * centred so no leftover trim rolls a hands-off jet, and freezes while the
 * surface is already at its stop.
 */
struct RollRateTrim {
    double integral{0.0};  ///< Accumulated roll-rate error [deg].

    constexpr void reset() noexcept { integral = 0.0; }

    /// Roll-rate error the trim integrates, at most [deg/s]. With the roll
    /// rate still building the error is the whole command; integrating all of
    /// it wound up enough trim to overshoot the commanded rate by ~30%.
    static constexpr double INTEGRATE_BAND_DPS = 20.0;

    /// @return Aileron increment [deg], same sign convention as da (negative = right roll).
    [[nodiscard]] double update(double dt, double p_cmd_dps, double p_dps,
                                double ki, double da_unsaturated, double da_limit) noexcept {
        if (dt <= 0.0) return -ki * integral;
        if (std::abs(p_cmd_dps) < 1.0) {
            integral *= std::max(0.0, 1.0 - 12.0 * dt);
        } else {
            const double err = std::clamp(p_cmd_dps - p_dps, -INTEGRATE_BAND_DPS, INTEGRATE_BAND_DPS);
            const double da_total = da_unsaturated - ki * integral;
            // Anti-windup: stop integrating further into a saturated surface.
            const bool pushing_into_stop = (da_total <= -da_limit && err > 0.0)
                                        || (da_total >=  da_limit && err < 0.0);
            if (!pushing_into_stop) integral += err * dt;
        }
        return -ki * integral;
    }
};

/**
 * @brief Roll command shaping and stability-axis yaw damping shared by every
 * fly-by-wire roll law.
 *
 * The stick's roll-rate command passes through a short first-order prefilter
 * (ROLL_CMD_TAU_S), so a stick snapped to a stop rolls in and out without a
 * jerk and still reaches the full rate within a few tenths of a second.
 *
 * The yaw damper acts on the stability-axis yaw rate r - p tan(alpha), washed
 * out. A body-axis damper opposes the yaw rate a jet needs to roll about its
 * flight path, so every roll swapped alpha for sideslip: rolling 100 deg at
 * 150 m/s dropped the G from 1 to ~0 and the nose fell away. The washout lets
 * the steady yaw rate of a banked turn through.
 */
struct RollShaping {
    static constexpr double ROLL_CMD_TAU_S   = 0.04;  ///< Roll-rate command prefilter [s]
    static constexpr double YAW_WASHOUT_S    = 1.5;   ///< Yaw damper washout [s]
    static constexpr double TAN_ALPHA_MAX    = 0.70;  ///< Cap on tan(alpha) (35 deg)

    double p_ref{0.0};        ///< Shaped roll-rate command [deg/s]
    double r_stab_lp{0.0};    ///< Low-passed stability-axis yaw rate [deg/s]

    constexpr void reset() noexcept { p_ref = 0.0; r_stab_lp = 0.0; }

    /// @brief Advance the prefilter toward @p p_cmd_dps; returns the shaped command.
    double shape(double dt, double p_cmd_dps) noexcept {
        if (dt > 0.0) p_ref += (p_cmd_dps - p_ref) * (1.0 - std::exp(-dt / ROLL_CMD_TAU_S));
        return p_ref;
    }

    /// @brief Washed-out stability-axis yaw rate [deg/s] for the yaw damper.
    double damper_rate(double dt, const IMUData& imu) noexcept {
        const double tan_a = std::clamp(std::tan(imu.alpha_deg * (M_PI / 180.0)), -TAN_ALPHA_MAX, TAN_ALPHA_MAX);
        const double r_stab = imu.r_deg - imu.p_deg * tan_a;
        if (dt > 0.0) r_stab_lp += (r_stab - r_stab_lp) * (1.0 - std::exp(-dt / YAW_WASHOUT_S));
        return r_stab - r_stab_lp;
    }
};

class LateralDirectionalController {
public:
    static constexpr double P_CMD_MAX_DPS   = 300.0;  // Maximum roll rate command [deg/s]
    static constexpr double ALPHA_ATTEN_MIN = 15.0;   // AoA where roll attenuation starts [deg]
    static constexpr double ALPHA_ATTEN_MAX = 25.5;   // AoA where roll attenuation reaches minimum [deg]

    // Gains
    double K_roll{0.15};           // Roll rate error to flaperon gain [deg da / (deg/s)]
    double K_yaw_damper{0.60};     // Yaw rate to rudder gain [deg dr / (deg/s)] (damping sign > 0)
    double K_beta{0.25};           // Sideslip feedback to rudder [deg dr / deg beta]
    double Ki_roll{0.25};          // Roll rate integral gain [deg da / deg]

    RollRateTrim roll_trim{};
    RollShaping shaping{};

    constexpr LateralDirectionalController() noexcept = default;

    constexpr void reset() noexcept {
        roll_trim.reset();
        shaping.reset();
    }

    /// @brief Flaperon [deg] that holds a steady roll at @p p_dps: the TP-1538
    /// flaperon power balancing roll damping at this alpha and airspeed.
    [[nodiscard]] static double steady_roll_flaperon_deg(double p_dps, double alpha_deg, double airspeed) noexcept {
        constexpr double CL_DA_PER_DEG = -0.0028; // TP-1538 flaperon roll power (tp1538_tables.hpp)
        constexpr double B_SPAN_M = 9.144;
        const double cl_p = aero::Interpolator::interp1d<aero::TP1538Tables::N_ALPHA>(
            aero::TP1538Tables::ALPHA_GRID, aero::TP1538Tables::CL_P, alpha_deg);
        const double damping = cl_p * B_SPAN_M / (2.0 * std::max(airspeed, 50.0)) * (p_dps * M_PI / 180.0);
        return -damping / CL_DA_PER_DEG;
    }

    /**
     * @brief Computes commanded flaperon (da) and rudder (dr) deflections.
     *
     * @param pilot Pilot cockpit commands
     * @param imu IMU sensor data
     * @param[out] da_cmd Output flaperon command [deg]
     * @param[out] dr_cmd Output rudder command [deg]
     */
    void update(
        double dt,
        const PilotCommands& pilot,
        const IMUData& imu,
        double& da_cmd,
        double& dr_cmd
    ) noexcept {
        const double roll_in  = std::clamp(pilot.roll_stick, -1.0, 1.0);
        const double yaw_in   = std::clamp(pilot.rudder_pedal, -1.0, 1.0);

        // 1. High-AoA Roll Rate Attenuation:
        // Attenuates roll command linearly from 100% at alpha <= 15 deg down to 15% at alpha >= 25.5 deg
        double roll_attenuation = 1.0;
        if (imu.alpha_deg > ALPHA_ATTEN_MIN) {
            const double fraction = (imu.alpha_deg - ALPHA_ATTEN_MIN) / (ALPHA_ATTEN_MAX - ALPHA_ATTEN_MIN);
            roll_attenuation = std::clamp(1.0 - fraction * 0.85, 0.15, 1.0);
        }

        // Commanded roll rate [deg/s] (positive roll stick commands positive/right roll rate),
        // through the command prefilter.
        const double p_cmd = shaping.shape(dt, roll_in * P_CMD_MAX_DPS * roll_attenuation);

        // 2. Closed-Loop Roll Rate Tracking:
        // Negative da produces positive roll (right wing down)
        // e_p = p_cmd - p_meas -> da = da_ff - K_roll * e_p
        // The feed-forward delivers the rate without waiting on the trim
        // integrator, which is left only what the model misses; charging the
        // integrator with the whole shortfall overshot the rate by ~30%.
        const double da_ff = std::clamp(steady_roll_flaperon_deg(p_cmd, imu.alpha_deg, imu.airspeed), -20.0, 20.0);
        const double err_p = p_cmd - imu.p_deg;
        const double da_p  = da_ff - K_roll * err_p;
        da_cmd = std::clamp(da_p + roll_trim.update(dt, p_cmd, imu.p_deg, Ki_roll, da_p, 20.0),
                            -20.0, 20.0);

        // 3. Aileron-Rudder Interconnect (ARI):
        // Automatically deflects rudder into turn to eliminate adverse yaw and coordinate roll
        // When rolling right (da < 0), dr_ari < 0 (trailing edge right -> nose right moment)
        const double K_ari = std::clamp(0.15 + 0.02 * std::max(0.0, imu.alpha_deg), 0.15, 0.65);
        const double dr_ari = K_ari * da_cmd;

        // 4. Yaw Rate Damper & Sideslip Feedback (Dutch Roll suppression):
        // NASA TP-1538: dr > 0 induces Cn < 0 (nose left).
        // For r > 0 (yawing right), dr must be > 0 to oppose yaw.
        // For beta > 0 (wind from right), dr must be < 0 to yaw nose right into relative wind.
        // The damped rate is the stability-axis one (RollShaping::damper_rate),
        // so the jet rolls about its flight path rather than trading alpha
        // for sideslip.
        const double dr_damper = K_yaw_damper * shaping.damper_rate(dt, imu) - K_beta * imu.beta_deg;

        // 5. Manual Pilot Pedal Command:
        // Positive pedal (right foot) commands nose-right yaw (dr < 0)
        const double dr_pedal = -yaw_in * 30.0;

        // Total Rudder Command:
        dr_cmd = std::clamp(dr_ari + dr_damper + dr_pedal, -30.0, 30.0);
    }
};

} // namespace flcs
} // namespace fastjet
