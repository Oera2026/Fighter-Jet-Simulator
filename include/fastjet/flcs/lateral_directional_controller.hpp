#pragma once

#include "pilot_commands.hpp"
#include "imu_sensor.hpp"
#include "../aero/tp1538_tables.hpp"
#include "../aero/interpolator.hpp"
#include "../aero/f16_aero_model.hpp"
#include <algorithm>
#include <cmath>

namespace fastjet {
namespace flcs {

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
 * @brief Roll command shaping and turn coordination shared by every
 * fly-by-wire roll law: the stick commands a roll about the flight path
 * (stability axis), not about the fuselage.
 *
 * The stick's roll-rate command passes through a short first-order prefilter
 * (ROLL_CMD_TAU_S by default), so a stick snapped to a stop rolls in and out
 * without a jerk and still reaches the full rate within a few tenths of a
 * second. The roll rate is the flight-path roll rate mu-dot; the body flies it
 * as p = mu-dot cos(alpha) and r = mu-dot sin(alpha). At high alpha most of a
 * flight-path roll is yaw, and the rudder can only build or stop yaw rate so
 * fast, so the command is held to what the rudder can coordinate: its onset
 * to the yaw acceleration the rudder has, and its rate to a yaw rate the
 * rudder can stop with no more than STOP_BETA_DEG of sideslip when the stick
 * comes back (coordinated_roll_rate_max_dps). At high dynamic pressure both
 * limits are far above the jet's rated roll rate.
 *
 * The yaw channel works on sideslip: a damper on sideslip rate, through the
 * yaw rate that would hold sideslip constant (coordinated_yaw_rate_dps), and
 * sideslip feedback. Neither opposes the yaw rate of a coordinated roll or a
 * banked turn, which is why no washout is needed. (Rolled about the body
 * axis with a plain yaw damper, a jet at 35 deg alpha swapped its alpha for
 * 25-50 deg of sideslip and departed.)
 */
struct RollShaping {
    static constexpr double ROLL_CMD_TAU_S   = 0.04;  ///< Default roll-rate command prefilter [s]
    static constexpr double ALPHA_KIN_MAX_DEG = 70.0; ///< Alpha range of the roll kinematics [deg]
    /// Share of full-rudder yaw acceleration the roll may take, leaving the
    /// rest to the sideslip loop.
    static constexpr double RUDDER_SHARE     = 0.7;
    /// Sideslip a stop from the steepest coordinated roll may build [deg].
    static constexpr double STOP_BETA_DEG    = 8.0;

    double p_ref{0.0};        ///< Shaped flight-path roll-rate command [deg/s]
    double p_ref_rate{0.0};   ///< Its rate of change [deg/s^2]

    constexpr void reset() noexcept { p_ref = 0.0; p_ref_rate = 0.0; }

    /**
     * @brief Advance the prefilter toward @p p_cmd_dps; returns the shaped command.
     * @param accel_up_dps2 Cap on how fast the command may grow [deg/s^2]. It
     *        only slows the roll-in: held to it, the stop as well left the
     *        ailerons at full deflection after the stick came back, and the jet
     *        rolled on 150-200 deg.
     */
    double shape(double dt, double p_cmd_dps, double tau_s = ROLL_CMD_TAU_S,
                 double accel_up_dps2 = 1e9) noexcept {
        if (dt <= 0.0) return p_ref;
        double next = p_ref + (p_cmd_dps - p_ref) * (1.0 - std::exp(-dt / std::max(tau_s, 1e-3)));
        const double grow = accel_up_dps2 * dt;
        if (next * p_ref < 0.0) {
            // Through zero: free to zero, held to the cap beyond it.
            next = std::clamp(next, -grow, grow);
        } else if (std::abs(next) > std::abs(p_ref)) {
            next = std::clamp(next, p_ref - grow, p_ref + grow);
        }
        p_ref_rate = (next - p_ref) / dt;
        p_ref = next;
        return p_ref;
    }

    /**
     * @brief Sideslip of the air over the jet [deg], from the lateral
     * accelerometer.
     *
     * The beta in IMUData is the sideslip of the jet's inertial velocity: in a
     * crosswind it is off the air's by wind / V (1.2 deg in a 5 m/s wind at
     * 250 m/s), and closing the rudder on it banked a hands-off jet away. The
     * side force the accelerometer reads comes from the air,
     * Ny m g = q S (CY_beta beta + the rest), so it is solved for beta. At
     * very low dynamic pressure, where Ny says little, the air data takes over.
     *
     * @param cy_beta_per_deg Side-force slope with sideslip [1/deg].
     * @param cy_other        Side force from everything else (rudder, rates).
     */
    [[nodiscard]] static double air_sideslip_deg(const IMUData& imu, double mass_kg, double s_ref,
                                                 double cy_beta_per_deg, double cy_other) noexcept {
        constexpr double G0 = 9.80665;
        const double w = std::clamp((imu.q_bar - 1500.0) / 1500.0, 0.0, 1.0);
        if (w <= 0.0 || mass_kg <= 0.0 || std::abs(cy_beta_per_deg) < 1e-9) return imu.beta_deg;
        const double cy = imu.Ny * mass_kg * G0 / (imu.q_bar * s_ref);
        return w * (cy - cy_other) / cy_beta_per_deg + (1.0 - w) * imu.beta_deg;
    }

    /// @brief Alpha used by the roll kinematics [rad].
    [[nodiscard]] static double kinematic_alpha(const IMUData& imu) noexcept {
        return std::clamp(imu.alpha_deg, -ALPHA_KIN_MAX_DEG, ALPHA_KIN_MAX_DEG) * (M_PI / 180.0);
    }

    /**
     * @brief Yaw rate [deg/s] that holds sideslip where it is: from
     * beta-dot = p sin(a) - r cos(a) + (g / V)(Ny + gy) = 0. Rolling about the
     * flight path at high alpha needs p tan(alpha) of it; a banked turn adds
     * gravity's lateral share.
     */
    [[nodiscard]] static double coordinated_yaw_rate_dps(const IMUData& imu) noexcept {
        constexpr double G0 = 9.80665;
        const double a = kinematic_alpha(imu);
        const double turn = (180.0 / M_PI) * G0 * (imu.Ny + imu.gy) / std::max(imu.airspeed, 50.0);
        return imu.p_deg * std::tan(a) + turn / std::cos(a);
    }

    /**
     * @brief Cap [deg/s^2] on the flight-path roll acceleration the rudder can
     * coordinate: building yaw rate mu-dot sin(alpha) needs yaw acceleration
     * the rudder must supply.
     * @param yaw_accel_max_rps2 Full-rudder yaw acceleration [rad/s^2].
     */
    [[nodiscard]] static double coordinated_roll_accel_max_dps2(const IMUData& imu, double yaw_accel_max_rps2) noexcept {
        const double s = std::abs(std::sin(kinematic_alpha(imu)));
        return RUDDER_SHARE * yaw_accel_max_rps2 * (180.0 / M_PI) / std::max(s, 0.05);
    }

    /**
     * @brief Highest flight-path roll rate [deg/s] whose yaw rate the rudder
     * can take out with no more than STOP_BETA_DEG of sideslip: stopping yaw
     * rate r at yaw acceleration a builds r^2 / (2 a) of it.
     */
    [[nodiscard]] static double coordinated_roll_rate_max_dps(const IMUData& imu, double yaw_accel_max_rps2) noexcept {
        const double s = std::abs(std::sin(kinematic_alpha(imu)));
        const double r_max = std::sqrt(2.0 * RUDDER_SHARE * yaw_accel_max_rps2 * STOP_BETA_DEG * (M_PI / 180.0));
        return (180.0 / M_PI) * r_max / std::max(s, 0.02);
    }
};

/**
 * @brief F-16 Lateral-Directional (Roll & Yaw) Control Laws.
 * Features:
 * - Flight-path (stability-axis) roll-rate command, up to 300 deg/s, with
 *   high-AoA roll-rate attenuation, shaped by RollShaping and flown with a
 *   feed-forward of the flaperon a steady roll needs (from the TP-1538 roll
 *   damping and flaperon power).
 * - Aileron-Rudder Interconnect (ARI): the rudder that builds the yaw rate a
 *   flight-path roll needs and cancels the flaperon's own yaw, from the
 *   TP-1538 yaw derivatives.
 * - Sideslip-rate damper and sideslip feedback for Dutch roll damping and
 *   coordination in rolls and turns.
 *
 * Zero dynamic heap allocation.
 */
class LateralDirectionalController {
public:
    static constexpr double P_CMD_MAX_DPS   = 300.0;  // Maximum roll rate command [deg/s]
    static constexpr double ALPHA_ATTEN_MIN = 15.0;   // AoA where roll attenuation starts [deg]
    static constexpr double ALPHA_ATTEN_MAX = 25.5;   // AoA where roll attenuation reaches minimum [deg]

    // TP-1538 geometry and control powers (tp1538_tables.hpp)
    static constexpr double B_SPAN_M        = 9.144;
    static constexpr double S_REF_M2        = 27.87;
    static constexpr double CL_DA_PER_DEG   = -0.0028;  // Flaperon roll power
    static constexpr double CN_DR_PER_DEG   = -0.0019;  // Rudder yaw power
    static constexpr double CN_DA_PER_DEG   = -0.0003;  // Flaperon yaw
    static constexpr double CY_BETA_PER_DEG = -0.0168;  // Side force with sideslip
    static constexpr double CY_DR_PER_DEG   = +0.0032;  // ...and rudder
    static constexpr double DA_LIMIT_DEG    = 20.0;
    static constexpr double DR_LIMIT_DEG    = 30.0;

    // Gains
    double K_roll{0.15};           // Roll rate error to flaperon gain [deg da / (deg/s)]
    double K_yaw_damper{0.60};     // Sideslip-rate damper [deg dr / (deg/s)] (on r less the coordinated yaw rate)
    double K_beta{0.80};           // Sideslip feedback to rudder [deg dr / deg beta]
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
     * @param Izz Yaw inertia [kg m^2] for the ARI's yaw-acceleration terms (0: steady terms only)
     * @param mass_kg Mass [kg] for the accelerometer sideslip (0: air-data beta)
     * @param rudder_deg Rudder position [deg], whose side force the accelerometer also reads
     */
    void update(
        double dt,
        const PilotCommands& pilot,
        const IMUData& imu,
        double& da_cmd,
        double& dr_cmd,
        double Izz = 0.0,
        double mass_kg = 0.0,
        double rudder_deg = 0.0
    ) noexcept {
        constexpr double D2R = M_PI / 180.0;
        const double roll_in  = std::clamp(pilot.roll_stick, -1.0, 1.0);
        const double yaw_in   = std::clamp(pilot.rudder_pedal, -1.0, 1.0);

        // 1. High-AoA Roll Rate Attenuation:
        // Attenuates roll command linearly from 100% at alpha <= 15 deg down to 15% at alpha >= 25.5 deg
        double roll_attenuation = 1.0;
        if (imu.alpha_deg > ALPHA_ATTEN_MIN) {
            const double fraction = (imu.alpha_deg - ALPHA_ATTEN_MIN) / (ALPHA_ATTEN_MAX - ALPHA_ATTEN_MIN);
            roll_attenuation = std::clamp(1.0 - fraction * 0.85, 0.15, 1.0);
        }

        // Commanded flight-path roll rate [deg/s] (positive roll stick commands
        // a right roll), through the command prefilter, its onset held to what
        // the rudder can coordinate.
        const double qsb = imu.q_bar * S_REF_M2 * B_SPAN_M;
        const double yaw_accel_max = Izz > 0.0 ? qsb * std::abs(CN_DR_PER_DEG) * DR_LIMIT_DEG / Izz : 1e9;
        const double mu_max = std::min(P_CMD_MAX_DPS * roll_attenuation,
                                       RollShaping::coordinated_roll_rate_max_dps(imu, yaw_accel_max));
        const double mu_cmd = shaping.shape(dt, roll_in * mu_max, RollShaping::ROLL_CMD_TAU_S,
                                            RollShaping::coordinated_roll_accel_max_dps2(imu, yaw_accel_max));
        const double a = RollShaping::kinematic_alpha(imu);
        const double p_cmd = mu_cmd * std::cos(a);   // body roll rate of the flight-path roll
        const double r_cmd = mu_cmd * std::sin(a);   // ...and its yaw rate

        // 2. Closed-Loop Roll Rate Tracking:
        // Negative da produces positive roll (right wing down)
        // e_p = p_cmd - p_meas -> da = da_ff - K_roll * e_p
        // The feed-forward delivers the rate without waiting on the trim
        // integrator, which is left only what the model misses; charging the
        // integrator with the whole shortfall overshot the rate by ~30%.
        const double da_ff = std::clamp(steady_roll_flaperon_deg(p_cmd, imu.alpha_deg, imu.airspeed), -DA_LIMIT_DEG, DA_LIMIT_DEG);
        const double err_p = p_cmd - imu.p_deg;
        const double da_p  = da_ff - K_roll * err_p;
        da_cmd = std::clamp(da_p + roll_trim.update(dt, p_cmd, imu.p_deg, Ki_roll, da_p, DA_LIMIT_DEG),
                            -DA_LIMIT_DEG, DA_LIMIT_DEG);

        // 3. Aileron-Rudder Interconnect (ARI): the rudder that holds the
        // flight-path roll's yaw rate against yaw damping (and builds it),
        // and cancels the flaperon's yaw. NASA TP-1538: dr > 0 induces Cn < 0
        // (nose left), so a right roll's yaw rate takes negative rudder.
        const double cn_r = aero::Interpolator::interp1d<aero::TP1538Tables::N_ALPHA>(
            aero::TP1538Tables::ALPHA_GRID, aero::TP1538Tables::CN_R, imu.alpha_deg);
        const double cn_p = aero::Interpolator::interp1d<aero::TP1538Tables::N_ALPHA>(
            aero::TP1538Tables::ALPHA_GRID, aero::TP1538Tables::CN_P, imu.alpha_deg);
        const double lat = B_SPAN_M / (2.0 * std::max(imu.airspeed, 50.0));
        const double cn_accel = (Izz > 0.0 && qsb > 1.0) ? Izz * shaping.p_ref_rate * std::sin(a) * D2R / qsb : 0.0;
        const double cn_needed = cn_accel - lat * (cn_r * r_cmd + cn_p * p_cmd) * D2R - CN_DA_PER_DEG * da_cmd;
        const double dr_ari = std::clamp(cn_needed / CN_DR_PER_DEG, -DR_LIMIT_DEG, DR_LIMIT_DEG);

        // 4. Sideslip-rate damper and sideslip feedback (Dutch roll damping and
        // coordination): r beyond the yaw rate that holds sideslip constant is
        // sideslip rate. For beta > 0 (wind from right), dr must be < 0 to yaw
        // nose right into the relative wind.
        const double pg = aero::F16AeroModel::prandtl_glauert(
            environment::Atmosphere1976::compute(imu.altitude, imu.airspeed).mach_number);
        const double cy_p = aero::Interpolator::interp1d<aero::TP1538Tables::N_ALPHA>(
            aero::TP1538Tables::ALPHA_GRID, aero::TP1538Tables::CY_P, imu.alpha_deg);
        const double cy_r = aero::Interpolator::interp1d<aero::TP1538Tables::N_ALPHA>(
            aero::TP1538Tables::ALPHA_GRID, aero::TP1538Tables::CY_R, imu.alpha_deg);
        const double cy_other = pg * (CY_DR_PER_DEG * rudder_deg + lat * (cy_p * imu.p_deg + cy_r * imu.r_deg) * D2R);
        const double beta = RollShaping::air_sideslip_deg(imu, mass_kg, S_REF_M2, pg * CY_BETA_PER_DEG, cy_other);
        const double dr_damper = K_yaw_damper * (imu.r_deg - RollShaping::coordinated_yaw_rate_dps(imu))
                               - K_beta * beta;

        // 5. Manual Pilot Pedal Command:
        // Positive pedal (right foot) commands nose-right yaw (dr < 0)
        const double dr_pedal = -yaw_in * 30.0;

        // Total Rudder Command:
        dr_cmd = std::clamp(dr_ari + dr_damper + dr_pedal, -DR_LIMIT_DEG, DR_LIMIT_DEG);
    }
};

} // namespace flcs
} // namespace fastjet
