#pragma once

#include "interpolator.hpp"
#include "compressible_lift.hpp"
#include "control_surfaces.hpp"
#include "tp1538_tables.hpp"
#include "../fdm/aircraft_forces.hpp"
#include "../fdm/flight_state.hpp"
#include "../environment/atmosphere1976.hpp"
#include <cmath>

namespace fastjet {
namespace aero {

/**
 * @brief Computed non-dimensional aerodynamic coefficients and parameters.
 */
struct AeroCoefficients {
    double CL{0.0};         // Lift coefficient [-]
    double CD{0.0};         // Drag coefficient [-]
    double Cm{0.0};         // Pitching moment coefficient [-]
    double CY{0.0};         // Sideforce coefficient [-]
    double Cl{0.0};         // Rolling moment coefficient [-]
    double Cn{0.0};         // Yawing moment coefficient [-]
    double alpha_deg{0.0};  // Angle of attack [deg]
    double beta_deg{0.0};   // Sideslip angle [deg]
    double q_bar{0.0};      // Dynamic pressure [Pa]
    double mach{0.0};       // Mach number [-]
};

/**
 * @brief F-16 Aerodynamic Model based on NASA TP-1538 wind-tunnel dataset.
 * Combines lookup table interpolation, damping derivatives, and dimensional synthesis.
 * Zero dynamic heap allocation.
 */
class F16AeroModel {
public:
    // Standard F-16 Geometric Reference Quantities (SI units)
    static constexpr double S_REF_SQFT   = 300.0;                       // Wing area [ft^2]
    static constexpr double C_BAR_FT     = 11.32;                       // Mean aerodynamic chord [ft]
    static constexpr double B_SPAN_FT    = 30.0;                        // Wingspan [ft]

    static constexpr double FT_TO_M      = 0.3048;
    static constexpr double SQFT_TO_SQM  = FT_TO_M * FT_TO_M;           // 0.09290304

    static constexpr double S_REF        = S_REF_SQFT * SQFT_TO_SQM;    // 27.870912 m^2
    static constexpr double C_BAR        = C_BAR_FT * FT_TO_M;          // 3.450336 m
    static constexpr double B_SPAN       = B_SPAN_FT * FT_TO_M;         // 9.144 m

    static constexpr double RAD_TO_DEG   = 180.0 / M_PI;
    static constexpr double DEG_TO_RAD   = M_PI / 180.0;

    // --- Compressibility -------------------------------------------------
    // The NASA TP-1538 tables are low-speed wind-tunnel data and carry no Mach
    // dependence. These corrections extend them across the transonic and
    // supersonic range the airframe can actually reach.
    static constexpr double MACH_CRIT       = 0.86;  // Drag-divergence onset
    static constexpr double MACH_PEAK       = 1.05;  // Transonic drag peak
    static constexpr double CD_WAVE_PEAK    = 0.026;
    // Quadratic supersonic drag growth beyond the transonic peak. Calibrated
    // so that maximum level speed lands near the real F-16C figures: about
    // Mach 2.05 at 11,000 m and roughly Mach 1.1 on the deck, where the
    // aircraft is dynamic-pressure limited rather than thrust limited.
    static constexpr double CD_SUPERSONIC_RISE = 0.006;

    // Dynamic-pressure drag rise. Onset sits below the F-16's ~102 kPa
    // (800 KEAS) structural limit so drag climbs steeply as the airframe
    // approaches the placard. This makes the aircraft q-limited on the deck
    // while leaving the high-altitude supersonic envelope untouched.
    static constexpr double Q_DRAG_ONSET = 60000.0; // [Pa]
    static constexpr double CD_Q_RISE    = 0.095;
    static constexpr double CD_WAVE_SUPER   = 0.017; // Supersonic plateau
    static constexpr double MACH_TABLE_LIMIT = 0.80; // Prandtl-Glauert validity cap

    // Mach tuck: aerodynamic centre shifts aft transonically, producing a
    // nose-down pitching moment increment.
    static constexpr double CM_TUCK_MAX = -0.030;
    // Aft shift of the aerodynamic centre through the transonic region, as a
    // fraction of the mean chord: Delta Cm = -shift * CL.
    static constexpr double AC_SHIFT_MAX   = 0.12;
    static constexpr double AC_SHIFT_START = 0.90;
    static constexpr double AC_SHIFT_END   = 1.20;

    // --- Speedbrake ------------------------------------------------------
    // Fully extended F-16 airbrakes add roughly Delta CD = +0.025 and, being
    // mounted above and below the tailpipe, a small nose-up pitch increment.
    static constexpr double CD_SPEEDBRAKE_FULL = 0.025;
    static constexpr double CM_SPEEDBRAKE_FULL = 0.006;

    // --- Stall and reversed flow -------------------------------------------
    /// Low-speed maximum lift coefficient of the table (35 deg, LEF down).
    static constexpr double CL_MAX_LOW_SPEED = 1.55;
    /// Flat-plate normal-force coefficient at 90 deg (matches the table's 90 deg row).
    static constexpr double CN_90 = 1.80;
    /// Pitching moment at 90 deg (matches the table's 90 deg row).
    static constexpr double CM_90 = -0.36;
    /// Skin-friction drag flying tail first.
    static constexpr double CD_REVERSED_MIN = 0.03;

    /**
     * @brief Prandtl-Glauert compressibility factor 1/sqrt(|1 - M^2|).
     *
     * Applied to lift and moment below the critical Mach number and reused in
     * the supersonic branch. The subsonic form is capped at MACH_TABLE_LIMIT to
     * avoid the singularity at M = 1.
     */
    [[nodiscard]] static double prandtl_glauert(double mach) noexcept {
        if (mach < MACH_TABLE_LIMIT) {
            return 1.0 / std::sqrt(1.0 - mach * mach);
        }
        if (mach <= MACH_PEAK) {
            // Blend through the transonic region: hold the value at the cap
            // rather than diverging as M -> 1.
            return 1.0 / std::sqrt(1.0 - MACH_TABLE_LIMIT * MACH_TABLE_LIMIT);
        }
        // Supersonic: Ackeret-like 1/sqrt(M^2 - 1), taken over only once it has
        // fallen below the transonic plateau (M ~1.17).  Bounding it at M^2-1 >= 0.1
        // instead jumped the factor from 1.67 to 3.1 just past MACH_PEAK — lift
        // nearly doubled in one tick and the jet spiked well past its G limit.
        const double plateau = 1.0 / std::sqrt(1.0 - MACH_TABLE_LIMIT * MACH_TABLE_LIMIT);
        const double m2 = std::max(mach * mach - 1.0, 1e-6);
        return std::min(plateau, 1.0 / std::sqrt(m2));
    }

    /**
     * @brief Wave-drag increment vs Mach number.
     *
     * Zero below the critical Mach, rising to a peak just above M = 1, then
     * settling to a supersonic plateau.
     */
    [[nodiscard]] static double wave_drag(double mach) noexcept {
        if (mach <= MACH_CRIT) return 0.0;

        if (mach <= MACH_PEAK) {
            // Smooth rise from onset to the transonic peak.
            const double t = (mach - MACH_CRIT) / (MACH_PEAK - MACH_CRIT);
            return CD_WAVE_PEAK * t * t * (3.0 - 2.0 * t); // smoothstep
        }

        // Decay from the transonic peak toward the supersonic level.
        const double decay = std::exp(-(mach - MACH_PEAK) / 0.45);
        const double base = CD_WAVE_SUPER + (CD_WAVE_PEAK - CD_WAVE_SUPER) * decay;

        // Supersonic drag rise. A flat plateau is not physical: shock losses
        // and inlet spillage keep growing with Mach.
        const double excess = std::max(0.0, mach - MACH_PEAK);
        return base + CD_SUPERSONIC_RISE * excess * excess;
    }

    /**
     * @brief Excrescence / inlet-spillage drag that scales with dynamic pressure.
     *
     * The F-16's low-altitude speed limit is set by dynamic pressure, not by
     * Mach number: the placarded 800 KEAS limit corresponds to q ~ 102 kPa,
     * which is reached at about Mach 1.2 on the deck but not until well beyond
     * Mach 2 at altitude. A drag term that depends only on Mach cannot
     * reproduce both ends of the envelope at once, because it penalises high
     * altitude (where the real aircraft is fastest) just as hard as sea level.
     *
     * Physically this represents the losses that grow with absolute air
     * loading rather than with Mach alone -- inlet spillage and bleed, control
     * surfaces trimming against rising hinge moments, and skin friction on a
     * compressible, heated boundary layer. It is negligible in the cruise
     * envelope and becomes dominant only near the structural limit.
     *
     * @param q_bar Dynamic pressure [Pa].
     * @return Drag-coefficient increment.
     */
    [[nodiscard]] static double dynamic_pressure_drag(double q_bar) noexcept {
        if (q_bar <= Q_DRAG_ONSET) return 0.0;
        const double excess = (q_bar - Q_DRAG_ONSET) / Q_DRAG_ONSET;
        return CD_Q_RISE * excess * excess;
    }

    /**
     * @brief Computes aerodynamic coefficients (CL, CD, Cm, CY, Cl, Cn).
     */
    [[nodiscard]] static AeroCoefficients compute_coefficients(
        const fdm::FlightState& state,
        const ControlSurfaces& controls,
        double q_bar,
        double mach = 0.0
    ) noexcept {
        AeroCoefficients coeffs;
        coeffs.q_bar = q_bar;
        coeffs.mach = mach;

        const double V = state.airspeed();
        const double alpha_rad = state.alpha();
        const double beta_rad  = state.beta();

        coeffs.alpha_deg = alpha_rad * RAD_TO_DEG;
        coeffs.beta_deg  = beta_rad * RAD_TO_DEG;

        const ControlSurfaces ctrl = controls.clamped();

        // 1. Longitudinal Interpolations: CL, CD, Cm
        const double cl_static = Interpolator::interp3d<
            TP1538Tables::N_ALPHA, TP1538Tables::N_DE, TP1538Tables::N_LEF>(
            TP1538Tables::ALPHA_GRID, TP1538Tables::ELEVATOR_GRID, TP1538Tables::LEF_GRID,
            TP1538Tables::TABLE_CL, coeffs.alpha_deg, ctrl.delta_e, ctrl.delta_lef
        );

        const double cd_static = Interpolator::interp3d<
            TP1538Tables::N_ALPHA, TP1538Tables::N_DE, TP1538Tables::N_LEF>(
            TP1538Tables::ALPHA_GRID, TP1538Tables::ELEVATOR_GRID, TP1538Tables::LEF_GRID,
            TP1538Tables::TABLE_CD, coeffs.alpha_deg, ctrl.delta_e, ctrl.delta_lef
        );

        const double cm_static = Interpolator::interp3d<
            TP1538Tables::N_ALPHA, TP1538Tables::N_DE, TP1538Tables::N_LEF>(
            TP1538Tables::ALPHA_GRID, TP1538Tables::ELEVATOR_GRID, TP1538Tables::LEF_GRID,
            TP1538Tables::TABLE_CM, coeffs.alpha_deg, ctrl.delta_e, ctrl.delta_lef
        );

        // Dynamic pitch damping factor: c_bar / (2 * V)
        const double v_safe = std::max(V, 1.0);
        const double lon_damping_factor = C_BAR / (2.0 * v_safe);
        const double q_rate = state.omega_b.y; // pitch rate [rad/s]

        const double cl_q = Interpolator::interp1d<TP1538Tables::N_ALPHA>(
            TP1538Tables::ALPHA_GRID, TP1538Tables::CL_Q, coeffs.alpha_deg
        );
        const double cd_q = Interpolator::interp1d<TP1538Tables::N_ALPHA>(
            TP1538Tables::ALPHA_GRID, TP1538Tables::CD_Q, coeffs.alpha_deg
        );
        const double cm_q = Interpolator::interp1d<TP1538Tables::N_ALPHA>(
            TP1538Tables::ALPHA_GRID, TP1538Tables::CM_Q, coeffs.alpha_deg
        );

        // Past 90 deg the jet is flying tail first: the wing is a flat plate
        // whose force stays normal to it, and the tail-first flow is unstable
        // in pitch, so the jet swaps ends. Continuous with the 90 deg row.
        double cl_s = cl_static, cd_s = cd_static, cm_s = cm_static;
        double cd_zero_lift = 0.0;
        const bool reversed = std::abs(coeffs.alpha_deg) > 90.0;
        if (reversed) {
            const double sa = std::sin(alpha_rad);
            const double ca = std::cos(alpha_rad);
            cl_s = CN_90 * sa * ca;
            cd_s = CN_90 * sa * sa + CD_REVERSED_MIN * ca * ca;
            cm_s = CM_90 * sa;
            cd_zero_lift = CD_REVERSED_MIN;
        } else if (mach > 0.0) {
            cd_zero_lift = Interpolator::interp3d<
                TP1538Tables::N_ALPHA, TP1538Tables::N_DE, TP1538Tables::N_LEF>(
                TP1538Tables::ALPHA_GRID, TP1538Tables::ELEVATOR_GRID, TP1538Tables::LEF_GRID,
                TP1538Tables::TABLE_CD, 0.0, ctrl.delta_e, ctrl.delta_lef
            );
        }

        coeffs.CL = cl_s + lon_damping_factor * cl_q * q_rate;
        coeffs.CD = cd_s + lon_damping_factor * cd_q * q_rate;
        coeffs.Cm = cm_s + lon_damping_factor * cm_q * q_rate;

        // 2. Lateral-Directional Interpolations: CY, Cl, Cn
        const double cy_static = Interpolator::interp2d<
            TP1538Tables::N_BETA, TP1538Tables::N_DR>(
            TP1538Tables::BETA_GRID, TP1538Tables::RUDDER_GRID,
            TP1538Tables::TABLE_CY, coeffs.beta_deg, ctrl.delta_r
        );

        const double cl_roll_static = Interpolator::interp3d<
            TP1538Tables::N_BETA, TP1538Tables::N_DA, TP1538Tables::N_DR>(
            TP1538Tables::BETA_GRID, TP1538Tables::AILERON_GRID, TP1538Tables::RUDDER_GRID,
            TP1538Tables::TABLE_CL_ROLL, coeffs.beta_deg, ctrl.delta_a, ctrl.delta_r
        );

        const double cn_yaw_static = Interpolator::interp3d<
            TP1538Tables::N_BETA, TP1538Tables::N_DA, TP1538Tables::N_DR>(
            TP1538Tables::BETA_GRID, TP1538Tables::AILERON_GRID, TP1538Tables::RUDDER_GRID,
            TP1538Tables::TABLE_CN_YAW, coeffs.beta_deg, ctrl.delta_a, ctrl.delta_r
        );

        // Lateral damping factor: b / (2 * V)
        const double lat_damping_factor = B_SPAN / (2.0 * v_safe);
        const double p_rate = state.omega_b.x; // roll rate [rad/s]
        const double r_rate = state.omega_b.z; // yaw rate [rad/s]

        const double cy_p = Interpolator::interp1d<TP1538Tables::N_ALPHA>(
            TP1538Tables::ALPHA_GRID, TP1538Tables::CY_P, coeffs.alpha_deg
        );
        const double cy_r = Interpolator::interp1d<TP1538Tables::N_ALPHA>(
            TP1538Tables::ALPHA_GRID, TP1538Tables::CY_R, coeffs.alpha_deg
        );
        const double cl_p = Interpolator::interp1d<TP1538Tables::N_ALPHA>(
            TP1538Tables::ALPHA_GRID, TP1538Tables::CL_P, coeffs.alpha_deg
        );
        const double cl_r = Interpolator::interp1d<TP1538Tables::N_ALPHA>(
            TP1538Tables::ALPHA_GRID, TP1538Tables::CL_R, coeffs.alpha_deg
        );
        const double cn_p = Interpolator::interp1d<TP1538Tables::N_ALPHA>(
            TP1538Tables::ALPHA_GRID, TP1538Tables::CN_P, coeffs.alpha_deg
        );
        const double cn_r = Interpolator::interp1d<TP1538Tables::N_ALPHA>(
            TP1538Tables::ALPHA_GRID, TP1538Tables::CN_R, coeffs.alpha_deg
        );

        coeffs.CY = cy_static + lat_damping_factor * (cy_p * p_rate + cy_r * r_rate);
        coeffs.Cl = cl_roll_static + lat_damping_factor * (cl_p * p_rate + cl_r * r_rate);
        coeffs.Cn = cn_yaw_static + lat_damping_factor * (cn_p * p_rate + cn_r * r_rate);

        // 3. Compressibility corrections.
        // Skipped entirely at mach = 0 so the incompressible table data is
        // reproduced exactly by callers that do not supply a Mach number.
        if (mach > 0.0) {
            const double m = std::clamp(mach, 0.0, 2.50);
            const double pg = prandtl_glauert(m);

            // Lift gains from compressibility only where the flow is
            // attached, is capped by the falling CLmax, and pays the drag of
            // the lift it actually makes (see CompressibleLift).
            const auto lift = CompressibleLift::apply(
                coeffs.CL, std::max(0.0, coeffs.CD - cd_zero_lift), CL_MAX_LOW_SPEED,
                coeffs.alpha_deg, pg, m);
            coeffs.CL = lift.cl;
            coeffs.CD = cd_zero_lift + lift.cd_lift;

            // Lateral coefficients scale with the PG factor.
            coeffs.CY *= pg;
            coeffs.Cl *= pg;
            coeffs.Cn *= pg;

            // Pitching moment scales, plus the transonic aft AC shift: a
            // nose-down trim change (Mach tuck) and a gain in static stability
            // as the aerodynamic centre moves aft of the CG. The table's
            // relaxed stability is a subsonic property; supersonic, the real
            // F-16 is stable, and left unstable here the FLCS fought a
            // divergent pitch oscillation at high dynamic pressure.
            coeffs.Cm *= pg;
            if (m > MACH_CRIT) {
                const double t = std::clamp((m - MACH_CRIT) / (MACH_PEAK - MACH_CRIT), 0.0, 1.0);
                coeffs.Cm += CM_TUCK_MAX * t * t * (3.0 - 2.0 * t);
            }
            const double t_ac = std::clamp((m - AC_SHIFT_START) / (AC_SHIFT_END - AC_SHIFT_START), 0.0, 1.0);
            coeffs.Cm -= AC_SHIFT_MAX * t_ac * t_ac * (3.0 - 2.0 * t_ac) * coeffs.CL;

            // Wave drag adds to the profile drag.
            coeffs.CD += wave_drag(m);
            coeffs.CD += dynamic_pressure_drag(q_bar);
        }

        // 4. Speedbrake increments (Mach-independent to first order).
        if (ctrl.speedbrake > 0.0) {
            coeffs.CD += CD_SPEEDBRAKE_FULL * ctrl.speedbrake;
            coeffs.Cm += CM_SPEEDBRAKE_FULL * ctrl.speedbrake;
        }

        return coeffs;
    }

    /**
     * @brief Computes total aerodynamic forces and moments in aircraft body frame.
     *
     * Transformation from stability/wind frame to body frame:
     *   Fx_b = q_bar * S * (CL * sin(alpha) - CD * cos(alpha))
     *   Fy_b = q_bar * S * CY
     *   Fz_b = q_bar * S * (-CL * cos(alpha) - CD * sin(alpha))
     *
     * Moments:
     *   Mx_b = q_bar * S * b * Cl (Roll)
     *   My_b = q_bar * S * c_bar * Cm (Pitch)
     *   Mz_b = q_bar * S * b * Cn (Yaw)
     *
     * @param state Current 6-DoF flight state
     * @param controls Control surface deflections [deg]
     * @param q_bar Dynamic pressure [Pa]
     * @return fdm::AircraftForces containing body force [N] and body moment [N*m]
     */
    [[nodiscard]] static fdm::AircraftForces compute_forces_and_moments(
        const fdm::FlightState& state,
        const ControlSurfaces& controls,
        double q_bar,
        double mach = 0.0
    ) noexcept {
        const AeroCoefficients coeffs = compute_coefficients(state, controls, q_bar, mach);

        const double alpha_rad = state.alpha();
        const double sin_a = std::sin(alpha_rad);
        const double cos_a = std::cos(alpha_rad);

        const double qS = coeffs.q_bar * S_REF;

        // Aerodynamic forces in body frame
        const double Fx_b = qS * (coeffs.CL * sin_a - coeffs.CD * cos_a);
        const double Fy_b = qS * coeffs.CY;
        const double Fz_b = qS * (-coeffs.CL * cos_a - coeffs.CD * sin_a);

        // Aerodynamic moments in body frame
        const double Mx_b = qS * B_SPAN * coeffs.Cl;
        const double My_b = qS * C_BAR * coeffs.Cm;
        const double Mz_b = qS * B_SPAN * coeffs.Cn;

        return {
            math::Vector3(Fx_b, Fy_b, Fz_b),
            math::Vector3(Mx_b, My_b, Mz_b)
        };
    }
};

} // namespace aero
} // namespace fastjet
