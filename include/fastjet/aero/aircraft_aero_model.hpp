#pragma once

#include "f16_aero_model.hpp"
#include "compressible_lift.hpp"
#include "../aircraft/aircraft_type.hpp"
#include "../aircraft/aircraft_config.hpp"
#include <cmath>
#include <algorithm>

namespace fastjet::aero {

/**
 * @brief Unified aerodynamic model supporting all six combat aircraft.
 * Delegates to NASA TP-1538 wind-tunnel lookup tables for F-16C, and utilizes
 * high-fidelity DATCOM / NASA TM dimensional polar synthesis with compressibility,
 * wave drag, damping derivatives, and high-alpha vortex lift for F-15EX, Typhoon, F-22, A-10 and F-35B.
 *
 * Zero dynamic heap allocation.
 */
class AircraftAeroModel {
public:
    aircraft::AircraftType aircraft_type{aircraft::AircraftType::F16_FIGHTING_FALCON};
    aircraft::AeroConfig config{};

    constexpr AircraftAeroModel() noexcept {
        configure(aircraft::AircraftType::F16_FIGHTING_FALCON);
    }

    explicit constexpr AircraftAeroModel(aircraft::AircraftType type) noexcept {
        configure(type);
    }

    constexpr void configure(aircraft::AircraftType type) noexcept {
        aircraft_type = type;
        config = aircraft::AircraftConfig::get(type).aero;
    }

    /**
     * @brief Prandtl-Glauert compressibility correction factor.
     *
     * Continuous across the whole Mach range: subsonic PG up to the table limit,
     * held on that plateau through the transonic region until the supersonic
     * Ackeret factor falls below it, then Ackeret — floored at 1.0 so behaviour
     * well above M1.4 is unchanged.  This used to switch to an Ackeret branch
     * bounded at M^2-1 >= 0.1 above a hard-coded M1.05 while the caller gated on
     * each airframe's own mach_peak (1.10-1.15), multiplying lift by up to 2.84
     * in that band and then dropping it to 1.0: 10-15 G spikes near Mach 1.1.
     */
    [[nodiscard]] double prandtl_glauert(double mach) const noexcept {
        constexpr double MACH_TABLE_LIMIT = 0.80;

        if (mach < MACH_TABLE_LIMIT) {
            return 1.0 / std::sqrt((std::max)(0.05, 1.0 - mach * mach));
        }
        const double plateau = 1.0 / std::sqrt(1.0 - MACH_TABLE_LIMIT * MACH_TABLE_LIMIT);
        if (mach <= 1.0) {
            return plateau;
        }
        const double ackeret = 1.0 / std::sqrt((std::max)(mach * mach - 1.0, 1e-6));
        return std::clamp(ackeret, 1.0, plateau);
    }

    [[nodiscard]] static constexpr double smoothstep(double t) noexcept {
        t = std::clamp(t, 0.0, 1.0);
        return t * t * (3.0 - 2.0 * t);
    }

    /**
     * @brief Transonic and supersonic wave drag increment.
     */
    [[nodiscard]] double wave_drag(double mach) const noexcept {
        if (mach <= config.mach_crit) return 0.0;

        if (mach <= config.mach_peak) {
            const double t = (mach - config.mach_crit) / (config.mach_peak - config.mach_crit);
            const double wave = config.cd_wave_peak * t * t * (3.0 - 2.0 * t);
            if (aircraft_type == aircraft::AircraftType::A10_THUNDERBOLT && mach > 0.70) {
                const double a10_rise = (mach - 0.70) / 0.15;
                return wave + 0.045 * a10_rise * a10_rise;
            }
            return wave;
        }

        const double decay = std::exp(-(mach - config.mach_peak) / 0.45);
        const double base = config.cd_wave_super + (config.cd_wave_peak - config.cd_wave_super) * decay;
        const double excess = (std::max)(0.0, mach - config.mach_peak);

        // A-10 is purely subsonic: wave drag climbs catastrophically past M 0.75
        if (aircraft_type == aircraft::AircraftType::A10_THUNDERBOLT) {
            return base + 0.15 * excess * excess;
        }

        return base + 0.006 * excess * excess;
    }

    /**
     * @brief Dynamic pressure drag rise near structural airspeed limits (~800 KEAS).
     */
    [[nodiscard]] double dynamic_pressure_drag(double q_bar) const noexcept {
        constexpr double Q_DRAG_ONSET = 60000.0; // 60 kPa onset
        if (q_bar <= Q_DRAG_ONSET) return 0.0;
        const double excess = (q_bar - Q_DRAG_ONSET) / Q_DRAG_ONSET;
        return 0.095 * excess * excess;
    }

    /**
     * @brief Computes non-dimensional aerodynamic coefficients for the active aircraft.
     */
    [[nodiscard]] AeroCoefficients compute_coefficients(
        const fdm::FlightState& state,
        const ControlSurfaces& controls,
        double q_bar,
        double mach = 0.0
    ) const noexcept {
        // For F-16C, utilize the exact NASA TP-1538 tables
        if (aircraft_type == aircraft::AircraftType::F16_FIGHTING_FALCON) {
            return F16AeroModel::compute_coefficients(state, controls, q_bar, mach);
        }

        AeroCoefficients coeffs{};
        coeffs.q_bar = q_bar;
        coeffs.mach = mach;

        constexpr double RAD_TO_DEG = 180.0 / M_PI;
        const double V = state.airspeed();
        const double v_safe = (std::max)(V, 1.0);
        const double alpha_rad = state.alpha();
        const double beta_rad  = state.beta();

        coeffs.alpha_deg = alpha_rad * RAD_TO_DEG;
        coeffs.beta_deg  = beta_rad * RAD_TO_DEG;

        const ControlSurfaces ctrl = controls.clamped();

        constexpr double DEG_TO_RAD = M_PI / 180.0;
        const double sin_a = std::sin(alpha_rad);
        const double cos_a = std::cos(alpha_rad);

        // 1. Low-speed lift and drag-due-to-lift against alpha, through the stall.
        // Attached flow: the lift line, plus leading-edge vortex lift on the
        // Typhoon (canard-delta) and F-22 (chined diamond wing), plus the
        // stabilator's own lift.
        double cl_vortex = 0.0;
        if (aircraft_type == aircraft::AircraftType::EUROFIGHTER_TYPHOON ||
            aircraft_type == aircraft::AircraftType::F22_RAPTOR) {
            // Polhamus leading-edge suction analogy for slender/delta wings:
            // CL_vortex ~ K_v * sin^2(alpha) * cos(alpha)
            constexpr double K_V = 2.4;
            cl_vortex = K_V * sin_a * std::abs(sin_a) * cos_a;
        }
        const double cl_delta_e = 0.40 * (ctrl.delta_e * DEG_TO_RAD);
        const double cl_linear = config.cl_alpha * alpha_rad + cl_vortex + cl_delta_e;

        // x: attached lift over CLmax. The lift rounds off to CLmax by
        // stall_peak_x, holds, then falls away to a flat plate between
        // stall_break_x and stall_break_x + stall_width_x. Beyond 90 deg the
        // jet is flying tail first and is a flat plate throughout.
        const double x = std::abs(cl_linear) / config.cl_max;
        const double stall = std::abs(coeffs.alpha_deg) >= 90.0
            ? 1.0 : smoothstep((x - config.stall_break_x) / config.stall_width_x);
        const double peak_onset = 2.0 - config.stall_peak_x;
        double cl_attached = x;
        if (x >= config.stall_peak_x) {
            cl_attached = 1.0;
        } else if (x > peak_onset) {
            cl_attached = x - (x - peak_onset) * (x - peak_onset) / (4.0 * (1.0 - peak_onset));
        }
        cl_attached = std::copysign(cl_attached * config.cl_max, cl_linear);

        const double cl_plate = config.cn_90 * sin_a * cos_a;
        const double cl_low_speed = (1.0 - stall) * cl_attached + stall * cl_plate;

        // Drag due to lift as K * cl_alpha * CL * tan(alpha): on the lift line
        // (tan(alpha) ~ CL / cl_alpha) this is exactly the polar's K * CL^2,
        // but it keeps climbing with alpha once the lift is capped, as the
        // leading-edge suction is lost and the force leans back towards the
        // normal of the wing. It then blends to the flat plate.
        const double tan_a = std::tan(std::clamp(alpha_rad, -1.48, 1.48));
        const double cd_lift_attached = config.k_induced * config.cl_alpha * std::abs(cl_attached * tan_a);
        const double cd_lift_low_speed = (1.0 - stall) * cd_lift_attached
                                       + stall * config.cn_90 * sin_a * sin_a;

        // 2. Compressibility on lift and drag-due-to-lift together.
        double cd_lift = cd_lift_low_speed;
        coeffs.CL = cl_low_speed;
        if (mach > 0.0) {
            const auto lift = CompressibleLift::apply(cl_low_speed, cd_lift_low_speed, config.cl_max,
                                                      coeffs.alpha_deg, prandtl_glauert(mach), mach);
            coeffs.CL = lift.cl;
            cd_lift = lift.cd_lift;
        }

        coeffs.CD = config.cd0 + cd_lift;
        if (mach > 0.0) {
            coeffs.CD += wave_drag(mach);
            coeffs.CD += dynamic_pressure_drag(q_bar);
        }

        // Speedbrake increment
        if (ctrl.speedbrake > 0.0) {
            coeffs.CD += config.cd_speedbrake * ctrl.speedbrake;
        }

        // 3. Pitching Moment (Cm)
        // Static margin stability + pitch control surface authority
        // Subsonic delta-canard (Typhoon) and F-22 have relaxed static stability
        double static_margin = 0.05; // Standard 5% positive for F-15 and A-10
        if (aircraft_type == aircraft::AircraftType::EUROFIGHTER_TYPHOON ||
            aircraft_type == aircraft::AircraftType::F22_RAPTOR) {
            // Transonic aerodynamic-centre shift: unstable subsonic, stable
            // supersonic. The AC moves aft progressively through the transonic
            // region; a step at M1.0 kicked the jet's pitch trim as it crossed.
            static_margin = -0.08 + 0.14 * smoothstep((mach - 0.95) / 0.20);
        } else if (aircraft_type == aircraft::AircraftType::F35B_LIGHTNING_II) {
            // F-35: relaxed stability, ~5% MAC unstable subsonic, ~10% stable supersonic.
            static_margin = -0.05 + 0.15 * smoothstep((mach - 0.95) / 0.20);
        }
        const double cm_alpha = -config.cl_alpha * static_margin;
        const double cm_delta_e = -0.90 * (ctrl.delta_e * DEG_TO_RAD); // Pitch control surface power
        const double lon_damping = config.c_bar / (2.0 * v_safe);
        const double cm_q = config.pitch_damping * (lon_damping * state.omega_b.y);

        // Once stalled the wing's centre of pressure moves to mid-chord, well
        // behind the CG: a nose-down moment that grows to cm_90 at 90 deg and
        // swaps the ends of a jet flying tail first. The tail keeps a little
        // authority in the wing's wake.
        const double cm_static = (1.0 - stall) * (cm_alpha * alpha_rad)
                               + stall * (-config.cm_90 * sin_a);
        const double tail_power = 1.0 - 0.6 * stall;
        coeffs.Cm = cm_static + tail_power * cm_delta_e + cm_q;

        // Speedbrake pitch increment
        if (ctrl.speedbrake > 0.0) {
            coeffs.Cm += config.cm_speedbrake * ctrl.speedbrake;
        }

        // 4. Lateral-Directional Coefficients (CY, Cl, Cn)
        const double lat_damping = config.b_span / (2.0 * v_safe);
        const double p_rate = state.omega_b.x;
        const double r_rate = state.omega_b.z;

        // Sideforce CY
        coeffs.CY = -0.75 * beta_rad + 0.18 * (ctrl.delta_r * DEG_TO_RAD);

        // Roll moment Cl
        // Dihedral effect + aileron/flaperon power + roll damping
        coeffs.Cl = -0.12 * beta_rad - 0.22 * (ctrl.delta_a * DEG_TO_RAD) +
                    config.roll_damping * (lat_damping * p_rate);

        // Yaw moment Cn
        // Directional stability + rudder power + yaw damping
        coeffs.Cn = +0.16 * beta_rad - 0.10 * (ctrl.delta_r * DEG_TO_RAD) +
                    config.yaw_damping * (lat_damping * r_rate);

        // Compressibility scaling of the moments and side force (continuous in
        // Mach). Lift was scaled above, together with its drag.
        if (mach > 0.0) {
            const double pg = prandtl_glauert(mach);
            coeffs.CY *= pg;
            coeffs.Cl *= pg;
            coeffs.Cn *= pg;
            coeffs.Cm *= pg;
        }

        return coeffs;
    }

    /**
     * @brief Computes total aerodynamic forces and moments in aircraft body frame.
     */
    [[nodiscard]] fdm::AircraftForces compute_forces_and_moments(
        const fdm::FlightState& state,
        const ControlSurfaces& controls,
        double q_bar,
        double mach = 0.0
    ) const noexcept {
        // Delegate F-16 directly
        if (aircraft_type == aircraft::AircraftType::F16_FIGHTING_FALCON) {
            return F16AeroModel::compute_forces_and_moments(state, controls, q_bar, mach);
        }

        const AeroCoefficients coeffs = compute_coefficients(state, controls, q_bar, mach);

        const double alpha_rad = state.alpha();
        const double sin_a = std::sin(alpha_rad);
        const double cos_a = std::cos(alpha_rad);

        const double qS = coeffs.q_bar * config.s_ref;

        // Forces in body frame
        const double Fx_b = qS * (coeffs.CL * sin_a - coeffs.CD * cos_a);
        const double Fy_b = qS * coeffs.CY;
        const double Fz_b = qS * (-coeffs.CL * cos_a - coeffs.CD * sin_a);

        // Moments in body frame
        const double Mx_b = qS * config.b_span * coeffs.Cl;
        const double My_b = qS * config.c_bar * coeffs.Cm;
        const double Mz_b = qS * config.b_span * coeffs.Cn;

        return {
            math::Vector3(Fx_b, Fy_b, Fz_b),
            math::Vector3(Mx_b, My_b, Mz_b)
        };
    }
};

} // namespace fastjet::aero
