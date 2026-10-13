#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace fastjet::aero {

/**
 * @brief Mach and stall effects on lift and drag-due-to-lift, shared by the
 * F-16 table model and the synthesised polars of the other four airframes.
 *
 * Both models start from a low-speed lift and drag curve against alpha. This
 * turns that into the lift and drag at speed, in three steps:
 *
 *  1. Compressibility raises the lift-curve slope (Prandtl-Glauert), but only
 *     where the flow is attached. Separated flow at high alpha gets no gain.
 *  2. Shock-induced separation lowers the maximum usable lift coefficient as
 *     Mach rises. A clean fighter keeps roughly 3/4 of its low-speed CLmax at
 *     M0.9 and about 3/5 at M1.2.
 *  3. Drag-due-to-lift follows the lift actually made, not the alpha. A given
 *     CL costs at least its low-speed drag at any subsonic Mach, and more once
 *     the wing loses leading-edge suction transonically.
 *
 * Applying Prandtl-Glauert to CL alone made every jet more efficient as it
 * approached Mach 1 (L/D at a fixed alpha rose from ~7 at M0.3 to ~11 at
 * M0.85). Fighters then sustained 8-9 G at 30,000 ft, where real ones hold
 * 3.5-4.5 G.
 */
struct CompressibleLift {
    /// Below this alpha the flow is attached and gets the full Prandtl-Glauert gain.
    static constexpr double ATTACHED_ALPHA_DEG  = 12.0;
    /// By this alpha the flow is separated and gets none of it.
    static constexpr double SEPARATED_ALPHA_DEG = 30.0;
    /// The Mach lift cap sits this far above the low-speed CLmax, so at low
    /// Mach it only rounds off the stall peak instead of lowering it.
    static constexpr double CAP_MARGIN = 1.03;
    /// Lift starts to round off towards the cap at this fraction of it.
    static constexpr double SOFT_ONSET = 0.85;

    /// @brief Share of the Prandtl-Glauert gain that applies at @p alpha_deg.
    [[nodiscard]] static double attached_fraction(double alpha_deg) noexcept {
        const double a = std::abs(alpha_deg);
        const double t = std::clamp((a - ATTACHED_ALPHA_DEG) / (SEPARATED_ALPHA_DEG - ATTACHED_ALPHA_DEG), 0.0, 1.0);
        return 1.0 - t * t * (3.0 - 2.0 * t);
    }

    /// @brief Prandtl-Glauert factor applied at @p alpha_deg.
    [[nodiscard]] static double effective_gain(double pg, double alpha_deg) noexcept {
        return 1.0 + (pg - 1.0) * attached_fraction(alpha_deg);
    }

    /// @brief Maximum usable lift against Mach, as a fraction of the low-speed value.
    ///
    /// Shock-induced separation and buffet: a generic clean-fighter trend
    /// (F-16 / F-15 wind-tunnel and flight-test buffet boundaries).
    [[nodiscard]] static double clmax_mach_factor(double mach) noexcept {
        struct P { double m, f; };
        static constexpr std::array<P, 11> table{{{0.0, 1.00}, {0.40, 1.00}, {0.50, 0.98}, {0.60, 0.94},
                                                  {0.70, 0.88}, {0.80, 0.81}, {0.90, 0.74}, {1.00, 0.69},
                                                  {1.20, 0.62}, {1.60, 0.53}, {2.00, 0.46}}};
        if (mach <= table.front().m) return table.front().f;
        for (size_t i = 1; i < table.size(); ++i) {
            if (mach <= table[i].m) {
                const double t = (mach - table[i - 1].m) / (table[i].m - table[i - 1].m);
                return table[i - 1].f + t * (table[i].f - table[i - 1].f);
            }
        }
        return table.back().f;
    }

    /// @brief Growth of drag-due-to-lift with Mach.
    ///
    /// Leading-edge suction collapses through the transonic region; by M1.2
    /// the lift force leans back almost normal to the wing, and supersonic
    /// wave drag due to lift keeps growing with Mach.
    [[nodiscard]] static double induced_mach_factor(double mach) noexcept {
        if (mach <= 0.75) return 1.0;
        if (mach <= 1.05) {
            const double t = (mach - 0.75) / 0.30;
            return 1.0 + 0.8 * t * t * (3.0 - 2.0 * t);
        }
        return 1.8 + 0.8 * std::min(mach - 1.05, 1.5);
    }

    /// @brief Smoothly limit @p cl to +/- @p cap: identity up to SOFT_ONSET of
    /// the cap, then a quadratic that meets the cap with zero slope.
    [[nodiscard]] static double soft_cap(double cl, double cap) noexcept {
        if (cap <= 0.0) return 0.0;
        const double m = std::abs(cl);
        const double a = SOFT_ONSET * cap;
        const double b = (2.0 - SOFT_ONSET) * cap;
        double out = m;
        if (m >= b) {
            out = cap;
        } else if (m > a) {
            out = m - (m - a) * (m - a) / (4.0 * (1.0 - SOFT_ONSET) * cap);
        }
        return cl < 0.0 ? -out : out;
    }

    /// @brief Lift and drag-due-to-lift at speed.
    struct Result {
        double cl;       ///< Lift coefficient at this Mach
        double cd_lift;  ///< Drag due to lift at this Mach
    };

    /**
     * @param cl_ls       Low-speed lift coefficient at this alpha
     * @param cd_lift_ls  Low-speed drag due to lift at this alpha (total drag less zero-lift drag)
     * @param clmax_ls    Low-speed maximum lift coefficient of the airframe
     * @param alpha_deg   Angle of attack
     * @param pg          Prandtl-Glauert factor at this Mach
     * @param mach        Mach number
     */
    [[nodiscard]] static Result apply(double cl_ls, double cd_lift_ls, double clmax_ls, double alpha_deg,
                                      double pg, double mach) noexcept {
        const double gain = effective_gain(pg, alpha_deg);
        const double cl_boosted = gain * cl_ls;
        const double cap = CAP_MARGIN * clmax_ls * clmax_mach_factor(mach);
        const double cl = soft_cap(cl_boosted, cap);
        // Lift at speed over lift at low speed, from the same alpha. Worked out
        // from the cap's own ratio so a near-zero cl_ls cannot blow it up.
        const double kept = std::abs(cl_boosted) > 1e-9 ? cl / cl_boosted : 1.0;
        const double ratio = gain * kept;
        // More lift from the same alpha costs the drag of that lift (drag
        // follows CL, roughly as CL^2). Less lift (the cap) leaves the
        // separated-flow drag of that alpha in place.
        const double cd_lift = cd_lift_ls * std::max(1.0, ratio * ratio) * induced_mach_factor(mach);
        return {cl, cd_lift};
    }
};

} // namespace fastjet::aero
