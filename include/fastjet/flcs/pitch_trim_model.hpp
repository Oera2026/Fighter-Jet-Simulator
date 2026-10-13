#pragma once

#include "imu_sensor.hpp"
#include "../aero/aircraft_aero_model.hpp"
#include "../environment/atmosphere1976.hpp"
#include <algorithm>
#include <cmath>

namespace fastjet::flcs {

/**
 * @brief Onboard pitch model: the stabilator a steady pull at a given G needs.
 *
 * The G-command laws used to find that trim with their integrator alone. At
 * 1 G that is harmless, but every pull needs a different trim, so the jet had
 * to miss the command before the integrator learnt it: a stable jet crept up
 * to the G and hung on after release; a relaxed-stability one (Typhoon, F-22,
 * which need nose-DOWN stabilator to hold more G) overshot the G, then dipped
 * toward 0 G on release while the trim unwound.
 *
 * A real FBW carries this as gain-scheduled trim tables. Here it comes from
 * the jet's own aero model: two Newton steps a tick, warm-started from the
 * last solution, solve for the alpha and stabilator at which the normal force
 * gives the commanded G and the pitching moment (with the steady pull's
 * pitch-rate damping) balances. The control law feeds that forward; its
 * integrator is left with only what the model does not know (thrust and
 * thrust-vectoring moments, CG).
 *
 * Zero heap allocation.
 */
/**
 * @brief Second-order G-command reference model: the G trajectory the pitch
 * law asks the airframe to fly.
 *
 * A first-order 40 ms prefilter asked for the new G far faster than any
 * airframe can deliver it, so the proportional term and the damper kicked
 * the stabilator hard and the G overshot. The reference moves at a pace the
 * jet can follow (natural frequency scheduled on dynamic pressure, since
 * pitch authority grows with it), barely overshoots (ZETA), and the law
 * tracks it: its feed-forward, pitch-rate reference and G error all come
 * from the same trajectory.
 */
struct GReferenceModel {
    static constexpr double ZETA = 0.85;

    double nz{1.0};       ///< Reference G
    double nz_rate{0.0};  ///< Its rate [G/s]

    constexpr void reset(double nz0 = 1.0) noexcept { nz = nz0; nz_rate = 0.0; }

    /// @brief Natural frequency [rad/s] at dynamic pressure @p q_bar.
    [[nodiscard]] static double natural_frequency(double q_bar) noexcept {
        return std::clamp(8.0 * std::sqrt(std::max(q_bar, 0.0) / 20000.0), 4.5, 11.0);
    }

    double step(double dt, double target, double wn) noexcept {
        if (dt <= 0.0) return nz;
        // Semi-implicit Euler: stable at 200 Hz for any wn used here.
        nz_rate += (wn * wn * (target - nz) - 2.0 * ZETA * wn * nz_rate) * dt;
        nz += nz_rate * dt;
        return nz;
    }
};

class PitchTrimModel {
public:
    aero::AircraftAeroModel aero{};

    PitchTrimModel() noexcept = default;
    explicit PitchTrimModel(aircraft::AircraftType type) noexcept : aero(type) {}

    void configure(aircraft::AircraftType type) noexcept {
        aero.configure(type);
        reset();
    }

    void reset() noexcept {
        alpha_deg_ = 3.0;
        de_deg_ = 0.0;
        alpha_prev_ = 3.0;
        alpha_rate_ = 0.0;
        primed_ = false;
        solved_once_ = false;
    }

    /// @brief Feed-forward stabilator [deg] for the reference G @p nz, and
    /// the pitch rate [deg/s] the jet flies while following it: the steady
    /// pull's rate plus the rate at which the solved alpha is changing.
    double feed_forward(double dt, double nz, const IMUData& imu, double mass_kg, double alpha_max_deg,
                        double de_limit_deg, double& q_ref_dps,
                        double (*lef_schedule)(double, double) = nullptr) noexcept {
        constexpr double G0 = 9.80665;
        const double de = stabilator_for(nz, imu, mass_kg, alpha_max_deg, de_limit_deg, lef_schedule);
        if (!primed_) {
            alpha_prev_ = alpha_deg_;
            primed_ = true;
        }
        if (dt > 0.0) {
            // No alpha rate is asked for while the solve sits at its alpha
            // cap, and the rate is bounded: a jump in the solved alpha must
            // not become a pitch-rate spike.
            const double raw = saturated_ ? 0.0 : std::clamp((alpha_deg_ - alpha_prev_) / dt, -ALPHA_RATE_MAX, ALPHA_RATE_MAX);
            alpha_rate_ += (raw - alpha_rate_) * (1.0 - std::exp(-dt / 0.03));
        }
        alpha_prev_ = alpha_deg_;
        // Gravity's share along the body z axis (imu.gz) is part of the steady
        // rate: holding 1 G inverted the jet pitches at 2 g / V. Assuming
        // wings level, the damper fought that rate and every full roll sagged
        // to -0.4 G on its back.
        q_ref_dps = (180.0 / M_PI) * G0 * (nz - imu.gz) / std::max(imu.airspeed, 50.0) + alpha_rate_;
        return de;
    }

    /**
     * @brief Stabilator [deg] that holds a steady wings-level pull at @p nz.
     *
     * @param alpha_max_deg Alpha is not solved beyond this: a G the wing
     *        cannot reach below it is left to the AoA limiter.
     * @param de_limit_deg Stabilator authority.
     * @param lef_schedule Leading-edge flap schedule (F-16), or null.
     */
    [[nodiscard]] double stabilator_for(double nz, const IMUData& imu, double mass_kg, double alpha_max_deg,
                                        double de_limit_deg,
                                        double (*lef_schedule)(double, double) = nullptr) noexcept {
        constexpr double G0 = 9.80665;
        constexpr double D2R = M_PI / 180.0;
        const double V = std::max(imu.airspeed, 30.0);
        if (imu.q_bar < 300.0 || mass_kg <= 0.0) return de_deg_;

        const double mach = environment::Atmosphere1976::compute(imu.altitude, V).mach_number;
        const double cn_req = nz * mass_kg * G0 / (imu.q_bar * aero.config.s_ref);
        const double q_rad = G0 * (nz - imu.gz) / V;

        auto residual = [&](double a_deg, double de, double& f_cn, double& f_cm) noexcept {
            fdm::FlightState s{};
            s.vel_b = math::Vector3(V * std::cos(a_deg * D2R), 0.0, V * std::sin(a_deg * D2R));
            s.omega_b = math::Vector3(0.0, q_rad, 0.0);
            const double lef = lef_schedule ? lef_schedule(a_deg, imu.q_bar) : 0.0;
            const auto c = aero.compute_coefficients(s, aero::ControlSurfaces{de, 0.0, 0.0, lef}, imu.q_bar, mach);
            const double a = a_deg * D2R;
            f_cn = c.CL * std::cos(a) + c.CD * std::sin(a) - cn_req;
            f_cm = c.Cm;
        };

        // Alternate one-dimensional Newton steps: alpha for the normal force,
        // then stabilator for the moment. The stabilator's own lift is a small
        // share of the total, so this converges in a step or two, and unlike
        // a joint 2x2 Newton step it stays well-posed at the lift peak, where
        // the normal force stops responding to alpha.
        constexpr double ALPHA_LO = -12.0;
        constexpr double EA = 0.25, ED = 0.25;
        constexpr double MIN_SLOPE = 0.02;  // [1/deg]: below this, the lift peak
        saturated_ = false;
        // Warm-started, two steps a tick keep up; cold, solve it out in full
        // so the first tick does not feed forward a guess.
        const int iterations = solved_once_ ? 2 : 12;
        solved_once_ = true;
        for (int it = 0; it < iterations; ++it) {
            double f1 = 0.0, f2 = 0.0, f1a = 0.0, f2a = 0.0;
            residual(alpha_deg_, de_deg_, f1, f2);
            residual(alpha_deg_ + EA, de_deg_, f1a, f2a);
            double slope = (f1a - f1) / EA;
            if (slope < MIN_SLOPE) {
                // Past the lift peak more alpha buys no more G: if more is
                // wanted it is out of reach, so hold alpha where it is.
                if (f1 < 0.0) {
                    saturated_ = true;
                } else {
                    slope = MIN_SLOPE;
                }
            }
            if (!saturated_) {
                alpha_deg_ = std::clamp(alpha_deg_ - std::clamp(f1 / slope, -4.0, 4.0), ALPHA_LO, alpha_max_deg);
            }
            if (alpha_deg_ >= alpha_max_deg || alpha_deg_ <= ALPHA_LO) saturated_ = true;

            double f1d = 0.0, f2d = 0.0;
            residual(alpha_deg_, de_deg_, f1, f2);
            residual(alpha_deg_, de_deg_ + ED, f1d, f2d);
            const double moment_slope = (f2d - f2) / ED;
            if (std::abs(moment_slope) > 1e-9) {
                de_deg_ = std::clamp(de_deg_ - std::clamp(f2 / moment_slope, -6.0, 6.0), -de_limit_deg, de_limit_deg);
            }
        }
        return de_deg_;
    }

    [[nodiscard]] double solved_alpha_deg() const noexcept { return alpha_deg_; }
    /// @brief True when the last solve could not reach the G (alpha capped).
    [[nodiscard]] bool saturated() const noexcept { return saturated_; }

    /// Bound on the alpha rate fed into the pitch-rate reference [deg/s].
    static constexpr double ALPHA_RATE_MAX = 20.0;

private:
    double alpha_deg_{3.0};
    double de_deg_{0.0};
    double alpha_prev_{3.0};
    double alpha_rate_{0.0};  ///< Rate of the solved alpha [deg/s]
    bool primed_{false};
    bool saturated_{false};
    bool solved_once_{false};
};

} // namespace fastjet::flcs
