#pragma once

#include <algorithm>
#include <cmath>

namespace fastjet {
namespace flcs {

/**
 * @brief Hydraulic Actuator Dynamics Model.
 * Implements first-order lag filter with strict rate and deflection limits:
 *   \dot{\delta} = (1 / tau) * (\delta_cmd - \delta)
 * subject to:
 *   |\dot{\delta}| <= rate_limit
 *   delta_min <= \delta <= delta_max
 *
 * Zero dynamic heap allocation.
 */
class Actuator {
public:
    double tau{0.05};          // Time constant [seconds]
    double rate_limit{60.0};   // Slew rate limit [deg/s]
    double pos_min{-25.0};     // Lower deflection limit [deg]
    double pos_max{+25.0};     // Upper deflection limit [deg]

    double position{0.0};      // Current deflection position [deg]
    double rate{0.0};          // Current deflection rate [deg/s]
    bool is_rate_limited{false};
    bool is_pos_limited{false};

    constexpr Actuator() noexcept = default;

    constexpr Actuator(double tau_s, double max_rate_dps, double min_pos_deg, double max_pos_deg, double init_pos = 0.0) noexcept
        : tau(tau_s),
          rate_limit(max_rate_dps),
          pos_min(min_pos_deg),
          pos_max(max_pos_deg),
          position(init_pos),
          rate(0.0) {}

    /**
     * @brief Creates standard F-16 Stabilator (Horizontal Tail / Elevator) actuator:
     * tau = 0.05s, rate limit = 60 deg/s, deflection limits = [-25, +25] deg.
     */
    static constexpr Actuator create_stabilator(double init_deg = 0.0) noexcept {
        return Actuator(0.05, 60.0, -25.0, 25.0, init_deg);
    }

    /**
     * @brief Creates the roll-surface actuator: the flaperons together with the
     * differential tail the TP-1538 aileron derivative lumps in.
     * tau = 0.05s, rate limit = 80 deg/s (Stevens & Lewis' lumped aileron),
     * deflection limits = [-20, +20] deg.
     *
     * At the flaperon's own 52 deg/s the roll surface needed 0.6 s to swing
     * from a full-rate roll to the opposite stop, and every jet rolled on
     * 100-180 deg after the stick came back to centre.
     */
    static constexpr Actuator create_flaperon(double init_deg = 0.0) noexcept {
        return Actuator(0.05, 80.0, -20.0, 20.0, init_deg);
    }

    /**
     * @brief Creates standard F-16 Rudder actuator:
     * tau = 0.05s, rate limit = 120 deg/s, deflection limits = [-30, +30] deg.
     */
    static constexpr Actuator create_rudder(double init_deg = 0.0) noexcept {
        return Actuator(0.05, 120.0, -30.0, 30.0, init_deg);
    }

    /**
     * @brief Steps actuator state forward by dt seconds.
     *
     * @param command Commanded surface deflection [deg]
     * @param dt Time step [s]
     * @return double Updated surface position [deg]
     */
    double step(double command, double dt) noexcept {
        if (dt <= 0.0) return position;

        // 1. Desired rate from first-order lag: (command - position) / tau
        const double safe_tau = std::max(tau, 1e-4);
        double desired_rate = (command - position) / safe_tau;

        // 2. Apply rate limiting: -rate_limit <= rate <= +rate_limit
        if (desired_rate > rate_limit) {
            desired_rate = rate_limit;
            is_rate_limited = true;
        } else if (desired_rate < -rate_limit) {
            desired_rate = -rate_limit;
            is_rate_limited = true;
        } else {
            is_rate_limited = false;
        }
        rate = desired_rate;

        // 3. Integrate position: position += rate * dt
        double new_pos = position + rate * dt;

        // 4. Apply position bounds: pos_min <= position <= pos_max
        if (new_pos > pos_max) {
            new_pos = pos_max;
            rate = 0.0;
            is_pos_limited = true;
        } else if (new_pos < pos_min) {
            new_pos = pos_min;
            rate = 0.0;
            is_pos_limited = true;
        } else {
            is_pos_limited = false;
        }

        position = new_pos;
        return position;
    }

    void reset(double pos = 0.0) noexcept {
        position = std::clamp(pos, pos_min, pos_max);
        rate = 0.0;
        is_rate_limited = false;
        is_pos_limited = false;
    }
};

} // namespace flcs
} // namespace fastjet
