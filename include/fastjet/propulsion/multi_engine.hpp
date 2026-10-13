#pragma once

#include "../input/avionics_controls.hpp"
#include "../environment/atmosphere1976.hpp"
#include "../aircraft/aircraft_type.hpp"
#include "../aircraft/aircraft_config.hpp"
#include <algorithm>
#include <cmath>

namespace fastjet::propulsion {

/**
 * @brief High-fidelity propulsion model supporting single and twin installations,
 * dry turbofans (A-10), supersonic afterburning turbofans (F-16, F-15EX, Typhoon, F-22),
 * altitude/Mach lapse, spool dynamics, nacelle pitch-thrust coupling, 2D thrust vectoring,
 * and the F135-PW-600's STOVL power mode (shaft power to the LiftFan raises total thrust).
 *
 * Zero dynamic heap allocation; safe to execute at 200 Hz in the hot physics loop.
 */
class MultiEngine {
public:
    aircraft::AircraftType aircraft_type{aircraft::AircraftType::F16_FIGHTING_FALCON};
    aircraft::PropulsionConfig config{};
    aircraft::StovlConfig stovl_config{};

    double commanded_thrust_n{0.0};
    double net_thrust_n{0.0};
    /// Thrust from the core (dry) spool, which accelerates slowly.
    double core_thrust_n{0.0};
    /// Afterburner increment on top of the core. It lights only once the core
    /// is near military power, then comes in quickly.
    double ab_thrust_n{0.0};
    double fuel_kg{3175.1};
    double fuel_flow_kgs{0.0};

    // Detent and lever power state
    input::ThrottleController::DetentState detent_state{input::ThrottleController::DetentState::IDLE};
    double dry_power{0.0};
    double ab_power{0.0};

    // Thrust vectoring pitch command [-1.0, +1.0] -> [-max_deg, +max_deg]
    double tvc_pitch_cmd{0.0};
    double tvc_pitch_actual_deg{0.0};

    /// LiftFan clutch engagement [0, 1], set each step from the lift system.
    /// Engaged, the LP turbine drives the fan and the engine's maximum dry
    /// thrust rises toward the total STOVL lift rating; net thrust is then the
    /// whole lift system's (StovlLiftSystem splits it).
    double lift_fan_clutch{0.0};
    /// STOVL engine mode: reheat locked out and the fast small-signal thrust
    /// response the jet-borne height loop needs.
    bool stovl_mode{false};

    /// Lever detents: cutoff, idle and military power.
    static constexpr double DETENT_CUTOFF = 0.03;
    static constexpr double DETENT_IDLE   = 0.08;
    static constexpr double DETENT_MIL    = 0.85;

    /// The afterburner lights once the core reaches this share of military
    /// thrust (~90% RPM on a low-bypass turbofan).
    static constexpr double AB_LIGHT_CORE_FRACTION = 0.80;
    /// Afterburner shut-down time constant [s].
    static constexpr double AB_CANCEL_TAU_S = 0.3;

    constexpr MultiEngine() noexcept {
        configure(aircraft::AircraftType::F16_FIGHTING_FALCON);
    }

    explicit constexpr MultiEngine(aircraft::AircraftType type) noexcept {
        configure(type);
    }

    /// @brief Installs a different powerplant.
    ///
    /// @param type          Airframe whose propulsion installation to adopt.
    /// @param preserve_fuel Carry the current fuel *fraction* across the swap
    ///                      rather than filling the new tanks. A mid-flight
    ///                      airframe change would otherwise silently refuel the
    ///                      aircraft, so an in-flight switch passes true and
    ///                      only a fresh start or reset fills the tanks.
    constexpr void configure(aircraft::AircraftType type, bool preserve_fuel = false) noexcept {
        const double prev_capacity =
            aircraft::AircraftConfig::get(aircraft_type).mass.internal_fuel_capacity_kg;
        const double prev_fraction =
            (prev_capacity > 0.0) ? (fuel_kg / prev_capacity) : 1.0;

        aircraft_type = type;
        const auto a_cfg = aircraft::AircraftConfig::get(type);
        config = a_cfg.propulsion;
        stovl_config = a_cfg.stovl;
        const double new_capacity = a_cfg.mass.internal_fuel_capacity_kg;
        fuel_kg = preserve_fuel
            ? std::clamp(prev_fraction, 0.0, 1.0) * new_capacity
            : new_capacity;
        commanded_thrust_n = 0.0;
        net_thrust_n = 0.0;
        core_thrust_n = 0.0;
        ab_thrust_n = 0.0;
        fuel_flow_kgs = 0.0;
        dry_power = 0.0;
        ab_power = 0.0;
        tvc_pitch_cmd = 0.0;
        tvc_pitch_actual_deg = 0.0;
        lift_fan_clutch = 0.0;
        stovl_mode = false;
    }

    /// @brief Maximum dry thrust at sea level static [N]: military power,
    /// raised toward the total lift rating as the LiftFan clutch engages.
    [[nodiscard]] double max_dry_rating_n() const noexcept {
        if (lift_fan_clutch <= 0.0) return config.static_dry_thrust_n;
        const double stovl_total = stovl_config.lift_fan_thrust_n + stovl_config.nozzle_thrust_n
                                 + stovl_config.roll_post_thrust_n;
        return config.static_dry_thrust_n + lift_fan_clutch * (stovl_total - config.static_dry_thrust_n);
    }

    /// @brief Lever position in the dry range that asks for @p thrust_n of
    /// steady net thrust at @p air (the STOVL control law's thrust demand).
    [[nodiscard]] double lever_for_thrust(double thrust_n, const environment::AirData& air) const noexcept {
        const double rating = thrust_n / (std::max)(1e-3, lapse_factor(air, false));
        const double span = max_dry_rating_n() - config.static_idle_thrust_n;
        const double dry = std::clamp((rating - config.static_idle_thrust_n) / span, 0.0, 1.0);
        return DETENT_IDLE + dry * (DETENT_MIL - DETENT_IDLE);
    }

    /**
     * @brief Computes installed thrust lapse with density ratio and Mach number.
     */
    [[nodiscard]] double lapse_factor(
        const environment::AirData& air,
        bool in_ab
    ) const noexcept {
        const double sigma = std::clamp(
            air.density / environment::Atmosphere1976::RHO0, 0.0, 1.5);

        // Density exponent
        const double exponent = in_ab ? 1.0 : 0.85;
        const double density_term = std::pow(sigma, exponent);

        // Ram recovery
        const double mach = std::clamp(air.mach_number, 0.0, 2.6);
        double k_ram = in_ab ? 0.36 : 0.16;

        // Enhanced ram recovery for supercruising aircraft (Typhoon, F-22)
        if (config.has_supercruise && !in_ab) {
            k_ram = 0.28;
        }
        // Higher bypass engines (A-10 TF34, BPR ~6) suffer greater momentum drag at
        // speed: installed thrust *falls* with Mach instead of gaining from ram
        // recovery — the standard high-bypass lapse 1 - 0.49 sqrt(M) (Mattingly),
        // ~63% of static at the A-10's M0.58 top speed.  A positive k_ram here let
        // the A-10 run well past its 381 kt level-flight maximum.
        if (!config.has_afterburner) {
            return density_term * (1.0 - 0.49 * std::sqrt(mach));
        }

        const double ram_term = 1.0 + k_ram * mach * mach;
        return density_term * ram_term;
    }

    /**
     * @brief Advances propulsion model by one timestep.
     *
     * @param throttle_position Lever in [0.0, 1.0].
     * @param air Air data at current altitude and speed.
     * @param dt Timestep in seconds.
     * @param ab_gate_open Physical afterburner detent gate.
     * @return Net thrust in Newtons.
     */
    double update(
        double throttle_position,
        const environment::AirData& air,
        double dt,
        bool ab_gate_open = true
    ) noexcept {
        const bool has_fuel = (fuel_kg > 0.0);
        const double u = has_fuel ? std::clamp(throttle_position, 0.0, 1.0) : 0.0;

        double static_rating = 0.0;
        bool in_ab = false;

        const double dry_max = max_dry_rating_n();
        if (stovl_mode) ab_gate_open = false;

        if (u < DETENT_CUTOFF) {
            detent_state = input::ThrottleController::DetentState::CUTOFF;
            dry_power = 0.0;
            ab_power = 0.0;
            static_rating = 0.0;
        } else if (u <= DETENT_IDLE) {
            detent_state = input::ThrottleController::DetentState::IDLE;
            dry_power = 0.0;
            ab_power = 0.0;
            static_rating = config.static_idle_thrust_n;
        } else if (!config.has_afterburner) {
            // Dry-only aircraft (e.g. A-10C): throttle maps [0.08, 1.00] linearly to max dry
            detent_state = input::ThrottleController::DetentState::MIL_POWER;
            const double span = 1.0 - DETENT_IDLE;
            dry_power = std::clamp((u - DETENT_IDLE) / span, 0.0, 1.0);
            ab_power = 0.0;
            static_rating = config.static_idle_thrust_n +
                            dry_power * (config.static_dry_thrust_n - config.static_idle_thrust_n);
        } else if (u <= DETENT_MIL || !ab_gate_open) {
            // Dry range on afterburning jet
            detent_state = input::ThrottleController::DetentState::MIL_POWER;
            const double mil_range = DETENT_MIL - DETENT_IDLE;
            dry_power = std::clamp((u - DETENT_IDLE) / mil_range, 0.0, 1.0);
            ab_power = 0.0;
            static_rating = config.static_idle_thrust_n +
                            dry_power * (dry_max - config.static_idle_thrust_n);
        } else {
            // Wet / Afterburner range (0.85 to 1.00)
            detent_state = input::ThrottleController::DetentState::AFTERBURNER;
            dry_power = 1.0;
            const double ab_range = 1.00 - DETENT_MIL;
            ab_power = std::clamp((u - DETENT_MIL) / ab_range, 0.0, 1.0);
            in_ab = true;
            static_rating = config.static_dry_thrust_n +
                            ab_power * (config.static_ab_thrust_n - config.static_dry_thrust_n);
        }

        // Apply installed atmospheric lapse
        commanded_thrust_n = static_rating * lapse_factor(air, in_ab);

        // Split the demand into core and afterburner. In reheat the core runs
        // at military power and the afterburner supplies the rest; the total
        // is the same steady thrust as before.
        const double core_target = (in_ab ? config.static_dry_thrust_n : static_rating) * lapse_factor(air, false);
        const double ab_target = in_ab ? (std::max)(0.0, commanded_thrust_n - core_target) : 0.0;

        // Spool dynamics
        if (dt > 0.0) {
            // Core: slow spool-up, quicker spool-down.
            const double core_tau = stovl_mode ? stovl_config.tau_spool_s
                                  : (core_target < core_thrust_n) ? config.tau_spool_down : config.tau_spool_up;
            core_thrust_n += (core_target - core_thrust_n) * (1.0 - std::exp(-dt / (std::max)(0.05, core_tau)));

            // Afterburner: lights only once the core is close to military
            // power. Slamming the lever from idle into full reheat used to
            // bring the whole thrust in on the 0.5 s light-off time constant,
            // skipping the several seconds a real core takes to spool up.
            const double mil_core = config.static_dry_thrust_n * lapse_factor(air, false);
            const bool ab_lit = in_ab && core_thrust_n >= AB_LIGHT_CORE_FRACTION * mil_core;
            const double ab_goal = ab_lit ? ab_target : 0.0;
            const double ab_tau = ab_lit ? config.tau_ab_light : AB_CANCEL_TAU_S;
            ab_thrust_n += (ab_goal - ab_thrust_n) * (1.0 - std::exp(-dt / (std::max)(0.05, ab_tau)));
            ab_thrust_n = (std::max)(0.0, ab_thrust_n);

            net_thrust_n = core_thrust_n + ab_thrust_n;

            // TVC nozzle actuator rate limit (60 deg/s)
            if (config.has_thrust_vectoring) {
                const double target_deg = std::clamp(tvc_pitch_cmd, -1.0, 1.0) * config.tvc_max_deflection_deg;
                const double max_rate = 60.0 * dt;
                const double delta = target_deg - tvc_pitch_actual_deg;
                tvc_pitch_actual_deg += std::clamp(delta, -max_rate, max_rate);
            }
        } else {
            core_thrust_n = core_target;
            ab_thrust_n = ab_target;
            net_thrust_n = commanded_thrust_n;
        }

        net_thrust_n = (std::max)(0.0, net_thrust_n);

        // Fuel burn calculation
        const double tsfc = in_ab ? config.tsfc_ab
                          : (lift_fan_clutch > 0.0)
                              ? config.tsfc_dry + lift_fan_clutch * (stovl_config.tsfc_stovl - config.tsfc_dry)
                              : config.tsfc_dry;
        fuel_flow_kgs = has_fuel ? (net_thrust_n * tsfc) : 0.0;
        if (dt > 0.0) {
            fuel_kg = (std::max)(0.0, fuel_kg - fuel_flow_kgs * dt);
        }

        return net_thrust_n;
    }

    /**
     * @brief Pitching moment generated by engine installation.
     * Includes A-10 high-mount nacelle pitch coupling and F-22 2D thrust vectoring.
     */
    [[nodiscard]] double pitch_moment() const noexcept {
        double my = 0.0;

        // 1. Vertical thrust offset arm (A-10 high nacelle is at Z = -1.2m above CG)
        // M = r x F -> My = z * Fx (where z < 0 produces negative/nose-down My)
        if (config.thrust_z_offset_m != 0.0) {
            my += net_thrust_n * config.thrust_z_offset_m;
        }

        // 2. 2D Thrust Vectoring (F-22 nozzle deflection)
        // Deflection angle delta (positive pitch up = nozzle pointing up = negative Z in body frame)
        if (config.has_thrust_vectoring && config.tvc_moment_arm_m > 0.0) {
            constexpr double DEG_TO_RAD = M_PI / 180.0;
            const double delta_rad = tvc_pitch_actual_deg * DEG_TO_RAD;
            // Upward thrust component produces pitch-up moment about CG
            const double fz_tvc = -net_thrust_n * std::sin(delta_rad);
            my += -fz_tvc * config.tvc_moment_arm_m;
        }

        return my;
    }

    [[nodiscard]] double fuel_fraction() const noexcept {
        const double cap = aircraft::AircraftConfig::get(aircraft_type).mass.internal_fuel_capacity_kg;
        return (cap > 0.0) ? std::clamp(fuel_kg / cap, 0.0, 1.0) : 0.0;
    }

    [[nodiscard]] bool is_flamed_out() const noexcept {
        return fuel_kg <= 0.0;
    }

    void reset(double fuel_load_kg = -1.0) noexcept {
        const double cap = aircraft::AircraftConfig::get(aircraft_type).mass.internal_fuel_capacity_kg;
        fuel_kg = (fuel_load_kg >= 0.0) ? std::clamp(fuel_load_kg, 0.0, cap) : cap;
        commanded_thrust_n = 0.0;
        net_thrust_n = 0.0;
        core_thrust_n = 0.0;
        ab_thrust_n = 0.0;
        fuel_flow_kgs = 0.0;
        dry_power = 0.0;
        ab_power = 0.0;
        tvc_pitch_cmd = 0.0;
        tvc_pitch_actual_deg = 0.0;
        lift_fan_clutch = 0.0;
        stovl_mode = false;
    }
};

} // namespace fastjet::propulsion
