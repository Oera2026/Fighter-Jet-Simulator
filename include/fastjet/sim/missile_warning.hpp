#pragma once

#include "fastjet/aircraft/aircraft_type.hpp"
#include "fastjet/math/vector3.hpp"
#include "fastjet/sim/aircraft.hpp"
#include "fastjet/sim/missile.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace fastjet::sim {

/// @brief How a missile approach warning system (MAWS) finds a missile.
enum class MawsKind : uint8_t {
    NONE,          ///< Not fitted: the RWR is the only threat warning
    PASSIVE_UV,    ///< Solar-blind ultraviolet plume detector: sees a burning motor, short range
    PASSIVE_IR,    ///< Mid-wave infrared plume detector: sees a burning motor, longer range
    PULSE_DOPPLER, ///< Small active radar: sees the missile body by its doppler, burning or coasting
};

[[nodiscard]] constexpr const char* to_string(MawsKind k) noexcept {
    switch (k) {
        case MawsKind::NONE:          return "NONE";
        case MawsKind::PASSIVE_UV:    return "UV";
        case MawsKind::PASSIVE_IR:    return "IR";
        case MawsKind::PULSE_DOPPLER: return "PD";
    }
    return "?";
}

/**
 * @brief A jet's missile approach warner, against an air-to-air missile.
 *
 * The RWR only hears missiles that transmit (an active seeker, or the
 * launcher's uplink). A heat seeker is silent, so on its own the RWR never
 * warns of one. A MAWS looks for the missile itself: a passive set for the
 * hot exhaust plume of its motor, an active set for the missile body.
 *
 * Every figure below is an estimate from open sources: the real numbers are
 * classified, and they vary with altitude, weather, background and motor.
 */
struct MawsSpec {
    MawsKind kind{MawsKind::NONE};
    const char* name{""};          ///< Displays and debrief, e.g. "AAR-47"
    double detect_range_m{0.0};    ///< Detection range against an air-to-air missile
    double declare_s{0.0};         ///< Continuous track before a threat is declared (false-alarm filter)

    [[nodiscard]] constexpr bool fitted() const noexcept { return kind != MawsKind::NONE; }
    /// @brief A plume detector sees nothing once the motor burns out.
    [[nodiscard]] constexpr bool needs_motor() const noexcept {
        return kind == MawsKind::PASSIVE_UV || kind == MawsKind::PASSIVE_IR;
    }
    /// @brief Only the radar measures range. A passive set has angles, and its
    /// time to go comes from how fast the plume brightens.
    [[nodiscard]] constexpr bool measures_range() const noexcept { return kind == MawsKind::PULSE_DOPPLER; }
};

/**
 * @brief The MAWS each jet carries. All of them cover the full sphere.
 *
 * Plume ranges are for a fighter-launched missile's motor (brighter than a
 * shoulder-launched SAM's, which the UV sets were first sized for). UV in
 * the solar-blind band (240-280 nm) has almost no background to compete with,
 * so false alarms are rare, but the atmosphere's ozone and oxygen absorb it
 * strongly and the range stays short. Mid-wave IR carries much further but
 * sees far more clutter (sun glint, hot ground, flares), so it needs more
 * processing and two-colour discrimination.
 */
[[nodiscard]] constexpr MawsSpec maws_spec(aircraft::AircraftType type) noexcept {
    using T = aircraft::AircraftType;
    switch (type) {
        // USAF F-16Cs fly with the RWR (AN/ALR-56M or -69A) and no missile
        // warner. Some export jets carry AN/AAR-60 MILDS; not modelled.
        case T::F16_FIGHTING_FALCON: return {MawsKind::NONE, "", 0.0, 0.0};
        // EPAWSS (AN/ALQ-250) includes missile warning, but its sensors are not
        // public. Modelled as passive IR plume detection; 10 km is an
        // estimate between the UV sets and the F-22's IR system.
        case T::F15EX_EAGLE_II:      return {MawsKind::PASSIVE_IR, "EPAWSS", 10000.0, 0.4};
        // DASS missile approach warner: a pulse-doppler radar (antennas fore and
        // aft, all round). It sees the missile itself, motor burning or not,
        // and measures range and closing speed. About 8 km.
        case T::EUROFIGHTER_TYPHOON: return {MawsKind::PULSE_DOPPLER, "DASS MAW", 8000.0, 0.25};
        // AN/AAR-56 missile launch detector: six IR sensors around the airframe
        // (the forerunner of the F-35's DAS). 12 km is an estimate: further
        // than the UV sets, short of the F-35's newer cameras.
        case T::F22_RAPTOR:          return {MawsKind::PASSIVE_IR, "AAR-56", 12000.0, 0.4};
        // AN/AAR-47 UV missile warner, built against shoulder-launched SAMs.
        // 5 km is an estimate: a few km against a MANPADS motor, somewhat
        // more against an air-to-air motor, short of the IR sets.
        case T::A10_THUNDERBOLT:     return {MawsKind::PASSIVE_UV, "AAR-47", 5000.0, 0.5};
        // AN/AAQ-37 DAS: six mid-wave IR cameras covering the full sphere.
        // Trials tracked ballistic rocket launches at over 1,000 km; against a
        // fighter-launched missile's motor 30 km is an estimate.
        case T::F35B_LIGHTNING_II:   return {MawsKind::PASSIVE_IR, "DAS", 30000.0, 0.3};
    }
    return {};
}

/// @brief A missile closes slower than this: not approaching (sensor noise, a tail chase that's run out of energy).
inline constexpr double MAWS_MIN_CLOSING_MPS = 50.0;
/// @brief Threat cone: the missile's velocity relative to the jet points within
/// this angle of the jet. Wide enough for a missile still pulling lead on a
/// turning target, narrow enough to drop one crossing well off.
inline constexpr double MAWS_THREAT_CONE_RAD = 30.0 * M_PI / 180.0;
/// @brief Any closing missile predicted to pass this close is a threat, whatever the angle.
inline constexpr double MAWS_MISS_FLOOR_M = 300.0;

/// @brief One missile as a MAWS sees it from one jet.
struct MawsContact {
    bool detected{false};   ///< In range and visible to the sensor (motor burning, for a plume detector)
    bool threat{false};     ///< Detected, and closing on the jet
    double range_m{0.0};
    double closing_mps{0.0};
    double miss_m{0.0};     ///< Predicted miss distance if both fly straight on
    double time_to_go_s{0.0};
};

/**
 * @brief What @p spec makes of missile @p m from a jet at @p own_pos, @p own_vel.
 *
 * A threat is judged by closing geometry, not by whom the missile was fired
 * at (a sensor cannot know that): the missile is closing, and its velocity
 * relative to the jet points at the jet (inside the threat cone, or a
 * predicted pass within the miss floor). One flying away, or crossing well
 * off on its way to someone else, is not a threat.
 *
 * Time to go is range over closing speed. The radar measures both. A passive
 * set gets the same figure without range from how fast the plume brightens:
 * intensity goes as 1/R^2, so I / (dI/dt) = R / (2 Vc).
 */
[[nodiscard]] inline MawsContact maws_assess(const MawsSpec& spec, const math::Vector3& own_pos,
                                             const math::Vector3& own_vel, const Missile& m) noexcept {
    MawsContact c;
    if (!spec.fitted() || !m.active) return c;
    if (spec.needs_motor() && !m.body.motor_burning(m.spec)) return c;
    const math::Vector3 r = m.body.pos - own_pos; // jet to missile
    c.range_m = r.norm();
    if (c.range_m > spec.detect_range_m) return c;
    c.detected = true;
    if (c.range_m < 1.0) return c;

    const math::Vector3 v = m.body.vel - own_vel; // missile relative to the jet
    c.closing_mps = -r.dot(v) / c.range_m;
    if (c.closing_mps < MAWS_MIN_CLOSING_MPS) return c;
    const double vv = v.norm_squared();
    const double t_cpa = vv > 1e-9 ? -r.dot(v) / vv : 0.0;
    c.miss_m = (r + v * t_cpa).norm();
    c.time_to_go_s = c.range_m / c.closing_mps;
    c.threat = c.miss_m <= std::max(MAWS_MISS_FLOOR_M, c.range_m * std::sin(MAWS_THREAT_CONE_RAD));
    return c;
}

/**
 * @brief What a jet's MAWS tells the pilot (and the AI), refreshed every step.
 *
 * Angles are in the jet's body axes, like the RWR's: bearing 0 on the nose,
 * positive clockwise (right); elevation positive above the wing plane.
 */
struct MawsStatus {
    bool threat{false};        ///< At least one declared missile is closing on the jet
    bool launch{false};        ///< A new threat was declared this step (true for one step only)
    int threat_count{0};       ///< Declared missiles closing now
    double bearing_rad{0.0};   ///< Most urgent threat (shortest time to go)
    double elevation_rad{0.0};
    int clock{12};             ///< Its clock position, 1-12 (12 on the nose, 6 behind)
    math::Vector3 los_ned{};   ///< Unit line of sight to it (NED), for steering against it
    double time_to_go_s{0.0};  ///< Estimated time to impact (allows for a missile still accelerating)
    double closing_mps{0.0};
    bool range_valid{false};   ///< Range measured (pulse-doppler only; passive sets have none)
    double range_m{0.0};
    uint32_t launch_count{0};  ///< Threats declared so far: a consumer slower than the sim sees a rise

    /// @brief Clock position of a relative bearing: 12 on the nose, 3 right, 6 behind.
    [[nodiscard]] static int clock_of(double bearing_rad) noexcept {
        const double deg = std::fmod(bearing_rad * 180.0 / M_PI + 360.0, 360.0);
        const int h = static_cast<int>(std::lround(deg / 30.0)) % 12;
        return h == 0 ? 12 : h;
    }
};

/**
 * @brief One jet's missile approach warning system.
 *
 * Each step it assesses every missile in the sky (its own jet's excepted).
 * A missile must be tracked as a closing threat for the spec's declaration
 * time before it is reported, which keeps a single bad frame from crying
 * wolf. The moment it is declared sets the one-step launch flag and counts
 * up launch_count; it is declared once in its flight.
 *
 * Time to go allows for a missile still accelerating. Range over closing
 * speed alone reads tens of seconds for a missile just off the rail and
 * still boosting, when it may be four seconds from impact. Each track's
 * closing acceleration is filtered from its closing speed (a passive set
 * reads it off the brightening curve), and while it is positive the time
 * to go solves R = Vc t + a t^2 / 2. Once the motor burns out the missile
 * slows, and range over closing speed is the (conservative) estimate again.
 *
 * Reads the world, never writes it, and draws no random numbers, so it
 * cannot change how an engagement plays out. Fixed size, no heap.
 */
class MissileWarningSystem {
public:
    static constexpr int kMaxTracks = 32; ///< Missile slots it can follow (>= World::kMaxMissiles)
    static constexpr double ACCEL_TAU_S = 0.2; ///< Smoothing of each track's closing acceleration

    MawsSpec spec{maws_spec(aircraft::AircraftType::F16_FIGHTING_FALCON)};
    MawsStatus status{};

    /// @brief Fit the system @p type carries and forget every track.
    void configure(aircraft::AircraftType type) noexcept {
        type_ = type;
        spec = maws_spec(type);
        reset();
    }

    /// @brief Forget every track (launch_count keeps counting).
    void reset() noexcept {
        const uint32_t launches = status.launch_count;
        status = MawsStatus{};
        status.launch_count = launches;
        held_s_.fill(0.0);
        closing_.fill(0.0);
        accel_.fill(0.0);
        declared_ = 0;
    }

    /// @brief Time to go for a missile at @p range_m closing at @p closing_mps
    /// and gaining closing speed at @p accel_mps2 (only a gain shortens it).
    [[nodiscard]] static double time_to_go(double range_m, double closing_mps, double accel_mps2) noexcept {
        if (accel_mps2 <= 1.0) return range_m / std::max(closing_mps, 1.0);
        return (std::sqrt(closing_mps * closing_mps + 2.0 * accel_mps2 * range_m) - closing_mps) / accel_mps2;
    }

    /// @brief One step for jet @p self, flying @p own, against every slot in @p missiles.
    template <std::size_t N>
    void update(double dt, int self, const Aircraft& own, const std::array<Missile, N>& missiles) noexcept {
        static_assert(N <= static_cast<std::size_t>(kMaxTracks), "MAWS track table too small for the missile slots");
        if (own.type != type_) configure(own.type);
        const uint32_t launches = status.launch_count;
        status = MawsStatus{};
        status.launch_count = launches;
        if (!spec.fitted()) return;
        const bool working = own.alive();
        const math::Vector3 p = own.position();
        const math::Vector3 v = own.velocity();
        const double blend = 1.0 - std::exp(-dt / ACCEL_TAU_S);
        double best_ttg = 1e18;
        int best = -1;
        MawsContact best_c{};
        for (std::size_t k = 0; k < N; ++k) {
            const Missile& m = missiles[k];
            const uint32_t bit = 1u << k;
            if (!m.active) {
                // The slot is free: a missile launched into it later is a new one.
                held_s_[k] = 0.0;
                declared_ &= ~bit;
                continue;
            }
            const MawsContact c = (working && m.shooter != self) ? maws_assess(spec, p, v, m) : MawsContact{};
            if (!c.threat) {
                held_s_[k] = 0.0;
                continue;
            }
            // Closing acceleration, from a track seen last step too.
            if (held_s_[k] > 0.0) accel_[k] += ((c.closing_mps - closing_[k]) / dt - accel_[k]) * blend;
            else accel_[k] = 0.0;
            closing_[k] = c.closing_mps;
            held_s_[k] += dt;
            if (!(declared_ & bit)) {
                if (held_s_[k] + 1e-9 < spec.declare_s) continue;
                declared_ |= bit;
                status.launch = true;
                ++status.launch_count;
            }
            ++status.threat_count;
            const double ttg = time_to_go(c.range_m, c.closing_mps, accel_[k]);
            if (ttg < best_ttg) {
                best_ttg = ttg;
                best = static_cast<int>(k);
                best_c = c;
            }
        }
        if (best < 0) return;

        const math::Vector3 los = missiles[static_cast<std::size_t>(best)].body.pos - p;
        const math::Vector3 d = own.state.q_att.rotate_ned_to_body(los);
        status.threat = true;
        status.bearing_rad = std::atan2(d.y, d.x);
        status.elevation_rad = std::atan2(-d.z, std::hypot(d.x, d.y));
        status.clock = MawsStatus::clock_of(status.bearing_rad);
        status.los_ned = los / std::max(1.0, best_c.range_m);
        status.time_to_go_s = best_ttg;
        status.closing_mps = best_c.closing_mps;
        status.range_valid = spec.measures_range();
        status.range_m = status.range_valid ? best_c.range_m : 0.0;
    }

private:
    aircraft::AircraftType type_{aircraft::AircraftType::F16_FIGHTING_FALCON};
    std::array<double, kMaxTracks> held_s_{};  ///< Time each slot has been tracked closing
    std::array<double, kMaxTracks> closing_{}; ///< Each track's closing speed last step [m/s]
    std::array<double, kMaxTracks> accel_{};   ///< ...and its filtered rate of change [m/s^2]
    uint32_t declared_{0};                     ///< Slots whose missile has been declared
};

} // namespace fastjet::sim
