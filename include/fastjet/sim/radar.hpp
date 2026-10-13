#pragma once

#include "fastjet/aircraft/aircraft_type.hpp"
#include "fastjet/sim/aircraft.hpp"
#include "fastjet/sim/missile.hpp"
#include <cstdint>

namespace fastjet::sim {

/// @brief Fire-control radar figures against a fighter-size target.
struct RadarSpec {
    bool fitted{true};
    const char* rwr_symbol{"16"};  ///< How the RWR labels this emitter
    double track_range_m{37040.0}; ///< Lock-on range against a fighter (20 nm)
    double gimbal_deg{60.0};
    double notch_mps{25.0};
    double memory_s{2.0};          ///< Coasting on an extrapolated track before the lock drops
    double chaff_seduction{0.6};   ///< Chance a bundle in the gates breaks the lock
};

[[nodiscard]] constexpr RadarSpec radar_spec(aircraft::AircraftType type) noexcept {
    using T = aircraft::AircraftType;
    switch (type) {
        case T::F16_FIGHTING_FALCON: return {true, "16", 37040.0, 60.0, 25.0, 2.0, 0.60};
        case T::F15EX_EAGLE_II:      return {true, "15", 46300.0, 60.0, 22.0, 2.0, 0.50};
        case T::EUROFIGHTER_TYPHOON: return {true, "EF", 42600.0, 60.0, 22.0, 2.0, 0.50};
        case T::F22_RAPTOR:          return {true, "22", 46300.0, 60.0, 20.0, 2.0, 0.45};
        // No air-to-air radar: the A-10 fights with its eyes and its gun.
        case T::A10_THUNDERBOLT:     return {false, "", 0.0, 0.0, 0.0, 0.0, 0.0};
        // AN/APG-81 AESA: fixed array, +-60 deg electronic scan.
        case T::F35B_LIGHTNING_II:   return {true, "35", 46300.0, 60.0, 20.0, 2.0, 0.45};
    }
    return {};
}

/**
 * @brief A fighter's fire-control radar hunting one target.
 *
 * SEARCH scans for the target. Once the target has been inside the radar's
 * limits for the operator's acquisition time, the radar locks it (single-target
 * track, STT). A lock survives short dropouts by coasting on the last track
 * (memory). A lock that stays notched, out of the gimbal, out of range or
 * decoyed by chaff for longer than the memory time breaks, and the radar
 * goes back to searching.
 *
 * The RWR on the target hears SEARCH as a paint and STT as a lock.
 */
class FireControlRadar {
public:
    enum class Mode : uint8_t { OFF, SEARCH, TRACK };

    RadarSpec spec{};
    Mode mode{Mode::OFF};
    int target{-1};
    double acquire_s{1.0};   ///< Operator's time to lock a target in the volume
    /// An AI operator goes straight back to hunting after a lost lock. A
    /// pilot-commanded lock (auto_reacquire false) switches the tracker off
    /// instead, so the pilot has to find the target and designate it again.
    bool auto_reacquire{true};
    // State
    bool coasting{false};    ///< In STT on memory, target not currently seen
    double coast_timer{0.0};
    double acq_timer{0.0};
    double lock_time{0.0};   ///< Continuous time in STT
    TrackCheck last_check{TrackCheck::RANGE};
    math::Vector3 track_pos{};
    math::Vector3 track_vel{};

    void configure(aircraft::AircraftType type) noexcept {
        spec = radar_spec(type);
        mode = Mode::OFF;
        target = -1;
    }

    /// @brief Switch on and hunt for @p target_index.
    void enable(int target_index, double acquire_time_s) noexcept {
        if (!spec.fitted) return;
        mode = Mode::SEARCH;
        target = target_index;
        acquire_s = acquire_time_s;
        auto_reacquire = true;
        acq_timer = 0.0;
        coasting = false;
        lock_time = 0.0;
    }

    /// @brief Stop tracking and stop transmitting at the target.
    void switch_off() noexcept {
        mode = Mode::OFF;
        coasting = false;
        lock_time = 0.0;
        acq_timer = 0.0;
    }

    [[nodiscard]] bool locked() const noexcept { return mode == Mode::TRACK; }
    /// @brief STT with the target actually seen this step (what a datalink can uplink).
    [[nodiscard]] bool track_fresh() const noexcept { return mode == Mode::TRACK && !coasting; }

    [[nodiscard]] SensorLimits limits() const noexcept {
        SensorLimits l;
        l.max_range_m = spec.track_range_m;
        l.gimbal_rad = spec.gimbal_deg * M_PI / 180.0;
        l.notch_mps = spec.notch_mps;
        return l;
    }

    /// @brief Break lock (chaff, or the operator gave up). The next lock needs
    /// a fresh acquisition, and @p penalty_s longer while the operator sorts it out.
    void break_lock(double penalty_s = 0.0) noexcept {
        if (mode == Mode::OFF) return;
        mode = auto_reacquire ? Mode::SEARCH : Mode::OFF;
        coasting = false;
        lock_time = 0.0;
        acq_timer = -penalty_s;
    }

    void update(double dt, const Aircraft& self, const Aircraft& tgt) noexcept {
        if (mode == Mode::OFF) return;
        if (!self.alive() || tgt.crashed) {
            if (mode == Mode::TRACK) break_lock();
            last_check = TrackCheck::RANGE;
            return;
        }
        const math::Vector3 nose = self.state.q_att.rotate_body_to_ned(math::Vector3(1.0, 0.0, 0.0));
        last_check = limits().check(self.position(), nose, tgt.position(), tgt.velocity());
        const bool seen = last_check == TrackCheck::OK;

        if (mode == Mode::SEARCH) {
            acq_timer = seen ? acq_timer + dt : std::min(acq_timer, 0.0);
            if (acq_timer >= acquire_s) {
                mode = Mode::TRACK;
                coasting = false;
                coast_timer = 0.0;
                lock_time = 0.0;
                track_pos = tgt.position();
                track_vel = tgt.velocity();
            }
            return;
        }

        // STT
        lock_time += dt;
        if (seen) {
            coasting = false;
            coast_timer = 0.0;
            track_pos = tgt.position();
            track_vel = tgt.velocity();
        } else {
            coasting = true;
            coast_timer += dt;
            track_pos += track_vel * dt;
            if (coast_timer > spec.memory_s) break_lock();
        }
    }

    /// @brief The target is inside the scan volume: the RWR hears the radar.
    [[nodiscard]] bool painting() const noexcept {
        return mode == Mode::SEARCH && (last_check == TrackCheck::OK || last_check == TrackCheck::NOTCH);
    }
};

/**
 * @brief What a radar warning receiver tells the pilot about the most
 * dangerous emitter.
 *
 * Bearings are relative azimuths in the receiving jet's body axes: 0 is the
 * nose and positive is clockwise (right), in radians.
 */
struct RwrStatus {
    enum class Level : uint8_t { CLEAR, SEARCH, LOCK, LAUNCH };

    Level level{Level::CLEAR};
    bool emitter_valid{false};
    double emitter_bearing{0.0};
    const char* emitter_symbol{""};
    bool missile_valid{false};    ///< A guided missile inbound on this jet
    double missile_bearing{0.0};
    double missile_range_m{0.0};
    bool missile_seeker_active{false};

    [[nodiscard]] static double relative_bearing(const fdm::FlightState& own, const math::Vector3& at) noexcept {
        const math::Vector3 d = own.q_att.rotate_ned_to_body(at - own.pos_ned);
        return std::atan2(d.y, d.x);
    }
};

[[nodiscard]] constexpr const char* to_string(RwrStatus::Level l) noexcept {
    switch (l) {
        case RwrStatus::Level::CLEAR:  return "CLEAR";
        case RwrStatus::Level::SEARCH: return "SEARCH";
        case RwrStatus::Level::LOCK:   return "LOCK";
        case RwrStatus::Level::LAUNCH: return "LAUNCH";
    }
    return "?";
}

} // namespace fastjet::sim
