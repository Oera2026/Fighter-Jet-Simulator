#pragma once

#include "fastjet/sim/missile.hpp"
#include "fastjet/sim/world.hpp"
#include <cstdint>

namespace fastjet::sim {

/// @brief When a pilot is willing to take a missile shot.
enum class LaunchDoctrine : uint8_t {
    NONE,      ///< Guns only
    MAX_RANGE, ///< As soon as the missile can reach a target that holds course, never from inside 3 km: easy to defeat by turning away
    NO_ESCAPE, ///< Only when the missile still catches a target that turns cold and runs
};

[[nodiscard]] constexpr const char* to_string(LaunchDoctrine d) noexcept {
    switch (d) {
        case LaunchDoctrine::NONE:      return "GUNS ONLY";
        case LaunchDoctrine::MAX_RANGE: return "RMAX SHOTS";
        case LaunchDoctrine::NO_ESCAPE: return "NO-ESCAPE SHOTS";
    }
    return "?";
}

/**
 * @brief The shoot decision of an AI pilot with radar missiles (the MRM store).
 *
 * The launch zone is a flyout of the missile's own dynamics and guidance
 * (predict_flyout) from the launcher's radar track, so a shot is taken only
 * where the missile can actually get there. Doctrine sets which target
 * behaviour the flyout assumes. It is the main difference between a pilot
 * who wastes missiles at max range and one who waits until you can't run.
 *
 * Launch constraints: a radar lock held long enough to confirm, the target
 * within the missile's launch ATA and outside minimum range, and one
 * missile at a time (or a two-missile salvo, shoot-shoot).
 */
class MissileShooter {
public:
    LaunchDoctrine doctrine{LaunchDoctrine::NONE};
    bool shoot_shoot{false};     ///< Ripple a second missile at the same target
    double confirm_lock_s{0.6};  ///< Lock held this long before pickling
    double eval_period_s{0.5};   ///< Launch-zone refresh rate

    // Diagnostics
    FlyoutResult last_flyout{};
    int launches{0};

    void reset() noexcept {
        eval_timer_ = 0.0;
        since_launch_ = 1e9;
        last_flyout = {};
        launches = 0;
    }

    /// @brief Decide and, if it's a shot, launch. Returns true on a launch.
    bool update(double dt, World& world, int self, int target) noexcept {
        eval_timer_ -= dt;
        since_launch_ += dt;
        if (doctrine == LaunchDoctrine::NONE) return false;
        const auto si = static_cast<size_t>(self);
        const Aircraft& a = world.aircraft[si];
        const Aircraft& t = world.aircraft[static_cast<size_t>(target)];
        const FireControlRadar& radar = world.radars[si];
        const MissileStore& store = world.stores[si].mrm;
        if (!a.alive() || !t.alive() || store.count <= 0) return false;
        if (!radar.track_fresh() || radar.target != target || radar.lock_time < confirm_lock_s) return false;

        const int in_flight = world.missiles_in_flight(self, target);
        if (in_flight > 0) {
            // Shoot-look-shoot: wait for the result, unless rippling a salvo.
            const bool salvo = shoot_shoot && in_flight == 1 && since_launch_ > SALVO_SPACING_S &&
                               since_launch_ < SALVO_WINDOW_S;
            if (!salvo) return false;
        }

        const MissileSpec& spec = store.spec;
        const math::Vector3 los = radar.track_pos - a.position();
        const double range = los.norm();
        const math::Vector3 nose = a.state.q_att.rotate_body_to_ned(math::Vector3(1.0, 0.0, 0.0));
        if (range < spec.min_launch_range_m) return false;
        // An Rmax shooter takes his shots from range; in a knife fight he
        // fights with the gun. (An AMRAAM fired from inside 3 km in a turning
        // fight is a no-escape shot, the next level's doctrine.)
        if (doctrine == LaunchDoctrine::MAX_RANGE && range < RMAX_MIN_SHOT_RANGE_M) return false;
        if (AirCombatGeometry::angle_between(nose, los) > spec.max_launch_ata_deg * M_PI / 180.0) return false;

        if (eval_timer_ > 0.0) return false;
        eval_timer_ = eval_period_s;
        const FlyoutTarget model =
            doctrine == LaunchDoctrine::NO_ESCAPE ? FlyoutTarget::TURN_COLD : FlyoutTarget::STRAIGHT;
        last_flyout = predict_flyout(spec, a.position(), a.velocity(), radar.track_pos, radar.track_vel, model);
        if (!last_flyout.hit) return false;
        if (world.launch_missile(self, target, WeaponSel::MRM) < 0) return false;
        since_launch_ = 0.0;
        eval_timer_ = 0.0;
        ++launches;
        return true;
    }

    /// Closest an Rmax-doctrine pilot will take a missile shot [m].
    static constexpr double RMAX_MIN_SHOT_RANGE_M = 3000.0;

private:
    static constexpr double SALVO_SPACING_S = 1.5;
    static constexpr double SALVO_WINDOW_S = 4.0;
    double eval_timer_{0.0};
    double since_launch_{1e9};
};

/**
 * @brief The heat-seeker shot decision of an AI pilot (the SRM store).
 *
 * A shot needs the jet's own seeker locked on the target (the growl turned to
 * a tone) for confirm_lock_s, the target no further off the nose than the
 * pilot's off-boresight limit (a novice trusts a lock only near the HUD; an
 * ace cues the seeker anywhere in its gimbal), outside minimum range and
 * inside the kinematic launch zone: a flyout of the missile itself against a
 * target that holds course. Shots are spaced, and only one of the pilot's
 * missiles is in the air at a time (shoot-look-shoot).
 *
 * The seeker only runs while the SRM store is selected, so the pilot keeps it
 * selected whenever he has heat seekers left (the gun fires on the trigger
 * whatever is selected).
 */
class HeatSeekerShooter {
public:
    double off_boresight_deg{30.0}; ///< Furthest off the nose the pilot will shoot
    double confirm_lock_s{0.5};     ///< Tone held this long before pickling
    double spacing_s{6.0};          ///< Least time between shots
    double eval_period_s{0.25};     ///< Launch-zone refresh rate

    // Diagnostics
    FlyoutResult last_flyout{};
    int launches{0};

    void reset() noexcept {
        eval_timer_ = 0.0;
        since_launch_ = 1e9;
        last_flyout = {};
        launches = 0;
    }

    /// @brief Decide and, if it's a shot, launch. Returns true on a launch.
    bool update(double dt, World& world, int self, int target) noexcept {
        eval_timer_ -= dt;
        since_launch_ += dt;
        const auto si = static_cast<size_t>(self);
        WeaponStores& stores = world.stores[si];
        if (stores.srm.count <= 0) return false;
        stores.select(WeaponSel::SRM);
        const Aircraft& a = world.aircraft[si];
        const Aircraft& t = world.aircraft[static_cast<size_t>(target)];
        const IrSeeker& seeker = world.seekers[si];
        if (!a.alive() || !t.alive() || !seeker.locked || seeker.target != target) return false;
        if (seeker.lock_time_s < confirm_lock_s || since_launch_ < spacing_s) return false;
        if (world.missiles_in_flight(self) > 0) return false;

        const MissileSpec& spec = stores.srm.spec;
        const math::Vector3 los = t.position() - a.position();
        const math::Vector3 nose = a.state.q_att.rotate_body_to_ned(math::Vector3(1.0, 0.0, 0.0));
        const double limit = std::min({off_boresight_deg, spec.seeker_gimbal_deg, spec.max_launch_ata_deg});
        if (los.norm() < spec.min_launch_range_m) return false;
        if (AirCombatGeometry::angle_between(nose, los) > limit * M_PI / 180.0) return false;

        if (eval_timer_ > 0.0) return false;
        eval_timer_ = eval_period_s;
        last_flyout = predict_flyout(spec, a.position(), a.velocity(), t.position(), t.velocity(),
                                     FlyoutTarget::STRAIGHT);
        if (!last_flyout.hit) return false;
        if (world.launch_missile(self, target, WeaponSel::SRM) < 0) return false;
        since_launch_ = 0.0;
        ++launches;
        return true;
    }

private:
    double eval_timer_{0.0};
    double since_launch_{1e9};
};

} // namespace fastjet::sim
