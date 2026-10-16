#pragma once

#include "fastjet/aircraft/aircraft_config.hpp"
#include "fastjet/environment/ground_collision.hpp"
#include "fastjet/flcs/onboard_flight_computer.hpp"
#include "fastjet/sim/aircraft.hpp"
#include "fastjet/sim/bfm_pilot.hpp"
#include "fastjet/sim/radar.hpp"
#include <algorithm>
#include <cmath>

namespace fastjet::sim {

/**
 * @brief A defensive pilot working only from the RWR: the textbook answer to
 * a radar missile shot.
 *
 *  - Locked, nothing in the air: beam the radar (put him at 3 or 9 o'clock)
 *    and go down, so the lock sits in the clutter notch; chaff while beaming.
 *  - Missile inbound: beam the missile the same way and keep dropping chaff
 *    while it is inside seeker range. If it drops off the RWR (decoyed,
 *    or searching), keep beaming the launcher for a while: the missile is
 *    somewhere on that line, and a notched launcher can't datalink it back on.
 *  - Endgame: about a second and a half before impact, a maximum-G break
 *    across the missile's line of sight, so its lagging autopilot can't follow.
 *  - Nothing on the RWR: run cold at military power.
 *
 * A jet with a missile approach warner (MAWS) also gets the missile itself:
 * its line of sight and time to go from launch, whether the RWR hears it or
 * not. One the MAWS sees and the RWR doesn't (and hasn't) is treated as a
 * heat seeker: beamed the same way, but with flares instead of chaff, out of
 * reheat, and a flare salvo with the break.
 *
 * It uses the same FLCS stick laws as the other AI pilots and sees no truth
 * data about the threat: only what the RWR shows and what
 * Engagement::detect_missile_threat() tells it (the MAWS, and missiles it can see).
 */
class EvasionPilot {
public:
    double floor_agl_m{1200.0};   ///< Descends toward this height to fight in the clutter
    double chaff_interval_s{1.0};
    double break_time_to_go_s{1.6};
    double threat_memory_s{8.0};  ///< Keeps defending this long after the RWR goes quiet
    double cruise_mach{0.85};     ///< Speed held running cold (an intercept bandit runs faster)

    // Diagnostics
    enum class Mode : uint8_t { CRUISE, NOTCH_RADAR, NOTCH_MISSILE, LAST_DITCH };
    Mode mode{Mode::CRUISE};
    double heading_error{0.0};

    void reset() noexcept {
        trim_ = 0.0;
        chaff_timer_ = 0.0;
        prev_missile_range_ = -1.0;
        missile_memory_ = 0.0;
        beam_side_ = 0.0;
        flare_timer_ = 0.0;
        salvo_left_ = 0;
        salvo_fired_ = false;
        flare_out_ = false;
        initialised_ = false;
        mode = Mode::CRUISE;
    }

    [[nodiscard]] AircraftControls update(double dt, const Aircraft& self, const RwrStatus& rwr,
                                          const MissileThreat& threat = {}) noexcept {
        constexpr double G0 = fdm::SixDoFFDM::GRAVITY_ACCEL;
        AircraftControls out{};
        out.throttle = 0.84;
        if (!self.alive()) return out;
        const double psi = self.state.yaw();
        if (!initialised_) {
            initialised_ = true;
            cruise_heading_ = psi;
            cruise_alt_ = self.state.altitude();
        }
        chaff_timer_ -= dt;
        flare_timer_ -= dt;

        // Closure on the missile from the change in its RWR range.
        double t_go = 1e9;
        if (rwr.missile_valid) {
            if (prev_missile_range_ > 0.0) {
                const double closure = (prev_missile_range_ - rwr.missile_range_m) / dt;
                if (closure > 50.0) t_go = rwr.missile_range_m / closure;
            }
            prev_missile_range_ = rwr.missile_range_m;
        } else {
            prev_missile_range_ = -1.0;
        }
        // The MAWS times the missile itself (a boosting one too), and may have
        // one the RWR doesn't. Once a passive set has lost it at burnout, its
        // remembered time to go keeps him beaming but never times the break:
        // a radar missile coasts for a long time, and the countdown ran out
        // early, so he broke out of the notch before it got there.
        const bool maws = threat.inbound && threat.maws;
        if (maws && !threat.held) t_go = std::min(t_go, threat.time_to_go());
        // A held track is only its last line of sight, and that goes stale.
        // Once the RWR has had the missile, he steers by the RWR's memory and
        // the launcher's emitter instead: beaming the stale line of sight let
        // a chaff-broken missile reacquire him.
        const bool maws_steer = maws && (!threat.held || missile_memory_ <= 0.0);
        const bool maws_only = maws_steer && !rwr.missile_valid;
        const bool missile_known = rwr.missile_valid || maws_steer;
        const double missile_bearing =
            rwr.missile_valid ? rwr.missile_bearing : RwrStatus::relative_bearing(self.state, threat.pos);
        // Silent on the RWR now and lately: a heat seeker.
        const bool heat_seeker = maws_only && threat.heat_seeker && missile_memory_ <= 0.0;
        const bool flares_wanted = heat_seeker || (maws && threat.wants_flares());
        if (threat.launch) {
            // MAWS launch call: flares at once; the turn follows below. Chaff
            // waits for the beam inside 12 km (below): a program at launch
            // range decoyed nothing and cost the F-35B most EXPERT shots.
            out.dispense_flare = flares_wanted;
            flare_timer_ = chaff_interval_s;
        }

        const double agl = environment::GroundCollision::get_agl(self.state);
        const double terrain = self.state.altitude() - agl;
        double heading = cruise_heading_;
        double altitude = cruise_alt_;
        bool dispense = false;

        if (missile_known && t_go < break_time_to_go_s) {
            mode = Mode::LAST_DITCH;
            // Break toward the side the missile is on (or the beam side if it
            // is dead astern), so the turn crosses its line of sight.
            const double side = std::abs(std::sin(missile_bearing)) > 0.2
                                    ? (missile_bearing > 0.0 ? 1.0 : -1.0)
                                    : (beam_side_ != 0.0 ? -beam_side_ : 1.0);
            out.stick = break_turn(self, side);
            out.throttle = heat_seeker ? 0.8 : 1.0;
            if (!heat_seeker) {
                out.dispense = chaff_timer_ <= 0.0;
                if (out.dispense) chaff_timer_ = 0.3;
            }
            // A flare salvo with the break against anything that may see heat.
            if (flares_wanted && !salvo_fired_) {
                salvo_left_ = FLARE_SALVO_PROGRAMS;
                salvo_fired_ = true;
            }
            fire_salvo(out);
            return out;
        }
        if (!missile_known || t_go > break_time_to_go_s + 1.0) salvo_fired_ = false;
        // A missile that drops off the RWR may only be searching: keep beaming
        // its last bearing until it has had time to go by.
        if (rwr.missile_valid) {
            missile_memory_ = threat_memory_s;
            missile_azimuth_ = psi + rwr.missile_bearing;
        } else if (maws_only) {
            // Only the MAWS has it: beam where it is.
            missile_azimuth_ = psi + missile_bearing;
            missile_memory_ = std::max(0.0, missile_memory_ - dt);
        } else {
            missile_memory_ = std::max(0.0, missile_memory_ - dt);
        }
        if (missile_known || missile_memory_ > 0.0) {
            mode = Mode::NOTCH_MISSILE;
            // Off the RWR, the missile is somewhere between the launcher and
            // here: beam the launcher, which also denies it the datalink.
            const double threat_az = missile_known       ? missile_azimuth_
                                     : rwr.emitter_valid ? psi + rwr.emitter_bearing
                                                         : missile_azimuth_;
            heading = beam_heading(psi, threat_az, beam_side_);
            altitude = terrain + floor_agl_m;
            dispense = rwr.missile_valid && rwr.missile_range_m < 12000.0 && beaming(rwr.missile_bearing);
            if (flares_wanted && t_go < FLARE_TGO_S && flare_timer_ <= 0.0) {
                out.dispense_flare = true;
                flare_timer_ = chaff_interval_s;
            }
        } else if (rwr.level == RwrStatus::Level::LOCK && rwr.emitter_valid) {
            mode = Mode::NOTCH_RADAR;
            heading = beam_heading(psi, psi + rwr.emitter_bearing, beam_side_);
            altitude = terrain + floor_agl_m;
            dispense = beaming(rwr.emitter_bearing);
        } else {
            mode = Mode::CRUISE;
            beam_side_ = 0.0;
            if (rwr.emitter_valid) {
                // Painted but not locked: turn cold.
                cruise_heading_ = psi + rwr.emitter_bearing + M_PI;
            }
            heading = cruise_heading_;
            altitude = std::max(cruise_alt_, terrain + floor_agl_m);
        }
        cruise_alt_ = mode == Mode::CRUISE ? cruise_alt_ : std::max(altitude, self.state.altitude() - 50.0);
        if (dispense && chaff_timer_ <= 0.0) {
            out.dispense = true;
            chaff_timer_ = chaff_interval_s;
        }
        fire_salvo(out);

        // Heading and altitude hold: turn rate from the heading error, a level
        // (or descending) turn at the bank that gives it, G from the altitude error.
        const double V = std::max(80.0, self.state.airspeed());
        const auto cfg = aircraft::AircraftConfig::get(self.type);
        const double g_avail = std::min(6.5, cfg.flcs.max_g_positive - 1.0);
        const double err = wrap(heading - psi);
        heading_error = err;
        const double omega_max = std::sqrt(std::max(0.0, g_avail * g_avail - 1.0)) * G0 / V;
        const double omega = std::clamp(1.2 * err, -omega_max, omega_max);
        const double phi_des = std::atan2(omega * V, G0);
        const double phi = self.state.roll();
        out.stick.roll_stick = std::clamp(2.0 * wrap(phi_des - phi) - 0.3 * self.state.omega_b.x, -1.0, 1.0);

        const double climb = -self.state.velocity_ned().z;
        const double climb_cmd = std::clamp(0.08 * (altitude - self.state.altitude()), -60.0, 40.0);
        const double n_turn = 1.0 / std::max(0.2, std::cos(phi));
        const double n_cmd = std::clamp(n_turn + 0.04 * (climb_cmd - climb), 0.0, g_avail + 1.0);
        out.stick.pitch_stick = pitch_for(dt, self, cfg, n_cmd);

        // Hold high subsonic: a beaming jet needs turn performance in the
        // endgame, and turn rate peaks near corner speed, far below Mach 1.4.
        // Running cold he holds cruise_mach.
        const double mach = mode == Mode::CRUISE ? cruise_mach : 0.85;
        const double v_target = mach * environment::Atmosphere1976::compute(self.state.altitude()).speed_of_sound;
        out.throttle = V < v_target ? 1.0 : 0.7;
        // Against a heat seeker, out of reheat: the plume is what it sees.
        if (heat_seeker) out.throttle = std::min(out.throttle, 0.8);
        return out;
    }

private:
    static constexpr int FLARE_SALVO_PROGRAMS = 3; ///< Flare programs fired with the last-ditch break
    static constexpr double FLARE_TGO_S = 6.0;     ///< Flares while beaming a missile this close to impact

    double trim_{0.0};
    double chaff_timer_{0.0};
    double flare_timer_{0.0};
    int salvo_left_{0};
    bool salvo_fired_{false};
    bool flare_out_{false};
    double prev_missile_range_{-1.0};
    double missile_memory_{0.0};
    double missile_azimuth_{0.0};
    double beam_side_{0.0}; ///< +1: threat on the left wing, -1: on the right, 0: not chosen yet
    double cruise_heading_{0.0};
    double cruise_alt_{5000.0};
    bool initialised_{false};

    [[nodiscard]] static double wrap(double a) noexcept {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    /// @brief A beam heading (threat at 90 deg). The side nearer the current
    /// heading is chosen once and then held: with the threat dead astern the
    /// two beams are equally far and a pilot who kept re-choosing would never turn.
    [[nodiscard]] static double beam_heading(double psi, double threat_bearing, double& side) noexcept {
        const double h_left = threat_bearing + 0.5 * M_PI;  // threat on the left wing
        const double h_right = threat_bearing - 0.5 * M_PI; // threat on the right wing
        const double e_left = std::abs(wrap(h_left - psi));
        const double e_right = std::abs(wrap(h_right - psi));
        constexpr double SWITCH_MARGIN = 60.0 * M_PI / 180.0;
        if (side == 0.0) side = e_left < e_right ? 1.0 : -1.0;
        else if (side > 0.0 && e_right + SWITCH_MARGIN < e_left) side = -1.0;
        else if (side < 0.0 && e_left + SWITCH_MARGIN < e_right) side = 1.0;
        return side > 0.0 ? h_left : h_right;
    }

    /// @brief Fire what is left of the flare salvo, one program per press of
    /// the switch (released every other step), and remember the switch.
    void fire_salvo(AircraftControls& out) noexcept {
        if (salvo_left_ > 0) {
            out.dispense_flare = !flare_out_;
            if (out.dispense_flare) --salvo_left_;
        }
        flare_out_ = out.dispense_flare;
    }

    [[nodiscard]] static bool beaming(double relative_bearing) noexcept {
        return std::abs(std::abs(relative_bearing) - 0.5 * M_PI) < 20.0 * M_PI / 180.0;
    }

    /// @brief Maximum-G break turn toward @p side (+1 right): 75 deg of bank,
    /// so the pull goes across the missile's line of sight without heading for the ground.
    [[nodiscard]] static flcs::PilotCommands break_turn(const Aircraft& self, double side) noexcept {
        flcs::PilotCommands c{};
        const double phi_err = wrap(side * 75.0 * M_PI / 180.0 - self.state.roll());
        c.roll_stick = std::clamp(2.5 * phi_err - 0.2 * self.state.omega_b.x, -1.0, 1.0);
        c.pitch_stick = std::abs(phi_err) < 0.5 ? 1.0 : 0.4;
        return c;
    }

    [[nodiscard]] double pitch_for(double dt, const Aircraft& self, const aircraft::AircraftConfig& cfg,
                                   double n_cmd) noexcept {
        const bool fbw = cfg.flcs.law_type != aircraft::FLCSConfig::LawType::HYDRO_SAS_AUGMENTED;
        const double err = n_cmd - self.imu.Nz;
        trim_ = std::clamp(trim_ + (fbw ? 0.05 : 0.3) * err * dt, -0.4, 0.4);
        const double ff = fbw ? (n_cmd - 1.0) / std::max(1.0, cfg.flcs.max_g_positive - 1.0) : 0.05 * (n_cmd - 1.0);
        return std::clamp(ff + trim_, -0.5, 1.0);
    }
};

} // namespace fastjet::sim
