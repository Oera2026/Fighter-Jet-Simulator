#pragma once

#include "fastjet/sim/world.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace fastjet::sim {

/// @brief One search return ("brick"): where and when the beam painted a jet.
/// Angles are in the stabilised scan frame: azimuth from the ownship's heading
/// (positive right), elevation from the local horizon.
struct RadarHit {
    int target{-1};
    int frame{0};         ///< Scan frame that painted it
    double az_rad{0.0};
    double el_rad{0.0};
    double range_m{0.0};
    double alt_m{0.0};
};

/**
 * @brief The pilot's air-to-air radar, as an APG-68-class set in its two
 * master modes: combined radar mode (CRM: range-while-search and single-target
 * track) and air combat mode (ACM: auto-acquisition for the visual fight).
 *
 * RWS sweeps a roll- and pitch-stabilised beam through a 4-bar raster at
 * 65 deg/s. The pilot picks the azimuth width (A6 +/-60 deg, A3 +/-30, A1
 * +/-10, centred on the nose): the full A6 frame takes about 7.4 s, A1 about
 * 1.1 s. A jet is painted once per bar that the beam crosses it, and only if
 * the radar could see it then: inside detection range, inside the gimbal and
 * out of the doppler notch (the same SensorLimits test the missiles and the
 * AI's radar use). Each paint leaves a brick where the jet was; bricks age
 * out after a few frames, so a moving jet leaves a short trail of history.
 * What RWS cannot tell the pilot is heading, speed or height.
 *
 * Designating a brick commands the jet's FireControlRadar into STT on that
 * target, which is what the target's RWR hears as a lock. STT follows the
 * target continuously, coasts on memory through a notch and, unlike the AI's
 * radar, does not reacquire by itself when the lock breaks: the scope drops
 * back to search and the pilot must find the target again.
 *
 * RWS is a long-range tool: its 4 bars cover only about 10 deg of elevation
 * around the antenna setting, and a jet a mile away, above or below the nose
 * in a turning fight, is outside it. ACM replaces the search with fast
 * nose-referenced patterns that lock the first jet they find inside 10 nm,
 * with no designation: 20x30 (the HUD field of view), 10x60 (a vertical
 * slice up the lift vector, for a bandit above the canopy bow) and BORE (a
 * fixed beam down the nose). A lock broken in ACM goes back to the ACM scan.
 */
class RadarScope {
public:
    /// OFF, RWS (CRM search), STT (either master mode), ACM (ACM search).
    enum class Mode : uint8_t { OFF, RWS, STT, ACM };
    enum class AcmScan : uint8_t { HUD_20X30, VERTICAL_10X60, BORE };

    /// @brief A scan pattern: beam-centre limits, and bars swept either
    /// across in azimuth (rows, bar 0 at the top) or up in elevation (columns,
    /// bar 0 on the left). No bars: a fixed beam.
    struct Pattern {
        double az_lo, az_hi, el_lo, el_hi; ///< rad
        int bars;
        bool vertical;
        bool body;   ///< Referenced to the nose, not stabilised to the horizon
        double rate; ///< rad/s
    };

    static constexpr std::array<double, 5> kRangeScalesNm{5.0, 10.0, 20.0, 40.0, 80.0};
    static constexpr int DEFAULT_RANGE_INDEX = 2;
    static constexpr int ACM_RANGE_INDEX = 1; ///< ACM shows the 10 nm scale
    static constexpr double DEG = M_PI / 180.0;
    static constexpr double SCAN_RATE_RAD_S = 65.0 * DEG;
    static constexpr double AZ_LIMIT_RAD = 60.0 * DEG; ///< Widest RWS scan, and the STT azimuth limit
    static constexpr std::array<double, 3> kRwsAzLimitsDeg{60.0, 30.0, 10.0}; ///< A6, A3, A1
    static constexpr int BARS = 4;
    static constexpr double BAR_SPACING_RAD = 2.2 * DEG;
    static constexpr double BEAM_HALF_RAD = 1.75 * DEG; ///< Half the 3.5 deg pencil beam
    static constexpr double ELEV_LIMIT_RAD = 55.0 * DEG; ///< Scan centre, keeping the top bar inside 60 deg
    static constexpr double ELEV_SLEW_RAD_S = 8.0 * DEG;
    /// Search detection range against a fighter, as a multiple of the lock-on range.
    static constexpr double DETECTION_RANGE_FACTOR = 1.25;
    static constexpr int HISTORY_FRAMES = 3;
    static constexpr int kMaxHits = 32;
    static constexpr double ACQUIRE_S = 0.1;      ///< Designation to STT with the jet in the beam
    static constexpr double LOCK_TIMEOUT_S = 0.5; ///< A designation that finds nothing gives up
    static constexpr double ACM_RANGE_M = 10.0 * 1852.0;
    static constexpr double ACM_SCAN_RATE_RAD_S = 110.0 * DEG;

    // Scan state
    Mode mode{Mode::OFF};
    bool acm{false};             ///< Master mode: ACM, else CRM
    AcmScan acm_scan{AcmScan::HUD_20X30};
    int rws_az_index{0};         ///< Into kRwsAzLimitsDeg
    int range_index{DEFAULT_RANGE_INDEX};
    double ant_az{-AZ_LIMIT_RAD}; ///< Stabilised in CRM, from the nose in the ACM scans
    double ant_el{0.0};
    double el_centre{0.0};   ///< Pilot's antenna elevation (RWS scan centre)
    int bar{0};
    int sweep_dir{1};
    int frame{0};
    std::array<RadarHit, kMaxHits> hits{};
    int hit_count{0};
    bool lock_failed_cue{false}; ///< The last designation found nothing to lock

    [[nodiscard]] bool fitted() const noexcept { return spec_.fitted; }
    [[nodiscard]] double range_scale_m() const noexcept {
        return kRangeScalesNm[static_cast<size_t>(range_index)] * 1852.0;
    }
    [[nodiscard]] double detection_range_m() const noexcept { return spec_.track_range_m * DETECTION_RANGE_FACTOR; }
    [[nodiscard]] int own() const noexcept { return own_; }
    /// @brief Half-width of the RWS scan (beam centre), radians.
    [[nodiscard]] double rws_az_limit_rad() const noexcept {
        return kRwsAzLimitsDeg[static_cast<size_t>(rws_az_index)] * DEG;
    }

    /// @brief Heading of the scan frame: the ownship's nose, projected level.
    [[nodiscard]] static double scan_heading(const fdm::FlightState& s) noexcept {
        const math::Vector3 nose = s.q_att.rotate_body_to_ned(math::Vector3(1.0, 0.0, 0.0));
        return std::atan2(nose.y, nose.x);
    }

    /// @brief Azimuth (from the scan heading, +right) and elevation (from the
    /// horizon) of a point, in the stabilised frame.
    static void stabilised_angles(const fdm::FlightState& own, const math::Vector3& at, double& az,
                                  double& el) noexcept {
        const math::Vector3 r = at - own.pos_ned;
        az = wrap_pi(std::atan2(r.y, r.x) - scan_heading(own));
        el = std::atan2(-r.z, std::hypot(r.x, r.y));
    }

    [[nodiscard]] static double wrap_pi(double a) noexcept {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    /// @brief Elevation of RWS bar @p b (bar 0 is the top one).
    [[nodiscard]] double bar_elevation(int b) const noexcept {
        return el_centre + (0.5 * (BARS - 1) - b) * BAR_SPACING_RAD;
    }
    /// @brief Upper and lower edges of the RWS elevation block.
    [[nodiscard]] double coverage_top_rad() const noexcept { return bar_elevation(0) + BEAM_HALF_RAD; }
    [[nodiscard]] double coverage_bottom_rad() const noexcept { return bar_elevation(BARS - 1) - BEAM_HALF_RAD; }

    /// @brief The RWS raster at the current azimuth and elevation settings.
    [[nodiscard]] Pattern rws_pattern() const noexcept {
        const double a = rws_az_limit_rad();
        return {-a, a, bar_elevation(BARS - 1), bar_elevation(0), BARS, false, false, SCAN_RATE_RAD_S};
    }
    /// @brief An ACM pattern. The 20x30 and 10x60 figures are the edges of
    /// the volume the beam covers, so the beam centre stays half a beam inside.
    [[nodiscard]] static Pattern acm_pattern(AcmScan s) noexcept {
        const double h = BEAM_HALF_RAD;
        switch (s) {
            case AcmScan::HUD_20X30:
                return {-10.0 * DEG + h, 10.0 * DEG - h, -10.0 * DEG + h, 20.0 * DEG - h, 9, false, true,
                        ACM_SCAN_RATE_RAD_S};
            case AcmScan::VERTICAL_10X60:
                return {-5.0 * DEG + h, 5.0 * DEG - h, -10.0 * DEG + h, 50.0 * DEG - h, 3, true, true,
                        ACM_SCAN_RATE_RAD_S};
            case AcmScan::BORE: break;
        }
        return {0.0, 0.0, 0.0, 0.0, 0, false, true, 0.0};
    }
    /// @brief The pattern the antenna is flying now (meaningless in STT).
    [[nodiscard]] Pattern pattern() const noexcept { return acm ? acm_pattern(acm_scan) : rws_pattern(); }

    // ---------------------------------------------------------------------
    // Pilot controls
    // ---------------------------------------------------------------------

    void range_up() noexcept { range_index = std::min(range_index + 1, static_cast<int>(kRangeScalesNm.size()) - 1); }
    void range_down() noexcept { range_index = std::max(range_index - 1, 0); }

    /// @brief Antenna elevation knob, held: @p dir +1 up, -1 down.
    void slew_elevation(double dir, double dt) noexcept {
        el_centre = std::clamp(el_centre + dir * ELEV_SLEW_RAD_S * dt, -ELEV_LIMIT_RAD, ELEV_LIMIT_RAD);
    }

    /// @brief Switch between CRM and ACM. Any lock is dropped; ACM shows the
    /// 10 nm scale and CRM gets the pilot's range scale back.
    void toggle_master_mode(World& w) noexcept {
        if (!fitted()) return;
        w.radars[static_cast<size_t>(own_)].switch_off();
        pending_ = 0.0;
        lock_failed_cue = false;
        acm = !acm;
        if (acm) {
            crm_range_index_ = range_index;
            range_index = ACM_RANGE_INDEX;
        } else {
            range_index = crm_range_index_;
        }
        enter_search();
    }

    /// @brief Select a master mode (the dogfight switch): ACM if @p want_acm, else CRM.
    void set_master_mode(World& w, bool want_acm) noexcept {
        if (fitted() && acm != want_acm) toggle_master_mode(w);
    }

    /// @brief Next scan pattern for the master mode: A6 / A3 / A1 in CRM,
    /// 20x30 / 10x60 / BORE in ACM. A lock in progress is kept.
    void cycle_scan() noexcept {
        if (!fitted()) return;
        if (acm) {
            acm_scan = static_cast<AcmScan>((static_cast<int>(acm_scan) + 1) % 3);
        } else {
            rws_az_index = (rws_az_index + 1) % static_cast<int>(kRwsAzLimitsDeg.size());
        }
        if (mode == Mode::RWS || mode == Mode::ACM) restart_scan();
    }

    /// @brief The brick under the acquisition cursor. With no cursor slew, the
    /// cursor rests on the nearest of the newest bricks.
    [[nodiscard]] const RadarHit* cursor_hit() const noexcept {
        const RadarHit* best = nullptr;
        for (int i = 0; i < hit_count; ++i) {
            const RadarHit& h = hits[static_cast<size_t>(i)];
            if (h.range_m > range_scale_m()) continue;
            if (!best || h.frame > best->frame || (h.frame == best->frame && h.range_m < best->range_m)) best = &h;
        }
        return best;
    }

    /// @brief TMS forward: lock the brick under the cursor. If the jet that
    /// left it is still inside the radar's limits the radar locks it after
    /// ACQUIRE_S; otherwise the designation times out and the scope stays in RWS.
    bool designate(World& w) noexcept {
        if (!fitted() || mode != Mode::RWS || pending_ > 0.0) return false;
        const RadarHit* h = cursor_hit();
        if (!h) return false;
        acquire(w, h->target);
        return true;
    }

    /// @brief TMS aft: drop the lock and return to search.
    void undesignate(World& w) noexcept {
        if (!fitted()) return;
        w.radars[static_cast<size_t>(own_)].switch_off();
        pending_ = 0.0;
        if (mode == Mode::STT) enter_search();
    }

    /// @brief STT (or a lock in progress): the target the radar is on.
    [[nodiscard]] int locked_target(const World& w) const noexcept {
        if (mode != Mode::STT) return -1;
        return w.radars[static_cast<size_t>(own_)].target;
    }

    // ---------------------------------------------------------------------
    // Stepping
    // ---------------------------------------------------------------------

    /// @brief Advance the antenna and paint what the beam crosses. Call after
    /// each World::step. Resets itself when the world or the airframe changes.
    void step(double dt, World& w, int own_index) noexcept {
        sync(w, own_index);
        if (!fitted() || mode == Mode::OFF) return;
        const Aircraft& self = w.aircraft[static_cast<size_t>(own_)];
        FireControlRadar& fcr = w.radars[static_cast<size_t>(own_)];

        if (pending_ > 0.0) {
            if (fcr.locked()) {
                pending_ = 0.0;
                mode = Mode::STT;
                hit_count = 0;
            } else if ((pending_ -= dt) <= 0.0) {
                pending_ = 0.0;
                fcr.switch_off();
                // ACM keeps trying on its own; only a pilot's designation fails.
                lock_failed_cue = !acm;
            }
        }

        if (mode == Mode::STT) {
            if (!fcr.locked()) {
                // The lock broke (memory ran out, chaff, gimbal): back to search.
                fcr.switch_off();
                enter_search();
                return;
            }
            stabilised_angles(self.state, fcr.track_pos, ant_az, ant_el);
            ant_az = std::clamp(ant_az, -AZ_LIMIT_RAD, AZ_LIMIT_RAD);
            auto_range((fcr.track_pos - self.position()).norm());
            return;
        }

        advance(pattern(), dt);
        if (mode == Mode::ACM) {
            if (pending_ <= 0.0) acm_acquire(w, self);
        } else {
            paint(w, self);
        }
    }

    /// @brief Follow the world the ownship is in. A new fight, a reset or a
    /// different jet gives a cold scope; the pilot's master mode, range,
    /// scan and elevation settings are kept. step() calls it; call it
    /// directly before reading the scope or designating when no step has run yet.
    void sync(const World& w, int own_index) noexcept {
        const aircraft::AircraftType type = w.aircraft[static_cast<size_t>(own_index)].type;
        const bool restarted = &w != world_ || w.time < last_time_ || own_index != own_;
        if (!configured_ || restarted || type != type_) {
            world_ = &w;
            own_ = own_index;
            type_ = type;
            spec_ = radar_spec(type);
            configured_ = true;
            pending_ = 0.0;
            lock_failed_cue = false;
            if (spec_.fitted) {
                enter_search();
            } else {
                mode = Mode::OFF;
                restart_scan();
            }
        }
        last_time_ = w.time;
    }

private:
    RadarSpec spec_{};
    aircraft::AircraftType type_{aircraft::AircraftType::F16_FIGHTING_FALCON};
    bool configured_{false};
    const World* world_{nullptr};
    double last_time_{-1.0};
    int own_{0};
    double pending_{0.0};
    int crm_range_index_{DEFAULT_RANGE_INDEX};
    std::array<bool, World::kMaxAircraft> in_beam_{};
    uint32_t rng_{0x9E3779B9u};

    void acquire(World& w, int target) noexcept {
        FireControlRadar& fcr = w.radars[static_cast<size_t>(own_)];
        fcr.enable(target, ACQUIRE_S);
        fcr.auto_reacquire = false;
        pending_ = LOCK_TIMEOUT_S;
        lock_failed_cue = false;
    }

    /// @brief Search in the current master mode, from the start of its pattern.
    void enter_search() noexcept {
        mode = acm ? Mode::ACM : Mode::RWS;
        restart_scan();
    }

    void restart_scan() noexcept {
        hit_count = 0;
        frame = 0;
        bar = 0;
        sweep_dir = 1;
        const Pattern p = pattern();
        ant_az = p.az_lo;
        ant_el = p.vertical ? p.el_lo : p.el_hi;
        in_beam_.fill(false);
    }

    /// @brief Fly the antenna along @p p: sweep, then step a bar at each end.
    void advance(const Pattern& p, double dt) noexcept {
        if (p.bars == 0) {
            ant_az = p.az_lo;
            ant_el = p.el_lo;
            return;
        }
        double& sweep = p.vertical ? ant_el : ant_az;
        double& cross = p.vertical ? ant_az : ant_el;
        const double lo = p.vertical ? p.el_lo : p.az_lo;
        const double hi = p.vertical ? p.el_hi : p.az_hi;
        sweep += sweep_dir * p.rate * dt;
        if (sweep_dir > 0 ? sweep >= hi : sweep <= lo) {
            sweep = std::clamp(sweep, lo, hi);
            sweep_dir = -sweep_dir;
            if (++bar >= p.bars) {
                bar = 0;
                ++frame;
                age_out();
            }
            in_beam_.fill(false);
        }
        // Columns step left to right, rows top to bottom.
        const double t = p.bars > 1 ? static_cast<double>(bar) / (p.bars - 1) : 0.0;
        cross = p.vertical ? p.az_lo + t * (p.az_hi - p.az_lo) : p.el_hi - t * (p.el_hi - p.el_lo);
    }

    /// @brief Where the beam points, in NED.
    [[nodiscard]] math::Vector3 beam_ned(const Aircraft& self, bool body) const noexcept {
        if (body) {
            const math::Vector3 b(std::cos(ant_el) * std::cos(ant_az), std::cos(ant_el) * std::sin(ant_az),
                                  -std::sin(ant_el));
            return self.state.q_att.rotate_body_to_ned(b);
        }
        const double hdg = scan_heading(self.state);
        return {std::cos(ant_el) * std::cos(hdg + ant_az), std::cos(ant_el) * std::sin(hdg + ant_az),
                -std::sin(ant_el)};
    }

    [[nodiscard]] SensorLimits search_limits(double max_range_m) const noexcept {
        SensorLimits lim;
        lim.max_range_m = max_range_m;
        lim.gimbal_rad = spec_.gimbal_deg * DEG;
        lim.notch_mps = spec_.notch_mps;
        return lim;
    }

    [[nodiscard]] double uniform() noexcept {
        rng_ ^= rng_ << 13;
        rng_ ^= rng_ >> 17;
        rng_ ^= rng_ << 5;
        return static_cast<double>(rng_) / 4294967296.0;
    }

    /// @brief Single-look detection probability: certain inside 75% of the
    /// detection range, fading to nothing at its edge.
    [[nodiscard]] double p_detect(double range) const noexcept {
        const double rd = detection_range_m();
        return std::clamp((rd - range) / (0.25 * rd), 0.0, 1.0);
    }

    void paint(const World& w, const Aircraft& self) noexcept {
        const math::Vector3 beam = beam_ned(self, false);
        const math::Vector3 nose = self.state.q_att.rotate_body_to_ned(math::Vector3(1.0, 0.0, 0.0));
        const SensorLimits lim = search_limits(detection_range_m());
        for (int j = 0; j < w.count; ++j) {
            if (j == own_) continue;
            const Aircraft& t = w.aircraft[static_cast<size_t>(j)];
            const math::Vector3 los = t.position() - self.position();
            const bool inside = !t.crashed && AirCombatGeometry::angle_between(beam, los) < BEAM_HALF_RAD;
            const bool entered = inside && !in_beam_[static_cast<size_t>(j)];
            in_beam_[static_cast<size_t>(j)] = inside;
            if (!entered) continue;
            if (lim.check(self.position(), nose, t.position(), t.velocity()) != TrackCheck::OK) continue;
            if (uniform() >= p_detect(los.norm())) continue;
            RadarHit h;
            h.target = j;
            h.frame = frame;
            stabilised_angles(self.state, t.position(), h.az_rad, h.el_rad);
            h.range_m = los.norm();
            h.alt_m = t.state.altitude();
            add_hit(h);
        }
    }

    /// @brief ACM: lock the nearest jet in the beam inside 10 nm. The beam
    /// dwells on a jet for several steps as it crosses, and BORE sits on it,
    /// so a failed attempt is retried while the jet stays in the beam.
    void acm_acquire(World& w, const Aircraft& self) noexcept {
        const math::Vector3 beam = beam_ned(self, true);
        const math::Vector3 nose = self.state.q_att.rotate_body_to_ned(math::Vector3(1.0, 0.0, 0.0));
        const SensorLimits lim = search_limits(std::min(ACM_RANGE_M, detection_range_m()));
        int best = -1;
        double best_range = 0.0;
        for (int j = 0; j < w.count; ++j) {
            if (j == own_) continue;
            const Aircraft& t = w.aircraft[static_cast<size_t>(j)];
            if (t.crashed) continue;
            const math::Vector3 los = t.position() - self.position();
            if (AirCombatGeometry::angle_between(beam, los) >= BEAM_HALF_RAD) continue;
            if (lim.check(self.position(), nose, t.position(), t.velocity()) != TrackCheck::OK) continue;
            const double r = los.norm();
            if (best < 0 || r < best_range) {
                best = j;
                best_range = r;
            }
        }
        if (best >= 0) acquire(w, best);
    }

    /// @brief One brick per jet per frame: overlapping bars repaint it in place.
    void add_hit(const RadarHit& h) noexcept {
        for (int i = 0; i < hit_count; ++i) {
            RadarHit& o = hits[static_cast<size_t>(i)];
            if (o.target == h.target && o.frame == h.frame) {
                o = h;
                return;
            }
        }
        if (hit_count < kMaxHits) {
            hits[static_cast<size_t>(hit_count++)] = h;
            return;
        }
        // Full: the oldest brick goes.
        int oldest = 0;
        for (int i = 1; i < hit_count; ++i) {
            if (hits[static_cast<size_t>(i)].frame < hits[static_cast<size_t>(oldest)].frame) oldest = i;
        }
        hits[static_cast<size_t>(oldest)] = h;
    }

    void age_out() noexcept {
        int n = 0;
        for (int i = 0; i < hit_count; ++i) {
            if (frame - hits[static_cast<size_t>(i)].frame < HISTORY_FRAMES) {
                hits[static_cast<size_t>(n++)] = hits[static_cast<size_t>(i)];
            }
        }
        hit_count = n;
    }

    /// @brief STT keeps the target between 40% and 95% of the scale.
    void auto_range(double range_m) noexcept {
        if (range_m > 0.95 * range_scale_m()) range_up();
        else if (range_index > 0 && range_m < 0.40 * range_scale_m() &&
                 range_m < 0.85 * kRangeScalesNm[static_cast<size_t>(range_index - 1)] * 1852.0) {
            range_down();
        }
    }
};

} // namespace fastjet::sim
