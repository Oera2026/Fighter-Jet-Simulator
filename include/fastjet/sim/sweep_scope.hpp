#pragma once

#include "fastjet/sim/radar_scope.hpp"
#include "fastjet/sim/world.hpp"
#include <array>
#include <cmath>

namespace fastjet::sim {

/**
 * @brief A 360 deg sweep display (PPI): a beam turns round the ownship and
 * each jet lights up as a blip when the beam crosses it, then fades until
 * the next pass paints it again.
 *
 * A fighter's own radar only nods across the nose (the FCR page); this is
 * the all-round picture the pilot gets from an AWACS over the datalink,
 * shown the way a search radar shows it. Blips hold where the jet was when
 * painted, in the ground frame, so between paints they drift across the
 * heading-up display as the ownship turns and moves, and a fast jet jumps
 * from its old blip to a new one each pass.
 */
class SweepScope {
public:
    static constexpr double PERIOD_S = 2.4;   ///< One turn of the beam
    static constexpr int kMaxBlips = World::kMaxAircraft;
    static constexpr double EXPIRE_TURNS = 1.6;
    static constexpr std::array<double, 6> SCALES_NM = {1.0, 2.0, 5.0, 10.0, 20.0, 40.0};

    struct Blip {
        int id = -1;              ///< World aircraft index
        math::Vector3 pos{};      ///< NED position when painted
        math::Vector3 vel{};
        double painted_s = 0.0;
    };

    double time_s = 0.0;
    double sweep_rad = 0.0;       ///< Beam bearing from the nose, clockwise, [0, 2 pi)
    int scale = 3;                ///< Index into SCALES_NM
    std::array<Blip, kMaxBlips> blips{};
    int blip_count = 0;

    void reset() noexcept { *this = SweepScope{}; }

    [[nodiscard]] double range_scale_m() const noexcept { return SCALES_NM[static_cast<size_t>(scale)] * 1852.0; }
    /// @brief Time since the blip was painted, in beam turns (0 fresh, 1 about to be repainted).
    [[nodiscard]] double age(const Blip& b) const noexcept { return (time_s - b.painted_s) / PERIOD_S; }

    /// @brief Bearing of @p p from the ownship's nose, clockwise, [0, 2 pi).
    [[nodiscard]] static double relative_bearing(const fdm::FlightState& own, const math::Vector3& p) noexcept {
        const math::Vector3 r = p - own.pos_ned;
        return wrap_2pi(std::atan2(r.y, r.x) - RadarScope::scan_heading(own));
    }

    /// @brief Turn the beam through @p dt and paint every jet it crosses.
    void update(const World& w, int own_idx, double dt) noexcept {
        time_s += dt;
        const Aircraft& own = w.aircraft[static_cast<size_t>(own_idx)];
        const double a0 = sweep_rad;
        const double swept = 2.0 * M_PI * dt / PERIOD_S;
        double nearest_m = 1e12;
        for (int i = 0; i < w.count; ++i) {
            if (i == own_idx) continue;
            const Aircraft& a = w.aircraft[static_cast<size_t>(i)];
            if (!a.alive()) continue;
            nearest_m = std::min(nearest_m, (a.position() - own.position()).norm());
            if (wrap_2pi(relative_bearing(own.state, a.position()) - a0) <= swept) paint(i, a);
        }
        // A blip not repainted (the jet went down) goes. Not at one turn:
        // turning with the beam stretches the time between paints.
        int n = 0;
        for (int i = 0; i < blip_count; ++i) {
            if (age(blips[static_cast<size_t>(i)]) <= EXPIRE_TURNS) blips[static_cast<size_t>(n++)] = blips[static_cast<size_t>(i)];
        }
        blip_count = n;
        sweep_rad = wrap_2pi(a0 + swept);
        // Range scale: the smallest that holds the nearest jet. Set outright
        // on the first turn; after that with hysteresis, so a jet near a
        // boundary does not flick it, and only as the beam passes the nose,
        // so a turn's blips share one scale.
        if (nearest_m < 1e12 && (!scaled_ || sweep_rad < a0)) {
            const double nm = nearest_m / 1852.0;
            if (!scaled_) {
                scale = 0;
                while (scale + 1 < static_cast<int>(SCALES_NM.size()) && nm > 0.85 * SCALES_NM[static_cast<size_t>(scale)]) ++scale;
                scaled_ = true;
            } else if (nm > 0.95 * SCALES_NM[static_cast<size_t>(scale)] && scale + 1 < static_cast<int>(SCALES_NM.size())) {
                ++scale;
            } else if (scale > 0 && nm < 0.7 * SCALES_NM[static_cast<size_t>(scale - 1)]) {
                --scale;
            }
        }
    }

    [[nodiscard]] static double wrap_2pi(double a) noexcept {
        a = std::fmod(a, 2.0 * M_PI);
        return a < 0.0 ? a + 2.0 * M_PI : a;
    }

private:
    bool scaled_ = false;

    void paint(int id, const Aircraft& a) noexcept {
        Blip* b = nullptr;
        for (int i = 0; i < blip_count; ++i) {
            if (blips[static_cast<size_t>(i)].id == id) b = &blips[static_cast<size_t>(i)];
        }
        if (!b) {
            if (blip_count >= kMaxBlips) return;
            b = &blips[static_cast<size_t>(blip_count++)];
        }
        b->id = id;
        b->pos = a.position();
        b->vel = a.velocity();
        b->painted_s = time_s;
    }
};

} // namespace fastjet::sim
